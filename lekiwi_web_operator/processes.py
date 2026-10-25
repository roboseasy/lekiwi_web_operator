"""기능(launch) 관리: 시작 · 종료 · 로그 · 외부 프로세스 찾기 (ROS 없는 로직).

명령줄은 그때그때 config.build_process_command 로 만들고, 터미널에서 켠 프로세스는
config.process_matches 로 알아본다 (같은 ROS_DOMAIN_ID 인 것만). dev/dev_server.py 가 둘을
바꿔 끼우므로 반드시 config 모듈을 거쳐 부른다. 노드 그래프는 robot.RobotService 가 1 Hz 로
set_node_names() 에 넣는다. psutil 로 프로세스를 훑는 일은 Pi CPU 를 아끼려고 웹이 켜지 않은
기능의 핵심 노드가 보일 때 · 시작 · 끄기 요청 때만 한다.
끄기는 터미널의 Ctrl+C 와 같게 프로세스 그룹에 SIGINT → SIGTERM → SIGKILL 순서로 보낸다.
"""

import collections
import logging
import os
import re
import shlex
import signal
import subprocess
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Final

import psutil

from lekiwi_web_operator import config
from lekiwi_web_operator.robot_api import CommandResult, error_result, ok_result

logger = logging.getLogger(__name__)

ACTIVE_STATES: Final[tuple[str, ...]] = ('starting', 'running', 'stopping', 'external')
SAVE_MAP: Final[str] = 'save_map'
PROCESS_LABELS: Final[dict[str, str]] = {
	'bringup': '로봇',
	'mapping': '지도 작성',
	'navigation': '자율 주행',
	SAVE_MAP: '지도 저장',
}
# save_map.sh 의 마지막 안내 줄: 'Nav2에서 사용할 지도: <경로>.yaml'
SAVED_MAP_PATTERN: Final[re.Pattern[str]] = re.compile(r'Nav2에서 사용할 지도: (.+\.yaml)\s*$')
LOG_FLUSH_PERIOD: Final[float] = 0.2       # [s] 로그 줄을 이만큼 묶어 보낸다
EXTERNAL_SCAN_PERIOD: Final[float] = 1.0   # [s] 외부 프로세스 찾기 주기
# [s] 필요해 보이지 않아도 이만큼마다 한 번은 훑는다: 노드가 죽고 launch 만 남은 것(Ctrl+Z 등)은
# 핵심 노드가 안 보여서 그대로면 '꺼짐'으로 보인다 (2026-10-19 lekiwi04 에서 실제로 있었다)
IDLE_EXTERNAL_SCAN_PERIOD: Final[float] = 10.0
EMPTY_LAUNCH_DETAIL: Final[str] = (
	'터미널에서 켠 launch 가 남아 있지만 노드가 보이지 않습니다 '
	'(Ctrl+Z 로 멈췄거나 노드가 죽었을 수 있습니다). 끄고 다시 켜세요'
)
SIGNAL_POLL_PERIOD: Final[float] = 0.1     # [s] 신호를 보낸 뒤 끝났는지 보는 간격
WEB_LOG_PREFIX: Final[str] = '[웹 오퍼레이터]'
SAVE_MAP_STARTED_MESSAGE: Final[str] = '지도 저장: 시작했습니다. 끝나면 알려 드립니다'


def make_event(level: str, message: str) -> dict:
	"""hub 로 보내는 이벤트 {'type': 'event', 'level', 'message', 'time'}."""
	return {'type': 'event', 'level': level, 'message': message, 'time': time.time()}


def parse_saved_map(line: str) -> str | None:
	"""save_map.sh 출력 한 줄에서 저장된 yaml 파일 이름을 꺼낸다. 그 줄이 아니면 None."""
	match = SAVED_MAP_PATTERN.search(line)
	return Path(match.group(1).strip()).name if match else None


@dataclass
class _Managed:
	"""웹에서 켠 프로세스 하나."""

	popen: subprocess.Popen
	options: dict
	started: float                   # clock()
	since: float                     # time.time(): 시작 시각, 끝나면 끝난 시각
	is_stop_requested: bool = False
	exit_code: int | None = None     # 살아 있으면 None
	last_line: str = ''


def _domain_id(environ: Mapping[str, str]) -> str:
	"""ROS_DOMAIN_ID (없거나 비었으면 ROS 기본값 '0')."""
	return environ.get('ROS_DOMAIN_ID', '').strip() or '0'


def _spawn(argv: list[str]) -> subprocess.Popen:
	"""새 세션(= 새 프로세스 그룹)으로 실행해 그룹 전체에 신호를 보낼 수 있게 한다."""
	env = {**os.environ, 'PYTHONUNBUFFERED': '1', 'RCUTILS_COLORIZED_OUTPUT': '0'}
	# systemd 의 C 로케일에서도 한글 로그가 깨져 읽기 스레드가 죽지 않도록 utf-8 · replace
	return subprocess.Popen(
		argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
		text=True, encoding='utf-8', errors='replace', bufsize=1,
		start_new_session=True, env=env,
	)


def _is_alive(groups: list[int], processes: list[psutil.Process]) -> bool:
	"""그룹 하나라도 구성원이 남았거나 프로세스 하나라도 살아 있으면 True."""
	for group in groups:
		try:
			os.killpg(group, 0)
			return True
		except ProcessLookupError:
			continue
		except PermissionError:   # 다른 사용자의 프로세스가 그 그룹에 남아 있다
			return True
	for process in processes:
		try:
			if process.is_running() and process.status() != psutil.STATUS_ZOMBIE:
				return True
		except psutil.NoSuchProcess:
			continue
	return False


def _send_signal(groups: list[int], processes: list[psutil.Process], number: int) -> None:
	"""그룹에는 killpg, 그룹 대표가 아닌 프로세스에는 그 프로세스에만 보낸다."""
	for group in groups:
		try:
			os.killpg(group, number)
		except ProcessLookupError:
			pass
		except PermissionError as error:
			logger.warning('프로세스 그룹 %d 에 신호를 보내지 못했습니다: %s', group, error)
	for process in processes:
		try:
			process.send_signal(number)
		except psutil.NoSuchProcess:
			pass
		except psutil.AccessDenied as error:
			logger.warning('프로세스 %d 에 신호를 보내지 못했습니다: %s', process.pid, error)


class ProcessManager:
	"""bringup · mapping · navigation · save_map 을 켜고 끄고, 터미널에서 켠 것도 찾는다.

	감시 스레드 하나가 LOG_FLUSH_PERIOD 마다 로그를 묶어 보내고 EXTERNAL_SCAN_PERIOD 마다
	외부 프로세스를 찾는다. 프로세스마다 로그 읽기 스레드가 하나씩 돈다.

	Args:
		emit: 이벤트 · 로그 dict 를 받을 함수 (여러 스레드에서 불린다).
		clock: 단조 시계 [s] (PROCESS_START_GRACE 판단). 시험에서는 가짜 시계를 넣는다.
	"""

	def __init__(
		self, emit: Callable[[dict], None], clock: Callable[[], float] = time.monotonic,
	) -> None:
		self._emit = emit
		self._clock = clock
		self._lock = threading.Lock()
		names = (*config.PROCESS_NAMES, SAVE_MAP)
		self._logs = {name: collections.deque(maxlen=config.PROCESS_LOG_LINES) for name in names}
		self._pending: dict[str, list[str]] = {name: [] for name in names}
		self._managed: dict[str, _Managed] = {}
		self._external: dict[str, list[psutil.Process]] = {
			name: [] for name in config.PROCESS_NAMES
		}
		self._external_stopping: set[str] = set()
		self._node_names: frozenset[str] = frozenset()
		self._save_state: dict = {'state': 'idle', 'detail': '', 'map': None, 'since': None}
		self._save_popen: subprocess.Popen | None = None
		self._stop_threads: list[threading.Thread] = []
		self._is_closed = False
		self._closed = threading.Event()
		self._monitor = threading.Thread(target=self._monitor_loop, name='processes', daemon=True)
		self._monitor.start()

	# ── 상태 ───────────────────────────────────────────────
	def set_node_names(self, node_names: frozenset[str]) -> None:
		"""ROS 그래프의 노드 이름 (핵심 노드로 running · external 을 판단한다)."""
		with self._lock:
			self._node_names = frozenset(node_names)

	def state(self, name: str) -> str:
		"""bringup · mapping · navigation 의 상태 (SPEC §6 processes.state)."""
		with self._lock:
			return self._state_locked(name)[0]

	def status(self) -> dict:
		"""status.processes (SPEC §6)."""
		with self._lock:
			result = {name: self._entry_locked(name) for name in config.PROCESS_NAMES}
			result[SAVE_MAP] = dict(self._save_state)
		return result

	def warnings(self) -> list[str]:
		"""같은 기능이 두 개 이상 떠 있으면 경고 문장."""
		result = []
		with self._lock:
			for name in config.PROCESS_NAMES:
				managed = self._managed.get(name)
				count = len(self._external[name])
				count += 1 if managed is not None and managed.exit_code is None else 0
				if count >= 2:
					result.append(
						f'같은 기능이 {count}개 떠 있습니다: {PROCESS_LABELS[name]} ({name}). '
						'하나만 남기고 끄세요'
					)
		return result

	def get_log(self, name: str, max_lines: int) -> list[str]:
		"""최근 로그 줄 (최대 PROCESS_LOG_LINES). 모르는 이름이면 빈 목록."""
		if name not in self._logs:
			return []
		count = max(0, min(int(max_lines), config.PROCESS_LOG_LINES))
		with self._lock:
			lines = list(self._logs[name])
		return lines[-count:] if count else []

	# ── 시작 · 끄기 ────────────────────────────────────────
	def check_start(self, name: str) -> CommandResult | None:
		"""시작할 수 없으면 그 이유 (conflict), 시작할 수 있으면 None (SPEC §8 의존).

		터미널에서 막 켠 launch 는 핵심 노드가 아직 안 보일 수 있어 먼저 프로세스를 훑는다.
		"""
		self._refresh_external_logged()
		with self._lock:
			return self._check_start_locked(name)

	def start(self, name: str, options: dict) -> CommandResult:
		"""config.build_process_command(name, options) 를 백그라운드로 실행한다.

		Args:
			name: 'bringup' · 'mapping' · 'navigation'.
			options: build_process_command 에 넘길 옵션 (status 의 options 로도 보인다).
		"""
		label = PROCESS_LABELS[name]
		with self._lock:
			if self._is_closed:
				return error_result('unavailable', '게이트웨이가 꺼지는 중입니다')
			blocked = self._check_start_locked(name)
			if blocked is not None:
				return blocked
			try:
				argv = config.build_process_command(name, options)
			except ValueError as error:
				return error_result('bad_request', f'{label}: 실행 명령을 만들 수 없습니다 ({error})')
			try:
				popen = self._launch_locked(name, argv)
			except OSError as error:
				failure = f'{label}: 실행하지 못했습니다 ({error})'
			else:
				failure = None
				managed = _Managed(popen, dict(options), self._clock(), time.time())
				self._managed[name] = managed
		if failure is not None:
			self._emit(make_event('error', failure))
			return error_result('failed', failure)
		threading.Thread(
			target=self._read_output, args=(name, managed), name=f'log-{name}', daemon=True,
		).start()
		self._emit(make_event('info', f'{label}: 시작했습니다'))
		return ok_result(f'{label}: 시작했습니다', pid=popen.pid)

	def stop(self, name: str) -> CommandResult:
		"""웹에서 켠 것이든 터미널에서 켠 것이든 끈다. 진행은 백그라운드 (상태 stopping)."""
		label = PROCESS_LABELS[name]
		self._refresh_external_logged()   # 핵심 노드가 안 보여 훑지 않았을 수 있다
		with self._lock:
			if name == 'bringup' and any(
				self._state_locked(other)[0] in ACTIVE_STATES for other in ('mapping', 'navigation')
			):
				return error_result('conflict', '로봇을 끄기 전에 지도 작성 · 자율 주행을 먼저 끄세요')
			# save_map.sh 는 SLAM 이 살아 있어야 한다 (/write_state). 끄면 저장 중인 지도를 잃는다
			if name == 'mapping' and self._save_state['state'] == 'running':
				return error_result('conflict', '지도를 저장하는 중입니다. 저장이 끝난 뒤 끄세요')
			managed = self._managed.get(name)
			if managed is not None and managed.exit_code is None:
				if managed.is_stop_requested:
					return ok_result(f'{label}: 끄는 중입니다')
				managed.is_stop_requested = True
				self._start_stop_thread_locked(name, [managed.popen.pid], [], False)
			elif self._external[name]:
				if name in self._external_stopping:
					return ok_result(f'{label}: 끄는 중입니다')
				self._external_stopping.add(name)
				groups, processes = self._external_targets(self._external[name])
				self._start_stop_thread_locked(name, groups, processes, True)
			elif self._has_key_nodes_locked(name):
				return error_result('conflict', f'{label}: 다른 컴퓨터에서 실행 중이라 끌 수 없습니다')
			else:
				return error_result('conflict', f'{label}: 켜져 있지 않습니다')
		self._emit(make_event('info', f'{label}: 끄는 중입니다'))
		return ok_result(f'{label}: 끄는 중입니다')

	def save_map(self) -> CommandResult:
		"""지도 작성 중에 save_map 을 백그라운드로 실행한다. 결과는 status · 이벤트."""
		with self._lock:
			if self._is_closed:
				return error_result('unavailable', '게이트웨이가 꺼지는 중입니다')
			if self._save_state['state'] == 'running':
				return error_result('conflict', '지도를 저장하는 중입니다')
			if self._state_locked('mapping')[0] not in ('running', 'external'):
				return error_result('conflict', '지도 작성 중일 때만 지도를 저장할 수 있습니다')
			try:
				argv = config.build_process_command(SAVE_MAP, {})
			except ValueError as error:
				return error_result('failed', f'지도 저장 명령을 만들 수 없습니다 ({error})')
			try:
				popen = self._launch_locked(SAVE_MAP, argv)
			except OSError as error:
				return error_result('failed', f'지도 저장을 실행하지 못했습니다 ({error})')
			self._save_popen = popen
			self._save_state = {'state': 'running', 'detail': '', 'map': None, 'since': time.time()}
		threading.Thread(
			target=self._run_save_map, args=(popen,), name=SAVE_MAP, daemon=True,
		).start()
		# 결과 문구를 이벤트와 같게 둔다: 화면은 같은 문장이면 토스트를 한 번만 띄운다
		self._emit(make_event('info', SAVE_MAP_STARTED_MESSAGE))
		return ok_result(SAVE_MAP_STARTED_MESSAGE)

	def shutdown(self) -> None:
		"""(막힘) 웹에서 켠 프로세스를 모두 끄고 감시를 멈춘다. 여러 번 불러도 된다.

		자율 주행 · 지도 작성 · 지도 저장을 먼저 끄고 기다린 뒤 로봇(bringup)을 끈다:
		Nav2 감독 도구가 베이스가 살아 있을 때 토크를 끌 수 있게.
		"""
		with self._lock:
			self._is_closed = True
		for names in (('navigation', 'mapping', SAVE_MAP), ('bringup',)):
			with self._lock:
				for name in names:
					managed = self._managed.get(name)
					if name == SAVE_MAP and self._save_popen is not None:
						self._start_stop_thread_locked(SAVE_MAP, [self._save_popen.pid], [], False)
					elif managed is not None and managed.exit_code is None:
						if not managed.is_stop_requested:
							managed.is_stop_requested = True
							self._start_stop_thread_locked(name, [managed.popen.pid], [], False)
				threads = list(self._stop_threads)
			for thread in threads:
				thread.join()
		self._closed.set()

	# ── 외부 프로세스 ──────────────────────────────────────
	def refresh_external(self) -> None:
		"""같은 사용자의 프로세스 중 config.process_matches 인 것을 찾는다.

		자기 자신 · 조상 · 웹에서 켠 프로세스는 빼고, ROS_DOMAIN_ID 가 우리와 다른 것도 뺀다
		(같은 PC 의 다른 로봇 · 개발 환경을 이 로봇의 기능으로 보거나 끄지 않게).
		"""
		me = psutil.Process()
		excluded = {me.pid, *(parent.pid for parent in me.parents())}
		uid = os.getuid()
		domain_id = _domain_id(os.environ)
		found: dict[str, list[psutil.Process]] = {name: [] for name in config.PROCESS_NAMES}
		for process in psutil.process_iter(['uids', 'cmdline']):
			uids = process.info['uids']
			if process.pid in excluded or uids is None or uids.real != uid:
				continue
			cmdline = process.info['cmdline'] or []
			names = [name for name in config.PROCESS_NAMES if config.process_matches(name, cmdline)]
			if not names:
				continue
			try:
				if _domain_id(process.environ()) != domain_id:
					continue
			except psutil.Error:   # 끝났거나 환경을 읽을 수 없다: 우리 것으로 보지 않는다
				continue
			for name in names:
				found[name].append(process)
		with self._lock:
			managed_pids = self._managed_pids_locked()
			# 잠금 안에서 다시 본다: 찾은 뒤에 끝난 웹 프로세스가 외부 프로세스로 남지 않게
			self._external = {
				name: [
					process for process in processes
					if process.pid not in managed_pids and process.is_running()
				]
				for name, processes in found.items()
			}

	def flush_logs(self) -> None:
		"""쌓인 로그 줄을 프로세스마다 하나의 {'type': 'log'} 로 보낸다."""
		with self._lock:
			batches = [(name, lines) for name, lines in self._pending.items() if lines]
			for name, _lines in batches:
				self._pending[name] = []
		for name, lines in batches:
			self._emit({'type': 'log', 'process': name, 'lines': lines})

	# ── 내부 ───────────────────────────────────────────────
	def _state_locked(self, name: str) -> tuple[str, str]:
		"""(state, detail) (잠금 안)."""
		managed = self._managed.get(name)
		if managed is not None and managed.exit_code is None:
			if managed.is_stop_requested:
				return 'stopping', ''
			if self._has_key_nodes_locked(name):
				return 'running', ''
			if self._clock() - managed.started < config.PROCESS_START_GRACE:
				return 'starting', ''
			nodes = ', '.join(config.PROCESS_KEY_NODES[name])
			return 'running', f'핵심 노드({nodes})가 아직 보이지 않습니다. 로그를 확인하세요'
		if self._external[name]:
			if name in self._external_stopping:
				return 'stopping', ''
			return 'external', '' if self._has_key_nodes_locked(name) else EMPTY_LAUNCH_DETAIL
		if self._has_key_nodes_locked(name):
			return 'external', '다른 컴퓨터에서 실행 중이라 끌 수 없습니다'
		if managed is not None and managed.exit_code != 0 and not managed.is_stop_requested:
			return 'failed', managed.last_line
		return 'stopped', ''

	def _entry_locked(self, name: str) -> dict:
		"""status.processes[name] (잠금 안)."""
		state, detail = self._state_locked(name)
		managed = self._managed.get(name)
		is_alive = managed is not None and managed.exit_code is None
		external = self._external[name]
		pid, since = None, None
		if is_alive:
			pid, since = managed.popen.pid, managed.since
		elif external:
			pid = external[0].pid
			try:
				since = external[0].create_time()
			except psutil.Error:
				since = None
		elif managed is not None:
			since = managed.since
		entry = {
			'state': state,
			'is_managed': is_alive,
			'pid': pid,
			'since': since,
			'exit_code': None if managed is None else managed.exit_code,
			'detail': detail,
			'options': dict(managed.options) if is_alive else {},
		}
		if name == 'navigation':
			map_path = managed.options.get('map_path') if is_alive else None
			entry['map'] = Path(map_path).name if map_path else None
		return entry

	def _check_start_locked(self, name: str) -> CommandResult | None:
		"""시작할 수 없는 이유 (잠금 안)."""
		state = self._state_locked(name)[0]
		if state == 'stopping':
			return error_result('conflict', f'{PROCESS_LABELS[name]}: 끄는 중입니다. 잠시 뒤 다시 켜세요')
		if state in ACTIVE_STATES:
			return error_result('conflict', f'{PROCESS_LABELS[name]}: 이미 켜져 있습니다')
		if name == 'bringup':
			return None
		if self._state_locked('bringup')[0] not in ('running', 'external'):
			return error_result('conflict', '먼저 로봇을 켜세요')
		other = 'navigation' if name == 'mapping' else 'mapping'
		if self._state_locked(other)[0] in ACTIVE_STATES:
			return error_result('conflict', f'먼저 {PROCESS_LABELS[other]}을 끄세요')
		return None

	def _has_key_nodes_locked(self, name: str) -> bool:
		return any(node in self._node_names for node in config.PROCESS_KEY_NODES[name])

	def _forget_key_nodes_locked(self, name: str) -> None:
		"""끝난 launch 의 핵심 노드를 지운다. 그래프는 1 Hz 로만 다시 읽어서, 그대로 두면
		그동안 '다른 컴퓨터에서 실행 중(external)' 으로 잘못 보인다. 정말 남아 있으면 다음 그래프에 다시 나온다."""
		self._node_names -= frozenset(config.PROCESS_KEY_NODES[name])

	def _managed_pids_locked(self) -> set[int]:
		pids = {
			managed.popen.pid for managed in self._managed.values() if managed.exit_code is None
		}
		if self._save_popen is not None:
			pids.add(self._save_popen.pid)
		return pids

	def _external_targets(
		self, processes: list[psutil.Process],
	) -> tuple[list[int], list[psutil.Process]]:
		"""그룹 대표면 그룹 전체(터미널 Ctrl+C 와 같다), 아니면 그 프로세스에만 보낸다.

		자기 자신 · 조상은 refresh_external 에서 이미 뺐다. 그룹 대표가 아닌데 그룹에 보내면
		같은 그룹의 상관없는 프로세스까지 끄게 되므로 그 프로세스에만 보낸다.
		"""
		groups, singles = [], []
		for process in processes:
			try:
				is_leader = os.getpgid(process.pid) == process.pid
			except ProcessLookupError:
				continue
			if is_leader:
				groups.append(process.pid)
			else:
				singles.append(process)
		return groups, singles

	def _start_stop_thread_locked(
		self, name: str, groups: list[int], processes: list[psutil.Process], is_external: bool,
	) -> None:
		"""끄기 스레드를 시작한다. shutdown 이 기다릴 수 있게 잠금 안에서 start 한다."""
		thread = threading.Thread(
			target=self._stop_targets, args=(name, groups, processes, is_external),
			name=f'stop-{name}', daemon=True,
		)
		self._stop_threads = [old for old in self._stop_threads if old.is_alive()]
		self._stop_threads.append(thread)
		thread.start()

	def _stop_targets(
		self, name: str, groups: list[int], processes: list[psutil.Process], is_external: bool,
	) -> None:
		"""(끄기 스레드) SIGINT → PROCESS_STOP_TIMEOUT → SIGTERM → PROCESS_TERM_TIMEOUT → SIGKILL.

		웹에서 켠 프로세스의 종료 이벤트는 로그 스레드가 종료 코드와 함께 보낸다.
		"""
		steps = (
			(signal.SIGINT, config.PROCESS_STOP_TIMEOUT[name]),
			(signal.SIGTERM, config.PROCESS_TERM_TIMEOUT),
			(signal.SIGKILL, config.PROCESS_TERM_TIMEOUT),
		)
		for number, timeout in steps:
			if not _is_alive(groups, processes):
				break
			self._append_log(name, f'{WEB_LOG_PREFIX} {number.name} 을 보냅니다')
			_send_signal(groups, processes, number)
			deadline = time.monotonic() + timeout
			while time.monotonic() < deadline and _is_alive(groups, processes):
				time.sleep(SIGNAL_POLL_PERIOD)
		is_gone = not _is_alive(groups, processes)
		# stopping 을 풀기 전에: 끝난 프로세스가 external 로 보이지 않게
		self._refresh_external_logged()
		with self._lock:
			self._external_stopping.discard(name)
			if is_external and is_gone:
				self._forget_key_nodes_locked(name)
		if not is_gone:
			self._emit(make_event('error', f'{PROCESS_LABELS[name]}: SIGKILL 뒤에도 남아 있습니다'))
		elif is_external:
			self._emit(make_event('info', f'{PROCESS_LABELS[name]}: 껐습니다'))

	def _read_output(self, name: str, managed: _Managed) -> None:
		"""(로그 스레드) 끝날 때까지 출력을 읽고, 끝나면 종료 코드로 stopped · failed 를 정한다."""
		code = self._pump(name, managed.popen)
		label = PROCESS_LABELS[name]
		with self._lock:
			last_line = self._logs[name][-1] if self._logs[name] else ''
			managed.exit_code = code
			managed.since = time.time()
			managed.last_line = last_line
			is_failed = code != 0 and not managed.is_stop_requested
			self._forget_key_nodes_locked(name)
			self._append_log_locked(name, f'{WEB_LOG_PREFIX} 종료 코드 {code}')
		if is_failed:
			self._emit(make_event('error', f'{label}: 실패 (종료 코드 {code}) {last_line}'))
		else:
			self._emit(make_event('info', f'{label}: 꺼졌습니다'))

	def _run_save_map(self, popen: subprocess.Popen) -> None:
		"""(지도 저장 스레드) 출력에서 저장된 지도 이름을 찾고 SAVE_MAP_TIMEOUT 을 지킨다."""
		is_timed_out = threading.Event()

		def stop_on_timeout() -> None:
			is_timed_out.set()
			self._stop_targets(SAVE_MAP, [popen.pid], [], False)

		timer = threading.Timer(config.SAVE_MAP_TIMEOUT, stop_on_timeout)
		timer.daemon = True
		timer.start()
		saved_maps: list[str] = []

		def find_saved_map(line: str) -> None:
			saved = parse_saved_map(line)
			if saved is not None:
				saved_maps.append(saved)

		code = self._pump(SAVE_MAP, popen, find_saved_map)
		timer.cancel()
		saved_map = saved_maps[-1] if saved_maps else None
		with self._lock:
			last_line = self._logs[SAVE_MAP][-1] if self._logs[SAVE_MAP] else ''
			if is_timed_out.is_set():
				state, detail = 'failed', f'시간이 {config.SAVE_MAP_TIMEOUT:.0f}초를 넘었습니다'
			elif code == 0 and saved_map is not None:
				state, detail = 'succeeded', ''
			elif code == 0:
				state, detail = 'failed', '저장한 지도 이름을 출력에서 찾지 못했습니다'
			else:
				state, detail = 'failed', f'종료 코드 {code}: {last_line}'
			self._save_state = {
				'state': state, 'detail': detail,
				'map': saved_map if state == 'succeeded' else None, 'since': time.time(),
			}
			self._save_popen = None
		if state == 'succeeded':
			self._emit(make_event('info', f'지도 저장: {saved_map}'))
		else:
			self._emit(make_event('error', f'지도 저장 실패: {detail}'))

	def _launch_locked(self, name: str, argv: list[str]) -> subprocess.Popen:
		"""실행 줄을 로그에 남기고 띄운다. 실패하면 그것도 남기고 OSError 를 다시 던진다 (잠금 안)."""
		self._append_log_locked(name, f'{WEB_LOG_PREFIX} 실행: {shlex.join(argv)}')
		try:
			return _spawn(argv)
		except OSError as error:
			self._append_log_locked(name, f'{WEB_LOG_PREFIX} 실행 실패: {error}')
			raise

	def _pump(
		self, name: str, popen: subprocess.Popen, on_line: Callable[[str], None] | None = None,
	) -> int:
		"""(읽기 스레드) 끝날 때까지 출력 줄을 로그에 넣고 (on_line 에도 넘기고) 종료 코드를 돌려준다."""
		try:
			with popen.stdout:   # 다 읽으면 파이프를 닫는다 (기능을 켜고 끌 때마다 남지 않게)
				for line in popen.stdout:
					line = line.rstrip('\n')
					self._append_log(name, line)
					if on_line is not None:
						on_line(line)
		except (OSError, ValueError):
			logger.exception('%s 출력을 읽지 못했습니다', name)
		return popen.wait()

	def _append_log(self, name: str, line: str) -> None:
		with self._lock:
			self._append_log_locked(name, line)

	def _append_log_locked(self, name: str, line: str) -> None:
		self._logs[name].append(line)
		self._pending[name].append(line)

	def _monitor_loop(self) -> None:
		"""(감시 스레드) 로그 묶어 보내기와 (필요할 때만) 외부 프로세스 찾기."""
		next_scan = 0.0
		next_idle_scan = 0.0
		while not self._closed.wait(LOG_FLUSH_PERIOD):
			try:
				self.flush_logs()
				now = time.monotonic()
				if now >= next_scan:
					next_scan = now + EXTERNAL_SCAN_PERIOD
					if self._needs_external_scan() or now >= next_idle_scan:
						next_idle_scan = now + IDLE_EXTERNAL_SCAN_PERIOD
						self.refresh_external()
			except Exception:   # 예: 파일 디스크립터가 모자라 /proc 을 못 연다 (OSError)
				logger.exception('프로세스 감시 중 오류가 났습니다. 계속합니다')
		self.flush_logs()

	def _needs_external_scan(self) -> bool:
		"""psutil 로 훑을지 (Pi CPU): 웹이 켜지 않은 기능의 핵심 노드가 보이거나, 찾아 둔 외부
		프로세스가 있어 끝났는지 봐야 할 때만."""
		with self._lock:
			for name in config.PROCESS_NAMES:
				managed = self._managed.get(name)
				is_managed = managed is not None and managed.exit_code is None
				if self._external[name] or (self._has_key_nodes_locked(name) and not is_managed):
					return True
			return False

	def _refresh_external_logged(self) -> None:
		"""refresh_external. 실패는 로그만 남긴다 (시작 · 끄기를 막지 않는다)."""
		try:
			self.refresh_external()
		except Exception:
			logger.exception('외부 프로세스를 찾지 못했습니다')
