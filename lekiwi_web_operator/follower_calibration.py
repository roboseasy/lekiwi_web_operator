"""팔로워 (르키위 팔) 보정 탭의 게이트웨이 쪽 (docs/calibration_plan.md): 파일 · 고르기 · 모터 값으로 만들기 ·
접은 자세 · 새로 보정 · 되돌리기 (보정 프로그램을 lerobot python 으로 따로 켠다).

- 보정 파일: 팔로워와 같은 폴더 (LEKIWI_CALIBRATION_DIR → 팔로워 프로파일 calibration_dir → 기본) 의 <id>.json
  (lerobot 형식). 모터 값은 드라이버가 낸 lekiwi/motor/arm_registers (팔 1~6, 'calibration' 묶음) 로 맞춰 본다
  (lerobot is_calibrated 와 같은 기준: homing_offset · range_min · range_max). 바퀴는 레지스터에 없어 보지 않는다.
- 팔로워 설정은 게이트웨이 전용 config.arm_profile_path() 를 통째로 쓴다 (임시 파일 + 이름 바꾸기). 처음 만들 때는
  지금 쓰는 값 (사람이 쓴 robot_profile.yaml · 팔로워가 알린 robot_id) 에서 시작한다. robot_profile 은 읽기만 한다.
- 새로 보정은 로봇 기능을 끄고 (버스 주인이 없게) 보정 프로그램과 표준 입출력 JSON 줄로 말한다. 보정 프로그램이
  꺼진 것을 본 뒤에만 꺼 둔 로봇 기능을 다시 켠다. 제로-포지션 위치 시키기 · 끝내기 · 취소에 FOLLOWER_WORKER_STEP_TIMEOUT 안에
  답이 없으면 죽인다. 새 보정을 마치면 접은 자세를 지운다 (가운데가 바뀌어 예전 tick 은 다른 자세다).
  제로-포지션 위치 시키기로 쓴 값은 보정 프로그램이 되돌린다 (취소 · 오류 · 권한 끝 · 비상 정지). 되돌리지 못하고 끝났으면
  (전원 · 죽음) 예전 값이 config.follower_restore_path() 에 남는다: 다음 새로 보정이 먼저 되돌리거나 '되돌리기'.
- 막는 것: 조작 권한, 이름 [A-Za-z0-9_-]{1,40}, 보정은 한 번에 하나, 팔 세션 · 팔 토크가 켜져 있을 수 있음 ·
  자율 주행 · 지도 작성 · 자동 주행 · 로봇 끄는 중에는 고르기 · 접은 자세 · 새로 보정 · 되돌리기를 하지 않는다.
  막는 것을 보고 바쁨을 거는 것은 _command_lock 안에서 한 번에 한다.

잠금: _command_lock → robot._lock → _lock. _lock 을 잡은 채로 robot 을 부르지 않는다 (robot 은 자기 잠금 안에서
is_wanting · abort 를 부른다).
"""

import json
import logging
import os
import re
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any, Final

import yaml

from lekiwi_web_operator import config
from lekiwi_web_operator.processes import ACTIVE_STATES
from lekiwi_web_operator.robot_api import CommandResult, error_result, ok_result

if TYPE_CHECKING:
	from lekiwi_web_operator.robot import RobotService

logger = logging.getLogger(__name__)

FOLLOWER_NODE: Final[str] = config.FOLLOWER_ARM_NODE
ARM_MOTORS: Final[tuple[str, ...]] = config.ARM_JOINTS   # 모터 ID 1~6 (lerobot LeKiwi 이름)
WHEEL_MOTORS: Final[tuple[str, ...]] = ('base_left_wheel', 'base_back_wheel', 'base_right_wheel')
# lerobot LeKiwi.calibrate: 바퀴는 homing 0 · drive_mode 0 · 0~4095
WHEEL_CALIBRATION: Final[dict[str, int]] = {'drive_mode': 0, 'homing_offset': 0, 'range_min': 0, 'range_max': 4095}
REGISTERS_WAIT: Final[float] = 2.0        # [s] 모터 값으로 만들기 · 고르기가 레지스터를 기다리는 시간
SESSION_STATES: Final[tuple[str, ...]] = (
	'idle', 'starting', 'waiting_middle', 'recording', 'saving', 'done', 'error', 'cancelled')
ACTIVE_SESSION_STATES: Final[frozenset[str]] = frozenset({'starting', 'waiting_middle', 'recording', 'saving'})
CANCELABLE_STATES: Final[tuple[str, ...]] = ('starting', 'waiting_middle', 'recording')
END_STATES: Final[frozenset[str]] = frozenset({'done', 'cancelled', 'error'})
STEPS: Final[tuple[str, ...]] = ('start', 'middle', 'finish', 'cancel')   # 보정 프로그램 명령 (ack)
STEP_LABELS: Final[dict[str, str]] = {'middle': '제로-포지션 위치 시키기', 'finish': '끝내기', 'cancel': '취소'}
BRINGUP_ON_STATES: Final[tuple[str, ...]] = ('starting', 'running', 'external')
IDLE_SESSION: Final[dict] = {'state': 'idle', 'robot_id': None, 'joints': None, 'reason': None}
MAX_JOINTS: Final[int] = 16
MAX_FILES: Final[int] = 32   # status 에 넣는 보정 파일 수 (lerobot_calibration.MAX_CALIBRATION_IDS 와 같다)
RESTORE_HINT: Final[str] = '모터 보정이 반쯤 쓰였을 수 있습니다. ‘복원’을 누르세요'
REST_CLEARED_MESSAGE: Final[str] = (
	'가운데가 바뀌어 접은 자세를 지웠습니다. 팔을 접은 자세로 두고 ‘접은 자세 저장’을 다시 누르세요')


def is_valid_id(value: Any) -> bool:
	"""보정 이름 (파일 이름으로 써도 안전한) 인지."""
	return isinstance(value, str) and re.fullmatch(config.CALIBRATION_ID_PATTERN, value) is not None


def is_rest_pose(value: Any) -> bool:
	"""팔로워가 받는 접은 자세: 모터 ID 1~6 의 tick 6개 (0~4095 정수)."""
	return isinstance(value, list) and len(value) == len(ARM_MOTORS) and all(
		isinstance(tick, int) and not isinstance(tick, bool) and 0 <= tick <= 4095 for tick in value)


def registers_match(calibration: dict, registers: dict) -> bool:
	"""보정 파일 (lerobot 형식) 의 팔 1~6 이 드라이버가 읽은 레지스터와 같은지 (homing · min · max)."""
	by_id = {motor_id: index for index, motor_id in enumerate(registers['ids'])}
	for name in ARM_MOTORS:
		motor = calibration.get(name)
		if not isinstance(motor, dict) or motor.get('id') not in by_id:
			return False
		index = by_id[motor['id']]
		if not registers['valid'][index]:
			return False
		values = (motor.get('homing_offset'), motor.get('range_min'), motor.get('range_max'))
		if values != (registers['homing_offset'][index], registers['min'][index], registers['max'][index]):
			return False
	return True


def calibration_from_registers(registers: dict) -> dict | None:
	"""레지스터 (팔 1~6) + 바퀴 lerobot 기본값 → lerobot LeKiwi 보정 파일 내용. 팔 하나라도 못 읽었으면 None."""
	by_id = {motor_id: index for index, motor_id in enumerate(registers['ids'])}
	data = {}
	for motor_id, name in enumerate(ARM_MOTORS, start=1):
		index = by_id.get(motor_id)
		if index is None or not registers['valid'][index]:
			return None
		data[name] = {
			'id': motor_id, 'drive_mode': 0, 'homing_offset': registers['homing_offset'][index],
			'range_min': registers['min'][index], 'range_max': registers['max'][index],
		}
	for motor_id, name in enumerate(WHEEL_MOTORS, start=len(ARM_MOTORS) + 1):
		data[name] = {'id': motor_id, **WHEEL_CALIBRATION}
	return data


def write_atomic(path: Path, text: str) -> None:
	"""같은 폴더의 겹치지 않는 임시 파일에 다 쓰고 fsync 한 뒤 이름을 바꾼다 (쓰다 꺼져도 반쯤 쓴 파일이 없게)."""
	path.parent.mkdir(parents=True, exist_ok=True)
	descriptor, temporary = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.', suffix='.tmp')
	try:
		with os.fdopen(descriptor, 'w', encoding='utf-8') as file:
			os.fchmod(file.fileno(), 0o644)   # mkstemp 는 0600 으로 만든다
			file.write(text)
			file.flush()
			os.fsync(file.fileno())
		os.replace(temporary, path)
	except BaseException:
		Path(temporary).unlink(missing_ok=True)
		raise


def read_follower_parameters(path: Path) -> dict:
	"""프로파일 YAML 의 lekiwi_follower_arm_node.ros__parameters (없거나 못 읽으면 {})."""
	try:
		data = yaml.safe_load(path.read_text(encoding='utf-8'))
	except (OSError, yaml.YAMLError):
		return {}
	section = data.get(FOLLOWER_NODE) if isinstance(data, dict) else None
	parameters = section.get('ros__parameters') if isinstance(section, dict) else None
	return parameters if isinstance(parameters, dict) else {}


def read_motor_port(path: Path) -> str:
	"""장비 프로파일의 모터 드라이버 port (없으면 config.DEFAULT_MOTOR_PORT)."""
	try:
		data = yaml.safe_load(path.read_text(encoding='utf-8'))
	except (OSError, yaml.YAMLError):
		return config.DEFAULT_MOTOR_PORT
	section = data.get('lekiwi_motor_driver_node') if isinstance(data, dict) else None
	parameters = section.get('ros__parameters') if isinstance(section, dict) else None
	port = parameters.get('port') if isinstance(parameters, dict) else None
	return port if isinstance(port, str) and port else config.DEFAULT_MOTOR_PORT


def arm_profile_text(robot_id: str, rest_pose_ticks: list[int] | None) -> str:
	"""arm_profile.yaml 내용. 두 값을 늘 적는다 (robot_profile 의 값이 남지 않게).

	robot_id 는 늘 작은따옴표 글 (rcl 은 따옴표 없는 04 · y · 1e3 을 글로 읽지 않아 팔로워가 끝난다).
	접은 자세가 없으면 '' : rcl 은 빈 목록 [] 을 버려 robot_profile 의 예전 tick 이 남는다.
	shortcut: '' 는 팔로워 (follower_logic) 가 글도 빈 목록처럼 '없음' 으로 받아서 된다, lekiwi_lerobot 이 빈 값을 따로 받게 되면 맞춘다.
	"""
	if not is_valid_id(robot_id):
		raise ValueError(f'보정 이름이 잘못됐습니다: {robot_id!r}')
	if rest_pose_ticks is not None and not is_rest_pose(rest_pose_ticks):
		raise ValueError(f'접은 자세는 0~4095 tick 6개여야 합니다: {rest_pose_ticks!r}')
	rest = "''" if rest_pose_ticks is None else '[' + ', '.join(str(tick) for tick in rest_pose_ticks) + ']'
	return (
		'# 웹 게이트웨이 보정 탭이 통째로 쓰는 파일 (손으로 고치지 않는다). bringup 이 robot_profile 뒤에 읽는다\n'
		f'{FOLLOWER_NODE}:\n'
		'  ros__parameters:\n'
		f"    robot_id: '{robot_id}'\n"
		f'    rest_pose_ticks: {rest}\n')


class FollowerCalibration:
	"""팔로워 보정 탭 (RobotService 가 하나 만든다).

	Args:
		robot: 로봇 서비스 (권한 · 팔 상태 · 프로세스 · ROS 노드 · 알림).
	"""

	def __init__(self, robot: 'RobotService') -> None:
		self._robot = robot
		self._command_lock = threading.Lock()   # 막는 것 보기 + 바쁨 걸기 (한 번에)
		self._lock = threading.Lock()
		self._session = dict(IDLE_SESSION)
		self._worker: subprocess.Popen | None = None
		self._worker_lock = threading.Lock()   # 표준 입력에 한 줄씩
		self._wanting = 0                      # 레지스터 · 모터 상태를 바로 원하는 동작 수
		self._is_busy = False                  # 고르기 · 접은 자세 · 새로 보정 · 되돌리기 명령을 처리하는 중
		self._job: threading.Thread | None = None   # 로봇 기능 다시 켜기 · 새로 보정 스레드
		self._abort: str | None = None              # 보정 프로그램이 뜨기 전에 온 취소의 이유
		self._ack_seq = 0
		self._pending_ack: tuple[int, str] | None = None   # (번호, 보낸 명령): 답을 기다린다
		self._step_failure: str | None = None              # 답이 없어 보정 프로그램을 죽인 이유
		self._files: dict[str, tuple[int, int, dict | None]] = {}   # 이름 → (mtime_ns, size, 내용)
		self._profiles: dict[Path, tuple[tuple[int, int, int], dict]] = {}   # YAML → ((ino, mtime, size), 값)
		self._is_closed = False

	# ── 상태 ───────────────────────────────────────────────
	def is_wanting(self) -> bool:
		"""(robot._lock 안에서 불린다) 보는 화면이 없어도 'calibration' · 'motors' 묶음을 원하는지."""
		with self._lock:
			return self._wanting > 0

	def is_active(self) -> bool:
		"""새로 보정 · 고르기가 도는 중 (그동안 로봇 기능을 켜지 않는다: 버스가 하나다)."""
		with self._lock:
			return self._is_busy or self._session['state'] in ACTIVE_SESSION_STATES or (
				self._job is not None and self._job.is_alive())

	def is_recording(self) -> bool:
		with self._lock:
			return self._session['state'] == 'recording'

	def status(self, follower: dict | None) -> dict:
		"""status.calibration.follower (docs/calibration_plan.md).

		Args:
			follower: snapshot()['arm']['status'] (팔로워가 알린 robot_id, 'arm' 묶음을 볼 때만).
		"""
		node = self._robot._node
		registers = None if node is None else node.arm_registers()
		files = self._read_files()
		parameters = self._effective_parameters()
		robot_id = (follower or {}).get('robot_id') or parameters.get('robot_id')
		listed = []
		for name, (mtime_ns, _size, data) in sorted(files.items()):
			is_match = None if registers is None or data is None else registers_match(data, registers)
			if data is None:
				is_match = False
			listed.append({'id': name, 'is_match': is_match, 'modified': round(mtime_ns / 1e9, 3)})
		current = next((item for item in listed if item['id'] == robot_id), None)
		is_restore_pending = config.follower_restore_path().exists()
		reason = None
		if is_restore_pending:
			reason = f'지난 새로 보정이 끝나지 않았습니다. {RESTORE_HINT}'
		elif robot_id is None:
			reason = '쓰는 보정 이름을 모릅니다 (팔로워 · 프로파일)'
		elif current is None:
			reason = f'보정 파일 {robot_id}.json 이 없습니다'
		elif current['is_match'] is None:
			reason = '모터 보정 값을 아직 받지 못했습니다 (lekiwi/motor/arm_registers)'
		elif not current['is_match']:
			reason = f'{robot_id}.json 이 모터와 다릅니다'
		with self._lock:
			session = dict(self._session)
		rest = parameters.get('rest_pose_ticks')
		return {
			'robot_id': robot_id,
			'is_match': None if current is None else current['is_match'],
			'reason': reason,
			'files': listed[:MAX_FILES],
			'motors': None if registers is None else {
				'ids': registers['ids'], 'homing_offset': registers['homing_offset'],
				'min': registers['min'], 'max': registers['max'],
			},
			'rest_pose_ticks': list(rest) if is_rest_pose(rest) else None,
			'restore_pending': is_restore_pending,
			'session': session,
		}

	# ── 명령 (조작 권한) ───────────────────────────────────
	def command(self, client_id: str, action: str, body: dict) -> CommandResult:
		"""select {robot_id} · from_motors {robot_id} · rest_pose · start {robot_id} · restore · middle · finish ·
		cancel."""
		denied = self._robot._check_operator(client_id)
		if denied is not None:
			return denied
		robot_id = body.get('robot_id')
		if action in ('select', 'from_motors', 'start') and not is_valid_id(robot_id):
			return error_result('bad_request', 'robot_id 는 [A-Za-z0-9_-] 1~40 글자여야 합니다')
		if action in ('middle', 'finish', 'cancel'):
			return self._step(action)
		zmq_blocked = self._robot.zmq_host_blocked()
		if zmq_blocked is not None:   # PC(ZMQ) 가 팔을 움직이는 중: 모터 버스 · 보정을 바꾸지 않는다
			return error_result('conflict', zmq_blocked)
		# 되돌릴 값이 남아 있으면 지금 모터 값은 반쯤 쓴 보정일 수 있다: 그 값으로 파일 · 접은 자세를 만들지 않는다
		if action in ('select', 'from_motors', 'rest_pose') and config.follower_restore_path().exists():
			return error_result('conflict', '끊긴 보정의 복원이 남아 있습니다. 먼저 ‘복원’을 누르세요')
		if action == 'from_motors':
			if self.is_active():   # 보정 프로그램이 모터 보정을 바꾸는 중: 레지스터 값이 맞지 않다
				return error_result('conflict', '다른 보정 작업이 도는 중입니다')
			return self._from_motors(robot_id)
		if action not in ('select', 'rest_pose', 'start', 'restore'):
			return error_result('bad_request', f'모르는 보정 명령입니다: {action}')
		with self._command_lock:   # 두 요청이 같이 막는 것을 지나가지 않게
			blocked = self._arm_blocked()
			if blocked is not None:
				return error_result('conflict', blocked)
			with self._lock:
				self._is_busy = True
		try:   # 스레드를 넘기는 동작은 바쁨을 풀기 전에 _job 을 켠다 (is_active 가 끊기지 않게)
			if action == 'select':
				return self._select(robot_id)
			if action == 'rest_pose':
				return self._rest_pose()
			if action == 'start':
				return self._start(robot_id)
			return self._restore()
		finally:
			with self._lock:
				self._is_busy = False

	def abort(self, reason: str) -> None:
		"""(막지 않음, robot 잠금 안에서도) 도는 새로 보정을 취소한다 (조작 권한 끝 · 비상 정지).
		제로-포지션 위치 시키기로 쓴 값은 보정 프로그램이 되돌린다. 아직 뜨기 전이면 띄우지 않는다."""
		with self._lock:
			if self._session['state'] not in CANCELABLE_STATES:
				return
			self._abort = reason
			worker = self._worker
		if worker is not None:
			self._send_step(worker, 'cancel')
		self._robot._emit('warn', f'팔로워 보정을 취소합니다: {reason}')

	def shutdown(self) -> None:
		"""(게이트웨이 종료) 보정 프로그램에 취소를 보내고 끝나지 않으면 죽인다. 로봇 기능은 다시 켜지 않는다."""
		with self._lock:
			self._is_closed = True
			worker = self._worker
		if worker is not None and worker.poll() is None:
			self._send({'cmd': 'cancel'})
			try:
				worker.wait(config.FOLLOWER_WORKER_STEP_TIMEOUT / 2)
			except subprocess.TimeoutExpired:
				worker.kill()
				worker.wait()

	# ── 내부: 파일 · 프로파일 ──────────────────────────────
	def _calibration_dir(self) -> Path:
		return config.follower_calibration_dir(self._effective_parameters().get('calibration_dir'))

	def _read_files(self) -> dict[str, tuple[int, int, dict | None]]:
		"""보정 폴더의 <id>.json (이름이 맞는 것만). 바뀐 파일만 다시 읽는다."""
		folder = self._calibration_dir()
		found: dict[str, tuple[int, int, dict | None]] = {}
		try:
			paths = sorted(folder.glob('*.json'))
		except OSError:
			paths = []
		for path in paths:
			if not is_valid_id(path.stem):
				continue
			try:
				stat = path.stat()
			except OSError:
				continue
			cached = self._files.get(path.stem)
			if cached is not None and cached[:2] == (stat.st_mtime_ns, stat.st_size):
				found[path.stem] = cached
				continue
			try:
				data = json.loads(path.read_text(encoding='utf-8'))
			except (OSError, ValueError):
				data = None
			found[path.stem] = (stat.st_mtime_ns, stat.st_size, data if isinstance(data, dict) else None)
		self._files = found
		return found

	def _profile(self, path: Path) -> dict:
		"""read_follower_parameters 를 (inode, mtime, 크기) 로 기억한다 (status 가 10 Hz 까지 부른다). 고치지 않는다."""
		try:
			stat = path.stat()
		except OSError:
			return {}
		key = (stat.st_ino, stat.st_mtime_ns, stat.st_size)   # write_atomic 은 늘 새 inode
		cached = self._profiles.get(path)
		if cached is not None and cached[0] == key:
			return cached[1]
		parameters = read_follower_parameters(path)
		self._profiles[path] = (key, parameters)
		return parameters

	def _effective_parameters(self) -> dict:
		"""팔로워가 bringup 에서 받는 값: robot_profile.yaml 위에 arm_profile.yaml (bringup 과 같은 순서)."""
		return {**self._profile(config.robot_profile_path()), **self._profile(config.arm_profile_path())}

	def _write_arm_profile(
		self, robot_id: str | None = None, rest_pose_ticks: list[int] | None = None, is_rest_cleared: bool = False,
	) -> None:
		"""arm_profile.yaml 을 통째로 쓴다. 주지 않은 값은 지금 쓰는 값 그대로 (처음이면 robot_profile · 팔로워 값).
		is_rest_cleared 면 접은 자세를 지운다 (새 보정)."""
		parameters = self._effective_parameters()
		if robot_id is None:
			node = self._robot._node
			follower = None if node is None else node.snapshot()['arm']['status']
			robot_id = parameters.get('robot_id') or (follower or {}).get('robot_id')
		if robot_id is None:
			raise ValueError('쓰는 보정 이름을 몰라 arm_profile.yaml 을 쓰지 않았습니다')
		rest = rest_pose_ticks if rest_pose_ticks is not None else parameters.get('rest_pose_ticks')
		if is_rest_cleared or not is_rest_pose(rest):
			rest = None   # 잘못된 값을 옮겨 적지 않는다
		write_atomic(config.arm_profile_path(), arm_profile_text(str(robot_id), rest))

	# ── 내부: 막는 것 ──────────────────────────────────────
	def _arm_blocked(self) -> str | None:
		"""고르기 · 접은 자세 · 새로 보정 · 되돌리기를 막는 이유 (한국어), 해도 되면 None."""
		robot = self._robot
		with robot._lock:
			is_session = robot._arm_session is not None
			is_maybe_on = robot._is_arm_maybe_on
			is_exploring = robot._explore_token is not None
		node = robot._node
		if is_session:
			return '팔 원격 조작 중입니다. 먼저 ‘팔 멈춤’을 누르세요'
		if is_maybe_on or (node is not None and node.arm_ready() is True):
			return '팔 토크가 켜져 있습니다. 먼저 ‘팔 힘 빼기’를 누르세요'
		if robot._is_navigation_active() or robot._processes.state('mapping') in ACTIVE_STATES or is_exploring:
			return '지도 작성 · 자율 주행 · 자동 주행 중에는 할 수 없습니다'
		if robot._processes.state('bringup') == 'stopping':
			return '로봇 기능을 끄는 중입니다. 잠시 뒤 다시 누르세요'
		if self.is_active():
			return '다른 보정 작업이 도는 중입니다'
		return None

	def _wait_registers(self) -> dict | None:
		"""레지스터를 구독하고 (보는 화면이 없어도) REGISTERS_WAIT 까지 기다린다."""
		with self._lock:
			self._wanting += 1
		try:
			self._robot._update_wanted_streams()
			deadline = time.monotonic() + REGISTERS_WAIT
			node = self._robot._require_node()
			while (registers := node.arm_registers()) is None and time.monotonic() < deadline:
				time.sleep(0.05)
			return registers
		finally:
			with self._lock:
				self._wanting -= 1
			self._robot._update_wanted_streams()

	def _clear_registers(self) -> None:
		node = self._robot._node
		if node is not None:
			node.clear_arm_registers()

	# ── 내부: 동작 ─────────────────────────────────────────
	def _from_motors(self, robot_id: str) -> CommandResult:
		path = self._calibration_dir() / f'{robot_id}.json'
		if path.exists():
			return error_result('conflict', f'{robot_id}.json 이 이미 있습니다. 다른 이름을 쓰세요')
		registers = self._wait_registers()
		if registers is None:
			return error_result('unavailable', '모터 보정 값을 받지 못했습니다 (로봇 기능 · 팔 쓰기를 켜세요)')
		data = calibration_from_registers(registers)
		if data is None:
			return error_result('failed', '드라이버가 읽지 못한 팔 모터가 있습니다 (lekiwi/motor/arm_registers)')
		try:
			if path.exists():   # 그사이 생겼다
				return error_result('conflict', f'{robot_id}.json 이 이미 있습니다. 다른 이름을 쓰세요')
			write_atomic(path, json.dumps(data, indent=4) + '\n')
		except OSError as error:
			return error_result('failed', f'보정 파일을 쓰지 못했습니다: {error}')
		message = f'모터 값으로 보정 파일 {robot_id}.json 을 만들었습니다'
		self._robot._emit('info', message)
		return ok_result(message)

	def _rest_pose(self) -> CommandResult:
		"""지금 팔 tick (모터 상태 ID 1~6) 을 rest_pose_ticks 로 (다음에 로봇 기능을 켤 때 팔로워가 읽는다)."""
		with self._lock:
			self._wanting += 1
		try:
			self._robot._update_wanted_streams()
			node = self._robot._require_node()
			deadline = time.monotonic() + REGISTERS_WAIT
			ticks = None
			while ticks is None and time.monotonic() < deadline:
				motors = {motor['id']: motor for motor in node.snapshot()['motors']}
				if all(motor_id in motors and motors[motor_id]['online'] for motor_id in range(1, 7)):
					ticks = [int(motors[motor_id]['position_ticks']) for motor_id in range(1, 7)]
				else:
					time.sleep(0.05)
		finally:
			with self._lock:
				self._wanting -= 1
			self._robot._update_wanted_streams()
		if ticks is None:
			return error_result('unavailable', '팔 모터 6개의 지금 위치를 받지 못했습니다 (lekiwi/motor/states)')
		try:
			self._write_arm_profile(rest_pose_ticks=ticks)
		except (OSError, ValueError) as error:
			return error_result('failed', f'접은 자세를 저장하지 못했습니다: {error}')
		message = f'접은 자세를 저장했습니다: {ticks} (다음에 로봇 기능을 켤 때 적용됩니다)'
		self._robot._emit('info', message)
		return ok_result(message, rest_pose_ticks=ticks)

	def _select(self, robot_id: str) -> CommandResult:
		files = self._read_files()
		if robot_id not in files:
			return error_result('bad_request', f'보정 파일 {robot_id}.json 이 없습니다')
		data = files[robot_id][2]
		if data is None:
			return error_result('conflict', f'{robot_id}.json 을 읽을 수 없습니다 (lerobot 보정 파일이 아닙니다)')
		registers = self._wait_registers()
		if registers is None:
			return error_result('unavailable', '모터 보정 값을 받지 못해 맞는지 볼 수 없습니다 (로봇 기능을 켜세요)')
		if not registers_match(data, registers):
			return error_result('conflict', f'{robot_id}.json 이 모터와 다릅니다. 모터와 맞는 파일을 고르세요')
		try:
			self._write_arm_profile(robot_id=robot_id)
		except (OSError, ValueError) as error:
			return error_result('failed', f'arm_profile.yaml 을 쓰지 못했습니다: {error}')
		if self._robot._processes.state('bringup') not in BRINGUP_ON_STATES:
			message = f'보정 {robot_id} 를 골랐습니다 (로봇 기능을 켤 때 적용됩니다)'
			self._robot._emit('info', message)
			return ok_result(message)
		job = threading.Thread(target=self._restart_bringup, args=(f'보정 {robot_id} 를 골랐습니다',),
			name='calibration-select', daemon=True)
		with self._lock:
			self._job = job
		job.start()
		return ok_result(f'보정 {robot_id} 를 골랐습니다. 로봇 기능을 다시 켭니다')

	def _restart_bringup(self, done_message: str) -> None:
		"""(스레드) 로봇 기능을 끄고 다시 켠다 (고르기)."""
		options = self._bringup_options()
		reason = self._stop_bringup()
		if reason is None:
			reason = self._start_bringup(options)
		if reason is None:
			self._robot._emit('info', f'{done_message}. 로봇 기능을 다시 켰습니다')
		else:
			self._robot._emit('error', f'{done_message}, 그러나 {reason}')

	def _bringup_options(self) -> dict:
		entry = self._robot._processes.status()['bringup']
		return dict(entry['options']) if entry.get('options') else {'use_camera': True}

	def _stop_bringup(self) -> str | None:
		"""로봇 기능을 끄고 꺼질 때까지 기다린다 (켜져 있지 않으면 바로). 못 껐으면 이유."""
		processes = self._robot._processes
		if processes.state('bringup') in BRINGUP_ON_STATES:
			result = processes.stop('bringup')
			if not result['ok']:
				return f'로봇 기능을 끄지 못했습니다: {result["message"]}'
		deadline = time.monotonic() + config.BRINGUP_STOP_WAIT
		while processes.state('bringup') not in ('stopped', 'failed'):
			if time.monotonic() >= deadline:
				return f'로봇 기능이 {config.BRINGUP_STOP_WAIT:g}초 안에 꺼지지 않았습니다'
			time.sleep(0.1)
		return None

	def _start_bringup(self, options: dict) -> str | None:
		"""로봇 기능을 다시 켠다. 게이트웨이를 끄는 중이거나 보정 프로그램이 살아 있으면 (버스 주인은 하나) 켜지 않는다."""
		with self._lock:
			if self._is_closed:
				return '게이트웨이를 끄는 중입니다'
			worker = self._worker
		if worker is not None and worker.poll() is None:
			return '보정 프로그램이 아직 돌고 있어 로봇 기능을 다시 켜지 않았습니다'
		result = self._robot._processes.start('bringup', options)
		return None if result['ok'] else f'로봇 기능을 다시 켜지 못했습니다: {result["message"]}'

	def _start(self, robot_id: str) -> CommandResult:
		"""새로 보정: 로봇 기능을 끄고 보정 프로그램을 켠다 (스레드). 진행은 status · 알림. 있는 이름은 덮어쓰지 않는다."""
		if (self._calibration_dir() / f'{robot_id}.json').exists():
			return error_result('conflict', f'{robot_id}.json 이 이미 있습니다. 새 이름을 쓰세요')
		self._spawn_session(robot_id)
		self._robot._emit('info', f'팔로워 새로 보정을 시작합니다 ({robot_id}): 로봇 기능을 끕니다')
		return ok_result(f'팔로워 새로 보정을 시작합니다 ({robot_id})')

	def _restore(self) -> CommandResult:
		"""되돌리기: 지난 새로 보정이 남긴 예전 보정 값을 모터에 쓴다 (로봇 기능을 끄고 보정 프로그램으로)."""
		if not config.follower_restore_path().exists():
			return error_result('conflict', '되돌릴 예전 보정 값이 없습니다')
		self._spawn_session(None)
		self._robot._emit('info', '예전 보정 값을 모터에 되돌립니다: 로봇 기능을 끕니다')
		return ok_result('예전 보정 값을 모터에 되돌립니다')

	def _spawn_session(self, robot_id: str | None) -> None:
		job = threading.Thread(target=self._run_session, args=(robot_id,), name='calibration', daemon=True)
		with self._lock:
			self._session = {**IDLE_SESSION, 'state': 'starting', 'robot_id': robot_id}
			self._abort, self._step_failure, self._job = None, None, job
		job.start()

	def _run_session(self, robot_id: str | None) -> None:
		"""(스레드) 로봇 기능 끄기 → 보정 프로그램 (robot_id 가 None 이면 되돌리기만) → 꺼진 것 확인 → arm_profile →
		로봇 기능 다시 켜기."""
		options = self._bringup_options()
		was_running = self._robot._processes.state('bringup') in BRINGUP_ON_STATES
		state, reason, backup = 'error', None, None
		try:
			reason = self._stop_bringup()
			with self._lock:
				aborted = '게이트웨이를 끄는 중입니다' if self._is_closed else self._abort
			if reason is None and aborted is not None:
				state, reason = 'cancelled', aborted
			elif reason is None:
				self._clear_registers()   # 보정 프로그램이 모터 보정을 바꾼다: 드라이버가 다시 알릴 때까지 모른다
				try:
					state, reason, backup = self._run_worker(robot_id)   # 돌아오면 보정 프로그램은 꺼져 있다
				finally:
					self._clear_registers()   # 그사이 온 (꺼지던 드라이버의) 값도 버린다
		except Exception as error:   # 보정 프로그램을 띄우지 못했다 등
			logger.exception('팔로워 보정 중 오류')
			state, reason = 'error', f'보정 프로그램 오류: {error}'
		with self._lock:
			if state == 'cancelled' and reason is None:
				reason = self._abort
		if state == 'done' and robot_id is not None:
			try:
				self._write_arm_profile(robot_id=robot_id, is_rest_cleared=True)
				saved = f'{robot_id}.json 을 저장했습니다' + (f' (예전 파일: {Path(backup).name})' if backup else '')
				reason = f'{saved}. {REST_CLEARED_MESSAGE}'
			except (OSError, ValueError) as error:
				state, reason = 'error', f'보정은 저장했지만 arm_profile.yaml 을 쓰지 못했습니다: {error}'
		if was_running:
			restart = self._start_bringup(options)
			if restart is not None:
				reason = f'{reason or ""} · {restart}'
		with self._lock:
			self._worker = None
			self._session = {**self._session, 'state': state, 'reason': reason}
		if robot_id is None:
			label = {'cancelled': '예전 보정 값을 되돌렸습니다'}.get(state, '보정 되돌리기가 실패했습니다')
		else:
			label = {'done': '팔로워 보정을 마쳤습니다', 'cancelled': '팔로워 보정을 취소했습니다'}.get(
				state, '팔로워 보정이 실패했습니다')
		level = 'info' if state in ('done', 'cancelled') else 'error'
		self._robot._emit(level, f'{label}: {reason}' if reason else label)

	def _run_worker(self, robot_id: str | None) -> tuple[str, str | None, str | None]:
		"""보정 프로그램을 켜고 끝날 때까지 상태를 읽는다. 돌아올 때는 늘 꺼져 있다. (끝 상태, 이유, 백업 경로)."""
		worker = subprocess.Popen(
			config.follower_worker_command(), stdin=subprocess.PIPE, stdout=subprocess.PIPE,
			stderr=subprocess.PIPE, text=True, encoding='utf-8', errors='replace', bufsize=1,
			start_new_session=True)
		threading.Thread(target=self._log_stderr, args=(worker,), daemon=True).start()
		started = time.monotonic()
		state, reason, backup = 'error', '보정 프로그램이 끝을 알리지 않고 꺼졌습니다', None
		has_reported = False
		watchdog = threading.Timer(config.FOLLOWER_WORKER_START_TIMEOUT, lambda: has_reported or worker.kill())
		watchdog.daemon = True
		try:
			folder = self._calibration_dir()
			folder.mkdir(parents=True, exist_ok=True)
			start = {
				'cmd': 'start', 'port': read_motor_port(config.robot_profile_path()),
				'calibration_path': str(folder / f'{robot_id}.json') if robot_id else '',
				'restore_path': str(config.follower_restore_path()), 'restore_only': robot_id is None,
			}
			try:   # 다른 명령 (취소) 이 start 앞에 가지 않게 _worker 로 알리기 전에 보낸다
				worker.stdin.write(json.dumps(start) + '\n')
				worker.stdin.flush()
			except (OSError, ValueError):
				return 'error', '보정 프로그램에 보내지 못했습니다', None
			with self._lock:
				self._worker = worker
				aborted = '게이트웨이를 끄는 중입니다' if self._is_closed else self._abort
			if aborted is not None:
				self._send({'cmd': 'cancel'})
			watchdog.start()
			for line in worker.stdout:
				message = self._parse_worker_line(line)
				if message is None:
					continue
				has_reported = True
				state = message['state']
				with self._lock:
					self._session = {
						**self._session, 'state': state, 'reason': message['reason'],
						'joints': message['joints'] if message['joints'] is not None else self._session['joints'],
					}
					if self._pending_ack is not None and message['ack'] == self._pending_ack[1]:
						self._pending_ack = None
				if state in END_STATES:
					reason, backup = message['reason'], message['backup']
					break
		finally:
			watchdog.cancel()
			with self._lock:
				self._pending_ack = None
			if worker.poll() is None and state not in END_STATES:   # 아직 돈다: 되돌리게 취소부터
				self._send({'cmd': 'cancel'})
			try:
				worker.wait(config.FOLLOWER_WORKER_STEP_TIMEOUT)
			except subprocess.TimeoutExpired:
				worker.kill()
				worker.wait()
		if state not in END_STATES:   # 끝을 알리지 않고 꺼졌다 (죽였다)
			with self._lock:
				failure = self._step_failure
			if failure is not None:
				reason = failure
			elif not has_reported and time.monotonic() - started >= config.FOLLOWER_WORKER_START_TIMEOUT:
				reason = f'보정 프로그램이 {config.FOLLOWER_WORKER_START_TIMEOUT:g}초 안에 시작하지 않았습니다'
			if config.follower_restore_path().exists() and RESTORE_HINT not in reason:
				reason = f'{reason}. {RESTORE_HINT}'
			state = 'error'
		return state, reason, backup

	def _parse_worker_line(self, line: str) -> dict | None:
		"""보정 프로그램의 한 줄 → {state, joints, reason, backup, ack} (검사한 것만, 아니면 None)."""
		try:
			data = json.loads(line)
		except ValueError:
			return None
		if not isinstance(data, dict) or data.get('state') not in SESSION_STATES:
			return None
		joints = data.get('joints')
		if joints is not None:
			if not isinstance(joints, dict) or len(joints) > MAX_JOINTS:
				joints = None
			else:
				joints = {
					str(name)[:40]: {key: int(value[key]) for key in ('now', 'min', 'max')}
					for name, value in joints.items()
					if isinstance(value, dict) and all(
						isinstance(value.get(key), int) and not isinstance(value.get(key), bool)
						for key in ('now', 'min', 'max'))
				}
		reason = data.get('reason')
		backup = data.get('backup')
		return {
			'state': data['state'], 'joints': joints,
			# lerobot 오류 글에는 줄바꿈이 있다: 한 줄로 (화면 · 알림)
			'reason': ' '.join(reason.split())[:300] if isinstance(reason, str) else None,
			'backup': backup if isinstance(backup, str) else None,
			'ack': data.get('ack') if data.get('ack') in STEPS else None,
		}

	def _step(self, action: str) -> CommandResult:
		"""제로-포지션 위치 시키기 · 끝내기 · 취소를 보정 프로그램에 넘긴다. FOLLOWER_WORKER_STEP_TIMEOUT 안에 답이 없으면 죽인다."""
		with self._lock:
			state = self._session['state']
			worker = self._worker
			expected = {'middle': ('waiting_middle',), 'finish': ('recording',), 'cancel': CANCELABLE_STATES}[action]
			if state not in expected:
				return error_result('conflict', f'지금은 할 수 없습니다 (보정 상태 {state})')
			if worker is None:   # 아직 로봇 기능을 끄는 중: 보정 프로그램을 띄우지 않는다
				if action != 'cancel':
					return error_result('conflict', '보정 프로그램을 켜는 중입니다. 잠시 뒤 다시 누르세요')
				self._abort = '취소를 눌렀습니다'
				return ok_result('보정을 취소합니다')
		if not self._send_step(worker, action):
			return error_result('failed', '보정 프로그램에 보내지 못했습니다')
		labels = {'middle': '가운데를 잡습니다', 'finish': '보정을 저장합니다', 'cancel': '보정을 취소합니다'}
		return ok_result(labels[action])

	def _send_step(self, worker: subprocess.Popen, action: str) -> bool:
		"""명령을 보내고 FOLLOWER_WORKER_STEP_TIMEOUT 안에 답 (ack) 이 없으면 죽인다 (_check_ack)."""
		with self._lock:
			self._ack_seq += 1
			token = (self._ack_seq, action)
			self._pending_ack = token
		if not self._send({'cmd': action}):
			return False
		timer = threading.Timer(config.FOLLOWER_WORKER_STEP_TIMEOUT, self._check_ack, args=(worker, token))
		timer.daemon = True
		timer.start()
		return True

	def _check_ack(self, worker: subprocess.Popen, token: tuple[int, str]) -> None:
		"""(타이머) 명령에 답이 없으면 보정 프로그램을 죽인다 (버스가 멈췄다: 로봇 기능을 켜려면 버스를 놓아야 한다).
		아직 뜨는 중 (starting) 이면 FOLLOWER_WORKER_START_TIMEOUT 이 본다 (그 뒤 취소는 쓴 것이 없어 버스를 쓰지 않는다)."""
		with self._lock:
			if self._pending_ack != token or self._worker is not worker or self._session['state'] == 'starting':
				return
			self._step_failure = (
				f'보정 프로그램이 {STEP_LABELS[token[1]]}에 {config.FOLLOWER_WORKER_STEP_TIMEOUT:g}초 안에 답하지 않아 '
				f'껐습니다. {RESTORE_HINT}')
		logger.error('팔로워 보정 프로그램이 %s 에 답하지 않아 죽입니다', token[1])
		worker.kill()

	def _send(self, message: dict) -> bool:
		with self._lock:
			worker = self._worker
		if worker is None or worker.poll() is not None:
			return False
		try:
			with self._worker_lock:
				worker.stdin.write(json.dumps(message) + '\n')
				worker.stdin.flush()
		except (OSError, ValueError):
			return False
		return True

	def _log_stderr(self, worker: subprocess.Popen) -> None:
		for line in worker.stderr:
			logger.info('팔로워 보정 프로그램: %s', line.rstrip()[:300])
