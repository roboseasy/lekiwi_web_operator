"""리더암: 포트 · 보정 파일 찾기, lerobot SO101Leader 열기, 30 Hz 읽기 스레드.

게이트웨이 규약은 모른다. 상태와 관절 값은 콜백으로 넘기고, 보내는 일은 agent 가 한다.
시험은 Leader 를 흉내 낸 가짜를 open_leader 로 넣는다 (lerobot 을 가져오지 않는다).
"""

import glob
import logging
import os
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Final, Protocol

logger = logging.getLogger(__name__)

LEADER_RATE: Final[float] = 30.0      # [Hz] 리더 값 읽기 · 보내기
READ_ERROR_LIMIT: Final[int] = 3      # 이만큼 연달아 못 읽으면 오류 (한 번의 응답 누락은 넘긴다)
ACM_GLOB: Final[str] = '/dev/ttyACM*'
BY_ID_GLOB: Final[str] = '/dev/serial/by-id/*'
LEADER_TYPE_DIR: Final[str] = 'so_leader'   # lerobot SOLeader.name: 보정 파일 폴더 이름

STATE_DISCONNECTED: Final[str] = 'disconnected'
STATE_CONNECTING: Final[str] = 'connecting'
STATE_CONNECTED: Final[str] = 'connected'
STATE_ERROR: Final[str] = 'error'

# 자주 나는 오류 → 사용자가 할 일 (README 문제 해결과 같다)
_HINTS: Final[tuple[tuple[str, str], ...]] = (
	('Permission denied', '포트 권한이 없습니다 (sudo usermod -aG dialout $USER 뒤 다시 로그인)'),
	('busy', '다른 프로그램이 포트를 쓰고 있습니다 (lerobot · 다른 에이전트를 끄세요)'),
	('No such file', '포트가 없습니다 (USB 케이블을 확인하세요)'),
	('Missing motor IDs', '모터가 답하지 않습니다 (리더암 전원 · 케이블을 확인하세요)'),
	('no status packet', '모터가 답하지 않습니다 (리더암 전원 · 케이블을 확인하세요)'),
)


class LeaderError(Exception):
	"""리더암을 열 수 없다. 메시지는 그대로 사용자에게 보인다 (한국어)."""


class Leader(Protocol):
	"""열린 리더암 하나. 만들어지면 연결된 상태다."""

	def read(self) -> dict[str, float]:
		"""관절 값 {'arm_shoulder_pan.pos': 도, …, 'arm_gripper.pos': 0~100}."""
		...

	def close(self) -> None:
		"""포트를 닫는다. 여러 번 불러도 된다."""
		...


def list_ports() -> list[dict[str, str | None]]:
	"""/dev/ttyACM* 와 그것을 가리키는 /dev/serial/by-id 경로 (없으면 None)."""
	by_id: dict[str, str] = {}
	for link in sorted(glob.glob(BY_ID_GLOB)):
		by_id.setdefault(os.path.realpath(link), link)
	return [{'path': path, 'by_id': by_id.get(path)} for path in sorted(glob.glob(ACM_GLOB))]


def default_calibration_dir() -> Path:
	"""lerobot 이 SO101Leader 보정 파일을 찾는 폴더 (HF_LEROBOT_CALIBRATION 을 따른다).

	Raises:
		ImportError: 이 파이썬에 lerobot 이 없다.
	"""
	from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS
	return Path(HF_LEROBOT_CALIBRATION) / TELEOPERATORS / LEADER_TYPE_DIR


def describe_error(prefix: str, error: Exception) -> str:
	"""예외 → 한국어 이유 한 줄 (알려진 오류면 할 일을 앞에 붙인다)."""
	if isinstance(error, LeaderError):
		return str(error)
	text = str(error) or type(error).__name__
	if error.__cause__ is not None:  # lerobot 은 시리얼 오류 (EBUSY 등) 를 ConnectionError 로 감싼다
		text += f' ({error.__cause__})'
	text = ' '.join(text.split())
	for pattern, hint in _HINTS:
		if pattern.lower() in text.lower():
			return f'{prefix}: {hint} — {text}'
	return f'{prefix}: {text}'


class LerobotLeader:
	"""lerobot SO101Leader 하나 (도 단위). 만들면 연결된다.

	보정은 절대 돌리지 않는다 (connect(calibrate=False)): 파일이 모터 값과 다르면 LeaderError.
	lerobot 의 connect 는 토크를 끄고 설정 레지스터를 쓴다 (리더에는 무해하다).

	Args:
		port: 시리얼 포트 (/dev/ttyACM0 또는 /dev/serial/by-id/…).
		calibration_id: 보정 파일 이름 (<id>.json).
		calibration_dir: 보정 파일 폴더.

	Raises:
		LeaderError: 보정 파일이 모터 값과 다르다.
		Exception: lerobot · 시리얼 오류 (포트 없음 · 권한 · 모터 응답 없음).
	"""

	def __init__(self, port: str, calibration_id: str, calibration_dir: Path) -> None:
		# 가져오는 데 1초쯤 걸려 처음 연결할 때 (리더 스레드에서) 가져온다
		from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

		from lekiwi_pc_agent.calibration import lock_port
		self._leader = SO101Leader(SO101LeaderConfig(
			port=port, id=calibration_id, calibration_dir=calibration_dir))
		try:
			self._leader.connect(calibrate=False)
			lock_port(self._leader.bus)  # 뒤에 켠 lerobot 프로그램이 같은 버스를 쓰지 않게
			if not self._leader.is_calibrated:
				raise LeaderError(
					f'보정 파일 {calibration_id}.json 이 모터 값과 다릅니다 '
					'(다른 리더암의 파일이면 맞는 보정을 고르고, 아니면 lerobot-calibrate 로 다시 보정하세요)')
		except BaseException:
			self.close()
			raise

	def read(self) -> dict[str, float]:
		# lerobot LeKiwi 기록 루프처럼 arm_ 를 붙인다: 'shoulder_pan.pos' → 'arm_shoulder_pan.pos'
		return {f'arm_{key}': float(value) for key, value in self._leader.get_action().items()}

	def close(self) -> None:
		if not self._leader.is_connected:
			return
		try:
			self._leader.disconnect()
		except Exception:
			# 뽑힌 뒤라 토크 끄기가 실패해도 포트는 닫는다
			self._leader.bus.port_handler.closePort()


class LeaderLink:
	"""리더암 하나를 맡는 스레드: 연결 → 30 Hz 읽기 → 끊기.

	connect · disconnect 는 막히지 않는다 (lerobot 연결이 1초 넘게 걸려도 생존 신호가 멈추지
	않게). 새 연결은 앞 스레드가 포트를 닫을 때까지 기다린 뒤 연다.

	Args:
		open_leader: (port, calibration_id) → 열린 Leader. 실패하면 예외.
		on_status: 상태가 바뀔 때 {'state', 'port', 'calibration_id', 'reason'} 로 부른다.
		on_joints: 읽은 관절 값마다 부른다 (리더 스레드에서).
	"""

	def __init__(
			self,
			open_leader: Callable[[str, str], Leader],
			on_status: Callable[[dict], None],
			on_joints: Callable[[dict[str, float]], None]) -> None:
		self._open_leader = open_leader
		self._on_status = on_status
		self._on_joints = on_joints
		self._lock = threading.Lock()
		self._worker: threading.Thread | None = None
		self._stop_event = threading.Event()
		self._status: dict = {
			'state': STATE_DISCONNECTED, 'port': None, 'calibration_id': None, 'reason': None}

	@property
	def status(self) -> dict:
		"""지금 상태 (복사본)."""
		with self._lock:
			return dict(self._status)

	def connect(self, port: str, calibration_id: str) -> None:
		"""이 포트 · 보정으로 (다시) 연결한다. 열려 있던 리더는 먼저 닫는다."""
		with self._lock:
			self._stop_event.set()
			stop_event = threading.Event()
			worker = threading.Thread(
				target=self._run, args=(port, calibration_id, stop_event, self._worker),
				name='leader', daemon=True)
			self._worker, self._stop_event = worker, stop_event
			worker.start()

	def disconnect(self) -> None:
		"""리더를 닫는다 (스레드가 닫고 disconnected 를 알린다). 오류 상태면 지운다."""
		with self._lock:
			self._stop_event.set()
			is_running = self._worker is not None and self._worker.is_alive()
		if not is_running and self.status['state'] != STATE_DISCONNECTED:
			status = self.status
			self._set_status(STATE_DISCONNECTED, status['port'], status['calibration_id'])

	def close(self, timeout: float) -> bool:
		"""리더를 닫고 스레드를 timeout 까지 기다린다.

		Returns:
			스레드가 끝나 포트가 풀렸는지 (lerobot 연결 중이면 늦을 수 있다).
		"""
		self.disconnect()
		worker = self._worker
		if worker is not None:
			worker.join(timeout)
			return not worker.is_alive()
		return True

	def _set_status(
			self, state: str, port: str | None, calibration_id: str | None,
			reason: str | None = None) -> None:
		status = {'state': state, 'port': port, 'calibration_id': calibration_id, 'reason': reason}
		with self._lock:
			self._status = status
		self._on_status(dict(status))

	def _run(
			self, port: str, calibration_id: str, stop_event: threading.Event,
			previous: threading.Thread | None) -> None:
		if previous is not None:
			previous.join()  # 같은 포트를 두 번 열지 않는다
		if stop_event.is_set():
			return
		self._set_status(STATE_CONNECTING, port, calibration_id)
		try:
			leader = self._open_leader(port, calibration_id)
		except Exception as error:
			logger.debug('리더암 열기 실패', exc_info=True)
			self._set_status(
				STATE_ERROR, port, calibration_id, describe_error('리더암을 열지 못했습니다', error))
			return
		reason = None
		try:
			if not stop_event.is_set():
				self._set_status(STATE_CONNECTED, port, calibration_id)
				reason = self._stream(leader, stop_event)
		finally:
			try:
				leader.close()
			except Exception:
				logger.warning('리더암을 닫다가 오류가 났습니다', exc_info=True)
		if reason is None:
			self._set_status(STATE_DISCONNECTED, port, calibration_id)
		else:
			self._set_status(STATE_ERROR, port, calibration_id, reason)

	def _stream(self, leader: Leader, stop_event: threading.Event) -> str | None:
		"""멈추라고 할 때까지 LEADER_RATE 로 읽는다. 읽기 오류면 그 이유를 돌려준다."""
		period = 1.0 / LEADER_RATE
		failures = 0
		next_time = time.monotonic()
		while not stop_event.is_set():
			try:
				joints = leader.read()
			except Exception as error:
				failures += 1
				if failures >= READ_ERROR_LIMIT:
					logger.debug('리더암 읽기 실패', exc_info=True)
					return describe_error('리더암을 읽지 못했습니다', error)
			else:
				failures = 0
				self._on_joints(joints)
			next_time += period
			delay = next_time - time.monotonic()
			if delay < 0:
				next_time = time.monotonic()  # 밀렸으면 따라잡으려 몰아 보내지 않는다
			stop_event.wait(max(delay, 0.0))
		return None
