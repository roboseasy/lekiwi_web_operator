"""리더암 보정 (규약 v2 calib_*): 버스를 쓰는 일을 스레드 하나에서 차례로 한다.

단계 자체는 calibration.py (게이트웨이 팔로워와 같은 코드) 가 한다. 여기서는 명령 순서 ·
버스 열고 닫기 · 상태 알리기 · 범위 재기 주기를 맡는다. 버스를 열기 전에 스트리밍 리더를 닫는다.
"""

import logging
import queue
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Final

from lekiwi_pc_agent import calibration
from lekiwi_pc_agent.calibration import CalibrationError, CalibrationSession
from lekiwi_pc_agent.leader import describe_error

logger = logging.getLogger(__name__)

RECORD_PERIOD: Final[float] = 1.0 / 50   # [s] 범위 재기 읽기 (끝 값을 놓치지 않게 촘촘히)
STATUS_PERIOD: Final[float] = 0.1        # [s] 범위 재는 동안 calib_status (10 Hz)

STATE_IDLE: Final[str] = 'idle'
STATE_STARTING: Final[str] = 'starting'

COMMAND_CHECK: Final[str] = 'check'
COMMAND_FROM_MOTORS: Final[str] = 'from_motors'
COMMAND_START: Final[str] = 'start'
COMMAND_MIDDLE: Final[str] = 'middle'
COMMAND_FINISH: Final[str] = 'finish'
COMMAND_CANCEL: Final[str] = 'cancel'
COMMANDS: Final[tuple[str, ...]] = (
	COMMAND_CHECK, COMMAND_FROM_MOTORS, COMMAND_START, COMMAND_MIDDLE, COMMAND_FINISH, COMMAND_CANCEL)
_BUS_COMMANDS: Final[tuple[str, ...]] = (COMMAND_CHECK, COMMAND_FROM_MOTORS, COMMAND_START)
_STOP: Final[str] = 'stop'


class CalibrationWorker:
	"""보정 명령을 차례로 처리하는 스레드.

	Args:
		calibration_dir: 리더 보정 파일 폴더.
		open_bus: port → 연 버스 (포트 확인 포함). 실패하면 예외.
		release_leader: 스트리밍 리더를 닫고 포트가 풀릴 때까지 기다린다.
		on_status: 상태가 바뀔 때 {'state', 'calibration_id', 'matches', 'joints', 'reason'}.
		on_files_changed: 보정 파일을 만들었을 때.
	"""

	def __init__(
			self,
			calibration_dir: Path,
			open_bus: Callable[[str | None], object],
			release_leader: Callable[[], None],
			on_status: Callable[[dict], None],
			on_files_changed: Callable[[], None]) -> None:
		self._calibration_dir = calibration_dir
		self._open_bus = open_bus
		self._release_leader = release_leader
		self._on_status = on_status
		self._on_files_changed = on_files_changed
		self._queue: queue.Queue = queue.Queue()
		self._lock = threading.Lock()
		self._pending = 0
		self._bus = None
		self._session: CalibrationSession | None = None
		self._status: dict = {
			'state': STATE_IDLE, 'calibration_id': None, 'matches': None, 'joints': None,
			'reason': None}
		self._thread = threading.Thread(target=self._run, name='calibration', daemon=True)
		self._thread.start()

	@property
	def status(self) -> dict:
		"""지금 상태 (복사본)."""
		with self._lock:
			return dict(self._status)

	@property
	def is_busy(self) -> bool:
		"""버스를 쓰는 중이거나 쓸 명령이 기다리는 중 (이때 리더암을 연결하지 않는다)."""
		with self._lock:
			return self._pending > 0 or self._session is not None

	def submit(self, command: str, port: str | None = None, calibration_id: object = None) -> None:
		"""명령 하나를 넣는다 (막히지 않는다)."""
		with self._lock:
			if command in _BUS_COMMANDS:
				self._pending += 1
		self._queue.put((command, port, calibration_id))

	def close(self, timeout: float) -> None:
		"""끝낼 때: 하던 보정은 취소해 (제로-포지션 위치 시키기 값을 되돌리고) 버스를 닫는다."""
		self._queue.put((_STOP, None, None))
		self._thread.join(timeout)

	def _run(self) -> None:
		last_status = 0.0
		while True:
			is_recording = (
				self._session is not None and self._session.state == calibration.STATE_RECORDING)
			try:
				command, port, calibration_id = self._queue.get(
					timeout=RECORD_PERIOD if is_recording else None)
			except queue.Empty:
				try:
					joints = self._session.sample()
				except Exception as error:
					self._fail(
						describe_error('범위를 읽지 못했습니다', error), self._status['calibration_id'])
					continue
				with self._lock:
					self._status['joints'] = joints
				if time.monotonic() - last_status >= STATUS_PERIOD:
					last_status = time.monotonic()
					self._on_status(self.status)
				continue
			if command == _STOP:
				if self._session is not None:
					try:
						self._cancel()
					except Exception:
						logger.warning('끝내면서 보정 값을 되돌리지 못했습니다', exc_info=True)
				return
			if command not in _BUS_COMMANDS:
				calibration_id = self._status['calibration_id']
			try:
				self._do(command, port, calibration_id)
			except CalibrationError as error:
				if self._session is not None:
					self._notice(str(error))  # 보정 중 할 수 없는 단계: 상태는 그대로, 이유만 알린다
				else:
					self._fail(str(error), calibration_id)
			except Exception as error:
				logger.debug('보정 오류', exc_info=True)
				self._fail(describe_error('보정하지 못했습니다', error), calibration_id)
			finally:
				if command in _BUS_COMMANDS:
					with self._lock:
						self._pending -= 1

	def _do(self, command: str, port: str | None, calibration_id: object) -> None:
		session = self._session
		if command == COMMAND_CANCEL:
			if session is None:
				self._set(STATE_IDLE)
			else:
				self._cancel()
			return
		if command in (COMMAND_MIDDLE, COMMAND_FINISH):
			if session is None:
				raise CalibrationError('보정을 시작하지 않았습니다')
			if command == COMMAND_MIDDLE:
				session.middle()
				self._set(session.state, calibration_id, joints=session.sample())
			else:
				self._finish(session)
			return
		if session is not None:
			raise CalibrationError('보정 중입니다 (끝내거나 취소한 뒤 하세요)')
		if command == COMMAND_CHECK:
			self._with_bus(port, lambda bus: self._set(
				STATE_IDLE, matches=calibration.check_files(bus, self._calibration_dir)))
			return
		if not calibration.is_valid_id(calibration_id):
			raise CalibrationError(f'보정 이름은 영문 · 숫자 · _ · - 로 40자까지입니다: {calibration_id!r}')
		path = self._calibration_dir / f'{calibration_id}.json'
		if command == COMMAND_FROM_MOTORS:
			if path.exists():
				raise CalibrationError(f'이미 있는 보정 이름입니다: {calibration_id} (다른 이름을 쓰세요)')
			self._with_bus(port, lambda bus: self._save_from_motors(bus, path, calibration_id))
		elif command == COMMAND_START:
			self._set(STATE_STARTING, calibration_id)
			self._release_leader()
			self._bus = self._open_bus(port)
			try:
				self._session = CalibrationSession(self._bus)
			except BaseException:
				self._close_bus()
				raise
			self._set(self._session.state, calibration_id)
		else:
			raise CalibrationError(f'모르는 보정 명령입니다: {command}')

	def _with_bus(self, port: str | None, action: Callable[[object], None]) -> None:
		"""리더를 닫고 버스를 열어 action 한 번 뒤 닫는다 (확인 · 모터 값으로 만들기)."""
		self._release_leader()
		self._bus = self._open_bus(port)
		try:
			action(self._bus)
		finally:
			self._close_bus()

	def _save_from_motors(self, bus: object, path: Path, calibration_id: str) -> None:
		calibration.save_calibration_file(path, calibration.calibration_from_motors(bus))
		logger.info('모터 값으로 보정 파일을 만들었습니다: %s', path)
		self._set(calibration.STATE_DONE, calibration_id, matches=self._recheck(bus))
		self._on_files_changed()

	def _finish(self, session: CalibrationSession) -> None:
		calibration_id = self._status['calibration_id']
		joints = self._status['joints']
		self._set(calibration.STATE_SAVING, calibration_id, joints=joints)
		try:
			path = session.finish(self._calibration_dir / f'{calibration_id}.json')
		except CalibrationError:
			self._set(session.state, calibration_id, joints=joints)  # 움직이지 않은 관절: recording 에 남는다
			raise
		logger.info('보정을 저장했습니다: %s (백업 %s)', path, session.backup_path)
		self._session = None  # 저장은 끝났다: 아래가 실패해도 오류 · 되돌리기로 가지 않는다
		matches = self._recheck(self._bus)
		self._close_bus()
		self._set(calibration.STATE_DONE, calibration_id, matches=matches, joints=joints)
		self._on_files_changed()

	def _recheck(self, bus: object) -> dict[str, bool] | None:
		"""저장한 뒤 파일 대조. 실패해도 저장은 이미 끝났으므로 None (대조 결과 없음) 으로 넘긴다."""
		try:
			return calibration.check_files(bus, self._calibration_dir)
		except Exception:
			logger.warning('저장 뒤 보정 파일 대조에 실패했습니다', exc_info=True)
			return None

	def _cancel(self) -> None:
		session, calibration_id = self._session, self._status['calibration_id']
		self._session = None
		try:
			session.cancel()
		finally:
			self._close_bus()
		self._set(calibration.STATE_CANCELLED, calibration_id)

	def _fail(self, reason: str, calibration_id: object) -> None:
		"""오류: 하던 보정을 되돌리고 버스를 닫는다."""
		session, self._session = self._session, None
		if session is not None:
			try:
				session.cancel()
			except Exception:
				logger.warning('보정 값을 되돌리지 못했습니다', exc_info=True)
				reason += ' (제로-포지션 위치 시키기 값을 되돌리지 못했습니다: 다시 보정하세요)'
		self._close_bus()
		if not calibration.is_valid_id(calibration_id):
			calibration_id = None  # 틀린 이름은 되돌려 보내지 않는다
		self._set(calibration.STATE_ERROR, calibration_id, reason=reason)

	def _close_bus(self) -> None:
		bus, self._bus = self._bus, None
		if bus is None:
			return
		try:
			bus.disconnect(disable_torque=False)
		except Exception:
			logger.warning('보정 버스를 닫다가 오류가 났습니다', exc_info=True)

	def _set(
			self, state: str, calibration_id: str | None = None, matches: dict | None = None,
			joints: dict | None = None, reason: str | None = None) -> None:
		with self._lock:
			self._status = {
				'state': state, 'calibration_id': calibration_id, 'matches': matches,
				'joints': joints, 'reason': reason}
		self._on_status(self.status)

	def _notice(self, reason: str) -> None:
		with self._lock:
			self._status['reason'] = reason
		self._on_status(self.status)
