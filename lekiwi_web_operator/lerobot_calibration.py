"""lerobot 0.6.0 보정 순서를 화면 버튼 단계로 나눈 것 (docs/calibration_plan.md).

리더 (노트북 PC 에이전트) 와 팔로워 (로봇 게이트웨이의 보정 프로그램) 가 같은 코드를 쓴다: 이 파일은 PC 에이전트 저장소
roboseasy/lekiwi_pc_agent 의 lekiwi_pc_agent/calibration.py 사본이다 (한쪽을 바꾸면 둘 다 바꾼다).
팔로워 보정 프로그램 (follower_calibration_worker.py) 만 lerobot 이 든 python 으로 가져온다.
그래서 lerobot · draccus · 표준 라이브러리 말고는 아무것도 가져오지 않는다.

lerobot 의 SO101Leader.calibrate() · LeKiwi.calibrate() 와 같은 순서 · 같은 버스 메서드다.
1. 토크 끔, Operating_Mode=POSITION (CalibrationSession 을 만들 때)
2. set_half_turn_homings: 지금 자세를 가운데 (2047) 로 (middle)
3. 범위 재기: Present_Position 의 최소 · 최대 (sample). wrist_roll · 바퀴는 0~4095
4. MotorCalibration (drive_mode 0) → write_calibration → <이름>.json (finish)
input() 을 부르지 않고, 토크를 켜는 일이 없다 (lerobot connect() → configure() 는 LeKiwi 토크를
켜므로 부르지 않는다. open_bus 는 버스만 열고 TIOCEXCL 로 독점한다).
"""

import dataclasses
import fcntl
import json
import os
import re
import shutil
import tempfile
import termios
import time
from pathlib import Path
from typing import Final

import draccus
from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

DEVICE_SO101_LEADER: Final[str] = 'so101_leader'
DEVICE_LEKIWI: Final[str] = 'lekiwi'
ID_PATTERN: Final[str] = r'[A-Za-z0-9_-]{1,40}'
MAX_CALIBRATION_IDS: Final[int] = 32   # 목록 · 대조 결과에 넣는 파일 수 (게이트웨이 메시지 한도)
FULL_TURN_KEYWORDS: Final[tuple[str, ...]] = ('wrist_roll', 'wheel')   # lerobot: 0~4095 로 둔다
NO_HOMING_KEYWORD: Final[str] = 'wheel'   # LeKiwi 바퀴: homing 0, 제로-포지션 위치 시키기 안 함
FULL_TURN_MAX: Final[int] = 4095

STATE_WAITING_MIDDLE: Final[str] = 'waiting_middle'
STATE_RECORDING: Final[str] = 'recording'
STATE_SAVING: Final[str] = 'saving'
STATE_DONE: Final[str] = 'done'
STATE_CANCELLED: Final[str] = 'cancelled'
STATE_ERROR: Final[str] = 'error'


class CalibrationError(Exception):
	"""보정 단계를 할 수 없다. 메시지는 그대로 사용자에게 보인다 (한국어)."""


def is_valid_id(calibration_id: object) -> bool:
	"""보정 이름이 [A-Za-z0-9_-]{1,40} 인지 (파일 이름으로 써도 안전한지)."""
	return isinstance(calibration_id, str) and re.fullmatch(ID_PATTERN, calibration_id) is not None


def open_bus(device_type: str, port: str) -> FeetechMotorsBus:
	"""lerobot 장치의 모터 표대로 버스를 열기만 한다 (핸드셰이크: ping · 펌웨어 읽기).

	장치 클래스는 모터 표를 얻으려고만 만들고 connect() · configure() 는 부르지 않는다:
	아무것도 쓰지 않고 토크 상태도 그대로다. 닫을 때는 bus.disconnect(disable_torque=False).

	Args:
		device_type: DEVICE_SO101_LEADER 또는 DEVICE_LEKIWI.
		port: 시리얼 포트.

	Raises:
		ValueError: 모르는 장치.
		ConnectionError: 포트를 못 열었거나 모터가 답하지 않는다 (lerobot).
	"""
	from lerobot.motors.feetech import FeetechMotorsBus

	if device_type == DEVICE_SO101_LEADER:
		from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
		device = SO101Leader(SO101LeaderConfig(port=port))
	elif device_type == DEVICE_LEKIWI:
		from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig
		device = LeKiwi(LeKiwiConfig(port=port, cameras={}))
	else:
		raise ValueError(f'모르는 장치입니다: {device_type}')
	motors, saved = device.bus.motors, device.bus.calibration
	# 장치는 모터 표만 얻고 연결 전에 버린다: lerobot 장치의 __del__ 은 연결돼 있으면 disconnect() 한다
	# (장치를 버리는 순간 이 버스를 닫고, LeKiwi 는 토크까지 끈다)
	del device
	bus = FeetechMotorsBus(port=port, motors=motors, calibration=saved)
	bus.connect()
	try:
		lock_port(bus)
	except BaseException:
		bus.disconnect(disable_torque=False)
		raise
	return bus


def lock_port(bus: FeetechMotorsBus) -> None:
	"""연 시리얼 포트를 TIOCEXCL 로 독점한다. 뒤에 같은 포트를 여는 프로그램은 EBUSY 로 실패한다.

	(C++ 드라이버 · lerobot-teleoperate 가 버스를 같이 써 패킷이 섞이지 않게.) root 는 막지 못하고,
	포트를 닫으면 풀린다. 이미 열려 있던 다른 프로그램은 막지 못한다.
	"""
	fcntl.ioctl(bus.port_handler.ser.fileno(), termios.TIOCEXCL)


def list_calibration_ids(calibration_dir: Path) -> list[str]:
	"""보정 폴더의 <id>.json 가운데 이름이 ID_PATTERN 인 것, 정렬해 MAX_CALIBRATION_IDS 개까지."""
	ids = (path.stem for path in Path(calibration_dir).glob('*.json'))
	return sorted(calibration_id for calibration_id in ids if is_valid_id(calibration_id))[
		:MAX_CALIBRATION_IDS]


def calibration_from_motors(bus: FeetechMotorsBus) -> dict[str, dict[str, int]]:
	"""모터 EEPROM 의 보정 (Homing_Offset · Min/Max_Position_Limit) → lerobot 보정 파일 형식.

	{motor: {'id', 'drive_mode', 'homing_offset', 'range_min', 'range_max'}}. 읽기만 한다.
	"""
	return {name: dataclasses.asdict(value) for name, value in bus.read_calibration().items()}


def load_calibration_file(path: Path) -> dict[str, MotorCalibration]:
	"""lerobot 보정 파일 하나.

	Raises:
		OSError · ValueError · TypeError · AttributeError: 없거나 형식이 틀렸다.
	"""
	data = json.loads(Path(path).read_text())
	return {name: MotorCalibration(**value) for name, value in data.items()}


def save_calibration_file(path: Path, calibration: dict[str, dict[str, int]]) -> Path | None:
	"""lerobot 형식 (draccus, indent 4) 으로 저장한다. 있던 파일은 <이름>.json.bak-<날짜시각> 로 복사해 둔다.

	SD 카드에서 전원이 나가도 <이름>.json 이 비거나 없어지지 않게:
	임시 파일에 쓰고 fsync → 있던 파일을 백업으로 복사 · fsync → os.replace → 폴더 fsync.
	<이름>.json 은 언제나 예전 것 아니면 새 것이다.

	Returns:
		남긴 백업 경로 (있던 파일이 없었으면 None).
	"""
	path = Path(path)
	values = {name: MotorCalibration(**value) for name, value in calibration.items()}
	descriptor, temporary = tempfile.mkstemp(dir=path.parent, prefix=f'.{path.name}.', suffix='.tmp')
	try:
		with os.fdopen(descriptor, 'w') as file, draccus.config_type('json'):
			os.fchmod(file.fileno(), 0o644)  # mkstemp 는 0600 으로 만든다
			draccus.dump(values, file, indent=4)
			file.flush()
			os.fsync(file.fileno())
		backup = _backup(path) if path.exists() else None
		os.replace(temporary, path)
	except BaseException:
		Path(temporary).unlink(missing_ok=True)
		raise
	_fsync_directory(path.parent)
	return backup


def _backup(path: Path) -> Path:
	"""path 를 <이름>.bak-<날짜시각>[-n] 으로 복사하고 fsync 한다. 이름은 O_EXCL 로 잡아 겹치지 않는다."""
	stamp = time.strftime('%Y%m%d-%H%M%S')
	for count in range(1000):
		backup = path.with_name(f'{path.name}.bak-{stamp}' + (f'-{count}' if count else ''))
		try:
			os.close(os.open(backup, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644))
		except FileExistsError:
			continue
		shutil.copy2(path, backup)
		with open(backup, 'rb') as file:
			os.fsync(file.fileno())
		return backup
	raise CalibrationError(f'백업 이름을 정하지 못했습니다: {path}.bak-{stamp}')


def _fsync_directory(directory: Path) -> None:
	descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
	try:
		os.fsync(descriptor)
	finally:
		os.close(descriptor)


def _is_same(
		file: dict[str, MotorCalibration], motors: dict[str, MotorCalibration],
		protocol_version: int) -> bool:
	"""lerobot FeetechMotorsBus.is_calibrated 와 같은 대조."""
	if set(file) != set(motors):
		return False
	is_same_range = all(
		file[name].range_min == value.range_min and file[name].range_max == value.range_max
		for name, value in motors.items())
	if protocol_version == 1:
		return is_same_range
	return is_same_range and all(
		file[name].homing_offset == value.homing_offset for name, value in motors.items())


def check_files(bus: FeetechMotorsBus, calibration_dir: Path) -> dict[str, bool]:
	"""보정 폴더의 <id>.json 마다 (list_calibration_ids 의 것만) 모터와 맞는지.

	모터는 한 번만 읽고 아무것도 쓰지 않는다. 읽을 수 없는 파일은 False.
	"""
	motors = bus.read_calibration()
	result = {}
	for calibration_id in list_calibration_ids(calibration_dir):
		try:
			file = load_calibration_file(Path(calibration_dir) / f'{calibration_id}.json')
		except (OSError, ValueError, TypeError, AttributeError):
			result[calibration_id] = False
			continue
		result[calibration_id] = _is_same(file, motors, bus.protocol_version)
	return result


class CalibrationSession:
	"""새로 보정 한 번. 만들면 토크를 끄고 '제로-포지션 위치 시키기' 를 기다린다.

	바퀴가 아닌 모터 가운데 토크가 켜진 것이 있으면 아무것도 쓰지 않고 CalibrationError.

	한 스레드에서만 부른다. 버스는 부른 쪽이 열고 닫는다 (끝 · 취소 · 오류 뒤).

	Attributes:
		state: waiting_middle → recording → saving → done. 또는 cancelled · error.
		backup_path: finish() 가 남긴 백업 (없으면 None).

	Args:
		bus: open_bus() 로 연 버스.
	"""

	def __init__(self, bus: FeetechMotorsBus) -> None:
		self._bus = bus
		names = list(bus.motors)
		self._homing_motors = [name for name in names if NO_HOMING_KEYWORD not in name]
		self._full_turn_motors = [
			name for name in names if any(keyword in name for keyword in FULL_TURN_KEYWORDS)]
		self._recorded_motors = [name for name in names if name not in self._full_turn_motors]
		self._homings: dict[str, int] = {}
		self._mins: dict[str, int] = {}
		self._maxes: dict[str, int] = {}
		self._is_homing_written = False
		self.backup_path: Path | None = None
		# 다른 것이 팔을 잡고 있으면 (토크 켬) 여기서 토크를 끄면 팔이 떨어진다
		torque = bus.sync_read('Torque_Enable', self._homing_motors, normalize=False)
		holding = [name for name, value in torque.items() if int(value)]
		if holding:
			raise CalibrationError(
				f'팔 토크가 켜져 있습니다 ({", ".join(holding)}). 먼저 팔 힘을 빼세요')
		# 취소하면 되돌릴 값: 아무것도 쓰기 전에 읽는다
		self._previous = bus.read_calibration()
		# lerobot 은 LeKiwi 바퀴 토크를 건드리지 않지만, 보정 중에는 아무 모터도 힘을 내지 않게 한다
		bus.disable_torque()
		for name in self._homing_motors:
			bus.write('Operating_Mode', name, OperatingMode.POSITION.value)
		self.state = STATE_WAITING_MIDDLE

	def middle(self) -> None:
		"""지금 자세를 가운데로: Homing_Offset 을 쓴다 (set_half_turn_homings). → recording."""
		self._expect(STATE_WAITING_MIDDLE, '제로-포지션 위치 시키기')
		self._is_homing_written = True  # 쓰다 실패해도 취소가 되돌리게 먼저 표시한다
		try:
			homings = self._bus.set_half_turn_homings(self._homing_motors)
			positions = self._read_positions()
		except BaseException:
			self.state = STATE_ERROR
			raise
		self._homings = {name: int(homings.get(name, 0)) for name in self._bus.motors}
		self._mins = {name: positions[name] for name in self._recorded_motors}
		self._maxes = dict(self._mins)
		self.state = STATE_RECORDING

	def sample(self) -> dict[str, dict[str, int]]:
		"""지금 tick 을 읽어 최소 · 최대를 넓힌다. 범위를 놓치지 않게 30 Hz 넘게 부른다.

		Returns:
			{motor: {'now', 'min', 'max'}} (모든 모터, 한 바퀴 모터는 0~4095).
		"""
		self._expect(STATE_RECORDING, '범위 재기')
		positions = self._read_positions()
		for name in self._recorded_motors:
			self._mins[name] = min(self._mins[name], positions[name])
			self._maxes[name] = max(self._maxes[name], positions[name])
		return {
			name: {'now': positions[name], 'min': self._range(name)[0], 'max': self._range(name)[1]}
			for name in self._bus.motors}

	def finish(self, calibration_path: Path) -> Path:
		"""보정을 모터에 쓰고 (write_calibration) 파일로 저장한다. → done.

		움직이지 않은 관절이 있으면 아무것도 쓰지 않고 recording 에 남는다 (lerobot 은 여기서 ValueError).
		모터 · 파일 쓰기가 실패하면 예전 보정을 모터에 되돌리고 error.

		Returns:
			저장한 파일 경로.

		Raises:
			CalibrationError: recording 이 아니거나 움직이지 않은 관절이 있다.
		"""
		self._expect(STATE_RECORDING, '끝내기')
		unmoved = [name for name in self._recorded_motors if self._mins[name] == self._maxes[name]]
		if unmoved:
			raise CalibrationError(
				f'움직이지 않은 관절이 있습니다: {", ".join(unmoved)} (끝까지 움직인 뒤 다시 끝내세요)')
		self.state = STATE_SAVING
		calibration = {
			name: MotorCalibration(
				id=motor.id, drive_mode=0, homing_offset=self._homings[name],
				range_min=self._range(name)[0], range_max=self._range(name)[1])
			for name, motor in self._bus.motors.items()}
		try:
			self._bus.write_calibration(calibration)
			self.backup_path = save_calibration_file(
				calibration_path,
				{name: dataclasses.asdict(value) for name, value in calibration.items()})
		except BaseException:
			self.state = STATE_ERROR
			self._restore()
			raise
		self.state = STATE_DONE
		return Path(calibration_path)

	def cancel(self) -> None:
		"""그만둔다. 제로-포지션 위치 시키기로 쓴 값 (Homing_Offset · 범위) 은 예전 것으로 되돌린다. → cancelled.

		done 뒤에는 아무것도 하지 않는다. 되돌리기가 실패하면 error 로 두고 예외를 낸다.
		"""
		if self.state in (STATE_DONE, STATE_CANCELLED):
			return
		try:
			self._restore()
		except BaseException:
			self.state = STATE_ERROR
			raise
		self.state = STATE_CANCELLED

	def _restore(self) -> None:
		# set_half_turn_homings 는 Homing_Offset 과 함께 Min/Max_Position_Limit 도 0~4095 로 지운다
		if self._is_homing_written:
			self._bus.write_calibration(self._previous)
			self._is_homing_written = False

	def _range(self, name: str) -> tuple[int, int]:
		if name in self._full_turn_motors:
			return 0, FULL_TURN_MAX
		return self._mins[name], self._maxes[name]

	def _read_positions(self) -> dict[str, int]:
		positions = self._bus.sync_read('Present_Position', normalize=False)
		return {name: int(value) for name, value in positions.items()}

	def _expect(self, state: str, step: str) -> None:
		if self.state != state:
			raise CalibrationError(f'지금은 {step}를 할 수 없습니다 (상태 {self.state})')
