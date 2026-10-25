"""조작 감독: 조작 권한(lease) · 생존 신호 · 속도 계산 (ROS 없는 순수 로직).

robot.RobotService 가 ROS 타이머(TELEOP_PERIOD)마다 step() 을 불러 내보낼 cmd_vel 을 정한다.
속도는 lekiwi_teleop keyboard_teleop 과 같은 느낌으로 만든다: 단계별 속도,
축마다 LINEAR_STEP · ANGULAR_STEP 씩 목표에 다가가고, 목표가 모두 0 이면 바로 0.
자동 주행 맵 생성도 같은 출력을 쓴다: 번호(start_autonomous)를 받은 탐색기만 set_autonomous 로
목표를 주고, 멈추는 길(stop · 권한 끝 · 직접 조작)은 모두 그 번호를 지운다.
HTTP · WS · ROS 스레드가 함께 부르므로 모든 상태를 잠금 하나로 보호한다.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Final

from lekiwi_web_operator import config
from lekiwi_web_operator.robot_api import CommandResult, error_result, ok_result

ZERO_COMMAND: Final[tuple[float, float, float]] = (0.0, 0.0, 0.0)
# 단계 속도가 상한과 같을 때 부동소수 오차로 빠지지 않게 한다 (lekiwi_teleop 과 같은 값)
LEVEL_TOLERANCE: Final[float] = 1e-9
MULTIPLE_BASES_MESSAGE: Final[str] = (
	'베이스 컨트롤러가 둘 이상입니다. 같은 ROS_DOMAIN_ID 에 다른 로봇이 있습니다'
)


def finite_number(value: Any) -> float | None:
	"""JSON 값이 유한한 숫자면 float, 아니면 None (bool · NaN · 무한대 · 너무 큰 정수)."""
	if isinstance(value, bool) or not isinstance(value, (int, float)):
		return None
	try:
		number = float(value)
	except OverflowError:
		return None
	return number if math.isfinite(number) else None


def is_finite_number(value: Any) -> bool:
	"""bool 이 아닌 유한한 int · float 인지."""
	return finite_number(value) is not None


def blocked_reason(
	is_read_only: bool,
	has_holder: bool,
	is_navigation_active: bool,
	has_base_controller: bool,
	other_publishers: tuple[str, ...],
	base_controller_count: int,
) -> str | None:
	"""조작을 막는 이유를 우선순위 순으로 하나 고른다 (SPEC §7-4).

	Nav2 감독 도구(check_command_graph)처럼 cmd_vel 의 발행자 · 구독자를 본다:
	우리 말고 발행자가 없고, 구독하는 베이스 컨트롤러가 하나뿐이어야 한다.

	Args:
		is_read_only: 보기 전용 게이트웨이인지.
		has_holder: 조작 권한을 가진 화면이 있는지.
		is_navigation_active: 자율 주행이 켜져 있거나 켜지는 중인지.
		has_base_controller: 베이스 컨트롤러 노드가 그래프에 있는지.
		other_publishers: 우리 말고 cmd_vel 을 발행하는 노드들.
		base_controller_count: cmd_vel 을 구독하는 베이스 컨트롤러 수.

	Returns:
		한국어 이유. 조작해도 되면 None.
	"""
	if is_read_only:
		return '보기 전용 모드입니다'
	if not has_holder:
		return '조작 권한이 없습니다'
	if is_navigation_active:
		return '자율 주행 중에는 직접 조작할 수 없습니다'
	if not has_base_controller:
		return '베이스 컨트롤러가 없습니다. 먼저 로봇을 켜세요'
	if base_controller_count > 1:
		return MULTIPLE_BASES_MESSAGE
	if other_publishers:
		return f'다른 프로그램이 cmd_vel 을 보내고 있습니다: {", ".join(other_publishers)}'
	return None


def max_level_for(limits: dict[str, float], pad: str = config.DEFAULT_PAD) -> int:
	"""그 패드 (config.PAD_LIMITS) 가 고를 수 있는 가장 큰 단계.

	lin ≤ min(max_linear_x, max_linear_speed, 패드 앞뒤 상한) 이고 ang ≤ min(max_angular_z, 패드 회전 상한) 인
	단계다: 베이스 상한을 올려도 패드 상한보다 빨라지지 않는다.
	어느 단계도 들지 않으면 0 이다 (그래도 compute_target 이 상한으로 자른다).
	"""
	pad_linear, _pad_linear_y, pad_angular = config.PAD_LIMITS[pad]
	linear_limit = min(
		limits['max_linear_x'], limits['max_linear_speed'], pad_linear,
	) + LEVEL_TOLERANCE
	angular_limit = min(limits['max_angular_z'], pad_angular) + LEVEL_TOLERANCE
	usable = [
		index for index, (linear, angular) in enumerate(config.SPEED_LEVELS)
		if linear <= linear_limit and angular <= angular_limit
	]
	return usable[-1] if usable else 0


def compute_target(
	direction: tuple[float, float, float],
	level: int,
	limits: dict[str, float],
	pad: str = config.DEFAULT_PAD,
) -> tuple[float, float, float]:
	"""방향 입력과 단계로 목표 속도를 만든다 (SPEC §7-6).

	Args:
		direction: (x, y, yaw) 입력. -1..1 로 자르고 (x, y) 길이가 1 넘으면 1 로 줄인다.
		level: SPEED_LEVELS 번호 (0..max_level 로 자른 값).
		limits: max_linear_x · max_linear_y · max_linear_speed · max_angular_z.
		pad: config.PAD_LIMITS 의 키 (옆 방향을 그 패드 상한으로도 자른다).

	Returns:
		상한 안으로 자른 (vx [m/s], vy [m/s], wz [rad/s]).
	"""
	x, y, yaw = (max(-1.0, min(1.0, value)) for value in direction)
	norm = math.hypot(x, y)
	if norm > 1.0:
		x, y = x / norm, y / norm
	linear, angular = config.SPEED_LEVELS[level]
	vx = _clamp(x * linear, limits['max_linear_x'])
	vy = _clamp(y * linear, min(limits['max_linear_y'], config.PAD_LIMITS[pad][1]))
	speed = math.hypot(vx, vy)
	if speed > limits['max_linear_speed']:
		scale = limits['max_linear_speed'] / speed
		vx, vy = vx * scale, vy * scale
	return vx, vy, _clamp(yaw * angular, limits['max_angular_z'])


def _clamp(value: float, limit: float) -> float:
	"""-limit..limit 로 자른다."""
	return max(-limit, min(limit, value))


def _approach(output: float, target: float, step: float) -> float:
	"""output 을 target 쪽으로 step 만큼 옮긴다 (넘지 않는다)."""
	if target > output:
		return min(target, output + step)
	return max(target, output - step)


@dataclass
class _Lease:
	"""조작 권한 하나."""

	client_id: str
	label: str
	token: int          # 권한마다 새 번호: 토크를 켜는 사이 권한이 바뀐 것을 알아챈다
	since: float        # time.time(), 화면 표시용
	last_seen: float    # clock(), 마지막 생존 신호


class TeleopSupervisor:
	"""조작 권한 · 생존 신호 · 우리가 켠 토크 · cmd_vel 출력.

	권한이 끝나면(놓음 · 신호 끊김 · 넘겨줌) 목표와 출력을 0 으로 하고, 그 권한 동안
	우리가 토크를 켰는지(is_powered)를 돌려준다. 토크를 끄는 일은 robot 이 한다.

	Args:
		clock: 단조 시계 [s]. 시험에서는 가짜 시계를 넣는다.
	"""

	def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
		self._clock = clock
		self._lock = threading.Lock()
		self._lease: _Lease | None = None
		self._next_token = 1
		self._is_powered = False
		self._direction = ZERO_COMMAND
		self._direction_time: float | None = None
		self._level = config.START_LEVEL
		self._limits = dict(config.DEFAULT_SPEED_LIMITS)
		self._limits_source = 'default'
		self._max_levels = {pad: max_level_for(self._limits, pad) for pad in config.PAD_LIMITS}
		self._pad = config.DEFAULT_PAD   # 마지막 방향 입력을 보낸 패드
		self._output = ZERO_COMMAND
		self._zero_since: float | None = -math.inf   # 처음에는 0 도 보내지 않는다
		self._is_zero_required = False   # stop() 뒤 0 입력을 한 번 받기 전까지 방향 입력을 버린다
		# 자동 주행: 번호가 있는 동안 직접 입력 대신 이 (vx, wz) 를 목표로 쓴다
		self._autonomous_token: int | None = None
		self._autonomous_command = (0.0, 0.0)
		self._autonomous_time: float | None = None

	# ── 조작 권한 ──────────────────────────────────────────
	def acquire(self, client_id: str, label: str) -> tuple[CommandResult, dict | None]:
		"""권한 받기 (SPEC §7-1). 비었거나 지금 holder 의 신호가 끊겼을 때만 성공.

		Returns:
			(결과, 끝난 권한). 신호가 끊긴 권한을 넘겨받았으면 그 권한의
			{'client_id', 'label', 'is_powered'} 를 함께 돌려준다 (robot 이 정리한다).
		"""
		with self._lock:
			now = self._clock()
			lease = self._lease
			if lease is not None and lease.client_id == client_id:
				lease.last_seen = now
				return ok_result('조작 권한을 받았습니다'), None
			if lease is not None and now - lease.last_seen <= config.OPERATOR_TIMEOUT:
				return error_result('conflict', f'다른 화면({lease.label})이 조작 중입니다'), None
			ended = self._end_lease_locked()
			self._lease = _Lease(client_id, label, self._next_token, time.time(), now)
			self._next_token += 1
			return ok_result('조작 권한을 받았습니다'), ended

	def release(self, client_id: str) -> dict | None:
		"""client_id 가 holder 면 권한을 끝내고 끝난 권한 정보를, 아니면 None."""
		with self._lock:
			if self._lease is None or self._lease.client_id != client_id:
				return None
			return self._end_lease_locked()

	def end_lease(self) -> dict | None:
		"""누가 가졌든 권한을 끝낸다 (게이트웨이 종료)."""
		with self._lock:
			return self._end_lease_locked()

	def expire(self) -> dict | None:
		"""holder 의 마지막 신호가 OPERATOR_TIMEOUT 보다 오래면 권한을 끝내고 그 정보를."""
		with self._lock:
			lease = self._lease
			if lease is None or self._clock() - lease.last_seen <= config.OPERATOR_TIMEOUT:
				return None
			return self._end_lease_locked()

	def touch(self, client_id: str) -> None:
		"""holder 의 생존 신호 (heartbeat · teleop 마다). 다른 화면이면 무시."""
		with self._lock:
			if self._lease is not None and self._lease.client_id == client_id:
				self._lease.last_seen = self._clock()

	def is_operator(self, client_id: str) -> bool:
		"""client_id 가 지금 holder 인지."""
		with self._lock:
			return self._lease is not None and self._lease.client_id == client_id

	def holder(self) -> dict | None:
		"""status.control.holder: {'client_id', 'label', 'since'} 또는 None."""
		with self._lock:
			lease = self._lease
			if lease is None:
				return None
			return {'client_id': lease.client_id, 'label': lease.label, 'since': lease.since}

	def lease_token(self, client_id: str) -> int | None:
		"""client_id 가 holder 면 그 권한 번호, 아니면 None."""
		with self._lock:
			if self._lease is None or self._lease.client_id != client_id:
				return None
			return self._lease.token

	# ── 우리가 켠 토크 ─────────────────────────────────────
	def mark_powered(self, token: int) -> bool:
		"""토크를 켠 권한(token)이 아직 그대로면 is_powered 를 켠다. 바뀌었으면 False."""
		with self._lock:
			if self._lease is None or self._lease.token != token:
				return False
			self._is_powered = True
			return True

	def take_powered(self) -> bool:
		"""is_powered 를 끄고 이전 값을 돌려준다. True 면 부른 쪽이 토크를 끈다."""
		with self._lock:
			is_powered = self._is_powered
			self._is_powered = False
			return is_powered

	def is_powered_by_us(self) -> bool:
		"""지금 권한으로 우리가 토크를 켰는지."""
		with self._lock:
			return self._is_powered

	# ── 속도 ───────────────────────────────────────────────
	def set_input(
		self, client_id: str, x: float, y: float, yaw: float, level: int,
		pad: str = config.DEFAULT_PAD,
	) -> bool:
		"""holder 의 방향 입력. 잘못된 값은 버린다 (명령이 끊긴 것과 같아 곧 0 이 된다).

		pad (config.PAD_LIMITS 의 키) 가 단계 · 옆 방향 상한을 정한다. 모르는 패드면 버린다.

		stop() 뒤에는 0 입력을 한 번 받을 때까지 0 이 아닌 입력을 버린다: 비상 정지 · 토크 끄기
		직후 누르고 있던 키가 다시 로봇을 움직이지 않게 한다 (키를 떼면 화면이 0 을 보낸다).
		자동 주행 중의 0 이 아닌 입력은 자동 주행을 끝내고 (stop() 과 같다) 버린다.

		Returns:
			이 입력이 자동 주행을 끝냈으면 True (robot 이 이유를 알린다).
		"""
		if not all(is_finite_number(value) for value in (x, y, yaw)):
			return False
		if not isinstance(level, int) or isinstance(level, bool):
			return False
		if pad not in config.PAD_LIMITS:
			return False
		with self._lock:
			if self._lease is None or self._lease.client_id != client_id:
				return False
			if self._autonomous_token is not None and (x, y, yaw) != ZERO_COMMAND:
				# 누르던 키는 뗐다가 다시 눌러야 움직인다 (같은 입력이 곧바로 몰지 않게)
				self._stop_locked()
				self._is_zero_required = True
				return True
			if self._is_zero_required:
				if (x, y, yaw) != ZERO_COMMAND:
					return False
				self._is_zero_required = False
			self._direction = (float(x), float(y), float(yaw))
			self._direction_time = self._clock()
			self._level = max(0, level)
			self._pad = pad
			return False

	def stop(self) -> None:
		"""목표와 출력을 바로 0 으로 (비상 정지 · 토크 끄기 · 자율 주행 시작 · 종료).

		자동 주행 번호도 지운다. 그 뒤 holder 가 0 입력을 한 번 보내기 전까지 방향 입력을 버린다.
		"""
		with self._lock:
			self._stop_locked()
			self._is_zero_required = True

	# ── 자동 주행 (탐색 스레드) ────────────────────────────
	def start_autonomous(self, client_id: str) -> int | None:
		"""자동 주행 번호를 새로 만든다. client_id 가 holder 가 아니면 None (그사이 권한이 바뀌었다).

		목표는 set_autonomous 가 줄 때까지 0 이다. 번호는 권한이 끝나면 함께 지워진다.
		"""
		with self._lock:
			if self._lease is None or self._lease.client_id != client_id:
				return None
			self._autonomous_token = self._next_token
			self._next_token += 1
			self._autonomous_command, self._autonomous_time = (0.0, 0.0), None
			return self._autonomous_token

	def is_autonomous(self, token: int) -> bool:
		"""token 이 아직 살아 있는 자동 주행 번호인지 (멈추는 길은 모두 번호를 지운다)."""
		with self._lock:
			return self._autonomous_token is not None and token == self._autonomous_token

	def set_autonomous(self, token: int, vx: float, wz: float) -> bool:
		"""탐색기의 목표 속도. 번호가 지금 것일 때만 받는다.

		전진만 (0 ≤ vx ≤ EXPLORE_SPEED, 옆으로는 받지 않는다), |wz| ≤ EXPLORE_TURN_SPEED 로 자른다.
		TELEOP_COMMAND_TIMEOUT 안에 다시 주지 않으면 step() 이 0 을 낸다.

		Returns:
			받았으면 True.
		"""
		if not (is_finite_number(vx) and is_finite_number(wz)):
			return False
		with self._lock:
			if self._autonomous_token is None or token != self._autonomous_token:
				return False
			self._autonomous_command = (
				max(0.0, min(float(vx), config.EXPLORE_SPEED)),
				_clamp(float(wz), config.EXPLORE_TURN_SPEED),
			)
			self._autonomous_time = self._clock()
			return True

	def stop_autonomous(self, token: int) -> None:
		"""token 이 아직 살아 있으면 stop() 과 같다. 이미 끝났으면 (직접 조작 중일 수 있다) 그대로 둔다."""
		with self._lock:
			if self._autonomous_token is not None and token == self._autonomous_token:
				self._stop_locked()
				self._is_zero_required = True

	def output(self) -> tuple[float, float, float]:
		"""지금 내보내는 (vx, vy, wz) (램프 · 상한을 지난 값)."""
		with self._lock:
			return self._output

	def set_limits(self, limits: dict[str, Any] | None) -> bool:
		"""베이스 컨트롤러에서 읽은 상한을 쓴다. 없거나 잘못됐으면 기본값 (SPEC §7-8).

		Returns:
			읽은 값을 썼으면 True, config.DEFAULT_SPEED_LIMITS 로 돌아갔으면 False.
		"""
		names = tuple(config.DEFAULT_SPEED_LIMITS)
		is_valid = limits is not None and all(
			is_finite_number(limits.get(name)) and limits[name] > 0.0 for name in names
		)
		with self._lock:
			if is_valid:
				self._limits = {name: float(limits[name]) for name in names}
				self._limits_source = 'base_controller'
			else:
				self._limits = dict(config.DEFAULT_SPEED_LIMITS)
				self._limits_source = 'default'
			self._max_levels = {pad: max_level_for(self._limits, pad) for pad in config.PAD_LIMITS}
		return is_valid

	def step(self, is_allowed: bool) -> tuple[float, float, float] | None:
		"""TELEOP_PERIOD 마다 한 번: 이번에 보낼 cmd_vel, 보내지 않으면 None (SPEC §7-5~7).

		Args:
			is_allowed: 지금 조작해도 되는지 (blocked_reason 이 None).

		Returns:
			(vx, vy, wz). 출력이 0 이 아니거나 0 이 된 뒤 ZERO_HOLD 동안만 값을 돌려준다.
		"""
		with self._lock:
			now = self._clock()
			is_autonomous = self._autonomous_token is not None
			# 자동 주행 번호가 있는 동안은 직접 입력을 보지 않는다
			command_time = self._autonomous_time if is_autonomous else self._direction_time
			is_fresh = (
				command_time is not None and now - command_time <= config.TELEOP_COMMAND_TIMEOUT
			)
			target = ZERO_COMMAND
			if is_allowed and self._lease is not None and is_fresh and is_autonomous:
				vx, wz = self._autonomous_command
				limits = self._limits
				vx = min(vx, limits['max_linear_x'], limits['max_linear_speed'])
				target = (vx, 0.0, _clamp(wz, limits['max_angular_z']))
			elif is_allowed and self._lease is not None and is_fresh:
				level = min(self._level, self._max_levels[self._pad])
				target = compute_target(self._direction, level, self._limits, self._pad)
			if target == ZERO_COMMAND:
				self._output = ZERO_COMMAND
			else:
				steps = (config.LINEAR_STEP, config.LINEAR_STEP, config.ANGULAR_STEP)
				self._output = tuple(
					_approach(output, goal, step)
					for output, goal, step in zip(self._output, target, steps)
				)
			if self._output != ZERO_COMMAND:
				self._zero_since = None
				return self._output
			if self._zero_since is None:
				self._zero_since = now
			return self._output if now - self._zero_since <= config.ZERO_HOLD else None

	def status(self, is_allowed: bool, reason: str | None) -> dict:
		"""status.teleop (SPEC §6). max_level · levels 는 기본 패드 ('mapping') 것이고, pads 에 패드마다
		{max_level, levels, max_linear_y} 가 있다. level 은 마지막 입력의 패드로 자른 값."""
		with self._lock:
			vx, vy, wz = self._output
			max_level = self._max_levels[config.DEFAULT_PAD]
			return {
				'is_allowed': is_allowed,
				'blocked_reason': reason,
				'is_active': self._output != ZERO_COMMAND,
				'command': {'vx': round(vx, 3), 'vy': round(vy, 3), 'wz': round(wz, 3)},
				'pad': self._pad,
				'level': min(self._level, self._max_levels[self._pad]),
				'max_level': max_level,
				'levels': [list(level) for level in config.SPEED_LEVELS[:max_level + 1]],
				'pads': {
					pad: {
						'max_level': top,
						'levels': [list(level) for level in config.SPEED_LEVELS[:top + 1]],
						'max_linear_y': min(self._limits['max_linear_y'], config.PAD_LIMITS[pad][1]),
					}
					for pad, top in self._max_levels.items()
				},
				'limits': {**self._limits, 'source': self._limits_source},
			}

	def _end_lease_locked(self) -> dict | None:
		"""권한을 끝내고 {'client_id', 'label', 'is_powered'} 를 돌려준다 (잠금 안)."""
		lease = self._lease
		if lease is None:
			return None
		ended = {'client_id': lease.client_id, 'label': lease.label, 'is_powered': self._is_powered}
		self._lease = None
		self._is_powered = False
		self._stop_locked()
		return ended

	def _stop_locked(self) -> None:
		"""목표 · 출력 0 · 자동 주행 번호 지움. 움직이던 중이었으면 ZERO_HOLD 동안 0 을 보낸다 (잠금 안)."""
		self._direction = ZERO_COMMAND
		self._direction_time = None
		self._autonomous_token = None
		self._autonomous_command, self._autonomous_time = (0.0, 0.0), None
		if self._output != ZERO_COMMAND:
			self._output = ZERO_COMMAND
			self._zero_since = self._clock()
