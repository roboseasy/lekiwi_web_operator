"""RobotService: RobotApi 구현 (ROS 노드 + 조작 감독 + 기능 관리 조립).

스레드:
- ROS 스레드 하나가 executor 를 돌린다. 조작 타이머(holder 가 있으면 TELEOP_PERIOD 20 Hz,
  없으면 IDLE_TELEOP_PERIOD 2 Hz), 그래프 감시(GRAPH_PERIOD, 1 Hz), 자율 주행 속도 다시 보내기
  (NAV_SPEED_PERIOD)도 여기서 돈다.
- HTTP · WS 스레드는 이 클래스의 메서드를 부른다. ROS 일은 node.post() 로 넘기고,
  결과가 필요하면 concurrent.futures 로 제한 시간 동안 기다린다.
- 자동 주행 맵 생성 중에는 탐색 스레드 하나가 EXPLORE_CONTROL_PERIOD 마다 explore.Explorer.step()
  을 부르고 그 (vx, wz) 를 조작 감독(set_autonomous)에 준다. 잠금으로 지킨 마지막 값만 읽고 ROS 스레드를
  기다리지 않는다 (계획 한 번이 Pi 에서 수백 ms 걸려도 cmd_vel 발행 · 비상 정지가 밀리지 않게).
  안전은 시간 약속에 기댄다: 충돌 예측 EXPLORE_HORIZON ≥ 스캔 나이 + 명령 유효 시간 + 주기 + 베이스 반응.
안전 규칙은 SPEC §7 을 따른다: 조작 권한 · 생존 신호 · 명령 신선도, 그리고 Nav2 감독 도구와의
약속(자율 주행 중에는 cmd_vel 퍼블리셔를 갖지 않고, cmd_vel 은 절대 구독하지 않는다).
비상 정지는 걸린 채로 남는다: holder 가 다시 토크 켜기나 자율 주행 시작에 성공할 때까지 조작
노드를 내려 Nav2 감독 도구가 토크를 켜지 못하게 하고, 그 직후(ESTOP_HOLDOFF) 온 토크 켜기 ·
자율 주행 시작 · 팔 시작은 거절한다 (SPEC §7-10 · §7-11 을 넘어선다. 이미 오고 있던 요청이 이기지 않게).
비상 정지는 바퀴 다음에 팔 토크도 끈다 (편 팔은 떨어진다, 사용자 결정).
팔 원격 조작 세션(arm_start)은 그 조작 권한 번호 하나다. 권한 끝 · 비상 정지 · 팔 멈춤 · 팔 힘 빼기 ·
자율 주행이 켜짐 · 팔로워 · 드라이버 팔 쓰기가 사라짐이 세션을 닫는다. 비상 정지 · 팔 힘 빼기만 팔 토크를
끄고, 나머지는 켠 채 둔다 (드라이버가 마지막 목표를 유지해 팔이 떨어지지 않는다). 세션 중에는 자율 주행 ·
자동 주행 맵 생성 시작을 거절한다. 팔 토크 켜기는 비상 정지 번호를 본 그 _lock 안에서 ROS 큐에 넣는다:
그 뒤의 비상 정지가 넣는 끄기는 같은 큐 · 같은 서비스 클라이언트로 늘 그 뒤에 간다.
ROS 는 start() 에서 처음 불러와, ROS 없이도 이 모듈을 import 할 수 있다.
"""

import concurrent.futures
import logging
import math
import socket
import threading
import time
from typing import TYPE_CHECKING, Any, Callable, Final

from lekiwi_web_operator import __version__, config, explore, map_files
from lekiwi_web_operator.follower_calibration import FollowerCalibration
from lekiwi_web_operator.processes import ACTIVE_STATES, ProcessManager, make_event
from lekiwi_web_operator.robot_api import CommandResult, error_result, ok_result
from lekiwi_web_operator.teleop import (
	MULTIPLE_BASES_MESSAGE,
	ZERO_COMMAND,
	TeleopSupervisor,
	blocked_reason,
	finite_number,
	is_finite_number,
)

if TYPE_CHECKING:
	from lekiwi_web_operator.ros_node import OperatorNode

logger = logging.getLogger(__name__)

GRAPH_PERIOD: Final[float] = 1.0                 # [s] 노드 그래프 감시
IDLE_TELEOP_PERIOD: Final[float] = 0.5           # [s] holder 가 없을 때 조작 타이머 (Pi CPU)
SPIN_TIMEOUT: Final[float] = 0.1                 # [s] executor 한 번 기다리는 시간
ROS_CALL_TIMEOUT: Final[float] = 2.0             # [s] ROS 스레드에 넘긴 짧은 일의 응답 대기
ROS_STOP_TIMEOUT: Final[float] = 2.0             # [s] 종료할 때 ROS 스레드를 기다리는 시간
READY_POLL_PERIOD: Final[float] = 0.05           # [s] lekiwi/motor/ready 확인 간격
NAVIGATION_READY_OFF_TIMEOUT: Final[float] = 3.0  # [s] 자율 주행 전 ready false 대기 (§7-12)
# [s] 토크 켜기 한 번의 최대 시간 (서비스 찾기 + 켜기 응답 + ready 대기 + 실패 시 끄기 응답)
POWER_ON_MAX_TIME: Final[float] = (
	config.MOTOR_SERVICE_DISCOVERY_TIMEOUT + 2 * config.MOTOR_SERVICE_TIMEOUT
	+ config.MOTOR_READY_TIMEOUT
)
LIMITS_RETRY_PERIOD: Final[float] = 3.0          # [s] 속도 상한 응답이 없으면 다시 묻는다
STALE_SENSOR_AGE: Final[float] = 1.0             # [s] 노드가 있는데 이만큼 안 오면 경고
MOTOR_HOT_TEMPERATURE: Final[float] = 60.0       # [degC]
NAV2_VELOCITY_NODE: Final[str] = 'collision_monitor'   # 자율 주행 중 cmd_vel 의 정상 발행자
NAVIGATION_UP_STATES: Final[tuple[str, ...]] = ('starting', 'running', 'external')   # 끄는 중 제외
# 감독 도구를 뺀 자율 주행 핵심 노드 (bt_navigator · amcl)
NAV2_CORE_NODES: Final[frozenset[str]] = (
	frozenset(config.PROCESS_KEY_NODES['navigation']) - {config.NAV2_MOTOR_GUARD_NODE}
)
# [s] 감독 도구가 이만큼 안 보이고 Nav2 노드는 남아 있어야 끝났다고 알린다
# (터미널에서 Ctrl+C 로 끄면 감독 도구가 Nav2 노드보다 조금 먼저 사라진다)
GUARD_GONE_DELAY: Final[float] = 3.0
STALE_CHECKS: Final[tuple[tuple[str, str, str], ...]] = (   # (토픽, 발행 노드, 경고)
	('scan', 'lekiwi_scan_filter_node', '스캔이 1초 넘게 들어오지 않습니다'),
	('odom', config.BASE_CONTROLLER_NODE, 'odom 이 1초 넘게 들어오지 않습니다'),
	('imu', 'lekiwi_imu_node', 'IMU 가 1초 넘게 들어오지 않습니다'),
	('motor_states', 'lekiwi_motor_driver_node', '모터 상태가 1초 넘게 들어오지 않습니다'),
)
# 그 채널을 보는 화면이 있을 때만 구독하는 묶음 (ros_node.STREAMS 와 같은 이름).
# 'map' 은 pose.map · is_localized (tf2_echo), 'motors' 는 모터 상태 · 전원,
# 'arm' 은 status.teleop_arm 의 arm_ready · follower · joints
STREAM_CHANNELS: Final[tuple[str, ...]] = (
	'scan', 'odom', 'motors', 'imu', 'map', 'arm', 'calibration',
	*(f'camera/{name}' for name in config.CAMERA_TOPICS),
)
# 'calibration' (보정 탭) 은 팔 보정 레지스터와 함께 모터 상태 (접은 자세 tick) 도 본다
CALIBRATION_STREAMS: Final[frozenset[str]] = frozenset({'calibration', 'motors'})
READ_ONLY_MESSAGE: Final[str] = '보기 전용 모드라 할 수 없습니다'
NOT_OPERATOR_MESSAGE: Final[str] = '조작 권한이 필요합니다. 먼저 조작 권한을 받으세요'
LEASE_LOST_MESSAGE: Final[str] = '조작 화면 신호가 끊겨 로봇을 멈췄습니다'
NO_NAVIGATION_MESSAGE: Final[str] = 'nav2_msgs 가 없어 자율 주행을 쓸 수 없습니다'
ESTOP_HOLDOFF_MESSAGE: Final[str] = '비상 정지 직후입니다. 잠시 뒤 다시 누르세요'
ESTOPPED_MESSAGE: Final[str] = '그사이 비상 정지가 눌렸습니다'
POWER_OFF_UNREACHABLE_MESSAGE: Final[str] = (
	'바퀴 토크 끄기 실패: lekiwi/motor/power 서비스를 찾지 못했습니다. 로봇이 움직이면 주 전원을 끄세요'
)
GUARD_GONE_MESSAGE: Final[str] = (
	'자율 주행 감독 도구가 끝나 바퀴 토크를 다시 켤 수 없습니다. '
	'자율 주행을 끄고 다시 켜세요 (로그: navigation)'
)
UNKNOWN_SCREEN_LABEL: Final[str] = '연결 안 된 화면'
# ── 자동 주행 맵 생성 ──
RUNNING_STATES: Final[tuple[str, ...]] = ('running', 'external')
# 화면이 원하지 않아도 자동 주행 동안 구독하는 묶음 (스캔 · odom · 지도 위 자세)
EXPLORE_STREAMS: Final[frozenset[str]] = frozenset({'scan', 'odom', 'map'})
EXPLORE_JOIN_TIMEOUT: Final[float] = 1.0   # [s] 종료할 때 탐색 스레드를 기다리는 시간
EXPLORE_SAVE_RETRY: Final[float] = 1.0     # [s] 다른 지도 저장이 도는 동안 자동 저장을 다시 해 보는 간격
EXPLORE_SAVE_POLL: Final[float] = 0.2      # [s] 자동 저장 결과를 보는 간격
# [s] ProcessManager 가 지도 저장을 끝내기까지 가장 오래 (시간 제한 → SIGINT → SIGTERM → SIGKILL)
SAVE_MAP_MAX_TIME: Final[float] = (
	config.SAVE_MAP_TIMEOUT + config.PROCESS_STOP_TIMEOUT['save_map'] + config.PROCESS_TERM_TIMEOUT
)
IDLE_EXPLORATION: Final[dict] = {
	'state': 'idle', 'reason': None, 'started_at': None, 'elapsed': None, 'explored_area': None,
	'frontiers': None, 'target': None, 'path': None, 'saved_map': None,
}
EXPLORE_STARTED_MESSAGE: Final[str] = '자동 주행 맵 생성을 시작했습니다'
EXPLORING_MESSAGE: Final[str] = '자동 주행 맵 생성 중입니다'
POWER_FIRST_MESSAGE: Final[str] = '먼저 ‘조작 켜기’를 누르세요'
# 끝난 이유 (status.exploration.reason · 이벤트)
EXPLORE_STOPPED_MESSAGE: Final[str] = '자동 주행을 멈췄습니다'
USER_STOP_MESSAGE: Final[str] = '멈춤을 눌러 자동 주행을 멈췄습니다'
TAKEOVER_MESSAGE: Final[str] = '직접 조작해 자동 주행을 멈췄습니다'
ESTOP_EXPLORE_MESSAGE: Final[str] = '비상 정지로 자동 주행을 멈췄습니다'
POWER_OFF_EXPLORE_MESSAGE: Final[str] = '주행을 꺼서 자동 주행을 멈췄습니다'
READY_LOST_MESSAGE: Final[str] = '바퀴 토크가 꺼져 자동 주행을 멈췄습니다'
MAPPING_STOP_MESSAGE: Final[str] = '지도 작성을 꺼서 자동 주행을 멈췄습니다'
MAPPING_GONE_MESSAGE: Final[str] = '지도 작성이 꺼져 자동 주행을 멈췄습니다'
NAVIGATION_EXPLORE_MESSAGE: Final[str] = '자율 주행을 시작해 자동 주행을 멈췄습니다'
SHUTDOWN_EXPLORE_MESSAGE: Final[str] = '게이트웨이를 꺼서 자동 주행을 멈췄습니다'
EXPLORER_ERROR_MESSAGE: Final[str] = '탐색기 오류로 자동 주행을 멈췄습니다 (서버 로그를 확인하세요)'
# ── 팔 원격 조작 ──
# [s] 팔 시작 한 번의 최대 시간 (서비스 찾기 + 켜기 응답 + 비상 정지에 진 켜기를 되끄는 응답)
ARM_STATUS_WAIT: Final[float] = 2.0   # [s] 팔 시작이 'arm' 묶음을 구독하고 팔로워 상태를 기다리는 시간
# (상태 기다림 + 앞 비상 정지의 팔 끄기 기다림 + 서비스 찾기 + 켜기 응답 + 비상 정지에 진 켜기를 되끄는 응답)
ARM_START_MAX_TIME: Final[float] = (
	ARM_STATUS_WAIT + 2 * config.MOTOR_SERVICE_DISCOVERY_TIMEOUT + 3 * config.MOTOR_SERVICE_TIMEOUT
)
ARM_STREAMS: Final[frozenset[str]] = frozenset({'arm'})   # 팔 시작 · 세션 동안 화면과 상관없이 구독
ARM_STARTED_MESSAGE: Final[str] = '팔 원격 조작을 시작했습니다 (팔 토크 켬)'
ARM_STOPPED_MESSAGE: Final[str] = '팔 원격 조작을 멈췄습니다. 팔은 토크를 켠 채 그 자리를 유지합니다'
# 권한 끝 · 자율 주행이 세션을 닫았다 ({reason})
ARM_HELD_MESSAGE: Final[str] = (
	'팔 원격 조작을 멈췄습니다 ({reason}). 팔은 토크를 켠 채 그 자리를 유지합니다'
)
ARM_RELAXED_MESSAGE: Final[str] = '팔 힘을 뺐습니다 (팔 토크 끔)'
ARM_SESSION_MESSAGE: Final[str] = '팔 원격 조작 중입니다. 먼저 ‘팔 멈춤’을 누르세요'
ARM_INTERRUPTED_MESSAGE: Final[str] = '그사이 ‘팔 멈춤’ · ‘팔 힘 빼기’가 눌렸습니다'
LEADER_LOST_MESSAGE: Final[str] = (
	f'리더 값이 {config.ARM_LEADER_TIMEOUT:g}초 넘게 오지 않아 팔 목표를 그쳤습니다 (팔은 그 자리를 유지합니다)'
)
ARM_TORQUE_MISSING_MESSAGE: Final[str] = (
	f'비상 정지: {config.ARM_TORQUE_SERVICE} 서비스가 없어 팔 토크를 끄지 못했습니다 '
	'(드라이버 팔 쓰기가 꺼져 있습니다). 팔 토크가 켜져 있으면 주 전원을 끄세요'
)
# 드라이버는 끝날 때 팔 토크를 그대로 둔다: 이 게이트웨이가 켠 뒤 끄기를 확인하지 못했으면 굳어 있을 수 있다
ARM_DRIVER_GONE_MESSAGE: Final[str] = (
	'비상 정지: 팔 토크를 끄지 못했습니다 (모터 드라이버 없음). 팔이 굳어 있으면 주 전원을 끄세요'
)
ARM_OFF_MESSAGE: Final[str] = '팔 토크를 껐습니다'
# 자율 주행 · 자동 주행 맵 생성을 세션 없이 팔 토크가 켜진 채 시작함 (막지 않는다, 사용자 결정)
ARM_TORQUED_MESSAGE: Final[str] = (
	'팔 토크가 켜져 있습니다. 팔을 접었는지 확인하세요 (자율 주행 · 자동 주행 맵 생성은 접은 팔 모양만 압니다)'
)
# 로봇(bringup)을 끄기 전 팔 접기 (status.teleop_arm.rest)
IDLE_ARM_REST: Final[dict] = {'state': 'idle', 'reason': None, 'time': None}
ARM_RESTING_MESSAGE: Final[str] = '로봇을 끄기 전에 팔을 접습니다'
ARM_RESTED_MESSAGE: Final[str] = '팔을 접고 팔 토크를 껐습니다'
ARM_FORCE_MESSAGE: Final[str] = '팔을 접지 않고 팔 힘을 뺀 뒤 로봇을 끕니다 (편 팔은 떨어집니다)'
ARM_DRIVER_GONE_STOP_MESSAGE: Final[str] = (
	'모터 드라이버가 없어 팔을 접거나 팔 토크를 끄지 못한 채 로봇을 끕니다. 팔이 굳어 있으면 주 전원을 끄세요'
)
ARM_FORCE_HINT: Final[str] = '팔을 손으로 받치고 ‘그래도 끄기’로 팔 힘을 뺀 뒤 끌 수 있습니다'


def _is_status_fresh(arm: dict) -> bool:
	"""OperatorNode.snapshot()['arm'] 의 팔로워 상태가 ARM_STATUS_MAX_AGE 안에 왔는지."""
	age = arm['status_age']
	return age is not None and age <= config.ARM_STATUS_MAX_AGE


def _step_status(step: explore.Step) -> dict:
	"""Step → status.exploration 값. numpy 수도 JSON 으로 보낼 수 있는 파이썬 값으로 바꾼다
	(np.int64 하나가 상태 전송 전체를 막지 않게)."""
	target = step.target
	return {
		'state': step.state,
		'reason': step.reason,
		'explored_area': (
			None if step.explored_area is None else round(float(step.explored_area), 2)
		),
		'frontiers': None if step.frontiers is None else int(step.frontiers),
		'target': None if target is None else {
			'x': round(float(target[0]), 3), 'y': round(float(target[1]), 3),
		},
		'path': None if step.path is None else [round(float(value), 3) for value in step.path],
	}
LOG_LEVELS: Final[dict[str, int]] = {
	'info': logging.INFO, 'warn': logging.WARNING, 'error': logging.ERROR,
}


class RobotService:
	"""RobotApi 구현. 생성자는 ROS 를 시작하지 않는다 (start() 에서).

	잠금 순서: _arm_lock → _lease_lock → _lock → (리스너인 hub 의 잠금). _lock 을 잡은 채로 ROS 를
	기다리지 않는다 (ROS 큐에 넣기만 한다).

	자동 주행은 조작 감독의 번호(start_autonomous) 하나로 돈다. 멈추는 길(비상 정지 · 주행 끄기 · 권한 끝 ·
	직접 조작 · 지도 작성 끔 · 자율 주행 시작 · 멈춤 · 종료)은 모두 그 번호를 지운다. 끝난 이유는 번호를
	지우기 전에, 또는 번호를 지운 그 _lease_lock 안에서 _note_explore_end 로 적는다 (탐색 스레드는 번호가
	없어진 것을 본 뒤 _lease_lock 안에서 이유를 읽는다).

	Args:
		is_read_only: True 면 구조적으로 퍼블리셔 · 서비스 · 액션 클라이언트 · 조작 노드를
			만들지 않고, 명령은 모두 read_only 로 거부한다.
	"""

	def __init__(self, is_read_only: bool) -> None:
		self._is_read_only = is_read_only
		self._lock = threading.Lock()
		self._listeners: list[Callable[[dict], None]] = []
		self._teleop = TeleopSupervisor()
		self._processes = ProcessManager(self._dispatch)
		self._features = {
			'has_motor_msgs': False,
			'has_navigation': False,
			'can_control_processes': not is_read_only,
		}
		self._node: 'OperatorNode | None' = None
		self._executor: Any = None
		self._thread: threading.Thread | None = None
		self._is_spinning = False
		self._domain_id: int | None = None
		self._allowed: tuple[bool, str | None] = (
			False, blocked_reason(is_read_only, False, False, False, (), 0),
		)
		# ROS 스레드만 쓰는 것
		self._teleop_timer: Any = None
		self._has_base_controller = False
		self._is_limits_stale = False
		self._limits_requested_at: float | None = None
		# 자율 주행 감독 도구: 이번 자율 주행에서 보였는지 · 보였다가 안 보이기 시작한 때
		# (time.monotonic()) · 끝났다고 알렸는지 (ROS 스레드가 쓴다)
		self._has_seen_guard = False
		self._guard_missing_since: float | None = None
		self._is_guard_gone = False
		# self._lock
		self._stream_wanted = {stream: False for stream in STREAM_CHANNELS}
		self._stream_changed_at = {stream: -math.inf for stream in STREAM_CHANNELS}
		self._power_offs: list[concurrent.futures.Future] = []   # 응답을 기다리는 토크 끄기
		self._arm_offs: list[concurrent.futures.Future] = []     # 그중 비상 정지의 팔 토크 끄기
		self._estop_seq = 0             # 비상 정지마다 +1 (진행 중인 토크 켜기 · 자율 주행 시작이 본다)
		self._estop_at = -math.inf      # time.monotonic()
		self._estop: dict | None = None   # 걸린 비상 정지 {'since', 'label'}
		self._is_preparing_navigation = False
		self._navigation_speed = config.NAV_SPEED_DEFAULT   # [%] Nav2 최고 속도 대비
		self._screen_streams: frozenset[str] = frozenset()   # (self._lock) 화면이 원하는 묶음
		# 자동 주행 (self._lock): 움직이는 동안의 번호 (끝나면 None) · 끝낸 쪽이 적은 이유 · status
		self._explore_token: int | None = None
		self._explore_reason: str | None = None
		self._exploration = dict(IDLE_EXPLORATION)
		# 팔 원격 조작 (self._lock): 세션을 연 조작 권한 번호 (없으면 None) · 마지막 리더 값을 받은
		# time.monotonic() · 지금 lekiwi/arm/goal 을 보내는지 · 마지막 비상 정지의 팔 토크 끄기 결과 ·
		# 팔 멈춤 · 힘 빼기마다 +1 (도는 팔 시작이 본다) · 팔 시작 중 · 이 게이트웨이가 켠 뒤 끄기를 확인하지 못했다
		self._arm_session: int | None = None
		self._leader_at: float | None = None
		self._is_relaying = False
		self._estop_arm: dict | None = None
		self._arm_stop_seq = 0
		self._is_arm_starting = False
		self._arm_rest = dict(IDLE_ARM_REST)   # 로봇을 끄기 전 팔 접기
		# shortcut: 이 게이트웨이를 켜기 전에 켜진 팔 토크는 모른다, 알아야 하면 arm_ready · 모터 상태로 채운다
		self._is_arm_maybe_on = False
		self._arm_lock = threading.Lock()   # 팔 시작 · 힘 빼기의 서비스 호출을 하나씩
		# 시작부터 탐색 스레드가 끝날 때까지 잡는다 (두 번 눌러도 스레드는 하나)
		self._explore_lock = threading.Lock()
		self._explore_stop = threading.Event()   # 종료
		self._explore_thread: threading.Thread | None = None
		# 권한 끝내기와 그 정리(_finish_lease_in_ros) post 를 한 번에: shutdown 이 놓치지 않게
		self._lease_lock = threading.Lock()
		self._motor_lock = threading.Lock()
		self._navigation_lock = threading.Lock()
		self._shutdown_lock = threading.Lock()
		self._is_shut_down = False
		self._calibration = FollowerCalibration(self)   # 보정 탭 (팔로워)

	# ── 수명 ───────────────────────────────────────────────
	def start(self) -> None:
		"""rclpy 를 시작하고 노드 · executor 스레드를 띄운다.

		신호 처리는 __main__ 이 하므로 rclpy 신호 처리기는 설치하지 않는다.

		Raises:
			RuntimeError: ROS 를 불러오거나 시작하지 못했을 때 (한국어 이유).
		"""
		try:
			import rclpy
			from rclpy.executors import SingleThreadedExecutor
			from rclpy.signals import SignalHandlerOptions

			from lekiwi_web_operator import ros_node
		except ImportError as error:
			raise RuntimeError(
				f'ROS 2 를 불러오지 못했습니다 ({error}). '
				'/opt/ros/jazzy/setup.bash 와 워크스페이스를 source 했는지 확인하세요'
			) from error
		try:
			rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
		except Exception as error:
			raise RuntimeError(f'ROS 를 시작하지 못했습니다: {error}') from error
		try:
			node = ros_node.OperatorNode(self._is_read_only, self._emit)
			executor = SingleThreadedExecutor()
			executor.add_node(node)
			# executor 스레드를 띄우기 전이라 여기서 만들어도 ROS 스레드와 겹치지 않는다
			self._teleop_timer = node.create_timer(IDLE_TELEOP_PERIOD, self.teleop_timer_callback)
			node.create_timer(GRAPH_PERIOD, self.graph_timer_callback)
			node.create_timer(config.NAV_SPEED_PERIOD, self.speed_timer_callback)
		except Exception as error:
			rclpy.try_shutdown()
			raise RuntimeError(f'ROS 노드를 만들지 못했습니다: {error}') from error
		self._features['has_motor_msgs'] = ros_node.HAS_MOTOR_MSGS
		self._features['has_navigation'] = ros_node.HAS_NAVIGATION
		self._domain_id = node.context.get_domain_id()
		self._node, self._executor = node, executor
		self._is_spinning = True
		self._thread = threading.Thread(target=self._spin, name='ros', daemon=True)
		self._thread.start()
		logger.info(
			'ROS 시작: domain %s, 노드 %s, 보기 전용 %s, 모터 메시지 %s, 자율 주행 %s',
			self._domain_id, node.get_name(), self._is_read_only, ros_node.HAS_MOTOR_MSGS,
			ros_node.HAS_NAVIGATION,
		)

	def shutdown(self) -> None:
		"""(막힘) 정지 · 자동 주행 끝냄 · 우리가 켠 토크 끔 · 목표 취소 · 조작 노드 삭제 · 웹에서 켠
		launch 끄기.

		여러 번 불러도 안전하다 (두 번째부터는 첫 번째가 끝나기를 기다렸다가 돌아온다).
		"""
		with self._shutdown_lock:
			if self._is_shut_down:
				return
			self._is_shut_down = True
			self._note_explore_end(SHUTDOWN_EXPLORE_MESSAGE)   # 번호는 바로 아래 stop() 이 지운다
			self._calibration.shutdown()   # 보정 프로그램을 취소 · 끝낸다 (로봇 기능은 다시 켜지 않는다)
			self._explore_stop.set()
			# _lease_lock: WS 스레드가 막 끝낸 권한의 정리(토크 끄기)가 이 정리보다 먼저 큐에 들어간다
			with self._lease_lock:
				ended = self._teleop.end_lease()
				self._teleop.stop()
				with self._lock:
					self._arm_stop_seq += 1
				self._clear_arm_session()   # 팔 토크는 아래에서 접은 뒤에만 끈다
				is_powered = ended is not None and ended['is_powered']
				future = None
				if self._node is not None:
					future = self._node.post(self._shutdown_in_ros, is_powered)
			if future is not None:
				try:
					power = future.result(timeout=ROS_CALL_TIMEOUT)
					if power is not None:
						result = self._wait_service(power, config.MOTOR_POWER_SERVICE)
						logger.info('종료: 바퀴 토크 끄기 %s', result['message'])
					# 켜는 중인 토크는 권한이 끝나 실패 처리(power false)된다: ROS 가 살아 있을 때 끝나게
					if self._motor_lock.acquire(timeout=POWER_ON_MAX_TIME):
						self._motor_lock.release()
					# hub 가 먼저 연결을 닫아 권한이 끝났으면 그때 보낸 토크 끄기도 기다린다
					with self._lock:
						pending = list(self._power_offs)
					concurrent.futures.wait(pending, timeout=config.MOTOR_SERVICE_TIMEOUT)
				except Exception:
					logger.exception('종료 중 ROS 정리에 실패했습니다')
			self._rest_arm_before_shutdown()
			# 탐색 스레드가 끝난 뒤에 Cartographer · 노드를 끈다 (닫힌 노드를 읽지 않게)
			thread = self._explore_thread
			if thread is not None:
				thread.join(EXPLORE_JOIN_TIMEOUT)
				if thread.is_alive():
					logger.error('탐색 스레드가 %g 초 안에 끝나지 않았습니다', EXPLORE_JOIN_TIMEOUT)
			self._processes.shutdown()
			self._stop_ros()

	# ── 읽기 ───────────────────────────────────────────────
	def get_info(self) -> dict:
		"""바뀌지 않는 정보 (SPEC §6 get_info)."""
		return {
			'robot_name': socket.gethostname(),
			'version': __version__,
			'is_read_only': self._is_read_only,
			'features': dict(self._features),
			'speed_levels': [list(level) for level in config.SPEED_LEVELS],
			'start_level': config.START_LEVEL,
			'map_dir': str(config.map_dir()),
		}

	def get_status(self) -> dict:
		"""상태 스냅숏 (SPEC §6 + estop). 웹 쪽이 type · system · network 를 덧붙인다."""
		node = self._require_node()
		snapshot = node.snapshot()
		node_names, node_count, others, base_count = node.graph()
		processes = self._processes.status()
		with self._lock:
			is_allowed, reason = self._allowed
			estop = self._estop
			exploration = dict(self._exploration)
			percent = self._navigation_speed
			arm_session, is_relaying, leader_at = self._arm_session, self._is_relaying, self._leader_at
			estop_arm, is_arm_maybe_on = self._estop_arm, self._is_arm_maybe_on
			arm_rest = dict(self._arm_rest)
		arm = snapshot['arm']
		mode = 'idle'
		if processes['mapping']['state'] in ACTIVE_STATES:
			mode = 'mapping'
		if processes['navigation']['state'] in ACTIVE_STATES:
			mode = 'navigation'
		teleop = self._teleop.status(is_allowed, reason)
		# 보기 전용은 로봇 게이트웨이가 정한 속도를 모른다: 제 기본값을 보이지 않는다
		is_speed_known = self._features['has_navigation'] and not self._is_read_only
		speed = percent if is_speed_known else None
		navigation = {**snapshot['navigation'], 'speed_percent': speed}
		distance = navigation.get('distance_remaining')
		if distance is not None:   # 추정: 남은 거리 / (Nav2 최고 속도 × 속도 비율) (Nav2 피드백은 받지 않는다)
			eta = distance / (config.NAV_MAX_LINEAR_SPEED * (percent / 100))
			navigation['eta'] = round(eta, 1)
		return {
			'time': time.time(),
			'ros': {
				'is_ok': node.context.ok(), 'domain_id': self._domain_id, 'node_count': node_count,
			},
			'mode': mode,
			'health': {name: name in node_names for name in config.BRINGUP_NODES},
			'rates': snapshot['rates'],
			'ages': snapshot['ages'],
			'processes': processes,
			'control': {'holder': self._teleop.holder()},
			'teleop': teleop,
			'motor': {
				'ready': snapshot['motor_ready'],
				'is_powered_by_us': self._teleop.is_powered_by_us(),
			},
			'estop': {'is_latched': estop is not None, **(estop or {'since': None, 'label': None})},
			'motors': snapshot['motors'],
			'power': snapshot['power'],
			'odom': snapshot['odom'],
			'pose': snapshot['pose'],
			'imu': snapshot['imu'],
			'navigation': navigation,
			'exploration': exploration,
			'teleop_arm': {
				'is_follower_present': config.FOLLOWER_ARM_NODE in node_names,
				'arm_ready': arm['ready'],
				'follower': arm['status'],
				'follower_age': arm['status_age'],
				'joints': arm['joints'],
				'is_session_active': arm_session is not None,
				'is_torque_maybe_on': is_arm_maybe_on,
				'blocked_reason': self._arm_blocked(arm, node_names),
				# shortcut: 2b 의 PC 에이전트 연결 · 리더 주기 · 관절은 아직 없다, 리더 전송을 만들 때 더한다
				'leader': {
					'is_relaying': is_relaying,
					'age': None if leader_at is None else round(time.monotonic() - leader_at, 2),
				},
				'estop': estop_arm,
				'rest': arm_rest,
			},
			'calibration': {'follower': self._calibration.status(arm['status'])},
			'warnings': self._warnings(snapshot, node_names, others, base_count),
		}

	def get_scan(self) -> dict | None:
		"""스캔 점 (base_footprint) 과 그때의 자세. 시작 전이거나 라이다 TF 를 모르면 None."""
		return None if self._node is None else self._node.get_scan()

	def get_map(self) -> dict | None:
		"""지도 PNG 와 정보. 없으면 None."""
		return None if self._node is None else self._node.get_map()

	def get_plan(self) -> dict | None:
		"""자율 주행 경로. 없으면 None."""
		return None if self._node is None else self._node.get_plan()

	def get_camera_frame(self, camera: str) -> tuple[int, bytes] | None:
		"""(순번, JPEG). 구독하지 않았으면 None."""
		return None if self._node is None else self._node.get_camera_frame(camera)

	def set_wanted_channels(self, channels: set[str], has_clients: bool) -> None:
		"""화면들이 원하는 채널에 맞춰 무거운 구독(STREAM_CHANNELS)을 켜고 끈다 (Pi CPU).

		새로 원하면 바로 구독하고, 원하지 않게 되면 CAMERA_RELEASE_DELAY 뒤에 푼다 (그래프 타이머).
		has_clients 는 쓰지 않는다: 화면이 연결됐다고 구독하지 않고 채널만 본다.
		"""
		with self._lock:
			self._screen_streams = frozenset(channels) & frozenset(STREAM_CHANNELS)
		self._update_wanted_streams()

	def _update_wanted_streams(self) -> None:
		"""(아무 스레드) 원하는 묶음 = 화면이 원하는 것 ∪ 자동 주행 중이면 EXPLORE_STREAMS (S4) ∪ 팔 시작 ·
		세션 중 · 팔을 접는 중이면 ARM_STREAMS.

		새로 원하게 된 것은 ROS 스레드에 넘겨 바로 구독한다. 원하지 않게 된 것은 그때부터
		CAMERA_RELEASE_DELAY 뒤에 그래프 타이머가 푼다.
		"""
		now = time.monotonic()
		is_added = False
		with self._lock:
			channels = self._screen_streams
			if self._explore_token is not None:
				channels = channels | EXPLORE_STREAMS
			if self._arm_session is not None or self._is_arm_starting or (
				self._arm_rest['state'] == 'resting'
			):
				channels = channels | ARM_STREAMS
			if 'calibration' in channels or self._calibration.is_wanting():
				channels = channels | CALIBRATION_STREAMS
			for stream in STREAM_CHANNELS:
				is_wanted = stream in channels
				if is_wanted != self._stream_wanted[stream]:
					self._stream_wanted[stream] = is_wanted
					self._stream_changed_at[stream] = now
					is_added = is_added or is_wanted
		if is_added and self._node is not None:
			self._node.post(self._sync_streams)

	def get_process_log(self, name: str, max_lines: int) -> list[str]:
		"""프로세스의 최근 로그 줄."""
		return self._processes.get_log(name, max_lines)

	def add_listener(self, callback: Callable[[dict], None]) -> None:
		"""이벤트 · 로그를 받을 함수를 등록한다."""
		with self._lock:
			self._listeners.append(callback)

	# ── 조작 권한 · 주행 ───────────────────────────────────
	def acquire_control(self, client_id: str, label: str) -> CommandResult:
		"""조작 권한 받기 (SPEC §7-1). 조작 노드 · 퍼블리셔는 바로 다음 조작 타이머에서 만든다."""
		if self._is_read_only:
			return error_result('read_only', '보기 전용 모드라 조작 권한을 받을 수 없습니다')
		is_already_operator = self._teleop.is_operator(client_id)
		if not is_already_operator and self._node is not None:
			# 1 Hz 그래프가 다른 게이트웨이에 방금 생긴 cmd_vel 발행자를 모르면 첫 조작 타이머가 퍼블리셔를 만든다
			self._run_in_ros(self._node.refresh_graph)
		with self._lease_lock:
			result, ended = self._teleop.acquire(client_id, label)
			if ended is not None:
				self._finish_lease(ended, f'{LEASE_LOST_MESSAGE} ({ended["label"]})', 'warn')
		if not result['ok'] or is_already_operator:
			return result
		message = f'조작 권한을 받았습니다 ({label})'
		self._emit('info', message)
		if self._node is not None:   # 2 Hz 로 쉬던 조작 타이머를 기다리지 않고 바로 한 번 돈다
			self._node.post(self.teleop_timer_callback)
		return ok_result(message)

	def release_control(self, client_id: str) -> CommandResult:
		"""권한 놓기: 정지, 이 권한으로 켠 토크 끔, 조작 노드 · cmd_vel 퍼블리셔 삭제."""
		if self._is_read_only:
			return error_result('read_only', READ_ONLY_MESSAGE)
		with self._lease_lock:
			ended = self._teleop.release(client_id)
			if ended is None:
				return error_result('not_operator', '조작 권한이 없습니다')
			message = f'조작 권한을 놓았습니다 ({ended["label"]})'
			self._finish_lease(ended, message, 'info')
		return ok_result(message)

	def is_operator(self, client_id: str) -> bool:
		"""지금 조작 권한을 가진 클라이언트인지."""
		return self._teleop.is_operator(client_id)

	def lease_token(self, client_id: str) -> int | None:
		"""client_id 가 holder 면 그 조작 권한 번호 (권한마다 새 번호), 아니면 None."""
		return self._teleop.lease_token(client_id)

	def touch_operator(self, client_id: str) -> None:
		"""조작 화면 생존 신호."""
		self._teleop.touch(client_id)

	def set_teleop(
		self, client_id: str, x: float, y: float, yaw: float, level: int,
		pad: str = config.DEFAULT_PAD,
	) -> None:
		"""방향 입력 (holder 만, 나머지는 무시). 자동 주행 중의 0 이 아닌 입력은 자동 주행을 끝낸다."""
		with self._lease_lock:   # 번호를 지운 것과 그 이유를 한 번에 (탐색 스레드가 이 잠금 안에서 읽는다)
			if self._teleop.set_input(client_id, x, y, yaw, level, pad):
				self._note_explore_end(TAKEOVER_MESSAGE)

	def set_motor_power(self, client_id: str, is_enabled: bool) -> CommandResult:
		"""(막힘) 바퀴 토크 켜기(holder · 조작 허용일 때만) · 끄기(holder) (SPEC §7-9)."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if not is_enabled:
			return self._power_off_by_operator()
		if not self._motor_lock.acquire(blocking=False):
			return error_result('conflict', '바퀴 토크를 켜는 중입니다')
		try:
			return self._power_on(client_id)
		finally:
			self._motor_lock.release()

	def emergency_stop(self, client_id: str, label: str = '') -> CommandResult:
		"""누구나 (연결이 끊긴 화면도): 속도 0, 자율 주행 목표 취소, 바퀴 · 팔 토크 끔 (누가 켰든).

		팔 토크 끄기는 바퀴 토크 끄기 다음에 보내고 기다리지 않는다 (바퀴를 늦추지 않는다). 결과는
		status.teleop_arm.estop 과 (안 됐으면) 이벤트로 알린다. 팔 원격 조작 세션도 닫는다.

		권한은 그대로 둔다 (§7-10). 비상 정지는 걸린 채로 남아(status.estop) 조작 노드를 내리고,
		ESTOP_HOLDOFF 안에 온 토크 켜기 · 자율 주행 시작은 거절한다. holder 가 다시 토크 켜기나
		자율 주행 시작에 성공하면 풀린다. 이벤트는 하나로 알린다 (실패한 단계만 따로 알린다).

		Args:
			client_id: 누른 화면 (비거나 끊긴 화면의 옛 id 일 수 있다).
			label: 누른 화면 표시 '<IP> · <브라우저>'. 비어 있으면 holder 의 표시나 UNKNOWN_SCREEN_LABEL.
		"""
		if self._is_read_only:
			return error_result('read_only', '보기 전용 모드라 비상 정지를 보낼 수 없습니다')
		node = self._require_node()
		if not label:
			holder = self._teleop.holder()
			is_holder = holder is not None and holder['client_id'] == client_id
			label = holder['label'] if is_holder else UNKNOWN_SCREEN_LABEL
		with self._lock:
			self._estop_seq += 1
			self._estop_at = time.monotonic()
			self._estop = {'since': time.time(), 'label': label}
			self._estop_arm = {'state': 'pending', 'message': None, 'time': time.time()}
			estop_seq = self._estop_seq
			# 번호와 함께: 도는 팔 시작은 이 번호를 보고 세션을 열지 않는다
			self._clear_arm_session_locked()
		self._note_explore_end(ESTOP_EXPLORE_MESSAGE)
		self._teleop.stop()   # 자동 주행 번호도 지운다
		self._teleop.take_powered()
		node.post(self._emergency_stop_in_ros, estop_seq)
		self._calibration.abort('비상 정지')   # 제로-포지션 위치 시키기로 쓴 값은 보정 프로그램이 되돌린다
		self._update_wanted_streams()
		# 토크 끄기는 기다리지 않는다: 끄는 중이라고 알리고, 안 되면 따로 알린다
		message = f'비상 정지 ({label}): 멈춤 · 목표 취소 · 바퀴 · 팔 토크 끄는 중'
		self._emit('warn', message)
		return ok_result(message)

	def client_disconnected(self, client_id: str) -> None:
		"""WS 가 끊겼다. 권한을 가졌으면 신호 끊김과 같게 처리한다 (§7-2)."""
		with self._lease_lock:
			ended = self._teleop.release(client_id)
			if ended is not None:
				self._finish_lease(ended, f'{LEASE_LOST_MESSAGE} ({ended["label"]})', 'warn')

	# ── 팔 원격 조작 ───────────────────────────────────────
	def arm_start(self, client_id: str) -> CommandResult:
		"""(막힘, 최대 ARM_START_MAX_TIME) 팔 원격 조작 시작 (holder): 막는 이유(_arm_blocked)가 없으면
		팔 토크를 켜고 (lekiwi/motor/arm_torque true, 이미 켜져 있으면 드라이버가 아무것도 쓰지 않는다)
		세션을 연다. 세션 중에만 relay_leader 가 lekiwi/arm/goal 을 보낸다.

		'arm' 묶음은 시작하는 동안과 세션 동안 화면과 상관없이 구독한다 (ARM_STATUS_WAIT 까지 상태를 기다린다).
		토크 켜기와 같은 규칙: 비상 정지 직후(ESTOP_HOLDOFF)에는 거절하고, 켜는 사이 비상 정지가
		눌렸으면 늘 conflict 로 끝낸다 (그 비상 정지가 팔 토크를 끈다, 한 번 더 끈다). 그사이 권한 끝 ·
		팔 멈춤 · 팔 힘 빼기 · 자율 주행 · 자동 주행이 있었으면 세션 없이 conflict 로 끝낸다 (토크는 그 길의 규칙대로).
		"""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if not self._arm_lock.acquire(blocking=False):
			return error_result('conflict', '팔을 시작하거나 힘을 빼는 중입니다')
		with self._lock:
			self._is_arm_starting = True
		try:
			self._update_wanted_streams()   # 'arm' 을 보는 화면이 없어도 (2b 의 에이전트) 팔로워 상태를 받는다
			result = self._arm_start(client_id)
		finally:
			with self._lock:
				self._is_arm_starting = False
			self._arm_lock.release()
			self._update_wanted_streams()   # 세션이 열렸으면 계속, 아니면 CAMERA_RELEASE_DELAY 뒤에 푼다
		return result

	def arm_stop(self, client_id: str) -> CommandResult:
		"""팔 원격 조작 멈춤 (holder): 세션을 닫는다. 팔 토크는 켠 채 그 자리를 유지한다."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		with self._lock:
			self._arm_stop_seq += 1   # 도는 팔 시작은 세션을 열지 않는다
		if not self._clear_arm_session():
			return ok_result('팔 원격 조작 중이 아닙니다')
		self._emit('info', ARM_STOPPED_MESSAGE)
		return ok_result(ARM_STOPPED_MESSAGE)

	def arm_relax(self, client_id: str) -> CommandResult:
		"""(막힘) 팔 힘 빼기 (holder): 세션을 닫고 팔 토크를 끈다 (편 팔은 떨어진다).

		세션은 바로 닫고 (도는 팔 시작도 세션을 열지 않는다), 도는 팔 시작이 있으면 끝나기를 기다린 뒤에
		끈다 (늦게 간 켜기가 이 끄기를 이기지 않게). 비상 정지 · 자율 주행 중에도 된다.
		"""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		with self._lock:
			self._arm_stop_seq += 1
		self._clear_arm_session()
		if not self._arm_lock.acquire(timeout=ARM_START_MAX_TIME):
			return error_result('conflict', '팔을 시작하는 중입니다. 잠시 뒤 다시 누르세요')
		try:
			state, detail = self._turn_arm_off()
		finally:
			self._arm_lock.release()
		if state != 'ok':
			message = f'팔 힘을 빼지 못했습니다: {detail}'
			self._emit('error', message)
			return error_result('unavailable' if state == 'unavailable' else 'failed', message)
		self._emit('info', ARM_RELAXED_MESSAGE)
		return ok_result(ARM_RELAXED_MESSAGE)

	def calibration_follower(self, client_id: str, action: str, body: dict) -> CommandResult:
		"""(막힘일 수 있음) 보정 탭 팔로워 명령 (follower_calibration.FollowerCalibration.command)."""
		if self._is_read_only:
			return error_result('read_only', READ_ONLY_MESSAGE)
		return self._calibration.command(client_id, action, body)

	def is_calibration_recording(self) -> bool:
		"""팔로워 보정이 범위를 재는 중 (화면 상태를 더 자주 보낸다)."""
		return self._calibration.is_recording()

	def is_arm_session_active(self) -> bool:
		"""팔 원격 조작 세션이 열려 있다 (리더 값이 팔로워로 간다: 리더 보정을 막는다)."""
		with self._lock:
			return self._arm_session is not None

	def end_arm_session(self, client_id: str, reason: str) -> bool:
		"""client_id 의 권한이 연 팔 세션이 열려 있으면 닫고 이유를 알린다 (팔 토크는 그대로, PC 에이전트가
		오래 끊겼을 때). 닫았으면 True."""
		token = self._teleop.lease_token(client_id)
		with self._lock:
			is_open = token is not None and self._arm_session == token
			if is_open:
				self._arm_stop_seq += 1
				self._clear_arm_session_locked()
		if is_open:
			self._update_wanted_streams()
			self._emit('warn', ARM_HELD_MESSAGE.format(reason=reason))
		return is_open

	def relay_leader(self, client_id: str, joints: Any) -> bool:
		"""리더 관절 하나를 lekiwi/arm/goal 로 보낸다 (2b 의 리더 전송이 약 30 Hz 로 부른다). 바로 돌아온다.

		팔 원격 조작 세션 중이고 client_id 가 그 세션을 연 조작 권한일 때만 보낸다. 생존 신호로 치지
		않는다. ARM_LEADER_TIMEOUT 넘게 오지 않으면 조작 타이머가 알리고 (status.teleop_arm.leader),
		ROS 스레드는 그보다 오래 밀린 값과 세션이 끝난 뒤의 값을 보내지 않는다.

		Args:
			client_id: 보내는 쪽의 조작 권한 (2b: 에이전트와 짝지은 권한 화면).
			joints: {config.ARM_JOINTS 의 이름 모두: 유한한 수} (LeRobot 단위). 아니면 버린다.

		Returns:
			ROS 스레드에 보내기를 넘겼으면 True.
		"""
		if not isinstance(joints, dict) or set(joints) != set(config.ARM_JOINTS):
			return False
		positions = [finite_number(joints[name]) for name in config.ARM_JOINTS]
		if None in positions:
			return False
		token = self._teleop.lease_token(client_id)
		received_at = time.monotonic()
		with self._lock:
			if token is None or token != self._arm_session:
				return False
			self._leader_at, self._is_relaying = received_at, True
		self._require_node().post(self._publish_leader_in_ros, token, positions, received_at)
		return True

	# ── 기능 (launch) ──────────────────────────────────────
	def start_process(self, client_id: str, name: str, options: dict) -> CommandResult:
		"""bringup {'use_camera': bool} · mapping {} · navigation {'map': '<이름>.yaml'}."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if name not in config.PROCESS_NAMES:
			return error_result('bad_request', f'모르는 기능입니다: {name}')
		if not isinstance(options, dict):
			return error_result('bad_request', 'options 는 JSON 객체여야 합니다')
		blocked = self._processes.check_start(name)
		if blocked is not None:
			return blocked
		if name == 'bringup':
			if self._calibration.is_active():   # 보정 프로그램이 모터 버스를 쓰고 있다 (버스 주인은 하나)
				return error_result('conflict', '팔로워 보정 중입니다. 보정을 마치거나 취소하면 다시 켭니다')
			use_camera = options.get('use_camera', True)
			if not isinstance(use_camera, bool):
				return error_result('bad_request', 'use_camera 는 true 또는 false 여야 합니다')
			return self._processes.start('bringup', {'use_camera': use_camera})
		if name == 'mapping':
			return self._processes.start('mapping', {})
		return self._start_navigation(client_id, options.get('map'))

	def stop_process(self, client_id: str, name: str, is_forced: bool = False) -> CommandResult:
		"""웹에서 켠 것과 터미널에서 켠 것 모두 끈다 (진행은 status). 지도 작성은 자동 주행을 먼저 끝낸다.

		로봇(bringup)은 이 게이트웨이가 켠 팔 토크가 남아 있을 수 있으면 먼저 (막힘) 팔을 접고 끈다
		(_rest_arm). 못 접으면 끄지 않고 conflict 와 이유. is_forced 면 접지 않고 팔 힘을 뺀 뒤 끈다.
		"""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if name not in config.PROCESS_NAMES:
			return error_result('bad_request', f'모르는 기능입니다: {name}')
		if name == 'mapping':   # Cartographer · tf2_echo 가 꺼지는 동안 움직이지 않게 (S6)
			self._end_exploration(MAPPING_STOP_MESSAGE)
		if name == 'bringup':
			refused = self._rest_arm_before_bringup_stop(is_forced)
			if refused is not None:
				return refused
			denied = self._check_operator(client_id)   # 접는 몇 초 사이 권한이 끝났을 수 있다
			if denied is not None:
				return denied
		return self._processes.stop(name)

	def save_map(self, client_id: str) -> CommandResult:
		"""지도 작성 중에 지도를 저장한다 (결과는 status · event)."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		return self._processes.save_map()

	# ── 자동 주행 맵 생성 ──────────────────────────────────
	def start_exploration(self, client_id: str) -> CommandResult:
		"""자동 주행 맵 생성 시작 (holder). 탐색 스레드를 띄우고 바로 돌아온다 (진행은 status).

		지도 작성 중 · 조작 허용 · 이 권한으로 켠 토크 (S5) · 비상 정지 없음 · 지도를 받음이어야 한다.
		번호는 그사이 비상 정지가 없었을 때만 만든다 (_power_on 과 같은 규칙, M1).
		"""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if not self._explore_lock.acquire(blocking=False):
			return error_result('conflict', EXPLORING_MESSAGE)
		is_started = False
		try:
			blocked = self._explore_blocked()
			if blocked is not None:
				return error_result('conflict', blocked)
			estop_seq = self._estop_seq_if_settled()
			if estop_seq is None:
				return error_result('conflict', ESTOP_HOLDOFF_MESSAGE)
			started = time.monotonic()
			explorer = explore.Explorer(started)
			# 팔 세션은 위 _explore_blocked 가 먼저 거절한다 (직접 조작을 멈추기 전에)
			self._teleop.stop()   # 아직 도는 직접 조작 명령이 섞이지 않게 (S5)
			with self._lock:
				is_estopped = self._estop is not None or self._estop_seq != estop_seq
				# 팔 세션과 같은 잠금 안에서 한 번 더 본다 (팔 시작도 이 잠금 안에서 자동 주행 번호를 본다)
				is_arm_active = self._arm_session is not None
				is_refused = is_estopped or is_arm_active
				token = None if is_refused else self._teleop.start_autonomous(client_id)
				if token is not None:
					self._explore_token, self._explore_reason = token, None
					self._exploration = {
						**IDLE_EXPLORATION, 'state': 'starting', 'started_at': time.time(),
						'elapsed': 0.0,
					}
			if token is None:
				message = f'{ESTOPPED_MESSAGE}. 자동 주행을 시작하지 않았습니다'
				if not is_estopped:
					message = ARM_SESSION_MESSAGE if is_arm_active else NOT_OPERATOR_MESSAGE
				return error_result('conflict', message)
			self._update_wanted_streams()   # 스캔 · odom · 지도 위 자세를 바로 구독한다 (S4)
			thread = threading.Thread(
				target=self._exploration_thread, args=(token, explorer, started), name='explore',
				daemon=True,
			)
			self._emit('info', EXPLORE_STARTED_MESSAGE)   # 스레드가 내는 끝 이벤트보다 먼저
			self._warn_if_arm_torqued()
			try:
				thread.start()
			except BaseException:   # 스레드를 못 띄웠다: 번호 · 구독 · status 를 되돌린다
				self._end_run(token, 'stopped', EXPLORER_ERROR_MESSAGE, None, started)
				raise
			self._explore_thread = thread
			is_started = True
		finally:
			if not is_started:
				self._explore_lock.release()
		return ok_result(EXPLORE_STARTED_MESSAGE)

	def stop_exploration(self, client_id: str) -> CommandResult:
		"""자동 주행 멈춤 (holder). 토크는 켜 둔다. 탐색 스레드가 다음 주기에 'stopped' 로 끝낸다."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if not self._end_exploration(USER_STOP_MESSAGE):
			return ok_result('자동 주행 중이 아닙니다')
		return ok_result(USER_STOP_MESSAGE)

	def set_initial_pose(self, client_id: str, x: float, y: float, yaw: float) -> CommandResult:
		"""자율 주행의 초기 위치 (map 프레임, §7-13)."""
		denied = self._check_navigation_command(client_id, (x, y, yaw))
		if denied is not None:
			return denied
		self._run_in_ros(self._require_node().publish_initial_pose, float(x), float(y), float(yaw))
		return ok_result('초기 위치를 보냈습니다')

	def send_nav_goal(self, client_id: str, x: float, y: float, yaw: float) -> CommandResult:
		"""자율 주행 목표 (map 프레임). 새 목표는 이전 것을 대체한다."""
		denied = self._check_navigation_command(client_id, (x, y, yaw))
		if denied is not None:
			return denied
		if not self._features['has_navigation']:
			return error_result('unavailable', NO_NAVIGATION_MESSAGE)
		is_sent = self._run_in_ros(self._send_goal_in_ros, float(x), float(y), float(yaw))
		if not is_sent:
			return error_result(
				'unavailable', f'자율 주행 서버({config.NAVIGATE_ACTION})가 아직 준비되지 않았습니다',
			)
		return ok_result('목표를 보냈습니다')

	def cancel_nav_goal(self, client_id: str) -> CommandResult:
		"""누구나: 진행 중인 자율 주행 목표 취소."""
		if self._is_read_only:
			return error_result('read_only', READ_ONLY_MESSAGE)
		if not self._features['has_navigation']:
			return error_result('unavailable', NO_NAVIGATION_MESSAGE)
		has_goal = self._run_in_ros(self._require_node().cancel_goal)
		return ok_result('목표 취소를 보냈습니다' if has_goal else '진행 중인 목표가 없습니다')

	def set_navigation_speed(self, client_id: str, percent: int) -> CommandResult:
		"""자율 주행 속도 = Nav2 최고 속도의 percent % (holder). 자율 주행이 꺼져 있어도 받아 기억하고,
		실행 중 · 외부 실행이면 바로 보낸다 (그 뒤로는 목표마다, 100 % 보다 낮으면 NAV_SPEED_PERIOD 마다)."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		low, high = config.NAV_SPEED_MIN, config.NAV_SPEED_MAX
		if isinstance(percent, bool) or not isinstance(percent, int) or not low <= percent <= high:
			return error_result('bad_request', f'percent 는 {low}~{high} 사이의 정수여야 합니다')
		if not self._features['has_navigation']:
			return error_result('unavailable', NO_NAVIGATION_MESSAGE)
		with self._lock:
			self._navigation_speed = percent
		if self._processes.state('navigation') not in RUNNING_STATES:
			return ok_result(f'자율 주행 속도를 {percent} %로 정했습니다. 자율 주행을 켜면 적용됩니다')
		self._run_in_ros(self._publish_speed_in_ros)
		return ok_result(f'자율 주행 속도를 {percent} %로 바꿨습니다')

	def reset_odometry(self, client_id: str) -> CommandResult:
		"""(막힘) odom 을 원점으로. 지도 작성 · 자율 주행 중에는 거부한다 (위치가 튀어 지도가 깨진다)."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if any(self._processes.state(name) in ACTIVE_STATES for name in ('mapping', 'navigation')):
			return error_result('conflict', '지도 작성 · 자율 주행 중에는 odom 원점을 바꿀 수 없습니다')
		future = self._require_node().call_reset_odometry()
		result = self._wait_service(future, config.RESET_ODOMETRY_SERVICE)
		if not result['ok']:
			return result
		self._emit('info', 'odom 을 원점으로 돌렸습니다')
		return ok_result('odom 을 원점으로 돌렸습니다')

	# ── ROS 스레드 ─────────────────────────────────────────
	def teleop_timer_callback(self) -> None:
		"""권한 시간 초과 · 조작 허용 · 조작 노드 · cmd_vel 퍼블리셔 · 발행.

		holder 가 있으면 TELEOP_PERIOD, 없으면 IDLE_TELEOP_PERIOD 마다 돈다 (_update_teleop_rate).
		"""
		node = self._node
		with self._lease_lock:
			ended = self._teleop.expire()
			if ended is not None:
				self._finish_lease(ended, f'{LEASE_LOST_MESSAGE} ({ended["label"]})', 'warn')
		is_allowed, reason = self._compute_allowed()
		with self._lock:
			self._allowed = (is_allowed, reason)
		node.set_view_node(self._is_view_wanted())
		if is_allowed:
			node.create_cmd_vel_publisher()
		else:
			node.drop_cmd_vel_publisher()
			if self._teleop.take_powered():
				# 조작할 수 없는데 우리가 켠 토크가 남아 있으면 끈다 (Nav2 감독 도구는 ready
				# false 일 때만 시작하므로, 터미널에서 자율 주행을 켰을 때도 막히지 않게 한다)
				self._power_off_async(f'바퀴 토크를 껐습니다: {reason}')
		command = self._teleop.step(is_allowed)
		if command is not None:
			node.publish_cmd_vel(command)
		self._watch_arm()
		self._update_teleop_rate()

	def graph_timer_callback(self) -> None:
		"""(GRAPH_PERIOD) 노드 그래프 · 기능 상태 · 감독 도구 · 속도 상한 · 필요할 때만 구독."""
		node = self._node
		node.refresh_graph()
		node_names, _count, _others, _base_count = node.graph()
		self._processes.set_node_names(node_names)
		self._watch_guard(node_names)
		has_base = config.BASE_CONTROLLER_NODE in node_names
		if has_base and not self._has_base_controller:
			self._is_limits_stale = True   # 나타날 때마다 다시 읽는다
		self._has_base_controller = has_base
		now = time.monotonic()
		is_waiting = (
			self._limits_requested_at is not None
			and now - self._limits_requested_at < LIMITS_RETRY_PERIOD
		)
		if has_base and self._is_limits_stale and not is_waiting:
			if node.request_speed_limits(self._limits_callback):
				self._limits_requested_at = now
		self._sync_streams()

	def _watch_guard(self, node_names: frozenset[str]) -> None:
		"""(ROS 스레드) 자율 주행 중 감독 도구가 보였다가 사라지면 바로 목표를 막고, Nav2 노드가
		남은 채 GUARD_GONE_DELAY 동안 안 보이면 끝났다고 한 번 알린다.

		감독 도구는 한 번 토크를 끄면(비상 정지 · 권한 끝 · 시간 제한 · 센서 끊김) 끝나고 다시
		켜지 않는다. 그 뒤 목표를 받으면 바퀴 없이 Nav2 진행 검사가 끝날 때까지 기다리게 된다.
		"""
		if self._processes.state('navigation') not in NAVIGATION_UP_STATES:
			# 끄는 중 · 꺼짐: 다음 자율 주행에서 다시
			self._has_seen_guard = self._is_guard_gone = False
			self._guard_missing_since = None
			return
		if config.NAV2_MOTOR_GUARD_NODE in node_names:
			self._has_seen_guard = True
			self._guard_missing_since = None
		elif not self._has_seen_guard or self._is_guard_gone:
			return
		elif self._guard_missing_since is None:
			self._guard_missing_since = time.monotonic()
		elif (
			node_names & NAV2_CORE_NODES   # Nav2 도 끝나는 중이면 자율 주행을 끄는 것이다 (Ctrl+C)
			and time.monotonic() - self._guard_missing_since >= GUARD_GONE_DELAY
		):
			self._is_guard_gone = True
			self._emit('warn', GUARD_GONE_MESSAGE)

	def speed_timer_callback(self) -> None:
		"""(NAV_SPEED_PERIOD) 자율 주행이 실행 중 · 외부 실행이고 100 % 보다 낮으면 속도를 다시 보낸다:
		다시 켠 controller_server 는 100 % 로 시작하고 volatile 토픽의 지난 값을 받지 못한다.
		100 % 는 다시 보내지 않는다: 새 controller_server 는 이미 100 % 이고, 아무도 고르지 않은
		기본값이 다른 게이트웨이 · 전에 켰던 게이트웨이가 낮춘 값을 덮으면 안 된다 (낮은 쪽이 안전)."""
		with self._lock:
			is_lowered = self._navigation_speed < 100   # 100 % = 새 controller_server 의 값
		if is_lowered and self._processes.state('navigation') in RUNNING_STATES:
			self._publish_speed_in_ros()

	def _limits_callback(self, limits: dict[str, Any] | None) -> None:
		"""(ROS 스레드) 속도 상한 응답."""
		self._limits_requested_at = None
		self._is_limits_stale = False
		if self._teleop.set_limits(limits):
			logger.info('베이스 컨트롤러 속도 상한: %s', limits)
		else:
			logger.warning('속도 상한을 읽지 못해 기본값을 씁니다 (받은 값: %s)', limits)

	def _sync_streams(self) -> None:
		"""(ROS 스레드) 원하는 묶음은 구독하고, 원하지 않게 된 지 CAMERA_RELEASE_DELAY 지나면 푼다."""
		now = time.monotonic()
		with self._lock:
			wanted = dict(self._stream_wanted)
			changed_at = dict(self._stream_changed_at)
		for stream, is_wanted in wanted.items():
			if is_wanted:
				self._node.set_stream_subscribed(stream, True)
			elif now - changed_at[stream] >= config.CAMERA_RELEASE_DELAY:
				self._node.set_stream_subscribed(stream, False)

	def _update_teleop_rate(self) -> None:
		"""(ROS 스레드) holder 가 있으면 TELEOP_PERIOD, 없으면 IDLE_TELEOP_PERIOD 로 바꾼다."""
		timer = self._teleop_timer
		if timer is None:
			return
		period = config.TELEOP_PERIOD if self._teleop.holder() is not None else IDLE_TELEOP_PERIOD
		period_ns = round(period * 1e9)
		if timer.timer_period_ns != period_ns:
			timer.timer_period_ns = period_ns
			timer.reset()   # 바꾼 주기를 다음 호출부터 바로 쓴다

	def _finish_lease_in_ros(self, is_powered: bool) -> None:
		"""권한이 끝났다: 0 한 번 보내고 퍼블리셔 삭제, 조작 노드 삭제, 우리가 켠 토크 끔."""
		self._node.drop_cmd_vel_publisher()
		self._node.set_view_node(self._is_view_wanted())
		if is_powered:
			self._power_off_async('조작 권한이 끝나 바퀴 토크를 껐습니다')
		self._update_teleop_rate()

	def _emergency_stop_in_ros(self, estop_seq: int) -> None:
		"""0 발행 · 목표 취소 · 조작 노드 내림 (감독 도구가 토크를 다시 켜지 못하게) · 바퀴 토크 끔 ·
		팔 토크 끔 (바퀴 다음에, 기다리지 않는다)."""
		self._node.publish_cmd_vel(ZERO_COMMAND)
		self._node.cancel_goal()
		self._node.set_view_node(False)
		self._power_off_async(None)
		future = self._node.call_arm_torque(False)
		with self._lock:   # 종료가 바퀴 토크 끄기처럼, 팔 시작이 켜기 전에 이 응답을 기다린다
			self._power_offs = [old for old in self._power_offs if not old.done()] + [future]
			self._arm_offs = [old for old in self._arm_offs if not old.done()] + [future]
		future.add_done_callback(lambda done: self._report_estop_arm(done, estop_seq))

	def _report_estop_arm(self, future: concurrent.futures.Future, estop_seq: int) -> None:
		"""비상 정지의 팔 토크 끄기 결과를 status.teleop_arm.estop 에 적고 알린다 (껐으면 이 게이트웨이가
		켰던 팔일 때만, 못 껐으면 이유와 함께)."""
		with self._lock:
			is_maybe_on = self._is_arm_maybe_on
		state, detail = self._arm_off_state(future, None)
		node = self._node
		has_driver = node is not None and config.MOTOR_DRIVER_NODE in node.graph()[0]
		level: str | None = 'error'
		if state == 'ok':
			message, level = f'비상 정지: {ARM_OFF_MESSAGE}', 'info' if is_maybe_on else None
		elif state == 'failed':
			message = f'비상 정지: {detail}'
		elif has_driver:   # 드라이버가 팔 쓰기 없이 돈다 (use_arm:=false)
			message, level = ARM_TORQUE_MISSING_MESSAGE, 'error' if is_maybe_on else 'warn'
		else:   # 드라이버가 없다. 끝날 때 팔 토크를 둔 드라이버일 수 있다
			message, level = ARM_DRIVER_GONE_MESSAGE, 'error' if is_maybe_on else None
		self._note_arm_off(state, message, estop_seq)
		if level is not None:
			self._emit(level, message)
		elif state != 'ok':
			logger.info('비상 정지: 팔 토크 서비스가 없습니다 (모터 드라이버도 없고, 이 게이트웨이가 켠 팔도 아닙니다)')

	def _arm_off_state(
		self, future: concurrent.futures.Future, timeout: float | None,
	) -> tuple[str, str]:
		"""arm_torque false 응답 → (state, 한국어 이유). state: 'ok' · 'failed' (응답 없음 · 오류 · 일부 팔을
		확인하지 못함) · 'unavailable' (서비스 없음)."""
		try:
			response = future.result(timeout=timeout)
		except TimeoutError:   # concurrent.futures.TimeoutError 와 _expire_calls 의 TimeoutError 모두
			return 'failed', f'팔 토크 끄기 응답이 {config.MOTOR_SERVICE_TIMEOUT:g}초 안에 없습니다'
		except Exception as error:
			return 'failed', f'팔 토크 끄기 실패: {error}'
		if response is None:
			return 'unavailable', f'{config.ARM_TORQUE_SERVICE} 서비스가 없습니다'
		if not response.success:   # 드라이버는 확인하지 못한 팔 ID 만 1초마다 다시 끈다
			return 'failed', (
				f'팔 토크 끄기를 확인하지 못해 드라이버가 1초마다 다시 시도 중입니다 ({response.message})'
			)
		return 'ok', ARM_OFF_MESSAGE

	def _turn_arm_off(
		self, estop_seq: int | None = None, timeout: float = config.MOTOR_SERVICE_TIMEOUT,
	) -> tuple[str, str]:
		"""(막힘) 팔 토크를 끄고 timeout 까지 기다려 결과를 적는다 (_note_arm_off).

		Args:
			estop_seq: 비상 정지 대신 끄는 것이면 그 번호 (status.teleop_arm.estop 에 적는다).
			timeout: [s] 응답을 기다리는 시간.

		Returns:
			(state, 한국어 이유) (_arm_off_state).
		"""
		future = self._require_node().call_arm_torque(False)
		state, detail = self._arm_off_state(future, timeout)
		self._note_arm_off(state, detail if estop_seq is None else f'비상 정지: {detail}', estop_seq)
		return state, detail

	def _warn_if_arm_torqued(self) -> None:
		"""세션 없이 팔 토크가 켜져 있을 수 있으면 (arm_ready true · 이 게이트웨이가 켜고 끄기를 확인하지 못함)
		자율 주행 · 자동 주행 맵 생성 시작을 알린다. 막지는 않는다 (세션 중에는 이미 거절했다)."""
		with self._lock:
			is_maybe_on = self._is_arm_maybe_on
		if is_maybe_on or self._require_node().arm_ready() is True:
			self._emit('warn', ARM_TORQUED_MESSAGE)

	def _rest_arm_before_bringup_stop(self, is_forced: bool) -> CommandResult | None:
		"""(막힘) 로봇 끄기 앞: 이 게이트웨이가 켠 팔 토크가 남아 있을 수 있으면 세션을 닫고 팔을 접어 끈다
		(is_forced 면 접지 않고 힘을 뺀다). 끌 수 있으면 None, 아니면 conflict (이벤트로도 알린다).

		로봇이 꺼져 있거나 processes 가 어차피 거절할 때 (지도 작성 · 자율 주행 중) 는 팔을 건드리지 않는다.
		"""
		if self._processes.state('bringup') not in ('starting', 'running', 'external'):
			return None
		if any(self._processes.state(name) in ACTIVE_STATES for name in ('mapping', 'navigation')):
			return None
		with self._lock:
			if not self._is_arm_maybe_on:
				return None
			self._arm_stop_seq += 1   # 도는 팔 시작은 세션을 열지 않는다
		self._clear_arm_session()
		if not self._arm_lock.acquire(timeout=ARM_START_MAX_TIME):
			return error_result('conflict', '팔을 시작하는 중입니다. 잠시 뒤 다시 누르세요')
		try:
			with self._lock:   # 두 번 누름: 앞의 끄기가 이미 팔을 접고 껐을 수 있다
				is_maybe_on = self._is_arm_maybe_on
			reason = None
			if not is_maybe_on:
				pass
			elif config.MOTOR_DRIVER_NODE not in self._require_node().graph()[0]:
				# 드라이버가 없다: 팔 토크를 끌 길이 없고 로봇을 꺼도 바뀌지 않는다. 막지 않고 알린다
				self._emit('warn', ARM_DRIVER_GONE_STOP_MESSAGE)
			elif is_forced:
				self._emit('warn', ARM_FORCE_MESSAGE)
				state, detail = self._turn_arm_off()
				if state != 'ok':   # 강제: 그래도 끈다 (사람이 팔을 받치고 있다)
					self._emit('warn', f'팔 힘을 빼지 못했지만 로봇을 끕니다: {detail}')
			else:
				deadline = time.monotonic() + 2 * config.MOTOR_SERVICE_TIMEOUT + config.ARM_REST_TIMEOUT
				reason = self._rest_arm(deadline)
				if reason is not None:
					reason = f'{reason}. {ARM_FORCE_HINT}'
		finally:
			self._arm_lock.release()
		if reason is None:
			return None
		message = f'로봇을 끄지 않았습니다: {reason}'
		self._emit('error', message)
		return error_result('conflict', message)

	def _rest_arm_before_shutdown(self) -> None:
		"""(막힘, 최대 ARM_SHUTDOWN_REST_TIME) 게이트웨이 종료: 웹에서 켠 로봇이 돌고 이 게이트웨이가 켠 팔
		토크가 남아 있을 수 있으면 팔을 접어 끈다. 못 접으면 힘을 빼지 않는다 (지켜보는 사람 없이 떨어뜨리지 않게)."""
		bringup = self._processes.status()['bringup']
		with self._lock:
			is_maybe_on = self._is_arm_maybe_on
		if not (is_maybe_on and bringup['is_managed'] and bringup['state'] in ('starting', 'running')):
			return
		deadline = time.monotonic() + config.ARM_SHUTDOWN_REST_TIME
		reason = '팔 시작이 끝나지 않았습니다'
		if self._arm_lock.acquire(timeout=config.ARM_SHUTDOWN_REST_TIME / 2):
			try:
				reason = self._rest_arm(deadline)
			finally:
				self._arm_lock.release()
		if reason is not None:
			self._emit('warn', f'게이트웨이 종료: 팔을 접지 못해 팔은 토크를 켠 채 둡니다: {reason}')

	def _rest_arm(self, deadline: float) -> str | None:
		"""(막힘, _arm_lock 안, 세션은 닫은 뒤) lekiwi/arm/rest → status 가 rested (ARM_REST_TIMEOUT 까지) →
		arm_torque false 확인. 모두 deadline 안에. status.teleop_arm.rest 에 적는다.

		Returns:
			못 했으면 그 이유 (한국어, 팔 토크는 끄지 않았다), 됐으면 None. 그사이 비상 정지 · 팔 힘 빼기로
			꺼진 것을 확인했으면 접지 않고 None.
		"""
		node = self._require_node()
		self._set_arm_rest('resting', None)
		self._emit('info', ARM_RESTING_MESSAGE)
		reason = None
		if config.FOLLOWER_ARM_NODE not in node.graph()[0]:
			reason = '팔 노드가 없어 팔을 접을 수 없습니다'
		else:
			try:
				response = node.call_arm_rest().result(
					timeout=max(0.0, min(config.MOTOR_SERVICE_TIMEOUT, deadline - time.monotonic())),
				)
			except TimeoutError:
				reason = f'{config.ARM_REST_SERVICE} 응답이 없습니다'
			except Exception as error:
				reason = f'{config.ARM_REST_SERVICE} 호출 실패: {error}'
			else:
				if response is None:
					reason = f'{config.ARM_REST_SERVICE} 서비스가 없습니다'
				elif not response.success:
					reason = f'팔로워가 팔 접기를 거절했습니다: {response.message}'
		rest_deadline = min(deadline, time.monotonic() + config.ARM_REST_TIMEOUT)
		is_off = False
		warnings: list[str] = []
		while reason is None:
			with self._lock:
				is_off = not self._is_arm_maybe_on
			arm = node.snapshot()['arm']
			status = arm['status'] or {}
			if is_off or (_is_status_fresh(arm) and status.get('state') == 'rested'):
				# 팔로워는 램프 뒤 1초에 못 온 관절이 있어도 경고와 함께 rested 로 한다 (끄게 한다)
				warnings = list(status.get('warnings') or [])
				break
			if time.monotonic() >= rest_deadline:
				reason = (
					f'팔이 {config.ARM_REST_TIMEOUT:g}초 안에 접히지 않았습니다 '
					f'(팔로워 {status.get("state")}: {status.get("reason") or "-"})'
				)
				break
			time.sleep(READY_POLL_PERIOD)
		if reason is None and not is_off:
			state, detail = self._turn_arm_off(
				timeout=max(0.0, min(config.MOTOR_SERVICE_TIMEOUT, deadline - time.monotonic())),
			)
			if state != 'ok':
				reason = f'팔을 접었지만 팔 토크를 끄지 못했습니다: {detail}'
		if reason is not None:
			self._set_arm_rest('failed', reason)
		elif is_off:
			self._set_arm_rest('idle', '그사이 팔 토크가 꺼져 접지 않았습니다')
		else:
			self._set_arm_rest('rested', ' · '.join(warnings) or None)
			for warning in warnings:
				self._emit('warn', f'팔 접기: {warning}')
			self._emit('info', ARM_RESTED_MESSAGE)
		return reason

	def _set_arm_rest(self, state: str, reason: str | None) -> None:
		"""status.teleop_arm.rest 를 바꾸고 'arm' 묶음을 맞춘다 (접는 동안 구독한다)."""
		with self._lock:
			self._arm_rest = {'state': state, 'reason': reason, 'time': time.time()}
		self._update_wanted_streams()

	def _note_arm_off(self, state: str, message: str, estop_seq: int | None) -> None:
		"""(아무 스레드) 팔 토크 끄기 결과를 적는다. 껐으면 '켜져 있을 수 있음'을 지우고, 남아 있던 비상 정지의
		실패도 그 확인으로 바꾼다. estop_seq 가 마지막 비상 정지면 그 결과로 적는다 (더 새 것을 덮지 않는다)."""
		with self._lock:
			estop = self._estop_arm
			is_stale_failure = estop is not None and estop['state'] in ('failed', 'unavailable')
			if state == 'ok':
				self._is_arm_maybe_on = False
			if (estop_seq is not None and estop_seq == self._estop_seq) or (
				state == 'ok' and is_stale_failure
			):
				self._estop_arm = {'state': state, 'message': message, 'time': time.time()}

	def _publish_leader_in_ros(self, token: int, positions: list[float], received_at: float) -> None:
		"""(ROS 스레드) 그 세션이 그대로이고 ARM_LEADER_TIMEOUT 안에 받은 값만 보낸다 (밀린 값 · 비상
		정지 · 권한 끝 뒤의 값은 버린다)."""
		with self._lock:
			is_current = self._arm_session == token
		if is_current and time.monotonic() - received_at <= config.ARM_LEADER_TIMEOUT:
			self._node.publish_arm_goal(positions)

	def _watch_arm(self) -> None:
		"""(ROS 스레드, 조작 타이머) 팔 세션 중 자율 주행이 켜지거나 (터미널에서 켬 · 시작과 겹침) 팔로워 ·
		드라이버 팔 쓰기가 그래프에서 사라지면 세션을 닫고 이유를 알린다 (아무 데도 가지 않는 중계를 보이지
		않게). 리더 값이 ARM_LEADER_TIMEOUT 넘게 끊기면 한 번 알린다 (다시 오면 이어 보낸다)."""
		with self._lock:
			has_session = self._arm_session is not None
			is_lost = self._is_relaying and time.monotonic() - self._leader_at > config.ARM_LEADER_TIMEOUT
			if is_lost:
				self._is_relaying = False
		if is_lost:
			self._emit('warn', LEADER_LOST_MESSAGE)
		if not has_session:
			return
		node = self._require_node()
		reason = None
		if self._is_navigation_active():
			reason = '자율 주행이 켜짐'
		elif config.FOLLOWER_ARM_NODE not in node.graph()[0]:
			reason = '팔 노드가 사라짐'
		elif not node.has_arm_driver():
			reason = '드라이버의 팔 쓰기가 사라짐'
		if reason is not None and self._clear_arm_session():
			self._emit('warn', ARM_HELD_MESSAGE.format(reason=reason))

	def _publish_speed_in_ros(self) -> None:
		"""지금 자율 주행 속도를 보낸다 (Nav2 노드가 안 보이면 지나간다). 값은 보낼 때 읽는다: 거의 함께 온
		두 요청의 순서가 ROS 스레드에서 바뀌어도 마지막에 정한 값이 남는다."""
		with self._lock:
			percent = self._navigation_speed
		self._node.publish_speed_limit(percent)

	def _send_goal_in_ros(self, x: float, y: float, yaw: float) -> bool:
		"""속도를 먼저 보내고 목표를 보낸다 (막 다시 켠 controller_server 도 정한 속도로 출발하게)."""
		self._publish_speed_in_ros()
		return self._node.send_goal(x, y, yaw)

	def _shutdown_in_ros(self, is_powered: bool) -> concurrent.futures.Future | None:
		self._node.stop_tf_echo()
		self._node.drop_cmd_vel_publisher()
		self._node.cancel_own_goal()   # 비상 정지와 달리 RViz 등 남의 목표는 두고 우리 것만
		self._node.set_view_node(False)
		return self._node.call_motor_power(False) if is_powered else None

	def _spin(self) -> None:
		"""ROS 스레드 본체."""
		while self._is_spinning and self._node.context.ok():
			try:
				self._executor.spin_once(timeout_sec=SPIN_TIMEOUT)
			except Exception:  # 콜백 하나의 오류로 ROS 스레드가 죽지 않게 한다
				if self._node.context.ok():
					logger.exception('ROS 콜백에서 오류가 났습니다')

	# ── 탐색 스레드 (자동 주행 맵 생성) ────────────────────
	def _exploration_thread(
		self, token: int, explorer: explore.Explorer, started: float,
	) -> None:
		"""(탐색 스레드) 자동 주행 한 번. 끝나면 번호 · 구독을 놓고, 저장할 끝이면 지도를 저장한다.

		Args:
			token: 이 자동 주행의 번호 (TeleopSupervisor.start_autonomous).
			explorer: explore.Explorer.
			started: 시작 time.monotonic().
		"""
		try:
			state, reason, should_save, step = 'stopped', EXPLORER_ERROR_MESSAGE, False, None
			try:
				state, reason, should_save, step = self._explore(token, explorer, started)
			except Exception:
				logger.exception('자동 주행 탐색기에서 오류가 났습니다')
			self._end_run(token, 'saving' if should_save else state, reason, step, started)
			if not should_save:
				self._emit('info' if state == 'done' else 'warn', reason)
				return
			self._emit('info', f'자동 주행 맵 생성을 마쳤습니다: {reason}')
			saved_map, failure = self._auto_save()
			if failure is not None:
				reason = f'자동 저장 실패: {failure} (‘지도 저장’을 누르세요)'
			with self._lock:
				self._exploration.update(state='done', reason=reason, saved_map=saved_map)
		finally:
			self._explore_lock.release()

	def _explore(
		self, token: int, explorer: explore.Explorer, started: float,
	) -> tuple[str, str, bool, explore.Step | None]:
		"""(탐색 스레드) EXPLORE_CONTROL_PERIOD 마다 Snapshot → step → 끝낼 이유 → status · 명령.

		Returns:
			(끝난 state, 이유, 지도를 자동 저장할지, 탐색기가 끝낸 마지막 Step 또는 None).
		"""
		while True:
			tick = time.monotonic()
			step = explorer.step(self._explore_snapshot())   # 느린 계획은 explore 가 경고한다
			reason = self._explore_end_reason(token)   # step 이 도는 사이 끝났을 수 있다
			if reason is not None:
				return 'stopped', reason, False, None
			if step.is_finished:
				reason = step.reason or EXPLORE_STOPPED_MESSAGE
				return step.state, reason, bool(step.should_save), step
			with self._lock:
				self._exploration.update(
					_step_status(step), elapsed=round(time.monotonic() - started, 1),
				)
			self._teleop.set_autonomous(token, float(step.vx), float(step.wz))
			rest = config.EXPLORE_CONTROL_PERIOD - (time.monotonic() - tick)
			if self._explore_stop.wait(max(0.0, rest)):
				return 'stopped', SHUTDOWN_EXPLORE_MESSAGE, False, None

	def _explore_snapshot(self) -> explore.Snapshot:
		"""(탐색 스레드) 잠금으로 지킨 마지막 값들로 Snapshot (ROS 스레드를 기다리지 않는다)."""
		inputs = self._require_node().explore_inputs()
		vx, _vy, wz = self._teleop.output()
		return explore.Snapshot(now=time.monotonic(), sent=(vx, wz), **inputs)

	def _explore_end_reason(self, token: int) -> str | None:
		"""(탐색 스레드) 이 자동 주행을 끝낼 이유. 계속해도 되면 None (M1 (4)).

		번호가 없어졌으면 끝낸 쪽이 적은 이유다 (권한을 끝낸 쪽은 _lease_lock 안에서 적는다).
		"""
		if not self._teleop.is_autonomous(token):
			with self._lease_lock, self._lock:
				return self._explore_reason or EXPLORE_STOPPED_MESSAGE
		with self._lock:
			is_latched = self._estop is not None
			is_allowed, blocked = self._allowed
		if is_latched:
			return ESTOP_EXPLORE_MESSAGE
		if not is_allowed:   # ready 보다 먼저: 조작할 수 없으면 게이트웨이가 토크를 끄기 때문이다
			return f'{EXPLORE_STOPPED_MESSAGE}: {blocked}'
		if self._require_node().motor_ready()[0] is not True:
			return READY_LOST_MESSAGE
		if self._processes.state('mapping') not in RUNNING_STATES:
			return MAPPING_GONE_MESSAGE
		return None

	def _end_run(
		self, token: int, state: str, reason: str, step: explore.Step | None, started: float,
	) -> None:
		"""자동 주행이 끝났다: 번호를 지우고 (로봇이 멈춘다) status 를 끝 상태로, 구독은 보통 규칙대로
		(화면이 원하지 않으면 CAMERA_RELEASE_DELAY 뒤에) 푼다."""
		self._teleop.stop_autonomous(token)
		with self._lock:
			self._explore_token = None
			self._exploration.update(
				{} if step is None else _step_status(step), state=state, reason=reason,
				elapsed=round(time.monotonic() - started, 1), target=None, path=None,
			)
		self._update_wanted_streams()

	def _auto_save(self) -> tuple[str | None, str | None]:
		"""(탐색 스레드) S7: 지도 저장을 맡기고 결과를 기다린다.

		다른 저장이 도는 동안(conflict)은 EXPLORE_SAVE_RETRY 마다 SAVE_MAP_TIMEOUT 까지 다시 해 본다.
		결과 이벤트는 ProcessManager 가 낸다 (시작도 못 했을 때만 여기서 알린다).

		Returns:
			(저장한 지도 파일 이름, None) 또는 (None, 실패 이유).
		"""
		deadline = time.monotonic() + config.SAVE_MAP_TIMEOUT
		while True:
			requested_at = time.time()
			result = self._processes.save_map()
			if result['ok']:
				break
			is_busy = (   # 지도 작성 중인데 거절: 다른 저장이 돌고 있다
				result['code'] == 'conflict'
				and self._processes.state('mapping') in RUNNING_STATES
			)
			if not is_busy or time.monotonic() >= deadline or self._explore_stop.wait(
				EXPLORE_SAVE_RETRY,
			):
				self._emit('error', f'자동 저장 실패: {result["message"]}')
				return None, result['message']
		deadline = time.monotonic() + SAVE_MAP_MAX_TIME
		while time.monotonic() < deadline and not self._explore_stop.wait(EXPLORE_SAVE_POLL):
			save = self._processes.status()['save_map']
			if save['state'] != 'running' and (save['since'] or 0.0) >= requested_at:
				if save['state'] == 'succeeded':
					return save['map'], None
				return None, save['detail'] or '저장하지 못했습니다'
		return None, '저장 결과를 받지 못했습니다'

	def _explore_blocked(self) -> str | None:
		"""자동 주행을 시작할 수 없는 이유 (한국어), 시작해도 되면 None (SPEC §4 · S4 · S5)."""
		with self._lock:
			is_arm_active = self._arm_session is not None
		if is_arm_active:   # 팔을 편 채 스스로 움직이지 않게 (탐색기는 접은 팔 모양만 안다)
			return ARM_SESSION_MESSAGE
		if self._processes.state('mapping') not in RUNNING_STATES:
			return '지도 작성이 실행 중일 때 시작할 수 있습니다'
		if self._processes.status()['save_map']['state'] == 'running':
			return '지도를 저장하는 중입니다'
		is_allowed, reason = self._compute_allowed()
		if not is_allowed:
			return reason
		node = self._require_node()
		with self._lock:
			is_latched = self._estop is not None
		# 이 권한으로 켠 토크여야 권한이 끝날 때 끈다 (다른 도구가 켠 토크는 남는다)
		if is_latched or node.motor_ready()[0] is not True or not self._teleop.is_powered_by_us():
			return POWER_FIRST_MESSAGE
		if node.map_message() is None:
			return '지도를 기다리는 중입니다'
		return None

	def _note_explore_end(self, reason: str) -> int | None:
		"""(아무 스레드) 자동 주행 중이면 끝난 이유를 적는다 (먼저 적은 것이 이긴다).

		번호를 지우기 전에, 또는 번호를 지운 그 _lease_lock 안에서 부른다 (클래스 설명).

		Returns:
			지금 자동 주행 번호. 자동 주행 중이 아니면 None.
		"""
		with self._lock:
			if self._explore_token is not None and self._explore_reason is None:
				self._explore_reason = reason
			return self._explore_token

	def _end_exploration(self, reason: str) -> bool:
		"""(아무 스레드) 자동 주행 중이면 이유를 적고 번호를 지운다 (로봇이 바로 멈춘다).

		Returns:
			자동 주행 중이었으면 True.
		"""
		token = self._note_explore_end(reason)
		if token is not None:
			self._teleop.stop_autonomous(token)
		return token is not None

	# ── 내부 ───────────────────────────────────────────────
	def _start_navigation(self, client_id: str, map_name: Any) -> CommandResult:
		"""(막힘, 최대 약 7 s) 목표 0 → 퍼블리셔 삭제 → 우리가 켠 토크 끔 → ready false 대기 →
		지도 경로 → launch (SPEC §7-12). 성공하면 걸려 있던 비상 정지가 풀린다."""
		if not self._features['has_navigation']:
			return error_result('unavailable', NO_NAVIGATION_MESSAGE)
		with self._lock:
			is_arm_active = self._arm_session is not None
		if is_arm_active:   # 겹쳐 시작한 팔은 조작 타이머가 닫는다 (_watch_arm)
			return error_result('conflict', ARM_SESSION_MESSAGE)
		map_path = map_files.resolve_map(config.map_dir(), map_name)
		if map_path is None:
			return error_result('bad_request', f'지도 파일을 찾을 수 없습니다: {map_name}')
		estop_seq = self._estop_seq_if_settled()
		if estop_seq is None:
			return error_result('conflict', ESTOP_HOLDOFF_MESSAGE)
		# 감독 도구는 cmd_vel 발행자 · 구독자가 Nav2 · 베이스 하나씩이 아니면 토크를 켜지 않고 끝난다
		self._run_in_ros(self._require_node().refresh_graph)
		_names, _count, others, base_count = self._require_node().graph()
		graph_reason = blocked_reason(False, True, False, True, others, base_count)
		if graph_reason is not None:
			return error_result('conflict', f'자율 주행을 시작하지 않았습니다: {graph_reason}')
		if not self._navigation_lock.acquire(blocking=False):
			return error_result('conflict', '자율 주행을 시작하는 중입니다')
		try:
			self._is_preparing_navigation = True   # 이 동안 조작 타이머가 퍼블리셔를 만들지 않는다
			token = self._teleop.lease_token(client_id)
			self._note_explore_end(NAVIGATION_EXPLORE_MESSAGE)
			self._teleop.stop()
			self._run_in_ros(self._require_node().drop_cmd_vel_publisher)
			if self._teleop.take_powered():
				result = self._wait_service(
					self._require_node().call_motor_power(False), config.MOTOR_POWER_SERVICE,
				)
				if not result['ok']:
					if token is not None:
						self._teleop.mark_powered(token)
					return error_result(
						result['code'], f'바퀴 토크를 끄지 못해 시작하지 않았습니다: {result["message"]}',
					)
				self._emit('info', '자율 주행을 위해 바퀴 토크를 껐습니다')
			is_ready_off = self._wait_motor_ready(
				lambda ready, _seq: ready is not True, NAVIGATION_READY_OFF_TIMEOUT,
			)
			if not is_ready_off:   # 감독 도구는 ready 가 true 면 토크를 켜지 않고 끝나 버린다
				return error_result(
					'conflict',
					'바퀴 토크가 켜져 있어 자율 주행을 시작하지 않았습니다. 다른 조작 도구를 끄세요',
				)
			if self._estop_seq != estop_seq:
				return error_result('conflict', f'{ESTOPPED_MESSAGE}. 자율 주행을 시작하지 않았습니다')
			# 새 자율 주행의 감독 도구를 본다
			self._has_seen_guard = self._is_guard_gone = False
			self._guard_missing_since = None
			result = self._processes.start('navigation', {'map_path': str(map_path)})
			if result['ok']:
				self._warn_if_arm_torqued()
				# 그사이 비상 정지가 눌렸으면 걸린 채로 둔다: 조작 노드가 없어 감독 도구가 켜지 못한다
				self._clear_estop(estop_seq)
			return result
		finally:
			self._is_preparing_navigation = False
			self._navigation_lock.release()

	def _power_on(self, client_id: str) -> CommandResult:
		"""(막힘, 최대 POWER_ON_MAX_TIME) 서비스 찾기 → power true → ready true 대기. 실패하면
		power false.

		비상 정지 직후(ESTOP_HOLDOFF)에는 거절하고, 켜는 사이 비상 정지가 눌렸으면 실패로 끝낸다.
		성공하면 걸려 있던 비상 정지가 풀린다.
		"""
		node = self._require_node()
		token = self._teleop.lease_token(client_id)
		self._run_in_ros(node.refresh_graph)   # 그래프는 1 Hz 로 읽는다: 켜기 전에 새로 본다
		is_allowed, reason = self._compute_allowed()
		if token is None or not is_allowed:
			return error_result('conflict', reason or NOT_OPERATOR_MESSAGE)
		estop_seq = self._estop_seq_if_settled()
		if estop_seq is None:
			return error_result('conflict', ESTOP_HOLDOFF_MESSAGE)

		def is_superseded() -> bool:
			"""그사이 비상 정지가 눌렸거나 권한이 바뀌어 더 기다리거나 켤 필요가 없다."""
			return self._estop_seq != estop_seq or self._teleop.lease_token(client_id) != token

		# 막 (다시) 켠 게이트웨이는 서비스를 아직 못 찾았을 수 있다: ROS 스레드에서 확인하며 기다린다
		deadline = time.monotonic() + config.MOTOR_SERVICE_DISCOVERY_TIMEOUT
		while not (is_superseded() or self._run_in_ros(node.is_power_service_ready)):
			if time.monotonic() >= deadline:
				break
			time.sleep(READY_POLL_PERIOD)
		_ready, ready_seq = node.motor_ready()
		if self._estop_seq != estop_seq:   # 찾는 사이 눌렸다: 켜지 않는다
			result = error_result('conflict', ESTOPPED_MESSAGE)
		elif self._teleop.lease_token(client_id) != token:
			result = error_result('conflict', NOT_OPERATOR_MESSAGE)
		else:
			result = self._wait_service(node.call_motor_power(True), config.MOTOR_POWER_SERVICE)

		def is_settled(ready: bool | None, seq: int) -> bool:
			"""새 ready true 가 왔거나 더 기다릴 필요가 없다 (is_superseded)."""
			return (ready is True and seq > ready_seq) or is_superseded()

		if result['ok'] and not self._wait_motor_ready(is_settled, config.MOTOR_READY_TIMEOUT):
			result = error_result(
				'failed', f'{config.MOTOR_READY_TOPIC} 가 {config.MOTOR_READY_TIMEOUT:g}초 안에 '
				'true 가 되지 않았습니다',
			)
		if result['ok']:
			is_allowed, reason = self._compute_allowed()
			if not is_allowed or not self._teleop.mark_powered(token):
				result = error_result(
					'conflict', f'그사이 조작할 수 없게 되었습니다: {reason or "권한 없음"}',
				)
			elif not self._clear_estop(estop_seq):
				# mark_powered 다음에 본다: 그 뒤의 비상 정지는 take_powered 로 이 토크를 끈다
				self._teleop.take_powered()
				result = error_result('conflict', ESTOPPED_MESSAGE)
		if not result['ok']:
			self._wait_service(node.call_motor_power(False), config.MOTOR_POWER_SERVICE)
			message = f'바퀴 토크를 켜지 못했습니다: {result["message"]}'
			self._emit('error', message)
			return error_result(result['code'], message)
		self._emit('info', '바퀴 토크를 켰습니다')
		return ok_result('바퀴 토크를 켰습니다')

	def _power_off_by_operator(self) -> CommandResult:
		"""(막힘) holder 의 토크 끄기."""
		self._note_explore_end(POWER_OFF_EXPLORE_MESSAGE)
		self._teleop.stop()
		result = self._wait_service(
			self._require_node().call_motor_power(False), config.MOTOR_POWER_SERVICE,
		)
		if not result['ok']:
			return error_result(result['code'], f'바퀴 토크를 끄지 못했습니다: {result["message"]}')
		self._teleop.take_powered()
		self._emit('info', '바퀴 토크를 껐습니다')
		return ok_result('바퀴 토크를 껐습니다')

	def _power_off_async(self, message: str | None) -> None:
		"""토크를 끄고 결과를 이벤트로 알린다 (기다리지 않는다. shutdown 은 기다린다).

		Args:
			message: 성공했을 때 알릴 문장. None 이면 실패만 알린다 (비상 정지는 한 번만 알린다).
		"""
		future = self._require_node().call_motor_power(False)
		with self._lock:
			self._power_offs = [old for old in self._power_offs if not old.done()] + [future]
		future.add_done_callback(lambda done: self._report_power_off(done, message))

	def _report_power_off(self, future: concurrent.futures.Future, message: str | None) -> None:
		try:
			response = future.result()
		except Exception as error:
			self._emit('error', f'바퀴 토크 끄기 실패: {error}')
			return
		if response is None:
			# 서비스를 끝내 못 찾았다. 베이스가 없으면 토크도 꺼져 있지만, 있으면 켜져 있을 수 있다
			node = self._node
			if node is not None and config.BASE_CONTROLLER_NODE in node.graph()[0]:
				self._emit('error', POWER_OFF_UNREACHABLE_MESSAGE)
			return
		if not response.success:
			self._emit('error', f'바퀴 토크 끄기 거부: {response.message}')
		elif message is not None:
			self._emit('info', message)

	def _finish_lease(self, ended: dict, message: str, level: str) -> None:
		"""(아무 스레드, _lease_lock 안) 권한이 끝났다 (놓음 · 신호 끊김 · 연결 끊김 · 넘겨줌).

		SPEC §7-3. post 를 이벤트보다 먼저 한다: shutdown 이 이 정리를 기다릴 수 있게.
		권한이 끝나며 지운 자동 주행 번호의 이유도 같은 _lease_lock 안에서 적는다.
		"""
		is_arm_held = self._clear_arm_session()   # 이 권한의 리더 값을 더 보내지 않는다 (팔 토크는 둔다)
		if self._node is not None:
			self._node.post(self._finish_lease_in_ros, ended['is_powered'])
		self._calibration.abort('조작 권한이 끝났습니다')   # 새로 보정은 그 권한의 것이다
		self._note_explore_end(message)
		self._emit(level, message)
		if is_arm_held:
			self._emit('info', ARM_HELD_MESSAGE.format(reason='조작 권한이 끝남'))

	def _is_view_wanted(self) -> bool:
		"""조작 노드를 둘지: holder 가 있고 비상 정지가 걸려 있지 않을 때만 (감독 도구가 이것을 본다)."""
		with self._lock:
			is_latched = self._estop is not None
		return not is_latched and self._teleop.holder() is not None

	def _estop_seq_if_settled(self) -> int | None:
		"""지금 비상 정지 번호. 마지막 비상 정지 뒤 ESTOP_HOLDOFF 가 지나지 않았으면 None."""
		with self._lock:
			if time.monotonic() - self._estop_at < config.ESTOP_HOLDOFF:
				return None
			return self._estop_seq

	def _clear_estop(self, estop_seq: int) -> bool:
		"""estop_seq 뒤로 비상 정지가 없었으면 걸린 비상 정지를 풀고 True, 있었으면 False."""
		with self._lock:
			if self._estop_seq != estop_seq:
				return False
			self._estop = None
			return True

	def _compute_allowed(self) -> tuple[bool, str | None]:
		"""(is_allowed, blocked_reason). 그래프는 1 Hz 로 읽은 마지막 값."""
		node = self._require_node()
		node_names, _count, others, base_count = node.graph()
		reason = blocked_reason(
			self._is_read_only,
			self._teleop.holder() is not None,
			self._is_navigation_active(),
			config.BASE_CONTROLLER_NODE in node_names,
			others,
			base_count,
		)
		return reason is None, reason

	def _is_navigation_active(self) -> bool:
		"""자율 주행이 켜져 있거나 켜지는 중 (직접 조작 · 팔 시작을 막는다)."""
		return (
			self._is_preparing_navigation or self._processes.state('navigation') in ACTIVE_STATES
		)

	def _arm_start(self, client_id: str) -> CommandResult:
		"""(막힘, _arm_lock 안) arm_start 의 본체. 처음 거절만 빼고 실패는 이벤트로도 알린다."""
		node = self._require_node()
		token = self._teleop.lease_token(client_id)
		with self._lock:
			stop_seq = self._arm_stop_seq
		self._run_in_ros(node.refresh_graph)   # 팔로워 노드 · arm_ready 발행자를 새로 본다 (그래프는 1 Hz)
		arm = node.snapshot()['arm']
		# 'arm' 묶음은 arm_start 가 막 구독했을 수 있다: 팔로워가 있으면 상태를 잠시 기다린다
		deadline = time.monotonic() + ARM_STATUS_WAIT
		while (
			not _is_status_fresh(arm) and config.FOLLOWER_ARM_NODE in node.graph()[0]
			and time.monotonic() < deadline
		):
			time.sleep(READY_POLL_PERIOD)
			arm = node.snapshot()['arm']
		blocked = self._arm_blocked(arm, node.graph()[0])
		if token is None or blocked is not None:
			return error_result('conflict', blocked or NOT_OPERATOR_MESSAGE)
		estop_seq = self._estop_seq_if_settled()
		if estop_seq is None:
			return error_result('conflict', ESTOP_HOLDOFF_MESSAGE)
		# 앞 비상 정지의 팔 끄기가 (서비스를 찾느라) 아직 가는 중이면 끝나기를 기다린다: 늦게 간 그 끄기가 이 세션의
		# 팔을 떨어뜨리지 않게, 그리고 비상 정지의 끄기를 버리지 않게 (취소하지 않고 기다리는 쪽을 골랐다)
		with self._lock:
			arm_offs = [future for future in self._arm_offs if not future.done()]
		if arm_offs:
			concurrent.futures.wait(
				arm_offs, timeout=config.MOTOR_SERVICE_DISCOVERY_TIMEOUT + config.MOTOR_SERVICE_TIMEOUT)
			if not all(future.done() for future in arm_offs):
				return error_result('conflict', '비상 정지의 팔 토크 끄기가 아직 끝나지 않았습니다. 잠시 뒤 다시 누르세요')
		# 막 (다시) 켠 게이트웨이는 서비스를 아직 못 찾았을 수 있다 (_power_on 과 같다)
		deadline = time.monotonic() + config.MOTOR_SERVICE_DISCOVERY_TIMEOUT
		while self._estop_seq == estop_seq and not self._run_in_ros(node.is_arm_torque_service_ready):
			if time.monotonic() >= deadline:
				break
			time.sleep(READY_POLL_PERIOD)
		with self._lock:
			# 번호를 본 그 잠금 안에서 큐에 넣는다: 뒤의 비상 정지가 넣는 끄기는 늘 이 켜기 뒤에 간다
			is_sent = self._estop_seq == estop_seq
			if is_sent:
				self._is_arm_maybe_on = True
				future = node.call_arm_torque(True)
		if is_sent:
			result = self._wait_service(future, config.ARM_TORQUE_SERVICE)
		else:   # 찾는 사이 눌렸다: 켜지 않는다
			result = error_result('conflict', ESTOPPED_MESSAGE)
		if result['ok']:
			is_navigation_active = self._is_navigation_active()   # 프로세스 잠금은 _lock 밖에서
			with self._lease_lock, self._lock:   # 권한 끝 · 비상 정지 · 팔 멈춤 · 자동 주행 시작과 한 번에
				if self._estop_seq != estop_seq:
					result = error_result('conflict', ESTOPPED_MESSAGE)
				elif self._arm_stop_seq != stop_seq:   # 팔 멈춤: 토크는 둔다, 팔 힘 빼기: 이 뒤에 끈다
					result = error_result('conflict', ARM_INTERRUPTED_MESSAGE)
				elif self._teleop.lease_token(client_id) != token:   # 권한 끝: 토크는 둔다
					result = error_result('conflict', NOT_OPERATOR_MESSAGE)
				elif is_navigation_active or self._explore_token is not None:
					result = error_result('conflict', '그사이 자율 주행 · 자동 주행 맵 생성이 시작되었습니다')
				else:
					self._arm_session = token
					self._leader_at, self._is_relaying = None, False
		if self._estop_seq != estop_seq:
			# 비상 정지가 끈다 (이 켜기 뒤에 간다). 응답을 받았으니 한 번 더 끄고, 못 끄면 그 비상 정지의 실패로 적는다
			result = error_result('conflict', ESTOPPED_MESSAGE)
			if is_sent:
				state, detail = self._turn_arm_off(self._estop_seq)
				if state != 'ok':
					self._emit('error', f'비상 정지: {detail}')
		if not result['ok']:
			message = f'팔을 시작하지 못했습니다: {result["message"]}'
			self._emit('error', message)
			return error_result(result['code'], message)
		with self._lock:
			if self._arm_session == token:
				self._arm_rest = dict(IDLE_ARM_REST)
		if self._estop_seq != estop_seq:   # 세션을 연 바로 뒤에 눌렸다: 그 비상 정지가 세션을 닫았다
			return error_result('conflict', f'팔을 시작하지 못했습니다: {ESTOPPED_MESSAGE}')
		self._emit('info', ARM_STARTED_MESSAGE)
		return ok_result(ARM_STARTED_MESSAGE)

	def _arm_blocked(self, arm: dict, node_names: frozenset[str]) -> str | None:
		"""팔 시작을 막는 이유 (한국어, 위가 먼저), 시작해도 되면 None (status.teleop_arm.blocked_reason).

		Args:
			arm: OperatorNode.snapshot()['arm'].
			node_names: 그래프의 노드 이름.
		"""
		with self._lock:
			is_latched = self._estop is not None
			is_exploring = self._explore_token is not None
		if self._is_read_only:
			return '보기 전용 모드입니다'
		if self._teleop.holder() is None:
			return '조작 권한이 없습니다'
		if is_latched:
			return '비상 정지가 걸려 있습니다. 먼저 ‘조작 켜기’로 푸세요'
		if self._is_navigation_active():
			return '자율 주행 중에는 팔을 시작할 수 없습니다'
		if is_exploring:
			return '자동 주행 맵 생성 중에는 팔을 시작할 수 없습니다'
		if config.FOLLOWER_ARM_NODE not in node_names:
			return '팔 노드가 없습니다 (로봇 · 팔을 켜세요)'
		if not arm['has_driver']:
			return (
				f'모터 드라이버가 팔 명령을 받지 않습니다 ({config.ARM_READY_TOPIC} 없음, '
				'bringup use_arm:=true)'
			)
		if not _is_status_fresh(arm):
			return f'팔로워 상태({config.ARM_STATUS_TOPIC})가 {config.ARM_STATUS_MAX_AGE:g}초 안에 오지 않았습니다'
		return None

	def _clear_arm_session(self) -> bool:
		"""(아무 스레드, _lock 밖) 팔 원격 조작 세션을 닫고 'arm' 묶음을 화면 규칙대로 돌린다 (팔 토크는 그대로).

		Returns:
			세션이 열려 있었으면 True.
		"""
		with self._lock:
			is_open = self._clear_arm_session_locked()
		self._update_wanted_streams()
		return is_open

	def _clear_arm_session_locked(self) -> bool:
		"""(_lock 안) 세션을 닫는다: 리더 값을 더 보내지 않는다. 세션이 열려 있었으면 True."""
		is_open = self._arm_session is not None
		self._arm_session = None
		self._leader_at, self._is_relaying = None, False
		return is_open

	def _check_operator(self, client_id: str) -> CommandResult | None:
		"""보기 전용이거나 holder 가 아니면 거부 결과."""
		if self._is_read_only:
			return error_result('read_only', READ_ONLY_MESSAGE)
		if not self._teleop.is_operator(client_id):
			return error_result('not_operator', NOT_OPERATOR_MESSAGE)
		return None

	def _check_navigation_command(
		self, client_id: str, values: tuple[Any, ...],
	) -> CommandResult | None:
		"""초기 위치 · 목표: holder, 유한한 숫자, 자율 주행 실행 중, 감독 도구가 살아 있음."""
		denied = self._check_operator(client_id)
		if denied is not None:
			return denied
		if not all(is_finite_number(value) for value in values):
			return error_result('bad_request', 'x · y · yaw 는 유한한 숫자여야 합니다')
		if self._processes.state('navigation') not in ('running', 'external'):
			return error_result('conflict', '자율 주행이 켜져 있지 않습니다')
		if self._is_guard_gone or self._guard_missing_since is not None:   # 안 보이는 동안 바로 막는다
			return error_result('conflict', GUARD_GONE_MESSAGE)
		return None

	def _wait_service(self, future: concurrent.futures.Future, service: str) -> CommandResult:
		"""(막힘) 서비스 응답을 MOTOR_SERVICE_TIMEOUT 까지 기다려 결과로 바꾼다."""
		try:
			response = future.result(timeout=config.MOTOR_SERVICE_TIMEOUT)
		except concurrent.futures.TimeoutError:
			return error_result(
				'failed', f'{service} 응답이 {config.MOTOR_SERVICE_TIMEOUT:g}초 안에 없습니다',
			)
		except Exception as error:
			return error_result('failed', f'{service} 호출 실패: {error}')
		if response is None:
			return error_result('unavailable', f'{service} 서비스가 없습니다. 먼저 로봇을 켜세요')
		if not response.success:
			return error_result('failed', f'{service} 거부: {response.message}')
		return ok_result(response.message)

	def _wait_motor_ready(
		self, predicate: Callable[[bool | None, int], bool], timeout: float,
	) -> bool:
		"""(막힘) predicate(ready, 받은 개수) 가 참이 될 때까지 timeout 동안 기다린다."""
		node = self._require_node()
		deadline = time.monotonic() + timeout
		while not predicate(*node.motor_ready()):
			if time.monotonic() >= deadline:
				return False
			time.sleep(READY_POLL_PERIOD)
		return True

	def _run_in_ros(self, function: Callable[..., Any], *args: Any) -> Any:
		"""ROS 스레드에서 실행하고 ROS_CALL_TIMEOUT 까지 결과를 기다린다.

		Raises:
			RuntimeError: ROS 스레드가 응답하지 않을 때 (웹 쪽이 500 으로 알린다).
		"""
		future = self._require_node().post(function, *args)
		try:
			return future.result(timeout=ROS_CALL_TIMEOUT)
		except concurrent.futures.TimeoutError as error:
			raise RuntimeError('ROS 스레드가 응답하지 않습니다') from error

	def _require_node(self) -> 'OperatorNode':
		if self._node is None:
			raise RuntimeError('로봇 서비스가 시작되지 않았습니다')
		return self._node

	def _warnings(
		self, snapshot: dict, node_names: frozenset[str], others: tuple[str, ...],
		base_count: int,
	) -> list[str]:
		"""status.warnings: 끊긴 센서 · 모터 상태 비트 · 고온 · 중복 launch · cmd_vel 그래프 ·
		자율 주행 감독 도구 끝남."""
		warnings = []
		for topic, publisher, message in STALE_CHECKS:
			if topic == 'motor_states' and not self._features['has_motor_msgs']:
				continue
			if snapshot['rates'][topic] is None:   # 보는 화면이 없어 구독하지 않는다
				continue
			age = snapshot['ages'][topic]
			if publisher in node_names and (age is None or age > STALE_SENSOR_AGE):
				warnings.append(message)
		for motor in snapshot['motors']:
			name = f'모터 {motor["id"]} ({motor["name"]})'
			if motor['status']:
				labels = ', '.join(motor['status_labels']) or f'상태 {motor["status"]}'
				warnings.append(f'{name} 이상: {labels}')
			if motor['temperature'] >= MOTOR_HOT_TEMPERATURE:
				warnings.append(f'{name} 온도가 높습니다: {motor["temperature"]:.0f} ℃')
		warnings += self._processes.warnings()
		unexpected = [name for name in others if name.rsplit('/', 1)[-1] != NAV2_VELOCITY_NODE]
		if unexpected:
			warnings.append(f'cmd_vel 에 다른 발행자가 있습니다: {", ".join(unexpected)}')
		if base_count > 1:
			warnings.append(MULTIPLE_BASES_MESSAGE)
		if self._is_guard_gone and self._processes.state('navigation') in NAVIGATION_UP_STATES:
			warnings.append(GUARD_GONE_MESSAGE)
		if snapshot['is_scan_tf_missing']:
			warnings.append(f'라이다 TF({config.BASE_FRAME} ← 스캔 프레임)가 없어 스캔을 그릴 수 없습니다')
		return warnings

	def _emit(self, level: str, message: str) -> None:
		"""사용자에게 알릴 이벤트."""
		self._dispatch(make_event(level, message))

	def _dispatch(self, message: dict) -> None:
		"""이벤트 · 로그를 리스너(hub)에 넘긴다. 이벤트는 서버 로그에도 남긴다."""
		if message.get('type') == 'event':
			logger.log(LOG_LEVELS.get(message['level'], logging.INFO), message['message'])
		with self._lock:
			listeners = list(self._listeners)
		for listener in listeners:
			try:
				listener(message)
			except Exception:
				logger.exception('이벤트를 넘기지 못했습니다')

	def _stop_ros(self) -> None:
		"""ROS 스레드를 멈추고 노드 · executor · rclpy 를 끝낸다."""
		if self._node is None:
			return
		self._is_spinning = False
		self._thread.join(timeout=ROS_STOP_TIMEOUT)
		if self._thread.is_alive():
			logger.error('ROS 스레드가 %g 초 안에 멈추지 않아 노드를 지우지 않습니다', ROS_STOP_TIMEOUT)
		else:
			try:
				self._node.close()
				self._executor.shutdown(timeout_sec=ROS_STOP_TIMEOUT)
			except Exception:
				logger.exception('ROS 노드를 지우다 오류가 났습니다')
		import rclpy

		# 모듈 함수로 끝내야 기본 Context 가 비워져 같은 프로세스에서 다시 start() 할 수 있다
		rclpy.try_shutdown()
