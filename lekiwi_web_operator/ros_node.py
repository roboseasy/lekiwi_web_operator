"""웹 오퍼레이터의 ROS 노드 OperatorNode: 구독 · 상태 저장 · 서비스 · 자율 주행 · 명령 큐.

- rclpy 객체를 만들고 지우고 발행하고 부르는 일은 ROS 스레드(executor)에서만 한다.
  다른 스레드는 post() 로 함수를 넘긴다 (queue.SimpleQueue + guard condition).
- cmd_vel 은 구독하지 않는다. Nav2 감독 도구(lekiwi_nav2_motor_guard)가 cmd_vel 구독자가
  베이스 컨트롤러 하나뿐인지 0.5 s 마다 검사한다.
- 콜백은 마지막 메시지 · 수신 시각만 저장하고, 변환(스캔 점 · 지도 PNG · 상태 dict)은 읽는 스레드가
  메시지마다 한 번만 한다. 상태로만 쓰는 odom · IMU · 모터 상태는 raw 로 받아 그때 푼다.
- rclpy 는 메시지 하나에 Pi 4 코어 약 2 ms 를 쓴다 (대부분 executor 가 대기 집합을 훑는 일). 그래서
  빈도가 높은 토픽(STREAMS)은 그 채널을 보는 화면이 있을 때만 구독하고, 지도 위 로봇 자세는 /tf
  (Cartographer 약 250 Hz) 대신 tf2_echo 자식 프로세스(tf_echo.TfEcho)가 10 Hz 로 찍는 것을 읽는다
  (/tf_static 의 라이다 TF 만 직접 받는다). 자율 주행은 ActionClient 대신 액션의 서비스 3개와
  status 만 쓴다 (피드백 약 100 Hz 를 받지 않는다). 노드에는 쓰지 않는 서비스 · QoS 이벤트 처리기를
  두지 않는다 (QuietNode).
- lekiwi_msgs · nav2_msgs 를 못 불러오면 그 기능만 끈다 (HAS_MOTOR_MSGS · HAS_NAVIGATION).
"""

import collections
import concurrent.futures
import itertools
import json
import logging
import math
import queue
import random
import re
import socket
import threading
import time
import uuid
from typing import Any, Callable, Final

import numpy as np
import tf2_ros
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.srv import GetParameters
from rclpy.client import Client
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import (
	DurabilityPolicy,
	QoSProfile,
	ReliabilityPolicy,
	qos_profile_action_status_default,
	qos_profile_sensor_data,
)
from rclpy.serialization import deserialize_message
from rclpy.task import Future as RosFuture
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Imu, JointState, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger
from tf2_msgs.msg import TFMessage

from lekiwi_web_operator import config, explore, map_codec, tf_echo
from lekiwi_web_operator.teleop import ZERO_COMMAND

try:
	from lekiwi_msgs.msg import MotorStateArray
except ImportError:  # 워크스페이스를 source 하지 않았다: 모터 상태만 끈다
	MotorStateArray = None
try:
	from lekiwi_msgs.msg import ArmRegisters
except ImportError:  # 옛 lekiwi_msgs: 보정 탭의 모터 값만 없다
	ArmRegisters = None
try:
	from nav2_msgs.action import NavigateToPose
	from nav2_msgs.msg import SpeedLimit
except ImportError:  # Nav2 가 없다: 자율 주행만 끈다
	NavigateToPose = SpeedLimit = None

HAS_MOTOR_MSGS: Final[bool] = MotorStateArray is not None
HAS_NAVIGATION: Final[bool] = NavigateToPose is not None

logger = logging.getLogger(__name__)

RATE_WINDOW: Final[float] = 2.0           # [s] 수신 빈도를 재는 구간
DISCOVERY_RETRY_PERIOD: Final[float] = 0.05   # [s] 서비스 서버를 아직 못 찾았을 때 다시 보는 간격
# 보는 화면이 있을 때만 구독하는 묶음 = 그것을 원하는 채널 (robot.STREAM_CHANNELS 와 같다).
# 'map' 은 지도 위 로봇 자세 (tf2_echo 자식 프로세스), 'arm' 은 팔로워 상태 · 관절 · arm_ready
STREAMS: Final[tuple[str, ...]] = (
	'scan', 'odom', 'motors', 'imu', 'map', 'arm', 'calibration',
	*(f'camera/{name}' for name in config.CAMERA_TOPICS),
)
# rates · ages 의 토픽 → 그 토픽을 구독하는 묶음 ('map' 은 빈도를 재지 않는다)
TOPIC_STREAMS: Final[dict[str, str]] = {
	'scan': 'scan', 'odom': 'odom', 'imu': 'imu', 'motor_states': 'motors',
	'arm_status': 'arm', 'arm_state': 'arm',
	**{f'{name}_camera': f'camera/{name}' for name in config.CAMERA_TOPICS},
}
# lekiwi/arm/status JSON 에서 화면에 보내는 키 (lekiwi_lerobot README)
ARM_STATUS_KEYS: Final[tuple[str, ...]] = (
	'state', 'reason', 'robot_id', 'diffs', 'ramp_progress', 'warnings',
)
# 상태로만 보내는 높은 빈도 묶음 → (rates 토픽, 메시지, ROS 토픽). raw 로 받아 상태를 만들 때 푼다
RAW_STREAMS: Final[dict[str, tuple[str, Any, str]]] = {
	'odom': ('odom', Odometry, config.ODOM_TOPIC),
	'imu': ('imu', Imu, config.IMU_TOPIC),
	'motors': ('motor_states', MotorStateArray, config.MOTOR_STATES_TOPIC),
}
AGE_TOPICS: Final[tuple[str, ...]] = ('scan', 'odom', 'imu', 'motor_states')
# [s] 이보다 오래 안 온 odom · IMU · 모터 상태는 지금 값처럼 보내지 않는다 (로봇을 끈 뒤 등)
STALE_DATA_AGE: Final[float] = RATE_WINDOW
SCAN_TF_REFRESH: Final[float] = 5.0       # [s] base_footprint ← 라이다 TF 를 다시 읽는 간격
PLAN_MAX_POINTS: Final[int] = 300
TF_AUTHORITY: Final[str] = 'default_authority'
# 끝점은 보이는데 그 노드 정보가 아직 안 온 동안 rmw 가 쓰는 이름 (발견 중)
UNKNOWN_NODE_NAME: Final[str] = '_NODE_NAME_UNKNOWN_'
# [s] 이름이 같은 다른 게이트웨이와 cmd_vel 이 겹치면 무작위로 이만큼 비킨다: 둘이 같은 박자로
# 지우고 다시 만들기를 되풀이하지 않고 하나만 남게 한다
PEER_HOLD: Final[tuple[float, float]] = (1.0, 4.0)
# RViz 2D Pose Estimate 와 같은 공분산: x, y [m^2], yaw [rad^2]
INITIAL_POSE_COVARIANCE: Final[tuple[float, float, float]] = (0.25, 0.25, 0.06853891945200942)
MOTOR_STATUS_LABELS: Final[tuple[tuple[int, str], ...]] = (   # lekiwi_msgs/MotorState STATUS_*
	(1, '전압'), (2, '각도 센서'), (4, '과열'), (8, '과전류'), (32, '과부하'),
)
GOAL_RESULTS: Final[dict[int, tuple[str, str]]] = {
	GoalStatus.STATUS_SUCCEEDED: ('succeeded', '목표에 도착했습니다'),
	GoalStatus.STATUS_CANCELED: ('canceled', '목표를 취소했습니다'),
	GoalStatus.STATUS_ABORTED: ('aborted', '목표에 가지 못했습니다'),
}
# 취소할 수 있는 목표 상태 (action_msgs/GoalStatus)
GOAL_ACTIVE_STATES: Final[tuple[int, ...]] = (
	GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
)
LATCHED_QOS: Final[QoSProfile] = QoSProfile(
	depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
VOLATILE_QOS: Final[QoSProfile] = QoSProfile(
	depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
)
TF_STATIC_QOS: Final[QoSProfile] = QoSProfile(
	depth=100, reliability=ReliabilityPolicy.RELIABLE,
	durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
# 카메라 노드는 reliable KeepLast(2) 로 발행한다. best_effort 구독은 호환되고 밀리지 않는다
CAMERA_QOS: Final[QoSProfile] = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def viewer_node_name() -> str:
	"""보기 전용 노드 이름: 같은 도메인의 게이트웨이와 겹치지 않게 hostname 을 붙인다."""
	host = re.sub(r'[^a-z0-9_]', '_', socket.gethostname().lower())
	return f'{config.NODE_NAME}_viewer_{host}'


def yaw_from_quaternion(q: Quaternion) -> float:
	"""z 축 회전각 [rad]."""
	return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def planar_transform(stamped: Any) -> tuple[float, float, float, float, float, float]:
	"""TransformStamped → (tx, ty, r00, r01, r10, r11): 회전 행렬의 xy 부분과 이동."""
	t, q = stamped.transform.translation, stamped.transform.rotation
	return (
		t.x, t.y,
		1.0 - 2.0 * (q.y * q.y + q.z * q.z), 2.0 * (q.x * q.y - q.z * q.w),
		2.0 * (q.x * q.y + q.z * q.w), 1.0 - 2.0 * (q.x * q.x + q.z * q.z),
	)


def scan_points_array(msg: LaserScan, transform: tuple[float, ...]) -> np.ndarray:
	"""스캔을 base_footprint 2D 점 float (N, 2) 로 바꾼다 (자동 주행 탐색기가 쓴다).

	라이다가 거꾸로(roll ≈ π) 달려 있어도 쿼터니언 회전을 그대로 적용하므로 좌우가 맞다.
	NaN · inf · range_min..range_max 밖은 버린다.
	"""
	tx, ty, r00, r01, r10, r11 = transform
	ranges = np.asarray(msg.ranges, dtype=np.float64)
	angles = msg.angle_min + np.arange(ranges.size) * msg.angle_increment
	is_valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
	px = ranges[is_valid] * np.cos(angles[is_valid])
	py = ranges[is_valid] * np.sin(angles[is_valid])
	return np.column_stack((r00 * px + r01 * py + tx, r10 * px + r11 * py + ty))


def scan_points(msg: LaserScan, transform: tuple[float, ...]) -> list[float]:
	"""scan_points_array 를 화면에 보낼 [x0, y0, x1, y1, ...] 으로 (소수 3자리)."""
	return np.round(scan_points_array(msg, transform), 3).ravel().tolist()


def grid_snapshot(seq: int, msg: OccupancyGrid, received_at: float) -> explore.GridSnapshot | None:
	"""OccupancyGrid → explore.GridSnapshot. data 는 복사하지 않고 numpy 로 본다 (int8 (h, w)).

	Returns:
		data 길이가 width × height 와 다르면 None (깨진 지도는 없는 것처럼 본다).
	"""
	info = msg.info
	try:
		data = np.frombuffer(msg.data, np.int8).reshape(info.height, info.width)
	except ValueError:
		return None
	data.flags.writeable = False   # 화면 PNG 도 같은 메시지를 읽는다
	origin = info.origin
	return explore.GridSnapshot(
		seq, info.resolution, origin.position.x, origin.position.y,
		yaw_from_quaternion(origin.orientation), data, received_at,
	)


def path_remaining(points: list[float], x: float, y: float) -> float | None:
	"""(x, y) 에서 경로의 가장 가까운 점까지 거리 + 그 점부터 끝까지 경로 길이 [m].

	Args:
		points: 경로 [x0, y0, x1, y1, ...] (로봇 위치와 같은 프레임).
		x: 로봇 x.
		y: 로봇 y.

	Returns:
		남은 거리. 경로에 점이 없으면 None.
	"""
	path = list(zip(points[0::2], points[1::2]))
	if not path:
		return None
	nearest = min(range(len(path)), key=lambda index: math.dist(path[index], (x, y)))
	rest = path[nearest:]
	return math.dist((x, y), rest[0]) + sum(map(math.dist, rest, rest[1:]))


def _idle_navigation() -> dict:
	"""eta 는 남은 거리로 robot 이 채우고, recoveries 는 피드백을 받지 않아 늘 None 이다."""
	return {
		'state': 'idle' if HAS_NAVIGATION else 'unavailable', 'goal': None,
		'distance_remaining': None, 'eta': None, 'recoveries': None, 'message': None,
	}


def _odom_dict(raw: bytes) -> dict:
	"""raw nav_msgs/Odometry → {'x', 'y', 'yaw', 'vx', 'vy', 'wz'}."""
	msg = deserialize_message(raw, Odometry)
	pose, twist = msg.pose.pose, msg.twist.twist
	return {
		'x': round(pose.position.x, 3), 'y': round(pose.position.y, 3),
		'yaw': round(yaw_from_quaternion(pose.orientation), 4),
		'vx': round(twist.linear.x, 3), 'vy': round(twist.linear.y, 3),
		'wz': round(twist.angular.z, 3),
	}


def _odom_pose(raw: bytes) -> tuple[float, float, float]:
	"""raw nav_msgs/Odometry → (x, y, yaw) (반올림하지 않는다: 자동 주행이 스캔 점을 옮길 때 쓴다)."""
	pose = deserialize_message(raw, Odometry).pose.pose
	return pose.position.x, pose.position.y, yaw_from_quaternion(pose.orientation)


def _pose_dict(odom: dict | None) -> dict | None:
	return None if odom is None else {key: odom[key] for key in ('x', 'y', 'yaw')}


def _imu_dict(raw: bytes) -> dict:
	"""raw sensor_msgs/Imu → 자세 (도) · 각속도 · 가속도."""
	msg = deserialize_message(raw, Imu)
	q = msg.orientation
	roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
	pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
	gyro, accel = msg.angular_velocity, msg.linear_acceleration
	return {
		'roll': round(math.degrees(roll), 2),
		'pitch': round(math.degrees(pitch), 2),
		'yaw': round(math.degrees(yaw_from_quaternion(q)), 2),
		'gyro': [round(gyro.x, 4), round(gyro.y, 4), round(gyro.z, 4)],
		'accel': [round(accel.x, 3), round(accel.y, 3), round(accel.z, 3)],
	}


def _motor_status(raw: bytes) -> tuple[list[dict], dict | None]:
	"""raw lekiwi_msgs/MotorStateArray → (모터 dict 목록, 켜진 모터 전압 범위 또는 None)."""
	motors = [
		{
			'id': motor.id, 'name': motor.name, 'online': motor.online, 'mode': motor.mode,
			'torque_enabled': motor.torque_enabled, 'position': round(motor.position, 3),
			'velocity': round(motor.velocity, 3), 'load': round(motor.load, 1),
			'voltage': round(motor.voltage, 2), 'temperature': round(motor.temperature, 1),
			'current': round(motor.current, 3), 'moving': motor.moving, 'status': motor.status,
			'position_ticks': motor.position_ticks,
			'status_labels': [label for bit, label in MOTOR_STATUS_LABELS if motor.status & bit],
		}
		for motor in deserialize_message(raw, MotorStateArray).motors
	]
	voltages = [motor['voltage'] for motor in motors if motor['online']]
	power = {'voltage_min': min(voltages), 'voltage_max': max(voltages)} if voltages else None
	return motors, power


def _arm_status_dict(raw: bytes) -> dict:
	"""raw lekiwi/arm/status (std_msgs/String, JSON) → ARM_STATUS_KEYS 만 (없는 키는 None)."""
	data = json.loads(deserialize_message(raw, String).data)
	return {key: data.get(key) for key in ARM_STATUS_KEYS}


def _arm_joints(raw: bytes) -> dict[str, float]:
	"""raw lekiwi/arm/state (JointState) → {관절: LeRobot 단위 값} (소수 2자리)."""
	msg = deserialize_message(raw, JointState)
	return {name: round(position, 2) for name, position in zip(msg.name, msg.position)}


def _parameter_values(names: list[str], future: RosFuture) -> dict[str, Any] | None:
	"""GetParameters 응답 → {이름: 파이썬 값}. 실패하면 None."""
	response = future.result() if future.exception() is None else None
	if response is None or len(response.values) != len(names):
		return None
	return {name: parameter_value_to_python(value) for name, value in zip(names, response.values)}


def _resolve(target: concurrent.futures.Future, source: RosFuture) -> None:
	"""rclpy Future 의 결과를 다른 스레드가 기다리는 concurrent Future 로 옮긴다."""
	if target.done():
		return
	error = source.exception()
	if error is not None:
		target.set_exception(error)
	else:
		target.set_result(source.result())


def _endpoint_name(info: Any) -> str:
	return f'{info.node_namespace.rstrip("/")}/{info.node_name}'


class QuietNode(Node):
	"""파라미터 서비스 · 타입 설명 서비스 · 기본 QoS 이벤트 처리기가 없는 노드 (Pi CPU).

	rclpy executor 는 메시지마다 대기 집합을 다시 만들며 항목을 파이썬으로 하나씩 훑는다. 쓰지 않는
	서비스 7개와 구독 · 발행마다 둘씩 붙는 기본 이벤트 처리기도 그 항목이라 메시지마다 CPU 를 쓴다.
	대신 QoS 가 맞지 않아도 rclpy 경고 로그가 남지 않는다.

	Args:
		node_name: 노드 이름.
		**options: Node 의 나머지 인자 (context 등).
	"""

	def __init__(self, node_name: str, **options: Any) -> None:
		super().__init__(
			node_name, start_parameter_services=False,
			parameter_overrides=[Parameter('start_type_description_service', value=False)],
			**options,
		)

	def create_publisher(self, *args: Any, **kwargs: Any) -> Any:
		"""Node.create_publisher 를 기본 이벤트 처리기 없이 (Node.__init__ 의 /parameter_events 도)."""
		callbacks = PublisherEventCallbacks(use_default_callbacks=False)
		return super().create_publisher(*args, **{'event_callbacks': callbacks, **kwargs})

	def create_subscription(self, *args: Any, **kwargs: Any) -> Any:
		"""Node.create_subscription 을 기본 이벤트 처리기 없이."""
		callbacks = SubscriptionEventCallbacks(use_default_callbacks=False)
		return super().create_subscription(*args, **{'event_callbacks': callbacks, **kwargs})


class OperatorNode(QuietNode):
	"""게이트웨이의 ROS 노드.

	Args:
		is_read_only: True 면 퍼블리셔 · 서비스 · 액션 · 파라미터 클라이언트를 만들지 않고,
			노드 이름도 viewer_node_name() 으로 바꾼다 (같은 도메인의 게이트웨이와 구별).
		on_event: (level, message) 사용자에게 알릴 일 (자율 주행 결과 등). ROS 스레드에서 불린다.
	"""

	def __init__(self, is_read_only: bool, on_event: Callable[[str, str], None]) -> None:
		super().__init__(viewer_node_name() if is_read_only else config.NODE_NAME)
		self._is_read_only = is_read_only
		self._on_event = on_event
		self._lock = threading.Lock()   # 아래 수신 상태 (다른 스레드가 읽는다)
		self._times = {topic: collections.deque() for topic in TOPIC_STREAMS}
		self._last_seen: dict[str, float] = {}
		self._subscribed_streams: set[str] = set()
		self._message_seq = itertools.count(1)
		# TOPIC_STREAMS 토픽 → (seq, 마지막 메시지). odom · IMU · 모터 상태는 raw bytes, 스캔은
		# (메시지, 라이다 TF, 그때의 odom 항목, 그때의 map 자세, 그 odom 을 받은 time.monotonic())
		self._latest: dict[str, tuple[int, Any] | None] = dict.fromkeys(TOPIC_STREAMS)
		self._conversions: dict[str, tuple[int, Any]] = {}   # 토픽 → (seq, 바꾼 값)
		self._bad_conversions: set[str] = set()   # 읽지 못한 메시지가 있었던 토픽 (경고는 한 번)
		self._motor_ready: bool | None = None
		self._ready_seq = 0
		self._arm_ready: bool | None = None   # 늘 구독한다 (상태에는 'arm' 묶음을 구독하는 동안만 보인다)
		# lekiwi/zmq_host/status: (session · reason · clients · client_ip, 받은 time.monotonic()), 받은 적 없으면 None
		self._zmq_host: tuple[dict, float] | None = None
		self._has_warned_zmq_host = False
		self._estop_latched: bool | None = None   # (ROS 스레드만) 마지막으로 보낸 estop_latched
		self._arm_registers: dict | None = None   # 'calibration' 묶음을 구독하는 동안만
		self._has_arm_driver = False   # lekiwi/motor/arm_ready 발행자가 있다 (드라이버 팔 쓰기 켬)
		self._scan_cache: dict | None = None
		self._scan_array: tuple[int, np.ndarray] | None = None   # (seq, 점) 탐색 스레드만 쓴다
		self._map: OccupancyGrid | None = None
		self._map_seq = 0
		self._map_received_at = -math.inf
		self._map_cache: dict | None = None
		self._map_encoded_at = -math.inf
		self._map_encode_lock = threading.Lock()
		self._plan: dict | None = None
		self._node_names: frozenset[str] = frozenset()
		self._node_count = 0
		self._other_cmd_vel: tuple[str, ...] = ()
		self._base_count = 0
		self._navigation = _idle_navigation()
		self._goal_plan_seq = 0   # 목표를 보낼 때의 경로 seq: 그 뒤에 온 경로로만 남은 거리를 잰다
		self._active_goals = 0    # 서버 status 의 취소할 수 있는 목표 수 (누가 보냈든)
		# 'map' 묶음: 지도 위 로봇 자세 (보기 전용도 쓴다)
		self._tf_echo = tf_echo.TfEcho(tf_echo.command(f'{self.get_name()}_tf_echo'))
		# ROS 스레드만 쓰는 것
		self._commands: queue.SimpleQueue = queue.SimpleQueue()
		# (ROS 스레드만) 응답을 기다리는 서비스 호출: (포기할 시각, 클라이언트, rclpy Future, 결과)
		self._pending_calls: list[tuple[float, Client, RosFuture, concurrent.futures.Future]] = []
		self._is_closed = False
		self._stream_subscriptions: dict[str, list[Any]] = {}
		self._scan_tf: tuple[float, ...] | None = None
		self._scan_tf_frame: str | None = None
		self._scan_tf_time = -math.inf
		self._has_warned_scan_tf = False
		self._warned_formats: set[str] = set()
		self._cmd_vel_publisher: Any = None
		self._peer_hold_until = -math.inf
		self._view_node: Node | None = None
		self._parameter_client: Client | None = None   # 속도 상한을 읽는 동안만 (request_speed_limits)
		# navigate_to_pose 액션의 send_goal · get_result · cancel_goal 서비스 (Nav2 노드가 보이는 동안)
		self._goal_clients: tuple[Client, Client, Client] | None = None
		self._goal_status_subscription: Any = None
		self._speed_limit_publisher: Any = None   # 목표 클라이언트와 함께 산다
		self._goal_seq = 0
		self._goal_id: Any = None   # 우리가 보낸 마지막 목표의 UUID (종료 때는 이것만 취소한다)
		self._is_goal_accepted = False
		self._is_cancel_pending = False

		self._command_guard = self.create_guard_condition(self.command_callback)
		self._tf_buffer = tf2_ros.Buffer()
		# 라이다 TF 는 정적이라 /tf_static 만 받는다 (map 자세는 'map' 묶음의 tf2_echo)
		self.create_subscription(TFMessage, '/tf_static', self.tf_static_callback, TF_STATIC_QOS)
		self.create_subscription(
			Bool, config.MOTOR_READY_TOPIC, self.motor_ready_callback, LATCHED_QOS,
		)
		# 바뀔 때만 오는 latched 값: 늘 받는다 (로봇 끄기 전 접기 · 보정이 누가 켠 팔 토크든 안다)
		self.create_subscription(Bool, config.ARM_READY_TOPIC, self.arm_ready_callback, LATCHED_QOS)
		# PC(ZMQ) 세션 (1 Hz): 팔 시작 · 자율 주행 · 보정 · 네트워크 전환을 막는다
		self.create_subscription(
			String, config.ZMQ_HOST_STATUS_TOPIC, self.zmq_host_callback, LATCHED_QOS,
		)
		# map_server 는 transient_local, cartographer 는 volatile 로 발행한다
		self.create_subscription(OccupancyGrid, config.MAP_TOPIC, self.map_callback, LATCHED_QOS)
		self.create_subscription(OccupancyGrid, config.MAP_TOPIC, self.map_callback, VOLATILE_QOS)
		self.create_subscription(Path, config.PLAN_TOPIC, self.plan_callback, 1)
		self.create_timer(config.STATUS_PERIOD, self.pose_timer_callback)

		self._power_client: Client | None = None
		self._reset_client: Client | None = None
		self._arm_torque_client: Client | None = None
		self._arm_rest_client: Client | None = None
		self._initial_pose_publisher: Any = None
		self._arm_goal_publisher: Any = None
		self._estop_latched_publisher: Any = None
		if not is_read_only:
			self._power_client = self.create_client(SetBool, config.MOTOR_POWER_SERVICE)
			self._reset_client = self.create_client(Trigger, config.RESET_ODOMETRY_SERVICE)
			# 비상 정지가 바로 부를 수 있게 늘 둔다 (끄기는 서버가 늦게 보여도 기다린다)
			self._arm_torque_client = self.create_client(SetBool, config.ARM_TORQUE_SERVICE)
			self._arm_rest_client = self.create_client(Trigger, config.ARM_REST_SERVICE)
			self._initial_pose_publisher = self.create_publisher(
				PoseWithCovarianceStamped, config.INITIAL_POSE_TOPIC, 1,
			)
			# 팔로워는 best effort 깊이 5 로 받는다. 퍼블리셔는 대기 집합에 들지 않는다 (QuietNode)
			self._arm_goal_publisher = self.create_publisher(
				JointState, config.ARM_GOAL_TOPIC, qos_profile_sensor_data,
			)
			# lekiwi_zmq_host 가 PC 세션 시작을 거절하는 데 쓴다. 시작할 때 false (걸림은 이 프로세스 안에만 있다)
			self._estop_latched_publisher = self.create_publisher(
				Bool, config.ESTOP_LATCHED_TOPIC, LATCHED_QOS,
			)
			self.publish_estop_latched(False)

	# ── 명령 큐 (아무 스레드 → ROS 스레드) ─────────────────
	def post(self, function: Callable[..., Any], *args: Any) -> concurrent.futures.Future:
		"""function(*args) 를 ROS 스레드에서 실행한다.

		Returns:
			반환값 또는 예외를 담을 Future. 노드가 닫혔으면 RuntimeError 가 담긴다.
		"""
		future: concurrent.futures.Future = concurrent.futures.Future()
		if self._is_closed:
			future.set_exception(RuntimeError('ROS 노드가 닫혔습니다'))
			return future
		self._commands.put((future, function, args))
		try:
			self._command_guard.trigger()
		except Exception as error:  # 노드를 지우는 중이다
			if not future.done():
				future.set_exception(RuntimeError(f'ROS 노드가 닫혔습니다: {error}'))
		return future

	def command_callback(self) -> None:
		"""(ROS 스레드) 쌓인 명령을 모두 실행한다."""
		while True:
			try:
				future, function, args = self._commands.get_nowait()
			except queue.Empty:
				return
			if not future.set_running_or_notify_cancel():
				continue
			try:
				future.set_result(function(*args))
			except Exception as error:
				logger.exception('ROS 스레드 명령이 실패했습니다: %s', function)
				future.set_exception(error)

	def close(self) -> None:
		"""ROS 스레드를 멈춘 뒤 부른다: tf2_echo · 조작 노드 · 이 노드를 지우고 남은 명령은 실패로
		끝낸다."""
		self._is_closed = True
		self._tf_echo.close(is_waiting=True)
		self.set_view_node(False)
		self.destroy_node()
		while True:
			try:
				future, _function, _args = self._commands.get_nowait()
			except queue.Empty:
				return
			if not future.done():
				future.set_exception(RuntimeError('ROS 노드가 닫혔습니다'))

	# ── 그래프 · 속도 상한 (ROS 스레드) ────────────────────
	def refresh_graph(self) -> None:
		"""노드 이름 · cmd_vel 의 다른 발행자 · cmd_vel 을 구독하는 베이스 컨트롤러 수를 읽는다.

		이름이 같은 게이트웨이가 하나 더 있을 수 있어 우리 발행자는 이름이 아니라 개수로 뺀다
		(우리 것은 많아야 하나). 자율 주행 노드가 보이는 동안만 액션 서비스 클라이언트 · status
		구독을 두고, 없으면 지우고 목표를 비운다. 베이스가 없으면 속도 상한 클라이언트도 지운다.
		"""
		pairs = self.get_node_names_and_namespaces()
		names = frozenset(name for name, _namespace in pairs)
		own = (self.get_name(), self.get_namespace())
		own_count = 0
		others: set[str] = set()
		for info in self.get_publishers_info_by_topic(config.CMD_VEL_TOPIC):
			if (info.node_name, info.node_namespace) == own:
				own_count += 1
			else:
				others.add(_endpoint_name(info))
		now = time.monotonic()
		if own_count > (0 if self._cmd_vel_publisher is None else 1):
			self._peer_hold_until = now + random.uniform(*PEER_HOLD)
		if now < self._peer_hold_until:
			others.add(f'{own[1].rstrip("/")}/{own[0]}')
		# 이름을 아직 모르는 구독자도 베이스로 센다 (다른 로봇의 베이스가 발견되는 중일 수 있다)
		base_count = sum(
			info.node_name in (config.BASE_CONTROLLER_NODE, UNKNOWN_NODE_NAME)
			for info in self.get_subscriptions_info_by_topic(config.CMD_VEL_TOPIC)
		)
		has_navigation_nodes = any(
			name in names for name in config.PROCESS_KEY_NODES['navigation']
		)
		has_arm_driver = self.count_publishers(config.ARM_READY_TOPIC) > 0
		with self._lock:
			self._has_arm_driver = has_arm_driver
			# 베이스 컨트롤러는 꺼질 때 ready false 를 보내지 않는다: 남은 ready true 를 믿지 않는다
			if config.BASE_CONTROLLER_NODE in self._node_names - names:
				self._motor_ready = None
			if config.MOTOR_DRIVER_NODE in self._node_names - names:   # arm_ready 도 같다
				self._arm_ready = None
			self._node_names, self._node_count = names, len(pairs)
			self._other_cmd_vel, self._base_count = tuple(sorted(others)), base_count
			if not has_navigation_nodes:
				self._navigation = _idle_navigation()
		if config.BASE_CONTROLLER_NODE not in names:
			self._drop_parameter_client()   # 다시 나타나면 request_speed_limits 가 새로 만든다
		if not has_navigation_nodes:
			self._is_goal_accepted = False
		self._set_goal_clients(has_navigation_nodes and not self._is_read_only and HAS_NAVIGATION)

	def graph(self) -> tuple[frozenset[str], int, tuple[str, ...], int]:
		"""(아무 스레드) (노드 이름, 노드 수, cmd_vel 의 다른 발행자, cmd_vel 을 구독하는
		베이스 컨트롤러 수) 마지막 값."""
		with self._lock:
			return self._node_names, self._node_count, self._other_cmd_vel, self._base_count

	def request_speed_limits(self, callback: Callable[[dict[str, Any] | None], None]) -> bool:
		"""베이스 컨트롤러 속도 상한 읽기를 시작한다 (SPEC §7-8).

		GetParameters 클라이언트는 읽는 동안만 둔다 (대기 집합을 작게): 처음 부를 때 만들고,
		응답이 오거나 베이스가 사라지면 지운다. Jazzy AsyncParameterClient 는 서비스 클라이언트
		6개를 만들고 지우는 방법이 없어, 그것이 get_parameters 에 쓰는 하나만 쓴다.

		Args:
			callback: (ROS 스레드) {이름: 값}, 읽지 못했으면 None.

		Returns:
			요청을 보냈으면 True. 보기 전용이거나 파라미터 서비스를 아직 못 찾았으면 False.
		"""
		if self._is_read_only:
			return False
		if self._parameter_client is None:
			self._parameter_client = self.create_client(
				GetParameters, f'{config.BASE_CONTROLLER_NODE}/get_parameters',
			)
		client = self._parameter_client
		if not client.service_is_ready():
			return False
		names = list(config.DEFAULT_SPEED_LIMITS)

		def done(future: RosFuture) -> None:
			if self._parameter_client is client:
				self._drop_parameter_client()
			callback(_parameter_values(names, future))

		client.call_async(GetParameters.Request(names=names)).add_done_callback(done)
		return True

	def _drop_parameter_client(self) -> None:
		if self._parameter_client is not None:
			self.destroy_client(self._parameter_client)
			self._parameter_client = None

	# ── 조작 (ROS 스레드) ──────────────────────────────────
	def create_cmd_vel_publisher(self) -> None:
		"""cmd_vel 퍼블리셔를 만든다 (조작이 허용된 holder 가 있을 때만 부른다)."""
		if self._is_read_only:
			raise RuntimeError('보기 전용 모드에서는 cmd_vel 퍼블리셔를 만들지 않습니다')
		if self._cmd_vel_publisher is None:
			self._cmd_vel_publisher = self.create_publisher(Twist, config.CMD_VEL_TOPIC, 10)

	def publish_cmd_vel(self, command: tuple[float, float, float]) -> None:
		"""(vx, vy, wz) 를 보낸다. 퍼블리셔가 없으면 아무것도 하지 않는다."""
		if self._cmd_vel_publisher is None:
			return
		msg = Twist()
		msg.linear.x, msg.linear.y, msg.angular.z = (float(value) for value in command)
		self._cmd_vel_publisher.publish(msg)

	def drop_cmd_vel_publisher(self) -> None:
		"""0 을 한 번 보내고 cmd_vel 퍼블리셔를 지운다 (Nav2 감독 도구 검사를 위해)."""
		if self._cmd_vel_publisher is None:
			return
		self.publish_cmd_vel(ZERO_COMMAND)
		self.destroy_publisher(self._cmd_vel_publisher)
		self._cmd_vel_publisher = None

	def set_view_node(self, is_wanted: bool) -> None:
		"""조작 화면 노드(OPERATOR_VIEW_NODE_NAME)를 만들거나 지운다 (SPEC §7-11)."""
		if is_wanted and self._view_node is None:
			if self._is_read_only:
				raise RuntimeError('보기 전용 모드에서는 조작 화면 노드를 만들지 않습니다')
			self._view_node = QuietNode(config.OPERATOR_VIEW_NODE_NAME, context=self.context)
			self.executor.add_node(self._view_node)
		elif not is_wanted and self._view_node is not None:
			if self.executor is not None:
				self.executor.remove_node(self._view_node)
			self._view_node.destroy_node()
			self._view_node = None

	# ── 서비스 (아무 스레드) ───────────────────────────────
	def motor_ready(self) -> tuple[bool | None, int]:
		"""(lekiwi/motor/ready 마지막 값 (모르면 None), 받은 개수)."""
		with self._lock:
			return self._motor_ready, self._ready_seq

	def is_power_service_ready(self) -> bool:
		"""(ROS 스레드) lekiwi/motor/power 서버를 찾았는지. 보기 전용이면 늘 False."""
		return self._power_client is not None and self._power_client.service_is_ready()

	def call_motor_power(self, is_enabled: bool) -> concurrent.futures.Future:
		"""lekiwi/motor/power 를 부른다. Future 결과는 응답, 서비스가 없으면 None."""
		request = SetBool.Request()
		request.data = is_enabled
		# 켜기는 robot._power_on 이 비상 정지를 보며 서버를 기다린다. 여기서도 기다리면
		# 늦게 간 켜기가 비상 정지의 끄기보다 뒤에 도착할 수 있다
		discovery_timeout = 0.0 if is_enabled else config.MOTOR_SERVICE_DISCOVERY_TIMEOUT
		return self._call(self._power_client, request, discovery_timeout)

	def is_arm_torque_service_ready(self) -> bool:
		"""(ROS 스레드) lekiwi/motor/arm_torque 서버를 찾았는지. 보기 전용이면 늘 False."""
		return self._arm_torque_client is not None and self._arm_torque_client.service_is_ready()

	def call_arm_torque(self, is_enabled: bool) -> concurrent.futures.Future:
		"""lekiwi/motor/arm_torque 를 부른다 (call_motor_power 와 같은 규칙: 켜기는 서버를 기다리지 않는다).
		Future 결과는 응답, 서비스가 없으면 (드라이버 팔 쓰기 꺼짐) None."""
		request = SetBool.Request()
		request.data = is_enabled
		discovery_timeout = 0.0 if is_enabled else config.MOTOR_SERVICE_DISCOVERY_TIMEOUT
		return self._call(self._arm_torque_client, request, discovery_timeout)

	def call_arm_rest(self) -> concurrent.futures.Future:
		"""lekiwi/arm/rest (팔로워가 팔을 접는다). Future 결과는 응답, 서비스가 없으면 None."""
		return self._call(self._arm_rest_client, Trigger.Request())

	def arm_ready(self) -> bool | None:
		"""(아무 스레드) lekiwi/motor/arm_ready 마지막 값 (늘 구독한다). 받은 적 없거나 드라이버가 사라졌으면 None."""
		with self._lock:
			return self._arm_ready

	def publish_estop_latched(self, is_latched: bool) -> None:
		"""(ROS 스레드) lekiwi/safety/estop_latched 를 바뀔 때만 보낸다. 보기 전용이면 아무것도 하지 않는다."""
		if self._estop_latched_publisher is None or is_latched == self._estop_latched:
			return
		self._estop_latched = is_latched
		self._estop_latched_publisher.publish(Bool(data=is_latched))

	def zmq_host_status(self) -> tuple[dict, float] | None:
		"""(아무 스레드) lekiwi/zmq_host/status 마지막 값과 받은 time.monotonic(). 받은 적 없으면 None."""
		with self._lock:
			return self._zmq_host

	def has_arm_driver(self) -> bool:
		"""(아무 스레드) lekiwi/motor/arm_ready 발행자 (팔 쓰기를 켠 드라이버) 가 있는지 (그래프, 1 Hz)."""
		with self._lock:
			return self._has_arm_driver

	def publish_arm_goal(self, positions: list[float]) -> None:
		"""(ROS 스레드) lekiwi/arm/goal: config.ARM_JOINTS 순서의 LeRobot 단위 관절."""
		msg = JointState()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.name = list(config.ARM_JOINTS)
		msg.position = [float(value) for value in positions]
		self._arm_goal_publisher.publish(msg)

	def call_reset_odometry(self) -> concurrent.futures.Future:
		"""lekiwi/motor/reset_odometry 를 부른다. Future 결과는 응답, 서비스가 없으면 None."""
		return self._call(self._reset_client, Trigger.Request())

	def _call(
		self, client: Client | None, request: Any,
		discovery_timeout: float = config.MOTOR_SERVICE_DISCOVERY_TIMEOUT,
	) -> concurrent.futures.Future:
		"""서비스를 부른다. Future 결과는 응답, 서버를 discovery_timeout 안에 못 찾으면 None,
		보낸 뒤 MOTOR_SERVICE_TIMEOUT 넘게 응답이 없으면 TimeoutError (_expire_calls)."""
		response: concurrent.futures.Future = concurrent.futures.Future()
		deadline = time.monotonic() + discovery_timeout

		def send() -> None:
			if client is not None and client.service_is_ready():
				pending = client.call_async(request)
				pending.add_done_callback(lambda done: _resolve(response, done))
				expire_at = time.monotonic() + config.MOTOR_SERVICE_TIMEOUT
				self._pending_calls.append((expire_at, client, pending, response))
			elif client is not None and time.monotonic() < deadline:
				# 막 켠 게이트웨이는 서버를 아직 못 찾았을 수 있다: 막지 않고 잠시 뒤 다시 본다
				retry = self.create_timer(DISCOVERY_RETRY_PERIOD, lambda: send_again(retry))
			else:
				response.set_result(None)

		def send_again(timer: Any) -> None:
			self.destroy_timer(timer)
			try:
				send()
			except Exception as error:   # 기다리는 쪽이 영영 기다리지 않게 알린다
				if not response.done():
					response.set_exception(error)

		def forward_error(posted: concurrent.futures.Future) -> None:
			if posted.exception() is not None and not response.done():
				response.set_exception(posted.exception())

		self.post(send).add_done_callback(forward_error)
		return response

	# ── 자율 주행 (ROS 스레드) ─────────────────────────────
	def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
		"""AMCL 초기 위치 (map 프레임, RViz 와 같은 공분산)."""
		msg = PoseWithCovarianceStamped()
		stamped = self._pose_in_map(x, y, yaw)
		msg.header = stamped.header
		msg.pose.pose = stamped.pose
		covariance = [0.0] * 36
		covariance[0], covariance[7], covariance[35] = INITIAL_POSE_COVARIANCE
		msg.pose.covariance = covariance
		self._initial_pose_publisher.publish(msg)

	def send_goal(self, x: float, y: float, yaw: float) -> bool:
		"""새 목표를 보낸다 (진행 중인 목표는 서버가 새 목표로 대체한다). 서버가 없으면 False."""
		clients = self._goal_clients
		if clients is None or not all(client.service_is_ready() for client in clients):
			return False
		request = NavigateToPose.Impl.SendGoalService.Request()
		request.goal_id.uuid = list(uuid.uuid4().bytes)
		self._goal_id = request.goal_id
		request.goal.pose = self._pose_in_map(x, y, yaw)
		self._goal_seq += 1
		seq = self._goal_seq
		self._is_goal_accepted = False
		self._is_cancel_pending = False
		with self._lock:
			self._goal_plan_seq = 0 if self._plan is None else self._plan['seq']
			self._navigation = {
				**_idle_navigation(), 'state': 'active',
				'goal': {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)},
			}
		clients[0].call_async(request).add_done_callback(
			lambda done: self.goal_response_callback(seq, request.goal_id, done),
		)
		return True

	def cancel_goal(self) -> bool:
		"""모든 목표를 취소한다 (goal_id · stamp 0: RViz · 이전 게이트웨이가 보낸 것도).

		Returns:
			취소할 목표가 있었으면 True (우리 것이 진행 중이거나 서버 status 에 진행 중인 것).
		"""
		if self._goal_clients is None:
			return False
		self._call(self._goal_clients[2], CancelGoal.Request())
		with self._lock:
			is_ours = self._navigation['state'] == 'active'
			has_others = self._active_goals > 0
		if is_ours and not self._is_goal_accepted:   # 수락 응답 전: 받는 즉시 다시 취소한다
			self._is_cancel_pending = True
		return is_ours or has_others

	def cancel_own_goal(self) -> None:
		"""(게이트웨이 종료) 우리가 보낸 목표가 진행 중일 때 그것만 취소한다.

		RViz · 터미널에서 보낸 목표는 그대로 둔다: 서비스를 다시 켜거나 SSH 가 끊겨도
		다른 사람의 주행을 멈추지 않게. (goal_id 만 주고 stamp 0 = 그 목표만 취소)
		"""
		with self._lock:
			is_ours = self._navigation['state'] == 'active'
		if self._goal_clients is None or not is_ours or self._goal_id is None:
			return
		request = CancelGoal.Request()
		request.goal_info.goal_id = self._goal_id
		self._call(self._goal_clients[2], request)

	def publish_speed_limit(self, percent: int) -> None:
		"""Nav2 속도 제한: controller_server 가 MPPI 의 vx · vy · wz 상한에 percent / 100 을 곱한다
		(가는 중인 목표에도 바로). Nav2 노드가 안 보여 퍼블리셔가 없으면 아무것도 하지 않는다."""
		if self._speed_limit_publisher is None:
			return
		msg = SpeedLimit(percentage=True, speed_limit=float(percent))
		msg.header.stamp = self.get_clock().now().to_msg()
		self._speed_limit_publisher.publish(msg)

	def stop_tf_echo(self) -> None:
		"""(게이트웨이 종료) tf2_echo 를 끄고 다시 띄우지 않는다."""
		self._tf_echo.close()

	def goal_response_callback(self, seq: int, goal_id: Any, future: RosFuture) -> None:
		"""(ROS 스레드) 목표 수락 · 거부. 수락되면 결과를 묻는다."""
		if seq != self._goal_seq:   # 그사이 새 목표를 보냈다
			return
		response = future.result() if future.exception() is None else None
		if response is None or not response.accepted:
			self._update_navigation(state='rejected', message='목표가 거부되었습니다')
			self._on_event('warn', '자율 주행: 목표가 거부되었습니다')
			return
		clients = self._goal_clients
		if clients is None:   # 그사이 Nav2 노드가 사라졌다 (목표도 비웠다)
			return
		self._is_goal_accepted = True
		if self._is_cancel_pending:
			self._is_cancel_pending = False
			self.cancel_goal()
		request = NavigateToPose.Impl.GetResultService.Request(goal_id=goal_id)
		clients[1].call_async(request).add_done_callback(
			lambda done: self.goal_result_callback(seq, done),
		)

	def goal_result_callback(self, seq: int, future: RosFuture) -> None:
		"""(ROS 스레드) 목표 결과."""
		if seq != self._goal_seq:
			return
		self._is_goal_accepted = False
		response = future.result() if future.exception() is None else None
		status = GoalStatus.STATUS_ABORTED if response is None else response.status
		state, text = GOAL_RESULTS.get(status, GOAL_RESULTS[GoalStatus.STATUS_ABORTED])
		detail = '' if response is None else response.result.error_msg
		message = f'{text}: {detail}' if detail else text
		self._update_navigation(state=state, message=message)
		self._on_event('info' if state == 'succeeded' else 'warn', f'자율 주행: {message}')

	def goal_status_callback(self, msg: GoalStatusArray) -> None:
		"""(ROS 스레드) 서버의 목표 상태 (누가 보냈든): 취소할 수 있는 목표 수만 센다."""
		count = sum(status.status in GOAL_ACTIVE_STATES for status in msg.status_list)
		with self._lock:
			self._active_goals = count

	def _set_goal_clients(self, is_wanted: bool) -> None:
		"""navigate_to_pose 액션의 서비스 클라이언트 3개 · status 구독 · 속도 제한 퍼블리셔를
		만들거나 지운다."""
		if is_wanted and self._goal_clients is None:
			base = f'{config.NAVIGATE_ACTION}/_action'
			self._goal_clients = (
				self.create_client(NavigateToPose.Impl.SendGoalService, f'{base}/send_goal'),
				self.create_client(NavigateToPose.Impl.GetResultService, f'{base}/get_result'),
				self.create_client(CancelGoal, f'{base}/cancel_goal'),
			)
			self._goal_status_subscription = self.create_subscription(
				GoalStatusArray, f'{base}/status', self.goal_status_callback,
				qos_profile_action_status_default,
			)
			self._speed_limit_publisher = self.create_publisher(
				SpeedLimit, config.SPEED_LIMIT_TOPIC, 1,
			)
		elif not is_wanted and self._goal_clients is not None:
			for client in self._goal_clients:
				self.destroy_client(client)
			self.destroy_subscription(self._goal_status_subscription)
			self.destroy_publisher(self._speed_limit_publisher)
			self._goal_clients = self._goal_status_subscription = None
			self._speed_limit_publisher = None
			with self._lock:
				self._active_goals = 0

	def _update_navigation(self, **fields: Any) -> None:
		with self._lock:
			self._navigation = {**self._navigation, **fields}

	def _pose_in_map(self, x: float, y: float, yaw: float) -> PoseStamped:
		msg = PoseStamped()
		msg.header.frame_id = config.MAP_FRAME
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.pose.position.x = float(x)
		msg.pose.position.y = float(y)
		msg.pose.orientation.z = math.sin(yaw / 2.0)
		msg.pose.orientation.w = math.cos(yaw / 2.0)
		return msg

	# ── 필요할 때만 구독 (ROS 스레드) ──────────────────────
	def set_stream_subscribed(self, stream: str, is_subscribed: bool) -> None:
		"""STREAMS 묶음 하나의 구독을 만들거나 지운다 (그 채널을 보는 화면이 있을 때만 구독한다).

		Args:
			stream: 'scan' · 'odom' · 'motors' · 'imu' · 'map' (tf2_echo) · 'arm' · 'camera/<이름>'.
			is_subscribed: 만들지 지울지. 지우면 그 묶음의 마지막 값과 수신 기록도 지운다.
		"""
		subscriptions = self._stream_subscriptions.get(stream)
		if is_subscribed and subscriptions is None:
			self._stream_subscriptions[stream] = self._create_stream(stream)
			with self._lock:
				self._subscribed_streams.add(stream)
		elif not is_subscribed and subscriptions is not None:
			for subscription in self._stream_subscriptions.pop(stream):
				self.destroy_subscription(subscription)
			if stream == 'map':
				self._tf_echo.stop()   # 마지막 map 자세도 지운다
			with self._lock:
				self._subscribed_streams.discard(stream)
				self._forget_stream_locked(stream)

	def _create_stream(self, stream: str) -> list[Any]:
		qos = qos_profile_sensor_data
		if stream == 'scan':
			return [self.create_subscription(LaserScan, config.SCAN_TOPIC, self.scan_callback, qos)]
		if stream == 'map':
			self._tf_echo.start()
			return []
		if stream == 'calibration':   # 시작할 때 한 번 오는 latched 값 (보정 탭)
			if ArmRegisters is None:
				return []
			return [self.create_subscription(
				ArmRegisters, config.ARM_REGISTERS_TOPIC, self.arm_registers_callback, LATCHED_QOS)]
		if stream == 'arm':   # 상태 30 Hz · 관절 20 Hz 는 raw 로 받아 상태를 만들 때 푼다 (arm_ready 는 늘 받는다)
			return [
				self.create_subscription(
					String, config.ARM_STATUS_TOPIC, lambda raw: self._store('arm_status', raw), qos,
					raw=True,
				),
				self.create_subscription(
					JointState, config.ARM_STATE_TOPIC, lambda raw: self._store('arm_state', raw), qos,
					raw=True,
				),
			]
		if stream in RAW_STREAMS:
			topic, message_type, ros_topic = RAW_STREAMS[stream]
			if message_type is None:   # lekiwi_msgs 없음
				return []
			return [self.create_subscription(
				message_type, ros_topic, lambda raw: self._store(topic, raw), qos, raw=True,
			)]
		name = stream.removeprefix('camera/')
		return [self.create_subscription(
			CompressedImage, config.CAMERA_TOPICS[name],
			lambda msg: self.camera_callback(name, msg), CAMERA_QOS,
		)]

	def _forget_stream_locked(self, stream: str) -> None:
		"""구독을 지운 묶음의 마지막 값 · 수신 기록을 지운다 (잠금 안)."""
		if stream == 'calibration':
			self._arm_registers = None
		for topic, owner in TOPIC_STREAMS.items():
			if owner == stream:
				self._times[topic].clear()
				self._last_seen.pop(topic, None)
				self._latest[topic] = None

	# ── 구독 콜백 (ROS 스레드) ─────────────────────────────
	def scan_callback(self, msg: LaserScan) -> None:
		"""스캔과 그때의 라이다 TF · odom · map 자세를 저장한다 (점 · 자세 변환은 get_scan 에서)."""
		transform = self._lidar_transform(msg.header.frame_id)
		map_pose = self._tf_echo.pose()
		with self._lock:
			odom, odom_seen = None, None
			if self._is_fresh_locked('odom'):
				odom, odom_seen = self._latest['odom'], self._last_seen['odom']
			self._store_locked('scan', (msg, transform, odom, map_pose, odom_seen))

	def motor_ready_callback(self, msg: Bool) -> None:
		"""lekiwi/motor/ready 값과 받은 개수 (토크를 켠 뒤 새 값을 기다릴 때 쓴다)."""
		with self._lock:
			self._motor_ready = bool(msg.data)
			self._ready_seq += 1

	def arm_registers_callback(self, msg: Any) -> None:
		"""lekiwi/motor/arm_registers → {ids, valid, homing_offset, min, max} (ID 순서 그대로)."""
		registers = {
			'ids': [int(value) for value in msg.ids], 'valid': [bool(value) for value in msg.valid],
			'homing_offset': [int(value) for value in msg.homing_offset],
			'min': [int(value) for value in msg.min_position_limit],
			'max': [int(value) for value in msg.max_position_limit],
		}
		with self._lock:
			self._arm_registers = registers

	def arm_registers(self) -> dict | None:
		"""(아무 스레드) 마지막 팔 보정 레지스터. 'calibration' 묶음을 구독하지 않으면 None."""
		with self._lock:
			return self._arm_registers

	def clear_arm_registers(self) -> None:
		"""(아무 스레드) 마지막 레지스터를 잊는다: 보정 프로그램이 모터 보정을 바꿨다 (드라이버가 다시 알릴 때까지 모른다)."""
		with self._lock:
			self._arm_registers = None

	def arm_ready_callback(self, msg: Bool) -> None:
		"""lekiwi/motor/arm_ready (늘 구독)."""
		with self._lock:
			self._arm_ready = bool(msg.data)

	def zmq_host_callback(self, msg: String) -> None:
		"""lekiwi/zmq_host/status JSON → session · reason · clients · client_ip (형이 다른 값은 None).
		읽지 못한 메시지는 버리고 경고는 한 번만."""
		try:
			data = json.loads(msg.data)
			if not isinstance(data, dict):
				raise ValueError('JSON 객체가 아닙니다')
		except ValueError as error:
			if not self._has_warned_zmq_host:
				self._has_warned_zmq_host = True
				logger.warning('%s 를 읽지 못해 버립니다: %s', config.ZMQ_HOST_STATUS_TOPIC, error)
			return
		clients = data.get('clients')
		status = {
			**{key: data[key] if isinstance(data.get(key), str) else None
				for key in ('session', 'reason', 'client_ip')},
			'clients': clients if isinstance(clients, int) and not isinstance(clients, bool) else None,
		}
		with self._lock:
			self._zmq_host = (status, time.monotonic())

	def tf_static_callback(self, msg: TFMessage) -> None:
		"""/tf_static (라이다 장착 위치 등)."""
		for transform in msg.transforms:
			self._tf_buffer.set_transform_static(transform, TF_AUTHORITY)

	def map_callback(self, msg: OccupancyGrid) -> None:
		"""새 지도마다 seq+1 (PNG 인코딩은 get_map 에서 MAP_PERIOD 에 한 번 이하)."""
		with self._lock:
			self._map_seq += 1
			self._map = msg
			self._map_received_at = time.monotonic()

	def plan_callback(self, msg: Path) -> None:
		"""경로를 PLAN_MAX_POINTS 점 이하로 줄여 저장한다 (끝점은 남긴다, 약 1 Hz)."""
		poses = msg.poses
		if len(poses) > PLAN_MAX_POINTS:
			step = math.ceil(len(poses) / (PLAN_MAX_POINTS - 1))
			poses = poses[::step] + [poses[-1]]
		points: list[float] = []
		for stamped in poses:
			points += (round(stamped.pose.position.x, 3), round(stamped.pose.position.y, 3))
		with self._lock:
			seq = 1 if self._plan is None else self._plan['seq'] + 1
			self._plan = {
				'seq': seq, 'frame': msg.header.frame_id or config.MAP_FRAME, 'points': points,
			}

	def camera_callback(self, name: str, msg: CompressedImage) -> None:
		"""JPEG 만 저장한다 (다시 압축하지 않고 넘긴다). JPEG 이 아니면 경고 한 번 하고 버린다."""
		image_format = msg.format.lower()
		if 'jpeg' not in image_format and 'jpg' not in image_format:
			if name not in self._warned_formats:
				self._warned_formats.add(name)
				logger.warning('%s 카메라 형식이 JPEG 이 아니라 버립니다: %s', name, msg.format)
			return
		self._store(f'{name}_camera', msg)

	def pose_timer_callback(self) -> None:
		"""STATUS_PERIOD 마다: 응답 없는 서비스 호출을 끝내고, 'map' 을 보는 동안 tf2_echo 가
		끝났으면 다시 띄운다 (타이머를 더 만들지 않으려고 한 주기에 함께 본다)."""
		self._expire_calls()
		self._tf_echo.watch()

	def _store(self, topic: str, data: Any) -> None:
		"""마지막 메시지 · 수신 시각 · 빈도 기록만 남긴다."""
		with self._lock:
			self._store_locked(topic, data)

	def _store_locked(self, topic: str, data: Any) -> None:
		now = time.monotonic()
		self._latest[topic] = (next(self._message_seq), data)
		self._last_seen[topic] = now
		times = self._times[topic]
		times.append(now)
		while now - times[0] > RATE_WINDOW:
			times.popleft()

	def _expire_calls(self) -> None:
		"""(ROS 스레드) MOTOR_SERVICE_TIMEOUT 넘게 응답이 없는 서비스 호출을 실패로 끝낸다.

		서버가 갑자기 죽으면 DDS 가 한동안 서비스를 보여 줘서 보낸 요청의 응답이 영영 오지 않는다.
		"""
		now = time.monotonic()
		waiting = []
		for call in self._pending_calls:
			expire_at, client, pending, response = call
			if response.done():
				continue
			if now < expire_at:
				waiting.append(call)
				continue
			client.remove_pending_request(pending)
			response.set_exception(TimeoutError(
				f'{client.srv_name} 응답이 {config.MOTOR_SERVICE_TIMEOUT:g}초 동안 없습니다',
			))
		self._pending_calls = waiting

	def _lidar_transform(self, frame: str) -> tuple[float, ...] | None:
		"""base_footprint ← frame. 정적 TF 라 캐시하고 SCAN_TF_REFRESH 마다 다시 읽는다."""
		now = time.monotonic()
		is_cached = frame == self._scan_tf_frame and self._scan_tf is not None
		if is_cached and now - self._scan_tf_time < SCAN_TF_REFRESH:
			return self._scan_tf
		try:
			stamped = self._tf_buffer.lookup_transform(config.BASE_FRAME, frame, Time())
		except tf2_ros.TransformException as error:
			if not self._has_warned_scan_tf:
				self._has_warned_scan_tf = True
				logger.warning('라이다 TF(%s ← %s)를 아직 모릅니다: %s', config.BASE_FRAME, frame, error)
			return self._scan_tf if is_cached else None
		self._scan_tf = planar_transform(stamped)
		self._scan_tf_frame = frame
		self._scan_tf_time = now
		return self._scan_tf

	def _is_fresh_locked(self, topic: str) -> bool:
		"""STALE_DATA_AGE 안에 받았는지 (잠금 안)."""
		last_seen = self._last_seen.get(topic)
		return last_seen is not None and time.monotonic() - last_seen <= STALE_DATA_AGE

	# ── 읽기 (아무 스레드) ─────────────────────────────────
	def _converted(
		self, topic: str, entry: tuple[int, Any] | None, convert: Callable[[Any], Any],
	) -> Any:
		"""저장한 (seq, 메시지) 를 convert 한 값. 같은 seq 는 한 번만 바꾼다 (화면이 여럿이어도).

		Returns:
			바꾼 값. entry 가 None 이면 None.
		"""
		if entry is None:
			return None
		seq, data = entry
		with self._lock:
			cached = self._conversions.get(topic)
		if cached is None or cached[0] != seq:
			try:
				value = convert(data)
			except Exception as error:   # 깨졌거나 정의가 다른 메시지 하나가 상태 전체를 막지 않게
				with self._lock:
					is_first = topic not in self._bad_conversions
					self._bad_conversions.add(topic)
				if is_first:
					logger.warning('%s 메시지를 읽지 못해 비워 둡니다: %s', topic, error)
				value = None
			cached = (seq, value)
			with self._lock:
				self._conversions[topic] = cached
		return cached[1]

	def snapshot(self) -> dict:
		"""상태 스냅숏 재료: rates · ages · motors · power · odom · pose · imu · motor_ready ·
		navigation · is_scan_tf_missing · arm. 구독하지 않는 토픽의 rates · ages · 값은 None.

		arm: {'ready': arm_ready, 'has_driver': arm_ready 발행자가 있다 (그래프, 늘),
		'status': lekiwi/arm/status 의 ARM_STATUS_KEYS, 'status_age' [s], 'joints': lekiwi/arm/state}.
		ready · status · joints 는 'arm' 묶음을 구독하는 동안만 (status · joints 는 STALE_DATA_AGE 안에 온 것).

		pose.map · is_localized 는 'map' 을 보는 동안 tf2_echo 가 새 자세를 찍었을 때만 (FRESH_AGE).
		navigation.distance_remaining 은 목표를 보낸 뒤 온 경로와 pose.map 으로 잰다.
		"""
		now = time.monotonic()
		map_pose = self._tf_echo.pose()
		with self._lock:
			rates: dict[str, float | None] = {}
			for topic, times in self._times.items():
				while times and now - times[0] > RATE_WINDOW:
					times.popleft()
				is_subscribed = TOPIC_STREAMS[topic] in self._subscribed_streams
				rates[topic] = round(len(times) / RATE_WINDOW, 1) if is_subscribed else None
			ages = {
				topic: round(now - self._last_seen[topic], 2) if topic in self._last_seen else None
				for topic in AGE_TOPICS
			}
			odom, imu, states, arm_status, arm_state = (
				self._latest[topic] if self._is_fresh_locked(topic) else None
				for topic in ('odom', 'imu', 'motor_states', 'arm_status', 'arm_state')
			)
			arm_seen = self._last_seen.get('arm_status')
			arm = {
				'ready': self._arm_ready if 'arm' in self._subscribed_streams else None,
				'has_driver': self._has_arm_driver,
				'status_age': None if arm_seen is None else round(now - arm_seen, 2),
			}
			ready = self._motor_ready
			navigation = dict(self._navigation)
			plan, goal_plan_seq = self._plan, self._goal_plan_seq
			scan = self._latest['scan']
		motors, power = self._converted('motor_states', states, _motor_status) or ([], None)
		odom_dict = self._converted('odom', odom, _odom_dict)
		is_goal_plan = (
			plan is not None and plan['seq'] > goal_plan_seq and plan['frame'] == config.MAP_FRAME
		)
		if navigation['state'] == 'active' and map_pose is not None and is_goal_plan:
			distance = path_remaining(plan['points'], map_pose['x'], map_pose['y'])
			navigation['distance_remaining'] = None if distance is None else round(distance, 2)
		return {
			'rates': rates,
			'ages': ages,
			'motors': motors,
			'power': power,
			'odom': odom_dict,
			'pose': {'map': map_pose, 'odom': _pose_dict(odom_dict)},
			'imu': self._converted('imu', imu, _imu_dict),
			'motor_ready': ready,
			'navigation': {**navigation, 'is_localized': map_pose is not None},
			'is_scan_tf_missing': scan is not None and scan[1][1] is None,
			'arm': {
				**arm, 'status': self._converted('arm_status', arm_status, _arm_status_dict),
				'joints': self._converted('arm_state', arm_state, _arm_joints),
			},
		}

	def get_scan(self) -> dict | None:
		"""{'seq', 'stamp', 'frame', 'points', 'range_max', 'pose'}. 라이다 TF 를 모르면 None.

		pose 는 이 스캔을 받았을 때의 로봇 자세 {'map', 'odom'} (지도 위에 점을 찍을 때 쓴다).
		각각 'map' · 'odom' 채널을 보는 동안만 있다.
		"""
		with self._lock:
			entry, cached = self._latest['scan'], self._scan_cache
		if entry is None or entry[1][1] is None:
			return None
		seq, (msg, transform, odom, map_pose, _odom_seen) = entry
		if cached is not None and cached['seq'] == seq:
			return cached
		odom_pose = _pose_dict(self._converted('odom', odom, _odom_dict))
		result = {
			'seq': seq,
			'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
			'frame': config.BASE_FRAME,
			'points': scan_points(msg, transform),
			'range_max': round(msg.range_max, 3),
			'pose': {'map': map_pose, 'odom': odom_pose},
		}
		with self._lock:
			self._scan_cache = result
		return result

	def get_map(self) -> dict | None:
		"""{'seq', 'info', 'png'}. PNG 는 MAP_PERIOD 에 한 번 이하로만 다시 만든다."""
		with self._map_encode_lock:
			with self._lock:
				msg, seq, cached = self._map, self._map_seq, self._map_cache
			now = time.monotonic()
			if msg is None or (cached is not None and (
				cached['seq'] == seq or now - self._map_encoded_at < config.MAP_PERIOD
			)):
				return cached
			self._map_encoded_at = now
			info = msg.info
			try:
				png = map_codec.occupancy_grid_to_png(info.width, info.height, msg.data)
			except ValueError as error:
				logger.warning('지도를 PNG 로 바꾸지 못했습니다: %s', error)
				return cached
			result = {
				'seq': seq,
				'info': {
					'width': info.width, 'height': info.height,
					'resolution': round(info.resolution, 6),
					'origin': {
						'x': info.origin.position.x, 'y': info.origin.position.y,
						'yaw': round(yaw_from_quaternion(info.origin.orientation), 6),
					},
					'frame': msg.header.frame_id or config.MAP_FRAME,
				},
				'png': png,
			}
			with self._lock:
				self._map_cache = result
			return result

	def map_message(self) -> tuple[int, OccupancyGrid, float] | None:
		"""(아무 스레드) 마지막 지도 그대로 (seq, 메시지, 받은 time.monotonic()). PNG 를 만들지 않는다.

		받은 메시지는 바꾸지 않으므로 msg.data 를 복사하지 않고 numpy 로 봐도 된다. 없으면 None.
		"""
		with self._lock:
			return None if self._map is None else (self._map_seq, self._map, self._map_received_at)

	def explore_inputs(self) -> dict:
		"""(탐색 스레드) 자동 주행 탐색기의 센서 값 (explore.Snapshot 의 같은 이름 필드). 시각은
		time.monotonic(), 모르는 값은 None.

		scan_points 는 마지막 스캔의 base_footprint 점 (N, 2) (라이다 TF 를 모르면 스캔 값이 모두 None),
		scan_odom 은 그 스캔을 받을 때의 odom 자세, scan_odom_lag 는 스캔을 받은 시각 − 그 odom 을 받은
		시각. odom 은 지금 odom 자세. map_pose 는 tf2_echo 의 마지막 자세 (FRESH_AGE 를 보지 않는다).
		스캔 · odom · map_pose 는 'scan' · 'odom' · 'map' 묶음을 구독하는 동안만 있다. grid 는 마지막 지도.
		"""
		map_entry = self._tf_echo.pose_with_age()
		now = time.monotonic()
		with self._lock:
			scan, scan_seen = self._latest['scan'], self._last_seen.get('scan')
			odom, odom_seen = self._latest['odom'], self._last_seen.get('odom')
		grid = self.map_message()
		inputs: dict[str, Any] = dict.fromkeys((
			'scan_seq', 'scan_points', 'scan_received_at', 'scan_odom', 'scan_odom_lag',
			'odom', 'odom_received_at', 'map_pose', 'map_pose_received_at',
		))
		inputs['grid'] = None if grid is None else grid_snapshot(*grid)
		if scan is not None and scan[1][1] is not None:
			seq, (msg, transform, scan_odom, _map_pose, scan_odom_seen) = scan
			if self._scan_array is None or self._scan_array[0] != seq:
				points = scan_points_array(msg, transform)
				points.flags.writeable = False   # 새 스캔이 올 때까지 주기마다 같은 배열을 준다
				self._scan_array = (seq, points)
			inputs.update(scan_seq=seq, scan_points=self._scan_array[1], scan_received_at=scan_seen)
			if scan_odom is not None:
				inputs.update(
					scan_odom=_odom_pose(scan_odom[1]), scan_odom_lag=scan_seen - scan_odom_seen,
				)
		if odom is not None:
			inputs.update(odom=_odom_pose(odom[1]), odom_received_at=odom_seen)
		if map_entry is not None:
			age, pose = map_entry
			inputs.update(
				map_pose=(pose['x'], pose['y'], pose['yaw']), map_pose_received_at=now - age,
			)
		return inputs

	def get_plan(self) -> dict | None:
		"""{'seq', 'frame', 'points'} (최대 PLAN_MAX_POINTS 점)."""
		with self._lock:
			return self._plan

	def get_camera_frame(self, name: str) -> tuple[int, bytes] | None:
		"""(순번, JPEG). 구독하지 않았거나 아직 프레임이 없으면 None."""
		topic = f'{name}_camera'
		with self._lock:
			entry = self._latest.get(topic)
		frame = self._converted(topic, entry, lambda msg: bytes(msg.data))
		return None if frame is None else (entry[0], frame)
