"""웹 오퍼레이터가 쓰는 이름 · 시간 · 명령의 기준.

토픽 · 서비스 · 노드 이름은 lekiwi_ws/src/docs/interfaces.md 와 같아야 한다.
속도 단계 0~19 는 lekiwi_teleop terminal_teleop.TerminalSpeedProfile.LEVELS 와 같다 (20~22 는 조작 화면 패드만).
dev/dev_server.py 가 build_process_command · process_matches 를 바꿔 끼우므로
다른 모듈은 `from lekiwi_web_operator import config` 로 가져와 `config.이름` 으로 쓴다.
"""

import math
import os
import shlex
from pathlib import Path
from typing import Final

# ── 웹 서버 ──────────────────────────────────────────────
DEFAULT_HOST: Final[str] = '0.0.0.0'
DEFAULT_PORT: Final[int] = 8080
WEB_DIR: Final[Path] = Path(__file__).resolve().parent.parent / 'web'

STATUS_PERIOD: Final[float] = 0.25           # [s] 상태 전송 주기 (4 Hz)
SCAN_PERIOD: Final[float] = 0.2              # [s] 스캔 전송 최소 간격 (5 Hz)
PLAN_PERIOD: Final[float] = 0.5              # [s] 경로 전송 최소 간격
MAP_PERIOD: Final[float] = 1.0               # [s] 지도 전송 최소 간격
CAMERA_MIN_PERIOD: Final[float] = 1.0 / 15   # [s] 카메라 한 대당 최대 15 fps
CAMERA_RELEASE_DELAY: Final[float] = 5.0     # [s] 보는 화면이 없으면 이만큼 뒤 구독 해제 (채널마다, tf2_echo 포함)
WS_READ_TIMEOUT: Final[float] = 10.0         # [s] 아무 프레임도 없으면 연결을 닫는다
WS_MAX_MESSAGE_BYTES: Final[int] = 1 << 20
AGENT_SILENCE_TIMEOUT: Final[float] = 2.0    # [s] PC 에이전트 (/agent) 가 이만큼 아무것도 안 보내면 닫는다
EVENT_QUEUE_LIMIT: Final[int] = 200          # 클라이언트 하나에 쌓아 둘 이벤트 수
SYSTEM_INFO_PERIOD: Final[float] = 2.0       # [s] CPU · 온도 갱신
NETWORK_INFO_PERIOD: Final[float] = 5.0      # [s] IP · 와이파이 갱신

# ── ROS 노드 ─────────────────────────────────────────────
NODE_NAME: Final[str] = 'lekiwi_web_operator'
# 조작 권한을 가진 브라우저가 살아 있고 비상 정지가 걸려 있지 않은 동안만 존재한다.
# Nav2 감독 도구(NAV2_MOTOR_GUARD_NODE)는 이 노드가 있을 때만 바퀴 토크를 켠다.
OPERATOR_VIEW_NODE_NAME: Final[str] = 'lekiwi_web_operator_view'

# ── 토픽 · 서비스 · 액션 (상대 이름) ─────────────────────
CMD_VEL_TOPIC: Final[str] = 'cmd_vel'
ODOM_TOPIC: Final[str] = 'odom'
SCAN_TOPIC: Final[str] = 'lekiwi/sensor/scan_filtered'
# 실물 기본 설정(use_low_pass_filter true, use_kalman_filter false)에서 나오는 필터 + 자세 토픽 (100 Hz)
IMU_TOPIC: Final[str] = 'lekiwi/sensor/lpf_imu'
MOTOR_STATES_TOPIC: Final[str] = 'lekiwi/motor/states'
MOTOR_READY_TOPIC: Final[str] = 'lekiwi/motor/ready'
MOTOR_POWER_SERVICE: Final[str] = 'lekiwi/motor/power'
RESET_ODOMETRY_SERVICE: Final[str] = 'lekiwi/motor/reset_odometry'
CAMERA_TOPICS: Final[dict[str, str]] = {
	'front': 'lekiwi/sensor/front_camera/image_raw/compressed',
	'wrist': 'lekiwi/sensor/wrist_camera/image_raw/compressed',
}
# 팔 (lekiwi_lerobot · 드라이버 팔 쓰기, lekiwi_ws/src/docs/interfaces.md). arm_torque · arm_ready 는
# 드라이버가 arm_write_enabled 일 때만 있다
ARM_TORQUE_SERVICE: Final[str] = 'lekiwi/motor/arm_torque'
ARM_READY_TOPIC: Final[str] = 'lekiwi/motor/arm_ready'
ARM_STATE_TOPIC: Final[str] = 'lekiwi/arm/state'
ARM_GOAL_TOPIC: Final[str] = 'lekiwi/arm/goal'
ARM_STATUS_TOPIC: Final[str] = 'lekiwi/arm/status'
# 팔로워가 팔을 접는다 (std_srvs/Trigger, 약 3 s 램프, status state resting → rested). 로봇을 끄기 전에 부른다
ARM_REST_SERVICE: Final[str] = 'lekiwi/arm/rest'
ARM_REGISTERS_TOPIC: Final[str] = 'lekiwi/motor/arm_registers'   # 드라이버가 읽은 팔 보정 레지스터 (transient_local)
MAP_TOPIC: Final[str] = 'map'
PLAN_TOPIC: Final[str] = 'plan'
INITIAL_POSE_TOPIC: Final[str] = 'initialpose'
NAVIGATE_ACTION: Final[str] = 'navigate_to_pose'
# Nav2 controller_server 의 speed_limit_topic 기본값 (구독 QoS(10): reliable · volatile)
SPEED_LIMIT_TOPIC: Final[str] = 'speed_limit'

MAP_FRAME: Final[str] = 'map'
ODOM_FRAME: Final[str] = 'odom'
BASE_FRAME: Final[str] = 'base_footprint'

# ── 노드 이름 (namespace 제외) ───────────────────────────
BASE_CONTROLLER_NODE: Final[str] = 'lekiwi_base_controller_node'
MOTOR_DRIVER_NODE: Final[str] = 'lekiwi_motor_driver_node'
FOLLOWER_ARM_NODE: Final[str] = 'lekiwi_follower_arm_node'   # lekiwi_lerobot (bringup use_arm:=true)
# navigation launch 의 감독 도구. 한 번 토크를 끄면 끝나고 다시 켜지 않는다 (Nav2 재시작 필요)
NAV2_MOTOR_GUARD_NODE: Final[str] = 'lekiwi_nav2_motor_guard'
# 대시보드 상태 표에 보여 줄 bringup 노드 (카메라는 use_camera:=true 일 때만)
BRINGUP_NODES: Final[tuple[str, ...]] = (
	'robot_state_publisher',
	'joint_state_publisher',
	'lekiwi_imu_node',
	'lekiwi_lidar_node',
	'lekiwi_scan_filter_node',
	'lekiwi_motor_driver_node',
	'lekiwi_base_controller_node',
	'front_camera_node',
	'wrist_camera_node',
)
# 이 노드 중 하나라도 그래프에 있으면 그 기능이 켜져 있다고 본다
PROCESS_KEY_NODES: Final[dict[str, tuple[str, ...]]] = {
	'bringup': (
		'lekiwi_base_controller_node',
		'lekiwi_motor_driver_node',
		'lekiwi_lidar_node',
	),
	'mapping': ('cartographer_node',),
	'navigation': ('bt_navigator', 'amcl', NAV2_MOTOR_GUARD_NODE),
}

# ── 조작 안전 ────────────────────────────────────────────
OPERATOR_TIMEOUT: Final[float] = 2.0         # [s] 조작 화면 신호가 끊기면 권한 회수 · 정지 · 토크 끔
TELEOP_COMMAND_TIMEOUT: Final[float] = 0.3   # [s] 조작 명령이 끊기면 목표 속도 0
TELEOP_PERIOD: Final[float] = 0.05           # [s] cmd_vel 발행 주기 (20 Hz)
ZERO_HOLD: Final[float] = 0.5                # [s] 멈춘 뒤 0을 더 보내는 시간
MOTOR_READY_TIMEOUT: Final[float] = 5.0      # [s] 토크를 켠 뒤 ready를 기다리는 시간
MOTOR_SERVICE_TIMEOUT: Final[float] = 3.0    # [s] lekiwi/motor/power 응답 대기
# [s] 막 (다시) 켠 게이트웨이가 lekiwi/motor/power 를 찾기까지 기다리는 시간 (lekiwi_teleop 과 같다)
MOTOR_SERVICE_DISCOVERY_TIMEOUT: Final[float] = 2.0
# [s] 비상 정지 뒤 이 안에 온 토크 켜기 · 자율 주행 시작은 거절한다 (이미 오고 있던 요청이 이기지 않게)
# shortcut: 이보다 오래 걸려 도착한 요청은 새 요청으로 본다, 그런 지연이 보이면 화면이 본 비상 정지 번호를 보낸다
ESTOP_HOLDOFF: Final[float] = 1.0
LINEAR_STEP: Final[float] = 0.02             # [m/s] 50 ms마다 목표에 다가가는 양
ANGULAR_STEP: Final[float] = 0.12            # [rad/s] 50 ms마다
SPEED_LEVELS: Final[tuple[tuple[float, float], ...]] = (   # (선속도 m/s, 회전 rad/s)
	(0.01, 0.06), (0.02, 0.12), (0.03, 0.17), (0.04, 0.23), (0.05, 0.29),
	(0.06, 0.35), (0.07, 0.41), (0.08, 0.46), (0.09, 0.52), (0.10, 0.58),
	(0.11, 0.64), (0.12, 0.70), (0.13, 0.75), (0.14, 0.81), (0.15, 0.87),
	(0.16, 0.93), (0.17, 0.99), (0.18, 1.04), (0.19, 1.10), (0.20, 1.16),
	# 조작 화면 패드만 (2026-10-22 사용자 결정 0.23 m/s · 88°/s): 회전을 조금 더 빨리 올려 0.23 에서 1.53 rad/s
	(0.21, 1.28), (0.22, 1.41), (0.23, 1.53),
)
START_LEVEL: Final[int] = 5                  # 0.06 m/s. 지도 작성 권장 속도 (README 7장). 두 패드 모두
# 패드마다 상한 (베이스 상한과 따로, 둘 중 낮은 쪽): (앞뒤 [m/s], 옆 [m/s], 회전 [rad/s]).
# 2026-10-22 베이스 컨트롤러 상한을 0.23 m/s · 1.53 rad/s 로 올렸다. 조작 화면 패드 ('drive') 는 그만큼 (단계 22),
# 지도 작성 패드 ('mapping') 는 전 베이스 상한(0.11 · 옆 0.10 · 0.75)보다 빠르게 몰지 않는다 (단계 10).
# teleop 메시지에 pad 가 없으면 'mapping' 이다 (낮은 쪽)
DRIVE_MAX_LINEAR: Final[float] = 0.11        # [m/s] 'mapping' 패드
DRIVE_MAX_LINEAR_Y: Final[float] = 0.10      # [m/s] 옆 방향 (전 max_linear_y)
DRIVE_MAX_ANGULAR: Final[float] = 0.75       # [rad/s]
PAD_LIMITS: Final[dict[str, tuple[float, float, float]]] = {
	'drive': (0.23, 0.23, 1.53),
	'mapping': (DRIVE_MAX_LINEAR, DRIVE_MAX_LINEAR_Y, DRIVE_MAX_ANGULAR),
}
DEFAULT_PAD: Final[str] = 'mapping'
# 베이스 컨트롤러 파라미터를 못 읽었을 때 쓰는 상한 (bringup 기본값보다 낮거나 같게)
DEFAULT_SPEED_LIMITS: Final[dict[str, float]] = {
	'max_linear_x': 0.10,
	'max_linear_y': 0.10,
	'max_linear_speed': 0.10,
	'max_angular_z': 0.58,
}

# ── 팔 원격 조작 ─────────────────────────────────────────
# lerobot LeKiwi 관절 이름 (lekiwi/arm/goal · state 의 name, LeRobot 단위: 몸 관절 [deg], 그리퍼 0~100)
ARM_JOINTS: Final[tuple[str, ...]] = (
	'arm_shoulder_pan', 'arm_shoulder_lift', 'arm_elbow_flex', 'arm_wrist_flex',
	'arm_wrist_roll', 'arm_gripper',
)
ARM_STATUS_MAX_AGE: Final[float] = 1.0       # [s] 팔로워 상태(30 Hz)가 이보다 오래되면 팔 시작을 거절한다
# [s] 리더 값이 이만큼 끊기면 lekiwi/arm/goal 을 그친다 (팔로워 goal_timeout_s 와 같다. 팔은 그 자리 유지)
ARM_LEADER_TIMEOUT: Final[float] = 0.5
ARM_REST_TIMEOUT: Final[float] = 6.0         # [s] 팔을 접으라 한 뒤 status 가 rested 가 되기를 기다리는 시간
# [s] 게이트웨이를 끌 때 웹에서 켠 로봇을 끄기 전에 팔을 접고 끄는 데 쓰는 시간 (못 접으면 팔은 그대로 둔다)
ARM_SHUTDOWN_REST_TIME: Final[float] = 8.0

# ── 자율 주행 속도 (SPEED_LIMIT_TOPIC, percentage) ───────
# MPPI 가 vx · vy · wz 상한(lekiwi_ws nav2_params.yaml)에 percent / 100 을 곱한다 (가는 중인 목표에도 바로)
NAV_SPEED_DEFAULT: Final[int] = 100          # [%] 게이트웨이를 켤 때 (Nav2 최고 속도 그대로)
NAV_SPEED_MIN: Final[int] = 10               # [%] 받는 값 (정수)
NAV_SPEED_MAX: Final[int] = 100
# [s] 자율 주행이 도는 동안 100 % 보다 낮은 값을 다시 보내는 주기: volatile 이라 다시 켠
# controller_server 는 지난 값을 못 받는다 (100 % 로 시작한다)
NAV_SPEED_PERIOD: Final[float] = 1.0
# [m/s] 자율 주행 앞으로 최고 속도: lekiwi_navigation2 nav2_params.yaml 의 velocity_smoother
# max_velocity x (MPPI vx_max 0.1 보다 낮다). 예상 시간 어림에 속도 비율을 곱해 쓴다
NAV_MAX_LINEAR_SPEED: Final[float] = 0.08

# ── 프로세스 (launch) ────────────────────────────────────
PROCESS_NAMES: Final[tuple[str, ...]] = ('bringup', 'mapping', 'navigation')
PROCESS_START_GRACE: Final[float] = 60.0     # [s] 이 안에 핵심 노드가 보이면 running
PROCESS_STOP_TIMEOUT: Final[dict[str, float]] = {   # [s] SIGINT 뒤 기다리는 시간
	'bringup': 15.0,
	'mapping': 15.0,
	'navigation': 30.0,
	'save_map': 5.0,
}
PROCESS_TERM_TIMEOUT: Final[float] = 5.0     # [s] SIGTERM 뒤 SIGKILL 까지
PROCESS_LOG_LINES: Final[int] = 500          # 프로세스마다 남기는 로그 줄 수
SAVE_MAP_TIMEOUT: Final[float] = 120.0       # [s] save_map.sh 전체 제한

# ── 자동 주행 맵 생성 (explore.py) ───────────────────────
# 몸 모양 (base_footprint, +x 정면). lekiwi_description URDF 의 visual 메시를 lekiwi04 의 접은(웅크린)
# 자세로 순운동학 계산 (2026-10-20, LeRobot 보정 0° = 기록 범위 가운데: pan −26.3 · lift −69.5 ·
# elbow +105.7 · wrist_flex +49.2 · roll +104.4°; URDF 의 shoulder_lift 부호를 LeRobot 과 반대로 본
# 경우만 눈으로 본 '약 10 cm' 와 맞았다): 몸체 정면 x 0.109 m, 앞바퀴 0.126 m, 몸체 반지름 0.180 m.
# 접은 팔 끝 x 0.212 m (몸체 정면에서 10.3 cm), 팔이 나온 부분 y −0.003..+0.093 m · 높이 z 0.156..0.301 m,
# 중심에서 가장 먼 점 0.217 m. 라이다 스캔 면은 z 0.025 m 라 팔 높이의 물체는 보지 못한다
EXPLORE_BODY_RADIUS: Final[float] = 0.19       # [m] 몸체 · 바퀴 원 (0.180 + 1 cm)
EXPLORE_ARM_REACH: Final[float] = 0.22         # [m] 중심 → 접은 팔 끝 (x, 0.212 + 8 mm)
EXPLORE_ARM_HALF_WIDTH: Final[float] = 0.10    # [m] 접은 팔 좌우 반폭 (y −0.003..+0.093 을 대칭으로 덮는다)
EXPLORE_MARGIN: Final[float] = 0.08            # [m] 충돌 예측 여유
# [m] 제자리 회전 때 팔 상자 모서리가 쓸고 지나가는 반지름 + 여유 (약 0.322)
EXPLORE_ROTATION_RADIUS: Final[float] = (
	math.hypot(EXPLORE_ARM_REACH, EXPLORE_ARM_HALF_WIDTH) + EXPLORE_MARGIN
)
# 정면으로만 간다 (옆 · 뒤는 라이다가 못 보는 바퀴 쪽). 방향이 35° 넘게 틀어지면 제자리에서 돈다.
# 베이스 상한 (0.11 m/s · 0.75 rad/s) 안, Cartographer 가 따라오게 지도 작성 권장 단계보다 조금 느리게
EXPLORE_SPEED: Final[float] = 0.05             # [m/s]
EXPLORE_TURN_SPEED: Final[float] = 0.3         # [rad/s]
# [s] 충돌 예측 시간. 스캔 나이 + 명령 유효 시간 + 주기 + 베이스 반응(약 0.2 s)보다 길어야 한다 (시험)
EXPLORE_HORIZON: Final[float] = 1.0
EXPLORE_CONTROL_PERIOD: Final[float] = 0.1     # [s] 탐색 스레드 주기 (ROS 스레드 밖)
EXPLORE_SCAN_MEMORY: Final[float] = 3.0        # [s] 지나간 스캔 점을 odom 으로 옮겨 기억 (사각에 든 장애물)
# [s] 데이터가 이보다 오래되면 0 을 보내고 기다린다 ('waiting')
EXPLORE_SCAN_MAX_AGE: Final[float] = 0.3
EXPLORE_POSE_MAX_AGE: Final[float] = 0.5       # 지도 위 자세 (tf2_echo 10 Hz)
EXPLORE_MAP_MAX_AGE: Final[float] = 5.0        # 지도 (Cartographer 1 Hz)
EXPLORE_ODOM_MAX_AGE: Final[float] = 0.2       # 지금 odom (50 Hz)
EXPLORE_ODOM_MAX_LAG: Final[float] = 0.1       # 스캔과 함께 저장한 odom 이 그 스캔보다 이만큼 넘게 먼저면 버린다
EXPLORE_SENSOR_TIMEOUT: Final[float] = 5.0     # [s] 이만큼 넘게 기다리면 끝낸다 (시작할 때도)
# Cartographer /map 값의 뜻. 기본 hit 0.55 · miss 0.49 (lekiwi_2d*.lua 가 바꾸지 않는다)로 본 모형:
# 처음 안 닿음 49, 처음 닿음 52, 벽은 약 75 까지 (실물 값 분포는 아직 재지 않았다).
# map_saver 기준(빈 곳 25 이하)으로 나누면 새로 본 바닥이 수십 초 동안 모름이라 바로 '끝'이 되어 버린다
EXPLORE_FREE_MAX: Final[int] = 49              # 0..49 빈 곳 (안 닿은 증거가 더 많다)
# 65..100 막힘 (높이 2.5 cm 스캔의 한 번 닿음은 거른다). 50..64 는 애매: 지나갈 수 있다고 보고
# 프런티어도 아니다 (가까운 것은 실시간 스캔 점이 막는다). -1 은 모름
EXPLORE_OCCUPIED_MIN: Final[int] = 65
EXPLORE_CELL: Final[float] = 0.1               # [m] 계획 칸 (0.05 m 지도 2 × 2)
# [m] 계획 격자에서 막힘 · 모름 · 스캔 점을 부풀리는 반지름: 회전 반지름 약 0.32 + 칸 나눔 · 따라가기 오차.
# 약 0.95 m 보다 좁은 문 · 틈은 지나지 않는다 (직접 몰고 지나간 뒤 다시 시작)
EXPLORE_PLAN_RADIUS: Final[float] = 0.45
EXPLORE_VIEW_DISTANCE: Final[float] = EXPLORE_PLAN_RADIUS + 0.25   # [m] 프런티어를 보러 가는 칸까지
EXPLORE_LIVE_RANGE: Final[float] = 3.0         # [m] 계획 격자 · 기억에 넣는 스캔 점 거리
EXPLORE_GUARD_RANGE: Final[float] = 1.0        # [m] 충돌 예측에 넣는 점 · 지도 막힘 칸 거리
EXPLORE_MIN_FRONTIER: Final[float] = 0.4       # [m] 이보다 짧은 프런티어 덩어리는 버린다 (4 칸)
EXPLORE_BLACKLIST_RADIUS: Final[float] = 0.5   # [m] 실패한 목표 둘레는 다시 고르지 않는다
EXPLORE_PLAN_PERIOD: Final[float] = 1.0        # [s] 목표가 없을 때 다시 계획 · 가는 중 경로 다시 보기
# [s] 목표에 닿아 프런티어를 보고 기다리는 시간 (Cartographer 정지 중 2 s 마다 삽입 + 지도 1 Hz)
EXPLORE_DWELL: Final[float] = 3.0
EXPLORE_BLOCKED_TIMEOUT: Final[float] = 3.0    # [s] 충돌 예측에 이만큼 넘게 막히면 목표 실패
EXPLORE_PROGRESS_TIMEOUT: Final[float] = 20.0  # [s] 이 동안 EXPLORE_PROGRESS_DISTANCE 도 못 가면 목표 실패
EXPLORE_PROGRESS_DISTANCE: Final[float] = 0.1  # [m]
EXPLORE_MAX_FAILURES: Final[int] = 3           # 목표가 연달아 이만큼 실패하면 멈춘다 (stuck)
EXPLORE_MIN_FREE_AREA: Final[float] = 2.0      # [m²] 이보다 좁게 닿으면 '다 만들었다'고 하지 않는다
EXPLORE_MAX_DURATION: Final[float] = 900.0     # [s] 이 시간이 지나면 끝내고 저장한다
EXPLORE_LOOK_ANGLE: Final[float] = math.radians(45.0)   # 시작할 때 둘러보기 (바퀴 사각 최대 38.5° 보다 크게)
# 걸림: 명령한 만큼 지도 위 자세가 움직이지 않으면 (높이 2.5 cm 라이다가 못 보는 가구 아래 틈 ·
# 턱에 몸체 · 팔이 걸림). 로봇에서 맞출 값
EXPLORE_BUMP_WINDOW: Final[float] = 3.0        # [s]
EXPLORE_BUMP_DISTANCE: Final[float] = 0.10     # [m] 이만큼 전진을 명령했는데
EXPLORE_BUMP_MOVED: Final[float] = 0.03        # [m] 이만큼도 움직이지 않았거나
EXPLORE_BUMP_TURN: Final[float] = 0.6          # [rad] 이만큼 회전을 명령했는데
EXPLORE_BUMP_YAWED: Final[float] = 0.15        # [rad] 이만큼도 돌지 않았으면 멈춘다


# ── 보정 탭 (docs/calibration_plan.md) ───────────────────
CALIBRATION_ID_PATTERN: Final[str] = r'[A-Za-z0-9_-]{1,40}'   # robot_id · calibration_id (파일 이름)
# 팔로워 보정 프로그램 (lerobot 이 든 python 으로 따로 돈다, 표준 입출력 JSON 줄)
FOLLOWER_WORKER: Final[Path] = Path(__file__).resolve().parent / 'follower_calibration_worker.py'
FOLLOWER_WORKER_START_TIMEOUT: Final[float] = 20.0   # [s] lerobot import (Pi 약 5.5 s) + 버스 열기
FOLLOWER_WORKER_STEP_TIMEOUT: Final[float] = 10.0    # [s] 제로-포지션 위치 시키기 · 끝내기 · 취소 응답
BRINGUP_STOP_WAIT: Final[float] = 30.0               # [s] 보정 · 고르기 전에 로봇 기능이 꺼지기를 기다리는 시간
DEFAULT_MOTOR_PORT: Final[str] = '/dev/ttyACM0'      # 장비 프로파일에 port 가 없을 때 (lekiwi_motor_driver.yaml)


def follower_calibration_dir(profile_value: object = None) -> Path:
	"""팔로워 (LeKiwi) lerobot 보정 폴더: LEKIWI_CALIBRATION_DIR (개발 · 시험) → 팔로워 프로파일의 calibration_dir
	(profile_value, 팔로워 노드가 읽는 값) → 기본."""
	value = os.environ.get('LEKIWI_CALIBRATION_DIR') or (profile_value if isinstance(profile_value, str) else '')
	if value:
		return Path(value).expanduser()
	return Path.home() / '.cache' / 'huggingface' / 'lerobot' / 'calibration' / 'robots' / 'lekiwi'


def robot_profile_path() -> Path:
	"""사람이 쓴 장비 프로파일 (읽기만 한다). LEKIWI_ROBOT_PROFILE 이 이긴다."""
	return Path(os.environ.get('LEKIWI_ROBOT_PROFILE') or Path.home() / 'lekiwi_ws' / 'robot_profile.yaml')


def arm_profile_path() -> Path:
	"""게이트웨이가 통째로 쓰는 팔로워 파일 (bringup 이 robot_profile 뒤에 읽는다). LEKIWI_ARM_PROFILE 이 이긴다."""
	return Path(os.environ.get('LEKIWI_ARM_PROFILE') or Path.home() / 'lekiwi_ws' / 'arm_profile.yaml')


def follower_restore_path() -> Path:
	"""새로 보정이 쓰기 전 모터의 예전 보정 (보정 프로그램이 남기고 되돌리거나 끝내면 지운다). arm_profile 옆."""
	return arm_profile_path().with_name('arm_calibration_restore.json')


def follower_worker_command() -> list[str]:
	"""팔로워 보정 프로그램 명령줄. LEKIWI_CALIB_WORKER (개발: 가짜 프로그램) 가 이긴다."""
	override = os.environ.get('LEKIWI_CALIB_WORKER')
	if override:
		return shlex.split(override)
	python = os.environ.get('LEKIWI_LEROBOT_PYTHON') or str(Path.home() / 'lerobot060_venv' / 'bin' / 'python')
	return [python, str(FOLLOWER_WORKER)]


def map_dir() -> Path:
	"""지도 폴더. save_map.sh 와 같은 규칙 (LEKIWI_MAP_DIR, 기본 ~/maps/lekiwi)."""
	value = os.environ.get('LEKIWI_MAP_DIR')
	if value is None:
		return Path.home() / 'maps' / 'lekiwi'
	return Path(value).expanduser().resolve()


def build_process_command(name: str, options: dict) -> list[str]:
	"""웹에서 켜는 프로세스의 명령줄.

	Args:
		name: 'bringup' · 'mapping' · 'navigation' · 'save_map'.
		options: bringup은 {'use_camera': bool}, navigation은 {'map_path': str} (절대 경로).

	Returns:
		subprocess 에 넘길 argv.

	Raises:
		ValueError: 모르는 이름이거나 필요한 옵션이 없을 때.
	"""
	if name == 'bringup':
		use_camera = 'true' if options.get('use_camera', True) else 'false'
		return [
			'ros2', 'launch', 'lekiwi_bringup', 'bringup.launch.py',
			f'use_camera:={use_camera}',
		]
	if name == 'mapping':
		return ['ros2', 'launch', 'lekiwi_cartographer', 'cartographer.launch.py']
	if name == 'navigation':
		map_path = options.get('map_path')
		if not map_path:
			raise ValueError('navigation needs map_path')
		return [
			'ros2', 'launch', 'lekiwi_navigation2', 'navigation.launch.py',
			f'map:={map_path}',
			f'operator_nodes:={OPERATOR_VIEW_NODE_NAME}',
		]
	if name == 'save_map':
		return ['ros2', 'run', 'lekiwi_cartographer', 'save_map.sh']
	raise ValueError(f'unknown process: {name}')


# 웹 밖(터미널)에서 켠 launch 도 찾아 끌 수 있게 명령줄 모양을 둔다
_LAUNCH_SIGNATURES: Final[dict[str, tuple[str, str, str]]] = {
	'bringup': ('launch', 'lekiwi_bringup', 'bringup.launch.py'),
	'mapping': ('launch', 'lekiwi_cartographer', 'cartographer.launch.py'),
	'navigation': ('launch', 'lekiwi_navigation2', 'navigation.launch.py'),
}


def process_matches(name: str, cmdline: list[str]) -> bool:
	"""cmdline 이 `ros2 launch <패키지> <파일>` 형태의 그 프로세스인지.

	ros2 는 파이썬 스크립트라 cmdline 이 ['/usr/bin/python3', '/opt/ros/jazzy/bin/ros2',
	'launch', ...] 처럼 보인다. 'ros2' 로 끝나는 원소 바로 뒤가 시그니처면 맞다.
	"""
	signature = _LAUNCH_SIGNATURES.get(name)
	if signature is None:
		return False
	for index, part in enumerate(cmdline):
		if Path(part).name == 'ros2':
			return tuple(cmdline[index + 1:index + 4]) == signature
	return False
