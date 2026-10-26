"""웹 서버(http_server · hub)가 로봇 쪽(robot.RobotService)을 부르는 경계.

웹 쪽은 이 Protocol 만 알고, ROS 는 모른다. 시험에서는 같은 메서드를 가진 가짜를 넣는다.
모든 메서드는 여러 스레드에서 동시에 불려도 안전해야 한다.
'(막힘)' 표시가 없는 메서드는 바로 돌아온다 (ROS 작업은 ROS 스레드에 넘긴다).

명령 결과(CommandResult)는 dict 이다:
	{'ok': bool, 'code': str, 'message': str, ...추가 필드}
	code: 'ok' | 'bad_request' | 'not_operator' | 'read_only' | 'conflict'
		| 'unavailable' | 'failed'
	message: 사용자에게 그대로 보여 줄 한국어 문장.
웹 쪽은 code 를 HTTP 상태로 바꾼다: ok 200, bad_request 400, not_operator · read_only 403,
conflict 409, failed 500, unavailable 503.
"""

from typing import Any, Callable, Protocol

CommandResult = dict[str, Any]

RESULT_HTTP_STATUS: dict[str, int] = {
	'ok': 200,
	'bad_request': 400,
	'not_operator': 403,
	'read_only': 403,
	'conflict': 409,
	'failed': 500,
	'unavailable': 503,
}


def ok_result(message: str, **extra: Any) -> CommandResult:
	"""성공 결과."""
	return {'ok': True, 'code': 'ok', 'message': message, **extra}


def error_result(code: str, message: str, **extra: Any) -> CommandResult:
	"""실패 결과. code 는 RESULT_HTTP_STATUS 의 키 중 하나."""
	if code not in RESULT_HTTP_STATUS or code == 'ok':
		raise ValueError(f'unknown error code: {code}')
	return {'ok': False, 'code': code, 'message': message, **extra}


class RobotApi(Protocol):
	"""로봇 상태 읽기와 명령. 반환 dict 모양은 SPEC §6 · §7 을 따른다."""

	# ── 읽기 ──────────────────────────────────────────────
	def get_info(self) -> dict:
		"""바뀌지 않는 정보: robot_name, version, is_read_only, features, speed_levels, map_dir."""

	def get_status(self) -> dict:
		"""상태 스냅숏 (SPEC §6). 웹 쪽이 system · network 를 덧붙여 보낸다.

		exploration (자동 주행 맵 생성): {'state': 'idle'|'starting'|'exploring'|'waiting'|'saving'|
		'done'|'stopped', 'reason': 한국어 | None, 'started_at': time.time() | None, 'elapsed' [s],
		'explored_area' [m²], 'frontiers', 'target': {'x', 'y'} | None, 'path': [x0, y0, ...] | None
		(map 프레임, 최대 200점), 'saved_map': 자동 저장한 지도 파일 | None}. 숫자는 모르면 None.
		navigation.speed_percent: 자율 주행 속도 [%] (int). 자율 주행을 쓸 수 없거나 보기 전용이면 None.
		teleop_arm (팔 원격 조작): {'is_follower_present': 그래프에 lekiwi_follower_arm_node,
		'arm_ready': lekiwi/motor/arm_ready | None, 'follower': lekiwi/arm/status 의 {'state':
		'idle'|'ramping'|'following'|'error', 'reason', 'robot_id', 'diffs', 'ramp_progress',
		'warnings'} | None, 'follower_age' [s] | None, 'joints': lekiwi/arm/state {관절: LeRobot 단위} |
		None, 'is_session_active', 'blocked_reason': 팔 시작을 막는 이유 | None, 'leader':
		{'is_relaying', 'age' [s] | None}, 'estop': 마지막 비상 정지의 팔 토크 끄기 {'state': 'pending'|
		'ok'|'failed'|'unavailable', 'message', 'time'} | None, 'is_torque_maybe_on': 이 게이트웨이가
		켠 팔 토크를 끄기를 확인하지 못했다, 'rest': 로봇을 끄기 전 팔 접기 {'state': 'idle'|'resting'|
		'rested'|'failed', 'reason', 'time'}}. arm_ready · follower · joints 는 'arm'
		채널을 보는 화면이 있을 때만 (없으면 None).
		"""

	def get_scan(self) -> dict | None:
		"""{'seq', 'stamp', 'frame': 'base_footprint', 'points': [x0, y0, x1, y1, ...], 'range_max',
		'pose': {'map': {'x','y','yaw'} | None, 'odom': {...} | None}}. pose 는 이 스캔을 받았을 때의
		로봇 자세라서, 지도 위에 점을 찍을 때 status.pose 대신 쓴다 (회전 중 어긋남 방지).
		pose.map · pose.odom 은 각각 'map' · 'odom' 채널을 보는 화면이 있을 때만 있다."""

	def get_map(self) -> dict | None:
		"""{'seq', 'info': {'width', 'height', 'resolution', 'origin': {'x', 'y', 'yaw'},
		'frame'}, 'png': bytes}. png 의 첫 줄이 지도 위쪽(+y 끝)이다."""

	def get_plan(self) -> dict | None:
		"""{'seq', 'frame': 'map', 'points': [x0, y0, ...]} (최대 300점)."""

	def get_camera_frame(self, camera: str) -> tuple[int, bytes] | None:
		"""('front' | 'wrist') 의 마지막 JPEG 과 순번. 구독하지 않았으면 None."""

	def set_wanted_channels(self, channels: set[str], has_clients: bool) -> None:
		"""모든 클라이언트가 구독한 채널의 합집합과 연결된 클라이언트가 있는지.

		hub 가 둘 중 하나라도 바뀔 때마다 부른다. 로봇 쪽은 채널마다 무거운 구독을 필요할 때만
		만든다 (Pi CPU): 'scan' 스캔, 'odom' odom, 'motors' 모터 상태, 'imu' IMU, 'map' 지도 위
		로봇 자세 (tf2_echo 자식 프로세스, pose.map · is_localized), 'arm' 팔로워 상태 · 관절 ·
		arm_ready (status.teleop_arm), 'camera/<이름>' 카메라.
		구독하지 않는 토픽의 status rates · ages · 값은 None 이다. 필요 없어지면 잠시
		뒤(CAMERA_RELEASE_DELAY) 푼다. has_clients 는 지금 쓰지 않는다.
		"""

	def get_process_log(self, name: str, max_lines: int) -> list[str]:
		"""프로세스(bringup · mapping · navigation · save_map)의 최근 로그 줄."""

	def add_listener(self, callback: Callable[[dict], None]) -> None:
		"""이벤트를 받을 함수 등록. 다른 스레드에서 불릴 수 있다.
		{'type': 'event', 'level': 'info'|'warn'|'error', 'message', 'time'}
		{'type': 'log', 'process', 'lines': [...]}"""

	# ── 조작 권한 · 주행 ─────────────────────────────────
	def acquire_control(self, client_id: str, label: str) -> CommandResult:
		"""조작 권한 받기. 비어 있거나 지금 가진 쪽 신호가 끊겼을 때만 성공."""

	def release_control(self, client_id: str) -> CommandResult:
		"""권한 놓기. 정지하고, 이 권한으로 켠 토크는 끈다."""

	def is_operator(self, client_id: str) -> bool:
		"""지금 조작 권한을 가진 클라이언트인지."""

	def lease_token(self, client_id: str) -> int | None:
		"""client_id 가 지금 조작 권한을 가졌으면 그 권한 번호 (권한이 바뀔 때마다 새 번호), 아니면 None."""

	def calibration_follower(self, client_id: str, action: str, body: dict) -> CommandResult:
		"""보정 탭 팔로워 (조작 권한, docs/calibration_plan.md): action 은 select {robot_id} · from_motors
		{robot_id} · rest_pose · start {robot_id} · restore · middle · finish · cancel. 상태는
		status.calibration.follower."""

	def is_calibration_recording(self) -> bool:
		"""팔로워 보정이 범위를 재는 중인지 (hub 가 상태를 10 Hz 로 보낸다)."""

	def is_arm_session_active(self) -> bool:
		"""팔 원격 조작 세션이 열려 있는지 (리더 보정을 막는다)."""

	def end_arm_session(self, client_id: str, reason: str) -> bool:
		"""client_id 의 권한이 연 팔 세션이 열려 있으면 닫는다 (팔 토크는 그대로). 닫았으면 True."""

	def relay_leader(self, client_id: str, joints: Any) -> bool:
		"""리더 관절 {config.ARM_JOINTS 이름: LeRobot 단위} 하나를 lekiwi/arm/goal 로 (바로 돌아온다).
		client_id 가 연 팔 원격 조작 세션 중일 때만 보낸다. 넘겼으면 True."""

	def touch_operator(self, client_id: str) -> None:
		"""조작 화면이 살아 있다는 신호 (WS heartbeat · teleop 마다)."""

	def set_teleop(
		self, client_id: str, x: float, y: float, yaw: float, level: int,
		pad: str = 'mapping',
	) -> None:
		"""방향 입력. x · y · yaw 는 -1..1, level 은 SPEED_LEVELS 번호, pad 는 config.PAD_LIMITS 의 키
		('drive' 조작 화면 0.23 m/s · 1.53 rad/s, 'mapping' 지도 작성 0.11 · 옆 0.10 · 0.75). 권한이 없으면 무시."""

	def set_motor_power(self, client_id: str, is_enabled: bool) -> CommandResult:
		"""(막힘, 최대 약 10 s) 바퀴 토크 켜기 · 끄기. 켜기는 막 켠 게이트웨이가 서비스를 찾기를
		잠시 기다리고, ready 가 true 가 될 때까지 기다린다."""

	def emergency_stop(self, client_id: str, label: str = '') -> CommandResult:
		"""누구나 부를 수 있다 (연결이 끊긴 화면도): 속도 0, 자율 주행 목표 취소, 토크 끔.

		label 은 누른 화면 표시 '<IP> · <브라우저>' (웹 쪽이 POST · WS 마다 넘긴다). 비어 있으면
		조작 권한을 가진 화면의 표시, 그것도 아니면 '연결 안 된 화면'.
		비상 정지는 걸린 채로 남는다 (status.estop.is_latched): 그동안 조작 노드를 내려
		Nav2 감독 도구가 토크를 켜지 못하게 하고, 이미 오고 있던 토크 켜기 · 자율 주행 시작은
		거절한다. holder 가 다시 '조작 켜기'나 '자율 주행 시작'을 누르면 풀린다.
		바퀴 다음에 팔 토크도 끈다 (lekiwi/motor/arm_torque false, 편 팔은 떨어진다). 그 결과는
		기다리지 않고 status.teleop_arm.estop 과 이벤트로 알린다. 팔 원격 조작 세션도 닫는다.
		"""

	def client_disconnected(self, client_id: str) -> None:
		"""WS 연결이 끊김. 권한을 가졌으면 release_control 과 같게 처리."""

	# ── 팔 원격 조작 ─────────────────────────────────────
	def arm_start(self, client_id: str) -> CommandResult:
		"""(막힘, 최대 약 8 s) 팔 원격 조작 시작 (조작 권한 필요): 팔 토크를 켜고 세션을 연다.

		팔로워 노드가 있고, 그 상태(lekiwi/arm/status)가 1초 안에 왔고 ('arm' 채널을 보는 화면이
		있어야 받는다), lekiwi/motor/arm_ready 발행자(팔 쓰기를 켠 드라이버)가 있고, 비상 정지가
		걸려 있지 않고, 자율 주행 · 자동 주행 맵 생성 중이 아닐 때만 (아니면 conflict 와 이유,
		status.teleop_arm.blocked_reason 과 같다). 비상 정지 직후 1초는 거절한다.
		"""

	def arm_stop(self, client_id: str) -> CommandResult:
		"""팔 원격 조작 멈춤 (조작 권한 필요): 세션을 닫는다. 팔 토크는 켠 채 그 자리를 유지한다.
		세션이 아니어도 ok."""

	def arm_relax(self, client_id: str) -> CommandResult:
		"""(막힘, 최대 약 13 s) 팔 힘 빼기 (조작 권한 필요): 세션을 닫고 팔 토크를 끈다 (편 팔은
		떨어진다). 도는 팔 시작이 있으면 끝난 뒤에 끈다."""

	def arm_fold(self, client_id: str) -> CommandResult:
		"""(막힘, 최대 약 10 s) 팔 접고 힘 빼기 (조작 권한 필요): 세션을 닫고 lekiwi/arm/rest → rested →
		arm_torque false. 접은 자세가 없거나 못 접으면 conflict 와 이유 (팔은 그대로)."""

	def zmq_host_blocked(self) -> str | None:
		"""PC(ZMQ) 세션 (status.zmq_host) 이 active 면 그 이유 (네트워크 전환 등을 막는다), 아니면 None."""

	# ── 기능 (launch) ────────────────────────────────────
	def start_process(self, client_id: str, name: str, options: dict) -> CommandResult:
		"""bringup {'use_camera': bool} · mapping {} · navigation {'map': '<파일 이름>.yaml'}."""

	def stop_process(self, client_id: str, name: str, is_forced: bool = False) -> CommandResult:
		"""웹에서 켠 것과 터미널에서 켠 것 모두 끈다. 바로 돌아오고 진행은 status 로 본다.

		로봇(bringup)은 이 게이트웨이가 켠 팔 토크가 남아 있을 수 있으면 먼저 (막힘, 최대 약 12 s) 팔을
		접고 (lekiwi/arm/rest → status rested) 팔 토크를 끈다. 못 접으면 끄지 않고 conflict 와 이유.
		is_forced (POST force: true) 면 접지 않고 팔 힘을 뺀 뒤 끈다 (편 팔은 떨어진다).
		"""

	def save_map(self, client_id: str) -> CommandResult:
		"""지도 작성 중에 save_map.sh 를 백그라운드로 실행. 결과는 status · event."""

	def start_exploration(self, client_id: str) -> CommandResult:
		"""자동 주행 맵 생성 시작 (조작 권한 필요). 바로 돌아오고 진행은 status.exploration.

		지도 작성이 실행 중이고, 조작할 수 있고, 이 권한으로 '조작 켜기'를 했고, 비상 정지가 걸려
		있지 않고, 지도를 받았을 때만 시작한다 (아니면 conflict 와 이유). 지도를 다 만들면 (또는
		시간이 다 되면) 스스로 멈추고 지도를 저장한다. 멈추는 길(멈춤 · 0 이 아닌 조작 입력 · 주행 끄기 ·
		권한 끝 · 비상 정지 · 지도 작성 끔)은 모두 자동 주행을 끝낸다 (그때는 저장하지 않는다).
		"""

	def stop_exploration(self, client_id: str) -> CommandResult:
		"""자동 주행 멈춤 (조작 권한 필요). 토크는 켜 둔다. 자동 주행 중이 아니어도 ok."""

	def set_initial_pose(self, client_id: str, x: float, y: float, yaw: float) -> CommandResult:
		"""자율 주행의 초기 위치 (map 프레임)."""

	def send_nav_goal(self, client_id: str, x: float, y: float, yaw: float) -> CommandResult:
		"""자율 주행 목표 (map 프레임). 진행은 status.navigation."""

	def cancel_nav_goal(self, client_id: str) -> CommandResult:
		"""누구나 부를 수 있다."""

	def set_navigation_speed(self, client_id: str, percent: int) -> CommandResult:
		"""자율 주행 속도: Nav2 최고 속도의 percent % (조작 권한 필요).

		percent 는 config.NAV_SPEED_MIN..NAV_SPEED_MAX 의 int (bool 아님), 아니면 bad_request.
		값은 게이트웨이가 기억한다 (켤 때 NAV_SPEED_DEFAULT, status.navigation.speed_percent).
		자율 주행이 꺼져 있어도 받는다. Nav2 가 돌면 바로, 게이트웨이가 보내는 목표마다 그 직전에,
		100 % 보다 낮으면 실행 중 · 외부 실행인 동안 NAV_SPEED_PERIOD 마다 speed_limit (percentage)
		으로 보낸다 (100 % 는 주기마다 보내지 않는다: 다른 쪽이 낮춘 값을 덮지 않게).
		MPPI 는 가는 중인 목표에도 바로 vx · vy · wz 상한을 줄인다. 이벤트는 내지 않는다.
		"""

	def reset_odometry(self, client_id: str) -> CommandResult:
		"""odom 을 원점으로 (조작 권한 필요)."""

	def shutdown(self) -> None:
		"""(막힘) 안전하게 끝내기: 정지, 우리가 켠 토크 끔, 웹에서 켠 프로세스 종료."""
