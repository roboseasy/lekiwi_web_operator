"""브라우저 WS 세션: 채널 구독 · 클라이언트별 송신 스레드 · robot 이벤트 분배 (SPEC §4 · §5).

송신 규칙: status 는 STATUS_PERIOD 마다, scan · map · plan · 카메라는 구독했고 seq 가
바뀌었을 때만 최소 간격을 지켜 최신 것만 보낸다. 이벤트 · 로그만 길이 제한 deque 에 쌓는다.
네트워크가 느리면 송신 스레드가 막혀 자연히 덜 보낸다.
모든 클라이언트 채널의 합집합과 연결 여부가 바뀌면 robot.set_wanted_channels 로 알린다
(로봇 쪽은 무거운 구독을 필요할 때만 만든다: Pi CPU).
"""

import base64
import json
import logging
import math
import secrets
import socket
import threading
import time
from collections import deque
from typing import Any, Callable, Final

from lekiwi_web_operator import config
from lekiwi_web_operator.agent import AgentLink
from lekiwi_web_operator.network import NetworkService
from lekiwi_web_operator.processes import make_event
from lekiwi_web_operator.robot_api import RobotApi
from lekiwi_web_operator.system_info import SystemInfo
from lekiwi_web_operator.teleop import finite_number
from lekiwi_web_operator.websocket import (
	CLOSE_GOING_AWAY,
	CLOSE_INTERNAL_ERROR,
	WebSocketConnection,
)

logger = logging.getLogger(__name__)

LOG_PROCESSES: Final[tuple[str, ...]] = (*config.PROCESS_NAMES, 'save_map')
CAMERA_PREFIXES: Final[dict[str, bytes]] = {'front': b'\x01', 'wrist': b'\x02'}
# 'odom' · 'motors' · 'imu' · 'arm' 은 메시지가 따로 없고 status 의 그 값을 채운다 (로봇 쪽이 그때만 구독)
CHANNELS: Final[frozenset[str]] = frozenset({
	'scan', 'map', 'plan', 'imu', 'odom', 'motors', 'arm', 'calibration',
	*(f'camera/{camera}' for camera in CAMERA_PREFIXES),
	*(f'logs/{name}' for name in LOG_PROCESSES),
})
LOG_BACKLOG_LINES: Final[int] = 200
CALIBRATION_STATUS_PERIOD: Final[float] = 0.1   # [s] 보정 범위를 재는 동안의 status 주기
# [s] 송신 스레드가 깨어나 새 값을 보는 간격 (카메라 15 fps 를 놓치지 않게)
POLL_PERIOD: Final[float] = config.CAMERA_MIN_PERIOD / 2
_NOT_SENT: Final[object] = object()   # 아직 보낸 적 없는 채널의 seq
BROWSER_TOKENS: Final[tuple[tuple[str, str], ...]] = (
	('Edg/', 'Edge'),
	('Whale/', 'Whale'),
	('SamsungBrowser/', 'Samsung Internet'),
	('Firefox/', 'Firefox'),
	('Chrome/', 'Chrome'),
	('Safari/', 'Safari'),
)


def to_json(value: Any) -> str:
	"""JSON 문자열. NaN · Infinity 는 null 로 바꾼다 (브라우저 JSON.parse 가 못 읽는다)."""
	return json.dumps(
		_finite_only(value), allow_nan=False, ensure_ascii=False, separators=(',', ':'))


def _finite_only(value: Any) -> Any:
	if isinstance(value, float):
		return value if math.isfinite(value) else None
	if isinstance(value, dict):
		return {key: _finite_only(item) for key, item in value.items()}
	if isinstance(value, (list, tuple)):
		return [_finite_only(item) for item in value]
	return value


def browser_name(user_agent: str) -> str:
	"""User-Agent → 짧은 브라우저 이름 (조작 권한 표시 '<IP> · <브라우저>' 용)."""
	for token, name in BROWSER_TOKENS:
		if token in user_agent:
			return name
	return '브라우저'


def client_label(ip: str, user_agent: str) -> str:
	"""화면 표시 '<IP> · <브라우저>' (조작 권한 · 비상 정지). WS 와 POST 가 같은 글이 되게 여기서만 만든다."""
	return f'{ip} · {browser_name(user_agent)}'


def _parse_object(message: str | bytes) -> dict | None:
	if isinstance(message, bytes):
		return None
	try:
		data = json.loads(message)
	except (ValueError, RecursionError):
		return None
	return data if isinstance(data, dict) else None


def _parse_teleop(data: dict) -> tuple[float, float, float, int, str] | None:
	"""teleop 메시지 → (x, y, yaw, level, pad). x · y · yaw 는 -1..1 로 자른다. pad 가 없으면
	config.DEFAULT_PAD (낮은 상한), 모르는 패드면 버린다."""
	axes = [finite_number(data.get(key)) for key in ('x', 'y', 'yaw')]
	level = data.get('level')
	pad = data.get('pad', config.DEFAULT_PAD)
	if None in axes or isinstance(level, bool) or not isinstance(level, int):
		return None
	if not isinstance(pad, str) or pad not in config.PAD_LIMITS:
		return None
	x, y, yaw = (min(1.0, max(-1.0, value)) for value in axes)
	return x, y, yaw, level, pad


class _ClientSession:
	"""브라우저 하나의 구독 · 송신 대기열. channels 이하 값은 condition 으로 보호한다."""

	def __init__(self, client_id: str, connection: WebSocketConnection, label: str) -> None:
		self.client_id = client_id
		self.connection = connection
		self.label = label   # client_label (비상 정지 표시)
		self.condition = threading.Condition()
		self.channels: frozenset[str] = frozenset()
		self.events: deque[str] = deque(maxlen=config.EVENT_QUEUE_LIMIT)
		self.logs: deque[str] = deque(maxlen=config.EVENT_QUEUE_LIMIT)
		self.is_closed = False


class Hub:
	"""연결된 브라우저들. 클라이언트마다 송신 스레드 하나를 둔다.

	잠금 순서: robot 메서드는 _lock 을 잡은 채로 부르지 않는다 (robot 이 리스너를
	자기 잠금 안에서 불러도 교착되지 않게). _wanted_lock 은 set_wanted_channels 순서만 지킨다.

	Args:
		robot: 로봇 쪽 경계.
		system: status.system.
		network: status.network · info.network_features · 접속 주소.
	"""

	def __init__(
		self, robot: RobotApi, system: SystemInfo, network: NetworkService,
	) -> None:
		self._robot = robot
		self._system = system
		self._network = network
		self._lock = threading.Lock()
		self._sessions: dict[str, _ClientSession] = {}
		self._wanted_lock = threading.Lock()
		# robot 에 마지막으로 알린 (채널 합집합, 연결된 클라이언트가 있는지)
		self._wanted: tuple[frozenset[str], bool] = (frozenset(), False)
		self._latest_sources: tuple[tuple[str, float, Callable[[], dict | None]], ...] = (
			('scan', config.SCAN_PERIOD, robot.get_scan),
			('map', config.MAP_PERIOD, robot.get_map),
			('plan', config.PLAN_PERIOD, robot.get_plan),
		)
		robot.add_listener(self._on_robot_event)
		# PC 에이전트 (/agent): 화면들과 같은 이벤트 길로 알린다
		self.agent = AgentLink(robot, self.broadcast_event)

	def broadcast_event(self, level: str, message: str) -> None:
		"""게이트웨이 쪽 (PC 에이전트) 알림을 모든 화면에 (서버 로그에도)."""
		logger.log(logging.WARNING if level in ('warn', 'error') else logging.INFO, message)
		self._on_robot_event(make_event(level, message))

	def is_connected(self, client_id: str) -> bool:
		"""client_id 가 지금 연결된 WS 클라이언트인지 (POST 검사)."""
		with self._lock:
			return client_id in self._sessions

	def get_info(self) -> dict:
		"""GET /api/info · welcome 의 info.

		Returns:
			robot.get_info() + host · urls · network_features (SPEC §6).
		"""
		return {
			**self._robot.get_info(),
			'host': socket.gethostname(),
			'urls': self._network.access_urls(),
			'network_features': self._network.features(),
		}

	def get_status(self, viewer_id: str | None = None) -> dict:
		"""status 메시지: robot.get_status() + type · system · network (SPEC §6).

		client_id 는 POST 의 자격 (X-Client-Id) 이라 holder 본인 (viewer_id) 에게만
		보여 주고, 다른 화면에는 None 으로 보낸다.
		"""
		robot_status = self._robot.get_status()
		status = {
			'type': 'status',
			**robot_status,
			# 보정 탭: 팔로워는 로봇 쪽, 리더는 PC 에이전트 (calib_status) 에서
			'calibration': {**robot_status.get('calibration', {}), 'leader': self.agent.calibration_status()},
			'agent': self.agent.status(),
			'system': self._system.get(),
			'network': self._network.get_status(),
		}
		control = status.get('control') or {}
		holder = control.get('holder')
		if holder is not None and holder.get('client_id') != viewer_id:
			status['control'] = {**control, 'holder': {**holder, 'client_id': None}}
		return status

	def serve_client(
		self, connection: WebSocketConnection, ip: str, user_agent: str,
	) -> None:
		"""WS 하나의 수명 동안 막힌다: welcome → 읽기 루프. 끝나면 robot 에 끊김을 알린다.

		Args:
			connection: 업그레이드가 끝난 연결.
			ip: 브라우저 IP (welcome 의 you.ip).
			user_agent: 로그에 남길 브라우저.
		"""
		client_id = secrets.token_hex(8)
		session = _ClientSession(client_id, connection, client_label(ip, user_agent))
		with self._lock:
			# welcome 을 받은 브라우저가 바로 POST 해도 통하도록 보내기 전에 등록한다
			self._sessions[client_id] = session
		logger.info('브라우저 연결: %s (%s)', client_id, session.label)
		try:
			self._update_wanted_channels()  # 연결 여부도 알린다 (구독은 채널을 받은 뒤)
			connection.send_text(to_json({
				'type': 'welcome',
				'client_id': client_id,
				'info': self.get_info(),
				'you': {'ip': ip},
			}))
			threading.Thread(
				target=self._write_loop, args=(session,), name=f'ws-{client_id}',
				daemon=True,
			).start()
			while (message := connection.receive()) is not None:
				self._handle_message(session, message)
		except OSError:
			pass  # welcome 을 보내다 끊겼다
		except Exception:
			logger.exception('클라이언트 %s 처리 중 오류로 연결을 닫습니다', client_id)
		finally:
			with self._lock:
				del self._sessions[client_id]
			try:
				self._robot.client_disconnected(client_id)  # 권한을 가졌으면 정지 · 토크 끔
			finally:
				with session.condition:
					session.is_closed = True
					session.condition.notify()
				connection.close(CLOSE_GOING_AWAY)
				self._update_wanted_channels()
				logger.info('브라우저 연결 끊김: %s', client_id)

	def close_all(self) -> None:
		"""서버를 끌 때 모든 WS 를 닫는다. 정리는 각 연결 스레드가 한다.

		막힌 연결은 닫는 데 CLOSE_TIMEOUT 쯤 걸리므로 동시에 닫는다 (하나씩 닫으면 막힌 연결 수만큼
		robot.shutdown 이 늦어져 systemd 의 TimeoutStopSec 를 넘길 수 있다).
		"""
		self.agent.close()
		with self._lock:
			sessions = list(self._sessions.values())
		closers = [
			threading.Thread(target=session.connection.close, args=(CLOSE_GOING_AWAY,))
			for session in sessions
		]
		for closer in closers:
			closer.start()
		for closer in closers:
			closer.join()

	def _handle_message(self, session: _ClientSession, message: str | bytes) -> None:
		"""클라이언트 메시지 하나. 모르는 것 · 잘못된 값은 경고만 남기고 연결은 유지한다."""
		client_id = session.client_id
		data = _parse_object(message)
		kind = None if data is None else data.get('type')
		if kind == 'heartbeat':
			self._robot.touch_operator(client_id)
		elif kind == 'teleop':
			command = _parse_teleop(data)
			if command is None:
				logger.warning('잘못된 teleop 을 무시합니다 (%s): %.120r', client_id, message)
				return
			self._robot.touch_operator(client_id)
			self._robot.set_teleop(client_id, *command)
		elif kind == 'subscribe':
			self._subscribe(session, data.get('channels'))
		elif kind == 'estop':   # 열린 WS 로도: 전체 연결 수가 꽉 차 새 POST 가 막혀도 멈출 수 있게
			self._robot.emergency_stop(client_id, session.label)   # 보기 전용은 robot 이 거절한다
		else:
			logger.warning('모르는 메시지를 무시합니다 (%s): %.120r', client_id, message)

	def _subscribe(self, session: _ClientSession, requested: Any) -> None:
		"""구독 집합을 통째로 바꾼다. 새로 구독한 로그는 backlog 를 먼저 보낸다."""
		if not isinstance(requested, list):
			logger.warning('subscribe 의 channels 는 목록이어야 합니다 (%s)', session.client_id)
			return
		known: set[str] = set()
		unknown = []
		for item in requested:
			if isinstance(item, str) and item in CHANNELS:
				known.add(item)
			else:
				unknown.append(item)
		if unknown:
			logger.warning('모르는 채널을 무시합니다 (%s): %.120r', session.client_id, unknown)
		channels = frozenset(known)
		with session.condition:
			added = channels - session.channels
			session.channels = channels
		# shortcut: backlog 과 실시간 묶음이 겹쳐 몇 줄이 두 번 보일 수 있다, 줄 번호가 생기면 고친다
		for channel in sorted(item for item in added if item.startswith('logs/')):
			process = channel.removeprefix('logs/')
			text = to_json({
				'type': 'log',
				'process': process,
				'lines': self._robot.get_process_log(process, LOG_BACKLOG_LINES),
				'is_backlog': True,
			})
			with session.condition:
				session.logs.append(text)
				session.condition.notify()
		self._update_wanted_channels()

	def _update_wanted_channels(self) -> None:
		"""채널 합집합이나 연결 여부가 마지막으로 알린 것과 다르면 robot 에 알린다."""
		with self._wanted_lock:
			with self._lock:
				sessions = list(self._sessions.values())
			channels = frozenset().union(*(session.channels for session in sessions))
			wanted = (channels, bool(sessions))
			if wanted != self._wanted:
				self._robot.set_wanted_channels(set(channels), bool(sessions))
				self._wanted = wanted  # 실패하면 다음 변화 때 다시 알린다

	def _on_robot_event(self, event: dict) -> None:
		"""(robot 스레드) event 는 모든 클라이언트, log 는 그 채널 구독자의 대기열에 넣는다."""
		try:
			kind = event.get('type')
			if kind == 'event':
				channel = None
				text = to_json(event)
			elif kind == 'log':
				channel = f'logs/{event.get("process")}'
				text = to_json({**event, 'is_backlog': False})
			else:
				logger.warning('모르는 robot 이벤트를 버립니다: %.120r', event)
				return
			with self._lock:
				sessions = list(self._sessions.values())
			for session in sessions:
				with session.condition:
					if channel is None:
						session.events.append(text)
					elif channel in session.channels:
						session.logs.append(text)
					else:
						continue
					session.condition.notify()
		except Exception:
			logger.exception('robot 이벤트를 전하지 못했습니다')

	def _write_loop(self, session: _ClientSession) -> None:
		"""(클라이언트마다 스레드 하나) 쌓인 이벤트 · 로그, status, 구독한 채널의 최신 값."""
		sent_seqs: dict[str, Any] = {}
		sent_times: dict[str, float] = {}
		channels: frozenset[str] = frozenset()
		next_status_time = 0.0
		try:
			while True:
				with session.condition:
					if not (session.is_closed or session.events or session.logs):
						session.condition.wait(POLL_PERIOD)
					if session.is_closed:
						return
					queued = [*session.events, *session.logs]
					session.events.clear()
					session.logs.clear()
					added = session.channels - channels
					channels = session.channels
				for channel in added:  # 다시 구독하면 seq 가 같아도 한 번 보낸다
					sent_seqs.pop(channel, None)
					sent_times.pop(channel, None)
				for text in queued:
					session.connection.send_text(text)
				now = time.monotonic()
				if now >= next_status_time:
					# 보정 범위를 재는 동안은 관절 tick 을 10 Hz 로 (화면이 끝 값을 따라가게)
					period = CALIBRATION_STATUS_PERIOD if self._is_calibration_recording() else config.STATUS_PERIOD
					next_status_time = now + period
					status = self.get_status(session.client_id)
					session.connection.send_text(to_json(status))
				self._send_latest(session, channels, sent_seqs, sent_times, now)
		except OSError:
			session.connection.close()  # 보내다 끊겼다: 읽기 쪽도 끝나게 한다
		except Exception:
			logger.exception('클라이언트 %s 송신 중 오류로 연결을 닫습니다', session.client_id)
			session.connection.close(CLOSE_INTERNAL_ERROR)

	def _is_calibration_recording(self) -> bool:
		return self.agent.is_calibration_recording() or self._robot.is_calibration_recording()

	def _send_latest(
		self,
		session: _ClientSession,
		channels: frozenset[str],
		sent_seqs: dict[str, Any],
		sent_times: dict[str, float],
		now: float,
	) -> None:
		"""구독한 scan · map · plan · 카메라 중 seq 가 바뀌고 최소 간격이 지난 것만 보낸다."""
		for channel, period, read in self._latest_sources:
			last_time = sent_times.get(channel, -math.inf)
			if channel not in channels or now - last_time < period:
				continue
			data = read()
			if data is None or sent_seqs.get(channel, _NOT_SENT) == data['seq']:
				continue
			sent_seqs[channel] = data['seq']
			sent_times[channel] = now
			if channel == 'map':
				data = {**data, 'png': base64.b64encode(data['png']).decode('ascii')}
			session.connection.send_text(to_json({'type': channel, **data}))
		for camera, prefix in CAMERA_PREFIXES.items():
			channel = f'camera/{camera}'
			last_time = sent_times.get(channel, -math.inf)
			if channel not in channels or now - last_time < config.CAMERA_MIN_PERIOD:
				continue
			frame = self._robot.get_camera_frame(camera)
			if frame is None or sent_seqs.get(channel, _NOT_SENT) == frame[0]:
				continue
			sent_seqs[channel] = frame[0]
			sent_times[channel] = now
			session.connection.send_binary(prefix + frame[1])
