"""HTTP 서버: 정적 페이지 · REST API · /ws 업그레이드 (SPEC §3 · §4).

모든 요청은 Host 가 이 로봇의 이름 · IP 여야 한다 (DNS rebinding 막기).
POST 는 모두 같은 검사를 거친다: Content-Type JSON, 같은 출처, 연결된 WS 의 X-Client-Id
(비상 정지만 빼고: WS 가 다시 연결되는 중인 화면도 멈출 수 있어야 한다), 본문 64 KiB 이하,
JSON 객체. robot 결과 code 는 RESULT_HTTP_STATUS 로 HTTP 상태가 된다.
"""

import ipaddress
import json
import logging
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Final
from urllib.parse import parse_qs, unquote, urlsplit

from lekiwi_web_operator import agent, config, map_files, websocket
from lekiwi_web_operator.hub import LOG_PROCESSES, Hub, client_label, to_json
from lekiwi_web_operator.network import STATUS_UNKNOWN_REASON, NetworkService, network_switch_block
from lekiwi_web_operator.robot_api import (
	RESULT_HTTP_STATUS,
	CommandResult,
	RobotApi,
	error_result,
)
from lekiwi_web_operator.teleop import finite_number

logger = logging.getLogger(__name__)

MAX_BODY_BYTES: Final[int] = 64 * 1024
IDLE_TIMEOUT: Final[float] = 30.0            # [s] 놀고 있는 keep-alive 연결을 닫는다
# 연결 수 상한 (WS 는 수명 동안 하나로 센다). 브라우저 하나는 HTTP 6 + 탭마다 WS 1 개쯤 쓴다.
# 전체 상한은 fd 기본 상한 1024 안에서 ROS · 프로세스 감시 몫을 남긴다.
MAX_CONNECTIONS_PER_IP: Final[int] = 32
MAX_CONNECTIONS: Final[int] = 256
REFUSED_LOG_PERIOD: Final[float] = 10.0      # [s] 연결 거절 경고를 이보다 자주 찍지 않는다
DEFAULT_LOG_LINES: Final[int] = 200
JSON_TYPE: Final[str] = 'application/json; charset=utf-8'
TEXT_TYPE: Final[str] = 'text/plain; charset=utf-8'
MIME_TYPES: Final[dict[str, str]] = {
	'.html': 'text/html; charset=utf-8',
	'.js': 'text/javascript; charset=utf-8',
	'.css': 'text/css; charset=utf-8',
	'.svg': 'image/svg+xml',
	'.png': 'image/png',
	'.ico': 'image/x-icon',
	'.json': JSON_TYPE,
}
DEFAULT_PORTS: Final[dict[str, int]] = {'http': 80, 'https': 443}
DISCONNECTED_MESSAGE: Final[str] = '연결이 끊겼습니다. 페이지를 새로 고치세요'
CALIBRATION_ACTIONS: Final[dict[str, frozenset[str]]] = {   # docs/calibration_plan.md
	'/api/calibration/follower/': frozenset({
		'select', 'from_motors', 'rest_pose', 'start', 'restore', 'middle', 'finish', 'cancel'}),
	'/api/calibration/leader/': frozenset({'check', 'from_motors', 'start', 'middle', 'finish', 'cancel'}),
}
AGENT_POSTS: Final[frozenset[str]] = frozenset({
	'/api/agent/approve', '/api/agent/unapprove', '/api/agent/leader_connect',
	'/api/agent/leader_disconnect',
})
NETWORK_POSTS: Final[frozenset[str]] = frozenset({   # 검색 (읽기) 과 화면을 끊을 수 있는 변경 넷
	'/api/network/scan', '/api/network/connect', '/api/network/ap_on', '/api/network/ap_off',
	'/api/network/ap_config',
})
INTERNAL_ERROR_MESSAGE: Final[str] = '서버 안에서 오류가 났습니다. 서버 로그를 확인하세요'

Reply = tuple[int, str, bytes]   # (HTTP 상태, Content-Type, 본문)


def is_allowed_host(host: str | None) -> bool:
	"""Host 헤더가 IP 주소 · localhost · 이 컴퓨터 이름 · mDNS 이름(*.local) 인지.

	인터넷 DNS 이름은 공격 사이트가 로봇 IP 로 바꿔 가리킬 수 있다 (DNS rebinding: 같은 출처로
	보여 Origin 검사를 지난다). *.local 은 인터넷 DNS 로 줄 수 없으므로, SD 이미지를 복제해
	이름이 겹쳐 avahi 가 바꾼 'lekiwi04-2.local' 같은 이름도 받는다.
	"""
	if host is None:
		return True  # Host 를 보내지 않는 도구 (브라우저는 늘 보낸다)
	try:
		name = (urlsplit(f'//{host}').hostname or '').rstrip('.')
	except ValueError:
		return False
	try:
		ipaddress.ip_address(name)
		return True
	except ValueError:
		pass
	return name in ('localhost', socket.gethostname().lower()) or name.endswith('.local')


def is_same_origin(origin: str, host: str | None) -> bool:
	"""Origin 헤더가 Host 헤더와 같은 출처 (같은 호스트 이름 · 포트) 인지."""
	if host is None:
		return False
	try:
		origin_url = urlsplit(origin)
		host_url = urlsplit(f'//{host}')
		origin_port = origin_url.port or DEFAULT_PORTS.get(origin_url.scheme)
		host_port = host_url.port or DEFAULT_PORTS['http']
	except ValueError:
		return False
	return (
		origin_url.scheme in DEFAULT_PORTS
		and origin_url.hostname is not None
		and origin_url.hostname == host_url.hostname
		and origin_port == host_port
	)


def _json_reply(status: int, payload: Any) -> Reply:
	return status, JSON_TYPE, to_json(payload).encode('utf-8')


def _error_reply(status: int, message: str) -> Reply:
	return _json_reply(status, error_result('bad_request', message))


def _static_file(url_path: str) -> tuple[str, bytes] | None:
	"""URL 경로 → config.WEB_DIR 안 파일의 (Content-Type, 내용). 밖 · 숨김 · 모르는 형식이면 None."""
	web_dir = config.WEB_DIR.resolve()
	relative = unquote(url_path).lstrip('/') or 'index.html'
	try:
		file_path = (web_dir / relative).resolve()
	except (OSError, ValueError):  # NUL 문자, 링크 고리
		return None
	if not file_path.is_relative_to(web_dir):
		return None
	if any(part.startswith('.') for part in file_path.relative_to(web_dir).parts):
		return None
	if file_path.suffix not in MIME_TYPES:
		return None
	try:
		return MIME_TYPES[file_path.suffix], file_path.read_bytes()
	except OSError:  # 폴더, 없는 파일, 권한
		return None


class OperatorRequestHandler(BaseHTTPRequestHandler):
	"""요청 하나 (keep-alive 면 같은 연결의 여러 요청). /ws 는 업그레이드 뒤 WS 수명 동안 막힌다."""

	protocol_version = 'HTTP/1.1'
	timeout = IDLE_TIMEOUT
	server: 'OperatorHttpServer'

	def do_GET(self) -> None:
		"""정적 파일, GET API, /ws 업그레이드."""
		url = urlsplit(self.path)
		if not is_allowed_host(self.headers.get('Host')):
			self._reject_host()
		elif url.path in ('/ws', '/agent'):
			self._serve_websocket(url.path == '/agent')
		elif url.path.startswith('/api/'):
			try:
				reply = self._get_api(url.path, parse_qs(url.query))
			except Exception:
				logger.exception('GET %s 처리 중 오류', url.path)
				reply = _json_reply(500, error_result('failed', INTERNAL_ERROR_MESSAGE))
			self._send(*reply)
		else:
			self._serve_static(url.path)

	def do_POST(self) -> None:
		"""POST API. 검사를 모두 통과해야 robot · network 에 넘긴다."""
		path = urlsplit(self.path).path
		client_id = self.headers.get('X-Client-Id', '')
		length = self.headers.get('Content-Length', '')
		if not is_allowed_host(self.headers.get('Host')):
			self._reject_host()
		elif self.headers.get_content_type() != 'application/json':
			self._reject(415, 'Content-Type 은 application/json 이어야 합니다')
		elif not self._is_same_origin():
			self._reject(403, '다른 사이트에서 온 요청은 받지 않습니다')
		elif path != '/api/estop' and not self.server.hub.is_connected(client_id):
			# 비상 정지는 WS 가 끊겨 다시 연결되는 중인 화면도 보낼 수 있어야 한다
			self._reject(403, DISCONNECTED_MESSAGE)
		elif not (length.isascii() and length.isdigit()):
			self._reject(411, 'Content-Length 가 필요합니다')
		elif len(length) > 6 or int(length) > MAX_BODY_BYTES:  # 6자리 넘으면 이미 너무 크다
			self._reject(413, '본문은 64 KiB 이하여야 합니다')
		else:
			self._send(*self._answer_post(path, client_id, self.rfile.read(int(length))))

	def _send(self, status: int, content_type: str, body: bytes) -> None:
		"""응답 하나. keep-alive 가 되도록 늘 Content-Length 를 붙인다."""
		self.send_response(status)
		self.send_header('Content-Type', content_type)
		self.send_header('Content-Length', str(len(body)))
		self.send_header('Cache-Control', 'no-store')
		# 다른 사이트가 이 화면을 보이지 않는 iframe 에 넣어 클릭을 훔치지 못하게 (clickjacking)
		self.send_header('X-Frame-Options', 'DENY')
		self.send_header('Content-Security-Policy', "frame-ancestors 'none'")
		if self.close_connection:
			self.send_header('Connection', 'close')
		self.end_headers()
		self.wfile.write(body)

	def _reject(self, status: int, message: str) -> None:
		"""검사에 걸린 요청. 본문을 읽지 않았으므로 다음 요청으로 읽히지 않게 연결을 닫는다."""
		self.close_connection = True
		self._send(*_error_reply(status, message))

	def _reject_host(self) -> None:
		"""허용 목록 밖의 Host: DNS rebinding 으로 온 다른 사이트일 수 있다."""
		address = f'http://{socket.gethostname()}.local:{self.server.server_address[1]}'
		self._reject(403, f'이 주소로는 열 수 없습니다. {address} 또는 IP 주소로 여세요')

	def _is_same_origin(self) -> bool:
		"""Origin 이 없거나 (브라우저 밖 도구) Host 와 같은 출처."""
		origin = self.headers.get('Origin')
		return origin is None or is_same_origin(origin, self.headers.get('Host'))

	def _serve_websocket(self, is_agent: bool) -> None:
		"""/ws (브라우저) 또는 /agent (PC 에이전트, 2초 생존 신호) 업그레이드 뒤 그 연결 수명 동안 막힌다."""
		# 업그레이드하든 거절하든 이 소켓을 다시 HTTP 로 읽지 않는다
		self.close_connection = True
		if not self._is_same_origin():
			self._send(*_error_reply(403, '다른 사이트에서 온 연결은 받지 않습니다'))
			return
		problem = websocket.handshake_error(self.headers)
		if problem is not None:
			self._send(*_error_reply(400, problem))
			return
		self.send_response(101)
		self.send_header('Upgrade', 'websocket')
		self.send_header('Connection', 'Upgrade')
		self.send_header('Sec-WebSocket-Accept',
			websocket.accept_key(self.headers['Sec-WebSocket-Key']))
		self.end_headers()
		if is_agent:
			connection = websocket.WebSocketConnection(
				self.connection, self.rfile, read_timeout=config.AGENT_SILENCE_TIMEOUT,
				max_message_bytes=agent.AGENT_MAX_MESSAGE_BYTES)
			self.server.hub.agent.serve(connection, self.client_address[0])
			return
		connection = websocket.WebSocketConnection(self.connection, self.rfile)
		self.server.hub.serve_client(
			connection, self.client_address[0], self.headers.get('User-Agent', ''))

	def _serve_static(self, url_path: str) -> None:
		found = _static_file(url_path)
		if found is None:
			self._send(404, TEXT_TYPE, '파일을 찾지 못했습니다'.encode('utf-8'))
			return
		self._send(200, *found)

	def _get_api(self, path: str, query: dict[str, list[str]]) -> Reply:
		server = self.server
		if path == '/api/info':
			return _json_reply(200, server.hub.get_info())
		if path == '/api/status':
			return _json_reply(200, server.hub.get_status())
		if path == '/api/maps':
			map_dir = config.map_dir()
			maps = map_files.list_maps(map_dir)
			return _json_reply(200, {'maps': maps, 'map_dir': str(map_dir)})
		if path == '/api/maps/preview':
			png = map_files.render_preview(config.map_dir(), query.get('name', [''])[0])
			if png is None:
				return _error_reply(404, '지도를 찾지 못했습니다')
			return 200, 'image/png', png
		if path == '/api/logs':
			process = query.get('process', [''])[0]
			count = query.get('lines', [str(DEFAULT_LOG_LINES)])[0]
			if process not in LOG_PROCESSES or not (count.isascii() and count.isdigit()):
				return _error_reply(400, 'process 이름과 lines(숫자)가 필요합니다')
			# 6자리를 넘으면 어차피 상한이다 (아주 긴 숫자의 int 변환을 피한다)
			max_lines = config.PROCESS_LOG_LINES if len(count) > 6 else int(count)
			max_lines = max(1, min(max_lines, config.PROCESS_LOG_LINES))
			lines = server.robot.get_process_log(process, max_lines)
			return _json_reply(200, {'lines': lines})
		if path == '/api/network':
			return _json_reply(200, server.network.get_status())
		return _error_reply(404, '없는 API 입니다')

	def _answer_post(self, path: str, client_id: str, raw_body: bytes) -> Reply:
		try:
			body = json.loads(raw_body)
		except (ValueError, RecursionError):
			body = None
		if not isinstance(body, dict):
			return _error_reply(400, '본문은 JSON 객체여야 합니다')
		try:
			result = self._run_post(path, client_id, body)
			if result is None:
				return _error_reply(404, '없는 API 입니다')
			return _json_reply(RESULT_HTTP_STATUS.get(result.get('code'), 500), result)
		except Exception:
			logger.exception('POST %s 처리 중 오류', path)
			return _json_reply(500, error_result('failed', INTERNAL_ERROR_MESSAGE))

	def _run_post(self, path: str, client_id: str, body: dict) -> CommandResult | None:
		"""POST 경로 → 명령. 모르는 경로면 None."""
		robot = self.server.robot
		label = client_label(self.client_address[0], self.headers.get('User-Agent', ''))
		if path == '/api/control/acquire':
			return robot.acquire_control(client_id, label)
		if path == '/api/control/release':
			return robot.release_control(client_id)
		if path == '/api/motor/power':
			is_enabled = body.get('enabled')
			if not isinstance(is_enabled, bool):
				return error_result('bad_request', 'enabled 는 true 또는 false 여야 합니다')
			return robot.set_motor_power(client_id, is_enabled)
		if path == '/api/estop':
			return robot.emergency_stop(client_id, label)
		if path == '/api/process/start':
			name, options = body.get('name'), body.get('options', {})
			if not isinstance(name, str) or not isinstance(options, dict):
				return error_result('bad_request', 'name(문자열)과 options(객체)가 필요합니다')
			return robot.start_process(client_id, name, options)
		if path == '/api/process/stop':
			name, is_forced = body.get('name'), body.get('force', False)
			if not isinstance(name, str) or not isinstance(is_forced, bool):
				return error_result('bad_request', 'name(문자열)이 필요하고 force 는 true 또는 false 여야 합니다')
			return robot.stop_process(client_id, name, is_forced)
		if path == '/api/map/save':
			return robot.save_map(client_id)
		if path == '/api/explore/start':
			return robot.start_exploration(client_id)
		if path == '/api/explore/stop':
			return robot.stop_exploration(client_id)
		if path in ('/api/nav/initial_pose', '/api/nav/goal'):
			x, y, yaw = (finite_number(body.get(key)) for key in ('x', 'y', 'yaw'))
			if x is None or y is None or yaw is None:
				return error_result('bad_request', 'x · y · yaw 는 유한한 숫자여야 합니다')
			if path == '/api/nav/goal':
				return robot.send_nav_goal(client_id, x, y, yaw)
			return robot.set_initial_pose(client_id, x, y, yaw)
		if path == '/api/nav/cancel':
			return robot.cancel_nav_goal(client_id)
		if path == '/api/nav/speed':
			percent = body.get('percent')
			if isinstance(percent, bool) or not isinstance(percent, int):   # 범위는 robot 이 본다
				return error_result('bad_request', 'percent 는 정수여야 합니다')
			return robot.set_navigation_speed(client_id, percent)
		if path == '/api/odom/reset':
			return robot.reset_odometry(client_id)
		if path in AGENT_POSTS:
			return self._run_agent_post(path, client_id, body)
		for prefix, run in (
			('/api/calibration/follower/', robot.calibration_follower),
			('/api/calibration/leader/', self.server.hub.agent.calibration_leader),
		):
			action = path.removeprefix(prefix)
			if action != path and action in CALIBRATION_ACTIONS[prefix]:
				return run(client_id, action, body)
		if path == '/api/arm/start':
			return robot.arm_start(client_id)
		if path == '/api/arm/stop':
			return robot.arm_stop(client_id)
		if path == '/api/arm/relax':
			return robot.arm_relax(client_id)
		if path in NETWORK_POSTS:
			# 보기 전용은 다른 PC 에서 돈다: 그 PC 의 와이파이를 보거나 바꾸지 않는다
			if robot.get_info().get('is_read_only'):
				return error_result('read_only', '보기 전용 모드에서는 와이파이를 다룰 수 없습니다')
			network = self.server.network
			if path == '/api/network/scan':
				return network.scan()
			if path == '/api/network/ap_on':
				return self._network_change(client_id, body, network.ap_on)
			if path == '/api/network/ap_off':
				return self._network_change(client_id, body, network.ap_off)
			if path == '/api/network/ap_config':
				return self._network_change(client_id, body, network.ap_config, 'ssid', 'password')
			return self._network_change(client_id, body, network.connect, 'ssid', 'password')
		return None

	def _run_agent_post(self, path: str, client_id: str, body: dict) -> CommandResult:
		"""PC 에이전트 허락 · 리더암 연결 (조작 권한, 값 검사는 AgentLink 가 한다)."""
		link = self.server.hub.agent
		if path == '/api/agent/approve':   # 화면이 본 연결 번호 (그사이 바뀐 연결을 허락하지 않게)
			return link.approve(client_id, body.get('agent_session'))
		if path == '/api/agent/unapprove':
			return link.unapprove(client_id)
		if path == '/api/agent/leader_connect':
			return link.leader_connect(client_id, body.get('port'), body.get('calibration_id'))
		return link.leader_disconnect(client_id)

	def _network_change(
		self, client_id: str, body: dict, run: Callable[..., CommandResult], *fields: str,
	) -> CommandResult:
		"""공유기 연결 · AP 켜기 · 끄기 · 설정: 조작 권한 → confirm → 형 → 움직임 거절 → run(*본문 fields).

		전환하면 화면이 끊겨 비상 정지를 누를 수 없으므로 로봇이 움직일 수 있는 동안은 거절한다
		(기획서 U2, 판정은 network_switch_block 한 곳). 로봇 상태를 못 읽어도 거절한다 (fail closed).
		"""
		robot = self.server.robot
		if not robot.is_operator(client_id):
			return error_result('not_operator', '네트워크를 바꾸려면 먼저 조작 권한을 받으세요')
		if body.get('confirm') is not True:
			return error_result('bad_request', '확인(confirm: true)이 필요합니다')
		values = [body.get(field) for field in fields]
		if not all(isinstance(value, str) for value in values):
			return error_result('bad_request', f'{" 와 ".join(fields)} 는 문자열이어야 합니다')
		try:
			reason = network_switch_block(robot.get_status())
		except Exception:
			logger.exception('네트워크를 바꾸기 전에 로봇 상태를 읽지 못했습니다')
			reason = STATUS_UNKNOWN_REASON
		if reason is not None:
			return error_result('conflict', reason)
		return run(*values)


class OperatorHttpServer(ThreadingHTTPServer):
	"""게이트웨이 HTTP 서버. 요청 연결 · WS 연결마다 daemon 스레드 하나.

	연결 수는 IP 당 MAX_CONNECTIONS_PER_IP, 전체 MAX_CONNECTIONS 까지만 받는다. LAN 의 한
	호스트가 fd · 스레드 · 메모리를 다 써서 다른 화면의 비상 정지와 프로세스 감시를 막지 못하게 한다.

	Args:
		address: (host, port). port 0 이면 빈 포트를 고른다.
		hub: WS 세션.
		robot: 로봇 쪽 경계.
		network: 네트워크 상태 · 와이파이.
	"""

	daemon_threads = True
	# 끌 때 keep-alive · WS 스레드를 기다리지 않는다 (안전 정지는 robot.shutdown 이 맡는다)
	block_on_close = False

	def __init__(
		self,
		address: tuple[str, int],
		hub: Hub,
		robot: RobotApi,
		network: NetworkService,
	) -> None:
		self.hub = hub
		self.robot = robot
		self.network = network
		self._count_lock = threading.Lock()
		self._ip_counts: dict[str, int] = {}     # 열려 있는 연결 수 (IP 마다)
		self._refused_log_time = -math.inf      # serve_forever 스레드만 쓴다
		super().__init__(address, OperatorRequestHandler)

	def process_request(
		self, request: socket.socket, client_address: tuple[str, int],
	) -> None:
		"""(serve_forever 스레드) 상한 안이면 연결 스레드를 띄우고, 넘으면 답 없이 닫는다."""
		ip = client_address[0]
		with self._count_lock:
			is_full = (
				self._ip_counts.get(ip, 0) >= MAX_CONNECTIONS_PER_IP
				or sum(self._ip_counts.values()) >= MAX_CONNECTIONS)
			if not is_full:
				self._ip_counts[ip] = self._ip_counts.get(ip, 0) + 1
		if is_full:
			now = time.monotonic()
			if now - self._refused_log_time >= REFUSED_LOG_PERIOD:
				self._refused_log_time = now
				logger.warning('연결이 너무 많아 %s 의 새 연결을 닫습니다 (IP 당 %d, 전체 %d 까지)',
					ip, MAX_CONNECTIONS_PER_IP, MAX_CONNECTIONS)
			self.shutdown_request(request)
			return
		try:
			super().process_request(request, client_address)
		except BaseException:
			self._forget_connection(ip)  # 스레드를 못 띄웠다 (소켓은 socketserver 가 닫는다)
			raise

	def process_request_thread(
		self, request: socket.socket, client_address: tuple[str, int],
	) -> None:
		"""(연결 스레드) 요청 · WS 를 처리하고, 소켓을 닫은 뒤 연결 수를 줄인다."""
		try:
			super().process_request_thread(request, client_address)
		finally:
			self._forget_connection(client_address[0])

	def _forget_connection(self, ip: str) -> None:
		with self._count_lock:
			self._ip_counts[ip] -= 1
			if self._ip_counts[ip] == 0:
				del self._ip_counts[ip]
