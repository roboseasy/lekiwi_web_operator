"""게이트웨이 규약 (docs/agent_protocol.md v2): 연결 · 생존 신호 · 포트 · 리더 · 보정 · 다시 붙기.

리더암 하드웨어는 leader.LeaderLink (스트리밍) 와 calib_worker.CalibrationWorker (보정) 가 맡는다.
여기서는 그 상태 · 값을 메시지로 보내고, 게이트웨이 명령을 둘에게 넘긴다. 보내기는 메인 스레드 (생존 신호 · 포트 · 상태) 와
리더 스레드 (30 Hz 값) 가 같이 한다 (WebSocketClient 쓰기는 스레드에 안전하다).
"""

import importlib.metadata
import json
import logging
import socket
import threading
import time
from collections.abc import Callable
from pathlib import Path
from typing import Final

from lekiwi_pc_agent.calib_worker import COMMANDS as CALIB_COMMANDS
from lekiwi_pc_agent.calib_worker import CalibrationWorker
from lekiwi_pc_agent.calibration import CalibrationError, list_calibration_ids
from lekiwi_pc_agent.leader import (
	STATE_CONNECTED, STATE_CONNECTING, STATE_DISCONNECTED, Leader, LeaderError, LeaderLink,
	list_ports)
from lekiwi_pc_agent.ws_client import WebSocketClient

logger = logging.getLogger(__name__)

PROTOCOL_VERSION: Final[int] = 2   # v2: calib_* (보정)
AGENT_PATH: Final[str] = '/agent'
HEARTBEAT_PERIOD: Final[float] = 0.5     # [s] 게이트웨이는 2초 조용하면 끊는다
PORTS_PERIOD: Final[float] = 2.0         # [s] 포트 목록 확인
PING_PERIOD: Final[float] = 1.0          # [s] WebSocket ping (게이트웨이가 pong 으로 답한다)
SILENCE_TIMEOUT: Final[float] = 3.0      # [s] pong 도 없이 이만큼 조용하면 끊긴 것으로 본다
CONNECT_TIMEOUT: Final[float] = 2.0      # [s] TCP 연결 · 업그레이드 응답
RECONNECT_MIN_DELAY: Final[float] = 0.5  # [s] 다시 붙기 간격: 두 배씩 늘려 최대까지
RECONNECT_MAX_DELAY: Final[float] = 5.0
RATE_LOG_PERIOD: Final[float] = 5.0      # [s] 터미널에 리더 주기를 쓰는 간격
LEADER_CLOSE_TIMEOUT: Final[float] = 5.0  # [s] 끝낼 때 리더 스레드가 포트를 닫기를 기다린다


_CALIB_LOG_LINES: Final[dict[str, str]] = {
	'starting': '보정 시작: %s (리더암 버스를 엽니다)',
	'waiting_middle': '보정 %s: 팔을 모든 관절 범위의 가운데로 옮긴 뒤 화면에서 "제로-포지션 위치 시키기"를 누르세요',
	'recording': '보정 %s: 모든 관절을 끝까지 천천히 움직인 뒤 "끝내기"를 누르세요 (wrist_roll 제외)',
	'done': '보정 %s: 저장했습니다',
	'cancelled': '보정 %s: 취소했습니다 (모터에 쓴 값은 되돌렸습니다)',
}


def _lerobot_version() -> str | None:
	try:
		return importlib.metadata.version('lerobot')
	except importlib.metadata.PackageNotFoundError:
		return None


class Agent:
	"""PC 에이전트 하나. run() 이 stop_event 까지 게이트웨이에 붙어 있는다.

	Args:
		robot_host: 게이트웨이 (로봇) 주소.
		port: 게이트웨이 포트.
		open_leader: (port, calibration_id) → 열린 Leader. 시험은 가짜를 넣는다.
		open_bus: port → 보정용으로 연 리더 버스 (calibration.open_bus). 시험은 가짜를 넣는다.
		calibration_dir: 리더 보정 파일 폴더.
		port_lister: 포트 목록 함수 (시험용으로 바꿔 끼운다).
		hostname: agent_hello 의 host (None 이면 이 PC 이름).
	"""

	def __init__(
			self,
			robot_host: str,
			port: int,
			open_leader: Callable[[str, str], Leader],
			open_bus: Callable[[str], object],
			calibration_dir: Path,
			port_lister: Callable[[], list[dict[str, str | None]]] = list_ports,
			hostname: str | None = None) -> None:
		self._robot_host = robot_host
		self._port = port
		self._open = open_leader
		self._open_bus = open_bus
		self._calibration_dir = calibration_dir
		self._port_lister = port_lister
		self._hostname = hostname or socket.gethostname()
		self._url = f'ws://{robot_host}:{port}{AGENT_PATH}'
		self._token: str | None = None
		self._is_approved = False
		self._ports: list[dict[str, str | None]] = []
		self._calibration_ids: list[str] = []
		# agent_welcome 을 받은 연결만. 리더 스레드는 이것이 있을 때만 값을 보낸다
		self._ws: WebSocketClient | None = None
		self._seq = 0
		self._sent_count = 0
		self._link = LeaderLink(self._open_leader, self._on_leader_status, self._on_leader_joints)
		self._last_calib_state: str | None = None
		self._calibration = CalibrationWorker(
			calibration_dir, self._open_calibration_bus, self._release_leader,
			self._on_calib_status, self._check_calibration_ids)

	def run(self, stop_event: threading.Event) -> None:
		"""stop_event 까지 붙고, 끊기면 간격을 늘려 가며 다시 붙는다. 끝에 리더를 닫는다."""
		logger.info('PC 에이전트 시작: %s 에 붙습니다 (끝내려면 Ctrl+C)', self._url)
		delay = RECONNECT_MIN_DELAY
		last_error = ''
		try:
			while not stop_event.is_set():
				try:
					ws = WebSocketClient.connect(
						self._robot_host, self._port, AGENT_PATH, CONNECT_TIMEOUT)
				except OSError as error:
					text = str(error) or type(error).__name__
					if text != last_error:  # 로봇이 꺼져 있는 동안 같은 줄을 되풀이하지 않는다
						logger.info('게이트웨이에 붙지 못했습니다: %s (계속 다시 시도합니다)', text)
						last_error = text
				else:
					last_error = ''
					if self._run_session(ws, stop_event):
						delay = RECONNECT_MIN_DELAY
				if stop_event.wait(delay):
					break
				delay = min(delay * 2, RECONNECT_MAX_DELAY)
		finally:
			self._close_hardware()
			logger.info('PC 에이전트를 끝냈습니다')

	def _run_session(self, ws: WebSocketClient, stop_event: threading.Event) -> bool:
		"""연결 하나. 끊기거나 stop_event 까지 돈다.

		Returns:
			agent_welcome 을 받았는지 (받았으면 다시 붙기 간격을 처음으로 돌린다).
		"""
		self._is_approved = False
		self._ports = self._port_lister()
		self._calibration_ids = list_calibration_ids(self._calibration_dir)
		hello = {
			'type': 'agent_hello', 'version': PROTOCOL_VERSION, 'host': self._hostname,
			'ports': self._ports, 'calibration_ids': self._calibration_ids,
			'lerobot_version': _lerobot_version()}
		if self._token is not None:
			hello['token'] = self._token  # 60초 안에 다시 붙으면 허락이 이어진다
		is_welcomed = False
		now = time.monotonic()
		next_heartbeat = now
		next_ping = now + PING_PERIOD
		next_ports = now + PORTS_PERIOD
		next_rate_log = now + RATE_LOG_PERIOD
		rate_start, rate_count = now, self._sent_count
		try:
			ws.send_text(json.dumps(hello))
			while not stop_event.is_set():
				now = time.monotonic()
				if now - ws.last_receive_time > SILENCE_TIMEOUT:
					raise ConnectionError(f'게이트웨이가 {SILENCE_TIMEOUT:.0f}초 동안 답하지 않습니다')
				if now >= next_heartbeat:
					ws.send_text(json.dumps({'type': 'heartbeat'}))
					next_heartbeat = now + HEARTBEAT_PERIOD
				if now >= next_ping:
					ws.ping()
					next_ping = now + PING_PERIOD
				if now >= next_ports:
					self._check_ports(ws)
					self._check_calibration_ids()
					next_ports = now + PORTS_PERIOD
				if now >= next_rate_log:
					self._log_rate((self._sent_count - rate_count) / (now - rate_start))
					rate_start, rate_count = now, self._sent_count
					next_rate_log = now + RATE_LOG_PERIOD
				timeout = min(next_heartbeat, next_ping, next_ports) - time.monotonic()
				message = ws.receive(max(timeout, 0.0))
				if message is not None:
					is_welcomed = self._handle(ws, message) or is_welcomed
			# 끝낼 때: 리더 · 보정을 먼저 닫아 그 상태가 아직 열린 연결로 나가게 한다
			self._close_hardware()
		except OSError as error:
			if not stop_event.is_set():
				logger.info('게이트웨이와 끊겼습니다: %s (다시 붙습니다)', error)
		finally:
			self._ws = None
			ws.close()
		return is_welcomed

	def _handle(self, ws: WebSocketClient, text: str | bytes) -> bool:
		"""게이트웨이 메시지 하나. agent_welcome 이었으면 True."""
		try:
			message = json.loads(text)
			kind = message['type']
			if not isinstance(kind, str):
				raise TypeError(kind)
		except (ValueError, TypeError, KeyError):
			logger.warning('알 수 없는 메시지를 버립니다: %.80r', text)
			return False
		if kind == 'agent_welcome':
			self._token = message.get('token')
			self._is_approved = bool(message.get('is_approved'))
			self._ws = ws
			logger.info('게이트웨이에 연결됐습니다 (%s)', self._url)
			self._log_approval()
			# 다시 붙었을 때 리더가 열려 있거나 보정 중이면 게이트웨이도 알아야 한다
			self._send({'type': 'leader_status', **self._link.status})
			self._send({'type': 'calib_status', **self._calibration.status})
			return True
		if kind == 'agent_rejected':
			logger.warning('게이트웨이가 이 에이전트를 받지 않았습니다: %s', message.get('reason'))
		elif kind == 'agent_approved':
			self._is_approved = True
			self._log_approval()
		elif kind == 'agent_unapproved':
			self._is_approved = False
			logger.info('허락이 끝났습니다: %s', message.get('reason') or '이유 없음')
		elif kind == 'leader_connect':
			self._link.connect(str(message.get('port')), str(message.get('calibration_id')))
		elif kind == 'leader_disconnect':
			self._link.disconnect()
		elif kind.removeprefix('calib_') in CALIB_COMMANDS and kind.startswith('calib_'):
			self._calibration.submit(
				kind.removeprefix('calib_'), self._calibration_port(message),
				message.get('calibration_id'))
		else:
			logger.debug('모르는 메시지 종류를 넘깁니다: %s', kind)
		return False

	def _open_leader(self, port: str, calibration_id: str) -> Leader:
		"""게이트웨이가 고른 포트 · 보정을 확인하고 연다 (리더 스레드에서).

		이 PC 가 알린 리더 포트 (/dev/ttyACM*) 만 연다: 다른 시리얼 장치에 쓰지 않는다.
		"""
		if self._calibration.is_busy:
			raise LeaderError('보정 중에는 리더암을 연결할 수 없습니다 (보정을 끝내거나 취소하세요)')
		self._check_port(port)
		if calibration_id not in list_calibration_ids(self._calibration_dir):
			raise LeaderError(
				f'보정 파일이 없습니다: {self._calibration_dir / (calibration_id + ".json")} '
				'(lerobot-calibrate 로 먼저 보정하세요)')
		return self._open(port, calibration_id)

	def _check_port(self, port: object) -> None:
		"""이 PC 가 알린 리더 포트 (/dev/ttyACM* · by-id) 인지. 아니면 LeaderError."""
		ports = self._port_lister()
		names = {entry['path'] for entry in ports} | {entry['by_id'] for entry in ports}
		if port not in names - {None}:
			raise LeaderError(f'이 PC 에 없는 리더 포트입니다: {port} (USB 를 확인하세요)')

	def _calibration_port(self, message: dict) -> str | None:
		"""보정할 포트: 메시지의 port, 없으면 마지막 리더 포트, 없으면 하나뿐인 포트."""
		port = message.get('port') or self._link.status['port']
		if port is None:
			paths = [entry['path'] for entry in self._port_lister()]
			port = paths[0] if len(paths) == 1 else None
		return port

	def _open_calibration_bus(self, port: str | None) -> object:
		"""보정 스레드에서: 포트를 확인하고 리더 버스를 연다."""
		if port is None:
			raise LeaderError('어느 리더 포트인지 모릅니다 (리더암을 한 번 연결하거나 port 를 주세요)')
		self._check_port(port)
		return self._open_bus(port)

	def _release_leader(self) -> None:
		"""보정 스레드에서: 스트리밍 리더를 닫고 포트가 풀릴 때까지 기다린다."""
		if not self._link.close(LEADER_CLOSE_TIMEOUT):
			# 포트를 쥔 채로 버스를 또 열면 두 스레드가 한 버스에 섞인다
			raise CalibrationError(
				'리더암이 아직 포트를 놓지 않았습니다 (연결 중일 수 있습니다. 잠시 뒤 다시 하세요)')

	def _close_hardware(self) -> None:
		self._link.close(LEADER_CLOSE_TIMEOUT)
		self._calibration.close(LEADER_CLOSE_TIMEOUT)

	def _check_calibration_ids(self) -> None:
		"""보정 파일 목록이 바뀌었으면 calibration_ids 를 보낸다 (주기 확인 · 보정 스레드)."""
		calibration_ids = list_calibration_ids(self._calibration_dir)
		if calibration_ids != self._calibration_ids:
			self._calibration_ids = calibration_ids
			self._send({'type': 'calibration_ids', 'calibration_ids': calibration_ids})

	def _check_ports(self, ws: WebSocketClient) -> None:
		ports = self._port_lister()
		if ports != self._ports:
			self._ports = ports
			ws.send_text(json.dumps({'type': 'ports', 'ports': ports}))
			logger.info('포트가 바뀌었습니다: %s', ', '.join(entry['path'] for entry in ports) or '없음')

	def _on_leader_status(self, status: dict) -> None:
		state = status['state']
		where = f'{status["port"]} · 보정 {status["calibration_id"]}'
		if state == STATE_CONNECTING:
			logger.info('리더암 연결 중: %s', where)
		elif state == STATE_CONNECTED:
			logger.info('리더암 연결됨: %s', where)
		elif state == STATE_DISCONNECTED:
			logger.info('리더암을 끊었습니다')
		else:
			logger.warning('리더암 오류: %s', status['reason'])
		self._send({'type': 'leader_status', **status})

	def _on_calib_status(self, status: dict) -> None:
		state = status['state']
		if state != self._last_calib_state:  # 범위를 재는 동안 10 Hz 로 오므로 바뀔 때만 쓴다
			self._last_calib_state = state
			calibration_id = status['calibration_id']
			if state in _CALIB_LOG_LINES:
				logger.info(_CALIB_LOG_LINES[state], calibration_id)
		if status['matches'] is not None:
			matches = ', '.join(
				f'{name} {"맞음" if is_match else "다름"}' for name, is_match in status['matches'].items())
			logger.info('보정 파일 대조: %s', matches or '파일 없음')
		if status['reason']:
			logger.warning('보정: %s', status['reason'])
		self._send({'type': 'calib_status', **status})

	def _send(self, message: dict) -> None:
		"""welcome 받은 연결로 보낸다. 끊겼으면 버린다 (다시 붙으면 지금 상태를 다시 보낸다)."""
		ws = self._ws
		if ws is None:
			return
		try:
			ws.send_text(json.dumps(message))
		except OSError:
			pass  # 끊김은 메인 스레드의 receive 가 알아채고 다시 붙는다

	def _on_leader_joints(self, joints: dict[str, float]) -> None:
		ws = self._ws
		if ws is None:
			return  # 끊긴 동안은 보내지 않는다 (리더는 열어 둔다)
		self._seq += 1
		try:
			ws.send_text(json.dumps({'type': 'leader', 'seq': self._seq, 'joints': joints}))
		except OSError:
			return
		self._sent_count += 1

	def _log_approval(self) -> None:
		if self._is_approved:
			logger.info('허락됨: 팔 세션이 열리면 리더 값이 팔로 갑니다')
		else:
			logger.info('허락 전: 로봇 화면에서 "이 PC 쓰기"를 누르세요')

	def _log_rate(self, rate: float) -> None:
		if self._link.status['state'] != STATE_CONNECTED or self._ws is None:
			return
		approval = '허락됨' if self._is_approved else '허락 전이라 팔로 가지 않음'
		logger.info('리더 값 %.1f Hz 보내는 중 (%s)', rate, approval)
