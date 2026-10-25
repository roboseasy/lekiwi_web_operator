"""PC 에이전트 (lerobot 리더암 프로그램) 의 WebSocket /agent (docs/agent_protocol.md v1).

- 한 번에 하나만 붙는다. 둘째 연결 · 보기 전용 게이트웨이는 agent_rejected 를 받고 닫힌다.
- 생존 신호: 연결마다 감시 스레드가 본다. agent_hello 가 AGENT_HELLO_TIMEOUT 안에 없거나, 맞는 메시지가
  AGENT_SILENCE_TIMEOUT (2 s) 동안 하나도 없으면 닫는다 (바이트를 찔끔 보내 자리를 잡지 못하게: 소켓 읽기
  시간 제한만이 아니다). 메시지는 16 KiB 까지 (그보다 크면 WebSocket 이 닫는다).
- 허락: 조작 권한을 가진 화면이 approve 할 때 지금 연결의 agent_session 을 돌려줘야 한다 (그사이 다른 에이전트로
  바뀌었으면 거절). 허락은 그 권한 번호와 묶인다. 연결이 끊겨도 AGENT_RESUME_WINDOW 안에 같은 token 으로 다시 붙으면
  이어지고, 권한이 끝나면 (번호가 바뀌면) 끝난다 (메시지마다 · 감시 스레드가 본다).
- 허락된 에이전트의 leader 값만, 허락한 권한이 그대로일 때 robot.relay_leader 로 넘긴다 (팔 세션 · 0.5 s 감시는
  robot 이 한다). 허락된 에이전트가 팔 세션 중 끊겨 AGENT_LOSS_STOP_DELAY 안에 돌아오지 않으면 세션을 닫는다.
- 모든 메시지를 여기서 검사한다 (신뢰 경계): 크기, JSON 객체, 형, 유한한 수, 관절 이름, 글 길이 · 출력할 수 있는
  글자, 목록 길이, 빈도. 에이전트 때문에 나는 알림은 종류마다 1초에 한 번까지 (리더암 상태는 허락된 에이전트만).

잠금: _lock 을 잡은 채로 robot · 이벤트 · 소켓을 부르지 않는다. status() 는 소켓에 보내지 않는다.
"""

import collections
import json
import logging
import re
import secrets
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Final

from lekiwi_web_operator import config
from lekiwi_web_operator.robot_api import CommandResult, RobotApi, error_result, ok_result
from lekiwi_web_operator.teleop import finite_number
from lekiwi_web_operator.websocket import CLOSE_GOING_AWAY, CLOSE_POLICY_VIOLATION, WebSocketConnection

logger = logging.getLogger(__name__)

PROTOCOL_VERSIONS: Final[tuple[int, ...]] = (1, 2)   # v2 는 보정 메시지를 더했다 (docs/agent_protocol.md)
AGENT_RESUME_WINDOW: Final[float] = 60.0      # [s] 끊긴 뒤 같은 token 이면 허락이 이어진다
AGENT_MAX_MESSAGE_BYTES: Final[int] = 16 * 1024
MAX_TEXT: Final[int] = 256                    # 글 하나 (경로 · 이름 · 이유) 최대 글자 수
MAX_LIST: Final[int] = 32                     # 포트 · 보정 id 목록 최대 길이
LEADER_RATE_WINDOW: Final[float] = 2.0        # [s] 리더 주기를 재는 구간
AGENT_HELLO_TIMEOUT: Final[float] = 2.0       # [s] 붙은 뒤 이 안에 맞는 agent_hello 가 없으면 닫는다
AGENT_LOSS_STOP_DELAY: Final[float] = 5.0     # [s] 허락된 에이전트가 끊긴 채 이만큼 지나면 팔 세션을 닫는다
WATCH_PERIOD: Final[float] = 0.1              # [s] 감시 스레드 주기
EVENT_MIN_INTERVAL: Final[float] = 1.0        # [s] 에이전트 알림 (종류마다) 의 최소 간격
# [s] 이보다 촘촘한 leader 는 버린다 (30 Hz 를 보내는 에이전트가 ROS 스레드를 넘치게 하지 못하게, 최대 약 60 Hz)
LEADER_MIN_INTERVAL: Final[float] = 1.0 / 65.0
MAX_JOINT_VALUE: Final[float] = 1000.0        # [deg · 0~100] 리더 값 크기 상한 (팔로워가 보정 범위로 다시 자른다)
LEADER_STATES: Final[frozenset[str]] = frozenset({'disconnected', 'connecting', 'connected', 'error'})
LEADER_KEYS: Final[dict[str, str]] = {f'{name}.pos': name for name in config.ARM_JOINTS}
# 리더 보정 (v2 calib_status.state)
CALIB_STATES: Final[frozenset[str]] = frozenset({
	'idle', 'starting', 'waiting_middle', 'recording', 'saving', 'done', 'cancelled', 'error'})
MAX_CALIB_JOINTS: Final[int] = 16
# 화면 명령 → 에이전트 메시지 (calibration_id · port 를 받는 것)
CALIB_COMMANDS: Final[dict[str, tuple[str, bool]]] = {   # action → (type, calibration_id 가 필요한지)
	'check': ('calib_check', False), 'from_motors': ('calib_from_motors', True),
	'start': ('calib_start', True), 'middle': ('calib_middle', False),
	'finish': ('calib_finish', False), 'cancel': ('calib_cancel', False),
}
IDLE_CALIB: Final[dict] = {'state': 'idle', 'calibration_id': None, 'joints': None, 'reason': None}
NOT_CONNECTED_MESSAGE: Final[str] = 'PC 에이전트가 연결되지 않았습니다 (PC에서 실행 명령을 실행하세요)'
NOT_OPERATOR_MESSAGE: Final[str] = '조작 권한이 필요합니다. 먼저 조작 권한을 받으세요'
LEASE_ENDED_REASON: Final[str] = '조작 권한이 끝났습니다'


def _text(value: Any, is_optional: bool = False, limit: int = MAX_TEXT) -> tuple[bool, str | None]:
	"""(맞는지, 값): limit 글자 이하의 출력할 수 있는 str (줄바꿈 · 제어 문자 없음, is_optional 이면 None 도)."""
	if value is None:
		return is_optional, None
	return isinstance(value, str) and len(value) <= limit and value.isprintable(), value


def _clean(value: Any) -> str | None:
	"""사람이 읽는 이유 글: limit 글자로 자르고 줄바꿈 · 제어 문자를 빈칸으로 (로그 · 알림 줄을 깨지 않게)."""
	if not isinstance(value, str):
		return None
	return ''.join(char if char.isprintable() else ' ' for char in value[:MAX_TEXT])


def _ports(value: Any) -> list[dict] | None:
	"""[{path, by_id}] (MAX_LIST 개 이하) 를 검사해 그 모양으로, 잘못됐으면 None."""
	if not isinstance(value, list) or len(value) > MAX_LIST:
		return None
	ports = []
	for item in value:
		if not isinstance(item, dict):
			return None
		is_path, path = _text(item.get('path'))
		is_by_id, by_id = _text(item.get('by_id'), is_optional=True)
		if not (is_path and is_by_id and path):
			return None
		ports.append({'path': path, 'by_id': by_id})
	return ports


def _ids(value: Any) -> list[str] | None:
	"""보정 id 목록 (MAX_LIST 개 이하, 비지 않은 글), 잘못됐으면 None."""
	if not isinstance(value, list) or len(value) > MAX_LIST:
		return None
	if not all(isinstance(item, str) and 0 < len(item) <= MAX_TEXT and item.isprintable() for item in value):
		return None
	return list(value)


def parse_hello(data: dict) -> dict | str:
	"""agent_hello → {version, host, token, ports, calibration_ids, lerobot_version}, 잘못됐으면 이유 (한국어)."""
	version = data.get('version')
	if isinstance(version, bool) or version not in PROTOCOL_VERSIONS:
		return f'규약 버전 {", ".join(map(str, PROTOCOL_VERSIONS))} 만 받습니다 (받은 것: {str(version)[:20]!r})'
	is_host, host = _text(data.get('host'))
	is_token, token = _text(data.get('token'), is_optional=True, limit=64)
	is_version, lerobot_version = _text(data.get('lerobot_version'), is_optional=True, limit=32)
	ports = _ports(data.get('ports', []))
	ids = _ids(data.get('calibration_ids', []))
	if not (is_host and host and is_token and is_version) or ports is None or ids is None:
		return 'agent_hello 의 값이 잘못되었습니다 (host · token · ports · calibration_ids · lerobot_version)'
	return {
		'version': version, 'host': host, 'token': token, 'ports': ports, 'calibration_ids': ids,
		'lerobot_version': lerobot_version,
	}


def parse_calib_status(data: dict) -> dict | None:
	"""calib_status (v2) → {state, calibration_id, matches, joints, reason}, 잘못됐으면 None.

	matches: {보정 id: bool} (MAX_LIST 개까지), joints: {관절 이름: {now, min, max} 정수 tick} (16개까지).
	"""
	state = data.get('state')
	calibration_id = data.get('calibration_id')
	matches, joints = data.get('matches'), data.get('joints')
	if state not in CALIB_STATES:
		return None
	if calibration_id is not None and not is_valid_calibration_id(calibration_id):
		return None
	if matches is not None and not (
		isinstance(matches, dict) and len(matches) <= MAX_LIST
		and all(is_valid_calibration_id(key) and isinstance(value, bool) for key, value in matches.items())
	):
		return None
	if joints is not None:
		if not isinstance(joints, dict) or len(joints) > MAX_CALIB_JOINTS:
			return None
		parsed = {}
		for name, value in joints.items():
			is_name, _name = _text(name, limit=40)
			if not (is_name and isinstance(value, dict)):
				return None
			ticks = [value.get(key) for key in ('now', 'min', 'max')]
			if not all(isinstance(tick, int) and not isinstance(tick, bool) and 0 <= tick <= 4095
					for tick in ticks):
				return None
			parsed[name] = dict(zip(('now', 'min', 'max'), ticks))
		joints = parsed
	return {
		'state': state, 'calibration_id': calibration_id, 'matches': matches, 'joints': joints,
		'reason': _clean(data.get('reason')),
	}


def is_valid_calibration_id(value: Any) -> bool:
	"""보정 이름 [A-Za-z0-9_-]{1,40} 인지."""
	return isinstance(value, str) and re.fullmatch(config.CALIBRATION_ID_PATTERN, value) is not None


def parse_leader(data: dict) -> tuple[int, dict[str, float]] | None:
	"""leader → (seq, {config.ARM_JOINTS 이름: 값}) ('.pos' 를 뗀다). 6 관절이 정확히 · 유한한 수로 · 크기
	MAX_JOINT_VALUE 안에 있어야 한다. 아니면 None."""
	seq, joints = data.get('seq'), data.get('joints')
	if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
		return None
	if not isinstance(joints, dict) or set(joints) != set(LEADER_KEYS):
		return None
	values = {name: finite_number(joints[key]) for key, name in LEADER_KEYS.items()}
	if any(value is None or abs(value) > MAX_JOINT_VALUE for value in values.values()):
		return None
	return seq, values


def parse_leader_status(data: dict) -> dict | None:
	"""leader_status → {state, port, calibration_id, reason}, 잘못됐으면 None."""
	state = data.get('state')
	checks = [_text(data.get(key), is_optional=True) for key in ('port', 'calibration_id')]
	reason = data.get('reason')
	if state not in LEADER_STATES or not all(is_ok for is_ok, _value in checks):
		return None
	if reason is not None and not isinstance(reason, str):
		return None
	port, calibration_id = (value for _is_ok, value in checks)
	return {'state': state, 'port': port, 'calibration_id': calibration_id, 'reason': _clean(reason)}


@dataclass
class _Session:
	"""붙어 있는 에이전트 연결 하나 (AgentLink._lock 으로 지킨다)."""

	connection: WebSocketConnection
	address: str
	connected_at: float = 0.0
	last_message_at: float = 0.0   # 마지막으로 받은 맞는 메시지 (생존 신호)
	token: str | None = None     # agent_hello 를 받은 뒤에만
	session_id: str | None = None   # status.agent.agent_session: 이 연결의 번호 (허락이 이것을 돌려준다)
	version: int = 1
	calibration: dict = field(default_factory=lambda: dict(IDLE_CALIB))   # 리더 보정 (calib_status)
	matches: dict | None = None   # 마지막 calib_check 결과 {보정 id: 맞는지}
	host: str | None = None
	ports: list[dict] = field(default_factory=list)
	calibration_ids: list[str] = field(default_factory=list)
	leader: dict = field(default_factory=lambda: dict.fromkeys(('state', 'port', 'calibration_id', 'reason')))
	last_seq: int = -1
	last_joints: dict | None = None   # 마지막 리더 관절 (화면 표시용)
	leader_times: collections.deque = field(default_factory=lambda: collections.deque(maxlen=256))
	warned: set[str] = field(default_factory=set)   # 경고를 한 번 남긴 잘못된 메시지 종류
	is_ended: threading.Event = field(default_factory=threading.Event)


class AgentLink:
	"""PC 에이전트 연결 · 허락 · 리더 값 중계.

	Args:
		robot: 로봇 쪽 경계 (is_operator · lease_token · relay_leader · get_info).
		emit: (level, message) 모든 화면에 알린다.
		clock: 단조 시계 [s] (시험에서 바꾼다).
	"""

	def __init__(
		self, robot: RobotApi, emit: Callable[[str, str], None],
		clock: Callable[[], float] = time.monotonic,
	) -> None:
		self._robot = robot
		self._emit = emit
		self._clock = clock
		self._lock = threading.Lock()
		self._session: _Session | None = None
		self._approval: tuple[str, int] | None = None   # (허락한 화면 client_id, 그때의 권한 번호)
		self._resume: tuple[str, float] | None = None    # 끊긴 에이전트의 (token, 끊긴 시각)
		self._event_times: dict[str, float] = {}         # 알림 종류 → 마지막으로 낸 시각
		self._loss_timer: threading.Timer | None = None

	# ── 연결 ───────────────────────────────────────────────
	def serve(self, connection: WebSocketConnection, address: str) -> None:
		"""(막힘) /agent 연결 하나의 수명 동안 읽는다. 자리가 차 있거나 보기 전용이면 바로 거절한다."""
		reason = None
		if self._robot.get_info().get('is_read_only'):
			reason = '보기 전용 게이트웨이는 PC 에이전트를 받지 않습니다'
		session = _Session(connection, address, self._clock(), self._clock())
		with self._lock:
			if reason is None and self._session is not None:
				reason = 'PC 에이전트가 이미 연결되어 있습니다 (한 번에 하나만)'
			if reason is None:
				self._session = session   # agent_hello 전에 자리를 잡는다 (동시에 온 둘째를 거절하게)
		if reason is not None:
			logger.warning('PC 에이전트 연결을 거절합니다 (%s): %s', address, reason)
			self._send(session, {'type': 'agent_rejected', 'reason': reason})
			connection.close(CLOSE_POLICY_VIOLATION)
			return
		threading.Thread(target=self._watch, args=(session,), name='agent-watch', daemon=True).start()
		try:
			while (message := connection.receive()) is not None:
				self._handle(session, message)
		except Exception:
			logger.exception('PC 에이전트 %s 처리 중 오류로 연결을 닫습니다', address)
		finally:
			session.is_ended.set()
			with self._lock:
				self._session = None
				if session.token is not None:
					self._resume = (session.token, self._clock())
				approval = self._approval
			connection.close(CLOSE_GOING_AWAY)
			if session.token is not None:
				self._emit_limited('disconnect', 'warn', f'PC 에이전트 연결이 끊겼습니다 ({session.host} · {address})')
			if approval is not None:   # 허락된 에이전트: 돌아오지 않으면 팔 세션을 닫는다
				self._start_loss_timer(approval)

	def close(self) -> None:
		"""(서버 종료) 붙어 있는 에이전트를 닫는다. 읽기 스레드가 정리한다."""
		with self._lock:
			session, timer = self._session, self._loss_timer
		if timer is not None:
			timer.cancel()
		if session is not None:
			session.connection.close(CLOSE_GOING_AWAY)

	# ── 화면 (REST, 조작 권한) ─────────────────────────────
	def approve(self, client_id: str, agent_session: Any = None) -> CommandResult:
		"""이 에이전트의 리더 값을 쓰도록 허락한다 (그 조작 권한이 끝날 때까지).

		agent_session 은 화면이 본 status.agent.agent_session 이어야 한다: 그사이 다른 연결 (다른 PC · 이름을
		흉내 낸 연결) 로 바뀌었으면 거절한다.
		"""
		lease = self._robot.lease_token(client_id)
		if lease is None:
			return error_result('not_operator', NOT_OPERATOR_MESSAGE)
		if not isinstance(agent_session, str) or not agent_session:
			return error_result('bad_request', 'agent_session (status.agent.agent_session) 이 필요합니다')
		with self._lock:
			session = self._ready_session_locked()
			is_same = session is not None and secrets.compare_digest(
				agent_session.encode(), session.session_id.encode())
			if is_same:
				self._approval = (client_id, lease)
		if session is None:
			return error_result('conflict', NOT_CONNECTED_MESSAGE)
		if not is_same:
			return error_result('conflict', '그사이 PC 에이전트 연결이 바뀌었습니다. 화면을 다시 보고 허락하세요')
		self._send(session, {'type': 'agent_approved'})
		message = f'PC 에이전트({session.host})의 리더암을 쓰도록 허락했습니다'
		self._emit('info', message)
		return ok_result(message)

	def unapprove(self, client_id: str) -> CommandResult:
		"""허락을 거둔다. 리더 값을 더 넘기지 않는다."""
		if self._robot.lease_token(client_id) is None:
			return error_result('not_operator', NOT_OPERATOR_MESSAGE)
		if not self._clear_approval('조작 화면이 허락을 거두었습니다'):
			return ok_result('허락한 PC 에이전트가 없습니다')
		return ok_result('PC 에이전트 허락을 거두었습니다')

	def leader_connect(self, client_id: str, port: Any, calibration_id: Any) -> CommandResult:
		"""에이전트에 리더암을 열라고 한다 (port · calibration_id 는 에이전트가 알린 것 중 하나여야 한다)."""
		if self._robot.lease_token(client_id) is None:
			return error_result('not_operator', NOT_OPERATOR_MESSAGE)
		is_port, port = _text(port)
		is_id, calibration_id = _text(calibration_id)
		if not (is_port and port and is_id and calibration_id):
			return error_result('bad_request', 'port 와 calibration_id (문자열) 가 필요합니다')
		with self._lock:
			session = self._ready_session_locked()
			known_ports = set() if session is None else {
				name for item in session.ports for name in (item['path'], item['by_id']) if name}
			known_ids = set() if session is None else set(session.calibration_ids)
		if session is None:
			return error_result('conflict', NOT_CONNECTED_MESSAGE)
		if port not in known_ports or calibration_id not in known_ids:
			return error_result('bad_request', 'PC 에이전트가 알린 포트 · 보정 id 가 아닙니다')
		if not self._send(session, {'type': 'leader_connect', 'port': port, 'calibration_id': calibration_id}):
			return error_result('failed', 'PC 에이전트에 보내지 못했습니다')
		return ok_result(f'리더암 연결을 요청했습니다 ({port}, {calibration_id})')

	def leader_disconnect(self, client_id: str) -> CommandResult:
		"""에이전트에 리더암을 닫으라고 한다."""
		if self._robot.lease_token(client_id) is None:
			return error_result('not_operator', NOT_OPERATOR_MESSAGE)
		with self._lock:
			session = self._ready_session_locked()
		if session is None:
			return error_result('conflict', NOT_CONNECTED_MESSAGE)
		if not self._send(session, {'type': 'leader_disconnect'}):
			return error_result('failed', 'PC 에이전트에 보내지 못했습니다')
		return ok_result('리더암 끊기를 요청했습니다')

	def status(self) -> dict:
		"""status.agent (docs/agent_protocol.md). 소켓에 보내지 않는다 (허락을 거두는 것은 감시 스레드가 한다)."""
		now = self._clock()
		with self._lock:
			approval = self._approval
		is_approved = approval is not None and self._robot.lease_token(approval[0]) == approval[1]
		with self._lock:
			session = self._ready_session_locked()
			if session is None:
				return {
					'is_connected': False, 'agent_session': None, 'host': None, 'address': None,
					'is_approved': False, 'ports': [], 'calibration_ids': [],
					'leader': {**dict.fromkeys(('state', 'port', 'calibration_id', 'reason')),
						'rate_hz': None, 'age': None, 'joints': None},
				}
			times = session.leader_times
			while times and now - times[0] > LEADER_RATE_WINDOW:
				times.popleft()
			return {
				'is_connected': True, 'agent_session': session.session_id, 'host': session.host,
				'address': session.address, 'is_approved': is_approved and self._approval == approval,
				'ports': [dict(item) for item in session.ports],
				'calibration_ids': list(session.calibration_ids),
				'leader': {
					**session.leader,
					'rate_hz': round(len(times) / LEADER_RATE_WINDOW, 1),
					'age': round(now - times[-1], 2) if times else None,
					# 1초 넘게 오지 않은 값은 보이지 않는다
					'joints': session.last_joints if times and now - times[-1] < 1.0 else None,
				},
			}

	# ── 내부 ───────────────────────────────────────────────
	def _handle(self, session: _Session, message: str | bytes) -> None:
		"""에이전트 메시지 하나. 잘못된 것은 버리고 종류마다 경고를 한 번 남긴다 (agent_hello 만 거절 · 닫기)."""
		data = None
		if isinstance(message, str) and len(message) <= AGENT_MAX_MESSAGE_BYTES:
			try:
				data = json.loads(message)
			except (ValueError, RecursionError):
				data = None
		kind = data.get('type') if isinstance(data, dict) else None
		if not isinstance(kind, str):
			self._warn_once(session, 'invalid', '읽을 수 없는 메시지 (JSON 객체 · type · 크기)')
			return
		if session.token is None and kind not in ('agent_hello', 'heartbeat'):
			self._warn_once(session, 'early', f'agent_hello 전의 {kind[:20]!r}')
			return
		with self._lock:
			session.last_message_at = self._clock()   # 읽을 수 있는 메시지만 생존 신호로 친다
		if kind == 'agent_hello':
			self._hello(session, data)
		elif kind == 'leader':
			self._leader(session, data)
		elif kind == 'leader_status':
			self._leader_status(session, data)
		elif kind == 'calib_status':
			self._calib_status(session, data)
		elif kind == 'calibration_ids':
			ids = _ids(data.get('calibration_ids'))
			if ids is None:
				self._warn_once(session, 'calibration_ids', '잘못된 calibration_ids')
			else:
				with self._lock:
					session.calibration_ids = ids
		elif kind == 'ports':
			ports = _ports(data.get('ports'))
			if ports is None:
				self._warn_once(session, 'ports', '잘못된 ports')
			else:
				with self._lock:
					session.ports = ports
		elif kind != 'heartbeat':
			self._warn_once(session, kind[:20], f'모르는 type {kind[:20]!r}')
		self._check_approval()   # 권한이 끝났으면 (0.5 s 마다 오는 heartbeat 로도) 허락을 거둔다

	def _hello(self, session: _Session, data: dict) -> None:
		if session.token is not None:   # 두 번째 agent_hello 는 무시한다
			return
		hello = parse_hello(data)
		if isinstance(hello, str):
			logger.warning('PC 에이전트 %s 를 거절합니다: %r', session.address, hello)
			self._send(session, {'type': 'agent_rejected', 'reason': hello})
			session.connection.close(CLOSE_POLICY_VIOLATION)
			return
		with self._lock:
			resume, self._resume = self._resume, None
			token = hello['token']
			is_resumed = (
				token is not None and resume is not None
				and secrets.compare_digest(resume[0].encode(), token.encode())
				and self._clock() - resume[1] <= AGENT_RESUME_WINDOW
			)
			if not is_resumed:   # 새 에이전트 · 늦게 온 에이전트: 허락은 처음부터
				self._approval = None
				token = secrets.token_hex(16)
			session.token, session.host = token, hello['host']
			session.session_id = secrets.token_hex(8)
			session.version = hello['version']
			session.ports, session.calibration_ids = hello['ports'], hello['calibration_ids']
		self._check_approval()
		with self._lock:
			is_approved = self._approval is not None
		self._send(session, {'type': 'agent_welcome', 'token': token, 'is_approved': is_approved})
		resumed = ' (다시 연결, 허락 이어짐)' if is_resumed and is_approved else ''
		self._emit_limited(
			'connect', 'info', f'PC 에이전트가 연결되었습니다 ({session.host} · {session.address}){resumed}')

	def _leader(self, session: _Session, data: dict) -> None:
		parsed = parse_leader(data)
		now = self._clock()
		with self._lock:
			times = session.leader_times
			while times and now - times[0] > LEADER_RATE_WINDOW:   # 상태를 묻는 화면이 없어도 쌓이지 않게
				times.popleft()
			is_bad = parsed is None
			is_stale = not is_bad and parsed[0] <= session.last_seq
			is_flood = bool(times) and now - times[-1] < LEADER_MIN_INTERVAL
			if not (is_bad or is_stale or is_flood):
				session.last_seq = parsed[0]
				session.last_joints = dict(parsed[1])
				times.append(now)
			approval = self._approval
		if is_bad:
			self._warn_once(session, 'leader', '잘못된 leader (seq · 관절 6개 · 유한한 수)')
		elif not (is_stale or is_flood) and approval is not None and (
			self._robot.lease_token(approval[0]) == approval[1]   # 허락한 그 권한일 때만
		):
			# 팔 세션 (그 권한 + 팔 시작) 이 열려 있을 때만 robot 이 보낸다
			self._robot.relay_leader(approval[0], parsed[1])

	def _leader_status(self, session: _Session, data: dict) -> None:
		leader = parse_leader_status(data)
		if leader is None:
			self._warn_once(session, 'leader_status', '잘못된 leader_status')
			return
		with self._lock:
			previous = session.leader['state']
			session.leader = leader
			is_approved = self._approval is not None
		# 알림은 허락된 에이전트만 (허락 전의 연결이 알림을 쏟아내지 못하게, 1초에 한 번까지)
		if is_approved and leader['state'] != previous and leader['state'] in ('connected', 'error'):
			level = 'info' if leader['state'] == 'connected' else 'warn'
			detail = leader['reason'] if leader['state'] == 'error' else leader['port']
			state = '연결됨' if leader['state'] == 'connected' else '오류'
			self._emit_limited('leader', level, f'리더암 ({session.host}): {state} {detail or ""}'.rstrip())

	def _calib_status(self, session: _Session, data: dict) -> None:
		calib = parse_calib_status(data)
		if calib is None:
			self._warn_once(session, 'calib_status', '잘못된 calib_status')
			return
		with self._lock:
			previous = session.calibration['state']
			if calib['matches'] is not None:
				session.matches = calib['matches']
			joints = calib['joints'] if calib['joints'] is not None else (
				session.calibration['joints'] if calib['state'] == 'recording' else None)
			session.calibration = {
				'state': calib['state'], 'calibration_id': calib['calibration_id'], 'joints': joints,
				'reason': calib['reason']}
		labels = {'waiting_middle': '리더 보정을 시작했습니다', 'done': '리더 보정을 마쳤습니다',
			'cancelled': '리더 보정을 취소했습니다', 'error': '리더 보정이 실패했습니다'}
		if calib['state'] != previous and calib['state'] in labels:
			level = 'error' if calib['state'] == 'error' else 'info'
			detail = f' ({calib["calibration_id"]})' if calib['calibration_id'] else ''
			reason = f': {calib["reason"]}' if calib['reason'] else ''
			self._emit_limited(f'calib-{calib["state"]}', level, f'{labels[calib["state"]]}{detail}{reason}')

	def calibration_status(self) -> dict:
		"""status.calibration.leader: {files: [{id, is_match}], session: {state, calibration_id, joints, reason}}."""
		with self._lock:
			session = self._ready_session_locked()
			if session is None:
				return {'files': [], 'session': dict(IDLE_CALIB)}
			matches = session.matches or {}
			ids = list(dict.fromkeys([*session.calibration_ids, *matches]))
			return {
				'files': [{'id': name, 'is_match': matches.get(name)} for name in ids],
				'session': dict(session.calibration),
			}

	def calibration_leader(self, client_id: str, action: str, body: dict) -> CommandResult:
		"""보정 탭 리더 명령을 에이전트에 넘긴다 (v2): check · from_motors {calibration_id} · start {calibration_id} ·
		middle · finish · cancel. port 는 주면 에이전트가 알린 포트여야 한다."""
		if self._robot.lease_token(client_id) is None:
			return error_result('not_operator', NOT_OPERATOR_MESSAGE)
		if action not in CALIB_COMMANDS:
			return error_result('bad_request', f'모르는 보정 명령입니다: {action}')
		kind, needs_id = CALIB_COMMANDS[action]
		calibration_id, port = body.get('calibration_id'), body.get('port')
		if needs_id and not is_valid_calibration_id(calibration_id):
			return error_result('bad_request', 'calibration_id 는 [A-Za-z0-9_-] 1~40 글자여야 합니다')
		with self._lock:
			session = self._ready_session_locked()
			version = 0 if session is None else session.version
			known_ports = set() if session is None else {
				name for item in session.ports for name in (item['path'], item['by_id']) if name}
		if session is None:
			return error_result('conflict', NOT_CONNECTED_MESSAGE)
		if version < 2:
			return error_result('conflict', 'PC 에이전트가 보정을 지원하지 않습니다 (규약 v2 에이전트로 다시 켜세요)')
		if action in ('check', 'from_motors', 'start') and self._robot.is_arm_session_active():
			# 보정이 리더 버스를 잡고 가운데를 바꾸면 따라가던 팔로워가 튄다
			return error_result('conflict', '팔 원격 조작 중에는 리더 보정을 할 수 없습니다. 먼저 ‘팔 멈춤’을 누르세요')
		message: dict[str, Any] = {'type': kind}
		if needs_id:
			message['calibration_id'] = calibration_id
		if action in ('check', 'from_motors', 'start') and port is not None:
			if not isinstance(port, str) or port not in known_ports:
				return error_result('bad_request', 'PC 에이전트가 알린 포트가 아닙니다')
			message['port'] = port
		if not self._send(session, message):
			return error_result('failed', 'PC 에이전트에 보내지 못했습니다')
		return ok_result(f'리더 보정 {action} 을 PC 에이전트에 보냈습니다')

	def is_calibration_recording(self) -> bool:
		with self._lock:
			session = self._ready_session_locked()
			return session is not None and session.calibration['state'] == 'recording'

	def _check_approval(self) -> None:
		"""허락한 조작 권한이 끝났으면 (번호가 바뀌었으면) 허락을 거두고 알린다."""
		with self._lock:
			approval = self._approval
		if approval is not None and self._robot.lease_token(approval[0]) != approval[1]:
			self._clear_approval(LEASE_ENDED_REASON, approval)

	def _clear_approval(self, reason: str, expected: tuple[str, int] | None = None) -> bool:
		"""허락을 거두고 에이전트에 agent_unapproved 를 보낸다. expected 면 그 허락일 때만. 거뒀으면 True."""
		with self._lock:
			if self._approval is None or (expected is not None and self._approval != expected):
				return False
			self._approval = None
			session = self._ready_session_locked()
		if session is not None:
			self._send(session, {'type': 'agent_unapproved', 'reason': reason})
		self._emit('info', f'PC 에이전트 허락을 거두었습니다: {reason}')
		return True

	def _watch(self, session: _Session) -> None:
		"""(연결마다 스레드) agent_hello 가 AGENT_HELLO_TIMEOUT 안에 없거나 맞는 메시지가 AGENT_SILENCE_TIMEOUT
		동안 없으면 닫는다. 허락한 권한이 끝났는지도 본다 (status() 가 소켓에 보내지 않게 여기서)."""
		while not session.is_ended.wait(WATCH_PERIOD):
			now = self._clock()
			with self._lock:
				is_ready = session.token is not None
				silence = now - session.last_message_at
			if not is_ready and now - session.connected_at > AGENT_HELLO_TIMEOUT:
				logger.warning('PC 에이전트 %s: %g초 안에 agent_hello 가 없어 닫습니다', session.address,
					AGENT_HELLO_TIMEOUT)
				session.connection.close(CLOSE_POLICY_VIOLATION)
				return
			if silence > config.AGENT_SILENCE_TIMEOUT:
				logger.warning('PC 에이전트 %s: %.1f초 동안 메시지가 없어 닫습니다', session.address, silence)
				session.connection.close(CLOSE_GOING_AWAY)
				return
			self._check_approval()

	def _start_loss_timer(self, approval: tuple[str, int]) -> None:
		"""허락된 에이전트가 끊겼다: AGENT_LOSS_STOP_DELAY 안에 돌아오지 않으면 그 권한의 팔 세션을 닫는다."""
		timer = threading.Timer(AGENT_LOSS_STOP_DELAY, self._stop_after_loss, args=(approval,))
		timer.daemon = True
		with self._lock:
			old, self._loss_timer = self._loss_timer, timer
		if old is not None:
			old.cancel()
		timer.start()

	def _stop_after_loss(self, approval: tuple[str, int]) -> None:
		with self._lock:
			is_back = self._ready_session_locked() is not None and self._approval == approval
		if not is_back and self._robot.lease_token(approval[0]) == approval[1]:
			self._robot.end_arm_session(
				approval[0], f'PC 에이전트가 {AGENT_LOSS_STOP_DELAY:g}초 넘게 끊김')

	def _emit_limited(self, kind: str, level: str, message: str) -> None:
		"""에이전트 때문에 나는 알림: 종류마다 EVENT_MIN_INTERVAL 에 한 번까지 (넘치는 것은 로그만)."""
		now = self._clock()
		with self._lock:
			is_due = now - self._event_times.get(kind, -float('inf')) >= EVENT_MIN_INTERVAL
			if is_due:
				self._event_times[kind] = now
		if is_due:
			self._emit(level, message)
		else:
			logger.info('PC 에이전트 알림을 건너뜁니다 (1초에 한 번): %r', message)

	def _ready_session_locked(self) -> _Session | None:
		"""agent_hello 를 마친 연결 (잠금 안)."""
		session = self._session
		return session if session is not None and session.token is not None else None

	def _send(self, session: _Session, message: dict) -> bool:
		try:
			session.connection.send_text(json.dumps(message, ensure_ascii=False))
		except OSError:
			return False
		return True

	def _warn_once(self, session: _Session, kind: str, detail: str) -> None:
		if kind not in session.warned and len(session.warned) < MAX_LIST:
			session.warned.add(kind)
			logger.warning('PC 에이전트 %s 메시지를 버립니다: %r (같은 종류는 더 남기지 않는다)',
				session.address, detail)
