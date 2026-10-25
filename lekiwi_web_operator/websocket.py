"""RFC 6455 WebSocket 의 서버 쪽 (이 게이트웨이에 필요한 만큼만).

확장 · 서브프로토콜은 받지 않는다. 클라이언트 프레임은 마스크가 있어야 하고 (없으면 1002),
조각난 메시지는 이어 붙인다. 서버 프레임은 마스크 없이 FIN=1 로 보낸다.
"""

import base64
import hashlib
import logging
import socket
import struct
import threading
from email.message import Message
from typing import BinaryIO, Final

from lekiwi_web_operator import config

logger = logging.getLogger(__name__)

ACCEPT_GUID: Final[str] = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

OPCODE_CONTINUATION: Final[int] = 0x0
OPCODE_TEXT: Final[int] = 0x1
OPCODE_BINARY: Final[int] = 0x2
OPCODE_CLOSE: Final[int] = 0x8
OPCODE_PING: Final[int] = 0x9
OPCODE_PONG: Final[int] = 0xA
DATA_OPCODES: Final[tuple[int, ...]] = (OPCODE_CONTINUATION, OPCODE_TEXT, OPCODE_BINARY)
CONTROL_OPCODES: Final[tuple[int, ...]] = (OPCODE_CLOSE, OPCODE_PING, OPCODE_PONG)
CONTROL_PAYLOAD_LIMIT: Final[int] = 125

CLOSE_NORMAL: Final[int] = 1000
CLOSE_GOING_AWAY: Final[int] = 1001
CLOSE_PROTOCOL_ERROR: Final[int] = 1002
CLOSE_POLICY_VIOLATION: Final[int] = 1008
CLOSE_INVALID_DATA: Final[int] = 1007
CLOSE_TOO_BIG: Final[int] = 1009
CLOSE_INTERNAL_ERROR: Final[int] = 1011
CLOSE_TIMEOUT: Final[float] = 1.0   # [s] 닫을 때 송신 잠금 · 닫기 프레임 전송을 기다리는 시간


class WebSocketProtocolError(Exception):
	"""클라이언트가 규약을 어겼다. code 로 연결을 닫는다."""

	def __init__(self, code: int, reason: str) -> None:
		super().__init__(reason)
		self.code = code


def accept_key(key: str) -> str:
	"""Sec-WebSocket-Key 에 대한 Sec-WebSocket-Accept 값 (RFC 6455 §4.2.2)."""
	digest = hashlib.sha1((key + ACCEPT_GUID).encode('ascii')).digest()
	return base64.b64encode(digest).decode('ascii')


def handshake_error(headers: Message) -> str | None:
	"""업그레이드 요청 헤더를 검사한다 (Origin 검사는 http_server 가 한다).

	Args:
		headers: HTTP 요청 헤더.

	Returns:
		문제가 없으면 None, 있으면 거절 이유 (한국어).
	"""
	if headers.get('Upgrade', '').lower() != 'websocket':
		return 'Upgrade: websocket 헤더가 필요합니다'
	tokens = {token.strip().lower() for token in headers.get('Connection', '').split(',')}
	if 'upgrade' not in tokens:
		return 'Connection 헤더에 Upgrade 가 필요합니다'
	if headers.get('Sec-WebSocket-Version') != '13':
		return 'WebSocket 버전 13 만 지원합니다'
	try:
		key = base64.b64decode(headers.get('Sec-WebSocket-Key', ''), validate=True)
	except ValueError:
		key = b''
	if len(key) != 16:
		return 'Sec-WebSocket-Key 가 올바르지 않습니다'
	return None


def encode_frame(opcode: int, payload: bytes) -> bytes:
	"""서버 프레임 하나 (FIN=1, 마스크 없음, 길이 7 · 16 · 64비트)."""
	first = 0x80 | opcode
	length = len(payload)
	if length < 126:
		header = struct.pack('!BB', first, length)
	elif length < 1 << 16:
		header = struct.pack('!BBH', first, 126, length)
	else:
		header = struct.pack('!BBQ', first, 127, length)
	return header + payload


def _check_close_payload(payload: bytes) -> None:
	"""닫기 프레임 본문: 비었거나 [쓸 수 있는 코드 2바이트 + UTF-8 이유] (RFC 6455 §5.5.1 · §7.4).

	Raises:
		WebSocketProtocolError: 1 바이트 본문 · 쓸 수 없는 코드는 1002, UTF-8 이 아닌 이유는 1007.
	"""
	if not payload:
		return
	code = int.from_bytes(payload[:2], 'big')
	if len(payload) == 1 or not (
			1000 <= code <= 1003 or 1007 <= code <= 1014 or 3000 <= code <= 4999):
		raise WebSocketProtocolError(
			CLOSE_PROTOCOL_ERROR, f'닫기 프레임의 코드가 올바르지 않습니다 ({code})')
	try:
		payload[2:].decode('utf-8')
	except UnicodeDecodeError as error:
		raise WebSocketProtocolError(
			CLOSE_INVALID_DATA, '닫기 프레임의 이유가 UTF-8 이 아닙니다') from error


def _unmask(payload: bytes, mask: bytes) -> bytes:
	"""마스크 XOR. 큰 정수 한 번으로 계산해 1 MiB 도 바로 끝난다."""
	length = len(payload)
	if length == 0:
		return payload
	key = (mask * (length // 4 + 1))[:length]
	value = int.from_bytes(payload, 'big') ^ int.from_bytes(key, 'big')
	return value.to_bytes(length, 'big')


class WebSocketConnection:
	"""업그레이드가 끝난 연결 하나. 읽기는 한 스레드, 쓰기는 여러 스레드에서 해도 된다.

	Args:
		sock: 101 응답을 보낸 소켓.
		reader: 그 소켓의 버퍼 읽기 파일. HTTP 파서가 미리 읽어 둔 바이트가 있을 수 있어
			소켓이 아니라 이것으로 읽는다.
	"""

	def __init__(
		self, sock: socket.socket, reader: BinaryIO, read_timeout: float | None = None,
		max_message_bytes: int | None = None,
	) -> None:
		self._sock = sock
		self._reader = reader
		# 아무 프레임도 없으면 닫는 시간 (None 이면 config.WS_READ_TIMEOUT, /agent 는 2 s)
		self._read_timeout = config.WS_READ_TIMEOUT if read_timeout is None else read_timeout
		# 메시지 하나의 최대 크기 (None 이면 읽을 때의 config.WS_MAX_MESSAGE_BYTES, /agent 는 16 KiB)
		self._max_bytes = max_message_bytes
		# 송신 스레드와 pong · 닫기 답장이 겹친다
		self._write_lock = threading.Lock()
		self._is_close_sent = False
		sock.settimeout(self._read_timeout)

	def send_text(self, text: str) -> None:
		"""텍스트 메시지. 닫혔거나 끊겼으면 OSError."""
		self._send_frame(OPCODE_TEXT, text.encode('utf-8'))

	def send_binary(self, data: bytes) -> None:
		"""바이너리 메시지. 닫혔거나 끊겼으면 OSError."""
		self._send_frame(OPCODE_BINARY, data)

	def receive(self) -> str | bytes | None:
		"""다음 메시지를 기다린다 (텍스트는 str, 바이너리는 bytes).

		ping 에는 pong 으로 답한다. 닫기 · 규약 위반 · 시간 초과 · 끊김이면
		알맞은 닫기 프레임을 보내고 None 을 돌려준다.
		"""
		try:
			return self._receive_message()
		except WebSocketProtocolError as error:
			logger.warning('WebSocket 규약 위반으로 닫습니다 (%d): %s', error.code, error)
			self.close(error.code)
		except TimeoutError:
			logger.info('WebSocket 이 %g초 동안 조용해서 닫습니다', self._read_timeout)
			self.close(CLOSE_GOING_AWAY)
		except (OSError, ValueError):
			# 끊김 · 리셋, 또는 다른 스레드가 닫은 파일
			self.close(CLOSE_GOING_AWAY)
		return None

	def close(self, code: int = CLOSE_NORMAL) -> None:
		"""닫기 프레임을 보내고 소켓을 끊는다. 여러 번 · 여러 스레드에서 불러도 된다.

		다른 스레드의 송신이 CLOSE_TIMEOUT 안에 안 끝나면 (막힌 연결) 닫기 프레임 없이
		끊는다. 막힌 연결 하나가 서버 종료 (robot.shutdown) 를 늦추지 않게 한다.
		"""
		if self._write_lock.acquire(timeout=CLOSE_TIMEOUT):
			try:
				if not self._is_close_sent:
					self._is_close_sent = True
					frame = encode_frame(OPCODE_CLOSE, struct.pack('!H', code))
					self._sock.settimeout(CLOSE_TIMEOUT)
					self._sock.sendall(frame)
			except OSError:
				pass
			finally:
				self._write_lock.release()
		try:
			self._sock.shutdown(socket.SHUT_RDWR)
		except OSError:
			pass

	def _max_message_bytes(self) -> int:
		return config.WS_MAX_MESSAGE_BYTES if self._max_bytes is None else self._max_bytes

	def _send_frame(self, opcode: int, payload: bytes) -> None:
		frame = encode_frame(opcode, payload)
		with self._write_lock:
			if self._is_close_sent:
				raise ConnectionError('WebSocket 을 이미 닫았습니다')
			self._sock.sendall(frame)

	def _receive_message(self) -> str | bytes | None:
		message_opcode: int | None = None
		# 조각 목록이 아니라 bytearray 에 모은다: 빈 조각을 끝없이 보내도 메모리가 늘지 않는다
		data = bytearray()
		while True:
			is_final, opcode, payload = self._read_frame(self._max_message_bytes() - len(data))
			if opcode == OPCODE_PING:
				self._send_frame(OPCODE_PONG, payload)
				continue
			if opcode == OPCODE_PONG:
				continue
			if opcode == OPCODE_CLOSE:
				_check_close_payload(payload)
				self.close(CLOSE_NORMAL)
				return None
			if opcode == OPCODE_CONTINUATION:
				if message_opcode is None:
					raise WebSocketProtocolError(
						CLOSE_PROTOCOL_ERROR, '이어 붙일 메시지가 없는데 continuation 이 왔습니다')
			elif message_opcode is not None:
				raise WebSocketProtocolError(
					CLOSE_PROTOCOL_ERROR, '조각난 메시지가 끝나기 전에 새 메시지가 왔습니다')
			else:
				message_opcode = opcode
			data += payload
			if is_final:
				break
		if message_opcode == OPCODE_BINARY:
			return bytes(data)
		try:
			return data.decode('utf-8')
		except UnicodeDecodeError as error:
			raise WebSocketProtocolError(
				CLOSE_INVALID_DATA, '텍스트가 UTF-8 이 아닙니다') from error

	def _read_frame(self, budget: int) -> tuple[bool, int, bytes]:
		"""프레임 하나 → (FIN, opcode, 마스크를 푼 payload).

		Args:
			budget: 데이터 프레임 payload 의 최대 길이 (메시지 크기 제한에서 남은 몫).
				넘으면 payload 를 읽기 전에 1009 로 닫는다.
		"""
		first, second = self._read_exact(2)
		is_final = bool(first & 0x80)
		opcode = first & 0x0F
		length = second & 0x7F
		if first & 0x70:
			raise WebSocketProtocolError(CLOSE_PROTOCOL_ERROR, '확장(RSV 비트)은 쓰지 않습니다')
		if not second & 0x80:
			raise WebSocketProtocolError(CLOSE_PROTOCOL_ERROR, '클라이언트 프레임에 마스크가 없습니다')
		if opcode in CONTROL_OPCODES:
			if not is_final or length > CONTROL_PAYLOAD_LIMIT:
				raise WebSocketProtocolError(
					CLOSE_PROTOCOL_ERROR, '제어 프레임은 조각낼 수 없고 125 바이트 이하여야 합니다')
		elif opcode not in DATA_OPCODES:
			raise WebSocketProtocolError(CLOSE_PROTOCOL_ERROR, f'모르는 opcode {opcode:#x}')
		if length == 126:
			(length,) = struct.unpack('!H', self._read_exact(2))
		elif length == 127:
			(length,) = struct.unpack('!Q', self._read_exact(8))
		mask = self._read_exact(4)
		if opcode in DATA_OPCODES and length > budget:
			raise WebSocketProtocolError(
				CLOSE_TOO_BIG, f'메시지가 {self._max_message_bytes()} 바이트를 넘습니다')
		return is_final, opcode, _unmask(self._read_exact(length), mask)

	def _read_exact(self, size: int) -> bytes:
		data = self._reader.read(size)
		if len(data) < size:  # 버퍼 read 는 끝(EOF)에서만 덜 돌려준다
			raise ConnectionError('상대가 연결을 끊었습니다')
		return data
