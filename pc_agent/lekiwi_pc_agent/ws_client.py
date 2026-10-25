"""RFC 6455 WebSocket 의 클라이언트 쪽 (에이전트에 필요한 만큼만).

게이트웨이 lekiwi_web_operator/websocket.py (서버 쪽) 의 짝이다. rclpy 가 없는
lerobot 환경에서 돌아야 하므로 그 패키지를 가져오지 않고 따로 둔다.
클라이언트 프레임은 마스크를 씌워 FIN=1 로 보내고, 서버 프레임은 마스크가 없어야 한다.
읽기는 select + 자체 버퍼라서 시간 제한으로 기다려도 프레임 중간이 깨지지 않는다.
"""

import base64
import hashlib
import os
import select
import socket
import struct
import threading
import time
from typing import Final

ACCEPT_GUID: Final[str] = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'

OPCODE_CONTINUATION: Final[int] = 0x0
OPCODE_TEXT: Final[int] = 0x1
OPCODE_BINARY: Final[int] = 0x2
OPCODE_CLOSE: Final[int] = 0x8
OPCODE_PING: Final[int] = 0x9
OPCODE_PONG: Final[int] = 0xA

CLOSE_NORMAL: Final[int] = 1000
CLOSE_NO_STATUS: Final[int] = 1005

MAX_MESSAGE_BYTES: Final[int] = 16 << 20   # 나중의 관측 (JPEG 2장) 까지 넉넉히
MAX_RESPONSE_BYTES: Final[int] = 16384     # 업그레이드 응답 헤더 한도
SEND_TIMEOUT: Final[float] = 2.0           # [s] 막힌 연결에서 보내기가 기다리는 시간
RECV_SIZE: Final[int] = 65536


class WebSocketError(ConnectionError):
	"""업그레이드 실패 · 서버의 규약 위반. 끊김과 같이 다룬다."""


def accept_key(key: str) -> str:
	"""Sec-WebSocket-Key 에 대한 Sec-WebSocket-Accept 값 (RFC 6455 §4.2.2)."""
	digest = hashlib.sha1((key + ACCEPT_GUID).encode('ascii')).digest()
	return base64.b64encode(digest).decode('ascii')


def _mask(payload: bytes, mask: bytes) -> bytes:
	"""마스크 XOR. 큰 정수 한 번으로 계산한다."""
	length = len(payload)
	if length == 0:
		return payload
	key = (mask * (length // 4 + 1))[:length]
	value = int.from_bytes(payload, 'big') ^ int.from_bytes(key, 'big')
	return value.to_bytes(length, 'big')


class WebSocketClient:
	"""업그레이드가 끝난 연결 하나. 읽기는 한 스레드, 쓰기는 여러 스레드에서 해도 된다.

	Attributes:
		last_receive_time: 마지막으로 바이트를 받은 time.monotonic() (pong 포함).
			조용한 게이트웨이 (끊긴 와이파이) 를 알아채는 데 쓴다.
	"""

	def __init__(self, sock: socket.socket, buffered: bytes = b'') -> None:
		self._sock = sock
		self._buffer = bytearray(buffered)
		self._write_lock = threading.Lock()
		self._is_closed = False
		self._message_opcode: int | None = None
		self._fragments = bytearray()
		self.last_receive_time = time.monotonic()

	@classmethod
	def connect(cls, host: str, port: int, path: str, timeout: float) -> 'WebSocketClient':
		"""TCP 연결 + HTTP 업그레이드.

		Args:
			host: 게이트웨이 주소.
			port: 게이트웨이 포트.
			path: 요청 경로 (예: '/agent').
			timeout: [s] TCP 연결 · 업그레이드 응답을 기다리는 시간.

		Returns:
			열린 연결.

		Raises:
			OSError: 연결 실패 · 시간 초과 · 업그레이드 거절 (WebSocketError).
		"""
		sock = socket.create_connection((host, port), timeout=timeout)
		try:
			# 30 Hz 작은 메시지를 모아 보내지 않게 한다 (Nagle 지연 끔)
			sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
			key = base64.b64encode(os.urandom(16)).decode('ascii')
			request = (
				f'GET {path} HTTP/1.1\r\n'
				f'Host: {host}:{port}\r\n'
				'Upgrade: websocket\r\n'
				'Connection: Upgrade\r\n'
				f'Sec-WebSocket-Key: {key}\r\n'
				'Sec-WebSocket-Version: 13\r\n\r\n')
			sock.sendall(request.encode('ascii'))
			response = bytearray()
			while b'\r\n\r\n' not in response:
				data = sock.recv(4096)
				if not data:
					raise WebSocketError('게이트웨이가 업그레이드 응답 전에 끊었습니다')
				response += data
				if len(response) > MAX_RESPONSE_BYTES:
					raise WebSocketError('업그레이드 응답이 너무 깁니다')
			head, _, rest = bytes(response).partition(b'\r\n\r\n')
			lines = head.decode('iso-8859-1').split('\r\n')
			if lines[0].split(' ')[1:2] != ['101']:
				raise WebSocketError(f'게이트웨이가 업그레이드를 거절했습니다 ({lines[0]})')
			headers = {}
			for line in lines[1:]:
				name, _, value = line.partition(':')
				headers[name.strip().lower()] = value.strip()
			if headers.get('sec-websocket-accept') != accept_key(key):
				raise WebSocketError('Sec-WebSocket-Accept 가 맞지 않습니다')
			sock.settimeout(SEND_TIMEOUT)
		except BaseException:
			sock.close()
			raise
		return cls(sock, rest)

	def send_text(self, text: str) -> None:
		"""텍스트 메시지. 닫혔거나 끊겼으면 OSError."""
		self._send_frame(OPCODE_TEXT, text.encode('utf-8'))

	def ping(self) -> None:
		"""ping. 답 pong 은 receive() 가 받아 last_receive_time 을 갱신한다."""
		self._send_frame(OPCODE_PING, b'')

	def receive(self, timeout: float) -> str | bytes | None:
		"""다음 메시지 (텍스트는 str, 바이너리는 bytes) 를 timeout 동안 기다린다.

		ping 에는 pong 으로 답하고, 조각난 메시지는 이어 붙인다.

		Returns:
			메시지, 또는 시간 안에 오지 않으면 None.

		Raises:
			ConnectionError: 상대가 닫았거나 끊었거나 규약을 어겼다 (WebSocketError).
			OSError: 소켓 오류.
		"""
		deadline = time.monotonic() + timeout
		while True:
			frame = self._pop_frame()
			if frame is not None:
				message = self._handle_frame(*frame)
				if message is not None:
					return message
				continue
			remaining = deadline - time.monotonic()
			if remaining <= 0:
				return None
			readable, _, _ = select.select([self._sock], [], [], remaining)
			if not readable:
				return None
			data = self._sock.recv(RECV_SIZE)
			if not data:
				raise ConnectionError('게이트웨이가 연결을 끊었습니다')
			self.last_receive_time = time.monotonic()
			self._buffer += data

	def close(self, code: int = CLOSE_NORMAL) -> None:
		"""닫기 프레임을 보내고 소켓을 닫는다. 여러 번 · 여러 스레드에서 불러도 된다."""
		with self._write_lock:
			if self._is_closed:
				return
			self._is_closed = True
			try:
				self._sock.sendall(self._encode(OPCODE_CLOSE, struct.pack('!H', code)))
			except OSError:
				pass
		try:
			self._sock.shutdown(socket.SHUT_RDWR)
		except OSError:
			pass
		self._sock.close()

	def _encode(self, opcode: int, payload: bytes) -> bytes:
		"""클라이언트 프레임 하나 (FIN=1, 마스크 있음, 길이 7 · 16 · 64비트)."""
		first = 0x80 | opcode
		length = len(payload)
		if length < 126:
			header = struct.pack('!BB', first, 0x80 | length)
		elif length < 1 << 16:
			header = struct.pack('!BBH', first, 0x80 | 126, length)
		else:
			header = struct.pack('!BBQ', first, 0x80 | 127, length)
		mask = os.urandom(4)
		return header + mask + _mask(payload, mask)

	def _send_frame(self, opcode: int, payload: bytes) -> None:
		frame = self._encode(opcode, payload)
		with self._write_lock:
			if self._is_closed:
				raise ConnectionError('WebSocket 을 이미 닫았습니다')
			self._sock.sendall(frame)

	def _pop_frame(self) -> tuple[bool, int, bytes] | None:
		"""버퍼에 다 온 프레임이 있으면 떼어 (FIN, opcode, payload), 아니면 None."""
		buffer = self._buffer
		if len(buffer) < 2:
			return None
		first, second = buffer[0], buffer[1]
		if first & 0x70:
			raise WebSocketError('서버 프레임에 RSV 비트가 있습니다')
		if second & 0x80:
			raise WebSocketError('서버 프레임에 마스크가 있습니다')
		length = second & 0x7F
		offset = 2
		if length == 126:
			if len(buffer) < 4:
				return None
			(length,) = struct.unpack('!H', buffer[2:4])
			offset = 4
		elif length == 127:
			if len(buffer) < 10:
				return None
			(length,) = struct.unpack('!Q', buffer[2:10])
			offset = 10
		if length > MAX_MESSAGE_BYTES:
			raise WebSocketError(f'메시지가 {MAX_MESSAGE_BYTES} 바이트를 넘습니다')
		if len(buffer) < offset + length:
			return None
		payload = bytes(buffer[offset:offset + length])
		del buffer[:offset + length]
		return bool(first & 0x80), first & 0x0F, payload

	def _handle_frame(self, is_final: bool, opcode: int, payload: bytes) -> str | bytes | None:
		"""프레임 하나를 처리한다. 메시지가 끝났으면 그 메시지, 아니면 None."""
		if opcode == OPCODE_PING:
			self._send_frame(OPCODE_PONG, payload)
			return None
		if opcode == OPCODE_PONG:
			return None
		if opcode == OPCODE_CLOSE:
			code = int.from_bytes(payload[:2], 'big') if len(payload) >= 2 else CLOSE_NO_STATUS
			reason = payload[2:].decode('utf-8', 'replace')
			self.close(CLOSE_NORMAL)
			raise ConnectionError(f'게이트웨이가 연결을 닫았습니다 ({code} {reason})'.rstrip())
		if opcode == OPCODE_CONTINUATION:
			if self._message_opcode is None:
				raise WebSocketError('이어 붙일 메시지가 없는데 continuation 이 왔습니다')
		elif opcode in (OPCODE_TEXT, OPCODE_BINARY):
			if self._message_opcode is not None:
				raise WebSocketError('조각난 메시지가 끝나기 전에 새 메시지가 왔습니다')
			self._message_opcode = opcode
		else:
			raise WebSocketError(f'모르는 opcode {opcode:#x}')
		self._fragments += payload
		if len(self._fragments) > MAX_MESSAGE_BYTES:
			raise WebSocketError(f'메시지가 {MAX_MESSAGE_BYTES} 바이트를 넘습니다')
		if not is_final:
			return None
		data = bytes(self._fragments)
		message_opcode = self._message_opcode
		self._fragments.clear()
		self._message_opcode = None
		if message_opcode == OPCODE_BINARY:
			return data
		try:
			return data.decode('utf-8')
		except UnicodeDecodeError as error:
			raise WebSocketError('텍스트가 UTF-8 이 아닙니다') from error
