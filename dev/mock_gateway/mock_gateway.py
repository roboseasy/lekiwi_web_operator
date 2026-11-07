"""가짜 게이트웨이 (시험 전용, 이 PC 에서만 돈다): web/ 를 서빙하고 /ws 로 status 를 보내며, 로봇 물리를 흉내 낸다.

teleop.py 의 규칙을 옮겼다: 0.3 s 끊기면 목표 0, 목표가 0 이면 출력도 램프 없이 바로 0, 아니면 램프, x·y·yaw × 속도 단계.
주입: GET /mock/inject?freeze_odom=1&clock_offset=-3600&exploring=1&speed_scale=1.1&yaw_drift=0.002 ...
확인: GET /mock/truth (참 경로) · /mock/teleop (받은 teleop 기록) · /mock/reset

사용: python3 mock_gateway.py <web 폴더> <포트>
"""

import base64
import hashlib
import json
import math
import os
import random
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = sys.argv[1]
PORT = int(sys.argv[2])
TEMPLATE = json.load(open(os.path.join(HERE, 'status_template.json')))
INFO = json.load(open(os.path.join(HERE, 'info.json')))
LEVELS = INFO['speed_levels']
WS_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
MIME = {'.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8',
		'.svg': 'image/svg+xml', '.png': 'image/png', '.json': 'application/json'}
LINEAR_STEP, ANGULAR_STEP = 0.02 / 0.05, 0.12 / 0.05   # [m/s²], [rad/s²]


def clamp(value, limit):
	return max(-limit, min(limit, value))


class World:
	def __init__(self):
		self.lock = threading.Lock()
		self.reset()

	def reset(self):
		with self.lock:
			self.truth = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
			self.odom = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
			self.out = [0.0, 0.0, 0.0]
			self.cmd = (0.0, 0.0, 0.0)
			self.cmd_level = 0
			self.cmd_at = -1e9
			self.zero_required = False   # teleop.py _is_zero_required: stop() 뒤 0 입력을 받을 때까지 방향 입력을 버린다
			self.rejected = 0
			self.holder = None
			self.ready = False
			self.estop = False
			self.inject = {'freeze_odom': 0, 'clock_offset': 0.0, 'exploring': 0, 'speed_scale': 1.0, 'odom_scale': 1.0,
						   'yaw_drift': 0.0, 'gyro_bias': 0.0, 'drop_status': 0, 'gyro_sign': 1.0, 'stuck_odom': 0}
			self.frozen = None
			self.frozen_since = None
			self.stuck = None
			self.truth_log = []
			self.teleop_log = []
			self.t0 = time.monotonic()
			self.last_log = 0.0

	def target(self, now):
		if not (self.holder and self.ready and not self.estop and now - self.cmd_at <= 0.3):
			return (0.0, 0.0, 0.0)
		level = max(0, min(self.cmd_level, len(LEVELS) - 1))
		linear, angular = LEVELS[level]
		x, y, yaw = (clamp(value, 1.0) for value in self.cmd)
		norm = math.hypot(x, y)
		if norm > 1.0:
			x, y = x / norm, y / norm
		return (x * linear, y * linear, yaw * angular)

	def step(self, dt):
		with self.lock:
			now = time.monotonic()
			target = self.target(now)
			if target == (0.0, 0.0, 0.0):
				self.out = [0.0, 0.0, 0.0]
			else:
				steps = (LINEAR_STEP * dt, LINEAR_STEP * dt, ANGULAR_STEP * dt)
				self.out = [o + clamp(g - o, s) for o, g, s in zip(self.out, target, steps)]
			inj = self.inject
			wx, wy, ww = (v * inj['speed_scale'] for v in self.out)
			true_w = ww + inj['yaw_drift']
			c, s = math.cos(self.truth['yaw'] + true_w * dt / 2), math.sin(self.truth['yaw'] + true_w * dt / 2)
			self.truth['x'] += (wx * c - wy * s) * dt
			self.truth['y'] += (wx * s + wy * c) * dt
			self.truth['yaw'] += true_w * dt
			c, s = math.cos(self.odom['yaw'] + ww * dt / 2), math.sin(self.odom['yaw'] + ww * dt / 2)
			self.odom['x'] += (wx * c - wy * s) * inj['odom_scale'] * dt
			self.odom['y'] += (wx * s + wy * c) * inj['odom_scale'] * dt
			self.odom['yaw'] += ww * dt
			if now - self.last_log >= 0.05:
				self.truth_log.append((round(now - self.t0, 3), self.truth['x'], self.truth['y'], self.truth['yaw']))
				self.last_log = now

	def status(self):
		with self.lock:
			s = json.loads(json.dumps(TEMPLATE))
			inj = self.inject
			now = time.monotonic()
			s['time'] = time.time() + inj['clock_offset']
			ww = self.out[2] * inj['speed_scale'] + inj['yaw_drift']
			noise = lambda sigma: random.gauss(0, sigma)
			odom = {'x': self.odom['x'] + noise(0.001), 'y': self.odom['y'] + noise(0.001), 'yaw': self.odom['yaw'] + noise(0.0005),
					'vx': self.out[0], 'vy': self.out[1], 'wz': self.out[2]}
			age = 0.02
			if inj['freeze_odom']:
				if self.frozen is None:
					self.frozen, self.frozen_since = dict(odom), now
				odom, age = dict(self.frozen), now - self.frozen_since + 0.02
			else:
				self.frozen = None
			if inj['stuck_odom']:   # 값은 새로 오는데(나이는 작다) 위치가 안 변한다
				if self.stuck is None:
					self.stuck = dict(odom)
				odom = dict(self.stuck)
			else:
				self.stuck = None
			if age > 2.0:   # 게이트웨이는 STALE_DATA_AGE(2 s) 뒤에 null 을 준다
				s['odom'], s['ages']['odom'] = None, None
			else:
				s['odom'] = {k: round(v, 4) for k, v in odom.items()}
				s['ages']['odom'] = round(age, 3)
			s['imu']['gyro'] = [0.0, 0.0, round(inj['gyro_sign'] * (ww + inj['gyro_bias']) + noise(0.001), 4)]
			s['ages']['imu'] = 0.0
			s['control']['holder'] = {'client_id': self.holder, 'label': 'mock · Chrome', 'since': time.time()} if self.holder else None
			s['motor'] = {'ready': self.ready, 'is_powered_by_us': self.ready}
			s['estop'] = {'is_latched': self.estop, 'since': None, 'label': None}
			s['exploration']['state'] = 'exploring' if inj['exploring'] else 'idle'
			allowed = bool(self.holder) and not self.estop
			s['teleop']['is_allowed'] = allowed
			s['teleop']['blocked_reason'] = None if allowed else '조작 권한이 없습니다'
			s['rates'].update({'odom': 50.0, 'imu': 100.0})
			return s


WORLD = World()


def physics_loop():
	last = time.monotonic()
	while True:
		time.sleep(0.01)
		now = time.monotonic()
		WORLD.step(now - last)
		last = now


class WsConnection:
	def __init__(self, sock):
		self.sock = sock
		self.send_lock = threading.Lock()
		self.is_open = True

	def send_text(self, text):
		data = text.encode()
		header = b'\x81'
		if len(data) < 126:
			header += bytes([len(data)])
		elif len(data) < 65536:
			header += b'\x7e' + struct.pack('>H', len(data))
		else:
			header += b'\x7f' + struct.pack('>Q', len(data))
		with self.send_lock:
			self.sock.sendall(header + data)

	def receive(self):
		def exact(count):
			data = b''
			while len(data) < count:
				chunk = self.sock.recv(count - len(data))
				if not chunk:
					raise ConnectionError
				data += chunk
			return data
		while True:
			head = exact(2)
			opcode, masked, length = head[0] & 0x0F, head[1] & 0x80, head[1] & 0x7F
			if length == 126:
				length = struct.unpack('>H', exact(2))[0]
			elif length == 127:
				length = struct.unpack('>Q', exact(8))[0]
			mask = exact(4) if masked else b'\0\0\0\0'
			payload = bytes(b ^ mask[i % 4] for i, b in enumerate(exact(length)))
			if opcode == 8:
				return None
			if opcode == 1:
				return payload.decode()


class Handler(BaseHTTPRequestHandler):
	protocol_version = 'HTTP/1.1'

	def log_message(self, *args):
		pass

	def json_reply(self, payload, status=200):
		body = json.dumps(payload, ensure_ascii=False).encode()
		self.send_response(status)
		self.send_header('Content-Type', 'application/json; charset=utf-8')
		self.send_header('Content-Length', str(len(body)))
		self.send_header('Cache-Control', 'no-store')
		self.end_headers()
		self.wfile.write(body)

	def do_GET(self):
		url = urlparse(self.path)
		if url.path == '/ws':
			return self.websocket()
		if url.path == '/api/info':
			return self.json_reply(INFO)
		if url.path == '/api/status':
			return self.json_reply(WORLD.status())
		if url.path == '/mock/inject':
			with WORLD.lock:
				for key, values in parse_qs(url.query).items():
					WORLD.inject[key] = float(values[0])
			return self.json_reply(WORLD.inject)
		if url.path == '/mock/reset':
			WORLD.reset()
			return self.json_reply({'ok': True})
		if url.path == '/mock/truth':
			with WORLD.lock:
				return self.json_reply({'truth': WORLD.truth_log, 'now': time.monotonic() - WORLD.t0})
		if url.path == '/mock/teleop':
			with WORLD.lock:
				return self.json_reply({'teleop': WORLD.teleop_log, 'now': time.monotonic() - WORLD.t0, 'rejected': WORLD.rejected})
		relative = url.path.lstrip('/') or 'index.html'
		path = os.path.realpath(os.path.join(WEB, relative))
		if not path.startswith(os.path.realpath(WEB)) or not os.path.isfile(path):
			return self.json_reply({'message': 'not found'}, 404)
		body = open(path, 'rb').read()
		self.send_response(200)
		self.send_header('Content-Type', MIME.get(os.path.splitext(path)[1], 'application/octet-stream'))
		self.send_header('Content-Length', str(len(body)))
		self.send_header('Cache-Control', 'no-store')
		self.end_headers()
		self.wfile.write(body)

	def do_POST(self):
		length = int(self.headers.get('Content-Length', '0'))
		body = json.loads(self.rfile.read(length) or b'{}')
		client = self.headers.get('X-Client-Id', '')
		path = urlparse(self.path).path
		with WORLD.lock:
			if path == '/api/control/acquire':
				WORLD.holder = client
				return self.json_reply({'ok': True, 'message': '조작 권한을 받았습니다'})
			if path == '/api/control/release':
				WORLD.holder, WORLD.ready, WORLD.zero_required = None, False, True
				return self.json_reply({'ok': True, 'message': '조작 권한을 놓았습니다'})
			if path == '/api/motor/power':
				WORLD.ready = bool(body.get('enabled')) and bool(WORLD.holder) and not WORLD.estop
				if not body.get('enabled'):
					WORLD.zero_required = True   # 조작 끄기도 teleop.stop() 을 부른다 (robot.py)
				return self.json_reply({'ok': True, 'message': '조작을 켰습니다' if WORLD.ready else '조작을 껐습니다'})
			if path == '/api/estop':
				WORLD.estop, WORLD.ready, WORLD.zero_required = True, False, True
				return self.json_reply({'ok': True, 'message': '비상 정지'})
		return self.json_reply({'ok': False, 'code': 'failed', 'message': f'mock 에 없는 요청 {path}'})

	def websocket(self):
		key = self.headers['Sec-WebSocket-Key']
		accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
		self.send_response(101)
		self.send_header('Upgrade', 'websocket')
		self.send_header('Connection', 'Upgrade')
		self.send_header('Sec-WebSocket-Accept', accept)
		self.end_headers()
		connection = WsConnection(self.connection)
		client_id = f'mock-{random.randint(1000, 9999)}'
		connection.send_text(json.dumps({'type': 'welcome', 'client_id': client_id, 'info': INFO, 'you': {'ip': '127.0.0.1'}}, ensure_ascii=False))

		def writer():
			while connection.is_open:
				if not WORLD.inject['drop_status']:
					try:
						connection.send_text(json.dumps(WORLD.status(), ensure_ascii=False))
					except OSError:
						break
				time.sleep(0.27)
		threading.Thread(target=writer, daemon=True).start()
		try:
			while (text := connection.receive()) is not None:
				message = json.loads(text)
				if message.get('type') == 'teleop' and WORLD.holder == client_id:
					with WORLD.lock:
						command = (float(message['x']), float(message['y']), float(message['yaw']))
						WORLD.teleop_log.append((round(time.monotonic() - WORLD.t0, 3), message['x'], message['y'], message['yaw'], message['level']))
						if WORLD.zero_required:
							if command != (0.0, 0.0, 0.0):
								WORLD.rejected += 1
								continue
							WORLD.zero_required = False
						WORLD.cmd = command
						WORLD.cmd_level, WORLD.cmd_at = int(message['level']), time.monotonic()
		except (ConnectionError, OSError):
			pass
		finally:
			connection.is_open = False


if __name__ == '__main__':
	threading.Thread(target=physics_loop, daemon=True).start()
	ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()
