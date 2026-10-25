"""네트워크 상태 · 와이파이 검색 · 연결 · AP 모드 (SPEC §6 network · §10, docs/ap_mode_spec.md).

wlan0 은 NetworkManager 가 맡는다. 공유기 연결 · AP 켜기 · 끄기 · AP 이름 바꾸기는 로봇의
~/lekiwi_net 스크립트 (run_wifi.sh · run_ap.sh · set_ap.sh) 를 `sudo -n` 으로 부르고, 모드는
스크립트 출력이 아니라 `iw` 로 읽는다. 비밀번호는 절대 로그에 남기지 않고 AP 비밀번호는 읽어 오지 않는다.
"""

import logging
import os
import re
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Final

import psutil

from lekiwi_web_operator import config
from lekiwi_web_operator.robot_api import CommandResult, error_result, ok_result

logger = logging.getLogger(__name__)

COMMAND_TIMEOUT: Final[float] = 2.0           # [s] iw · systemctl · sudo -n true · nm_kf.py get
SCAN_TIMEOUT: Final[float] = 15.0             # [s]
SWITCH_TIMEOUT: Final[float] = 150.0          # [s] 전환 스크립트 하나 (최악: 공유기 30 s → 실패하면 AP 올리기 기본 90 s)
NET_DIR_NAME: Final[str] = 'lekiwi_net'       # 홈 폴더 기준, 골든 이미지의 NetworkManager 세대 스크립트
RUN_AP_SCRIPT: Final[str] = 'run_ap.sh'
RUN_WIFI_SCRIPT: Final[str] = 'run_wifi.sh'
SET_AP_SCRIPT: Final[str] = 'set_ap.sh'
KEYFILE_TOOL: Final[str] = 'nm_kf.py'
NET_SCRIPTS: Final[tuple[str, ...]] = (RUN_AP_SCRIPT, RUN_WIFI_SCRIPT, SET_AP_SCRIPT, KEYFILE_TOOL)
AP_HOST: Final[str] = '192.168.4.1'           # lekiwi-ap 프로필의 로봇 주소
HIDDEN_SSID_LABEL: Final[str] = '(숨김)'
OPEN_SECURITY_LABEL: Final[str] = '개방'
AP_SCAN_REASON: Final[str] = (
	'AP 모드에서는 와이파이를 검색할 수 없습니다. 공유기 모드로 돌아가면 검색할 수 있습니다')
SWITCHING_MESSAGE: Final[str] = '이미 네트워크를 바꾸고 있습니다. 끝난 뒤 다시 하세요'
NO_ROUTER_REASON: Final[str] = (
	'저장된 공유기가 없어 공유기 모드로 돌아갈 수 없습니다. 공유기 이름과 비밀번호를 입력해 연결하세요')
# str.isprintable() 이 거절하는 글자 (제어 · 서식 문자, 일반 공백이 아닌 공백). 화면 network.js 와 같은 문구
UNPRINTABLE_TEXT: Final[str] = '제어 문자나 특수 공백(줄바꿈 · 탭 · 전각 공백 등)을 쓸 수 없습니다'
_ESCAPED_BYTE: Final[re.Pattern[bytes]] = re.compile(rb'\\x([0-9a-fA-F]{2})')
_IW_CHANNEL: Final[re.Pattern[str]] = re.compile(r'(\d+) \(([\d.]+) MHz\)')   # '6 (2437 MHz), width: …'
SYS_NET: Final[Path] = Path('/sys/class/net')     # 인터페이스마다 carrier · speed · device · wireless
PROC_ROUTE: Final[Path] = Path('/proc/net/route')  # IPv4 경로 표 (기본 경로 · 게이트웨이 · metric)
# status.exploration.state 중 자동 주행이 바퀴를 움직이는 상태 ('saving' 은 멈춘 뒤, 화면 isExploring 과 같다)
EXPLORING_STATES: Final[frozenset[str]] = frozenset({'starting', 'exploring', 'waiting'})
STATUS_UNKNOWN_REASON: Final[str] = '로봇 상태를 확인하지 못해 바꿀 수 없습니다'


def decode_ssid(text: str) -> str:
	"""iw 가 `\\xNN` 으로 바꿔 찍은 바이트 (한글 · 앞뒤 공백 · 역슬래시) 를 되돌린다."""
	raw = _ESCAPED_BYTE.sub(
		lambda match: bytes([int(match.group(1), 16)]), text.encode('utf-8'))
	return raw.decode('utf-8', errors='replace')


def _number(text: str) -> float | None:
	"""'-52.00 dBm' · '5180.0' 의 앞 숫자."""
	try:
		return float(text.split()[0])
	except (IndexError, ValueError):
		return None


def _mhz(text: str) -> int | None:
	value = _number(text)
	return None if value is None else round(value)


def parse_iw_interfaces(text: str) -> list[str]:
	"""`iw dev` 출력 → 무선 인터페이스 이름 (나온 순서)."""
	return re.findall(r'^\s*Interface\s+(\S+)', text, re.MULTILINE)


def parse_iw_link(text: str) -> dict:
	"""`iw dev <if> link` 출력 → {'is_connected', 'ssid', 'signal_dbm', 'freq_mhz'}."""
	link = {'is_connected': text.startswith('Connected to'), 'ssid': None,
		'signal_dbm': None, 'freq_mhz': None}
	if not link['is_connected']:
		return link
	for line in text.splitlines():
		stripped = line.strip()  # iw 는 SSID 앞뒤 공백을 \x20 으로 찍으므로 잘라도 된다
		if stripped.startswith('SSID:'):
			link['ssid'] = decode_ssid(stripped[len('SSID:'):].strip())
		elif stripped.startswith('signal:'):
			link['signal_dbm'] = _number(stripped[len('signal:'):])
		elif stripped.startswith('freq:'):
			link['freq_mhz'] = _mhz(stripped[len('freq:'):])
	return link


def parse_iw_info(text: str) -> dict:
	"""`iw dev <if> info` 출력 → {'type', 'ssid', 'channel', 'freq_mhz'}. 없는 값은 None.

	type 은 'AP' · 'managed' 처럼 iw 가 찍은 그대로다. 연결이 없으면 ssid · channel 줄이 없다.
	"""
	info = {'type': None, 'ssid': None, 'channel': None, 'freq_mhz': None}
	for line in text.splitlines():
		key, _, value = line.strip().partition(' ')  # iw 는 SSID 앞뒤 공백을 \x20 으로 찍는다
		if key == 'type':
			info['type'] = value or None
		elif key == 'ssid':
			info['ssid'] = decode_ssid(value)
		elif key == 'channel' and (match := _IW_CHANNEL.match(value)):
			info['channel'], info['freq_mhz'] = int(match[1]), _mhz(match[2])
	return info


def parse_station_count(text: str) -> int:
	"""`iw dev <if> station dump` 출력 → 붙은 기기 수."""
	return sum(line.startswith('Station ') for line in text.splitlines())


def _parse_bss(block: str) -> dict:
	"""scan 출력의 BSS 하나 → 네트워크 dict."""
	ssid = ''
	signal_dbm = None
	freq_mhz = None
	has_rsn = has_wpa = has_privacy = False
	for line in block.splitlines():
		stripped = line.strip()
		if stripped.startswith('SSID:'):
			ssid = decode_ssid(stripped[len('SSID:'):].strip())
		elif stripped.startswith('signal:'):
			signal_dbm = _number(stripped[len('signal:'):])
		elif stripped.startswith('freq:'):
			freq_mhz = _mhz(stripped[len('freq:'):])
		elif stripped.startswith('capability:'):
			has_privacy = 'Privacy' in stripped.split()
		elif stripped.startswith('RSN:'):
			has_rsn = True
		elif stripped.startswith('WPA:'):
			has_wpa = True
	if has_rsn:
		security = 'WPA2/3'
	elif has_wpa:
		security = 'WPA'
	elif has_privacy:
		security = 'WEP'
	else:
		security = OPEN_SECURITY_LABEL
	is_hidden = not ssid.strip('\x00')
	return {
		'ssid': HIDDEN_SSID_LABEL if is_hidden else ssid,
		'signal_dbm': signal_dbm,
		'freq_mhz': freq_mhz,
		'security': security,
		'is_hidden': is_hidden,
	}


def parse_iw_scan(text: str) -> list[dict]:
	"""`iw dev <if> scan` 출력 → SSID 마다 가장 센 신호 하나, 센 순서.

	~/wifi_list.sh 와 같은 규칙: RSN → 'WPA2/3', WPA → 'WPA', Privacy → 'WEP', 아니면 '개방',
	숨김 SSID 는 '(숨김)' (is_hidden=True, 이 이름으로는 연결할 수 없다).
	"""
	best: dict[str, dict] = {}
	for block in re.split(r'^BSS ', text, flags=re.MULTILINE)[1:]:
		network = _parse_bss(block)
		previous = best.get(network['ssid'])
		if previous is None or _signal_key(network) > _signal_key(previous):
			best[network['ssid']] = network
	return sorted(best.values(), key=_signal_key, reverse=True)


def _signal_key(network: dict) -> float:
	signal_dbm = network['signal_dbm']
	return float('-inf') if signal_dbm is None else signal_dbm


def credentials_problem(ssid: str, password: str) -> str | None:
	"""공유기 SSID · 비밀번호로 run_wifi.sh 에 넘길 수 있는 값인지 본다 (화면 routerCredentialsProblem 과 같다).

	AP 이름 규칙과 같은 까닭으로 제어 문자 · 특수 공백 · 앞뒤 공백 (공백만 포함) · SSID 의 `;` 를 막는다
	(keyfile 이 앞뒤 공백을 버리고, ssid= 의 `;` 는 목록 구분자라 '65;66;' 이 다른 SSID 'AB' 가 된다).
	`"` · `\\` 는 아직 시험하지 않은 글자라 막아 둔다 (기획서 5장 입력 검증).
	WPA 비밀번호 길이는 바이트로 센다 (한글은 한 글자 3 바이트).

	Returns:
		문제가 없으면 None, 있으면 사용자에게 보여 줄 이유.
	"""
	if not ssid.isprintable() or not password.isprintable():
		return f'SSID · 비밀번호에 {UNPRINTABLE_TEXT}'
	if ssid != ssid.strip() or password != password.strip():
		return 'SSID · 비밀번호의 앞뒤에 공백을 둘 수 없습니다'
	if ';' in ssid:
		return 'SSID 에 ;(세미콜론)는 쓸 수 없습니다'
	if any(char in '"\\' for char in ssid + password):
		return ('SSID · 비밀번호에 큰따옴표(")와 역슬래시(\\)는 쓸 수 없습니다 '
			'(아직 시험하지 않은 글자입니다)')
	if not 1 <= len(ssid.encode('utf-8')) <= 32:
		return 'SSID 는 1~32 바이트여야 합니다'
	if not 8 <= len(password.encode('utf-8')) <= 63:
		return ('비밀번호는 8~63 바이트여야 합니다 (한글은 한 글자 3 바이트, '
			'비밀번호 없는 와이파이는 지원하지 않습니다)')
	return None


def ap_credentials_problem(ssid: str, password: str) -> str | None:
	"""set_ap.sh 에 넘길 AP 이름 · 비밀번호인지 본다 (기획서 5장 입력 검증).

	이름은 1~32 바이트 (한글은 한 글자 3 바이트), 비밀번호는 WPA2 규격대로 인쇄 가능한 ASCII 8~63자
	(화면 apCredentialsProblem 과 같다). 둘 다 제어 문자 · 특수 공백 · 앞뒤 공백을 쓸 수 없다 (keyfile 이
	앞뒤 공백을 버리고, 폰에서 치기 어렵다). 이름의 `;` 도 막는다: keyfile 의 ssid= 는 `;` 를 목록 구분자로
	읽는데 (NetworkManager 는 `\\;` 로 쓴다) nm_kf.py 는 `\\` 만 두 배로 쓴다. 이름의 `\\` 는 공유기 이름처럼
	막는다. 비밀번호 (psk=) 의 `;` 는 그냥 글자라 괜찮다.

	Returns:
		문제가 없으면 None, 있으면 사용자에게 보여 줄 이유.
	"""
	if not ssid.isprintable() or not password.isprintable():
		return f'AP 이름 · 비밀번호에 {UNPRINTABLE_TEXT}'
	if ssid != ssid.strip() or password != password.strip():
		return 'AP 이름 · 비밀번호의 앞뒤에 공백을 둘 수 없습니다'
	if ';' in ssid:
		return 'AP 이름에 ;(세미콜론)는 쓸 수 없습니다'
	if '\\' in ssid:
		return 'AP 이름에 역슬래시(\\)는 쓸 수 없습니다'
	if not 1 <= len(ssid.encode('utf-8')) <= 32:
		return 'AP 이름은 1~32 바이트여야 합니다 (한글은 한 글자 3 바이트)'
	if not (password.isascii() and 8 <= len(password) <= 63):
		return 'AP 비밀번호는 영문 · 숫자 · 기호 8~63자여야 합니다 (한글은 쓸 수 없습니다)'
	return None


def network_switch_block(status: dict) -> str | None:
	"""로봇이 움직일 수 있어 와이파이 · AP 를 바꾸면 안 되는 이유 (기획서 U2). 바꿔도 되면 None.

	전환하면 화면이 끊겨 비상 정지를 누를 수 없다. 이 판정은 서버의 이 함수 한 곳에서만 한다
	(화면은 다시 계산하지 않는다). 키가 없거나 모양이 다르면 거절한다 (fail closed).

	Args:
		status: robot.get_status() (mode · motor · exploration · teleop_arm 을 본다).
	"""
	try:
		mode = status['mode']
		is_wheel_on = status['motor']['ready'] is True   # None 은 드라이버가 없다 (로봇 꺼짐)
		is_exploring = status['exploration']['state'] in EXPLORING_STATES
		arm = status['teleop_arm']
		is_arm_session, is_arm_torque = arm['is_session_active'], arm['is_torque_maybe_on']
	except (KeyError, TypeError):
		return STATUS_UNKNOWN_REASON
	if is_exploring:
		return '자동 주행 맵 생성 중이라 네트워크를 바꿀 수 없습니다. 먼저 ‘자동 주행 멈춤’을 누르세요'
	if mode == 'navigation':
		return '자율 주행 중이라 네트워크를 바꿀 수 없습니다. 먼저 자율 주행을 끄세요'
	if mode == 'mapping':
		return '지도 작성 중이라 네트워크를 바꿀 수 없습니다. 먼저 지도 작성을 끄세요'
	if mode != 'idle':
		return STATUS_UNKNOWN_REASON
	if is_arm_session:
		return ('팔 원격 조작 중이라 네트워크를 바꿀 수 없습니다. '
			'조작 화면에서 먼저 ‘멈춤’과 ‘팔 힘 빼기’를 누르세요')
	if is_wheel_on:
		return '바퀴 토크가 켜져 있어 네트워크를 바꿀 수 없습니다. 먼저 ‘조작 끄기’를 누르세요'
	if is_arm_torque:
		return '팔 토크가 켜져 있을 수 있어 네트워크를 바꿀 수 없습니다. 조작 화면에서 먼저 ‘팔 힘 빼기’를 누르세요'
	return None


def _run(argv: list[str]) -> str | None:
	"""짧은 명령의 표준 출력. 실패하면 None. 비밀번호가 든 명령에는 쓰지 않는다."""
	try:
		completed = subprocess.run(
			argv, capture_output=True, text=True, encoding='utf-8', errors='replace',
			timeout=COMMAND_TIMEOUT, check=False)
	except (OSError, subprocess.TimeoutExpired):
		return None
	return completed.stdout if completed.returncode == 0 else None


def _wireless_interface() -> str | None:
	"""wlan0 (AP · 공유기 프로필이 쓰는 이름). 없으면 첫 무선 인터페이스 (USB 와이파이가 먼저 나와도 wlan0)."""
	names = parse_iw_interfaces(_run(['iw', 'dev']) or '')
	if 'wlan0' in names:
		return 'wlan0'
	return names[0] if names else None


def _switch_problem(net_dir: Path) -> str | None:
	"""~/lekiwi_net 스크립트로 와이파이 · AP 를 바꿀 수 없는 이유. 바꿀 수 있으면 None."""
	missing = [name for name in NET_SCRIPTS if not os.access(net_dir / name, os.X_OK)]
	if missing:
		return (f'이 로봇에는 ~/{NET_DIR_NAME} 의 {", ".join(missing)} 가 없거나 실행할 수 없어 와이파이 · AP '
			'모드를 바꿀 수 없습니다. 먼저 로봇을 NetworkManager 세대로 옮기세요 (robot_net/provision_ap_mode.sh)')
	if (_run(['systemctl', 'is-active', 'NetworkManager']) or '').strip() != 'active':
		return 'NetworkManager 가 켜져 있지 않아 와이파이 · AP 모드를 바꿀 수 없습니다'
	if _run(['sudo', '-n', 'true']) is None:
		return '비밀번호 없는 sudo 가 안 돼 와이파이 · AP 모드를 바꿀 수 없습니다'
	return None


def _interfaces() -> list[dict]:
	"""루프백을 뺀 인터페이스: {'name', 'is_up', 'ipv4': [..]}."""
	stats = psutil.net_if_stats()
	return [
		{
			'name': name,
			'is_up': name in stats and stats[name].isup,
			'ipv4': [item.address for item in addresses if item.family == socket.AF_INET],
		}
		for name, addresses in sorted(psutil.net_if_addrs().items())
		if name != 'lo'
	]


def parse_default_routes(text: str) -> dict[str, tuple[str, int]]:
	"""/proc/net/route 의 기본 경로 → {인터페이스: (게이트웨이 IPv4, metric)}. 인터페이스마다 metric 이 가장 작은 것."""
	routes: dict[str, tuple[str, int]] = {}
	for line in text.splitlines()[1:]:
		fields = line.split()
		if len(fields) < 7 or fields[1] != '00000000':
			continue
		try:
			gateway = socket.inet_ntoa(int(fields[2], 16).to_bytes(4, 'little'))
			metric = int(fields[6])
		except (ValueError, OverflowError):
			continue
		if fields[0] not in routes or metric < routes[fields[0]][1]:
			routes[fields[0]] = (gateway, metric)
	return routes


def _read_text(path: Path) -> str | None:
	try:
		return path.read_text(encoding='utf-8').strip()
	except OSError:   # 케이블이 없거나 꺼진 인터페이스는 speed 를 읽으면 EINVAL 이다
		return None


def _ethernet(interfaces: list[dict]) -> tuple[list[dict], str | None]:
	"""유선 인터페이스 (실물 장치가 있고 무선이 아닌 것)와 기본 경로(인터넷이 나가는) 인터페이스 이름.

	Returns:
		([{'interface', 'is_link', 'speed_mbps', 'ipv4', 'gateway', 'is_default_route'}], 기본 경로 이름 또는 None).
	"""
	routes = parse_default_routes(_read_text(PROC_ROUTE) or '')
	default = min(routes, key=lambda name: routes[name][1]) if routes else None
	wired = []
	for item in interfaces:
		base = SYS_NET / item['name']
		if not (base / 'device').exists() or (base / 'wireless').exists():
			continue
		speed = _read_text(base / 'speed')
		wired.append({
			'interface': item['name'],
			'is_link': _read_text(base / 'carrier') == '1',
			'speed_mbps': int(speed) if speed and speed.isdigit() and int(speed) > 0 else None,
			'ipv4': item['ipv4'],
			'gateway': routes[item['name']][0] if item['name'] in routes else None,
			'is_default_route': item['name'] == default,
		})
	return wired, default


def _last_line(text: str) -> str:
	lines = text.strip().splitlines()
	return lines[-1] if lines else ''


def _error_line(output: str) -> str:
	"""스크립트 출력의 마지막 'error:' 줄에서 'error:' 를 뺀 내용 (앱 · 화면이 읽는 줄). 없으면 ''."""
	found = [line.strip()[len('error:'):].strip() for line in output.splitlines()
		if line.strip().startswith('error:')]
	return found[-1] if found else ''


def _scan_error(code: str, message: str) -> CommandResult:
	"""scan 실패 결과 (networks 가 늘 있다)."""
	return error_result(code, message, networks=[])


class NetworkService:
	"""status.network · network_features · 와이파이 검색 · 공유기 연결 · AP 모드.

	Args:
		port: 게이트웨이 포트 (접속 주소 안내에 쓴다).
	"""

	def __init__(self, port: int) -> None:
		self._port = port
		self._lock = threading.Lock()            # _status · _features
		self._status: dict | None = None
		self._status_time = 0.0
		self._features: dict | None = None
		self._router_ssid: str | None = None     # home-wifi 프로필의 공유기 이름 (_features 와 같이 읽는다)
		self._last_switch: dict | None = None
		self._scan_lock = threading.Lock()       # iw scan 은 한 번에 하나
		self._switch_lock = threading.Lock()     # 전환 스레드 (연결 · AP 켜기 · 끄기 · 설정) 가 도는 동안

	def get_status(self) -> dict:
		"""§6 network. NETWORK_INFO_PERIOD 동안 같은 dict 를 돌려준다 (바꾸지 말 것).

		Returns:
			{'hostname', 'interfaces', 'wifi', 'ethernet', 'default_route', 'mode', 'ap', 'ap_ssid',
			'router_ssid', 'last_switch'}.
			무선이 없으면 wifi 는 None. ethernet 은 유선 인터페이스 목록, default_route 는 인터넷이
			나가는 인터페이스 이름 (없으면 None). mode 는 'ap' (AP 로 동작) · 'client' (공유기에 붙음) ·
			'none' (붙은 곳 없음) · None (iw · 무선이 없거나 출력을 모름). ap 는 AP 일 때만
			{'ssid', 'ip', 'channel', 'freq_mhz', 'clients'} (clients 는 붙은 기기 수, 못 읽으면 None).
			ap_ssid 는 features()['ap_ssid'] 와 같다: info 는 연결할 때 한 번만 가므로 ap_config 로 바꾼
			이름을 화면이 상태로 본다 (공유기 모드에서도 'AP 를 켜면 붙을 이름' 안내).
			router_ssid 는 home-wifi 프로필에 저장된 공유기 이름 (없으면 None: '공유기 모드로 돌아가기' 를 못 한다).
			last_switch 는 마지막 전환 결과 {'ok', 'action': 'ap_on'|'ap_off'|'ap_config'|'connect', 'message':
			실패면 스크립트의 마지막 error: 줄 (없으면 '종료 코드 N'), 성공이면 '', 'at': time.time()} 또는 None.
		"""
		self.features()   # 기능 검사를 먼저 끝낸다 (상태가 info 보다 먼저 불릴 수 있다)
		with self._lock:
			now = time.monotonic()
			age = now - self._status_time
			if self._status is None or age >= config.NETWORK_INFO_PERIOD:
				interfaces = _interfaces()
				ethernet, default_route = _ethernet(interfaces)
				wifi, mode, ap = self._wifi_status()
				self._status = {
					'hostname': socket.gethostname(),
					'interfaces': interfaces,
					'wifi': wifi,
					'ethernet': ethernet,
					'default_route': default_route,
					'mode': mode,
					'ap': ap,
					'ap_ssid': self._features['ap_ssid'],
					'router_ssid': self._router_ssid,
					'last_switch': self._last_switch,
				}
				self._status_time = now
			return self._status

	def features(self) -> dict:
		"""§6 network_features. 처음 불릴 때 한 번 검사한다 (저장된 AP · 공유기 이름도 이때 처음 읽는다).

		Returns:
			{'can_scan_wifi', 'can_connect_wifi', 'can_ap_mode', 'ap_mode_reason': 바꿀 수 없는 이유 | None,
			'ap_ssid': lekiwi-ap 프로필의 AP 이름 | None}. AP 비밀번호는 읽지 않는다.
		"""
		with self._lock:
			if self._features is None:
				problem = _switch_problem(self._net_dir())
				names = {} if problem is not None else self._saved_names()
				self._router_ssid = names.get('router_ssid')
				self._features = {
					'can_scan_wifi': shutil.which('iw') is not None,
					'can_connect_wifi': problem is None,
					'can_ap_mode': problem is None,
					'ap_mode_reason': problem,
					'ap_ssid': names.get('ap_ssid'),
				}
			return self._features

	def access_urls(self) -> list[str]:
		"""브라우저 접속 주소: http://<hostname>.local:<port> 와 IPv4 마다 하나."""
		urls = [f'http://{socket.gethostname()}.local:{self._port}']
		for addresses in psutil.net_if_addrs().values():
			for item in addresses:
				if item.family == socket.AF_INET and not item.address.startswith('127.'):
					urls.append(f'http://{item.address}:{self._port}')
		return urls

	def scan(self) -> CommandResult:
		"""(막힘, 최대 SCAN_TIMEOUT) 주변 와이파이. 결과에 'networks' 가 늘 있다."""
		if shutil.which('iw') is None:
			return _scan_error('unavailable', '이 로봇에는 iw 가 없어 와이파이를 검색할 수 없습니다')
		if self._switch_lock.locked():   # NetworkManager 가 올리는 연결을 scan 이 흔든다
			return _scan_error('conflict', SWITCHING_MESSAGE)
		interface = _wireless_interface()
		if interface is None:
			return _scan_error('unavailable', '무선 인터페이스를 찾지 못했습니다')
		# AP 가 켜진 채 scan 하면 붙은 기기가 흔들리거나 실패한다 (기획서 U4). 캐시가 아니라 지금 모드를 본다
		if parse_iw_info(_run(['iw', 'dev', interface, 'info']) or '')['type'] == 'AP':
			return _scan_error('unavailable', AP_SCAN_REASON)
		if not self._scan_lock.acquire(blocking=False):
			return _scan_error('conflict', '다른 화면에서 와이파이를 검색하고 있습니다. 잠시 뒤 다시 하세요')
		try:
			completed = subprocess.run(
				['sudo', '-n', 'iw', 'dev', interface, 'scan'], capture_output=True,
				text=True, encoding='utf-8', errors='replace', timeout=SCAN_TIMEOUT,
				check=False)
		except subprocess.TimeoutExpired:
			return _scan_error('failed', f'와이파이 검색이 {SCAN_TIMEOUT:.0f}초 안에 끝나지 않았습니다')
		except OSError as error:
			return _scan_error('failed', f'와이파이 검색을 실행하지 못했습니다: {error.strerror}')
		finally:
			self._scan_lock.release()
		if completed.returncode != 0:
			detail = _last_line(completed.stderr)
			return _scan_error('failed', f'와이파이 검색에 실패했습니다: {detail}')
		networks = parse_iw_scan(completed.stdout)
		return ok_result(f'와이파이 {len(networks)}개를 찾았습니다', networks=networks)

	def connect(self, ssid: str, password: str) -> CommandResult:
		"""run_wifi.sh SSID PASSWORD: 새 공유기를 저장하고 (DHCP) 붙는다. 못 붙으면 스크립트가 AP 로 돌아간다."""
		problem = credentials_problem(ssid, password)
		if problem is not None:
			return error_result('bad_request', problem)
		return self._switch('connect', RUN_WIFI_SCRIPT, (ssid, password), (
			'연결을 시도합니다. 연결이 끊기면 새 와이파이에서 '
			f'http://{socket.gethostname()}.local:{self._port} 으로 다시 접속하세요. 붙지 못하면 '
			f'AP 모드(‘{self._ap_name()}’, http://{AP_HOST}:{self._port})로 돌아옵니다'), secret=password)

	def ap_on(self) -> CommandResult:
		"""run_ap.sh: AP 모드로 (공유기 자동 연결을 꺼서 재부팅해도 AP). 못 올리면 스크립트가 직전 모드로 돌아간다."""
		return self._switch('ap_on', RUN_AP_SCRIPT, (), (
			f'AP 모드로 바꿉니다. 이 화면이 끊기면 폰 · 노트북의 와이파이를 ‘{self._ap_name()}’로 옮긴 뒤 '
			f'http://{AP_HOST}:{self._port} 또는 http://{socket.gethostname()}.local:{self._port} '
			'으로 다시 접속하세요'))

	def ap_off(self) -> CommandResult:
		"""run_wifi.sh (인자 없음): 저장된 공유기로 돌아간다. 못 붙으면 스크립트가 AP 로 돌아간다.

		저장된 공유기가 없으면 (이미지 직후 카드) 스크립트를 부르지 않고 거절한다: 공유기는 connect 로 처음 등록한다.
		"""
		if self.features()['can_ap_mode'] and self._router_ssid is None:
			return error_result('unavailable', NO_ROUTER_REASON)
		return self._switch('ap_off', RUN_WIFI_SCRIPT, (), (
			f'공유기 ‘{self._router_ssid}’로 돌아갑니다. 이 화면이 끊기면 그 공유기 와이파이에서 '
			f'http://{socket.gethostname()}.local:{self._port} 또는 공유기가 준 주소로 다시 접속하세요. '
			'공유기에 붙지 못하면 AP 모드로 돌아옵니다'))

	def ap_config(self, ssid: str, password: str) -> CommandResult:
		"""set_ap.sh SSID PASSWORD: AP 이름 · 비밀번호를 바꾼다 (AP 가 켜져 있으면 다시 켠다). 비밀번호는 쓰기 전용."""
		problem = ap_credentials_problem(ssid, password)
		if problem is not None:
			return error_result('bad_request', problem)
		return self._switch('ap_config', SET_AP_SCRIPT, (ssid, password), (
			f'AP 이름을 ‘{ssid}’로 저장합니다. AP 모드가 켜져 있으면 AP 를 다시 켜서 이 화면이 끊깁니다: '
			f'새 이름 · 비밀번호로 와이파이에 다시 붙은 뒤 http://{AP_HOST}:{self._port} 으로 접속하세요'),
			secret=password)

	def _net_dir(self) -> Path:
		return Path.home() / NET_DIR_NAME

	def _saved_names(self) -> dict[str, str | None]:
		"""keyfile 에 저장된 {'ap_ssid', 'router_ssid'} (sudo -n nm_kf.py get). 개발 서버의 가짜가 바꿔 낀다.

		값이 비었으면 (프로필 · 키 없음) None. 명령이 실패한 이름은 빼서, 부른 쪽이 앞의 값을 두게 한다.
		"""
		names: dict[str, str | None] = {}
		for key, profile in (('ap_ssid', 'lekiwi-ap'), ('router_ssid', 'home-wifi')):
			found = _run(['sudo', '-n', str(self._net_dir() / KEYFILE_TOOL), 'get', profile, 'wifi', 'ssid'])
			if found is not None:
				names[key] = found.removesuffix('\n') or None
		return names

	def _ap_name(self) -> str:
		"""안내에 쓸 AP 이름. keyfile 을 못 읽었으면 호스트 이름 (AP 이름은 호스트 이름과 같게 둔다)."""
		return self.features()['ap_ssid'] or socket.gethostname()

	def _wifi_status(self) -> tuple[dict | None, str | None, dict | None]:
		"""첫 무선 인터페이스의 (wifi, mode, ap) (get_status 참고). iw 나 무선 인터페이스가 없으면 셋 다 None."""
		interface = _wireless_interface()
		if interface is None:
			return None, None, None
		info = parse_iw_info(_run(['iw', 'dev', interface, 'info']) or '')
		if info['type'] == 'AP':
			stations = _run(['iw', 'dev', interface, 'station', 'dump'])
			ap = {'ssid': info['ssid'], 'ip': AP_HOST, 'channel': info['channel'],
				'freq_mhz': info['freq_mhz'],
				'clients': None if stations is None else parse_station_count(stations)}
			return {'interface': interface, **parse_iw_link('')}, 'ap', ap
		link = parse_iw_link(_run(['iw', 'dev', interface, 'link']) or '')
		mode = None
		if info['type'] == 'managed':
			mode = 'client' if link['is_connected'] else 'none'
		return {'interface': interface, **link}, mode, None

	def _switch(
		self, action: str, script: str, args: tuple[str, ...], message: str, secret: str | None = None,
	) -> CommandResult:
		"""~/lekiwi_net/<script> 를 스레드에서 `sudo -n` 으로 돌리고 바로 돌아온다 (전환은 한 번에 하나).

		결과는 서버 로그와 다음 상태 (mode · wifi · ap · last_switch) 로 확인한다. 화면은 전환하면서 끊길 수 있다.

		Args:
			action: last_switch 의 action ('ap_on' · 'ap_off' · 'ap_config' · 'connect').
			script: NET_SCRIPTS 중 하나.
			args: 스크립트 인자. 셸을 거치지 않아 한 글자도 바뀌지 않고 간다.
			message: 시작했을 때 화면에 보일 안내 (다시 접속할 주소, 기획서 U3).
			secret: 로그 · last_switch 에서 *** 로 가릴 비밀번호 (args 에도 로그에 찍지 않는다).
		"""
		features = self.features()
		if not features['can_ap_mode']:
			return error_result('unavailable', features['ap_mode_reason'])
		if not self._switch_lock.acquire(blocking=False):
			return error_result('conflict', SWITCHING_MESSAGE)
		logger.info('네트워크 전환 시작: %s %r', script, [arg for arg in args if arg != secret])
		try:
			# shortcut: 비밀번호가 argv 라 ps 와 sudo 기록(auth.log)에 보인다, 스크립트가 stdin 으로 받게 되면 고친다
			argv = ['sudo', '-n', str(self._net_dir() / script), *args]
			threading.Thread(
				target=self._run_switch, args=(action, argv, secret), name='network-switch', daemon=True,
			).start()
		except Exception:   # 홈 폴더를 모름 · 스레드를 못 띄움: 잠금을 쥔 채 두면 다시는 바꿀 수 없다
			self._switch_lock.release()
			logger.exception('네트워크 전환을 시작하지 못했습니다: %s', script)
			return error_result('failed', '네트워크 전환을 시작하지 못했습니다. 서버 로그를 확인하세요')
		return ok_result(message)

	def _run_switch(self, action: str, argv: list[str], secret: str | None) -> None:
		"""(전환 스레드) 스크립트를 돌리고 결과를 last_switch 에 남긴다. 명령줄 · 예외 문구는 찍지 않는다 (비밀번호).

		종료 코드가 0 이 아니면 실패다 (run_wifi.sh 가 공유기에 못 붙어 AP 로 돌아왔을 때도 종료 1 + error: 줄).
		끝나면 성공 · 실패와 상관없이 저장된 AP · 공유기 이름을 keyfile 에서 다시 읽는다: set_ap.sh 는 저장한 뒤
		AP 를 다시 켜다 실패할 수 있고, run_wifi.sh 는 새 공유기를 저장한 뒤 붙지 못할 수 있다.
		"""
		script = Path(argv[2]).name
		is_ok, message, names = False, '전환하다 서버에서 오류가 났습니다. 서버 로그를 확인하세요', {}
		try:
			try:
				completed = self._run_script(argv)
			except subprocess.TimeoutExpired:   # _run_script 는 끝까지 기다린다. 바꿔 낀 것을 위한 방어
				message = f'{script} 가 {SWITCH_TIMEOUT:.0f}초 안에 끝나지 않았습니다'
				logger.error('%s', message)
			except OSError as error:
				message = f'{script} 를 실행하지 못했습니다: {error.strerror}'
				logger.error('%s', message)
			else:
				output = completed.stdout + completed.stderr
				if secret:
					output = output.replace(secret, '***')
				is_ok = completed.returncode == 0
				message = '' if is_ok else _error_line(output) or f'종료 코드 {completed.returncode}'
				if is_ok:
					logger.info('%s 가 끝났습니다: %s', script, output.strip()[-500:])
				else:
					logger.error('%s 가 실패했습니다 (종료 코드 %d): %s',
						script, completed.returncode, output.strip()[-500:])
			names = self._saved_names()
		finally:
			with self._lock:
				self._features = {**self._features, 'ap_ssid': names.get('ap_ssid', self._features['ap_ssid'])}
				self._router_ssid = names.get('router_ssid', self._router_ssid)
				self._last_switch = {'ok': is_ok, 'action': action, 'message': message, 'at': time.time()}
				self._status = None   # 다음 상태에서 새 모드 · 연결 · 결과를 읽는다
			self._switch_lock.release()

	def _run_script(self, argv: list[str]) -> subprocess.CompletedProcess:
		"""(전환 스레드) 스크립트 한 번 (개발 서버의 가짜 네트워크가 바꿔 낀다).

		SWITCH_TIMEOUT 이 지나도 죽이지 않고 끝까지 기다린다 (잠금도 쥔 채). sudo 만 죽으면 root 스크립트가 남아
		닫힌 파이프에 쓰다 SIGPIPE 로 죽는다: 공유기 실패 뒤 AP 로 돌아가던 중이면 와이파이도 AP 도 없어진다.
		스크립트는 nmcli --wait 로 스스로 끝난다.
		"""
		with subprocess.Popen(
				argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding='utf-8',
				errors='replace') as process:
			try:
				stdout, stderr = process.communicate(timeout=SWITCH_TIMEOUT)
			except subprocess.TimeoutExpired:
				logger.error('%s 가 %.0f초가 지나도 끝나지 않았습니다. 죽이지 않고 끝날 때까지 기다립니다',
					Path(argv[2]).name, SWITCH_TIMEOUT)
				stdout, stderr = process.communicate()
		return subprocess.CompletedProcess(argv, process.returncode, stdout, stderr)
