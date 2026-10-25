"""로봇 컴퓨터 상태: CPU · 온도 · 메모리 · 디스크 · 전원 경고 (SPEC §6 system · §10)."""

import os
import shutil
import socket
import subprocess
import threading
import time
from pathlib import Path
from typing import Final

import psutil

from lekiwi_web_operator import config

THERMAL_ZONE_PATH: Final[Path] = Path('/sys/class/thermal/thermal_zone0/temp')
VCGENCMD_TIMEOUT: Final[float] = 1.0       # [s]
# vcgencmd get_throttled 비트 (Raspberry Pi 문서)
UNDERVOLTAGE_NOW_BIT: Final[int] = 0x1
THROTTLED_NOW_BIT: Final[int] = 0x4
UNDERVOLTAGE_OCCURRED_BIT: Final[int] = 0x10000


def parse_throttled(output: str) -> dict | None:
	"""`vcgencmd get_throttled` 출력 ('throttled=0x50005') 을 해석한다.

	Returns:
		{'raw', 'is_undervoltage_now', 'has_undervoltage_occurred', 'is_throttled_now'},
		모양이 다르면 None.
	"""
	name, _, raw = output.strip().partition('=')
	if name != 'throttled':
		return None
	try:
		bits = int(raw, 16)
	except ValueError:
		return None
	return {
		'raw': raw,
		'is_undervoltage_now': bool(bits & UNDERVOLTAGE_NOW_BIT),
		'has_undervoltage_occurred': bool(bits & UNDERVOLTAGE_OCCURRED_BIT),
		'is_throttled_now': bool(bits & THROTTLED_NOW_BIT),
	}


def _cpu_temperature() -> float | None:
	"""[℃] thermal_zone0, 없으면 psutil 온도 센서의 첫 값."""
	try:
		return round(int(THERMAL_ZONE_PATH.read_text()) / 1000.0, 1)
	except (OSError, ValueError):
		pass
	for entries in psutil.sensors_temperatures().values():
		if entries:
			return round(entries[0].current, 1)
	return None


def _throttled() -> dict | None:
	try:
		completed = subprocess.run(
			['vcgencmd', 'get_throttled'], capture_output=True, text=True,
			timeout=VCGENCMD_TIMEOUT, check=False)
	except (OSError, subprocess.TimeoutExpired):
		return None
	if completed.returncode != 0:
		return None
	return parse_throttled(completed.stdout)


class SystemInfo:
	"""status.system 값. SYSTEM_INFO_PERIOD 동안 같은 dict 를 돌려준다 (바꾸지 말 것)."""

	def __init__(self) -> None:
		self._lock = threading.Lock()
		self._cache: dict | None = None
		self._cache_time = 0.0
		self._has_vcgencmd = shutil.which('vcgencmd') is not None
		psutil.cpu_percent(None)  # 첫 호출은 기준점만 잡고 0.0 을 돌려준다

	def get(self) -> dict:
		"""§6 system 값 (캐시).

		Returns:
			hostname, cpu_percent, cpu_temp [℃], load, memory_percent, memory_used ·
			memory_total [B], disk_percent, uptime [s], throttled (vcgencmd 가 없으면 None).
		"""
		with self._lock:
			now = time.monotonic()
			if self._cache is None or now - self._cache_time >= config.SYSTEM_INFO_PERIOD:
				self._cache = self._read()
				self._cache_time = now
			return self._cache

	def _read(self) -> dict:
		memory = psutil.virtual_memory()
		return {
			'hostname': socket.gethostname(),
			'cpu_percent': psutil.cpu_percent(None),
			'cpu_temp': _cpu_temperature(),
			'load': [round(value, 2) for value in os.getloadavg()],
			'memory_percent': memory.percent,
			# percent 와 같은 기준 (psutil 의 used 는 total - free - 버퍼 - 캐시라 percent 와 맞지 않는다)
			'memory_used': memory.total - memory.available,
			'memory_total': memory.total,
			'disk_percent': psutil.disk_usage('/').percent,
			'uptime': round(time.time() - psutil.boot_time(), 1),
			'throttled': _throttled() if self._has_vcgencmd else None,
		}
