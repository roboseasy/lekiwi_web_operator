"""지도 위 로봇 자세 (map → base_footprint) 를 tf2_echo 자식 프로세스로 읽는다.

rclpy 로 /tf 를 받으면 메시지마다 Pi 4 코어 약 2 ms 를 쓰는데, Cartographer 는 /tf 를 약 250 Hz
(map → odom 200 Hz + odom → base_footprint 50 Hz) 로 낸다. 그래서 C++ tf2_echo 가 RATE 로 찍는
출력만 읽는다. ROS 없이도 불러올 수 있다 (명령줄을 만들 때만 ament_index_python 이 필요하다).
"""

import logging
import math
import os
import signal
import subprocess
import threading
import time
from typing import Final, Iterable, Iterator

from lekiwi_web_operator import config

logger = logging.getLogger(__name__)

RATE: Final[int] = 10                     # [Hz] tf2_echo 가 자세를 찍는 빈도
FRESH_AGE: Final[float] = 1.0             # [s] 이 안에 새 자세가 왔으면 위치를 안다 (is_localized)
# [s] 곧 끝나기를 되풀이하면 처음 값부터 두 배씩, 최대 값 간격으로 다시 띄운다.
# 최대 값보다 오래 돌다 끝났으면 바로 다시 띄운다
RESTART_DELAYS: Final[tuple[float, float]] = (1.0, 30.0)
STOP_TIMEOUT: Final[float] = 2.0          # [s] SIGINT 뒤 SIGKILL 까지 (보통 0.1 s 안에 끝난다)
TIME_PREFIX: Final[str] = 'At time '
TRANSLATION_PREFIX: Final[str] = '- Translation: ['
YAW_PREFIX: Final[str] = '- Rotation: in RPY (radian) ['


def command(node_name: str) -> list[str]:
	"""map → base_footprint 를 RATE 로 찍는 tf2_echo 명령줄 (ROS 를 source 한 환경에서).

	Jazzy tf2_echo 는 빈도를 -r 로만 받는다 (뒤에 숫자만 두면 'Unknown argument' 로 끝난다).
	--log-level warn 은 변환을 기다리는 동안 1 s 마다 찍는 INFO 를 끈다.

	Args:
		node_name: 그래프에 보일 노드 이름.

	Raises:
		ament_index_python.packages.PackageNotFoundError: tf2_ros 가 없을 때.
	"""
	from ament_index_python.packages import get_package_prefix   # ROS 없이 이 모듈을 불러오게

	executable = os.path.join(get_package_prefix('tf2_ros'), 'lib', 'tf2_ros', 'tf2_echo')
	return [
		executable, config.MAP_FRAME, config.BASE_FRAME, '-r', str(RATE),
		'--ros-args', '-r', f'__node:={node_name}', '--log-level', 'warn',
	]


def _bracket_values(text: str) -> list[float]:
	"""'... [a, b, c]' 의 숫자들."""
	return [float(value) for value in text[text.index('[') + 1:text.index(']')].split(',')]


def parse_tf_echo(lines: Iterable[str]) -> Iterator[tuple[float, dict]]:
	"""tf2_echo 출력 줄 → echo 마다 (At time [s], {'x', 'y', 'yaw'}).

	At time 의 나노초는 0 을 채우지 않고 찍힐 때가 있어 (1792409889.98369768 은 …889.098369768)
	float 로 읽지 않는다. 쿼터니언 · 도 · 행렬 줄과 읽지 못한 줄은 건너뛴다.

	Args:
		lines: stdout 줄 (끝의 줄바꿈은 있어도 된다).

	Yields:
		(변환 시각, 자세). 시각은 마지막 공통 변환의 것이라 발행이 멈추면 같은 값이 되풀이된다.
	"""
	stamp: float | None = None
	translation: list[float] | None = None
	for line in lines:
		text = line.strip()
		try:
			if text.startswith(TIME_PREFIX):
				seconds, _, nanoseconds = text[len(TIME_PREFIX):].partition('.')
				stamp, translation = int(seconds) + int(nanoseconds or '0') * 1e-9, None
			elif text.startswith(TRANSLATION_PREFIX):
				translation = _bracket_values(text)[:2]
			elif text.startswith(YAW_PREFIX) and stamp is not None and translation is not None:
				x, y = translation
				yaw = _bracket_values(text)[2]
				yield stamp, {'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4)}
				stamp = translation = None
		except (ValueError, IndexError):   # 잘린 줄: 이 echo 는 버린다
			stamp = translation = None


def _stop(process: subprocess.Popen) -> None:
	"""SIGINT (Ctrl+C 처럼), STOP_TIMEOUT 안에 안 끝나면 SIGKILL. 끝날 때까지 기다린다."""
	if process.poll() is None:
		process.send_signal(signal.SIGINT)
	try:
		process.wait(STOP_TIMEOUT)
	except subprocess.TimeoutExpired:
		process.kill()
		process.wait()


class TfEcho:
	"""tf2_echo 자식 프로세스 하나: 원하는 동안 띄워 두고, 끝나면 점점 늦게 다시 띄운다.

	start · stop · watch 는 한 스레드(ROS 스레드)에서 부르고 pose 는 아무 스레드에서 부른다.
	자식은 새 세션으로 띄워 터미널의 Ctrl+C 를 받지 않는다 (게이트웨이가 끈다).

	Args:
		argv: 실행할 명령줄 (보통 command()).
	"""

	def __init__(self, argv: list[str]) -> None:
		self._argv = argv
		self._lock = threading.Lock()
		self._process: subprocess.Popen | None = None   # 쓰기는 self._lock 안 (읽기 스레드가 본다)
		self._pose: tuple[float, dict] | None = None    # self._lock: (받은 time.monotonic(), 자세)
		# start · stop · watch 를 부르는 스레드만 쓴다
		self._is_wanted = False
		self._is_closed = False     # close 뒤에는 다시 띄우지 않는다 (게이트웨이 종료 중)
		self._started_at = -math.inf
		self._failures = 0          # 곧 끝난 횟수 (연달아)
		self._retry_at = -math.inf

	def start(self) -> None:
		"""원한다: 아직 없으면 바로 띄운다."""
		if not self._is_wanted and not self._is_closed:
			self._is_wanted, self._failures, self._retry_at = True, 0, -math.inf
			self.watch()

	def stop(self, is_waiting: bool = False) -> None:
		"""원하지 않는다: 끄고 마지막 자세를 지운다.

		Args:
			is_waiting: True 면 끝날 때까지 기다린다 (종료). 아니면 기다리는 일은 다른 스레드가 한다.
		"""
		self._is_wanted = False
		with self._lock:
			process, self._process, self._pose = self._process, None, None
		if process is None:
			return
		if is_waiting:
			_stop(process)
			return
		threading.Thread(target=_stop, args=(process,), name='tf-echo-stop', daemon=True).start()

	def close(self, is_waiting: bool = False) -> None:
		"""끄고 다시는 띄우지 않는다. systemd 가 tf2_echo 에도 SIGINT 를 보내 끝나도, 종료하는
		동안 watch 가 새로 띄우지 않게 한다.

		Args:
			is_waiting: stop 과 같다.
		"""
		self._is_closed = True
		self.stop(is_waiting)

	def watch(self) -> None:
		"""(주기적으로) 원하는데 없거나 끝났으면 띄운다 (RESTART_DELAYS 간격)."""
		if not self._is_wanted:
			return
		now = time.monotonic()
		process = self._process
		if process is not None:
			if process.poll() is None:
				return
			with self._lock:
				self._process = None
			delay = self._schedule_retry(now)
			logger.warning(
				'tf2_echo 가 끝났습니다 (종료 코드 %s). %g초 뒤 다시 띄웁니다', process.returncode, delay,
			)
		if now < self._retry_at:
			return
		self._started_at = now
		try:
			process = subprocess.Popen(
				self._argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, text=True,
				encoding='utf-8', errors='replace', start_new_session=True,
			)
		except OSError as error:
			logger.error('tf2_echo 를 띄우지 못했습니다 (%g초 뒤 다시): %s', self._schedule_retry(now), error)
			return
		with self._lock:
			self._process = process
		threading.Thread(target=self._read, args=(process,), name='tf-echo', daemon=True).start()

	def pose(self) -> dict | None:
		"""(아무 스레드) FRESH_AGE 안에 온 마지막 자세 {'x', 'y', 'yaw'} (map 프레임). 없으면 None."""
		entry = self.pose_with_age()
		return None if entry is None or entry[0] > FRESH_AGE else entry[1]

	def pose_with_age(self) -> tuple[float, dict] | None:
		"""(아무 스레드) (나이 [s], 마지막 자세). FRESH_AGE 를 보지 않는다 (자동 주행은 더 짧게 본다).

		나이는 새 자세(시각이 바뀐 echo)를 받은 뒤 지난 시간이다. 끈 뒤 · 아직 없으면 None.
		"""
		with self._lock:
			pose = self._pose
		return None if pose is None else (time.monotonic() - pose[0], pose[1])

	def _schedule_retry(self, now: float) -> float:
		"""다시 띄울 시각을 정하고 기다릴 시간 [s] 을 돌려준다."""
		first, longest = RESTART_DELAYS
		# 몇 시간씩 되풀이돼도 2 ** n 이 float 범위를 넘지 않게 센다 (2 ** 15 s 면 이미 최대 값이다)
		self._failures = min(self._failures + 1, 16) if now - self._started_at < longest else 0
		delay = min(first * 2 ** (self._failures - 1), longest) if self._failures else 0.0
		self._retry_at = now + delay
		return delay

	def _read(self, process: subprocess.Popen) -> None:
		"""(읽기 스레드) 출력 → 자세. 시각이 바뀐 것만 새 자세로 본다: 발행이 멈추면 tf2_echo 는
		마지막 변환을 같은 시각으로 계속 찍는다."""
		last_stamp = None
		try:
			with process.stdout:   # 끝나면 파이프를 닫는다
				for stamp, pose in parse_tf_echo(process.stdout):
					if stamp == last_stamp:
						continue
					last_stamp = stamp
					with self._lock:
						if self._process is not process:   # 그사이 껐다
							return
						self._pose = (time.monotonic(), pose)
		except (OSError, ValueError):
			logger.exception('tf2_echo 출력을 읽지 못했습니다')
