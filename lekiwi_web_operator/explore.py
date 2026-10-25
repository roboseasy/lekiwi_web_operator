"""자동 주행 맵 생성: 프런티어 탐색 · 경로 따라가기 · 충돌 예측 (ROS 없는 순수 로직, numpy).

robot.RobotService 의 탐색 스레드가 EXPLORE_CONTROL_PERIOD 마다 Snapshot 을 만들어 Explorer.step()
을 부르고, 돌려받은 Step 의 (vx, wz) 를 TeleopSupervisor.set_autonomous 로 넘긴다.
비상 정지 · 권한 · 토크 · 지도 작성 상태 같은 바깥 조건은 robot 이 step() 앞에서 본다.
이 모듈은 센서 값만 보고 갈 곳 · 속도 · 끝을 정한다.

흐름: 새 스캔 · odom · 자세 · 지도를 기다린다 → 제자리에서 EXPLORE_LOOK_ANGLE 넘게 둘러본다
(바퀴 쪽 사각을 한 번씩 보려고) → 새 지도 두 장 → 계획 (지도 0.1 m 칸 ∪ 스캔 점 ∪ 모름을
EXPLORE_PLAN_RADIUS 로 부풀린 격자에서 BFS, 프런티어 덩어리를 보러 갈 칸) → 따라가기
(방향 오차가 크면 제자리 회전, 아니면 전진 + 비례 회전) → 닿으면 프런티어 쪽을 보고 기다린다.
모든 명령은 내보내기 전에 충돌 예측(unsafe_points)을 지난다.
부딪힘 안전은 정면 시야(약 ±42°)의 실시간 스캔이 부딪힐 점을 미리 보는 데 기댄다 (시험 S10).
스캔 기억 · 지도 막힘 칸은 바퀴 사각에 든 점과 스캔에서 잠깐 빠지는 어둡거나 가는 물체를 붙잡아 둔다.
"""

import logging
import math
import time
from collections import deque
from dataclasses import dataclass
from functools import lru_cache
from typing import Final

import numpy as np

from lekiwi_web_operator import config

logger = logging.getLogger(__name__)

Pose = tuple[float, float, float]   # (x, y, yaw) [m, m, rad]

# ── 명세 · 보완에 숫자로 적힌 값 (config 에 두지 않은 것) ──
ARRIVE_DISTANCE: Final[float] = 0.15         # [m] 웨이포인트 · 보러 갈 칸에 이만큼 다가가면 닿았다
VANISH_DISTANCE: Final[float] = 0.3          # [m] 목표 프런티어 점 둘레에 프런티어 칸이 없으면 사라졌다
SWITCH_GAIN: Final[float] = 1.0              # [m] 다른 후보 점수가 이만큼 낮아야 목표를 바꾼다 (오락가락 막기)
SIZE_WEIGHT: Final[float] = 0.5              # 점수 = 경로 길이 [m] − SIZE_WEIGHT × 덩어리 길이 [m]
TURN_START: Final[float] = math.radians(35.0)   # 방향 오차가 이보다 크면 제자리 회전
TURN_STOP: Final[float] = math.radians(15.0)    # 제자리 회전은 이보다 작아지면 끝 (떨림 막기)
TURN_GAIN: Final[float] = 1.2                # wz = TURN_GAIN × 방향 오차
GUARD_STEP: Final[float] = 0.1               # [s] 충돌 예측에서 자세를 보는 간격
DONE_SETTLE: Final[float] = 3.0              # [s] '다 만들었다'의 두 번째 지도는 멈춘 지 이만큼 뒤에 받은 것
LOOK_MAP_SEQS: Final[int] = 2                # 처음 둘러본 뒤 기다리는 새 지도 수
ORIGIN_YAW_TOLERANCE: Final[float] = 1e-3    # [rad] 지도 원점이 이보다 돌아가 있으면 기다린다
PLAN_WARN_TIME: Final[float] = 0.3           # [s] 계획 한 번이 이보다 오래 걸리면 경고
PATH_MAX_POINTS: Final[int] = 200            # Step.path 점 수 상한
NEIGHBOURS: Final[tuple[tuple[int, int], ...]] = tuple(   # 자기 칸과 8-이웃 (dilate 모양)
	(drow, dcol) for drow in (-1, 0, 1) for dcol in (-1, 0, 1)
)

TIGHT_START_REASON: Final[str] = (
	'주변이 좁아 시작할 수 없습니다. 벽 · 가구에서 50 cm 넘게 떨어진 넓은 곳에 두고 다시 시작하세요'
)
TIGHT_REASON: Final[str] = '주변이 좁아 둘러보지 못해 멈췄습니다. 넓은 곳으로 직접 옮긴 뒤 다시 시작하세요'
START_SENSOR_REASON: Final[str] = '센서를 기다리다 시작하지 못했습니다'
SENSOR_REASON: Final[str] = '센서가 끊겨 자동 주행을 멈췄습니다'
STUCK_REASON: Final[str] = '막혀서 자동 주행을 멈췄습니다. 직접 조금 옮긴 뒤 다시 시작하세요'
NARROW_REASON: Final[str] = (
	'갈 수 있는 곳이 좁아 더 탐색하지 못했습니다. 넓은 곳으로 직접 옮긴 뒤 다시 시작하세요'
)
BUMP_REASON: Final[str] = '무언가에 걸린 것 같아 멈췄습니다. 몸체 · 팔 높이의 가구 아래 틈을 확인하세요'
TIME_LIMIT_REASON: Final[str] = (
	f'{config.EXPLORE_MAX_DURATION / 60:.0f}분이 지나 탐색을 끝냈습니다'
)
DONE_REASON: Final[str] = '지도를 다 만들었습니다 (갈 수 없는 곳 {unreachable}곳 · 건너뛴 곳 {skipped}곳)'
SCAN_WAIT: Final[str] = '라이다 스캔을 기다리는 중입니다'
ODOM_WAIT: Final[str] = 'odom 을 기다리는 중입니다'
POSE_WAIT: Final[str] = '지도 위 자세를 기다리는 중입니다'
MAP_WAIT: Final[str] = '지도를 기다리는 중입니다'
ORIGIN_WAIT: Final[str] = '지도 원점이 돌아가 있어 기다립니다'


@dataclass(frozen=True)
class GridSnapshot:
	"""Cartographer /map 한 장 (nav_msgs/OccupancyGrid).

	Attributes:
		seq: 받은 지도마다 1 씩 는다.
		resolution: 칸 크기 [m] (Cartographer 0.05).
		origin_x: data[0, 0] 칸 모서리의 map x [m].
		origin_y: data[0, 0] 칸 모서리의 map y [m].
		origin_yaw: 원점 회전 [rad] (0 이 아니면 탐색하지 않는다).
		data: int8 (height, width), data[row, col] = OccupancyGrid.data[row * width + col]
			(row 가 y, col 이 x). msg.data 를 복사하지 않고 본 배열이라 고쳐 쓰지 않는다.
		received_at: 받은 time.monotonic().
	"""

	seq: int
	resolution: float
	origin_x: float
	origin_y: float
	origin_yaw: float
	data: np.ndarray
	received_at: float


@dataclass(frozen=True)
class Snapshot:
	"""탐색 스레드 한 주기의 입력. 시간은 모두 time.monotonic(), 없는 값은 None.

	Attributes:
		now: 이 주기의 시각.
		scan_seq: 마지막 스캔 번호 (새 스캔마다 는다).
		scan_points: 그 스캔의 점 float (N, 2), base_footprint [m]. NaN · inf · range 밖은 뺐다.
		scan_received_at: 그 스캔을 받은 시각.
		scan_odom: 그 스캔을 받을 때 저장한 odom 자세.
		scan_odom_lag: 스캔을 받은 시각 − 그 odom 을 받은 시각 [s].
		odom: 지금 odom 자세 (odom → base_footprint).
		odom_received_at: 그 odom 을 받은 시각.
		map_pose: 지금 base_footprint 의 map 자세 (tf2_echo).
		map_pose_received_at: 그 자세를 받은 시각.
		grid: 마지막 지도.
		sent: TeleopSupervisor 가 지난 주기에 실제로 낸 (vx, wz) (램프 · 상한 뒤, 걸림 판단용).
	"""

	now: float
	scan_seq: int | None
	scan_points: np.ndarray | None
	scan_received_at: float | None
	scan_odom: Pose | None
	scan_odom_lag: float | None
	odom: Pose | None
	odom_received_at: float | None
	map_pose: Pose | None
	map_pose_received_at: float | None
	grid: GridSnapshot | None
	sent: tuple[float, float]


@dataclass(frozen=True)
class Step:
	"""Explorer.step() 의 결과.

	Attributes:
		vx: 전진 목표 [m/s], 0 ≤ vx ≤ EXPLORE_SPEED (옆 · 뒤로는 가지 않는다).
		wz: 회전 목표 [rad/s], |wz| ≤ EXPLORE_TURN_SPEED.
		state: 'starting' | 'exploring' | 'waiting' | 'done' | 'stopped'
			('done' 은 다 만들었거나 시간이 다 됨, 'stopped' 은 그 밖의 끝).
		reason: 한국어 문구. 'waiting' 의 이유와 끝난 이유. 그 밖에는 None.
		is_finished: True 면 이 탐색은 끝났다 (vx = wz = 0). 다시 부르지 않는다.
		should_save: 끝났고 지도를 자동 저장할 끝 ('done': 다 만듦 · 시간 다 됨) 이면 True.
		target: 지금 가는 곳 (map 프레임 x, y). 없으면 None.
		path: 남은 경로 [x0, y0, x1, y1, ...] (map 프레임, 최대 200 점). 없으면 None.
		frontiers: 닿을 수 있고 블랙리스트가 아닌 프런티어 덩어리 수. 모르면 None.
		explored_area: 알려진 빈 칸 (0..EXPLORE_FREE_MAX) 넓이 [m²]. 모르면 None.
	"""

	vx: float
	wz: float
	state: str
	reason: str | None
	is_finished: bool
	should_save: bool
	target: tuple[float, float] | None
	path: list[float] | None
	frontiers: int | None
	explored_area: float | None


# ── 좌표 ─────────────────────────────────────────────────
def wrap_angle(angle: float) -> float:
	"""-π..π 로 줄인다."""
	return (angle + math.pi) % (2.0 * math.pi) - math.pi


def to_parent(points: np.ndarray, pose: Pose) -> np.ndarray:
	"""pose 프레임의 점 (N, 2) 를 그 부모 프레임으로 (예: base_footprint → odom)."""
	x, y, yaw = pose
	c, s = math.cos(yaw), math.sin(yaw)
	return points @ np.array([[c, s], [-s, c]]) + (x, y)


def to_local(points: np.ndarray, pose: Pose) -> np.ndarray:
	"""부모 프레임의 점 (N, 2) 를 pose 프레임으로 (예: odom → 지금 base_footprint)."""
	x, y, yaw = pose
	c, s = math.cos(yaw), math.sin(yaw)
	return (points - (x, y)) @ np.array([[c, -s], [s, c]])


def _is_pose(pose: Pose | None) -> bool:
	"""세 값이 모두 유한한 자세인지."""
	return pose is not None and all(math.isfinite(value) for value in pose)


def _is_fresh(now: float, received_at: float | None, max_age: float) -> bool:
	"""received_at 이 있고 now 에서 max_age 안인지."""
	return received_at is not None and now - received_at <= max_age


def _clamp(value: float, limit: float) -> float:
	"""-limit..limit 로 자른다."""
	return max(-limit, min(limit, value))


# ── 충돌 예측 (S1) ───────────────────────────────────────
def footprint_grow(points: np.ndarray) -> np.ndarray:
	"""점마다 몸 모양을 얼마나 키워야 그 점이 안에 드는지 [m] (음수: 이미 실제 몸 안).

	몸 모양 = 원(EXPLORE_BODY_RADIUS) ∪ 팔 상자(x 0..EXPLORE_ARM_REACH, |y| ≤ EXPLORE_ARM_HALF_WIDTH).
	g 만큼 키운 모양은 원(BODY + g) ∪ 상자(x 0..REACH + g, |y| ≤ HALF + g) 다 (상자 모서리는 각지다).

	Args:
		points: base_footprint 점 (..., 2).
	"""
	x, y = points[..., 0], points[..., 1]
	circle = np.hypot(x, y) - config.EXPLORE_BODY_RADIUS
	box = np.maximum(x - config.EXPLORE_ARM_REACH, np.abs(y) - config.EXPLORE_ARM_HALF_WIDTH)
	return np.minimum(circle, np.where(x >= 0.0, box, np.inf))


def arc_pose(vx: float, wz: float, t: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
	"""(vx, wz) 로 t 초 움직인 자세 (x, y, yaw), 지금 몸 기준 (t 는 배열)."""
	if abs(wz) < 1e-9:
		return vx * t, np.zeros_like(t), np.zeros_like(t)
	radius = vx / wz
	return radius * np.sin(wz * t), radius * (1.0 - np.cos(wz * t)), wz * t


def unsafe_points(points: np.ndarray, vx: float, wz: float) -> np.ndarray:
	"""S1: 명령 (vx, wz) 를 EXPLORE_HORIZON 동안 따라가면 부딪힐 수 있는 점 (bool (N,)).

	GUARD_STEP 마다의 호 위 자세에서 몸 모양을 EXPLORE_MARGIN 만큼 키운 모양 안에 드는 점이다.
	지금 이미 그 안에 든 점은 몸과의 거리(footprint_grow)가 줄어드는 명령만 막는다
	(조금 밀린 뒤 모든 명령이 영원히 막히지 않게). 제자리 회전(vx = 0)은 팔 상자가 쓸고 가는
	원 EXPLORE_ROTATION_RADIUS 안의 점이 어느 쪽에 있어도 막는다. 0 명령은 늘 안전하다.

	Args:
		points: 지금 base_footprint 점 (N, 2).
		vx: 전진 [m/s].
		wz: 회전 [rad/s].
	"""
	if (vx == 0.0 and wz == 0.0) or not len(points):
		return np.zeros(len(points), bool)
	times = np.arange(1, round(config.EXPLORE_HORIZON / GUARD_STEP) + 1) * GUARD_STEP
	px, py, yaw = arc_pose(vx, wz, times)
	c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
	dx, dy = points[:, 0] - px[:, None], points[:, 1] - py[:, None]
	grow = footprint_grow(np.stack((c * dx + s * dy, c * dy - s * dx), axis=-1))   # (T, N)
	start = footprint_grow(points)
	is_inside = start <= config.EXPLORE_MARGIN
	unsafe = np.where(is_inside, grow < start - 1e-9, grow <= config.EXPLORE_MARGIN).any(axis=0)
	if vx == 0.0:
		unsafe |= np.hypot(points[:, 0], points[:, 1]) < config.EXPLORE_ROTATION_RADIUS
	return unsafe


# ── 지도 → 계획 격자 (M3 · M4 · M5) ──────────────────────
def downsample(data: np.ndarray, factor: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
	"""M3: /map 을 factor × factor 칸으로 줄인다 (홀수 너비 · 높이는 모름으로 채운다).

	칸 하나: 아래 칸 중 하나라도 EXPLORE_OCCUPIED_MIN 이상이면 막힘, 아니면 절반 이상이
	모름(음수)이면 모름, 아니면 빈 칸(지나갈 수 있다).

	Returns:
		(occupied, unknown, clear). clear 는 빈 칸 중 아는 아래 칸이 모두 0..EXPLORE_FREE_MAX 인 칸.
		50..64 ('애매', 한두 번 닿음) 가 섞인 빈 칸은 지나갈 수 있지만 프런티어가 되지 않는다.
	"""
	height, width = data.shape
	rows, cols = -(-height // factor), -(-width // factor)
	padded = np.full((rows * factor, cols * factor), -1, np.int8)
	padded[:height, :width] = data
	blocks = padded.reshape(rows, factor, cols, factor).swapaxes(1, 2).reshape(rows, cols, -1)
	occupied = (blocks >= config.EXPLORE_OCCUPIED_MIN).any(axis=2)
	unknown = ~occupied & (2 * (blocks < 0).sum(axis=2) >= factor * factor)
	clear = ~unknown & ~(blocks > config.EXPLORE_FREE_MAX).any(axis=2)
	return occupied, unknown, clear


def frontier_mask(unknown: np.ndarray, clear: np.ndarray) -> np.ndarray:
	"""M5: 4-이웃에 모르는 칸이 있는 clear 칸 (격자 밖은 덧댄 모름 칸이 맡는다)."""
	near = np.zeros_like(unknown)
	near[1:] |= unknown[:-1]
	near[:-1] |= unknown[1:]
	near[:, 1:] |= unknown[:, :-1]
	near[:, :-1] |= unknown[:, 1:]
	return clear & near


@lru_cache(maxsize=8)
def disk_offsets(radius: float, cell: float) -> tuple[tuple[int, int], ...]:
	"""칸 중심 거리가 radius 이하인 (drow, dcol) 들 (0.45 m · 0.1 m 칸 → 반지름 4 칸)."""
	reach = int(radius / cell + 1e-9)
	return tuple(
		(drow, dcol)
		for drow in range(-reach, reach + 1)
		for dcol in range(-reach, reach + 1)
		if math.hypot(drow, dcol) * cell <= radius + 1e-9
	)


def dilate(mask: np.ndarray, offsets: tuple[tuple[int, int], ...]) -> np.ndarray:
	"""mask 의 True 칸을 offsets 모양만큼 넓힌다 (격자 밖은 False 로 본다)."""
	reach = max(max(abs(drow), abs(dcol)) for drow, dcol in offsets)
	padded = np.pad(mask, reach)
	height, width = mask.shape
	out = np.zeros_like(mask)
	for drow, dcol in offsets:
		out |= padded[reach + drow:reach + drow + height, reach + dcol:reach + dcol + width]
	return out


@dataclass(frozen=True)
class PlanMap:
	"""지도 한 장(seq)에서 만든 계획용 칸. 둘레에 모름 칸을 덧대 두었다 (격자 밖 = 모름).

	Attributes:
		seq: 지도 번호.
		cell: 칸 크기 [m] (지도 칸 × 정수배, Cartographer 0.05 → 0.1).
		origin_x: blocked[0, 0] 칸 모서리의 map x [m].
		origin_y: blocked[0, 0] 칸 모서리의 map y [m].
		blocked: 막힘 ∪ 모름 (bool, row 가 y).
		frontier: 프런티어 칸 (bool).
		explored_area: 빈 칸 (0..EXPLORE_FREE_MAX) 넓이 [m²] (원래 지도 칸으로 센다).
	"""

	seq: int
	cell: float
	origin_x: float
	origin_y: float
	blocked: np.ndarray
	frontier: np.ndarray
	explored_area: float

	def cell_of(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
		"""map 점 (N, 2) → (row, col, 격자 안인지)."""
		cols = np.floor((points[:, 0] - self.origin_x) / self.cell).astype(int)
		rows = np.floor((points[:, 1] - self.origin_y) / self.cell).astype(int)
		height, width = self.blocked.shape
		return rows, cols, (rows >= 0) & (rows < height) & (cols >= 0) & (cols < width)

	def centres(self, cells: np.ndarray) -> np.ndarray:
		"""칸 (N, 2) [row, col] → 칸 중심 map 점 (N, 2)."""
		cells = np.asarray(cells).reshape(-1, 2)
		return np.column_stack((
			self.origin_x + (cells[:, 1] + 0.5) * self.cell,
			self.origin_y + (cells[:, 0] + 0.5) * self.cell,
		))

	def has_frontier_near(self, point: tuple[float, float], radius: float) -> bool:
		"""point 에서 radius 안에 프런티어 칸 중심이 있는지."""
		reach = math.ceil(radius / self.cell) + 1
		(row,), (col,), _ = self.cell_of(np.array([point]))
		row0, col0 = max(0, row - reach), max(0, col - reach)
		row1, col1 = max(row0, row + reach + 1), max(col0, col + reach + 1)
		rows, cols = np.nonzero(self.frontier[row0:row1, col0:col1])
		centres = self.centres(np.column_stack((rows + row0, cols + col0)))
		return bool((np.hypot(*(centres - point).T) <= radius).any())


def build_plan_map(grid: GridSnapshot) -> PlanMap:
	"""M3 · M5: 지도 한 장을 계획용 칸으로 (막힘 ∪ 모름 · 프런티어 · 알아낸 넓이)."""
	factor = max(1, round(config.EXPLORE_CELL / grid.resolution))
	cell = factor * grid.resolution
	occupied, unknown, clear = downsample(grid.data, factor)
	pad = math.ceil(config.EXPLORE_PLAN_RADIUS / cell) + 1   # 부풀린 모름이 지도 가장자리까지 덮게
	unknown = np.pad(unknown, pad, constant_values=True)
	occupied, clear = np.pad(occupied, pad), np.pad(clear, pad)
	free = (grid.data >= 0) & (grid.data <= config.EXPLORE_FREE_MAX)
	return PlanMap(
		seq=grid.seq,
		cell=cell,
		origin_x=grid.origin_x - pad * cell,
		origin_y=grid.origin_y - pad * cell,
		blocked=occupied | unknown,
		frontier=frontier_mask(unknown, clear),
		explored_area=float(np.count_nonzero(free)) * grid.resolution ** 2,
	)


def passable_grid(
	plan: PlanMap, points: np.ndarray, x: float, y: float,
) -> tuple[np.ndarray, np.ndarray]:
	"""M4: 계획 격자 = 지도 막힘 ∪ 모름 ∪ 점이 든 칸을 EXPLORE_PLAN_RADIUS 까지 막은 것.

	EXPLORE_PLAN_RADIUS 는 칸 중심 사이 거리다 (제자리 회전 원 + 한 칸, 약 0.95 m 보다 좁은 틈은
	지나지 않는다). 지도가 막은 칸 바로 옆(8-이웃) 칸의 점은 넣지 않는다: 아는 벽 · 모름 경계를 한 칸
	더 두껍게 할 뿐인데, 면이 칸 경계에 걸리면 잡음 · 자세 시차로 계획마다 칸이 바뀌어 약 1 m 틈이
	열렸다 닫혔다 한다 (그 점은 충돌 예측이 그대로 본다). 로봇 몸체 원(EXPLORE_BODY_RADIUS) 아래 칸은
	비워 로봇 칸이 막히지 않게 하고, 가장자리 한 줄은 늘 막는다 (bfs 가 경계를 보지 않는다).

	Args:
		plan: build_plan_map 결과.
		points: 막을 점 (map 프레임 (N, 2), 실시간 스캔 · 기억 점).
		x: 로봇 map x [m].
		y: 로봇 map y [m].

	Returns:
		(지나갈 수 있는 칸, 몸체 칸) bool 격자.
	"""
	blocked = plan.blocked.copy()
	rows, cols, is_inside = plan.cell_of(points)
	is_inside[is_inside] = ~dilate(plan.blocked, NEIGHBOURS)[rows[is_inside], cols[is_inside]]
	blocked[rows[is_inside], cols[is_inside]] = True
	passable = ~dilate(blocked, disk_offsets(config.EXPLORE_PLAN_RADIUS, plan.cell))
	height, width = blocked.shape
	xs = plan.origin_x + (np.arange(width) + 0.5) * plan.cell
	ys = plan.origin_y + (np.arange(height) + 0.5) * plan.cell
	body = (xs[None, :] - x) ** 2 + (ys[:, None] - y) ** 2 <= config.EXPLORE_BODY_RADIUS ** 2
	passable |= body
	passable[[0, -1], :] = False
	passable[:, [0, -1]] = False
	return passable, body


def bfs(passable: np.ndarray, start: tuple[int, int]) -> np.ndarray:
	"""8-이웃 BFS: 칸마다 start 에서 몇 걸음인지 (닿지 않으면 -1).

	passable 가장자리 한 줄은 False 이고 start 는 그 안쪽이어야 한다 (경계 검사를 빼서 빠르게).
	"""
	height, width = passable.shape
	is_open = passable.ravel().tolist()
	dist = [-1] * (height * width)
	origin = int(start[0]) * width + int(start[1])
	dist[origin] = 0
	is_open[origin] = False
	queue = [origin]
	offsets = (-width - 1, -width, -width + 1, -1, 1, width - 1, width, width + 1)
	for cell in queue:   # 도는 동안 queue 에 붙인 칸도 차례로 꺼낸다
		step = dist[cell] + 1
		for offset in offsets:
			neighbour = cell + offset
			if is_open[neighbour]:
				is_open[neighbour] = False
				dist[neighbour] = step
				queue.append(neighbour)
	return np.array(dist, np.int32).reshape(height, width)


def trace_path(dist: np.ndarray, goal: tuple[int, int]) -> list[tuple[int, int]]:
	"""BFS 거리를 거슬러 start → goal 칸 목록 (곧은 이웃을 먼저 고른다)."""
	width = dist.shape[1]
	flat = dist.ravel().tolist()
	cell = goal[0] * width + goal[1]
	path = [cell]
	offsets = (-width, width, -1, 1, -width - 1, -width + 1, width - 1, width + 1)
	while flat[cell] > 0:
		previous = flat[cell] - 1
		cell = next(cell + offset for offset in offsets if flat[cell + offset] == previous)
		path.append(cell)
	path.reverse()
	return [divmod(cell, width) for cell in path]


def supercover(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
	"""두 칸 중심을 잇는 선분이 지나는 칸 모두 (S14, 모서리를 지나면 양옆 칸도)."""
	(row, col), (end_row, end_col) = start, end
	rows, cols = abs(end_row - row), abs(end_col - col)
	row_step, col_step = (1 if end_row > row else -1), (1 if end_col > col else -1)
	cells = [(row, col)]
	row_count = col_count = 0
	while row_count < rows or col_count < cols:
		# 다음에 넘는 칸 경계: 가로줄이 먼저면 음수, 동시(모서리)면 0
		decision = (1 + 2 * row_count) * cols - (1 + 2 * col_count) * rows
		if decision == 0:
			cells += [(row + row_step, col), (row, col + col_step)]
			row, col = row + row_step, col + col_step
			row_count, col_count = row_count + 1, col_count + 1
		elif decision < 0:
			row, row_count = row + row_step, row_count + 1
		else:
			col, col_count = col + col_step, col_count + 1
		cells.append((row, col))
	return cells


def simplify(path: list[tuple[int, int]], passable: np.ndarray) -> list[list[tuple[int, int]]]:
	"""S14: BFS 칸 경로를 직선 구간으로 줄인다 (구간마다 지나는 칸 목록, 끝 칸이 웨이포인트).

	방향이 바뀌는 칸만 후보로 두고, 앞 웨이포인트에서 supercover 선이 막히지 않는 가장 먼 후보까지
	잇는다. 이웃 후보 사이(같은 방향의 BFS 칸 줄)는 그대로 쓴다.
	"""
	if len(path) < 2:
		return []
	steps = [(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
	corners = [0] + [i for i in range(1, len(steps)) if steps[i] != steps[i - 1]] + [len(path) - 1]
	segments = []
	anchor = 0
	while anchor < len(corners) - 1:
		end = anchor + 1
		cells = path[corners[anchor]:corners[end] + 1]
		while end + 1 < len(corners):
			line = supercover(path[corners[anchor]], path[corners[end + 1]])
			rows, cols = zip(*line)
			if not passable[rows, cols].all():
				break
			end, cells = end + 1, line
		segments.append(cells)
		anchor = end
	return segments


def frontier_clusters(mask: np.ndarray) -> list[np.ndarray]:
	"""8-이웃으로 이어진 프런티어 칸 덩어리들, 덩어리마다 (n, 2) [row, col]."""
	height, width = mask.shape
	remaining = set(np.flatnonzero(mask).tolist())
	groups = []
	while remaining:
		stack = [remaining.pop()]
		members = list(stack)
		while stack:
			row, col = divmod(stack.pop(), width)
			for drow in (-1, 0, 1):
				for dcol in (-1, 0, 1):
					neighbour = (row + drow) * width + col + dcol
					if 0 <= col + dcol < width and neighbour in remaining:
						remaining.remove(neighbour)
						stack.append(neighbour)
						members.append(neighbour)
		groups.append(np.column_stack(np.divmod(np.array(members), width)))
	return groups


def view_cell(
	dist: np.ndarray, cells: np.ndarray, offsets: tuple[tuple[int, int], ...],
) -> tuple[int, int] | None:
	"""M5: 덩어리 칸(cells (n, 2))에서 offsets 안에 있고 닿는 칸 중 BFS 거리가 가장 짧은 칸."""
	reach = max(max(abs(drow), abs(dcol)) for drow, dcol in offsets)
	row0, col0 = np.maximum(cells.min(axis=0) - reach, 0)
	row1, col1 = cells.max(axis=0) + reach + 1
	window = dist[row0:row1, col0:col1]
	mask = np.zeros(window.shape, bool)
	mask[cells[:, 0] - row0, cells[:, 1] - col0] = True
	near = dilate(mask, offsets) & (window >= 0)
	if not near.any():
		return None
	row, col = divmod(int(np.where(near, window, np.iinfo(np.int32).max).argmin()), window.shape[1])
	return int(row0 + row), int(col0 + col)


@dataclass(frozen=True)
class Candidate:
	"""보러 갈 프런티어 덩어리 하나 (M5 · M7).

	Attributes:
		view: 보러 갈 칸 (row, col), 닿는 칸 중 덩어리에서 EXPLORE_VIEW_DISTANCE 안.
		point: 그 칸에서 가장 가까운 프런티어 칸 중심 (map) — 바라볼 곳 · 블랙리스트.
		cells: 덩어리에서 블랙리스트 밖 칸 중심 (k, 2) map — 하던 목표 찾기.
		score: 경로 길이 [m] − SIZE_WEIGHT × 덩어리 길이 [m] (낮을수록 좋다).
	"""

	view: tuple[int, int]
	point: tuple[float, float]
	cells: np.ndarray
	score: float


def find_candidates(
	plan: PlanMap, dist: np.ndarray | None, blacklist: list[tuple[float, float]],
) -> tuple[list[Candidate], int, int]:
	"""M5 · M7: EXPLORE_MIN_FRONTIER 넘는 프런티어 덩어리마다 보러 갈 칸을 찾는다.

	EXPLORE_BLACKLIST_RADIUS 안의 실패한 점 둘레 칸은 덩어리에서 뺀다.

	Args:
		plan: build_plan_map 결과.
		dist: passable_grid 위 BFS 거리. 시작조차 못 하면 None (모두 닿지 않음).
		blacklist: 실패한 프런티어 점 (map).

	Returns:
		(후보, 닿지 않는 덩어리 수, 블랙리스트로 남은 칸이 모자란 덩어리 수).
	"""
	min_cells = math.ceil(config.EXPLORE_MIN_FRONTIER / plan.cell - 1e-9)
	offsets = disk_offsets(config.EXPLORE_VIEW_DISTANCE, plan.cell)
	failed = np.array(blacklist, float).reshape(-1, 2)
	candidates, unreachable, skipped = [], 0, 0
	for cells in frontier_clusters(plan.frontier):
		if len(cells) < min_cells:
			continue
		centres = plan.centres(cells)
		if len(failed):
			gaps = np.hypot(*(centres[:, None, :] - failed[None, :, :]).transpose(2, 0, 1))
			is_open = gaps.min(axis=1) > config.EXPLORE_BLACKLIST_RADIUS
			cells, centres = cells[is_open], centres[is_open]
			if len(cells) < min_cells:
				skipped += 1
				continue
		view = None if dist is None else view_cell(dist, cells, offsets)
		if view is None:
			unreachable += 1
			continue
		view_centre = plan.centres(np.array([view]))[0]
		point = centres[np.hypot(*(centres - view_centre).T).argmin()]
		score = float(dist[view]) * plan.cell - SIZE_WEIGHT * len(cells) * plan.cell
		candidates.append(Candidate(view, (float(point[0]), float(point[1])), centres, score))
	return candidates, unreachable, skipped


def choose(
	candidates: list[Candidate], committed: tuple[float, float] | None,
) -> tuple[Candidate, bool]:
	"""M7: 점수가 가장 낮은 후보. 하던 목표가 아직 있으면 다른 후보가 SWITCH_GAIN 넘게 낮을 때만 바꾼다.

	Args:
		candidates: find_candidates 후보 (하나 이상).
		committed: 하던 목표의 프런티어 점 (map). 없으면 None.

	Returns:
		(고른 후보, 하던 목표(committed 에서 VANISH_DISTANCE 안의 덩어리)인지).
	"""
	best = min(candidates, key=lambda candidate: candidate.score)
	if committed is None:
		return best, False
	same = [
		candidate for candidate in candidates
		if np.hypot(*(candidate.cells - committed).T).min() <= VANISH_DISTANCE
	]
	if not same:
		return best, False
	current = min(same, key=lambda candidate: candidate.score)
	if best.score <= current.score - SWITCH_GAIN:
		return best, False
	return current, True


def scan_to_odom(points: np.ndarray, scan_odom: Pose) -> np.ndarray:
	"""S2: 스캔 점(그때의 base_footprint) 중 EXPLORE_LIVE_RANGE 안의 유한한 점을 odom 프레임으로."""
	points = np.asarray(points, float).reshape(-1, 2)
	is_kept = np.isfinite(points).all(axis=1)
	is_kept[is_kept] = np.hypot(*points[is_kept].T) <= config.EXPLORE_LIVE_RANGE
	return to_parent(points[is_kept], scan_odom)


def occupied_points(grid: GridSnapshot, x: float, y: float, radius: float) -> np.ndarray:
	"""지도 막힘 칸 (EXPLORE_OCCUPIED_MIN 이상) 중 (x, y) 둘레 radius 창 안의 칸 중심 (map, (N, 2))."""
	resolution = grid.resolution
	col, row = (x - grid.origin_x) / resolution, (y - grid.origin_y) / resolution
	reach = radius / resolution + 1.0
	row0, col0 = max(0, math.floor(row - reach)), max(0, math.floor(col - reach))
	row1, col1 = max(row0, math.floor(row + reach) + 1), max(col0, math.floor(col + reach) + 1)
	rows, cols = np.nonzero(grid.data[row0:row1, col0:col1] >= config.EXPLORE_OCCUPIED_MIN)
	return np.column_stack((
		grid.origin_x + (cols + col0 + 0.5) * resolution,
		grid.origin_y + (rows + row0 + 0.5) * resolution,
	))


@dataclass
class _Target:
	"""정한 목표 (M7). mode: 'go' (경로 따라가기) → 'face' (프런티어 쪽으로 돌기) → 'dwell'."""

	view: tuple[float, float]     # 보러 갈 칸 중심 (map)
	point: tuple[float, float]    # 바라볼 프런티어 점 (map)
	segments: list[np.ndarray]    # 남은 구간마다 지나는 칸 중심 (k, 2) map, 끝 행이 웨이포인트
	mode: str = 'go'
	since: float = 0.0            # 'dwell' 을 시작한 시각


class Explorer:
	"""자동 주행 한 번. 탐색 스레드 하나에서만 부른다 (잠금 없음)."""

	def __init__(self, started_at: float) -> None:
		"""시작 시각을 기억한다.

		Args:
			started_at: 시작 time.monotonic() (EXPLORE_MAX_DURATION 기준).
		"""
		self._started_at = started_at
		# 'start' → 'look' → 'settle' → 'explore' (오래 기다린 뒤에는 'look' 다시)
		self._phase = 'start'
		self._is_started = False        # 첫 계획을 마쳤는지 (그 뒤 상태는 'exploring')
		self._finished: Step | None = None
		self._waiting_since: float | None = None
		# S2 스캔 기억: (받은 시각, odom 프레임 점)
		self._memory: deque[tuple[float, np.ndarray]] = deque()
		self._scan_seq: int | None = None
		self._is_scan_placed = False
		# M8 걸림: (시각, 그 사이 낸 전진 [m], 회전 [rad], map 자세)
		self._motion: deque[tuple[float, float, float, Pose]] = deque()
		self._last_tick: float | None = None
		self._last_moved_at = -math.inf
		# M6 둘러보기
		self._look_direction = 1.0
		self._is_look_reversed = False   # 이번 둘러보기에서 막혀 한 번 돌아섰는지
		self._look_yaw: float | None = None
		self._look_turn = (0.0, 0.0, 0.0)   # (지금까지 돈 각, 그 최소, 최대) [rad]
		self._settle_seq = 0
		# 계획 · 목표
		self._plan_map: PlanMap | None = None
		self._target: _Target | None = None
		self._is_rotating = False
		self._should_replan = False
		self._last_plan_at = -math.inf
		self._last_check = (-math.inf, -1)   # S13 남은 경로를 다시 본 (시각, 지도 seq)
		self._blacklist: list[tuple[float, float]] = []
		self._failures = 0
		self._blocked_since: float | None = None
		self._progress: tuple[float, float, float] | None = None   # (x, y, 시각)
		self._empty_seq: int | None = None
		self._can_start = False
		self._frontiers: int | None = None

	def step(self, snapshot: Snapshot) -> Step:
		"""한 주기: 이번에 보낼 (vx, wz) 와 상태.

		Args:
			snapshot: 이 주기의 센서 값.

		Returns:
			Step. is_finished 가 True 면 탐색이 끝났다.
		"""
		s = snapshot
		if self._finished is not None:
			return self._finished
		if s.now - self._started_at >= config.EXPLORE_MAX_DURATION:
			return self._finish('done', TIME_LIMIT_REASON, should_save=True)
		reason = self._stale_reason(s)
		if reason is not None:
			return self._wait(s, reason)
		self._resume(s)
		plan_map = self._map(s.grid)
		if self._is_bumped(s):
			return self._finish('stopped', BUMP_REASON)
		points = self._guard_points(s)
		state = 'exploring' if self._is_started else 'starting'
		if self._phase == 'start':
			self._begin_look()
		if self._phase == 'look':
			command = self._look(s, points)
			if command is None:
				reason = TIGHT_REASON if self._is_started else TIGHT_START_REASON
				return self._finish('stopped', reason)
			return self._output(s, *command, state)
		if self._phase == 'settle':
			if s.grid.seq < self._settle_seq + LOOK_MAP_SEQS:
				return self._output(s, 0.0, 0.0, state)
			is_tight = bool(
				(np.hypot(points[:, 0], points[:, 1]) < config.EXPLORE_ROTATION_RADIUS).any()
			)
			finished = None if is_tight else self._plan(s, plan_map)
			if is_tight or not self._can_start:
				return self._finish('stopped', TIGHT_START_REASON)
			if finished is not None:
				return finished
			self._phase, self._is_started = 'explore', True
		return self._explore(s, plan_map, points)

	# ── 한 주기의 단계 ─────────────────────────────────────
	def _explore(self, s: Snapshot, plan_map: PlanMap, points: np.ndarray) -> Step:
		"""계획 (S13: 목표가 없거나 · 막혔거나 · 경로가 막혔을 때만) → 따라가기 → 충돌 예측."""
		target = self._target
		if target is not None and target.mode == 'go' and (
			s.now - self._last_check[0] >= config.EXPLORE_PLAN_PERIOD
			or plan_map.seq != self._last_check[1]
		):
			self._check_path(s, plan_map)
		if (self._target is None or self._should_replan) and (
			s.now - self._last_plan_at >= config.EXPLORE_PLAN_PERIOD
		):
			finished = self._plan(s, plan_map)
			if finished is not None:
				return finished
		vx, wz = (0.0, 0.0) if self._target is None else self._follow(s, plan_map)
		if (vx or wz) and unsafe_points(points, vx, wz).any():
			vx = wz = 0.0
			self._should_replan = True   # 새 스캔 점을 넣어 같은 목표로 경로를 다시 찾는다
			if self._blocked_since is None:
				self._blocked_since = s.now
		elif vx or wz:
			self._blocked_since = None
		if self._target is not None and self._is_failing(s):
			vx = wz = 0.0
			finished = self._fail()
			if finished is not None:
				return finished
		return self._output(s, vx, wz, 'exploring')

	def _stale_reason(self, s: Snapshot) -> str | None:
		"""신선도 (스캔 · odom · 지도 위 자세 · 지도) 와 S2 기억. 쓸 수 없으면 'waiting' 이유."""
		now = s.now
		if s.scan_points is None or not _is_fresh(
			now, s.scan_received_at, config.EXPLORE_SCAN_MAX_AGE,
		):
			return SCAN_WAIT
		if not _is_pose(s.odom) or not _is_fresh(
			now, s.odom_received_at, config.EXPLORE_ODOM_MAX_AGE,
		):
			return ODOM_WAIT
		self._remember(s)
		if not self._is_scan_placed:
			return ODOM_WAIT
		if not _is_pose(s.map_pose) or not _is_fresh(
			now, s.map_pose_received_at, config.EXPLORE_POSE_MAX_AGE,
		):
			return POSE_WAIT
		grid = s.grid
		if grid is None or not (grid.resolution > 0.0 and grid.data.ndim == 2 and grid.data.size):
			return MAP_WAIT
		if not _is_fresh(now, grid.received_at, config.EXPLORE_MAP_MAX_AGE):
			return MAP_WAIT
		if not abs(grid.origin_yaw) <= ORIGIN_YAW_TOLERANCE:
			return ORIGIN_WAIT
		return None

	def _remember(self, s: Snapshot) -> None:
		"""S2: 새 스캔을 odom 프레임으로 기억하고 EXPLORE_SCAN_MEMORY 보다 오래된 것은 버린다.

		스캔과 함께 저장한 odom 이 그 스캔보다 EXPLORE_ODOM_MAX_LAG 넘게 먼저면 점을 놓을 수 없어
		기억을 모두 버린다 (다음 좋은 스캔까지 'waiting').
		"""
		if s.scan_seq != self._scan_seq:
			self._scan_seq = s.scan_seq
			lag = s.scan_odom_lag
			self._is_scan_placed = (
				_is_pose(s.scan_odom) and lag is not None and lag <= config.EXPLORE_ODOM_MAX_LAG
			)
			if self._is_scan_placed:
				self._memory.append((s.scan_received_at, scan_to_odom(s.scan_points, s.scan_odom)))
			else:
				self._memory.clear()
		while self._memory and s.now - self._memory[0][0] > config.EXPLORE_SCAN_MEMORY:
			self._memory.popleft()

	def _wait(self, s: Snapshot, reason: str) -> Step:
		"""데이터를 쓸 수 없다: 0 을 내고 기다린다. 너무 오래면 끝낸다.

		M8 걸림 창은 비우지 않는다: 기다리는 동안 낸 명령은 0 이고 오래된 칸은 창에서 저절로 빠진다
		(비우면 잠깐씩 끊길 때마다 처음부터 세어, 걸린 채 앞으로 미는 것을 끝내 알아채지 못한다).
		"""
		if self._phase == 'start':
			if s.now - self._started_at > config.EXPLORE_SENSOR_TIMEOUT:
				return self._finish('stopped', START_SENSOR_REASON)
			return self._output(s, 0.0, 0.0, 'starting')
		if self._waiting_since is None:
			self._waiting_since = s.now
		if s.now - self._waiting_since > config.EXPLORE_SENSOR_TIMEOUT:
			return self._finish('stopped', SENSOR_REASON)
		return self._output(s, 0.0, 0.0, 'waiting', reason)

	def _resume(self, s: Snapshot) -> None:
		"""기다림이 끝났다. EXPLORE_SCAN_MEMORY 넘게 기다렸으면 앞으로 가기 전에 다시 둘러본다 (M6).

		막힘 · 진척 없음 타이머는 기다린 만큼 늦춘다: 기다린 시간은 세지 않되 그 전에 센 시간은 남긴다
		(지우면 잠깐씩 끊길 때마다 처음부터 세어 막힌 목표가 끝내 실패하지 않는다).
		"""
		if self._waiting_since is None:
			return
		waited = s.now - self._waiting_since
		if waited > config.EXPLORE_SCAN_MEMORY and self._phase != 'start':
			self._begin_look()
		self._waiting_since = None
		if self._blocked_since is not None:
			self._blocked_since += waited
		if self._progress is not None:
			x, y, since = self._progress
			self._progress = (x, y, since + waited)

	def _is_bumped(self, s: Snapshot) -> bool:
		"""M8: 지난 EXPLORE_BUMP_WINDOW 동안 낸 명령만큼 지도 위 자세가 움직이지 않았는지."""
		dt = 0.0 if self._last_tick is None else s.now - self._last_tick
		self._last_tick = s.now
		vx, wz = s.sent
		if abs(vx) > 1e-6 or abs(wz) > 1e-6:
			self._last_moved_at = s.now
		self._motion.append((s.now, abs(vx) * dt, abs(wz) * dt, s.map_pose))
		while s.now - self._motion[0][0] > config.EXPLORE_BUMP_WINDOW:
			self._motion.popleft()
		window = list(self._motion)
		forward = sum(entry[1] for entry in window[1:])
		turn = sum(entry[2] for entry in window[1:])
		moved = math.dist(window[0][3][:2], s.map_pose[:2])
		yawed = sum(abs(wrap_angle(b[3][2] - a[3][2])) for a, b in zip(window, window[1:]))
		return (
			(forward >= config.EXPLORE_BUMP_DISTANCE and moved < config.EXPLORE_BUMP_MOVED)
			or (turn >= config.EXPLORE_BUMP_TURN and yawed < config.EXPLORE_BUMP_YAWED)
		)

	def _begin_look(self) -> None:
		"""M6 둘러보기를 (다시) 시작한다."""
		self._phase = 'look'
		self._is_look_reversed = False
		self._look_yaw = None
		self._look_turn = (0.0, 0.0, 0.0)

	def _look(self, s: Snapshot, points: np.ndarray) -> tuple[float, float] | None:
		"""M6: 충돌 예측이 허락하는 쪽으로 제자리에서 돈다 (가 본 방향 폭이 EXPLORE_LOOK_ANGLE 넘을 때까지).

		돈 각은 지도 위 자세로 잰다 (돌지 못하면 M8 걸림이 먼저 끝낸다). 막히면 한 번만 돌아선다:
		두 끝이 다 막힌 틈은 오가도 넓어지지 않는다 (회전 원 바로 밖의 가는 기둥을 팔 상자 모서리가 쓸 때).

		Returns:
			이번 명령. 돌아선 뒤 다시 막혔거나 어느 쪽으로도 돌 수 없으면 None.
		"""
		yaw = s.map_pose[2]
		if self._look_yaw is not None:
			turned, low, high = self._look_turn
			turned += wrap_angle(yaw - self._look_yaw)
			self._look_turn = (turned, min(low, turned), max(high, turned))
		self._look_yaw = yaw
		if self._look_turn[2] - self._look_turn[1] >= config.EXPLORE_LOOK_ANGLE:
			self._phase = 'explore' if self._is_started else 'settle'
			self._settle_seq, self._should_replan = s.grid.seq, True
			return 0.0, 0.0
		wz = self._look_direction * config.EXPLORE_TURN_SPEED
		if not unsafe_points(points, 0.0, wz).any():
			return 0.0, wz
		if self._is_look_reversed or unsafe_points(points, 0.0, -wz).any():
			return None
		self._is_look_reversed, self._look_direction = True, -self._look_direction
		return 0.0, -wz

	def _plan(self, s: Snapshot, plan_map: PlanMap) -> Step | None:
		"""전체 계획: BFS · 후보 · 목표 (M7 다짐) · 경로. 끝낼 때만 Step."""
		began = time.perf_counter()
		self._last_plan_at, self._should_replan = s.now, False
		self._last_check = (s.now, plan_map.seq)   # 새 계획은 새 격자로 본 것
		x, y = s.map_pose[0], s.map_pose[1]
		passable, body = passable_grid(plan_map, self._plan_points(s), x, y)
		(row,), (col,), _ = plan_map.cell_of(np.array([[x, y]]))
		height, width = passable.shape
		is_inside = 0 < row < height - 1 and 0 < col < width - 1
		dist = bfs(passable, (row, col)) if is_inside else None
		self._can_start = dist is not None and bool(((dist >= 0) & ~body).any())
		candidates, unreachable, skipped = find_candidates(plan_map, dist, self._blacklist)
		self._frontiers = len(candidates)
		finished = None
		if candidates:
			self._empty_seq = None
			committed = None if self._target is None else self._target.point
			choice, is_same = choose(candidates, committed)
			if not is_same:
				self._drop_target()
			segments = simplify(trace_path(dist, choice.view), passable)
			view_x, view_y = plan_map.centres(np.array([choice.view]))[0]
			self._target = _Target(
				view=(float(view_x), float(view_y)),
				point=choice.point,
				segments=[plan_map.centres(np.array(segment)) for segment in segments],
			)
		else:
			self._drop_target()
			area = 0.0 if dist is None else np.count_nonzero(dist >= 0) * plan_map.cell ** 2
			is_point_blocked = dist is not None and bool(find_candidates(
				plan_map, bfs(passable_grid(plan_map, np.empty((0, 2)), x, y)[0], (row, col)),
				self._blacklist,
			)[0])
			finished = self._on_empty(s, area, unreachable, skipped, is_point_blocked)
		elapsed = time.perf_counter() - began
		if elapsed > PLAN_WARN_TIME:
			logger.warning('탐색 계획이 %.2f s 걸렸습니다 (계획 칸 %d × %d)', elapsed, height, width)
		return finished

	def _on_empty(
		self, s: Snapshot, area: float, unreachable: int, skipped: int, is_point_blocked: bool,
	) -> Step | None:
		"""M7: 갈 프런티어가 없는 계획. 서로 다른 지도 두 장에서 연달아 없어야 끝낸다.

		두 번째 지도는 로봇이 멈춘 지 DONE_SETTLE 넘게 지나 받은 것이어야 한다. 그때 BFS 가
		EXPLORE_MIN_FREE_AREA 넘게 닿았으면 'done' (저장), 시작조차 못 하거나 좁으면 'stopped'.
		is_point_blocked (지도만으로는 갈 프런티어가 있는데 지도에 없는 스캔 점이 막았다: 문에 선 사람 등) 면
		'done' 으로 세지 않고 기다린다. 비키면 이어 가고, 그대로 서 있으면 지도에 들어 끝난다.
		"""
		grid = s.grid
		is_settled = grid.received_at >= self._last_moved_at + DONE_SETTLE
		if self._empty_seq is None:
			self._empty_seq = grid.seq
			return None
		if grid.seq == self._empty_seq or not is_settled:
			return None
		if not self._can_start or area < config.EXPLORE_MIN_FREE_AREA:
			return self._finish('stopped', NARROW_REASON)
		if is_point_blocked:
			self._empty_seq = None
			return None
		reason = DONE_REASON.format(unreachable=unreachable, skipped=skipped)
		return self._finish('done', reason, should_save=True)

	def _check_path(self, s: Snapshot, plan_map: PlanMap) -> None:
		"""S13: 가는 동안 새 격자로 남은 경로 칸만 다시 본다. 목표 프런티어가 사라졌으면 다음 목표 (M7 b)."""
		self._last_check = (s.now, plan_map.seq)
		target = self._target
		if not plan_map.has_frontier_near(target.point, VANISH_DISTANCE):
			self._drop_target()
			return
		passable, _ = passable_grid(plan_map, self._plan_points(s), s.map_pose[0], s.map_pose[1])
		rows, cols, is_inside = plan_map.cell_of(np.concatenate(target.segments))
		if not (is_inside.all() and passable[rows, cols].all()):
			self._should_replan = True   # M7: 같은 목표로 경로만 다시

	def _follow(self, s: Snapshot, plan_map: PlanMap) -> tuple[float, float]:
		"""S14 따라가기 · M7 (a) 닿으면 프런티어 쪽으로 돌아 EXPLORE_DWELL 기다린다."""
		target = self._target
		x, y, yaw = s.map_pose
		if target.mode == 'go':
			if math.dist((x, y), target.view) <= ARRIVE_DISTANCE:   # 한 칸 경로면 이미 그 칸 안
				target.mode, target.segments = 'face', []
			else:
				while len(target.segments) > 1 and (
					math.dist((x, y), target.segments[0][-1]) <= ARRIVE_DISTANCE
				):
					target.segments.pop(0)
				aim_x, aim_y = target.segments[0][-1]
				error = wrap_angle(math.atan2(aim_y - y, aim_x - x) - yaw)
				if abs(error) > TURN_START:
					self._is_rotating = True
				elif abs(error) < TURN_STOP:
					self._is_rotating = False
				vx = 0.0 if self._is_rotating else config.EXPLORE_SPEED * max(0.0, math.cos(error))
				return vx, _clamp(TURN_GAIN * error, config.EXPLORE_TURN_SPEED)
		if target.mode == 'face':
			error = wrap_angle(math.atan2(target.point[1] - y, target.point[0] - x) - yaw)
			if abs(error) >= TURN_STOP:
				return 0.0, _clamp(TURN_GAIN * error, config.EXPLORE_TURN_SPEED)
			target.mode, target.since = 'dwell', s.now
		if s.now - target.since >= config.EXPLORE_DWELL:
			if plan_map.has_frontier_near(target.point, config.EXPLORE_BLACKLIST_RADIUS):
				self._blacklist.append(target.point)   # 보고도 못 지웠다 (검은 · 유리 · 먼 곳)
			self._failures = 0
			self._drop_target()
		return 0.0, 0.0

	def _is_failing(self, s: Snapshot) -> bool:
		"""M7 (c): 충돌 예측에 EXPLORE_BLOCKED_TIMEOUT 넘게 막혔거나, 가는 동안
		EXPLORE_PROGRESS_TIMEOUT 사이 EXPLORE_PROGRESS_DISTANCE 도 못 갔는지."""
		if self._blocked_since is not None and (
			s.now - self._blocked_since > config.EXPLORE_BLOCKED_TIMEOUT
		):
			return True
		if self._target.mode != 'go':
			self._progress = None
			return False
		x, y = s.map_pose[0], s.map_pose[1]
		progress = self._progress
		if progress is None or math.dist((x, y), progress[:2]) >= config.EXPLORE_PROGRESS_DISTANCE:
			self._progress = (x, y, s.now)
			return False
		return s.now - progress[2] > config.EXPLORE_PROGRESS_TIMEOUT

	def _fail(self) -> Step | None:
		"""목표 실패: 블랙리스트에 넣고 다음 목표. 연달아 EXPLORE_MAX_FAILURES 번이면 끝낸다."""
		self._blacklist.append(self._target.point)
		self._drop_target()
		self._failures += 1
		if self._failures >= config.EXPLORE_MAX_FAILURES:
			return self._finish('stopped', STUCK_REASON)
		return None

	def _drop_target(self) -> None:
		"""목표와 그 목표에 묶인 타이머를 버린다."""
		self._target = None
		self._blocked_since = self._progress = None

	# ── 점 · 지도 ──────────────────────────────────────────
	def _map(self, grid: GridSnapshot) -> PlanMap:
		"""지도 seq 마다 한 번만 계획용 칸을 만든다."""
		if self._plan_map is None or self._plan_map.seq != grid.seq:
			self._plan_map = build_plan_map(grid)
		return self._plan_map

	def _memory_points(self) -> np.ndarray:
		"""기억한 스캔 점 모두 (odom 프레임 (N, 2), 지금 스캔 포함)."""
		if not self._memory:
			return np.empty((0, 2))
		return np.concatenate([points for _, points in self._memory])

	def _plan_points(self, s: Snapshot) -> np.ndarray:
		"""계획 격자에 넣는 스캔 · 기억 점 (map 프레임): odom → 지금 몸 → map."""
		return to_parent(to_local(self._memory_points(), s.odom), s.map_pose)

	def _guard_points(self, s: Snapshot) -> np.ndarray:
		"""S2: 충돌 예측 점 (지금 base_footprint, EXPLORE_GUARD_RANGE 안).

		스캔 · 기억 점은 odom 으로, 지도 막힘 칸은 지도 위 자세로 지금 몸에 옮긴다.
		"""
		x, y = s.map_pose[0], s.map_pose[1]
		mapped = occupied_points(s.grid, x, y, config.EXPLORE_GUARD_RANGE)
		points = np.concatenate((
			to_local(self._memory_points(), s.odom), to_local(mapped, s.map_pose),
		))
		return points[np.hypot(points[:, 0], points[:, 1]) <= config.EXPLORE_GUARD_RANGE]

	# ── 결과 ───────────────────────────────────────────────
	def _output(
		self, s: Snapshot, vx: float, wz: float, state: str, reason: str | None = None,
	) -> Step:
		"""끝나지 않은 Step (목표 · 남은 경로 · 프런티어 수 · 알아낸 넓이)."""
		target = path = None
		if self._target is not None:
			target = (round(self._target.view[0], 3), round(self._target.view[1], 3))
			points = [segment[-1] for segment in self._target.segments]
			if points and _is_pose(s.map_pose):
				points.insert(0, s.map_pose[:2])
			if len(points) > PATH_MAX_POINTS:
				points = points[:PATH_MAX_POINTS - 1] + points[-1:]
			path = [round(float(value), 3) for point in points for value in point] or None
		return Step(
			float(vx), float(wz), state, reason, False, False, target, path,
			self._frontiers, self._explored_area(),
		)

	def _finish(self, state: str, reason: str, should_save: bool = False) -> Step:
		"""끝난 Step 을 만들어 두고 그 뒤로도 같은 것을 돌려준다."""
		self._target = None
		self._finished = Step(
			0.0, 0.0, state, reason, True, should_save, None, None,
			self._frontiers, self._explored_area(),
		)
		return self._finished

	def _explored_area(self) -> float | None:
		"""마지막 지도에서 알아낸 빈 넓이 [m²]."""
		return None if self._plan_map is None else round(self._plan_map.explored_area, 2)
