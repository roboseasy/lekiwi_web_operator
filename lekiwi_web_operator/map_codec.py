"""OccupancyGrid → PNG (회색조). ROS 없이 값만 받는다.

값: -1(모름) → 205, 0(빈 곳) → 254, 100(막힘) → 0, 1..99 → 그 사이 회색 (map_saver 와 같은 방향).
OccupancyGrid 는 0 행이 origin(아래쪽)이므로 위아래를 뒤집어 PNG 첫 줄이 지도 위쪽(+y 끝)이 되게 한다.
"""

import array
import io
from typing import Final

from PIL import Image

UNKNOWN_SHADE: Final[int] = 205
FREE_SHADE: Final[int] = 254
PNG_COMPRESS_LEVEL: Final[int] = 1   # Pi CPU 를 아낀다 (지도는 단순해 크기 차이가 작다)


def _shade_table() -> bytes:
	"""int8 값을 부호 없는 바이트로 본 것(-1 → 255) → 회색 값. 0..100 밖은 모름."""
	table = bytearray([UNKNOWN_SHADE]) * 256
	for value in range(101):
		table[value] = round(FREE_SHADE * (100 - value) / 100)
	return bytes(table)


SHADE_TABLE: Final[bytes] = _shade_table()


def occupancy_grid_to_png(
	width: int,
	height: int,
	data: bytes | bytearray | array.array,
) -> bytes:
	"""OccupancyGrid 칸 값을 PNG 로 바꾼다.

	Args:
		width: 가로 칸 수.
		height: 세로 칸 수.
		data: OccupancyGrid.data (rclpy 의 array('b') 또는 같은 바이트). 행 우선, 0 행이 아래쪽.

	Returns:
		8비트 회색조 PNG. 첫 줄이 +y 끝이다.

	Raises:
		ValueError: 크기가 0 이하이거나 칸 수가 맞지 않을 때.
	"""
	if width <= 0 or height <= 0 or len(data) != width * height:
		raise ValueError(f'occupancy grid size mismatch: {width}x{height}, {len(data)} cells')
	pixels = bytes(data).translate(SHADE_TABLE)
	image = Image.frombytes('L', (width, height), pixels)
	buffer = io.BytesIO()
	image.transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(
		buffer, format='PNG', compress_level=PNG_COMPRESS_LEVEL,
	)
	return buffer.getvalue()
