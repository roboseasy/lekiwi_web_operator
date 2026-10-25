"""저장된 지도 (save_map.sh 결과) 의 목록 · 이름 검사 · 미리보기 (SPEC §10)."""

import io
import logging
from pathlib import Path
from typing import Any, Final

import yaml
from PIL import Image

logger = logging.getLogger(__name__)

LAST_MAP_FILE: Final[str] = 'last_map.txt'   # save_map.sh 가 마지막 yaml 이름을 적는다
PREVIEW_MAX_SIZE: Final[int] = 1024          # [px] 미리보기 긴 변
MAX_NAME_BYTES: Final[int] = 255             # 파일 이름 길이 제한 (ext4)
# yaml · 이미지가 깨졌을 때 나는 오류 (UnidentifiedImageError 는 OSError)
_READ_ERRORS: Final[tuple[type[Exception], ...]] = (
	OSError, ValueError, KeyError, TypeError, yaml.YAMLError,
	Image.DecompressionBombError,
)


def is_valid_map_name(name: Any) -> bool:
	"""경로 구분자 · `..` · 숨김 · 제어 문자가 없는 `<이름>.yaml` 인지."""
	return (
		isinstance(name, str)
		and name.isprintable()
		and name.endswith('.yaml')
		and len(name) > len('.yaml')
		and not name.startswith('.')
		and '/' not in name
		and '\\' not in name
		and '..' not in name
		and len(name.encode('utf-8')) <= MAX_NAME_BYTES
	)


def resolve_map(map_dir: Path, name: str) -> Path | None:
	"""지도 이름 → 절대 경로.

	Args:
		map_dir: 지도 폴더 (config.map_dir()).
		name: 브라우저가 보낸 yaml 파일 이름.

	Returns:
		map_dir 바로 아래의 실제 파일이면 그 절대 경로, 아니면 None.
	"""
	if not is_valid_map_name(name):
		return None
	folder = map_dir.resolve()
	path = (folder / name).resolve()  # 폴더 밖을 가리키는 링크도 여기서 걸러진다
	if path.parent != folder or not path.is_file():
		return None
	return path


def list_maps(map_dir: Path) -> list[dict]:
	"""지도 폴더의 `*.yaml` (숨김 제외) 을 최근 수정 순으로. 못 읽는 것은 경고 후 뺀다.

	Returns:
		[{'name', 'modified', 'width', 'height', 'resolution', 'origin': [x, y, yaw],
		'has_pbstream', 'is_last'}]
	"""
	if not map_dir.is_dir():
		return []
	last_name = _last_map_name(map_dir)
	maps = []
	for candidate in map_dir.glob('*.yaml'):
		path = resolve_map(map_dir, candidate.name)
		if path is None:
			continue
		try:
			data = _load_yaml(path)
			with Image.open(_image_path(path, data)) as image:
				width, height = image.size
			origin = [float(value) for value in data['origin']]
			if len(origin) != 3:
				raise ValueError('origin 은 [x, y, yaw] 여야 합니다')
			maps.append({
				'name': candidate.name,
				'modified': path.stat().st_mtime,
				'width': width,
				'height': height,
				'resolution': float(data['resolution']),
				'origin': origin,
				'has_pbstream': path.with_suffix('.pbstream').is_file(),
				'is_last': candidate.name == last_name,
			})
		except _READ_ERRORS as error:
			logger.warning('지도 %s 를 읽지 못해 목록에서 뺍니다: %s', candidate.name, error)
	maps.sort(key=lambda item: item['modified'], reverse=True)
	return maps


def render_preview(map_dir: Path, name: str) -> bytes | None:
	"""지도 이미지를 PNG 로 (긴 변 PREVIEW_MAX_SIZE 이하). 없거나 못 읽으면 None."""
	path = resolve_map(map_dir, name)
	if path is None:
		return None
	try:
		with Image.open(_image_path(path, _load_yaml(path))) as image:
			preview = image.convert('L')
		preview.thumbnail((PREVIEW_MAX_SIZE, PREVIEW_MAX_SIZE))
		buffer = io.BytesIO()
		preview.save(buffer, format='PNG')
	except _READ_ERRORS as error:
		logger.warning('지도 %s 미리보기를 만들지 못했습니다: %s', name, error)
		return None
	return buffer.getvalue()


def _load_yaml(path: Path) -> dict:
	data = yaml.safe_load(path.read_text(encoding='utf-8'))
	if not isinstance(data, dict):
		raise ValueError('지도 yaml 이 아닙니다')
	return data


def _image_path(yaml_path: Path, data: dict) -> Path:
	"""yaml 의 image. 상대 경로면 yaml 폴더 기준 (map_server 와 같다)."""
	image = Path(data['image'])
	return image if image.is_absolute() else yaml_path.parent / image


def _last_map_name(map_dir: Path) -> str | None:
	try:
		text = (map_dir / LAST_MAP_FILE).read_text(encoding='utf-8')
	except (OSError, ValueError):
		return None
	return Path(text.strip()).name or None
