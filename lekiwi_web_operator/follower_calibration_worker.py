"""팔로워 (LeKiwi) 새로 보정 프로그램: 게이트웨이가 lerobot 이 든 python 으로 따로 켠다 (docs/calibration_plan.md).

게이트웨이의 다른 모듈은 가져오지 않는다 (python 이 다르다). 보정 단계는 옆의 lerobot_calibration.py
(PC 에이전트 저장소 lekiwi_pc_agent/calibration.py 의 사본 = 리더와 같은 코드, lerobot 0.6.0) 가 한다. 로봇 기능 (bringup) 을 끄고 팔 토크가 꺼진 뒤에만 켠다 (버스 주인이 없게).

표준 입력 (게이트웨이 → 이 프로그램), JSON 한 줄씩:
	{"cmd": "start", "port": "/dev/ttyACM0", "calibration_path": ".../lekiwi01.json",
	 "restore_path": ".../arm_calibration_restore.json", "restore_only": false}   처음 한 번
	{"cmd": "middle"} · {"cmd": "finish"} · {"cmd": "cancel"}
표준 출력 (이 프로그램 → 게이트웨이), JSON 한 줄씩:
	{"state": waiting_middle|recording|saving|done|cancelled|error, "joints": {모터: {now, min, max}} | null,
	 "reason": 한국어 | null, "backup": 남긴 백업 경로 | null, "ack": 이 줄이 답하는 명령 | null}
	명령마다 ack 가 그 명령인 줄을 하나 낸다. 범위를 재는 동안 50 Hz 로 읽고 10 Hz 로 알린다 (ack null).
	done · cancelled · error 를 알리면 버스를 닫고 끝난다.

되돌리기: 어떻게 끝나든 (오류 · 예외 · 표준 입력 닫힘 = 게이트웨이가 죽었다 · SIGTERM) session.cancel() 로
제로-포지션 위치 시키기로 쓴 값을 예전 것으로 되돌린다. 되돌리지 못하면 error 의 reason 에 적는다. SIGINT 는 무시한다
(터미널 Ctrl-C 가 되돌리기를 건너뛰지 않게). 전원이 나가거나 죽어도 되돌릴 수 있게 아무것도 쓰기 전에 모터의
예전 보정을 restore_path 에 남기고 (fsync) 되돌리거나 끝내면 지운다. 시작할 때 restore_path 가 남아 있으면
먼저 그 값을 모터에 쓴다 (restore_only 면 그것만 하고 cancelled 로 끝난다).
"""

import json
import queue
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Final

# 스크립트로 켜져도 패키지 이름으로 가져온다 (패키지 __init__ 은 __version__ 뿐이다)
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lekiwi_web_operator import lerobot_calibration as calibration  # noqa: E402

RECORD_PERIOD: Final[float] = 1.0 / 50   # [s] 범위 재기 읽기
STATUS_PERIOD: Final[float] = 0.1        # [s] 범위 재는 동안 알림
IDLE_PERIOD: Final[float] = 0.5          # [s] 명령을 기다리는 동안 (SIGTERM 을 보는 주기)
STOP: Final[threading.Event] = threading.Event()   # SIGTERM: 다음 명령 대신 취소


def emit(
	state: str, joints: dict | None = None, reason: str | None = None, backup: str | None = None,
	ack: str | None = None,
) -> None:
	"""상태 한 줄. 게이트웨이가 죽어 표준 출력이 닫혔어도 되돌리기는 계속한다."""
	try:
		print(json.dumps(
			{'state': state, 'joints': joints, 'reason': reason, 'backup': backup, 'ack': ack},
			ensure_ascii=False), flush=True)
	except (OSError, ValueError):
		pass


def describe(error: BaseException) -> str:
	return str(error) or type(error).__name__


def read_commands(commands: queue.Queue) -> None:
	"""(스레드) 표준 입력의 명령. 끝나면 cancel 을 넣는다."""
	for line in sys.stdin:
		try:
			command = json.loads(line)
		except ValueError:
			continue
		if isinstance(command, dict):
			commands.put(command)
	commands.put({'cmd': 'cancel'})


def open_session(bus: Any, restore_path: Path | None) -> tuple[Any, str | None]:
	"""(지난 보정이 남긴 값이 있으면 먼저 되돌리고) 새 세션을 만들고 예전 보정을 restore_path 에 남긴다.

	Returns:
		(세션, 되돌렸으면 그 알림).

	Raises:
		CalibrationError: 팔 토크가 켜져 있다 (아무것도 쓰지 않았다). 그 밖의 버스 · 파일 오류.
	"""
	session = calibration.CalibrationSession(bus)   # 토크 검사 · 끄기 (켜져 있으면 아무것도 쓰지 않고 오류)
	note = None
	if restore_path is not None and restore_path.exists():
		bus.write_calibration(calibration.load_calibration_file(restore_path))
		restore_path.unlink()
		note = '지난 보정이 끝나지 않아 예전 보정 값을 먼저 모터에 되돌렸습니다'
		session = calibration.CalibrationSession(bus)   # 되돌린 값을 '예전 값' 으로 다시 읽는다
	if restore_path is not None:
		calibration.save_calibration_file(restore_path, calibration.calibration_from_motors(bus))
	return session, note


def close_session(session: Any, restore_path: Path | None) -> str | None:
	"""제로-포지션 위치 시키기로 쓴 값을 되돌린다 (done · cancelled 뒤면 아무것도 하지 않는다). 못 했으면 이유."""
	if session is None:
		return None
	try:
		session.cancel()
	except BaseException as error:   # 되돌리기 파일을 남긴다: 다음 시작이 다시 되돌린다
		return f'예전 보정 값을 모터에 되돌리지 못했습니다 ({describe(error)}). 다음 보정 시작 때 다시 되돌립니다'
	if restore_path is not None:
		restore_path.unlink(missing_ok=True)
	return None


def run_steps(commands: queue.Queue, session: Any, path: Path, restore_path: Path | None) -> int:
	"""제로-포지션 위치 시키기 → 범위 재기 → 끝내기 · 취소. 쓰다 실패한 오류는 그대로 낸다 (부른 쪽이 되돌린다)."""
	next_status = 0.0
	while True:
		is_recording = session.state == calibration.STATE_RECORDING
		command = None
		if STOP.is_set():
			command = {'cmd': 'cancel'}
		else:
			try:
				command = commands.get(timeout=RECORD_PERIOD if is_recording else IDLE_PERIOD)
			except queue.Empty:
				pass
		if command is None:
			if is_recording:
				joints = session.sample()
				if time.monotonic() >= next_status:
					next_status = time.monotonic() + STATUS_PERIOD
					emit(session.state, joints)
			continue
		name = command.get('cmd')
		try:
			if name == 'middle':
				session.middle()
				emit(session.state, session.sample(), ack=name)
			elif name == 'finish':
				session.finish(path)
				if restore_path is not None:
					restore_path.unlink(missing_ok=True)   # 새 보정을 다 썼다: 예전 값은 이제 쓰지 않는다
				backup = None if session.backup_path is None else str(session.backup_path)
				emit(session.state, backup=backup, ack=name)
				return 0
			elif name == 'cancel':
				reason = close_session(session, restore_path)
				if reason is not None:
					emit(calibration.STATE_ERROR, reason=reason, ack=name)
					return 1
				emit(session.state, ack=name)
				return 0
			else:
				emit(session.state, reason=f'모르는 명령입니다: {name}', ack=str(name))
		except calibration.CalibrationError as error:
			if session.state == calibration.STATE_ERROR:
				raise   # 쓰다 실패했다
			emit(session.state, reason=str(error), ack=name)   # 그 단계를 할 수 없다: 상태는 그대로


def run(commands: queue.Queue) -> int:
	first: Any = commands.get()
	if first.get('cmd') != 'start':
		emit(calibration.STATE_ERROR, reason='start 명령이 먼저 와야 합니다')
		return 1
	path = Path(str(first.get('calibration_path')))
	restore_path = Path(str(first['restore_path'])) if first.get('restore_path') else None
	try:
		bus = calibration.open_bus(calibration.DEVICE_LEKIWI, str(first.get('port')))
	except BaseException as error:   # lerobot 이 여러 예외를 낸다: 사람이 읽을 글로 넘긴다
		emit(calibration.STATE_ERROR, reason=f'모터 버스를 열지 못했습니다: {describe(error)}')
		return 1
	session = None
	try:
		session, note = open_session(bus, restore_path)
		if first.get('restore_only'):
			close_session(session, restore_path)
			emit(calibration.STATE_CANCELLED, reason=note or '되돌릴 예전 보정 값이 없었습니다', ack='start')
			return 0
		emit(session.state, reason=note, ack='start')
		return run_steps(commands, session, path, restore_path)
	except BaseException as error:   # KeyboardInterrupt · SystemExit 도: 되돌리지 않고 끝내지 않는다
		if isinstance(error, calibration.CalibrationError):
			reason = str(error)
		else:
			reason = f'보정 중 오류: {describe(error)}'
		failure = close_session(session, restore_path)
		if failure is not None:
			reason = f'{reason} · {failure}'
		elif session is not None:
			reason = f'{reason} (모터 보정은 예전 값으로 되돌렸습니다)'
		emit(calibration.STATE_ERROR, reason=reason)
		return 1
	finally:
		try:
			bus.disconnect(disable_torque=False)
		except Exception as error:   # 할 일은 끝났다: 기록만
			print(f'버스를 닫지 못했습니다: {describe(error)}', file=sys.stderr)


def main() -> int:
	signal.signal(signal.SIGINT, signal.SIG_IGN)
	signal.signal(signal.SIGTERM, lambda *_: STOP.set())
	commands: queue.Queue = queue.Queue()
	threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
	return run(commands)


if __name__ == '__main__':
	raise SystemExit(main())
