"""`python -m lekiwi_pc_agent --robot <주소>`: PC 에이전트를 실행한다.

SIGINT (Ctrl+C) · SIGTERM · SIGHUP (터미널 닫힘) → 리더암을 닫고 → WebSocket 을 닫고 → 0 으로 끝낸다.
"""

import argparse
import logging
import signal
import sys
import threading
from typing import Final

from lekiwi_pc_agent.leader import LerobotLeader, default_calibration_dir

DEFAULT_PORT: Final[int] = 8080


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		prog='python -m lekiwi_pc_agent',
		description='LeKiwi PC 에이전트: 이 PC 의 리더암 값을 로봇 게이트웨이로 보낸다')
	parser.add_argument('--robot', required=True, help='로봇 주소 (예: 192.168.0.204, lekiwi04.local)')
	parser.add_argument(
		'--port', type=int, default=DEFAULT_PORT, help='게이트웨이 포트 (기본 %(default)s)')
	parser.add_argument('--verbose', action='store_true', help='자세한 로그 (lerobot 포함)')
	return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
	"""에이전트 실행. 끝 신호를 받을 때까지 막힌다.

	Args:
		argv: 명령줄 인자 (None 이면 sys.argv).

	Returns:
		종료 코드 (정상 0, lerobot 없음 1).
	"""
	args = _parse_args(argv)
	logging.basicConfig(
		level=logging.DEBUG if args.verbose else logging.INFO,
		format='%(asctime)s %(message)s', datefmt='%H:%M:%S')
	if not args.verbose:
		logging.getLogger('lerobot').setLevel(logging.WARNING)
	stop_event = threading.Event()
	stop_signals = [signal.SIGINT, signal.SIGTERM]
	if signal.getsignal(signal.SIGHUP) is not signal.SIG_IGN:  # nohup 으로 띄웠으면 그대로 둔다
		stop_signals.append(signal.SIGHUP)
	for signal_number in stop_signals:
		signal.signal(signal_number, lambda _number, _frame: stop_event.set())
	try:
		# lerobot 을 가져오는 모듈: 없으면 traceback 대신 알기 쉬운 한 줄로 끝낸다
		from lekiwi_pc_agent import calibration
		from lekiwi_pc_agent.agent import Agent
		calibration_dir = default_calibration_dir()
	except ImportError as error:
		logging.error(
			'이 파이썬에 lerobot 이 없습니다 (%s): conda run -n lerobot060 으로 실행하세요', error)
		return 1
	agent = Agent(
		args.robot, args.port,
		lambda port, calibration_id: LerobotLeader(port, calibration_id, calibration_dir),
		lambda port: calibration.open_bus(calibration.DEVICE_SO101_LEADER, port),
		calibration_dir)
	agent.run(stop_event)
	return 0


if __name__ == '__main__':
	sys.exit(main())
