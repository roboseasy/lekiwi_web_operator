"""`python3 -m lekiwi_web_operator`: 게이트웨이를 조립해 실행한다 (SPEC §2 · §15).

SIGINT · SIGTERM · SIGHUP (터미널 · SSH 끊김) → 서버 종료 → WS 닫기 → robot.shutdown()
(정지 · 토크 끔 · 웹에서 켠 launch 끔).
"""

import argparse
import logging
import signal
import sys
import threading

from lekiwi_web_operator import __version__, config
from lekiwi_web_operator.http_server import OperatorHttpServer
from lekiwi_web_operator.hub import Hub
from lekiwi_web_operator.network import NetworkService
from lekiwi_web_operator.robot import RobotService
from lekiwi_web_operator.system_info import SystemInfo

logger = logging.getLogger('lekiwi_web_operator')


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
	parser = argparse.ArgumentParser(
		prog='python3 -m lekiwi_web_operator',
		description='LeKiwi 웹 오퍼레이터: 브라우저로 로봇을 보고 조작하는 게이트웨이')
	parser.add_argument(
		'--host', default=config.DEFAULT_HOST, help='열 주소 (기본 %(default)s)')
	parser.add_argument(
		'--port', type=int, default=config.DEFAULT_PORT, help='포트 (기본 %(default)s)')
	parser.add_argument(
		'--read-only', action='store_true',
		help='보기 전용: 로봇을 움직이거나 기능 · 와이파이를 바꾸지 않는다')
	return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
	"""게이트웨이 실행. SIGINT · SIGTERM · SIGHUP 을 받을 때까지 막힌다.

	Args:
		argv: 명령줄 인자 (None 이면 sys.argv).

	Returns:
		종료 코드 (정상 0, 시작 실패 1).
	"""
	args = _parse_args(argv)
	logging.basicConfig(
		level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s')
	stop_event = threading.Event()
	stop_signals = [signal.SIGINT, signal.SIGTERM]
	# 터미널 · SSH 가 끊기면 Ctrl+C 처럼 정리한다 (nohup 으로 무시하게 띄웠으면 그대로 둔다)
	if signal.getsignal(signal.SIGHUP) is not signal.SIG_IGN:
		stop_signals.append(signal.SIGHUP)
	for signal_number in stop_signals:
		signal.signal(signal_number, lambda _number, _frame: stop_event.set())
	robot = RobotService(args.read_only)
	try:
		return _serve(robot, args, stop_event)
	finally:
		robot.shutdown()  # 어떤 길로 끝나든 정지 · 토크 끔 · 웹에서 켠 launch 종료


def _serve(robot: RobotService, args: argparse.Namespace, stop_event: threading.Event) -> int:
	try:
		robot.start()
	except RuntimeError as error:
		print(f'로봇 쪽을 시작하지 못했습니다: {error}', file=sys.stderr)
		return 1
	network = NetworkService(args.port)
	hub = Hub(robot, SystemInfo(), network)
	try:
		server = OperatorHttpServer((args.host, args.port), hub, robot, network)
	except (OSError, OverflowError) as error:
		print(f'주소를 열지 못했습니다 ({args.host}:{args.port}): {error}', file=sys.stderr)
		return 1
	threading.Thread(target=server.serve_forever, name='http', daemon=True).start()
	port = server.server_address[1]
	is_any_address = args.host in ('', '0.0.0.0')
	urls = network.access_urls() if is_any_address else [f'http://{args.host}:{port}']
	mode = ' (보기 전용)' if args.read_only else ''
	print(f'LeKiwi 웹 오퍼레이터 {__version__}{mode} 시작. 브라우저로 이 주소에 접속하세요:',
		flush=True)
	for url in urls:
		print(f'  {url}', flush=True)
	stop_event.wait()
	logger.info('끄는 중입니다: 로봇을 멈추고 웹에서 켠 기능을 끕니다')
	server.shutdown()
	hub.close_all()
	server.server_close()
	return 0


if __name__ == '__main__':
	raise SystemExit(main())
