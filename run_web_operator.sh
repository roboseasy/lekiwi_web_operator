#!/usr/bin/env bash
# 로봇(Pi)에서 웹 오퍼레이터를 실행한다. systemd(lekiwi-web-operator.service)도 이 스크립트를 부른다.
#
# 사용법: ~/lekiwi_web_operator/run_web_operator.sh [--port 8080] [--read-only]
# 접속: http://<로봇 이름>.local:8080
# systemd 는 .bashrc 를 읽지 않으므로 ROS 환경과 기본값을 여기서 정한다
# (ROS_DOMAIN_ID 42, ROS_AUTOMATIC_DISCOVERY_RANGE SUBNET, RMW_IMPLEMENTATION rmw_fastrtps_cpp).
# venv 를 쓰지 않는다: 시스템 Python 의 rclpy · PIL · psutil · yaml 을 쓴다.
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
LEKIWI_SETUP="$HOME/lekiwi_ws/install/setup.bash"
if [[ ! -f "$LEKIWI_SETUP" ]]; then
	printf 'lekiwi_ws 가 빌드되지 않았습니다: %s 이 없습니다.\n' "$LEKIWI_SETUP" >&2
	exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$LEKIWI_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONPATH="$SCRIPT_DIR${PYTHONPATH:+:$PYTHONPATH}"

exec /usr/bin/python3 -m lekiwi_web_operator "$@"
