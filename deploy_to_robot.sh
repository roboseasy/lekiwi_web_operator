#!/usr/bin/env bash
# PC → 로봇으로 웹 오퍼레이터를 복사한다 (dev/ · tests/ · docs/ · .omc/ 는 보내지 않는다).
#
# 사용법: ./deploy_to_robot.sh <사용자@호스트>
#   예) ./deploy_to_robot.sh roboseasy@lekiwi04.local
# 로봇의 ~/lekiwi_web_operator/ 를 이 폴더와 똑같이 맞춘다 (rsync --delete).
set -eo pipefail

usage() {
	echo '사용법: ./deploy_to_robot.sh <사용자@호스트>   예) ./deploy_to_robot.sh roboseasy@lekiwi04.local'
}
# 대상은 꼭 적는다. rsync 옵션 · 원격 경로로 읽히지 않게 [사용자@]호스트 모양만 받는다
if (( $# != 1 )) || [[ "$1" == -* || ! "$1" =~ ^([A-Za-z0-9._-]+@)?[A-Za-z0-9.-]+$ ]]; then
	usage >&2
	exit 2
fi
TARGET="$1"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 원격 경로는 원격 홈 기준이다 (= ~/lekiwi_web_operator/)
# .omc 는 에이전트 도구의 작업 상태 폴더, docs · TechnicalSpecification.md 는 내부 기획 · 위험 기록이다. 로봇에 올라가면 이미지에 실린다
# (rsync 는 제외한 것을 로봇에서 지우지 않는다: 이미 올라간 것은 손으로 지운다)
rsync -a --delete --exclude dev --exclude tests --exclude __pycache__ --exclude .pytest_cache \
	--exclude .omc --exclude pytest.ini --exclude docs --exclude TechnicalSpecification.md "$SCRIPT_DIR/" "$TARGET:lekiwi_web_operator/"

cat <<EOF
복사했습니다: $TARGET:~/lekiwi_web_operator/

바로 실행 (시험용. Ctrl+C 나 터미널을 닫으면 정리하고 끝. 오래 켜 둘 때 · 자율 주행은 아래 자동 실행):
  ssh -t $TARGET '~/lekiwi_web_operator/run_web_operator.sh'

부팅할 때 자동 실행 (처음 한 번, 서비스 사용자는 roboseasy):
  ssh -t $TARGET 'sudo cp ~/lekiwi_web_operator/lekiwi-web-operator.service /etc/systemd/system/'
  ssh -t $TARGET 'sudo systemctl daemon-reload && sudo systemctl enable --now lekiwi-web-operator'
  (서비스가 시작할 때 'loginctl enable-linger roboseasy' 를 켭니다: 마지막 SSH 를 끊으면
   logind 가 ROS 공유 메모리를 지워 로봇과 통신이 끊기는 것을 막습니다.
   확인: ssh $TARGET 'loginctl show-user roboseasy -p Linger' → Linger=yes)

다시 배포한 뒤:  ssh -t $TARGET 'sudo systemctl restart lekiwi-web-operator'
상태:            ssh -t $TARGET 'systemctl status lekiwi-web-operator'
로그:            ssh -t $TARGET 'journalctl -u lekiwi-web-operator -f'
접속:            http://${TARGET#*@}:8080
EOF
