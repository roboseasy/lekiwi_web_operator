#!/usr/bin/env bash
# 이동 설계 페이지의 정적 파일(web/)만 로봇에 올린다. 서비스 재시작 · bringup · Python 은 건드리지 않는다.
# (저장소 루트의 deploy_to_robot.sh 는 Python 까지 통째로 올리고 서비스를 재시작한다. 이것은 그것과 다르다.)
#
#   dev/deploy_motion_page.sh [--check] <로봇 IP 또는 이름>...
#   --check  올리지 않고 로봇마다 상태만 본다
#
# 로봇마다 SSH 연결을 한 번만 열어 비밀번호는 한 번만 묻는다 (키가 등록돼 있으면 안 묻는다).
# 올리기 전에 로봇의 web/ 가 '원래 버전'(이 페이지를 더하기 전)인지 확인하고, 아니면 거부한다 (FORCE=1 로 무시).
# 백업은 로봇의 ~/lekiwi_web_operator/web.bak-<날짜> 에 남긴다. 되돌리기: 그 폴더를 web 으로 복사.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_NAME="${ROBOT_USER:-roboseasy}"
REMOTE_WEB='$HOME/lekiwi_web_operator/web'
BACKUP="web.bak-$(date +%Y%m%d)"
# 올릴 것 (web/ 기준). 이 중 앞의 셋이 원래 있던 파일이고, 나머지는 새 파일이다.
FILES=(css/app.css index.html js/app.js js/motion js/pages/motion.js)
# 이동 설계 페이지를 더하기 전, 로봇의 원래 파일 md5 (lekiwi04 실측, 2026-10-25)
declare -A ORIGINAL=(
	[css/app.css]=aaa75a52797f38928e91d6183471fdb5
	[index.html]=9e0ecbbb085ed0eb30d76993fbaefe85
	[js/app.js]=1e84ef4006cef88b7a8528770c5a15e3
)

IS_CHECK=0
if [[ "${1:-}" == "--check" ]]; then
	IS_CHECK=1
	shift
fi
if [[ $# -eq 0 ]]; then
	sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
	exit 1
fi

local_md5() { (cd "$ROOT/web" && md5sum "$1" | cut -d' ' -f1); }

for host in "$@"; do
	echo "=== $host ==="
	control="$(mktemp -u /tmp/lekiwi-deploy-XXXXXX)"
	if ! ssh -o ControlMaster=yes -o ControlPath="$control" -o ControlPersist=300 -o ConnectTimeout=8 -fN "$USER_NAME@$host"; then
		echo "  SSH 연결 실패 (키 · 비밀번호 · 호스트 키를 확인하세요)"
		continue
	fi
	remote() { ssh -o ControlPath="$control" "$USER_NAME@$host" "$@"; }

	if ! remote "test -d $REMOTE_WEB"; then
		echo "  ~/lekiwi_web_operator/web 가 없습니다"
		ssh -o ControlPath="$control" -O exit "$USER_NAME@$host" 2>/dev/null || true
		continue
	fi
	echo "  로봇: $(remote 'hostname') · 서비스 환경: $(remote 'systemctl show lekiwi-web-operator -p Environment 2>/dev/null | cut -c1-80')"

	# 원래 버전인가 / 이미 최신인가
	state="original"
	for f in css/app.css index.html js/app.js; do
		remote_md5="$(remote "md5sum $REMOTE_WEB/$f 2>/dev/null | cut -d' ' -f1")"
		if [[ "$remote_md5" == "$(local_md5 "$f")" ]]; then
			[[ "$state" == "original" ]] && state="latest"
		elif [[ "$remote_md5" != "${ORIGINAL[$f]}" ]]; then
			state="unknown"
			echo "  $f 가 원래 버전도 최신도 아닙니다 ($remote_md5)"
		fi
	done
	echo "  web/ 상태: $state"
	if [[ "$state" == "latest" ]]; then
		echo "  이미 최신입니다. 올리지 않습니다."
	elif [[ "$state" == "unknown" && "${FORCE:-0}" != "1" ]]; then
		echo "  다른 버전이라 올리지 않습니다 (알고 있다면 FORCE=1)."
	elif [[ "$IS_CHECK" == "1" ]]; then
		echo "  (확인만) 올리면 됩니다."
	else
		remote "if [ -e $REMOTE_WEB/../$BACKUP ]; then echo '  백업이 이미 있습니다: $BACKUP'; else cp -a $REMOTE_WEB $REMOTE_WEB/../$BACKUP && echo '  백업: ~/lekiwi_web_operator/$BACKUP'; fi"
		tar -C "$ROOT/web" -cf - "${FILES[@]}" | remote "tar -C $REMOTE_WEB -xf - && echo '  복사 완료'"
		# 올린 파일을 바이트 단위로 대조한다
		mismatch=0
		while IFS= read -r f; do
			[[ "$(remote "md5sum $REMOTE_WEB/$f | cut -d' ' -f1")" == "$(local_md5 "$f")" ]] || { echo "  대조 실패: $f"; mismatch=1; }
		done < <(cd "$ROOT/web" && find "${FILES[@]}" -type f | sort)
		[[ "$mismatch" == "0" ]] && echo "  대조 통과: 올린 파일이 로컬과 같습니다 (서비스 재시작 없음, 브라우저 새로고침)"
	fi
	ssh -o ControlPath="$control" -O exit "$USER_NAME@$host" 2>/dev/null || true
done
