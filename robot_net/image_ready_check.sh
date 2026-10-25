#!/bin/bash
# 이미지를 뜨기 직전 골든 카드 점검. 읽기 전용: 아무것도 쓰거나 지우지 않는다. 로봇에서 sudo로 돌린다.
#   sudo image_ready_check.sh
# 항목마다 한 줄 [OK|WARN|FAIL] <항목>: <설명> (찾은 것이 많으면 10개까지), 마지막 줄 '요약: FAIL n · WARN m'.
# FAIL이 하나라도 있으면 종료 1. 비밀번호 · 토큰 · 키는 값이 아니라 파일 이름(과 개수)만 보인다.
# 비밀번호는 두 길로 찾는다: 줄 모양(psk= · password: · wpa-psk · JSON "password":) 그리고 실제 값(값 추적:
# 프로필 · 그 줄들에서 읽은 값과 공유기 64자 hex PSK를 grep -F -f <(…)로, 값은 출력에도 명령줄(ps)에도 나오지 않는다).
# 시험용 환경변수: CHECK_ROOT (모든 절대 경로 앞에 붙는다. 있으면 root 확인을 건너뛴다),
#   CHECK_HOME (기본 $SUDO_USER의 홈), CHECK_BOOT_DEVICE (부트 파티션 장치, 기본 findmnt로 /boot/firmware의 것)
set -u
shopt -s nullglob

ROOT="${CHECK_ROOT:-}"
if [ -n "$ROOT" ]; then     # 가짜 루트는 시험용 (provision · clone_init와 같은 규칙)
	[ "$(id -u)" -ne 0 ] || { echo "error: CHECK_ROOT는 시험용입니다 (root로는 쓰지 않는다)" >&2; exit 1; }
else
	[ "$(id -u)" -eq 0 ] || { echo "error: sudo로 실행하세요 (root만 읽을 수 있는 파일이 있다)" >&2; exit 1; }
fi
HOME_DIR="${CHECK_HOME:-}"
[ -n "$HOME_DIR" ] || [ -z "${SUDO_USER:-}" ] || HOME_DIR=$(getent passwd "$SUDO_USER" | cut -d: -f6)
[ -n "$HOME_DIR" ] || { echo "error: 홈 폴더를 모릅니다 (sudo로 실행하거나 CHECK_HOME)" >&2; exit 1; }
H="$ROOT$HOME_DIR"
KFDIR="$ROOT/etc/NetworkManager/system-connections"
AP_KF="$KFDIR/lekiwi-ap.nmconnection"
HOME_KF="$KFDIR/home-wifi.nmconnection"
AP_NAME=$(sed -n 's/^ssid=//p' "$AP_KF" 2>/dev/null | head -n 1)
AP_PSK_RAW=$(sed -n 's/^psk=//p' "$AP_KF" 2>/dev/null | head -n 1)     # 비교만 한다 (찍지 않는다)
SECRET_LINE='^[[:space:]]*(psk=|password:|wpa-psk)|"(password|psk)"[[:space:]]*:'

NFAIL=0
NWARN=0
say() {   # say 등급 항목 설명
	printf '[%s] %s: %s\n' "$1" "$2" "$3"
	case "$1" in FAIL) NFAIL=$((NFAIL + 1)) ;; WARN) NWARN=$((NWARN + 1)) ;; esac
}
report() {   # report 등급 항목 [찾은 것...]: 찾은 것마다 한 줄 (CHECK_ROOT는 떼고 10개까지), 없으면 OK 한 줄
	local level="$1" item="$2" n=0 found
	shift 2
	[ $# -gt 0 ] || { say OK "$item" '없음'; return; }
	for found in "$@"; do
		n=$((n + 1))
		[ "$n" -gt 10 ] || say "$level" "$item" "${found#"$ROOT"}"
	done
	[ "$n" -le 10 ] || say "$level" "$item" "… 외 $((n - 10))개"
}
existing() {   # existing 경로...: 있는 것만 (깨진 링크도)
	local p
	for p in "$@"; do
		if [ -e "$p" ] || [ -L "$p" ]; then printf '%s\n' "$p"; fi
	done
}
nonempty() {   # nonempty 폴더...: 안에 무엇이든 든 폴더만
	local d
	for d in "$@"; do
		[ -z "$(find "$d" -mindepth 1 -print -quit 2>/dev/null)" ] || printf '%s\n' "$d"
	done
}
COMMON_SRC=()     # 흔한 글자라 추적하지 않은 값이 나온 파일
COMMON_ROUTER=''
COMMON_AP=''
add_val() {   # add_val 배열이름 값 [출처]: 8자 이상만 (WPA 최소). keyfile에 \ 가 두 번 적혔으면 푼 모양도 함께.
	# /etc/passwd · group · hostname에 든 흔한 글자(사용자 · 그룹 · 호스트 이름)는 곳곳에 있어 추적하지 않는다 (값은 stdin으로만)
	local -n into="$1"
	local v forms=("$2")
	[ ${#2} -ge 8 ] || return 0
	[ "${2//\\\\/\\}" = "$2" ] || forms+=("${2//\\\\/\\}")
	for v in "${forms[@]}"; do
		if grep -qsF -f <(printf '%s\n' "$v") "$ROOT/etc/passwd" "$ROOT/etc/group" "$ROOT/etc/hostname"; then
			COMMON_SRC+=("${3:-?}")
			if [ "$1" = AP_VALS ]; then COMMON_AP=1; else COMMON_ROUTER=1; fi
		else
			into+=("$v")
		fi
	done
}
secret_of() {   # secret_of 줄: 그 줄의 비밀번호 값. 자리표시($… <…>) · 코드 모양(따옴표 없이 공백 · 괄호 · 쉼표)은 값이 아니다
	local l="${1#"${1%%[![:space:]]*}"}" v=''
	if [[ $l =~ \"(password|psk)\"[[:space:]]*:[[:space:]]*\"([^\"]*)\" ]]; then
		v=${BASH_REMATCH[2]}
	elif [[ $l =~ ^(psk=|password:|wpa-psk)[[:space:]]*(.*)$ ]]; then
		v=${BASH_REMATCH[2]}
		v=${v%"${v##*[![:space:]]}"}
		case "$v" in
		\"*\"|\'*\') v=${v:1:${#v}-2} ;;
		*) [ "${BASH_REMATCH[1]}" = psk= ] || case "$v" in *[[:space:]\(\)\[\]\{\},]*) v='' ;; esac ;;
		esac
	fi
	case "$v" in \$*|\<*) v='' ;; esac
	[ ${#v} -ge 8 ] && printf '%s' "$v"
}
is_login() {   # cloud-init 로그인 설정 (password:는 계정 비밀번호다. 공유기 것과 따로 본다)
	case "$1" in
	"$ROOT/boot/firmware/user-data"|"$ROOT"/var/lib/cloud/instances/*/user-data.txt*|"$ROOT"/var/lib/cloud/instances/*/cloud-config.txt) return 0 ;;
	esac
	return 1
}

# --- 훑을 곳 (줄 모양 검사 · 값 추적이 같이 쓴다)
# /etc · /var(apt 목록 · 캐시 빼고) · /boot/firmware · /srv · /usr/local · /tmp · /opt(ROS 패키지 빼고) · /root · /home/*.
# 큰 코드 · 도구 폴더(venv · .vscode-server · lekiwi_ws의 build · install · log)는 뺀다. /run · /proc는 휘발성이라 뺀다.
# 한 단계씩 펼쳐 넘기고 링크는 뺀다 (grep -r은 명령줄의 링크를 따라간다: /var/run → /run 등)
SYS_SCAN=()
HOME_SCAN=()
add_scan() {   # add_scan 배열이름 경로...
	local -n into="$1"
	local p
	shift
	for p in "$@"; do
		if [ -e "$p" ] && [ ! -L "$p" ]; then into+=("$p"); fi
	done
}
add_scan SYS_SCAN "$ROOT/etc" "$ROOT/boot/firmware" "$ROOT/srv" "$ROOT/usr/local" "$ROOT/tmp"
for p in "$ROOT"/var/* "$ROOT"/var/lib/* "$ROOT"/var/cache/*; do
	case "${p#"$ROOT"}" in /var/lib|/var/cache|/var/lib/apt|/var/cache/apt) ;; *) add_scan SYS_SCAN "$p" ;; esac
done
for p in "$ROOT"/opt/*; do [ "${p##*/}" = ros ] || add_scan SYS_SCAN "$p"; done
for home in "$ROOT/root" "$ROOT"/home/*; do
	for p in "$home"/* "$home"/.[!.]* "$home"/lekiwi_ws/* "$home"/lekiwi_ws/.[!.]*; do
		case "${p#"$home"/}" in
		lerobot060_venv|.vscode-server|lekiwi_ws|lekiwi_ws/build|lekiwi_ws/install|lekiwi_ws/log) ;;
		*) add_scan HOME_SCAN "$p" ;;
		esac
	done
done
SKIP_DIRS=(--exclude-dir=lerobot060_venv --exclude-dir=.vscode-server)

# --- FAIL: 이미지에 들어가면 안 되는 것
mapfile -t found < <(
	existing "$H/backups" "$ROOT"/var/backups/lekiwi_net_* "$H/robot_net_staging"
	nonempty "$ROOT/var/lib/lekiwi-net"
	find "$H" -mindepth 1 -maxdepth 2 \( -path "$H/backups" -o -path "$H/robot_net_staging" \) -prune -o -type f \
		\( -name '*.tar*' -o -name '*.tgz' -o -name '*.bak*' -o -name '*.orig' -o -name '*.prev' \) -print 2>/dev/null
)
report FAIL '백업 · 작업 흔적' "${found[@]}"

mapfile -t found < <(find "$ROOT/root" "$ROOT/tmp" "$ROOT/var/tmp" "$ROOT/srv" "$ROOT/usr/local" "$ROOT/var/backups" \
	"$ROOT"/opt \( -path "$ROOT/opt/ros" -o -path "$ROOT/root/.vscode-server" \) -prune -o -type f \
	\( -name '*.tgz' -o -name '*.tar' -o -name '*.tar.*' -o -name '*.zip' -o -name '*.7z' \) \
	! -name 'alternatives.tar*' ! -name 'lekiwi_net_*' -print 2>/dev/null)
report FAIL '홈 밖의 보관 파일 (백업 · 묶음)' "${found[@]}"

# 줄 모양 검사: 값을 읽어 값 추적의 씨앗으로도 쓴다
declare -A REPORTED=()
SECRET_FILES=()
LOGIN_FILES=()
ROUTER_VALS=()
AP_VALS=()
HOME_PSK=''
while IFS= read -r -d '' path && IFS= read -r line; do
	if is_login "$path"; then
		[ -n "${REPORTED[$path]:-}" ] || { REPORTED[$path]=login; LOGIN_FILES+=("$path"); }
		continue
	fi
	v=$(secret_of "$line") || continue
	case "$path" in
	"$AP_KF")     # AP 프로필은 남긴다. 공개된 값(AP 이름과 같거나 처음 기본값 lekiwi01)은 곳곳에 있어 추적하지 않는다
		case "${v//\\\\/\\}" in "${AP_NAME//\\\\/\\}"|lekiwi01) ;; *) add_val AP_VALS "$v" "$path" ;; esac ;;
	"$HOME_KF") add_val ROUTER_VALS "$v" "$path"; HOME_PSK=$v ;;   # 공유기 프로필은 아래 WARN
	*)
		add_val ROUTER_VALS "$v" "$path"
		[ -n "${REPORTED[$path]:-}" ] || { REPORTED[$path]=secret; SECRET_FILES+=("$path"); } ;;
	esac
done < <(grep -rHZsIE "${SKIP_DIRS[@]}" "$SECRET_LINE" "${SYS_SCAN[@]}" "${HOME_SCAN[@]}")
report FAIL '비밀번호가 든 파일' "${SECRET_FILES[@]}"

# 값 추적: 줄 모양이 없는 곳(cloud-init JSON 사본, sudo 기록이 남은 로그, 셸 기록)을 실제 값으로 찾는다
HOME_SSID=$(sed -n 's/^ssid=//p' "$HOME_KF" 2>/dev/null | head -n 1)
if [ -n "$HOME_SSID" ] && [ -n "$HOME_PSK" ]; then     # wpa_supplicant가 쓰는 64자 hex PSK (값은 stdin으로만)
	add_val ROUTER_VALS "$(printf '%s\n%s\n' "${HOME_SSID//\\\\/\\}" "${HOME_PSK//\\\\/\\}" | python3 -c '
import hashlib, sys
ssid, psk = sys.stdin.read().split("\n")[:2]
print(hashlib.pbkdf2_hmac("sha1", psk.encode(), ssid.encode(), 4096, 32).hex())')" "$HOME_KF"
fi
if [ ${#COMMON_SRC[@]} -gt 0 ]; then     # 출처 파일 이름만 (값은 찍지 않는다)
	declare -A seen_src=()
	SRC_LIST=''
	for f in "${COMMON_SRC[@]}"; do
		[ -z "${seen_src[$f]:-}" ] || continue
		seen_src[$f]=1
		SRC_LIST+="${SRC_LIST:+, }${f#"$ROOT"}"
	done
	say INFO '값 추적' "흔한 글자라 건너뜀, 출처 파일: $SRC_LIST"
fi
ROUTER_WHY='home-wifi 프로필도 비밀번호 줄도 없다'
[ -z "$COMMON_ROUTER" ] || ROUTER_WHY='값이 흔한 글자뿐이다'
if [ ! -f "$AP_KF" ]; then
	AP_WHY='lekiwi-ap 프로필이 없다'
elif [ -n "$COMMON_AP" ]; then
	AP_WHY='값이 흔한 글자다'
else
	AP_WHY='AP 이름과 같거나 lekiwi01인 공개 기본값'
fi
if [ $((${#ROUTER_VALS[@]} + ${#AP_VALS[@]})) -eq 0 ]; then
	say INFO '값 추적' "건너뜀 (공유기: $ROUTER_WHY / AP: $AP_WHY)"
else
	[ ${#ROUTER_VALS[@]} -gt 0 ] || say INFO '값 추적' "공유기 값은 건너뜀 ($ROUTER_WHY)"
	[ ${#AP_VALS[@]} -gt 0 ] || say INFO '값 추적' "AP 값은 건너뜀 ($AP_WHY)"
	mapfile -t hits < <(grep -rlsaF "${SKIP_DIRS[@]}" -f <(printf '%s\n' "${ROUTER_VALS[@]}" "${AP_VALS[@]}") \
		"${SYS_SCAN[@]}" "${HOME_SCAN[@]}")
	router_hits=()
	ap_hits=()
	for f in "${hits[@]}"; do
		[ "${REPORTED[$f]:-}" != secret ] || continue      # 위에서 이미 FAIL
		if [ ${#ROUTER_VALS[@]} -gt 0 ] && grep -qsaF -f <(printf '%s\n' "${ROUTER_VALS[@]}") "$f"; then
			case "$f" in "$HOME_KF"|"$ROOT"/etc/netplan/*) ;; *) router_hits+=("$f") ;; esac
		fi
		if [ ${#AP_VALS[@]} -gt 0 ] && grep -qsaF -f <(printf '%s\n' "${AP_VALS[@]}") "$f"; then
			[ "$f" = "$AP_KF" ] || ap_hits+=("$f")
		fi
	done
	[ ${#ROUTER_VALS[@]} -eq 0 ] || report FAIL '공유기 비밀번호 값이 든 파일' "${router_hits[@]}"
	[ ${#AP_VALS[@]} -eq 0 ] || report FAIL 'AP 비밀번호 값이 든 파일' "${ap_hits[@]}"
fi

# 부트 파티션(vfat)은 고쳐 쓴 옛 클러스터가 빈 자리로 남고, 이미지 도구는 p1을 지우지 않는다: 원시 장치에서 센다
if [ ${#ROUTER_VALS[@]} -gt 0 ]; then
	BOOT_DEV="${CHECK_BOOT_DEVICE:-}"
	[ -n "$BOOT_DEV" ] || [ -n "$ROOT" ] || BOOT_DEV=$(findmnt -no SOURCE /boot/firmware 2>/dev/null)
	ITEM='부트 파티션 빈 자리의 공유기 값'
	N=''
	if [ -n "$BOOT_DEV" ] && [ -r "$BOOT_DEV" ]; then
		N=$(grep -caF -f <(printf '%s\n' "${ROUTER_VALS[@]}") "$BOOT_DEV" 2>/dev/null)
		[ $? -le 1 ] || N=''
	fi
	if [ -z "$N" ]; then
		say WARN "$ITEM" "장치를 읽지 못했습니다 (${BOOT_DEV:-장치를 모름}) — 직접 확인할 것"
	elif [ "$N" -gt 0 ]; then
		say FAIL "$ITEM" "${BOOT_DEV#"$ROOT"} 의 $N 곳 (빈 공간을 0으로 채운다: 구현계획서 P7.2)"
	else
		say OK "$ITEM" '없음'
	fi
fi

mapfile -t found < <(grep -rlsaE 'access-points|wifis' "$ROOT/var/lib/cloud")
report FAIL 'cloud-init에 남은 와이파이 설정 사본' "${found[@]}"

mapfile -t found < <(find "$ROOT/var/log" "$ROOT/var/crash" "$ROOT/var/lib/systemd/coredump" -type f \
	\( -name '*.gz' -o -name '*.xz' -o -name '*.zst' -o -name '*.lz4' -o -name '*.crash' \) 2>/dev/null)
report FAIL '압축 로그 · 크래시 덤프 (속을 검사할 수 없다)' "${found[@]}"

mapfile -t found < <(grep -rlsaE '(run_wifi|set_ap|set_wifi)\.sh [^ ]+ [^ ]+|nm_kf\.py set .*psk ' "$ROOT/var/log")
report FAIL '로그에 남은 비밀번호 명령줄' "${found[@]}"

mapfile -t found < <(grep -rlsE '^[[:space:]]*wifis:' "$ROOT/etc/netplan" "$ROOT/boot/firmware/network-config")
report FAIL '옛 와이파이 설정 (wifis:)' "${found[@]}"

mapfile -t found < <(existing "$H"/set_wifi.sh* "$H"/set_ip.sh* "$H"/set_dhcp.sh* "$H/clone_init.sh" \
	"$ROOT/etc/networkd-dispatcher/routable.d/50-kit-ip" "$ROOT/etc/systemd/system/lekiwi_host.service")
report FAIL '옛 도구 (netplan 세대 · 호스트 실행)' "${found[@]}"

if [ -d "$H/lekiwi_ws/install" ]; then
	mapfile -t found < <(find -L "$H/lekiwi_ws/install" -type l 2>/dev/null)   # -L 에서 링크로 남는 것 = 대상이 없다
	report FAIL '깨진 링크 (~/lekiwi_ws/install → build)' "${found[@]}"
else
	say FAIL '깨진 링크 (~/lekiwi_ws/install → build)' "$HOME_DIR/lekiwi_ws/install 이 없습니다"
fi

mapfile -t found < <(
	{
		existing "$H/.vscode-server" "$H"/.claude* "$ROOT/root/.vscode-server" "$ROOT"/root/.claude*
		find "$H" -mindepth 1 \( -path "$H/.vscode-server" -o -path "$H/.cache" -o -path "$H/lerobot060_venv" \) -prune \
			-o -name .omc -print -prune 2>/dev/null
		find "$H/.ssh" "$ROOT/root/.ssh" -maxdepth 1 -type f -name 'id_*' ! -name '*.pub' 2>/dev/null
		# 이름이 id_* 가 아닌 개인키도 머리글로, 시스템 폴더까지 (복제한 로봇이 모두 같은 키를 갖는다. 예: fwupd의 secret.key).
		# PEM 키는 글자 파일이라 -I. 호스트키 /etc/ssh/ssh_host_*는 이미지 도구가 지우고 첫 부팅에 새로 만든다
		grep -rlsIE "${SKIP_DIRS[@]}" -e '-----BEGIN [A-Z0-9 ]*PRIVATE KEY-----' "${SYS_SCAN[@]}" "${HOME_SCAN[@]}" \
			| while IFS= read -r f; do case "$f" in "$ROOT"/etc/ssh/ssh_host_*) ;; *) printf '%s\n' "$f" ;; esac; done
	} | sort -u
)
report FAIL '개발 도구 흔적 · SSH 개인키' "${found[@]}"

mapfile -t found < <(for home in "$ROOT/root" "$ROOT"/home/*; do
	existing "$home/.cache/huggingface/token" "$home/.huggingface/token" "$home/.git-credentials" "$home/.netrc" \
		"$home/.config/gh/hosts.yml"
done)
report FAIL '토큰 · 자격 증명 파일' "${found[@]}"

mapfile -t found < <(existing "$ROOT/run/lekiwi-ap-password")
report FAIL 'AP 임시 비밀번호 파일' "${found[@]}"

mapfile -t found < <(existing "$ROOT/etc/NetworkManager/conf.d/90-lekiwi-hold.conf")
report FAIL 'NetworkManager 붙잡는 파일 (남으면 와이파이 · AP가 뜨지 않는다)' "${found[@]}"

NM_LINK="$ROOT/etc/systemd/system/multi-user.target.wants/NetworkManager.service"
if [ -L "$NM_LINK" ]; then
	say OK 'NetworkManager 부팅 때 켜짐' '켜짐'
else
	say FAIL 'NetworkManager 부팅 때 켜짐' "${NM_LINK#"$ROOT"} 링크가 없다 (systemctl enable NetworkManager)"
fi

# --- WARN: 알고 남기는 것 (이유를 기록한다)
mapfile -t found < <(existing "$HOME_KF")
report WARN 'home-wifi 프로필 (이미지 도구가 복사본에서 지운다)' "${found[@]}"

report WARN 'cloud-init 로그인 설정 (password:는 계정 비밀번호, 공유기 것이 아니다)' "${LOGIN_FILES[@]}"

HOST=$(cat "$ROOT/etc/hostname" 2>/dev/null || true)
mapfile -t found < <(
	existing "$H/lekiwi_ws/robot_profile.yaml" "$H/lekiwi_ws/arm_profile.yaml" "$H/lekiwi_ws/arm_calibration_restore.json"
	find "$H/.cache/huggingface/lerobot/calibration" -type f -name '*.json' 2>/dev/null
	nonempty "$H/maps"
	[[ $HOST =~ ^lekiwi[0-9][0-9]$ ]] || echo "호스트 이름 '$HOST' 이 lekiwiNN 꼴이 아니다"
	[ -f "$AP_KF" ] || echo "lekiwi-ap 프로필이 없다"
)
report WARN '로봇별 값 (카드별 초기화가 덮어쓴다)' "${found[@]}"

mapfile -t found < <(
	if [ -f "$AP_KF" ]; then
		AUTO=$(sed -n 's/^autoconnect=//p' "$AP_KF" | head -n 1)
		BAND=$(sed -n 's/^band=//p' "$AP_KF" | head -n 1)
		[ "${AUTO:-true}" = true ] || echo "autoconnect=$AUTO (true여야 공유기가 없을 때 AP가 뜬다)"
		[ "$BAND" = bg ] || echo "band=${BAND:-없음} (기본은 bg · 2.4 GHz, 채널 1 · 6 · 11)"
	fi
)
report WARN 'AP 프로필 설정' "${found[@]}"

# 처음 AP는 모든 로봇이 lekiwi01 / lekiwi01 (사용자 결정, R25). 비밀번호는 같은지만 본다
mapfile -t found < <(
	if [ -f "$AP_KF" ]; then
		[ "$AP_NAME" = lekiwi01 ] || echo "이름이 '$AP_NAME'"
		[ "$AP_PSK_RAW" = lekiwi01 ] || echo "비밀번호가 lekiwi01이 아니다"
	fi
)
report WARN '처음 AP 기본값이 아님 (출고 카드는 lekiwi01 / lekiwi01)' "${found[@]}"

mapfile -t found < <(for f in "$ROOT"/root/.ssh/authorized_keys "$ROOT"/home/*/.ssh/authorized_keys; do
	[ -f "$f" ] || continue
	awk -v path="${f#"$ROOT"}" '
		/^[[:space:]]*(#|$)/ { next }
		{
			total++; c = ""; body = ""
			for (i = 1; i <= NF; i++) if ($i ~ /^(ssh-|ecdsa-|sk-)/) {
				body = $(i + 1)
				for (j = i + 2; j <= NF; j++) c = c (c == "" ? "" : " ") $j
				break
			}
			if (c == "physical-labs-lekiwi") {     # 앱 설치 키: 본문으로 겹침을 센다 (본문은 찍지 않는다)
				app++
				if (!(body in appkey)) { appkey[body] = 1; distinct++ }
			} else {
				personal++
				if (!(body in mine)) { mine[body] = 1; list = list (list == "" ? "" : ", ") (c == "" ? "(주석 없음)" : c) }
			}
		}
		END {
			if (personal) {
				dup = (distinct != app) ? sprintf(" (서로 다른 키 %d개)", distinct) : ""
				printf "%s: 키 %d개 = 앱 설치 %d%s + 개인 %d (%s) — 이 키 소유자는 이 이미지를 받은 모든 로봇에 root로 들어올 수 있다\n", \
					path, total, app, dup, personal, list
			}
		}
	' "$f"
done)
report WARN 'authorized_keys의 개인 키 (R23)' "${found[@]}"

mapfile -t found < <(
	find "$ROOT/var/log" -path "$ROOT/var/log/journal" -prune -o -type f -size +1M \
		! -name '*.gz' ! -name '*.[0-9]' ! -name '*.old' -print 2>/dev/null
	nonempty "$ROOT/var/log/journal" "$H/.ros/log" "$H/lekiwi_ws/log"
)
report WARN '로그 (1 MB 넘는 지금 로그 · journal · 실행 기록)' "${found[@]}"

mapfile -t found < <(for f in "$H/.bash_history" "$ROOT/root/.bash_history" "$H/.python_history"; do
	[ ! -s "$f" ] || printf '%s\n' "$f"
done)
report WARN '셸 기록 (이미지 도구가 복사본에서 지우지만 비밀번호가 든 명령이 있을 수 있다)' "${found[@]}"

mapfile -t found < <(
	[ -z "$(find "$ROOT/var/lib/apt/lists" -maxdepth 1 -type f ! -name lock -print -quit 2>/dev/null)" ] \
		|| echo "$ROOT/var/lib/apt/lists"
	[ -z "$(find "$ROOT/var/cache/apt/archives" -maxdepth 1 -type f -name '*.deb' -print -quit 2>/dev/null)" ] \
		|| echo "$ROOT/var/cache/apt/archives/*.deb"
)
report WARN 'apt 목록 · 캐시' "${found[@]}"

mapfile -t found < <(existing "$ROOT"/var/lib/NetworkManager/*.leases "$ROOT/var/lib/NetworkManager/secret_key")
report WARN 'NetworkManager 상태 (AP 임대 기록 · secret_key)' "${found[@]}"

# --- 정보
say INFO '사용량' "$(df -h "$ROOT/" 2>/dev/null | awk 'NR == 2 { print $3 " / " $2 " (" $5 ")" }')"
say INFO '큰 폴더 (MB)' "$(du -xsm "$H"/*/ "$H"/.[!.]*/ "$ROOT/var" "$ROOT/usr" "$ROOT/opt" 2>/dev/null | sort -rn \
	| head -n 5 | awk -v root="$ROOT" '{ sub("^" root, "", $2); sub("/$", "", $2); printf "%s %s  ", $2, $1 }')"
say INFO '프로필' "$(nmcli -t -f NAME,TYPE con show 2>/dev/null | tr '\n' ' ')"
say INFO '이름' "호스트 ${HOST:-?} · kit-number $(cat "$ROOT/etc/kit-number" 2>/dev/null || echo '?') · AP ${AP_NAME:-?}"

echo "요약: FAIL $NFAIL · WARN $NWARN"
[ "$NFAIL" -eq 0 ]
