#!/bin/bash
# 복제한 카드마다 한 번: 이 카드의 호스트 이름 · 킷 번호 · AP 채널 · 공유기 고정 주소를 정한다
# (NetworkManager 세대, ~/lekiwi_net 에 둔다)
#   sudo ~/lekiwi_net/clone_init.sh <호스트 이름> <킷 번호 1~54> [--ap-ssid NAME] [--ap-password PW]
#     예) sudo ~/lekiwi_net/clone_init.sh lekiwi07 7
# - 호스트 이름은 lekiwi%02d(킷 번호)여야 한다. AP 채널 = (1, 6, 11)[킷 번호 % 3]
# - AP 이름 · 비밀번호는 건드리지 않는다: 처음 AP는 모든 로봇이 lekiwi01 / lekiwi01 (사용자 결정, 고객이 바꾼다).
#   --ap-ssid · --ap-password를 줄 때만 그것을 쓴다 (비밀번호는 sudo 명령줄에 남지 않게 nm_kf.py로 직접)
# - home-wifi 프로필이 있으면 공유기 고정 주소를 192.168.0.(200+킷 번호)로 (set_ip.sh)
# - SSH 호스트키 · machine-id는 이미지의 첫 부팅 서비스가 새로 만든다 (여기서는 하지 않는다)
# 종료: 0 성공 / 1 인자 오류 또는 실패한 단계.
# 시험용 환경변수: CLONE_ROOT (모든 절대 경로 앞에 붙는다. 있으면 root 확인을 건너뛴다)
set -euo pipefail

ROOT="${CLONE_ROOT:-}"
D=$(dirname "$(readlink -f "$0")")
export NM_KF_DIR="$ROOT/etc/NetworkManager/system-connections" LEKIWI_NET_STATE_DIR="$ROOT/var/lib/lekiwi-net"

bad() { echo "error: $*" >&2; exit 1; }
usage() { bad "사용법: sudo $0 <호스트 이름 lekiwiNN> <킷 번호 1~54> [--ap-ssid NAME] [--ap-password PW]"; }

[ $# -ge 2 ] || usage
NEW_HOSTNAME="$1"
KIT="$2"
shift 2
AP_SSID=''
AP_PW=''
SET_SSID=''
SET_PW=''
while [ $# -gt 0 ]; do
	[ $# -ge 2 ] || usage
	case "$1" in
	--ap-ssid) AP_SSID="$2"; SET_SSID=1 ;;
	--ap-password) AP_PW="$2"; SET_PW=1 ;;
	*) usage ;;
	esac
	shift 2
done
case "$KIT" in ''|*[!0-9]*) usage ;; esac
KIT=$((10#$KIT))
{ [ "$KIT" -ge 1 ] && [ "$KIT" -le 54 ]; } || bad "킷 번호는 1~54 입니다"
WANT_NAME=$(printf 'lekiwi%02d' "$KIT")
[ "$NEW_HOSTNAME" = "$WANT_NAME" ] || bad "호스트 이름은 $WANT_NAME 이어야 합니다 (킷 번호 $KIT)"
if [ -n "$SET_SSID" ]; then
	N=$(printf %s "$AP_SSID" | wc -c)
	{ [ "$N" -ge 1 ] && [ "$N" -le 32 ]; } || bad "AP 이름은 1~32바이트입니다"
	[[ ! $AP_SSID =~ [[:cntrl:]] && ! $AP_SSID =~ ^[[:space:]]|[[:space:]]$ ]] || bad "AP 이름에 제어 문자 · 앞뒤 공백을 둘 수 없습니다"
	case "$AP_SSID" in *\;*) bad "AP 이름에 ; 를 쓸 수 없습니다 (keyfile의 ssid=가 목록으로 읽는다)" ;; esac
fi
if [ -n "$SET_PW" ]; then
	case "$AP_PW" in *[!\ -~]*) bad "AP 비밀번호는 인쇄 가능한 ASCII만 씁니다 (WPA2)" ;; esac
	{ [ ${#AP_PW} -ge 8 ] && [ ${#AP_PW} -le 63 ]; } || bad "AP 비밀번호는 8~63자입니다 (WPA2)"
	case "$AP_PW" in ' '*|*' ') bad "AP 비밀번호 앞뒤에 공백을 둘 수 없습니다" ;; esac
fi
if [ -n "$ROOT" ]; then     # 가짜 루트는 시험용: root로 돌면 잘못 붙은 경로가 실제 /etc를 바꿀 수 있다
	[ "$(id -u)" -ne 0 ] || bad "CLONE_ROOT는 시험용입니다 (root로는 쓰지 않는다)"
else
	[ "$(id -u)" -eq 0 ] || bad "sudo로 실행하세요"
fi
for f in set_ip.sh nm_kf.py; do [ -x "$D/$f" ] || bad "$D/$f 가 없습니다 (~/lekiwi_net 에서 실행)"; done
[ -f "$NM_KF_DIR/lekiwi-ap.nmconnection" ] || bad "lekiwi-ap 프로필이 없습니다 (NetworkManager 세대 카드가 아님)"

echo "[clone_init] 호스트 이름: $NEW_HOSTNAME (킷 $KIT)"
# cloud-init이 부팅마다 user-data의 호스트 이름으로 되돌리지 않게 (옛 clone_init.sh와 같다)
[ ! -d "$ROOT/etc/cloud/cloud.cfg.d" ] || echo 'preserve_hostname: true' > "$ROOT/etc/cloud/cloud.cfg.d/99_preserve_hostname.cfg"
printf '%s\n' "$NEW_HOSTNAME" > "$ROOT/etc/hostname"
hostnamectl set-hostname "$NEW_HOSTNAME"
if grep -q '^127.0.1.1' "$ROOT/etc/hosts"; then
	sed -i "s/^127.0.1.1.*/127.0.1.1 $NEW_HOSTNAME/" "$ROOT/etc/hosts"
else
	echo "127.0.1.1 $NEW_HOSTNAME" >> "$ROOT/etc/hosts"
fi
printf '%s\n' "$KIT" > "$ROOT/etc/kit-number"

CHANNELS=(1 6 11)
CHANNEL=${CHANNELS[KIT % 3]}     # 옆 카드와 채널이 겹치지 않게
AP_SET=(wifi band bg wifi channel "$CHANNEL")
[ -z "$SET_SSID" ] || AP_SET+=(wifi ssid "$AP_SSID")
# set_ap.sh는 안에서 sudo로 nm_kf.py를 불러 비밀번호가 auth.log에 남는다: 이미 root이니 직접 쓴다
[ -z "$SET_PW" ] || AP_SET+=(wifi-security psk "$AP_PW")
"$D/nm_kf.py" set lekiwi-ap "${AP_SET[@]}"
nmcli con load "$NM_KF_DIR/lekiwi-ap.nmconnection"
ACTIVE=$(nmcli -t -f NAME con show --active 2>/dev/null || true)
if grep -qx lekiwi-ap <<<"$ACTIVE"; then     # 켜져 있던 AP는 새 설정으로 다시 올린다 (set_ap.sh와 같다)
	nmcli con up lekiwi-ap >/dev/null
	echo "[clone_init] AP를 다시 올렸습니다 (AP로 붙어 있었다면 다시 연결하세요)"
fi
echo "[clone_init] AP: 이름 $("$D/nm_kf.py" get lekiwi-ap wifi ssid) · 2.4 GHz 채널 $CHANNEL${SET_PW:+ · 비밀번호 바꿈}"

RC=0
if [ -f "$NM_KF_DIR/home-wifi.nmconnection" ]; then
	"$D/set_ip.sh" "$KIT" || { RC=$?; echo "[clone_init] 공유기 고정 주소를 바꾸지 못했습니다 (set_ip.sh 종료 $RC)" >&2; }
fi
rm -rf "$LEKIWI_NET_STATE_DIR"     # 공유기 설정 사본 자리 (이미지 도구가 검사하지 않는다)
echo "[clone_init] 끝. 재부팅하면 모든 서비스가 새 이름을 씁니다: sudo reboot"
exit "$RC"
