#!/bin/bash
# 로봇(netplan 세대)의 wlan0을 NetworkManager 세대(~/lekiwi_net 스크립트)로 옮긴다. eth0의 설정 · 주소는 그대로 두지만
# netplan apply가 eth0의 DHCP를 다시 한다 (유선 ssh가 잠깐 끊길 수 있어 setsid nohup으로 돌린다, 구현계획서 P2.4).
#   sudo provision_ap_mode.sh [--dry-run] [--mode wifi|ap] [--ap-ssid NAME] [--kit N | --dhcp] [--hold-nm]
#     --mode wifi  공유기 우선 (못 붙으면 AP로 뜬다, 기본) / --mode ap  AP 고정
#     --ap-ssid    처음 AP 이름이자 비밀번호 (기본 lekiwi01: 모든 로봇이 같고 고객이 바꾼다, 8~32자 ASCII)
#     --kit N      공유기 고정 주소 192.168.0.(200+N) (기본: /etc/kit-number) / --dhcp  공유기에서 DHCP
#     --hold-nm    apt-mark hold network-manager
# 종료: 0 성공 · 이미 적용됨 / 3 사전 점검에서 거절 (아무것도 안 바뀜) / 4 바꾼 뒤 실패 (롤백함)
#   / 5 바꾼 뒤 실패했고 롤백도 다 되지 않음 (유선으로 들어와 고친다). 신호(INT · TERM · HUP)로 멈춰도 롤백한다.
# 백업 /var/backups/lekiwi_net_*.tar.gz 에는 공유기 비밀번호가 있다: PC로 옮겨 sha256을 맞춘 뒤 로봇에서 지운다.
# 처음 AP는 이름 = 비밀번호 = lekiwi01 (사용자 결정: 공개된 기본값이라 숨길 것이 없다, 기획서 R25).
# 시험용 환경변수: PROVISION_ROOT (모든 절대 경로 앞에 붙는다. 있으면 root 확인을 건너뛴다),
#   PROVISION_USER (기본 $SUDO_USER), PROVISION_HOME (기본 그 사용자의 홈)
set -euo pipefail
umask 077

ROOT="${PROVISION_ROOT:-}"
HERE=$(dirname "$(readlink -f "$0")")
NET_SCRIPTS=(nm_kf.py run_ap.sh run_wifi.sh set_ap.sh net_status.sh set_ip.sh set_dhcp.sh _apply_home_ipv4.sh)
LEGACY=(set_wifi.sh set_ip.sh set_dhcp.sh)
NETPLAN="$ROOT/etc/netplan/50-cloud-init.yaml"
NETCFG="$ROOT/boot/firmware/network-config"
KFDIR="$ROOT/etc/NetworkManager/system-connections"
# NetworkManager가 장치를 잡지 못하게 붙잡는 파일. wlan0만이 아니라 모든 장치(*)를 붙잡아 이름에 wlan0이 없다.
# 같은 conf.d 폴더 안에서는 이름순으로 나중 파일이 이기므로 90-으로 다른 조각보다 뒤에 읽히게 한다.
HOLD="$ROOT/etc/NetworkManager/conf.d/90-lekiwi-hold.conf"
MARK="$ROOT/etc/lekiwi-net-provisioned"
# ~/lekiwi_net 스크립트도 같은 트리를 보게 한다 (PROVISION_ROOT가 없으면 원래 경로와 같다)
export NM_KF_DIR="$KFDIR" LEKIWI_NET_STATE_DIR="$ROOT/var/lib/lekiwi-net"

CHANGED=''
BACKUP=''
TIMER_FIRED=''
refuse() { echo "[provision] 거절: $*" >&2; exit 3; }
fail() { echo "[provision] 실패: $*" >&2; exit 4; }
on_exit() {
	local rc=$?
	[ "$rc" -ne 0 ] || return 0
	if [ -n "$CHANGED" ]; then
		trap '' INT TERM HUP     # 되돌리는 동안 두 번째 신호(ssh가 끊겨 오는 HUP 등)로 멈추지 않게
		local why=$rc     # 130이면 신호로 멈춘 것
		rc=4
		if [ -n "$TIMER_FIRED" ]; then     # 타이머의 롤백이 맡았다: 겹쳐 돌리지 않는다
			echo "[provision] 되돌림 타이머의 롤백이 맡았습니다 (journalctl -u lekiwi-net-rollback)" >&2
		else
			echo "[provision] 종료 $why -> 백업으로 되돌립니다" >&2
			if ! PROVISION_ROOT="$ROOT" PROVISION_HOME="$HOME_DIR" "$HERE/rollback_ap_mode.sh" "$BACKUP"; then
				echo "[provision] 롤백도 실패했습니다 — 유선(eth0)으로 들어와 위 [rollback] 줄을 보고 고치세요" >&2
				rc=5
			fi
		fi
		echo "[provision] 백업 ${BACKUP#"$ROOT"} 에 공유기 비밀번호가 있습니다: PC로 옮긴 뒤 로봇에서 지우세요 (P2.8)" >&2
		exit "$rc"
	fi
	[ -z "$BACKUP" ] || rm -f "$BACKUP"     # 만들다 만 백업
	exit 3
}
trap on_exit EXIT
trap 'exit 130' INT TERM HUP     # 신호로 죽으면 $?가 0이라 on_exit가 롤백하지 않는다

put() {   # put MODE FILE < 내용: 같은 폴더의 임시 파일에 쓴 뒤 바꿔 끼운다 (끊겨도 반쪽 파일이 남지 않는다)
	cat > "$2.lekiwi-tmp"
	chmod "$1" "$2.lekiwi-tmp"
	mv -f "$2.lekiwi-tmp" "$2"
}
strip_wifis() {   # strip_wifis FILE: `wifis:` 줄과 그보다 깊이 들여쓴 줄 · 빈 줄을 뺀 내용 (PyYAML 없이)
	awk '
		{ s = $0; sub(/^ +/, "", s); ind = length($0) - length(s) }
		skip && (s == "" || ind > depth) { next }
		{ skip = 0 }
		s ~ /^wifis:/ { skip = 1; depth = ind; next }
		{ print }
	' "$1"
}
new_uuid() { python3 -c 'import uuid; print(uuid.uuid4())'; }

DRY=''; MODE=wifi; AP_SSID=lekiwi01; KIT=''; DHCP=''; HOLD_NM=''
while [ $# -gt 0 ]; do
	case "$1" in
	--dry-run) DRY=1 ;;
	--mode|--ap-ssid|--kit)
		[ $# -ge 2 ] || refuse "$1 에 값이 필요합니다"
		case "$1" in
		--mode) MODE="$2" ;;
		--ap-ssid) AP_SSID="$2" ;;
		--kit) KIT="$2" ;;
		esac
		shift ;;
	--dhcp) DHCP=1 ;;
	--hold-nm) HOLD_NM=1 ;;
	-h|--help) sed -n '2,12p' "$0"; exit 0 ;;
	*) refuse "모르는 인자: $1 (--help)" ;;
	esac
	shift
done
case "$MODE" in wifi|ap) ;; *) refuse "--mode 는 wifi 또는 ap 입니다" ;; esac
[ -z "$DHCP" ] || [ -z "$KIT" ] || refuse "--kit 과 --dhcp 는 함께 쓸 수 없습니다"

# 1. 사전 점검 (읽기만)
if [ -e "$MARK" ]; then
	echo "[provision] 이미 적용됨 (${MARK#"$ROOT"}) — 아무것도 하지 않습니다"
	exit 0
fi
if [ -n "$ROOT" ]; then     # 가짜 루트는 시험용: root로 돌면 잘못 붙은 경로가 실제 /etc를 바꿀 수 있다
	[ "$(id -u)" -ne 0 ] || refuse "PROVISION_ROOT는 시험용입니다 (root로는 쓰지 않는다)"
else
	[ "$(id -u)" -eq 0 ] || refuse "root로 실행하세요 (sudo)"
fi
USER_NAME="${PROVISION_USER:-${SUDO_USER:-}}"
{ [ -n "$USER_NAME" ] && [ "$USER_NAME" != root ]; } || refuse "대상 사용자를 모릅니다 (sudo로 실행하거나 PROVISION_USER)"
HOME_DIR="${PROVISION_HOME:-$(getent passwd "$USER_NAME" | cut -d: -f6 || true)}"
H="$ROOT$HOME_DIR"
{ [ -n "$HOME_DIR" ] && [ -d "$H" ]; } || refuse "홈 폴더가 없습니다: ${HOME_DIR:-?}"
for f in "${NET_SCRIPTS[@]}" bashrc_net.sh rollback_ap_mode.sh; do
	[ -f "$HERE/$f" ] || refuse "$HERE/$f 가 없습니다"
done
if [ -z "$DHCP" ]; then
	[ -n "$KIT" ] || [ ! -f "$ROOT/etc/kit-number" ] || KIT=$(tr -d '[:space:]' < "$ROOT/etc/kit-number")
	case "$KIT" in ''|*[!0-9]*) refuse "킷 번호를 모릅니다 (--kit 1~54 또는 --dhcp)" ;; esac
	KIT=$((10#$KIT))
	{ [ "$KIT" -ge 1 ] && [ "$KIT" -le 54 ]; } || refuse "킷 번호는 1~54 입니다 (받은 값 $KIT)"
fi
# 처음 AP 이름이 곧 비밀번호다: 이름 규칙(32바이트까지)과 WPA2 비밀번호 규칙(인쇄 가능한 ASCII 8자 이상)을 함께
case "$AP_SSID" in *[!\ -~]*) refuse "AP 이름(= 처음 비밀번호)은 인쇄 가능한 ASCII만 씁니다 (--ap-ssid)" ;; esac
if [ ${#AP_SSID} -lt 8 ] || [ ${#AP_SSID} -gt 32 ] || [[ $AP_SSID =~ ^[[:space:]]|[[:space:]]$ ]]; then
	refuse "AP 이름(= 처음 비밀번호)은 8~32자, 앞뒤 공백 없이 (--ap-ssid)"
fi
# keyfile의 ssid=는 ;를 목록 구분자로 본다 (nm_kf.py는 \ 만 두 배로 쓴다)
case "$AP_SSID" in *\;*) refuse "AP 이름에 ; 를 쓸 수 없습니다 (--ap-ssid)" ;; esac
ETH0_ADDR=$(ip -4 -o addr show dev eth0 2>/dev/null | awk '{print $4; exit}' || true)
[ -n "$ETH0_ADDR" ] || refuse "eth0에 IPv4 주소가 없습니다 (랜선으로 PC에 닿을 때만 옮깁니다)"
LINK=$(ip -o link show dev eth0 2>/dev/null || true)
[[ $LINK == *LOWER_UP* ]] || refuse "eth0 케이블이 연결되어 있지 않습니다 (랜선으로 PC에 닿을 때만 옮깁니다)"
[ -f "$NETPLAN" ] || refuse "netplan 파일이 없습니다: ${NETPLAN#"$ROOT"}"
grep -qE '^ *wifis:' "$NETPLAN" || refuse "netplan에 wifis가 없습니다 (이미 옮겼거나 모르는 구성)"
NEW_NETPLAN=$(strip_wifis "$NETPLAN")
grep -qE '^ +eth0:' <<<"$NEW_NETPLAN" || refuse "wifis를 지운 netplan에 eth0이 남지 않습니다 (모르는 모양)"
LEFTOVER='^[[:space:]]*(password:|access-points:)'     # wifis 블록 밖에도 와이파이 설정이 있다 = 모르는 모양
! grep -qE "$LEFTOVER" <<<"$NEW_NETPLAN" || refuse "wifis를 지운 netplan에 비밀번호 · access-points 줄이 남습니다 (모르는 모양)"
NETCFG_EDIT=''
if [ -f "$NETCFG" ] && grep -qE '^ *wifis:' "$NETCFG"; then
	NETCFG_EDIT=1
	NEW_NETCFG=$(strip_wifis "$NETCFG")
	! grep -qE "$LEFTOVER" <<<"$NEW_NETCFG" || refuse "wifis를 지운 network-config에 비밀번호 · access-points 줄이 남습니다 (모르는 모양)"
fi
# 공유기 이름 · 비밀번호: set_wifi.sh가 쓴 꼴 그대로 ("이름": / password: "값"). 값은 어디에도 찍지 않는다.
PW_LINE=$(grep -E '^[[:space:]]*password:' "$NETPLAN" || true)
SSID_LINE=$(awk '/^[[:space:]]*access-points:/ { found = 1; next } found && NF && $1 !~ /^#/ { print; exit }' "$NETPLAN")
[ "$(grep -c . <<<"$PW_LINE")" = 1 ] || refuse "netplan에서 공유기 비밀번호 줄이 하나가 아닙니다"
[[ $SSID_LINE =~ ^[[:space:]]*\".*\":[[:space:]]*$ ]] || refuse "netplan에서 공유기 이름을 읽지 못했습니다 (\"이름\": 꼴이 아님)"
[[ $PW_LINE =~ ^[[:space:]]*password:[[:space:]]*\".*\"[[:space:]]*$ ]] || refuse "netplan에서 공유기 비밀번호를 읽지 못했습니다 (큰따옴표 꼴이 아님)"
ROUTER_SSID=${SSID_LINE#*\"}; ROUTER_SSID=${ROUTER_SSID%\"*}
ROUTER_PW=${PW_LINE#*\"}; ROUTER_PW=${ROUTER_PW%\"*}
{ [ -n "$ROUTER_SSID" ] && [ -n "$ROUTER_PW" ]; } || refuse "netplan의 공유기 이름이나 비밀번호가 비어 있습니다"
case "$ROUTER_SSID$ROUTER_PW" in *\\*) refuse "공유기 이름이나 비밀번호에 \\ 가 있어 옮기지 않습니다 (YAML 이스케이프)" ;; esac
case "$ROUTER_SSID" in *\;*) refuse "공유기 이름에 ; 가 있어 옮기지 않습니다 (keyfile의 ssid=가 목록으로 읽는다)" ;; esac
if [[ $ROUTER_SSID =~ ^[[:space:]]|[[:space:]]$ ]] || [[ $ROUTER_PW =~ ^[[:space:]]|[[:space:]]$ ]]; then
	refuse "공유기 이름이나 비밀번호 앞뒤에 공백이 있어 옮기지 않습니다 (keyfile은 앞뒤 공백을 잘라 읽는다)"
fi

BACKUP_NAME="lekiwi_net_$(date +%Y%m%d-%H%M%S).tar.gz"
WANT=home-wifi; [ "$MODE" = wifi ] || WANT=lekiwi-ap
if [ -n "$DHCP" ]; then IP_DESC='DHCP'; else IP_DESC="고정 192.168.0.$((200 + KIT))"; fi
STEPS=(
	"사전 점검: 통과 (eth0 $ETH0_ADDR, netplan에서 공유기 정보를 읽음)"
	"백업: /var/backups/$BACKUP_NAME (root 600, 공유기 비밀번호 포함: PC로 옮긴 뒤 지운다)"
	"NetworkManager 설치 (Recommends 없이 + dnsmasq-base): 먼저 ${HOLD#"$ROOT"} 로 모든 장치를 붙잡아 둔다${HOLD_NM:+ (apt-mark hold)}"
	"스크립트 · 설정 설치: $HOME_DIR/lekiwi_net, 국가코드 KR, 절전 끔, .bashrc 블록"
	"keyfile: lekiwi-ap (이름 = 비밀번호 = $AP_SSID: 처음 기본값, 고객이 바꾼다), home-wifi ($IP_DESC)"
	"되돌림 타이머: 180초 뒤 rollback_ap_mode.sh (멈추기: sudo systemctl stop lekiwi-net-rollback.timer)"
	"옮기기: netplan의 wifis 삭제 → netplan apply → netplan-wpa-wlan0 정지 → NetworkManager (wlan0을 맡을 때까지 20초) → run_$MODE.sh"
	"옛 도구 막기: set_wifi.sh · set_ip.sh · set_dhcp.sh → *.legacy-netplan, 50-kit-ip 삭제, network-config의 wifis 삭제, wifi-powersave-off 끔"
	"확인: 60초 안에 eth0 주소 그대로 · wlan0이 $WANT 로 activated"
)
step() { echo "[provision] $1/9 ${STEPS[$1 - 1]}"; }
if [ -n "$DRY" ]; then
	for i in 1 2 3 4 5 6 7 8 9; do step "$i"; done
	echo "DRY-RUN: 변경 없음"
	exit 0
fi
step 1

step 2
BACKUP="$ROOT/var/backups/$BACKUP_NAME"
MEMBERS=()
for rel in etc/netplan boot/firmware/network-config etc/networkd-dispatcher/routable.d/50-kit-ip \
	etc/systemd/system/wifi-powersave-off.service etc/kit-number etc/modprobe.d/cfg80211-kr.conf \
	"${HOME_DIR#/}"/{set_wifi.sh,set_ip.sh,set_dhcp.sh,wifi_list.sh,clone_init.sh,.bashrc,lekiwi_net}; do
	[ ! -e "$ROOT/$rel" ] || MEMBERS+=("$rel")
done
for link in "$ROOT"/etc/systemd/system/*.wants/wifi-powersave-off.service; do   # 켜 둔 상태도 되돌리게
	[ ! -L "$link" ] || MEMBERS+=("${link#"$ROOT"/}")
done
mkdir -p "$ROOT/var/backups"
tar -czf "$BACKUP" -C "$ROOT/" "${MEMBERS[@]}"
chmod 600 "$BACKUP"

step 3
CHANGED=1
install -d -m 755 "${HOLD%/*}"     # umask 077이라 mkdir -p면 /etc/NetworkManager · conf.d가 700이 된다
# 설치하자마자 NetworkManager가 wlan0을 잡지 않게. interface-name:wlan0 만 쓰면 Ubuntu 기본값
# (*,except:type:wifi,...)을 덮어써 오히려 eth0을 맡게 되므로 모든 장치를 붙잡는다.
put 644 "$HOLD" <<'EOF'
[keyfile]
unmanaged-devices=*
EOF
# Recommends(ppp · pptp 등)는 빼고, AP 공유 모드의 DHCP에 꼭 필요한 dnsmasq-base만 더한다.
# 패키지 스크립트가 만드는 파일은 보통 권한으로(umask 022), needrestart는 서비스를 다시 켜지 않게
( umask 022; NEEDRESTART_SUSPEND=1 DEBIAN_FRONTEND=noninteractive \
	apt-get install -y --no-install-recommends network-manager dnsmasq-base )
[ -z "$HOLD_NM" ] || apt-mark hold network-manager
systemctl enable --now NetworkManager     # 앞서 롤백으로 꺼 두었을 때를 위해
for _ in 1 2 3 4 5; do
	DEVS=$(nmcli -t -f DEVICE,STATE dev 2>/dev/null || true)
	if grep -qx 'wlan0:unmanaged' <<<"$DEVS"; then break; fi
	sleep 1
done
grep -qx 'wlan0:unmanaged' <<<"$DEVS" || fail "NetworkManager가 wlan0을 잡았습니다 (붙잡는 파일이 듣지 않음)"
ETH0_NM=$(awk -F: '$1 == "eth0" { print $2 }' <<<"$DEVS")
[ "${ETH0_NM:-unmanaged}" = unmanaged ] || fail "NetworkManager가 eth0을 잡았습니다 ($ETH0_NM)"

step 4
install -d -m 755 "$H/lekiwi_net"
for f in "${NET_SCRIPTS[@]}"; do install -m 755 "$HERE/$f" "$H/lekiwi_net/$f"; done
chown -R "$USER_NAME:" "$H/lekiwi_net"
mkdir -p "$ROOT/etc/modprobe.d"
put 644 "$ROOT/etc/modprobe.d/cfg80211-kr.conf" <<<'options cfg80211 ieee80211_regdom=KR'
put 644 "$ROOT/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf" <<'EOF'
[connection]
wifi.powersave = 2
EOF
if ! grep -qF '# >>> lekiwi_net >>>' "$H/.bashrc" 2>/dev/null; then
	[ -e "$H/.bashrc" ] || { install -m 644 /dev/null "$H/.bashrc"; chown "$USER_NAME:" "$H/.bashrc"; }
	[ ! -s "$H/.bashrc" ] || [ -z "$(tail -c 1 "$H/.bashrc")" ] || echo >> "$H/.bashrc"
	cat "$HERE/bashrc_net.sh" >> "$H/.bashrc"
fi

step 5
install -d -m 700 "$KFDIR"
AP_SSID_KF=${AP_SSID//\\/\\\\}     # nm_kf.py와 같게 \ 는 두 번 (이름과 비밀번호가 같은 값)
if [ "$MODE" = ap ]; then HOME_AUTO=false; else HOME_AUTO=true; fi
if [ -n "$DHCP" ]; then
	IPV4='method=auto'
else
	IPV4="method=manual
address1=192.168.0.$((200 + KIT))/24,192.168.0.1
dns=8.8.8.8;1.1.1.1;"
fi
put 600 "$KFDIR/lekiwi-ap.nmconnection" <<EOF
[connection]
id=lekiwi-ap
uuid=$(new_uuid)
type=wifi
interface-name=wlan0
autoconnect=true
autoconnect-priority=0

[wifi]
mode=ap
ssid=$AP_SSID_KF
band=bg
channel=6

[wifi-security]
key-mgmt=wpa-psk
proto=rsn;
pairwise=ccmp;
group=ccmp;
psk=$AP_SSID_KF

[ipv4]
method=shared
address1=192.168.4.1/24

[ipv6]
method=disabled
EOF
put 600 "$KFDIR/home-wifi.nmconnection" <<EOF
[connection]
id=home-wifi
uuid=$(new_uuid)
type=wifi
interface-name=wlan0
autoconnect=$HOME_AUTO
autoconnect-priority=10

[wifi]
mode=infrastructure
ssid=$ROUTER_SSID

[wifi-security]
key-mgmt=wpa-psk
psk=$ROUTER_PW

[ipv4]
$IPV4

[ipv6]
method=disabled
EOF

step 6
TIMER_ENV=(--setenv=PROVISION_HOME="$HOME_DIR")
[ -z "$ROOT" ] || TIMER_ENV+=(--setenv=PROVISION_ROOT="$ROOT")
systemd-run --unit=lekiwi-net-rollback --on-active=180 --collect "${TIMER_ENV[@]}" \
	"$HERE/rollback_ap_mode.sh" "$BACKUP"
TIMER_AT=$(date +%s)

step 7
put 600 "$NETPLAN" <<<"$NEW_NETPLAN"
netplan generate
netplan apply
# netplan이 만든 wpa_supplicant가 wifis를 지운 뒤에도 wlan0을 잡고 남을 수 있다 (유닛이 없으면 넘어간다)
systemctl stop netplan-wpa-wlan0.service 2>/dev/null || true
WPA=$(systemctl is-active netplan-wpa-wlan0.service 2>/dev/null || true)
case "$WPA" in inactive|failed|unknown) ;; *) fail "netplan-wpa-wlan0.service가 아직 돌고 있습니다 (${WPA:-?})" ;; esac
rm -f "$HOLD"
systemctl restart NetworkManager
# 다시 켠 직후 wlan0은 잠시 unmanaged · unavailable이라, 그때 올리면 바로 실패해 AP로 떨어진다
MANAGED=''
for _ in {1..20}; do
	DEVS=$(nmcli -t -f DEVICE,STATE dev 2>/dev/null || true)
	if grep -qE '^wlan0:(disconnected|connecting|connected)' <<<"$DEVS"; then MANAGED=1; break; fi
	sleep 1
done
[ -n "$MANAGED" ] || fail "NetworkManager가 20초 안에 wlan0을 맡지 않았습니다 ($(grep '^wlan0:' <<<"$DEVS" || echo 'wlan0 없음'))"
nmcli con load "$KFDIR/lekiwi-ap.nmconnection" "$KFDIR/home-wifi.nmconnection"
iw reg set KR
"$H/lekiwi_net/run_$MODE.sh"

step 8
for f in "${LEGACY[@]}"; do
	[ ! -e "$H/$f" ] || mv -f "$H/$f" "$H/$f.legacy-netplan"
done
rm -f "$ROOT/etc/networkd-dispatcher/routable.d/50-kit-ip"
if [ -n "$NETCFG_EDIT" ]; then     # 사전 점검에서 지운 결과를 확인했다
	# 부트 파티션(vfat)은 다시 쓰면 옛 내용(공유기 비밀번호)이 빈 클러스터로 남고 이미지 도구는 p1을 지우지 않는다:
	# 같은 자리를 같은 길이의 0으로 먼저 덮는다 (1<> 는 자르지 않고 열어 같은 클러스터에 쓴다)
	head -c "$(wc -c < "$NETCFG")" /dev/zero 1<> "$NETCFG"
	sync "$NETCFG"
	printf '%s\n' "$NEW_NETCFG" > "$NETCFG"     # 바꿔 끼우지 않고 제자리에 쓴다 (vfat)
	sync "$NETCFG"
fi
[ ! -e "$ROOT/etc/systemd/system/wifi-powersave-off.service" ] || systemctl disable wifi-powersave-off.service

step 9
verified() {
	NOW_ETH0=$(ip -4 -o addr show dev eth0 2>/dev/null | awk '{print $4; exit}' || true)
	ACTIVE=$(nmcli -t -f NAME,DEVICE,STATE con show --active 2>/dev/null || true)
	[ "$NOW_ETH0" = "$ETH0_ADDR" ] && grep -qxF "$WANT:wlan0:activated" <<<"$ACTIVE"
}
OK=''
for _ in {1..30}; do
	if verified; then OK=1; break; fi
	sleep 2
done
[ -n "$OK" ] || fail "60초 안에 확인하지 못했습니다 (eth0 ${NOW_ETH0:-주소 없음}, wlan0이 $WANT 로 activated 아님)"
# is-active는 이미 울린 타이머(elapsed)도 active라, 아직 기다리는 중(waiting)인지 본다
if [ "$(systemctl show -p SubState --value lekiwi-net-rollback.timer 2>/dev/null)" != waiting ]; then
	TIMER_FIRED=1
	fail "되돌림 타이머가 이미 울렸습니다 (롤백이 돌았거나 도는 중): 표지를 쓰지 않습니다"
fi
LEFT=$((180 - ($(date +%s) - TIMER_AT)))
put 644 "$MARK" <<<"provisioned $(date -Is) mode=$MODE backup=/var/backups/$BACKUP_NAME"
echo "[provision] 끝. 남은 약 ${LEFT}초 안에 PC에서 확인한 뒤 되돌림 타이머를 멈추세요: sudo systemctl stop lekiwi-net-rollback.timer"
echo "[provision] 처음 AP: 이름 · 비밀번호 모두 $AP_SSID (배송 뒤 고객이 네트워크 화면에서 바꾼다)"
