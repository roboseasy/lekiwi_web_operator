#!/bin/bash
# provision_ap_mode.sh가 바꾼 것을 그 백업으로 되돌린다: wlan0은 다시 netplan + wpa_supplicant, NetworkManager는 꺼 둔다.
#   sudo rollback_ap_mode.sh [BACKUP_TAR]   (없으면 가장 새 /var/backups/lekiwi_net_*.tar.gz)
# 순서: 파일(netplan · network-config · 옛 도구 · 훅 · 서비스 · .bashrc)을 먼저 되돌리고, 그게 되어야 NetworkManager를
# 끄고 keyfile을 지운 뒤 netplan apply. 파일을 못 되돌리면 NetworkManager를 켠 채로 두고 멈춘다 (wlan0을 맡은 쪽을 남긴다).
# 그 뒤 단계는 하나가 실패해도 다음으로 간다. 종료 0 모두 됨 / 1 백업이 없거나(아무것도 안 바꿈) 실패한 단계가 있음.
# netplan이 백업 때와 같으면(예: 설치 단계에서 실패) netplan apply를 건너뛴다 (eth0의 DHCP를 다시 하지 않게).
# 패키지(network-manager)는 지우지 않는다. provision_ap_mode.sh의 실패 처리와 되돌림 타이머가 부른다.
# 시험용 환경변수: PROVISION_ROOT · PROVISION_HOME (provision_ap_mode.sh와 같다)
set -u

ROOT="${PROVISION_ROOT:-}"
NET_SCRIPTS=(nm_kf.py run_ap.sh run_wifi.sh set_ap.sh net_status.sh set_ip.sh set_dhcp.sh _apply_home_ipv4.sh)  # provision과 같은 목록
LEGACY=(set_wifi.sh set_ip.sh set_dhcp.sh)
KFDIR="$ROOT/etc/NetworkManager/system-connections"
HOLD="$ROOT/etc/NetworkManager/conf.d/90-lekiwi-hold.conf"     # provision과 같은 파일 (이름의 까닭은 거기 주석)

if [ -n "$ROOT" ]; then     # 가짜 루트는 시험용: root로 돌면 잘못 붙은 경로가 실제 /etc를 바꿀 수 있다
	[ "$(id -u)" -ne 0 ] || { echo "error: PROVISION_ROOT는 시험용입니다 (root로는 쓰지 않는다)" >&2; exit 1; }
else
	[ "$(id -u)" -eq 0 ] || { echo "error: root로 실행하세요 (sudo)" >&2; exit 1; }
fi
TAR="${1:-}"
if [ -z "$TAR" ]; then
	for f in "$ROOT"/var/backups/lekiwi_net_*.tar.gz; do   # 이름이 날짜시각이라 마지막이 가장 새것
		[ ! -f "$f" ] || TAR="$f"
	done
fi
if [ -z "$TAR" ] || [ ! -f "$TAR" ]; then
	echo "error: 백업이 없습니다 (${TAR:-/var/backups/lekiwi_net_*.tar.gz}) — 아무것도 바꾸지 않았습니다" >&2
	exit 1
fi
LIST=$(tar -tzf "$TAR") || { echo "error: 백업을 읽지 못했습니다: $TAR — 아무것도 바꾸지 않았습니다" >&2; exit 1; }
HOME_DIR="${PROVISION_HOME:-}"
[ -n "$HOME_DIR" ] || [ -z "${SUDO_USER:-}" ] || HOME_DIR=$(getent passwd "$SUDO_USER" | cut -d: -f6)
H="$ROOT$HOME_DIR"

FAILS=0
step() {   # step 설명 명령 [인자...]: 결과를 한 줄로 찍고 실패를 센다 (명령의 종료 코드를 돌려준다)
	local what="$1"
	shift
	if "$@"; then
		echo "[rollback] OK   $what"
	else
		echo "[rollback] 실패 $what"
		FAILS=$((FAILS + 1))
		return 1
	fi
}
in_backup() { grep -qxF "$1" <<<"$LIST"; }     # in_backup 상대경로: 백업에 든 경로인가
extra_netplan() {   # 백업 뒤에 생긴 netplan 파일 (예: NetworkManager가 쓴 90-NM-*.yaml)
	for f in "$ROOT"/etc/netplan/* "$ROOT"/etc/netplan/.[!.]*; do
		[ ! -f "$f" ] || in_backup "etc/netplan/${f##*/}" || printf '%s\n' "$f"
	done
}
prune_netplan() {
	local f
	while IFS= read -r f; do rm -f "$f" || return 1; done < <(extra_netplan)
}
unrename_legacy() {
	[ -n "$HOME_DIR" ] || { echo "[rollback] 홈 폴더를 모릅니다 (sudo로 실행하거나 PROVISION_HOME)"; return 1; }
	for f in "${LEGACY[@]}"; do
		[ ! -e "$H/$f.legacy-netplan" ] || mv -f "$H/$f.legacy-netplan" "$H/$f" || return 1
	done
}
write_hold() {   # NetworkManager가 다시 켜져도 어떤 장치도 잡지 않게
	install -d -m 755 "${HOLD%/*}" && printf '[keyfile]\nunmanaged-devices=*\n' > "$HOLD" && chmod 644 "$HOLD"
}
remove_installed() {   # 프로비저닝이 새로 만든 것. 백업에 들어 있던 것(전부터 있던 것)은 방금 되살렸으니 둔다
	rm -f "$KFDIR"/lekiwi-ap.nmconnection* "$KFDIR"/home-wifi.nmconnection* "$ROOT/run/lekiwi-ap-password" || return 1
	in_backup etc/modprobe.d/cfg80211-kr.conf || rm -f "$ROOT/etc/modprobe.d/cfg80211-kr.conf" || return 1
	[ -n "$HOME_DIR" ] || return 0
	for f in "${NET_SCRIPTS[@]}"; do
		in_backup "${HOME_DIR#/}/lekiwi_net/$f" || rm -f "$H/lekiwi_net/$f" || return 1
	done
	in_backup "${HOME_DIR#/}/lekiwi_net/" || rmdir "$H/lekiwi_net" 2>/dev/null || true   # 사람이 더 넣은 파일이 있으면 둔다
}

echo "[rollback] 백업: ${TAR#"$ROOT"}"
systemctl stop lekiwi-net-rollback.timer 2>/dev/null || true    # 이미 되돌리는 중이니 타이머가 또 돌지 않게 (없으면 넘어간다)
NETPLAN_SAME=''
if [ -z "$(extra_netplan)" ] && tar -dzf "$TAR" -C "$ROOT/" etc/netplan >/dev/null 2>&1; then NETPLAN_SAME=1; fi

# 1. 파일 먼저. 여기서 실패하면 NetworkManager를 그대로 둔다
step "옛 도구 이름 되돌리기 (*.legacy-netplan)" unrename_legacy
if ! step "netplan 폴더를 백업 때 모양으로" prune_netplan \
	|| ! step "백업에서 되돌리기 (netplan · network-config · 옛 도구 · 훅 · 서비스 · .bashrc)" tar -xpzf "$TAR" -C "$ROOT/"; then
	echo "[rollback] 파일을 되돌리지 못해 여기서 멈춘다: NetworkManager와 keyfile은 그대로 둔다 (wlan0이 갈 곳을 잃지 않게)"
	echo "[rollback] 끝: 실패 $FAILS 개"
	exit 1
fi

# 2. 그다음 NetworkManager를 내리고 wlan0을 netplan에 돌려준다
step "NetworkManager가 장치를 잡지 않게 (${HOLD#"$ROOT"})" write_hold
nmcli -w 10 dev disconnect wlan0 >/dev/null 2>&1 || true        # AP · 공유기 연결을 먼저 내린다 (주소 · dnsmasq 정리)
step "NetworkManager 끄기 (disable --now)" systemctl disable --now NetworkManager
pkill -f 'dnsmasq.*nm-dnsmasq-wlan0' 2>/dev/null || true        # 남은 AP용 dnsmasq (없으면 넘어간다)
step "keyfile · 설치한 파일 지우기" remove_installed
step "systemd 다시 읽기" systemctl daemon-reload
if [ -n "$NETPLAN_SAME" ]; then
	echo "[rollback] 건너뜀 netplan apply (netplan이 백업 때와 같다)"
else
	step "netplan apply" netplan apply
fi
step "국가코드 KR" iw reg set KR
step "표지 파일 지우기" rm -f "$ROOT/etc/lekiwi-net-provisioned"
if tar -xzOf "$TAR" --wildcards 'etc/netplan/*.yaml' 2>/dev/null | grep -qE '^ *wifis:'; then
	step "wlan0을 netplan이 다시 맡음 (netplan-wpa-wlan0.service active)" systemctl is-active --quiet netplan-wpa-wlan0.service
fi
echo "[rollback] 끝: 실패 $FAILS 개"
[ "$FAILS" -eq 0 ]
