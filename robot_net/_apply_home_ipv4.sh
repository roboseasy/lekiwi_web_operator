#!/bin/bash
# home-wifi(공유기) 의 IPv4 를 바꾸고 적용한다 — set_ip.sh / set_dhcp.sh 공용.
#   _apply_home_ipv4.sh manual 192.168.0.203
#   _apply_home_ipv4.sh auto
# - 공유기 모드(home-wifi 활성)면 바로 다시 연결해 IP 를 확인하고, 실패하면 이전 설정으로 되돌린다
#   (되돌리기도 실패하면 AP 로 복귀 — 파이에 닿을 길을 남긴다).
# - AP 모드면 저장만 한다 (다음 run_wifi 때 적용).
# ⚠️ nmcli con modify 는 쓰지 않는다 (Ubuntu 24.04 NM 버그) — nm_kf.py 로 keyfile 을 고치고 con load.
D=$(dirname "$(readlink -f "$0")")
KFDIR="${NM_KF_DIR:-/etc/NetworkManager/system-connections}"
KF="$KFDIR/home-wifi.nmconnection"
STATE_DIR="${LEKIWI_NET_STATE_DIR:-/var/lib/lekiwi-net}"
BK="$STATE_DIR/home-wifi.prev"
GW=192.168.0.1
DNS="8.8.8.8;1.1.1.1;"
MODE="$1"; IP="$2"

sudo test -f "$KF" || { echo "error: home-wifi 프로필이 없습니다 — 먼저 run_wifi SSID PW"; exit 1; }
sudo install -d -m 700 "$STATE_DIR"
sudo cp -p "$KF" "$BK"

if [ "$MODE" = "manual" ]; then
  sudo "$D/nm_kf.py" set home-wifi ipv4 method manual ipv4 address1 "$IP/24,$GW" ipv4 dns "$DNS"
else
  sudo "$D/nm_kf.py" set home-wifi ipv4 method auto
  sudo "$D/nm_kf.py" del home-wifi ipv4 address1 dns
fi
sudo nmcli con load "$KF"

if ! nmcli -t -f NAME con show --active | grep -qx home-wifi; then
  echo "[ip] 저장됨 ($MODE${IP:+ $IP}) — 지금은 AP 모드라 다음 run_wifi 때 적용됩니다."
  sudo rm -f "$BK"   # 공유기 비밀번호가 든 사본을 남기지 않는다 (이미지 도구가 이 폴더를 검사하지 않는다)
  exit 0
fi

ok=0
if sudo nmcli --wait 30 con up home-wifi >/dev/null 2>&1; then
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    CUR=$(ip -4 -o addr show wlan0 | awk '{print $4}' | cut -d/ -f1 | head -1)
    if [ -n "$CUR" ] && { [ "$MODE" = "auto" ] || [ "$CUR" = "$IP" ]; }; then ok=1; break; fi
    sleep 1
  done
fi
if [ "$ok" = 1 ]; then
  echo "[ip] 적용됨: wlan0 $CUR ($( [ "$MODE" = manual ] && echo 고정 || echo DHCP ))"
  ip -4 -o addr show wlan0   # 앱(Physical Labs)이 결과를 읽는 줄
  sudo rm -f "$BK"
  exit 0
fi

echo "[ip] 적용 실패 -> 이전 설정으로 되돌립니다"
echo "error: 공유기 IP 설정($MODE${IP:+ $IP})을 적용하지 못해 이전 설정으로 되돌렸습니다"   # 앱이 읽는 줄
sudo cp -p "$BK" "$KF" && sudo rm -f "$BK"
sudo nmcli con load "$KF"
if sudo nmcli --wait 30 con up home-wifi >/dev/null 2>&1; then
  echo "[ip] 이전 설정 복구: wlan0 $(ip -4 -o addr show wlan0 | awk '{print $4}' | head -1)"
else
  echo "[ip] 복구도 실패 -> AP 모드로 전환"
  sudo nmcli con up lekiwi-ap >/dev/null 2>&1
fi
exit 1
