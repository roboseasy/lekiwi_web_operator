#!/bin/bash
# AP 이름/비밀번호 변경
#   set_ap SSID PASSWORD   (비밀번호 8~63자, WPA2 규격)
#   set_ap                 : 현재 설정 보기
D=$(dirname "$(readlink -f "$0")")
KFDIR="${NM_KF_DIR:-/etc/NetworkManager/system-connections}"
KF="$KFDIR/lekiwi-ap.nmconnection"
if [ $# -eq 0 ]; then
  echo "SSID: $(sudo "$D/nm_kf.py" get lekiwi-ap wifi ssid)"
  exit 0
fi
if [ $# -ne 2 ]; then echo "사용법: set_ap SSID PASSWORD"; exit 1; fi
case "$1$2" in *$'\n'*|*$'\r'*) echo "[set_ap] 이름 · 비밀번호에 줄바꿈을 넣을 수 없습니다"; exit 1;; esac
N=$(printf %s "$1" | wc -c)
if [ "$N" -lt 1 ] || [ "$N" -gt 32 ]; then echo "[set_ap] AP 이름은 1~32바이트여야 합니다"; exit 1; fi
if [ ${#2} -lt 8 ] || [ ${#2} -gt 63 ]; then
  echo "[set_ap] 비밀번호는 8~63자여야 합니다 (WPA2 규격)"; exit 1
fi
sudo "$D/nm_kf.py" set lekiwi-ap wifi ssid "$1" wifi-security psk "$2"
sudo nmcli con load "$KF"
echo "[set_ap] 저장됨: SSID=$1"
if nmcli -t -f NAME con show --active | grep -qx lekiwi-ap; then
  sudo nmcli con up lekiwi-ap >/dev/null && echo "[set_ap] AP 재시작 완료 (새 SSID 로 다시 연결하세요)"
fi
