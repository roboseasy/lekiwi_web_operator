#!/bin/bash
# wlan0 -> 공유기 모드 (재부팅해도 유지, 공유기가 안 보이면 AP 로 뜸)
#   run_wifi               : 저장된 공유기로 연결
#   run_wifi SSID PASSWORD : 새 공유기 정보 저장(DHCP) 후 연결
D=$(dirname "$(readlink -f "$0")")
KFDIR="${NM_KF_DIR:-/etc/NetworkManager/system-connections}"
KF="$KFDIR/home-wifi.nmconnection"
if [ $# -eq 2 ]; then
  if [ ${#2} -lt 8 ] || [ ${#2} -gt 63 ]; then echo "[run_wifi] 비밀번호는 8~63자"; exit 1; fi
  # keyfile 은 값의 앞뒤 공백을 잘라 읽으므로 그대로 저장하면 다른 이름 · 비밀번호가 된다
  for v in "$1" "$2"; do case "$v" in [[:space:]]*|*[[:space:]]) echo "[run_wifi] SSID · 비밀번호 앞뒤에 공백을 둘 수 없습니다"; exit 1;; esac; done
  # 이미지 직후 카드에는 home-wifi 프로필이 없다 (이미지 도구가 지운다). SSID · 비밀번호만 담은 파일은 key-mgmt 등이
  # 빠져 NetworkManager 가 거절하므로, uuid 가 없으면(없는 파일 · 반쪽 파일) 뼈대부터 만든다
  if [ -z "$(sudo "$D/nm_kf.py" get home-wifi connection uuid)" ]; then
    sudo "$D/nm_kf.py" set home-wifi connection id home-wifi connection uuid "$(python3 -c 'import uuid; print(uuid.uuid4())')" \
      connection type wifi connection interface-name wlan0 connection autoconnect-priority 10 \
      wifi mode infrastructure wifi-security key-mgmt wpa-psk ipv6 method disabled
  fi
  sudo "$D/nm_kf.py" set home-wifi wifi ssid "$1" wifi-security psk "$2" ipv4 method auto
  sudo "$D/nm_kf.py" del home-wifi ipv4 address1 dns
  echo "[run_wifi] 공유기 정보 저장: $1 (DHCP)"
elif [ $# -ne 0 ]; then
  echo "사용법: run_wifi [SSID PASSWORD]"; exit 1
elif [ -z "$(sudo "$D/nm_kf.py" get home-wifi wifi ssid)" ]; then
  echo "[run_wifi] 저장된 공유기가 없습니다 — run_wifi SSID PASSWORD 로 먼저 등록하세요"
  echo "error: 저장된 공유기가 없습니다 — 먼저 공유기 SSID · 비밀번호를 등록하세요"; exit 1
fi
sudo "$D/nm_kf.py" set home-wifi connection autoconnect true
sudo nmcli con load "$KF"
SSID=$(sudo "$D/nm_kf.py" get home-wifi wifi ssid)
if sudo nmcli --wait 30 con up home-wifi >/dev/null; then
  echo "[run_wifi] 연결됨: $SSID / $(ip -4 -br a show wlan0 | awk '{print $3}')"
  ip -4 -o addr show wlan0   # 앱(Physical Labs)이 결과를 읽는 줄 — "inet <IP>"
else
  echo "[run_wifi] '$SSID' 연결 실패 -> AP 모드로 복귀"
  sudo nmcli con up lekiwi-ap >/dev/null
  echo "error: '$SSID' 연결 실패 — SSID·비밀번호를 확인하세요 (AP 모드로 돌아왔습니다)"   # 앱이 읽는 줄
  exit 1
fi
