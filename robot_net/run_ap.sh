#!/bin/bash
# wlan0 -> AP 모드 (재부팅해도 유지). 파이 IP: 192.168.4.1
D=$(dirname "$(readlink -f "$0")")
KFDIR="${NM_KF_DIR:-/etc/NetworkManager/system-connections}"
HOME_KF="$KFDIR/home-wifi.nmconnection"
# 이미지 직후 카드에는 공유기 프로필이 없다. 없는 프로필을 nm_kf.py 로 만들면 반쪽 파일이 되므로 있을 때만 고친다
PREV_AUTO=''
if sudo test -f "$HOME_KF"; then
  PREV_AUTO=$(sudo "$D/nm_kf.py" get home-wifi connection autoconnect)   # 실패하면 정확히 이 값으로 되돌린다 (빈 값 = 키 없음 = true)
  sudo "$D/nm_kf.py" set home-wifi connection autoconnect false
  sudo nmcli con load "$HOME_KF"
fi
# NM 이 비정상 종료되면 AP 용 dnsmasq 가 남아 192.168.4.1 을 점유 -> AP 가 꺼진 상태면 정리
if ! nmcli -t -f NAME con show --active | grep -qx lekiwi-ap; then
  sudo pkill -f "dnsmasq.*nm-dnsmasq-wlan0" 2>/dev/null
fi
if sudo nmcli con up lekiwi-ap >/dev/null; then
  echo "[run_ap] AP 모드 ON -> PC 를 '$(sudo "$D/nm_kf.py" get lekiwi-ap wifi ssid)' 에 연결 후: ssh $USER@192.168.4.1"
else
  echo "[run_ap] AP 전환 실패"
  if sudo test -f "$HOME_KF"; then
    if [ -n "$PREV_AUTO" ]; then
      sudo "$D/nm_kf.py" set home-wifi connection autoconnect "$PREV_AUTO"
    else
      sudo "$D/nm_kf.py" del home-wifi connection autoconnect
    fi
    sudo nmcli con load "$HOME_KF"
    if [ "${PREV_AUTO:-true}" != false ]; then   # 공유기 우선 모드였을 때만 공유기로 돌아간다
      sudo nmcli --wait 30 con up home-wifi >/dev/null
      echo "error: AP 전환 실패 (공유기 모드로 돌아갑니다)"; exit 1
    fi
  fi
  echo "error: AP 전환 실패 (저장된 모드는 그대로입니다)"; exit 1
fi
