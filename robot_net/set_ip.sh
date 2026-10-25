#!/bin/bash
# 공유기(home-wifi) 고정 IP: 192.168.0.(200+킷번호) — 예전 setip N 대체 (NetworkManager 세대)
#   setip N   (N = 1~54, 예: setip 3 -> 192.168.0.203)
#   setip     : 현재 IP 설정 보기
D=$(dirname "$(readlink -f "$0")")
if [ $# -eq 0 ]; then
  echo "method : $(sudo "$D/nm_kf.py" get home-wifi ipv4 method)"
  echo "address: $(sudo "$D/nm_kf.py" get home-wifi ipv4 address1)"
  echo "wlan0  : $(ip -4 -o addr show wlan0 | awk '{print $4}' | head -1)"
  exit 0
fi
case "$1" in ''|*[!0-9]*) echo "[setip] 사용법: setip <킷번호 1~54>"; exit 1;; esac
if [ "$1" -lt 1 ] || [ "$1" -gt 54 ]; then echo "[setip] 킷 번호는 1~54"; exit 1; fi
exec "$D/_apply_home_ipv4.sh" manual "192.168.0.$((200 + $1))"
