#!/bin/bash
# 공유기(home-wifi) 를 DHCP 로 — 예전 setdhcp 대체 (고정 IP 로 돌아가려면 setip N)
D=$(dirname "$(readlink -f "$0")")
exec "$D/_apply_home_ipv4.sh" auto
