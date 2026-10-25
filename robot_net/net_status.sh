#!/bin/bash
nmcli -f NAME,DEVICE,STATE con show --active | grep -v '^lo '
ip -4 -br a | grep -v '^lo'
