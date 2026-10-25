# robot_net: 로봇의 네트워크 스크립트 (NetworkManager 세대)

로봇(Pi)의 `~/lekiwi_net/`에 깔리는 AP ↔ 공유기 전환 스크립트와, 로봇을 이 방식으로 옮기고 이미지 직전에 점검하는 도구를 둔다.

- 기획서: [../docs/ap_mode_spec.md](../docs/ap_mode_spec.md)
- 구현계획서: [../docs/ap_mode_implementation_plan.md](../docs/ap_mode_implementation_plan.md)
- 이 폴더는 게이트웨이 배포(`deploy_to_robot.sh`) 때 로봇의 `~/lekiwi_web_operator/robot_net/`에 폴더째 실린다. 그래서 **비밀번호를 넣지 않는다** (키 파일 틀의 `psk=<...>` 자리표시뿐).

## 출처

`nm_kf.py` · `run_ap.sh` · `run_wifi.sh` · `set_ap.sh` · `net_status.sh` · `set_ip.sh` · `set_dhcp.sh` · `_apply_home_ipv4.sh` 여덟 개는 Physical Labs 골든 이미지 `lekiwi1005.img`(2026-10-05, 호스트 `lekiwi01`)의 `~/lekiwi_net/`에서 **수정 없이** 가져온 것이다 (이 폴더의 첫 커밋). 이후에 바뀐 곳은 `git diff <첫 커밋> -- web_operator/robot_net/`로 그대로 볼 수 있다. 가져온 방법은 루트 권한 없이 읽기만 하는 `debugfs`다:

```bash
OFFSET=$((1050624*512))     # lekiwi1005.img의 두 번째 파티션 시작 (fdisk -l로 확인)
debugfs -c -R "cat /home/roboseasy/lekiwi_net/<파일>" "lekiwi1005.img?offset=$OFFSET" > robot_net/<파일>
```

원본의 sha256 (가져온 직후):

| 파일 | sha256 |
|------|--------|
| `_apply_home_ipv4.sh` | `ad4aa029a8b8beaa99aa49c2d9676fd1013f00da14a21ed89d33291cf12a74a7` |
| `net_status.sh` | `3888a51b5e573f4c52731140d11f8ddb37255e4f8027b82ecf19fd92ad3a125b` |
| `nm_kf.py` | `c39913c896055bb52c1a7f9c86c80279a0b5713bb219f983b271c1af2a767312` |
| `run_ap.sh` | `3f918d449f08491d83de3b9c9c92757a4d66ff2b4beb5de0d0d3c3e806c8fecf` |
| `run_wifi.sh` | `dbeeb2e2e86e2fd2921ad0149ddabbb809d8ee14e57bf548536cdf8a183105c9` |
| `set_ap.sh` | `1ccb5c64e703f3b4b387be3f93cf34335a65ca023630c0964bae3d530221bece` |
| `set_dhcp.sh` | `6a3f8a7516753b466f1cbe509d1d0178df54f0eac701b2135cfcf6bdc5387f40` |
| `set_ip.sh` | `caa556072c6c957e1b2acab37cbec0b7e0b73c8205d48e0823ab8a67e906736f` |

원본 들여쓰기(공백)를 유지한다. 공유 제품 파일이라 차이를 최소로 두기 위해서다 (로보시지 Python 규칙서의 탭 규칙은 이 폴더의 들여온 파일에는 적용하지 않는다). 새로 쓴 파일(`provision_ap_mode.sh` · `rollback_ap_mode.sh` · `image_ready_check.sh` · `clone_init.sh`)은 탭이다.

## 고친 곳 (가져온 뒤 일곱 가지)

`net_status.sh` · `set_ip.sh` · `set_dhcp.sh`는 위 표의 sha256 그대로다. 나머지 다섯은 아래만 바뀌었다 (`git diff 5556aa1 -- web_operator/robot_net/`).

| # | 파일 | 바꾼 것 | 이유 |
|---|------|------|------|
| 1 | `nm_kf.py` · `run_ap.sh` · `run_wifi.sh` · `set_ap.sh` · `_apply_home_ipv4.sh` | keyfile 폴더 `NM_KF_DIR` · 상태 폴더 `LEKIWI_NET_STATE_DIR` 환경변수로 바꿀 수 있게 함. 기본값이 원래 경로라 **로봇에서는 동작이 같다** | 가짜 `nmcli`와 임시 폴더에서 시험하려고 (`tests/test_robot_net.py`) |
| 2 | `run_ap.sh` | AP 켜기가 실패하면 `home-wifi`의 자동 연결을 **끄기 전 값 그대로**(없던 키는 다시 지움) 되돌리고, 전에 켜져 있었을 때(공유기 우선 모드)만 공유기를 다시 올린 뒤 `error:` 줄을 찍고 종료 1 | 원본은 자동 연결을 끈 채 실패해서, 다시 켤 때까지 와이파이가 아예 없을 수 있었다. AP 고정 모드에서 실패했을 때 공유기 모드로 바꿔 버리지 않게 전 값을 지킨다. 게이트웨이는 실패한 출력(`error:` 줄)을 로그에 남긴다 |
| 3 | `set_ap.sh` | 인자 없이 부르면 **이름만** 보인다 (비밀번호 줄 삭제). 이름 · 비밀번호의 줄바꿈과 1~32바이트를 벗어난 이름은 거절 | 비밀번호가 터미널 · 게이트웨이 로그에 남지 않게. 줄바꿈은 keyfile 줄을 깬다 |
| 4 | `_apply_home_ipv4.sh` | 끝날 때 (적용됨 · 이전 설정으로 복구 · AP라 저장만) `home-wifi.prev` 사본을 지운다 | 사본에 공유기 비밀번호가 있는데 이미지 도구가 `/var/lib/lekiwi-net`을 검사하지 않아 이미지에 실릴 수 있었다 |
| 5 | `run_wifi.sh` | 처음 등록이면(`home-wifi`가 없거나 uuid가 없는 반쪽 파일) `id` · `uuid` · `type` · `interface-name` · 우선순위 10 · `mode` · `key-mgmt=wpa-psk` · `ipv6` 뼈대부터 쓴다. 저장된 공유기가 없는데 인자 없이 부르면 파일을 만들지 않고 `error:` 줄과 종료 1 | 이미지 도구가 `home-wifi`를 지우므로 새 카드의 첫 등록은 파일을 처음부터 만드는 길이다. 원본은 SSID · 비밀번호만 담아서 NetworkManager(libnm 1.46.0)가 `key-mgmt: property is missing`으로 거절한다 |
| 6 | `run_ap.sh` | `home-wifi` 파일이 있을 때만 자동 연결을 고친다 | 없는 프로필을 `nm_kf.py`로 만들면 반쪽 파일이 남고, 5번의 "uuid가 없으면" 판단에 기대야 한다 |
| 7 | `run_wifi.sh` | SSID · 비밀번호의 앞뒤 공백은 거절 (종료 1, 아무것도 안 바꿈) | keyfile은 값의 앞뒤 공백을 잘라 읽어, 그대로 저장하면 다른 이름 · 비밀번호가 된다 (`provision_ap_mode.sh`도 netplan의 값에서 같은 것을 거절한다) |

## 스크립트 (로봇의 `~/lekiwi_net/`)

모두 `sudo`가 필요한 일은 스크립트 안에서 부른다. 로봇에서는 `bashrc_net.sh`가 `.bashrc`에 넣는 단축 명령(`wifi` · `setip` · `setdhcp` · `run_ap` · `run_wifi` · `set_ap` · `net_status`)으로 쓴다.

| 스크립트 | 하는 일 |
|------|------|
| `run_ap.sh` | wlan0을 AP로 (재부팅해도 유지). `home-wifi` 자동 연결을 끈다. AP 주소 192.168.4.1 |
| `run_wifi.sh [SSID PASSWORD]` | 공유기로 (재부팅해도 유지, 못 붙으면 AP로 뜬다). 인자를 주면 새 공유기를 저장한다 (DHCP) |
| `set_ap.sh [SSID PASSWORD]` | AP 이름 · 비밀번호(8~63자)를 정한다. 인자 없이는 이름만 본다 |
| `set_ip.sh [N]` · `set_dhcp.sh` | 공유기 주소를 192.168.0.(200+N) 고정 (N 1~54) / DHCP로. 인자 없이는 지금 값 |
| `net_status.sh` | 켜진 연결과 주소 |
| `nm_kf.py get\|set\|del` | keyfile 편집 (`nmcli con modify`는 NetworkManager 1.46.0이 죽어서 쓰지 않는다) |
| `_apply_home_ipv4.sh` | `set_ip` · `set_dhcp`의 공용 도우미 |

이 폴더에만 있고 로봇의 `~/lekiwi_net/`에는 깔리지 않는 것:

| 스크립트 | 언제 · 어떻게 |
|------|------|
| `provision_ap_mode.sh [--dry-run] [--mode wifi\|ap] [--kit N \| --dhcp] [--ap-ssid NAME] [--hold-nm]` | 한 번. 옛 netplan 세대 카드를 NetworkManager 세대로 옮긴다 (패키지 설치 · 위 스크립트 설치 · 프로필 · 옛 도구 비활성). 먼저 `--dry-run`으로 단계만 본다. 종료 0 성공 · 3 사전 점검 거절 (안 바뀜) · 4 바꾼 뒤 실패 (롤백함) · 5 롤백도 다 되지 않음 (유선으로 들어와 고친다). 신호(INT · TERM · HUP)로 멈춰도 롤백한다 |
| `rollback_ap_mode.sh [백업 tar]` | 위 스크립트가 바꾼 것을 되돌린다 (아래 복구) |
| `image_ready_check.sh` | 이미지를 뜨기 직전, 읽기만 하는 점검 (`sudo`). 한 줄에 `[OK\|WARN\|FAIL] 항목`, 끝에 `요약: FAIL n · WARN m`. FAIL이 있으면 종료 1. 비밀번호 줄 모양과 실제 값(프로필 · 그 줄의 값 · 64자 hex PSK)으로 찾고, 값 없이도 걸리는 흔적(cloud-init 사본 · 압축 로그 · 로그의 명령줄 · 토큰 · 개인키), 부트 파티션 원시 장치의 공유기 값, NetworkManager 상태를 본다. 시스템 폴더의 개인키(`/etc/ssh` 호스트키 빼고) · `authorized_keys`의 앱 설치 · 개인 키 수 · 셸 기록도 본다. `/etc/passwd` · `group` · `hostname`에 든 흔한 글자는 값 추적에서 뺀다. 비밀번호 · 키는 값이 아니라 파일 이름(과 개수)만 보인다 |
| `clone_init.sh <lekiwiNN> <킷 1~54> [--ap-ssid NAME] [--ap-password PW]` | 복제한 카드마다 한 번. 호스트 이름 · 킷 번호 · AP 채널(1·6·11) · 공유기 고정 주소를 정한다. **AP 이름 · 비밀번호는 건드리지 않는다** (처음 AP는 모든 로봇이 `lekiwi01` / `lekiwi01`). 옵션을 줄 때만 쓰고, 비밀번호는 `nm_kf.py`로 직접 써서 sudo 명령줄(auth.log)에 남기지 않는다. 이것만 `~/lekiwi_net/`에 따로 복사해 쓴다 |

## 복구

- **유선이 늘 길이다.** 어떤 스크립트도 `eth0`(10.42.0.195)의 설정 · 주소를 바꾸지 않는다. 와이파이 설정이 꼬여도 PC와 랜선으로 이으면 `ssh roboseasy@10.42.0.195`로 들어와 고친다. 다만 이관과 롤백의 `netplan apply`가 eth0의 DHCP를 다시 해 **유선 ssh가 잠깐 끊길 수 있다**: `provision_ap_mode.sh`는 `sudo setsid nohup … &`로 돌리고 로그를 따로 본다 (구현계획서 P2.4).
- **자동 롤백:** `provision_ap_mode.sh`는 바꾸기 전에 `/var/backups/lekiwi_net_<날짜시각>.tar.gz`(root 600, 공유기 비밀번호가 들어 있다)를 만들고, 180초 뒤 `rollback_ap_mode.sh`를 부르는 타이머(`lekiwi-net-rollback`)를 건다. PC에서 접속을 확인했으면 `sudo systemctl stop lekiwi-net-rollback.timer`로 멈춘다. 아무것도 안 하면 3분 뒤 스스로 되돌린다. 바꾸는 중 실패해도 같은 롤백이 돈다.
- **손으로 되돌리기:** `sudo ./rollback_ap_mode.sh` (또는 `… <백업 tar>`). 파일(netplan · network-config · 옛 도구 · 훅 · 서비스 · `.bashrc`)을 **먼저** 백업에서 되돌리고, 그게 되어야 NetworkManager를 끄고 keyfile을 지운 뒤 `netplan apply` (netplan이 백업 때와 같으면 건너뛴다). 파일을 못 되돌리면 NetworkManager를 켠 채로 두고 멈춘다. wlan0은 다시 netplan + wpa_supplicant가 맡고, 끝에 `netplan-wpa-wlan0.service`가 active인지 본다. 설치한 패키지는 지우지 않는다. 두 번 돌려도 괜찮다.
- **백업 파일은 로봇에 남기지 않는다.** PC로 옮겨 sha256을 맞춘 뒤 로봇에서 `sudo shred -u`로 지운다 (이미지에 공유기 비밀번호가 실리지 않게).
- **처음 AP:** 모든 로봇이 이름 `lekiwi01` · 비밀번호 `lekiwi01`이다 (사용자 결정, 기획서 D2 · R25). 같은 방에 로봇이 여럿이면 이름이 겹친다. 배송 뒤 고객이 네트워크 화면(또는 `~/lekiwi_net/set_ap.sh <이름> <새 비밀번호>`)으로 바꾼다. 바꾼 비밀번호를 잊었으면 유선으로 들어와 같은 명령으로 다시 정한다.
- **netplan 세대에서 `eth0` 설정을 손으로 고쳐야 할 때**는 `netplan apply` 대신 `sudo netplan try`를 쓴다 (120초 안에 확인하지 않으면 이전 설정으로 돌아간다). NetworkManager 세대에서는 wlan0을 netplan이 아니라 NetworkManager가 맡는다.
