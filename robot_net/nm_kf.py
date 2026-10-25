#!/usr/bin/env python3
# NetworkManager keyfile 편집기 (Ubuntu NM 의 netplan 저장 버그 회피용: nmcli con modify 대신 사용)
#   nm_kf.py get  NAME SECTION KEY
#   nm_kf.py set  NAME SECTION KEY VALUE [SECTION KEY VALUE ...]
#   nm_kf.py del  NAME SECTION KEY [KEY ...]
import configparser, os, sys
DIR = os.environ.get("NM_KF_DIR", "/etc/NetworkManager/system-connections")
op, name, args = sys.argv[1], sys.argv[2], sys.argv[3:]
path = os.path.join(DIR, name + ".nmconnection")
cp = configparser.RawConfigParser(delimiters=("=",), interpolation=None)
cp.optionxform = str
cp.read(path)
if op == "get":
    print(cp.get(args[0], args[1], fallback=""))
    sys.exit(0)
if op == "set":
    for i in range(0, len(args), 3):
        sec, key, val = args[i:i + 3]
        if not cp.has_section(sec):
            cp.add_section(sec)
        cp.set(sec, key, val.replace("\\", "\\\\"))
elif op == "del":
    for key in args[1:]:
        cp.remove_option(args[0], key)
tmp = path + ".tmp"
with open(tmp, "w") as f:
    cp.write(f, space_around_delimiters=False)
os.chmod(tmp, 0o600)
os.replace(tmp, path)
