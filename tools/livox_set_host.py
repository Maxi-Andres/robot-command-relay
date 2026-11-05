#!/usr/bin/env python3
"""livox_set_host.py — point the Mid-360's point cloud and IMU at another address, and put them
back. The ONLY write this project makes to the lidar, and it is reversible.

    python3 tools/livox_set_host.py show                       # where they go now (read-only)
    python3 tools/livox_set_host.py set 192.168.123.255        # broadcast: PC1 AND PC2 get it
    python3 tools/livox_set_host.py restore                    # back to what `set` saved

Why (2026-11-05, tools/livox_query.py on the robot): the lidar samples and sends its points to
PC1 (192.168.123.161:56301) only, unicast, so PC2 never sees them. Our obstacle map runs on PC2.
The gentlest change is the subnet BROADCAST address: every PC on the robot's bus receives the
same packets, PC1 included — IF PC1's software listens on any address (a socket bound to
0.0.0.0 gets broadcasts; one bound to 192.168.123.161 alone does not). `show` before and after,
and watch the robot: if anything of Unitree's misbehaves, `restore`.

What it touches: keys 0x0006 (point cloud host) and 0x0007 (IMU host) with command 0x0100, and
nothing else — not the lidar's own IP, not the status push (0x0005), not the work mode. `set`
first saves the current values to ~/.livox_host_backup.json (never overwritten by a second
`set`), and `restore` writes exactly those back.

Unitree's software on PC1, if it is a Livox-SDK2 "master", re-sends its own host configuration
when IT starts (Livox-SDK2 device_manager: UpdateLidarCfg on discovery), so a robot reboot may
undo `set` by itself. Run `show` after a reboot.

Protocol facts: tools/livox_query.py (frame, CRCs) and the Mid-360 protocol page, keys table.
"""
import json
import os
import socket
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import livox_query as lq

LIDAR = os.environ.get("LIVOX_IP", "192.168.123.120")
CONFIGURE = 0x0100
KEYS = {0x0006: ("point cloud",), 0x0007: ("imu",)}
BACKUP = os.path.expanduser("~/.livox_host_backup.json")
# Only addresses on the robot's own bus: a typo must not send the points anywhere else.
ROBOT_NET = "192.168.123."


def current(sock):
    got = lq.inquire(sock, LIDAR, list(KEYS), seq=201)
    if got is None or any(k not in got for k in KEYS):
        raise SystemExit("the lidar did not answer the inquire: nothing changed")
    out = {}
    for k in KEYS:
        v = got[k]
        port, src = struct.unpack("<HH", v[4:8])
        out[f"0x{k:04x}"] = {"ip": socket.inet_ntoa(v[:4]), "port": port, "src": src}
    return out


def configure(sock, values, seq):
    """values {key: (ip, port, src)} -> ACK ret_code, error_key. Raises on no answer."""
    data = struct.pack("<HH", len(values), 0)
    for k, (ip, port, src) in values.items():
        data += struct.pack("<HH", k, 8) + socket.inet_aton(ip) + struct.pack("<HH", port, src)
    sock.sendto(lq.frame(CONFIGURE, data, seq), (LIDAR, lq.CMD_PORT))
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        try:
            buf, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        got = lq.parse(buf)
        if got and got[0] == CONFIGURE and got[1] == 1 and got[2] == seq and len(got[3]) >= 3:
            return got[3][0], struct.unpack("<H", got[3][1:3])[0]
    raise SystemExit("no answer to the configuration: run `show` to see what the lidar has")


def show(sock, title):
    print(title)
    for k, v in current(sock).items():
        print(f"  {k} {KEYS[int(k, 16)][0]:12s} -> {v['ip']}:{v['port']}  (lidar port {v['src']})")


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("show", "set", "restore"):
        print(__doc__.split("\n\n")[1])
        return 2
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0))
    s.settimeout(0.3)
    cmd = sys.argv[1]
    if cmd == "show":
        show(s, f"== {LIDAR} now")
        return 0
    if cmd == "set":
        if len(sys.argv) < 3:
            raise SystemExit("set needs the address, e.g. 192.168.123.255")
        ip = sys.argv[2]
        socket.inet_aton(ip)
        if not ip.startswith(ROBOT_NET):
            raise SystemExit(f"refused: {ip} is not on the robot's bus ({ROBOT_NET}x)")
        before = current(s)
        if not os.path.exists(BACKUP):
            with open(BACKUP, "w") as fh:
                json.dump({"lidar": LIDAR, "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                           "keys": before}, fh, indent=1)
            print(f"saved the current values to {BACKUP}")
        else:
            print(f"{BACKUP} already exists: keeping the first values saved, for restore")
        # Only the address changes: the ports stay as the lidar has them now.
        values = {k: (ip, before[f"0x{k:04x}"]["port"], before[f"0x{k:04x}"]["src"])
                  for k in KEYS}
    else:
        if not os.path.exists(BACKUP):
            raise SystemExit(f"nothing to restore: {BACKUP} does not exist")
        with open(BACKUP) as fh:
            saved = json.load(fh)["keys"]
        values = {int(k, 16): (v["ip"], v["port"], v["src"]) for k, v in saved.items()}
    show(s, f"== {LIDAR} before")
    ret, err_key = configure(s, values, seq=301)
    print(f"configuration answered ret_code 0x{ret:02x}, error_key 0x{err_key:04x}"
          + ("" if ret == 0 else "  <- NOT applied (0x21 = needs a lidar reboot)"))
    time.sleep(0.5)
    show(s, f"== {LIDAR} after")
    if cmd == "restore" and ret == 0:
        os.rename(BACKUP, BACKUP + ".restored")
        print(f"restored; the backup is kept as {BACKUP}.restored")
    return 0 if ret == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
