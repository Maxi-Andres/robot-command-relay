#!/usr/bin/env python3
"""livox_query.py — ask the G1's Mid-360 where it sends its data and what state it is in.
READ-ONLY: it sends only the protocol's "parameter inquire" (0x0101) and the broadcast discovery
(0x0000), never "parameter configuration" (0x0100), a reboot or anything that changes the lidar.

    python3 tools/livox_query.py [lidar_ip=192.168.123.120]

Why: on 2026-11-05 the lidar answered ping from PC2 but no UDP of it reached PC2 and no DDS topic
carried a point cloud. Its own config says which host gets the point cloud (PC1? nobody?) and
whether it is sampling at all — what decides how our SLAM on PC2 can get the points.

Protocol (Livox Mid-360 Ethernet protocol v1.4.12, livox-wiki-en.readthedocs.io, and Livox-SDK2's
sdk_core/comm/sdk_protocol.*): a 24-byte little-endian header — sof 0xAA, version 0, length
(header + data), seq u32, cmd_id u16, cmd_type 0 = request, sender_type 0 = host, 6 reserved,
crc16 (CCITT-FALSE over bytes 0..17), crc32 (standard CRC-32 over the data only, 0 if none) —
then the data. Commands go to the lidar's port 56100; discovery is a broadcast to 56000.
"""
import socket
import struct
import sys
import time
import zlib

CMD_PORT = 56100
DISCOVERY_PORT = 56000
DISCOVERY_LISTEN = 56001          # Livox-SDK2's kDetectionListenPort
QUERY = 0x0101                    # parameter inquire: read-only
DISCOVER = 0x0000

KEYS = {                          # key: (name, decoder)
    0x0004: ("lidar ip / mask / gateway", "ipcfg12"),
    0x0005: ("status push -> host", "hostcfg"),
    0x0006: ("POINT CLOUD -> host", "hostcfg"),
    0x0007: ("imu -> host", "hostcfg"),
    0x001A: ("work target mode", "mode"),
    0x8006: ("current work state", "mode"),
    0x8000: ("serial number", "text"),
    0x8001: ("product", "text"),
    0x8002: ("firmware", "ver"),
    0x8005: ("mac", "mac"),
    0x800C: ("time sync (0 none, 1 PTP, 2 GPS)", "u8"),
    0x8007: ("core temperature C", "temp"),
}
# Working states (wiki "LiDAR Working Status"; the SDK adds 0x03 sleep, 0x07 motor stopping).
MODES = {0x01: "SAMPLING", 0x02: "IDLE", 0x03: "SLEEP", 0x04: "ERROR", 0x05: "SELFCHECK",
         0x06: "MOTOR STARTUP", 0x07: "MOTOR STOPPING", 0x08: "UPGRADE", 0x09: "READY"}


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd_id, data=b"", seq=1):
    head = struct.pack("<BBHIHBB6s", 0xAA, 0, 24 + len(data), seq, cmd_id, 0, 0, b"\0" * 6)
    crc32 = zlib.crc32(data) & 0xFFFFFFFF if data else 0
    return head + struct.pack("<HI", crc16_ccitt_false(head), crc32) + data


def parse(buf):
    """(cmd_id, cmd_type, seq, data) of a frame with valid CRCs, or None."""
    if len(buf) < 24 or buf[0] != 0xAA:
        return None
    _sof, _ver, length, seq, cmd_id, cmd_type, _sender, _r = struct.unpack("<BBHIHBB6s", buf[:18])
    crc16, crc32 = struct.unpack("<HI", buf[18:24])
    data = buf[24:length]
    if crc16 != crc16_ccitt_false(buf[:18]):
        return None
    if data and crc32 != zlib.crc32(data) & 0xFFFFFFFF:
        return None
    return cmd_id, cmd_type, seq, data


def decode(kind, v):
    if kind == "hostcfg" and len(v) >= 8:
        ip = socket.inet_ntoa(v[:4])
        dst, src = struct.unpack("<HH", v[4:8])
        return f"{ip}:{dst}  (from lidar port {src})"
    if kind == "ipcfg12" and len(v) >= 12:
        return " / ".join(socket.inet_ntoa(v[i:i + 4]) for i in (0, 4, 8))
    if kind == "mode" and v:
        return f"0x{v[0]:02x} {MODES.get(v[0], '?')}"
    if kind == "text":
        return v.split(b"\0")[0].decode(errors="replace")
    if kind == "ver" and len(v) >= 4:
        return ".".join(str(b) for b in v[:4])
    if kind == "mac" and len(v) >= 6:
        return ":".join(f"{b:02x}" for b in v[:6])
    if kind == "u8" and v:
        return str(v[0])
    if kind == "temp" and len(v) >= 4:
        return f"{struct.unpack('<i', v[:4])[0] / 100:.1f}"
    return v.hex()


def inquire(sock, lidar, keys, seq):
    data = struct.pack("<HH", len(keys), 0) + b"".join(struct.pack("<H", k) for k in keys)
    sock.sendto(frame(QUERY, data, seq), (lidar, CMD_PORT))
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        try:
            buf, _addr = sock.recvfrom(2048)
        except socket.timeout:
            continue
        got = parse(buf)
        if not got or got[0] != QUERY or got[1] != 1 or got[2] != seq:
            continue
        d = got[3]
        ret, n = d[0], struct.unpack("<H", d[1:3])[0]
        if ret != 0:
            print(f"  the lidar answered ret_code 0x{ret:02x} (not success)")
        off, out = 3, {}
        for _ in range(n):
            k, ln = struct.unpack("<HH", d[off:off + 4])
            out[k] = d[off + 4:off + 4 + ln]
            off += 4 + ln
        return out
    return None


def discover(lidar_net_bcast="192.168.123.255"):
    """The broadcast discovery: who answers, with which serial and ip."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", DISCOVERY_LISTEN))
    except OSError as exc:
        print(f"  cannot listen on {DISCOVERY_LISTEN}: {exc}")
        return
    s.settimeout(0.3)
    s.sendto(frame(DISCOVER, seq=7), (lidar_net_bcast, DISCOVERY_PORT))
    end, seen = time.monotonic() + 2.0, False
    while time.monotonic() < end:
        try:
            buf, addr = s.recvfrom(2048)
        except socket.timeout:
            continue
        got = parse(buf)
        if not got or got[0] != DISCOVER or got[1] != 1 or len(got[3]) < 24:
            continue
        d = got[3]
        sn = d[2:18].split(b"\0")[0].decode(errors="replace")
        print(f"  {addr[0]}: dev_type {d[1]} (9 = Mid-360), sn {sn}, ip "
              f"{socket.inet_ntoa(d[18:22])}, cmd port {struct.unpack('<H', d[22:24])[0]}")
        seen = True
    if not seen:
        print("  no discovery answer reached this PC in 2 s")


def main():
    lidar = sys.argv[1] if len(sys.argv) > 1 else "192.168.123.120"
    assert crc16_ccitt_false(b"123456789") == 0x29B1        # the catalogue's check values
    assert zlib.crc32(b"123456789") == 0xCBF43926
    print(f"== Livox discovery (broadcast to {DISCOVERY_PORT})")
    discover()
    print(f"\n== parameter inquire to {lidar}:{CMD_PORT} (read-only)")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("", 0))
    s.settimeout(0.3)
    keys = list(KEYS)
    got = {}
    # Two small batches: an answer has to fit in one 1400-byte frame.
    for seq, batch in ((101, keys[:6]), (102, keys[6:])):
        out = inquire(s, lidar, batch, seq)
        if out is None:
            print(f"  no answer to keys {[hex(k) for k in batch]} in 2 s (the lidar may reply "
                  f"only to its configured host)")
            continue
        got.update(out)
    for k, (name, kind) in KEYS.items():
        if k in got:
            print(f"  {name:36s} {decode(kind, got[k])}")
    return 0 if got else 1


if __name__ == "__main__":
    sys.exit(main())
