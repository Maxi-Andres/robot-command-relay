#!/usr/bin/env python3
"""livox_listen.py — do the Mid-360's points reach THIS machine, and what do they look like?
Read-only: it only listens.

    python3 tools/livox_listen.py [seconds=5]

Listens on the host ports the lidar sends to (point cloud 56301, IMU 56401) on every address of
this machine, broadcast included, and reports packets/s, points/s, MB/s, the data type, and the
range of the points it parsed. Run it after tools/livox_set_host.py points the lidar here.

Packet format (Mid-360 protocol, "Point Cloud Data"): a 36-byte header — version u8, length
u16, time_interval u16 (0.1 us), dot_num u16, udp_cnt u16, frame_cnt u8, data_type u8,
time_type u8, 12 reserved, crc32 u32, timestamp u64 ns — then dot_num points: data_type 1 =
x, y, z int32 mm + reflectivity u8 + tag u8 (14 bytes, 96 per packet); 2 = int16 x, y, z
(10 mm) + refl + tag; 0 = IMU, gyro xyz float rad/s + acc xyz float g.
"""
import math
import select
import socket
import struct
import sys
import time

HEADER = struct.Struct("<BHHHHBBB12sIQ")       # 36 bytes
POINT = {1: struct.Struct("<iiiBB"), 2: struct.Struct("<hhhBB")}
SCALE = {1: 0.001, 2: 0.01}                    # to metres


def listen(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
    s.bind(("", port))
    s.setblocking(False)
    return s


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    socks = {listen(56301): "points", listen(56401): "imu"}
    stats = {name: {"pkts": 0, "bytes": 0, "dots": 0, "types": set(), "src": set()}
             for name in socks.values()}
    ranges, zs, imu = [], [], None
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        ready, _w, _x = select.select(list(socks), [], [], 0.2)
        for s in ready:
            while True:
                try:
                    buf, addr = s.recvfrom(2048)
                except BlockingIOError:
                    break
                name = socks[s]
                st = stats[name]
                st["pkts"] += 1
                st["bytes"] += len(buf)
                st["src"].add(addr[0])
                if len(buf) < HEADER.size:
                    continue
                h = HEADER.unpack_from(buf)
                dot_num, data_type = h[3], h[6]
                st["dots"] += dot_num
                st["types"].add(data_type)
                if data_type in POINT and st["pkts"] % 20 == 1:     # a sample, not every packet
                    for x, y, z, _r, _t in POINT[data_type].iter_unpack(
                            buf[HEADER.size:HEADER.size + dot_num * POINT[data_type].size]):
                        if x or y or z:
                            k = SCALE[data_type]
                            ranges.append(math.sqrt(x * x + y * y + z * z) * k)
                            zs.append(z * k)
                if data_type == 0 and len(buf) >= HEADER.size + 24:
                    imu = struct.unpack_from("<6f", buf, HEADER.size)
    for name, st in stats.items():
        print(f"{name:7s} {st['pkts'] / seconds:7.1f} pkt/s  {st['dots'] / seconds:9.0f} pts/s  "
              f"{st['bytes'] / seconds / 1048576:6.2f} MB/s  data_type {sorted(st['types'])}  "
              f"from {sorted(st['src'])}")
    if not stats["points"]["pkts"]:
        print("NO POINTS reached this machine: the lidar sends them elsewhere "
              "(tools/livox_query.py says where)")
        return 1
    if ranges:
        ranges.sort()
        zs.sort()
        print(f"points (sampled {len(ranges)}): range m min {ranges[0]:.2f} median "
              f"{ranges[len(ranges) // 2]:.2f} max {ranges[-1]:.2f}; z m {zs[0]:.2f} .. {zs[-1]:.2f}"
              f" (lidar frame)")
    if imu:
        g = math.sqrt(sum(a * a for a in imu[3:]))
        print(f"imu: gyro {imu[0]:+.3f} {imu[1]:+.3f} {imu[2]:+.3f} rad/s, acc {imu[3]:+.2f} "
              f"{imu[4]:+.2f} {imu[5]:+.2f} g (|a| {g:.2f}: ~1 = gravity, the unit is right)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
