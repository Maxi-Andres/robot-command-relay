#!/usr/bin/env python3
"""udp_from_host.py — which UDP a host sends that reaches this machine: destination address and
port, packets, MB/s. A passive raw-socket listen, for PCs without tcpdump (PC2 has none).

    sudo python3 tools/udp_from_host.py eth0 192.168.123.120 5

Needs root (a raw socket). It only sees what the switch delivers here: traffic addressed to this
machine, broadcast or multicast. Nothing seen means the host talks to another PC, not that it is
silent.
"""
import socket
import struct
import sys
import time

ETH_P_ALL = 0x0003
SOL_PACKET, PACKET_ADD_MEMBERSHIP, PACKET_MR_ALLMULTI = 263, 1, 2


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    src = sys.argv[2] if len(sys.argv) > 2 else "192.168.123.120"
    seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
    want = socket.inet_aton(src)
    s = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(ETH_P_ALL))
    s.bind((iface, 0))
    # Multicast this machine never joined is dropped by the NIC; all-multi lets it through for as
    # long as this socket lives (a lidar can be set to send to a group).
    mreq = struct.pack("iHH8s", socket.if_nametoindex(iface), PACKET_MR_ALLMULTI, 0, b"")
    s.setsockopt(SOL_PACKET, PACKET_ADD_MEMBERSHIP, mreq)
    s.settimeout(0.2)
    seen = {}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            frame = s.recv(65535)
        except socket.timeout:
            continue
        if len(frame) < 42 or frame[12:14] != b"\x08\x00":       # IPv4 only
            continue
        ip = frame[14:]
        ihl = (ip[0] & 0x0F) * 4
        if ip[9] != 17 or ip[12:16] != want:                      # UDP from `src`
            continue
        dst = socket.inet_ntoa(ip[16:20])
        sport, dport, length = struct.unpack("!HHH", ip[ihl:ihl + 6])
        key = (dst, sport, dport)
        n, b = seen.get(key, (0, 0))
        seen[key] = (n + 1, b + length - 8)
    if not seen:
        print(f"no UDP from {src} reached {iface} in {seconds:.0f} s")
        return 1
    for (dst, sport, dport), (n, b) in sorted(seen.items()):
        print(f"{src}:{sport} -> {dst}:{dport}  {n:7d} pkts  {n / seconds:8.1f} pkt/s  "
              f"{b / seconds / 1048576:7.2f} MB/s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
