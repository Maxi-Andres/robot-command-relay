"""M0 probe for the G1 microphone: join 239.168.123.161:5555 on the 192.168.123.x interface,
record N seconds of 16 kHz s16le mono to /tmp/g1_mic_probe.wav, and report what arrived.
Python 3.8 stdlib only. Run it FROM the PC, nothing is copied to the robot:
    ssh unitree@192.168.51.115 python3 - 5 < tools/g1_mic_probe.py
    scp unitree@192.168.51.115:/tmp/g1_mic_probe.wav .     # the recording, back to the PC
Step M0 of the plan in ~/Desktop/.claude/roadmap/G1.md §6.6."""
import array
import math
import socket
import struct
import subprocess
import sys
import time
import wave

GROUP, PORT, SECONDS = "239.168.123.161", 5555, float(sys.argv[1]) if len(sys.argv) > 1 else 5.0

# The interface address on the robot's internal network (like Unitree's example).
local = None
out = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True).stdout
for line in out.splitlines():
    parts = line.split()
    addr = parts[3].split("/")[0]
    if addr.startswith("192.168.123."):
        local = addr
        print("interface", parts[1], addr)
        break
if not local:
    sys.exit("no 192.168.123.x address on this machine")

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("", PORT))
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
             struct.pack("4s4s", socket.inet_aton(GROUP), socket.inet_aton(local)))
s.settimeout(2.0)

pcm, sizes, senders, t0, first = bytearray(), [], set(), time.time(), None
while time.time() - t0 < SECONDS:
    try:
        data, src = s.recvfrom(65535)
    except socket.timeout:
        print("no packet for 2 s")
        continue
    if first is None:
        first = time.time() - t0
    sizes.append(len(data))
    senders.add(src)
    pcm += data
elapsed = time.time() - t0
if not sizes:
    sys.exit(f"NOTHING arrived on {GROUP}:{PORT} in {SECONDS:.0f} s")
samples = array.array("h", bytes(pcm[: len(pcm) // 2 * 2]))
rms = math.sqrt(sum(x * x for x in samples) / len(samples)) / 32768
peak = max(abs(x) for x in samples) / 32768
print(f"first packet after {first:.2f} s; {len(sizes)} packets in {elapsed:.1f} s "
      f"({len(sizes) / elapsed:.1f}/s); sizes {sorted(set(sizes))[:5]}; from {senders}")
print(f"bytes {len(pcm)} = {len(pcm) / 32000:.2f} s of 16 kHz mono s16 (wall {elapsed:.2f} s); "
      f"rms {rms:.4f} peak {peak:.4f}")
with wave.open("/tmp/g1_mic_probe.wav", "wb") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(16000)
    w.writeframes(bytes(pcm))
print("wrote /tmp/g1_mic_probe.wav")
