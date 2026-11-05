#!/usr/bin/env bash
# g1_nav_probe.sh — what the G1 gives us for a map, read off PC2 in one go. Read-only: nothing
# moves, nothing is installed, nothing is left running.
#
#   cd ~/robot-command-relay && git pull && tools/g1_nav_probe.sh [iface=eth0]
#   tools/g1_nav_probe.sh eth0 odom [seconds=60]    the odometry's drift, walking a known path
#
# With the robot standing or sitting, anything. It asks for sudo once (only the lidar listen of
# section 2 needs it). The output also lands in ~/nav_probe_<date>.txt, to bring back to the PC.
#
# Sections:
#   1. the network: is the Mid-360 (192.168.123.120) there, who on PC2 listens on Livox's
#      ports, what Unitree or anyone runs that looks like lidar/SLAM
#   2. the lidar's own UDP that reaches PC2: to which address and port, how many packets, how
#      many MB/s. PC2 sees only what is sent to it (or broadcast): nothing here means the lidar
#      talks to PC1, not that it is off
#   3. DDS (tools/g1_nav_probe.cpp): every topic, and rate/size/frame of the map-related ones
set -uo pipefail
cd "$(dirname "$0")/.."

IFACE="${1:-eth0}"
MODE="${2:-}"
LIDAR=192.168.123.120
SDK="${UNITREE_SDK2_DIR:-$HOME/unitree_sdk2}"
ARCH="$(uname -m)"
OUT="$HOME/nav_probe_$(date +%Y%m%d-%H%M%S).txt"
BIN=tools/g1_nav_probe

section() { printf '\n==================== %s\n' "$*"; }

# Build first, outside the report: a failed build is the thing to see, alone.
if [ ! -x "$BIN" ] || [ tools/g1_nav_probe.cpp -nt "$BIN" ]; then
  echo "building $BIN ($ARCH) ..."
  rm -f "$BIN"
  g++ -O2 -std=c++17 tools/g1_nav_probe.cpp -o "$BIN" \
      -I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx" \
      "$SDK/lib/$ARCH/libunitree_sdk2.a" -L"$SDK/thirdparty/lib/$ARCH" -lddscxx -lddsc \
      -Wl,-rpath,"$SDK/thirdparty/lib/$ARCH" -lpthread 2>&1 | grep -E 'error' || true
  [ -x "$BIN" ] || { echo "build failed"; exit 1; }
  sync
fi

if [ "$MODE" = odom ]; then
  # Mark the start on the floor, then walk with the remote: 2 m straight, turn 90 degrees,
  # back to the mark. What it reports against what was walked is the drift.
  "$BIN" "$IFACE" odom "${3:-60}" 2>&1 | grep -v 'not multicast-capable' \
    | tee "$HOME/nav_odom_$(date +%Y%m%d-%H%M%S).txt"
  exit
fi

echo "sudo is only for the lidar listen (section 2):"
sudo -v || echo "(no sudo: section 2 will be skipped)"

{
  section "0. host"
  echo "$(hostname)  $(date -Is)  $ARCH  iface=$IFACE"
  ip -br addr

  section "1a. the Mid-360 at $LIDAR"
  ping -c 3 -W 1 "$LIDAR" | tail -2
  ip neigh show "$LIDAR"

  section "1b. who on PC2 listens on Livox's host ports (56101 cmd, 56201 push, 56301 points, 56401 imu, 56501 log)"
  sudo -n ss -uanp 2>/dev/null | grep -E ':56[1-5]01\b' || echo "nobody on PC2"

  section "1c. processes that look like lidar / SLAM / navigation"
  ps -eo pid,user,pcpu,rss,args --sort=-pcpu \
    | grep -iE 'lidar|livox|slam|lio|mapping|locali|nav2|octomap|pointcloud|unitree' \
    | grep -vE 'grep|g1_nav_probe' || echo "none"
  section "1d. services and containers"
  systemctl list-units --type=service --no-pager --plain 2>/dev/null \
    | grep -iE 'lidar|livox|slam|lio|mapping|nav|unitree|robot' || echo "no matching service"
  docker ps --format '{{.Names}}  {{.Image}}  {{.Status}}' 2>/dev/null || echo "(no docker access)"

  section "2. UDP from the lidar that reaches PC2 ($IFACE, 5 s)"
  if sudo -n true 2>/dev/null; then
    sudo -n python3 tools/udp_from_host.py "$IFACE" "$LIDAR" 5
  else
    echo "skipped: no sudo"
  fi

  section "3. DDS"
  "$BIN" "$IFACE" 5 10 2>&1 | grep -v 'not multicast-capable'
} 2>&1 | tee "$OUT"

echo
echo "report saved: $OUT"
