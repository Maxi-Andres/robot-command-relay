#!/usr/bin/env bash
# Build command_sender against the prebuilt Unitree SDK. No cmake, no ROS2.
# Works unchanged on x86_64 (dev box) and aarch64 (the robot's Jetson) because the SDK
# ships a static library for both.
set -euo pipefail
cd "$(dirname "$0")"

SDK="${UNITREE_SDK2_DIR:-$HOME/unitree_sdk2}"
ARCH="$(uname -m)"

if [ ! -f "$SDK/lib/$ARCH/libunitree_sdk2.a" ]; then
  echo "error: $SDK/lib/$ARCH/libunitree_sdk2.a not found" >&2
  echo "set UNITREE_SDK2_DIR to the unitree_sdk2 checkout" >&2
  exit 1
fi

INCS=(-I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx")
LIBS=("$SDK/lib/$ARCH/libunitree_sdk2.a" -L"$SDK/thirdparty/lib/$ARCH" -lddscxx -lddsc
      -Wl,-rpath,"$SDK/thirdparty/lib/$ARCH" -lpthread)


# Command senders: the only binaries here that can MOVE a robot. Safety (verb allowlist,
# velocity clamp, dead-man switch) lives inside them — in the shared src/sender_core.hpp —
# not in the HTTP layer above. One per robot: relay_server.py runs the one ROBOT_MODEL names.
for robot in go2 g1; do
  g++ -O2 -std=c++17 "src/${robot}_command_sender.cpp" -o "${robot}_command_sender" \
      "${INCS[@]}" "${LIBS[@]}"
  echo "built ./${robot}_command_sender ($ARCH)"
done
# Flush before returning: the robots are powered off by their switch, and on 2026-10-01 a
# power-off right after a build left binaries at 0 bytes (robot-telemetry-agent, same recipe).
sync
