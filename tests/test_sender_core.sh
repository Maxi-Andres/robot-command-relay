#!/usr/bin/env bash
# Drive src/sender_core.hpp — the safety envelope of BOTH robots' senders — with a fake robot.
#
#   UNITREE_SDK2_DIR=~/unitree_sdk2 bash tests/test_sender_core.sh
#
# Needs the Unitree SDK to compile (it is not vendored, so CI skips this; run it on the dev box
# or on a robot). Binds DDS to the loopback interface: nothing reaches a robot.
#
# What must hold, the four defences in sender_core.hpp:
#   1. an unknown verb is refused and never reaches the robot
#   2. move is clamped to the robot's limits
#   3. a move not refreshed within DEADMAN_MS is stopped automatically
#   4. closing stdin stops the robot before exiting
#   5. joy is refused outside pose, ends with any other verb, and zeroes when not refreshed
set -u
SDK="${UNITREE_SDK2_DIR:-$HOME/unitree_sdk2}"
ARCH="$(uname -m)"
[ -f "$SDK/lib/$ARCH/libunitree_sdk2.a" ] || { echo "SKIP: no Unitree SDK at $SDK"; exit 0; }
HERE=$(cd "$(dirname "$0")" && pwd)
BASE=$(mktemp -d); trap 'rm -rf "$BASE"' EXIT
# --no-as-needed: this fake calls nothing in libddsc directly, so the linker would drop it, and
# libddscxx then cannot find it — RUNPATH is not inherited by a library's own dependencies.
# The real senders do not need this (they use libddsc themselves).
g++ -O1 -std=c++17 "$HERE/sender_core_fake.cpp" -o "$BASE/fake" \
    -I"$SDK/include" -I"$SDK/thirdparty/include" -I"$SDK/thirdparty/include/ddscxx" \
    "$SDK/lib/$ARCH/libunitree_sdk2.a" -L"$SDK/thirdparty/lib/$ARCH" \
    -Wl,--no-as-needed -lddscxx -lddsc -Wl,--as-needed \
    -Wl,-rpath,"$SDK/thirdparty/lib/$ARCH" -lpthread || { echo "FAIL: build"; exit 1; }

# 1 s dead-man; the script pauses 1.5 s after a move so the dead-man must fire.
{ echo "front_flip"; echo "move 9 -9 9"; sleep 1.5; echo "wave_hand"; } \
  | DDS_IFACE=lo DEADMAN_MS=1000 timeout 20 "$BASE/fake" >"$BASE/out" 2>"$BASE/err"

fail=0
expect() { grep -qF -- "$2" "$BASE/$3" && echo "  ok    $1" || { echo "  FAIL  $1 (no '$2' in $3)"; fail=1; }; }
refute() { grep -qF -- "$2" "$BASE/$3" && { echo "  FAIL  $1 ('$2' in $3)"; fail=1; } || echo "  ok    $1"; }

expect "unknown verb refused"          "err unknown verb"            out
refute "unknown verb never dispatched" "call front_flip"             err
expect "move clamped to the limits"    "call move 0.30 -0.20 0.50"   err
expect "clamped values echoed"         "applied=0.3,-0.2,0.5"        out
expect "dead-man fired"                "ev deadman_stop"             out
expect "allowed verb dispatched"       "call wave_hand"              err
expect "EOF stops the robot"           "stdin closed"                err
[ "$(grep -c 'call stop_move' "$BASE/err")" -ge 2 ] \
  && echo "  ok    stop_move sent by dead-man AND at EOF" \
  || { echo "  FAIL  expected 2+ stop_move calls"; fail=1; }
# Rule 5: refused before pose; accepted (clamped) after; zeroed by the dead-man; ended by a verb.
{ echo "joy 0 0 0.4 0"; echo "pose_on"; echo "joy 0 0 2 -0.3"; sleep 1.5; \
  echo "joy 0 0 0.5 0"; echo "wave_hand"; echo "joy 0 0 0.6 0"; } \
  | DDS_IFACE=lo DEADMAN_MS=1000 timeout 20 "$BASE/fake" >"$BASE/jout" 2>"$BASE/jerr"
expect "joy refused outside pose"      "err joy only in pose"        jout
refute "refused joy never published"   "call joy 0.00 0.00 0.40"     jerr
expect "joy clamped to the stick range" "call joy 0.00 0.00 1.00 -0.30" jerr
expect "stale joy zeroed by dead-man"  "ev deadman_joy_zero"         jout
expect "zeroed sticks published"       "call joy 0.00 0.00 0.00 0.00" jerr
refute "any other verb ends pose joy"  "call joy 0.00 0.00 0.60"     jerr
[ "$(grep -c 'err joy only in pose' "$BASE/jout")" = 2 ] \
  && echo "  ok    joy refused again after wave_hand" \
  || { echo "  FAIL  expected joy refused before pose AND after wave_hand"; fail=1; }

[ "$fail" = 0 ] && echo PASS || { echo "--- out"; cat "$BASE/out" "$BASE/jout"; echo "--- err"; cat "$BASE/err" "$BASE/jerr"; }
exit "$fail"
