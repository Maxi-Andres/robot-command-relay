#!/usr/bin/env bash
# uplink-failover — cable when the cable works, WiFi when it does not, on the G1's PC2.
#
# WHY THIS EXISTS. On the G1, eth0 is PC2's port on the robot's INTERNAL switch (PC1 and the
# motion MCU hang off it too); the external RJ45 is only a tap off that switch. Unplugging
# the external cable therefore does NOT drop eth0's carrier — it stays UP, measured
# carrier=1 with nothing plugged in. NetworkManager never learns the cable is gone, the
# default route via eth0 (metric 100) keeps beating wlan0's (600), and everything the robot
# sends goes to a gateway that is no longer there. On a normal machine the kernel fails over
# on link loss; here link loss never happens, so reachability has to be probed instead.
#
# WHAT IT DOES. Pings the wired gateway out of eth0 every INTERVAL seconds. After FAIL_AFTER
# consecutive misses it DEMOTES the wired default route (re-adds it ABOVE every other default
# route) so traffic leaves by WiFi; after OK_AFTER consecutive replies it restores it.
#
# "Above every other default route", not a fixed number: NetworkManager adds 20000 to the
# metric of a device whose connectivity check fails, and with the cable black-holing all
# traffic, wlan0's check DOES fail — measured 2026-10-01 after a cold boot, wlan0 at 20600.
# A fixed demoted metric of 900 then still beat WiFi, and the robot was unreachable on every
# path with the failover reporting "uplink -> WiFi". The demoted metric is therefore computed
# each tick from the routes actually present.
#
# WHAT IT NEVER TOUCHES. The 192.168.123.0/24 on-link route: that is the bus to PC1, and the
# DDS the telemetry, relay and video depend on rides it whatever the uplink is.
#
# It reconciles every tick instead of acting only on transitions: NetworkManager re-adds
# its own metric-100 route on a reapply, and a loop that only acted once would lose that race.
set -uo pipefail

IFACE="${IFACE:-eth0}"
GW="${GW:-192.168.123.1}"
PRIMARY_METRIC="${PRIMARY_METRIC:-100}"
DEMOTED_FLOOR="${DEMOTED_FLOOR:-900}"   # lowest metric a demoted route gets
INTERVAL="${INTERVAL:-2}"
FAIL_AFTER="${FAIL_AFTER:-3}"
OK_AFTER="${OK_AFTER:-2}"
MAX_TICKS="${MAX_TICKS:-0}"          # 0 = forever; tests set it

log() { echo "[uplink] $*" >&2; }

# The metrics of the default routes via $GW on $IFACE, one per line.
wired_metrics() {
  ip -4 route show default dev "$IFACE" 2>/dev/null \
    | awk -v gw="$GW" '$3 == gw { m = 0; for (i = 1; i <= NF; i++) if ($i == "metric") m = $(i + 1); print m }'
}

# The highest metric of any default route that is NOT the wired one.
other_max() {
  ip -4 route show default 2>/dev/null \
    | awk -v dev="$IFACE" '{ d = ""; m = 0
        for (i = 1; i <= NF; i++) { if ($i == "dev") d = $(i + 1); if ($i == "metric") m = $(i + 1) }
        if (d != dev && m > mx) mx = m } END { print mx + 0 }'
}

# Demoted = above every other default route, and never below the floor.
demoted_metric() {
  local o; o=$(other_max)
  [ $((o + 100)) -gt "$DEMOTED_FLOOR" ] && echo $((o + 100)) || echo "$DEMOTED_FLOOR"
}

# Leave exactly one wired default route, at metric $1. Add first, then delete the others:
# the other way round opens a window with no wired default at all.
set_metric() {
  local want="$1" m
  if ! wired_metrics | grep -qx "$want"; then
    ip -4 route add default via "$GW" dev "$IFACE" metric "$want" 2>/dev/null \
      || { log "could not add default via $GW metric $want"; return 1; }
  fi
  for m in $(wired_metrics); do
    [ "$m" = "$want" ] || ip -4 route del default via "$GW" dev "$IFACE" metric "$m" 2>/dev/null
  done
}

state=up   # assume the cable until proven otherwise: the boot configuration is "wired first"
fails=0
oks=0
tick=0
log "iface=$IFACE gw=$GW metrics primary=$PRIMARY_METRIC demoted=above every other default (floor $DEMOTED_FLOOR)"

while :; do
  if ping -I "$IFACE" -c 1 -W 1 -q "$GW" >/dev/null 2>&1; then
    oks=$((oks + 1)); fails=0
  else
    fails=$((fails + 1)); oks=0
  fi

  if [ "$state" = up ] && [ "$fails" -ge "$FAIL_AFTER" ]; then
    state=down; log "gateway $GW unreachable on $IFACE ($fails misses): uplink -> WiFi"
  elif [ "$state" = down ] && [ "$oks" -ge "$OK_AFTER" ]; then
    state=up; log "gateway $GW back on $IFACE: uplink -> cable"
  fi

  # Only act when there IS a wired default to steer. With none (e.g. the profile is down),
  # adding one would invent a route nobody configured.
  if [ -n "$(wired_metrics)" ]; then
    if [ "$state" = up ]; then set_metric "$PRIMARY_METRIC"; else set_metric "$(demoted_metric)"; fi
  fi

  tick=$((tick + 1))
  [ "$MAX_TICKS" -gt 0 ] && [ "$tick" -ge "$MAX_TICKS" ] && break
  sleep "$INTERVAL"
done
