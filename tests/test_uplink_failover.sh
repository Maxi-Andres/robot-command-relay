#!/usr/bin/env bash
# Exercise uplink-failover.sh with stub `ip` and `ping`, on the real script.
#
#   bash tests/test_uplink_failover.sh      (also run by pytest: tests/test_uplink_failover.py)
#
# The stub `ip` keeps a routing table in a file; the stub `ping` answers from a script of
# up/down per tick. What must hold:
#   1. cable reachable            -> wired default stays at the primary metric
#   2. gateway gone for 3 ticks   -> wired default demoted above WiFi, never deleted outright
#   3. gateway back for 2 ticks   -> restored to the primary metric
#   4. one lost ping              -> no flap (hysteresis) — counted as switches in the log,
#      because a flap that ends where it started leaves the same final table
#   5. NetworkManager re-adding metric 100 while down -> demoted again on the next tick
#   6. the on-link 192.168.123.0/24 route is never touched
#   7. WiFi penalised by NetworkManager (+20000, metric 20600) -> the wired route is still
#      demoted ABOVE it. A fixed 900 lost to it after a cold boot on 2026-10-01.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
BASE=$(mktemp -d); trap 'rm -rf "$BASE"' EXIT
mkdir -p "$BASE/bin"
export ROUTES="$BASE/routes" PINGS="$BASE/pings" TICK="$BASE/tick"

cat >"$BASE/bin/ip" <<'EOF'
#!/usr/bin/env bash
# minimal `ip -4 route {show default dev X | add default via G dev X metric M | del ...}`
shift   # -4
[ "$1" = route ] || exit 1; shift
case "$1" in
  show) if [ "${3:-}" = dev ]; then grep '^default' "$ROUTES" | grep " dev $4 "
        else grep '^default' "$ROUTES"; fi ;;
  add)  line="default via $4 dev $6 metric $8"
        grep -qxF "$line" "$ROUTES" && exit 2; echo "$line" >>"$ROUTES" ;;
  del)  line="default via $4 dev $6 metric $8"
        grep -vxF "$line" "$ROUTES" >"$ROUTES.n"; mv "$ROUTES.n" "$ROUTES" ;;
esac
EOF
cat >"$BASE/bin/ping" <<'EOF'
#!/usr/bin/env bash
n=$(cat "$TICK" 2>/dev/null || echo 0); n=$((n + 1)); echo "$n" >"$TICK"
# the hook lets a case mutate the table between ticks, like NetworkManager would
[ -n "${HOOK_AT:-}" ] && [ "$n" = "$HOOK_AT" ] && echo "default via 192.168.123.1 dev eth0 metric 100" >>"$ROUTES"
[ "$(sed -n "${n}p" "$PINGS")" = up ]
EOF
chmod +x "$BASE/bin/ip" "$BASE/bin/ping"
export PATH="$BASE/bin:$PATH"

fail=0
run() {   # name, ping script, expected final wired metric(s), expected uplink switches
  local name="$1" script="$2" want="$3" want_sw="$4"
  printf '%s\n' \
    "default via 192.168.123.1 dev eth0 metric 100" \
    "default via 192.168.51.1 dev wlan0 metric ${WLAN_METRIC:-600}" \
    "192.168.123.0/24 dev eth0 scope link metric 100" >"$ROUTES"
  tr ' ' '\n' <<<"$script" >"$PINGS"; : >"$TICK"
  INTERVAL=0 MAX_TICKS=$(wc -w <<<"$script") bash "$HERE/../host/g1/uplink-failover.sh" 2>"$BASE/log"
  local got; got=$(grep '^default' "$ROUTES" | grep ' dev eth0 ' | awk '{print $NF}' | sort | tr '\n' ' ' | sed 's/ $//')
  local onlink; onlink=$(grep -c '^192.168.123.0/24 dev eth0' "$ROUTES")
  local sw; sw=$(grep -c 'uplink ->' "$BASE/log")
  if [ "$got" = "$want" ] && [ "$onlink" = 1 ] && [ "$sw" = "$want_sw" ]; then echo "PASS  $name  (wired metric $got, $sw switches)"
  else echo "FAIL  $name  want [$want]/$want_sw switches got [$got]/$sw onlink=$onlink"; sed 's/^/      /' "$BASE/log"; fail=1; fi
}

run "cable healthy"                 "up up up up"                     "100" 0
run "cable pulled -> WiFi"          "up down down down"               "900" 1
run "cable back -> cable"           "down down down up up"            "100" 2
run "one lost ping does not flap"   "up down up down up"              "100" 0
HOOK_AT=5 run "NM re-adds 100 while down" "down down down down down"  "900" 1
WLAN_METRIC=20600 run "WiFi at 20600 still wins"  "down down down down"  "20700" 1
exit $fail
