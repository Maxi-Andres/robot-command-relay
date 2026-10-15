# robot-command-relay

The remote command path for a Unitree robot. **It runs on the robot's own high-level
computer**, and it is the only component in this ecosystem that can make the robot move.

Split out of `robot-telemetry-agent` (formerly `robot-splunk-bridge`), which is read-only.
They shared a repo because they share a build recipe and a deploy target, but nothing else:
a name that said "splunk" made the robot's control path invisible to anyone auditing it.

## Why on the robot

DDS cannot be read or published across a subnet boundary on these robots. Measured on a Go2:

| From | DDS topics visible |
|---|---|
| The robot's own subnet (`192.168.123.0/24`) | **122** |
| Another subnet, routed (ping works, 1.3 ms) | **2** |
| Another subnet with explicit unicast DDS peers | **3** |

So the process that publishes commands has to live next to the robot's DDS, and what
crosses the network is HTTP. Full reasoning: `robot-splunk-docs/RED-Y-DDS.md`.


## Go2 vs G1

**`host/g1/uplink-failover.sh`** (+ `.service`, runs as root on the G1's PC2) is not part of
the relay: it is host config for the G1, kept here because the relay is what depends most on
the uplink. On the G1, eth0 is the internal bus and never loses carrier, so unplugging the
external cable never triggers a failover; the script probes the wired gateway and demotes
the wired default route when it stops answering. Test: `tests/test_uplink_failover.sh`.

**The senders.** `src/sender_core.hpp` is the safety envelope — allowlist by construction,
velocity clamp, dead-man switch, EOF stops the robot — and it is ONE file for both robots.
`src/go2_command_sender.cpp` (was `command_sender.cpp` until 2026-10-01) and
`src/g1_command_sender.cpp` add only their SDK client and verb table. `./build.sh` builds
both; `relay_server.py` runs the one `ROBOT_MODEL` names, with that robot's allowlist
(`VERBS_BY_MODEL`), and **refuses to start** if `SENDER_BIN` names another binary — the stale
`command_sender` included. Test the envelope with a fake robot: `bash tests/test_sender_core.sh`
(needs the SDK; CI skips it).

The G1's table is narrower and uses the FSM ids measured on this robot, not the SDK's
convenience calls — its header says why each verb is in or out. In short: no `damp`, no
`zero_torque`, no SDK `Squat()` (observed half-falling), walk is FSM 501.

| | Go2 | G1 |
|---|---|---|
| env | `relay.env.example` | `relay.g1.env.example` (`ROBOT_MODEL=g1`, clamps 1.2 / 0.5 / 1.2) |
| unit | `systemd/robot-command-relay.service` | `systemd/robot-command-relay.g1.service`, installed under the same name |

This repo serves both robots. Files prefixed `go2_` run only on the Go2, `g1_` (or under
`host/g1/`) only on the G1, unprefixed ones on both; `ROBOT_MODEL` picks the variant. The
full map — what runs where, per repo — is `robot-splunk-docs/QUE-CORRE-EN-CADA-ROBOT.md`.

## Shape

```
AI-VL executor (anywhere) ──HTTPS/VPN──▶ relay_server.py ──stdin──▶ command_sender ──DDS──▶ 🤖
                                         allowlist, rate limit      clamp, dead-man
                                         token, audit log           allowlist by construction
```

- **`relay_server.py`** — stdlib only (the robot has Python 3.8). Bearer token on every
  request, per-second rate limit, verb translation (no passthrough), audit log.
- **`src/command_sender.cpp`** — native Unitree SDK, no ROS2. Owns the safety envelope:
  velocity clamp, dead-man switch, and a dispatch table with no generic `api_id` path, so
  acrobatics simply do not exist here. Closing its stdin sends `StopMove` before exiting.

## Defences, from outside in

1. **Bearer token** on every request, separate from the Splunk token.
2. **Rate limit** per second, so a stuck caller cannot flood the control bus.
3. **No passthrough.** Verbs are translated to a fixed line protocol; an unknown verb is
   rejected here and would be rejected again by `command_sender`.
4. **Velocity clamp** to `MAX_VX` / `MAX_VY` / `MAX_VYAW`, whatever the caller asks for.
5. **Dead-man switch** in `command_sender`, not here, so it still protects the robot if
   this process hangs or is killed.
6. **EOF stops the robot.** If the HTTP layer dies, stdin closes and `StopMove` is sent.
7. **Audit log**: one line per command with time, source address, verb and result.

## Deploy to the robot

```bash
ssh unitree@<robot-jetson>
git clone https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
git clone <this-repo> ~/robot-command-relay
cd ~/robot-command-relay && ./build.sh

printf '%s' 'A-LONG-RANDOM-STRING' > ~/.relay_token && chmod 600 ~/.relay_token
sudo cp systemd/robot-command-relay.service /etc/systemd/system/
sudo systemctl enable --now robot-command-relay
```

Updating later: `git pull && ./build.sh && sudo systemctl restart robot-command-relay`.
The rebuild is **not** optional — the binary is gitignored, so a pull brings new source
without rebuilding it.

The token lives in `~/.relay_token`, outside the repo: a pull never overwrites it and a
push never leaks it.

## Build

```bash
UNITREE_SDK2_DIR=~/unitree_sdk2 ./build.sh     # x86_64 or aarch64, same command
```

## Run

```bash
RELAY_TOKEN=... ./relay_server.py              # spawns ./command_sender itself
```

## Configuration

| Variable | Default | Meaning |
|---|---|---|
| `RELAY_BIND` | `0.0.0.0` | Address to listen on. Should be the VPN-facing address only |
| `RELAY_PORT` | `8092` | HTTP port the AI-VL executor forwards to |
| `RELAY_TOKEN_FILE` | `~/.relay_token` | Read when `RELAY_TOKEN` is unset; refuses to start without a token |
| `DDS_IFACE` | `eth0` | Interface CycloneDDS binds to. **Required** — `Init(0, iface)` alone receives nothing |
| `MAX_VX` / `MAX_VY` / `MAX_VYAW` | `0.6` / `0.4` / `1.0` | Velocity clamps, enforced in `command_sender` |
| `DEADMAN_MS` | `1000` | A movement not refreshed within this window is stopped automatically |
| `RELAY_UDP_PORT` | `0` (off) | UDP port for continuous teleop: `move` and `stop_move` only, HMAC-authenticated with the relay token. See the UDP block in `relay_server.py` |
| `MAX_PER_SEC` | `20` | Rate limit |
| `AUDIT_LOG` | `/var/tmp/robot-relay-audit.log` | One line per command |
| `JOINT_BRIDGE_BIN` | `./g1_joint_bridge` | G1 only: the joint-state reader `GET /joints/stream` runs |
| `JOINT_MAX_CLIENTS` | `4` | G1 only: joint viewers at once; one more gets 503 |
| `JOINT_IDLE_S` | `10` | G1 only: the bridge keeps running this long after the last viewer leaves |
| `JOINT_RATE_HZ` | `30` | Read by `g1_joint_bridge`: lines per second (clamped 1-60) |
| `JOINT_STATE_TOPIC` | `rt/lowstate` | Read by `g1_joint_bridge`: `rt/lf/lowstate` (20 Hz) is the cheap alternative |
| `HANDS` | `1` | Read by `g1_joint_bridge` and `g1_joint_sender`: `0` = leave the BrainCo hands on `/dev/ttyUSB*` alone |
| `HAND_SPEED` | `300` | Read by `g1_joint_sender`: finger speed, of the hand's 1..1000 (capped at 600) |
| `JOINT_SENDER_BIN` | `./g1_joint_sender` | G1 only: what `POST /joints/control` spawns on "take" |
| `JOINT_CTL_MAX_PER_SEC` | `30` | G1 only: rate limit of `/joints/control`, apart from `/cmd`'s (a release is never limited) |
| `JOINT_CTL_ALLOW` | every arm joint + `waist_yaw` | Joints the page may move. Read by the relay (to report it) and by `g1_joint_sender` (to enforce it) |
| `JOINT_CTL_FSM_OK` | `500,501` | Read by `g1_joint_sender`: FSM ids control may start and stay in (Walk) |
| `JOINT_CTL_MAX_VEL` / `JOINT_CTL_EXCURSION` | `0.5` / `0` | Read by `g1_joint_sender` (and the excursion also by the relay, to report it): rad/s, capped at 1.0; and rad from where the joint was taken, `0` = off, the full URDF range |
| `JOINT_CTL_DEADMAN_MS` / `JOINT_CTL_RELEASE_MS` | `1000` / `5000` | Read by `g1_joint_sender`: silence before the arm is frozen / handed back |
| `JOINT_CTL_MAX_ACC` | `2.0` | Read by `g1_joint_sender`: rad/s² — how fast a joint's speed may change (smooth starts and stops), capped at 10 |
| `JOINT_CTL_STALL_FRAC` / `JOINT_CTL_STALL_MS` | `0.3` / `150` | Read by `g1_joint_sender`: a joint pushing over this share of its rated torque for this long, short of its command, stops where it is |

## Endpoints

| Endpoint | Purpose |
|---|---|
| `GET /health` | Sender liveness, allowed verbs, and what the robot reports about its own video, telemetry and safety limits |
| `POST /cmd` | `{verb, vx?, vy?, vyaw?, ts?}` — moves the robot. `ts` (the caller's clock) orders it against the UDP path: a `move` older than the last accepted command or stop is refused with 409 |
| UDP `RELAY_UDP_PORT` | `move` / `stop_move` datagrams, 40 bytes, HMAC; an authenticated one is answered with a 28-byte ack, anything else with silence |
| `GET /joints/stream` | **G1 only, read-only.** The joint state for the app's Joints page: one JSON line per tick (`src/g1_joint_bridge.cpp` documents the fields) until the caller hangs up, `{"stale": true}` when nothing new arrives within 1 s. Same token |
| `POST /joints/control` | **G1 only, MOVES the arms.** `{op: take}` → `{session, allowed, state, w}`; then `{op: target \| hand \| keepalive \| status \| release, session, joint?, q?, side?, finger?, pos?}` (`hand`: one finger, 0 open .. 1000 closed). One session at a time. Same token |

### The joint stream (G1)

`g1_joint_bridge` reads `rt/lowstate` and the two hands, and prints them; it cannot move
anything (no DDS publisher, and its Modbus code only builds read requests). The hands are
BrainCo Revo 2 on the FTDI board PC2 sees as `/dev/ttyUSB*` — left id 126, right id 127, 460800
baud; `src/brainco_modbus.hpp` has the registers and why (tested in CI:
`pytest tests/test_brainco_modbus.py`). The relay's user must be in `dialout` to open them. The relay starts it with the first viewer and stops it
`JOINT_IDLE_S` after the last one, keeps only its newest line (a slow viewer skips lines, never
delays another), and caps viewers at `JOINT_MAX_CLIENTS`. It runs inside the relay's unit, so
under the same limits (`CPUWeight`, not a quota: see the unit file). Test: `pytest tests/test_joint_stream.py`. Plan and next steps:
`.claude/roadmap/G1-CONTROL-ARTICULAR.md` on the workstation.

### Moving arm joints (G1)

`POST /joints/control` is how the Joints page moves the arms, the waist yaw and the fingers. `take` spawns `g1_joint_sender`,
which publishes `rt/arm_sdk` while the robot's own controller (PC1) keeps balancing; after the
release ramp the sender exits, and the session is over. **Every limit lives in
`src/joint_ctl_core.hpp`** — read its header: FSM gate (Walk only), no jump at the start (targets
begin where the joints are, weight ramps 0 → 1 in 2 s), allowlist, the URDF range (and an
optional maximum excursion), a speed cap with smooth acceleration (200 Hz), a dead-man (frozen after 1 s of silence, released
after 5 s), a release that ramps too, a stall guard (a joint that keeps pushing over 30% of its
rated torque for 150 ms while short of its command gives up where it is — the full range lets an arm reach
the body) and a temperature release (75 °C). It is plain C++ with no SDK, so CI tests it:
`pytest tests/test_joint_ctl_core.py` (builds `tests/joint_ctl_test.cpp` with g++).

While a session is open the relay keeps the robot standing: `take` sends `stop_move` first;
`move` and the arm actions are refused with 409 on HTTP and dropped on UDP; and a `stop_move`
from anywhere releases the arm. Only one page holds the arms (the session id), and the AI-VL
backend releases them when that page's socket closes. Test: `pytest tests/test_joint_control.py`.

The fingers ride the same session (rule 10 of `joint_ctl_core.hpp`): a hand is taken only if it
answered at `take`, its targets start where its fingers are, writes are clamped and at most one
per hand every 50 ms, and nothing is written after the release — the fingers then stay put. The
one write is `src/brainco_command.hpp` (finger positions and speeds, registers 1022-1033 only);
the read-only bridge does not include it, and `tests/test_brainco_modbus.py` fails if it ever
does. The hand's own protection current stays as BrainCo set it.

Allowed verbs: `move`, `stop_move`, `stand_up`, `stand_down`, `damp`, `balance_stand`,
`recovery_stand`, `sit`, `rise_sit`, `hello`, `stretch`, `scrape`, `heart`, `pose_on`,
`pose_off`, `keepalive` (Go2; the G1's narrower set is in `g1_command_sender.cpp`).

## Who calls this

The AI-VL robot executor (`unitree_ros2/robot_executor/`), when a robot's transport is set
to `relay` instead of `dds`. That mode is what works once the robot is itinerant and no
longer shares a subnet with the server.
