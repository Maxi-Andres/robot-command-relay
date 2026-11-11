#!/usr/bin/env python3
"""
relay_server — HTTP front end for remote robot commands. Runs ON THE ROBOT.

    AI-VL executor (anywhere) --HTTPS/VPN--> relay_server --stdin--> command_sender --DDS--> robot

Why it exists: DDS cannot cross a subnet boundary on these robots (measured: 122 topics from
the robot's own subnet, 2 from another one, 3 even with explicit unicast peers — see
robot-splunk-docs/RED-Y-DDS.md). So the process that publishes commands has to live on the robot,
and what crosses the network is HTTP.

This is the ONLY component in the project that can move the robot, so unlike the telemetry
agent it is not read-only. Defences, from outside in:

  * BEARER TOKEN on every request, separate from the Splunk token.
  * RATE LIMIT per second, so a stuck caller cannot flood the control bus.
  * NO PASSTHROUGH: verbs are translated to a fixed line protocol. An unknown verb is
    rejected here and would be rejected again by command_sender, which has no generic
    api_id path at all.
  * AUDIT LOG: one line per command with time, source address, verb and result.
  * DEAD-MAN SWITCH lives in command_sender, not here, so it still protects the robot if
    this process hangs or is killed.
  * BIND ADDRESS defaults to the VPN-facing address only. Never expose this to the internet.

Standard library only: nothing to install on the robot (Python 3.8 there).
"""
import hashlib
import hmac
import json
import math
import os
import queue
import re
import secrets
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BIND = os.environ.get("RELAY_BIND", "0.0.0.0")  # noqa: S104  # known finding P0-1: binds broadly, no auth yet
PORT = int(os.environ.get("RELAY_PORT", "8092"))
TOKEN_FILE = os.environ.get("RELAY_TOKEN_FILE", os.path.expanduser("~/.relay_token"))
TOKEN = os.environ.get("RELAY_TOKEN", "")
# WHICH ROBOT this relay drives. It picks the sender binary and the verb allowlist below; the
# HTTP layer, the rate limit, the UDP path and the audit log are the same for both robots.
ROBOT_MODEL = os.environ.get("ROBOT_MODEL", "go2")
if ROBOT_MODEL not in ("go2", "g1"):
    raise SystemExit(f"ROBOT_MODEL must be go2 or g1 (got {ROBOT_MODEL!r})")
SENDER = os.environ.get("SENDER_BIN", f"./{ROBOT_MODEL}_command_sender")
AUDIT_LOG = os.environ.get("AUDIT_LOG", "/var/tmp/robot-relay-audit.log")
MAX_PER_SEC = float(os.environ.get("MAX_PER_SEC", "20"))
REPLY_TIMEOUT = float(os.environ.get("REPLY_TIMEOUT", "3"))

# The robot's own speaker (POST /audio/*), through <model>_audio_sender. The two robots reach
# it differently — the Go2 through its undocumented audiohub, the G1 through the SDK's
# AudioClient — but the relay drives both with the same one-line protocol. The limits bound
# what one request can make the robot say: a phrase, not a playlist. The Go2's upload is
# ~100 ms per 60 KB block (measured 2026-10-14), so the reply timeout has room to spare.
AUDIO_SENDER = os.environ.get("AUDIO_SENDER_BIN", f"./{ROBOT_MODEL}_audio_sender")
AUDIO_MAX_BYTES = int(os.environ.get("AUDIO_MAX_BYTES", str(4 * 1024 * 1024)))
AUDIO_MAX_S = float(os.environ.get("AUDIO_MAX_S", "30"))
AUDIO_REPLY_TIMEOUT = float(os.environ.get("AUDIO_REPLY_TIMEOUT", "15"))
# Each robot's own volume scale: the Go2's VUI is 0-10 (it read back 10 at full volume), the
# G1's AudioClient is 0-100 (Unitree's example sets 100).
VOLUME_MAX = {"go2": 10, "g1": 100}[ROBOT_MODEL]

# UDP port for continuous teleop (`move` and `stop_move` only). 0 = off, the default: the
# HTTP path below is then the only way in, exactly as before. See the UDP block further down.
_udp = os.environ.get("RELAY_UDP_PORT", "0").strip()
RELAY_UDP_PORT = int(_udp) if _udp.isascii() and _udp.isdigit() and 1024 <= int(_udp) <= 65535 \
    else 0

# The G1's arm actions and dances (ids and why high five is out: g1_command_sender.cpp).
G1_ACTIONS = (
    {f"action_{a}" for a in ("hug", "clap", "face_wave", "left_kiss", "heart", "hands_up",
                             "x_ray", "right_hand_up", "reject", "shake_hand")}
    | {f"arm_{a}" for a in ("release_arm", "turn_back_wave", "two_hand_kiss", "right_kiss",
                            "right_heart", "high_wave", "box_win_left", "box_win_right",
                            "box_win_both", "hand_on_heart", "hands_up_right", "forward_push")}
    | {f"dance_{d}" for d in ("waist_drum", "scratch_head", "spin_discs", "throw_money")}
    | {"stop_dance"}
)
# The Go2's whole SDK action set since 2026-10-01: what can hurt the robot is gated by the
# executor's safe mode, not left out here.
GO2_ACROBATICS = {"dance1", "dance2", "front_jump", "front_pounce", "front_flip", "back_flip",
                  "left_flip", "handstand_on", "handstand_off", "walk_upright_on",
                  "walk_upright_off"}
GO2_GAITS = {f"gait_{g}" for g in ("classic", "free_walk", "trot_run", "static_walk",
                                     "economic", "cross_step")}

# Mirrors each sender's dispatch table (src/<robot>_command_sender.cpp). Kept here too so a bad
# verb is refused before it reaches the control process — defence in depth, not a single gate.
# tests/test_relay_boundary.py reads both and fails if they drift.
VERBS_BY_MODEL = {
    "go2": {"stop_move", "stand_up", "stand_down", "damp", "balance_stand",
            "recovery_stand", "sit", "rise_sit", "hello", "stretch", "scrape", "heart",
            "pose_on", "pose_off", "keepalive", *GO2_ACROBATICS, *GO2_GAITS},
    # Why each verb is in or out: g1_command_sender.cpp. Lists both walks and both runs;
    # verbs_for() keeps the pair matching the waist lock.
    "g1": {"stop_move", "stand_up", "walk_waist", "start", "run", "run_waist", "climb",
           "squat", "lie_up", "damp", "zero_torque", "balance_stand", "high_stand",
           "low_stand", "wave_hand", "keepalive", *G1_ACTIONS},
}


def verbs_for(model, waist_locked):
    """The verbs this relay accepts. On the G1 only ONE walk and ONE run: 500 / 801 (`start`,
    `run`) with the waist locked, 501 / 802 (`walk_waist`, `run_waist`) with it free — the lock
    is an app setting this process cannot read, so relay.env declares it (G1_WAIST_LOCK=1).
    g1_command_sender drops the same ones."""
    verbs = set(VERBS_BY_MODEL[model])
    if model == "g1":
        verbs -= {"walk_waist", "run_waist"} if waist_locked else {"start", "run"}
    return verbs


G1_WAIST_LOCK = os.environ.get("G1_WAIST_LOCK", "").strip() == "1"
VERBS = verbs_for(ROBOT_MODEL, G1_WAIST_LOCK)
# Verbs that carry numbers, handled before the table on both sides (sender_core.hpp): `move`,
# and `joy` on the robots whose sender publishes a joystick — the Go2, for its pose mode.
# The sender accepts `joy` only after pose_on; this layer just validates the numbers.
JOY_MODELS = {"go2"}
ARG_VERBS = {"move"} | ({"joy"} if ROBOT_MODEL in JOY_MODELS else set())

# The joint state stream (GET /joints/stream, G1 only): see JointStream below.
JOINT_BRIDGE = os.environ.get("JOINT_BRIDGE_BIN", "./g1_joint_bridge")
# Viewers at once. Each one costs a thread here and ~30 lines/s over the robot's uplink.
JOINT_MAX_CLIENTS = int(os.environ.get("JOINT_MAX_CLIENTS", "4"))
# How long the bridge keeps running after the last viewer leaves: long enough that reloading
# the page does not cost a new DDS participant, short enough not to sit on PC2 for nothing.
JOINT_IDLE_S = float(os.environ.get("JOINT_IDLE_S", "10"))
# A viewer that gets no new line for this long receives {"stale": true}: the page must be able
# to tell "the robot is still" from "nothing is arriving".
JOINT_HEARTBEAT_S = 1.0

# The G1's own odometry (GET /odom/stream, and what POST /walk/control steers by): see Walk.
# rt/state_estimator/odom_torso, read by g1_odom_bridge at ODOM_RATE_HZ (its default 20 Hz).
ODOM_BRIDGE = os.environ.get("ODOM_BRIDGE_BIN", "./g1_odom_bridge")

# WALK TO A POINT (POST /walk/control, G1 only, 2026-11-05). The robot walks to a pose given
# RELATIVE to where it stands, closing the loop on its own odometry, here on PC2: a dropped link
# cannot leave it walking, because the moves go to command_sender like any other and its 1 s
# dead-man still applies. Measured that day: 1.978 m reported for 2 m walked, 0 drift standing.
# What it does NOT have: eyes. Nothing here sees an obstacle — the operator with the remote is
# the obstacle detector until the lidar map exists (G1.md §6.2, phase 7b).
# 2026-11-11: 3 m was a first-day caution; the odometry drifts ~1 % (1.978 m for 2 m) and the
# lidar replans every frame, so a walk across a room is fine. The planner's grid grows with it.
WALK_MAX_M = float(os.environ.get("WALK_MAX_M", "10.0"))     # per request, straight-line
WALK_MAX_TURN = math.pi                                     # per request, either way
WALK_VX = float(os.environ.get("WALK_VX", "0.3"))           # m/s forward/back, at most
WALK_VY = float(os.environ.get("WALK_VY", "0.15"))          # m/s sideways
WALK_VYAW = float(os.environ.get("WALK_VYAW", "0.4"))       # rad/s
WALK_MIN_V = 0.08            # slower than this the robot marks time instead of stepping
# ...in Walk. In Run it stood 19-20 cm short marking time at 0.08 for good (2026-11-11): a guess,
# to be measured, that Run needs more to step at all.
WALK_MIN_V_RUN = float(os.environ.get("WALK_MIN_V_RUN", "0.15"))
# NOT GETTING CLOSER = DONE: within WALK_STALL_NEAR_M of the goal, less than WALK_STALL_GAIN_M
# closer in WALK_STALL_S, and it stops trying — the final turn, then success or uncertain by
# WALK_OK_POS as always, saying how short it stopped. Never again a walk that never ends.
WALK_STALL_NEAR_M = 0.35
WALK_STALL_GAIN_M = 0.02
WALK_STALL_S = 3.0
WALK_MIN_W = 0.12
WALK_GAIN = 1.0              # 1/s: speed = distance left x this, between the two above
WALK_YAW_GAIN = 1.5
WALK_POS_TOL = 0.06          # m: this close = arrived
# ...or this: once within WALK_NEAR_M of the goal, the goal falling BEHIND the way it came in =
# it walked past the point, and that is arrival too. A G1 step is longer than WALK_POS_TOL: on the
# robot (2026-11-11) it walked past, stepped back past, and again, for good. WALK_OK_POS still
# judges success.
WALK_NEAR_M = 0.25
WALK_YAW_TOL = math.radians(3)
WALK_HZ = 10.0               # moves sent per second
WALK_ODOM_STALE_S = 0.5      # no odometry for this long = stop, failure
WALK_SETTLE_S = 1.0          # after the stop, the robot finishes its step before measuring
WALK_OK_POS = 0.15           # what counts as success once settled (else "uncertain")
WALK_OK_YAW = math.radians(8)

# ITS EYES (2026-11-05): the Mid-360's obstacle map, from g1_lidar_bridge (GET /lidar/stream).
# The lidar sends to PC1 unless tools/livox_set_host.py pointed it at the bus broadcast; the bridge
# levels the points with the lidar's own gravity and keeps what stands 10 cm..1.5 m over the floor.
LIDAR_BRIDGE = os.environ.get("LIDAR_BRIDGE_BIN", "./g1_lidar_bridge")
# With the bridge built, a walk needs FRESH lidar: no eyes, no walk. 0 = walk blind, as before.
WALK_REQUIRE_LIDAR = os.environ.get("WALK_REQUIRE_LIDAR", "1").strip() != "0"
WALK_STOP_M = float(os.environ.get("WALK_STOP_M", "0.5"))  # m from the torso, along the way
# m each side of the way: the shoulders (0.45 m) and a little. NARROWER than the planner's
# clearance (WALK_CLEARANCE) on purpose: a planned way grazes its 0.35 m margin, and a stop corridor
# as wide as that stopped the robot on its own path (2026-11-05). This is for what steps INTO it.
WALK_HALF_WIDTH = 0.25
WALK_BLOCKED_WAIT_S = 3.0    # blocked this long (someone crossing?) and the walk gives up
WALK_LIDAR_STALE_S = 1.0     # no obstacle map for this long = stop, failure
LIDAR_VIEW_HZ = 2.0          # the page's copy: coarser and slower, it crosses the uplink
LIDAR_VIEW_RES = 0.10

JOINT_SENDER = os.environ.get("JOINT_SENDER_BIN", "./g1_joint_sender")
# Its own budget, apart from /cmd's: a dragged slider sends ~10 targets a second plus keepalives,
# and that must neither starve the drive pad nor be starved by it.
JOINT_CTL_MAX_PER_SEC = float(os.environ.get("JOINT_CTL_MAX_PER_SEC", "30"))
# Every joint rt/arm_sdk can move on this G1: the arms and the waist yaw (its roll and pitch have
# no motor, lock-waist variant). SECOND COPY of src/joint_ctl_core.hpp's table minus LOCKED;
# tests/test_joint_control.py reads the header and fails if they drift.
ARM_SDK_JOINTS = ["waist_yaw"] + [f"{side}_{j}" for side in ("left", "right") for j in (
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
    "wrist_roll", "wrist_pitch", "wrist_yaw")]
# Which joints the page may move (unset = all of the above). The sender reads the same variable
# and is the one that enforces it; the relay only reports it, so the page offers exactly those.
JOINT_CTL_ALLOW = [j for j in os.environ.get("JOINT_CTL_ALLOW", "").split(",") if j] \
    or list(ARM_SDK_JOINTS)
# Reported to the page for its sliders, enforced by the sender: rad from where a joint was when
# control was taken (0 = off, the full URDF range), and the margin kept inside that range.
try:
    JOINT_CTL_EXCURSION = max(0.0, float(os.environ.get("JOINT_CTL_EXCURSION", "0") or 0))
except ValueError:
    JOINT_CTL_EXCURSION = 0.0
JOINT_CTL_MARGIN = 0.05
# The sender's top speed (rule 5 of joint_ctl_core.hpp), mirrored with the same default and cap:
# a trajectory that asks for more is refused up front instead of falling behind its own clock.
try:
    JOINT_CTL_MAX_VEL = min(1.0, max(0.05, float(os.environ.get("JOINT_CTL_MAX_VEL", "0.5"))))
except ValueError:
    JOINT_CTL_MAX_VEL = 0.5
# Trajectories (op "trajectory" inside a control session, 2026-10-30): the arm motion a planner
# (GR00T, an IK, a replayed demo) hands over as points in time. The relay interpolates them and
# feeds the sender TRAJ_HZ points a second; every rule of joint_ctl_core.hpp still applies.
TRAJ_MAX_POINTS = 600
TRAJ_MAX_S = 60.0
# RETREAT BEFORE LETTING GO (2026-11-03). A released arm is taken back by PC1 to ITS pose —
# hanging straight down — and from a hand reached out over a table that path goes THROUGH the
# table: a hot shoulder released the arm mid-grasp and the hand hit it. A session may name a
# SAFE pose at `take` (the hand tucked in front of the belly, behind any table edge); then, when
# a held joint reaches JOINT_CTL_RETREAT_C (under the sender's own 75 °C release, rule 9), when
# the client goes quiet for JOINT_CTL_CLIENT_LOST_S (under the sender's 1 s dead-man), or when
# a release is asked away from it, the relay first moves the arm there, then releases. Without
# a safe pose nothing changes, and the sender's own limits stay the last line either way.
JOINT_CTL_RETREAT_C = int(os.environ.get("JOINT_CTL_RETREAT_C", "68"))
JOINT_CTL_CLIENT_LOST_S = float(os.environ.get("JOINT_CTL_CLIENT_LOST_S", "0.9"))
RETREAT_VEL_FRACTION = 0.5     # of JOINT_CTL_MAX_VEL
RETREAT_AT_RAD = 0.15          # a release this close to the safe pose needs no retreat
TRAJ_HZ = 20.0
TRAJ_TOL = 0.05          # rad: every joint this close to its last point = arrived
TRAJ_SETTLE_S = 3.0      # after the last point, how long it may take to get there
# Where each arm_sdk joint sits in rt/lowstate's 29 (the joint stream's "q"), to check arrival.
BODY_INDEX = {"waist_yaw": 12,
              **{name: 15 + k for k, name in enumerate(ARM_SDK_JOINTS[1:8])},
              **{name: 22 + k for k, name in enumerate(ARM_SDK_JOINTS[8:])}}
# What the sender's replies look like: "<ok|err> ... state=<s> w=<weight>".
_JOINT_REPLY = re.compile(r"^(ok|err)\b(.*?)\s+state=(\w+)\s+w=([0-9.]+)$")
_JOINT_NAME = re.compile(r"^[a-z_]{1,40}$")


def log(msg):
    print(f"[relay] {msg}", file=sys.stderr, flush=True)


def sender_mismatch(sender, model):
    """Why `sender` is the wrong binary for `model`, or "" if it is the right one.

    THE TRAP THIS CATCHES: until 2026-10-01 the sender was `command_sender`, and the installed
    unit and relay.env both pin SENDER_BIN to that path. After a pull + build the new binary is
    `go2_command_sender` but the OLD one is still on disk, so a stale SENDER_BIN keeps running
    yesterday's code with no error anywhere. Worse, a G1 pointed at a Go2 sender would accept
    Go2 verbs. So the file name must say the robot.
    """
    name = os.path.basename(sender)
    want = f"{model}_command_sender"
    return "" if name == want else f"SENDER_BIN is {sender!r} but ROBOT_MODEL={model} needs {want}"


# --------------------------------------------------------------------------- #
# Video tuning — the authenticated front door for the live-view knobs
#
# WHY IT LIVES HERE: mjpeg_server binds 0.0.0.0 with no authentication (known finding
# P0-1), so a WRITE route on it reachable from the network would make that worse. This
# relay already has the token, and its /health already reports the video config. So it
# validates, and only then calls mjpeg_server from 127.0.0.1 — which is the only address
# that endpoint accepts.
#
# The allowlist is the same shape as VERBS above, and for the same reason: a generic
# "write any key" would let a typo (NIC=eth9) take the video off the air on a robot in the
# field, recoverable only over SSH — the exact trip this feature exists to save.
#
# Two kinds of key live here, and the difference is visible to the caller (see `live`
# below). The live ones apply instantly; the rest can only be WRITTEN to video.env,
# because restarting the video service needs privileges this process does not have.
# Saving them is still the point: it is what removes the SSH trip.
# --------------------------------------------------------------------------- #
VIDEO_ENV = os.environ.get(
    "VIDEO_ENV", "/home/unitree/robot-video-pipeline/robot/video.env")
MJPEG_LOCAL = os.environ.get("MJPEG_LOCAL", "http://127.0.0.1:8093")
# camera_source, the process in front of mjpeg_server that picks WHICH camera is sent. Same
# localhost-only contract as MJPEG_LOCAL. Its choices mirror camera_source.CAMERAS — two repos,
# one machine, so a test on this side keeps the copies honest.
CAMERA_LOCAL = os.environ.get("CAMERA_LOCAL", "http://127.0.0.1:8094")
CAMERAS = ("front", "rs_color", "rs_depth")
# Newest battery reading, written by the telemetry shipper (see its snapshot_battery()).
BATTERY_FILE = os.environ.get("BATTERY_FILE", "/var/tmp/robot-battery.json")
# Past this age the reading is reported as stale instead of as the truth. The reader's
# default period is 3 s, so anything beyond a few periods means the telemetry pipeline
# stopped — and a battery percentage frozen at 95% is the easiest stale value to believe.
BATTERY_STALE_S = float(os.environ.get("BATTERY_STALE_S", "20"))
# key -> (env name, cast, min, max, live)
#
# live=True  : mjpeg_server reads it per frame, so it applies instantly with no restart.
#              These three mirror mjpeg_server.LIVE_PARAMS — the two run in different repos
#              on the same machine, so the boundary forbids a shared module and a test on
#              each side keeps them honest. Change one, change the other.
# live=False : run-video.sh interpolates it into the gst-launch command line when the
#              service starts, so it can only be SAVED here; it takes effect on the next
#              restart of robot-video. Writing them is still worth it — it is the whole
#              reason this endpoint exists — but nothing here can apply them.
#
# The ranges are not decoration. BITRATE sat at 60000 on the robot (60 kbps for 1080p
# H.264, almost certainly a missing zero) and nobody noticed because nothing checked it;
# the floor below refuses that value now.
VIDEO_PARAMS = {
    "fps":     ("MJPEG_FPS", float, 0.0, 60.0, True),
    "width":   ("MJPEG_WIDTH", int, 0, 1920, True),
    "quality": ("MJPEG_QUALITY", int, 1, 100, True),
    # The DRIVE view: all-intra H.264 over UDP, what Drive shows. Live on the publisher too,
    # and they were missing here, which is why tuning them on 2026-10-07 took SSH.
    "h264_qp":    ("H264_QP", int, 10, 51, True),
    "h264_width": ("H264_WIDTH", int, 64, 1920, True),
    "bitrate": ("BITRATE", int, 200000, 8000000, False),
    "maxfps":  ("MAXFPS", int, 0, 30, False),
    "idr":     ("IDR_FRAMES", int, 1, 300, False),
    "nvr":     ("NVR_ENABLE", int, 0, 1, False),
}
LIVE_KEYS = {k for k, v in VIDEO_PARAMS.items() if v[4]}


def h264_height_for(width):
    """Twin of mjpeg_server.h264_height_for: 16:9, even. Persisting H264_WIDTH without the
    height it implies would bring the stretched picture back on the next restart."""
    return max(2, round(width * 9 / 16 / 2) * 2)


def validate_video(body):
    """{fps,width,quality} -> {ENV_NAME: value}. Raises ValueError.

    Everything is validated BEFORE anything is written, so a bad value in a two-key request
    cannot leave the file half-updated.
    """
    unknown = set(body) - set(VIDEO_PARAMS)
    if unknown:
        raise ValueError(f"unknown parameter(s): {sorted(unknown)}; "
                         f"allowed: {sorted(VIDEO_PARAMS)}")
    if not body:
        raise ValueError("nothing to set")
    out = {}
    for key, raw in body.items():
        name, cast, lo, hi, _live = VIDEO_PARAMS[key]
        try:
            value = cast(raw)
        except (TypeError, ValueError):
            raise ValueError(f"'{key}' must be {cast.__name__}, got {raw!r}") from None
        if not lo <= value <= hi:
            raise ValueError(f"'{key}' must be between {lo} and {hi}, got {value}")
        out[name] = value
    if "H264_WIDTH" in out:
        out["H264_HEIGHT"] = h264_height_for(out["H264_WIDTH"])
    return out


def split_live(body):
    """(live subset, restart-only subset). Only the first can be pushed to the publisher."""
    live = {k: v for k, v in body.items() if k in LIVE_KEYS}
    deferred = {k: v for k, v in body.items() if k not in LIVE_KEYS}
    return live, deferred


def read_env_file(path):
    """KEY=VALUE pairs from an env file. Missing file = {}, never an error."""
    out = {}
    try:
        with open(path, encoding="utf-8") as fh:
            for line in fh:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, _, v = line.partition("=")
                    out[k.strip()] = v.strip()
    except OSError:
        pass
    return out


def write_env_keys(path, updates):
    """Replace these keys in an env file, keeping every other line and its order.

    Atomic (write-then-rename): a half-written video.env would leave run-video.sh unable to
    start, and this file is read on every restart of the publisher.

    Twin of `_set_env_keys` in unitree_ros2/robot_executor/robot_executor_service.py — the
    two live on different machines so the boundary forbids sharing the module. Each has its
    own test.
    """
    lines = []
    try:
        with open(path, encoding="utf-8") as fh:
            lines = fh.read().splitlines()
    except OSError:
        pass
    remaining = dict(updates)
    out = []
    for line in lines:
        key = line.split("=", 1)[0].strip() if "=" in line and not line.startswith("#") \
            else ""
        if key in remaining:
            out.append(f"{key}={remaining.pop(key)}")
        else:
            out.append(line)
    for key, value in remaining.items():
        out.append(f"{key}={value}")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write("\n".join(out).rstrip("\n") + "\n")
    os.replace(tmp, path)


def mjpeg_live():
    """The publisher's RUNNING knobs, or {} if it is not answering.

    Never raises: this is reporting, and a video publisher that is down must not take the
    relay's status endpoint down with it.
    """
    try:
        with urllib.request.urlopen(f"{MJPEG_LOCAL}/health", timeout=2) as r:
            d = json.loads(r.read())
        out = {"fps": d.get("fps_cap") or 0, "width": d.get("width"),
               "quality": d.get("quality")}
        if d.get("h264_qp") is not None:
            out["h264_qp"] = d["h264_qp"]
        size = str(d.get("h264_size") or "")
        if "x" in size:
            out["h264_width"] = int(size.split("x")[0])
        return out
    except Exception:
        return {}


def validate_camera(value):
    """The camera name, or ValueError. An allowlist like every other knob here."""
    if value not in CAMERAS:
        raise ValueError(f"'camera' must be one of {list(CAMERAS)}, got {value!r}")
    return value


def camera_live():
    """{'camera': …, 'camera_error': …} from camera_source, or {} if it is not answering.

    Never raises, for the same reason as mjpeg_live(). camera_error is why the last switch
    did not land (no RealSense, library missing) — the UI shows it instead of guessing."""
    try:
        with urllib.request.urlopen(f"{CAMERA_LOCAL}/camera", timeout=2) as r:
            d = json.loads(r.read())
        return {"camera": d.get("camera"), "camera_error": d.get("error"),
                "cameras": d.get("choices")}
    except Exception:
        return {}


def camera_apply(camera):
    """Switch the camera on localhost. {'error': …} only when it could not be ASKED.

    A switch that was asked but fell back (no RealSense) is not an error here: the picture is
    still on, and the answer says which camera is running and why. The timeout covers
    camera_source's own wait for the RealSense to open (CAMERA_SWITCH_WAIT_S, 3 s) and stays
    inside the executor's 5 s to this relay."""
    try:
        req = urllib.request.Request(
            f"{CAMERA_LOCAL}/camera", data=json.dumps({"camera": camera}).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=4) as r:
            d = json.loads(r.read() or b"{}")
        out = {"camera": d.get("camera"), "camera_error": d.get("error")}
        if d.get("pending"):
            out["camera_pending"] = d["pending"]
        return out
    except urllib.error.HTTPError as exc:
        detail = exc.read()[:200].decode("utf-8", "replace")
        return {"error": f"camera switch refused it: HTTP {exc.code}: {detail}"}
    except Exception as exc:
        return {"error": f"camera switch unreachable on {CAMERA_LOCAL}: {exc} "
                         f"(is the video service running a version with camera_source?)"}


def depth_fetch(frames):
    """(png bytes, info header) from camera_source's GET /depth, or (None, (status, dict)).

    The robot side decides whether snapshots exist at all (DEPTH_SNAPSHOT, off on the Go2), so
    its 404 is passed through as it is: the answer says why, not just "not found". The timeout
    covers the frames it medians (5 at 15 fps) plus opening the node when depth is not live."""
    try:
        with urllib.request.urlopen(f"{CAMERA_LOCAL}/depth?frames={int(frames)}",
                                    timeout=8) as r:
            return r.read(), r.headers.get("X-Depth-Info", "{}")
    except urllib.error.HTTPError as exc:
        try:
            body = json.loads(exc.read() or b"{}")
        except ValueError:
            body = {}
        return None, (exc.code, {"ok": False,
                                 "error": body.get("error") or f"HTTP {exc.code}"})
    except Exception as exc:
        return None, (502, {"ok": False, "error": f"camera_source unreachable on "
                                                  f"{CAMERA_LOCAL}: {exc}"})


def mjpeg_apply(body):
    """Push {fps,width,quality} to the publisher on localhost. {'error': …} on failure."""
    try:
        req = urllib.request.Request(
            f"{MJPEG_LOCAL}/config", data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=3) as r:
            d = json.loads(r.read() or b"{}")
        return {k: d.get(k) for k in VIDEO_PARAMS if k in d}
    except urllib.error.HTTPError as exc:
        detail = exc.read()[:200].decode("utf-8", "replace")
        return {"error": f"publisher refused it: HTTP {exc.code}: {detail}"}
    except Exception as exc:
        return {"error": f"video publisher unreachable on {MJPEG_LOCAL}: {exc}"}


def parse_wav(data):
    """{'rate','channels','seconds'} for a playable WAV, or ValueError saying why not.

    The speaker takes 16-bit PCM WAV (Piper's 22.05 kHz mono and 44.1 kHz were both heard on
    the robot, 2026-10-14). Everything else is refused here rather than handed to a service
    whose failure modes nobody has documented: a request that cannot play must say so."""
    if len(data) > AUDIO_MAX_BYTES:
        raise ValueError(f"audio is {len(data)} bytes; the limit is {AUDIO_MAX_BYTES}")
    if len(data) < 44 or data[:4] != b"RIFF" or data[8:12] != b"WAVE":
        raise ValueError("not a WAV file (RIFF/WAVE header missing)")
    fmt = None
    i = 12
    while i + 8 <= len(data):
        cid, size = data[i:i + 4], struct.unpack("<I", data[i + 4:i + 8])[0]
        if cid == b"fmt " and size >= 16:
            fmt = struct.unpack("<HHIIHH", data[i + 8:i + 24])
        elif cid == b"data":
            if fmt is None:
                raise ValueError("WAV data chunk comes before its fmt chunk")
            tag, channels, rate, byte_rate, _align, bits = fmt
            if tag != 1 or bits != 16:
                raise ValueError(f"must be 16-bit PCM (format {tag}, {bits} bits)")
            if channels not in (1, 2) or not 8000 <= rate <= 48000:
                raise ValueError(f"must be mono or stereo at 8-48 kHz ({channels} ch, {rate} Hz)")
            seconds = min(size, len(data) - i - 8) / byte_rate
            if seconds > AUDIO_MAX_S:
                raise ValueError(f"{seconds:.1f} s of audio; the limit is {AUDIO_MAX_S:g} s")
            return {"rate": rate, "channels": channels, "seconds": round(seconds, 2)}
        i += 8 + size + (size & 1)
    raise ValueError("WAV has no data chunk")


# The G1's microphone (GET /audio/listen). PC1 streams it to multicast on the robot's internal
# network only while the voice service is in mic mode, which the audio sender switches
# (`mic on` / `mic off`, voice API 1008 — see g1_audio_sender.cpp). Measured 2026-10-19 on the
# G1: 16 kHz mono s16le in 5120-byte packets, 6.3 a second, from 192.168.123.161. That
# multicast never crosses the robot's Wi-Fi, so this relay is what carries it out, a few
# seconds per request and only when asked: the stream is ~32 KB/s.
MIC_GROUP = os.environ.get("MIC_GROUP", "239.168.123.161")
MIC_PORT = int(os.environ.get("MIC_PORT", "5555"))
MIC_RATE = 16000
MIC_MAX_S = float(os.environ.get("MIC_MAX_S", "15"))
MIC_FIRST_PACKET_S = float(os.environ.get("MIC_FIRST_PACKET_S", "3"))
# Mic mode stays on this long after the last listen, so a conversation does not pay the
# stream's start-up on every phrase; then it is switched off (it also runs the robot's ASR).
MIC_IDLE_OFF_S = float(os.environ.get("MIC_IDLE_OFF_S", "60"))


def mic_local_ip():
    """This machine's address on the robot's internal network, which the group is joined on.
    A UDP socket 'connected' towards PC1 makes the OS pick the route; nothing is sent."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        try:
            s.connect(("192.168.123.161", 1))
            ip = s.getsockname()[0]
        except OSError:
            return "0.0.0.0"  # noqa: S104 - let the OS pick the interface
    return ip if ip.startswith("192.168.123.") else "0.0.0.0"  # noqa: S104


def validate_listen_seconds(value):
    """How long one /audio/listen records, as a float in (0, MIC_MAX_S], or ValueError."""
    try:
        seconds = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"'seconds' must be a number, got {value!r}") from None
    if not 0 < seconds <= MIC_MAX_S or math.isnan(seconds):
        raise ValueError(f"'seconds' must be more than 0 and at most {MIC_MAX_S:g}")
    return seconds


def pcm_to_wav(pcm, rate=MIC_RATE):
    """16-bit mono PCM bytes as a WAV file."""
    data = bytes(pcm[: len(pcm) // 2 * 2])
    return (b"RIFF" + struct.pack("<I", 36 + len(data)) + b"WAVEfmt "
            + struct.pack("<IHHIIHH", 16, 1, 1, rate, rate * 2, 2, 16)
            + b"data" + struct.pack("<I", len(data)) + data)


def mic_socket():
    """A UDP socket joined to the mic multicast. Several may be open at once (a listen and a
    stream): SO_REUSEADDR lets each get every packet."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
    sock.bind(("", MIC_PORT))
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                    socket.inet_aton(MIC_GROUP) + socket.inet_aton(mic_local_ip()))
    return sock


def record_mic(seconds, open_socket=None, clock=time.monotonic):
    """`seconds` of the mic multicast as PCM bytes. TimeoutError when no packet comes within
    MIC_FIRST_PACKET_S (mic mode off, or PC1 not streaming). `open_socket` is for tests."""
    want = int(seconds * MIC_RATE) * 2
    sock = (open_socket or mic_socket)()
    pcm = bytearray()
    try:
        sock.settimeout(MIC_FIRST_PACKET_S)
        # Bounded by wall time too: a stream that stops half-way must not hang the request.
        end = clock() + MIC_FIRST_PACKET_S + seconds + 2.0
        while len(pcm) < want and clock() < end:
            try:
                data, _src = sock.recvfrom(65535)
            except socket.timeout:
                if not pcm:
                    raise TimeoutError(
                        f"no microphone audio within {MIC_FIRST_PACKET_S:g} s") from None
                break
            pcm += data
    finally:
        sock.close()
    return bytes(pcm[:want])


class Mic:
    """Mic mode on demand. A listen turns it on and leaves it on MIC_IDLE_OFF_S, so the next
    phrase does not wait for the stream to start; an open stream (the app's "robot mic" switch)
    keeps it on, and closing the last stream turns it off AT ONCE — switching the robot's mic
    off in the app must mean off. One listen at a time; a second is refused, not queued."""

    def __init__(self, audio, record=record_mic, idle_off_s=None, max_streams=2):
        self.audio = audio
        self.record = record
        self.idle_off_s = MIC_IDLE_OFF_S if idle_off_s is None else idle_off_s
        self.max_streams = max_streams
        self.busy = threading.Lock()
        self.state = threading.Lock()
        self.on = False
        self.streams = 0
        self.timer = None

    def _ensure_on(self):              # caller holds self.state
        if self.timer:
            self.timer.cancel()
            self.timer = None
        if not self.on:
            reply = self.audio.send("mic on")
            if not reply.startswith("ok"):
                raise RuntimeError(f"the robot did not turn its mic on: {reply}")
            self.on = True

    def _off_now(self):                # caller holds self.state
        if self.on and self.streams == 0:
            reply = self.audio.send("mic off")
            self.on = not reply.startswith("ok")
            log(f"mic off: {reply}")

    def listen(self, seconds):
        """PCM bytes, or raises RuntimeError (busy, or the sender refused) / TimeoutError."""
        if not self.busy.acquire(blocking=False):
            raise RuntimeError("busy: another listen is recording")
        try:
            with self.state:
                self._ensure_on()
            return self.record(seconds)
        finally:
            with self.state:
                if self.on and self.streams == 0:
                    self.timer = threading.Timer(self.idle_off_s, self.off)
                    self.timer.daemon = True
                    self.timer.start()
            self.busy.release()

    def open_stream(self):
        """Count one more stream and make sure the mic is on. RuntimeError when there are
        already max_streams, or the robot refused."""
        with self.state:
            if self.streams >= self.max_streams:
                raise RuntimeError(f"busy: {self.streams} mic streams already open")
            self._ensure_on()
            self.streams += 1

    def close_stream(self):
        with self.state:
            self.streams = max(0, self.streams - 1)
            if self.streams == 0 and not self.busy.locked():
                self._off_now()

    def off(self):
        with self.state:
            self._off_now()


def validate_volume(value):
    """The volume level as an int 0-10, or ValueError."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value):
        raise ValueError(f"'level' must be a whole number, got {value!r}")
    if not 0 <= value <= VOLUME_MAX:
        raise ValueError(f"'level' must be between 0 and {VOLUME_MAX}, got {value}")
    return int(value)


def validate_rgb(body):
    """The head LED colour {"r", "g", "b"} as three ints 0-255, or ValueError."""
    if not isinstance(body, dict):
        raise ValueError("expected a JSON object with 'r', 'g' and 'b'")
    out = []
    for key in ("r", "g", "b"):
        value = body.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value):
            raise ValueError(f"'{key}' must be a whole number, got {value!r}")
        if not 0 <= value <= 255:
            raise ValueError(f"'{key}' must be between 0 and 255, got {value}")
        out.append(int(value))
    return tuple(out)


def audit(addr, verb, detail, result):
    line = (f'{time.strftime("%Y-%m-%dT%H:%M:%S")} src={addr} verb={verb} '
            f'{detail} result={result}\n')
    try:
        with open(AUDIT_LOG, "a") as fh:
            fh.write(line)
    except OSError as exc:                      # never let logging break control
        log(f"audit write failed: {exc}")


def _proc_env(pattern):
    """Environment of the first process matching `pattern`, or {}.

    Read from /proc so what is reported is what the RUNNING process uses, not what a config
    file currently says — those diverge the moment someone edits a file without restarting.
    """
    try:
        pids = subprocess.run(["pgrep", "-f", pattern],
                              capture_output=True, text=True, timeout=3).stdout.split()
        if not pids:
            return {}
        with open(f"/proc/{pids[0]}/environ", "rb") as fh:
            return dict(
                kv.split("=", 1) for kv in fh.read().decode("utf-8", "replace").split("\0")
                if "=" in kv)
    except Exception:
        return {}


def battery_status():
    """Charge, health and whether it is charging. {} when there is no reading to give.

    THE SOURCE is the telemetry shipper's snapshot file: the battery already crosses that
    process on its way to Splunk, so the app gets it without a second DDS subscriber. This
    endpoint only reads and interprets.

    ⚠️ THE SIGN CONVENTION IS THE ONE THING TO GET RIGHT, and it is MEASURED, not assumed.
    Same Go2, same minute, 2026-09-21, lifted off its wireless dock between the two readings:

        on the dock   current  +471 / +475     volt 32783 mV
        off the dock  current -5678 / -5557    volt 32538 mV

    So POSITIVE = CHARGING. The magnitudes corroborate it: 5.6 A at 32.5 V is ~185 W, which is
    a Go2 standing with LiDAR, DDS and video running, and the pack voltage dropped 245 mV the
    moment the charger left. One constant, in one place, because getting it backwards makes
    the app say "charging" while the robot drains — a wrong answer that looks right, which is
    worse than no answer.

    Never raises: a missing or half-written file must not take the relay's status endpoint
    down with it.
    """
    try:
        with open(BATTERY_FILE, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError):
        return {}
    if not isinstance(d, dict) or "soc" not in d:
        return {}
    age = round(time.time() - float(d.get("at") or 0), 1)
    current = d.get("current")
    return {
        "percent": d.get("soc"),
        "charging": (current > 0) if isinstance(current, (int, float)) else None,
        "current": current,
        "volts": round((d.get("volt_mv") or 0) / 1000.0, 2),
        "cycles": d.get("cycles"),
        "temp_c": max(x for x in (d.get("mcu_ntc"), d.get("bq_ntc")) if x is not None)
        if (d.get("mcu_ntc") is not None or d.get("bq_ntc") is not None) else None,
        "age_s": age,
        # Reported, not hidden: a consumer that trusts a frozen reading will happily show 95%
        # on a robot that has been off for an hour.
        "stale": age > BATTERY_STALE_S,
    }


def telemetry_status():
    """Where the robot ships telemetry to. Configured ON the robot, so the app can only
    read it — which is exactly why it is reported here instead of being guessed."""
    env = _proc_env("hec_shipper")
    if not env:
        return {"running": False}
    url = env.get("HEC_URL", "")
    return {
        "running": True,
        "hec_url": url,
        "index": env.get("HEC_INDEX", ""),
        "robot_name": env.get("ROBOT_NAME", ""),
        "period_s": env.get("PERIOD", ""),
        "daily_byte_cap": env.get("DAILY_BYTE_CAP", ""),
    }


def limits_status():
    """The safety envelope this relay enforces. Read-only from the app on purpose: the
    clamps and the dead-man window are the robot's own guarantees, not the caller's."""
    return {
        "max_vx": os.environ.get("MAX_VX", "0.6"),
        "max_vy": os.environ.get("MAX_VY", "0.4"),
        "max_vyaw": os.environ.get("MAX_VYAW", "1.0"),
        "deadman_ms": os.environ.get("DEADMAN_MS", "1000"),
        "max_per_sec": str(MAX_PER_SEC),
        "dds_iface": os.environ.get("DDS_IFACE", "eth0"),
    }


def video_status():
    """What the video publisher is ACTUALLY configured with, right now.

    Read from /proc/<pid>/environ of the running run-video.sh — not from video.env — so it
    reports what is in effect rather than what a file says. Someone editing the file without
    restarting the service would otherwise make this lie, and the whole point is that the
    app can trust it instead of hardcoding an address.
    """
    try:
        env = _proc_env("run-video.sh")
        if not env:
            return {"running": False}
        return {
            "running": True,
            "publish_host": env.get("PUBLISH_HOST", ""),
            "proto": env.get("PROTO", ""),
            "stream": env.get("STREAM", ""),
            "port": env.get("PUBLISH_PORT", "1935" if env.get("PROTO") == "rtmp" else ""),
            "bitrate": env.get("BITRATE", ""),
            "maxfps": env.get("MAXFPS", ""),
        }
    except Exception as exc:                     # never let this break the relay
        return {"running": False, "error": str(exc)}


# --------------------------------------------------------------------------- #
# Continuous teleop over UDP — `move` and `stop_move`, nothing else.
#
# WHY: over Starlink (measured 2026-09-23: 3.4-4% loss, in bursts) every HTTP command opens a
# new TCP connection — two round trips, 178 ms median for a keepalive, and a lost SYN costs a
# full second before TCP retries. For a `move` that is superseded 100 ms later anyway, none
# of that waiting buys anything: a lost datagram is simply replaced by the next one, and the
# dead-man in command_sender still stops the robot if they all stop arriving.
#
# WHY ONLY THESE TWO: the discrete verbs (stand_up, sit, damp, ...) are rare, need their
# answer, and would need retransmission — i.e. TCP. They stay on HTTP. A stop comes over
# BOTH paths from the executor, and whichever lands first wins.
#
# AUTHENTICATION, per datagram: HMAC-SHA256 keyed with the relay token, truncated to 16 bytes.
# The token itself never crosses the network on this path (on the HTTP path it does, in the
# clear, in every Authorization header). A datagram that fails any check gets NO reply, so
# this port cannot be used to reflect traffic at anyone.
#
# ORDER AND REPLAY, with one number: `ts`, the executor's wall clock when it sent the
# command. It must be STRICTLY greater than the last one accepted, and within
# UDP_CLOCK_WINDOW_S of this robot's clock (both machines run NTP; measured offset 3-16 ms).
# The same `ts` rides on the HTTP commands, and a stop from either path raises the bar — so
# a `move` delayed in the network can never land after a stop and restart the robot, which
# is the race the engineering standard names (§3, "commands that supersede each other").
#
# WIRE FORMAT, little-endian. SECOND COPY in unitree_ros2/robot_executor
# (`RelayTransport`); both test suites assert the same golden bytes — change one, change both.
#   command  "RC" | version u8 | kind u8 (1 move, 2 stop) | ts f64 | vx f32 | vy f32
#            | vyaw f32 | mac[16]                                          = 40 bytes
#   ack      "RA" | version u8 | status u8 (0 ok, 1 sender error) | ts f64 | mac[16] = 28
# --------------------------------------------------------------------------- #
UDP_VERSION = 1
UDP_CMD = struct.Struct("<2sBBdfff")
UDP_ACK = struct.Struct("<2sBBd")
UDP_MAC_LEN = 16
UDP_KIND_MOVE = 1
UDP_KIND_STOP = 2
# Wider than any NTP error seen here, narrower than anything a replay would need.
UDP_CLOCK_WINDOW_S = 1.0
# A stop that carries no `ts` (an executor older than this file) still has to beat every
# move already in flight. Robot clock plus a margin larger than the worst one-way delay
# measured over Starlink (379 ms RTT) does that; a new move within it is refused, which is
# what someone who just pressed stop wants anyway.
STOP_MARGIN_S = 0.5


def udp_mac(key, payload):
    return hmac.new(key, payload, hashlib.sha256).digest()[:UDP_MAC_LEN]


def decode_command(datagram, key, robot_now):
    """(kind, ts, vx, vy, vyaw) from an authenticated datagram. Raises ValueError(reason).

    The MAC is checked FIRST and in constant time, before a single field is trusted: an
    unauthenticated datagram tells the sender nothing, not even which check it failed.
    """
    if len(datagram) != UDP_CMD.size + UDP_MAC_LEN:
        raise ValueError("size")
    body, mac = datagram[:UDP_CMD.size], datagram[UDP_CMD.size:]
    if not hmac.compare_digest(mac, udp_mac(key, body)):
        raise ValueError("mac")
    magic, version, kind, ts, vx, vy, vyaw = UDP_CMD.unpack(body)
    if magic != b"RC" or version != UDP_VERSION or kind not in (UDP_KIND_MOVE, UDP_KIND_STOP):
        raise ValueError("format")
    # NaN survives float() and every clamp (NaN > limit is False), and would reach Move().
    if not all(math.isfinite(x) for x in (ts, vx, vy, vyaw)):
        raise ValueError("format")
    if abs(robot_now - ts) > UDP_CLOCK_WINDOW_S:
        raise ValueError("clock")
    return kind, ts, vx, vy, vyaw


def encode_ack(key, status, ts):
    body = UDP_ACK.pack(b"RA", UDP_VERSION, status, ts)
    return body + udp_mac(key, body)


class CommandOrder:
    """The newest-wins rule for movement, shared by the UDP and HTTP paths.

    A move is admitted only if its `ts` is strictly newer than everything admitted so far.
    A stop is NEVER refused — it only raises the bar, so nothing sent before it can move the
    robot afterwards.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._last = 0.0

    def admit_move(self, ts):
        with self._lock:
            if ts <= self._last:
                return False
            self._last = ts
            return True

    def stop(self, ts, robot_now):
        with self._lock:
            self._last = max(self._last, ts if ts is not None else robot_now + STOP_MARGIN_S)


class UdpControl:
    """Datagram in, at most one command to command_sender, at most one ack out."""

    def __init__(self, key, sender, limiter, order, joint_control=None, walk=None):
        self.key = key
        self.walk = walk
        self.sender = sender
        self.limiter = limiter
        self.order = order
        self.joint_control = joint_control
        self.stats = {"accepted": 0, "size": 0, "mac": 0, "format": 0, "clock": 0,
                      "stale": 0, "limited": 0, "arms_held": 0}

    def process(self, datagram, src, robot_now):
        """Return the ack to send back, or None to stay silent."""
        try:
            kind, ts, vx, vy, vyaw = decode_command(datagram, self.key, robot_now)
        except ValueError as exc:
            self.stats[str(exc)] += 1
            return None
        if self.walk is not None:            # the drive pad wins over a walk to a point
            self.walk.cancel(f"udp {'stop' if kind == UDP_KIND_STOP else 'move'} from {src}",
                             stop=kind == UDP_KIND_STOP)
        if kind == UDP_KIND_STOP:
            # Not rate limited: repeating a stop is harmless, refusing one is not.
            self.order.stop(ts, robot_now)
            if self.joint_control is not None:
                self.joint_control.release_all("udp stop")
            line, verb, detail = "stop_move", "stop_move", "-"
        else:
            if self.joint_control is not None and self.joint_control.busy():
                self.stats["arms_held"] += 1     # no walking while the page holds an arm
                return None
            if not self.order.admit_move(ts):
                self.stats["stale"] += 1
                return None
            if not self.limiter.allow():
                self.stats["limited"] += 1
                return None
            line = f"move {vx:.3f} {vy:.3f} {vyaw:.3f}"
            verb, detail = "move", f"vx={vx:.3f} vy={vy:.3f} vyaw={vyaw:.3f}"
        reply = self.sender.send(line)
        audit(f"{src}/udp", verb, detail, reply)
        self.stats["accepted"] += 1
        return encode_ack(self.key, 0 if reply.startswith("ok") else 1, ts)

    def serve(self, sock):
        while True:
            try:
                datagram, src = sock.recvfrom(128)
            except OSError as exc:
                log(f"udp control: recv failed: {exc}")
                time.sleep(0.1)
                continue
            ack = self.process(datagram, src[0], time.time())
            if ack is not None:
                try:
                    sock.sendto(ack, src)
                except OSError:
                    pass                    # the ack is diagnostics; the command already ran


class Sender:
    """Owns a long-lived sender child. One DDS participant, created once.

    The command sender starts with the relay; the audio sender (`lazy`) starts the first time
    something is said, so a relay that never speaks costs nothing extra."""

    def __init__(self, argv, label="command_sender", timeout=None, lazy=False):
        self.argv = argv
        self.label = label
        self.timeout = timeout
        self.lock = threading.Lock()
        self.proc = None
        if not lazy:
            self._spawn()

    def _spawn(self):
        self.proc = subprocess.Popen(
            self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        log(f"{self.label} started (pid {self.proc.pid})")

    def send(self, line):
        """Write one command, read its reply. Serialised: the protocol is one-in-one-out."""
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                if self.proc is not None:
                    log(f"{self.label} is gone — respawning")
                try:
                    self._spawn()
                except OSError as exc:
                    return f"err sender-start {exc}"
            try:
                self.proc.stdin.write(line + "\n")
                self.proc.stdin.flush()
            except (BrokenPipeError, ValueError) as exc:
                return f"err sender-write {exc}"

            deadline = time.time() + (self.timeout or REPLY_TIMEOUT)
            while time.time() < deadline:
                reply = self.proc.stdout.readline()
                if not reply:
                    return "err sender-eof"
                reply = reply.strip()
                # Asynchronous events (e.g. the dead-man stop) must not be mistaken for
                # this command's reply.
                if reply.startswith("ev "):
                    log(f"event from sender: {reply}")
                    audit("-", "event", reply, "-")
                    continue
                return reply
            return "err sender-timeout"


class RateLimiter:
    def __init__(self, per_sec):
        self.per_sec = per_sec
        self.lock = threading.Lock()
        self.window = 0.0
        self.count = 0

    def allow(self):
        with self.lock:
            now = time.time()
            if now - self.window >= 1.0:
                self.window, self.count = now, 0
            if self.count >= self.per_sec:
                return False
            self.count += 1
            return True


class JointStream:
    """One g1_joint_bridge child shared by every viewer of the Joints page. READ-ONLY.

    The child prints one JSON line per tick (src/g1_joint_bridge.cpp). A reader thread keeps
    ONLY THE NEWEST line — deliberately not a queue: a slow viewer skips lines and never
    delays the bridge or another viewer (engineering standard §3, "drop, never queue").

    The child runs only while someone watches: the first viewer starts it, and it is stopped
    JOINT_IDLE_S after the last one leaves. If it dies, the next viewer that subscribes starts
    a new one. Its stdin is ours, so closing it is how it is told to exit.
    """

    def __init__(self, argv, max_clients=JOINT_MAX_CLIENTS, idle_s=JOINT_IDLE_S,
                 label="joint bridge", side_prefix=None):
        self.argv = argv
        self.label = label
        # Lines starting with `side_prefix` are a second stream from the same child (the lidar
        # bridge's 3D cloud): kept apart, newest wins too, so neither crowds out the other.
        self.side_prefix = side_prefix
        self.side_latest = None
        self.side_seq = 0
        self.max_clients = max_clients
        self.idle_s = idle_s
        self.cond = threading.Condition()
        self.proc = None
        self.latest = None
        self.seq = 0
        self.clients = 0
        self.internal = 0                   # of them, the relay's own readers (uncapped)
        self._idle_timer = None

    def _ensure_running(self):
        """Start the child if it is not running. Caller holds self.cond."""
        if self.proc is not None and self.proc.poll() is None:
            return
        self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.latest = None
        log(f"{self.label} started (pid {self.proc.pid})")
        threading.Thread(target=self._read, args=(self.proc,), name=self.label.replace(" ", "-"),
                         daemon=True).start()

    def _read(self, proc):
        for line in proc.stdout:
            line = line.strip()
            # Only whole JSON objects go out: a torn or garbled line must not reach a viewer.
            if not (line.startswith("{") and line.endswith("}")):
                continue
            with self.cond:
                if self.side_prefix and line.startswith(self.side_prefix):
                    self.side_latest = line
                    self.side_seq += 1
                else:
                    self.latest = line
                    self.seq += 1
                self.cond.notify_all()
        with self.cond:
            if self.proc is proc:
                self.proc = None
                self.latest = None
            self.cond.notify_all()
        log(f"{self.label} exited (code {proc.wait()})")

    def subscribe(self, internal=False):
        """Register a viewer. False when the viewer cap is reached. `internal` = the relay's own
        reader (a walk, its plan, the trail): never refused and not counted against the cap —
        on 2026-11-11 two open Navigation pages took all 4 places and a plan failed with "no
        obstacle map from the lidar" while the lidar was fine."""
        with self.cond:
            if not internal and self.clients - self.internal >= self.max_clients:
                return False
            self.clients += 1
            self.internal += 1 if internal else 0
            if self._idle_timer is not None:
                self._idle_timer.cancel()
                self._idle_timer = None
            self._ensure_running()
            return True

    def unsubscribe(self, internal=False):
        with self.cond:
            self.clients = max(0, self.clients - 1)
            if internal:
                self.internal = max(0, self.internal - 1)
            if self.clients == 0 and self._idle_timer is None:
                self._idle_timer = threading.Timer(self.idle_s, self._stop_if_idle)
                self._idle_timer.daemon = True
                self._idle_timer.start()

    def _stop_if_idle(self):
        with self.cond:
            self._idle_timer = None
            if self.clients:
                return
            proc, self.proc, self.latest = self.proc, None, None
        if proc is not None:
            self._terminate(proc)

    def next_line(self, seen, timeout):
        """(seq, line) once a line newer than `seen` exists, or (seen, None) after `timeout`."""
        with self.cond:
            self.cond.wait_for(lambda: self.seq != seen and self.latest is not None, timeout)
            if self.seq != seen and self.latest is not None:
                return self.seq, self.latest
            return seen, None

    def next_side(self, seen, timeout):
        """The same for the side stream (lines starting with `side_prefix`)."""
        with self.cond:
            self.cond.wait_for(lambda: self.side_seq != seen and self.side_latest is not None,
                               timeout)
            if self.side_seq != seen and self.side_latest is not None:
                return self.side_seq, self.side_latest
            return seen, None

    @staticmethod
    def _terminate(proc):
        try:
            proc.stdin.close()          # the bridge exits on EOF
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

    def stop(self):
        with self.cond:
            proc, self.proc = self.proc, None
            if self._idle_timer is not None:
                self._idle_timer.cancel()
        if proc is not None:
            self._terminate(proc)

    def status(self):
        with self.cond:
            return {"running": self.proc is not None and self.proc.poll() is None,
                    "clients": self.clients - self.internal, "internal": self.internal,
                    "max_clients": self.max_clients}


def validate_trajectory(points, allowed, start=None, max_vel=None):
    """A planner's trajectory as [(t, {joint: rad}, {side: [6 fingers]})], or ValueError.

    `points`: [{"t": s, "q": {joint: rad}, "hands": {"left"|"right": [6 x 0-1000]}}], t > 0 and
    strictly increasing, at most TRAJ_MAX_POINTS and TRAJ_MAX_S. The FIRST point names every
    joint the trajectory moves; a later point may leave one out (it holds). Between points, and
    from `start` (the measured pose, when known) to the first one, no joint may need more than
    `max_vel` rad/s: the sender would never keep up, and the motion would lag its own clock."""
    max_vel = JOINT_CTL_MAX_VEL if max_vel is None else max_vel
    if not isinstance(points, list) or not points:
        raise ValueError("points must be a non-empty list")
    if len(points) > TRAJ_MAX_POINTS:
        raise ValueError(f"at most {TRAJ_MAX_POINTS} points")
    out, prev_t, prev_q = [], 0.0, dict(start or {})
    for n, p in enumerate(points):
        if not isinstance(p, dict):
            raise ValueError(f"point {n} must be an object")
        t, q, hands = p.get("t"), p.get("q") or {}, p.get("hands") or {}
        if isinstance(t, bool) or not isinstance(t, (int, float)) or not math.isfinite(t):
            raise ValueError(f"point {n}: t must be a number of seconds")
        if t <= prev_t:
            raise ValueError(f"point {n}: t must be after the previous point (and > 0)")
        if t > TRAJ_MAX_S:
            raise ValueError(f"point {n}: at most {TRAJ_MAX_S:g} s in all")
        if not isinstance(q, dict) or not isinstance(hands, dict):
            raise ValueError(f"point {n}: q and hands must be objects")
        if n == 0 and not q:
            raise ValueError("the first point must name the joints the trajectory moves")
        if n > 0 and set(q) - set(out[0][1]):
            raise ValueError(f"point {n}: a joint not in the first point")
        for name, v in q.items():
            if name not in allowed:
                raise ValueError(f"point {n}: joint not allowed: {name}")
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                raise ValueError(f"point {n}: {name} must be a number of radians")
            if name in prev_q and abs(v - prev_q[name]) / (t - prev_t) > max_vel + 1e-9:
                raise ValueError(f"point {n}: {name} would move faster than {max_vel:g} rad/s")
        for side, fingers in hands.items():
            if side not in ("left", "right") or not isinstance(fingers, list) or len(fingers) != 6 \
                    or not all(isinstance(f, (int, float)) and not isinstance(f, bool)
                               and math.isfinite(f) for f in fingers):
                raise ValueError(f"point {n}: hands are {{left|right: [6 numbers 0-1000]}}")
        prev_q.update({k: float(v) for k, v in q.items()})
        out.append((float(t), {k: float(v) for k, v in q.items()},
                    {side: [round(min(1000.0, max(0.0, f))) for f in fingers]
                     for side, fingers in hands.items()}))
        prev_t = t
    return out


def validate_safe_pose(pose, allowed):
    """{joint: rad} for the session's safe pose, or ValueError. None stays None."""
    if pose is None:
        return None
    if not isinstance(pose, dict) or not pose:
        raise ValueError("'safe' must be a non-empty {joint: rad} object")
    out = {}
    for name, v in pose.items():
        if name not in allowed:
            raise ValueError(f"safe pose: joint not allowed: {name}")
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) \
                or abs(v) > math.pi:
            raise ValueError(f"safe pose: {name} must be radians in [-pi, pi]")
        out[name] = float(v)
    return out


class Trajectory:
    """Runs ONE validated trajectory inside a control session, on its own thread.

    Every 1/TRAJ_HZ s it sends the sender one `targets` line: the pose interpolated for that
    instant (from the measured pose to the first point, then point to point), and each point's
    finger targets once its time comes. The planner contract (G1.md §6.9): progress while it
    runs; a result — success (every joint within TRAJ_TOL of the last point, measured), uncertain
    (not there TRAJ_SETTLE_S after the end, or no measured pose to tell), failure (a stall, a hot
    motor, the release, the sender refusing) — and cancel, which freezes the arm where it is."""

    def __init__(self, ctl, points, start=None, measured=None):
        self.ctl = ctl
        self.points = points
        self.start = dict(start or {})
        self.measured = measured or (lambda: None)
        self.hz, self.tol, self.settle_s = TRAJ_HZ, TRAJ_TOL, TRAJ_SETTLE_S
        self.joints = list(points[0][1])
        self.duration = points[-1][0]
        self.state, self.reason, self.error = "running", None, None
        self.t = 0.0
        self.cancelled = threading.Event()
        self._last = {}
        self._hands_done = 0
        self.thread = threading.Thread(target=self._run, name="trajectory", daemon=True)

    def pose_at(self, t):
        """The pose for instant `t`, linear between points; before the first point, from the
        measured start (a joint with no start goes straight to the first point's value)."""
        prev_t, prev = 0.0, {j: self.start.get(j, self.points[0][1][j]) for j in self.joints}
        for pt, q, _hands in self.points:
            nxt = {**prev, **q}
            if t <= pt:
                f = (t - prev_t) / (pt - prev_t)
                return {j: prev[j] + (nxt[j] - prev[j]) * f for j in self.joints}
            prev_t, prev = pt, nxt
        return prev

    def snapshot(self):
        return {"state": self.state, "reason": self.reason, "t": round(self.t, 2),
                "duration": self.duration, "points": len(self.points),
                "error_rad": None if self.error is None else round(self.error, 3)}

    def _finish(self, state, reason=None):
        self.state, self.reason = state, reason

    def _send(self, line):
        with self.ctl.lock:
            if not self.ctl.busy() or self.ctl.traj is not self:
                return "err no control session"
            return self.ctl._send(line)

    def _arrival_error(self):
        q = self.measured()
        if q is None:
            return None
        final = self.pose_at(self.duration)
        return max(abs(q[j] - final[j]) for j in self.joints if j in q) if self.joints else 0.0

    def _run(self):
        # The joint stream runs only while someone watches: the trajectory watches too, so its
        # arrival is checked even with the Joints page closed.
        stream = self.ctl.stream
        watching = stream is not None and stream.subscribe()
        try:
            self._loop()
        finally:
            if watching:
                stream.unsubscribe()

    def _loop(self):
        t0, seq0 = time.monotonic(), self.ctl.event_seq
        period = 1.0 / self.hz
        while True:
            self.t = time.monotonic() - t0
            if self.cancelled.is_set():
                # Freeze: the goal becomes where the arm IS (or the last pose sent), so the
                # sender stops it with its own deceleration instead of finishing the motion.
                here = self.measured() or self._last
                if here:
                    self._send("targets " + " ".join(f"{j} {here[j]:.4f}"
                                                     for j in self.joints if j in here))
                return self._finish("cancelled", "cancel asked")
            if self.ctl.event_seq != seq0:
                ev = self.ctl.last_event or ""
                if ev.startswith(("stall", "released", "hot", "deadman")):
                    return self._finish("failure", ev)
                seq0 = self.ctl.event_seq
            pose = self.pose_at(min(self.t, self.duration))
            reply = self._send("targets " + " ".join(f"{j} {v:.4f}" for j, v in pose.items()))
            if not reply.startswith("ok"):
                return self._finish("failure", reply)
            self._last = pose
            while self._hands_done < len(self.points) and \
                    self.points[self._hands_done][0] <= self.t:
                for side, fingers in self.points[self._hands_done][2].items():
                    for f, pos in enumerate(fingers):
                        reply = self._send(f"hand {side} {f} {pos}")
                        if not reply.startswith("ok"):
                            return self._finish("failure", reply)
                self._hands_done += 1
            if self.t >= self.duration:
                self.error = self._arrival_error()
                if self.error is not None and self.error <= self.tol:
                    return self._finish("success")
                if self.t >= self.duration + self.settle_s:
                    return self._finish("uncertain", "no measured pose to check" if self.error
                                        is None else f"still {self.error:.3f} rad from the end")
            time.sleep(max(0.0, period - (time.monotonic() - t0 - self.t)))


class JointControl:
    """One g1_joint_sender child per control SESSION: moving single arm joints from the page.

    The rules (allowlist, limits, speed, weight ramps, dead-man, FSM gate) are in the sender —
    src/joint_ctl_core.hpp. This class only owns the session:

      * ONE CONTROLLER AT A TIME. `take` returns a random session id; every later op must carry
        it, so a second browser cannot steer an arm someone else is holding.
      * ONE PROCESS PER SESSION. `take` spawns the sender; after the release ramp it exits by
        itself, and the session is over. Nothing carries over into the next one.
      * WHILE A SESSION IS OPEN THE ROBOT DOES NOT WALK: `take` stops any movement first, and
        the /cmd and UDP paths refuse `move` and the arm actions until it ends (busy()). A
        stop_move from anywhere releases the arm (release_all).
    """

    def __init__(self, argv, cmd_sender=None, order=None, reply_timeout=None, stream=None):
        self.argv = argv
        self.stream = stream                # JointStream: the measured pose, for trajectories
        self.traj = None                    # the Trajectory running in this session, if any
        self.cmd_sender = cmd_sender
        self.order = order
        self.reply_timeout = reply_timeout or REPLY_TIMEOUT
        self.lock = threading.Lock()        # one op at a time: the protocol is one-in-one-out
        self.proc = None
        self.session = None
        self.replies = queue.Queue()
        self.state = {"state": "off", "w": 0.0}
        self.last_event = None
        # Counts the sender's events in this session. Every reply repeats the LAST event, so two
        # stalls of the same joint in a row would read the same; the count tells the page there
        # was a second one.
        self.event_seq = 0
        # Rule 12 of the sender: each hand's last grasp, for the planner's contract (G1.md §6.9):
        # {"state": closing | held | empty | uncertain, "forces": [5 x N or None]}.
        self.grasp = {"left": None, "right": None}
        # The session's safe pose and its retreat (see JOINT_CTL_RETREAT_C).
        self.safe = None
        self.retreat = None                 # {"state": running | done, "reason", "result"}
        self.last_client = 0.0
        self.take_pose = None               # where the arm was at `take`: PC1's own pose

    def busy(self):
        """True while a session holds (or is handing back) the arms."""
        proc = self.proc
        return proc is not None and proc.poll() is None

    def _read(self, proc):
        for line in proc.stdout:
            line = line.strip()
            if line.startswith("ev "):
                self.last_event = line[3:]
                self.event_seq += 1
                parts = line.split()
                if len(parts) >= 4 and parts[1] == "grasp" and parts[2] in self.grasp:
                    self.grasp[parts[2]] = {
                        "state": parts[3],
                        "forces": [None if f == "-" else float(f) for f in parts[4:9]]}
                log(f"joint control: {line}")
                audit("-", "joints-control", "event", line[3:])
                if line.startswith("ev released"):
                    # The sender exits by itself right after this. If it does not, the session
                    # would look open forever and every later `take` be refused: end it.
                    threading.Timer(3.0, self._reap, args=(proc,)).start()
                continue
            self.replies.put(line)
        code = proc.wait()
        with self.lock:
            if self.proc is proc:
                self.proc = None
                self.session = None
                self.state = {"state": "off", "w": 0.0}
        log(f"joint sender exited (code {code})")

    @staticmethod
    def _reap(proc):
        if proc.poll() is None:
            log("joint sender still running 3 s after its release: killing it")
            proc.kill()

    def _send(self, line, timeout=None):
        """One line in, its reply out. Caller holds self.lock."""
        try:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError, OSError) as exc:
            return f"err sender-write {exc}"
        try:
            reply = self.replies.get(timeout=timeout or self.reply_timeout)
        except queue.Empty:
            return "err sender-timeout"
        m = _JOINT_REPLY.match(reply)
        if m:
            self.state = {"state": m.group(3), "w": float(m.group(4))}
        return reply

    def _answer(self, reply, code_err=409):
        ok = reply.startswith("ok")
        m = _JOINT_REPLY.match(reply)
        out = {"ok": ok, "reply": m.group(2).strip() if m else reply, **self.state,
               "event": self.last_event, "event_seq": self.event_seq,
               "grasp": dict(self.grasp)}
        return (200 if ok else code_err), out

    def take(self, safe=None):
        try:
            safe = validate_safe_pose(safe, set(JOINT_CTL_ALLOW))
        except ValueError as exc:
            return 400, {"ok": False, "error": str(exc)}
        with self.lock:
            if self.busy():
                return 409, {"ok": False, "error": "the arms are already being controlled",
                             **self.state}
            # Nothing may be walking or playing an arm action when the arms are taken; and
            # nothing already in flight may start a walk after this point.
            if self.order is not None:
                self.order.stop(None, time.time())
            if self.cmd_sender is not None:
                self.cmd_sender.send("stop_move")
            while not self.replies.empty():
                self.replies.get_nowait()
            try:
                self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE,
                                             stdout=subprocess.PIPE, text=True, bufsize=1)
            except OSError as exc:
                self.proc = None
                return 503, {"ok": False, "error": f"joint sender cannot start: {exc}"}
            log(f"joint sender started (pid {self.proc.pid})")
            self.last_event = None
            self.event_seq = 0
            self.grasp = {"left": None, "right": None}
            threading.Thread(target=self._read, args=(self.proc,), name="joint-sender",
                             daemon=True).start()
            # Longer than an op: the sender first joins DDS and reads the robot once (~1.5 s).
            reply = self._send("take", self.reply_timeout + 3)
            if not reply.startswith("ok"):
                # Refused (robot not standing, no state): end the process, nothing was published.
                self._close_locked()
                return self._answer(reply)
            self.session = secrets.token_hex(8)
            self.safe, self.retreat = safe, None
            self.last_client = time.monotonic()
            self.take_pose = self.measured()
            if safe is not None:
                threading.Thread(target=self._guard, args=(self.proc, self.session),
                                 name="joint-guard", daemon=True).start()
            code, out = self._answer(reply)
            out.update(session=self.session, allowed=JOINT_CTL_ALLOW,
                       excursion=JOINT_CTL_EXCURSION, margin=JOINT_CTL_MARGIN,
                       safe=safe is not None)
            return code, out

    def measured_temp(self):
        """{joint: °C} of the arm_sdk joints from the joint stream, or None without one."""
        stream = self.stream
        line = stream.latest if stream is not None else None
        try:
            t = json.loads(line).get("temp") if line else None
            return {name: float(t[i]) for name, i in BODY_INDEX.items()} if t else None
        except (ValueError, KeyError, IndexError, TypeError, AttributeError):
            return None

    def _guard(self, proc, session):
        """While this session lasts: retreat to the safe pose on a warm joint or a lost client.
        Watches the joint stream (temperatures, pose) for as long as it runs."""
        stream = self.stream
        watching = stream is not None and stream.subscribe()
        try:
            while self.proc is proc and self.session == session and proc.poll() is None:
                time.sleep(0.2)
                if self.retreat is not None:
                    continue
                temps = self.measured_temp() or {}
                warm = [(j, t) for j, t in temps.items()
                        if j in self.safe and t >= JOINT_CTL_RETREAT_C]
                if warm:
                    j, t = max(warm, key=lambda x: x[1])
                    self._start_retreat(session, f"warm {j} {t:.0f}C")
                elif time.monotonic() - self.last_client > JOINT_CTL_CLIENT_LOST_S:
                    self._start_retreat(session, "client lost")
        finally:
            if watching:
                stream.unsubscribe()

    def _near(self, pose):
        q = self.measured()
        return q is not None and pose is not None and all(
            abs(q[j] - v) <= RETREAT_AT_RAD for j, v in pose.items() if j in q)

    def _near_safe(self):
        return self._near(self.safe)

    def _at_rest(self):
        """Back where the arm was when it was taken — the pose PC1 takes it to on release, so
        letting go there moves nothing. A retreat from there would raise the arm only for PC1
        to lower it again (2026-11-03: the planner brought the arm home, released, and the
        relay lifted it to the safe pose first)."""
        return self._near({j: self.take_pose[j] for j in self.safe if j in (self.take_pose or {})})

    def _start_retreat(self, session, reason):
        """Cancel what runs, move to the safe pose, then release. Never called with self.lock
        held: the running trajectory needs it to stop. False if it could not start (no safe
        pose, no measured pose): the sender's own limits then decide, as before."""
        if self.safe is None or self.retreat is not None or self._at_rest():
            return False
        traj = self.traj
        if traj is not None and traj.state == "running":
            traj.cancelled.set()
            traj.thread.join(timeout=2.0)
        start = self.measured()
        if start is None:
            return False
        far = max(abs(start.get(j, v) - v) for j, v in self.safe.items())
        t = max(1.5, far / (JOINT_CTL_MAX_VEL * RETREAT_VEL_FRACTION))
        try:
            plan = validate_trajectory([{"t": round(t, 2), "q": self.safe}],
                                       set(JOINT_CTL_ALLOW), start)
        except ValueError as exc:
            log(f"joint control: no retreat ({exc})")
            return False
        with self.lock:
            if self.session != session or not self.busy():
                return False
            self.retreat = {"state": "running", "reason": reason, "result": None}
            self.traj = Trajectory(self, plan, start=start, measured=self.measured)
            retreat_traj = self.traj
        log(f"joint control: retreating to the safe pose ({reason}), {t:.1f} s")
        audit("-", "joints-control", "retreat", reason)
        retreat_traj.thread.start()

        def finish():
            retreat_traj.thread.join()
            with self.lock:
                if self.session == session and self.busy():
                    reply = self._send("release")
                    log(f"joint control: released after the retreat: {reply}")
            self.retreat = {"state": "done", "reason": reason,
                            "result": retreat_traj.snapshot()}
        threading.Thread(target=finish, name="joint-retreat", daemon=True).start()
        return True

    def measured(self):
        """The arm_sdk joints' measured angles from the joint stream, or None without one."""
        stream = self.stream
        line = stream.latest if stream is not None else None
        try:
            q = json.loads(line)["q"] if line else None
            return {name: float(q[i]) for name, i in BODY_INDEX.items()} if q else None
        except (ValueError, KeyError, IndexError, TypeError):
            return None

    def _traj_running(self):
        return self.traj is not None and self.traj.state == "running"

    def op(self, session, op, joint=None, q=None, side=None, finger=None, pos=None,
           points=None, thumb=None, thumb_flex=None):
        """keepalive | status | target | hand | grasp | open | trajectory | cancel | release,
        in a session. `grasp` and `open` take `side`; `grasp` an optional force in `pos` (N)."""
        if isinstance(session, str) and self.session is not None and \
                hmac.compare_digest(session, self.session):
            self.last_client = time.monotonic()
            if self.retreat is not None and op not in ("keepalive", "status", "release"):
                return 409, {"ok": False, **self.state, "retreat": self.retreat,
                             "error": f"retreating to the safe pose ({self.retreat['reason']})"}
            if self.retreat is not None:          # keepalive/status/release: report, nothing more
                return 200, {"ok": True, **self.state, "retreat": self.retreat,
                             "trajectory": self.traj.snapshot() if self.traj else None}
            if op == "release" and self.safe is not None and not self._near_safe() and \
                    not self._at_rest() and self._start_retreat(session, "release asked"):
                return 200, {"ok": True, **self.state, "retreat": self.retreat}
        if op == "trajectory":
            return self._start_trajectory(session, points)
        with self.lock:
            if not self.busy() or self.session is None:
                return 409, {"ok": False, "error": "no control session", **self.state,
                             "event": self.last_event, "event_seq": self.event_seq,
                             "retreat": self.retreat}
            if not isinstance(session, str) or not hmac.compare_digest(session, self.session):
                return 409, {"ok": False, "error": "another page holds the arms"}
            if op == "cancel":
                if self._traj_running():
                    self.traj.cancelled.set()
                return 200, {"ok": True, **self.state, "trajectory":
                             self.traj.snapshot() if self.traj else None}
            if op in ("target", "hand") and self._traj_running():
                return 409, {"ok": False, "error": "a trajectory is running: cancel it first"}
            if op == "release" and self._traj_running():
                self.traj.cancelled.set()
            if op == "target":
                if not isinstance(joint, str) or not _JOINT_NAME.match(joint):
                    return 400, {"ok": False, "error": "joint must be a joint name"}
                try:
                    q = float(q)
                except (TypeError, ValueError):
                    return 400, {"ok": False, "error": "q must be a number"}
                if not math.isfinite(q):
                    return 400, {"ok": False, "error": "q must be finite"}
                line = f"target {joint} {q:.4f}"
            elif op == "hand":
                # One finger of a BrainCo hand, 0 open .. 1000 closed (the sender clamps again).
                if side not in ("left", "right"):
                    return 400, {"ok": False, "error": "side must be left or right"}
                if isinstance(finger, bool) or not isinstance(finger, int) or not 0 <= finger <= 5:
                    return 400, {"ok": False, "error": "finger must be 0-5"}
                try:
                    pos = float(pos)
                except (TypeError, ValueError):
                    return 400, {"ok": False, "error": "pos must be a number"}
                if not math.isfinite(pos):
                    return 400, {"ok": False, "error": "pos must be finite"}
                line = f"hand {side} {finger} {round(min(1000.0, max(0.0, pos)))}"
            elif op in ("grasp", "open"):
                # Rule 12 of the sender: close until the fingertips confirm, or open.
                if side not in ("left", "right"):
                    return 400, {"ok": False, "error": "side must be left or right"}
                line = f"{op} {side}"
                if op == "grasp" and pos is not None:
                    try:
                        force = float(pos)
                    except (TypeError, ValueError):
                        return 400, {"ok": False, "error": "force must be a number of newtons"}
                    if not (math.isfinite(force) and 0.2 <= force <= 10):
                        return 400, {"ok": False, "error": "force must be 0.2-10 N"}
                    line += f" {force:.2f}"
                if op == "grasp" and thumb is not None:
                    # Turn the thumb opposite the fingers first (a power grip); the core needs a
                    # force before the thumb, so the default one goes in when none was given.
                    if isinstance(thumb, bool) or not isinstance(thumb, (int, float)) \
                            or not 0 <= thumb <= 1000:
                        return 400, {"ok": False, "error": "thumb must be 0-1000"}
                    if pos is None:
                        line += " 1.00"
                    line += f" {round(thumb)}"
                    if thumb_flex is not None:
                        if isinstance(thumb_flex, bool) or not isinstance(thumb_flex, (int, float)) \
                                or not 0 <= thumb_flex <= 1000:
                            return 400, {"ok": False, "error": "thumb_flex must be 0-1000"}
                        line += f" {round(thumb_flex)}"
                if op == "grasp":
                    self.grasp[side] = {"state": "closing", "forces": None}
            elif op in ("keepalive", "status", "release", "keep_hands"):
                line = op
            else:
                return 400, {"ok": False, "error": f"unknown op {op!r}"}
            code, out = self._answer(self._send(line))
            if self.traj is not None:
                out["trajectory"] = self.traj.snapshot()
            return code, out

    def _start_trajectory(self, session, points):
        """Check a trajectory against the session, the allowlist and the speed limit, then run
        it. Answers at once; progress and the result come with every later op (status)."""
        with self.lock:
            if not self.busy() or self.session is None:
                return 409, {"ok": False, "error": "no control session", **self.state}
            if not isinstance(session, str) or not hmac.compare_digest(session, self.session):
                return 409, {"ok": False, "error": "another page holds the arms"}
            if self._traj_running():
                return 409, {"ok": False, "error": "a trajectory is already running"}
            start = self.measured()
            try:
                plan = validate_trajectory(points, set(JOINT_CTL_ALLOW), start)
            except ValueError as exc:
                return 400, {"ok": False, "error": str(exc)}
            self.traj = Trajectory(self, plan, start=start, measured=self.measured)
            self.traj.thread.start()
            return 200, {"ok": True, **self.state, "trajectory": self.traj.snapshot()}

    def release_all(self, why):
        """Hand the arms back whoever holds them (a stop pressed anywhere, shutdown)."""
        with self.lock:
            if not self.busy():
                return False
            reply = self._send("release")
        log(f"joint control released ({why}): {reply}")
        return True

    def _close_locked(self):
        proc, self.proc, self.session = self.proc, None, None
        if proc is None:
            return
        try:
            proc.stdin.close()          # EOF: the sender releases (ramps down) and exits
            proc.wait(timeout=5)
        except Exception:
            proc.kill()

    def stop(self):
        """Relay shutdown: EOF makes the sender ramp the weight down before it exits."""
        with self.lock:
            self._close_locked()

    def status(self):
        return {"active": self.busy(), **self.state, "event": self.last_event,
                "event_seq": self.event_seq, "allowed": JOINT_CTL_ALLOW,
                "safe": self.safe is not None, "retreat": self.retreat}


def wrap_angle(a):
    """An angle in (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def validate_goto(body):
    """(dx, dy, dyaw) from a goto request, or raise ValueError with the reason."""
    try:
        dx, dy, dyaw = (float(body.get(k, 0) or 0) for k in ("dx", "dy", "dyaw"))
    except (TypeError, ValueError):
        raise ValueError("dx/dy/dyaw must be numbers") from None
    if not all(math.isfinite(v) for v in (dx, dy, dyaw)):
        raise ValueError("dx/dy/dyaw must be finite")
    if math.hypot(dx, dy) > WALK_MAX_M:
        raise ValueError(f"at most {WALK_MAX_M:g} m per request")
    if abs(dyaw) > WALK_MAX_TURN + 1e-9:
        raise ValueError("at most half a turn (pi rad) per request")
    if math.hypot(dx, dy) < 0.01 and abs(dyaw) < math.radians(1):
        raise ValueError("nothing to do: dx, dy and dyaw are all zero")
    return dx, dy, dyaw


def walk_command(pose, goal, phase, aim=None, face=None, lim=None, min_v=WALK_MIN_V):
    """(vx, vy, vyaw, phase) toward `goal` from `pose`, both {x, y, yaw} in the odom frame.

    With a planned path, `aim` (x, y) is the point on it to head for now — the speed still comes
    from how far the GOAL is — and `face` the heading to keep meanwhile (the way it walks round
    an obstacle); without them, straight at the goal holding the start heading. `lim` is the
    walk's (vx, vy, vyaw) top speeds; the defaults when absent.

    Two phases, deterministic and easy to watch: TRANSLATE to the goal's position holding the
    heading the walk started with (so "2 m forward" is a straight line, not an arc), then TURN in
    place to the goal's heading. While turning the position is not chased: the torso sways ~10 cm
    when a humanoid turns in place (seen 2026-11-05), and chasing that would make it shuffle.
    `phase` "done" = arrived."""
    yaw = pose["yaw"]
    lvx, lvy, lvyaw = lim or (WALK_VX, WALK_VY, WALK_VYAW)
    if phase == "translate":
        ex_w, ey_w = goal["x"] - pose["x"], goal["y"] - pose["y"]
        left = math.hypot(ex_w, ey_w)
        if left > WALK_POS_TOL:
            if aim is not None:
                ax, ay = aim[0] - pose["x"], aim[1] - pose["y"]
                d = math.hypot(ax, ay)
                if d > 1e-6:
                    ex_w, ey_w = ax / d * left, ay / d * left
            # The error in the robot's own frame: forward is x, left is y.
            ex = math.cos(yaw) * ex_w + math.sin(yaw) * ey_w
            ey = -math.sin(yaw) * ex_w + math.cos(yaw) * ey_w
            vx, vy = WALK_GAIN * ex, WALK_GAIN * ey
            # Both axes scaled together, so the robot heads AT the goal, not past it.
            scale = min(1.0, lvx / max(abs(vx), 1e-9), lvy / max(abs(vy), 1e-9))
            vx, vy = vx * scale, vy * scale
            speed = math.hypot(vx, vy)
            if speed < min_v:
                vx, vy = vx * min_v / speed, vy * min_v / speed
            hold = wrap_angle((goal["hold"] if face is None else face) - yaw)
            vyaw = max(-lvyaw, min(lvyaw, WALK_YAW_GAIN * hold))
            return vx, vy, vyaw, "translate"
        phase = "turn"
    if phase == "turn":
        err = wrap_angle(goal["yaw"] - yaw)
        if abs(err) > WALK_YAW_TOL:
            w = max(-lvyaw, min(lvyaw, WALK_YAW_GAIN * err))
            if abs(w) < WALK_MIN_W:
                w = math.copysign(WALK_MIN_W, err)
            return 0.0, 0.0, w, "turn"
    return 0.0, 0.0, 0.0, "done"


TRAIL_STEP_M = 0.03       # a new trail point once the robot moved this far
TRAIL_MAX = 6000          # ~180 m of walking at 3 cm; the oldest go first


class OdomTrail:
    """Where the robot has walked, recorded HERE, all the time — whoever moves it (the drive pad,
    the remote, a walk to a point, the agent) and whether or not a page is watching. The page
    asks for it on opening (GET /odom/trail) and draws the rest live; it used to keep the trail
    itself, and lost it on every tab change (2026-11-05).

    It holds the odometry bridge open for good (one 51 Hz DDS reader on PC2: nothing), and keeps a
    point every TRAIL_STEP_M. The odometry's origin is where the robot booted, so a reboot starts
    a new frame: a jump of more than TRAIL_JUMP_M between two readings starts the trail again."""

    TRAIL_JUMP_M = 1.0

    def __init__(self, odom, step=TRAIL_STEP_M, cap=TRAIL_MAX):
        self.odom = odom
        self.step, self.cap = step, cap
        self.lock = threading.Lock()
        self.points = []                  # [x, y, yaw, unix t]
        self.cleared_at = None
        self.thread = None

    def start(self):
        if not self.odom.subscribe(internal=True):
            log("trail: no room on the odometry stream")
            return self
        self.thread = threading.Thread(target=self._run, name="odom-trail", daemon=True)
        self.thread.start()
        return self

    def _run(self):
        seen = 0
        while True:
            seen, line = self.odom.next_line(seen, 5.0)
            if line is None:
                continue
            try:
                d = json.loads(line)
                p = [round(float(d["x"]), 3), round(float(d["y"]), 3), round(float(d["yaw"]), 3),
                     round(float(d["t"]), 1)]
            except (ValueError, KeyError, TypeError):
                continue
            if d.get("age_ms", -1) < 0:
                continue
            self.add(p)

    def add(self, p):
        with self.lock:
            last = self.points[-1] if self.points else None
            if last is not None:
                moved = math.hypot(p[0] - last[0], p[1] - last[1])
                if moved > self.TRAIL_JUMP_M:
                    self.points = []          # a new odometry frame (the robot rebooted)
                elif moved < self.step:
                    return
            self.points.append(p)
            if len(self.points) > self.cap:
                del self.points[:len(self.points) - self.cap]

    def snapshot(self):
        with self.lock:
            return {"points": list(self.points), "cleared_at": self.cleared_at}

    def clear(self):
        with self.lock:
            self.points = self.points[-1:]
            self.cleared_at = time.time()


def walk_options(body):
    """(lim, mode) from a goto request: `speed` {vx, vy, vyaw} top speeds (each within WALK_CAPS,
    defaults WALK_VX / WALK_VY / WALK_VYAW) or "auto" (lim is then None: Walk picks it), and
    `mode` "walk" | "run" | None (keep the robot's). ValueError with the reason."""
    mode = body.get("mode")
    if mode not in (None, "walk", "run"):
        raise ValueError("mode must be walk or run")
    if body.get("speed") == "auto":
        return None, mode
    sp = body.get("speed") or {}
    if not isinstance(sp, dict):
        raise ValueError("speed must be {vx, vy, vyaw}")
    lim = []
    for k, default, cap in zip(("vx", "vy", "vyaw"), (WALK_VX, WALK_VY, WALK_VYAW), WALK_CAPS):
        try:
            v = float(sp.get(k, default))
        except (TypeError, ValueError):
            raise ValueError(f"speed.{k} must be a number") from None
        if not (math.isfinite(v) and 0 < v <= cap + 1e-9):
            raise ValueError(f"speed.{k} must be over 0 and at most {cap:g}")
        lim.append(v)
    return tuple(lim), mode


def parse_lidar(line):
    """(cells as (x, y) metres in the odom frame, the line's dict) from a g1_lidar_bridge line,
    or (None, dict) when it cannot be used: not fresh, or not in the odometry frame."""
    try:
        d = json.loads(line)
    except ValueError:
        return None, {}
    if d.get("age_ms", -1) < 0 or d["age_ms"] > 500 or not d.get("odom"):
        return None, d
    res = float(d.get("res", 0.05))
    return [(ix * res, iy * res) for ix, iy in d.get("cells", ())], d


def obstacle_ahead(pose, vx, vy, cells, reach=WALK_STOP_M, half=WALK_HALF_WIDTH):
    """Metres to the nearest obstacle cell in the corridor the robot is about to walk into
    (`reach` long, `half` each side, along the direction of vx, vy from `pose`), or None."""
    if not cells or math.hypot(vx, vy) < 1e-6:
        return None
    h = pose["yaw"] + math.atan2(vy, vx)
    c, s = math.cos(h), math.sin(h)
    nearest = None
    for x, y in cells:
        px, py = x - pose["x"], y - pose["y"]
        along = px * c + py * s
        if 0 < along <= reach and abs(-px * s + py * c) <= half:
            nearest = along if nearest is None else min(nearest, along)
    return nearest


WALK_PLAN_RES = 0.10          # m: the planner's grid
# m kept between the robot's centre and anything the lidar saw. The body is 0.45 m across the
# shoulders (0.225 each side); 0.35 closed a 0.8 m passage in the office (2026-11-11, "no way" with
# room to spare).
WALK_CLEARANCE = float(os.environ.get("WALK_CLEARANCE", "0.30"))
# ...except at the END of the walk, where it may stand closer: at a table to pick something up
# the torso is ~0.3 m from the edge (2026-11-03). The body is ~0.45 m across the shoulders.
WALK_GOAL_CLEARANCE = float(os.environ.get("WALK_GOAL_CLEARANCE", "0.22"))
WALK_PLAN_MARGIN = 1.5        # m of room around start and goal to go round things
WALK_LOOKAHEAD = 0.3          # m along the path: the point it heads for (0.5 cut the corners: 0.29 m from a box planned at 0.42)
# An obstacle stays on the map this long after it was last seen: long enough to cover the
# lidar's blind ring round the robot (LIDAR_SELF_M), short enough that a person who walked past does not
# leave a wake that blocks ways already free (2 s did, 2026-11-05).
WALK_MEMORY_S = 1.0
WALK_BLIND_M = 0.55          # the bridge's LIDAR_SELF_M (0.45) and a margin: remembered inside only
# ONE MAP IS NOT ENOUGH TO STOP. On the robot (2026-11-11) an obstacle or a "no way" that showed in
# one map and was gone in the next (its own swinging hand, the floor while it sways) made it stop,
# start, stop: a stop_move makes a biped settle back a little, so it rocked and never got anywhere.
# Something in the way must show in WALK_BLOCK_MAPS maps in a row (0.4 s at 5 Hz) before it
# stops — unless it is already within WALK_BLOCK_NOW_M, then at once. A plan that fails keeps the
# last good one for WALK_NOWAY_GRACE_S, the same idea.
WALK_BLOCK_MAPS = 2
WALK_BLOCK_NOW_M = 0.35
WALK_NOWAY_GRACE_S = 0.6
# SLOW DOWN NEAR THINGS: with something within WALK_SLOW_M ahead (in a corridor WALK_SLOW_HALF
# each side), the speed falls in proportion, to WALK_SLOW_MIN of it at the stop distance — so it
# comes up to a person or a table gently and stops short, instead of at full speed then all at once.
# FACE THE WAY IT WALKS (2026-11-05: it crabbed 1.9 m sideways at 0.15 m/s and timed out). A walk
# turns to face its path, walks forward, and turns to the final heading at the end. Short moves keep
# the heading instead: a side step up to WALK_SIDESTEP_M, a step back up to WALK_BACKSTEP_M ("50 cm
# back" means backing up, not turning round). More than WALK_ALIGN_RAD off the path's direction =
# stop and turn first; within WALK_FREEZE_FACE_M of the goal the heading is held, or it would spin
# chasing a point under its own feet.
WALK_SIDESTEP_M = 0.4
WALK_BACKSTEP_M = 1.0
WALK_ALIGN_RAD = math.radians(35)
# ...and turns until within this, then sets off and corrects the rest while walking. 8° was finer
# than a G1 turning in place can settle (its torso sways more): on the robot (2026-11-11) it stood
# "turning to face the way, 9° off" for good.
WALK_ALIGNED_RAD = math.radians(15)
WALK_FREEZE_FACE_M = 0.35
# The heading it faces is toward a point this far along the path, and while it turns in place that
# heading is held. With the 0.3 m steering point, the few cm a G1 drifts turning in place swung the
# wanted heading ~18° and it turned after it for good ("turning to face the way, 18° off",
# 2026-11-11).
WALK_FACE_LOOK_M = 1.0
# SPEED AND MODE, chosen per walk on the page (2026-11-05), as on the drive pad: the walk's top
# speeds come with the request, up to WALK_CAPS (the command sender clamps at the same 1.2 / 0.5 /
# 1.2 anyway). Faster means stopping sooner: the stop distance grows WALK_STOP_PER_MS metres for
# every m/s over 0.3, and the slow zone starts WALK_SLOW_EXTRA_M before it.
WALK_CAPS = (1.2, 0.5, 1.2)
WALK_STOP_PER_MS = 1.0
WALK_SLOW_EXTRA_M = 0.7
# AUTO SPEED ("speed": "auto"): the robot picks its own top speed from how open the way ahead is —
# WALK_AUTO_CAP in the open, down to the slow preset's 0.3 m/s with anything within WALK_AUTO_SLOW_M
# of its path — and its stop distance follows the speed it is going, not the cap.
WALK_AUTO_CAP = (0.9, 0.4, 1.0)
WALK_AUTO_LOOK_M = 3.0          # how far ahead it looks to judge how open the way is
WALK_AUTO_HALF = 0.6            # ...and how wide (each side)
WALK_AUTO_SLOW_M = 0.8          # something this close along the way: the slowest
WALK_AUTO_GAIN = 0.4            # m/s more for every metre of room beyond that
# The locomotion modes a walk may start from, and switch between (rt FSM ids, g1_commands.py).
WALK_FSM = {500: "walk", 501: "walk", 801: "run", 802: "run"}
MODE_SWITCH_S = 4.0
WALK_SLOW_M = 1.2
WALK_SLOW_HALF = 0.35
WALK_SLOW_MIN = 0.35


class NoWay(RuntimeError):
    """No path: the message says why, in words for the operator and the agent."""


def plan_path(start, goal, cells, res=WALK_PLAN_RES, clearance=WALK_CLEARANCE,
              margin=WALK_PLAN_MARGIN, goal_clearance=WALK_GOAL_CLEARANCE):
    """A* on a grid round start and goal (x, y in metres, odom frame): the shortest way that keeps
    `clearance` from every obstacle cell — `goal_clearance` near the goal, where it may stand
    closer — as a list of points from start to goal with the straight stretches merged. Raises
    NoWay saying why when there is none. The start's own cell is always free (the robot is
    there), so a robot already too close to something can still walk away from it."""
    import heapq
    near_goal = min((math.hypot(x - goal[0], y - goal[1]) for x, y in cells), default=math.inf)
    if near_goal < goal_clearance:
        raise NoWay(f"the point is {near_goal * 100:.0f} cm from something: it needs "
                    f"{goal_clearance * 100:.0f} cm to stand there")
    x0 = min(start[0], goal[0]) - margin
    y0 = min(start[1], goal[1]) - margin
    w = int((max(start[0], goal[0]) + margin - x0) / res) + 1
    h = int((max(start[1], goal[1]) + margin - y0) / res) + 1

    def cell(p):
        return round((p[0] - x0) / res), round((p[1] - y0) / res)

    def inflate(radius):
        out = set()
        r = math.ceil(radius / res)
        disk = [(i, j) for i in range(-r, r + 1) for j in range(-r, r + 1)
                if i * i + j * j <= r * r]
        for p in cells:
            ci, cj = cell(p)
            if -r <= ci < w + r and -r <= cj < h + r:
                for i, j in disk:
                    out.add((ci + i, cj + j))
        return out

    blocked = inflate(clearance)
    # The straight legs of string pulling are checked against a margin half a cell wider: a line
    # between cell centres can pass that much closer than the cells it crosses.
    line_blocked = inflate(clearance + res / 2)
    s, g = cell(start), cell(goal)
    if goal_clearance < clearance:
        # Within `clearance` of the goal, only the tighter margin counts: the last stretch may
        # come in closer to things, so that it can stand where it was asked.
        # The same round the START, but only when the robot already stands that close (at a
        # table, after a grasp): it must be able to step back. Never while it walks past things,
        # or every replan would let it shave a little closer.
        tight = inflate(goal_clearance)
        rr = math.ceil(clearance / res)
        for end in (g, s) if s in blocked else (g,):
            for i in range(-rr, rr + 1):
                for j in range(-rr, rr + 1):
                    c = (end[0] + i, end[1] + j)
                    if i * i + j * j <= rr * rr and c not in tight:
                        blocked.discard(c)
                        line_blocked.discard(c)
    for c in (s, g):
        blocked.discard(c)
        line_blocked.discard(c)
    octile = math.sqrt(2) - 2

    def heur(c):
        dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
        return dx + dy + octile * min(dx, dy)

    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    came, cost = {s: None}, {s: 0.0}
    frontier = [(heur(s), s)]
    while frontier:
        _f, c = heapq.heappop(frontier)
        if c == g:
            break
        for di, dj, k in steps:
            n = (c[0] + di, c[1] + dj)
            if not (0 <= n[0] < w and 0 <= n[1] < h) or n in blocked:
                continue
            nc = cost[c] + k
            if nc < cost.get(n, math.inf):
                cost[n], came[n] = nc, c
                heapq.heappush(frontier, (nc + heur(n), n))
    if g not in came:
        raise NoWay("no way to the point: something blocks every way round it")
    path = [g]
    while came[path[-1]] is not None:
        path.append(came[path[-1]])
    path.reverse()

    def clear_line(a, b):
        """True if the straight segment a -> b (grid cells) crosses no blocked cell."""
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 1
        for k in range(n + 1):
            u = k / n
            c = (round(a[0] + u * (b[0] - a[0])), round(a[1] + u * (b[1] - a[1])))
            if c in line_blocked:
                return False
        return True

    # STRING PULLING: A* on a grid only moves in 8 directions, so a line at any other angle
    # comes out as a staircase — and every step looked like a turn round something (seen on the
    # robot 2026-11-05: 1 m forward and 0.5 m right with nothing in the way "went round something",
    # 1.2 m of zigzag). From each kept point, jump to the farthest later cell it sees in a
    # straight line that stays clear: with nothing in the way the path is one segment.
    keep, k = [path[0]], 0
    while k < len(path) - 1:
        far = len(path) - 1
        while far > k + 1 and not clear_line(path[k], path[far]):
            far -= 1
        keep.append(path[far])
        k = far
    pts = [(x0 + i * res, y0 + j * res) for i, j in keep]
    pts[0], pts[-1] = tuple(start), tuple(goal)
    return pts


def lookahead(path, pose, dist=WALK_LOOKAHEAD):
    """The point `dist` metres along `path` past the point of it nearest to `pose`."""
    best, at = math.inf, (0, 0.0)
    for k, (a, b) in enumerate(zip(path, path[1:])):
        sx, sy = b[0] - a[0], b[1] - a[1]
        L2 = sx * sx + sy * sy or 1e-9
        u = max(0.0, min(1.0, ((pose["x"] - a[0]) * sx + (pose["y"] - a[1]) * sy) / L2))
        d = math.hypot(a[0] + u * sx - pose["x"], a[1] + u * sy - pose["y"])
        if d < best:
            best, at = d, (k, u)
    k, u = at
    left = dist
    while k < len(path) - 1:
        a, b = path[k], path[k + 1]
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        rest = seg * (1 - u)
        if rest >= left and seg > 0:
            t = u + left / seg
            return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])
        left -= rest
        k, u = k + 1, 0.0
    return path[-1]


def coarsen_lidar(line, res=LIDAR_VIEW_RES):
    """The page's copy of a bridge line: the same, with its cells merged into `res` squares."""
    try:
        d = json.loads(line)
    except ValueError:
        return None
    r = float(d.get("res", 0.05))
    k = max(1, round(res / r))
    d["cells"] = sorted({(ix // k, iy // k) for ix, iy in d.get("cells", ())})
    d["res"] = r * k
    return json.dumps(d, separators=(",", ":"))


class Walk:
    """WALK TO A POINT, relative to where the robot stands (POST /walk/control). G1 only.

    One goal at a time, on its own thread: read the odometry (OdomStream, g1_odom_bridge), send
    `move` to the same command_sender the drive pad uses — its clamp and its 1 s dead-man still
    hold — and end with a stop_move whatever happens. The result follows the planner's contract
    (G1.md §6.9): success | uncertain | failure | cancelled, with the reason and the error left.

    Who wins: the OPERATOR. A stop from anywhere, a drive-pad move, or the Joints page taking an
    arm cancels the walk (the handlers call cancel()); a walk is refused while an arm is held.
    """

    def __init__(self, sender, odom, joint_control=None, clock=time.monotonic,
                 settle_s=WALK_SETTLE_S, hz=WALK_HZ, lidar=None, require_lidar=WALK_REQUIRE_LIDAR,
                 blocked_wait_s=WALK_BLOCKED_WAIT_S):
        self.sender = sender
        self.odom = odom
        self.lidar = lidar                  # JointStream of g1_lidar_bridge, or None (blind)
        self.require_lidar = require_lidar and lidar is not None
        self.blocked_wait_s = blocked_wait_s
        self.joint_control = joint_control
        self.clock = clock
        self.settle_s = settle_s
        self.hz = hz
        self.lock = threading.Lock()
        self.cancelled = threading.Event()
        self.thread = None
        self.state = None           # the last goal's snapshot: what status() reports
        self.seq = 0

    def running(self):
        t = self.thread
        return t is not None and t.is_alive()

    def status(self):
        with self.lock:
            return dict(self.state) if self.state else {"state": "idle"}

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def cancel(self, why, stop=True):
        """End a running walk. `stop`=False when the operator's own command is taking over (a
        drive-pad move): then the walk must not send a stop_move after it."""
        if self.running():
            self._set(cancel_reason=why, cancel_stops=stop)
            self.cancelled.set()
            audit("-", "walk-cancel", why, "-")
            return True
        return False

    def start(self, dx, dy, dyaw, lim=None, mode=None, auto=False):
        """(http code, answer). Refused while an arm is held or another walk runs."""
        if self.joint_control is not None and self.joint_control.busy():
            return 409, {"ok": False, "error": "the Joints page holds the arms: release them "
                                               "first"}
        with self.lock:
            if self.thread is not None and self.thread.is_alive():
                return 409, {"ok": False, "error": "a walk is already running",
                             "walk": dict(self.state)}
            self.seq += 1
            self.cancelled.clear()
            self.state = {"id": self.seq, "state": "starting", "phase": "translate",
                          "request": {"dx": dx, "dy": dy, "dyaw": dyaw}, "reason": None,
                          "start": None, "goal": None, "pose": None, "error": None,
                          "elapsed_s": 0.0, "blocked": None, "path": None,
                          "eyes": "lidar" if self.lidar is not None else "none"}
            self.state["speed"] = "auto" if auto else dict(
                zip(("vx", "vy", "vyaw"), lim or (WALK_VX, WALK_VY, WALK_VYAW)))
            self.state["mode"] = mode
            self.thread = threading.Thread(target=self._run, args=(dx, dy, dyaw, lim, mode, auto),
                                           name="walk", daemon=True)
            self.thread.start()
            return 200, {"ok": True, "walk": dict(self.state)}

    def plan(self, dx, dy, dyaw, wait_s=1.5):
        """(http code, answer) for a walk NOT taken: where it would go and the way it would take,
        from where the robot stands now and what the lidar sees now. Nothing moves. The page
        shows it and the operator accepts it (a goto), which plans again as it walks."""
        if not self.odom.subscribe(internal=True):
            return 503, {"ok": False, "error": "the odometry stream has no room for one more"}
        looking = self.lidar is not None and self.lidar.subscribe(internal=True)
        try:
            _seen, pose = self._pose(0, wait_s)
            if pose is None:
                return 503, {"ok": False, "error": "no odometry from the robot"}
            c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
            goal = {"x": pose["x"] + c * dx - s * dy, "y": pose["y"] + s * dx + c * dy,
                    "yaw": wrap_angle(pose["yaw"] + dyaw)}
            out = {"start": pose, "goal": goal, "request": {"dx": dx, "dy": dy, "dyaw": dyaw}}
            cells = None
            if looking:
                end = self.clock() + wait_s
                seen = 0
                while cells is None and self.clock() < end:
                    seen, line = self.lidar.next_line(seen, 0.3)
                    if line is not None:
                        cells, _d = parse_lidar(line)
            if cells is None:
                if self.require_lidar:
                    return 503, {"ok": False, **out, "error": "no obstacle map from the lidar"}
                return 200, {"ok": True, "plan": {**out, "path": [[pose["x"], pose["y"]],
                                                                   [goal["x"], goal["y"]]],
                                                  "eyes": "none"}}
            if math.hypot(dx, dy) < 0.01:
                path = [(pose["x"], pose["y"])]          # a turn in place: no way to plan
            else:
                try:
                    path = plan_path((pose["x"], pose["y"]), (goal["x"], goal["y"]), cells)
                except NoWay as exc:
                    return 200, {"ok": False, "plan": {**out, "path": None}, "error": str(exc)}
            return 200, {"ok": True, "plan": {**out, "eyes": "lidar", "path": [
                [round(x, 3), round(y, 3)] for x, y in path]}}
        finally:
            self.odom.unsubscribe(internal=True)
            if looking:
                self.lidar.unsubscribe(internal=True)

    # -- the loop -------------------------------------------------------- #
    def _pose(self, seen, timeout):
        """(seq, pose) of the next fresh odometry line, or (seq, None) if none arrived."""
        seen, line = self.odom.next_line(seen, timeout)
        if line is None:
            return seen, None
        try:
            d = json.loads(line)
        except ValueError:
            return seen, None
        if d.get("age_ms", -1) < 0 or d["age_ms"] > WALK_ODOM_STALE_S * 1000:
            return seen, None
        out = {k: float(d[k]) for k in ("x", "y", "yaw")}
        out["fsm"] = int(d.get("fsm", -1))
        return seen, out

    def _errors(self, pose, goal):
        return {"pos_m": round(math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"]), 3),
                "yaw_deg": round(math.degrees(wrap_angle(goal["yaw"] - pose["yaw"])), 1)}

    def _send(self, line):
        reply = self.sender.send(line)
        if not reply.startswith("ok"):
            raise RuntimeError(f"the robot refused '{line.split()[0]}': {reply}")

    def _eyes(self, lidar_seen, cells, since):
        """(seq, cells, time of the last fresh map) after a look at the lidar, never waiting."""
        if self.lidar is None:
            return lidar_seen, cells, since
        seq, line = self.lidar.next_line(lidar_seen, 0)
        if line is not None:
            got, _d = parse_lidar(line)
            if got is not None:
                return seq, got, self.clock()
        return seq, cells, since

    def _switch_mode(self, seen, pose, mode):
        """Put the robot in `mode` (walk | run) if it is in the other one; refuse if it is in
        neither — this never stands a squatting or sitting robot up. -> (seen, pose)."""
        fsm = pose.get("fsm", -1)
        if fsm not in WALK_FSM:
            raise RuntimeError(
                f"the robot is not walking or running (mode {fsm}): stand it up in Walk first, "
                f"from the remote or the drive pad" if fsm >= 0 else
                "the robot's mode could not be read, so it was not switched")
        if WALK_FSM[fsm] == mode:
            return seen, pose
        verb = {("walk", True): "start", ("walk", False): "walk_waist",
                ("run", True): "run", ("run", False): "run_waist"}[(mode, G1_WAIST_LOCK)]
        self._set(phase="mode")
        audit("-", "walk-mode", f"{WALK_FSM[fsm]} -> {mode} ({verb})", "-")
        self._send(verb)
        end = self.clock() + MODE_SWITCH_S
        while self.clock() < end:
            if self.cancelled.is_set():
                raise _WalkCancelled()
            seen, fresh = self._pose(seen, WALK_ODOM_STALE_S)
            if fresh is not None:
                pose = fresh
                if WALK_FSM.get(pose["fsm"]) == mode:
                    return seen, pose
        raise RuntimeError(f"it did not switch to {mode} in {MODE_SWITCH_S:g} s")

    def _run(self, dx, dy, dyaw, lim=None, mode=None, auto=False):
        if not self.odom.subscribe(internal=True):
            self._set(state="failure", reason="the odometry stream has no room for one more")
            return
        looking = self.lidar is not None and self.lidar.subscribe(internal=True)
        t0 = self.clock()
        pose = goal = None
        try:
            if self.require_lidar and not looking:
                raise RuntimeError("the lidar stream has no room for one more")
            lidar_seen, cells, seen_at = 0, None, None
            memory = {}                         # obstacle (x, y) -> when it was last seen
            plan, planned_at, no_way = None, None, None
            planned_ok = -math.inf              # when the last plan was found
            block_maps, block_seen = 0, None    # maps in a row with something in the way
            came_in = None                      # the way it came within WALK_NEAR_M (unit x, y)
            seen, pose = self._pose(0, WALK_ODOM_STALE_S * 2)
            if pose is None:
                raise RuntimeError("no odometry from the robot")
            if mode is not None:
                seen, pose = self._switch_mode(seen, pose, mode)
            min_v = WALK_MIN_V_RUN if WALK_FSM.get(pose.get("fsm")) == "run" else WALK_MIN_V
            stall_at, stall_left, short = None, math.inf, None
            lim = WALK_AUTO_CAP if auto else (lim or (WALK_VX, WALK_VY, WALK_VYAW))
            cap = lim

            def margins(top):
                """(stop, slow) distances for a top speed: the faster, the sooner."""
                stop = WALK_STOP_M + max(0.0, top - 0.3) * WALK_STOP_PER_MS
                return stop, stop + WALK_SLOW_EXTRA_M

            stop_m, slow_m = margins(lim[0])
            c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
            goal = {"x": pose["x"] + c * dx - s * dy, "y": pose["y"] + s * dx + c * dy,
                    "yaw": wrap_angle(pose["yaw"] + dyaw), "hold": pose["yaw"]}
            dist = math.hypot(dx, dy)
            # Generous: the slowest the robot goes, plus time to start and to stop.
            # Room for turning to face the way (up to half a turn) as well as the final turn.
            limit = dist / WALK_MIN_V + (abs(dyaw) + math.pi) / WALK_MIN_W + 10.0
            facing = not (dist <= WALK_SIDESTEP_M or
                          (dx < 0 and abs(dy) <= 0.3 and dist <= WALK_BACKSTEP_M))
            last_face = None
            aligning = False
            self._set(state="running", start=dict(pose), pose=dict(pose),
                      goal={k: goal[k] for k in ("x", "y", "yaw")}, limit_s=round(limit, 1))
            phase = "translate" if dist >= 0.01 else "turn"
            last_send = -1.0
            blocked_since, waited = None, 0.0
            while True:
                if self.cancelled.is_set():
                    raise _WalkCancelled()
                seen, fresh = self._pose(seen, WALK_ODOM_STALE_S)
                if fresh is None:
                    raise RuntimeError(f"odometry stopped for {WALK_ODOM_STALE_S:g} s")
                pose = fresh
                now = self.clock() - t0
                if now - waited - (now - blocked_since if blocked_since is not None else 0) > limit:
                    raise RuntimeError(f"not there after {limit:.0f} s")
                lidar_seen, cells, seen_at = self._eyes(lidar_seen, cells, seen_at)
                if self.require_lidar and ((seen_at is None and now > WALK_LIDAR_STALE_S * 2) or
                                           (seen_at is not None and
                                           self.clock() - seen_at > WALK_LIDAR_STALE_S)):
                    raise RuntimeError("no obstacle map from the lidar (is it pointed at PC2? "
                                       "tools/livox_set_host.py show)")
                if self.require_lidar and seen_at is None:
                    continue                    # the first map is on its way: do not move yet
                aim = face = None
                if cells is not None and phase == "translate":
                    if seen_at != planned_at:   # a new map: remember it, plan again
                        t_seen = self.clock()
                        for c in cells:
                            memory[c] = t_seen
                        # Remembered only where the lidar cannot look (its blind ring): what it
                        # can see and no longer sees is gone. Remembering everything 1 s left a
                        # trail of its own swinging hand ahead of it, which it then walked into
                        # ("an obstacle 14 cm ahead", 2026-11-11).
                        for c in [c for c, at in memory.items()
                                  if t_seen - at > WALK_MEMORY_S or (
                                      at != t_seen and math.hypot(c[0] - pose["x"], c[1] - pose["y"])
                                      > WALK_BLIND_M)]:
                            del memory[c]
                        planned_at = seen_at
                        try:
                            plan = plan_path((pose["x"], pose["y"]), (goal["x"], goal["y"]),
                                             list(memory))
                            no_way, planned_ok = None, t_seen
                            self._set(path=[[round(x, 2), round(y, 2)] for x, y in plan])
                        except NoWay as exc:
                            # Maybe someone stands there for a moment: stop and wait, like for
                            # something in front, and give up only if it stays (2026-11-05: a
                            # walk ended at once on "the point is 11 cm from something"). A
                            # single bad map keeps the last good plan (WALK_NOWAY_GRACE_S).
                            if plan is not None and t_seen - planned_ok <= WALK_NOWAY_GRACE_S:
                                no_way = None
                            else:
                                plan, no_way = None, str(exc)
                    if plan is not None and (facing or len(plan) > 2):
                        aim = lookahead(plan, pose)
                        if not facing:          # a short step round something: hold heading
                            face = math.atan2(aim[1] - pose["y"], aim[0] - pose["x"])
                if facing and phase == "translate":
                    if aligning and last_face is not None:
                        face = last_face        # turning in place: hold the heading it set out for
                    else:
                        target = (lookahead(plan, pose, WALK_FACE_LOOK_M) if plan is not None
                                  else (goal["x"], goal["y"]))
                        if math.hypot(goal["x"] - pose["x"],
                                      goal["y"] - pose["y"]) > WALK_FREEZE_FACE_M:
                            last_face = math.atan2(target[1] - pose["y"], target[0] - pose["x"])
                        face = last_face
                if no_way is not None and phase == "translate":
                    if blocked_since is None:
                        self._send("stop_move")
                        blocked_since = now
                        audit("-", "walk-blocked", no_way, "-")
                    self._set(blocked={"dist_m": 0.0, "reason": no_way})
                    if now - blocked_since > self.blocked_wait_s:
                        raise NoWay(no_way)
                    continue
                off = abs(wrap_angle(face - pose["yaw"])) if face is not None else 0.0
                aligning = (facing and phase == "translate" and face is not None
                            and off > (WALK_ALIGNED_RAD if aligning else WALK_ALIGN_RAD))
                turn_first = aligning
                if turn_first:
                    # Facing too far off the way: stop stepping and turn in place to it first.
                    err = wrap_angle(face - pose["yaw"])
                    w = max(-lim[2], min(lim[2], WALK_YAW_GAIN * err))
                    vx, vy, vyaw = 0.0, 0.0, math.copysign(max(abs(w), WALK_MIN_W), err)
                else:
                    if auto and cells is not None:
                        # How open is the way? The nearest thing within WALK_AUTO_HALF of the
                        # straight ahead, up to WALK_AUTO_LOOK_M: the more room, the faster.
                        ahead_dir = face if face is not None else pose["yaw"]
                        probe = (math.cos(ahead_dir - pose["yaw"]), math.sin(ahead_dir - pose["yaw"]))
                        room = obstacle_ahead(pose, probe[0], probe[1],
                                              list(memory) if memory else cells,
                                              WALK_AUTO_LOOK_M, WALK_AUTO_HALF)
                        top = cap[0] if room is None else max(
                            WALK_VX, min(cap[0], WALK_VX + WALK_AUTO_GAIN * (room - WALK_AUTO_SLOW_M)))
                        lim = (top, min(cap[1], max(WALK_VY, top / 2)), cap[2])
                        stop_m, slow_m = margins(top)
                        self._set(speed_now=round(top, 2))
                    if phase == "translate":
                        gx, gy = goal["x"] - pose["x"], goal["y"] - pose["y"]
                        left_now = math.hypot(gx, gy)
                        if came_in is None and 1e-6 < left_now < WALK_NEAR_M:
                            came_in = (gx / left_now, gy / left_now)
                        if came_in is not None and gx * came_in[0] + gy * came_in[1] < 0:
                            phase = "turn"      # walked past it: do not step back and forth
                        if left_now < WALK_STALL_NEAR_M:
                            if left_now < stall_left - WALK_STALL_GAIN_M:
                                stall_at, stall_left = now, left_now
                            elif stall_at is not None and now - stall_at > WALK_STALL_S:
                                phase = "turn"  # not getting any closer: stop trying
                                short = left_now
                    vx, vy, vyaw, phase = walk_command(pose, goal, phase, aim, face, lim, min_v)
                self._set(face=round(face, 4) if turn_first else None,
                          pose=dict(pose), phase="align" if turn_first else phase,
                          elapsed_s=round(now, 1), error=self._errors(pose, goal))
                if phase == "done":
                    break
                # Only what lies before the goal can be in the way: a table the walk ends 30 cm
                # from is where it is going, not something stepping in front.
                left = math.hypot(goal["x"] - pose["x"], goal["y"] - pose["y"])
                reach = min(stop_m, left + WALK_GOAL_CLEARANCE - 0.05)
                ahead = (obstacle_ahead(pose, vx, vy, list(memory) if memory else cells, reach)
                         if phase == "translate" else None)
                if ahead is None:
                    block_maps = 0
                elif seen_at != block_seen:
                    block_maps, block_seen = block_maps + 1, seen_at
                if (ahead is not None and blocked_since is None and ahead > WALK_BLOCK_NOW_M
                        and block_maps < WALK_BLOCK_MAPS):
                    ahead = None                # one map only: keep going (slowing) and look again
                if ahead is not None:
                    # Something in the way: stop, give it a moment to move off, then give up.
                    if blocked_since is None:
                        self._send("stop_move")
                        blocked_since = now
                        audit("-", "walk-blocked", f"{ahead:.2f} m ahead", "-")
                    self._set(blocked={"dist_m": round(ahead, 2)})
                    if now - blocked_since > self.blocked_wait_s:
                        raise RuntimeError(f"blocked: something {ahead:.2f} m ahead did not move "
                                           f"in {self.blocked_wait_s:g} s")
                    continue
                if blocked_since is not None:
                    waited += now - blocked_since      # time spent waiting does not count
                    blocked_since = None
                    self._set(blocked=None)
                if phase == "translate":
                    near = obstacle_ahead(pose, vx, vy, list(memory) if memory else cells,
                                          min(slow_m, left + WALK_GOAL_CLEARANCE),
                                          WALK_SLOW_HALF)
                    if near is not None:
                        f = (near - stop_m) / max(slow_m - stop_m, 1e-6)
                        f = max(WALK_SLOW_MIN, min(1.0, f))
                        speed = math.hypot(vx, vy)
                        if speed * f < min_v:
                            f = min_v / max(speed, 1e-6)
                        vx, vy = vx * min(1.0, f), vy * min(1.0, f)
                if now - last_send >= 1.0 / self.hz:
                    self._send(f"move {vx:.3f} {vy:.3f} {vyaw:.3f}")
                    last_send = now
            self._send("stop_move")
            # A biped finishes its step after the stop: measure where it ends, not where it was.
            end = self.clock() + self.settle_s
            while self.clock() < end:
                seen, fresh = self._pose(seen, WALK_ODOM_STALE_S)
                if fresh is not None:
                    pose = fresh
            err = self._errors(pose, goal)
            ok = err["pos_m"] <= WALK_OK_POS and abs(math.radians(err["yaw_deg"])) <= WALK_OK_YAW
            why = (f"it stopped getting closer {short * 100:.0f} cm from the point"
                   if short is not None else "stopped further off than expected")
            self._set(state="success" if ok else "uncertain", phase="done", pose=dict(pose),
                      error=err, elapsed_s=round(self.clock() - t0, 1),
                      reason=None if ok else why)
        except _WalkCancelled:
            with self.lock:
                why = self.state.get("cancel_reason") or "cancelled"
                stop = self.state.get("cancel_stops", True)
            if stop:
                self._stop_quietly()
            self._set(state="cancelled", reason=why, elapsed_s=round(self.clock() - t0, 1),
                      **({"pose": dict(pose), "error": self._errors(pose, goal)}
                         if pose and goal else {}))
        except Exception as exc:                  # every way out stops the robot
            self._stop_quietly()
            self._set(state="failure", reason=str(exc), elapsed_s=round(self.clock() - t0, 1),
                      **({"pose": dict(pose), "error": self._errors(pose, goal)}
                         if pose and goal else {}))
        finally:
            self.odom.unsubscribe(internal=True)
            if looking:
                self.lidar.unsubscribe(internal=True)
            with self.lock:
                st = dict(self.state)
            audit("-", "walk-end", f"id={st['id']} {st['state']}",
                  f"{st.get('reason') or '-'} error={st.get('error')}")

    def _stop_quietly(self):
        try:
            self.sender.send("stop_move")
        except Exception as exc:                  # the sender's dead-man is the last line
            log(f"walk: stop_move failed: {exc}")


class _WalkCancelled(Exception):
    pass


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "robot-relay"

    def log_message(self, *a):     # keep the journal to our own audit lines
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authorised(self):
        got = self.headers.get("Authorization", "")
        return got.startswith("Bearer ") and got[7:] == self.server.token

    def _video_config(self):
        """POST /video-config {fps?, width?, quality?, camera?, persist?}

        Applies the values to the running publisher immediately, and with persist=true
        also writes them to video.env so they survive the next restart. Applying and
        saving are separate on purpose: tuning while driving wants the former, and only
        the values you settle on deserve the latter.
        """
        if not self._authorised():
            audit(self.client_address[0], "video-config", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json(400, {"error": "bad json"})
        persist = bool(body.pop("persist", False))
        # The camera is not a number with a range, and it is applied by camera_source rather
        # than mjpeg_server, so it travels beside VIDEO_PARAMS instead of inside it.
        camera = body.pop("camera", None)
        try:
            env_updates = {}
            if camera is not None:
                env_updates["CAMERA"] = validate_camera(camera)
            if body or camera is None:          # an empty request is still refused
                env_updates.update(validate_video(body))
        except ValueError as exc:
            audit(self.client_address[0], "video-config", str(body), "rejected")
            return self._json(400, {"ok": False, "error": str(exc)})

        live, deferred = split_live(body)
        # A restart-only knob that is not being saved would do nothing at all, silently.
        # Refuse instead: a control that appears to work and does not is worse than one
        # that says no.
        if deferred and not persist:
            return self._json(400, {
                "ok": False,
                "error": f"{sorted(deferred)} only take effect when the video service "
                         f"restarts, so they must be saved — send persist=true"})

        applied = {}
        if live:
            applied = mjpeg_apply(live)
            if applied.get("error"):
                audit(self.client_address[0], "video-config", str(body),
                      "publisher-unreachable")
                return self._json(502, {"ok": False, "error": applied["error"]})
        if camera is not None:
            switched = camera_apply(camera)
            if switched.get("error"):
                audit(self.client_address[0], "video-config", f"camera={camera}",
                      "camera-switch-unreachable")
                return self._json(502, {"ok": False, "error": switched["error"]})
            applied.update(switched)
            body["camera"] = camera             # so the audit line below records it

        saved = False
        if persist:
            try:
                write_env_keys(VIDEO_ENV, env_updates)
                saved = True
            except OSError as exc:
                # The live change already took: report the partial success honestly rather
                # than pretending the whole request failed.
                audit(self.client_address[0], "video-config", str(body), "applied-not-saved")
                return self._json(200, {"ok": True, "running": applied, "saved": False,
                                        "error": f"applied but not saved: {exc}"})
        audit(self.client_address[0], "video-config", str(body),
              "applied+saved" if saved else "applied")
        return self._json(200, {
            "ok": True, "running": applied, "saved": saved,
            # Named explicitly so the UI can say WHICH values are waiting, instead of a
            # blanket "restart to apply" the operator has to decode.
            "pending_restart": sorted(deferred),
        })

    def _joint_stream(self):
        """GET /joints/stream — the G1's joint state, one JSON line per tick, until the
        caller hangs up. Read-only: nothing on this path can move the robot.

        A line is the bridge's own (src/g1_joint_bridge.cpp), passed through untouched; when
        no new one arrives within JOINT_HEARTBEAT_S the caller gets {"stale": true} instead,
        which is also how a dead caller is noticed (the write fails).
        """
        return self._ndjson_stream(self.server.joints, "joints")

    def _odom_stream(self):
        """GET /odom/stream — where the G1 thinks it is (src/g1_odom_bridge.cpp), one JSON line
        per tick, the same way as the joints. Read-only."""
        return self._ndjson_stream(getattr(self.server, "odom", None), "odom")

    def _lidar_stream(self):
        """GET /lidar/stream — the obstacle map around the G1 (src/g1_lidar_bridge.cpp), for the
        Navigation page: coarser (LIDAR_VIEW_RES) and slower (LIDAR_VIEW_HZ) than the walk's own
        copy, because this one crosses the uplink. Read-only."""
        return self._ndjson_stream(getattr(self.server, "lidar", None), "lidar",
                                   shape=coarsen_lidar, min_interval=1.0 / LIDAR_VIEW_HZ)

    def _odom_trail(self, method):
        """GET /odom/trail -> {ok, points: [[x, y, yaw, t], ...], cleared_at}: where the robot
        walked since it booted (or since the last clear). POST {"op": "clear"} starts it again."""
        src = self.client_address[0]
        if not self._authorised():
            audit(src, "odom-trail", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        trail = getattr(self.server, "trail", None)
        if trail is None:
            return self._json(404, {"error": f"no trail on this {ROBOT_MODEL} (G1 only, and its "
                                             f"odometry bridge built by ./build.sh)"})
        if method == "POST":
            trail.clear()
            audit(src, "odom-trail", "clear", "-")
        return self._json(200, {"ok": True, **trail.snapshot()})

    def _lidar_cloud(self):
        """GET /lidar/cloud — the 3D point cloud around the G1 for the page's 3D view (the lidar
        bridge's "cloud" lines: base64 int16 centimetres, ~2 Hz, 8 cm cubes). Read-only."""
        return self._ndjson_stream(getattr(self.server, "lidar", None), "lidar-cloud",
                                   side=True, heartbeat=3.0)

    def _ndjson_stream(self, joints, name, shape=None, min_interval=0.0, side=False,
                       heartbeat=JOINT_HEARTBEAT_S):
        src = self.client_address[0]
        if not self._authorised():
            audit(src, f"{name}-stream", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        if joints is None:
            return self._json(404, {"error": f"no {name} stream on this {ROBOT_MODEL} (G1 only, "
                                             f"and its bridge built by ./build.sh)"})
        if not joints.subscribe():
            return self._json(503, {"error": f"too many {name} viewers",
                                    "max_clients": joints.max_clients})
        audit(src, f"{name}-stream", "-", "opened")
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seen = 0
            while True:
                seen, line = (joints.next_side if side else joints.next_line)(seen, heartbeat)
                if line is not None and shape is not None:
                    line = shape(line)
                self.wfile.write(((line or '{"stale":true}') + "\n").encode())
                self.wfile.flush()
                if min_interval:
                    time.sleep(min_interval)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                        # the caller hung up: the normal way this ends
        finally:
            joints.unsubscribe()
            audit(src, f"{name}-stream", "-", "closed")

    def _joint_control(self):
        """POST /joints/control — move single arm joints of the G1 from the Joints page.

          {"op": "take", "safe"?: {joint: rad}}             -> {ok, session, allowed, state, w}
            With "safe", a warm joint, a quiet client or a release away from that pose first
            moves the arm there, then releases ("retreat" in every answer while it runs).
          {"op": "target", "session", "joint", "q": rad}   -> {ok, reply, state, w}
          {"op": "hand", "session", "side", "finger": 0-5, "pos": 0-1000}
          {"op": "keepalive" | "status" | "release", "session"}
          {"op": "trajectory", "session", "points": [{"t": s, "q": {joint: rad},
                                                     "hands": {side: [6 x 0-1000]}}]}
          {"op": "cancel", "session"}                       (freezes a running trajectory)
          {"op": "grasp", "session", "side", "force"?: N, "thumb"?: 0-1000, "thumb_flex"?: 0-1000}
                                                            (close until the fingertips confirm,
                                                             the thumb turned there first)
          {"op": "open", "session", "side"}
          {"op": "keep_hands", "session"}                   (the release leaves the hands as they
                                                             are now, not as they were at take)
          Every answer carries {"grasp": {"left"|"right": {state: closing | held | empty |
          uncertain, forces: [5 x N or null]} | null}} (rule 12 of joint_ctl_core.hpp).
          Every answer in a session with a trajectory carries {"trajectory": {state, reason,
          t, duration, points, error_rad}}: state running | success | uncertain | failure |
          cancelled (G1.md §6.9, the planner's contract).

        The limits are the sender's (src/joint_ctl_core.hpp); this checks the token, the
        session and the shape of the numbers, and keeps an audit line per op that moves
        something (keepalives and status are not logged: ten a second would bury the rest).
        """
        src = self.client_address[0]
        if not self._authorised():
            audit(src, "joints-control", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        ctl = self.server.joint_control
        if ctl is None:
            return self._json(404, {"error": f"no joint control on a {ROBOT_MODEL}"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json(400, {"error": "bad json"})
        if not isinstance(body, dict):
            return self._json(400, {"error": "bad json"})
        op = body.get("op")
        # A release is never rate limited, like a stop: refusing one is the dangerous answer.
        if op != "release" and not self.server.joint_limiter.allow():
            return self._json(429, {"error": "rate limited"})
        if op == "take" and getattr(self.server, "walk", None) is not None and self.server.walk.running():
            audit(src, "joints-control", "take", "rejected-walking")
            return self._json(409, {"ok": False, "error": "the robot is walking to a point: "
                                                         "cancel the walk first"})
        if op == "take":
            code, out = ctl.take(body.get("safe"))
            audit(src, "joints-control", "take", out.get("reply") or out.get("error", "-"))
        else:
            pos = body.get("force") if op == "grasp" else body.get("pos")
            code, out = ctl.op(body.get("session"), op, body.get("joint"), body.get("q"),
                               body.get("side"), body.get("finger"), pos,
                               body.get("points"), body.get("thumb"), body.get("thumb_flex"))
            if op in ("target", "hand", "release", "trajectory", "cancel", "grasp", "open"):
                pts = body.get("points")
                detail = (f"{body.get('joint')}={body.get('q')}" if op == "target" else
                          f"{body.get('side')}[{body.get('finger')}]={body.get('pos')}"
                          if op == "hand" else
                          f"trajectory points={len(pts) if isinstance(pts, list) else '-'}"
                          if op == "trajectory" else
                          f"{op} {body.get('side')}" if op in ("grasp", "open") else op)
                audit(src, "joints-control", detail, out.get("reply") or out.get("error", "-"))
        return self._json(code, out)

    def _walk_control(self):
        """POST /walk/control — the G1 walks to a pose RELATIVE to where it stands (see Walk).

          {"op": "goto", "dx": m forward, "dy": m left, "dyaw": rad left,
           "speed"?: {vx, vy, vyaw} top speeds (<= 1.2 / 0.5 / 1.2) | "auto",
           "mode"?: "walk" | "run"}
                                                                              -> {ok, walk}
          {"op": "plan", same numbers} -> {ok, plan: {start, goal, path}}: the way it WOULD take,
                                          nothing moves (ok false + error when there is none)
          {"op": "cancel"}                                                    -> {ok, walk}
          {"op": "status"}                                                    -> {ok, walk}

        `walk` is {id, state: starting | running | success | uncertain | failure | cancelled,
        phase: translate | turn | done, request, start, goal, pose (odom frame: x, y, yaw),
        error: {pos_m, yaw_deg}, reason, elapsed_s, limit_s}.
        """
        src = self.client_address[0]
        if not self._authorised():
            audit(src, "walk-control", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        walk = getattr(self.server, "walk", None)
        if walk is None:
            return self._json(404, {"error": f"no walk control on this {ROBOT_MODEL} (G1 only, "
                                             f"and g1_odom_bridge built by ./build.sh)"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            body = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json(400, {"error": "bad json"})
        if not isinstance(body, dict):
            return self._json(400, {"error": "bad json"})
        op = body.get("op")
        if op == "status":
            return self._json(200, {"ok": True, "walk": walk.status()})
        if op == "cancel":
            # Never rate limited, like a stop.
            walk.cancel(f"cancel from {src}")
            return self._json(200, {"ok": True, "walk": walk.status()})
        if op not in ("goto", "plan"):
            return self._json(400, {"error": "op must be goto, plan, cancel or status"})
        if not self.server.limiter.allow():
            return self._json(429, {"error": "rate limited"})
        try:
            dx, dy, dyaw = validate_goto(body)
            lim, mode = walk_options(body)
        except ValueError as exc:
            return self._json(400, {"error": str(exc)})
        auto = lim is None and body.get("speed") == "auto"
        if op == "plan":
            # Moves nothing: the page shows the way and the operator accepts it with a goto.
            return self._json(*walk.plan(dx, dy, dyaw))
        code, out = walk.start(dx, dy, dyaw, lim, mode, auto)
        audit(src, "walk-goto", f"dx={dx:.2f} dy={dy:.2f} dyaw={dyaw:.2f} "
                                f"speed={'auto' if auto else f'{(lim or (WALK_VX,))[0]:g}'} "
                                f"mode={mode or '-'}",
              "started" if out.get("ok") else out.get("error", "-"))
        return self._json(code, out)

    def do_GET(self):
        if self.path.split("?")[0].rstrip("/").startswith("/audio/"):
            return self._audio("GET")
        if self.path.split("?")[0].rstrip("/") == "/camera/depth":
            return self._depth()
        if self.path.split("?")[0].rstrip("/") == "/joints/stream":
            return self._joint_stream()
        if self.path.split("?")[0].rstrip("/") == "/odom/stream":
            return self._odom_stream()
        if self.path.split("?")[0].rstrip("/") == "/lidar/stream":
            return self._lidar_stream()
        if self.path.split("?")[0].rstrip("/") == "/lidar/cloud":
            return self._lidar_cloud()
        if self.path.split("?")[0].rstrip("/") == "/odom/trail":
            return self._odom_trail("GET")
        if self.path.split("?")[0].rstrip("/") == "/video-config":
            if not self._authorised():
                return self._json(401, {"error": "unauthorised"})
            # RUNNING and SAVED are different things and the UI must be able to show both:
            # editing video.env without restarting is exactly how this report started
            # lying before, which is why /health reads /proc instead of the file.
            saved = read_env_file(VIDEO_ENV)
            camera = camera_live()
            # The cameras THIS robot offers come from camera_source (the G1 has no rs_color);
            # CAMERAS here is the allowlist of every name, which is not the same thing.
            offered = camera.pop("cameras", None) or list(CAMERAS)
            return self._json(200, {
                "ok": True,
                "running": {**mjpeg_live(), **camera},
                "saved": {**{k: saved.get(name) for k, (name, *_) in VIDEO_PARAMS.items()},
                          "camera": saved.get("CAMERA")},
                # `live` tells the UI which knobs apply instantly and which cost a restart
                # of the video service — the difference the operator must see BEFORE
                # touching one while driving.
                "limits": {k: {"min": lo, "max": hi, "live": live}
                           for k, (_n, _c, lo, hi, live) in VIDEO_PARAMS.items()},
                # Not a range, so not in `limits`: a client iterating those expects min/max.
                "cameras": offered,
            })
        if self.path.split("?")[0] != "/health":
            return self._json(404, {"error": "not found"})
        proc = self.server.sender.proc
        self._json(200, {"ok": True,
                         "sender_alive": bool(proc and proc.poll() is None),
                         "robot_model": ROBOT_MODEL,
                         "verbs": sorted(VERBS | ARG_VERBS),
                         # Everything below is configured ON THE ROBOT: the app can only
                         # read it, so the robot reports it instead of the app guessing.
                         "video": video_status(),
                         "battery": battery_status(),
                         "telemetry": telemetry_status(),
                         "limits": limits_status(),
                         "udp": ({"port": RELAY_UDP_PORT, **self.server.udp.stats}
                                 if self.server.udp else {"port": 0}),
                         "joints": (self.server.joints.status() if self.server.joints
                                    else {"running": False, "available": False}),
                         "odom": (self.server.odom.status() if getattr(self.server, "odom", None)
                                  else {"running": False, "available": False}),
                         "walk": (self.server.walk.status() if getattr(self.server, "walk", None)
                                  else {"state": "unavailable"}),
                         "lidar": (self.server.lidar.status()
                                   if getattr(self.server, "lidar", None)
                                   else {"running": False, "available": False}),
                         "joint_control": (self.server.joint_control.status()
                                           if self.server.joint_control
                                           else {"active": False, "available": False}),
                         "audio": {"available": self.server.audio is not None}})

    def _depth(self):
        """GET /camera/depth?frames=N -> image/png, 16-bit millimetres, plus X-Depth-Info.

        One picture on request, for the PC to measure where a thing is before a grasp; never a
        stream. Whether the robot serves it at all is camera_source's DEPTH_SNAPSHOT (on on the
        G1, off on the Go2's LTE). The token, like every route that reaches the robot."""
        src = self.client_address[0]
        if not self._authorised():
            audit(src, "camera-depth", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        query = dict(p.split("=", 1) for p in self.path.partition("?")[2].split("&") if "=" in p)
        try:
            frames = int(query.get("frames", "5"))
            if not 1 <= frames <= 15:
                raise ValueError
        except ValueError:
            return self._json(400, {"ok": False, "error": "'frames' must be 1..15"})
        png, info = depth_fetch(frames)
        if png is None:
            code, body = info
            audit(src, "camera-depth", f"frames={frames}", f"refused {body['error']}")
            return self._json(code, body)
        audit(src, "camera-depth", f"frames={frames}", f"ok {len(png)} bytes")
        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(png)))
        self.send_header("X-Depth-Info", info)
        self.end_headers()
        self.wfile.write(png)
        return None

    def _audio(self, method):
        """The Go2's speaker. All of it needs the token, like every other route that acts.

          POST /audio/say     body: a WAV        -> {ok, seconds}   (replies once uploaded)
          POST /audio/stop                       -> {ok}
          GET  /audio/volume                     -> {ok, volume}
          POST /audio/volume  {"level": 0-10}    -> {ok, volume}
          POST /audio/led     {"r","g","b": 0-255} -> {ok}   (the G1's head LED; the Go2 says no)
            It is the MIDDLE LED of the head. Unitree's own service sets it back to blue a few
            seconds later: a colour that must stay is resent about once a second. Brightness is
            the colour's scale: 40,0,0 is a dim red, 255,0,0 the brightest (2026-10-30).
          GET  /audio/listen?seconds=N           -> a 16 kHz mono WAV of the robot's mic (G1)
          GET  /audio/mic/stream                 -> the mic as raw 16 kHz s16le, until hung up
        """
        src = self.client_address[0]
        path = self.path.split("?")[0].rstrip("/")
        if not self._authorised():
            audit(src, "audio", path, "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        if self.server.audio is None:
            return self._json(404, {"ok": False,
                                    "error": f"no speaker on this {ROBOT_MODEL} relay: "
                                             f"{AUDIO_SENDER} is not built (./build.sh)"})
        n = int(self.headers.get("Content-Length", "0") or 0)
        if n > AUDIO_MAX_BYTES:
            return self._json(413, {"ok": False, "error": f"over {AUDIO_MAX_BYTES} bytes"})
        body = self.rfile.read(n) if n else b""

        if method == "GET" and path == "/audio/listen":
            return self._listen(src)
        if method == "GET" and path == "/audio/mic/stream":
            return self._mic_stream(src)
        if method == "GET" and path == "/audio/volume":
            line = "volume"
        elif method == "POST" and path == "/audio/volume":
            try:
                level = validate_volume(json.loads(body or b"{}").get("level"))
            except (ValueError, AttributeError) as exc:
                return self._json(400, {"ok": False, "error": str(exc)})
            line = f"volume {level}"
        elif method == "POST" and path == "/audio/led":
            try:
                r, g, b = validate_rgb(json.loads(body or b"{}"))
            except ValueError as exc:
                return self._json(400, {"ok": False, "error": str(exc)})
            line = f"led {r} {g} {b}"
        elif method == "POST" and path == "/audio/stop":
            line = "stop"
        elif method == "POST" and path == "/audio/say":
            try:
                info = parse_wav(body)
            except ValueError as exc:
                audit(src, "audio-say", f"bytes={len(body)}", "rejected")
                return self._json(400, {"ok": False, "error": str(exc)})
            # A file, not the bytes on the pipe: the sender's protocol is one text line.
            with tempfile.NamedTemporaryFile(prefix="relay-say-", suffix=".wav",
                                             delete=False) as fh:
                fh.write(body)
            try:
                reply = self.server.audio.send(f"say {fh.name}")
            finally:
                os.unlink(fh.name)
            audit(src, "audio-say", f"seconds={info['seconds']} rate={info['rate']}", reply)
            if not reply.startswith("ok"):
                return self._json(502, {"ok": False, "error": reply})
            return self._json(200, {"ok": True, "seconds": info["seconds"]})
        else:
            return self._json(404, {"error": "not found"})

        reply = self.server.audio.send(line)
        if line != "volume":
            audit(src, "audio", line, reply)
        if not reply.startswith("ok"):
            return self._json(502, {"ok": False, "error": reply})
        out = {"ok": True}
        if reply.startswith("ok volume "):
            # The scale travels with the value: 4 is quiet on a G1 and loud on a Go2.
            out.update(volume=int(reply.split()[-1]), max=VOLUME_MAX)
        return self._json(200, out)

    def _listen(self, src):
        """GET /audio/listen?seconds=N: N seconds of the robot's microphone, as a WAV."""
        mic = getattr(self.server, "mic", None)
        if mic is None:
            return self._json(404, {"ok": False, "error": f"no microphone on this {ROBOT_MODEL} "
                                                          "relay (the G1's only)"})
        query = dict(p.split("=", 1) for p in self.path.partition("?")[2].split("&") if "=" in p)
        try:
            seconds = validate_listen_seconds(query.get("seconds", "5"))
        except ValueError as exc:
            return self._json(400, {"ok": False, "error": str(exc)})
        try:
            pcm = mic.listen(seconds)
        except RuntimeError as exc:
            audit(src, "audio-listen", f"seconds={seconds:g}", f"refused {exc}")
            return self._json(409 if str(exc).startswith("busy") else 502,
                              {"ok": False, "error": str(exc)})
        except (TimeoutError, OSError) as exc:
            audit(src, "audio-listen", f"seconds={seconds:g}", f"failed {exc}")
            return self._json(504, {"ok": False, "error": str(exc)})
        audit(src, "audio-listen", f"seconds={seconds:g}", f"ok {len(pcm) / 2 / MIC_RATE:.2f}s")
        wav = pcm_to_wav(pcm)
        self.send_response(200)
        self.send_header("Content-Type", "audio/wav")
        self.send_header("Content-Length", str(len(wav)))
        self.end_headers()
        self.wfile.write(wav)
        return None

    def _mic_stream(self, src, open_socket=None):
        """GET /audio/mic/stream: the robot's mic as raw 16 kHz mono s16le, packet by packet,
        until the caller hangs up — then, if it was the last stream, the mic goes off. Nothing
        is sent while the robot sends nothing; the caller decides how long that may last."""
        mic = getattr(self.server, "mic", None)
        if mic is None:
            return self._json(404, {"ok": False, "error": f"no microphone on this {ROBOT_MODEL} "
                                                          "relay (the G1's only)"})
        try:
            mic.open_stream()
        except RuntimeError as exc:
            audit(src, "audio-mic-stream", "-", f"refused {exc}")
            return self._json(409 if str(exc).startswith("busy") else 502,
                              {"ok": False, "error": str(exc)})
        audit(src, "audio-mic-stream", "-", "opened")
        sock = None
        sent = 0
        try:
            sock = (open_socket or mic_socket)()
            sock.settimeout(1.0)
            self.send_response(200)
            self.send_header("Content-Type", f"audio/L16;rate={MIC_RATE};channels=1")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            while True:
                try:
                    data, _src = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                self.wfile.write(data)
                self.wfile.flush()
                sent += len(data)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                        # the caller hung up: the normal way this ends
        finally:
            if sock is not None:
                sock.close()
            mic.close_stream()
            audit(src, "audio-mic-stream", "-", f"closed {sent / 2 / MIC_RATE:.1f}s")
        return None

    def do_POST(self):
        if self.path.split("?")[0].rstrip("/").startswith("/audio/"):
            return self._audio("POST")
        if self.path.split("?")[0].rstrip("/") == "/video-config":
            return self._video_config()
        if self.path.split("?")[0].rstrip("/") == "/joints/control":
            return self._joint_control()
        if self.path.split("?")[0].rstrip("/") == "/walk/control":
            return self._walk_control()
        if self.path.split("?")[0].rstrip("/") == "/odom/trail":
            return self._odom_trail("POST")
        if self.path.split("?")[0] != "/cmd":
            return self._json(404, {"error": "not found"})
        if not self._authorised():
            audit(self.client_address[0], "-", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        try:
            n = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json(400, {"error": "bad json"})

        verb = str(payload.get("verb", ""))
        src = self.client_address[0]
        # A stop is never rate limited: repeating one is harmless, refusing one is not.
        if verb != "stop_move" and not self.server.limiter.allow():
            return self._json(429, {"error": "rate limited"})
        # Optional `ts`, the executor's clock at send time — the same ordering the UDP path
        # uses, so the two paths cannot overtake each other. Absent = an older executor.
        ts = payload.get("ts")
        if ts is not None:
            try:
                ts = float(ts)
            except (TypeError, ValueError):
                return self._json(400, {"error": "ts must be a number"})
            if not math.isfinite(ts):
                return self._json(400, {"error": "ts must be finite"})
        # The operator wins over a walk to a point: a stop stops it, anything else takes over.
        if getattr(self.server, "walk", None) is not None and verb not in ("", "keepalive"):
            self.server.walk.cancel(f"{verb} from {src}", stop=verb == "stop_move")
        if verb == "stop_move":
            self.server.order.stop(ts, time.time())
            # A stop pressed anywhere also hands back an arm held from the Joints page.
            if self.server.joint_control is not None:
                self.server.joint_control.release_all(f"stop_move from {src}")
        # While the page holds an arm, the robot stands: no walking, no arm action fighting it.
        elif (self.server.joint_control is not None and self.server.joint_control.busy()
              and (verb in ARG_VERBS or verb in G1_ACTIONS)):
            audit(src, verb, "-", "rejected-arms-held")
            return self._json(409, {"ok": False, "reply": "err the Joints page holds the arms: "
                                    "release them first"})

        if verb == "move":
            try:
                vx = float(payload.get("vx", 0))
                vy = float(payload.get("vy", 0))
                vyaw = float(payload.get("vyaw", 0))
            except (TypeError, ValueError):
                return self._json(400, {"error": "vx/vy/vyaw must be numbers"})
            # NaN passes float() and every clamp, all the way into Move().
            if not all(math.isfinite(v) for v in (vx, vy, vyaw)):
                return self._json(400, {"error": "vx/vy/vyaw must be finite"})
            if ts is not None and not self.server.order.admit_move(ts):
                audit(src, verb, f"ts={ts:.3f}", "rejected-stale")
                return self._json(409, {"ok": False, "reply": "err stale: a newer command "
                                        "or a stop already arrived"})
            # Values are clamped again in command_sender: this is convenience, not the limit.
            line = f"move {vx:.3f} {vy:.3f} {vyaw:.3f}"
            detail = f"vx={vx:.3f} vy={vy:.3f} vyaw={vyaw:.3f}"
        elif verb == "joy" and "joy" in ARG_VERBS:
            try:
                sticks = [float(payload.get(k, 0)) for k in ("lx", "ly", "rx", "ry")]
            except (TypeError, ValueError):
                return self._json(400, {"error": "lx/ly/rx/ry must be numbers"})
            if not all(math.isfinite(v) for v in sticks):
                return self._json(400, {"error": "lx/ly/rx/ry must be finite"})
            # Clamped to -1..1 again in the sender, which also refuses it outside pose.
            line = "joy " + " ".join(f"{v:.3f}" for v in sticks)
            detail = "lx={:.3f} ly={:.3f} rx={:.3f} ry={:.3f}".format(*sticks)
        elif verb in VERBS:
            line, detail = verb, "-"
        else:
            audit(src, verb or "-", "-", "rejected-unknown-verb")
            return self._json(400, {"error": f"unknown verb '{verb}'",
                                    "allowed": sorted(VERBS | ARG_VERBS)})

        reply = self.server.sender.send(line)
        audit(src, verb, detail, reply)
        ok = reply.startswith("ok")
        self._json(200 if ok else 502, {"ok": ok, "reply": reply})


def main():
    token = TOKEN
    if not token and os.path.exists(TOKEN_FILE):
        with open(TOKEN_FILE) as fh:
            token = fh.read().strip()
    if not token:
        sys.exit(f"no token: set RELAY_TOKEN or create {TOKEN_FILE}\n"
                 f"  printf '%s' 'A-LONG-RANDOM-STRING' > {TOKEN_FILE} && "
                 f"chmod 600 {TOKEN_FILE}")

    srv = ThreadingHTTPServer((BIND, PORT), Handler)
    srv.token = token
    bad = sender_mismatch(SENDER, ROBOT_MODEL)
    if bad:
        # Refuse rather than run the wrong code: a relay that does not start leaves the robot
        # still; one that starts on the wrong sender drives it with the wrong rules.
        log(f"REFUSING TO START: {bad}. Fix SENDER_BIN in relay.env (or delete it: the default "
            f"follows ROBOT_MODEL) and in the installed unit, then restart.")
        sys.exit(2)
    srv.sender = Sender([SENDER])
    srv.limiter = RateLimiter(MAX_PER_SEC)
    srv.order = CommandOrder()
    # Started lazily by the first viewer, so a relay nobody watches costs nothing extra.
    srv.joints = JointStream([JOINT_BRIDGE]) if ROBOT_MODEL == "g1" else None
    # Started by "take control" and gone after the release; G1 only.
    srv.joint_control = (JointControl([JOINT_SENDER], srv.sender, srv.order, stream=srv.joints)
                         if ROBOT_MODEL == "g1" else None)
    srv.joint_limiter = RateLimiter(JOINT_CTL_MAX_PER_SEC)
    # The odometry and the walk to a point, only if the bridge was built (G1 only).
    srv.odom = srv.walk = srv.lidar = srv.trail = None
    if ROBOT_MODEL == "g1":
        if os.access(LIDAR_BRIDGE, os.X_OK):
            srv.lidar = JointStream([LIDAR_BRIDGE], label="lidar bridge",
                                    side_prefix='{"kind":"cloud"')
        else:
            log(f"no {LIDAR_BRIDGE}: /lidar is off and walks are blind until ./build.sh is run")
        if os.access(ODOM_BRIDGE, os.X_OK):
            srv.odom = JointStream([ODOM_BRIDGE], label="odom bridge")
            srv.walk = Walk(srv.sender, srv.odom, srv.joint_control, lidar=srv.lidar)
            srv.trail = OdomTrail(srv.odom).start()
        else:
            log(f"no {ODOM_BRIDGE}: /odom and /walk are off until ./build.sh is run")
    # The speaker, only if its binary was built (a relay pulled but not rebuilt keeps working,
    # it just has no /audio).
    srv.audio = None
    srv.mic = None
    if os.access(AUDIO_SENDER, os.X_OK):
        srv.audio = Sender([AUDIO_SENDER, os.environ.get("DDS_IFACE", "eth0")],
                           label="audio_sender", timeout=AUDIO_REPLY_TIMEOUT, lazy=True)
        if ROBOT_MODEL == "g1":
            srv.mic = Mic(srv.audio)
    else:
        log(f"no {AUDIO_SENDER}: /audio is off until ./build.sh is run")
    srv.udp = None
    if RELAY_UDP_PORT:
        srv.udp = UdpControl(token.encode(), srv.sender, srv.limiter, srv.order,
                             srv.joint_control, srv.walk)
        usock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        usock.bind((BIND, RELAY_UDP_PORT))
        threading.Thread(target=srv.udp.serve, args=(usock,), name="udp-control",
                         daemon=True).start()
        log(f"udp teleop on {BIND}:{RELAY_UDP_PORT} (move/stop_move only, HMAC)")
    log(f"listening on {BIND}:{PORT}  audit={AUDIT_LOG}  limit={MAX_PER_SEC}/s")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        if srv.walk is not None:
            srv.walk.cancel("relay shutting down")
        # The arms first: EOF makes the joint sender ramp its weight down to 0 before exiting,
        # so PC1 has the arms back before anything else goes away.
        if srv.joint_control is not None:
            srv.joint_control.stop()
        # Closing the child's stdin makes command_sender StopMove before it exits, so
        # shutting the relay down can never leave the robot walking.
        try:
            srv.sender.proc.stdin.close()
            srv.sender.proc.wait(timeout=5)
        except Exception:
            pass
        if srv.joints is not None:
            srv.joints.stop()
        if srv.odom is not None:
            srv.odom.stop()
        if srv.lidar is not None:
            srv.lidar.stop()
        # EOF makes the audio sender leave megaphone mode before it exits.
        if srv.audio is not None and srv.audio.proc is not None:
            try:
                srv.audio.proc.stdin.close()
                srv.audio.proc.wait(timeout=5)
            except Exception:
                pass


if __name__ == "__main__":
    main()
