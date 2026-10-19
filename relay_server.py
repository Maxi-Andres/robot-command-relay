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

# Moving single arm joints from the Joints page (POST /joints/control, G1 only): see JointControl.
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


def record_mic(seconds, open_socket=None, clock=time.monotonic):
    """`seconds` of the mic multicast as PCM bytes. TimeoutError when no packet comes within
    MIC_FIRST_PACKET_S (mic mode off, or PC1 not streaming). `open_socket` is for tests."""
    want = int(seconds * MIC_RATE) * 2
    if open_socket is None:
        def open_socket():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
            sock.bind(("", MIC_PORT))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                            socket.inet_aton(MIC_GROUP) + socket.inet_aton(mic_local_ip()))
            return sock
    sock = open_socket()
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
    """Mic mode on demand: on for a listen, off after MIC_IDLE_OFF_S without one. One listen
    at a time — a second one while recording is refused, not queued."""

    def __init__(self, audio, record=record_mic, idle_off_s=None):
        self.audio = audio
        self.record = record
        self.idle_off_s = MIC_IDLE_OFF_S if idle_off_s is None else idle_off_s
        self.busy = threading.Lock()
        self.state = threading.Lock()
        self.on = False
        self.timer = None

    def listen(self, seconds):
        """PCM bytes, or raises RuntimeError (busy, or the sender refused) / TimeoutError."""
        if not self.busy.acquire(blocking=False):
            raise RuntimeError("busy: another listen is recording")
        try:
            with self.state:
                if self.timer:
                    self.timer.cancel()
                    self.timer = None
                if not self.on:
                    reply = self.audio.send("mic on")
                    if not reply.startswith("ok"):
                        raise RuntimeError(f"the robot did not turn its mic on: {reply}")
                    self.on = True
            return self.record(seconds)
        finally:
            with self.state:
                if self.on:
                    self.timer = threading.Timer(self.idle_off_s, self.off)
                    self.timer.daemon = True
                    self.timer.start()
            self.busy.release()

    def off(self):
        with self.state:
            if self.on:
                reply = self.audio.send("mic off")
                self.on = not reply.startswith("ok")
                log(f"mic off after {self.idle_off_s:g} s idle: {reply}")


def validate_volume(value):
    """The volume level as an int 0-10, or ValueError."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != int(value):
        raise ValueError(f"'level' must be a whole number, got {value!r}")
    if not 0 <= value <= VOLUME_MAX:
        raise ValueError(f"'level' must be between 0 and {VOLUME_MAX}, got {value}")
    return int(value)


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

    def __init__(self, key, sender, limiter, order, joint_control=None):
        self.key = key
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

    def __init__(self, argv, max_clients=JOINT_MAX_CLIENTS, idle_s=JOINT_IDLE_S):
        self.argv = argv
        self.max_clients = max_clients
        self.idle_s = idle_s
        self.cond = threading.Condition()
        self.proc = None
        self.latest = None
        self.seq = 0
        self.clients = 0
        self._idle_timer = None

    def _ensure_running(self):
        """Start the child if it is not running. Caller holds self.cond."""
        if self.proc is not None and self.proc.poll() is None:
            return
        self.proc = subprocess.Popen(self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.latest = None
        log(f"joint bridge started (pid {self.proc.pid})")
        threading.Thread(target=self._read, args=(self.proc,), name="joint-bridge",
                         daemon=True).start()

    def _read(self, proc):
        for line in proc.stdout:
            line = line.strip()
            # Only whole JSON objects go out: a torn or garbled line must not reach a viewer.
            if not (line.startswith("{") and line.endswith("}")):
                continue
            with self.cond:
                self.latest = line
                self.seq += 1
                self.cond.notify_all()
        with self.cond:
            if self.proc is proc:
                self.proc = None
                self.latest = None
            self.cond.notify_all()
        log(f"joint bridge exited (code {proc.wait()})")

    def subscribe(self):
        """Register a viewer. False when the viewer cap is reached."""
        with self.cond:
            if self.clients >= self.max_clients:
                return False
            self.clients += 1
            if self._idle_timer is not None:
                self._idle_timer.cancel()
                self._idle_timer = None
            self._ensure_running()
            return True

    def unsubscribe(self):
        with self.cond:
            self.clients = max(0, self.clients - 1)
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
                    "clients": self.clients, "max_clients": self.max_clients}


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

    def __init__(self, argv, cmd_sender=None, order=None, reply_timeout=None):
        self.argv = argv
        self.cmd_sender = cmd_sender
        self.order = order
        self.reply_timeout = reply_timeout or REPLY_TIMEOUT
        self.lock = threading.Lock()        # one op at a time: the protocol is one-in-one-out
        self.proc = None
        self.session = None
        self.replies = queue.Queue()
        self.state = {"state": "off", "w": 0.0}
        self.last_event = None

    def busy(self):
        """True while a session holds (or is handing back) the arms."""
        proc = self.proc
        return proc is not None and proc.poll() is None

    def _read(self, proc):
        for line in proc.stdout:
            line = line.strip()
            if line.startswith("ev "):
                self.last_event = line[3:]
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
               "event": self.last_event}
        return (200 if ok else code_err), out

    def take(self):
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
            threading.Thread(target=self._read, args=(self.proc,), name="joint-sender",
                             daemon=True).start()
            # Longer than an op: the sender first joins DDS and reads the robot once (~1.5 s).
            reply = self._send("take", self.reply_timeout + 3)
            if not reply.startswith("ok"):
                # Refused (robot not standing, no state): end the process, nothing was published.
                self._close_locked()
                return self._answer(reply)
            self.session = secrets.token_hex(8)
            code, out = self._answer(reply)
            out.update(session=self.session, allowed=JOINT_CTL_ALLOW,
                       excursion=JOINT_CTL_EXCURSION, margin=JOINT_CTL_MARGIN)
            return code, out

    def op(self, session, op, joint=None, q=None, side=None, finger=None, pos=None):
        """keepalive | status | target | hand | release, inside a session."""
        with self.lock:
            if not self.busy() or self.session is None:
                return 409, {"ok": False, "error": "no control session", **self.state,
                             "event": self.last_event}
            if not isinstance(session, str) or not hmac.compare_digest(session, self.session):
                return 409, {"ok": False, "error": "another page holds the arms"}
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
            elif op in ("keepalive", "status", "release"):
                line = op
            else:
                return 400, {"ok": False, "error": f"unknown op {op!r}"}
            return self._answer(self._send(line))

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
                "allowed": JOINT_CTL_ALLOW}


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
        src = self.client_address[0]
        if not self._authorised():
            audit(src, "joints-stream", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        joints = self.server.joints
        if joints is None:
            return self._json(404, {"error": f"no joint stream on a {ROBOT_MODEL}"})
        if not joints.subscribe():
            return self._json(503, {"error": "too many joint viewers",
                                    "max_clients": joints.max_clients})
        audit(src, "joints-stream", "-", "opened")
        try:
            self.send_response(200)
            self.send_header("Content-Type", "application/x-ndjson")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            seen = 0
            while True:
                seen, line = joints.next_line(seen, JOINT_HEARTBEAT_S)
                self.wfile.write(((line or '{"stale":true}') + "\n").encode())
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                        # the caller hung up: the normal way this ends
        finally:
            joints.unsubscribe()
            audit(src, "joints-stream", "-", "closed")

    def _joint_control(self):
        """POST /joints/control — move single arm joints of the G1 from the Joints page.

          {"op": "take"}                                   -> {ok, session, allowed, state, w}
          {"op": "target", "session", "joint", "q": rad}   -> {ok, reply, state, w}
          {"op": "hand", "session", "side", "finger": 0-5, "pos": 0-1000}
          {"op": "keepalive" | "status" | "release", "session"}

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
        if op == "take":
            code, out = ctl.take()
            audit(src, "joints-control", "take", out.get("reply") or out.get("error", "-"))
        else:
            code, out = ctl.op(body.get("session"), op, body.get("joint"), body.get("q"),
                               body.get("side"), body.get("finger"), body.get("pos"))
            if op in ("target", "hand", "release"):
                detail = (f"{body.get('joint')}={body.get('q')}" if op == "target" else
                          f"{body.get('side')}[{body.get('finger')}]={body.get('pos')}"
                          if op == "hand" else "release")
                audit(src, "joints-control", detail, out.get("reply") or out.get("error", "-"))
        return self._json(code, out)

    def do_GET(self):
        if self.path.split("?")[0].rstrip("/").startswith("/audio/"):
            return self._audio("GET")
        if self.path.split("?")[0].rstrip("/") == "/joints/stream":
            return self._joint_stream()
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
                         "joint_control": (self.server.joint_control.status()
                                           if self.server.joint_control
                                           else {"active": False, "available": False}),
                         "audio": {"available": self.server.audio is not None}})

    def _audio(self, method):
        """The Go2's speaker. All of it needs the token, like every other route that acts.

          POST /audio/say     body: a WAV        -> {ok, seconds}   (replies once uploaded)
          POST /audio/stop                       -> {ok}
          GET  /audio/volume                     -> {ok, volume}
          POST /audio/volume  {"level": 0-10}    -> {ok, volume}
          GET  /audio/listen?seconds=N           -> a 16 kHz mono WAV of the robot's mic (G1)
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
        if method == "GET" and path == "/audio/volume":
            line = "volume"
        elif method == "POST" and path == "/audio/volume":
            try:
                level = validate_volume(json.loads(body or b"{}").get("level"))
            except (ValueError, AttributeError) as exc:
                return self._json(400, {"ok": False, "error": str(exc)})
            line = f"volume {level}"
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

    def do_POST(self):
        if self.path.split("?")[0].rstrip("/").startswith("/audio/"):
            return self._audio("POST")
        if self.path.split("?")[0].rstrip("/") == "/video-config":
            return self._video_config()
        if self.path.split("?")[0].rstrip("/") == "/joints/control":
            return self._joint_control()
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
    srv.joint_control = (JointControl([JOINT_SENDER], srv.sender, srv.order)
                         if ROBOT_MODEL == "g1" else None)
    srv.joint_limiter = RateLimiter(JOINT_CTL_MAX_PER_SEC)
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
                             srv.joint_control)
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
        # EOF makes the audio sender leave megaphone mode before it exits.
        if srv.audio is not None and srv.audio.proc is not None:
            try:
                srv.audio.proc.stdin.close()
                srv.audio.proc.wait(timeout=5)
            except Exception:
                pass


if __name__ == "__main__":
    main()
