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
import socket
import struct
import subprocess
import sys
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

# UDP port for continuous teleop (`move` and `stop_move` only). 0 = off, the default: the
# HTTP path below is then the only way in, exactly as before. See the UDP block further down.
_udp = os.environ.get("RELAY_UDP_PORT", "0").strip()
RELAY_UDP_PORT = int(_udp) if _udp.isascii() and _udp.isdigit() and 1024 <= int(_udp) <= 65535 \
    else 0

# Mirrors each sender's dispatch table (src/<robot>_command_sender.cpp). Kept here too so a bad
# verb is refused before it reaches the control process — defence in depth, not a single gate.
# tests/test_relay_boundary.py reads both and fails if they drift.
VERBS_BY_MODEL = {
    "go2": {"stop_move", "stand_up", "stand_down", "damp", "balance_stand",
            "recovery_stand", "sit", "rise_sit", "hello", "stretch", "scrape", "heart",
            "pose_on", "pose_off", "keepalive"},
    # Narrower on purpose: the G1 falls. Why each verb is in or out: g1_command_sender.cpp.
    # Lists BOTH walks; verbs_for() keeps the one matching the waist lock.
    "g1": {"stop_move", "stand_up", "walk_waist", "start", "squat", "lie_up", "balance_stand",
           "high_stand", "low_stand", "wave_hand", "keepalive"},
}


def verbs_for(model, waist_locked):
    """The verbs this relay accepts. On the G1 only ONE walk: 500 (`start`) with the waist
    locked, 501 (`walk_waist`) with it free — the lock is an app setting this process cannot
    read, so relay.env declares it (G1_WAIST_LOCK=1). g1_command_sender drops the same one."""
    verbs = set(VERBS_BY_MODEL[model])
    if model == "g1":
        verbs.discard("walk_waist" if waist_locked else "start")
    return verbs


G1_WAIST_LOCK = os.environ.get("G1_WAIST_LOCK", "").strip() == "1"
VERBS = verbs_for(ROBOT_MODEL, G1_WAIST_LOCK)


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
    "bitrate": ("BITRATE", int, 200000, 8000000, False),
    "maxfps":  ("MAXFPS", int, 0, 30, False),
    "idr":     ("IDR_FRAMES", int, 1, 300, False),
    "nvr":     ("NVR_ENABLE", int, 0, 1, False),
}
LIVE_KEYS = {k for k, v in VIDEO_PARAMS.items() if v[4]}


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
        return {"fps": d.get("fps_cap") or 0, "width": d.get("width"),
                "quality": d.get("quality")}
    except Exception:
        return {}


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

    def __init__(self, key, sender, limiter, order):
        self.key = key
        self.sender = sender
        self.limiter = limiter
        self.order = order
        self.stats = {"accepted": 0, "size": 0, "mac": 0, "format": 0, "clock": 0,
                      "stale": 0, "limited": 0}

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
            line, verb, detail = "stop_move", "stop_move", "-"
        else:
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
    """Owns the long-lived command_sender child. One DDS participant, created once."""

    def __init__(self, argv):
        self.argv = argv
        self.lock = threading.Lock()
        self.proc = None
        self._spawn()

    def _spawn(self):
        self.proc = subprocess.Popen(
            self.argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        log(f"command_sender started (pid {self.proc.pid})")

    def send(self, line):
        """Write one command, read its reply. Serialised: the protocol is one-in-one-out."""
        with self.lock:
            if self.proc is None or self.proc.poll() is not None:
                log("command_sender is gone — respawning")
                self._spawn()
            try:
                self.proc.stdin.write(line + "\n")
                self.proc.stdin.flush()
            except (BrokenPipeError, ValueError) as exc:
                return f"err sender-write {exc}"

            deadline = time.time() + REPLY_TIMEOUT
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
        """POST /video-config {fps?, width?, quality?, persist?}

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
        try:
            env_updates = validate_video(body)
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

    def do_GET(self):
        if self.path.split("?")[0].rstrip("/") == "/video-config":
            if not self._authorised():
                return self._json(401, {"error": "unauthorised"})
            # RUNNING and SAVED are different things and the UI must be able to show both:
            # editing video.env without restarting is exactly how this report started
            # lying before, which is why /health reads /proc instead of the file.
            saved = read_env_file(VIDEO_ENV)
            return self._json(200, {
                "ok": True,
                "running": mjpeg_live(),
                "saved": {k: saved.get(name) for k, (name, *_) in VIDEO_PARAMS.items()},
                # `live` tells the UI which knobs apply instantly and which cost a restart
                # of the video service — the difference the operator must see BEFORE
                # touching one while driving.
                "limits": {k: {"min": lo, "max": hi, "live": live}
                           for k, (_n, _c, lo, hi, live) in VIDEO_PARAMS.items()},
            })
        if self.path.split("?")[0] != "/health":
            return self._json(404, {"error": "not found"})
        proc = self.server.sender.proc
        self._json(200, {"ok": True,
                         "sender_alive": bool(proc and proc.poll() is None),
                         "robot_model": ROBOT_MODEL,
                         "verbs": sorted(VERBS | {"move"}),
                         # Everything below is configured ON THE ROBOT: the app can only
                         # read it, so the robot reports it instead of the app guessing.
                         "video": video_status(),
                         "battery": battery_status(),
                         "telemetry": telemetry_status(),
                         "limits": limits_status(),
                         "udp": ({"port": RELAY_UDP_PORT, **self.server.udp.stats}
                                 if self.server.udp else {"port": 0})})

    def do_POST(self):
        if self.path.split("?")[0].rstrip("/") == "/video-config":
            return self._video_config()
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
        elif verb in VERBS:
            line, detail = verb, "-"
        else:
            audit(src, verb or "-", "-", "rejected-unknown-verb")
            return self._json(400, {"error": f"unknown verb '{verb}'",
                                    "allowed": sorted(VERBS | {"move"})})

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
    srv.udp = None
    if RELAY_UDP_PORT:
        srv.udp = UdpControl(token.encode(), srv.sender, srv.limiter, srv.order)
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
        # Closing the child's stdin makes command_sender StopMove before it exits, so
        # shutting the relay down can never leave the robot walking.
        try:
            srv.sender.proc.stdin.close()
            srv.sender.proc.wait(timeout=5)
        except Exception:
            pass


if __name__ == "__main__":
    main()
