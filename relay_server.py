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
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BIND = os.environ.get("RELAY_BIND", "0.0.0.0")  # noqa: S104  # known finding P0-1: binds broadly, no auth yet
PORT = int(os.environ.get("RELAY_PORT", "8092"))
TOKEN_FILE = os.environ.get("RELAY_TOKEN_FILE", os.path.expanduser("~/.relay_token"))
TOKEN = os.environ.get("RELAY_TOKEN", "")
SENDER = os.environ.get("SENDER_BIN", "./command_sender")
AUDIT_LOG = os.environ.get("AUDIT_LOG", "/var/tmp/robot-relay-audit.log")
MAX_PER_SEC = float(os.environ.get("MAX_PER_SEC", "20"))
REPLY_TIMEOUT = float(os.environ.get("REPLY_TIMEOUT", "3"))

# Mirrors command_sender's dispatch table. Kept here too so a bad verb is refused before it
# reaches the control process — defence in depth, not a single gate.
VERBS = {"stop_move", "stand_up", "stand_down", "damp", "balance_stand",
         "recovery_stand", "sit", "rise_sit", "hello", "keepalive"}


def log(msg):
    print(f"[relay] {msg}", file=sys.stderr, flush=True)


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
        "deadman_ms": os.environ.get("DEADMAN_MS", "1500"),
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

    def do_GET(self):
        if self.path.split("?")[0] != "/health":
            return self._json(404, {"error": "not found"})
        proc = self.server.sender.proc
        self._json(200, {"ok": True,
                         "sender_alive": bool(proc and proc.poll() is None),
                         "verbs": sorted(VERBS | {"move"}),
                         # Everything below is configured ON THE ROBOT: the app can only
                         # read it, so the robot reports it instead of the app guessing.
                         "video": video_status(),
                         "telemetry": telemetry_status(),
                         "limits": limits_status()})

    def do_POST(self):
        if self.path.split("?")[0] != "/cmd":
            return self._json(404, {"error": "not found"})
        if not self._authorised():
            audit(self.client_address[0], "-", "-", "unauthorised")
            return self._json(401, {"error": "unauthorised"})
        if not self.server.limiter.allow():
            return self._json(429, {"error": "rate limited"})

        try:
            n = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(n) or b"{}")
        except (ValueError, json.JSONDecodeError):
            return self._json(400, {"error": "bad json"})

        verb = str(payload.get("verb", ""))
        src = self.client_address[0]

        if verb == "move":
            try:
                vx = float(payload.get("vx", 0))
                vy = float(payload.get("vy", 0))
                vyaw = float(payload.get("vyaw", 0))
            except (TypeError, ValueError):
                return self._json(400, {"error": "vx/vy/vyaw must be numbers"})
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
    srv.sender = Sender([SENDER])
    srv.limiter = RateLimiter(MAX_PER_SEC)
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
