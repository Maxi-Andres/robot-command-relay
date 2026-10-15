"""Moving the G1's arm joints from the Joints page: POST /joints/control and JointControl.

This path MOVES the robot, so refusal comes first. The limits themselves are the sender's and
are tested in test_joint_ctl_core.py; what is tested here is the session around them:

* only one page holds the arms, and only with the session id `take` gave it;
* while it does, nothing makes the robot walk or play an arm action (HTTP /cmd and UDP);
* a stop from anywhere hands the arms back;
* a refused `take` leaves no process behind;
* the token gates every op, and a Go2 relay has no such route.

A fake sender (a Python script speaking the same line protocol) stands in for g1_joint_sender;
nothing touches DDS.
"""
from __future__ import annotations

import http.client
import json
import re
import sys
import textwrap
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server  # noqa: E402

TOKEN = "test-token"  # noqa: S105  # a test fixture, not a secret

# Speaks g1_joint_sender's protocol. Every line it receives is appended to LOG. FAKE_REFUSE in
# the environment makes it refuse `take` like a robot that is not standing.
FAKE_SENDER = textwrap.dedent("""
    import os, sys
    log = open(os.environ["FAKE_LOG"], "a")
    state, w = "idle", 0.0
    for line in sys.stdin:
        line = line.strip()
        log.write(line + "\\n"); log.flush()
        verb = line.split()[0]
        if verb == "take":
            if os.environ.get("FAKE_REFUSE"):
                print("err fsm 1 not allowed: stand in Walk state=idle w=0.00", flush=True)
                continue
            state = "up"
            print("ok take state=up w=0.00", flush=True)
            print("ev state active", flush=True)
            state, w = "active", 1.0
        elif verb == "target":
            _, j, q = line.split()
            print("ok target %s %s state=%s w=%.2f" % (j, q, state, w), flush=True)
        elif verb == "release":
            print("ok release state=down w=1.00", flush=True)
            print("ev released requested", flush=True)
            sys.exit(0)
        else:
            print("ok %s state=%s w=%.2f" % (verb, state, w), flush=True)
    log.write("EOF\\n")
""")


class FakeCmdSender:
    proc = None                      # /health asks whether the real one is alive

    def __init__(self):
        self.lines = []

    def send(self, line):
        self.lines.append(line)
        return "ok " + line.split()[0]


class OpenLimiter:
    def allow(self):
        return True


def _wait(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def fake(tmp_path, monkeypatch):
    script = tmp_path / "sender.py"
    script.write_text(FAKE_SENDER, encoding="utf-8")
    log = tmp_path / "sender.log"
    log.write_text("", encoding="utf-8")
    monkeypatch.setenv("FAKE_LOG", str(log))
    return [sys.executable, str(script)], log


@pytest.fixture
def ctl(fake):
    argv, _log = fake
    jc = relay_server.JointControl(argv, FakeCmdSender(), relay_server.CommandOrder(),
                                   reply_timeout=3)
    yield jc
    jc.stop()


# --------------------------------------------------------------------------- #
# The session
# --------------------------------------------------------------------------- #
def test_take_stops_the_robot_first_and_returns_a_session(ctl, fake):
    code, out = ctl.take()
    assert code == 200 and out["ok"]
    assert len(out["session"]) == 16
    assert out["allowed"] == relay_server.JOINT_CTL_ALLOW
    assert ctl.cmd_sender.lines == ["stop_move"], "a walk must not be running under a held arm"
    assert ctl.busy()


def test_one_page_at_a_time(ctl):
    _, first = ctl.take()
    code, out = ctl.take()
    assert code == 409 and not out["ok"]
    code, out = ctl.op("0123456789abcdef", "target", "left_elbow", 1.0)
    assert code == 409 and "another page" in out["error"]
    code, out = ctl.op(None, "keepalive")
    assert code == 409
    code, out = ctl.op(first["session"], "keepalive")
    assert code == 200 and out["state"] == "active"


def test_ops_need_an_open_session(ctl):
    code, out = ctl.op("whatever", "target", "left_elbow", 1.0)
    assert code == 409 and out["error"] == "no control session"


def test_targets_are_checked_and_forwarded(ctl, fake):
    _, log = fake
    s = ctl.take()[1]["session"]
    assert ctl.op(s, "target", "left_elbow", "x")[0] == 400
    assert ctl.op(s, "target", "left_elbow", float("nan"))[0] == 400
    assert ctl.op(s, "target", "left_elbow; damp", 1.0)[0] == 400, "no second command smuggled"
    assert ctl.op(s, "dance", "left_elbow", 1.0)[0] == 400
    code, out = ctl.op(s, "target", "left_elbow", 1.23456)
    assert code == 200 and out["reply"] == "target left_elbow 1.2346"
    assert "target left_elbow 1.2346" in log.read_text()


def test_release_ends_the_session_and_the_process(ctl):
    s = ctl.take()[1]["session"]
    code, _ = ctl.op(s, "release")
    assert code == 200
    assert _wait(lambda: not ctl.busy()), "the sender exits after its release ramp"
    assert ctl.status()["state"] == "off"
    assert ctl.take()[0] == 200, "and the arms can be taken again, by a new process"


def test_a_refused_take_leaves_nothing_running(ctl, fake, monkeypatch):
    monkeypatch.setenv("FAKE_REFUSE", "1")
    code, out = ctl.take()
    assert code == 409 and not out["ok"] and "not allowed" in out["reply"]
    assert "session" not in out
    assert not ctl.busy()


def test_stop_closes_stdin_so_the_sender_ramps_down(ctl, fake):
    _, log = fake
    ctl.take()
    ctl.stop()
    assert _wait(lambda: "EOF" in log.read_text())
    assert not ctl.busy()


# --------------------------------------------------------------------------- #
# No walking while an arm is held — HTTP and UDP
# --------------------------------------------------------------------------- #
@pytest.fixture
def server(fake):
    argv, _log = fake
    srv = ThreadingHTTPServer(("127.0.0.1", 0), relay_server.Handler)
    srv.token = TOKEN
    srv.sender = FakeCmdSender()
    srv.limiter = OpenLimiter()
    srv.joint_limiter = OpenLimiter()
    srv.order = relay_server.CommandOrder()
    srv.joints = None
    srv.udp = None
    srv.audio = None
    srv.joint_control = relay_server.JointControl(argv, srv.sender, srv.order, reply_timeout=3)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    if srv.joint_control is not None:
        srv.joint_control.stop()


def _post(srv, path, body, token=TOKEN):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=10)
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    conn.request("POST", path, json.dumps(body), headers)
    r = conn.getresponse()
    out = json.loads(r.read() or b"{}")
    conn.close()
    return r.status, out


def test_control_needs_the_token(server):
    code, _ = _post(server, "/joints/control", {"op": "take"}, token=None)
    assert code == 401
    assert not server.joint_control.busy()


def test_no_control_on_a_go2(server):
    server.joint_control.stop()
    server.joint_control = None
    code, _ = _post(server, "/joints/control", {"op": "take"})
    assert code == 404


def test_walking_and_arm_actions_are_refused_while_an_arm_is_held(server):
    code, out = _post(server, "/joints/control", {"op": "take"})
    assert code == 200
    session = out["session"]
    code, out = _post(server, "/joints/control",
                      {"op": "target", "session": session, "joint": "left_elbow", "q": 1.1})
    assert code == 200 and out["state"] == "active"
    for verb in ("move", "action_hug", "dance_spin_discs"):
        code, out = _post(server, "/cmd", {"verb": verb, "vx": 0.3})
        assert code == 409, verb
    assert server.sender.lines == ["stop_move"], "nothing but the take's own stop reached it"
    # A stop is always accepted, and hands the arm back.
    code, _ = _post(server, "/cmd", {"verb": "stop_move"})
    assert code == 200
    assert _wait(lambda: not server.joint_control.busy())
    code, _ = _post(server, "/cmd", {"verb": "move", "vx": 0.1})
    assert code == 200, "walking is allowed again once the arms are back"


def test_udp_moves_are_dropped_while_an_arm_is_held(server):
    key = b"test-key"
    udp = relay_server.UdpControl(key, server.sender, OpenLimiter(), server.order,
                                  server.joint_control)
    server.joint_control.take()

    def datagram(kind, ts):
        body = relay_server.UDP_CMD.pack(b"RC", 1, kind, ts, 0.3, 0.0, 0.0)
        return body + relay_server.udp_mac(key, body)

    now = time.time()
    assert udp.process(datagram(relay_server.UDP_KIND_MOVE, now + 1e-3), "x", now) is None
    assert udp.stats["arms_held"] == 1
    assert udp.process(datagram(relay_server.UDP_KIND_STOP, now + 2e-3), "x", now) is not None
    assert _wait(lambda: not server.joint_control.busy()), "a UDP stop releases the arm too"


def test_the_relay_joint_list_matches_the_sender():
    """The relay reports what the sender enforces: same names, same order, minus locked."""
    header = (REPO / "src" / "joint_ctl_core.hpp").read_text(encoding="utf-8")
    table = header[header.index("constexpr Joint JOINTS[N]"):]
    table = table[:table.index("};")]
    names = re.findall(r'\{"([a-z_]+)"', table)
    locked = re.findall(r'"([a-z_]+)"', header[header.index("LOCKED[]"):].split(";")[0])
    assert [n for n in names if n not in locked] == relay_server.ARM_SDK_JOINTS
    assert "waist_pitch" not in relay_server.JOINT_CTL_ALLOW


def test_take_reports_the_limits_the_page_draws(ctl):
    out = ctl.take()[1]
    assert out["excursion"] == relay_server.JOINT_CTL_EXCURSION
    assert out["margin"] == 0.05
    assert out["allowed"] == relay_server.ARM_SDK_JOINTS, "every arm joint by default"


def test_health_reports_the_control(server):
    conn = http.client.HTTPConnection("127.0.0.1", server.server_address[1], timeout=5)
    conn.request("GET", "/health")
    r = conn.getresponse()
    body = json.loads(r.read())
    conn.close()
    assert body["joint_control"]["active"] is False
    assert body["joint_control"]["allowed"] == relay_server.JOINT_CTL_ALLOW


# A sender that says it released and then hangs instead of exiting — what the real one did on
# 2026-10-14 (its exit waited on a stdio lock held by its own stdin reader).
HANGING_SENDER = textwrap.dedent("""
    import sys, time
    for line in sys.stdin:
        if line.startswith("take"):
            print("ok take state=up w=0.00", flush=True)
        elif line.startswith("release"):
            print("ok release state=down w=1.00", flush=True)
            print("ev released requested", flush=True)
            time.sleep(60)
        else:
            print("ok x state=active w=1.00", flush=True)
""")


def test_a_sender_that_does_not_exit_after_its_release_is_killed(tmp_path):
    script = tmp_path / "hang.py"
    script.write_text(HANGING_SENDER, encoding="utf-8")
    jc = relay_server.JointControl([sys.executable, str(script)], FakeCmdSender(),
                                   relay_server.CommandOrder(), reply_timeout=3)
    try:
        s = jc.take()[1]["session"]
        assert jc.op(s, "release")[0] == 200
        assert jc.busy()
        assert _wait(lambda: not jc.busy(), timeout=6), "otherwise every later take is refused"
        assert jc.take()[0] == 200
    finally:
        jc.stop()
