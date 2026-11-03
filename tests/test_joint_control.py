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
            if float(q) > 9:                     # a target the arm cannot reach: it stalls
                print("ev stall %s" % j, flush=True)
            print("ok target %s %s state=%s w=%.2f" % (j, q, state, w), flush=True)
        elif verb == "targets":
            pairs = line.split()[1:]
            for j, q in zip(pairs[::2], pairs[1::2]):
                if float(q) > 9:                 # unreachable: it stalls
                    print("ev stall %s" % j, flush=True)
            print("ok targets %d state=%s w=%.2f" % (len(pairs) // 2, state, w), flush=True)
        elif verb in ("grasp", "open"):
            side = line.split()[1]
            print("ok %s %s state=%s w=%.2f" % (verb, side, state, w), flush=True)
            if verb == "grasp":                  # the fingertips confirm at once
                print("ev grasp %s held 1.50 2.00 - 0.00 0.00" % side, flush=True)
        elif verb == "hand":
            _, side, finger, pos = line.split()
            print("ok hand %s %s %s state=%s w=%.2f" % (side, finger, pos, state, w), flush=True)
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


def test_finger_targets_are_checked_and_forwarded(ctl, fake):
    _, log = fake
    s = ctl.take()[1]["session"]
    assert ctl.op(s, "hand", side="middle", finger=1, pos=500)[0] == 400
    assert ctl.op(s, "hand", side="left", finger=6, pos=500)[0] == 400
    assert ctl.op(s, "hand", side="left", finger="2", pos=500)[0] == 400
    assert ctl.op(s, "hand", side="left", finger=True, pos=500)[0] == 400
    assert ctl.op(s, "hand", side="left", finger=2, pos=float("inf"))[0] == 400
    code, out = ctl.op(s, "hand", side="left", finger=2, pos=1200.4)
    assert code == 200 and out["reply"] == "hand left 2 1000"
    assert "hand left 2 1000" in log.read_text()
    assert ctl.op("someone-else", "hand", side="left", finger=2, pos=10)[0] == 409


def test_every_event_is_counted_so_a_repeated_stall_is_not_lost(ctl):
    """Every reply carries the LAST event, so two stalls of the same joint read the same text;
    only the count tells the page there were two (2026-10-22: the Joints page lists them)."""
    s = ctl.take()[1]["session"]
    _, out = ctl.op(s, "keepalive")
    assert out["event"] == "state active" and out["event_seq"] == 1
    _, first = ctl.op(s, "target", joint="left_elbow", q=9.9)
    _, second = ctl.op(s, "target", joint="left_elbow", q=9.9)
    assert first["event"] == second["event"] == "stall left_elbow"
    assert (first["event_seq"], second["event_seq"]) == (2, 3)
    _, out = ctl.op(s, "keepalive")
    assert out["event_seq"] == 3, "a keepalive repeats the last event without counting it again"
    assert ctl.status()["event_seq"] == 3
    ctl.op(s, "release")
    assert _wait(lambda: not ctl.busy())
    assert ctl.take()[1]["event_seq"] == 1, "a new session counts from the start"


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


# --------------------------------------------------------------------------- #
# Trajectories: the planner's arm motion (G1.md §6.8-6.9)
# --------------------------------------------------------------------------- #
class FakeStream:
    """The joint stream: the measured pose, as rt/lowstate's 29 angles in a JSON line."""

    def __init__(self, pose=None):
        self.latest = None
        self.subs = 0
        if pose:
            self.set(pose)

    def set(self, pose):
        q = [0.0] * 29
        for name, v in pose.items():
            q[relay_server.BODY_INDEX[name]] = v
        self.latest = json.dumps({"q": q})

    def subscribe(self):
        self.subs += 1
        return True

    def unsubscribe(self):
        self.subs -= 1


ALLOWED = set(relay_server.ARM_SDK_JOINTS)


def test_a_good_trajectory_is_accepted():
    plan = relay_server.validate_trajectory(
        [{"t": 1, "q": {"right_elbow": 0.4, "right_shoulder_pitch": 0.1}},
         {"t": 2, "q": {"right_elbow": 0.8}, "hands": {"right": [0, 0, 500, 500, 500, 2000]}}],
        ALLOWED, start={"right_elbow": 0.0, "right_shoulder_pitch": 0.0})
    assert plan[0] == (1.0, {"right_elbow": 0.4, "right_shoulder_pitch": 0.1}, {})
    assert plan[1][2] == {"right": [0, 0, 500, 500, 500, 1000]}, "fingers clamped to 0-1000"


@pytest.mark.parametrize("points, why", [
    ([], "non-empty"),
    ([{"t": 0, "q": {"right_elbow": 0.1}}], "after the previous"),
    ([{"t": 1, "q": {"right_elbow": 0.1}}, {"t": 1, "q": {"right_elbow": 0.2}}], "after"),
    ([{"t": 61, "q": {"right_elbow": 0.1}}], "at most 60"),
    ([{"t": 1, "q": {}}], "first point must name"),
    ([{"t": 1, "q": {"left_knee": 0.1}}], "not allowed: left_knee"),
    ([{"t": 1, "q": {"right_elbow": float("nan")}}], "number of radians"),
    ([{"t": 1, "q": {"right_elbow": 0.1}}, {"t": 2, "q": {"left_elbow": 0.1}}],
     "not in the first point"),
    ([{"t": 1, "q": {"right_elbow": 0.1}}, {"t": 2, "q": {"right_elbow": 0.9}}], "faster"),
    ([{"t": 1, "q": {"right_elbow": 0.1}, "hands": {"right": [1, 2]}}], "6 numbers"),
])
def test_a_bad_trajectory_is_refused(points, why):
    with pytest.raises(ValueError, match=why):
        relay_server.validate_trajectory(points, ALLOWED)


def test_too_fast_from_where_the_arm_is_is_refused():
    with pytest.raises(ValueError, match="faster"):
        relay_server.validate_trajectory([{"t": 0.5, "q": {"right_elbow": 1.0}}], ALLOWED,
                                         start={"right_elbow": 0.0})


def _traj_ctl(fake, stream, monkeypatch):
    monkeypatch.setattr(relay_server, "TRAJ_HZ", 50.0)
    monkeypatch.setattr(relay_server, "TRAJ_SETTLE_S", 0.3)
    argv, _log = fake
    return relay_server.JointControl(argv, FakeCmdSender(), relay_server.CommandOrder(),
                                     reply_timeout=3, stream=stream)


def _traj_state(jc, session):
    return jc.op(session, "status")[1]["trajectory"]["state"]


def test_a_trajectory_runs_and_arrives(fake, monkeypatch):
    stream = FakeStream({"right_elbow": 0.0})
    jc = _traj_ctl(fake, stream, monkeypatch)
    try:
        session = jc.take()[1]["session"]
        code, out = jc.op(session, "trajectory", points=[
            {"t": 0.2, "q": {"right_elbow": 0.05}},
            {"t": 0.4, "q": {"right_elbow": 0.1}, "hands": {"right": [0, 0, 900, 900, 900, 900]}}])
        assert code == 200 and out["trajectory"]["state"] == "running"
        assert jc.op(session, "target", "left_elbow", 0.3)[0] == 409, "no slider while it runs"
        stream.set({"right_elbow": 0.1})          # the arm got there
        assert _wait(lambda: _traj_state(jc, session) == "success")
        sent = fake[1].read_text().splitlines()
        moves = [ln for ln in sent if ln.startswith("targets right_elbow")]
        assert len(moves) >= 5 and moves[-1] == "targets right_elbow 0.1000"
        assert "hand right 2 900" in sent, "the point's fingers were sent"
        assert stream.subs == 0, "it stops watching the joint stream when it ends"
    finally:
        jc.stop()


def test_short_of_the_end_is_uncertain(fake, monkeypatch):
    jc = _traj_ctl(fake, FakeStream({"right_elbow": 0.0}), monkeypatch)
    try:
        session = jc.take()[1]["session"]
        jc.op(session, "trajectory", points=[{"t": 0.5, "q": {"right_elbow": 0.2}}])
        assert _wait(lambda: _traj_state(jc, session) == "uncertain")
        assert "0.200 rad from the end" in jc.op(session, "status")[1]["trajectory"]["reason"]
    finally:
        jc.stop()


def test_a_stall_fails_the_trajectory(fake, monkeypatch):
    jc = _traj_ctl(fake, FakeStream(), monkeypatch)
    try:
        session = jc.take()[1]["session"]
        jc.op(session, "trajectory", points=[{"t": 0.3, "q": {"right_elbow": 9.5}}])
        assert _wait(lambda: _traj_state(jc, session) == "failure")
        assert jc.op(session, "status")[1]["trajectory"]["reason"].startswith("stall")
    finally:
        jc.stop()


def test_cancel_freezes_the_arm_where_it_is(fake, monkeypatch):
    stream = FakeStream({"right_elbow": 0.0})
    jc = _traj_ctl(fake, stream, monkeypatch)
    try:
        session = jc.take()[1]["session"]
        jc.op(session, "trajectory", points=[{"t": 2.0, "q": {"right_elbow": 0.5}}])
        stream.set({"right_elbow": 0.123})
        time.sleep(0.1)
        jc.op(session, "cancel")
        assert _wait(lambda: _traj_state(jc, session) == "cancelled")
        moves = [ln for ln in fake[1].read_text().splitlines() if ln.startswith("targets")]
        assert moves[-1] == "targets right_elbow 0.1230"
    finally:
        jc.stop()


def test_trajectory_over_http(server, monkeypatch):
    monkeypatch.setattr(relay_server, "TRAJ_SETTLE_S", 0.2)
    session = _post(server, "/joints/control", {"op": "take"})[1]["session"]
    code, out = _post(server, "/joints/control", {"op": "trajectory", "session": session,
                                                   "points": [{"t": 0.2, "q": {"left_knee": 1}}]})
    assert code == 400 and "left_knee" in out["error"]
    code, out = _post(server, "/joints/control", {"op": "trajectory", "session": session,
                                                   "points": [{"t": 0.2, "q": {"left_elbow": 0.05}}]})
    assert code == 200 and out["trajectory"]["state"] == "running"


# --------------------------------------------------------------------------- #
# Grasp by touch (rule 12 of the sender)
# --------------------------------------------------------------------------- #
def test_grasp_reports_what_the_fingertips_said(ctl, fake):
    session = ctl.take()[1]["session"]
    assert ctl.op(session, "grasp", side="up")[0] == 400
    assert ctl.op(session, "grasp", side="right", pos=50)[0] == 400, "force over 10 N refused"
    assert ctl.op(session, "grasp", side="right", pos=1.5)[0] == 200
    assert _wait(lambda: (ctl.op(session, "status")[1]["grasp"]["right"] or {}).get("state")
                 == "held")
    assert ctl.op(session, "status")[1]["grasp"]["right"]["forces"] == [1.5, 2.0, None, 0.0, 0.0]
    assert "grasp right 1.50" in fake[1].read_text().splitlines()
    assert ctl.op(session, "open", side="right")[0] == 200
    assert "open right" in fake[1].read_text().splitlines()


def test_grasp_over_http(server):
    session = _post(server, "/joints/control", {"op": "take"})[1]["session"]
    code, out = _post(server, "/joints/control", {"op": "grasp", "session": session,
                                                   "side": "left", "force": 1.0})
    assert code == 200 and out["grasp"]["left"]["state"] in ("closing", "held")


def test_grasp_turns_the_thumb_first_when_asked(ctl, fake):
    session = ctl.take()[1]["session"]
    assert ctl.op(session, "grasp", side="right", thumb=1500)[0] == 400
    assert ctl.op(session, "grasp", side="right", thumb=900)[0] == 200
    assert "grasp right 1.00 900" in fake[1].read_text().splitlines(), "default force, then thumb"
    assert ctl.op(session, "grasp", side="left", pos=2, thumb=700)[0] == 200
    assert "grasp left 2.00 700" in fake[1].read_text().splitlines()


def test_grasp_with_the_thumb_as_a_post(ctl, fake):
    session = ctl.take()[1]["session"]
    assert ctl.op(session, "grasp", side="right", thumb=950, thumb_flex=30)[0] == 200
    assert "grasp right 1.00 950 30" in fake[1].read_text().splitlines()
    assert ctl.op(session, "grasp", side="right", thumb=950, thumb_flex=-5)[0] == 400


# --------------------------------------------------------------------------- #
# Retreat to the safe pose before letting go (2026-11-03)
# --------------------------------------------------------------------------- #
SAFE = {"right_shoulder_pitch": 0.57, "right_elbow": -0.04}
OUT = {"right_shoulder_pitch": 0.0, "right_elbow": 0.4}       # reached out over the table


class HotStream(FakeStream):
    """The joint stream with temperatures too (°C, rt/lowstate's 29)."""

    def __init__(self, pose, temp=40.0):
        self.temp = [temp] * 29
        super().__init__(pose)

    def set(self, pose):
        super().set(pose)
        d = json.loads(self.latest)
        d["temp"] = self.temp
        self.latest = json.dumps(d)

    def heat(self, joint, c):
        self.temp[relay_server.BODY_INDEX[joint]] = c
        self.set({k: json.loads(self.latest)["q"][i] for k, i in relay_server.BODY_INDEX.items()})


def _retreat_ctl(fake, stream, monkeypatch, lost_s=30.0):
    monkeypatch.setattr(relay_server, "JOINT_CTL_CLIENT_LOST_S", lost_s)
    monkeypatch.setattr(relay_server, "JOINT_CTL_MAX_VEL", 5.0)      # short retreats in tests
    return _traj_ctl(fake, stream, monkeypatch)


def _sent(fake):
    return fake[1].read_text().splitlines()


def test_a_warm_joint_brings_the_arm_back_to_the_safe_pose_before_letting_go(fake, monkeypatch):
    """The defect (G1, 2026-11-03): a hot shoulder released the arm mid-grasp; PC1 took it to
    ITS pose, straight down, and the hand hit the table. Under the sender's 75 °C, the relay
    moves the arm to the session's safe pose first, and only then releases."""
    stream = HotStream(OUT)
    jc = _retreat_ctl(fake, stream, monkeypatch)
    try:
        session = jc.take(SAFE)[1]["session"]
        stream.heat("right_shoulder_pitch", relay_server.JOINT_CTL_RETREAT_C + 1)
        assert _wait(lambda: (jc.retreat or {}).get("state") == "running")
        assert "warm right_shoulder_pitch" in jc.retreat["reason"]
        assert jc.op(session, "target", "right_elbow", 0.1)[0] == 409, "nothing else meanwhile"
        stream.set(SAFE)                                   # the arm got back
        assert _wait(lambda: not jc.busy(), timeout=5)
        lines = _sent(fake)
        assert any(ln.startswith("targets") and "right_shoulder_pitch 0.5700" in ln
                   for ln in lines)
        last_move = max(i for i, ln in enumerate(lines) if ln.startswith("targets"))
        assert lines.index("release") > last_move, "released only after the retreat"
    finally:
        jc.stop()


def test_a_client_that_goes_quiet_gets_the_arm_put_away(fake, monkeypatch):
    """The defect: the planner's PC dies with the hand over the table; the sender's dead-man
    would release it there 5 s later. The relay notices the silence first and retreats."""
    stream = HotStream(OUT)
    jc = _retreat_ctl(fake, stream, monkeypatch, lost_s=0.3)
    try:
        jc.take(SAFE)
        assert _wait(lambda: (jc.retreat or {}).get("reason") == "client lost")
        stream.set(SAFE)
        assert _wait(lambda: not jc.busy(), timeout=5)
        assert "release" in _sent(fake)
    finally:
        jc.stop()


def test_a_release_asked_away_from_the_safe_pose_goes_there_first(fake, monkeypatch):
    stream = HotStream(OUT)
    jc = _retreat_ctl(fake, stream, monkeypatch)
    try:
        session = jc.take(SAFE)[1]["session"]
        code, out = jc.op(session, "release")
        assert code == 200 and out["retreat"]["reason"] == "release asked"
        assert "release" not in _sent(fake), "not yet: the arm is still out"
        stream.set(SAFE)
        assert _wait(lambda: not jc.busy(), timeout=5)
    finally:
        jc.stop()


def test_a_release_at_the_safe_pose_is_immediate(fake, monkeypatch):
    stream = HotStream(SAFE)
    jc = _retreat_ctl(fake, stream, monkeypatch)
    try:
        session = jc.take(SAFE)[1]["session"]
        assert jc.op(session, "release")[0] == 200
        assert _wait(lambda: not jc.busy())
        assert jc.retreat is None
    finally:
        jc.stop()


def test_without_a_safe_pose_nothing_changes(fake, monkeypatch):
    """The Joints page names no safe pose: a warm joint is the sender's business (rule 9)."""
    stream = HotStream(OUT, temp=72.0)
    jc = _retreat_ctl(fake, stream, monkeypatch, lost_s=0.2)
    try:
        session = jc.take()[1]["session"]
        time.sleep(0.6)
        assert jc.retreat is None and jc.busy()
        assert jc.op(session, "release")[0] == 200
    finally:
        jc.stop()


@pytest.mark.parametrize("safe, why", [
    ({"left_knee": 0.1}, "not allowed"),
    ({"right_elbow": 9.0}, "radians"),
    ({}, "non-empty"),
    ([0.1], "non-empty"),
])
def test_a_bad_safe_pose_is_refused_before_anything_starts(ctl, fake, safe, why):
    code, out = ctl.take(safe)
    assert code == 400 and why in out["error"]
    assert not ctl.busy()
