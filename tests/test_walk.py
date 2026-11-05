"""Walk to a point: POST /walk/control, GET /odom/stream, and the Walk loop behind them.

The robot is simulated: a fake odometry that integrates the last `move` the fake sender got
(body-frame velocities, like the G1's Move) and hands the pose out the way JointStream does. The
clock is the simulation's, so a 2 m walk takes milliseconds. Nothing touches DDS.

What must hold, beyond "it gets there": every way out of a walk stops the robot; the operator
wins (a stop, a drive-pad move, a cancel); no odometry or a refused move ends it as a failure;
and it never starts while the Joints page holds an arm.
"""
from __future__ import annotations

import http.client
import json
import math
import sys
import threading
import time
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server  # noqa: E402

TOKEN = "test-token"  # noqa: S105  # a test fixture, not a secret
DT = 0.05                       # the odometry bridge's 20 Hz


class SimRobot:
    """The sender AND the odometry of a robot that walks exactly as told, with a lag."""

    def __init__(self, x=1.0, y=-2.0, yaw=0.3, refuse_move=False, odom_dies_at=None):
        self.x, self.y, self.yaw = x, y, yaw
        self.t = 0.0
        self.v = (0.0, 0.0, 0.0)
        self.lines = []
        self.refuse_move = refuse_move
        self.odom_dies_at = odom_dies_at
        self.lock = threading.Lock()
        self.subscribers = 0
        self.seq = 0
        self.max_clients = 4

    # -- the sender ---------------------------------------------------- #
    def send(self, line):
        with self.lock:
            self.lines.append(line)
            parts = line.split()
            if parts[0] == "move":
                if self.refuse_move:
                    return "err move -1"
                self.v = tuple(float(p) for p in parts[1:])
            elif parts[0] == "stop_move":
                self.v = (0.0, 0.0, 0.0)
            return f"ok {parts[0]}"

    # -- the odometry (JointStream's interface) ------------------------- #
    def subscribe(self):
        self.subscribers += 1
        return True

    def unsubscribe(self):
        self.subscribers -= 1

    def clock(self):
        return self.t

    def next_line(self, seen, timeout):
        # A little real time per tick, so a test can still cancel a walk that is running (the
        # simulated clock alone would finish a 3 m walk before the test thread looks).
        time.sleep(0.002)
        with self.lock:
            if self.odom_dies_at is not None and self.t >= self.odom_dies_at:
                self.t += timeout
                return seen, None
            vx, vy, w = self.v
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (c * vx - s * vy) * DT
            self.y += (s * vx + c * vy) * DT
            self.yaw += w * DT
            self.t += DT
            self.seq += 1
            line = json.dumps({"t": self.t, "age_ms": 20, "x": self.x, "y": self.y,
                               "yaw": self.yaw})
            return self.seq, line


def walk_for(robot, joint_control=None):
    return relay_server.Walk(robot, robot, joint_control, clock=robot.clock, settle_s=0.5)


def wait_done(walk, timeout=10.0):
    end = time.time() + timeout
    while walk.running() and time.time() < end:
        time.sleep(0.01)
    assert not walk.running(), "the walk never ended"
    return walk.status()


def moves(robot):
    return [ln for ln in robot.lines if ln.startswith("move")]


# --------------------------------------------------------------------------- #
# The pure parts
# --------------------------------------------------------------------------- #
def test_validate_goto_bounds():
    assert relay_server.validate_goto({"dx": 2}) == (2.0, 0.0, 0.0)
    for bad, why in (({"dx": 5}, "at most"), ({"dyaw": 4}, "half a turn"),
                     ({}, "nothing to do"), ({"dx": "a"}, "numbers"),
                     ({"dx": float("nan")}, "finite")):
        with pytest.raises(ValueError, match=why):
            relay_server.validate_goto(bad)


def test_command_heads_at_the_goal_within_limits_and_holds_heading():
    goal = {"x": 2.0, "y": 0.0, "yaw": 0.0, "hold": 0.0}
    vx, vy, w, phase = relay_server.walk_command({"x": 0, "y": 0, "yaw": 0.1}, goal, "translate")
    assert phase == "translate"
    assert 0 < vx <= relay_server.WALK_VX and abs(vy) <= relay_server.WALK_VY
    assert w < 0                     # turned 0.1 left of the heading it started with: back right
    # Near the goal it slows, but not below the speed at which the robot still steps.
    vx, vy, w, _ = relay_server.walk_command({"x": 1.99 - 0.06, "y": 0, "yaw": 0}, goal,
                                              "translate")
    assert math.hypot(vx, vy) == pytest.approx(relay_server.WALK_MIN_V)


def test_command_turns_after_arriving_and_ignores_the_sway():
    goal = {"x": 0.0, "y": 0.0, "yaw": 1.0, "hold": 0.0}
    _vx, _vy, w, phase = relay_server.walk_command({"x": 0, "y": 0, "yaw": 0}, goal, "translate")
    assert phase == "turn" and w > 0
    # Turning, a 10 cm sway of the torso is not chased.
    vx, vy, w, phase = relay_server.walk_command({"x": 0.1, "y": 0, "yaw": 0.5}, goal, "turn")
    assert (vx, vy, phase) == (0.0, 0.0, "turn") and w > 0
    assert relay_server.walk_command({"x": 0.1, "y": 0, "yaw": 1.0}, goal, "turn")[3] == "done"


# --------------------------------------------------------------------------- #
# The loop, on the simulated robot
# --------------------------------------------------------------------------- #
def test_two_metres_forward_from_wherever_it_stands():
    robot = SimRobot(x=1.0, y=-2.0, yaw=0.3)
    walk = walk_for(robot)
    code, out = walk.start(2.0, 0.0, 0.0)
    assert code == 200 and out["ok"]
    st = wait_done(walk)
    assert st["state"] == "success", st
    # 2 m along ITS heading (0.3 rad), not along the odom frame's x.
    assert robot.x == pytest.approx(1.0 + 2 * math.cos(0.3), abs=0.08)
    assert robot.y == pytest.approx(-2.0 + 2 * math.sin(0.3), abs=0.08)
    assert robot.lines[-1] == "stop_move"
    assert all(abs(float(ln.split()[1])) <= relay_server.WALK_VX for ln in moves(robot))
    assert robot.subscribers == 0                 # the odometry is let go


def test_turn_in_place_a_quarter_left():
    robot = SimRobot(yaw=3.0)                     # across the +-pi seam
    walk = walk_for(robot)
    walk.start(0.0, 0.0, math.pi / 2)
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert relay_server.wrap_angle(robot.yaw - (3.0 + math.pi / 2)) == pytest.approx(0, abs=0.06)
    assert all(float(ln.split()[1]) == 0 for ln in moves(robot))   # no stepping, only turning


def test_cancel_stops_the_robot():
    robot = SimRobot()
    walk = walk_for(robot)
    walk.start(3.0, 0.0, 0.0)
    while not moves(robot):
        time.sleep(0.001)
    assert walk.cancel("test")
    st = wait_done(walk)
    assert st["state"] == "cancelled" and st["reason"] == "test"
    assert robot.lines[-1] == "stop_move"


def test_the_drive_pad_taking_over_is_not_stopped():
    robot = SimRobot()
    walk = walk_for(robot)
    walk.start(3.0, 0.0, 0.0)
    while not moves(robot):
        time.sleep(0.001)
    walk.cancel("move from the pad", stop=False)
    st = wait_done(walk)
    assert st["state"] == "cancelled"
    assert "stop_move" not in robot.lines         # the operator's next move is not undone


def test_odometry_lost_is_a_failure_and_a_stop():
    robot = SimRobot(odom_dies_at=0.5)
    walk = walk_for(robot)
    walk.start(2.0, 0.0, 0.0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "odometry" in st["reason"]
    assert robot.lines[-1] == "stop_move"


def test_a_refused_move_is_a_failure_that_names_why():
    robot = SimRobot(refuse_move=True)
    walk = walk_for(robot)
    walk.start(1.0, 0.0, 0.0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "err move -1" in st["reason"]
    assert robot.lines[-1] == "stop_move"


class BusyArms:
    def busy(self):
        return True


def test_refused_while_an_arm_is_held_and_one_at_a_time():
    robot = SimRobot()
    code, out = walk_for(robot, BusyArms()).start(1.0, 0, 0)
    assert code == 409 and "arms" in out["error"]
    walk = walk_for(robot)
    walk.start(3.0, 0, 0)
    code, out = walk.start(1.0, 0, 0)
    assert code == 409 and "already" in out["error"]
    walk.cancel("end")
    wait_done(walk)


# --------------------------------------------------------------------------- #
# Over HTTP
# --------------------------------------------------------------------------- #
class OpenLimiter:
    def allow(self):
        return True


@pytest.fixture
def server():
    robot = SimRobot()
    srv = ThreadingHTTPServer(("127.0.0.1", 0), relay_server.Handler)
    srv.token = TOKEN
    srv.sender = robot
    srv.limiter = OpenLimiter()
    srv.joint_limiter = OpenLimiter()
    srv.order = relay_server.CommandOrder()
    srv.joints = srv.udp = srv.audio = srv.joint_control = None
    srv.odom = robot
    srv.walk = walk_for(robot)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv, robot
    srv.walk.cancel("teardown")
    srv.shutdown()


def call(srv, path, body=None, token=TOKEN):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
    headers = {"Content-Type": "application/json"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    conn.request("POST", path, json.dumps(body or {}), headers)
    r = conn.getresponse()
    return r.status, json.loads(r.read() or b"{}")


def test_http_needs_the_token_and_checks_the_numbers(server):
    srv, _robot = server
    assert call(srv, "/walk/control", {"op": "status"}, token=None)[0] == 401
    code, out = call(srv, "/walk/control", {"op": "goto", "dx": 9})
    assert code == 400 and "at most" in out["error"]
    assert call(srv, "/walk/control", {"op": "dance"})[0] == 400


def test_http_goto_then_a_stop_from_the_pad_cancels_it(server):
    srv, robot = server
    code, out = call(srv, "/walk/control", {"op": "goto", "dx": 3})
    assert code == 200 and out["walk"]["state"] in ("starting", "running")
    while not moves(robot):
        time.sleep(0.001)
    assert call(srv, "/cmd", {"verb": "stop_move"})[0] == 200
    st = wait_done(srv.walk)
    assert st["state"] == "cancelled" and "stop_move" in st["reason"]
    assert call(srv, "/walk/control", {"op": "status"})[1]["walk"]["state"] == "cancelled"


def test_http_a_keepalive_does_not_cancel_a_walk(server):
    srv, robot = server
    call(srv, "/walk/control", {"op": "goto", "dx": 3})
    while not moves(robot):
        time.sleep(0.001)
    call(srv, "/cmd", {"verb": "keepalive"})
    assert srv.walk.running()


# --------------------------------------------------------------------------- #
# Its eyes: the lidar's obstacle map (g1_lidar_bridge), and the stop before hitting something
# --------------------------------------------------------------------------- #
class FakeLidar:
    """The bridge's newest line, made from obstacle points in the odometry frame."""

    def __init__(self, robot, obstacles=(), silent=False):
        self.robot = robot
        self.obstacles = list(obstacles)
        self.silent = silent
        self.seq = 0
        self.subscribers = 0

    def subscribe(self):
        self.subscribers += 1
        return True

    def unsubscribe(self):
        self.subscribers -= 1

    def next_line(self, seen, timeout):
        if self.silent:
            return seen, None
        self.seq += 1
        cells = [[round(x / 0.05), round(y / 0.05)] for x, y in self.obstacles]
        return self.seq, json.dumps({"t": 0, "age_ms": 40, "odom": 1, "res": 0.05,
                                     "pose": [self.robot.x, self.robot.y, self.robot.yaw],
                                     "cells": cells})


def seeing_walk(robot, lidar, wait=0.5):
    return relay_server.Walk(robot, robot, None, clock=robot.clock, settle_s=0.5, lidar=lidar,
                             require_lidar=True, blocked_wait_s=wait)


def test_obstacle_ahead_measures_along_the_way_only():
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    assert relay_server.obstacle_ahead(pose, 0.3, 0, [(0.4, 0.1)]) == pytest.approx(0.4)
    assert relay_server.obstacle_ahead(pose, 0.3, 0, [(0.4, 0.5)]) is None      # beside
    assert relay_server.obstacle_ahead(pose, 0.3, 0, [(-0.3, 0.0)]) is None     # behind
    assert relay_server.obstacle_ahead(pose, -0.3, 0, [(-0.3, 0.0)]) == pytest.approx(0.3)


def test_it_stops_before_a_box_in_the_way_and_gives_up():
    robot = SimRobot(x=0, y=0, yaw=0)
    box = [(1.2, dy) for dy in (-0.1, 0.0, 0.1)]
    walk = seeing_walk(robot, FakeLidar(robot, box))
    walk.start(2.0, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "blocked" in st["reason"]
    assert robot.x < 1.2 - 0.35                    # it stopped short, it did not touch it
    assert robot.lines[-1] == "stop_move"


def test_it_goes_on_when_the_way_clears():
    robot = SimRobot(x=0, y=0, yaw=0)
    lidar = FakeLidar(robot, [(1.2, 0.0)])
    walk = seeing_walk(robot, lidar, wait=30)
    walk.start(2.0, 0, 0)
    while not (walk.status().get("blocked")):
        time.sleep(0.001)
    lidar.obstacles = []                           # the person walked off
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert robot.x == pytest.approx(2.0, abs=0.08)


def test_something_beside_the_way_does_not_stop_it():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, [(1.0, 0.6), (1.5, -0.6)]))
    walk.start(2.0, 0, 0)
    assert wait_done(walk)["state"] == "success"


def test_no_eyes_no_walk():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, silent=True))
    walk.start(1.0, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "lidar" in st["reason"]
    assert moves(robot) == []                      # it never took a step


def test_the_page_copy_is_coarser():
    line = json.dumps({"age_ms": 1, "odom": 1, "res": 0.05, "cells": [[0, 0], [1, 1], [3, 0]]})
    d = json.loads(relay_server.coarsen_lidar(line))
    assert d["res"] == pytest.approx(0.1) and d["cells"] == [[0, 0], [1, 0]]
