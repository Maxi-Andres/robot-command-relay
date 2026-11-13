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


# The mode verbs and the FSM each one lands in (g1_commands.py).
MODES = {"start": 500, "walk_waist": 501, "run": 801, "run_waist": 802}


class SimRobot:
    """The sender AND the odometry of a robot that walks exactly as told, with a lag."""

    def __init__(self, x=1.0, y=-2.0, yaw=0.3, refuse_move=False, odom_dies_at=None, fsm=501):
        self.x, self.y, self.yaw = x, y, yaw
        self.fsm = fsm
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
            elif parts[0] in MODES:
                self.fsm = MODES[parts[0]]
            return f"ok {parts[0]}"

    # -- the odometry (JointStream's interface) ------------------------- #
    def subscribe(self, internal=False):
        self.subscribers += 1
        return True

    def unsubscribe(self, internal=False):
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
                               "yaw": self.yaw, "fsm": self.fsm})
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
    assert relay_server.validate_goto({"dx": 8}) == (8.0, 0.0, 0.0)     # 3 m was the old cap
    for bad, why in (({"dx": relay_server.WALK_MAX_M + 1}, "at most"), ({"dyaw": 4}, "half a turn"),
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
    code, out = call(srv, "/walk/control", {"op": "goto", "dx": relay_server.WALK_MAX_M + 1})
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

    def subscribe(self, internal=False):
        self.subscribers += 1
        return True

    def unsubscribe(self, internal=False):
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


class Track:
    """Records how close the simulated robot ever got to some points, as it walks."""

    def __init__(self, robot, points):
        self.robot, self.points, self.closest, self.widest = robot, points, math.inf, 0.0
        inner = robot.next_line

        def next_line(seen, timeout):
            out = inner(seen, timeout)
            self.widest = max(self.widest, abs(robot.y))
            for x, y in self.points:
                self.closest = min(self.closest, math.hypot(robot.x - x, robot.y - y))
            return out
        robot.next_line = next_line


def box(cx, cy, half=0.2):
    """The outline of a square box, as the lidar's 5 cm cells would show it."""
    k = round(2 * half / 0.05)
    side = [-half + i * 0.05 for i in range(k + 1)]
    return ([(cx + d, cy - half) for d in side] + [(cx + d, cy + half) for d in side]
            + [(cx - half, cy + d) for d in side] + [(cx + half, cy + d) for d in side])


def test_it_goes_round_a_box_in_the_way():
    robot = SimRobot(x=0, y=0, yaw=0)
    obstacle = box(1.0, 0.0)
    track = Track(robot, obstacle)
    walk = seeing_walk(robot, FakeLidar(robot, obstacle))
    walk.start(2.0, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert robot.x == pytest.approx(2.0, abs=0.1) and robot.y == pytest.approx(0.0, abs=0.1)
    assert track.closest > 0.3                     # never brushed it
    assert relay_server.wrap_angle(robot.yaw) == pytest.approx(0.0, abs=0.07)   # faces as asked
    assert track.widest > 0.4                      # it left the straight line to go round


def test_a_wall_across_the_way_is_a_failure_without_a_step_into_it():
    robot = SimRobot(x=0, y=0, yaw=0)
    wall = [(1.0, -3.0 + k * 0.05) for k in range(121)]          # 6 m wide, past the margin
    walk = seeing_walk(robot, FakeLidar(robot, wall))
    walk.start(2.0, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "every way round" in st["reason"]
    assert robot.x < 1.0 - 0.3
    assert robot.lines[-1] == "stop_move"


def test_someone_stepping_in_front_is_never_walked_into():
    robot = SimRobot(x=0, y=0, yaw=0)
    lidar = FakeLidar(robot, [])
    walk = seeing_walk(robot, lidar, wait=30)
    walk.start(3.0, 0, 0)
    end = time.time() + 5
    while robot.x < 0.8 and time.time() < end:
        time.sleep(0.001)
    person = [(robot.x + 0.45, robot.y + dy) for dy in (-0.1, -0.05, 0.0, 0.05, 0.1)]
    track = Track(robot, person)
    lidar.obstacles = person                       # right in front, out of nowhere
    end = time.time() + 5
    while time.time() < end and walk.running() and track.closest > 0.6:
        time.sleep(0.01)
    time.sleep(0.2)
    closest = track.closest                        # while the person stood there
    track.points = []
    lidar.obstacles = []                           # and gone again
    st = wait_done(walk)
    assert closest > 0.25, closest
    assert st["state"] == "success", st


def test_the_planner_alone():
    straight = relay_server.plan_path((0, 0), (2, 0), [])
    assert straight == [(0, 0), (2, 0)]
    round_it = relay_server.plan_path((0, 0), (2, 0), box(1.0, 0.0))
    assert len(round_it) > 2
    assert all(min(math.hypot(px - x, py - y) for x, y in box(1.0, 0.0)) >= 0.3
               for px, py in round_it)
    with pytest.raises(relay_server.NoWay, match="cm from something"):
        relay_server.plan_path((0, 0), (1.0, 0.0), box(1.0, 0.0))                # goal inside


def test_it_walks_up_to_a_table_and_back_away_from_it():
    robot = SimRobot(x=0, y=0, yaw=0)
    table = [(1.0, -1.0 + k * 0.05) for k in range(41)]
    walk = seeing_walk(robot, FakeLidar(robot, table))
    walk.start(0.7, 0, 0)                          # to 30 cm from the edge, as for a grasp
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert robot.x == pytest.approx(0.7, abs=0.08)
    walk.start(-0.5, 0, 0)                         # and from there, back off
    assert wait_done(walk)["state"] == "success"


def test_it_may_end_closer_to_things_than_it_passes_them():
    table = [(1.0, -1.0 + k * 0.05) for k in range(41)]           # a table edge 1 m ahead
    path = relay_server.plan_path((0, 0), (0.7, 0.0), table)       # stand 30 cm from it
    assert path[-1] == (0.7, 0.0)
    with pytest.raises(relay_server.NoWay, match="20 cm from something"):
        relay_server.plan_path((0, 0), (0.8, 0.0), table)


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



# --------------------------------------------------------------------------- #
# The trail, recorded on the robot whoever moves it
# --------------------------------------------------------------------------- #
def test_the_trail_keeps_a_point_every_few_cm_and_restarts_on_a_new_frame():
    trail = relay_server.OdomTrail(None, step=0.03, cap=5)
    for k in range(10):
        trail.add([k * 0.01, 0.0, 0.0, k])           # 9 cm in 1 cm steps
    assert [p[0] for p in trail.snapshot()["points"]] == [0.0, 0.03, 0.06, 0.09]
    for k in range(10, 14):
        trail.add([k * 0.05, 0.0, 0.0, k])
    assert len(trail.snapshot()["points"]) == 5       # capped, the oldest went
    trail.add([50.0, 50.0, 0.0, 99])                  # a reboot: a new origin
    assert trail.snapshot()["points"] == [[50.0, 50.0, 0.0, 99]]
    trail.clear()
    assert len(trail.snapshot()["points"]) == 1 and trail.snapshot()["cleared_at"]


def test_the_trail_records_from_the_odometry_stream():
    robot = SimRobot(x=0, y=0, yaw=0)
    robot.v = (0.3, 0.0, 0.0)                          # someone drives it
    trail = relay_server.OdomTrail(robot).start()
    end = time.time() + 5
    while robot.x < 1.0 and time.time() < end:
        time.sleep(0.01)
    pts = trail.snapshot()["points"]
    assert len(pts) > 20 and pts[-1][0] > 0.9



# --------------------------------------------------------------------------- #
# Plan first, walk after the operator accepts
# --------------------------------------------------------------------------- #
def test_a_plan_shows_the_way_round_and_moves_nothing():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, box(1.0, 0.0)))
    code, out = walk.plan(2.0, 0, 0)
    assert code == 200 and out["ok"]
    path = out["plan"]["path"]
    assert len(path) > 2 and path[-1] == [2.0, 0.0]
    assert out["plan"]["goal"]["x"] == pytest.approx(2.0)
    assert robot.lines == []                       # not a single command


def test_a_plan_with_no_way_says_why():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, box(1.0, 0.0)))
    code, out = walk.plan(1.0, 0, 0)               # into the box
    assert code == 200 and not out["ok"] and "cm from something" in out["error"]
    assert robot.lines == []



def test_a_way_at_an_angle_with_nothing_in_it_is_one_straight_segment():
    # Seen on the robot: 1 m forward, 0.5 m right, nothing there -> a 1.2 m zigzag "round
    # something". It is one segment of 1.12 m.
    for goal in ((1.0, -0.5), (2.3, 0.7), (-0.4, 1.9)):
        assert relay_server.plan_path((0.0, 0.0), goal, []) == [(0.0, 0.0), goal]
    # ...and with something really in the way it still bends, every leg of it clear.
    path = relay_server.plan_path((0.0, 0.0), (2.0, -1.0), box(1.0, -0.5))
    assert len(path) > 2
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        for k in range(21):
            px, py = ax + (bx - ax) * k / 20, ay + (by - ay) * k / 20
            assert min(math.hypot(px - x, py - y) for x, y in box(1.0, -0.5)) > 0.3



def test_someone_standing_on_the_goal_for_a_moment_does_not_end_the_walk():
    robot = SimRobot(x=0, y=0, yaw=0)
    lidar = FakeLidar(robot, [])
    walk = seeing_walk(robot, lidar, wait=30)
    walk.start(2.0, 0, 0)
    end = time.time() + 5
    while robot.x < 0.5 and time.time() < end:
        time.sleep(0.001)
    lidar.obstacles = [(2.05, 0.0)]                # someone right on the goal
    end = time.time() + 5
    while not (walk.status().get("blocked") or {}).get("reason") and time.time() < end:
        time.sleep(0.005)
    assert "cm from something" in walk.status()["blocked"]["reason"]
    assert walk.running()
    lidar.obstacles = []                           # and they step away
    memory_s = relay_server.WALK_MEMORY_S
    st = wait_done(walk, timeout=10 + memory_s)
    assert st["state"] == "success", st


def test_a_goal_that_stays_blocked_ends_the_walk_saying_why():
    robot = SimRobot(x=0, y=0, yaw=0)
    lidar = FakeLidar(robot, [])
    walk = seeing_walk(robot, lidar, wait=0.5)
    walk.start(2.0, 0, 0)
    end = time.time() + 5
    while robot.x < 0.5 and time.time() < end:
        time.sleep(0.001)
    lidar.obstacles = [(2.05, 0.0)]
    st = wait_done(walk)
    assert st["state"] == "failure" and "cm from something" in st["reason"]
    assert robot.lines[-1] == "stop_move"



def test_it_slows_down_coming_up_to_something():
    robot = SimRobot(x=0, y=0, yaw=0)
    table = [(1.6, -1.0 + k * 0.05) for k in range(41)]
    walk = seeing_walk(robot, FakeLidar(robot, table))
    walk.start(1.3, 0, 0)                           # to 30 cm from a table edge
    st = wait_done(walk)
    assert st["state"] == "success", st
    speeds = [float(ln.split()[1]) for ln in moves(robot)]
    assert max(speeds) == pytest.approx(relay_server.WALK_VX)       # full speed when clear
    # Within the slow zone (the last stretch) it never ran at full speed.
    assert max(speeds[-5:]) < relay_server.WALK_VX * 0.8


def test_a_walk_to_the_side_turns_and_walks_forward_not_like_a_crab():
    # Seen on the robot: 0.7 m forward and 1.8 m right, done sideways at 0.15 m/s -> timed out.
    robot = SimRobot(x=0, y=0, yaw=0.4)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(0.7, -1.8, 0.0)
    st = wait_done(walk)
    assert st["state"] == "success", st
    steps = [tuple(float(v) for v in ln.split()[1:]) for ln in moves(robot)]
    walking = [s for s in steps if math.hypot(s[0], s[1]) > 0.05]
    assert walking and max(abs(vy) for _vx, vy, _w in walking) < 0.08      # forward, not sideways
    assert max(vx for vx, _vy, _w in walking) > 0.25                        # at walking speed
    # It ends where asked, facing as it started (dyaw 0): it turned back at the end.
    c, s = math.cos(0.4), math.sin(0.4)
    assert robot.x == pytest.approx(c * 0.7 + s * 1.8, abs=0.1)
    assert robot.y == pytest.approx(s * 0.7 - c * 1.8, abs=0.1)
    assert relay_server.wrap_angle(robot.yaw - 0.4) == pytest.approx(0, abs=0.07)


def test_half_a_metre_back_backs_up_without_turning_round():
    robot = SimRobot(x=0, y=0, yaw=1.0)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(-0.5, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert all(abs(float(ln.split()[3])) < 0.2 for ln in moves(robot))    # no turning round
    assert relay_server.wrap_angle(robot.yaw - 1.0) == pytest.approx(0, abs=0.07)



# --------------------------------------------------------------------------- #
# Speed and mode, chosen per walk
# --------------------------------------------------------------------------- #
def test_walk_options_bounds():
    assert relay_server.walk_options({}) == ((relay_server.WALK_VX, relay_server.WALK_VY,
                                              relay_server.WALK_VYAW), None)
    assert relay_server.walk_options({"speed": {"vx": 0.9, "vy": 0.4, "vyaw": 1.0},
                                      "mode": "run"}) == ((0.9, 0.4, 1.0), "run")
    for bad, why in (({"speed": {"vx": 2.0}}, "at most 1.2"), ({"speed": {"vx": 0}}, "over 0"),
                     ({"mode": "fly"}, "walk or run"), ({"speed": 3}, "vx, vy, vyaw")):
        with pytest.raises(ValueError, match=why):
            relay_server.walk_options(bad)


def test_a_faster_walk_goes_faster():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(2.5, 0, 0, lim=(0.9, 0.4, 1.0))
    assert wait_done(walk)["state"] == "success"
    assert max(float(ln.split()[1]) for ln in moves(robot)) == pytest.approx(0.9)


def test_faster_means_stopping_further_from_someone():
    robot = SimRobot(x=0, y=0, yaw=0)
    lidar = FakeLidar(robot, [])
    walk = seeing_walk(robot, lidar, wait=0.3)
    walk.start(3.0, 0, 0, lim=(1.2, 0.5, 1.2))
    end = time.time() + 5
    while robot.x < 0.3 and time.time() < end:
        time.sleep(0.001)
    wall = [(robot.x + 1.6, -2.0 + k * 0.05) for k in range(81)]   # 1.6 m ahead, all across
    lidar.obstacles = wall
    st = wait_done(walk)
    assert st["state"] == "failure"
    assert wall[0][0] - robot.x > 0.9                # at 1.2 m/s it kept ~1 m, not 0.5


def test_it_switches_to_run_before_walking():
    robot = SimRobot(x=0, y=0, yaw=0, fsm=501)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(1.0, 0, 0, mode="run")
    assert wait_done(walk)["state"] == "success"
    assert robot.lines[0] == "run_waist" and robot.fsm == 802     # waist free: 802


def test_it_never_stands_a_squatting_robot_up():
    robot = SimRobot(x=0, y=0, yaw=0, fsm=706)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(1.0, 0, 0, mode="walk")
    st = wait_done(walk)
    assert st["state"] == "failure" and "not walking or running" in st["reason"]
    assert [ln for ln in robot.lines if ln != "stop_move"] == []   # not a step, not a mode


def test_the_same_mode_sends_no_switch():
    robot = SimRobot(x=0, y=0, yaw=0, fsm=501)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(1.0, 0, 0, mode="walk")
    assert wait_done(walk)["state"] == "success"
    assert not any(ln.split()[0] in MODES for ln in robot.lines)



def test_auto_speed_is_fast_in_the_open_and_slow_near_things():
    assert relay_server.walk_options({"speed": "auto"}) == (None, None)
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(2.8, 0, 0, auto=True)
    assert wait_done(walk)["state"] == "success"
    open_top = max(float(ln.split()[1]) for ln in moves(robot))
    assert open_top == pytest.approx(relay_server.WALK_AUTO_CAP[0])

    robot = SimRobot(x=0, y=0, yaw=0)
    near = [(0.5 + k * 0.1, 0.45) for k in range(30)]    # a wall along the way, 45 cm to the side
    walk = seeing_walk(robot, FakeLidar(robot, near))
    walk.start(2.8, 0, 0, auto=True)
    assert wait_done(walk)["state"] == "success"
    assert max(float(ln.split()[1]) for ln in moves(robot)) < open_top * 0.6


# --------------------------------------------------------------------------- #
# 2026-11-11, on the robot: it rocked and never got anywhere
# --------------------------------------------------------------------------- #
class FlickerLidar(FakeLidar):
    """An obstacle in front that shows in one map out of every `every` (its own swinging hand,
    the floor while it sways), the rest empty."""

    def __init__(self, robot, spot, every=3):
        super().__init__(robot, [])
        self.spot, self.every = spot, every

    def next_line(self, seen, timeout):
        self.obstacles = [self.robot_rel(self.spot)] if (self.seq + 1) % self.every == 0 else []
        return super().next_line(seen, timeout)

    def robot_rel(self, spot):
        c, s = math.cos(self.robot.yaw), math.sin(self.robot.yaw)
        return (self.robot.x + c * spot[0] - s * spot[1], self.robot.y + s * spot[0] + c * spot[1])


def test_one_map_with_something_in_front_does_not_stop_it():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FlickerLidar(robot, (0.7, 0.0)))
    walk.start(1.5, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert "stop_move" not in robot.lines[:-1], "a one-map obstacle must not stop it"


def test_walked_past_the_point_is_arrival_not_a_step_back():
    goal = {"x": 1.0, "y": 0.0, "yaw": 0.0, "hold": 0.0}
    # Its last step took it 8 cm past (finer than WALK_POS_TOL is more than a G1 step).
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = walk_for(robot)
    walk.start(1.0, 0, 0)
    end = time.time() + 5
    while robot.x < 0.9 and time.time() < end:
        time.sleep(0.001)
    with robot.lock:
        robot.x = 1.08                               # the step overshoots
    st = wait_done(walk)
    backs = [ln for ln in moves(robot) if float(ln.split()[1]) < 0]
    assert not backs, backs
    assert st["state"] == "success", st              # within WALK_OK_POS
    assert goal                                      # (the goal the walk was given)


def test_it_sets_off_within_15_degrees_instead_of_turning_for_good():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = walk_for(robot)
    walk.start(2.0, 0.6, 0)                          # 17° left of where it faces
    st = wait_done(walk)
    assert st["state"] == "success", st


class DriftingRobot(SimRobot):
    """Turning in place, it drifts sideways, away from the way it turns, as the G1 did
    (2026-11-11): 15 cm/s, enough to swing the bearing to a point 0.3 m ahead faster than the
    turn closes it, so chasing that bearing never ends."""

    def next_line(self, seen, timeout):
        with self.lock:
            vx, vy, w = self.v
            if vx == 0 and vy == 0 and w != 0:
                side = -math.copysign(0.15, w)            # turning left, it slips right
                self.x += -math.sin(self.yaw) * side * DT
                self.y += math.cos(self.yaw) * side * DT
        return super().next_line(seen, timeout)


def test_turning_in_place_holds_the_heading_it_set_out_for_while_it_drifts():
    robot = DriftingRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, []))
    walk.start(0.5, 1.6, 0)                          # 73° left: it turns first
    faces = set()
    end = time.time() + 10
    while walk.running() and time.time() < end:
        f = walk.status().get("face")
        if f is not None:
            faces.add(f)
        time.sleep(0.0005)
    assert wait_done(walk)["state"] in ("success", "uncertain")
    assert len(faces) == 1, f"the heading it turned toward moved while it turned: {sorted(faces)}"


class StuckShortRobot(SimRobot):
    """Marks time instead of stepping below 0.12 m/s, as the G1 in Run did 19-20 cm short of the
    point (2026-11-11)."""

    def next_line(self, seen, timeout):
        with self.lock:
            if math.hypot(self.v[0], self.v[1]) < 0.12:
                self.v = (0.0, 0.0, self.v[2])
        return super().next_line(seen, timeout)


def test_a_walk_that_stops_getting_closer_ends_and_says_how_short():
    robot = StuckShortRobot(x=0, y=0, yaw=0)
    walk = walk_for(robot)
    walk.start(1.0, 0, 0)
    st = wait_done(walk)
    assert st["state"] in ("success", "uncertain"), st
    if st["state"] == "uncertain":
        assert "stopped getting closer" in st["reason"], st
    assert robot.lines[-1] == "stop_move"


class StuckTurningRobot(SimRobot):
    """Does not turn at all below 0.2 rad/s: the G1 stood 9 deg off its heading for good."""

    def next_line(self, seen, timeout):
        with self.lock:
            if abs(self.v[2]) < 0.2 and self.v[0] == self.v[1] == 0:
                self.v = (0.0, 0.0, 0.0)
        return super().next_line(seen, timeout)


@pytest.mark.parametrize("turn", [math.radians(45), -math.radians(45), math.radians(90)])
def test_a_turn_that_stops_turning_ends_soon_and_says_how_short(turn):
    """2026-11-13: a 45 deg turn waited out its 43 s limit 9 deg off, and the agent took the
    failure for "not turned" and turned it again."""
    robot = StuckTurningRobot(x=0, y=0, yaw=0)
    walk = walk_for(robot)
    walk.start(0.0, 0.0, turn)
    st = wait_done(walk)
    assert st["state"] in ("success", "uncertain"), st          # never "not there after 43 s"
    assert st["elapsed_s"] < 15, st
    if st["state"] == "uncertain":
        assert "stopped turning" in st["reason"] and "short of the heading" in st["reason"], st
    assert math.copysign(1, robot.yaw) == math.copysign(1, turn)  # it did turn, the right way
    assert robot.lines[-1] == "stop_move"


def test_run_mode_steps_at_its_own_minimum_speed():
    robot = SimRobot(x=0, y=0, yaw=0, fsm=801)
    walk = walk_for(robot)
    walk.start(1.0, 0, 0)
    assert wait_done(walk)["state"] == "success"
    speeds = [math.hypot(float(ln.split()[1]), float(ln.split()[2])) for ln in moves(robot)]
    assert min(s for s in speeds if s > 0) >= relay_server.WALK_MIN_V_RUN - 1e-3


def test_a_goal_against_a_wall_moves_to_where_it_can_stand():
    wall = [(1.0, -1.0 + k * 0.05) for k in range(41)]               # a wall 1 m ahead
    assert relay_server.free_goal((2.0, 0.0), wall) == (2.0, 0.0)     # clear already
    moved = relay_server.free_goal((0.9, 0.0), wall)                  # 10 cm from it
    assert moved is not None and 1.0 - moved[0] >= relay_server.WALK_GOAL_CLEARANCE - 1e-9
    assert relay_server.free_goal((1.0, 0.0), wall, within=0.1) is None   # too far to move
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = seeing_walk(robot, FakeLidar(robot, wall))
    walk.start(0.9, 0, 0)
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert st["goal_moved_cm"] >= 10 and robot.x <= 1.0 - 0.2


# --------------------------------------------------------------------------- #
# Nav2 does the walk (WALK_NAV2_URL, g1_nav's relay_bridge), 2026-11-11
# --------------------------------------------------------------------------- #
class FakeNav2:
    """Nav2 through relay_bridge, as the relay sees it: it takes a goal, and while it runs it
    drives the robot through the relay's own /walk/move (walk.nav2_move), like the bridge."""

    def __init__(self, robot, up=True, blocked_polls=0, gives_up=False):
        self.robot, self.up, self.walk = robot, up, None
        self.blocked_polls, self.gives_up = blocked_polls, gives_up
        self.goal, self.state, self.calls, self.refused = None, "idle", [], []

    def available(self):
        return self.up

    def goto(self, dx, dy, dyaw):
        self.calls.append(("goto", dx, dy, dyaw))
        r = self.robot
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        self.goal = (r.x + c * dx - s * dy, r.y + s * dx + c * dy)
        self.state = "running"
        return 200, {"ok": True}

    def cancel(self):
        self.calls.append(("cancel",))
        self.state = "canceled"
        return 200, {"ok": True}

    def plan(self, dx, dy, dyaw):
        self.calls.append(("plan", dx, dy, dyaw))
        return 200, {"ok": True, "start": {"x": 0, "y": 0, "yaw": 0}, "goal": {"x": dx, "y": dy},
                     "path": [[0, 0], [dx, dy]]}

    def drive(self, body):
        code, out = self.walk.nav2_move(body)
        if code != 200:
            self.refused.append((code, out))

    def status(self):
        if self.state == "running":
            if self.gives_up:
                self.state = "aborted"
            elif self.blocked_polls > 0:
                self.blocked_polls -= 1
                self.drive({"verb": "stop_move"})
                return 200, {"state": "running", "blocked": True, "path": [[0, 0], [1, 0]]}
            else:
                r = self.robot
                dx, dy = self.goal[0] - r.x, self.goal[1] - r.y
                if math.hypot(dx, dy) < 0.05:
                    self.drive({"verb": "stop_move"})
                    self.state = "succeeded"
                else:
                    c, s = math.cos(r.yaw), math.sin(r.yaw)
                    fx, fy = c * dx + s * dy, -s * dx + c * dy
                    k = 0.3 / math.hypot(fx, fy)
                    self.drive({"verb": "move", "vx": fx * k, "vy": fy * k, "vyaw": 0})
        return 200, {"state": self.state, "blocked": False, "path": [[0, 0], [1, 0]],
                     "reason": "the way stayed blocked" if self.state == "aborted" else None}


def nav2_walk(robot, nav2):
    walk = relay_server.Walk(robot, robot, None, clock=robot.clock, settle_s=0.3, nav2=nav2,
                             nav2_poll_s=0.0)
    nav2.walk = walk
    return walk


def test_a_goto_is_handed_to_nav2_which_moves_the_robot_through_the_relay():
    robot = SimRobot(x=0, y=0, yaw=0)
    nav2 = FakeNav2(robot, blocked_polls=3)
    walk = nav2_walk(robot, nav2)
    code, out = walk.start(1.0, 0.0, 0.0)
    assert code == 200 and out["walk"]["eyes"] == "nav2" and out["walk"]["speed"] == "nav2"
    st = wait_done(walk)
    assert st["state"] == "success", st
    assert nav2.calls[0] == ("goto", 1.0, 0.0, 0.0)
    assert robot.x == pytest.approx(1.0, abs=0.1)
    assert not nav2.refused, nav2.refused
    assert robot.lines[-1] == "stop_move"


def test_nav2_never_moves_the_robot_without_a_walk():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = nav2_walk(robot, FakeNav2(robot))
    code, out = walk.nav2_move({"verb": "move", "vx": 0.3, "vy": 0, "vyaw": 0})
    assert code == 409 and "only" in out["error"]
    assert not moves(robot)


def test_the_operator_cancels_a_nav2_walk_and_nav2_with_it():
    robot = SimRobot(x=0, y=0, yaw=0)
    nav2 = FakeNav2(robot, blocked_polls=10_000)      # stays waiting for a blocked way
    walk = nav2_walk(robot, nav2)
    walk.start(2.0, 0.0, 0.0)
    end = time.time() + 3
    while walk.status().get("blocked") is None and time.time() < end:
        time.sleep(0.01)
    assert walk.status()["blocked"]["reason"].startswith("something is in the way")
    walk.cancel("stop from the page")
    st = wait_done(walk)
    assert st["state"] == "cancelled" and ("cancel",) in nav2.calls
    assert walk.nav2_move({"verb": "move", "vx": 0.3})[0] == 409     # and it moves no more
    assert robot.lines[-1] == "stop_move"


def test_when_nav2_gives_up_the_walk_fails_saying_why():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = nav2_walk(robot, FakeNav2(robot, gives_up=True))
    walk.start(1.0, 0.0, 0.0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "Nav2 gave up" in st["reason"], st
    assert robot.lines[-1] == "stop_move"


def test_without_nav2_or_for_a_turn_in_place_the_relay_walks_itself():
    robot = SimRobot(x=0, y=0, yaw=0)
    down = FakeNav2(robot, up=False)
    walk = nav2_walk(robot, down)
    assert walk.start(1.0, 0.0, 0.0)[1]["walk"]["eyes"] != "nav2"
    assert wait_done(walk)["state"] == "success" and not down.calls
    up = FakeNav2(robot)
    walk = nav2_walk(robot, up)
    walk.start(0.0, 0.0, 1.0)                          # a turn in place: not Nav2's
    assert wait_done(walk)["state"] == "success" and not up.calls


def wall_at(x, y0=-1.0, y1=1.0):
    """Obstacle points of a wall across the way at x, every 5 cm."""
    return [(x, y0 + i * 0.05) for i in range(round((y1 - y0) / 0.05) + 1)]


def seeing_nav2_walk(robot, nav2, lidar):
    walk = relay_server.Walk(robot, robot, None, clock=robot.clock, settle_s=0.3, nav2=nav2,
                             nav2_poll_s=0.0, lidar=lidar)
    nav2.walk = walk
    return walk


def test_a_goal_on_a_wall_is_refused_before_nav2_sees_it():
    # 2026-11-12: a point marked on the wall; Nav2 stopped short and waited for ever.
    robot = SimRobot(x=0, y=0, yaw=0)
    nav2 = FakeNav2(robot)
    walk = seeing_nav2_walk(robot, nav2, FakeLidar(robot, wall_at(1.5)))
    code, out = walk.plan(1.5, 0.0, 0.0)
    assert code == 200 and not out["ok"] and "obstacle" in out["error"]
    assert not [c for c in nav2.calls if c[0] == "plan"]
    walk.start(1.5, 0.0, 0.0)
    st = wait_done(walk)
    assert st["state"] == "failure" and "obstacle" in st["reason"], st
    assert not [c for c in nav2.calls if c[0] == "goto"] and not moves(robot)


def test_a_goal_close_to_a_wall_is_moved_off_it_for_nav2():
    robot = SimRobot(x=0, y=0, yaw=0)
    nav2 = FakeNav2(robot)
    walk = seeing_nav2_walk(robot, nav2, FakeLidar(robot, wall_at(1.5)))
    code, out = walk.plan(1.3, 0.0, 0.0)                  # 20 cm from the wall
    assert code == 200 and out["ok"] and out["plan"]["goal_moved_cm"] == 15
    _op, dx, dy, _dyaw = nav2.calls[-1]
    assert dx == pytest.approx(1.15, abs=0.01) and dy == pytest.approx(0.0, abs=0.01)
    code, out = walk.plan(1.0, 0.0, 0.0)                  # room enough: as asked
    assert out["ok"] and "goal_moved_cm" not in out["plan"]
    assert nav2.calls[-1] == ("plan", 1.0, 0.0, 0.0)


def test_nav2_goal_room_turns_the_moved_goal_back_into_the_robots_frame():
    pose = {"x": 1.0, "y": 2.0, "yaw": math.pi / 2}       # facing +y
    dx, dy, cm = relay_server.nav2_goal_room(pose, 1.3, 0.0, [(x / 20, 3.5) for x in range(0, 41)])
    assert cm == 15 and dx == pytest.approx(1.15, abs=0.01) and dy == pytest.approx(0, abs=0.01)


def test_a_plan_comes_from_nav2_in_the_same_shape():
    robot = SimRobot(x=0, y=0, yaw=0)
    walk = nav2_walk(robot, FakeNav2(robot))
    code, out = walk.plan(1.0, 0.5, 0.0)
    assert code == 200 and out["ok"] and out["plan"]["eyes"] == "nav2"
    assert out["plan"]["path"] == [[0, 0], [1.0, 0.5]]


def test_http_walk_move_needs_the_token_and_a_nav2_walk(server):
    srv, robot = server
    assert call(srv, "/walk/move", {"verb": "move", "vx": 0.3}, token=None)[0] == 401
    code, out = call(srv, "/walk/move", {"verb": "move", "vx": 0.3})
    assert code == 409 and "only" in out["error"]
    assert not moves(robot)
