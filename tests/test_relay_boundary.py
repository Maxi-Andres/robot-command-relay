"""The relay's security boundary: what it accepts, how fast, and whether its allowlist still
agrees with the one in C++.

This service is the only remote path that can move the robot, so the tests here are about
refusal, not capability. Nothing touches DDS or spawns `command_sender`.
"""
# Lazy annotations: this repo targets Python 3.8 (the robot's Jetson), where `set[str]`
# in an evaluated annotation is a TypeError. CI on 3.8 caught it; ruff's FA rule now does.
from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server  # noqa: E402

# --------------------------------------------------------------------------- #
# The allowlist, and the copy of it that lives in C++
# --------------------------------------------------------------------------- #
MODELS = sorted(relay_server.VERBS_BY_MODEL)


def _sender_verbs(model: str = "go2") -> set[str]:
    """The verbs `<model>_command_sender.cpp` actually dispatches, read out of its VERBS map.

    Parsing the C++ is deliberate. The two lists are duplicated ON PURPOSE — defence in
    depth, and the boundary forbids sharing code with a binary — so the only way to keep them
    honest is a test that reads both.
    """
    src = (REPO / "src" / f"{model}_command_sender.cpp").read_text(encoding="utf-8")
    table = re.search(r"VERBS\s*=\s*\{(.*?)\n\s*\};", src, re.S)
    assert table, f"could not find the VERBS dispatch table in {model}_command_sender.cpp"
    verbs = set(re.findall(r'\{\s*"(\w+)"', table.group(1)))
    # `move` and `keepalive` are handled before the table (they take arguments / refresh the
    # dead-man rather than dispatching a client call), so they are added here explicitly.
    return verbs | {"move", "keepalive"}


@pytest.mark.parametrize("model", MODELS)
def test_python_and_cpp_allowlists_agree(model):
    """Catches: a verb added to one side only.

    Added to C++ alone, it is unreachable — confusing but safe. Added to Python alone, the
    relay accepts a command that `command_sender` then refuses, which surfaces as a useless
    502 mid-drive. Either way the two must not drift.
    """
    python_verbs = relay_server.VERBS_BY_MODEL[model] | {"move"}
    assert python_verbs == _sender_verbs(model)


@pytest.mark.parametrize("model", MODELS)
def test_acrobatics_are_absent_from_the_allowlist(model):
    """Catches: someone adding a flip or a handstand to the remote path.

    These exist in the robot's SDK and are deliberately unreachable from the network: the
    relay cannot be the way a flip gets triggered from another continent.
    """
    for verb in ["front_flip", "back_flip", "handstand", "walk_upright", "dance1", "front_jump"]:
        assert verb not in relay_server.VERBS_BY_MODEL[model]
        assert verb not in _sender_verbs(model)


@pytest.mark.parametrize("model", MODELS)
def test_stop_is_always_in_the_allowlist(model):
    """Catches: a refactor that removes the one verb you cannot afford to lose."""
    assert "stop_move" in relay_server.VERBS_BY_MODEL[model]
    assert "stop_move" in _sender_verbs(model)


def test_the_g1_cannot_be_dropped_from_the_network():
    """Catches: someone adding the verbs that make a STANDING humanoid fall.

    zero_torque and damp go limp; user_ctrl hands the joints to a controller the relay is not;
    start (FSM 500) and the SDK's squat (FSM 2) are the wrong ids on THIS robot, the second
    observed half-falling. Any of them, sent to a G1 on its feet, can drop it.
    See g1_command_sender.cpp.
    """
    for verb in ["zero_torque", "damp", "start", "squat_sdk", "user_ctrl",
                 "switch_to_user_ctrl", "shake_hand", "set_fsm_id", "run", "climb"]:
        assert verb not in relay_server.VERBS_BY_MODEL["g1"]
        assert verb not in _sender_verbs("g1")


@pytest.mark.parametrize("sender,model,ok", [
    ("/home/unitree/robot-command-relay/go2_command_sender", "go2", True),
    ("./g1_command_sender", "g1", True),
    ("/home/unitree/robot-command-relay/command_sender", "go2", False),   # the stale binary
    ("./go2_command_sender", "g1", False),                                # wrong robot
])
def test_the_sender_binary_must_name_the_robot(sender, model, ok):
    """Catches: a relay starting on yesterday's binary, or on the other robot's."""
    assert (relay_server.sender_mismatch(sender, model) == "") is ok


# --------------------------------------------------------------------------- #
# Rate limiting
# --------------------------------------------------------------------------- #
def test_rate_limiter_allows_the_budget_then_refuses():
    """Catches: a stuck or malicious caller flooding the control bus.

    Asserts the invariant (exactly N allowed, the next refused), never wall-clock timing.
    """
    limiter = relay_server.RateLimiter(per_sec=5)
    assert sum(limiter.allow() for _ in range(5)) == 5
    assert limiter.allow() is False


def test_rate_limiter_recovers_in_the_next_window():
    """Catches: a limiter that latches closed and never lets a stop through again."""
    limiter = relay_server.RateLimiter(per_sec=2)
    assert limiter.allow() and limiter.allow()
    assert limiter.allow() is False
    limiter.window -= 1.5   # advance past the window instead of sleeping
    assert limiter.allow() is True


@pytest.mark.xfail(
    strict=True,
    reason="fixed windows let 2x the budget through across a boundary. Switch to a sliding "
    "window or a token bucket, then delete this marker.",
)
def test_rate_limiter_does_not_allow_double_the_budget_across_a_boundary():
    """Catches: the burst a fixed window permits — spend the whole budget at the end of one
    window and the whole budget again immediately after it rolls."""
    limiter = relay_server.RateLimiter(per_sec=5)
    for _ in range(5):
        limiter.allow()
    limiter.window -= 1.0            # the window rolls
    allowed_right_after = sum(limiter.allow() for _ in range(5))
    assert allowed_right_after < 5


# --------------------------------------------------------------------------- #
# Token comparison
# --------------------------------------------------------------------------- #
def test_token_check_rejects_wrong_missing_and_malformed():
    """Catches: an auth check that accepts a prefix, an empty token, or a bare token with no
    `Bearer` scheme."""

    class FakeServer:
        token = "the-real-token"  # noqa: S105  # test fixture, not a credential

    handler = relay_server.Handler.__new__(relay_server.Handler)
    handler.server = FakeServer()

    for header in [
        "",
        "the-real-token",              # no scheme
        "Bearer ",
        "Bearer wrong",
        "Bearer the-real",             # prefix of the real one
        "Bearer the-real-token-extra",
        "bearer the-real-token",       # scheme is case-sensitive here
    ]:
        handler.headers = {"Authorization": header}
        assert handler._authorised() is False, f"{header!r} was accepted"

    handler.headers = {"Authorization": "Bearer the-real-token"}
    assert handler._authorised() is True


@pytest.mark.xfail(
    strict=True,
    reason="the token is compared with `==`, which short-circuits on the first differing "
    "byte. Use hmac.compare_digest, then delete this marker.",
)
def test_token_comparison_is_constant_time():
    """Catches: a timing side channel on the token check. Asserts the implementation, not a
    measurement — timing measurements in a test suite are flaky by nature."""
    import inspect
    src = inspect.getsource(relay_server.Handler._authorised)
    assert "compare_digest" in src


# --------------------------------------------------------------------------- #
# Video tuning: the allowlist and the env writer
#
# This route writes a file on the robot and retunes the video the operator steers by, so
# the tests are again about REFUSAL. A value that gets through and ruins the stream on a
# robot in the field can only be undone over SSH — the exact trip the feature removes.
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("body, expect", [
    ({"fps": 0}, {"MJPEG_FPS": 0.0}),
    ({"fps": 60}, {"MJPEG_FPS": 60.0}),
    ({"width": 0}, {"MJPEG_WIDTH": 0}),
    ({"width": 1920}, {"MJPEG_WIDTH": 1920}),
    ({"quality": 1}, {"MJPEG_QUALITY": 1}),
    ({"quality": 100}, {"MJPEG_QUALITY": 100}),
    ({"fps": 5, "width": 640, "quality": 55},
     {"MJPEG_FPS": 5.0, "MJPEG_WIDTH": 640, "MJPEG_QUALITY": 55}),
])
def test_valid_video_params_map_to_env_keys(body, expect):
    assert relay_server.validate_video(body) == expect


@pytest.mark.parametrize("body", [
    {"fps": -1}, {"fps": 61}, {"fps": "fast"},
    {"width": -1}, {"width": 4096},
    {"quality": 0}, {"quality": 101},
    {"bitrate": 100},          # 100 bps: the floor exists because 60 kbps was live on the robot
    {"bitrate": 99000000},     # and the ceiling, for the same class of typo
    {"nvr": 2},                # a flag is 0 or 1
    {"idr": 0},                # a keyframe interval of zero frames is meaningless
    {"NIC": "eth9"},           # would take the video off the air; must never reach the file
    {"PUBLISH_HOST": "evil"},  # nor may an arbitrary env key be smuggled through
    {},                        # an empty request is a mistake, not a no-op
])
def test_bad_video_params_are_refused(body):
    with pytest.raises(ValueError):
        relay_server.validate_video(body)


def test_the_allowlist_agrees_with_the_publisher_on_the_same_machine():
    """The defect: the relay and mjpeg_server drifting apart, so the relay accepts a value
    the publisher then rejects — or worse, silently clamps.

    They are separate repos on one machine and the network boundary forbids a shared
    module, so this comparison is the only thing keeping the two copies honest. It is the
    same reason the MJPEG parser is tested on both sides.
    """
    publisher = (Path.home() / "Desktop/robot-ecosystem/robot-video-pipeline/robot"
                 / "mjpeg_server.py")
    if not publisher.exists():
        pytest.skip("the video-pipeline repo is not checked out next to this one")
    src = publisher.read_text(encoding="utf-8")
    for key, (_env, cast, lo, hi, live) in relay_server.VIDEO_PARAMS.items():
        if not live:
            continue   # restart-only knobs are the relay's alone; the publisher never sees them
        pattern = rf'"{key}":\s*\("[A-Z_]+",\s*{cast.__name__},\s*([0-9.]+),\s*([0-9.]+)\)'
        m = re.search(pattern, src)
        assert m, f"'{key}' is missing from mjpeg_server.LIVE_PARAMS (or changed shape)"
        assert (float(m.group(1)), float(m.group(2))) == (float(lo), float(hi)), (
            f"'{key}' range differs: relay {lo}-{hi}, publisher {m.group(1)}-{m.group(2)}")


def test_writing_env_keys_preserves_everything_else(tmp_path):
    """The defect: rewriting video.env from scratch and losing PUBLISH_HOST, the comments,
    or the operator's other settings. run-video.sh REFUSES to start without PUBLISH_HOST,
    so that mistake takes the video off the air until someone SSHes in."""
    f = tmp_path / "video.env"
    f.write_text("# a comment\nPUBLISH_HOST=192.168.20.99\nMJPEG_FPS=0\nBITRATE=600000\n")
    relay_server.write_env_keys(str(f), {"MJPEG_FPS": 15, "MJPEG_WIDTH": 640})
    out = f.read_text().splitlines()
    assert "# a comment" in out, "comments were dropped"
    assert "PUBLISH_HOST=192.168.20.99" in out, "an unrelated key was lost"
    assert "BITRATE=600000" in out
    assert "MJPEG_FPS=15" in out, "the key was not replaced in place"
    assert "MJPEG_WIDTH=640" in out, "a new key was not appended"
    assert sum(1 for line in out if line.startswith("MJPEG_FPS=")) == 1, "duplicated key"


def test_writing_env_keys_is_atomic(tmp_path):
    """The defect: a half-written video.env. run-video.sh reads it on every restart, so a
    truncated file is a publisher that will not come back."""
    f = tmp_path / "video.env"
    f.write_text("PUBLISH_HOST=x\n")
    relay_server.write_env_keys(str(f), {"MJPEG_FPS": 5})
    assert not (tmp_path / "video.env.tmp").exists(), "the temp file was left behind"
    assert f.read_text().endswith("\n")


@pytest.mark.parametrize("key, live", [
    ("fps", True), ("width", True), ("quality", True),
    ("bitrate", False), ("maxfps", False), ("idr", False), ("nvr", False),
])
def test_each_knob_declares_whether_it_applies_live(key, live):
    """The operator has to know, BEFORE touching a control while driving, whether it
    costs five seconds of black screen. That fact belongs to the robot, so the UI reads
    it from here instead of hardcoding its own idea of which is which."""
    assert relay_server.VIDEO_PARAMS[key][4] is live
    assert (key in relay_server.LIVE_KEYS) is live


def test_restart_only_knobs_are_split_out_from_the_live_ones():
    """The defect: pushing a restart-only key to the publisher, which does not know it.
    It would be accepted, do nothing, and report success — a control that lies."""
    live, deferred = relay_server.split_live(
        {"fps": 10, "bitrate": 2000000, "nvr": 1})
    assert live == {"fps": 10}
    assert deferred == {"bitrate": 2000000, "nvr": 1}


def test_the_bitrate_floor_would_have_caught_the_value_that_was_live_on_the_robot():
    """Regression for a real one: BITRATE=60000 (60 kbps for 1080p H.264) sat in the
    robot's video.env, almost certainly a missing zero, and nothing rejected it because
    nothing checked. Found 2026-09-10."""
    with pytest.raises(ValueError, match="between"):
        relay_server.validate_video({"bitrate": 60000})
    assert relay_server.validate_video({"bitrate": 600000}) == {"BITRATE": 600000}
