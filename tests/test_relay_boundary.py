"""The relay's security boundary: what it accepts, how fast, and whether its allowlist still
agrees with the one in C++.

This service is the only remote path that can move the robot, so the tests here are about
refusal, not capability. Nothing touches DDS or spawns `command_sender`.
"""
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
def _sender_verbs() -> set[str]:
    """The verbs `command_sender.cpp` actually dispatches, read out of its VERBS map.

    Parsing the C++ is deliberate. The two lists are duplicated ON PURPOSE — defence in
    depth, and the boundary forbids sharing code with a binary — so the only way to keep them
    honest is a test that reads both.
    """
    src = (REPO / "src" / "command_sender.cpp").read_text(encoding="utf-8")
    table = re.search(r"VERBS\s*=\s*\{(.*?)\n\};", src, re.S)
    assert table, "could not find the VERBS dispatch table in command_sender.cpp"
    verbs = set(re.findall(r'\{\s*"(\w+)"', table.group(1)))
    # `move` and `keepalive` are handled before the table (they take arguments / refresh the
    # dead-man rather than dispatching a client call), so they are added here explicitly.
    return verbs | {"move", "keepalive"}


def test_python_and_cpp_allowlists_agree():
    """Catches: a verb added to one side only.

    Added to C++ alone, it is unreachable — confusing but safe. Added to Python alone, the
    relay accepts a command that `command_sender` then refuses, which surfaces as a useless
    502 mid-drive. Either way the two must not drift.
    """
    python_verbs = relay_server.VERBS | {"move"}
    assert python_verbs == _sender_verbs()


def test_acrobatics_are_absent_from_the_allowlist():
    """Catches: someone adding a flip or a handstand to the remote path.

    These exist in the robot's SDK and are deliberately unreachable from the network: the
    relay cannot be the way a flip gets triggered from another continent.
    """
    for verb in ["front_flip", "back_flip", "handstand", "walk_upright", "dance1", "front_jump"]:
        assert verb not in relay_server.VERBS
        assert verb not in _sender_verbs()


def test_stop_is_always_in_the_allowlist():
    """Catches: a refactor that removes the one verb you cannot afford to lose."""
    assert "stop_move" in relay_server.VERBS
    assert "stop_move" in _sender_verbs()


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
