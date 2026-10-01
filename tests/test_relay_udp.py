"""Continuous teleop over UDP — the relay half. Refusal first: this path can move the robot.

WHY THE PATH EXISTS: over Starlink every HTTP command opened a new TCP connection (178 ms
median for a keepalive, a lost SYN costs a full second). See the UDP block in relay_server.py.

WHAT EACH TEST CATCHES:

* The datagram layout is a CONTRACT with `RelayTransport` in unitree_ros2/robot_executor,
  which cannot import this module. `GOLDEN_*` are the same bytes its test builds — keep
  them identical, or every UDP command is silently refused and the robot only stops.
* A forged, replayed, reordered or clock-skewed datagram must never reach command_sender.
* A `move` delayed in the network must never land after a stop and restart the robot.
* An invalid datagram must get NO reply, so this port cannot reflect traffic at anyone.
* NaN survives float() and every clamp; it must be refused before it reaches Move().

Nothing here opens a socket or spawns command_sender.
"""
from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server as r  # noqa: E402

KEY = b"test-key"
T = 1790001782.5
# move 0.25/-0.125/0.5 at T, and stop at T + 0.25, keyed with b"test-key".
# SAME BYTES as the executor side's test — keep them identical.
GOLDEN_MOVE = bytes.fromhex(
    "524301010000a09d50acda410000803e000000be0000003faacc43a3b69996a4bfaf308232e6f1a6")
GOLDEN_STOP = bytes.fromhex(
    "524301020000b09d50acda41000000000000000000000000d5f78883e27df1aca935ca4de3521738")
GOLDEN_ACK = bytes.fromhex("524101000000a09d50acda41283081628d0b7b1dd931b231d4c23212")


def cmd(kind=r.UDP_KIND_MOVE, ts=T, vx=0.25, vy=-0.125, vyaw=0.5, key=KEY):
    body = struct.pack("<2sBBdfff", b"RC", 1, kind, ts, vx, vy, vyaw)
    return body + r.udp_mac(key, body)


class FakeSender:
    def __init__(self):
        self.lines = []

    def send(self, line):
        self.lines.append(line)
        return "ok " + line.split()[0]


class OpenLimiter:
    def allow(self):
        return True


def control(limiter=None):
    return r.UdpControl(KEY, FakeSender(), limiter or OpenLimiter(), r.CommandOrder())


# --------------------------------------------------------------------------- #
# The contract
# --------------------------------------------------------------------------- #
def test_the_golden_datagrams_decode_to_the_documented_command():
    assert cmd() == GOLDEN_MOVE
    assert r.decode_command(GOLDEN_MOVE, KEY, T) == (r.UDP_KIND_MOVE, T, 0.25, -0.125, 0.5)
    assert r.decode_command(GOLDEN_STOP, KEY, T)[:2] == (r.UDP_KIND_STOP, T + 0.25)
    assert r.encode_ack(KEY, 0, T) == GOLDEN_ACK


def test_the_fields_sit_where_the_contract_says():
    """By offset, so a reordered Struct cannot pass by agreeing with itself."""
    assert GOLDEN_MOVE[0:2] == b"RC"
    assert GOLDEN_MOVE[2] == 1 and GOLDEN_MOVE[3] == r.UDP_KIND_MOVE
    assert struct.unpack("<d", GOLDEN_MOVE[4:12])[0] == T
    assert struct.unpack("<fff", GOLDEN_MOVE[12:24]) == (0.25, -0.125, 0.5)
    assert len(GOLDEN_MOVE) == 40 and len(GOLDEN_ACK) == 28


# --------------------------------------------------------------------------- #
# Refusal
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("datagram, reason", [
    (cmd(key=b"wrong-key"), "mac"),                       # forged
    (GOLDEN_MOVE[:-1] + bytes([GOLDEN_MOVE[-1] ^ 1]), "mac"),   # one bit off
    (GOLDEN_MOVE[:15] + bytes([GOLDEN_MOVE[15] ^ 1]) + GOLDEN_MOVE[16:], "mac"),  # vx edited
    (GOLDEN_MOVE[:-1], "size"),
    (GOLDEN_MOVE + b"\x00", "size"),
    (b"", "size"),
    (cmd(kind=7), "format"),                              # unknown kind, correctly signed
    (cmd(vx=math.nan), "format"),
    (cmd(vyaw=math.inf), "format"),
    (cmd(ts=T - 1.5), "clock"),                           # too old for the window
    (cmd(ts=T + 1.5), "clock"),                           # from the future
])
def test_bad_datagrams_are_refused_with_the_right_reason(datagram, reason):
    with pytest.raises(ValueError, match=f"^{reason}$"):
        r.decode_command(datagram, KEY, T)


def test_a_refused_datagram_reaches_nothing_and_gets_no_reply():
    """No reply is the anti-reflection property: a spoofed source learns nothing and
    receives nothing."""
    c = control()
    assert c.process(cmd(key=b"forged"), "6.6.6.6", T) is None
    assert c.sender.lines == []
    assert c.stats["mac"] == 1


def test_a_replayed_datagram_moves_the_robot_once():
    c = control()
    assert c.process(GOLDEN_MOVE, "hq", T) is not None
    assert c.process(GOLDEN_MOVE, "hq", T) is None
    assert c.sender.lines == ["move 0.250 -0.125 0.500"]
    assert c.stats["stale"] == 1


def test_an_older_move_arriving_late_is_dropped():
    c = control()
    c.process(cmd(ts=T + 0.2, vx=0.5), "hq", T)
    assert c.process(cmd(ts=T + 0.1, vx=0.1), "hq", T) is None
    assert c.sender.lines == ["move 0.500 -0.125 0.500"]


def test_a_move_sent_before_a_udp_stop_cannot_restart_the_robot():
    c = control()
    c.process(GOLDEN_STOP, "hq", T)                       # stop at T + 0.25
    assert c.process(cmd(ts=T + 0.1), "hq", T) is None    # the delayed move from before it
    assert c.sender.lines == ["stop_move"]


def test_a_move_sent_before_an_http_stop_cannot_restart_the_robot():
    """The two paths share one order: a stop over HTTP must fence the UDP moves too."""
    c = control()
    c.order.stop(T + 0.3, T)
    assert c.process(cmd(ts=T + 0.2), "hq", T) is None
    assert c.sender.lines == []


def test_an_http_stop_without_ts_still_fences_moves_already_in_flight():
    """An older executor sends no ts; the fence falls back to the robot clock plus a margin."""
    order = r.CommandOrder()
    order.stop(None, T)
    assert not order.admit_move(T + r.STOP_MARGIN_S - 0.01)
    assert order.admit_move(T + r.STOP_MARGIN_S + 0.01)


def test_a_stop_is_never_refused_even_when_rate_limited_or_out_of_order():
    class ClosedLimiter:
        def allow(self):
            return False
    c = control(ClosedLimiter())
    c.process(cmd(ts=T + 0.5), "hq", T)                   # a later move… refused by the limit
    assert c.process(cmd(kind=r.UDP_KIND_STOP, ts=T + 0.1), "hq", T) is not None
    assert c.sender.lines == ["stop_move"]
    assert c.stats["limited"] == 1


def test_the_ack_echoes_the_command_and_reports_a_sender_error():
    c = control()
    c.sender.send = lambda line: "err move -1"            # robot not in a sport state
    ack = c.process(GOLDEN_MOVE, "hq", T)
    magic, _v, status, ts = struct.unpack("<2sBBd", ack[:12])
    assert (magic, status, ts) == (b"RA", 1, T)
    assert ack[12:] == r.udp_mac(KEY, ack[:12])


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
def test_the_dead_man_defaults_are_one_second_everywhere():
    """Catches: the window changed in one of the four places that declare it and not the
    others — the C++ default, the unit file, the example env and what /health reports."""
    cpp = (REPO / "src" / "sender_core.hpp").read_text(encoding="utf-8")
    unit = (REPO / "systemd" / "robot-command-relay.service").read_text(encoding="utf-8")
    env = (REPO / "relay.env.example").read_text(encoding="utf-8")
    assert 'env_f("DEADMAN_MS", 1000)' in cpp
    assert "Environment=DEADMAN_MS=1000" in unit
    assert "\nDEADMAN_MS=1000\n" in env
    assert "os.environ.get(\"DEADMAN_MS\", \"1000\")" in (REPO / "relay_server.py").read_text()
