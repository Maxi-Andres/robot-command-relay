"""The G1's microphone route (GET /audio/listen) — mic mode on demand, the multicast read, WAV.

Nothing here needs a robot: the audio sender is a fake speaking the one-line protocol, and the
multicast socket is a fake that hands out 160 ms packets. The real stream was measured on the
G1 on 2026-10-19 (see the MIC_* block in relay_server.py).
"""
from __future__ import annotations

import socket
import sys
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server  # noqa: E402

TOKEN = "test-token"  # noqa: S105 — a fake relay in a test, not a credential
PACKET = 5120          # 160 ms of 16 kHz s16le, what PC1 sends


class FakeSocket:
    """Hands out `packets` (bytes each) then times out, like the multicast going quiet."""

    def __init__(self, packets):
        self.packets = list(packets)
        self.closed = False

    def settimeout(self, _t):
        pass

    def recvfrom(self, _n):
        if not self.packets:
            raise socket.timeout()
        return self.packets.pop(0), ("192.168.123.161", 58106)

    def close(self):
        self.closed = True


class FakeAudio:
    def __init__(self, replies=None):
        self.lines = []
        self.replies = replies or {}

    def send(self, line):
        self.lines.append(line)
        return self.replies.get(line, f"ok {line}")


# --------------------------------------------------------------------------- #
# Pieces
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("value, seconds", [("5", 5.0), ("0.5", 0.5), (15, 15.0)])
def test_listen_lengths_are_accepted(value, seconds):
    assert relay_server.validate_listen_seconds(value) == seconds


@pytest.mark.parametrize("value", ["0", "-1", "16", "nan", "abc", None])
def test_listen_lengths_out_of_bounds_are_refused(value):
    """Catches: an open-ended listen — the relay carrying the robot's mic for as long as asked."""
    with pytest.raises(ValueError):
        relay_server.validate_listen_seconds(value)


def test_the_wav_is_what_the_speaker_route_would_accept():
    wav = relay_server.pcm_to_wav(b"\x01\x00" * 16000)
    assert relay_server.parse_wav(wav) == {"rate": 16000, "channels": 1, "seconds": 1.0}


def test_record_returns_exactly_the_seconds_asked():
    sock = FakeSocket([bytes([k]) * PACKET for k in range(20)])
    pcm = relay_server.record_mic(1.0, open_socket=lambda: sock)
    assert len(pcm) == 32000 and pcm[:PACKET] == b"\x00" * PACKET
    assert sock.closed


def test_no_packet_at_all_is_a_timeout_not_silence():
    """Catches: mic mode off answered with an empty WAV the speech-to-text turns into nothing —
    the failure must say the robot sent no audio."""
    with pytest.raises(TimeoutError, match="no microphone audio"):
        relay_server.record_mic(1.0, open_socket=lambda: FakeSocket([]))


def test_a_stream_that_stops_half_way_returns_what_came():
    pcm = relay_server.record_mic(2.0, open_socket=lambda: FakeSocket([b"\x00" * PACKET] * 3))
    assert len(pcm) == 3 * PACKET


# --------------------------------------------------------------------------- #
# Mic mode on demand
# --------------------------------------------------------------------------- #
def test_mic_mode_is_switched_on_once_and_off_after_idle():
    audio = FakeAudio()
    mic = relay_server.Mic(audio, record=lambda s: b"\x00" * int(s * 32000), idle_off_s=0.2)
    assert len(mic.listen(0.5)) == 16000
    mic.listen(0.5)
    assert audio.lines == ["mic on"], "a second listen inside the idle window reuses the mode"
    time.sleep(0.4)
    assert audio.lines == ["mic on", "mic off"] and not mic.on


def test_a_refused_mic_on_is_an_error_and_leaves_it_off():
    audio = FakeAudio({"mic on": "err mic: set mode rc=3104"})
    mic = relay_server.Mic(audio, record=lambda s: b"", idle_off_s=0.1)
    with pytest.raises(RuntimeError, match="rc=3104"):
        mic.listen(1.0)
    assert not mic.on and mic.timer is None


def test_only_one_listen_records_at_a_time():
    """Catches: two listens interleaving on one multicast socket — each would get half."""
    started, release = threading.Event(), threading.Event()

    def slow_record(_s):
        started.set()
        release.wait(2)
        return b""
    mic = relay_server.Mic(FakeAudio(), record=slow_record, idle_off_s=5)
    t = threading.Thread(target=mic.listen, args=(1.0,))
    t.start()
    assert started.wait(2)
    with pytest.raises(RuntimeError, match="busy"):
        mic.listen(1.0)
    release.set()
    t.join(2)
    mic.timer.cancel()


# --------------------------------------------------------------------------- #
# Over HTTP
# --------------------------------------------------------------------------- #
@pytest.fixture
def relay(tmp_path, monkeypatch):
    monkeypatch.setattr(relay_server, "AUDIT_LOG", str(tmp_path / "audit.log"))
    srv = relay_server.ThreadingHTTPServer(("127.0.0.1", 0), relay_server.Handler)
    srv.token = TOKEN
    srv.audio = FakeAudio()
    srv.mic = relay_server.Mic(srv.audio, record=lambda s: b"\x02\x00" * int(s * 16000),
                               idle_off_s=5)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}", srv
    srv.shutdown()
    if srv.mic and srv.mic.timer:
        srv.mic.timer.cancel()


def get(url, path, token=TOKEN):
    req = urllib.request.Request(url + path, headers={"Authorization": f"Bearer {token}"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.headers.get("Content-Type"), r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers.get("Content-Type"), exc.read()


def test_listen_answers_a_wav_of_the_seconds_asked(relay):
    url, srv = relay
    code, ctype, body = get(url, "/audio/listen?seconds=1.5")
    assert (code, ctype) == (200, "audio/wav")
    assert relay_server.parse_wav(body)["seconds"] == 1.5
    assert srv.audio.lines == ["mic on"]


def test_listen_needs_the_token(relay):
    url, srv = relay
    code, _ctype, _body = get(url, "/audio/listen?seconds=1", token="wrong")  # noqa: S106
    assert code == 401 and srv.audio.lines == []


def test_listen_refuses_a_bad_length(relay):
    url, _srv = relay
    code, _ctype, body = get(url, "/audio/listen?seconds=60")
    assert code == 400 and b"at most" in body


def test_a_relay_without_a_mic_says_so(relay):
    """The Go2's relay has a speaker but no mic route (its mic is WebRTC only)."""
    url, srv = relay
    srv.mic = None
    code, _ctype, body = get(url, "/audio/listen?seconds=1")
    assert code == 404 and b"no microphone" in body


# --------------------------------------------------------------------------- #
# The continuous stream (the app's "robot mic" switch)
# --------------------------------------------------------------------------- #
class EndlessSocket(FakeSocket):
    """A multicast that keeps sending 160 ms packets, slowly, until closed."""

    def __init__(self):
        super().__init__([])

    def recvfrom(self, _n):
        if self.closed:
            raise OSError("closed")
        time.sleep(0.01)
        return b"\x03\x00" * (PACKET // 2), ("192.168.123.161", 58106)


def test_streams_keep_the_mic_on_and_the_last_one_turns_it_off_at_once():
    """Catches: switching the robot's mic off in the app leaving it on for another minute."""
    audio = FakeAudio()
    mic = relay_server.Mic(audio, record=lambda s: b"", idle_off_s=60)
    mic.open_stream()
    mic.open_stream()
    mic.close_stream()
    assert audio.lines == ["mic on"], "one stream still open"
    mic.close_stream()
    assert audio.lines == ["mic on", "mic off"] and not mic.on and mic.timer is None


def test_a_listen_during_a_stream_does_not_schedule_the_mic_off():
    audio = FakeAudio()
    mic = relay_server.Mic(audio, record=lambda s: b"", idle_off_s=0.1)
    mic.open_stream()
    mic.listen(1.0)
    time.sleep(0.3)
    assert audio.lines == ["mic on"] and mic.on
    mic.close_stream()
    assert audio.lines == ["mic on", "mic off"]


def test_too_many_streams_are_refused():
    mic = relay_server.Mic(FakeAudio(), record=lambda s: b"", max_streams=1)
    mic.open_stream()
    with pytest.raises(RuntimeError, match="busy"):
        mic.open_stream()


def test_the_stream_passes_packets_until_the_caller_hangs_up(relay, monkeypatch):
    url, srv = relay
    sockets = []
    monkeypatch.setattr(relay_server, "mic_socket",
                        lambda: sockets.append(EndlessSocket()) or sockets[-1])
    req = urllib.request.Request(url + "/audio/mic/stream",
                                 headers={"Authorization": f"Bearer {TOKEN}"})
    r = urllib.request.urlopen(req, timeout=10)
    assert r.headers.get("Content-Type") == "audio/L16;rate=16000;channels=1"
    got = r.read(3 * PACKET)
    assert got == b"\x03\x00" * (3 * PACKET // 2)
    r.close()
    deadline = time.time() + 5
    while srv.mic.on and time.time() < deadline:
        time.sleep(0.05)
    assert srv.audio.lines == ["mic on", "mic off"], "hanging up switched the mic off"
    assert sockets[0].closed


def test_the_stream_needs_the_token_and_a_mic(relay):
    url, srv = relay
    code, _ctype, _body = get(url, "/audio/mic/stream", token="wrong")  # noqa: S106
    assert code == 401
    srv.mic = None
    code, _ctype, body = get(url, "/audio/mic/stream")
    assert code == 404 and b"no microphone" in body
