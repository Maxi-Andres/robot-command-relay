"""The Go2's speaker routes (/audio/*) — what gets through to the robot and what is refused.

Nothing here needs a robot: the audio sender is replaced by a small script that speaks the same
one-line protocol and records what it was asked. The real go2_audio_sender was checked against
the robot on 2026-10-14 (see its header).
"""
from __future__ import annotations

import io
import json
import os
import struct
import sys
import textwrap
import threading
import urllib.error
import urllib.request
import wave
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server  # noqa: E402

TOKEN = "test-token"  # noqa: S105 — a fake relay in a test, not a credential


def make_wav(seconds=1.0, rate=22050, channels=1, width=2):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(channels)
        w.setsampwidth(width)
        w.setframerate(rate)
        w.writeframes(b"\x00" * int(seconds * rate) * channels * width)
    return buf.getvalue()


# --------------------------------------------------------------------------- #
# What the speaker accepts
# --------------------------------------------------------------------------- #
@pytest.mark.parametrize("rate, channels", [(22050, 1), (44100, 1), (44100, 2), (16000, 1)])
def test_playable_wavs_are_accepted(rate, channels):
    """22.05 kHz mono is what Piper makes; 44.1 kHz is what the first test on the robot used.
    Both were heard on the Go2 (2026-10-14)."""
    info = relay_server.parse_wav(make_wav(2.0, rate, channels))
    assert info == {"rate": rate, "channels": channels, "seconds": 2.0}


@pytest.mark.parametrize("data, why", [
    (b"ID3\x04" + b"\x00" * 100, "not a WAV"),                 # an MP3
    (make_wav(1.0, width=1), "16-bit"),                         # 8-bit PCM
    (make_wav(1.0, rate=96000), "8-48 kHz"),
    (make_wav(31.0), "limit is 30"),                            # a playlist, not a phrase
    (b"RIFF\x00\x00\x00\x00WAVE" + b"\x00" * 40, "no data chunk"),
])
def test_unplayable_audio_is_refused_with_a_reason(data, why):
    with pytest.raises(ValueError, match=why):
        relay_server.parse_wav(data)


def test_oversize_audio_is_refused_before_parsing(monkeypatch):
    monkeypatch.setattr(relay_server, "AUDIO_MAX_BYTES", 1000)
    with pytest.raises(ValueError, match="limit is 1000"):
        relay_server.parse_wav(make_wav(1.0))


@pytest.mark.parametrize("level", [0, 5, 10, 7.0])
def test_volume_levels_are_accepted(level):
    assert relay_server.validate_volume(level) == int(level)


@pytest.mark.parametrize("level", [-1, 11, 2.5, "5", None, True])
def test_bad_volume_levels_are_refused(level):
    with pytest.raises(ValueError):
        relay_server.validate_volume(level)


# --------------------------------------------------------------------------- #
# End to end over HTTP, with a fake audio sender
# --------------------------------------------------------------------------- #
FAKE_SENDER = textwrap.dedent("""
    import os, sys
    log = open(sys.argv[1], "a")
    volume = 10
    for line in sys.stdin:
        line = line.strip()
        log.write(line + "\\n"); log.flush()
        if line.startswith("say "):
            path = line[4:]
            log.write(f"exists={os.path.exists(path)}\\n"); log.flush()
            print("ok say 1.00", flush=True)
        elif line == "stop":
            print("ok stop", flush=True)
        elif line == "volume":
            print(f"ok volume {volume}", flush=True)
        elif line.startswith("volume "):
            volume = int(line.split()[1]); print(f"ok volume {volume}", flush=True)
        else:
            print("err unknown", flush=True)
""")


@pytest.fixture
def relay(tmp_path, monkeypatch):
    monkeypatch.setattr(relay_server, "AUDIT_LOG", str(tmp_path / "audit.log"))
    fake = tmp_path / "fake_audio.py"
    fake.write_text(FAKE_SENDER)
    calls = tmp_path / "calls.log"
    srv = relay_server.ThreadingHTTPServer(("127.0.0.1", 0), relay_server.Handler)
    srv.token = TOKEN
    srv.audio = relay_server.Sender([sys.executable, str(fake), str(calls)],
                                    label="audio_sender", timeout=5, lazy=True)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    audio = srv.audio
    yield f"http://127.0.0.1:{srv.server_address[1]}", srv, calls
    srv.shutdown()
    if audio.proc:
        audio.proc.stdin.close()
        audio.proc.wait(timeout=5)


def call(url, path, body=None, token=TOKEN, method=None, ctype="application/json"):
    req = urllib.request.Request(url + path, data=body, method=method,
                                 headers={"Authorization": f"Bearer {token}",
                                          "Content-Type": ctype})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_say_plays_the_wav_and_cleans_up_its_temp_file(relay):
    url, _srv, calls = relay
    code, body = call(url, "/audio/say", make_wav(1.5), ctype="audio/wav")
    assert (code, body) == (200, {"ok": True, "seconds": 1.5})
    log = calls.read_text().splitlines()
    assert log[0].startswith("say /") and log[1] == "exists=True"
    assert not os.path.exists(log[0][4:]), "the temp WAV was left on the robot's disk"


def test_volume_round_trip(relay):
    url, _srv, _calls = relay
    assert call(url, "/audio/volume", method="GET") == (200, {"ok": True, "volume": 10})
    assert call(url, "/audio/volume", json.dumps({"level": 4}).encode()) == \
        (200, {"ok": True, "volume": 4})
    assert call(url, "/audio/volume", method="GET") == (200, {"ok": True, "volume": 4})


def test_a_bad_wav_never_reaches_the_robot(relay):
    url, _srv, calls = relay
    code, body = call(url, "/audio/say", b"not audio", ctype="audio/wav")
    assert code == 400 and "not a WAV" in body["error"]
    assert not calls.exists() or "say" not in calls.read_text()


@pytest.mark.parametrize("path, method, body", [
    ("/audio/say", "POST", make_wav(0.5)),
    ("/audio/volume", "GET", None),
    ("/audio/volume", "POST", b'{"level": 0}'),
    ("/audio/stop", "POST", b""),
])
def test_every_audio_route_needs_the_token(relay, path, method, body):
    """A speaker on a robot in a public place is an outward-facing control: same rule as /cmd."""
    url, _srv, calls = relay
    code, _ = call(url, path, body, token=TOKEN + "-wrong", method=method)
    assert code == 401
    assert not calls.exists()


def test_a_relay_without_a_speaker_says_so(relay):
    """The G1's relay (and a Go2 relay not rebuilt yet) answers 404 with a reason."""
    url, srv, _calls = relay
    srv.audio = None
    code, body = call(url, "/audio/volume", method="GET")
    assert code == 404 and "no speaker" in body["error"]


def test_the_wav_parser_matches_the_cpp_duration_rule():
    """go2_audio_sender times the end of a phrase as data bytes / byte rate, the same rule as
    parse_wav. A disagreement would cut phrases short or leave megaphone mode open."""
    wav = make_wav(2.5, 44100, 2)
    data_at = wav.find(b"data")
    size = struct.unpack("<I", wav[data_at + 4:data_at + 8])[0]
    assert size / (44100 * 2 * 2) == relay_server.parse_wav(wav)["seconds"]
