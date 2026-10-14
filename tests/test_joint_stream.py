"""The joint state stream: GET /joints/stream and the JointStream hub behind it.

Read-only by design, so these tests are about lifecycle and isolation, not refusal of motion:
the bridge starts with the first viewer and stops after the last, a slow or vanished viewer
never holds anything up, only whole JSON lines reach a viewer, and the token still gates it.
A fake bridge (a Python script) stands in for g1_joint_bridge; nothing touches DDS.
"""
from __future__ import annotations

import http.client
import json
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

# Prints {"n": k} every 20 ms (one garbage line among them) until stdin closes, like the real
# bridge does on EOF.
FAKE_BRIDGE = textwrap.dedent("""
    import sys, threading, time
    done = threading.Event()
    def watch():
        sys.stdin.read()
        done.set()
    threading.Thread(target=watch, daemon=True).start()
    n = 0
    while not done.is_set():
        n += 1
        print('{"n": %d}' % n, flush=True)
        if n == 2:
            print("not json", flush=True)
        time.sleep(0.02)
""")

# One line, then silence: what a viewer sees when the bridge stalls.
ONE_LINE_BRIDGE = textwrap.dedent("""
    import sys
    print('{"n": 1}', flush=True)
    sys.stdin.read()
""")


def _script(tmp_path, body, name="bridge.py"):
    path = tmp_path / name
    path.write_text(body, encoding="utf-8")
    return [sys.executable, str(path)]


def _wait(cond, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def stream(tmp_path):
    js = relay_server.JointStream(_script(tmp_path, FAKE_BRIDGE), max_clients=2, idle_s=0.2)
    yield js
    js.stop()


def test_first_viewer_starts_the_bridge_and_gets_whole_lines(stream):
    assert stream.status()["running"] is False, "nobody watching: no bridge"
    assert stream.subscribe()
    seen, lines = 0, []
    for _ in range(5):
        seen, line = stream.next_line(seen, 2.0)
        assert line is not None
        lines.append(json.loads(line))       # a torn or garbage line would fail here
    assert [d["n"] for d in lines] == sorted(d["n"] for d in lines)
    stream.unsubscribe()


def test_viewer_cap(stream):
    assert stream.subscribe() and stream.subscribe()
    assert not stream.subscribe(), "the third viewer must be refused, not queued"
    assert stream.status()["clients"] == 2
    stream.unsubscribe()
    stream.unsubscribe()


def test_bridge_stops_after_the_last_viewer_leaves(stream):
    assert stream.subscribe()
    proc = stream.proc
    stream.unsubscribe()
    assert _wait(lambda: proc.poll() is not None), "the bridge must exit once idle"
    assert stream.status()["running"] is False


def test_coming_back_within_the_idle_window_keeps_the_same_bridge(stream):
    assert stream.subscribe()
    pid = stream.proc.pid
    stream.unsubscribe()
    assert stream.subscribe()                 # a page reload, inside idle_s
    time.sleep(0.4)                           # past idle_s: the stop must have been cancelled
    assert stream.proc is not None and stream.proc.pid == pid
    stream.unsubscribe()


def test_a_dead_bridge_is_restarted_by_the_next_viewer(stream):
    assert stream.subscribe()
    first = stream.proc
    first.kill()
    assert _wait(lambda: stream.proc is None), "a dead bridge must be forgotten"
    stream.unsubscribe()
    assert stream.subscribe()
    assert stream.proc is not None and stream.proc.pid != first.pid
    stream.unsubscribe()


# --------------------------------------------------------------------------- #
# Over HTTP
# --------------------------------------------------------------------------- #
@pytest.fixture
def server(tmp_path, monkeypatch):
    monkeypatch.setattr(relay_server, "AUDIT_LOG", str(tmp_path / "audit.log"))
    monkeypatch.setattr(relay_server, "JOINT_HEARTBEAT_S", 0.2)
    srv = ThreadingHTTPServer(("127.0.0.1", 0), relay_server.Handler)
    srv.token = TOKEN
    srv.joints = relay_server.JointStream(_script(tmp_path, FAKE_BRIDGE), idle_s=0.2)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    if srv.joints is not None:
        srv.joints.stop()


def _get(srv, token=TOKEN):
    conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
    headers = {"Authorization": f"Bearer {token}"} if token else {}
    conn.request("GET", "/joints/stream", headers=headers)
    return conn, conn.getresponse()


def test_stream_needs_the_token(server):
    for token in (None, "wrong"):
        conn, r = _get(server, token)
        assert r.status == 401
        conn.close()
    assert server.joints.status()["running"] is False, "a refused caller must not start it"


def test_stream_delivers_lines_and_releases_the_viewer_on_hang_up(server):
    conn, r = _get(server)
    assert r.status == 200
    assert r.getheader("Content-Type") == "application/x-ndjson"
    for _ in range(3):
        assert "n" in json.loads(r.fp.readline())
    r.close()       # the response holds its own reference to the socket
    conn.close()
    assert _wait(lambda: server.joints.status()["clients"] == 0), \
        "a viewer that hung up must be released, or the cap fills with ghosts"


def test_a_stalled_bridge_sends_stale_heartbeats(server, tmp_path):
    server.joints = relay_server.JointStream(
        _script(tmp_path, ONE_LINE_BRIDGE, "one.py"), idle_s=0.2)
    conn, r = _get(server)
    assert json.loads(r.fp.readline()) == {"n": 1}
    assert json.loads(r.fp.readline()) == {"stale": True}
    conn.close()


def test_no_stream_on_a_go2(server):
    server.joints = None
    conn, r = _get(server)
    assert r.status == 404
    conn.close()
