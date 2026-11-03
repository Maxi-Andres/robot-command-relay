"""GET /camera/depth — one depth picture in millimetres, passed through from camera_source.

Nothing here needs a robot: camera_source is a small HTTP server that answers /depth the way
the real one does (robot-video-pipeline/robot/camera_source.py, tested there).
"""
from __future__ import annotations

import json
import sys
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import relay_server  # noqa: E402

TOKEN = "test-token"  # noqa: S105 — a fake relay in a test, not a credential
PNG = b"\x89PNG\r\n\x1a\n" + b"\x00" * 64
INFO = {"width": 848, "height": 480, "frames": 5, "scale_m": 0.001}


@pytest.fixture
def camera(monkeypatch):
    """A fake camera_source. `state["off"]` makes it answer like a robot with snapshots off."""
    state = {"off": False, "asked": []}

    class Fake(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            state["asked"].append(self.path)
            if state["off"]:
                body = json.dumps({"ok": False, "error": "depth snapshots are off on this "
                                                         "robot (DEPTH_SNAPSHOT=1)"}).encode()
                self.send_response(404)
            else:
                body = PNG
                self.send_response(200)
                self.send_header("X-Depth-Info", json.dumps(INFO))
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = ThreadingHTTPServer(("127.0.0.1", 0), Fake)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    monkeypatch.setattr(relay_server, "CAMERA_LOCAL", f"http://127.0.0.1:{srv.server_address[1]}")
    yield state
    srv.shutdown()


@pytest.fixture
def relay(tmp_path, monkeypatch):
    monkeypatch.setattr(relay_server, "AUDIT_LOG", str(tmp_path / "audit.log"))
    srv = relay_server.ThreadingHTTPServer(("127.0.0.1", 0), relay_server.Handler)
    srv.token = TOKEN
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def get(url, token=TOKEN):
    req = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}"})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, r.headers, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers, exc.read()


def test_the_png_and_its_facts_pass_through_untouched(relay, camera):
    """The defect: the relay re-encoding or dropping the header that says what a pixel is."""
    code, headers, body = get(relay + "/camera/depth?frames=3")
    assert code == 200 and body == PNG
    assert headers["Content-Type"] == "image/png"
    assert json.loads(headers["X-Depth-Info"]) == INFO
    assert camera["asked"] == ["/depth?frames=3"]


def test_it_needs_the_token(relay, camera):
    code, _h, _b = get(relay + "/camera/depth", token="wrong")  # noqa: S106 — a fake token in a test
    assert code == 401 and camera["asked"] == []


@pytest.mark.parametrize("frames", ["0", "16", "x"])
def test_the_frame_count_is_bounded(relay, camera, frames):
    """The defect: a caller asking for 1000 frames and holding the depth node for a minute."""
    code, _h, _b = get(relay + f"/camera/depth?frames={frames}")
    assert code == 400 and camera["asked"] == []


def test_a_robot_with_snapshots_off_says_so(relay, camera):
    """The Go2: off on purpose (LTE). The answer must say why, not just 'not found'."""
    camera["off"] = True
    code, _h, body = get(relay + "/camera/depth")
    assert code == 404 and "DEPTH_SNAPSHOT" in json.loads(body)["error"]


def test_no_camera_source_is_an_honest_502(relay, monkeypatch):
    monkeypatch.setattr(relay_server, "CAMERA_LOCAL", "http://127.0.0.1:9")
    code, _h, body = get(relay + "/camera/depth")
    assert code == 502 and "unreachable" in json.loads(body)["error"]
