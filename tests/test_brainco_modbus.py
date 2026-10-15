"""Builds and runs tests/brainco_test.cpp: the Modbus framing used to READ the G1's BrainCo
hands (src/brainco_modbus.hpp), checked against real answers from the robot, with no device.

Plain C++17 and POSIX headers, so this runs wherever g++ does — CI included.
"""
import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


@pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")
def test_brainco_modbus_framing(tmp_path):
    exe = tmp_path / "brainco_test"
    build = subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O1",
         str(HERE / "brainco_test.cpp"), "-o", str(exe)],
        capture_output=True, text=True, timeout=120)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "PASS" in run.stdout


def test_the_viewer_cannot_move_a_finger():
    """g1_joint_bridge feeds the read-only Joints view: it must never get the hands' write."""
    bridge = (HERE.parent / "src" / "g1_joint_bridge.cpp").read_text(encoding="utf-8")
    assert "brainco_command" not in bridge
    assert "write_targets" not in bridge and "WRITE_MULTIPLE" not in bridge
    reader = (HERE.parent / "src" / "brainco_modbus.hpp").read_text(encoding="utf-8")
    assert "0x10" not in reader and "0x06" not in reader, "the reader builds function 04 only"
