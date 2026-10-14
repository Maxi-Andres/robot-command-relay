"""Builds and runs tests/joint_ctl_test.cpp: the rules for moving the G1's arm joints
(src/joint_ctl_core.hpp), checked with no SDK and no robot.

The core is plain C++17 on purpose, so this runs wherever g++ does — CI included, unlike the
senders that link the Unitree SDK.
"""
import shutil
import subprocess
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent


@pytest.mark.skipif(shutil.which("g++") is None, reason="needs g++")
def test_joint_control_rules(tmp_path):
    exe = tmp_path / "joint_ctl_test"
    build = subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-O1",
         str(HERE / "joint_ctl_test.cpp"), "-o", str(exe)],
        capture_output=True, text=True, timeout=120)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stdout + run.stderr
    assert "PASS" in run.stdout
