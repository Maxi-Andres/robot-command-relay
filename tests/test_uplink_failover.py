"""Run the uplink-failover shell test inside the pytest gate (pre-commit and CI).

The script under test is bash and edits routes, so its real test is a bash harness with
stub `ip` and `ping`; this wrapper only makes sure that harness runs wherever pytest does.
"""
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent


def test_uplink_failover_harness():
    r = subprocess.run(["bash", str(HERE / "test_uplink_failover.sh")],
                       capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.count("PASS") == 5, r.stdout
