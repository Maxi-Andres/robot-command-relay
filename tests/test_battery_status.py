"""`relay_server.battery_status` — what the app is told about the battery.

TWO THINGS ARE WORTH GUARDING HERE, and neither is the happy path:

* **The sign convention.** `charging` comes from the sign of the BMS current. Backwards, the
  app says "charging" while the robot drains — a wrong answer that looks right, which is the
  worst kind. Measured on the Go2 on its dock at 95%: current +475, so POSITIVE = CHARGING.
* **Staleness.** A battery percentage that stops updating looks perfectly healthy. If the
  telemetry pipeline dies, this must say so rather than serve 95% for ever.

And the endpoint must never raise: it is folded into /health, which has to answer even when
everything it reports on is down.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import relay_server


def write(tmp_path, monkeypatch, **fields):
    f = tmp_path / "battery.json"
    f.write_text(json.dumps(fields))
    monkeypatch.setattr(relay_server, "BATTERY_FILE", str(f))
    return f


def test_a_positive_current_means_charging(tmp_path, monkeypatch):
    """Measured on the robot on its wireless dock at 95%: +475."""
    write(tmp_path, monkeypatch, soc=95, current=475, at=time.time())
    assert relay_server.battery_status()["charging"] is True


def test_a_negative_current_means_discharging(tmp_path, monkeypatch):
    """The same robot a minute later, lifted off the dock: -5678. Both numbers are real, and
    together they are what pins the convention — the magnitudes corroborate it, since 5.6 A at
    32.5 V is ~185 W and that is a Go2 standing with LiDAR, DDS and video running."""
    write(tmp_path, monkeypatch, soc=95, current=-5678, at=time.time())
    assert relay_server.battery_status()["charging"] is False


def test_an_unknown_current_does_not_claim_either(tmp_path, monkeypatch):
    """`null` is an honest answer; guessing "not charging" would be a lie with a number
    behind it."""
    write(tmp_path, monkeypatch, soc=80, at=time.time())
    assert relay_server.battery_status()["charging"] is None


def test_a_fresh_reading_is_not_stale(tmp_path, monkeypatch):
    write(tmp_path, monkeypatch, soc=95, current=475, at=time.time())
    st = relay_server.battery_status()
    assert st["stale"] is False and st["age_s"] < 5


def test_an_old_reading_is_marked_stale(tmp_path, monkeypatch):
    """THE one that matters: the telemetry pipeline died an hour ago and the file still says
    95%. The app must be able to tell."""
    write(tmp_path, monkeypatch, soc=95, current=475, at=time.time() - 3600)
    st = relay_server.battery_status()
    assert st["stale"] is True
    assert st["percent"] == 95, "the last known value is still worth showing, just labelled"


def test_a_missing_file_is_empty_and_not_an_error(tmp_path, monkeypatch):
    """Nothing has shipped yet, or the battery snapshot is turned off."""
    monkeypatch.setattr(relay_server, "BATTERY_FILE", str(tmp_path / "nope.json"))
    assert relay_server.battery_status() == {}


def test_a_half_written_file_is_empty_and_not_an_error(tmp_path, monkeypatch):
    """The writer renames atomically, so this should not happen — but /health must answer
    even when it does."""
    f = tmp_path / "battery.json"
    f.write_text('{"soc": 9')
    monkeypatch.setattr(relay_server, "BATTERY_FILE", str(f))
    assert relay_server.battery_status() == {}


def test_a_document_without_soc_is_empty(tmp_path, monkeypatch):
    """Something else wrote there, or the schema changed. Better nothing than a battery
    reading with no charge in it."""
    write(tmp_path, monkeypatch, current=475, at=time.time())
    assert relay_server.battery_status() == {}


def test_volts_and_temperature_come_through_in_readable_units(tmp_path, monkeypatch):
    """The BMS reports millivolts and two thermistors; the app wants volts and the hottest."""
    write(tmp_path, monkeypatch, soc=95, current=475, volt_mv=32783, mcu_ntc=34, bq_ntc=32,
          at=time.time())
    st = relay_server.battery_status()
    assert st["volts"] == 32.78
    assert st["temp_c"] == 34
