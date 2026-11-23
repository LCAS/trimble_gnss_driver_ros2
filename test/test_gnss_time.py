import struct
from types import SimpleNamespace

import pytest

from trimble_gnss_driver_ros2.gsof_driver import (ATTITUDE, POSITION_TIME, SECONDS_PER_WEEK, GSOFDriver,
                                                  gps_to_unix)
from trimble_gnss_driver_ros2.scripts.parser import parse_maps

WEEK, TOW_MS = 2446, 134504400          # 2026-11-23T13:21:26.4Z (GPS) = 13:21:26.4 - 18 s UTC
UNIX = 315964800 + WEEK * SECONDS_PER_WEEK + TOW_MS / 1000 - 18


def test_gps_time_with_week():
    assert gps_to_unix(TOW_MS, UNIX + 0.04, 18, WEEK) == pytest.approx(UNIX)


def test_gps_time_week_from_host_clock_across_rollover():
    assert gps_to_unix(TOW_MS, UNIX + 0.04, 18) == pytest.approx(UNIX)
    # epoch just before the week rollover, host clock just after it
    end = 315964800 + (WEEK + 1) * SECONDS_PER_WEEK - 18
    assert gps_to_unix(SECONDS_PER_WEEK * 1000 - 100, end + 0.05, 18) == pytest.approx(end - 0.1)


def _parse(records):
    body = b"".join(bytes([t, len(p)]) + p for t, p in records)
    warnings = []
    node = SimpleNamespace(msg_bytes=body, records=[], rec_dict={}, current_time=0,
                           get_logger=lambda: SimpleNamespace(warn=lambda *a, **k: warnings.append(a)))
    GSOFDriver.get_records(node)
    return node, warnings


def test_records_with_optional_fields_and_epochs():
    velocity_with_local_heading = struct.pack(">B4f", 3, 1.2, 0.5, 0.0, 0.4)   # 17 bytes, map has 13
    fmt = parse_maps[ATTITUDE][1]
    attitude = struct.pack(fmt, TOW_MS, *([0] * (len(struct.unpack(fmt, bytes(struct.calcsize(fmt)))) - 1)))
    node, warnings = _parse([(POSITION_TIME, struct.pack(">LH4B", TOW_MS, WEEK, 20, 0, 0, 0)),
                             (8, velocity_with_local_heading), (ATTITUDE, attitude)])
    assert (node.pos_tow_ms, node.pos_week) == (TOW_MS, WEEK)
    assert node.rec_dict["VELOCITY"] == pytest.approx(1.2)
    assert node.att_tow_ms == TOW_MS
    assert node.records == [POSITION_TIME, 8, ATTITUDE] and not warnings
    _, warnings = _parse([(8, b"\x00" * 5)])      # truncated record: skipped, no crash
    assert warnings
