"""
sim_demo/test/test_pose_history.py -- PURPOSE: unit tests of pose_history.py: interpolation by
time, and finding the shutter moment of a photo from the GPS in its file name (including the
hover case where many samples share one position).
"""
import math

import pytest

from sim_demo.geo import GeoOrigin
from sim_demo.photos import gps_photo_name, parse_latlon_from_name
from sim_demo.pose_history import PoseHistory, PoseSample

ORIGIN = GeoOrigin(38.3154, -76.549, 10.0)


def _line_history():
    """Drone flying east at 5 m/s, 40 m up, heading 90, samples every 0.2 s for 60 s."""
    h = PoseHistory()
    for i in range(301):
        t = 1000.0 + i * 0.2
        lat, lon, _ = ORIGIN.to_geo(5.0 * (t - 1000.0), 0.0)
        h.add(PoseSample(t, lat, lon, rel_alt=40.0 + 0.01 * i, heading_deg=90.0, ground_speed=5.0))
    return h


def test_at_time_interpolates():
    h = _line_history()
    s = h.at_time(1010.1)
    e, n, _ = ORIGIN.to_enu(s.lat, s.lon)
    assert e == pytest.approx(50.5, abs=1e-3)
    assert h.at_time(500.0) is None            # far outside the history


def test_capture_pose_from_file_name():
    h = _line_history()
    # The camera named the photo from the sample at t = 1020.0 (100 m east).
    s_true = h.at_time(1020.0)
    name = gps_photo_name(s_true.lat, s_true.lon)
    lat, lon = parse_latlon_from_name(name)
    s, how = h.capture_pose(ORIGIN, lat, lon, file_time=1022.5)
    assert how == 'gps-match'
    assert s.t == pytest.approx(1020.0, abs=0.2)
    assert s.rel_alt == pytest.approx(41.0, abs=0.02)       # sample 100: 40 + 0.01 * 100
    assert (s.lat, s.lon) == (lat, lon)


def test_capture_pose_without_gps_uses_time():
    h = _line_history()
    s, how = h.capture_pose(ORIGIN, None, None, file_time=1031.0, latency_s=1.0)
    assert how == 'time' and s.t == pytest.approx(1030.0)


def test_hover_picks_the_sample_closest_to_the_expected_time():
    h = PoseHistory()
    for i in range(100):                                      # hovering: same spot, climbing
        h.add(PoseSample(2000.0 + i * 0.5, ORIGIN.lat, ORIGIN.lon, rel_alt=10.0 + i * 0.5, heading_deg=0.0))
    s, how = h.capture_pose(ORIGIN, ORIGIN.lat, ORIGIN.lon, file_time=2040.0, latency_s=1.0)
    assert how == 'gps-match' and s.t == pytest.approx(2039.0, abs=0.5)


def test_empty_history():
    s, how = PoseHistory().capture_pose(ORIGIN, 1.0, 2.0, 0.0)
    assert s is None and how == 'none'


def test_heading_interpolation_wraps():
    h = PoseHistory()
    h.add(PoseSample(0.0, 1.0, 1.0, heading_deg=350.0))
    h.add(PoseSample(1.0, 1.0, 1.0, heading_deg=10.0))
    assert h.at_time(0.5).heading_deg == pytest.approx(0.0, abs=1e-9) or \
        h.at_time(0.5).heading_deg == pytest.approx(360.0, abs=1e-9)
    assert math.isnan(PoseSample(0, 0, 0).rel_alt)


def test_old_samples_are_dropped():
    h = PoseHistory(max_age_s=10.0)
    for i in range(100):
        h.add(PoseSample(float(i), 1.0, 1.0))
    kept = h.samples()
    # Pruning runs when the span exceeds 1.2 x max_age (so it does not run on every sample).
    assert kept[-1].t == 99.0 and kept[-1].t - kept[0].t <= 12.0 + 1e-9
