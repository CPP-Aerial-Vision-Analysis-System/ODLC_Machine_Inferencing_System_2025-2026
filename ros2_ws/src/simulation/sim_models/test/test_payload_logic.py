"""
test/test_payload_logic.py -- PURPOSE: unit tests of sim_models.payload_logic (no ROS, no Gazebo):
when a payload is released (servo pulse width rules), when a falling payload counts as landed,
and the ENU -> latitude/longitude conversion.

Run:  python3 -m pytest -q test/test_payload_logic.py     (or colcon test --packages-select sim_models)
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from sim_models.payload_logic import (
    CLOSED_PWM_US,
    OPEN_PWM_US,
    LandingDetector,  # noqa: E402
    ServoReleaseDetector,
    enu_to_geodetic,
)


# ----------------------------------------------------------------- ServoReleaseDetector
def test_flight_code_sequence_releases_once():
    """main_controller.py: 1400 us (open) for 3 s, then 500 us (closed): exactly one release."""
    det = ServoReleaseDetector()
    t = 0.0
    results = []
    for _ in range(150):  # 3 s at 50 Hz
        results.append(det.update(OPEN_PWM_US, t))
        t += 0.02
    for _ in range(50):
        results.append(det.update(CLOSED_PWM_US, t))
        t += 0.02
    assert results.count(True) == 1
    assert results[0] is True  # the very first open reading releases
    assert det.released and det.release_time == 0.0 and det.release_pwm_us == OPEN_PWM_US


def test_closed_and_zero_never_release():
    """0 us (output off), 500 us (closed) and NaN are all 'closed'."""
    det = ServoReleaseDetector()
    for i, pwm in enumerate([0.0, 500.0, 1000.0, 1399.0, float("nan"), -5.0]):
        assert det.update(pwm, i * 0.1) is False
    assert not det.released


def test_float_rounding_of_relayed_value_still_opens():
    """3000 * (1400 / 3000) can come out as 1399.9999999: must count as open."""
    det = ServoReleaseDetector(1400.0)
    assert det.is_open(3000.0 * (1400.0 / 3000.0))
    assert det.is_open(1399.6)
    assert not det.is_open(1399.4)


def test_values_above_threshold_open():
    """Older team code opened with 1900 us: that is open too."""
    det = ServoReleaseDetector()
    assert det.update(1900.0, 1.0) is True


def test_no_second_release_after_reopen():
    det = ServoReleaseDetector()
    assert det.update(1400.0, 0.0)
    assert not det.update(500.0, 1.0)
    assert not det.update(1400.0, 2.0)


def test_min_open_time_debounce():
    """With min_open_time_s = 0.2 a 0.1 s glitch does not release, a 0.2 s opening does."""
    det = ServoReleaseDetector(1400.0, min_open_time_s=0.2)
    assert not det.update(1400.0, 0.0)
    assert not det.update(1400.0, 0.1)
    assert not det.update(500.0, 0.15)  # glitch over: timer resets
    assert not det.update(1400.0, 1.0)
    assert not det.update(1400.0, 1.1)
    assert det.update(1400.0, 1.2)


def test_bad_detector_arguments():
    with pytest.raises(ValueError):
        ServoReleaseDetector(0.0)
    with pytest.raises(ValueError):
        ServoReleaseDetector(1400.0, min_open_time_s=-1.0)


# ----------------------------------------------------------------- LandingDetector
def _fall_samples(z0=15.0, dt=0.05, rest_extra_s=2.0):
    """Free fall from z0 to the ground (z = 0.03), then resting."""
    t, z, vz = 0.0, z0, 0.0
    out = [(t, 1.0, 2.0, z)]
    while z > 0.03:
        t += dt
        vz -= 9.81 * dt
        z = max(0.03, z + vz * dt)
        out.append((t, 1.0, 2.0, z))
    end = t + rest_extra_s
    while t < end:
        t += dt
        out.append((t, 1.0, 2.0, 0.03))
    return out


def test_landing_detected_after_fall():
    land = LandingDetector(rest_speed=0.05, rest_time_s=1.0)
    samples = _fall_samples()
    land.start(*samples[0])
    hits = [land.update(*s) for s in samples[1:]]
    assert hits.count(True) == 1
    t, x, y, z = land.landed_point
    assert z == pytest.approx(0.03)
    summary = land.fall_summary()
    assert summary["fall_height_m"] == pytest.approx(15.0 - 0.03, abs=1e-3)
    assert 1.5 < summary["fall_time_s"] < 3.5  # ~1.75 s of fall + 1 s at rest
    assert summary["drift_m"] == pytest.approx(0.0)


def test_no_landing_before_start_or_while_hovering_start():
    land = LandingDetector(min_fall_time_s=0.3)
    assert not land.update(0.0, 0, 0, 10)  # not started
    land.start(0.0, 0, 0, 10)
    # still at rest relative to the world in the first 0.3 s: must NOT count as landed
    assert not land.update(0.1, 0, 0, 10)
    assert not land.update(0.2, 0, 0, 10)
    assert not land.landed


def test_landing_needs_rest_time():
    """Rest is counted from the start of the first sample interval without movement."""
    land = LandingDetector(rest_speed=0.05, rest_time_s=1.0, min_fall_time_s=0.0)
    land.start(0.0, 0, 0, 1.0)
    assert not land.update(0.5, 0, 0, 0.0)  # 0.0 -> 0.5 s: falling (2 m/s)
    assert not land.update(1.0, 0, 0, 0.0)  # 0.5 -> 1.0 s: at rest for 0.5 s
    assert land.update(1.5, 0, 0, 0.0)  # 0.5 -> 1.5 s: at rest for 1.0 s = landed
    assert land.landed_point == (1.5, 0, 0, 0.0)
    assert not land.update(2.0, 0, 0, 0.0)  # reported only once


def test_landing_bad_arguments():
    with pytest.raises(ValueError):
        LandingDetector(rest_speed=0.0)


# ----------------------------------------------------------------- enu_to_geodetic
def test_enu_origin_is_origin():
    lat, lon, alt = enu_to_geodetic(0, 0, 0, 36.2124, -96.004, 200.0)
    assert (lat, lon, alt) == pytest.approx((36.2124, -96.004, 200.0))


def test_enu_offsets_match_known_distances():
    """100 m north / east at latitude 36.2 deg: compare with the haversine distance."""
    lat0, lon0 = 36.2124, -96.004
    lat_n, lon_n, _ = enu_to_geodetic(0, 100.0, 0, lat0, lon0)
    lat_e, lon_e, _ = enu_to_geodetic(100.0, 0, 0, lat0, lon0)
    assert lon_n == pytest.approx(lon0) and lat_n > lat0
    assert lat_e == pytest.approx(lat0) and lon_e > lon0

    def haversine(a1, o1, a2, o2):
        r = 6371008.8
        p1, p2 = math.radians(a1), math.radians(a2)
        dp, dl = p2 - p1, math.radians(o2 - o1)
        h = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return 2 * r * math.asin(math.sqrt(h))

    # the spherical haversine differs from the ellipsoid by < 0.5 %
    assert haversine(lat0, lon0, lat_n, lon_n) == pytest.approx(100.0, rel=5e-3)
    assert haversine(lat0, lon0, lat_e, lon_e) == pytest.approx(100.0, rel=5e-3)
