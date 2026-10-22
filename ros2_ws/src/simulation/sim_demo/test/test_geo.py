"""
sim_demo/test/test_geo.py -- PURPOSE: unit tests of geo.py (GPS <-> world metres, distances,
home-string parsing, angle helpers). No ROS needed.
"""
import math

import pytest

from sim_demo.geo import (GeoOrigin, compass_to_enu_yaw_deg, enu_yaw_to_compass_deg, haversine_m,
                          local_distance_m, parse_home, quaternion_to_yaw_deg, wrap_deg)

ORIGIN = GeoOrigin(38.3154, -76.5490, 10.0)


def test_origin_maps_to_zero():
    e, n, u = ORIGIN.to_enu(38.3154, -76.5490, 10.0)
    assert abs(e) < 1e-9 and abs(n) < 1e-9 and abs(u) < 1e-9


@pytest.mark.parametrize('east,north', [(100.0, 0.0), (0.0, 250.0), (-80.0, 40.0), (500.0, -300.0)])
def test_round_trip(east, north):
    lat, lon, alt = ORIGIN.to_geo(east, north, 12.0)
    e, n, u = ORIGIN.to_enu(lat, lon, alt)
    assert e == pytest.approx(east, abs=1e-6)
    assert n == pytest.approx(north, abs=1e-6)
    assert u == pytest.approx(12.0, abs=1e-9)


def test_one_degree_of_latitude_matches_ardupilot_and_wgs84():
    # ArduPilot (default): sphere of radius 6378100 m -> 111318.8 m per degree everywhere.
    _, n, _ = ORIGIN.to_enu(39.3154, -76.5490)
    assert n == pytest.approx(111318.845, abs=0.01)
    # Real ellipsoid at 38 deg: about 111 km.
    _, n84, _ = GeoOrigin(38.3154, -76.549, 10.0, 'wgs84').to_enu(39.3154, -76.5490)
    assert 110_900 < n84 < 111_100


def test_matches_sim_worlds_truth_file_numbers():
    # sim_worlds/truth/runway.yaml (ArduPilot convention): tent x=-0.006 y=80.812 at
    # lat 36.21502595 lon -96.00680007 with home 36.2143, -96.0068.
    home = GeoOrigin(36.2143, -96.0068, 210.0)
    e, n, _ = home.to_enu(36.21502595, -96.00680007)
    assert (e, n) == (pytest.approx(-0.006, abs=0.002), pytest.approx(80.812, abs=0.002))
    lat, lon, _ = home.to_geo(52.568, 92.666)                    # the hiker
    assert (lat, lon) == (pytest.approx(36.21513243, abs=2e-8), pytest.approx(-96.00621469, abs=2e-8))


@pytest.mark.parametrize('conv', ['ardupilot', 'wgs84'])
def test_round_trip_both_conventions(conv):
    o = GeoOrigin(36.2143, -96.0068, 210.0, conv)
    lat, lon, _ = o.to_geo(-123.4, 456.7)
    e, n, _ = o.to_enu(lat, lon)
    assert (e, n) == (pytest.approx(-123.4, abs=1e-6), pytest.approx(456.7, abs=1e-6))


def test_east_shrinks_with_cos_latitude():
    e, _, _ = ORIGIN.to_enu(38.3154, -76.5390)       # 0.01 deg east
    expected = math.radians(0.01) * 6378137.0 * math.cos(math.radians(38.3154))
    assert e == pytest.approx(expected, rel=0.01)


def test_distances_agree():
    lat2, lon2, _ = ORIGIN.to_geo(300.0, 400.0)
    assert local_distance_m(ORIGIN, ORIGIN.lat, ORIGIN.lon, lat2, lon2) == pytest.approx(500.0, abs=1e-3)
    assert haversine_m(ORIGIN.lat, ORIGIN.lon, lat2, lon2) == pytest.approx(500.0, rel=0.005)


def test_parse_home():
    assert parse_home('38.3154,-76.549,10.5,90') == (38.3154, -76.549, 10.5, 90.0)
    assert parse_home(' 1.5 , 2.5 ') == (1.5, 2.5, 0.0, 0.0)
    with pytest.raises(ValueError):
        parse_home('38.3')


def test_angle_helpers():
    assert compass_to_enu_yaw_deg(0.0) == pytest.approx(90.0)     # north = +90 ENU
    assert compass_to_enu_yaw_deg(90.0) == pytest.approx(0.0)     # east = 0 ENU
    assert enu_yaw_to_compass_deg(180.0) == pytest.approx(270.0)  # west
    assert wrap_deg(190.0) == pytest.approx(-170.0)
    assert wrap_deg(-180.0) == pytest.approx(180.0)
    s = math.sin(math.radians(45) / 2)
    c = math.cos(math.radians(45) / 2)
    assert quaternion_to_yaw_deg(0, 0, s, c) == pytest.approx(45.0)
