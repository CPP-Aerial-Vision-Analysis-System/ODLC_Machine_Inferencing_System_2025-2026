"""
sim_worlds/test/test_geo_survey.py -- PURPOSE: unit tests of the GPS <-> metres conversions (sim_worlds.geo) and
of the camera footprint / survey lane planner (sim_worlds.survey).
"""

import math

import pytest

from sim_worlds import geo, survey

HOME = geo.Home(36.2143, -96.0068, 210.0, 0.0)


def test_home_parse_and_string():
    h = geo.Home.parse('36.2143,-96.0068,210,0')
    assert (h.lat, h.lon, h.alt, h.heading) == (36.2143, -96.0068, 210.0, 0.0)
    assert h.sitl_string() == '36.2143000,-96.0068000,210.0,0'
    assert geo.Home.parse(h.sitl_string()) == h
    with pytest.raises(ValueError):
        geo.Home.parse('1,2')
    with pytest.raises(ValueError):
        geo.Home.parse('100,0,0,0')


def test_ardupilot_convention_round_trip():
    for x, y in ((0.0, 0.0), (100.0, 0.0), (0.0, 250.0), (-480.0, 700.0), (370.0, -280.0)):
        lat, lon = geo.enu_to_latlon(x, y, HOME)
        x2, y2 = geo.latlon_to_enu(lat, lon, HOME)
        assert math.hypot(x2 - x, y2 - y) < 1e-6


def test_ardupilot_scale_matches_location_cpp():
    # 1e-7 degree of latitude is LOCATION_SCALING_FACTOR metres in ArduPilot
    lat, _ = geo.ardupilot_offset(0.0, 0.0, geo.LOCATION_SCALING_FACTOR, 0.0)
    assert lat == pytest.approx(1e-7, rel=1e-9)
    # 100 m east at the home latitude
    _, lon = geo.ardupilot_offset(HOME.lat, HOME.lon, 0.0, 100.0)
    assert (lon - HOME.lon) * geo.ARDUPILOT_M_PER_DEG * math.cos(math.radians(HOME.lat)) == pytest.approx(100.0, abs=1e-3)


def test_wgs84_round_trip_and_altitude():
    for x, y, z in ((0, 0, 0), (300.0, -200.0, 0.5), (-600.0, 750.0, 30.0)):
        lat, lon, alt = geo.enu_to_wgs84(x, y, z, HOME)
        x2, y2, z2 = geo.wgs84_to_enu(lat, lon, alt, HOME)
        assert math.dist((x, y, z), (x2, y2, z2)) < 1e-3
    assert geo.enu_to_wgs84(0, 0, 0, HOME)[2] == pytest.approx(210.0, abs=1e-3)


def test_conventions_differ_by_at_most_about_a_metre_in_our_worlds():
    worst = 0.0
    for x, y in ((370.0, -280.0), (-670.0, 430.0), (440.0, 790.0)):
        a = geo.enu_to_latlon(x, y, HOME, 'ardupilot')
        w = geo.enu_to_latlon(x, y, HOME, 'wgs84')
        xa, ya = geo.latlon_to_enu(*w, HOME, 'ardupilot')       # where ArduPilot would put the WGS84 point
        worst = max(worst, math.hypot(xa - x, ya - y))
        assert a != w
    assert worst < 3.0


def test_heading_yaw():
    assert geo.heading_to_yaw(0.0) == pytest.approx(math.pi / 2)
    assert geo.heading_to_yaw(90.0) == pytest.approx(0.0)
    assert geo.yaw_to_heading(math.pi / 2) == pytest.approx(0.0)
    assert geo.yaw_to_heading(geo.heading_to_yaw(20.0)) == pytest.approx(20.0)


def test_polygon_helpers():
    sq = [(0, 0), (10, 0), (10, 10), (0, 10)]
    assert geo.point_in_polygon(5, 5, sq)
    assert not geo.point_in_polygon(15, 5, sq)
    assert geo.distance_to_polygon_edge(5, 5, sq) == pytest.approx(5.0)
    assert geo.inside_with_margin(5, 5, sq, 4.0) and not geo.inside_with_margin(5, 5, sq, 6.0)


def test_footprint_a8_lens_2k():
    fp = survey.footprint(50.0)
    assert fp.width_m == pytest.approx(2 * 50 * math.tan(1.4137 / 2), rel=1e-9)
    assert fp.width_m == pytest.approx(85.4, abs=0.1)
    assert fp.height_m == pytest.approx(fp.width_m * 1440 / 2560)
    assert fp.gsd_m * 100 == pytest.approx(3.34, abs=0.01)
    with pytest.raises(ValueError):
        survey.footprint(0.0)


def test_rect_helpers():
    r = survey.Rect(10.0, 20.0, 100.0, 40.0, math.pi / 2)
    x, y = r.to_world(10.0, 5.0)
    assert r.to_rect(x, y) == pytest.approx((10.0, 5.0))
    assert r.contains(10.0, 60.0, 9.0) and not r.contains(10.0, 75.0)
    assert len(r.corners()) == 4


def test_plan_lanes_runway_box():
    rect = survey.Rect(25.0, 95.0, 120.0, 100.0, 0.0)
    plan = survey.plan_lanes(rect, 30.0, sidelap=0.3)
    assert plan.n_lanes == 3
    assert plan.spacing_m <= plan.max_spacing_m + 1e-9
    assert len(plan.waypoints) == 6
    # first and last photo strips end exactly at the box edges
    vs = sorted({round(rect.to_rect(*p)[1], 6) for p in plan.waypoints})
    assert vs[0] - plan.footprint.width_m / 2 == pytest.approx(-50.0, abs=1e-6)
    assert vs[-1] + plan.footprint.width_m / 2 == pytest.approx(50.0, abs=1e-6)
    # the start is the corner nearest home (0, 0)
    first = plan.waypoints[0]
    corners = [rect.to_world(u, v) for u in (-60.0, 60.0) for v in (vs[0], vs[-1])]
    assert math.hypot(*first) == pytest.approx(min(math.hypot(*c) for c in corners))
    # lanes alternate direction (lawn-mower)
    (a0, a1), (b0, b1) = plan.lanes[0], plan.lanes[1]
    assert (a1[0] - a0[0]) * (b1[0] - b0[0]) < 0


def test_plan_lanes_single_lane_and_bad_input():
    plan = survey.plan_lanes(survey.Rect(0, 0, 100.0, 20.0), 50.0)
    assert plan.n_lanes == 1 and plan.notes
    with pytest.raises(ValueError):
        survey.plan_lanes(survey.Rect(0, 0, 100.0, 20.0), 50.0, sidelap=1.0)


def test_survey_speed():
    # 30 m: footprint height 28.8 m, 40 % new ground per photo every 2 s -> 5.76 -> 5.5 m/s
    assert survey.survey_speed(30.0) == 5.5
    assert survey.survey_speed(50.0) == 9.5
    assert survey.survey_speed(200.0) == 10.0          # clamped to max_speed
    assert survey.survey_speed(5.0) == 2.0             # clamped to min_speed
