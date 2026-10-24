"""
lattice_bridge/test/test_world_areas.py -- PURPOSE: prove the adapter from the simulator's world files to
latitude/longitude corner lists: the home parser, the rectangle maths (centre, size, yaw), the way config
areas and the truth file's flight boundary are read, and the errors for malformed files. (The comparison with
the real worlds' own corner lists is in test_scenarios.py.)
"""

import math

import pytest

from lattice_bridge import constants as C
from lattice_bridge import world_areas as W
from lattice_bridge.geo import haversine_m, ring_signed_area

HOME = (36.2143, -96.0068)


def test_parse_home_from_text_and_mapping():
    assert W.parse_home('36.2143000,-96.0068000,210.0,0') == HOME
    assert W.parse_home(' 36.2143 , -96.0068 ') == HOME
    assert W.parse_home({'lat': 36.2143, 'lon': -96.0068, 'alt_msl': 210.0}) == HOME


@pytest.mark.parametrize('bad', [None, '', '36.2', 'a,b', '91,0.5', '36.2,-181', 5, [36.2, -96.0], {'lat': 1},
                                 '0,0,210,0', 'nan,1'])
def test_parse_home_rejects_bad_input(bad):
    with pytest.raises(ValueError, match='home'):
        W.parse_home(bad)


def test_rectangle_corners_for_an_east_west_rectangle():
    corners = W.rectangle_corners(*HOME, center_xy=[0.0, 0.0], size=[200.0, 100.0], yaw_rad=0.0)
    assert len(corners) == 4
    sw, se, ne, nw = corners
    assert haversine_m(*sw, *se) == pytest.approx(200.0, rel=0.002)          # along the lanes (east)
    assert haversine_m(*se, *ne) == pytest.approx(100.0, rel=0.002)          # across (north)
    assert sw[0] < nw[0] and sw[1] < se[1]
    assert ring_signed_area(corners) > 0                                     # counter-clockwise


def test_rotating_by_a_quarter_turn_swaps_the_sides():
    corners = W.rectangle_corners(*HOME, [0.0, 0.0], [200.0, 100.0], math.pi / 2)
    side_a = haversine_m(*corners[0], *corners[1])
    side_b = haversine_m(*corners[1], *corners[2])
    assert side_a == pytest.approx(200.0, rel=0.002) and side_b == pytest.approx(100.0, rel=0.002)
    lats = [c[0] for c in corners]
    assert (max(lats) - min(lats)) * 111318 == pytest.approx(200.0, rel=0.01)       # 200 m now runs north
    assert ring_signed_area(corners) > 0


def test_the_centre_is_where_it_says():
    corners = W.rectangle_corners(*HOME, [100.0, -50.0], [40.0, 20.0], 0.7)
    mean_lat = sum(c[0] for c in corners) / 4
    mean_lon = sum(c[1] for c in corners) / 4
    east = (mean_lon - HOME[1]) * 111318.845 * math.cos(math.radians(HOME[0]))
    north = (mean_lat - HOME[0]) * 111318.845
    assert east == pytest.approx(100.0, abs=0.05) and north == pytest.approx(-50.0, abs=0.05)


@pytest.mark.parametrize('center, size, yaw', [
    (None, [10, 10], 0), ([0, 0], None, 0), ([0], [10, 10], 0), ([0, 0], [10], 0), ([0, 0], [0, 10], 0),
    ([0, 0], [10, -5], 0), ([float('nan'), 0], [10, 10], 0), ([0, 0], [10, 10], float('inf')),
    ([0, 0], ['a', 10], 0), ([0, 0], [10, 10], None), ('ab', [10, 10], 0),
])
def test_rectangle_rejects_bad_input(center, size, yaw):
    with pytest.raises(ValueError):
        W.rectangle_corners(*HOME, center, size, yaw)


CONFIG = {
    'home': '36.2143000,-96.0068000,210.0,0',
    'search_areas': [
        {'name': 'search_box', 'note': 'one hiker', 'center': [25.0, 95.0], 'size': [120.0, 100.0], 'yaw': 0.0},
        {'name': 'second', 'center': [0.0, 300.0], 'size': [50.0, 50.0]},
    ],
}


def test_search_areas_from_a_config_dict():
    specs = W.search_areas(CONFIG)
    assert [s.name for s in specs] == ['search_box', 'second']
    assert specs[0].note == 'one hiker' and specs[1].note == ''
    assert all(s.geo_type == C.GEO_TYPE_SEARCH_AREA and len(s.corners_latlon) == 4 for s in specs)
    assert isinstance(specs[0].corners_latlon, tuple)


def test_a_config_without_areas_gives_an_empty_list():
    assert W.search_areas({'home': '36.2,-96.0'}) == []
    assert W.search_areas({'home': '36.2,-96.0', 'search_areas': None}) == []


@pytest.mark.parametrize('config, message', [
    ({}, 'home'), ({'home': 'x'}, 'home'), ({'home': '36.2,-96.0', 'search_areas': 'abc'}, 'list'),
    ({'home': '36.2,-96.0', 'search_areas': [5]}, 'name'),
    ({'home': '36.2,-96.0', 'search_areas': [{'center': [0, 0], 'size': [1, 1]}]}, 'name'),
    ({'home': '36.2,-96.0', 'search_areas': [{'name': 'a', 'size': [1, 1]}]}, 'pair'),
])
def test_malformed_configs_raise_clear_errors(config, message):
    with pytest.raises(ValueError, match=message):
        W.search_areas(config)


TRUTH = {
    'flight_boundary': {'latlon': [[36.22, -96.00], [36.22, -96.01], [36.21, -96.01], [36.21, -96.00]]},
    'altitude_rule_agl_m': {'min': 45.7, 'max': 121.9},
}


def test_flight_boundary_with_altitude_rule():
    spec = W.flight_boundary(TRUTH)
    assert spec.name == 'flight_boundary' and spec.geo_type == C.GEO_TYPE_FLIGHT_BOUNDARY
    assert len(spec.corners_latlon) == 4
    assert spec.note == 'flight boundary: fly 45.7-121.9 m above ground'


def test_flight_boundary_without_rule_or_without_boundary():
    assert W.flight_boundary({'flight_boundary': TRUTH['flight_boundary']}).note == 'flight boundary'
    assert W.flight_boundary({}) is None
    assert W.flight_boundary({'flight_boundary': None}) is None
    assert W.flight_boundary({'flight_boundary': {}}) is None


@pytest.mark.parametrize('truth', [
    {'flight_boundary': {'latlon': []}}, {'flight_boundary': 'x'}, {'flight_boundary': {'altitude': 1}},
    {'flight_boundary': {'latlon': [[1]]}}, {'flight_boundary': {'latlon': [[0, 0]]}},
    {'flight_boundary': {'latlon': [[95, 1]]}}, {'flight_boundary': {'latlon': 5}},
])
def test_malformed_flight_boundaries_raise(truth):
    with pytest.raises(ValueError):
        W.flight_boundary(truth)
