"""
lattice_bridge/test/test_geo.py -- PURPOSE: prove the shared number checks, the distance and offset maths, the
heading quaternion and the polygon rules (closed, counter-clockwise, simple) that every other module relies on.
"""

import math

import pytest

from lattice_bridge import geo
from support import rng

BAD_NUMBERS = [float('nan'), float('inf'), -float('inf'), None, 'x', '5', [], {}, True, False, 10 ** 400]


# ------------------------------------------------------------------------------------ number checks
@pytest.mark.parametrize('value', BAD_NUMBERS)
def test_is_real_number_rejects_non_numbers(value):
    assert geo.is_real_number(value) is False


@pytest.mark.parametrize('value', [0, 1, -1, 3.5, 1e300, -1e-300, 10 ** 20])
def test_is_real_number_accepts_numbers(value):
    assert geo.is_real_number(value) is True


@pytest.mark.parametrize('value', BAD_NUMBERS)
def test_require_number_raises_a_clear_error(value):
    with pytest.raises(ValueError, match='my_input'):
        geo.require_number('my_input', value)


def test_require_number_ranges_and_conversion():
    assert geo.require_number('x', 5, 0, 10) == 5.0 and isinstance(geo.require_number('x', 5), float)
    with pytest.raises(ValueError, match='>= 0'):
        geo.require_number('x', -0.1, 0, 10)
    with pytest.raises(ValueError, match='<= 10'):
        geo.require_number('x', 10.1, 0, 10)


def test_error_text_never_floods_the_log():
    with pytest.raises(ValueError) as info:
        geo.require_number('x', 'A' * 100000)
    assert len(str(info.value)) < 200


# ------------------------------------------------------------------------------------ positions
def test_require_lat_lon_accepts_real_places():
    assert geo.require_lat_lon(36.2143, -96.0068) == (36.2143, -96.0068)
    assert geo.require_lat_lon(-90, 180) == (-90.0, 180.0)
    assert geo.require_lat_lon(0.0, 10.0) == (0.0, 10.0)           # the equator is fine
    assert geo.require_lat_lon(10.0, 0.0) == (10.0, 0.0)           # so is the prime meridian


@pytest.mark.parametrize('lat, lon', [(90.0001, 0.5), (-90.0001, 0.5), (1, 180.0001), (1, -180.0001),
                                      (float('nan'), 1), (1, float('nan')), (float('inf'), 1), (1, float('-inf')),
                                      (None, 1), (1, None), ('36', '-96'), (True, 1)])
def test_require_lat_lon_rejects_bad_values(lat, lon):
    with pytest.raises(ValueError):
        geo.require_lat_lon(lat, lon)


def test_zero_zero_means_no_gps_fix():
    for pair in [(0, 0), (0.0, -0.0), (1e-9, 1e-9)]:
        with pytest.raises(ValueError, match='no GPS fix'):
            geo.require_lat_lon(*pair)


def test_error_message_names_the_input():
    with pytest.raises(ValueError, match='UAV position latitude'):
        geo.require_lat_lon(200, 1, 'UAV position')


# ------------------------------------------------------------------------------------ distances
def test_haversine_known_values():
    assert geo.haversine_m(10, 20, 10, 20) == 0.0
    assert geo.haversine_m(0.0, 0.0, 1.0, 0.0) == pytest.approx(111195.0, rel=1e-3)      # 1 degree of latitude
    assert geo.haversine_m(36.2143, -96.0068, 36.2152, -96.0068) == pytest.approx(100.0, rel=0.01)
    assert geo.haversine_m(0.0, 179.5, 0.0, -179.5) == pytest.approx(111195.0, rel=1e-3)  # across the date line


def test_haversine_is_symmetric_and_never_negative():
    r = rng(2)
    for _ in range(300):
        a = (r.uniform(-89, 89), r.uniform(-180, 180))
        b = (r.uniform(-89, 89), r.uniform(-180, 180))
        d1, d2 = geo.haversine_m(*a, *b), geo.haversine_m(*b, *a)
        assert d1 == pytest.approx(d2) and d1 >= 0.0


def test_offset_agrees_with_haversine_and_round_trips():
    r = rng(3)
    for _ in range(300):
        lat, lon = r.uniform(-70, 70), r.uniform(-170, 170)
        east, north = r.uniform(-500, 500), r.uniform(-500, 500)
        lat2, lon2 = geo.offset_latlon(lat, lon, east, north)
        back_e, back_n = geo.latlon_to_enu(lat, lon, lat2, lon2)
        assert back_e == pytest.approx(east, abs=1e-6) and back_n == pytest.approx(north, abs=1e-6)
        assert geo.haversine_m(lat, lon, lat2, lon2) == pytest.approx(math.hypot(east, north), rel=0.01, abs=0.05)


def test_offset_north_of_home_matches_the_world_files():
    """The same arithmetic produced corners_latlon in the truth files: 45 m north of the runway home."""
    lat, lon = geo.offset_latlon(36.2143, -96.0068, 0.0, 45.0)
    assert lat == pytest.approx(36.21470424, abs=2e-8) and lon == pytest.approx(-96.0068)


def test_offset_wraps_longitude():
    _, lon = geo.offset_latlon(0.5, 179.9999, 100.0, 0.0)
    assert -180.0 <= lon < 180.0 and lon < 0
    assert geo.wrap_lon(190) == -170 and geo.wrap_lon(-190) == 170 and geo.wrap_lon(0) == 0


# ------------------------------------------------------------------------------------ heading
@pytest.mark.parametrize('heading, z, w', [
    (90.0, 0.0, 1.0),                         # flying east = no turn from the east axis
    (45.0, 0.382683, 0.92388),                # the Lattice docs' drone example: velocity 50 east + 50 north
    (0.0, 0.707107, 0.707107),                # north = a quarter turn counter-clockwise
    (180.0, -0.707107, 0.707107),
    (270.0, -1.0, 0.0),
    (360.0, 0.707107, 0.707107),              # 360 == 0
])
def test_heading_to_quaternion(heading, z, w):
    q = geo.heading_to_attitude_enu(heading)
    assert q['x'] == 0.0 and q['y'] == 0.0
    assert q['z'] == pytest.approx(z, abs=1e-6) and q['w'] == pytest.approx(w, abs=1e-6)


def test_heading_quaternion_matches_the_lattice_docs_example_numbers():
    q = geo.heading_to_attitude_enu(45.0)
    assert round(q['z'], 4) == 0.3827 and round(q['w'], 4) == 0.9239      # "z": 0.3827, "w": 0.9239


def test_heading_quaternion_is_always_a_unit_quaternion_without_negative_zero():
    r = rng(4)
    for _ in range(500):
        q = geo.heading_to_attitude_enu(r.uniform(0, 360))
        assert math.sqrt(sum(v * v for v in q.values())) == pytest.approx(1.0, abs=1e-5)
    for heading in (0, 90, 180, 270, 360):
        assert '-0.0' not in str(geo.heading_to_attitude_enu(heading))


def test_heading_360_is_exactly_heading_0_and_w_is_never_negative():
    assert geo.heading_to_attitude_enu(360.0) == geo.heading_to_attitude_enu(0.0)
    r = rng(14)
    for _ in range(500):
        assert geo.heading_to_attitude_enu(r.uniform(0, 360))['w'] >= 0.0
    q = geo.heading_to_attitude_enu(300.0)                    # north-west-ish: yaw -210 is folded to +150
    assert q['z'] == pytest.approx(0.965926, abs=1e-6) and q['w'] == pytest.approx(0.258819, abs=1e-6)


@pytest.mark.parametrize('bad', [-0.001, 360.001, 720, float('nan'), float('inf'), None, '90', True])
def test_heading_rejects_out_of_range(bad):
    with pytest.raises(ValueError):
        geo.heading_to_attitude_enu(bad)


# ------------------------------------------------------------------------------------ polygons
SQUARE_CCW = [(1.0, 1.0), (1.0, 2.0), (2.0, 2.0), (2.0, 1.0)]       # (lat, lon): SW, SE, NE, NW


def test_ring_signed_area_sign():
    assert geo.ring_signed_area(SQUARE_CCW) > 0
    assert geo.ring_signed_area(SQUARE_CCW[::-1]) < 0
    assert geo.ring_signed_area([(1, 1), (2, 2), (3, 3)]) == 0.0
    assert geo.ring_signed_area([(1, 1), (2, 2)]) == 0.0


def test_ccw_input_stays_as_is_and_is_closed():
    ring = geo.clean_polygon_ring(SQUARE_CCW)
    assert ring == SQUARE_CCW + [SQUARE_CCW[0]]


def test_clockwise_input_is_reversed_and_keeps_its_first_corner():
    ring = geo.clean_polygon_ring(SQUARE_CCW[::-1])
    assert ring[0] == ring[-1] == SQUARE_CCW[::-1][0]
    assert geo.ring_signed_area(ring[:-1]) > 0
    assert set(ring) == set(SQUARE_CCW)


def test_already_closed_input_is_not_closed_twice():
    ring = geo.clean_polygon_ring(SQUARE_CCW + [SQUARE_CCW[0]])
    assert len(ring) == 5 and ring[0] == ring[-1]


def test_cleaning_is_idempotent():
    r = rng(5)
    for _ in range(100):
        count = r.randint(3, 8)
        center = (r.uniform(-60, 60), r.uniform(-170, 170))
        angles = sorted(r.uniform(0, 2 * math.pi) for _ in range(count))
        points = [(center[0] + 0.01 * math.sin(a), center[1] + 0.01 * math.cos(a)) for a in angles]
        if r.random() < 0.5:
            points.reverse()
        try:
            once = geo.clean_polygon_ring(points)
        except ValueError:
            continue                                     # a random star-shaped ring may be too thin
        assert geo.clean_polygon_ring(once) == once
        assert geo.ring_signed_area(once[:-1]) > 0


@pytest.mark.parametrize('corners, message', [
    ([(1, 1), (1, 2)], 'at least 3'),
    ([], 'at least 3'),
    ([(1, 1), (2, 2), (3, 3)], 'zero area'),
    ([(1, 1), (1, 3), (2, 1), (2, 2)], 'crosses itself'),                       # a bow tie
    ([(1, 1), (1, 1), (2, 2), (1, 2)], 'same point'),
    ([(1, 1), (1, 2), (float('nan'), 2)], 'finite'),
    ([(1, 1), (1, 2), (0, 0)], 'no GPS fix'),
    ([(1, 1), (1, 2), (95, 3)], 'latitude'),
    ([(1, 1), (1, 2), (2, 'x')], 'finite'),
    ([(1, 1), (1, 2), (2,)], 'pair'),
    ([(1, 1), (1, 2), 5], 'pair'),
    ([(1.0, -170.0), (1.0, 20.0), (2.0, 20.0)], 'antimeridian'),
    (5, 'list'),
])
def test_bad_polygons_raise_clear_errors(corners, message):
    with pytest.raises(ValueError, match=message):
        geo.clean_polygon_ring(corners)


def test_too_many_corners_are_refused():
    ring = [(10 + 0.001 * math.sin(i / 300 * 2 * math.pi), 10 + 0.001 * math.cos(i / 300 * 2 * math.pi))
            for i in range(300)]
    with pytest.raises(ValueError, match='limit'):
        geo.clean_polygon_ring(ring)


def test_concave_but_simple_polygons_are_allowed():
    ell = [(0.1, 0.1), (0.1, 0.4), (0.2, 0.4), (0.2, 0.2), (0.4, 0.2), (0.4, 0.1)]
    assert geo.is_simple_ring(ell) and geo.clean_polygon_ring(ell)


def test_segment_intersection_cases():
    assert geo.segments_intersect((0, 0), (2, 2), (0, 2), (2, 0))          # crossing
    assert not geo.segments_intersect((0, 0), (1, 1), (2, 2), (3, 3))      # collinear, apart
    assert geo.segments_intersect((0, 0), (2, 2), (1, 1), (3, 3))          # collinear, overlapping
    assert geo.segments_intersect((0, 0), (1, 1), (1, 1), (2, 0))          # touching at an end
    assert not geo.segments_intersect((0, 0), (1, 0), (0, 1), (1, 1))      # parallel
