"""
lattice_bridge/test/test_entities_geo.py -- PURPOSE: prove the two map-shape builders: build_area_geo (search
areas and the flight boundary as closed, counter-clockwise polygons) and build_drop_point (a geo point at the
release position), including the shape Lattice documents (geoShape / geoDetails / TEMPLATE_GEO, no
milView, no expiry-less entities).
"""

from datetime import timedelta

import pytest

from lattice_bridge import constants as C
from lattice_bridge.entities import build_area_geo, build_drop_point, find_problems
from support import MISSION, NOW

SOURCE = NOW - timedelta(minutes=3)
EXPIRY = NOW + timedelta(hours=12)
BOX = [(36.21470424, -96.0071897), (36.21470424, -96.00585359), (36.21560256, -96.00585359),
       (36.21560256, -96.0071897)]                                       # the runway search box, SW SE NE NW


def area(corners=BOX, **overrides):
    kwargs = dict(source_update_time=SOURCE, now=NOW, expiry_time=EXPIRY)
    kwargs.update(overrides)
    return build_area_geo(f'astra-{MISSION}-area-search-box', kwargs.pop('name', 'Search box'), corners, **kwargs)


def drop(kind='person', **overrides):
    kwargs = dict(release_time=SOURCE, now=NOW, expiry_time=EXPIRY)
    kwargs.update(overrides)
    return build_drop_point(f'astra-{MISSION}-drop-{kind}-1', kind, kwargs.pop('lat_deg', 36.2151),
                            kwargs.pop('lon_deg', -96.0062), **kwargs)


# ------------------------------------------------------------------------------------ areas
def test_area_has_the_documented_geo_entity_shape():
    e = area()
    assert e['ontology'] == {'template': 'TEMPLATE_GEO'}
    assert e['geoDetails'] == {'type': 'GEO_TYPE_GENERAL'}
    assert list(e['geoShape']) == ['polygon']
    assert 'milView' not in e and 'location' not in e and 'noExpiry' not in e
    assert e['provenance']['dataType'] == 'mission_area'
    assert e['provenance']['sourceUpdateTime'] == '2026-10-24T15:27:00.000000Z'
    assert e['expiryTime'] == '2026-10-25T03:30:00.000000Z'
    assert e['aliases']['name'] == 'Search box'
    assert find_problems(e, NOW) == []


def test_polygon_ring_is_closed_and_uses_the_documented_nesting():
    ring = area()['geoShape']['polygon']['rings']
    assert len(ring) == 1 and set(ring[0]) == {'positions'}
    positions = ring[0]['positions']
    assert len(positions) == 5 and positions[0] == positions[-1]
    assert positions[1] == {'position': {'latitudeDegrees': 36.21470424, 'longitudeDegrees': -96.00585359}}


def test_polygon_is_counter_clockwise_even_when_given_clockwise():
    cw = area(BOX[::-1])['geoShape']['polygon']['rings'][0]['positions']
    ccw = area(BOX)['geoShape']['polygon']['rings'][0]['positions']
    # same set of corners, the CCW one starts where the input started
    assert {(p['position']['latitudeDegrees'], p['position']['longitudeDegrees']) for p in cw} == set(BOX)
    first = cw[0]['position']
    assert (first['latitudeDegrees'], first['longitudeDegrees']) == BOX[::-1][0]
    from lattice_bridge.geo import ring_signed_area
    for ring in (cw, ccw):
        pts = [(p['position']['latitudeDegrees'], p['position']['longitudeDegrees']) for p in ring][:-1]
        assert ring_signed_area(pts) > 0


def test_flight_boundary_uses_the_control_area_type():
    e = area(geo_type=C.GEO_TYPE_FLIGHT_BOUNDARY, name='Flight boundary', description='fly 45.7-121.9 m AGL')
    assert e['geoDetails'] == {'type': 'GEO_TYPE_CONTROL_AREA'}
    assert e['description'] == 'fly 45.7-121.9 m AGL'
    assert find_problems(e, NOW) == []


@pytest.mark.parametrize('geo_type', ['GEO_TYPE_INVALID', 'general', '', None, 5])
def test_unknown_geo_types_raise(geo_type):
    with pytest.raises(ValueError, match='geo_type'):
        area(geo_type=geo_type)


def test_every_documented_geo_type_is_accepted():
    for geo_type in sorted(C.GEO_TYPES):
        assert area(geo_type=geo_type)['geoDetails']['type'] == geo_type


def test_closed_input_is_not_closed_twice():
    assert len(area(BOX + [BOX[0]])['geoShape']['polygon']['rings'][0]['positions']) == 5


@pytest.mark.parametrize('corners', [
    [], BOX[:2], [(1, 1), (2, 2), (3, 3)], [(1, 1), (1, 3), (2, 1), (2, 2)], BOX[:3] + [(float('nan'), 1.0)],
    BOX[:3] + [(0, 0)], BOX[:3] + [(95, 1)], 'abc', None, BOX[:3] + [(1.0,)],
])
def test_bad_polygons_raise(corners):
    with pytest.raises(ValueError):
        area(corners)


def test_expiry_is_mandatory_and_bounded():
    with pytest.raises(ValueError, match='future'):
        area(expiry_time=NOW)
    with pytest.raises(ValueError, match='days'):
        area(expiry_time=NOW + timedelta(days=45))
    with pytest.raises(TypeError):
        build_area_geo('astra-x-area-a', 'A', BOX, source_update_time=SOURCE, now=NOW)       # no default expiry


def test_area_times_must_be_aware():
    import datetime as dt
    with pytest.raises(ValueError, match='naive'):
        area(source_update_time=dt.datetime(2026, 10, 24, 15, 0, 0))


@pytest.mark.parametrize('name', ['', '  ', 'x' * 81, None, 7])
def test_bad_area_names_raise(name):
    with pytest.raises(ValueError, match='name'):
        area(name=name)


def test_area_retire_update_is_complete():
    e = area(is_live=False)
    assert e['isLive'] is False and find_problems(e, NOW) == []


# ------------------------------------------------------------------------------------ drop points
def test_drop_point_is_a_geo_point_at_the_release_position():
    e = drop('person')
    assert e['ontology'] == {'template': 'TEMPLATE_GEO'}
    assert e['geoDetails'] == {'type': 'GEO_TYPE_GENERAL'}
    assert e['geoShape'] == {'point': {'position': {'latitudeDegrees': 36.2151, 'longitudeDegrees': -96.0062}}}
    assert e['provenance']['dataType'] == 'payload_drop'
    assert e['provenance']['sourceUpdateTime'] == '2026-10-24T15:27:00.000000Z'
    assert e['entityId'] == f'astra-{MISSION}-drop-person-1'
    assert 'Person' in e['aliases']['name'] and 'released' in e['description']
    assert find_problems(e, NOW) == []


def test_tent_drop_point():
    e = drop('tent')
    assert 'Tent' in e['aliases']['name'] and find_problems(e, NOW) == []


def test_drop_altitude_is_optional_and_never_invented():
    assert set(drop()['geoShape']['point']['position']) == {'latitudeDegrees', 'longitudeDegrees'}
    assert drop(alt_hae_m=240.5)['geoShape']['point']['position']['altitudeHaeMeters'] == 240.5


@pytest.mark.parametrize('lat, lon', [(0, 0), (float('nan'), 1.0), (95.0, 1.0), (None, None)])
def test_bad_drop_positions_raise(lat, lon):
    with pytest.raises(ValueError):
        drop(lat_deg=lat, lon_deg=lon)


def test_bad_drop_kind_raises():
    with pytest.raises(ValueError, match='kind'):
        build_drop_point('astra-x-drop-dog-1', 'dog', 36.2, -96.0, release_time=SOURCE, now=NOW, expiry_time=EXPIRY)
