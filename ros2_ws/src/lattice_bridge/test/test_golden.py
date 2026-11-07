"""
lattice_bridge/test/test_golden.py -- PURPOSE: golden-file tests. Each builder is run with fixed inputs and its
output must match, character for character, a complete example request saved in test/golden/*.json. The golden
files were reviewed BY HAND against the Lattice documentation (review notes: test/golden/README.md and
docs/lattice/DATA_CONTRACT.md), so a change that alters what we send to Lattice cannot slip in unnoticed: the
test fails and the person making the change must look at the diff and re-approve it.

Regenerate after an intended change:  LATTICE_UPDATE_GOLDEN=1 python3 -m pytest -q test/test_golden.py
"""

import json
from datetime import timedelta

import pytest

from lattice_bridge import world_areas
from lattice_bridge.entities import (
    build_area_geo, build_drop_point, build_target_track, build_uav_asset, find_problems,
)
from support import GOLDEN, MISSION, NOW, assert_golden, scenarios

DATA = scenarios()
UAV_ID = f'astra-{MISSION}-uav'
EXPIRY_AREA = NOW + timedelta(hours=12)
EXPIRY_TARGET = NOW + timedelta(minutes=10) + timedelta(hours=6)        # planned mission end + 6 h margin
HOME = DATA['runway']['truth']['home']


def _runway_box():
    spec = world_areas.search_areas(DATA['runway']['config'])[0]
    return build_area_geo(f'astra-{MISSION}-area-{spec.name.replace("_", "-")}', 'Search box', spec.corners_latlon,
                          source_update_time=NOW - timedelta(minutes=2), now=NOW, expiry_time=EXPIRY_AREA,
                          description=spec.note)


def _suas_boundary_2():
    spec = [s for s in world_areas.search_areas(DATA['suas_field_real']['config']) if s.name == 'search_boundary_2'][0]
    return build_area_geo(f'astra-{MISSION}-area-search-boundary-2', 'Search boundary 2', spec.corners_latlon,
                          source_update_time=NOW - timedelta(minutes=2), now=NOW, expiry_time=EXPIRY_AREA,
                          description=spec.note)


def _suas_flight_boundary():
    spec = world_areas.flight_boundary(DATA['suas_field_real']['truth'])
    return build_area_geo(f'astra-{MISSION}-area-flight-boundary', 'Flight boundary', spec.corners_latlon,
                          source_update_time=NOW - timedelta(minutes=2), now=NOW, expiry_time=EXPIRY_AREA,
                          geo_type=spec.geo_type, description=spec.note)


hiker = [t for t in DATA['runway']['truth']['targets'] if t['class'] == 'person'][0]
tent = [t for t in DATA['runway']['truth']['targets'] if t['class'] == 'tent'][0]

CASES = {
    'uav_full': lambda: build_uav_asset(
        UAV_ID, HOME['lat'], HOME['lon'], fix_time=NOW - timedelta(seconds=1), now=NOW,
        alt_amsl_m=HOME['alt_msl'] + 49.0, rel_alt_m=49.0, geoid_undulation_m=-29.0, heading_deg=45.0,
        velocity_ned_mps=(3.0, 4.0, -1.0), mode='AUTO', armed=True, battery_percent=87.0, link_age_s=0.8,
        created_time=NOW - timedelta(minutes=10), specific_type='ArduCopter quadrotor'),
    'uav_amsl_as_hae': lambda: build_uav_asset(
        UAV_ID, HOME['lat'], HOME['lon'], fix_time=NOW - timedelta(seconds=1), now=NOW,
        alt_amsl_m=HOME['alt_msl'] + 49.0, rel_alt_m=49.0, heading_deg=90.0, mode='GUIDED', armed=True),
    'uav_minimal': lambda: build_uav_asset(UAV_ID, HOME['lat'], HOME['lon'], fix_time=NOW, now=NOW),
    'uav_goodbye': lambda: build_uav_asset(                    # sent at shutdown with a NEWER time than the last fix
        UAV_ID, HOME['lat'], HOME['lon'], fix_time=NOW, now=NOW, is_live=False, mode='RTL', armed=False),
    'track_person': lambda: build_target_track(
        f'astra-{MISSION}-tgt-person-1', 'person', hiker['lat'], hiker['lon'], 0.95,
        measured_time=NOW - timedelta(seconds=5), now=NOW, expiry_time=EXPIRY_TARGET, sightings=7,
        first_seen_time=NOW - timedelta(seconds=95), name='Person 1 (ASTRA)'),
    'track_tent': lambda: build_target_track(
        f'astra-{MISSION}-tgt-tent-1', 'tent', tent['lat'], tent['lon'], 0.85,
        measured_time=NOW - timedelta(seconds=40), now=NOW, expiry_time=EXPIRY_TARGET, sightings=3,
        first_seen_time=NOW - timedelta(seconds=70), name='Tent 1 (ASTRA)'),
    'area_runway_search_box': _runway_box,
    'area_suas_search_boundary_2': _suas_boundary_2,
    'area_suas_flight_boundary': _suas_flight_boundary,
    'drop_person': lambda: build_drop_point(
        f'astra-{MISSION}-drop-person-1', 'person', hiker['lat'], hiker['lon'], release_time=NOW - timedelta(seconds=20),
        now=NOW, expiry_time=EXPIRY_AREA),
    'drop_tent': lambda: build_drop_point(
        f'astra-{MISSION}-drop-tent-1', 'tent', tent['lat'], tent['lon'], release_time=NOW - timedelta(seconds=60),
        now=NOW, expiry_time=EXPIRY_AREA, alt_hae_m=221.0),
}


@pytest.mark.parametrize('name', sorted(CASES))
def test_builder_output_matches_the_reviewed_golden_file(name):
    assert_golden(name, CASES[name]())


@pytest.mark.parametrize('name', sorted(CASES))
def test_every_golden_file_meets_the_contract(name):
    golden = json.loads((GOLDEN / f'{name}.json').read_text(encoding='utf-8'))
    assert find_problems(golden, NOW) == []


def test_there_are_no_stale_or_unlisted_golden_files():
    on_disk = {p.stem for p in GOLDEN.glob('*.json')}
    assert on_disk == set(CASES), f'golden files and CASES differ: {sorted(on_disk ^ set(CASES))}'


def test_golden_files_are_plain_ascii_json_with_unix_line_endings():
    for path in GOLDEN.glob('*.json'):
        raw = path.read_bytes()
        assert raw.isascii() and b'\r' not in raw and raw.endswith(b'\n'), path.name
        json.loads(raw.decode('ascii'))


def test_the_golden_set_covers_every_shape_and_every_optional_part():
    text = {name: json.dumps(CASES[name]()) for name in CASES}
    joined = ' '.join(text.values())
    for word in ('TEMPLATE_ASSET', 'TEMPLATE_TRACK', 'TEMPLATE_GEO', 'GEO_TYPE_GENERAL', 'GEO_TYPE_CONTROL_AREA',
                 'altitudeHaeMeters', 'altitudeAglMeters', 'attitudeEnu', 'velocityEnu', 'platformActivity',
                 'healthStatus', 'trackQualityWrapper', 'lastMeasurementTime', 'createdTime', 'polygon', 'point',
                 '"isLive": false', 'Person', 'Tent', 'UAV'):
        assert word in joined, word
    assert '"point"' in text['drop_person'] and '"polygon"' in text['area_runway_search_box']
