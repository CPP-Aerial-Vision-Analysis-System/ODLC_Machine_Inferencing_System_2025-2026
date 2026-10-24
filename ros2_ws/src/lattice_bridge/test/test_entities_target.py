"""
lattice_bridge/test/test_entities_target.py -- PURPOSE: prove build_target_track: a detected hiker or tent
becomes a Lattice TRACK at the PROJECTED TARGET position (never the drone's), with the class, the confidence
and the measurement time stored in supported places, and bad input is refused instead of published.
"""

import math
from datetime import datetime, timedelta

import pytest

from lattice_bridge import constants as C
from lattice_bridge.entities import build_target_track, confidence_to_track_quality, find_problems
from support import MISSION, NOW, rng

PERSON_ID = f'astra-{MISSION}-tgt-person-1'
TENT_ID = f'astra-{MISSION}-tgt-tent-1'
MEASURED = NOW - timedelta(seconds=5)
EXPIRY = NOW + timedelta(hours=8)
TARGET_LAT, TARGET_LON = 36.21513243, -96.00621469          # the runway world's hiker


def track(kind='person', conf=0.95, **overrides):
    kwargs = dict(measured_time=MEASURED, now=NOW, expiry_time=EXPIRY)
    kwargs.update(overrides)
    entity_id = kwargs.pop('entity_id', PERSON_ID if kind == 'person' else TENT_ID)
    return build_target_track(entity_id, kind, kwargs.pop('lat_deg', TARGET_LAT),
                              kwargs.pop('lon_deg', TARGET_LON), conf, **kwargs)


def test_person_track_has_every_required_component():
    e = track('person')
    assert e['entityId'] == PERSON_ID and e['isLive'] is True
    assert e['ontology'] == {'template': 'TEMPLATE_TRACK', 'platformType': 'Person'}
    assert e['milView'] == {'disposition': 'DISPOSITION_NEUTRAL', 'environment': 'ENVIRONMENT_SURFACE'}
    assert e['location']['position'] == {'latitudeDegrees': TARGET_LAT, 'longitudeDegrees': TARGET_LON}
    assert e['provenance'] == {'integrationName': 'astra_odlc', 'dataType': 'odlc_detection',
                               'sourceUpdateTime': '2026-10-24T15:29:55.000000Z'}
    assert e['expiryTime'] == '2026-10-24T23:30:00.000000Z'
    assert find_problems(e, NOW) == []


def test_tent_track_differs_only_in_class_words():
    p, t = track('person'), track('tent')
    assert t['ontology']['platformType'] == 'Tent' and t['entityId'] == TENT_ID
    assert t['aliases']['name'].startswith('Tent') and p['aliases']['name'].startswith('Person')
    assert 'Tent detected' in t['description'] and 'Person detected' in p['description']
    assert find_problems(t, NOW) == []


def test_location_is_the_target_not_the_drone():
    """The builder has no drone position input at all; the only position is the target's."""
    e = track(lat_deg=36.5, lon_deg=-96.5)
    assert e['location']['position']['latitudeDegrees'] == 36.5
    assert e['location']['position']['longitudeDegrees'] == -96.5
    assert set(e['location']) == {'position'}


def test_no_altitude_is_invented_for_a_target():
    assert set(track()['location']['position']) == {'latitudeDegrees', 'longitudeDegrees'}
    assert track(alt_hae_m=123.4567)['location']['position']['altitudeHaeMeters'] == 123.457


def test_confidence_lives_in_the_description_and_the_track_quality():
    e = track(conf=0.95, sightings=7)
    assert e['description'] == 'Person detected by ASTRA ODLC: confidence 95%, 7 sighting(s)'
    assert e['tracked']['trackQualityWrapper'] == 14                       # 0.95 * 15 = 14.25
    assert e['tracked']['lastMeasurementTime'] == '2026-10-24T15:29:55.000000Z'


@pytest.mark.parametrize('conf, quality', [(0.0, 0), (0.03, 0), (0.04, 1), (0.5, 8), (0.95, 14), (1.0, 15)])
def test_confidence_to_track_quality(conf, quality):
    assert confidence_to_track_quality(conf) == quality


def test_track_quality_is_monotonic_and_in_range():
    values = [confidence_to_track_quality(i / 1000) for i in range(1001)]
    assert values == sorted(values) and values[0] == 0 and values[-1] == C.TRACK_QUALITY_MAX


@pytest.mark.parametrize('conf', [-0.01, 1.01, float('nan'), float('inf'), None, '0.9', True, [0.9]])
def test_bad_confidence_raises(conf):
    with pytest.raises(ValueError, match='confidence'):
        track(conf=conf)


def test_source_update_time_is_the_measurement_time_never_now():
    old = track(measured_time=NOW - timedelta(minutes=30))
    assert old['provenance']['sourceUpdateTime'] == '2026-10-24T15:00:00.000000Z'
    assert old['tracked']['lastMeasurementTime'] == '2026-10-24T15:00:00.000000Z'
    assert find_problems(old, NOW) == []


def test_created_time_is_the_first_sighting():
    e = track(first_seen_time=NOW - timedelta(minutes=2))
    assert e['createdTime'] == '2026-10-24T15:28:00.000000Z'
    assert 'createdTime' not in track()


def test_expiry_is_mission_end_plus_margin_and_under_30_days():
    mission_end = NOW + timedelta(minutes=10)
    expiry = mission_end + timedelta(seconds=C.TARGET_EXPIRY_MARGIN_S)
    e = track(expiry_time=expiry)
    assert e['expiryTime'] == '2026-10-24T21:40:00.000000Z'
    with pytest.raises(ValueError, match='days'):
        track(expiry_time=NOW + timedelta(days=30))
    with pytest.raises(ValueError, match='future'):
        track(expiry_time=NOW - timedelta(minutes=1))


@pytest.mark.parametrize('kind', ['dog', 'Person', '', None, 'p', 3])
def test_bad_kind_raises(kind):
    with pytest.raises(ValueError, match='kind'):
        build_target_track(PERSON_ID, kind, 36.2, -96.0, 0.9, measured_time=MEASURED, now=NOW, expiry_time=EXPIRY)


@pytest.mark.parametrize('lat, lon', [(0, 0), (float('nan'), -96.0), (36.2, float('inf')), (91, -96.0),
                                      (36.2, -181), (None, None), ('36.2', '-96')])
def test_bad_target_positions_raise(lat, lon):
    with pytest.raises(ValueError):
        track(lat_deg=lat, lon_deg=lon)


@pytest.mark.parametrize('sightings', [0, -1, 1.5, '3', None, True, 2_000_000])
def test_bad_sighting_counts_raise(sightings):
    with pytest.raises(ValueError, match='sightings'):
        track(sightings=sightings)


def test_naive_and_future_times_raise():
    naive = datetime(2026, 10, 24, 15, 29, 55)
    with pytest.raises(ValueError, match='naive'):
        track(measured_time=naive)
    with pytest.raises(ValueError, match='naive'):
        track(first_seen_time=naive)
    with pytest.raises(ValueError, match='future'):
        track(measured_time=NOW + timedelta(minutes=5))


def test_retiring_a_track_keeps_it_complete():
    e = track(is_live=False)
    assert e['isLive'] is False and find_problems(e, NOW) == []


def test_name_and_description_overrides():
    e = track(name='Hiker Randy', description='custom')
    assert e['aliases']['name'] == 'Hiker Randy' and e['description'] == 'custom'
    with pytest.raises(ValueError, match='name'):
        track(name='')


def test_rounding_keeps_about_a_millimetre():
    e = track(lat_deg=36.123456789123, lon_deg=-96.987654321987)
    assert e['location']['position']['latitudeDegrees'] == 36.12345679
    assert e['location']['position']['longitudeDegrees'] == -96.98765432


def test_random_valid_inputs_always_validate():
    r = rng(9)
    for _ in range(400):
        now = NOW + timedelta(seconds=r.uniform(0, 86400 * 50))
        kind = r.choice(C.TARGET_KINDS)
        entity = build_target_track(
            f'astra-{MISSION}-tgt-{kind}-{r.randint(1, 999)}', kind,
            r.uniform(-89, 89) or 1.0, r.uniform(-179, 179) or 1.0, r.random(),
            measured_time=now - timedelta(seconds=r.uniform(0, 7200)), now=now,
            expiry_time=now + timedelta(seconds=r.uniform(1, 86400 * 28)),
            sightings=r.randint(1, 500), first_seen_time=r.choice([None, now - timedelta(hours=1)]),
            alt_hae_m=r.choice([None, r.uniform(-100, 3000)]), is_live=r.choice([True, False]))
        assert find_problems(entity, now) == []
        pos = entity['location']['position']
        assert math.isfinite(pos['latitudeDegrees']) and -90 <= pos['latitudeDegrees'] <= 90
