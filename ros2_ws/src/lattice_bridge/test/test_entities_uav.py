"""
lattice_bridge/test/test_entities_uav.py -- PURPOSE: prove build_uav_asset: the required Lattice components,
the altitude rules (AMSL vs HAE, AGL from relative altitude), heading/velocity/mode/health, the rolling 120 s
expiry, sourceUpdateTime = the FIX time, the shutdown update, and that bad or missing inputs either leave a
component out or raise a clear ValueError, never produce a made-up value.
"""

import copy
import math
from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge import constants as C
from lattice_bridge import entities as E
from lattice_bridge.entities import build_uav_asset, find_problems
from support import MISSION, NOW, rng

UAV_ID = f'astra-{MISSION}-uav'
FIX = NOW - timedelta(seconds=1)


def uav(**overrides):
    kwargs = dict(fix_time=FIX, now=NOW)
    kwargs.update(overrides)
    lat = kwargs.pop('lat_deg', 36.2143)
    lon = kwargs.pop('lon_deg', -96.0068)
    return build_uav_asset(UAV_ID, lat, lon, **kwargs)


def test_minimal_uav_has_exactly_the_required_components_and_nothing_invented():
    e = uav()
    assert set(e) == {'entityId', 'description', 'isLive', 'expiryTime', 'aliases', 'provenance', 'ontology',
                      'location', 'milView'}
    assert e['location'] == {'position': {'latitudeDegrees': 36.2143, 'longitudeDegrees': -96.0068}}
    assert 'health' not in e and 'status' not in e and 'createdTime' not in e
    assert find_problems(e, NOW) == []


def test_required_lattice_components_and_fixed_choices():
    e = uav()
    assert e['entityId'] == UAV_ID and e['isLive'] is True
    assert e['ontology'] == {'template': 'TEMPLATE_ASSET', 'platformType': 'UAV'}
    assert e['milView'] == {'disposition': 'DISPOSITION_FRIENDLY', 'environment': 'ENVIRONMENT_AIR'}
    assert e['aliases']['name'] == 'ASTRA UAV'
    assert e['provenance'] == {'integrationName': 'astra_odlc', 'dataType': 'uav_telemetry',
                               'sourceUpdateTime': '2026-10-24T15:29:59.000000Z'}


def test_source_update_time_is_the_fix_time_not_now():
    e = uav(fix_time=NOW - timedelta(seconds=45))
    assert e['provenance']['sourceUpdateTime'] == '2026-10-24T15:29:15.000000Z'
    assert find_problems(e, NOW) == []                 # stale data keeps its true time and is still valid


def test_expiry_is_now_plus_120_seconds_and_rolls_forward_with_every_update():
    first = uav()
    assert first['expiryTime'] == '2026-10-24T15:32:00.000000Z'
    later = NOW + timedelta(seconds=1)
    second = build_uav_asset(UAV_ID, 36.2143, -96.0068, fix_time=later, now=later)
    assert second['expiryTime'] == '2026-10-24T15:32:01.000000Z'
    assert second['expiryTime'] > first['expiryTime']


def test_explicit_expiry_is_respected_but_checked():
    assert uav(expiry_time=NOW + timedelta(minutes=5))['expiryTime'] == '2026-10-24T15:35:00.000000Z'
    with pytest.raises(ValueError, match='future'):
        uav(expiry_time=NOW)
    with pytest.raises(ValueError, match='future'):
        uav(expiry_time=NOW - timedelta(seconds=1))
    with pytest.raises(ValueError, match='days'):
        uav(expiry_time=NOW + timedelta(days=30))
    with pytest.raises(ValueError, match='days'):
        uav(expiry_time=NOW + C.MAX_EXPIRY + timedelta(seconds=1))
    assert uav(expiry_time=NOW + C.MAX_EXPIRY)['expiryTime']      # exactly our limit is fine


def test_goodbye_update_at_shutdown():
    e = uav(is_live=False)
    assert e['isLive'] is False
    assert find_problems(e, NOW) == []                 # still a complete entity, as Lattice requires
    assert e['expiryTime'] == '2026-10-24T15:32:00.000000Z'


# ------------------------------------------------------------------------------------ altitude
def test_hae_is_amsl_plus_geoid_undulation():
    e = uav(alt_amsl_m=265.0, geoid_undulation_m=-29.0)
    assert e['location']['position']['altitudeHaeMeters'] == 236.0
    assert 'altitude: AMSL used as HAE' not in e['description']
    assert E.hae_from_amsl(100.0, 25.5) == 125.5
    assert E.hae_from_amsl(100.0, -30.0) == 70.0


def test_unconfigured_undulation_publishes_amsl_as_hae_and_says_so():
    e = uav(alt_amsl_m=265.0)
    assert e['location']['position']['altitudeHaeMeters'] == 265.0
    assert E.ALTITUDE_NOTE in e['description']
    assert E.altitude_is_approximate(None) is True


def test_an_explicit_zero_undulation_counts_as_configured():
    e = uav(alt_amsl_m=265.0, geoid_undulation_m=0.0)
    assert e['location']['position']['altitudeHaeMeters'] == 265.0
    assert E.ALTITUDE_NOTE not in e['description']
    assert E.altitude_is_approximate(0.0) is False and E.altitude_is_approximate(-29.0) is False


def test_no_altitude_input_means_no_altitude_and_no_warning_note():
    e = uav()
    assert 'altitudeHaeMeters' not in e['location']['position']
    assert E.ALTITUDE_NOTE not in e['description']


def test_agl_comes_from_relative_altitude():
    e = uav(rel_alt_m=49.1234)
    assert e['location']['position']['altitudeAglMeters'] == 49.123
    assert 'altitudeHaeMeters' not in e['location']['position']      # relative altitude alone says nothing about HAE


def test_warning_texts_explain_the_problem():
    assert 'AMSL' in E.ALTITUDE_WARNING and 'HAE' in E.ALTITUDE_WARNING and 'geoid_undulation_m' in E.ALTITUDE_WARNING
    assert len(E.ALTITUDE_NOTE) < 80


@pytest.mark.parametrize('field, value', [
    ('alt_amsl_m', float('nan')), ('alt_amsl_m', float('inf')), ('alt_amsl_m', 1e9), ('alt_amsl_m', -5000),
    ('alt_amsl_m', 'high'), ('alt_amsl_m', True), ('rel_alt_m', float('nan')), ('rel_alt_m', 1e9),
    ('rel_alt_m', '50'), ('geoid_undulation_m', float('nan')), ('geoid_undulation_m', 500.0),
])
def test_bad_altitude_inputs_raise(field, value):
    extra = {'alt_amsl_m': 100.0} if field == 'geoid_undulation_m' else {}
    with pytest.raises(ValueError, match=field):
        uav(**{field: value}, **extra)


# ------------------------------------------------------------------------------------ heading, speed, mode
def test_heading_becomes_an_enu_quaternion():
    q = uav(heading_deg=45.0)['location']['attitudeEnu']
    assert (q['x'], q['y']) == (0.0, 0.0)
    assert q['z'] == pytest.approx(0.382683, abs=1e-6) and q['w'] == pytest.approx(0.92388, abs=1e-6)
    assert 'attitudeEnu' not in uav()['location']


def test_velocity_is_converted_from_ned_to_enu():
    loc = uav(velocity_ned_mps=(3.0, 4.0, -1.0))['location']          # 3 north, 4 east, 1 UP (down = -1)
    assert loc['velocityEnu'] == {'e': 4.0, 'n': 3.0, 'u': 1.0}
    assert loc['speedMps'] == pytest.approx(math.sqrt(26), abs=1e-3)
    assert uav(velocity_ned_mps=(0, 0, 0))['location']['velocityEnu'] == {'e': 0.0, 'n': 0.0, 'u': 0.0}
    assert '-0.0' not in str(uav(velocity_ned_mps=(0, 0, 0)))


@pytest.mark.parametrize('bad', [(1, 2), (1, 2, 3, 4), 'abc', 5, (1, 2, float('nan')), (1, 2, 'x'), (9999, 0, 0)])
def test_bad_velocity_raises(bad):
    with pytest.raises(ValueError):
        uav(velocity_ned_mps=bad)


@pytest.mark.parametrize('mode, armed, text', [
    ('AUTO', None, 'AUTO'), ('GUIDED', True, 'GUIDED'), ('GUIDED', False, 'GUIDED (disarmed)'),
    (None, False, 'DISARMED'), (None, True, 'ARMED'), ('GUIDED_NOGPS', None, 'GUIDED_NOGPS'),
    ('CMODE(15)', None, 'CMODE(15)'),
])
def test_mode_and_armed_become_platform_activity(mode, armed, text):
    assert uav(mode=mode, armed=armed)['status'] == {'platformActivity': text}


@pytest.mark.parametrize('mode', ['', 'A' * 25, 'AU\nTO', 'AUTO;rm', 5, ['AUTO'], 'ÄUTO'])
def test_bad_mode_raises(mode):
    with pytest.raises(ValueError, match='mode'):
        uav(mode=mode)


@pytest.mark.parametrize('armed', ['yes', 1, 0])
def test_bad_armed_raises(armed):
    with pytest.raises(ValueError, match='armed'):
        uav(mode='AUTO', armed=armed)


# ------------------------------------------------------------------------------------ health
@pytest.mark.parametrize('percent, state', [
    (100, 'HEALTHY'), (30.0, 'HEALTHY'), (29.9, 'WARN'), (15.0, 'WARN'), (14.9, 'FAIL'), (0, 'FAIL'),
])
def test_battery_thresholds(percent, state):
    h = uav(battery_percent=percent)['health']
    assert h['components'][0]['health'] == f'HEALTH_STATUS_{state}'
    assert h['healthStatus'] == f'HEALTH_STATUS_{state}'


@pytest.mark.parametrize('age, state', [
    (0.0, 'HEALTHY'), (3.0, 'HEALTHY'), (3.1, 'WARN'), (10.0, 'WARN'), (10.1, 'FAIL'), (3600, 'FAIL'),
])
def test_link_thresholds(age, state):
    h = uav(link_age_s=age)['health']
    assert h['components'][0]['health'] == f'HEALTH_STATUS_{state}'


def test_health_rolls_up_to_the_worst_component():
    h = uav(battery_percent=90, link_age_s=12)['health']
    assert [c['id'] for c in h['components']] == ['battery-0', 'telemetry-link']
    assert [c['health'] for c in h['components']] == ['HEALTH_STATUS_HEALTHY', 'HEALTH_STATUS_FAIL']
    assert h['healthStatus'] == 'HEALTH_STATUS_FAIL'
    assert uav(battery_percent=20, link_age_s=1)['health']['healthStatus'] == 'HEALTH_STATUS_WARN'


def test_health_components_follow_the_documented_shape():
    comp = uav(battery_percent=87.4)['health']['components'][0]
    assert comp == {'id': 'battery-0', 'name': 'Battery', 'health': 'HEALTH_STATUS_HEALTHY',
                    'messages': [{'status': 'HEALTH_STATUS_HEALTHY', 'message': 'Battery at 87% charge.'}]}


def test_connection_status_is_left_to_lattice():
    h = uav(battery_percent=50, link_age_s=1)['health']
    assert 'connectionStatus' not in h and 'updateTime' not in h


@pytest.mark.parametrize('field, value', [('battery_percent', -1), ('battery_percent', 101),
                                          ('battery_percent', float('nan')), ('battery_percent', 'full'),
                                          ('link_age_s', -0.1), ('link_age_s', float('inf')),
                                          ('link_age_s', None.__class__)])
def test_bad_health_inputs_raise(field, value):
    with pytest.raises(ValueError, match=field):
        uav(**{field: value})


# ------------------------------------------------------------------------------------ position and times
@pytest.mark.parametrize('lat, lon', [(0, 0), (91, 0.5), (-91, 0.5), (1, 181), (1, -181), (float('nan'), 1),
                                      (1, float('inf')), (None, 1), (1, None), ('36.2', '-96.0'), (True, True)])
def test_bad_positions_raise(lat, lon):
    with pytest.raises(ValueError):
        uav(lat_deg=lat, lon_deg=lon)


def test_no_gps_fix_is_refused_with_a_helpful_message():
    with pytest.raises(ValueError, match='no GPS fix'):
        uav(lat_deg=0.0, lon_deg=0.0)


def test_times_must_be_aware_and_not_in_the_future():
    naive = datetime(2026, 10, 24, 15, 29, 59)
    with pytest.raises(ValueError, match='naive'):
        uav(fix_time=naive)
    with pytest.raises(ValueError, match='naive'):
        uav(now=naive)
    with pytest.raises(ValueError, match='naive'):
        uav(expiry_time=datetime(2026, 10, 24, 16, 0, 0))
    with pytest.raises(ValueError, match='naive'):
        uav(created_time=naive)
    with pytest.raises(ValueError, match='future'):
        uav(fix_time=NOW + timedelta(seconds=C.FUTURE_TOLERANCE_S + 1))
    assert uav(fix_time=NOW + timedelta(seconds=C.FUTURE_TOLERANCE_S))      # small clock skew is tolerated
    with pytest.raises(ValueError, match='datetime'):
        uav(fix_time='2026-10-24T15:29:59Z')


def test_created_time_and_specific_type_are_optional_extras():
    e = uav(created_time=NOW - timedelta(minutes=10), specific_type='Iris quadcopter')
    assert e['createdTime'] == '2026-10-24T15:20:00.000000Z'
    assert e['ontology']['specificType'] == 'Iris quadcopter'
    assert find_problems(e, NOW) == []


@pytest.mark.parametrize('entity_id', ['', 'has space', 'slash/id', 'x' * 129, None, 5, 'ünï', 'new\nline'])
def test_bad_entity_ids_raise(entity_id):
    with pytest.raises(ValueError, match='entity_id'):
        build_uav_asset(entity_id, 36.2, -96.0, fix_time=FIX, now=NOW)


@pytest.mark.parametrize('name', ['', '   ', 'x' * 81, 'tab\there', None, 5])
def test_bad_names_raise(name):
    with pytest.raises(ValueError, match='name'):
        uav(name=name)


def test_description_override_and_limits():
    assert uav(description='custom text')['description'] == 'custom text'
    with pytest.raises(ValueError, match='description'):
        uav(description='x' * 301)


# ------------------------------------------------------------------------------------ purity and properties
def test_same_inputs_give_equal_results_and_inputs_are_not_modified():
    kwargs = dict(fix_time=FIX, now=NOW, alt_amsl_m=265.0, rel_alt_m=50.0, heading_deg=10.0,
                  velocity_ned_mps=[1.0, 2.0, 3.0], mode='AUTO', armed=True, battery_percent=80, link_age_s=1.0)
    snapshot = copy.deepcopy(kwargs)
    a = build_uav_asset(UAV_ID, 36.2, -96.0, **kwargs)
    b = build_uav_asset(UAV_ID, 36.2, -96.0, **kwargs)
    assert a == b and a is not b
    assert kwargs == snapshot
    a['location']['position']['latitudeDegrees'] = 0
    assert build_uav_asset(UAV_ID, 36.2, -96.0, **kwargs)['location']['position']['latitudeDegrees'] == 36.2


def test_random_valid_inputs_always_produce_a_valid_entity():
    r = rng(7)
    for _ in range(400):
        now = NOW + timedelta(seconds=r.uniform(0, 86400 * 100))
        kwargs = dict(
            fix_time=now - timedelta(seconds=r.uniform(0, 600)), now=now,
            alt_amsl_m=r.choice([None, r.uniform(-400, 5000)]), rel_alt_m=r.choice([None, r.uniform(-20, 500)]),
            geoid_undulation_m=r.choice([None, r.uniform(-100, 100)]),
            heading_deg=r.choice([None, r.uniform(0, 360)]),
            velocity_ned_mps=r.choice([None, (r.uniform(-30, 30), r.uniform(-30, 30), r.uniform(-10, 10))]),
            mode=r.choice([None, 'AUTO', 'GUIDED', 'RTL', 'LOITER']), armed=r.choice([None, True, False]),
            battery_percent=r.choice([None, r.uniform(0, 100)]), link_age_s=r.choice([None, r.uniform(0, 100)]),
            is_live=r.choice([True, False]),
        )
        if kwargs['alt_amsl_m'] is None:
            kwargs['geoid_undulation_m'] = r.choice([None, 1.0])
        lat, lon = r.uniform(-89, 89) or 1.0, r.uniform(-179, 179) or 1.0
        entity = build_uav_asset(UAV_ID, lat, lon, **kwargs)
        assert find_problems(entity, now) == [], (kwargs, find_problems(entity, now))
        assert entity['provenance']['sourceUpdateTime'].endswith('Z')


def test_random_garbage_raises_value_error_or_works_never_anything_else():
    r = rng(8)
    junk = [None, float('nan'), float('inf'), -1e308, 1e308, '', 'x', [], {}, (), True, 10 ** 30, object(), b'a',
            datetime(2026, 1, 1), NOW]
    for _ in range(500):
        kwargs = {k: r.choice(junk) for k in r.sample(
            ['alt_amsl_m', 'rel_alt_m', 'geoid_undulation_m', 'heading_deg', 'velocity_ned_mps', 'mode', 'armed',
             'battery_percent', 'link_age_s', 'name', 'description', 'is_live', 'expiry_time', 'created_time'],
            r.randint(1, 4))}
        try:
            build_uav_asset(UAV_ID, r.choice([36.2, None, float('nan'), 'x']), r.choice([-96.0, None, 1e9]),
                            fix_time=r.choice([FIX, None, 'x']), now=NOW, **kwargs)
        except ValueError:
            pass


def test_timezone_aware_non_utc_inputs_are_written_as_utc():
    plus2 = timezone(timedelta(hours=2))
    e = build_uav_asset(UAV_ID, 36.2, -96.0, fix_time=datetime(2026, 10, 24, 17, 29, 59, tzinfo=plus2),
                        now=datetime(2026, 10, 24, 17, 30, 0, tzinfo=plus2))
    assert e['provenance']['sourceUpdateTime'] == '2026-10-24T15:29:59.000000Z'
    assert e['expiryTime'] == '2026-10-24T15:32:00.000000Z'
