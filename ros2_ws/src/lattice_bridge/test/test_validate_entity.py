"""
lattice_bridge/test/test_validate_entity.py -- PURPOSE: prove the contract checker. Starting from one valid
entity of each shape, every rule of the Lattice docs is broken in turn and the checker must name the
problem. It must also survive garbage of any shape (it is the last gate before a request is sent), accept
every entity our builders make, and never be fooled by NaN, snake_case keys or naive times.
"""

import copy
from datetime import datetime, timedelta

import pytest

from lattice_bridge import constants as C
from lattice_bridge.entities import (
    EntityValidationError, build_area_geo, build_drop_point, build_target_track, build_uav_asset, find_problems,
    validate_entity,
)
from support import NOW, rng

BOX = [(36.2147, -96.0072), (36.2147, -96.0059), (36.2156, -96.0059), (36.2156, -96.0072)]
EXPIRY = NOW + timedelta(hours=1)


def make_uav():
    return build_uav_asset('astra-x-uav', 36.2, -96.0, fix_time=NOW, now=NOW, alt_amsl_m=250.0,
                           geoid_undulation_m=-29.0, heading_deg=10.0, mode='AUTO', battery_percent=80,
                           link_age_s=1.0)


def make_track():
    return build_target_track('astra-x-tgt-person-1', 'person', 36.2, -96.0, 0.9, measured_time=NOW, now=NOW,
                              expiry_time=EXPIRY)


def make_area():
    return build_area_geo('astra-x-area-box', 'Box', BOX, source_update_time=NOW, now=NOW, expiry_time=EXPIRY)


def make_drop():
    return build_drop_point('astra-x-drop-tent-1', 'tent', 36.2, -96.0, release_time=NOW, now=NOW,
                            expiry_time=EXPIRY)


MAKERS = [make_uav, make_track, make_area, make_drop]


def problems_after(maker, mutate):
    entity = maker()
    mutate(entity)
    return find_problems(entity, NOW)


def assert_one_mentions(problems, text):
    assert any(text in p for p in problems), f'{text!r} not in {problems}'


@pytest.mark.parametrize('maker', MAKERS)
def test_every_builder_output_is_valid(maker):
    assert find_problems(maker(), NOW) == []
    validate_entity(maker(), NOW)                                            # returns None, raises nothing


def test_validate_raises_with_the_full_list():
    entity = make_track()
    entity['isLive'] = 'yes'
    del entity['aliases']
    with pytest.raises(EntityValidationError) as info:
        validate_entity(entity, NOW)
    assert isinstance(info.value, ValueError)
    assert len(info.value.problems) == 2
    assert 'isLive' in str(info.value) and 'aliases.name' in str(info.value)


# ------------------------------------------------------------------------------------ the five required parts
@pytest.mark.parametrize('maker', MAKERS)
@pytest.mark.parametrize('path, text', [
    (('entityId',), 'entityId'), (('isLive',), 'isLive'), (('expiryTime',), 'expiryTime'),
    (('provenance',), 'integrationName'), (('aliases',), 'aliases.name'), (('ontology',), 'ontology.template'),
])
def test_missing_required_components_are_reported(maker, path, text):
    def mutate(entity):
        del entity[path[0]]
    assert_one_mentions(problems_after(maker, mutate), text)


def test_missing_source_update_time_and_data_type_are_reported():
    assert_one_mentions(problems_after(make_track, lambda e: e['provenance'].pop('sourceUpdateTime')),
                        'sourceUpdateTime')
    assert_one_mentions(problems_after(make_track, lambda e: e['provenance'].pop('dataType')), 'dataType')
    assert_one_mentions(problems_after(make_track, lambda e: e['provenance'].update(integrationName='other')),
                        'integrationName')


@pytest.mark.parametrize('value', ['yes', 1, 0, None, 'true'])
def test_is_live_must_be_a_real_boolean(value):
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(isLive=value)), 'isLive')


@pytest.mark.parametrize('entity_id', ['', 'a b', 'a/b', 5, None, 'x' * 129])
def test_entity_id_rules(entity_id):
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(entityId=entity_id)), 'entityId')


# ------------------------------------------------------------------------------------ expiry
def test_expiry_must_be_in_the_future():
    past = (NOW - timedelta(seconds=1)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(expiryTime=past)), 'not in the future')
    now_text = NOW.strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(expiryTime=now_text)), 'not in the future')


def test_expiry_must_be_under_30_days_we_allow_29():
    def at(delta):
        return (NOW + delta).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    assert problems_after(make_uav, lambda e: e.update(expiryTime=at(C.MAX_EXPIRY))) == []
    for delta in (C.MAX_EXPIRY + timedelta(seconds=1), timedelta(days=30), timedelta(days=400)):
        assert_one_mentions(problems_after(make_uav, lambda e: e.update(expiryTime=at(delta))), 'days away')


def test_the_expiry_check_uses_the_time_you_pass():
    entity = make_uav()
    assert find_problems(entity, NOW) == []
    assert_one_mentions(find_problems(entity, NOW + timedelta(minutes=5)), 'not in the future')
    assert_one_mentions(find_problems(entity, NOW - timedelta(days=40)), 'days away')


def test_no_expiry_flag_is_refused():
    assert_one_mentions(problems_after(make_area, lambda e: e.update(noExpiry=True)), 'noExpiry')
    assert problems_after(make_area, lambda e: e.update(noExpiry=False)) == []


# ------------------------------------------------------------------------------------ times
@pytest.mark.parametrize('text', ['2026-10-24T16:30:00', '2026-10-24T16:30:00+00:00', '2026-10-24 16:30:00Z',
                                  '2026-10-24T16:30:00.1234567Z', 'tomorrow', '', None, 5, 1761319800.0])
def test_expiry_text_must_be_utc_with_z(text):
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(expiryTime=text)), 'expiryTime')


@pytest.mark.parametrize('key_path', [('provenance', 'sourceUpdateTime'), ('createdTime',),
                                      ('tracked', 'lastMeasurementTime')])
def test_every_time_field_must_be_utc_with_z(key_path):
    def mutate(entity):
        entity.setdefault('createdTime', '2026-10-24T15:00:00.000000Z')
        target = entity
        for key in key_path[:-1]:
            target = target[key]
        target[key_path[-1]] = '2026-10-24T15:00:00+02:00'
    assert_one_mentions(problems_after(make_track, mutate), key_path[-1])


def test_source_update_time_cannot_be_in_the_future():
    future = (NOW + timedelta(seconds=C.FUTURE_TOLERANCE_S + 5)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    assert_one_mentions(problems_after(make_uav, lambda e: e['provenance'].update(sourceUpdateTime=future)),
                        'in the future')
    ok = (NOW + timedelta(seconds=C.FUTURE_TOLERANCE_S - 5)).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
    assert problems_after(make_uav, lambda e: e['provenance'].update(sourceUpdateTime=ok)) == []


def test_naive_now_is_a_programming_error():
    with pytest.raises(ValueError, match='naive'):
        find_problems(make_uav(), datetime(2026, 10, 24, 15, 30, 0))


# ------------------------------------------------------------------------------------ keys and unknown fields
def test_snake_case_keys_are_caught_at_any_depth():
    assert_one_mentions(problems_after(make_uav, lambda e: e['ontology'].update(platform_type='UAV')), 'camelCase')
    assert_one_mentions(problems_after(make_uav, lambda e: e['location']['position'].update(latitude_degrees=1.0)),
                        'camelCase')
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(is_live=True)), 'is_live')


def test_unknown_top_level_keys_are_reported_and_typos_too():
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(entityID='x')), 'unknown top-level keys')
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(confidence=0.9)), 'unknown top-level keys')


def test_non_string_keys_are_caught():
    assert_one_mentions(problems_after(make_uav, lambda e: e.update({5: 'x'})), 'camelCase')


# ------------------------------------------------------------------------------------ enums
@pytest.mark.parametrize('maker, mutate, text', [
    (make_uav, lambda e: e['ontology'].update(template='TEMPLATE_PLANE'), 'ontology.template'),
    (make_uav, lambda e: e['ontology'].update(template='TEMPLATE_INVALID'), 'ontology.template'),
    (make_uav, lambda e: e['milView'].update(disposition='FRIENDLY'), 'disposition'),
    (make_uav, lambda e: e['milView'].update(disposition='DISPOSITION_INVALID'), 'disposition'),
    (make_uav, lambda e: e['milView'].update(environment='ENVIRONMENT_SKY'), 'environment'),
    (make_area, lambda e: e['geoDetails'].update(type='GEO_TYPE_BOX'), 'geoDetails.type'),
    (make_uav, lambda e: e['health'].update(healthStatus='OK'), 'healthStatus'),
    (make_uav, lambda e: e['health'].update(connectionStatus='ONLINE'), 'connectionStatus'),
    (make_uav, lambda e: e['health']['components'][0].update(health='GOOD'), 'health'),
])
def test_invalid_enum_words_are_reported(maker, mutate, text):
    assert_one_mentions(problems_after(maker, mutate), text)


def test_valid_enum_words_pass_everywhere_they_are_allowed():
    for disposition in sorted(C.DISPOSITIONS):
        assert problems_after(make_track, lambda e: e['milView'].update(disposition=disposition)) == []
    for environment in sorted(C.ENVIRONMENTS):
        assert problems_after(make_track, lambda e: e['milView'].update(environment=environment)) == []
    for template in (C.TEMPLATE_ASSET, C.TEMPLATE_TRACK):
        assert problems_after(make_track, lambda e: e['ontology'].update(template=template)) == []
    for state in sorted(C.HEALTH_STATUSES):
        assert problems_after(make_uav, lambda e: e['health'].update(healthStatus=state)) == []
    for state in sorted(C.CONNECTION_STATUSES):
        assert problems_after(make_uav, lambda e: e['health'].update(connectionStatus=state)) == []


# ------------------------------------------------------------------------------------ shape specific parts
@pytest.mark.parametrize('maker', [make_uav, make_track])
def test_assets_and_tracks_need_location_milview_and_platform_type(maker):
    assert_one_mentions(problems_after(maker, lambda e: e.pop('milView')), 'milView')
    assert_one_mentions(problems_after(maker, lambda e: e.pop('location')), 'location.position')
    assert_one_mentions(problems_after(maker, lambda e: e['ontology'].pop('platformType')), 'platformType')
    assert_one_mentions(problems_after(maker, lambda e: e['location']['position'].pop('longitudeDegrees')),
                        'longitudeDegrees')


@pytest.mark.parametrize('maker', [make_area, make_drop])
def test_geo_entities_need_geo_details_and_shape(maker):
    assert_one_mentions(problems_after(maker, lambda e: e.pop('geoDetails')), 'geoDetails.type')
    assert_one_mentions(problems_after(maker, lambda e: e.pop('geoShape')), 'geoShape is required')


def test_geo_shape_must_hold_exactly_one_shape():
    both = problems_after(make_area, lambda e: e['geoShape'].update(point={'position': {
        'latitudeDegrees': 1.0, 'longitudeDegrees': 2.0}}))
    assert_one_mentions(both, 'exactly one')
    assert_one_mentions(problems_after(make_area, lambda e: e.update(geoShape={})), 'exactly one')
    assert_one_mentions(problems_after(make_area, lambda e: e.update(geoShape={'circle': {}})), 'exactly one')


def test_polygon_ring_must_be_closed_and_long_enough():
    def unclose(entity):
        entity['geoShape']['polygon']['rings'][0]['positions'].pop()
    assert_one_mentions(problems_after(make_area, unclose), 'closed')

    def break_close(entity):
        entity['geoShape']['polygon']['rings'][0]['positions'][-1]['position']['latitudeDegrees'] += 0.001
    assert_one_mentions(problems_after(make_area, break_close), 'not closed')

    def shorten(entity):
        entity['geoShape']['polygon']['rings'][0]['positions'] = entity['geoShape']['polygon']['rings'][0][
            'positions'][:3]
    assert_one_mentions(problems_after(make_area, shorten), 'at least')
    assert_one_mentions(problems_after(make_area, lambda e: e['geoShape']['polygon'].update(rings=[])), 'rings')


def test_geo_parts_on_non_geo_entities_are_reported():
    assert_one_mentions(problems_after(make_track, lambda e: e.update(geoDetails={'type': 'GEO_TYPE_GENERAL'})),
                        'TEMPLATE_GEO')


# ------------------------------------------------------------------------------------ numbers
@pytest.mark.parametrize('field, value', [('latitudeDegrees', 90.5), ('latitudeDegrees', -91), ('longitudeDegrees', 181),
                                          ('latitudeDegrees', 'x'), ('longitudeDegrees', None),
                                          ('altitudeHaeMeters', 1e9), ('altitudeAglMeters', -5000)])
def test_position_numbers_are_range_checked(field, value):
    assert_one_mentions(problems_after(make_uav, lambda e: e['location']['position'].update({field: value})), field)


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nan_and_infinity_anywhere_are_reported(value):
    problems = problems_after(make_uav, lambda e: e['location']['position'].update(latitudeDegrees=value))
    assert_one_mentions(problems, 'not plain JSON')
    problems = problems_after(make_uav, lambda e: e['health']['components'][0]['messages'][0].update(message=value))
    assert_one_mentions(problems, 'not plain JSON')


def test_non_json_content_is_reported_not_crashed():
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(description={1, 2})), 'not plain JSON')
    loop = make_uav()
    loop['self'] = loop
    assert_one_mentions(find_problems(loop, NOW), 'not plain JSON')


def test_quaternion_and_track_quality_rules():
    assert_one_mentions(problems_after(make_uav, lambda e: e['location']['attitudeEnu'].update(z=0.9, w=0.9)),
                        'unit quaternion')
    assert_one_mentions(problems_after(make_uav, lambda e: e['location']['attitudeEnu'].pop('w')), 'attitudeEnu')
    for bad in (16, -1, 3.5, '7', True):
        assert_one_mentions(problems_after(make_track, lambda e: e['tracked'].update(trackQualityWrapper=bad)),
                            'trackQualityWrapper')
    assert problems_after(make_track, lambda e: e['tracked'].update(trackQualityWrapper=0)) == []


def test_health_components_need_id_and_name():
    assert_one_mentions(problems_after(make_uav, lambda e: e['health']['components'][0].pop('id')), 'stable id')
    assert_one_mentions(problems_after(make_uav, lambda e: e['health'].update(components='x')), 'components')


def test_aliases_name_and_description_limits():
    assert_one_mentions(problems_after(make_uav, lambda e: e['aliases'].update(name='')), 'aliases.name')
    assert_one_mentions(problems_after(make_uav, lambda e: e['aliases'].update(name='x' * 81)), 'aliases.name')
    assert_one_mentions(problems_after(make_uav, lambda e: e.update(description='x' * 301)), 'description')


# ------------------------------------------------------------------------------------ garbage in
@pytest.mark.parametrize('junk', [None, 5, 'entity', [], [make_uav()], (), set(), b'{}', 3.14, True])
def test_non_dict_input_gets_a_problem_not_a_crash(junk):
    assert find_problems(junk, NOW) == ['entity must be a dict']
    with pytest.raises(EntityValidationError):
        validate_entity(junk, NOW)


def test_empty_dict_lists_the_missing_parts():
    problems = find_problems({}, NOW)
    for text in ('entityId', 'isLive', 'expiryTime', 'integrationName', 'aliases.name', 'ontology.template'):
        assert_one_mentions(problems, text)


def _garbage(r, depth=0):
    kinds = ['int', 'float', 'str', 'none', 'bool', 'list', 'dict', 'nan']
    kind = r.choice(kinds if depth < 3 else kinds[:5])
    if kind == 'int':
        return r.randint(-10 ** 12, 10 ** 12)
    if kind == 'float':
        return r.uniform(-1e12, 1e12)
    if kind == 'nan':
        return r.choice([float('nan'), float('inf'), -float('inf')])
    if kind == 'str':
        return r.choice(['', 'x', '2026-10-24T15:30:00.000000Z', 'DISPOSITION_FRIENDLY', 'TEMPLATE_ASSET', 'a' * 500,
                         'snake_case', '\n', 'Z'])
    if kind == 'none':
        return None
    if kind == 'bool':
        return r.random() < 0.5
    if kind == 'list':
        return [_garbage(r, depth + 1) for _ in range(r.randint(0, 3))]
    keys = ['entityId', 'isLive', 'expiryTime', 'provenance', 'aliases', 'ontology', 'location', 'position',
            'milView', 'geoShape', 'geoDetails', 'health', 'components', 'polygon', 'rings', 'positions', 'point',
            'tracked', 'attitudeEnu', 'template', 'type', 'name', 'latitudeDegrees', 'longitudeDegrees', 'x', 'w',
            'bad_key', 5]
    return {r.choice(keys): _garbage(r, depth + 1) for _ in range(r.randint(0, 5))}


def test_random_garbage_always_returns_a_list_of_strings():
    r = rng(10)
    for _ in range(2000):
        problems = find_problems(_garbage(r), NOW)
        assert isinstance(problems, list) and all(isinstance(p, str) for p in problems)


def test_random_single_mutations_of_valid_entities_never_crash():
    r = rng(11)
    for _ in range(1500):
        entity = copy.deepcopy(r.choice(MAKERS)())
        stack = [entity]
        while stack:                                    # walk to a random place and overwrite it with garbage
            node = stack.pop()
            if isinstance(node, dict) and node:
                key = r.choice(list(node))
                if r.random() < 0.4 or not isinstance(node[key], (dict, list)):
                    node[key] = _garbage(r)
                else:
                    stack.append(node[key])
            elif isinstance(node, list) and node:
                index = r.randrange(len(node))
                if r.random() < 0.4 or not isinstance(node[index], (dict, list)):
                    node[index] = _garbage(r)
                else:
                    stack.append(node[index])
        find_problems(entity, NOW)


def test_the_checker_agrees_with_the_builders_on_random_inputs():
    """Whatever the builders accept, the checker accepts (and the other way round for clear violations)."""
    r = rng(12)
    for _ in range(300):
        lat, lon = r.uniform(-89, 89) or 1.0, r.uniform(-179, 179) or 1.0
        entity = build_uav_asset('astra-x-uav', lat, lon, fix_time=NOW, now=NOW, heading_deg=r.uniform(0, 360))
        assert find_problems(entity, NOW) == []
        entity['location']['position']['latitudeDegrees'] = r.uniform(90.01, 1000)
        assert find_problems(entity, NOW) != []


def test_absurdly_deep_nesting_is_reported_not_crashed():
    entity = make_uav()
    node = entity
    for _ in range(40):
        node['x'] = {}
        node = node['x']
    assert find_problems(entity, NOW) == ['entity is nested too deeply to be a Lattice entity']
    deeper = make_uav()
    node = deeper
    for _ in range(5000):
        node['x'] = {}
        node = node['x']
    assert_one_mentions(find_problems(deeper, NOW), 'not plain JSON')


def test_unhashable_values_where_enum_words_belong_are_reported():
    for bad in ([], {}, [1], {'a': 1}):
        assert_one_mentions(problems_after(make_uav, lambda e: e['milView'].update(disposition=bad)), 'disposition')
        assert_one_mentions(problems_after(make_uav, lambda e: e['ontology'].update(template=bad)), 'template')
        assert_one_mentions(problems_after(make_area, lambda e: e['geoDetails'].update(type=bad)), 'geoDetails.type')
