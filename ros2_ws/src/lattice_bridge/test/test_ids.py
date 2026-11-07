"""
lattice_bridge/test/test_ids.py -- PURPOSE: prove entity ids are stable, distinct, never reused by a later
mission, readable back, and safe (no way to smuggle odd characters into an id).
"""

from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge import ids
from support import MISSION, NOW, rng

PLUS_2 = timezone(timedelta(hours=2))


def test_mission_id_with_and_without_label():
    assert ids.make_mission_id(NOW, 'runway') == 'runway20261024t153000z'
    assert ids.make_mission_id(NOW) == 'm20261024t153000z'
    assert ids.make_mission_id(NOW, 'suas_field_real') == 'suasfieldrea20261024t153000z'


def test_mission_id_cleans_the_label():
    assert ids.make_mission_id(NOW, 'Run way!!') == 'runway20261024t153000z'
    assert ids.make_mission_id(NOW, '2026 test') == 'm2026test20261024t153000z'          # must start with a letter
    assert ids.make_mission_id(NOW, '###') == 'm20261024t153000z'
    assert ids.make_mission_id(NOW, 'ä-ö-ü') == 'm20261024t153000z'                   # non-ASCII is dropped
    assert ids.make_mission_id(NOW, 'a' * 50) == 'a' * 12 + '20261024t153000z'


def test_mission_ids_differ_when_missions_start_seconds_apart():
    """A simulator restart within the same minute must not reuse the old mission's ids."""
    first = ids.make_mission_id(NOW, 'runway')
    second = ids.make_mission_id(NOW + timedelta(seconds=7), 'runway')
    assert first != second
    assert ids.uav_entity_id(first) != ids.uav_entity_id(second)


def test_mission_id_uses_utc_whatever_the_zone():
    local = datetime(2026, 10, 24, 17, 30, 0, tzinfo=PLUS_2)
    assert ids.make_mission_id(local, 'runway') == ids.make_mission_id(NOW, 'runway')


def test_mission_id_rejects_naive_time():
    with pytest.raises(ValueError, match='naive'):
        ids.make_mission_id(datetime(2026, 10, 24, 15, 30, 0))


@pytest.mark.parametrize('bad', ['', 'Runway', '1abc', 'a-b', 'a b', 'a_b', 'x' * 33, None, 5, 'ünï'])
def test_check_mission_id_rejects(bad):
    with pytest.raises(ValueError, match='mission id'):
        ids.check_mission_id(bad)


def test_the_documented_id_shapes():
    assert ids.uav_entity_id(MISSION) == 'astra-runway20261024t153000z-uav'
    assert ids.target_entity_id(MISSION, 'person', 1) == 'astra-runway20261024t153000z-tgt-person-1'
    assert ids.target_entity_id(MISSION, 'tent', 12) == 'astra-runway20261024t153000z-tgt-tent-12'
    assert ids.drop_entity_id(MISSION, 'person') == 'astra-runway20261024t153000z-drop-person-1'
    assert ids.drop_entity_id(MISSION, 'tent', 2) == 'astra-runway20261024t153000z-drop-tent-2'
    assert ids.area_entity_id(MISSION, 'search_boundary_1') == 'astra-runway20261024t153000z-area-search-boundary-1'


@pytest.mark.parametrize('name, slug', [
    ('search_box', 'search-box'), ('Search Boundary 2', 'search-boundary-2'), ('  flight_boundary ', 'flight-boundary'),
    ('A--B__C', 'a-b-c'), ('x', 'x'), ('9', '9'),
])
def test_area_slug(name, slug):
    assert ids.area_slug(name) == slug


@pytest.mark.parametrize('bad', ['', '   ', '---', '!!!', None, 5, 'ö'])
def test_area_slug_rejects_names_without_letters_or_digits(bad):
    with pytest.raises(ValueError):
        ids.area_slug(bad)


def test_area_slug_is_length_limited():
    assert len(ids.area_slug('a' * 500)) == 40


@pytest.mark.parametrize('kind', ['', 'Person', 'dog', None, 1, 'p'])
def test_bad_kinds_are_rejected(kind):
    with pytest.raises(ValueError, match='kind'):
        ids.target_entity_id(MISSION, kind, 1)
    with pytest.raises(ValueError, match='kind'):
        ids.drop_entity_id(MISSION, kind)


@pytest.mark.parametrize('number', [0, -1, 1000000, 1.0, '1', None, True])
def test_bad_numbers_are_rejected(number):
    with pytest.raises(ValueError, match='number'):
        ids.target_entity_id(MISSION, 'person', number)


def test_parse_reads_every_shape_back():
    assert ids.parse_entity_id(ids.uav_entity_id(MISSION)) == ids.ParsedId(MISSION, 'uav')
    assert ids.parse_entity_id(ids.target_entity_id(MISSION, 'tent', 3)) == ids.ParsedId(MISSION, 'target', 'tent', 3)
    assert ids.parse_entity_id(ids.drop_entity_id(MISSION, 'person', 2)) == ids.ParsedId(
        MISSION, 'drop', 'person', 2)
    parsed = ids.parse_entity_id(ids.area_entity_id(MISSION, 'search_boundary_1'))
    assert parsed.role == 'area' and parsed.area == 'search-boundary-1' and parsed.mission == MISSION


@pytest.mark.parametrize('foreign', [
    '', 'astra', 'astra-uav', 'astra-m1-tgt-dog-1', 'astra-m1-tgt-person-0', 'astra-m1-tgt-person-01',
    'astra-m1-tgt-person-1-extra', 'ASTRA-m1-uav', 'astra-M1-uav', 'astra-m1-uav\n', 'astra-m-1-uav',
    'astra-m1-area-', 'astra-m1-area--x', 'astra-m1-area-X', 'hello', None, 5, ['astra-m1-uav'],
])
def test_parse_returns_none_for_foreign_ids(foreign):
    assert ids.parse_entity_id(foreign) is None


def test_every_made_id_parses_back_to_its_inputs():
    r = rng(6)
    for _ in range(300):
        mission = ids.make_mission_id(NOW + timedelta(seconds=r.randrange(10 ** 7)), r.choice(['', 'runway', 'x1', 'SUAS26']))
        kind, number = r.choice(['person', 'tent']), r.randint(1, 999999)
        assert ids.parse_entity_id(ids.target_entity_id(mission, kind, number)) == ids.ParsedId(
            mission, 'target', kind, number)
        assert ids.parse_entity_id(ids.drop_entity_id(mission, kind, number)) == ids.ParsedId(
            mission, 'drop', kind, number)
        assert ids.parse_entity_id(ids.uav_entity_id(mission)) == ids.ParsedId(mission, 'uav')


def test_ids_of_different_things_never_collide():
    made = {ids.uav_entity_id(MISSION)}
    for kind in ('person', 'tent'):
        for n in range(1, 6):
            made.add(ids.target_entity_id(MISSION, kind, n))
            made.add(ids.drop_entity_id(MISSION, kind, n))
    for name in ('search_box', 'flight_boundary', 'search_boundary_1', 'search_boundary_2'):
        made.add(ids.area_entity_id(MISSION, name))
    assert len(made) == 1 + 10 + 10 + 4


def test_ids_pass_the_entity_validator_pattern():
    from lattice_bridge.entities import _ENTITY_ID
    for value in (ids.uav_entity_id(MISSION), ids.target_entity_id(MISSION, 'tent', 999999),
                  ids.area_entity_id(MISSION, 'a' * 100)):
        assert _ENTITY_ID.fullmatch(value) and len(value) <= 128


def test_display_name():
    assert ids.target_display_name('person', 1) == 'Person 1 (ASTRA)'
    assert ids.target_display_name('tent', 2) == 'Tent 2 (ASTRA)'
    with pytest.raises(ValueError):
        ids.target_display_name('dog', 1)
