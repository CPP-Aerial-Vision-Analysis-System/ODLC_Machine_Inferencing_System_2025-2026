"""
lattice_bridge/test/test_time_handling.py -- PURPOSE: the time rules, checked across the WHOLE library in one
place: a naive datetime is refused by every function that takes a time; time zones other than UTC are accepted
and converted (the same instant gives the same output); every time that ends up in an entity is UTC text with a
Z and six fraction digits; and expiry/sourceUpdateTime semantics (expiry in the future and under 30 days,
sourceUpdateTime = the measurement time, never the publish time) hold for every builder.
"""

import re
from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge.clustering import TargetClusterer
from lattice_bridge.entities import (
    build_area_geo, build_drop_point, build_target_track, build_uav_asset, find_problems,
)
from lattice_bridge.ids import make_mission_id
from lattice_bridge.sinks.jsonl_sink import JsonlSink
from support import MISSION, NOW

TIME_TEXT = re.compile(r'[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}\.[0-9]{6}Z')
BOX = [(36.2147, -96.0072), (36.2147, -96.0059), (36.2156, -96.0059), (36.2156, -96.0072)]
NAIVE = datetime(2026, 10, 24, 15, 29, 0)
PLUS_5_30 = timezone(timedelta(hours=5, minutes=30))
MINUS_8 = timezone(timedelta(hours=-8))


# The times every builder is called with unless a test overrides one of them.
BASE_TIMES = {
    'uav': dict(fix_time=NOW - timedelta(seconds=1), now=NOW, created_time=NOW - timedelta(minutes=1),
                expiry_time=NOW + timedelta(minutes=2)),
    'track': dict(measured_time=NOW - timedelta(seconds=5), now=NOW, expiry_time=NOW + timedelta(hours=8),
                  first_seen_time=NOW - timedelta(minutes=1)),
    'area': dict(source_update_time=NOW - timedelta(minutes=1), now=NOW, expiry_time=NOW + timedelta(hours=8)),
    'drop': dict(release_time=NOW - timedelta(minutes=1), now=NOW, expiry_time=NOW + timedelta(hours=8)),
}


def call_uav(**times):
    return build_uav_asset('astra-x-uav', 36.2, -96.0, **{**BASE_TIMES['uav'], **times})


def call_track(**times):
    return build_target_track('astra-x-tgt-person-1', 'person', 36.2, -96.0, 0.9, **{**BASE_TIMES['track'], **times})


def call_area(**times):
    return build_area_geo('astra-x-area-a', 'A', BOX, **{**BASE_TIMES['area'], **times})


def call_drop(**times):
    return build_drop_point('astra-x-drop-tent-1', 'tent', 36.2, -96.0, **{**BASE_TIMES['drop'], **times})


BUILDERS = {
    'uav': (call_uav, list(BASE_TIMES['uav'])),
    'track': (call_track, list(BASE_TIMES['track'])),
    'area': (call_area, list(BASE_TIMES['area'])),
    'drop': (call_drop, list(BASE_TIMES['drop'])),
}


@pytest.mark.parametrize('builder, field', [(b, f) for b, (_, fields) in BUILDERS.items() for f in fields])
def test_every_builder_refuses_a_naive_datetime_in_every_time_argument(builder, field):
    function = BUILDERS[builder][0]
    with pytest.raises(ValueError, match='naive'):
        function(**{field: NAIVE})


@pytest.mark.parametrize('builder, field', [(b, f) for b, (_, fields) in BUILDERS.items() for f in fields])
@pytest.mark.parametrize('junk', ['2026-10-24T15:29:00Z', 1761319800, 1761319800.5, object(), (2026, 10, 24)])
def test_every_builder_refuses_non_datetime_time_arguments(builder, field, junk):
    function = BUILDERS[builder][0]
    with pytest.raises(ValueError):
        function(**{field: junk})


def test_other_clusterer_and_helper_entry_points_refuse_naive_times():
    with pytest.raises(ValueError, match='naive'):
        TargetClusterer(MISSION).feed('person', 36.2, -96.0, 0.9, NAIVE)
    with pytest.raises(ValueError, match='naive'):
        make_mission_id(NAIVE)
    with pytest.raises(ValueError, match='naive'):
        find_problems({}, NAIVE)


def test_the_sink_refuses_a_naive_time_without_raising(tmp_path):
    sink = JsonlSink(tmp_path / 'r.jsonl')
    assert sink.write('note', 'x', NAIVE) is False
    sink.close()


@pytest.mark.parametrize('zone', [PLUS_5_30, MINUS_8, timezone.utc])
@pytest.mark.parametrize('builder', sorted(BUILDERS))
def test_the_same_instant_in_another_zone_gives_the_identical_entity(builder, zone):
    function = BUILDERS[builder][0]
    shifted = {key: value.astimezone(zone) for key, value in BASE_TIMES[builder].items()}
    assert function(**shifted) == function()


def _times_in(value, found=None):
    """Every string stored under a key that ends in 'Time', anywhere in the entity."""
    found = [] if found is None else found
    if isinstance(value, dict):
        for key, item in value.items():
            if key.endswith('Time'):
                found.append((key, item))
            _times_in(item, found)
    elif isinstance(value, list):
        for item in value:
            _times_in(item, found)
    return found


@pytest.mark.parametrize('builder', sorted(BUILDERS))
@pytest.mark.parametrize('zone', [PLUS_5_30, MINUS_8])
def test_every_time_in_the_output_is_utc_text_with_six_digits_and_z(builder, zone):
    function = BUILDERS[builder][0]
    base = {'uav': dict(fix_time=NOW - timedelta(seconds=1, microseconds=7), now=NOW),
            'track': dict(measured_time=NOW - timedelta(seconds=5, microseconds=7), now=NOW),
            'area': dict(source_update_time=NOW - timedelta(seconds=5, microseconds=7), now=NOW),
            'drop': dict(release_time=NOW - timedelta(seconds=5, microseconds=7), now=NOW)}[builder]
    entity = function(**{k: v.astimezone(zone) for k, v in base.items()})
    times = _times_in(entity)
    assert len(times) >= 2
    for key, text in times:
        assert TIME_TEXT.fullmatch(text), (key, text)


def test_microseconds_survive():
    entity = call_track(measured_time=NOW - timedelta(seconds=2, microseconds=123456))
    assert entity['provenance']['sourceUpdateTime'] == '2026-10-24T15:29:57.876544Z'


def test_source_update_time_is_the_measurement_time_in_every_builder():
    assert call_uav(fix_time=NOW - timedelta(seconds=30))['provenance']['sourceUpdateTime'] == '2026-10-24T15:29:30.000000Z'
    assert call_track(measured_time=NOW - timedelta(hours=1))['provenance']['sourceUpdateTime'] == '2026-10-24T14:30:00.000000Z'
    assert call_area(source_update_time=NOW - timedelta(days=2))['provenance']['sourceUpdateTime'] == '2026-10-22T15:30:00.000000Z'
    assert call_drop(release_time=NOW - timedelta(minutes=5))['provenance']['sourceUpdateTime'] == '2026-10-24T15:25:00.000000Z'


@pytest.mark.parametrize('builder', sorted(BUILDERS))
def test_expiry_must_be_in_the_future_and_under_30_days_for_every_builder(builder):
    function = BUILDERS[builder][0]
    for bad in (NOW, NOW - timedelta(seconds=1), NOW - timedelta(days=1), NOW + timedelta(days=30),
                NOW + timedelta(days=29, seconds=1), NOW + timedelta(days=365)):
        with pytest.raises(ValueError):
            function(expiry_time=bad)
    for good in (NOW + timedelta(seconds=1), NOW + timedelta(days=29)):
        assert find_problems(function(expiry_time=good), NOW) == []


@pytest.mark.parametrize('builder', sorted(BUILDERS))
def test_a_time_in_the_future_is_refused_beyond_the_clock_skew_allowance(builder):
    function, fields = BUILDERS[builder]
    field = fields[0]
    with pytest.raises(ValueError, match='future'):
        function(**{field: NOW + timedelta(minutes=10)})
    assert function(**{field: NOW + timedelta(seconds=30)})


def test_stale_data_keeps_its_true_time_even_if_very_old():
    entity = call_track(measured_time=NOW - timedelta(days=3))
    assert entity['provenance']['sourceUpdateTime'] == '2026-10-21T15:30:00.000000Z'
    assert find_problems(entity, NOW) == []


def test_leap_day_and_year_ends_format_correctly():
    moment = datetime(2028, 2, 29, 23, 59, 59, 999999, tzinfo=timezone.utc)
    entity = build_target_track('astra-x-tgt-tent-1', 'tent', 36.2, -96.0, 0.5, measured_time=moment, now=moment,
                                expiry_time=moment + timedelta(hours=1))
    assert entity['provenance']['sourceUpdateTime'] == '2028-02-29T23:59:59.999999Z'
    assert entity['expiryTime'] == '2028-03-01T00:59:59.999999Z'
