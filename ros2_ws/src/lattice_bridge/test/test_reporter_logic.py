"""
lattice_bridge/test/test_reporter_logic.py -- PURPOSE: prove the DECISIONS of the Jetson-side target reporter
(lattice_bridge/reporter_logic.py) with plain numbers and a fake clock, no ROS involved:

  * the parameter table: every default is valid, bad values are replaced by the default and reported;
  * when a sighting becomes a radio message (new / clearly better / moved), when it does not, and the exact
    edges (N-1 points is quiet, N points is not; 4.8 m is quiet, 5.2 m is not);
  * the limits: one message per interval for ALL targets, K per target (the first is never blocked), a total,
    a bounded target table, a bounded waiting list, the order new > better > moved, a message counter that
    counts and wraps;
  * garbage never makes a message and never raises (NaN, infinity, wrong types, (0, 0), out of range);
  * every message is at most 50 characters and decodes with the ground decoder to what was meant;
  * the whole chain on the REAL simulator worlds: the texts rebuild exactly one track per real target on the
    ground, close to the truth, with a handful of messages.
"""

import math
import random
from datetime import timedelta

import pytest

from lattice_bridge import constants as C
from lattice_bridge import reporter_logic as R
from lattice_bridge.clustering import TargetClusterer
from lattice_bridge.geo import haversine_m, offset_latlon
from lattice_bridge.report_codec import decode_tgt, encode_tgt
from lattice_bridge.reporter_logic import (
    AimSettings, OnceGate, ReporterSettings, TargetReporter, clean_parameters, clean_value,
)
from support import NOW, rng, scenarios

LAT, LON = 36.2150, -96.0068


def at(east_m, north_m=0.0):
    """A position `east_m` / `north_m` metres from the test origin."""
    return offset_latlon(LAT, LON, east_m, north_m)


def fast(**over):
    """A reporter that may send every 0.1 s (the smallest allowed), so tests do not need long fake times."""
    settings = {'min_interval_s': 0.1}
    settings.update(over)
    return TargetReporter(ReporterSettings(**settings))


# ----------------------------------------------------------------------------------------------- the table
def test_the_parameter_table_is_consistent():
    names = [spec.name for spec in R.PARAMETERS]
    assert len(names) == len(set(names))
    assert set(R.PARAM_BY_NAME) == set(names)
    for spec in R.PARAMETERS:
        assert spec.kind in ('float', 'int', 'bool', 'str'), spec.name
        assert spec.group in R.PARAM_GROUPS, spec.name
        assert len(spec.text) > 20 and spec.text.isascii(), spec.name
        value, problem = clean_value(spec, spec.default)
        assert problem == '' and value == spec.default, f'{spec.name}: the default must be valid'
        assert type(value) is type(spec.default), spec.name
    assert set(R.AIM_NAMES) <= set(names)


def test_the_documented_defaults():
    defaults = {spec.name: spec.default for spec in R.PARAMETERS}
    assert defaults['min_confidence'] == 0.0                   # main_controller has no limit of its own
    assert defaults['min_interval_s'] == 2.0                   # one message per 2 s in total
    assert defaults['same_target_radius_m'] == C.CLUSTER_MERGE_RADIUS_M == 12.0
    assert defaults['min_gain_points'] == 5.0 and defaults['min_move_m'] == 5.0
    assert defaults['max_messages_per_target'] == 6
    assert defaults['dry_run'] is False and defaults['follow_main_controller'] is True
    assert defaults['allow_photo_position_fallback'] is False
    # the five aiming settings carry main_controller's own defaults
    assert (defaults['aim_at_detection_box'], defaults['default_camera_pitch_deg'],
            defaults['default_camera_hfov_deg'], defaults['target_ground_height_m'],
            defaults['max_aim_offset_m']) == (True, -90.0, 81.0, 0.0, 150.0)


@pytest.mark.parametrize('name, raw', [
    ('min_confidence', float('nan')), ('min_confidence', float('inf')), ('min_confidence', -0.1),
    ('min_confidence', 1.5), ('min_confidence', 'high'), ('min_confidence', None), ('min_confidence', True),
    ('min_interval_s', 0.0), ('min_interval_s', -1), ('min_interval_s', 601),
    ('max_messages_per_target', 0), ('max_messages_per_target', 2.5), ('max_messages_per_target', True),
    ('max_messages_per_target', 'six'), ('max_targets', 10 ** 6), ('queue_size', 0),
    ('dry_run', 1), ('dry_run', 'maybe'), ('dry_run', None), ('photo_dir', 5), ('photo_dir', None),
    ('target_ground_height_m', float('nan')), ('max_aim_offset_m', float('inf')),
    ('default_camera_pitch_deg', 'down'), ('follow_period_s', -5.0),
])
def test_a_bad_value_becomes_the_default_and_says_so(name, raw):
    spec = R.PARAM_BY_NAME[name]
    value, problem = clean_value(spec, raw)
    assert value == spec.default and type(value) is type(spec.default)
    assert problem and name in problem


@pytest.mark.parametrize('name, raw, expected', [
    ('min_interval_s', 2, 2.0), ('min_gain_points', 5, 5.0), ('max_messages_per_target', 6.0, 6),
    ('dry_run', 'true', True), ('dry_run', ' FALSE ', False), ('dry_run', True, True),
    ('photo_dir', '  /tmp/photos ', '/tmp/photos'), ('min_confidence', 0.4, 0.4), ('min_confidence', 1, 1.0),
    ('default_camera_pitch_deg', -60, -60.0), ('target_ground_height_m', -3, -3.0),
])
def test_good_values_are_accepted_and_given_the_declared_type(name, raw, expected):
    value, problem = clean_value(R.PARAM_BY_NAME[name], raw)
    assert problem == '' and value == expected and type(value) is type(expected)


def test_clean_parameters_fills_missing_values_and_collects_problems():
    values, problems = clean_parameters({'min_confidence': 0.4, 'dry_run': 'perhaps', 'max_targets': 5})
    assert set(values) == {spec.name for spec in R.PARAMETERS}
    assert values['min_confidence'] == 0.4 and values['max_targets'] == 5
    assert values['dry_run'] is False                                     # bad -> default
    assert values['min_interval_s'] == 2.0                                # missing -> default, no complaint
    assert len(problems) == 1 and 'dry_run' in problems[0]
    assert clean_parameters({})[1] == []


def test_reporter_settings_defaults_come_from_the_table_and_refuse_nonsense():
    settings = ReporterSettings()
    assert settings.min_interval_s == 2.0 and settings.max_messages_per_target == 6
    assert ReporterSettings.from_values(clean_parameters({})[0]) == settings
    assert ReporterSettings(min_interval_s=3).min_interval_s == 3.0       # an int becomes the declared float
    assert isinstance(ReporterSettings(max_messages_per_target=4.0).max_messages_per_target, int)
    for bad in ({'min_interval_s': 0}, {'min_interval_s': float('nan')}, {'max_messages_per_target': 0},
                {'max_messages_per_target': 1.5}, {'min_gain_points': 0}, {'same_target_radius_m': -1},
                {'min_confidence': 2}, {'max_targets': 0}, {'start_seq': -1}, {'start_seq': 1000000},
                {'start_seq': True}, {'start_seq': 1.0}):
        with pytest.raises(ValueError):
            ReporterSettings(**bad)


def test_aim_settings_follow_main_controllers_own_rule():
    aim, problems = AimSettings.from_values({})
    assert aim == AimSettings() and problems == []
    aim, problems = AimSettings.from_values({'aim_at_detection_box': False, 'target_ground_height_m': 3,
                                             'default_camera_hfov_deg': 70.5, 'max_aim_offset_m': 99,
                                             'default_camera_pitch_deg': -80})
    assert aim == AimSettings(False, -80.0, 70.5, 3.0, 99.0) and problems == []
    # a number that is not a finite number gives the DEFAULT (like main_controller._number_parameter) ...
    aim, problems = AimSettings.from_values({'target_ground_height_m': float('nan'), 'max_aim_offset_m': 'far',
                                             'default_camera_hfov_deg': None})
    assert aim == AimSettings() and len(problems) == 2
    # ... and the yes/no setting is read as true / false
    assert AimSettings.from_values({'aim_at_detection_box': 0})[0].aim_at_detection_box is False
    assert 'ground_height=3' in AimSettings(target_ground_height_m=3.0).describe()


def test_once_gate_is_true_once_and_bounded():
    gate = OnceGate(limit=3)
    assert [gate.first('a'), gate.first('a'), gate.first('b'), gate.first('c'), gate.first('d')] == \
        [True, False, True, True, False]                    # 'd' does not fit: bounded memory, never True
    roomy = OnceGate()
    assert roomy.first(('error', 'tick')) is True and roomy.first(('error', 'tick')) is False


# ------------------------------------------------------------------------------------- the first message
def test_the_first_sighting_is_a_new_target_with_one_message():
    reporter = TargetReporter()
    offer = reporter.offer('person', 36.214935, -96.006203, 0.95, 0.0)
    assert (offer.action, offer.target) == ('new', 1)
    assert reporter.waiting() == 1 and len(reporter) == 1
    out = reporter.next_message(0.0)
    assert out.text == 'TGT p 36.214935 -96.006203 95 0'              # the documented format, counter 0
    assert (out.kind, out.number, out.why, out.seq, out.messages_for_target) == ('person', 1, 'new', 0, 1)
    assert out.confidence_percent == 95
    assert reporter.next_message(1000.0) is None                      # nothing else is waiting
    assert reporter.sent_total == 1 and reporter.next_seq_number == 1 and reporter.waiting() == 0


def test_a_tent_is_reported_with_the_t_code_and_its_own_numbering():
    reporter = TargetReporter()
    assert reporter.offer('tent', *at(0), 0.85, 0.0).target == 1
    assert reporter.offer('person', *at(0), 0.90, 1.0).target == 1     # same place, other kind: another target
    assert reporter.offer('person', *at(100), 0.90, 2.0).target == 2
    first = reporter.next_message(3.0)
    assert first.text.startswith('TGT t ') and first.kind == 'tent'    # the oldest waiting report goes first


def test_repeating_the_same_sighting_costs_nothing():
    reporter = fast()
    reporter.offer('person', LAT, LON, 0.9, 0)
    reporter.next_message(0)
    for step in range(1, 20):
        offer = reporter.offer('person', LAT, LON, 0.9, step)
        assert offer.action == 'unchanged'
        assert reporter.next_message(step) is None
    assert reporter.sent_total == 1


# ------------------------------------------------------------------------ better / moved, and their edges
def test_better_by_exactly_n_points_is_sent_and_n_minus_one_is_not():
    reporter = fast()
    reporter.offer('person', LAT, LON, 0.80, 0)
    reporter.next_message(0)
    assert reporter.offer('person', LAT, LON, 0.84, 1).action == 'unchanged'      # +4 points
    assert reporter.next_message(10) is None
    offer = reporter.offer('person', LAT, LON, 0.85, 2)                           # +5 points
    assert offer.action == 'improved' and offer.target == 1
    out = reporter.next_message(10)
    assert out.why == 'better' and out.confidence_percent == 85 and out.messages_for_target == 2


def test_the_gain_is_counted_from_what_the_ground_was_told_not_from_the_last_sighting():
    reporter = fast()
    reporter.offer('person', LAT, LON, 0.50, 0)
    reporter.next_message(0)                                                      # the ground knows 50
    for step, conf in enumerate([0.52, 0.54, 0.56], start=1):                     # +2 each: never +5 at once
        assert reporter.offer('person', LAT, LON, conf, step).action in ('unchanged', 'improved')
    assert reporter.waiting() == 1                                                # 56 vs 50 = +6: wanted now
    assert reporter.next_message(10).confidence_percent == 56


def test_a_best_sighting_more_than_m_metres_away_is_sent_again_with_the_same_confidence_class():
    reporter = fast()
    reporter.offer('person', *at(0), 0.80, 0)
    reporter.next_message(0)
    assert reporter.offer('person', *at(4.8), 0.81, 1).action == 'unchanged'      # +1 point, 4.8 m: quiet
    assert reporter.next_message(5) is None
    offer = reporter.offer('person', *at(5.2), 0.82, 2)                           # 5.2 m from the REPORT
    assert offer.action == 'improved' and 'moved' in offer.reason
    out = reporter.next_message(5)
    assert out.why == 'moved'
    assert haversine_m(out.lat_deg, out.lon_deg, *at(5.2)) < 0.01                 # the sighting itself, no mean


def test_a_worse_sighting_never_changes_the_best_not_even_far_away():
    reporter = fast()
    reporter.offer('person', *at(0), 0.90, 0)
    reporter.next_message(0)
    offer = reporter.offer('person', *at(10), 0.60, 1)                            # 10 m away, much less sure
    assert offer.action == 'unchanged' and 'not better' in offer.reason
    assert reporter.next_message(5) is None
    assert haversine_m(reporter.targets()[0].lat_deg, reporter.targets()[0].lon_deg, *at(0)) < 0.01


def test_equal_confidence_keeps_the_older_sighting():
    reporter = fast()
    reporter.offer('person', *at(0), 0.70, 0)
    reporter.offer('person', *at(8), 0.70, 1)
    info = reporter.targets()[0]
    assert haversine_m(info.lat_deg, info.lon_deg, *at(0)) < 0.01 and info.sightings == 2


def test_a_sighting_beyond_the_radius_is_another_target_and_the_nearest_target_wins_inside_it():
    reporter = fast()
    assert reporter.offer('person', *at(0), 0.7, 0).target == 1
    assert reporter.offer('person', *at(11.5), 0.7, 0).action == 'unchanged'      # 11.5 m: still target 1
    assert reporter.offer('person', *at(12.5), 0.7, 0).target == 2                # 12.5 m: a new target
    # a sighting between them goes to the nearer one (target 2, 3 m away; target 1 is 9.5 m away)
    offer = reporter.offer('person', *at(9.5), 0.99, 0)
    assert offer.target == 2


def test_a_better_sighting_replaces_the_report_that_is_still_waiting():
    reporter = fast()
    reporter.offer('person', *at(0), 0.50, 0)
    offer = reporter.offer('person', *at(3), 0.70, 1)
    assert offer.action == 'improved' and reporter.waiting() == 1                 # still ONE report
    out = reporter.next_message(2)
    assert out.why == 'new' and out.confidence_percent == 70
    assert haversine_m(out.lat_deg, out.lon_deg, *at(3)) < 0.01
    assert reporter.next_message(10) is None


def test_a_waiting_move_report_is_cancelled_when_the_next_best_sighting_is_back_near_the_old_place():
    reporter = fast()
    reporter.offer('person', *at(0), 0.80, 0)
    reporter.next_message(0)
    assert reporter.offer('person', *at(8), 0.81, 1).action == 'improved'
    assert reporter.waiting() == 1
    offer = reporter.offer('person', *at(1), 0.82, 2)                             # near the reported place again
    assert offer.action == 'unchanged' and reporter.waiting() == 0
    assert reporter.next_message(10) is None


def test_a_waiting_move_report_becomes_a_better_report_when_the_gain_arrives():
    reporter = fast()
    reporter.offer('person', *at(0), 0.60, 0)
    reporter.next_message(0)
    reporter.offer('person', *at(8), 0.61, 1)                                     # moved
    reporter.offer('person', *at(8), 0.70, 2)                                     # +10 points as well
    assert reporter.targets()[0].waiting == 'better'
    assert reporter.next_message(10).why == 'better'


# --------------------------------------------------------------------------------------------- the limits
def test_a_target_gets_at_most_k_messages_and_the_first_is_never_blocked():
    reporter = fast(max_messages_per_target=2)
    sent, capped = [], 0
    for step, conf in enumerate([0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90]):
        if reporter.offer('person', LAT, LON, conf, step).action == 'capped':
            capped += 1
        out = reporter.next_message(step)
        if out:
            sent.append(out)
    assert [o.confidence_percent for o in sent] == [30, 40] and capped == 5
    assert reporter.waiting() == 0 and reporter.stats['capped'] == 5
    only_one = fast(max_messages_per_target=1)
    only_one.offer('person', LAT, LON, 0.30, 0)
    assert only_one.next_message(0) is not None                                   # K=1 still sends the first
    assert only_one.offer('person', LAT, LON, 0.90, 1).action == 'capped'
    assert only_one.next_message(5) is None


def test_the_total_limit_stops_everything_and_clears_what_waits():
    reporter = fast(max_messages_total=3)
    for index in range(5):
        reporter.offer('person', *at(100 * index), 0.8, index)
    out = [reporter.next_message(float(i)) for i in range(10)]
    assert sum(1 for item in out if item) == 3 and reporter.sent_total == 3
    assert reporter.waiting() == 0                                                # the other two were dropped
    assert reporter.offer('tent', *at(0), 0.9, 20).action == 'capped'
    assert reporter.next_message(100.0) is None


def test_the_target_table_is_bounded_and_known_targets_keep_working():
    reporter = fast(max_targets=2)
    assert reporter.offer('person', *at(0), 0.5, 0).action == 'new'
    assert reporter.offer('person', *at(100), 0.5, 0).action == 'new'
    refused = reporter.offer('person', *at(200), 0.5, 0)
    assert refused.action == 'ignored' and refused.key == 'table_full'
    assert len(reporter) == 2
    assert reporter.offer('person', *at(100), 0.9, 1).action == 'improved'        # still updated
    assert reporter.stats.as_dict()['ignored_by_reason'] == {'table_full': 1}


def test_one_message_per_interval_for_all_targets_together():
    reporter = TargetReporter(ReporterSettings(min_interval_s=2.0))
    for index in range(3):
        reporter.offer('person', *at(100 * index), 0.8, 0)
    assert reporter.next_message(0.0) is not None
    assert reporter.next_message(0.0) is None and reporter.next_message(1.99) is None
    assert reporter.next_message(2.0) is not None
    assert reporter.next_message(3.9) is None
    assert reporter.next_message(4.0) is not None
    assert reporter.next_message(100.0) is None                                   # all three sent


def test_new_goes_before_better_before_moved_and_the_oldest_goes_first():
    reporter = TargetReporter(ReporterSettings(min_interval_s=2.0))
    reporter.offer('person', *at(0), 0.60, 0.0)                                   # A
    reporter.offer('tent', *at(300), 0.60, 0.1)                                   # B
    reporter.offer('person', *at(600), 0.60, 0.2)                                 # C
    sent = [reporter.next_message(t) for t in (0.0, 2.0, 4.0)]
    assert [(o.kind, o.number) for o in sent] == [('person', 1), ('tent', 1), ('person', 2)]
    reporter.offer('person', *at(600), 0.62, 5)                                   # C slightly better: nothing
    reporter.offer('person', *at(0, 9), 0.61, 6)                                  # A: moved 9 m  -> "moved", t=6
    reporter.offer('tent', *at(300), 0.80, 7)                                     # B: +20 points -> "better", t=7
    reporter.offer('person', *at(900), 0.70, 8)                                   # D: new, t=8
    assert reporter.waiting() == 3
    sent = [reporter.next_message(t) for t in (6.0, 8.0, 10.0)]
    assert [(o.why, o.kind, o.number) for o in sent] == [('new', 'person', 3), ('better', 'tent', 1),
                                                         ('moved', 'person', 1)]
    assert [o.seq for o in sent] == [3, 4, 5]


def test_the_clock_going_backwards_makes_it_wait_a_full_interval_from_the_new_time():
    reporter = TargetReporter(ReporterSettings(min_interval_s=2.0))
    reporter.offer('person', *at(0), 0.8, 100.0)
    reporter.offer('person', *at(100), 0.8, 100.0)
    assert reporter.next_message(100.0) is not None
    assert reporter.next_message(50.0) is None                                    # clock jumped back
    assert reporter.next_message(51.9) is None
    assert reporter.next_message(52.0) is not None


def test_the_message_counter_counts_from_zero_and_wraps_after_999999():
    reporter = fast(start_seq=C.TGT_SEQ_MAX - 1)
    for index in range(3):
        reporter.offer('person', *at(100 * index), 0.8, 0)
    seqs = [reporter.next_message(float(t)).seq for t in range(3)]
    assert seqs == [999998, 999999, 0]
    assert reporter.next_seq_number == 1


def test_a_bad_clock_is_a_programming_error():
    reporter = TargetReporter()
    for bad in (None, 'now', float('nan'), float('inf'), True):
        with pytest.raises(ValueError):
            reporter.offer('person', LAT, LON, 0.9, bad)
        with pytest.raises(ValueError):
            reporter.next_message(bad)


# ----------------------------------------------------------------------------------- garbage never gets through
BAD_SIGHTINGS = [
    ('person', float('nan'), LON, 0.9), ('person', LAT, float('nan'), 0.9), ('person', float('inf'), LON, 0.9),
    ('person', LAT, -float('inf'), 0.9), ('person', 91.0, LON, 0.9), ('person', -90.5, LON, 0.9),
    ('person', LAT, 181.0, 0.9), ('person', LAT, -180.5, 0.9), ('person', 0.0, 0.0, 0.9),
    ('person', None, LON, 0.9), ('person', LAT, None, 0.9), ('person', 'north', LON, 0.9),
    ('person', True, LON, 0.9), ('person', [LAT], LON, 0.9), ('person', LAT, {}, 0.9),
    ('boat', LAT, LON, 0.9), (None, LAT, LON, 0.9), (['person'], LAT, LON, 0.9), ({'a': 1}, LAT, LON, 0.9),
    (7, LAT, LON, 0.9), ('PERSON', LAT, LON, 0.9), ('p', LAT, LON, 0.9),
    ('person', LAT, LON, float('nan')), ('person', LAT, LON, float('inf')), ('person', LAT, LON, -0.1),
    ('person', LAT, LON, 1.01), ('person', LAT, LON, 100), ('person', LAT, LON, 0.0), ('person', LAT, LON, None),
    ('person', LAT, LON, '0.9'), ('person', LAT, LON, True), ('person', LAT, LON, [0.9]),
]


@pytest.mark.parametrize('sighting', BAD_SIGHTINGS, ids=lambda s: repr(s)[:40])
def test_a_bad_sighting_is_ignored_and_never_becomes_a_message(sighting):
    reporter = fast()
    offer = reporter.offer(*sighting, 0.0)
    assert offer.action == 'ignored' and offer.reason and offer.key
    assert len(reporter) == 0 and reporter.waiting() == 0
    assert reporter.next_message(100.0) is None
    assert reporter.stats['ignored'] == 1 and reporter.sent_total == 0


def test_a_confidence_below_the_minimum_is_ignored_and_the_minimum_itself_is_accepted():
    reporter = fast(min_confidence=0.40)
    assert reporter.offer('person', LAT, LON, 0.39, 0).key == 'low_confidence'
    assert reporter.offer('person', LAT, LON, 0.40, 0).action == 'new'
    assert [reporter.accepts_score(s) for s in (0.39, 0.40, 1.0, 1.01, 0.0, float('nan'), None, 'x', True)] == \
        [False, True, True, False, False, False, False, False, False]
    assert TargetReporter().accepts_score(0.0001) and not TargetReporter().accepts_score(0.0)


# ------------------------------------------------------------------------------------- the radio text
def test_every_message_is_ascii_at_most_50_characters_and_decodes_to_what_was_meant():
    r = rng(11)
    reporter = TargetReporter(ReporterSettings(min_interval_s=0.1, max_targets=256, max_messages_total=10000))
    known = []                                                      # (kind, lat, lon) of every target so far
    clock, checked = 0.0, 0
    for step in range(5000):
        if step < 256 or r.random() < 0.2:                          # a new random place anywhere on Earth ...
            kind = r.choice(C.TARGET_KINDS)
            lat, lon = r.uniform(-89.9, 89.9), r.uniform(-179.9, 179.9)
            if abs(lat) < 1e-3 and abs(lon) < 1e-3:
                continue
            known.append((kind, lat, lon))
        else:                                                       # ... or another look at a known target
            kind, lat, lon = r.choice(known)
        conf = r.choice([r.random(), r.random(), 1.0, 0.004, 0.995])
        reporter.offer(kind, lat, lon, conf or 0.5, clock)
        clock += 0.2
        out = reporter.next_message(clock)
        if out is None:
            continue
        assert len(out.text) <= C.STATUSTEXT_MAX_CHARS and out.text.isascii()
        decoded = decode_tgt(out.text)
        assert decoded is not None, out.text
        assert (decoded.kind, decoded.confidence_percent, decoded.seq) == (out.kind, out.confidence_percent, out.seq)
        assert decoded.lat_deg == pytest.approx(out.lat_deg, abs=6e-7)
        assert decoded.lon_deg == pytest.approx(out.lon_deg, abs=6e-7)
        checked += 1
    assert checked > 300


def test_percent_is_the_codecs_own_rounding():
    r = rng(5)
    for _ in range(2000):
        conf = r.random()
        assert R.percent(conf) == decode_tgt(encode_tgt('person', LAT, LON, conf, 1)).confidence_percent
    assert [R.percent(c) for c in (0.0, 0.004, 0.006, 0.994, 1.0)] == [0, 0, 1, 99, 100]


# ---------------------------------------------------------------------------- never raise, never grow
NASTY = [None, True, False, 0, 1, -1, 0.0, 0.5, 1.0, 1.5, -0.5, float('nan'), float('inf'), -float('inf'), 'x', '',
         'person', 'tent', [], {}, (1,), b'p', 36.2, -96.0, 91, 181, 10 ** 400, object()]


def test_random_garbage_never_raises_and_the_memory_stays_bounded():
    r = random.Random(99)
    reporter = TargetReporter(ReporterSettings(min_interval_s=0.1, max_targets=8, max_messages_per_target=3,
                                               max_messages_total=500))
    clock = 0.0
    for _ in range(20000):
        kind = r.choice(['person', 'tent', 'person', 'tent'] + NASTY)
        lat = r.choice([LAT + r.uniform(-0.01, 0.01)] * 6 + NASTY)
        lon = r.choice([LON + r.uniform(-0.01, 0.01)] * 6 + NASTY)
        conf = r.choice([r.random()] * 6 + NASTY)
        clock += r.uniform(0.0, 0.3)
        reporter.offer(kind, lat, lon, conf, clock)
        out = reporter.next_message(clock)
        if out is not None:
            assert len(out.text) <= C.STATUSTEXT_MAX_CHARS and decode_tgt(out.text) is not None
        assert len(reporter) <= 8 and reporter.waiting() <= 8
    assert reporter.sent_total <= 500
    assert all(info.messages <= 3 for info in reporter.targets())
    assert len(reporter.stats.as_dict()['ignored_by_reason']) <= 16


def test_the_message_rate_over_a_long_busy_flight():
    """40 targets, 600 sightings over 10 minutes: never two messages closer than the interval, never over K."""
    r = rng(3)
    reporter = TargetReporter(ReporterSettings(min_interval_s=2.0, max_messages_per_target=4, max_targets=64))
    centres = [(r.choice(C.TARGET_KINDS), at(r.uniform(-400, 400), r.uniform(-400, 400))) for _ in range(40)]
    times, clock = [], 0.0
    sightings = sorted((r.uniform(0, 600.0), r.randrange(40)) for _ in range(600))
    index = 0
    while clock < 700.0:
        while index < len(sightings) and sightings[index][0] <= clock:
            kind, (lat, lon) = centres[sightings[index][1]]
            e, n = r.gauss(0, 2.0), r.gauss(0, 2.0)
            reporter.offer(kind, *offset_latlon(lat, lon, e, n), r.uniform(0.3, 0.99), clock)
            index += 1
        out = reporter.next_message(clock)
        if out:
            times.append(clock)
        clock += 0.25
    gaps = [b - a for a, b in zip(times, times[1:])]
    assert gaps and min(gaps) >= 2.0
    assert len(times) <= 700 / 2.0 + 1
    assert max(info.messages for info in reporter.targets()) <= 4
    assert len(reporter) <= 40 + 5                                                # noise stays within the radius


def test_the_summary_line_tells_the_numbers():
    reporter = fast(min_confidence=0.2)
    reporter.offer('person', LAT, LON, 0.9, 0)
    reporter.offer('tent', *at(300), 0.9, 0)
    reporter.offer('tent', LAT, LON, 0.1, 0)
    reporter.next_message(0)
    line = reporter.summary_line()
    for part in ('1 person', '1 tent', 'sent: 1', 'waiting: 1', 'ignored: 1', 'low_confidence 1'):
        assert part in line, line


# --------------------------------------------------- the whole chain on the real simulator worlds
DATA = scenarios()


def world_sightings(world, seed=7):
    """[(time_s, kind, lat, lon, conf, truth_id)]: noisy photo sightings of every real target and decoy person."""
    r = rng(seed)
    truth = DATA[world]['truth']
    real = {t['id'] for t in truth['targets']}
    objects = [(t['id'], 'person' if t['class'] == 'person' else 'tent', t['lat'], t['lon'], (0.55, 0.97), 9)
               for t in truth['targets']]
    objects += [(d['id'], 'person', d['lat'], d['lon'], (0.26, 0.5), 2) for d in truth['person_decoys']]
    rows = []
    for index, (oid, kind, lat, lon, (low, high), photos) in enumerate(objects):
        for k in range(photos):
            east, north = (max(-5.0, min(5.0, r.gauss(0, 2.0))) for _ in range(2))
            rows.append((index * 25.0 + k * 1.0, kind, *offset_latlon(lat, lon, east, north),
                         round(r.uniform(low, high), 2), oid))
    rows.sort()
    return rows, real


@pytest.mark.parametrize('world', sorted(DATA))
def test_the_radio_texts_rebuild_one_track_per_real_target_on_the_ground(world):
    rows, real = world_sightings(world)
    reporter = TargetReporter()                                                   # the defaults, as flown
    ground = TargetClusterer('scenario')                                          # the ground-side clusterer
    messages, clock, index = [], 0.0, 0
    end = rows[-1][0] + 60.0
    while clock <= end:
        while index < len(rows) and rows[index][0] <= clock:
            _, kind, lat, lon, conf, _ = rows[index]
            reporter.offer(kind, lat, lon, conf, rows[index][0])
            index += 1
        out = reporter.next_message(clock)
        if out:
            messages.append(out)
            decoded = decode_tgt(out.text)
            assert decoded is not None and len(out.text) <= 50
            ground.feed(decoded.kind, decoded.lat_deg, decoded.lon_deg, decoded.confidence,
                        NOW + timedelta(seconds=clock))
        clock += 0.5
    tracks = ground.targets()
    truth = DATA[world]['truth']
    for target in truth['targets']:
        kind = 'person' if target['class'] == 'person' else 'tent'
        nearest = min((t for t in tracks if t.kind == kind),
                      key=lambda t: haversine_m(t.lat_deg, t.lon_deg, target['lat'], target['lon']))
        assert haversine_m(nearest.lat_deg, nearest.lon_deg, target['lat'], target['lon']) < 6.0, target['id']
        assert nearest.confidence >= 0.55, target['id']
    # one track per real object (targets + decoy people), nothing doubled
    assert len(tracks) == len(truth['targets']) + len(truth['person_decoys'])
    assert len(messages) <= len(tracks) * 6
    per_target = {}
    for out in messages:
        per_target[(out.kind, out.number)] = per_target.get((out.kind, out.number), 0) + 1
    assert max(per_target.values()) <= 6
    assert len(messages) <= len(rows) * 0.6                                       # far fewer than one per photo
    assert all(m2.seq == m1.seq + 1 for m1, m2 in zip(messages, messages[1:]))


def test_work_per_sighting_is_bounded_by_the_target_table(monkeypatch):
    """No timing assertion (laptops lag): count the work instead. A sighting is compared with <= max_targets."""
    calls = []
    real = R.haversine_m
    monkeypatch.setattr(R, 'haversine_m', lambda *args: calls.append(1) or real(*args))
    reporter = TargetReporter(ReporterSettings(max_targets=64))
    r = rng(2)
    for _ in range(64):
        reporter.offer('person', *at(r.uniform(-900, 900), r.uniform(-900, 900)), 0.5, 0.0)
    assert 0 < len(reporter) <= 64
    for _ in range(500):
        before = len(calls)
        reporter.offer('person', *at(r.uniform(-900, 900), r.uniform(-900, 900)), r.uniform(0.3, 1.0), 1.0)
        assert len(calls) - before <= 64 + 1                       # one distance per known target, plus one
    assert len(reporter) <= 64
