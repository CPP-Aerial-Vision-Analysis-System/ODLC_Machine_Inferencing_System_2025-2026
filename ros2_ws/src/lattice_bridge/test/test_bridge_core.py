"""
lattice_bridge/test/test_bridge_core.py -- PURPOSE: prove the brain of the bridge by feeding it a script of events
with a fake clock and reading what comes out: the drone is published about once a second and only when there is a
NEWER position; a silent link stops the updates (nothing is repeated or invented) and, if asked, marks the drone not
live; one real target stays one entity however often it is reported, a report that joins two targets merges them; a
release without a known position is never guessed; every entity is checked before it leaves; a broken event or a
broken output cannot stop the bridge; shutdown says goodbye once. No sockets, no threads, no ROS.
"""

import threading
from datetime import datetime, timedelta, timezone

import pytest

from lattice_bridge import constants as C
from lattice_bridge.bridge_core import BridgeCore, CoreSettings, SinkOut, run_loop
from lattice_bridge.events import DroneFix, DropEvent, SourceNotice, TargetSighting
from lattice_bridge.world_areas import AreaSpec

T0 = datetime(2026, 10, 24, 15, 30, 0, tzinfo=timezone.utc)
UAV = 'astra-t1-uav'


def at(seconds):
    return T0 + timedelta(seconds=seconds)


class Collect:
    """An output that remembers what it was given (the core's EntityOut)."""

    def __init__(self):
        self.entities, self.flushed, self.closed, self.accept = [], 0, 0, True

    def publish(self, entity, now):
        self.entities.append(entity)
        return self.accept

    def flush(self, timeout_s=5.0):
        self.flushed += 1

    def close(self):
        self.closed += 1

    def of(self, entity_id):
        return [e for e in self.entities if e['entityId'] == entity_id]


def make_core(**overrides):
    settings = dict(mission_id='t1', geoid_undulation_m=-30.0)
    settings.update(overrides)
    out = Collect()
    core = BridgeCore(CoreSettings(**settings), out)
    core.start(T0)
    return core, out


def fix(seconds, lat=36.2150, lon=-96.0068, **kwargs):
    kwargs.setdefault('alt_amsl_m', 240.0)
    return DroneFix(fix_time=at(seconds), arrival=at(seconds), lat_deg=lat, lon_deg=lon, **kwargs)


def sighting(kind='person', seconds=0, lat=36.2151, lon=-96.0062, confidence=0.9):
    return TargetSighting(kind, lat, lon, confidence, at(seconds))


def position(entity):
    return entity['location']['position']


# ------------------------------------------------------------------------------------------ the drone
def test_the_drone_is_published_when_a_position_arrives_and_not_again_without_a_newer_one():
    core, out = make_core()
    core.tick(at(0))
    assert out.entities == []                                          # nothing known yet: nothing invented
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.tick(at(0.5))
    core.tick(at(1))
    assert len(out.of(UAV)) == 1                                       # same position, still the same update
    core.handle(fix(1), at(1))
    core.tick(at(1))
    assert len(out.of(UAV)) == 2 and core.stats['drone_published'] == 2


def test_updates_are_spaced_by_the_configured_interval():
    core, out = make_core(drone_interval_s=2.0)
    for second in range(0, 7):
        core.handle(fix(second), at(second))
        core.tick(at(second))
    assert len(out.of(UAV)) == 4                                       # seconds 0, 2, 4, 6


def test_an_old_or_repeated_position_is_ignored():
    core, out = make_core()
    core.handle(fix(5, lat=36.2150), at(5))
    core.handle(fix(4, lat=36.2999), at(5))
    core.handle(fix(5, lat=36.2998), at(5))
    core.tick(at(5))
    assert core.stats['fixes_ignored_old'] == 2 and position(out.of(UAV)[0])['latitudeDegrees'] == 36.215


def test_sea_level_height_is_corrected_to_ellipsoid_height_but_ellipsoid_height_is_not():
    core, out = make_core()
    core.handle(fix(0, alt_amsl_m=240.0), at(0))
    core.tick(at(0))
    assert position(out.of(UAV)[0])['altitudeHaeMeters'] == pytest.approx(210.0)       # 240 + (-30)
    core2, out2 = make_core()
    core2.handle(fix(0, alt_amsl_m=None, alt_hae_m=210.0), at(0))
    core2.tick(at(0))
    assert position(out2.of(UAV)[0])['altitudeHaeMeters'] == pytest.approx(210.0)       # already HAE: never corrected twice


def test_the_drone_entity_carries_mode_battery_and_the_age_of_the_telemetry():
    core, out = make_core()
    core.handle(fix(0, mode='AUTO', armed=True, battery_percent=76.0, heading_deg=90.0, rel_alt_m=30.0), at(0))
    core.tick(at(3))                                                   # the position is 3 s old when it is published
    entity = out.of(UAV)[0]
    assert entity['isLive'] is True and entity['status']['platformActivity'] == 'AUTO'
    assert entity['provenance']['sourceUpdateTime'].startswith('2026-10-24T15:30:00')       # the measurement time, not "now"
    messages = [m['message'] for c in entity['health']['components'] for m in c['messages']]
    assert 'Battery at 76% charge.' in messages and 'Last telemetry 3.0 s ago.' in messages


def test_a_silent_link_stops_the_updates_and_never_repeats_the_old_position():
    core, out = make_core(stale_after_s=10.0)
    core.handle(fix(0), at(0))
    core.tick(at(0))
    for second in (5, 9, 11, 30, 120):
        core.tick(at(second))
    assert len(out.of(UAV)) == 1 and core.link_state == 'stale' and core.stats['stale_events'] == 1


def test_a_returning_link_resumes_the_updates():
    core, out = make_core(stale_after_s=10.0)
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.tick(at(40))
    core.handle(fix(41), at(41))
    core.tick(at(41))
    assert core.link_state == 'fresh' and core.stats['recoveries'] == 1 and len(out.of(UAV)) == 2


def test_the_not_live_policy_marks_the_drone_offline_once_and_puts_it_back_when_telemetry_returns():
    core, out = make_core(stale_after_s=10.0, offline_after_s=30.0, stale_policy='not_live')
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.tick(at(20))
    assert core.link_state == 'stale' and len(out.of(UAV)) == 1
    core.tick(at(31))
    core.tick(at(60))
    assert core.link_state == 'offline' and [e['isLive'] for e in out.of(UAV)] == [True, False]
    core.handle(fix(61), at(61))
    core.tick(at(61))
    assert [e['isLive'] for e in out.of(UAV)] == [True, False, True]
    times = [e['provenance']['sourceUpdateTime'] for e in out.of(UAV)]
    assert times == sorted(set(times))                                  # strictly newer every time, or Lattice ignores it


def test_shutdown_sends_one_goodbye_flushes_closes_and_ignores_later_events():
    core, out = make_core()
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.shutdown(at(5))
    core.shutdown(at(6))                                                # twice is harmless
    assert [e['isLive'] for e in out.of(UAV)] == [True, False] and (out.flushed, out.closed) == (1, 1)
    core.handle(fix(7), at(7))
    core.handle(sighting(), at(7))
    core.tick(at(7))
    assert len(out.entities) == 2


def test_shutdown_without_any_telemetry_sends_nothing_about_a_drone_that_was_never_seen():
    core, out = make_core()
    core.shutdown(at(1))
    assert out.entities == [] and out.closed == 1


def test_shutdown_after_the_drone_went_offline_does_not_send_a_second_goodbye():
    core, out = make_core(stale_after_s=10.0, offline_after_s=15.0, stale_policy='not_live')
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.tick(at(11))
    core.tick(at(20))
    core.shutdown(at(25))
    assert [e['isLive'] for e in out.of(UAV)] == [True, False]


# ------------------------------------------------------------------------------------------ the targets
def test_one_target_reported_many_times_stays_one_entity_and_only_real_changes_are_sent():
    core, out = make_core()
    core.handle(sighting(seconds=0, confidence=0.80), at(0))
    core.handle(sighting(seconds=1, confidence=0.80), at(1))                           # nothing new
    core.handle(sighting(seconds=2, lat=36.21510 + 3e-6, confidence=0.81), at(2))     # 0.3 m: not worth an update
    core.handle(sighting(seconds=3, confidence=0.95), at(3))                           # clearly surer: update
    ids = {e['entityId'] for e in out.entities}
    assert ids == {'astra-t1-tgt-person-1'} and len(out.entities) == 2
    assert core.stats['targets_created'] == 1 and core.stats['targets_updated'] == 1 and core.stats['targets_unchanged'] >= 1


def test_two_real_targets_are_two_entities_and_a_person_and_a_tent_do_not_mix():
    core, out = make_core()
    core.handle(sighting('person', 0, lat=36.2151, lon=-96.0062), at(0))
    core.handle(sighting('person', 1, lat=36.2160, lon=-96.0050), at(1))               # 130 m away
    core.handle(sighting('tent', 2, lat=36.2151, lon=-96.0062), at(2))                 # same spot, other kind
    assert {e['entityId'] for e in out.entities} == {'astra-t1-tgt-person-1', 'astra-t1-tgt-person-2', 'astra-t1-tgt-tent-1'}


def test_a_report_between_two_targets_merges_them_and_deletes_the_younger_one():
    core, out = make_core(merge_radius_m=14.0)
    core.handle(sighting('person', 0, lat=36.215000, lon=-96.0062, confidence=0.5), at(0))
    core.handle(sighting('person', 1, lat=36.215190, lon=-96.0062, confidence=0.5), at(1))      # 21 m north: a second target
    core.handle(sighting('person', 2, lat=36.215095, lon=-96.0062, confidence=1.0), at(2))      # sure, and between: one target
    survivors = {e['entityId'] for e in out.entities if e['isLive']}
    retired = [e for e in out.entities if not e['isLive']]
    assert 'astra-t1-tgt-person-1' in survivors and len(retired) == 1 and retired[0]['entityId'] == 'astra-t1-tgt-person-2'
    assert core.stats['targets_retired'] == 1
    times = [e['provenance']['sourceUpdateTime'] for e in out.of('astra-t1-tgt-person-2')]
    assert times == sorted(set(times))


def test_a_low_confidence_report_is_ignored_when_a_minimum_is_set():
    core, out = make_core(min_confidence=0.5)
    core.handle(sighting(confidence=0.3), at(0))
    assert out.entities == [] and core.stats['targets_ignored'] == 1


def test_target_updates_are_strictly_newer_each_time_even_for_reports_with_the_same_time():
    core, out = make_core()
    for index, confidence in enumerate((0.5, 0.7, 0.9)):
        core.handle(TargetSighting('person', 36.2151, -96.0062, confidence, at(0)), at(index))
    times = [e['provenance']['sourceUpdateTime'] for e in out.entities]
    assert len(times) == 3 and times == sorted(set(times))


def test_targets_expire_after_the_mission_plus_the_margin():
    core, out = make_core(mission_duration_s=600.0, target_margin_s=300.0)
    core.handle(sighting(), at(0))
    assert out.entities[0]['expiryTime'].startswith('2026-10-24T15:45:00')              # 10 min + 5 min


# ------------------------------------------------------------------------------------------ drops and areas
def test_a_release_with_a_position_becomes_a_map_point_and_without_one_it_is_only_counted():
    core, out = make_core()
    core.handle(DropEvent('person', at(5), 36.2150, -96.0060), at(5))
    core.handle(DropEvent('tent', at(9)), at(9))
    core.handle(DropEvent('person', at(12), 36.2152, -96.0061), at(12))
    assert [e['entityId'] for e in out.entities] == ['astra-t1-drop-person-1', 'astra-t1-drop-person-2']
    assert core.stats['drops_published'] == 2 and core.stats['drops_without_position'] == 1


SQUARE = ((36.2150, -96.0070), (36.2150, -96.0060), (36.2160, -96.0060), (36.2160, -96.0070))


def test_areas_are_published_once_at_start_and_a_broken_one_is_counted_not_fatal():
    good = AreaSpec('search_box', 'the box', SQUARE, C.GEO_TYPE_SEARCH_AREA)
    bad = AreaSpec('flat', '', SQUARE[:2], C.GEO_TYPE_SEARCH_AREA)
    out = Collect()
    core = BridgeCore(CoreSettings(mission_id='t1', geoid_undulation_m=-30.0, areas=(good, bad)), out)
    core.start(T0)
    assert [e['entityId'] for e in out.entities] == ['astra-t1-area-search-box']
    assert core.stats['areas_published'] == 1 and core.stats['area_failures'] == 1


# ------------------------------------------------------------------------------------------ nothing can stop it
def test_an_output_that_refuses_is_counted_and_the_bridge_carries_on():
    core, out = make_core()
    out.accept = False
    core.handle(fix(0), at(0))
    core.tick(at(0))
    assert core.stats['publish_failed'] == 1 and core.stats['drone_published'] == 0
    out.accept = True
    core.handle(fix(1), at(1))
    core.tick(at(1))
    assert core.stats['drone_published'] == 1


def test_an_output_that_raises_is_survived():
    class Exploding(Collect):
        def publish(self, entity, now):
            raise RuntimeError('boom')
    core = BridgeCore(CoreSettings(mission_id='t1', geoid_undulation_m=-30.0), Exploding())
    core.start(T0)
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.shutdown(at(1))
    assert core.stats['publish_failed'] >= 1 and core.closed


def test_a_bug_while_handling_one_event_is_logged_and_the_next_event_still_works(monkeypatch):
    core, out = make_core()
    monkeypatch.setattr(core, '_on_sighting', lambda *a: (_ for _ in ()).throw(ValueError('bug')))
    core.handle(sighting(), at(0))
    assert core.stats['handle_errors'] == 1
    core.handle(fix(1), at(1))
    core.tick(at(1))
    assert len(out.of(UAV)) == 1


def test_an_entity_that_breaks_the_contract_is_never_handed_to_the_output():
    core, out = make_core()
    core._emit({'entityId': 'astra-t1-uav'}, at(0), 'broken')
    assert out.entities == [] and core.stats['rejected_invalid'] == 1


def test_a_naive_clock_is_refused_up_front():
    with pytest.raises(ValueError):
        make_core()[0].start(datetime(2026, 10, 24, 15, 30))


@pytest.mark.parametrize('changes', [{'mission_id': 'Bad Id'}, {'stale_policy': 'maybe'}, {'drone_interval_s': 0},
                                     {'offline_after_s': 1.0, 'stale_after_s': 10.0}, {'mission_duration_s': 40 * 86400.0}])
def test_settings_that_cannot_work_are_refused_at_construction(changes):
    with pytest.raises(ValueError):
        BridgeCore(CoreSettings(**{'mission_id': 't1', **changes}), Collect())


def test_the_notice_that_a_replay_ended_is_remembered():
    core, _ = make_core()
    core.handle(SourceNotice('link_lost', 'radio gone', at(0)), at(0))
    assert not core.source_ended
    core.handle(SourceNotice('ended', 'done', at(1)), at(1))
    assert core.source_ended and core.stats['notices'] == 2


def test_the_health_line_summarises_the_state_in_one_line():
    core, _ = make_core()
    core.handle(fix(0), at(0))
    core.tick(at(0))
    core.handle(sighting('tent'), at(0))
    line = core.health_line(at(2), {'received': 40, 'tgt_reports': 1, 'link_losses': 0})
    assert '\n' not in line and 'drone=fresh' in line and 'fix_age=2.0s' in line and '0p/1t' in line
    assert 'received=40' in line and 'link_losses' not in line


def test_sink_out_writes_straight_into_a_sink():
    class Sink:
        def __init__(self):
            self.records, self.closed = [], False

        def write(self, kind, payload, t=None):
            self.records.append((kind, payload))
            return True

        def close(self):
            self.closed = True
    sink = Sink()
    out = SinkOut(sink)
    assert out.publish({'a': 1}, T0) is True
    out.flush()
    out.close()
    assert sink.records == [('entity', {'a': 1})] and sink.closed


# ------------------------------------------------------------------------------------------ the loop
class ScriptedSource:
    """A fake source: hands out scripted batches of events, with a virtual clock that steps 1 s per poll."""
    name = 'scripted'

    def __init__(self, batches):
        self.batches = list(batches)
        self.t = T0
        self.polls = 0

    def now(self):
        return self.t

    def poll(self, timeout_s):
        self.polls += 1
        self.t += timedelta(seconds=1)
        item = self.batches.pop(0) if self.batches else []
        if isinstance(item, Exception):
            raise item
        return item

    def stats(self):
        return {'received': self.polls}


def test_the_loop_runs_until_the_source_says_it_ended():
    core, out = make_core()
    source = ScriptedSource([[fix(1)], [fix(2)], [SourceNotice('ended', 'done', at(3))]])
    assert run_loop(core, source, threading.Event(), poll_s=0.0, health_interval_s=0) == 'ended'
    assert len(out.of(UAV)) == 2


def test_the_loop_stops_when_asked_and_survives_a_source_that_raises():
    core, out = make_core()
    stop = threading.Event()

    class Flaky(ScriptedSource):
        def poll(self, timeout_s):
            if self.polls == 1:
                stop.set()
            return super().poll(timeout_s)
    source = Flaky([[fix(1)], RuntimeError('radio exploded'), [fix(3)]])
    assert run_loop(core, source, stop, poll_s=0.0, health_interval_s=0) == 'stopped'
    assert len(out.of(UAV)) >= 1


def test_the_loop_logs_a_health_line_by_the_sources_own_time(caplog):
    core, _ = make_core()
    source = ScriptedSource([[fix(1)]] + [[] for _ in range(10)] + [[SourceNotice('ended', 'done', at(12))]])
    with caplog.at_level('INFO', logger='lattice_bridge.bridge'):
        run_loop(core, source, threading.Event(), poll_s=0.0, health_interval_s=5.0)
    assert sum('health:' in r.getMessage() for r in caplog.records) == 2
