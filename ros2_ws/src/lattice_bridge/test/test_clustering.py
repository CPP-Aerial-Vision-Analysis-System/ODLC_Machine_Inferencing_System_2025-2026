"""
lattice_bridge/test/test_clustering.py -- PURPOSE: prove the TargetClusterer rules one by one: two photos of
one target merge, two targets stay apart, kinds never mix, the best confidence wins, the position is the
confidence-squared weighted mean, quiet updates, clusters that grow together merge (and the old id
survives), bad data is ignored with a reason, memory stays bounded, and the answer does not depend on the
order of arrival (except the numbering of ids).
"""

import itertools
import random
from datetime import datetime, timedelta

import pytest

from lattice_bridge import constants as C
from lattice_bridge.clustering import (
    ACTION_CREATE, ACTION_IGNORED, ACTION_NO_CHANGE, ACTION_UPDATE, TargetClusterer, state_track_kwargs,
)
from lattice_bridge.entities import build_target_track, find_problems
from lattice_bridge.geo import haversine_m, latlon_to_enu, offset_latlon
from support import MISSION, NOW, rng

BASE = (36.21513243, -96.00621469)


def at(east_m=0.0, north_m=0.0):
    """A point `east_m`/`north_m` metres from BASE."""
    return offset_latlon(BASE[0], BASE[1], east_m, north_m)


def sec(n):
    return NOW + timedelta(seconds=n)


def feed(clusterer, east=0.0, north=0.0, conf=0.9, when=0, kind='person'):
    lat, lon = at(east, north)
    return clusterer.feed(kind, lat, lon, conf, sec(when))


def metres_from_base(lat, lon):
    return latlon_to_enu(BASE[0], BASE[1], lat, lon)


@pytest.fixture
def clusterer():
    return TargetClusterer(MISSION)


# ------------------------------------------------------------------------------------ create / merge / separate
def test_first_sighting_creates_a_target_with_a_stable_id(clusterer):
    d = feed(clusterer, conf=0.8)
    assert d.action == ACTION_CREATE and d.should_publish and d.time_advanced
    assert d.entity_id == f'astra-{MISSION}-tgt-person-1'
    assert d.state.confidence == 0.8 and d.state.sightings == 1 and d.state.number == 1
    assert d.retired == () and d.retired_ids == ()
    assert len(clusterer) == 1


def test_two_photos_of_one_target_merge_into_one_entity(clusterer):
    first = feed(clusterer, 0, 0, 0.8, when=0)
    second = feed(clusterer, 2.0, -1.0, 0.85, when=1)
    assert second.entity_id == first.entity_id
    assert len(clusterer) == 1
    assert second.state.sightings == 2
    assert second.state.first_seen == sec(0) and second.state.last_seen == sec(1)


def test_two_real_targets_stay_apart(clusterer):
    a = feed(clusterer, 0, 0)
    b = feed(clusterer, 0, 60.0)
    assert a.action == b.action == ACTION_CREATE
    assert a.entity_id != b.entity_id
    assert (a.state.number, b.state.number) == (1, 2)
    assert len(clusterer) == 2


def test_a_person_and_a_tent_at_the_same_place_are_different_targets(clusterer):
    p = feed(clusterer, kind='person')
    t = feed(clusterer, kind='tent')
    assert p.entity_id.endswith('tgt-person-1') and t.entity_id.endswith('tgt-tent-1')
    assert len(clusterer) == 2


def test_numbers_count_per_kind(clusterer):
    ids = [feed(clusterer, 0, 100 * i, kind=k).entity_id for i, k in enumerate(['person', 'tent', 'person', 'tent'])]
    assert [i.split('tgt-')[1] for i in ids] == ['person-1', 'tent-1', 'person-2', 'tent-2']


def test_merge_radius_boundary():
    clusterer = TargetClusterer(MISSION, merge_radius_m=10.0)
    feed(clusterer, 0, 0)
    inside = feed(clusterer, 9.9, 0)
    assert inside.entity_id.endswith('person-1')
    outside = feed(clusterer, 0, 80.0)
    assert outside.action == ACTION_CREATE
    c2 = TargetClusterer(MISSION, merge_radius_m=10.0)
    feed(c2, 0, 0)
    assert feed(c2, 10.6, 0).action == ACTION_CREATE                          # just beyond 10 m


def test_the_nearest_target_wins_when_several_are_in_range():
    c = TargetClusterer(MISSION, merge_radius_m=30.0)
    feed(c, 0, 0)                            # person-1
    feed(c, 50, 0)                           # person-2 (50 m away: outside 30 m of person-1)
    d = feed(c, 35, 0)                       # 35 m from person-1 (outside), 15 m from person-2
    assert d.entity_id.endswith('person-2')


# ------------------------------------------------------------------------------------ confidence and position
def test_best_confidence_wins(clusterer):
    for conf in (0.5, 0.9, 0.6):
        d = feed(clusterer, conf=conf)
    assert d.state.confidence == 0.9
    assert clusterer.targets()[0].confidence == 0.9


def test_position_is_the_confidence_squared_weighted_mean(clusterer):
    feed(clusterer, 0, 0, 0.9)
    d = feed(clusterer, 10, 0, 0.3)          # weights 0.81 and 0.09: the estimate moves 10% of the way
    east, north = metres_from_base(d.state.lat_deg, d.state.lon_deg)
    assert east == pytest.approx(1.0, abs=0.01) and north == pytest.approx(0.0, abs=0.01)


def test_equal_confidences_give_the_plain_mean(clusterer):
    feed(clusterer, 0, 0, 0.7)
    feed(clusterer, 4, 0, 0.7)
    d = feed(clusterer, 2, 6, 0.7)
    east, north = metres_from_base(d.state.lat_deg, d.state.lon_deg)
    assert east == pytest.approx(2.0, abs=0.01) and north == pytest.approx(2.0, abs=0.01)


def test_a_poor_outlier_cannot_drag_the_estimate_far(clusterer):
    for i in range(4):
        feed(clusterer, 0, 0, 0.95, when=i)
    d = feed(clusterer, 11, 0, 0.1, when=5)                    # a bad sighting at the edge of the radius
    east, _ = metres_from_base(d.state.lat_deg, d.state.lon_deg)
    assert east < 0.1


def test_zero_confidence_does_not_break_the_mean(clusterer):
    feed(clusterer, 0, 0, 0.0)
    d = feed(clusterer, 2, 0, 0.0)
    east, _ = metres_from_base(d.state.lat_deg, d.state.lon_deg)
    assert east == pytest.approx(1.0, abs=0.01)


# ------------------------------------------------------------------------------------ quiet updates
def test_small_changes_are_no_change(clusterer):
    feed(clusterer, 0, 0, 0.9)
    d = feed(clusterer, 0.2, 0, 0.9, when=1)
    assert d.action == ACTION_NO_CHANGE and not d.should_publish and d.state is not None
    assert 'below the thresholds' in d.reason


def test_a_move_of_one_metre_triggers_an_update(clusterer):
    feed(clusterer, 0, 0, 0.5)
    d = feed(clusterer, 4, 0, 0.5, when=1)                     # mean moves 2 m
    assert d.action == ACTION_UPDATE and d.should_publish and d.time_advanced


def test_a_confidence_gain_triggers_an_update(clusterer):
    feed(clusterer, 0, 0, 0.60)
    assert feed(clusterer, 0, 0, 0.61, when=1).action == ACTION_NO_CHANGE        # +0.01
    assert feed(clusterer, 0, 0, 0.63, when=2).action == ACTION_UPDATE           # +0.03 since the last publish


def test_thresholds_are_measured_from_the_last_published_state_not_the_last_sighting(clusterer):
    feed(clusterer, 0, 0, 0.9)
    decisions = [feed(clusterer, 0.45 * i, 0, 0.9, when=i) for i in range(1, 8)]     # creeping drift
    assert any(d.action == ACTION_UPDATE for d in decisions)                           # the drift is noticed


def test_a_hiker_seen_in_twenty_photos_causes_few_publishes():
    c = TargetClusterer(MISSION)
    r = rng(20)
    publishes = 0
    for i in range(20):
        d = feed(c, r.gauss(0, 0.4), r.gauss(0, 0.4), min(0.99, 0.80 + 0.001 * i), when=i)
        publishes += d.should_publish
    assert 1 <= publishes <= 8
    assert len(c) == 1


# ------------------------------------------------------------------------------------ times
def test_time_bookkeeping(clusterer):
    feed(clusterer, when=10)
    d = feed(clusterer, 5, 0, 0.9, when=4)                     # an OLDER photo arrives later
    assert d.state.first_seen == sec(4) and d.state.last_seen == sec(10)
    assert d.action == ACTION_UPDATE and d.time_advanced is False        # sourceUpdateTime would not move
    d2 = feed(clusterer, 5, 0, 0.99, when=11)
    assert d2.state.last_seen == sec(11) and d2.time_advanced is True


def test_naive_measurement_time_is_a_programming_error(clusterer):
    with pytest.raises(ValueError, match='naive'):
        clusterer.feed('person', *at(), 0.9, datetime(2026, 10, 24, 15, 30, 0))
    with pytest.raises(ValueError, match='datetime'):
        clusterer.feed('person', *at(), 0.9, 1761319800)
    assert len(clusterer) == 0


def test_timezone_aware_non_utc_times_are_fine(clusterer):
    from datetime import timezone
    plus2 = timezone(timedelta(hours=2))
    d = clusterer.feed('person', *at(), 0.9, datetime(2026, 10, 24, 17, 30, 0, tzinfo=plus2))
    assert d.state.last_seen == NOW


# ------------------------------------------------------------------------------------ ignored data
@pytest.mark.parametrize('kind, lat, lon, conf, reason', [
    ('dog', 36.2, -96.0, 0.9, 'unknown kind'),
    (None, 36.2, -96.0, 0.9, 'unknown kind'),
    (['person'], 36.2, -96.0, 0.9, 'unknown kind'),
    ('person', float('nan'), -96.0, 0.9, 'bad position'),
    ('person', 36.2, float('inf'), 0.9, 'bad position'),
    ('person', 0.0, 0.0, 0.9, 'bad position'),
    ('person', 95.0, -96.0, 0.9, 'bad position'),
    ('person', None, None, 0.9, 'bad position'),
    ('person', 36.2, -96.0, 1.5, 'confidence'),
    ('person', 36.2, -96.0, -0.1, 'confidence'),
    ('person', 36.2, -96.0, float('nan'), 'confidence'),
    ('person', 36.2, -96.0, None, 'confidence'),
    ('person', 36.2, -96.0, '0.9', 'confidence'),
    ('person', 36.2, -96.0, True, 'confidence'),
])
def test_bad_data_is_ignored_with_a_reason_and_changes_nothing(clusterer, kind, lat, lon, conf, reason):
    d = clusterer.feed(kind, lat, lon, conf, NOW)
    assert d.action == ACTION_IGNORED and reason in d.reason and not d.should_publish
    assert d.state is None and d.entity_id is None and d.retired == ()
    assert len(clusterer) == 0
    with pytest.raises(ValueError):
        d.track_kwargs()


def test_min_confidence_filters_weak_sightings():
    c = TargetClusterer(MISSION, min_confidence=0.25)
    assert feed(c, conf=0.24).action == ACTION_IGNORED
    assert feed(c, conf=0.25).action == ACTION_CREATE


def test_garbage_between_good_data_does_not_disturb_the_state(clusterer):
    feed(clusterer, 0, 0, 0.9, when=0)
    before = clusterer.targets()
    for kind, lat, lon, conf in [('dog', 1, 1, 1), ('person', float('nan'), 1, 1), ('person', 1, 1, 7)]:
        assert clusterer.feed(kind, lat, lon, conf, NOW).action == ACTION_IGNORED
    assert clusterer.targets() == before


# ------------------------------------------------------------------------------------ growing together
def test_clusters_that_grow_together_merge_and_the_older_id_survives():
    c = TargetClusterer(MISSION, merge_radius_m=5.0)
    a = feed(c, 0, 0, 0.5)
    b = feed(c, 8, 0, 0.5)
    assert a.entity_id != b.entity_id and len(c) == 2                    # 8 m apart with a 5 m radius
    d = feed(c, 4, 0, 1.0)                                               # in range of both: goes to the older
    assert d.action == ACTION_UPDATE and len(c) == 1
    assert d.entity_id == a.entity_id
    assert d.retired_ids == (b.entity_id,) and d.retired[0].number == 2
    assert d.state.sightings == 3 and d.state.confidence == 1.0
    east, _ = metres_from_base(d.state.lat_deg, d.state.lon_deg)
    assert east == pytest.approx(4.0, abs=0.01)                          # (0*.25 + 8*.25 + 4*1.0)/1.5
    assert c.get(b.entity_id) is None and c.get(a.entity_id) is not None


def test_ids_are_never_reused_after_a_merge():
    c = TargetClusterer(MISSION, merge_radius_m=5.0)
    feed(c, 0, 0, 0.5)
    feed(c, 8, 0, 0.5)
    feed(c, 4, 0, 1.0)                                                   # person-2 retired
    d = feed(c, 0, 200)
    assert d.entity_id.endswith('person-3')


def test_a_retired_target_can_be_turned_into_a_goodbye_entity():
    c = TargetClusterer(MISSION, merge_radius_m=5.0)
    feed(c, 0, 0, 0.5, when=0)
    feed(c, 8, 0, 0.5, when=1)
    d = feed(c, 4, 0, 1.0, when=2)
    expiry = NOW + timedelta(hours=8)
    for state in d.retired:
        tomb = build_target_track(now=NOW, expiry_time=expiry, is_live=False,
                                  **state_track_kwargs(state, measured_time=d.state.last_seen))
        assert tomb['isLive'] is False and find_problems(tomb, NOW) == []
        assert tomb['provenance']['sourceUpdateTime'] == '2026-10-24T15:30:02.000000Z'     # the merging photo's time
        plain = state_track_kwargs(state)
        assert plain['measured_time'] == state.last_seen
    with pytest.raises(ValueError, match='naive'):
        state_track_kwargs(d.retired[0], measured_time=datetime(2026, 10, 24, 15, 30, 2))


def test_decision_feeds_the_track_builder_directly(clusterer):
    d = feed(clusterer, conf=0.93)
    entity = build_target_track(now=NOW, expiry_time=NOW + timedelta(hours=8), **d.track_kwargs())
    assert find_problems(entity, NOW) == []
    assert entity['entityId'] == d.entity_id and entity['aliases']['name'] == 'Person 1 (ASTRA)'
    assert entity['tracked']['lastMeasurementTime'] == '2026-10-24T15:30:00.000000Z'
    assert entity['createdTime'] == '2026-10-24T15:30:00.000000Z'


def test_antimeridian_sightings_merge():
    c = TargetClusterer(MISSION)
    c.feed('person', 36.0, 179.99999, 0.9, NOW)
    d = c.feed('person', 36.0, -179.99999, 0.9, NOW + timedelta(seconds=1))
    assert d.entity_id.endswith('person-1') and len(c) == 1
    assert abs(abs(d.state.lon_deg) - 180.0) < 0.0001


# ------------------------------------------------------------------------------------ bounds and settings
def test_memory_is_bounded_by_cluster_count_not_by_sightings():
    c = TargetClusterer(MISSION)
    for i in range(5000):
        feed(c, 0.001 * (i % 7), 0, 0.9, when=i)
    assert len(c) == 1 and c.targets()[0].sightings == 5000


def test_cluster_limit_ignores_further_new_targets():
    c = TargetClusterer(MISSION, max_clusters=3)
    for i in range(3):
        assert feed(c, 0, 100 * i).action == ACTION_CREATE
    d = feed(c, 0, 1000)
    assert d.action == ACTION_IGNORED and 'cluster limit' in d.reason and len(c) == 3
    assert feed(c, 1, 0).entity_id.endswith('person-1')                  # existing targets still accept sightings


@pytest.mark.parametrize('kwargs', [
    dict(merge_radius_m=0), dict(merge_radius_m=-1), dict(merge_radius_m=float('nan')), dict(merge_radius_m='5'),
    dict(update_min_move_m=-1), dict(update_min_conf_gain=float('inf')), dict(min_confidence=None),
    dict(max_clusters=0), dict(max_clusters=1.5), dict(max_clusters=True),
])
def test_bad_settings_raise(kwargs):
    with pytest.raises(ValueError):
        TargetClusterer(MISSION, **kwargs)


def test_bad_mission_id_raises():
    for bad in ('', 'Run-way', None, 'x' * 40):
        with pytest.raises(ValueError):
            TargetClusterer(bad)


def test_defaults_come_from_constants():
    c = TargetClusterer(MISSION)
    assert c.merge_radius_m == C.CLUSTER_MERGE_RADIUS_M
    assert c.update_min_move_m == C.CLUSTER_UPDATE_MIN_MOVE_M
    assert c.update_min_conf_gain == C.CLUSTER_UPDATE_MIN_CONF_GAIN
    assert c.max_clusters == C.CLUSTER_MAX_CLUSTERS


def test_same_input_sequence_gives_identical_decisions():
    def run():
        c = TargetClusterer(MISSION)
        r = random.Random(3)
        return [feed(c, r.uniform(-30, 30), r.uniform(-30, 30), r.random(), when=i, kind=r.choice(['person', 'tent']))
                for i in range(200)]
    assert run() == run()


def test_states_are_snapshots_not_live_views(clusterer):
    d = feed(clusterer, conf=0.5)
    feed(clusterer, 0, 0, 0.9, when=1)
    assert d.state.confidence == 0.5 and d.state.sightings == 1
    with pytest.raises(Exception):
        d.state.confidence = 1.0                               # frozen dataclass


# ------------------------------------------------------------------------------------ order independence
def _targets_signature(clusterer):
    rows = [(s.kind, s.lat_deg, s.lon_deg, s.confidence, s.sightings, s.first_seen, s.last_seen)
            for s in clusterer.targets()]
    return sorted(rows, key=lambda row: (row[0], round(row[1], 5), round(row[2], 5)))


def test_the_answer_does_not_depend_on_the_arrival_order():
    r = rng(21)
    centres = [('person', at(0, 0)), ('person', at(0, 150)), ('tent', at(120, 40)), ('tent', at(-90, -60))]
    sightings = []
    for index, (kind, (lat, lon)) in enumerate(centres):
        for k in range(8):
            east, north = r.uniform(-3, 3), r.uniform(-3, 3)
            s_lat, s_lon = offset_latlon(lat, lon, east, north)
            sightings.append((kind, s_lat, s_lon, round(r.uniform(0.3, 0.99), 2), sec(index * 100 + k)))
    reference = None
    for trial in range(60):
        order = sightings[:]
        r.shuffle(order)
        c = TargetClusterer(MISSION)
        for kind, lat, lon, conf, when in order:
            c.feed(kind, lat, lon, conf, when)
        signature = _targets_signature(c)
        assert len(signature) == 4
        if reference is None:
            reference = signature
            continue
        for got, want in zip(signature, reference):
            assert got[0] == want[0]
            assert got[1] == pytest.approx(want[1], abs=1e-9) and got[2] == pytest.approx(want[2], abs=1e-9)
            assert got[3:] == want[3:]


def test_order_independence_when_clusters_have_to_merge():
    """Sightings spread so that the first ones start separate clusters; the final partition must still be one."""
    points = [(0, 0, 0.5), (8, 0, 0.5), (4, 0, 1.0), (2, 1, 0.7), (6, -1, 0.7)]
    results = set()
    for order in itertools.permutations(range(len(points))):
        c = TargetClusterer(MISSION, merge_radius_m=5.0)
        for i in order:
            east, north, conf = points[i]
            feed(c, east, north, conf, when=i)
        assert len(c) == 1, order
        s = c.targets()[0]
        results.add((round(s.lat_deg, 7), round(s.lon_deg, 7), s.confidence, s.sightings))
    assert len(results) == 1


def test_every_pair_of_sightings_within_the_radius_ends_in_one_target_for_any_order():
    r = rng(22)
    for _ in range(200):
        n = r.randint(2, 8)
        pts = [(r.uniform(-4, 4), r.uniform(-4, 4), r.uniform(0.2, 1.0)) for _ in range(n)]     # all within 8 m
        order = list(range(n))
        r.shuffle(order)
        c = TargetClusterer(MISSION)
        for i in order:
            feed(c, pts[i][0], pts[i][1], pts[i][2], when=i)
        assert len(c) == 1
        assert c.targets()[0].sightings == n
        s = c.targets()[0]
        assert s.confidence == max(p[2] for p in pts)


def test_positions_stay_inside_the_hull_of_the_sightings():
    r = rng(23)
    for _ in range(100):
        pts = [(r.uniform(-5, 5), r.uniform(-5, 5)) for _ in range(6)]
        c = TargetClusterer(MISSION)
        for i, (e, n) in enumerate(pts):
            d = feed(c, e, n, r.uniform(0.3, 1.0), when=i)
        east, north = metres_from_base(d.state.lat_deg, d.state.lon_deg)
        assert min(p[0] for p in pts) - 0.01 <= east <= max(p[0] for p in pts) + 0.01
        assert min(p[1] for p in pts) - 0.01 <= north <= max(p[1] for p in pts) + 0.01
        assert haversine_m(*BASE, d.state.lat_deg, d.state.lon_deg) < 8
