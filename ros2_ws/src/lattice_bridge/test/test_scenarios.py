"""
lattice_bridge/test/test_scenarios.py -- PURPOSE: run the library against the REAL positions of the two
simulator worlds (runway, suas_field_real): the true hiker, tent, decoy people, search areas and flight boundary
(copied into data/scenario_truth.yaml; a freshness test compares the copy with the real world files when the
full repo is present). This is the "does it make sense on the actual field" test:

  * photos of each real target (with realistic position noise) merge into exactly one track each, and the
    track ends up close to the truth, whatever order the photos arrive in;
  * decoy people stay separate tracks and never swallow the hiker;
  * the search areas and the flight boundary turn into valid polygons that contain the targets / the areas;
  * the drop points and the drone at home build valid entities.
"""

from datetime import timedelta

import pytest

from lattice_bridge import constants as C
from lattice_bridge import world_areas
from lattice_bridge.clustering import TargetClusterer, state_track_kwargs
from lattice_bridge.entities import build_area_geo, build_drop_point, build_target_track, build_uav_asset, find_problems
from lattice_bridge.geo import haversine_m, offset_latlon
from support import NOW, rng, scenarios

DATA = scenarios()
WORLDS = sorted(DATA)
EXPIRY = NOW + timedelta(hours=8)
RADIUS = C.CLUSTER_MERGE_RADIUS_M


def sightings_for(world, include_decoys, seed=5, photos=8, sigma=1.5):
    """[(kind, lat, lon, conf, when, truth_id)] simulated photo detections of every target (and decoy)."""
    r = rng(seed)
    truth = DATA[world]['truth']
    rows = []
    objects = [(t['id'], 'person' if t['class'] == 'person' else 'tent', t['lat'], t['lon'], (0.7, 0.97))
               for t in truth['targets']]
    if include_decoys:
        objects += [(d['id'], 'person', d['lat'], d['lon'], (0.3, 0.55)) for d in truth['person_decoys']]
    for index, (oid, kind, lat, lon, (low, high)) in enumerate(objects):
        for k in range(photos if oid in {t['id'] for t in truth['targets']} else 2):
            east = max(-4.0, min(4.0, r.gauss(0, sigma)))
            north = max(-4.0, min(4.0, r.gauss(0, sigma)))
            s_lat, s_lon = offset_latlon(lat, lon, east, north)
            rows.append((kind, s_lat, s_lon, round(r.uniform(low, high), 2), NOW + timedelta(seconds=index * 20 + k), oid))
    return rows


def run(rows):
    clusterer = TargetClusterer('scenario')
    decisions = [clusterer.feed(k, lat, lon, conf, when) for k, lat, lon, conf, when, _ in rows]
    return clusterer, decisions


@pytest.mark.parametrize('world', WORLDS)
def test_real_objects_of_one_kind_are_further_apart_than_twice_the_merge_radius(world):
    truth = DATA[world]['truth']
    objects = [(t['id'], t['class'], t['lat'], t['lon']) for t in truth['targets']]
    objects += [(d['id'], 'person', d['lat'], d['lon']) for d in truth['person_decoys']]
    closest = min(haversine_m(a[2], a[3], b[2], b[3]) for i, a in enumerate(objects) for b in objects[i + 1:]
                  if a[1] == b[1])
    assert closest > 2 * RADIUS, f'{world}: closest same-kind pair is only {closest:.1f} m'
    targets = [o for o in objects if o[0] in {t['id'] for t in truth['targets']}]
    far = min(haversine_m(a[2], a[3], b[2], b[3]) for i, a in enumerate(targets) for b in targets[i + 1:]
              if a[1] == b[1]) if len(targets) > 2 else 1e9
    assert far > 100.0


@pytest.mark.parametrize('world', WORLDS)
def test_photos_of_each_real_target_merge_into_one_track_close_to_the_truth(world):
    clusterer, _ = run(sightings_for(world, include_decoys=False))
    truth = DATA[world]['truth']['targets']
    states = clusterer.targets()
    assert len(states) == len(truth)
    for target in truth:
        kind = 'person' if target['class'] == 'person' else 'tent'
        nearest = min((s for s in states if s.kind == kind),
                      key=lambda s: haversine_m(s.lat_deg, s.lon_deg, target['lat'], target['lon']))
        error = haversine_m(nearest.lat_deg, nearest.lon_deg, target['lat'], target['lon'])
        assert error < 2.0, f"{target['id']}: track is {error:.2f} m from the truth"
        assert nearest.sightings == 8
    assert len({s.entity_id for s in states}) == len(states)


@pytest.mark.parametrize('world', WORLDS)
def test_decoy_people_stay_separate_and_do_not_swallow_the_hiker(world):
    truth = DATA[world]['truth']
    clusterer, _ = run(sightings_for(world, include_decoys=True))
    persons = [s for s in clusterer.targets() if s.kind == 'person']
    hikers = [t for t in truth['targets'] if t['class'] == 'person']
    assert len(persons) == len(hikers) + len(truth['person_decoys'])
    for hiker in hikers:
        nearest = min(persons, key=lambda s: haversine_m(s.lat_deg, s.lon_deg, hiker['lat'], hiker['lon']))
        assert haversine_m(nearest.lat_deg, nearest.lon_deg, hiker['lat'], hiker['lon']) < 2.0
        assert nearest.sightings == 8                        # only the hiker's own photos, none of the decoys'
        assert nearest.confidence >= 0.7
    assert len([s for s in clusterer.targets() if s.kind == 'tent']) == len([t for t in truth['targets'] if t['class'] == 'tent'])


@pytest.mark.parametrize('world', WORLDS)
def test_arrival_order_does_not_change_the_result(world):
    rows = sightings_for(world, include_decoys=True)
    r = rng(30)
    reference = None
    for _ in range(25):
        shuffled = rows[:]
        r.shuffle(shuffled)
        clusterer, _ = run(shuffled)
        signature = sorted((s.kind, round(s.lat_deg, 6), round(s.lon_deg, 6), s.confidence, s.sightings)
                           for s in clusterer.targets())
        if reference is None:
            reference = signature
        assert signature == reference


@pytest.mark.parametrize('world', WORLDS)
def test_the_published_tracks_are_valid_entities_at_the_projected_target_position(world):
    clusterer, decisions = run(sightings_for(world, include_decoys=False))
    published = [d for d in decisions if d.should_publish]
    assert len(published) >= len(clusterer)
    for decision in published:
        entity = build_target_track(now=NOW + timedelta(minutes=5), expiry_time=EXPIRY, **decision.track_kwargs())
        assert find_problems(entity, NOW + timedelta(minutes=5)) == []
    # final state -> one entity per target, each within 2 m of a truth target
    final = {s.entity_id: build_target_track(now=NOW + timedelta(minutes=5), expiry_time=EXPIRY,
                                             **state_track_kwargs(s))
             for s in clusterer.targets()}
    for entity in final.values():
        pos = entity['location']['position']
        assert min(haversine_m(pos['latitudeDegrees'], pos['longitudeDegrees'], t['lat'], t['lon'])
                   for t in DATA[world]['truth']['targets']) < 2.0


def point_in_polygon(lat, lon, ring):
    """Ray casting on (lat, lon) corners (the ring may be open or closed)."""
    inside = False
    n = len(ring)
    for i in range(n):
        (lat1, lon1), (lat2, lon2) = ring[i], ring[(i + 1) % n]
        if (lat1 > lat) != (lat2 > lat):
            cross = lon1 + (lat - lat1) * (lon2 - lon1) / (lat2 - lat1)
            if lon < cross:
                inside = not inside
    return inside


@pytest.mark.parametrize('world', WORLDS)
def test_search_areas_become_valid_polygons_that_contain_their_targets(world):
    specs = world_areas.search_areas(DATA[world]['config'])
    assert specs
    by_name = {s.name: s for s in specs}
    for spec in specs:
        entity = build_area_geo(f'astra-scenario-area-{spec.name.replace("_", "-")}', spec.name, spec.corners_latlon,
                                source_update_time=NOW, now=NOW, expiry_time=EXPIRY, description=spec.note or None)
        assert find_problems(entity, NOW) == []
        assert entity['geoDetails']['type'] == 'GEO_TYPE_GENERAL'
    for target in DATA[world]['truth']['targets']:
        spec = by_name[target['area']]
        assert point_in_polygon(target['lat'], target['lon'], list(spec.corners_latlon)), target['id']


def test_the_flight_boundary_is_a_valid_control_area_that_contains_the_search_areas():
    truth = DATA['suas_field_real']['truth']
    boundary = world_areas.flight_boundary(truth)
    assert boundary.geo_type == C.GEO_TYPE_CONTROL_AREA and len(boundary.corners_latlon) == 11
    assert '45.7' in boundary.note and '121.9' in boundary.note
    entity = build_area_geo('astra-scenario-area-flight-boundary', 'Flight boundary', boundary.corners_latlon,
                            source_update_time=NOW, now=NOW, expiry_time=EXPIRY, geo_type=boundary.geo_type,
                            description=boundary.note)
    assert find_problems(entity, NOW) == []
    assert len(entity['geoShape']['polygon']['rings'][0]['positions']) == 12
    ring = list(boundary.corners_latlon)
    for spec in world_areas.search_areas(DATA['suas_field_real']['config']):
        for lat, lon in spec.corners_latlon:
            assert point_in_polygon(lat, lon, ring), f'{spec.name} corner outside the flight boundary'
    home = truth['home']
    assert point_in_polygon(home['lat'], home['lon'], ring)


def test_the_runway_world_has_no_flight_boundary():
    assert world_areas.flight_boundary(DATA['runway']['truth']) is None


@pytest.mark.parametrize('world', WORLDS)
def test_drop_points_and_the_drone_at_home_build_valid_entities(world):
    truth = DATA[world]['truth']
    for target in truth['targets']:
        kind = 'person' if target['class'] == 'person' else 'tent'
        drop = build_drop_point(f'astra-scenario-drop-{kind}-1', kind, target['lat'], target['lon'],
                                release_time=NOW, now=NOW, expiry_time=EXPIRY)
        assert find_problems(drop, NOW) == []
    home = truth['home']
    uav = build_uav_asset('astra-scenario-uav', home['lat'], home['lon'], fix_time=NOW, now=NOW,
                          alt_amsl_m=home['alt_msl'] + 49.0, rel_alt_m=49.0, geoid_undulation_m=-29.0)
    assert find_problems(uav, NOW) == []
    assert uav['location']['position']['altitudeHaeMeters'] == pytest.approx(home['alt_msl'] + 49.0 - 29.0)


@pytest.mark.parametrize('world', WORLDS)
def test_world_areas_agree_with_the_corner_lists_in_the_truth_files(world):
    """Our metres->degrees conversion reproduces the generator's own corners_latlon to a few centimetres."""
    specs = {s.name: s for s in world_areas.search_areas(DATA[world]['config'])}
    for area in DATA[world]['truth']['search_areas']:
        ours = specs[area['name']].corners_latlon
        for want in area['corners_latlon']:
            assert min(haversine_m(c[0], c[1], want[0], want[1]) for c in ours) < 0.2, area['name']


def test_the_committed_scenario_copy_is_up_to_date():
    """Skipped outside the full repo (a clean overlay has no sim_worlds); inside it, the copy must match."""
    from scenario_data import build_from_repo, load_fixture, sim_worlds_dir
    worlds_dir = sim_worlds_dir()
    if worlds_dir is None:
        pytest.skip('sim_worlds is not next to this package (clean overlay): freshness cannot be checked')
    assert load_fixture() == build_from_repo(worlds_dir), (
        'data/scenario_truth.yaml is out of date: run  python3 test/scenario_data.py --write')
