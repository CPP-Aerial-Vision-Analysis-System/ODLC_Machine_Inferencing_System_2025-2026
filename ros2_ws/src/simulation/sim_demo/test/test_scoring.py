"""
sim_demo/test/test_scoring.py -- PURPOSE: unit tests of the run scorer (scoring.py): matching
detections to targets with and without photo footprints, location and drop errors, what
main_controller chose, TP/FP/FN counting, and the Markdown/JSON report files.
"""
import json

import pytest

from sim_demo.detections import Box, DetectionFrame
from sim_demo.scoring import (DropRecord, PhotoRecord, report_markdown, score_run_data,
                              write_reports)


def _square(cx, cy, half):
    return [(cx - half, cy + half), (cx + half, cy + half), (cx + half, cy - half), (cx - half, cy - half)]


def _setup(world):
    o = world.origin
    hiker, tent = world.targets
    # Photo A shows the hiker (taken 5 m east of it); photo B shows the tent; photo C shows nothing.
    la, loa, _ = o.to_geo(hiker.east + 5, hiker.north)
    lb, lob, _ = o.to_geo(tent.east, tent.north - 3)
    lc, loc, _ = o.to_geo(-15, -15)
    photos = [PhotoRecord('A.jpg', 10.0, la, loa, _square(hiker.east + 5, hiker.north, 12), 'gps-match'),
              PhotoRecord('B.jpg', 11.0, lb, lob, _square(tent.east, tent.north - 3, 12), 'gps-match'),
              PhotoRecord('C.jpg', 12.0, lc, loc, _square(-15, -15, 4), 'gps-match')]
    frames = [DetectionFrame('A.jpg', la, loa, [Box('person', 0.81, 1, 1, 1, 1)], received=20.0),
              DetectionFrame('B.jpg', lb, lob, [Box('tent', 0.9, 1, 1, 1, 1)], received=21.0),
              DetectionFrame('C.jpg', lc, loc, [Box('tent', 0.4, 1, 1, 1, 1)], received=22.0)]
    rl, rlo, _ = o.to_geo(tent.east + 1.0, tent.north)
    drops = [DropRecord('tent', 30.0, rl, rlo, landing_en=(tent.east + 2.0, tent.north)),
             DropRecord('person', 40.0, *o.to_geo(hiker.east, hiker.north + 4.0)[:2])]
    timeline = [(1.0, 'armed'), (30.0, 'tent payload released')]
    return photos, frames, drops, timeline


def test_scores(world):
    photos, frames, drops, timeline = _setup(world)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, timeline,
                        started=0.0, ended=100.0, detector='truth')
    hiker, tent = rs.targets
    assert hiker.detected and hiker.best_confidence == 0.81 and hiker.detections == 1
    assert hiker.location_error_m == pytest.approx(5.0, abs=0.01)
    assert hiker.photos_showing == 1 and hiker.first_detected_s == 20.0
    assert tent.detected and tent.location_error_m == pytest.approx(3.0, abs=0.01)
    assert tent.drop_error_m == pytest.approx(2.0, abs=0.01)
    assert tent.drop_source == 'Gazebo landing point'
    assert hiker.drop_error_m == pytest.approx(4.0, abs=0.01)
    assert hiker.drop_source == 'drone position at release'
    person, tents = rs.classes
    assert (person.tp, person.fp, person.fn) == (1, 0, 0)
    assert (tents.tp, tents.fp, tents.fn) == (1, 1, 0)          # C.jpg tent is a false alarm
    assert tents.precision == pytest.approx(0.5) and tents.recall == pytest.approx(1.0)
    assert rs.false_detections[0]['photo'] == 'C.jpg'
    choices = {c['class']: c for c in rs.choices}
    assert choices['tent']['photo'] == 'B.jpg' and choices['tent']['error_m'] == pytest.approx(3.0, abs=0.01)
    assert rs.photos_taken == 3 and rs.photos_processed == 3 and rs.photos_with_detections == 3


def test_missed_target_counts_as_fn(world):
    photos, frames, drops, timeline = _setup(world)
    frames[0] = DetectionFrame('A.jpg', frames[0].lat, frames[0].lon, [], received=20.0)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, [], [],
                        0.0, 50.0)
    hiker = rs.targets[0]
    assert not hiker.detected and rs.classes[0].fn == 1 and hiker.location_error_m is None


def test_without_footprints_uses_radius(world):
    photos, frames, _, _ = _setup(world)
    for p in photos:
        p.footprint = None
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, [], [], 0.0, 1.0,
                        geo_match_radius_m=10.0)
    assert rs.targets[0].detected and rs.targets[1].detected
    assert rs.classes[1].fp == 1              # C.jpg is ~60 m from the tent


def test_reports_are_written(world, tmp_path):
    photos, frames, drops, timeline = _setup(world)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, timeline,
                        0.0, 125.0, detector='real')
    md = report_markdown(rs)
    assert '# ASTRA search demo - score report' in md
    assert '| hiker | person | YES | 0.81 |' in md
    assert 'REAL MODEL' in md and '2:05' in md
    md_path, js_path = write_reports(rs, str(tmp_path / 'run'))
    data = json.load(open(js_path))
    assert data['targets_found'] == 2 and data['world'] == 'testfield'
    assert data['classes'][1]['precision'] == pytest.approx(0.5)
    assert open(md_path).read() == md


def test_mission_map_picture(world, tmp_path):
    pytest.importorskip('PIL')
    from PIL import Image

    from sim_demo.mission_map import draw_mission_map
    photos, frames, drops, _ = _setup(world)
    hiker, tent = world.targets
    path = draw_mission_map(str(tmp_path / 'map.png'),
                            [(t.name, t.cls, t.east, t.north) for t in world.targets],
                            world.search_areas, [p.footprint for p in photos],
                            [(0, 0), (10, 20), (35, 40), (70, 40)],
                            [('person', hiker.east + 5, hiker.north, 0.81)],
                            [('tent', tent.east, tent.north - 3)],
                            [('tent', tent.east + 2, tent.north, 'landed')], 'test map')
    with Image.open(path) as im:
        assert im.size[0] > 500 and im.size[1] > 300
        # Something was drawn in the target colours (hiker red, tent orange).
        colours = {c for _, c in im.convert('RGB').getcolors(maxcolors=1_000_000)}
        assert (255, 77, 77) in colours and (255, 163, 26) in colours


def test_report_links_the_map(world):
    rs = score_run_data('testfield', world.origin, world.targets, [], [], [], [], 0.0, 1.0)
    rs.map_file = 'mission_map.png'
    assert '](mission_map.png)' in report_markdown(rs)


def test_empty_run(world):
    rs = score_run_data('testfield', world.origin, world.targets, [], [], [], [], 0.0, 1.0)
    md = report_markdown(rs)
    assert 'nothing to fly to' in md and 'No payload was released.' in md
    assert all(not t.detected for t in rs.targets)
