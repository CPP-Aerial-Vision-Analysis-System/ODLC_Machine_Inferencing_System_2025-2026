"""
sim_demo/test/test_score_aim.py -- PURPOSE: unit tests of the AIM ERROR in the score report
(October 2026). Since then main_controller drops on the ground point under the detection box, not
on the photo's GPS position, and it says where with a status text "Aim <class> <lat>,<lon>". These
tests check reading that text, the aim error per target and per choice, that the old photo GPS
number is kept (same JSON key) and clearly labelled, the note when main_controller sent no aim,
that the mission map's "chosen" rings move to the aim points, and that re-scoring an older run
finds the aim texts in its events.jsonl.

Pure Python (no ROS):  cd ros2_ws/src/simulation/sim_demo && python3 -m pytest -q test
"""
import json
from dataclasses import asdict

import pytest

from sim_demo import mission_map
from sim_demo.detections import Box, DetectionFrame
from sim_demo.score_run import add_aim, aims_from_events, draw_map, rescore
from sim_demo.scoring import (AimRecord, DropRecord, PhotoRecord, parse_aim_text,
                              report_markdown, score_run_data, write_reports)


def _square(cx, cy, half):
    return [(cx - half, cy + half), (cx + half, cy + half), (cx + half, cy - half),
            (cx - half, cy - half)]


def _aim_text(cls, lat, lon):
    """Exactly what main_controller sends (main_controller.py _divert_to_targets)."""
    return f'Aim {cls} {lat:.7f},{lon:.7f}'


def _setup(world):
    """Hiker photographed 5 m east of it, tent 3 m south of it (the photo GPS errors)."""
    o = world.origin
    hiker, tent = world.targets
    la, loa, _ = o.to_geo(hiker.east + 5, hiker.north)
    lb, lob, _ = o.to_geo(tent.east, tent.north - 3)
    photos = [PhotoRecord('A.jpg', 10.0, la, loa, _square(hiker.east + 5, hiker.north, 12),
                          'gps-match'),
              PhotoRecord('B.jpg', 11.0, lb, lob, _square(tent.east, tent.north - 3, 12),
                          'gps-match')]
    frames = [DetectionFrame('A.jpg', la, loa, [Box('person', 0.81, 1, 1, 1, 1)], received=20.0),
              DetectionFrame('B.jpg', lb, lob, [Box('tent', 0.9, 1, 1, 1, 1)], received=21.0)]
    drops = [DropRecord('tent', 60.0, landing_en=(tent.east + 2.0, tent.north)),
             DropRecord('person', 80.0, *o.to_geo(hiker.east, hiker.north + 4.0)[:2])]
    return photos, frames, drops


def _aims(world, tent_off=(1.0, 0.0), hiker_off=(0.0, 0.5), t=50.0):
    """AimRecords as main_controller would send them: tent 1 m east, hiker 0.5 m north."""
    o = world.origin
    hiker, tent = world.targets
    out = []
    for cls, target, (de, dn) in (('tent', tent, tent_off), ('person', hiker, hiker_off)):
        lat, lon, _ = o.to_geo(target.east + de, target.north + dn)
        parsed = parse_aim_text(_aim_text(cls, lat, lon))           # through the real text
        out.append(AimRecord(parsed[0], t, parsed[1], parsed[2]))
    return out


# ------------------------------------------------------------------ the text
def test_parse_aim_text_reads_main_controllers_format():
    assert parse_aim_text('Aim tent 36.2150261,-96.0068008') == ('tent', 36.2150261, -96.0068008)
    assert parse_aim_text('aim PERSON  1.5 , -2.25 ') == ('person', 1.5, -2.25)
    assert parse_aim_text('Aim person -33.8688000,151.2093000') == ('person', -33.8688, 151.2093)


@pytest.mark.parametrize('text', [
    'Targets: tent@11 -> person@12. Diverting.', 'Over tent: loiter 6s then drop',
    'Aim tent', 'Aim car 1.0,2.0', 'Aim tent 95.0,10.0', 'Aim tent 10.0,190.0',
    'Aim tent 1.0;2.0', 'Detected tent', '', None,
])
def test_parse_aim_text_ignores_everything_else(text):
    assert parse_aim_text(text) is None


def test_add_aim_skips_a_text_heard_twice_but_keeps_a_later_divert():
    aims = []
    text = 'Aim tent 36.2150261,-96.0068008'
    assert add_aim(aims, 10.0, text) is not None
    assert add_aim(aims, 11.0, text) is None                  # the same text again
    assert add_aim(aims, 12.0, 'Targets: tent@11. Diverting.') is None
    assert add_aim(aims, 40.0, text) is not None              # a later divert counts again
    assert [a.t for a in aims] == [10.0, 40.0]


# ------------------------------------------------------------------ the numbers
def test_aim_error_per_target_and_per_choice(world):
    photos, frames, drops = _setup(world)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, [],
                        started=0.0, ended=100.0, aims=_aims(world))
    hiker, tent = rs.targets
    assert hiker.aim_error_m == pytest.approx(0.5, abs=0.02)
    assert tent.aim_error_m == pytest.approx(1.0, abs=0.02)
    # The old number is still there, unchanged (same JSON key as before).
    assert hiker.location_error_m == pytest.approx(5.0, abs=0.01)
    assert tent.location_error_m == pytest.approx(3.0, abs=0.01)
    choices = {c['class']: c for c in rs.choices}
    assert choices['tent']['aim_error_m'] == pytest.approx(1.0, abs=0.02)
    assert choices['tent']['aim_target'] == 'tent'
    assert choices['tent']['error_m'] == pytest.approx(3.0, abs=0.01)    # photo GPS
    assert choices['person']['aim_error_m'] == pytest.approx(0.5, abs=0.02)
    assert {a['class']: a['time_s'] for a in rs.aims} == {'person': 50.0, 'tent': 50.0}


def test_the_newest_aim_per_class_counts(world):
    photos, frames, drops = _setup(world)
    aims = _aims(world, tent_off=(10.0, 0.0), t=50.0) + _aims(world, t=70.0)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, [],
                        0.0, 100.0, aims=aims)
    assert rs.targets[1].aim_error_m == pytest.approx(1.0, abs=0.02)
    assert len(rs.aims) == 2


def test_without_aims_the_aim_error_is_unknown_and_the_report_says_why(world):
    photos, frames, drops = _setup(world)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, [],
                        0.0, 100.0)
    assert all(t.aim_error_m is None for t in rs.targets)
    assert all(c['aim_error_m'] is None for c in rs.choices)
    md = report_markdown(rs)
    assert '| hiker | person | YES | 0.81 | 1 | 1 | - | 5.0 m |' in md
    assert 'did not report where it aims for: person, tent' in md


# ------------------------------------------------------------------ the report files
def test_report_labels_both_numbers_clearly(world):
    photos, frames, drops = _setup(world)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, [],
                        0.0, 100.0, aims=_aims(world))
    md = report_markdown(rs)
    assert '| Aim error | Photo GPS error (old measure) | Drop error |' in md
    assert ('| hiker | person | YES | 0.81 | 1 | 1 | 0.5 m | 5.0 m | '
            '4.0 m (drone position at release) |') in md
    assert '| Class | Confidence | Aims at (lat, lon) | Aim error | Photo |' in md
    assert '| tent | 0.90 |' in md and '1.0 m (tent) | B.jpg | 3.0 m (tent) |' in md
    assert '- **Aim error**:' in md and '- **Photo GPS error (old measure)**:' in md
    assert 'did not report where it aims' not in md


def test_json_keeps_the_old_key_and_adds_the_aim(world, tmp_path):
    photos, frames, drops = _setup(world)
    rs = score_run_data('testfield', world.origin, world.targets, frames, photos, drops, [],
                        0.0, 100.0, aims=_aims(world))
    _md, js = write_reports(rs, str(tmp_path / 'run'))
    data = json.load(open(js))
    hiker = data['targets'][0]
    assert hiker['location_error_m'] == pytest.approx(5.0, abs=0.01)
    assert hiker['aim_error_m'] == pytest.approx(0.5, abs=0.02)
    assert sorted(a['class'] for a in data['aims']) == ['person', 'tent']


def test_map_rings_are_drawn_at_the_aim_points(world, tmp_path, monkeypatch):
    photos, frames, drops = _setup(world)
    hiker, tent = world.targets
    seen = {}

    def fake_draw(path, targets, areas, footprints, track, detections, choices, drop_pts, title):
        seen['choices'] = {c[0]: (c[1], c[2]) for c in choices}
        return path

    monkeypatch.setattr(mission_map, 'draw_mission_map', fake_draw)
    tent_only = [a for a in _aims(world) if a.cls == 'tent']
    assert draw_map(str(tmp_path), world, frames, photos, drops, [], 't', aims=tent_only)
    e, n = seen['choices']['tent']                       # where main_controller aims
    assert (e, n) == (pytest.approx(tent.east + 1.0, abs=0.02), pytest.approx(tent.north, abs=0.02))
    e, n = seen['choices']['person']                     # no aim: the photo position, as before
    assert (e, n) == (pytest.approx(hiker.east + 5.0, abs=0.02),
                      pytest.approx(hiker.north, abs=0.02))


# ------------------------------------------------------------------ re-scoring older runs
def _write_run(world, worlds_dir, run_dir, aims=None, events=()):
    """A recorded run folder: run_data.json (optionally with "aims") and events.jsonl."""
    photos, frames, drops = _setup(world)
    data = dict(world='testfield', worlds_dir=worlds_dir, truth_file='', detector='real',
                started=0.0, ended=100.0, frames=[asdict(f) for f in frames],
                photos=[asdict(p) for p in photos], drops=[asdict(d) for d in drops],
                timeline=[], notes=[], track=[])
    if aims is not None:
        data['aims'] = [asdict(a) for a in aims]
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / 'run_data.json').write_text(json.dumps(data), encoding='utf-8')
    (run_dir / 'events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events),
                                          encoding='utf-8')


def test_rescore_of_an_older_run_finds_the_aims_in_events(world, worlds_dir, tmp_path):
    run = tmp_path / 'old_run'
    events = [{'t': 50.0, 'kind': 'statustext', 'source': 'team',
               'text': _aim_text(a.cls, a.lat, a.lon)} for a in _aims(world)]
    events.insert(1, {'t': 50.5, 'kind': 'statustext', 'source': 'team',
                      'text': 'Targets: tent@11 -> person@12. Diverting.'})
    _write_run(world, worlds_dir, run, aims=None, events=events)   # no "aims": an older run
    rescore(str(run), world='testfield', worlds_dir=worlds_dir)
    report = json.load(open(run / 'score_report.json'))
    errors = {t['name']: t['aim_error_m'] for t in report['targets']}
    assert errors['hiker'] == pytest.approx(0.5, abs=0.02)
    assert errors['tent'] == pytest.approx(1.0, abs=0.02)


def test_rescore_prefers_the_recorded_aims(world, worlds_dir, tmp_path):
    run = tmp_path / 'new_run'
    far = _aims(world, tent_off=(20.0, 0.0))
    events = [{'t': 50.0, 'kind': 'statustext', 'text': _aim_text(a.cls, a.lat, a.lon)}
              for a in far]
    _write_run(world, worlds_dir, run, aims=_aims(world), events=events)
    rescore(str(run), world='testfield', worlds_dir=worlds_dir)
    report = json.load(open(run / 'score_report.json'))
    assert report['targets'][1]['aim_error_m'] == pytest.approx(1.0, abs=0.02)


def test_aims_from_events_copes_with_missing_and_broken_files(tmp_path):
    assert aims_from_events(str(tmp_path / 'nothing.jsonl')) == []
    path = tmp_path / 'events.jsonl'
    path.write_text('{"t": 1.0, "kind": "statustext", "text": "Aim tent 1.0,2.0"}\n'
                    'not json at all\n'
                    '{"t": 2.0, "kind": "photo", "name": "x.jpg"}\n'
                    '{"t": 3.0, "kind": "statustext", "text": "Aim person 3.0,4.0"}\n'
                    '{"t": 4.0, "kind": "statustext", "text": "Aim tent 1.0,2.0', encoding='utf-8')
    aims = aims_from_events(str(path))
    assert [(a.cls, a.lat, a.lon) for a in aims] == [('tent', 1.0, 2.0), ('person', 3.0, 4.0)]
