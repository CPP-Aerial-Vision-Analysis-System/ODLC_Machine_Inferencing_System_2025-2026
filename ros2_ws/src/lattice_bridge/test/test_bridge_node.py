"""
lattice_bridge/test/test_bridge_node.py -- PURPOSE: prove the bridge PROGRAM end to end, the way a person runs it:
a made-up simulator run folder is replayed through the real source, core, publisher and sinks into (a) a file and
(b) the fake Lattice over real sockets on this computer. Also the things around it: the dry run, the exit codes for
bad settings / missing run folder / missing token, the secret filter, and the guard that a fast replay may not go to
a Lattice. No ROS, no simulator, no internet.
"""

import json
import logging
import threading

import pytest

from lattice_bridge import bridge_config as BC
from lattice_bridge.bridge_node import PublisherOut, RedactFilter, build_parser, run
from lattice_bridge.fake_lattice import FakeLattice
from lattice_bridge.publisher import REJECTED
from support_net import GOOD_TOKEN, make_publisher

T0 = 1_792_000_000          # a Unix time in October 2026 (the replay moves it to "now" anyway)

WORLD_CONFIG = """\
world_name: runway
home: "36.2143000,-96.0068000,210.0,0"
search_areas:
  - name: search_box
    center: [25.0, 95.0]
    size: [120.0, 100.0]
    yaw: 0.0
"""


def make_run(root, name='20261024-101500_runway', seconds=3):
    """A tiny simulator run folder: photos (the drone's track), TGT lines, one drop. Returns the folder path."""
    run_dir = root / name
    (run_dir / 'photos').mkdir(parents=True)
    for index in range(seconds + 1):
        sidecar = {'shutter_unix_s': T0 + index, 'position': {'lat': 36.2150 + index * 1e-5, 'lon': -96.0068,
                                                            'age_s': 0.0, 'alt_msl_m': 240.0},
                   'rel_alt_m': 30.0, 'heading_deg': 90.0}
        (run_dir / 'photos' / f'p{index}.json').write_text(json.dumps(sidecar))
    lines = [{'kind': 'armed', 't': T0 - 5, 'mode': 'GUIDED'},
             {'kind': 'statustext', 't': T0 + 1, 'text': 'TGT p 36.215000 -96.006000 91 1'},
             {'kind': 'statustext', 't': T0 + 2, 'text': 'TGT p 36.215004 -96.006003 95 2'},     # the same person again
             {'kind': 'statustext', 't': T0 + 2, 'text': 'TGT t 36.215300 -96.007500 80 3'}]
    (run_dir / 'events.jsonl').write_text('\n'.join(json.dumps(line) for line in lines) + '\n')
    (run_dir / 'run_data.json').write_text(json.dumps({'world': 'runway', 'drops': [
        {'t': T0 + 3, 'cls': 'person', 'release_lat': 36.2150, 'release_lon': -96.0060}]}))
    return run_dir


@pytest.fixture()
def run_dir(tmp_path):
    return make_run(tmp_path)


@pytest.fixture()
def world_config(tmp_path):
    path = tmp_path / 'runway_config.yaml'
    path.write_text(WORLD_CONFIG)
    return path


@pytest.fixture(autouse=True)
def clean_environment(monkeypatch):
    for name in ('LATTICE_ENDPOINT', 'LATTICE_TOKEN', 'SANDBOXES_TOKEN'):
        monkeypatch.delenv(name, raising=False)


def go(*args):
    """Run the program without installing signal handlers; returns its exit code."""
    return run([str(a) for a in args], stop=threading.Event(), install_signals=False)


def records(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def entity_ids(path):
    return sorted({r['payload']['entityId'] for r in records(path) if r['kind'] == 'entity'})


# ------------------------------------------------------------------------------------------ replay into a file
def test_a_replay_into_a_file_writes_the_drone_the_targets_the_drop_and_the_area(tmp_path, run_dir, world_config):
    out = tmp_path / 'requests.jsonl'
    code = go('--source', 'replay', '--run-dir', run_dir, '--speed', 0, '--sink', 'jsonl', '--output', out,
              '--area-config', world_config, '--mission-id', 'itest', '--geoid-undulation', -30)
    assert code == 0
    ids = entity_ids(out)
    assert ids == ['astra-itest-area-search-box', 'astra-itest-drop-person-1', 'astra-itest-tgt-person-1',
                   'astra-itest-tgt-tent-1', 'astra-itest-uav']
    drone = [r['payload'] for r in records(out) if r['payload']['entityId'] == 'astra-itest-uav']
    assert len(drone) >= 3 and drone[-1]['isLive'] is False                         # many updates, then the goodbye
    times = [d['provenance']['sourceUpdateTime'] for d in drone]
    assert times == sorted(set(times))                                              # each strictly newer than the last


def test_one_person_reported_twice_is_still_one_entity(tmp_path, run_dir):
    out = tmp_path / 'requests.jsonl'
    assert go('--source', 'replay', '--run-dir', run_dir, '--speed', 0, '--output', out, '--mission-id', 'itest',
              '--geoid-undulation', -30) == 0
    persons = [r['payload'] for r in records(out) if r['payload']['entityId'] == 'astra-itest-tgt-person-1']
    assert persons and not any(p['entityId'].endswith('person-2') for p in persons)


def test_the_mission_id_is_made_from_the_world_when_none_is_given(tmp_path, run_dir):
    out = tmp_path / 'requests.jsonl'
    assert go('--source', 'replay', '--run-dir', run_dir, '--speed', 0, '--output', out, '--geoid-undulation', -30) == 0
    assert all(i.startswith('astra-runway') for i in entity_ids(out))


def test_the_test_sink_works_as_a_destination_too(run_dir):
    assert go('--source', 'replay', '--run-dir', run_dir, '--speed', 0, '--sink', 'mock', '--geoid-undulation', -30) == 0


# ------------------------------------------------------------------------------------------ replay into the fake Lattice
def test_a_real_time_replay_into_the_fake_lattice_fills_its_picture_and_takes_the_drone_off_at_the_end(
        tmp_path, monkeypatch, world_config):
    run_dir = make_run(tmp_path, seconds=2)
    with FakeLattice(token=GOOD_TOKEN) as fake:
        monkeypatch.setenv('LATTICE_TOKEN', GOOD_TOKEN)
        code = go('--source', 'replay', '--run-dir', run_dir, '--speed', 1, '--sink', 'rest', '--endpoint', fake.url,
                  '--area-config', world_config, '--mission-id', 'itest', '--geoid-undulation', -30)
        picture = fake.entities()
        stats = fake.stats()
    assert code == 0
    assert sorted(picture) == ['astra-itest-area-search-box', 'astra-itest-drop-person-1', 'astra-itest-tgt-person-1',
                               'astra-itest-tgt-tent-1']
    assert stats['went_offline'] >= 1 and stats['refused_contract'] == 0 and stats['refused_auth'] == 0


def test_a_wrong_token_does_not_stop_the_bridge_and_loses_nothing_it_could_keep(tmp_path, monkeypatch, world_config):
    run_dir = make_run(tmp_path, seconds=1)
    with FakeLattice(token=GOOD_TOKEN) as fake:
        monkeypatch.setenv('LATTICE_TOKEN', 'wrong-' + GOOD_TOKEN)
        code = go('--source', 'replay', '--run-dir', run_dir, '--speed', 1, '--sink', 'rest', '--endpoint', fake.url,
                  '--mission-id', 'itest', '--geoid-undulation', -30)
        stats = fake.stats()
    assert code == 0 and stats['accepted'] == 0 and stats['refused_auth'] >= 1


# ------------------------------------------------------------------------------------------ the dry run
def test_a_dry_run_prints_the_plan_and_sends_nothing(tmp_path, run_dir, world_config, capsys):
    out = tmp_path / 'never_written.jsonl'
    code = go('--source', 'replay', '--run-dir', run_dir, '--output', out, '--area-config', world_config, '--dry-run')
    text = capsys.readouterr().out
    assert code == 0 and not out.exists()
    assert 'PLAN' in text and 'areas  : 1 (search_box)' in text and 'verdict: ready' in text
    assert 'ABOUT 30 m off' in text                                                 # no geoid undulation was given


def test_a_dry_run_for_lattice_says_what_is_missing_and_never_prints_a_secret(monkeypatch, run_dir, capsys):
    monkeypatch.setenv('SANDBOXES_TOKEN', 'sbx-should-never-appear')
    code = go('--source', 'replay', '--run-dir', run_dir, '--sink', 'rest', '--dry-run')
    text = capsys.readouterr().out
    assert code == BC.EXIT_CONFIG and 'NOT READY' in text and 'LATTICE_TOKEN is not set' in text
    assert 'sbx-should-never-appear' not in text and 'sandbox_token=set' in text


def test_a_dry_run_for_lattice_with_everything_set_is_ready(monkeypatch, run_dir, capsys):
    monkeypatch.setenv('LATTICE_TOKEN', GOOD_TOKEN)
    monkeypatch.setenv('LATTICE_ENDPOINT', 'http://127.0.0.1:8089')
    code = go('--source', 'replay', '--run-dir', run_dir, '--sink', 'rest', '--dry-run')
    assert code == 0 and 'verdict: ready' in capsys.readouterr().out


# ------------------------------------------------------------------------------------------ exit codes
def test_a_missing_run_folder_exits_with_the_source_code(tmp_path, capsys):
    assert go('--source', 'replay', '--run-dir', tmp_path / 'nope', '--output', tmp_path / 'o.jsonl') == BC.EXIT_SOURCE
    assert 'does not exist' in capsys.readouterr().err


def test_a_replay_without_a_folder_exits_with_the_settings_code(capsys):
    assert go('--source', 'replay') == BC.EXIT_CONFIG
    assert '--run-dir' in capsys.readouterr().err


def test_lattice_without_an_address_exits_with_the_settings_code(run_dir, capsys):
    assert go('--source', 'replay', '--run-dir', run_dir, '--sink', 'rest') == BC.EXIT_CONFIG
    assert 'no Lattice address' in capsys.readouterr().err


def test_lattice_without_a_token_exits_with_the_settings_code(run_dir, capsys):
    assert go('--source', 'replay', '--run-dir', run_dir, '--sink', 'rest', '--endpoint', 'http://127.0.0.1:1') == BC.EXIT_CONFIG
    assert 'LATTICE_TOKEN' in capsys.readouterr().err


def test_a_fast_replay_into_lattice_is_refused_with_the_reason(run_dir, capsys):
    assert go('--source', 'replay', '--run-dir', run_dir, '--sink', 'rest', '--endpoint', 'http://127.0.0.1:1',
              '--speed', 5) == BC.EXIT_CONFIG
    assert 'real time' in capsys.readouterr().err


def test_a_mistyped_profile_key_exits_with_the_settings_code_and_a_hint(tmp_path, capsys):
    profile = tmp_path / 'p.yaml'
    profile.write_text('drone:\n  stale_aftre_s: 5\n')
    assert go('--profile', profile) == BC.EXIT_CONFIG
    assert 'stale_after_s' in capsys.readouterr().err


def test_a_bad_flag_exits_with_the_usage_code():
    with pytest.raises(SystemExit) as stopped:
        go('--no-such-flag')
    assert stopped.value.code == BC.EXIT_USAGE


def test_the_help_lists_the_profile_and_dry_run_flags():
    help_text = build_parser().format_help()
    assert '--profile' in help_text and '--dry-run' in help_text and '--source' in help_text


def test_an_unwritable_output_file_is_reported_not_a_traceback(tmp_path, run_dir, capsys):
    code = go('--source', 'replay', '--run-dir', run_dir, '--output', tmp_path / 'no_such_folder' / 'x' / '\0bad')
    assert code == BC.EXIT_SOURCE
    assert 'Traceback' not in capsys.readouterr().err


# ------------------------------------------------------------------------------------------ pieces
def test_the_log_filter_blanks_secrets_out_of_every_line():
    secrets = BC.Secrets('http://x', GOOD_TOKEN, '')
    record = logging.LogRecord('lattice_bridge.test', logging.ERROR, __file__, 1, 'refused %s for %s', (GOOD_TOKEN, 'me'), None)
    assert RedactFilter(secrets).filter(record) is True
    assert GOOD_TOKEN not in record.getMessage() and '***' in record.getMessage()


def test_the_log_filter_survives_a_broken_format_string():
    record = logging.LogRecord('x', logging.ERROR, __file__, 1, 'needs %s and %s', ('one',), None)
    assert RedactFilter(BC.Secrets('', 'abcdefghij', '')).filter(record) is True


def test_the_core_output_adapter_files_entities_and_refuses_junk():
    from support_net import uav_at
    publisher, sink, clock = make_publisher()
    out = PublisherOut(publisher)
    assert out.publish(uav_at(clock), clock.now()) is True
    assert out.publish({'not': 'an entity'}, clock.now()) is False
    assert publisher.submit({'not': 'an entity'}) == REJECTED


def test_flush_gives_up_at_once_when_nothing_can_go_out(monkeypatch):
    from lattice_bridge.sinks.base import AuthSinkError
    from support_net import uav_at
    publisher, sink, clock = make_publisher()
    sink.go_down(AuthSinkError('HTTP 401', status=401))
    out = PublisherOut(publisher)
    out.publish(uav_at(clock), clock.now())
    publisher.pump()
    out.flush(30.0)                                                     # would take 30 s if it waited for a miracle
