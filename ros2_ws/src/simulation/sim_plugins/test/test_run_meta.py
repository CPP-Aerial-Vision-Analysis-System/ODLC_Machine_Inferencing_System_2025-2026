"""
sim_plugins/test/test_run_meta.py -- PURPOSE: unit tests of the "what really ran" recorder (run_meta.py) without a
simulator: the ArduCopter boot banner is read from a real-looking sim.log, AUTOPILOT_VERSION numbers decode, the
SITL command line and a FAKE /proc process table give the program, instance, home and --defaults files, a version
mismatch is noted, the detector record holds the model's name / size / SHA-256, the team-stack record holds the
checkout, and the command line writes run_meta/*.json with the documented exit codes (0, 3 = bad model).
"""

import hashlib
import json
from pathlib import Path

from conftest import write_fake_arducopter

from sim_plugins import run_meta as rm

SIM_LOG = """[INFO] [launch]: All log files can be found below /home/me/.ros/log
[mavproxy-5] Connect tcp:127.0.0.1:5760 source_system=255
[mavproxy-5] AP: ArduCopter V4.6.3 (3fc7011a)
[mavproxy-5] AP: 7b1d4b5c3c6e4f8b9d2a0e1f5a6b7c8d
[mavproxy-5] AP: Frame: QUAD/X
[mavproxy-5] online system 1
"""
ZIP_HEAD = b'PK\x03\x04' + b'\0' * 60


def fake_proc(root: Path, pid: int, argv, env) -> Path:
    """One process in a fake /proc: cmdline and environ as NUL-separated bytes (like Linux)."""
    d = root / str(pid)
    d.mkdir(parents=True)
    (d / 'cmdline').write_bytes(b'\0'.join(a.encode() for a in argv) + b'\0')
    (d / 'environ').write_bytes(b'\0'.join(e.encode() for e in env) + b'\0')
    return d


# ---- autopilot -------------------------------------------------------------------------------------------

def test_parse_banner_reads_the_last_banner_and_frame():
    b = rm.parse_banner(SIM_LOG + '[mavproxy-5] AP: ArduCopter V4.7.1-beta2 (abcdef12)\n')
    assert b['firmware'] == 'ArduCopter' and b['version'] == '4.7.1-beta2' and b['git'] == 'abcdef12'
    assert b['frame'] == 'QUAD/X'
    assert rm.parse_banner('nothing here') is None


def test_decode_autopilot_version_fields():
    assert rm.decode_sw_version((4 << 24) | (6 << 16) | (3 << 8) | 255) == {'version': '4.6.3', 'type': 'official',
                                                                           'type_raw': 255}
    assert rm.decode_sw_version((4 << 24) | (7 << 16) | (0 << 8) | 128)['type'] == 'beta'
    assert rm.decode_git(list(b'3fc7011a')) == '3fc7011a'
    assert rm.decode_git(list(b'abc\0\0\0\0\0')) == 'abc'


def test_parse_sitl_cmdline():
    info = rm.parse_sitl_cmdline(['/home/me/sitl/arducopter', '--model', 'JSON', '-I0', '--home',
                                  '36.2,-96.0,210,0', '--defaults', 'a.parm,b.parm', '-w'])
    assert info == {'binary': '/home/me/sitl/arducopter', 'defaults': ['a.parm', 'b.parm'], 'instance': 0,
                    'home': '36.2,-96.0,210,0', 'wipe': True}
    assert rm.parse_sitl_cmdline(['x', '-I', '2'])['instance'] == 2


def test_find_sitl_process_only_takes_this_session(tmp_path):
    proc = tmp_path / 'proc'
    fake_proc(proc, 100, ['/usr/bin/python3', 'other'], [])
    fake_proc(proc, 200, ['/x/arducopter', '--model', 'JSON', '-I0'], ['ASTRA_SIM_SESSION=someone-else'])
    fake_proc(proc, 300, ['/y/arducopter', '--model', 'JSON', '-I0'], ['ASTRA_SIM_SESSION=demo-1'])
    found = rm.find_sitl_process('demo-1', proc=str(proc))
    assert found['pid'] == 300 and found['argv'][0] == '/y/arducopter' and found['exe'] == ''
    assert rm.find_sitl_process('nobody', proc=str(proc)) is None
    assert rm.find_sitl_process('', proc=str(tmp_path / 'no_proc')) is None


def test_autopilot_meta_from_log_process_and_state(tmp_path):
    binary = write_fake_arducopter(tmp_path / 'sitl' / 'arducopter', version='4.6.3', commit='3fc7011a')
    parm = tmp_path / 'astra_sitl.parm'
    parm.write_text('SIM_SPEEDUP 1\n', encoding='utf-8')
    log = tmp_path / 'sim.log'
    log.write_text(SIM_LOG, encoding='utf-8')
    state = tmp_path / 'current_launch.yaml'
    state.write_text('autopilot: sitl\nsitl_bin: /nowhere/arducopter\nsitl_run_dir: /tmp/sitl_I0\nprofile: ""\n',
                     encoding='utf-8')
    proc = tmp_path / 'proc'
    fake_proc(proc, 42, [str(binary), '--model', 'JSON', '-I0', '--home', '1,2,3,0', '--defaults', str(parm)],
              ['ASTRA_SIM_SESSION=demo-7'])
    data = rm.autopilot_meta(str(log), str(state), 'demo-7', proc=str(proc))
    assert data['schema'] == 'astra.run_meta.autopilot.v1' and data['kind'] == 'sitl'
    assert data['reported']['version'] == '4.6.3' and data['reported']['git'] == '3fc7011a'
    assert data['binary']['path'] == str(binary) and data['binary']['compiled_version'].startswith('4.6.3')
    assert data['process']['pid'] == 42 and data['process']['instance'] == 0
    assert data['process']['defaults'][0]['sha256'] == hashlib.sha256(parm.read_bytes()).hexdigest()
    assert data['notes'] == []


def test_autopilot_meta_notes_a_version_mismatch_and_a_missing_banner(tmp_path):
    binary = write_fake_arducopter(tmp_path / 'arducopter', version='4.7.1')
    log = tmp_path / 'sim.log'
    log.write_text(SIM_LOG, encoding='utf-8')                       # the log says 4.6.3
    state = tmp_path / 'state.yaml'
    state.write_text(f'autopilot: sitl\nsitl_bin: {binary}\n', encoding='utf-8')
    data = rm.autopilot_meta(str(log), str(state), 'demo-x', proc=str(tmp_path / 'proc'))
    assert any('reported 4.6.3 but the program says 4.7.1' in n for n in data['notes'])
    assert any('SITL process was not found' in n for n in data['notes'])
    empty = tmp_path / 'empty.log'
    empty.write_text('', encoding='utf-8')
    data = rm.autopilot_meta(str(empty), '', '', proc=str(tmp_path / 'proc'))
    assert data['reported'] is None and any('banner' in n for n in data['notes'])


def test_external_autopilot_does_not_look_for_a_process(tmp_path):
    state = tmp_path / 'state.yaml'
    state.write_text('autopilot: external\n', encoding='utf-8')
    data = rm.autopilot_meta('', str(state), proc=str(tmp_path / 'proc'))
    assert data['kind'] == 'external' and data['process'] is None and data['binary'] is None


# ---- detector and team stack ------------------------------------------------------------------------------

def test_detector_meta_records_the_model(tmp_path):
    model = tmp_path / 'swap_test.pt'
    model.write_bytes(ZIP_HEAD + b'x' * 1000)
    data = rm.detector_meta('real', str(model))
    assert data['schema'] == 'astra.run_meta.detector.v1' and data['kind'] == 'real'
    assert data['model']['name'] == 'swap_test.pt' and data['model']['format'] == 'pytorch'
    assert data['model']['sha256'] == hashlib.sha256(model.read_bytes()).hexdigest()
    assert data['check']['ok'] is True and data['check']['loaded'] is False
    truth = rm.detector_meta('truth', str(model), text='answer key')
    assert truth['model'] is None and truth['text'] == 'answer key'


def test_team_stack_meta_of_a_plain_folder(tmp_path):
    (tmp_path / 'launcher.sh').write_bytes(b'#!/bin/bash\n')
    data = rm.team_stack_meta(str(tmp_path), 'ros2 launch sim_bringup team_stack.launch.py', 'tcp://127.0.0.1:5763')
    assert data['schema'] == 'astra.run_meta.team_stack.v1' and data['git'] is None
    assert data['launcher']['size_bytes'] == len(b'#!/bin/bash\n')
    assert data['fcu_url'] == 'tcp://127.0.0.1:5763' and 'not a git checkout' in data['notes'][0]


# ---- command line -----------------------------------------------------------------------------------------

def test_cli_writes_every_record(tmp_path, capsys):
    out = tmp_path / 'run' / 'run_meta'
    model = tmp_path / 'm.pt'
    model.write_bytes(ZIP_HEAD)
    log = tmp_path / 'sim.log'
    log.write_text(SIM_LOG, encoding='utf-8')
    assert rm.main(['detector', '--out', str(out / 'detector.json'), '--kind', 'real', '--model', str(model)]) == 0
    assert rm.main(['autopilot', '--out', str(out / 'autopilot.json'), '--sim-log', str(log)]) == 0
    assert rm.main(['team-stack', '--out', str(out / 'team_stack.json'), '--repo', str(tmp_path)]) == 0
    det = json.loads((out / 'detector.json').read_text(encoding='utf-8'))
    ap = json.loads((out / 'autopilot.json').read_text(encoding='utf-8'))
    assert det['model']['name'] == 'm.pt' and ap['reported']['version'] == '4.6.3'
    assert 'ArduCopter V4.6.3' in capsys.readouterr().out


def test_cli_bad_model_exits_3_and_still_writes(tmp_path, capsys):
    out = tmp_path / 'detector.json'
    code = rm.main(['detector', '--out', str(out), '--kind', 'real', '--model', str(tmp_path / 'missing.pt')])
    assert code == 3
    assert json.loads(out.read_text(encoding='utf-8'))['check']['ok'] is False
    assert '[run_meta] ERROR' in capsys.readouterr().err


def test_team_stack_meta_takes_the_commit_from_the_source_checkout(tmp_path, monkeypatch):
    copy = tmp_path / 'linux_copy'
    copy.mkdir()
    monkeypatch.setattr(rm, 'git_info', lambda repo: {'commit': 'abc123', 'branch': 'gazebo-sim', 'dirty': False}
                        if repo == '/src/checkout' else None)
    data = rm.team_stack_meta(str(copy), source_repo='/src/checkout')
    assert data['git']['commit'] == 'abc123' and data['git_from'] == '/src/checkout'
    assert 'has no .git' in data['notes'][0]
    assert rm.team_stack_meta(str(copy))['git'] is None


def test_git_info_keeps_the_commit_when_git_status_is_too_slow(tmp_path, monkeypatch):
    import subprocess
    (tmp_path / '.git').mkdir()

    def fake_run(cmd, **kw):
        if 'status' in cmd:
            raise subprocess.TimeoutExpired(cmd, kw.get('timeout'))
        out = 'abc123' if cmd[-1] == 'HEAD' and '--abbrev-ref' not in cmd else 'gazebo-sim'
        return subprocess.CompletedProcess(cmd, 0, stdout=out + '\n', stderr='')
    monkeypatch.setattr(rm.subprocess, 'run', fake_run)
    info = rm.git_info(str(tmp_path), dirty_timeout=0.1)
    assert info['commit'] == 'abc123' and info['branch'] == 'gazebo-sim'
    assert info['dirty'] is None and 'longer than 0.1 s' in info['dirty_note']
    assert rm.git_info(str(tmp_path / 'not_a_checkout')) is None
