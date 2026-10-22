"""
sim_plugins/test/test_profile_tool.py -- PURPOSE: unit tests of the command line (profile_tool.py): every
sub-command's output and exit code (0 fine, 1 problem, 2 bad usage), and that `shell` prints variables bash can
`eval` exactly as demo.sh / start_sim.sh do.
"""

import os
import subprocess

import pytest

from sim_plugins import profile_tool

from conftest import PROFILES, bash_available


def run(capsys, *argv):
    code = profile_tool.main(['--profiles-dir', str(PROFILES), *argv])
    out = capsys.readouterr()
    return code, out.out, out.err


def test_list_and_validate(sim_home, capsys):
    code, out, _ = run(capsys, 'list')
    assert code == 0 and 'example_detector' in out and 'BROKEN' not in out
    code, out, _ = run(capsys, 'validate')
    assert code == 0 and out.count(' OK ') >= 9


def test_validate_reports_a_broken_profile(sim_home, capsys, tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('camera: {model: a9}\n')
    code, out, _ = run(capsys, 'validate', str(bad))
    assert code == 1 and 'camera.model' in out


def test_plan_and_resolve(sim_home, capsys):
    code, out, _ = run(capsys, 'plan', '--profile', 'ardupilot_version')
    assert code == 0 and 'PROFILE ardupilot_version' in out and 'sitl_version:=4.7.1' in out
    code, out, _ = run(capsys, 'plan', '--profile', 'ardupilot_version', '--check')
    assert code == 1 and 'fetch_sitl.sh 4.7.1' in out                      # not downloaded in the test home
    code, out, _ = run(capsys, 'resolve', '--profile', 'truth_detector', '--set', 'camera.model=a8mini')
    assert code == 0 and 'kind: truth' in out and 'model: a8mini' in out


def test_bad_profile_exit_code(sim_home, capsys):
    code, _, err = run(capsys, 'plan', '--profile', 'does_not_exist')
    assert code == 1 and 'not found' in err


def test_contract(capsys):
    code, out, _ = run(capsys, 'contract', '--slot', 'detector')
    assert code == 0 and '/image_detection' in out
    code, out, _ = run(capsys, 'contract', '--markdown')
    assert code == 0 and '| Topic |' in out
    code, _, err = run(capsys, 'contract', '--slot', 'nope')
    assert code == 2 and 'unknown slot' in err


def test_shell_for_demo_without_checks(sim_home, capsys):
    code, out, err = run(capsys, 'shell', '--for', 'demo', '--profile', 'example_detector', '--no-check',
                         '--no-write')
    assert code == 0 and 'DETECTOR=custom' in out and 'PLUGIN_STACK_ARGS=(--skip-node new_od' in out
    assert not (sim_home['home'] / 'generated').exists()                   # --no-write wrote nothing


def test_shell_writes_the_resolved_profile(sim_home, capsys):
    code, out, _ = run(capsys, 'shell', '--for', 'demo', '--profile', 'example_listener', '--no-check')
    assert code == 0
    written = list((sim_home['home'] / 'generated').glob('profile_example_listener_*.yaml'))
    assert len(written) == 1 and f'profile:={written[0]}' in out


def test_shell_stops_on_missing_things(sim_home, capsys):
    code, out, err = run(capsys, 'shell', '--for', 'demo', '--profile', 'ardupilot_version')
    assert code == 1 and out == '' and 'missing on this computer' in err


@pytest.mark.skipif(not bash_available(), reason='needs bash')
def test_shell_output_is_evaluated_by_bash(sim_home):
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([str(PROFILES.parents[1] / 'ros2_ws' / 'src' / 'simulation'
                                                          / 'sim_plugins'), os.environ.get('PYTHONPATH', '')]))
    script = ('eval "$(python3 -m sim_plugins.profile_tool --profiles-dir "$P" shell --for demo '
              '--profile example_detector --no-check --no-write --set "world.name=runway")" && '
              'echo "$WORLD|$DETECTOR|${#PLUGIN_STACK_ARGS[@]}|${PLUGIN_STACK_ARGS[3]}"')
    out = subprocess.run(['bash', '-c', script], env=dict(env, P=str(PROFILES)), capture_output=True, text=True,
                         check=True).stdout.strip()
    assert out == 'runway|custom|4|example_detector'
