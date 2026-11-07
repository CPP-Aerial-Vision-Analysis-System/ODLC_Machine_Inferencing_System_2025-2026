"""
sim_plugins/test/test_team_overlay.py -- PURPOSE: test sim_plugins/scripts/team_overlay.sh, the wrapper demo.sh
--team-repo puts in front of team_stack.launch.py: bad usage and an unbuilt workspace exit 2 with the fix, and (with
ROS 2 on the PATH) two FAKE workspaces show that the team checkout's packages win over the simulator's copies while
sim_bringup still comes from the simulator, then the command runs. Needs bash (Linux / WSL); skipped on Windows.
"""

import os
import shutil
import subprocess
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'team_overlay.sh'
posix_only = pytest.mark.skipif(os.name != 'posix' or not Path('/bin/bash').exists(), reason='needs bash (Linux/WSL)')


def fake_ws(root: Path, packages) -> Path:
    """A workspace whose install/local_setup.bash puts its fake packages first on AMENT_PREFIX_PATH."""
    install = root / 'install'
    lines = ['_here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"']
    for pkg in packages:
        index = install / pkg / 'share' / 'ament_index' / 'resource_index' / 'packages'
        index.mkdir(parents=True)
        (index / pkg).write_text('', encoding='utf-8')
        lines.append(f'export AMENT_PREFIX_PATH="$_here/{pkg}${{AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}}"')
    (install / 'local_setup.bash').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return root


def run(*args, env=None):
    return subprocess.run(['bash', str(SCRIPT), *args], capture_output=True, text=True, timeout=60, env=env)


@posix_only
def test_usage_errors_exit_2_with_the_fix(tmp_path):
    r = run()
    assert r.returncode == 2 and 'both --sim-ws and --team-ws' in r.stderr and 'Fix:' in r.stderr
    r = run('--sim-ws', str(tmp_path / 'a'), '--team-ws', str(tmp_path / 'b'), '--', 'true')
    assert r.returncode == 2 and 'not built' in r.stderr and 'colcon build' in r.stderr


@posix_only
def test_help_prints_the_header():
    r = run('--help')
    assert r.returncode == 0 and 'team_overlay.sh' in r.stdout and 'HOW TO RUN' in r.stdout


@posix_only
@pytest.mark.skipif(shutil.which('ros2') is None, reason='needs ROS 2 (source sim_env.sh)')
def test_the_team_checkout_wins_and_sim_bringup_stays(tmp_path):
    sim = fake_ws(tmp_path / 'sim_ws', ['main', 'detection', 'sim_bringup'])
    team = fake_ws(tmp_path / 'team_ws', ['main', 'detection'])
    env = dict(os.environ)
    r = run('--sim-ws', str(sim), '--team-ws', str(team), '--', 'bash', '-c', 'echo RAN', env=env)
    assert r.returncode == 0, r.stderr
    assert f'main comes from {team}/install/main' in r.stdout
    assert f'sim_bringup comes from {sim}/install/sim_bringup' in r.stdout
    assert 'WARNING' not in r.stdout and r.stdout.strip().endswith('RAN')
