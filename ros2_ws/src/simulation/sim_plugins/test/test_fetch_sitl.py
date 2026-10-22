"""
sim_plugins/test/test_fetch_sitl.py -- PURPOSE: tests of simulation/setup/fetch_sitl.sh WITHOUT the internet:
argument handling and --dry-run (which never downloads), and real installs where a FAKE `curl` placed first on
the PATH serves a fake ArduCopter program and parameter files (or fails on purpose). Checks: the right URLs in
the right order (stable-X.Y.Z, then beta-X.Y.Z), the folder layout ~/sitl/versions/<V>/, SOURCE.txt, refusing a
non-Linux file, "already there" / --force, --from-file, --url, the x86_64-only rule, and that a failed download
leaves nothing behind. Needs bash (Linux / WSL).
"""

import os
import subprocess
from pathlib import Path

import pytest

from conftest import REPO, bash_available

SCRIPT = REPO / 'simulation' / 'setup' / 'fetch_sitl.sh'
pytestmark = pytest.mark.skipif(not bash_available(), reason='needs bash (Linux / WSL)')

FAKE_CURL = r'''#!/usr/bin/env bash
# fake curl for the fetch_sitl.sh tests: records every URL, writes a fake file to -o, fails on purpose when the
# URL matches $FAKE_CURL_FAIL (an extended regular expression).
out=""; url=""
while [ $# -gt 0 ]; do
  case "$1" in -o) shift; out="$1" ;; -*) ;; *) url="$1" ;; esac
  shift
done
echo "$url" >> "$FAKE_CURL_LOG"
if [ -n "${FAKE_CURL_FAIL:-}" ] && printf '%s' "$url" | grep -qE "$FAKE_CURL_FAIL"; then exit 22; fi
case "$url" in
  */arducopter)
    if [ "${FAKE_CURL_NOT_ELF:-0}" = 1 ]; then echo "<html>404</html>" > "$out"
    else printf '\177ELF\0\0\0\0ArduCopter V%s (abcdef12)\0' "${FAKE_CURL_VERSION:-4.7.1}" > "$out"; fi ;;
  *) echo "PARAM_FROM_FAKE 1" > "$out" ;;
esac
'''


@pytest.fixture
def env(tmp_path):
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    curl = fake_bin / 'curl'
    curl.write_text(FAKE_CURL)
    curl.chmod(0o755)
    home = tmp_path / 'home'
    home.mkdir()
    e = dict(os.environ, HOME=str(home), ASTRA_SITL_VERSIONS=str(tmp_path / 'versions'),
             PATH=f'{fake_bin}:{os.environ.get("PATH", "")}', FAKE_CURL_LOG=str(tmp_path / 'curl.log'))
    e.pop('ASTRA_FETCH_ARCH', None)
    return {'env': e, 'versions': tmp_path / 'versions', 'log': tmp_path / 'curl.log', 'tmp': tmp_path}


def fetch(env, *args, **extra):
    e = dict(env['env'], **{k: str(v) for k, v in extra.items()})
    return subprocess.run(['bash', str(SCRIPT), *args], env=e, capture_output=True, text=True, timeout=60)


def urls(env):
    return env['log'].read_text().split() if env['log'].exists() else []


# ---- usage -------------------------------------------------------------------------------------------

def test_help(env):
    r = fetch(env, '--help')
    assert r.returncode == 0 and 'PURPOSE' in r.stdout and '--dry-run' in r.stdout


@pytest.mark.parametrize('args', [(), ('4.7.1', '4.6.3'), ('--bogus',), ('4.7.1;rm',), ('--url',)])
def test_bad_usage_exit_2(env, args):
    r = fetch(env, *args)
    assert r.returncode == 2, (r.stdout, r.stderr)


def test_dry_run_shows_urls_and_downloads_nothing(env):
    r = fetch(env, '4.7.1', '--dry-run')
    assert r.returncode == 0
    assert 'Copter/stable-4.7.1/SITL_x86_64_linux_gnu/arducopter' in r.stdout
    assert 'Copter/beta-4.7.1/SITL_x86_64_linux_gnu/arducopter' in r.stdout
    assert 'ardupilot/Copter-4.7.1/Tools/autotest/default_params' in r.stdout
    assert str(env['versions'] / '4.7.1') in r.stdout
    assert urls(env) == [] and not env['versions'].exists()


def test_dry_run_names(env):
    r = fetch(env, 'latest', '--dry-run')
    assert 'Copter/latest/SITL_x86_64_linux_gnu/arducopter' in r.stdout and 'else master' in r.stdout
    r = fetch(env, 'Copter-4.6.3', '--dry-run')
    assert 'stable-4.6.3' in r.stdout and str(env['versions'] / '4.6.3') in r.stdout


# ---- installs with the fake curl ----------------------------------------------------------------------

def test_install_a_version(env):
    r = fetch(env, '4.7.1')
    assert r.returncode == 0, r.stderr
    dest = env['versions'] / '4.7.1'
    prog = dest / 'arducopter'
    assert prog.read_bytes().startswith(b'\x7fELF') and os.access(prog, os.X_OK)
    assert (dest / 'copter.parm').read_text().startswith('PARAM_FROM_FAKE')
    assert (dest / 'gazebo-iris.parm').is_file()
    source = (dest / 'SOURCE.txt').read_text()
    assert 'ArduCopter V4.7.1 (abcdef12)' in source and 'program_sha256:' in source
    assert urls(env)[0].endswith('Copter/stable-4.7.1/SITL_x86_64_linux_gnu/arducopter')
    assert any('Copter-4.7.1/Tools/autotest/default_params/copter.parm' in u for u in urls(env))
    assert not list(env['versions'].glob('.fetch_*'))                        # no temporary folder left


def test_falls_back_to_beta(env):
    r = fetch(env, '4.8.0', FAKE_CURL_FAIL='stable-4.8.0', FAKE_CURL_VERSION='4.8.0-beta2')
    assert r.returncode == 0, r.stderr
    assert 'beta-4.8.0' in (env['versions'] / '4.8.0' / 'SOURCE.txt').read_text()


def test_version_not_on_the_server(env):
    r = fetch(env, '9.9.9', FAKE_CURL_FAIL='arducopter$')
    assert r.returncode == 1 and '--list' in r.stderr
    assert not (env['versions'] / '9.9.9').exists() and not list(env['versions'].glob('.fetch_*'))


def test_refuses_a_file_that_is_not_a_program(env):
    r = fetch(env, '4.7.1', FAKE_CURL_NOT_ELF=1)
    assert r.returncode == 1 and 'not a Linux program' in r.stderr
    assert not (env['versions'] / '4.7.1').exists()


def test_latest_takes_parameter_files_of_the_version_inside(env):
    r = fetch(env, 'latest', FAKE_CURL_VERSION='4.8.1')
    assert r.returncode == 0, r.stderr
    assert any('Copter-4.8.1/Tools/autotest/default_params' in u for u in urls(env))


def test_parameter_files_fall_back_to_master_then_home_copy(env):
    r = fetch(env, '4.7.2', FAKE_CURL_FAIL='Copter-4.7.2/Tools')
    assert r.returncode == 0 and any('/master/Tools/' in u for u in urls(env))
    home_sitl = Path(env['env']['HOME']) / 'sitl'
    home_sitl.mkdir()
    (home_sitl / 'copter.parm').write_text('HOME_COPY 1\n')
    r = fetch(env, '4.7.3', FAKE_CURL_FAIL='default_params/copter.parm')
    assert r.returncode == 0 and 'using the copy from ~/sitl' in r.stderr
    assert (env['versions'] / '4.7.3' / 'copter.parm').read_text() == 'HOME_COPY 1\n'


def test_already_there_and_force(env):
    assert fetch(env, '4.7.1').returncode == 0
    env['log'].unlink()
    r = fetch(env, '4.7.1')
    assert r.returncode == 0 and 'already there' in r.stdout and urls(env) == []
    r = fetch(env, '4.7.1', '--force', FAKE_CURL_VERSION='4.7.1')
    assert r.returncode == 0 and len(urls(env)) == 3
    assert not (env['versions'] / '4.7.1.old').exists()


def test_from_file_and_url(env, fake_arducopter):
    mine = fake_arducopter(env['tmp'] / 'my_build' / 'arducopter', '4.6.9', '1234abcd')
    r = fetch(env, 'mybuild', '--from-file', str(mine), '--params-ref', 'Copter-4.6.3')
    assert r.returncode == 0, r.stderr
    source = (env['versions'] / 'mybuild' / 'SOURCE.txt').read_text()
    assert str(mine) in source and 'V4.6.9 (1234abcd)' in source
    r = fetch(env, 'ci', '--url', 'https://example.org/builds/77/arducopter')
    assert r.returncode == 0 and 'https://example.org/builds/77/arducopter' in urls(env)


def test_only_x86_64_downloads(env, fake_arducopter):
    r = fetch(env, '4.7.1', ASTRA_FETCH_ARCH='aarch64')
    assert r.returncode == 1 and 'only for x86_64' in r.stderr and urls(env) == []
    mine = fake_arducopter(env['tmp'] / 'arm' / 'arducopter', '4.6.3')
    r = fetch(env, 'arm-build', '--from-file', str(mine), ASTRA_FETCH_ARCH='aarch64')
    assert r.returncode == 0, r.stderr


def test_the_installed_version_is_found_by_the_autopilot_slot(env, monkeypatch):
    assert fetch(env, '4.7.1').returncode == 0
    monkeypatch.setenv('ASTRA_SITL_VERSIONS', str(env['versions']))
    from sim_plugins import autopilot as ap
    plan = ap.plan_autopilot(ap.AutopilotSettings(version='4.7.1'), write_params=False)
    assert plan.errors == [] and plan.version == '4.7.1 (abcdef12)'
    assert plan.stock_param_files(finder=lambda n: None)[0].endswith('4.7.1/copter.parm')
