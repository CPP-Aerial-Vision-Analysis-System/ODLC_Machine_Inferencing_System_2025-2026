"""
simulation/docker/test/test_check_host.py -- PURPOSE: check that the Docker doctor (simulation/docker/check_host.sh)
finds the first-run problems it promises and prints the fix, and that the helpers it relies on behave: Windows line
endings in scripts (and their repair without leaving git "modified" files), a missing buildx, a too-old Compose, a
password helper that does not exist, an image that runs as root or is older than docker/Dockerfile, a leftover
astra_net network or astra-sim container, a taken 192.168.144.0/24 subnet (also by a wider Docker network; a wider
VPN route is only a warning), Docker's "permission denied"; plus the labels and proxy settings build_images.sh passes
to `docker build`, the mount folders compose_up.sh creates, the subnet arithmetic, the "Linux-side copy is older"
count (tool caches do not count) and the user-id advice for accounts below 1000 (no rebuild that cannot help).
No Docker daemon, image, network or GPU is used: `docker` and `ip` are small stand-in scripts on PATH, the
repository is a throwaway git repo in a temporary folder, and the internet checks are switched off.
Run inside Linux (WSL Ubuntu or a Linux PC):  python3 -m pytest -q -p no:cacheprovider simulation/docker/test
"""

import os
from pathlib import Path
import subprocess
import sys

import pytest


DOCKER = Path(__file__).resolve().parents[1]
DOCTOR = DOCKER / 'check_host.sh'

pytestmark = pytest.mark.skipif(not sys.platform.startswith('linux'), reason='the Docker scripts run in Linux')

# A stand-in `docker` that answers like a working Docker Engine. STUB_* variables change single answers.
STUB_DOCKER = r'''#!/bin/bash
case "$1" in
  info)
    if [ -n "${STUB_INFO_ERR:-}" ]; then echo "$STUB_INFO_ERR" >&2; exit 1; fi
    case "$*" in
      *Runtimes*) echo "runc ${STUB_RUNTIMES:-}" ;;
      *ServerVersion*) echo "${STUB_SV:-29.0.0}|Ubuntu 22.04.5 LTS|/var/lib/docker||" ;;
      *) echo "Server: stub" ;;
    esac ;;
  buildx) [ -n "${STUB_NO_BUILDX:-}" ] && exit 1; echo "github.com/docker/buildx v0.20.0 0000000" ;;
  compose) case "$*" in *"version --short"*) echo "${STUB_COMPOSE:-2.29.0}" ;; esac; exit 0 ;;
  image)
    img="$3"; fmt="${5:-}"
    case " ${STUB_IMAGES:-} " in *" $img "*) ;; *) exit 1 ;; esac
    case "$fmt" in
      *Created*) echo "2026-10-30T10:00:00Z" ;;
      *Config.User*) echo "${STUB_USER:-ubuntu}" ;;
      *org.astra.inputs*) echo "${STUB_INPUTS:-}" ;;
      *org.astra.uid*) echo "${STUB_UID_LABEL:-}" ;;
      *org.astra.torch*) echo "cpu" ;;
    esac ;;
  ps) if [ "${2:-}" = -a ]; then [ -n "${STUB_PS_A:-}" ] && echo "$STUB_PS_A"; else [ -n "${STUB_PS:-}" ] && echo "$STUB_PS"; fi; exit 0 ;;
  network)
    case "${2:-}" in
      ls) echo bridge; [ -n "${STUB_NET:-}" ] && echo astra_net; [ -n "${STUB_OTHER_NET:-}" ] && echo "${STUB_OTHER_NET%% *}" ;;
      inspect)
        # STUB_OTHER_NET="NAME SUBNET": one more Docker network, from another project
        if [ -n "${STUB_OTHER_NET:-}" ] && [ "${3:-}" = "${STUB_OTHER_NET%% *}" ]; then echo "${STUB_OTHER_NET#* } "; exit 0; fi
        [ "${3:-}" = astra_net ] && [ -n "${STUB_NET:-}" ] || exit 1
        case "$*" in *Subnet*) echo "192.168.144.0/24 " ;; *compose.network*) echo "${STUB_NET_LABEL:-}" ;; *Containers*) echo 0 ;; esac ;;
    esac ;;
  context) echo default ;;
esac
exit 0
'''

# A stand-in `ip`: no routes and no addresses unless STUB_ROUTE says otherwise.
STUB_IP = r'''#!/bin/bash
case "$*" in
  *route*) [ -n "${STUB_ROUTE:-}" ] && echo "$STUB_ROUTE" ;;
  *addr*) echo "    inet 127.0.0.1/8 scope host lo" ;;
esac
exit 0
'''


def git(repo, *args):
    """Run git in REPO quietly (fails the test on error) and return its output."""
    return subprocess.run(['git', '-C', str(repo), *args], check=True, text=True, capture_output=True).stdout


@pytest.fixture()
def sandbox(tmp_path):
    """A temporary HOME, data folder, stub commands on PATH and a small LF git checkout with ros2_ws/src."""
    if subprocess.run(['git', '--version'], capture_output=True).returncode != 0:
        pytest.skip('git is not installed')
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    for name, text in (('docker', STUB_DOCKER), ('ip', STUB_IP)):
        (bin_dir / name).write_text(text, encoding='utf-8')
        (bin_dir / name).chmod(0o755)
    home = tmp_path / 'home'
    home.mkdir()
    repo = tmp_path / 'repo'
    (repo / 'ros2_ws' / 'src' / 'pkg').mkdir(parents=True)
    (repo / 'ros2_ws' / 'src' / 'pkg' / 'tool.py').write_bytes(b'#!/usr/bin/env python3\nprint(1)\n')
    (repo / 'run.sh').write_bytes(b'#!/bin/bash\necho hi\n')
    # Committed WITH CRLF on purpose: the repo's own state; the doctor must not blame the clone for it.
    (repo / 'ros2_ws' / 'src' / 'pkg' / 'old.py').write_bytes(b'#!/usr/bin/env python3\r\nprint(2)\r\n')
    # Only staged (git's index), never committed: everything the doctor reads (ls-files, grep --cached) is there.
    subprocess.run(['git', 'init', '-q', str(repo)], check=True)
    git(repo, 'config', 'core.autocrlf', 'false')
    git(repo, 'add', '-A')
    env = dict(os.environ, PATH=f'{bin_dir}:{os.environ["PATH"]}', HOME=str(home),
               ASTRA_DOCKER_DATA=str(tmp_path / 'data'), ASTRA_CHECK_OFFLINE='1')
    for key in [k for k in env if k.startswith('STUB_') or k == 'DOCKER_HOST']:
        del env[key]
    return {'repo': repo, 'env': env, 'home': home, 'tmp': tmp_path}


def inputs_hash():
    """The fingerprint build_images.sh would store for the docker/ files of this repo."""
    out = subprocess.run(['bash', '-c', 'source "$1" && astra_docker_inputs_hash', '_', str(DOCKER / '_host_common.sh')],
                         check=True, text=True, capture_output=True)
    return out.stdout.strip()


def doctor(sb, *args, **stub):
    """Run the doctor on the sandbox repo with extra STUB_* settings; return (exit code, output)."""
    env = dict(sb['env'], **{f'STUB_{k.upper()}': v for k, v in stub.items()})
    result = subprocess.run(['bash', str(DOCTOR), '--repo', str(sb['repo']), *args], env=env, text=True,
                            capture_output=True, timeout=180)
    return result.returncode, result.stdout + result.stderr


def fail_lines(output):
    return [line for line in output.splitlines() if '[FAIL]' in line]


def modified_in_worktree(repo):
    """Paths `git status` shows as modified in the working tree (second status letter M)."""
    return [line[3:] for line in git(repo, 'status', '--porcelain').splitlines() if line[1:2] == 'M']


def test_help_and_unknown_option(sandbox):
    """--help prints the purpose header; a wrong option is refused with a pointer to --help."""
    result = subprocess.run(['bash', str(DOCTOR), '--help'], text=True, capture_output=True, timeout=30)
    assert result.returncode == 0
    assert 'doctor' in result.stdout and '--fix-line-endings' in result.stdout
    code, out = doctor(sandbox, '--bogus')
    assert code == 1 and 'unknown option --bogus' in out


def test_healthy_engine_passes(sandbox):
    """A working engine with up-to-date images built for this user: no [FAIL], exit code 0."""
    images = 'astra-dev:latest astra-sitl:latest astra-sim:latest'
    code, out = doctor(sandbox, images=images, inputs=inputs_hash(), uid_label=str(os.getuid()))
    assert fail_lines(out) == [], out
    assert code == 0, out
    assert 'up to date with docker/' in out
    assert 'Nothing blocks a Docker run here' in out


def test_windows_line_endings_found_and_repaired(sandbox):
    """A script turned into CRLF by a Windows clone fails the check; --fix-line-endings repairs it and git stays clean."""
    repo = sandbox['repo']
    tool = repo / 'ros2_ws' / 'src' / 'pkg' / 'tool.py'
    tool.write_bytes(tool.read_bytes().replace(b'\n', b'\r\n'))
    assert modified_in_worktree(repo) == ['ros2_ws/src/pkg/tool.py']
    code, out = doctor(sandbox)
    assert code == 1
    assert any('1 script(s) have Windows line endings' in line and 'tool.py' in line for line in fail_lines(out)), out
    assert 'old.py' not in ' '.join(fail_lines(out))            # committed with CRLF: not the clone's fault
    assert '--fix-line-endings' in out
    fix = subprocess.run(['bash', str(DOCTOR), '--repo', str(repo), '--fix-line-endings'], env=sandbox['env'],
                         text=True, capture_output=True, timeout=120)
    assert fix.returncode == 0, fix.stdout + fix.stderr
    assert tool.read_bytes() == b'#!/usr/bin/env python3\nprint(1)\n'
    assert (repo / 'ros2_ws' / 'src' / 'pkg' / 'old.py').read_bytes().count(b'\r\n') == 2   # left as committed
    assert modified_in_worktree(repo) == []                   # git does not list the repaired file as "modified"
    code, out = doctor(sandbox)
    assert 'line endings' in out and not any('Windows line endings' in line for line in fail_lines(out))


def test_crlf_shell_script_outside_git(sandbox, tmp_path):
    """A folder without git (a ZIP download): a CRLF shell script is still found, with the one-file repair."""
    plain = tmp_path / 'plain'
    (plain / 'ros2_ws' / 'src').mkdir(parents=True)
    (plain / 'go.sh').write_bytes(b'#!/bin/bash\r\necho hi\r\n')
    result = subprocess.run(['bash', str(DOCTOR), '--repo', str(plain)], env=sandbox['env'], text=True,
                            capture_output=True, timeout=120)
    assert result.returncode == 1
    assert any('shell script(s)' in line and 'go.sh' in line for line in fail_lines(result.stdout)), result.stdout
    assert "sed -i 's/\\r$//' FILE" in result.stdout


@pytest.mark.parametrize('stub,expected', [
    ({'no_buildx': '1'}, 'buildx'),
    ({'compose': '2.12.2'}, 'Docker Compose 2.12.2 is too old'),
    ({'images': 'astra-sim:latest', 'user': 'root'}, 'runs as root'),
    ({'net': '1', 'net_label': ''}, 'Docker network "astra_net" exists that compose did not make'),
    ({'ps_a': 'astra-sim someotherproject'}, 'a container called astra-sim exists'),
    ({'route': '192.168.144.0/24 dev eth1 proto kernel scope link src 192.168.144.20'}, 'already used on this computer'),
    # a Docker network that COVERS the drone network (Docker's own pool hands out 192.168.144.0/20)
    ({'other_net': 'otherproj_default 192.168.144.0/20'}, 'already used on this computer'),
    ({'info_err': 'permission denied while trying to connect to the Docker daemon socket'}, 'may not use Docker yet'),
])
def test_blocking_problems_fail_with_a_fix(sandbox, stub, expected):
    """Each problem that would stop a build or a run is a [FAIL] line followed by the line to type."""
    code, out = doctor(sandbox, **stub)
    assert code == 1, out
    hits = [line for line in fail_lines(out) if expected in line]
    assert hits, out
    lines = out.splitlines()
    assert lines[lines.index(hits[0]) + 1].strip().startswith('->'), out


def test_unrelated_routes_and_networks_pass(sandbox):
    """The default route, the WSL network and a Docker network next door (192.168.145.0/24) do not block anything."""
    routes = 'default via 172.26.0.1 dev eth0 proto kernel\n172.26.0.0/20 dev eth0 proto kernel scope link src 172.26.1.2'
    code, out = doctor(sandbox, route=routes, other_net='neighbour_default 192.168.145.0/24')
    assert not any('drone network' in line for line in fail_lines(out)), out
    assert 'is free for the containers' in out
    assert 'wider network route' not in out


def test_wider_route_is_a_warning_not_a_failure(sandbox):
    """A VPN route 192.168.0.0/16 covers the drone network but does not stop the containers: [WARN], not [FAIL]."""
    code, out = doctor(sandbox, route='192.168.0.0/16 dev tun0 scope link')
    assert not any('drone network' in line for line in fail_lines(out)), out
    assert any('[WARN]' in line and 'wider network route' in line and '192.168.0.0/16' in line
               for line in out.splitlines()), out


def helpers(snippet, env=None):
    """Run a bash SNIPPET with _host_common.sh loaded; return its stdout + stderr."""
    result = subprocess.run(['bash', '-c', f'source "$1"\n{snippet}', '_', str(DOCKER / '_host_common.sh')],
                            env=env, text=True, capture_output=True, timeout=60)
    return result.stdout + result.stderr


@pytest.mark.parametrize('net,overlaps', [
    ('192.168.144.0/24', True), ('192.168.144.25', True), ('192.168.144.128/25', True),
    ('192.168.144.0/20', True), ('192.168.0.0/16', True), ('192.168.145.0/24', False),
    ('192.168.143.255/32', False), ('10.0.0.0/8', False), ('fd00::/64', False), ('junk', False),
])
def test_cidr_overlap(net, overlaps):
    """Subnet arithmetic: any shared address counts, inside or around 192.168.144.0/24; IPv6 and junk never do."""
    out = helpers(f'astra_cidr_overlap {net} && echo YES || echo NO').strip()
    assert out == ('YES' if overlaps else 'NO')


def test_copy_check_ignores_tool_caches():
    """The 'Linux-side copy is OLDER' count skips caches and folders, counts only real files."""
    itemized = '\n'.join([
        '>f+++++++++ ros2_ws/src/main/src/main_controller.py',
        '>f.st...... simulation/scripts/demo.sh',
        '>f+++++++++ .ruff_cache/0.6.9/12345',
        '>f+++++++++ ros2_ws/src/simulation/sim_demo/.pytest_cache/v/cache/nodeids',
        '>f+++++++++ .claude/settings.local.json',
        'cd+++++++++ ros2_ws/src/new_folder/',
        '>f+++++++++ docs/a file with spaces.md',
    ])
    out = helpers('printf "%s\\n" "$ITEMS" | astra_count_copy_changes', env=dict(os.environ, ITEMS=itemized))
    assert out.strip() == '3'
    assert helpers("printf '' | astra_count_copy_changes").strip() == '0'


def test_uid_below_1000_gets_no_useless_rebuild_advice(sandbox):
    """A user id below 1000 always gets images with id 1000: explained, without a 'rebuild' that cannot help."""
    fake_id = 'id() { case "$1" in -u) echo 500 ;; -g) echo 500 ;; *) command id "$@" ;; esac; }'
    env = dict(sandbox['env'], STUB_IMAGES='astra-sim:latest', STUB_UID_LABEL='1000')
    out = helpers(f'{fake_id}\necho "build uid: $(astra_build_uid)"\nastra_check_image_user astra-sim:latest\n'
                  'echo "mismatch: ${ASTRA_UID_MISMATCH:-}"', env=env)
    assert 'build uid: 1000' in out
    assert 'below 1000' in out and 'nothing to' in out
    assert 'build_images.sh' not in out                 # no rebuild advice
    assert 'mismatch: 1000' in out                      # X11 still needs the wider xhost rule
    env['STUB_UID_LABEL'] = '4242'                      # a really foreign image: rebuild advice as before
    out = helpers(f'{fake_id}\nastra_check_image_user astra-sim:latest', env=env)
    assert 'built for user id 4242' in out and 'build_images.sh' in out


def test_missing_password_helper_fails(sandbox):
    """A credsStore naming a helper that is not installed breaks every pull: reported with a safe removal command."""
    docker_cfg = sandbox['home'] / '.docker'
    docker_cfg.mkdir()
    (docker_cfg / 'config.json').write_text('{"auths": {}, "credsStore": "astra-test-missing"}', encoding='utf-8')
    code, out = doctor(sandbox)
    assert code == 1
    assert any('astra-test-missing' in line for line in fail_lines(out)), out
    assert "d.pop('credsStore',None)" in out


def test_stale_and_foreign_images_warn(sandbox):
    """An image older than docker/Dockerfile, or built for another user id, is a warning with the rebuild command."""
    code, out = doctor(sandbox, images='astra-dev:latest', inputs='0123456789abcdef', uid_label='4242')
    assert 'astra-dev:latest' in out and 'is OLDER than docker/Dockerfile' in out
    assert 'was built for user id 4242' in out
    assert 'build_images.sh dev' in out


def test_build_images_dry_run_labels_and_proxy(sandbox):
    """build_images.sh stores the user id and the docker/ fingerprint in the image and forwards proxy settings."""
    env = dict(sandbox['env'], https_proxy='http://proxy.example:3128')
    result = subprocess.run(['bash', str(DOCKER / 'build_images.sh'), 'dev', '--dry-run', '--torch', 'cpu'], env=env,
                            text=True, capture_output=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr
    command = next(line for line in result.stdout.splitlines() if '+ docker build ' in line)
    assert f'org.astra.inputs={inputs_hash()}' in command
    assert 'org.astra.uid=' in command
    assert '--build-arg https_proxy' in command


def test_compose_up_creates_mount_folders_first(sandbox):
    """compose_up.sh makes ros2_ws/build, install, log before Docker could create them owned by root."""
    (sandbox['repo'] / 'simulation' / 'docker').mkdir(parents=True)
    (sandbox['repo'] / 'simulation' / 'docker' / 'container_sim.sh').write_text('', encoding='utf-8')
    # --force: a native simulator may be running on this computer during the test; the dry run starts nothing.
    result = subprocess.run(['bash', str(DOCKER / 'compose_up.sh'), 'runway', '--dry-run', '--headless', '--no-gpu',
                             '--force', '--repo', str(sandbox['repo'])], env=sandbox['env'], text=True,
                            capture_output=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    for name in ('build', 'install', 'log'):
        assert f"mkdir -p {sandbox['repo']}/ros2_ws/{name}" in result.stdout
