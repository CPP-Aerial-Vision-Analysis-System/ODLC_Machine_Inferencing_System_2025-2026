"""
sim_plugins/run_meta.py -- PURPOSE: write down WHICH PIECES REALLY RAN in a simulator run, so a run can be compared
with another one ("was it the same ArduPilot? the same model? the same Jetson code?") and the run summary can
show it. demo.sh calls it and the files land in the run folder:

    <run>/run_meta/autopilot.json   which ArduCopter flew: the version the autopilot ITSELF reported (its boot
                                    banner "ArduCopter V4.6.3 (3fc7011a)" as MAVProxy logged it in sim.log, or the
                                    MAVLink message AUTOPILOT_VERSION with --mavlink), the program that ran (path,
                                    size, SHA-256, the version compiled into it), its exact --defaults parameter
                                    files (each with SHA-256), instance and home, read from the running process
    <run>/run_meta/detector.json    which detector flew: kind (real / truth / custom), and for the team's new_od the
                                    model file (name, path, size, SHA-256, format) with model_check's results
    <run>/run_meta/team_stack.json  which Jetson code flew: the checkout, its git commit (when it is a git
                                    checkout), launcher.sh's SHA-256, TRACKER_CMD and FCU_URL

Every file has "schema": "astra.run_meta.<what>.v1" and "written" (local time). The format is small on purpose
and only grows by NEW keys (readers must ignore keys they do not know).

    python3 -m sim_plugins.run_meta autopilot --out DIR/run_meta/autopilot.json --sim-log DIR/sim.log \\
            [--state ~/.astra_sim/state/current_launch.yaml] [--session ID] [--wait 20] [--mavlink udpin:127.0.0.1:14550]
    python3 -m sim_plugins.run_meta detector --out FILE --kind real --model ~/.astra_sim/models/yolo26m.pt [--load]
    python3 -m sim_plugins.run_meta detector --out FILE --kind truth|custom --text "what runs"
    python3 -m sim_plugins.run_meta team-stack --out FILE --repo DIR [--tracker-cmd CMD] [--fcu-url URL] \\
            [--source-repo GIT_CHECKOUT]        (default $ASTRA_SOURCE_REPO: where a copy without .git came from)
Exit codes: 0 = written (missing details are written as null with a note), 3 = the detector model cannot be used
(detector --kind real only; the file is still written), 1 = the file could not be written, 2 = bad usage.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from . import model_check

SCHEMA_PREFIX = 'astra.run_meta'

#: The boot banner ArduPilot sends (MAVProxy logs it as "AP: ArduCopter V4.6.3 (3fc7011a)").
BANNER_RE = re.compile(r'\b(ArduCopter|ArduPlane|ArduRover|ArduSub|Blimp|AntennaTracker)\s+'
                       r'V(\d+\.\d+\.\d+[0-9A-Za-z.\-]*)\s*\(([0-9a-f]{6,40})\)')
FRAME_RE = re.compile(r'\bFrame:\s*([A-Z0-9_/ ]+?)\s*$', re.M)

#: AUTOPILOT_VERSION flight_sw_version's last byte: the firmware type.
FW_TYPES = {0: 'dev', 64: 'alpha', 128: 'beta', 192: 'rc', 255: 'official'}


def now_text() -> str:
    """Local time as text (for 'written')."""
    return time.strftime('%Y-%m-%d %H:%M:%S')


def write_json(path: str, data: Dict) -> None:
    """Write one run_meta file (folders created)."""
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, indent=2, sort_keys=False) + '\n', encoding='utf-8')


def file_fingerprint(path: str) -> Dict:
    """{path, size_bytes, sha256} of a file ({path, missing: true} when it is not there)."""
    p = Path(path)
    if not p.is_file():
        return {'path': str(path), 'missing': True}
    try:
        return {'path': str(p), 'size_bytes': p.stat().st_size, 'sha256': model_check.sha256_of(str(p))}
    except OSError as exc:
        return {'path': str(p), 'error': str(exc)}


# ---------------------------------------------------------------------------------------------
# Autopilot
# ---------------------------------------------------------------------------------------------

def parse_banner(text: str) -> Optional[Dict]:
    """The LAST firmware banner in a log text: {'firmware', 'version', 'git', 'banner', 'frame'} or None."""
    found = list(BANNER_RE.finditer(text or ''))
    if not found:
        return None
    m = found[-1]
    out = {'firmware': m.group(1), 'version': m.group(2), 'git': m.group(3), 'banner': m.group(0)}
    frames = FRAME_RE.findall(text or '')
    if frames:
        out['frame'] = frames[-1].strip()
    return out


def decode_sw_version(value: int) -> Dict:
    """AUTOPILOT_VERSION.flight_sw_version (major<<24 | minor<<16 | patch<<8 | type) as {'version', 'type'}."""
    value = int(value)
    major, minor, patch, kind = (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF
    # type_raw kept as the number too: the MAVLink FIRMWARE_VERSION_TYPE the firmware was BUILT with (a SITL
    # program built from a release branch can say beta = 128 although its banner shows a release number)
    return {'version': f'{major}.{minor}.{patch}', 'type': FW_TYPES.get(kind, str(kind)), 'type_raw': kind}


def decode_git(custom: Sequence[int]) -> str:
    """AUTOPILOT_VERSION.flight_custom_version (8 bytes of the git hash as ASCII) as text."""
    try:
        return bytes(int(b) & 0xFF for b in custom).split(b'\0', 1)[0].decode('ascii', 'replace')
    except (TypeError, ValueError):
        return ''


def parse_sitl_cmdline(argv: Sequence[str]) -> Dict:
    """The interesting parts of an arducopter command line: binary, instance, home, --defaults files."""
    argv = list(argv)
    out: Dict = {'binary': argv[0] if argv else '', 'defaults': [], 'instance': None, 'home': '', 'wipe': False}
    i = 1
    while i < len(argv):
        word = argv[i]
        nxt = argv[i + 1] if i + 1 < len(argv) else ''
        if word in ('-I', '--instance'):
            out['instance'], i = nxt, i + 1
        elif word.startswith('-I') and word[2:].isdigit():
            out['instance'] = word[2:]
        elif word == '--home':
            out['home'], i = nxt, i + 1
        elif word == '--defaults':
            out['defaults'], i = [p for p in nxt.split(',') if p], i + 1
        elif word.startswith('--defaults='):
            out['defaults'] = [p for p in word.split('=', 1)[1].split(',') if p]
        elif word in ('-w', '--wipe'):
            out['wipe'] = True
        i += 1
    try:
        out['instance'] = int(out['instance']) if out['instance'] is not None else None
    except ValueError:
        pass
    return out


def find_sitl_process(session: str = '', proc: str = '/proc') -> Optional[Dict]:
    """The running ArduPilot SITL of a simulator session: {'pid', 'argv', 'exe'} or None.

    A SITL process is one whose command line has "--model JSON" (sim.launch.py starts it like that); with a
    session id only processes carrying ASTRA_SIM_SESSION=<session> count (never another person's simulator).
    """
    root = Path(proc)
    if not root.is_dir():
        return None
    for d in sorted(root.iterdir(), key=lambda p: p.name):
        if not d.name.isdigit():
            continue
        try:
            argv = [a for a in (d / 'cmdline').read_bytes().split(b'\0') if a]
            argv = [a.decode('utf-8', 'replace') for a in argv]
        except OSError:
            continue
        if '--model' not in argv or 'JSON' not in argv or not argv:
            continue
        if 'wait_then_exec' in ' '.join(argv[:3]):
            continue                                   # the waiter that starts SITL, not SITL itself
        if session:
            try:
                env = (d / 'environ').read_bytes().split(b'\0')
            except OSError:
                continue
            if f'ASTRA_SIM_SESSION={session}'.encode() not in env:
                continue
        try:
            exe = os.path.realpath(str(d / 'exe'))
        except OSError:
            exe = ''
        # An unreadable /proc/<pid>/exe link "resolves" to itself: then argv[0] is the better answer.
        if not exe or exe == os.path.realpath(str(d)) + os.sep + 'exe' or not os.path.isfile(exe):
            exe = ''
        return {'pid': int(d.name), 'argv': argv, 'exe': exe}
    return None


def read_state(path: str) -> Dict:
    """state/current_launch.yaml of the running simulator ({} when missing or unreadable)."""
    try:
        import yaml
        data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        return data if isinstance(data, dict) else {}
    except Exception:  # noqa: BLE001 - a missing / broken state file only means fewer details
        return {}


def query_autopilot_version(url: str, timeout: float = 8.0) -> Optional[Dict]:
    """Ask the autopilot for AUTOPILOT_VERSION over MAVLink (pymavlink). None when nothing answered.

    Only use a MAVLink address nobody else is listening on at that moment (demo.sh's ground-station port is
    free between mission_tool calls).
    """
    try:
        from pymavlink import mavutil
    except ImportError:
        return None
    conn = None
    try:
        conn = mavutil.mavlink_connection(url, source_system=254, source_component=191)
        hb = conn.wait_heartbeat(timeout=timeout)
        if hb is None:
            return None
        deadline = time.monotonic() + timeout
        last_ask = 0.0
        while time.monotonic() < deadline:
            if time.monotonic() - last_ask > 2.0:
                conn.mav.command_long_send(conn.target_system or 1, conn.target_component or 1,
                                           512, 0, 148, 0, 0, 0, 0, 0, 0)    # MAV_CMD_REQUEST_MESSAGE AUTOPILOT_VERSION
                last_ask = time.monotonic()
            msg = conn.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=0.5)
            if msg is not None:
                out = decode_sw_version(msg.flight_sw_version)
                out['git'] = decode_git(msg.flight_custom_version)
                out['board_version'] = int(getattr(msg, 'board_version', 0))
                out['source'] = f'AUTOPILOT_VERSION via {url}'
                return out
        return None
    except Exception:  # noqa: BLE001 - a recorder never breaks the run
        return None
    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:  # noqa: BLE001 - closing a dead link is not worth a message
                pass


def autopilot_meta(sim_log: str = '', state_file: str = '', session: str = '', wait_s: float = 0.0,
                   mavlink: str = '', mavlink_timeout: float = 8.0, proc: str = '/proc') -> Dict:
    """Build the autopilot.json record (see the module docstring). Never raises."""
    notes: List[str] = []
    state = read_state(state_file) if state_file else {}
    data: Dict = {'schema': f'{SCHEMA_PREFIX}.autopilot.v1', 'written': now_text(),
                  'kind': state.get('autopilot') or 'sitl', 'reported': None, 'binary': None, 'process': None,
                  'run_dir': state.get('sitl_run_dir', ''), 'profile': state.get('profile', ''), 'notes': notes}
    # 1. what the autopilot itself said (its banner in sim.log); wait a little for it after boot
    deadline = time.monotonic() + max(0.0, wait_s)
    banner = None
    while True:
        try:
            text = Path(sim_log).read_text(encoding='utf-8', errors='replace') if sim_log else ''
        except OSError:
            text = ''
        banner = parse_banner(text)
        if banner or time.monotonic() >= deadline:
            break
        time.sleep(1.0)
    if banner:
        banner['source'] = f'boot banner in {Path(sim_log).name} (MAVProxy "AP:" line)'
        data['reported'] = banner
    elif sim_log:
        notes.append(f'no "ArduCopter V..." banner found in {sim_log}')
    if mavlink:
        av = query_autopilot_version(mavlink, mavlink_timeout)
        if av:
            data['autopilot_version'] = av
            if data['reported'] is None:
                data['reported'] = av
        else:
            notes.append(f'no AUTOPILOT_VERSION answer on {mavlink} within {mavlink_timeout:g} s')
    # 2. the program that really runs (from the process table), else what the launch wrote down
    p = find_sitl_process(session, proc=proc) if data['kind'] != 'external' else None
    binary = ''
    if p:
        info = parse_sitl_cmdline(p['argv'])
        binary = p['exe'] or info['binary']
        data['process'] = {'pid': p['pid'], 'cmdline': ' '.join(p['argv']), 'instance': info['instance'],
                           'home': info['home'], 'wipe': info['wipe'],
                           'defaults': [file_fingerprint(f) for f in info['defaults']]}
    elif data['kind'] != 'external':
        binary = str(state.get('sitl_bin') or '')
        notes.append('the SITL process was not found' + (f' for session {session}' if session else '')
                     + (f'; using sitl_bin from {state_file}' if binary else ''))
    if binary:
        fp = file_fingerprint(binary)
        if not fp.get('missing'):
            from . import autopilot as ap_mod
            fp['compiled_version'] = ap_mod.read_binary_version(binary)
        data['binary'] = fp
    rep, comp = data.get('reported') or {}, (data.get('binary') or {}).get('compiled_version', '')
    if rep.get('version') and comp and not comp.startswith(rep['version']):
        notes.append(f'the autopilot reported {rep["version"]} but the program says {comp}')
    return data


# ---------------------------------------------------------------------------------------------
# Detector and team stack
# ---------------------------------------------------------------------------------------------

def detector_meta(kind: str, model: str = '', text: str = '', load: bool = False, device: str = 'auto',
                  camera: str = 'zr10') -> Dict:
    """Build the detector.json record. kind real = the team's new_od with `model` (checked by model_check)."""
    data: Dict = {'schema': f'{SCHEMA_PREFIX}.detector.v1', 'written': now_text(), 'kind': kind, 'text': text,
                  'model': None, 'check': None}
    if kind in ('real', 'team'):
        rep = model_check.check(model, load=load, device=device, camera=camera)
        data['model'] = {'name': rep.name, 'path': rep.path, 'size_bytes': rep.size_bytes,
                         'size_mb': round(rep.size_bytes / 1e6, 1), 'sha256': rep.sha256, 'format': rep.format}
        data['check'] = {'ok': rep.ok, 'loaded': bool(rep.load) and rep.ok and load, 'errors': rep.errors,
                         'warnings': rep.warnings, 'notes': rep.notes, 'load': rep.load}
    return data


#: How long the "uncommitted changes?" check may take. `git status` on a Windows drive seen from WSL (/mnt/d/...)
#: takes ~40 s for this repo (measured 2026-10-31), far too long for demo.sh to wait before the flight.
DIRTY_TIMEOUT_S = 15.0


def git_info(repo: str, dirty_timeout: float = DIRTY_TIMEOUT_S) -> Optional[Dict]:
    """{commit, branch, dirty} of a git checkout (None when it is not one or git is missing).

    dirty is None (with 'dirty_note') when `git status` did not answer within dirty_timeout seconds: the commit
    and branch are still recorded, because they are what makes two runs comparable."""
    if not (Path(repo) / '.git').exists():
        return None

    def git(*args: str, timeout: float = 10.0) -> str:
        return subprocess.run(['git', '-C', repo, *args], capture_output=True, text=True, timeout=timeout,
                              check=False).stdout.strip()
    try:
        out: Dict = {'commit': git('rev-parse', 'HEAD'), 'branch': git('rev-parse', '--abbrev-ref', 'HEAD')}
    except (OSError, subprocess.SubprocessError):
        return None
    if not out['commit']:
        return None                                    # not a usable checkout (or git refused it)
    try:
        out['dirty'] = bool(git('status', '--porcelain', '--untracked-files=no', timeout=dirty_timeout))
    except (OSError, subprocess.SubprocessError):
        out['dirty'] = None
        out['dirty_note'] = (f'not checked: git status took longer than {dirty_timeout:g} s (a Windows drive seen '
                             'from WSL is slow)')
    return out


def team_stack_meta(repo: str, tracker_cmd: str = '', fcu_url: str = '', source_repo: str = '') -> Dict:
    """Build the team_stack.json record (which checkout of the Jetson code ran, and how).

    source_repo: the git checkout this folder was COPIED from ($ASTRA_SOURCE_REPO, e.g. the Windows checkout that
    simulation/setup/03_build.sh mirrors into the Linux copy, which has no .git). Used only when repo itself is
    not a git checkout; the record then says so ('git_from')."""
    launcher = Path(repo) / 'launcher.sh'
    git = git_info(repo)
    data: Dict = {'schema': f'{SCHEMA_PREFIX}.team_stack.v1', 'written': now_text(), 'repo': str(repo),
                  'launcher': file_fingerprint(str(launcher)), 'git': git, 'tracker_cmd': tracker_cmd,
                  'fcu_url': fcu_url, 'notes': []}
    if git is None and source_repo:
        git = git_info(source_repo)
        if git is not None:
            data['git'], data['git_from'] = git, str(source_repo)
            data['notes'].append(f'{repo} has no .git: the commit is the one of {source_repo} ($ASTRA_SOURCE_REPO, the '
                                 'checkout it was copied from; uncommitted edits there count as "dirty")')
    if data['git'] is None:
        data['notes'].append('not a git checkout (the Linux copy made by simulation/setup/03_build.sh has no .git): '
                             'see the Windows checkout for the commit, or set ASTRA_SOURCE_REPO to it')
    return data


# ---------------------------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    """The argument parser (one sub-command per record)."""
    parser = argparse.ArgumentParser(prog='run_meta', description=__doc__.split('\n\n')[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='what', required=True)
    p = sub.add_parser('autopilot', help='which ArduCopter really ran')
    p.add_argument('--out', required=True)
    p.add_argument('--sim-log', default='', help='the run\'s sim.log (MAVProxy logs the autopilot banner there)')
    p.add_argument('--state', default='', help='~/.astra_sim/state/current_launch.yaml of the running simulator')
    p.add_argument('--session', default='', help='ASTRA_SIM_SESSION of the run (finds its SITL process)')
    p.add_argument('--wait', type=float, default=0.0, help='seconds to wait for the banner in sim.log')
    p.add_argument('--mavlink', default='', help='also ask AUTOPILOT_VERSION on this MAVLink address')
    p.add_argument('--mavlink-timeout', type=float, default=8.0)
    p = sub.add_parser('detector', help='which detector / model really ran')
    p.add_argument('--out', required=True)
    p.add_argument('--kind', required=True, choices=('real', 'team', 'truth', 'custom'))
    p.add_argument('--model', default='', help='kind real: the weights file new_od loads')
    p.add_argument('--text', default='', help='a short description (demo.sh: the plan\'s detector line)')
    p.add_argument('--load', action='store_true', help='kind real: load the model once (model_check --load)')
    p.add_argument('--device', default='auto')
    p.add_argument('--camera', default='zr10', choices=sorted(model_check.PHOTO_SIZES))
    p = sub.add_parser('team-stack', help='which Jetson code really ran')
    p.add_argument('--out', required=True)
    p.add_argument('--repo', required=True)
    p.add_argument('--tracker-cmd', default='')
    p.add_argument('--fcu-url', default='')
    p.add_argument('--source-repo', default='', help='the git checkout REPO was copied from (default '
                                                     '$ASTRA_SOURCE_REPO); used when REPO has no .git')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Command-line entry point (see the module docstring)."""
    args = build_parser().parse_args(list(sys.argv[1:] if argv is None else argv))
    code = 0
    if args.what == 'autopilot':
        data = autopilot_meta(args.sim_log, args.state, args.session, args.wait, args.mavlink, args.mavlink_timeout)
        rep = data.get('reported') or {}
        binary = data.get('binary') or {}
        print(f' OK autopilot: {rep.get("banner") or rep.get("version") or "version not reported"}'
              + (f'; program {binary.get("path")} ({binary.get("compiled_version") or "?"})' if binary else ''))
        for n in data['notes']:
            print(f' .. {n}')
    elif args.what == 'detector':
        data = detector_meta(args.kind, args.model, args.text, args.load, args.device, args.camera)
        chk = data.get('check')
        if chk is not None:
            rep = model_check.ModelReport(path=data['model']['path'], name=data['model']['name'],
                                          format=data['model']['format'], size_bytes=data['model']['size_bytes'],
                                          sha256=data['model']['sha256'], errors=chk['errors'],
                                          warnings=chk['warnings'], notes=chk['notes'])
            print(rep.render(), flush=True)
            if not chk['ok']:
                print(f'[run_meta] ERROR: the detector model {args.model or "(none)"} cannot be used (reasons above, '
                      'each with its fix).', file=sys.stderr)
                code = model_check.EXIT_UNUSABLE
        else:
            print(f' OK detector: {args.kind}{": " + args.text if args.text else ""}')
    else:
        data = team_stack_meta(args.repo, args.tracker_cmd, args.fcu_url,
                               args.source_repo or os.environ.get('ASTRA_SOURCE_REPO', ''))
        git = data.get('git') or {}
        print(f' OK team code: {args.repo}' + (f' (git {git.get("commit", "")[:10]}'
                                               f'{", uncommitted changes" if git.get("dirty") else ""})' if git else ''))
    try:
        write_json(args.out, data)
    except OSError as exc:
        print(f'[run_meta] ERROR: could not write {args.out}: {exc}. Fix: check that the run folder exists and is '
              'writable.', file=sys.stderr)
        return 1
    return code


if __name__ == '__main__':
    sys.exit(main())
