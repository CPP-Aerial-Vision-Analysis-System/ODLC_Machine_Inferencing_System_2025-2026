"""
sim_demo/run_summary.py -- PURPOSE: write the "what happened in this run" page of a run folder, so a
teammate can debug a run without opening ten log files, and keep one line per run in a table of all runs.

At the end of every demo.sh run (and of every START OVER run; and, if the run crashed or its window was
closed, a PARTIAL one written by the flight recorder) it writes into ~/.astra_sim/runs/<stamp>_<world>/:
    run_summary.md     for people: what ran (git commit and whether it had uncommitted changes, profile,
                       world, ArduPilot version, the team code and where it ran, the detector model with
                       its SHA-256), the result and scores, how long each mission phase took, EVERY warning
                       and error line found in the logs (grouped, with file and line number; lines written
                       while the simulator shut down after the run are listed apart), the Jetson's power mode
                       and load when the team code ran on a Jetson, what still differs from the field, and
                       where every log and CSV file is (also as a Windows path)
    run_summary.json   the same for scripts
and adds (or replaces) the run's row in ~/.astra_sim/runs/runs_index.csv (one row per run: world,
profile, detector + model, ArduPilot version, result, scores, duration).

It also does the work behind simulation/scripts/logs.sh:
    python3 -m sim_demo.run_summary finish RUN_DIR [--result TEXT] [--exit-code N] [--partial] [--note TEXT]
                                                   [--refresh] [--logs-until EPOCH]
    python3 -m sim_demo.run_summary show   RUN_DIR             print run_summary.md (writes it first if missing;
                                                               a run that is still going is shown, never written)
    python3 -m sim_demo.run_summary running RUN_DIR            exit 0 + why when the run is still going, else 1
    python3 -m sim_demo.run_summary errors RUN_DIR [--all]     every warning / error line of every log
    python3 -m sim_demo.run_summary list   [--runs-dir DIR]    one line per run, newest first
    python3 -m sim_demo.run_summary bundle RUN_DIR [--out FILE.zip] [--photos] [--max-mb 20]
    python3 -m sim_demo.run_summary winpath PATH               the Windows path of a WSL path
--refresh writes the summary again with the result, exit code, note and git commit it already has (after
the scorer rewrote its files at shutdown, or START OVER moved the photos in). --logs-until EPOCH: log lines
stamped after that moment (the simulator shutting down after the run) are listed apart, not counted.
Exit codes: 0 = done, 1 = the run folder is missing or a file could not be written (reason printed),
2 = bad usage. Standard library only (no ROS), so it also runs on a Windows PC.
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import os
import posixpath
import re
import subprocess
import sys
import time
import zipfile

if __package__ in (None, ''):          # run as a plain file: python3 .../sim_demo/run_summary.py
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = 'sim_demo'           # noqa: A001 - lets the relative imports below work

from .run_csv import (CSV_DIR_NAME, INDEX_FILE_NAME, PHASE_ORDER, RUNS_INDEX_COLUMNS,  # noqa: E402
                      count_rows, csv_dir, num, read_csv, to_float, utc_iso, write_csv_atomic)

PROG = 'run_summary'
EXIT_OK, EXIT_FAILED, EXIT_USAGE = 0, 1, 2
SUMMARY_MD, SUMMARY_JSON = 'run_summary.md', 'run_summary.json'
MAX_HITS_PER_FILE = 400            # distinct warning/error groups kept PER LOG FILE in run_summary.json
MAX_GROUPS_SHOWN = 25              # distinct problems shown per log file in run_summary.md
RUNNING_QUIET_S = 60.0             # a run without a recorder counts as still going while a log changed this recently

# What counts as an error / a warning in a log line (case matters: "drop error" in a report is not one).
RE_ERROR = re.compile(r'\[ERROR\]|\bERROR\b|\bFATAL\b|\bCRITICAL\b|Traceback \(most recent call last\)|'
                      r'process has died|\bException\b|\b[A-Z][A-Za-z]*Error:| XX ')
RE_WARN = re.compile(r'\[WARN\]|\bWARNING\b|\bWarning\b|\bWARN\b| !! ')
# Lines the docs call NORMAL (AGENTS.md 4.4 / 5.5): listed apart so they do not look like faults.
KNOWN_NORMAL = [
    ('PreArm:', 'normal while the simulated autopilot settles (the first 30-60 s)'),
    ('Gyros inconsistent', 'normal while the simulated autopilot settles (the first 30-60 s)'),
    ('Could not confirm 5.0x', 'expected with --zoom survey (the camera emulator keeps 1x on purpose)'),
    ('Zoom attempt', 'expected with --zoom survey (the camera emulator keeps 1x on purpose)'),
]
# Time of a log line: demo.sh's stamp at the start of every line ("2026-10-31 11:26:39 ..."), else a ROS stamp.
RE_TIME = re.compile(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})|\[(\d{10}\.\d+)\]|\[(\d{2}:\d{2}:\d{2})\]')
RE_NOISE = re.compile(r'\[\d{10}\.\d+\]|\[\d{2}:\d{2}:\d{2}(?:\.\d+)?\]|^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2} |'
                      r'0x[0-9a-fA-F]+|\d+')
# File names never put into a bundle (they could hold a password or a key).
SECRET_PATTERNS = ('*token*', '*secret*', '*password*', '*passwd*', '*credential*', '*.key', '*.pem', 'id_rsa*',
                   'id_dsa*', 'id_ecdsa*', 'id_ed25519*', '*.ppk', '*.p12', '*.pfx', '*.kdbx', '*.gpg',
                   'known_hosts*', 'authorized_keys*', '*.netrc', '.env', 'target.env')
# ... and file CONTENTS that look like a key or a password (checked in every text file up to 5 MB).
RE_SECRET_TEXT = re.compile(
    r'-----BEGIN [A-Z ]*PRIVATE KEY-----|PuTTY-User-Key-File|\bghp_[A-Za-z0-9]{30,}|\bgithub_pat_[A-Za-z0-9_]{30,}|'
    r'\bAKIA[0-9A-Z]{16}\b|\bxox[abprs]-[A-Za-z0-9-]{10,}|\bsk-[A-Za-z0-9]{20,}|'
    r'(?i:\b(?:password|passwd|api[_-]?key|secret[_-]?key|access[_-]?token)\b\s*[=:]\s*["\']?[^\s"\']{6,})')
WHAT_IS = {   # what each well-known file of a run folder is (for the "Files" table)
    'demo.env': 'what demo.sh started (world, mission, session)',
    'start_over.env': 'what START OVER started (world, mission, the run it continued)',
    'demo.log': 'everything the demo window showed (also the checks before the start), with a time stamp per line',
    'sim.log': 'the simulator: Gazebo, SITL, MAVProxy, camera emulator, scorer (ros2 launch output, a time stamp '
               'per line)',
    'team_stack.log': "the team's flight code: MAVROS + the 6 nodes (./launcher.sh sim, a time stamp per line)",
    'flight_recorder.log': 'the MAVLink flight recorder (writes csv/ live)',
    'flight_recorder.json': 'what the flight recorder heard (autopilot id, version, message counts)',
    'flight_recorder.pid': 'the flight recorder is still running (its process id)',
    'flight_recorder_tlog.json': 'what a .tlog replay (export_csv --tlog) heard',
    'sitl.log': 'START OVER: ArduCopter SITL', 'mavproxy.log': 'START OVER: MAVProxy',
    'helpers.log': 'START OVER: score_run, payload release, markers', 'phases.log': 'START OVER: the run watcher',
    'guard.log': 'START OVER: the clean-up guard',
    'phases.jsonl': 'phase_monitor: every mission phase, detection and photo line',
    'events.jsonl': 'score_run: every status text, arm/disarm, waypoint, photo, detection',
    'run_data.json': 'every input of the score (re-score: ros2 run sim_demo score_run --rescore DIR)',
    'score_report.md': 'the score report (targets found, aim and drop errors, detection quality)',
    'score_report.json': 'the score report for scripts',
    'mission_map.png': 'map: search area, photo footprints, track, detections, drops',
    'write_score.log': 'the request asking score_run to write its report',
    'run_summary.md': 'this summary', 'run_summary.json': 'this summary for scripts',
    'run_summary.log': 'what the summary step printed (look here when run_summary.md is missing)',
    'codemap_trace.jsonl': 'the live code map trace (only with ASTRA_TRACE=1)',
}


def err(what: str, why: str = '', fix: str = '') -> None:
    """An error in the shared shape: WHAT failed. Likely cause: WHY. Fix: HOW."""
    parts = [f'[{PROG}] ERROR: {what}']
    if why:
        parts.append(f'Likely cause: {why}')
    if fix:
        parts.append(f'Fix: {fix}')
    print(' '.join(parts), file=sys.stderr, flush=True)


def load_json(path: str):
    """A JSON file's content (None when missing or broken)."""
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def read_env(path: str) -> dict:
    """KEY=VALUE lines of a file like demo.env ({} when missing)."""
    out = {}
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            for line in f:
                k, sep, v = line.rstrip('\n').partition('=')
                if sep and k.strip():
                    out[k.strip()] = v
    except OSError:
        pass
    return out


def run_env(run_dir: str, depth: int = 5) -> dict:
    """How a run was started: its demo.env / start_over.env (START OVER's own file wins), completed for the
    keys it does not have from the run START OVER continued (PREVIOUS_RUN=), up to `depth` runs back."""
    env: dict = {}
    for name in ('demo.env', 'start_over.env'):
        env.update(read_env(os.path.join(run_dir, name)))
    prev = env.get('PREVIOUS_RUN', '')
    if depth > 0 and prev and os.path.isdir(prev) and os.path.abspath(prev) != os.path.abspath(run_dir):
        for k, v in run_env(prev, depth - 1).items():
            env.setdefault(k, v)
    return env


def windows_path(path: str, distro: str = '') -> str:
    """The path as Windows sees it: /mnt/d/x -> D:\\x; a Linux path -> \\\\wsl.localhost\\<distro>\\...

    Returns '' when it cannot be known (not inside WSL and not a /mnt/<drive> path).
    """
    p = path if path.startswith('/') else os.path.abspath(path)
    if not p.startswith('/'):
        return ''                           # already a Windows path (this runs on Windows)
    p = posixpath.normpath(p)
    m = re.match(r'^/mnt/([a-zA-Z])(/.*)?$', p)
    if m:
        return m.group(1).upper() + ':' + (m.group(2) or '/').replace('/', '\\')
    distro = distro or os.environ.get('WSL_DISTRO_NAME', '')
    if not distro or not p.startswith('/'):
        return ''
    return '\\\\wsl.localhost\\' + distro + p.replace('/', '\\')


def human_size(n: int) -> str:
    """1536 -> '1.5 KB'."""
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024 or unit == 'GB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024.0
    return f'{n:.1f} GB'


def local_time(t: float | None) -> str:
    """Seconds since 1970 -> '2026-10-31 10:14:31' in this computer's time zone ('' for None)."""
    if t is None:
        return ''
    return time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t))


def mmss(seconds: float | None) -> str:
    """75.4 -> '1:15'."""
    if seconds is None:
        return '-'
    s = int(round(abs(seconds)))
    return ('-' if seconds < 0 and s else '') + f'{s // 60}:{s % 60:02d}'


# ------------------------------------------------------------------------------------------ git
SOURCE_JSON = os.path.join('run_meta', 'source.json')   # which commit ran, looked up when the run STARTED


def git_info(repo: str, status_timeout_s: float = 20.0) -> dict | None:
    """{'repo', 'commit', 'branch', 'dirty', 'changed_files'} of a git checkout, or None (no .git / no git).
    `git status` on the Windows drive takes seconds (more when the PC is busy): after status_timeout_s the
    dirty flag is None ("not known") instead of making the caller wait."""
    if not repo or not os.path.exists(os.path.join(repo, '.git')):
        return None

    def git(*args, timeout=20.0):
        return subprocess.run(['git', '-C', repo, *args], capture_output=True, text=True, timeout=timeout,
                              check=False).stdout.strip()
    try:
        commit = git('rev-parse', 'HEAD')
        if not commit:
            return None
        out = {'repo': repo, 'commit': commit, 'branch': git('rev-parse', '--abbrev-ref', 'HEAD')}
    except (OSError, subprocess.SubprocessError):
        return None
    try:
        # dirty = the code that ran is not exactly the commit: changed files OR new files git does not
        # ignore (a new source file is built and runs too); ignored files (builds, logs) do not count.
        changes = [ln for ln in git('status', '--porcelain', timeout=status_timeout_s).splitlines() if ln.strip()]
        out.update(dirty=bool(changes), changed_files=len(changes))
    except (OSError, subprocess.SubprocessError):
        out.update(dirty=None, changed_files=None, note=f'git status took longer than {status_timeout_s:g} s')
    return out


def _windows_repo() -> str:
    """$ASTRA_WINDOWS_REPO (set by the Windows .bat launchers, e.g. D:\\x\\repo) as a WSL path (/mnt/d/x/repo)."""
    w = os.environ.get('ASTRA_WINDOWS_REPO', '').strip().rstrip('\\/')
    m = re.match(r'^([A-Za-z]):[\\/]?(.*)$', w)
    return f'/mnt/{m.group(1).lower()}/' + m.group(2).replace('\\', '/') if m else ''


def write_source(run_dir: str, git_repo: str = '') -> dict:
    """Look up which commit runs (find_git) and save it as <run>/run_meta/source.json. demo.sh calls this in the
    background when the run STARTS: that is the commit that was built, and the end of the run never waits for git."""
    gi = dict(find_git(run_dir, git_repo), looked_up_utc=utc_iso(time.time()))
    path = os.path.join(run_dir, SOURCE_JSON)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _write_text_atomic(path, json.dumps(gi, indent=1) + '\n')
    return gi


def _recorded_source(run_dir: str) -> dict:
    """The commit the run wrote down when it started (run_meta/source.json), or {}."""
    gi = load_json(os.path.join(run_dir, SOURCE_JSON)) or {}
    if gi.get('commit'):
        gi = dict(gi, how=gi.get('how', '') + ' (looked up when the run started: run_meta/source.json)')
        return gi
    return {}


def find_git(run_dir: str, git_repo: str = '') -> dict:
    """Which commit of this repo ran. Tries, in order: --git-repo, $ASTRA_SOURCE_REPO, $ASTRA_WINDOWS_REPO, the
    build copy's .astra_source file (written by setup/03_build.sh when it copies the repo), $ASTRA_REPO itself."""
    tried = []
    for cand in (git_repo, os.environ.get('ASTRA_SOURCE_REPO', ''), _windows_repo()):
        if cand:
            tried.append(cand)
            gi = git_info(cand)
            if gi:
                gi['how'] = 'git in ' + cand
                return gi
    repo = os.environ.get('ASTRA_REPO', '')
    src = read_env(os.path.join(repo, '.astra_source')) if repo else {}
    if src.get('COMMIT'):
        return {'repo': src.get('SOURCE', ''), 'commit': src['COMMIT'], 'branch': src.get('BRANCH', ''),
                'dirty': src.get('DIRTY', '') in ('1', 'true', 'yes'), 'how': f'{repo}/.astra_source (build copy)'}
    if repo:
        tried.append(repo)
        gi = git_info(repo)
        if gi:
            gi['how'] = 'git in ' + repo
            return gi
    return {'repo': '', 'commit': '', 'branch': '', 'dirty': None,
            'how': 'unknown: no git checkout found (tried ' + (', '.join(tried) or 'nothing') + '). The Linux build '
                   'copy has no .git; set ASTRA_SOURCE_REPO to the Windows checkout (e.g. /mnt/d/.../'
                   'ODLC_Machine_Inferencing_System_2025-2026) before demo.sh'}


def _old_git(run_dir: str) -> dict:
    """The commit of a run summarised AFTER the fact: only what the run itself wrote down."""
    rec = _recorded_source(run_dir)
    if rec:
        return rec
    old = (load_json(os.path.join(run_dir, SUMMARY_JSON)) or {}).get('git') or {}
    if old.get('commit'):
        return old
    ts = (load_json(os.path.join(run_dir, 'run_meta', 'team_stack.json')) or {}).get('git') or {}
    if ts.get('commit'):
        return dict(ts, repo=ts.get('repo', ''), how='run_meta/team_stack.json')
    return {'repo': '', 'commit': '', 'branch': '', 'dirty': None,
            'how': 'not recorded: this summary was written after the run, and the commit of today may differ'}


# ------------------------------------------------------------------------------------------ logs
def log_files(run_dir: str) -> list[str]:
    """Every .log file of the run (top level and one folder down), relative paths, sorted."""
    out = []
    for root, dirs, files in os.walk(run_dir):
        rel = os.path.relpath(root, run_dir)
        depth = 0 if rel == '.' else rel.count(os.sep) + 1
        dirs[:] = [d for d in dirs if depth < 1 and d not in ('photos', 'detections', CSV_DIR_NAME)]
        for f in files:
            if f.endswith('.log'):
                out.append(os.path.normpath(os.path.join(rel, f)) if rel != '.' else f)
    return sorted(out)


def _line_time(line: str) -> tuple[str, float | None]:
    """(HH:MM:SS for people, seconds since 1970 or None) of a log line; ('', None) when it has no time."""
    m = RE_TIME.search(line)
    if not m:
        return '', None
    if m.group(1):                          # demo.sh's stamp (local time, whole seconds)
        try:
            return m.group(1)[11:], time.mktime(time.strptime(m.group(1), '%Y-%m-%d %H:%M:%S'))
        except (ValueError, OverflowError):
            return m.group(1)[11:], None
    if m.group(2):                          # a ROS stamp in seconds since 1970
        t = float(m.group(2))
        return time.strftime('%H:%M:%S', time.localtime(t)), t
    return m.group(3), None                 # a ROS stamp with only the time of day


def scan_logs(run_dir: str, files: list[str] | None = None, until: float | None = None) -> list[dict]:
    """Every warning / error line of the run's logs: [{file, line, level, time, text, normal, after_end}].

    level is 'error' or 'warning'; normal is '' or why the docs call this line normal; after_end is True for
    a line stamped after `until` (the simulator shutting down after the run ended).
    """
    hits = []
    for rel in files if files is not None else log_files(run_dir):
        kept = 0
        try:
            with open(os.path.join(run_dir, rel), encoding='utf-8', errors='replace') as f:
                for no, line in enumerate(f, 1):
                    line = line.rstrip('\n')
                    level = 'error' if RE_ERROR.search(line) else 'warning' if RE_WARN.search(line) else ''
                    if not level:
                        continue
                    normal = next((why for pat, why in KNOWN_NORMAL if pat in line), '')
                    hhmmss, t = _line_time(line)
                    hits.append({'file': rel, 'line': no, 'level': level, 'time': hhmmss,
                                 'text': line.strip()[:400], 'normal': normal,
                                 'after_end': bool(until is not None and t is not None and t > until + 1.0)})
                    kept += 1
                    if kept >= MAX_HITS_PER_FILE * 10:
                        break               # a runaway log: enough to see the pattern
        except OSError:
            continue
    return hits


def group_hits(hits: list[dict], per_file: int = MAX_HITS_PER_FILE) -> list[dict]:
    """Group identical problems (same file, level and text apart from numbers and times); at most `per_file`
    groups per log file, so one noisy log never pushes another log's problems out of the summary."""
    groups: dict[tuple, dict] = {}
    for h in hits:
        key = (h['file'], h['level'], bool(h.get('after_end')), RE_NOISE.sub('#', h['text']))
        g = groups.get(key)
        if g is None:
            groups[key] = dict(h, count=1, last_line=h['line'])
        else:
            g['count'] += 1
            g['last_line'] = h['line']
    out, per = [], {}
    for g in sorted(groups.values(), key=lambda g: (g['file'], g['line'])):
        per[g['file']] = per.get(g['file'], 0) + 1
        if per[g['file']] <= per_file:
            out.append(g)
    return out


# ------------------------------------------------------------------------------------------ phases
def phase_times(run_dir: str) -> list[dict]:
    """[{code, detail, start, t_since_arm_s, duration_s, source}] from csv/phases.csv, else phases.jsonl."""
    rows = read_csv(os.path.join(csv_dir(run_dir), 'phases.csv'))
    out = []
    for r in rows:
        t = to_float(r.get('unix_time_s'))
        if t is None:
            continue
        out.append({'code': r.get('phase_code', ''), 'detail': r.get('detail', ''), 'start': t,
                    't_since_arm_s': to_float(r.get('t_since_arm_s')), 'source': 'csv/phases.csv'})
    if not out:
        from .csv_export import load_jsonl
        for ev in load_jsonl(os.path.join(run_dir, 'phases.jsonl')):
            if ev.get('kind') == 'phase' and to_float(ev.get('time')) is not None:
                out.append({'code': ev.get('phase', ''), 'detail': ev.get('text', ''),
                            'start': float(ev['time']), 't_since_arm_s': None, 'source': 'phases.jsonl'})
        arm = next((p['start'] for p in out if p['code'] == 'ARMED'), None)
        for p in out:
            p['t_since_arm_s'] = None if arm is None else round(p['start'] - arm, 1)
    out.sort(key=lambda p: p['start'])
    for i, p in enumerate(out):
        nxt = out[i + 1]['start'] if i + 1 < len(out) else None
        p['duration_s'] = None if nxt is None else round(nxt - p['start'], 1)
    return out


# ------------------------------------------------------------------------------------------ summary
def _meta(run_dir: str) -> dict:
    meta = {}
    folder = os.path.join(run_dir, 'run_meta')
    for name in ('autopilot', 'detector', 'team_stack'):
        data = load_json(os.path.join(folder, name + '.json'))
        if data is not None:
            meta[name] = data
    return meta


def _ardupilot(meta: dict, recorder: dict, run_dir: str) -> dict:
    rep = (meta.get('autopilot') or {}).get('reported') or {}
    if rep.get('version'):
        return {'version': rep['version'], 'git': rep.get('git', ''), 'source': 'run_meta/autopilot.json'}
    if recorder.get('ardupilot_version'):
        return {'version': recorder['ardupilot_version'], 'git': '', 'source': 'flight_recorder.json (boot banner)'}
    for r in read_csv(os.path.join(csv_dir(run_dir), 'events.csv')):
        m = re.search(r'Ardu\w+ V(\d+\.\d+\.\d+\S*)(?: \(([0-9a-f]+)\))?', r.get('autopilot_text', ''))
        if m:
            return {'version': m.group(1), 'git': m.group(2) or '', 'source': 'csv/events.csv'}
    from .csv_export import load_jsonl
    for ev in load_jsonl(os.path.join(run_dir, 'events.jsonl')):
        m = re.search(r'Ardu\w+ V(\d+\.\d+\.\d+\S*)(?: \(([0-9a-f]+)\))?', str(ev.get('text', '')))
        if m:
            return {'version': m.group(1), 'git': m.group(2) or '', 'source': 'events.jsonl'}
    return {'version': '', 'git': '', 'source': 'not found in this run'}


def detection_backend(det: dict) -> str:
    """Which detection backend ran (owner: record it in every run summary): an explicit 'backend' in
    run_meta/detector.json first, else worked out from the model format and the load test's device. '' = unknown."""
    load = ((det.get('check') or {}).get('load') or {})
    explicit = det.get('backend') or load.get('backend') or ''
    if explicit:
        return str(explicit)
    if det.get('kind') == 'truth':
        return 'answer-key stand-in (no model)'
    fmt = str((det.get('model') or {}).get('format') or '').lower()
    device = str(load.get('device') or '')
    name = {'pytorch': 'PyTorch', 'engine': 'TensorRT', 'tensorrt': 'TensorRT', 'onnx': 'ONNX'}.get(fmt, fmt)
    if not name:
        return ''
    where = (f' on CUDA ({load["gpu"]})' if load.get('gpu') else ' on CUDA') if device.startswith('cuda') else \
        ' on the CPU' if device == 'cpu' else ''
    return name + where


def _detector(meta: dict, env: dict, score: dict) -> dict:
    det = meta.get('detector') or {}
    model = det.get('model') or {}
    return {'kind': det.get('kind') or env.get('DETECTOR') or score.get('detector', ''),
            'text': det.get('text', ''), 'model': model.get('name') or os.path.basename(env.get('MODEL', '')),
            'model_path': model.get('path') or env.get('MODEL', ''), 'sha256': model.get('sha256') or '',
            'backend': detection_backend(det),
            'source': 'run_meta/detector.json' if det else 'demo.env' if env.get('DETECTOR') else
            'score_report.json' if score else ''}


def _team_stack(meta: dict, env: dict) -> dict:
    ts = meta.get('team_stack') or {}
    where = env.get('TEAM_STACK') or env.get('TEAM_STACK_WHERE') or 'local'
    if env.get('JETSON_HOST') or env.get('JETSON_LINK'):   # demo.sh --jetson usb|ssh|virtual (USB cable, or SSH)
        host, link = env.get('JETSON_HOST', ''), env.get('JETSON_LINK', '')
        where = f'jetson ({host or "?"}' + (f', {link}' if link and link != host else '') + ')'
    return {'where': where, 'repo': ts.get('repo', ''), 'git': ts.get('git'),
            'launcher_sha256': (ts.get('launcher') or {}).get('sha256', ''),
            'tracker_cmd': ts.get('tracker_cmd', ''), 'fcu_url': ts.get('fcu_url', ''),
            'inherited_from': ts.get('inherited_from', '')}


# ------------------------------------------------------------------------------------------ Jetson + field
RE_TEGRA_RAM = re.compile(r'\bRAM (\d+)/(\d+)MB')
RE_TEGRA_CPU = re.compile(r'\bCPU \[([^\]]*)\]')
RE_TEGRA_GPU = re.compile(r'\bGR3D_FREQ (\d+)%')
RE_TEGRA_TEMP = re.compile(r'\b([A-Za-z][A-Za-z0-9_]*)@(-?\d+(?:\.\d+)?)C\b')
RE_TEGRA_POWER = re.compile(r'\bVDD_IN (\d+)mW')


def tegrastats_summary(path: str) -> dict:
    """A few numbers from a Jetson's tegrastats.log (one line per sample): GPU and CPU load (average, max),
    the most memory used, the hottest temperature per sensor, the board's input power. {} when no samples."""
    gpu, cpu, ram, power, temps, total, n = [], [], [], [], {}, None, 0
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            for line in f:
                m_gpu, m_cpu, m_ram = RE_TEGRA_GPU.search(line), RE_TEGRA_CPU.search(line), RE_TEGRA_RAM.search(line)
                if not (m_gpu or m_cpu or m_ram):
                    continue
                n += 1
                if m_gpu:
                    gpu.append(int(m_gpu.group(1)))
                if m_cpu:
                    cores = [int(c) for c in re.findall(r'(\d+)%@', m_cpu.group(1))]
                    if cores:
                        cpu.append(sum(cores) / len(cores))
                if m_ram:
                    ram.append(int(m_ram.group(1)))
                    total = int(m_ram.group(2))
                m_pw = RE_TEGRA_POWER.search(line)
                if m_pw:
                    power.append(int(m_pw.group(1)))
                for name, value in RE_TEGRA_TEMP.findall(line):
                    temps[name] = max(temps.get(name, -1e9), float(value))
    except OSError:
        return {}
    if not n:
        return {}

    def avg_max(xs):
        return {'avg': round(sum(xs) / len(xs), 1), 'max': round(max(xs), 1)} if xs else None
    return {'samples': n, 'gpu_load_pct': avg_max(gpu), 'cpu_load_pct': avg_max(cpu),
            'ram_used_mb_max': max(ram) if ram else None, 'ram_total_mb': total,
            'power_in_mw': avg_max(power), 'temp_max_c': dict(sorted(temps.items()))}


def _jetson(run_dir: str, env: dict) -> dict:
    """What a Jetson run leaves in RUN_DIR/jetson/ (the Jetson tools: simulation/jetson/pc/collect.sh, hil_pc.py):
    the board's facts (status_start.json / status_end.json: power_mode, clocks, l4t, image ...; or facts.env) and
    tegrastats.log; the link (demo.env JETSON_HOST / JETSON_LINK, else jetson/link_state.env). {} for a PC run."""
    folder = os.path.join(run_dir, 'jetson')
    facts = read_env(os.path.join(folder, 'facts.env'))
    for name, suffix in (('status_start.json', ''), ('status_end.json', ' (end)')):
        data = load_json(os.path.join(folder, name))
        if not isinstance(data, dict):
            continue
        for k, v in data.items():
            if isinstance(v, (str, int, float, bool)) and str(v) != '' and (not suffix or
                                                                             re.search(r'power|clock', k, re.I)):
                facts.setdefault(k + suffix, str(v))
    link_state = read_env(os.path.join(folder, 'link_state.env'))
    tegra = tegrastats_summary(os.path.join(folder, 'tegrastats.log'))
    if not facts and not tegra and not env.get('JETSON_HOST') and not os.path.isdir(folder):
        return {}
    power = {k: v for k, v in facts.items() if re.search(r'NVP|POWER|CLOCK', k, re.IGNORECASE)}
    return {'host': env.get('JETSON_HOST') or link_state.get('HOST', ''),
            'link': env.get('JETSON_LINK') or link_state.get('LINK', ''), 'facts': facts, 'power': power,
            'tegrastats': tegra}


def _md_section_items(path: str, heading: str) -> list[str]:
    """The '- item' lines under a '## heading' of a Markdown file ([] when there is no such section)."""
    items, inside = [], False
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            for line in f:
                if line.startswith('#'):
                    inside = line.lstrip('#').strip().lower().startswith(heading.lower())
                    continue
                if inside and line.lstrip().startswith(('- ', '* ')):
                    items.append(line.strip()[2:].strip())
    except OSError:
        return []
    return items


def differences_from_field(run_dir: str, env: dict, det: dict, ts: dict) -> dict:
    """What this run does NOT do like the drone at the field (owner: list it in every run summary).
    The Jetson tools' own list wins (RUN_DIR/jetson/differences.txt one per line, or the "Differences from the
    field" section of RUN_DIR/jetson/summary.md); else a short built-in one."""
    for p in (os.path.join(run_dir, 'jetson', 'differences.txt'), os.path.join(run_dir, 'differences.txt')):
        try:
            with open(p, encoding='utf-8', errors='replace') as f:
                items = [ln.strip().lstrip('-* ').strip() for ln in f if ln.strip() and not ln.startswith('#')]
            if items:
                return {'items': items, 'source': os.path.relpath(p, run_dir)}
        except OSError:
            continue
    items = _md_section_items(os.path.join(run_dir, 'jetson', 'summary.md'), 'Differences from the field')
    if items:
        return {'items': items, 'source': os.path.join('jetson', 'summary.md') + ' (the Jetson tools)'}
    items = []
    if ts.get('where', 'local').startswith('jetson'):
        items.append(f'The team code ran on the Jetson ({ts["where"][7:-1]}); the autopilot, the world and the camera '
                     'pictures came from the simulator on the PC.')
    elif ts.get('where') == 'external':
        items.append('The team code ran on another computer or container (run.team_stack: external), not on the '
                     'drone\'s Jetson.')
    else:
        items.append('The team code ran on this PC (WSL), not on the Jetson Orin Nano: detection speed, timing and '
                     'memory differ from the drone.')
    items.append('The autopilot is ArduCopter SITL with Gazebo physics, not the Pixhawk; MAVROS reaches it over TCP '
                 'like the TELEM2 serial link.')
    if (env.get('CAMERA_KIND') or 'siyi_emulator') != 'none':
        items.append('The camera is the SIYI emulator (the real SIYI protocol, pictures drawn by Gazebo), not the '
                     'real camera.')
    if det.get('kind') == 'truth':
        items.append('The detector was the answer-key stand-in (truth_detector), not the team\'s YOLO model.')
    elif det.get('backend') and 'tensorrt' not in det['backend'].lower():
        items.append(f'Detection ran with {det["backend"]}, not TensorRT on the Jetson GPU as at the field.')
    elif not det.get('backend'):
        items.append('The detection backend was not recorded (the field runs TensorRT on the Jetson GPU).')
    items.append('GPS, wind and the payload fall are simulated.')
    return {'items': items, 'source': 'built in (no RUN_DIR/jetson/differences.txt)'}


def run_in_progress(run_dir: str, now: float | None = None) -> str:
    """'' when the run is over; otherwise why it looks like it is still going (logs.sh then writes nothing)."""
    now = time.time() if now is None else now
    try:
        from .run_records import recorder_pid
        pid = recorder_pid(run_dir)
    except Exception:  # noqa: BLE001 - no /proc (Windows): fall back to the file times below
        pid = None
    if pid:
        return f'its flight recorder is still recording (pid {pid})'
    rec_json = os.path.join(run_dir, 'flight_recorder.json')
    rec = load_json(rec_json) or {}
    try:
        if rec and not rec.get('stopped_utc') and now - os.path.getmtime(rec_json) < 30.0:
            return 'its flight recorder wrote flight_recorder.json less than 30 s ago'
    except OSError:
        pass
    old = load_json(os.path.join(run_dir, SUMMARY_JSON)) or {}
    if old and not old.get('partial'):
        return ''                           # it was finished normally (the simulator may still be open: --keep)
    for name in ('demo.log', 'sim.log', 'team_stack.log', 'phases.jsonl', 'phases.log', 'events.jsonl'):
        try:
            age = now - os.path.getmtime(os.path.join(run_dir, name))
        except OSError:
            continue
        if age < RUNNING_QUIET_S:
            return f'{name} changed {age:.0f} s ago'
    return ''


def _files(run_dir: str) -> list[dict]:
    out = []
    try:
        names = sorted(os.listdir(run_dir))
    except OSError:
        return out
    for n in names:
        p = os.path.join(run_dir, n)
        if os.path.isdir(p):
            count, size = 0, 0
            for root, _d, fs in os.walk(p):
                for f in fs:
                    count += 1
                    try:
                        size += os.path.getsize(os.path.join(root, f))
                    except OSError:
                        pass
            out.append({'name': n + '/', 'size': size, 'files': count,
                        'what': {'photos': "the team camera's photos (+ JSON sidecars)",
                                 'detections': "new_od's annotated photos", CSV_DIR_NAME: 'the CSV files (below)',
                                 'run_meta': 'which pieces ran (ArduPilot, detector, team code)',
                                 'lattice': 'the Lattice bridge (fake Lattice journal, radio tlog)'}.get(n, '')})
        else:
            try:
                size = os.path.getsize(p)
            except OSError:
                size = 0
            what = WHAT_IS.get(n, '')
            if n == 'run_summary.log' and size == 0:
                what += ' (still being written while this summary was made)'
            out.append({'name': n, 'size': size, 'files': 1, 'what': what})
    have = {f['name'] for f in out}
    for n in (SUMMARY_MD, SUMMARY_JSON):    # written right after this list was made
        if n not in have:
            out.append({'name': n, 'size': 0, 'files': 1, 'what': WHAT_IS[n] + ' (written now)'})
    out.sort(key=lambda f: f['name'])
    return out


def _result(run_dir: str, phases: list[dict], given: str, exit_code) -> str:
    if given:
        return given
    codes = {p['code'] for p in phases}
    if 'LANDED' in codes:
        return 'landed'
    if exit_code not in (None, ''):
        return f'ended with exit code {exit_code}'
    return 'unknown (the run did not reach LANDED, or it is still running)'


def build_summary(run_dir: str, result: str = '', exit_code=None, partial: bool = False, note: str = '',
                  git_repo: str = '', live: bool = False, logs_until: float | None = None, git: dict | None = None,
                  in_progress: str = '') -> dict:
    """Collect everything the summary shows (no files written)."""
    env = run_env(run_dir)
    meta = _meta(run_dir)
    recorder = load_json(os.path.join(run_dir, 'flight_recorder.json')) or {}
    score = load_json(os.path.join(run_dir, 'score_report.json')) or {}
    phases = phase_times(run_dir)
    all_hits = scan_logs(run_dir, until=logs_until)
    hits = [h for h in all_hits if not h['after_end']]          # what happened during the run itself
    late = [h for h in all_hits if h['after_end']]              # the simulator shutting down afterwards
    groups = group_hits(hits)
    det = _detector(meta, env, score)
    ts = _team_stack(meta, env)
    started = to_float(score.get('started')) or (phases[0]['start'] if phases else None)
    ended = to_float(score.get('ended')) or (phases[-1]['start'] if phases else None)
    csvs = []
    for name in ('telemetry.csv', 'events.csv', 'phases.csv', 'photos.csv', 'detections.csv',
                 'targets_and_drops.csv'):
        p = os.path.join(csv_dir(run_dir), name)
        csvs.append({'name': name, 'rows': count_rows(p), 'path': p, 'windows': windows_path(p)})
    serious: dict[tuple, dict] = {}      # (sender, text) -> first time, severity, how often
    for r in read_csv(os.path.join(csv_dir(run_dir), 'events.csv')):
        if r.get('kind') != 'statustext' or to_float(r.get('severity')) is None or float(r['severity']) > 4:
            continue
        txt = r.get('autopilot_text') or r.get('team_text') or ''
        key = (r.get('sender'), txt)
        if key in serious:
            serious[key]['count'] += 1
            continue
        serious[key] = {'time': r.get('wall_time_utc'), 't_since_arm_s': r.get('t_since_arm_s'),
                        'severity': r.get('severity_name'), 'sender': r.get('sender'), 'text': txt, 'count': 1,
                        'normal': next((why for pat, why in KNOWN_NORMAL if pat in txt), '')}
    targets = []
    for t in score.get('targets') or []:
        targets.append({k: t.get(k) for k in ('name', 'cls', 'detected', 'best_confidence', 'detections',
                                              'aim_error_m', 'drop_error_m', 'drop_source', 'first_detected_s')})
    if git is None:     # (what the run wrote down at its start wins: no waiting for git at the end)
        git = (_recorded_source(run_dir) or find_git(run_dir, git_repo)) if live else _old_git(run_dir)
    return {
        'schema': 'astra.run_summary.v1', 'written_utc': utc_iso(time.time()), 'partial': bool(partial),
        'in_progress': in_progress,
        'note': note, 'run': os.path.basename(os.path.normpath(run_dir)), 'run_dir': run_dir,
        'run_dir_windows': windows_path(run_dir), 'result': _result(run_dir, phases, result, exit_code),
        'exit_code': exit_code, 'started_utc': utc_iso(started), 'ended_utc': utc_iso(ended),
        'started_local': local_time(started), 'duration_s': None if started is None or ended is None
        else round(ended - started, 1), 'logs_until': logs_until,
        'world': env.get('WORLD') or score.get('world', ''), 'profile': env.get('PROFILE', '') or 'default',
        'camera': env.get('CAMERA', ''), 'mission': env.get('MISSION', ''), 'manual': env.get('MANUAL', ''),
        'previous_run': env.get('PREVIOUS_RUN', ''),
        'git': git, 'ardupilot': _ardupilot(meta, recorder, run_dir),
        'detector': det, 'team_stack': ts,
        'recorder': {k: recorder.get(k) for k in ('url', 'vehicle_system', 'rows', 'armed_utc')} if recorder else {},
        'score': {'targets_found': score.get('targets_found'), 'targets_total': len(score.get('targets') or [])
                  if score else None, 'photos_taken': score.get('photos_taken'),
                  'photos_checked': score.get('photos_processed'), 'targets': targets} if score else {},
        'phases': phases, 'problems': {'errors': sum(1 for h in hits if h['level'] == 'error' and not h['normal']),
                                       'warnings': sum(1 for h in hits if h['level'] == 'warning' and not h['normal']),
                                       'normal': sum(1 for h in hits if h['normal']),
                                       'groups': groups,
                                       'after_end': {'errors': sum(1 for h in late if h['level'] == 'error'),
                                                     'warnings': sum(1 for h in late if h['level'] == 'warning'),
                                                     'groups': group_hits(late, per_file=40)}},
        'serious_status_texts': list(serious.values())[:100],
        'jetson': _jetson(run_dir, env), 'differences_from_field': differences_from_field(run_dir, env, det, ts),
        'csv': csvs, 'files': _files(run_dir), 'logs': log_files(run_dir), 'env': env,
    }


def render_markdown(s: dict) -> str:
    """The summary as Markdown (plain text works too)."""
    g, ap, det, ts = s['git'], s['ardupilot'], s['detector'], s['team_stack']
    L = [f'# Run summary: {s["run"]}', '']
    if s.get('in_progress'):
        L += [f'> **RUN IN PROGRESS** ({s["in_progress"]}). Nothing was written: this is what the files say so far. '
              'The real summary is written when the run ends.', '']
    elif s['partial']:
        L += ['> **PARTIAL SUMMARY.** Written from what was on disk at that moment, not by the normal end of the '
              'run. Why: see the note below.', '']
    if s['note']:
        L += [f'Note: {s["note"]}', '']
    if s.get('previous_run'):
        L += [f'This run was started by START OVER after `{os.path.basename(s["previous_run"])}` (same simulator, '
              'fresh drone and autopilot).', '']
    commit = (g.get('commit') or '')[:10]
    L += [f'- **Result:** {s["result"]}' + (f' (exit code {s["exit_code"]})' if s['exit_code'] not in (None, '') else ''),
          f'- **When:** {s["started_local"] or "?"} (local time), {mmss(s["duration_s"])} min:s from arming to the end',
          f'- **World:** {s["world"] or "?"}; **profile:** {s["profile"]}; **camera:** {s["camera"] or "?"}'
          + ('; Mission Planner mode (--manual)' if s['manual'] == '1' else ''),
          f'- **Code:** git {commit or "?"}' + (' **with uncommitted changes**' if g.get('dirty') else '')
          + (f' ({g["changed_files"]} files differ from the commit)' if g.get('dirty') and g.get('changed_files') else '')
          + (f' on {g["branch"]}' if g.get('branch') else '') + f' ({g.get("how", "")})',
          f'- **ArduPilot:** {ap["version"] or "?"}' + (f' ({ap["git"]})' if ap.get('git') else '')
          + f' (from {ap["source"]})',
          f'- **Team code ran:** {ts["where"]}' + (f', FCU {ts["fcu_url"]}' if ts.get('fcu_url') else ''),
          f'- **Detector:** {det["kind"] or "?"}' + (f', model {det["model"]}' if det.get('model') else '')
          + (f' (SHA-256 {det["sha256"][:16]}...)' if det.get('sha256') else '')
          + (f'; backend {det["backend"]}' if det.get('backend') else '; backend not recorded'),
          f'- **Run folder:** `{s["run_dir"]}`' + (f'  (Windows: `{s["run_dir_windows"]}`)' if s['run_dir_windows'] else ''),
          '']
    sc = s['score']
    L += ['## Score', '']
    if sc:
        L += [f'Targets found: {sc.get("targets_found")} of {sc.get("targets_total")}; photos: '
              f'{sc.get("photos_taken")} taken, {sc.get("photos_checked")} checked by the detector. '
              'Details: `score_report.md`.', '',
              '| Target | Class | Found | Best confidence | Aim error | Drop error |', '|---|---|---|---|---|---|']
        for t in sc['targets']:
            L.append(f'| {t["name"]} | {t["cls"]} | {"yes" if t["detected"] else "no"} | '
                     f'{t["best_confidence"] if t["detected"] else "-"} | '
                     f'{"-" if t["aim_error_m"] is None else str(t["aim_error_m"]) + " m"} | '
                     f'{"-" if t["drop_error_m"] is None else str(t["drop_error_m"]) + " m"} |')
    else:
        L.append('No score report in this run (score_run did not run or did not finish).')
    L += ['', '## Mission phases', '']
    if s['phases']:
        L += ['| Phase | Started (local) | After arming | Lasted | Detail |', '|---|---|---|---|---|']
        for p in s['phases']:
            L.append(f'| {p["code"]} | {local_time(p["start"])[11:]} | '
                     f'{"-" if p["t_since_arm_s"] is None else mmss(p["t_since_arm_s"])} | {mmss(p["duration_s"])} | '
                     f'{p["detail"]} |')
        # (WAITING is where every run starts: the live recorder writes no row for it)
        missing = [c for c in PHASE_ORDER if c != 'WAITING' and c not in {p['code'] for p in s['phases']}]
        if missing:
            L += ['', f'Phases never reached: {", ".join(missing)}.']
        L += ['', f'(from {s["phases"][0]["source"]})']
    else:
        L.append('No phase was recorded.')
    pr = s['problems']
    L += ['', '## Warnings and errors in the logs', '',
          f'{pr["errors"]} error lines and {pr["warnings"]} warning lines (plus {pr["normal"]} lines the docs '
          'call normal). Identical lines are grouped; "line" is where the first one is. See every line with '
          '`bash simulation/scripts/logs.sh errors`.', '']
    shown: dict[str, int] = {}
    real = [x for x in pr['groups'] if not x['normal']]
    if real:
        L += ['| Level | File:line | Time | Times | Text |', '|---|---|---|---|---|']
        for x in real:
            shown[x['file']] = shown.get(x['file'], 0) + 1
            if shown[x['file']] > MAX_GROUPS_SHOWN:
                continue
            text = x['text'].replace('|', '/')[:220]
            L.append(f'| {x["level"]} | `{x["file"]}:{x["line"]}` | {x["time"]} | {x["count"]} | {text} |')
        cut = {f: n - MAX_GROUPS_SHOWN for f, n in shown.items() if n > MAX_GROUPS_SHOWN}
        for f, n in cut.items():
            L.append(f'| | `{f}` | | | ... and {n} more kinds (logs.sh errors) |')
    else:
        L.append('None found.')
    normal = [x for x in pr['groups'] if x['normal']]
    if normal:
        L += ['', 'Lines the docs call normal: ' + '; '.join(
            f'`{x["file"]}:{x["line"]}` x{x["count"]} ({x["normal"]})' for x in normal[:8])]
    late = pr.get('after_end') or {}
    if late.get('errors') or late.get('warnings'):
        until = local_time(s.get('logs_until'))[11:] if s.get('logs_until') else '?'
        L += ['', f'After the run ended ({until}), while the simulator shut down or START OVER restarted it, the logs '
                  f'got {late["errors"]} more error and {late["warnings"]} more warning lines. They are not counted '
                  'above (stopping programs prints such lines); the first ones: ' + '; '.join(
                      f'`{x["file"]}:{x["line"]}` {x["text"][:90].replace("|", "/")}' for x in late['groups'][:4])]
    if s['serious_status_texts']:
        L += ['', '### Autopilot and team messages of severity WARNING or worse (csv/events.csv)', '',
              'Each different text once, with how often it came; the first time is UTC.', '']
        for m in s['serious_status_texts'][:30]:
            L.append(f'- {m["time"]} {m["severity"]} ({m["sender"]}): {m["text"]}'
                     + (f' (x{m["count"]})' if m['count'] > 1 else '')
                     + (f' - normal: {m["normal"]}' if m['normal'] else ''))
    jet = s.get('jetson') or {}
    if jet:
        L += ['', '## The Jetson', '']
        if jet.get('host'):
            L.append(f'- **Board:** {jet["host"]}' + (f' (link: {jet["link"]})' if jet.get('link') else ''))
        if jet.get('power'):
            L.append('- **Power setup:** ' + ', '.join(f'{k}={v}' for k, v in jet['power'].items()))
        else:
            L.append('- **Power setup:** not recorded (no power mode / jetson_clocks line in jetson/facts.env)')
        tg = jet.get('tegrastats') or {}
        if tg:
            gpu, cpu = tg.get('gpu_load_pct') or {}, tg.get('cpu_load_pct') or {}
            hot = max((tg.get('temp_max_c') or {}).items(), key=lambda kv: kv[1], default=None)
            L.append(f'- **During the flight (tegrastats, {tg["samples"]} samples):** GPU {gpu.get("avg", "?")}% average, '
                     f'{gpu.get("max", "?")}% max; CPU {cpu.get("avg", "?")}% average; memory up to '
                     f'{tg.get("ram_used_mb_max", "?")} of {tg.get("ram_total_mb", "?")} MB'
                     + (f'; hottest {hot[0]} {hot[1]:.1f} C' if hot else ''))
        else:
            L.append('- **During the flight:** no tegrastats samples (jetson/tegrastats.log missing or empty)')
        other = {k: v for k, v in (jet.get('facts') or {}).items() if k not in (jet.get('power') or {})}
        if other:
            L.append('- **Board facts:** ' + ', '.join(f'{k}={v}' for k, v in list(other.items())[:12])
                     + (' ...' if len(other) > 12 else ''))
    dif = s.get('differences_from_field') or {}
    if dif.get('items'):
        L += ['', '## What still differs from the field', '']
        L += [f'- {item}' for item in dif['items']]
        L += ['', f'(from {dif["source"]})']
    L += ['', '## Files of this run', '', '| File | Size | What it is |', '|---|---|---|']
    for f in s['files']:
        extra = f' ({f["files"]} files)' if f['name'].endswith('/') else ''
        L.append(f'| `{f["name"]}` | {human_size(f["size"])}{extra} | {f["what"]} |')
    L += ['', '### CSV files (`csv/`, open them in Excel, LibreOffice or pandas)', '',
          '| File | Rows | Windows path |', '|---|---|---|']
    for c in s['csv']:
        L.append(f'| `{c["name"]}` | {"missing" if c["rows"] is None else c["rows"]} | `{c["windows"] or c["path"]}` |')
    L += ['', '## Look deeper', '',
          '```bash', 'bash simulation/scripts/logs.sh errors           # every warning / error line, with file:line',
          'bash simulation/scripts/logs.sh tail team_stack  # the end of one log',
          'bash simulation/scripts/logs.sh csv              # where the CSV files are (also for Windows)',
          'bash simulation/scripts/logs.sh bundle           # one zip to send to a teammate', '```',
          '', 'How these files are made: docs/simulation/LOGS_AND_DATA.md', '']
    return '\n'.join(L)


def index_row(s: dict) -> dict:
    """The run's row of runs_index.csv."""
    by_cls = {}
    for t in (s['score'] or {}).get('targets', []):
        by_cls.setdefault(t['cls'], t)
    tent, hiker = by_cls.get('tent', {}), by_cls.get('person', {})
    g = s['git']
    det = s['detector']
    return {'run': s['run'], 'world': s['world'], 'started_utc': s['started_utc'],
            'duration_s': num(s['duration_s'], 1), 'result': s['result'], 'exit_code': s['exit_code'],
            'profile': s['profile'], 'detector': det['kind'], 'detector_model': det['model'],
            'detector_model_sha256': det['sha256'], 'ardupilot_version': s['ardupilot']['version'],
            'team_stack': s['team_stack']['where'], 'git_commit': (g.get('commit') or '')[:12],
            'git_dirty': '' if g.get('dirty') is None else int(bool(g['dirty'])),
            'targets_found': (s['score'] or {}).get('targets_found'),
            'targets_total': (s['score'] or {}).get('targets_total'),
            'tent_aim_error_m': tent.get('aim_error_m'), 'tent_drop_error_m': tent.get('drop_error_m'),
            'hiker_aim_error_m': hiker.get('aim_error_m'), 'hiker_drop_error_m': hiker.get('drop_error_m'),
            'photos_taken': (s['score'] or {}).get('photos_taken'),
            'photos_checked': (s['score'] or {}).get('photos_checked'),
            'warnings': s['problems']['warnings'], 'errors': s['problems']['errors'], 'run_dir': s['run_dir']}


def update_index(s: dict, runs_dir: str) -> str:
    """Add (or replace) the run's row in <runs_dir>/runs_index.csv. Returns the file path."""
    path = os.path.join(runs_dir, INDEX_FILE_NAME)
    rows = [r for r in read_csv(path) if r.get('run') != s['run']]
    rows.append(index_row(s))
    rows.sort(key=lambda r: r.get('run', ''))
    return write_csv_atomic(path, RUNS_INDEX_COLUMNS, rows)


def _write_text_atomic(path: str, text: str) -> None:
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    os.replace(tmp, path)


def finish_run(run_dir: str, result: str = '', exit_code=None, partial: bool = False, note: str = '',
               git_repo: str = '', export: bool = True, index: bool = True, quiet: bool = True,
               live: bool = False, refresh: bool = False, logs_until: float | None = None) -> dict:
    """Complete the CSV files, write run_summary.md + .json and the runs_index.csv row. Returns the summary.

    A partial summary never overwrites a complete one (a late recorder must not undo demo.sh's work).
    refresh=True: keep the result, exit code, note, git commit and logs_until the summary already has (the
    scorer's files or the photos changed after it was written); only the files are read again.
    """
    run_dir = os.path.abspath(run_dir)
    if not os.path.isdir(run_dir):
        raise FileNotFoundError(run_dir)
    old = load_json(os.path.join(run_dir, SUMMARY_JSON)) or {}
    if partial and old and not old.get('partial'):
        return old
    git = None
    if refresh and old:
        result = result or old.get('result', '')
        exit_code = old.get('exit_code') if exit_code is None else exit_code
        note = note or old.get('note', '')
        partial = partial or bool(old.get('partial'))
        logs_until = old.get('logs_until') if logs_until is None else logs_until
        if (old.get('git') or {}).get('commit') or not live:
            git = old.get('git') or None
    if export:
        try:
            from .csv_export import export_run
            export_run(run_dir, quiet=quiet)
        except Exception as exc:  # noqa: BLE001 - the summary is still worth writing
            note = (note + '; ' if note else '') + f'the CSV export failed ({exc})'
    s = build_summary(run_dir, result, exit_code, partial, note, git_repo, live, logs_until=logs_until, git=git)
    _write_text_atomic(os.path.join(run_dir, SUMMARY_MD), render_markdown(s))
    _write_text_atomic(os.path.join(run_dir, SUMMARY_JSON), json.dumps(s, indent=1, default=str) + '\n')
    if index:
        s['index_file'] = update_index(s, os.path.dirname(run_dir))
    return s


# ------------------------------------------------------------------------------------------ logs.sh helpers
def list_runs(runs_dir: str) -> list[dict]:
    """One dict per run folder, newest first: run, result, duration, targets, csv, summary."""
    out = []
    try:
        names = sorted((n for n in os.listdir(runs_dir) if os.path.isdir(os.path.join(runs_dir, n))), reverse=True)
    except OSError:
        return out
    for i, n in enumerate(names):
        d = os.path.join(runs_dir, n)
        s = load_json(os.path.join(d, SUMMARY_JSON)) or {}
        sc = load_json(os.path.join(d, 'score_report.json')) or {}
        running = run_in_progress(d) if i < 3 else ''     # (only the newest runs can still be going)
        out.append({'run': n, 'result': ('STILL RUNNING' if running else '') or s.get('result') or
                    ('scored' if sc else ''),
                    'partial': bool(s.get('partial')),
                    'duration': mmss(s.get('duration_s') if s else sc.get('duration_s')),
                    'targets': (f'{sc.get("targets_found")}/{len(sc.get("targets") or [])}' if sc else '-'),
                    'csv': os.path.isdir(os.path.join(d, CSV_DIR_NAME)), 'summary': bool(s)})
    return out


def secret_line(path: str, max_bytes: int = 5 * 1024 * 1024) -> int:
    """The first line number of a TEXT file that looks like a private key, a token or a password (0 = none;
    binary files and files over max_bytes are not read)."""
    try:
        if os.path.getsize(path) > max_bytes:
            return 0
        with open(path, 'rb') as f:
            data = f.read()
    except OSError:
        return 0
    if b'\0' in data[:4096]:
        return 0                            # a picture, a .tlog or another binary file
    text = data.decode('utf-8', 'replace')
    m = RE_SECRET_TEXT.search(text)
    return text.count('\n', 0, m.start()) + 1 if m else 0


def bundle(run_dir: str, out: str = '', photos: bool = False, max_mb: float = 20.0,
           note: str = '') -> tuple[str, list, list]:
    """Zip one run for a teammate. Returns (zip path, included names, [(skipped name, why)]).

    Left out unless asked: photos/ and detections/ (--photos), any file bigger than max_mb, every file whose
    name looks like it holds a secret (token, password, SSH key ...) and every text file with a line that
    looks like a private key, a token or a password.
    """
    run_dir = os.path.abspath(run_dir)
    name = os.path.basename(run_dir)
    if not out:
        out = os.path.join(os.path.dirname(os.path.dirname(run_dir)), 'bundles', name + '.zip')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    included, skipped = [], []
    limit = max_mb * 1024 * 1024
    tmp = out + '.tmp'
    with zipfile.ZipFile(tmp, 'w', compression=zipfile.ZIP_DEFLATED) as z:
        for root, dirs, files in os.walk(run_dir):
            rel_root = os.path.relpath(root, run_dir)
            top = rel_root.split(os.sep)[0]
            if top in ('photos', 'detections') and not photos:
                if rel_root == top:
                    skipped.append((top + '/', f'{len(files)} files: pictures and their sidecars '
                                                '(add --photos to include them)'))
                dirs[:] = []
                continue
            for f in sorted(files):
                rel = os.path.normpath(os.path.join(rel_root, f)) if rel_root != '.' else f
                p = os.path.join(root, f)
                if f.endswith('.tmp'):
                    continue
                if any(fnmatch.fnmatch(f.lower(), pat) for pat in SECRET_PATTERNS):
                    skipped.append((rel, 'the name looks like a secret (token / password / key): never bundled'))
                    continue
                try:
                    size = os.path.getsize(p)
                except OSError:
                    continue
                if size > limit:
                    skipped.append((rel, f'{human_size(size)} is bigger than --max-mb {max_mb:g}'))
                    continue
                bad = secret_line(p)
                if bad:
                    skipped.append((rel, f'line {bad} looks like a key, a token or a password: never bundled '
                                         '(look at it; remove the secret if you want to send the file)'))
                    continue
                z.write(p, os.path.join(name, rel))
                included.append(rel)
        manifest = [f'Run {name}, bundled {local_time(time.time())} by simulation/scripts/logs.sh bundle', ''] + \
                   ([f'NOTE: {note}', ''] if note else []) + \
                   ['Included:'] + [f'  {n}' for n in included] + ['', 'Left out:'] + \
                   [f'  {n}: {why}' for n, why in skipped] + ['']
        z.writestr(os.path.join(name, 'BUNDLE_MANIFEST.txt'), '\n'.join(manifest))
    os.replace(tmp, out)
    return out, included, skipped


# ------------------------------------------------------------------------------------------ command line
def build_parser() -> argparse.ArgumentParser:
    """Command-line options (one sub-command per job)."""
    ap = argparse.ArgumentParser(prog=PROG, description='Run summaries, the runs table and the logs.sh helpers.')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('finish', help='complete csv/, write run_summary.md/.json and the runs_index.csv row')
    p.add_argument('run_dir')
    p.add_argument('--result', default='', help='what happened, e.g. "landed" (default: worked out from the phases)')
    p.add_argument('--exit-code', default=None, help="demo.sh's exit code")
    p.add_argument('--partial', action='store_true', help='the run did not finish normally')
    p.add_argument('--note', default='', help='one extra line for the top of the summary')
    p.add_argument('--git-repo', default='', help='the git checkout the code came from (default: see find_git)')
    p.add_argument('--no-export', action='store_true', help='do not run the CSV exporter first')
    p.add_argument('--no-index', action='store_true', help='do not touch runs_index.csv')
    p.add_argument('--live', action='store_true', help='the run has just happened (demo.sh): look up the git '
                   'commit now (without it only what the run wrote down is used)')
    p.add_argument('--refresh', action='store_true', help='keep the result, exit code, note and git commit the '
                   'summary already has; read the files again (after the scorer rewrote them at shutdown)')
    p.add_argument('--logs-until', type=float, default=None, metavar='EPOCH',
                   help='log lines stamped after this moment (s since 1970) are listed apart as "after the run"')
    p = sub.add_parser('show', help='print run_summary.md (written first when it is missing; a run that is '
                       'still going is only shown, never written)')
    p.add_argument('run_dir')
    p = sub.add_parser('running', help='exit 0 and say why when the run is still going, else exit 1')
    p.add_argument('run_dir')
    p = sub.add_parser('source', help='write run_meta/source.json: which git commit runs (demo.sh, at the start)')
    p.add_argument('run_dir')
    p.add_argument('--git-repo', default='', help='the git checkout the code came from (default: see find_git)')
    p = sub.add_parser('errors', help='every warning / error line of every log of a run')
    p.add_argument('run_dir')
    p.add_argument('--all', action='store_true', help='also the lines the docs call normal')
    p.add_argument('--errors-only', action='store_true', help='only errors, no warnings')
    p = sub.add_parser('list', help='one line per run, newest first')
    p.add_argument('--runs-dir', default=os.path.join(os.environ.get('ASTRA_SIM_HOME') or
                                                      os.path.expanduser('~/.astra_sim'), 'runs'))
    p.add_argument('--limit', type=int, default=30)
    p = sub.add_parser('bundle', help='zip one run (no photos, no huge files, no secrets unless asked)')
    p.add_argument('run_dir')
    p.add_argument('--out', default='', help='the zip file (default ~/.astra_sim/bundles/<run>.zip)')
    p.add_argument('--photos', action='store_true', help='also the photos and annotated photos')
    p.add_argument('--max-mb', type=float, default=20.0, help='leave out files bigger than this (default 20)')
    p.add_argument('--note', default='', help='one line for the top of BUNDLE_MANIFEST.txt')
    p = sub.add_parser('winpath', help='print the Windows path of a WSL path')
    p.add_argument('path')
    return ap


def main(argv=None) -> int:
    """Entry point: `ros2 run sim_demo run_summary <cmd> ...` / `python3 -m sim_demo.run_summary <cmd> ...`."""
    args = build_parser().parse_args([a for a in (sys.argv[1:] if argv is None else argv) if not a.startswith('__')])
    if args.cmd == 'winpath':
        print(windows_path(args.path) or args.path)
        return EXIT_OK
    if args.cmd == 'list':
        runs = list_runs(args.runs_dir)
        if not runs:
            err(f'no run folders in {args.runs_dir}.', 'no simulator run has finished on this computer yet, or '
                'ASTRA_SIM_HOME points somewhere else.', 'run a demo first:  bash simulation/scripts/demo.sh')
            return EXIT_FAILED
        print(f'{"run":34} {"result":28} {"time":>6} {"targets":>7}  csv  summary')
        for r in runs[:args.limit]:
            res = (r['result'] + (' (partial)' if r['partial'] else ''))[:28]
            print(f'{r["run"]:34} {res:28} {r["duration"]:>6} {r["targets"]:>7}  {"yes" if r["csv"] else "-":3}  '
                  f'{"yes" if r["summary"] else "-"}')
        if len(runs) > args.limit:
            print(f'... and {len(runs) - args.limit} older runs (--limit N shows more)')
        return EXIT_OK
    run_dir = os.path.abspath(os.path.expanduser(args.run_dir))
    if not os.path.isdir(run_dir):
        err(f'the run folder {run_dir} does not exist.', 'a typo in the name, or the run is on another computer.',
            'list the runs with:  bash simulation/scripts/logs.sh list')
        return EXIT_FAILED
    try:
        if args.cmd == 'running':
            why = run_in_progress(run_dir)
            print(f'running: {why}' if why else 'finished (or stopped): nothing in it is changing')
            return EXIT_OK if why else EXIT_FAILED
        if args.cmd == 'source':
            gi = write_source(run_dir, args.git_repo)
            print(f'[{PROG}] code: git {(gi.get("commit") or "?")[:10]}' + (' with uncommitted changes' if gi.get('dirty')
                  else '') + f' ({gi.get("how", "")}) -> {os.path.join(run_dir, SOURCE_JSON)}')
            return EXIT_OK
        if args.cmd == 'finish':
            s = finish_run(run_dir, args.result, args.exit_code, args.partial, args.note, args.git_repo,
                           export=not args.no_export, index=not args.no_index, live=args.live,
                           refresh=args.refresh, logs_until=args.logs_until)
            print(f'[{PROG}] {"PARTIAL " if s.get("partial") else ""}summary: {os.path.join(run_dir, SUMMARY_MD)} '
                  f'({s["problems"]["errors"]} error lines, {s["problems"]["warnings"]} warning lines in the logs)')
            if s.get('index_file'):
                print(f'[{PROG}] runs table: {s["index_file"]}')
        elif args.cmd == 'show':
            path = os.path.join(run_dir, SUMMARY_MD)
            if not os.path.isfile(path):
                why = run_in_progress(run_dir)
                if why:             # never write into a run that is still going: show what is there so far
                    sys.stdout.write(render_markdown(build_summary(run_dir, partial=True, in_progress=why)))
                    return EXIT_OK
                print(f'[{PROG}] {os.path.basename(run_dir)} has no run summary yet: writing one now from its files '
                      '(run_summary.md/.json, csv/, a row in runs_index.csv)', file=sys.stderr)
                finish_run(run_dir, partial=True, note='written later by logs.sh from the files on disk: this run '
                           'never wrote one (it ran before run summaries existed, or it ended before its finish step)')
            with open(path, encoding='utf-8') as f:
                sys.stdout.write(f.read())
        elif args.cmd == 'errors':
            hits = scan_logs(run_dir)
            shown = 0
            for h in hits:
                if (h['normal'] and not args.all) or (args.errors_only and h['level'] != 'error'):
                    continue
                shown += 1
                tag = 'ERROR' if h['level'] == 'error' else 'warn '
                print(f'{tag} {h["file"]}:{h["line"]}  {h["text"]}' + (f'   [normal: {h["normal"]}]' if h['normal'] else ''))
            files = ', '.join(log_files(run_dir)) or 'none'
            print(f'[{PROG}] {shown} lines shown from: {files}', file=sys.stderr)
        elif args.cmd == 'bundle':
            out, inc, skipped = bundle(run_dir, args.out, args.photos, args.max_mb, args.note)
            print(f'[{PROG}] bundle: {out} ({len(inc)} files, {human_size(os.path.getsize(out))})')
            win = windows_path(out)
            if win:
                print(f'[{PROG}] Windows path: {win}')
            for n, why in skipped:
                print(f'[{PROG}]   left out {n}: {why}')
    except OSError as exc:
        err(f'cannot write into {run_dir} ({exc}).', 'the disk is full or the folder is not yours.',
            'check with:  df -h ~ ; ls -ld ' + run_dir)
        return EXIT_FAILED
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
