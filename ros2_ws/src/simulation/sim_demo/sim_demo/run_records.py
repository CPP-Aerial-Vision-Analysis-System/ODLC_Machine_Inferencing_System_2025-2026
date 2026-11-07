"""
sim_demo/run_records.py -- PURPOSE: start, stop and finish the RECORDS of one run (the live flight recorder,
the CSV files and the run summary) for whoever owns that run: demo.sh, START OVER (run_reset.py) or a person.

Why this exists: a run can end in several ways. demo.sh normally finishes its own run (finish_records in
demo.sh). But START OVER (start_over.sh / the demo panel's button, code in run_reset.py) can take the
simulator over in the middle of a run and then starts a NEW run folder itself. Both runs must still get
their live CSV files and a run summary. The pieces here do that and are small enough to call from anywhere:

    start_recorder(run_dir, url)     start sim_demo flight_recorder for a run (its own MAVProxy output, e.g.
                                     udpin:127.0.0.1:14555); it writes <run>/flight_recorder.pid
    stop_recorder(run_dir)           stop that run's recorder politely (Ctrl+C, then TERM after 10 s)
    finish(run_dir, result, code)    stop the recorder, complete csv/, write run_summary.md/.json + the
                                     runs_index.csv row (run_summary.finish_run)
    refresh(run_dir)                 write the summary again with the result it already has (after the
                                     scorer rewrote its files, or the photos were moved in later)
    start_over_hook(new, old, ...)   what START OVER calls after it restarted the per-run helpers (step 10):
                                     refresh the OLD run, copy its run_meta, start the NEW run's recorder
    watcher_end_hook(run, rc, ...)   what START OVER's run watcher calls after it collected the run

Command line (the same, for scripts and people):
    python3 -m sim_demo.run_records start   RUN_DIR [--url udpin:127.0.0.1:14555] [--mission FILE] [--parent-pid P]
    python3 -m sim_demo.run_records stop    RUN_DIR
    python3 -m sim_demo.run_records finish  RUN_DIR --result TEXT [--exit-code N] [--note TEXT]
    python3 -m sim_demo.run_records refresh RUN_DIR
Exit codes: 0 = done, 1 = failed (WHAT / Likely cause / Fix printed), 2 = bad usage.
The recorder only LISTENS to MAVLink: starting or stopping it can never change a flight.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

if __package__ in (None, ''):          # run as a plain file
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = 'sim_demo'           # noqa: A001 - lets the relative imports below work

from .run_summary import SUMMARY_JSON, finish_run, load_json, run_env  # noqa: E402

PROG = 'run_records'
EXIT_OK, EXIT_FAILED, EXIT_USAGE = 0, 1, 2
PID_FILE = 'flight_recorder.pid'
RECORDER_LOG = 'flight_recorder.log'


def _say(text: str) -> None:
    print(f'[{PROG}] {text}', flush=True)


def _warn(what: str, consequence: str = '', fix: str = '') -> None:
    parts = [f'[{PROG}] WARNING: {what}']
    if consequence:
        parts.append(f'Consequence: {consequence}')
    if fix:
        parts.append(f'Fix: {fix}')
    print(' '.join(parts), file=sys.stderr, flush=True)


def _err(what: str, why: str = '', fix: str = '') -> None:
    parts = [f'[{PROG}] ERROR: {what}']
    if why:
        parts.append(f'Likely cause: {why}')
    if fix:
        parts.append(f'Fix: {fix}')
    print(' '.join(parts), file=sys.stderr, flush=True)


# ------------------------------------------------------------------------------------------ the run's settings
def recorder_url(run_dir: str) -> str:
    """The recorder's MAVLink address of a run (RECORDER= in its env files, '' = the run had no recorder).
    Older runs wrote MAVProxy's side of it (udp:HOST:PORT): the recorder listens there (udpin:HOST:PORT)."""
    url = run_env(run_dir).get('RECORDER', '').strip()
    return 'udpin:' + url[4:] if url.startswith('udp:') else url


# ------------------------------------------------------------------------------------------ the process
def _pid_and_url(run_dir: str) -> tuple[int | None, str]:
    try:
        with open(os.path.join(run_dir, PID_FILE), encoding='utf-8') as f:
            parts = f.read().split()
        return int(parts[0]), (parts[1] if len(parts) > 1 else '')
    except (OSError, ValueError, IndexError):
        return None, ''


def _is_recorder(pid: int) -> bool:
    """True when pid is a live sim_demo flight recorder (Linux /proc; never signals anything)."""
    if os.name == 'nt':
        return False            # (Windows: os.kill would TERMINATE the process; run folders live in WSL anyway)
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            cmd = f.read().replace(b'\0', b' ').decode('utf-8', 'replace')
        with open(f'/proc/{pid}/stat', encoding='utf-8') as f:
            stat = f.read()
        return 'flight_recorder' in cmd and stat[stat.rindex(')') + 2] != 'Z'
    except (OSError, ValueError, IndexError):
        return False


def recorder_pid(run_dir: str) -> int | None:
    """The pid of the flight recorder that is recording this run right now, else None."""
    pid, _url = _pid_and_url(run_dir)
    return pid if pid and _is_recorder(pid) else None


def stop_recorder(run_dir: str, timeout_s: float = 10.0) -> str:
    """Stop the run's recorder: Ctrl+C (it closes its files cleanly), TERM after timeout_s.
    Returns 'not running', 'stopped' or 'stopped hard'."""
    pid = recorder_pid(run_dir)
    if pid is None:
        return 'not running'
    for sig, wait in ((signal.SIGINT, timeout_s), (signal.SIGTERM, 3.0)):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            return 'stopped'
        except PermissionError:
            return 'not running'
        end = time.time() + wait
        while time.time() < end:
            if not _is_recorder(pid):
                return 'stopped' if sig == signal.SIGINT else 'stopped hard'
            time.sleep(0.2)
    return 'stopped hard'


def port_free(url: str) -> bool:
    """For udpin:HOST:PORT: True when nobody listens there yet (other kinds of address: True)."""
    if not url.startswith('udpin:'):
        return True
    host, _, port = url[len('udpin:'):].rpartition(':')
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind((host or '0.0.0.0', int(port)))
        finally:
            s.close()
        return True
    except (OSError, ValueError):
        return False


def _popen_spawn(argv: list, cwd: str, env: dict, log_path: str) -> int:
    """Start a program in its OWN session (it outlives the caller and a closed window), output appended to
    log_path. The same shape as run_reset.ProcessOps.spawn, so START OVER can pass its own (and its tests a fake)."""
    with open(log_path, 'ab') as log:
        return subprocess.Popen(list(argv), cwd=cwd or None, env=dict(env), stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True).pid


def start_recorder(run_dir: str, url: str, mission: str = '', parent_pid: int = 0, env: dict | None = None,
                   python: str = '', wait_s: float = 1.5, spawn=None) -> tuple[int | None, str]:
    """Start sim_demo flight_recorder for run_dir, detached. Returns (pid or None, one plain line saying what
    happened). spawn(argv, cwd, env, log_path) -> pid starts the program (default: its own session)."""
    if not url:
        return None, 'no live CSV for this run: the run it continues had no flight recorder (--no-recorder)'
    if recorder_pid(run_dir):
        return recorder_pid(run_dir), 'the flight recorder of this run is already running'
    if not port_free(url):
        return None, (f'WARNING: {url} is already in use, so this run records no live CSV (photos, detections and '
                      'the summary still come). Likely cause: an older flight recorder still runs. Fix: '
                      'pkill -f sim_demo.flight_recorder')
    argv = [python or sys.executable, '-m', 'sim_demo.flight_recorder', '--run-dir', run_dir, '--url', url]
    if mission:
        argv += ['--mission', mission]
    if parent_pid:
        argv += ['--parent-pid', str(parent_pid)]
    os.makedirs(run_dir, exist_ok=True)
    pid = (spawn or _popen_spawn)(argv, run_dir, dict(os.environ if env is None else env),
                                  os.path.join(run_dir, RECORDER_LOG))
    end = time.time() + wait_s
    while time.time() < end:
        if os.name != 'nt' and not _is_recorder(pid):
            return None, ('WARNING: the flight recorder stopped right away, so this run records no live CSV. The '
                          f'reason is in {os.path.join(run_dir, RECORDER_LOG)}')
        time.sleep(0.1)
    return pid, f'flight recorder: MAVLink on {url} -> {os.path.join(run_dir, "csv")} (live CSV)'


# ------------------------------------------------------------------------------------------ the summary
def finish(run_dir: str, result: str, exit_code=None, note: str = '', git_repo: str = '',
           logs_until: float | None = None, live: bool = True) -> dict:
    """Stop the recorder, complete csv/, write run_summary.md/.json and the runs_index.csv row."""
    stopped = stop_recorder(run_dir)
    if stopped == 'stopped hard':
        note = (note + '; ' if note else '') + 'the flight recorder had to be stopped hard (last second of csv/ may miss)'
    return finish_run(run_dir, result=result, exit_code=exit_code, note=note, git_repo=git_repo, live=live,
                      logs_until=logs_until)


def refresh(run_dir: str, default_result: str = '', note: str = '') -> dict:
    """Write the run's summary again with the result it already has (after the scorer rewrote its files or
    the photos were moved in); when it has none yet, write one with default_result."""
    old = load_json(os.path.join(run_dir, SUMMARY_JSON)) or {}
    stop_recorder(run_dir)
    if old and not old.get('partial'):
        return finish_run(run_dir, refresh=True, note=note)
    return finish_run(run_dir, result=default_result or old.get('result', ''), note=note or old.get('note', ''),
                      live=True, exit_code=old.get('exit_code'))


def inherit_meta(new_run: str, old_run: str) -> list[str]:
    """Copy run_meta/*.json of the run START OVER continued (same autopilot program, detector and team code)
    into the new run, with "inherited_from"; never overwrites. Returns the copied names."""
    src, dst = os.path.join(old_run, 'run_meta'), os.path.join(new_run, 'run_meta')
    copied = []
    try:
        names = sorted(n for n in os.listdir(src) if n.endswith('.json'))
    except OSError:
        return copied
    os.makedirs(dst, exist_ok=True)
    for n in names:
        target = os.path.join(dst, n)
        if os.path.exists(target):
            continue
        data = load_json(os.path.join(src, n))
        if isinstance(data, dict):
            data['inherited_from'] = old_run
            with open(target, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=1)
        else:
            shutil.copy2(os.path.join(src, n), target)
        copied.append(n)
    return copied


def start_over_hook(new_run: str, old_run: str, mission: str = '', env: dict | None = None,
                    spawn=None, wait_s: float = 1.5) -> list[tuple[str, str]]:
    """START OVER, after the per-run helpers restarted (run_reset step 10). Returns (kind, text) progress lines.

    1. the OLD run: its recorder is stopped (if it still runs) and its summary written again, now with the
       scorer's final files (score_run wrote them when it was restarted) and the photos archived in step 5;
    2. the NEW run inherits the old run's run_meta and RECORDER setting;
    3. the NEW run gets its own flight recorder on the same MAVProxy output (MAVProxy was restarted with the
       same command line, so the output is there again)."""
    lines: list[tuple[str, str]] = []
    if old_run and os.path.isdir(old_run):
        try:
            refresh(old_run, default_result='ended by START OVER (a new run started: '
                                            f'{os.path.basename(os.path.normpath(new_run))})')
            lines.append(('info', f'run summary of the previous run written: {os.path.join(old_run, "run_summary.md")}'))
        except Exception as exc:  # noqa: BLE001 - never stop a START OVER for a summary
            lines.append(('warn', f'the previous run\'s summary could not be written ({exc}); write it later with '
                                  f'python3 -m sim_demo.run_summary finish {old_run}'))
        inherit_meta(new_run, old_run)
    url = recorder_url(old_run) if old_run else ''
    try:
        with open(os.path.join(new_run, 'start_over.env'), 'a', encoding='utf-8') as f:
            f.write(f'RECORDER={url}\n')
    except OSError:
        pass
    pid, text = start_recorder(new_run, url, mission, env=env, spawn=spawn, wait_s=wait_s)
    lines.append(('info' if pid or not text.startswith('WARNING') else 'warn', text.replace('WARNING: ', '')))
    return lines


def watcher_end_hook(run_dir: str, rc: int, reason: str, new_start_over: bool = False) -> list[str]:
    """START OVER's run watcher, after it collected the run (photos, score report): stop the recorder and
    write the run's summary. Returns plain lines for its log."""
    if new_start_over:
        result, code = 'ended by START OVER (a new run started)', rc
    else:
        result = {0: 'landed: mission complete', 3: f'timed out: {reason}'}.get(rc, f'ended: {reason}')
        code = rc
    try:
        s = finish(run_dir, result, code)
        return [f'run summary: {os.path.join(run_dir, "run_summary.md")} ({s["problems"]["errors"]} error lines, '
                f'{s["problems"]["warnings"]} warning lines in the logs)']
    except Exception as exc:  # noqa: BLE001 - the flight is over; say it, never crash the watcher
        return [f'WARNING: the run summary could not be written ({exc}). Fix: python3 -m sim_demo.run_summary '
                f'finish {run_dir}']


# ------------------------------------------------------------------------------------------ command line
def build_parser() -> argparse.ArgumentParser:
    """Command-line options (one sub-command per job)."""
    ap = argparse.ArgumentParser(prog=PROG, description='Start, stop and finish the records of one run.')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('start', help='start the flight recorder for a run')
    p.add_argument('run_dir')
    p.add_argument('--url', default='', help='MAVLink address (default: RECORDER= of the run, else '
                   'udpin:127.0.0.1:14555)')
    p.add_argument('--mission', default='')
    p.add_argument('--parent-pid', type=int, default=0)
    p = sub.add_parser('stop', help='stop the flight recorder of a run')
    p.add_argument('run_dir')
    p = sub.add_parser('finish', help='stop the recorder and write the run summary')
    p.add_argument('run_dir')
    p.add_argument('--result', required=True)
    p.add_argument('--exit-code', default=None)
    p.add_argument('--note', default='')
    p = sub.add_parser('refresh', help='write the summary again with the result it already has')
    p.add_argument('run_dir')
    return ap


def main(argv=None) -> int:
    """Entry point: `python3 -m sim_demo.run_records <cmd> ...` / `ros2 run sim_demo run_records <cmd> ...`."""
    args = build_parser().parse_args([a for a in (sys.argv[1:] if argv is None else argv) if not a.startswith('__')])
    run_dir = os.path.abspath(os.path.expanduser(args.run_dir))
    if not os.path.isdir(run_dir):
        _err(f'the run folder {run_dir} does not exist.', 'a typo in the name, or the run is on another computer.',
             'list the runs with:  bash simulation/scripts/logs.sh list')
        return EXIT_FAILED
    try:
        if args.cmd == 'start':
            pid, text = start_recorder(run_dir, args.url or recorder_url(run_dir) or 'udpin:127.0.0.1:14555',
                                       args.mission, args.parent_pid)
            (_say if pid else _warn)(text.replace('WARNING: ', ''))
            return EXIT_OK if pid else EXIT_FAILED
        if args.cmd == 'stop':
            _say(f'flight recorder of {os.path.basename(run_dir)}: {stop_recorder(run_dir)}')
            return EXIT_OK
        if args.cmd == 'finish':
            s = finish(run_dir, args.result, args.exit_code, args.note)
        else:
            s = refresh(run_dir)
        _say(f'summary: {os.path.join(run_dir, "run_summary.md")} ({s["problems"]["errors"]} error lines, '
             f'{s["problems"]["warnings"]} warning lines in the logs)')
    except OSError as exc:
        _err(f'cannot write into {run_dir} ({exc}).', 'the disk is full or the folder is not yours.',
             'check with:  df -h ~ ; ls -ld ' + run_dir)
        return EXIT_FAILED
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
