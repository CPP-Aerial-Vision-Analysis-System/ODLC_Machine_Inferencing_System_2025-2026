r"""
sim_demo/mission_planner.py -- PURPOSE: open, close and check Mission Planner (the Windows ground-station
program the team flies with) from Ubuntu inside WSL. The demo panel's OPEN / CLOSE MISSION PLANNER buttons
and its "Mission Planner: running" light use it, and so does simulation/scripts/mission_planner.sh.

Why this can work at all: the simulator runs in Ubuntu inside WSL, but Mission Planner is a Windows
program. WSL's "interop" lets a Linux program start any Windows program (an .exe file on the C: drive,
which Ubuntu sees under /mnt/c). This module only ever runs three Windows programs:
    tasklist.exe        Windows' task list: "is MissionPlanner.exe running?" (it only looks)
    taskkill.exe        "please close MissionPlanner.exe" (like clicking its X); with /F: "close it now"
    MissionPlanner.exe  Mission Planner itself, to open it
What is in here:
    is_wsl(), find_exe(), find_windows_tool()   where are we, and where are those programs?
    availability()      can the buttons work on this computer? If not, a sentence saying why
    is_running()        is Mission Planner open right now?
    start()             opens it, detached (it stays open when the panel or the terminal closes); it
                        refuses politely when Mission Planner is already open (one copy is enough)
    stop()              closes it politely; forces it closed only if it is still open after 5 s
    MissionPlannerMonitor  a background thread for the panel: checks every 2 s and carries out the button
                        presses, so the window never waits for Windows (starting a program takes seconds)
Every check and action returns a Result with a short sentence for people. Normal problems (not inside WSL,
not installed, interop switched off, Windows slow to answer) never raise an error: they become that
sentence. No ROS and no Qt in here. Closing Mission Planner never disturbs the simulated drone: its
autopilot has no ground-station failsafe (FS_GCS_ENABLE 0 in sim_models/config/astra_sitl.parm).

Run by hand:   python3 -m sim_demo.mission_planner status|start|stop
               (simulation/scripts/mission_planner.sh does exactly this, and works without a build)
"""
from __future__ import annotations

import argparse
import csv
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable

IMAGE_NAME = 'MissionPlanner.exe'      # the program's name in Windows' task list
ENV_EXE = 'MISSION_PLANNER_EXE'        # optional environment variable: where MissionPlanner.exe is
DEFAULT_EXE_CANDIDATES = (             # where the official installer puts it (C: as Ubuntu sees it)
    '/mnt/c/Program Files (x86)/Mission Planner/MissionPlanner.exe',
    '/mnt/c/Program Files/Mission Planner/MissionPlanner.exe',
)
SYSTEM32_DIR = '/mnt/c/Windows/System32'   # tasklist.exe / taskkill.exe when they are not on the PATH
PROC_VERSION = '/proc/version'             # the Linux kernel's name; inside WSL it contains "microsoft"
SESSION_VAR = 'ASTRA_SIM_SESSION'          # the simulator scripts' process tag (simulation/scripts/_common.sh)
CONNECT_HINT = 'In Mission Planner: UDP -> CONNECT -> port 14550'

QUERY_TIMEOUT_S = 10.0    # [s] tasklist / taskkill normally answer in well under a second
POLITE_WAIT_S = 5.0       # [s] time Mission Planner gets to close by itself before it is forced
FORCE_WAIT_S = 3.0        # [s] time a forced close may take to show up in the task list
START_CONFIRM_S = 8.0     # [s] start() waits this long to see the new program in the task list
POLL_S = 0.5              # [s] pause between two task-list checks while waiting

NOT_WSL_MESSAGE = ('Not inside WSL on Windows, so there is no Mission Planner to open from here '
                   '(these buttons work on the Windows laptop that runs the simulator).')
INTEROP_MESSAGE = ('Ubuntu cannot start Windows programs here: WSL "interop" seems to be switched off '
                   '(check "[interop] enabled=true" in /etc/wsl.conf).')

def _via_shell(args, cwd):
    """Wrap a Windows command so a tiny Linux shell, not this process, enters `cwd` and starts it.

    Why: Python holds its global lock while it starts a child process and waits for the child's exec().
    Changing into a C: folder (/mnt/c, slow) and exec'ing a Windows .exe through WSL interop takes a few
    hundred milliseconds, and during that time the demo panel's other threads, including the live camera
    view, freeze: the video hitched every few seconds while the Mission Planner light was polling. Starting
    /bin/sh (a fast Linux program) and letting IT do the slow cd + exec keeps the panel smooth. The
    Windows program still starts in the same folder as before.
    """
    if not cwd:
        return list(args)
    return ['/bin/sh', '-c', 'cd "$0" 2>/dev/null; exec "$@"', cwd, *args]


def _run_smooth(args, cwd=None, **kwargs):
    """subprocess.run for Windows programs without freezing the panel (see _via_shell)."""
    return subprocess.run(_via_shell(args, cwd), **kwargs)


def _popen_smooth(args, cwd=None, **kwargs):
    """subprocess.Popen for Windows programs without freezing the panel (see _via_shell)."""
    return subprocess.Popen(_via_shell(args, cwd), **kwargs)


# The outside world in one place. The unit tests replace these five names to pretend to be Windows, so
# no test ever starts or stops a real program.
_run = _run_smooth
_popen = _popen_smooth
_which = shutil.which
_sleep = time.sleep
_now = time.monotonic


@dataclass(frozen=True)
class Result:
    """What a check or an action found out.

    ok:       True = the question was answered, or the action did what was asked
    message:  one short sentence for people (the panel's status line and event log, the terminal)
    running:  is Mission Planner open now? True / False, or None = could not tell
    label:    two or three words for small displays (the panel's status light), e.g. "not running"
    """
    ok: bool
    message: str
    running: bool | None = None
    label: str = ''


# ------------------------------------------------------------------ where are we, where are the programs
def is_wsl() -> bool:
    """True when this Linux runs inside WSL on Windows (its kernel name then contains "microsoft",
    e.g. "5.15.167.4-microsoft-standard-WSL2"). Reads PROC_VERSION (the tests point it elsewhere)."""
    try:
        with open(PROC_VERSION, encoding='utf-8', errors='replace') as f:
            return 'microsoft' in f.read().lower()
    except OSError:
        return False


def to_wsl_path(path: str) -> str:
    r"""Turn a Windows path into the path Ubuntu sees: 'C:\Tools\MissionPlanner.exe' ->
    '/mnt/c/Tools/MissionPlanner.exe'. Linux paths come back unchanged. (So MISSION_PLANNER_EXE may be
    copied straight from the address bar of the Windows file explorer.)"""
    p = path.strip().strip('"')
    if len(p) >= 3 and p[0].isalpha() and p[1] == ':' and p[2] in '\\/':
        return f'/mnt/{p[0].lower()}/' + p[3:].replace('\\', '/')
    return p


def exe_candidates(exe_override: str = '') -> list[str]:
    """Where find_exe() looks, in this order.

    A given path (`exe_override`: the panel's mission_planner_exe parameter or the script's --exe), else
    $MISSION_PLANNER_EXE, REPLACES the usual install folders: if someone points at a path, a typo there
    must be reported, not silently ignored.
    """
    given = exe_override or os.environ.get(ENV_EXE, '')
    if given.strip():
        return [to_wsl_path(given)]
    return list(DEFAULT_EXE_CANDIDATES)


def find_exe(exe_override: str = '') -> str | None:
    """Path of MissionPlanner.exe, or None when it is not where we look (see exe_candidates)."""
    for path in exe_candidates(exe_override):
        if os.path.isfile(path):
            return path
    return None


def find_windows_tool(name: str) -> str | None:
    """Path of a Windows command such as 'tasklist.exe'. WSL normally puts C:\\Windows\\System32 on the
    PATH; when that is switched off (appendWindowsPath=false in /etc/wsl.conf), look there directly."""
    found = _which(name)
    if found:
        return found
    path = os.path.join(SYSTEM32_DIR, name)
    return path if os.path.isfile(path) else None


def not_installed_message(exe_override: str = '') -> str:
    """The sentence shown when MissionPlanner.exe is missing (names every place that was searched)."""
    where = ' or '.join(f'"{p}"' for p in exe_candidates(exe_override))
    return (f'Mission Planner is not installed: there is no {where}. Install it from ardupilot.org, or '
            f'set {ENV_EXE} (panel parameter mission_planner_exe) to where MissionPlanner.exe is.')


def availability(exe_override: str = '') -> Result:
    """Can OPEN / CLOSE work on this computer? ok=False comes with the reason (for the tooltip of the
    greyed-out buttons). Only looks at files: quick, and never starts a program."""
    if not is_wsl():
        return Result(False, NOT_WSL_MESSAGE, None, 'not on Windows')
    if find_exe(exe_override) is None:
        return Result(False, not_installed_message(exe_override), None, 'not installed')
    return Result(True, 'Mission Planner is installed.', None, 'installed')


# ------------------------------------------------------------------ talking to Windows
def _first_line(text: str) -> str:
    """First non-empty line of a program's output (for short error sentences)."""
    for line in (text or '').splitlines():
        if line.strip():
            return line.strip()[:160]
    return ''


def _windows_command(args: list[str], timeout: float = QUERY_TIMEOUT_S):
    """Run a Windows console program (tasklist.exe / taskkill.exe) through WSL interop.

    Returns (finished process, '') or (None, a sentence saying what went wrong).
    The working directory is the program's own folder on C:, because a Windows program started from a
    Linux folder would get a wsl.localhost network path as its working directory.
    """
    name = os.path.basename(args[0])
    try:
        done = _run(args, stdin=subprocess.DEVNULL, capture_output=True, text=True, errors='replace',
                    timeout=timeout, cwd=os.path.dirname(args[0]) or None)
    except subprocess.TimeoutExpired:
        return None, f'Windows did not answer within {timeout:.0f} s ({name}); it will be asked again.'
    except OSError as exc:      # "Exec format error" = interop off; or the file could not be run
        return None, f'{INTEROP_MESSAGE} ({name}: {exc.strerror or exc})'
    return done, ''


def parse_tasklist(text: str, image: str = IMAGE_NAME) -> list[str]:
    """Process ids (as text) of `image` in tasklist.exe's output.

    Understands the CSV form we ask for     "MissionPlanner.exe","46792","Console","1","468,956 K"
    and the plain table form                MissionPlanner.exe    46792 Console    1    468,956 K
    When nothing matches, Windows prints a sentence instead ("INFO: No tasks are running which match the
    specified criteria.", translated on a non-English Windows). That sentence matches no row, so the
    language of Windows does not matter.
    """
    pids = []
    for line in (text or '').splitlines():
        line = line.strip()
        if not line:
            continue
        fields = next(csv.reader([line])) if line.startswith('"') else line.split()
        if len(fields) >= 2 and fields[0].strip().lower() == image.lower():
            pids.append(fields[1].strip())
    return pids


def is_running() -> Result:
    """Is Mission Planner open on Windows? Asks tasklist.exe (it only looks; about 0.3 s)."""
    if not is_wsl():
        return Result(False, NOT_WSL_MESSAGE, None, 'not on Windows')
    tool = find_windows_tool('tasklist.exe')
    if tool is None:
        return Result(False, 'tasklist.exe was not found (is the C: drive at /mnt/c?).', None, 'unknown')
    # /FI keeps only Mission Planner; /FO CSV + /NH = one '"name","PID",...' line per copy, no heading.
    done, error = _windows_command([tool, '/FI', f'IMAGENAME eq {IMAGE_NAME}', '/FO', 'CSV', '/NH'])
    if error:
        return Result(False, error, None, 'unknown')
    if done.returncode != 0:
        detail = _first_line(done.stderr or done.stdout) or f'exit code {done.returncode}'
        return Result(False, f'tasklist.exe failed: {detail}', None, 'unknown')
    pids = parse_tasklist(done.stdout)
    if pids:
        return Result(True, f'Mission Planner is running (Windows process {", ".join(pids)}).', True,
                      'running')
    return Result(True, 'Mission Planner is not running.', False, 'not running')


def _reap_later(proc) -> None:
    """WSL keeps a small Linux helper process for every Windows program it started; it lives exactly as
    long as Mission Planner. Collect its exit status in a daemon thread when Mission Planner closes, so it
    never lingers as a "zombie" next to the panel (a daemon thread never keeps the panel from quitting)."""
    threading.Thread(target=proc.wait, name='mission_planner_reaper', daemon=True).start()


def start(exe_override: str = '', confirm_s: float = START_CONFIRM_S) -> Result:
    """Open Mission Planner on Windows, unless it is already open.

    It is started DETACHED, so it stays open when the panel or the terminal closes:
      * start_new_session: its own process group, so Ctrl+C or closing the terminal does not reach it;
      * stdin/stdout/stderr -> /dev/null: nothing ties it to our terminal or to a pipe (a script that
        captures our output, like mission_planner.sh, would otherwise wait until Mission Planner closes);
      * no ASTRA_SIM_SESSION in its environment: stop_sim.sh stops every process that carries the
        simulator's session tag, and Mission Planner is not part of the simulator;
      * working directory = the Mission Planner folder, where Windows programs expect to start.
    Then it waits up to `confirm_s` seconds until MissionPlanner.exe shows up in Windows' task list.
    """
    avail = availability(exe_override)
    if not avail.ok:
        return avail
    now = is_running()
    if now.running:
        return Result(True, 'Mission Planner is already open (not starting a second copy).', True, 'running')
    if not now.ok:
        return now          # Windows cannot even be asked: starting a program would fail the same way
    exe = find_exe(exe_override)
    if exe is None:         # it was there a moment ago (uninstalled meanwhile?)
        return Result(False, not_installed_message(exe_override), None, 'not installed')
    env = {k: v for k, v in os.environ.items() if k != SESSION_VAR}
    try:
        proc = _popen([exe], cwd=os.path.dirname(exe), env=env, stdin=subprocess.DEVNULL,
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    except OSError as exc:
        return Result(False, f'Could not start Mission Planner: {exc.strerror or exc}. {INTEROP_MESSAGE}',
                      False, 'not running')
    end = _now() + confirm_s
    while True:
        if is_running().running:
            _reap_later(proc)
            return Result(True, f'Mission Planner is opening (its window takes 10-30 s). {CONNECT_HINT}.',
                          True, 'running')
        code = proc.poll()
        if code is not None:      # the helper ended: Mission Planner closed again (or never started)
            return Result(False, f'Mission Planner closed right after starting (exit code {code}).',
                          False, 'not running')
        if _now() >= end:
            _reap_later(proc)
            return Result(True, 'Mission Planner was started, but Windows does not list it yet; '
                                'give it a few seconds.', None, 'starting ...')
        _sleep(POLL_S)


def _wait_until_closed(seconds: float) -> bool:
    """Check the task list until Mission Planner is gone (True) or `seconds` have passed (False).
    A check where Windows does not answer counts as "not gone yet"."""
    end = _now() + seconds
    while True:
        found = is_running()
        if found.ok and not found.running:
            return True
        if _now() >= end:
            return False
        _sleep(POLL_S)


def stop(polite_wait_s: float = POLITE_WAIT_S) -> Result:
    """Close Mission Planner (every copy that runs).

    First politely: taskkill.exe /IM MissionPlanner.exe sends the same "please close" message as clicking
    its X, so it can save its settings. Only if it is still open after `polite_wait_s` seconds (it hangs,
    or it shows a question) is it forced closed: taskkill.exe /F.
    """
    now = is_running()
    if not now.ok:
        return now
    if not now.running:
        return Result(True, 'Mission Planner is not open (nothing to close).', False, 'not running')
    tool = find_windows_tool('taskkill.exe')
    if tool is None:
        return Result(False, 'taskkill.exe was not found (is the C: drive at /mnt/c?).', True, 'running')
    done, error = _windows_command([tool, '/IM', IMAGE_NAME])
    if error:
        return Result(False, error, None, 'unknown')
    polite_sent = done.returncode == 0
    # If the polite request could not even be delivered, waiting is pointless: check once, then force.
    if _wait_until_closed(polite_wait_s if polite_sent else 0.0):
        return Result(True, 'Mission Planner closed.', False, 'not running')
    done, error = _windows_command([tool, '/F', '/IM', IMAGE_NAME])
    if error:
        return Result(False, error, None, 'unknown')
    if _wait_until_closed(FORCE_WAIT_S):
        why = (f'it did not close within {polite_wait_s:.0f} s' if polite_sent
               else 'it did not accept a polite close')
        return Result(True, f'Mission Planner was forced to close ({why}).', False, 'not running')
    detail = _first_line(done.stderr or done.stdout)
    return Result(False, f'Could not close Mission Planner{": " + detail if detail else ""}.', True, 'running')


# ------------------------------------------------------------------ background worker for the panel
@dataclass(frozen=True)
class PlannerState:
    """The latest picture for the panel. Frozen (cannot change), so the Qt thread can read it safely.

    checked:    False until the first check has finished
    available:  True = the buttons can work here (inside WSL, installed, and Windows answers)
    running:    is Mission Planner open? True / False, or None = unknown
    busy:       '' or the action in progress: 'opening' / 'closing'
    label:      words for the status light ("running", "not running", "not installed" ...)
    message:    the full sentence (tooltip): what was found, or why the buttons cannot work
    """
    checked: bool = False
    available: bool = False
    running: bool | None = None
    busy: str = ''
    label: str = 'checking ...'
    message: str = 'Checking whether Mission Planner is installed and running ...'


class MissionPlannerMonitor:
    """Background thread for the demo panel: every `period_s` seconds it checks whether Mission Planner
    runs, and it carries out the OPEN / CLOSE presses one at a time. The Qt thread only calls
    request_open() / request_close() (they return at once) and snapshot() (the latest PlannerState).

    Args:
        report: callback(text, level) with each action's result sentence, level 'ok' or 'error'. It is
            called from this worker thread, so it must not touch Qt widgets (the panel stores the text
            and its 250 ms timer shows it).
        exe_override: path to MissionPlanner.exe ('' = $MISSION_PLANNER_EXE, else the usual folders).
        period_s: seconds between two checks.
    """

    def __init__(self, report: Callable[[str, str], None], exe_override: str = '',
                 period_s: float = 2.0):
        self.report = report
        self.exe_override = exe_override
        self.period_s = period_s
        self._lock = threading.Lock()
        self._state = PlannerState()
        self._jobs: queue.Queue = queue.Queue()      # 'opening' / 'closing'; None = stop
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='mission_planner', daemon=True)

    # ------------------------------------------------------------------ public (any thread)
    def start(self) -> None:
        """Start the worker thread (the first check begins at once)."""
        self._thread.start()

    def stop(self) -> None:
        """Stop the worker thread. An action already running finishes by itself (it is detached)."""
        self._stop.set()
        self._jobs.put(None)

    def snapshot(self) -> PlannerState:
        """The latest state (an immutable object: safe to keep and read from any thread)."""
        with self._lock:
            return self._state

    def request_open(self) -> bool:
        """Queue OPEN. False (and nothing happens) while another action is still running."""
        return self._request('opening')

    def request_close(self) -> bool:
        """Queue CLOSE. False (and nothing happens) while another action is still running."""
        return self._request('closing')

    # ------------------------------------------------------------------ worker thread
    def _request(self, what: str) -> bool:
        with self._lock:
            if self._state.busy:
                return False
            self._state = replace(self._state, busy=what)
        self._jobs.put(what)
        return True

    def _run(self) -> None:
        while not self._stop.is_set():
            self._refresh()
            try:
                job = self._jobs.get(timeout=self.period_s)   # waits for a press, or the next check
            except queue.Empty:
                continue
            if job is None:
                return
            self._do(job)

    def _refresh(self) -> None:
        """One check: inside WSL and installed? If so, is Mission Planner running?"""
        try:
            avail = availability(self.exe_override)
            found = is_running() if avail.ok else avail
        except Exception as exc:  # noqa: BLE001 - a surprise must not kill the worker thread
            found = avail = Result(False, f'Mission Planner check failed: {exc}', None, 'unknown')
        with self._lock:
            self._state = replace(self._state, checked=True, available=avail.ok and found.ok,
                                  running=found.running if found.ok else None,
                                  label=found.label, message=found.message)

    def _do(self, job: str) -> None:
        """Carry out one press (start() / stop() wait for Windows: up to about 10 s)."""
        try:
            res = start(self.exe_override) if job == 'opening' else stop()
        except Exception as exc:  # noqa: BLE001 - report it, never let the worker thread die
            res = Result(False, f'Mission Planner {job} failed: {exc}')
        with self._lock:
            # Take over what the action found at once, so the buttons do not flicker back for a moment.
            running = res.running if res.running is not None else self._state.running
            self._state = replace(self._state, busy='', running=running,
                                  label=res.label or self._state.label, message=res.message)
        try:
            self.report(res.message, 'ok' if res.ok else 'error')
        except Exception:  # noqa: BLE001 - a display problem must not stop the checks
            pass


# ------------------------------------------------------------------ command line
def main(argv: list[str] | None = None) -> int:
    """python3 -m sim_demo.mission_planner start|stop|status [--exe PATH]

    Prints one sentence. Exit code: 0 = done (status: running), 1 = failed or cannot tell,
    3 = status: not running (like `systemctl status`).
    """
    parser = argparse.ArgumentParser(
        prog='python3 -m sim_demo.mission_planner',
        description='Open, close or check Mission Planner (Windows) from Ubuntu inside WSL.')
    parser.add_argument('command', choices=('start', 'stop', 'status'),
                        help='start = open it, stop = close it, status = is it running?')
    parser.add_argument('--exe', default='',
                        help=f'path to MissionPlanner.exe (default: ${ENV_EXE}, else the usual folders)')
    args = parser.parse_args(argv)
    if args.command == 'status':
        res = is_running()
        code = 0 if res.running else (3 if res.ok else 1)
    else:
        res = start(args.exe) if args.command == 'start' else stop()
        code = 0 if res.ok else 1
    print(res.message)
    return code


if __name__ == '__main__':
    sys.exit(main())
