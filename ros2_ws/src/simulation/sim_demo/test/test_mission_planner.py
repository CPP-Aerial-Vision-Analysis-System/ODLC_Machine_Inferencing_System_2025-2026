"""
sim_demo/test/test_mission_planner.py -- PURPOSE: unit tests for sim_demo/mission_planner.py, the helper
behind the demo panel's OPEN / CLOSE MISSION PLANNER buttons and simulation/scripts/mission_planner.sh.

Windows is FAKED: FakeWindows answers like tasklist.exe / taskkill.exe and "starts" Mission Planner when
Popen is called, and a fake clock makes the 5-second waits instant. No test ever runs a real Windows
program: an autouse fixture makes any real subprocess call fail the test at once.
Covered: running / not running (and every tasklist output form), start ok, start when already running,
start that dies at once, start that Windows does not list yet, stop polite, stop forced after 5 s, stop
refused politely, stop when nothing runs, exe missing, MISSION_PLANNER_EXE (also as a Windows path), not
inside WSL, interop switched off, Windows too slow, tasklist errors, the System32 fallback, the panel's
background monitor (checks, one press at a time, reasons) and the command line with its exit codes.
"""
import errno
import os
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from sim_demo import mission_planner as mpl

WSL_KERNEL = ('Linux version 5.15.167.4-microsoft-standard-WSL2 (root@f9c826d3017f) (gcc (GCC) 11.2.0) '
              '#1 SMP Tue Nov 5 00:21:55 UTC 2024\n')
PLAIN_KERNEL = 'Linux version 6.8.0-45-generic (buildd@lcy02-amd64-075) #45-Ubuntu SMP\n'
MP_ROW = '"MissionPlanner.exe","46792","Console","1","468,956 K"\n'      # real tasklist /FO CSV /NH line
NO_TASKS = 'INFO: No tasks are running which match the specified criteria.\n'


class FakeProc:
    """What Popen returns: WSL's little Linux helper process for the started Windows program."""

    def __init__(self, code=None):
        self.code = code            # None = still running

    def poll(self):
        return self.code

    def wait(self, timeout=None):
        return 0 if self.code is None else self.code


class FakeWindows:
    """Pretends to be Windows behind WSL interop.

    running:  is MissionPlanner.exe in the task list right now?
    polite:   what a polite taskkill (no /F) does: 'closes' it, is 'ignored' (it hangs or asks a
              question) or is 'refused' ("can only be terminated forcefully")
    interop:  False = every Windows program fails with "Exec format error" (WSL interop off)
    start:    what Popen does: it 'shows' in the task list, 'dies' at once (exit code 1), or stays
              'silent' (runs, but Windows never lists it)
    """

    def __init__(self, running=False, polite='closes', interop=True, start='shows'):
        self.running, self.polite, self.interop, self.start = running, polite, interop, start
        self.calls = []                  # Windows commands run through subprocess.run, in order
        self.popen_calls = []            # (args, kwargs) given to subprocess.Popen
        self.clock = 0.0                 # fake seconds (advanced by the fake sleep)
        self.gate = None                 # threading.Event: Popen waits for it (to look at 'busy')
        self.entered = threading.Event()

    def run(self, args, **kwargs):
        self.calls.append([os.path.basename(args[0])] + list(args[1:]))
        if not self.interop:
            raise OSError(errno.ENOEXEC, 'Exec format error')
        name = os.path.basename(args[0]).lower()
        if name == 'tasklist.exe':
            return SimpleNamespace(returncode=0, stdout=MP_ROW if self.running else NO_TASKS, stderr='')
        if name == 'taskkill.exe':
            if not self.running:
                return SimpleNamespace(returncode=128, stdout='',
                                       stderr='ERROR: The process "MissionPlanner.exe" not found.\n')
            if '/F' not in args and self.polite == 'refused':
                return SimpleNamespace(returncode=1, stdout='', stderr=(
                    'ERROR: The process "MissionPlanner.exe" with PID 46792 could not be terminated.\n'
                    'Reason: This process can only be terminated forcefully (with /F option).\n'))
            if '/F' in args or self.polite == 'closes':
                self.running = False
            return SimpleNamespace(returncode=0, stderr='', stdout=(
                'SUCCESS: Sent termination signal to the process "MissionPlanner.exe" with PID 46792.\n'))
        raise AssertionError(f'unexpected Windows command {args}')

    def popen(self, args, **kwargs):
        self.popen_calls.append((list(args), kwargs))
        self.entered.set()
        if self.gate is not None:
            assert self.gate.wait(5.0)
        if not self.interop:
            raise OSError(errno.ENOEXEC, 'Exec format error')
        if self.start == 'dies':
            return FakeProc(code=1)
        if self.start == 'shows':
            self.running = True
        return FakeProc()

    def now(self):
        return self.clock

    def sleep(self, seconds):
        self.clock += seconds


@pytest.fixture(autouse=True)
def no_real_windows(monkeypatch):
    """Safety net: a real subprocess call from mission_planner.py fails the test immediately."""
    def refuse(*args, **kwargs):
        raise AssertionError(f'a test tried to run a real program: {args}')
    monkeypatch.setattr(mpl, '_run', refuse)
    monkeypatch.setattr(mpl, '_popen', refuse)


@pytest.fixture
def exe(tmp_path):
    """A stand-in MissionPlanner.exe (a tiny file in a temporary 'Mission Planner' folder)."""
    path = tmp_path / 'Program Files (x86)' / 'Mission Planner' / 'MissionPlanner.exe'
    path.parent.mkdir(parents=True)
    path.write_bytes(b'MZ')
    return str(path)


def _kernel(tmp_path, text):
    """Write a fake /proc/version and return its path."""
    path = tmp_path / 'proc_version'
    path.write_text(text)
    return str(path)


@pytest.fixture
def win(monkeypatch, tmp_path, exe):
    """A Windows + WSL computer with Mission Planner installed and NOT running (change win.* freely)."""
    fake = FakeWindows()
    monkeypatch.setattr(mpl, 'PROC_VERSION', _kernel(tmp_path, WSL_KERNEL))
    monkeypatch.setattr(mpl, 'DEFAULT_EXE_CANDIDATES', (exe,))
    monkeypatch.delenv(mpl.ENV_EXE, raising=False)
    monkeypatch.setattr(mpl, '_which', lambda name: f'/mnt/c/WINDOWS/system32/{name}')
    monkeypatch.setattr(mpl, '_run', fake.run)
    monkeypatch.setattr(mpl, '_popen', fake.popen)
    monkeypatch.setattr(mpl, '_sleep', fake.sleep)
    monkeypatch.setattr(mpl, '_now', fake.now)
    return fake


def _kills(win):
    """The taskkill.exe command lines that were run."""
    return [c for c in win.calls if c[0] == 'taskkill.exe']


# ------------------------------------------------------------------ is it running?
def test_is_running_yes(win):
    win.running = True
    res = mpl.is_running()
    assert (res.ok, res.running, res.label) == (True, True, 'running')
    assert '46792' in res.message
    # One read-only question, filtered on the program name, as CSV without a heading line.
    assert win.calls == [['tasklist.exe', '/FI', 'IMAGENAME eq MissionPlanner.exe', '/FO', 'CSV', '/NH']]


def test_is_running_no(win):
    res = mpl.is_running()
    assert (res.ok, res.running, res.label) == (True, False, 'not running')
    assert res.message == 'Mission Planner is not running.'


def test_parse_tasklist_formats():
    assert mpl.parse_tasklist(MP_ROW) == ['46792']
    assert mpl.parse_tasklist('MissionPlanner.exe          46792 Console        1    468,956 K\n') == ['46792']
    two = MP_ROW + '"MissionPlanner.exe","5120","Console","1","120,000 K"\n'
    assert mpl.parse_tasklist(two) == ['46792', '5120']                  # two copies
    assert mpl.parse_tasklist(NO_TASKS) == []
    # A French Windows says "nothing found" in French: still no row, still "not running".
    assert mpl.parse_tasklist('INFORMATION : Aucune tâche en service ne correspond aux critères.\n') == []
    assert mpl.parse_tasklist('"MissionPlannerHelper.exe","1","Console","1","1 K"\n') == []
    assert mpl.parse_tasklist('"missionplanner.exe","7","Console","1","1 K"\n') == ['7']   # any case
    assert mpl.parse_tasklist('') == [] and mpl.parse_tasklist(None) == []


# ------------------------------------------------------------------ start
def test_start_ok(win, exe, monkeypatch):
    monkeypatch.setenv('ASTRA_SIM_SESSION', 'sim-20261017-084825-123')
    res = mpl.start()
    assert (res.ok, res.running, res.label) == (True, True, 'running')
    assert mpl.CONNECT_HINT in res.message
    assert len(win.popen_calls) == 1
    args, kw = win.popen_calls[0]
    assert args == [exe]
    assert kw['start_new_session'] is True            # survives Ctrl+C and a closed terminal
    assert kw['stdin'] == kw['stdout'] == kw['stderr'] == subprocess.DEVNULL   # tied to nothing
    assert kw['cwd'] == os.path.dirname(exe)          # starts in its own folder
    assert 'ASTRA_SIM_SESSION' not in kw['env']       # so stop_sim.sh leaves it alone
    assert kw['env'].get('PATH') == os.environ.get('PATH')   # everything else is passed on


def test_start_when_already_running(win):
    win.running = True
    res = mpl.start()
    assert (res.ok, res.running) == (True, True)
    assert 'already open' in res.message
    assert win.popen_calls == []                      # never a second copy


def test_start_that_closes_at_once(win):
    win.start = 'dies'
    res = mpl.start()
    assert (res.ok, res.running) == (False, False)
    assert 'exit code 1' in res.message


def test_start_not_listed_yet(win):
    win.start = 'silent'
    res = mpl.start()
    assert (res.ok, res.running) == (True, None)
    assert 'does not list it yet' in res.message
    assert win.clock >= mpl.START_CONFIRM_S           # it waited the whole confirm time


# ------------------------------------------------------------------ stop
def test_stop_polite(win):
    win.running = True
    res = mpl.stop()
    assert (res.ok, res.running, res.message) == (True, False, 'Mission Planner closed.')
    assert _kills(win) == [['taskkill.exe', '/IM', 'MissionPlanner.exe']]     # no /F needed
    assert win.clock < mpl.POLITE_WAIT_S


def test_stop_forced_after_5_s(win):
    win.running, win.polite = True, 'ignored'
    res = mpl.stop()
    assert (res.ok, res.running) == (True, False)
    assert 'forced' in res.message and 'within 5 s' in res.message
    assert _kills(win) == [['taskkill.exe', '/IM', 'MissionPlanner.exe'],
                           ['taskkill.exe', '/F', '/IM', 'MissionPlanner.exe']]
    assert win.clock >= mpl.POLITE_WAIT_S             # it got its 5 s to close by itself first


def test_stop_polite_refused_forces_without_waiting(win):
    win.running, win.polite = True, 'refused'
    res = mpl.stop()
    assert (res.ok, res.running) == (True, False)
    assert 'did not accept a polite close' in res.message
    assert _kills(win)[-1] == ['taskkill.exe', '/F', '/IM', 'MissionPlanner.exe']
    assert win.clock < mpl.POLITE_WAIT_S              # nothing to wait for: the request never arrived


def test_stop_when_not_running(win):
    res = mpl.stop()
    assert (res.ok, res.running) == (True, False)
    assert 'not open' in res.message
    assert _kills(win) == []                          # nothing was closed


# ------------------------------------------------------------------ where is it?
def test_exe_missing(win, monkeypatch, tmp_path):
    missing = str(tmp_path / 'nowhere' / 'MissionPlanner.exe')
    monkeypatch.setattr(mpl, 'DEFAULT_EXE_CANDIDATES', (missing,))
    avail = mpl.availability()
    assert (avail.ok, avail.label) == (False, 'not installed')
    assert missing in avail.message and mpl.ENV_EXE in avail.message
    res = mpl.start()
    assert (res.ok, res.label) == (False, 'not installed')
    assert win.popen_calls == [] and win.calls == []  # nothing was run at all


def test_exe_from_environment(win, monkeypatch, exe):
    monkeypatch.setattr(mpl, 'DEFAULT_EXE_CANDIDATES', ('/nowhere/MissionPlanner.exe',))
    monkeypatch.setenv(mpl.ENV_EXE, exe)
    assert mpl.find_exe() == exe
    # A path given by the caller wins over the environment, and is reported when it is wrong.
    assert mpl.find_exe('/also/nowhere/MissionPlanner.exe') is None
    assert '/also/nowhere/MissionPlanner.exe' in mpl.availability('/also/nowhere/MissionPlanner.exe').message
    monkeypatch.setenv(mpl.ENV_EXE, r'C:\Tools\Mission Planner\MissionPlanner.exe')
    assert mpl.exe_candidates() == ['/mnt/c/Tools/Mission Planner/MissionPlanner.exe']


def test_to_wsl_path():
    assert (mpl.to_wsl_path(r'C:\Program Files (x86)\Mission Planner\MissionPlanner.exe')
            == '/mnt/c/Program Files (x86)/Mission Planner/MissionPlanner.exe')
    assert mpl.to_wsl_path('"D:/MP/MissionPlanner.exe"') == '/mnt/d/MP/MissionPlanner.exe'
    assert mpl.to_wsl_path('/mnt/c/x/MissionPlanner.exe') == '/mnt/c/x/MissionPlanner.exe'


def test_tool_lookup_falls_back_to_system32(monkeypatch, tmp_path):
    monkeypatch.setattr(mpl, '_which', lambda name: None)    # Windows folders are not on the PATH
    monkeypatch.setattr(mpl, 'SYSTEM32_DIR', str(tmp_path))
    assert mpl.find_windows_tool('tasklist.exe') is None
    (tmp_path / 'tasklist.exe').write_bytes(b'MZ')
    assert mpl.find_windows_tool('tasklist.exe') == str(tmp_path / 'tasklist.exe')


def test_tasklist_not_found(win, monkeypatch, tmp_path):
    monkeypatch.setattr(mpl, '_which', lambda name: None)
    monkeypatch.setattr(mpl, 'SYSTEM32_DIR', str(tmp_path / 'empty'))
    res = mpl.is_running()
    assert (res.ok, res.running) == (False, None)
    assert 'tasklist.exe was not found' in res.message


# ------------------------------------------------------------------ cannot ask Windows
def test_not_inside_wsl(win, monkeypatch, tmp_path):
    monkeypatch.setattr(mpl, 'PROC_VERSION', _kernel(tmp_path, PLAIN_KERNEL))
    assert mpl.is_wsl() is False
    for res in (mpl.availability(), mpl.is_running(), mpl.start(), mpl.stop()):
        assert (res.ok, res.running, res.label) == (False, None, 'not on Windows')
        assert 'WSL' in res.message
    assert win.calls == [] and win.popen_calls == []  # never even tried


def test_is_wsl_without_proc_version(monkeypatch, tmp_path):
    monkeypatch.setattr(mpl, 'PROC_VERSION', str(tmp_path / 'missing'))
    assert mpl.is_wsl() is False


def test_interop_switched_off(win):
    win.interop = False
    res = mpl.is_running()
    assert (res.ok, res.running, res.label) == (False, None, 'unknown')
    assert 'interop' in res.message and 'Exec format error' in res.message
    for res in (mpl.start(), mpl.stop()):
        assert not res.ok and 'interop' in res.message
    assert win.popen_calls == []


def test_windows_too_slow(win, monkeypatch):
    def slow(args, **kwargs):
        raise subprocess.TimeoutExpired(args, kwargs.get('timeout'))
    monkeypatch.setattr(mpl, '_run', slow)
    res = mpl.is_running()
    assert (res.ok, res.running) == (False, None)
    assert 'did not answer' in res.message


def test_tasklist_error_exit(win, monkeypatch):
    monkeypatch.setattr(mpl, '_run', lambda args, **kw: SimpleNamespace(
        returncode=1, stdout='', stderr='ERROR: Invalid argument/option.\n'))
    res = mpl.is_running()
    assert (res.ok, res.running) == (False, None)
    assert 'Invalid argument' in res.message


# ------------------------------------------------------------------ the panel's background worker
def _wait_for(condition, timeout=5.0):
    """Real-time wait (the worker thread runs for real; only Windows and its clock are fake)."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.01)
    return False


def test_monitor_checks_and_carries_out_presses(win):
    reports = []
    mon = mpl.MissionPlannerMonitor(lambda text, level: reports.append((level, text)), period_s=0.05)
    assert (mon.snapshot().checked, mon.snapshot().label) == (False, 'checking ...')
    mon.start()
    try:
        assert _wait_for(lambda: mon.snapshot().checked)
        st = mon.snapshot()
        assert (st.available, st.running, st.label) == (True, False, 'not running')
        win.gate = threading.Event()                  # hold the start until 'busy' was checked
        assert mon.request_open() is True
        assert win.entered.wait(5.0)
        assert mon.snapshot().busy == 'opening'
        assert mon.request_close() is False           # one action at a time: the press is ignored
        win.gate.set()
        assert _wait_for(lambda: len(reports) == 1)
        assert reports[0][0] == 'ok' and 'opening' in reports[0][1]
        assert _wait_for(lambda: not mon.snapshot().busy and mon.snapshot().running is True)
        assert mon.request_close() is True
        assert _wait_for(lambda: len(reports) == 2)
        assert reports[1] == ('ok', 'Mission Planner closed.')
        assert _wait_for(lambda: mon.snapshot().running is False)
    finally:
        mon.stop()
        mon._thread.join(timeout=5.0)
    assert not mon._thread.is_alive()                 # stop() really ends the thread


def test_monitor_says_why_it_cannot_work(win, monkeypatch, tmp_path):
    monkeypatch.setattr(mpl, 'PROC_VERSION', _kernel(tmp_path, PLAIN_KERNEL))
    mon = mpl.MissionPlannerMonitor(lambda *args: None, period_s=0.05)
    mon.start()
    try:
        assert _wait_for(lambda: mon.snapshot().checked)
        st = mon.snapshot()
        assert (st.available, st.running, st.label) == (False, None, 'not on Windows')
        assert 'WSL' in st.message
    finally:
        mon.stop()
        mon._thread.join(timeout=5.0)
    assert win.calls == []                            # not inside WSL: Windows is never asked


# ------------------------------------------------------------------ command line
def test_command_line(win, capsys):
    assert mpl.main(['status']) == 3                  # 3 = not running
    assert 'not running' in capsys.readouterr().out
    assert mpl.main(['start']) == 0
    assert 'opening' in capsys.readouterr().out
    assert mpl.main(['status']) == 0                  # 0 = running
    assert mpl.main(['stop']) == 0
    assert 'closed' in capsys.readouterr().out
    win.interop = False
    assert mpl.main(['status']) == 1                  # 1 = cannot tell
    with pytest.raises(SystemExit):
        mpl.main(['explode'])
