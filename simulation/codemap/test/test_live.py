"""test_live.py -- PURPOSE: tests of the LIVE part of the ASTRA code map (simulation/codemap).

What is tested, in plain words:
  * the Python tracer (trace/sitecustomize.py + trace/astra_trace.py):
      - it does NOTHING unless ASTRA_TRACE=1 (the PYTHONPATH entry alone changes nothing);
      - it still runs the system's own sitecustomize that it hides (Ubuntu's apport hook), like Python would;
      - when on, a small program in a pretend repo sends UDP messages with the right module, file, first line
        (decorator line for decorated functions), function name and number of calls, also from threads; code
        outside the repo (json, threading ...) and lambdas/comprehensions are never reported;
      - it sends at most one message every 100 ms (rate limit), loses no counts, and never raises or hangs
        when ASTRA_TRACE_ADDR is nonsense;
      - it takes at most 1 frame of Python's recursion limit, and its profiler catches only RecursionError and
        MemoryError, so it never swallows an exception that belongs to the program (checked on the profiler
        itself; real signal timing cannot be controlled in a test);
      - programs named in ASTRA_TRACE_SKIP (default: MAVProxy) get no profiler and send nothing;
  * the bash tracer (trace/astra_trace.sh, switched on by simulation/scripts/_common.sh):
      - each function entry is reported once, only for files of the repo, under set -euo pipefail, keeping $?,
        PIPESTATUS and the script's exit status, and nothing at all without ASTRA_TRACE=1;
      - at most one flush every 100 ms per process (the contract), with the rest sent at exit, right before
        `exec`, or at the first command once the 100 ms are over; subshells report their own calls exactly once;
  * server.py also lists the Linux-side copy's recordings (~/.astra_sim/codemap/recordings) by default;
  * server.py (started on free ports with a small fake codemap.json):
      - it serves /, /codemap.json, /health, /recordings, /recordings/<file>, /events (Server-Sent Events);
      - it maps Python messages to fn ids by module name or by the end of the file path, bash messages by
        script path + function name, lists what it cannot place at /health, records every event as JSON lines,
        accepts at most 8 live pages, re-reads codemap.json when it changes;
      - the command-line rules that turn running programs into mod:/ext: ids ("procs").
  * one end-to-end check: a traced program -> the real server -> fn ids on the event stream.

Run (Windows or Linux Python 3.10+, no ROS needed; from the repo root):
    python -m pytest -q -p no:cacheprovider simulation/codemap/test/test_live.py
The bash tests need a bash with /dev/udp (Linux bash, or Git for Windows' bash); they are skipped otherwise.
"""

from __future__ import annotations

import http.client
import importlib.util
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

CODEMAP_DIR = Path(__file__).resolve().parents[1]
TRACE_DIR = CODEMAP_DIR / 'trace'
SERVER_PY = CODEMAP_DIR / 'server.py'
REPO = CODEMAP_DIR.parents[1]
COMMON_SH = REPO / 'simulation' / 'scripts' / '_common.sh'


# =============================================================================================================
#  helpers
# =============================================================================================================
def clean_env(**extra: str) -> dict:
    """The test process's environment without any tracer settings, plus `extra`."""
    env = dict(os.environ)
    for key in ('ASTRA_TRACE', 'ASTRA_TRACE_ADDR', 'ASTRA_TRACE_ROOTS', 'ASTRA_TRACE_SKIP', 'PYTHONPATH',
                'PYTHONSTARTUP'):
        env.pop(key, None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONIOENCODING'] = 'utf-8'
    env.update(extra)
    return env


def free_port(kind: int = socket.SOCK_STREAM) -> int:
    s = socket.socket(socket.AF_INET, kind)
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


class UdpCollector:
    """A UDP socket on a free port that keeps every JSON datagram it receives (in a thread)."""

    def __init__(self) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.port = self.sock.getsockname()[1]
        self.sock.settimeout(0.05)
        self.messages: list[dict] = []
        self.raw: list[bytes] = []
        self.last = time.monotonic()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.sock.recvfrom(70000)[0]
            except socket.timeout:
                continue
            except OSError:
                continue
            self.raw.append(data)
            self.messages.append(json.loads(data.decode('utf-8')))
            self.last = time.monotonic()

    def wait_quiet(self, quiet: float = 0.6, limit: float = 8.0) -> None:
        """Wait until nothing new arrived for `quiet` seconds (at most `limit` seconds)."""
        end = time.monotonic() + limit
        while time.monotonic() < end:
            if time.monotonic() - self.last >= quiet:
                return
            time.sleep(0.05)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(2)
        self.sock.close()


@pytest.fixture
def collector():
    c = UdpCollector()
    yield c
    c.close()


def line_of(text: str, needle: str) -> int:
    """1-based number of the first line of `text` that contains `needle`."""
    for number, line in enumerate(text.splitlines(), start=1):
        if needle in line:
            return number
    raise AssertionError('%r not found' % needle)


TINYMOD = '''\
import json
import threading
from pkg_tiny import helpers


def hello(n):
    return n + 1


class Greeter:
    def wave(self):
        return 'hi'


def deco(f):
    return f


@deco
def decorated():
    return [i * 2 for i in range(3)]


def main():
    for i in range(5):
        hello(i)
    g = Greeter()
    for _ in range(3):
        g.wave()
    decorated()
    helpers.twice()
    helpers.twice()
    worker = threading.Thread(target=lambda: [hello(i) for i in range(4)])
    worker.start()
    worker.join()
    json.dumps({'a': [1, 2, 3]})
    print('done')


main()
'''

HELPERS = '''\
def twice():
    return 2
'''


@pytest.fixture
def tiny_repo(tmp_path: Path) -> Path:
    """A pretend repo with one program (tinymod.py) and one package module (pkg_tiny/helpers.py)."""
    repo = tmp_path / 'repo'
    (repo / 'pkg_tiny').mkdir(parents=True)
    (repo / 'tinymod.py').write_text(TINYMOD, encoding='utf-8')
    (repo / 'pkg_tiny' / '__init__.py').write_text('', encoding='utf-8')
    (repo / 'pkg_tiny' / 'helpers.py').write_text(HELPERS, encoding='utf-8')
    return repo


def run_traced(script: Path, port: int | None, roots: Path | None, trace: str | None = '1',
               addr: str | None = None, extra_path: str | None = None, timeout: float = 60,
               extra_env: dict | None = None):
    """Run `script` with the trace folder on PYTHONPATH; returns (completed process, pid)."""
    env = clean_env(PYTHONPATH=str(TRACE_DIR) + (os.pathsep + extra_path if extra_path else ''))
    env.update(extra_env or {})
    if trace is not None:
        env['ASTRA_TRACE'] = trace
    if roots is not None:
        env['ASTRA_TRACE_ROOTS'] = str(roots)
    if addr is not None:
        env['ASTRA_TRACE_ADDR'] = addr
    elif port is not None:
        env['ASTRA_TRACE_ADDR'] = '127.0.0.1:%d' % port
    proc = subprocess.Popen([sys.executable, str(script)], env=env, cwd=str(script.parent),
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, encoding='utf-8')
    out, err = proc.communicate(timeout=timeout)
    return subprocess.CompletedProcess(proc.args, proc.returncode, out, err), proc.pid


def totals(messages: list[dict]) -> dict:
    """Python messages -> {(module, file name, first line, function name): calls}."""
    out: dict = {}
    for m in messages:
        for module, filename, line, name, count in m['calls']:
            key = (module, os.path.basename(filename), line, name)
            out[key] = out.get(key, 0) + count
    return out


# =============================================================================================================
#  Python tracer
# =============================================================================================================
PROBE = '''\
import json, sys
import sitecustomize
print(json.dumps({"profile": repr(sys.getprofile()), "loaded": "astra_trace" in sys.modules,
                  "site": getattr(sitecustomize, "__file__", None)}))
'''


@pytest.mark.parametrize('value', [None, '0', '', 'yes', 'true', '2'])
def test_python_tracer_is_off_unless_astra_trace_is_1(tmp_path, tiny_repo, collector, value):
    probe = tiny_repo / 'probe.py'
    probe.write_text(PROBE, encoding='utf-8')
    result, _pid = run_traced(probe, collector.port, tiny_repo, trace=value)
    assert result.returncode == 0, result.stderr
    info = json.loads(result.stdout)
    assert info['profile'] == 'None'
    assert info['loaded'] is False
    assert Path(info['site']).resolve() == (TRACE_DIR / 'sitecustomize.py').resolve()   # our file did load
    result, _pid = run_traced(tiny_repo / 'tinymod.py', collector.port, tiny_repo, trace=value)
    assert result.returncode == 0 and result.stdout.strip() == 'done', result.stderr
    assert result.stderr == ''
    collector.wait_quiet(0.4, 2)
    assert collector.messages == []


@pytest.mark.parametrize('trace', [None, '1'])
def test_hidden_system_sitecustomize_still_runs(tmp_path, tiny_repo, collector, trace):
    system = tmp_path / 'system_site'
    system.mkdir()
    (system / 'sitecustomize.py').write_text("import os\nos.environ['ASTRA_TEST_CHAINED'] = 'yes'\n",
                                             encoding='utf-8')
    probe = tiny_repo / 'probe_chain.py'
    probe.write_text("import os\nprint(os.environ.get('ASTRA_TEST_CHAINED'))\n", encoding='utf-8')
    result, _pid = run_traced(probe, collector.port, tiny_repo, trace=trace, extra_path=str(system))
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'yes'
    assert result.stderr == ''


def test_a_failing_hidden_sitecustomize_gives_pythons_own_message(tmp_path, tiny_repo, collector):
    system = tmp_path / 'system_site'
    system.mkdir()
    (system / 'sitecustomize.py').write_text("raise RuntimeError('broken system file')\n", encoding='utf-8')
    probe = tiny_repo / 'probe_ok.py'
    probe.write_text("print('still runs')\n", encoding='utf-8')
    result, _pid = run_traced(probe, collector.port, tiny_repo, trace='1', extra_path=str(system))
    assert result.returncode == 0
    assert result.stdout.strip() == 'still runs'
    assert 'Error in sitecustomize; set PYTHONVERBOSE for traceback:' in result.stderr
    assert 'RuntimeError: broken system file' in result.stderr


def test_python_tracer_reports_our_calls_with_module_file_line_name_count(tiny_repo, collector):
    result, pid = run_traced(tiny_repo / 'tinymod.py', collector.port, tiny_repo)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'done'
    assert result.stderr == ''
    collector.wait_quiet()
    assert collector.messages, 'no datagram arrived'
    for m in collector.messages:
        assert m['pid'] == pid
        assert m['proc'] == 'tinymod.py'
        assert set(m) == {'pid', 'proc', 'calls'}
        for entry in m['calls']:
            assert len(entry) == 5
            module, filename, line, name, count = entry
            assert isinstance(module, str) and isinstance(filename, str) and isinstance(name, str)
            assert isinstance(line, int) and isinstance(count, int) and count > 0
            # only files of the pretend repo, reported with their real path
            assert os.path.normcase(filename).startswith(os.path.normcase(os.path.realpath(tiny_repo)))
            assert not name.startswith('<'), entry       # no <module>, <lambda>, <listcomp>
    got = totals(collector.messages)
    assert got[('__main__', 'tinymod.py', line_of(TINYMOD, 'def hello'), 'hello')] == 9   # 5 + 4 in a thread
    assert got[('__main__', 'tinymod.py', line_of(TINYMOD, 'def wave'), 'wave')] == 3
    assert got[('__main__', 'tinymod.py', line_of(TINYMOD, 'def main'), 'main')] == 1
    # a decorated function is reported at its first decorator's line (Python's co_firstlineno)
    assert got[('__main__', 'tinymod.py', line_of(TINYMOD, '@deco'), 'decorated')] == 1
    assert got[('__main__', 'tinymod.py', line_of(TINYMOD, 'def deco'), 'deco')] == 1
    # a module imported from a package carries its dotted name
    assert got[('pkg_tiny.helpers', 'helpers.py', 1, 'twice')] == 2


def test_python_tracer_skips_code_outside_the_roots(tiny_repo, tmp_path, collector):
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'lib_outside.py').write_text('def far():\n    return 1\n', encoding='utf-8')
    script = tiny_repo / 'uses_outside.py'
    script.write_text('import sys\nsys.path.insert(0, %r)\nimport lib_outside\n\ndef near():\n    return 2\n\n'
                      'for _ in range(3):\n    lib_outside.far()\n    near()\nprint("done")\n' % str(outside),
                      encoding='utf-8')
    result, _pid = run_traced(script, collector.port, tiny_repo)
    assert result.returncode == 0 and result.stdout.strip() == 'done', result.stderr
    collector.wait_quiet()
    got = totals(collector.messages)
    assert got == {('__main__', 'uses_outside.py', 5, 'near'): 3}


RATE = '''\
import time


def tick():
    return 1


calls = 0
end = time.monotonic() + 1.5
while time.monotonic() < end:
    tick()
    calls += 1
    time.sleep(0.001)
print(calls)
'''


def test_python_tracer_is_rate_limited_and_loses_no_counts(tiny_repo, collector):
    script = tiny_repo / 'rate.py'
    script.write_text(RATE, encoding='utf-8')
    result, _pid = run_traced(script, collector.port, tiny_repo)
    assert result.returncode == 0, result.stderr
    calls = int(result.stdout.strip())
    collector.wait_quiet()
    n = len(collector.messages)
    assert 5 <= n <= 1.5 / 0.1 + 4, n          # it flushes while running (every 100 ms), never more often
    assert totals(collector.messages)[('__main__', 'rate.py', 4, 'tick')] == calls


@pytest.mark.parametrize('addr', ['nonsense', '127.0.0.1:notaport', '127.0.0.1:0', '127.0.0.1:70000',
                                  '999.999.999.999:8091', 'name.invalid:8091', ':8091', '[::1', '[::1]:x'])
def test_python_tracer_never_raises_or_hangs_with_a_bad_address(tiny_repo, addr):
    start = time.monotonic()
    result, _pid = run_traced(tiny_repo / 'tinymod.py', None, tiny_repo, addr=addr, timeout=30)
    assert result.returncode == 0
    assert result.stdout.strip() == 'done'
    assert result.stderr == ''
    assert time.monotonic() - start < 20


FORK = '''\
import os
import time


def hello():
    return 1


for _ in range(3):
    hello()
pid = os.fork()
if pid == 0:                       # the child: reports only its own 2 calls, with its own pid
    hello()
    hello()
    time.sleep(0.3)
    raise SystemExit(0)
os.waitpid(pid, 0)
hello()
print(pid)
'''


@pytest.mark.skipif(not hasattr(os, 'fork'), reason='os.fork() exists on Linux/macOS only')
def test_python_tracer_after_fork_each_process_reports_its_own_calls(tiny_repo, collector):
    script = tiny_repo / 'forky.py'
    script.write_text(FORK, encoding='utf-8')
    result, parent = run_traced(script, collector.port, tiny_repo)
    assert result.returncode == 0, result.stderr
    child = int(result.stdout.strip())
    collector.wait_quiet()
    per_pid: dict = {}
    for m in collector.messages:
        for _mod, _file, _line, name, count in m['calls']:
            if name == 'hello':
                per_pid[m['pid']] = per_pid.get(m['pid'], 0) + count
    assert per_pid == {parent: 4, child: 2}


SKIP_PROBE = '''\
import json, sys


def hello():
    return 1


hello()
import astra_trace
print(json.dumps({"profile": sys.getprofile() is not None, "skipped": astra_trace.stats()["skipped_program"]}))
'''


@pytest.mark.parametrize('name, skip, traced', [
    ('mavproxy.py', None, False),               # the default list: MAVProxy is never traced
    ('MAVProxy.py', None, False),               # names are compared without case
    ('mavproxy.py', '', True),                  # ASTRA_TRACE_SKIP="" -> every program is traced
    ('tool_a.py', 'tool_a', False),             # "tool_a" matches tool_a.py (".py" is optional)
    ('tool_b.py', ' x.py , tool_b.py ', False),  # spaces around the names do not matter
    ('main_ok.py', None, True),                 # everything else is traced as usual
])
def test_python_tracer_leaves_skipped_programs_alone(tiny_repo, collector, name, skip, traced):
    script = tiny_repo / name
    script.write_text(SKIP_PROBE, encoding='utf-8')
    extra = {} if skip is None else {'ASTRA_TRACE_SKIP': skip}
    result, _pid = run_traced(script, collector.port, tiny_repo, extra_env=extra)
    assert result.returncode == 0 and result.stderr == '', result.stderr
    info = json.loads(result.stdout)
    assert info['profile'] is traced
    assert info['skipped'] == ('' if traced else name)
    collector.wait_quiet(0.4, 3)
    assert bool(collector.messages) is traced          # a skipped program sends nothing at all


def test_python_tracer_skips_by_module_name_for_python_dash_m(tiny_repo, collector):
    if not os.path.exists('/proc/self/cmdline'):
        pytest.skip('"python -m" names are read from /proc/self/cmdline (Linux only)')
    (tiny_repo / 'pkg_tiny' / 'busytool.py').write_text(SKIP_PROBE, encoding='utf-8')
    env = clean_env(PYTHONPATH=str(TRACE_DIR), ASTRA_TRACE='1', ASTRA_TRACE_ROOTS=str(tiny_repo),
                    ASTRA_TRACE_ADDR='127.0.0.1:%d' % collector.port, ASTRA_TRACE_SKIP='pkg_tiny.busytool')
    result = subprocess.run([sys.executable, '-m', 'pkg_tiny.busytool'], env=env, cwd=str(tiny_repo),
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0 and result.stderr == '', result.stderr
    assert json.loads(result.stdout) == {'profile': False, 'skipped': 'pkg_tiny.busytool'}


def test_a_loose_copy_of_the_python_tracer_does_not_count_every_file(tmp_path, collector):
    # The default roots include "the repo this tracer lives in" = three folders up from trace/. A copy of the
    # tracer that does NOT sit at <repo>/simulation/codemap/trace must not turn that into a root (here it would
    # be a folder that holds the whole pytest temp area, so the program below would count as team code).
    loose = tmp_path / 'loose_trace'
    loose.mkdir()
    for name in ('sitecustomize.py', 'astra_trace.py'):
        shutil.copy(TRACE_DIR / name, loose / name)
    elsewhere = tmp_path / 'elsewhere'
    elsewhere.mkdir()
    prog = elsewhere / 'prog.py'
    prog.write_text('def f():\n    return 1\n\n\nfor _ in range(3):\n    f()\nprint("done")\n', encoding='utf-8')
    env = clean_env(PYTHONPATH=str(loose), ASTRA_TRACE='1', ASTRA_TRACE_ADDR='127.0.0.1:%d' % collector.port)
    result = subprocess.run([sys.executable, str(prog)], env=env, cwd=str(elsewhere), capture_output=True,
                            text=True, timeout=60)
    assert result.returncode == 0 and result.stdout.strip() == 'done', result.stderr
    collector.wait_quiet(0.4, 3)
    assert collector.messages == []
    # ... while the same program inside a folder named like the repo still counts (the "segment" rule)
    named = tmp_path / 'ODLC_Machine_Inferencing_System_2025-2026' / 'tools'
    named.mkdir(parents=True)
    shutil.copy(prog, named / 'prog.py')
    result = subprocess.run([sys.executable, str(named / 'prog.py')], env=env, cwd=str(named),
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0 and result.stdout.strip() == 'done', result.stderr
    collector.wait_quiet()
    assert totals(collector.messages) == {('__main__', 'prog.py', 1, 'f'): 3}


def test_python_tracer_never_blocks_when_nobody_listens(tiny_repo):
    port = free_port(socket.SOCK_DGRAM)               # nothing listens there
    result, _pid = run_traced(tiny_repo / 'rate.py' if (tiny_repo / 'rate.py').exists() else
                              tiny_repo / 'tinymod.py', port, tiny_repo)
    assert result.returncode == 0, result.stderr
    assert result.stderr == ''


RECURSE = '''\
import sys
sys.setrecursionlimit(200)
depth = [0]


def probe(n):
    depth[0] = n
    probe(n + 1)


try:
    probe(0)
except RecursionError:
    pass
print(depth[0])
'''


def test_python_tracer_takes_at_most_one_frame_of_the_recursion_limit(tiny_repo, collector):
    # The profiler runs as one more Python frame on top of the program's frames (that is how sys.setprofile
    # works), so a traced program may reach 1 level less before RecursionError, never 2. (Before the whole
    # profiler body was inside its try, Python 3.10 lost 2 levels and the error came from inside the profiler.)
    script = tiny_repo / 'recurse.py'
    script.write_text(RECURSE, encoding='utf-8')
    plain, _pid = run_traced(script, collector.port, tiny_repo, trace=None)
    traced, _pid = run_traced(script, collector.port, tiny_repo)
    assert plain.returncode == 0 and traced.returncode == 0, traced.stderr
    assert traced.stderr == ''
    assert int(plain.stdout) - 1 <= int(traced.stdout) <= int(plain.stdout), (plain.stdout, traced.stdout)


def _load_astra_trace():
    """A private copy of trace/astra_trace.py in this test process (importing it installs nothing)."""
    spec = importlib.util.spec_from_file_location('astra_trace_under_test', TRACE_DIR / 'astra_trace.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_python_profiler_catches_only_its_own_errors():
    # The profiler may swallow only the two errors it can cause itself (RecursionError, MemoryError), and that from
    # its very first line on. Anything else, for example an exception that the program's own signal handler raises
    # while the profiler runs, must pass through to the program. Checked directly, without real signals (their
    # timing cannot be controlled in a test): a stand-in for the once-per-function look that raises, and an event
    # whose comparison with 'call' (the profiler's first line) raises.
    tracer = _load_astra_trace()
    frame = sys._getframe()

    class ProgramsOwn(Exception):
        pass

    class Event:
        def __init__(self, error):
            self.error = error

        def __ne__(self, other):
            raise self.error

    def raising(error):
        def first_call(_frame, _code):
            raise error
        return first_call

    for own in (RecursionError('too deep'), MemoryError()):
        assert tracer._make_profiler(set(), {}, raising(own))(frame, 'call', None) is None
        assert tracer._make_profiler(set(), {}, None)(frame, Event(own), None) is None
    with pytest.raises(ProgramsOwn):
        tracer._make_profiler(set(), {}, raising(ProgramsOwn('mine')))(frame, 'call', None)
    with pytest.raises(ProgramsOwn):
        tracer._make_profiler(set(), {}, None)(frame, Event(ProgramsOwn('mine')), None)
    with pytest.raises(KeyboardInterrupt):                    # Ctrl+C is never swallowed either
        tracer._make_profiler(set(), {}, raising(KeyboardInterrupt()))(frame, 'call', None)
    # and the normal work: count a function of ours (the first call goes through the first look), skip 'return'
    counts: dict = {}
    profiler = tracer._make_profiler({'elsewhere.py'}, counts, lambda _f, code: counts.__setitem__(code, 1))
    profiler(frame, 'call', None)
    profiler(frame, 'call', None)
    profiler(frame, 'return', None)
    assert counts == {frame.f_code: 2}


# =============================================================================================================
#  bash tracer
# =============================================================================================================
def find_bash() -> str | None:
    """A bash that is not Windows' WSL launcher (C:\\Windows\\System32\\bash.exe)."""
    candidates = []
    if os.name == 'nt':
        candidates += [r'C:\Program Files\Git\bin\bash.exe', r'C:\Program Files\Git\usr\bin\bash.exe']
    found = shutil.which('bash')
    if found:
        candidates.append(found)
    for c in candidates:
        low = c.lower()
        if os.path.isfile(c) and 'system32' not in low and 'windowsapps' not in low:
            return c
    return None


BASH = find_bash()


def bash_has_dev_udp() -> bool:
    if not BASH:
        return False
    c = UdpCollector()
    try:
        subprocess.run([BASH, '-c', "printf x > /dev/udp/127.0.0.1/%d" % c.port], timeout=20,
                       capture_output=True)
        c.wait_quiet(0.3, 2)
        return bool(c.raw)
    except (OSError, subprocess.SubprocessError):
        return False
    finally:
        c.close()


needs_bash = pytest.mark.skipif(not bash_has_dev_udp(), reason='no bash with /dev/udp support here')


def make_bash_repo(tmp_path: Path, script_text: str) -> Path:
    """tmp/repo/simulation/{codemap/trace/astra_trace.sh, scripts/t.sh} (+ tmp/outside/lib.sh)."""
    repo = tmp_path / 'repo'
    (repo / 'simulation' / 'codemap' / 'trace').mkdir(parents=True)
    (repo / 'simulation' / 'scripts').mkdir(parents=True)
    shutil.copy(TRACE_DIR / 'astra_trace.sh', repo / 'simulation' / 'codemap' / 'trace' / 'astra_trace.sh')
    (tmp_path / 'outside').mkdir(exist_ok=True)
    (tmp_path / 'outside' / 'lib.sh').write_bytes(b'other_fn() { :; }\n')
    script = repo / 'simulation' / 'scripts' / 't.sh'
    script.write_bytes(script_text.encode('utf-8'))   # LF line endings, like .gitattributes wants
    return script


def run_bash(script: Path, port: int, trace: str | None = '1', timeout: float = 60):
    env = clean_env(ASTRA_TRACE_ADDR='127.0.0.1:%d' % port)
    env.pop('ASTRA_TRACE_ROOTS', None)
    if trace is not None:
        env['ASTRA_TRACE'] = trace
    # Run from the script's folder with a relative name: Git Bash on Windows then sees MSYS paths everywhere.
    return subprocess.run([BASH, script.name], cwd=str(script.parent), env=env, capture_output=True,
                          timeout=timeout)


def bash_totals(messages: list[dict]) -> dict:
    out: dict = {}
    for m in messages:
        for name, count in m['calls']:
            key = (m['sh'].replace('\\', '/').rsplit('/', 1)[-1], name)
            out[key] = out.get(key, 0) + count
    return out


STRICT = r'''#!/bin/bash
set -euo pipefail
source ../codemap/trace/astra_trace.sh
source ../../../outside/lib.sh
inner() { local x=1; [ "$x" = 1 ]; }
outer() { inner; inner; }
outer
outer
other_fn
v=$(inner; echo sub)
st=0; false || st=$?
echo "after-false=$st"
f3() { return 3; }
r=0; f3 || r=$?
echo "f3=$r"
set +o pipefail
true | false | true
echo "pipestatus=${PIPESTATUS[*]}"
echo "v=$v"
exit 7
'''


@needs_bash
def test_bash_tracer_reports_each_function_entry_once(tmp_path, collector):
    script = make_bash_repo(tmp_path, STRICT)
    result = run_bash(script, collector.port)
    out = result.stdout.decode('utf-8', 'replace').split()
    assert result.returncode == 7, result.stderr      # the script's own exit status is kept
    assert result.stderr == b'', result.stderr
    assert out == ['after-false=1', 'f3=3', 'pipestatus=0', '1', '0', 'v=sub']
    collector.wait_quiet()
    got = bash_totals(collector.messages)
    assert got == {('t.sh', 'outer'): 2, ('t.sh', 'inner'): 5, ('t.sh', 'f3'): 1}   # inner: 2x2 + 1 in $( )
    for m in collector.messages:
        assert set(m) == {'pid', 'sh', 'calls'}
        assert isinstance(m['pid'], int)
        assert m['sh'].endswith('/simulation/scripts/t.sh')


LOOP = r'''#!/bin/bash
source ../codemap/trace/astra_trace.sh
g() { :; }
for ((i = 0; i < 400; i++)); do g; done
echo done
'''


@needs_bash
def test_bash_tracer_is_rate_limited_and_sends_the_rest_at_exit(tmp_path, collector):
    script = make_bash_repo(tmp_path, LOOP)
    start = time.monotonic()
    result = run_bash(script, collector.port)
    elapsed = time.monotonic() - start
    assert result.returncode == 0 and result.stdout.strip() == b'done', result.stderr
    collector.wait_quiet()
    assert bash_totals(collector.messages) == {('t.sh', 'g'): 400}
    # at most one flush every 100 ms (the first one at once) plus the last one at exit
    assert len(collector.messages) <= 1 + 10 * elapsed + 1, (len(collector.messages), elapsed)


BUSY = r'''#!/bin/bash
source ../codemap/trace/astra_trace.sh
g() { :; }
n=0
end=$(( ${EPOCHREALTIME//[!0-9]/} + 1200000 ))
while [ "${EPOCHREALTIME//[!0-9]/}" -lt "$end" ]; do g; n=$((n + 1)); done
echo "$n"
'''


@needs_bash
def test_bash_tracer_sends_at_most_once_per_100_ms_while_busy(tmp_path, collector):
    # 1.2 s of nothing but function calls: the counts go out about every 100 ms while it runs (not only at the
    # end), never more often, and not one call is lost.
    script = make_bash_repo(tmp_path, BUSY)
    result = run_bash(script, collector.port)
    assert result.returncode == 0 and result.stderr == b'', result.stderr
    calls = int(result.stdout.strip())
    collector.wait_quiet()
    assert bash_totals(collector.messages) == {('t.sh', 'g'): calls}
    assert len({m['pid'] for m in collector.messages}) == 1
    assert 5 <= len(collector.messages) <= 1 + 12 + 1, len(collector.messages)   # first + 1.2 s / 0.1 s + exit


SUBSHELLS = r'''#!/bin/bash
source ../codemap/trace/astra_trace.sh
f() { :; }
g() { :; }
h() { :; }
k() { :; }
f; f; f                         # the 1st goes at once, the 2nd and 3rd wait (less than 100 ms later)
x=$(g; g; echo sub)             # a subshell: its own counts, never the parent's waiting f's
( h; h )
echo pipe | { k; k; cat; }
echo "$x $$"
'''


@needs_bash
def test_bash_tracer_subshells_report_their_own_calls_exactly_once(tmp_path, collector):
    script = make_bash_repo(tmp_path, SUBSHELLS)
    result = run_bash(script, collector.port)
    assert result.returncode == 0 and result.stderr == b'', result.stderr
    lines = result.stdout.decode('utf-8', 'replace').split()
    assert lines[0] == 'pipe' and lines[1] == 'sub'
    parent = int(lines[2])
    collector.wait_quiet()
    assert bash_totals(collector.messages) == {('t.sh', 'f'): 3, ('t.sh', 'g'): 2, ('t.sh', 'h'): 2,
                                               ('t.sh', 'k'): 2}
    names_by_pid: dict = {}
    for m in collector.messages:
        names_by_pid.setdefault(m['pid'], set()).update(name for name, _count in m['calls'])
    assert names_by_pid.pop(parent) == {'f'}             # the parent sends only its own f's (2 of them at exit) ...
    assert sorted(sorted(names) for names in names_by_pid.values()) == [['g'], ['h'], ['k']]   # ... each subshell its own


EXEC = r'''#!/bin/bash
source ../codemap/trace/astra_trace.sh
f() { :; }
f; f
exec true
'''


@needs_bash
def test_bash_tracer_sends_the_waiting_counts_right_before_exec(tmp_path, collector):
    # `exec` replaces the shell without running the EXIT trap: the 2nd f (waiting for its 100 ms) must go first.
    script = make_bash_repo(tmp_path, EXEC)
    result = run_bash(script, collector.port)
    assert result.returncode == 0 and result.stderr == b'', result.stderr
    collector.wait_quiet()
    assert bash_totals(collector.messages) == {('t.sh', 'f'): 2}


PROMPT = r'''#!/bin/bash
source ../codemap/trace/astra_trace.sh
f() { :; }
f; f
end=$(( ${EPOCHREALTIME//[!0-9]/} + 300000 ))
while [ "${EPOCHREALTIME//[!0-9]/}" -lt "$end" ]; do :; done
sleep 4
echo done
'''


@needs_bash
def test_bash_tracer_sends_waiting_counts_without_waiting_for_the_next_function(tmp_path, collector):
    # The 2nd f waits (less than 100 ms after the 1st). It must go out at the first plain command after the
    # 100 ms, not only when the script ends: so it is there while the script still sleeps.
    script = make_bash_repo(tmp_path, PROMPT)
    env = clean_env(ASTRA_TRACE='1', ASTRA_TRACE_ADDR='127.0.0.1:%d' % collector.port)
    proc = subprocess.Popen([BASH, script.name], cwd=str(script.parent), env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    try:
        end = time.monotonic() + 3.0
        while time.monotonic() < end and bash_totals(collector.messages).get(('t.sh', 'f'), 0) < 2:
            time.sleep(0.05)
        assert proc.poll() is None, 'the script ended too early for this test'
        assert bash_totals(collector.messages) == {('t.sh', 'f'): 2}
    finally:
        out, err = proc.communicate(timeout=30)
    assert proc.returncode == 0 and out.strip() == b'done' and err == b'', err


@needs_bash
def test_a_loose_copy_of_the_bash_tracer_does_not_count_every_script(tmp_path, collector):
    # Like the Python test: astra_trace.sh copied OUTSIDE <repo>/simulation/codemap/trace must not make
    # "three folders up" (here: a folder holding the whole pytest temp area) a root.
    (tmp_path / 'loose').mkdir()
    shutil.copy(TRACE_DIR / 'astra_trace.sh', tmp_path / 'loose' / 'astra_trace.sh')
    (tmp_path / 'elsewhere').mkdir()
    script = tmp_path / 'elsewhere' / 's.sh'
    script.write_bytes(b'source ../loose/astra_trace.sh\ng() { :; }\ng\ng\necho "trap=[$(trap -p DEBUG)]"\n')
    result = run_bash(script, collector.port)
    assert result.returncode == 0 and result.stderr == b'', result.stderr
    assert result.stdout.startswith(b'trap=[trap -- ')          # the tracer is on ...
    collector.wait_quiet(0.4, 3)
    assert collector.messages == []                             # ... but s.sh is not team code


@needs_bash
@pytest.mark.parametrize('trace', [None, '0'])
def test_bash_tracer_does_nothing_without_astra_trace(tmp_path, collector, trace):
    script = make_bash_repo(tmp_path, 'source ../codemap/trace/astra_trace.sh\ng() { :; }\ng\n'
                                      'echo "trap=[$(trap -p DEBUG)] T=[${-//[^T]/}]"\n')
    result = run_bash(script, collector.port, trace=trace)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == b'trap=[] T=[]'
    collector.wait_quiet(0.4, 2)
    assert collector.messages == []


@needs_bash
def test_bash_tracer_ignores_a_bad_address(tmp_path):
    script = make_bash_repo(tmp_path, 'set -eu\nsource ../codemap/trace/astra_trace.sh\ng() { :; }\ng\n'
                                      'echo "trap=[$(trap -p DEBUG)]"\n')
    for addr in ('nonsense', 'example.com:8091', '127.0.0.1:0', '127.0.0.1:99999', '127.0.0.1:'):
        env = clean_env(ASTRA_TRACE='1', ASTRA_TRACE_ADDR=addr)
        result = subprocess.run([BASH, script.name], cwd=str(script.parent), env=env, capture_output=True,
                                timeout=60)
        assert result.returncode == 0, (addr, result.stderr)
        assert result.stdout.strip() == b'trap=[]', addr         # not switched on at all
        assert result.stderr == b''


@needs_bash
def test_common_sh_hook_switches_the_bash_tracer_on_only_with_astra_trace(collector):
    probe = "source ./_common.sh; say hello >/dev/null; echo \"trap=[$(trap -p DEBUG)]\""
    for trace, expect_on in ((None, False), ('0', False), ('1', True)):
        env = clean_env(ASTRA_TRACE_ADDR='127.0.0.1:%d' % collector.port)
        if trace is not None:
            env['ASTRA_TRACE'] = trace
        result = subprocess.run([BASH, '-c', probe], cwd=str(COMMON_SH.parent), env=env, capture_output=True,
                                timeout=60)
        assert result.returncode == 0, result.stderr
        text = result.stdout.decode('utf-8', 'replace').strip()
        assert (text != 'trap=[]') is expect_on, text
    collector.wait_quiet()
    got = bash_totals(collector.messages)
    assert got.get(('_common.sh', 'say')) == 1, got


# =============================================================================================================
#  demo_traced.sh (with a stand-in demo.sh: no simulator)
# =============================================================================================================
STUB_DEMO = r'''#!/usr/bin/env bash
# demo.sh -- stand-in for the test: sources _common.sh, calls helpers, runs team code, like the real demo would.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
on_int() { echo "stub interrupted"; exit 130; }
trap on_int INT TERM
say "stub step 1"
mkdir -p "$ASTRA_SIM_HOME/runs/20990101-000000_stub"
python3 -c "import sys; sys.path.insert(0, 'ros2_ws/src/main/src'); import target_geolocation as g
print(g.wrap_360(370.0))"
say "stub step 2"
echo "stub env: ASTRA_TRACE=${ASTRA_TRACE:-} ADDR=${ASTRA_TRACE_ADDR:-}"
if [ "${STUB_WAIT:-0}" = 1 ]; then echo "stub waiting"; for _ in $(seq 1 300); do sleep 0.1; done; fi
exit 0
'''


def make_traced_demo_repo(tmp_path: Path) -> tuple[Path, str, str]:
    """A throw-away repo copy: the real demo_traced.sh, _common.sh, server.py and trace/, a stand-in demo.sh,
    the real target_geolocation.py and a small codemap.json naming `say` and `wrap_360`."""
    repo = tmp_path / 'fakerepo'
    scripts = repo / 'simulation' / 'scripts'
    scripts.mkdir(parents=True)
    (repo / 'simulation' / 'setup').mkdir()
    for name in ('demo_traced.sh', '_common.sh'):
        shutil.copy(REPO / 'simulation' / 'scripts' / name, scripts / name)
    (scripts / 'demo.sh').write_bytes(STUB_DEMO.encode('utf-8'))
    shutil.copytree(TRACE_DIR, repo / 'simulation' / 'codemap' / 'trace',
                    ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copy(SERVER_PY, repo / 'simulation' / 'codemap' / 'server.py')
    geo_src = REPO / 'ros2_ws' / 'src' / 'main' / 'src' / 'target_geolocation.py'
    (repo / 'ros2_ws' / 'src' / 'main' / 'src').mkdir(parents=True)
    shutil.copy(geo_src, repo / 'ros2_ws' / 'src' / 'main' / 'src' / 'target_geolocation.py')
    wrap_line = line_of(geo_src.read_text(encoding='utf-8'), 'def wrap_360')
    say_id, wrap_id = 'fn:simulation/scripts/_common.sh::say', 'fn:ros2_ws/src/main/src/target_geolocation.py::wrap_360'
    codemap = {
        'schema': 1, 'generated': 'test', 'commit': 'test', 'edges': [],
        'nodes': [{'id': 'mod:simulation/scripts/_common.sh', 'kind': 'module', 'label': '_common.sh',
                   'parent': None, 'path': 'simulation/scripts/_common.sh'},
                  {'id': say_id, 'kind': 'function', 'label': 'say', 'parent': 'mod:simulation/scripts/_common.sh'},
                  {'id': 'mod:ros2_ws/src/main/src/target_geolocation.py', 'kind': 'module', 'label': 'x',
                   'parent': None, 'path': 'ros2_ws/src/main/src/target_geolocation.py'},
                  {'id': wrap_id, 'kind': 'function', 'label': 'wrap_360',
                   'parent': 'mod:ros2_ws/src/main/src/target_geolocation.py', 'line': wrap_line}],
        'trace_index': {'py_modules': {}, 'py_files': {}, 'functions': {
            'mod:ros2_ws/src/main/src/target_geolocation.py': {str(wrap_line): wrap_id}}},
    }
    (repo / 'simulation' / 'codemap' / 'codemap.json').write_text(json.dumps(codemap), encoding='utf-8')
    return repo, say_id, wrap_id


def recorded_calls(path: Path) -> dict:
    out: dict = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        event = json.loads(line)
        if event['event'] == 'calls':
            for fid, n in event['calls'].items():
                out[fid] = out.get(fid, 0) + n
    return out


def port_open(port: int) -> bool:
    try:
        socket.create_connection(('127.0.0.1', port), timeout=0.5).close()
        return True
    except OSError:
        return False


linux_only = pytest.mark.skipif(os.name == 'nt' or not shutil.which('python3') or not BASH,
                                reason='demo_traced.sh runs in Ubuntu (WSL); needs bash and python3')


@pytest.mark.skipif(not BASH, reason='no bash here')
def test_demo_traced_help_and_a_bad_port(tmp_path):
    script = REPO / 'simulation' / 'scripts' / 'demo_traced.sh'
    result = subprocess.run([BASH, script.name, '--help'], cwd=str(script.parent), env=clean_env(),
                            capture_output=True, text=True, timeout=60)
    assert result.returncode == 0 and 'demo_traced.sh' in result.stdout and 'PURPOSE' in result.stdout
    assert '--map-port' in result.stdout and '--map-only' in result.stdout
    result = subprocess.run([BASH, script.name, '--map-port', 'abc'], cwd=str(script.parent), env=clean_env(),
                            capture_output=True, text=True, timeout=60)
    assert result.returncode in (1, 2)          # 2 = "typed wrong" with _common.sh's die_usage
    assert 'need a port number, not "abc"' in result.stderr


@linux_only
def test_demo_traced_runs_the_demo_traced_records_and_stops_the_server(tmp_path):
    repo, say_id, wrap_id = make_traced_demo_repo(tmp_path)
    sim_home = tmp_path / 'simhome'
    port, udp = free_port(), free_port(socket.SOCK_DGRAM)
    env = clean_env(ASTRA_SIM_HOME=str(sim_home))
    result = subprocess.run([BASH, 'simulation/scripts/demo_traced.sh', '--map-port', str(port), '--trace-port',
                             str(udp), 'runway', '--headless'], cwd=str(repo), env=env, capture_output=True,
                            text=True, timeout=120, stdin=subprocess.DEVNULL)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'open http://127.0.0.1:%d/' % port in result.stdout
    assert 'stub env: ASTRA_TRACE=1 ADDR=127.0.0.1:%d' % udp in result.stdout
    recordings = list((sim_home / 'codemap' / 'recordings').glob('*.jsonl'))
    assert len(recordings) == 1, result.stdout
    got = recorded_calls(recordings[0])
    assert got.get(say_id, 0) >= 2 and got.get(wrap_id) == 1, got
    copied = sim_home / 'runs' / '20990101-000000_stub' / 'codemap_trace.jsonl'
    assert copied.is_file() and copied.read_bytes() == recordings[0].read_bytes()
    assert not port_open(port)                         # the server was stopped at the end


@linux_only
def test_demo_traced_ctrl_c_finishes_the_demo_then_stops_the_server(tmp_path):
    import signal
    repo, say_id, _wrap_id = make_traced_demo_repo(tmp_path)
    sim_home = tmp_path / 'simhome'
    port, udp = free_port(), free_port(socket.SOCK_DGRAM)
    env = clean_env(ASTRA_SIM_HOME=str(sim_home), STUB_WAIT='1')
    proc = subprocess.Popen([BASH, 'simulation/scripts/demo_traced.sh', '--map-port', str(port), '--trace-port',
                             str(udp)], cwd=str(repo), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, stdin=subprocess.DEVNULL, start_new_session=True)
    lines: list[str] = []
    end = time.monotonic() + 60
    while time.monotonic() < end:
        line = proc.stdout.readline()
        if not line:
            break
        lines.append(line)
        if 'stub waiting' in line:
            break
    assert any('stub waiting' in x for x in lines), ''.join(lines)
    assert port_open(port)
    os.killpg(proc.pid, signal.SIGINT)                 # what Ctrl+C in the terminal does
    out, _ = proc.communicate(timeout=60)
    text = ''.join(lines) + out
    assert proc.returncode == 130, text                # demo.sh's own Ctrl+C exit status
    assert 'stub interrupted' in text
    assert not port_open(port)
    recordings = list((sim_home / 'codemap' / 'recordings').glob('*.jsonl'))
    assert len(recordings) == 1 and recorded_calls(recordings[0]).get(say_id, 0) >= 2


# =============================================================================================================
#  server.py
# =============================================================================================================
def load_server_module():
    spec = importlib.util.spec_from_file_location('codemap_server_under_test', SERVER_PY)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module          # dataclass-free, but keeps pickling/reprs sane
    spec.loader.exec_module(module)
    return module


def fake_codemap(hello_line: int) -> dict:
    def node(nid, kind, parent, **extra):
        d = {'id': nid, 'kind': kind, 'label': nid.rsplit('::', 1)[-1].rsplit('/', 1)[-1], 'parent': parent}
        d.update(extra)
        return d

    return {
        'schema': 1, 'generated': '2026-10-31T10:00:00', 'commit': 'abc1234',
        'nodes': [
            node('sys:team_stack', 'system', None),
            node('pkg:tools', 'package', 'sys:team_stack', path='tools'),
            node('mod:tools/run.py', 'module', 'pkg:tools', path='tools/run.py', lang='python'),
            node('fn:tools/run.py::main', 'function', 'mod:tools/run.py', line=10),
            node('mod:pkg/mod.py', 'module', 'sys:team_stack', path='pkg/mod.py', lang='python'),
            node('fn:pkg/mod.py::f', 'function', 'mod:pkg/mod.py', line=3),
            node('fn:pkg/mod.py::Thing', 'class', 'mod:pkg/mod.py', line=6),
            node('fn:pkg/mod.py::Thing.go', 'method', 'fn:pkg/mod.py::Thing', line=7),
            node('mod:simulation/scripts/demo.sh', 'module', 'sys:team_stack', path='simulation/scripts/demo.sh',
                 lang='bash'),
            node('fn:simulation/scripts/demo.sh::finish', 'function', 'mod:simulation/scripts/demo.sh'),
            node('mod:tinymod.py', 'module', 'sys:team_stack', path='tinymod.py', lang='python'),
            node('fn:tinymod.py::hello', 'function', 'mod:tinymod.py', line=hello_line),
        ],
        'edges': [],
        'trace_index': {
            'py_modules': {'pkg.mod': 'mod:pkg/mod.py', 'tools.run': 'mod:tools/run.py'},
            'py_files': {'pkg/mod.py': 'mod:pkg/mod.py', 'tools/run.py': 'mod:tools/run.py',
                         'tinymod.py': 'mod:tinymod.py'},
            'functions': {
                'mod:pkg/mod.py': {'3': 'fn:pkg/mod.py::f', '7': 'fn:pkg/mod.py::Thing.go'},
                'mod:tools/run.py': {'10': 'fn:tools/run.py::main'},
                'mod:tinymod.py': {str(hello_line): 'fn:tinymod.py::hello'},
            },
        },
    }


def http_get(port: int, path: str, headers: dict | None = None, timeout: float = 10):
    """(status, headers, body bytes) of GET http://127.0.0.1:<port><path>."""
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    try:
        conn.request('GET', path, headers=headers or {})
        resp = conn.getresponse()
        return resp.status, dict(resp.getheaders()), resp.read()
    finally:
        conn.close()


class ServerUnderTest:
    def __init__(self, tmp_path: Path, hello_line: int):
        self.port = free_port()
        self.udp_port = free_port(socket.SOCK_DGRAM)
        self.dir = tmp_path / 'server'
        (self.dir / 'web').mkdir(parents=True)
        (self.dir / 'web' / 'index.html').write_text('<!doctype html><title>test page</title>TEST-INDEX-MARKER',
                                                     encoding='utf-8')
        self.codemap_path = self.dir / 'codemap.json'
        self.codemap = fake_codemap(hello_line)
        self.codemap_path.write_text(json.dumps(self.codemap), encoding='utf-8')
        self.recordings = self.dir / 'recordings'
        self.record = self.recordings / 'test_run.jsonl'
        self.proc = subprocess.Popen(
            [sys.executable, str(SERVER_PY), '--port', str(self.port), '--udp-port', str(self.udp_port),
             '--codemap', str(self.codemap_path), '--web', str(self.dir / 'web'), '--record', str(self.record),
             '--recordings', str(self.recordings), '--procs-interval', '0.5', '--no-windows'],
            env=clean_env(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8')
        end = time.monotonic() + 20
        while True:
            try:
                status, _h, _b = http_get(self.port, '/health', timeout=1)
                if status == 200:
                    break
            except OSError:
                pass
            if self.proc.poll() is not None or time.monotonic() > end:
                out = self.proc.stdout.read() if self.proc.stdout else ''
                raise AssertionError('server.py did not start: %s' % out)
            time.sleep(0.1)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self, message) -> None:
        data = message if isinstance(message, bytes) else json.dumps(message).encode('utf-8')
        self.udp.sendto(data, ('127.0.0.1', self.udp_port))

    def health(self) -> dict:
        status, _h, body = http_get(self.port, '/health')
        assert status == 200
        return json.loads(body)

    def stop(self) -> None:
        self.udp.close()
        self.proc.terminate()
        try:
            self.proc.wait(10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
        if self.proc.stdout:
            self.proc.stdout.close()


@pytest.fixture
def server(tmp_path):
    s = ServerUnderTest(tmp_path, hello_line=line_of(TINYMOD, 'def hello'))
    yield s
    s.stop()


class SseReader:
    """Reads /events in a thread and keeps (event, data) pairs; also the raw text for format checks."""

    def __init__(self, port: int):
        self.conn = http.client.HTTPConnection('127.0.0.1', port, timeout=30)
        self.conn.request('GET', '/events', headers={'Accept': 'text/event-stream'})
        # http.client gives the socket to the response for an HTTP/1.0 answer (conn.sock becomes None):
        # keep our own handle so close() really hangs up.
        self.sock = self.conn.sock
        self.resp = self.conn.getresponse()
        self.status = self.resp.status
        self.headers = dict(self.resp.getheaders())
        self.events: list[tuple[str, dict]] = []
        self.raw_lines: list[str] = []
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        if self.status == 200:
            self._thread.start()

    def _run(self) -> None:
        event, data = None, []
        try:
            while not self._stop:
                raw = self.resp.readline()
                if not raw:
                    break
                line = raw.decode('utf-8').rstrip('\n')
                self.raw_lines.append(line)
                if line == '':
                    if event and data:
                        self.events.append((event, json.loads('\n'.join(data))))
                    event, data = None, []
                elif line.startswith('event: '):
                    event = line[len('event: '):]
                elif line.startswith('data: '):
                    data.append(line[len('data: '):])
        except (OSError, ValueError, AttributeError):
            pass      # close() hung up while readline() waited (http.client then finds its file gone)

    def calls(self) -> dict:
        out: dict = {}
        for event, data in list(self.events):
            if event == 'calls':
                for fid, n in data['calls'].items():
                    out[fid] = out.get(fid, 0) + n
        return out

    def wait_for(self, predicate, limit: float = 8.0) -> bool:
        end = time.monotonic() + limit
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.05)
        return predicate()

    def close(self) -> None:
        self._stop = True
        for action in (lambda: self.sock.shutdown(socket.SHUT_RDWR), self.sock.close, self.resp.close,
                       self.conn.close):
            try:
                action()
            except (OSError, AttributeError):
                pass


def test_server_serves_the_page_the_codemap_and_health(server):
    status, headers, body = http_get(server.port, '/')
    assert status == 200 and b'TEST-INDEX-MARKER' in body
    assert headers['Content-Type'].startswith('text/html')
    status, headers, body = http_get(server.port, '/codemap.json')
    assert status == 200 and json.loads(body) == server.codemap
    status, headers, body = http_get(server.port, '/codemap.json', headers={'Accept-Encoding': 'gzip'})
    assert status == 200
    if headers.get('Content-Encoding') == 'gzip':
        import gzip
        body = gzip.decompress(body)
    assert json.loads(body) == server.codemap
    h = server.health()
    assert h['ok'] is True
    assert h['codemap']['loaded'] is True and h['codemap']['commit'] == 'abc1234'
    assert h['codemap']['nodes'] == len(server.codemap['nodes'])
    assert h['codemap']['functions_indexed'] == 4
    assert h['udp']['port'] == server.udp_port
    assert h['recording'] and Path(h['recording']).name == 'test_run.jsonl'
    assert http_get(server.port, '/no/such/file')[0] == 404
    assert http_get(server.port, '/../server.py')[0] == 404


def test_server_maps_python_and_bash_messages_to_function_ids(server):
    sse = SseReader(server.port)
    try:
        assert sse.status == 200
        server.send({'pid': 11, 'proc': 'x.py', 'calls': [
            ['pkg.mod', '/home/u/repo/pkg/mod.py', 3, 'f', 2],                 # by dotted module name
            ['__main__', 'C:\\Users\\u\\repo\\tools\\run.py', 10, 'main', 1],    # by the end of a Windows path
            ['__main__', '/home/u/ODLC/tools/run.py', 10, 'main', 1],            # by the end of a Linux path
            ['pkg.mod', '/home/u/repo/pkg/mod.py', 7, 'go', 4],                  # a method
            ['other', '/home/u/repo/other.py', 1, 'zzz', 5],                     # unknown
            ['pkg.mod', '/home/u/repo/pkg/mod.py', 99, 'gone', 1],               # line not in the index
        ]})
        server.send({'pid': 12, 'sh': '/home/u/ODLC/simulation/scripts/demo.sh',
                     'calls': [['finish', 2], ['nosuch', 1]]})
        server.send(b'this is not json')
        server.send({'pid': 13, 'proc': 'odd.py', 'calls': [['too', 'short'], 'garbage', ['m', 'f', 'x', 'n', 1]]})
        expected = {'fn:pkg/mod.py::f': 2, 'fn:tools/run.py::main': 2, 'fn:pkg/mod.py::Thing.go': 4,
                    'fn:simulation/scripts/demo.sh::finish': 2}
        assert sse.wait_for(lambda: sse.calls() == expected), sse.calls()
        time.sleep(0.5)
        assert sse.calls() == expected                     # nothing unknown ever reaches the page
        for event, data in sse.events:
            assert set(data) == ({'t', 'calls'} if event == 'calls' else {'t', 'running'})
            assert isinstance(data['t'], (int, float)) and abs(data['t'] - time.time()) < 60
        h = server.health()
        unknown_py = {(u['name'], u['line']): u['count'] for u in h['unknown_python']}
        assert unknown_py == {('zzz', 1): 5, ('gone', 99): 1}
        assert [(u['name'], u['count']) for u in h['unknown_bash']] == [('nosuch', 1)]
        assert h['received']['bad_datagrams'] == 1
        assert h['received']['datagrams'] == 4
        assert h['received']['mapped_calls'] == 10
        assert {t['pid'] for t in h['tracers']} == {11, 12, 13}
    finally:
        sse.close()


def test_server_event_stream_format(server):
    s = socket.create_connection(('127.0.0.1', server.port), timeout=10)
    try:
        s.sendall(b'GET /events HTTP/1.1\r\nHost: 127.0.0.1\r\nAccept: text/event-stream\r\n\r\n')
        data = b''
        end = time.monotonic() + 5
        while b'event: procs' not in data or not data.endswith(b'\n\n'):
            chunk = s.recv(4096)
            if not chunk or time.monotonic() > end:
                break
            data += chunk
        head, _, body = data.partition(b'\r\n\r\n')
        assert head.startswith(b'HTTP/1.0 200')
        assert b'Content-Type: text/event-stream' in head
        assert b'Cache-Control: no-cache' in head
        text = body.decode('utf-8')
        assert text.startswith('retry: 2000\n\n')
        block = text[len('retry: 2000\n\n'):].split('\n\n')[0].split('\n')
        assert block[0] == 'event: procs'
        assert block[1].startswith('data: ')
        payload = json.loads(block[1][len('data: '):])
        assert set(payload) == {'t', 'running'} and isinstance(payload['running'], list)
    finally:
        s.close()


def test_server_records_every_event_and_lists_recordings(server):
    sse = SseReader(server.port)
    try:
        server.send({'pid': 21, 'proc': 'x.py', 'calls': [['pkg.mod', '/r/pkg/mod.py', 3, 'f', 3]]})
        assert sse.wait_for(lambda: sse.calls() == {'fn:pkg/mod.py::f': 3})
        assert sse.wait_for(lambda: sum(1 for e, _d in sse.events if e == 'procs') >= 2, 5)
    finally:
        sse.close()
    lines = [json.loads(x) for x in server.record.read_text(encoding='utf-8').splitlines()]
    assert lines, 'nothing recorded'
    assert {x['event'] for x in lines} == {'calls', 'procs'}
    for x in lines:
        assert isinstance(x['t'], (int, float))
        assert set(x) == ({'event', 't', 'calls'} if x['event'] == 'calls' else {'event', 't', 'running'})
    assert [x['calls'] for x in lines if x['event'] == 'calls'] == [{'fn:pkg/mod.py::f': 3}]
    status, _h, body = http_get(server.port, '/recordings')
    listing = json.loads(body)
    assert status == 200 and [r['name'] for r in listing] == ['test_run.jsonl']
    assert listing[0]['bytes'] > 0 and listing[0]['url'] == '/recordings/test_run.jsonl'
    assert set(listing[0]) == {'name', 'bytes', 'modified', 'url'}
    status, headers, body = http_get(server.port, '/recordings/test_run.jsonl')
    assert status == 200 and body.decode('utf-8').splitlines()[0] == server.record.read_text(
        encoding='utf-8').splitlines()[0]
    assert http_get(server.port, '/recordings/..%2Fcodemap.json')[0] == 404
    assert http_get(server.port, '/recordings/nothing.jsonl')[0] == 404


@pytest.mark.skipif(os.name == 'nt', reason='Windows has no SIGTERM for another process (terminate() kills)')
def test_server_records_the_last_calls_when_it_is_stopped_right_away(server):
    import signal
    for _ in range(20):
        server.send({'pid': 41, 'proc': 'x.py', 'calls': [['pkg.mod', '/r/pkg/mod.py', 3, 'f', 1]]})
    server.send({'pid': 41, 'proc': 'x.py', 'calls': [['pkg.mod', '/r/pkg/mod.py', 99, 'gone', 2]]})
    time.sleep(0.3)                                    # let the UDP thread take the unknown one in
    server.proc.send_signal(signal.SIGTERM)            # what demo_traced.sh does right after the demo ends
    server.proc.wait(10)
    assert server.proc.returncode == 0
    assert recorded_calls(server.record) == {'fn:pkg/mod.py::f': 20}
    # the last /health answer is saved next to the recording, with the call it could not place
    health = json.loads(server.record.with_name('test_run.health.json').read_text(encoding='utf-8'))
    assert health['final'] is True and health['received']['mapped_calls'] == 20
    assert [(u['name'], u['line'], u['count']) for u in health['unknown_python']] == [('gone', 99, 2)]


def test_server_final_health_file_sits_next_to_the_recording_and_is_never_listed(tmp_path):
    # Direct test of App.write_final_health (works on Windows too, no subprocess, no signals).
    import server as server_module
    codemap = tmp_path / 'codemap.json'
    codemap.write_text(json.dumps({'schema': 1, 'nodes': [], 'edges': [], 'trace_index': {}}), encoding='utf-8')
    args = server_module.parse_args(['--codemap', str(codemap), '--record', str(tmp_path / 'rec' / 'run1.jsonl'),
                                     '--recordings', str(tmp_path / 'rec'), '--no-windows', '--procs-interval', '0'])
    app = server_module.App(args)
    try:
        app.handle_datagram(json.dumps({'pid': 5, 'proc': 'x.py',
                                        'calls': [['m', '/r/m.py', 3, 'nope', 4]]}).encode('utf-8'))
        # colcon's own helper (seen in the first traced demo): unknown, and counted as generated code
        app.handle_datagram(json.dumps({'pid': 6, 'proc': '_local_setup_util_sh.py', 'calls': [
            ['__main__', '/home/u/ODLC/ros2_ws/install/_local_setup_util_sh.py', 176, '_include_comments', 7]]}
        ).encode('utf-8'))
        app.handle_datagram(json.dumps({'pid': 7, 'sh': '/home/u/ODLC/ros2_ws/install/setup.bash',
                                        'calls': [['_colcon_prefix_chain_bash_source_script', 2]]}).encode('utf-8'))
        written = app.write_final_health()
    finally:
        app.hub.close()
    assert written == tmp_path / 'rec' / 'run1.health.json'
    health = json.loads(written.read_text(encoding='utf-8'))
    assert health['final'] is True and health['received']['unknown_calls'] == 13
    assert health['received']['unknown_generated_calls'] == 9
    assert len(health['unknown_python']) == 2 and len(health['unknown_bash']) == 1
    assert [r['name'] for r in app.list_recordings()] == ['run1.jsonl']     # the .health.json is not a recording
    # without --record nothing is written
    app2 = server_module.App(server_module.parse_args(['--codemap', str(codemap), '--no-windows']))
    try:
        assert app2.write_final_health() is None
    finally:
        app2.hub.close()


def test_server_lists_the_linux_copy_recordings_by_default(tmp_path, monkeypatch):
    # demo_traced.sh started from the Linux-side copy (no .git there) records into ~/.astra_sim/codemap/recordings
    # ($ASTRA_SIM_HOME/codemap/recordings). A server started later WITHOUT --recordings must list those runs too,
    # so the Replay tab finds them; an explicit --recordings DIR lists only DIR.
    import server as server_module
    sim_home = tmp_path / 'sim_home'
    linux_dir = sim_home / 'codemap' / 'recordings'
    linux_dir.mkdir(parents=True)
    (linux_dir / '20991231-235959.jsonl').write_text('{"event": "procs", "t": 1, "running": []}\n', encoding='utf-8')
    monkeypatch.setenv('ASTRA_SIM_HOME', str(sim_home))
    codemap = tmp_path / 'codemap.json'
    codemap.write_text(json.dumps({'schema': 1, 'nodes': [], 'edges': [], 'trace_index': {}}), encoding='utf-8')
    assert server_module.sim_home_recordings() == linux_dir.resolve()
    app = server_module.App(server_module.parse_args(['--codemap', str(codemap), '--no-windows',
                                                      '--procs-interval', '0']))
    try:
        assert app.recording_dirs() == [(CODEMAP_DIR / 'recordings').resolve(), linux_dir.resolve()]
        assert '20991231-235959.jsonl' in [r['name'] for r in app.list_recordings()]
        assert app.find_recording('20991231-235959.jsonl') == linux_dir.resolve() / '20991231-235959.jsonl'
    finally:
        app.hub.close()
    only = tmp_path / 'only'
    only.mkdir()
    app2 = server_module.App(server_module.parse_args(['--codemap', str(codemap), '--recordings', str(only),
                                                       '--no-windows', '--procs-interval', '0']))
    try:
        assert app2.recording_dirs() == [only.resolve()]
        assert app2.list_recordings() == [] and app2.find_recording('20991231-235959.jsonl') is None
    finally:
        app2.hub.close()


def test_server_accepts_at_most_8_live_pages(server):
    readers = [SseReader(server.port) for _ in range(8)]
    try:
        assert [r.status for r in readers] == [200] * 8
        status, _h, body = http_get(server.port, '/events')
        assert status == 503 and b'Too many' in body
    finally:
        for r in readers:
            r.close()
    # the places free up again once pages close
    end = time.monotonic() + 5
    while server.health()['sse_clients'] and time.monotonic() < end:
        time.sleep(0.1)
    again = SseReader(server.port)
    try:
        assert again.status == 200
    finally:
        again.close()


def test_server_rereads_the_codemap_when_it_changes(server):
    data = server.codemap
    data['nodes'].append({'id': 'fn:pkg/mod.py::later', 'kind': 'function', 'label': 'later',
                          'parent': 'mod:pkg/mod.py', 'line': 20})
    data['trace_index']['functions']['mod:pkg/mod.py']['20'] = 'fn:pkg/mod.py::later'
    time.sleep(0.05)
    server.codemap_path.write_text(json.dumps(data), encoding='utf-8')
    os.utime(server.codemap_path, (time.time() + 5, time.time() + 5))     # a clearly newer time stamp
    end = time.monotonic() + 6
    while server.health()['codemap']['functions_indexed'] != 5 and time.monotonic() < end:
        time.sleep(0.1)
    assert server.health()['codemap']['functions_indexed'] == 5
    sse = SseReader(server.port)
    try:
        server.send({'pid': 31, 'proc': 'x.py', 'calls': [['pkg.mod', '/r/pkg/mod.py', 20, 'later', 1]]})
        assert sse.wait_for(lambda: sse.calls() == {'fn:pkg/mod.py::later': 1})
    finally:
        sse.close()


def test_end_to_end_traced_program_lights_up_its_function(server, tiny_repo):
    sse = SseReader(server.port)
    try:
        result, _pid = run_traced(tiny_repo / 'tinymod.py', server.udp_port, tiny_repo)
        assert result.returncode == 0 and result.stdout.strip() == 'done', result.stderr
        assert sse.wait_for(lambda: sse.calls().get('fn:tinymod.py::hello') == 9), sse.calls()
    finally:
        sse.close()
    names = {u['name'] for u in server.health()['unknown_python']}
    assert {'main', 'wave', 'twice'} <= names           # not in the fake codemap: listed, never sent


# ---- pure mapping rules (no server process) --------------------------------------------------------------
REAL_LIKE = {
    'nodes': [
        {'id': 'pkg:ros2_ws/src/simulation/sim_bringup', 'kind': 'package', 'label': 'sim_bringup', 'parent': None},
        {'id': 'pkg:ros2_ws/src/main', 'kind': 'package', 'label': 'main', 'parent': None},
        {'id': 'pkg:ros2_ws/src/simulation/sim_demo', 'kind': 'package', 'label': 'sim_demo', 'parent': None},
        {'id': 'mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'kind': 'module', 'label': 'x',
         'parent': 'pkg:ros2_ws/src/simulation/sim_bringup',
         'path': 'ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py'},
        {'id': 'mod:ros2_ws/src/simulation/sim_bringup/sim_bringup/phase_monitor.py', 'kind': 'module',
         'label': 'x', 'parent': 'pkg:ros2_ws/src/simulation/sim_bringup', 'entry_point': 'phase_monitor',
         'path': 'ros2_ws/src/simulation/sim_bringup/sim_bringup/phase_monitor.py'},
        {'id': 'mod:ros2_ws/src/main/src/main_controller.py', 'kind': 'module', 'label': 'x',
         'parent': 'pkg:ros2_ws/src/main', 'path': 'ros2_ws/src/main/src/main_controller.py'},
        {'id': 'mod:ros2_ws/src/simulation/sim_demo/sim_demo/run_reset.py', 'kind': 'module', 'label': 'x',
         'parent': 'pkg:ros2_ws/src/simulation/sim_demo',
         'path': 'ros2_ws/src/simulation/sim_demo/sim_demo/run_reset.py'},
        {'id': 'mod:simulation/scripts/demo.sh', 'kind': 'module', 'label': 'x', 'parent': None,
         'path': 'simulation/scripts/demo.sh'},
        {'id': 'mod:launcher.sh', 'kind': 'module', 'label': 'x', 'parent': None, 'path': 'launcher.sh'},
        {'id': 'fn:launcher.sh::shutdown', 'kind': 'function', 'label': 'x', 'parent': 'mod:launcher.sh'},
    ],
    'edges': [],
    'trace_index': {'py_modules': {'sim_demo.run_reset': 'mod:ros2_ws/src/simulation/sim_demo/sim_demo/run_reset.py'},
                    'py_files': {}, 'functions': {}},
}
H = '/home/j/ODLC_Machine_Inferencing_System_2025-2026'
COMMAND_CASES = [
    (['gz sim -s -r -v 1 %s/ros2_ws/install/sim_worlds/share/sim_worlds/worlds/runway.sdf      ' % H], None,
     {'ext:gazebo'}),
    (['ruby', '/usr/bin/gz', 'sim', '-g'], None, {'ext:gazebo'}),
    (['/home/j/sitl/arducopter', '--model', 'JSON', '-I', '0'], None, {'ext:ardupilot_sitl'}),
    (['/home/j/sitl/venv/bin/python3', '/home/j/sitl/venv/bin/mavproxy.py', '--master', 'tcp:127.0.0.1:5760'],
     None, {'ext:mavproxy'}),
    (['/opt/ros/humble/lib/mavros/mavros_node', '--ros-args', '-r', '__node:=mavros'], None, {'ext:mavros'}),
    (['mono', '/opt/MissionPlanner/MissionPlanner.exe'], None, {'ext:mission_planner'}),
    (['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'sim_bringup', 'sim.launch.py', 'world:=runway'],
     None, {'mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py'}),
    (['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'sim_bringup', 'phase_monitor', '--log', 'x'], None,
     {'mod:ros2_ws/src/simulation/sim_bringup/sim_bringup/phase_monitor.py'}),
    (['/usr/bin/python3', H + '/ros2_ws/install/sim_bringup/lib/sim_bringup/phase_monitor', '--log', 'x'], None,
     {'mod:ros2_ws/src/simulation/sim_bringup/sim_bringup/phase_monitor.py'}),
    (['/usr/bin/python3', H + '/ros2_ws/install/main/lib/main/main_controller.py', '--ros-args'], None,
     {'mod:ros2_ws/src/main/src/main_controller.py'}),
    (['bash', H + '/simulation/scripts/demo.sh', 'runway'], None, {'mod:simulation/scripts/demo.sh'}),
    (['bash', 'simulation/scripts/demo.sh'], H, {'mod:simulation/scripts/demo.sh'}),
    (['/bin/bash', './launcher.sh', 'sim'], H, {'mod:launcher.sh'}),
    (['setsid', 'ros2', 'launch', 'sim_bringup', 'sim.launch.py'], None,
     {'mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py'}),
    (['python3', '-u', '-m', 'sim_demo.run_reset', 'reset'], None,
     {'mod:ros2_ws/src/simulation/sim_demo/sim_demo/run_reset.py'}),
    (['python3', '-c', 'print(1)'], None, set()),
    (['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'topic', 'echo', '/x'], None, set()),
    (['timeout', '15', 'ros2', 'service', 'call', '/mavros/mission/pull'], None, set()),
    (['sleep', '2'], None, set()),
    ([], None, set()),
]


@pytest.mark.parametrize('argv, cwd, expected', COMMAND_CASES)
def test_running_programs_are_recognised_from_their_command_lines(argv, cwd, expected):
    server_module = load_server_module()
    index = server_module.CodeMapIndex(REAL_LIKE, repo_root=None)
    assert set(index.map_command(argv, cwd)) == expected


def test_index_rules_for_python_and_bash_messages():
    server_module = load_server_module()
    index = server_module.CodeMapIndex(fake_codemap(hello_line=6), repo_root=None)
    assert index.map_py('pkg.mod', '/x/pkg/mod.py', 3) == 'fn:pkg/mod.py::f'
    assert index.map_py('pkg.mod', '/x/other/zz.py', 3) is None           # the name says pkg.mod, the file not
    assert index.map_py('__main__', 'D:\\repo\\tools\\run.py', 10) == 'fn:tools/run.py::main'
    assert index.map_py('__main__', '/elsewhere/run.py', 10) is None       # basename alone is not enough
    assert index.map_py('__main__', '/h/u/ODLC/install/tools/lib/tools/run.py', 10) == 'fn:tools/run.py::main'
    assert index.map_py('tools.run', '/x/tools/run.py', 11) is None        # a line the index does not know
    assert index.map_sh('/h/u/ODLC/simulation/scripts/demo.sh', 'finish') == 'fn:simulation/scripts/demo.sh::finish'
    assert index.map_sh('/h/u/ODLC/simulation/scripts/demo.sh', 'nosuch') is None
    assert index.map_sh('/other/place/thing.sh', 'finish') is None
    assert index.file_to_mod('simulation/scripts/demo.sh') == 'mod:simulation/scripts/demo.sh'


def test_index_places_a_function_by_its_name_when_the_file_changed_after_extract():
    # Seen in the first traced demo (2026-10-31): other people edited flight_recorder.py after extract.py ran, so
    # every line moved and its calls could not be placed by line. A name that is unique in the file is enough.
    server_module = load_server_module()
    data = fake_codemap(hello_line=6)
    data['trace_index']['functions']['mod:pkg/mod.py']['20'] = 'fn:pkg/mod.py::Other.f'   # a second "f"
    index = server_module.CodeMapIndex(data, repo_root=None)
    assert index.map_py('tools.run', '/x/tools/run.py', 14, 'main') == 'fn:tools/run.py::main'   # moved 10 -> 14
    assert index.map_py('__main__', '/h/ODLC/tools/run.py', 15, 'main') == 'fn:tools/run.py::main'  # by the path
    assert index.by_name_hits == 2
    assert index.map_py('pkg.mod', '/x/pkg/mod.py', 9, 'go') == 'fn:pkg/mod.py::Thing.go'        # method: short name
    assert index.map_py('pkg.mod', '/x/pkg/mod.py', 30, 'f') is None       # two functions named f: never guess
    assert index.map_py('pkg.mod', '/x/pkg/mod.py', 31, 'brand_new') is None                     # a new function
    assert index.map_py('pkg.mod', '/x/pkg/mod.py', 32, '<lambda>') is None                      # never lambdas
    assert index.map_py('pkg.mod', '/x/pkg/mod.py', 3, 'renamed') == 'fn:pkg/mod.py::f'          # line still wins
    assert index.map_py('nope', '/elsewhere/zz.py', 3, 'main') is None                           # unknown file


EXTRACTOR_STYLE = {
    'nodes': [
        {'id': 'ext:rviz', 'kind': 'external', 'label': 'RViz2', 'parent': None, 'process_match': ['\\brviz2\\b']},
        {'id': 'ext:gazebo', 'kind': 'external', 'label': 'Gazebo', 'parent': None,
         'process_match': ['(^|/|\\s)gz\\s+sim\\b', '[unclosed']},                  # a broken pattern is skipped
        {'id': 'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py', 'kind': 'module', 'label': 'x', 'parent': None,
         'path': 'ros2_ws/src/video_cam/video_cam/siyi_node.py'},
        {'id': 'mod:ros2_ws/src/detection/detection/new_od.py', 'kind': 'module', 'label': 'x', 'parent': None,
         'path': 'ros2_ws/src/detection/detection/new_od.py'},
        {'id': 'fn:ros2_ws/src/detection/detection/new_od.py::main', 'kind': 'function', 'label': 'main',
         'parent': 'mod:ros2_ws/src/detection/detection/new_od.py', 'entry_point': 'ros2 run detection new_od'},
        {'id': 'fn:ros2_ws/src/video_cam/video_cam/siyi_node.py::SiyiNode.take', 'kind': 'method', 'label': 'x',
         'parent': 'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py'},
    ],
    'edges': [],
    'trace_index': {
        'py_modules': {}, 'functions': {
            'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py': {
                '40': 'fn:ros2_ws/src/video_cam/video_cam/siyi_node.py::SiyiNode.take'}},
        'py_files': {'ros2_ws/build/video_cam/video_cam/siyi_node.py':
                     'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py'},
        'programs': {'ros2_ws/install/video_cam/lib/video_cam/siyi':
                     'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py'},
        'sh_files': {},
    },
}


@pytest.mark.parametrize('argv, expected', [
    (['/opt/ros/humble/lib/rviz2/rviz2', '-d', 'x.rviz'], {'ext:rviz'}),
    (['ruby', '/usr/bin/gz', 'sim', '-g'], {'ext:gazebo'}),
    (['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'x', 'y', 'note:=rviz2'], set()),   # 4th word
    (['/usr/bin/python3', H + '/ros2_ws/install/video_cam/lib/video_cam/siyi', '--ros-args'],
     {'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py'}),                                         # programs
    (['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'video_cam', 'siyi'],
     {'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py'}),
    (['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'detection', 'new_od'],
     {'mod:ros2_ws/src/detection/detection/new_od.py'}),                                            # entry_point
])
def test_extractor_tables_drive_the_command_line_rules(argv, expected):
    server_module = load_server_module()
    index = server_module.CodeMapIndex(EXTRACTOR_STYLE, repo_root=None)
    assert set(index.map_command(argv, None)) == expected


def test_copies_listed_in_py_files_map_to_the_source_module():
    server_module = load_server_module()
    index = server_module.CodeMapIndex(EXTRACTOR_STYLE, repo_root=None)
    assert index.map_py('__main__', H + '/ros2_ws/build/video_cam/video_cam/siyi_node.py', 40) == \
        'fn:ros2_ws/src/video_cam/video_cam/siyi_node.py::SiyiNode.take'


REAL_CODEMAP = CODEMAP_DIR / 'codemap.json'


@pytest.mark.skipif(not REAL_CODEMAP.is_file(), reason='no codemap.json yet (made by extract.py)')
def test_the_real_codemap_places_real_messages_and_programs():
    """The extractor's own codemap.json: real file paths, real command lines -> ids that exist in it."""
    server_module = load_server_module()
    data = json.loads(REAL_CODEMAP.read_text(encoding='utf-8'))
    index = server_module.CodeMapIndex(data, repo_root=REPO)
    ids = index.node_ids
    # bash: a function of _common.sh, reported with the Linux-side copy's absolute path
    say = index.map_sh(H + '/simulation/scripts/_common.sh', 'say')
    assert say == 'fn:simulation/scripts/_common.sh::say' and say in ids
    # Python: every (module file, first line) pair of the trace index comes back as its own fn id, both by
    # the Linux-side copy's absolute path and, for package modules, by the dotted module name
    by_module = {mod: name for name, mod in data['trace_index']['py_modules'].items()}
    checked = 0
    for mod, lines in list(data['trace_index']['functions'].items())[:80]:
        path = mod[4:]
        for line, fid in list(lines.items())[:5]:
            assert index.map_py('__main__', H + '/' + path, int(line)) == fid
            if mod in by_module:
                assert index.map_py(by_module[mod], H + '/' + path, int(line)) == fid
            checked += 1
    assert checked > 50
    # running programs (command lines as seen in WSL during a demo)
    cases = {
        ('/usr/bin/python3', H + '/ros2_ws/install/video_cam/lib/video_cam/siyi', '--ros-args'):
            'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py',
        ('/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'run', 'sim_bringup', 'phase_monitor', '--log', 'x'):
            'mod:ros2_ws/src/simulation/sim_bringup/sim_bringup/phase_monitor.py',
        ('/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'sim_bringup', 'sim.launch.py', 'world:=runway'):
            'mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py',
        ('/usr/bin/python3', H + '/ros2_ws/install/main/lib/main/main_controller.py', '--ros-args'):
            'mod:ros2_ws/src/main/src/main_controller.py',
        ('bash', H + '/simulation/scripts/demo.sh', 'runway'): 'mod:simulation/scripts/demo.sh',
        ('gz sim -s -r -v 1 %s/ros2_ws/install/sim_worlds/share/sim_worlds/worlds/runway.sdf   ' % H,): 'ext:gazebo',
        ('/home/j/sitl/arducopter', '--model', 'JSON', '-I', '0'): 'ext:ardupilot_sitl',
        ('/home/j/sitl/venv/bin/python3', '/home/j/sitl/venv/bin/mavproxy.py', '--master', 'tcp:x'): 'ext:mavproxy',
        ('/opt/ros/humble/lib/mavros/mavros_node', '--ros-args'): 'ext:mavros',
        ('/opt/ros/humble/lib/rviz2/rviz2', '-d', 'sim.rviz'): 'ext:rviz',
        ('/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge', '/clock@x'): 'ext:ros_gz_bridge',
    }
    for argv, expected in cases.items():
        if expected.startswith('ext:') and expected not in ids:
            continue
        assert expected in ids, expected
        assert set(index.map_command(list(argv), None)) == {expected}, argv


def test_entry_points_come_from_setup_py_of_the_real_repo():
    server_module = load_server_module()
    if not (REPO / 'ros2_ws' / 'src' / 'video_cam' / 'setup.py').is_file():
        pytest.skip('the team packages are not in this checkout')
    data = {'nodes': [{'id': 'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py', 'kind': 'module', 'label': 'x',
                       'parent': None, 'path': 'ros2_ws/src/video_cam/video_cam/siyi_node.py'}],
            'edges': [], 'trace_index': {'py_modules': {}, 'py_files': {}, 'functions': {}}}
    index = server_module.CodeMapIndex(data, repo_root=REPO)
    setup = (REPO / 'ros2_ws' / 'src' / 'video_cam' / 'setup.py').read_text(encoding='utf-8')
    if 'video_cam.siyi_node' not in setup:
        pytest.skip('video_cam setup.py does not name video_cam.siyi_node')
    argv = ['/usr/bin/python3', H + '/ros2_ws/install/video_cam/lib/video_cam/siyi', '--ros-args']
    assert set(index.map_command(argv, None)) == {'mod:ros2_ws/src/video_cam/video_cam/siyi_node.py'}
