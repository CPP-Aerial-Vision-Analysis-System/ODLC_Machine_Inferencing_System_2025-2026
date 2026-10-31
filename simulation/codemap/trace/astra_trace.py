"""astra_trace.py -- PURPOSE: the opt-in live call tracer of the ASTRA code map (simulation/codemap).

What it is for
--------------
The code map web page (simulation/codemap/web/index.html, served by simulation/codemap/server.py) draws every
file and function of the team code as blocks. While the simulator runs, a block lights up green when its
function is called. This file is the part that NOTICES the calls inside each Python program.

How it works, in plain words
----------------------------
1. A Python program starts with ASTRA_TRACE=1 and with the folder of this file (simulation/codemap/trace) on
   PYTHONPATH. Python then runs sitecustomize.py (next to this file) before the program's own code, and
   sitecustomize.py calls install() below. Without ASTRA_TRACE=1 nothing in this file ever runs.
2. install() asks Python to call a tiny function, the "profiler", every time any function starts
   (sys.setprofile for this thread, threading.setprofile for every thread started later).
3. The profiler looks at the FILE the function comes from. Only files that live in the team repo count:
   the Linux-side copy ~/ODLC_Machine_Inferencing_System_2025-2026 (what the simulator runs), the Windows
   checkout (/mnt/<drive>/.../ODLC_Machine_Inferencing_System_2025-2026), and whatever ASTRA_TRACE_ROOTS
   lists. The decision is made ONCE per file (the answer is kept in a dictionary keyed by the file name);
   after that a call from Python itself, ROS 2, numpy, torch ... costs one quick dictionary look-up and is
   skipped. Files of the code map tools themselves (simulation/codemap/...) never count.
   Symbolic links are resolved first, so a file that `colcon build --symlink-install` linked into
   ros2_ws/build or ros2_ws/install is reported at its real place, ros2_ws/src/....
4. For our files it counts the calls per code object (one code object = one function or method).
5. A small background thread (started only when the first file of ours is seen) sends the new counts every
   100 ms as one UDP message (a "datagram") with JSON text to 127.0.0.1:8091 (ASTRA_TRACE_ADDR), where
   server.py turns them into "this function just ran" for the web page. When the program ends normally the
   last counts are sent too (atexit).

The message (the code map's shared contract, do not change it without changing server.py):
    {"pid": 1234, "proc": "main_controller.py",
     "calls": [[module __name__, file, first line of the def, function name, number of calls], ...]}
  "first line" is code.co_firstlineno: the line of the first decorator when the function has decorators.
  "file" is the real path of the file (symbolic links resolved).

Safety rules (the team code must behave the same with tracing on; the known limits follow right after)
-----------------------------------------------------------------------------------------------------
  * OFF unless ASTRA_TRACE=1 (sitecustomize.py checks it; the PYTHONPATH entry alone does nothing).
  * Programs named in ASTRA_TRACE_SKIP (default: MAVProxy) are left completely alone, even with ASTRA_TRACE=1.
  * Errors: the profiler (the function Python calls for every call) catches exactly the two errors it can cause
    ITSELF, RecursionError and MemoryError, and nothing else. So it never swallows an exception that belongs to
    the program (for example one that the program's own signal handler raises at the very instant the profiler
    runs): that one reaches the program as usual. The rest of the tracer (the once-per-function look at a new
    function, sending, install) catches every Exception: a broken tracer only stops reporting.
  * Never blocks: the UDP socket is non-blocking; a message that cannot be sent at once is dropped. Name
    look-ups (DNS) of ASTRA_TRACE_ADDR happen in the background thread only, never in the program's code.
  * Rate-limited: at most one flush every 100 ms per process. A flush is one datagram (split only if it
    would be larger than about 60 kB, at most 8 datagrams per flush; the rest waits for the next flush).
    When the program ends normally, one last flush sends what the last 100 ms counted (atexit).
  * Overhead: see "Measured cost" below. Code outside the repo pays one tiny Python call per profiler event
    and nothing else (no counting, no sending).
  * Counts are exact in a single thread; when several threads call the SAME function at the same instant
    an increment can be lost (the map only needs "it ran" and a rough count).

Known limits (they come from sys.setprofile itself, the only way Python 3.10 offers to see every call; they
apply only with ASTRA_TRACE=1)
----------------------------------------------------------------------------------------------------------
  * Recursion: the profiler runs as one more Python frame on top of the program's own frames, so a traced
    program has 1 frame less of Python's recursion limit (sys.getrecursionlimit(), 1000 by default). Code that
    recurses down to the very last frame gets its RecursionError 1 frame sooner than untraced. Measured
    2026-10-31 with sys.setrecursionlimit(300), a function calling itself until RecursionError: WSL Python 3.10
    plain 298 levels, traced 297 (before the whole profiler body was inside the try below: 296, and the error
    came from inside the profiler); the Windows Python 3.11 numbers are in simulation/codemap/README.md.
  * When an exception leaves the profiler anyway (that RecursionError at the last frame, or the program's own
    exception raised by its signal handler while the profiler runs, such as KeyboardInterrupt on Ctrl+C), Python
    hands it to the program as usual AND switches the profiler off for that thread: the program goes on exactly
    as untraced, but that thread reports no more calls. (The only signal handler in the repo that raises is
    sim_worlds/gzrun.py's, which turns SIGINT/SIGTERM into KeyboardInterrupt to stop.)
  * The once-per-function first look (_first_call) catches every Exception, so a program's signal-handler
    exception raised during exactly that look (a few microseconds, once per function) would be lost.

Measured cost (2026-10-31, the team laptop, re-measured after the last change; best of 5 runs; the same code
once OUTSIDE the roots (Python, ROS, numpy ...: looked at, skipped) and once INSIDE them (team code: counted))
--------------------------------------------------------------------------------------------------------------
Python calls the profiler for every function call AND return AND every call of a C function (len, abs, the
json C code ...). Each such "event" costs about 160-250 ns when the code is NOT ours; a call of OUR code
(counted) costs about 420-430 ns. Plain time -> time with the tracer, outside / inside the roots:
                                     WSL, Python 3.10                   Windows, Python 3.11
  1,000,000 calls of a tiny function   0.061 -> 0.309 / 0.490 s (x5.1/x8.0)  0.051 -> 0.302 / 0.470 s (x5.9/x9.2)
  1,000,000 rounds of len/abs/isinstance 0.150 -> 0.804 / 0.817 s (x5.4)   0.111 -> 0.603 / 0.477 s (x5.4/x4.3)
  3,000 json dumps+loads round trips   0.065 -> 0.092 / 0.094 s (x1.4)     0.078 -> 0.073 / 0.075 s (noise)
  fib(24), 75,025 recursive calls      0.008 -> 0.051 / 0.082 s (x6.2/x10)  0.007 -> 0.046 / 0.071 s (x7/x11)
  (the first two rows are the worst case: code that does nothing but call tiny functions)
  real code: the 83 ROS-free tests of ros2_ws/src/main (pytest, WSL): 1.9-2.4 s -> 2.7-2.8 s (about +15-40 %)
  real program NOT ours: `mavproxy.py --help` (WSL): 184-193 ms plain; 191-193 ms with ASTRA_TRACE=1 (skipped
  by the default ASTRA_TRACE_SKIP); 327-356 ms when forced to be traced (ASTRA_TRACE_SKIP="").
  Python start-up (python3 -c pass, WSL, median of 21, trace folder on the Linux disk): 15.2 ms plain, 15.9 ms
  with the folder on PYTHONPATH and tracing off, 18.4 ms with tracing on. From /mnt/<drive> (the slow Windows
  disk seen from WSL) every start-up costs more (about 150 ms in an earlier measurement): always trace from
  the Linux-side copy (demo_traced.sh does). (Windows start-up times varied too much to quote.)
  For comparison only (NOT used, because sys.settrace fights with debuggers and coverage tools): the same
  filter as a sys.settrace function cost x5.2 on the tiny-call loop but only x1.8 on the len/abs loop.
  How it was measured: each loop timed with install(roots=[a temp folder]) off and on, the copy outside vs.
  inside that folder; ASTRA_TRACE_ADDR=127.0.0.1:9 (nobody listens: every datagram is simply dropped).
  Re-measured the same way after the whole profiler body went inside its try (2026-10-31, WSL): the first row
  0.060 -> 0.327 / 0.483 s, the main package's tests 1.92-1.94 s -> 2.49-2.55 s, start-up 14.9 / 16.0 / 18.2 ms,
  `mavproxy.py --help` skipped 195-225 ms, forced 318-343 ms: the same as above within the noise of the laptop.

Settings (environment variables)
--------------------------------
  ASTRA_TRACE=1            turn the tracer on (read by sitecustomize.py)
  ASTRA_TRACE_ADDR         where to send, HOST:PORT (default 127.0.0.1:8091; [IPv6]:PORT works too)
  ASTRA_TRACE_ROOTS        folders whose files count, separated by os.pathsep (":" on Linux, ";" on
                           Windows). When set, ONLY these count. Default: the repo this file lives in,
                           ~/ODLC_Machine_Inferencing_System_2025-2026, and any path that contains a folder
                           named ODLC_Machine_Inferencing_System_2025-2026 (covers the Windows checkout).
  ASTRA_TRACE_SKIP         programs that never get the profiler, separated by commas: the file name of the
                           program (sys.argv[0], "mavproxy.py"), the same without ".py", or the module of
                           "python -m <module>". Default: mavproxy.py,mavproxy. Why: MAVProxy is a busy
                           Python program that is NOT our code (it routes every MAVLink message between the
                           SITL autopilot, MAVROS and Mission Planner); sim.launch.py starts it with our
                           environment, so without this it would pay the profiler's cost (up to about 5 times
                           slower Python) for nothing. Set it to "" to trace every program.

Python 3.10+ (WSL has 3.10, which has no sys.monitoring, hence sys.setprofile). Standard library only.
"""

import os
import sys

try:
    import _thread
except ImportError:  # pragma: no cover - every CPython has _thread
    _thread = None

# ---- settings ------------------------------------------------------------------------------------------
#: The repo folder's name (its GitHub name). With the default roots, a file whose real path contains a folder
#: with this name counts as ours: that covers the Linux-side copy AND the Windows checkout without writing any
#: machine-specific path into the repo (the repo is public).
REPO_DIR_NAME = 'ODLC_Machine_Inferencing_System_2025-2026'
DEFAULT_ADDR = '127.0.0.1:8091'
FLUSH_SECONDS = 0.1             # at most one flush every 100 ms (the contract)
MAX_DATAGRAM_BYTES = 60000      # UDP allows 65507; stay well below
MAX_DATAGRAMS_PER_FLUSH = 8     # rate limit in bytes too: at most ~480 kB per flush, normally one tiny datagram
MAX_SKIPPED_FILES = 200000      # memory guard for programs that compile code at run time (exec, templates)
#: Programs that never get the profiler unless ASTRA_TRACE_SKIP says otherwise (see "Settings" above).
DEFAULT_SKIP = 'mavproxy.py,mavproxy'

_HERE = os.path.dirname(os.path.realpath(os.path.abspath(__file__)))


def _repo_of_tracer(here):
    """<repo>/simulation/codemap/trace -> <repo>. None when this folder does not sit at that place: a copy of
    the tracer somewhere else (say /tmp/trace) must never make "/" or another big folder a root, which would
    count every file of the computer as team code."""
    parts = here.replace('\\', '/').rstrip('/').split('/')
    if [p.lower() for p in parts[-3:]] != ['simulation', 'codemap', 'trace']:
        return None
    repo = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    if not repo or os.path.dirname(repo) == repo:      # "/" or "C:\": never a root
        return None
    return repo


_REPO_OF_TRACER = _repo_of_tracer(_HERE)

# ---- state (module globals; the profiler reaches them through a closure for speed) -----------------------
_not_ours = set()   # co_filename strings decided "not our code" (the fast reject path)
_ours = {}          # co_filename -> real path, for files decided "our code"
_counts = {}        # code object -> calls so far; -1 = one of ours that is not a map function (<lambda> ...)
_meta = {}          # code object -> (module __name__, real path, first line, function name)
_sent = {}          # code object -> calls already reported
_state = {
    'installed': False,
    'profiler': None,
    'roots': (),        # normalised root prefixes ("/home/u/ODLC_.../"), see _norm()
    'exclude': (),      # normalised prefixes that never count (the code map tools)
    'segment': '',      # "/ODLC_Machine_Inferencing_System_2025-2026/" with the default roots, else ''
    'addr_text': DEFAULT_ADDR,
    'sock': None,       # the UDP socket, made by the first flush
    'addr': None,       # the resolved (host, port) tuple
    'bad_addr': False,  # True when ASTRA_TRACE_ADDR cannot be used: counts are then dropped
    'flusher': False,   # True once the background thread was started
    'lock': _thread.allocate_lock() if _thread else None,
    'proc': None,       # the process name sent with every datagram (computed at the first flush)
    'pid': None,
    'datagrams': 0,     # how many datagrams were sent (for stats())
    'dropped': 0,       # how many could not be sent
    'skipped': '',      # the program name that matched ASTRA_TRACE_SKIP ('' = this program is traced)
}


# ---- helpers -----------------------------------------------------------------------------------------------
def _norm(path):
    """A path in the one spelling used for every comparison: real path, normal case, "/" separators."""
    return os.path.normcase(os.path.realpath(path)).replace('\\', '/')


def _as_prefix(path):
    """_norm(path) with exactly one "/" at the end, so "/a/b" never matches "/a/bc/x.py"."""
    return _norm(path).rstrip('/') + '/'


def _default_roots():
    """The default folders: the repo this file lives in and the Linux-side copy in the home folder."""
    roots = [os.path.join(os.path.expanduser('~'), REPO_DIR_NAME)]
    if _REPO_OF_TRACER:
        roots.insert(0, _REPO_OF_TRACER)
    seen, out = set(), []
    for r in roots:
        p = _as_prefix(r)
        if p not in seen:
            seen.add(p)
            out.append(p)
    return tuple(out)


def _program_names():
    """The names this program goes by, for ASTRA_TRACE_SKIP: the file name of sys.argv[0] ("mavproxy.py"),
    or, for "python -m <module>", the module. Python has already set sys.argv when sitecustomize runs, but for
    -m it holds only "-m" (the module is not read yet), so on Linux the module is read from /proc/self/cmdline.
    """
    names = []
    argv = getattr(sys, 'argv', None) or []
    first = argv[0] if argv and isinstance(argv[0], str) else ''
    if first == '-m':
        try:
            with open('/proc/self/cmdline', 'rb') as handle:
                words = handle.read().split(b'\0')
            names.append(words[words.index(b'-m') + 1].decode('utf-8', 'replace'))
        except Exception:                               # not Linux, or an odd command line: no name
            pass
    elif first and first != '-c':
        names.append(os.path.basename(first))
    return names


def _skip_match(skip_text=None):
    """The program name that is listed in ASTRA_TRACE_SKIP (default DEFAULT_SKIP), or '' when none is."""
    text = os.environ.get('ASTRA_TRACE_SKIP') if skip_text is None else skip_text
    if text is None:
        text = DEFAULT_SKIP
    skip = {word.strip().lower() for word in text.split(',') if word.strip()}
    if not skip:
        return ''
    for name in _program_names():
        low = name.lower()
        if low in skip or (low.endswith('.py') and low[:-3] in skip):
            return name
    return ''


def _decide(filename):
    """Return the real path when the file named in a code object is OUR code, else None (called once per file)."""
    if not filename or filename[0] == '<':          # "<frozen importlib._bootstrap>", "<string>", "<stdin>" ...
        return None
    real = os.path.realpath(filename)               # resolves colcon --symlink-install links to ros2_ws/src
    key = os.path.normcase(real).replace('\\', '/')
    for prefix in _state['exclude']:
        if key.startswith(prefix):
            return None
    if '/simulation/codemap/' in key and (_state['segment'] and _state['segment'] in key):
        return None                                  # the code map tools of any checkout never count
    for prefix in _state['roots']:
        if key.startswith(prefix):
            return real
    segment = _state['segment']
    if segment and segment in key:
        return real
    return None


def _first_call(frame, code):
    """Slow path, run once per code object: is it ours, and what do we report for it? It catches every Exception
    (a surprise in an odd file name must never reach the program; see "Known limits" for the price)."""
    try:
        filename = code.co_filename
        real = _ours.get(filename)
        if real is None:
            if filename in _not_ours:
                return
            real = _decide(filename)
            if real is None:
                if len(_not_ours) > MAX_SKIPPED_FILES:
                    _not_ours.clear()
                _not_ours.add(filename)
                return
            _ours[filename] = real
            _start_flusher()
        name = code.co_name
        if name[:1] == '<':
            # "<module>" (a file's top level, runs once at import), "<lambda>", "<listcomp>", "<genexpr>" ...:
            # not functions of the map. Remember them as -1 so this slow path is not taken again.
            _counts[code] = -1
            return
        module = frame.f_globals.get('__name__')
        _meta[code] = (module if isinstance(module, str) else '', real, code.co_firstlineno, name)
        _counts[code] = 1
    except Exception:
        pass


#: The only errors the profiler can cause ITSELF. It catches these and nothing else (see "Safety rules").
_OWN_ERRORS = (RecursionError, MemoryError)


def _make_profiler(not_ours, counts, first_call):
    """Build the profiler function. The containers are closure variables: the fastest look-up Python has."""
    own_errors = _OWN_ERRORS

    def astra_profile(frame, event, arg):
        # Python calls this for "call", "return", "c_call", "c_return" and "c_exception". Only "call" (a Python
        # function starts) matters; everything else returns at once. This is the hot path: keep it tiny.
        # The WHOLE body is inside the try: when the program is already within 2 frames of Python's recursion
        # limit, even the comparison on the first line raises RecursionError (this function's own frame and the
        # comparison count towards the limit too). Catching it here lets the program go on untouched (that one
        # call is simply not counted). Only our own two errors are caught: an exception that the program's
        # signal handler raises while this runs must reach the program, so it is never caught here.
        try:
            if event != 'call':
                return
            code = frame.f_code
            if code.co_filename in not_ours:        # most calls end here: not our code
                return
            n = counts.get(code)
            if n is None:
                first_call(frame, code)             # catches every Exception itself (the slow path)
            elif n >= 0:
                counts[code] = n + 1
        except own_errors:
            pass

    return astra_profile


# ---- sending -------------------------------------------------------------------------------------------------
def _parse_addr(text):
    """'HOST:PORT' or '[IPv6]:PORT' -> (host, port). Raises ValueError when it is not usable."""
    text = (text or '').strip()
    if text.startswith('['):
        host, _, rest = text[1:].partition(']')
        port_text = rest[1:] if rest.startswith(':') else ''
    else:
        host, _, port_text = text.rpartition(':')
    port = int(port_text)
    if not host or not 0 < port < 65536:
        raise ValueError('bad address %r' % text)
    return host, port


def _is_numeric_host(host):
    """True for an IP address written as numbers (no name look-up needed)."""
    import socket
    for family in (socket.AF_INET, socket.AF_INET6):
        try:
            socket.inet_pton(family, host)
            return True
        except (OSError, ValueError, AttributeError):
            continue
    return False


def _get_socket(allow_lookup=True):
    """The non-blocking UDP socket and target address, made on first use. (None, None) when unusable."""
    if _state['sock'] is not None:
        return _state['sock'], _state['addr']
    if _state['bad_addr']:
        return None, None
    import socket
    try:
        host, port = _parse_addr(_state['addr_text'])
        if not allow_lookup and not (_is_numeric_host(host) or host == 'localhost'):
            return None, None                         # at exit: never wait for a DNS answer
        family = socket.AF_INET6 if ':' in host else socket.AF_INET
        infos = socket.getaddrinfo(host, port, family, socket.SOCK_DGRAM)
        addr = infos[0][4]
        sock = socket.socket(family, socket.SOCK_DGRAM)
        sock.setblocking(False)                       # never block the program
    except Exception:
        _state['bad_addr'] = True                     # e.g. "nonsense", port 0, unknown host: drop counts
        return None, None
    _state['sock'], _state['addr'] = sock, addr
    return sock, addr


def _proc_name():
    """A readable process name: the program's file name, plus the ROS node name when one is given."""
    name = _state['proc']
    if name:
        return name
    argv = list(getattr(sys, 'argv', None) or [])
    first = argv[0] if argv else ''
    if first in ('', '-c'):
        base = 'python -c' if first == '-c' else 'python'
    elif first == '-m':
        base = 'python -m'
    else:
        base = os.path.basename(first) or first
        if base == '__main__.py':                       # python3 -m pytest -> ".../pytest/__main__.py"
            base = os.path.basename(os.path.dirname(first)) or base
    node = ''
    for a in argv[1:]:
        if isinstance(a, str) and a.startswith('__node:='):   # ROS 2: "--ros-args -r __node:=siyi"
            node = a[len('__node:='):]
    name = '%s (node %s)' % (base, node) if node and node != base else base
    _state['proc'] = name
    return name


def _send(pending, allow_lookup=True):
    """Send [(code, total, entry), ...] as one or more datagrams; remember what was reported."""
    import json
    sock, addr = _get_socket(allow_lookup)
    if sock is None:
        for code, total, _entry in pending:            # nowhere to send: drop (never pile up)
            _sent[code] = total
        return
    pid = os.getpid()
    head = '{"pid":%d,"proc":%s,"calls":[' % (pid, json.dumps(_proc_name()))
    budget = MAX_DATAGRAM_BYTES - len(head) - 2
    chunk, size, datagrams = [], 0, 0

    def emit(items):
        data = (head + ','.join(text for _c, _t, text in items) + ']}').encode('ascii')
        try:
            sock.sendto(data, addr)
            _state['datagrams'] += 1
        except Exception:                               # busy (BlockingIOError), no listener, network gone ...
            _state['dropped'] += 1
        for code, total, _text in items:               # sent or dropped: reported either way
            _sent[code] = total

    for code, total, entry in pending:
        text = json.dumps(entry, separators=(',', ':'))   # ensure_ascii: 1 character = 1 byte
        if chunk and size + len(text) + 1 > budget:
            emit(chunk)
            datagrams += 1
            chunk, size = [], 0
            if datagrams >= MAX_DATAGRAMS_PER_FLUSH:
                return                                    # the rest stays pending for the next flush
        chunk.append((code, total, text))
        size += len(text) + 1
    if chunk:
        emit(chunk)


def _flush(wait=0.0, at_exit=False):
    """Send the calls counted since the last flush. Catches every Exception; never waits longer than `wait` seconds
    (for a flush that is running in another thread) and never waits for the network."""
    lock = _state['lock']
    if lock is None:
        return
    try:
        got = lock.acquire(True, wait) if wait > 0 else lock.acquire(False)
    except Exception:
        return
    if not got:
        return
    try:
        snapshot = _counts.copy()                       # one C-level copy: safe while other threads count
        pending = []
        for code, total in snapshot.items():
            if total > 0:
                before = _sent.get(code, 0)
                if total > before:
                    meta = _meta.get(code)
                    if meta is not None:
                        pending.append((code, total, [meta[0], meta[1], meta[2], meta[3], total - before]))
        if pending:
            _send(pending, allow_lookup=not at_exit)
    except Exception:
        pass
    finally:
        lock.release()


def _flush_loop():
    """The background thread: flush every FLUSH_SECONDS, forever (it dies with the program)."""
    import time
    while True:
        try:
            time.sleep(FLUSH_SECONDS)
            _flush()
        except Exception:
            pass


def _start_flusher():
    """Start the background thread once (only programs that run our code ever get one)."""
    if _state['flusher'] or _thread is None:
        return
    _state['flusher'] = True
    try:
        # A raw _thread (not threading.Thread): it starts at once, never waits, is not joined at exit, and does
        # not inherit the profiler (threading.setprofile only reaches threading.Thread), so it never counts itself.
        _thread.start_new_thread(_flush_loop, ())
    except Exception:
        _state['flusher'] = False


def _flush_at_exit():
    """atexit: send what the last 100 ms counted (short programs would report nothing otherwise)."""
    if _state['flusher']:
        _flush(wait=0.25, at_exit=True)


def _after_fork_in_child():
    """os.fork() without exec: the child starts fresh (new lock, socket, pid, name) and reports only its own calls."""
    try:
        _state['lock'] = _thread.allocate_lock()
        _state['sock'] = None
        _state['proc'] = None
        _state['flusher'] = False
        _sent.clear()
        _sent.update((code, n) for code, n in _counts.items() if n > 0)   # the parent reports these
        if _ours:
            _start_flusher()
    except Exception:
        pass


# ---- public functions ------------------------------------------------------------------------------------
def install(roots=None, addr=None, skip=None):
    """Start tracing this process. Returns True when the profiler is installed, False otherwise (it catches every
    Exception and then returns False).

    roots: folders whose files count (default: ASTRA_TRACE_ROOTS, else the default roots, see the top).
    addr:  'HOST:PORT' to send to (default: ASTRA_TRACE_ADDR, else 127.0.0.1:8091).
    skip:  comma-separated program names that are never traced (default: ASTRA_TRACE_SKIP, else
           DEFAULT_SKIP). When this program is one of them, nothing is installed and False is returned.
    """
    try:
        if _state['installed']:
            return True
        _state['skipped'] = _skip_match(skip)
        if _state['skipped']:
            return False                               # e.g. MAVProxy: not our code, keep it at full speed
        if roots is None:
            env_roots = os.environ.get('ASTRA_TRACE_ROOTS', '').strip()
            roots = [r for r in env_roots.split(os.pathsep) if r.strip()] if env_roots else None
        if roots:
            _state['roots'] = tuple(_as_prefix(r.strip()) for r in roots)
            _state['segment'] = ''
        else:
            _state['roots'] = _default_roots()
            # the same spelling as _norm(): normal case AND "/" (normcase turns "/" into "\" on Windows)
            _state['segment'] = os.path.normcase('/' + REPO_DIR_NAME + '/').replace('\\', '/')
        exclude = {_as_prefix(_HERE)}                  # this tracer itself
        for r in _state['roots']:
            exclude.add(r + 'simulation/codemap/')     # the code map tools (extractor, server, tracer)
        _state['exclude'] = tuple(sorted(exclude))
        _state['addr_text'] = addr or os.environ.get('ASTRA_TRACE_ADDR', '').strip() or DEFAULT_ADDR

        profiler = _make_profiler(_not_ours, _counts, _first_call)
        _state['profiler'] = profiler
        import threading                               # only when tracing is on
        threading.setprofile(profiler)                 # every thread started from now on
        sys.setprofile(profiler)                       # this (the main) thread
        import atexit
        atexit.register(_flush_at_exit)
        if hasattr(os, 'register_at_fork'):
            os.register_at_fork(after_in_child=_after_fork_in_child)
        _state['installed'] = True
        return True
    except Exception:
        return False


def uninstall():
    """Stop tracing new calls in this thread and in threads started later (tests and benchmarks use it)."""
    try:
        import threading
        threading.setprofile(None)
        sys.setprofile(None)
        _state['installed'] = False
    except Exception:
        pass


def flush_now():
    """Send the pending counts right away (tests and tools; waits at most 0.5 s for a running flush)."""
    _flush(wait=0.5)


def stats():
    """A small dictionary that says what the tracer has seen (for tests and debugging)."""
    return {
        'installed': _state['installed'],
        'skipped_program': _state['skipped'],
        'roots': list(_state['roots']),
        'files_ours': len(_ours),
        'files_skipped': len(_not_ours),
        'functions_counted': sum(1 for n in _counts.values() if n > 0),
        'calls_counted': sum(n for n in _counts.values() if n > 0),
        'datagrams_sent': _state['datagrams'],
        'datagrams_dropped': _state['dropped'],
        'bad_addr': _state['bad_addr'],
        'flusher_started': _state['flusher'],
    }
