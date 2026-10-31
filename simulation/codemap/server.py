#!/usr/bin/env python3
"""server.py -- PURPOSE: the localhost collector and web server of the ASTRA live code map.

What it is for
--------------
The code map page (web/index.html) shows every system, package, file and function of the team code as
connected blocks. This small server makes that page LIVE while the simulator runs:
  * it LISTENS for the tracers' messages on UDP 127.0.0.1:8091. Every traced Python program
    (trace/astra_trace.py) and every traced simulator bash script (trace/astra_trace.sh) sends "these functions
    just ran" at most every 100 ms;
  * it TRANSLATES each message into the code map's ids (fn:<file>::<function>) with the trace_index part of
    codemap.json: first by the Python module name (video_cam.camera_interface -> its file), else by the end of
    the file path (/home/u/ODLC.../ros2_ws/src/main/src/main_controller.py -> ros2_ws/src/main/src/main_controller.py),
    then by the line where the function starts (when that file was edited after extract.py ran, so the lines
    moved, by the function's name if exactly one function of that file has it). Messages it cannot place are
    counted and listed at /health (files colcon/rosidl generated are counted apart: they are never in the map);
  * every 200 ms it SENDS the new calls to the open pages (Server-Sent Events, "event: calls"), and every 2 s
    the list of running programs it recognises ("event: procs"): our scripts and ROS entry points (mod:...) and
    the outside programs (ext:gazebo for "gz sim", ext:ardupilot_sitl for arducopter, ext:mavproxy,
    ext:mavros for mavros_node, ext:mission_planner for MissionPlanner.exe, asked through Windows'
    tasklist.exe when running inside WSL);
  * with --record PATH it WRITES every calls/procs event as one JSON line, so a run can be replayed later
    (GET /recordings) and embedded in the shareable artifact (build_artifact.py). When it stops it also saves
    its last /health answer next to the recording (<stamp>.health.json: the calls it could not place).
  * GET /recordings lists the recordings of two folders by default: recordings/ next to this file (where
    demo_traced.sh records when it runs from a git checkout, e.g. the Windows one) and
    ~/.astra_sim/codemap/recordings (where it records when it runs from the Linux-side copy, which has no .git;
    $ASTRA_SIM_HOME/codemap/recordings when ASTRA_SIM_HOME is set). --recordings DIR lists only DIR instead.
    The folder of the current --record file is always listed too.

Endpoints (http://127.0.0.1:8090)
---------------------------------
  GET /                    the explorer page (web/index.html; a tiny built-in page when it is missing)
  GET /codemap.json        the static code map (re-read automatically when extract.py rewrites it)
  GET /events              Server-Sent Events:
                             event: calls   data: {"t": <epoch seconds>, "calls": {"<fn id>": <count>, ...}}
                             event: procs   data: {"t": <epoch seconds>, "running": ["mod:...", "ext:...", ...]}
                           (at most 8 pages at once; the newest procs list is sent right after connecting)
  GET /recordings          JSON list, newest first: [{"name", "bytes", "modified", "url"}, ...]
  GET /recordings/<file>   one recording: JSON lines {"event": "calls"|"procs", "t": ..., ...same data...}
  GET /health              JSON: is it alive, what came in, which messages could not be placed (and why to care)

How to run (standard library only; Python 3.10+; inside WSL, where the simulator runs -- a browser on Windows
reaches it at the same address because WSL forwards localhost):
    python3 simulation/codemap/server.py                       # then open http://127.0.0.1:8090/
    python3 simulation/codemap/server.py --record auto         # also record into recordings/<stamp>.jsonl
  Options: --port 8090 --udp-port 8091 --host 127.0.0.1 --udp-host 127.0.0.1 --codemap PATH --web DIR
           --record PATH|DIR|auto --recordings DIR --procs-interval SECONDS (0 = off) --no-windows --verbose
  The simplest way is simulation/scripts/demo_traced.sh, which starts this server, runs the demo with the
  tracers switched on, and stops the server at the end.
"""

from __future__ import annotations

import argparse
import datetime
import gzip
import json
import mimetypes
import os
import queue
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import quote, unquote, urlsplit

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent

DEFAULT_HOST = '127.0.0.1'
DEFAULT_PORT = 8090
DEFAULT_UDP_PORT = 8091
BATCH_SECONDS = 0.2           # "event: calls" at most every 200 ms
PROCS_SECONDS = 2.0           # "event: procs" every 2 s
MISSION_PLANNER_SECONDS = 6.0  # asking Windows (tasklist.exe) is slower: every 6 s
KEEPALIVE_SECONDS = 15.0      # an SSE comment line now and then keeps idle connections open
MAX_CLIENTS = 8
MAX_UNKNOWN = 5000            # memory guard for the "could not place" lists
UNKNOWN_SHOWN = 50

# Programs that start another program or script named on their command line (looked through when matching).
_INTERPRETERS = re.compile(r'^(python[0-9.]*|pypy[0-9.]*|bash|sh|dash|zsh|env|setsid|nice|nohup|stdbuf|'
                            r'timeout|exec|xvfb-run|ruby)$')
_INSTALL_LAYOUT = re.compile(r'/(?:install|build)/([^/]+)/')
_CONSOLE_SCRIPT = re.compile(r'''["']\s*([A-Za-z0-9_.\-]+)\s*=\s*([A-Za-z0-9_.]+)\s*:\s*([A-Za-z0-9_.]+)\s*["']''')
_PACKAGE_NAME = re.compile(r'<name>\s*([^<\s]+)\s*</name>')
_RECORDING_NAME = re.compile(r'^[A-Za-z0-9_.\-]+\.jsonl$')


def _basename(path: str) -> str:
    """The last part of a path written with / or \\."""
    return str(path).replace('\\', '/').rstrip('/').rsplit('/', 1)[-1]


def _is_generated(path: str) -> bool:
    """True for files colcon or rosidl MADE inside ros2_ws/install or ros2_ws/build (setup helpers such as
    install/_local_setup_util_sh.py and install/setup.bash, generated message classes). The tracers report them
    because they sit inside the repo folder, but they are not source files, so the map never lists them.
    (Our own files that colcon only LINKED there are reported at their real place, ros2_ws/src/..., by the
    tracer, so they never reach this test.)"""
    norm = '/' + str(path).replace('\\', '/').lstrip('/')
    return '/ros2_ws/install/' in norm or '/ros2_ws/build/' in norm


def _now_iso(epoch: float | None = None) -> str:
    """A readable local time stamp (for /health and /recordings)."""
    return datetime.datetime.fromtimestamp(epoch if epoch is not None else time.time()).isoformat(
        timespec='seconds')


def sim_home_recordings() -> Path | None:
    """The folder where simulation/scripts/demo_traced.sh records runs started from the Linux-side copy:
    $ASTRA_SIM_HOME/codemap/recordings, by default ~/.astra_sim/codemap/recordings (the same rule as the script's
    SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"). None when no home folder can be found. The folder may not exist
    yet (no such run so far); listing it then simply finds nothing."""
    try:
        home = os.environ.get('ASTRA_SIM_HOME') or str(Path.home() / '.astra_sim')
        return (Path(home).expanduser() / 'codemap' / 'recordings').resolve()
    except (OSError, RuntimeError, ValueError):
        return None


# =============================================================================================================
#  The code map index: everything needed to turn raw tracer messages and command lines into code map ids
# =============================================================================================================
class CodeMapIndex:
    """Look-up tables built from codemap.json (and, for entry points, the package files of the repo)."""

    def __init__(self, data: dict | None, repo_root: Path | None = None):
        data = data if isinstance(data, dict) else {}
        self.data = data
        nodes = [n for n in (data.get('nodes') or []) if isinstance(n, dict) and isinstance(n.get('id'), str)]
        trace = data.get('trace_index') if isinstance(data.get('trace_index'), dict) else {}
        self.py_modules: dict[str, str] = {k: v for k, v in (trace.get('py_modules') or {}).items()
                                           if isinstance(k, str) and isinstance(v, str)}
        self.py_files: dict[str, str] = {k.replace('\\', '/'): v for k, v in (trace.get('py_files') or {}).items()
                                         if isinstance(k, str) and isinstance(v, str)}
        self.functions: dict[str, dict[str, str]] = {}
        for mod, lines in (trace.get('functions') or {}).items():
            if isinstance(lines, dict):
                self.functions[mod] = {str(k): v for k, v in lines.items() if isinstance(v, str)}
        self.node_ids: set[str] = {n['id'] for n in nodes}

        # module id -> repo-relative path ("ros2_ws/src/main/src/main_controller.py")
        self.mod_paths: dict[str, str] = {}
        for n in nodes:
            if n.get('kind') == 'module' or n['id'].startswith('mod:'):
                path = n.get('path') if isinstance(n.get('path'), str) else n['id'][4:]
                path = path.replace('\\', '/')
                self.mod_paths[n['id']] = path[2:] if path.startswith('./') else path
        for path, mod in self.py_files.items():
            self.mod_paths.setdefault(mod, path)

        # Without a trace_index (an old or hand-made codemap): use the function nodes' own "line".
        if not self.functions:
            nodes_by_id = {m['id']: m for m in nodes}
            for n in nodes:
                if n.get('kind') in ('function', 'method') and isinstance(n.get('line'), int):
                    mod = self._module_of(n, nodes_by_id)
                    if mod:
                        self.functions.setdefault(mod, {})[str(n['line'])] = n['id']
        self.functions_indexed = sum(len(v) for v in self.functions.values())
        # module id -> {short function name: [fn ids]}: the fall-back when a file was edited AFTER extract.py ran
        # (other lines, same names). "fn:x.py::Camera.take_photo" -> "take_photo". Used only when the name is
        # unique in that file, so it can never pick the wrong one of two functions with the same name.
        self.names: dict[str, dict[str, list[str]]] = {}
        for mod, lines in self.functions.items():
            for fid in lines.values():
                short = fid.rsplit('::', 1)[-1].rsplit('.', 1)[-1]
                self.names.setdefault(mod, {}).setdefault(short, []).append(fid)
        self.by_name_hits = 0         # how many look-ups needed the fall-back (a sign codemap.json is out of date)

        # Every path under which a module may show up -> module id, grouped by file name for the suffix match:
        # the module's own path, plus the extractor's extra spellings: copies colcon makes (py_files with
        # ros2_ws/build/... or .../site-packages/...), installed programs (programs: "ros2_ws/install/video_cam/
        # lib/video_cam/siyi" -> the module whose main() it runs) and shell scripts (sh_files).
        self.by_basename: dict[str, list[tuple[str, str]]] = {}
        seen_paths: set[tuple[str, str]] = set()
        extra = []
        for table in ('py_files', 'programs', 'sh_files'):
            value = trace.get(table)
            if isinstance(value, dict):
                extra += [(k.replace('\\', '/'), v) for k, v in value.items()
                          if isinstance(k, str) and isinstance(v, str)]
        own = [(path, mod) for mod, path in self.mod_paths.items()]
        for path, mod in own + extra:
            if (path, mod) not in seen_paths:
                seen_paths.add((path, mod))
                self.by_basename.setdefault(_basename(path), []).append((path, mod))
        self.programs: dict[tuple[str, str], str] = {}     # (package, executable) -> module id
        for path, mod in extra:
            m = re.search(r'(?:^|/)install/([^/]+)/lib/([^/]+)/([^/]+)$', path)
            if m:
                self.programs[(m.group(1), m.group(3))] = mod

        # Programs that are not ours: the extractor's "process_match" patterns of each external node, matched
        # against the first words of a command line; built-in rules cover the contract's ids when it has none.
        self.ext_patterns: list[tuple[str, re.Pattern]] = []
        for n in nodes:
            if n.get('kind') == 'external' and isinstance(n.get('process_match'), list):
                for pattern in n['process_match']:
                    try:
                        self.ext_patterns.append((n['id'], re.compile(str(pattern))))
                    except re.error:
                        continue

        self.pkg_dirs: dict[str, set[str]] = {}           # ROS package name -> repo-relative folders
        for n in nodes:
            if n.get('kind') == 'package' and n['id'].startswith('pkg:'):
                folder = (n.get('path') if isinstance(n.get('path'), str) else n['id'][4:]).replace('\\', '/')
                self.pkg_dirs.setdefault(_basename(folder), set()).add(folder.strip('/'))
        self.entry_points: dict[tuple[str, str], set[str]] = {}   # (package or '', executable) -> module ids
        self._entry_points_from_nodes(nodes)
        if repo_root is not None:
            self._scan_repo(Path(repo_root))
        self._py_cache: dict[tuple, str | None] = {}
        self._file_cache: dict[str, str | None] = {}
        self._cmd_cache: dict[tuple, frozenset] = {}

    # ---- building -------------------------------------------------------------------------------------------
    @staticmethod
    def _module_of(node: dict, nodes_by_id: dict) -> str | None:
        """Walk up the "parent" links of a function node until a module is found."""
        seen = 0
        cur = node
        while isinstance(cur, dict) and seen < 20:
            parent = cur.get('parent')
            if not isinstance(parent, str):
                return None
            if parent.startswith('mod:'):
                return parent
            cur = nodes_by_id.get(parent)
            seen += 1
        return None

    def _add_entry_point(self, package: str, exe: str, mod: str) -> None:
        if exe and mod:
            self.entry_points.setdefault((package or '', exe), set()).add(mod)
            self.entry_points.setdefault(('', exe), set()).add(mod)

    def _entry_points_from_nodes(self, nodes: list[dict]) -> None:
        """Nodes may say how they are started ("entry_point").

        A function node (the extractor's way) holds a command: "ros2 run detection new_od" on
        fn:ros2_ws/src/detection/detection/new_od.py::main means `ros2 run detection new_od` runs that file.
        A module node may hold a program name, a list or dict of names, or true (= its own file name).
        """
        for n in nodes:
            value = n.get('entry_point')
            if n['id'].startswith('fn:') and isinstance(value, str):
                words = value.split()
                mod = 'mod:' + n['id'][3:].split('::', 1)[0]
                if len(words) >= 4 and _basename(words[0]) == 'ros2' and words[1] == 'run':
                    self._add_entry_point(words[2], words[3], mod)
                continue
            if not n['id'].startswith('mod:') or 'entry_point' not in n:
                continue
            names: list[str] = []
            if value is True:
                names = [_basename(self.mod_paths.get(n['id'], n['id']))]
            elif isinstance(value, str):
                names = [value]
            elif isinstance(value, list):
                names = [v for v in value if isinstance(v, str)]
            elif isinstance(value, dict):
                names = [k for k in value if isinstance(k, str)]
            for name in names:
                name = name.split('=', 1)[0].strip()
                package = ''
                if '/' in name:                                   # "video_cam/siyi"
                    package, name = name.rsplit('/', 1)
                if re.match(r'^[A-Za-z0-9_.\-]+$', name):
                    self._add_entry_point(package, name, n['id'])

    def _scan_repo(self, repo: Path) -> None:
        """Read package.xml names and setup.py console_scripts (ros2 run <package> <executable> -> module)."""
        src = repo / 'ros2_ws' / 'src'
        if not src.is_dir():
            return
        for pattern in ('*/package.xml', '*/*/package.xml', '*/*/*/package.xml'):
            for xml in src.glob(pattern):
                try:
                    text = xml.read_text(encoding='utf-8', errors='replace')
                except OSError:
                    continue
                m = _PACKAGE_NAME.search(text)
                if not m:
                    continue
                folder = xml.parent.relative_to(repo).as_posix()
                self.pkg_dirs.setdefault(m.group(1), set()).add(folder)
                setup = xml.parent / 'setup.py'
                if setup.is_file():
                    try:
                        setup_text = setup.read_text(encoding='utf-8', errors='replace')
                    except OSError:
                        continue
                    for exe, module, _func in _CONSOLE_SCRIPT.findall(setup_text):
                        mod = self.py_modules.get(module)
                        if mod is None:      # not in the trace index: try <package folder>/<module path>.py
                            mod = self._mod_for_path(folder + '/' + module.replace('.', '/') + '.py')
                        if mod:
                            self._add_entry_point(m.group(1), exe, mod)

    def _mod_for_path(self, repo_path: str) -> str | None:
        for path, mod in self.by_basename.get(_basename(repo_path), []):
            if path == repo_path:
                return mod
        return None

    # ---- Python calls ---------------------------------------------------------------------------------------
    def map_py(self, module: str, filename: str, line: int, name: str = '') -> str | None:
        """[module __name__, file, first line, name] from the Python tracer -> fn id (None when unknown)."""
        key = (module, filename, line)
        if key in self._py_cache:
            return self._py_cache[key]
        fn = None
        mod = self._module_by_name(module, filename)
        if mod is not None:
            fn = self.functions.get(mod, {}).get(str(line))
        mod2 = None
        if fn is None:
            mod2 = self.file_to_mod(filename)
            if mod2 is not None and mod2 != mod:
                fn = self.functions.get(mod2, {}).get(str(line))
        if fn is None and name and not name.startswith('<'):
            # The file changed after extract.py ran, so its lines moved: accept the function by its name when
            # exactly one function of that file has it (never for <lambda>, <listcomp>, <module> ...).
            for candidate_mod in (mod, mod2):
                same = self.names.get(candidate_mod, {}).get(name, []) if candidate_mod else []
                if len(set(same)) == 1:
                    fn = same[0]
                    self.by_name_hits += 1
                    break
        if len(self._py_cache) > 50000:
            self._py_cache.clear()
        self._py_cache[key] = fn
        return fn

    def _module_by_name(self, module: str, filename: str) -> str | None:
        """By dotted module name, but only when the file name agrees (two packages may reuse a short name)."""
        if not module or module == '__main__':
            return None
        mod = self.py_modules.get(module)
        if mod is None:
            return None
        path = self.mod_paths.get(mod, '')
        base = _basename(filename) if filename else ''
        if base and path and _basename(path) != base:
            return None
        return mod

    def file_to_mod(self, filename: str) -> str | None:
        """Absolute or relative file path (any computer, / or \\) -> module id, by the end of the path."""
        if not filename:
            return None
        if filename in self._file_cache:
            return self._file_cache[filename]
        norm = str(filename).replace('\\', '/')
        candidates = self.by_basename.get(_basename(norm), [])
        best: tuple[str, str] | None = None
        for path, mod in candidates:
            if norm == path or norm.endswith('/' + path):
                if best is None or len(path) > len(best[0]):
                    best = (path, mod)
        found = best[1] if best else None
        if found is None and candidates:
            # A copy made by colcon (without --symlink-install): .../install/<pkg>/lib/<pkg>/<file>,
            # .../build/<pkg>/<pkg>/<file> or .../install/<pkg>/lib/python3.10/site-packages/<pkg>/<file>.
            m = _INSTALL_LAYOUT.search(norm)
            if m:
                inside = [mod for path, mod in candidates if self._in_package(path, m.group(1))]
                if len(inside) == 1:
                    found = inside[0]
        if len(self._file_cache) > 50000:
            self._file_cache.clear()
        self._file_cache[filename] = found
        return found

    def _in_package(self, repo_path: str, package: str) -> bool:
        folders = self.pkg_dirs.get(package)
        if folders:
            return any(repo_path.startswith(f.rstrip('/') + '/') for f in folders)
        return ('/' + package + '/') in ('/' + repo_path)

    # ---- bash calls -----------------------------------------------------------------------------------------
    def map_sh(self, script: str, function: str) -> str | None:
        """("/home/u/ODLC.../simulation/scripts/_common.sh", "say") -> "fn:simulation/scripts/_common.sh::say"."""
        mod = self.file_to_mod(script)
        if mod is None:
            return None
        fid = 'fn:%s::%s' % (self.mod_paths.get(mod, mod[4:]), function)
        return fid if fid in self.node_ids else None

    # ---- running programs -----------------------------------------------------------------------------------
    def map_command(self, argv: list[str], cwd: str | None = None) -> frozenset:
        """One process's command line -> the code map ids it stands for (mod:..., ext:...)."""
        key = (tuple(argv), cwd)
        hit = self._cmd_cache.get(key)
        if hit is not None:
            return hit
        result = frozenset(self._map_command(list(argv), cwd))
        if len(self._cmd_cache) > 5000:
            self._cmd_cache.clear()
        self._cmd_cache[key] = result
        return result

    def _map_command(self, argv: list[str], cwd: str | None) -> set[str]:
        ids: set[str] = set()
        args = [a for a in argv if a]
        if len(args) == 1 and ' ' in args[0].strip():      # a renamed process title, e.g. "gz sim -s -r ..."
            args = args[0].split()
        if not args:
            return ids
        names = [_basename(a).lower() for a in args[:4]]
        lowered = [a.lower() for a in args[:5]]
        # -- the programs that are not ours: the codemap's process_match patterns, on the first three words only
        # (so an argument that merely mentions "arducopter" does not count)
        head = ' '.join(args[:3])
        for ext_id, pattern in self.ext_patterns:
            if pattern.search(head):
                ids.add(ext_id)
        # built-in rules for the contract's ids (an older codemap without process_match)
        if ('gz' in names[:2] and 'sim' in lowered[1:4]) or any(
                n.startswith(('gz-sim', 'gzserver', 'gzclient', 'ign-gazebo')) for n in names[:2]):
            ids.add('ext:gazebo')
        if names[0].startswith('arducopter') or (len(names) > 1 and names[0].startswith('python')
                                                 and names[1].startswith('arducopter')):
            ids.add('ext:ardupilot_sitl')
        if any('mavproxy' in n for n in names[:3]):
            ids.add('ext:mavproxy')
        if any(n == 'mavros_node' for n in names[:3]):
            ids.add('ext:mavros')
        if any('missionplanner' in n for n in names[:3]):
            ids.add('ext:mission_planner')
        if ids:
            return ids
        # -- ros2 launch <pkg> <file> / ros2 run <pkg> <exe>
        for i, n in enumerate(names[:3]):
            if n == 'ros2' and i + 1 < len(args) and args[i + 1] in ('launch', 'run'):
                rest = [a for a in args[i + 2:] if not a.startswith('-')]
                if args[i + 1] == 'launch' and rest:
                    mod = (self._mod_for_launch(rest[0], rest[1] if len(rest) > 1 else '', cwd))
                elif len(rest) >= 2:
                    mod = self._mod_for_executable(rest[0], rest[1])
                else:
                    mod = None
                if mod:
                    ids.add(mod)
                return ids
        # -- an interpreter running a file, or python -m <module>
        i = 0
        while i < len(args) and _INTERPRETERS.match(_basename(args[i]).lower()):
            interp = _basename(args[i]).lower()
            i += 1
            while i < len(args) and args[i].startswith('-'):
                opt = args[i]
                if interp.startswith(('python', 'pypy')) and opt == '-m' and i + 1 < len(args):
                    mod = self.py_modules.get(args[i + 1])
                    if mod:
                        ids.add(mod)
                    return ids
                if interp.startswith(('python', 'pypy')) and opt == '-c':
                    return ids
                if opt in ('-X', '-W', '-k', '-s', '--signal', '-n') and i + 1 < len(args):
                    i += 1                                  # an option with a value
                i += 1
            if interp in ('timeout',) and i < len(args) and re.match(r'^[0-9.]+[smhd]?$', args[i]):
                i += 1                                      # timeout DURATION command ...
        if i < len(args):
            mod = self._mod_for_program(args[i], cwd)
            if mod:
                ids.add(mod)
        return ids

    def _absolute(self, path: str, cwd: str | None) -> str:
        """Make a command-line path absolute (with the process's working folder) and resolve links when it exists."""
        p = path
        if not os.path.isabs(p) and not p.startswith('/') and cwd:
            p = os.path.join(cwd, p)
        try:
            if os.path.exists(p):
                p = os.path.realpath(p)
        except OSError:
            pass
        return p.replace('\\', '/')

    def _mod_for_program(self, path: str, cwd: str | None) -> str | None:
        """A program file on a command line (script, ROS executable) -> module id."""
        full = self._absolute(path, cwd)
        mod = self.file_to_mod(full) or (self.file_to_mod(path) if '/' in path else None)
        if mod:
            return mod
        m = re.search(r'/install/([^/]+)/lib/([^/]+)/([^/]+)$', full)
        if m:                                               # ROS executable: install/<pkg>/lib/<pkg>/<exe>
            return self._mod_for_executable(m.group(1), m.group(3))
        return None

    def _mod_for_executable(self, package: str, exe: str) -> str | None:
        """(ROS package, executable) as in `ros2 run <package> <exe>` -> module id."""
        if (package, exe) in self.programs:                 # the extractor's table of installed programs
            return self.programs[(package, exe)]
        mods = self.entry_points.get((package, exe)) or set()
        if not mods:
            mods = {m for m in self.entry_points.get(('', exe), set())
                    if self._in_package(self.mod_paths.get(m, ''), package)}
        if not mods:                                        # scripts installed under their own name
            for name in (exe, exe + '.py'):
                mods = {mod for path, mod in self.by_basename.get(name, []) if self._in_package(path, package)}
                if mods:
                    break
        return sorted(mods)[0] if mods else None

    def _mod_for_launch(self, package_or_path: str, file: str, cwd: str | None) -> str | None:
        """`ros2 launch <package> <file>` (or `ros2 launch <path>`) -> the launch file's module id."""
        if not file or '/' in package_or_path or package_or_path.endswith(('.py', '.xml', '.yaml')):
            return self._mod_for_program(package_or_path, cwd)
        candidates = [(path, mod) for path, mod in self.by_basename.get(_basename(file), [])
                      if self._in_package(path, package_or_path)]
        if not candidates:
            return None
        candidates.sort(key=lambda pm: (0 if '/launch/' in pm[0] else 1, len(pm[0])))
        return candidates[0][1]


# =============================================================================================================
#  Running programs
# =============================================================================================================
def list_processes() -> list[tuple[int, list[str], str | None]]:
    """[(pid, argv, folder to read the working folder from)] of the programs on this computer."""
    out: list[tuple[int, list[str], str | None]] = []
    if os.path.isdir('/proc') and os.path.exists('/proc/self/cmdline'):
        for name in os.listdir('/proc'):
            if not name.isdigit():
                continue
            try:
                with open('/proc/%s/cmdline' % name, 'rb') as handle:
                    raw = handle.read()
            except OSError:
                continue
            if raw:
                argv = [a.decode('utf-8', 'replace') for a in raw.split(b'\0') if a]
                out.append((int(name), argv, '/proc/%s/cwd' % name))
        return out
    if os.name == 'nt':
        return out            # the simulator never runs on Windows itself (Mission Planner is checked apart)
    try:
        text = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return out
    for row in text.splitlines():
        parts = row.strip().split()
        if len(parts) >= 2 and parts[0].isdigit():
            out.append((int(parts[0]), parts[1:], None))
    return out


class MissionPlannerWatch:
    """Is MissionPlanner.exe running on Windows? Asked through tasklist.exe (WSL interop) every 6 s."""

    def __init__(self, enabled: bool = True):
        self.exe = None
        if enabled:
            for candidate in ('tasklist.exe', 'tasklist'):
                self.exe = shutil.which(candidate)
                if self.exe:
                    break
            if not self.exe and os.path.exists('/mnt/c/Windows/System32/tasklist.exe'):
                self.exe = '/mnt/c/Windows/System32/tasklist.exe'
        self.state = 'off' if not enabled else ('not found' if not self.exe else 'not asked yet')
        self._last = 0.0
        self._running = False

    def running(self) -> bool:
        if not self.exe:
            return False
        now = time.monotonic()
        if now - self._last >= MISSION_PLANNER_SECONDS:
            self._last = now
            try:
                result = subprocess.run([self.exe, '/FI', 'IMAGENAME eq MissionPlanner.exe', '/FO', 'CSV', '/NH'],
                                        capture_output=True, timeout=8, stdin=subprocess.DEVNULL)
                self._running = b'missionplanner.exe' in result.stdout.lower()
                self.state = 'running' if self._running else 'not running'
            except (OSError, subprocess.SubprocessError) as error:
                self.state = 'error: %s' % error.__class__.__name__
                self._running = False
        return self._running


# =============================================================================================================
#  The hub: batches calls, fans events out to the open pages, writes the recording
# =============================================================================================================
class Hub:
    def __init__(self, record_path: Path | None = None, max_clients: int = MAX_CLIENTS):
        self.lock = threading.Lock()
        self.pending: dict[str, int] = {}
        self.clients: list[queue.Queue] = []
        self.max_clients = max_clients
        self.record_path = record_path
        self.record = None
        self.record_lock = threading.Lock()
        self.recorded = 0
        if record_path is not None:
            record_path.parent.mkdir(parents=True, exist_ok=True)
            # line-buffered: every event is on disk at once, even if the server is killed
            self.record = open(record_path, 'a', encoding='utf-8', buffering=1)
        self.latest_procs: dict = {'t': round(time.time(), 3), 'running': []}
        self.events_sent = {'calls': 0, 'procs': 0}
        self.slow_client_drops = 0

    def add_calls(self, calls: dict[str, int]) -> None:
        with self.lock:
            for fid, count in calls.items():
                self.pending[fid] = self.pending.get(fid, 0) + count

    def flush_calls(self) -> None:
        with self.lock:
            batch, self.pending = self.pending, {}
        if batch:
            self.publish('calls', {'t': round(time.time(), 3), 'calls': batch})

    def set_procs(self, running: list[str]) -> None:
        self.latest_procs = {'t': round(time.time(), 3), 'running': running}
        self.publish('procs', self.latest_procs)

    @staticmethod
    def sse_chunk(event: str, payload: dict) -> bytes:
        return ('event: %s\ndata: %s\n\n' % (event, json.dumps(payload, separators=(',', ':')))).encode('utf-8')

    def publish(self, event: str, payload: dict) -> None:
        chunk = self.sse_chunk(event, payload)
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(chunk)
            except queue.Full:
                self.slow_client_drops += 1            # a page that does not read: skip it, never wait
        self.events_sent[event] = self.events_sent.get(event, 0) + 1
        if self.record is not None:
            line = json.dumps(dict({'event': event}, **payload), separators=(',', ':'))
            with self.record_lock:
                try:
                    self.record.write(line + '\n')
                    self.recorded += 1
                except (OSError, ValueError):
                    pass

    def register(self) -> queue.Queue | None:
        with self.lock:
            if len(self.clients) >= self.max_clients:
                return None
            q: queue.Queue = queue.Queue(maxsize=512)
            self.clients.append(q)
            return q

    def unregister(self, q: queue.Queue) -> None:
        with self.lock:
            if q in self.clients:
                self.clients.remove(q)

    def close(self) -> None:
        with self.lock:
            clients = list(self.clients)
        for q in clients:
            try:
                q.put_nowait(None)                      # tells the page's handler thread to finish
            except queue.Full:
                pass
        with self.record_lock:
            if self.record is not None:
                try:
                    self.record.close()
                except OSError:
                    pass
                self.record = None


# =============================================================================================================
#  The application: code map loading, tracer messages, process watching, health
# =============================================================================================================
class App:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.started = time.time()
        self.stopping = threading.Event()
        self.codemap_path = Path(args.codemap).expanduser().resolve()
        self.web_dir = Path(args.web).expanduser().resolve()
        # /recordings: --recordings DIR alone, or by default recordings/ next to this file plus the Linux-side
        # copy's folder ~/.astra_sim/codemap/recordings (see sim_home_recordings)
        if args.recordings:
            self.recordings_dir = Path(args.recordings).expanduser().resolve()
            self.extra_recording_dirs: list[Path] = []
        else:
            self.recordings_dir = (HERE / 'recordings').resolve()
            home_dir = sim_home_recordings()
            self.extra_recording_dirs = [home_dir] if home_dir is not None else []
        self.record_path = self._record_path(args.record)
        self.hub = Hub(self.record_path)
        self.index = CodeMapIndex(None)
        self.codemap_bytes = b''
        self.codemap_mtime = None
        self.codemap_error = 'not loaded yet'
        self.gzip_cache: dict[tuple, bytes] = {}
        self.load_codemap()
        self.mission_planner = MissionPlannerWatch(enabled=not args.no_windows)
        self.procs_scan = 'proc' if os.path.isdir('/proc') else ('none' if os.name == 'nt' else 'ps')
        # what came in
        self.stats_lock = threading.Lock()
        self.datagrams = 0
        self.bad_datagrams = 0
        self.bytes_in = 0
        self.mapped_calls = 0
        self.unknown_calls = 0
        self.unknown_generated_calls = 0   # part of unknown_calls: colcon/rosidl-made files (never in the map)
        self.unknown_py: dict[tuple, int] = {}
        self.unknown_sh: dict[tuple, int] = {}
        self.tracers: dict[int, dict] = {}

    # ---- set-up ---------------------------------------------------------------------------------------------
    def _record_path(self, value: str | None) -> Path | None:
        if not value:
            return None
        stamp = time.strftime('%Y%m%d-%H%M%S')
        if value == 'auto':
            return self.recordings_dir / ('%s.jsonl' % stamp)
        path = Path(value).expanduser()
        if path.is_dir() or value.endswith(('/', os.sep)):
            return (path / ('%s.jsonl' % stamp)).resolve()
        return path.resolve()

    def load_codemap(self) -> bool:
        """(Re)read codemap.json when it is new or changed. Returns True when a new version was loaded."""
        try:
            mtime = self.codemap_path.stat().st_mtime
        except OSError:
            if self.codemap_mtime is not None or self.codemap_error == 'not loaded yet':
                self.codemap_error = 'file not found: %s (make it with simulation/codemap/extract.py)' % (
                    self.codemap_path)
            return False
        if mtime == self.codemap_mtime:
            return False
        try:
            raw = self.codemap_path.read_bytes()
            data = json.loads(raw.decode('utf-8'))
            index = CodeMapIndex(data, repo_root=REPO)
        except (OSError, ValueError) as error:
            self.codemap_error = 'cannot read %s: %s' % (self.codemap_path, error)
            self.codemap_mtime = mtime                  # do not retry until the file changes again
            return False
        self.index, self.codemap_bytes, self.codemap_mtime = index, raw, mtime
        self.codemap_error = None
        self.gzip_cache.clear()
        return True

    # ---- tracer messages ------------------------------------------------------------------------------------
    def handle_datagram(self, data: bytes) -> None:
        """One UDP message from a tracer -> counts per fn id (added to the hub) or 'could not place' entries."""
        with self.stats_lock:
            self.datagrams += 1
            self.bytes_in += len(data)
        try:
            msg = json.loads(data.decode('utf-8'))
            calls = msg['calls']
            if not isinstance(calls, list):
                raise ValueError('calls is not a list')
        except (ValueError, KeyError, TypeError, UnicodeDecodeError):
            with self.stats_lock:
                self.bad_datagrams += 1
            return
        index = self.index
        mapped: dict[str, int] = {}
        unknown = 0
        generated = 0
        is_bash = 'sh' in msg
        script = str(msg.get('sh') or '')
        for item in calls:
            try:
                if is_bash:
                    name, count = str(item[0]), int(item[1])
                    fid = index.map_sh(script, name)
                    key = (script, name)
                else:
                    module, filename, line, name, count = item[0], item[1], int(item[2]), item[3], int(item[4])
                    module = module if isinstance(module, str) else ''
                    filename = filename if isinstance(filename, str) else ''
                    name = name if isinstance(name, str) else ''
                    fid = index.map_py(module, filename, line, name)
                    key = (module, filename, line, name)
            except (ValueError, TypeError, IndexError):
                continue
            if count <= 0:
                continue
            if fid is not None:
                mapped[fid] = mapped.get(fid, 0) + count
            else:
                unknown += count
                if _is_generated(script if is_bash else filename):
                    generated += count
                table = self.unknown_sh if is_bash else self.unknown_py
                with self.stats_lock:
                    if key in table or len(table) < MAX_UNKNOWN:
                        table[key] = table.get(key, 0) + count
        with self.stats_lock:
            self.mapped_calls += sum(mapped.values())
            self.unknown_calls += unknown
            self.unknown_generated_calls += generated
            try:
                pid = int(msg.get('pid') or 0)
            except (ValueError, TypeError):
                pid = 0
            entry = self.tracers.setdefault(pid, {'datagrams': 0})
            entry['datagrams'] += 1
            entry['proc'] = str(msg.get('proc') or (_basename(script) if script else '?'))
            entry['kind'] = 'bash' if is_bash else 'python'
            entry['last'] = time.time()
            if len(self.tracers) > 2000:                  # forget the oldest
                for old in sorted(self.tracers, key=lambda p: self.tracers[p].get('last', 0))[:500]:
                    self.tracers.pop(old, None)
        if mapped:
            self.hub.add_calls(mapped)

    # ---- running programs -----------------------------------------------------------------------------------
    def scan_running(self) -> list[str]:
        ids: set[str] = set()
        index = self.index
        me = os.getpid()
        for pid, argv, cwd_link in list_processes():
            if pid == me:
                continue
            cwd = None
            if cwd_link and any(not a.startswith(('/', '-')) and a.endswith(('.py', '.sh')) for a in argv[1:4]):
                try:
                    cwd = os.readlink(cwd_link)               # only needed for relative script paths
                except OSError:
                    cwd = None
            try:
                ids |= index.map_command(argv, cwd)
            except Exception:  # noqa: BLE001 - one odd command line must never stop the watcher
                continue
        if self.mission_planner.running():
            ids.add('ext:mission_planner')
        return sorted(ids)

    # ---- health ---------------------------------------------------------------------------------------------
    def health(self, shown: int = UNKNOWN_SHOWN) -> dict:
        """The /health answer. `shown` = how many 'could not place' entries to list (the busiest first)."""
        now = time.time()
        with self.stats_lock:
            unknown_py = sorted(self.unknown_py.items(), key=lambda kv: -kv[1])[:shown]
            unknown_sh = sorted(self.unknown_sh.items(), key=lambda kv: -kv[1])[:shown]
            tracers = [dict(pid=pid, proc=t.get('proc'), kind=t.get('kind'), datagrams=t.get('datagrams'),
                            last_seen_s_ago=round(now - t.get('last', now), 1))
                       for pid, t in sorted(self.tracers.items(), key=lambda kv: -kv[1].get('last', 0))[:60]]
            counts = dict(datagrams=self.datagrams, bad_datagrams=self.bad_datagrams, bytes=self.bytes_in,
                          mapped_calls=self.mapped_calls, unknown_calls=self.unknown_calls,
                          unknown_generated_calls=self.unknown_generated_calls,
                          unknown_python_entries=len(self.unknown_py), unknown_bash_entries=len(self.unknown_sh))
        data = self.index.data
        return {
            'ok': True,
            'started': _now_iso(self.started),
            'uptime_s': round(now - self.started, 1),
            'url': 'http://%s:%d/' % (self.args.host, self.args.port),
            'udp': {'host': self.args.udp_host, 'port': self.args.udp_port},
            'codemap': {
                'path': str(self.codemap_path),
                'loaded': self.codemap_error is None,
                'error': self.codemap_error,
                'generated': data.get('generated'),
                'commit': data.get('commit'),
                'nodes': len(self.index.node_ids),
                'functions_indexed': self.index.functions_indexed,
                # functions placed by their NAME because their line did not match: those files changed after
                # extract.py ran (count of distinct file+line pairs). Not 0 = re-run extract.py.
                'placed_by_name_only': self.index.by_name_hits,
            },
            'received': counts,
            'unknown_python': [dict(module=k[0], file=k[1], line=k[2], name=k[3], count=v) for k, v in unknown_py],
            'unknown_bash': [dict(sh=k[0], name=k[1], count=v) for k, v in unknown_sh],
            'unknown_note': 'calls the tracers reported that codemap.json does not list: files colcon/rosidl '
                            'generated (counted in received.unknown_generated_calls), files changed since '
                            'extract.py ran (new functions; moved ones are still placed by name, see '
                            'codemap.placed_by_name_only), lambdas. Harmless; re-run extract.py after code changes',
            'tracers': tracers,
            'events_sent': dict(self.hub.events_sent),
            'sse_clients': len(self.hub.clients),
            'max_clients': self.hub.max_clients,
            'slow_client_drops': self.hub.slow_client_drops,
            'recording': str(self.record_path) if self.record_path else None,
            'recorded_events': self.hub.recorded,
            'procs': self.hub.latest_procs,
            'procs_scan': self.procs_scan if self.args.procs_interval > 0 else 'off',
            'mission_planner_check': self.mission_planner.state,
        }

    # ---- recordings -----------------------------------------------------------------------------------------
    def recording_dirs(self) -> list[Path]:
        """The folders GET /recordings lists, in this order (the first one wins when two hold the same name):
        --recordings (default recordings/ next to this file), the folder of the current --record file, and, when
        --recordings was not given, ~/.astra_sim/codemap/recordings (runs started from the Linux-side copy)."""
        dirs = [self.recordings_dir]
        if self.record_path is not None and self.record_path.parent not in dirs:
            dirs.append(self.record_path.parent)
        for folder in self.extra_recording_dirs:
            if folder not in dirs:
                dirs.append(folder)
        return dirs

    def list_recordings(self) -> list[dict]:
        found: dict[str, Path] = {}
        for folder in self.recording_dirs():
            try:
                for p in folder.glob('*.jsonl'):
                    found.setdefault(p.name, p)
            except OSError:
                continue
        rows = []
        for name, p in found.items():
            try:
                st = p.stat()
            except OSError:
                continue
            rows.append({'name': name, 'bytes': st.st_size, 'modified': _now_iso(st.st_mtime),
                         'url': '/recordings/' + quote(name), '_mtime': st.st_mtime})
        rows.sort(key=lambda r: -r['_mtime'])
        for r in rows:
            r.pop('_mtime')
        return rows

    def find_recording(self, name: str) -> Path | None:
        if not _RECORDING_NAME.match(name or ''):
            return None
        for folder in self.recording_dirs():
            p = folder / name
            if p.is_file():
                return p
        return None

    # ---- background threads ---------------------------------------------------------------------------------
    def udp_loop(self, sock: socket.socket) -> None:
        sock.settimeout(0.5)
        while not self.stopping.is_set():
            try:
                data, _addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                if self.stopping.is_set():
                    break
                time.sleep(0.05)                          # e.g. Windows' WSAECONNRESET: keep listening
                continue
            try:
                self.handle_datagram(data)
            except Exception:  # noqa: BLE001 - a strange message must never stop the listener
                with self.stats_lock:
                    self.bad_datagrams += 1

    def batch_loop(self) -> None:
        while not self.stopping.wait(BATCH_SECONDS):
            try:
                self.hub.flush_calls()
            except Exception:  # noqa: BLE001
                pass

    def drain_and_flush(self, sock: socket.socket, limit_seconds: float = 1.0) -> None:
        """At shutdown: handle the messages still waiting in the UDP socket, then send/record the last batch.

        Without this the calls of the last ~200 ms (for example everything a short demo did just before it ended,
        right before demo_traced.sh stops this server) would never reach the recording.
        """
        try:
            sock.setblocking(False)
            end = time.monotonic() + limit_seconds
            while time.monotonic() < end:
                try:
                    data, _addr = sock.recvfrom(65535)
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    break
                try:
                    self.handle_datagram(data)
                except Exception:  # noqa: BLE001
                    pass
        except OSError:
            pass
        try:
            self.hub.flush_calls()
        except Exception:  # noqa: BLE001
            pass

    def write_final_health(self) -> Path | None:
        """At shutdown, with --record: save the last /health answer next to the recording.

        The file is <recording without .jsonl>.health.json (for 20261031-150000.jsonl: 20261031-150000.health.json).
        It keeps what the live page cannot show after the server is gone: how many calls were placed, which calls
        could NOT be placed (file, line, function name: usually files changed after extract.py ran) and which
        programs sent messages. Its name does not end in .jsonl, so /recordings never lists it as a recording.
        Returns the path written, or None (no recording, or the file could not be written: never an error).
        """
        if self.record_path is None:
            return None
        target = self.record_path.with_name(self.record_path.stem + '.health.json')
        try:
            summary = self.health(shown=MAX_UNKNOWN)          # every entry, not only the busiest 50
            summary['final'] = True
            target.write_text(json.dumps(summary, indent=1, sort_keys=True) + '\n', encoding='utf-8')
        except Exception:  # noqa: BLE001 - the shutdown must finish even if this fails
            return None
        received = summary.get('received', {})
        print('code map server summary: %s calls placed, %s could not be placed (list: %s)'
              % (received.get('mapped_calls'), received.get('unknown_calls'), target), flush=True)
        return target

    def procs_loop(self) -> None:
        interval = self.args.procs_interval
        while not self.stopping.is_set():
            try:
                self.load_codemap()
                if interval > 0:
                    self.hub.set_procs(self.scan_running())
            except Exception:  # noqa: BLE001
                pass
            if self.stopping.wait(interval if interval > 0 else PROCS_SECONDS):
                break

    def gzip_of(self, key: tuple, body: bytes) -> bytes:
        hit = self.gzip_cache.get(key)
        if hit is None:
            hit = gzip.compress(body, compresslevel=6)
            if len(self.gzip_cache) > 16:
                self.gzip_cache.clear()
            self.gzip_cache[key] = hit
        return hit


# =============================================================================================================
#  HTTP
# =============================================================================================================
FALLBACK_PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>ASTRA live code map (server)</title>
<style>
 :root { --bg:#fff; --fg:#1d2433; --muted:#5b6475; --ok:#1a7f37; }
 @media (prefers-color-scheme: dark) { :root { --bg:#14171c; --fg:#e6e9ef; --muted:#9aa3b2; --ok:#56d364; } }
 body { background:var(--bg); color:var(--fg); font:15px/1.5 system-ui, sans-serif; margin:0 16px; }
 code, pre { font:13px/1.4 ui-monospace, monospace; } pre { white-space:pre-wrap; word-break:break-all; }
 .muted { color:var(--muted); } .ok { color:var(--ok); }
</style></head><body>
<h1>ASTRA live code map: the server runs</h1>
<p class="muted">The explorer page (<code>simulation/codemap/web/index.html</code>) was not found, so this small
page shows the raw live feed instead. Data: <a href="/codemap.json">/codemap.json</a>,
<a href="/health">/health</a>, <a href="/recordings">/recordings</a>, events at <code>/events</code>.</p>
<h2>Running programs</h2><pre id="procs" class="muted">waiting ...</pre>
<h2>Functions that just ran</h2><pre id="calls" class="muted">waiting ...</pre>
<script>
 const calls = document.getElementById('calls'), procs = document.getElementById('procs'); const lines = [];
 const es = new EventSource('/events');
 es.addEventListener('calls', e => {
   const d = JSON.parse(e.data), when = new Date(d.t * 1000).toLocaleTimeString();
   for (const [id, n] of Object.entries(d.calls)) lines.unshift(when + '  ' + n + 'x  ' + id);
   lines.length = Math.min(lines.length, 200); calls.textContent = lines.join('\\n'); calls.className = 'ok';
 });
 es.addEventListener('procs', e => {
   const d = JSON.parse(e.data); procs.textContent = d.running.join('\\n') || '(none recognised)';
 });
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    server_version = 'AstraCodeMap/1'
    protocol_version = 'HTTP/1.0'      # streaming /events then simply ends with the connection

    @property
    def app(self) -> App:
        return self.server.app          # type: ignore[attr-defined]

    def log_message(self, format: str, *args) -> None:  # noqa: A002 - the base class's name
        if self.app.args.verbose:
            sys.stderr.write('[codemap server] %s - %s\n' % (self.address_string(), format % args))

    # ---- small helpers --------------------------------------------------------------------------------------
    def _cors(self) -> None:
        """Allow pages opened from a local file (Origin: null) or another localhost port to read the data."""
        origin = self.headers.get('Origin')
        if origin and (origin == 'null' or re.match(r'^https?://(127\.0\.0\.1|localhost|\[::1\])(:\d+)?$', origin)):
            self.send_header('Access-Control-Allow-Origin', origin)
            self.send_header('Vary', 'Origin')

    def _send(self, status: int, body: bytes, content_type: str, cache_key: tuple | None = None,
              head_only: bool = False) -> None:
        gz = (cache_key is not None and len(body) > 2048 and 'gzip' in (self.headers.get('Accept-Encoding') or ''))
        if gz:
            body = self.app.gzip_of(cache_key, body)
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-cache')
        if gz:
            self.send_header('Content-Encoding', 'gzip')
        self._cors()
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def _json(self, value, status: int = 200, head_only: bool = False) -> None:
        self._send(status, json.dumps(value, indent=1).encode('utf-8'), 'application/json; charset=utf-8',
                   head_only=head_only)

    def _text(self, status: int, text: str, head_only: bool = False) -> None:
        self._send(status, text.encode('utf-8'), 'text/plain; charset=utf-8', head_only=head_only)

    # ---- routes ---------------------------------------------------------------------------------------------
    def do_HEAD(self) -> None:  # noqa: N802 - http.server naming
        self._route(head_only=True)

    def do_GET(self) -> None:  # noqa: N802 - http.server naming
        self._route(head_only=False)

    def _route(self, head_only: bool) -> None:
        try:
            path = unquote(urlsplit(self.path).path)
            app = self.app
            if path in ('/', '/index.html'):
                index = app.web_dir / 'index.html'
                if index.is_file():
                    body = index.read_bytes()
                    self._send(200, body, 'text/html; charset=utf-8', ('index', index.stat().st_mtime), head_only)
                else:
                    self._send(200, FALLBACK_PAGE.encode('utf-8'), 'text/html; charset=utf-8', head_only=head_only)
            elif path == '/codemap.json':
                app.load_codemap()
                if app.codemap_error is None:
                    self._send(200, app.codemap_bytes, 'application/json; charset=utf-8',
                               ('codemap', app.codemap_mtime), head_only)
                else:
                    self._json({'error': app.codemap_error}, status=404, head_only=head_only)
            elif path == '/events':
                if head_only:
                    self._text(200, '', head_only=True)
                else:
                    self._events()
            elif path == '/health':
                self._json(app.health(), head_only=head_only)
            elif path in ('/recordings', '/recordings/'):
                self._json(app.list_recordings(), head_only=head_only)
            elif path.startswith('/recordings/'):
                found = app.find_recording(path[len('/recordings/'):])
                if found is None:
                    self._text(404, 'no such recording\n', head_only)
                else:
                    self._send(200, found.read_bytes(), 'application/x-ndjson; charset=utf-8', None, head_only)
            else:
                self._web_file(path, head_only)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def _web_file(self, path: str, head_only: bool) -> None:
        """Any other file of the web folder (only inside it; never '..')."""
        rel = path.lstrip('/')
        target = (self.app.web_dir / rel).resolve()
        try:
            target.relative_to(self.app.web_dir)
        except ValueError:
            self._text(404, 'not found\n', head_only)
            return
        if not rel or not target.is_file():
            self._text(404, 'not found\n', head_only)
            return
        ctype = mimetypes.guess_type(target.name)[0] or 'application/octet-stream'
        if ctype.startswith('text/') or ctype in ('application/javascript', 'application/json'):
            ctype += '; charset=utf-8'
        self._send(200, target.read_bytes(), ctype, ('web', rel, target.stat().st_mtime), head_only)

    def _events(self) -> None:
        """Server-Sent Events: stays open, writes "event: calls/procs" chunks as the hub publishes them."""
        hub = self.app.hub
        q = hub.register()
        if q is None:
            self._text(503, 'Too many live code map pages are open (the limit is %d). Close one and reload.\n'
                       % hub.max_clients)
            return
        try:
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('X-Accel-Buffering', 'no')
            self._cors()
            self.end_headers()
            self.wfile.write(b'retry: 2000\n\n')
            self.wfile.write(hub.sse_chunk('procs', hub.latest_procs))
            self.wfile.flush()
            while not self.app.stopping.is_set():
                try:
                    chunk = q.get(timeout=KEEPALIVE_SECONDS)
                except queue.Empty:
                    chunk = b': keep-alive\n\n'
                if chunk is None:
                    break
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            hub.unregister(q)


class Server(ThreadingHTTPServer):
    daemon_threads = True               # open /events connections never keep the program alive
    block_on_close = False
    allow_reuse_address = os.name != 'nt'   # on Windows SO_REUSEADDR would let two servers share a port


# =============================================================================================================
#  main
# =============================================================================================================
def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog='server.py',
        description='ASTRA live code map: collects the tracers\' messages (UDP) and serves the explorer page '
                    'with live events (HTTP, Server-Sent Events). Open the printed URL in a browser.')
    p.add_argument('--host', default=DEFAULT_HOST, help='HTTP address to listen on (default 127.0.0.1)')
    p.add_argument('--port', type=int, default=DEFAULT_PORT, help='HTTP port (default 8090)')
    p.add_argument('--udp-host', default=DEFAULT_HOST, help='UDP address for the tracers (default 127.0.0.1)')
    p.add_argument('--udp-port', type=int, default=DEFAULT_UDP_PORT, help='UDP port for the tracers (default 8091)')
    p.add_argument('--codemap', default=str(HERE / 'codemap.json'), help='codemap.json made by extract.py')
    p.add_argument('--web', default=str(HERE / 'web'), help='folder with index.html (default: web/ next to me)')
    p.add_argument('--record', metavar='PATH', default=None,
                   help='write every calls/procs event as JSON lines to PATH (a file; a folder -> <stamp>.jsonl in '
                        'it; "auto" -> recordings/<stamp>.jsonl)')
    p.add_argument('--recordings', default=None, metavar='DIR',
                   help='the folder listed at /recordings (default: recordings/ next to me AND '
                        '~/.astra_sim/codemap/recordings, where demo_traced.sh records runs started from the '
                        'Linux-side copy; $ASTRA_SIM_HOME/codemap/recordings when ASTRA_SIM_HOME is set)')
    p.add_argument('--procs-interval', type=float, default=PROCS_SECONDS,
                   help='seconds between "procs" events (default 2; 0 = do not watch programs)')
    p.add_argument('--no-windows', action='store_true',
                   help='do not ask Windows (tasklist.exe) whether Mission Planner runs')
    p.add_argument('--verbose', action='store_true', help='print every HTTP request')
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    app = App(args)
    try:
        udp = socket.socket(socket.AF_INET6 if ':' in args.udp_host else socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind((args.udp_host, args.udp_port))
    except OSError as error:
        sys.stderr.write('The code map server cannot listen on UDP %s:%d (%s). Is another server.py already '
                         'running? Stop it, or use --udp-port.\n' % (args.udp_host, args.udp_port, error))
        return 2
    try:
        httpd = Server((args.host, args.port), Handler)
    except OSError as error:
        sys.stderr.write('The code map server cannot listen on http://%s:%d/ (%s). Is another server.py already '
                         'running? Stop it, or use --port.\n' % (args.host, args.port, error))
        udp.close()
        return 2
    httpd.app = app  # type: ignore[attr-defined]
    threads = []
    for target, arg in ((app.udp_loop, (udp,)), (app.batch_loop, ()), (app.procs_loop, ())):
        thread = threading.Thread(target=target, args=arg, daemon=True)
        thread.start()
        threads.append(thread)

    def _stop(signum, frame):  # noqa: ARG001 - signal handler signature
        raise SystemExit(0)

    try:
        signal.signal(signal.SIGTERM, _stop)
    except (ValueError, OSError, AttributeError):
        pass
    url = 'http://%s:%d/' % ('127.0.0.1' if args.host in ('0.0.0.0', '') else args.host, args.port)
    print('ASTRA live code map server', flush=True)
    print('  open        %s   (a browser on Windows reaches it too: WSL forwards localhost)' % url, flush=True)
    print('  tracers     UDP %s:%d' % (args.udp_host, args.udp_port), flush=True)
    if app.codemap_error is None:
        print('  code map    %s (%d blocks, %d functions indexed)'
              % (app.codemap_path, len(app.index.node_ids), app.index.functions_indexed), flush=True)
    else:
        print('  code map    NOT LOADED: %s' % app.codemap_error, flush=True)
    if app.record_path:
        print('  recording   %s' % app.record_path, flush=True)
    print('  stop it     Ctrl+C', flush=True)
    try:
        httpd.serve_forever(poll_interval=0.5)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)   # a second TERM must not cut the clean-up short
        except (ValueError, OSError, AttributeError):
            pass
        app.stopping.set()
        for thread in threads[:2]:              # the UDP listener and the batcher end within 0.5 s
            thread.join(1.0)
        app.drain_and_flush(udp)                # the last messages and the last batch reach the recording
        app.write_final_health()                # <recording>.health.json: what could not be placed, per tracer
        app.hub.close()
        try:
            httpd.server_close()
        except OSError:
            pass
        udp.close()
        print('code map server stopped%s' % (' (recording: %s, %d events)' % (app.record_path, app.hub.recorded)
                                             if app.record_path else ''), flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
