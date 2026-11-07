#!/usr/bin/env python3
"""
simulation/codemap/extract.py -- PURPOSE: read every source file of this repository (without running any of
it) and write ONE map of the whole project, simulation/codemap/codemap.json. The code-map explorer
(simulation/codemap/web/index.html, served by server.py) draws that map as connected, labelled blocks:

    systems (17 big parts, from systems.yaml) -> packages (folders) -> modules (files) -> classes -> functions

plus the wires between the blocks: which function CALLS which, which file IMPORTS which, which ROS 2 topics
a node PUBLISHES and SUBSCRIBES to, which launch file LAUNCHES which program, which script RUNS which
script, and the network links (MAVLink, HTTP, data) between the big programs such as Gazebo, ArduPilot SITL,
MAVProxy, MAVROS and Mission Planner. It also writes the "wiring" part that the explorer's Wiring view draws:
how the programs talk (boxes, and wires coloured by kind of message with their transport and port), copied
from systems.yaml's "wiring:" section and from every link there that has "wire:".

How it works, in plain words:
  1. It walks the repository folder and decides what kind of file each one is. EVERY file becomes a block
     (code and settings are read closely; documents, pictures, data files, backups and marker files get a
     short description). Left out: .git, build, install, log, __pycache__, node_modules, simulation/codemap/dist
     and the saved *.mhtml web pages (the contract), the owner's git-ignored LOCAL_NOTES.md, codemap.json itself,
     and the git-ignored folders the flight code fills with photos while it runs (mapping_photos ...).
  2. Python files are parsed with Python's own `ast` module ("abstract syntax tree": the program as a tree
     of parts). The tree tells us every class, function and method, its line numbers, its arguments, its
     docstring, the calls it makes and the ROS topics it creates. Nothing is imported or executed.
  3. Bash, C#, PowerShell and batch files are read with careful text rules: a small scanner that knows about
     quotes, comments and here-documents, plus regular expressions (text patterns).
  4. Launch files (*.launch.py, *.launch.xml) are read to find which ROS package/executable they start.
  5. YAML, SDF, XML, mission and parameter files become "module" blocks whose summary is their header comment.
  6. simulation/codemap/systems.yaml (written by hand) puts every folder into one system (and, with "files:",
     single files that belong to another system than their folder), describes the programs that are not ours
     ("externals", including the team's new video pipeline whose code is not in this repo), lists the network
     links between the big programs and lays out the Wiring view (build_wiring checks every box, wire and tour
     step: a wire that no box can hold, a wire id used twice or an unknown kind makes --check fail).
  7. Explanations written for people (simulation/codemap/explanations/*.json) are copied onto the matching
     blocks. A block without one gets the first paragraph of its docstring, marked "(from the docstring)";
     a test function gets its name turned into a sentence ("Test: the publisher follows the schedule.").
  8. A "trace index" is added: the live tracer (trace/astra_trace.py) reports "this file, this line number,
     this function name ran", and the index turns that report into a block id so the block lights up green.

Run it (Windows or Ubuntu, Python 3.10 or newer, standard library only):
    python3 simulation/codemap/extract.py                 writes simulation/codemap/codemap.json
    python3 simulation/codemap/extract.py --no-source     the same without each function's source text (smaller)
    python3 simulation/codemap/extract.py --check         exit code 1 if a file has no package/system, an id
                                                          appears twice, systems.yaml is unclear or a file
                                                          could not be analysed (use it in tests / CI)
    python3 simulation/codemap/extract.py --out FILE      write somewhere else
    python3 simulation/codemap/extract.py --quiet         only print problems
The output is sorted and written one block per line, so a change in the code shows up as a small, readable
change in codemap.json. Only the "generated" time stamp changes on every run (set the environment variable
SOURCE_DATE_EPOCH to a number of seconds to freeze it).

The format (ids, node fields, edge kinds, trace_index) is the shared contract of the code-map tools; it is
described in simulation/codemap/README.md. Ids:
    sys:<key>  ext:<key>  pkg:<folder>  mod:<file>  fn:<file>::<qualname>  topic:<ROS name>
  e.g. fn:ros2_ws/src/main/src/main_controller.py::MainController.image_result_cb
A class has the id fn:<file>::<ClassName> (the same "qualified name" rule as its methods).

Extras on top of the contract (readers that do not know them can ignore them):
  * module "lang" says what kind of file it is: python bash csharp cpp batch powershell html javascript css |
    xml sdf yaml rosidl params mission cmake dockerfile config json | markdown text pdf image mesh data weights
    backup marker other. Pictures, PDFs, data and weights have no line numbers.
  * ROS 2 services are "topic" nodes too (topic:/addWaypoint); their summary says "ROS 2 service". The client
    has a "calls" edge to it, the server a "subscribes" edge labelled "serves ...".
  * external nodes carry "process_match": regular expressions that recognise the program in a process list.
  * "called_when" is filled from the code when nobody explained it: ROS subscription / service / timer
    callbacks, console entry points, `if __name__ == '__main__'` calls, bash trap handlers.
  * a package block may group the files one system claims from one folder: id pkg:<folder>@<system key>, its
    "path" is that folder (systems.yaml "files:").
  * top-level "wiring" (only when systems.yaml has a wiring section): {"schema", "title", "intro", "width",
    "height", "kinds": [{key, label, text}], "statuses": [...], "zones": [{id, label, note, x, y, w, h}],
    "boxes": [{id, title, x, y, w, h, status, members, node?, tag?, lines?, chips?, where?, stands_for?, explain?,
    shape?}], "wires": [{id, from, to (box ids), kind, status, arrow, title, via?, explain?, code?, path?,
    label_at?, offset?, map_from?, map_to?, label?}], "steps": [{title, text, wires, boxes}]}.
  * trace_index also has "sh_files" {repo path of a bash script: mod id} and "programs" {installed program path
    (ros2_ws/install/<pkg>/lib/<pkg>/<exe>): mod id}. "py_files" also lists where colcon puts our Python files
    (ros2_ws/build/<pkg>/..., ros2_ws/install/<pkg>/lib/python3.10/site-packages/...,
    .../local/lib/python3.10/dist-packages/..., ros2_ws/install/<pkg>/lib/<pkg>/<script>), so the end of a
    reported file path can be matched against these keys. A file run as a program reports __name__ "__main__":
    find it by its path. A function defined twice under one name (a property's getter and setter, overloads)
    is ONE block, and all its lines point to it.
"""

from __future__ import annotations

import argparse
import ast
import datetime as _dt
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path

# ---------------------------------------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------------------------------------

SCHEMA = 1                       # version of the codemap.json format
SOURCE_CAP = 4000                # at most this many characters of each function's source text
DOC_CAP = 6000                   # at most this many characters of a docstring / header comment
SUMMARY_CAP = 700                # at most this many characters in a one-paragraph summary
EXPLAIN_KEYS = ('explain', 'called_when', 'inputs', 'outputs', 'effects', 'how_it_fits')
DOCSTRING_MARK = ' (from the docstring)'      # added to an "explain" that was copied from a docstring
GENERATED_MARK = ' (made from the code)'      # added to an "explain" that this script wrote from code facts
NODE_KINDS = ('system', 'external', 'package', 'module', 'class', 'function', 'method', 'topic')
EDGE_KINDS = ('calls', 'imports', 'publishes', 'subscribes', 'launches', 'runs', 'mavlink', 'http', 'data')

# Folders never looked into (any depth), and repo-relative folders never looked into. Besides the build output
# and caches of the contract, these are the git-ignored folders the flight code fills while it runs (photos,
# detector pictures, terrain downloads): in the Linux copy they can hold hundreds of files after a flight.
SKIP_DIR_NAMES = frozenset({'.git', 'build', 'install', 'log', '__pycache__', 'node_modules', '.ruff_cache',
                            '.pytest_cache', '.mypy_cache', '.venv', 'venv', '.tox', '.idea', '.vscode', '.claude',
                            'mapping_photos', 'detection_results_sahi', 'detection_results_steroids', 'camera_feed'})
SKIP_REL_DIRS = frozenset({'simulation/codemap/dist', 'simulation/codemap/recordings', 'ros2_ws/terrain',
                           'ros2_ws/detection', 'ros2_ws/src/video_cam/video_cam/documentations',
                           'ros2_ws/src/video_cam/Documentation'})

# The Python of ROS 2 Humble on Ubuntu 22.04 (used to name files inside the colcon install tree).
INSTALL_PYTHON = 'python3.10'

# Bash words that only PRINT their arguments: a script path inside them is a message, not a command.
PRINT_COMMANDS = frozenset({'echo', 'printf', 'say', 'info', 'warn', 'die', 'ok', 'note', 'log', 'tip', 'fail',
                            'bad', 'problem', 'usage_error', 'err', 'error', 'msg', 'step', 'result', 'row',
                            'read', 'cat', 'title', 'rem', 'pause'})
BASH_KEYWORDS = frozenset({'if', 'then', 'else', 'elif', 'fi', 'do', 'done', 'while', 'until', 'time', '!', '{',
                           '}', 'esac', 'coproc', ';;', 'in'})
BASH_WRAPPERS = frozenset({'exec', 'command', 'builtin', 'nohup', 'setsid', 'nice', 'stdbuf', 'unbuffer'})
PYTHON_EXE_RE = re.compile(r'^(?:.*/)?python(?:3(?:\.\d+)?)?(?:\.exe)?$')

CS_KEYWORDS = frozenset({'if', 'for', 'foreach', 'while', 'switch', 'catch', 'using', 'lock', 'return', 'new',
                         'typeof', 'sizeof', 'nameof', 'default', 'checked', 'unchecked', 'fixed', 'base',
                         'this', 'get', 'set', 'add', 'remove', 'when', 'where', 'else', 'do', 'throw', 'await',
                         'is', 'as', 'in', 'out', 'ref', 'params', 'yield', 'case', 'goto', 'delegate',
                         'operator', 'implicit', 'explicit', 'stackalloc', 'unsafe', 'try', 'finally'})


# ---------------------------------------------------------------------------------------------------------
# Small text helpers
# ---------------------------------------------------------------------------------------------------------

def read_text(path) -> str:
    """Read a file as text: UTF-8 (bad bytes become a replacement mark), no byte-order mark, '\\n' line ends."""
    data = Path(path).read_bytes()
    if data.startswith(b'\xef\xbb\xbf'):
        data = data[3:]
    return data.decode('utf-8', errors='replace').replace('\r\n', '\n').replace('\r', '\n')


def cap_text(text: str | None, cap: int) -> str | None:
    """Cut a long text at a line end before `cap` characters and say how much was left out."""
    if text is None or len(text) <= cap:
        return text
    cut = text.rfind('\n', 0, cap)
    if cut < cap // 2:
        cut = cap
    return text[:cut].rstrip() + f'\n... ({len(text) - cut} more characters not shown)'


def shorten(text: str, cap: int = SUMMARY_CAP) -> str:
    """Cut a one-paragraph text near `cap` characters, at the end of a sentence when possible."""
    if len(text) <= cap:
        return text
    head = text[:cap]
    cut = max(head.rfind('. '), head.rfind('! '), head.rfind('? '))
    if cut > cap // 2:
        return head[:cut + 1]
    cut = head.rfind(' ')
    return (head[:cut] if cut > cap // 2 else head).rstrip(' ,;:') + ' ...'


_DECORATION_RE = re.compile(r'^[\s=\-#*~_+.─-╿]*$')


def clean_comment_lines(lines) -> str:
    """Join comment lines (markers already removed) into text, dropping '=====' style decoration lines."""
    out = []
    for line in lines:
        if line.strip() and _DECORATION_RE.match(line) and len(line.strip()) >= 3:
            continue
        out.append(line.rstrip())
    while out and not out[0].strip():
        out.pop(0)
    while out and not out[-1].strip():
        out.pop()
    # remove the common indentation, so indented lists stay readable
    widths = [len(x) - len(x.lstrip()) for x in out if x.strip()]
    pad = min(widths) if widths else 0
    return '\n'.join(x[pad:] for x in out).strip()


def first_paragraph(doc: str | None) -> str | None:
    """The first paragraph of a docstring, on one line, capitalised, at most SUMMARY_CAP characters."""
    if not doc or not doc.strip():
        return None
    para = re.split(r'\n\s*\n', doc.strip(), maxsplit=1)[0]
    para = ' '.join(para.split())
    if not para:
        return None
    para = para[0].upper() + para[1:]
    return shorten(para)


def purpose_summary(doc: str | None) -> str | None:
    """The PURPOSE paragraph of a header (this repo's rule: every file starts with "<file> -- PURPOSE: ...").

    Without the word PURPOSE it is the first paragraph, minus a leading "<file name> --" title."""
    if not doc or not doc.strip():
        return None
    text = doc.strip()
    m = re.search(r'\bPURPOSE\s*:\s*', text, re.I)
    if m:
        text = text[m.end():]
    else:
        text = re.sub(r'^[\w./\\ -]+\.[A-Za-z0-9]+\s+(?:--|-|:)\s+', '', text, count=1)
    return first_paragraph(text)


def posix_join(*parts: str) -> str:
    """Join repo-relative path parts with '/', treating '.' and '' as the repo root."""
    clean = [p for p in parts if p not in ('', '.')]
    return '/'.join(clean) if clean else '.'


def posix_dirname(rel: str) -> str:
    """Folder of a repo-relative path ('.' for files in the repo root)."""
    return rel.rsplit('/', 1)[0] if '/' in rel else '.'


def posix_norm(rel: str) -> str | None:
    """Normalise 'a/./b/../c' to 'a/c'; None when it climbs out of the repo."""
    out = []
    for part in rel.replace('\\', '/').split('/'):
        if part in ('', '.'):
            continue
        if part == '..':
            if not out:
                return None
            out.pop()
            continue
        out.append(part)
    return '/'.join(out) if out else '.'


def node_text(lines: list[str], first: int, last: int) -> str:
    """Lines first..last (1-based, inclusive) of a file as one text."""
    return '\n'.join(lines[max(first, 1) - 1:max(last, first)])


# ---------------------------------------------------------------------------------------------------------
# A tiny YAML reader for systems.yaml (the standard library has none)
# ---------------------------------------------------------------------------------------------------------

class YamlError(ValueError):
    """systems.yaml uses something this small reader does not understand (the message says where)."""


def _yaml_strip_comment(s: str) -> str:
    """Remove a ' # comment' that is outside quotes."""
    quote = None
    for i, c in enumerate(s):
        if quote:
            if c == quote:
                quote = None
        elif c in ('"', "'") and (i == 0 or s[i - 1] in ' [,{:'):
            quote = c
        elif c == '#' and (i == 0 or s[i - 1] in ' \t'):
            return s[:i].rstrip()
    return s.rstrip()


def _yaml_scalar(s: str, where: str):
    """One YAML scalar: 'single', "double" (with backslash escapes), [a, b] flow list, or plain text."""
    s = s.strip()
    if not s:
        return ''
    if s[0] == '[':
        if not s.endswith(']'):
            raise YamlError(f'{where}: a [list] must end on the same line')
        inner = s[1:-1].strip()
        if not inner:
            return []
        items, cur, quote = [], '', None
        for c in inner:
            if quote:
                cur += c
                if c == quote:
                    quote = None
            elif c in ('"', "'") and not cur.strip():
                quote = c
                cur += c
            elif c == ',':
                items.append(cur)
                cur = ''
            else:
                cur += c
        items.append(cur)
        return [_yaml_scalar(x, where) for x in items]
    if s[0] == "'":
        if not s.endswith("'") or len(s) < 2:
            raise YamlError(f'{where}: unclosed single quote')
        return s[1:-1].replace("''", "'")
    if s[0] == '"':
        if not s.endswith('"') or len(s) < 2:
            raise YamlError(f'{where}: unclosed double quote')
        body = s[1:-1]
        return re.sub(r'\\(["\\/nt])', lambda m: {'n': '\n', 't': '\t'}.get(m.group(1), m.group(1)), body)
    if s[0] in '{&*!|>' or s.startswith('- '):
        raise YamlError(f'{where}: "{s[:20]}" is not supported here (use quotes)')
    return s


class _YamlReader:
    """Indentation-based reader for the YAML subset systems.yaml uses: mappings, lists, plain/quoted scalars,
    [flow lists] and folded '>' / '>-' or literal '|' / '|-' blocks. Values stay strings."""

    def __init__(self, text: str, name: str):
        self.lines = text.replace('\r\n', '\n').split('\n')
        self.name = name
        self.i = 0

    def where(self) -> str:
        return f'{self.name} line {self.i + 1}'

    @staticmethod
    def indent(line: str) -> int:
        return len(line) - len(line.lstrip(' '))

    def skip_blank(self):
        while self.i < len(self.lines):
            s = self.lines[self.i].strip()
            if s and not s.startswith('#') and s != '---':
                return
            self.i += 1

    def parse(self):
        self.skip_blank()
        if self.i >= len(self.lines):
            return {}
        return self.block(self.indent(self.lines[self.i]))

    def block(self, indent: int):
        content = _yaml_strip_comment(self.lines[self.i].strip())
        if content == '-' or content.startswith('- '):
            return self.seq(indent)
        return self.mapping(indent)

    def value_after(self, rest: str, indent: int):
        """The value of 'key: rest' (or '- rest'): inline scalar, block scalar or an indented block."""
        if rest in ('>', '>-', '|', '|-', '>+', '|+'):
            return self.block_scalar(indent, rest)
        if rest:
            return _yaml_scalar(rest, self.where())
        self.skip_blank()
        if self.i >= len(self.lines):
            return None
        nxt = self.lines[self.i]
        ind = self.indent(nxt)
        content = _yaml_strip_comment(nxt.strip())
        if ind > indent:
            return self.block(ind)
        if ind == indent and (content == '-' or content.startswith('- ')):
            return self.seq(ind)
        return None

    def mapping(self, indent: int) -> dict:
        out = {}
        while True:
            self.skip_blank()
            if self.i >= len(self.lines):
                return out
            raw = self.lines[self.i]
            ind = self.indent(raw)
            if ind < indent:
                return out
            if ind > indent:
                raise YamlError(f'{self.where()}: unexpected indentation')
            content = _yaml_strip_comment(raw.strip())
            if content == '-' or content.startswith('- '):
                return out
            m = re.match(r'''^("[^"]*"|'[^']*'|[^:#][^:]*?)\s*:(?:\s+(.*)|\s*)$''', content)
            if not m:
                raise YamlError(f'{self.where()}: expected "key: value", got "{content[:40]}"')
            key = _yaml_scalar(m.group(1), self.where())
            if key in out:
                raise YamlError(f'{self.where()}: the key "{key}" appears twice')
            self.i += 1
            out[key] = self.value_after((m.group(2) or '').strip(), indent)

    def seq(self, indent: int) -> list:
        out = []
        while True:
            self.skip_blank()
            if self.i >= len(self.lines):
                return out
            raw = self.lines[self.i]
            ind = self.indent(raw)
            content = _yaml_strip_comment(raw.strip())
            if ind != indent or not (content == '-' or content.startswith('- ')):
                if ind > indent:
                    raise YamlError(f'{self.where()}: unexpected indentation in a list')
                return out
            after = content[1:]
            item_indent = ind + 1 + (len(after) - len(after.lstrip(' ')))
            after = after.strip()
            if not after:
                self.i += 1
                out.append(self.value_after('', indent))
            elif re.match(r'''^("[^"]*"|'[^']*'|[^:#\['"][^:]*?)\s*:(\s|$)''', after):
                # (a plain key never starts with a quote, so a quoted item such as "HUD: camera" stays one text)
                # "- key: value" starts a mapping whose keys line up with "key"
                self.lines[self.i] = ' ' * item_indent + raw.lstrip()[1:].lstrip(' ')
                out.append(self.mapping(item_indent))
            else:
                self.i += 1
                out.append(self.value_after(after, indent) if after in ('>', '>-', '|', '|-')
                           else _yaml_scalar(after, self.where()))

    def block_scalar(self, indent: int, style: str) -> str:
        lines = []
        block_indent = None
        while self.i < len(self.lines):
            raw = self.lines[self.i]
            if raw.strip():
                ind = self.indent(raw)
                if ind <= indent:
                    break
                if block_indent is None:
                    block_indent = ind
                lines.append(raw[min(block_indent, ind):])
            else:
                lines.append('')
            self.i += 1
        while lines and not lines[-1]:
            lines.pop()
        if style.startswith('|'):
            text = '\n'.join(lines)
        else:
            paras, cur = [], []
            for line in lines:
                if line:
                    cur.append(line.strip())
                else:
                    paras.append(' '.join(cur))
                    cur = []
            paras.append(' '.join(cur))
            text = '\n'.join(paras)
        return text if style.endswith('-') else text + '\n'


def parse_yaml_subset(text: str, name: str = 'yaml'):
    """Read the YAML subset described in _YamlReader (raises YamlError with the line number on anything else)."""
    return _YamlReader(text, name).parse()


# ---------------------------------------------------------------------------------------------------------
# Which files are source files, and which language
# ---------------------------------------------------------------------------------------------------------

# Kinds of files ("lang" of a module block). Code and configuration are read closely; the rest only get a
# block with a short description (the explorer shows every file of the repo).
CODE_LANGS = frozenset({'python', 'bash', 'csharp', 'cpp', 'batch', 'powershell', 'html', 'javascript', 'css'})
CONFIG_LANGS = frozenset({'xml', 'sdf', 'yaml', 'rosidl', 'params', 'mission', 'cmake', 'dockerfile', 'config',
                          'json'})
OTHER_LANGS = frozenset({'markdown', 'text', 'pdf', 'image', 'mesh', 'data', 'weights', 'backup', 'marker', 'other'})
TEXT_LANGS = CODE_LANGS | CONFIG_LANGS | {'markdown', 'text', 'backup'}     # files whose lines are counted


def classify(rel: str, head: bytes = b'') -> str | None:
    """The kind of a repo file ('python', 'bash', 'yaml', 'markdown', 'image' ...). None = leave it out
    (the saved *.mhtml web pages, as the code-map contract says)."""
    name = rel.rsplit('/', 1)[-1]
    low = name.lower()
    if low.endswith('.mhtml'):
        return None
    if re.search(r"\.(bak|orig|rej|swp)\b|\.bak[\w.-]*$|prefix_bak$|~$|'$|\.\d{8}-\d{6}$", low):
        return 'backup'
    table = (
        (('.py',), 'python'), (('.sh', '.bash'), 'bash'), (('.cs',), 'csharp'),
        (('.cpp', '.cc', '.cxx', '.hpp', '.hh', '.h', '.c'), 'cpp'), (('.bat', '.cmd'), 'batch'),
        (('.ps1',), 'powershell'), (('.sdf', '.xacro', '.world'), 'sdf'), (('.xml', '.config', '.launch'), 'xml'),
        (('.yaml', '.yml', '.rviz'), 'yaml'), (('.msg', '.srv', '.action'), 'rosidl'),
        (('.parm', '.param'), 'params'), (('.waypoints',), 'mission'), (('.cmake',), 'cmake'),
        (('.toml', '.cfg', '.ini', '.dsv.in'), 'config'), (('.html', '.htm'), 'html'), (('.js', '.mjs'), 'javascript'),
        (('.css',), 'css'),
    )
    for suffixes, lang in table:
        if low.endswith(suffixes):
            return lang
    if name == 'CMakeLists.txt':
        return 'cmake'
    if name == 'Dockerfile' or low.endswith('.dockerfile'):
        return 'dockerfile'
    if re.match(r'^requirements[\w.-]*\.txt$', low):
        return 'config'
    if name == 'devcontainer.json':
        return 'json'
    if '.' not in name and head.startswith(b'#!'):
        first = head.split(b'\n', 1)[0]
        if b'python' in first:
            return 'python'
        if re.search(rb'\b(ba)?sh\b', first):
            return 'bash'
    if low.endswith(('.md', '.markdown', '.rst')):
        return 'markdown'
    if low.endswith(('.png', '.jpg', '.jpeg', '.gif', '.svg', '.bmp', '.ico', '.tif', '.tiff', '.webp')):
        return 'image'
    if low.endswith(('.obj', '.mtl', '.stl', '.dae', '.fbx', '.glb', '.gltf', '.ply')):
        return 'mesh'
    if low.endswith('.pdf') or head.startswith(b'%PDF'):
        return 'pdf'
    if low.endswith(('.json', '.jsonl', '.csv', '.geojson', '.jgw', '.npz', '.npy', '.bin', '.tlog')):
        return 'data'
    if low.endswith(('.pt', '.onnx', '.engine', '.trt')):
        return 'weights'
    if name in ('COLCON_IGNORE', 'AMENT_IGNORE', 'CATKIN_IGNORE') or ('/resource/' in '/' + rel and '.' not in name):
        return 'marker'
    if low.startswith('.git') or low in ('.clang-format', '.clang-tidy', '.editorconfig', '.dockerignore'):
        return 'config'
    if low.endswith(('.txt', '.cff')) or name in ('LICENSE', 'NOTICE', 'AUTHORS'):
        return 'text'
    return 'other'


def file_kind_summary(rel: str, lang: str, size: int) -> str | None:
    """A short factual sentence for a file that is not code (pictures, data, backups, marker files ...)."""
    name = rel.rsplit('/', 1)[-1]
    ext = name.rsplit('.', 1)[-1].upper() if '.' in name else ''
    if lang == 'backup':
        orig = re.sub(r"(\.bak[\w.-]*|\.orig|\.prefix_bak|~|'|\.\d{8}-\d{6})$", '', name)
        return f'An old copy of {orig}, kept for reference (a backup): nothing runs or imports it.'
    if lang == 'marker':
        if name.upper().endswith('IGNORE'):
            return 'An empty marker file: the ROS build tool (colcon) does not look into this folder.'
        return (f'An empty marker file: after the build, ROS 2 finds the package "{name}" by this file '
                '(the ament resource index).')
    if lang == 'image':
        return f'A picture ({ext}, {size // 1024} KiB).'
    if lang == 'mesh':
        return f'A 3D shape file ({ext}) used by a Gazebo model.'
    if lang == 'pdf':
        return f'A PDF document ({size // 1024} KiB).'
    if lang == 'data':
        return f'A data file ({ext or "no extension"}, {size // 1024} KiB).'
    if lang == 'weights':
        return f'Trained model weights ({ext}, {size // (1024 * 1024)} MiB): a large binary file, not code.'
    return None


def markdown_summary(text: str) -> tuple[str | None, str | None]:
    """(header comment, summary) of a Markdown file: its <!-- PURPOSE --> comment, else its first paragraph."""
    comment = xml_comment_header(text) if text.lstrip().startswith('<!--') else None
    if comment:
        return comment, purpose_summary(comment)
    body = []
    for para in re.split(r'\n\s*\n', text.strip()):
        p = para.strip()
        if not p or p.startswith(('#', '|', '```', '<', '![', '[!', '---', '>')):
            continue
        body.append(p)
        break
    return None, first_paragraph(body[0]) if body else None


def scan_repo(repo: Path, skip_files=()):
    """Walk the repository: ({rel path: lang} of every file it keeps, every file seen, counts per lang)."""
    files: dict[str, str] = {}
    everything: set[str] = set()
    counts: Counter = Counter()
    skip_files = set(skip_files)
    for root, dirs, names in os.walk(repo):
        rel_root = Path(root).relative_to(repo).as_posix()
        rel_root = '' if rel_root == '.' else rel_root
        dirs[:] = sorted(d for d in dirs
                         if d not in SKIP_DIR_NAMES and posix_join(rel_root, d) not in SKIP_REL_DIRS)
        for name in sorted(names):
            rel = posix_join(rel_root, name)
            everything.add(rel)
            if rel in skip_files:
                counts['left out on purpose'] += 1
                continue
            head = b''
            if '.' not in name:
                try:
                    with open(os.path.join(root, name), 'rb') as f:
                        head = f.read(200)
                except OSError:
                    head = b''
            lang = classify(rel, head)
            if lang:
                files[rel] = lang
                counts[lang] += 1
            else:
                counts['left out (*.mhtml saved web pages)'] += 1
    return files, everything, counts


# ---------------------------------------------------------------------------------------------------------
# Header comments of non-Python files
# ---------------------------------------------------------------------------------------------------------

def hash_comment_header(text: str) -> str | None:
    """The leading block of '#' comment lines (after a #! line and blank lines), markers removed."""
    lines = text.split('\n')
    i = 0
    if lines and lines[0].startswith('#!'):
        i = 1
    while i < len(lines) and (not lines[i].strip() or lines[i].strip() == '---'):
        i += 1
    block = []
    while i < len(lines) and lines[i].lstrip().startswith('#'):
        block.append(re.sub(r'^\s*#+ ?', '', lines[i]))
        i += 1
    text = clean_comment_lines(block)
    return text or None


def xml_comment_header(text: str) -> str | None:
    """The first <!-- comment --> that comes before the first real element (after <?xml ...?>)."""
    for m in re.finditer(r'<!--(.*?)-->|<(?![?!])', text, re.S):
        if m.group(1) is None:
            return None
        body = clean_comment_lines(m.group(1).split('\n'))
        return body or None
    return None


def slash_comment_header(text: str) -> str | None:
    """A leading /* ... */ block or run of // lines (C#, C++, JavaScript, CSS)."""
    s = text.lstrip()
    if s.startswith('/*'):
        end = s.find('*/')
        body = s[2:end if end >= 0 else len(s)]
        return clean_comment_lines(re.sub(r'^\s*\*+ ?', '', ln) for ln in body.split('\n')) or None
    block = []
    for line in s.split('\n'):
        if line.lstrip().startswith('//'):
            block.append(re.sub(r'^\s*//+ ?', '', line))
        else:
            break
    return clean_comment_lines(block) or None


def batch_comment_header(text: str) -> str | None:
    """The leading 'rem' / '::' lines of a Windows batch file (after '@echo off')."""
    block = []
    for line in text.split('\n'):
        s = line.strip()
        if not s or s.lower() in ('@echo off', 'echo off'):
            if block:
                block.append('')
            continue
        m = re.match(r'^(?:@?rem\b ?|::+ ?)(.*)$', s, re.I)
        if not m:
            break
        block.append(m.group(1))
    return clean_comment_lines(block) or None


def powershell_comment_header(text: str) -> str | None:
    """A leading <# ... #> block or run of # lines."""
    s = text.lstrip()
    if s.startswith('<#'):
        end = s.find('#>')
        return clean_comment_lines(s[2:end if end >= 0 else len(s)].split('\n')) or None
    return hash_comment_header(text)


def rosidl_summary(rel: str, text: str, ros_pkg: str | None) -> str:
    """A plain sentence listing the fields of a ROS 2 message (.msg) or service (.srv) definition."""
    kind = {'msg': 'message', 'srv': 'service', 'action': 'action'}[rel.rsplit('.', 1)[-1]]
    stem = rel.rsplit('/', 1)[-1].rsplit('.', 1)[0]
    parts = text.split('\n---\n') if kind != 'message' else [text]

    def fields(block):
        names = []
        for line in block.split('\n'):
            line = line.split('#', 1)[0].strip()
            m = re.match(r'^[\w/\[\]<=]+\s+(\w+)', line)
            if m:
                names.append(m.group(1))
        return names
    full = f'{ros_pkg}/{rel.rsplit(".", 1)[-1]}/{stem}' if ros_pkg else stem
    if kind == 'message':
        return f'ROS 2 message type {full} with the fields: {", ".join(fields(text)) or "(none)"}.'
    req = fields(parts[0])
    ans = fields(parts[1]) if len(parts) > 1 else []
    return (f'ROS 2 {kind} type {full}. The request has: {", ".join(req) or "(nothing)"}; '
            f'the answer has: {", ".join(ans) or "(nothing)"}.')


MAV_CMD_NAMES = {16: 'waypoint', 17: 'loiter', 19: 'timed loiter', 20: 'return-to-launch', 21: 'landing',
                 22: 'take-off', 177: 'jump', 178: 'speed change', 183: 'servo command', 201: 'region of interest',
                 203: 'camera trigger', 206: 'camera trigger distance', 2000: 'camera start'}


def mission_summary(text: str) -> str:
    """A plain sentence describing a QGC WPL 110 mission file (count of each kind of item)."""
    rows = [ln.split('\t') for ln in text.split('\n')[1:] if ln.strip()]
    counts = Counter()
    for row in rows[1:]:          # row 0 = home
        try:
            cmd = int(row[3])
        except (IndexError, ValueError):
            continue
        counts[MAV_CMD_NAMES.get(cmd, f'command {cmd}')] += 1
    parts = [f'{n} {name}' + ('s' if n > 1 and not name.endswith('s') else '') for name, n in counts.items()]
    return (f'Mission file (QGC WPL 110) with {len(rows)} items: home, ' + ', '.join(parts) + '.') if rows \
        else 'Mission file (QGC WPL 110), empty.'


# ---------------------------------------------------------------------------------------------------------
# ROS package facts: package.xml, setup.py entry points, CMake install rules
# ---------------------------------------------------------------------------------------------------------

class RosPackage:
    """What we know about one ROS 2 package folder (a folder that has a package.xml)."""

    def __init__(self, folder: str):
        self.dir = folder
        self.name = folder.rsplit('/', 1)[-1]
        self.build_type = ''
        self.description = ''
        self.comment = None
        self.console_scripts: list[tuple[str, str, str]] = []   # (executable, dotted module, function)
        self.cmake_programs: list[tuple[str, str]] = []         # (source file relative to the package, exe)
        self.cmake_lib_files: list[str] = []                   # helper files installed next to the programs
        self.cmake_python: list[str] = []                      # Python packages installed by ament_cmake_python

    @property
    def workspace(self) -> str | None:
        """The colcon workspace folder (the part before /src/), e.g. 'ros2_ws'."""
        return self.dir.split('/src/', 1)[0] if '/src/' in self.dir else None


def read_package_xml(repo: Path, pkg: RosPackage):
    """Fill name, build type, description and the header comment from package.xml."""
    text = read_text(repo / pkg.dir / 'package.xml')
    pkg.comment = xml_comment_header(text)
    try:
        root = ET.fromstring(text.encode('utf-8'))
    except ET.ParseError:
        m = re.search(r'<name>\s*([\w-]+)\s*</name>', text)
        if m:
            pkg.name = m.group(1)
        return
    pkg.name = (root.findtext('name') or pkg.name).strip()
    pkg.description = ' '.join((root.findtext('description') or '').split())
    pkg.build_type = (root.findtext('export/build_type') or '').strip()


def read_console_scripts(text: str) -> list[tuple[str, str, str]]:
    """setup.py entry_points 'console_scripts': [('new_od', 'detection.new_od', 'main'), ...]."""
    try:
        tree = ast.parse(text)
    except SyntaxError:
        return []
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values, strict=True):
                if isinstance(key, ast.Constant) and key.value == 'console_scripts':
                    for elt in getattr(value, 'elts', []):
                        if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                            m = re.match(r'^\s*([\w.\-]+)\s*=\s*([\w.]+)\s*:\s*([\w.]+)', elt.value)
                            if m:
                                out.append(m.groups())
    return out


def read_cmake_installs(text: str, project: str):
    """CMakeLists.txt: programs installed into lib/<package> (with RENAME), helper files installed there, and
    Python packages installed by ament_python_install_package()."""
    text = re.sub(r'#[^\n]*', '', text)
    programs, lib_files, py_pkgs = [], [], []
    keywords = {'DESTINATION', 'RENAME', 'PERMISSIONS', 'CONFIGURATIONS', 'COMPONENT', 'OPTIONAL',
                'EXCLUDE_FROM_ALL', 'USE_SOURCE_PERMISSIONS'}
    for m in re.finditer(r'\binstall\s*\((.*?)\)', text, re.S | re.I):
        toks = [t.strip('"') for t in m.group(1).split()]
        if not toks or toks[0].upper() not in ('PROGRAMS', 'FILES'):
            continue
        files, i = [], 1
        while i < len(toks) and toks[i].upper() not in keywords:
            files.append(toks[i])
            i += 1
        opts = {}
        while i < len(toks):
            if toks[i].upper() in ('DESTINATION', 'RENAME') and i + 1 < len(toks):
                opts[toks[i].upper()] = toks[i + 1]
                i += 2
            else:
                i += 1
        dest = opts.get('DESTINATION', '').replace('${PROJECT_NAME}', project)
        if dest != f'lib/{project}':
            continue
        if toks[0].upper() == 'PROGRAMS':
            for f in files:
                exe = opts['RENAME'] if 'RENAME' in opts and len(files) == 1 else f.rsplit('/', 1)[-1]
                programs.append((f, exe))
        else:
            lib_files.extend(files)
    for m in re.finditer(r'ament_python_install_package\s*\(\s*([^\s)]+)', text):
        py_pkgs.append(m.group(1).replace('${PROJECT_NAME}', project))
    return programs, lib_files, py_pkgs


# ---------------------------------------------------------------------------------------------------------
# Python analysis (ast)
# ---------------------------------------------------------------------------------------------------------

FuncNode = (ast.FunctionDef, ast.AsyncFunctionDef)
DefNode = (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)
NONE_ASSIGN = ('none',)       # "this name was set to None": does not hide an import of the same name
SHADOW = ('shadow',)          # "this name holds something we cannot follow"


def first_line_of(node) -> int:
    """The line Python reports as co_firstlineno: the first decorator's line, else the def/class line."""
    decorators = getattr(node, 'decorator_list', None) or []
    return min([node.lineno] + [d.lineno for d in decorators])


def iter_defs(stmts):
    """Every def/class inside these statements in source order, not looking inside other defs or lambdas."""
    stack = list(reversed(stmts))
    while stack:
        n = stack.pop()
        if isinstance(n, DefNode):
            yield n
            continue
        if isinstance(n, ast.Lambda):
            continue
        stack.extend(reversed(list(ast.iter_child_nodes(n))))


def own_nodes(stmts):
    """Every AST node of this code that is not inside a nested def/class (lambdas ARE included)."""
    stack = list(reversed(stmts))
    while stack:
        n = stack.pop()
        if isinstance(n, DefNode):
            continue
        yield n
        stack.extend(reversed(list(ast.iter_child_nodes(n))))


def dotted_of(expr) -> list[str] | None:
    """a.b.c -> ['a', 'b', 'c'] when the expression is only names and attributes."""
    parts = []
    while isinstance(expr, ast.Attribute):
        parts.append(expr.attr)
        expr = expr.value
    if isinstance(expr, ast.Name):
        parts.append(expr.id)
        return list(reversed(parts))
    return None


def _assigned_names(target, out: set):
    """Names bound by an assignment target (a, (a, b), [a, *b]); attributes/subscripts bind nothing local."""
    if isinstance(target, ast.Name):
        out.add(target.id)
    elif isinstance(target, (ast.Tuple, ast.List)):
        for elt in target.elts:
            _assigned_names(elt, out)
    elif isinstance(target, ast.Starred):
        _assigned_names(target.value, out)


class PyDef:
    """One class or function in a Python file (same-name duplicates, e.g. a property's getter and setter,
    are merged into one: `nodes` holds every definition, in source order)."""

    __slots__ = ('qual', 'name', 'kind', 'nodes', 'parent', 'fid', 'bindings', 'self_name', 'is_static')

    def __init__(self, qual, name, kind, node, parent):
        self.qual = qual
        self.name = name
        self.kind = kind                     # 'class' | 'function' | 'method'
        self.nodes = [node]
        self.parent = parent                 # PyDef or None (module level)
        self.fid = ''
        self.bindings = None                 # names bound inside a function (filled on demand)
        self.self_name = None                # first parameter of a method ('self' / 'cls')
        self.is_static = False

    @property
    def node(self):
        return self.nodes[0]

    @property
    def is_class(self) -> bool:
        return self.kind == 'class'

    def enclosing_class(self):
        """The class this def is directly inside (methods only)."""
        return self.parent if self.parent is not None and self.parent.is_class else None


class PyModule:
    """One parsed Python file and the names it defines and imports."""

    def __init__(self, rel: str, text: str, mod_id: str):
        self.rel = rel
        self.mod_id = mod_id
        self.text = text
        self.lines = text.split('\n')
        self.error = None
        try:
            self.tree = ast.parse(text)
        except (SyntaxError, ValueError, RecursionError, MemoryError) as exc:
            self.tree = None
            self.error = f'could not be parsed: {exc}'
        self.dotted = None          # importable name, e.g. 'sim_bringup.mission_tool' or 'main_controller'
        self.in_package = False     # True when the file is inside a Python package (folder with __init__.py)
        self.area = ''              # ROS package folder (or the code-map package folder) for import guesses
        self.defs: dict[str, PyDef] = {}
        self.top: dict[str, PyDef] = {}          # module-level defs by name
        self.bindings: dict[str, tuple] = {}     # module-level names
        self.consts: dict[str, str] = {}         # module-level NAME = 'text'
        self.class_info: dict[str, dict] = {}    # class qual -> attr types / constants / publishers ...

    def package_name(self) -> str | None:
        """The Python package this module sits in (for relative imports)."""
        if not self.in_package or not self.dotted:
            return None
        return self.dotted if self.rel.endswith('/__init__.py') else self.dotted.rpartition('.')[0]


def collect_defs(mod: PyModule):
    """Find every class/function/method with the qualified-name rule (outer.inner, Class.method, never
    <locals>), merging same-name duplicates."""
    if mod.tree is None:
        return

    def visit(stmts, parent: PyDef | None):
        for node in iter_defs(stmts):
            prefix = parent.qual + '.' if parent else ''
            qual = prefix + node.name
            if isinstance(node, ast.ClassDef):
                kind = 'class'
            else:
                kind = 'method' if parent is not None and parent.is_class else 'function'
            if qual in mod.defs:
                existing = mod.defs[qual]
                existing.nodes.append(node)
                d = existing
            else:
                d = PyDef(qual, node.name, kind, node, parent)
                d.fid = f'fn:{mod.rel}::{qual}'
                mod.defs[qual] = d
                if parent is None:
                    mod.top[node.name] = d
            if kind == 'method':
                decos = {getattr(x, 'id', getattr(x, 'attr', '')) for x in node.decorator_list}
                d.is_static = 'staticmethod' in decos
                args = node.args.posonlyargs + node.args.args
                if args and not d.is_static:
                    d.self_name = args[0].arg
            visit(node.body, d)

    visit(mod.tree.body, None)


def bindings_of(stmts, params=()) -> dict[str, tuple]:
    """Names bound in one scope: imports, defs, assignments (SHADOW), parameters (SHADOW)."""
    table: dict[str, tuple] = {}

    def bind(name, b):
        old = table.get(name)
        if old is None or old == b:
            table[name] = b
        elif b == NONE_ASSIGN:
            pass
        elif old == NONE_ASSIGN:
            table[name] = b
        else:
            table[name] = SHADOW

    for p in params:
        table[p] = SHADOW
    declared_outer = set()
    for node in own_nodes(stmts):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.asname:
                    bind(alias.asname, ('mod', alias.name))
                else:
                    first = alias.name.split('.')[0]
                    bind(first, ('mod', first))
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                if alias.name == '*':
                    continue
                bind(alias.asname or alias.name, ('from', node.level, node.module or '', alias.name))
        elif isinstance(node, (ast.Global, ast.Nonlocal)):
            declared_outer.update(node.names)
        elif isinstance(node, ast.Assign):
            names = set()
            for t in node.targets:
                _assigned_names(t, names)
            is_none = isinstance(node.value, ast.Constant) and node.value.value is None
            for n in names:
                bind(n, NONE_ASSIGN if is_none else SHADOW)
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            names = set()
            _assigned_names(node.target, names)
            for n in names:
                if isinstance(node, ast.AnnAssign) and node.value is None:
                    continue
                bind(n, SHADOW)
        elif isinstance(node, (ast.For, ast.AsyncFor, ast.comprehension)):
            names = set()
            _assigned_names(node.target, names)
            for n in names:
                bind(n, SHADOW)
        elif isinstance(node, (ast.With, ast.AsyncWith)):
            for item in node.items:
                if item.optional_vars is not None:
                    names = set()
                    _assigned_names(item.optional_vars, names)
                    for n in names:
                        bind(n, SHADOW)
        elif isinstance(node, ast.ExceptHandler) and node.name:
            bind(node.name, SHADOW)
        elif isinstance(node, ast.NamedExpr):
            bind(node.target.id, SHADOW)
    for node in iter_defs(stmts):
        bind(node.name, ('def', node.name))
    for n in declared_outer:
        table.pop(n, None)
    return table


def function_params(node) -> list[str]:
    a = node.args
    names = [x.arg for x in a.posonlyargs + a.args + a.kwonlyargs]
    if a.vararg:
        names.append(a.vararg.arg)
    if a.kwarg:
        names.append(a.kwarg.arg)
    return names


def python_signature(d: PyDef) -> str:
    """'name(arg: type = default) -> result' (or 'class Name(Base)') rebuilt from the tree."""
    node = d.node
    try:
        if isinstance(node, ast.ClassDef):
            bases = [ast.unparse(b) for b in node.bases] + [ast.unparse(k) for k in node.keywords]
            return f'class {node.name}' + (f'({", ".join(bases)})' if bases else '')
        prefix = 'async ' if isinstance(node, ast.AsyncFunctionDef) else ''
        sig = f'{prefix}{node.name}({ast.unparse(node.args)})'
        if node.returns is not None:
            sig += f' -> {ast.unparse(node.returns)}'
        return sig
    except Exception:  # noqa: BLE001 - a signature is a nice extra; never fail the whole map for it
        return node.name


def test_sentence(qual: str) -> str:
    """test_publisher_follows_the_schedule -> 'Test: publisher follows the schedule.' (class name in brackets)."""
    parts = qual.split('.')
    words = re.sub(r'^test_?', '', parts[-1]).replace('_', ' ')
    words = ' '.join(words.split()) or parts[-1]
    cls = next((p for p in reversed(parts[:-1]) if p[:1].isupper()), None)
    ctx = ''
    if cls:
        name = re.sub(r'^Test_?', '', cls)
        name = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', ' ', name).lower().strip()
        if name:
            ctx = f' ({name})'
    return f'Test{ctx}: {words}.'


# ---------------------------------------------------------------------------------------------------------
# Bash analysis
# ---------------------------------------------------------------------------------------------------------

def bash_clean(text: str):
    """Blank out comments and here-document bodies (newlines kept, so line numbers stay right).

    Returns (clean text, per-line flag "this line starts outside any quote" for 1-based line numbers)."""
    out = list(text)
    n = len(text)
    i = 0
    line = 1
    starts_normal = {1: True}
    ctx: list = []                 # stack: 'S' '...', 'D' "...", 'A' $'...', 'C' "$( )" code, 'B' "` `" code
    paren_depth: list = []         # parenthesis depth for each 'C' frame
    pending: list = []             # here-documents waiting for the end of this line: (word, strip_tabs)

    def blank(a, b):
        for k in range(a, b):
            if out[k] != '\n':
                out[k] = ' '

    while i < n:
        c = text[i]
        top = ctx[-1] if ctx else None
        if c == '\\' and top != 'S' and i + 1 < n and text[i + 1] == '\n':
            # a line continuation: the next line belongs to the same command (not a fresh line start)
            line += 1
            starts_normal[line] = False
            i += 2
            continue
        if c == '\n':
            line += 1
            if top is None and pending:
                # skip the here-document bodies that start on the next line
                j = i + 1
                for word, strip_tabs in pending:
                    while j < n:
                        k = text.find('\n', j)
                        k = n if k < 0 else k
                        body = text[j:k]
                        test = body.lstrip('\t') if strip_tabs else body
                        blank(j, k)
                        j = k + 1
                        line += 1
                        if test.strip() == word:
                            break
                pending = []
                i = j
                starts_normal[line] = True
                continue
            starts_normal[line] = top is None
            i += 1
            continue
        if top == 'S':
            if c == "'":
                ctx.pop()
            i += 1
            continue
        if top == 'A':
            if c == '\\':
                i += 2
                continue
            if c == "'":
                ctx.pop()
            i += 1
            continue
        if top == 'D':
            if c == '\\':
                i += 2
                continue
            if c == '"':
                ctx.pop()
            elif text.startswith('$(', i) and not text.startswith('$((', i):
                ctx.append('C')
                paren_depth.append(0)
                i += 2
                continue
            elif c == '`':
                ctx.append('B')
            i += 1
            continue
        # ---- code (top is None, 'C' or 'B') ----
        if c == '\\':
            i += 2
            continue
        if top == 'B' and c == '`':
            ctx.pop()
            i += 1
            continue
        if top == 'C':
            if c == '(':
                paren_depth[-1] += 1
            elif c == ')':
                if paren_depth[-1] == 0:
                    ctx.pop()
                    paren_depth.pop()
                    i += 1
                    continue
                paren_depth[-1] -= 1
        if c == "'":
            ctx.append('A' if i > 0 and text[i - 1] == '$' else 'S')
            i += 1
            continue
        if c == '"':
            ctx.append('D')
            i += 1
            continue
        if c == '#' and (i == 0 or text[i - 1] in ' \t\n;|&()'):
            k = text.find('\n', i)
            k = n if k < 0 else k
            blank(i, k)
            i = k
            continue
        if text.startswith('$((', i) or (c == '(' and text.startswith('((', i)):
            k = text.find('))', i)
            i = n if k < 0 else k + 2
            continue
        if text.startswith('<<<', i):
            i += 3                      # a here-STRING (cmd <<< "text"), not a here-document
            continue
        if text.startswith('<<', i):
            m = re.match(r'<<(-?)\s*(?:\'([^\']+)\'|"([^"]+)"|\\?([A-Za-z_][\w.-]*))', text[i:])
            if m:
                pending.append((m.group(2) or m.group(3) or m.group(4), bool(m.group(1))))
                i += m.end()
                continue
        i += 1
    return ''.join(out), starts_normal


def _match_close(text: str, start: int, open_ch: str, close_ch: str) -> int:
    """Index of the bracket that closes text[start] (an open bracket), ignoring quoted text; -1 if none."""
    depth = 0
    quote = None
    i = start
    n = len(text)
    while i < n:
        c = text[i]
        if quote:
            if c == '\\' and quote == '"':
                i += 2
                continue
            if c == quote:
                quote = None
        elif c == '\\':
            i += 2
            continue
        elif c in ('"', "'"):
            quote = c
        elif c == open_ch:
            depth += 1
        elif c == close_ch:
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return -1


# (no '^': Pattern.match(text, pos) already starts at pos, the first character of a line)
BASH_FUNC_RE = re.compile(r'([ \t]*)(?:function[ \t]+([A-Za-z_][\w:.-]*)[ \t]*(?:\([ \t]*\))?'
                          r'|([A-Za-z_][\w:-]*)[ \t]*\([ \t]*\))[ \t]*')


def bash_functions(clean: str, starts_normal: dict):
    """Function definitions: list of (name, first line, last line, header span, body span) (spans = offsets)."""
    found = []
    offsets = [0]
    for m in re.finditer('\n', clean):
        offsets.append(m.end())
    for idx, start in enumerate(offsets):
        lineno = idx + 1
        if not starts_normal.get(lineno, False):
            continue
        end_of_line = clean.find('\n', start)
        end_of_line = len(clean) if end_of_line < 0 else end_of_line
        m = BASH_FUNC_RE.match(clean, start, end_of_line)
        if not m:
            continue
        name = m.group(2) or m.group(3)
        j = m.end()
        # the body opener may be on the next non-empty line
        k = j
        while k < len(clean) and clean[k] in ' \t\n':
            k += 1
        if k >= len(clean) or clean[k] not in '{(':
            continue
        close = _match_close(clean, k, clean[k], '}' if clean[k] == '{' else ')')
        if close < 0:
            continue
        last = clean.count('\n', 0, close) + 1
        found.append((name, lineno, last, (start + len(m.group(1)), m.end()), (k, close)))
    return found


def _strip_expansions(word: str) -> str:
    """'$(cd ..)/x.sh' / '${DIR}/x.sh' / '$HERE/x.sh' -> the literal text after the last expansion."""
    s = word.replace('"', '').replace("'", '')
    prev = None
    while prev != s:
        prev = s
        s = re.sub(r'\$\([^()]*\)', '\0', s)
    s = re.sub(r'\$\{[^}]*\}|\$\w+|\$.', '\0', s)
    return s.rsplit('\0', 1)[-1]


def bash_split_commands(text: str, base_line: int = 1):
    """Split (comment-free) bash text into simple commands: list of (line, [words]).

    Quotes are removed from words; $( ... ) and `...` contents become commands of their own; redirections
    and their targets are dropped; ';' '&' '|' '(' ')' and newlines end a command."""
    cmds = []
    words: list = []
    cur: list = []
    state = {'line': base_line, 'cmd_line': None, 'skip_next': False, 'has_word': False}

    def flush_word():
        if state['has_word']:
            w = ''.join(cur)
            if state['skip_next']:
                state['skip_next'] = False
            else:
                if not words:
                    state['cmd_line'] = state['line']
                words.append(w)
            cur.clear()
            state['has_word'] = False

    def flush_cmd():
        flush_word()
        if words:
            cmds.append((state['cmd_line'] or state['line'], list(words)))
        words.clear()
        state['skip_next'] = False

    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        if c == '\n':
            flush_cmd()
            state['line'] += 1
            i += 1
            continue
        if c in ' \t':
            flush_word()
            i += 1
            continue
        if c == '\\':
            if i + 1 < n and text[i + 1] == '\n':
                state['line'] += 1
                i += 2
                continue
            cur.append(text[i + 1:i + 2])
            state['has_word'] = True
            i += 2
            continue
        if c == "'":
            j = text.find("'", i + 1)
            j = n if j < 0 else j
            cur.append(text[i + 1:j])
            state['has_word'] = True
            state['line'] += text.count('\n', i, j)
            i = j + 1
            continue
        if c == '"':
            j = i + 1
            buf = []
            while j < n and text[j] != '"':
                if text[j] == '\\' and j + 1 < n:
                    buf.append(text[j + 1])
                    j += 2
                    continue
                if text.startswith('$(', j) and not text.startswith('$((', j):
                    close = _match_close(text, j + 1, '(', ')')
                    close = n - 1 if close < 0 else close
                    cmds.extend(bash_split_commands(text[j + 2:close], state['line'] + text.count('\n', i, j)))
                    buf.append(text[j:close + 1])
                    j = close + 1
                    continue
                buf.append(text[j])
                j += 1
            cur.append(''.join(buf))
            state['has_word'] = True
            state['line'] += text.count('\n', i, j)
            i = j + 1
            continue
        if text.startswith('$((', i):
            j = text.find('))', i)
            j = n - 2 if j < 0 else j
            cur.append(text[i:j + 2])
            state['has_word'] = True
            i = j + 2
            continue
        if text.startswith('$(', i) or text.startswith('<(', i) or text.startswith('>(', i):
            close = _match_close(text, i + 1, '(', ')')
            close = n - 1 if close < 0 else close
            cmds.extend(bash_split_commands(text[i + 2:close], state['line']))
            if text[i] == '$':
                cur.append(text[i:close + 1])
                state['has_word'] = True
            state['line'] += text.count('\n', i, close)
            i = close + 1
            continue
        if c == '`':
            j = text.find('`', i + 1)
            j = n if j < 0 else j
            cmds.extend(bash_split_commands(text[i + 1:j], state['line']))
            cur.append('$(...)')
            state['has_word'] = True
            i = j + 1
            continue
        if text.startswith('${', i):
            close = _match_close(text, i + 1, '{', '}')
            close = n - 1 if close < 0 else close
            cur.append(text[i:close + 1])
            state['has_word'] = True
            i = close + 1
            continue
        if c in '<>' or (c == '&' and text.startswith('&>', i)):
            if state['has_word'] and ''.join(cur).isdigit():
                cur.clear()
                state['has_word'] = False
            flush_word()
            j = i
            while j < n and text[j] in '<>&-0123456789':
                if text[j] == '&' and j > i and j + 1 < n and text[j + 1].isdigit():
                    j += 1
                    continue
                j += 1
            op = text[i:j]
            if not re.search(r'&\d|&-$', op):
                state['skip_next'] = True
            i = j
            continue
        if c in ';&|()':
            flush_cmd()
            i += 1
            continue
        cur.append(c)
        state['has_word'] = True
        i += 1
    flush_cmd()
    return cmds


def bash_command_words(words: list) -> tuple[list, list]:
    """Drop leading keywords, VAR=value assignments and wrappers (exec, nohup, sudo, env, timeout ...).

    Returns (the real command words, the assignment values that were skipped)."""
    assigns = []
    i = 0
    while i < len(words):
        w = words[i]
        if w in ('for', 'select', 'case', 'function'):
            return [], assigns
        if w in BASH_KEYWORDS or w in ('[[', ']]'):
            i += 1
            continue
        m = re.match(r'^[A-Za-z_]\w*(?:\[[^\]]*\])?\+?=(.*)$', w, re.S)
        if m:
            assigns.append(m.group(1))
            i += 1
            continue
        if w in BASH_WRAPPERS:
            i += 1
            while i < len(words) and words[i].startswith('-'):
                i += 1
            continue
        if w == 'sudo':
            i += 1
            while i < len(words) and words[i].startswith('-'):
                i += 2 if words[i] in ('-u', '-g') else 1
            continue
        if w == 'env':
            i += 1
            while i < len(words) and (words[i].startswith('-') or re.match(r'^[A-Za-z_]\w*=', words[i])):
                i += 1
            continue
        if w == 'timeout':
            i += 1
            while i < len(words) and words[i].startswith('-'):
                i += 1
            i += 1
            continue
        break
    return words[i:], assigns


# ---------------------------------------------------------------------------------------------------------
# C# and PowerShell scanners
# ---------------------------------------------------------------------------------------------------------

def cs_clean(text: str):
    """Blank C# comments and string/char literals (newlines kept). Returns (clean text, string literals as
    (start offset, value))."""
    out = list(text)
    strings = []
    i = 0
    n = len(text)

    def blank(a, b):
        for k in range(a, b):
            if out[k] != '\n':
                out[k] = ' '

    while i < n:
        c = text[i]
        if text.startswith('//', i):
            k = text.find('\n', i)
            k = n if k < 0 else k
            blank(i, k)
            i = k
            continue
        if text.startswith('/*', i):
            k = text.find('*/', i + 2)
            k = n if k < 0 else k + 2
            blank(i, k)
            i = k
            continue
        m = re.match(r'(\$@|@\$|@|\$)?"', text[i:i + 3])
        if m:
            prefix = m.group(1) or ''
            j = i + len(prefix) + 1
            buf = []
            verbatim = '@' in prefix
            while j < n:
                ch = text[j]
                if verbatim and ch == '"' and j + 1 < n and text[j + 1] == '"':
                    buf.append('"')
                    j += 2
                    continue
                if not verbatim and ch == '\\' and j + 1 < n:
                    buf.append(text[j:j + 2])
                    j += 2
                    continue
                if ch == '"' or (not verbatim and ch == '\n'):
                    break
                buf.append(ch)
                j += 1
            strings.append((i, ''.join(buf)))
            blank(i + len(prefix) + 1, j)
            i = j + 1
            continue
        if c == "'":
            m2 = re.match(r"'(\\.|[^\\'\n]){1,8}'", text[i:])
            if m2:
                blank(i + 1, i + m2.end() - 1)
                i += m2.end()
                continue
        i += 1
    return ''.join(out), strings


def ps_clean(text: str):
    """Blank PowerShell comments and strings (newlines kept). Returns (clean text, string literals)."""
    out = list(text)
    strings = []
    i = 0
    n = len(text)

    def blank(a, b):
        for k in range(a, b):
            if out[k] != '\n':
                out[k] = ' '

    while i < n:
        c = text[i]
        if text.startswith('<#', i):
            k = text.find('#>', i + 2)
            k = n if k < 0 else k + 2
            blank(i, k)
            i = k
            continue
        if c == '#':
            k = text.find('\n', i)
            k = n if k < 0 else k
            blank(i, k)
            i = k
            continue
        if text.startswith('@"', i) or text.startswith("@'", i):
            q = text[i + 1]
            k = text.find('\n' + q + '@', i + 2)
            k = n if k < 0 else k
            strings.append((i, text[i + 2:k]))
            blank(i + 2, k)
            i = k + 3
            continue
        if c in ('"', "'"):
            j = i + 1
            while j < n:
                if c == '"' and text[j] == '`':
                    j += 2
                    continue
                if text[j] == c:
                    if j + 1 < n and text[j + 1] == c:
                        j += 2
                        continue
                    break
                j += 1
            strings.append((i, text[i + 1:j]))
            blank(i + 1, j)
            i = j + 1
            continue
        i += 1
    return ''.join(out), strings


# ---------------------------------------------------------------------------------------------------------
# The builder: everything above put together into nodes and edges
# ---------------------------------------------------------------------------------------------------------

class Package:
    """One package block: a ROS package folder, its test/ folder, a folder named in systems.yaml, or the files one
    system claims from one folder ("files:" in systems.yaml; id pkg:<folder>@<system key>)."""

    def __init__(self, folder: str, kind: str, recursive: bool):
        self.dir = folder
        self.kind = kind              # 'ros' | 'test' | 'declared' | 'subpkg' | 'files' | 'auto'
        self.recursive = recursive    # True: also owns files in sub-folders that are not packages themselves
        self.system = None
        self.ros: RosPackage | None = None
        self.label = None
        self.summary = None
        self.summary_from_doc = False
        self.pid: str | None = None   # set for a group of claimed files (its folder is not its id)

    @property
    def id(self) -> str:
        return self.pid or f'pkg:{self.dir}'


def number_list(text, count: int | None = None) -> list | None:
    """'310 140 220 140' or '840,490' -> [310.0, 140.0, ...] (None when it is not that many numbers)."""
    parts = re.split(r'[\s,]+', str(text or '').strip())
    try:
        nums = [float(p) for p in parts if p]
    except ValueError:
        return None
    if count is not None and len(nums) != count:
        return None
    return nums


def yaml_false(value) -> bool:
    """True when a systems.yaml value says no ('false', 'no', 'off', '0'; the small reader keeps them as text)."""
    return str(value).strip().lower() in ('false', 'no', 'off', '0')


# The Wiring view's vocabulary (systems.yaml "wire:" and "status:"): kinds of message and how finished a wire is.
WIRE_KINDS = ('mav', 'ros', 'gz', 'cam', 'web', 'files')
WIRE_STATUSES = ('current', 'being built', 'proposal', 'legacy', 'optional')
WIRE_ARROWS = ('to', 'from', 'both')


def glob_to_regex(pattern: str):
    """'ros2_ws/src/*' / '**/test' style folder patterns -> compiled regular expression."""
    out = ''
    i = 0
    while i < len(pattern):
        if pattern.startswith('**/', i):
            out += '(?:.*/)?'
            i += 3
        elif pattern.startswith('**', i):
            out += '.*'
            i += 2
        elif pattern[i] == '*':
            out += '[^/]*'
            i += 1
        elif pattern[i] == '?':
            out += '[^/]'
            i += 1
        else:
            out += re.escape(pattern[i])
            i += 1
    return re.compile('^' + out + '$')


def build_wiring_section(spec, links, node_ids=None):
    """The "wiring" part of codemap.json, read by the explorer's Wiring view: the zones and boxes of
    systems.yaml "wiring:", and one wire for every link that has "wire:". A wire goes from the box that holds the
    link's "from" end (a box holds the ids in its "node" and "members") to the box that holds its "to" end, unless
    wire_from / wire_to name a box. Returns (wiring or None, problems, notes): problems are mistakes (an unknown
    box, kind or status, a wire id used twice, a tour step that names something unknown; --check fails on them),
    notes are harmless (a box whose "node" is not in the map yet). node_ids = the ids of the map (None: no check).
    Used by CodeMapBuilder.build_wiring and by web/make_sample.py (the sample map's Wiring view)."""
    problems, notes = [], []
    if not isinstance(spec, dict) or not spec:
        return None, problems, notes

    def prob(msg):
        problems.append(f'systems.yaml wiring: {msg}')

    def text(value):
        return ' '.join(str(value).split()) if value not in (None, '') else None

    def id_list(value):
        return [str(v) for v in value] if isinstance(value, list) else ([str(value)] if value else [])

    size = number_list(spec.get('size'), 2) or [1600.0, 1200.0]
    out = {'schema': 1, 'title': text(spec.get('title')) or 'How the programs talk',
           'intro': text(spec.get('intro')), 'width': size[0], 'height': size[1],
           'kinds': [], 'statuses': [], 'zones': [], 'boxes': [], 'wires': [], 'steps': []}
    for item, key, allowed in ((spec.get('kinds'), 'kinds', WIRE_KINDS), (spec.get('statuses'), 'statuses',
                                                                            WIRE_STATUSES)):
        for k in item or []:
            if not isinstance(k, dict) or k.get('key') not in allowed:
                prob(f'{key}: "{k.get("key") if isinstance(k, dict) else k}" is not one of {", ".join(allowed)}')
                continue
            out[key].append({'key': k['key'], 'label': text(k.get('label')) or k['key'],
                             'text': text(k.get('text'))})
    zone_ids = set()
    for z in spec.get('zones') or []:
        at = number_list(z.get('at'), 4) if isinstance(z, dict) else None
        if not at or not z.get('id'):
            prob(f'zone {z!r:.60} needs an id and "at: x y width height"')
            continue
        zone_ids.add(z['id'])
        out['zones'].append({'id': z['id'], 'label': text(z.get('label')) or z['id'], 'note': text(z.get('note')),
                             'x': at[0], 'y': at[1], 'w': at[2], 'h': at[3]})
    box_ids, holder = set(), {}
    for b in spec.get('boxes') or []:
        at = number_list(b.get('at'), 4) if isinstance(b, dict) else None
        if not at or not b.get('id'):
            prob(f'box {b!r:.60} needs an id and "at: x y width height"')
            continue
        bid = b['id']
        if bid in box_ids:
            prob(f'the box id "{bid}" is used twice')
            continue
        box_ids.add(bid)
        status = b.get('status') or 'current'
        if status not in WIRE_STATUSES:
            prob(f'box {bid}: unknown status "{status}"')
        node = b.get('node')
        members = id_list(b.get('members')) if 'members' in b else ([node] if node else [])
        if node and node not in members:
            members.insert(0, node)
        if node and node_ids is not None and node not in node_ids:
            notes.append(f'systems.yaml wiring: box {bid} opens "{node}", which is not in the map (yet)')
        for m in members:
            if m in holder and holder[m] != bid:
                prob(f'"{m}" is a member of two boxes ({holder[m]}, {bid}): a wire would not know where to go')
            holder.setdefault(m, bid)
        box = {'id': bid, 'title': text(b.get('title')) or bid, 'x': at[0], 'y': at[1], 'w': at[2], 'h': at[3],
               'status': status, 'members': members}
        for key in ('tag', 'where', 'stands_for', 'explain', 'shape', 'zone'):
            if text(b.get(key)):
                box[key] = text(b.get(key))
        if node:
            box['node'] = node
        for key in ('lines', 'chips'):
            if isinstance(b.get(key), list):
                box[key] = [str(x) for x in b[key]]
        out['boxes'].append(box)
    wire_ids = set()
    for link in links or []:
        if not isinstance(link, dict) or not link.get('wire'):
            continue
        wid = link.get('id')
        if not wid:
            prob(f'the wire {link.get("from")} -> {link.get("to")} needs an "id"')
            continue
        if wid in wire_ids:
            prob(f'the wire id "{wid}" is used twice')
            continue
        wire_ids.add(wid)
        kind, status, arrow = link['wire'], link.get('status') or 'current', link.get('arrow') or 'to'
        for what, value, allowed in (('wire', kind, WIRE_KINDS), ('status', status, WIRE_STATUSES),
                                     ('arrow', arrow, WIRE_ARROWS)):
            if value not in allowed:
                prob(f'wire {wid}: {what} "{value}" is not one of {", ".join(allowed)}')
        ends = []
        for side in ('from', 'to'):
            box = link.get(f'wire_{side}') or holder.get(link.get(side))
            if box not in box_ids:
                named = link.get(f'wire_{side}') or link.get(side)
                prob(f'wire {wid}: no box holds its "{side}" end ({named})')
            ends.append(box)
        if ends[0] == ends[1] and ends[0] is not None:
            prob(f'wire {wid}: both ends are the box {ends[0]}')
        wire = {'id': wid, 'from': ends[0], 'to': ends[1], 'kind': kind, 'status': status, 'arrow': arrow,
                'title': text(link.get('title')) or wid}
        for key in ('via', 'explain'):
            if text(link.get(key)):
                wire[key] = text(link.get(key))
        if not yaml_false(link.get('map', 'true')):
            wire['map_from'], wire['map_to'] = link.get('from'), link.get('to')
            if text(link.get('label')):
                wire['label'] = text(link.get('label'))
        code = id_list(link.get('code'))
        if code:
            wire['code'] = code
        if link.get('path'):
            pts = number_list(link['path'])
            if not pts or len(pts) % 2:
                prob(f'wire {wid}: path must be pairs of numbers, "x,y x,y ..."')
            else:
                wire['path'] = [[pts[i], pts[i + 1]] for i in range(0, len(pts), 2)]
        if link.get('label_at'):
            parts = str(link['label_at']).split()
            xy = number_list(parts[0] if parts else '', 2)
            if not xy or any(p not in ('start', 'middle', 'end', 'stack') for p in parts[1:]):
                prob(f'wire {wid}: label_at must be "x,y" followed by start, middle or end and/or stack')
            else:
                wire['label_at'] = xy + parts[1:]
        if link.get('offset') not in (None, ''):
            off = number_list(link['offset'], 1)
            if off is None:
                prob(f'wire {wid}: offset must be a number')
            else:
                wire['offset'] = off[0]
        out['wires'].append(wire)
    for i, step in enumerate(spec.get('steps') or [], 1):
        if not isinstance(step, dict):
            continue
        wires, boxes = id_list(step.get('wires')), id_list(step.get('boxes'))
        for w in wires:
            if w not in wire_ids:
                prob(f'tour step {i} names the wire "{w}", which does not exist')
        for b in boxes:
            if b not in box_ids:
                prob(f'tour step {i} names the box "{b}", which does not exist')
        out['steps'].append({'title': text(step.get('title')) or f'Step {i}', 'text': text(step.get('text')),
                             'wires': wires, 'boxes': boxes})
    return out, problems, notes


class CodeMapBuilder:
    """Reads the repository and builds the codemap dictionary (see build_codemap)."""

    def __init__(self, repo: Path, systems_file: Path | None, explanations_dir: Path | None,
                 include_source: bool = True, skip_files=()):
        self.repo = Path(repo)
        self.systems_file = systems_file
        self.explanations_dir = explanations_dir
        self.include_source = include_source
        self.skip_files = set(skip_files)
        self.nodes: dict[str, dict] = {}
        self.edges: set = set()
        self.problems: list[str] = []          # things the --check option fails on
        self.warnings: list[str] = []          # things worth reading, not fatal
        self.duplicates: list[str] = []
        self.unassigned: list[str] = []
        self.explain_source: dict[str, str] = {}
        self.unmatched_explanations: list[str] = []
        self.auto_called_when: dict[str, str] = {}
        self.topic_types: dict[str, set] = defaultdict(set)
        self.dynamic_topics: list[str] = []      # file:line of topic names the code computes at run time
        self.service_names: set = set()
        self.stats = Counter()

    # ---------------------------------------------------------------- nodes and edges
    def add_node(self, node: dict) -> dict:
        nid = node['id']
        if nid in self.nodes:
            self.duplicates.append(nid)
            return self.nodes[nid]
        self.nodes[nid] = node
        return node

    def guard(self, where: str, func, *args):
        """Run one per-file step; if it crashes on an odd file, note it as a problem (--check fails) and go on,
        so the rest of the map is still written."""
        try:
            return func(*args)
        except Exception as exc:  # noqa: BLE001 - one strange file must not stop the whole map
            self.problems.append(f'{where}: {getattr(func, "__name__", "step")} failed: {exc!r}')
            return None

    def add_edge(self, src: str, dst: str, kind: str, label: str | None = None):
        if not src or not dst or src == dst:
            return
        assert kind in EDGE_KINDS, kind
        self.edges.add((src, dst, kind, label or ''))

    # ---------------------------------------------------------------- systems.yaml
    def load_systems(self):
        self.systems = []
        self.externals = []
        self.links = []
        self.topic_owners = []
        self.package_info = {}
        self.system_patterns = []
        self.file_patterns = []
        self.wiring_spec = {}
        self.ext_ros_packages = {}
        self.ext_programs = {}
        self.ext_launch_names = {}
        if not self.systems_file or not Path(self.systems_file).is_file():
            self.problems.append(f'systems file not found: {self.systems_file}')
            return
        data = parse_yaml_subset(read_text(self.systems_file), str(self.systems_file))
        self.systems = data.get('systems') or []
        self.externals = data.get('externals') or []
        self.links = data.get('links') or []
        self.topic_owners = data.get('topic_owners') or []
        self.package_info = data.get('package_info') or {}
        self.wiring_spec = data.get('wiring') or {}
        keys = [s.get('key') for s in self.systems]
        for k, n in Counter(keys).items():
            if n > 1:
                self.problems.append(f'systems.yaml: system "{k}" is listed {n} times')
        for s in self.systems:              # (system key, pattern text, regex, literal?)
            for pat in s.get('packages') or []:
                literal = not any(ch in pat for ch in '*?')
                self.system_patterns.append((s['key'], pat, glob_to_regex(pat), literal))
            for pat in s.get('files') or []:      # single files a system claims (see claim_files)
                pat = str(pat).strip('/')
                self.file_patterns.append((s['key'], pat, glob_to_regex(pat), not any(ch in pat for ch in '*?')))
        for e in self.externals:
            eid = f'ext:{e["key"]}'
            for p in e.get('ros_packages') or []:
                self.ext_ros_packages[p] = eid
            for p in e.get('programs') or []:
                self.ext_programs[p] = eid
            for p in e.get('launch_names') or []:
                self.ext_launch_names[p] = eid

    # ---------------------------------------------------------------- packages
    def discover_packages(self):
        self.packages: dict[str, Package] = {}
        self.ros_packages: dict[str, RosPackage] = {}       # folder -> facts
        for rel in sorted(self.everything):
            if rel.endswith('/package.xml') or rel == 'package.xml':
                folder = posix_dirname(rel)
                rp = RosPackage(folder)
                read_package_xml(self.repo, rp)
                self.ros_packages[folder] = rp
                pkg = Package(folder, 'ros', True)
                pkg.ros = rp
                self.packages[folder] = pkg
        # a Python sub-package inside a ROS package (e.g. lattice_bridge/lattice_bridge/sinks) is its own block,
        # in the same system as its ROS package
        self.inherit: dict[str, str] = {}
        for rel in sorted(self.everything):
            if not rel.endswith('/__init__.py'):
                continue
            folder = posix_dirname(rel)
            parent = posix_dirname(folder)
            if f'{parent}/__init__.py' not in self.everything or folder in self.packages:
                continue
            owner = next((f for f in sorted(self.ros_packages, key=len, reverse=True)
                          if folder.startswith(f + '/')), None)
            if owner is None or re.search(r'/tests?(/|$)', folder[len(owner):]):
                continue
            sub = Package(folder, 'subpkg', True)
            top = folder
            while f'{posix_dirname(top)}/__init__.py' in self.everything and posix_dirname(top) != owner:
                top = posix_dirname(top)
            sub.label = folder[len(posix_dirname(top)) + 1:].replace('/', '.')
            self.packages[folder] = sub
            self.inherit[folder] = owner
        # folders named in systems.yaml ("dir" = only the files directly in it; "dir/**" = with sub-folders)
        for _key, pat, _rx, literal in self.system_patterns:
            recursive = pat.endswith('/**')
            folder = pat[:-3] if recursive else pat
            if not recursive and not literal:
                continue
            if any(ch in folder for ch in '*?'):
                continue
            folder = folder.strip('/') or '.'
            if folder in self.packages:
                continue
            if folder != '.' and not any(r.startswith(folder + '/') for r in self.everything):
                self.warnings.append(f'systems.yaml names the folder "{folder}", which has no files')
                continue
            self.packages[folder] = Package(folder, 'declared', recursive and folder != '.')
        # every package's own test/ folder becomes a test package
        for folder in sorted(list(self.packages)):
            if folder == '.':
                continue
            for tname in ('test', 'tests'):
                tdir = f'{folder}/{tname}'
                if tdir in self.packages:
                    continue
                if any(r.startswith(tdir + '/') for r in self.sources):
                    self.packages[tdir] = Package(tdir, 'test', True)
        # labels and summaries
        for folder, pkg in self.packages.items():
            info = self.package_info.get(folder) or {}
            if pkg.kind == 'ros':
                pkg.label = pkg.ros.name
            elif pkg.kind == 'subpkg':
                pass                                   # label set above (dotted Python name)
            elif pkg.kind == 'test':
                parent = self.packages.get(posix_dirname(folder))
                base = parent.ros.name if parent is not None and parent.ros else posix_dirname(folder)
                pkg.label = f'{base} tests'
            else:
                pkg.label = 'top folder of the repo' if folder == '.' else folder
            if info.get('label'):
                pkg.label = info['label']
            if info.get('summary'):
                pkg.summary = ' '.join(str(info['summary']).split())
            else:
                doc = None
                if pkg.ros is not None:
                    doc = pkg.ros.comment or pkg.ros.description
                if not doc and f'{folder}/README.md' in self.everything:
                    readme = read_text(self.repo / folder / 'README.md')
                    doc = xml_comment_header(readme) or readme
                    doc = re.sub(r'^#+ .*\n', '', doc.strip()) if doc else doc
                if not doc and f'{folder}/__init__.py' in self.everything:
                    try:
                        doc = ast.get_docstring(ast.parse(read_text(self.repo / folder / '__init__.py')))
                    except SyntaxError:
                        doc = None
                pkg.summary = purpose_summary(doc)
                pkg.summary_from_doc = bool(pkg.summary)

    def package_of(self, rel: str) -> Package | None:
        """The deepest package owning this file (a flat package only owns the files directly in it)."""
        own = posix_dirname(rel)
        cur = own
        while True:
            pkg = self.packages.get(cur)
            if pkg is not None and (cur == own or pkg.recursive):
                return pkg
            if cur == '.':
                return None
            cur = posix_dirname(cur)

    def assign_systems(self):
        """Put every package into one system. A folder named exactly ("dir" or "dir/**") wins over a pattern
        ("ros2_ws/src/*", "**/test"); "dir/**" only means "this folder with its sub-folders is ONE package", it
        does not claim packages inside it. Between patterns, the one with more fixed characters wins."""
        for folder, pkg in sorted(self.packages.items()):
            best = None
            tie = []
            for key, pat, rx, literal in self.system_patterns:
                if pat.endswith('/**'):
                    if pat[:-3].strip('/') != folder:
                        continue
                    score = (2, len(pat))
                elif literal:
                    if (pat.strip('/') or '.') != folder:
                        continue
                    score = (2, len(pat))
                elif rx.match(folder):
                    score = (1, len(re.sub(r'[*?]', '', pat)))
                else:
                    continue
                if best is None or score > best[0]:
                    best = (score, key)
                    tie = [key]
                elif score == best[0] and key not in tie:
                    tie.append(key)
            if best is None or (best[0][0] < 2 and folder in self.inherit):
                if folder in self.inherit:      # a Python sub-package: the system of its ROS package
                    owner = self.packages.get(self.inherit[folder])
                    pkg.system = owner.system if owner is not None else None
                continue
            if len(tie) > 1:
                self.problems.append(f'package {folder} matches two systems equally well: {", ".join(sorted(tie))}')
            pkg.system = sorted(tie)[0] if len(tie) > 1 else best[1]
        # A package nothing names that sits inside a "dir/**" package (for example a ROS package in a folder that is
        # still being built) belongs to that package's system: the deepest such folder wins.
        recursive = sorted((f for f, p in self.packages.items() if p.kind == 'declared' and p.recursive and p.system),
                           key=len, reverse=True)
        for folder, pkg in sorted(self.packages.items()):
            if pkg.system is not None:
                continue
            outer = next((f for f in recursive if folder.startswith(f + '/')), None)
            if outer is not None:
                pkg.system = self.packages[outer].system

    def claim_files(self):
        """systems.yaml "files:": single files a system claims, even inside another system's package. The claimed
        files of one folder become ONE package block, id pkg:<folder>@<system key> (label and summary from
        package_info["<folder>@<key>"]). Returns {repo path: Package}."""
        claims: dict[str, Package] = {}
        matched = Counter()
        for rel in sorted(self.sources):
            keys = sorted({key for key, _pat, rx, _lit in self.file_patterns if rx.match(rel)})
            for key, pat, rx, _lit in self.file_patterns:
                if rx.match(rel):
                    matched[pat] += 1
            if not keys:
                continue
            if len(keys) > 1:
                self.problems.append(f'{rel} is claimed by two systems in systems.yaml ("files:"): {", ".join(keys)}')
            key = keys[0]
            folder = posix_dirname(rel)
            group = f'{folder}@{key}'
            pkg = self.packages.get(group)
            if pkg is None:
                pkg = Package(folder, 'files', False)
                pkg.pid = f'pkg:{group}'
                pkg.system = key
                info = self.package_info.get(group) or {}
                pkg.label = info.get('label') or f'{folder.rsplit("/", 1)[-1]} (part, files of {folder})'
                if info.get('summary'):
                    pkg.summary = ' '.join(str(info['summary']).split())
                self.packages[group] = pkg
            claims[rel] = pkg
        for group, pkg in self.packages.items():
            if pkg.kind == 'files' and not pkg.summary:
                names = sorted(r.rsplit('/', 1)[-1] for r, p in claims.items() if p is pkg)
                pkg.summary = f'Files of {pkg.dir} that belong here: {", ".join(names)}.'
        for _key, pat, _rx, literal in self.file_patterns:
            if not matched[pat]:
                self.warnings.append(f'systems.yaml claims the file{"" if literal else "s"} "{pat}", which '
                                     f'{"does" if literal else "do"} not exist (yet)')
        return claims

    # ---------------------------------------------------------------- the main build
    def build(self) -> dict:
        self.load_systems()
        self.sources, self.everything, self.skipped = scan_repo(self.repo, self.skip_files)
        self.discover_packages()
        self.assign_systems()
        self.file_claims = self.claim_files()
        self.add_system_nodes()
        self.module_package: dict[str, Package] = {}
        for rel in sorted(self.sources):
            pkg = self.file_claims.get(rel) or self.package_of(rel)
            if pkg is None or pkg.system is None:
                self.unassigned.append(rel)
                pkg = self.fallback_package(rel)
            self.module_package[rel] = pkg
        self.add_package_nodes()
        self.read_ros_facts()
        self.add_module_nodes()
        self.analyze_python()
        self.analyze_bash()
        self.analyze_csharp()
        self.analyze_powershell()
        self.analyze_batch()
        self.analyze_launch_xml()
        self.add_links()
        self.finish_topics()
        self.merge_explanations()
        self.check_edges()
        self.wiring = self.build_wiring()
        return self.result()

    def fallback_package(self, rel: str) -> Package:
        """A package for a file nothing claims: its own folder, put in system "other" (reported by --check)."""
        folder = posix_dirname(rel)
        pkg = self.packages.get(folder)
        if pkg is None:
            pkg = Package(folder, 'auto', False)
            pkg.label = folder
            self.packages[folder] = pkg
        if pkg.system is None:
            if any(s.get('key') == 'other' for s in self.systems):
                pkg.system = 'other'
            else:
                pkg.system = 'unassigned'
                if 'sys:unassigned' not in self.nodes:
                    self.add_node({'id': 'sys:unassigned', 'kind': 'system', 'label': 'Not in any system',
                                   'parent': None, 'summary': 'Files that systems.yaml does not place anywhere.'})
        return pkg

    # ---------------------------------------------------------------- systems, externals, packages
    def add_system_nodes(self):
        for s in self.systems:
            summary = ' '.join(str(s.get('summary') or '').split()) or None
            node = {'id': f'sys:{s["key"]}', 'kind': 'system', 'label': s.get('label') or s['key'],
                    'parent': None}
            if summary:
                node['summary'] = summary
            self.add_node(node)
        known = {s['key'] for s in self.systems}
        for e in self.externals:
            summary = ' '.join(str(e.get('summary') or '').split()) or None
            parent = e.get('system')
            if parent not in known:
                self.problems.append(f'systems.yaml: external "{e["key"]}" names an unknown system "{parent}"')
            node = {'id': f'ext:{e["key"]}', 'kind': 'external', 'label': e.get('label') or e['key'],
                    'parent': f'sys:{parent}' if parent in known else None}
            if summary:
                node['summary'] = summary
            if e.get('process_match'):
                node['process_match'] = list(e['process_match'])
            self.add_node(node)

    def add_package_nodes(self):
        used = {pkg.id for pkg in self.module_package.values()}
        self.package_by_id = {pkg.id: pkg for pkg in self.packages.values()}
        for folder, pkg in sorted(self.packages.items()):
            if pkg.id not in used:
                continue
            if pkg.system is None:
                self.unassigned.append(f'(package) {folder}')
                pkg.system = 'other'
            node = {'id': pkg.id, 'kind': 'package', 'label': pkg.label, 'parent': f'sys:{pkg.system}',
                    'path': pkg.dir}
            if pkg.summary:
                node['summary'] = pkg.summary
            if pkg.kind == 'test':
                node['is_test'] = True
            self.add_node(node)

    def read_ros_facts(self):
        """Entry points of every ROS package: (package, executable) -> module / function."""
        self.exec_map: dict[tuple, dict] = {}
        self.py_aliases: dict[str, str] = {}         # extra trace_index py_files keys -> module rel path
        self.programs: dict[str, str] = {}           # installed program path -> module rel path
        for folder, rp in sorted(self.ros_packages.items()):
            setup = f'{folder}/setup.py'
            if setup in self.sources:
                rp.console_scripts = read_console_scripts(read_text(self.repo / setup))
            cmake = f'{folder}/CMakeLists.txt'
            if cmake in self.everything:
                rp.cmake_programs, rp.cmake_lib_files, rp.cmake_python = read_cmake_installs(
                    read_text(self.repo / cmake), rp.name)
            ws = rp.workspace
            for exe, dotted, func in rp.console_scripts:
                parts = dotted.split('.')
                rel = posix_join(folder, *parts) + '.py'
                if rel not in self.sources:
                    rel = posix_join(folder, *parts, '__init__.py')
                if rel not in self.sources:
                    self.warnings.append(f'{setup}: entry point {exe} -> {dotted} has no file')
                    continue
                self.exec_map[(rp.name, exe)] = {'rel': rel, 'func': func, 'how': 'console'}
                if ws:
                    self.programs[f'{ws}/install/{rp.name}/lib/{rp.name}/{exe}'] = rel
            for src, exe in rp.cmake_programs:
                rel = posix_norm(posix_join(folder, src))
                if rel not in self.sources:
                    continue
                self.exec_map[(rp.name, exe)] = {'rel': rel, 'func': None, 'how': 'cmake'}
                if ws:
                    self.py_aliases[f'{ws}/install/{rp.name}/lib/{rp.name}/{exe}'] = rel
                    self.programs[f'{ws}/install/{rp.name}/lib/{rp.name}/{exe}'] = rel
            for src in rp.cmake_lib_files:
                rel = posix_norm(posix_join(folder, src))
                if rel in self.sources and ws:
                    self.py_aliases[f'{ws}/install/{rp.name}/lib/{rp.name}/{rel.rsplit("/", 1)[-1]}'] = rel

    # ---------------------------------------------------------------- module nodes
    def add_module_nodes(self):
        self.py: dict[str, PyModule] = {}
        self.summary_generated: set = set()       # module ids whose summary was written by this script
        for rel, lang in sorted(self.sources.items()):
            pkg = self.module_package[rel]
            node = {'id': f'mod:{rel}', 'kind': 'module', 'label': rel.rsplit('/', 1)[-1], 'parent': pkg.id,
                    'path': rel, 'lang': lang}
            if self.is_test_path(rel):
                node['is_test'] = True
            if lang not in TEXT_LANGS:
                try:
                    size = (self.repo / rel).stat().st_size
                except OSError:
                    size = 0
                summary = file_kind_summary(rel, lang, size)
                if summary:
                    node['summary'] = summary
                    self.summary_generated.add(node['id'])
                self.add_node(node)
                continue
            try:
                text = read_text(self.repo / rel)
            except OSError as exc:
                self.problems.append(f'{rel}: could not be read ({exc})')
                self.add_node(node)
                continue
            lines = text.split('\n')
            node['line'] = 1
            node['end_line'] = max(1, len(lines) - (lines[-1] == ''))
            doc = None
            summary = None
            if lang in ('backup', 'marker'):
                summary = file_kind_summary(rel, lang, len(text))
                self.summary_generated.add(node['id'])
            elif lang == 'markdown':
                doc, summary = markdown_summary(text)
            elif lang == 'text':
                summary = first_paragraph(text[:3000])
            elif lang == 'python':
                pm = PyModule(rel, text, node['id'])
                self.py[rel] = pm
                if pm.tree is not None:
                    doc = ast.get_docstring(pm.tree)
                if not doc:
                    doc = hash_comment_header(text)
                if pm.error:
                    self.warnings.append(f'{rel}: {pm.error}')
            elif lang in ('bash', 'yaml', 'params', 'config', 'cmake', 'dockerfile'):
                doc = hash_comment_header(text)
            elif lang in ('xml', 'sdf', 'html'):
                doc = xml_comment_header(text)
            elif lang in ('csharp', 'cpp', 'javascript', 'css'):
                doc = slash_comment_header(text)
            elif lang == 'powershell':
                doc = powershell_comment_header(text)
            elif lang == 'batch':
                doc = batch_comment_header(text)
            elif lang == 'rosidl':
                doc = hash_comment_header(text)
                ros = pkg.ros.name if pkg.ros is not None else None
                summary = rosidl_summary(rel, text, ros)
                self.summary_generated.add(node['id'])
            elif lang == 'mission':
                summary = mission_summary(text)
                self.summary_generated.add(node['id'])
            if doc:
                node['doc'] = cap_text(doc, DOC_CAP)
                summary = summary or purpose_summary(doc)
            if summary:
                node['summary'] = summary
            self.add_node(node)

    @staticmethod
    def is_test_path(rel: str) -> bool:
        parts = rel.split('/')
        return any(p in ('test', 'tests') for p in parts[:-1]) or parts[-1].startswith('test_')

    # ---------------------------------------------------------------- Python
    def analyze_python(self):
        mods = [self.py[r] for r in sorted(self.py)]
        # 1. importable names and definitions
        self.dotted_map = defaultdict(list)
        self.stem_map = defaultdict(list)
        for pm in mods:
            self.set_dotted(pm)
            collect_defs(pm)
            if pm.tree is not None:
                pm.bindings = bindings_of(pm.tree.body)
                self.collect_consts(pm)
            if pm.in_package:
                self.dotted_map[pm.dotted].append(pm)
            elif pm.dotted:
                self.stem_map[pm.dotted].append(pm)
        # 2. nodes for classes, functions and methods
        for pm in mods:
            self.add_python_def_nodes(pm)
        # 3. class facts (attribute types, constants, publishers) and edges
        for pm in mods:
            if pm.tree is None:
                continue
            for d in pm.defs.values():
                if d.is_class:
                    self.guard(pm.rel, self.class_facts, pm, d)
        self.entry_points()
        for pm in mods:
            if pm.tree is None:
                continue
            self.guard(pm.rel, self.python_imports, pm)
            self.guard(pm.rel, self.python_owner, pm, None)
            for d in pm.defs.values():
                self.guard(f'{pm.rel}::{d.qual}', self.python_owner, pm, d)
            if pm.rel.endswith('.launch.py'):
                self.guard(pm.rel, self.python_launch, pm)

    def set_dotted(self, pm: PyModule):
        """The name Python would import this file under (package path, or the file stem for scripts)."""
        rel = pm.rel
        folder = posix_dirname(rel)
        ros = None
        cur = folder
        while True:
            if cur in self.ros_packages:
                ros = cur
                break
            if cur == '.':
                break
            cur = posix_dirname(cur)
        stem = rel.rsplit('/', 1)[-1][:-3]
        if f'{folder}/__init__.py' in self.sources and folder != '.':
            top = folder
            while True:
                parent = posix_dirname(top)
                if parent == '.' or parent == ros or f'{parent}/__init__.py' not in self.sources:
                    break
                top = parent
            base = posix_dirname(top)
            inner = rel[len(base) + 1:] if base != '.' else rel
            dotted = inner[:-3].replace('/', '.')
            if dotted.endswith('.__init__'):
                dotted = dotted[:-len('.__init__')]
            pm.dotted = dotted
            pm.in_package = True
        elif stem != '__init__':
            pm.dotted = stem
        pkg = self.module_package.get(rel)
        pm.area = ros or (re.sub(r'/tests?$', '', pkg.dir) if pkg else folder)

    def collect_consts(self, pm: PyModule):
        """Module-level NAME = 'text' constants (used to read topic names)."""
        counts = Counter()
        values = {}
        for node in own_nodes(pm.tree.body):
            if isinstance(node, ast.Assign):
                for t in node.targets:
                    if isinstance(t, ast.Name):
                        counts[t.id] += 1
                        if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
                            values[t.id] = node.value.value
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.value is not None:
                counts[node.target.id] += 1
                if isinstance(node.value, ast.Constant) and isinstance(node.value.value, str):
                    values[node.target.id] = node.value.value
        pm.consts = {k: v for k, v in values.items() if counts[k] == 1}

    def add_python_def_nodes(self, pm: PyModule):
        is_test = self.is_test_path(pm.rel)
        for d in pm.defs.values():
            first = first_line_of(d.node)
            end = getattr(d.node, 'end_lineno', None) or d.node.lineno
            node = {'id': d.fid, 'kind': d.kind, 'label': d.name,
                    'parent': d.parent.fid if d.parent is not None else pm.mod_id,
                    'path': pm.rel, 'lang': 'python', 'line': first, 'end_line': end,
                    'signature': python_signature(d)}
            doc = ast.get_docstring(d.node)
            if doc:
                node['doc'] = cap_text(doc, DOC_CAP)
            if is_test:
                node['is_test'] = True
            if self.include_source:
                parts = []
                for k, n in enumerate(d.nodes):
                    f = first_line_of(n)
                    e = getattr(n, 'end_lineno', n.lineno) or n.lineno
                    if k:
                        parts.append(f'# ---- also defined at line {f} (same name) ----')
                    parts.append(node_text(pm.lines, f, e))
                node['source'] = cap_text('\n'.join(parts), SOURCE_CAP)
            self.add_node(node)

    # ---- name resolution ---------------------------------------------------------------
    def find_module(self, dotted: str, importer: PyModule) -> PyModule | None:
        """The repo module an import name refers to (None for outside libraries or unclear cases)."""
        if not dotted:
            return None
        if '.' not in dotted and not importer.in_package:
            folder = posix_dirname(importer.rel)
            for cand in (posix_join(folder, dotted + '.py'), posix_join(folder, dotted, '__init__.py')):
                if cand in self.py:
                    return self.py[cand]
        cands = self.dotted_map.get(dotted) or []
        if len(cands) == 1:
            return cands[0]
        if len(cands) > 1:
            same = [m for m in cands if m.area == importer.area]
            return same[0] if len(same) == 1 else None
        if '.' not in dotted:
            same = [m for m in self.stem_map.get(dotted, []) if m.area == importer.area]
            if len(same) == 1:
                return same[0]
        return None

    def from_base(self, pm: PyModule, level: int, module: str) -> str | None:
        """The absolute module name of 'from <level dots><module> import ...'."""
        if level == 0:
            return module
        pkg = pm.package_name()
        if pkg is None:
            return None
        parts = pkg.split('.')
        if level - 1 > len(parts) - 1:
            return None
        base = '.'.join(parts[:len(parts) - (level - 1)])
        return f'{base}.{module}' if module else base

    def resolve_binding(self, pm: PyModule, b: tuple, depth: int = 0):
        """What a bound name refers to: ('def', module, PyDef) | ('module', module) | ('ext', dotted) |
        ('const', text) | None."""
        if b is None or depth > 5 or b in (SHADOW, NONE_ASSIGN):
            return None
        if b[0] == 'def':
            d = pm.top.get(b[1])
            return ('def', pm, d) if d else None
        if b[0] == 'local':                      # a def nested in a function: ('local', PyDef)
            return ('def', pm, b[1])
        if b[0] == 'mod':
            target = self.find_module(b[1], pm)
            return ('module', target) if target else ('ext', b[1])
        if b[0] == 'from':
            _, level, module, attr = b
            base = self.from_base(pm, level, module)
            if base is None:
                return None
            sub = self.find_module(f'{base}.{attr}' if base else attr, pm)
            if sub is not None:
                return ('module', sub)
            target = self.find_module(base, pm)
            if target is None:
                return ('ext', f'{base}.{attr}') if level == 0 else None
            return self.lookup_in_module(target, attr, depth + 1)
        return None

    def lookup_in_module(self, pm: PyModule, name: str, depth: int = 0):
        """A module-level name of another module (a def, a constant, or something it imports itself)."""
        if name in pm.top:
            return ('def', pm, pm.top[name])
        if name in pm.consts:
            return ('const', pm.consts[name])
        b = pm.bindings.get(name)
        if b is not None and b[0] in ('mod', 'from'):
            return self.resolve_binding(pm, b, depth + 1)
        return None

    def scope_bindings(self, pm: PyModule, d: PyDef) -> dict:
        """Names bound inside one function (its parameters, local imports, nested defs, assignments)."""
        if d.bindings is None:
            node = d.node
            table = bindings_of(node.body, function_params(node))
            for k, v in list(table.items()):
                if v[0] == 'def':
                    nested = pm.defs.get(f'{d.qual}.{v[1]}')
                    table[k] = ('local', nested) if nested else SHADOW
            d.bindings = table
        return d.bindings

    def lookup(self, pm: PyModule, owner: PyDef | None, name: str):
        """Python's rule: this function, the functions around it (classes are skipped), then the module."""
        scope = owner
        if scope is not None and scope.is_class:
            # code in a class body sees the class's own names first
            q = f'{scope.qual}.{name}'
            if q in pm.defs:
                return ('def', pm, pm.defs[q])
            scope = scope.parent
        while scope is not None:
            if not scope.is_class:
                b = self.scope_bindings(pm, scope).get(name)
                if b is not None:
                    return self.resolve_binding(pm, b)
            scope = scope.parent
        b = pm.bindings.get(name)
        if b is None:
            return None
        return self.resolve_binding(pm, b)

    def class_bases(self, pm: PyModule, cls: PyDef):
        out = []
        for base in cls.node.bases:
            r = self.resolve_expr(pm, cls.parent, base)
            if r and r[0] == 'def' and r[2] is not None and r[2].is_class:
                out.append((r[1], r[2]))
        return out

    def find_method(self, pm: PyModule, cls: PyDef, name: str, seen=None):
        """A method of a class or of one of its (repo) base classes: (module, PyDef) or None."""
        seen = seen or set()
        key = (pm.rel, cls.qual)
        if key in seen:
            return None
        seen.add(key)
        d = pm.defs.get(f'{cls.qual}.{name}')
        if d is not None:
            return (pm, d)
        for bpm, bcls in self.class_bases(pm, cls):
            r = self.find_method(bpm, bcls, name, seen)
            if r:
                return r
        return None

    def resolve_expr(self, pm: PyModule, owner: PyDef | None, expr):
        """Follow a name / attribute chain to what it means (see resolve_binding)."""
        parts = dotted_of(expr)
        if not parts:
            return None
        r = self.lookup(pm, owner, parts[0])
        for attr in parts[1:]:
            if r is None:
                return None
            if r[0] == 'module':
                target = r[1]
                sub = self.find_module(f'{target.dotted}.{attr}', target) if target.dotted else None
                r = ('module', sub) if sub else self.lookup_in_module(target, attr)
            elif r[0] == 'def' and r[2] is not None and r[2].is_class:
                m = self.find_method(r[1], r[2], attr)
                r = ('def', m[0], m[1]) if m else None
            elif r[0] == 'ext':
                r = ('ext', f'{r[1]}.{attr}')
            else:
                return None
        return r

    def method_class(self, owner: PyDef | None):
        """For code inside a method: (the class, the name of its self parameter)."""
        d = owner
        while d is not None:
            cls = d.enclosing_class()
            if d.kind == 'method' and cls is not None:
                return cls, d.self_name
            d = d.parent
        return None, None

    def call_class_of(self, pm: PyModule, owner, value):
        """If `value` is `SomeRepoClass(...)`: (module, class PyDef), else None."""
        if not isinstance(value, ast.Call):
            return None
        r = self.resolve_expr(pm, owner, value.func)
        if r and r[0] == 'def' and r[2] is not None and r[2].is_class:
            return (r[1], r[2])
        return None

    def class_facts(self, pm: PyModule, cls: PyDef):
        """What a class's methods store in self.<attr>: one repo class (its type) or one text (a constant,
        possibly a ROS parameter's default)."""
        types, bad_types = {}, set()
        consts, bad_consts = {}, set()
        params = {}
        class_consts = {}
        for stmt in cls.node.body:
            if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1 and isinstance(stmt.targets[0], ast.Name) \
                    and isinstance(stmt.value, ast.Constant) and isinstance(stmt.value.value, str):
                class_consts[stmt.targets[0].id] = stmt.value.value
        methods = [d for d in pm.defs.values() if d.parent is cls and d.kind == 'method']
        for m in methods:
            for node in own_nodes(m.node.body):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) \
                        and node.func.attr == 'declare_parameter' and len(node.args) >= 2 \
                        and isinstance(node.args[0], ast.Constant) and isinstance(node.args[1], ast.Constant) \
                        and isinstance(node.args[1].value, str):
                    params[node.args[0].value] = node.args[1].value
        for m in methods:
            if not m.self_name:
                continue
            for node in own_nodes(m.node.body):
                targets, value = [], None
                if isinstance(node, ast.Assign):
                    targets, value = node.targets, node.value
                elif isinstance(node, ast.AnnAssign) and node.value is not None:
                    targets, value = [node.target], node.value
                for t in targets:
                    if not (isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name)
                            and t.value.id == m.self_name):
                        continue
                    attr = t.attr
                    if isinstance(value, ast.Constant) and value.value is None:
                        continue
                    ct = self.call_class_of(pm, m, value)
                    if ct is None:
                        bad_types.add(attr)
                    elif attr in types and types[attr][1] is not ct[1]:
                        bad_types.add(attr)
                    else:
                        types[attr] = ct
                    sv = self.param_or_const(value, params)
                    if sv is None:
                        bad_consts.add(attr)
                    elif attr in consts and consts[attr] != sv:
                        bad_consts.add(attr)
                    else:
                        consts[attr] = sv
        pm.class_info[cls.qual] = {
            'types': {k: v for k, v in types.items() if k not in bad_types},
            'consts': {k: v for k, v in consts.items() if k not in bad_consts},
            'params': params, 'class_consts': class_consts,
        }

    @staticmethod
    def param_or_const(value, params) -> tuple | None:
        """('text', None) for 'text'; ('default', 'param name') for self.declare_parameter('p', 'default')...;
        None for anything else."""
        v = value
        if isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == 'str' and v.args:
            v = v.args[0]
        if isinstance(v, ast.Constant) and isinstance(v.value, str):
            return (v.value, None)
        while isinstance(v, ast.Attribute) and v.attr in ('value', 'string_value'):
            v = v.value
        if isinstance(v, ast.Call) and isinstance(v.func, ast.Attribute) and not v.args and \
                v.func.attr == 'get_parameter_value':
            v = v.func.value
        if isinstance(v, ast.Call) and isinstance(v.func, ast.Attribute) and v.args and \
                isinstance(v.args[0], ast.Constant):
            name = v.args[0].value
            if v.func.attr == 'declare_parameter' and len(v.args) >= 2 and isinstance(v.args[1], ast.Constant) \
                    and isinstance(v.args[1].value, str):
                return (v.args[1].value, name)
            if v.func.attr == 'get_parameter' and name in params:
                return (params[name], name)
        return None

    def local_facts(self, pm: PyModule, owner: PyDef):
        """Inside one function: names that always hold one repo class, and names that hold one text."""
        types, bad = {}, set()
        consts, bad_c = {}, set()
        params = {}
        cls, _ = self.method_class(owner)
        if cls is not None:
            params = pm.class_info.get(cls.qual, {}).get('params', {})
        for node in own_nodes(owner.node.body):
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                name, value = node.targets[0].id, node.value
                if isinstance(value, ast.Constant) and value.value is None:
                    continue
                ct = self.call_class_of(pm, owner, value)
                if ct is None or (name in types and types[name][1] is not ct[1]):
                    bad.add(name)
                else:
                    types[name] = ct
                sv = self.param_or_const(value, params)
                if sv is None or (name in consts and consts[name] != sv):
                    bad_c.add(name)
                else:
                    consts[name] = sv
            elif isinstance(node, (ast.AugAssign, ast.AnnAssign, ast.For, ast.With, ast.NamedExpr)):
                names = set()
                tgt = getattr(node, 'target', None)
                if tgt is not None:
                    _assigned_names(tgt, names)
                bad |= names
                bad_c |= names
        return ({k: v for k, v in types.items() if k not in bad},
                {k: v for k, v in consts.items() if k not in bad_c})

    def resolve_call(self, pm: PyModule, owner, func, ltypes):
        """The block a call goes to: (id, label) or None. Conservative: only clear cases."""
        cls, self_name = self.method_class(owner)
        if isinstance(func, ast.Attribute):
            base = func.value
            # self.method(...) / cls.method(...)
            if isinstance(base, ast.Name) and cls is not None and base.id == self_name:
                m = self.find_method(pm, cls, func.attr)
                return (m[1].fid, None) if m else None
            # self.thing.method(...) where self.thing always holds one repo class
            if isinstance(base, ast.Attribute) and isinstance(base.value, ast.Name) and cls is not None \
                    and base.value.id == self_name:
                t = pm.class_info.get(cls.qual, {}).get('types', {}).get(base.attr)
                if t:
                    m = self.find_method(t[0], t[1], func.attr)
                    return (m[1].fid, None) if m else None
                return None
            # super().method(...)
            if isinstance(base, ast.Call) and isinstance(base.func, ast.Name) and base.func.id == 'super' \
                    and cls is not None:
                for bpm, bcls in self.class_bases(pm, cls):
                    m = self.find_method(bpm, bcls, func.attr)
                    if m:
                        return (m[1].fid, None)
                return None
            # local_variable.method(...) where the variable always holds one repo class
            if isinstance(base, ast.Name) and base.id in ltypes:
                t = ltypes[base.id]
                m = self.find_method(t[0], t[1], func.attr)
                return (m[1].fid, None) if m else None
        r = self.resolve_expr(pm, owner, func)
        if r is None or r[0] != 'def' or r[2] is None:
            return None
        return (r[2].fid, 'creates') if r[2].is_class else (r[2].fid, None)

    def str_value(self, pm: PyModule, owner, expr, lconsts, depth=0):
        """A text value for a topic name: ('text', parameter name or None) or None."""
        if depth > 4 or expr is None:
            return None
        if isinstance(expr, ast.Constant) and isinstance(expr.value, str):
            return (expr.value, None)
        if isinstance(expr, ast.JoinedStr):
            if all(isinstance(v, ast.Constant) for v in expr.values):
                return (''.join(v.value for v in expr.values), None)
            return None
        if isinstance(expr, ast.Call) and isinstance(expr.func, ast.Name) and expr.func.id == 'str' and expr.args:
            return self.str_value(pm, owner, expr.args[0], lconsts, depth + 1)
        cls, self_name = self.method_class(owner)
        if isinstance(expr, ast.Name):
            if expr.id in lconsts:
                return lconsts[expr.id]
            if expr.id in pm.consts and pm.bindings.get(expr.id, SHADOW) == SHADOW:
                return (pm.consts[expr.id], None)
            r = self.lookup(pm, owner, expr.id)
            if r and r[0] == 'const':
                return (r[1], None)
            return None
        if isinstance(expr, ast.Attribute):
            if isinstance(expr.value, ast.Name) and cls is not None and expr.value.id == self_name:
                info = pm.class_info.get(cls.qual, {})
                if expr.attr in info.get('consts', {}):
                    return info['consts'][expr.attr]
                if expr.attr in info.get('class_consts', {}):
                    return (info['class_consts'][expr.attr], None)
                return None
            r = self.resolve_expr(pm, owner, expr)
            if r and r[0] == 'const':
                return (r[1], None)
            return None
        params = pm.class_info.get(cls.qual, {}).get('params', {}) if cls is not None else {}
        return self.param_or_const(expr, params)

    def msg_type(self, pm: PyModule, owner, expr) -> str | None:
        """'std_msgs/msg/String' for a message/service class imported from a ROS interface package."""
        parts = dotted_of(expr)
        if not parts:
            return None
        r = None
        try:
            r = self.resolve_expr(pm, owner, expr)
        except RecursionError:
            r = None
        if r and r[0] == 'ext':
            segs = r[1].split('.')
            if len(segs) >= 3 and segs[-2] in ('msg', 'srv', 'action'):
                return '/'.join(segs[-3:])
            return r[1]
        if len(parts) == 1:
            # imported under try/except with a None fallback: read the import statement itself
            b = pm.bindings.get(parts[0])
            if b and b[0] == 'from' and b[2]:
                segs = b[2].split('.') + [b[3]]
                if len(segs) >= 3 and segs[-2] in ('msg', 'srv', 'action'):
                    return '/'.join(segs[-3:])
        return '.'.join(parts)

    @staticmethod
    def topic_name(raw: str) -> str | None:
        raw = raw.strip()
        if not raw or raw.startswith('~') or any(c in raw for c in ' {}$<>'):
            return None
        return raw if raw.startswith('/') else '/' + raw

    def callback_target(self, pm, owner, expr, ltypes):
        """The function a callback argument points to (self.method, a function name, or a lambda's call)."""
        if expr is None:
            return None
        if isinstance(expr, ast.Lambda):
            # lambda m: emit(state.on_state(m.armed)) -> prefer the call that gets the message itself (m)
            params = {a.arg for a in expr.args.args}
            best = None
            for c in (n for n in ast.walk(expr.body) if isinstance(n, ast.Call)):
                r = self.resolve_call(pm, owner, c.func, ltypes)
                if not r or r[1] is not None:
                    continue
                direct = any(isinstance(a, ast.Name) and a.id in params or
                             isinstance(a, ast.Attribute) and isinstance(a.value, ast.Name) and a.value.id in params
                             for a in list(c.args) + [k.value for k in c.keywords])
                if best is None or (direct and not best[1]):
                    best = (r[0], direct)
            return best[0] if best else None
        if isinstance(expr, (ast.Name, ast.Attribute)):
            r = self.resolve_call(pm, owner, expr, ltypes)
            if r and r[1] is None:
                return r[0]
        return None

    def python_imports(self, pm: PyModule):
        for node in ast.walk(pm.tree):
            targets = []
            if isinstance(node, ast.Import):
                for alias in node.names:
                    targets.append(self.find_module(alias.name, pm))
            elif isinstance(node, ast.ImportFrom):
                base = self.from_base(pm, node.level, node.module or '')
                if base is None:
                    continue
                for alias in node.names:
                    sub = self.find_module(f'{base}.{alias.name}' if base else alias.name, pm) \
                        if alias.name != '*' else None
                    targets.append(sub or self.find_module(base, pm))
            for t in targets:
                if t is not None and t is not pm:
                    self.add_edge(pm.mod_id, t.mod_id, 'imports')

    def python_owner(self, pm: PyModule, owner: PyDef | None):
        """Calls, topics, timers and 'runs' made by one block's own code (module, class body or function)."""
        if owner is None:
            stmts = pm.tree.body
            src = pm.mod_id
        else:
            stmts = owner.node.body
            src = owner.fid
        ltypes, lconsts = ({}, {})
        if owner is not None and not owner.is_class:
            ltypes, lconsts = self.local_facts(pm, owner)
        cls, self_name = self.method_class(owner)
        cls_src = cls.fid if cls is not None else src
        local_pubs = {}
        # "sub = node.create_subscription" then "sub(Type, '/topic', cb)": remember such short names
        aliases = {}
        for node in own_nodes(stmts):
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) \
                    and isinstance(node.value, ast.Attribute) and node.value.attr in (
                        'create_publisher', 'create_subscription', 'create_client', 'create_service'):
                aliases[node.targets[0].id] = node.value.attr
        for node in own_nodes(stmts):
            if isinstance(node, (ast.List, ast.Tuple)):
                self.python_runs(pm, src, node)
                continue
            if not isinstance(node, ast.Call):
                continue
            try:
                target = self.resolve_call(pm, owner, node.func, ltypes)
            except RecursionError:
                target = None
            if target:
                self.add_edge(src, target[0], 'calls', target[1])
                if owner is None and isinstance(node.func, ast.Name) and self.under_main_guard(pm, node):
                    self.auto_called_when.setdefault(target[0], 'When the file is run as a program.')
            func = node.func
            if isinstance(func, ast.Name) and func.id in aliases:
                self.python_topic(pm, owner, node, aliases[func.id], src, cls_src, ltypes, lconsts, local_pubs)
                continue
            if not isinstance(func, ast.Attribute):
                continue
            kind = func.attr
            if kind in ('create_publisher', 'create_subscription', 'create_client', 'create_service'):
                self.python_topic(pm, owner, node, kind, src, cls_src, ltypes, lconsts, local_pubs)
            elif kind == 'create_timer' and node.args:
                cb = node.args[1] if len(node.args) > 1 else next(
                    (k.value for k in node.keywords if k.arg == 'callback'), None)
                tid = self.callback_target(pm, owner, cb, ltypes)
                if tid:
                    p = node.args[0]
                    if isinstance(p, ast.Constant) and isinstance(p.value, (int, float)):
                        text = f'Every {p.value:g} seconds (a ROS timer).'
                    else:
                        text = 'Again and again on a ROS timer.'
                    self.auto_called_when.setdefault(tid, text)
            elif kind == 'publish' and isinstance(func.value, ast.Name) and func.value.id in local_pubs:
                topic, label = local_pubs[func.value.id]
                self.add_edge(src, f'topic:{topic}', 'publishes', label)

    def under_main_guard(self, pm: PyModule, call) -> bool:
        """True when a module-level call sits inside `if __name__ == '__main__':`."""
        for stmt in pm.tree.body:
            if isinstance(stmt, ast.If) and 'name' in ast.dump(stmt.test) and '__main__' in ast.dump(stmt.test):
                for n in ast.walk(stmt):
                    if n is call:
                        return True
        return False

    def python_topic(self, pm, owner, node, kind, src, cls_src, ltypes, lconsts, local_pubs):
        args = list(node.args)
        kw = {k.arg: k.value for k in node.keywords if k.arg}
        type_expr = args[0] if args else kw.get('msg_type', kw.get('srv_type'))
        name_expr = args[1] if len(args) > 1 else kw.get('topic', kw.get('srv_name'))
        sv = self.str_value(pm, owner, name_expr, lconsts)
        topic = self.topic_name(sv[0]) if sv is not None else None
        if topic is None:
            self.stats['dynamic topic names (not resolved)'] += 1
            self.dynamic_topics.append(f'{pm.rel}:{node.lineno}')
            return
        mtype = self.msg_type(pm, owner, type_expr) if type_expr is not None else None
        service = kind in ('create_client', 'create_service')
        if service:
            self.service_names.add(topic)
        if mtype:
            self.topic_types[topic].add(mtype)
        label = mtype or ''
        if sv[1]:
            label = (label + ' ' if label else '') + f'(default of the parameter {sv[1]})'
        tid = f'topic:{topic}'
        self.ensure_topic(topic)
        cls, self_name = self.method_class(owner)
        holder = self.assigned_holder(owner, node)
        if kind == 'create_publisher':
            users = self.attr_users(pm, cls, self_name, holder, ('publish',)) if holder and holder[0] == 'self' \
                else []
            if holder and holder[0] == 'local':
                local_pubs[holder[1]] = (topic, label)
                if any(isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == 'publish'
                       and isinstance(n.func.value, ast.Name) and n.func.value.id == holder[1]
                       for n in own_nodes(owner.node.body if owner else pm.tree.body)):
                    return
            for u in users:
                self.add_edge(u, tid, 'publishes', label)
            if not users:
                self.add_edge(cls_src, tid, 'publishes', label)
        elif kind == 'create_client':
            users = self.attr_users(pm, cls, self_name, holder, ('call_async', 'call')) \
                if holder and holder[0] == 'self' else []
            for u in users:
                self.add_edge(u, tid, 'calls', ('service client ' + label).strip())
            if not users:
                self.add_edge(cls_src if holder and holder[0] == 'self' else src, tid, 'calls',
                              ('service client ' + label).strip())
        else:
            cb = args[2] if len(args) > 2 else kw.get('callback')
            target = self.callback_target(pm, owner, cb, ltypes)
            if kind == 'create_service':
                self.add_edge(target or cls_src, tid, 'subscribes', ('serves ' + label).strip())
                if target:
                    self.auto_called_when.setdefault(
                        target, f'When another program calls the ROS service {topic}.')
            else:
                self.add_edge(target or cls_src, tid, 'subscribes', label)
                if target:
                    self.auto_called_when.setdefault(target, f'Every time a message arrives on {topic}.')

    @staticmethod
    def assigned_holder(owner, call):
        """Where a create_publisher(...) result is stored: ('self', attr) or ('local', name) or None."""
        stmts = owner.node.body if owner is not None else []
        for node in own_nodes(stmts):
            if isinstance(node, ast.Assign) and node.value is call and len(node.targets) == 1:
                t = node.targets[0]
                if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name):
                    return ('self', t.attr)
                if isinstance(t, ast.Name):
                    return ('local', t.id)
        return None

    def attr_users(self, pm, cls, self_name, holder, methods):
        """Methods of the class that call self.<attr>.publish(...) (or call_async ...)."""
        if cls is None:
            return []
        users = []
        for d in pm.defs.values():
            if d.parent is not cls or d.kind != 'method' or not d.self_name:
                continue
            for n in own_nodes(d.node.body):
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr in methods:
                    v = n.func.value
                    if isinstance(v, ast.Attribute) and v.attr == holder[1] and isinstance(v.value, ast.Name) \
                            and v.value.id == d.self_name:
                        users.append(d.fid)
                        break
        return users

    def python_runs(self, pm: PyModule, src: str, node):
        """['ros2', 'run', pkg, exe] / ['ros2', 'launch', pkg, file] / ['bash', 'x.sh'] lists in Python code."""
        vals = []
        for elt in node.elts:
            if isinstance(elt, ast.Constant) and isinstance(elt.value, str):
                vals.append(elt.value)
            else:
                break
        if len(vals) >= 4 and vals[0] == 'ros2' and vals[1] in ('run', 'launch'):
            self.ros2_target(src, vals[1], vals[2], vals[3], 'runs')
        elif len(vals) >= 2 and vals[0] in ('bash', 'sh') and vals[1].endswith('.sh'):
            rel = self.resolve_script(vals[1], pm.rel)
            if rel:
                self.add_edge(src, f'mod:{rel}', 'runs')

    def ros2_target(self, src: str, verb: str, pkg: str, what: str, kind: str):
        """Edge for `ros2 run pkg exe` / `ros2 launch pkg file` (to our module, or to an external program).

        A variable in the package name means we cannot know; a variable in the file name is fine for an
        external program (`ros2 launch mavros $MAVROS_LAUNCH` still starts MAVROS)."""
        if '$' in pkg:
            return
        known = '$' not in what
        if known and verb == 'run':
            e = self.exec_map.get((pkg, what))
            if e:
                self.add_edge(src, f'mod:{e["rel"]}', kind, f'ros2 run {pkg} {what}')
                return
        elif known:
            rel = self.launch_file_in(pkg, what)
            if rel:
                self.add_edge(src, f'mod:{rel}', kind, f'ros2 launch {pkg} {what}')
                return
        ext = self.ext_ros_packages.get(pkg)
        if ext:
            self.add_edge(src, ext, kind, f'ros2 {verb} {pkg} {what}' if known else f'ros2 {verb} {pkg} ...')
        elif known and any(rp.name == pkg for rp in self.ros_packages.values()):
            self.warnings.append(f'{src}: "ros2 {verb} {pkg} {what}" names nothing that package provides')

    def launch_file_in(self, pkg: str, name: str) -> str | None:
        """The repo file of launch file `name` in ROS package `pkg` (searched by file name)."""
        folders = [f for f, rp in self.ros_packages.items() if rp.name == pkg]
        base = name.rsplit('/', 1)[-1]
        for folder in folders:
            for rel in self.sources:
                if rel.startswith(folder + '/') and rel.rsplit('/', 1)[-1] == base:
                    return rel
        return None

    def entry_points(self):
        """entry_point text on the function a console script names (and on its module)."""
        by_rel = defaultdict(list)
        for (pkg, exe), e in sorted(self.exec_map.items()):
            text = f'ros2 run {pkg} {exe}'
            by_rel[e['rel']].append(text)
            if e['func']:
                fid = f'fn:{e["rel"]}::{e["func"]}'
                node = self.nodes.get(fid)
                if node is not None:
                    node['entry_point'] = '; '.join(filter(None, [node.get('entry_point'), text]))
                    self.auto_called_when.setdefault(fid, f'When you start the program: {text}')
                else:
                    self.warnings.append(f'entry point {text} names {e["func"]}, which is not in {e["rel"]}')
        for rel, texts in by_rel.items():
            node = self.nodes.get(f'mod:{rel}')
            if node is not None:
                node['entry_point'] = '; '.join(texts)

    # ---- launch files ------------------------------------------------------------------
    def python_launch(self, pm: PyModule):
        """Which programs a *.launch.py file starts (Node(package=, executable=), ExecuteProcess, includes)."""
        src = pm.mod_id
        launch_basenames = defaultdict(list)
        for rel in self.sources:
            base = rel.rsplit('/', 1)[-1]
            if re.search(r'\.launch(\.py|\.xml)?$', base):
                launch_basenames[base].append(rel)
        for node in ast.walk(pm.tree):
            if isinstance(node, ast.Call):
                name = node.func.attr if isinstance(node.func, ast.Attribute) else getattr(node.func, 'id', '')
                kw = {k.arg: k.value for k in node.keywords if k.arg}

                def lit(key, *alts, kw=kw):
                    """The text value of the first of these keyword arguments that is a plain text."""
                    for k in (key,) + alts:
                        v = kw.get(k)
                        if isinstance(v, ast.Constant) and isinstance(v.value, str):
                            return v.value
                    return None
                if name == 'Node':
                    pkg, exe = lit('package'), lit('executable', 'node_executable')
                    if pkg and exe:
                        self.ros2_target(src, 'run', pkg, exe, 'launches')
                elif name == 'ExecuteProcess':
                    pname = lit('name')
                    hit = self.ext_launch_names.get(pname) if pname else None
                    if hit:
                        self.add_edge(src, hit, 'launches', f'process {pname}')
                    elif pname:
                        cands = [k for k in self.exec_map if k[1] == pname]
                        if len(cands) == 1:
                            self.ros2_target(src, 'run', cands[0][0], pname, 'launches')
                    cmd = kw.get('cmd')
                    if isinstance(cmd, (ast.List, ast.Tuple)) and cmd.elts and \
                            isinstance(cmd.elts[0], ast.Constant) and isinstance(cmd.elts[0].value, str):
                        prog = cmd.elts[0].value.rsplit('/', 1)[-1]
                        if prog in self.ext_programs:
                            self.add_edge(src, self.ext_programs[prog], 'launches', f'process {prog}')
                else:
                    strs = [a.value for a in node.args if isinstance(a, ast.Constant) and isinstance(a.value, str)]
                    if len(strs) >= 2 and (strs[0], strs[1]) in self.exec_map:
                        self.ros2_target(src, 'run', strs[0], strs[1], 'launches')
        # launch files named by a text, e.g. os.path.join(get_package_share_directory('main'), 'launch',
        # 'tracker.launch.xml'): a file name two packages share is settled by a package name in the same call
        parents = {}
        for node in ast.walk(pm.tree):
            for child in ast.iter_child_nodes(node):
                parents[child] = node
        ros_names = {rp.name: folder for folder, rp in self.ros_packages.items()}
        for node in ast.walk(pm.tree):
            if not (isinstance(node, ast.Constant) and isinstance(node.value, str)):
                continue
            base = node.value.rsplit('/', 1)[-1]
            rels = [r for r in launch_basenames.get(base, []) if r != pm.rel]
            if len(rels) > 1:
                call = parents.get(node)
                while call is not None and not isinstance(call, ast.Call):
                    call = parents.get(call)
                hints = set()
                if call is not None:
                    for n in ast.walk(call):
                        if isinstance(n, ast.Constant) and isinstance(n.value, str) and n.value in ros_names:
                            hints.add(ros_names[n.value])
                rels = [r for r in rels if any(r.startswith(h + '/') for h in hints)]
            if len(rels) == 1:
                self.add_edge(src, f'mod:{rels[0]}', 'launches', 'includes this launch file')

    def analyze_launch_xml(self):
        for rel, lang in sorted(self.sources.items()):
            if lang == 'xml' and re.search(r'\.launch(\.xml)?$', rel):
                self.guard(rel, self.launch_xml_file, rel)

    def launch_xml_file(self, rel: str):
        """Which programs one *.launch.xml file starts (<node>, <include>, <executable>)."""
        text = read_text(self.repo / rel)
        src = f'mod:{rel}'
        try:
            root = ET.fromstring(text.encode('utf-8'))
            elements = list(root.iter())
        except ET.ParseError:
            self.warnings.append(f'{rel}: XML could not be parsed; read with simple text rules')
            stripped = re.sub(r'<!--.*?-->', '', text, flags=re.S)
            elements = []
            for m in re.finditer(r'<(node|include|executable)\b([^>]*)>', stripped):
                el = ET.Element(m.group(1), dict(re.findall(r'([\w-]+)="([^"]*)"', m.group(2))))
                elements.append(el)
        for el in elements:
            if el.tag == 'node':
                pkg = el.get('pkg') or el.get('package')
                exe = el.get('exec') or el.get('executable') or el.get('type')
                if pkg and exe:
                    self.ros2_target(src, 'run', pkg, exe, 'launches')
            elif el.tag == 'include':
                f = el.get('file') or ''
                m = re.match(r'\$\(find-pkg-share\s+([\w-]+)\)/(.+)$', f)
                if m:
                    self.ros2_target(src, 'launch', m.group(1), m.group(2), 'launches')
            elif el.tag == 'executable':
                prog = (el.get('cmd') or '').split()[:1]
                if prog and prog[0].rsplit('/', 1)[-1] in self.ext_programs:
                    self.add_edge(src, self.ext_programs[prog[0].rsplit('/', 1)[-1]], 'launches')

    # ---------------------------------------------------------------- script references
    def resolve_script(self, word: str, from_rel: str, exts=('.sh', '.bash', '.py', '.ps1', '.bat')) -> str | None:
        """Which repo file a path word in a script means ('$HERE/../setup/x.sh', 'simulation/scripts/demo.sh').

        Tried in order: relative to the script's folder, relative to the repo root, then a unique file whose
        path ends the same way (same folder preferred). Unclear -> None."""
        tail = _strip_expansions(word).replace('\\', '/').strip().lstrip('/')
        if not tail or not tail.endswith(exts):
            return None
        here = posix_dirname(from_rel)
        for cand in (posix_norm(posix_join(here, tail)), posix_norm(tail)):
            if cand and cand in self.sources:
                return cand
        clean = posix_norm(re.sub(r'^(\.\./|\./)+', '', tail))
        if not clean or clean == '.':
            return None
        # '~/repo/simulation/scripts/x.sh' or '/home/u/<repo>/...': try shorter and shorter path endings
        parts = clean.split('/')
        for k in range(len(parts)):
            ending = '/'.join(parts[k:])
            hits = [r for r in self.sources if r == ending or r.endswith('/' + ending)]
            if len(hits) == 1:
                return hits[0]
            if len(hits) > 1:
                same = [r for r in hits if posix_dirname(r) == here]
                return same[0] if len(same) == 1 else None
        return None

    # ---------------------------------------------------------------- Bash
    def analyze_bash(self):
        self.sh: dict[str, dict] = {}
        for rel, lang in sorted(self.sources.items()):
            if lang == 'bash':
                self.guard(rel, self.bash_file, rel)
        # who sources whom (needed to find functions defined in a sourced file such as _common.sh)
        for rel, info in self.sh.items():
            for _line, words in bash_split_commands(self.blank_func_headers(info)):
                cmd, _ = bash_command_words(words)
                if len(cmd) >= 2 and cmd[0] in ('source', '.'):
                    target = self.resolve_script(cmd[1], rel, ('.sh', '.bash'))
                    if target and target in self.sh and target != rel:
                        info['sources'].append(target)
        for rel in sorted(self.sh):
            self.guard(rel, self.bash_edges, self.sh[rel])

    def bash_file(self, rel: str):
        """The functions of one bash script (one block each); remembers the file for the edge pass."""
        text = read_text(self.repo / rel)
        clean, starts = bash_clean(text)
        funcs = bash_functions(clean, starts)
        lines = text.split('\n')
        info = {'rel': rel, 'text': text, 'clean': clean, 'funcs': funcs, 'names': {}, 'sources': []}
        is_test = self.is_test_path(rel)
        for name, first, last, _hdr, _body in funcs:
            fid = f'fn:{rel}::{name}'
            info['names'][name] = fid
            if fid in self.nodes:
                # redefined later in the same file: keep one block, show both texts
                if self.include_source:
                    node = self.nodes[fid]
                    node['source'] = cap_text(node.get('source', '') + f'\n# ---- also defined at line {first} '
                                              f'(same name) ----\n' + node_text(lines, first, last), SOURCE_CAP)
                continue
            node = {'id': fid, 'kind': 'function', 'label': name, 'parent': f'mod:{rel}', 'path': rel,
                    'lang': 'bash', 'line': first, 'end_line': last, 'signature': f'{name}()'}
            doc = self.bash_function_doc(lines, first)
            if doc:
                node['doc'] = cap_text(doc, DOC_CAP)
            if is_test:
                node['is_test'] = True
            if self.include_source:
                node['source'] = cap_text(node_text(lines, first, last), SOURCE_CAP)
            self.add_node(node)
        self.sh[rel] = info

    @staticmethod
    def bash_function_doc(lines, first) -> str | None:
        """Comment lines right above a bash function, or a '# comment' after its opening brace."""
        block = []
        k = first - 2
        while k >= 0 and lines[k].lstrip().startswith('#') and not lines[k].lstrip().startswith('#!'):
            block.insert(0, re.sub(r'^\s*#+ ?', '', lines[k]))
            k -= 1
        text = clean_comment_lines(block)
        if not text:
            m = re.search(r'\{\s*#\s*(.+)$', lines[first - 1])
            text = m.group(1).strip() if m else ''
        return text or None

    @staticmethod
    def blank_func_headers(info) -> str:
        """The clean text with every 'name() {' header blanked (so a definition is not read as a call)."""
        chars = list(info['clean'])
        for _name, _first, _last, (a, b), _body in info['funcs']:
            for i in range(a, b):
                if chars[i] != '\n':
                    chars[i] = ' '
        return ''.join(chars)

    def bash_known(self, rel, depth=0, seen=None):
        """Functions callable from this script: its own, then those of the files it sources."""
        seen = seen or set()
        if rel in seen or depth > 3:
            return {}
        seen.add(rel)
        info = self.sh[rel]
        known = {}
        sourced = {}
        for target in info['sources']:
            for name, fid in self.bash_known(target, depth + 1, seen).items():
                sourced.setdefault(name, set()).add(fid)
        for name, fids in sourced.items():
            if len(fids) == 1:
                known[name] = next(iter(fids))
        known.update(info['names'])
        return known

    def bash_edges(self, info):
        rel = info['rel']
        known = self.bash_known(rel)
        funcs = info['funcs']
        text = self.blank_func_headers(info)

        def owner_of(line):
            best = None
            for name, first, last, _h, _b in funcs:
                if first <= line <= last and (best is None or first >= best[1]):
                    best = (name, first)
            return f'fn:{rel}::{best[0]}' if best else f'mod:{rel}'

        for line, words in bash_split_commands(text):
            self.bash_command(rel, owner_of(line), words, known, 0)

    def bash_command(self, rel, src, words, known, depth):
        """Edges for one simple bash command (calls, runs, imports, topics), recursing into command strings."""
        if depth > 3:
            return
        cmd, assigns = bash_command_words(words)
        for value in assigns:
            v = value.strip()
            m = re.match(r'^\$\{\w+:?[-=](.*)\}$', v, re.S)
            if m:
                v = m.group(1).strip()
            if re.match(r'^(ros2|bash|python3?|\./|source)\b', v):
                for _l, sub in bash_split_commands(v):
                    self.bash_command(rel, src, sub, known, depth + 1)
        if not cmd:
            return
        w0 = cmd[0]
        base0 = w0.rsplit('/', 1)[-1]
        if w0 in PRINT_COMMANDS and w0 not in known:
            return
        if w0 in known:
            self.add_edge(src, known[w0], 'calls')
            return
        if w0 == 'trap' and len(cmd) >= 2:
            for _l, sub in bash_split_commands(cmd[1]):
                sc, _ = bash_command_words(sub)
                if sc and sc[0] in known:
                    self.add_edge(src, known[sc[0]], 'calls', 'trap handler')
                    sig = ' '.join(cmd[2:]) or 'EXIT'
                    self.auto_called_when.setdefault(
                        known[sc[0]], f'When the script gets the signal(s) {sig} (bash trap), e.g. on Ctrl+C or exit.')
            return
        if base0 in ('wsl', 'wsl.exe') and '--' in cmd:
            self.bash_command(rel, src, cmd[cmd.index('--') + 1:], known, depth + 1)
            return
        if base0 in ('bash', 'sh', 'zsh'):
            rest = cmd[1:]
            k = 0
            while k < len(rest) and rest[k].startswith('-'):
                if 'c' in rest[k][1:] and k + 1 < len(rest):
                    for _l, sub in bash_split_commands(rest[k + 1]):
                        self.bash_command(rel, src, sub, known, depth + 1)
                    return
                k += 1
            if k < len(rest):
                target = self.resolve_script(rest[k], rel)
                if target:
                    self.add_edge(src, f'mod:{target}', 'runs')
            return
        if w0 in ('source', '.') and len(cmd) >= 2:
            target = self.resolve_script(cmd[1], rel)
            if target:
                self.add_edge(src, f'mod:{target}', 'imports', 'source')
            return
        if PYTHON_EXE_RE.match(w0):
            rest = cmd[1:]
            k = 0
            while k < len(rest):
                if rest[k] == '-m' and k + 1 < len(rest):
                    mods = self.dotted_map.get(rest[k + 1]) or self.stem_map.get(rest[k + 1]) or []
                    if len(mods) == 1:
                        self.add_edge(src, mods[0].mod_id, 'runs', f'python3 -m {rest[k + 1]}')
                    return
                if rest[k] == '-c':
                    return
                if rest[k].startswith('-'):
                    k += 1
                    continue
                target = self.resolve_script(rest[k], rel, ('.py',))
                if target:
                    self.add_edge(src, f'mod:{target}', 'runs')
                return
            return
        if w0 == 'ros2' and len(cmd) >= 3:
            verb = cmd[1]
            if verb in ('run', 'launch') and len(cmd) >= 4:
                self.ros2_target(src, verb, cmd[2], cmd[3], 'runs')
            elif verb == 'topic' and cmd[2] in ('pub', 'echo', 'hz'):
                names = [w for w in cmd[3:] if w.startswith('/') and '$' not in w]
                if names:
                    topic = names[0]
                    self.ensure_topic(topic)
                    pos = cmd.index(topic)
                    mtype = cmd[pos + 1] if pos + 1 < len(cmd) and '/' in cmd[pos + 1] and '$' not in cmd[pos + 1] \
                        else ''
                    if mtype:
                        self.topic_types[topic].add(mtype)
                    if cmd[2] == 'pub':
                        self.add_edge(src, f'topic:{topic}', 'publishes', (mtype + ' (ros2 topic pub)').strip())
                    else:
                        self.add_edge(src, f'topic:{topic}', 'subscribes', f'ros2 topic {cmd[2]}')
            elif verb == 'service' and cmd[2] == 'call' and len(cmd) >= 4 and cmd[3].startswith('/') \
                    and '$' not in cmd[3]:
                topic = cmd[3]
                self.ensure_topic(topic)
                self.service_names.add(topic)
                mtype = cmd[4] if len(cmd) > 4 and '/' in cmd[4] and '$' not in cmd[4] else ''
                if mtype:
                    self.topic_types[topic].add(mtype)
                self.add_edge(src, f'topic:{topic}', 'calls', ('service client ' + mtype).strip())
            return
        if base0 in self.ext_programs:
            self.add_edge(src, self.ext_programs[base0], 'runs', base0)
            return
        if base0.endswith(('.sh', '.bash', '.py')):
            target = self.resolve_script(w0, rel)
            if target:
                self.add_edge(src, f'mod:{target}', 'runs')

    # ---------------------------------------------------------------- C#
    def analyze_csharp(self):
        for rel, lang in sorted(self.sources.items()):
            if lang == 'csharp':
                self.guard(rel, self.csharp_file, rel)

    def csharp_file(self, rel: str):
        """Classes, methods, properties and calls of one C# file (careful text rules, see cs_clean)."""
        text = read_text(self.repo / rel)
        lines = text.split('\n')
        clean, strings = cs_clean(text)
        is_test = self.is_test_path(rel)
        n = len(clean)
        # depth[i] = how many { } blocks are open just before character i (strings/comments are blank)
        depth = [0] * (n + 1)
        d = 0
        for i, ch in enumerate(clean):
            depth[i] = d
            if ch == '{':
                d += 1
            elif ch == '}':
                d -= 1
        depth[n] = d

        def block_end(open_at):
            """Index of the '}' that closes the '{' at open_at (-1 if the file ends first)."""
            want = depth[open_at]
            for j in range(open_at + 1, n):
                if clean[j] == '}' and depth[j] == want + 1:
                    return j
            return -1

        # classes, structs, interfaces and enums: [qualified name, name, body {, body }, start, kind]
        types = []
        for m in re.finditer(r'\b(class|struct|interface|enum)\s+([A-Za-z_]\w*)', clean):
            k = clean.find('{', m.end())
            if k < 0 or ';' in clean[m.end():k] or '=' in clean[m.end():k]:
                continue
            close = block_end(k)
            if close < 0:
                continue
            types.append([None, m.group(2), k, close, self.cs_stmt_start(clean, m.start()), m.group(1)])
        for t in types:
            outer = sorted((p for p in types if p is not t and p[2] < t[4] and p[3] > t[3]), key=lambda p: p[2])
            t[0] = '.'.join([p[1] for p in outer] + [t[1]])
        for qual, name, k, close, start, _tkind in types:
            outer = [p for p in types if p[2] < start and p[3] > close]
            parent = f'fn:{rel}::{max(outer, key=lambda p: p[2])[0]}' if outer else f'mod:{rel}'
            decl_line = clean.count('\n', 0, start) + 1
            node = {'id': f'fn:{rel}::{qual}', 'kind': 'class', 'label': name, 'parent': parent, 'path': rel,
                    'lang': 'csharp', 'line': decl_line, 'end_line': clean.count('\n', 0, close) + 1,
                    'signature': ' '.join(text[start:k].split())}
            doc = self.cs_doc(lines, decl_line)
            if doc:
                node['doc'] = doc
            if is_test:
                node['is_test'] = True
            self.add_node(node)

        # methods and constructors: "<modifiers> <type> Name(<arguments>) {" directly inside a type body
        methods = {}
        for mm in re.finditer(r'(?<![\w.])(~?[A-Za-z_]\w*)\s*(?:<[^<>(){};=]*>)?\s*\(', clean):
            pos = mm.start(1)
            mname = mm.group(1)
            if mname in CS_KEYWORDS:
                continue
            owners = [t for t in types if t[2] < pos < t[3] and depth[pos] == depth[t[2]] + 1
                      and t[5] != 'enum']
            if not owners:
                continue
            inner = max(owners, key=lambda t: t[2])      # (only the innermost type has this depth)
            qual, cname = inner[0], inner[1]
            stmt = self.cs_stmt_start(clean, pos)
            prefix = clean[stmt:pos]
            if '=' in prefix or re.search(r'\b(return|new|throw|await|using|typeof|delegate|event)\b', prefix):
                continue
            if not prefix.split() and mname.lstrip('~') != cname:
                continue                    # a declaration has a type or modifier before its name
            pclose = _match_close(clean, mm.end() - 1, '(', ')')
            if pclose < 0:
                continue
            mb = re.match(r'\s*(?::\s*(?:base|this)\s*\((?:[^()]|\([^()]*\))*\)\s*)?(?:where\s[^{;=]*)?(\{|=>|;)',
                          clean[pclose + 1:pclose + 600])
            if not mb or mb.group(1) == ';':
                continue
            if mb.group(1) == '{':
                bopen = pclose + 1 + mb.end() - 1
                bclose = block_end(bopen)
            else:
                bopen = pclose + 1 + mb.end() - 2
                bclose = clean.find(';', bopen)
            if bclose < 0:
                continue
            mid = f'fn:{rel}::{qual}.{mname}'
            first = clean.count('\n', 0, stmt) + 1
            last = clean.count('\n', 0, bclose) + 1
            if mid in methods:              # an overload (same name, other arguments): one block
                methods[mid]['extra'].append((first, last, (bopen, bclose)))
                continue
            methods[mid] = {'name': mname, 'cls': qual, 'first': first, 'last': last,
                            'sig': ' '.join(text[stmt:pclose + 1].split()), 'body': (bopen, bclose), 'extra': []}
        # properties with code: "<type> Name { get { ... } }" or "<type> Name => expression;" (their getter
        # runs every time the name is read, so they are blocks like methods)
        properties = set()
        for pm_ in re.finditer(r'(?<![\w.])([A-Za-z_]\w*)\s*(\{\s*(?:get|set|init)\b|=>)', clean):
            pos = pm_.start(1)
            pname = pm_.group(1)
            if pname in CS_KEYWORDS:
                continue
            owners = [t for t in types if t[2] < pos < t[3] and depth[pos] == depth[t[2]] + 1
                      and t[5] != 'enum']
            if not owners:
                continue
            inner = max(owners, key=lambda t: t[2])
            stmt = self.cs_stmt_start(clean, pos)
            prefix = clean[stmt:pos]
            if '=' in prefix or '(' in prefix or not prefix.split():
                continue
            if pm_.group(2) == '=>':
                bopen = pm_.start(2)
                bclose = clean.find(';', bopen)
            else:
                bopen = pm_.start(2)
                bclose = block_end(bopen)
            if bclose < 0:
                continue
            mid = f'fn:{rel}::{inner[0]}.{pname}'
            if mid in methods:
                continue
            first = clean.count('\n', 0, stmt) + 1
            last = clean.count('\n', 0, bclose) + 1
            methods[mid] = {'name': pname, 'cls': inner[0], 'first': first, 'last': last,
                            'sig': shorten(' '.join(text[stmt:bclose + 1].split()), 200),
                            'body': (bopen, bclose), 'extra': []}
            properties.add(mid)
        for mid, m in sorted(methods.items()):
            node = {'id': mid, 'kind': 'method', 'label': m['name'], 'parent': f'fn:{rel}::{m["cls"]}',
                    'path': rel, 'lang': 'csharp', 'line': m['first'], 'end_line': m['last'],
                    'signature': m['sig']}
            doc = self.cs_doc(lines, m['first'])
            if doc:
                node['doc'] = doc
            if is_test:
                node['is_test'] = True
            if self.include_source:
                parts = [node_text(lines, m['first'], m['last'])]
                for f, last, _body in m['extra']:
                    parts.append(f'// ---- also defined at line {f} (same name, other arguments) ----')
                    parts.append(node_text(lines, f, last))
                node['source'] = cap_text('\n'.join(parts), SOURCE_CAP)
            self.add_node(node)
        # calls between the file's own methods and classes, and scripts started through strings
        class_names = defaultdict(list)
        for t in types:
            class_names[t[1]].append(t[0])
        prop_names = defaultdict(list)          # class qual -> property names
        for pid in properties:
            prop_names[methods[pid]['cls']].append(methods[pid]['name'])
        for mid, m in methods.items():
            for bopen, bclose in [m['body']] + [e[2] for e in m['extra']]:
                body = clean[bopen:bclose]
                self.cs_body_edges(rel, mid, m['cls'], body, methods, class_names)
                # reading a property of the same class (or an enclosing one) runs its getter
                parts = m['cls'].split('.')
                for k in range(len(parts), 0, -1):
                    cq = '.'.join(parts[:k])
                    for pname in prop_names.get(cq, []):
                        pid = f'fn:{rel}::{cq}.{pname}'
                        if pid != mid and re.search(rf'(?<![\w.])(?:this\s*\.\s*)?{re.escape(pname)}\b(?!\s*\()',
                                                    body):
                            self.add_edge(mid, pid, 'calls', 'reads the property')
                for pos, value in strings:
                    if bopen <= pos <= bclose:
                        for sm in re.finditer(r'[\w./\\-]*/[\w.-]+\.sh\b', value):
                            target = self.resolve_script(sm.group(0), rel)
                            if target:
                                self.add_edge(mid, f'mod:{target}', 'runs')

    def cs_body_edges(self, rel, mid, cls, body, methods, class_names):
        """Calls inside one C# method body to methods/classes of the same file (only clear cases)."""
        for cm in re.finditer(r'\bnew\s+([A-Za-z_]\w*)\s*[({]', body):
            quals = class_names.get(cm.group(1), [])
            if len(quals) == 1:
                self.add_edge(mid, f'fn:{rel}::{quals[0]}', 'calls', 'creates')
        for cm in re.finditer(r'(?<![\w.])(?:this\s*\.\s*)?([A-Za-z_]\w*)\s*\(', body):
            name = cm.group(1)
            if name in CS_KEYWORDS or re.search(r'\bnew\s+$', body[:cm.start()]):
                continue
            target = self.cs_find_method(methods, cls, name, rel)
            if target:
                self.add_edge(mid, target, 'calls')
        for cm in re.finditer(r'(?<![\w.])([A-Z]\w*)\s*\.\s*([A-Za-z_]\w*)\s*\(', body):
            quals = class_names.get(cm.group(1), [])
            if len(quals) == 1:
                target = f'fn:{rel}::{quals[0]}.{cm.group(2)}'
                if target in methods:
                    self.add_edge(mid, target, 'calls')

    @staticmethod
    def cs_stmt_start(clean, pos) -> int:
        """Start of the declaration that contains position pos (after the previous ; { } or ])."""
        k = pos
        while k > 0 and clean[k - 1] not in ';{}]':
            k -= 1
        while k < pos and clean[k] in ' \t\n':
            k += 1
        return k

    @staticmethod
    def cs_doc(lines, decl_line) -> str | None:
        """The /// XML comment lines above a declaration, tags removed."""
        block = []
        k = decl_line - 2
        while k >= 0 and (lines[k].strip().startswith('///') or lines[k].strip().startswith('[')):
            if lines[k].strip().startswith('///'):
                block.insert(0, re.sub(r'^\s*///\s?', '', lines[k]))
            k -= 1
        text = re.sub(r'<[^>]+>', '', '\n'.join(block))
        text = clean_comment_lines(text.split('\n'))
        return text or None

    @staticmethod
    def cs_find_method(methods, cls, name, rel):
        """An unqualified call inside class cls: its own method, else one of an enclosing class."""
        parts = cls.split('.')
        while parts:
            mid = f'fn:{rel}::{".".join(parts)}.{name}'
            if mid in methods:
                return mid
            parts.pop()
        return None

    # ---------------------------------------------------------------- PowerShell and batch
    def analyze_powershell(self):
        for rel, lang in sorted(self.sources.items()):
            if lang == 'powershell':
                self.guard(rel, self.powershell_file, rel)

    def powershell_file(self, rel: str):
        """Functions of one PowerShell file, and the repo files it names (copied / run)."""
        text = read_text(self.repo / rel)
        lines = text.split('\n')
        clean, strings = ps_clean(text)
        for m in re.finditer(r'(?im)^[ \t]*function[ \t]+([\w-]+)[^{\n]*\{', clean):
            k = m.end() - 1
            close = _match_close(clean, k, '{', '}')
            if close < 0:
                continue
            first = clean.count('\n', 0, m.start()) + 1
            last = clean.count('\n', 0, close) + 1
            fid = f'fn:{rel}::{m.group(1)}'
            node = {'id': fid, 'kind': 'function', 'label': m.group(1), 'parent': f'mod:{rel}', 'path': rel,
                    'lang': 'powershell', 'line': first, 'end_line': last, 'signature': m.group(1)}
            if self.include_source:
                node['source'] = cap_text(node_text(lines, first, last), SOURCE_CAP)
            if fid not in self.nodes:
                self.add_node(node)
        for _pos, value in strings:
            v = value.replace('\\', '/')
            for sm in re.finditer(r'[\w./-]+\.(cs|sh|py|ps1|bat|yaml)\b', v):
                target = self.resolve_script(sm.group(0), rel, ('.cs', '.sh', '.py', '.ps1', '.bat', '.yaml'))
                if target and target != rel:
                    kind = 'runs' if target.endswith(('.sh', '.ps1', '.bat')) and 'bash' in v else 'data'
                    self.add_edge(f'mod:{rel}', f'mod:{target}', kind, None if kind == 'runs' else 'uses this file')

    def analyze_batch(self):
        for rel, lang in sorted(self.sources.items()):
            if lang == 'batch':
                self.guard(rel, self.batch_file, rel)

    def batch_file(self, rel: str):
        """The scripts one Windows batch file starts (through wsl.exe ... bash ...)."""
        text = read_text(self.repo / rel)
        for line in text.split('\n'):
            s = line.strip()
            if not s or re.match(r'^(@?rem\b|::|@?echo\b|set\b|title\b|goto\b|:|pause\b|setlocal|endlocal|if\b|for\b)',
                                 s, re.I):
                continue
            for _l, words in bash_split_commands(s.replace('%', '')):
                self.bash_command(rel, f'mod:{rel}', words, {}, 0)

    # ---------------------------------------------------------------- systems.yaml links and topics
    def ensure_topic(self, topic: str):
        tid = f'topic:{topic}'
        if tid not in self.nodes:
            self.nodes[tid] = {'id': tid, 'kind': 'topic', 'label': topic, 'parent': None}

    def add_links(self):
        for link in self.links:
            if yaml_false(link.get('map', 'true')):
                continue                    # a wire of the Wiring view only (build_wiring reads it)
            src, dst, kind = link.get('from'), link.get('to'), link.get('kind')
            if kind not in EDGE_KINDS:
                self.problems.append(f'systems.yaml link {src} -> {dst}: unknown kind "{kind}"')
                continue
            ok = True
            for end in (src, dst):
                if end and end.startswith('topic:'):
                    self.ensure_topic(end[len('topic:'):])
                elif end not in self.nodes:
                    self.warnings.append(f'systems.yaml link {src} -> {dst}: "{end}" is not in the map')
                    ok = False
            if ok:
                label = ' '.join(str(link.get('label') or '').split()) or None
                self.add_edge(src, dst, kind, label)

    def finish_topics(self):
        """Topic owners (MAVROS answers the /mavros/ names), topic parents and plain summaries.

        Test code (fake publishers, fake services) is left out when deciding who owns a topic and in the
        summary; its edges stay in the map (the explorer can hide tests)."""
        pubs, subs, calls = defaultdict(set), defaultdict(set), defaultdict(set)
        test_only = defaultdict(set)
        for src, dst, kind, _label in self.edges:
            if not dst.startswith('topic:'):
                continue
            if self.nodes.get(src, {}).get('is_test'):
                test_only[dst].add(src)
                continue
            {'publishes': pubs, 'subscribes': subs, 'calls': calls}.get(kind, defaultdict(set))[dst].add(src)
        owners = [(o.get('prefix', ''), o.get('owner'), o.get('label')) for o in self.topic_owners if o.get('owner')]
        pins = [(o.get('prefix', ''), o.get('parent')) for o in self.topic_owners if o.get('parent')]
        for tid, node in sorted(self.nodes.items()):
            if node['kind'] != 'topic':
                continue
            topic = node['label']
            is_service = topic in self.service_names
            hit = next(((o, lab) for p, o, lab in owners if p and topic.startswith(p) and o in self.nodes), None)
            owner = hit[0] if hit else None
            pin = next((par for p, par in pins if p and topic.startswith(p) and par in self.nodes), None)
            if owner:
                label = hit[1] or None
                if is_service:
                    if not subs[tid]:
                        self.add_edge(owner, tid, 'subscribes', label or 'serves this service')
                        subs[tid].add(owner)
                elif not pubs[tid] or pubs[tid] <= {owner}:
                    self.add_edge(owner, tid, 'publishes', label)
                    pubs[tid].add(owner)
                elif owner not in subs[tid]:
                    self.add_edge(owner, tid, 'subscribes', label or 'passes it on')
                    subs[tid].add(owner)
                if owner.startswith('mod:') or owner.startswith('fn:'):
                    owner = self.system_of(owner)
            if owner is None:
                ext_pubs = sorted(p for p in pubs[tid] if p.startswith('ext:'))
                owner = ext_pubs[0] if ext_pubs else None
            if owner is None:
                for group in (pubs[tid], subs[tid], calls[tid], test_only[tid]):
                    systems = Counter(s for s in (self.system_of(x) for x in group) if s)
                    if systems:
                        owner = sorted(systems.items(), key=lambda kv: (-kv[1], kv[0]))[0][0]
                        break
            node['parent'] = pin or owner
            types = sorted(self.topic_types.get(topic, ()))
            if is_service:
                parts = ['ROS 2 service (one program asks, another answers).']
            else:
                parts = ['ROS 2 topic (a named channel: programs send messages on it, others read them).']
            if types:
                parts.append(f'Type: {", ".join(types)}.')
            if is_service:
                if subs[tid]:
                    parts.append(f'Answered by: {self.names(subs[tid])}.')
                if calls[tid]:
                    parts.append(f'Called by: {self.names(calls[tid])}.')
            else:
                if pubs[tid]:
                    parts.append(f'Sent by: {self.names(pubs[tid])}.')
                if subs[tid]:
                    parts.append(f'Read by: {self.names(subs[tid])}.')
            if not (pubs[tid] or subs[tid] or calls[tid]) and test_only[tid]:
                parts.append(f'Only used in tests: {self.names(test_only[tid])}.')
            node['summary'] = ' '.join(parts)

    def system_of(self, nid: str) -> str | None:
        seen = 0
        while nid and seen < 20:
            node = self.nodes.get(nid)
            if node is None:
                return None
            if node['kind'] == 'system':
                return nid
            nid = node.get('parent')
            seen += 1
        return None

    def names(self, ids) -> str:
        """Readable names for a set of ids: the file name for code, the label for externals."""
        out = []
        for i in sorted(ids):
            node = self.nodes.get(i)
            if node is None:
                continue
            if node['kind'] in ('function', 'method', 'class'):
                out.append(f'{node["path"].rsplit("/", 1)[-1]} ({node["label"]})')
            else:
                out.append(node['label'])
        out = list(dict.fromkeys(out))
        return ', '.join(out[:8]) + (f' and {len(out) - 8} more' if len(out) > 8 else '')

    # ---------------------------------------------------------------- explanations
    def merge_explanations(self):
        files = sorted(Path(self.explanations_dir).glob('*.json')) if self.explanations_dir and \
            Path(self.explanations_dir).is_dir() else []
        claimed = {}
        for f in files:
            try:
                data = json.loads(read_text(f))
                entries = data.get('entries') or {}
                if not isinstance(entries, dict):
                    raise ValueError('"entries" is not an object')
            except (ValueError, AttributeError) as exc:
                self.warnings.append(f'explanations file {f.name} could not be read: {exc}')
                continue
            for nid in sorted(entries):
                entry = entries[nid]
                if not isinstance(entry, dict):
                    self.warnings.append(f'{f.name}: entry {nid} is not an object')
                    continue
                node = self.nodes.get(nid)
                if node is None:
                    self.unmatched_explanations.append(f'{f.name}: {nid}')
                    continue
                if nid in claimed:
                    self.warnings.append(f'{nid} is explained in {claimed[nid]} and again in {f.name} (the later '
                                         'file wins)')
                claimed[nid] = f.name
                for key in EXPLAIN_KEYS:
                    value = entry.get(key)
                    if isinstance(value, str) and value.strip():
                        node[key] = value.strip()
                if entry.get('explain'):
                    self.explain_source[nid] = 'explanations'
        # fall-backs for every block without an explanation
        for nid, node in self.nodes.items():
            if nid not in self.explain_source:
                text, source = self.fallback_explain(node)
                if text:
                    node['explain'] = text
                    self.explain_source[nid] = source
            if nid in self.auto_called_when and 'called_when' not in node:
                node['called_when'] = self.auto_called_when[nid]
        self.stats['explanation files'] = len(files)

    def fallback_explain(self, node: dict):
        kind = node['kind']
        if kind in ('system', 'external') and node.get('summary'):
            return node['summary'], 'systems.yaml'
        if kind == 'topic':
            return node['summary'] + GENERATED_MARK, 'generated'
        if kind == 'package':
            pkg = getattr(self, 'package_by_id', {}).get(node['id'])
            if node.get('summary'):
                if pkg is not None and pkg.summary_from_doc:
                    return node['summary'] + DOCSTRING_MARK, 'docstring'
                return node['summary'], 'systems.yaml'
            return None, None
        if kind in ('function', 'method') and node.get('is_test') and \
                re.match(r'^test', node['label']) and node.get('lang') == 'python':
            qual = node['id'].split('::', 1)[1]
            return test_sentence(qual), 'test name'
        if kind == 'module':
            summary = node.get('summary')
            if summary:
                if node['id'] in self.summary_generated:
                    return summary + GENERATED_MARK, 'generated'
                return summary + DOCSTRING_MARK, 'docstring'
            return None, None
        para = first_paragraph(node.get('doc'))
        if para:
            return para + DOCSTRING_MARK, 'docstring'
        return None, None

    # ---------------------------------------------------------------- final checks and output
    def check_edges(self):
        bad = [e for e in self.edges if e[0] not in self.nodes or e[1] not in self.nodes]
        for e in bad:
            self.warnings.append(f'edge {e[0]} -> {e[1]} ({e[2]}) dropped: an end is not in the map')
        self.edges -= set(bad)
        for nid, node in self.nodes.items():
            parent = node.get('parent')
            if parent is not None and parent not in self.nodes:
                self.warnings.append(f'{nid}: parent {parent} is not in the map')

    # ---------------------------------------------------------------- the Wiring view
    def build_wiring(self) -> dict | None:
        """The Wiring view of codemap.json (see build_wiring_section); its mistakes become problems (--check fails)."""
        out, problems, notes = build_wiring_section(self.wiring_spec, self.links, set(self.nodes))
        self.problems.extend(problems)
        self.warnings.extend(notes)
        if out:
            self.stats['wiring view boxes'] = len(out['boxes'])
            self.stats['wiring view wires'] = len(out['wires'])
        return out

    def trace_index(self) -> dict:
        py_modules, py_files, functions = {}, {}, {}
        for rel, pm in sorted(self.py.items()):
            py_files[rel] = pm.mod_id
            lines = {}
            for d in pm.defs.values():
                for n in d.nodes:
                    lines[str(first_line_of(n))] = d.fid
            if lines:
                functions[pm.mod_id] = dict(sorted(lines.items(), key=lambda kv: int(kv[0])))
        # importable names; a name two files share is left out (the file path still finds them)
        names = defaultdict(set)
        for pm in self.py.values():
            if pm.dotted:
                names[pm.dotted].add(pm.mod_id)
        for name, ids in names.items():
            if len(ids) == 1:
                py_modules[name] = next(iter(ids))
            else:
                self.stats['import names shared by several files (left out of py_modules)'] += 1
        # where colcon puts our Python files (the tracer reports these paths at run time)
        for folder, rp in self.ros_packages.items():
            ws = rp.workspace
            if not ws:
                continue
            for rel, pm in self.py.items():
                if not rel.startswith(folder + '/'):
                    continue
                inner = rel[len(folder) + 1:]
                if pm.in_package:
                    if rp.build_type == 'ament_python' or not rp.cmake_python:
                        py_files[f'{ws}/build/{rp.name}/{inner}'] = pm.mod_id
                        py_files[f'{ws}/install/{rp.name}/lib/{INSTALL_PYTHON}/site-packages/{inner}'] = pm.mod_id
                    if inner.split('/', 1)[0] in rp.cmake_python:
                        py_files[f'{ws}/install/{rp.name}/local/lib/{INSTALL_PYTHON}/dist-packages/{inner}'] = \
                            pm.mod_id
        for alias, rel in self.py_aliases.items():
            if rel in self.py:
                py_files[alias] = self.py[rel].mod_id
        sh_files = {rel: f'mod:{rel}' for rel in sorted(self.sh)}
        programs = {p: f'mod:{rel}' for p, rel in sorted(self.programs.items())}
        return {'py_modules': dict(sorted(py_modules.items())), 'py_files': dict(sorted(py_files.items())),
                'functions': dict(sorted(functions.items())), 'sh_files': sh_files, 'programs': programs}

    def result(self) -> dict:
        nodes = [self.nodes[k] for k in sorted(self.nodes)]
        for node in nodes:
            for k in [k for k, v in node.items() if v is None and k != 'parent']:
                del node[k]
        edges = []
        for src, dst, kind, label in sorted(self.edges):
            e = {'from': src, 'to': dst, 'kind': kind}
            if label:
                e['label'] = label
            edges.append(e)
        out = {'schema': SCHEMA, 'nodes': nodes, 'edges': edges, 'trace_index': self.trace_index()}
        if getattr(self, 'wiring', None):
            out['wiring'] = self.wiring
        return out

    def coverage(self, cm: dict) -> dict:
        """Explained / total per kind and per system, split by where the explanation came from."""
        by_kind = defaultdict(Counter)
        by_system = defaultdict(Counter)
        for node in cm['nodes']:
            src = self.explain_source.get(node['id'], 'none')
            by_kind[node['kind']]['total'] += 1
            by_kind[node['kind']][src] += 1
            system = self.system_of(node['id']) or '(top level)'
            by_system[system]['total'] += 1
            by_system[system][src] += 1
        return {'by_kind': {k: dict(v) for k, v in by_kind.items()},
                'by_system': {k: dict(v) for k, v in by_system.items()}}


# ---------------------------------------------------------------------------------------------------------
# Public entry points
# ---------------------------------------------------------------------------------------------------------

def git_commit(repo: Path) -> str:
    """The short hash of the checked-out commit, read from .git directly (falls back to the git program)."""
    try:
        git = repo / '.git'
        if git.is_file():
            m = re.match(r'gitdir:\s*(.+)', read_text(git).strip())
            git = (repo / m.group(1)).resolve() if m else git
        head = read_text(git / 'HEAD').strip()
        if head.startswith('ref:'):
            ref = head.split(None, 1)[1]
            ref_file = git / ref
            if ref_file.is_file():
                return read_text(ref_file).strip()[:7]
            common = git
            if (git / 'commondir').is_file():
                common = (git / read_text(git / 'commondir').strip()).resolve()
            for cand in (common / ref, ):
                if cand.is_file():
                    return read_text(cand).strip()[:7]
            packed = common / 'packed-refs'
            if packed.is_file():
                for line in read_text(packed).split('\n'):
                    if line.endswith(' ' + ref):
                        return line[:7]
        elif re.match(r'^[0-9a-f]{40}$', head):
            return head[:7]
    except (OSError, AttributeError):
        pass
    try:
        out = subprocess.run(['git', '-C', str(repo), 'rev-parse', '--short', 'HEAD'], capture_output=True,
                             text=True, timeout=10)
        if out.returncode == 0 and out.stdout.strip():
            return out.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    return 'unknown'


def now_iso() -> str:
    """UTC time stamp, or SOURCE_DATE_EPOCH (seconds) when that environment variable is set."""
    epoch = os.environ.get('SOURCE_DATE_EPOCH')
    t = _dt.datetime.fromtimestamp(int(epoch), _dt.timezone.utc) if epoch and epoch.isdigit() \
        else _dt.datetime.now(_dt.timezone.utc)
    return t.strftime('%Y-%m-%dT%H:%M:%SZ')


# Files never put into the map: the machine-specific notes of the owner's laptop (git-ignored, never committed,
# and the map may be published as a web page) and the map itself (it would describe its own previous run).
DEFAULT_SKIP_FILES = ('LOCAL_NOTES.md', 'simulation/codemap/codemap.json')


def build_codemap(repo, systems_file=None, explanations_dir=None, include_source=True, generated=None,
                  commit=None, skip_files=DEFAULT_SKIP_FILES):
    """Build the code map of `repo`. Returns (codemap dict, report dict).

    report keys: problems, warnings, duplicates, unassigned, unmatched_explanations, skipped (file counts per
    kind), stats, coverage, systems (package folder -> system key)."""
    repo = Path(repo)
    here = repo / 'simulation' / 'codemap'
    systems_file = Path(systems_file) if systems_file else here / 'systems.yaml'
    explanations_dir = Path(explanations_dir) if explanations_dir else here / 'explanations'
    builder = CodeMapBuilder(repo, systems_file, explanations_dir, include_source, skip_files)
    body = builder.build()
    cm = {'schema': SCHEMA, 'generated': generated or now_iso(),
          'commit': commit if commit is not None else git_commit(repo),
          'nodes': body['nodes'], 'edges': body['edges'], 'trace_index': body['trace_index']}
    if body.get('wiring'):
        cm['wiring'] = body['wiring']
    report = {'problems': builder.problems, 'warnings': builder.warnings, 'duplicates': builder.duplicates,
              'unassigned': builder.unassigned, 'unmatched_explanations': builder.unmatched_explanations,
              'skipped': dict(builder.skipped), 'stats': dict(builder.stats), 'coverage': builder.coverage(cm),
              'systems': {k: pkg.system for k, pkg in builder.packages.items()},
              'dynamic_topics': builder.dynamic_topics}
    return cm, report


def dumps_codemap(cm: dict) -> str:
    """codemap.json text: keys sorted, one node / edge per line (small, readable diffs)."""
    def one(x):
        return json.dumps(x, sort_keys=True, ensure_ascii=False, separators=(',', ':'))
    out = ['{', f'"schema":{one(cm["schema"])},', f'"generated":{one(cm["generated"])},',
           f'"commit":{one(cm["commit"])},', '"nodes":[']
    out.append(',\n'.join(one(n) for n in cm['nodes']))
    out.append('],')
    out.append('"edges":[')
    out.append(',\n'.join(one(e) for e in cm['edges']))
    out.append('],')
    out.append('"trace_index":{')
    ti = cm['trace_index']
    sections = []
    for key in sorted(ti):
        value = ti[key]
        inner = ',\n'.join(f'{one(k)}:{one(v)}' for k, v in value.items())
        sections.append(f'{one(key)}:{{\n{inner}\n}}')
    out.append(',\n'.join(sections))
    out.append('}' + (',' if cm.get('wiring') else ''))
    if cm.get('wiring'):
        # the Wiring view: one zone / box / wire / tour step per line, like the nodes
        w = cm['wiring']
        parts = []
        for key in sorted(w):
            value = w[key]
            if isinstance(value, list) and value:
                parts.append(f'{one(key)}:[\n' + ',\n'.join(one(v) for v in value) + '\n]')
            else:
                parts.append(f'{one(key)}:{one(value)}')
        out.append('"wiring":{\n' + ',\n'.join(parts) + '\n}')
    out.append('}')
    return '\n'.join(out) + '\n'


def print_report(cm: dict, report: dict, out_path: Path, size: int, quiet: bool):
    """The summary printed after a run: counts, coverage, and every problem."""
    p = print
    if not quiet:
        kinds = Counter(n['kind'] for n in cm['nodes'])
        ekinds = Counter(e['kind'] for e in cm['edges'])
        p(f'codemap: {out_path} ({size / 1024:.0f} KiB, commit {cm["commit"]})')
        p('  nodes: ' + ', '.join(f'{k} {kinds[k]}' for k in NODE_KINDS if kinds[k])
          + f'  (total {len(cm["nodes"])})')
        p('  edges: ' + ', '.join(f'{k} {ekinds[k]}' for k in EDGE_KINDS if ekinds[k])
          + f'  (total {len(cm["edges"])})')
        p('  files by kind: ' + ', '.join(f'{k} {v}' for k, v in sorted(report['skipped'].items())))
        for k, v in sorted(report['stats'].items()):
            p(f'  {k}: {v}')
        p('  explained blocks per kind (explanations files / docstrings / test names / generated / systems.yaml '
          '/ none):')
        cols = ('explanations', 'docstring', 'test name', 'generated', 'systems.yaml', 'none')
        for kind in NODE_KINDS:
            c = report['coverage']['by_kind'].get(kind)
            if c:
                p(f'    {kind:9s} {c["total"]:5d}: ' + ' / '.join(str(c.get(x, 0)) for x in cols))
        p('  explained blocks per system (same columns):')
        for system, c in sorted(report['coverage']['by_system'].items()):
            p(f'    {system:24s} {c["total"]:5d}: ' + ' / '.join(str(c.get(x, 0)) for x in cols))
    if report['unmatched_explanations']:
        p(f'  explanation ids that match no block ({len(report["unmatched_explanations"])}):')
        for x in report['unmatched_explanations'][:60]:
            p(f'    {x}')
        if len(report['unmatched_explanations']) > 60:
            p(f'    ... and {len(report["unmatched_explanations"]) - 60} more')
    if report['warnings'] and not quiet:
        p(f'  notes ({len(report["warnings"])}):')
        for w in report['warnings'][:80]:
            p(f'    {w}')
        if len(report['warnings']) > 80:
            p(f'    ... and {len(report["warnings"]) - 80} more')
    for title, items in (('PROBLEMS', report['problems']), ('FILES WITHOUT A PACKAGE/SYSTEM', report['unassigned']),
                         ('DUPLICATE IDS', report['duplicates'])):
        if items:
            p(f'  {title} ({len(items)}):')
            for x in items[:80]:
                p(f'    {x}')


def main(argv=None) -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description='Write simulation/codemap/codemap.json: the map of every source file, '
                                             'function and link of this repository (see the header of this file).')
    ap.add_argument('--repo', default=str(here.parents[1]), help='repository root (default: two folders up)')
    ap.add_argument('--out', default=None, help='output file (default: simulation/codemap/codemap.json)')
    ap.add_argument('--systems', default=None, help='systems file (default: simulation/codemap/systems.yaml)')
    ap.add_argument('--explanations', default=None, help='explanations folder (default: .../explanations)')
    ap.add_argument('--no-source', action='store_true', help="leave out each function's source text")
    ap.add_argument('--check', action='store_true', help='exit 1 if a file has no package/system or an id repeats')
    ap.add_argument('--quiet', action='store_true', help='only print problems')
    args = ap.parse_args(argv)
    repo = Path(args.repo).resolve()
    out = Path(args.out).resolve() if args.out else repo / 'simulation' / 'codemap' / 'codemap.json'
    skip = list(DEFAULT_SKIP_FILES)
    try:
        skip.append(out.relative_to(repo).as_posix())       # never map the output file itself
    except ValueError:
        pass
    try:
        cm, report = build_codemap(repo, args.systems, args.explanations, include_source=not args.no_source,
                                   skip_files=skip)
    except YamlError as exc:
        print(f'extract.py: systems.yaml cannot be read: {exc}', file=sys.stderr)
        return 2
    text = dumps_codemap(cm)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print_report(cm, report, out, len(text.encode('utf-8')), args.quiet)
    if args.check and (report['unassigned'] or report['duplicates'] or report['problems']):
        print('extract.py --check: FAILED (see above)', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
