#!/usr/bin/env python3
"""
check_docs.py -- PURPOSE: keep the repo's documentation findable and its links working.

The documentation index is docs/README.md: one list of EVERY Markdown file in the repo, what it is for and
who it is for. This checker reads the repo and fails (exit code 1) when:

  1. a Markdown file in the repo is not linked from docs/README.md (a new or renamed doc nobody can find),
     unless it is on the NOT_INDEXED allow-list below (vendored third-party files, gitignored notes);
  2. a link in docs/README.md points to a file that does not exist (a moved or deleted doc);
  3. a relative link inside ANY doc points to a file that does not exist, or to a #section heading the target
     page does not have;
  4. a doc names a script under simulation/scripts/, simulation/jetson/, simulation/setup/,
     simulation/docker/ or simulation/windows/ that does not exist (a renamed script, a typo).

Docs that are planned but not written yet go in the "## Being written" section of docs/README.md: links
there may point to files that do not exist yet, and that is not an error (it prints a note when such a doc
has appeared, so it can move to its real group). Other docs may already link to such a planned doc while it
is being written (that prints a NOTE, not an error); once the index stops listing it under "Being written",
those links are errors again. A script named in that section may be planned too (for example the Jetson
kit's simulation/jetson/ scripts before they exist): check 4 skips that section only.

It uses only the Python standard library and runs on Windows and Linux (Python 3.8 or newer). It finds the
Markdown files with `git ls-files` (tracked files + new files that are not gitignored); where git is missing
(for example the Linux build copy, which has no .git folder) it walks the folder instead and skips what
.gitignore lists.

HOW TO RUN (from anywhere; it finds the repo from its own location):
    python3 simulation/scripts/check_docs.py              # Linux / WSL
    py -3 simulation\\scripts\\check_docs.py               # Windows
    python3 simulation/scripts/check_docs.py --list       # also print every doc and where it is indexed
    python3 simulation/scripts/check_docs.py --root DIR   # check another copy of the repo

HOW DO I KNOW IT WORKED: the last line says "[check_docs] OK: ..." and the exit code is 0.

EXIT CODES: 0 = everything is fine; 1 = problems found (each one is printed with what, why and how to fix it);
2 = the checker could not run (the repo root or docs/README.md was not found).

The test ros2_ws/src/simulation/sim_plugins/test/test_docs_index.py runs this checker on the real repo, so
`colcon test` and pytest fail too when the index is out of date.
"""

import argparse
import fnmatch
import os
import re
import subprocess
import sys
import unicodedata
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Set, Tuple
from urllib.parse import unquote

PROG = '[check_docs]'

# The index, relative to the repo root.
INDEX = 'docs/README.md'

# The heading of the index section whose links may point to docs that do not exist yet.
BEING_WRITTEN_HEADING = 'being written'

# Markdown files that do NOT have to be linked from the index, each with the reason. Patterns are
# repo-relative, '/'-separated, fnmatch style ('**' and '*' both match across folders). Links inside these
# files are not checked either (they belong to someone else's layout).
NOT_INDEXED: Dict[str, str] = {
    INDEX: 'the index itself',
    'ros2_ws/src/ultralytics_ros/*': 'vendored third-party package (Ultralytics ROS, has COLCON_IGNORE); '
                                     'its docs describe the upstream project, not ours',
    'ros2_ws/src/ultralytics/*': 'a local clone of the third-party Ultralytics library that some teammates '
                                 'keep in the workspace; not our documentation',
    'LOCAL_NOTES.md': 'gitignored machine notes for the owner laptop; never committed',
}

# Relative links that are known to be broken in files this project must not edit (the team's own notes),
# as (file, link target) -> reason. Keep this empty if you can: fix the link instead.
_OLD_DETECTION_DOCS = ('team doc, not edited by the simulator work: the old detection README links to docs '
                       'deleted long ago (QUICKSTART.md in 0111e4f, IMPLEMENTATION_SUMMARY.md in 450e222); a patch '
                       'that replaces the table was proposed to the team')
KNOWN_BROKEN_LINKS: Dict[Tuple[str, str], str] = {
    ('ros2_ws/src/detection/Documentation/README.md', 'QUICKSTART.md'): _OLD_DETECTION_DOCS,
    ('ros2_ws/src/detection/Documentation/README.md', 'README_SAHI.md'): _OLD_DETECTION_DOCS,
    ('ros2_ws/src/detection/Documentation/README.md', 'IMPLEMENTATION_SUMMARY.md'): _OLD_DETECTION_DOCS,
}

# Folders whose scripts the docs name; a named script that is missing is an error.
SCRIPT_DIRS = ('scripts', 'jetson', 'setup', 'docker', 'windows')
SCRIPT_RE = re.compile(
    r'simulation[/\\](' + '|'.join(SCRIPT_DIRS) + r')[/\\]([A-Za-z0-9_.\-]+\.(?:sh|py|ps1|bat))(?![A-Za-z0-9_])')

# Folders never searched for Markdown files (build outputs, caches, the git database).
SKIP_DIRS = {'.git', 'build', 'install', 'log', '__pycache__', '.pytest_cache', '.ruff_cache', 'node_modules',
             '.venv', 'venv', '.claude', '.vscode'}

# [text](target) and ![alt](target); target may be <wrapped in angle brackets> and may carry a "title".
INLINE_LINK_RE = re.compile(r'!?\[(?P<text>(?:[^\[\]]|\[[^\[\]]*\])*)\]\(\s*(?P<target><[^>]*>|[^)\s]+)'
                            r'(?:\s+(?:"[^"]*"|\'[^\']*\'))?\s*\)')
# [label]: target   (reference-style link definitions)
REF_DEF_RE = re.compile(r'^\s{0,3}\[[^\]]+\]:\s*(?P<target><[^>]*>|\S+)')
# <a id="x"> / <a name="x"> anchors written by hand
HTML_ANCHOR_RE = re.compile(r'<a\s+[^>]*(?:id|name)\s*=\s*["\']([^"\']+)["\']', re.IGNORECASE)
HEADING_RE = re.compile(r'^\s{0,3}(#{1,6})\s+(.*?)\s*#*\s*$')
FENCE_RE = re.compile(r'^\s{0,3}(```+|~~~+)')
SCHEME_RE = re.compile(r'^[A-Za-z][A-Za-z0-9+.\-]*:')


@dataclass
class Problem:
    """One thing wrong, printed in the "what / why / how to fix" shape."""
    where: str          # 'file:line' or 'file'
    what: str
    why: str
    fix: str

    def text(self) -> str:
        return f'{PROG} ERROR: {self.where}: {self.what}. Likely cause: {self.why}. Fix: {self.fix}'


@dataclass
class Link:
    """One relative link found in a Markdown file."""
    source: str         # repo-relative file the link is in
    line: int
    target: str         # the link exactly as written (after removing <>)
    section: str        # the ## heading it sits under ('' before the first one)


@dataclass
class Report:
    problems: List[Problem] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)
    docs: List[str] = field(default_factory=list)
    indexed: Set[str] = field(default_factory=set)

    @property
    def ok(self) -> bool:
        return not self.problems


# --------------------------------------------------------------------------------------------- finding docs

def _git_markdown_files(root: Path) -> Optional[List[str]]:
    """Tracked + untracked-but-not-ignored *.md files according to git, or None if git cannot answer."""
    try:
        top = subprocess.run(['git', '-C', str(root), 'rev-parse', '--show-toplevel'],
                             capture_output=True, timeout=60, check=False)
        # Only trust git when `root` IS the top of a checkout (not a folder inside some other repo).
        if top.returncode != 0 or Path(top.stdout.decode('utf-8', 'replace').strip()).resolve() != root.resolve():
            return None
        out = subprocess.run(['git', '-C', str(root), 'ls-files', '-co', '--exclude-standard', '-z', '--',
                              '*.md', '*.MD', '*.markdown'],
                             capture_output=True, timeout=60, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None          # not a git checkout, or git refuses (e.g. "dubious ownership"): walk instead
    names = [n for n in out.stdout.decode('utf-8', 'replace').split('\0') if n]
    # git also lists files deleted in the working tree but not yet committed: keep only files that exist.
    return sorted({n.replace('\\', '/') for n in names if (root / n).is_file()})


def _gitignore_rules(root: Path) -> List[str]:
    """The plain rules of the top-level .gitignore (comments, blanks and '!' re-includes dropped)."""
    try:
        lines = (root / '.gitignore').read_text(encoding='utf-8', errors='replace').splitlines()
    except OSError:
        return []
    return [ln.strip() for ln in lines if ln.strip() and not ln.strip().startswith(('#', '!'))]


def _ignored(rel: str, rules: Iterable[str]) -> bool:
    """A small subset of .gitignore matching, enough for the walk fallback (not a full implementation)."""
    parts = rel.split('/')
    for rule in rules:
        r = rule.rstrip('/')
        anchored = '/' in r
        r = r.lstrip('/')
        if anchored:
            # a path rule: the file itself or any parent folder matches it
            for i in range(1, len(parts) + 1):
                if fnmatch.fnmatchcase('/'.join(parts[:i]), r):
                    return True
        elif any(fnmatch.fnmatchcase(p, r) for p in parts):
            return True
    return False


def _walk_markdown_files(root: Path) -> List[str]:
    rules = _gitignore_rules(root)
    found = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d not in SKIP_DIRS)
        for name in filenames:
            if name.lower().endswith(('.md', '.markdown')):
                rel = Path(dirpath, name).relative_to(root).as_posix()
                if not _ignored(rel, rules):
                    found.append(rel)
    return sorted(found)


def markdown_files(root: Path) -> List[str]:
    """Every Markdown file of the repo, repo-relative with '/' separators."""
    files = _git_markdown_files(root)
    return files if files is not None else _walk_markdown_files(root)


def not_indexed_reason(rel: str) -> Optional[str]:
    for pattern, reason in NOT_INDEXED.items():
        if fnmatch.fnmatchcase(rel, pattern):
            return reason
    return None


# --------------------------------------------------------------------------------------------- parsing docs

def _strip_code_spans(line: str) -> str:
    """Blank out `inline code` so links shown as code examples are not checked."""
    return re.sub(r'(`+)(.+?)\1', lambda m: ' ' * len(m.group(0)), line)


def _prose_lines(text: str) -> Iterable[Tuple[int, str, bool]]:
    """(line number, line, inside a fenced code block?) for every line."""
    fence = None
    for number, line in enumerate(text.splitlines(), start=1):
        m = FENCE_RE.match(line)
        if m:
            marker = m.group(1)
            if fence is None:
                fence = marker[0] * 3
                yield number, line, True
                continue
            if marker.startswith(fence):
                fence = None
                yield number, line, True
                continue
        yield number, line, fence is not None


def _strip_html_comments(text: str) -> str:
    """Replace <!-- comments --> with blank lines of the same count (keeps line numbers right)."""
    return re.sub(r'<!--.*?-->', lambda m: '\n' * m.group(0).count('\n'), text, flags=re.DOTALL)


def find_links(rel: str, text: str) -> List[Link]:
    """Every relative link (not http:, mailto:, ...) in one Markdown text, outside code and HTML comments."""
    links: List[Link] = []
    section = ''
    for number, line, in_code in _prose_lines(_strip_html_comments(text)):
        if in_code:
            continue
        h = HEADING_RE.match(line)
        if h and len(h.group(1)) == 2:
            section = h.group(2).strip()
        prose = _strip_code_spans(line)
        targets = [m.group('target') for m in INLINE_LINK_RE.finditer(prose)]
        d = REF_DEF_RE.match(prose)
        if d:
            targets.append(d.group('target'))
        for target in targets:
            target = target.strip()
            if target.startswith('<') and target.endswith('>'):
                target = target[1:-1].strip()
            if not target or SCHEME_RE.match(target) or target.startswith('//'):
                continue
            links.append(Link(rel, number, target, section))
    return links


def slugify(heading: str) -> str:
    """The #anchor GitHub gives a heading (lower case, punctuation dropped, spaces -> '-')."""
    text = re.sub(r'!?\[([^\]]*)\]\([^)]*\)', r'\1', heading)      # [text](url) -> text
    text = re.sub(r'<[^>]+>', '', text)                              # inline HTML tags
    text = text.strip().lower()
    out = []
    for ch in text:
        cat = unicodedata.category(ch)
        if ch in ' -' or ch == '_' or cat[0] in ('L', 'N') or cat == 'Mn':
            out.append('-' if ch == ' ' else ch)
    return ''.join(out)


def anchors_of(text: str) -> Set[str]:
    """Every #anchor a Markdown page offers: heading slugs (with GitHub's -1, -2 for repeats) and <a id>."""
    found: Set[str] = set()
    seen: Dict[str, int] = {}
    for _number, line, in_code in _prose_lines(text):
        if in_code:
            continue
        for a in HTML_ANCHOR_RE.findall(line):
            found.add(a)
        h = HEADING_RE.match(line)
        if h:
            slug = slugify(h.group(2))
            if slug in seen:
                seen[slug] += 1
                found.add(f'{slug}-{seen[slug]}')
            else:
                seen[slug] = 0
                found.add(slug)
    return found


def section_of_each_line(text: str) -> Dict[int, str]:
    """{line number: the ## heading that line sits under ('' before the first one)}, like find_links tracks it."""
    sections: Dict[int, str] = {}
    section = ''
    for number, line, in_code in _prose_lines(_strip_html_comments(text)):
        if not in_code:
            h = HEADING_RE.match(line)
            if h and len(h.group(1)) == 2:
                section = h.group(2).strip()
        sections[number] = section
    return sections


def find_script_mentions(text: str) -> List[Tuple[int, str]]:
    """(line, 'simulation/<dir>/<name>') for every script path a doc names (also inside code blocks)."""
    mentions = []
    for number, line in enumerate(_strip_html_comments(text).splitlines(), start=1):
        for m in SCRIPT_RE.finditer(line):
            mentions.append((number, f'simulation/{m.group(1)}/{m.group(2)}'))
    return mentions


def resolve(root: Path, source: str, target: str) -> Tuple[Path, str]:
    """(absolute path, '#anchor' part or '') of a relative link written in `source`."""
    path_part, _, anchor = target.partition('#')
    path_part = unquote(path_part.split('?', 1)[0])
    if not path_part:
        return root / source, anchor            # '#section' = a heading of the same page
    if path_part.startswith('/'):
        base = root / path_part.lstrip('/')    # GitHub treats a leading '/' as the repo root
    else:
        base = (root / source).parent / path_part
    return Path(os.path.normpath(str(base))), anchor


def _rel(root: Path, path: Path) -> Optional[str]:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return None


# --------------------------------------------------------------------------------------------- the checks

def check(root: Path) -> Report:
    """Run every check on the repo at `root`; never raises for a doc problem (they go in the report)."""
    root = Path(root).resolve()
    report = Report()
    docs = markdown_files(root)
    report.docs = docs
    texts: Dict[str, str] = {}
    for rel in docs:
        try:
            texts[rel] = (root / rel).read_text(encoding='utf-8', errors='replace')
        except OSError as exc:
            report.problems.append(Problem(rel, f'cannot read the file ({exc})', 'file permissions or a broken '
                                           'file', 'check the file opens in an editor'))
    anchor_cache: Dict[Path, Set[str]] = {}

    def anchors(path: Path) -> Set[str]:
        if path not in anchor_cache:
            try:
                anchor_cache[path] = anchors_of(path.read_text(encoding='utf-8', errors='replace'))
            except OSError:
                anchor_cache[path] = set()
        return anchor_cache[path]

    index_text = texts.get(INDEX)
    if index_text is None:
        index_path = root / INDEX
        if not index_path.is_file():
            raise FileNotFoundError(str(index_path))
        index_text = index_path.read_text(encoding='utf-8', errors='replace')
        texts[INDEX] = index_text

    # ---- 1 + 2: the index
    planned_missing: Set[str] = set()        # repo-relative paths of planned docs that do not exist yet
    for link in find_links(INDEX, index_text):
        path, _anchor = resolve(root, INDEX, link.target)
        rel = _rel(root, path)
        planned = link.section.strip().lower().startswith(BEING_WRITTEN_HEADING)
        if path.exists():
            if rel:
                report.indexed.add(rel)
                if planned:
                    report.notes.append(f'{PROG} NOTE: {rel} now exists: move it from "Being written" in {INDEX} '
                                        f'to its group (until then it counts as indexed).')
        elif planned and rel:
            planned_missing.add(rel)
        # (broken non-planned links of the index are reported by check 3 below, like any other doc)

    for rel in docs:
        if rel in report.indexed or not_indexed_reason(rel):
            continue
        report.problems.append(Problem(
            rel, f'this Markdown file is not listed in the documentation index {INDEX}',
            'a new, moved or renamed doc was not added to the index',
            f'add a line for it to the right group of {INDEX} (link, what it is for, who it is for), or, if it '
            f'is third-party or private, add it to NOT_INDEXED in simulation/scripts/check_docs.py with the reason'))

    # ---- 3 + 4: links and script names inside every doc
    for rel in docs:
        if rel != INDEX and not_indexed_reason(rel):
            continue
        text = texts.get(rel)
        if text is None:
            continue
        for link in find_links(rel, text):
            if (rel, link.target) in KNOWN_BROKEN_LINKS:
                continue
            path, anchor = resolve(root, rel, link.target)
            where = f'{rel}:{link.line}'
            if not path.exists() and _rel(root, path) in planned_missing:
                # A doc listed under "Being written" may already be linked while it is written (parallel work).
                # When the index no longer lists it there, the same link becomes an error again.
                if rel != INDEX:
                    report.notes.append(f'{PROG} NOTE: {where}: links to {_rel(root, path)}, which is planned under '
                                        f'"Being written" in {INDEX} but does not exist yet.')
                continue
            if not path.exists():
                report.problems.append(Problem(
                    where, f"the link '{link.target}' points to a file that does not exist "
                           f"({_rel(root, path) or path})",
                    'the target was moved, renamed or deleted, or the link has a typo (links are relative to the '
                    'folder of the file they are in)',
                    'correct the link' + (f' (and the line in {INDEX})' if rel != INDEX else '') +
                    f', or, if it is a doc somebody is writing right now, list it under "## Being written" in '
                    f'{INDEX}'))
                continue
            if anchor and path.is_file() and path.suffix.lower() in ('.md', '.markdown'):
                if anchor not in anchors(path) and anchor.lower() not in anchors(path):
                    report.problems.append(Problem(
                        where, f"the link '{link.target}' points to a section '#{anchor}' that "
                               f"{_rel(root, path)} does not have",
                        'the heading was renamed or removed (GitHub makes the anchor from the heading text: lower '
                        'case, punctuation dropped, spaces become -)',
                        'change the #part of the link to the current heading, or link the page without #part'))
        # A script named under "## Being written" of the index may be planned as well: skip only those lines.
        planned_lines: Set[int] = set()
        if rel == INDEX:
            planned_lines = {n for n, s in section_of_each_line(text).items()
                             if s.lower().startswith(BEING_WRITTEN_HEADING)}
        for number, script in find_script_mentions(text):
            if number in planned_lines:
                continue
            if not (root / script).is_file():
                report.problems.append(Problem(
                    f'{rel}:{number}', f'the doc names the script {script}, which does not exist',
                    'the script was renamed or removed, or the name has a typo',
                    f'correct the name in {rel} (see the folder simulation/{script.split("/")[1]}/), or add the '
                    f'script'))
    return report


# --------------------------------------------------------------------------------------------- command line

def find_repo_root(start: Path) -> Optional[Path]:
    """The folder above `start` that holds docs/ and simulation/ (the repo root)."""
    for p in [start] + list(start.parents):
        if (p / 'simulation').is_dir() and (p / 'ros2_ws').is_dir():
            return p
    return None


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog='check_docs.py',
        description='Check that docs/README.md indexes every Markdown file and that doc links and script '
                    'names are not broken. Exit 0 = OK, 1 = problems found, 2 = could not run.')
    parser.add_argument('--root', help='the repo root to check (default: the repo this script is in)')
    parser.add_argument('--list', action='store_true', help='also print every Markdown file and its status')
    args = parser.parse_args(argv)

    root = Path(args.root).resolve() if args.root else find_repo_root(Path(__file__).resolve().parent)
    if root is None or not root.is_dir():
        print(f'{PROG} ERROR: could not find the repo root. Likely cause: the script was copied out of the repo. '
              f'Fix: run it as simulation/scripts/check_docs.py inside the repo, or pass --root <repo folder>.',
              file=sys.stderr)
        return 2
    try:
        report = check(root)
    except FileNotFoundError as exc:
        print(f'{PROG} ERROR: the documentation index is missing ({exc}). Likely cause: docs/README.md was '
              f'moved, renamed or not created yet. Fix: restore docs/README.md (git checkout -- docs/README.md) '
              f'or pass the right --root.', file=sys.stderr)
        return 2

    if args.list:
        for rel in report.docs:
            reason = not_indexed_reason(rel)
            state = 'indexed' if rel in report.indexed else (f'not indexed on purpose: {reason}' if reason
                                                             else 'NOT INDEXED')
            print(f'{rel}  [{state}]')
    for note in report.notes:
        print(note)
    for problem in report.problems:
        print(problem.text(), file=sys.stderr)
    if report.problems:
        print(f'{PROG} FAILED: {len(report.problems)} problem(s) in {len(report.docs)} Markdown files. Fix each '
              f'line above, then run this checker again: python3 simulation/scripts/check_docs.py',
              file=sys.stderr)
        return 1
    print(f'{PROG} OK: {len(report.docs)} Markdown files, all indexed in {INDEX} (or allow-listed), no broken '
          f'links, every named script exists.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
