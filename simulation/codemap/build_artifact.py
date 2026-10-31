#!/usr/bin/env python3
"""
simulation/codemap/build_artifact.py -- PURPOSE: make ONE shareable file, dist/astra_code_map.html, that shows the
code map together with one recorded simulator run and needs no server and no network. It takes the explorer page
(web/index.html), puts codemap.json and a recording inside it (as window.ASTRA_CODEMAP and window.ASTRA_TRACE,
right before the page's own script), and removes the outer <!doctype>, <html>, <head> and <body> tags, because a
Claude artifact adds its own page frame. When the page sees window.ASTRA_CODEMAP it makes no network requests at
all and replays the embedded recording (play / pause / speed, and a note on how to watch live).

Keeping it small (an artifact page must stay under 16 MB; this script aims for 15 MB):
  * the recording is DOWNSAMPLED: "procs" events are kept only when the list of running programs changes, and
    "calls" events are merged into time windows (0.4 s, 0.8 s, 1.6 s ...) until at most --max-events events and
    --max-trace-bytes bytes are left; call counts are added up, never dropped;
  * the code map loses its trace_index (only the live server needs it); if the page is still too big, source code
    is reduced step by step: none from files (modules) -> at most 60 lines per function -> none at all; the panel
    then says where the code is in the repo; last of all, docstrings of functions that already have a plain
    explanation are left out.
Every text is put in as JSON with "<" written as \\u003c, so no code-map text can ever end the script block early.

Run (Windows or Linux, Python 3.10+, standard library only), from anywhere:
    python3 simulation/codemap/build_artifact.py                       # newest recording, codemap.json
  "newest recording" = the newest *.jsonl in recordings/ next to this file (runs recorded from a git checkout) or in
  ~/.astra_sim/codemap/recordings (runs that demo_traced.sh recorded from the Linux-side copy; $ASTRA_SIM_HOME/...
  when ASTRA_SIM_HOME is set). The last lines printed name the recording that was used.
    python3 simulation/codemap/build_artifact.py --recording simulation/codemap/recordings/20261031-150000.jsonl
    python3 simulation/codemap/build_artifact.py --codemap simulation/codemap/web/sample_codemap.json \\
            --recording simulation/codemap/web/sample_recording.jsonl --out /tmp/try.html    # the sample
Options: --codemap PATH, --recording PATH|none, --out PATH, --index PATH, --max-bytes N (15000000),
         --max-events N (3000), --max-trace-bytes N (3000000), --no-source.
Exit code 0 = written (the last line says the size), 2 = something is missing or the page cannot be made small enough.
"""

from __future__ import annotations

import argparse
import copy
import datetime
import json
import os
import re
import sys
from pathlib import Path
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
EMBED_MARKER = '<!-- ASTRA_EMBED -->'
DEFAULT_MAX_BYTES = 15_000_000
DEFAULT_MAX_EVENTS = 3000
DEFAULT_MAX_TRACE_BYTES = 3_000_000
SHORT_SOURCE_LINES = 60
# The only places an artifact page may load anything from (scripts from the CDNs, fonts from Google Fonts).
ALLOWED_HOSTS = {'cdnjs.cloudflare.com', 'cdn.jsdelivr.net', 'unpkg.com', 'fonts.googleapis.com', 'fonts.gstatic.com'}
CODEMAP_STAGES = ('full', 'no-module-source', 'short-source', 'no-source', 'no-source-no-doc')


class BuildError(Exception):
    """Something the person has to fix (a missing file, a page that cannot be made small enough)."""


# ------------------------------------------------------------------------------------------------------------
# The page: strip the outer tags, embed the data
# ------------------------------------------------------------------------------------------------------------
_PROTECTED = re.compile(r'<!--.*?-->|<script\b[^>]*>.*?</script\s*>|<style\b[^>]*>.*?</style\s*>', re.S | re.I)
_OUTER_TAGS = re.compile(
    r'<!doctype[^>]*>[ \t]*\n?|</?html\b[^>]*>[ \t]*\n?|</?head\b[^>]*>[ \t]*\n?|</?body\b[^>]*>[ \t]*\n?'
    r'|<meta\s+charset[^>]*>[ \t]*\n?|<meta\s+name="viewport"[^>]*>[ \t]*\n?', re.I)


def strip_outer(html: str) -> str:
    """Remove <!doctype>, <html>, <head>, <body> (and the charset / viewport metas the artifact frame already has).

    Comments, scripts and style blocks are left exactly as they are, even if their text mentions those tags."""
    out, pos = [], 0
    for m in _PROTECTED.finditer(html):
        out.append(_OUTER_TAGS.sub('', html[pos:m.start()]))
        out.append(m.group(0))
        pos = m.end()
    out.append(_OUTER_TAGS.sub('', html[pos:]))
    return ''.join(out).lstrip()


def js_json(obj) -> str:
    """JSON that is safe inside a <script> block: "<" (and the two JavaScript line separators) are escaped."""
    text = json.dumps(obj, ensure_ascii=False, separators=(',', ':'))
    return text.replace('<', '\\u003c').replace('\u2028', '\\u2028').replace('\u2029', '\\u2029')


def embed(page: str, codemap: dict, trace: list) -> str:
    if page.count(EMBED_MARKER) != 1:
        raise BuildError('web/index.html must contain the marker ' + EMBED_MARKER + ' exactly once (before its app script)')
    block = ('<script id="astra-embed">\nwindow.ASTRA_CODEMAP = ' + js_json(codemap) + ';\nwindow.ASTRA_TRACE = '
             + js_json(trace) + ';\n</script>')
    return page.replace(EMBED_MARKER, block)


def check_external_urls(page: str) -> None:
    """Every src= / href= that leaves the page must point at an allowed host (the artifact blocks all others)."""
    bad = []
    for m in re.finditer(r'\b(?:src|href)\s*=\s*"(https?:[^"]+)"', page):
        host = urlparse(m.group(1)).hostname or ''
        if host not in ALLOWED_HOSTS:
            bad.append(m.group(1))
    if bad:
        raise BuildError('the page loads from hosts an artifact cannot reach: ' + ', '.join(bad))


# ------------------------------------------------------------------------------------------------------------
# Recordings: read (same rules as the page's normalizeEvent) and downsample
# ------------------------------------------------------------------------------------------------------------
def normalize_event(obj):
    """One recorded line -> {"event":"calls","t","calls"} / {"event":"procs","t","running"} / {"event":"meta","note"} / None."""
    if not isinstance(obj, dict):
        return None
    p = obj.get('data')
    if isinstance(p, str):
        try:
            p = json.loads(p)
        except ValueError:
            p = None
    if not isinstance(p, dict):
        p = obj
    kind = obj.get('event') or obj.get('type') or obj.get('kind') or p.get('event')
    if not kind:
        kind = 'calls' if p.get('calls') else ('procs' if p.get('running') else None)
    t = p.get('t', obj.get('t'))
    if kind in ('meta', 'note'):
        return {'event': 'meta', 'note': str(p.get('note') or p.get('text') or obj.get('note') or '')}
    try:
        t = float(t)
    except (TypeError, ValueError):
        return None
    if t != t or t in (float('inf'), float('-inf')):
        return None
    if kind == 'calls':
        calls = p.get('calls')
        if isinstance(calls, list):
            merged = {}
            for item in calls:
                if isinstance(item, (list, tuple)) and item and isinstance(item[0], str):
                    try:
                        n = float(item[1]) if len(item) > 1 else 1.0
                    except (TypeError, ValueError):
                        n = 1.0
                    merged[item[0]] = merged.get(item[0], 0) + (n or 1)
            calls = merged
        if not isinstance(calls, dict):
            return None
        out = {}
        for k, v in calls.items():
            try:
                n = float(v)
            except (TypeError, ValueError):
                continue
            if n > 0 and n != float('inf'):
                out[str(k)] = int(n) if n == int(n) else n
        return {'event': 'calls', 't': t, 'calls': out} if out else None
    if kind == 'procs':
        running = p.get('running')
        return {'event': 'procs', 't': t, 'running': [x for x in running if isinstance(x, str)] if isinstance(running, list) else []}
    return None


def read_recording(path: Path) -> list:
    text = path.read_text(encoding='utf-8', errors='replace').strip()
    if not text:
        return []
    if text.startswith('['):
        try:
            return [e for e in (normalize_event(x) for x in json.loads(text)) if e]
        except ValueError:
            pass
    events = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            ev = normalize_event(json.loads(line))
        except ValueError:
            continue
        if ev:
            events.append(ev)
    return events


def _merge_calls(calls: list, window: float) -> list:
    """Add up the calls of every `window` seconds into one event (stamped with the time of its first event)."""
    if not calls:
        return []
    t0 = calls[0]['t']
    buckets = {}
    for ev in calls:
        key = int((ev['t'] - t0) // window)
        b = buckets.get(key)
        if b is None:
            b = buckets[key] = {'event': 'calls', 't': ev['t'], 'calls': {}}
        for fid, n in ev['calls'].items():
            b['calls'][fid] = b['calls'].get(fid, 0) + n
    return [buckets[k] for k in sorted(buckets)]


def downsample(events: list, max_events: int = DEFAULT_MAX_EVENTS, max_bytes: int = DEFAULT_MAX_TRACE_BYTES):
    """Return (events, window_seconds). Totals per function are kept exactly; only the timing gets coarser."""
    metas = [e for e in events if e.get('event') == 'meta']
    calls = sorted((e for e in events if e.get('event') == 'calls'), key=lambda e: e['t'])
    procs = sorted((e for e in events if e.get('event') == 'procs'), key=lambda e: e['t'])
    kept_procs, last, last_t = [], None, None
    for ev in procs:   # only when the running list changes (and at least every 30 s, so a seek finds it)
        key = tuple(sorted(ev['running']))
        if key != last or ev['t'] - last_t >= 30:
            kept_procs.append(ev)
            last, last_t = key, ev['t']
    window = 0.0
    while True:
        merged = _merge_calls(calls, window) if window else calls
        out = metas + sorted(merged + kept_procs, key=lambda e: e['t'])
        if (len(merged) + len(kept_procs) <= max_events and len(js_json(out)) <= max_bytes) or window >= 120:
            return out, window
        window = 0.4 if not window else window * 2


# ------------------------------------------------------------------------------------------------------------
# The code map: make it smaller step by step
# ------------------------------------------------------------------------------------------------------------
def slim_codemap(cm: dict, stage: str) -> dict:
    """A copy of the code map for one size stage (see CODEMAP_STAGES); the original is not changed."""
    out = copy.copy(cm)
    out.pop('trace_index', None)   # only server.py needs it
    nodes = []
    had_source = False
    for n in cm.get('nodes', []):
        if not isinstance(n, dict):
            continue
        n = dict(n)
        src = n.get('source')
        if src is not None:
            had_source = True
        if stage != 'full' and src is not None:
            if n.get('kind') == 'module' or stage in ('no-source', 'no-source-no-doc'):
                n.pop('source', None)
            elif stage == 'short-source':
                lines = str(src).split('\n')
                if len(lines) > SHORT_SOURCE_LINES:
                    n['source'] = '\n'.join(lines[:SHORT_SOURCE_LINES]) + '\n... (' + str(len(lines) - SHORT_SOURCE_LINES) + ' more lines in the repo)'
        if stage == 'no-source-no-doc' and n.get('explain') and n.get('kind') in ('function', 'method', 'class'):
            n.pop('doc', None)
        nodes.append(n)
    out['nodes'] = nodes
    if stage == 'full' or not had_source:
        source_state = 'full'
    elif stage in ('no-module-source', 'short-source'):
        source_state = 'partial'
    else:
        source_state = 'dropped'
    out['artifact'] = dict(out.get('artifact') or {}, stage=stage, source=source_state)
    return out


# ------------------------------------------------------------------------------------------------------------
# Putting it together
# ------------------------------------------------------------------------------------------------------------
def build(index_html: str, codemap: dict, events: list, recording_name: str = '', max_bytes: int = DEFAULT_MAX_BYTES,
          max_events: int = DEFAULT_MAX_EVENTS, max_trace_bytes: int = DEFAULT_MAX_TRACE_BYTES, no_source: bool = False):
    """Return (page_text, report dict). Raises BuildError when the page cannot fit in max_bytes."""
    if not isinstance(codemap, dict) or not isinstance(codemap.get('nodes'), list):
        raise BuildError('the code map has no "nodes" list: is it really a codemap.json?')
    page = strip_outer(index_html)
    check_external_urls(page)
    if '<title>' not in page[:8192]:
        raise BuildError('the page title must sit in the first 8 KB of the file (the artifact gallery reads only that)')
    built = datetime.datetime.now(datetime.timezone.utc).replace(microsecond=0).isoformat()
    stages = [s for s in CODEMAP_STAGES if not no_source or s in ('no-source', 'no-source-no-doc')]
    trace_budget, events_budget = max_trace_bytes, max_events
    attempts = []
    page_bytes = len(page.encode('utf-8'))
    slimmed = {}   # stage -> (code map copy, its JSON size): a big map is turned into JSON once per stage

    def slim(stage):
        if stage not in slimmed:
            cm_stage = slim_codemap(codemap, stage)
            slimmed[stage] = (cm_stage, len(js_json(cm_stage).encode('utf-8')))
        return slimmed[stage]

    while True:
        trace, window = downsample(events, events_budget, trace_budget)
        trace_bytes = len(js_json(trace).encode('utf-8'))
        for stage in stages:
            cm_stage, cm_bytes = slim(stage)
            estimate = page_bytes + cm_bytes + trace_bytes + 400   # + the embed block's own words and the build info
            attempts.append((stage, events_budget, estimate))
            if estimate > max_bytes:
                continue
            cm = copy.copy(cm_stage)
            cm['artifact'] = dict(cm_stage['artifact'], built=built, recording=recording_name, window_s=window,
                                  events=len([e for e in trace if e['event'] != 'meta']), events_in_recording=len(events))
            text = embed(page, cm, trace)
            size = len(text.encode('utf-8'))
            if size <= max_bytes:
                report = {'bytes': size, 'stage': stage, 'source': cm['artifact']['source'], 'window_s': window,
                          'events': cm['artifact']['events'], 'events_in_recording': len(events), 'nodes': len(cm['nodes']),
                          'edges': len(cm.get('edges', [])), 'attempts': attempts}
                return text, report
        if events_budget <= 200 or not events:
            break
        events_budget = max(200, events_budget // 2)
        trace_budget = max(200_000, trace_budget // 2)
    smallest = min(a[2] for a in attempts)
    raise BuildError('the artifact cannot be made smaller than %d bytes (limit %d). Even with no source code and a short '
                     'recording it is too big: build a smaller codemap.json or raise --max-bytes.' % (smallest, max_bytes))


def default_recording_dirs() -> list:
    """Where recordings are looked for when no --recording is given: recordings/ next to this file, then the folder
    demo_traced.sh records into when it runs from the Linux-side copy ($ASTRA_SIM_HOME/codemap/recordings, by default
    ~/.astra_sim/codemap/recordings; the same rule as server.py's sim_home_recordings)."""
    dirs = [HERE / 'recordings']
    try:
        home = os.environ.get('ASTRA_SIM_HOME') or str(Path.home() / '.astra_sim')
        dirs.append((Path(home).expanduser() / 'codemap' / 'recordings').resolve())
    except (OSError, RuntimeError, ValueError):
        pass
    return dirs


def newest_recording(*folders: Path):
    """The most recently changed *.jsonl file of the given folders (missing folders are skipped), or None."""
    files = []
    for folder in folders:
        try:
            if folder.is_dir():
                files.extend((p.stat().st_mtime, str(p), p) for p in folder.glob('*.jsonl'))
        except OSError:
            continue
    return max(files)[2] if files else None


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description='Make dist/astra_code_map.html: the code map explorer with codemap.json and one '
                                             'recorded run inside it (no server, no network).')
    ap.add_argument('--codemap', default=str(HERE / 'codemap.json'), help='codemap.json to embed (default: %(default)s)')
    ap.add_argument('--recording', default=None,
                    help='a recordings/*.jsonl file, or "none" (default: the newest in recordings/ or in '
                         '~/.astra_sim/codemap/recordings)')
    ap.add_argument('--out', default=str(HERE / 'dist' / 'astra_code_map.html'), help='output file (default: %(default)s)')
    ap.add_argument('--index', default=str(HERE / 'web' / 'index.html'), help='the explorer page (default: %(default)s)')
    ap.add_argument('--max-bytes', type=int, default=DEFAULT_MAX_BYTES, help='size limit of the result (default: %(default)s)')
    ap.add_argument('--max-events', type=int, default=DEFAULT_MAX_EVENTS, help='events kept from the recording (default: %(default)s)')
    ap.add_argument('--max-trace-bytes', type=int, default=DEFAULT_MAX_TRACE_BYTES, help='bytes kept from the recording (default: %(default)s)')
    ap.add_argument('--no-source', action='store_true', help='always leave the source code out')
    args = ap.parse_args(argv)
    try:
        cm_path, index_path = Path(args.codemap), Path(args.index)
        if not cm_path.is_file():
            raise BuildError('no code map at %s: run "python3 simulation/codemap/extract.py" first (or pass --codemap '
                             'simulation/codemap/web/sample_codemap.json to try the sample)' % cm_path)
        if not index_path.is_file():
            raise BuildError('the explorer page is missing: ' + str(index_path))
        try:
            codemap = json.loads(cm_path.read_text(encoding='utf-8'))
        except ValueError as err:
            raise BuildError('%s is not valid JSON: %s' % (cm_path, err)) from err
        if args.recording and args.recording.lower() == 'none':
            rec_path = None
        elif args.recording:
            rec_path = Path(args.recording)
            if not rec_path.is_file():
                raise BuildError('no recording at ' + str(rec_path))
        else:
            rec_path = newest_recording(*default_recording_dirs())
        events = read_recording(rec_path) if rec_path else []
        if rec_path is None:
            print('note: no recording embedded (record one with: python3 simulation/codemap/server.py --record auto); '
                  'the page will show the map without a replay')
        elif not events:
            print('note: ' + str(rec_path) + ' has no calls/procs events')
        page, report = build(index_path.read_text(encoding='utf-8'), codemap, events,
                             recording_name=rec_path.name if rec_path else '', max_bytes=args.max_bytes,
                             max_events=args.max_events, max_trace_bytes=args.max_trace_bytes, no_source=args.no_source)
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(page, encoding='utf-8', newline='\n')
    except BuildError as err:
        print('build_artifact: ' + str(err), file=sys.stderr)
        return 2
    print('code map: %d nodes, %d links; source code: %s (stage %s)' % (report['nodes'], report['edges'], report['source'], report['stage']))
    if rec_path:
        print('recording: %s, %d of %d events kept%s' % (rec_path, report['events'], report['events_in_recording'],
                                                         ', calls merged into %.1f s windows' % report['window_s'] if report['window_s'] else ''))
    print('wrote %s (%.2f MB)' % (out, report['bytes'] / 1e6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
