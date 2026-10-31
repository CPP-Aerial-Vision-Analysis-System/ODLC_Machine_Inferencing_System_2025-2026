"""
simulation/codemap/test/test_build_artifact.py -- PURPOSE: tests for build_artifact.py, the script that makes the
shareable one-file code map (dist/astra_code_map.html), using the sample files in simulation/codemap/web/. They
check that the result is a bare artifact page (no <!doctype>/<html>/<head>/<body>, title in the first 8 KB, the
side-padding meta and rules kept), that the code map and the recording are embedded before the page's own script
and come back unchanged, that no text from the code map can end the script block early, that the recording is
downsampled without losing a single call, that source code is dropped step by step only when the page would be too
big, that a page which cannot fit fails with a clear message, and that only allowed hosts are referenced. When node
is installed it also runs the page's own tests (web/run_selftest.js, web/dom_smoke.js) and parses every script of a
built artifact with `node --check`.

Run (Windows or Linux, Python 3.10+, pytest; no ROS needed):
    python -m pytest -q -p no:cacheprovider simulation/codemap/test/test_build_artifact.py
"""

import importlib.util
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

CODEMAP_DIR = Path(__file__).resolve().parents[1]
WEB = CODEMAP_DIR / 'web'
SAMPLE = WEB / 'sample_codemap.json'
RECORDING = WEB / 'sample_recording.jsonl'
INDEX = WEB / 'index.html'


def _load_builder():
    spec = importlib.util.spec_from_file_location('build_artifact', CODEMAP_DIR / 'build_artifact.py')
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ba = _load_builder()
EMBED_RE = re.compile(r'<script id="astra-embed">\nwindow\.ASTRA_CODEMAP = (.*?);\nwindow\.ASTRA_TRACE = (.*?);\n</script>', re.S)


def _embedded(page):
    m = EMBED_RE.search(page)
    assert m, 'the embed block is missing'
    return json.loads(m.group(1)), json.loads(m.group(2)), m


def _markup(page):
    """The page without its comments, scripts and style blocks (where the outer tags would be)."""
    return ba._PROTECTED.sub('', page)


def _build_sample(tmp_path, *extra):
    out = tmp_path / 'astra_code_map.html'
    code = ba.main(['--codemap', str(SAMPLE), '--recording', str(RECORDING), '--out', str(out), *extra])
    assert code == 0
    return out.read_text(encoding='utf-8')


def _sample():
    return json.loads(SAMPLE.read_text(encoding='utf-8'))


def _totals(events):
    tot = {}
    for e in events:
        if e.get('event') == 'calls':
            for k, v in e['calls'].items():
                tot[k] = tot.get(k, 0) + v
    return tot


def test_sample_artifact_is_a_bare_page(tmp_path):
    page = _build_sample(tmp_path)
    markup = _markup(page)
    assert not re.search(r'<!doctype|<html\b|</html>|<head\b|</head>|<body\b|</body>', markup, re.I)
    assert not re.search(r'<meta\s+charset|<meta\s+name="viewport"', markup, re.I)
    head = page[:8192]
    assert '<title>ASTRA Code Map</title>' in head
    assert '<meta name="claude:viewport-fit-sides" content="cover">' in head
    assert 'padding-left: max(16px, env(safe-area-inset-left, 0px))' in page
    assert 'padding-right: max(16px, env(safe-area-inset-right, 0px))' in page
    assert page.count('id="astra-core"') == 1 and page.count('id="astra-app"') == 1
    assert ba.EMBED_MARKER not in page


def test_embedded_data_round_trips(tmp_path):
    page = _build_sample(tmp_path)
    cm, trace, _ = _embedded(page)
    sample = _sample()
    assert len(cm['nodes']) == len(sample['nodes']) and len(cm['edges']) == len(sample['edges'])
    assert 'trace_index' not in cm, 'only the live server needs trace_index'
    assert cm['artifact']['stage'] == 'full' and cm['artifact']['source'] == 'full'
    assert cm['artifact']['recording'] == RECORDING.name
    assert trace[0]['event'] == 'meta' and 'SYNTHETIC' in trace[0]['note']
    assert {e['event'] for e in trace[1:]} == {'calls', 'procs'}
    assert _totals(trace) == _totals(ba.read_recording(RECORDING)), 'downsampling must never lose a call'
    by_id = {n['id']: n for n in cm['nodes']}
    aim = by_id['fn:ros2_ws/src/main/src/target_geolocation.py::aim_at_box']
    assert aim['source'].startswith('def aim_at_box(') and aim['explain']


def test_embed_block_sits_between_the_core_and_the_app_script(tmp_path):
    page = _build_sample(tmp_path)
    assert page.index('id="astra-core"') < page.index('id="astra-embed"') < page.index('id="astra-app"')


def test_code_map_text_cannot_end_the_script_block(tmp_path):
    cm = _sample()
    nasty = '</script><script>alert(1)</script> <!-- x -->     </SCRIPT >'
    cm['nodes'][0] = dict(cm['nodes'][0], source=nasty, explain=nasty)
    page, _ = ba.build(INDEX.read_text(encoding='utf-8'), cm, [{'event': 'calls', 't': 1.0, 'calls': {nasty: 2}}])
    m = EMBED_RE.search(page)
    block = m.group(1) + m.group(2)
    assert '</script' not in block.lower() and '<!--' not in block
    assert ' ' not in block and ' ' not in block
    back, trace, _ = _embedded(page)
    assert back['nodes'][0]['source'] == nasty and trace[0]['calls'] == {nasty: 2}


def test_downsample_keeps_totals_and_caps_events():
    events = [{'event': 'meta', 'note': 'n'}]
    for i in range(600):   # 120 s of calls every 0.2 s
        events.append({'event': 'calls', 't': 1000 + i * 0.2, 'calls': {'fn:a.py::f': 1, 'fn:a.py::g': i % 3 + 1}})
    for k in range(60):    # procs every 2 s, the same programs the whole time
        events.append({'event': 'procs', 't': 1000 + k * 2.0, 'running': ['ext:gazebo', 'mod:a.py']})
    out, window = ba.downsample(events, max_events=50, max_bytes=10_000_000)
    calls = [e for e in out if e['event'] == 'calls']
    procs = [e for e in out if e['event'] == 'procs']
    assert window > 0 and len(calls) + len(procs) <= 50
    assert _totals(out) == _totals(events)
    assert len(procs) == 4, 'procs kept only when they change, and once every 30 s'
    assert out[0]['event'] == 'meta'
    assert [e['t'] for e in out[1:]] == sorted(e['t'] for e in out[1:])
    same, w0 = ba.downsample(events, max_events=10_000, max_bytes=10_000_000)
    assert w0 == 0 and len([e for e in same if e['event'] == 'calls']) == 600


def test_source_is_dropped_step_by_step_when_too_big():
    index = INDEX.read_text(encoding='utf-8')
    cm = _sample()
    events = ba.read_recording(RECORDING)
    _, full = ba.build(index, cm, events)
    assert full['stage'] == 'full'
    _, smaller = ba.build(index, cm, events, max_bytes=full['bytes'] - 1)
    assert smaller['stage'] != 'full' and smaller['source'] in ('partial', 'dropped')
    page_ns, no_src = ba.build(index, cm, events, no_source=True)
    assert no_src['source'] == 'dropped'
    page, tight = ba.build(index, cm, events, max_bytes=no_src['bytes'] + 50)
    assert tight['stage'] in ('no-source', 'no-source-no-doc')
    back, _, _ = _embedded(page)
    assert not any('source' in n for n in back['nodes'])
    assert back['artifact']['source'] == 'dropped'
    assert cm['nodes'][-1].get('source') is not None or any('source' in n for n in cm['nodes']), 'the input map is not changed'


def test_too_big_fails_with_a_clear_message(tmp_path, capsys):
    out = tmp_path / 'x.html'
    code = ba.main(['--codemap', str(SAMPLE), '--recording', str(RECORDING), '--out', str(out), '--max-bytes', '5000'])
    assert code == 2 and not out.exists()
    assert 'cannot be made smaller' in capsys.readouterr().err


def test_missing_codemap_says_what_to_do(tmp_path, capsys):
    code = ba.main(['--codemap', str(tmp_path / 'nope.json'), '--recording', 'none', '--out', str(tmp_path / 'x.html')])
    assert code == 2
    err = capsys.readouterr().err
    assert 'extract.py' in err and 'sample_codemap.json' in err


def test_no_recording_still_builds(tmp_path):
    out = tmp_path / 'map.html'
    assert ba.main(['--codemap', str(SAMPLE), '--recording', 'none', '--out', str(out)]) == 0
    cm, trace, _ = _embedded(out.read_text(encoding='utf-8'))
    assert trace == [] and cm['artifact']['events'] == 0


def test_newest_recording_looks_in_every_folder_given(tmp_path):
    import os
    a, b = tmp_path / 'a', tmp_path / 'b'
    a.mkdir()
    b.mkdir()
    old, new = a / '20260101-000000.jsonl', b / '20260102-000000.jsonl'
    old.write_text('{}\n', encoding='utf-8')
    new.write_text('{}\n', encoding='utf-8')
    os.utime(old, (1_000_000_000, 1_000_000_000))
    os.utime(new, (1_000_000_100, 1_000_000_100))
    (a / 'notes.txt').write_text('not a recording', encoding='utf-8')
    assert ba.newest_recording(a, b, tmp_path / 'missing') == new
    assert ba.newest_recording(a) == old
    assert ba.newest_recording(tmp_path / 'missing') is None


def test_default_recordings_include_the_linux_copy_folder(tmp_path, monkeypatch):
    # demo_traced.sh started from the Linux-side copy records into $ASTRA_SIM_HOME/codemap/recordings (default
    # ~/.astra_sim/codemap/recordings): with no --recording the newest run there must be found too.
    linux_dir = tmp_path / 'sim_home' / 'codemap' / 'recordings'
    linux_dir.mkdir(parents=True)
    monkeypatch.setenv('ASTRA_SIM_HOME', str(tmp_path / 'sim_home'))
    assert ba.default_recording_dirs() == [CODEMAP_DIR / 'recordings', linux_dir.resolve()]
    run = linux_dir / '20991231-235959.jsonl'
    run.write_text(RECORDING.read_text(encoding='utf-8'), encoding='utf-8')     # changed now: the newest of all
    out = tmp_path / 'map.html'
    assert ba.main(['--codemap', str(SAMPLE), '--out', str(out)]) == 0
    cm, _trace, _ = _embedded(out.read_text(encoding='utf-8'))
    assert cm['artifact']['recording'] == '20991231-235959.jsonl'


def test_strip_outer_leaves_scripts_comments_and_styles_alone():
    html = ('<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n<meta name="viewport" content="x">\n'
            '<title>X</title>\n<style>body { color: red } /* <body> */</style>\n</head>\n<body>\n'
            '<!-- a <body> tag in a comment -->\n<script>var s = "<body></html>";</script>\n<p>hi</p>\n</body>\n</html>\n')
    out = ba.strip_outer(html)
    assert out.startswith('<title>X</title>')
    assert '<script>var s = "<body></html>";</script>' in out
    assert '<!-- a <body> tag in a comment -->' in out
    assert '/* <body> */' in out
    assert not re.search(r'<html|<head|<body|<meta', ba._PROTECTED.sub('', out))


def test_only_allowed_hosts_are_referenced(tmp_path):
    page = _build_sample(tmp_path)
    urls = re.findall(r'\b(?:src|href)\s*=\s*"(https?:[^"]+)"', page)
    assert urls, 'expected the dagre script and the font stylesheet'
    for u in urls:
        assert re.match(r'https://(cdnjs\.cloudflare\.com|cdn\.jsdelivr\.net|unpkg\.com|fonts\.googleapis\.com|fonts\.gstatic\.com)(/|$)', u), u
    assert 'dagre/0.8.5/dagre.min.js' in page, 'libraries are pinned to an exact version'
    with pytest.raises(ba.BuildError):
        ba.check_external_urls('<script src="https://example.com/x.js"></script>')


def test_recording_lines_are_read_like_the_page_reads_them(tmp_path):
    rec = tmp_path / 'r.jsonl'
    rec.write_text('\n'.join([
        '{"event":"meta","note":"hello"}',
        '{"event":"calls","t":10,"calls":{"a":2,"b":0}}',
        'not json',
        '{"event":"calls","data":{"t":10.2,"calls":[["a",1],["a",2]]}}',
        '{"t":11,"running":["ext:gazebo"]}',
        '{"event":"calls","calls":{"a":1}}',
        '{"event":"calls","t":"nan","calls":{"a":1}}',
        '',
    ]), encoding='utf-8')
    evs = ba.read_recording(rec)
    assert evs[0] == {'event': 'meta', 'note': 'hello'}
    assert evs[1] == {'event': 'calls', 't': 10.0, 'calls': {'a': 2}}
    assert evs[2] == {'event': 'calls', 't': 10.2, 'calls': {'a': 3}}
    assert evs[3] == {'event': 'procs', 't': 11.0, 'running': ['ext:gazebo']}
    assert len(evs) == 4
    assert ba.read_recording(RECORDING)[0]['event'] == 'meta'


NODE = shutil.which('node')


@pytest.mark.skipif(NODE is None, reason='node is not installed')
def test_page_logic_self_test_passes_in_node():
    r = subprocess.run([NODE, str(WEB / 'run_selftest.js')], capture_output=True, text=True, timeout=180)
    assert r.returncode == 0, r.stdout + r.stderr
    assert 'checks passed' in r.stdout


@pytest.mark.skipif(NODE is None, reason='node is not installed')
def test_page_screen_code_smoke_test_passes_in_node():
    r = subprocess.run([NODE, str(WEB / 'dom_smoke.js')], capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.skipif(NODE is None, reason='node is not installed')
def test_every_script_of_a_built_artifact_parses(tmp_path):
    page = _build_sample(tmp_path)
    scripts = re.findall(r'<script id="([\w-]+)">(.*?)</script>', page, re.S)
    assert [s[0] for s in scripts] == ['astra-core', 'astra-embed', 'astra-app']
    for name, code in scripts:
        f = tmp_path / (name + '.js')
        f.write_text(code, encoding='utf-8')
        r = subprocess.run([NODE, '--check', str(f)], capture_output=True, text=True, timeout=120)
        assert r.returncode == 0, name + ': ' + r.stderr
