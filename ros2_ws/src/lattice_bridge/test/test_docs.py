"""
lattice_bridge/test/test_docs.py -- PURPOSE: keep the documentation honest. The Python examples in the README
are executed (so they cannot rot), the interface table must mention every public name of the package, every
relative link in the package docs must point at a file that exists, and the numbers quoted in the docs about
limits must match constants.py.
"""

import os
import pkgutil
import re
from pathlib import Path

import pytest

import lattice_bridge
from lattice_bridge import constants as C

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
README = PKG / 'README.md'
DOCS = PKG.parents[2] / 'docs' / 'lattice'


def python_blocks(path):
    text = path.read_text(encoding='utf-8')
    return re.findall(r'```python\n(.*?)```', text, flags=re.S)


def test_readme_has_python_examples():
    assert len(python_blocks(README)) >= 1


def test_readme_examples_run(tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)                          # the examples write lattice_requests.jsonl here
    for index, block in enumerate(python_blocks(README)):
        namespace = {'__name__': f'readme_block_{index}'}
        exec(compile(block, f'README.md block {index}', 'exec'), namespace)       # noqa: S102 - our own docs
    out = capsys.readouterr().out
    assert 'TGT p 36.214935 -96.006203 95 12' in out
    assert 'altitudeHaeMeters' in out and '231.0' in out
    assert 'create astra-runway20261024t153000z-tgt-person-1' in out
    assert (tmp_path / 'lattice_requests.jsonl').read_text(encoding='utf-8').count('\n') == 1


def test_the_interface_table_lists_every_public_name():
    text = README.read_text(encoding='utf-8')
    section = text[text.index('## 3. The public interface'):text.index('## 4. Ideas you need')]
    missing = [name for name in lattice_bridge.__all__ if f'`{name}' not in section]
    assert not missing, f'README section 3 does not mention: {missing}'


def test_every_name_in_all_exists_and_nothing_public_is_missing_from_all():
    for name in lattice_bridge.__all__:
        assert hasattr(lattice_bridge, name), name
    assert len(set(lattice_bridge.__all__)) == len(lattice_bridge.__all__)
    assert list(lattice_bridge.__all__) == sorted(lattice_bridge.__all__)
    # Sub-modules show up in dir() as soon as any test imports them, so they are found by looking at the
    # package folder instead of a hand-written list (a new module needs no edit here).
    submodules = {module.name for module in pkgutil.iter_modules(lattice_bridge.__path__)}
    for name in dir(lattice_bridge):
        if name.startswith('_') or name in submodules:
            continue
        assert name in lattice_bridge.__all__, f'{name} is importable but not in __all__'


def relative_links(path):
    return [target for target in re.findall(r'\]\(([^)#\s]+)(?:#[^)]*)?\)', path.read_text(encoding='utf-8'))
            if not re.match(r'[a-z]+:', target)]


@pytest.mark.parametrize('path', [README] + sorted((HERE / 'golden').glob('README.md'))
                         + (sorted(DOCS.glob('*.md')) if DOCS.is_dir() else []), ids=lambda p: p.name)
def test_relative_links_point_at_existing_files(path):
    """Links inside the package are always checked; links into the rest of the repo only when it is there."""
    for target in relative_links(path):
        resolved = (path.parent / target).resolve()
        if PKG.resolve() not in resolved.parents and not DOCS.is_dir():
            continue                                       # a clean overlay holds the package alone
        assert resolved.exists(), f'{path.name}: broken link {target}'


def test_documented_limits_match_the_constants():
    text = README.read_text(encoding='utf-8')
    assert f'{int(C.CLUSTER_MERGE_RADIUS_M)} m' in text
    assert f'{C.CLUSTER_MAX_CLUSTERS} clusters' in text
    assert f'now + {int(C.UAV_TTL_S)} s' in text
    assert f'at most {C.MAX_EXPIRY.days}' in text
    assert f'{C.STATUSTEXT_MAX_CHARS} characters' in text


@pytest.mark.skipif(not DOCS.is_dir(), reason='docs/lattice is not next to this package (clean overlay)')
def test_the_data_contract_names_every_enum_word_we_use():
    text = (DOCS / 'DATA_CONTRACT.md').read_text(encoding='utf-8')
    for word in (C.TEMPLATE_ASSET, C.TEMPLATE_TRACK, C.TEMPLATE_GEO, C.UAV_DISPOSITION, C.UAV_ENVIRONMENT,
                 C.TARGET_DISPOSITION, C.TARGET_ENVIRONMENT, C.GEO_TYPE_SEARCH_AREA, C.GEO_TYPE_FLIGHT_BOUNDARY,
                 C.INTEGRATION_NAME, C.PLATFORM_TYPE_UAV, C.PLATFORM_TYPE_PERSON, C.PLATFORM_TYPE_TENT):
        assert word in text, word


@pytest.mark.skipif(os.environ.get('LATTICE_BRIDGE_USE_INSTALLED') == '1', reason='source-tree only')
def test_golden_readme_lists_every_golden_file():
    text = (HERE / 'golden' / 'README.md').read_text(encoding='utf-8')
    for path in (HERE / 'golden').glob('*.json'):
        assert path.name in text, f'{path.name} is not described in golden/README.md'
