"""
sim_worlds/test/test_assets_generators.py -- PURPOSE: tests of the Fuel asset catalogue (sane bounding boxes,
fetch_models.sh in sync), of the SDF helpers, and of the world generators (deterministic for a seed, the
committed worlds equal the generator output, the offline "primitive" variant needs no Fuel model).
"""

import math
import os
import re
import subprocess
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from sim_worlds.worldgen import assets, sdf


def test_catalogue_is_sane():
    assert assets._check_catalogue() == []
    for pose in assets.HIKER_POSES:
        a = assets.asset(f'hiker_{pose}')
        assert a.detector_class == 'person' and a.kind == 'mannequin'
    with pytest.raises(KeyError):
        assets.asset('nope')


def test_lying_hiker_is_centred_and_flat():
    a = assets.asset('hiker_lying_face_up')
    assert a.center()[:2] == pytest.approx([0.0, 0.0], abs=1e-3)
    assert a.size()[2] < 0.4 and max(a.size()[:2]) > 1.7


def test_fetch_models_list_matches_catalogue(pkg_dir):
    with open(os.path.join(pkg_dir, 'scripts', 'fetch_models.sh'), encoding='utf-8') as f:
        text = f.read()
    listed = set(re.findall(r'^\s+"([^"]+/\d+)"\s*$', text, re.M))
    expected = {f'{m.owner}/{m.name}/{m.version}' for m in assets.all_fuel_models()}
    assert listed == expected


def test_fetch_models_list_option(pkg_dir):
    out = subprocess.run(['bash', os.path.join(pkg_dir, 'scripts', 'fetch_models.sh'), '--list'], capture_output=True,
                         text=True, check=True).stdout.split('\n')
    assert 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue Randy/3' in out


def test_grass_quilt_is_seamless():
    """The ground picture repeats every 160 m: its left/right and top/bottom edges must continue each other."""
    from sim_worlds.worldgen import textures
    a = np.asarray(textures.grass_quilt(np.random.default_rng(1), 256), dtype=float)
    inner = np.abs(np.diff(a, axis=1)).mean()
    assert np.abs(a[:, -1] - a[:, 0]).mean() < 1.5 * inner
    assert np.abs(a[-1, :] - a[0, :]).mean() < 1.5 * inner


def test_rotation_helpers_round_trip():
    for rpy in ((0.1, -0.4, 2.0), (math.pi / 2, 0.0, 0.3), (-1.0, 0.2, -2.5)):
        assert sdf.matrix_to_rpy(sdf.rpy_to_matrix(*rpy)) == pytest.approx(rpy, abs=1e-9)
    r = sdf.rpy_to_matrix(0, 0, math.pi / 2)
    assert r @ np.array([1.0, 0.0, 0.0]) == pytest.approx([0.0, 1.0, 0.0], abs=1e-12)


def test_model_xml_is_valid_sdf_fragment():
    parts = assets.asset_parts('hiker_sitting') + [sdf.part('b', 'ellipsoid', (1, 1, 0.5), color=(0.1, 0.2, 0.1))]
    root = ET.fromstring(sdf.model_xml('m', parts, (1, 2, 0), 0.5))
    assert root.find('static').text == 'true'
    assert root.find('link/visual/geometry/mesh/uri').text.startswith('https://fuel.gazebosim.org/1.0/openrobotics/')
    assert root.find('link/collision/geometry/box/size') is not None


def _gen(module, seed, mode):
    """Run a generator script's build() in memory (nothing is written) and return (builder, world text)."""
    import importlib
    m = importlib.import_module(module)
    b = m.build(seed, mode, log=lambda *_: None)
    return b, b.world_sdf(m.PURPOSE)


def test_runway_generator_deterministic_and_committed(pkg_dir):
    b1, w1 = _gen('build_runway', 7, 'fuel')
    _, w2 = _gen('build_runway', 7, 'fuel')
    assert w1 == w2
    with open(os.path.join(pkg_dir, 'worlds', 'runway.sdf'), encoding='utf-8') as f:
        assert f.read() == w1, 'worlds/runway.sdf differs from the generator: re-run scripts/build_runway.py'
    _, w3 = _gen('build_runway', 8, 'fuel')
    assert w3 != w1
    ET.fromstring(w1)


def test_suas_field_generator_committed(pkg_dir):
    _, w = _gen('build_suas_field', 2026, 'fuel')
    with open(os.path.join(pkg_dir, 'worlds', 'suas_field.sdf'), encoding='utf-8') as f:
        assert f.read() == w, 'worlds/suas_field.sdf differs from the generator: re-run scripts/build_suas_field.py'


def test_primitive_mode_needs_no_fuel():
    b, w = _gen('build_runway', 7, 'primitive')
    assert 'fuel.gazebosim.org' not in w
    assert not b.used_assets
    root = ET.fromstring(w)
    names = [m.get('name') for m in root.find('world').findall('model')]
    assert 'hiker' in names and 'tent' in names
