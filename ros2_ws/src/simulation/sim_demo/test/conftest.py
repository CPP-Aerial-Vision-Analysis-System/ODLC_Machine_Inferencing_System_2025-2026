"""
sim_demo/test/conftest.py -- PURPOSE: shared pytest setup for the sim_demo tests.

Puts the package source folder first on sys.path (so `import sim_demo...` works with plain
`python3 -m pytest test/` without building), and provides fixtures: the fake test world
(test/fixtures/worlds) and its loaded WorldInfo.
"""
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.dirname(HERE)
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

FIXTURE_WORLDS = os.path.join(HERE, 'fixtures', 'worlds')


@pytest.fixture
def worlds_dir():
    """Folder of the fake sim_worlds files used by the tests."""
    return FIXTURE_WORLDS


@pytest.fixture
def world(worlds_dir):
    """The loaded fake world 'testfield' (home 38.3154, -76.549)."""
    from sim_demo.world_info import load_world
    return load_world('testfield', worlds_dir)
