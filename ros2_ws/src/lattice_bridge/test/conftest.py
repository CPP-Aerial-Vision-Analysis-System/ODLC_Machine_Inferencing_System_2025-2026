"""
lattice_bridge/test/conftest.py -- PURPOSE: shared pytest setup for the lattice_bridge tests.

  * puts the SOURCE folder of lattice_bridge on the Python path, so the tests run on any computer without
    building the workspace and test the files you just edited. Set LATTICE_BRIDGE_USE_INSTALLED=1 to do the
    opposite: test the INSTALLED copy (after `colcon build` and `source install/setup.bash`), which is how
    the clean-overlay run proves that packaging works;
  * the fixtures `now` (a fixed time, so no test depends on the real clock) and `mission` (a fixed mission id).

Shared constants and helpers (golden files, scenario data) live in support.py next to the tests.
"""

import os
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PKG = HERE.parent                                   # ros2_ws/src/lattice_bridge

if os.environ.get('LATTICE_BRIDGE_USE_INSTALLED') != '1' and str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))


@pytest.fixture
def now():
    """The fixed 'publish time' used by tests: 2026-10-24 15:30:00 UTC."""
    from support import NOW
    return NOW


@pytest.fixture
def mission():
    """A fixed mission id."""
    from support import MISSION
    return MISSION
