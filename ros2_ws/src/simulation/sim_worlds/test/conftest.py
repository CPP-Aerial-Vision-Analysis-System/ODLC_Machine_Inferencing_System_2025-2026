"""
sim_worlds/test/conftest.py -- PURPOSE: pytest setup shared by every test of this package: make the package
source tree importable (so the tests check THIS code, installed or not) and give tests the package folder.
"""

import os
import sys

import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG not in sys.path:
    sys.path.insert(0, PKG)
SCRIPTS = os.path.join(PKG, 'scripts')
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)


@pytest.fixture
def pkg_dir():
    """Absolute path of the sim_worlds package source folder."""
    return PKG
