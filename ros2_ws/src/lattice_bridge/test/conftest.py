"""
lattice_bridge/test/conftest.py -- PURPOSE: shared pytest setup for the lattice_bridge tests.

  * puts the SOURCE folder of lattice_bridge on the Python path, so the tests run on any computer without
    building the workspace and test the files you just edited. Set LATTICE_BRIDGE_USE_INSTALLED=1 to do the
    opposite: test the INSTALLED copy (after `colcon build` and `source install/setup.bash`), which is how
    the clean-overlay run proves that packaging works;
  * the fixtures `now` (a fixed time, so no test depends on the real clock) and `mission` (a fixed mission id);
  * keeps Python's loggers NORMAL when ROS 2 is sourced. ROS installs pytest plugins (launch_testing, launch_ros)
    that are loaded into every pytest run; they import launch.logging, which calls
    logging.setLoggerClass(LaunchLogger), so every logger created afterwards has `propagate = False`. pytest's
    `caplog` listens at the root logger, so the tests that check a log line saw nothing in WSL with ROS sourced (and
    passed on Windows). Found in the first WSL run with ROS, 2026-10-31. Two parts, both repeated before every test
    (launch.logging.reset() would set the class again): (1) the logger class goes back to logging.Logger, so a
    logger first made inside a test is normal; (2) this package's loggers that were already made as LaunchLoggers
    pass their records on again. Only this test process is changed: the bridge program never imports launch.

Shared constants and helpers (golden files, scenario data) live in support.py next to the tests.
"""

import logging
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


def _plain_logger_class() -> None:
    """Undo launch.logging's setLoggerClass(LaunchLogger) for this test process (no-op without ROS)."""
    if logging.getLoggerClass() is not logging.Logger and \
            getattr(logging.getLoggerClass(), '__module__', '').startswith('launch'):
        logging.setLoggerClass(logging.Logger)


_plain_logger_class()                               # before any test module makes its loggers


@pytest.fixture(autouse=True)
def _lattice_loggers_reach_the_root():
    """Before every test: new loggers are plain, and this package's loggers reach the root (see the module text)."""
    _plain_logger_class()
    for name, logger in list(logging.Logger.manager.loggerDict.items()):
        if (name == 'lattice_bridge' or name.startswith('lattice_bridge.')) and isinstance(logger, logging.Logger):
            logger.propagate = True
    yield


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
