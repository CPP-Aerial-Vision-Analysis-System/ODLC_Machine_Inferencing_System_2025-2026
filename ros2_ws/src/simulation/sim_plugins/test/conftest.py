"""
sim_plugins/test/conftest.py -- PURPOSE: shared pytest fixtures for the sim_plugins tests.

  * puts the SOURCE folders of sim_plugins and sim_bringup on the Python path, so the tests run on any computer
    without building the workspace (and test the files you just edited, not an old installed copy);
  * REPO / PROFILES / SIM: paths inside the repo (the real simulation/profiles folder is tested too);
  * sim_home: a temporary ~/.astra_sim and ~/sitl for each test (nothing in your real home is touched);
  * fake_arducopter(path, version): writes a tiny file that looks like an ArduCopter SITL program
    (ELF header + the version text ArduPilot compiles in), executable;
  * include_location(include, context): the launch file an IncludeLaunchDescription really loads, worked out
    with a LaunchContext the way the launch system does it (see its docstring for why `.location` alone is
    not enough in ROS 2 Humble).
"""

import os
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PKG = HERE.parent                                   # ros2_ws/src/simulation/sim_plugins
SIM = PKG.parent                                    # ros2_ws/src/simulation
REPO = SIM.parent.parent.parent                     # the repo root
PROFILES = REPO / 'simulation' / 'profiles'

for path in (PKG, SIM / 'sim_bringup'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))


def write_fake_arducopter(path: Path, version: str = '4.7.1', commit: str = 'abcdef12') -> Path:
    """A file that looks like an ArduCopter SITL program (ELF magic + 'ArduCopter V<version> (<commit>)')."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'\x7fELF' + b'\0' * 60 + f'ArduCopter V{version} ({commit})'.encode() + b'\0' * 32)
    path.chmod(0o755)
    return path


@pytest.fixture
def fake_arducopter():
    return write_fake_arducopter


@pytest.fixture
def sim_home(tmp_path, monkeypatch):
    """A private ~/.astra_sim, ~/sitl and ~/sitl/versions for one test. Returns a dict of the paths."""
    home = tmp_path / 'astra_sim'
    sitl = tmp_path / 'sitl'
    versions = sitl / 'versions'
    for p in (home, versions):
        p.mkdir(parents=True)
    monkeypatch.setenv('ASTRA_SIM_HOME', str(home))
    monkeypatch.setenv('ASTRA_SITL_DIR', str(sitl))
    monkeypatch.setenv('ASTRA_SITL_VERSIONS', str(versions))
    monkeypatch.setenv('ASTRA_PROFILES_DIR', str(PROFILES))
    monkeypatch.delenv('ARDUPILOT_SITL_BIN', raising=False)
    return {'home': home, 'sitl': sitl, 'versions': versions, 'tmp': tmp_path}


@pytest.fixture
def profiles_dir():
    return PROFILES


@pytest.fixture
def repo():
    return REPO


def include_location(include, context) -> str:
    """The file an IncludeLaunchDescription loads, found exactly the way the launch system finds it.

    A launch-description source keeps its location as a list of launch SUBSTITUTIONS. Until they are
    performed, `source.location` only joins their text forms, and in ROS 2 Humble (launch 1.0.x) that text
    is the object's repr ("<...TextSubstitution object at 0x...>"), never the path. So the location is
    performed in the LaunchContext with get_launch_description(context): that is what
    IncludeLaunchDescription.execute() does at launch time, and it also READS the file, so a missing or
    broken launch file (for example a bad generated copy of tracker.launch.xml) fails the test. Nothing is
    started: loading a launch file only builds its LaunchDescription.
    """
    source = include.launch_description_source
    source.get_launch_description(context)
    return source.location


def bash_available() -> bool:
    """True when a POSIX bash can run the repo's scripts (Linux / WSL)."""
    return os.name == 'posix' and Path('/bin/bash').exists()
