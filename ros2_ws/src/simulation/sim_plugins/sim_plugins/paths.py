"""
sim_plugins/paths.py -- PURPOSE: the few folders sim_plugins needs, found the same way as the rest of the
simulator finds them (sim_bringup/sim_paths.py), without importing sim_bringup.

Why a copy instead of an import: sim_bringup's launch files import sim_plugins, so sim_plugins must not
need sim_bringup to work (no circular dependency, and the profile tools also run where only this package
is on the Python path, e.g. in the unit tests on any computer). The rules are the same:

    sim_home()      $ASTRA_SIM_HOME, else ~/.astra_sim              (run folders, generated files)
    repo_dir()      $ASTRA_REPO, else ~/ODLC_Machine_Inferencing_System_2025-2026   (the Linux-side copy)
    profiles_dir()  $ASTRA_PROFILES_DIR, else <repo>/simulation/profiles
    sitl_dir()      $ASTRA_SITL_DIR, else ~/sitl                    (02_setup_ardupilot.sh installs here)
    sitl_versions_dir()  $ASTRA_SITL_VERSIONS, else <sitl_dir>/versions   (fetch_sitl.sh installs here)
"""

from __future__ import annotations

import os
from pathlib import Path

#: The folder name of the team repo's Linux-side copy (made by simulation/setup/03_build.sh).
REPO_FOLDER_NAME = 'ODLC_Machine_Inferencing_System_2025-2026'


def expand(path: str) -> str:
    """Expand ~ and $VARIABLES in a path string; an empty string stays empty."""
    return os.path.expandvars(os.path.expanduser(path)) if path else ''


def truthy(text) -> bool:
    """True for 'true', '1', 'yes', 'on' (any case) and the boolean True; False for anything else.

    Launch arguments always arrive as strings, so this is how switches are read.
    """
    if isinstance(text, bool):
        return text
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def sim_home() -> Path:
    """Folder for everything the simulator writes at run time ($ASTRA_SIM_HOME or ~/.astra_sim)."""
    return Path(expand(os.environ.get('ASTRA_SIM_HOME', '') or '~/.astra_sim'))


def generated_dir() -> Path:
    """Folder for files made at run time (resolved profiles, cleaned parameter files, filtered launch files)."""
    return sim_home() / 'generated'


def repo_dir() -> Path:
    """The team repo the simulator runs from ($ASTRA_REPO, set by simulation/setup/sim_env.sh).

    Without $ASTRA_REPO: the source tree this file lives in, when it is inside a repo checkout
    (ros2_ws/src/simulation/sim_plugins/sim_plugins/paths.py -> 6 levels up), else the Linux-side copy.
    """
    env = os.environ.get('ASTRA_REPO', '')
    if env:
        return Path(expand(env))
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'simulation' / 'profiles').is_dir() and (parent / 'ros2_ws').is_dir():
            return parent
    return Path(expand('~/' + REPO_FOLDER_NAME))


def profiles_dir() -> Path:
    """Where the shared profiles live ($ASTRA_PROFILES_DIR, else <repo>/simulation/profiles)."""
    env = os.environ.get('ASTRA_PROFILES_DIR', '')
    return Path(expand(env)) if env else repo_dir() / 'simulation' / 'profiles'


def sitl_dir() -> Path:
    """The native SITL folder made by simulation/setup/02_setup_ardupilot.sh ($ASTRA_SITL_DIR or ~/sitl)."""
    return Path(expand(os.environ.get('ASTRA_SITL_DIR', '') or '~/sitl'))


def sitl_versions_dir() -> Path:
    """Where simulation/setup/fetch_sitl.sh puts other SITL versions (<sitl_dir>/versions/<VERSION>/)."""
    env = os.environ.get('ASTRA_SITL_VERSIONS', '')
    return Path(expand(env)) if env else sitl_dir() / 'versions'
