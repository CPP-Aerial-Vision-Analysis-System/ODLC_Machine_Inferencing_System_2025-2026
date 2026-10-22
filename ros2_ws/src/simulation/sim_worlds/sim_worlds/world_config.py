"""
sim_worlds/world_config.py -- PURPOSE: find and read the per-world configuration file config/<world>.yaml and
turn its relative paths into absolute ones, so launch files and scripts never hard-code where things live.

Contract fields of config/<world>.yaml (all paths are relative to the sim_worlds share folder):
  world_file   worlds/<world>.sdf
  home         "lat,lon,alt_msl,heading_deg"  -> SITL --home (equals the world's <spherical_coordinates>)
  spawn        {x, y, z, yaw}                 drone start pose in metres / radians (ENU)
  mission      missions/<world>_survey.waypoints
  truth        truth/<world>.yaml
Extra fields written by the generators: world_name, description, search_areas, survey, gui.

Use from Python:   from sim_worlds.world_config import load_world_config; cfg = load_world_config('runway')
Use from a shell:  ros2 run sim_worlds world_info.py runway home
"""

import glob
import os
from dataclasses import dataclass, field

import yaml

from sim_worlds.geo import Home

PACKAGE = 'sim_worlds'
REQUIRED = ('world_file', 'home', 'spawn', 'mission', 'truth')


def share_dir():
    """The sim_worlds share folder: the installed package if ROS can find it, else this source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PACKAGE)
    except Exception:                                         # noqa: BLE001 (not installed / no ROS)
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        if os.path.isdir(os.path.join(here, 'config')):
            return here
        raise FileNotFoundError('cannot find the sim_worlds share folder: source your workspace '
                                '(source install/setup.bash) or run from the source tree')


def available_worlds(share=None):
    """Names of every world that has a config file, e.g. ['runway', 'suas_field']."""
    share = share or share_dir()
    return sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(share, 'config', '*.yaml')))


@dataclass
class WorldConfig:
    """A loaded config/<world>.yaml with absolute paths.

    name: config name ('runway'). world_name: the <world name="..."> inside the SDF (gz services use it).
    world_file, mission, truth: absolute paths. home: geo.Home. spawn: dict(x, y, z, yaw) [m, rad].
    search_areas, survey: extra sections (lists/dicts, may be empty). raw: the whole YAML as a dict.
    """

    name: str
    world_name: str
    world_file: str
    home: Home
    spawn: dict
    mission: str
    truth: str
    share: str
    search_areas: list = field(default_factory=list)
    survey: dict = field(default_factory=dict)
    raw: dict = field(default_factory=dict)

    def as_dict(self):
        """Plain dict (for printing): absolute paths, home as the SITL string."""
        return {'name': self.name, 'world_name': self.world_name, 'world_file': self.world_file,
                'home': self.home.sitl_string(), 'spawn': self.spawn, 'mission': self.mission,
                'truth': self.truth, 'share': self.share}


def load_world_config(world, share=None):
    """Load a world config.

    world: a world name ('runway', 'suas_field') or a path to a YAML file. share: optional share folder
    (default: share_dir()). Returns a WorldConfig. Raises FileNotFoundError / ValueError with a clear message.
    """
    share = share or share_dir()
    if os.path.isfile(world):
        path = world
        name = os.path.splitext(os.path.basename(world))[0]
    else:
        path = os.path.join(share, 'config', f'{world}.yaml')
        name = world
        if not os.path.isfile(path):
            raise FileNotFoundError(f'no config for world "{world}" ({path}); known worlds: '
                                    f'{", ".join(available_worlds(share)) or "none"}')
    with open(path, encoding='utf-8') as f:
        raw = yaml.safe_load(f) or {}
    missing = [k for k in REQUIRED if k not in raw]
    if missing:
        raise ValueError(f'{path}: missing field(s) {", ".join(missing)}')
    spawn = raw['spawn']
    for k in ('x', 'y', 'z', 'yaw'):
        if k not in spawn:
            raise ValueError(f'{path}: spawn needs x, y, z and yaw (missing {k})')
    base = os.path.dirname(os.path.dirname(os.path.abspath(path))) if os.path.isfile(world) else share

    def absolute(p):
        return p if os.path.isabs(p) else os.path.normpath(os.path.join(base, p))

    return WorldConfig(name=name, world_name=str(raw.get('world_name', name)), world_file=absolute(raw['world_file']),
                       home=Home.parse(raw['home']), spawn={k: float(spawn[k]) for k in ('x', 'y', 'z', 'yaw')},
                       mission=absolute(raw['mission']), truth=absolute(raw['truth']), share=base,
                       search_areas=list(raw.get('search_areas') or []), survey=dict(raw.get('survey') or {}),
                       raw=raw)
