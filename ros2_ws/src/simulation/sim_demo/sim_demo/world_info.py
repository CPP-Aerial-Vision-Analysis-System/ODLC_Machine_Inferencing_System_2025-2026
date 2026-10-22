"""
sim_demo/world_info.py -- PURPOSE: read a simulated world's description files: the world
config (sim_worlds/config/<world>.yaml: home GPS point, spawn, mission file, truth file) and
the answer key of where the targets really are (sim_worlds/truth/<world>.yaml).

The contract fixes the config keys {world_file, home: "lat,lon,alt_msl,heading_deg", spawn,
mission, truth}. The truth file layout belongs to the sim_worlds package, so this loader is
deliberately forgiving: it understands targets written in GPS degrees OR in Gazebo metres,
listed or keyed by name, with the class under any of the usual key names. Anything it cannot
understand is skipped with a clear message instead of crashing.

Output: a WorldInfo with the GPS origin, the targets (each with BOTH lat/lon and east/north)
and the search-area polygon (east/north metres), ready for the panel, the stand-in detector,
the markers and the scorer.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

from .geo import GeoOrigin, parse_home

# How each class name in a truth file is understood. The team's detector reports only
# "person" (class_id "0") and "tent" (class_id "1"), so everything maps to those two.
CLASS_ALIASES = {
    'person': 'person', 'mannequin': 'person', 'manikin': 'person', 'hiker': 'person',
    'human': 'person', 'dummy': 'person', 'people': 'person',
    'tent': 'tent', 'shelter': 'tent',
}

# Default object sizes (length, width, height in metres) when the truth file gives none.
DEFAULT_SIZES = {
    ('person', 'lying'): (1.8, 0.5, 0.3),
    ('person', 'sitting'): (0.9, 0.6, 1.0),
    ('person', 'standing'): (0.5, 0.4, 1.8),
    ('tent', ''): (2.4, 2.1, 1.4),
}


@dataclass
class TruthTarget:
    """One real target in the simulated world (from the answer key).

    Attributes:
        name: label, e.g. 'hiker' or 'tent_1'.
        cls: 'person' or 'tent' (the detector's two classes).
        lat, lon: GPS position [deg].
        east, north, up: Gazebo world position [m] (bottom centre of the object).
        yaw_deg: ENU yaw of the object's long axis [deg].
        size: (length, width, height) [m].
        posture: 'lying' / 'sitting' / 'standing' for persons, '' otherwise.
    """

    name: str
    cls: str
    lat: float
    lon: float
    east: float
    north: float
    up: float = 0.0
    yaw_deg: float = 0.0
    size: tuple[float, float, float] = (1.0, 1.0, 1.0)
    posture: str = ''


@dataclass
class WorldInfo:
    """Everything sim_demo needs to know about one simulated world.

    Attributes:
        name: world name, e.g. 'runway'.
        origin: GPS point that is (0, 0, 0) in Gazebo (= SITL home).
        heading_deg: home heading from the config (informational).
        targets: the answer key.
        search_area: polygon of (east, north) metres; empty if unknown.
        mission_path, truth_path, world_file, config_path: absolute paths ('' if unknown).
        spawn: dict with x, y, z, yaw of the drone's start.
        warnings: human-readable problems found while loading.
    """

    name: str
    origin: GeoOrigin
    heading_deg: float = 0.0
    targets: list[TruthTarget] = field(default_factory=list)
    search_area: list[tuple[float, float]] = field(default_factory=list)
    search_areas: list = field(default_factory=list)     # [(name, polygon)], all areas
    mission_path: str = ''
    truth_path: str = ''
    world_file: str = ''
    config_path: str = ''
    spawn: dict = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    survey_hints: dict = field(default_factory=dict)   # config survey.planned (lanes, hold_wp ...)
    distractors: list[TruthTarget] = field(default_factory=list)  # look-alikes (standing people ...)

    @property
    def hold_index(self) -> int:
        """Mission index of the hold waypoint from the config (-1 = unknown)."""
        try:
            return int(self.survey_hints.get('hold_wp', -1))
        except (TypeError, ValueError):
            return -1


def normalize_class(text: str | None) -> str | None:
    """Map any target class word to 'person' or 'tent' (None if it is neither)."""
    if not text:
        return None
    t = str(text).strip().lower()
    if t in CLASS_ALIASES:
        return CLASS_ALIASES[t]
    for key, value in CLASS_ALIASES.items():
        if key in t:
            return value
    return None


def find_worlds_dir(override: str = '') -> str:
    """Folder of the sim_worlds package files (config/, truth/, missions/, worlds/).

    Order: `override` (parameter) -> ASTRA_WORLDS_DIR environment variable -> the installed
    sim_worlds share folder. Returns '' if none is found.
    """
    if override:
        return os.path.abspath(os.path.expanduser(override))
    env = os.environ.get('ASTRA_WORLDS_DIR', '')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('sim_worlds')
    except Exception:  # noqa: BLE001 - not installed
        return ''


def _load_yaml(path: str):
    import yaml  # python3-yaml
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _first(d: dict, keys, default=None):
    for k in keys:
        if isinstance(d, dict) and k in d and d[k] is not None:
            return d[k]
    return default


def _num_list(value) -> list[float] | None:
    """[1, 2] / (1, 2) / "1, 2" / "1 2" -> [1.0, 2.0]; None if not a list of numbers."""
    if isinstance(value, str):
        parts = value.replace(',', ' ').split()
    elif isinstance(value, (list, tuple)):
        parts = list(value)
    else:
        return None
    try:
        return [float(p) for p in parts]
    except (TypeError, ValueError):
        return None


def _looks_geo(a: float, b: float, origin: GeoOrigin) -> bool:
    """True if (a, b) looks like (lat, lon) degrees near the world's origin."""
    return abs(a - origin.lat) < 1.0 and abs(b - origin.lon) < 1.0


def _point_to_en(value, origin: GeoOrigin, geo_hint: bool | None = None) -> tuple[float, float] | None:
    """Turn one polygon/target point (GPS or metres, list/dict/string) into (east, north)."""
    if isinstance(value, dict):
        lat = _first(value, ('lat', 'latitude'))
        lon = _first(value, ('lon', 'lng', 'long', 'longitude'))
        if lat is not None and lon is not None:
            e, n, _ = origin.to_enu(float(lat), float(lon))
            return e, n
        x = _first(value, ('x', 'east', 'e'))
        y = _first(value, ('y', 'north', 'n'))
        if x is not None and y is not None:
            return float(x), float(y)
        return None
    nums = _num_list(value)
    if not nums or len(nums) < 2:
        return None
    a, b = nums[0], nums[1]
    is_geo = _looks_geo(a, b, origin) if geo_hint is None else geo_hint
    if is_geo:
        e, n, _ = origin.to_enu(a, b)
        return e, n
    return a, b


def parse_polygon(value, origin: GeoOrigin) -> list[tuple[float, float]]:
    """Parse a polygon given as a list of points or as a dict holding one.

    Accepts points as [lat, lon] / {lat:, lon:} / [x, y] / {x:, y:} / "lat,lon", and dicts
    like {points: [...]} / {latlon: [...]} / {xy: [...]} / {center: [x, y], size: [l, w]}.
    Returns (east, north) points; empty list if it cannot be understood.
    """
    geo_hint = None
    if isinstance(value, dict):
        # sim_worlds writes corners_xy (metres) and corners_latlon (GPS): prefer the metres.
        for key, hint in (('corners_xy', False), ('corners_enu', False), ('corners_latlon', True)):
            if key in value:
                pts = [q for q in (_point_to_en(p, origin, hint) for p in value[key] or []) if q]
                if len(pts) >= 3:
                    return pts
        if 'center' in value and 'size' in value:
            c = _point_to_en(value['center'], origin)
            s = _num_list(value['size'])
            if c and s and len(s) >= 2:
                if 'yaw_deg' in value:
                    yaw = math.radians(float(value['yaw_deg']))
                else:     # sim_worlds: radians (lane direction, 0 = east)
                    v = float(value.get('yaw', 0.0) or 0.0)
                    yaw = v if abs(v) <= 2 * math.pi + 1e-6 else math.radians(v)
                hl, hw = s[0] / 2.0, s[1] / 2.0
                cs, sn = math.cos(yaw), math.sin(yaw)
                return [(c[0] + cs * dx - sn * dy, c[1] + sn * dx + cs * dy)
                        for dx, dy in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]
            return []
        for key, hint in (('latlon', True), ('gps', True), ('geo', True), ('lat_lon', True),
                          ('xy', False), ('enu', False), ('local', False), ('points', None),
                          ('polygon', None), ('vertices', None)):
            if key in value:
                value, geo_hint = value[key], hint
                break
        else:
            return []
    if not isinstance(value, (list, tuple)):
        return []
    pts = []
    for p in value:
        q = _point_to_en(p, origin, geo_hint)
        if q is not None:
            pts.append(q)
    return pts if len(pts) >= 3 else []


def _yaw_deg_from(entry: dict) -> float:
    if 'yaw_deg' in entry:
        return float(entry['yaw_deg'])
    if 'heading_deg' in entry:
        return float(entry['heading_deg'])
    if 'yaw_rad' in entry:
        return math.degrees(float(entry['yaw_rad']))
    if 'yaw' in entry:
        v = float(entry['yaw'])
        return math.degrees(v) if abs(v) <= 2 * math.pi + 1e-6 else v
    pose = _num_list(entry.get('pose')) if 'pose' in entry else None
    if pose and len(pose) >= 6:
        return math.degrees(pose[5])          # SDF-style pose: x y z roll pitch yaw (rad)
    return 0.0


def parse_target(entry: dict, origin: GeoOrigin, default_name: str = '') -> tuple[TruthTarget | None, str]:
    """Turn one truth-file entry into a TruthTarget.

    Returns:
        (target, problem): target is None when the entry could not be understood, and
        `problem` then says why (for a one-time warning).
    """
    if not isinstance(entry, dict):
        return None, f'target entry {default_name!r} is not a mapping'
    name = str(_first(entry, ('name', 'id', 'label_name'), default_name) or default_name)
    cls = normalize_class(_first(entry, ('class', 'cls', 'type', 'kind', 'category',
                                         'label', 'object_class', 'model')))
    if cls is None:
        cls = normalize_class(name)
    if cls is None:
        return None, f'target {name!r}: class is not person/mannequin/hiker or tent'
    posture = str(_first(entry, ('posture', 'pose_type', 'state', 'position_type'), '') or '')
    if not posture and isinstance(entry.get('pose'), str):
        posture = entry['pose']                       # sim_worlds: pose: lying_face_up
    posture = posture.lower() if cls == 'person' else ''
    for word in ('lying', 'sitting', 'standing'):
        if word in posture:
            posture = word
    if cls == 'person' and posture not in ('lying', 'sitting', 'standing'):
        posture = 'lying'

    lat = _first(entry, ('lat', 'latitude'))
    lon = _first(entry, ('lon', 'lng', 'long', 'longitude'))
    if lat is None or lon is None:
        g = _first(entry, ('gps', 'latlon', 'lat_lon', 'geo'))
        nums = _num_list(g) if g is not None else None
        if nums and len(nums) >= 2:
            lat, lon = nums[0], nums[1]
    east = north = None
    up = float(_first(entry, ('z', 'up', 'alt_agl', 'height_above_ground'), 0.0) or 0.0)
    x = _first(entry, ('x', 'east'))
    y = _first(entry, ('y', 'north'))
    if x is not None and y is not None:
        east, north = float(x), float(y)
    else:
        xyz = _first(entry, ('position', 'xyz', 'enu', 'local', 'pose', 'world_xyz'))
        if isinstance(xyz, dict):
            q = _point_to_en(xyz, origin)
            if q:
                east, north = q
                up = float(xyz.get('z', xyz.get('up', up)) or up)
        else:
            nums = _num_list(xyz) if xyz is not None else None
            if nums and len(nums) >= 2:
                if _looks_geo(nums[0], nums[1], origin):
                    lat, lon = nums[0], nums[1]
                else:
                    east, north = nums[0], nums[1]
                    if len(nums) >= 3:
                        up = nums[2]
    if east is None and lat is not None and lon is not None:
        east, north, _ = origin.to_enu(float(lat), float(lon))
    if east is None:
        return None, f'target {name!r}: no position (need lat/lon or x/y)'
    if lat is None or lon is None:
        lat, lon, _ = origin.to_geo(east, north)

    size_raw = _first(entry, ('size', 'dimensions', 'dims', 'size_m', 'extent'))
    size = None
    if isinstance(size_raw, dict):
        size = (float(size_raw.get('length', size_raw.get('l', 1.0))),
                float(size_raw.get('width', size_raw.get('w', 1.0))),
                float(size_raw.get('height', size_raw.get('h', 1.0))))
    else:
        nums = _num_list(size_raw) if size_raw is not None else None
        if nums and len(nums) >= 3:
            size = (nums[0], nums[1], nums[2])
        elif nums and len(nums) == 2:
            size = (nums[0], nums[1], DEFAULT_SIZES.get((cls, posture), (1, 1, 1))[2])
    if size is None:
        size = DEFAULT_SIZES.get((cls, posture)) or DEFAULT_SIZES.get((cls, ''), (1.0, 1.0, 1.0))
    # sim_worlds gives the bounding box centre separately ('center' [x, y, z], 'center_z'):
    # the box can be offset from the model origin, and its bottom is center_z - height / 2.
    box_e, box_n = float(east), float(north)
    center = _num_list(entry.get('center')) if 'center' in entry else None
    if center and len(center) >= 2 and not _looks_geo(center[0], center[1], origin):
        box_e, box_n = center[0], center[1]
        if len(center) >= 3 and 'center_z' not in entry:
            up = center[2] - float(size[2]) / 2.0
    if 'center_z' in entry:
        try:
            up = float(entry['center_z']) - float(size[2]) / 2.0
        except (TypeError, ValueError):
            pass
    return TruthTarget(name=name, cls=cls, lat=float(lat), lon=float(lon), east=box_e,
                       north=box_n, up=max(-100.0, float(up)), yaw_deg=_yaw_deg_from(entry),
                       size=tuple(float(s) for s in size), posture=posture), ''


def parse_truth(data, origin: GeoOrigin) -> tuple[list[TruthTarget], list[tuple[float, float]], list[str]]:
    """Parse a loaded truth YAML document.

    Returns:
        (targets, search_area_polygon_en, warnings)
    """
    warnings: list[str] = []
    targets: list[TruthTarget] = []
    if data is None:
        return targets, [], ['truth file is empty']
    raw = data
    if isinstance(data, dict):
        raw = _first(data, ('targets', 'objects', 'ground_truth', 'truth', 'items', 'odlc'))
        if raw is None:
            # Maybe the whole document is {name: {...}} keyed by target name.
            raw = {k: v for k, v in data.items() if isinstance(v, dict) and (
                normalize_class(k) or normalize_class(_first(v, ('class', 'type', 'kind'))))}
    entries = []
    if isinstance(raw, dict):
        entries = [(str(k), v) for k, v in raw.items()]
    elif isinstance(raw, list):
        entries = [(f'target_{i}', v) for i, v in enumerate(raw)]
    for default_name, entry in entries:
        # default_name (dict key or "target_<i>") is only used when the entry has no name/id.
        t, problem = parse_target(entry, origin, default_name)
        if t is not None:
            targets.append(t)
        else:
            warnings.append(problem)
    areas = parse_search_areas(data, origin)
    if not targets:
        warnings.append('no person/tent targets found in the truth file')
    return targets, (areas[0][1] if areas else []), warnings


def parse_search_areas(data, origin: GeoOrigin) -> list[tuple[str, list[tuple[float, float]]]]:
    """All search areas of a truth/config document as (name, polygon of (east, north)).

    Understands sim_worlds' `search_areas: [{name, center, size, yaw, corners_xy, ...}]` and a
    single polygon under search_area / search_polygon / area.
    """
    if not isinstance(data, dict):
        return []
    out = []
    many = data.get('search_areas')
    if isinstance(many, list):
        for i, a in enumerate(many):
            poly = parse_polygon(a, origin)
            if poly:
                name = str(a.get('name', f'area {i + 1}')) if isinstance(a, dict) else f'area {i + 1}'
                out.append((name, poly))
    if out:
        return out
    for key in ('search_area', 'search_area_polygon', 'search_polygon', 'search_zone',
                'search_boundary', 'area'):
        if key in data:
            poly = parse_polygon(data[key], origin)
            if poly:
                return [(key, poly)]
    return []


def _resolve(base_dir: str, rel: str) -> str:
    if not rel:
        return ''
    rel = os.path.expanduser(str(rel))
    return rel if os.path.isabs(rel) else os.path.normpath(os.path.join(base_dir, rel))


def load_world(world: str, worlds_dir: str = '', truth_override: str = '') -> WorldInfo:
    """Load a world's config and truth file.

    Args:
        world: world name (e.g. 'runway') or a path to a config YAML.
        worlds_dir: sim_worlds folder ('' = find it, see find_worlds_dir()).
        truth_override: use this truth YAML instead of the one named in the config.
    Returns:
        WorldInfo (with `warnings` filled if something was missing).
    Raises:
        FileNotFoundError: the world config could not be found.
        ValueError: the config has no usable home.
    """
    if world.endswith(('.yaml', '.yml')) and os.path.isfile(world):
        config_path = os.path.abspath(world)
        base = os.path.dirname(os.path.dirname(config_path))   # .../config/x.yaml -> ...
        name = os.path.splitext(os.path.basename(world))[0]
    else:
        base = find_worlds_dir(worlds_dir)
        if not base:
            raise FileNotFoundError(
                'cannot find the sim_worlds package files: build sim_worlds or set the '
                'worlds_dir parameter / ASTRA_WORLDS_DIR')
        config_path = os.path.join(base, 'config', f'{world}.yaml')
        name = world
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f'world config not found: {config_path}')
    cfg = _load_yaml(config_path) or {}
    if isinstance(cfg, dict) and 'home' not in cfg and len(cfg) == 1:
        only = next(iter(cfg.values()))
        if isinstance(only, dict):
            cfg = only
    home = cfg.get('home') if isinstance(cfg, dict) else None
    if isinstance(home, dict):
        lat0 = float(_first(home, ('lat', 'latitude')))
        lon0 = float(_first(home, ('lon', 'longitude', 'lng')))
        alt0 = float(_first(home, ('alt', 'alt_msl', 'altitude'), 0.0))
        hdg = float(_first(home, ('heading', 'heading_deg', 'yaw'), 0.0))
    elif home is not None:
        lat0, lon0, alt0, hdg = parse_home(home)
    else:
        raise ValueError(f'{config_path}: no "home" entry')
    origin = GeoOrigin(lat0, lon0, alt0)
    info = WorldInfo(name=str(cfg.get('world_name', cfg.get('name', name))), origin=origin,
                     heading_deg=hdg,
                     config_path=config_path,
                     mission_path=_resolve(base, cfg.get('mission', '')),
                     truth_path=_resolve(base, truth_override or cfg.get('truth', '')),
                     world_file=_resolve(base, cfg.get('world_file', '')),
                     spawn=dict(cfg.get('spawn') or {}),
                     survey_hints=dict(((cfg.get('survey') or {}).get('planned') or {})
                                       if isinstance(cfg.get('survey'), dict) else {}))
    if truth_override:
        info.truth_path = os.path.abspath(os.path.expanduser(truth_override))
    if info.truth_path and os.path.isfile(info.truth_path):
        try:
            truth_doc = _load_yaml(info.truth_path)
            targets, area, warns = parse_truth(truth_doc, origin)
            info.targets, info.search_area = targets, area
            info.search_areas = parse_search_areas(truth_doc, origin)
            info.warnings.extend(warns)
            # sim_worlds also lists distractors; keep the ones a detector could call person/tent.
            if isinstance(truth_doc, dict) and isinstance(truth_doc.get('distractors'), list):
                for i, entry in enumerate(truth_doc['distractors']):
                    if isinstance(entry, dict) and normalize_class(entry.get('class')):
                        d, _ = parse_target(entry, origin, f'distractor_{i}')
                        if d is not None:
                            info.distractors.append(d)
        except Exception as exc:  # noqa: BLE001 - a broken truth file must not kill the panel
            info.warnings.append(f'could not read truth file {info.truth_path}: {exc}')
    else:
        info.warnings.append(f'truth file not found: {info.truth_path or "(not set in config)"}')
    if not info.search_areas:
        info.search_areas = parse_search_areas(cfg, origin)
        if info.search_areas and not info.search_area:
            info.search_area = info.search_areas[0][1]
    return info
