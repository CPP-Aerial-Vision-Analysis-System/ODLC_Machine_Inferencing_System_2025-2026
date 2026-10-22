"""
sim_worlds/truth.py -- PURPOSE: read and check the ground-truth files truth/<world>.yaml (where the hiker =
mannequin and the tent REALLY are), and answer "which target is nearest to this GPS point?" for scoring.

A truth file lists `targets` (the things the drone must find, with the class name the team's detector uses:
"person" for the mannequin, "tent" for the tent) and `distractors` (standing people, vehicles, debris...).
Each entry has local metres (x, y, z, yaw), GPS (lat, lon in ArduPilot's convention = what the drone's GPS
reads directly above it; lat_wgs84/lon_wgs84 = what a Gazebo NavSat sensor reads) and a bounding box
(size [sx, sy, sz] in the object's own frame, centred at height center_z).
Never give a truth file to the detector: it is only for scoring and for the optional truth_detector stand-in.
"""

import math

import yaml

from sim_worlds import geo

#: Detector class names -> class ids, exactly as detection/detection/new_od.py CLASS_ID writes them.
CLASS_IDS = {'person': '0', 'tent': '1', 'object': '2'}
TARGET_CLASSES = ('person', 'tent')
ENTRY_KEYS = ('id', 'class', 'x', 'y', 'z', 'yaw', 'lat', 'lon', 'size', 'center_z')


def load_truth(path):
    """Read a truth YAML file and return it as a dict. Raises FileNotFoundError / yaml errors."""
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def targets(truth, cls=None):
    """The target entries (list of dicts), optionally only those with class `cls` ('person' or 'tent')."""
    return [t for t in truth.get('targets', []) if cls is None or t.get('class') == cls]


def home_of(truth):
    """The world origin of a truth file as a geo.Home."""
    h = truth['home']
    return geo.Home(float(h['lat']), float(h['lon']), float(h['alt_msl']), float(h.get('heading', 0.0)))


def validate_truth(truth):
    """Check a truth dict. Returns a list of problems ([] = all good)."""
    p = []
    for key in ('world_name', 'home', 'targets', 'distractors', 'search_areas'):
        if key not in truth:
            p.append(f'missing top-level key "{key}"')
    if p:
        return p
    try:
        home = home_of(truth)
    except Exception as exc:                                   # noqa: BLE001
        return [f'bad home: {exc}']
    classes = [t.get('class') for t in truth['targets']]
    for c in TARGET_CLASSES:
        if classes.count(c) < 1:
            p.append(f'no target of class "{c}"')
    ids = set()
    for group in ('targets', 'distractors'):
        for t in truth[group]:
            for k in ENTRY_KEYS:
                if k not in t:
                    p.append(f'{group} entry {t.get("id", "?")} is missing "{k}"')
            if t.get('id') in ids:
                p.append(f'duplicate id "{t.get("id")}"')
            ids.add(t.get('id'))
            if group == 'targets':
                if t.get('class') not in TARGET_CLASSES:
                    p.append(f'target {t.get("id")} has class "{t.get("class")}" (must be person or tent)')
                if str(t.get('class_id')) != CLASS_IDS.get(t.get('class'), '?'):
                    p.append(f'target {t.get("id")} class_id {t.get("class_id")} does not match its class')
            if all(k in t for k in ('x', 'y', 'lat', 'lon')):
                x, y = geo.latlon_to_enu(t['lat'], t['lon'], home)
                if math.hypot(x - t['x'], y - t['y']) > 0.05:
                    p.append(f'{t["id"]}: lat/lon and x/y disagree by {math.hypot(x - t["x"], y - t["y"]):.2f} m')
    return p


def nearest_target(truth, lat, lon, cls=None):
    """The target nearest to (lat, lon) [ArduPilot convention], optionally of class cls.

    Returns (target_dict, distance_m) or (None, inf) when there is no target.
    """
    home = home_of(truth)
    x, y = geo.latlon_to_enu(lat, lon, home)
    best, best_d = None, float('inf')
    for t in targets(truth, cls):
        d = math.hypot(t['x'] - x, t['y'] - y)
        if d < best_d:
            best, best_d = t, d
    return best, best_d
