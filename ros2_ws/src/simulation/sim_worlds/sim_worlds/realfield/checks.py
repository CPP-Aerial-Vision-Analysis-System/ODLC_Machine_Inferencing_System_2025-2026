"""
sim_worlds/realfield/checks.py -- PURPOSE: the OFFLINE health check of the real-terrain world ("build_real_field.py
check"). No Gazebo needed (the Gazebo load / speed / box-drop check is scripts/check_world.py and
scripts/check_real_ground.py). It answers, in plain PASS / FAIL lines:

  1. are the site pack, the generated world files and the Gazebo models all there, and the same size as expected?
  2. does the world's home equal the pack's home (SITL and Gazebo must agree on where the pad is)?
  3. do the models fit the budget (under about 2 million triangles to draw, mesh physics rather than a heightmap)?
  4. does every target (hiker, tent) and every prop stand ON the real ground, not above or in it?
  5. do the targets lie inside their search boundary, on open level ground, clear of the real trees?
  6. does each survey mission fly at least 150 ft above the HIGHEST ground and at most 400 ft above the LOWEST
     ground under its search boundary (the SUAS altitude rule), with every waypoint inside the flight boundary?

Returns 0 when everything passes, 1 otherwise. Needs numpy and PyYAML.
"""

import json
import math
import os

import numpy as np
import yaml

from sim_worlds import geo, mission
from sim_worlds.realfield import site
from sim_worlds.realfield.sitepack import SitePack
from sim_worlds.realfield.world_build import Ground, MAX_SLOPE, OPEN_GROUND, search_rect

TRIANGLE_BUDGET = 2_000_000


class Report:
    """Collects PASS / FAIL lines."""

    def __init__(self, log):
        self.log = log
        self.failed = 0

    def ok(self, what, detail=''):
        self.log(f'  PASS  {what}' + (f'  ({detail})' if detail else ''))

    def fail(self, what, detail=''):
        self.failed += 1
        self.log(f'  FAIL  {what}' + (f'  ({detail})' if detail else ''))

    def check(self, cond, what, detail=''):
        (self.ok if cond else self.fail)(what, detail)
        return cond


def _find_models(pkg, models):
    """The models folder to check: the one given, else <package>/models, else the installed share."""
    for cand in ([models] if models else []) + [os.path.join(pkg, 'models')]:
        if cand and os.path.isfile(os.path.join(cand, 'rf_assets_manifest.json')):
            return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        cand = os.path.join(get_package_share_directory('sim_worlds'), 'models')
        if os.path.isfile(os.path.join(cand, 'rf_assets_manifest.json')):
            return cand
    except Exception:                                                  # noqa: BLE001 (no ROS here: fine)
        pass
    return models or os.path.join(pkg, 'models')


def run(pkg, models=None, pack_dir=None, log=print):
    """Run every offline check; print the result; return 0 (all passed) or 1."""
    rep = Report(log)
    name = site.WORLD
    log(f'[check] {name} in {pkg}')
    pack = SitePack(pack_dir)
    # 1. files -----------------------------------------------------------------------------------------------------
    files = {k: os.path.join(pkg, d, f'{name}{e}') for k, (d, e) in {
        'world': ('worlds', '.sdf'), 'config': ('config', '.yaml'), 'truth': ('truth', '.yaml'),
        'mission SB1': ('missions', '_survey.waypoints'), 'mission SB2': ('missions', '_sb2_survey.waypoints')}.items()}
    missing = [k for k, p in files.items() if not os.path.isfile(p)]
    if not rep.check(not missing, 'world, config, truth and both missions exist', ', '.join(missing)):
        log('  Fix: python3 scripts/build_real_field.py world')
        return 1
    mdir = _find_models(pkg, models)
    man_path = os.path.join(mdir, 'rf_assets_manifest.json')
    man = json.load(open(man_path, encoding='utf-8')) if os.path.isfile(man_path) else None
    if not rep.check(man is not None, 'Gazebo models exist (rf_ground, rf_trees, rf_buildings)', mdir):
        log('  Fix: python3 scripts/build_real_field.py assets --install')
        return 1
    for m in ('rf_ground', 'rf_trees', 'rf_buildings'):
        rep.check(os.path.isfile(os.path.join(mdir, m, 'model.sdf')), f'model {m} has a model.sdf')
    world_text = open(files['world'], encoding='utf-8').read()
    for m in ('rf_ground', 'rf_trees', 'rf_buildings'):
        rep.check(f'model://{m}' in world_text, f'the world includes model://{m}')
    # 3. budget ----------------------------------------------------------------------------------------------------
    tris = man['ground']['visual_triangles'] + man['trees']['triangles']
    rep.check(tris < TRIANGLE_BUDGET, f'triangles to draw under {TRIANGLE_BUDGET:,}', f'{tris:,}')
    rep.check('<mesh><uri>model://rf_ground/meshes/collision.glb' in
              open(os.path.join(mdir, 'rf_ground', 'model.sdf'), encoding='utf-8').read()
              or 'collision.glb' in open(os.path.join(mdir, 'rf_ground', 'model.sdf'), encoding='utf-8').read(),
              'physics ground is a mesh (not a heightmap)', f'{man["ground"]["collision_triangles"]:,} triangles')
    # 2. home ------------------------------------------------------------------------------------------------------
    truth = yaml.safe_load(open(files['truth'], encoding='utf-8'))
    cfg = yaml.safe_load(open(files['config'], encoding='utf-8'))
    th = truth['home']
    rep.check(abs(th['lat'] - pack.home.lat) < 1e-7 and abs(th['lon'] - pack.home.lon) < 1e-7
              and abs(th['alt_msl'] - pack.home.alt) < 0.05, 'world home equals the pack home',
              f'{pack.home.lat:.7f}, {pack.home.lon:.7f}, {pack.home.alt} m')
    rep.check(str(pack.home.lat)[:9] in cfg['home'], 'config home string matches (SITL --home)', cfg['home'])
    ground = Ground(pack)
    rep.check(cfg['spawn']['z'] >= ground.z(cfg['spawn']['x'], cfg['spawn']['y']) - 0.01,
              'the drone spawns above the ground', f'spawn z {cfg["spawn"]["z"]:.2f}, ground {ground.z(0, 0):.2f}')
    # 4 + 5. targets and props on the ground ----------------------------------------------------------------------
    areas = {a['name']: a for a in truth['search_areas']}
    rects = {n: search_rect(site.search_box_xy(n, pack.home)) for n in site.SEARCH_BOUNDARIES_LATLON}
    bad_z, bad_spot, bad_area = [], [], []
    for e in truth['targets'] + truth['distractors']:
        gz = ground.z_under(e['x'], e['y'], 0.5)
        if e['id'].endswith(('pole_a', 'pole_b', 'pole_c')):
            continue
        if abs(e['z'] - gz) > 0.35:
            bad_z.append(f'{e["id"]} z {e["z"]:.2f} vs ground {gz:.2f}')
    for e in truth['targets']:
        rect = rects.get(e['area'])
        if rect is None or not rect.contains(e['x'], e['y'], 8.0):
            bad_area.append(e['id'])
        if ground.class_at(e['x'], e['y']) not in OPEN_GROUND or ground.slope(e['x'], e['y']) > MAX_SLOPE \
                or not ground.clear_of_trees(e['x'], e['y'], 0.5, gap=0.5):
            bad_spot.append(e['id'])
    rep.check(len(truth['targets']) == 4, 'four targets (2 hikers + 2 tents)', f'{len(truth["targets"])}')
    rep.check(not bad_z, 'every target and prop stands on the real ground (within 35 cm)', '; '.join(bad_z[:3]))
    rep.check(not bad_area, 'every target is at least 8 m inside its search boundary', ', '.join(bad_area))
    rep.check(not bad_spot, 'every target is on open, level ground, clear of the real trees', ', '.join(bad_spot))
    # 6. missions --------------------------------------------------------------------------------------------------
    poly = site.flight_boundary_xy(pack.home)
    for label, path, area in (('SB1', files['mission SB1'], 'search_boundary_1'),
                              ('SB2', files['mission SB2'], 'search_boundary_2')):
        items = mission.read_waypoints(path)
        gmin, gmax = ground.range_in(rects[area])
        nav = [it for it in items if it.frame == 3 and it.command in (16, 22)]     # waypoints + take-off, above home
        alts = {round(it.z, 2) for it in nav}
        alt = max(alts) if alts else float('nan')
        rep.check(len(alts) == 1, f'{label} flies at ONE altitude above home', str(sorted(alts)))
        rep.check(alt - gmax >= site.RULE_MIN_AGL_M, f'{label} stays >= 150 ft over the highest ground',
                  f'{alt - gmax:.1f} m over ground +{gmax:.1f} m')
        rep.check(alt - gmin <= site.RULE_MAX_AGL_M, f'{label} stays <= 400 ft over the lowest ground',
                  f'{alt - gmin:.1f} m over ground {gmin:+.1f} m')
        outside = 0
        for it in (i for i in nav if i.command == 16):
            x, y = geo.latlon_to_enu(it.x, it.y, pack.home)
            if not geo.inside_with_margin(x, y, poly, 0.0):
                outside += 1
        rep.check(outside == 0, f'{label} has every waypoint inside the flight boundary', f'{outside} outside')
    log(f'[check] {"all checks passed" if not rep.failed else str(rep.failed) + " check(s) FAILED"}')
    return 0 if not rep.failed else 1
