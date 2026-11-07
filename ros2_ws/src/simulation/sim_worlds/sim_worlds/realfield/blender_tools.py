"""
sim_worlds/realfield/blender_tools.py -- PURPOSE: Blender is a declared dependency of the real-terrain world for INSPECTION: it opens the
very meshes and pictures Gazebo loads (rf_ground, rf_trees, rf_buildings), lights them with the real sun, saves a .blend you can fly around
in, and renders test views (the drone's own nadir view at 50 m, oblique and overview shots) so the terrain can be judged and compared with
Mission Planner screenshots without starting the simulator. Blender is NOT needed to BUILD the world (that is numpy + Pillow) or to fly it.

This module only finds Blender, makes the camera views from the site pack and starts Blender with scripts/blender_real_field.py:

  python3 scripts/build_real_field.py blender --check              # where is Blender, which version
  python3 scripts/build_real_field.py blender --out /tmp/rf_blend  # .blend + 7 PNG renders (about 1-2 minutes on a GPU)

Where it looks for Blender: --blender PATH, then $BLENDER_EXE, then `blender` on the PATH, then (inside WSL) the Windows install
(/mnt/c|d/Program Files/Blender Foundation/Blender <version>/blender.exe, the newest). Tested with Blender 5.2.3. Install on Windows:
  winget install --id BlenderFoundation.Blender -e
"""

import glob
import json
import os
import re
import shutil
import subprocess

from sim_worlds.realfield import site
from sim_worlds.realfield.sitepack import SitePack

TESTED_VERSION = '5.2'
SCRIPT = 'blender_real_field.py'
INSTALL_HINT = ('Install Blender 5.x on Windows:  bash simulation/setup/install_blender.sh   (or PowerShell: winget install --id '
                'BlenderFoundation.Blender -e, or https://www.blender.org/download/), then run this again; or give its path with '
                '--blender / $BLENDER_EXE. (Blender is only for inspecting the world; the world compiles without it.)')


def find_blender(explicit=None):
    """Path of a Blender executable, or None."""
    for cand in (explicit, os.environ.get('BLENDER_EXE')):
        if cand and os.path.isfile(cand):
            return cand
    on_path = shutil.which('blender')
    if on_path:
        return on_path
    pats = []
    for drive in ('c', 'd', 'e'):
        pats += [f'/mnt/{drive}/Program Files/Blender Foundation/Blender */blender.exe',
                 f'/mnt/{drive}/Program Files (x86)/Blender Foundation/Blender */blender.exe']
    found = []
    for pat in pats:
        found += glob.glob(pat)
    if not found:
        return None

    def version_key(p):
        m = re.search(r'Blender (\d+)\.(\d+)', p)
        return (int(m.group(1)), int(m.group(2))) if m else (0, 0)
    return sorted(found, key=version_key)[-1]


def version(exe):
    """Blender's version text (for example '5.2.3'), or '' when it cannot be asked."""
    try:
        out = subprocess.run([exe, '--version'], capture_output=True, text=True, timeout=60).stdout
    except (OSError, subprocess.SubprocessError):
        return ''
    m = re.search(r'Blender\s+(\d+\.\d+(?:\.\d+)?)', out)
    return m.group(1) if m else ''


def report(exe, log=print):
    """Print where Blender is and whether its version is the tested one. Returns 0 (found) or 1."""
    if exe is None:
        log('[blender] NOT FOUND. ' + INSTALL_HINT)
        return 1
    v = version(exe)
    note = '' if v.startswith(TESTED_VERSION) else f'  (tested with {TESTED_VERSION}.x: other versions may need a small change)'
    log(f'[blender] found: {exe}  version {v or "unknown"}{note}')
    return 0


def _path_for(exe, p):
    """The path as Blender wants it: a Windows path (C:/...) when a Windows blender.exe is started from WSL."""
    p = os.path.abspath(p)
    if exe.lower().endswith('.exe') and shutil.which('wslpath'):
        return subprocess.run(['wslpath', '-m', p], capture_output=True, text=True).stdout.strip() or p
    return p


def make_views(path, pack=None):
    """Write views.json: camera placements 50 m above the real ground over each search boundary and the pad (the drone's own view),
    an oblique view, a low view and an overview, the sun of the aerial photo and the camera's 81 degree field of view."""
    pack = pack or SitePack()
    home = pack.home

    def ground(x, y):
        return float(pack.ground_z(x, y))
    sbs = {n: site.search_box_xy(n, home) for n in site.SEARCH_BOUNDARIES_LATLON}
    views = []
    for n, (x0, y0, x1, y1) in sbs.items():
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        views.append({'name': f'{n}_nadir50', 'x': cx, 'y': cy, 'z': ground(cx, cy) + 50, 'yaw_deg': 0, 'pitch_deg': -90})
        views.append({'name': f'{n}_oblique', 'x': cx, 'y': y0 - 60, 'z': ground(cx, y0 - 60) + 25, 'yaw_deg': 0, 'pitch_deg': -25})
    views.append({'name': 'pad_nadir50', 'x': 0, 'y': 0, 'z': ground(0, 0) + 50, 'yaw_deg': 0, 'pitch_deg': -90})
    views.append({'name': 'pad_low_south', 'x': 0, 'y': -40, 'z': ground(0, -40) + 6, 'yaw_deg': 0, 'pitch_deg': -8})
    views.append({'name': 'overview', 'x': 0, 'y': -260, 'z': 220, 'yaw_deg': 0, 'pitch_deg': -35})
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'views': views, 'sun_direction_enu': list(site.SUN_DIRECTION_ENU), 'search_boxes': sbs, 'hfov_deg': 81.0}, f, indent=1)
    return views


#: sections of the terrain to audit in 3D: name -> (x, y, radius in metres, what to look for)
AUDIT_SECTIONS = {
    'pad_strip': (0.0, -35.0, 40.0, 'the VTOL pad on the paved strip: flat, sharp, slab edges, grass verge, nearest trees'),
    'sb1_pasture': (237.0, -250.0, 60.0, 'Search Boundary 1: grass height and texture, poles, trees, slope'),
    'sb2_pasture': (293.0, -12.0, 60.0, 'Search Boundary 2: grass, trees, slope'),
    'wetland': (53.0, -214.0, 45.0, 'the dug wetland basin and the water: depth, shore, vegetation'),
    'sand_pit': (-567.0, 33.0, 70.0, 'the large bare sand / soil area west of the strip: sand look, ruts, edges'),
    'tracks_east': (385.0, -230.0, 50.0, 'dirt tracks and bare patches east of Search Boundary 1'),
    'forest_edge': (150.0, 330.0, 70.0, 'closed forest edge and canopy to the north: tree models, canopy, gaps'),
    'tree_line_sw': (-30.0, -150.0, 55.0, 'single trees and tree lines: crown shapes, cedars, trunks, shadows'),
    'buildings_south': (300.0, -590.0, 70.0, 'the casino buildings and car parks: walls, roofs, edges'),
    'creek_valley': (-250.0, 420.0, 80.0, 'creek valley terrain: slopes, banks, water, forest on the slopes'),
}


def make_audit_views(path, pack=None):
    """Write views.json with 4 views per audited section (see AUDIT_SECTIONS): top, from_south and from_west (30 degrees down, looking at the
    section's centre) and eye (1.7 m above the ground, looking at the centre from the edge). Returns the list of views."""
    import math
    pack = pack or SitePack()

    def ground(x, y):
        return float(pack.ground_z(x, y))
    views = []
    for name, (cx, cy, r, _what) in AUDIT_SECTIONS.items():
        gz = ground(cx, cy)
        views.append({'name': f'{name}__top', 'x': cx, 'y': cy, 'z': gz + 55, 'yaw_deg': 0, 'pitch_deg': -90})
        d, h = r * 1.3, r * 0.75
        for tag, dx, dy, yaw in (('from_south', 0.0, -d, 0.0), ('from_west', -d, 0.0, 90.0)):
            x, y = cx + dx, cy + dy
            views.append({'name': f'{name}__{tag}', 'x': x, 'y': y, 'z': max(ground(x, y), gz) + h, 'yaw_deg': yaw,
                          'pitch_deg': -math.degrees(math.atan2(h, d))})
        x, y = cx, cy - r
        views.append({'name': f'{name}__eye', 'x': x, 'y': y, 'z': ground(x, y) + 1.7, 'yaw_deg': 0, 'pitch_deg': -2})
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'views': views, 'sun_direction_enu': list(site.SUN_DIRECTION_ENU), 'search_boxes': {}, 'hfov_deg': 70.0}, f, indent=1)
    return views


def contact_sheets(out_dir, log=print):
    """Join the four renders of every audited section into one picture <section>__sheet.png (2 x 2: top, south, west, eye)."""
    from PIL import Image
    made = []
    for name in AUDIT_SECTIONS:
        tiles = [os.path.join(out_dir, f'{name}__{t}.png') for t in ('top', 'from_south', 'from_west', 'eye')]
        if not all(os.path.isfile(t) for t in tiles):
            continue
        ims = [Image.open(t).convert('RGB').resize((800, 450)) for t in tiles]
        sheet = Image.new('RGB', (1600, 900))
        for i, im in enumerate(ims):
            sheet.paste(im, ((i % 2) * 800, (i // 2) * 450))
        p = os.path.join(out_dir, f'{name}__sheet.png')
        sheet.save(p)
        made.append(p)
    log(f'[blender] {len(made)} contact sheets (top | from south / from west | eye level) in {out_dir}')
    return made


def run(models_dir, out_dir, exe, only=None, samples=48, no_render=False, log=print, audit=False):
    """Make views.json and start Blender in the background to build suas_field_real.blend and render the views."""
    os.makedirs(out_dir, exist_ok=True)
    views_json = os.path.join(out_dir, 'views.json')
    views = make_audit_views(views_json) if audit else make_views(views_json)
    script = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'scripts', SCRIPT)
    cmd = [exe, '--background', '--python', _path_for(exe, script), '--',
           _path_for(exe, models_dir), _path_for(exe, views_json), _path_for(exe, out_dir), str(samples)]
    if only:
        cmd.append(f'only={only}')
    if no_render:
        cmd.append('--no-render')
    log(f'[blender] {version(exe) or "?"}: {len(views)} views -> {out_dir}')
    r = subprocess.run(cmd, capture_output=True, text=True)
    for line in r.stdout.splitlines():
        if line.startswith(('IMPORTED', 'RENDER', 'SAVED', 'RENDERED')):
            log('  ' + line)
    if r.returncode != 0:
        log(r.stdout[-1500:] + r.stderr[-1500:])
        raise SystemExit(f'[blender] ERROR: Blender exited with {r.returncode}. Fix: read the output above; give models with --models.')
    if audit and not no_render:
        contact_sheets(out_dir, log=log)
    log(f'[blender] done: open {os.path.join(out_dir, "suas_field_real.blend")} in Blender; renders are the PNG files beside it')
    return out_dir
