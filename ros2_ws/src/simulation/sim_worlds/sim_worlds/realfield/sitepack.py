"""
sim_worlds/realfield/sitepack.py -- PURPOSE: read the SITE PACK of the real-terrain world: the compact, committed
folder sim_worlds/data/suas_field_real/ that holds everything stages 3 (assets) and 4 (world) need, made by
stage 2 (pack.py) from the real data. Needs only numpy and Pillow (works on the team's ROS 2 Humble machine).

What is in the pack (all positions in WORLD metres: x east, y north, origin = the Runway 2 pad centre):
  pack.json              where every raster sits, home, statistics, data sources and licences
  height_1m.png          16-bit ground height: metres NAVD88 = offset + value * scale (cm steps)
  colour_far_1m.jpg      ground colour of the whole terrain (NAIP 2023, season graded, trees painted out)
  colour_near_0p25m.jpg  the same at 0.25 m around the search areas and the pad (the "near" zone)
  class_detail_0p25m.png ground type per pixel in the search areas (grass, dry, bare, gravel, ... see CLASSES)
  canopy_1m.png          closed-forest canopy height above the ground (value * 0.125 m), 0 = no forest
  trees.csv              every single tree: position, height, crown, kind
  features.json          buildings, water, streams, roads, runway, pipeline (OpenStreetMap + Microsoft)
  detail/*.png           CC0 ground-detail pictures (Poly Haven), already turned into "brightness detail"

Typical use:
  pk = SitePack()                      # default folder: the package's data/suas_field_real
  z = pk.ground_z(120.0, -250.0)       # ground height at a world point, metres above the pad (z = 0)
  rgb = pk.raster('colour_near').sample(x, y)
"""

import csv
import json
import os

import numpy as np

from sim_worlds import geo
from sim_worlds.realfield import grids

#: ground classes of class_detail_0p25m.png (value -> name). Stage 3 picks the CC0 detail picture by class.
CLASSES = {0: 'grass', 1: 'dry_grass', 2: 'bare_soil', 3: 'gravel', 4: 'asphalt', 5: 'water', 6: 'canopy',
           7: 'under_tree', 8: 'wetland'}


def default_pack_dir():
    """<package>/data/suas_field_real in the source tree, else the installed share folder."""
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    cand = os.path.join(here, 'data', 'suas_field_real')
    if os.path.isfile(os.path.join(cand, 'pack.json')):
        return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        cand2 = os.path.join(get_package_share_directory('sim_worlds'), 'data', 'suas_field_real')
        if os.path.isfile(os.path.join(cand2, 'pack.json')):
            return cand2
    except Exception:                                                  # noqa: BLE001 (not a ROS machine)
        pass
    return cand


def _load_image(path):
    """A PNG/JPG as a numpy array (uint8 RGB/grey, or uint16 for 16-bit PNGs)."""
    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None                    # our own files: big on purpose
    with Image.open(path) as im:
        if im.mode in ('I;16', 'I;16B', 'I;16L', 'I'):
            return np.array(im, dtype=np.uint16) if im.mode != 'I' else np.array(im).astype(np.uint16)
        return np.array(im)


class SitePack:
    """The site pack in `folder` (default: the package's data/suas_field_real)."""

    def __init__(self, folder=None):
        self.folder = folder or default_pack_dir()
        path = os.path.join(self.folder, 'pack.json')
        if not os.path.isfile(path):
            raise FileNotFoundError(f'{path} not found: the site pack is missing (see docs/simulation/WORLDS.md)')
        with open(path, encoding='utf-8') as f:
            self.meta = json.load(f)
        h = self.meta['home']
        self.home = geo.Home(h['lat'], h['lon'], h['alt_msl'], h.get('heading', 0.0))
        self._cache = {}

    def path(self, rel):
        """Absolute path of a file inside the pack."""
        return os.path.join(self.folder, rel)

    def raster(self, name):
        """A pack raster as grids.Raster with decoded values: 'height' (metres NAVD88, float32), 'colour_far' /
        'colour_near' (uint8 RGB), 'class' (uint8 codes, see CLASSES), 'canopy' (metres above ground, float32)."""
        if name not in self._cache:
            e = self.meta['rasters'][name]
            a = _load_image(self.path(e['file']))
            if 'scale' in e:
                a = (a.astype(np.float32) * float(e['scale']) + float(e.get('offset', 0.0))).astype(np.float32)
            self._cache[name] = grids.Raster(a, e['x0'], e['y1'], e['res'], e.get('what', ''))
        return self._cache[name]

    def ground_msl(self, x, y):
        """Ground height (metres NAVD88) at world point(s) (x, y): bilinear in the 1 m height raster, the same
        surface the terrain meshes are made from (they follow it to within a few cm)."""
        return self.raster('height').sample(x, y)

    def ground_z(self, x, y):
        """Ground height at world point(s) as world z (metres above the pad = home altitude)."""
        return self.ground_msl(x, y) - self.home.alt

    def trees(self):
        """List of tree dicts from trees.csv (x, y, height_m, crown_diam_m, kind, source, forest, ...)."""
        if 'trees' not in self._cache:
            out = []
            with open(self.path(self.meta['files']['trees']), encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    for k in ('x', 'y', 'ground_navd88', 'height_m', 'crown_diam_m'):
                        row[k] = float(row[k])
                    row['forest'] = int(row['forest'])
                    out.append(row)
            self._cache['trees'] = out
        return self._cache['trees']

    def features(self):
        """features.json as a dict (buildings, water, streams, runways, taxiways, roads, pipelines)."""
        if 'features' not in self._cache:
            with open(self.path(self.meta['files']['features']), encoding='utf-8') as f:
                self._cache['features'] = json.load(f)
        return self._cache['features']

    def detail_textures(self):
        """List of the CC0 detail pictures: dicts {name, file, real_size_m, use, ...}."""
        return self.meta.get('detail_textures', [])

    def detail_image(self, name):
        """A detail picture as float32 array (1.0 = unchanged brightness), shape [px, px]."""
        key = 'detail:' + name
        if key not in self._cache:
            e = next(d for d in self.detail_textures() if d['name'] == name)
            a = _load_image(self.path(e['file'])).astype(np.float32)
            self._cache[key] = 1.0 + (a - 128.0) / 128.0 * float(e.get('amplitude', 0.5))
        return self._cache[key]
