"""
sim_worlds/realfield/prepare.py -- PURPOSE: generator stage 1 ("prepare"): turn the raw downloads of the real
SUAS 2026 site into rasters and feature lists on the WORLD grid (origin = the Runway 2 pad, x east, y north),
saved as plain .npy/.json/.csv files that stage 2 reads with numpy only (realfield/grids.py).

Needs a GIS Python (NOT the ROS one): rasterio, pyproj, laspy with the lazrs backend, numpy, scipy, Pillow.
How to get one is in docs/simulation/WORLDS.md ("Regenerate the real field"). Inputs come from
fetch_real_field_data.sh (folder layout and checksums in sim_worlds/data/real_field_manifest.json).

What it makes (prepared/ folder):
  dtm      0.5 m  bare-earth ground height, NAVD88 m        USGS 3DEP lidar DTM (OPR tiles, 2017 flight)
  chm      0.5 m  canopy height above ground, m              USGS 3DEP point cloud (first returns - DTM)
  naip     0.25 m R, G, B, NIR photo (uint8)                 USDA NAIP 2023-09-18, 0.3 m
  leafoff  0.5 m  1 = "dark olive in March" = evergreen       OKMaps 2025 leaf-off photo (optional; only a mask)
  metachm  1 m    canopy height (2019 imagery model)          Meta / WRI canopy height v2 (optional fallback)
  features.json   buildings, water, streams, runway, roads    OpenStreetMap (ODbL) + Microsoft footprints (ODbL)
  trees.csv       one row per tree (see trees_detect.py)
  grid.json       where every raster sits, home height, season grade, sources

Why reproject everything onto ONE world grid here: the sources come in three different map projections
(NAIP: UTM zone 14; the lidar: UTM zone 15, although the field is just WEST of the zone 14/15 line at
96 deg W; the photo-based layers: Web Mercator). Doing it once, carefully, with PROJ means stage 2 never has
to think about projections again.
"""

import csv
import glob
import json
import math
import os
import time

import numpy as np

from sim_worlds.realfield import grids, site

# Imported lazily inside the functions that need them, so `import prepare` works for --help without GIS libs:
#   rasterio, rasterio.warp, pyproj, laspy, scipy.ndimage

DTM_RES = 0.5
CHM_RES = 0.5
NAIP_RES = 0.25
LEAFOFF_RES = 0.5
META_RES = 1.0
GRID_MARGIN_M = 8.0          #: rasters extend this far past the terrain EXTENT (normals and skirts at the edge)
DEFAULT_GRASS_GRADE = (0.85, 0.93, 0.85)   #: used when no Sentinel-2 pair is available (see texture.py)


def data_dirs(data_root=None):
    """(root, raw, prepared) folders of the real field's data. Default root: $ASTRA_WORLD_DATA or
    ~/.astra_sim/world_data, then /suas_field_real."""
    base = data_root or os.environ.get('ASTRA_WORLD_DATA') or os.path.join(os.path.expanduser('~'), '.astra_sim',
                                                                           'world_data')
    root = os.path.join(base, site.WORLD) if os.path.basename(os.path.normpath(base)) != site.WORLD else base
    return root, os.path.join(root, 'raw'), os.path.join(root, 'prepared')


def manifest_path():
    """sim_worlds/data/real_field_manifest.json (source tree or installed share folder)."""
    here = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    for cand in (os.path.join(here, 'data', 'real_field_manifest.json'),):
        if os.path.isfile(cand):
            return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('sim_worlds'), 'data', 'real_field_manifest.json')
    except Exception as exc:                                          # noqa: BLE001
        raise FileNotFoundError('real_field_manifest.json not found (source tree or installed sim_worlds)') from exc


def load_manifest():
    """The data manifest as a dict (sources with URLs, checksums, licences)."""
    with open(manifest_path(), encoding='utf-8') as f:
        return json.load(f)


def raw_files(raw, manifest, layer):
    """Existing raw files of one manifest layer ('dtm', 'lidar', 'naip', ...), in manifest order."""
    out = []
    for s in manifest['sources']:
        if s['layer'] == layer:
            p = os.path.join(raw, s['file'])
            if os.path.isfile(p):
                out.append(p)
    return out


def world_grid(res, margin=GRID_MARGIN_M):
    """(x0, y1, cols, rows) of a north-up world raster at `res` covering the terrain EXTENT + margin."""
    x_min, y_min, x_max, y_max = site.EXTENT
    x0, y1 = x_min - margin, y_max + margin
    cols = int(round((x_max - x_min + 2 * margin) / res))
    rows = int(round((y_max - y_min + 2 * margin) / res))
    return x0, y1, cols, rows


def _reproject(sources, bands, res, resampling, dtype, src_nodata=None, fill=np.nan, margin=GRID_MARGIN_M):
    """Reproject one or more GeoTIFFs onto the world grid. Returns an array [rows, cols] (one band) or
    [rows, cols, len(bands)] of `dtype`. Later sources only fill cells the earlier ones left empty.

    Memory: one float32 band at a time is reprojected and converted straight into the output type, so the
    0.25 m NAIP photo (7520 x 7520 x 4) needs about 0.5 GB instead of 1.5 GB (other jobs share this laptop)."""
    import rasterio
    from rasterio.transform import from_origin
    from rasterio.warp import Resampling, reproject

    x0, y1, cols, rows = world_grid(res, margin)
    dst_crs = rasterio.crs.CRS.from_proj4(site.local_proj4())
    transform = from_origin(x0, y1, res, res)
    integer = np.issubdtype(np.dtype(dtype), np.integer)
    out = np.zeros((rows, cols, len(bands)), dtype=dtype)
    rs = getattr(Resampling, resampling)
    for k, b in enumerate(bands):
        acc = np.full((rows, cols), np.nan, dtype=np.float32)
        for path in sources:
            with rasterio.open(path) as src:
                tmp = np.full((rows, cols), np.nan, dtype=np.float32)
                reproject(rasterio.band(src, b), tmp, src_nodata=src_nodata if src_nodata is not None else src.nodata,
                          dst_transform=transform, dst_crs=dst_crs, dst_nodata=np.nan, resampling=rs,
                          num_threads=4)
                m = np.isnan(acc) & ~np.isnan(tmp)
                acc[m] = tmp[m]
                del tmp
        if integer:
            acc = np.where(np.isnan(acc), fill, acc)
            out[..., k] = np.clip(np.round(acc), np.iinfo(dtype).min, np.iinfo(dtype).max).astype(dtype)
        else:
            out[..., k] = acc.astype(dtype)
        del acc
    return (out[..., 0] if len(bands) == 1 else out), (x0, y1)


def _fill_nearest(a):
    """Fill NaN cells of a 2-D float array with the value of the nearest valid cell (edge extension)."""
    from scipy import ndimage
    bad = np.isnan(a)
    if not bad.any():
        return a, 0.0
    idx = ndimage.distance_transform_edt(bad, return_distances=False, return_indices=True)
    return a[tuple(idx)], float(bad.mean())


# ------------------------------------------------------------------------------------------------------------
# layers
# ------------------------------------------------------------------------------------------------------------

def _feather_box(X, Y, box, feather):
    """Weight 1 inside the box, falling smoothly (cosine) to 0 at `feather` metres outside it."""
    x0, y0, x1, y1 = box
    dx = np.maximum(np.maximum(x0 - X, X - x1), 0.0)
    dy = np.maximum(np.maximum(y0 - Y, Y - y1), 0.0)
    d = np.hypot(dx, dy)
    return np.where(d >= feather, 0.0, 0.5 * (1.0 + np.cos(np.pi * np.clip(d / feather, 0.0, 1.0))))


def apply_dtm_edits(dtm, x0, y1, res, meta, log):
    """Patch the 2017 ground where it is out of date (site.DTM_EDITS): level the vanished soil piles and dig
    the new wetland. Records what changed in meta['dtm_edits'] (these edits are ESTIMATES; see WORLDS.md)."""
    from scipy import ndimage
    edits = []
    for e in site.DTM_EDITS:
        if e['kind'] == 'level':
            bx0, by0, bx1, by1 = e['box']
            m = e['feather_m'] + e['window_m']               # work window: box + feather + one structuring size
            r = grids.Raster(dtm, x0, y1, res)
            r0, r1, c0, c1 = r.window(bx0 - m, by0 - m, bx1 + m, by1 + m)
            sub = dtm[r0:r1, c0:c1].astype(np.float64)
            k = int(round(e['window_m'] / res)) | 1
            opened = ndimage.grey_opening(sub, size=(k, k))
            xs = x0 + (np.arange(c0, c1) + 0.5) * res
            ys = y1 - (np.arange(r0, r1) + 0.5) * res
            X, Y = np.meshgrid(xs, ys)
            w = _feather_box(X, Y, e['box'], e['feather_m'])
            new = sub - w * (sub - np.minimum(sub, opened))
            drop = sub - new
            dtm[r0:r1, c0:c1] = new.astype(dtm.dtype)
            info = {'name': e['name'], 'max_lowered_m': round(float(drop.max()), 2),
                    'volume_removed_m3': round(float(drop.sum() * res * res)), 'why': e['why']}
        else:
            cx, cy = e['center']
            ax, ay = e['radii']
            r = grids.Raster(dtm, x0, y1, res)
            r0, r1, c0, c1 = r.window(cx - ax, cy - ay, cx + ax, cy + ay)
            xs = x0 + (np.arange(c0, c1) + 0.5) * res
            ys = y1 - (np.arange(r0, r1) + 0.5) * res
            X, Y = np.meshgrid(xs, ys)
            q = np.hypot((X - cx) / ax, (Y - cy) / ay)
            bowl = e['depth_m'] * np.where(q < 1.0, 0.5 * (1.0 + np.cos(np.pi * q)), 0.0)
            dtm[r0:r1, c0:c1] = (dtm[r0:r1, c0:c1] - bowl).astype(dtm.dtype)
            info = {'name': e['name'], 'max_lowered_m': round(float(bowl.max()), 2),
                    'volume_removed_m3': round(float(bowl.sum() * res * res)), 'why': e['why']}
        edits.append(info)
        log(f'    DTM edit {info["name"]}: lowered up to {info["max_lowered_m"]} m, '
            f'{info["volume_removed_m3"]:,} m3 removed')
    meta['dtm_edits'] = edits
    return dtm


def prepare_dtm(raw, manifest, out, meta, log):
    """USGS 3DEP OPR DTM tiles (0.5 m, EPSG:6344 = NAD83(2011) UTM 15N, NAVD88 m) -> world grid 0.5 m.
    Also sets meta['home'] = DTM median over the pad square (the SITL home height)."""
    files = raw_files(raw, manifest, 'dtm')
    if not files:
        raise FileNotFoundError('no DTM tiles in raw/dtm: run fetch_real_field_data.sh first')
    log(f'  DTM: {len(files)} tiles -> {DTM_RES} m world grid (bilinear)')
    dtm, (x0, y1) = _reproject(files, [1], DTM_RES, 'bilinear', np.float32, src_nodata=-999999.0)
    dtm, filled = _fill_nearest(dtm)
    log(f'    filled {100 * filled:.2f} % of the cells from the nearest valid one (outside the lidar coverage)')
    dtm = apply_dtm_edits(dtm, x0, y1, DTM_RES, meta, log)
    r = grids.Raster(dtm, x0, y1, DTM_RES)
    s = site.PAD_SIZE_M / 2.0
    xs = np.arange(-s, s + 1e-6, 0.5)
    pad = r.resample_grid(xs, xs[::-1])
    home_alt = float(np.median(pad))
    meta['home'] = {'lat': site.PAD_LATLON[0], 'lon': site.PAD_LATLON[1], 'alt_msl': round(home_alt, 2),
                    'heading': 0.0, 'how': 'median NAVD88 height of the 3DEP DTM over the 12.2 m pad square',
                    'pad_relief_m': round(float(pad.max() - pad.min()), 3)}
    log(f'    home (pad) height {home_alt:.2f} m NAVD88 ({home_alt / 0.3048:.1f} ft), pad relief '
        f'{pad.max() - pad.min():.2f} m; DTM range {dtm.min():.1f} .. {dtm.max():.1f} m')
    grids.save_raster(out, meta, 'dtm', dtm, x0, y1, DTM_RES, 'bare-earth height NAVD88 m (USGS 3DEP OPR DTM, 2017)')
    return r


def prepare_chm(raw, manifest, out, meta, dtm, log):
    """USGS 3DEP point cloud (LAZ, EPSG:6344 + NAVD88) -> highest return per 0.5 m cell -> canopy height
    model = DSM - DTM, cleaned of bird/noise spikes."""
    import laspy
    from pyproj import Transformer
    from scipy import ndimage

    files = raw_files(raw, manifest, 'lidar')
    if not files:
        raise FileNotFoundError('no lidar LAZ tiles in raw/lidar: run fetch_real_field_data.sh first')
    x0, y1, cols, rows = world_grid(CHM_RES)
    dsm = np.full(rows * cols, -np.inf, dtype=np.float32)
    tr = Transformer.from_crs('EPSG:6344', site.local_proj4(), always_xy=True)
    total = used = 0
    for path in files:
        t0 = time.time()
        with laspy.open(path) as f:
            for pts in f.chunk_iterator(3_000_000):
                cls = np.asarray(pts.classification)
                keep = (cls != 7) & (cls != 18)                    # ASPRS 7 = low noise, 18 = high noise
                try:
                    keep &= ~np.asarray(pts.withheld, dtype=bool)
                except Exception:                                    # noqa: BLE001 (older point formats)
                    pass
                x, y = tr.transform(np.asarray(pts.x)[keep], np.asarray(pts.y)[keep])
                z = np.asarray(pts.z)[keep].astype(np.float32)
                c = np.floor((x - x0) / CHM_RES).astype(np.int64)
                r = np.floor((y1 - y) / CHM_RES).astype(np.int64)
                ok = (c >= 0) & (c < cols) & (r >= 0) & (r < rows)
                np.maximum.at(dsm, r[ok] * cols + c[ok], z[ok])
                total += len(cls)
                used += int(ok.sum())
        log(f'    {os.path.basename(path)}: {time.time() - t0:.0f} s')
    dsm = dsm.reshape(rows, cols)
    empty = ~np.isfinite(dsm)
    ground = dtm.sample(*_centres(x0, y1, cols, rows, CHM_RES))
    chm = np.where(empty, 0.0, dsm - ground).astype(np.float32)
    chm = np.clip(chm, 0.0, 45.0)                                   # tallest real tree here is about 28 m
    med = ndimage.median_filter(chm, size=3)
    spikes = chm > med + 6.0                                        # single-cell birds / wires / noise
    chm[spikes] = med[spikes]
    log(f'  CHM: {used:,} of {total:,} lidar points on the grid, {100 * empty.mean():.1f} % empty cells, '
        f'{int(spikes.sum()):,} spikes removed, {100 * (chm > 2).mean():.1f} % of cells above 2 m')
    grids.save_raster(out, meta, 'chm', chm, x0, y1, CHM_RES,
                      'canopy/structure height above the DTM, m (USGS 3DEP point cloud, flown 2017-02-10, leaf-off)')
    return grids.Raster(chm, x0, y1, CHM_RES)


def _centres(x0, y1, cols, rows, res):
    """World x, y of every pixel centre of a grid (2-D arrays)."""
    xs = x0 + (np.arange(cols) + 0.5) * res
    ys = y1 - (np.arange(rows) + 0.5) * res
    return np.meshgrid(xs, ys)


def prepare_naip(raw, manifest, out, meta, log, imagery_override=None):
    """NAIP 2023 (0.3 m, R G B NIR, EPSG:26914 = NAD83 UTM 14N) -> world grid 0.25 m (cubic)."""
    files = raw_files(raw, manifest, 'naip')
    if not files:
        raise FileNotFoundError('no NAIP photo in raw/naip: run fetch_real_field_data.sh first')
    log(f'  NAIP: {os.path.basename(files[0])} -> {NAIP_RES} m world grid (cubic)')
    naip, (x0, y1) = _reproject(files[:1], [1, 2, 3, 4], NAIP_RES, 'cubic', np.uint8, fill=0)
    missing = float((naip[..., :3].max(axis=-1) == 0).mean())
    log(f'    {100 * missing:.2f} % of the cells have no photo (filled later from the nearest photo pixel)')
    grids.save_raster(out, meta, 'naip', naip, x0, y1, NAIP_RES, 'USDA NAIP 2023-09-18 R,G,B,NIR uint8 (public domain)')
    return grids.Raster(naip, x0, y1, NAIP_RES)


def prepare_leafoff(raw, manifest, out, meta, log):
    """Optional: OKMaps 2025 leaf-off photo (0.15 m) -> 0.5 m mean colour -> 'evergreen-looking' mask.
    Only this yes/no mask is kept (eastern red-cedars are dark olive in March while oaks are bare); the photo
    itself is not stored or used as a texture (its licence for redistribution is not written down)."""
    files = raw_files(raw, manifest, 'leafoff')
    if not files:
        log('  leaf-off photo: not downloaded (optional) -> tree kinds come from crown shape instead')
        return None
    rgb, (x0, y1) = _reproject(files, [1, 2, 3], LEAFOFF_RES, 'average', np.float32)
    R, G, B = rgb[..., 0].astype(np.float32), rgb[..., 1].astype(np.float32), rgb[..., 2].astype(np.float32)
    br = (R + G + B) / 3.0
    # rule calibrated on THIS photo (2026-10-17, inside 2023 tree crowns of both search boundaries): crowns that
    # are dark in March (brightness median 53 of 255) are much less red (G - R median -7 to -9, 95 % above -14)
    # than bare deciduous crowns and dormant grass (brightness about 92 and 110, G - R about -15 and -16)
    ever = (br >= 18) & (br < 62) & (G - R > -14)
    log(f'  leaf-off evergreen mask: {100 * ever.mean():.1f} % of the cells look evergreen in March 2025')
    grids.save_raster(out, meta, 'leafoff', ever.astype(np.uint8), x0, y1, LEAFOFF_RES,
                      '1 = evergreen-looking in the 2025-03-05 leaf-off photo (OKMaps), mask only')
    return grids.Raster(ever.astype(np.uint8), x0, y1, LEAFOFF_RES)


def prepare_metachm(raw, manifest, out, meta, log):
    """Optional fallback: Meta/WRI global canopy height v2 (1.2 m, imagery of 2019) -> world grid 1 m."""
    files = raw_files(raw, manifest, 'metachm')
    if not files:
        log('  Meta canopy height: not downloaded (optional) -> new trees get heights from their crown size')
        return None
    a, (x0, y1) = _reproject(files, [1], META_RES, 'bilinear', np.float32)
    a = np.nan_to_num(a, nan=0.0)
    log(f'  Meta canopy height v2: {100 * (a > 2).mean():.1f} % of the cells above 2 m')
    grids.save_raster(out, meta, 'metachm', a, x0, y1, META_RES,
                      'canopy height m, Meta/WRI High Resolution Canopy Height Maps v2 (CC-BY 4.0), imagery 2019')
    return grids.Raster(a, x0, y1, META_RES)


def prepare_season_grade(raw, manifest, meta, naip, log):
    """Optional: ratio of Sentinel-2 grass colour on competition day (2026-09-15) to the NAIP week (2023-09-23)
    -> per-band multipliers for grass pixels (texture.py). Falls back to DEFAULT_GRASS_GRADE."""
    files = {os.path.basename(p): p for p in raw_files(raw, manifest, 's2')}
    old = next((p for n, p in files.items() if '2023' in n), None)
    new = next((p for n, p in files.items() if '2026' in n), None)
    grade = {'factors_rgb': list(DEFAULT_GRASS_GRADE), 'how': 'default (no Sentinel-2 pair downloaded)'}
    if old and new:
        import rasterio
        with rasterio.open(old) as a, rasterio.open(new) as b:
            A = a.read([1, 2, 3, 4]).astype(np.float64)
            Bv = b.read([1, 2, 3, 4]).astype(np.float64)
        ndvi_a = (A[3] - A[0]) / (A[3] + A[0] + 1e-6)
        ndvi_b = (Bv[3] - Bv[0]) / (Bv[3] + Bv[0] + 1e-6)
        # grass-like 10 m cells: green but not forest in both years (NDVI 0.30..0.62, NIR not forest-dark)
        m = (ndvi_a > 0.30) & (ndvi_a < 0.62) & (ndvi_b > 0.30) & (ndvi_b < 0.75)
        if m.sum() > 50:
            f = [float(np.median(Bv[k][m] / np.maximum(A[k][m], 1.0))) for k in range(3)]
            f = [round(min(max(v, 0.6), 1.3), 3) for v in f]
            grade = {'factors_rgb': f, 'cells': int(m.sum()),
                     'how': f'median Sentinel-2 reflectance ratio {os.path.basename(new)} / {os.path.basename(old)} '
                            'over grass-like 10 m cells (R, G, B)'}
    meta['season_grade'] = grade
    log(f'  season grade for grass (R, G, B): {grade["factors_rgb"]} ({grade["how"]})')


# ------------------------------------------------------------------------------------------------------------
# vector features: OpenStreetMap + Microsoft building footprints
# ------------------------------------------------------------------------------------------------------------

def _ring_area(pts):
    """Signed area of a polygon ring [(x, y), ...] (positive = counter-clockwise)."""
    a = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        a += x0 * y1 - x1 * y0
    return a / 2.0


def prepare_features(raw, manifest, out, meta, home, log):
    """OSM (Overpass JSON) + Microsoft footprints (GeoJSON) -> features.json in world metres."""
    feats = {'buildings': [], 'water': [], 'streams': [], 'runways': [], 'taxiways': [], 'roads': [],
             'pipelines': [], 'note': 'world metres (x east, y north from the Runway 2 pad), polygons counter-clockwise'}
    osm = raw_files(raw, manifest, 'osm')
    if osm:
        with open(osm[0], encoding='utf-8') as f:
            d = json.load(f)
        nodes = {e['id']: (e['lat'], e['lon']) for e in d['elements'] if e['type'] == 'node'}
        for e in d['elements']:
            if e['type'] != 'way':
                continue
            t = e.get('tags', {})
            pts = [site.world_xy(*nodes[n], home) for n in e.get('nodes', []) if n in nodes]
            if len(pts) < 2:
                continue
            closed = e.get('nodes', [None])[0] == e.get('nodes', [None])[-1] and len(pts) >= 4
            poly = [(round(x, 2), round(y, 2)) for x, y in pts[:-1]] if closed else None
            line = [(round(x, 2), round(y, 2)) for x, y in pts]
            if 'building' in t and poly:
                if _ring_area(poly) < 0:
                    poly = poly[::-1]
                feats['buildings'].append({'source': 'osm', 'id': e['id'], 'kind': t['building'],
                                           'name': t.get('name', ''), 'polygon': poly,
                                           'levels': t.get('building:levels'), 'height': t.get('height')})
            elif poly and (t.get('natural') == 'water' or t.get('landuse') in ('basin', 'reservoir', 'pond')
                           or t.get('water')):
                feats['water'].append({'source': 'osm', 'id': e['id'], 'polygon': poly, 'kind': t.get('landuse') or
                                       t.get('water') or 'water'})
            elif t.get('waterway') in ('stream', 'river', 'ditch', 'drain', 'canal'):
                feats['streams'].append({'source': 'osm', 'id': e['id'], 'line': line, 'kind': t['waterway'],
                                         'name': t.get('name', '')})
            elif t.get('aeroway') == 'runway':
                feats['runways'].append({'source': 'osm', 'id': e['id'], 'line': line, 'ref': t.get('ref', '')})
            elif t.get('aeroway') in ('taxiway', 'apron'):
                feats['taxiways'].append({'source': 'osm', 'id': e['id'], 'line': line, 'kind': t['aeroway']})
            elif t.get('man_made') == 'pipeline':
                feats['pipelines'].append({'source': 'osm', 'id': e['id'], 'line': line,
                                           'location': t.get('location', '')})
            elif 'highway' in t:
                feats['roads'].append({'source': 'osm', 'id': e['id'], 'line': line, 'kind': t['highway'],
                                       'name': t.get('name', ''), 'surface': t.get('surface', '')})
    ms = raw_files(raw, manifest, 'buildings')
    n_ms = 0
    if ms:
        with open(ms[0], encoding='utf-8') as f:
            g = json.load(f)
        osm_polys = [b['polygon'] for b in feats['buildings']]
        for k, ft in enumerate(g.get('features', [])):
            geom = ft.get('geometry') or {}
            rings = geom.get('coordinates', [])
            if geom.get('type') == 'MultiPolygon':
                rings = rings[0] if rings else []
            if not rings:
                continue
            poly = [site.world_xy(lat, lon, home) for lon, lat in rings[0][:-1]]
            poly = [(round(x, 2), round(y, 2)) for x, y in poly]
            if len(poly) < 3:
                continue
            if _ring_area(poly) < 0:
                poly = poly[::-1]
            cx = sum(p[0] for p in poly) / len(poly)
            cy = sum(p[1] for p in poly) / len(poly)
            if any(_inside(cx, cy, op) for op in osm_polys):
                continue                                           # OSM already has it (OSM wins: it has names)
            h = (ft.get('properties') or {}).get('height', -1)
            feats['buildings'].append({'source': 'microsoft', 'id': f'ms{k}', 'kind': 'yes', 'name': '',
                                       'polygon': poly, 'levels': None,
                                       'height': None if h is None or float(h) <= 0 else float(h)})
            n_ms += 1
    path = os.path.join(out, 'features.json')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(feats, f, indent=0)
    meta.setdefault('files', {})['features'] = 'features.json'
    log(f'  features: {len(feats["buildings"])} buildings ({n_ms} from Microsoft footprints), {len(feats["water"])} '
        f'water areas, {len(feats["streams"])} streams, {len(feats["roads"])} roads, {len(feats["runways"])} runway')
    return feats


def _inside(x, y, poly):
    """Point-in-polygon (even-odd)."""
    c = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            c = not c
        j = i
    return c


# ------------------------------------------------------------------------------------------------------------
# driver
# ------------------------------------------------------------------------------------------------------------

def run(data_root=None, log=print, skip_lidar=False):
    """Run the whole prepare stage. Returns the prepared folder."""
    from sim_worlds.realfield import trees_detect
    root, raw, out = data_dirs(data_root)
    os.makedirs(out, exist_ok=True)
    manifest = load_manifest()
    t0 = time.time()
    meta = {'world': site.WORLD, 'frame': site.local_proj4(), 'extent': list(site.EXTENT),
            'generated_by': 'sim_worlds/realfield/prepare.py', 'raw_folder': raw}
    old = os.path.join(out, 'grid.json')
    if os.path.isfile(old) and skip_lidar:
        with open(old, encoding='utf-8') as f:
            meta = json.load(f)
    log(f'prepare: raw data {raw} -> {out}')
    dtm = prepare_dtm(raw, manifest, out, meta, log)
    home = site.home_at(meta['home']['alt_msl'])
    if skip_lidar and 'chm' in meta.get('rasters', {}):
        chm = grids.Prepared(out).raster('chm', mmap=False)
        log('  CHM: reused (--skip-lidar)')
    else:
        chm = prepare_chm(raw, manifest, out, meta, dtm, log)
    naip = prepare_naip(raw, manifest, out, meta, log)
    leafoff = prepare_leafoff(raw, manifest, out, meta, log)
    metachm = prepare_metachm(raw, manifest, out, meta, log)
    prepare_season_grade(raw, manifest, meta, naip, log)
    prepare_features(raw, manifest, out, meta, home, log)
    trees_detect.run(out, meta, dtm, chm, naip, leafoff, metachm, home, log)
    meta['sources'] = [{k: s.get(k) for k in ('id', 'layer', 'file', 'licence', 'attribution', 'sha256', 'pixel_sha256')}
                       for s in manifest['sources'] if os.path.isfile(os.path.join(raw, s['file']))]
    meta['prepare_seconds'] = round(time.time() - t0, 1)
    grids.write_meta(out, meta)
    log(f'prepare: done in {time.time() - t0:.0f} s')
    return out
