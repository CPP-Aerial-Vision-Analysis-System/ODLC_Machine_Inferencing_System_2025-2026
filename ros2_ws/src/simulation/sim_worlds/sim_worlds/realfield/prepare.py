"""
sim_worlds/realfield/prepare.py -- PURPOSE: generator stage 1 ("prepare"): turn the raw downloads of the real
SUAS 2026 site into rasters and feature lists on the WORLD grid (origin = the Runway 2 pad, x east, y north),
saved as plain .npy/.json/.csv files that stage 2 reads with numpy only (realfield/grids.py).

Needs a GIS Python (NOT the ROS one): rasterio, pyproj, laspy with the lazrs backend, numpy, scipy, Pillow.
How to get one is in docs/simulation/WORLDS.md ("Regenerate the real field"). Inputs come from
fetch_real_field_data.sh (folder layout and checksums in sim_worlds/data/real_field_manifest.json).

What it makes (prepared/ folder):
  dtm      0.5 m  bare-earth ground height, NAVD88 m        USGS 3DEP lidar DTM (OPR tiles, 2017 flight)
  chm      0.5 m  canopy height above ground, m              USGS 3DEP point cloud (highest return - DTM), with
                                                             the floating "haze" points removed (see prepare_chm)
  naip     0.25 m R, G, B, NIR photo (uint8)                 USDA NAIP 2023-09-18, 0.3 m
  leafoff  0.5 m  1 = "dark olive in March" = evergreen       OKMaps 2025 leaf-off photo (optional; only a mask)
  metachm  1 m    canopy height (2019 imagery model)          Meta / WRI canopy height v2 (optional fallback)
  features.json   buildings, water, streams, runway, roads    OpenStreetMap (ODbL 1.0) + Microsoft Global ML
                                                             Building Footprints (CDLA Permissive 2.0); building
                                                             heights measured in the lidar
  trees.csv       one row per tree (see trees_detect.py)
  grid.json       where every raster sits, home height, season grade, sources

Research fixes applied here (2026-10-24, from the terrain_sources research; numbers in WORLDS.md):
  * lidar haze: points more than HAZE_ABOVE_DTM_M above the bare earth are dropped before the canopy model
    (they made fake 45 m "trees");
  * Sentinel-2 L2A pixels carry a +1000 offset (processing baseline 04.00 and later): it is removed before NDVI
    and band ratios, and the grass colour grade is measured on PURE grass cells of the NAIP photo;
  * building heights come from the lidar (median canopy height inside each footprint).

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
#: Lidar points higher than this above the bare earth are "haze" (unclassified returns 40-140 m up along the
#: flight-line edges, terrain_sources research 6.1), not trees: the tallest real tree here is 24.6-29.4 m.
HAZE_ABOVE_DTM_M = 35.0
CHM_CLIP_M = 40.0            #: safety clip of the canopy model (nothing real here is taller than about 30 m)
S2_RES = 10.0                #: Sentinel-2 grid used for the season grade (world grid, nearest neighbour)
S2_DEFAULT_OFFSET = -1000    #: BOA_ADD_OFFSET of Sentinel-2 L2A since processing baseline 04.00 (2022-01-25)
PURE_GRASS_SHARE = 0.9       #: a 10 m cell is "pure grass" when this share of its NAIP pixels looks like grass
BUILDING_MIN_LIDAR_H = 2.0   #: a footprint whose lidar median is lower than this was built after 2017 (no height)


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
    model = DSM - DTM, cleaned of floating haze and of bird/noise spikes.

    Haze (research fix 1): the point cloud has unclassified returns floating 40-140 m above the ground in narrow
    north-south strips along the flight-line edges. The old code only clipped the canopy model at 45 m, so those
    cells stayed at 45 m and became 196 fake 45 m "tree tops" in the research area. Here every point higher than
    HAZE_ABOVE_DTM_M above the bare earth is dropped BEFORE the highest-return step, so they never exist.
    Note: the 3x3 spike filter below also removes the three thin wooden poles inside Search Boundary 1 (0.3 m
    wide); they are added back as explicit structures (site.MANUAL_STRUCTURES)."""
    import laspy
    from pyproj import Transformer
    from scipy import ndimage

    files = raw_files(raw, manifest, 'lidar')
    if not files:
        raise FileNotFoundError('no lidar LAZ tiles in raw/lidar: run fetch_real_field_data.sh first')
    x0, y1, cols, rows = world_grid(CHM_RES)
    dsm = np.full(rows * cols, -np.inf, dtype=np.float32)
    tr = Transformer.from_crs('EPSG:6344', site.local_proj4(), always_xy=True)
    total = used = haze = 0
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
                # haze: higher than HAZE_ABOVE_DTM_M above the bare earth under the point -> not a real object
                floating = np.zeros(len(z), dtype=bool)
                floating[ok] = z[ok] > dtm.sample(x[ok], y[ok]) + HAZE_ABOVE_DTM_M
                haze += int(floating.sum())
                ok &= ~floating
                np.maximum.at(dsm, r[ok] * cols + c[ok], z[ok])
                total += len(cls)
                used += int(ok.sum())
        log(f'    {os.path.basename(path)}: {time.time() - t0:.0f} s')
    dsm = dsm.reshape(rows, cols)
    empty = ~np.isfinite(dsm)
    ground = dtm.sample(*_centres(x0, y1, cols, rows, CHM_RES))
    chm = np.where(empty, 0.0, dsm - ground).astype(np.float32)
    clipped = int((chm > CHM_CLIP_M).sum())
    chm = np.clip(chm, 0.0, CHM_CLIP_M)
    med = ndimage.median_filter(chm, size=3)
    spikes = chm > med + 6.0                                        # single-cell birds / wires / noise
    chm[spikes] = med[spikes]
    meta['chm_cleaning'] = {'haze_points_dropped': haze, 'haze_rule_m_above_dtm': HAZE_ABOVE_DTM_M,
                            'cells_clipped_at_m': [clipped, CHM_CLIP_M], 'spike_cells_flattened': int(spikes.sum()),
                            'max_height_m': round(float(chm.max()), 2)}
    log(f'  CHM: {used:,} of {total:,} lidar points on the grid, {haze:,} haze points dropped (> {HAZE_ABOVE_DTM_M:g} m '
        f'above the ground), {100 * empty.mean():.1f} % empty cells, {int(spikes.sum()):,} spikes removed, '
        f'{100 * (chm > 2).mean():.1f} % of cells above 2 m, highest {chm.max():.1f} m')
    grids.save_raster(out, meta, 'chm', chm, x0, y1, CHM_RES,
                      'canopy/structure height above the DTM, m (USGS 3DEP point cloud, flown 2017-02-10, leaf-off; '
                      f'haze > {HAZE_ABOVE_DTM_M:g} m above the ground removed)')
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


def s2_offset(entry):
    """BOA_ADD_OFFSET of one Sentinel-2 L2A manifest entry (reflectance x 10000 = DN + offset).

    Since processing baseline 04.00 (25 Jan 2022) every L2A pixel is stored with +1000 added, i.e. the product
    metadata says BOA_ADD_OFFSET = -1000. Our two scenes are baselines 05.10 and 05.12. Order of trust: the
    manifest's "boa_add_offset", then the baseline in the product name (N0400 or later -> -1000), else -1000."""
    import re
    if entry and 'boa_add_offset' in entry:
        return float(entry['boa_add_offset'])
    text = ' '.join((entry or {}).get('urls', []) + [(entry or {}).get('scene', ''), (entry or {}).get('url', '')])
    m = re.search(r'_N(\d{4})_', text)
    if m:
        return -1000.0 if int(m.group(1)) >= 400 else 0.0
    return float(S2_DEFAULT_OFFSET)


def _ndvi(a):
    """NDVI of a [rows, cols, 4] array ordered R, G, B, NIR (any scale)."""
    return (a[..., 3] - a[..., 0]) / (a[..., 3] + a[..., 0] + 1e-6)


def _ratio_grade(a, b, mask):
    """Median b / a over mask -> (clipped, unclipped) RGB factors; (None, None) without any samples."""
    if not np.any(mask):
        return None, None
    raw = [float(np.median(b[..., k][mask] / np.maximum(a[..., k][mask], 1.0))) for k in range(3)]
    return [round(min(max(v, 0.6), 1.3), 3) for v in raw], [round(v, 3) for v in raw]


def naip_grass_share(naip, rows, cols, factor, block=40):
    """Share (0..1) of grass-like NAIP pixels inside every (factor x factor)-pixel cell of a coarser grid that
    starts at the same corner (the 10 m Sentinel-2 world grid: factor 40 at 0.25 m).

    Grass-like, calibrated on this photo (trees_detect.crown_mask text): sunlit grass has NDVI about 0.20 (95 %
    below 0.34) and brightness about 96 of 255; tree crowns NDVI about 0.43, shadows are dark, bare clay and
    concrete have NDVI below 0.08. So: 0.08 <= NDVI <= 0.36 and 70 <= brightness <= 150."""
    out = np.zeros((rows, cols), dtype=np.float32)
    d = naip.data
    for r0 in range(0, rows, block):
        r1 = min(r0 + block, rows)
        a = np.asarray(d[r0 * factor:r1 * factor, :cols * factor], dtype=np.float32)
        R, G, B, N = a[..., 0], a[..., 1], a[..., 2], a[..., 3]
        nd = (N - R) / (N + R + 1e-6)
        br = (R + G + B) / 3.0
        g = ((nd >= 0.08) & (nd <= 0.36) & (br >= 70.0) & (br <= 150.0)).astype(np.float32)
        h = (r1 - r0)
        out[r0:r1] = g[:h * factor].reshape(h, factor, cols, factor).mean(axis=(1, 3))
    return out


def prepare_season_grade(raw, manifest, meta, naip, log):
    """Optional: how much greener/darker the grass was on competition day (Sentinel-2 2026-09-15) than in the
    NAIP week (Sentinel-2 2023-09-23) -> per-band multipliers for grass pixels (pack.py applies them).

    Research fix 4: Sentinel-2 L2A values carry a +1000 offset since processing baseline 04.00. The first version
    used the raw numbers, so NDVI came out too low (the "grass" mask took 75 % of all cells, forest included) and
    the colour change looked only about half as strong. Now the offset is removed first (s2_offset). Three numbers
    are recorded so the effect can be checked: as before the fix, offset removed with the old NDVI mask (the
    research's check), and offset removed over PURE grass cells (>= 90 % grass pixels in the 0.25 m NAIP photo):
    the last one is what pack.py applies, because the grade is only ever applied to grass pixels."""
    entries = {os.path.basename(s['file']): s for s in manifest['sources'] if s['layer'] == 's2'}
    files = {os.path.basename(p): p for p in raw_files(raw, manifest, 's2')}
    old = next((n for n in files if '2023' in n), None)
    new = next((n for n in files if '2026' in n), None)
    grade = {'factors_rgb': None, 'how': 'none: no Sentinel-2 pair downloaded (pack.py uses site.GRASS_GRADE_RGB)'}
    if old and new:
        A0, _ = _reproject([files[old]], [1, 2, 3, 4], S2_RES, 'nearest', np.float32)
        B0, _ = _reproject([files[new]], [1, 2, 3, 4], S2_RES, 'nearest', np.float32)
        valid = np.isfinite(A0).all(-1) & np.isfinite(B0).all(-1) & (A0 > 0).all(-1) & (B0 > 0).all(-1)
        A0 = np.nan_to_num(A0)
        B0 = np.nan_to_num(B0)
        off_a, off_b = s2_offset(entries.get(old)), s2_offset(entries.get(new))

        def legacy(a, b):                       # the first version's "grass-like" NDVI band (both years)
            return (_ndvi(a) > 0.30) & (_ndvi(a) < 0.62) & (_ndvi(b) > 0.30) & (_ndvi(b) < 0.75)

        m_before = valid & legacy(A0, B0)
        f_before, _ = _ratio_grade(A0, B0, m_before)
        A, B = A0 + off_a, B0 + off_b
        valid &= (A > 0).all(-1) & (B > 0).all(-1)
        m_old_mask = valid & legacy(A, B)
        f_old_mask, f_old_raw = _ratio_grade(A, B, m_old_mask)
        rows, cols = A.shape[:2]
        factor = int(round(S2_RES / naip.res))
        share = naip_grass_share(naip, rows, cols, factor)
        m_pure = valid & (share >= PURE_GRASS_SHARE) & (_ndvi(A) > 0.2) & (_ndvi(B) > 0.2)
        f_pure, f_pure_raw = _ratio_grade(A, B, m_pure)
        n = int(valid.sum())
        use_pure = int(m_pure.sum()) >= 50
        use_old = not use_pure and int(m_old_mask.sum()) >= 50
        # A few pixels (or no overlapping grass at all) cannot establish a season grade. Keep the documented
        # site fallback rather than emitting NaN JSON or applying an unstable ratio from a tiny sample.
        how = 'none: too few overlapping grass cells (pack.py uses site.GRASS_GRADE_RGB)'
        if use_pure or use_old:
            how = (f'median Sentinel-2 L2A reflectance ratio {new} / {old} (offsets {off_b:g} / {off_a:g} '
                   'removed) over ' + (f'pure-grass 10 m cells (>= {PURE_GRASS_SHARE:.0%} grass pixels in NAIP)'
                                      if use_pure else 'grass-like NDVI cells (too few pure-grass cells)'))
        grade = {'factors_rgb': f_pure if use_pure else f_old_mask if use_old else None,
                 'how': how,
                 'cells': int(m_pure.sum() if use_pure else m_old_mask.sum() if use_old else 0),
                 'unclipped_rgb': f_pure_raw if use_pure else f_old_raw if use_old else None,
                 'before_offset_fix': {'factors_rgb': f_before, 'cells': int(m_before.sum()),
                                       'share_of_cells': round(float(m_before.sum()) / max(n, 1), 3)},
                 'offset_fixed_old_mask': {'factors_rgb': f_old_mask, 'cells': int(m_old_mask.sum()),
                                           'share_of_cells': round(float(m_old_mask.sum()) / max(n, 1), 3)},
                 'offset_fixed_pure_grass': {'factors_rgb': f_pure, 'cells': int(m_pure.sum()),
                                             'share_of_cells': round(float(m_pure.sum()) / max(n, 1), 3)},
                 'boa_add_offset': {old: off_a, new: off_b}}
        log(f'  season grade before the offset fix: {f_before} over {grade["before_offset_fix"]["share_of_cells"]:.0%} '
            f'of the cells; offset removed, same mask: {f_old_mask} over '
            f'{grade["offset_fixed_old_mask"]["share_of_cells"]:.0%}; pure grass: {f_pure} over {int(m_pure.sum())} cells')
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


def building_heights_from_lidar(buildings, chm):
    """Research fix 5: give every footprint the MEDIAN lidar canopy height inside it ('height_lidar', m) and its
    90th percentile ('height_lidar_p90'). The 2017 lidar roofs (3.6-5.7 m in the flight boundary) are more
    trustworthy than Microsoft's AI heights (3.0-3.6 m, two missing) and OSM has none here. A footprint whose
    median is below BUILDING_MIN_LIDAR_H was probably built after the 2017 flight: it keeps no lidar height."""
    for b in buildings:
        poly = b['polygon']
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        r0, r1, c0, c1 = chm.window(min(xs), min(ys), max(xs), max(ys))
        if r1 <= r0 or c1 <= c0:
            continue
        X, Y = np.meshgrid(chm.x0 + (np.arange(c0, c1) + 0.5) * chm.res, chm.y1 - (np.arange(r0, r1) + 0.5) * chm.res)
        inside = np.zeros(X.shape, dtype=bool)
        n = len(poly)
        for i in range(n):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % n]
            cond = (y1 > Y) != (y2 > Y)
            xi = (x2 - x1) * (Y - y1) / ((y2 - y1) if y2 != y1 else 1e-12) + x1
            inside ^= cond & (X < xi)
        vals = np.asarray(chm.data[r0:r1, c0:c1])[inside]
        if vals.size >= 4 and float(np.median(vals)) >= BUILDING_MIN_LIDAR_H:
            b['height_lidar'] = round(float(np.median(vals)), 2)
            b['height_lidar_p90'] = round(float(np.percentile(vals, 90)), 2)


def prepare_features(raw, manifest, out, meta, home, log, chm=None):
    """OSM (Overpass JSON) + Microsoft footprints (GeoJSON) -> features.json in world metres (with lidar
    building heights when the canopy model `chm` is given)."""
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
                                       'height': None if h is None or float(h) <= 0 else round(float(h), 2)})
            n_ms += 1
    if chm is not None:
        building_heights_from_lidar(feats['buildings'], chm)
    feats['licences'] = {'osm': 'OpenStreetMap contributors, ODbL 1.0 (keep this credit; share changes under ODbL)',
                         'microsoft': 'Microsoft Global ML Building Footprints, CDLA Permissive 2.0',
                         'height_lidar': 'USGS 3DEP lidar 2017 (public domain)'}
    path = os.path.join(out, 'features.json')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(feats, f, indent=0)
    meta.setdefault('files', {})['features'] = 'features.json'
    n_l = sum(1 for b in feats['buildings'] if b.get('height_lidar'))
    log(f'  features: {len(feats["buildings"])} buildings ({n_ms} from Microsoft footprints, {n_l} with a lidar '
        f'height), {len(feats["water"])} water areas, {len(feats["streams"])} streams, {len(feats["roads"])} roads, '
        f'{len(feats["runways"])} runway')
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
    feats = prepare_features(raw, manifest, out, meta, home, log, chm=chm)
    trees_detect.run(out, meta, dtm, chm, naip, leafoff, metachm, home, log, buildings=feats['buildings'])
    meta['sources'] = [{k: s.get(k) for k in ('id', 'layer', 'file', 'licence', 'attribution', 'sha256', 'pixel_sha256')}
                       for s in manifest['sources'] if os.path.isfile(os.path.join(raw, s['file']))]
    meta['prepare_seconds'] = round(time.time() - t0, 1)
    grids.write_meta(out, meta)
    log(f'prepare: done in {time.time() - t0:.0f} s')
    return out
