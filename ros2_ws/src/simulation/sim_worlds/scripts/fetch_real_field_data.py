#!/usr/bin/env python3
"""
sim_worlds/scripts/fetch_real_field_data.py -- PURPOSE: the Python half of fetch_real_field_data.sh. It
downloads (or copies) every raw input of the real-terrain world "suas_field_real" into
~/.astra_sim/world_data/suas_field_real/raw/ and proves each file is the right one with a checksum, exactly as
listed in sim_worlds/data/real_field_manifest.json. Run it through the .sh wrapper (that picks the right Python).

Download methods (manifest field "method"):
  http         plain file download (USGS elevation tiles, lidar point clouds, CC0 textures, ...) -> SHA-256 of
               the file must match "sha256"
  cog_window   read only a pixel window of a big cloud-optimised GeoTIFF (NAIP photo, Meta canopy height,
               Sentinel-2 bands) with rasterio, save it as a small GeoTIFF -> SHA-256 of the decoded PIXELS
               must match "pixel_sha256" (pixels do not depend on the GDAL version; the file bytes would)
  overpass     OpenStreetMap query at a FIXED date ("[date:...]") -> SHA-256 of the sorted elements
  ms_buildings Microsoft building footprints: download one dated quadkey file, keep the buildings inside the
               area -> SHA-256 of the downloaded file
  copy_only    cannot be downloaded automatically (see "how"); only copied with --from

Planetary Computer (NAIP, Sentinel-2) needs a short-lived, free, anonymous access token, fetched here from
https://planetarycomputer.microsoft.com/api/sas/v1/token/<account>/<container>.

Usage (normally via the .sh):
  fetch_real_field_data.py [--data DIR] [--from DIR] [--only LAYER[,LAYER]] [--optional] [--check] [--force]
                           [--record]
  --from DIR   copy files from a folder that already has them (same relative paths or same file names),
               still verified by checksum. --record writes the measured checksums back into the manifest
               (maintainers only, after a deliberate data update).
"""

import argparse
import gzip
import hashlib
import io
import json
import os
import shutil
import sys
import time
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
UA = {'User-Agent': 'sim_worlds-fetch/1.0 (SUAS student team digital twin)'}


def manifest_path():
    """The manifest next to the package (source tree) or in the installed share folder."""
    for cand in (os.path.join(PKG, 'data', 'real_field_manifest.json'),
                 os.path.join(PKG, 'share', 'sim_worlds', 'data', 'real_field_manifest.json')):
        if os.path.isfile(cand):
            return cand
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('sim_worlds'), 'data', 'real_field_manifest.json')
    except Exception:                                                  # noqa: BLE001
        sys.exit('real_field_manifest.json not found: run from the sim_worlds source tree or source the workspace')


def sha256_file(path):
    """SHA-256 hex digest of a file (read in 1 MB pieces)."""
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def sha256_pixels(path):
    """SHA-256 of the decoded pixel values of a GeoTIFF (all bands, band-sequential, native dtype)."""
    import numpy as np
    import rasterio
    with rasterio.open(path) as ds:
        a = ds.read()
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def sha256_osm(path):
    """SHA-256 of an Overpass JSON's elements, sorted by (type, id), as compact JSON (header ignored)."""
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    els = sorted(d.get('elements', []), key=lambda e: (e.get('type', ''), e.get('id', 0)))
    return hashlib.sha256(json.dumps(els, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def verify(entry, path):
    """(ok, measured_checksum, which) for a downloaded file, using the entry's checksum kind."""
    if entry['method'] == 'cog_window':
        got, want, kind = sha256_pixels(path), entry.get('pixel_sha256'), 'pixel_sha256'
    elif entry['method'] == 'overpass':
        got, want, kind = sha256_osm(path), entry.get('elements_sha256'), 'elements_sha256'
    elif entry['method'] == 'ms_buildings':
        got, want, kind = sha256_osm_geojson(path), entry.get('features_sha256'), 'features_sha256'
    else:
        got, want, kind = sha256_file(path), entry.get('sha256'), 'sha256'
    return (want is None or got == want), got, kind


def sha256_osm_geojson(path):
    """SHA-256 of a GeoJSON's features (sorted compact JSON)."""
    with open(path, encoding='utf-8') as f:
        d = json.load(f)
    feats = sorted((json.dumps(ft, sort_keys=True, separators=(',', ':')) for ft in d.get('features', [])))
    return hashlib.sha256('\n'.join(feats).encode()).hexdigest()


def http_get(url, dest, tries=3):
    """Download url to dest (atomic: .part file then rename), with a few retries."""
    tmp = dest + '.part'
    for k in range(tries):
        try:
            req = urllib.request.Request(url, headers=UA)
            with urllib.request.urlopen(req, timeout=120) as r, open(tmp, 'wb') as f:
                shutil.copyfileobj(r, f, 1 << 20)
            os.replace(tmp, dest)
            return
        except Exception as exc:                                       # noqa: BLE001
            print(f'    try {k + 1}/{tries} failed: {exc}')
            time.sleep(3 * (k + 1))
    raise RuntimeError(f'download failed: {url}')


_TOKENS = {}


def pc_sign(url, sas):
    """Append a Planetary Computer SAS token for 'account/container' to a blob URL."""
    if not sas:
        return url
    if sas not in _TOKENS:
        api = f'https://planetarycomputer.microsoft.com/api/sas/v1/token/{sas}'
        with urllib.request.urlopen(urllib.request.Request(api, headers=UA), timeout=60) as r:
            _TOKENS[sas] = json.load(r)['token']
    return url + ('&' if '?' in url else '?') + _TOKENS[sas]


def _read_chunked(url, sas, bands, col, row, w, h, chunk=1024, tries=4):
    """Read a big pixel window of a remote cloud GeoTIFF in chunk x chunk pieces, retrying each piece.

    WHY in pieces: one huge read (the NAIP window is 6764 x 6764 x 4 = 183 MB) runs for minutes over the
    network, and a single dropped connection anywhere in it used to throw the whole read away ("Read failed").
    Here a failed piece is retried (with a fresh Planetary Computer token, they expire after about an hour)
    and only that piece is downloaded again. Returns (array [bands, h, w], profile of the source file)."""
    import numpy as np
    import rasterio
    from rasterio.windows import Window
    src, out, profile = None, None, None
    pieces = [(r0, c0) for r0 in range(0, h, chunk) for c0 in range(0, w, chunk)]
    try:
        for n, (r0, c0) in enumerate(pieces, start=1):
            win = Window(col + c0, row + r0, min(chunk, w - c0), min(chunk, h - r0))
            for k in range(tries):
                try:
                    if src is None:
                        src = rasterio.open(pc_sign(url, sas))
                    a = src.read(bands or list(range(1, src.count + 1)), window=win)
                    break
                except Exception as exc:                               # noqa: BLE001
                    print(f'    piece {n}/{len(pieces)} try {k + 1}/{tries} failed: {exc}')
                    if src is not None:
                        src.close()
                    src = None
                    _TOKENS.pop(sas, None)                             # ask for a fresh token next time
                    time.sleep(5 * (k + 1))
            else:
                raise RuntimeError(f'piece {n} of {url} failed {tries} times')
            if out is None:
                out = np.zeros((a.shape[0], h, w), dtype=a.dtype)
                profile = src.profile.copy()
                profile['transform'] = src.window_transform(Window(col, row, w, h))
            out[:, r0:r0 + a.shape[1], c0:c0 + a.shape[2]] = a
            if n % 10 == 0 or n == len(pieces):
                print(f'    {n}/{len(pieces)} pieces read')
    finally:
        if src is not None:
            src.close()
    return out, profile


def cog_window(entry, dest):
    """Read the manifest's pixel window [col_off, row_off, width, height] of one or more COG bands and save
    them as one GeoTIFF (band order = the order of entry['urls'] or entry['bands'])."""
    import numpy as np
    import rasterio
    col, row, w, h = entry['window_px']
    urls = entry.get('urls') or [entry['url']]
    arrays, profile = [], None
    for url in urls:
        bands = entry.get('bands') if len(urls) == 1 else [1]
        a, prof = _read_chunked(url, entry.get('sas'), bands, col, row, w, h)
        arrays.append(a)
        if profile is None:
            profile = prof
            profile.update(driver='GTiff', width=w, height=h, compress='deflate', tiled=True, blockxsize=512,
                           blockysize=512, BIGTIFF='IF_SAFER')
            profile.pop('photometric', None)
            profile.pop('jpeg_quality', None)
    a = np.concatenate(arrays, axis=0)
    profile.update(count=a.shape[0], dtype=str(a.dtype))
    tmp = dest + '.part.tif'
    with rasterio.open(tmp, 'w', **profile) as dst:
        dst.write(a)
    os.replace(tmp, dest)


def overpass(entry, dest):
    """POST the manifest's Overpass query (it carries a fixed [date:...]) and save the JSON answer."""
    data = urllib.parse.urlencode({'data': entry['query']}).encode()
    last = None
    for url in entry.get('endpoints', ['https://overpass-api.de/api/interpreter']):
        try:
            req = urllib.request.Request(url, data=data, headers=UA)
            with urllib.request.urlopen(req, timeout=180) as r:
                body = r.read()
            json.loads(body)
            with open(dest, 'wb') as f:
                f.write(body)
            return
        except Exception as exc:                                       # noqa: BLE001
            last = exc
            print(f'    {url}: {exc}')
    raise RuntimeError(f'all Overpass endpoints failed: {last}')


def ms_buildings(entry, dest):
    """Download the dated Microsoft footprint file of one quadkey and keep the buildings in the bbox."""
    raw = dest + '.quadkey.csv.gz'
    if not (os.path.isfile(raw) and entry.get('source_sha256') and sha256_file(raw) == entry['source_sha256']):
        http_get(entry['url'], raw)
    lon0, lat0, lon1, lat1 = entry['bbox_lonlat']
    feats = []
    with gzip.open(raw, 'rt', encoding='utf-8') as f:
        for line in f:
            ft = json.loads(line)
            ring = ft['geometry']['coordinates'][0] if ft['geometry']['type'] == 'Polygon' else \
                ft['geometry']['coordinates'][0][0]
            cx = sum(p[0] for p in ring) / len(ring)
            cy = sum(p[1] for p in ring) / len(ring)
            if lon0 <= cx <= lon1 and lat0 <= cy <= lat1:
                feats.append(ft)
    with open(dest, 'w', encoding='utf-8') as f:
        json.dump({'type': 'FeatureCollection', 'features': feats}, f)
    os.remove(raw)


def find_local(entry, from_dir):
    """A file for this entry inside from_dir: same relative path, else the same file name anywhere below."""
    p = os.path.join(from_dir, entry['file'])
    if os.path.isfile(p):
        return p
    name = os.path.basename(entry['file'])
    for alias in [name] + entry.get('aliases', []):
        for root, _, files in os.walk(from_dir):
            if alias in files:
                return os.path.join(root, alias)
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--data', default=None, help='data folder (default $ASTRA_WORLD_DATA or ~/.astra_sim/world_data)')
    ap.add_argument('--from', dest='from_dir', default=None, help='copy files from this folder instead of downloading')
    ap.add_argument('--only', default=None, help='comma-separated layers (dtm,lidar,naip,osm,buildings,texture,...)')
    ap.add_argument('--optional', action='store_true', help='also fetch the optional layers (leaf-off photo, ...)')
    ap.add_argument('--check', action='store_true', help='only verify what is already there (no network)')
    ap.add_argument('--force', action='store_true', help='download again even if a verified file exists')
    ap.add_argument('--record', action='store_true', help='MAINTAINERS: write measured checksums into the manifest')
    ap.add_argument('--list', action='store_true', help='print the sources and licences and exit')
    args = ap.parse_args(argv)
    try:                                       # print each line at once, also when the output goes to a log file
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:                                             # very old Python: harmless
        pass

    mpath = manifest_path()
    with open(mpath, encoding='utf-8') as f:
        manifest = json.load(f)
    base = args.data or os.environ.get('ASTRA_WORLD_DATA') or os.path.join(os.path.expanduser('~'), '.astra_sim',
                                                                           'world_data')
    root = base if os.path.basename(os.path.normpath(base)) == manifest['world'] else os.path.join(base,
                                                                                                    manifest['world'])
    raw = os.path.join(root, 'raw')
    only = set(args.only.split(',')) if args.only else None
    if args.list:
        for e in manifest['sources']:
            print(f'{e["layer"]:10s} {"required" if e.get("required") else "optional":8s} {e["file"]}\n'
                  f'           {e.get("licence", "")}')
        return 0
    print(f'fetch_real_field_data: {len(manifest["sources"])} sources in the manifest; data folder {raw}')
    bad, done, skipped = [], 0, 0
    changed = False
    for e in manifest['sources']:
        if only and e['layer'] not in only:
            continue
        if not e.get('required') and not args.optional and not only:
            skipped += 1
            continue
        dest = os.path.join(raw, e['file'])
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        if os.path.isfile(dest) and not args.force:
            ok, got, kind = verify(e, dest)
            if ok:
                print(f'  ok        {e["file"]}')
                done += 1
                continue
            print(f'  MISMATCH  {e["file"]} ({kind} {got[:12]}... expected {str(e.get(kind))[:12]}...)')
            if args.check:
                bad.append(e['file'])
                continue
        elif args.check:
            print(f'  missing   {e["file"]}')
            if e.get('required'):
                bad.append(e['file'])
            continue
        t0 = time.time()
        src = find_local(e, args.from_dir) if args.from_dir else None
        try:
            if src:
                print(f'  copy      {e["file"]}  <- {src}')
                shutil.copyfile(src, dest)
            elif e['method'] == 'http':
                print(f'  download  {e["file"]}  ({e.get("bytes", 0) / 1e6:.1f} MB)')
                http_get(e['url'], dest)
            elif e['method'] == 'cog_window':
                print(f'  window    {e["file"]}  (reading part of a cloud GeoTIFF)')
                cog_window(e, dest)
            elif e['method'] == 'overpass':
                print(f'  overpass  {e["file"]}')
                overpass(e, dest)
            elif e['method'] == 'ms_buildings':
                print(f'  buildings {e["file"]}')
                ms_buildings(e, dest)
            else:
                print(f'  SKIP      {e["file"]}: {e.get("how", "cannot be downloaded automatically")}')
                if e.get('required'):
                    bad.append(e['file'])
                continue
        except Exception as exc:                                       # noqa: BLE001
            print(f'  FAILED    {e["file"]}: {exc}')
            bad.append(e['file'])
            continue
        ok, got, kind = verify(e, dest)
        if args.record:
            e[kind] = got
            if e['method'] == 'http' or kind == 'sha256':
                e['bytes'] = os.path.getsize(dest)
            changed = True
            ok = True
        print(f'            {"verified" if ok else "CHECKSUM MISMATCH"} ({kind} {got[:16]}...), '
              f'{os.path.getsize(dest) / 1e6:.1f} MB, {time.time() - t0:.0f} s')
        if ok:
            done += 1
        else:
            bad.append(e['file'])
    if changed:
        with open(mpath, 'w', encoding='utf-8', newline='\n') as f:
            json.dump(manifest, f, indent=1)
            f.write('\n')
        print(f'recorded checksums into {mpath}')
    print(f'fetch_real_field_data: {done} ok, {len(bad)} problem(s), {skipped} optional skipped'
          + ('' if not bad else '\n  problems: ' + ', '.join(bad)))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
