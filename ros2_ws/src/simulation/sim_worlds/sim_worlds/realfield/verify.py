"""
sim_worlds/realfield/verify.py -- PURPOSE: answer "is the real-terrain world on THIS PC the same world as on the reference PC?" with
a PASS / FAIL list ("build_real_field.py verify"). It checks three things, from the cheapest to the most telling:

  1. the committed world files (worlds/, config/, truth/, missions/) equal what the generator makes again from the site pack
     (same seed -> same targets, props, heights; compared as text);
  2. the GEOMETRY of every generated model (ground tiles, physics mesh, trees, canopy, buildings) equals the reference
     signatures in data/suas_field_real/reference_geometry.json (vertex and triangle counts, and a hash of the positions rounded to
     1 mm, so last-bit floating point noise between CPUs does not matter);
  3. the PICTURES: the average colour of every ground and crown picture is within 3 levels of the reference, and the ground was
     painted from the AI-sharper picture (rf_assets_manifest.json "upscaled_ground": true) as on the reference PC.

What it cannot promise: the AI-sharpened pixels themselves may differ slightly between GPUs and torch versions (fine grass texture,
not shapes or colours), and Gazebo's picture depends on the graphics driver. Geometry, colours, targets and flight plans are exact.

  python3 scripts/build_real_field.py verify                 # compare with the reference
  python3 scripts/build_real_field.py verify --write-reference   # maintainers: record this PC's models as the reference
"""

import hashlib
import json
import os
import struct
import tempfile

import numpy as np

REFERENCE = 'reference_geometry.json'
COLOUR_TOLERANCE = 3.0
_CT = {5126: np.float32, 5123: np.uint16, 5125: np.uint32}
_NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}


def read_glb(path):
    """(positions [N, 3] float64, indices [M]) of the first primitive of a .glb written by glb.write_glb."""
    with open(path, 'rb') as f:
        data = f.read()
    jlen = struct.unpack_from('<I', data, 12)[0]
    j = json.loads(data[20:20 + jlen])
    binch = data[20 + jlen + 8:]

    def acc(i):
        a = j['accessors'][i]
        v = j['bufferViews'][a['bufferView']]
        n = a['count'] * _NC[a['type']]
        arr = np.frombuffer(binch, dtype=_CT[a['componentType']], count=n, offset=v['byteOffset'] + a.get('byteOffset', 0))
        return arr.reshape(-1, _NC[a['type']]) if a['type'] != 'SCALAR' else arr
    p = j['meshes'][0]['primitives'][0]
    return acc(p['attributes']['POSITION']).astype(np.float64), acc(p['indices']).astype(np.int64)


def signature(path):
    """{'v': vertex count, 't': triangle count, 'h': hash of the positions rounded to 1 mm and the indices} of one .glb."""
    pos, idx = read_glb(path)
    h = hashlib.sha256()
    h.update(np.round(pos * 1000.0).astype(np.int64).tobytes())
    h.update(idx.astype(np.int64).tobytes())
    return {'v': int(len(pos)), 't': int(len(idx) // 3), 'h': h.hexdigest()[:20]}


def geometry(models_dir):
    """{'rf_ground/meshes/b00.glb': signature, ...} of every generated mesh."""
    out = {}
    for model in ('rf_ground', 'rf_trees', 'rf_buildings'):
        d = os.path.join(models_dir, model, 'meshes')
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if f.endswith('.glb'):
                    out[f'{model}/meshes/{f}'] = signature(os.path.join(d, f))
    return out


def picture_means(models_dir):
    """{'rf_ground/textures/b00.jpg': [mean R, G, B]} of every ground and crown picture (rounded to 0.1)."""
    from PIL import Image
    out = {}
    for model in ('rf_ground', 'rf_trees'):
        d = os.path.join(models_dir, model, 'textures')
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith(('.jpg', '.png')) and not f.startswith('leaf_atlas'):
                im = Image.open(os.path.join(d, f)).convert('RGB')
                im.draft('RGB', (im.width // 8, im.height // 8)) if f.endswith('.jpg') else None
                a = np.asarray(im.resize((max(im.width // 8, 1), max(im.height // 8, 1))), dtype=np.float64)
                out[f'{model}/textures/{f}'] = [round(float(v), 1) for v in a.reshape(-1, 3).mean(axis=0)]
    return out


def write_reference(pkg, models_dir, log=print):
    """Record this PC's generated models as the reference (maintainers, after a deliberate change of the models)."""
    with open(os.path.join(models_dir, 'rf_assets_manifest.json'), encoding='utf-8') as f:
        man = json.load(f)
    ref = {'purpose': 'Reference signatures of the generated real-field models, made by verify.py --write-reference; '
                      'build_real_field.py verify compares another PC with them.',
           'quality': man.get('quality'), 'upscaled_ground': bool(man.get('upscaled_ground')),
           'geometry': geometry(models_dir), 'picture_means': picture_means(models_dir)}
    path = os.path.join(pkg, 'data', 'suas_field_real', REFERENCE)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        json.dump(ref, f, indent=0, sort_keys=True)
        f.write('\n')
    log(f'[verify] wrote {path}: {len(ref["geometry"])} meshes, {len(ref["picture_means"])} pictures')
    return path


def run(pkg, models_dir, pack_dir=None, log=print):
    """Run the three checks. Returns 0 when the world on this PC matches the reference, else 1."""
    from sim_worlds.realfield import world_build
    failed = 0

    def check(ok, what, detail=''):
        nonlocal failed
        failed += 0 if ok else 1
        log(f'  {"PASS" if ok else "FAIL"}  {what}' + (f'  ({detail})' if detail else ''))

    log(f'[verify] the real-terrain world in {pkg} and the models in {models_dir}')
    # 1. the committed world files equal a fresh generation --------------------------------------------------------------
    with tempfile.TemporaryDirectory() as tmp:
        world_build.run(tmp, pack_dir=pack_dir, asset_mode='fuel', log=lambda *_: None)
        for rel in ('worlds/suas_field_real.sdf', 'config/suas_field_real.yaml', 'truth/suas_field_real.yaml',
                    'missions/suas_field_real_survey.waypoints', 'missions/suas_field_real_sb2_survey.waypoints'):
            a = os.path.join(pkg, rel)
            b = os.path.join(tmp, rel)
            same = os.path.isfile(a) and open(a, encoding='utf-8').read().replace('\r\n', '\n') == \
                open(b, encoding='utf-8').read().replace('\r\n', '\n')
            check(same, f'{rel} equals a fresh generation (same seed, same targets)')
    ref_path = os.path.join(pkg, 'data', 'suas_field_real', REFERENCE)
    if not os.path.isfile(ref_path):
        check(False, 'the reference signatures exist', f'{ref_path} is missing')
        return 1
    with open(ref_path, encoding='utf-8') as f:
        ref = json.load(f)
    man_path = os.path.join(models_dir, 'rf_assets_manifest.json')
    if not os.path.isfile(man_path):
        check(False, 'the Gazebo models exist', 'run: python3 scripts/build_real_field.py assets --install')
        return 1
    with open(man_path, encoding='utf-8') as f:
        man = json.load(f)
    # 2. geometry ---------------------------------------------------------------------------------------------------------
    mine = geometry(models_dir)
    missing = sorted(set(ref['geometry']) - set(mine))
    extra = sorted(set(mine) - set(ref['geometry']))
    differ = [k for k in ref['geometry'] if k in mine and mine[k] != ref['geometry'][k]]
    check(not missing and not extra, 'the same set of meshes', f'missing {len(missing)}, extra {len(extra)}')
    check(not differ, f'all {len(ref["geometry"])} meshes have the reference geometry',
          f'{len(differ)} differ, e.g. {differ[:3]}')
    # 3. pictures ---------------------------------------------------------------------------------------------------------
    check(bool(man.get('upscaled_ground')) == bool(ref['upscaled_ground']),
          'the ground is painted from the AI-sharper picture like the reference' if ref['upscaled_ground'] else
          'the ground is painted like the reference',
          f'this PC: upscaled_ground={bool(man.get("upscaled_ground"))}; fix: bash simulation/setup/upscale_env.sh, '
          f'then upscale and assets (README step 1)' if bool(man.get('upscaled_ground')) != bool(ref['upscaled_ground']) else '')
    pm = picture_means(models_dir)
    off = [k for k, v in ref['picture_means'].items()
           if k not in pm or max(abs(a - b) for a, b in zip(pm[k], v)) > COLOUR_TOLERANCE]
    check(not off, f'the average colour of all {len(ref["picture_means"])} pictures is within {COLOUR_TOLERANCE:g} levels',
          f'{len(off)} off, e.g. {off[:3]}')
    log(f'[verify] {"this PC builds the same world as the reference PC" if not failed else str(failed) + " check(s) FAILED"}')
    return 0 if not failed else 1
