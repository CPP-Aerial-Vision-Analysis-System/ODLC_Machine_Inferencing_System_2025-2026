"""
sim_worlds/realfield/glb.py -- PURPOSE: write (and inspect) binary glTF 2.0 mesh files (.glb) with numpy only.
GLB is what the real-terrain world uses for terrain tiles, trees and buildings: it is compact and loads fast in
Gazebo (gz-common reads it through assimp), much faster than OBJ or COLLADA text (gazebo_engine research).

Conventions that matter in Gazebo (checked in the gz-common / assimp source, see WORLDS.md):
  * vertices are written directly in Gazebo's frame (x east, y north, z UP): gz-common keeps the glTF node
    transform and does NOT rotate glTF's usual "y up" into "z up", so we simply never use y-up here;
  * texture coordinates use the glTF rule u -> right, v -> DOWN (v = 0 is the TOP row of the picture);
    assimp flips v when reading glTF and gz-common flips it back (aiProcess_FlipUVs), so it arrives unchanged;
  * triangles are counter-clockwise seen from outside (from above for the ground).
"""

import json
import os
import struct

import numpy as np

_MAGIC, _JSON, _BIN = 0x46546C67, 0x4E4F534A, 0x004E4942


def write_glb(path, prims, materials=None, generator='sim_worlds/realfield'):
    """Write a .glb file. Returns its size in bytes.

    prims: list of dicts with 'pos' (N, 3) float, optional 'nrm' (N, 3), optional 'uv' (N, 2), 'idx' (M,) ints
           (triangles, 3 per face), optional 'mat' (index into materials) and optional 'name'.
    materials: list of dicts {name, rgba (4 floats), roughness=0.9, metallic=0.0, double_sided=False}; a mesh
           whose visual in the SDF has its own <material> ignores these.
    16-bit indices are used whenever a primitive has fewer than 65536 vertices (smaller files)."""
    chunks, views, accessors, mesh_prims = [], [], [], []
    offset = 0

    def add_view(arr, target):
        nonlocal offset
        data = np.ascontiguousarray(arr).tobytes()
        pad = (-len(data)) % 4
        chunks.append(data + b'\0' * pad)
        views.append({'buffer': 0, 'byteOffset': offset, 'byteLength': len(data), 'target': target})
        offset += len(data) + pad
        return len(views) - 1

    for p in prims:
        pos = np.asarray(p['pos'], dtype='<f4')
        n = len(pos)
        attrs = {}
        v = add_view(pos, 34962)
        accessors.append({'bufferView': v, 'componentType': 5126, 'count': n, 'type': 'VEC3',
                          'min': pos.min(0).tolist(), 'max': pos.max(0).tolist()})
        attrs['POSITION'] = len(accessors) - 1
        if p.get('nrm') is not None:
            v = add_view(np.asarray(p['nrm'], dtype='<f4'), 34962)
            accessors.append({'bufferView': v, 'componentType': 5126, 'count': n, 'type': 'VEC3'})
            attrs['NORMAL'] = len(accessors) - 1
        if p.get('uv') is not None:
            v = add_view(np.asarray(p['uv'], dtype='<f4'), 34962)
            accessors.append({'bufferView': v, 'componentType': 5126, 'count': n, 'type': 'VEC2'})
            attrs['TEXCOORD_0'] = len(accessors) - 1
        idx = np.asarray(p['idx'])
        small = n < 65536
        v = add_view(idx.astype('<u2' if small else '<u4'), 34963)
        accessors.append({'bufferView': v, 'componentType': 5123 if small else 5125, 'count': int(len(idx)),
                          'type': 'SCALAR'})
        prim = {'attributes': attrs, 'indices': len(accessors) - 1, 'mode': 4}
        if p.get('mat') is not None and materials:
            prim['material'] = int(p['mat'])
        mesh_prims.append(prim)
    gltf = {'asset': {'version': '2.0', 'generator': generator},
            'scene': 0, 'scenes': [{'nodes': [0]}],
            'nodes': [{'mesh': 0, 'name': os.path.splitext(os.path.basename(path))[0]}],
            'meshes': [{'name': os.path.splitext(os.path.basename(path))[0], 'primitives': mesh_prims}],
            'buffers': [{'byteLength': offset}], 'bufferViews': views, 'accessors': accessors}
    if materials:
        gltf['materials'] = [{'name': m['name'], 'doubleSided': bool(m.get('double_sided', False)),
                              'pbrMetallicRoughness': {'baseColorFactor': [float(c) for c in m['rgba']],
                                                       'metallicFactor': float(m.get('metallic', 0.0)),
                                                       'roughnessFactor': float(m.get('roughness', 0.9))}}
                             for m in materials]
    jb = json.dumps(gltf, separators=(',', ':')).encode()
    jb += b' ' * ((-len(jb)) % 4)
    bb = b''.join(chunks)
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'wb') as f:
        f.write(struct.pack('<III', _MAGIC, 2, 12 + 8 + len(jb) + 8 + len(bb)))
        f.write(struct.pack('<II', len(jb), _JSON))
        f.write(jb)
        f.write(struct.pack('<II', len(bb), _BIN))
        f.write(bb)
    return os.path.getsize(path)


def read_glb_json(path):
    """The JSON part of a .glb (to count vertices / triangles without loading the geometry)."""
    with open(path, 'rb') as f:
        magic, version, _ = struct.unpack('<III', f.read(12))
        if magic != _MAGIC or version != 2:
            raise ValueError(f'{path} is not a glTF 2.0 binary file')
        n, kind = struct.unpack('<II', f.read(8))
        if kind != _JSON:
            raise ValueError(f'{path}: first chunk is not JSON')
        return json.loads(f.read(n))


def glb_stats(path):
    """{'vertices', 'triangles', 'bbox_min', 'bbox_max', 'primitives', 'bytes'} of a .glb file."""
    g = read_glb_json(path)
    acc = g['accessors']
    verts = tris = 0
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    nprim = 0
    for m in g['meshes']:
        for p in m['primitives']:
            a = acc[p['attributes']['POSITION']]
            verts += a['count']
            tris += acc[p['indices']]['count'] // 3
            lo = np.minimum(lo, a['min'])
            hi = np.maximum(hi, a['max'])
            nprim += 1
    return {'vertices': verts, 'triangles': tris, 'bbox_min': lo.tolist(), 'bbox_max': hi.tolist(),
            'primitives': nprim, 'bytes': os.path.getsize(path)}


def read_glb_positions(path):
    """All (positions (N, 3), triangle indices (M, 3)) of a .glb written by write_glb (used by the checks)."""
    with open(path, 'rb') as f:
        f.read(12)
        n, _ = struct.unpack('<II', f.read(8))
        g = json.loads(f.read(n))
        nb, _ = struct.unpack('<II', f.read(8))
        blob = f.read(nb)
    acc, views = g['accessors'], g['bufferViews']

    def arr(i, comps):
        a = acc[i]
        vw = views[a['bufferView']]
        dt = {5126: '<f4', 5125: '<u4', 5123: '<u2'}[a['componentType']]
        x = np.frombuffer(blob, dtype=dt, count=a['count'] * comps, offset=vw['byteOffset'] + a.get('byteOffset', 0))
        return x.reshape(a['count'], comps) if comps > 1 else x
    P, T, base = [], [], 0
    for m in g['meshes']:
        for p in m['primitives']:
            pos = arr(p['attributes']['POSITION'], 3)
            idx = arr(p['indices'], 1).astype(np.int64).reshape(-1, 3)
            P.append(pos)
            T.append(idx + base)
            base += len(pos)
    return np.concatenate(P), np.concatenate(T)
