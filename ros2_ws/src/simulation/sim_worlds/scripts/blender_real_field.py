"""
sim_worlds/scripts/blender_real_field.py -- PURPOSE: runs INSIDE Blender (5.x). Builds a Blender scene from the generated real-field Gazebo
models (rf_ground, rf_trees, rf_buildings: the exact .glb meshes and baked pictures Gazebo loads), lights it with the real sun direction,
saves it as suas_field_real.blend and renders the test views (the drone's own nadir view 50 m above the ground, oblique and overview shots).
Start it with  python3 scripts/build_real_field.py blender  (it finds Blender, writes views.json and calls this with the right paths).
Direct use:  blender --background --python blender_real_field.py -- MODELS_DIR VIEWS_JSON OUT_DIR [samples] [sun=4.0] [sky=0.1]
             [suncol=1.0,0.96,0.88] [only=view1,view2] [--no-render]
It reads the .glb files itself (no addon), so it needs no addon (tested with Blender 5.2.3); crown pictures per block (crowns_<block>.jpg) are used when
the models have them. The lighting numbers were tuned by eye against the baked ground pictures: Gazebo, not Blender, is what the detector sees.
"""
import json
import math
import os
import struct
import sys

import bpy
import numpy as np

args = sys.argv[sys.argv.index('--') + 1:]
MODELS, VIEWS, OUT = args[0], args[1], args[2]
SAMPLES = int(args[3]) if len(args) > 3 and args[3].isdigit() else 48
NO_RENDER = '--no-render' in args
KV = dict(a.split('=', 1) for a in args if '=' in a)
SUN_STRENGTH = float(KV.get('sun', 4.0))
SKY_STRENGTH = float(KV.get('sky', 0.1))
ONLY = KV.get('only', '').split(',') if KV.get('only') else None
os.makedirs(OUT, exist_ok=True)

_CT = {5126: np.float32, 5123: np.uint16, 5125: np.uint32}
_NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}


def read_glb(path):
    """(pos [N,3], uv [N,2] or None, idx [M]) of the first primitive of a .glb (our own writer's layout)."""
    data = open(path, 'rb').read()
    jlen = struct.unpack_from('<I', data, 12)[0]
    j = json.loads(data[20:20 + jlen])
    off = 20 + jlen + 8
    binch = data[off:]

    def acc(i):
        a = j['accessors'][i]
        v = j['bufferViews'][a['bufferView']]
        n = a['count'] * _NC[a['type']]
        arr = np.frombuffer(binch, dtype=_CT[a['componentType']], count=n, offset=v['byteOffset'] + a.get('byteOffset', 0))
        return arr.reshape(-1, _NC[a['type']]) if a['type'] != 'SCALAR' else arr
    p = j['meshes'][0]['primitives'][0]
    pos = acc(p['attributes']['POSITION']).astype(np.float64)
    uv = acc(p['attributes']['TEXCOORD_0']).astype(np.float64) if 'TEXCOORD_0' in p['attributes'] else None
    idx = acc(p['indices']).astype(np.int64)
    return pos, uv, idx


def make_object(name, pos, uv, idx, mat):
    me = bpy.data.meshes.new(name)
    nv, nf = len(pos), len(idx) // 3
    me.vertices.add(nv)
    me.vertices.foreach_set('co', pos.astype(np.float32).ravel())
    me.loops.add(nf * 3)
    me.loops.foreach_set('vertex_index', idx.astype(np.int32))
    me.polygons.add(nf)
    me.polygons.foreach_set('loop_start', (np.arange(nf) * 3).astype(np.int32))
    me.polygons.foreach_set('loop_total', np.full(nf, 3, dtype=np.int32)) if hasattr(me.polygons[0], 'loop_total') else None
    me.update()
    if uv is not None:
        layer = me.uv_layers.new(name='UVMap')
        # glTF v runs DOWN from the top of the picture; Blender's v runs UP from the bottom
        luv = uv[idx].copy()
        luv[:, 1] = 1.0 - luv[:, 1]
        layer.data.foreach_set('uv', luv.astype(np.float32).ravel())
    me.polygons.foreach_set('use_smooth', np.ones(nf, dtype=bool))
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    if mat is not None:
        me.materials.append(mat)
    return ob


_mats = {}


def image_material(path, roughness=0.95):
    if path in _mats:
        return _mats[path]
    m = bpy.data.materials.new(os.path.basename(path))
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes['Principled BSDF']
    bsdf.inputs['Roughness'].default_value = roughness
    if 'Specular IOR Level' in bsdf.inputs:
        bsdf.inputs['Specular IOR Level'].default_value = 0.05
    tex = nt.nodes.new('ShaderNodeTexImage')
    tex.image = bpy.data.images.load(path)
    tex.image.colorspace_settings.name = 'sRGB'
    tex.interpolation = 'Linear'
    nt.links.new(tex.outputs['Color'], bsdf.inputs['Base Color'])
    _mats[path] = m
    return m


def plain_material(name, rgb, roughness=0.9):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes['Principled BSDF']
    b.inputs['Base Color'].default_value = (rgb[0], rgb[1], rgb[2], 1.0)
    b.inputs['Roughness'].default_value = roughness
    return m


bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

counts = {'ground': 0, 'trees': 0, 'buildings': 0}
g = os.path.join(MODELS, 'rf_ground')
for f in sorted(os.listdir(os.path.join(g, 'meshes'))):
    if f == 'collision.glb':
        continue
    name = f[:-4]
    pos, uv, idx = read_glb(os.path.join(g, 'meshes', f))
    make_object('ground_' + name, pos, uv, idx, image_material(os.path.join(g, 'textures', name + '.jpg')))
    counts['ground'] += 1
t = os.path.join(MODELS, 'rf_trees')
atlas = image_material(os.path.join(t, 'textures', 'leaf_atlas.png'), 0.9)
trunk = plain_material('trunk', (0.28, 0.2, 0.14))
for f in sorted(os.listdir(os.path.join(t, 'meshes'))):
    name = f[:-4]
    pos, uv, idx = read_glb(os.path.join(t, 'meshes', f))
    if name.startswith('crowns_'):
        tex_top = os.path.join(t, 'textures', name + '.jpg')
        mat = image_material(tex_top, 0.9) if os.path.isfile(tex_top) else atlas
    elif name.startswith('trunks_'):
        mat = trunk
    else:
        mat = image_material(os.path.join(t, 'textures', name + '.jpg'), 0.95)
    make_object('trees_' + name, pos, uv, idx, mat)
    counts['trees'] += 1
b = os.path.join(MODELS, 'rf_buildings')
if os.path.isdir(os.path.join(b, 'meshes')):
    pal = os.path.join(b, 'textures', 'palette.png')
    for f in sorted(os.listdir(os.path.join(b, 'meshes'))):
        pos, uv, idx = read_glb(os.path.join(b, 'meshes', f))
        make_object('buildings_' + f[:-4], pos, uv, idx, image_material(pal, 0.85))
        counts['buildings'] += 1
print('IMPORTED', counts)

cfg = json.load(open(VIEWS))
d = np.array(cfg['sun_direction_enu'], dtype=float)
d /= np.linalg.norm(d)
sun_data = bpy.data.lights.new('Sun', 'SUN')
sun_data.energy = SUN_STRENGTH
sun_data.angle = math.radians(0.53)                      # the real sun's width in the sky
sun_data.color = tuple(float(x) for x in KV.get('suncol', '1.0,0.96,0.88').split(','))
sun = bpy.data.objects.new('Sun', sun_data)
scene.collection.objects.link(sun)
from mathutils import Vector
sun.rotation_euler = Vector((float(d[0]), float(d[1]), float(d[2]))).to_track_quat('-Z', 'Y').to_euler()

world = bpy.data.worlds.new('Sky')
world.use_nodes = True
nt = world.node_tree
for n in list(nt.nodes):
    nt.nodes.remove(n)
sky = nt.nodes.new('ShaderNodeTexSky')
sky.sky_type = 'MULTIPLE_SCATTERING' if 'MULTIPLE_SCATTERING' in sky.bl_rna.properties['sky_type'].enum_items.keys() else 'NISHITA'
sky.sun_disc = False
sky.sun_elevation = math.asin(-d[2])
bg = nt.nodes.new('ShaderNodeBackground')
bg.inputs['Strength'].default_value = SKY_STRENGTH
out = nt.nodes.new('ShaderNodeOutputWorld')
nt.links.new(sky.outputs['Color'], bg.inputs['Color'])
nt.links.new(bg.outputs['Background'], out.inputs['Surface'])
scene.world = world

scene.render.engine = 'CYCLES'
scene.cycles.samples = SAMPLES
scene.cycles.use_denoising = True
scene.view_settings.view_transform = 'Standard'
scene.render.resolution_x, scene.render.resolution_y = 1280, 720
scene.render.image_settings.file_format = 'PNG'
try:
    prefs = bpy.context.preferences.addons['cycles'].preferences
    for dt in ('OPTIX', 'CUDA'):
        try:
            prefs.compute_device_type = dt
            prefs.get_devices()
            gpus = [dev for dev in prefs.devices if dev.type != 'CPU']
            if gpus:
                for dev in prefs.devices:
                    dev.use = dev.type != 'CPU'
                scene.cycles.device = 'GPU'
                print('RENDER DEVICE', dt, [dev.name for dev in gpus])
                break
        except Exception as exc:                          # noqa: BLE001
            print('device', dt, 'failed:', exc)
except Exception as exc:                                  # noqa: BLE001
    print('GPU setup failed, using CPU:', exc)

cam_data = bpy.data.cameras.new('DroneCam')
cam_data.sensor_fit = 'HORIZONTAL'
cam_data.angle = math.radians(cfg.get('hfov_deg', 81.0))
cam_data.clip_end = 3000
cam = bpy.data.objects.new('DroneCam', cam_data)
scene.collection.objects.link(cam)
scene.camera = cam

blend = os.path.join(OUT, 'suas_field_real.blend')
bpy.ops.wm.save_as_mainfile(filepath=blend)
print('SAVED', blend)
if not NO_RENDER:
    for v in cfg['views']:
        if ONLY and v['name'] not in ONLY:
            continue
        cam.location = (v['x'], v['y'], v['z'])
        cam.rotation_euler = (math.radians(90.0 + v['pitch_deg']), 0.0, -math.radians(v['yaw_deg']))
        scene.render.filepath = os.path.join(OUT, v['name'] + '.png')
        bpy.ops.render.render(write_still=True)
        print('RENDERED', v['name'])
