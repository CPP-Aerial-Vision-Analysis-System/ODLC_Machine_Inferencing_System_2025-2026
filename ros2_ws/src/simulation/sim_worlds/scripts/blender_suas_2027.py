#!/usr/bin/env python3
"""PURPOSE: Turn a 2027 scene recipe into editable Blender geometry and portable Gazebo meshes.

Run through build_suas_2027.py. All geometry/textures are procedural and generated locally. Terrain
OBJ and collision use identical triangles; scenery uses a small COLLADA writer because recent Blender
versions no longer bundle the old exporter. Preview cameras never add objects to the exported scene.
"""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import random
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Euler, Vector
import numpy as np

MATERIALS = {}
COLORS = {}


def material(name, color, roughness=0.85):
    """Share simple exportable materials; avoid procedural-only shaders that Gazebo would lose."""
    if name not in MATERIALS:
        m = bpy.data.materials.new(name)
        m.diffuse_color = (*color, 1)
        m.use_nodes = True
        bsdf = m.node_tree.nodes.get('Principled BSDF')
        bsdf.inputs['Base Color'].default_value = (*color, 1)
        bsdf.inputs['Roughness'].default_value = roughness
        MATERIALS[name] = m
    return MATERIALS[name]


def box(name, center, size, mat, yaw=0, bevel=0):
    """Create a box with a modest bevel for clothing, luggage, buildings and vehicles."""
    bpy.ops.mesh.primitive_cube_add(size=1, location=center)
    obj = bpy.context.object
    obj.name, obj.dimensions, obj.rotation_euler.z = name, size, yaw
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new('rounded_edges', 'BEVEL')
        mod.width, mod.segments = bevel, 3
    return obj


def ellipsoid(name, center, radii, mat, detail=12):
    """Smooth low-cost surfaces for anatomically proportioned limbs, rocks and foliage."""
    bpy.ops.mesh.primitive_uv_sphere_add(segments=detail, ring_count=8, location=center)
    obj = bpy.context.object
    obj.name, obj.scale = name, radii
    obj.data.materials.append(mat)
    for polygon in obj.data.polygons:
        polygon.use_smooth = True
    return obj


def rod(name, start, end, radius, mat):
    """Cylinder joining exact endpoints; also used for visible overhead cables."""
    a, b = Vector(start), Vector(end)
    bpy.ops.mesh.primitive_cylinder_add(vertices=10, radius=radius, depth=(b-a).length,
                                       location=(a+b)/2)
    obj = bpy.context.object
    obj.name = name
    obj.rotation_euler = (b-a).to_track_quat('Z', 'Y').to_euler()
    obj.data.materials.append(mat)
    return obj


def limb(name, start, end, radius, mat):
    """A tapered-looking rounded limb, with joint continuity rather than disconnected blocks."""
    a, b = Vector(start), Vector(end)
    obj = ellipsoid(name, (a+b)/2, (radius, radius * 0.9, (b-a).length/2 + radius*0.4), mat)
    obj.rotation_euler = (b-a).to_track_quat('Z', 'Y').to_euler()
    return obj


def mannequin(person, name, standing=False):
    """Build separate shirt, pants, skin, shoes and hair; articulate supine/prone/seated poses."""
    shirt = material('shirt_' + person['shirt_color'], COLORS[person['shirt_color']])
    pants = material('pants_' + person['pants_color'], COLORS[person['pants_color']])
    skin = material('skin', (0.47, 0.29, 0.17))
    dark = material('boots', (0.025, 0.021, 0.017))
    hair = material('hair', (0.04, 0.025, 0.012))
    before = set(bpy.data.objects)
    seated = person.get('pose') == 'seated' and not standing
    hip_z = 0.5 if seated else 0.9
    torso_z = hip_z + 0.35
    ellipsoid(name + '_shirt', (0, 0, torso_z), (0.235, 0.14, 0.32), shirt, 20)
    ellipsoid(name + '_hips', (0, 0, hip_z), (0.20, 0.14, 0.16), pants)
    rod(name + '_neck', (0, 0, torso_z + .27), (0, 0, torso_z + .40), .07, skin)
    ellipsoid(name + '_head', (0, -.01, torso_z + .48), (.105, .105, .14), skin, 20)
    ellipsoid(name + '_hair', (0, .025, torso_z + .56), (.108, .09, .075), hair)
    ellipsoid(name + '_nose', (0, -.108, torso_z + .48), (.025, .035, .025), skin)
    for side in (-1, 1):
        shoulder = (side * .22, 0, torso_z + .17)
        elbow = (side * .32, -.04, torso_z - .12)
        wrist = (side * .4, -.13, torso_z - .35)
        limb(name + '_sleeve', shoulder, elbow, .075, shirt)
        limb(name + '_forearm', elbow, wrist, .05, skin)
        ellipsoid(name + '_hand', wrist, (.055, .04, .08), skin)
        hip = (side * .115, 0, hip_z)
        knee = (side * .15, -.43 if seated else -.015, .46 if seated else .49)
        ankle = (side * .17, -.63 if seated else -.025, .10)
        limb(name + '_thigh', hip, knee, .09, pants)
        limb(name + '_shin', knee, ankle, .065, pants)
        ellipsoid(name + '_boot', (ankle[0], ankle[1]-.045, .075), (.08, .145, .07), dark)
    objects = list(set(bpy.data.objects) - before)
    # Pose in local coordinates, then put the LOWEST mesh point on the exact terrain triangle.
    # bpy defers scale/location updates: reading matrix_world immediately can turn the last boot
    # back into a unit sphere. Flush those updates before copying any object transform.
    bpy.context.view_layer.update()
    pose = person.get('pose', 'standing')
    rotation = Euler((0 if seated or standing else (-math.pi/2 if pose == 'supine' else math.pi/2),
                      0, person['yaw'])).to_matrix().to_4x4()
    for obj in objects:
        obj.matrix_world = rotation @ obj.matrix_world
    bpy.context.view_layer.update()
    points = [obj.matrix_world @ Vector(v) for obj in objects for v in obj.bound_box]
    lo, hi = [min(p[i] for p in points) for i in range(3)], [max(p[i] for p in points) for i in range(3)]
    translation = Vector((person['x']-(lo[0]+hi[0])/2, person['y']-(lo[1]+hi[1])/2,
                          person['z'] - lo[2] + .025))
    for obj in objects:
        obj.location += translation
    return dict(size=[hi[i]-lo[i] for i in range(3)],
                center=[person['x'], person['y'], person['z']+(hi[2]-lo[2])/2+.025], yaw=0)


def suitcase(name, pos, color):
    """Suitcase with contrasting handles, seams and wheels, associated with one nearby mannequin."""
    x, y, z = pos['x'], pos['y'], pos['z']
    mat = material('luggage_' + color, COLORS[color])
    dark = material('luggage_trim', (0.035, .04, .045))
    box(name, (x, y, z+.16), (.65, .44, .30), mat, bevel=.055)
    box(name+'_seam', (x, y, z+.17), (.66, .445, .016), dark, bevel=.007)
    for dx in (-.09, .09):
        rod(name+'_handle', (x+dx, y, z+.31), (x+dx, y, z+.39), .012, dark)
    rod(name+'_grip', (x-.09, y, z+.39), (x+.09, y, z+.39), .018, dark)
    for dx in (-.25, .25):
        ellipsoid(name+'_wheel', (x+dx, y+.15, z+.035), (.04,.04,.04), dark)


def make_terrain(scene, mesh_dir):
    """One textured grid with an OBJ UV map; collision references this identical OBJ."""
    grid = scene['terrain']['heights']
    GRID_CELLS = len(grid) - 1
    GRID_MIN = scene['terrain']['origin']
    GRID_STEP = scene['terrain']['step_m']
    n = GRID_CELLS + 1
    vertices = [(GRID_MIN[0]+i*GRID_STEP, GRID_MIN[1]+j*GRID_STEP, grid[j][i])
                for j in range(n) for i in range(n)]
    faces = []
    for j in range(GRID_CELLS):
        for i in range(GRID_CELLS):
            k = j*n+i
            faces += [(k, k+1, k+n+1), (k, k+n+1, k+n)]
    mesh = bpy.data.meshes.new('terrain')
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new('terrain', mesh)
    bpy.context.collection.objects.link(obj)
    uv = mesh.uv_layers.new(name='map')
    for polygon in mesh.polygons:
        polygon.use_smooth = True
        for loop in polygon.loop_indices:
            vi = mesh.loops[loop].vertex_index
            uv.data[loop].uv = ((vi % n)/GRID_CELLS, (vi // n)/GRID_CELLS)
    # A real exported albedo image, not a Blender-only Noise node. Seed varies fine texture consistently.
    pixels = 1024
    xs = np.linspace(GRID_MIN[0], GRID_MIN[0]+GRID_STEP*GRID_CELLS, pixels)
    ys = np.linspace(GRID_MIN[1], GRID_MIN[1]+GRID_STEP*GRID_CELLS, pixels)
    x, y = np.meshgrid(xs, ys)
    rng = np.random.default_rng(scene['seed'])
    variation = (.028*np.sin(x/9)*np.cos(y/13)+.017*np.cos((x-y)/3)
                 + rng.normal(0, .018, (pixels,pixels)))
    colors = np.empty((pixels,pixels,4), dtype=np.float32)
    colors[:,:,:3] = np.array([.24,.285,.115]) + variation[:,:,None]
    shore = x > 119 + 4*np.sin(y/41)
    colors[shore,:3] = np.array([.48,.40,.25]) + variation[shore,None]
    track = np.abs(x - 15*np.sin(y/53) + 22) < 3
    colors[track,:3] = np.array([.32,.25,.15]) + variation[track,None]
    colors[:,:,3] = 1
    np.clip(colors, 0, 1, out=colors)
    image = bpy.data.images.new('terrain_albedo', width=pixels, height=pixels)
    image.pixels.foreach_set(colors.ravel())
    image.filepath_raw, image.file_format = str(mesh_dir/'terrain.png'), 'PNG'
    image.save()
    mat = material('terrain', (1,1,1), .96)
    node = mat.node_tree.nodes.new('ShaderNodeTexImage')
    node.image = image
    mat.node_tree.links.new(node.outputs['Color'], mat.node_tree.nodes.get('Principled BSDF').inputs['Base Color'])
    mesh.materials.append(mat)
    with (mesh_dir/'terrain.obj').open('w', encoding='utf-8', newline='\n') as f:
        f.write('# PURPOSE: generated Z-up metre terrain, shared visual and collision triangles.\n')
        for vx, vy, vz in vertices:
            f.write(f'v {vx:.6f} {vy:.6f} {vz:.6f}\n')
        for j in range(n):
            for i in range(n):
                f.write(f'vt {i/GRID_CELLS:.7f} {j/GRID_CELLS:.7f}\n')
        # Harmonic/DART requires a normal for every imported vertex. An OBJ without normals
        # is accepted by Blender but rejected by CustomMeshShape and can crash ODE collision.
        for vertex in mesh.vertices:
            normal=vertex.normal
            f.write(f'vn {normal.x:.7f} {normal.y:.7f} {normal.z:.7f}\n')
        for a,b,c in faces:
            f.write(f'f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}\n')
    return obj, len(faces)


def scenery(scene):
    """Trees, shoreline harbor, rescue scene and hard negatives; return measured hiker bounds."""
    bark = material('bark', (.12,.06,.027))
    metal = material('metal', (.15,.17,.18))
    foliage = [material(f'foliage{i}', c) for i,c in enumerate(
        ((.10,.18,.035),(.16,.23,.055),(.22,.26,.065),(.08,.14,.03)))]
    rng = random.Random(scene['seed']+3)
    for o in scene['objects']:
        x,y,z = o['x'],o['y'],o['z']
        name, kind = o['id'], o['kind']
        if kind == 'tree':
            h = o['height']
            rod(name+'_trunk', (x,y,z), (x,y,z+h*.68), .18, bark)
            for i in range(6):
                angle = i*math.tau/6 + o['yaw']
                tip = (x+math.cos(angle)*h*.19,y+math.sin(angle)*h*.19,z+h*(.6+.05*(i%3)))
                rod(name+'_branch', (x,y,z+h*.44), tip, .07, bark)
                crown = ellipsoid(name+'_leaves', tip, (h*.22,h*.19,h*.25), foliage[(i+o['variant'])%4])
                for v in crown.data.vertices:
                    v.co *= rng.uniform(.83,1.17)
        elif kind == 'bush':
            for i in range(4):
                s = o['scale']
                ellipsoid(name, (x+rng.uniform(-.4,.4)*s,y+rng.uniform(-.4,.4)*s,z+.65*s),
                          (.65*s,.6*s,.7*s),foliage[i%4])
        elif kind == 'rock':
            s = o['scale']
            obj = ellipsoid(name,(x,y,z+.2*s),(.6*s,.45*s,.35*s),material('rock',(.28,.28,.25)))
            for v in obj.data.vertices:
                v.co *= rng.uniform(.75,1.25)
        elif kind == 'log':
            rod(name,(x-1.3,y,z+.18),(x+1.3,y+.6,z+.18),.18,bark)
        elif kind == 'tarp':
            box(name,(x,y,z+.035),(1.8,1.0,.05),material('tarp',(.05,.18,.28)),o['yaw'],.02)
        elif kind == 'bystander':
            mannequin(o,name,standing=True)
        elif kind == 'suitcase':
            suitcase(name,o,o['color'])
        elif kind in ('building','cargo'):
            size = o['size']
            box(name,(x,y,z+size[2]/2),size,material('walls',(.48,.40,.29)),o['yaw'],.07)
            box(name+'_roof',(x,y,z+size[2]+.08),(size[0]+.4,size[1]+.4,.2),
                material('roof',(.16,.18,.17)),o['yaw'])
        elif kind == 'vehicle':
            box(name,(x,y,z+.65),(4.4,1.9,1.1),material(o['color'],COLORS[o['color']]),o['yaw'],.16)
            box(name+'_cab',(x,y,z+1.25),(2.3,1.7,.7),material('glass',(.06,.1,.14)),o['yaw'],.12)
        elif kind == 'wire':
            for side in (-1,1):
                rod(name+'_pole',(x+side*o['length']/2,y,z),(x+side*o['length']/2,y,z+o['height']),.13,bark)
            rod(name+'_cable',(x-o['length']/2,y,z+o['height']),
                (x+o['length']/2,y,z+o['height']),.025,metal)
    # Water and harbor are visual only: this scene cannot claim buoyancy, waves or specular-glare fidelity.
    water = material('water',(.03,.19,.23),.22)
    box('harbor_water',(197,60,scene['terrain']['sea_level_m']),(124,480,.04),water)
    timber = material('dock',(.29,.19,.09))
    box('dock',(153,35,.2),(36,4,.35),timber)
    for i in range(3):
        box(f'boat_{i}',(163+i*8,44,-.1),(6,2.4,1.2),material('boat_hull',(.75,.74,.67)),bevel=.45)
        box(f'boat_cabin_{i}',(163+i*8,44,.8),(2.8,2,1),metal,bevel=.15)
    box('takeoff_pad',(0,0,.015),(12.2,12.2,.03),material('concrete',(.55,.54,.49)))
    bbox = None
    if scene['hiker']:
        bbox = mannequin(scene['hiker'],'hiker')
        suitcase('hiker_suitcase',scene['hiker']['suitcase'],scene['hiker']['suitcase_color'])
    return bbox


def export_scenery(objects, path):
    """Export evaluated world-space triangles/normals and diffuse materials as standard COLLADA 1.4.1."""
    groups = defaultdict(lambda: [[],[],[]])
    deps = bpy.context.evaluated_depsgraph_get()
    for obj in objects:
        evaluated = obj.evaluated_get(deps)
        mesh = evaluated.to_mesh()
        mesh.calc_loop_triangles()
        matrix = obj.matrix_world
        normal_matrix = matrix.to_3x3().inverted().transposed()
        for triangle in mesh.loop_triangles:
            mat = mesh.materials[triangle.material_index]
            positions,normals,faces = groups[mat.name]
            base = len(positions)
            for vi in triangle.vertices:
                positions.append(tuple(matrix @ mesh.vertices[vi].co))
                normals.append(tuple((normal_matrix @ mesh.vertices[vi].normal).normalized()))
            faces.append((base,base+1,base+2))
        evaluated.to_mesh_clear()
    ET.register_namespace('', 'http://www.collada.org/2005/11/COLLADASchema')
    root = ET.Element('COLLADA',xmlns='http://www.collada.org/2005/11/COLLADASchema',version='1.4.1')
    asset=ET.SubElement(root,'asset')
    ET.SubElement(asset,'unit',name='meter',meter='1')
    ET.SubElement(asset,'up_axis').text='Z_UP'
    effects,materials,geometries,visuals = [ET.SubElement(root,tag) for tag in
        ('library_effects','library_materials','library_geometries','library_visual_scenes')]
    visual=ET.SubElement(visuals,'visual_scene',id='Scene',name='Scene')
    triangles=0
    for index,(name,(positions,normals,faces)) in enumerate(sorted(groups.items())):
        key=f'm{index}'
        effect=ET.SubElement(effects,'effect',id=key+'fx')
        profile=ET.SubElement(effect,'profile_COMMON')
        technique=ET.SubElement(profile,'technique',sid='common')
        phong=ET.SubElement(technique,'phong')
        for label,col in (('ambient',(.12,.12,.12,1)),('diffuse',MATERIALS[name].diffuse_color),
                          ('specular',(.08,.08,.08,1))):
            ET.SubElement(ET.SubElement(phong,label),'color').text=' '.join(f'{v:.6f}' for v in col)
        ET.SubElement(ET.SubElement(phong,'shininess'),'float').text='12'
        mat=ET.SubElement(materials,'material',id=key+'mat',name=name)
        ET.SubElement(mat,'instance_effect',url='#'+key+'fx')
        mesh=ET.SubElement(ET.SubElement(geometries,'geometry',id=key+'geo',name=name),'mesh')
        for label,values in (('pos',positions),('normal',normals)):
            source=ET.SubElement(mesh,'source',id=key+label)
            ET.SubElement(source,'float_array',id=key+label+'array',count=str(len(values)*3)).text=' '.join(
                f'{v:.6f}' for point in values for v in point)
            accessor=ET.SubElement(ET.SubElement(source,'technique_common'),'accessor',
                                   source='#'+key+label+'array',count=str(len(values)),stride='3')
            for axis in 'XYZ':
                ET.SubElement(accessor,'param',name=axis,type='float')
        vertex=ET.SubElement(mesh,'vertices',id=key+'vertices')
        ET.SubElement(vertex,'input',semantic='POSITION',source='#'+key+'pos')
        tris=ET.SubElement(mesh,'triangles',count=str(len(faces)),material=key+'symbol')
        ET.SubElement(tris,'input',semantic='VERTEX',source='#'+key+'vertices',offset='0')
        ET.SubElement(tris,'input',semantic='NORMAL',source='#'+key+'normal',offset='1')
        ET.SubElement(tris,'p').text=' '.join(f'{vi} {vi}' for face in faces for vi in face)
        instance=ET.SubElement(ET.SubElement(visual,'node',id=key+'node'),'instance_geometry',url='#'+key+'geo')
        common=ET.SubElement(ET.SubElement(instance,'bind_material'),'technique_common')
        ET.SubElement(common,'instance_material',symbol=key+'symbol',target='#'+key+'mat')
        triangles += len(faces)
    ET.SubElement(ET.SubElement(root,'scene'),'instance_visual_scene',url='#Scene')
    path.write_text('<?xml version="1.0"?>\n<!-- PURPOSE: generated scenery with portable diffuse materials. -->\n'
                    +ET.tostring(root,encoding='unicode')+'\n',encoding='utf-8')
    return triangles


def previews(scene, output):
    """Overview plus a camera frame using the simulated camera's 81-degree horizontal field of view."""
    render=bpy.context.scene
    # These are fast inspection views. Actual sensor appearance is measured in Ogre/Gazebo;
    # a multi-minute path-traced render is not needed to check metre scale and clothing colors.
    try:
        render.render.engine='BLENDER_EEVEE'
    except TypeError:
        render.render.engine='BLENDER_EEVEE_NEXT'
    render.view_settings.view_transform='Standard'
    bpy.ops.object.light_add(type='SUN',location=(0,0,100))
    sun=bpy.context.object
    sun.data.energy=2.2
    el=math.radians(scene['lighting']['elevation_deg'])
    az=math.radians(scene['lighting']['azimuth_deg'])
    sun.rotation_euler=Vector((-math.cos(el)*math.sin(az),-math.cos(el)*math.cos(az),-math.sin(el))).to_track_quat('-Z','Y').to_euler()
    render.world.color=(.25,.28,.32)
    bpy.ops.object.camera_add()
    cam=bpy.context.object
    render.camera=cam
    cam.data.lens_unit='FOV'
    cam.data.sensor_fit='HORIZONTAL'
    cam.data.angle=math.radians(81)
    hiker=scene['hiker'] or dict(x=87,y=124,z=3)
    for name,eye,target,width in (
        ('overview',(-150,-115,210),(40,100,0),1280),
        ('hiker_50m',(hiker['x'],hiker['y'],hiker['z']+50),(hiker['x'],hiker['y'],hiker['z']),2560),
        ('hiker_detail',(hiker['x']+2,hiker['y']-3,hiker['z']+5),(hiker['x'],hiker['y'],hiker['z']+.2),1024)):
        cam.location=eye
        cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler()
        render.render.resolution_x,render.render.resolution_y=width,width*9//16
        render.render.resolution_percentage=100
        render.render.filepath=str(output/f'{name}.png')
        bpy.ops.render.render(write_still=True)


def main():
    """Read the recipe, build/export assets, save an editable .blend and optional previews."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene',required=True,type=Path)
    parser.add_argument('--out-dir',required=True,type=Path)
    parser.add_argument('--preview',action='store_true')
    args=parser.parse_args(sys.argv[sys.argv.index('--')+1:])
    scene=json.loads(args.scene.read_text(encoding='utf-8'))
    COLORS.update(scene['palette'])
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    mesh_dir=args.out_dir/'models'/scene['world_name']/'meshes'
    mesh_dir.mkdir(parents=True,exist_ok=True)
    terrain,terrain_triangles=make_terrain(scene,mesh_dir)
    bbox=scenery(scene)
    bpy.context.view_layer.update()
    objects=[o for o in bpy.context.scene.objects if o.type=='MESH' and o!=terrain]
    scenery_triangles=export_scenery(objects,mesh_dir/'scenery.dae')
    metadata=dict(schema='astra.suas2027.assets.v1',blender_version=bpy.app.version_string,
                  terrain_triangles=terrain_triangles,scenery_triangles=scenery_triangles,
                  total_triangles=terrain_triangles+scenery_triangles,hiker_bbox=bbox,
                  license='Original procedural assets; generated from repository code, no source imagery',
                  materials={name:list(mat.diffuse_color) for name,mat in MATERIALS.items()})
    (args.out_dir/'assets.json').write_text(json.dumps(metadata,indent=2)+'\n',encoding='utf-8')
    if metadata['total_triangles']>500000:
        raise RuntimeError('scene exceeds the 500000-triangle training budget')
    bpy.ops.wm.save_as_mainfile(filepath=str(args.out_dir/'scene.blend'))
    if args.preview:
        previews(scene,args.out_dir)
        bpy.ops.wm.save_as_mainfile(filepath=str(args.out_dir/'scene.blend'))
    print(json.dumps({k:v for k,v in metadata.items() if k!='materials'}))


if __name__=='__main__':
    main()
