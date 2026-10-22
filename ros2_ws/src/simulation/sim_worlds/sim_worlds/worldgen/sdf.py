"""
sim_worlds/worldgen/sdf.py -- PURPOSE: write the SDF text of static objects from simple "parts".

A part is ONE shape (box, cylinder, capsule, sphere, ellipsoid) with a colour, or ONE mesh file (a local
model:// file or a pinned Gazebo Fuel file URI) with an optional material. An object is a list of parts in its
own frame (x forward, y left, z up, ground at z = 0). model_xml() places the object in the world.

Also here: tiny rotation helpers (roll-pitch-yaw <-> 3x3 matrix, SDF's fixed-axis convention R = Rz*Ry*Rx)
and bounding-box helpers, so generated objects can be rotated and set down exactly on the ground.
"""

import math

import numpy as np


# ---------------------------------------------------------------------------------------------------------------
# rotations (SDF pose convention: roll about x, then pitch about y, then yaw about z, all fixed axes)
# ---------------------------------------------------------------------------------------------------------------

def rpy_to_matrix(roll, pitch, yaw):
    """3x3 rotation matrix of an SDF roll-pitch-yaw triple (radians): R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(m):
    """Inverse of rpy_to_matrix(): returns (roll, pitch, yaw) in radians (pitch in [-pi/2, pi/2])."""
    m = np.asarray(m, dtype=float)
    sp = -m[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) < 1.0 - 1e-9:
        roll = math.atan2(m[2, 1], m[2, 2])
        yaw = math.atan2(m[1, 0], m[0, 0])
    else:                                    # gimbal lock: put everything into yaw
        roll = 0.0
        yaw = math.atan2(-m[0, 1], m[1, 1])
    return roll, pitch, yaw


def pose_str(xyz, rpy=(0.0, 0.0, 0.0)):
    """'x y z roll pitch yaw' text with 4 decimals."""
    return (f'{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f} '
            f'{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}')


def xml_comment(text):
    """Make text safe inside <!-- ... -->: XML forbids "--" inside a comment (strict parsers reject the file)."""
    text = str(text)
    while '--' in text:
        text = text.replace('--', '-')
    return text.rstrip('-')


# ---------------------------------------------------------------------------------------------------------------
# parts
# ---------------------------------------------------------------------------------------------------------------

def part(name, shape, size=(0.0,), xyz=(0, 0, 0), rpy=(0, 0, 0), color=(0.5, 0.5, 0.5), collision=False,
         roughness=0.8, double_sided=False, cast_shadows=True, uri=None, albedo=None, scale=None, submesh=None,
         material='color', normal=None, roughness_map=None, metalness_map=None, visual=True):
    """Describe one part (returns a dict).

    shape: 'box' (size = sx, sy, sz), 'cylinder' / 'capsule' (size = radius, length), 'sphere' (radius,),
           'ellipsoid' (rx, ry, rz) or 'mesh' (uri = file URI, optional scale (sx, sy, sz), submesh name).
    xyz, rpy: pose inside the object [m, rad]. color: (r, g, b) 0..1. collision: also add a <collision>.
    material: 'color' (use color, optional albedo/normal/roughness/metalness maps), or None = no <material>
              element at all (a mesh then keeps the colours and textures stored inside the mesh file).
    visual: False = collision only (an invisible box around a detailed object, cheap physics).
    """
    return {'name': name, 'shape': shape, 'size': tuple(float(v) for v in size),
            'xyz': tuple(float(v) for v in xyz), 'rpy': tuple(float(v) for v in rpy), 'color': tuple(color),
            'collision': collision, 'roughness': roughness, 'double_sided': double_sided,
            'cast_shadows': cast_shadows, 'uri': uri, 'albedo': albedo, 'scale': scale, 'submesh': submesh,
            'material': material, 'normal': normal, 'roughness_map': roughness_map,
            'metalness_map': metalness_map, 'visual': visual}


def collision_box(name, size, xyz):
    """An invisible collision box (physics only): one simple box around an object made of many visuals."""
    return part(name, 'box', size, xyz, collision=True, visual=False)


def _geometry_xml(p, ind):
    s = p['size']
    sh = p['shape']
    if sh == 'box':
        return f'{ind}<box><size>{s[0]:.3f} {s[1]:.3f} {s[2]:.3f}</size></box>'
    if sh == 'cylinder':
        return f'{ind}<cylinder><radius>{s[0]:.3f}</radius><length>{s[1]:.3f}</length></cylinder>'
    if sh == 'capsule':
        return f'{ind}<capsule><radius>{s[0]:.3f}</radius><length>{s[1]:.3f}</length></capsule>'
    if sh == 'sphere':
        return f'{ind}<sphere><radius>{s[0]:.3f}</radius></sphere>'
    if sh == 'ellipsoid':
        return f'{ind}<ellipsoid><radii>{s[0]:.3f} {s[1]:.3f} {s[2]:.3f}</radii></ellipsoid>'
    if sh == 'mesh':
        sc = ''
        if p['scale']:
            sc = '<scale>' + ' '.join(f'{v:.5f}' for v in p['scale']) + '</scale>'
        sub = f'<submesh><name>{p["submesh"]}</name></submesh>' if p['submesh'] else ''
        return f'{ind}<mesh><uri>{p["uri"]}</uri>{sub}{sc}</mesh>'
    raise ValueError(f'unknown shape {sh}')


def material_xml(p, ind):
    """<material> text of a part (empty string when the part keeps the mesh's own material)."""
    if p['material'] is None:
        return ''
    r, g, b = p['color']
    out = [f'{ind}<material>']
    if p['double_sided']:
        out.append(f'{ind}  <double_sided>true</double_sided>')
    out += [f'{ind}  <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>',
            f'{ind}  <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>',
            f'{ind}  <specular>0.12 0.12 0.12 1</specular>',
            f'{ind}  <pbr><metal>']
    if p['albedo']:
        out.append(f'{ind}    <albedo_map>{p["albedo"]}</albedo_map>')
    if p['normal']:
        out.append(f'{ind}    <normal_map>{p["normal"]}</normal_map>')
    if p['roughness_map']:
        out.append(f'{ind}    <roughness_map>{p["roughness_map"]}</roughness_map>')
    if p['metalness_map']:
        out.append(f'{ind}    <metalness_map>{p["metalness_map"]}</metalness_map>')
    out += [f'{ind}    <roughness>{p["roughness"]:.2f}</roughness><metalness>0.0</metalness>',
            f'{ind}  </metal></pbr>',
            f'{ind}</material>']
    return '\n'.join(out)


def part_xml(p, ind='        '):
    """<collision> (if wanted) and <visual> (unless hidden) text of one part."""
    out = []
    pose = f'{ind}  <pose>{pose_str(p["xyz"], p["rpy"])}</pose>'
    if p['collision']:
        out += [f'{ind}<collision name="{p["name"]}_col">', pose, f'{ind}  <geometry>',
                _geometry_xml(p, ind + '    '), f'{ind}  </geometry>', f'{ind}</collision>']
    if p['visual']:
        out += [f'{ind}<visual name="{p["name"]}">', pose]
        if not p['cast_shadows']:
            out.append(f'{ind}  <cast_shadows>false</cast_shadows>')
        out += [f'{ind}  <geometry>', _geometry_xml(p, ind + '    '), f'{ind}  </geometry>']
        mat = material_xml(p, ind + '  ')
        if mat:
            out.append(mat)
        out.append(f'{ind}</visual>')
    return '\n'.join(out)


def model_xml(name, parts, xyz=(0, 0, 0), yaw=0.0, rpy=None, comment=None):
    """A complete STATIC <model> holding all parts in one link, placed at xyz with heading yaw (radians)."""
    rpy = rpy if rpy is not None else (0.0, 0.0, yaw)
    lines = []
    if comment:
        lines.append(f'    <!-- {xml_comment(comment)} -->')
    lines += [f'    <model name="{name}">', '      <static>true</static>',
              f'      <pose>{pose_str(xyz, rpy)}</pose>', '      <link name="link">']
    lines += [part_xml(p) for p in parts]
    lines += ['      </link>', '    </model>']
    return '\n'.join(lines)


def include_xml(name, uri, xyz, rpy=(0, 0, 0)):
    """An <include> of a model by URI (model://... or a Fuel URL), forced static."""
    return (f'    <include>\n      <name>{name}</name>\n      <uri>{uri}</uri>\n      <static>true</static>\n'
            f'      <pose>{pose_str(xyz, rpy)}</pose>\n    </include>')


# ---------------------------------------------------------------------------------------------------------------
# moving whole objects
# ---------------------------------------------------------------------------------------------------------------

def transform_parts(parts, rot=None, shift=(0.0, 0.0, 0.0)):
    """Rotate every part about the object origin by the 3x3 matrix `rot` (None = no rotation), then shift."""
    rot = np.eye(3) if rot is None else np.asarray(rot, dtype=float)
    out = []
    for p in parts:
        c = rot @ np.array(p['xyz']) + np.asarray(shift, dtype=float)
        r = matrix_to_rpy(rot @ rpy_to_matrix(*p['rpy']))
        q = dict(p)
        q['xyz'] = tuple(float(v) for v in c)
        q['rpy'] = tuple(float(v) for v in r)
        out.append(q)
    return out


def _part_points(p):
    """A few surface points of a primitive part (object frame), for bounding boxes."""
    rot = rpy_to_matrix(*p['rpy'])
    c = np.array(p['xyz'])
    s = p['size']
    sh = p['shape']
    if sh == 'box':
        pts = [np.array([a * s[0] / 2, b * s[1] / 2, d * s[2] / 2]) for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)]
    elif sh in ('cylinder', 'capsule'):
        r, length = s
        half = length / 2 + (r if sh == 'capsule' else 0.0)
        pts = [np.array([r * math.cos(t), r * math.sin(t), ez]) for ez in (-half, half)
               for t in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    elif sh == 'sphere':
        r = s[0]
        pts = [np.array(v) * r for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    elif sh == 'ellipsoid':
        pts = [np.array(v) * np.array(s) for v in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    else:
        pts = [np.zeros(3)]
    return [rot @ q + c for q in pts]


def parts_bbox(parts):
    """(min_xyz, max_xyz) numpy arrays of the primitive parts (meshes count as a point at their origin)."""
    pts = np.array([q for p in parts for q in _part_points(p)])
    return pts.min(axis=0), pts.max(axis=0)


def rest_on_ground(parts, clearance=0.0):
    """Shift primitive parts up/down so their lowest point sits at z = clearance."""
    lo, _ = parts_bbox(parts)
    return transform_parts(parts, None, (0.0, 0.0, clearance - lo[2]))
