"""
sim_worlds/worldgen/assets.py -- PURPOSE: the catalogue of realistic Gazebo Fuel models the worlds use, with
PINNED versions, exact placement on the ground and bounding boxes (for the truth files).

Why not just <include> the Fuel models?  Several of them are not usable as-is: some are not static, some lift
their link in the air, one pulls in an extra "Artifact Proximity Detector" and an old thermal plugin, and none
of them can lie down. So the worlds reference the Fuel MESH FILES directly (versioned URLs such as
https://fuel.gazebosim.org/1.0/openrobotics/models/rescue randy/3/files/meshes/rescue_randy.dae) inside our own
static models, with poses measured and checked in side-view renders (sim_worlds README, "Assets").

The first time a world loads, Gazebo downloads each model once into ~/.gz/fuel; scripts/fetch_models.sh does
that in advance (setup runs it) so the first start does not stall. If a model cannot be downloaded the world
still loads (that one object is missing and Gazebo prints an error); build the world with
`--assets primitive` for a fully offline copy made only of simple shapes.

Every model below is CC0 or CC-BY 4.0 (see LICENSE_NOTE); CC-BY ones are credited in the README.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from sim_worlds.worldgen import sdf

FUEL_SERVER = 'https://fuel.gazebosim.org/1.0'
LICENSE_NOTE = 'CC0 = public domain, CC-BY 4.0 = free to use with credit to the author'


@dataclass(frozen=True)
class FuelModel:
    """One Fuel model at a pinned version.

    owner/name: as shown on fuel.gazebosim.org (e.g. 'OpenRobotics', 'Rescue Randy'). version: pinned integer.
    license: 'CC0' or 'CC-BY 4.0'.
    """

    owner: str
    name: str
    version: int
    license: str

    @property
    def model_url(self):
        """URL of this model version (what `gz fuel download -u` takes)."""
        return f'{FUEL_SERVER}/{self.owner}/models/{self.name}/{self.version}'

    def file_url(self, rel_path):
        """URL of one file inside the model, lower-case owner/name exactly like Fuel's own SDF files use."""
        return f'{FUEL_SERVER}/{self.owner.lower()}/models/{self.name.lower()}/{self.version}/files/{rel_path}'


# --- the Fuel models (pinned) -----------------------------------------------------------------------------------
RANDY = FuelModel('OpenRobotics', 'Rescue Randy', 3, 'CC-BY 4.0')
RANDY_SITTING = FuelModel('OpenRobotics', 'Rescue Randy Sitting', 8, 'CC0')
STANDING_PERSON = FuelModel('OpenRobotics', 'Standing person', 3, 'CC0')
WALKING_PERSON = FuelModel('OpenRobotics', 'Walking person', 3, 'CC0')
CASUAL_FEMALE = FuelModel('OpenRobotics', 'Casual female', 4, 'CC0')
FEMALE_VISITOR = FuelModel('OpenRobotics', 'FemaleVisitor', 2, 'CC-BY 4.0')
HATCHBACK_BLUE = FuelModel('OpenRobotics', 'Hatchback blue', 5, 'CC0')
SUV = FuelModel('OpenRobotics', 'SUV', 4, 'CC0')
PICKUP = FuelModel('OpenRobotics', 'Pickup', 4, 'CC0')
OAK = FuelModel('OpenRobotics', 'Oak tree', 7, 'CC0')
PINE = FuelModel('OpenRobotics', 'Pine Tree', 6, 'CC0')
CONE = FuelModel('OpenRobotics', 'Construction Cone', 3, 'CC0')
BARREL = FuelModel('OpenRobotics', 'Construction Barrel', 4, 'CC0')
DRUM = FuelModel('OpenRobotics', '55gal_Drum', 1, 'CC-BY 4.0')
CHAIR = FuelModel('OpenRobotics', 'foldable_chair', 3, 'CC-BY 4.0')
TABLE = FuelModel('OpenRobotics', 'foldable_table', 4, 'CC-BY 4.0')
CANOPY = FuelModel('OpenRobotics', 'tent', 4, 'CC-BY 4.0')      # a pop-up CANOPY (flight-line shade), not the target


@dataclass(frozen=True)
class Visual:
    """One mesh visual of an asset: file inside the Fuel model, pose in the asset frame, scale, optional
    submesh name and optional material maps (paths inside the same Fuel model). material=None keeps the
    colours/textures stored in the mesh file itself."""

    mesh: str
    xyz: tuple = (0.0, 0.0, 0.0)
    rpy: tuple = (0.0, 0.0, 0.0)
    scale: tuple = (1.0, 1.0, 1.0)
    submesh: str = None
    material: dict = None


@dataclass(frozen=True)
class Asset:
    """A placeable object made of Fuel meshes.

    key: our name. model: the FuelModel. visuals: Visual tuple. bbox_min/bbox_max: (x, y, z) bounding box of
    the object in its own frame (metres, after the visual poses, ground at z = 0), measured from the mesh files
    and checked in side-view renders. kind: what it is (mannequin, person, vehicle, tree, prop, canopy).
    detector_class: the class name the team's detector would give it ('person', 'tent' or 'object').
    """

    key: str
    model: FuelModel
    visuals: tuple
    bbox_min: tuple
    bbox_max: tuple
    kind: str
    detector_class: str = 'object'
    description: str = ''
    extra_models: tuple = field(default_factory=tuple)

    def size(self, scale=1.0):
        """Bounding-box size [sx, sy, sz] in metres at `scale` (a number or an (sx, sy, sz) tuple)."""
        s = scale3(scale)
        return [round((b - a) * k, 3) for a, b, k in zip(self.bbox_min, self.bbox_max, s)]

    def center(self, scale=1.0):
        """Bounding-box centre (x, y, z) in the asset frame at `scale`."""
        s = scale3(scale)
        return [round((a + b) / 2.0 * k, 3) for a, b, k in zip(self.bbox_min, self.bbox_max, s)]

    def radius(self, scale=1.0):
        """Radius of a circle (top view, around the asset origin) that contains the whole object."""
        s = scale3(scale)
        xs = (self.bbox_min[0] * s[0], self.bbox_max[0] * s[0])
        ys = (self.bbox_min[1] * s[1], self.bbox_max[1] * s[1])
        return max(math.hypot(x, y) for x in xs for y in ys)


def scale3(scale):
    """A scale given as one number or as (sx, sy, sz) -> (sx, sy, sz)."""
    if isinstance(scale, (int, float)):
        return (float(scale),) * 3
    s = tuple(float(v) for v in scale)
    if len(s) != 3:
        raise ValueError('scale must be a number or (sx, sy, sz)')
    return s


def _randy_pbr():
    return {'albedo': 'materials/textures/rescue_randy_albedo.png', 'normal': 'materials/textures/rescue_randy_normal.png',
            'roughness': 'materials/textures/rescue_randy_roughness.png',
            'metalness': 'materials/textures/rescue_randy_metalness.png'}


H = math.pi / 2.0
RANDY_MESH = 'meshes/rescue_randy.dae'

ASSETS = {
    # ---- the hiker (SUAS mannequin) in four poses -------------------------------------------------------------
    # Rescue Randy stands in a T-pose facing -y: arms along x (span 1.364 m), 0.357 m deep, 1.821 m tall.
    'hiker_standing': Asset('hiker_standing', RANDY, (Visual(RANDY_MESH, material=_randy_pbr()),),
                            (-0.682, -0.178, 0.0), (0.682, 0.179, 1.821), 'mannequin', 'person',
                            'Rescue Randy mannequin standing (hi-vis jacket)'),
    # roll -90 deg: face up, head towards +y, centred on the origin, back on the ground
    'hiker_lying_face_up': Asset('hiker_lying_face_up', RANDY,
                                 (Visual(RANDY_MESH, (0.0, -0.9105, 0.179), (-H, 0.0, 0.0), material=_randy_pbr()),),
                                 (-0.682, -0.9105, 0.0), (0.682, 0.9105, 0.357), 'mannequin', 'person',
                                 'Rescue Randy mannequin lying on its back, arms spread'),
    # roll +90 deg: face down, head towards -y
    'hiker_lying_face_down': Asset('hiker_lying_face_down', RANDY,
                                   (Visual(RANDY_MESH, (0.0, 0.9105, 0.178), (H, 0.0, 0.0), material=_randy_pbr()),),
                                   (-0.682, -0.9105, 0.0), (0.682, 0.9105, 0.357), 'mannequin', 'person',
                                   'Rescue Randy mannequin lying face down, arms spread'),
    # Rescue Randy Sitting: the Fuel model lowers its link by 0.623996 m; centred here, legs towards -y
    'hiker_sitting': Asset('hiker_sitting', RANDY_SITTING,
                           (Visual(RANDY_MESH, (0.0995, 0.205, -0.618)),),
                           (-0.3735, -0.509, 0.0), (0.3735, 0.509, 0.730), 'mannequin', 'person',
                           'Rescue Randy mannequin sitting on the ground, legs forward'),
    # ---- standing people (distractors: the detector calls them "person" too) ----------------------------------
    'standing_person': Asset('standing_person', STANDING_PERSON,
                             (Visual('meshes/standing.dae', (0.0, 0.0, 0.02), (0.04, 0.0, 0.0)),),
                             (-0.277, -0.252, 0.0), (0.267, 0.11, 1.93), 'person', 'person',
                             'man in a white T-shirt and jeans, standing'),
    'walking_person': Asset('walking_person', WALKING_PERSON,
                            (Visual('meshes/walking.dae', (0.0, 0.0, -0.02)),),
                            (-0.274, -0.405, 0.0), (0.272, 0.477, 1.88), 'person', 'person',
                            'man in a white T-shirt and jeans, walking'),
    'casual_female': Asset('casual_female', CASUAL_FEMALE,
                           (Visual('meshes/casual_female.dae', (0.0, 0.0, 0.02), (0.04, 0.0, 0.0)),),
                           (-0.288, -0.362, 0.0), (0.322, 0.209, 1.67), 'person', 'person',
                           'woman in a blue top and jeans, standing'),
    'female_visitor': Asset('female_visitor', FEMALE_VISITOR,
                            (Visual('meshes/FemaleVisitor.obj'),),
                            (-0.317, -0.338, 0.0), (0.234, 0.142, 1.635), 'person', 'person',
                            'woman in a red dress with a handbag, standing'),
    # ---- vehicles (centred, length along +x) -------------------------------------------------------------------
    # (the plain 'Hatchback' Fuel model is not used: its wheel textures do not resolve and render white)
    'hatchback_blue': Asset('hatchback_blue', HATCHBACK_BLUE,
                            (Visual('meshes/hatchback.obj', (-0.3435, 0.0, 0.011), (0.0, 0.0, H), (0.0254,) * 3),),
                            (-2.0, -1.07, 0.0), (2.0, 1.07, 1.568), 'vehicle', 'object', 'blue-grey hatchback car'),
    'suv': Asset('suv', SUV,
                 (Visual('meshes/suv.obj', (-0.067, -0.034, 0.008), (0.0, 0.0, -H), (0.06,) * 3),),
                 (-2.51, -1.302, 0.0), (2.51, 1.302, 2.157), 'vehicle', 'object', 'grey off-road SUV'),
    'pickup': Asset('pickup', PICKUP,
                    (Visual('meshes/pickup.dae', (0.12, 0.0, 0.007), (0.0, 0.0, -H)),),
                    (-2.832, -1.236, 0.0), (2.832, 1.236, 1.866), 'vehicle', 'object', 'white pickup truck'),
    # ---- trees (bushes are small oaks) ---------------------------------------------------------------------------
    'oak_tree': Asset('oak_tree', OAK,
                      (Visual('meshes/oak_tree.dae', submesh='Branch',
                              material={'albedo': 'materials/textures/branch_diffuse.png', 'double_sided': True}),
                       Visual('meshes/oak_tree.dae', submesh='Bark',
                              material={'albedo': 'materials/textures/bark_diffuse.png'})),
                      (-5.921, -4.186, 0.0), (4.69, 4.304, 6.528), 'tree', 'object', 'oak tree'),
    'pine_tree': Asset('pine_tree', PINE,
                       # tint: the needles render almost white with full brightness; darkened to cedar green
                       (Visual('meshes/pine_tree.dae', submesh='Branch',
                               material={'albedo': 'materials/textures/branch_2_diffuse.png', 'double_sided': True,
                                         'tint': (0.42, 0.50, 0.40)}),
                        Visual('meshes/pine_tree.dae', submesh='Bark',
                               material={'albedo': 'materials/textures/bark_diffuse.png'})),
                       (-1.586, -1.586, 0.0), (1.586, 1.586, 5.078), 'tree', 'object', 'pine / cedar tree'),
    # ---- props ---------------------------------------------------------------------------------------------------
    'cone': Asset('cone', CONE, (Visual('meshes/construction_cone.dae', scale=(6.5,) * 3),),
                  (-0.163, -0.163, 0.0), (0.163, 0.163, 0.708), 'prop', 'object', 'orange traffic cone (0.7 m)'),
    'barrel': Asset('barrel', BARREL, (Visual('meshes/construction_barrel.dae', scale=(0.65,) * 3),),
                    (-0.332, -0.332, 0.0), (0.332, 0.332, 1.008), 'prop', 'object', 'orange/white construction barrel'),
    'drum': Asset('drum', DRUM, (Visual('base_visual.glb'),),
                  (-0.297, -0.297, 0.0), (0.297, 0.297, 0.862), 'prop', 'object', 'blue 55-gallon drum'),
    # (the Fuel 'Cardboard box' is not used: its mesh asks for a texture file that does not resolve; the
    #  worlds use primitives.box_parts() cartons instead)
    'folding_chair': Asset('folding_chair', CHAIR, (Visual('mesh/foldable_chair.dae', scale=(0.1,) * 3),),
                           (-0.262, -0.198, 0.0), (0.262, 0.327, 0.805), 'prop', 'object', 'folding chair'),
    'folding_table': Asset('folding_table', TABLE, (Visual('mesh/foldable_table.dae'),),
                           (-0.914, -0.381, 0.0), (0.914, 0.381, 0.684), 'prop', 'object', 'folding table'),
    'canopy': Asset('canopy', CANOPY, (Visual('mesh/tent.dae'),),
                    (-1.5, -1.5, 0.0), (1.5, 1.5, 3.214), 'canopy', 'object',
                    'green 3 x 3 m pop-up canopy (flight-line shade, NOT the search target)'),
}

HIKER_POSES = ('lying_face_up', 'lying_face_down', 'sitting', 'standing')
PEOPLE = ('standing_person', 'walking_person', 'casual_female', 'female_visitor')
VEHICLES = ('hatchback_blue', 'suv', 'pickup')


def asset(key):
    """Look up an Asset by key. Raises KeyError listing the known keys."""
    if key not in ASSETS:
        raise KeyError(f'unknown asset "{key}"; known: {", ".join(sorted(ASSETS))}')
    return ASSETS[key]


def asset_parts(key, scale=1.0, collision=True, name_prefix='m'):
    """Parts (see worldgen.sdf.part) that draw asset `key` at `scale` (a number, or (sx, sy, sz) in the asset
    frame, e.g. (0.25, 0.25, 0.15) turns an oak tree into a low bush).

    Each Fuel mesh becomes one mesh part; with collision=True one invisible collision box around the whole
    object is added (cheap physics; the payload can land on it). Returns a list of part dicts.
    Non-uniform scales are only correct for visuals without a roll/pitch (true for every tree and prop).
    """
    a = asset(key)
    s3 = scale3(scale)
    parts = []
    for i, v in enumerate(a.visuals):
        mat = None
        albedo = normal = rough = metal = None
        double = False
        tint = (1.0, 1.0, 1.0)
        if v.material:
            mat = 'color'
            albedo = a.model.file_url(v.material['albedo']) if v.material.get('albedo') else None
            normal = a.model.file_url(v.material['normal']) if v.material.get('normal') else None
            rough = a.model.file_url(v.material['roughness']) if v.material.get('roughness') else None
            metal = a.model.file_url(v.material['metalness']) if v.material.get('metalness') else None
            double = bool(v.material.get('double_sided'))
            tint = tuple(v.material.get('tint', (1.0, 1.0, 1.0)))
        # the mesh scale is applied in the mesh frame; for a visual turned by yaw only, swap x/y when needed
        c, s = abs(math.cos(v.rpy[2])), abs(math.sin(v.rpy[2]))
        mesh_s = (s3[0] * c + s3[1] * s, s3[0] * s + s3[1] * c, s3[2])
        sc = tuple(vs * ms for vs, ms in zip(v.scale, mesh_s))
        parts.append(sdf.part(f'{name_prefix}{i}', 'mesh', xyz=tuple(cc * k for cc, k in zip(v.xyz, s3)), rpy=v.rpy,
                              color=tint, uri=a.model.file_url(v.mesh), scale=sc, submesh=v.submesh,
                              material=mat, albedo=albedo, normal=normal, roughness_map=rough, metalness_map=metal,
                              double_sided=double, roughness=0.85))
    if collision:
        parts.append(sdf.collision_box('box', a.size(scale), a.center(scale)))
    return parts


def fuel_models_used(keys):
    """Sorted unique FuelModels behind the given asset keys (what fetch_models.sh must download)."""
    models = {asset(k).model for k in keys}
    return sorted(models, key=lambda m: (m.owner, m.name))


def bbox_corners_world(key, x, y, yaw, scale=1.0):
    """The 4 top-view corners (world x, y) of an asset placed at (x, y) with heading yaw (radians)."""
    a = asset(key)
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for bx, by in ((a.bbox_min[0], a.bbox_min[1]), (a.bbox_max[0], a.bbox_min[1]),
                   (a.bbox_max[0], a.bbox_max[1]), (a.bbox_min[0], a.bbox_max[1])):
        bx, by = bx * scale, by * scale
        pts.append((x + bx * c - by * s, y + bx * s + by * c))
    return pts


def all_fuel_models():
    """Every FuelModel in the catalogue."""
    return fuel_models_used(list(ASSETS))


def _check_catalogue():
    """Sanity check used by the unit tests: every bbox is non-empty and starts at the ground."""
    problems = []
    for k, a in ASSETS.items():
        lo, hi = np.array(a.bbox_min), np.array(a.bbox_max)
        if not np.all(hi > lo):
            problems.append(f'{k}: empty bounding box')
        if abs(lo[2]) > 1e-6:
            problems.append(f'{k}: bounding box does not start at the ground (z min {lo[2]})')
    return problems
