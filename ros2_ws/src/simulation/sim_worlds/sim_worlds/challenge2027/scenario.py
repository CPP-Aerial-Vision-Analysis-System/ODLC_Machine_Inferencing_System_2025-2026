"""PURPOSE: Describe reproducible 2027 reconnaissance challenges independently of Blender and ROS.

The owner's Primer & Task Ideas is a preliminary brief, not a complete rulebook. Landing-zone
dimensions, slopes, terrain and distractors below are explicitly local training choices. ENU metres
and radians are used throughout. Never feed this answer key to a detector or mission brain.
"""

import math
import random

SCHEMA = 'astra.suas2027.scene.v1'
CASES = ('baseline', 'occluded', 'low_contrast', 'low_sun', 'negative')
COLORS = {
    'red': (0.65, 0.025, 0.018), 'blue': (0.025, 0.12, 0.65),
    'yellow': (0.85, 0.65, 0.025), 'orange': (0.9, 0.18, 0.018),
    'green': (0.12, 0.23, 0.055), 'black': (0.018, 0.023, 0.027),
    'white': (0.85, 0.83, 0.76), 'brown': (0.20, 0.105, 0.045),
}
GRID_MIN = (-220.0, -180.0)
GRID_STEP = 3.0
GRID_CELLS = 160
SEA_LEVEL = -0.65
ZONES = (
    ('blocked', 67.0, 110.0, 'obstruction'),
    ('slope', 108.0, 95.0, 'slope'),
    ('wires', 115.0, 150.0, 'overhead_wire'),
    ('clearing_near', 54.0, 158.0, None),
    ('clearing_far', -30.0, 180.0, None),
    ('water', 139.0, 195.0, 'water'),
)


def _smooth(t):
    """Blend a bounded transition with zero derivative at both ends."""
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def raw_height(x, y, seed):
    """Rolling synthetic coastal ground, with a shallow erosion channel and submerged eastern shelf."""
    p = (seed % 997) / 997.0 * math.tau
    h = (3.1 + 2.2 * math.sin(x / 58 + p) * math.cos(y / 74)
         + 1.1 * math.sin((x + y) / 33 + p) + 0.32 * math.cos(x / 9) * math.sin(y / 11))
    h -= 1.0 * math.exp(-((x + 28 - 13 * math.sin(y / 49)) / 9) ** 2)
    shore = 128 + 4 * math.sin(y / 41)
    return h * (1 - _smooth((x - shore + 6) / 14)) - 2.6 * _smooth((x - shore + 6) / 14)


def designed_height(x, y, seed):
    """Make the launch pad and chosen clearings flat; one candidate has an intentional eight-degree slope."""
    h = raw_height(x, y, seed)
    h *= _smooth((math.hypot(x, y) - 13) / 10)
    for _, cx, cy, reason in ZONES:
        if reason == 'water':
            continue
        distance = math.hypot(x - cx, y - cy)
        influence = 1 - _smooth((distance - 12) / 5)
        if influence:
            level = raw_height(cx, cy, seed)
            if reason == 'slope':
                level += (x - cx) * math.tan(math.radians(8))
            h = h * (1 - influence) + level * influence
    return h


def terrain_grid(seed):
    """Grid shared by Blender's visual and Gazebo's triangle-mesh collision surface."""
    return [[designed_height(GRID_MIN[0] + i * GRID_STEP, GRID_MIN[1] + j * GRID_STEP, seed)
             for i in range(GRID_CELLS + 1)] for j in range(GRID_CELLS + 1)]


def surface_height(x, y, grid):
    """Interpolate the exact exported triangle (diagonal bottom-left to top-right), not a separate DEM."""
    u, v = (x - GRID_MIN[0]) / GRID_STEP, (y - GRID_MIN[1]) / GRID_STEP
    if not (0 <= u <= GRID_CELLS and 0 <= v <= GRID_CELLS):
        raise ValueError('point outside generated terrain')
    i, j = min(int(u), GRID_CELLS - 1), min(int(v), GRID_CELLS - 1)
    a, b = u - i, v - j
    h00, h10 = grid[j][i], grid[j][i + 1]
    h01, h11 = grid[j + 1][i], grid[j + 1][i + 1]
    if b <= a:
        return h00 * (1 - a) + h10 * (a - b) + h11 * b
    return h00 * (1 - b) + h11 * a + h01 * (b - a)


def make_scene(seed=2027, case='baseline'):
    """Return a complete seeded answer key and scene recipe; no external assets or network are needed."""
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= 2**31 - 1:
        raise ValueError('seed must be an integer from 0 to 2147483647')
    if case not in CASES:
        raise ValueError(f'unknown challenge case: {case}')
    rng = random.Random(seed)
    grid = terrain_grid(seed)

    def z(x, y):
        return surface_height(x, y, grid)

    shirt, pants, suitcase = rng.sample(list(COLORS), 3)
    if case == 'low_contrast':
        shirt, pants, suitcase = 'green', 'brown', 'black'
    hx, hy = 87 + rng.uniform(-3, 3), 124 + rng.uniform(-3, 3)
    hiker = dict(id='hiker', x=hx, y=hy, z=z(hx, hy), yaw=rng.uniform(-math.pi, math.pi),
                 pose=rng.choice(('supine', 'prone', 'seated')), shirt_color=shirt,
                 pants_color=pants, suitcase_color=suitcase)
    sx, sy = hx + 1.5, hy - 0.8
    hiker['suitcase'] = dict(x=sx, y=sy, z=z(sx, sy), yaw=rng.uniform(-math.pi, math.pi))
    zones = []
    for name, x, y, reason in ZONES:
        zones.append(dict(id=name, x=x, y=y, z=z(x, y), radius_m=10.0,
                          valid=reason is None, reasons=[] if reason is None else [reason]))
    objects = []

    def add(kind, x, y, **extra):
        objects.append(dict(id=f'{kind}_{len(objects):03d}', kind=kind, x=x, y=y,
                            z=z(x, y), yaw=rng.uniform(-math.pi, math.pi), **extra))

    # Clear candidate disks, the hiker and takeoff area remain interpretable across all seeds.
    for _ in range(105):
        for _attempt in range(100):
            x, y = rng.uniform(-150, 125), rng.uniform(-100, 255)
            if math.hypot(x, y) < 27 or math.hypot(x - hx, y - hy) < 9:
                continue
            if any(math.hypot(x - c['x'], y - c['y']) < 22 for c in zones):
                continue
            add('tree', x, y, height=rng.uniform(4.5, 12.0), variant=rng.randrange(4))
            break
    for _ in range(22):
        x, y = rng.uniform(-65, 117), rng.uniform(35, 205)
        if math.hypot(x - hx, y - hy) < 5 or any(math.hypot(x-c['x'], y-c['y']) < 16 for c in zones):
            continue
        add(rng.choice(('rock', 'bush', 'log', 'tarp')), x, y, scale=rng.uniform(0.6, 1.8))
    for x, y in ((-58, 58), (-24, 65), (12, 46)):
        add('bystander', x, y, shirt_color=rng.choice(list(COLORS)), pants_color=rng.choice(list(COLORS)))
        add('suitcase', x + 2, y, color=rng.choice(list(COLORS)))
    for x, y in ((-85, -75), (-65, -75), (-85, -50), (-38, -66)):
        add('building', x, y, size=[rng.uniform(8, 13), rng.uniform(7, 11), rng.uniform(4, 7)])
    add('cargo', 67, 110, size=[3.0, 2.0, 1.8])
    add('wire', 115, 150, length=26.0, height=9.0)
    add('vehicle', -16, 78, color='white')
    add('vehicle', 25, 200, color='red')
    if case == 'occluded':
        add('bush', hx + 0.45, hy + 0.5, scale=0.9)
    return dict(schema=SCHEMA, world_name=f'suas2027_{case}_{seed}', seed=seed, case=case,
                source=dict(title='SUAS 2027 Primer & Task Ideas', pages=[6, 17, 18],
                            status='preliminary; final handbook and report schema not supplied'),
                coordinates='metres ENU; z above synthetic launch datum; yaw radians', palette=COLORS,
                home=dict(lat=36.2143, lon=-96.0068, alt_msl=210.0, heading=0),
                site='fictional coastal training scene; not a survey of Skyway Range',
                search_area=dict(name='reconnaissance', center=[35.0, 120.0], size=[220.0, 184.0], yaw=0),
                terrain=dict(origin=list(GRID_MIN), step_m=GRID_STEP, heights=grid, sea_level_m=SEA_LEVEL),
                hiker=None if case == 'negative' else hiker, landing_zones=zones, objects=objects,
                training_lz_policy=dict(radius_m=10.0, max_slope_deg=5.0,
                                        note='local training assumptions; no official LZ criteria supplied'),
                delivery=dict(min_agl_m=45.72, target_radius_m=15.24, max_mass_kg=1.36077711,
                              max_size_m=[0.2794, 0.2286, 0.1016],
                              retardant='required by primer; not simulated by this static world'),
                lighting=dict(elevation_deg=12 if case == 'low_sun' else 52, azimuth_deg=135),
                limitations=['procedural mannequin and scenery, not photogrammetry',
                             'static water, vegetation and bystanders',
                             'no weather, moving aircraft, retardant physics or map-quality score',
                             'occlusion and contrast are designed challenges, not measured visibility labels'])
