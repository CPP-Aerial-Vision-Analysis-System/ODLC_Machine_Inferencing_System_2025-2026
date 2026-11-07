"""PURPOSE: Package Blender geometry as an external Gazebo world, mission and separate 2027 answer key.

Assets stay outside the source tree and use model:// URIs so Windows-generated meshes also load in WSL.
The legacy 2026 scorer and controller are not a 2027 mission implementation; this bundle is a sandbox.
"""

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

from sim_worlds import geo, mission, planner, survey
from sim_worlds.worldgen import sdf, world


def write_json(path, value):
    """Write portable metadata, rejecting non-finite JSON numbers."""
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def _yaml(path, value, purpose):
    """Write a purpose header and a portable YAML document."""
    Path(path).write_text(f'# PURPOSE: {purpose}\n' + yaml.safe_dump(value, sort_keys=False), encoding='utf-8')


def write_bundle(scene, output):
    """Assemble previously exported meshes. Return relative output names without local paths."""
    output = Path(output)
    name = scene['world_name']
    model = output / 'models' / name
    for asset in ('terrain.obj', 'terrain.png', 'scenery.dae'):
        if not (model / 'meshes' / asset).is_file():
            raise ValueError(f'Blender has not exported {asset}')
    for directory in ('worlds', 'config', 'truth', 'missions'):
        (output / directory).mkdir(parents=True, exist_ok=True)
    h = scene['home']
    home = geo.Home(h['lat'], h['lon'], h['alt_msl'], h['heading'])
    uri = f'model://{name}/meshes/'
    parts = [sdf.part('terrain', 'mesh', uri=uri + 'terrain.obj', collision=True,
                      color=(1, 1, 1), albedo=uri + 'terrain.png', roughness=0.96),
             sdf.part('scenery', 'mesh', uri=uri + 'scenery.dae', material=None)]
    # Tree trunks and large solid obstacles have cheap collision shapes; crowns remain visual only.
    for obj in scene['objects']:
        kind = obj['kind']
        x, y, z = obj['x'], obj['y'], obj['z']
        if kind == 'tree':
            height = obj['height'] * 0.6
            parts.append(sdf.part(obj['id'], 'cylinder', (0.18, height),
                                  (x, y, z + height / 2), collision=True, visual=False))
        elif kind in ('building', 'cargo', 'vehicle'):
            size = obj.get('size', [4.4, 1.9, 1.6])
            parts.append(sdf.part(obj['id'], 'box', size, (x, y, z + size[2] / 2),
                                  (0, 0, obj['yaw']), collision=True, visual=False))
        elif kind == 'wire':
            # Match Blender's poles and cable exactly; thin wires are visual + geometric training hazards.
            for sign in (-1, 1):
                parts.append(sdf.part(f'{obj["id"]}_pole_{sign}', 'cylinder', (0.13, obj['height']),
                                      (x + sign * obj['length'] / 2, y, z + obj['height'] / 2),
                                      collision=True, visual=False))
            parts.append(sdf.part(f'{obj["id"]}_cable', 'cylinder', (0.025, obj['length']),
                                  (x, y, z + obj['height']), (0, math.pi / 2, 0),
                                  collision=True, visual=False))
    link = '\n'.join(sdf.part_xml(p) for p in parts)
    (model / 'model.sdf').write_text(
        '<?xml version="1.0"?>\n<!-- PURPOSE: Blender terrain with matching mesh collision. -->\n'
        f'<sdf version="1.9"><model name="{name}"><static>true</static><link name="site">'
        f'{link}</link></model></sdf>\n', encoding='utf-8')
    (model / 'model.config').write_text(
        '<?xml version="1.0"?>\n<!-- PURPOSE: Register the generated 2027 training model. -->\n'
        f'<model><name>{name}</name><version>1</version><sdf version="1.9">model.sdf</sdf>'
        '<description>Procedural training scene, no downloaded assets.</description></model>\n', encoding='utf-8')
    gui = world.gui_xml(world.look_at((-90, -100, 175), (35, 115, 0)))
    header = world.header(name, home, ['PURPOSE: opt-in fictional SUAS 2027 perception sandbox.',
                                     'Generated; no 2027 mission brain or official score is implemented.'], gui)
    text = header + f'<include><uri>model://{name}</uri></include>\n' + world.footer()
    root = ET.fromstring(text)
    light = root.find('world/light')
    elevation = math.radians(scene['lighting']['elevation_deg'])
    azimuth = math.radians(scene['lighting']['azimuth_deg'])
    light.find('direction').text = f'{-math.cos(elevation)*math.sin(azimuth)} {-math.cos(elevation)*math.cos(azimuth)} {-math.sin(elevation)}'
    (output / 'worlds' / f'{name}.sdf').write_text(
        '<?xml version="1.0"?>\n<!-- PURPOSE: generated 2027 training world; edit its generator. -->\n'
        + ET.tostring(root, encoding='unicode') + '\n', encoding='utf-8')
    area = scene['search_area']
    rect = planner.rect_from_area(area)
    grid = scene['terrain']['heights']
    ground_max = max(max(row) for row in grid)
    altitude = math.ceil(ground_max + 50.0)
    settings = planner.SurveySettings(altitude_m=altitude, sidelap=0.7, frontlap=0.8,
                                      photo_interval_s=1.0, speed_mps=5.0, lead_in_s=16.0)
    # Plan overlap at the SMALLEST local clearance, while all MAVLink items retain above-home altitude.
    conservative_height = altitude - ground_max
    local_plan = survey.plan_lanes(rect, conservative_height, settings.sidelap, settings.hfov_rad,
                                   settings.width_px, settings.height_px)
    xy = list(local_plan.waypoints)
    xy[0] = planner.lead_in_point(xy[0], xy[1], settings.lead_in_s * settings.speed_mps)
    items = mission.build_survey_mission(home, [geo.enu_to_latlon(x, y, home) for x, y in xy],
                                         altitude, geo.enu_to_latlon(rect.cx, rect.cy, home), settings.speed_mps)
    problems = mission.check_team_shape(items, alt_min=altitude, alt_max=altitude)
    if problems:
        raise ValueError(f'generated survey is invalid: {problems}')
    mission_file = f'missions/{name}_survey.waypoints'
    mission.write_waypoints(str(output / mission_file), items)
    targets = []
    target = scene['hiker']
    if target:
        lat, lon = geo.enu_to_latlon(target['x'], target['y'], home)
        measured = json.loads((output / 'assets.json').read_text(encoding='utf-8'))['hiker_bbox']
        targets.append(dict(id='hiker', **{'class': 'person'}, class_id='0', area=area['name'],
                            x=target['x'], y=target['y'], z=target['z'], yaw=target['yaw'], lat=lat, lon=lon,
                            size=measured['size'], center=measured['center'], center_z=measured['center'][2],
                            pose=target['pose']))
        targets[-1]['yaw'] = measured['yaw']
    # Compatible position fields for visualization only; the legacy validator requires a tent and is not used.
    truth = dict(world_name=name, home=h, classes={'person': '0'}, targets=targets,
                 distractors=[], search_areas=[area], challenge_schema=scene['schema'],
                 note='2027 answer key is scene.json; legacy 2026 scores do not certify this scene')
    _yaml(output / 'truth' / f'{name}.yaml', truth, '2027 hiker visualization positions, not an official report.')
    config = dict(world_name=name, description='SUAS 2027 sandbox; 2026 demo is not a 2027 acceptance run',
                  world_file=f'worlds/{name}.sdf', home=home.sitl_string(),
                  spawn=dict(x=0.0, y=0.0, z=0.25, yaw=math.pi / 2), mission=mission_file,
                  truth=f'truth/{name}.yaml', search_areas=[area],
                  survey=dict(area=area['name'], altitude_m=altitude, sidelap=0.7, frontlap=0.8,
                              photo_interval_s=1.0, speed_mps=5.0, lead_in_s=16.0,
                              camera=dict(hfov_rad=settings.hfov_rad, width_px=2560, height_px=1440),
                              conservative_ground_max_m=ground_max,
                              note='lane spacing uses minimum local AGL; regenerated make_mission defaults do not'),
                  challenge=dict(schema=scene['schema'], answer_key='scene.json', official_rules=False))
    _yaml(output / 'config' / f'{name}.yaml', config, 'Opt-in 2027 Gazebo sandbox configuration.')
    write_json(output / 'scene.json', scene)
    write_json(output / 'task.json', dict(
        schema='astra.suas2027.task.v1', world_name=name, coordinates='local ENU metres',
        search_area=scene['search_area'],
        landing_zone_candidates=[{k: zone[k] for k in ('id', 'x', 'y', 'radius_m')}
                                 for zone in scene['landing_zones']],
        note='Local candidate-selection benchmark, not an official task/report schema.'))
    return dict(world=f'worlds/{name}.sdf', config=f'config/{name}.yaml', mission=mission_file,
                answer_key='scene.json', public_task='task.json', survey_altitude_above_home_m=altitude,
                minimum_survey_agl_m=conservative_height, search_area_acres=220 * 184 / 4046.8564224)
