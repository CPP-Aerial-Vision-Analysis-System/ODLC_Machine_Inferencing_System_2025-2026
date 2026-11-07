"""PURPOSE: Run inside Blender to catch deferred-transform scale bugs and preserve human-size geometry.

Separate from pytest because ordinary ROS Python has no bpy. Run Blender in background with --python
pointing to this file and --python-exit-code 1. This creates no scene output or simulator processes.
"""

from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import bpy

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import blender_suas_2027 as generator  # noqa: E402


def main():
    """Assert physical scale after actual Blender transforms and inspect the exported mesh contract."""
    generator.COLORS.update(red=(.65,.025,.018), blue=(.025,.12,.65))
    for pose in ('supine', 'prone', 'seated', 'standing'):
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.object.delete(use_global=False)
        person=dict(x=84, y=123, z=3.4, yaw=.7, pose=pose, shirt_color='red', pants_color='blue')
        bounds=generator.mannequin(person, 'scale_probe', standing=pose=='standing')
        height=bounds['size'][2]
        if pose in ('supine','prone'):
            assert .2 < height < .5, (pose,bounds)
        elif pose=='seated':
            assert 1.1 < height < 1.6, (pose,bounds)
        else:
            assert 1.6 < height < 2, (pose,bounds)
        assert max(bounds['size'][:2]) < 2.3, (pose,bounds)
        assert abs(bounds['center'][2]-height/2-person['z']-.025)<1e-5
        assert len([o for o in bpy.context.scene.objects if 'boot' in o.name])==2
        with tempfile.TemporaryDirectory(prefix='astra_blender_smoke_') as directory:
            target=Path(directory)/'person.dae'
            count=generator.export_scenery([o for o in bpy.context.scene.objects if o.type=='MESH'],target)
            root=ET.parse(target).getroot()
            ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
            assert root.find('c:asset/c:up_axis',ns).text=='Z_UP'
            assert count > 100
            names={mat.get('name') for mat in root.findall('c:library_materials/c:material',ns)}
            assert {'shirt_red','pants_blue'} <= names
        print(f'[blender_geometry_smoke] {pose}: {bounds["size"]}',flush=True)
    print('[blender_geometry_smoke] PASS: four human poses and COLLADA material/scale contracts',flush=True)
    with tempfile.TemporaryDirectory(prefix='astra_blender_terrain_') as directory:
        recipe=dict(seed=2027,terrain=dict(heights=[[0,1],[2,3]],origin=[-1,-1],step_m=3))
        _,triangles=generator.make_terrain(recipe,Path(directory))
        lines=(Path(directory)/'terrain.obj').read_text().splitlines()
        vertices=[line for line in lines if line.startswith('v ')]
        normals=[line for line in lines if line.startswith('vn ')]
        assert len(vertices)==len(normals)==4
        assert triangles==2
        for line in lines:
            if line.startswith('f '):
                assert all(len(item.split('/'))==3 for item in line.split()[1:])
        for line in normals:
            values=list(map(float,line.split()[1:]))
            assert abs(sum(v*v for v in values)-1)<1e-5
            assert values[2]>0
    print('[blender_geometry_smoke] PASS: terrain OBJ has unit vertex normals for DART collision',flush=True)


if __name__=='__main__':
    main()
