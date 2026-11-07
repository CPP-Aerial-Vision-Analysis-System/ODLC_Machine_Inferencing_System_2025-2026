#!/usr/bin/env python3
"""PURPOSE: Visibly load an external 2027 bundle, capture actual Gazebo camera images and drop ground probes.

This bounded integration check starts its own Gazebo partition/server/window and stops only those
processes. It does not fly the drone, evaluate a detector or certify a 2027 mission. Serialize it with
all other heavy jobs. Outputs go in the bundle's checks/ directory, outside the public repository.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

HERE = Path(__file__).resolve().parent
if (HERE.parent / 'sim_worlds').is_dir():
    sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))

from render_preview import camera_model_sdf, image_to_array, parse_camera  # noqa: E402
from sim_worlds.challenge2027.scenario import surface_height  # noqa: E402
from sim_worlds.gzrun import HeadlessGazebo, inject  # noqa: E402


def drop_probe(name, x, y, height):
    """A 20 cm radius sphere whose center should settle 20 cm above the triangle surface."""
    return f'''<model name="{name}"><pose>{x} {y} {height+3} 0 0 0</pose><link name="ball">
      <inertial><mass>0.1</mass><inertia><ixx>0.0016</ixx><iyy>0.0016</iyy><izz>0.0016</izz>
      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <visual name="ball"><geometry><sphere><radius>0.2</radius></sphere></geometry>
      <material><diffuse>1 0.1 0.1 1</diffuse></material></visual>
      <collision name="ball"><geometry><sphere><radius>0.2</radius></sphere></geometry></collision>
      </link></model>'''


def stop_child(proc):
    """Bound shutdown to the GUI process group created by this check."""
    if proc is not None and proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=5)


def main(argv=None):
    """Run visible mesh/material and terrain-contact checks, keeping numeric evidence and camera frames."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--no-gui', action='store_true', help='for unattended CI only; default opens a window')
    args = parser.parse_args(argv)
    bundle = args.bundle.resolve()
    scene = json.loads((bundle / 'scene.json').read_text(encoding='utf-8'))
    output = bundle / 'checks'
    output.mkdir(exist_ok=True)
    world_path = bundle / 'worlds' / f'{scene["world_name"]}.sdf'
    grid = scene['terrain']['heights']
    zones = {z['id']: z for z in scene['landing_zones']}
    points = dict(origin=(0, 0), clearing=(zones['clearing_near']['x'], zones['clearing_near']['y']),
                  slope=(zones['slope']['x'], zones['slope']['y']))
    snippets = [drop_probe('check_drop_' + name, x, y, surface_height(x, y, grid))
                for name, (x, y) in points.items()]
    h = scene['hiker'] or dict(x=87, y=124, z=3)
    cameras = [parse_camera('overview:-150,-115,210@40,100,0,1280,720'),
               parse_camera(f'hiker_50m:{h["x"]},{h["y"]},{h["z"]+50},0,{math.pi/2},{math.pi/2},2560,1440'),
               parse_camera(f'hiker_detail:{h["x"]+2},{h["y"]-3},{h["z"]+5}@{h["x"]},{h["y"]},{h["z"]+.2},1024,576')]
    text = inject(world_path.read_text(encoding='utf-8'), snippets + [camera_model_sdf(c, 1) for c in cameras])
    lock = threading.Lock()
    poses, frames, stats = {}, {}, dict(sim=0.0, rtf=[])
    started = time.monotonic()
    gui = None
    result = dict(schema='astra.suas2027.gazebo_check.v1', world_name=scene['world_name'],
                  visible=not args.no_gui, problems=[], drop_probes={})
    with HeadlessGazebo(str(world_path), text, str(output / 'gz_server.log'), verbosity=4) as server:
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from PIL import Image as PillowImage

        def pose_callback(msg):
            """Keep only our dynamic probe model poses from the world's broadcaster."""
            with lock:
                for pose in msg.pose:
                    if pose.name.startswith('check_drop_'):
                        poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)

        def stats_callback(msg):
            """Record simulated progress and steady-state real-time factor."""
            with lock:
                stats['sim'] = msg.sim_time.sec + msg.sim_time.nsec * 1e-9
                stats['rtf'].append(msg.real_time_factor)

        def image_callback_for(name):
            """Cache actual Ogre camera images without doing I/O in the transport callback."""
            def callback(msg):
                frame = image_to_array(msg)
                if frame is not None:
                    with lock:
                        frames[name] = frame
            return callback

        node = Node()
        node.subscribe(Pose_V, f'/world/{scene["world_name"]}/dynamic_pose/info', pose_callback)
        node.subscribe(WorldStatistics, f'/world/{scene["world_name"]}/stats', stats_callback)
        callbacks = []
        for camera in cameras:
            callbacks.append(image_callback_for(camera['name']))
            node.subscribe(Image, f'/preview/{camera["name"]}/image', callbacks[-1])
        with (output / 'gz_gui.log').open('w', encoding='utf-8') as gui_log:
            try:
                if not args.no_gui:
                    gui = subprocess.Popen(['gz', 'sim', '-g', '-v', '2'], env=server.env(),
                                           stdout=gui_log, stderr=subprocess.STDOUT, start_new_session=True)
                print(f'[check_suas_2027] running {server.partition}; visible={not args.no_gui}', flush=True)
                while time.monotonic() - started < args.timeout and server.alive():
                    with lock:
                        ready = stats['sim'] >= 10 and len(poses) == len(points) and len(frames) == len(cameras)
                    if ready:
                        break
                    time.sleep(.25)
                with lock:
                    for name, frame in frames.items():
                        PillowImage.fromarray(frame).save(output / f'{name}.png')
                    for name in points:
                        pos = poses.get('check_drop_' + name)
                        if pos is None:
                            result['problems'].append(f'no pose received for {name} drop probe')
                            continue
                        expected = surface_height(pos[0], pos[1], grid) + .2
                        error = abs(pos[2] - expected)
                        result['drop_probes'][name] = dict(position=list(pos), expected_center_z=expected,
                                                          error_m=error, passed=error < .05)
                        if error >= .05:
                            result['problems'].append(f'{name}: sphere missed terrain contact by {error:.3f} m')
                    rtfs = sorted(stats['rtf'][len(stats['rtf']) // 2:])
                    result.update(sim_seconds=stats['sim'], wall_seconds=time.monotonic()-started,
                                  real_time_factor_median=rtfs[len(rtfs)//2] if rtfs else 0,
                                  camera_frames=sorted(frames), server_alive=server.alive(),
                                  gui_alive=gui.poll() is None if gui else None)
                errors = [line for line in server.log_text().splitlines() if '[Err]' in line]
                result['server_errors'] = errors
                if errors:
                    result['problems'].append(f'{len(errors)} Gazebo server errors; inspect gz_server.log')
                if not server.alive() or stats['sim'] < 10:
                    result['problems'].append('server failed to reach ten simulated seconds')
                if len(frames) != len(cameras):
                    result['problems'].append('not all camera frames arrived')
                if gui and gui.poll() is not None:
                    result['problems'].append('visible Gazebo window exited unexpectedly')
                if result['real_time_factor_median'] < .8:
                    result['problems'].append('steady-state real-time factor below 0.8')
            finally:
                stop_child(gui)
    result['passed'] = not result['problems']
    (output / 'gazebo_check.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
