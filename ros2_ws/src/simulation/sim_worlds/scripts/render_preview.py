#!/usr/bin/env python3
"""
sim_worlds/scripts/render_preview.py -- PURPOSE: take still pictures of a Gazebo world from cameras that you
place yourself, WITHOUT a drone and WITHOUT opening any window.

What it does, step by step:
  1. reads a world file (for example worlds/runway.sdf),
  2. makes a temporary copy with extra static "preview cameras" (each one is a tiny model with a camera sensor
     that publishes on the gz topic /preview/<name>/image),
  3. starts `gz sim -s -r` (server only, headless) on that copy in its own GZ_PARTITION, so it never disturbs
     another simulation (sim_worlds/gzrun.py),
  4. waits until the simulation clock passed --settle seconds (so every texture is loaded), saves one PNG per
     camera into --out, prints the real-time factor, and
  5. stops its own gz server (and nothing else).

Why it exists: to LOOK at the worlds the way the drone camera sees them (are the hiker and the tent
recognisable? does the grass look natural from 50 m?), and to make test pictures for the detection model.

Camera spec (repeat --camera for more cameras):
  NAME:x,y,z,roll,pitch,yaw[,width,height[,hfov_rad]]      pose in the world frame (metres, radians)
  NAME:x,y,z@tx,ty,tz[,width,height[,hfov_rad]]            "look at" form: camera at x,y,z looking at tx,ty,tz
  pitch = +1.5708 looks straight DOWN (Gazebo cameras look along their +x axis); yaw 1.5708 = image top north.
  Defaults: 2560 x 1440 pixels, hfov 1.4137 rad (81 deg, the SIYI A8 mini lens).

Example:
  render_preview.py --world worlds/runway.sdf --out /tmp/prev \\
      --camera "top:25,95,50,0,1.5708,1.5708" --camera "oblique:-60,-40,60@25,95,0,1920,1080"
"""

import argparse
import math
import os
import re
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.isdir(os.path.join(os.path.dirname(HERE), 'sim_worlds')):
    sys.path.insert(0, os.path.dirname(HERE))              # running from the source tree

from sim_worlds.gzrun import HeadlessGazebo, inject     # noqa: E402

DEFAULT_W, DEFAULT_H, DEFAULT_HFOV = 2560, 1440, 1.4137


def parse_camera(spec):
    """Parse one --camera string into a dict.

    Input:  "NAME:x,y,z,roll,pitch,yaw[,w,h[,hfov]]" or "NAME:x,y,z@tx,ty,tz[,w,h[,hfov]]".
    Output: dict(name, x, y, z, roll, pitch, yaw, width, height, hfov) in metres / radians / pixels.
    Raises ValueError with a readable message when the string is malformed.
    """
    if ':' not in spec:
        raise ValueError(f'camera "{spec}" needs NAME:numbers')
    name, rest = spec.split(':', 1)
    if not re.fullmatch(r'[A-Za-z0-9_]+', name):
        raise ValueError(f'camera name "{name}" may only use letters, digits and _')
    if '@' in rest:
        pos_s, tgt_s = rest.split('@', 1)
        pos = [float(v) for v in pos_s.split(',')]
        tgt = [float(v) for v in tgt_s.split(',')]
        if len(pos) != 3 or len(tgt) < 3:
            raise ValueError(f'camera "{spec}": look-at form is x,y,z@tx,ty,tz[,w,h[,hfov]]')
        extra = tgt[3:]
        dx, dy, dz = tgt[0] - pos[0], tgt[1] - pos[1], tgt[2] - pos[2]
        roll, pitch, yaw = 0.0, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)
    else:
        vals = [float(v) for v in rest.split(',')]
        if len(vals) < 6:
            raise ValueError(f'camera "{spec}": need x,y,z,roll,pitch,yaw')
        pos, (roll, pitch, yaw), extra = vals[:3], vals[3:6], vals[6:]
    return {'name': name, 'x': pos[0], 'y': pos[1], 'z': pos[2], 'roll': roll, 'pitch': pitch, 'yaw': yaw,
            'width': int(extra[0]) if len(extra) > 0 else DEFAULT_W,
            'height': int(extra[1]) if len(extra) > 1 else DEFAULT_H,
            'hfov': float(extra[2]) if len(extra) > 2 else DEFAULT_HFOV}


def camera_model_sdf(cam, rate_hz=2.0):
    """SDF text of one static preview-camera model publishing gz.msgs.Image on /preview/<name>/image."""
    c = cam
    return f'''
    <!-- preview camera added by render_preview.py (not part of the real world) -->
    <model name="preview_cam_{c['name']}">
      <static>true</static>
      <pose>{c['x']:.3f} {c['y']:.3f} {c['z']:.3f} {c['roll']:.5f} {c['pitch']:.5f} {c['yaw']:.5f}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <always_on>1</always_on>
          <update_rate>{rate_hz}</update_rate>
          <topic>/preview/{c['name']}/image</topic>
          <camera>
            <horizontal_fov>{c['hfov']:.5f}</horizontal_fov>
            <image><width>{c['width']}</width><height>{c['height']}</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>4000</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
'''


def image_to_array(msg):
    """gz.msgs.Image (8-bit RGB or BGR) -> numpy uint8 array (h, w, 3) in RGB order, or None if incomplete."""
    import numpy as np
    w, h = msg.width, msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8)
    step = msg.step if msg.step else w * 3
    if data.size < step * h:
        return None
    arr = data[:step * h].reshape(h, step)[:, :w * 3].reshape(h, w, 3)
    if msg.pixel_format_type == 8:                 # gz.msgs.PixelFormatType BGR_INT8
        arr = arr[:, :, ::-1]
    return arr.copy()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--world', required=True, help='world SDF file to look at')
    ap.add_argument('--out', required=True, help='folder for the PNG files and the gz log')
    ap.add_argument('--camera', action='append', default=[], help='camera spec (see the help text above)')
    ap.add_argument('--settle', type=float, default=6.0, help='simulated seconds to wait before saving (default 6)')
    ap.add_argument('--timeout', type=float, default=240.0, help='give up after this many wall seconds (default 240)')
    ap.add_argument('--resource-path', action='append', default=[],
                    help='extra folder(s) for model:// URIs (added to GZ_SIM_RESOURCE_PATH)')
    args = ap.parse_args(argv)
    if not args.camera:
        ap.error('give at least one --camera')
    cams = [parse_camera(s) for s in args.camera]
    os.makedirs(args.out, exist_ok=True)
    with open(args.world, encoding='utf-8') as f:
        world_text = inject(f.read(), [camera_model_sdf(c) for c in cams])

    from PIL import Image as PILImage
    t_start = time.time()
    saved, stats, lock = {}, {'rtf': [], 'sim': 0.0}, threading.Lock()
    ok = False
    with HeadlessGazebo(args.world, world_text, os.path.join(args.out, 'gz_server.log'),
                        extra_resource_paths=args.resource_path) as gz:
        print(f'[render_preview] gz sim pid {gz.proc.pid}, partition {gz.partition}, log {gz.log_path}', flush=True)
        from gz.transport13 import Node                       # noqa: E402 (needs the partition set by start())
        from gz.msgs10.image_pb2 import Image                 # noqa: E402
        from gz.msgs10.world_stats_pb2 import WorldStatistics  # noqa: E402

        def make_cb(cam):
            def cb(msg):
                stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
                with lock:
                    if cam['name'] in saved or stamp < args.settle:
                        return
                arr = image_to_array(msg)
                if arr is None:
                    return
                path = os.path.join(args.out, f'{cam["name"]}.png')
                PILImage.fromarray(arr, 'RGB').save(path)
                with lock:
                    saved[cam['name']] = (path, stamp, time.time() - t_start)
            return cb

        def stats_cb(msg):
            with lock:
                stats['rtf'].append(msg.real_time_factor)
                stats['sim'] = msg.sim_time.sec + 1e-9 * msg.sim_time.nsec

        node = Node()
        for c in cams:
            node.subscribe(Image, f'/preview/{c["name"]}/image', make_cb(c))
        if gz.world_name:
            node.subscribe(WorldStatistics, f'/world/{gz.world_name}/stats', stats_cb)
        next_report = time.time() + 15.0
        try:
            while time.time() - t_start < args.timeout:
                if not gz.alive():
                    print(f'[render_preview] gz sim exited early (see {gz.log_path})')
                    break
                with lock:
                    if len(saved) == len(cams):
                        ok = True
                        break
                    if time.time() > next_report:
                        print(f'[render_preview] ... wall {time.time() - t_start:.0f} s, sim {stats["sim"]:.1f} s, '
                              f'rtf {stats["rtf"][-1] if stats["rtf"] else 0.0:.2f}, frames {len(saved)}/{len(cams)}',
                              flush=True)
                        next_report = time.time() + 15.0
                time.sleep(0.25)
        except KeyboardInterrupt as exc:
            print(f'[render_preview] interrupted ({exc}); stopping gz sim')

    rtf = stats['rtf'][len(stats['rtf']) // 2:] or [0.0]
    print(f'[render_preview] sim time reached {stats["sim"]:.1f} s, median real-time factor '
          f'{sorted(rtf)[len(rtf) // 2]:.2f}, wall {time.time() - t_start:.0f} s')
    for c in cams:
        if c['name'] in saved:
            p, st, wall = saved[c['name']]
            print(f'[render_preview] saved {p} (sim {st:.1f} s, wall {wall:.0f} s)')
        else:
            print(f'[render_preview] MISSING frame for camera {c["name"]}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
