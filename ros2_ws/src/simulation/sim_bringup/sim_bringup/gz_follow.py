"""
sim_bringup/gz_follow.py -- PURPOSE: make the Gazebo window's 3D view follow the drone, like a chase camera.

Gazebo Harmonic's GUI plugin "CameraTracking" offers the services /gui/follow (gz.msgs.StringMsg = the
name of the entity to follow) and /gui/follow/offset (gz.msgs.Vector3d = where the camera sits relative
to the drone, metres). Both only exist while the Gazebo WINDOW is open. This tool waits until
  1. the GUI is up (the service /gui/follow is listed), and
  2. the drone entity exists in the world (the spawn finished),
then asks the GUI to follow the drone and sets the camera offset. It uses the `gz service` command line
tool, so it inherits GZ_PARTITION / GZ_IP from the environment like every other Gazebo program.

Usage:
  ros2 run sim_bringup gz_follow [--entity iris] [--offset -12 0 6] [--timeout 240]
Exit codes: 0 = following, 1 = gave up (GUI or drone never appeared, or the request failed).
"""

import argparse
import re
import subprocess
import sys
import time
from typing import List, Optional


def run_gz(args: List[str], timeout: float = 15.0) -> Optional[str]:
    """Run `gz <args>` and return its stdout text, or None when it failed or timed out."""
    try:
        res = subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return res.stdout if res.returncode == 0 else None


def service_listed(name: str) -> bool:
    """True when `gz service --list` shows the service `name`."""
    out = run_gz(['service', '--list'])
    return bool(out) and any(line.strip() == name for line in out.splitlines())


def entity_exists(entity: str) -> bool:
    """True when `gz model --list` shows a model called `entity`."""
    out = run_gz(['model', '--list'], timeout=20.0)
    return bool(out) and re.search(rf'^\s*-\s*{re.escape(entity)}\s*$', out, re.M) is not None


def request(service: str, req_type: str, rep_type: str, req: str) -> bool:
    """Call a Gazebo service once; True when it answered with data: true."""
    out = run_gz(['service', '-s', service, '--reqtype', req_type, '--reptype', rep_type,
                  '--timeout', '3000', '--req', req])
    return bool(out) and 'data: true' in out


def follow(entity: str, offset: List[float]) -> bool:
    """Ask the Gazebo GUI to follow `entity` and place the camera at `offset` (x, y, z metres)."""
    ok = request('/gui/follow', 'gz.msgs.StringMsg', 'gz.msgs.Boolean', f'data: "{entity}"')
    if ok:
        x, y, z = offset
        request('/gui/follow/offset', 'gz.msgs.Vector3d', 'gz.msgs.Boolean', f'x: {x}, y: {y}, z: {z}')
    return ok


def main(argv=None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = argparse.ArgumentParser(prog='gz_follow', description=__doc__.split('\n\n')[0])
    parser.add_argument('--entity', default='iris', help='Gazebo entity to follow (default: iris)')
    parser.add_argument('--offset', type=float, nargs=3, default=[-12.0, 0.0, 6.0], metavar=('X', 'Y', 'Z'),
                        help='camera position relative to the drone in metres (default: -12 0 6 = behind, above)')
    parser.add_argument('--timeout', type=float, default=240.0, help='give up after this many seconds')
    args = parser.parse_args(argv)

    start = time.monotonic()
    stage = 'the Gazebo window (service /gui/follow)'
    last_note = -100.0
    while time.monotonic() - start < args.timeout:
        if stage.startswith('the Gazebo window'):
            if service_listed('/gui/follow'):
                stage = f'the drone "{args.entity}"'
                continue
        elif stage.startswith('the drone'):
            if entity_exists(args.entity):
                stage = 'follow request'
                continue
        else:
            if follow(args.entity, args.offset):
                print(f'[gz_follow] the Gazebo 3D view now follows "{args.entity}" '
                      f'(camera offset {args.offset[0]:g}, {args.offset[1]:g}, {args.offset[2]:g} m)', flush=True)
                return 0
        elapsed = time.monotonic() - start
        if elapsed - last_note >= 20.0:
            print(f'[gz_follow] waiting for {stage} ({elapsed:.0f} s)', flush=True)
            last_note = elapsed
        time.sleep(2.0)
    print(f'[gz_follow] gave up after {args.timeout:.0f} s while waiting for {stage}. '
          'You can still follow the drone by hand: right-click it in the Gazebo window -> "Follow".', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
