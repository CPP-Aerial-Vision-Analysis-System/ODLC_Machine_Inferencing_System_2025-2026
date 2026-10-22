"""
sim_bringup/photo_trigger.py -- PURPOSE: take one photo with the simulated camera ("press the shutter").

Two ways, tried in this order:
  1. TEAM WAY (default): publish std_msgs/Bool true on /camera/trigger, the team's own topic. The team's
     video_cam node (started by ./launcher.sh sim) then sends the SIYI "take photo" command to the camera
     emulator, which renders the photo in Gazebo, and video_cam downloads it into the team's photo folder
     (<team ros2_ws>/src/video_cam/mapping_photos, named "<lat> , <lon>.jpg"). The tool waits for the new
     file and prints its path.
  2. DIRECT WAY (--direct, or automatically when nobody listens on /camera/trigger): send the SIYI SDK
     take-photo packet (55 66 01 01 00 00 00 0c 00 34 ce) straight to the emulator (UDP 192.168.144.25:37260),
     then fetch the new photo from the emulator's media web server (TCP port 82), exactly the way video_cam
     does, and save it in ~/.astra_sim/photos/.

Usage:
  ros2 run sim_bringup photo_trigger [--direct] [--camera-ip 192.168.144.25] [--timeout 30] [--photo-dir DIR]
Exit codes: 0 = a new photo arrived (path printed), 1 = no photo within the timeout.
"""

import argparse
import binascii
import json
import os
import socket
import struct
import sys
import time
import urllib.parse
import urllib.request
from typing import List, Optional, Tuple

SDK_PORT = 37260
MEDIA_PORT = 82
CMD_TAKE_PHOTO = 0x0C


def siyi_packet(cmd_id: int, data: bytes = b'', seq: int = 0, need_ack: bool = True) -> bytes:
    """Build one SIYI SDK frame: 55 66 | ctrl | data_len (u16 LE) | seq (u16 LE) | cmd | data | CRC16 (LE).

    The CRC is CRC-16/XMODEM (binascii.crc_hqx with start value 0) over everything before it, the same
    as video_cam/camera_interface.py. siyi_packet(0x0C, b'\\x00') == 55 66 01 01 00 00 00 0c 00 34 ce.
    """
    body = b'\x55\x66' + bytes([1 if need_ack else 0]) + struct.pack('<HH', len(data), seq) + bytes([cmd_id]) + data
    return body + struct.pack('<H', binascii.crc_hqx(body, 0))


def _http_json(url: str, timeout: float = 5.0) -> Optional[dict]:
    """GET a URL and parse JSON; None on any network/JSON problem."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            return json.loads(resp.read().decode('utf-8', errors='replace'))
    except Exception:  # noqa: BLE001
        return None


def emulator_media_list(camera_ip: str) -> Tuple[str, List[dict]]:
    """Ask the camera's media server for its photo folder and the files in it (oldest first).

    Uses the same requests as video_cam: getdirectories?media_type=0, then getmedialist for the LAST folder.
    Returns (folder, list of {"name", "url", ...}); ('', []) when the server does not answer.
    """
    base = f'http://{camera_ip}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1'
    dirs = _http_json(f'{base}/getdirectories?media_type=0')
    try:
        folder = dirs['data']['directories'][-1]['path']
    except (TypeError, KeyError, IndexError):
        return '', []
    q = urllib.parse.urlencode({'media_type': 0, 'path': folder, 'start': 0, 'count': 9999})
    listing = _http_json(f'{base}/getmedialist?{q}')
    try:
        return folder, list(listing['data']['list'])
    except (TypeError, KeyError):
        return folder, []


def take_direct(camera_ip: str, timeout: float, out_dir: str) -> Optional[str]:
    """Direct way: SDK packet to the emulator, wait for a new file on its media server, download it."""
    _, before = emulator_media_list(camera_ip)
    names_before = {item.get('name') for item in before}
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(siyi_packet(CMD_TAKE_PHOTO, b'\x00'), (camera_ip, SDK_PORT))
    except OSError as exc:
        print(f'[photo] could not send to the camera at {camera_ip}:{SDK_PORT}: {exc}', flush=True)
        return None
    print(f'[photo] take-photo command sent to the camera emulator at {camera_ip}:{SDK_PORT}', flush=True)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        time.sleep(0.5)
        _, items = emulator_media_list(camera_ip)
        new = [it for it in items if it.get('name') not in names_before]
        if not new:
            continue
        item = new[-1]
        url = str(item.get('url', '')).replace('192.168.144.25', camera_ip)
        os.makedirs(out_dir, exist_ok=True)
        dest = os.path.join(out_dir, os.path.basename(str(item.get('name', 'photo.jpg'))))
        try:
            with urllib.request.urlopen(url, timeout=10) as resp, open(dest, 'wb') as f:
                f.write(resp.read())
        except Exception as exc:  # noqa: BLE001
            print(f'[photo] the photo {item.get("name")} exists on the emulated SD card but the download failed: {exc}',
                  flush=True)
            return None
        return dest
    return None


def take_via_team_topic(timeout: float, photo_dir: str) -> Tuple[bool, Optional[str]]:
    """Team way: publish /camera/trigger and wait for a new file in the team photo folder.

    Returns (somebody_listened, new_file_path_or_None).
    """
    import rclpy
    from std_msgs.msg import Bool

    def files():
        try:
            return {n for n in os.listdir(photo_dir) if n.lower().endswith(('.jpg', '.jpeg', '.png')) and '.tmp.' not in n}
        except OSError:
            return set()

    rclpy.init()
    node = rclpy.create_node('photo_trigger')
    pub = node.create_publisher(Bool, '/camera/trigger', 10)
    before = files()
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    if pub.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        return False, None
    pub.publish(Bool(data=True))
    print('[photo] shutter pressed: published true on /camera/trigger (the team\'s video_cam takes the photo)', flush=True)
    found = None
    end = time.monotonic() + timeout
    while time.monotonic() < end and found is None:
        rclpy.spin_once(node, timeout_sec=0.5)
        new = sorted(files() - before)
        if new:
            found = os.path.join(photo_dir, new[-1])
    node.destroy_node()
    rclpy.shutdown()
    return True, found


def main(argv=None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = argparse.ArgumentParser(prog='photo_trigger', description=__doc__.split('\n\n')[0])
    parser.add_argument('--direct', action='store_true', help='skip /camera/trigger, talk to the emulator directly')
    parser.add_argument('--camera-ip', default='192.168.144.25', help='address of the camera (emulator)')
    parser.add_argument('--timeout', type=float, default=30.0, help='seconds to wait for the new photo')
    parser.add_argument('--photo-dir', default='', help='team photo folder ("" = video_cam\'s own rule)')
    parser.add_argument('--out-dir', default='', help='where --direct saves photos ("" = ~/.astra_sim/photos)')
    args = parser.parse_args(argv)

    from sim_bringup.sim_paths import sim_home, team_photo_dir
    if not args.direct:
        photo_dir = team_photo_dir(args.photo_dir)
        listened, path = take_via_team_topic(args.timeout, photo_dir)
        if listened:
            if path:
                print(f'[photo] NEW PHOTO: {path}', flush=True)
                return 0
            print(f'[photo] no new photo in {photo_dir} within {args.timeout:.0f} s (see the team stack log: '
                  'video_cam needs the camera emulator and a GPS fix)', flush=True)
            return 1
        print('[photo] nobody listens on /camera/trigger (the team stack is not running): '
              'using the direct way instead', flush=True)
    out_dir = args.out_dir or str(sim_home() / 'photos')
    path = take_direct(args.camera_ip, args.timeout, out_dir)
    if path:
        print(f'[photo] NEW PHOTO: {path}', flush=True)
        return 0
    print(f'[photo] no new photo from the camera emulator at {args.camera_ip} within {args.timeout:.0f} s '
          '(is the simulator running with emulator:=true, and does the camera address exist? see status.sh)', flush=True)
    return 1


if __name__ == '__main__':
    sys.exit(main())
