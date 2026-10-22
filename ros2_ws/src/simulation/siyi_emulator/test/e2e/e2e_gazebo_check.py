"""
test/e2e/e2e_gazebo_check.py -- PURPOSE: drive a running siyi_emulator (attached to the Gazebo test world
test/fixtures/gimbal_stand.sdf) through the SIYI SDK and its media web server like video_cam does, and
check with real rendered photos that:
  1. yaw 0 looks at the RED box; +30 turns LEFT to the GREEN box; -30 turns right to the BLUE box;
  2. pitch -90 looks straight down at the YELLOW disc;
  3. 0x0D reports the commanded attitude; a 0x07 speed of +50 for 1 s turns the camera RIGHT ~45 deg;
  4. zoom 3x makes the red box 3x wider (digital crop, or optical with --optical-zoom);
  5. lock mode keeps the world heading when the drone turns; follow mode turns with it;
  6. stabilisation: the horizon stays level (and at the same height) when the drone rolls and pitches;
  7. every photo arrives with 0x0B inside 0.8 s, appears in getmedialist and downloads as a 2560x1440 JPEG.
It saves every photo plus a labelled contact sheet and report.json in --out, prints PASS/FAIL per check
and exits 0 only if everything passed.
Usage (normally via run_e2e_gazebo.sh): python3 e2e_gazebo_check.py --ip 127.0.0.3 --out /tmp/siyi_e2e
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from siyi_emulator.gimbal_control import quat_from_rot, rot_from_rpy
from siyi_emulator.sdk_client import SiyiClient

CHECKS: List[Dict] = []
PHOTOS: List[Tuple[str, np.ndarray]] = []


def check(name: str, ok: bool, detail: str) -> bool:
    """Record and print one PASS/FAIL line."""
    CHECKS.append({"check": name, "pass": bool(ok), "detail": detail})
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)
    return ok


# ------------------------------------------------------------------------------------- Gazebo
class GzPose:
    """Teleports the floating 'iris' body through /world/<world>/set_pose (gz-transport Python)."""

    def __init__(self, world: str) -> None:
        from gz.transport13 import Node
        self.node = Node()
        self.service = f"/world/{world}/set_pose"

    def set(self, roll_deg: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0, z: float = 1.0) -> bool:
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.pose_pb2 import Pose
        req = Pose()
        req.name = "iris"
        req.position.z = z
        qx, qy, qz, qw = quat_from_rot(rot_from_rpy(math.radians(roll_deg), math.radians(pitch_deg),
                                                    math.radians(yaw_deg)))
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = qx, qy, qz, qw
        ok, rep = self.node.request(self.service, req, Pose, Boolean, 3000)
        return bool(ok and rep.data)


# ------------------------------------------------------------------------------------- images
HSV_RANGES = {   # OpenCV HSV: H 0-179
    "red": [((0, 120, 60), (8, 255, 255)), ((172, 120, 60), (179, 255, 255))],
    "green": [((45, 120, 120), (75, 255, 255))],
    "blue": [((110, 120, 60), (130, 255, 255))],
    "yellow": [((22, 120, 120), (35, 255, 255))],
}


def blob(img: np.ndarray, color: str) -> Optional[Dict[str, float]]:
    """Largest blob of a colour: centre (cx, cy), width, height, area in pixels (None if absent)."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(img.shape[:2], np.uint8)
    for lo, hi in HSV_RANGES[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, _, stats, cents = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[i, cv2.CC_STAT_AREA] < 200:
        return None
    return {"cx": float(cents[i][0]), "cy": float(cents[i][1]), "w": float(stats[i, cv2.CC_STAT_WIDTH]),
            "h": float(stats[i, cv2.CC_STAT_HEIGHT]), "area": float(stats[i, cv2.CC_STAT_AREA])}


def horizon(img: np.ndarray) -> Optional[Tuple[float, float]]:
    """Sky/ground boundary: (tilt angle in degrees, mean row in pixels), or None if not found.

    Sky = the world's light-blue background (blue clearly above red); for each column the first
    non-sky pixel from the top is the boundary; a straight line is fitted through them.
    """
    b, g, r = (img[:, :, k].astype(np.int16) for k in range(3))
    sky = (b > 170) & (b - r > 50)
    sky = cv2.morphologyEx(sky.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)).astype(bool)
    xs, ys = [], []
    for x in range(10, img.shape[1] - 10, 16):
        column = sky[:, x]
        if not column[: img.shape[0] // 20].all():
            continue                                  # top of this column is not sky
        non_sky = np.nonzero(~column)[0]
        if non_sky.size:
            xs.append(x)
            ys.append(non_sky[0])
    if len(xs) < 20:
        return None
    slope, intercept = np.polyfit(np.array(xs, float), np.array(ys, float), 1)
    mean_row = slope * img.shape[1] / 2 + intercept
    return math.degrees(math.atan(slope)), float(mean_row)


# ------------------------------------------------------------------------------------- camera
def wait_for_camera(c: SiyiClient, timeout: float = 40.0) -> Optional[str]:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        hw = c.hardware_id()
        if hw:
            return hw
        time.sleep(1.0)
    return None


def snap(c: SiyiClient, label: str, out: str) -> Optional[np.ndarray]:
    """Take a photo the way video_cam does, check 0x0B timing + listing + download, save it."""
    before = {item["name"] for item in c.media_list()}
    t0 = time.monotonic()
    info = c.take_photo(feedback_timeout=0.8)
    t_feedback = time.monotonic() - t0
    item = None
    end = time.monotonic() + 15.0
    while time.monotonic() < end and item is None:
        new = [i for i in c.media_list() if i["name"] not in before]
        item = new[-1] if new else None
        if item is None:
            time.sleep(0.25)
    t_listed = time.monotonic() - t0
    if not check(f"photo '{label}' delivered", info == 0 and t_feedback < 0.8 and item is not None,
                 f"0x0B={info} after {t_feedback:.2f}s, listed after {t_listed:.2f}s as "
                 f"{item['name'] if item else None}"):
        return None
    data = c.download(item["url"])
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    check(f"photo '{label}' is a 2560x1440 JPEG", img is not None and img.shape == (1440, 2560, 3),
          f"{len(data) / 1e6:.2f} MB, shape {None if img is None else img.shape}")
    path = os.path.join(out, f"{len(PHOTOS) + 1:02d}_{label}.jpg")
    with open(path, "wb") as handle:
        handle.write(data)
    PHOTOS.append((label, img))
    return img


def centred(b: Optional[Dict[str, float]], img: np.ndarray, axis: str = "x", tol: float = 0.06) -> Tuple[bool, str]:
    if b is None:
        return False, "not visible"
    size = img.shape[1] if axis == "x" else img.shape[0]
    value = b["cx"] if axis == "x" else b["cy"]
    off = (value - size / 2) / size
    return abs(off) < tol, f"centre {axis}={value:.0f} px ({off * 100:+.1f}% of the frame)"


def settle(c: SiyiClient, yaw: float, pitch: float, wait: float = 2.5) -> None:
    c.set_angles(yaw, pitch)
    time.sleep(wait)


def contact_sheet(out: str) -> str:
    """All photos as labelled 640x360 thumbnails in one image."""
    thumbs = []
    for label, img in PHOTOS:
        t = cv2.resize(img, (640, 360), interpolation=cv2.INTER_AREA)
        cv2.rectangle(t, (0, 0), (640, 34), (0, 0, 0), -1)
        cv2.putText(t, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        thumbs.append(t)
    while len(thumbs) % 3:
        thumbs.append(np.zeros((360, 640, 3), np.uint8))
    rows = [np.hstack(thumbs[i:i + 3]) for i in range(0, len(thumbs), 3)]
    sheet = np.vstack(rows)
    path = os.path.join(out, "e2e_contact_sheet.jpg")
    cv2.imwrite(path, sheet, [cv2.IMWRITE_JPEG_QUALITY, 85])
    return path


# ------------------------------------------------------------------------------------- main
def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    p.add_argument("--ip", default="127.0.0.3")
    p.add_argument("--port", type=int, default=37260)
    p.add_argument("--media-port", type=int, default=82)
    p.add_argument("--out", default="/tmp/siyi_e2e")
    p.add_argument("--world", default="gimbal_stand")
    p.add_argument("--optical-zoom", action="store_true", help="the world has CameraZoomPlugin (sharp zoom)")
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)

    c = SiyiClient(args.ip, args.port, args.media_port, timeout=2.0)
    hw = wait_for_camera(c)
    if not check("emulator answers 0x02", hw is not None, f"hardware id {hw}"):
        return 1
    gz = GzPose(args.world)
    check("drone pose service works", gz.set(0, 0, 0), "set_pose iris level, heading east")
    time.sleep(1.0)

    # 1-2: directions -------------------------------------------------------------------
    settle(c, 0, 0, 3.0)
    img = snap(c, "yaw0_pitch0_red_ahead", args.out)
    red_1x = blob(img, "red") if img is not None else None
    if img is not None:
        ok, txt = centred(red_1x, img)
        check("yaw 0 looks at the RED box", ok, txt)
        expected_w = (2560 / 2) / math.tan(1.4137 / 2) * (1.0 / 5.5)   # 1 m box, front face ~5.5 m away
        check("1x field of view is 81 deg (box width)", red_1x is not None and abs(red_1x["w"] / expected_w - 1) < 0.25,
              f"red box {red_1x['w'] if red_1x else 0:.0f} px wide, expected ~{expected_w:.0f}")
    settle(c, 30, 0)
    img = snap(c, "yaw+30_green_left", args.out)
    if img is not None:
        ok, txt = centred(blob(img, "green"), img)
        check("yaw +30 turns LEFT to the GREEN box", ok, txt)
    settle(c, -30, 0)
    img = snap(c, "yaw-30_blue_right", args.out)
    if img is not None:
        ok, txt = centred(blob(img, "blue"), img)
        check("yaw -30 turns RIGHT to the BLUE box", ok, txt)
    settle(c, 0, -90, 3.0)
    img = snap(c, "pitch-90_yellow_below", args.out)
    if img is not None:
        b = blob(img, "yellow")
        okx, tx = centred(b, img, "x", 0.08)
        oky, ty = centred(b, img, "y", 0.08)
        check("pitch -90 looks STRAIGHT DOWN at the YELLOW disc", okx and oky, f"{tx}; {ty}")

    # 3: attitude report and speed command ---------------------------------------------------
    settle(c, 30, -45)
    att = c.attitude()
    check("0x0D reports the commanded attitude",
          att is not None and abs(att["yaw"] - 30) < 1.0 and abs(att["pitch"] + 45) < 1.0 and abs(att["roll"]) < 1.0,
          f"0x0D = {att}")
    settle(c, 0, 0)
    c.rotate(50, 0)                       # +50 = right at 45 deg/s
    time.sleep(1.0)
    c.rotate(0, 0)
    time.sleep(0.8)
    att = c.attitude()
    check("0x07 speed +50 for 1 s turns RIGHT ~45 deg", att is not None and -60 < att["yaw"] < -30,
          f"yaw after rotation = {att['yaw'] if att else None}")

    # 4: zoom ----------------------------------------------------------------------------------
    settle(c, 0, 0)
    c.absolute_zoom(3.0)
    time.sleep(2.0)
    zoom_now = c.current_zoom()
    img = snap(c, "zoom3x_red_ahead", args.out)
    if img is not None and red_1x is not None:
        b = blob(img, "red")
        ratio = (b["w"] / red_1x["w"]) if b else 0.0
        check("zoom 3x makes the red box 3x wider", b is not None and 2.6 < ratio < 3.4,
              f"0x18={zoom_now}, width ratio {ratio:.2f} ({'optical' if args.optical_zoom else 'digital'})")
        sharp = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
        check("zoomed photo sharpness measured", True, f"Laplacian variance {sharp:.0f} "
              f"({'optical zoom should stay sharp' if args.optical_zoom else 'digital crop is softer'})")
    c.absolute_zoom(1.0)
    time.sleep(1.5)

    # 5: lock / follow ------------------------------------------------------------------------
    c.set_mode("lock")
    time.sleep(0.3)
    check("0x0C func 3 -> lock (0x19)", c.gimbal_mode() == "lock", f"0x19 = {c.gimbal_mode()}")
    gz.set(0, 0, 45)                        # drone turns 45 deg left
    time.sleep(2.0)
    img = snap(c, "lock_drone_yaw45_still_red", args.out)
    att = c.attitude()
    if img is not None:
        ok, txt = centred(blob(img, "red"), img)
        check("lock mode keeps looking east when the drone turns", ok and att is not None and abs(att["yaw"] + 45) < 2,
              f"{txt}; 0x0D yaw (relative to the drone) = {att['yaw'] if att else None}")
    c.set_mode("follow")
    time.sleep(0.3)
    gz.set(0, 0, 30)                        # drone heading 30 deg left; follow mode, camera yaw 0
    time.sleep(1.0)
    settle(c, 0, 0)
    img = snap(c, "follow_drone_yaw30_green", args.out)
    if img is not None:
        ok, txt = centred(blob(img, "green"), img)
        check("follow mode turns with the drone", ok, txt)

    # 6: stabilisation (look west where nothing stands, so only sky and ground are in view) ------
    gz.set(0, 0, 180)
    time.sleep(2.0)
    img_level = snap(c, "west_level", args.out)
    gz.set(20, 10, 180)                     # drone rolled 20 deg and pitched 10 deg
    time.sleep(2.5)
    img_tilt = snap(c, "west_drone_roll20_pitch10", args.out)
    att = c.attitude()
    if img_level is not None and img_tilt is not None:
        h0, h1 = horizon(img_level), horizon(img_tilt)
        ok = h0 is not None and h1 is not None and abs(h1[0]) < 1.5 and abs(h1[1] - h0[1]) < 25
        check("horizon stays level and in place when the drone tilts", ok,
              f"level: {h0}, tilted drone: {h1} (angle deg, row px); 0x0D = {att}")
    gz.set(0, 0, 0)
    time.sleep(1.0)

    # 7: listing + format -------------------------------------------------------------------------
    listed = c.media_list()
    check("getmedialist lists every photo, oldest first",
          len(listed) == len(PHOTOS) and [i["name"] for i in listed] == sorted(i["name"] for i in listed),
          f"{len(listed)} listed, {len(PHOTOS)} taken")

    sheet = contact_sheet(args.out)
    passed = all(cc["pass"] for cc in CHECKS)
    with open(os.path.join(args.out, "report.json"), "w", encoding="utf-8") as handle:
        json.dump({"passed": passed, "checks": CHECKS, "contact_sheet": sheet}, handle, indent=2)
    print(f"\n{'ALL CHECKS PASSED' if passed else 'SOME CHECKS FAILED'} ({sum(cc['pass'] for cc in CHECKS)}/"
          f"{len(CHECKS)}). Photos and contact sheet in {args.out}", flush=True)
    c.close()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
