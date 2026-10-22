"""
test/e2e/e2e_sim_models_check.py -- PURPOSE: integration check of the emulator against the REAL simulated
drone (sim_models iris_a8mini) parked facing north in sim_models' own test world
(sim_models/test/worlds/sim_models_test.sdf: red pole 60 m due north, magenta pole 30 deg west of north).
Through the SIYI SDK and the media web server it checks that:
  * the measured gimbal attitude (0x0D, from the model's joint states) follows each command;
  * yaw 0 looks NORTH at the red pole and yaw +30 turns LEFT to the magenta pole;
  * pitch -90 sees only ground; every photo arrives with 0x0B < 0.8 s as a 2560x1440 JPEG.
Saves the photos, a contact sheet and report.json in --out; exit code 0 only if all checks pass.
Usage (normally via run_e2e_sim_models.sh): python3 e2e_sim_models_check.py --ip 127.0.0.3 --out DIR
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

import cv2
import numpy as np

from siyi_emulator.sdk_client import SiyiClient

RESULTS = []
PHOTOS = []
HSV = {"red": [((0, 120, 80), (8, 255, 255)), ((172, 120, 80), (179, 255, 255))],
       "magenta": [((140, 100, 100), (162, 255, 255))]}


def check(name: str, ok: bool, detail: str) -> None:
    """Record and print one PASS/FAIL line."""
    RESULTS.append({"check": name, "pass": bool(ok), "detail": detail})
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)


def blob(img, color):
    """Centre x, centre y and area of the largest blob of a colour, or None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = np.zeros(img.shape[:2], np.uint8)
    for lo, hi in HSV[color]:
        mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
    count, _, stats, cents = cv2.connectedComponentsWithStats(mask)
    if count <= 1:
        return None
    i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    if stats[i, cv2.CC_STAT_AREA] < 100:
        return None
    return float(cents[i][0]), float(cents[i][1]), int(stats[i, cv2.CC_STAT_AREA])


def snap(c: SiyiClient, label: str, out: str):
    """Photo like video_cam: 0x0C, wait 0x0B, poll the listing, download; save and return the image."""
    before = {i["name"] for i in c.media_list()}
    t0 = time.monotonic()
    info = c.take_photo()
    t_feedback = time.monotonic() - t0
    item = None
    while time.monotonic() - t0 < 15 and item is None:
        new = [i for i in c.media_list() if i["name"] not in before]
        item = new[-1] if new else None
        time.sleep(0.25)
    if item is None:
        check(f"photo {label}", False, f"0x0B={info} after {t_feedback:.2f}s, never listed")
        return None
    data = c.download(item["url"])
    img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    check(f"photo {label}", info == 0 and t_feedback < 0.8 and img is not None and img.shape == (1440, 2560, 3),
          f"0x0B={info} after {t_feedback:.2f}s, {item['name']} {None if img is None else img.shape}, "
          f"{len(data) / 1e6:.2f} MB")
    if img is not None:
        cv2.imwrite(os.path.join(out, f"{len(PHOTOS) + 1:02d}_{label}.jpg"), img)
        PHOTOS.append((label, img))
    return img


def point(c: SiyiClient, yaw: float, pitch: float, tol: float = 1.5) -> None:
    """Command the gimbal, wait for the slew, and check the measured attitude."""
    c.set_angles(yaw, pitch)
    time.sleep(3.0)
    att = c.attitude()
    check(f"measured attitude follows ({yaw:g}, {pitch:g})",
          att is not None and abs(att["yaw"] - yaw) < tol and abs(att["pitch"] - pitch) < tol, f"0x0D = {att}")


def main() -> int:
    p = argparse.ArgumentParser(description="siyi_emulator vs the real sim_models drone")
    p.add_argument("--ip", default="127.0.0.3")
    p.add_argument("--port", type=int, default=37260)
    p.add_argument("--media-port", type=int, default=82)
    p.add_argument("--out", default="/tmp/siyi_e2e_sim_models")
    args = p.parse_args()
    os.makedirs(args.out, exist_ok=True)
    c = SiyiClient(args.ip, args.port, args.media_port, timeout=2.0)
    hw = None
    for _ in range(30):
        hw = c.hardware_id()
        if hw:
            break
        time.sleep(1.0)
    check("emulator answers 0x02", hw is not None, str(hw))

    point(c, 0, 0)
    img = snap(c, "yaw0_red_pole_north", args.out)
    if img is not None:
        b = blob(img, "red")
        check("yaw 0 looks NORTH at the red pole", b is not None and abs(b[0] - 1280) < 0.06 * 2560, f"red {b}")
    point(c, 30, 0)
    img = snap(c, "yaw+30_magenta_pole", args.out)
    if img is not None:
        b = blob(img, "magenta")
        check("yaw +30 turns LEFT to the magenta pole", b is not None and abs(b[0] - 1280) < 0.06 * 2560, f"magenta {b}")
    point(c, 0, -90)
    img = snap(c, "pitch-90_ground", args.out)
    if img is not None:
        b, r = img[:, :, 0].astype(int), img[:, :, 2].astype(int)
        sky = ((b > 170) & (b - r > 40)).mean()
        check("pitch -90 sees only ground", sky < 0.01, f"sky fraction {sky:.3f}")

    if PHOTOS:
        thumbs = []
        for label, im in PHOTOS:
            t = cv2.resize(im, (640, 360))
            cv2.rectangle(t, (0, 0), (640, 34), (0, 0, 0), -1)
            cv2.putText(t, label, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            thumbs.append(t)
        cv2.imwrite(os.path.join(args.out, "sim_models_contact_sheet.jpg"), np.hstack(thumbs))
    passed = all(r["pass"] for r in RESULTS)
    with open(os.path.join(args.out, "report.json"), "w", encoding="utf-8") as handle:
        json.dump({"passed": passed, "checks": RESULTS}, handle, indent=2)
    print("ALL CHECKS PASSED" if passed else "SOME CHECKS FAILED", flush=True)
    c.close()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
