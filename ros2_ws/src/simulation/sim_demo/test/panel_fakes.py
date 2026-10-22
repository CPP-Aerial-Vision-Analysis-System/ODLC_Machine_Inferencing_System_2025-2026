"""
sim_demo/test/panel_fakes.py -- PURPOSE: fake inputs for rendering the demo panel WITHOUT ROS
or a simulator: a synthetic aerial camera frame (grass, runway, tent, mannequin), a synthetic
2560x1440 photo written into a temporary photo folder under a real "<lat> , <lon>.jpg" name,
matching detector boxes, FakePanelNode, an object with the same attributes the window
reads from demo_panel.PanelNode, and FakeMissionPlanner, a stand-in for the Mission Planner
worker that shows a chosen state and only records button presses (it never touches Windows).
Used by test_panel_offscreen.py and render_panel_preview.py.
Needs numpy and Pillow (python3-numpy, python3-pil); no OpenCV.
"""
from __future__ import annotations

import os
import threading
import time
from collections import deque
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

from sim_demo import mission_planner as mpl
from sim_demo.detections import Box, DetectionFrame
from sim_demo.phases import PhaseTracker
from sim_demo.photos import gps_photo_name


def aerial_scene(width: int, height: int, seed: int = 1, tent_xy=None, person_xy=None,
                 scale: float = 1.0) -> np.ndarray:
    """A top-down 'field' picture (uint8 RGB): grass texture, a runway, optional targets.

    tent_xy / person_xy are pixel centres; scale multiplies the target sizes.
    """
    rng = np.random.default_rng(seed)
    base = np.zeros((height, width, 3), np.float32)
    base[..., 0], base[..., 1], base[..., 2] = 74, 112, 52
    noise = rng.normal(0, 1, (height // 8 + 1, width // 8 + 1, 1)).astype(np.float32)
    noise = np.kron(noise, np.ones((8, 8, 1), np.float32))[:height, :width]
    base += noise * 10 + rng.normal(0, 6, (height, width, 1)).astype(np.float32)
    img = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(1))
    d = ImageDraw.Draw(img)
    rw = int(height * 0.16)
    y0 = int(height * 0.62)
    d.rectangle([0, y0, width, y0 + rw], fill=(92, 92, 96))
    for x in range(0, width, max(40, width // 16)):
        d.rectangle([x, y0 + rw // 2 - 3, x + width // 40, y0 + rw // 2 + 3], fill=(230, 230, 230))
    if tent_xy is not None:
        tx, ty = tent_xy
        s = 0.045 * width * scale
        d.polygon([(tx - s, ty + s * 0.8), (tx + s, ty + s * 0.8), (tx + s * 0.8, ty - s * 0.8),
                   (tx - s * 0.8, ty - s * 0.8)], fill=(255, 140, 20))
        d.line([(tx - s * 0.8, ty - s * 0.8), (tx + s, ty + s * 0.8)], fill=(170, 80, 0), width=3)
        d.line([(tx + s * 0.8, ty - s * 0.8), (tx - s, ty + s * 0.8)], fill=(170, 80, 0), width=3)
    if person_xy is not None:
        px, py = person_xy
        s = 0.012 * width * scale
        d.ellipse([px - 4 * s, py - s, px + 2.6 * s, py + s], fill=(30, 60, 160))     # body
        d.ellipse([px + 2.4 * s, py - 1.1 * s, px + 4.4 * s, py + 1.1 * s], fill=(230, 190, 160))
    return np.asarray(img)


def write_photo(folder: str, lat: float, lon: float, width: int = 2560, height: int = 1440,
                tent_xy=(1500, 610), person_xy=(860, 980)) -> str:
    """Write a synthetic photo named like video_cam does. Returns the file name."""
    os.makedirs(folder, exist_ok=True)
    name = gps_photo_name(lat, lon)
    arr = aerial_scene(width, height, seed=int(abs(lat * 1e6)) % 1000, tent_xy=tent_xy,
                       person_xy=person_xy, scale=1.0)
    tmp = os.path.join(folder, name.replace('.jpg', '.tmp.jpg'))
    Image.fromarray(arr).save(tmp, quality=90)
    os.replace(tmp, os.path.join(folder, name))
    return name


def fake_image_msg(arr: np.ndarray) -> SimpleNamespace:
    """Something shaped like a sensor_msgs/Image (rgb8) for the panel's video view."""
    h, w = arr.shape[:2]
    return SimpleNamespace(encoding='rgb8', width=w, height=h, step=w * 3,
                           data=np.ascontiguousarray(arr).tobytes())


class _Flight:
    def __init__(self):
        self.snap = dict(connected=False, armed=False, mode='', state_t=0.0, lat=None, lon=None,
                         alt_msl=float('nan'), fix_t=0.0, rel_alt=float('nan'),
                         heading=float('nan'), ground_speed=float('nan'), climb=float('nan'),
                         last_reached=-1)

    def snapshot(self):
        return dict(self.snap)


class FakePanelNode:
    """Has every attribute/method DemoWindow uses from demo_panel.PanelNode."""

    def __init__(self, photo_dir: str, world=None, mission_items=None, **cfg):
        self.cfg = dict(world='runway', camera_topic='/sim/camera/image', detector='auto',
                        mavlink_url='', mission_link='mavros', auto_start=False,
                        auto_start_delay_s=5.0, settle_timeout_s=150.0, look_down_on_start=True,
                        look_down_command='gimbal_set_angles 0,-90',
                        look_forward_command='gimbal_set_angles 0,0', hit_hold_s=8.0,
                        window_width=1600, window_height=960, fullscreen=False,
                        screenshot_path='', screenshot_after_s=6.0, screenshot_every_s=0.0,
                        screenshot_count=1, quit_after_screenshot=False, mission_planner_exe='')
        self.cfg.update(cfg)
        self.lock = threading.Lock()
        self.events: deque = deque(maxlen=600)
        self.world = world
        self.world_error = '' if world else 'world "runway" not found (offscreen test)'
        self.mission_items = list(mission_items or [])
        self.phases = PhaseTracker()
        self.photo_dir = photo_dir
        self.frames: list[DetectionFrame] = []
        self.frame_queue: deque = deque()
        self.frame_by_name: dict = {}
        self.last_hit = None
        self.last_hit_t = 0.0
        self.image_msg = None
        self.image_seq = 0
        self.image_times: deque = deque(maxlen=60)
        self.trigger_count = 0
        self.detector_nodes: list[str] = []
        self.flight = _Flight()
        self.mavros = SimpleNamespace(available=lambda: False)
        self.published: list = []

    # same API as PanelNode
    def event(self, source, text, level=''):
        self.events.append((time.time(), source, text, level))

    def take_photo(self):
        self.published.append(('/camera/trigger', True))

    def camera_command(self, text, label):
        self.published.append(('/camera/command', text))

    def get_logger(self):
        return SimpleNamespace(info=print, warn=print, error=print)

    # helpers for tests
    def set_image(self, arr):
        with self.lock:
            self.image_msg = fake_image_msg(arr)
            self.image_seq += 1
            now = time.time()
            self.image_times.extend([now - 0.08, now - 0.04, now])

    def add_frame(self, frame: DetectionFrame):
        with self.lock:
            self.frames.append(frame)
            self.frame_queue.append(frame)
            self.frame_by_name[frame.image_name] = frame
            if frame.boxes:
                self.last_hit, self.last_hit_t = frame, time.time()


def planner_state(kind: str) -> 'mpl.PlannerState':
    """A Mission Planner state as the real worker would report it. kind: 'running',
    'not running', 'opening', 'closing', 'not on Windows', 'not installed' or 'unknown'."""
    if kind == 'running':
        return mpl.PlannerState(True, True, True, '', 'running',
                                'Mission Planner is running (Windows process 46792).')
    if kind == 'not running':
        return mpl.PlannerState(True, True, False, '', 'not running', 'Mission Planner is not running.')
    if kind in ('opening', 'closing'):       # busy: the last check's state + the action
        return replace(planner_state('not running' if kind == 'opening' else 'running'), busy=kind)
    if kind == 'not on Windows':
        return mpl.PlannerState(True, False, None, '', 'not on Windows', mpl.NOT_WSL_MESSAGE)
    if kind == 'not installed':
        return mpl.PlannerState(True, False, None, '', 'not installed', mpl.not_installed_message())
    return mpl.PlannerState(True, False, None, '', 'unknown', mpl.INTEROP_MESSAGE)


class FakeMissionPlanner:
    """Same five methods as mission_planner.MissionPlannerMonitor; shows `state`, records presses
    in `requests` and never starts, stops or asks Windows anything."""

    def __init__(self, state=None):
        self.state = state or planner_state('not running')
        self.requests: list[str] = []
        self.started = False
        self.stopped = False

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def snapshot(self):
        return self.state

    def request_open(self):
        self.requests.append('open')
        return True

    def request_close(self):
        self.requests.append('close')
        return True


def demo_frame(name: str, lat: float, lon: float, stand_in: bool = False) -> DetectionFrame:
    """Detector result matching write_photo()'s default target positions."""
    return DetectionFrame(name, lat, lon,
                          [Box('tent', 0.91, 1500, 610, 240, 200), Box('person', 0.78, 860, 980, 200, 70)],
                          method='truth_detector (stand-in: answer key, not the AI model)' if stand_in
                          else 'sahi+yolo', num_detections=2, received=time.time())
