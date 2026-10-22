"""
sim_demo/test/test_truth_model.py -- PURPOSE: unit tests of the stand-in detector's logic
(truth_model.py) and of photo_geometry.py: a target under the camera is found with a box at
the right pixel and a sensible confidence, targets outside the photo or too small are not,
results are repeatable, zoom changes what is visible, and confidences behave sensibly.
"""
import math

import pytest

from sim_demo.camera_model import CameraIntrinsics, CameraPose
from sim_demo.photo_geometry import PhotoGeometry
from sim_demo.photos import PhotoEvent
from sim_demo.pose_history import PoseHistory, PoseSample
from sim_demo.truth_model import TruthModelConfig, confidence_for, detect_targets, pose_from_sample
from sim_demo.world_info import TruthTarget
from sim_demo.zoom import ZoomTracker

INTR = CameraIntrinsics(2560, 1440, 1.4137, 1.0)
TENT = TruthTarget('tent', 'tent', 0, 0, east=0.0, north=0.0, size=(2.4, 2.1, 1.4))
HIKER = TruthTarget('hiker', 'person', 0, 0, east=10.0, north=5.0, size=(1.8, 0.5, 0.3), posture='lying')
ALWAYS = TruthModelConfig(detect_probability=1.0)


def test_target_below_is_found_at_image_centre():
    hits = detect_targets(INTR, CameraPose(0, 0, 40), [TENT], 'p1.jpg', ALWAYS)
    assert len(hits) == 1
    b = hits[0].box
    assert b.cls == 'tent'
    assert b.cx == pytest.approx(1280, abs=15) and b.cy == pytest.approx(720, abs=15)
    assert 0.6 < b.score <= 0.97
    assert hits[0].visible_fraction == pytest.approx(1.0)


def test_hiker_offset_appears_right_and_up_when_heading_north():
    hits = detect_targets(INTR, CameraPose(0, 0, 40, yaw_deg=0.0), [HIKER], 'p2.jpg', ALWAYS)
    assert len(hits) == 1
    assert hits[0].box.cx > 1280 and hits[0].box.cy < 720       # east = right, north = up
    assert hits[0].box.cls == 'person'


def test_outside_and_too_small_targets_are_not_reported():
    far = TruthTarget('far', 'tent', 0, 0, east=300.0, north=0.0, size=(2.4, 2.1, 1.4))
    assert detect_targets(INTR, CameraPose(0, 0, 40), [far], 'p3.jpg', ALWAYS) == []
    # From 400 m up a lying 0.5 x 1.8 m mannequin is ~9 px long: below min_pixels.
    tiny = detect_targets(INTR, CameraPose(10, 5, 400), [HIKER], 'p4.jpg', ALWAYS)
    assert tiny == []


def test_zoom_shrinks_what_is_visible():
    pose = CameraPose(0, 0, 40)
    assert len(detect_targets(INTR, pose, [HIKER], 'p5.jpg', ALWAYS)) == 1
    z5 = CameraIntrinsics(2560, 1440, 1.4137, 5.0)
    assert detect_targets(z5, pose, [HIKER], 'p5.jpg', ALWAYS) == []   # 11 m off-centre: outside
    assert len(detect_targets(z5, pose, [TENT], 'p5.jpg', ALWAYS)) == 1


def test_repeatable_and_seed_dependent():
    a = detect_targets(INTR, CameraPose(0, 0, 40), [TENT, HIKER], 'same.jpg', ALWAYS)
    b = detect_targets(INTR, CameraPose(0, 0, 40), [TENT, HIKER], 'same.jpg', ALWAYS)
    assert [h.box for h in a] == [h.box for h in b]
    c = detect_targets(INTR, CameraPose(0, 0, 40), [TENT, HIKER], 'same.jpg',
                       TruthModelConfig(detect_probability=1.0, seed=7))
    assert [h.box.score for h in a] != [h.box.score for h in c]


def test_detect_probability_zero_reports_nothing():
    cfg = TruthModelConfig(detect_probability=0.0)
    assert detect_targets(INTR, CameraPose(0, 0, 40), [TENT], 'x.jpg', cfg) == []


def test_confidence_model_shape():
    cfg = TruthModelConfig()
    big_centre = confidence_for(cfg, 'tent', 200, 1.0, 0.0)
    small = confidence_for(cfg, 'tent', 14, 1.0, 0.0)
    cut = confidence_for(cfg, 'tent', 200, 0.5, 0.0)
    edge = confidence_for(cfg, 'tent', 200, 1.0, 1.0)
    assert big_centre > small and big_centre > cut and big_centre > edge
    assert cfg.min_confidence <= small <= cfg.max_confidence
    assert confidence_for(cfg, 'tent', 200, 1.0, 0.0, noise=1.0) == cfg.max_confidence


def test_pose_from_sample_uses_height_and_heading(world):
    s = PoseSample(0.0, *world.origin.to_geo(20.0, 30.0)[:2], rel_alt=45.0, heading_deg=270.0)
    pose = pose_from_sample(world.origin, s)
    assert (pose.east, pose.north, pose.up) == (pytest.approx(20.0), pytest.approx(30.0), 45.0)
    assert pose.yaw_deg == 270.0 and pose.pitch_deg == -90.0
    nan_sample = PoseSample(0.0, world.origin.lat, world.origin.lon)
    assert pose_from_sample(world.origin, nan_sample, default_height=33.0).up == 33.0


class _FakeFlight:
    def __init__(self, history):
        self.history = history
        self.zoom = ZoomTracker()

    def capture_pose(self, origin, lat, lon, file_time, latency_s=1.0):
        return self.history.capture_pose(origin, lat, lon, file_time, latency_s=latency_s)


def test_photo_geometry_end_to_end(world, tmp_path):
    h = PoseHistory()
    lat, lon, _ = world.origin.to_geo(30.0, 40.0)        # right above the hiker
    for i in range(20):
        h.add(PoseSample(100.0 + i * 0.25, lat, lon, rel_alt=30.0, heading_deg=0.0))
    flight = _FakeFlight(h)
    flight.zoom.on_statustext(90.0, 'Zoom set to 2.0x')
    geo = PhotoGeometry(world.origin, flight)
    ev = PhotoEvent(str(tmp_path / 'none.jpg'), 'p.jpg', 104.0, lat, lon, 1000)
    sol = geo.solve(ev)
    assert sol.how == 'gps-match' and sol.intr.zoom == 2.0
    assert sol.intr.width == 2560                        # default size (file unreadable)
    width_m = 2 * 30.0 * math.tan(1.4137 / 2) / 2.0
    xs = [p[0] for p in sol.footprint]
    assert max(xs) - min(xs) == pytest.approx(width_m, rel=1e-6)
    hits = detect_targets(sol.intr, sol.pose, world.targets, ev.name, ALWAYS)
    assert [hh.target.name for hh in hits] == ['hiker']
    pre = PhotoEvent('x', 'old.jpg', 104.0, lat, lon, 1000, preexisting=True)
    assert geo.solve(pre).pose is None
