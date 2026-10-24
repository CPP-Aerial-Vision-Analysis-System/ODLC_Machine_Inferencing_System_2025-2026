"""
Safety tests of main_controller's own methods (the Task 2 review fixes), without any ROS graph.

test_controller_safety.py -- PURPOSE: run the REAL MainController methods on a small stand-in
object (no node is created, nothing spins, nothing is sent to a flight controller) and check the
two Task 2 fixes where they live, in the controller itself:
  * the GUIDED processing hold ends at PROCESSING_TIMEOUT_SECONDS even when the photo folder is
    missing or unreadable (before the fix the check raised on every tick and the drone hovered
    until a failsafe), the folder problem is logged once, not every 2 s, and a wall-clock jump
    backwards cannot stretch the hold;
  * a detection whose photo position, box centre or score is NaN / infinite never becomes a
    target, so it can never produce a waypoint or a drop, and _divert_to_targets refuses a target
    whose aim point is not a valid position.

Needs ROS 2 sourced (rclpy, mavros_msgs, interfaces, cv_bridge for the import); otherwise skipped:
    source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh
    python3 -m pytest ros2_ws/src/main/test/test_controller_safety.py -v
"""

import json
import math
import os
import sys
import time
import types
from pathlib import Path

import pytest

pytest.importorskip('rclpy', reason='ROS 2 (rclpy) is not sourced')
pytest.importorskip('interfaces.msg', reason='the interfaces package is not built/sourced')
pytest.importorskip('mavros_msgs.msg', reason='mavros_msgs is not installed')
pytest.importorskip('cv_bridge', reason='cv_bridge is not installed')

from interfaces.msg import ImageResult  # noqa: E402
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import main_controller as mc  # noqa: E402
from main_controller import Detection_Object, MainController  # noqa: E402


class FakeLogger:
    """Collect log lines as (level, text)."""

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    warning = warn

    def error(self, text):
        self.lines.append(('error', text))

    def text(self, level=None):
        return '\n'.join(t for lv, t in self.lines if level in (None, lv))


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


def bind(obj, *names):
    """Attach MainController methods (those that exist in this version) to a stand-in object."""
    for name in names:
        if hasattr(MainController, name):
            setattr(obj, name, types.MethodType(getattr(MainController, name), obj))


def make_controller(photo_dir):
    """A stand-in with exactly the state the tested MainController methods use."""
    c = types.SimpleNamespace()
    c.log = FakeLogger()
    c.get_logger = lambda: c.log
    c.acks, c.modes, c.diverts, c.sent_waypoints = [], [], [], []
    c.send_ack = c.acks.append
    c.change_mode = c.modes.append
    c.camera_feed_path = str(photo_dir)
    c.mission_phase = 'processing'
    c.processed_image_names = set()
    c.processing_check_timer = FakeTimer()
    c.processing_started_at = time.monotonic()
    c.waiting_for_processing = True
    c._photo_folder_error = ''
    c.aim_at_box = True
    c.default_pitch, c.default_hfov, c.ground_height, c.max_aim_offset = -90.0, 81.0, 0.0, 150.0
    c.detections = {'person': Detection_Object('person', 0, 0.0, 0.0),
                    'tent': Detection_Object('tent', 0, 0.0, 0.0)}
    c._latlon_from_name = MainController._latlon_from_name
    bind(c, '_check_all_images_processed', '_end_processing_hold', 'locate_detection',
         'image_result_cb', 'valid_detection')
    return c


def fake_divert(c):
    """Record the divert instead of sending waypoints (and leave the processing phase)."""
    def divert():
        c.diverts.append(time.monotonic())
        c.mission_phase = 'visiting'
    c._divert_to_targets = divert


# ------------------------------------------------------------------ the processing hold
def test_missing_photo_folder_still_ends_the_hold_at_the_timeout(tmp_path):
    c = make_controller(tmp_path / 'not_there')
    fake_divert(c)
    c.processing_started_at = time.monotonic() - 10.0
    c._check_all_images_processed()
    assert c.diverts == []                                  # still waiting before the limit
    c.processing_started_at = time.monotonic() - mc.PROCESSING_TIMEOUT_SECONDS - 1.0
    c._check_all_images_processed()
    assert len(c.diverts) == 1                              # before the fix: never
    assert c.processing_check_timer is None
    assert any('Processing timeout' in a and 'images' in a for a in c.acks)
    assert all(len(a) <= 50 for a in c.acks)                # MAVLink STATUSTEXT limit


@pytest.mark.skipif(not hasattr(os, 'geteuid') or os.geteuid() == 0,
                    reason='needs a non-root POSIX user (root may read any folder)')
def test_unreadable_photo_folder_still_ends_the_hold_at_the_timeout(tmp_path):
    locked = tmp_path / 'mapping_photos'
    locked.mkdir()
    (locked / '36.2 , -96.0.jpg').write_bytes(b'')
    locked.chmod(0)
    try:
        c = make_controller(locked)
        fake_divert(c)
        c.processing_started_at = time.monotonic() - mc.PROCESSING_TIMEOUT_SECONDS - 1.0
        c._check_all_images_processed()
    finally:
        locked.chmod(0o755)
    assert len(c.diverts) == 1
    assert 'PermissionError' in c.log.text('warn')


def test_folder_problem_is_logged_once_not_every_tick(tmp_path):
    c = make_controller(tmp_path / 'not_there')
    fake_divert(c)
    for _ in range(5):
        c._check_all_images_processed()
    assert c.log.text('warn').count('Cannot read the photo folder') == 1
    assert c.log.text('error') == ''                        # the old code: 5 errors


def test_wall_clock_jump_back_does_not_stretch_the_hold(tmp_path, monkeypatch):
    (tmp_path / 'a.jpg').write_bytes(b'')                  # one photo never answered
    c = make_controller(tmp_path)
    fake_divert(c)
    c.mission_phase, c.processing_check_timer = 'survey', None
    c.last_nav_before_rtl, c.last_before_rtl, c.visit_plan, c.visit_idx = 10, 10, [], 0
    c.create_timer = lambda period, cb: FakeTimer()
    bind(c, 'update_waypoint_reached')
    c.update_waypoint_reached(types.SimpleNamespace(wp_seq=10))      # the hold starts
    assert c.mission_phase == 'processing' and c.modes == ['GUIDED']
    real_time, real_mono = time.time(), time.monotonic()
    # 91 s later by the steady clock, but the wall clock was just set back by an hour.
    monkeypatch.setattr(mc.time, 'time', lambda: real_time - 3600.0 + 91.0)
    monkeypatch.setattr(mc.time, 'monotonic', lambda: real_mono + 91.0)
    c._check_all_images_processed()
    assert len(c.diverts) == 1


def test_a_failing_divert_goes_home_instead_of_hovering(tmp_path):
    (tmp_path / 'a.jpg').write_bytes(b'')
    c = make_controller(tmp_path)
    c.processed_image_names = {'a.jpg'}

    def broken():
        raise RuntimeError('bug')
    c._divert_to_targets = broken
    c._check_all_images_processed()
    assert c.modes == ['RTL'] and c.mission_phase == 'done'


# ------------------------------------------------------------------ NaN / infinity
GOOD_LAT, GOOD_LON = 36.2150000, -96.0068000
W, H = 2560, 1440


def detection_msg(name, u, v, score=0.9, class_id='0'):
    msg = ImageResult()
    msg.image_name = name
    d = Detection2D()
    d.bbox.center.position.x = float(u)
    d.bbox.center.position.y = float(v)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis.class_id = class_id
    hyp.hypothesis.score = float(score)
    d.results.append(hyp)
    msg.detections.detections.append(d)
    return msg


def write_photo(folder, name, meta=None):
    """A tiny JPEG header with the 2560x1440 size, plus an optional sidecar."""
    import struct
    sof = b'\xff\xc0' + struct.pack('>HBHH', 11, 8, H, W) + b'\x03\x01\x22\x00'
    (folder / name).write_bytes(b'\xff\xd8' + sof + b'\xff\xd9')
    if meta is not None:
        (folder / (os.path.splitext(name)[0] + '.json')).write_text(json.dumps(meta))


def sidecar(**over):
    m = {'format': 'astra.photo_metadata.v1', 'rel_alt_m': 30.0, 'heading_deg': 0.0,
         'position': {'lat': GOOD_LAT, 'lon': GOOD_LON},
         'camera': {'hfov_deg': 81.0, 'zoom_x': 1.0, 'zoom_changing': False,
                    'image_top_direction_deg': 0.0},
         'gimbal': {'yaw_deg': 0.0, 'pitch_deg': -90.0, 'roll_deg': 0.0}}
    m.update(over)
    return m


def test_a_good_detection_is_kept(tmp_path):
    name = f'{GOOD_LAT:.6f} , {GOOD_LON:.6f}.jpg'
    write_photo(tmp_path, name, sidecar())
    c = make_controller(tmp_path)
    c.image_result_cb(detection_msg(name, W / 2 + 100, H / 2))
    d = c.detections['person']
    assert d.confidence == pytest.approx(0.9)
    assert math.isfinite(d.lat) and math.isfinite(d.long) and d.lat != 0.0


@pytest.mark.parametrize('name, meta', [
    ('nan , nan.jpg', None),                                       # GPS NaN in the file name
    ('inf , -96.006800.jpg', None),                                # GPS infinite
    ('95.000000 , -96.006800.jpg', None),                          # latitude out of range
    ('nan , nan.jpg', sidecar(position={'lat': float('nan'), 'lon': float('nan')})),
])
def test_nonfinite_gps_never_becomes_a_target(tmp_path, name, meta):
    write_photo(tmp_path, name, meta)
    c = make_controller(tmp_path)
    c.image_result_cb(detection_msg(name, W / 2, H / 2))
    assert c.detections['person'].confidence == 0                  # ignored: no target
    assert 'no usable GPS position' in c.log.text('warn')


@pytest.mark.parametrize('u, v', [(float('nan'), 100.0), (100.0, float('inf')),
                                  (float('-inf'), float('nan'))])
def test_nonfinite_box_centre_never_becomes_a_target(tmp_path, u, v):
    name = f'{GOOD_LAT:.6f} , {GOOD_LON:.6f}.jpg'
    write_photo(tmp_path, name, sidecar())
    c = make_controller(tmp_path)
    c.image_result_cb(detection_msg(name, u, v))
    assert c.detections['person'].confidence == 0
    assert 'not a finite number' in c.log.text('warn')


def test_nonfinite_height_or_heading_is_not_used(tmp_path):
    # A NaN height/heading in the sidecar counts as "not recorded": the documented fallback to
    # the photo position, always a finite, valid point.
    name = f'{GOOD_LAT:.6f} , {GOOD_LON:.6f}.jpg'
    write_photo(tmp_path, name, sidecar(rel_alt_m=float('nan'), heading_deg=float('inf')))
    c = make_controller(tmp_path)
    c.image_result_cb(detection_msg(name, 10.0, 10.0))
    d = c.detections['person']
    assert (d.lat, d.long) == pytest.approx((GOOD_LAT, GOOD_LON))
    assert 'photo position' in d.aim_note


@pytest.mark.parametrize('score', [float('inf'), float('nan')])
def test_nonfinite_score_never_becomes_a_target(tmp_path, score):
    name = f'{GOOD_LAT:.6f} , {GOOD_LON:.6f}.jpg'
    write_photo(tmp_path, name, sidecar())
    c = make_controller(tmp_path)
    c.image_result_cb(detection_msg(name, W / 2, H / 2, score=score))
    assert c.detections['person'].confidence == 0
    c.image_result_cb(detection_msg(name, W / 2, H / 2, score=0.5))   # a normal one still wins
    assert c.detections['person'].confidence == pytest.approx(0.5)


def test_divert_refuses_a_target_without_a_valid_position(tmp_path):
    c = make_controller(tmp_path)
    c.last_before_rtl = 10
    c.send_waypoint_data = c.sent_waypoints.append
    c.divert_and_resume = lambda seq: None
    bind(c, '_divert_to_targets')
    c.detections['tent'] = Detection_Object('tent', 0.8, float('nan'), GOOD_LON)
    c._divert_to_targets()
    assert c.sent_waypoints == []                       # no waypoint, so no drop
    assert c.modes == ['RTL']                           # nothing valid left: go home
    assert 'not a valid position' in c.log.text('error')
