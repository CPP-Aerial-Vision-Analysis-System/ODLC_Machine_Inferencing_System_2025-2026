"""
Unit tests of the SIYI node's photo-name rule for broken GPS fixes (Task 2 review fix).

test_gps_filename.py -- PURPOSE: run the REAL SIYINode._generate_gps_filename on a stand-in object
(no node, no camera, no disk) and check that a NaN, infinite or out-of-range GPS fix is treated as
"no fix" (no '<lat> , <lon>.jpg' name: video_cam then keeps the SD-card name, and main_controller
ignores the photo for aiming), exactly like the (0, 0) "no lock" fix already was. Before the fix a
NaN fix gave the photo name "nan , nan.jpg", which new_od and main_controller parse back as NaN.

Needs ROS 2 sourced (the module imports rclpy, mavros_msgs, cv_bridge); otherwise skipped:
    python3 -m pytest ros2_ws/src/video_cam/test/test_gps_filename.py -v
"""

import sys
import types
from pathlib import Path

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
pytest.importorskip('cv_bridge')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from video_cam import siyi_node  # noqa: E402


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(text)

    def warn(self, text):
        self.lines.append(text)

    warning = warn


def name_for(lat, lon):
    log = _Log()
    fake = types.SimpleNamespace(latest_gps=types.SimpleNamespace(latitude=lat, longitude=lon),
                                 get_logger=lambda: log)
    return siyi_node.SIYINode._generate_gps_filename(fake)


def test_a_good_fix_names_the_photo_like_before():
    assert name_for(36.2150004, -96.0068) == '36.215000 , -96.006800.jpg'


@pytest.mark.parametrize('lat, lon', [
    (float('nan'), float('nan')), (float('nan'), -96.0), (36.2, float('inf')),
    (float('-inf'), -96.0), (95.0, -96.0), (36.2, 200.0), (0.0, 0.0),
])
def test_a_broken_fix_gives_no_gps_name(lat, lon):
    assert name_for(lat, lon) is None
