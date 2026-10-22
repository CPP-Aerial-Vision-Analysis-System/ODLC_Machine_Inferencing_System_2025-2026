"""
Unit tests for video_cam/capture_queue.py and the camera capability table.

test_capture_queue.py -- PURPOSE: unit tests for the waiting line for photo requests that replaced
the "drop the trigger while the lens is busy" logic (October 2026), and for the per-camera
capability table in video_cam/config.py (which commands each SIYI model answers, so an A8 mini is
never asked to autofocus). Checks: first-in-first-out order, nothing lost while the queue has room,
the OLDEST request is dropped and counted when it is full, clear() counts discarded requests, the
statistics line, and the capability / field-of-view lookups.

Needs nothing but Python (no ROS, no build):
    python3 -m pytest ros2_ws/src/video_cam/test/test_capture_queue.py -v
"""

import os
import sys
import threading

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from video_cam.capture_queue import CaptureRequestQueue  # noqa: E402
from video_cam.config import (  # noqa: E402
    DEFAULT_HFOV_DEG,
    camera_capabilities,
    camera_hfov_deg,
)


def test_first_in_first_out():
    q = CaptureRequestQueue(max_pending=5)
    for t in (1.0, 2.0, 3.0):
        assert q.push(t) is True
    assert [q.pop(), q.pop(), q.pop(), q.pop()] == [1.0, 2.0, 3.0, None]
    assert q.received == 3 and q.served == 3 and q.dropped == 0


def test_nothing_is_lost_while_there_is_room():
    # 1 Hz triggers while one photo takes ~0.6 s: the line never grows past 1 and nothing drops.
    q = CaptureRequestQueue(max_pending=5)
    for second in range(90):
        q.push(float(second))
        assert q.pop() == float(second)
    assert q.dropped == 0 and q.served == 90 and q.max_depth == 1


def test_full_queue_drops_the_oldest_and_keeps_the_newest():
    q = CaptureRequestQueue(max_pending=3)
    for t in (1.0, 2.0, 3.0):
        q.push(t)
    assert q.push(4.0) is False                  # full: 1.0 is dropped
    assert q.dropped == 1 and len(q) == 3
    assert [q.pop(), q.pop(), q.pop()] == [2.0, 3.0, 4.0]


def test_clear_counts_discarded_requests_separately():
    q = CaptureRequestQueue(max_pending=5)
    q.push(1.0)
    q.push(2.0)
    assert q.clear() == 2
    assert q.discarded == 2 and q.dropped == 0 and q.pending == 0


def test_stats_text_mentions_every_counter():
    q = CaptureRequestQueue(max_pending=1)
    q.push(1.0)
    q.push(2.0)
    q.pop()
    text = q.stats_text()
    assert '2 triggers' in text and '1 served' in text and '1 dropped' in text


def test_thread_safe_pushes():
    q = CaptureRequestQueue(max_pending=10000)

    def worker():
        for i in range(500):
            q.push(float(i))
    threads = [threading.Thread(target=worker) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert q.received == 2000 and len(q) == 2000 and q.dropped == 0


def test_minimum_size_is_one():
    q = CaptureRequestQueue(max_pending=0)
    assert q.max_pending == 1


# ------------------------------------------------------------------ camera capabilities
def test_a8_mini_has_no_focus_commands_and_no_zoom_readback():
    caps = camera_capabilities('A8MINI')
    assert caps['autofocus'] is False          # SIYI table: 0x04 only on optical-zoom cameras
    assert caps['manual_focus'] is False       # 0x06 likewise
    assert caps['zoom_readback'] is False      # 0x18 not answered


def test_zr10_can_do_everything():
    caps = camera_capabilities('ZR10')
    assert caps['autofocus'] and caps['manual_focus'] and caps['zoom_readback']


def test_unknown_model_keeps_the_old_behaviour():
    caps = camera_capabilities('SOMETHING_NEW')
    assert caps['autofocus'] and caps['manual_focus'] and caps['zoom_readback']


def test_field_of_view_lookup():
    assert camera_hfov_deg('A8MINI') == 81.0
    assert camera_hfov_deg('ZR10') > 0
    assert camera_hfov_deg('NOT_A_CAMERA') == DEFAULT_HFOV_DEG
