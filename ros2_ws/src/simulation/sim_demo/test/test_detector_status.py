"""
sim_demo/test/test_detector_status.py -- PURPOSE: unit tests of demo_panel.detector_status(), which
decides the DETECTOR pill and the amber stand-in banner from the messages actually received, so an
idle new_od (sim_bringup's detector:=truth mode) never counts as a second detector.
"""
import pytest

pytest.importorskip('rclpy')

from sim_demo.demo_panel import detector_status  # noqa: E402
from sim_demo.detections import DetectionFrame  # noqa: E402

TRUTH = DetectionFrame('a.jpg', 1, 1, method='truth_detector (stand-in: answer key, not the AI model)')
REAL = DetectionFrame('b.jpg', 1, 1, method='sahi+yolo')


def test_idle_new_od_is_not_a_second_detector():
    state, text, tip, banner = detector_status(['new_od', 'truth_detector'], [TRUTH])
    assert (state, text, banner) == ('warn', 'STAND-IN', True) and 'idle' in tip


def test_two_detectors_really_publishing():
    assert detector_status(['new_od', 'truth_detector'], [TRUTH, REAL])[:2] == ('bad', 'TWO detectors!')


def test_real_only_and_nothing_yet():
    assert detector_status(['new_od'], [REAL])[:2] == ('ok', 'REAL (new_od)')
    assert detector_status(['new_od'], [])[3] is False
    assert detector_status(['truth_detector'], [])[3] is True
    assert detector_status([], [])[:2] == ('off', 'not running')
