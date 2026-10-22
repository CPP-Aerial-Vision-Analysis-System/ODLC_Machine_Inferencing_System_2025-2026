"""
example_listener/test/test_summary.py -- PURPOSE: unit tests of the template listener's bookkeeping (no ROS):
armed time, modes, distance, best detection per class, and the JSON summary.
Run:  python3 -m pytest ros2_ws/src/simulation/sim_plugin_examples -q
"""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from example_listener.summary import FlightSummary, distance_m  # noqa: E402


def test_distance_one_millidegree_latitude_is_about_111_m():
    assert distance_m(36.0, -96.0, 36.001, -96.0) == pytest.approx(111.2, abs=0.5)


def test_flight_bookkeeping():
    s = FlightSummary()
    s.on_state(True, False, 'STABILIZE', now=0.0)
    s.on_state(True, True, 'AUTO', now=10.0)
    s.on_state(True, True, 'AUTO', now=20.0)          # same mode: not repeated
    s.on_state(True, True, 'GUIDED', now=30.0)
    s.on_state(True, False, 'RTL', now=70.0)
    assert s.modes == ['STABILIZE', 'AUTO', 'GUIDED', 'RTL']
    assert s.armed_seconds(now=100.0) == pytest.approx(60.0)
    s.on_fix(36.0, -96.0)
    s.on_fix(36.001, -96.0)
    s.on_fix(0.0, 0.0)                               # no fix yet: ignored
    s.on_fix(37.0, -96.0)                            # a 100 km jump: ignored
    assert s.distance_m == pytest.approx(111.2, abs=0.5)
    s.on_rel_alt(12.0)
    s.on_rel_alt(30.5)
    s.on_rel_alt(float('nan'))
    assert s.max_rel_alt_m == 30.5
    s.on_reached(1)
    s.on_reached(2)
    s.on_trigger(True)
    s.on_trigger(False)
    s.on_trigger(True)
    assert s.shutter_presses == 2
    s.on_statustext('Mission: 3 DigiCamCtrl')


def test_best_detection_per_class():
    s = FlightSummary()
    s.on_detection('a.jpg', [('0', 0.4), ('1', 0.7)])
    s.on_detection('b.jpg', [('0', 0.9)])
    s.on_detection('c.jpg', [])                      # a photo with nothing in it still counts as reported
    assert s.photos_reported == 3
    assert s.detections == {'person': 2, 'tent': 1}
    assert s.best['person'] == (0.9, 'b.jpg') and s.best['tent'] == (0.7, 'a.jpg')
    text = s.text(now=0.0)
    assert 'person 0.90 (b.jpg)' in text and 'photos reported: 3' in text
    data = s.as_dict(now=0.0)
    json.dumps(data)                                 # JSON-friendly
    assert data['best']['tent'] == {'score': 0.7, 'image': 'a.jpg'}
