"""
sim_demo/test/test_payloads_zoom.py -- PURPOSE: unit tests of payloads.py (fall and landing
detection from Gazebo pose samples, payload name -> class) and zoom.py (zoom history from the
emulator, the team's "Zoom set to" status text and /camera/command).
"""
import pytest

from sim_demo.payloads import GzPayloadSubscriber, PayloadTracker, payload_class
from sim_demo.zoom import ZoomTracker


def test_payload_class_names():
    assert payload_class('payload_tent') == 'tent'
    assert payload_class('payload_person') == 'person'
    assert payload_class('tent') is None              # the TARGET tent is not a payload
    assert payload_class('payload_x') is None
    assert GzPayloadSubscriber.model_name_for('/sim/payload/tent/odometry') == 'payload_tent'


def test_fall_and_land():
    tr = PayloadTracker()
    t = 0.0
    for _ in range(20):                       # carried by the drone, slow descent (1 m/s)
        tr.add(t, 'payload_tent', 5.0, 5.0, 40.0 - t)
        t += 0.1
    assert tr.landing_points() == {}
    z, vz = 38.0, 0.0
    while z > 0.05:                           # free fall
        vz -= 9.81 * 0.1
        z = max(0.05, z + vz * 0.1)
        tr.add(t, 'payload_tent', 5.3, 4.9, z)
        t += 0.1
    for _ in range(15):                       # resting on the ground
        tr.add(t, 'payload_tent', 5.3, 4.9, 0.05)
        t += 0.1
    lp = tr.landing_points()
    assert 'payload_tent' in lp
    _, x, y, zz = lp['payload_tent']
    assert (x, y, zz) == (pytest.approx(5.3), pytest.approx(4.9), pytest.approx(0.05))
    snap = tr.snapshot()[0]
    assert snap.cls == 'tent' and snap.falling_since is not None


def test_other_models_ignored():
    tr = PayloadTracker()
    tr.add(0.0, 'iris_a8mini', 0, 0, 0)
    assert tr.snapshot() == []


def test_zoom_sources():
    z = ZoomTracker()
    assert z.zoom_at(10.0) == (1.0, ZoomTracker.SOURCE_DEFAULT)
    assert z.on_statustext(5.0, 'Zoom set to 5.0x')
    assert not z.on_statustext(6.0, 'Focus set (infinity)')
    assert z.zoom_at(4.0)[0] == 1.0 and z.zoom_at(5.5) == (5.0, ZoomTracker.SOURCE_STATUS)
    assert z.on_camera_command(7.0, 'zoom_absolute 2.5')
    assert z.zoom_at(8.0) == (2.5, ZoomTracker.SOURCE_COMMAND)
    z.on_emulator_zoom(1.0, 3.0)             # the emulator's own report wins over everything else
    assert z.zoom_at(8.0) == (3.0, ZoomTracker.SOURCE_EMULATOR)
    assert ZoomTracker(fixed_zoom=4.0).zoom_at(0.0) == (4.0, ZoomTracker.SOURCE_FIXED)
