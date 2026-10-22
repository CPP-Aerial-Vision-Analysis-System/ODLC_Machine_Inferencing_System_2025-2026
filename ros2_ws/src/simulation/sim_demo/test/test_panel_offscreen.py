"""
sim_demo/test/test_panel_offscreen.py -- PURPOSE: render the demo panel offscreen with fake
inputs (fake camera frame, fake photo in a temp folder, fake detector message, fake Mission
Planner worker) and check what it shows: the PNG files exist with the window size, the newest
photo with its boxes is displayed, the detection table has the rows, the right phase is lit,
the amber stand-in banner appears only when the truth_detector is the detector, and the OPEN /
CLOSE MISSION PLANNER buttons, their status light and hint are visible, readable (not clipped,
also in the smallest window) and clickable only when it makes sense. A press only reaches the
fake worker: these tests never ask or touch Windows. Also checks the raw-image conversion used
for the live view. Skipped when PyQt5 / rclpy are not installed.
Set SIM_DEMO_PREVIEW_DIR to keep the PNGs (e.g. to look at them).
"""
import os

import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def test_ros_image_to_qimage():
    from PyQt5 import QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['t'])
    from sim_demo.panel_widgets import ros_image_to_qimage
    rgb = bytes([255, 0, 0] * 4)                      # 2x2 red
    img = ros_image_to_qimage('rgb8', 2, 2, 6, rgb)
    assert img is not None and img.pixelColor(0, 0).red() == 255
    bgr = ros_image_to_qimage('bgr8', 2, 2, 6, rgb)   # same bytes read as BGR = blue
    assert bgr.pixelColor(1, 1).blue() == 255 and bgr.pixelColor(1, 1).red() == 0
    assert ros_image_to_qimage('mono8', 2, 2, 2, bytes([7, 7, 7, 7])).pixelColor(0, 0).red() == 7
    assert ros_image_to_qimage('16UC1', 2, 2, 4, bytes(8)) is None
    assert ros_image_to_qimage('rgb8', 4, 4, 12, rgb) is None          # too few bytes
    assert app is not None


def test_panel_renders_all_states(tmp_path):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from render_panel_preview import render_all
    from PyQt5 import QtGui
    out = os.environ.get('SIM_DEMO_PREVIEW_DIR') or str(tmp_path)
    res = render_all(out)
    for key in ('waiting', 'searching', 'standin'):
        img = QtGui.QImage(res[key])
        assert not img.isNull(), key
        assert (img.width(), img.height()) == (1600, 960), key
    assert (QtGui.QImage(res['small']).width(), QtGui.QImage(res['small']).height()) == (1200, 760)
    assert res['waiting_banner'] is False
    assert res['searching_photo'].endswith('.jpg')
    assert res['searching_rows'] == 2                  # tent + person rows
    assert res['searching_phase'] == 2                 # "Searching" lit
    assert res['searching_banner'] is False            # real detector: no banner
    assert res['standin_banner'] is True               # stand-in: amber banner
    assert res['standin_phase'] == 4                   # "Flying to target" lit
    # Mission Planner controls visible, and no text in the Controls card clipped (old buttons
    # included), in every state and also in the smallest window.
    for state in ('waiting', 'searching', 'standin', 'small'):
        mp = res[f'{state}_mp']
        assert mp['visible'] and mp['fits'], (state, mp['sizes'])
        assert not QtGui.QImage(res[f'{state}_controls']).isNull(), state
    assert (res['waiting_mp']['open_enabled'], res['waiting_mp']['close_enabled']) == (True, False)
    assert 'not running' in res['waiting_mp']['pill']
    assert (res['searching_mp']['open_enabled'], res['searching_mp']['close_enabled']) == (False, True)
    assert res['searching_mp']['pill'].endswith('Mission Planner: running')
    assert (res['standin_mp']['open_enabled'], res['standin_mp']['close_enabled']) == (False, False)
    assert 'WSL' in res['standin_mp']['open_tip']      # greyed out, and the tooltip says why
    assert 'opening' in res['small_mp']['pill']


def test_mission_planner_buttons(tmp_path):
    """Which Mission Planner button is clickable in each state, and that a press only reaches the
    worker (here a fake; the real one would start/close the program on its own thread)."""
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import panel_fakes as pf
    from PyQt5 import QtWidgets
    from sim_demo.demo_panel import DemoWindow, prepare_qt_environment
    prepare_qt_environment()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['t'])
    node = pf.FakePanelNode(str(tmp_path / 'mapping_photos'))
    fake = pf.FakeMissionPlanner(pf.planner_state('not running'))
    w = DemoWindow(node, app, mission_planner=fake)
    try:
        assert fake.started                            # the window starts its worker
        w.win.show()
        app.processEvents()

        def show(kind):
            fake.state = pf.planner_state(kind)
            w._update_ui()
            return w.btn_mp_open.isEnabled(), w.btn_mp_close.isEnabled()

        assert show('not running') == (True, False)    # closed: only OPEN
        assert 'Start Mission Planner' in w.btn_mp_open.toolTip()
        w.btn_mp_open.click()
        assert fake.requests == ['open']
        assert show('running') == (False, True)        # open: only CLOSE
        assert 'Opening Mission Planner' in w.cmd_status.text()   # the press shows at once
        w.btn_mp_close.click()
        assert fake.requests == ['open', 'close']
        assert show('opening') == (False, False)       # busy: neither
        assert 'opening ...' in w.pill_mp.text()
        assert show('closing') == (False, False)
        for kind, why in (('not on Windows', 'WSL'), ('not installed', 'MISSION_PLANNER_EXE'),
                          ('unknown', 'interop')):
            assert show(kind) == (False, False), kind  # cannot work here: both grey ...
            assert why in w.btn_mp_open.toolTip() and why in w.pill_mp.toolTip(), kind  # ... and why
        # Both presses are in the Events log (each UI tick moves node.events into the log widget).
        w._update_ui()
        log = w.log.toPlainText()
        assert 'OPEN MISSION PLANNER' in log and 'CLOSE MISSION PLANNER' in log
    finally:
        w.close()
        w.win.close()
    assert fake.stopped                                # closing the window stops the worker
