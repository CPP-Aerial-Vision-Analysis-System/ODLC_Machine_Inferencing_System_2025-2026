"""
sim_demo/test/test_panel_offscreen.py -- PURPOSE: render the demo panel offscreen with fake
inputs (fake camera frame, fake photo in a temp folder, fake detector message, fake Mission
Planner worker) and check what it shows: the PNG files exist with the window size, the newest
photo with its boxes is displayed, the detection table has the rows, the right phase is lit,
the amber stand-in banner appears only when the truth_detector is the detector, and the OPEN /
CLOSE MISSION PLANNER buttons, their status light and hint are visible, readable (not clipped,
also in the smallest window) and clickable only when it makes sense. A press only reaches the
fake worker: these tests never ask or touch Windows. START OVER: two clicks within 5 s start
it (a recording fake instead of start_over.sh), its progress lines (written exactly like
run_reset.py writes them) reach the Events list and the status line, the mission buttons wait
for it, the old run leaves the display, failures and lockouts are explained. Also checks the
raw-image conversion used for the live view. Skipped when PyQt5 / rclpy are not installed.
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
    # START OVER: visible in every state; its text fits (checked with the Controls card above) also in the
    # smallest window while it runs; the window keeps its size (the extra row does not grow it).
    for state in ('waiting', 'searching', 'standin', 'small', 'restarting', 'small_restarting'):
        so = res[f'{state}_so']
        assert so['reset_visible'], state
    for state in ('waiting', 'searching', 'standin', 'small'):
        assert (res[f'{state}_so']['reset_text'], res[f'{state}_so']['reset_enabled']) == ('START OVER', True)
    assert res['small_so']['size'] == (1200, 760) and res['small_restarting_so']['size'] == (1200, 760)
    for state in ('restarting', 'small_restarting'):
        mp, so = res[f'{state}_mp'], res[f'{state}_so']
        assert mp['visible'] and mp['fits'], (state, mp['sizes'])
        assert res[f'{state}_rows_before'] == 2 and so['rows'] == 0     # the old run left the display
        assert res[f'{state}_launches'] == [['--no-fresh']]            # 2 clicks = 1 start (never a fresh demo)
        assert so['reset_text'] == 'STARTING OVER 9/14 ...' and not so['reset_enabled']
        assert not so['start_enabled']                                  # mission buttons wait for it
        assert 'START OVER 9/14' in so['status']
    assert res['armed_so']['reset_text'].startswith('CLICK AGAIN TO START OVER') and res['armed_so']['reset_enabled']
    assert res['armed_mp']['fits'], res['armed_mp']['sizes']


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
    w = DemoWindow(node, app, mission_planner=fake, start_over=pf.start_over_link(str(tmp_path)))
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


def _window(tmp_path, launches, pid=0, launcher_error=None):
    """A panel with an old run on screen (one photo + its detections) and a recording START OVER link."""
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import panel_fakes as pf
    from PyQt5 import QtWidgets
    from sim_demo.demo_panel import DemoWindow, prepare_qt_environment
    prepare_qt_environment()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['t'])
    folder = str(tmp_path / 'mapping_photos')
    name = pf.write_photo(folder, 36.2146, -96.0064)
    node = pf.FakePanelNode(folder, mission_items=['item'])           # a mission: UPLOAD MISSION + START works
    node.add_frame(pf.demo_frame(name, 36.2146, -96.0064))
    link = pf.start_over_link(str(tmp_path), launches, pid=pid)
    if launcher_error is not None:
        def broken(_args):
            raise launcher_error
        link._launcher = broken
    w = DemoWindow(node, app, mission_planner=pf.FakeMissionPlanner(), start_over=link)
    w.win.show()
    for _ in range(3):
        app.processEvents()
        w._poll_photos()
        w._update_ui()
    return w, node, pf, app


def test_start_over_button_needs_two_clicks_and_follows_the_progress(tmp_path):
    """START OVER: one click only asks; a second click within 5 s starts start_over.sh (here: a recording fake,
    always with --no-fresh); its progress lines appear in the Events list and the status line; the mission
    buttons are locked meanwhile; the old run leaves the display at its start; at its end everything unlocks."""
    launches = []
    w, node, pf, app = _window(tmp_path, launches)
    try:
        assert w.table.rowCount() == 2 and w.btn_start.isEnabled()
        assert w.btn_reset.text() == 'START OVER' and w.btn_reset.isEnabled()
        w.btn_reset.click()                                            # 1st click: nothing starts yet
        w._update_ui()
        assert launches == [] and 'again within 5 s' in w.cmd_status.text()
        assert w.btn_reset.text().startswith('CLICK AGAIN TO START OVER') and w.btn_reset.property('armed')
        w._reset_armed_until = 1.0                                     # ... 5 s pass without a 2nd click
        w._update_ui()
        assert w.btn_reset.text() == 'START OVER' and not w.btn_reset.property('armed')
        assert 'not confirmed' in w.cmd_status.text() and launches == []
        w.btn_reset.click()
        w.btn_reset.click()                                            # 2 clicks within 5 s: it starts
        w._update_ui()
        assert launches == [['--no-fresh']]
        assert not w.btn_reset.isEnabled() and not w.btn_start.isEnabled()   # locked until it reports back
        w.btn_reset.click()
        w._on_start()                                                  # (a press that slips through: refused)
        w._update_ui()
        assert launches == [['--no-fresh']] and 'still running' in w.cmd_status.text()

        prog = pf.progress_writer(str(tmp_path), 'r1', 14)
        prog.start()
        prog.step('check', 1)
        prog.say('ok', 'simulator found: world runway, session demo-1')
        w._poll_start_over()
        w._update_ui()
        assert w.table.rowCount() == 0 and node.frames == [] and node.resets == 1   # old run left the display
        assert w.btn_reset.text() == 'STARTING OVER 1/14 ...' and not w.btn_start.isEnabled()
        prog.step('land', 4)
        prog.detail('landing: height 12.0 m, mode LAND, armed')
        prog.say('warn', 'the drone did not land within 300 s')
        w._poll_start_over()
        w._update_ui()
        assert 'STARTING OVER 4/14' in w.btn_reset.text()
        prog.run_dir = str(tmp_path / 'runs' / '20261024-101010_runway')
        prog.finish(True, 'ready to fly again. Press UPLOAD MISSION + START in the demo panel ...')
        w._poll_start_over()
        w._update_ui()
        assert w.btn_reset.text() == 'START OVER' and w.btn_reset.isEnabled() and w.btn_start.isEnabled()
        assert 'START OVER done: ready' in w.cmd_status.text() and '20261024-101010_runway' in w.cmd_status.text()
        assert node.resets == 2                                        # the new run starts clean too
        log = w.log.toPlainText()
        for text in ('START OVER clicked once', 'START OVER confirmed', 'STEP 1/14', 'simulator found',
                     'STEP 4/14', 'did not land within 300 s', 'START OVER done'):
            assert text in log, text
        assert 'landing: height 12.0 m' not in log                     # tool details: status line only
    finally:
        w.close()
        w.win.close()


def test_start_over_failure_and_lockouts(tmp_path):
    """A failed START OVER unlocks the panel with its reason; START OVER waits while UPLOAD MISSION + START
    runs; a launcher that cannot start says so; a script that ends without any progress is reported."""
    launches = []
    w, node, pf, app = _window(tmp_path, launches)
    try:
        w.commander.busy = True                                       # UPLOAD MISSION + START is running
        w._update_ui()
        assert not w.btn_reset.isEnabled() and 'UPLOAD MISSION + START' in w.btn_reset.toolTip()
        w._on_reset()
        w._update_ui()
        assert launches == [] and 'still running' in w.cmd_status.text()
        w.commander.busy = False
        w._update_ui()
        assert w.btn_reset.isEnabled()
        # a START OVER started in a TERMINAL (start_over.sh) also locks the panel's mission buttons
        prog = pf.progress_writer(str(tmp_path), 'r2', 14)
        prog.start()
        prog.step('check', 1)
        w._poll_start_over()
        w._update_ui()
        assert not w.btn_start.isEnabled() and not w.btn_reset.isEnabled()
        prog.finish(False, 'Gazebo did not remove the drone (service /world/.../remove failed).')
        w._poll_start_over()
        w._update_ui()
        assert w.btn_start.isEnabled() and w.btn_reset.isEnabled()
        assert 'START OVER FAILED: Gazebo did not remove the drone' in w.cmd_status.text()
        assert '[start over] START OVER FAILED' in w.log.toPlainText()
    finally:
        w.close()
        w.win.close()
    # the launcher cannot start at all
    w, node, pf, app = _window(tmp_path, [], launcher_error=OSError('bash not found'))
    try:
        w.btn_reset.click()
        w.btn_reset.click()
        w._update_ui()
        assert 'START OVER could not start: bash not found' in w.cmd_status.text() and w.btn_reset.isEnabled()
    finally:
        w.close()
        w.win.close()
    # the script ends before writing any progress (e.g. "sim_demo is not built"): its last log line is shown
    log_dir = tmp_path / 'logs'
    log_dir.mkdir(exist_ok=True)
    (log_dir / 'start_over_test_panel.log').write_text('==> START OVER\n\x1b[1;31m XX sim_demo is not built\x1b[0m\n')
    w, node, pf, app = _window(tmp_path, [], pid=4194305)   # above Linux's largest pid: "it has ended"
    try:
        w.btn_reset.click()
        w.btn_reset.click()
        w._poll_start_over()
        w._update_ui()
        assert 'START OVER did not start: XX sim_demo is not built' in w.cmd_status.text()
        assert w.btn_reset.isEnabled() and w.btn_start.isEnabled()
    finally:
        w.close()
        w.win.close()


def test_panel_started_during_reset_keeps_mission_controls_locked(tmp_path):
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import panel_fakes as pf
    prog = pf.progress_writer(str(tmp_path), 'active-before-panel', 14)
    prog.start()
    prog.step('land', 4)
    w, _node, _pf, _app = _window(tmp_path, [])
    try:
        assert not w.btn_start.isEnabled() and not w.btn_reset.isEnabled()
        assert w.btn_reset.text() == 'STARTING OVER 4/14 ...'
        assert 'Bring the drone down' in w.cmd_status.text()
        assert 'STEP 4/14' not in w.log.toPlainText()       # history was used for state, not replayed
    finally:
        w.close()
        w.win.close()
