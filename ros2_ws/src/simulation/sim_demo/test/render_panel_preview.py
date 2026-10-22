"""
sim_demo/test/render_panel_preview.py -- PURPOSE: draw the demo panel OFFSCREEN with fake
inputs and save PNG screenshots, so the look of the window can be checked (by a person or a
test) without ROS, a simulator or a screen. Four states are rendered:
    panel_waiting.png    nothing running yet ("waiting for ..."), Mission Planner not running
    panel_searching.png  searching with the REAL detector: live view, newest photo with boxes;
                         Mission Planner running
    panel_standin.png    flying to a target with the truth_detector stand-in (amber banner);
                         not inside WSL, so the Mission Planner buttons are greyed out
    panel_small.png      the smallest window (1200 x 760) while Mission Planner is opening
plus a close-up of the Controls card for each (controls_<state>.png). The Mission Planner
worker is a fake (panel_fakes.FakeMissionPlanner): rendering never asks or touches Windows.

Run:  python3 test/render_panel_preview.py OUTPUT_DIR      (from the sim_demo folder)
"""
from __future__ import annotations

import os
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)


def _pump(app, window, until=None, timeout=4.0):
    end = time.time() + timeout
    while time.time() < end:
        app.processEvents()
        window._poll_photos()
        window._update_video()
        window._update_ui()
        if until is not None and until():
            break
        time.sleep(0.05)
    for _ in range(5):
        app.processEvents()


def mission_planner_facts(window) -> dict:
    """What the Mission Planner controls show, and whether EVERY text in the Controls card fits
    inside its widget (sizeHint = the width the text needs, padding included; wider than the
    widget = clipped). All of the card's buttons are checked, so a new row can never squeeze
    the older ones."""
    from PyQt5 import QtWidgets
    mp = [window.btn_mp_open, window.btn_mp_close, window.mp_hint, window.pill_mp]
    texts = window.controls_card.findChildren(QtWidgets.QPushButton) + mp[2:]
    return dict(open_enabled=window.btn_mp_open.isEnabled(),
                close_enabled=window.btn_mp_close.isEnabled(),
                pill=window.pill_mp.text(), open_tip=window.btn_mp_open.toolTip(),
                visible=all(w.isVisible() for w in mp),
                fits=all(w.sizeHint().width() <= w.width() for w in texts),
                sizes={w.text(): (w.sizeHint().width(), w.width()) for w in texts})


def _shoot(window, out_dir: str, state: str, results: dict) -> None:
    """Save the window and a close-up of its Controls card; note the Mission Planner facts."""
    path = os.path.join(out_dir, f'panel_{state}.png')
    window.win.grab().save(path)
    crop = os.path.join(out_dir, f'controls_{state}.png')
    window.controls_card.grab().save(crop)
    results[state] = path
    results[f'{state}_controls'] = crop
    results[f'{state}_mp'] = mission_planner_facts(window)


def render_all(out_dir: str, worlds_dir: str | None = None) -> dict:
    """Render the four states; returns {state: png_path} plus the window objects' facts."""
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from sim_demo.demo_panel import DemoWindow, prepare_qt_environment
    from sim_demo.mission_file import analyze_items, load_waypoints_file
    from sim_demo.phases import FLYING_TO_TARGET, SEARCHING
    from sim_demo.world_info import load_world
    from PyQt5 import QtWidgets

    import panel_fakes as pf

    prepare_qt_environment()
    os.makedirs(out_dir, exist_ok=True)
    worlds_dir = worlds_dir or os.path.join(HERE, 'fixtures', 'worlds')
    world = load_world('testfield', worlds_dir)
    items = load_waypoints_file(world.mission_path)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['render_panel_preview'])
    results = {}

    # ---------------------------------------------------------------- 1. waiting
    with tempfile.TemporaryDirectory() as photos:
        node = pf.FakePanelNode(os.path.join(photos, 'mapping_photos'), world, items)
        w = DemoWindow(node, app, mission_planner=pf.FakeMissionPlanner(pf.planner_state('not running')))
        w.win.resize(1600, 960)
        w.win.show()
        _pump(app, w, timeout=0.8)
        _shoot(w, out_dir, 'waiting', results)
        results['waiting_banner'] = w.banner.isVisible()
        w.close()
        w.win.close()

    # ---------------------------------------------------------------- 2. searching (real model)
    with tempfile.TemporaryDirectory() as photos:
        folder = os.path.join(photos, 'mapping_photos')
        lat, lon, _ = world.origin.to_geo(31.0, 38.0)
        older = pf.write_photo(folder, *world.origin.to_geo(5.0, 30.0)[:2], tent_xy=None,
                               person_xy=None)
        time.sleep(0.05)
        name = pf.write_photo(folder, lat, lon)
        node = pf.FakePanelNode(folder, world, items)
        node.detector_nodes = ['new_od']
        node.flight.snap.update(connected=True, armed=True, mode='AUTO', state_t=time.time(),
                                lat=lat, lon=lon, rel_alt=30.2, heading=91.0, ground_speed=5.1)
        t0 = time.time() - 95
        ph = node.phases
        ph.set_mission(analyze_items(items))
        ph.on_state(t0, True, True, 'AUTO')
        ph.on_statustext(t0 + 2, 'Mission: 1 Takeoff')
        ph.on_statustext(t0 + 14, 'Mission: 2 DigiCamCtrl')
        ph.on_reached(t0 + 40, 4)
        ph.on_reached(time.time() - 12, 5)
        assert ph.phase == SEARCHING
        for txt, src in (('Mission: 1 Takeoff', 'autopilot'), ('Mission: 2 DigiCamCtrl', 'autopilot'),
                         ('Camera trigger STARTED', 'team'), ('Zoom set to 5.0x', 'team'),
                         ('Mission: 5 WP', 'autopilot'), ('Detected tent', 'team'),
                         ('Detected person', 'team')):
            node.event(src, txt)
        node.add_frame(pf.DetectionFrame(older, *world.origin.to_geo(5.0, 30.0)[:2], [],
                                         method='sahi+yolo', received=time.time() - 3))
        node.add_frame(pf.demo_frame(name, lat, lon, stand_in=False))
        node.event('detector', f'TENT 91%, PERSON 78% in {name}')
        node.set_image(pf.aerial_scene(1280, 720, seed=3, tent_xy=(700, 260), person_xy=None,
                                       scale=0.8))
        w = DemoWindow(node, app, mission_planner=pf.FakeMissionPlanner(pf.planner_state('running')))
        w.win.resize(1600, 960)
        w.win.show()
        w.new_photos = 2
        _pump(app, w, until=lambda: w.shown_photo == name)
        _shoot(w, out_dir, 'searching', results)
        results.update(searching_photo=w.shown_photo,
                       searching_rows=w.table.rowCount(), searching_phase=w.phase_strip.current,
                       searching_banner=w.banner.isVisible())
        w.close()
        w.win.close()

    # ---------------------------------------------------------------- 3. stand-in, flying to target
    with tempfile.TemporaryDirectory() as photos:
        folder = os.path.join(photos, 'mapping_photos')
        lat, lon, _ = world.origin.to_geo(69.0, 41.0)
        name = pf.write_photo(folder, lat, lon, tent_xy=(1300, 700), person_xy=(600, 400))
        node = pf.FakePanelNode(folder, world, items)
        node.detector_nodes = ['truth_detector']
        node.flight.snap.update(connected=True, armed=True, mode='AUTO', state_t=time.time(),
                                lat=lat, lon=lon, rel_alt=49.0, heading=181.0, ground_speed=7.4)
        t0 = time.time() - 300
        ph = node.phases
        ph.on_state(t0, True, True, 'AUTO')
        ph.on_statustext(t0 + 10, 'Mission: 2 DigiCamCtrl')
        ph.on_statustext(t0 + 200, 'Reached last WP. Holding to finish image processing before RTL')
        ph.on_statustext(t0 + 230, 'All 182 images processed. Selecting targets.')
        ph.on_statustext(time.time() - 20, 'Targets: tent@9 -> person@10. Diverting.')
        assert ph.phase == FLYING_TO_TARGET
        frame = pf.DetectionFrame(name, lat, lon,
                                  [pf.Box('tent', 0.88, 1300, 700, 230, 190),
                                   pf.Box('person', 0.74, 600, 400, 190, 70)],
                                  method='truth_detector (stand-in: answer key, not the AI model)',
                                  received=time.time() - 30)
        node.add_frame(frame)
        node.last_hit_t = time.time() - 60          # older than hit_hold_s: Auto shows newest
        for txt, src in (('Reached last WP. Holding to finish image processing before RTL', 'team'),
                         ('All 182 images processed. Selecting targets.', 'team'),
                         ('Targets: tent@9 -> person@10. Diverting.', 'team'),
                         ('Set current WP -> 9', 'team'), ('Resumed AUTO -> WP 9', 'team')):
            node.event(src, txt)
        node.set_image(pf.aerial_scene(1280, 720, seed=5, tent_xy=(640, 360), person_xy=(300, 200),
                                       scale=1.4))
        w = DemoWindow(node, app,
                       mission_planner=pf.FakeMissionPlanner(pf.planner_state('not on Windows')))
        w.win.resize(1600, 960)
        w.win.show()
        w.new_photos = 182
        _pump(app, w, until=lambda: w.shown_photo == name)
        _shoot(w, out_dir, 'standin', results)
        results.update(standin_banner=w.banner.isVisible(), standin_phase=w.phase_strip.current)
        w.close()
        w.win.close()

    # ---------------------------------------------------------------- 4. smallest window, MP opening
    with tempfile.TemporaryDirectory() as photos:
        node = pf.FakePanelNode(os.path.join(photos, 'mapping_photos'), world, items)
        w = DemoWindow(node, app, mission_planner=pf.FakeMissionPlanner(pf.planner_state('opening')))
        w.win.resize(1200, 760)            # = the window's minimum size
        w.win.show()
        _pump(app, w, timeout=0.8)
        _shoot(w, out_dir, 'small', results)
        results['small_size'] = (w.win.width(), w.win.height())
        w.close()
        w.win.close()
    return results


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(tempfile.gettempdir(), 'sim_demo_preview')
    res = render_all(out)
    for k, v in res.items():
        print(f'{k}: {v}')
