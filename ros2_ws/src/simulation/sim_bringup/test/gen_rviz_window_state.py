#!/usr/bin/env python3
"""
sim_bringup/test/gen_rviz_window_state.py -- PURPOSE: developer tool that writes the dock layout of rviz/sim.rviz.

RViz stores where its panes sit (left/right docks, sizes) as a Qt QMainWindow::saveState() blob in hex,
under "Window Geometry: QMainWindow State". RViz only writes it when somebody clicks File > Save Config in a
real window. This script builds the same dock widgets (same names as the panes: "Displays", "Camera POV
(live)", ...) in an invisible PyQt5 window, arranges them the way we want, and writes the hex blob into the
.rviz file. Qt only needs the names to match.

Layout made: Displays (with Views + Tool Properties as tabs) on the left, the 3D view in the middle, and on
the right a BIG "Camera POV (live)" pane above the "Newest photo (photo camera)" pane.

Usage (no screen needed):
    QT_QPA_PLATFORM=offscreen python3 test/gen_rviz_window_state.py --write rviz/sim.rviz
"""

import argparse
import os
import re
import sys

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtWidgets  # noqa: E402  (must come after QT_QPA_PLATFORM)

WIDTH, HEIGHT = 1700, 1000


def build_state() -> str:
    """Create the docks, lay them out and return QMainWindow.saveState() as a hex string."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    win = QtWidgets.QMainWindow()
    win.resize(WIDTH, HEIGHT)
    win.setCorner(QtCore.Qt.TopRightCorner, QtCore.Qt.RightDockWidgetArea)
    win.setCorner(QtCore.Qt.BottomRightCorner, QtCore.Qt.RightDockWidgetArea)
    win.setCorner(QtCore.Qt.BottomLeftCorner, QtCore.Qt.LeftDockWidgetArea)
    central = QtWidgets.QWidget()
    central.setMinimumSize(300, 300)
    win.setCentralWidget(central)

    def dock(name, area):
        d = QtWidgets.QDockWidget(name, win)
        d.setObjectName(name)
        inner = QtWidgets.QWidget()
        inner.setMinimumSize(100, 80)
        d.setWidget(inner)
        d.setAllowedAreas(QtCore.Qt.AllDockWidgetAreas)
        win.addDockWidget(area, d)
        return d

    displays = dock('Displays', QtCore.Qt.LeftDockWidgetArea)
    views = dock('Views', QtCore.Qt.LeftDockWidgetArea)
    tools = dock('Tool Properties', QtCore.Qt.LeftDockWidgetArea)
    win.tabifyDockWidget(views, tools)
    pov = dock('Camera POV (live)', QtCore.Qt.RightDockWidgetArea)
    photo = dock('Newest photo (photo camera)', QtCore.Qt.RightDockWidgetArea)
    win.splitDockWidget(pov, photo, QtCore.Qt.Vertical)
    win.show()
    app.processEvents()
    win.resizeDocks([displays, views], [700, 200], QtCore.Qt.Vertical)
    win.resizeDocks([displays, pov], [290, 780], QtCore.Qt.Horizontal)
    win.resizeDocks([pov, photo], [480, 440], QtCore.Qt.Vertical)
    app.processEvents()
    state = bytes(win.saveState()).hex()
    win.close()
    return state


def main() -> int:
    """Print the blob, or write it into the given .rviz file."""
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--write', default='', help='.rviz file to update in place')
    args = ap.parse_args()
    state = build_state()
    if not args.write:
        print(state)
        return 0
    with open(args.write, 'r', encoding='utf-8') as f:
        text = f.read()
    line = f'  QMainWindow State: {state}\n'
    if re.search(r'^  QMainWindow State: .*$', text, flags=re.M):
        text = re.sub(r'^  QMainWindow State: .*\n', line, text, flags=re.M)
    else:
        text = text.replace('  Hide Right Dock: false\n', '  Hide Right Dock: false\n' + line, 1)
    with open(args.write, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    print(f'QMainWindow State ({len(state)} hex chars) written into {args.write}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
