"""
sim_plugins/team_repo.py -- PURPOSE: check ANOTHER COPY of the team's Jetson code (a teammate's fork, another
branch, this year's code) before the simulator runs it, so a checkout that cannot work stops the demo at once
with a clear message instead of a five-minute wait for nodes that never come.

How the simulator runs the Jetson code (the same way the real Jetson Orin Nano does): demo.sh goes into the
checkout and starts ITS ./launcher.sh sim. launcher.sh sources /opt/ros/humble and that checkout's
ros2_ws/install, starts MAVROS (fcu_url from FCU_URL) and then TRACKER_CMD (the simulator's team_stack.launch.py,
which includes that checkout's own main/launch/tracker.launch.xml). So the checkout must keep "the two doors"
and the names the simulator watches (contracts.TEAM_STACK):

  * ./launcher.sh that honours the environment variables TRACKER_CMD and FCU_URL (the only simulator hook);
  * ros2_ws built (colcon build) with the team packages: main, video_cam, detection, mapping, waypoint_mavros,
    gps_ros2, wp_sender, interfaces;
  * main/launch/tracker.launch.xml (team_stack.launch.py includes it);
  * the node names stack_check waits for (waypoint_manager, gps_mavros_service_node, new_od, siyi,
    mission_camera_trigger, main_controller), looked for in the package sources (a warning when not found);
  * the status texts the phase monitor parses ("Camera trigger STARTED", "Captured", "Reached last WP",
    "Targets:", "Resumed AUTO", "Over ", "Servo ", "RTL"), looked for in the sources (warnings).

check(repo) -> (errors, warnings, notes); every error says what is wrong, why it matters and the exact fix.
Pure Python (no ROS): plan.check_environment() calls it for the profile key run.team_repo (demo.sh --team-repo).
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

#: The team packages ./launcher.sh sim needs in the checkout's ros2_ws/install (same list demo.sh checks).
TEAM_PACKAGES = ('main', 'video_cam', 'detection', 'mapping', 'waypoint_mavros', 'gps_ros2', 'wp_sender',
                 'interfaces')

#: package -> the ROS node name stack_check (sim_bringup/stack_check.py TEAM_NODES) waits for.
NODE_OF_PACKAGE: Dict[str, str] = {
    'waypoint_mavros': 'waypoint_manager', 'gps_ros2': 'gps_mavros_service_node', 'detection': 'new_od',
    'video_cam': 'siyi', 'mapping': 'mission_camera_trigger', 'main': 'main_controller',
}

#: (text, package, what the simulator does with it): STATUSTEXT the phase monitor / panel parse (AGENTS.md 7).
STATUS_TEXTS: Tuple[Tuple[str, str, str], ...] = (
    ('Camera trigger STARTED', 'mapping', 'phase SEARCH: the photo loop started'),
    ('Captured', 'video_cam', 'the photo counter ("SUCCESS: Captured 2K image #N")'),
    ('Reached last WP', 'main', 'phase HOLD (waiting for the detector)'),
    ('Targets:', 'main', 'phase TARGETS ("Targets: tent@N -> person@M. Diverting.")'),
    ('Resumed AUTO', 'main', 'the flight to the targets'),
    ('Over ', 'main', 'phase DROP ("Over tent: loiter 6s then drop")'),
    ('Servo ', 'main', 'the drop itself ("Servo 13 -> 1400")'),
    ('RTL', 'main', 'phase RTL (going home)'),
)


def _read(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='replace')
    except OSError:
        return ''


def _sources(src: Path, package: str) -> str:
    """All Python source of one package (backup / old / test files skipped), as one text."""
    folder = src / package
    if not folder.is_dir():
        return ''
    parts = []
    for f in sorted(folder.rglob('*.py')):
        rel = str(f.relative_to(folder))
        if any(bad in rel for bad in ('.bak', 'old_', 'test')):
            continue
        parts.append(_read(f))
    return '\n'.join(parts)


def check(repo: str, packages: Sequence[str] = TEAM_PACKAGES) -> Tuple[List[str], List[str], List[str]]:
    """Check one checkout of the team code. Returns (errors, warnings, notes)."""
    errors: List[str] = []
    warnings: List[str] = []
    notes: List[str] = []
    text = str(repo or '').strip()
    if not text:
        return errors, warnings, notes
    root = Path(text)
    if ' ' in text:
        errors.append(f'the team code folder "{text}" contains a space. Why: colcon and ./launcher.sh do not '
                      'work in folders with spaces. Fix: put the checkout in your Linux home, e.g. '
                      'git clone <url> ~/astra_fork')
    if not root.is_dir():
        errors.append(f'the team code folder {text} does not exist. Fix: clone it first, e.g. '
                      f'git clone <the fork URL> {text}   (then build it: cd {text}/ros2_ws && '
                      'colcon build --symlink-install)')
        return errors, warnings, notes
    launcher = root / 'launcher.sh'
    ltext = _read(launcher)
    if not launcher.is_file():
        errors.append(f'{text} has no launcher.sh. Why: the simulator starts the Jetson code exactly like the real '
                      'Jetson does, with ./launcher.sh sim. Fix: give the folder that holds launcher.sh and ros2_ws/ '
                      '(the repo root of the team code).')
    else:
        for var, why in (('TRACKER_CMD', 'without it the launcher starts tracker.launch.xml with the Jetson\'s model '
                                         'path, so new_od never loads a model on this PC'),
                         ('FCU_URL', 'without it MAVROS cannot be pointed at the simulated autopilot')):
            if var not in ltext:
                errors.append(f'{launcher} does not read the environment variable {var} (an older launcher). Why: '
                              f'{why}. Fix: copy the simulator-ready launcher into that checkout: '
                              f'cp ~/ODLC_Machine_Inferencing_System_2025-2026/launcher.sh {text}/launcher.sh')
    ws = root / 'ros2_ws'
    install = ws / 'install'
    if not (install / 'setup.bash').is_file():
        errors.append(f'{text}/ros2_ws is not built (no install/setup.bash). Why: ./launcher.sh sources it to find '
                      f'the team nodes. Fix: source /opt/ros/humble/setup.bash && cd {text}/ros2_ws && '
                      'colcon build --symlink-install --packages-ignore ultralytics_ros')
    else:
        missing = [p for p in packages if not (install / p).is_dir()]
        if missing:
            errors.append(f'{text}/ros2_ws/install has no {", ".join(missing)}. Why: ./launcher.sh sim needs every '
                          f'team package. Fix: cd {text}/ros2_ws && colcon build --symlink-install '
                          f'--packages-up-to {" ".join(missing)}')
    tracker = ws / 'src' / 'main' / 'launch' / 'tracker.launch.xml'
    if not tracker.is_file():
        errors.append(f'{tracker} is missing. Why: the simulator\'s team_stack.launch.py includes the checkout\'s '
                      'own tracker.launch.xml (the flight launch). Fix: keep main/launch/tracker.launch.xml in the fork.')
    src = ws / 'src'
    for package, node in NODE_OF_PACKAGE.items():
        code = _sources(src, package)
        if code and not re.search(r'[\'"]' + re.escape(node) + r'[\'"]', code):
            warnings.append(f'the node name "{node}" was not found in {package}\'s source. Consequence: stack_check '
                            f'waits for a node called {node} and demo.sh stops after 5 minutes. Fix: keep the '
                            'node name, or use a profile with a custom slot and node_name (docs/simulation/PLUGINS.md).')
    for needle, package, role in STATUS_TEXTS:
        code = _sources(src, package)
        if code and needle not in code:
            warnings.append(f'the status text "{needle.strip()}" was not found in {package}\'s source. Consequence: '
                            f'the simulator\'s phase monitor misses {role} (the flight itself still works).')
    if (root / '.git').exists():
        notes.append(f'team code from {text} (a git checkout: run_meta/team_stack.json records its commit)')
    else:
        notes.append(f'team code from {text}')
    return errors, warnings, notes
