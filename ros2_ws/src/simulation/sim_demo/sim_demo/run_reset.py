"""
sim_demo/run_reset.py -- PURPOSE: START OVER. Brings a running simulator back to the START of the search
mission so it can be flown again, WITHOUT closing any window (Gazebo, RViz and the demo panel stay open).

After one demo flight the payloads lie on the ground, the team's main_controller has finished its sequence
and old photos sit in video_cam's photo folder, so pressing UPLOAD MISSION + START again is NOT a clean
restart. This module does the clean restart, in this order (each step prints a numbered line):

   1. check     find the running simulator (state/current_launch.yaml + its session tag)
   2. handover  stop the previous run's watchers (demo.sh's phase monitor, an older run watcher); demo.sh
                then finishes politely and leaves everything open (state/keep_open tells it to)
   3. stop_team stop the team flight stack (./launcher.sh sim: MAVROS, video_cam, new_od, trigger,
                waypoint_manager, main_controller), so nothing fights the landing or writes photos
   4. land      if the drone is flying: RTL or LAND, wait until it has landed and disarmed (with a timeout)
   5. archive   move the old photos + detector results into the OLD run folder; the team photo folder is
                empty again (old photos would be detected again and could send the drone to old targets)
   6. remove    remove the drone entity "iris" from Gazebo WHILE SITL STILL RUNS (see "Why this order")
   7. stop_ap   stop ArduCopter SITL and MAVProxy (their exact command lines were recorded first)
   8. spawn     spawn a fresh drone from the SAME generated model file at the world's spawn pose: both
                payloads hang under it again, attached by their DetachableJoints, the gimbal is centred
   9. start_ap  start SITL and MAVProxy again with the SAME command lines (same ports, same home), so the
                autopilot boots on the spawn point and its EKF agrees with the physics
  10. helpers   a NEW run folder; restart the per-run helper nodes with fresh memories: score_run (score
                report of the new run), payload_release, demo_markers, truth_detector (if used); the Gazebo
                window follows the new drone
  11. ready     wait for the autopilot to settle (GPS + EKF + pre-arm, the normal ~30-60 s)
  12. upload    upload the world's survey mission (like Mission Planner's "Write WPs")
  13. team      start the team flight stack exactly like demo.sh does, and wait until it is ready
  14. watch     start the run watcher (phase monitor -> photos + score into the new run folder)
  15. fly       (only with --fly) arm and start the mission (AUTO)

Why this order (measured in Gazebo Harmonic 8.15 + ardupilot_gazebo, see simulation/README.md):
  * ardupilot_gazebo's plugin runs in LOCK-STEP with SITL: once SITL has talked to it, Gazebo's whole update
    loop WAITS for SITL's next packet. If SITL is stopped first, Gazebo freezes and cannot even remove the
    drone. So the drone is removed while SITL still runs (the plugin is destroyed with the drone and frees
    UDP 9002), and SITL is started again only after the new drone listens on UDP 9002.
  * a brand-new drone entity is the only way that restores EVERYTHING at once (both payloads in their
    slots, latches, gimbal, sensors) without fighting the physics engine; a new SITL boots on the spawn
    point, so GPS, EKF and Gazebo agree (no "teleport" for the EKF to digest).

Used by: simulation/scripts/start_over.sh (terminal, Windows "Start Over.bat") and the demo panel's START
OVER button. Progress goes to stdout AND, as JSON lines, to <sim home>/state/start_over_progress.jsonl,
which the demo panel follows (ProgressTracker below) to show every step in its Events list and status line,
to keep its mission buttons locked while a start over runs, and to clear the old run from its display.
Settings of the running simulator win over the terminal's: its GZ_PARTITION and ROS_DOMAIN_ID (recorded in
state/current_launch.yaml) are used for every gz / ros2 command and for the restarted team stack.

Command line (start_over.sh calls these; all output is plain words):
  python3 -m sim_demo.run_reset reset [--fly] [--world W] [--mission FILE] [--land rtl|land]
                                       [--land-timeout S] [--no-team-stack]
  python3 -m sim_demo.run_reset watch-run --run-dir DIR [--photo-dir DIR] [--timeout-min M]   (internal)
  python3 -m sim_demo.run_reset guard --session ID                                            (internal)
  python3 -m sim_demo.run_reset mp-steps [--world W] [--mission FILE]     print the Mission Planner steps
  python3 -m sim_demo.run_reset plan                                       print the steps, change nothing
Exit codes: 0 = done, 1 = a step failed (the message says which and what to do), 2 = bad usage,
3 = no simulator is running (start_over.sh then starts a fresh demo instead, unless --no-fresh),
130 = stopped with Ctrl+C (run it again to finish).

Everything that touches the outside world (processes, Gazebo, MAVLink, the clock) goes through small
adapter classes, so test/test_run_reset.py checks the whole state machine with fakes, without ROS or Gazebo.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence

# ---------------------------------------------------------------------------------------------
# Fixed names (the simulation contract, SIM_CONTRACT.md) and default timeouts
# ---------------------------------------------------------------------------------------------

DRONE_ENTITY = 'iris'                 #: the drone's entity name in Gazebo
PAYLOADS = ('person', 'tent')         #: payload names; gz topics /sim/payload/<name>/...

#: (step id, title) in the order they run. The panel and start_over.sh show "STEP n/N <title>".
STEPS = [
    ('check', 'Check the running simulator'),
    ('handover', 'Take over from the previous run'),
    ('stop_team', 'Stop the team flight stack'),
    ('land', 'Bring the drone down (if it is flying)'),
    ('archive', 'Archive the old photos and detections'),
    ('remove', 'Remove the old drone from Gazebo'),
    ('stop_ap', 'Stop the autopilot (SITL + MAVProxy)'),
    ('spawn', 'Spawn a fresh drone with both payloads on the start point'),
    ('start_ap', 'Start the autopilot again (SITL + MAVProxy)'),
    ('helpers', 'New run folder + restart the run recorder, payload release and markers'),
    ('ready', 'Wait for the autopilot to settle (GPS + EKF, about 30-60 s)'),
    ('upload', 'Upload the survey mission'),
    ('team', 'Start the team flight stack (./launcher.sh sim)'),
    ('watch', 'Start watching the new run'),
    ('fly', 'Arm and start the mission (AUTO)'),
]
STEP_TITLES = dict(STEPS)


@dataclass
class Timeouts:
    """How long each wait may take [s] before start over gives a clear message."""
    land: float = 300.0          # RTL from the far end of the field + landing; LAND alone needs ~60 s
    demo_exit: float = 40.0      # demo.sh writes the old score report and copies photos, then exits
    stop: float = 20.0           # a process group gets this long after Ctrl+C before TERM / KILL
    remove: float = 30.0         # Gazebo removes the drone and frees UDP 9002
    spawn: float = 90.0          # the new drone loads (meshes, cameras) and its plugin opens UDP 9002
    sitl_port: float = 60.0      # SITL listens on TCP 5760+10*I
    ready: float = 360.0         # GPS fix + EKF + pre-arm checks (the normal ~30-60 s, more on a busy PC)
    stack: float = 300.0         # the team stack is ready (MAVROS connected, mission read, model loaded)


# ---------------------------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------------------------

class ResetError(RuntimeError):
    """A step failed. The message is a plain sentence for the user (what happened + what to do)."""


class NotRunning(ResetError):
    """No simulator is running (start_over.sh then starts a fresh demo instead)."""


def sim_home() -> Path:
    """The simulator's state folder: $ASTRA_SIM_HOME, else ~/.astra_sim (same rule as sim_bringup)."""
    return Path(os.path.expanduser(os.environ.get('ASTRA_SIM_HOME', '') or '~/.astra_sim'))


def yaw_to_quaternion(yaw: float):
    """(x, y, z, w) of a rotation by `yaw` radians about the vertical axis."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def new_run_dir(home: Path, world: str, stamp: Optional[str] = None) -> Path:
    """The folder of a new run: <home>/runs/<YYYYmmdd-HHMMSS>_<world> (the same pattern demo.sh uses)."""
    return home / 'runs' / f'{stamp or time.strftime("%Y%m%d-%H%M%S")}_{world}'


# ---------------------------------------------------------------------------------------------
# What the running simulator uses (written by sim_bringup sim.launch.py at start)
# ---------------------------------------------------------------------------------------------

@dataclass
class SimState:
    """The parts of state/current_launch.yaml that start over needs."""
    session: str = ''
    world: str = 'runway'
    world_name: str = 'runway'       # the <world name=...> inside the .sdf (Gazebo service names use it)
    model_file: str = ''             # the generated drone model (same file is spawned again)
    spawn: Dict[str, float] = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.25, 'yaw': 0.0})
    mission: str = ''
    detector: str = 'real'
    sitl_instance: int = 0
    ports: Dict[str, int] = field(default_factory=dict)
    gcs_port: int = 14550
    mavros_port: int = 14552
    panel_port: int = 14553
    windows_gcs: str = ''
    sitl_run_dir: str = ''
    photo_dir: str = ''
    home: str = ''
    gz_partition: str = ''           # GZ_PARTITION the simulator runs in (gz commands must use the same)
    ros_domain_id: str = ''          # ROS_DOMAIN_ID of the simulator (the team stack must use the same)

    @property
    def fdm_port(self) -> int:
        """UDP port of the Gazebo <-> SITL link (9002 + 10*I)."""
        return int(self.ports.get('fdm', 9002 + 10 * self.sitl_instance))

    @property
    def serial0_port(self) -> int:
        """TCP port SITL's first MAVLink serial listens on (5760 + 10*I)."""
        return int(self.ports.get('serial0', 5760 + 10 * self.sitl_instance))

    @property
    def gcs_url(self) -> str:
        """pymavlink URL of MAVProxy's ground-station output inside Linux (our scripts use it)."""
        return f'udpin:127.0.0.1:{self.gcs_port}'


def load_sim_state(home: Path) -> SimState:
    """Read <home>/state/current_launch.yaml. Raises NotRunning when there is none."""
    path = home / 'state' / 'current_launch.yaml'
    try:
        import yaml
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except FileNotFoundError:
        raise NotRunning(f'No simulator has been started yet ({path} is missing).') from None
    except Exception as exc:  # noqa: BLE001 - a broken file becomes one clear sentence
        raise ResetError(f'Cannot read {path}: {exc}') from None
    st = SimState()
    for key in ('session', 'world', 'world_name', 'model_file', 'mission', 'detector', 'windows_gcs',
                'sitl_run_dir', 'photo_dir', 'home', 'gz_partition', 'ros_domain_id'):
        if data.get(key) not in (None, ''):
            setattr(st, key, str(data[key]))
    for key in ('sitl_instance', 'gcs_port', 'mavros_port', 'panel_port'):
        if data.get(key) not in (None, ''):
            setattr(st, key, int(data[key]))
    if isinstance(data.get('spawn'), dict):
        st.spawn = {k: float(data['spawn'].get(k, st.spawn.get(k, 0.0))) for k in ('x', 'y', 'z', 'yaw')}
    if isinstance(data.get('ports'), dict):
        st.ports = {k: int(v) for k, v in data['ports'].items()}
    # The session recorded by start_sim.sh / demo.sh wins (a launch started by hand records its own).
    recorded = (home / 'state' / 'session')
    if recorded.is_file():
        text = recorded.read_text(encoding='utf-8').strip()
        if text:
            st.session = text
    return st


# ---------------------------------------------------------------------------------------------
# Processes: who is who (pure functions on a process table, unit-tested)
# ---------------------------------------------------------------------------------------------

@dataclass
class Proc:
    """One running process: pid, process group, parent, command line words, working folder, environment."""
    pid: int
    pgid: int
    ppid: int
    argv: List[str]
    cwd: str = ''
    env: Dict[str, str] = field(default_factory=dict)

    @property
    def cmd(self) -> str:
        """The command line as one string (for matching and messages)."""
        return ' '.join(self.argv)

    @property
    def session(self) -> str:
        """The simulator session tag this process carries ('' = none)."""
        return self.env.get('ASTRA_SIM_SESSION', '')


#: The team's flight stack, by command line (launcher.sh and the nodes of main/launch/tracker.launch.xml,
#: wrapped by sim_bringup team_stack.launch.py). Matching is by path fragments, so a sim helper never matches.
TEAM_PATTERNS = ('launcher.sh', 'team_stack.launch.py', 'tracker.launch.xml', 'mavros_node', 'launch mavros',
                 '/lib/detection/new_od', '/lib/video_cam/siyi', '/lib/mapping/trigger',
                 '/lib/waypoint_mavros/waypoint', '/lib/gps_ros2/gpstest', 'main_controller.py', 'humanize.sh')

#: Roles that start over must NEVER stop (the windows and the world stay open).
PROTECTED_ROLES = ('gz_server', 'gz_gui', 'panel', 'rviz', 'sim_launch', 'bridge', 'emulator', 'odom_tf')


def classify(p: Proc) -> str:
    """The role of a process, from its command line (see the module docstring for the list)."""
    cmd = p.cmd
    words = [os.path.basename(a) for a in p.argv]
    if 'sim_demo.run_reset' in cmd:
        for sub in ('watch-run', 'guard', 'reset'):
            if sub in p.argv:
                return {'watch-run': 'watcher', 'guard': 'guard', 'reset': 'start_over'}[sub]
    if any(w == 'start_over.sh' for w in words):
        return 'start_over'
    if any(w == 'demo.sh' for w in words):
        return 'demo'
    if re.search(r'(^|/)gz sim(\s|$)', cmd):
        # `gz sim -s ...` = the server (the world), `gz sim -g ...` = the window (the gz tool is a ruby
        # script, so the process is "ruby /usr/bin/gz sim ...")
        return 'gz_gui' if '-g' in p.argv else 'gz_server'
    if 'arducopter' in words or ('wait_then_exec' in cmd and '--label sitl' in cmd):
        return 'sitl'
    if 'mavproxy.py' in words or ('wait_then_exec' in cmd and '--label mavproxy' in cmd):
        return 'mavproxy'
    if any(t in cmd for t in TEAM_PATTERNS):
        return 'team'
    if 'sim.launch.py' in cmd and 'launch' in cmd:
        return 'sim_launch'
    for frag, role in (('/lib/sim_demo/demo_panel', 'panel'), ('/lib/sim_demo/score_run', 'score_run'),
                       ('/lib/sim_demo/demo_markers', 'markers'), ('/lib/sim_demo/truth_detector', 'truth_detector'),
                       ('/lib/sim_models/payload_release', 'payload_release'),
                       ('/lib/sim_bringup/phase_monitor', 'phase_monitor'),
                       ('/lib/sim_bringup/mission_tool', 'mission_tool'),
                       ('/lib/sim_bringup/stack_check', 'stack_check'),
                       ('/lib/sim_bringup/gz_follow', 'gz_follow'), ('/lib/sim_bringup/odom_tf', 'odom_tf'),
                       ('/lib/siyi_emulator/siyi_emulator', 'emulator'), ('parameter_bridge', 'bridge'),
                       ('rviz2', 'rviz')):
        if frag in cmd:
            return role
    return 'other'


def select(procs: Iterable[Proc], session: str, roles: Sequence[str], domain: str = '',
           exclude: Iterable[int] = ()) -> List[Proc]:
    """The processes of `session` whose role is in `roles`.

    Only the simulator's session tag establishes ownership. A team stack started by hand in another
    terminal has no such tag and must be stopped by its operator, even if it uses the same ROS domain.
    `exclude` = pids that must never be touched (start over itself and its parents). `domain` is retained
    for callers of older versions; it never grants permission to stop an untagged process.
    """
    skip = set(exclude)
    out = []
    for p in procs:
        if p.pid in skip:
            continue
        role = classify(p)
        if role not in roles or role in PROTECTED_ROLES:
            continue
        if p.session == session and session:
            out.append(p)
    return out


def demo_scripts(procs: Sequence[Proc], session: str, exclude: Iterable[int] = ()) -> List[Proc]:
    """The demo.sh process(es) of `session`.

    demo.sh exports ASTRA_SIM_SESSION only AFTER it has started, so its own /proc environment does not
    show the tag; its CHILDREN (the launch, phase_monitor, mission_tool ...) do. A demo.sh counts when it
    carries the tag itself or has at least one tagged child.
    """
    skip = set(exclude)
    tagged_parents = {p.ppid for p in procs if p.session == session}
    return [p for p in procs if p.pid not in skip and classify(p) == 'demo'
            and (p.session == session or p.pid in tagged_parents)]


def safe_groups(targets: Sequence[Proc], everyone: Sequence[Proc], exclude: Iterable[int] = ()) -> List[int]:
    """Process groups that may be stopped as a whole (Ctrl+C to the group, like demo.sh's stop_group).

    A group is only used when EVERY member is one of the targets, e.g. the team stack, which demo.sh starts
    with `setsid` (its own group: launcher.sh, the two launches and their nodes). A group with anything
    else in it is never signalled as a whole: demo.sh's phase monitor shares demo.sh's group, and a Ctrl+C
    to that group would make demo.sh close the whole simulator. The caller then signals pid by pid.
    """
    skip = set(exclude)
    wanted = {p.pid for p in targets}
    groups = []
    for g in sorted({p.pgid for p in targets if p.pgid > 1}):
        members = [p for p in everyone if p.pgid == g]
        if any(p.pid in skip or p.pid not in wanted or classify(p) in PROTECTED_ROLES for p in members):
            continue
        groups.append(g)
    return groups


class ProcessOps:
    """The real process table and signals (Linux /proc). Tests use a fake with the same methods."""

    def table(self) -> List[Proc]:
        """Every process of this user (pid, group, parent, argv, cwd, environment)."""
        uid = os.getuid()
        out = []
        for d in Path('/proc').iterdir():
            if not d.name.isdigit():
                continue
            try:
                if d.stat().st_uid != uid:
                    continue
                argv = [a for a in (d / 'cmdline').read_bytes().decode(errors='replace').split('\0') if a]
                if not argv:
                    continue
                stat = (d / 'stat').read_text()
                rest = stat[stat.rindex(')') + 2:].split()      # fields after "(comm)"
                ppid, pgid = int(rest[1]), int(rest[2])
                env = {}
                for item in (d / 'environ').read_bytes().decode(errors='replace').split('\0'):
                    k, sep, v = item.partition('=')
                    if sep:
                        env[k] = v
                try:
                    cwd = os.readlink(d / 'cwd')
                except OSError:
                    cwd = ''
                out.append(Proc(int(d.name), pgid, ppid, argv, cwd, env))
            except (OSError, ValueError, IndexError):
                continue                                         # the process ended meanwhile
        return out

    def alive(self, pid: int) -> bool:
        """True while the process exists (and is not a zombie)."""
        try:
            state = Path(f'/proc/{pid}/stat').read_text()
            return state[state.rindex(')') + 2] != 'Z'
        except (OSError, ValueError, IndexError):
            return False

    def signal_pid(self, pid: int, sig: int) -> None:
        """Send `sig` to one process (ignores processes that are already gone)."""
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def signal_group(self, pgid: int, sig: int) -> None:
        """Send `sig` to a whole process group."""
        try:
            os.killpg(pgid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def spawn(self, argv: Sequence[str], cwd: str, env: Dict[str, str], log_path: str) -> int:
        """Start a program in the background in its OWN process group (it outlives start over); output
        appended to `log_path`. Returns its pid."""
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        with open(log_path, 'ab') as log:
            proc = subprocess.Popen(list(argv), cwd=cwd or None, env=dict(env), stdout=log,
                                    stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        return proc.pid


def ancestors(pid: Optional[int] = None) -> List[int]:
    """`pid` (default: this process) and all its parents: start over never stops itself or its caller."""
    out, p = [], pid or os.getpid()
    while p and p > 1:
        out.append(p)
        try:
            stat = Path(f'/proc/{p}/stat').read_text()
            p = int(stat[stat.rindex(')') + 2:].split()[1])
        except (OSError, ValueError, IndexError):
            break
    return out


def stop_procs(ops, targets: Sequence[Proc], everyone: Sequence[Proc], clock, grace: float,
               exclude: Iterable[int] = ()) -> List[Proc]:
    """Stop `targets` politely: Ctrl+C (to safe process groups, else to each pid), wait up to `grace`
    seconds, then TERM, then KILL. Returns the processes that are STILL alive (normally none)."""
    if not targets:
        return []
    groups = safe_groups(targets, everyone, exclude)
    for g in groups:
        ops.signal_group(g, signal.SIGINT)
    for p in targets:
        if p.pgid not in groups:
            ops.signal_pid(p.pid, signal.SIGINT)
    for sig, wait in ((None, grace), (signal.SIGTERM, 5.0), (signal.SIGKILL, 3.0)):
        if sig is not None:
            for p in targets:
                if ops.alive(p.pid):
                    ops.signal_pid(p.pid, sig)
        end = clock.time() + wait
        while clock.time() < end and any(ops.alive(p.pid) for p in targets):
            clock.sleep(0.5)
        if not any(ops.alive(p.pid) for p in targets):
            return []
    return [p for p in targets if ops.alive(p.pid)]


# ---------------------------------------------------------------------------------------------
# Restarting a process with the SAME command line (recorded from /proc before it is stopped)
# ---------------------------------------------------------------------------------------------

@dataclass
class Spec:
    """How to start a program again: argv, working folder, environment."""
    argv: List[str]
    cwd: str = ''
    env: Dict[str, str] = field(default_factory=dict)

    def to_json(self) -> dict:
        """JSON form for state/start_over_specs.json (the environment is not saved: it is re-read)."""
        return {'argv': self.argv, 'cwd': self.cwd}


def spec_from_proc(p: Proc) -> Spec:
    """The Spec that starts `p` again. A program still waiting in sim_bringup's wait_then_exec is unwrapped
    (start over waits for the ports itself), so the Spec is always the real program."""
    argv = list(p.argv)
    if any('wait_then_exec' in a for a in argv[:3]) and '--' in argv:
        argv = argv[argv.index('--') + 1:]
    return Spec(argv, p.cwd, dict(p.env))


def with_ros_param(argv: Sequence[str], name: str, value: str) -> List[str]:
    """argv of a ROS 2 node with `-p name:=value` added (a later -p wins over the launch's params file).
    An earlier `-p name:=...` given on the command line is replaced."""
    out = list(argv)
    i = 0
    while i < len(out) - 1:
        if out[i] == '-p' and out[i + 1].startswith(f'{name}:='):
            del out[i:i + 2]
            continue
        i += 1
    if '--ros-args' not in out:
        out.append('--ros-args')
    return out + ['-p', f'{name}:={value}']


def drop_missing_params_files(argv: Sequence[str], fallback: Dict[str, str]) -> List[str]:
    """argv of a ROS 2 node whose `--params-file` (a temporary file the launch wrote into /tmp) may be gone
    by now: a missing file is removed from argv and `fallback` parameters are given with -p instead."""
    out, i, dropped = [], 0, False
    while i < len(argv):
        if argv[i] == '--params-file' and i + 1 < len(argv) and not Path(argv[i + 1]).is_file():
            dropped, i = True, i + 2
            continue
        out.append(argv[i])
        i += 1
    if dropped:
        for name, value in fallback.items():
            if value:
                out = with_ros_param(out, name, value)
    return out


def ros_param_from_argv(argv: Sequence[str], name: str) -> str:
    """The value of ROS parameter `name` of a node process: from `-p name:=v` or its --params-file ('' if none)."""
    for i, a in enumerate(argv[:-1]):
        if a == '-p' and argv[i + 1].startswith(f'{name}:='):
            return argv[i + 1].split(':=', 1)[1]
    for i, a in enumerate(argv[:-1]):
        if a == '--params-file':
            try:
                import yaml
                data = yaml.safe_load(Path(argv[i + 1]).read_text(encoding='utf-8')) or {}
            except Exception:  # noqa: BLE001 - the launch's temporary file may be gone
                continue
            for node_params in data.values():
                params = (node_params or {}).get('ros__parameters', {}) if isinstance(node_params, dict) else {}
                if params.get(name) not in (None, ''):
                    return str(params[name])
    return ''


# ---------------------------------------------------------------------------------------------
# Gazebo (through the `gz` command line tool, like sim_bringup gz_follow) and sockets
# ---------------------------------------------------------------------------------------------

def _bound_ports(kind: str) -> set:
    """Local ports that are bound (UDP) or listening (TCP), read from /proc/net (no test connection)."""
    ports = set()
    for path in (f'/proc/net/{kind}', f'/proc/net/{kind}6'):
        try:
            lines = Path(path).read_text().splitlines()[1:]
        except OSError:
            continue
        for line in lines:
            cols = line.split()
            if len(cols) < 4:
                continue
            if kind == 'tcp' and cols[3] != '0A':      # 0A = LISTEN
                continue
            try:
                ports.add(int(cols[1].rsplit(':', 1)[1], 16))
            except ValueError:
                continue
    return ports


class GzOps:
    """Gazebo world services through `gz service` (inherits GZ_PARTITION / GZ_IP from the environment)."""

    def __init__(self, world_name: str):
        self.world = world_name

    def _gz(self, args: Sequence[str], timeout: float = 20.0) -> Optional[str]:
        try:
            res = subprocess.run(['gz', *args], capture_output=True, text=True, timeout=timeout, check=False)
        except (OSError, subprocess.TimeoutExpired):
            return None
        return res.stdout if res.returncode == 0 else None

    def _service(self, name: str, reqtype: str, req: str, timeout_ms: int = 8000) -> bool:
        out = self._gz(['service', '-s', f'/world/{self.world}/{name}', '--reqtype', reqtype,
                        '--reptype', 'gz.msgs.Boolean', '--timeout', str(timeout_ms), '--req', req],
                       timeout=timeout_ms / 1000.0 + 10.0)
        return bool(out) and 'data: true' in out

    def model_exists(self, name: str) -> bool:
        """True when `gz model --list` shows the model."""
        out = self._gz(['model', '--list'], timeout=25.0)
        if out is None:
            raise ResetError('Cannot check whether the drone exists: Gazebo did not answer the model list. '
                             'The drone is left in place; check the simulator before trying START OVER again.')
        return re.search(rf'^\s*-\s*{re.escape(name)}\s*$', out, re.M) is not None

    def remove_model(self, name: str) -> bool:
        """Ask Gazebo to remove the model `name` (its plugins, sensors and nested payloads go with it)."""
        return self._service('remove', 'gz.msgs.Entity', f'name: "{name}", type: MODEL')

    def spawn_model(self, sdf_file: str, name: str, x: float, y: float, z: float, yaw: float) -> bool:
        """Spawn `sdf_file` as entity `name` at (x, y, z, yaw) (what ros_gz_sim create does at start)."""
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        req = (f'sdf_filename: "{sdf_file}", name: "{name}", allow_renaming: false, '
               f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}')
        return self._service('create', 'gz.msgs.EntityFactory', req, timeout_ms=15000)

    def gui_running(self) -> bool:
        """True when the Gazebo WINDOW offers its follow service (there is something to re-follow)."""
        out = self._gz(['service', '--list']) or ''
        return any(line.strip() == '/gui/follow' for line in out.splitlines())

    def position(self, topic: str, timeout: float = 8.0) -> Optional[tuple]:
        """(x, y, z) from ONE gz.msgs.Odometry message on `topic`, or None (used to check the payloads)."""
        out = self._gz(['topic', '-e', '-n', '1', '-t', topic], timeout=timeout)
        if not out:
            return None
        block = out.split('pose', 1)[-1]
        vals = {}
        for axis in ('x', 'y', 'z'):
            m = re.search(rf'\b{axis}:\s*([-0-9.eE+]+)', block)
            vals[axis] = float(m.group(1)) if m else 0.0
        return vals['x'], vals['y'], vals['z']

    @staticmethod
    def udp_bound(port: int) -> bool:
        """True while some program has UDP `port` open (the drone's plugin opens 9002+10*I)."""
        return port in _bound_ports('udp')

    @staticmethod
    def tcp_listening(port: int) -> bool:
        """True while some program listens on TCP `port` (SITL: 5760+10*I)."""
        return port in _bound_ports('tcp')


# ---------------------------------------------------------------------------------------------
# MAVLink: is the drone flying? land it, wait for landed + disarmed
# ---------------------------------------------------------------------------------------------

@dataclass
class FlightSnapshot:
    """What the autopilot says right now."""
    armed: bool = False
    mode: str = '?'
    rel_alt: Optional[float] = None      # metres above home
    landed_state: int = 0                # MAV_LANDED_STATE: 1 on ground, 2 in air, 3 taking off, 4 landing

    @property
    def on_ground(self) -> bool:
        """True when the autopilot says it is on the ground (or it is less than 1.5 m above home)."""
        if self.landed_state == 1:
            return True
        return self.rel_alt is not None and self.rel_alt < 1.5 and self.landed_state not in (2, 3, 4)


class MavlinkOps:
    """A tiny pymavlink client on MAVProxy's ground-station output (opened only for the landing step)."""

    def __init__(self, url: str, timeout: float = 20.0):
        from pymavlink import mavutil    # imported here: the module works (and is tested) without pymavlink
        self.mavlink = mavutil.mavlink
        self.m = mavutil.mavlink_connection(url, source_system=246, source_component=190, autoreconnect=True)
        self.last: Dict[str, object] = {}
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg is not None and msg.autopilot == self.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.last['HEARTBEAT'] = msg
                self.target = (msg.get_srcSystem(), msg.get_srcComponent())
                return
        raise ResetError(f'No autopilot heartbeat on {url} within {timeout:.0f} s. Is MAVProxy running, and is '
                         'no other program listening on that port?')

    def snapshot(self, listen_s: float = 1.5) -> FlightSnapshot:
        """Listen for `listen_s` seconds and return armed / mode / altitude / landed state."""
        end = time.monotonic() + listen_s
        while time.monotonic() < end:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is not None and msg.get_type() != 'BAD_DATA' and msg.get_srcSystem() == self.target[0]:
                self.last[msg.get_type()] = msg
        hb, gp, ext = self.last.get('HEARTBEAT'), self.last.get('GLOBAL_POSITION_INT'), self.last.get('EXTENDED_SYS_STATE')
        names = {v: k for k, v in (self.m.mode_mapping() or {}).items()}
        return FlightSnapshot(
            armed=bool(hb and hb.base_mode & self.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
            mode=names.get(hb.custom_mode, str(hb.custom_mode)) if hb else '?',
            rel_alt=gp.relative_alt / 1000.0 if gp else None,
            landed_state=int(ext.landed_state) if ext else 0)

    def set_mode(self, name: str) -> bool:
        """Switch flight mode (RTL / LAND) and confirm it from the heartbeat (up to 8 s)."""
        mapping = self.m.mode_mapping() or {}
        if name not in mapping:
            return False
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            self.m.set_mode(mapping[name])
            if self.snapshot(1.0).mode == name:
                return True
        return False

    def disarm(self, force: bool = False) -> None:
        """Disarm (force only on the ground: 21196 is ArduPilot's 'force' magic number)."""
        self.m.mav.command_long_send(self.target[0], self.target[1],
                                     self.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196 if force else 0,
                                     0, 0, 0, 0, 0)

    def close(self) -> None:
        """Free the UDP port again (the mission tool needs it next)."""
        try:
            self.m.close()
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------------------------
# Running the simulator's own command line tools (mission_tool, stack_check, ...)
# ---------------------------------------------------------------------------------------------

class CommandOps:
    """Runs a command, prints its lines as they come, and hands each line to `on_line` (progress details)."""

    def run(self, argv: Sequence[str], timeout: float, on_line: Callable[[str], None] = print,
            env: Optional[Dict[str, str]] = None) -> int:
        """Run `argv` to the end (or kill it after `timeout` s). Returns the exit code (124 = timed out)."""
        try:
            proc = subprocess.Popen(list(argv), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                    stdin=subprocess.DEVNULL, env=env, bufsize=1)
        except OSError as exc:
            on_line(f'cannot start {argv[0]}: {exc}')
            return 127
        end = time.monotonic() + timeout
        import selectors
        sel = selectors.DefaultSelector()
        sel.register(proc.stdout, selectors.EVENT_READ)
        while True:
            if time.monotonic() > end:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(10)
                except subprocess.TimeoutExpired:
                    proc.kill()
                return 124
            if sel.select(timeout=0.5):
                line = proc.stdout.readline()
                if line:
                    on_line(line.rstrip())
                    continue
            if proc.poll() is not None:
                for line in proc.stdout.read().splitlines():
                    on_line(line.rstrip())
                return proc.returncode


class Clock:
    """Real time. Tests use a fake clock that jumps instead of sleeping."""

    @staticmethod
    def time() -> float:
        """Seconds (monotonic)."""
        return time.monotonic()

    @staticmethod
    def sleep(seconds: float) -> None:
        """Wait."""
        time.sleep(seconds)


def wait_until(clock, seconds: float, cond: Callable[[], bool], every: float = 0.5) -> bool:
    """Check `cond` every `every` s until it is True (returns True) or `seconds` have passed (False)."""
    end = clock.time() + seconds
    while True:
        if cond():
            return True
        if clock.time() >= end:
            return False
        clock.sleep(every)


# ---------------------------------------------------------------------------------------------
# Progress: stdout for the terminal, JSON lines for the demo panel
# ---------------------------------------------------------------------------------------------

def progress_path(home: Path) -> Path:
    """The file the demo panel reads to show start over's progress."""
    return home / 'state' / 'start_over_progress.jsonl'


class Progress:
    """Prints `[start over] ...` lines and appends the same as JSON lines to the progress file.

    Each line: {"t": unix time, "id": reset id, "kind": step|info|ok|warn|error|detail|done|failed,
    "step": step id, "n": step number, "total": number of steps, "text": sentence, "run_dir": new run}.
    """

    def __init__(self, path: Optional[Path], reset_id: str, total: int, echo: Callable[[str], None] = print):
        self.path, self.id, self.total, self.echo = path, reset_id, total, echo
        self.step_id, self.n = '', 0
        self.run_dir = ''
        self.last_detail = 0.0

    def start(self) -> None:
        """Begin a new progress file (the panel sees a new reset id and clears its run display).

        The old file is REPLACED (a new file under the same name), not emptied: a reader that checks the
        file's identity (ProgressTracker) then always notices the new start, even when the new file has
        already grown past the old one's size before it looks again."""
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self.path.with_name(self.path.name + '.new')
            tmp.write_text('', encoding='utf-8')
            os.replace(tmp, self.path)

    def _write(self, kind: str, text: str) -> None:
        if not self.path:
            return
        try:
            with open(self.path, 'a', encoding='utf-8') as f:
                f.write(json.dumps({'t': round(time.time(), 2), 'id': self.id, 'kind': kind, 'step': self.step_id,
                                    'n': self.n, 'total': self.total, 'text': text, 'run_dir': self.run_dir}) + '\n')
        except OSError:
            pass

    def step(self, step_id: str, n: int) -> None:
        """A new step starts."""
        self.step_id, self.n = step_id, n
        text = STEP_TITLES.get(step_id, step_id)
        self.echo(f'[start over] ===== STEP {n}/{self.total}: {text}')
        self._write('step', text)

    def say(self, kind: str, text: str) -> None:
        """info / ok / warn / error line."""
        mark = {'ok': ' OK ', 'warn': ' !! ', 'error': ' XX '}.get(kind, '    ')
        self.echo(f'[start over] {mark}{text}')
        self._write(kind, text)

    def detail(self, text: str) -> None:
        """A line from a tool (mission_tool, stack_check): printed; at most one per second goes to the file."""
        self.echo(f'               {text}')
        now = time.time()
        if now - self.last_detail >= 1.0 and text.strip():
            self.last_detail = now
            self._write('detail', text.strip())

    def finish(self, ok: bool, text: str) -> None:
        """The last line: done (ok) or failed."""
        self.echo(f'[start over] {"DONE" if ok else "FAILED"}: {text}')
        self._write('done' if ok else 'failed', text)


def read_progress(path: Path, offset: int = 0):
    """Read new progress lines from byte `offset`. Returns (entries, new_offset). A file that got SHORTER
    than `offset` (a new start over began) is read from the start again. Used by the demo panel."""
    try:
        size = path.stat().st_size
    except OSError:
        return [], 0
    if size < offset:
        offset = 0
    if size == offset:
        return [], offset
    entries = []
    with open(path, 'rb') as f:
        f.seek(offset)
        data = f.read()
    end = data.rfind(b'\n') + 1                     # keep a half-written last line for the next call
    for raw in data[:end].splitlines():
        try:
            entries.append(json.loads(raw.decode('utf-8')))
        except ValueError:
            continue
    return entries, offset + end


@dataclass
class ProgressUpdate:
    """One thing the demo panel shows, made from one progress line.

    text      the sentence (Events list and status line)
    level     '' | 'ok' | 'warn' | 'error' (the colour)
    step      the step id the line belongs to ('' for a run watcher's line)
    kind      the progress line's kind: step | info | ok | warn | error | detail | done | failed
    new_reset True on the first line of a NEW start over: the panel clears the old run from its display
    finished  True = the start over is done, False = it failed, None = still running / not a start over line
    detail    True = a tool's progress line: status line only (it would flood the Events list)
    """
    text: str
    level: str = ''
    step: str = ''
    kind: str = ''
    new_reset: bool = False
    finished: Optional[bool] = None
    detail: bool = False


class ProgressTracker:
    """Follows the progress file (progress_path) for the demo panel: which START OVER runs, at which step,
    and whether it ended. Pure Python (no Qt, no ROS), so test/test_run_reset.py checks it with plain files.

    A start over counts as running from its first 'step' line until its 'done' / 'failed' line, but at most
    `stale_s` seconds after its last line: a start over killed from outside (it then writes no last line)
    must not lock the panel's buttons forever. Its longest silent wait is far shorter (spawn: 90 s)."""

    def __init__(self, path: Path, stale_s: float = 900.0, now: Callable[[], float] = time.time):
        self.path, self.stale_s, self.now = Path(path), stale_s, now
        self.offset = 0
        self.file_id = None         # (device, inode) of the file being read: Progress.start makes a new one
        self.reset_id = ''          # id of the start over seen last
        self.running = False
        self.n, self.total = 0, 0   # its current step number / number of steps
        self.step_text = ''         # its current step's title
        self.run_dir = ''           # the new run folder it announced
        self.t_last = 0.0           # when its last line was read

    def _identity(self):
        try:
            st = self.path.stat()
        except OSError:
            return None, 0
        return (st.st_dev, st.st_ino), st.st_size

    def skip_existing(self) -> None:
        """Load current state without replaying history when a panel starts during a reset.

        Completed resets stay history. An unfinished reset still locks the mission controls immediately,
        using its recorded timestamp rather than the panel's start time to decide whether it is stale.
        """
        self.file_id, _size = self._identity()
        entries, self.offset = read_progress(self.path)
        for entry in entries:
            self.feed(entry)
            if str(entry.get('id', '')) == self.reset_id:
                try:
                    self.t_last = float(entry.get('t', 0.0))
                except (TypeError, ValueError):
                    self.t_last = 0.0

    @property
    def busy(self) -> bool:
        """True while a start over runs (it has not finished and its last line is not too old)."""
        return self.running and self.now() - self.t_last < self.stale_s

    def poll(self) -> List[ProgressUpdate]:
        """Read the lines written since the last call and turn them into updates (oldest first)."""
        file_id, _size = self._identity()
        if file_id != self.file_id:            # a new start over replaced the file: read it from the start
            self.file_id, self.offset = file_id, 0
        entries, self.offset = read_progress(self.path, self.offset)
        return [u for u in (self.feed(e) for e in entries) if u is not None]

    def feed(self, e: dict) -> Optional[ProgressUpdate]:
        """One progress line -> one update (None for an empty line)."""
        kind, text, rid = str(e.get('kind', '')), str(e.get('text', '')).strip(), str(e.get('id', ''))
        step = str(e.get('step', ''))
        if not text:
            return None
        ours = rid == self.reset_id
        if kind == 'step':
            new = not ours or not self.running
            if new:
                self.reset_id, self.running = rid, True
            self.n, self.total, self.step_text = int(e.get('n', 0) or 0), int(e.get('total', 0) or 0), text
            self.run_dir = str(e.get('run_dir', '') or self.run_dir)
            self.t_last = self.now()
            return ProgressUpdate(f'STEP {self.n}/{self.total}: {text}', '', step, kind, new_reset=new)
        level = {'ok': 'ok', 'warn': 'warn', 'error': 'error', 'failed': 'error', 'done': 'ok'}.get(kind, '')
        if not (ours and self.running):
            # Not part of a running start over: e.g. the run watcher's "Run finished" line after a flight.
            return ProgressUpdate(text, level, step, kind, detail=kind == 'detail')
        self.t_last = self.now()
        self.run_dir = str(e.get('run_dir', '') or self.run_dir)
        if kind in ('done', 'failed'):
            self.running = False
            return ProgressUpdate(('START OVER done: ' if kind == 'done' else 'START OVER FAILED: ') + text,
                                  level, step, kind, finished=kind == 'done')
        return ProgressUpdate(text, level, step, kind, detail=kind == 'detail')


# ---------------------------------------------------------------------------------------------
# Photos: archive into the old run folder
# ---------------------------------------------------------------------------------------------

def move_folder_contents(src: Path, dst: Path) -> int:
    """Move every entry of `src` into `dst` (an entry with the same name in `dst` is replaced: demo.sh may
    already have COPIED the same photos there). The folder `src` itself stays, because video_cam and
    new_od keep using it. Returns the number of entries moved."""
    import shutil
    if not src.is_dir():
        return 0
    moved = 0
    for item in sorted(src.iterdir()):
        dst.mkdir(parents=True, exist_ok=True)
        target = dst / item.name
        try:
            if target.is_dir() and not target.is_symlink():
                shutil.rmtree(target, ignore_errors=True)
            elif target.exists() or target.is_symlink():
                target.unlink()
            shutil.move(str(item), str(target))       # a rename on the same disk, else copy + delete
            moved += 1
        except OSError:
            continue
    return moved


def archive_photos(photo_dir: str, old_run_dir: str, home: Path, stamp: str) -> tuple:
    """Move the team's photos (and new_od's result files) of the previous run out of the team folder.

    Into the previous run's folder (photos/ and detections/) when it is known, else into
    <home>/photo_archive/<stamp>/ (what start_sim.sh does). Returns (where, photos_moved, results_moved).
    """
    photos = Path(photo_dir)
    results = photos.parent / 'detection_results_sahi'
    if old_run_dir and Path(old_run_dir).is_dir():
        base = Path(old_run_dir)
        p_dst, r_dst = base / 'photos', base / 'detections'
    else:
        base = home / 'photo_archive' / stamp
        p_dst, r_dst = base / 'mapping_photos', base / 'detection_results_sahi'
    n_p = move_folder_contents(photos, p_dst)
    n_r = move_folder_contents(results, r_dst)
    photos.mkdir(parents=True, exist_ok=True)
    return str(base), n_p, n_r


# ---------------------------------------------------------------------------------------------
# The reset itself
# ---------------------------------------------------------------------------------------------

@dataclass
class Options:
    """What the user asked for (start_over.sh options)."""
    fly: bool = False
    world: str = ''
    mission: str = ''
    land: str = 'land'            # land (straight down, quickest) | rtl (fly home first)
    team_stack: bool = True
    detector: str = ''            # '' = what the running simulator uses
    watch_timeout_min: float = 120.0
    timeouts: Timeouts = field(default_factory=Timeouts)


class Resetter:
    """Runs the steps of START OVER in order (see the module docstring). All outside effects go through
    the adapters, so the tests replace them with fakes:
        ops (ProcessOps), gz_factory(world_name) -> GzOps, mav_factory(url) -> MavlinkOps,
        cmd (CommandOps), clock (Clock), progress (Progress)."""

    def __init__(self, opts: Options, home: Optional[Path] = None, ops=None, gz_factory=None, mav_factory=None,
                 cmd=None, clock=None, progress: Optional[Progress] = None, environ: Optional[Dict[str, str]] = None,
                 self_pids: Optional[Iterable[int]] = None):
        self.opts = opts
        self.home = home or sim_home()
        self.ops = ops or ProcessOps()
        self.gz_factory = gz_factory or GzOps
        self.mav_factory = mav_factory or (lambda url: MavlinkOps(url))
        self.cmd = cmd or CommandOps()
        self.clock = clock or Clock()
        self.environ = dict(os.environ if environ is None else environ)
        # With the real environment, a setting taken from the running simulator (see use_recorded_env) must
        # also reach the gz / ros2 tools this process runs; a test's own environ dict never touches os.environ.
        self._own_env = environ is None
        self.exclude = set(self_pids if self_pids is not None else ancestors())
        self.stamp = time.strftime('%Y%m%d-%H%M%S')
        steps = [s for s, _ in STEPS if s != 'fly' or opts.fly]
        self.steps = steps
        self.progress = progress or Progress(progress_path(self.home), self.stamp, len(steps))
        self.state: Optional[SimState] = None
        self.gz = None
        self.mission = ''
        self.old_run_dir = ''
        self.run_dir: Optional[Path] = None
        self.specs: Dict[str, Spec] = {}
        self.launcher_env: Dict[str, str] = {}     # TRACKER_CMD / FCU_URL of the old launcher.sh
        self.team_record: Dict[str, str] = {}      # state/team_stack.env written by demo.sh
        self.had_team = False
        self.lock_file = None

    # -- helpers -------------------------------------------------------------------------------
    def say(self, kind: str, text: str) -> None:
        self.progress.say(kind, text)

    def table(self) -> List[Proc]:
        return self.ops.table()

    def session_procs(self, roles: Sequence[str]) -> List[Proc]:
        st = self.state
        return select(self.table(), st.session, roles, self.environ.get('ROS_DOMAIN_ID', '0'), self.exclude)

    def stop(self, roles: Sequence[str], what: str, grace: Optional[float] = None) -> int:
        """Stop the session's processes with these roles; returns how many there were."""
        everyone = self.table()
        targets = select(everyone, self.state.session, roles, self.environ.get('ROS_DOMAIN_ID', '0'), self.exclude)
        if not targets:
            return 0
        self.say('info', f'stopping the {what} ({len(targets)} process{"es" if len(targets) != 1 else ""})')
        left = stop_procs(self.ops, targets, everyone, self.clock, self.opts.timeouts.stop if grace is None else grace,
                          self.exclude)
        if left:
            raise ResetError(f'Could not stop the {what}: still running: '
                             + '; '.join(f'{p.pid} {p.cmd[:80]}' for p in left))
        return len(targets)

    def tool(self, args: Sequence[str], timeout: float) -> int:
        """Run `ros2 run sim_bringup <args>`, showing its lines as progress details."""
        return self.cmd.run(['ros2', 'run', 'sim_bringup', *args], timeout, self.progress.detail)

    def spawn(self, spec: Spec, log_name: str, extra_env: Optional[Dict[str, str]] = None) -> int:
        env = dict(spec.env or self.environ)
        env['ASTRA_SIM_SESSION'] = self.state.session     # stop_sim.sh finds it by this tag
        if extra_env:
            env.update(extra_env)
        return self.ops.spawn(spec.argv, spec.cwd, env, str((self.run_dir or self.home / 'logs') / log_name))

    # -- main ------------------------------------------------------------------------------------
    def run(self) -> int:
        """Run every step. Returns 0 (done), 1 (a step failed), 3 (no simulator running) or 130 (Ctrl+C).

        Whatever happens, the progress file gets a last 'done' or 'failed' line (the demo panel waits for it
        to unlock its buttons), and only THEN is the one-at-a-time lock released."""
        try:
            self.acquire_lock()
        except ResetError as exc:
            # A rejected concurrent request must not replace or append to the active reset's progress.
            self.progress.echo(f'[start over] FAILED: {exc}')
            return 1
        code, ok = 0, True
        try:
            self.progress.start()
            for n, step in enumerate(self.steps, start=1):
                self.progress.step(step, n)
                getattr(self, f'step_{step}')()
            if self.opts.fly:
                text = f'the drone is flying the mission again. New run folder: {self.run_dir}'
            else:
                text = (f'ready to fly again. Press UPLOAD MISSION + START in the demo panel, or ARM + AUTO in '
                        f'Mission Planner. New run folder: {self.run_dir}')
        except NotRunning as exc:
            code, ok, text = 3, False, str(exc)
        except ResetError as exc:
            code, ok, text = 1, False, f'{exc}  (full log: {self.run_dir or self.home / "logs"})'
        except KeyboardInterrupt:
            # Ctrl+C in the start_over.sh terminal: what already restarted keeps running; a second START OVER
            # finishes the job (it copes with a missing drone, a stopped SITL and a stopped team stack).
            code, ok, text = 130, False, (f'stopped with Ctrl+C during step "{self.progress.step_id}". Run START '
                                          'OVER again to finish putting everything back.')
        except Exception as exc:  # noqa: BLE001 - a bug must still end with a clear line for the panel
            import traceback
            traceback.print_exc()
            code, ok, text = 1, False, f'unexpected error in step "{self.progress.step_id}": {exc!r}'
        try:
            self.progress.finish(ok, text)
        finally:
            self.release_lock()
        return code

    def release_lock(self) -> None:
        if self.lock_file is not None:
            try:
                fcntl.flock(self.lock_file, fcntl.LOCK_UN)
                self.lock_file.close()
            except OSError:
                pass
            self.lock_file = None

    def acquire_lock(self) -> None:
        """Claim the reset before touching its shared progress file or any simulator state."""
        state_dir = self.home / 'state'
        state_dir.mkdir(parents=True, exist_ok=True)
        # One start over at a time (the panel button and a terminal could both ask).
        self.lock_file = open(state_dir / 'start_over.lock', 'a+')
        try:
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock_file.close()
            self.lock_file = None
            raise ResetError('Another START OVER is already running. Wait for it to finish (see the demo panel '
                             'or ~/.astra_sim/logs/start_over_*.log).') from None

    # -- 1. check --------------------------------------------------------------------------------
    def step_check(self) -> None:
        state_dir = self.home / 'state'
        self.state = st = load_sim_state(self.home)
        if not st.session:
            raise NotRunning('No simulator session is recorded.')
        roles = {classify(p) for p in self.table() if p.session == st.session}
        if 'gz_server' not in roles:
            raise NotRunning(f'The simulator (session {st.session}) is not running any more.')
        self.use_recorded_env(st)
        domain = self.environ.get('ROS_DOMAIN_ID', '0')
        unowned = [p for p in self.table() if p.pid not in self.exclude and not p.session
                   and classify(p) == 'team' and p.env.get('ROS_DOMAIN_ID', '0') == domain]
        if unowned:
            raise ResetError('A team flight stack started outside this simulator uses the same ROS domain '
                             f'({domain}). Stop it in its own terminal before START OVER; no unowned process '
                             'was stopped: ' + '; '.join(f'{p.pid} {p.cmd[:80]}' for p in unowned))
        if self.opts.world and self.opts.world not in (st.world, st.world_name):
            raise ResetError(f'The running simulator shows the world "{st.world}", not "{self.opts.world}". '
                             'START OVER keeps the windows open, so it restarts the SAME world. To fly another '
                             f'world: bash stop_sim.sh, then bash demo.sh {self.opts.world}')
        if self.opts.detector and st.detector and self.opts.detector != st.detector:
            # The detector is part of the simulator (truth_detector starts with it), so it cannot change here.
            raise ResetError(f'The running simulator uses the detector "{st.detector}", not "{self.opts.detector}". '
                             'START OVER keeps the same setup. To switch: bash stop_sim.sh, then bash demo.sh '
                             f'{st.world} --detector {self.opts.detector}')
        if not st.model_file or not Path(st.model_file).is_file():
            raise ResetError(f'The drone model file of this simulator is missing ({st.model_file or "not recorded"}).'
                             ' Restart the simulator (stop_sim.sh, then demo.sh).')
        self.old_run_dir = self.find_old_run_dir()
        self.mission = self.opts.mission or self.mission_of_run(self.old_run_dir) or st.mission
        if not self.mission or not Path(self.mission).is_file():
            raise ResetError(f'No mission file to upload ({self.mission or "none recorded"}); give one with --mission FILE.')
        self.run_dir = new_run_dir(self.home, st.world, self.stamp)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.progress.run_dir = str(self.run_dir)
        (self.run_dir / 'start_over.env').write_text(
            f'SESSION={st.session}\nWORLD={st.world}\nMISSION={self.mission}\nRUN_DIR={self.run_dir}\n'
            f'PREVIOUS_RUN={self.old_run_dir}\nSTARTED={time.ctime()}\nFLY={int(self.opts.fly)}\n', encoding='utf-8')
        # From now on demo.sh (if it is still running) must leave everything open when it finishes.
        (state_dir / 'keep_open').write_text(st.session + '\n', encoding='utf-8')
        self.gz = self.gz_factory(st.world_name)
        self.say('ok', f'simulator found: world {st.world}, session {st.session}, mission {self.mission}')
        self.say('info', f'new run folder: {self.run_dir}')

    def use_recorded_env(self, st: SimState) -> None:
        """Talk to the simulator in ITS Gazebo partition and ROS domain (recorded by sim.launch.py), even when
        start over runs from a terminal with other settings: otherwise `gz service` would ask another (or no)
        Gazebo, and the restarted team stack would not see the simulator's ROS topics."""
        for key, value in (('GZ_PARTITION', st.gz_partition), ('ROS_DOMAIN_ID', st.ros_domain_id)):
            if not value or self.environ.get(key, '0' if key == 'ROS_DOMAIN_ID' else '') == value:
                continue
            self.say('info', f'using the simulator\'s own {key}={value} (this terminal had '
                             f'"{self.environ.get(key, "")}")')
            self.environ[key] = value
            if self._own_env:
                os.environ[key] = value

    def find_old_run_dir(self) -> str:
        """The run folder of the run that is ending: the last start over's, else the one demo.sh gave the
        simulator (ASTRA_RUN_DIR of its processes / score_run's run_dir), else '' (start_sim.sh)."""
        current = self.home / 'state' / 'current_run.json'
        try:
            data = json.loads(current.read_text(encoding='utf-8'))
            if data.get('session') == self.state.session and Path(data.get('run_dir', '')).is_dir():
                return data['run_dir']
        except (OSError, ValueError):
            pass
        procs = [p for p in self.table() if p.session == self.state.session]
        for p in procs:
            if classify(p) == 'score_run':
                value = ros_param_from_argv(p.argv, 'run_dir') or p.env.get('ASTRA_RUN_DIR', '')
                if value and Path(value).is_dir():
                    return value
        for p in procs:
            value = p.env.get('ASTRA_RUN_DIR', '')
            if value and Path(value).is_dir():
                return value
        return ''

    @staticmethod
    def mission_of_run(run_dir: str) -> str:
        """The mission a run flew (MISSION= in its demo.env / start_over.env), '' if not recorded."""
        for name in ('start_over.env', 'demo.env'):
            try:
                for line in (Path(run_dir) / name).read_text(encoding='utf-8').splitlines():
                    if line.startswith('MISSION=') and line[8:].strip():
                        return line[8:].strip()
            except OSError:
                continue
        return ''

    # -- 2. handover -----------------------------------------------------------------------------
    def step_handover(self) -> None:
        demo = demo_scripts(self.table(), self.state.session, self.exclude)
        n = self.stop(['phase_monitor', 'mission_tool', 'stack_check', 'watcher'], 'previous run\'s watchers', 10.0)
        if not n and not demo:
            self.say('info', 'nothing to take over (no demo.sh / run watcher running)')
        if demo:
            self.say('info', 'demo.sh is finishing the previous run (score report, photos); it leaves everything open')
            if not wait_until(self.clock, self.opts.timeouts.demo_exit,
                              lambda: not any(self.ops.alive(p.pid) for p in demo), 1.0):
                self.say('warn', 'demo.sh is still finishing; it will exit by itself without closing anything')
        self.ask_old_score()

    def ask_old_score(self) -> None:
        """Make sure the previous run has its score report before its recorder is restarted."""
        if not self.old_run_dir or (Path(self.old_run_dir) / 'score_report.md').exists():
            return
        if not self.session_procs(['score_run']):
            return
        self.say('info', 'asking the run recorder for the previous run\'s score report')
        self.cmd.run(['timeout', '25', 'ros2', 'service', 'call', '/sim/demo/write_score', 'std_srvs/srv/Trigger', '{}'],
                     30.0, lambda line: None)

    # -- 3. stop_team ----------------------------------------------------------------------------
    def step_stop_team(self) -> None:
        self.team_record = self.read_team_record()
        if self.team_record.get('TEAM_STACK_WHERE') == 'external':
            self.say('info', 'the team flight stack runs on another computer / container (run.team_stack: '
                             'external): restart it THERE after this start over')
            return
        # How demo.sh started it: TRACKER_CMD and FCU_URL are in launcher.sh's own environment (demo.sh sets
        # them for it), so the restart uses exactly the same ones, whatever profile demo.sh used.
        team = self.session_procs(['team'])
        for p in team:
            if 'launcher.sh' in p.cmd:
                self.launcher_env = {k: p.env[k] for k in ('TRACKER_CMD', 'FCU_URL') if p.env.get(k)}
                break
        # The process groups the team stack LEADS (demo.sh and start over start launcher.sh with setsid, so it
        # is the leader of its own group). Everything else in such a group belongs to the team stack too, e.g.
        # a plug-in's own detector node that the patterns above do not know.
        groups = {p.pgid for p in team if p.pid == p.pgid and p.pgid > 1}
        n = self.stop(['team'], 'team flight stack (launcher.sh, MAVROS, team nodes)', self.opts.timeouts.stop)
        self.had_team = n > 0
        if not n:
            self.say('info', 'the team flight stack was not running')
        everyone = self.table()
        rest = [p for p in everyone if p.pgid in groups and p.session == self.state.session
                and p.pid not in self.exclude
                and classify(p) not in PROTECTED_ROLES]
        if rest:
            self.say('info', f'also stopping {len(rest)} other program(s) the team stack had started (plug-ins)')
            left = stop_procs(self.ops, rest, everyone, self.clock, self.opts.timeouts.stop, self.exclude)
            if left:
                raise ResetError('Could not stop part of the team stack: still running: '
                                 + '; '.join(f'{p.pid} {p.cmd[:80]}' for p in left))

    def read_team_record(self) -> Dict[str, str]:
        """state/team_stack.env, written by demo.sh when it starts the team stack (TRACKER_CMD, FCU_URL,
        STACK_CHECK_ARGS, TEAM_STACK_WHERE), if it belongs to this session; {} otherwise."""
        rec: Dict[str, str] = {}
        try:
            for line in (self.home / 'state' / 'team_stack.env').read_text(encoding='utf-8').splitlines():
                key, sep, value = line.partition('=')
                if sep:
                    rec[key.strip()] = value.strip()
        except OSError:
            return {}
        return rec if rec.get('SESSION') == self.state.session else {}

    # -- 4. land ---------------------------------------------------------------------------------
    def step_land(self) -> None:
        to = self.opts.timeouts
        if not self.gz.model_exists(DRONE_ENTITY):
            self.say('info', 'the old drone is already absent; no landing is needed to finish the reset')
            return
        if not self.session_procs(['mavproxy']):
            raise ResetError('MAVProxy is not running, so the drone\'s landing cannot be verified. '
                             'The drone is left in place; restore the link and land it before START OVER.')
        try:
            mav = self.mav_factory(self.state.gcs_url)
        except Exception as exc:  # noqa: BLE001 - e.g. the port is used by another program
            raise ResetError(f'Cannot talk to the autopilot on {self.state.gcs_url} ({exc}). '
                             'The drone is left in place; restore the link and land it before START OVER.') from None
        try:
            snap = mav.snapshot(2.0)
            alt = f'{snap.rel_alt:.1f} m' if snap.rel_alt is not None else '? m'
            if not snap.armed and snap.on_ground:
                self.say('ok', f'the drone is on the ground and disarmed (mode {snap.mode}, height {alt})')
                return
            if snap.on_ground:
                self.say('info', f'armed on the ground (mode {snap.mode}): disarming')
                mav.disarm(force=True)
                if not wait_until(self.clock, 10.0, lambda: not mav.snapshot(1.0).armed, 0.5):
                    raise ResetError('The autopilot did not disarm on the ground. The drone is left in place; '
                                     'disarm it before trying START OVER again.')
                return
            mode = 'RTL' if self.opts.land == 'rtl' else 'LAND'
            self.say('info', f'the drone is FLYING ({alt}, mode {snap.mode}): commanding {mode} and waiting until '
                             f'it has landed and disarmed (at most {to.land:.0f} s)')
            if not mav.set_mode(mode):
                self.say('warn', f'the autopilot did not confirm {mode}; trying LAND')
                mav.set_mode('LAND')
            end = self.clock.time() + to.land
            last_note = -1e9
            while self.clock.time() < end:
                snap = mav.snapshot(1.0)
                self.clock.sleep(0.5)              # (the timeout is measured on the clock, not in snapshots)
                if not snap.armed and snap.on_ground:
                    self.say('ok', 'LANDED and disarmed')
                    return
                if self.clock.time() - last_note >= 10.0:
                    last_note = self.clock.time()
                    alt = f'{snap.rel_alt:.1f} m' if snap.rel_alt is not None else '? m'
                    self.progress.detail(f'landing: height {alt}, mode {snap.mode}, '
                                         f'{"armed" if snap.armed else "disarmed"}')
            raise ResetError(f'The drone did not land within {to.land:.0f} s. It is left in place; '
                             'land and disarm it before trying START OVER again.')
        finally:
            mav.close()

    # -- 5. archive ------------------------------------------------------------------------------
    def step_archive(self) -> None:
        photo_dir = self.state.photo_dir
        if not photo_dir:
            self.say('warn', 'the team photo folder is not recorded; nothing archived')
            return
        where, n_p, n_r = archive_photos(photo_dir, self.old_run_dir, self.home, self.stamp)
        if n_p or n_r:
            self.say('ok', f'moved {n_p} photo(s) and {n_r} detector result(s) of the previous run to {where}')
        else:
            self.say('info', 'the team photo folder was already empty')
        self.say('info', f'the team photo folder is empty again: {photo_dir}')

    # -- 6. remove -------------------------------------------------------------------------------
    def record_autopilot_specs(self) -> None:
        """Record how SITL and MAVProxy were started (from the running processes, else the saved file,
        else rebuilt from the simulator's settings), before anything is stopped."""
        saved = {}
        path = self.home / 'state' / 'start_over_specs.json'
        try:
            saved = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            saved = {}
        for role in ('sitl', 'mavproxy'):
            live = self.session_procs([role])
            if live:
                self.specs[role] = spec_from_proc(live[0])
            elif isinstance(saved.get(role), dict) and saved[role].get('argv'):
                self.specs[role] = Spec(list(saved[role]['argv']), saved[role].get('cwd', ''), dict(self.environ))
            else:
                self.specs[role] = self.rebuilt_spec(role)
        try:
            path.write_text(json.dumps({r: s.to_json() for r, s in self.specs.items()}, indent=1), encoding='utf-8')
        except OSError:
            pass

    def rebuilt_spec(self, role: str) -> Spec:
        """SITL / MAVProxy command built from the simulator's settings (only when no record exists)."""
        st = self.state
        try:
            from ament_index_python.packages import get_package_share_directory
            from sim_bringup import sim_paths as sp
        except ImportError as exc:
            raise ResetError(f'cannot rebuild the {role} command ({exc}); restart the simulator') from None
        if role == 'sitl':
            binary = sp.find_sitl_binary('')
            parm = os.path.join(get_package_share_directory('sim_models'), 'config', 'astra_sitl.parm')
            defaults, _ = sp.sitl_defaults(parm)
            if not binary:
                raise ResetError('ArduCopter SITL was not found (~/sitl/arducopter); run 02_setup_ardupilot.sh')
            argv = [binary, '--model', 'JSON', '-I', str(st.sitl_instance), '--home', st.home]
            if defaults:
                argv += ['--defaults', ','.join(defaults)]
            return Spec(argv, st.sitl_run_dir, dict(self.environ))
        mavproxy = sp.find_mavproxy('')
        if not mavproxy:
            raise ResetError('MAVProxy was not found (~/sitl/venv/bin/mavproxy.py)')
        argv = [mavproxy, '--master', f'tcp:127.0.0.1:{st.serial0_port}', '--out', f'udp:127.0.0.1:{st.gcs_port}',
                '--out', f'udp:127.0.0.1:{st.mavros_port}', '--out', f'udp:127.0.0.1:{st.panel_port}']
        if st.windows_gcs:
            argv += ['--out', st.windows_gcs]
        argv += ['--daemon', '--state-basedir', st.sitl_run_dir, '--logfile', 'mav.tlog']
        return Spec(argv, st.sitl_run_dir, dict(self.environ))

    def step_remove(self) -> None:
        st, to = self.state, self.opts.timeouts
        self.record_autopilot_specs()
        if not self.gz.model_exists(DRONE_ENTITY):
            self.say('info', f'there is no "{DRONE_ENTITY}" in Gazebo (an earlier start over stopped half way)')
            return
        if not self.session_procs(['sitl']):
            # Gazebo waits for SITL in lock-step: without a SITL it cannot remove anything. A temporary SITL
            # with the same command line wakes it up (it is stopped again in the next step).
            self.say('warn', 'SITL is not running, so Gazebo is waiting for it: starting it briefly')
            self.spawn(self.specs['sitl'], 'sitl.log')
            wait_until(self.clock, to.sitl_port, lambda: self.gz.tcp_listening(st.serial0_port))
            self.clock.sleep(3.0)
        if not self.gz.remove_model(DRONE_ENTITY):
            raise ResetError('Gazebo did not remove the drone (service /world/.../remove failed). Is the Gazebo '
                             'server busy or frozen? Stop everything with stop_sim.sh and start again.')
        gone = wait_until(self.clock, to.remove, lambda: not self.gz.udp_bound(st.fdm_port)
                          and not self.gz.model_exists(DRONE_ENTITY), 1.0)
        if not gone:
            raise ResetError(f'Gazebo said it removed the drone, but after {to.remove:.0f} s it is still there '
                             f'(or UDP {st.fdm_port} is still open). Stop everything with stop_sim.sh and start again.')
        self.say('ok', f'the old drone is gone (UDP {st.fdm_port} is free again)')

    # -- 7. stop_ap ------------------------------------------------------------------------------
    def step_stop_ap(self) -> None:
        n = self.stop(['mavproxy', 'sitl'], 'autopilot (MAVProxy + ArduCopter SITL)', 5.0)
        st = self.state
        if not wait_until(self.clock, 15.0, lambda: not self.gz.tcp_listening(st.serial0_port)):
            raise ResetError(f'TCP {st.serial0_port} is still in use after stopping SITL; stop_sim.sh, then start again')
        self.say('ok' if n else 'info', 'the autopilot is stopped' if n else 'the autopilot was not running')

    # -- 8. spawn --------------------------------------------------------------------------------
    def step_spawn(self) -> None:
        st, to = self.state, self.opts.timeouts
        sp = st.spawn
        if not self.gz.spawn_model(st.model_file, DRONE_ENTITY, sp['x'], sp['y'], sp['z'], sp['yaw']):
            raise ResetError('Gazebo did not accept the new drone (service /world/.../create failed). '
                             'Stop everything with stop_sim.sh and start again.')
        if not wait_until(self.clock, to.spawn, lambda: self.gz.udp_bound(st.fdm_port), 1.0):
            raise ResetError(f'The new drone did not open UDP {st.fdm_port} within {to.spawn:.0f} s (its ArduPilot '
                             'plugin did not load). See the simulator log, then stop_sim.sh + demo.sh.')
        self.say('ok', f'a fresh drone stands on the start point (x {sp["x"]:.1f}, y {sp["y"]:.1f}, '
                       f'heading {math.degrees(sp["yaw"]):.0f} deg)')
        drone = self.gz.position('/sim/drone/odometry')
        for name in PAYLOADS:
            pos = self.gz.position(f'/sim/payload/{name}/odometry')
            if drone and pos:
                gap = math.dist(drone, pos)
                if gap < 0.5:
                    self.say('ok', f'{name} payload is attached under the drone ({gap * 100:.0f} cm from its centre)')
                else:
                    self.say('warn', f'{name} payload is {gap:.1f} m from the drone: it may not be attached')
            else:
                self.say('info', f'{name} payload position not received yet (it is part of the new drone)')

    # -- 9. start_ap -----------------------------------------------------------------------------
    def step_start_ap(self) -> None:
        st, to = self.state, self.opts.timeouts
        self.spawn(self.specs['sitl'], 'sitl.log')
        if not wait_until(self.clock, to.sitl_port, lambda: self.gz.tcp_listening(st.serial0_port), 1.0):
            raise ResetError(f'SITL did not start listening on TCP {st.serial0_port} within {to.sitl_port:.0f} s '
                             f'(see {self.run_dir}/sitl.log)')
        self.clock.sleep(1.0)
        self.spawn(self.specs['mavproxy'], 'mavproxy.log')
        self.clock.sleep(3.0)
        if not self.session_procs(['mavproxy']):
            raise ResetError(f'MAVProxy stopped right after starting (see {self.run_dir}/mavproxy.log)')
        self.say('ok', f'SITL and MAVProxy run again with their old command lines (MAVLink TCP {st.serial0_port}, '
                       f'outputs {st.gcs_port}/{st.mavros_port}/{st.panel_port}'
                       + (f', Windows {st.windows_gcs}' if st.windows_gcs else '') + ')')
        self.start_guard()

    def start_guard(self) -> None:
        """One guard per session: when the Gazebo server ends (window closed, Ctrl+C, stop_sim.sh), it stops
        the processes start over started, so nothing is left behind."""
        if self.session_procs(['guard']):
            return
        self.spawn(Spec([sys.executable, '-m', 'sim_demo.run_reset', 'guard', '--session', self.state.session],
                        '', dict(self.environ)), 'guard.log')

    # -- 10. helpers -----------------------------------------------------------------------------
    def step_helpers(self) -> None:
        run = str(self.run_dir)
        everyone = self.table()
        sess = self.state.session
        helpers = select(everyone, sess, ['score_run', 'payload_release', 'markers', 'truth_detector'],
                         exclude=self.exclude)
        specs = []
        st = self.state
        fallback = {'world': st.world, 'photo_dir': st.photo_dir}
        for p in helpers:
            spec = spec_from_proc(p)
            role = classify(p)
            spec.argv = drop_missing_params_files(
                spec.argv, dict(fallback, detector=st.detector) if role == 'score_run' else
                (fallback if role in ('markers', 'truth_detector') else {}))
            if role == 'score_run':
                spec.argv = with_ros_param(spec.argv, 'run_dir', run)
            specs.append((role, spec))
        if helpers:
            left = stop_procs(self.ops, helpers, everyone, self.clock, 8.0, self.exclude)
            if left:
                raise ResetError('could not stop: ' + ', '.join(classify(p) for p in left))
        for role, spec in specs:
            self.spawn(spec, 'helpers.log', {'ASTRA_RUN_DIR': run})
        names = ', '.join(r for r, _ in specs) or 'none were running'
        self.say('ok', f'restarted with fresh memories: {names}')
        if any(r == 'score_run' for r, _ in specs):
            self.say('info', f'the score report of the new run will be written to {run}/score_report.md')
        follow = select(everyone, sess, ['gz_follow'], exclude=self.exclude)
        if self.gz.gui_running():
            offset = ['-12', '0', '6']
            if follow and '--offset' in follow[0].argv:
                i = follow[0].argv.index('--offset')
                offset = follow[0].argv[i + 1:i + 4]
            self.spawn(Spec(['ros2', 'run', 'sim_bringup', 'gz_follow', '--entity', DRONE_ENTITY, '--offset', *offset,
                             '--timeout', '120'], '', dict(self.environ)), 'helpers.log')
            self.say('info', 'the Gazebo window will follow the new drone in a few seconds')
        try:
            (self.home / 'state' / 'current_run.json').write_text(json.dumps(
                {'session': sess, 'run_dir': run, 'world': self.state.world, 'mission': self.mission,
                 'started': time.strftime('%Y-%m-%d %H:%M:%S'), 'by': 'start over'}, indent=1), encoding='utf-8')
        except OSError:
            pass

    # -- 11. ready -------------------------------------------------------------------------------
    def step_ready(self) -> None:
        rc = self.tool(['mission_tool', '--url', self.state.gcs_url, '--timeout', str(int(self.opts.timeouts.ready)),
                        'wait-ready'], self.opts.timeouts.ready + 60)
        if rc != 0:
            raise ResetError(f'the new autopilot did not become ready to arm (see {self.run_dir}/sitl.log). '
                             'Try START OVER again, or stop_sim.sh + demo.sh.')
        self.say('ok', 'the autopilot is ready (GPS fix, healthy EKF, pre-arm checks passed)')

    # -- 12. upload ------------------------------------------------------------------------------
    def step_upload(self) -> None:
        rc = self.tool(['mission_tool', '--url', self.state.gcs_url, 'upload', '--mission', self.mission], 90.0)
        if rc != 0:
            raise ResetError(f'the mission upload failed ({self.mission})')
        self.say('ok', f'mission uploaded: {self.mission}')

    # -- 13. team --------------------------------------------------------------------------------
    def team_env(self) -> Dict[str, str]:
        """The environment for ./launcher.sh sim: the same TRACKER_CMD / FCU_URL as before (recorded from the
        old launcher.sh or demo.sh's record), else what demo.sh uses without a profile."""
        st = self.state
        detector = self.opts.detector or st.detector or 'real'
        env = dict(self.environ)
        env.update({'TRACKER_CMD': f'ros2 launch sim_bringup team_stack.launch.py detector:={detector}',
                    'FCU_URL': f'udp://:{st.mavros_port}@localhost:{st.mavros_port}'})
        for key in ('TRACKER_CMD', 'FCU_URL'):
            value = self.launcher_env.get(key) or self.team_record.get(key)
            if value and not (key == 'TRACKER_CMD' and self.opts.detector):
                env[key] = value
        env.update({'ASTRA_SIM_SESSION': st.session, 'ASTRA_RUN_DIR': str(self.run_dir)})
        return env

    def stack_check_args(self) -> List[str]:
        """stack_check options: demo.sh's own (recorded, e.g. a plug-in's --skip-node), else --need-detector
        for the real detector."""
        if self.team_record.get('STACK_CHECK_ARGS') is not None and not self.opts.detector:
            return self.team_record['STACK_CHECK_ARGS'].split()
        detector = self.opts.detector or self.state.detector or 'real'
        return ['--need-detector'] if detector == 'real' else []

    def step_team(self) -> None:
        if not self.opts.team_stack:
            self.say('info', '--no-team-stack: the team flight stack is not started (start ./launcher.sh sim yourself)')
            return
        if self.team_record.get('TEAM_STACK_WHERE') == 'external':
            self.say('info', 'the team flight stack runs elsewhere: start it there now, then fly')
            return
        repo = self.environ.get('ASTRA_REPO', '') or os.path.expanduser('~/ODLC_Machine_Inferencing_System_2025-2026')
        launcher = Path(repo) / 'launcher.sh'
        if not launcher.is_file():
            raise ResetError(f'the team\'s launcher.sh is missing in {repo} (is ASTRA_REPO right?)')
        need = self.stack_check_args()
        log = self.run_dir / 'team_stack.log'
        for attempt in (1, 2):
            # Exactly what demo.sh does: ( cd $ASTRA_REPO && TRACKER_CMD=... FCU_URL=... setsid ./launcher.sh sim )
            self.ops.spawn(['./launcher.sh', 'sim'], repo, self.team_env(), str(log))
            self.say('info', f'./launcher.sh sim started (log {log}); waiting until it is ready '
                             '(MAVROS connected, mission read, detector loaded, nodes stay up 20 s)')
            rc = self.tool(['stack_check', '--timeout', str(int(self.opts.timeouts.stack)), '--settle', '20', *need],
                           self.opts.timeouts.stack + 300)
            if rc == 0:
                break
            if attempt == 1:
                self.say('warn', 'the team stack is not ready (a node stopped or never started): starting it once more')
                self.stop(['team'], 'team flight stack')
                with open(log, 'a', encoding='utf-8') as f:
                    f.write(f'===== start over: team stack restarted {time.strftime("%H:%M:%S")} =====\n')
                continue
            raise ResetError(f'the team flight stack did not get ready (see {log})')
        self.say('ok', 'the team flight stack is ready')
        # Same as demo.sh: point the camera straight down through the team's own camera command.
        self.cmd.run(['timeout', '15', 'ros2', 'topic', 'pub', '-w', '1', '--once', '/camera/command',
                      'std_msgs/msg/String', "{data: 'gimbal_set_angles 0,-90'}"], 20.0, lambda line: None)

    # -- 14. watch -------------------------------------------------------------------------------
    def step_watch(self) -> None:
        run = str(self.run_dir)
        argv = [sys.executable, '-m', 'sim_demo.run_reset', 'watch-run', '--run-dir', run,
                '--photo-dir', self.state.photo_dir, '--timeout-min', str(self.opts.watch_timeout_min)]
        self.spawn(Spec(argv, '', dict(self.environ)), 'phases.log', {'ASTRA_RUN_DIR': run})
        self.say('ok', f'the run watcher records the flight (phases: {run}/phases.log; photos and the score '
                       'report are collected after the landing)')

    # -- 15. fly ---------------------------------------------------------------------------------
    def step_fly(self) -> None:
        rc = self.tool(['mission_tool', '--url', self.state.gcs_url, 'fly'], 300.0)
        if rc != 0:
            raise ResetError('the mission did not start (see the messages above); press UPLOAD MISSION + START '
                             'in the demo panel to try again')
        self.say('ok', 'TAKE-OFF confirmed: the drone flies the search mission')


# ---------------------------------------------------------------------------------------------
# The run watcher (runs in the background after a start over, until the drone has landed)
# ---------------------------------------------------------------------------------------------

def collect_run(run_dir: Path, photo_dir: str, cmd=None, wait_report_s: float = 30.0, clock=None) -> List[str]:
    """After the landing: make sure score_report.md exists (score_run writes it ~8 s after the landing,
    otherwise it is asked to), and COPY the photos + detector results into the run folder (the team folder
    keeps them, so the demo panel can still show them; the next start over moves them away).
    Returns plain lines for the log."""
    clock = clock or Clock()
    cmd = cmd or CommandOps()
    lines = []
    report = run_dir / 'score_report.md'
    if not wait_until(clock, wait_report_s, report.exists, 1.0):
        cmd.run(['timeout', '25', 'ros2', 'service', 'call', '/sim/demo/write_score', 'std_srvs/srv/Trigger', '{}'],
                30.0, lambda line: None)
    lines.append(f'score report: {report}' if report.exists() else 'no score report was written (see helpers.log)')
    import shutil
    for src, dst in ((Path(photo_dir), run_dir / 'photos'),
                     (Path(photo_dir).parent / 'detection_results_sahi', run_dir / 'detections')):
        files = [f for f in src.iterdir() if f.is_file()] if src.is_dir() else []
        if files:
            dst.mkdir(parents=True, exist_ok=True)
            for f in files:
                try:
                    shutil.copy2(f, dst / f.name)
                except OSError:
                    pass
            lines.append(f'{len(files)} file(s) copied to {dst}')
    return lines


def watch_run(run_dir: Path, photo_dir: str, timeout_min: float, home: Optional[Path] = None) -> int:
    """Run sim_bringup's phase monitor for the new run until the drone has landed, then collect the run."""
    home = home or sim_home()
    prog = Progress(progress_path(home), f'watch-{run_dir.name}', 0)
    argv = ['ros2', 'run', 'sim_bringup', 'phase_monitor', '--log', str(run_dir / 'phases.jsonl'),
            '--photo-dir', photo_dir, '--timeout', str(int(timeout_min * 60))]
    child = subprocess.Popen(argv)
    stop = {'sig': None}

    def forward(sig, _frame):
        stop['sig'] = sig
        child.send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    rc = child.wait()          # (Python retries the wait itself when a signal arrives meanwhile)
    reason = {0: 'the drone has landed', 3: f'no landing within {timeout_min:.0f} min'}.get(rc, f'stopped ({rc})')
    if stop['sig'] is not None:
        reason = 'a new START OVER began'
    print(f'[run watcher] {reason}: collecting the run into {run_dir}', flush=True)
    lines = collect_run(run_dir, photo_dir, wait_report_s=30.0 if rc == 0 else 3.0)
    for line in lines:
        print(f'[run watcher] {line}', flush=True)
    if stop['sig'] is None:
        prog.say('ok' if rc == 0 else 'warn', f'Run finished ({reason}). ' + ' | '.join(lines))
    return 0


def guard(session: str, home: Optional[Path] = None, ops=None, clock=None, poll_s: float = 2.0) -> int:
    """Wait while the session's Gazebo server runs; when it is gone, stop the processes of the session
    that are still running (the SITL, MAVProxy, team stack and helpers a start over started: they are not
    children of the simulator's launch, so its Ctrl+C does not reach them)."""
    ops, clock = ops or ProcessOps(), clock or Clock()
    me = set(ancestors())
    # Start over never restarts Gazebo, so the server's pids found now are the ones to wait for. Checking
    # only those pids is cheap; reading the whole process table every 2 s for hours would not be.
    servers = [p.pid for p in ops.table() if p.session == session and classify(p) == 'gz_server']
    while any(ops.alive(pid) for pid in servers):
        clock.sleep(poll_s)
    everyone = ops.table()
    left = [p for p in everyone if p.session == session and p.pid not in me and classify(p) not in ('guard',)]
    if left:
        print(f'[guard] the simulator of session {session} has stopped: stopping {len(left)} leftover process(es)',
              flush=True)
        stop_procs(ops, left, everyone, clock, 10.0, me)
    return 0


# ---------------------------------------------------------------------------------------------
# Mission Planner steps (printed by demo.sh --manual and start_over.sh, and shown in the docs)
# ---------------------------------------------------------------------------------------------

def windows_path(linux_path: str) -> str:
    """The Windows name of a Linux file (wslpath -w, e.g. \\\\wsl.localhost\\Ubuntu-22.04\\home\\...); '' outside WSL."""
    try:
        res = subprocess.run(['wslpath', '-w', linux_path], capture_output=True, text=True, timeout=5, check=False)
        return res.stdout.strip() if res.returncode == 0 else ''
    except (OSError, subprocess.TimeoutExpired):
        return ''


def source_mission_path(mission: str, environ: Optional[Dict[str, str]] = None) -> str:
    """The same mission file in the Windows copy of the repo (D:\\...\\sim_worlds\\missions\\<file>), when the
    Windows folder is known ($ASTRA_WINDOWS_REPO, set by the simulation/windows/*.bat launchers); else ''."""
    env = os.environ if environ is None else environ
    win_repo = env.get('ASTRA_WINDOWS_REPO', '').strip().rstrip('\\/')
    if not win_repo or '/sim_worlds/missions/' not in (mission or '').replace('\\', '/'):
        return ''                          # unknown Windows folder, or a mission from somewhere else
    return f'{win_repo}\\ros2_ws\\src\\simulation\\sim_worlds\\missions\\{os.path.basename(mission)}'


def mp_steps(mission: str, windows_gcs: str = '', environ: Optional[Dict[str, str]] = None,
             wsl_name: Callable[[str], str] = windows_path) -> List[str]:
    """The competition procedure in Mission Planner, as numbered plain lines (see simulation/README.md)."""
    port = windows_gcs.rsplit(':', 1)[-1] if windows_gcs else '14550'
    src = source_mission_path(mission, environ)
    unc = wsl_name(mission) if mission else ''
    lines = [
        'MISSION PLANNER (Windows) - fly it like at the competition:',
        '  1. CONNECT: open Mission Planner. Top right, set the connection box to UDP, click CONNECT, keep',
        f'     port {port}, click OK. The drone appears on the map at the runway; the HUD says DISARMED.',
        '  2. MISSION: click PLAN (top left; older versions say FLIGHT PLAN). On the right click',
        '     "Load File" (older: "Load WP File") and pick the mission file:',
    ]
    if src:
        lines.append(f'        {src}')
    if unc:
        lines.append(f'        {"or the copy the simulator uses: " if src else ""}{unc}')
    if not src and not unc:
        lines.append(f'        {mission}')
    lines += [
        '     Then click "Write" (older: "Write WPs"). A small window shows the upload and closes. The team',
        '     code then re-reads the mission by itself (DATA > Messages shows "Flight plan received").',
        '  3. WAIT: click DATA (top left). Wait until the autopilot has settled: about 30 s after the start',
        '     ("PreArm: ..." and "Gyros inconsistent" messages before that are normal).',
        '  4. ARM: in the Actions tab under the HUD, click "Arm/Disarm". The HUD says ARMED.',
        '  5. AUTO: within 10 s (or the motors switch off again), pick AUTO in the mode box of the Actions',
        '     tab and click "Set Mode" (or click the "Auto" button). The drone takes off.',
        '     (Also fine: "Set Mode" AUTO first, then "Arm/Disarm": it takes off at once.)',
        '  6. WATCH: take-off, search lanes with a photo every second, detections, the team code adds the',
        '     target waypoints, both payloads fall, the drone flies home (RTL) and lands.',
        'RUN IT AGAIN: press START OVER in the demo panel (click it twice) and wait for "START OVER done"',
        '  (about 2-3 minutes); then repeat steps 2-5. Mission Planner stays connected (if it lost the',
        '  link: DISCONNECT, then CONNECT again). Always do step 2: the autopilot remembers the last',
        '  mission it had, even after a restart (after a flight: with the target waypoints the team code added).',
    ]
    return lines


# ---------------------------------------------------------------------------------------------
# The demo panel's side: how to start a START OVER in the background
# ---------------------------------------------------------------------------------------------

def find_start_over_script(environ: Optional[Dict[str, str]] = None, here: Optional[str] = None) -> Optional[str]:
    """simulation/scripts/start_over.sh of the repo this simulator runs from ($ASTRA_REPO first, then up
    from this file, which a --symlink-install build links into the repo), or None."""
    env = os.environ if environ is None else environ
    candidates = []
    if env.get('ASTRA_REPO'):
        candidates.append(Path(env['ASTRA_REPO']) / 'simulation' / 'scripts' / 'start_over.sh')
    real = Path(os.path.realpath(here or __file__))
    for parent in list(real.parents)[:8]:
        candidates.append(parent / 'simulation' / 'scripts' / 'start_over.sh')
    for c in candidates:
        if c.is_file():
            return str(c)
    return None


def launch_start_over(args: Sequence[str] = (), environ: Optional[Dict[str, str]] = None,
                      log_dir: Optional[Path] = None) -> tuple:
    """Start a START OVER in the background, detached from the caller (the panel keeps running and the
    processes start over starts are not the panel's children). Returns (pid, log_path)."""
    env = dict(os.environ if environ is None else environ)
    log_dir = log_dir or (sim_home() / 'logs')
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f'start_over_{time.strftime("%Y%m%d-%H%M%S")}_panel.log'
    script = find_start_over_script(env)
    # (--no-fresh is start_over.sh's own option; the Python fallback never starts a fresh demo anyway)
    argv = ['bash', script, *args] if script else [sys.executable, '-m', 'sim_demo.run_reset', 'reset',
                                                   *[a for a in args if a != '--no-fresh']]
    with open(log_path, 'ab') as log:
        proc = subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                                start_new_session=True)
    return proc.pid, str(log_path)


# ---------------------------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------------------------

def main(argv=None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = argparse.ArgumentParser(prog='run_reset', description='START OVER for the ASTRA simulator')
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('reset', help='bring the running simulator back to the start of the mission')
    p.add_argument('--fly', action='store_true', help='arm and start the mission (AUTO) at the end')
    p.add_argument('--world', default='', help='only check that the running world is this one')
    p.add_argument('--mission', default='', help='mission file to upload (default: the one the last run flew)')
    p.add_argument('--land', choices=('land', 'rtl'), default='land', help='how a flying drone comes down first')
    p.add_argument('--land-timeout', type=float, default=Timeouts.land, help='seconds to wait for the landing')
    p.add_argument('--no-team-stack', action='store_true', help='do not start ./launcher.sh sim again')
    p.add_argument('--detector', choices=('', 'real', 'truth'), default='',
                   help='only a check: START OVER keeps the detector the simulator runs (another one: stop_sim.sh, '
                        'then demo.sh --detector X)')
    p.add_argument('--watch-timeout-min', type=float, default=120.0, help='how long the run watcher waits')
    w = sub.add_parser('watch-run', help='(internal) watch one run until landed, then collect it')
    w.add_argument('--run-dir', required=True)
    w.add_argument('--photo-dir', default='')
    w.add_argument('--timeout-min', type=float, default=120.0)
    g = sub.add_parser('guard', help='(internal) clean up after the simulator ends')
    g.add_argument('--session', required=True)
    m = sub.add_parser('mp-steps', help='print the Mission Planner steps')
    m.add_argument('--mission', default='')
    m.add_argument('--windows-gcs', default='')
    sub.add_parser('plan', help='print the steps of a start over (changes nothing)')
    args = parser.parse_args(argv)

    if args.cmd == 'reset':
        opts = Options(fly=args.fly, world=args.world, mission=args.mission, land=args.land,
                       team_stack=not args.no_team_stack, detector=args.detector,
                       watch_timeout_min=args.watch_timeout_min)
        opts.timeouts.land = args.land_timeout
        return Resetter(opts).run()
    if args.cmd == 'watch-run':
        photo = args.photo_dir
        if not photo:
            try:
                photo = load_sim_state(sim_home()).photo_dir
            except ResetError:
                photo = ''
        return watch_run(Path(args.run_dir), photo, args.timeout_min)
    if args.cmd == 'guard':
        return guard(args.session)
    if args.cmd == 'mp-steps':
        mission, gcs = args.mission, args.windows_gcs
        if not mission or not gcs:
            try:
                st = load_sim_state(sim_home())
                mission, gcs = mission or st.mission, gcs or st.windows_gcs
            except ResetError:
                pass
        print('\n'.join(mp_steps(mission, gcs)))
        return 0
    if args.cmd == 'plan':
        for n, (step, title) in enumerate(STEPS, start=1):
            print(f'{n:2d}. {step:9s} {title}{"   (only with --fly)" if step == "fly" else ""}')
        return 0
    return 2


if __name__ == '__main__':
    sys.exit(main())
