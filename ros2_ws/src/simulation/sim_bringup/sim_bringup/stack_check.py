"""
sim_bringup/stack_check.py -- PURPOSE: wait until the team's flight stack (./launcher.sh sim) is ready to fly the mission.

"Ready" means, checked again every second:
  1. MAVROS is connected to the autopilot          (/mavros/state connected == true)
  2. the team's nodes are running: waypoint_manager, main_controller, siyi (video_cam), mission_camera_trigger,
     new_od and gps_mavros_service_node (names from main/launch/tracker.launch.xml)
  3. waypoint_manager has read the mission         (its parameter buffer_wp >= 0, so the photo loop will stop
                                                    and main_controller knows where to insert target waypoints)
  4. optional (--need-detector): new_od has loaded its model (/sahi/get_health says "ModelLoaded: True")
  5. optional (--settle S): S seconds later the team nodes are all STILL running (the team's main_controller
     can die ~10 s after start: main_controller.py fetch_mission_indices does not handle a parameter read that
     timed out)
simulation/scripts/demo.sh runs this before arming, so the drone never takes off while the team code is
still starting (main_controller blocks on its services for a while at start). waypoint_manager's parameters
are read at most once per second (its parameter service is shared with the team nodes).

Usage:
  ros2 run sim_bringup stack_check [--timeout 300] [--need-detector] [--detector-timeout 240] [--settle 20]
                                   [--skip-node NAME]... [--add-node NAME]...
  --skip-node / --add-node: a plug-in profile (sim_plugins) replaced a team node, e.g. a custom detector:
  --skip-node new_od --add-node example_detector (wait for the plug-in's node instead of the team's).
  --no-mission: skip check 3, for demo.sh --manual (Mission Planner writes the mission after the stack is up).
Exit codes: 0 = ready, 1 = not ready in time or a node stopped during --settle (it prints what is missing).
"""

import argparse
import sys
import time
from typing import Dict, List, Optional

#: Node names started by the team's tracker.launch.xml (what `ros2 node list` shows).
TEAM_NODES = ['waypoint_manager', 'main_controller', 'siyi', 'mission_camera_trigger', 'new_od',
              'gps_mavros_service_node']


def parse_health(message: str) -> Dict[str, str]:
    """Parse new_od's health text "Healthy: True, ConsecErrors: 0, ModelLoaded: True, Format: pytorch" to a dict."""
    out = {}
    for part in message.split(','):
        if ':' in part:
            key, value = part.split(':', 1)
            out[key.strip()] = value.strip()
    return out


def missing_nodes(running: List[str], wanted: List[str]) -> List[str]:
    """Return the wanted node names that are not in `running` (names compared without leading '/')."""
    have = {n.lstrip('/').split('/')[-1] for n in running}
    return [w for w in wanted if w not in have]


def wanted_nodes(skip: List[str], add: List[str]) -> List[str]:
    """TEAM_NODES without the --skip-node names, plus the --add-node names (a plug-in profile replaced some).

    With neither option this is exactly TEAM_NODES (the check is unchanged).
    """
    out = [n for n in TEAM_NODES if n not in set(skip)]
    out += [n for n in add if n not in out]
    return out


def main(argv=None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = argparse.ArgumentParser(prog='stack_check', description=__doc__.split('\n\n')[0])
    parser.add_argument('--timeout', type=float, default=300.0, help='seconds to wait for checks 1-3')
    parser.add_argument('--need-detector', action='store_true', help='also wait for new_od\'s model (check 4)')
    parser.add_argument('--detector-timeout', type=float, default=240.0,
                        help='seconds to wait for the model after checks 1-3 passed (it is a warning only)')
    parser.add_argument('--settle', type=float, default=0.0,
                        help='after everything is ready, wait this long and check the team nodes are STILL running '
                             '(the team main_controller can die ~10 s after a parameter update; exit 1 if one is gone)')
    parser.add_argument('--skip-node', action='append', default=[], metavar='NAME',
                        help='do not wait for this team node (a plug-in profile replaced it); repeatable')
    parser.add_argument('--add-node', action='append', default=[], metavar='NAME',
                        help='also wait for this node (the plug-in that replaced a team node); repeatable')
    parser.add_argument('--no-mission', action='store_true',
                        help='skip check 3: Mission Planner uploads the mission LATER (demo.sh --manual); '
                             'waypoint_manager re-reads it when ArduPilot says "Flight plan received"')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    import rclpy
    from mavros_msgs.msg import State
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    rclpy.init(args=ros_args)
    node = Node('stack_check')
    nodes_wanted = wanted_nodes(args.skip_node, args.add_node)
    status = {'connected': False, 'buffer_wp': None, 'health': None}
    node.create_subscription(State, '/mavros/state', lambda m: status.update(connected=bool(m.connected)), 10)
    params = node.create_client(GetParameters, '/waypoint_manager/get_parameters')
    health = node.create_client(Trigger, '/sahi/get_health')
    pending = {'params': None, 'health': None}
    last_request = {'params': -10.0, 'health': -10.0}

    def drop_if_unanswered(name, client):
        # A service answer can get lost while the team stack starts (seen with /sahi/get_health and with
        # waypoint_manager's parameters): after 3 s without an answer, forget the request and ask again,
        # instead of waiting for it forever.
        if pending[name] is not None and not pending[name].done() and \
                time.monotonic() - last_request[name] > 3.0:
            if hasattr(client, 'remove_pending_request'):
                client.remove_pending_request(pending[name])
            pending[name] = None

    def poll_services():
        drop_if_unanswered('params', params)
        drop_if_unanswered('health', health)
        # At most one parameter read per second: waypoint_manager answers every node's parameter reads on one
        # single-threaded service, and the team's main_controller gives up (and dies) when its own read takes
        # longer than 10 s, so this check must not keep that service busy.
        if params.service_is_ready() and pending['params'] is None and \
                time.monotonic() - last_request['params'] >= 1.0:
            last_request['params'] = time.monotonic()
            pending['params'] = params.call_async(GetParameters.Request(names=['buffer_wp', 'rtl_index']))
        if pending['params'] is not None and pending['params'].done():
            try:
                values = pending['params'].result().values
                status['buffer_wp'] = values[0].integer_value if values and values[0].type == 2 else None
            except Exception:  # noqa: BLE001 - the node may be restarting
                status['buffer_wp'] = None
            pending['params'] = None
        if health.service_is_ready() and pending['health'] is None and \
                time.monotonic() - last_request['health'] >= 1.0:
            last_request['health'] = time.monotonic()
            pending['health'] = health.call_async(Trigger.Request())
        if pending['health'] is not None and pending['health'].done():
            try:
                status['health'] = parse_health(pending['health'].result().message)
            except Exception:  # noqa: BLE001
                status['health'] = None
            pending['health'] = None

    def what_is_missing() -> List[str]:
        names = [n for n, _ in node.get_node_names_and_namespaces()]
        missing = []
        if not status['connected']:
            missing.append('MAVROS connection to the autopilot')
        missing += [f'node {n}' for n in missing_nodes(names, nodes_wanted)]
        if not args.no_mission and (status['buffer_wp'] is None or status['buffer_wp'] < 0):
            missing.append('waypoint_manager mission indices (buffer_wp)')
        return missing

    def say(text):
        print(f'[stack_check] {text}', flush=True)

    start, last_print = time.monotonic(), -100.0
    ok = False
    missing: List[str] = []
    while rclpy.ok() and time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        poll_services()
        missing = what_is_missing()
        if not missing:
            ok = True
            break
        elapsed = time.monotonic() - start
        if elapsed - last_print >= 10.0:
            say(f'waiting for: {", ".join(missing)} ({elapsed:.0f} s)')
            last_print = elapsed
    if not ok:
        say(f'NOT READY after {args.timeout:.0f} s; missing: {", ".join(missing)}')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    say(f'team stack ready after {time.monotonic() - start:.0f} s (buffer_wp = {status["buffer_wp"]})')

    if args.need_detector:
        t0, last_print = time.monotonic(), -100.0
        loaded: Optional[str] = None
        while rclpy.ok() and time.monotonic() - t0 < args.detector_timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            poll_services()
            loaded = (status['health'] or {}).get('ModelLoaded')
            if loaded == 'True':
                say(f'detector model loaded ({(status["health"] or {}).get("Format", "?")})')
                break
            if time.monotonic() - t0 - last_print >= 15.0:
                say('waiting for new_od to load its model (first start can take a minute) ...')
                last_print = time.monotonic() - t0
        else:
            say('WARNING: new_od has not loaded a model; the real detector will not find targets '
                '(check the team stack log, or use --detector truth)')
    code = 0
    if args.settle > 0:
        # 5. still alive after a while: a node that crashed right after start (seen with main_controller when a
        # parameter read times out) would otherwise only be noticed in the air.
        say(f'checking that the team nodes keep running for {args.settle:.0f} s ...')
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < args.settle:
            rclpy.spin_once(node, timeout_sec=0.5)
        gone = missing_nodes([n for n, _ in node.get_node_names_and_namespaces()], nodes_wanted)
        if gone:
            say(f'NOT READY: {", ".join(gone)} stopped during the {args.settle:.0f} s check (see the team stack log)')
            code = 1
        else:
            say('all team nodes are still running')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
