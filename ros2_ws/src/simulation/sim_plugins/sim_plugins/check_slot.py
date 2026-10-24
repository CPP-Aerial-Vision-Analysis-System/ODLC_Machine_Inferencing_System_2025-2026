"""
sim_plugins/check_slot.py -- PURPOSE: the CONFORMANCE CHECKER. Looks at the LIVE ROS graph (and, for the camera,
the network) and says, in plain words, whether the component that fills a slot keeps that slot's contract
(contracts.py): the topics it must publish / subscribe with the right message types and QoS, the services
it must serve or call, and, when a sample message arrives, whether its fields make sense.

    ros2 run sim_plugins check_slot --slot detector                  # the running detector (new_od or yours)
    ros2 run sim_plugins check_slot --slot detector --node my_det --take-photo --timeout 90
    ros2 run sim_plugins check_slot --slot mission_brain
    ros2 run sim_plugins check_slot --slot camera  [--camera-ip 192.168.144.25]
    ros2 run sim_plugins check_slot --slot autopilot | world
    ros2 run sim_plugins check_slot --slot extras --node my_logger
Options: --node NAME (which node is the component; default: found by itself), --timeout S (default 60: how
long to wait for the graph and for a sample), --no-sample, --take-photo (detector: press the team's shutter
/camera/trigger once so a photo, and so a detection message, happens), --photo-dir DIR.
Exit code: 0 = keeps the contract (warnings allowed), 1 = something is wrong, 2 = bad usage.

How it works: the checks are plain functions over a `Graph` (what nodes, topics, publishers with their QoS,
subscribers, services and service clients exist). RosGraph reads them from rclpy; the unit tests give a
FakeGraph instead, so every rule is tested without ROS. Sample messages are checked by plain functions too
(check_image_result, check_mavros_state, check_odometry) that only read attributes.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence

from . import contracts

#: Nodes that are tools, not components (never chosen as "the component").
TOOL_NODES = ('check_slot', 'stack_check', 'phase_monitor', 'demo_panel', 'demo_markers', 'score_run')

#: How the component of a slot is recognised when --node is not given and the team's node is not running:
#: (what the component does, the name). 'publish' = publishes that topic, 'client' = calls that service.
PRIMARY = {
    'detector': ('publish', '/image_detection'),
    'mission_brain': ('client', '/addWaypoint'),
    'camera': ('publish', '/sim/camera/photo_info'),
}

#: Topics other slots own: an "extra" should not publish on them (unless that is the point of the test).
OWNED_TOPICS = ('/image_detection', '/camera/trigger', '/camera/command', '/mavros/setpoint_raw/local',
                '/mavros/setpoint_position/local', '/mavros/setpoint_velocity/cmd_vel')

#: Detector class ids main_controller understands.
CLASS_NAMES = {'0': 'person', '1': 'tent'}


# ---------------------------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------------------------

@dataclass
class Finding:
    """One line of the result. level: 'ok', 'fail' (breaks the contract), 'warn' (works, but check it), 'info'."""
    level: str
    text: str
    hint: str = ''

    def render(self) -> str:
        mark = {'ok': ' OK ', 'fail': ' XX ', 'warn': ' !! ', 'info': ' .. '}[self.level]
        return mark + self.text + (f'\n      -> {self.hint}' if self.hint else '')


@dataclass
class Report:
    """Everything check_slot found for one slot."""
    slot: str
    node: str = ''
    findings: List[Finding] = field(default_factory=list)

    def add(self, level: str, text: str, hint: str = '') -> None:
        self.findings.append(Finding(level, text, hint))

    @property
    def failed(self) -> bool:
        return any(f.level == 'fail' for f in self.findings)

    def counts(self) -> Dict[str, int]:
        out = {'ok': 0, 'fail': 0, 'warn': 0, 'info': 0}
        for f in self.findings:
            out[f.level] += 1
        return out

    def render(self) -> str:
        head = f'check_slot: {self.slot}' + (f' (component node: {self.node})' if self.node else '')
        c = self.counts()
        verdict = (f'RESULT: FAIL ({c["fail"]} problem(s), {c["warn"]} warning(s))' if self.failed else
                   f'RESULT: PASS ({c["ok"]} check(s) passed, {c["warn"]} warning(s))')
        return '\n'.join([head] + [f.render() for f in self.findings] + [verdict])


# ---------------------------------------------------------------------------------------------
# The graph (what ROS knows right now)
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Endpoint:
    """A publisher or subscriber: which node, which message type, its reliability ('reliable'/'best_effort')."""
    node: str
    type: str
    reliability: str = 'reliable'


class Graph:
    """What the checks need to know. RosGraph (rclpy) and the tests' FakeGraph implement it."""

    def node_names(self) -> List[str]:                     # plain node names (no namespace)
        raise NotImplementedError

    def publishers(self, topic: str) -> List[Endpoint]:
        raise NotImplementedError

    def subscribers(self, topic: str) -> List[Endpoint]:
        raise NotImplementedError

    def service_types(self) -> Dict[str, List[str]]:       # every service in the graph -> its type(s)
        raise NotImplementedError

    def node_servers(self, node: str) -> Dict[str, List[str]]:
        raise NotImplementedError

    def node_clients(self, node: str) -> Dict[str, List[str]]:
        raise NotImplementedError

    def node_publications(self, node: str) -> Dict[str, List[str]]:
        raise NotImplementedError


class FakeGraph(Graph):
    """A graph made of plain dicts (unit tests, and examples in the docs).

    nodes        ['new_od', ...]
    pubs / subs  {'/topic': [Endpoint, ...]}
    servers      {'node': {'/service': ['pkg/srv/Type']}}
    clients      {'node': {'/service': ['pkg/srv/Type']}}
    """

    def __init__(self, nodes=(), pubs=None, subs=None, servers=None, clients=None):
        self.nodes = list(nodes)
        self.pubs = dict(pubs or {})
        self.subs = dict(subs or {})
        self.servers = dict(servers or {})
        self.clients = dict(clients or {})

    def node_names(self):
        return list(self.nodes)

    def publishers(self, topic):
        return list(self.pubs.get(topic, []))

    def subscribers(self, topic):
        return list(self.subs.get(topic, []))

    def service_types(self):
        out: Dict[str, List[str]] = {}
        for services in self.servers.values():
            for name, types in services.items():
                out.setdefault(name, list(types))
        return out

    def node_servers(self, node):
        return dict(self.servers.get(node, {}))

    def node_clients(self, node):
        return dict(self.clients.get(node, {}))

    def node_publications(self, node):
        out: Dict[str, List[str]] = {}
        for topic, eps in self.pubs.items():
            for ep in eps:
                if ep.node == node:
                    out.setdefault(topic, []).append(ep.type)
        return out


class RosGraph(Graph):
    """The live ROS graph, read through an rclpy node."""

    def __init__(self, node):
        self.node = node

    def _namespaces(self) -> Dict[str, str]:
        return {name: ns for name, ns in self.node.get_node_names_and_namespaces()}

    def node_names(self):
        return sorted({name for name, _ in self.node.get_node_names_and_namespaces()
                       if not name.startswith('_')})

    @staticmethod
    def _reliability(qos) -> str:
        text = str(getattr(qos, 'reliability', '')).lower()
        return 'best_effort' if 'best_effort' in text else ('reliable' if 'reliable' in text else 'unknown')

    def publishers(self, topic):
        return [Endpoint(i.node_name, i.topic_type, self._reliability(i.qos_profile))
                for i in self.node.get_publishers_info_by_topic(topic)]

    def subscribers(self, topic):
        return [Endpoint(i.node_name, i.topic_type, self._reliability(i.qos_profile))
                for i in self.node.get_subscriptions_info_by_topic(topic)]

    def service_types(self):
        return {name: list(types) for name, types in self.node.get_service_names_and_types()}

    def _by_node(self, getter, node):
        ns = self._namespaces().get(node)
        if ns is None:
            return {}
        try:
            return {name: list(types) for name, types in getter(node, ns)}
        except Exception:  # noqa: BLE001 - the node may have just gone away
            return {}

    def node_servers(self, node):
        return self._by_node(self.node.get_service_names_and_types_by_node, node)

    def node_clients(self, node):
        return self._by_node(self.node.get_client_names_and_types_by_node, node)

    def node_publications(self, node):
        return self._by_node(self.node.get_publisher_names_and_types_by_node, node)


# ---------------------------------------------------------------------------------------------
# Finding the component
# ---------------------------------------------------------------------------------------------

def find_component(graph: Graph, slot: str, wanted: str = '') -> List[str]:
    """Node name(s) that fill the slot: --node, else the team's default node, else by the slot's primary role."""
    nodes = [n for n in graph.node_names() if n not in TOOL_NODES]
    if wanted:
        return [wanted] if wanted in nodes else []
    c = contracts.contract(slot)
    defaults = [n for n in c.node_names if n in nodes]
    role, name = PRIMARY.get(slot, ('', ''))
    found: List[str] = []
    if role == 'publish':
        found = [ep.node for ep in graph.publishers(name) if ep.node not in TOOL_NODES]
    elif role == 'client':
        found = [n for n in nodes if name in graph.node_clients(n)]
    out = []
    for n in defaults + found:            # the team's node first, then anything else in that role
        if n not in out:
            out.append(n)
    return out


# ---------------------------------------------------------------------------------------------
# Contract checks on the graph
# ---------------------------------------------------------------------------------------------

def check_topics(graph: Graph, c: contracts.SlotContract, node: str, report: Report) -> None:
    """Check every topic rule of a contract. node '' = any node may provide it (autopilot / world)."""
    for rule in c.topics:
        eps = graph.publishers(rule.name) if rule.role == 'publish' else graph.subscribers(rule.name)
        mine = [ep for ep in eps if not node or ep.node == node]
        verb = 'publish' if rule.role == 'publish' else 'subscribe to'
        who = node or 'any node'
        if not mine:
            level = 'fail' if rule.required else 'info'
            text = f'{rule.name}: {who} does not {verb} it' + ('' if rule.required else ' (optional)')
            hint = rule.note if rule.required else ''
            if rule.required and eps:
                hint = (f'it is {verb.split()[0]}ed by {", ".join(sorted({e.node for e in eps}))} instead; '
                        + rule.note)
            report.add(level, text, hint)
            continue
        for ep in mine:
            if ep.type != rule.type:
                report.add('fail', f'{rule.name}: {ep.node} uses type {ep.type}', f'it must be {rule.type}')
                continue
            if rule.role == 'publish' and rule.qos == 'reliable' and ep.reliability == 'best_effort':
                report.add('fail', f'{rule.name}: {ep.node} publishes BEST_EFFORT',
                           'the team\'s subscriber is RELIABLE and never receives a BEST_EFFORT publisher: use '
                           'QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)')
                continue
            qos = f', {ep.reliability.upper()}' if ep.reliability in ('reliable', 'best_effort') else ''
            report.add('ok', f'{rule.name}: {ep.node} {"publishes" if rule.role == "publish" else "subscribes"} '
                             f'{ep.type}{qos}')
        if rule.role == 'publish' and rule.required and node:
            others = sorted({ep.node for ep in eps if ep.node != node and ep.node not in TOOL_NODES})
            if others:
                report.add('warn', f'{rule.name}: also published by {", ".join(others)}',
                           'run ONE component per slot (main_controller cannot tell two detectors apart); in '
                           'detector:=truth mode an idle new_od is normal')


def check_services(graph: Graph, c: contracts.SlotContract, node: str, report: Report) -> None:
    """Check every service rule: 'server' = the component serves it, 'client' = the component calls it."""
    all_services = graph.service_types()
    for rule in c.services:
        if rule.role == 'server':
            have = graph.node_servers(node) if node else {k: v for k, v in all_services.items()}
        else:
            if not node:
                report.add('warn', f'{rule.name}: cannot check the client side without a component node',
                           'give --node NAME')
                continue
            have = graph.node_clients(node)
        types = have.get(rule.name)
        verb = 'serves' if rule.role == 'server' else 'calls'
        if not types:
            level = 'fail' if rule.required else 'info'
            hint = ''
            if rule.required:
                hint = rule.note + ('. Create the client when your node starts (like main_controller), so it '
                                    'can be seen' if rule.role == 'client' else '')
            report.add(level, f'{rule.name}: {node or "nobody"} never {verb} it'
                       + ('' if rule.required else ' (optional)'), hint)
            continue
        if rule.type not in types:
            report.add('fail', f'{rule.name}: type {", ".join(types)}', f'it must be {rule.type}')
            continue
        report.add('ok', f'{rule.name}: {node or "someone"} {verb} it ({rule.type})')
        if rule.role == 'client' and rule.name not in all_services:
            report.add('info', f'{rule.name}: nobody serves it yet (MAVROS / waypoint_manager not running?)')


def check_extras(graph: Graph, node: str, report: Report) -> None:
    """An extra has no fixed contract: it must exist and should not publish on topics other slots own."""
    pubs = graph.node_publications(node)
    clash = sorted(t for t in pubs if t in OWNED_TOPICS)
    if clash:
        report.add('warn', f'{node} publishes {", ".join(clash)}', 'these topics belong to other slots; only '
                   'publish there if that is what you are testing')
    else:
        report.add('ok', f'{node} does not publish on topics other slots own')
    listed = ', '.join(sorted(pubs)) or 'nothing'
    report.add('info', f'{node} publishes: {listed}')


# ---------------------------------------------------------------------------------------------
# Sample message checks (plain attribute reads: unit-tested with SimpleNamespace fakes)
# ---------------------------------------------------------------------------------------------

def check_image_result(msg, report: Report, photo_dir: str = '', allow_truth: bool = False) -> None:
    """Sanity-check one interfaces/ImageResult message (see contracts.DETECTOR rules)."""
    name = str(getattr(msg, 'image_name', '') or '')
    if not name:
        report.add('fail', 'sample: image_name is empty', 'set it to the photo\'s file name: main_controller counts '
                   'which photos were reported')
    elif '/' in name or '\\' in name:
        report.add('fail', f'sample: image_name "{name}" contains a folder', 'only the file name, e.g. '
                   '"36.215026 , -96.006800.jpg"')
    else:
        report.add('ok', f'sample: image_name "{name}"')
        if photo_dir:
            if os.path.isfile(os.path.join(photo_dir, name)):
                report.add('ok', 'sample: that photo exists in the team photo folder')
            else:
                report.add('warn', f'sample: {name} is not in {photo_dir}', 'main_controller waits until every '
                           'photo NAME in that folder was reported; a name that is not there never counts')
    method = str(getattr(msg, 'method', '') or '')
    if not method:
        report.add('warn', 'sample: method is empty', 'put your detector\'s name there (the panel shows it)')
    elif method.startswith('truth_detector') and not allow_truth:
        report.add('fail', f'sample: method "{method}" starts with truth_detector',
                   'the demo panel would show the answer-key banner: use your own name')
    else:
        report.add('ok', f'sample: method "{method}"')
    dets = list(getattr(getattr(msg, 'detections', None), 'detections', []) or [])
    num = getattr(msg, 'num_detections', None)
    classes = list(getattr(msg, 'classes', []) or [])
    # new_od counts EVERY detection in num_detections and the parallel lists (classes ...), also the class
    # "object" it keeps OUT of detections (only persons / tents go there). So num_detections equals len(classes)
    # when the lists are filled (else the detections list), and it is never smaller than the detections list.
    expected = len(classes) if classes else len(dets)
    if num is not None and (int(num) != expected or int(num) < len(dets)):
        report.add('warn', f'sample: num_detections = {num}, but {len(dets)} person/tent detection(s) and '
                           f'{len(classes)} entries in classes',
                   'num_detections = every detection (= len(classes)); detections = only persons and tents')
    bad = []
    for i, det in enumerate(dets):
        results = list(getattr(det, 'results', []) or [])
        if not results:
            bad.append(f'detection {i} has no results[0]')
            continue
        hyp = getattr(results[0], 'hypothesis', results[0])
        cid, score = str(getattr(hyp, 'class_id', '')), float(getattr(hyp, 'score', -1.0))
        if cid not in CLASS_NAMES:
            bad.append(f'detection {i}: class_id "{cid}" (only "0" person / "1" tent count)')
        if not 0.0 <= score <= 1.0 or math.isnan(score):
            bad.append(f'detection {i}: score {score} is not 0..1')
        bbox = getattr(det, 'bbox', None)
        if bbox is not None:
            w, h = float(getattr(bbox, 'size_x', 0.0)), float(getattr(bbox, 'size_y', 0.0))
            pos = getattr(getattr(bbox, 'center', None), 'position', None)
            x, y = (float(getattr(pos, 'x', -1.0)), float(getattr(pos, 'y', -1.0))) if pos is not None else (-1, -1)
            if w <= 0 or h <= 0:
                bad.append(f'detection {i}: box size {w:g} x {h:g} px')
            if x < 0 or y < 0:
                bad.append(f'detection {i}: box centre ({x:g}, {y:g}) is outside the photo')
    if bad:
        report.add('fail', 'sample: ' + '; '.join(bad[:4]) + (' ...' if len(bad) > 4 else ''),
                   'class_id "0"/"1", score 0..1, box centre + size in pixels of the full photo')
    else:
        classes = [CLASS_NAMES.get(str(getattr(getattr(d.results[0], 'hypothesis', d.results[0]), 'class_id', '')),
                                   '?') for d in dets] if dets else []
        report.add('ok', f'sample: {len(dets)} detection(s){" (" + ", ".join(classes) + ")" if classes else ""}'
                         ' with valid class ids, scores and boxes')
    lat, lon = float(getattr(msg, 'latitude', 0.0)), float(getattr(msg, 'longitude', 0.0))
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        report.add('fail', f'sample: latitude/longitude {lat}, {lon} are not a position')
    elif lat == 0.0 and lon == 0.0:
        report.add('warn', 'sample: latitude/longitude are 0, 0', 'copy them from the photo file name '
                   '"<lat> , <lon>.jpg" (0 only when the name has none)')
    else:
        report.add('ok', f'sample: position {lat:.6f}, {lon:.6f}')


def check_mavros_state(msg, report: Report) -> None:
    """One mavros_msgs/State: is MAVROS connected to the autopilot?"""
    if bool(getattr(msg, 'connected', False)):
        report.add('ok', f'sample: MAVROS is connected to the autopilot (mode {getattr(msg, "mode", "?")}, '
                         f'armed {bool(getattr(msg, "armed", False))})')
    else:
        report.add('fail', 'sample: MAVROS runs but is NOT connected to the autopilot',
                   'check MAVProxy\'s MAVROS output (network.mavros_out / mavros_port) and launcher.sh\'s FCU_URL')


def check_odometry(msg, report: Report) -> None:
    """One nav_msgs/Odometry from Gazebo: is the drone in the world (finite position)?"""
    pos = msg.pose.pose.position
    if all(math.isfinite(float(v)) for v in (pos.x, pos.y, pos.z)):
        report.add('ok', f'sample: the drone is in the world at x={pos.x:.1f} y={pos.y:.1f} z={pos.z:.1f} m')
    else:
        report.add('fail', 'sample: the drone position is not a number')


# ---------------------------------------------------------------------------------------------
# One slot, start to end (graph part; samples are passed in)
# ---------------------------------------------------------------------------------------------

def evaluate(graph: Graph, slot: str, node: str = '', sample=None, sample_wanted: bool = False,
             photo_dir: str = '', camera_probe: Optional[Callable[[], List[Finding]]] = None) -> Report:
    """All checks of one slot on a graph (plus an optional sample message and camera probe results)."""
    report = Report(slot=slot)
    c = contracts.contract(slot)
    if slot == 'extras':
        if not node:
            report.add('fail', 'extras have no fixed node name', 'give --node NAME (your extra\'s ROS node name)')
            return report
        if node not in graph.node_names():
            report.add('fail', f'node {node} is not running', 'is it in the profile\'s extras, and did it start? '
                       '(ros2 node list)')
            return report
        report.node = node
        check_extras(graph, node, report)
        return report

    component = ''
    if slot in PRIMARY or c.node_names:
        found = find_component(graph, slot, node)
        if node and not found:
            report.add('fail', f'node {node} is not running', 'check the name with: ros2 node list')
            return report
        if not found:
            if slot == 'camera':
                report.add('info', 'no camera node in the ROS graph (a camera program without ROS is fine: '
                                   'the network checks below decide)')
            else:
                role, name = PRIMARY[slot]
                report.add('fail', f'no node {"publishes" if role == "publish" else "calls"} {name}',
                           f'is your component running? (the team default is {", ".join(c.node_names)})')
                return report
        else:
            component = found[0]
            if len(found) > 1:
                report.add('warn', f'several nodes fill this slot: {", ".join(found)}; checking {component}',
                           'pick one with --node NAME')
    report.node = component
    if component or slot not in ('camera',):
        check_topics(graph, c, component, report)
        check_services(graph, c, component, report)
    if camera_probe is not None:
        report.findings.extend(camera_probe())
    if sample_wanted:
        if sample is None:
            report.add('warn', 'no sample message arrived in time', SAMPLE_HINTS.get(slot, ''))
        elif slot == 'detector':
            # A message does not say which node sent it. In detector:=truth mode new_od runs idle next to
            # truth_detector, so a truth_detector message is expected there (and is no fault of new_od).
            from_truth = str(getattr(sample, 'method', '') or '').startswith('truth_detector')
            truth_running = 'truth_detector' in [ep.node for ep in graph.publishers('/image_detection')]
            if from_truth and truth_running and component != 'truth_detector':
                report.add('warn', f'the sample came from truth_detector (the answer key), not from {component}',
                           'detector:=truth mode: check the stand-in itself with --node truth_detector')
            check_image_result(sample, report, photo_dir, allow_truth=from_truth and truth_running)
        elif slot == 'autopilot':
            check_mavros_state(sample, report)
        elif slot == 'world':
            check_odometry(sample, report)
    return report


SAMPLE_HINTS = {
    'detector': 'a detection message comes once per photo: start the mission, press TAKE PHOTO in the demo '
                'panel, or run check_slot with --take-photo',
    'autopilot': 'is ./launcher.sh sim (MAVROS) running?',
    'world': 'is the simulator running (sim.launch.py with bridge:=true)?',
}

#: slot -> (topic, (module, class), QoS kind) of the sample message check_slot waits for.
SAMPLES = {
    'detector': ('/image_detection', ('interfaces.msg', 'ImageResult'), 'reliable'),
    'autopilot': ('/mavros/state', ('mavros_msgs.msg', 'State'), 'reliable'),
    'world': ('/sim/drone/odometry', ('nav_msgs.msg', 'Odometry'), 'sensor'),
}


def camera_findings(ip: str, timeout: float = 2.0, probe=None) -> List[Finding]:
    """The camera slot's network checks (siyi_probe), as findings."""
    from . import siyi_probe
    probe = probe or siyi_probe
    out = []
    ok, text = probe.probe_sdk(ip, timeout=timeout)
    out.append(Finding('ok' if ok else 'fail', text))
    ok, text = probe.probe_media(ip, timeout=timeout + 1.0)
    out.append(Finding('ok' if ok else 'fail', text))
    return out


# ---------------------------------------------------------------------------------------------
# Command line (the ROS part)
# ---------------------------------------------------------------------------------------------

def _default_photo_dir() -> str:
    try:
        from sim_bringup import sim_paths
        return sim_paths.team_photo_dir()
    except Exception:  # noqa: BLE001 - sim_bringup not available: skip the file check
        return ''


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog='check_slot', description=__doc__.split('\n\n')[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--slot', required=True, choices=[s for s in contracts.CONTRACTS if s != 'network'])
    parser.add_argument('--node', default='', help='the component\'s ROS node name (default: found by itself)')
    parser.add_argument('--timeout', type=float, default=60.0, help='seconds to wait for the graph and a sample')
    parser.add_argument('--no-sample', action='store_true', help='do not wait for a sample message')
    parser.add_argument('--take-photo', action='store_true',
                        help='detector: publish /camera/trigger once (the team\'s shutter) so a photo happens')
    parser.add_argument('--photo-dir', default=None, help='the team photo folder (default: found like sim_paths)')
    parser.add_argument('--camera-ip', default='192.168.144.25', help='camera: the address to probe')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = build_parser()
    args, ros_args = parser.parse_known_args(list(sys.argv[1:] if argv is None else argv))
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    except ImportError:
        print(' XX ROS 2 is not available here: source /opt/ros/humble/setup.bash and the workspace first',
              file=sys.stderr)
        return 2
    rclpy.init(args=ros_args)
    node = Node('check_slot')
    graph = RosGraph(node)
    deadline = time.monotonic() + max(args.timeout, 3.0)
    photo_dir = args.photo_dir if args.photo_dir is not None else _default_photo_dir()
    print(f'[check_slot] looking at the ROS graph for the {args.slot} slot (up to {args.timeout:.0f} s) ...',
          flush=True)

    # 1. let discovery fill the graph, then re-check until the contract's graph part passes (or time is up)
    t_min = time.monotonic() + 2.0
    report = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.25)
        if time.monotonic() < t_min:
            continue
        report = evaluate(graph, args.slot, args.node)
        if not report.failed or time.monotonic() > deadline - (0 if args.no_sample else args.timeout / 3):
            break
        time.sleep(0.5)

    # 2. the sample message (and the camera probe)
    sample = None
    want_sample = args.slot in SAMPLES and not args.no_sample and report is not None and not report.failed
    if want_sample:
        topic, (module, cls), qos_kind = SAMPLES[args.slot]
        try:
            msg_type = getattr(__import__(module, fromlist=[cls]), cls)
        except ImportError as exc:
            print(f' !! cannot import {module}.{cls} ({exc}): no sample check', flush=True)
            want_sample = False
        else:
            got = []
            qos = qos_profile_sensor_data if qos_kind == 'sensor' else QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(msg_type, topic, got.append, qos)
            if args.take_photo and args.slot == 'detector':
                from std_msgs.msg import Bool
                pub = node.create_publisher(Bool, '/camera/trigger', 10)
                for _ in range(8):                      # give discovery a moment before the one message
                    rclpy.spin_once(node, timeout_sec=0.25)
                pub.publish(Bool(data=True))
                print('[check_slot] pressed the shutter (/camera/trigger) once', flush=True)
            # A photo takes a few seconds (shutter, Gazebo render, download, detection): with --take-photo wait
            # at least 30 s for its result even when the graph part used up most of --timeout.
            sample_deadline = max(deadline, time.monotonic() + (30.0 if args.take_photo else 5.0))
            print(f'[check_slot] waiting for one message on {topic} ...', flush=True)
            while rclpy.ok() and not got and time.monotonic() < sample_deadline:
                rclpy.spin_once(node, timeout_sec=0.25)
            sample = got[0] if got else None
    probe = (lambda: camera_findings(args.camera_ip)) if args.slot == 'camera' else None
    report = evaluate(graph, args.slot, args.node, sample=sample, sample_wanted=want_sample,
                      photo_dir=photo_dir, camera_probe=probe)
    print(report.render(), flush=True)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
