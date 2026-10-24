"""
Node-level tests of waypoint_manager's mission re-read (waypoint_mavros/waypoint.py).

test_waypoint_node.py -- PURPOSE: drive the REAL WaypointManager node (in its own ROS domain, never
spun, so it cannot touch a running simulator or drone) with a fake /mavros/mission/pull client and
a clock the test moves by hand, and check the Task 2 review fix: a mission pull whose answer never
arrives is abandoned after the deadline, the last good mission copy and its index parameters stay
exactly as they were, the reason is logged, and the read is sent again. Before the fix the node
waited for that answer for ever and never sent another pull (not even when the drone was armed).
Also checks that a target waypoint with a NaN / infinite / (0, 0) position is never inserted or
uploaded (the last line of defence after main_controller's own checks).

Needs ROS 2 sourced (rclpy, mavros_msgs, interfaces); otherwise the tests are skipped:
    source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh
    python3 -m pytest ros2_ws/src/waypoint_mavros/test/test_waypoint_node.py -v
"""

import os
import sys

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS 2 (rclpy) is not sourced')
pytest.importorskip('mavros_msgs.msg', reason='mavros_msgs is not installed')
pytest.importorskip('interfaces.srv', reason='the interfaces package is not built/sourced')

from mavros_msgs.msg import Waypoint, WaypointList  # noqa: E402
from rclpy.task import Future  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import waypoint_mavros.waypoint as wpmod  # noqa: E402

TAKEOFF, RTL, WP, DIGICAM, SPEED = 22, 20, 16, 203, 178
# The simulator's runway survey mission: 0 home, 1 TAKEOFF, ..., 11 RTL.
RUNWAY = [WP, TAKEOFF, WP, DIGICAM, SPEED, WP, WP, WP, WP, WP, WP, RTL]


class FakeLogger:
    """Collect log lines as (level, text)."""

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    warning = warn

    def error(self, text):
        self.lines.append(('error', text))

    def fatal(self, text):
        self.lines.append(('fatal', text))

    def text(self, level=None):
        return '\n'.join(t for lv, t in self.lines if level in (None, lv))


class HungPullClient:
    """A /mavros/mission/pull client whose requests are never answered (MAVROS hung)."""

    def __init__(self):
        self.sent = []          # one rclpy Future per call_async
        self.removed = []

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        future = Future()
        self.sent.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class Clock:
    """A monotonic clock the test moves by hand."""

    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


@pytest.fixture
def manager(monkeypatch):
    """A WaypointManager in an isolated ROS domain with a hung pull client and a hand clock."""
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=150 + os.getpid() % 30)
    clock = Clock()
    monkeypatch.setattr(wpmod.time, 'monotonic', clock)
    # WaypointManager() builds its Node on the default context: give it ours instead.
    real_init = wpmod.Node.__init__
    monkeypatch.setattr(wpmod.Node, '__init__',
                        lambda self, name, **kw: real_init(self, name, context=context, **kw))
    node = wpmod.WaypointManager()
    log = FakeLogger()
    node.get_logger = lambda: log
    node.waypoint_pull = HungPullClient()
    try:
        yield node, clock, log
    finally:
        node.destroy_node()
        context.shutdown()


def mission(commands):
    """A WaypointList with one item per MAVLink command id."""
    msg = WaypointList()
    msg.waypoints = [Waypoint(command=c) for c in commands]
    return msg


def params(node):
    return {name: node.get_parameter(name).value for name in wpmod.INDEX_NAMES}


def test_a_pull_that_never_answers_times_out_and_keeps_the_last_mission(manager):
    node, clock, log = manager
    node.waypoints_list(mission(RUNWAY))               # the last good mission copy
    good = params(node)
    assert good['rtl_index'] == 11 and good['buffer_wp'] == 9

    node.request_mission_pull('a test upload')
    clock.t += node.repull.delay_s
    node._repull_tick()
    client = node.waypoint_pull
    assert len(client.sent) == 1                        # the pull went out ...

    clock.t += node.repull.pull_timeout_s - 1.0
    node._repull_tick()
    assert len(client.sent) == 1 and not client.sent[0].cancelled()   # ... still allowed to run

    clock.t += 1.0
    node._repull_tick()                                 # ... and now it is abandoned
    assert client.sent[0].cancelled()
    assert client.removed == [client.sent[0]]
    assert params(node) == good                         # indices untouched
    assert len(node.waypoint_list.waypoints) == len(RUNWAY)
    warn = log.text('warn')
    assert 'did not answer the mission re-read' in warn and 'Keeping the last mission' in warn
    assert 'rtl_index=11' in warn and 'trying again' in warn

    clock.t += node.repull.retry_delay_s
    node._repull_tick()
    assert len(client.sent) == 2                        # retried (never blocked for ever)


def test_a_late_answer_to_an_abandoned_pull_is_ignored(manager):
    node, clock, log = manager
    node.waypoints_list(mission(RUNWAY))
    node.request_mission_pull('a test upload')
    clock.t += node.repull.delay_s
    node._repull_tick()
    clock.t += node.repull.pull_timeout_s
    node._repull_tick()                                 # abandoned
    first = node.waypoint_pull.sent[0]
    attempts = node.repull.attempts
    node._on_pull_done(first)                           # its answer finally turns up
    assert node.repull.attempts == attempts             # not counted twice
    assert 'Re-read the mission' not in log.text()


def test_arming_is_served_after_a_hung_pull_gives_up(manager):
    node, clock, log = manager
    node.waypoints_list(mission(RUNWAY))
    node.repull.max_attempts = 1
    node.request_mission_pull('connected to the autopilot (start-up)')
    clock.t += node.repull.delay_s
    node._repull_tick()
    node.request_mission_pull('the drone was armed', armed=True)   # waits behind the hung pull
    clock.t += node.repull.pull_timeout_s
    node._repull_tick()                                 # gives up on the first one
    assert 'giving up for now' in log.text('warn')
    clock.t += node.repull.delay_s
    node._repull_tick()
    assert len(node.waypoint_pull.sent) == 2            # the arming re-read still goes out


@pytest.mark.parametrize('lat, lon', [(float('nan'), -96.0), (36.2, float('inf')), (0.0, 0.0)])
def test_a_nonfinite_target_waypoint_is_never_uploaded(manager, lat, lon):
    node, clock, log = manager
    node.waypoints_list(mission(RUNWAY))
    pushes = []
    node.push_waypoints = lambda: pushes.append(True)
    ok = node.insert_new_waypoint([{'command': 16, 'lat': lat, 'lon': lon, 'alt': 49.0,
                                    'index': 11}])
    assert ok is False and pushes == []
    assert len(node.waypoint_list.waypoints) == len(RUNWAY)
    assert 'Refusing NAV_WAYPOINT' in log.text('error')


def test_a_good_target_waypoint_is_still_inserted(manager):
    node, clock, log = manager
    node.waypoints_list(mission(RUNWAY))
    pushes = []
    node.push_waypoints = lambda: pushes.append(True)
    assert node.insert_new_waypoint([{'command': 16, 'lat': 36.215, 'lon': -96.0068,
                                      'alt': 49.0, 'index': 11}]) is True
    assert pushes == [True] and len(node.waypoint_list.waypoints) == len(RUNWAY) + 1
    assert node.waypoint_list.waypoints[11].x_lat == 36.215
