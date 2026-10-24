"""
Robustness test: the REAL main_controller node survives a mission-index read that fails once.

test_controller_robustness.py -- PURPOSE: Task 2 (a) of docs/HANDOFF.md, at unit-test size. A real
MainController node runs in its own ROS domain (it can never see a simulator or a drone) next to
fake MAVROS services (/mavros/set_mode, /mavros/cmd/command, /addWaypoint, all answering "ok")
and a fake waypoint_manager whose /waypoint_manager/get_parameters we control. We make ONE read
time out (the fake answers only after 6.5 s; the reader gives up after 5 s) and ONE read fail (an
incomplete answer), and check that main_controller:
  * keeps running (its executor thread never sees an exception),
  * keeps the OLD mission indices while the read is failing,
  * logs the failure and retries by itself, and picks up the NEW indices on the next good read.
This is the situation that killed the old main_controller (a blocking read inside a callback
returned None after a timeout). The full-size version with a frozen waypoint_manager in a flying
simulator is recorded in docs/HANDOFF.md (Task 2 evidence).

Needs ROS 2 sourced (rclpy, mavros_msgs, interfaces, cv_bridge, wp_sender); otherwise skipped:
    source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh
    python3 -m pytest ros2_ws/src/main/test/test_controller_robustness.py -v      (~20 s)
"""

import os
import sys
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS 2 (rclpy) is not sourced')
pytest.importorskip('interfaces.srv', reason='the interfaces package is not built/sourced')
pytest.importorskip('mavros_msgs.srv', reason='mavros_msgs is not installed')
pytest.importorskip('cv_bridge', reason='cv_bridge is not installed')
pytest.importorskip('wp_sender.parameter', reason='wp_sender is not built/sourced')

from interfaces.srv import AddWaypoint  # noqa: E402
from mavros_msgs.srv import CommandLong, SetMode  # noqa: E402
from rcl_interfaces.msg import (  # noqa: E402
    Parameter, ParameterEvent, ParameterType, ParameterValue)
from rcl_interfaces.srv import GetParameters  # noqa: E402
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
import main_controller as mc  # noqa: E402

OLD = {'num_waypoints': 12, 'takeoff_index': 1, 'rtl_index': 11, 'next_after_takeoff': 2,
       'last_before_rtl': 10}
NEW = {'num_waypoints': 14, 'takeoff_index': 1, 'rtl_index': 13, 'next_after_takeoff': 2,
       'last_before_rtl': 12}


class FakeLogger:
    """Collect log lines (thread-safe enough: list.append is atomic in CPython)."""

    def __init__(self):
        self.lines = []

    def info(self, text):
        self.lines.append(('info', text))

    def warn(self, text):
        self.lines.append(('warn', text))

    warning = warn

    def error(self, text):
        self.lines.append(('error', text))

    def text(self, level=None):
        return '\n'.join(t for lv, t in list(self.lines) if level in (None, lv))


class FakeWaypointManager(Node):
    """Serves /waypoint_manager/get_parameters with values and failures the test chooses."""

    def __init__(self, context):
        super().__init__('waypoint_manager', context=context, start_parameter_services=False)
        self.values = dict(OLD)
        self.stall_next_s = 0.0          # answer the next request only after this many seconds
        self.incomplete_next = False     # answer the next request with no values at all
        self.requests = 0
        self.create_service(GetParameters, '/waypoint_manager/get_parameters', self._get)
        self.events = self.create_publisher(ParameterEvent, '/parameter_events', 10)

    def _get(self, request, response):
        self.requests += 1
        if self.stall_next_s:
            stall, self.stall_next_s = self.stall_next_s, 0.0
            time.sleep(stall)            # OK here: this fake runs on a multi-threaded executor
        if self.incomplete_next:
            self.incomplete_next = False
            return response
        for name in request.names:
            response.values.append(ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                                  integer_value=int(self.values[name])))
        return response

    def announce_change(self):
        """Publish what waypoint_manager publishes when an index changes (main_controller re-reads)."""
        msg = ParameterEvent(node='/waypoint_manager', changed_parameters=[Parameter(
            name='rtl_index', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                                                   integer_value=int(self.values['rtl_index'])))])
        msg.stamp = self.get_clock().now().to_msg()
        self.events.publish(msg)


class FakeMavros(Node):
    """The three services main_controller waits for at start-up, all answering 'ok'."""

    def __init__(self, context):
        super().__init__('fake_mavros', context=context)
        self.create_service(SetMode, '/mavros/set_mode', self._mode)
        self.create_service(CommandLong, '/mavros/cmd/command', self._cmd)
        self.create_service(AddWaypoint, '/addWaypoint', self._add)

    @staticmethod
    def _mode(_req, res):
        res.mode_sent = True
        return res

    @staticmethod
    def _cmd(_req, res):
        res.success = True
        return res

    @staticmethod
    def _add(_req, res):
        res.success = True
        return res


def wait_for(condition, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if condition():
            return True
        time.sleep(0.05)
    return condition()


def indices(node):
    return {'num_waypoints': node.num_waypoints, 'takeoff_index': node.takeoff_index,
            'rtl_index': node.rtl_index, 'next_after_takeoff': node.next_after_takeoff,
            'last_before_rtl': node.last_before_rtl}


@pytest.fixture(scope='module')
def stack():
    """Fakes + a real MainController, each on its own executor thread, in an isolated domain."""
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=200 + os.getpid() % 20)
    fake_wm, fake_mavros = FakeWaypointManager(context), FakeMavros(context)
    fakes_exec = MultiThreadedExecutor(num_threads=4, context=context)
    fakes_exec.add_node(fake_wm)
    fakes_exec.add_node(fake_mavros)
    threading.Thread(target=fakes_exec.spin, daemon=True).start()

    # MainController() builds its Node on the default context: give it ours instead.
    real_init = Node.__init__
    mc.Node.__init__ = lambda self, name, **kw: real_init(self, name, context=context, **kw)
    try:
        controller = mc.MainController()
    finally:
        mc.Node.__init__ = real_init
    log = FakeLogger()
    controller.get_logger = lambda: log
    errors = []
    ctrl_exec = SingleThreadedExecutor(context=context)
    ctrl_exec.add_node(controller)

    def spin():
        try:
            ctrl_exec.spin()
        except Exception as exc:  # noqa: BLE001 - recorded: the test fails on any
            errors.append(exc)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        yield fake_wm, controller, log, errors, thread
    finally:
        ctrl_exec.shutdown()
        fakes_exec.shutdown()
        controller.destroy_node()
        fake_wm.destroy_node()
        fake_mavros.destroy_node()
        context.shutdown()


def test_startup_read_delivers_the_indices(stack):
    fake_wm, controller, log, errors, thread = stack
    assert wait_for(lambda: indices(controller) == OLD, 20.0), log.text()


def test_a_read_that_times_out_once_keeps_the_old_indices(stack):
    fake_wm, controller, log, errors, thread = stack
    assert wait_for(lambda: indices(controller) == OLD, 20.0)
    fake_wm.values = dict(NEW)
    fake_wm.stall_next_s = 6.5            # longer than the reader's 5 s deadline
    fake_wm.announce_change()
    assert wait_for(lambda: 'no answer within 5 s' in log.text('warn'), 12.0), log.text()
    assert indices(controller) == OLD      # failed read: the old values stay
    assert thread.is_alive() and errors == []
    # The reader retries by itself; the next answer is delivered.
    assert wait_for(lambda: indices(controller) == NEW, 20.0), log.text()
    assert 'again after 1 failed attempt' in log.text('info')
    assert thread.is_alive() and errors == []


def test_an_incomplete_answer_once_keeps_the_old_indices(stack):
    fake_wm, controller, log, errors, thread = stack
    assert wait_for(lambda: indices(controller) == NEW, 20.0)
    before = len(log.lines)
    fake_wm.values = dict(OLD)
    fake_wm.incomplete_next = True
    fake_wm.announce_change()
    assert wait_for(lambda: 'not set (yet)' in '\n'.join(t for _, t in log.lines[before:]),
                    10.0), log.text()
    assert indices(controller) == NEW
    assert wait_for(lambda: indices(controller) == OLD, 15.0), log.text()
    assert thread.is_alive() and errors == []
