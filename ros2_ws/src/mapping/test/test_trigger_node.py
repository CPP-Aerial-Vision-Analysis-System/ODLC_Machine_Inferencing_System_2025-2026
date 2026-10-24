"""
Unit tests for mapping/do_digi_cam_trigger.py (the mission_camera_trigger node).

test_trigger_node.py -- PURPOSE: test the survey photo trigger after its October 2026 change (the
photo and autofocus periods became ROS parameters). The tests start the REAL node with rclpy, but
never spin it against a simulator: they call its callbacks directly with hand-made messages and
look at the timers it creates. They check that
  * the defaults are the old constants (a photo every 1 s, an autofocus every 5 s),
  * the parameters work, with decimals AND whole numbers (`-p autofocus_period_s:=0` used to make
    the node refuse to start, so not a single photo was taken), `0` switches the autofocus off,
    a tiny photo period is raised to 0.1 s, and a value that is not a number falls back to the
    default instead of stopping the node,
  * a repeated DigiCamCtrl starts ONE photo timer and ONE autofocus timer, never more,
  * reaching waypoint buffer_wp stops both (also when nothing was started yet),
  * it reacts to waypoint_manager's parameter events and its buffer_wp answer as before,
  * it really publishes what the camera node listens for (/camera/command "capture_setup" and
    "autofocus", /camera/trigger true).

Every test uses its own ROS domain (TEST_DOMAIN_ID), so it can never reach a simulator or a
drone that runs on this computer at the same time (no photo is ever triggered by a test).

Needs ROS 2 (rclpy, mavros_msgs) sourced, otherwise the tests are skipped:
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
    python3 -m pytest ros2_ws/src/mapping/test/test_trigger_node.py -v
"""

import os
import sys
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy', reason='ROS 2 (rclpy) is not sourced')
pytest.importorskip('mavros_msgs', reason='MAVROS messages are not installed / sourced')

from mavros_msgs.msg import StatusText, WaypointReached  # noqa: E402
from rcl_interfaces.msg import Parameter, ParameterEvent, ParameterValue  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

# The package folder (ros2_ws/src/mapping) first, so the SOURCE is tested even before a build.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mapping.do_digi_cam_trigger import (  # noqa: E402
    AUTOFOCUS_PERIOD_SECONDS,
    CAPTURE_PERIOD_SECONDS,
    MissionCameraTrigger,
)

# A ROS domain nobody else uses (0 is the default of the simulator and of the drone). The process
# id spreads parallel test runs over a few domains.
TEST_DOMAIN_ID = 60 + os.getpid() % 30

DIGICAM_TEXT = 'Mission: 3 DigiCamCtrl'


@pytest.fixture
def make_node():
    """Start ROS in the test domain with the given -p name:=value settings and build the node."""
    nodes = []

    def _make(**params):
        args = ['--ros-args']
        for name, value in params.items():
            args += ['-p', f'{name}:={value}']
        rclpy.init(args=args, domain_id=TEST_DOMAIN_ID)
        node = MissionCameraTrigger()
        nodes.append(node)
        return node

    yield _make
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def status(text):
    """Build an autopilot text message as MAVROS delivers it on /mavros/statustext/recv."""
    return StatusText(severity=6, text=text)


def period_s(timer):
    """Return a timer's period in seconds."""
    return timer.timer_period_ns / 1e9


def timer_count(node):
    """Count the timers that exist on the node."""
    return len(list(node.timers))


# ------------------------------------------------------------------ parameters
def test_defaults_are_the_old_constants(make_node):
    node = make_node()
    assert CAPTURE_PERIOD_SECONDS == 1.0 and AUTOFOCUS_PERIOD_SECONDS == 5.0
    assert node.capture_period_s == 1.0 and node.autofocus_period_s == 5.0
    assert node.get_parameter('capture_period_s').value == 1.0
    node.statustext_callback(status(DIGICAM_TEXT))
    assert node.timer is not None and period_s(node.timer) == pytest.approx(1.0)
    assert node.autofocus_timer is not None
    assert period_s(node.autofocus_timer) == pytest.approx(5.0)


def test_decimal_values_set_the_periods(make_node):
    node = make_node(capture_period_s='2.5', autofocus_period_s='7.5')
    node.statustext_callback(status(DIGICAM_TEXT))
    assert period_s(node.timer) == pytest.approx(2.5)
    assert period_s(node.autofocus_timer) == pytest.approx(7.5)


def test_whole_numbers_are_accepted(make_node):
    # Regression: with a plain decimal default ROS 2 refuses `:=0` / `:=2` (INTEGER) and the node
    # did not start at all (InvalidParameterTypeException), so no photo was ever taken.
    node = make_node(capture_period_s='2', autofocus_period_s='0')
    assert node.capture_period_s == 2.0 and isinstance(node.capture_period_s, float)
    assert node.autofocus_period_s == 0.0
    node.statustext_callback(status(DIGICAM_TEXT))
    assert period_s(node.timer) == pytest.approx(2.0)
    assert node.autofocus_timer is None              # 0 = autofocus loop off


def test_autofocus_zero_switches_the_loop_off_but_photos_continue(make_node):
    node = make_node(autofocus_period_s='0.0')
    node.statustext_callback(status(DIGICAM_TEXT))
    assert node.autofocus_timer is None
    assert node.timer is not None and period_s(node.timer) == pytest.approx(1.0)


@pytest.mark.parametrize('value', ['0', '0.0', '0.01', '-3.0'])
def test_capture_period_is_at_least_a_tenth_of_a_second(make_node, value):
    node = make_node(capture_period_s=value)
    assert node.capture_period_s == pytest.approx(0.1)


def test_a_value_that_is_not_a_number_falls_back_to_the_default(make_node):
    node = make_node(capture_period_s='true', autofocus_period_s='abc')
    assert node.capture_period_s == 1.0 and node.autofocus_period_s == 5.0


# ------------------------------------------------------------------ start / stop
def test_repeated_digicamctrl_starts_one_photo_timer_and_one_autofocus_timer(make_node):
    node = make_node()
    before = timer_count(node)                       # the buffer_wp lookup timer
    node.statustext_callback(status(DIGICAM_TEXT))
    capture, autofocus = node.timer, node.autofocus_timer
    for _ in range(3):                               # ArduPilot repeats it on every pass
        node.statustext_callback(status(DIGICAM_TEXT))
    assert node.timer is capture and node.autofocus_timer is autofocus
    assert timer_count(node) == before + 2


def test_other_texts_do_not_start_photos(make_node):
    node = make_node()
    for text in ('Mission: 2 WP', 'PreArm: Gyros inconsistent', 'Flight plan received'):
        node.statustext_callback(status(text))
    assert node.timer is None and node.autofocus_timer is None


def test_reaching_buffer_wp_stops_photos_and_autofocus(make_node):
    node = make_node()
    node.buffer_wp = 9
    before = timer_count(node)
    node.statustext_callback(status(DIGICAM_TEXT))
    node.update_waypoint_reached(WaypointReached(wp_seq=8))
    assert node.timer is not None and node.autofocus_timer is not None   # not there yet
    node.update_waypoint_reached(WaypointReached(wp_seq=9))
    assert node.timer is None and node.autofocus_timer is None
    assert timer_count(node) == before               # both timers destroyed, none leaked


def test_reaching_buffer_wp_before_any_photo_is_harmless(make_node):
    node = make_node()
    node.buffer_wp = 9
    node.update_waypoint_reached(WaypointReached(wp_seq=9))
    assert node.timer is None and node.autofocus_timer is None


def test_a_new_digicamctrl_after_the_stop_starts_photos_again(make_node):
    # Flying the mission again (START OVER, a second lap) must take photos again.
    node = make_node()
    node.buffer_wp = 9
    node.statustext_callback(status(DIGICAM_TEXT))
    node.update_waypoint_reached(WaypointReached(wp_seq=9))
    node.statustext_callback(status(DIGICAM_TEXT))
    assert node.timer is not None and node.autofocus_timer is not None


# ------------------------------------------------------------------ buffer_wp (waypoint_manager)
class FakeFuture:
    """Stand-in for an rclpy service future: result() returns the answer or raises."""

    def __init__(self, answer=None, error=None):
        self.answer, self.error = answer, error

    def result(self):
        if self.error is not None:
            raise self.error
        return self.answer


def test_buffer_wp_answer_is_used(make_node):
    node = make_node()
    answer = SimpleNamespace(values=[ParameterValue(type=2, integer_value=9)])
    node._buffer_wp_cb(FakeFuture(answer))
    assert node.buffer_wp == 9


@pytest.mark.parametrize('future', [
    FakeFuture(SimpleNamespace(values=[ParameterValue(type=0)])),     # NOT_SET
    FakeFuture(SimpleNamespace(values=[])),                           # no value at all
    FakeFuture(error=RuntimeError('service gone')),                   # the call failed
])
def test_a_failed_buffer_wp_lookup_means_minus_one(make_node, future):
    # -1 = photos run until shutdown instead of stopping at buffer_wp; never a crash.
    node = make_node()
    node.buffer_wp = 5
    node._buffer_wp_cb(future)
    assert node.buffer_wp == -1


def test_only_waypoint_manager_buffer_wp_events_cause_a_new_lookup(make_node):
    node = make_node()
    calls = []
    node.fetch_mission_indices = lambda: calls.append(1)

    def event(node_name, param_name):
        return ParameterEvent(node=node_name, changed_parameters=[Parameter(name=param_name)])

    node.parameter_event_cb(event('/waypoint_manager', 'buffer_wp'))
    node.parameter_event_cb(event('/waypoint_manager', 'rtl_index'))
    node.parameter_event_cb(event('/some_other_node', 'buffer_wp'))
    assert calls == [1]


# ------------------------------------------------------------------ what it publishes
def test_it_publishes_what_the_camera_node_listens_for(make_node):
    node = make_node()
    listener = rclpy.create_node('trigger_test_listener')
    commands, triggers, acks = [], [], []
    listener.create_subscription(String, '/camera/command', lambda m: commands.append(m.data), 10)
    listener.create_subscription(Bool, '/camera/trigger', lambda m: triggers.append(m.data), 10)
    listener.create_subscription(StatusText, '/mavros/statustext/send',
                                 lambda m: acks.append(m.text), 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(listener)
    try:
        def spin_until(done, seconds):
            end = time.monotonic() + seconds
            while not done() and time.monotonic() < end:
                executor.spin_once(timeout_sec=0.05)
            return done()

        # Wait until the publishers have found the listener (discovery takes a moment).
        assert spin_until(lambda: (node.camera_command_pub.get_subscription_count() >= 1
                                   and node.camera_trigger_pub.get_subscription_count() >= 1
                                   and node.status_publisher.get_subscription_count() >= 1), 10.0)
        node.statustext_callback(status(DIGICAM_TEXT))
        node.trigger_camera()
        node.request_autofocus()
        assert spin_until(lambda: ('capture_setup' in commands and 'autofocus' in commands
                                   and True in triggers
                                   and 'Camera trigger STARTED' in acks), 10.0), \
            f'commands={commands} triggers={triggers} acks={acks}'
    finally:
        executor.shutdown()
        listener.destroy_node()
