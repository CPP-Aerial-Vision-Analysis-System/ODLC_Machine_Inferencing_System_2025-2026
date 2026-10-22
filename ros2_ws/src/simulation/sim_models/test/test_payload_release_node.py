"""
test/test_payload_release_node.py -- PURPOSE: test the payload_release ROS node without Gazebo.
A fake GzLink records the topics the node subscribes/publishes and lets the test inject servo
values, payload positions and DetachableJoint states; the node's timer step (_tick) is called by
hand. Checks: no drop while closed, exactly one release on 1400 us, detach repeats until the joint
confirms, events (released / detached / landed) with ENU and lat/lon, the other payload untouched.

Run:  python3 -m pytest -q test/test_payload_release_node.py   (needs ROS 2 Humble sourced)
"""

import json
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

rclpy = pytest.importorskip("rclpy", reason="ROS 2 (rclpy) is needed for the node test")

from sim_models.payload_release_node import _make_node_class  # noqa: E402


class FakeGzLink:
    """Stands in for GzLink: stores callbacks per topic and records published detach topics."""

    def __init__(self):
        self.callbacks = {}
        self.rates = {}
        self.published = []

    def subscribe_double(self, topic, callback, rate_hz=0.0):
        self.callbacks[topic] = callback
        self.rates[topic] = rate_hz
        return True

    def subscribe_odometry(self, topic, callback, rate_hz=0.0):
        self.callbacks[topic] = callback
        return True

    def subscribe_string(self, topic, callback):
        self.callbacks[topic] = callback
        return True

    def advertise_empty(self, topic):
        return lambda: self.published.append(topic)


class FakePublisher:
    def __init__(self):
        self.events = []

    def publish(self, msg):
        self.events.append(json.loads(msg.data))


@pytest.fixture
def node_and_link():
    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            "world_origin_lat:=36.2124",
            "-p",
            "world_origin_lon:=-96.004",
            "-p",
            "world_origin_alt:=200.0",
        ]
    )
    _, PayloadReleaseNode = _make_node_class()
    link = FakeGzLink()
    node = PayloadReleaseNode(gz_link=link)
    node.events_pub = FakePublisher()
    yield node, link
    node.destroy_node()
    rclpy.shutdown()


def test_subscribes_to_contract_topics(node_and_link):
    node, link = node_and_link
    for topic in (
        "/sim/payload/servo11",
        "/sim/payload/servo13",
        "/sim/drone/odometry",
        "/sim/payload/person/odometry",
        "/sim/payload/tent/odometry",
        "/sim/payload/person/state",
        "/sim/payload/tent/state",
    ):
        assert topic in link.callbacks, topic
    assert link.rates["/sim/payload/servo11"] == pytest.approx(50.0)  # throttled 1 kHz stream


def test_closed_servo_does_not_drop(node_and_link):
    node, link = node_and_link
    link.callbacks["/sim/payload/servo11"](500.0)
    link.callbacks["/sim/payload/servo13"](0.0)
    node._tick()
    assert link.published == []
    assert node.events_pub.events == []


def test_release_detach_confirm_and_landing(node_and_link):
    node, link = node_and_link
    cb = link.callbacks
    cb["/sim/drone/odometry"]((10.0, 1.0, 2.0, 15.0))
    cb["/sim/payload/person/odometry"]((10.0, 1.0, 2.0, 14.92))
    cb["/sim/payload/servo11"](500.0)
    node._tick()
    cb["/sim/payload/servo11"](3000.0 * (1400.0 / 3000.0))  # 1400 us through the float relay
    node._tick()
    assert link.published == ["/sim/payload/person/detach"]
    released = node.events_pub.events[-1]
    assert released["event"] == "released" and released["payload"] == "person"
    assert released["servo"] == 11 and released["pwm_us"] == pytest.approx(1400.0, abs=0.01)
    assert released["drone_enu"] == [1.0, 2.0, 15.0]
    assert released["position_enu"] == [1.0, 2.0, 14.92]
    assert 36.2 < released["position_lat"] < 36.3 and -96.1 < released["position_lon"] < -96.0

    # no confirmation yet: the node repeats the detach (0.2 s apart, 3 in total)
    time.sleep(0.25)
    node._tick()
    assert link.published.count("/sim/payload/person/detach") == 2
    cb["/sim/payload/person/state"]("detached")
    node._tick()
    time.sleep(0.25)
    node._tick()
    assert link.published.count("/sim/payload/person/detach") == 2  # confirmed: no more repeats
    assert node.events_pub.events[-1]["event"] == "detached"

    # the payload falls (sim time from the odometry) and rests on the ground
    t, z, vz = 10.0, 14.92, 0.0
    while z > 0.033:
        t += 0.05
        vz -= 9.81 * 0.05
        z = max(0.033, z + vz * 0.05)
        cb["/sim/payload/person/odometry"]((t, 1.0, 2.0, z))
    for _ in range(30):
        t += 0.05
        cb["/sim/payload/person/odometry"]((t, 1.0, 2.0, 0.033))
    node._tick()
    landed = node.events_pub.events[-1]
    assert landed["event"] == "landed"
    assert landed["position_enu"] == [1.0, 2.0, 0.033]
    assert landed["fall_height_m"] == pytest.approx(14.887, abs=0.01)
    assert landed["drift_m"] == pytest.approx(0.0)

    # the servo closes and opens again: nothing more happens; the tent was never touched
    cb["/sim/payload/servo11"](500.0)
    node._tick()
    cb["/sim/payload/servo11"](1400.0)
    node._tick()
    assert link.published.count("/sim/payload/person/detach") == 2
    assert "/sim/payload/tent/detach" not in link.published


def test_release_without_positions_still_detaches(node_and_link):
    """Missing odometry must not stop the drop (only the landing point is then unknown)."""
    node, link = node_and_link
    link.callbacks["/sim/payload/servo13"](1400.0)
    node._tick()
    assert link.published == ["/sim/payload/tent/detach"]
    ev = node.events_pub.events[-1]
    assert ev["event"] == "released" and ev["payload"] == "tent" and "position_enu" not in ev


def test_attached_again_rearms(node_and_link):
    """Gazebo restarted while the node kept running: the new payload must be droppable."""
    node, link = node_and_link
    cb = link.callbacks
    cb["/sim/payload/servo11"](1400.0)
    cb["/sim/payload/person/state"]("detached")
    node._tick()
    assert link.published.count("/sim/payload/person/detach") == 1
    cb["/sim/payload/person/state"]("attached")  # a new drone/payload was spawned
    cb["/sim/payload/servo11"](1000.0)  # SITL start value = closed
    node._tick()
    assert node.events_pub.events[-1]["event"] == "attached"
    assert link.published.count("/sim/payload/person/detach") == 1
    cb["/sim/payload/servo11"](1400.0)
    node._tick()
    assert link.published.count("/sim/payload/person/detach") == 2
