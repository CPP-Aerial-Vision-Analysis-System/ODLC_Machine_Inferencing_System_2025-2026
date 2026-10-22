"""
example_listener/node.py -- PURPOSE: the ROS 2 node of the TEMPLATE "extra": it LISTENS to the flight (MAVROS
telemetry, mission progress, the shutter, the detector's results, the text messages) and prints a summary every
few seconds; when it stops it saves the summary as JSON. It publishes nothing, so it can never disturb the
mission: that is the rule for extras (docs/simulation/PLUGINS.md, "extras").

Topics (all subscribed; QoS chosen so it hears every publisher):
  /mavros/state                     mavros_msgs/State        BEST_EFFORT  (hears RELIABLE publishers too)
  /mavros/global_position/global    sensor_msgs/NavSatFix    BEST_EFFORT  (MAVROS publishes it best effort)
  /mavros/global_position/rel_alt   std_msgs/Float64         BEST_EFFORT
  /mavros/mission/reached           mavros_msgs/WaypointReached BEST_EFFORT
  /mavros/statustext/recv           mavros_msgs/StatusText   BEST_EFFORT
  /camera/trigger                   std_msgs/Bool            BEST_EFFORT
  /image_detection                  interfaces/ImageResult   RELIABLE (like main_controller)
Parameters: summary_period_s (10.0; 0 = only at the end), summary_file ("" = ~/.astra_sim/example_listener_summary.json).
Copy this package to make your own logger / monitor (README.md: copy -> rename -> change -> plug in -> check).
"""

from __future__ import annotations

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

from .summary import FlightSummary


class ExampleListener(Node):
    """Feeds every message into a FlightSummary and prints / saves it."""

    def __init__(self):
        super().__init__('example_listener')
        self.declare_parameter('summary_period_s', 10.0)
        self.declare_parameter('summary_file', '')
        period = float(self.get_parameter('summary_period_s').value)
        self.summary_file = (self.get_parameter('summary_file').value
                             or os.path.join(os.environ.get('ASTRA_SIM_HOME', os.path.expanduser('~/.astra_sim')),
                                             'example_listener_summary.json'))
        self.summary = FlightSummary()

        from interfaces.msg import ImageResult
        from mavros_msgs.msg import State, StatusText, WaypointReached
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Bool, Float64
        best_effort = qos_profile_sensor_data     # BEST_EFFORT: receives from reliable AND best-effort publishers
        s = self.summary
        self.create_subscription(State, '/mavros/state',
                                 lambda m: s.on_state(m.connected, m.armed, m.mode, time.monotonic()), best_effort)
        self.create_subscription(NavSatFix, '/mavros/global_position/global',
                                 lambda m: s.on_fix(m.latitude, m.longitude), best_effort)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', lambda m: s.on_rel_alt(m.data),
                                 best_effort)
        self.create_subscription(WaypointReached, '/mavros/mission/reached', lambda m: s.on_reached(m.wp_seq),
                                 best_effort)
        self.create_subscription(StatusText, '/mavros/statustext/recv', lambda m: s.on_statustext(m.text),
                                 best_effort)
        self.create_subscription(Bool, '/camera/trigger', lambda m: s.on_trigger(m.data), best_effort)
        self.create_subscription(ImageResult, '/image_detection', self._on_detection,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        if period > 0:
            self.create_timer(period, self._print)
        self.get_logger().info(f'listening (summary every {period:g} s; saved to {self.summary_file} at the end)')

    def _on_detection(self, msg) -> None:
        results = []
        for det in msg.detections.detections:
            if det.results:
                results.append((det.results[0].hypothesis.class_id, float(det.results[0].hypothesis.score)))
        self.summary.on_detection(msg.image_name, results)

    def _print(self) -> None:
        self.get_logger().info('summary:\n' + self.summary.text(time.monotonic()))

    def save(self) -> None:
        """Write the summary JSON (called when the node stops)."""
        try:
            os.makedirs(os.path.dirname(self.summary_file) or '.', exist_ok=True)
            with open(self.summary_file, 'w', encoding='utf-8') as f:
                json.dump(self.summary.as_dict(time.monotonic()), f, indent=2)
            self.get_logger().info(f'summary saved to {self.summary_file}')
        except OSError as exc:
            self.get_logger().warn(f'could not save the summary: {exc}')


def main(args=None) -> None:
    """Entry point: ros2 run example_listener example_listener."""
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:                      # older rclpy
        ExternalShutdownException = KeyboardInterrupt  # noqa: N806
    rclpy.init(args=args)
    node = ExampleListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl+C, or the launch system stopping us
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
