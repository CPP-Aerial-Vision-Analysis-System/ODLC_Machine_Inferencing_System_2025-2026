#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from mavros_msgs.msg import StatusText, WaypointReached
import re
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterEvent

from wp_sender.parameter import ParameterManager


class MissionCameraTrigger(Node):

    def __init__(self):
        super().__init__("mission_camera_trigger")
        self.create_subscription(StatusText,'/mavros/statustext/recv', self.statustext_callback, qos_profile_sensor_data)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.update_waypoint_reached, 1)

        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        self.camera_trigger_pub = self.create_publisher(Bool, "/camera/trigger", 10)
        # Equivalent of:
        #   ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'autofocus'"
        self.camera_command_pub = self.create_publisher(String, "/camera/command", 10)
        self.create_subscription(ParameterEvent, "/parameter_events", self.parameter_event_cb, 10)


        self.buffer_wp = -1
        self.waypoint_reached = 0
        # Single capture timer. None when not capturing; guards against stacking
        # multiple timers and against cancelling a timer that was never created.
        self.timer = None

        # buffer_wp is fetched asynchronously on this node's own executor.
        # It must never block __init__: main() would not reach rclpy.spin(),
        # leaving the node alive with its statustext subscription registered
        # but no callback ever running -- DigiCamCtrl silently dropped and no
        # photos taken, while every other node looks healthy.
        # It must also not run on a side thread: building a second node while
        # this one spins corrupts the executor wait set (IndexError: wait set
        # index too big). Client here (before spin), request from a timer.
        self.param_client = self.create_client(
            GetParameters, 'waypoint_manager/get_parameters')
        self.param_timer = self.create_timer(2.0, self.fetch_mission_indices)

    def fetch_mission_indices(self):
        # Retried by the timer until waypoint_manager shows up.
        if not self.param_client.service_is_ready():
            return
        if self.param_timer is not None:
            self.param_timer.cancel()
            self.destroy_timer(self.param_timer)
            self.param_timer = None

        req = GetParameters.Request()
        req.names = ['buffer_wp']
        self.param_client.call_async(req).add_done_callback(self._buffer_wp_cb)

    def _buffer_wp_cb(self, future):
        # -1 just means the trigger runs until shutdown instead of stopping at
        # buffer_wp. Capture must never depend on this lookup succeeding.
        try:
            values = future.result().values
            if values and values[0].type == 2:
                self.buffer_wp = values[0].integer_value
            else:
                self.get_logger().warning('buffer_wp unset, using -1')
                self.buffer_wp = -1
        except Exception as exc:
            self.get_logger().error(f'buffer_wp lookup failed ({exc}); using -1')
            self.buffer_wp = -1
        self.get_logger().info(f'buffer_wp = {self.buffer_wp}')

    def parameter_event_cb(self, msg: ParameterEvent):
        if msg.node == "/waypoint_manager":
            for changed_param in msg.changed_parameters:
                name = changed_param.name
                value = changed_param.value

                if name in {"buffer_wp"}:
                    # Non-blocking: call_async + done-callback on this executor.
                    self.fetch_mission_indices()
                    break

    def update_waypoint_reached(self, msg):
         self.waypoint_reached = msg.wp_seq
         if self.waypoint_reached == self.buffer_wp:
            self.stop_camera_trigger()

    def stop_camera_trigger(self):
        # Guard: buffer_wp may be reached before any DigiCamCtrl ever started a
        # timer. Only cancel/destroy when a timer actually exists.
        if self.timer is None:
            return
        self.timer.cancel()
        self.destroy_timer(self.timer)
        self.timer = None
        self.last_before_rtl = -1
        self.get_logger().info("Camera trigger STOPPED")
        self.send_ack("Camera trigger STOPPED")

    def statustext_callback(self, msg):
        if "DigiCamCtrl" in msg.text:
            match = re.search(r"Mission:\s*(\d+)", msg.text)
            wp = match.group(1) if match else "?"
            # self.get_logger().info(f"Camera trigger from DigiCamCtrl at waypoint {wp}")
            # Refocus the lens on arrival at the DO_DIGICAM_CONTROL waypoint.
            # siyi_node queues this onto its focus worker and holds the camera
            # lock through the lens settle, so captures started below simply
            # wait for a settled lens instead of firing mid-rack.
            self.request_autofocus(wp)
            # Idempotent start: if a timer is already running, a second
            # DigiCamCtrl must NOT spawn another timer (that would stack the
            # capture rate and leak timers that cancel() can no longer reach).
            if self.timer is not None:
                return
            self.timer = self.create_timer(1, self.trigger_camera)
            self.get_logger().info("Camera trigger STARTED")
            self.send_ack(f"Camera trigger STARTED")

    def request_autofocus(self, wp="?"):
        # siyi_node parses a bare (non-JSON) string as "<command> [parameter]".
        # "focus" (not "autofocus") lets video_cam/config.py FOCUS_MODE decide
        # HOW to focus -- infinity by default for mapping, since every subject
        # at survey altitude is past the hyperfocal distance. Publishing
        # "autofocus" here would hard-force AF and override that setting.
        self.camera_command_pub.publish(String(data="focus"))
        self.get_logger().info(f"Focus requested (DigiCamCtrl at waypoint {wp})")

    def trigger_camera(self):
        self.get_logger().info("Triggering camera...")
        self.camera_trigger_pub.publish(Bool(data=True))

    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {text}")


def main(args=None):
        rclpy.init(args=args)
        mapping_node = MissionCameraTrigger()
        rclpy.spin(mapping_node)
        mapping_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
