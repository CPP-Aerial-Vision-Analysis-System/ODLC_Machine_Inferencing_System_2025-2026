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


# Seconds between the repeating autofocus passes that DO_DIGICAM_CONTROL starts.
# Each pass is CMD 0x04 "Auto Focus" (ZR10 User Manual v1.7 p.43), whose payload
# byte is documented as "1: Start auto focus for once" -- a ONE-SHOT cycle, not a
# continuous-AF mode the camera keeps running by itself. Holding focus across a
# survey therefore means re-sending the command on a timer.
AUTOFOCUS_PERIOD_SECONDS = 5.0


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
        # Repeating autofocus timer, started by the same DigiCamCtrl that starts
        # capture. Same None-guard contract as self.timer above.
        self.autofocus_timer = None

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
        # Stopped FIRST, and deliberately outside the self.timer guard below:
        # the autofocus timer is (re)started by EVERY DigiCamCtrl while the
        # capture timer is only created by the first, so returning early on
        # `self.timer is None` would leak an autofocus timer that kept driving
        # the lens after the survey had already stopped.
        self.stop_autofocus_loop()
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
            # Set the lens up on arrival at the DO_DIGICAM_CONTROL waypoint:
            # rack to CAPTURE_ZOOM_X, then focus. siyi_node queues both onto
            # one lens worker and holds the camera lock throughout, so the
            # captures started below wait for a settled lens rather than
            # firing mid-rack.
            self.request_capture_setup(wp)
            # Keep re-focusing for the rest of the survey. Started BEFORE the
            # capture-timer guard below, so a repeat DigiCamCtrl that returns
            # early there still re-arms this loop if buffer_wp had stopped it.
            self.start_autofocus_loop()
            # Idempotent start: if a timer is already running, a second
            # DigiCamCtrl must NOT spawn another timer (that would stack the
            # capture rate and leak timers that cancel() can no longer reach).
            if self.timer is not None:
                return
            self.timer = self.create_timer(1, self.trigger_camera)
            self.get_logger().info("Camera trigger STARTED")
            self.send_ack(f"Camera trigger STARTED")

    def request_capture_setup(self, wp="?"):
        # siyi_node parses a bare (non-JSON) string as "<command> [parameter]".
        # "capture_setup" racks the lens to video_cam/config.py CAPTURE_ZOOM_X
        # and THEN focuses per FOCUS_MODE -- infinity by default for mapping,
        # since every subject at survey altitude is past the hyperfocal
        # distance. (Not "autofocus", which would hard-force AF and override
        # that setting.)
        #
        # Deliberately ONE command rather than a "zoom" followed by a "focus":
        # published separately the two would race on siyi_node's worker, and a
        # focus that won would be undone anyway by the autofocus CMD 0x0F
        # carries. It is also idempotent -- DigiCamCtrl repeats on every pass
        # through the waypoint, and a lens already at CAPTURE_ZOOM_X is left
        # alone rather than re-racked.
        self.camera_command_pub.publish(String(data="capture_setup"))
        self.get_logger().info(
            f"Zoom + focus requested (DigiCamCtrl at waypoint {wp})")

    def start_autofocus_loop(self):
        """Autofocus every AUTOFOCUS_PERIOD_SECONDS until the survey stops.

        Publishes the plain "autofocus" command, the one siyi_node handles
        AHEAD of its pipeline-busy guard and queues onto the single lens worker
        thread. Three properties of that path are what make a 5 s loop safe to
        run underneath 1 Hz captures:

          - it is not rejected while the pipeline is busy, which during a survey
            is almost continuously -- the trap that stopped the original
            one-shot autofocus here from ever taking effect;
          - a pass that lands while the DO_DIGICAM_CONTROL zoom rack is still
            travelling is DROPPED by the worker's busy check rather than queued
            behind it, so passes can never stack up on a slow lens;
          - "autofocus" pins mode='auto' (CMD 0x04 at the frame centre) instead
            of following FOCUS_MODE, so this is a real AF cycle and not the
            drive-to-infinity that FOCUS_MODE='infinity' would otherwise do.

        Idempotent, like the capture timer: DigiCamCtrl repeats on every pass
        through the waypoint and a second one must not spawn a second timer.
        """
        if self.autofocus_timer is not None:
            return
        self.autofocus_timer = self.create_timer(
            AUTOFOCUS_PERIOD_SECONDS, self.request_autofocus)
        self.get_logger().info(
            f"Autofocus loop STARTED (every {AUTOFOCUS_PERIOD_SECONDS:.0f}s)")
        self.send_ack(f"Autofocus every {AUTOFOCUS_PERIOD_SECONDS:.0f}s")

    def request_autofocus(self):
        self.camera_command_pub.publish(String(data="autofocus"))
        self.get_logger().info("Autofocus requested")

    def stop_autofocus_loop(self):
        # Same guard as stop_camera_trigger: buffer_wp can be reached before any
        # DigiCamCtrl ever started the loop.
        if self.autofocus_timer is None:
            return
        self.autofocus_timer.cancel()
        self.destroy_timer(self.autofocus_timer)
        self.autofocus_timer = None
        self.get_logger().info("Autofocus loop STOPPED")
        self.send_ack("Autofocus loop STOPPED")

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
