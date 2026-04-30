#!/usr/bin/env python3

import rclpy
from rclpy.node import Node 
from std_msgs.msg import Bool
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
        self.create_subscription(ParameterEvent, "/parameter_events", self.parameter_event_cb, 10)


        self.buffer_wp = -1
        self.waypoint_reached = 0

        self.param_manager = ParameterManager()
        self.fetch_mission_indices()
    
    def fetch_mission_indices(self):
        wp_params = ['buffer_wp']
        params = self.param_manager.get_param(self.param_manager.waypoint_client, list_params=wp_params)
        if not params or 'buffer_wp' not in params:
            self.get_logger().warning('Could not fetch buffer_wp, using default -1')
            self.buffer_wp = -1
            return

        try:
            self.buffer_wp = int(params['buffer_wp'])
        except (TypeError, ValueError) as exc:
            self.get_logger().error(f'Invalid parameter buffer_wp: {exc}')
            self.buffer_wp = -1

    def parameter_event_cb(self, msg: ParameterEvent):
        if msg.node == "/waypoint_manager":
            for changed_param in msg.changed_parameters:
                name = changed_param.name
                value = changed_param.value

                if name in {"buffer_wp"}:
                    #self.get_logger().info(f"[Param Update] {name} changed")
                    self.fetch_mission_indices()
                    break
    
    def update_waypoint_reached(self, msg):
         self.waypoint_reached = msg.wp_seq  
         if self.waypoint_reached == self.buffer_wp:
            self.timer.cancel()    
            self.last_before_rtl = -1
            self.get_logger().info("Camera trigger STOPPED")
            self.send_ack(f"Camera trigger STOPPED")

    def statustext_callback(self, msg):
        if "DigiCamCtrl" in msg.text:
            match = re.search(r"Mission:\s*(\d+)", msg.text)
            wp = match.group(1) if match else "?"
            # self.get_logger().info(f"Camera trigger from DigiCamCtrl at waypoint {wp}")
            self.timer = self.create_timer(1, self.trigger_camera)
            self.get_logger().info("Camera trigger STARTED")
            self.send_ack(f"Camera trigger STARTED")

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