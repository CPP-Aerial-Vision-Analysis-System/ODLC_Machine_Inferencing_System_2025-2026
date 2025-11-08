
import rclpy
from rclpy.node import Node 
from std_msgs.msg import Bool
from mavros_msgs.msg import StatusText
import re
from rclpy.qos import QoSProfile, qos_profile_sensor_data


class MissionCameraTrigger(Node):

    def __init__(self):
        super().__init__("mission_camera_trigger")
        self.create_subscription(StatusText,'/mavros/statustext/recv', self.statustext_callback, qos_profile_sensor_data)
        self.camera_trigger_pub = self.create_publisher(Bool, "/camera/trigger", 10) 

    def statustext_callback(self, msg):
        if "DigiCamCtrl" in msg.text:
            match = re.search(r"Mission:\s*(\d+)", msg.text)
            wp = match.group(1) if match else "?"
            self.get_logger().info(f"Camera trigger from DigiCamCtrl at waypoint {wp}")
            self.trigger_camera()

    def trigger_camera(self):
        self.get_logger().info("Triggering Jetson-side camera")
        self.camera_trigger_pub.publish(Bool(data=True))


def main(args=None):
        rclpy.init(args=args)
        mapping_node = MissionCameraTrigger()
        rclpy.spin(mapping_node)
        mapping_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()