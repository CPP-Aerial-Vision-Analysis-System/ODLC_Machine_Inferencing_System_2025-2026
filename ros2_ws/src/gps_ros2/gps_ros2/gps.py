#!/usr/bin/env python3

# ROS2 imports (rclpy replaces rospy)
import rclpy                                  # ROS2 client library
from rclpy.node import Node                   # Base class to create a ROS2 node

# Messages / Services (same packages, ROS2 modules)
from mavros_msgs.msg import State             # /mavros/state message (same type name)
from sensor_msgs.msg import NavSatFix         # GPS fix message (same type name)
from std_msgs.msg import Float64              # Heading / yaw in degrees (same type name)
from mavros_msgs.srv import StreamRate        # MAVROS service for stream rate (same type name)
from interfaces.srv import GetGPSData         # Your custom service (ROS2 module path)

from rclpy.qos import qos_profile_sensor_data, QoSProfile


# ROS2 we prefer timers for heartbeat

class GPSMavrosServiceNode(Node):
    """
    ROS2 version of your ROS1 gps_mavros_service_node.
    - Keeps the SAME node name: 'gps_mavros_service_node'
    - Keeps the SAME variable names: connected, latest_gps, latest_yaw
    - Converts rospy APIs to rclpy equivalents
    """

    def __init__(self):
        # ROS1: rospy.init_node("gps_mavros_service_node", anonymous=True)
        # ROS2: initialize by calling super().__init__ with the node name
        super().__init__('gps_mavros_service_node')

        # Variables (kept same names as requested) ===
        # ROS1: globals; ROS2: make them instance vars but keep same names
        self.connected = False                 # same meaning as ROS1 global
        self.latest_gps = None                 # same meaning as ROS1 global
        self.latest_yaw = None                 # same meaning as ROS1 global
        self._last_connected = None            # for heartbeat change detection

        # === Subscribers (ROS1: rospy.Subscriber → ROS2: create_subscription) ===
        # State
        self.create_subscription(State,'/mavros/state', self.state_cb,10)  # msg type, topic (same), callback (same name), QoS depth (ROS2 needs QoS; 10 is a good default)
                                    
        
        # GPS
        self.create_subscription(NavSatFix,'/mavros/global_position/global', self.gps_cb, qos_profile_sensor_data)
        
        # Yaw / heading (Float64)
        self.create_subscription(Float64,'/mavros/global_position/compass_hdg', self.pose_callback, qos_profile_sensor_data)

        # (ROS1: rospy.Service → ROS2: create_service) ===
        # comment out until needed
        self.service = self.create_service(
           GetGPSData,                        # srv type (ROS2 import path)
           '/get_drone_data',                 # service name (same)
           self.handle_drone_data_request     # callback (same name)
        )
        self.get_logger().info('Drone Data Service Ready on /get_drone_data')

        #Service Client (ROS1: wait_for_service + ServiceProxy → ROS2: create_client) ===
        # Create the client once and reuse it
        self._stream_rate_cli = self.create_client(StreamRate, '/mavros/set_stream_rate')

        # In ROS1 rospy.wait_for_service('/mavros/set_stream_rate')
        # In ROS2, wait with a loop (non-blocking spin is okay; we’re in constructor)
        while not self._stream_rate_cli.wait_for_service(timeout_sec=1.0):
            # If node is shutting down, stop waiting
            if not rclpy.ok():
                self.get_logger().warn('Shutting down while waiting for /mavros/set_stream_rate')
                break
            self.get_logger().warn('Waiting for /mavros/set_stream_rate ...')

        #Heartbeat (ROS1 used a thread with rospy.Rate; ROS2 prefers timers) ===
        # This timer runs every 1.0s and logs only on connection state changes
        self.create_timer(1.0, self.heartbeat_tick)

        # === Optionally set stream rate once at startup (mirrors your ROS1 main) ===
        # ROS1: set_stream_rate(stream_id=0, message_rate=1, on_off=True)
        # ROS2: call our method below (uses async client under the hood)
        self.set_stream_rate(stream_id=0, message_rate=1, on_off=True)

    # callbacks (kept same names, translated to ROS2) ========

    def state_cb(self, msg):
        """Callback to monitor MAVROS connection state."""
        # ROS1: global connected = msg.connected
        # ROS2: instance variable but same name
        self.connected = msg.connected

    def gps_cb(self, msg):
        """Callback to store the latest GPS data."""
        self.latest_gps = msg
        # self.get_logger().info(f"{self.latest_gps}")

    def pose_callback(self, msg: Float64):
        """Callback to store the latest yaw data (Z-axis rotation)."""
        self.latest_yaw = msg.data

    # Heartbeat via timer (replaces while loop + rospy.Rate + thread) 

    def heartbeat_tick(self):
        """
        ROS1 heartbeat() used:
          rate = rospy.Rate(1); while not rospy.is_shutdown(): if changed: log; rate.sleep()
        ROS2 version:
          Use a 1 Hz timer that fires this method; log only on change.
        """
        if self.connected != self._last_connected:
            if self.connected:
                self.get_logger().info('Heartbeat: Connected to Pixhawk')
            else:
                self.get_logger().warn('Heartbeat: Disconnected from Pixhawk')
            self._last_connected = self.connected

    # service: Set MAVROS stream rate (ROS2 client)

    def set_stream_rate(self, stream_id: int = 0, message_rate: int = 10, on_off: bool = True):
        """
        Set MAVROS data stream rate.
        ROS1:
          rospy.wait_for_service + ServiceProxy + try/except rospy.ServiceException
        ROS2:
          create_client once; build Request; call_async; spin until result; check future.exception()
        """
        if not self._stream_rate_cli.service_is_ready():
            self.get_logger().warn('Service /mavros/set_stream_rate not ready (skipping initial call).')
            return

        req = StreamRate.Request()
        req.stream_id = stream_id
        req.message_rate = message_rate
        req.on_off = on_off

        future = self._stream_rate_cli.call_async(req)

        # Block this node until the service returns (like your ROS1 call was blocking)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            # StreamRate.srv response is empty; success if no exception
            self.get_logger().info(f"Stream rate set: ID={stream_id}, Rate={message_rate}Hz")
        else:
            # ROS2: errors surface as exceptions on the future (no rospy.ServiceException)
            self.get_logger().error(f"Failed to set stream rate: {future.exception()}")

    # Service Server callback (returns response object in ROS2) 

    def handle_drone_data_request(self, request: GetGPSData.Request, response: GetGPSData.Response):
        """
        ROS1:
          if latest_* present → fill GetGPSDataResponse(...) and return
          else → warn and return zeros
        ROS2:
          We fill the provided 'response' object and return it.
        """
        if (self.latest_gps is not None) and (self.latest_yaw is not None):
            response.latitude = self.latest_gps.latitude
            response.longitude = self.latest_gps.longitude
            response.altitude = self.latest_gps.altitude
            response.yaw = self.latest_yaw
        else:
            self.get_logger().warn('No GPS or yaw data received yet!')
            response.latitude = 0.0
            response.longitude = 0.0
            response.altitude = 0.0
            response.yaw = 0.0
        return response


# main() mirrors yhe ConvertServer pattern
def main(args=None):
    # ROS1: rospy.init_node(...)
    # ROS2: rclpy.init()
    rclpy.init(args=args)
    test = GPSMavrosServiceNode()
    rclpy.spin(test)
    test.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
