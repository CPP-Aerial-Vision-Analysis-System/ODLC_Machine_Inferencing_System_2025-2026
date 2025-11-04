import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode, WaypointReached, State, StatusText
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, SetMode

from interfaces.srv import AddWaypoint, DelWaypoint, UpdateMission

class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        # Initialize variables
        self.connected = False 
        self.waypoint_reached = 0 
        self.waypoint_list = WaypointList()

        # Subscribers
        self.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.waypoint_reached_cb, 10)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_list, 10)
        self.status_pub = self.create_publisher(StatusText, "/mavros/statustext/send", 10)   # should this be pub??

        # mavros Clients
        self.waypoint_pull = self.create_client(WaypointPull, "/mavros/mission/pull")
        self.waypoint_push = self.create_client(WaypointPush, "/mavros/mission/push")
        self.waypoint_clear = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")

        # Custom Services
        self.add_srv = self.create_service(AddWaypoint, "/addWaypoint", self.handle_wp_req)
        self.del_srv = self.create_service(DelWaypoint, "/delWaypoint", self.handle_wp_del_req)
        self.update_srv = self.create_service(UpdateMission, "/updateMission", self.handle_update_mission)
        # self.do_jump_srv = self.create_service(DoJump, "/doJump", self.handle_do_jump)

        # Node parameters   (move to a mission manager?)
        #Where are these used? we should create getters for these parameters, otherwise we are just sitting here 
        self.declare_parameter('num_waypoints', 0)
        self.declare_parameter('takeoff_index', -1)
        self.declare_parameter('rtl_index', -1)
        self.declare_parameter('next_after_takeoff', -1)
        self.declare_parameter('last_before_rtl', -1)

    def state_callback(self, msg):
        """Callback function for state updates."""
        self.connected = msg.connected
    
    def waypoints_list(self, data):
        """receives and stores the waypoint lists from mavros"""
        self.waypoint_list = data
        for i, wp in enumerate(self.waypoint_list.waypoints):
            self.get_logger().info(f"Waypoint {type(wp)} {i}: Lat: {wp.x_lat}, Lon: {wp.y_long}, Alt: {wp.z_alt}")
    
    def push_waypoints(self):
        # infinite loop until the waypoint push service is available(toFix)
        """Push waypoints to the drone"""
        while not self.waypoint_push.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint push service...")
        
        try:
            waypoint_push_request = WaypointPush.Request()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            push_result = self.waypoint_push.call_async(waypoint_push_request)
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")
    
    def pull_waypoints(self):
        """Update waypoint list from the drone."""
        while not self.waypoint_pull.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint pull service...")
        
        future = self.waypoint_pull.call_async(WaypointPull.Request())
        rclpy.spin_until_future_complete(self, future)
        future.add_done_callback(self.pull_request)
        self.get_logger().info("Waypoint pull request...")
    
    def pull_request(self, future):         # don't need this if we use rclpy.spin_unttil_future_complete
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully pulled {response.wp_received} from drone.")
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")

    def clear_waypoints(self):
        """Clear all waypoints from the drone."""
        while not self.waypoint_clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint clear service...")
        
        try:
            clear_result = self.waypoint_clear_client.call_async()
            if clear_result.result().success:
                self.get_logger().info("Successfully cleared waypoints.")
        except Exception as e:
            self.get_logger().info(f"Service call failed: {e}")
    
    def insert_new_waypoint(self, lat, lon, alt, index):
        """Insert new waypoint into the waypoint list and push the updated list"""
        new_waypoint = Waypoint()
        new_waypoint.frame = 3  # Global relative altitude
        new_waypoint.command = 16
        new_waypoint.is_current = False
        new_waypoint.autocontinue = True
        new_waypoint.param1 = float(5)  # Hold time in seconds
        new_waypoint.param2 = float(0)  # Acceptance radius in meters
        new_waypoint.param3 = float(0)  # Pass through waypoint
        new_waypoint.param4 = float('nan')  # Yaw angle
        new_waypoint.x_lat = float(lat)
        new_waypoint.y_long = float(lon)
        new_waypoint.z_alt = float(alt)
        self.pull_waypoints()
        self.get_logger().info(f"Inserting new waypoint at index {index}: Lat: {lat}, Lon: {lon}, Alt: {alt}")
        self.waypoint_list.waypoints.insert(index, new_waypoint)
        self.push_waypoints()
        self.get_logger().info("Waypoint inserted and pushed successfully.")
    
    def delete_waypoint(self, index):
        """Delete waypoint from the waypoint list and push the updated list"""
        self.pull_waypoints()
        if 0 <= index < len(self.waypoint_list.waypoints):
            del self.waypoint_list.waypoints[index]
            self.get_logger().info(f"Deleted waypoint at index {index}.")
            self.push_waypoints()
            self.get_logger().info("Waypoint deleted and pushed successfully.")
        else:
            self.get_logger().info(f"Index {index} out of range. No waypoint deleted.")
    
    def waypoint_reached_cb(self, msg):                         # change this to match 2025-2026 mission
        """Tells us which waypoint we just reached."""
        self.waypoint_reached = msg.wp_seq
        self.get_logger().info(f"Waypoint {msg.wp_seq} reached.")

        if self.waypoint_reached < len(self.waypoint_list.waypoints):
            wp = self.waypoint_list.waypoints[self.waypoint_reached]
            if int(wp.param1) > 0:
                if int(wp.command) == 16:  # Waypoint command
                    self.get_logger().info("Object waypoint reached.")
                else:
                    self.get_logger().info("Loiter finished. Continuing to next waypoint.")
    
    def change_mode(self, mode):
        """Change flight mode of the drone."""
        self.get_logger().info(f"Chaning mode to {mode}")
        response = self.set_mode.call_async(custom_mode = mode)

        if response.mode_sent:
            self.get_logger().info(f"Mode changed to {mode} successfully.")
        else:
            self.get_logger().info(f"Failed to change mode.")
    
    def send_status(self, text, throttle=False):
        """Send status message to the drone."""
        now = time.time()
        if not throttle or (now - self.last_status_time > self.status_interval):
            status_msg = StatusText()
            status_msg.severity = 6  # 6 = NOTICE
            status_msg.text = text
            self.status_pub.publish(status_msg)
            self.last_status_time = now

    def handle_wp_req(self, request, response):
        """Handle AddWaypoint service request"""
        self.get_logger().info(f"Received AddWaypoint request: lat={request.latitude}, lon={request.longitude}, alt={request.altitude}, index={request.index}")

        response = AddWaypoint.Response()
        try:
            self.insert_new_waypoint(request.latitude, request.longitude, request.altitude, request.index)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to add waypoint: {e}")
            response.success = False
        return response

    def handle_wp_del_req(self, request, response):
        """Handle DelWaypoint service request"""
        self.get_logger().info(f"Received DelWaypoint request: index={request.index}")

        response = DelWaypoint.Response()
        try:
            self.delete_waypoint(request.index)
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Failed to delete waypoint: {e}")
            response.success = False
        return response

    def handle_update_mission(self, request, response):
        self.get_logger().info(f"Received UpdateMission request")
        try:
            # update mission logic 
            # its just setting params for other nodes?
            response = UpdateMission.Response()
            response.success = True         # nothing coded yet
        except Exception as e:
            self.get_logger().error(f"Failed to udpate mission: {e}")
        return response
    
    def main(self):
        self.get_logger().info("Waiting for connection to FCU...")
        while not self.connected:
            rclpy.spin_once(self)
        
        message = f"Heartbeat established"
        self.get_logger().info(message)
        self.send_status(message)
        # self.insert_new_waypoint(1,1,1,3)
        # self.delete_waypoint(2)
        rclpy.spin(self)
        
def main():
    rclpy.init()
    manager = WaypointManager()
    manager.main()
