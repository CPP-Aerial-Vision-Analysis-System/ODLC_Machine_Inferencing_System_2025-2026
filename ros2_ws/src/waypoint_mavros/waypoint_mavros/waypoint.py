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
        self.takeoff_index = -1
        self.next_after_takeoff = -1
        self.rtl_index = -1
        self.last_before_rtl = -1

        # Subscribers and a publisher
        self.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.waypoint_reached_cb, 10)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_list, 10)
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # Mavros Clients
        self.waypoint_pull = self.create_client(WaypointPull, "/mavros/mission/pull") # req current mission from AP
        self.waypoint_push = self.create_client(WaypointPush, "/mavros/mission/push") # gives new mission to AP
        self.waypoint_clear = self.create_client(WaypointClear, "/mavros/mission/clear") # clear mission 
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode") # flight mode change( auto, guided, rtl)

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
        self.declare_parameter('buffer_wp', -1)

    def state_callback(self, msg):
        """Callback function for state updates."""
        self.connected = msg.connected
    
    def waypoints_list(self, data):
        """receives and stores the waypoint lists from mavros"""
        self.waypoint_list = data #stdy
        for i, wp in enumerate(self.waypoint_list.waypoints):       
                 
            if wp.command == CommandCode.NAV_TAKEOFF:
                self.takeoff_index = i
                if i + 1 < len(self.waypoint_list.waypoints):
                    self.next_after_takeoff = i + 1

            if wp.command == CommandCode.NAV_RETURN_TO_LAUNCH:
                self.rtl_index = i
                if i - 1 > 0:
                    self.last_before_rtl = i - 1


        self.get_logger().info(f"Takeoff Index: {self.takeoff_index}, Next After Takeoff: {self.next_after_takeoff}, Last Before RTL: {self.last_before_rtl}, RTL Index: {self.rtl_index}")
        
        self.set_parameters([rclpy.parameter.Parameter('num_waypoints', rclpy.Parameter.Type.INTEGER, len(self.waypoint_list.waypoints))])
        self.set_parameters([rclpy.parameter.Parameter('takeoff_index', rclpy.Parameter.Type.INTEGER, self.takeoff_index)])
        self.set_parameters([rclpy.parameter.Parameter('next_after_takeoff', rclpy.Parameter.Type.INTEGER, self.next_after_takeoff)])
        self.set_parameters([rclpy.parameter.Parameter('rtl_index', rclpy.Parameter.Type.INTEGER, self.rtl_index)])
        self.set_parameters([rclpy.parameter.Parameter('last_before_rtl', rclpy.Parameter.Type.INTEGER, self.last_before_rtl)])
        self.set_parameters([rclpy.parameter.Parameter('buffer_wp', rclpy.Parameter.Type.INTEGER, self.last_before_rtl - 1)])

    
    def reset_indices(self):
        self.num_waypoints = 0
        self.takeoff_index = -1
        self.rtl_index = -1
        self.next_after_takeoff = -1
        self.last_before_rtl = -1
        self.buffer_wp = -1
        self.set_parameters([rclpy.parameter.Parameter('num_waypoints', rclpy.Parameter.Type.INTEGER, 0)])
        self.set_parameters([rclpy.parameter.Parameter('takeoff_index', rclpy.Parameter.Type.INTEGER, -1)])
        self.set_parameters([rclpy.parameter.Parameter('next_after_takeoff', rclpy.Parameter.Type.INTEGER, -1)])
        self.set_parameters([rclpy.parameter.Parameter('rtl_index', rclpy.Parameter.Type.INTEGER, -1)])
        self.set_parameters([rclpy.parameter.Parameter('last_before_rtl', rclpy.Parameter.Type.INTEGER, -1)])
        self.set_parameters([rclpy.parameter.Parameter('buffer_wp', rclpy.Parameter.Type.INTEGER, -1)])

    def push_waypoints(self):
        # infinite loop until the waypoint push service is available(toFix)
        """Push waypoints to the drone"""
        while not self.waypoint_push.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint push service...")
        
        try:
            waypoint_push_request = WaypointPush.Request()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            future = self.waypoint_push.call_async(waypoint_push_request)
            # rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Waypoints pushed successfully")
                    return True
            else:
                # self.get_logger().error("Failed to push waypoints")
                return False
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
    
    def insert_new_waypoint(self, wp_list):
        """Insert new waypoints (NAV + DO_SET_SERVO) into the mission and push.

        wp_list: list of dicts, each with keys:
            - 'command': int (16 for NAV_WAYPOINT, 183 for DO_SET_SERVO)
            - 'index': int (insertion index in mission list)
            For NAV_WAYPOINT (command=16):
                - 'lat', 'lon', 'alt': float
            For DO_SET_SERVO (command=183):
                - 'channel': int (servo output channel, e.g. 9 = AUX1)
                - 'pwm': int (pulse width in μs, e.g. 1900)
        """
        try:
            if len(wp_list) == 0:
                self.get_logger().warn("No waypoints to insert")
                return False

            original_waypoints = self.waypoint_list.waypoints.copy()

            for wp in wp_list:
                command = wp.get('command', 16)
                index = wp['index']

                if index > len(self.waypoint_list.waypoints):
                    self.get_logger().error(f"Index {index} is out of range")
                    self.waypoint_list.waypoints = original_waypoints
                    return False

                new_waypoint = Waypoint()
                new_waypoint.is_current = False
                new_waypoint.autocontinue = True

                if command == 183:  # MAV_CMD_DO_SET_SERVO
                    new_waypoint.frame = 3
                    new_waypoint.command = 183
                    new_waypoint.param1 = float(wp['channel'])
                    new_waypoint.param2 = float(wp['pwm'])
                    new_waypoint.param3 = 0.0
                    new_waypoint.param4 = 0.0
                    new_waypoint.x_lat = 0.0
                    new_waypoint.y_long = 0.0
                    new_waypoint.z_alt = 0.0
                    self.get_logger().info(f"Inserting DO_SET_SERVO at index {index}: ch={wp['channel']}, pwm={wp['pwm']}")
                else:  # MAV_CMD_NAV_WAYPOINT
                    new_waypoint.frame = 3
                    new_waypoint.command = 16
                    new_waypoint.param1 = float(3)  # Hold time
                    new_waypoint.param2 = 0.0
                    new_waypoint.param3 = 0.0
                    new_waypoint.param4 = float('nan')
                    new_waypoint.x_lat = float(wp['lat'])
                    new_waypoint.y_long = float(wp['lon'])
                    new_waypoint.z_alt = float(wp['alt'])
                    self.get_logger().info(f"Inserting NAV_WAYPOINT at index {index}: lat={wp['lat']}, lon={wp['lon']}, alt={wp['alt']}")

                self.waypoint_list.waypoints.insert(index, new_waypoint)

            self.push_waypoints()
            self.get_logger().info(f"Successfully pushed {len(wp_list)} mission items")
            return True

        except Exception as e:
            self.get_logger().error(f"Error inserting waypoints: {str(e)}")
            return False
    
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
                    self.send_ack(f"Object waypoint reached. Holding for {int(wp.param1)} seconds.")
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
    
    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {text}")

    def handle_wp_req(self, request, response):
        """Handle AddWaypoint service request (NAV_WAYPOINT or DO_SET_SERVO)"""
        command = request.command if request.command != 0 else 16

        response = AddWaypoint.Response()
        try:
            if command == 183:
                self.get_logger().info(f"Received DO_SET_SERVO request: ch={request.channel}, pwm={request.pwm}, index={request.index}")
                wp_list = [{"command": 183, "channel": request.channel, "pwm": request.pwm, "index": request.index}]
            else:
                self.get_logger().info(f"Received NAV_WAYPOINT request: lat={request.latitude}, lon={request.longitude}, alt={request.altitude}, index={request.index}")
                wp_list = [{"command": 16, "lat": request.latitude, "lon": request.longitude, "alt": request.altitude, "index": request.index}]

            self.insert_new_waypoint(wp_list)
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
        self.send_ack(message)
        self.pull_waypoints()
        try:
            rclpy.spin(self)
        except Exception:
            import traceback
            self.get_logger().fatal(
                'waypoint_manager crashed: ' + traceback.format_exc())
            raise
        
def main():
    rclpy.init()
    manager = WaypointManager()
    manager.main()
