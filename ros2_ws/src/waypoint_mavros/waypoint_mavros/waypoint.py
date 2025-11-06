import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode, WaypointReached, State, StatusText
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, SetMode, WaypointSetCurrent

from interfaces.srv import AddWaypoint, DelWaypoint, UpdateMission

MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20

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
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_list_cb, 10)
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
        self.declare_parameter('num_waypoints', 0)
        self.declare_parameter('takeoff_index', -1)
        self.declare_parameter('rtl_index', -1)
        self.declare_parameter('next_after_takeoff', -1)
        self.declare_parameter('last_before_rtl', -1)

        self.set_current = self.create_client(WaypointSetCurrent, "/mavros/mission/set_current")
        while not self.set_current.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Set current service not available, waiting ...")

    def set_current_waypoint(self, index):
        self.get_logger().info(f"Requesting set_current to index={index}")
        try:
            # prepare request
            req = WaypointSetCurrent.Request()
            req.wp_seq = int(index)

            # call service
            future = self.set_current.call_async(req)

            # wait with timeout so we don't block indefinitely
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            if not future.done():
                self.get_logger().warn("set_current service call timed out (no response)")
                return False

            # inspect result safely
            try:
                resp = future.result()
            except Exception as e:
                self.get_logger().error(f"set_current future raised exception: {e}")
                return False

            if resp is None:
                self.get_logger().warn("set_current returned None response")
                return False

            # Log full response for debugging
            self.get_logger().info(f"set_current response: {resp}")

            if getattr(resp, 'success', False):
                self.get_logger().info(f"Set current mission index to {index}")
                return True
            else:
                self.get_logger().warn(f"Failed to set current waypoint (success=False). Response: {resp}")
                return False

        except Exception as e:
            self.get_logger().error(f"Exception while calling set_current: {e}")
            return False

       
    def state_callback(self, msg):
        """Callback function for state updates."""
        self.connected = msg.connected
    
    def waypoints_list_cb(self, msg: WaypointList):
        """Callback to store and log the recieved waypoints"""
        self.waypoint_list = msg
        self.get_logger().info(f"Received {len(msg.waypoints)} waypoints")

        # reset indices
        self.reset_indices()

        for i, wp in enumerate(self.waypoint_list.waypoints):
            self.get_logger().info(f"Waypoint {i}: Lat: {wp.x_lat}, Lon: {wp.y_long}, Alt: {wp.z_alt}")
            
            if wp.command == MAV_CMD_NAV_TAKEOFF:
                self.takeoff_index = i
                if i + 1 < len(self.waypoint_list.waypoints):
                    self.next_after_takeoff = i + 1

            if wp.command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
                self.rtl_index = i
                if i - 1 > 0:
                    self.last_before_rtl = i - 1
        
        self.get_logger().info(f"Takeoff Index: {self.takeoff_index}, Next After Takeoff: {self.next_after_takeoff}, Last Before RTL: {self.last_before_rtl}, RTL Index: {self.rtl_index}")
        self.set_parameters([rclpy.parameter.Parameter('num_waypoints', rclpy.Parameter.Type.INTEGER, len(self.waypoint_list.waypoints))])
        self.set_parameters([rclpy.parameter.Parameter('takeoff_index', rclpy.Parameter.Type.INTEGER, self.takeoff_index)])
        self.set_parameters([rclpy.parameter.Parameter('next_after_takeoff', rclpy.Parameter.Type.INTEGER, self.next_after_takeoff)])
        self.set_parameters([rclpy.parameter.Parameter('rtl_index', rclpy.Parameter.Type.INTEGER, self.rtl_index)])
        self.set_parameters([rclpy.parameter.Parameter('last_before_rtl', rclpy.Parameter.Type.INTEGER, self.last_before_rtl)])
        
        
    def reset_indices(self):
        self.num_waypoints = 0
        self.takeoff_index = -1
        self.rtl_index = -1
        self.next_after_takeoff = -1
        self.last_before_rtl = -1

    def push_waypoints(self):
        """Push waypoints to the drone"""
        while not self.waypoint_push.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint push service...")
        
        try:
            waypoint_push_request = WaypointPush.Request()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            future = self.waypoint_push.call_async(waypoint_push_request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Waypoints pushed successfully")
                    return True
            else:
                self.get_logger().error("Failed to push waypoints")
                return False
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False
    
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
    
    # def insert_new_waypoint(self, lat, lon, alt, index):
    def insert_new_waypoint(self, wp_list):
        """Insert new waypoint into the waypoint list and push the updated list"""
        try:
            # Check if we're connected to FCU
            if not self.connected:
                self.get_logger().error("Not connected to FCU. Cannot insert waypoint.")
                return False

            for wp in wp_list:
                self.get_logger().info(f"Inserting new waypoint at index {wp.index}: Lat: {wp.lat}, Lon: {wp.lon}, Alt: {wp.alt}")
                
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
                self.waypoint_list.waypoints.insert(wp.index, new_waypoint)

            # Wait for push service
            # if not self.waypoint_push.wait_for_service(timeout_sec=5.0):
            #     self.get_logger().error("Waypoint push service not available")
            #     return False

            # Add waypoint to list
            # self.waypoint_list.waypoints.insert(index, new_waypoint)
            
            # Create and send push request
            push_req = WaypointPush.Request()
            push_req.start_index = 0
            push_req.waypoints = self.waypoint_list.waypoints
            
            push_future = self.waypoint_push.call_async(push_req)

            # We'll wait up to timeout seconds for either the service response
            # or for the WaypointList subscription to show the inserted waypoint.
            self.set_current_waypoint(wp_list[0].index)
            timeout = 8.0
            start_time = self.get_clock().now()
            initial_len = len(self.waypoint_list.waypoints)

            while (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
                # If service finished, inspect result
                if push_future.done():
                    try:
                        resp = push_future.result()
                        if resp is not None and getattr(resp, 'success', False):
                            self.get_logger().info("Waypoint pushed (service replied success=True)")
                            return True
                        # service replied but marked failure; fallthrough to check via subscription
                    except Exception as e:
                        self.get_logger().warn(f"Push service raised exception: {e}")

                # Check subscription-updated waypoint list for insertion
                if len(self.waypoint_list.waypoints) > initial_len:
                    # Simple sanity check: compare the waypoint at the requested index
                    try:
                        wp = self.waypoint_list.waypoints[index]
                        if abs(wp.x_lat - float(lat)) < 1e-6 and abs(wp.y_long - float(lon)) < 1e-6:
                            self.get_logger().info("Waypoint appears in WaypointList (subscription) — insertion succeeded")
                            return True
                    except Exception:
                        # index may be out of range while list is updating; ignore and continue
                        pass

                rclpy.spin_once(self, timeout_sec=0.1)

            # Timeout reached: check one last time
            if push_future.done():
                try:
                    resp = push_future.result()
                    if resp is not None and getattr(resp, 'success', False):
                        self.get_logger().info("Waypoint pushed (service replied success=True)")
                        return True
                except Exception as e:
                    self.get_logger().warn(f"Push service raised exception at final check: {e}")

            # As a last attempt, look for the waypoint in the list
            for wp in self.waypoint_list.waypoints:
                if abs(wp.x_lat - float(lat)) < 1e-6 and abs(wp.y_long - float(lon)) < 1e-6:
                    self.get_logger().info("Waypoint present in WaypointList after timeout — treating as success")
                    return True

            self.get_logger().error("Push request timed out and waypoint not found in WaypointList")
            return False

        except Exception as e:
            self.get_logger().error(f"Error in insert_new_waypoint: {str(e)}")
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
        self.get_logger().info(f"Processing AddWaypoint request: lat={request.latitude}, lon={request.longitude}, alt={request.altitude}, index={request.index}")
       
        try:
            # Input validation
            if not all(isinstance(x, (int, float)) for x in [request.latitude, request.longitude, request.altitude, request.index]):
                raise ValueError("Invalid input types")
                
            if not (-90 <= request.latitude <= 90 and -180 <= request.longitude <= 180):
                raise ValueError("Invalid latitude or longitude values")
                
            success = self.insert_new_waypoint(request.latitude, request.longitude, request.altitude, request.index)
            response.success = success
            
            # Let the caller know the result through the response
            # but don't log an error if we succeeded in adding the waypoint
            if not success and len(self.waypoint_list.waypoints) == 0:
                self.get_logger().warn("Waypoint may not have been inserted properly")
                
        except Exception as e:
            self.get_logger().error(f"Error processing waypoint request: {str(e)}")
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
        
        # Wait for connection with timeout
        start_time = self.get_clock().now()
        while not self.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > 10.0:  # 10 second timeout
                self.get_logger().error("Failed to establish connection with FCU")
                return
        
        message = f"Heartbeat established"
        self.get_logger().info(message)
        self.send_status(message)
        
        # Initial waypoint pull
        self.pull_waypoints()
        
        # Main loop
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
def main():
    rclpy.init()
    try:
        manager = WaypointManager()
        manager.main()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()