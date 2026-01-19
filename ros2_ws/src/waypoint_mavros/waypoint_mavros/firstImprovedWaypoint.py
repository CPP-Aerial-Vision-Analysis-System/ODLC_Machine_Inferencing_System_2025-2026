import rclpy
import time
from rclpy.node import Node
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode, WaypointReached, State, StatusText
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, SetMode

from interfaces.srv import AddWaypoint, DelWaypoint, UpdateMission

# Constants for MAV commands
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20

# Constants for service timeouts
SERVICE_TIMEOUT = 1.0
MAX_SERVICE_WAIT_RETRIES = 10


class WaypointManager(Node):
    
    def __init__(self):
        super().__init__("waypoint_manager")

        self.connected = False 
        self.waypoint_reached = 0 
        self.waypoint_list = WaypointList()

        self.num_waypoints = 0
        self.takeoff_index = -1
        self.rtl_index = -1
        self.next_after_takeoff = -1
        self.last_before_rtl = -1
        
        # Cache for parameter change detection
        # Without caching, the parameters can only change when mission is changed. 
        self._cached_params = {
            'num_waypoints': 0,
            'takeoff_index': -1,
            'rtl_index': -1,
            'next_after_takeoff': -1,
            'last_before_rtl': -1
        }

        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_mavros_clients()
        self._setup_custom_services()
        self._initialize_parameters()
        
        self.get_logger().info("WaypointManager initialized successfully")

    def _setup_subscriptions(self):
        self.create_subscription(State, "/mavros/state", self.state_callback, 10)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.waypoint_reached_cb, 10)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_list_cb, 10)

    def _setup_publishers(self):
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

    def _setup_mavros_clients(self):
        self.waypoint_pull = self.create_client(WaypointPull, "/mavros/mission/pull")
        self.waypoint_push = self.create_client(WaypointPush, "/mavros/mission/push")
        self.waypoint_clear = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")

    def _setup_custom_services(self):
        self.add_srv = self.create_service(AddWaypoint, "/addWaypoint", self.handle_wp_req)
        self.del_srv = self.create_service(DelWaypoint, "/delWaypoint", self.handle_wp_del_req)
        self.update_srv = self.create_service(UpdateMission, "/updateMission", self.handle_update_mission)

    def _initialize_parameters(self):
        # This might seem redundant, but without this, these are not ROS parameters, just python variables created in the constructor. You can't list them, cant override through a launch file, other nodes cant read them and dynamic reconfiguration wont work.
        self.declare_parameter('num_waypoints', 0)
        self.declare_parameter('takeoff_index', -1)
        self.declare_parameter('rtl_index', -1)
        self.declare_parameter('next_after_takeoff', -1)
        self.declare_parameter('last_before_rtl', -1)

    def _update_parameters(self):
        current_params = {
            'num_waypoints': self.num_waypoints,
            'takeoff_index': self.takeoff_index,
            'rtl_index': self.rtl_index,
            'next_after_takeoff': self.next_after_takeoff,
            'last_before_rtl': self.last_before_rtl
        }
        
        # Only update if something changed
        if current_params != self._cached_params:
            param_updates = [
                rclpy.parameter.Parameter('num_waypoints', rclpy.Parameter.Type.INTEGER, self.num_waypoints),
                rclpy.parameter.Parameter('takeoff_index', rclpy.Parameter.Type.INTEGER, self.takeoff_index),
                rclpy.parameter.Parameter('rtl_index', rclpy.Parameter.Type.INTEGER, self.rtl_index),
                rclpy.parameter.Parameter('next_after_takeoff', rclpy.Parameter.Type.INTEGER, self.next_after_takeoff),
                rclpy.parameter.Parameter('last_before_rtl', rclpy.Parameter.Type.INTEGER, self.last_before_rtl),
            ]
            self.set_parameters(param_updates)
            self._cached_params = current_params.copy()

    def state_callback(self, msg):
        self.connected = msg.connected
    
    # things to ask what are special waypoints ? what are useful derived indices ? what are important indeces ? 

    def waypoints_list_cb(self, data):        
        self.waypoint_list = data
        
        # recreating them just in case
        self.takeoff_index = -1
        self.rtl_index = -1
        self.next_after_takeoff = -1
        self.last_before_rtl = -1
        
        # Analyze waypoint list to find special waypoints
        for i, wp in enumerate(self.waypoint_list.waypoints):       
            if wp.command == MAV_CMD_NAV_TAKEOFF:
                self.takeoff_index = i
                if i + 1 < len(self.waypoint_list.waypoints):
                    self.next_after_takeoff = i + 1

            if wp.command == MAV_CMD_NAV_RETURN_TO_LAUNCH:
                self.rtl_index = i
                if i > 0:  # Changed from i - 1 > 0 to i > 0 (in a case of 2 wp, this would be invalid)
                    self.last_before_rtl = i - 1

        # Update waypoint count and params
        self.num_waypoints = len(self.waypoint_list.waypoints)
        self._update_parameters()
        
        self.get_logger().info(
            f"Mission Structure - Waypoints: {self.num_waypoints}, "
            f"Takeoff: {self.takeoff_index}, Next after takeoff: {self.next_after_takeoff}, "
            f"Last before RTL: {self.last_before_rtl}, RTL: {self.rtl_index}"
        )
    
    def reset_indices(self):
        self.num_waypoints = 0
        self.takeoff_index = -1
        self.rtl_index = -1
        self.next_after_takeoff = -1
        self.last_before_rtl = -1
        
        # Update parameters to reflect reset
        self._update_parameters()

    def _wait_for_service(self, client, service_name, max_retries=MAX_SERVICE_WAIT_RETRIES):
        retry_count = 0
        while not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            retry_count += 1
            if retry_count >= max_retries:
                self.get_logger().error(f"Service {service_name} not available after {max_retries} attempts")
                return False
            self.get_logger().info(f"Waiting for {service_name} service... (attempt {retry_count}/{max_retries})")
        return True

    def push_waypoints(self, callback=None):
        if not self._wait_for_service(self.waypoint_push, "waypoint push"):
            if callback:
                callback(False)
            return None
        
        try:
            waypoint_push_request = WaypointPush.Request()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            
            future = self.waypoint_push.call_async(waypoint_push_request) # a placeholder that will get a value later
            
            def done_callback(fut):
                try:
                    result = fut.result()
                    success = result is not None and result.success
                    if success:
                        self.get_logger().info(f"Successfully pushed {len(self.waypoint_list.waypoints)} waypoints")
                    else:
                        self.get_logger().error("Failed to push waypoints")
                    if callback:
                        callback(success)
                except Exception as e:
                    self.get_logger().error(f"Exception in push callback: {e}")
                    if callback:
                        callback(False)
            
            future.add_done_callback(done_callback)
            return future
                
        except Exception as e:
            self.get_logger().error(f"Exception during waypoint push: {e}")
            if callback:
                callback(False)
            return None
    
    def pull_waypoints(self, callback=None):
        if not self._wait_for_service(self.waypoint_pull, "waypoint pull"):
            if callback:
                callback(False)
            return None
        
        try:
            future = self.waypoint_pull.call_async(WaypointPull.Request())
            
            def done_callback(fut):
                try:
                    response = fut.result()
                    success = response and response.success
                    if success:
                        self.get_logger().info(f"Successfully pulled {response.wp_received} waypoints from autopilot")
                    else:
                        self.get_logger().error("Failed to pull waypoints")
                    if callback:
                        callback(success)
                except Exception as e:
                    self.get_logger().error(f"Exception in pull callback: {e}")
                    if callback:
                        callback(False)
            
            future.add_done_callback(done_callback)
            return future
                
        except Exception as e:
            self.get_logger().error(f"Exception during waypoint pull: {e}")
            if callback:
                callback(False)
            return None

    def clear_waypoints(self, callback=None):
        if not self._wait_for_service(self.waypoint_clear, "waypoint clear"):
            if callback:
                callback(False)
            return None
        
        try:
            future = self.waypoint_clear.call_async(WaypointClear.Request())
            
            def done_callback(fut):
                try:
                    response = fut.result()
                    success = response and response.success
                    if success:
                        self.get_logger().info("Successfully cleared all waypoints")
                        self.reset_indices()
                    else:
                        self.get_logger().error("Failed to clear waypoints")
                    if callback:
                        callback(success)
                except Exception as e:
                    self.get_logger().error(f"Exception in clear callback: {e}")
                    if callback:
                        callback(False)
            
            future.add_done_callback(done_callback)
            return future
                
        except Exception as e:
            self.get_logger().error(f"Exception during waypoint clear: {e}")
            if callback:
                callback(False)
            return None
    
    def _create_waypoint(self, lat, lon, alt):
        new_waypoint = Waypoint()
        new_waypoint.frame = 3  # Global relative altitude (MAV_FRAME_GLOBAL_RELATIVE_ALT)
        new_waypoint.command = MAV_CMD_NAV_WAYPOINT
        new_waypoint.is_current = False
        new_waypoint.autocontinue = True
        new_waypoint.param1 = float(15)  # Hold time in seconds
        new_waypoint.param2 = float(0)   # Acceptance radius in meters
        new_waypoint.param3 = float(0)   # Pass through waypoint
        new_waypoint.param4 = float('nan')  # Yaw angle
        new_waypoint.x_lat = float(lat)
        new_waypoint.y_long = float(lon)
        new_waypoint.z_alt = float(alt)
        return new_waypoint

    def insert_new_waypoint(self, wp_list, callback=None):
        try:
            if not wp_list or len(wp_list) == 0:
                self.get_logger().warn("No waypoints to insert")
                if callback:
                    callback(False)
                return
            
            # Validate all waypoints before starting
            for wp in wp_list:
                if not all(key in wp for key in ['lat', 'lon', 'alt', 'index']):
                    self.get_logger().error("Waypoint missing required fields")
                    if callback:
                        callback(False)
                    return
            
            self.get_logger().info("Step 1/3: Pulling fresh waypoint list from autopilot")
            
            # Step 1: Pull fresh waypoints
            def on_pull_complete(pull_success):
                if not pull_success:
                    self.get_logger().error("Failed to pull waypoints before insert")
                    if callback:
                        callback(False)
                    return
                
                # Step 2: Validate and modify
                try:
                    for wp in wp_list:
                        if wp['index'] > len(self.waypoint_list.waypoints):
                            self.get_logger().error(f"Index {wp['index']} is out of range")
                            if callback:
                                callback(False)
                            return
                    
                    # Store original for rollback
                    original_waypoints = self.waypoint_list.waypoints.copy()
                    
                    # Sort and insert
                    sorted_wp_list = sorted(wp_list, key=lambda x: x['index'], reverse=True)
                    
                    for wp in sorted_wp_list:
                        self.get_logger().info(
                            f"Inserting waypoint at index {wp['index']}: "
                            f"Lat:{wp['lat']:.6f}, Lon:{wp['lon']:.6f}, Alt:{wp['alt']:.1f}m"
                        )
                        new_waypoint = self._create_waypoint(wp['lat'], wp['lon'], wp['alt'])
                        self.waypoint_list.waypoints.insert(wp['index'], new_waypoint)
                    
                    self.get_logger().info("Step 2/3: Modified waypoint list")
                    self.get_logger().info("Step 3/3: Pushing updated list to autopilot")
                    
                    # Step 3: Push modified list
                    def on_push_complete(push_success):
                        if push_success:
                            self.get_logger().info(f"Successfully inserted {len(wp_list)} new waypoints")
                        else:
                            self.get_logger().warn("Push failed, reverting waypoint list")
                            self.waypoint_list.waypoints = original_waypoints
                        
                        if callback:
                            callback(push_success)
                    
                    self.push_waypoints(callback=on_push_complete)
                    
                except Exception as e:
                    self.get_logger().error(f"Error during modification: {str(e)}")
                    if callback:
                        callback(False)
            
            self.pull_waypoints(callback=on_pull_complete)
                
        except Exception as e:
            self.get_logger().error(f"Error in insert_new_waypoint: {str(e)}")
            if callback:
                callback(False)
    
    def delete_waypoint(self, index, callback=None):
        try:
            self.get_logger().info(f"Step 1/3: Pulling fresh waypoint list before deletion")
            
            # Step 1: Pull fresh waypoints
            def on_pull_complete(pull_success):
                if not pull_success:
                    self.get_logger().error("Failed to pull waypoints before delete")
                    if callback:
                        callback(False)
                    return
                
                # Step 2: Validate and modify
                try:
                    if not (0 <= index < len(self.waypoint_list.waypoints)):
                        self.get_logger().error(f"Index {index} out of range (0-{len(self.waypoint_list.waypoints)-1})")
                        if callback:
                            callback(False)
                        return
                    
                    # Store original for rollback
                    original_waypoints = self.waypoint_list.waypoints.copy()
                    
                    # Delete waypoint
                    del self.waypoint_list.waypoints[index]
                    self.get_logger().info(f"Step 2/3: Deleted waypoint at index {index}")
                    self.get_logger().info("Step 3/3: Pushing updated list to autopilot")
                    
                    # Step 3: Push modified list
                    def on_push_complete(push_success):
                        if push_success:
                            self.get_logger().info("Waypoint deleted and mission updated successfully")
                        else:
                            self.get_logger().warn("Push failed, reverting waypoint list")
                            self.waypoint_list.waypoints = original_waypoints
                        
                        if callback:
                            callback(push_success)
                    
                    self.push_waypoints(callback=on_push_complete)
                    
                except Exception as e:
                    self.get_logger().error(f"Error during deletion: {str(e)}")
                    if callback:
                        callback(False)
            
            self.pull_waypoints(callback=on_pull_complete)
                
        except Exception as e:
            self.get_logger().error(f"Error in delete_waypoint: {str(e)}")
            if callback:
                callback(False)
    
    # set_current_waypoint removed - it was a dangerous stub
    # When needed, implement WaypointSetCurrent service properly
    # For now, autopilot manages current waypoint automatically
    
    def waypoint_reached_cb(self, msg):
        self.waypoint_reached = msg.wp_seq
        self.get_logger().info(f"Waypoint {msg.wp_seq} reached")

        # Check if we have a valid waypoint at this index
        if self.waypoint_reached < len(self.waypoint_list.waypoints):
            wp = self.waypoint_list.waypoints[self.waypoint_reached]
            
            # Check for hold time (param1)
            hold_time = int(wp.param1)
            if hold_time > 0 and wp.command == MAV_CMD_NAV_WAYPOINT:
                self.get_logger().info(f"Waypoint with hold time reached. Holding for {hold_time} seconds")
                self.send_ack(f"Object waypoint reached. Holding for {hold_time} seconds")
    
    def change_mode(self, mode, callback=None):
        if not self._wait_for_service(self.set_mode, "set mode"):
            if callback:
                callback(False)
            return None
            
        try:
            self.get_logger().info(f"Changing mode to {mode}")
            
            req = SetMode.Request()
            req.custom_mode = mode
            
            future = self.set_mode.call_async(req)
            
            def done_callback(fut):
                try:
                    response = fut.result()
                    success = response and response.mode_sent
                    if success:
                        self.get_logger().info(f"Mode changed to {mode} successfully")
                    else:
                        self.get_logger().error(f"Failed to change mode to {mode}")
                    if callback:
                        callback(success)
                except Exception as e:
                    self.get_logger().error(f"Exception in mode change callback: {e}")
                    if callback:
                        callback(False)
            
            future.add_done_callback(done_callback)
            return future
                
        except Exception as e:
            self.get_logger().error(f"Exception during mode change: {e}")
            if callback:
                callback(False)
            return None
    
    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO severity level
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status sent: {text}")

    def handle_wp_req(self, request, response):
        self.get_logger().info(
            f"AddWaypoint request - {len(request.latitude)} waypoint(s)"
        )

        try:
            # Validate arrays are same length
            if not (len(request.latitude) == len(request.longitude) == 
                    len(request.altitude) == len(request.index)):
                self.get_logger().error("AddWaypoint array length mismatch")
                response.success = False
                return response
            
            # Build waypoint list from arrays
            wp_list = []
            for i in range(len(request.latitude)):
                wp_list.append({
                    'lat': request.latitude[i],
                    'lon': request.longitude[i],
                    'alt': request.altitude[i],
                    'index': request.index[i]
                })
            
            # Note: We must return response immediately, but operation is async
            # The actual success/failure happens in callback
            # For synchronous service, we need to block here (limitation of ROS2 service pattern)
            # Alternative: make this an action instead of service for true async
            
            self._pending_service_response = response
            
            def on_complete(success):
                # This callback happens after service already returned
                # Log the actual result
                if success:
                    self.get_logger().info("Waypoint insertion completed successfully")
                else:
                    self.get_logger().error("Waypoint insertion failed")
            
            self.insert_new_waypoint(wp_list, callback=on_complete)
            
            # For now, return optimistic response
            # TODO: Consider converting to Action for proper async feedback
            response.success = True
            self.get_logger().warn("Returning optimistic response - actual result in callbacks")
            
        except Exception as e:
            self.get_logger().error(f"Failed to add waypoint: {e}")
            response.success = False
            
        return response

    def handle_wp_del_req(self, request, response):
        self.get_logger().info(f"DelWaypoint request - index={request.index}")

        try:
            def on_complete(success):
                if success:
                    self.get_logger().info("Waypoint deletion completed successfully")
                else:
                    self.get_logger().error("Waypoint deletion failed")
            
            self.delete_waypoint(request.index, callback=on_complete)
            
            # Return optimistic response (actual result in callback)
            response.success = True
            self.get_logger().warn("Returning optimistic response - actual result in callbacks")
            
        except Exception as e:
            self.get_logger().error(f"Failed to delete waypoint: {e}")
            response.success = False
            
        return response

    def handle_update_mission(self, request, response):
        self.get_logger().info("UpdateMission request received")
        
        try:
            def on_complete(success):
                if success:
                    self.get_logger().info("Mission updated successfully")
                else:
                    self.get_logger().error("Failed to update mission")
            
            self.pull_waypoints(callback=on_complete)
            
            # Return optimistic response (actual result in callback)
            response.success = True
            self.get_logger().warn("Returning optimistic response - actual result in callbacks")
                
        except Exception as e:
            self.get_logger().error(f"Failed to update mission: {e}")
            response.success = False
            
        return response
    
    def main(self):
        self.get_logger().info("Waiting for connection to FCU...")
        
        # Wait for FCU connection
        while not self.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        message = "Heartbeat established with FCU"
        self.get_logger().info(message)
        self.send_ack(message)
        
        # Pull initial waypoint list
        self.pull_waypoints()
        
        # Enter main spin loop
        rclpy.spin(self)
        
        
def main():
    rclpy.init()
    manager = WaypointManager()
    
    try:
        manager.main()
    except KeyboardInterrupt:
        manager.get_logger().info("Waypoint manager shutting down...")
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
