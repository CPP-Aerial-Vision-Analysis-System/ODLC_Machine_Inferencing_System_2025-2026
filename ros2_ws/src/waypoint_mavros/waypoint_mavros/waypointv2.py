#!/usr/bin/env python3

"""
Improved Waypoint Manager Node for ROS2
Handles waypoint management with MAVROS integration

Fixes:
- Parameters are properly retrieved and used
- Added validation using mission parameters
- Fixed bug in clear_waypoints (wrong client name)
- Fixed send_status initialization issues
- Improved error handling
- Better async service call handling
"""

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
        self.status_pub = self.create_publisher(StatusText, "/mavros/statustext/send", 10)

        # MAVROS Clients
        self.waypoint_pull = self.create_client(WaypointPull, "/mavros/mission/pull")
        self.waypoint_push = self.create_client(WaypointPush, "/mavros/mission/push")
        self.waypoint_clear = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")

        # Custom Services
        self.add_srv = self.create_service(AddWaypoint, "/addWaypoint", self.handle_wp_req)
        self.del_srv = self.create_service(DelWaypoint, "/delWaypoint", self.handle_wp_del_req)
        self.update_srv = self.create_service(UpdateMission, "/updateMission", self.handle_update_mission)

        # Declare mission parameters
        self.declare_parameter('num_waypoints', 0)
        self.declare_parameter('takeoff_index', -1)
        self.declare_parameter('rtl_index', -1)
        self.declare_parameter('next_after_takeoff', -1)
        self.declare_parameter('last_before_rtl', -1)
        
        # Retrieve and store parameter values
        self.num_waypoints = self.get_parameter('num_waypoints').value
        self.takeoff_index = self.get_parameter('takeoff_index').value
        self.rtl_index = self.get_parameter('rtl_index').value
        self.next_after_takeoff = self.get_parameter('next_after_takeoff').value
        self.last_before_rtl = self.get_parameter('last_before_rtl').value
        
        self.get_logger().info(f"Mission parameters loaded:")
        self.get_logger().info(f"  num_waypoints: {self.num_waypoints}")
        self.get_logger().info(f"  takeoff_index: {self.takeoff_index}")
        self.get_logger().info(f"  rtl_index: {self.rtl_index}")
        self.get_logger().info(f"  next_after_takeoff: {self.next_after_takeoff}")
        self.get_logger().info(f"  last_before_rtl: {self.last_before_rtl}")

        # Initialize status throttling variables
        self.last_status_time = 0.0
        self.status_interval = 1.0  # Minimum seconds between status messages when throttled

    def state_callback(self, msg):
        """Callback function for state updates."""
        self.connected = msg.connected
    
    def waypoints_list(self, data):
        """Receives and stores waypoints list from MAVROS"""
        self.waypoint_list = data
        self.get_logger().info(f"Received waypoint list with {len(self.waypoint_list.waypoints)} waypoints")
        for i, wp in enumerate(self.waypoint_list.waypoints):
            self.get_logger().info(
                f"Waypoint {i}: Lat: {wp.x_lat:.6f}, Lon: {wp.y_long:.6f}, Alt: {wp.z_alt:.2f}, "
                f"Command: {wp.command}, Frame: {wp.frame}"
            )
    
    def push_waypoints(self):
        """Push waypoints to the drone"""
        if not self.waypoint_push.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Waypoint push service not available")
            return False
        
        try:
            waypoint_push_request = WaypointPush.Request()
            waypoint_push_request.start_index = 0
            waypoint_push_request.waypoints = self.waypoint_list.waypoints
            
            future = self.waypoint_push.call_async(waypoint_push_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Successfully pushed {len(self.waypoint_list.waypoints)} waypoints to drone")
                    return True
                else:
                    self.get_logger().error(f"Failed to push waypoints: service returned failure")
                    return False
            else:
                self.get_logger().error("Waypoint push request timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Error pushing waypoints: {e}")
            return False
    
    def pull_waypoints(self):
        """Update waypoint list from the drone."""
        if not self.waypoint_pull.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Waypoint pull service not available")
            return False
        
        try:
            future = self.waypoint_pull.call_async(WaypointPull.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(f"Successfully pulled {response.wp_received} waypoints from drone")
                    # Note: The actual waypoint list comes via the waypoints_list callback
                    return True
                else:
                    self.get_logger().error("Failed to pull waypoints: service returned failure")
                    return False
            else:
                self.get_logger().error("Waypoint pull request timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Error pulling waypoints: {e}")
            return False

    def clear_waypoints(self):
        """Clear all waypoints from the drone."""
        if not self.waypoint_clear.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Waypoint clear service not available")
            return False
        
        try:
            future = self.waypoint_clear.call_async(WaypointClear.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info("Successfully cleared waypoints from drone")
                    self.waypoint_list.waypoints = []  # Clear local list
                    return True
                else:
                    self.get_logger().error("Failed to clear waypoints: service returned failure")
                    return False
            else:
                self.get_logger().error("Waypoint clear request timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Error clearing waypoints: {e}")
            return False
    
    def _validate_waypoint_index(self, index, operation="access"):
        """
        Validate waypoint index using mission parameters
        
        Args:
            index: Waypoint index to validate
            operation: Type of operation ('insert', 'delete', 'access')
            
        Returns:
            tuple: (is_valid, error_message)
        """
        # Check basic bounds
        if index < 0:
            return False, f"Invalid index {index}: cannot be negative"
        
        # Validate against mission structure if parameters are set
        if self.takeoff_index >= 0 and index < self.takeoff_index:
            return False, f"Invalid index {index}: cannot {operation} before takeoff waypoint (index {self.takeoff_index})"
        
        if self.rtl_index >= 0 and index >= self.rtl_index:
            return False, f"Invalid index {index}: cannot {operation} at or after RTL waypoint (index {self.rtl_index})"
        
        # Protect critical waypoints from deletion
        if operation == "delete":
            if index == self.takeoff_index:
                return False, f"Cannot delete takeoff waypoint (index {self.takeoff_index})"
            if index == self.rtl_index:
                return False, f"Cannot delete RTL waypoint (index {self.rtl_index})"
        
        return True, ""
    
    def insert_new_waypoint(self, lat, lon, alt, index):
        """
        Insert new waypoint into the waypoint list and push the updated list
        
        Args:
            lat: Latitude
            lon: Longitude
            alt: Altitude (meters)
            index: Index where to insert waypoint
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Validate index
        is_valid, error_msg = self._validate_waypoint_index(index, "insert")
        if not is_valid:
            self.get_logger().error(f"Cannot insert waypoint: {error_msg}")
            return False
        
        # Validate coordinates
        if not (-90 <= lat <= 90):
            self.get_logger().error(f"Invalid latitude: {lat} (must be between -90 and 90)")
            return False
        if not (-180 <= lon <= 180):
            self.get_logger().error(f"Invalid longitude: {lon} (must be between -180 and 180)")
            return False
        if alt < 0:
            self.get_logger().warn(f"Warning: Altitude is negative: {alt}")
        
        # Pull current waypoint list
        if not self.pull_waypoints():
            self.get_logger().error("Failed to pull waypoints before insertion")
            return False
        
        # Check if index is within bounds of current list
        if index > len(self.waypoint_list.waypoints):
            self.get_logger().warn(
                f"Index {index} is beyond current list length ({len(self.waypoint_list.waypoints)}). "
                f"Will append at end."
            )
            index = len(self.waypoint_list.waypoints)
        
        # Create new waypoint
        new_waypoint = Waypoint()
        new_waypoint.frame = 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        new_waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
        new_waypoint.is_current = False
        new_waypoint.autocontinue = True
        new_waypoint.param1 = float(5)  # Hold time in seconds
        new_waypoint.param2 = float(0)  # Acceptance radius in meters
        new_waypoint.param3 = float(0)  # Pass through waypoint
        new_waypoint.param4 = float('nan')  # Yaw angle
        new_waypoint.x_lat = float(lat)
        new_waypoint.y_long = float(lon)
        new_waypoint.z_alt = float(alt)
        
        # Insert waypoint
        self.get_logger().info(f"Inserting waypoint at index {index}: Lat: {lat:.6f}, Lon: {lon:.6f}, Alt: {alt:.2f}")
        self.waypoint_list.waypoints.insert(index, new_waypoint)
        
        # Push updated waypoint list
        if self.push_waypoints():
            self.get_logger().info("Waypoint inserted and pushed successfully")
            return True
        else:
            self.get_logger().error("Failed to push waypoints after insertion")
            return False
    
    def delete_waypoint(self, index):
        """
        Delete waypoint from the waypoint list and push the updated list
        
        Args:
            index: Index of waypoint to delete
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Validate index
        is_valid, error_msg = self._validate_waypoint_index(index, "delete")
        if not is_valid:
            self.get_logger().error(f"Cannot delete waypoint: {error_msg}")
            return False
        
        # Pull current waypoint list
        if not self.pull_waypoints():
            self.get_logger().error("Failed to pull waypoints before deletion")
            return False
        
        # Check bounds
        if index < 0 or index >= len(self.waypoint_list.waypoints):
            self.get_logger().error(f"Index {index} out of range [0, {len(self.waypoint_list.waypoints)})")
            return False
        
        # Get waypoint info for logging
        wp = self.waypoint_list.waypoints[index]
        self.get_logger().info(
            f"Deleting waypoint {index}: Lat: {wp.x_lat:.6f}, Lon: {wp.y_long:.6f}, Alt: {wp.z_alt:.2f}"
        )
        
        # Delete waypoint
        del self.waypoint_list.waypoints[index]
        
        # Push updated waypoint list
        if self.push_waypoints():
            self.get_logger().info("Waypoint deleted and pushed successfully")
            return True
        else:
            self.get_logger().error("Failed to push waypoints after deletion")
            return False
    
    def waypoint_reached_cb(self, msg):
        """Callback when a waypoint is reached."""
        #talks to waypoint_reached_cb in object_detection_sahi.py
        self.waypoint_reached = msg.wp_seq
        self.get_logger().info(f"Waypoint {msg.wp_seq} reached")

        if self.waypoint_reached < len(self.waypoint_list.waypoints):
            wp = self.waypoint_list.waypoints[self.waypoint_reached]
            if int(wp.param1) > 0:
                if int(wp.command) == 16:  # MAV_CMD_NAV_WAYPOINT
                    self.get_logger().info(f"Object waypoint {msg.wp_seq} reached (hold time: {wp.param1}s)")
                else:
                    self.get_logger().info(f"Loiter finished at waypoint {msg.wp_seq}. Continuing to next waypoint.")
    
    def change_mode(self, mode):
        """
        Change flight mode of the drone.
        
        Args:
            mode: Flight mode string (e.g., "AUTO", "MANUAL", "GUIDED")
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.set_mode.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        self.get_logger().info(f"Changing mode to {mode}")
        try:
            request = SetMode.Request()
            request.custom_mode = mode
            
            future = self.set_mode.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.mode_sent:
                    self.get_logger().info(f"Mode changed to {mode} successfully")
                    return True
                else:
                    self.get_logger().error(f"Failed to change mode to {mode}")
                    return False
            else:
                self.get_logger().error("Set mode request timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Error changing mode: {e}")
            return False
    
    def send_status(self, text, throttle=False):
        """
        Send status message to the drone.
        
        Args:
            text: Status message text
            throttle: If True, throttle messages based on status_interval
        """
        now = time.time()
        if not throttle or (now - self.last_status_time > self.status_interval):
            status_msg = StatusText()
            status_msg.severity = 6  # MAV_SEVERITY_NOTICE
            status_msg.text = text
            self.status_pub.publish(status_msg)
            self.last_status_time = now
            self.get_logger().debug(f"Status sent: {text}")
    
    def handle_wp_req(self, request, response):
        """Handle AddWaypoint service request"""
        self.get_logger().info(
            f"Received AddWaypoint request: lat={request.latitude}, lon={request.longitude}, "
            f"alt={request.altitude}, index={request.index}"
        )

        try:
            success = self.insert_new_waypoint(
                request.latitude, 
                request.longitude, 
                request.altitude, 
                request.index
            )
            response.success = success
            if not success:
                self.get_logger().error("AddWaypoint service failed")
        except Exception as e:
            self.get_logger().error(f"Failed to add waypoint: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
        return response

    def handle_wp_del_req(self, request, response):
        """Handle DelWaypoint service request"""
        self.get_logger().info(f"Received DelWaypoint request: index={request.index}")

        try:
            success = self.delete_waypoint(request.index)
            response.success = success
            if not success:
                self.get_logger().error("DelWaypoint service failed")
        except Exception as e:
            self.get_logger().error(f"Failed to delete waypoint: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            response.success = False
        return response

    def handle_update_mission(self, request, response):
        """
        Handle UpdateMission service request.
        This can be used to update mission parameters dynamically.
        """
        self.get_logger().info("Received UpdateMission request")
        try:
            # TODO: Implement mission update logic
            # This could update parameters like takeoff_index, rtl_index, etc.
            # For now, just acknowledge the request
            
            response = UpdateMission.Response()
            response.success = True
            self.get_logger().info("UpdateMission acknowledged (not yet fully implemented)")
        except Exception as e:
            self.get_logger().error(f"Failed to update mission: {e}")
            response = UpdateMission.Response()
            response.success = False
        return response
    
    def main(self):
        """Main execution loop"""
        self.get_logger().info("Waypoint Manager Node starting...")
        self.get_logger().info("Waiting for connection to FCU...")
        
        while not self.connected:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        message = "Heartbeat established with FCU"
        self.get_logger().info(message)
        self.send_status(message)
        
        self.get_logger().info("Waypoint Manager Node is ready")
        rclpy.spin(self)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        manager = WaypointManager()
        manager.main()
    except KeyboardInterrupt:
        manager.get_logger().info("Shutting down Waypoint Manager...")
    except Exception as e:
        print(f"Fatal error in Waypoint Manager: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'manager' in locals():
            manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

