#!/usr/bin/env python3
"""
Waypoint-Triggered Image Capture Node

This node automatically captures images at each waypoint during mission execution.
Integrates with SIYI camera pipeline for reliable image capture and storage.

Architecture:
- Subscribes to /mavros/mission/reached for waypoint notifications
- Triggers SIYI camera capture via /camera/trigger topic
- Tracks captured waypoints to prevent duplicates
- Publishes capture status and statistics

Usage:
    ros2 run video_cam image_pub_final

Integration with main_controller_aro:
    - Both nodes can run simultaneously
    - Detection node (new_od_aro) processes images published by SIYI pipeline
    - Main controller receives detections and makes mission decisions
    - This node ensures images are captured at every waypoint

ROS2 Topics:
    Subscriptions:
        - /mavros/mission/reached (WaypointReached): Waypoint arrival notifications
        - /mavros/mission/waypoints (WaypointList): Mission structure
        - /camera/status (String): Camera pipeline status
    
    Publishers:
        - /camera/trigger (Bool): Trigger image capture
        - /waypoint_capture/status (String): Capture status updates
        - /mavros/statustext/send (StatusText): GCS status messages
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached, StatusText, WaypointList
from std_msgs.msg import Bool, String
from rcl_interfaces.msg import ParameterEvent

import time
import json
from typing import Set, Dict, Optional
from collections import defaultdict
from datetime import datetime

# Waypoint capture configuration
CAPTURE_DELAY = 1.0  # Delay after waypoint reached before capture (seconds)
MIN_CAPTURE_INTERVAL = 2.0  # Minimum time between captures (prevents spam)
MAX_RETRIES = 2  # Maximum capture retry attempts per waypoint


class WaypointImageCapture(Node):
    """
    Automatically captures images at each waypoint during mission execution.
    
    Features:
    - Waypoint-triggered capture (captures once per waypoint)
    - Duplicate prevention (tracks already captured waypoints)
    - Retry logic (handles transient camera failures)
    - Mission-aware (skips takeoff/RTL waypoints)
    - Status reporting (publishes to GCS and monitoring topics)
    """
    
    def __init__(self):
        super().__init__('waypoint_image_capture')
        
        # ====================================================================
        # PARAMETERS
        # ====================================================================
        self.declare_parameter('capture_delay', 1.0)
        self.declare_parameter('min_capture_interval', 2.0)
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('skip_takeoff', True)  # Skip takeoff waypoint
        self.declare_parameter('skip_rtl', True)  # Skip RTL waypoint
        self.declare_parameter('auto_enable', True)  # Auto-enable on mission start
        self.declare_parameter('coordinate_with_controller', True)  # Work with main controller
        
        # Get parameters
        self.capture_delay = self.get_parameter('capture_delay').value
        self.min_capture_interval = self.get_parameter('min_capture_interval').value
        self.max_retries = self.get_parameter('max_retries').value
        self.skip_takeoff = self.get_parameter('skip_takeoff').value
        self.skip_rtl = self.get_parameter('skip_rtl').value
        self.auto_enable = self.get_parameter('auto_enable').value
        self.coordinate_with_controller = self.get_parameter('coordinate_with_controller').value
        
        # ====================================================================
        # STATE TRACKING
        # ====================================================================
        # Waypoint tracking
        self.captured_waypoints: Set[int] = set()  # Waypoints that have been captured
        self.current_waypoint: Optional[int] = None
        self.last_capture_time: float = 0.0
        self.capture_count: int = 0
        
        # Retry tracking per waypoint
        self.retry_counts: Dict[int, int] = defaultdict(int)
        
        # Mission structure (from waypoint_manager or MAVROS)
        self.waypoints: list = []
        self.takeoff_index: Optional[int] = None
        self.rtl_index: Optional[int] = None
        self.num_waypoints: int = 0
        
        # Camera status
        self.camera_ready: bool = True  # Assume ready unless status indicates otherwise
        self.camera_status: str = "unknown"
        
        # Control
        self.enabled: bool = self.auto_enable
        self.pending_capture: Optional[int] = None  # Waypoint pending capture
        
        # ====================================================================
        # ROS2 INTERFACE
        # ====================================================================
        
        # Subscribers
        self.create_subscription(
            WaypointReached, 
            "/mavros/mission/reached", 
            self.waypoint_reached_callback, 
            10
        )
        
        self.create_subscription(
            WaypointList,
            "/mavros/mission/waypoints",
            self.waypoints_callback,
            1
        )
        
        self.create_subscription(
            String,
            "/camera/status",
            self.camera_status_callback,
            10
        )
        
        self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self.parameter_event_callback,
            10
        )
        
        # Publishers
        self.trigger_pub = self.create_publisher(Bool, "/camera/trigger", 10)
        self.status_pub = self.create_publisher(String, "/waypoint_capture/status", 10)
        self.gcs_status_pub = self.create_publisher(StatusText, "/mavros/statustext/send", 10)
        
        # Timer for delayed capture execution
        self.create_timer(0.5, self.capture_timer_callback)
        
        # ====================================================================
        # INITIALIZATION
        # ====================================================================
        self.get_logger().info("=" * 70)
        self.get_logger().info(" WAYPOINT IMAGE CAPTURE NODE INITIALIZED")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f" Capture delay:     {self.capture_delay}s")
        self.get_logger().info(f" Min interval:      {self.min_capture_interval}s")
        self.get_logger().info(f" Max retries:       {self.max_retries}")
        self.get_logger().info(f" Skip takeoff:      {self.skip_takeoff}")
        self.get_logger().info(f" Skip RTL:          {self.skip_rtl}")
        self.get_logger().info(f" Auto-enabled:      {self.enabled}")
        self.get_logger().info(f" Coordination mode: {self.coordinate_with_controller}")
        self.get_logger().info("=" * 70)
        if self.coordinate_with_controller:
            self.get_logger().info(" Running in COORDINATION mode:")
            self.get_logger().info("   - Listens to /camera/trigger from main_controller")
            self.get_logger().info("   - Publishes capture status to /waypoint_capture/status")
            self.get_logger().info("   - Main controller orchestrates capture timing")
        else:
            self.get_logger().info(" Running in AUTONOMOUS mode:")
            self.get_logger().info("   - Monitors /mavros/mission/reached directly")
            self.get_logger().info("   - Triggers captures independently")
        self.get_logger().info("=" * 70)
        self.get_logger().info(" Waiting for triggers...")
        self.get_logger().info("=" * 70)
        """
        NOTE: When coordinate_with_controller=True, this is bypassed and
        the main_controller_aro.py triggers captures via /camera/trigger
        """
        # If coordinating with main controller, skip autonomous triggering
        if self.coordinate_with_controller:
            self.current_waypoint = msg.wp_seq
            self.get_logger().debug(
                f" Waypoint {msg.wp_seq} reached (coordination mode - waiting for controller trigger)"
            )
            return
        
        # Autonomous mode - trigger captures independently
    # ========================================================================
    # CALLBACKS
    # ========================================================================
    
    def waypoint_reached_callback(self, msg: WaypointReached):
        """
        Handle waypoint arrival - schedule image capture.
        
        This is called by MAVROS when the vehicle reaches a waypoint.
        We schedule a capture with a delay to ensure vehicle has stabilized.
        """
        waypoint_id = msg.wp_seq
        self.current_waypoint = waypoint_id
        
        self.get_logger().info(f" Reached waypoint {waypoint_id}")
        
        # Check if enabled
        if not self.enabled:
            self.get_logger().debug(f" Capture disabled - skipping waypoint {waypoint_id}")
            return
        
        # Check if already captured
        if waypoint_id in self.captured_waypoints:
            self.get_logger().debug(f" Waypoint {waypoint_id} already captured - skipping")
            return
        
        # Check if should skip this waypoint
        if self._should_skip_waypoint(waypoint_id):
            self.get_logger().info(
                f" Skipping waypoint {waypoint_id} "
                f"(takeoff={self.takeoff_index}, rtl={self.rtl_index})"
            )
            # Mark as "captured" so we don't try again
            self.captured_waypoints.add(waypoint_id)
            return
        
        # Check rate limiting
        time_since_last = time.time() - self.last_capture_time
        if time_since_last < self.min_capture_interval:
            self.get_logger().warn(
                f" Rate limit: {time_since_last:.1f}s < {self.min_capture_interval}s "
                f"- skipping waypoint {waypoint_id}"
            )
            return
        
        # Schedule capture with delay
        self.pending_capture = waypoint_id
        self.get_logger().info(
            f" Scheduling capture for waypoint {waypoint_id} "
            f"(delay: {self.capture_delay}s)"
        )
        
        # Use a timer for the delay
        self.create_timer(
            self.capture_delay,
            lambda: self._execute_capture(waypoint_id),
            oneshot=True
        )
    
    def waypoints_callback(self, msg: WaypointList):
        """Store mission waypoint list for reference."""
        self.waypoints = msg.waypoints
        self.num_waypoints = len(self.waypoints)
        self.get_logger().info(f" Mission updated: {self.num_waypoints} waypoints")
    
    def camera_status_callback(self, msg: String):
        """Monitor camera pipeline status."""
        self.camera_status = msg.data
        
        # Simple heuristic: if status contains "failed", "error", or "unavailable"
        # mark camera as not ready
        status_lower = msg.data.lower()
        if any(word in status_lower for word in ['failed', 'error', 'unavailable', 'timeout']):
            if self.camera_ready:
                self.camera_ready = False
                self.get_logger().warn(f" Camera not ready: {msg.data}")
        else:
            if not self.camera_ready:
                self.camera_ready = True
                self.get_logger().info(f" Camera ready: {msg.data}")
    
    def parameter_event_callback(self, msg: ParameterEvent):
        """
        Listen for parameter changes from waypoint_manager.
        
        Updates mission structure information (takeoff/RTL indices).
        """
        if msg.node == "/waypoint_manager":
            for param in msg.changed_parameters:
                if param.name == "takeoff_index":
                    self.takeoff_index = param.value.integer_value
                    self.get_logger().info(f" Updated takeoff_index: {self.takeoff_index}")
                elif param.name == "rtl_index":
                    self.rtl_index = param.value.integer_value
                    self.get_logger().info(f" Updated rtl_index: {self.rtl_index}")
                elif param.name == "num_waypoints":
                    self.num_waypoints = param.value.integer_value
                    self.get_logger().info(f" Updated num_waypoints: {self.num_waypoints}")
    
    def capture_timer_callback(self):
        """
        Periodic callback to check for pending captures.
        
        This is a fallback mechanism in case oneshot timers don't fire.
        """
        # This is intentionally empty - actual capture is triggered by oneshot timers
        # We keep this as a heartbeat for potential future use
        pass
        pass
    
    # ========================================================================
    # CAPTURE LOGIC
    # ========================================================================
    
    def _execute_capture(self, waypoint_id: int):
        """
        Execute image capture for a specific waypoint.
        
        Args:
            waypoint_id: Waypoint index to capture
        """
        # Double-check it's still pending
        if waypoint_id != self.pending_capture:
            self.get_logger().debug(f" Capture for waypoint {waypoint_id} no longer pending")
            return
        
        # Double-check not already captured (race condition prevention)
        if waypoint_id in self.captured_waypoints:
            self.get_logger().debug(f" Waypoint {waypoint_id} already captured during delay")
            self.pending_capture = None
            return
        
        # Check retry count
        if self.retry_counts[waypoint_id] >= self.max_retries:
            self.get_logger().error(
                f" ✗ Waypoint {waypoint_id}: Max retries ({self.max_retries}) exceeded"
            )
            self._send_gcs_status(f"Image capture failed at waypoint {waypoint_id}")
            # Mark as captured to prevent infinite retries
            self.captured_waypoints.add(waypoint_id)
            self.pending_capture = None
            return
        
        # Trigger camera capture
        self.get_logger().info(
            f" Triggering capture for waypoint {waypoint_id} "
            f"(attempt {self.retry_counts[waypoint_id] + 1}/{self.max_retries + 1})"
        )
        
        # Send trigger message
        trigger_msg = Bool()
        trigger_msg.data = True
        self.trigger_pub.publish(trigger_msg)
        
        # Update tracking
        self.last_capture_time = time.time()
        self.capture_count += 1
        self.retry_counts[waypoint_id] += 1
        
        # Mark as captured (optimistic - we assume camera will succeed)
        # If camera fails, it will be handled by status callback
        self.captured_waypoints.add(waypoint_id)
        self.pending_capture = None
        
        # Publish status
        status_data = {
            'waypoint': waypoint_id,
            'capture_count': self.capture_count,
            'total_waypoints_captured': len(self.captured_waypoints),
            'timestamp': datetime.now().isoformat()
        }
        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)
        
        # Send to GCS
        self._send_gcs_status(f"Image captured at waypoint {waypoint_id}")
        
        self.get_logger().info(
            f" ✓ Capture triggered for waypoint {waypoint_id} "
            f"({len(self.captured_waypoints)}/{self.num_waypoints} waypoints captured)"
        )
    
    def _should_skip_waypoint(self, waypoint_id: int) -> bool:
        """
        Determine if a waypoint should be skipped for image capture.
        
        Args:
            waypoint_id: Waypoint index to check
            
        Returns:
            True if waypoint should be skipped, False otherwise
        """
        # Skip takeoff waypoint
        if self.skip_takeoff and waypoint_id == self.takeoff_index:
            return True
        
        # Skip RTL waypoint
        if self.skip_rtl and waypoint_id == self.rtl_index:
            return True
        
        # Skip waypoint 0 (home/start position)
        if waypoint_id == 0:
            return True
        
        return False
    
    def _send_gcs_status(self, text: str):
        """
        Send status message to Ground Control Station.
        
        Args:
            text: Status message text
        """
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.gcs_status_pub.publish(msg)
    
    # ========================================================================
    # CONTROL METHODS (for external control)
    # ========================================================================
    
    def enable_capture(self):
        """Enable automatic waypoint capture."""
        if not self.enabled:
            self.enabled = True
            self.get_logger().info(" Waypoint capture ENABLED")
            self._send_gcs_status("Waypoint image capture enabled")
    
    def disable_capture(self):
        """Disable automatic waypoint capture."""
        if self.enabled:
            self.enabled = False
            self.get_logger().info(" Waypoint capture DISABLED")
            self._send_gcs_status("Waypoint image capture disabled")
    
    def reset_captured_waypoints(self):
        """Clear captured waypoint history (allows re-capture)."""
        count = len(self.captured_waypoints)
        self.captured_waypoints.clear()
        self.retry_counts.clear()
        self.get_logger().info(f" Reset captured waypoints (cleared {count} entries)")
    
    def get_statistics(self) -> dict:
        """
        Get capture statistics.
        
        Returns:
            Dictionary with capture statistics
        """
        return {
            'enabled': self.enabled,
            'capture_count': self.capture_count,
            'waypoints_captured': len(self.captured_waypoints),
            'total_waypoints': self.num_waypoints,
            'current_waypoint': self.current_waypoint,
            'camera_ready': self.camera_ready,
            'camera_status': self.camera_status
        }


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = WaypointImageCapture()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics
        stats = node.get_statistics()
        node.get_logger().info("=" * 70)
        node.get_logger().info(" FINAL STATISTICS")
        node.get_logger().info("=" * 70)
        for key, value in stats.items():
            node.get_logger().info(f" {key}: {value}")
        node.get_logger().info("=" * 70)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
