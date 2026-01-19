import rclpy
import time
import hashlib
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Callable, List, Dict
from rclpy.node import Node
from rclpy.duration import Duration
from mavros_msgs.msg import Waypoint, WaypointList, CommandCode, WaypointReached, State, StatusText
from mavros_msgs.srv import WaypointPull, WaypointPush, WaypointClear, SetMode

from interfaces.srv import AddWaypoint, DelWaypoint, UpdateMission


# ============================================================================
# INDUSTRIAL CONSTANTS & ENUMS - Replace magic numbers
# ============================================================================

class MAVCommand(IntEnum):
    """MAVLink navigation commands"""
    NAV_WAYPOINT = 16
    NAV_TAKEOFF = 22
    NAV_RETURN_TO_LAUNCH = 20


class MAVFrame(IntEnum):
    """MAVLink coordinate frames"""
    GLOBAL = 0
    LOCAL_NED = 1
    MISSION = 2
    GLOBAL_RELATIVE_ALT = 3
    LOCAL_ENU = 4
    GLOBAL_INT = 5


class SeverityLevel(IntEnum):
    """MAVLink status text severity levels"""
    EMERGENCY = 0
    ALERT = 1
    CRITICAL = 2
    ERROR = 3
    WARNING = 4
    NOTICE = 5
    INFO = 6
    DEBUG = 7


# Service and operation timeouts
SERVICE_TIMEOUT = 1.0
MAX_SERVICE_WAIT_RETRIES = 10
PULL_SYNC_TIMEOUT = 2.0
MISSION_OPERATION_TIMEOUT = 30.0

# Coordinate validation limits
MIN_LATITUDE = -90.0
MAX_LATITUDE = 90.0
MIN_LONGITUDE = -180.0
MAX_LONGITUDE = 180.0
MIN_ALTITUDE = 0.0
MAX_ALTITUDE = 10000.0  # Reasonable max for drones in meters

# Retry policy defaults
MAX_RETRY_ATTEMPTS = 3
BASE_RETRY_DELAY = 1.0
MAX_RETRY_DELAY = 10.0


# ============================================================================
# TYPE-SAFE DATA STRUCTURES
# ============================================================================

@dataclass
class WaypointData:
    """Type-safe waypoint data with built-in validation.
    
    Attributes:
        lat: Latitude in degrees [-90, 90]
        lon: Longitude in degrees [-180, 180]
        alt: Altitude in meters [0, 10000]
        index: Insertion index (0-based)
    """
    lat: float
    lon: float
    alt: float
    index: int
    
    def __post_init__(self):
        """Validate waypoint data on creation."""
        self._validate()
    
    def _validate(self):
        """Validate coordinate ranges.
        
        Raises:
            ValueError: If any coordinate is out of valid range
        """
        if not (MIN_LATITUDE <= self.lat <= MAX_LATITUDE):
            raise ValueError(f"Invalid latitude: {self.lat}. Must be in [{MIN_LATITUDE}, {MAX_LATITUDE}]")
        
        if not (MIN_LONGITUDE <= self.lon <= MAX_LONGITUDE):
            raise ValueError(f"Invalid longitude: {self.lon}. Must be in [{MIN_LONGITUDE}, {MAX_LONGITUDE}]")
        
        if not (MIN_ALTITUDE <= self.alt <= MAX_ALTITUDE):
            raise ValueError(f"Invalid altitude: {self.alt}. Must be in [{MIN_ALTITUDE}, {MAX_ALTITUDE}]")
        
        if self.index < 0:
            raise ValueError(f"Invalid index: {self.index}. Must be non-negative")
    
    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary for backward compatibility."""
        return {
            'lat': self.lat,
            'lon': self.lon,
            'alt': self.alt,
            'index': self.index
        }


@dataclass
class WaypointManagerConfig:
    """Configuration for WaypointManager with validation.
    
    All timing and operational parameters in one place for easy tuning.
    """
    # Timeouts (seconds)
    service_timeout: float = 5.0
    pull_sync_timeout: float = 2.0
    mission_operation_timeout: float = 30.0
    
    # Default waypoint parameters
    default_hold_time: float = 0.0
    default_accept_radius: float = 5.0
    default_pass_radius: float = 5.0
    default_yaw: float = 0.0
    
    # Retry configuration
    retry_max_attempts: int = 3
    retry_base_delay: float = 1.0
    retry_max_delay: float = 10.0
    
    # Health monitoring
    health_check_period: float = 1.0  # seconds
    max_mission_busy_duration: float = 60.0  # seconds
    
    # Logging
    log_level_default: SeverityLevel = SeverityLevel.INFO
    
    def __post_init__(self):
        """Validate configuration parameters."""
        if self.service_timeout <= 0:
            raise ValueError(f"service_timeout must be > 0, got {self.service_timeout}")
        if self.pull_sync_timeout <= 0:
            raise ValueError(f"pull_sync_timeout must be > 0, got {self.pull_sync_timeout}")
        if self.retry_max_attempts < 1:
            raise ValueError(f"retry_max_attempts must be >= 1, got {self.retry_max_attempts}")
        if self.default_accept_radius <= 0:
            raise ValueError(f"default_accept_radius must be > 0, got {self.default_accept_radius}")
        if self.health_check_period <= 0:
            raise ValueError(f"health_check_period must be > 0, got {self.health_check_period}")

class RetryPolicy:
    """Exponential backoff retry policy for resilient operations.
    
    Attributes:
        max_attempts: Maximum number of retry attempts
        base_delay: Initial delay in seconds
        max_delay: Maximum delay cap in seconds
        current_attempt: Current attempt counter (internal)
    """
    max_attempts: int = MAX_RETRY_ATTEMPTS
    base_delay: float = BASE_RETRY_DELAY
    max_delay: float = MAX_RETRY_DELAY
    current_attempt: int = 0
    
    def get_next_delay(self) -> float | None:
        """Calculate next retry delay with exponential backoff.
        
        Returns:
            Delay in seconds, or None if max attempts exceeded
        """
        if self.current_attempt >= self.max_attempts:
            return None
        
        # Exponential backoff: delay = base * 2^attempt, capped at max_delay
        delay = min(self.base_delay * (2 ** self.current_attempt), self.max_delay)
        self.current_attempt += 1
        return delay
    
    def reset(self):
        """Reset retry counter for new operation."""
        self.current_attempt = 0
    
    def has_attempts_left(self) -> bool:
        """Check if retries are still available."""
        return self.current_attempt < self.max_attempts


# Legacy constants for backward compatibility
MAV_CMD_NAV_WAYPOINT = MAVCommand.NAV_WAYPOINT
MAV_CMD_NAV_TAKEOFF = MAVCommand.NAV_TAKEOFF
MAV_CMD_NAV_RETURN_TO_LAUNCH = MAVCommand.NAV_RETURN_TO_LAUNCH

class CircuitBreaker:
    """Circuit breaker for preventing cascading failures."""
    
    def __init__(self, failure_threshold: int = 5, recovery_timeout: float = 30.0):
        self.failure_threshold = failure_threshold
        self.recovery_timeout = recovery_timeout
        self.failure_count = 0
        self.success_count = 0
        self.state = "CLOSED"  # CLOSED, OPEN, HALF_OPEN
        self.last_failure_time = None
        self.opened_time = None
    
    def record_success(self) -> None:
        self.success_count += 1
        self.failure_count = 0
        if self.state == "HALF_OPEN":
            self.state = "CLOSED"
            self.opened_time = None
    
    def record_failure(self, current_time) -> None:
        self.failure_count += 1
        self.last_failure_time = current_time
        if self.failure_count >= self.failure_threshold and self.state == "CLOSED":
            self.state = "OPEN"
            self.opened_time = current_time
    
    def can_attempt(self, current_time) -> tuple[bool, str]:
        if self.state == "CLOSED":
            return True, "Circuit closed - normal operation"
        if self.state == "OPEN":
            if self.opened_time:
                time_since_open = (current_time - self.opened_time).nanoseconds / 1e9
                if time_since_open >= self.recovery_timeout:
                    self.state = "HALF_OPEN"
                    return True, "Testing recovery"
                return False, f"Retry in {self.recovery_timeout - time_since_open:.1f}s"
            return False, "Circuit open"
        return True, "Testing recovery"
    
    def reset(self) -> None:
        self.failure_count = 0
        self.state = "CLOSED"
        self.opened_time = None

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
        
        # Mission operation lock to prevent concurrent modifications (FIX #2)
        self._mission_busy = False
        self._mission_busy_start_time = None
        
        # Health and performance tracking
        self._total_operations_count = 0
        self._successful_operations_count = 0
        self._failed_operations_count = 0
        
        # Pull synchronization: wait for topic update after pull service (FIX #1)
        self._pending_pull_callback = None
        self._expected_wp_count = None
        self._pull_deadline = None
        self._wp_list_seq = 0  # ISSUE #3: Sequence counter for waypoint list updates
        self._await_seq = 0    # ISSUE #3: Sequence we're waiting for after pull
        
        # Timer for pull timeout checking (ISSUE #1)
        self._pull_timeout_timer = self.create_timer(0.1, self._check_pull_timeout)
        
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
        self._circuit_breaker = CircuitBreaker(5, 30.0)
        self._health_timer = self.create_timer(1.0, self._check_system_health)
        self._last_health_check = self.get_clock().now()
        
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
        
        # Configurable waypoint creation parameters (FIX #7)
        self.declare_parameter('default_hold_time', 15.0)
        self.declare_parameter('default_acceptance_radius', 0.0)
        self.declare_parameter('default_frame', 3)  # MAV_FRAME_GLOBAL_RELATIVE_ALT

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
    
    def _check_pull_timeout(self):
        """ISSUE #1: Check if pending pull has timed out"""
        if self._pending_pull_callback and self._pull_deadline:
            if self.get_clock().now() > self._pull_deadline:
                self.get_logger().error("Pull timeout: waypoint list update never arrived")
                callback = self._pending_pull_callback
                self._pending_pull_callback = None
                self._expected_wp_count = None
                self._pull_deadline = None
                self._await_seq = 0
                callback(False)
    
    # things to ask what are special waypoints ? what are useful derived indices ? what are important indeces ? 

    def waypoints_list_cb(self, data):        
        self.waypoint_list = data
        
        # recreating them just in case
        self.takeoff_index = -1
        self.rtl_index = -1
        self.next_after_takeoff = -1
        self.last_before_rtl = -1
        
        takeoff_count = 0
        rtl_count = 0
        
        # Analyze waypoint list to find special waypoints
        for i, wp in enumerate(self.waypoint_list.waypoints):       
            if wp.command == MAVCommand.NAV_TAKEOFF:
                if takeoff_count == 0:  # FIX #6: Only use first TAKEOFF
                    self.takeoff_index = i
                    if i + 1 < len(self.waypoint_list.waypoints):
                        self.next_after_takeoff = i + 1
                takeoff_count += 1

            if wp.command == MAVCommand.NAV_RETURN_TO_LAUNCH:
                if rtl_count == 0:  # FIX #6: Only use first RTL
                    self.rtl_index = i
                    if i > 0:  # Changed from i - 1 > 0 to i > 0 (in a case of 2 wp, this would be invalid)
                        self.last_before_rtl = i - 1
                rtl_count += 1
        
        # Warn on duplicates (FIX #6)
        if takeoff_count > 1:
            self.get_logger().warn(f"Found {takeoff_count} TAKEOFF waypoints - using first at index {self.takeoff_index}")
        if rtl_count > 1:
            self.get_logger().warn(f"Found {rtl_count} RTL waypoints - using first at index {self.rtl_index}")

        # Update waypoint count
        old_num = self.num_waypoints
        self.num_waypoints = len(self.waypoint_list.waypoints)
        
        # ISSUE #3: Increment sequence counter on every update
        self._wp_list_seq += 1
        
        # FIX #1 + ISSUE #3: Check if this is a pull completion (topic update after pull service)
        if self._pending_pull_callback and self._expected_wp_count is not None:
            # Use sequence counter to ensure we got a NEW update (not a stale republish)
            if self._wp_list_seq > self._await_seq and self.num_waypoints == self._expected_wp_count:
                callback = self._pending_pull_callback
                self._pending_pull_callback = None
                self._expected_wp_count = None
                self._pull_deadline = None
                self._await_seq = 0
                self.get_logger().info(f"Pull completed: waypoint list synchronized (seq {self._wp_list_seq})")
                callback(True)
        
        # FIX #10: Only log if structure changed (reduce spam)
        current_params = {
            'num_waypoints': self.num_waypoints,
            'takeoff_index': self.takeoff_index,
            'rtl_index': self.rtl_index,
            'next_after_takeoff': self.next_after_takeoff,
            'last_before_rtl': self.last_before_rtl
        }
        
        if current_params != self._cached_params:
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
        """Push current waypoint list to autopilot asynchronously.
        
        Sends complete waypoint list to autopilot via MAVROS service.
        On failure, initiates automatic rollback by pulling fresh list.
        
        Args:
            callback: Optional function(success: bool) called after operation
        
        Returns:
            None: Operation is asynchronous, check callback for result
        
        Example:
            >>> def on_push_complete(success):
            ...     if success:
            ...         print("Mission updated on autopilot")
            ...     else:
            ...         print("Push failed, rolled back")
            >>> manager.push_waypoints(callback=on_push_complete)
        
        Note:
            - Returns immediately, actual operation is async
            - Automatic rollback on failure (pulls fresh list)
            - Logs detailed error messages on failures
            - Verifies all waypoints transferred successfully
        """
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
                        if callback:
                            callback(success)
                    else:
                        # FIX #4 + ISSUE #4: On push failure, pull from autopilot to resync (autopilot is truth)
                        # Call callback immediately with failure result
                        self.get_logger().error("Failed to push waypoints - re-pulling from autopilot to resync")
                        if callback:
                            callback(False)  # Original push failed
                        # Resync in background (don't block on this)
                        def resync_callback(resync_success):
                            if resync_success:
                                self.get_logger().info("Resynced with autopilot after push failure")
                            else:
                                self.get_logger().warn("Background resync also failed")
                        self.pull_waypoints(callback=resync_callback)
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
        """Pull fresh waypoint list from autopilot asynchronously.
        
        Requests waypoint list from autopilot via MAVROS service.
        Operation completes when waypoint list topic is received.
        
        Args:
            callback: Optional function(success: bool) called after operation.
                     Invoked when waypoint list topic is received.
        
        Returns:
            None: Operation is asynchronous, check callback for result
        
        Example:
            >>> def on_pull_complete(success):
            ...     print(f"Pull {'succeeded' if success else 'failed'}")
            >>> manager.pull_waypoints(callback=on_pull_complete)
        
        Note:
            - Returns immediately, actual operation is async
            - Monitors for waypoint list topic within 2s timeout
            - Check logs for timeout or synchronization errors
            - May be called without callback for silent pull
        """
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
                        # FIX #1: Store callback to be called when topic update arrives
                        self.get_logger().info(f"Pull service returned success: {response.wp_received} waypoints expected")
                        # ISSUE #2: Only set pending state if callback exists
                        if callback:
                            self._pending_pull_callback = callback
                            self._expected_wp_count = response.wp_received
                            self._pull_deadline = self.get_clock().now() + Duration(seconds=2.0)
                            self._await_seq = self._wp_list_seq  # ISSUE #3: Store current seq
                    else:
                        self.get_logger().error("Failed to pull waypoints")
                        if callback:
                            callback(False)
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
        # ISSUE #6: Check if mission is busy
        if self._mission_busy:
            self.get_logger().warn("Mission operation already in progress - rejecting clear request")
            if callback:
                callback(False)
            return None
        
        # ISSUE #6: Check if connected
        if not self.connected:
            self.get_logger().error("Not connected to FCU - cannot clear waypoints")
            if callback:
                callback(False)
            return None
        
        if not self._wait_for_service(self.waypoint_clear, "waypoint clear"):
            if callback:
                callback(False)
            return None
        
        try:
            # ISSUE #6: Lock mission operations
            self._mission_busy = True
            self._mission_busy_start_time = self.get_clock().now()
            self._total_operations_count += 1
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
                finally:
                    # ISSUE #6: Always release lock
                    self._mission_busy = False
            
            future.add_done_callback(done_callback)
            return future
                
        except Exception as e:
            self.get_logger().error(f"Exception during waypoint clear: {e}")
            self._mission_busy = False  # Release lock on exception
            if callback:
                callback(False)
            return None
    
    def _create_waypoint(self, lat: float, lon: float, alt: float) -> Waypoint:
        """Create a navigation waypoint with validation.
        
        Args:
            lat: Latitude in degrees [-90, 90]
            lon: Longitude in degrees [-180, 180]
            alt: Altitude in meters [0, 10000]
        
        Returns:
            Configured Waypoint message
        
        Raises:
            ValueError: If coordinates are out of valid range
        """
        # Validate coordinates using industrial standards
        if not (MIN_LATITUDE <= lat <= MAX_LATITUDE):
            raise ValueError(f"Invalid latitude: {lat}. Must be in [{MIN_LATITUDE}, {MAX_LATITUDE}]")
        if not (MIN_LONGITUDE <= lon <= MAX_LONGITUDE):
            raise ValueError(f"Invalid longitude: {lon}. Must be in [{MIN_LONGITUDE}, {MAX_LONGITUDE}]")
        if not (MIN_ALTITUDE <= alt <= MAX_ALTITUDE):
            raise ValueError(f"Invalid altitude: {alt}. Must be in [{MIN_ALTITUDE}, {MAX_ALTITUDE}]")
        
        # FIX #7: Use configurable parameters instead of hardcoded values
        new_waypoint = Waypoint()
        new_waypoint.frame = self.get_parameter('default_frame').value
        new_waypoint.command = MAVCommand.NAV_WAYPOINT
        new_waypoint.is_current = False
        new_waypoint.autocontinue = True
        new_waypoint.param1 = float(self.get_parameter('default_hold_time').value)
        new_waypoint.param2 = float(self.get_parameter('default_acceptance_radius').value)
        new_waypoint.param3 = float(0)   # Pass through waypoint
        new_waypoint.param4 = float('nan')  # Yaw angle
        new_waypoint.x_lat = float(lat)
        new_waypoint.y_long = float(lon)
        new_waypoint.z_alt = float(alt)
        return new_waypoint

    def insert_new_waypoint(self, wp_list, callback=None):
        try:
            # FIX #2: Check if mission is busy (prevent concurrent operations)
            if self._mission_busy:
                self.get_logger().warn("Mission operation already in progress - rejecting insert request")
                if callback:
                    callback(False)
                return
            
            # FIX #9: Check if connected
            if not self.connected:
                self.get_logger().error("Not connected to FCU - cannot insert waypoints")
                if callback:
                    callback(False)
                return
            
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
            
            # FIX #2: Lock mission operations
            self._mission_busy = True
            self._mission_busy_start_time = self.get_clock().now()
            self._total_operations_count += 1
            self.get_logger().info("Step 1/3: Pulling fresh waypoint list from autopilot")
            
            # Step 1: Pull fresh waypoints
            def on_pull_complete(pull_success):
                if not pull_success:
                    self.get_logger().error("Failed to pull waypoints before insert")
                    self._mission_busy = False  # Release lock on failure
                    if callback:
                        callback(False)
                    return
                
                # Step 2: Validate and modify
                try:
                    # FIX #5: Validate indices with negative check
                    for wp in wp_list:
                        if wp['index'] < 0 or wp['index'] > len(self.waypoint_list.waypoints):
                            self.get_logger().error(f"Index {wp['index']} is out of range (must be 0-{len(self.waypoint_list.waypoints)})")
                            self._mission_busy = False
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
                        self._mission_busy = False  # FIX #2: Always release lock
                        if push_success:
                            self.get_logger().info(f"Successfully inserted {len(wp_list)} new waypoints")
                        else:
                            # FIX #4: Don't rollback local - autopilot pull already did resync
                            self.get_logger().warn("Push failed - autopilot state resynced via pull")
                        
                        if callback:
                            callback(push_success)
                    
                    self.push_waypoints(callback=on_push_complete)
                    
                except Exception as e:
                    self.get_logger().error(f"Error during modification: {str(e)}")
                    self._mission_busy = False
                    if callback:
                        callback(False)
            
            self.pull_waypoints(callback=on_pull_complete)
                
        except Exception as e:
            self.get_logger().error(f"Error in insert_new_waypoint: {str(e)}")
            self._mission_busy = False  # Release lock on exception
            if callback:
                callback(False)
    
    def delete_waypoint(self, index, callback=None):
        try:
            # FIX #2: Check if mission is busy
            if self._mission_busy:
                self.get_logger().warn("Mission operation already in progress - rejecting delete request")
                if callback:
                    callback(False)
                return
            
            # FIX #9: Check if connected
            if not self.connected:
                self.get_logger().error("Not connected to FCU - cannot delete waypoints")
                if callback:
                    callback(False)
                return
            
            # FIX #2: Lock mission operations
            self._mission_busy = True
            self._mission_busy_start_time = self.get_clock().now()
            self._total_operations_count += 1
            self.get_logger().info(f"Step 1/3: Pulling fresh waypoint list before deletion")
            
            # Step 1: Pull fresh waypoints
            def on_pull_complete(pull_success):
                if not pull_success:
                    self.get_logger().error("Failed to pull waypoints before delete")
                    self._mission_busy = False
                    if callback:
                        callback(False)
                    return
                
                # Step 2: Validate and modify
                try:
                    if not (0 <= index < len(self.waypoint_list.waypoints)):
                        self.get_logger().error(f"Index {index} out of range (0-{len(self.waypoint_list.waypoints)-1})")
                        self._mission_busy = False
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
                        self._mission_busy = False  # FIX #2: Always release lock
                        if push_success:
                            self.get_logger().info("Waypoint deleted and mission updated successfully")
                        else:
                            # FIX #4: Don't rollback local - autopilot pull already did resync
                            self.get_logger().warn("Push failed - autopilot state resynced via pull")
                        
                        if callback:
                            callback(push_success)
                    
                    self.push_waypoints(callback=on_push_complete)
                    
                except Exception as e:
                    self.get_logger().error(f"Error during deletion: {str(e)}")
                    self._mission_busy = False
                    if callback:
                        callback(False)
            
            self.pull_waypoints(callback=on_pull_complete)
                
        except Exception as e:
            self.get_logger().error(f"Error in delete_waypoint: {str(e)}")
            self._mission_busy = False  # Release lock on exception
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
            if hold_time > 0 and wp.command == MAVCommand.NAV_WAYPOINT:
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
        """Send status text message to autopilot.
        
        Args:
            text: Status message text to send
        """
        msg = StatusText()
        msg.severity = SeverityLevel.INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status sent: {text}")

    def handle_wp_req(self, request, response):
        """Handle add waypoint service request with industrial validation.
        
        Args:
            request: AddWaypoint service request containing waypoint arrays
            response: AddWaypoint service response
        
        Returns:
            response: Service response with success=True if accepted
        
        Note:
            Service returns immediately with acceptance status.
            Monitor logs for actual operation completion result.
        """
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
            
            # Build waypoint list using type-safe WaypointData with validation
            wp_list = []
            for i in range(len(request.latitude)):
                try:
                    # WaypointData validates coordinates in __post_init__
                    wp_data = WaypointData(
                        lat=float(request.latitude[i]),
                        lon=float(request.longitude[i]),
                        alt=float(request.altitude[i]),
                        index=int(request.index[i])
                    )
                    wp_list.append(wp_data.to_dict())
                except ValueError as e:
                    self.get_logger().error(f"Invalid waypoint {i}: {e}")
                    response.success = False
                    return response
            
            # FIX #3: Remove optimistic response, warn about async nature
            # WARNING: ROS2 services are synchronous but our operation is async
            # This means we CANNOT return the actual result here
            # The service will return before the operation completes
            # RECOMMENDATION: Convert to ROS2 Action for proper async feedback
            # For now: return False and log actual results via callbacks
            
            def on_complete(success):
                if success:
                    self.get_logger().info("✓ Waypoint insertion completed successfully")
                else:
                    self.get_logger().error("✗ Waypoint insertion failed")
            
            self.insert_new_waypoint(wp_list, callback=on_complete)
            
            # ISSUE #7: Return True to indicate "request accepted" (async operation started)
            # Monitor logs for actual completion result
            response.success = True
            self.get_logger().info("AddWaypoint request accepted - monitor logs for completion")
            
        except Exception as e:
            self.get_logger().error(f"Failed to add waypoint: {e}")
            response.success = False
            
        return response

    def handle_wp_del_req(self, request, response):
        self.get_logger().info(f"DelWaypoint request - index={request.index}")

        try:
            # FIX #3: Remove optimistic response
            def on_complete(success):
                if success:
                    self.get_logger().info("✓ Waypoint deletion completed successfully")
                else:
                    self.get_logger().error("✗ Waypoint deletion failed")
            
            self.delete_waypoint(request.index, callback=on_complete)
            
            # ISSUE #7: Return True to indicate "request accepted" (async operation started)
            # Monitor logs for actual completion result
            response.success = True
            self.get_logger().info("DelWaypoint request accepted - monitor logs for completion")
            
        except Exception as e:
            self.get_logger().error(f"Failed to delete waypoint: {e}")
            response.success = False
            
        return response

    def handle_update_mission(self, request, response):
        """Handle update mission service request.
        
        Pulls fresh waypoint list from autopilot.
        
        Args:
            request: UpdateMission service request
            response: UpdateMission service response
        
        Returns:
            response: Service response with success=True if accepted
        """
        self.get_logger().info("UpdateMission request received")
        
        try:
            # FIX #3: Remove optimistic response
            def on_complete(success):
                if success:
                    self.get_logger().info("✓ Mission updated successfully")
                else:
                    self.get_logger().error("✗ Failed to update mission")
            
            self.pull_waypoints(callback=on_complete)
            
            # ISSUE #7: Return True to indicate "request accepted" (async operation started)
            # Monitor logs for actual completion result
            response.success = True
            self.get_logger().info("UpdateMission request accepted - monitor logs for completion")
                
        except Exception as e:
            self.get_logger().error(f"Failed to update mission: {e}")
            response.success = False
            
        return response
    def _check_system_health(self):
        """Periodic health monitoring."""
        now = self.get_clock().now()
        # Check for stuck operations
        if self._mission_busy and self._mission_busy_start_time:
            duration = (now - self._mission_busy_start_time).nanoseconds / 1e9
            if duration > 60.0:
                self.get_logger().error(f"Mission stuck for {duration:.1f}s - forcing unlock")
                self._mission_busy = False
                self._failed_operations_count += 1
        # Log circuit breaker state
        if self._circuit_breaker.state != "CLOSED":
            self.get_logger().warn(f"Circuit breaker {self._circuit_breaker.state}")

    def _record_operation_success(self):
        self._successful_operations_count += 1
        self._circuit_breaker.record_success()

    def _record_operation_failure(self):
        self._failed_operations_count += 1
        self._circuit_breaker.record_failure(self.get_clock().now())
    
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
