#!/usr/bin/env python3

"""
Payload Drop Planner Node

This node:
1. Monitors detection data from waypoint_detection_tracker
2. Finds waypoints with highest confidence for mannequin AND tent
3. Plans return mission to drop payloads at those locations

Subscribes to:
    - /waypoint_detection_summary (String) - Detection summary from tracker

Uses Services:
    - /addWaypoint (AddWaypoint) - Add return waypoints for payload drop
    - /get_drone_data (GetGPSData) - Get current drone position

Logic:
    - Finds waypoint with highest mannequin confidence
    - Finds waypoint with highest tent confidence
    - Adds waypoints to return to those locations (after scan complete)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.srv import AddWaypoint, GetGPSData
import json
from typing import Dict, Tuple, Optional


class PayloadDropPlanner(Node):
    def __init__(self):
        super().__init__('payload_drop_planner')
        
        # Detection data storage
        self.waypoint_data = {}  # waypoint_index -> {mannequin_conf, tent_conf, location}
        
        # Track if mission is complete
        self.scan_complete = False
        self.return_waypoints_added = False
        
        # Subscribers
        self.create_subscription(
            String,
            '/waypoint_detection_summary',
            self.summary_callback,
            10
        )
        
        # Service clients
        self.add_wp_client = self.create_client(AddWaypoint, "/addWaypoint")
        self.gps_client = self.create_client(GetGPSData, "/get_drone_data")
        
        # Wait for services
        self._wait_for_services()
        
        # Parameters
        self.declare_parameter('min_confidence_threshold', 0.3)  # Minimum confidence to consider
        self.declare_parameter('drop_altitude', 10.0)  # Altitude for payload drop (meters)
        self.declare_parameter('scan_waypoint_start', 0)  # First waypoint index of scan grid
        self.declare_parameter('scan_waypoint_end', -1)  # Last waypoint index of scan grid (-1 = auto)
        self.declare_parameter('rtl_index', -1)  # Return to launch waypoint index
        
        self.min_confidence = self.get_parameter('min_confidence_threshold').value
        self.drop_altitude = self.get_parameter('drop_altitude').value
        self.scan_start = self.get_parameter('scan_waypoint_start').value
        self.scan_end = self.get_parameter('scan_waypoint_end').value
        self.rtl_index = self.get_parameter('rtl_index').value
        
        self.get_logger().info("="*80)
        self.get_logger().info("Payload Drop Planner initialized")
        self.get_logger().info(f"Min confidence threshold: {self.min_confidence}")
        self.get_logger().info(f"Drop altitude: {self.drop_altitude}m")
        self.get_logger().info("="*80)
    
    def _wait_for_services(self):
        """Wait for required services to be available"""
        self.get_logger().info("Waiting for services...")
        while not self.add_wp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /addWaypoint service...")
        while not self.gps_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /get_drone_data service...")
        self.get_logger().info("All services available")
    
    def summary_callback(self, msg):
        """Process detection summary and update waypoint data"""
        try:
            summary = json.loads(msg.data)
            
            # Extract confidence data for each waypoint
            for wp_idx_str, wp_data in summary.get('waypoints', {}).items():
                wp_idx = int(wp_idx_str)
                
                # Only process waypoints in scan range
                if self.scan_end >= 0 and (wp_idx < self.scan_start or wp_idx > self.scan_end):
                    continue
                
                # Extract mannequin and tent confidences from summary
                mannequin_conf = wp_data.get('mannequin_confidence', 0.0)
                tent_conf = wp_data.get('tent_confidence', 0.0)
                
                # Store waypoint data
                self.waypoint_data[wp_idx] = {
                    'mannequin_confidence': mannequin_conf,
                    'tent_confidence': tent_conf,
                    'location': wp_data.get('location', {}),
                    'num_detections': wp_data.get('num_detections', 0)
                }
            
            # Check if we should plan payload drop
            if not self.return_waypoints_added:
                self.check_and_plan_drop()
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse summary JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing summary: {e}")
    
    def find_max_confidence_waypoints(self) -> Tuple[Optional[int], Optional[int]]:
        """
        Find waypoints with highest confidence for mannequin and tent
        
        Returns:
            tuple: (mannequin_waypoint_index, tent_waypoint_index)
        """
        max_mannequin_wp = None
        max_mannequin_conf = 0.0
        
        max_tent_wp = None
        max_tent_conf = 0.0
        
        for wp_idx, data in self.waypoint_data.items():
            mannequin_conf = data.get('mannequin_confidence', 0.0)
            tent_conf = data.get('tent_confidence', 0.0)
            
            # Only consider if above minimum threshold
            if mannequin_conf >= self.min_confidence and mannequin_conf > max_mannequin_conf:
                max_mannequin_conf = mannequin_conf
                max_mannequin_wp = wp_idx
            
            if tent_conf >= self.min_confidence and tent_conf > max_tent_conf:
                max_tent_conf = tent_conf
                max_tent_wp = wp_idx
        
        return max_mannequin_wp, max_tent_wp
    
    def check_and_plan_drop(self):
        """Check if scan is complete and plan payload drop if ready"""
        # TODO: Add logic to detect when scan is complete
        # For now, we'll plan when we have enough data
        
        if len(self.waypoint_data) < 5:  # Need at least some data
            return
        
        # Find max confidence waypoints
        mannequin_wp, tent_wp = self.find_max_confidence_waypoints()
        
        if mannequin_wp is None and tent_wp is None:
            self.get_logger().warn("No waypoints found above confidence threshold")
            return
        
        self.get_logger().info("="*80)
        self.get_logger().info("PLANNING PAYLOAD DROP MISSION")
        self.get_logger().info("="*80)
        
        if mannequin_wp is not None:
            data = self.waypoint_data[mannequin_wp]
            self.get_logger().info(
                f"Max mannequin confidence: Waypoint {mannequin_wp}, "
                f"Confidence: {data['mannequin_confidence']:.2f}"
            )
        
        if tent_wp is not None:
            data = self.waypoint_data[tent_wp]
            self.get_logger().info(
                f"Max tent confidence: Waypoint {tent_wp}, "
                f"Confidence: {data['tent_confidence']:.2f}"
            )
        
        # Add return waypoints
        self.add_return_waypoints(mannequin_wp, tent_wp)
    
    def add_return_waypoints(self, mannequin_wp: Optional[int], tent_wp: Optional[int]):
        """Add waypoints to return to target locations for payload drop"""
        if self.return_waypoints_added:
            return
        
        # Determine insertion index (before RTL)
        if self.rtl_index > 0:
            insert_index = self.rtl_index
        else:
            # Insert at end if RTL index not specified
            insert_index = max(self.waypoint_data.keys()) + 1 if self.waypoint_data else 100
        
        waypoints_added = 0
        
        # Add mannequin drop waypoint
        if mannequin_wp is not None:
            data = self.waypoint_data[mannequin_wp]
            loc = data.get('location', {})
            
            if loc.get('lat') is not None and loc.get('lon') is not None:
                success = self.add_drop_waypoint(
                    loc['lat'],
                    loc['lon'],
                    self.drop_altitude,
                    insert_index,
                    f"Mannequin drop (from WP {mannequin_wp})"
                )
                if success:
                    waypoints_added += 1
                    insert_index += 1
        
        # Add tent drop waypoint
        if tent_wp is not None:
            data = self.waypoint_data[tent_wp]
            loc = data.get('location', {})
            
            if loc.get('lat') is not None and loc.get('lon') is not None:
                success = self.add_drop_waypoint(
                    loc['lat'],
                    loc['lon'],
                    self.drop_altitude,
                    insert_index,
                    f"Tent drop (from WP {tent_wp})"
                )
                if success:
                    waypoints_added += 1
        
        if waypoints_added > 0:
            self.return_waypoints_added = True
            self.get_logger().info(f"Successfully added {waypoints_added} payload drop waypoints")
        else:
            self.get_logger().error("Failed to add payload drop waypoints - location data missing")
    
    def add_drop_waypoint(self, lat: float, lon: float, alt: float, index: int, description: str) -> bool:
        """Add a waypoint for payload drop"""
        try:
            request = AddWaypoint.Request()
            request.latitude = lat
            request.longitude = lon
            request.altitude = alt
            request.index = index
            
            future = self.add_wp_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info(
                        f"Added {description} waypoint at index {index}: "
                        f"Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m"
                    )
                    return True
                else:
                    self.get_logger().error(f"Failed to add waypoint: service returned failure")
                    return False
            else:
                self.get_logger().error("Add waypoint request timed out")
                return False
        except Exception as e:
            self.get_logger().error(f"Error adding drop waypoint: {e}")
            return False
    
    def get_detection_data(self) -> Dict:
        """
        Get simplified detection data in format: 
        {waypoint_index: (mannequin_confidence, tent_confidence)}
        """
        result = {}
        for wp_idx, data in self.waypoint_data.items():
            result[wp_idx] = {
                'mannequin_confidence': data.get('mannequin_confidence', 0.0),
                'tent_confidence': data.get('tent_confidence', 0.0)
            }
        return result


def main(args=None):
    rclpy.init(args=args)
    
    planner = PayloadDropPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        planner.get_logger().info("Shutting down Payload Drop Planner...")
        # Print final results
        mannequin_wp, tent_wp = planner.find_max_confidence_waypoints()
        planner.get_logger().info(f"Final max mannequin waypoint: {mannequin_wp}")
        planner.get_logger().info(f"Final max tent waypoint: {tent_wp}")
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

