#!/usr/bin/env python3

"""
Waypoint Detection Tracker Node

This node tracks detections per waypoint and maintains a data structure mapping:
    waypoint_index -> {
        'location': {'lat': float, 'lon': float},
        'detections': [{'class': str, 'confidence': float, 'bbox': [...]}, ...],
        'images': [image_filename1, image_filename2, ...],
        'timestamp': 'ISO timestamp'
    }

Subscribes to:
    - /image_detections (ImageResult) - Detection results from object_detection_sahi
    - /mavros/mission/waypoints (WaypointList) - Waypoint locations

Publishes:
    - /waypoint_detection_summary (String) - JSON summary of all detections

Saves to:
    - JSON file with complete detection history per waypoint
"""

import rclpy
from rclpy.node import Node
from interfaces.msg import ImageResult
from mavros_msgs.msg import WaypointList, Waypoint
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from datetime import datetime
import json
import os
from ament_index_python.packages import get_package_share_directory
from collections import defaultdict


class WaypointDetectionTracker(Node):
    def __init__(self):
        super().__init__('waypoint_detection_tracker')
        
        # Data structure: waypoint_index -> detection data
        self.waypoint_detections = defaultdict(lambda: {
            'location': {'lat': None, 'lon': None},
            'detections': [],
            'images': [],
            'timestamps': [],
            'num_total_detections': 0,
            # Simplified format for mannequin and tent
            'mannequin_confidence': 0.0,
            'tent_confidence': 0.0
        })
        
        # Store waypoint list for location lookup
        self.waypoint_list = None
        
        # Subscribers
        self.create_subscription(
            ImageResult, 
            '/image_detections', 
            self.image_result_callback, 
            10
        )
        
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            10
        )
        
        # Publisher for summary
        self.summary_publisher = self.create_publisher(
            String,
            '/waypoint_detection_summary',
            10
        )
        
        # Output directory for JSON files
        self.output_dir = os.path.join(
            get_package_share_directory("video_cam"),
            "waypoint_detection_data"
        )
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Auto-save timer (save every 30 seconds)
        self.save_timer = self.create_timer(30.0, self.auto_save_data)
        
        # Statistics
        self.total_waypoints_with_detections = 0
        self.total_detections = 0
        
        self.get_logger().info("="*80)
        self.get_logger().info("Waypoint Detection Tracker initialized")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info("="*80)
    
    def waypoints_callback(self, msg):
        #runs weherver a new waypoint list message is published to the /mavros/mission/waypoints topic
        #1. stores the latest mission waypoint list so we know the GPS coordinates of each waypoint.
        #2. updates waypoint detection records with the correct latitude and longitude of each waypoint.
        self.waypoint_list = msg
        self.get_logger().info(f"Received waypoint list with {len(msg.waypoints)} waypoints")
        
        # Update location data for known waypoints
        for i, wp in enumerate(msg.waypoints):
            if i in self.waypoint_detections:
                self.waypoint_detections[i]['location'] = {
                    'lat': wp.x_lat,
                    'lon': wp.y_long
                }
    
    def image_result_callback(self, msg):
        # It takes the detection results from the image, 
        # figures out which waypoint the drone was at when the image was taken,
        # stores all detection info under that waypoint in the tracker’s data structure.
        waypoint_index = msg.waypoint_index
        
        # Get waypoint location if available
        location = {'lat': None, 'lon': None}
        if self.waypoint_list and waypoint_index < len(self.waypoint_list.waypoints):
            wp = self.waypoint_list.waypoints[waypoint_index]
            location = {
                'lat': wp.x_lat,
                'lon': wp.y_long
            }
        
        # Update location
        self.waypoint_detections[waypoint_index]['location'] = location
        
        # Extract detections from ImageResult message
        detections = []
        for i, detection in enumerate(msg.detections.detections):
            # Get class and confidence
            class_name = msg.classes[i] if i < len(msg.classes) else 'unknown'
            confidence = msg.confidences[i] if i < len(msg.confidences) else 0.0
            description = msg.descriptions[i] if i < len(msg.descriptions) else class_name
            area = msg.areas[i] if i < len(msg.areas) else 0.0
            
            # Get bounding box
            bbox = {
                'center_x': detection.bbox.center.x,
                'center_y': detection.bbox.center.y,
                'size_x': detection.bbox.size_x,
                'size_y': detection.bbox.size_y
            }
            
            detection_data = {
                'class': class_name,
                'confidence': float(confidence),
                'description': description,
                'bbox': bbox,
                'area': float(area),
                'image': msg.image_name,
                'timestamp': msg.timestamp
            }
            
            detections.append(detection_data)
        
        # Add detections to waypoint
        self.waypoint_detections[waypoint_index]['detections'].extend(detections)
        self.waypoint_detections[waypoint_index]['images'].append(msg.image_name)
        self.waypoint_detections[waypoint_index]['timestamps'].append(msg.timestamp)
        self.waypoint_detections[waypoint_index]['num_total_detections'] += len(detections)
        
        # Update max confidence for mannequin and tent
        self._update_max_confidence(waypoint_index, detections)
        
        # Update statistics
        self.total_detections += len(detections)
        if len(detections) > 0:
            self.get_logger().info(
                f"Waypoint {waypoint_index}: Added {len(detections)} detections "
                f"(Total: {self.waypoint_detections[waypoint_index]['num_total_detections']})"
            )
        
        # Publish summary
        self.publish_summary()
    
    def _update_max_confidence(self, waypoint_index, detections):
        #self explanatory
        for det in detections:
            class_name = det.get('class', '').lower()
            confidence = det.get('confidence', 0.0)
            
            # Check for mannequin (person, mannequin, etc.)
            if 'person' in class_name or 'mannequin' in class_name:
                self.waypoint_detections[waypoint_index]['mannequin_confidence'] = max(
                    self.waypoint_detections[waypoint_index]['mannequin_confidence'],
                    confidence
                )
            
            # Check for tent
            if 'tent' in class_name:
                self.waypoint_detections[waypoint_index]['tent_confidence'] = max(
                    self.waypoint_detections[waypoint_index]['tent_confidence'],
                    confidence
                )
    
    def get_detection_summary(self):
        """
        Generate summary statistics for all waypoints
        
        Returns:
            dict: Summary with counts per waypoint and overall statistics
        """
        summary = {
            'total_waypoints_scanned': len(self.waypoint_detections),
            'total_detections': self.total_detections,
            'waypoints': {}
        }
        
        # Count detections per class
        class_counts = defaultdict(int)
        
        for wp_idx, data in self.waypoint_detections.items():
            wp_summary = {
                'location': data['location'],
                'num_detections': data['num_total_detections'],
                'num_images': len(data['images']),
                'detections_by_class': defaultdict(int),
                'mannequin_confidence': data['mannequin_confidence'],
                'tent_confidence': data['tent_confidence']
            }
            
            # Count by class
            for det in data['detections']:
                class_name = det['class']
                class_counts[class_name] += 1
                wp_summary['detections_by_class'][class_name] += 1
            
            summary['waypoints'][wp_idx] = dict(wp_summary)
        
        summary['detections_by_class'] = dict(class_counts)
        
        return summary
    
    def publish_summary(self):
        """Publish summary as JSON string"""
        summary = self.get_detection_summary()
        summary_msg = String()
        summary_msg.data = json.dumps(summary, indent=2)
        self.summary_publisher.publish(summary_msg)
    
    def save_data_to_json(self, filename=None):
        """
        Save waypoint detection data to JSON file
        
        Args:
            filename: Optional filename. If None, uses timestamp-based name
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"waypoint_detections_{timestamp}.json"
        
        filepath = os.path.join(self.output_dir, filename)
        
        # Convert defaultdict to regular dict for JSON serialization
        data = {
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'total_waypoints': len(self.waypoint_detections),
                'total_detections': self.total_detections
            },
            'waypoints': dict(self.waypoint_detections)
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2, default=str)
            
            self.get_logger().info(f"Saved detection data to {filepath}")
            return filepath
        except Exception as e:
            self.get_logger().error(f"Failed to save data: {e}")
            return None
    
    def auto_save_data(self):
        """Auto-save data periodically"""
        if len(self.waypoint_detections) > 0:
            self.save_data_to_json("waypoint_detections_latest.json")
    
    def get_detection_dict(self):
        """
        Get the detection dictionary in the format requested:
        waypoint_index -> {
            'location': {'lat': float, 'lon': float},
            'detections': [{'class': str, 'confidence': float, ...}, ...]
        }
        
        Returns:
            dict: Formatted detection dictionary
        """
        result = {}
        for wp_idx, data in self.waypoint_detections.items():
            result[wp_idx] = {
                'location': data['location'],
                'detections': [
                    {
                        'class': det['class'],
                        'confidence': det['confidence'],
                        'bbox': det['bbox'],
                        'description': det['description'],
                        'area': det['area']
                    }
                    for det in data['detections']
                ]
            }
        return result
    
    def get_simplified_detection_data(self):
        """
        Get simplified detection data in format:
        {waypoint_index: {'mannequin_confidence': float, 'tent_confidence': float}}
        
        This is the format requested: (waypoint_index, mannequin_confidence, tent_confidence)
        
        Returns:
            dict: Simplified detection data
        """
        result = {}
        for wp_idx, data in self.waypoint_detections.items():
            result[wp_idx] = {
                'mannequin_confidence': data['mannequin_confidence'],
                'tent_confidence': data['tent_confidence']
            }
        return result
    
    def print_detection_summary(self):
        """Print human-readable summary to console"""
        print("\n" + "="*80)
        print("WAYPOINT DETECTION SUMMARY")
        print("="*80)
        
        for wp_idx in sorted(self.waypoint_detections.keys()):
            data = self.waypoint_detections[wp_idx]
            loc = data['location']
            
            print(f"\nWaypoint {wp_idx}:")
            if loc['lat'] is not None:
                print(f"  Location: Lat={loc['lat']:.6f}, Lon={loc['lon']:.6f}")
            else:
                print(f"  Location: Unknown")
            
            print(f"  Total Detections: {data['num_total_detections']}")
            print(f"  Images Processed: {len(data['images'])}")
            
            if len(data['detections']) > 0:
                print("  Detections:")
                for det in data['detections']:
                    print(f"    - {det['class']}: {det['confidence']:.2f} confidence")
        
        print("\n" + "="*80)
        print(f"Total Waypoints with Detections: {len(self.waypoint_detections)}")
        print(f"Total Detections: {self.total_detections}")
        print("="*80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    
    tracker = WaypointDetectionTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        tracker.get_logger().info("Shutting down Waypoint Detection Tracker...")
        # Save final data
        tracker.save_data_to_json()
        tracker.print_detection_summary()
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

