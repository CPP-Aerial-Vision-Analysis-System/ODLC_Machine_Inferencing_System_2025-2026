# Waypoint Detection Tracking System

## Overview

This system tracks object detections per waypoint as your drone flies over a grid pattern. It creates a dictionary mapping each waypoint to its location, detected objects, and confidence scores.

## System Architecture

```
┌─────────────────┐
│  image_pub_siyi │  Takes photos at waypoints
│   (camera node) │  Saves to camera_feed/
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│ object_detection_sahi   │  Processes images
│  (detection node)       │  Publishes ImageResult
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ waypoint_detection_     │  Tracks detections
│ tracker (new node)      │  Creates dictionary
└─────────────────────────┘
```

## Data Structure

The tracker creates a dictionary with this structure:

```python
{
    waypoint_index: {
        'location': {
            'lat': float,      # Latitude
            'lon': float,      # Longitude  
            'alt': float       # Altitude (meters)
        },
        'detections': [
            {
                'class': 'person',           # Detected class
                'confidence': 0.85,          # Detection confidence (0-1)
                'bbox': {                    # Bounding box
                    'center_x': 640,
                    'center_y': 480,
                    'size_x': 100,
                    'size_y': 150
                },
                'description': 'person',      # Human-readable description
                'area': 15000,               # Bounding box area (pixels)
                'image': 'photo_20250124_123456.jpg',  # Source image
                'timestamp': '2025-01-24T12:34:56'     # Detection timestamp
            },
            # ... more detections
        ],
        'images': ['photo1.jpg', 'photo2.jpg'],  # All images from this waypoint
        'timestamps': ['2025-01-24T12:34:56', ...],
        'num_total_detections': 5
    }
}
```

## Setup Instructions

### 1. Build the Package

```bash
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```

### 2. Run the Complete System

You need to run **three nodes** simultaneously:

**Terminal 1: Camera Node**
```bash
ros2 run video_cam image_pub
```
- Takes photos at waypoints
- Saves images to `camera_feed/` directory

**Terminal 2: Detection Node**
```bash
ros2 run video_cam object_detection_sahi
```
- Monitors `camera_feed/` for new images
- Processes images with SAHI + YOLO11s
- Publishes `ImageResult` messages to `/image_detections`

**Terminal 3: Tracker Node (NEW)**
```bash
ros2 run video_cam waypoint_detection_tracker
```
- Subscribes to `/image_detections` and `/mavros/mission/waypoints`
- Tracks detections per waypoint
- Saves data to JSON file

### 3. Using Launch Files (Recommended)

Create a launch file to start all nodes together:

```python
# launch/waypoint_detection.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_cam',
            executable='image_pub',
            name='siyi_camera_publisher'
        ),
        Node(
            package='video_cam',
            executable='object_detection_sahi',
            name='sahi_detection_node',
            parameters=[{
                'confidence_threshold': 0.15,
                'slice_height': 512,
                'slice_width': 512
            }]
        ),
        Node(
            package='video_cam',
            executable='waypoint_detection_tracker',
            name='waypoint_tracker_node'
        )
    ])
```

Then run:
```bash
ros2 launch video_cam waypoint_detection.launch.py
```

## How It Works

### Step-by-Step Flow

1. **Drone Reaches Waypoint**
   - MAVROS publishes `WaypointReached` to `/mavros/mission/reached`
   - `object_detection_sahi` updates `waypoint_reached` variable

2. **Camera Captures Image**
   - `image_pub_siyi` saves image to `camera_feed/` directory
   - Filename: `photo_YYYYMMDD-HHMMSS.jpg`

3. **Detection Processing**
   - `object_detection_sahi` detects new image in `camera_feed/`
   - Processes image with SAHI + YOLO11s
   - Creates `ImageResult` message with:
     - Detections (class, confidence, bbox)
     - Waypoint index (from step 1)
     - Image filename
     - Timestamp

4. **Tracking**
   - `waypoint_detection_tracker` receives `ImageResult`
   - Looks up waypoint location from `/mavros/mission/waypoints`
   - Adds detections to dictionary for that waypoint
   - Auto-saves every 30 seconds

## Accessing the Data

### Option 1: Read from JSON File

The tracker automatically saves data to:
```
install/video_cam/share/video_cam/waypoint_detection_data/
  ├── waypoint_detections_latest.json  (auto-updated every 30s)
  └── waypoint_detections_YYYYMMDD_HHMMSS.json  (timestamped saves)
```

```python
import json

# Load the data
with open('waypoint_detections_latest.json', 'r') as f:
    data = json.load(f)

# Access detections for waypoint 3
wp_3 = data['waypoints'][3]
print(f"Waypoint 3 location: {wp_3['location']}")
print(f"Detections: {wp_3['detections']}")

# Count detections by class
for wp_idx, wp_data in data['waypoints'].items():
    print(f"\nWaypoint {wp_idx}:")
    for det in wp_data['detections']:
        print(f"  - {det['class']}: {det['confidence']:.2f}")
```

### Option 2: Subscribe to Summary Topic

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.create_subscription(
            String,
            '/waypoint_detection_summary',
            self.summary_callback,
            10
        )
    
    def summary_callback(self, msg):
        summary = json.loads(msg.data)
        print(f"Total waypoints: {summary['total_waypoints_scanned']}")
        print(f"Total detections: {summary['total_detections']}")

rclpy.init()
node = DetectionSubscriber()
rclpy.spin(node)
```

### Option 3: Use the Dictionary Directly (In Code)

If you need to access the data from within another ROS2 node:

```python
# This would require modifying waypoint_detection_tracker.py
# to provide a service or expose the dictionary

# Example: Add a service to get detections
def get_waypoint_detections(self, waypoint_index):
    return self.waypoint_detections.get(waypoint_index, {})
```

## Example Output

### JSON File Structure
```json
{
  "metadata": {
    "timestamp": "2025-01-24T12:34:56.789",
    "total_waypoints": 5,
    "total_detections": 12
  },
  "waypoints": {
    "0": {
      "location": {
        "lat": 37.7749,
        "lon": -122.4194,
        "alt": 15.5
      },
      "detections": [
        {
          "class": "person",
          "confidence": 0.87,
          "bbox": {
            "center_x": 640,
            "center_y": 480,
            "size_x": 100,
            "size_y": 150
          },
          "description": "person",
          "area": 15000,
          "image": "photo_20250124_123400.jpg",
          "timestamp": "2025-01-24T12:34:00"
        },
        {
          "class": "tent",
          "confidence": 0.72,
          "bbox": {...},
          "description": "tent-like (backpack)",
          "area": 8000,
          "image": "photo_20250124_123400.jpg",
          "timestamp": "2025-01-24T12:34:00"
        }
      ],
      "images": ["photo_20250124_123400.jpg"],
      "timestamps": ["2025-01-24T12:34:00"],
      "num_total_detections": 2
    },
    "1": {
      "location": {...},
      "detections": [...],
      ...
    }
  }
}
```

## Tips for Grid-Based Scanning

### 1. Ensure Proper Waypoint Tagging

The `object_detection_sahi` node tags images with the **current** waypoint when processing. To ensure accurate tagging:

- Make sure images are processed quickly after capture
- Or use a timestamp-based matching system (future enhancement)

### 2. Multiple Images Per Waypoint

If you take multiple images at each waypoint, they will all be associated with that waypoint's index. The tracker maintains a list of all images per waypoint.

### 3. Handling Edge Cases

- **Waypoint location unknown**: If waypoint list isn't received yet, location will be `None`
- **No detections**: Waypoint entry still created, but `detections` list will be empty
- **Late detections**: If an image is processed after the drone has moved to next waypoint, it will be tagged with the old waypoint index

## Troubleshooting

### No detections being tracked
- Check that `object_detection_sahi` is publishing to `/image_detections`
- Verify `waypoint_detection_tracker` is running: `ros2 node list`
- Check logs: `ros2 topic echo /image_detections`

### Waypoint locations are None
- Ensure MAVROS is running and connected
- Check that `/mavros/mission/waypoints` topic has data: `ros2 topic echo /mavros/mission/waypoints`

### JSON file not updating
- Check that tracker node is running
- Verify write permissions in `waypoint_detection_data/` directory
- Auto-save happens every 30 seconds (check logs)

## Future Enhancements

Potential improvements:
1. Add service to query detections for specific waypoint
2. Add visualization node (RViz markers)
3. Export to CSV for analysis
4. Add confidence threshold filtering
5. Time-based matching instead of waypoint index

