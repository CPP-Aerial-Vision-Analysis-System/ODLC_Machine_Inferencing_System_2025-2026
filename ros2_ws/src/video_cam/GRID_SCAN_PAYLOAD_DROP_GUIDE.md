# Grid Scan and Payload Drop Mission Guide

## Mission Overview

This system implements a complete grid-based scanning mission with automatic payload drop planning:

1. **Grid Setup**: Create waypoints in a snake pattern (left→right, down, right→left, etc.)
2. **Image Capture**: Take photos at each waypoint (center of each grid box)
3. **Object Detection**: Detect mannequins and tents using SAHI + YOLO11s
4. **Data Storage**: Store (waypoint_index, mannequin_confidence, tent_confidence)
5. **Payload Planning**: Find waypoints with highest confidence for mannequin AND tent
6. **Return Mission**: Add waypoints to return to those locations for payload drop

## System Architecture

```
┌─────────────────┐
│  Grid Waypoints  │  Pre-planned waypoints in snake pattern
│  (yellow box)    │  Each waypoint = center of a grid box
└────────┬─────────┘
         │
         ▼
┌─────────────────┐
│  image_pub_siyi │  Takes photo at each waypoint
│   (camera node) │  Saves to camera_feed/
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│ object_detection_sahi   │  Processes images
│  (detection node)       │  Detects mannequin & tent
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ waypoint_detection_     │  Tracks detections per waypoint
│ tracker                 │  Stores: (wp_idx, mannequin_conf, tent_conf)
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ payload_drop_planner     │  Finds max confidence waypoints
│                          │  Adds return waypoints for payload drop
└─────────────────────────┘
```

## Data Format

### Simplified Format (as requested)
```python
{
    waypoint_index: {
        'mannequin_confidence': 0.85,
        'tent_confidence': 0.72
    }
}
```

### Full Format (stored in JSON)
```python
{
    waypoint_index: {
        'location': {'lat': 37.7749, 'lon': -122.4194, 'alt': 15.5},
        'mannequin_confidence': 0.85,
        'tent_confidence': 0.72,
        'detections': [...],
        'images': ['photo1.jpg', 'photo2.jpg']
    }
}
```

## Setup and Running

### 1. Build Package
```bash
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```

### 2. Pre-Flight: Create Grid Waypoints

Before flight, you need to create waypoints in a snake pattern covering the yellow box:

**Pattern:**
```
Row 1: WP0 → WP1 → WP2 → ... → WP9  (left to right)
Row 2: WP10 ← WP11 ← WP12 ← ... ← WP19  (right to left)
Row 3: WP20 → WP21 → WP22 → ... → WP29  (left to right)
...
```

**Example Python script to generate waypoints:**
```python
# generate_grid_waypoints.py
from wp_sender.wp_sender.waypoint_sender import WaypointClient
import rclpy

def generate_snake_pattern_waypoints(top_left_lat, top_left_lon, 
                                     box_width, box_height, 
                                     num_rows, num_cols, altitude):
    """
    Generate waypoints in snake pattern
    
    Args:
        top_left_lat: Top-left corner latitude
        top_left_lon: Top-left corner longitude
        box_width: Width of each grid box (degrees)
        box_height: Height of each grid box (degrees)
        num_rows: Number of rows
        num_cols: Number of columns
        altitude: Flight altitude (meters)
    """
    rclpy.init()
    client = WaypointClient()
    
    waypoint_index = 0
    
    for row in range(num_rows):
        # Determine direction: even rows = left→right, odd rows = right→left
        if row % 2 == 0:
            col_range = range(num_cols)
        else:
            col_range = range(num_cols - 1, -1, -1)
        
        for col in col_range:
            # Calculate waypoint position (center of grid box)
            lat = top_left_lat - (row * box_height + box_height/2)
            lon = top_left_lon + (col * box_width + box_width/2)
            
            # Add waypoint
            response = client.send_AddWP_request(lon, lat, altitude, waypoint_index)
            if response.success:
                print(f"Added waypoint {waypoint_index}: ({lat:.6f}, {lon:.6f})")
            else:
                print(f"Failed to add waypoint {waypoint_index}")
            
            waypoint_index += 1
    
    rclpy.shutdown()

# Example usage:
# generate_snake_pattern_waypoints(
#     top_left_lat=37.7800,
#     top_left_lon=-122.4200,
#     box_width=0.0005,  # ~55m at this latitude
#     box_height=0.0005,
#     num_rows=6,
#     num_cols=10,
#     altitude=15.0
# )
```

### 3. Run Complete System (4 Nodes)

**Terminal 1: Camera**
```bash
ros2 run video_cam image_pub
```

**Terminal 2: Detection**
```bash
ros2 run video_cam object_detection_sahi \
  --ros-args \
  -p confidence_threshold:=0.15 \
  -p slice_height:=512 \
  -p slice_width:=512
```

**Terminal 3: Tracker**
```bash
ros2 run video_cam waypoint_detection_tracker
```

**Terminal 4: Payload Planner**
```bash
ros2 run video_cam payload_drop_planner \
  --ros-args \
  -p min_confidence_threshold:=0.3 \
  -p drop_altitude:=10.0 \
  -p scan_waypoint_start:=0 \
  -p scan_waypoint_end:=59 \
  -p rtl_index:=60
```

### 4. Launch File (Recommended)

Create `launch/grid_scan_mission.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Camera node
        Node(
            package='video_cam',
            executable='image_pub',
            name='siyi_camera_publisher'
        ),
        
        # Detection node
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
        
        # Tracker node
        Node(
            package='video_cam',
            executable='waypoint_detection_tracker',
            name='waypoint_tracker_node'
        ),
        
        # Payload planner node
        Node(
            package='video_cam',
            executable='payload_drop_planner',
            name='payload_drop_planner_node',
            parameters=[{
                'min_confidence_threshold': 0.3,
                'drop_altitude': 10.0,
                'scan_waypoint_start': 0,
                'scan_waypoint_end': 59,
                'rtl_index': 60
            }]
        )
    ])
```

Then run:
```bash
ros2 launch video_cam grid_scan_mission.launch.py
```

## How It Works

### Step 1: Grid Scanning
- Drone flies through pre-planned waypoints in snake pattern
- At each waypoint, `image_pub_siyi` captures and saves image
- Image saved to `camera_feed/` directory

### Step 2: Detection Processing
- `object_detection_sahi` detects new images in `camera_feed/`
- Processes with SAHI + YOLO11s
- Publishes `ImageResult` with detections and waypoint_index

### Step 3: Tracking
- `waypoint_detection_tracker` receives `ImageResult`
- Extracts max confidence for mannequin and tent per waypoint
- Stores: `{wp_idx: {mannequin_conf, tent_conf}}`
- Publishes summary to `/waypoint_detection_summary`

### Step 4: Payload Planning
- `payload_drop_planner` subscribes to summary
- Finds waypoint with **highest mannequin confidence**
- Finds waypoint with **highest tent confidence**
- Adds return waypoints (before RTL) to those locations

### Step 5: Payload Drop
- Drone completes scan mission
- Returns to mannequin waypoint (drops payload)
- Returns to tent waypoint (drops payload)
- Returns to launch (RTL)

## Accessing Detection Data

### Option 1: Read JSON File
```python
import json

with open('waypoint_detections_latest.json', 'r') as f:
    data = json.load(f)

# Get simplified format
for wp_idx, wp_data in data['waypoints'].items():
    print(f"Waypoint {wp_idx}:")
    print(f"  Mannequin: {wp_data['mannequin_confidence']:.2f}")
    print(f"  Tent: {wp_data['tent_confidence']:.2f}")
```

### Option 2: Use Tracker Method
```python
# From within ROS2 node
from waypoint_detection_tracker import WaypointDetectionTracker

tracker = WaypointDetectionTracker()
# ... after processing ...
simplified_data = tracker.get_simplified_detection_data()
# Returns: {wp_idx: {'mannequin_confidence': float, 'tent_confidence': float}}
```

## Finding Max Confidence Waypoints

### Automatic (via payload_drop_planner)
The planner automatically:
1. Monitors detection data
2. Finds max confidence waypoints
3. Adds return waypoints

### Manual (from code)
```python
# From payload_drop_planner
planner = PayloadDropPlanner()
mannequin_wp, tent_wp = planner.find_max_confidence_waypoints()

print(f"Max mannequin: Waypoint {mannequin_wp}")
print(f"Max tent: Waypoint {tent_wp}")
```

## Mission Flow Example

```
1. Pre-flight: Generate 60 waypoints (6 rows × 10 cols) in snake pattern
2. Start mission: Drone flies waypoint sequence
3. At WP0: Take photo → Detect → Store (mannequin: 0.0, tent: 0.0)
4. At WP1: Take photo → Detect → Store (mannequin: 0.0, tent: 0.0)
5. At WP5: Take photo → Detect → Store (mannequin: 0.87, tent: 0.0)
6. At WP12: Take photo → Detect → Store (mannequin: 0.0, tent: 0.72)
7. ... continue scanning ...
8. After scan complete: Planner finds:
   - Max mannequin: WP5 (0.87)
   - Max tent: WP12 (0.72)
9. Adds return waypoints:
   - WP60: Return to WP5 location (mannequin drop)
   - WP61: Return to WP12 location (tent drop)
10. Drone executes return mission and drops payloads
```

## Configuration Parameters

### waypoint_detection_tracker
- Auto-saves every 30 seconds
- Output: `waypoint_detection_data/waypoint_detections_latest.json`

### payload_drop_planner
- `min_confidence_threshold`: Minimum confidence to consider (default: 0.3)
- `drop_altitude`: Altitude for payload drop in meters (default: 10.0)
- `scan_waypoint_start`: First waypoint index of scan grid (default: 0)
- `scan_waypoint_end`: Last waypoint index of scan grid (default: -1 = auto)
- `rtl_index`: Return-to-launch waypoint index (where to insert return waypoints)

## Troubleshooting

### No detections being tracked
- Verify `object_detection_sahi` is publishing to `/image_detections`
- Check that images are being saved to `camera_feed/`
- Ensure `waypoint_detection_tracker` is running

### Payload waypoints not being added
- Check that `min_confidence_threshold` is not too high
- Verify waypoint locations are available (GPS data)
- Check that `scan_waypoint_end` includes your scan waypoints
- Ensure `rtl_index` is set correctly

### Confidence values are 0
- Check that detections are actually happening (look at detection logs)
- Verify class names match ('person' or 'mannequin' for mannequin, 'tent' for tent)
- Check `confidence_threshold` in `object_detection_sahi` (should be low enough)

## Output Files

### Detection Data
- Location: `install/video_cam/share/video_cam/waypoint_detection_data/`
- Files:
  - `waypoint_detections_latest.json` - Auto-updated every 30s
  - `waypoint_detections_YYYYMMDD_HHMMSS.json` - Timestamped saves

### Example Output
```json
{
  "metadata": {
    "timestamp": "2025-01-24T12:34:56",
    "total_waypoints": 60,
    "total_detections": 45
  },
  "waypoints": {
    "5": {
      "location": {"lat": 37.7749, "lon": -122.4194, "alt": 15.0},
      "mannequin_confidence": 0.87,
      "tent_confidence": 0.0,
      "detections": [...],
      "images": ["photo_20250124_123400.jpg"]
    },
    "12": {
      "location": {"lat": 37.7750, "lon": -122.4195, "alt": 15.0},
      "mannequin_confidence": 0.0,
      "tent_confidence": 0.72,
      "detections": [...],
      "images": ["photo_20250124_123415.jpg"]
    }
  }
}
```

This system provides a complete end-to-end solution for grid-based scanning with automatic payload drop planning!

