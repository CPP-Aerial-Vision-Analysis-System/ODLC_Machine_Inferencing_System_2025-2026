# Package Communication Flow

## Overview
This document explains how all packages in `/src` communicate with each other through ROS2 topics, services, and messages.

---

## Communication Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    MAVROS (Flight Controller)                   │
│  └─> Provides: /mavros/* topics and services                    │
└─────────────────────────────────────────────────────────────────┘
                              ▲ │
                              │ │
                    ┌─────────┴─┴─────────┐
                    │                     │
                    ▼                     ▼
        ┌──────────────────┐    ┌──────────────────┐
        │  waypoint_mavros │    │    gps_ros2      │
        │  (waypoint.py)   │    │    (gps.py)      │
        └──────────────────┘    └──────────────────┘
                    │                     │
                    │                     │
        ┌───────────┴──────────┐          │
        │                      │          │
        ▼                      ▼          ▼
┌───────────────┐    ┌──────────────┐     │
│  wp_sender    │    │ video_cam    │     │
│(waypoint_     │    │(object_      │     │
│ sender.py)    │    │ detection_   │     │
└───────────────┘    │ sahi.py)     │     │
                     └──────────────┘     │
                             │            │
                             │            │
                             ▼            ▼
                    ┌────────────────────────┐
                    │  ultralytics_ros       │
                    │  (yolo_result_sub.py)  │
                    └────────────────────────┘
```

---

## Package-by-Package Communication

### 1. **waypoint_mavros** (Waypoint Manager)

**Role:** Core waypoint management server - interfaces with MAVROS

**Subscribes to (listens):**
- `/mavros/state` (State) - Drone connection status
- `/mavros/mission/reached` (WaypointReached) - Waypoint arrival notifications
- `/mavros/mission/waypoints` (WaypointList) - Current waypoint list from drone

**Publishes to:**
- `/mavros/statustext/send` (StatusText) - Status messages to ground control station

**Provides Services (others can call):**
- `/addWaypoint` (AddWaypoint) - Add waypoint to mission
- `/delWaypoint` (DelWaypoint) - Delete waypoint from mission
- `/updateMission` (UpdateMission) - Update mission parameters

**Uses MAVROS Services (calls):**
- `/mavros/mission/pull` (WaypointPull) - Get waypoints from drone
- `/mavros/mission/push` (WaypointPush) - Send waypoints to drone
- `/mavros/mission/clear` (WaypointClear) - Clear all waypoints
- `/mavros/set_mode` (SetMode) - Change flight mode

**Communicates with:**
- `wp_sender` → via `/addWaypoint` and `/delWaypoint` services
- `video_cam` → indirectly via `/mavros/mission/reached` topic (both subscribe)
- `ultralytics_ros` → indirectly via `/mavros/mission/reached` topic

---

### 2. **wp_sender** (Waypoint Client)

**Role:** Client for modifying waypoints

**Provides Services:**
- None (this is a client-only package)

**Uses Services (calls):**
- `/addWaypoint` (AddWaypoint) - Calls waypoint_mavros service
- `/delWaypoint` (DelWaypoint) - Calls waypoint_mavros service
- `/updateMission` (UpdateMission) - Calls waypoint_mavros service (commented out)

**Communicates with:**
- `waypoint_mavros` → via `/addWaypoint` and `/delWaypoint` services

**Note:** This package also includes `parameter.py` which can get parameters from `waypoint_manager` node.

---

### 3. **gps_ros2** (GPS Service)

**Role:** GPS data broker - provides current drone position

**Subscribes to (listens):**
- `/mavros/state` (State) - Drone connection status
- `/mavros/global_position/global` (NavSatFix) - GPS coordinates
- `/mavros/global_position/compass_hdg` (Float64) - Heading/yaw

**Publishes to:**
- None (service-based, not topic-based)

**Provides Services:**
- `/get_drone_data` (GetGPSData) - Returns current GPS position and heading

**Uses MAVROS Services:**
- `/mavros/set_stream_rate` (StreamRate) - Configure data stream rate

**Communicates with:**
- `ultralytics_ros` → via `/get_drone_data` service (yolo_result_sub.py calls it)

---

### 4. **video_cam** (Object Detection)

**Role:** Detects objects in images and publishes results

**Subscribes to (listens):**
- `/mavros/mission/reached` (WaypointReached) - To tag detections with waypoint index

**Publishes to:**
- `/sahi_detection_results` (Image) - Annotated images with detections
- `/sahi_detection_info` (String) - Detection metadata (JSON string)
- `/image_detections` (ImageResult) - **Main output** - Structured detection results

**Provides Services:**
- None

**Uses Services:**
- None

**Communicates with:**
- `ultralytics_ros` → via `/image_detections` topic (yolo_result_sub.py subscribes)
- `waypoint_mavros` → indirectly via `/mavros/mission/reached` topic (both subscribe)

**Note:** This package reads images from file system (`camera_feed/`) and writes results to `detection_results_sahi/`

---

### 5. **ultralytics_ros** (Mission Planner)

**Role:** Processes detection results and creates waypoints for detected objects

**Subscribes to (listens):**
- `/yolo_result` (YoloResult) - From tracker_node (if used)
- `/image_detection` (ImageResult) - **Main input** from video_cam
- `/mavros/mission/waypoints` (WaypointList) - Current waypoint list
- `/mavros/mission/reached` (WaypointReached) - Waypoint arrival notifications
- `/mavros/vfr_hub` (VfrHud) - Airspeed and ground speed
- `/mavros/statustext/recv` (StatusText) - Status messages from GCS
- `/yolo_image` (Image) - Processed YOLO images (if used)
- `/mavros/global_position/global` (NavSatFix) - GPS for geofence checking

**Publishes to:**
- `/mavros/statustext/send` (StatusText) - Status messages to GCS
- `/camera/object_detected` (Bool) - Notification when objects detected

**Uses Services (calls):**
- `/get_drone_data` (GetGPSData) - Gets current GPS from gps_ros2
- `/mavros/mission/set_current` (WaypointSetCurrent) - Jump to specific waypoint
- `/mavros/mission/pull` (WaypointPull) - Get waypoints (has typo: "/mavross/mission/pull")
- `/mavros/set_mode` (SetMode) - Change flight mode
- `/AddWaypoint` (AddWaypoint) - Add waypoint (note: different case than waypoint_mavros)
- `/DelWaypoint` (DelWaypoint) - Delete waypoint (note: different case)

**Communicates with:**
- `video_cam` → via `/image_detection` topic (subscribes to ImageResult messages)
- `gps_ros2` → via `/get_drone_data` service
- `waypoint_mavros` → indirectly via `/mavros/mission/reached` and `/mavros/mission/waypoints` topics

**Note:** This package calculates GPS coordinates for detected objects and can add waypoints to revisit them.

---

### 6. **interfaces** (Message/Service Definitions)

**Role:** Defines shared message and service types

**Provides:**
- **Messages:**
  - `ImageResult.msg` - Detection results with waypoint index
  - `YoloResult.msg` - YOLO detection results

- **Services:**
  - `AddWaypoint.srv` - Add waypoint request
  - `DelWaypoint.srv` - Delete waypoint request
  - `GetGPSData.srv` - Get GPS data request
  - `UpdateMission.srv` - Update mission request

**Used by:**
- All other packages import these definitions

---

## Data Flow Examples

### Example 1: Object Detection → Mission Planning

```
1. video_cam (object_detection_sahi.py)
   ├─> Reads image from camera_feed/
   ├─> Processes with SAHI + YOLO11s
   ├─> Tags with waypoint_index from /mavros/mission/reached
   └─> Publishes ImageResult to /image_detections

2. ultralytics_ros (yolo_result_sub.py)
   ├─> Subscribes to /image_detections
   ├─> Receives ImageResult with detections
   ├─> Calls /get_drone_data service (gps_ros2) to get current GPS
   ├─> Calculates GPS coordinates for detected objects
   └─> Calls /AddWaypoint service (waypoint_mavros) to add waypoint

3. waypoint_mavros (waypoint.py)
   ├─> Receives /AddWaypoint service call
   ├─> Validates and inserts waypoint
   ├─> Calls /mavros/mission/push to send to drone
   └─> Drone receives new waypoint in mission
```

### Example 2: Waypoint Tracking

```
1. Drone reaches waypoint
   └─> MAVROS publishes WaypointReached to /mavros/mission/reached

2. Multiple nodes subscribe:
   ├─> waypoint_mavros (waypoint.py)
   │   └─> Updates self.waypoint_reached
   │
   ├─> video_cam (object_detection_sahi.py)
   │   └─> Updates self.waypoint_reached (tags next detection)
   │
   └─> ultralytics_ros (yolo_result_sub.py)
       └─> Updates waypoint tracking for mission planning
```

### Example 3: GPS Data Request

```
1. ultralytics_ros (yolo_result_sub.py)
   └─> Calls /get_drone_data service

2. gps_ros2 (gps.py)
   ├─> Receives service call
   ├─> Reads latest GPS from /mavros/global_position/global (already subscribed)
   ├─> Reads latest heading from /mavros/global_position/compass_hdg
   └─> Returns GPS data to caller

3. ultralytics_ros
   └─> Uses GPS data to calculate object locations
```

---

## Communication Patterns

### 1. **Service-Based (Request-Response)**
- `wp_sender` → `waypoint_mavros` (AddWaypoint, DelWaypoint)
- `ultralytics_ros` → `gps_ros2` (GetGPSData)
- `ultralytics_ros` → `waypoint_mavros` (AddWaypoint, DelWaypoint)

### 2. **Topic-Based (Publish-Subscribe)**
- `video_cam` → `ultralytics_ros` (/image_detections)
- `waypoint_mavros` ← MAVROS (/mavros/mission/reached)
- `video_cam` ← MAVROS (/mavros/mission/reached)
- `ultralytics_ros` ← MAVROS (/mavros/mission/reached)

### 3. **File System-Based**
- `video_cam` reads from `camera_feed/` directory
- `video_cam` writes to `detection_results_sahi/` directory

---

## Potential Issues Found

1. **Service Name Mismatch:**
   - `waypoint_mavros` provides: `/addWaypoint` (lowercase 'a')
   - `ultralytics_ros` calls: `/AddWaypoint` (uppercase 'A')
   - This will cause service calls to fail!

2. **Typo in ultralytics_ros:**
   - Line 101: `/mavross/mission/pull` (extra 's')
   - Should be: `/mavros/mission/pull`

3. **Missing Import:**
   - `object_detection_sahi.py` uses `WaypointReached` but doesn't import it
   - Should add: `from mavros_msgs.msg import WaypointReached`

---

## Summary

The system follows a **pipeline architecture**:

1. **Input:** Images from camera_feed/ directory
2. **Processing:** video_cam detects objects and tags with waypoint
3. **Planning:** ultralytics_ros processes detections and creates waypoints
4. **Management:** waypoint_mavros manages mission waypoints
5. **Execution:** MAVROS executes mission on drone

All packages communicate through ROS2 topics and services, with `interfaces` package providing shared message/service definitions.

