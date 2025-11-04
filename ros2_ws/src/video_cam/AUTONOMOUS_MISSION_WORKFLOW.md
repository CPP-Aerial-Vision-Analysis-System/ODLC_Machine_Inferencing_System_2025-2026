# Autonomous Mission Workflow

## Your Planned Workflow

1. Create waypoints in ArduPilot
2. Fly over waypoints autonomously
3. Detect objects at each waypoint
4. Find waypoints with highest confidence
5. Send drone back to those waypoints autonomously

**This is exactly what the system does!** Here's how:

---

## Complete Autonomous Mission Flow

### Phase 1: Mission Planning (Before Flight)

**You create waypoints in ArduPilot Mission Planner:**
- Plan your grid pattern (snake pattern covering yellow box)
- Upload waypoints to flight controller
- Mission includes: Takeoff → Scan waypoints → RTL

**System Setup:**
- Start ROS2 nodes (see below)
- Nodes are ready to receive waypoint data from MAVROS

---

### Phase 2: Autonomous Scanning (During Flight)

**What Happens:**

1. **Drone Takes Off** (ArduPilot)
   - Flight controller executes mission
   - MAVROS publishes waypoint list to `/mavros/mission/waypoints`

2. **Drone Reaches Waypoint** (ArduPilot)
   - ArduPilot publishes `WaypointReached` to `/mavros/mission/reached`
   - `object_detection_sahi` node updates `waypoint_reached` variable

3. **Camera Captures Image** (`image_pub_siyi.py`)
   - Takes photo at waypoint
   - Saves to `camera_feed/` directory
   - Image filename: `photo_YYYYMMDD-HHMMSS.jpg`

4. **Detection Processes Image** (`object_detection_sahi.py`)
   - Detects new image in `camera_feed/`
   - Processes with SAHI + YOLO11s
   - Tags with `waypoint_index` from step 2
   - Publishes `ImageResult` to `/image_detections`

5. **Tracker Stores Detection** (`waypoint_detection_tracker.py`)
   - Receives `ImageResult` message
   - Extracts mannequin and tent confidence
   - Stores: `{waypoint_index: {mannequin_confidence, tent_confidence}}`
   - Publishes summary to `/waypoint_detection_summary`

6. **Repeat for Each Waypoint**
   - Drone continues autonomously through mission
   - Each waypoint: capture → detect → store

---

### Phase 3: Payload Drop Planning (After Scan Complete)

**What Happens:**

1. **Payload Planner Analyzes Data** (`payload_drop_planner.py`)
   - Monitors `/waypoint_detection_summary` topic
   - Finds waypoint with **highest mannequin confidence**
   - Finds waypoint with **highest tent confidence**

2. **Planner Adds Return Waypoints**
   - Calls `/addWaypoint` service (waypoint_mavros)
   - Inserts waypoints **before RTL** in mission
   - Waypoint 1: Return to mannequin location
   - Waypoint 2: Return to tent location
   - Then: Continue to RTL

3. **Waypoint Manager Updates Mission** (`waypoint_mavros/waypointv2.py`)
   - Receives waypoint addition requests
   - Pulls current mission from flight controller
   - Inserts new waypoints at specified index
   - Pushes updated mission back to flight controller

---

### Phase 4: Autonomous Return Mission (After Scan)

**What Happens:**

1. **Drone Completes Scan Mission**
   - Reaches last scan waypoint
   - Flight controller sees new waypoints in mission

2. **Drone Executes Return Waypoints** (ArduPilot)
   - Autonomous flight to mannequin waypoint
   - Drops payload at mannequin location
   - Autonomous flight to tent waypoint  
   - Drops payload at tent location
   - Returns to launch (RTL)

**All autonomous - no manual intervention needed!**

---

## How to Run the Complete System

### Step 1: Pre-Flight Setup

```bash
# Build package
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```

### Step 2: Create Waypoints in ArduPilot

- Use Mission Planner or QGroundControl
- Create waypoints in your grid pattern
- Upload to flight controller
- Mission should end with RTL waypoint

### Step 3: Start ROS2 Nodes (Before Takeoff)

**Terminal 1: Camera Node**
```bash
ros2 run video_cam image_pub
```

**Terminal 2: Detection Node**
```bash
ros2 run video_cam object_detection_sahi \
  --ros-args \
  -p confidence_threshold:=0.15
```

**Terminal 3: Tracker Node**
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

**Parameters:**
- `scan_waypoint_start`: First waypoint index of your scan grid
- `scan_waypoint_end`: Last waypoint index of your scan grid
- `rtl_index`: Return-to-launch waypoint index (where to insert return waypoints)

### Step 4: Start Mission

- Arm and takeoff from Mission Planner
- Mission executes autonomously
- System automatically:
  - Captures images
  - Detects objects
  - Tracks confidence per waypoint
  - Plans return waypoints
  - Executes payload drop

---

## System Architecture

```
┌─────────────────────────────────┐
│  ArduPilot Flight Controller    │
│  - Executes waypoint mission    │
│  - Publishes waypoint data      │
└──────────────┬──────────────────┘
               │
               ▼
┌─────────────────────────────────┐
│  MAVROS Bridge                  │
│  - /mavros/mission/reached      │
│  - /mavros/mission/waypoints    │
└──────────────┬──────────────────┘
               │
       ┌───────┴────────┐
       │                 │
       ▼                 ▼
┌─────────────┐  ┌──────────────────┐
│ image_pub   │  │ object_detection │
│ (camera)    │  │ _sahi            │
└──────┬──────┘  └────────┬─────────┘
       │                  │
       └────────┬──────────┘
                │
                ▼
┌──────────────────────────────┐
│ waypoint_detection_tracker   │
│ - Tracks detections per WP   │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ payload_drop_planner         │
│ - Finds max confidence WPs   │
│ - Adds return waypoints      │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│ waypoint_mavros              │
│ - Manages waypoints          │
│ - Updates mission            │
└──────────────┬───────────────┘
               │
               ▼
┌──────────────────────────────┐
│  ArduPilot (updated mission) │
│  - Executes return waypoints │
└──────────────────────────────┘
```

---

## Data Flow

### During Scan:
```
Waypoint 5 Reached
  ↓
Camera captures image → camera_feed/photo_xxx.jpg
  ↓
Detection processes → Finds person (confidence: 0.87)
  ↓
Tracker stores: WP5 → {mannequin: 0.87, tent: 0.0}
  ↓
(Continues for all waypoints...)
```

### After Scan:
```
All waypoints scanned
  ↓
Payload planner analyzes all data
  ↓
Finds: Max mannequin = WP5 (0.87), Max tent = WP12 (0.72)
  ↓
Calls /addWaypoint service:
  - Adds WP60: Return to WP5 location
  - Adds WP61: Return to WP12 location
  ↓
waypoint_mavros pushes updated mission to flight controller
  ↓
Drone executes return waypoints autonomously
```

---

## Key Features

### Fully Autonomous
- No manual waypoint selection needed
- System automatically finds best locations
- Mission updates happen in-flight

### Real-time Processing
- Detections happen as drone flies
- Data accumulates per waypoint
- Planning happens after scan completes

### Safety Features
- Waypoint validation (won't insert invalid waypoints)
- Mission structure protection (won't delete takeoff/RTL)
- Confidence threshold filtering

### Data Persistence
- All detection data saved to JSON
- Waypoint locations stored
- Full history for analysis

---

## Configuration

### Payload Planner Parameters

```python
min_confidence_threshold: 0.3  # Minimum confidence to consider
drop_altitude: 10.0            # Altitude for payload drop (meters)
scan_waypoint_start: 0         # First scan waypoint index
scan_waypoint_end: 59          # Last scan waypoint index  
rtl_index: 60                  # RTL waypoint index (insert before this)
```

### Detection Parameters

```python
confidence_threshold: 0.15     # Detection sensitivity (lower = more detections)
slice_height: 512              # SAHI slice size
slice_width: 512
overlap_height_ratio: 0.3      # Overlap between slices
```

---

## Example Mission Timeline

```
T+0:00  - Takeoff
T+0:30  - Reached WP0 → Capture → Detect → Store
T+0:33  - Reached WP1 → Capture → Detect → Store
T+0:36  - Reached WP2 → Capture → Detect → Store
...
T+3:00  - Reached WP59 (last scan waypoint)
T+3:03  - Payload planner analyzes all data
T+3:04  - Finds max confidence: WP5 (mannequin), WP12 (tent)
T+3:05  - Adds return waypoints to mission
T+3:06  - Drone sees new waypoints in mission
T+3:10  - Drone flies to WP5 location (mannequin)
T+3:20  - Drops payload at WP5
T+3:25  - Drone flies to WP12 location (tent)
T+3:35  - Drops payload at WP12
T+3:40  - RTL
```

---

## Verification

### Check Detection Data
```bash
cat install/video_cam/share/video_cam/waypoint_detection_data/waypoint_detections_latest.json
```

### Monitor Topics
```bash
# Watch waypoint progress
ros2 topic echo /mavros/mission/reached

# Watch detections
ros2 topic echo /image_detections

# Watch summary
ros2 topic echo /waypoint_detection_summary
```

### Check Logs
Each node logs:
- Waypoint reached notifications
- Detection results
- Confidence values
- Waypoint addition confirmations

---

## Summary

**Our workflow is fully supported!** The system:

1. Accepts waypoints from ArduPilot (via MAVROS)
2. Tracks waypoint progress automatically
3. Captures and processes images at each waypoint
4. Finds highest confidence waypoints automatically
5. Adds return waypoints to mission autonomously
6. Drone executes return mission without manual intervention

**Everything is autonomous from takeoff to payload drop!**

The only manual steps are:
- Creating initial waypoints in Mission Planner
- Starting ROS2 nodes
- Arming and starting mission

After that, the system handles everything automatically.