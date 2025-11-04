# Phase 2 Files Breakdown - Autonomous Scanning

This document provides a comprehensive breakdown of every file involved in Phase 2: Autonomous Scanning (During Flight). This includes all files, no matter how big or small, that are responsible for the autonomous scanning workflow.

---

## Files Responsible for Phase 2 (Autonomous Scanning)

### Step 1: Drone Takes Off - Waypoint List Published

**External Components:**
- **ArduPilot Flight Controller**: Executes mission, manages waypoints
- **MAVROS Bridge** (external ROS2 package): Bridges ArduPilot ↔ ROS2

**Our Code Files:**

1. **`waypoint_mavros/waypoint_mavros/waypointv2.py`** (Lines 36-37, 81)
   - **Function**: Subscribes to `/mavros/mission/waypoints` topic
   - **Method**: `waypoints_list()` callback (line 81)
   - **Purpose**: Receives and stores waypoint list from MAVROS
   - **Data**: Stores `WaypointList` in `self.waypoint_list`

2. **`waypoint_mavros/setup.py`**
   - **Function**: Build configuration for waypoint_mavros package
   - **Purpose**: Defines entry point `waypoint=waypoint_mavros.waypointv2:main`

3. **`waypoint_mavros/package.xml`**
   - **Function**: Package metadata and dependencies
   - **Dependencies**: `mavros_msgs` (for WaypointList message type)

4. **`waypoint_mavros/resource/waypoint_mavros`**
   - **Function**: Package resource marker file
   - **Purpose**: Identifies package for ROS2 build system

5. **`waypoint_mavros/setup.cfg`**
   - **Function**: Setuptools configuration
   - **Purpose**: Python package build settings

**Message Types Used (from mavros_msgs package):**
- `mavros_msgs/msg/WaypointList.msg` - Waypoint list structure
- `mavros_msgs/msg/Waypoint.msg` - Individual waypoint structure

---

### Step 2: Drone Reaches Waypoint - Waypoint Reached Notification

**Our Code Files:**

1. **`video_cam/video_cam/object_detection_sahi.py`** (Lines 150-156)
   - **Function**: Subscribes to `/mavros/mission/reached` topic
   - **Method**: `waypoint_reached_cb()` callback (line 154)
   - **Purpose**: Updates `self.waypoint_reached` with current waypoint index
   - **Line 151**: `self.waypoint_reached = 0` (initialization)
   - **Line 152**: Subscribes to `WaypointReached` messages
   - **Line 155**: Stores `msg.wp_seq` in `self.waypoint_reached`

2. **`video_cam/video_cam/object_detection_sahi.py`** (Line 27)
   - **Import**: `from mavros_msgs.msg import WaypointReached`
   - **Purpose**: Imports message type for waypoint reached notifications

3. **`waypoint_mavros/waypoint_mavros/waypointv2.py`** (Lines 36, 314)
   - **Function**: Also subscribes to waypoint reached (for waypoint management)
   - **Method**: `waypoint_reached_cb()` callback (line 314)
   - **Purpose**: Tracks waypoint progress for waypoint management

**Message Types Used:**
- `mavros_msgs/msg/WaypointReached.msg` - Waypoint reached notification
  - Contains: `wp_seq` (waypoint sequence number)

---

### Step 3: Camera Captures Image

**Our Code Files:**

1. **`video_cam/video_cam/image_pub_siyi.py`** (Main file - 304 lines)
   - **Class**: `SiyiA8Publisher` (line 21)
   - **Node Name**: `siyi_a8_publisher` (line 23)
   
   **Key Methods:**
   - **`__init__()`** (Lines 22-106): Initialization
     - Line 42: Sets `camera_feed` directory path
     - Line 62: Creates timer for camera loop (0.1s interval)
   - **`camera_loop()`** (Lines 239-294): Main capture loop
     - Line 240: Checks if camera enabled (altitude threshold)
     - Line 246: Captures frame from camera
     - Line 259: Saves image to `camera_feed/` with timestamp
   - **`check_altitude()`** (Lines 224-237): Enables camera above threshold
   - **`cv2_to_imgmsg_manual()`** (Lines 188-197): Image conversion fallback

   **Subscribes To:**
   - `/camera/trigger` (Bool) - Manual trigger (line 30)
   - `/mavros/global_position/rel_alt` (Float64) - Altitude check (line 31)
   - `/webcam/image_raw` (Image) - Simulation mode (line 32)

   **Publishes To:**
   - `image_raw` (Image) - Raw camera images (line 26)
   - `/mavros/statustext/send` (StatusText) - Status messages (line 27)

2. **`video_cam/setup.py`** (Line 25)
   - **Entry Point**: `image_pub = video_cam.image_pub_siyi:main`
   - **Purpose**: Makes `ros2 run video_cam image_pub` work

3. **`video_cam/package.xml`**
   - **Dependencies**: 
     - `cv_bridge` (for image conversion)
     - `sensor_msgs` (for Image messages)
     - `mavros_msgs` (for StatusText)

**Supporting Files:**
- **`cv_bridge` package** (external): Converts OpenCV ↔ ROS Image messages
- **OpenCV library** (external): Image capture and processing

**File System:**
- **`install/video_cam/share/video_cam/camera_feed/`** - Directory where images are saved
  - Created automatically by `image_pub_siyi.py` (lines 42-44)

---

### Step 4: Detection Processes Image

**Our Code Files:**

1. **`video_cam/video_cam/object_detection_sahi.py`** (Main file - 875 lines)
   - **Class**: `SAHIObjectDetectionNode` (line 65)
   - **Node Name**: `sahi_object_detection_node` (line 68)
   
   **Key Methods:**
   - **`__init__()`** (Lines 66-153): Initialization
     - Lines 70-88: Parameter declarations and retrieval
     - Line 97: Publishes to `/image_detections`
     - Line 100: Sets `camera_feed` directory path
     - Line 139: Creates timer to check for new images
     - Line 152: Subscribes to waypoint reached
   - **`check_for_new_images()`** (Lines 307-346): Monitors `camera_feed/`
     - Line 310: Checks if directory exists
     - Lines 319-322: Finds image files (.jpg, .jpeg, .png, .bmp)
     - Lines 334-339: Processes new images
   - **`process_image()`** (Lines 348-394): Processes single image
     - Line 354: Loads image with OpenCV
     - Line 363: Calls `detect_objects_sahi()`
     - Line 371: Publishes results
   - **`detect_objects_sahi()`** (Lines 396-458): SAHI detection
     - Lines 416-428: Runs SAHI sliced prediction
     - Lines 433-448: Converts results to detection format
   - **`_categorize_detection()`** (Lines 460-524): Categorizes detections
     - Lines 476-486: Person detection logic
     - Lines 488-523: Tent detection logic (maps various YOLO classes)
   - **`publish_results()`** (Lines 706-815): Publishes detection results
     - Lines 762-771: Creates `ImageResult` message
     - Line 771: Sets `waypoint_index` from `self.waypoint_reached`
     - Line 803: Publishes to `/image_detections` topic

2. **`video_cam/video_cam/object_detection_sahi.py`** (Lines 25-47)
   - **Imports**: SAHI and YOLO dependencies
   - Lines 27-32: Imports SAHI library
   - Lines 35-46: Imports YOLO (Ultralytics)
   - Line 27: Imports `WaypointReached` from mavros_msgs

3. **`video_cam/video_cam/yolo11s.pt`**
   - **Function**: YOLO11s model weights file
   - **Purpose**: Neural network model for object detection
   - **Size**: ~43MB (65812 lines)
   - **Usage**: Loaded by SAHI AutoDetectionModel (line 278)

4. **`video_cam/setup.py`** (Line 28)
   - **Entry Point**: `object_detection_sahi = video_cam.object_detection_sahi:main`

5. **`video_cam/package.xml`**
   - **Dependencies**: All standard ROS2 message types

6. **`video_cam/requirements_sahi.txt`** (if exists)
   - **Function**: Python dependencies for SAHI
   - **Purpose**: Lists required packages (sahi, ultralytics, torch, etc.)

**Message Types Used:**
- **`interfaces/msg/ImageResult.msg`** (our custom message):
  - Contains: detections, waypoint_index, confidence, classes, etc.
  - Defined in: `ros2_ws/src/interfaces/msg/ImageResult.msg`

**External Dependencies:**
- **SAHI library**: Slicing Aided Hyper Inference
- **Ultralytics YOLO**: Object detection model
- **PyTorch**: Deep learning framework
- **OpenCV**: Image processing

**File System:**
- **`install/video_cam/share/video_cam/camera_feed/`** - Reads images from here
- **`install/video_cam/share/video_cam/detection_results_sahi/`** - Saves annotated images here

---

### Step 5: Tracker Stores Detection

**Our Code Files:**

1. **`video_cam/video_cam/waypoint_detection_tracker.py`** (Main file - 377 lines)
   - **Class**: `WaypointDetectionTracker` (line 38)
   - **Node Name**: `waypoint_detection_tracker` (line 40)
   
   **Key Methods:**
   - **`__init__()`** (Lines 39-93): Initialization
     - Lines 43-52: Data structure initialization
     - Lines 55-60: Subscribes to `/image_detections`
     - Lines 62-67: Subscribes to `/mavros/mission/waypoints`
     - Lines 70-74: Publishes to `/waypoint_detection_summary`
     - Line 84: Auto-save timer (30 seconds)
   - **`waypoints_callback()`** (Lines 95-107): Receives waypoint list
     - Line 98: Stores waypoint list
     - Lines 101-107: Updates location data for known waypoints
   - **`image_result_callback()`** (Lines 109-176): Processes detections
     - Line 111: Gets waypoint_index from message
     - Lines 113-124: Gets waypoint location
     - Lines 126-153: Extracts detections from ImageResult
     - Lines 158-161: Adds detections to waypoint data
     - Line 165: Updates max confidence for mannequin/tent
     - Line 176: Publishes summary
   - **`_update_max_confidence()`** (Lines 178-196): Updates confidence
     - Lines 185-189: Updates mannequin confidence
     - Lines 192-196: Updates tent confidence
   - **`get_detection_summary()`** (Lines 198-232): Generates summary
   - **`publish_summary()`** (Lines 234-238): Publishes JSON summary
   - **`save_data_to_json()`** (Lines 240-265): Saves to JSON file

2. **`video_cam/video_cam/waypoint_detection_tracker.py`** (Lines 25-35)
   - **Imports**:
     - `interfaces.msg.ImageResult` (line 27)
     - `mavros_msgs.msg.WaypointList` (line 28)
     - `std_msgs.msg.String` (line 29)

3. **`video_cam/setup.py`** (Line 29)
   - **Entry Point**: `waypoint_detection_tracker = video_cam.waypoint_detection_tracker:main`

**Message Types Used:**
- **`interfaces/msg/ImageResult.msg`**: Input from detection node
- **`mavros_msgs/msg/WaypointList.msg`**: Waypoint locations
- **`std_msgs/msg/String.msg`**: JSON summary output

**File System:**
- **`install/video_cam/share/video_cam/waypoint_detection_data/`** - Saves JSON files here
  - Created automatically (line 81)

---

### Step 6: Repeat for Each Waypoint

**All files from Steps 1-5 are involved in the loop.**

**Additional Supporting Files:**

1. **`interfaces/msg/ImageResult.msg`** (24 lines)
   - **Purpose**: Custom message definition for detection results
   - **Fields**: 
     - `waypoint_index` (int32) - Waypoint where image was captured
     - `detections` (Detection2DArray) - Object detections
     - `classes[]` (string[]) - Detection classes
     - `confidences[]` (float32[]) - Confidence scores
     - `image_name` (string) - Source image filename
   - **Used By**: 
     - `object_detection_sahi.py` (publishes)
     - `waypoint_detection_tracker.py` (subscribes)

2. **`interfaces/CMakeLists.txt`**
   - **Purpose**: Build configuration for interfaces package
   - **Function**: Generates Python/C++ bindings for messages

3. **`interfaces/package.xml`**
   - **Purpose**: Package metadata for interfaces
   - **Dependencies**: `std_msgs`, `sensor_msgs`, `vision_msgs`

4. **`video_cam/package.xml`**
   - **Purpose**: Package metadata for video_cam
   - **Dependencies**: `interfaces`, `cv_bridge`, `mavros_msgs`

5. **`video_cam/setup.py`**
   - **Purpose**: Python package setup
   - **Function**: Defines entry points for all executables

6. **`video_cam/resource/video_cam`**
   - **Purpose**: Package resource marker
   - **Function**: Identifies package for ROS2 build system

---

## Complete File List for Phase 2

### Core Processing Files (Python Nodes):
1. **`video_cam/video_cam/image_pub_siyi.py`** (304 lines) - Camera capture
2. **`video_cam/video_cam/object_detection_sahi.py`** (875 lines) - Object detection
3. **`video_cam/video_cam/waypoint_detection_tracker.py`** (377 lines) - Detection tracking
4. **`waypoint_mavros/waypoint_mavros/waypointv2.py`** (478 lines) - Waypoint management

### Message/Service Definitions:
5. **`interfaces/msg/ImageResult.msg`** (24 lines) - Detection results message
6. **`interfaces/msg/YoloResult.msg`** (4 lines) - YOLO results (legacy)
7. **`interfaces/srv/AddWaypoint.srv`** (6 lines) - Add waypoint service
8. **`interfaces/srv/DelWaypoint.srv`** (3 lines) - Delete waypoint service
9. **`interfaces/srv/UpdateMission.srv`** (2 lines) - Update mission service

### Package Configuration Files:
10. **`video_cam/package.xml`** (26 lines) - Package metadata
11. **`video_cam/setup.py`** (34 lines) - Python package setup
12. **`video_cam/resource/video_cam`** (1 line) - Resource marker
13. **`waypoint_mavros/package.xml`** (18 lines) - Package metadata
14. **`waypoint_mavros/setup.py`** (26 lines) - Python package setup
15. **`waypoint_mavros/resource/waypoint_mavros`** (1 line) - Resource marker
16. **`interfaces/package.xml`** (26 lines) - Package metadata
17. **`interfaces/CMakeLists.txt`** (38 lines) - Build configuration

### Model Files:
18. **`video_cam/video_cam/yolo11s.pt`** (65,812 lines) - YOLO11s model weights

### Launch Files (Optional):
19. **`video_cam/launch/sahi_detection.launch.py`** - Launch file for detection node

### External Dependencies (Not in our codebase):
- **MAVROS package** (external): Provides `/mavros/mission/reached`, `/mavros/mission/waypoints`
- **cv_bridge package**: Image conversion
- **SAHI library**: Python package for slicing inference
- **Ultralytics YOLO**: Python package for object detection
- **PyTorch**: Deep learning framework

---

## Data Flow Through Files

```
ArduPilot → MAVROS → waypointv2.py (waypoints_list callback)
                              ↓
                    Stores in self.waypoint_list
                              ↓
        waypointv2.py (waypoint_reached_cb callback)
                    ↓
        object_detection_sahi.py (waypoint_reached_cb)
                    ↓
        Stores in self.waypoint_reached
                    ↓
        image_pub_siyi.py (camera_loop)
                    ↓
        Saves to camera_feed/photo_xxx.jpg
                    ↓
        object_detection_sahi.py (check_for_new_images)
                    ↓
        object_detection_sahi.py (process_image)
                    ↓
        object_detection_sahi.py (detect_objects_sahi)
                    ↓
        Uses yolo11s.pt model
                    ↓
        object_detection_sahi.py (publish_results)
                    ↓
        Creates ImageResult.msg
                    ↓
        Publishes to /image_detections
                    ↓
        waypoint_detection_tracker.py (image_result_callback)
                    ↓
        waypoint_detection_tracker.py (_update_max_confidence)
                    ↓
        Stores in self.waypoint_detections[wp_idx]
                    ↓
        waypoint_detection_tracker.py (publish_summary)
                    ↓
        Publishes to /waypoint_detection_summary
```

**Total: 19 files directly involved in Phase 2, plus external dependencies.**

