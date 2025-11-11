# Execution Flow: `ros2 launch video_cam video_cam_complete.launch.py`

## Command
```bash
ros2 launch video_cam video_cam_complete.launch.py
```

This launch file starts **TWO nodes simultaneously**:
1. **Image Publisher Node** (`image_pub`) - Captures camera frames and saves to disk
2. **SAHI Object Detection Node** (`object_detection_sahi`) - Processes images and detects objects

---

## Complete Execution Flow

### Step 1: ROS2 Launch System
**File:** ROS2 launch system (built-in)

**What happens:**
1. ROS2 parses the command: `ros2 launch <package> <launch_file>`
2. Looks for launch file at: `install/video_cam/share/video_cam/launch/video_cam_complete.launch.py`
3. Executes the Python launch file

---

### Step 2: Launch File Execution
**File:** `launch/video_cam_complete.launch.py`

#### 2.1 Imports (lines 1-4)
**What happens:**
- Imports `LaunchDescription`, `Node`, `DeclareLaunchArgument`, `LaunchConfiguration`

#### 2.2 Declares Launch Arguments (lines 23-69)
**What happens:**
- Declares 8 launch arguments for SAHI detection node:
  - `model_path` (default: 'yolo11s.pt')
  - `confidence_threshold` (default: '0.15')
  - `slice_height` (default: '512')
  - `slice_width` (default: '512')
  - `overlap_height_ratio` (default: '0.3')
  - `overlap_width_ratio` (default: '0.3')
  - `check_interval` (default: '2.0')
  - `device` (default: 'auto')

#### 2.3 Creates Image Publisher Node (lines 75-81)
**What happens:**
- Creates Node action for `image_pub` executable
- Package: `video_cam`
- Executable: `image_pub` (from setup.py entry_points)
- Node name: `siyi_a8_publisher`
- No parameters (uses defaults from code)

#### 2.4 Creates SAHI Detection Node (lines 87-103)
**What happens:**
- Creates Node action for `object_detection_sahi` executable
- Package: `video_cam`
- Executable: `object_detection_sahi` (from setup.py entry_points)
- Node name: `sahi_object_detection_node`
- Parameters: Passes all 8 launch arguments as ROS2 parameters

#### 2.5 Returns LaunchDescription (lines 109-122)
**What happens:**
- Returns LaunchDescription containing:
  - All 8 launch arguments
  - Image publisher node
  - SAHI detection node
- ROS2 executes all actions simultaneously (both nodes start in parallel)

---

## NODE 1: Image Publisher Node

### Step 3: Image Publisher Executable Resolution
**File:** `setup.py` (line 26)

**What happens:**
1. ROS2 looks up executable `image_pub` in package manifest
2. Finds entry point: `'image_pub = video_cam.image_pub_siyi:main'`
3. Resolves to: `video_cam/image_pub_siyi.py` → `main()` function
4. Executes the Python script

---

### Step 4: Image Publisher Module Execution
**File:** `video_cam/image_pub_siyi.py`

#### 4.1 Imports (lines 2-19)
**What happens:**
- **ROS2 imports:**
  - `rclpy` - ROS2 Python client library
  - `Node` - Base class for ROS2 nodes
  - `Image` - ROS2 image message type
  - `Bool`, `Float64` - ROS2 message types
  - `CvBridge` - Converts between ROS Image and OpenCV formats
  - `StatusText` - MAVROS status message

- **Computer Vision imports:**
  - `cv2` - OpenCV (camera access and image processing)
  - `numpy` - Numerical arrays

- **System imports:**
  - `os`, `time`, `subprocess` - File system, timing, process management

- **Dependency checks:**
  - Sets `CV_BRIDGE_AVAILABLE` flag
  - Falls back to manual conversion if cv_bridge unavailable

#### 4.2 Node Initialization: `SiyiA8Publisher.__init__()` (lines 22-98)

##### 4.2.1 ROS2 Node Setup (lines 22-23)
**What happens:**
- Calls `super().__init__('siyi_a8_publisher')`
- Creates ROS2 node with name `siyi_a8_publisher`
- Node becomes discoverable on ROS2 network

##### 4.2.2 Publisher Creation (lines 25-27)
**What happens:**
- Creates 2 publishers:
  - `/image_raw` (Image) - Publishes camera frames (topic name: 'image_raw')
  - `/mavros/statustext/send` (StatusText) - Sends status messages to MAVROS

##### 4.2.3 Subscriber Creation (lines 29-32)
**What happens:**
- Creates 3 subscribers:
  - `/camera/trigger` (Bool) - Manual photo capture trigger
  - `/mavros/global_position/rel_alt` (Float64) - Altitude data (enables/disables camera)
  - `/webcam/image_raw` (Image) - Simulation images (fallback mode)

##### 4.2.4 CvBridge Initialization (lines 34-39)
**What happens:**
- Creates `CvBridge()` if available
- Falls back to manual conversion methods if unavailable
- Logs warning if cv_bridge unavailable

##### 4.2.5 Directory Setup (lines 41-77)
**What happens:**
1. Gets absolute path of current file
2. Navigates up directory tree to find `ros2_ws/` directory
3. Constructs path: `ros2_ws/video_cam/`
4. Creates directories:
   - `camera_feed/` - Saves captured images (input for detection node)
   - `mapping_photos/` - Saves triggered photos (manual capture)

##### 4.2.6 Camera Configuration (lines 79-95)
**What happens:**
- Sets `use_real_camera = None` (to be determined)
- Sets `camera_enabled = True` (enabled by default for simulation)
- Sets `ALT_THRESHOLD = 13.716` meters (altitude threshold)
- Sets `capture_photo = False` (manual capture flag)
- Creates timer with 5.0 second interval → `camera_loop()` callback
- Initializes camera variables (`latest_image_msg`, `gstreamer_process`, `capture`)

##### 4.2.7 Camera Initialization (lines 98)
**What happens:**
- Calls `initialize_camera()`
- **`initialize_camera()` function** (lines 100-201):
  1. Attempts to connect to SIYI camera via RTSP:
     - URL: `rtsp://192.168.144.25:8554/main.264`
     - Timeout: 10 seconds
     - Tries multiple methods:
       - FFmpeg backend
       - GStreamer pipeline
       - Default backend
  2. If SIYI camera found:
     - Sets `use_real_camera = True`
     - Logs success message
     - Sends status acknowledgment
  3. If SIYI camera not found:
     - Falls back to webcam (tries /dev/video0, /dev/video1, /dev/video2)
     - Sets `use_real_camera = False`
     - Logs fallback message
  4. If no physical camera found:
     - Sets `capture = None`
     - Waits for simulation images from `/webcam/image_raw` topic
     - Logs simulation mode message

#### 4.3 Main Function Execution
**File:** `video_cam/image_pub_siyi.py` → `main()` (lines 392-397)

##### 4.3.1 ROS2 Initialization (line 393)
**What happens:**
- Calls `rclpy.init(args=args)`
- Initializes ROS2 communication layer

##### 4.3.2 Node Creation (line 394)
**What happens:**
- Creates `SiyiA8Publisher()` instance
- Executes all initialization code (Step 4.2)

##### 4.3.3 Node Spinning (line 395)
**What happens:**
- Calls `rclpy.spin(siyi_a8_publisher)`
- **Enters main loop:**
  - Processes ROS2 callbacks (trigger, altitude, simulation images)
  - Executes timer callbacks (`camera_loop()` every 5 seconds)
  - Handles ROS2 communication
  - Runs until interrupted (Ctrl+C)

##### 4.3.4 Cleanup (lines 396-397)
**What happens:**
- On shutdown:
  - Destroys node
  - Shuts down ROS2

#### 4.4 Runtime Behavior: Timer Callback
**File:** `video_cam/image_pub_siyi.py` → `camera_loop()` (lines 335-390)

**Executed every 5 seconds:**

##### 4.4.1 Camera Enable Check (line 337)
**What happens:**
- Checks if `camera_enabled` is True (based on altitude threshold)
- Skips if disabled (altitude below 13.716m)

##### 4.4.2 Physical Camera Mode (lines 339-363)
**What happens:**
- If `capture` is not None (physical camera available):
  1. Reads frame from camera using `capture.read()`
  2. If frame successfully captured:
     - Converts frame to ROS Image message (BGR8 encoding)
     - Publishes to `/image_raw` topic
     - Saves frame to `camera_feed/photo_TIMESTAMP.jpg`
     - Logs camera type (SIYI or Webcam)
  3. If `capture_photo` flag is set:
     - Saves frame to `mapping_photos/mapping_photo_TIMESTAMP.jpg`
     - Resets `capture_photo` flag
     - Logs photo capture

##### 4.4.3 Simulation Mode (lines 368-388)
**What happens:**
- If `capture` is None (no physical camera):
  1. Checks if `latest_image_msg` is not None (simulation image received)
  2. If simulation image available:
     - Republishes image to `/image_raw` topic
     - Converts ROS Image to OpenCV format
     - Saves image to `camera_feed/photo_TIMESTAMP.jpg`
     - If `capture_photo` flag is set:
       - Saves to `mapping_photos/mapping_photo_TIMESTAMP.jpg`
       - Resets flag
  3. If no simulation image:
     - Logs warning (waiting for `/webcam/image_raw` topic)

#### 4.5 Subscriber Callbacks

##### 4.5.1 Camera Trigger Callback (lines 314-317)
**File:** `camera_trigger_callback()`
**What happens:**
- Receives Bool message on `/camera/trigger` topic
- If message.data is True:
  - Sets `capture_photo = True`
  - Next frame will be saved to `mapping_photos/`

##### 4.5.2 Altitude Callback (lines 319-333)
**File:** `check_altitude()`
**What happens:**
- Receives Float64 message on `/mavros/global_position/rel_alt` topic
- Gets current altitude from message.data
- If altitude >= 13.716m:
  - Enables camera (`camera_enabled = True`)
  - Logs enable message if just enabled
- If altitude < 13.716m:
  - Disables camera (`camera_enabled = False`)
  - Logs disable message if just disabled

##### 4.5.3 Simulation Image Callback (lines 202-203)
**File:** `sim_image_callback()`
**What happens:**
- Receives Image message on `/webcam/image_raw` topic
- Stores message in `latest_image_msg`
- Used by `camera_loop()` for simulation mode

---

## NODE 2: SAHI Object Detection Node

### Step 5: SAHI Detection Executable Resolution
**File:** `setup.py` (line 29)

**What happens:**
1. ROS2 looks up executable `object_detection_sahi` in package manifest
2. Finds entry point: `'object_detection_sahi = video_cam.object_detection_sahi:main'`
3. Resolves to: `video_cam/object_detection_sahi.py` → `main()` function
4. Executes the Python script (runs in parallel with image publisher)

---

### Step 6: SAHI Detection Module Execution
**File:** `video_cam/object_detection_sahi.py`

#### 6.1 Imports (lines 26-77)
**What happens:**
- **ROS2 imports:**
  - `rclpy` - ROS2 Python client library
  - `Node` - Base class for ROS2 nodes
  - `CvBridge` - Converts between ROS Image and OpenCV formats
  - `Image` - ROS2 image message type
  - `ImageResult` - Custom message from interfaces package
  - `WaypointReached` - MAVROS waypoint message
  - `Detection2DArray` - Vision messages for detections

- **ML/AI imports:**
  - `sahi` - Slicing Aided Hyper Inference library
  - `torch` - PyTorch (for GPU support)
  - `ultralytics` - YOLO library
  - `cv2` - OpenCV (computer vision)
  - `numpy` - Numerical arrays

- **Utility imports:**
  - `os`, `time`, `datetime`, `pathlib` - File system and timing

- **Dependency checks:**
  - Sets `SAHI_AVAILABLE`, `TORCH_AVAILABLE`, `YOLO_AVAILABLE` flags
  - Prints errors if critical dependencies missing

#### 6.2 Helper Function: `get_video_cam_directory()` (lines 80-112)
**What happens:**
1. Gets absolute path of current file
2. Navigates up directory tree to find `ros2_ws/` (looks for `install/` and `src/` directories)
3. Constructs path: `ros2_ws/video_cam/`
4. Creates directory if it doesn't exist
5. Returns directory path

#### 6.3 Node Initialization: `SAHIObjectDetectionNode.__init__()` (lines 115-225)

##### 6.3.1 ROS2 Node Setup (lines 115-117)
**What happens:**
- Calls `super().__init__('sahi_object_detection_node')`
- Creates ROS2 node with name `sahi_object_detection_node`
- Node becomes discoverable on ROS2 network

##### 6.3.2 Parameter Declaration (lines 120-143)
**What happens:**
- Declares 8 ROS2 parameters with default values
- Gets parameter values from launch file (or uses defaults)
- Parameters:
  - `model_path`: Path to YOLO model file
  - `confidence_threshold`: Detection confidence (0.15)
  - `slice_height`: SAHI slice height (512)
  - `slice_width`: SAHI slice width (512)
  - `overlap_height_ratio`: Vertical overlap (0.3)
  - `overlap_width_ratio`: Horizontal overlap (0.3)
  - `check_interval`: Timer interval (2.0 seconds)
  - `device`: Compute device ('auto', 'cpu', 'cuda:0', 'mps')

##### 6.3.3 ROS2 Setup (lines 145-152)
**What happens:**
- Creates `CvBridge()` for image conversion
- Creates 3 publishers:
  - `/sahi_detection_results` (Image) - Annotated images
  - `/sahi_detection_info` (String) - Detection metadata
  - `/image_detections` (ImageResult) - Structured detection data

##### 6.3.4 Directory Setup (lines 154-174)
**What happens:**
- Calls `get_video_cam_directory()` to find workspace
- Sets up paths:
  - `camera_feed/` - Input directory (monitored for new images from image publisher)
  - `detection_results_sahi/` - Output directory (saves annotated images)
- Creates directories if they don't exist
- Logs directory paths

##### 6.3.5 Device Detection (lines 176-179)
**What happens:**
- Calls `_get_device()` if device is 'auto'
- **`_get_device()` function** (lines 231-314):
  1. Checks if PyTorch is available
  2. Tries CUDA (NVIDIA GPU) first
  3. Tries MPS (Apple Silicon) second
  4. Falls back to CPU
  5. Logs device information
  6. Returns device string ('cuda:0', 'mps', or 'cpu')

##### 6.3.6 Model Initialization (lines 181-183)
**What happens:**
- Calls `initialize_sahi_model()`
- **`initialize_sahi_model()` function** (lines 316-403):
  1. Checks if SAHI and YOLO are available
  2. Resolves model path (checks multiple locations):
     - `ros2_ws/video_cam/yolo11s.pt`
     - `video_cam/video_cam/yolo11s.pt`
     - Current directory
     - Downloads from Ultralytics if not found
  3. If CUDA device:
     - Clears GPU cache
     - Logs GPU memory info
     - Warns if low memory
  4. Creates `AutoDetectionModel`:
     - Model type: 'yolov8' (works for YOLO v8-11)
     - Model path: Resolved path to yolo11s.pt
     - Confidence threshold: From parameter
     - Device: Detected device (CUDA/CPU/MPS)
  5. Verifies model is on correct device
  6. Logs success/failure

##### 6.3.7 State Initialization (lines 195-205)
**What happens:**
- Creates `processed_images` set (tracks processed images)
- Initializes statistics dictionary:
  - `total_images_processed`: 0
  - `total_detections`: 0
  - `total_tents`: 0
  - `total_people`: 0
  - `avg_processing_time`: 0.0

##### 6.3.8 Timer Setup (lines 207-208)
**What happens:**
- Creates ROS2 timer with interval from `check_interval` parameter (default: 2.0 seconds)
- Timer callback: `check_for_new_images()`
- Timer runs continuously until node shuts down

##### 6.3.9 Subscriber Setup (lines 224-225)
**What happens:**
- Creates subscriber to `/mavros/mission/reached` (WaypointReached)
- Callback: `waypoint_reached_cb()` - Updates waypoint index for detection context

##### 6.3.10 Initialization Complete (lines 210-221)
**What happens:**
- Logs initialization summary:
  - Model path
  - Confidence threshold
  - Slice size
  - Overlap ratio
  - Device
  - MobileNet validation status (disabled)

#### 6.4 Main Function Execution
**File:** `video_cam/object_detection_sahi.py` → `main()` (lines 1301-1345)

##### 6.4.1 ROS2 Initialization (line 1302)
**What happens:**
- Calls `rclpy.init(args=args)`
- Initializes ROS2 communication layer

##### 6.4.2 Dependency Check (lines 1305-1316)
**What happens:**
- Checks if SAHI and YOLO are available
- Prints error and exits if critical dependencies missing
- Continues if dependencies available

##### 6.4.3 Node Creation (line 1335)
**What happens:**
- Creates `SAHIObjectDetectionNode()` instance
- Executes all initialization code (Step 6.3)

##### 6.4.4 Node Spinning (lines 1337-1345)
**What happens:**
- Calls `rclpy.spin(node)`
- **Enters main loop:**
  - Processes ROS2 callbacks (waypoint messages)
  - Executes timer callbacks (`check_for_new_images()` every 2 seconds)
  - Handles ROS2 communication
  - Runs until interrupted (Ctrl+C)

##### 6.4.5 Cleanup (lines 1341-1344)
**What happens:**
- On KeyboardInterrupt (Ctrl+C):
  - Logs shutdown message
  - Logs final statistics
  - Destroys node
  - Shuts down ROS2

#### 6.5 Runtime Behavior: Timer Callback
**File:** `video_cam/object_detection_sahi.py` → `check_for_new_images()` (lines 509-548)

**Executed every 2 seconds (or `check_interval` parameter):**

##### 6.5.1 Directory Check (lines 512-514)
**What happens:**
- Checks if `camera_feed/` directory exists
- Warns and returns if directory doesn't exist

##### 6.5.2 Image Discovery (lines 521-524)
**What happens:**
- Scans `camera_feed/` directory for image files
- Filters for: `.jpg`, `.jpeg`, `.png`, `.bmp` files
- Creates list of image files
- **Note:** These images are saved by the Image Publisher Node (Node 1)

##### 6.5.3 Status Logging (lines 527-531)
**What happens:**
- Logs total images found
- Logs number of already processed images
- Helps debug processing status

##### 6.5.4 Image Processing (lines 534-541)
**What happens:**
- For each image file:
  1. Checks if image is in `processed_images` set
  2. If new image:
     - Calls `process_image(image_path)`
     - Adds image to `processed_images` set
     - Increments `new_images_processed` counter

##### 6.5.5 Statistics Update (lines 543-545)
**What happens:**
- If new images processed:
  - Logs number of processed images
  - Calls `_log_statistics()` to print stats

#### 6.6 Image Processing
**File:** `video_cam/object_detection_sahi.py` → `process_image()` (lines 550-597)

##### 6.6.1 Image Loading (lines 557-560)
**What happens:**
- Loads image using `cv2.imread(image_path)`
- Returns if image cannot be loaded
- Gets image dimensions (width, height)

##### 6.6.2 Object Detection (line 566)
**What happens:**
- Calls `detect_objects_sahi(frame)`
- **`detect_objects_sahi()` function** (lines 599-665):
  1. Converts BGR → RGB (OpenCV uses BGR, PyTorch uses RGB)
  2. Calls SAHI `get_sliced_prediction()`:
     - Slices image into overlapping patches (512x512, 30% overlap)
     - Runs YOLO on each slice
     - Merges results with NMS (Non-Maximum Suppression)
     - Removes duplicate detections
  3. Converts SAHI results to internal format:
     - Extracts bounding boxes (x1, y1, x2, y2)
     - Gets class names and confidence scores
     - Calls `_categorize_detection()` to filter for target classes
  4. Applies additional filtering:
     - Calls `_filter_detections()` to remove duplicates
     - Applies NMS within each class
  5. Returns list of detections

##### 6.6.3 Detection Categorization
**File:** `_categorize_detection()` (lines 667-750)

**What happens:**
- Filters detections for target classes:
  - **Person:** `class_name == 'person'` with confidence > 0.25
  - **Mannequin:** `class_name == 'doll'` with confidence > 0.20
  - **Tent:** `class_name in ['kite', 'umbrella']` with confidence > 0.20
- Validates size and aspect ratio
- Returns detection dict or None

##### 6.6.4 Detection Filtering
**File:** `_filter_detections()` (lines 752-778)

**What happens:**
- Sorts detections by confidence (highest first)
- Applies NMS within each class:
  - Calls `_apply_nms()` for 'person' class
  - Calls `_apply_nms()` for 'tent' class
- Returns filtered detections

##### 6.6.5 Frame Annotation (line 571)
**What happens:**
- Calls `annotate_frame(frame, detections, processing_time)`
- **`annotate_frame()` function** (lines 1048-1162):
  1. Copies original frame
  2. For each detection:
     - Draws bounding box (Green for person, Yellow for tent)
     - Adds text labels with class and confidence
     - Adds background rectangle for text
  3. Adds header with detection method and count
  4. Adds processing time
  5. Adds SAHI mode indicator
  6. Returns annotated frame

##### 6.6.6 Result Publishing (line 574)
**What happens:**
- Calls `publish_results(annotated_frame, detections, image_path)`
- **`publish_results()` function** (lines 1164-1287):
  1. Converts annotated frame to ROS Image message
  2. Sets timestamp and frame_id
  3. Publishes to `/sahi_detection_results` topic
  4. Saves annotated image to `detection_results_sahi/` directory
  5. Creates `ImageResult` message:
     - Includes detection array (Detection2DArray)
     - Includes classes, confidences, areas, descriptions
     - Includes waypoint index
     - Includes timestamp and image name
  6. Publishes to `/image_detections` topic
  7. Creates detection info dictionary
  8. Publishes to `/sahi_detection_info` topic (String)
  9. Logs publication success

##### 6.6.7 Statistics Update (lines 577-586)
**What happens:**
- Updates statistics:
  - `total_images_processed += 1`
  - `total_detections += len(detections)`
  - `total_tents += count of tent detections`
  - `total_people += count of person detections`
  - Updates `avg_processing_time` (running average)

##### 6.6.8 Logging (lines 588-592)
**What happens:**
- Logs detection results:
  - Number of objects detected
  - Processing time
  - Count of people and tents

---

## Complete System Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    EXTERNAL INPUTS                           │
├─────────────────────────────────────────────────────────────┤
│  • SIYI Camera (RTSP: rtsp://192.168.144.25:8554/main.264)  │
│  • Webcam (/dev/video0, /dev/video1, /dev/video2)           │
│  • Simulation (/webcam/image_raw topic)                     │
│  • Altitude data (/mavros/global_position/rel_alt)          │
│  • Camera trigger (/camera/trigger)                         │
│  • Waypoint info (/mavros/mission/reached)                  │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│         NODE 1: Image Publisher (siyi_a8_publisher)         │
├─────────────────────────────────────────────────────────────┤
│  1. Initializes camera (SIYI → Webcam → Simulation)        │
│  2. Timer: Every 5 seconds                                  │
│     ├─ Checks altitude threshold (13.716m)                  │
│     ├─ Captures frame from camera                           │
│     ├─ Converts to ROS Image message                        │
│     ├─ Publishes to /image_raw topic                        │
│     └─ Saves to camera_feed/photo_TIMESTAMP.jpg            │
│  3. Subscribers:                                            │
│     ├─ /camera/trigger → Manual photo capture              │
│     ├─ /mavros/global_position/rel_alt → Altitude check    │
│     └─ /webcam/image_raw → Simulation images               │
└─────────────────────────────────────────────────────────────┘
                          ↓
                    File System
                    camera_feed/
                    photo_*.jpg
                          ↓
┌─────────────────────────────────────────────────────────────┐
│    NODE 2: SAHI Detection (sahi_object_detection_node)      │
├─────────────────────────────────────────────────────────────┤
│  1. Initializes YOLO model via SAHI                         │
│  2. Timer: Every 2 seconds                                  │
│     ├─ Scans camera_feed/ directory                         │
│     ├─ Detects new images                                   │
│     ├─ Processes with SAHI + YOLO:                         │
│     │  • Slice image (512x512, 30% overlap)                │
│     │  • Run YOLO on each slice                             │
│     │  • Merge results with NMS                            │
│     │  • Filter for person/tent classes                    │
│     ├─ Annotates frame with bounding boxes                  │
│     ├─ Publishes to ROS2 topics                            │
│     └─ Saves to detection_results_sahi/                    │
│  3. Subscribers:                                            │
│     └─ /mavros/mission/reached → Waypoint tracking         │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│                    OUTPUT TOPICS                             │
├─────────────────────────────────────────────────────────────┤
│  • /image_raw (Image) - Raw camera frames                   │
│  • /sahi_detection_results (Image) - Annotated images       │
│  • /sahi_detection_info (String) - Detection metadata       │
│  • /image_detections (ImageResult) - Structured detections  │
│  • /mavros/statustext/send (StatusText) - Status messages   │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│              File System Outputs                             │
├─────────────────────────────────────────────────────────────┤
│  • camera_feed/photo_*.jpg - Raw captured images            │
│  • mapping_photos/mapping_photo_*.jpg - Triggered photos    │
│  • detection_results_sahi/sahi_detected_*.jpg - Annotated   │
└─────────────────────────────────────────────────────────────┘
```

---

## File Summary

### Files Called (in order):

1. **`launch/video_cam_complete.launch.py`**
   - Launch file that configures and starts BOTH nodes
   - Declares launch arguments for SAHI detection
   - Creates two node actions (image publisher + detection)

2. **`setup.py`**
   - Resolves executable names to Python modules
   - Maps `image_pub` → `video_cam.image_pub_siyi:main`
   - Maps `object_detection_sahi` → `video_cam.object_detection_sahi:main`

3. **`video_cam/image_pub_siyi.py`** (Node 1)
   - Image publisher module
   - Contains `SiyiA8Publisher` class
   - Contains `main()` function

4. **`video_cam/object_detection_sahi.py`** (Node 2)
   - SAHI detection module
   - Contains `SAHIObjectDetectionNode` class
   - Contains `main()` function
   - Contains helper functions

### Key Functions (Node 1: Image Publisher):

1. **`main()`** - Entry point, initializes ROS2 and creates node
2. **`SiyiA8Publisher.__init__()`** - Node initialization
3. **`initialize_camera()`** - Connects to camera (SIYI/Webcam/Simulation)
4. **`camera_loop()`** - Timer callback (runs every 5 seconds)
5. **`camera_trigger_callback()`** - Manual photo capture
6. **`check_altitude()`** - Altitude-based camera enable/disable
7. **`sim_image_callback()`** - Simulation image handler

### Key Functions (Node 2: SAHI Detection):

1. **`main()`** - Entry point, initializes ROS2 and creates node
2. **`SAHIObjectDetectionNode.__init__()`** - Node initialization
3. **`get_video_cam_directory()`** - Finds workspace directory
4. **`_get_device()`** - Auto-detects compute device (GPU/CPU)
5. **`initialize_sahi_model()`** - Loads YOLO model via SAHI
6. **`check_for_new_images()`** - Timer callback (runs every 2 seconds)
7. **`process_image()`** - Processes single image
8. **`detect_objects_sahi()`** - Runs SAHI + YOLO detection
9. **`_categorize_detection()`** - Filters detections for target classes
10. **`_filter_detections()`** - Applies NMS to remove duplicates
11. **`annotate_frame()`** - Draws bounding boxes and labels
12. **`publish_results()`** - Publishes results to ROS2 topics and saves to disk

---

## Runtime Behavior

### Node 1: Image Publisher
1. **Startup:** Node initializes, connects to camera, sets up directories
2. **Monitoring:** Timer captures frames every 5 seconds
3. **Altitude Check:** Monitors altitude, enables/disables camera based on threshold
4. **Image Capture:** Captures frame from camera (SIYI/Webcam/Simulation)
5. **Publishing:** Publishes to `/image_raw` topic
6. **Saving:** Saves frame to `camera_feed/photo_TIMESTAMP.jpg`
7. **Loop:** Continues until node is shut down

### Node 2: SAHI Detection
1. **Startup:** Node initializes, loads model, sets up directories
2. **Monitoring:** Timer checks `camera_feed/` directory every 2 seconds
3. **Processing:** New images are detected and processed
4. **Detection:** SAHI + YOLO detects objects in images
5. **Publishing:** Results are published to ROS2 topics and saved to disk
6. **Loop:** Continues until node is shut down

### Interaction Between Nodes
- **Node 1** saves images to `camera_feed/` directory
- **Node 2** monitors `camera_feed/` directory for new images
- **Node 2** processes images saved by Node 1
- Both nodes run simultaneously and independently
- Both nodes can be stopped independently (Ctrl+C stops both when launched together)

---

## Key Directories

- **Input (Node 1):** Camera stream (RTSP/Webcam) or `/webcam/image_raw` topic
- **Output (Node 1):** `ros2_ws/video_cam/camera_feed/` - Saves captured images
- **Input (Node 2):** `ros2_ws/video_cam/camera_feed/` - Monitored for new images
- **Output (Node 2):** `ros2_ws/video_cam/detection_results_sahi/` - Saves annotated images
- **Model:** `ros2_ws/video_cam/yolo11s.pt` (or downloads from Ultralytics)

---

## Key Topics

### Subscribed (Node 1):
- `/camera/trigger` (Bool) - Manual photo capture
- `/mavros/global_position/rel_alt` (Float64) - Altitude data
- `/webcam/image_raw` (Image) - Simulation images

### Published (Node 1):
- `/image_raw` (Image) - Raw camera frames
- `/mavros/statustext/send` (StatusText) - Status messages

### Subscribed (Node 2):
- `/mavros/mission/reached` (WaypointReached) - Waypoint information

### Published (Node 2):
- `/sahi_detection_results` (Image) - Annotated images with bounding boxes
- `/image_detections` (ImageResult) - Structured detection data
- `/sahi_detection_info` (String) - Detection metadata (JSON-like)

---

## Dependencies

### Required:
- `rclpy` - ROS2 Python client
- `sahi` - Slicing Aided Hyper Inference
- `ultralytics` - YOLO library
- `torch` - PyTorch (for GPU support)
- `cv2` - OpenCV
- `numpy` - Numerical arrays
- `cv_bridge` - ROS/OpenCV bridge
- `sensor_msgs` - ROS2 sensor messages
- `interfaces` - Custom message package
- `mavros_msgs` - MAVROS messages
- `vision_msgs` - Vision messages

### Optional:
- CUDA GPU (for acceleration)
- MPS (Apple Silicon GPU)
- SIYI camera (RTSP stream)
- Webcam (/dev/video0, etc.)

---

## Example Output

### Node 1 (Image Publisher):
```
[INFO] [timestamp] [siyi_a8_publisher]: Attempting to initialize SIYI camera (10 second timeout)...
[INFO] [timestamp] [siyi_a8_publisher]: Successfully connected to SIYI camera!
[INFO] [timestamp] [siyi_a8_publisher]: SIYI Frame Publishing
[INFO] [timestamp] [siyi_a8_publisher]: SIYI Frame Publishing
```

### Node 2 (SAHI Detection):
```
[INFO] [timestamp] [sahi_object_detection_node]: SAHI Object Detection Node Initialized
[INFO] [timestamp] [sahi_object_detection_node]: Model: yolo11s.pt
[INFO] [timestamp] [sahi_object_detection_node]: Confidence Threshold: 0.15
[INFO] [timestamp] [sahi_object_detection_node]: Slice Size: 512x512
[INFO] [timestamp] [sahi_object_detection_node]: Device: cuda:0
[INFO] [timestamp] [sahi_object_detection_node]: Found 5 total images, 0 already processed
[INFO] [timestamp] [sahi_object_detection_node]: Processing new image: photo_20250101-120000.jpg
[INFO] [timestamp] [sahi_object_detection_node]: Found 3 objects in 0.85s: 2 people, 1 tents
[INFO] [timestamp] [sahi_object_detection_node]: Published and saved results -> sahi_detected_photo_20250101-120000.jpg
```

---

## Timing Behavior

### Node 1 (Image Publisher):
- **Timer interval:** 5.0 seconds
- **Action:** Captures frame, publishes to topic, saves to disk
- **Frequency:** ~0.2 Hz (12 images per minute)

### Node 2 (SAHI Detection):
- **Timer interval:** 2.0 seconds (configurable via `check_interval` parameter)
- **Action:** Scans directory, processes new images
- **Frequency:** ~0.5 Hz (30 checks per minute)

### Processing Delay:
- Image captured by Node 1 → Saved to `camera_feed/`
- Node 2 checks directory every 2 seconds
- Maximum delay: 2 seconds (if image saved just after check)
- Average delay: ~1 second

---

## Troubleshooting

### No images being captured:
- Check if camera is connected (SIYI/Webcam)
- Check if altitude threshold is met (>= 13.716m)
- Check if `/webcam/image_raw` topic is publishing (simulation mode)
- Check `camera_feed/` directory permissions

### No detections:
- Check if images are in `camera_feed/` directory
- Lower `confidence_threshold` parameter
- Check if model is loaded correctly
- Check Node 2 logs for errors

### Model not found:
- Check if `yolo11s.pt` exists in `ros2_ws/video_cam/`
- Model will auto-download from Ultralytics if not found

### GPU not working:
- Check if PyTorch has CUDA support: `python3 -c "import torch; print(torch.cuda.is_available())"`
- Install PyTorch with CUDA support if needed
- Use `device:=cpu` parameter to force CPU

### Dependencies missing:
- Install: `pip install sahi ultralytics torch opencv-python`

---

## Summary

When you run `ros2 launch video_cam video_cam_complete.launch.py`:

1. **Launch file** configures and starts TWO nodes simultaneously:
   - **Node 1:** Image Publisher - Captures camera frames and saves to `camera_feed/`
   - **Node 2:** SAHI Detection - Processes images from `camera_feed/` and detects objects

2. **Node 1 initializes** by connecting to camera (SIYI/Webcam/Simulation), setting up directories, and creating ROS2 publishers/subscribers

3. **Node 2 initializes** by loading the YOLO model, setting up directories, and creating ROS2 publishers/subscribers

4. **Node 1 timer runs** every 5 seconds to capture frames, publish to `/image_raw` topic, and save to `camera_feed/` directory

5. **Node 2 timer runs** every 2 seconds to check `camera_feed/` directory for new images and process them

6. **Node 2 processes images** using SAHI + YOLO to detect objects (person, tent)

7. **Node 2 publishes results** to ROS2 topics and saves annotated images to `detection_results_sahi/` directory

8. **Process repeats** until nodes are shut down (Ctrl+C stops both)

The system is designed as a complete pipeline: **Camera → Image Capture → Object Detection → Results**. Both nodes work together to provide end-to-end object detection from camera feed to detected results.
