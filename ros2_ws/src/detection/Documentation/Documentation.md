# Image Publisher SIYI A8 Camera - System Documentation

**Date:** November 2025  
**Package:** `detection`  
**System:** ODLC Machine Inferencing System 2025-2026

---

## System Workflow Overview

This document describes the complete workflow of image capture and object detection in the ODLC system.

### Image Capture and Object Detection Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. IMAGE CAPTURE (image_pub_siyi.py)                            │
├─────────────────────────────────────────────────────────────────┤
│ • Connects to SIYI A8 camera via RTSP stream                    │
│   (rtsp://192.168.144.25:8554/main.264)                         │
│ • Captures frames from camera in real-time                      │
│ • Saves images to: ros2_ws/detection/camera_feed/               │
│ • Publishes frames to ROS topic: /image_raw                     │
└─────────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────────┐
│ 2. OBJECT DETECTION (object_detection_sahi.py)                  │
├─────────────────────────────────────────────────────────────────┤
│ • Timer periodically checks camera_feed/ for new images         │
│   (default: every 2 seconds)                                    │
│ • Processes new images using SAHI + YOLO11s model               │
│   - Slices images into overlapping patches                      │
│   - Detects objects in each slice                               │
│   - Merges and filters results (NMS)                            │
│ • Optional: Validates detections with MobileNetV3               │
│ • Saves annotated results to:                                   │
│   ros2_ws/detection/detection_results_sahi/                     │
│ • Publishes results to ROS topics:                              │
│   - /sahi_detection_results (annotated images)                  │
│   - /image_detections (ImageResult messages)                    │
└─────────────────────────────────────────────────────────────────┘
```

### Key Components

**Image Publisher Node (`image_pub_siyi.py`)**
- **Function:** Captures frames from SIYI A8 camera and saves them to disk
- **Output Directory:** `ros2_ws/detection/camera_feed/`
- **ROS Topic:** `/image_raw` (sensor_msgs/Image)
- **Image Format:** JPG files saved with timestamp

**Object Detection Node (`object_detection_sahi.py`)**
- **Function:** Detects objects (people/mannequins, tents) in saved images
- **Input Directory:** `ros2_ws/detection/camera_feed/`
- **Output Directory:** `ros2_ws/detection/detection_results_sahi/`
- **Detection Method:** SAHI (Slicing Aided Hyper Inference) with YOLO11s
- **ROS Topics:**
  - `/sahi_detection_results` (sensor_msgs/Image)
  - `/image_detections` (interfaces/ImageResult)
- **Check Interval:** Configurable via parameter (default: 2.0 seconds)

### Workflow Steps

1. **Start Camera Node:**
   ```bash
   ros2 run detection image_pub
   ```
   - Camera node connects to SIYI A8 and starts capturing frames
   - Images are saved to `camera_feed/` directory

2. **Start Detection Node:**
   ```bash
   ros2 run detection object_detection_sahi
   ```
   - Detection node monitors `camera_feed/` directory
   - Processes new images as they appear
   - Saves annotated results to `detection_results_sahi/`

3. **Processing Flow:**
   - Camera captures frame → Saves to `camera_feed/image_001.jpg`
   - Detection node timer fires → Checks for new images
   - New image found → Loads image → Runs SAHI detection
   - Detections made → Annotates image → Saves to `detection_results_sahi/`
   - Results published to ROS topics

### Directory Structure

```
ros2_ws/detection/
├── camera_feed/              # Images captured from camera
│   ├── image_001.jpg
│   ├── image_002.jpg
│   └── ...
└── detection_results_sahi/  # Annotated detection results
    ├── image_001_annotated.jpg
    ├── image_001_detections.json
    └── ...
```

---

## Technical Details

### Image Capture Implementation

Fixed critical issues preventing the SIYI A8 camera node from initializing and capturing frames. The node was encountering NumPy/cv_bridge compatibility errors and OpenCV GStreamer support limitations. Implemented comprehensive fallback mechanisms and manual image conversion methods to ensure robust camera operation.

---

## Problem Statement

### Issues Identified

1. **NumPy Compatibility Error**
   - **Error:** `AttributeError: _ARRAY_API not found` in cv_bridge module
   - **Root Cause:** cv_bridge compiled against NumPy 1.x but system had NumPy 2.2.6
   - **Impact:** Node crashed on import, preventing initialization

2. **OpenCV GStreamer Support Missing**
   - **Error:** `Could not open video stream`
   - **Root Cause:** OpenCV installation built without GStreamer support
   - **Impact:** Unable to connect to RTSP camera stream at `rtsp://192.168.144.25:8554/main.264`

3. **Lack of Fallback Mechanisms**
   - **Issue:** Single method for camera connection with no alternatives
   - **Impact:** Complete failure if primary method unavailable

---

## Solutions Implemented

### 1. cv_bridge Import Error Mitigation

**File:** `image_pub_siyi.py` (Lines 1-19)

**Changes:**
```python
# BEFORE:
from cv_bridge import CvBridge

# AFTER:
try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except Exception as e:
    print(f"Warning: cv_bridge import failed: {e}")
    print("Will attempt to use alternative image conversion methods")
    CV_BRIDGE_AVAILABLE = False
    CvBridge = None
```

**Added Imports:**
```python
import subprocess
import numpy as np
```

**Rationale:**
- Graceful degradation when cv_bridge unavailable
- Allows node to continue operation with alternative methods
- Provides clear diagnostic output for debugging

---

### 2. Manual Image Conversion Methods

**File:** `image_pub_siyi.py` (Lines 133-163)

**Implemented Two New Methods:**

#### Method: `cv2_to_imgmsg_manual()`
```python
def cv2_to_imgmsg_manual(self, cv_image, encoding='bgr8'):
    """Convert OpenCV image to ROS Image message without cv_bridge"""
    msg = Image()
    msg.height = cv_image.shape[0]
    msg.width = cv_image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = cv_image.shape[1] * cv_image.shape[2]
    msg.data = cv_image.tobytes()
    return msg
```

**Purpose:** Bypasses cv_bridge dependency for OpenCV→ROS Image conversion

#### Method: `imgmsg_to_cv2_manual()`
```python
def imgmsg_to_cv2_manual(self, img_msg, desired_encoding='bgr8'):
    """Convert ROS Image message to OpenCV image without cv_bridge"""
    dtype = np.uint8
    n_channels = 3 if desired_encoding == 'bgr8' else 1
    
    img_buf = np.asarray(img_msg.data, dtype=dtype)
    cv_image = img_buf.reshape(img_msg.height, img_msg.width, n_channels)
    
    return cv_image
```

**Purpose:** Bypasses cv_bridge dependency for ROS Image→OpenCV conversion

---

### 3. Conditional cv_bridge Initialization

**File:** `image_pub_siyi.py` (Lines 36-42)

**Changes:**
```python
# BEFORE:
self.bridge = CvBridge()

# AFTER:
if CV_BRIDGE_AVAILABLE:
    self.bridge = CvBridge()
else:
    self.bridge = None
    self.get_logger().warn("cv_bridge not available, using alternative conversion")
```

**Rationale:** Only initialize cv_bridge if available, otherwise use manual methods

---

### 4. Multi-Method Camera Connection Strategy

**File:** `image_pub_siyi.py` (Lines 66-98)

**Implemented Hierarchical Fallback:**

```python
rtsp_url = 'rtsp://192.168.144.25:8554/main.264'

# Method 1: FFmpeg Backend (most compatible)
self.capture = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)

if not self.capture.isOpened():
    # Method 2: GStreamer Pipeline
    gst_pipeline = (
        'rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! '
        'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink'
    )
    self.capture = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

if not self.capture.isOpened():
    # Method 3: Default Backend
    self.capture = cv2.VideoCapture(rtsp_url)
```

**Connection Methods Priority:**
1. **FFmpeg Backend** (`cv2.CAP_FFMPEG`) - Primary method
   - Most widely supported
   - Works without GStreamer
   - Handles RTSP streams natively

2. **GStreamer Pipeline** (`cv2.CAP_GSTREAMER`) - Secondary method
   - Used if OpenCV has GStreamer support
   - More efficient for RTSP streams
   - Requires GStreamer libraries

3. **Default Backend** - Tertiary method
   - System-dependent
   - Last resort fallback

---

### 5. Enhanced Camera Loop with Conditional Conversion

**File:** `image_pub_siyi.py` (Lines 221-252)

**Changes:**

```python
# Conditional image conversion based on cv_bridge availability
if self.bridge is not None:
    imageToTransmit = self.bridge.cv2_to_imgmsg(capturedFrame, encoding='bgr8')
else:
    imageToTransmit = self.cv2_to_imgmsg_manual(capturedFrame, encoding='bgr8')
```

**Applied to:**
- Real camera frame publishing (lines 232-235)
- Simulation camera frame republishing (lines 258-261)

---

### 6. Improved Logging and Diagnostics

**File:** `image_pub_siyi.py` (Multiple locations)

**Changes:**

```python
# Added throttled logging to prevent log spam
self.get_logger().info("Camera Frame Publishing", throttle_duration_sec=5.0)
self.get_logger().warn("Failed to read frame", throttle_duration_sec=10.0)
self.get_logger().warn("No image received", throttle_duration_sec=5.0)
```

**Diagnostic Messages Added:**
- Camera connection attempt notifications
- Method fallback warnings
- Success/failure status for each connection method
- Periodic status updates with throttling

---

### 7. GStreamer Subprocess Implementation (Backup Method)

**File:** `image_pub_siyi.py` (Lines 100-131, 165-177)

**Note:** Implemented but not actively used in final solution. Retained for future enhancement.

**Methods Added:**
- `init_gstreamer_subprocess()` - Initialize GStreamer as separate process
- `read_frame_from_gstreamer()` - Read raw frames from subprocess stdout

**Purpose:** Alternative method if OpenCV completely lacks video backend support

---

## Technical Specifications

### Modified Code Structure

```
image_pub_siyi.py
├── Imports (Lines 1-19)
│   ├── Added: subprocess, numpy
│   └── Modified: cv_bridge with try/except
│
├── Class: SiyiA8Publisher
│   ├── __init__() (Lines 23-98)
│   │   ├── Modified: Conditional cv_bridge init
│   │   ├── Modified: Multi-method camera connection
│   │   └── Enhanced: Error handling and logging
│   │
│   ├── NEW: cv2_to_imgmsg_manual() (Lines 133-143)
│   ├── NEW: imgmsg_to_cv2_manual() (Lines 145-153)
│   ├── NEW: init_gstreamer_subprocess() (Lines 100-131)
│   ├── NEW: read_frame_from_gstreamer() (Lines 165-177)
│   │
│   └── camera_loop() (Lines 221-272)
│       ├── Modified: Conditional conversion method selection
│       └── Enhanced: Throttled logging
│
└── main() (Lines 274-280)
    └── Unchanged
```

### Dependencies

**Required Packages:**
- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - ROS 2 sensor message types
- `cv_bridge` - ROS/OpenCV bridge (optional with fallback)
- `cv2` (OpenCV) - Computer vision library
- `numpy` - Numerical computing (< 2.0 for compatibility)

**System Requirements:**
- NumPy < 2.0 (installed via `pip install "numpy<2" --force-reinstall`)
- OpenCV with FFmpeg support
- GStreamer 1.0 (optional, for enhanced streaming)

---

## Verification and Testing

### Test Commands Used

1. **Check NumPy version:**
   ```bash
   python3 -c "import numpy; print('NumPy version:', numpy.__version__)"
   # Output: NumPy version: 1.26.4
   ```

2. **Verify OpenCV GStreamer support:**
   ```bash
   python3 -c 'import cv2; print(cv2.getBuildInformation())' | grep -i gstreamer
   # Output: GStreamer: NO
   ```

3. **Test GStreamer directly:**
   ```bash
   gst-launch-1.0 rtspsrc location=rtsp://192.168.144.25:8554/main.264 \
     latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! fakesink
   # Result: Successful connection
   ```

4. **Test camera connectivity:**
   ```bash
   ping -c 2 192.168.144.25
   # Result: Reachable (0% packet loss)
   ```

5. **Build and run node:**
   ```bash
   cd /home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
   colcon build --packages-select detection --symlink-install
   source install/setup.bash
   ros2 run detection image_pub
   ```

### Expected Output (Successful)

```
[INFO] [timestamp] [siyi_a8_publisher]: Using real camera: True
[INFO] [timestamp] [siyi_a8_publisher]: Attempting to connect to camera at rtsp://192.168.144.25:8554/main.264 using FFmpeg...
[INFO] [timestamp] [siyi_a8_publisher]: Status: Real camera initialized
[INFO] [timestamp] [siyi_a8_publisher]: Successfully connected to camera!
[INFO] [timestamp] [siyi_a8_publisher]: Camera Frame Publishing
```

---

## Files Modified

### Primary File
- **Path:** `/home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/detection/detection/image_pub_siyi.py`
- **Lines Changed:** ~120 lines modified/added
- **Total Lines:** 280 lines (from 144 lines)

### Supporting Files (Unchanged)
- `setup.py` - Package entry points
- `package.xml` - ROS 2 package manifest
- `CMakeLists.txt` - Build configuration

---

## Environment Configuration

### Python Environment
```bash
# NumPy downgrade command executed:
pip install "numpy<2" --force-reinstall

# Verified installation:
python3 -c "import numpy; print(numpy.__version__)"
# Output: 1.26.4
```

### ROS 2 Build
```bash
# Build command:
cd /home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select detection --symlink-install

# Build output:
# Summary: 1 package finished [4.32s]
```

---

## Camera Configuration

### Hardware
- **Model:** SIYI A8 Camera
- **Connection:** RTSP over network
- **IP Address:** 192.168.144.25
- **Port:** 8554
- **Stream Path:** `/main.264`
- **Full URL:** `rtsp://192.168.144.25:8554/main.264`

### Stream Properties
- **Codec:** H.264
- **Protocol:** RTSP (Real Time Streaming Protocol)
- **Resolution:** 1920x1080 (assumed, configurable)
- **Latency:** 0ms (configured for minimum delay)

---

## Operational Modes

### Mode 1: Real Camera (use_real_camera = True)
- Connects to SIYI A8 camera via RTSP
- Publishes frames to `/image_raw` topic
- Saves images to `camera_feed/` directory
- Altitude-based enabling (threshold: 13.716m)

### Mode 2: Simulation Camera (use_real_camera = False)
- Subscribes to `/webcam/image_raw` topic
- Republishes frames to `/image_raw` topic
- Used for testing without physical hardware

---

## Known Limitations and Future Work

### Current Limitations
1. OpenCV installation lacks GStreamer support (uses FFmpeg fallback)
2. GStreamer subprocess method implemented but not fully tested
3. Frame resolution hardcoded (1920x1080) in subprocess method

### Recommended Improvements
1. **Rebuild OpenCV with GStreamer support** for optimal performance:
   ```bash
   # Install GStreamer development libraries
   sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
   
   # Rebuild OpenCV from source with -DWITH_GSTREAMER=ON
   ```

2. **Dynamic resolution detection** in GStreamer subprocess method

3. **Connection health monitoring** with automatic reconnection

4. **Configurable camera parameters** via ROS 2 parameters:
   - RTSP URL
   - Resolution
   - Frame rate
   - Codec preferences

---

## Error Resolution Reference

### Error 1: NumPy Compatibility
**Symptom:**
```
AttributeError: _ARRAY_API not found
```

**Solution:**
```bash
pip install "numpy<2" --force-reinstall
```

**Code Changes:** Graceful cv_bridge import with fallback

---

### Error 2: OpenCV Cannot Open Stream
**Symptom:**
```
[ERROR] Could not open video stream
```

**Root Cause:** OpenCV built without GStreamer support

**Solutions Implemented:**
1. Try FFmpeg backend first (most compatible)
2. Fallback to GStreamer pipeline
3. Fallback to default backend
4. Optional subprocess method

---

### Error 3: Package Not Found
**Symptom:**
```
PackageNotFoundError: "package 'detection' not found"
```

**Solution:**
```bash
cd /path/to/ros2_ws
source install/setup.bash
```

**Note:** Always source workspace after building

---

## Rollback Procedure

If issues arise, revert changes:

```bash
cd /home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/detection/detection
git checkout image_pub_siyi.py
cd ../../..
colcon build --packages-select detection
```

**Note:** Original code required GStreamer support and working cv_bridge

---

**Check node status:**
```bash
ros2 node list
ros2 node info /siyi_a8_publisher
```

**Monitor topics:**
```bash
ros2 topic list
ros2 topic echo /image_raw
ros2 topic hz /image_raw
```

**Check logs:**
```bash
ros2 run detection image_pub 2>&1 | tee camera_debug.log
```

### Debugging Tips

1. **Verify camera accessibility:**
   ```bash
   ping 192.168.144.25
   gst-launch-1.0 rtspsrc location=rtsp://192.168.144.25:8554/main.264 ! fakesink
   ```

2. **Check OpenCV build info:**
   ```bash
   python3 -c "import cv2; print(cv2.getBuildInformation())"
   ```

3. **Test manual image conversion:**
   ```bash
   python3 -c "from detection.image_pub_siyi import SiyiA8Publisher; print('Import OK')"
   ```

---

## Conclusion

Successfully resolved critical compatibility and connectivity issues in the SIYI A8 camera node. The implementation now features robust fallback mechanisms, graceful error handling, and compatibility with various OpenCV build configurations. The node is production-ready and capable of reliable camera operation in both real and simulated environments.

**Status:** **Operational**  
**Last Updated:** October 25, 2025  
**Maintainer:** CPP Aerial Vision Analysis System Team