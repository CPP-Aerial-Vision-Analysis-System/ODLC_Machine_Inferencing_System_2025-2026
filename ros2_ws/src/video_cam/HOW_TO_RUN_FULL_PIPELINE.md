# Complete Guide: SIYI Camera Full Cycle Image Processing Pipeline

**Date:** February 17, 2026  
**System:** ROS 2 Humble | SIYI A8 Mini Camera | Object Detection with YOLO

---

## 📋 Table of Contents
1. [System Overview](#system-overview)
2. [Architecture Components](#architecture-components)
3. [How It Works](#how-it-works)
4. [Prerequisites](#prerequisites)
5. [Running the Full Pipeline](#running-the-full-pipeline)
6. [Understanding the Workflow](#understanding-the-workflow)
7. [ROS Topics & Commands](#ros-topics--commands)
8. [Troubleshooting](#troubleshooting)

---

## 🎯 System Overview

This is a **complete image processing pipeline** that:
1. **Captures** high-resolution images from a SIYI A8 Mini camera
2. **Downloads** images from the camera's SD card over Ethernet
3. **Stores** images locally with verification
4. **Publishes** images to ROS 2 topics for processing
5. **Detects** objects using YOLO with SAHI (optional)

**Key Feature:** All components work together seamlessly through ROS 2 topics!

---

## 🏗️ Architecture Components

### Core Files

| File | Purpose |
|------|---------|
| **`config.py`** | Configuration constants (IPs, ports, timeouts, paths) |
| **`camera_interface.py`** | Low-level camera communication (UDP/HTTP/SDK) |
| **`storage_manager.py`** | Local file storage with atomic writes & verification |
| **`pipeline_orchestrator.py`** | Coordinates the 4-phase capture workflow |
| **`siyi_node_refactored.py`** | ROS 2 node that wraps everything together |

### How They Connect

```
siyi_node_refactored.py (ROS 2 Node)
    ├── Uses: pipeline_orchestrator.py
    │   ├── Uses: camera_interface.py (talks to camera)
    │   └── Uses: storage_manager.py (saves files)
    └── All use: config.py (shared constants)
```

---

## ⚙️ How It Works

### The 4-Phase Pipeline

When you trigger a capture, this happens:

```
PHASE 1: CAPTURE CONTROL
├── Send UDP command to camera (192.168.144.25:37260)
├── Camera captures 4K image to SD card
└── Wait for acknowledgment (2-3 seconds)

PHASE 2: SD CARD INDEXING
├── Poll camera's HTTP API (192.168.144.25:82)
├── Get list of files on SD card
├── Find the NEW image (not previously downloaded)
└── Retry every 0.5s until found (max 15s timeout)

PHASE 3: DOWNLOAD & SAVE
├── Download image bytes over HTTP
├── Verify disk space (need ~10MB free)
├── Decode JPEG image
├── Verify dimensions (4K = 3840x2160 min)
├── Save to local storage with atomic write
└── Mark file as downloaded (prevent duplicates)

PHASE 4: PUBLISH (Automatic)
├── Publish to /image_raw topic
├── Available for detection nodes
└── Continue monitoring for next trigger
```

**Total Time:** 3-5 seconds per image

---

## ✅ Prerequisites

### 1. Hardware Setup
```bash
# Connect camera to Jetson via Ethernet cable
# Camera should be powered on with SD card inserted
```

### 2. Network Configuration
```bash
# Set Jetson IP to 192.168.144.100
sudo nmtui
# Select "Edit a connection" → Ethernet → Manual
# IP: 192.168.144.100, Netmask: 255.255.255.0

# Test camera connection
ping 192.168.144.25
# Should see: time<5ms

# Test camera API
curl http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0
# Should return: {"success":true,...}
```

### 3. Build the Workspace
```bash
cd ~/astra/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```

---

## 🚀 Running the Full Pipeline

### Method 1: Using ROS 2 Commands (Recommended)

#### Terminal 1: Launch the Camera Node
```bash
cd ~/astra/ros2_ws
source install/setup.bash

# Option A: Run the refactored node (latest version)
ros2 run video_cam siyi_unified_pipeline_refactored

# Option B: Use launch file
ros2 launch video_cam siyi_unified_pipeline.launch.py
```

**Expected Output:**
```
[INFO] [siyi_unified_pipeline]: Starting SIYI camera node...
[INFO] [siyi_unified_pipeline]: Camera interface initialized
[INFO] [siyi_unified_pipeline]: Storage manager initialized at: /home/astra-dev/astra/ros2_ws/video_cam_data
[INFO] [siyi_unified_pipeline]: Pipeline orchestrator initialized
[INFO] [siyi_unified_pipeline]: ✓ Camera stream connected
[INFO] [siyi_unified_pipeline]: ✓ SD card ready
[INFO] [siyi_unified_pipeline]: Pipeline ready. Waiting for triggers...
```

#### Terminal 2: Trigger Image Capture
```bash
source install/setup.bash

# Trigger a single capture
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
```

**What Happens:**
```
[INFO] [siyi_unified_pipeline]: ======================================================================
[INFO] [siyi_unified_pipeline]: STARTING CAPTURE PIPELINE
[INFO] [siyi_unified_pipeline]: ======================================================================
[INFO] [siyi_unified_pipeline]: [Phase 1] Capture command sent (photo #1)
[INFO] [siyi_unified_pipeline]: [Phase 2] Polling SD card...
[INFO] [siyi_unified_pipeline]: ✓ Found new image: DSCF0001.JPG
[INFO] [siyi_unified_pipeline]: [Phase 3] Downloading: DSCF0001.JPG
[INFO] [siyi_unified_pipeline]: Saved: /home/astra-dev/astra/ros2_ws/video_cam_data/mapping_photos/DSCF0001.JPG
[INFO] [siyi_unified_pipeline]: ✓ PIPELINE COMPLETED in 4.2s
```

#### Terminal 3 (Optional): Run Object Detection
```bash
source install/setup.bash

# Run YOLO detection with SAHI
ros2 run detection object_detection_sahi
```

This will:
- Monitor `/image_raw` topic
- Automatically detect objects in published images
- Save results to `src/detection/detection_results_sahi/`

---

### Method 2: Using Convenience Scripts

```bash
cd ~/astra/ros2_ws/src/video_cam

# Test connection
./siyi_unified_pipeline_start.sh test

# Launch pipeline
./siyi_unified_pipeline_start.sh launch

# In another terminal: Trigger capture
./siyi_unified_pipeline_start.sh trigger
```

---

## 📊 Understanding the Workflow

### Data Flow Diagram

```
SIYI Camera (192.168.144.25)
    │
    ├─ UDP Port 37260 ──────► Capture Control Commands
    ├─ HTTP Port 82 ─────────► SD Card File API
    └─ HTTP Download ────────► Image File Transfer
                                     │
                                     ▼
                         Camera Interface
                         (camera_interface.py)
                                     │
                                     ▼
                         Pipeline Orchestrator
                         (pipeline_orchestrator.py)
                         │                    │
                         │                    ▼
                         │            Storage Manager
                         │            (storage_manager.py)
                         │                    │
                         ▼                    ▼
                  ROS 2 Topics          Local Files
                  (/image_raw)          (video_cam_data/)
                         │
                         ▼
                  Detection Node
                  (object_detection_sahi)
```

### File Storage Structure

```
~/astra/ros2_ws/
└── video_cam_data/
    └── mapping_photos/          # Single directory for all images
        ├── DSCF0001.JPG         # Original SD card filename
        ├── DSCF0002.JPG
        ├── DSCF0003.JPG
        └── .tracking_state.json # Prevents duplicate downloads
```

**Note:** The new architecture uses a **single directory** for simplicity. Old multi-directory structure is deprecated.

---

## 🎛️ ROS Topics & Commands

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | `sensor_msgs/Image` | Published image (after capture) |
| `/camera/status` | `std_msgs/String` | Camera status updates |
| `/camera/disk_free_mb` | `std_msgs/Float64` | Free disk space in MB |
| `/mavros/statustext/send` | `mavros_msgs/StatusText` | Status messages |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/trigger` | `std_msgs/Bool` | Trigger image capture |
| `/camera/set_resolution` | `std_msgs/String` | Change resolution (4K/2.7K/1080P) |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | Altitude (for auto-enable) |

### Common Commands

```bash
# === CAPTURE COMMANDS ===

# Single capture
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

# Continuous captures (every 2 seconds)
ros2 topic pub --rate 0.5 /camera/trigger std_msgs/msg/Bool "data: true"


# === RESOLUTION COMMANDS ===

# Set to 4K (3840x2160) - default
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# Set to 2.7K (2704x1520)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# Set to 1080P (1920x1080)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"


# === MONITORING COMMANDS ===

# Watch camera status
ros2 topic echo /camera/status

# Monitor image publication rate
ros2 topic hz /image_raw

# View images in real-time
ros2 run rqt_image_view rqt_image_view /image_raw

# Check node is running
ros2 node list | grep siyi

# Check all camera topics
ros2 topic list | grep camera


# === DEBUGGING COMMANDS ===

# See full camera node info
ros2 node info /siyi_unified_pipeline

# Check image message structure
ros2 topic echo --once /image_raw

# Monitor disk space
ros2 topic echo /camera/disk_free_mb
```

---

## 🔍 Key Configuration Parameters

From `config.py`:

```python
# Network Settings
CAMERA_IP = "192.168.144.25"
CONTROL_PORT = 37260  # UDP for capture commands
MEDIA_PORT = 82       # HTTP for file API

# Timing
CAPTURE_TIMEOUT_SECONDS = 15.0    # Max wait for image on SD
SD_POLL_INTERVAL = 0.5            # How often to check SD card
MAX_PIPELINE_DURATION = 60.0      # Overall pipeline timeout

# Storage
MIN_FREE_SPACE_MB = 50            # Minimum disk space required
REQUIRED_DOWNLOAD_SPACE_MB = 10   # Space needed per download

# Image Verification
RESOLUTION_SPECS = {
    '4K': {'min_width': 3000, 'min_height': 1600, 'min_file_size': 50000},
    '2.7K': {'min_width': 2000, 'min_height': 1200, 'min_file_size': 30000},
    '1080P': {'min_width': 1800, 'min_height': 900, 'min_file_size': 20000}
}
```

---

## 🔧 Troubleshooting

### Problem: "Cannot connect to camera"

**Symptoms:**
```
[ERROR] [camera_interface]: Connection failed to 192.168.144.25
```

**Solutions:**
```bash
# 1. Check physical connection
ip addr show eth0  # Should show 192.168.144.100

# 2. Ping camera
ping 192.168.144.25  # Should respond

# 3. Test API directly
curl http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0

# 4. Restart camera (power cycle)

# 5. Check firewall
sudo ufw status
sudo ufw allow from 192.168.144.0/24
```

---

### Problem: "SD verification timeout"

**Symptoms:**
```
[ERROR] [pipeline_orchestrator]: ✗ Timeout after 15.0s
[ERROR] [pipeline_orchestrator]: [Phase 2] Image not found on SD card
```

**Solutions:**
```bash
# 1. Check if SD card is inserted in camera

# 2. Verify SD card has space
curl http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0
# Look for directories

# 3. Try clearing SD card (backup first!)
# Access camera web interface at http://192.168.144.25

# 4. Increase timeout in config.py
CAPTURE_TIMEOUT_SECONDS = 30.0  # Instead of 15.0
```

---

### Problem: "Package 'video_cam' not found"

**Solutions:**
```bash
# 1. Rebuild workspace
cd ~/astra/ros2_ws
colcon build --packages-select video_cam

# 2. Source workspace
source install/setup.bash

# 3. Verify package exists
ros2 pkg list | grep video_cam

# 4. Check setup.py entry points
cat src/video_cam/setup.py | grep siyi
```

---

### Problem: "Insufficient disk space"

**Symptoms:**
```
[ERROR] [storage_manager]: Disk space critical: 45.2MB free (need 50.0MB)
```

**Solutions:**
```bash
# 1. Check disk space
df -h ~/astra/ros2_ws

# 2. Clean old images
cd ~/astra/ros2_ws/video_cam_data/mapping_photos
ls -lh | wc -l  # Count images
rm DSCF00[0-5]*.JPG  # Delete specific ranges

# 3. Adjust threshold in config.py
MIN_FREE_SPACE_MB = 20  # Reduce if needed
```

---

### Problem: "Image dimensions below expected"

**Symptoms:**
```
[WARN] [storage_manager]: Image dimensions below expected for 4K
```

**Cause:** Camera might be set to wrong resolution

**Solutions:**
```bash
# Set resolution explicitly
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# Verify with next capture
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

# Check saved image
cd ~/astra/ros2_ws/video_cam_data/mapping_photos
identify -format "%f: %wx%h\n" DSCF*.JPG | tail -1
```

---

### Problem: Node crashes or freezes

**Solutions:**
```bash
# 1. Check for errors
ros2 node list
ros2 topic list

# 2. Kill and restart
pkill -9 -f siyi_unified_pipeline
ros2 run video_cam siyi_unified_pipeline_refactored

# 3. Check logs
cd ~/.ros/log/latest
cat siyi_unified_pipeline-*.log

# 4. Monitor system resources
htop  # Check CPU/memory usage
```

---

## 📝 Quick Reference Card

```bash
# === SETUP ===
cd ~/astra/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash

# === RUN ===
# Terminal 1
ros2 run video_cam siyi_unified_pipeline_refactored

# Terminal 2
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

# Terminal 3 (optional)
ros2 run detection object_detection_sahi

# === CHECK ===
ros2 topic list | grep camera
ros2 node info /siyi_unified_pipeline
ls -lh ~/astra/ros2_ws/video_cam_data/mapping_photos/

# === STOP ===
Ctrl+C in all terminals
```

---

## 🎓 Understanding the Code

### Camera Interface (`camera_interface.py`)

**Purpose:** Direct communication with camera hardware

**Key Methods:**
- `send_capture_command()` - Sends UDP packet to trigger capture
- `get_directories()` - Gets SD card directory structure via HTTP
- `get_media_list()` - Lists files in a directory
- `download_image()` - Downloads image bytes via HTTP
- `decode_image()` - Converts JPEG bytes to numpy array

**Protocol Used:**
- UDP (port 37260) for real-time commands
- HTTP (port 82) for file operations

---

### Storage Manager (`storage_manager.py`)

**Purpose:** Safe file storage with verification

**Key Methods:**
- `save_image()` - Atomic write with verification
- `verify_file()` - Checks dimensions and integrity
- `check_disk_space()` - Ensures enough space
- `load_tracking_state()` - Prevents duplicate downloads
- `save_tracking_state()` - Persists download history

**Safety Features:**
- Atomic writes (write to `.tmp` then rename)
- JPEG header/footer verification
- Dimension checking
- File size validation

---

### Pipeline Orchestrator (`pipeline_orchestrator.py`)

**Purpose:** Coordinates the 4-phase workflow

**Key Methods:**
- `execute_pipeline()` - Runs full capture sequence
- `_phase1_capture()` - Trigger camera
- `_phase2_index()` - Find new image on SD
- `_phase3_download()` - Download and save
- `get_state()` - Check if busy

**State Machine:**
```
IDLE → CAPTURING → INDEXING → DOWNLOADING → IDLE
                                    ↓
                                 FAILED
```

---

### SIYI Node (`siyi_node_refactored.py`)

**Purpose:** ROS 2 wrapper that connects everything

**Key Features:**
- ROS 2 parameter server integration
- Topic publishers/subscribers
- Main event loop
- Simulation mode support

**Main Loop:**
```python
def _pipeline_loop(self):
    """Called at 10 Hz"""
    # Check for capture trigger
    if self.capture_requested.is_set():
        self.capture_requested.clear()
        success = self.pipeline.execute_pipeline()
        # Publish status
        
    # Publish heartbeat messages
```

---

## 🌟 Best Practices

1. **Always source workspace** before running commands
2. **Check camera connection** before starting node
3. **Monitor disk space** regularly
4. **Use 4K resolution** for best results
5. **Wait for "Pipeline ready"** message before triggering
6. **Don't spam triggers** - wait for pipeline to complete (3-5s)
7. **Back up important images** from `video_cam_data/`
8. **Check logs** in `~/.ros/log/` if issues occur

---

## 📚 Additional Resources

- Camera API Documentation: See `siyi_documentations/` directory
- Quick Start Guide: `QUICK_START.md`
- Simplification Summary: `SIMPLIFICATION_SUMMARY.md`
- Setup Check Script: `./siyi_setup_check.sh`

---

**End of Guide** | Questions? Check the troubleshooting section or logs in `~/.ros/log/latest/`
