# SIYI Camera System - Build & Run Instructions

## ✅ System Fixed and Working!

The SIYI camera system has been successfully built and is now operational. The system:
- ✅ Takes 4K images from SIYI A8 camera
- ✅ Saves images to camera SD card
- ✅ Downloads images from SD card via Ethernet to Jetson
- ✅ Falls back to stream capture if SD card unavailable

---

## Quick Start

### 1. Build the Package (First Time Only)

```bash
cd /home/astra-dev/astra/ros2_ws
colcon build --packages-select video_cam
```

### 2. Launch the Camera System

**Option A: Using the convenience script**
```bash
/home/astra-dev/astra/ros2_ws/src/video_cam/launch_siyi.sh
```

**Option B: Manual launch**
```bash
cd /home/astra-dev/astra/ros2_ws
source install/setup.bash
ros2 launch video_cam siyi_camera2.launch.py
```

### 3. Capture Photos

**In a new terminal:**

**Option A: Using the convenience script**
```bash
/home/astra-dev/astra/ros2_ws/src/video_cam/capture_photo.sh
```

**Option B: Manual trigger**
```bash
cd /home/astra-dev/astra/ros2_ws
source install/setup.bash
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
```

---

## What the System Does

1. **Connects to Camera**
   - Camera IP: 192.168.144.25
   - Stream: RTSP (for live view)
   - API: HTTP (for SD card access)

2. **On Photo Trigger:**
   - Sends 4K capture command to camera
   - Waits for photo to appear on SD card
   - Downloads photo via Ethernet
   - Saves to local directories:
     - `~/astra/ros2_ws/video_cam_data/camera_feed/` (for processing)
     - `~/astra/ros2_ws/video_cam_data/mapping_photos/` (for archival)

3. **Fallback Mode:**
   - If SD card unavailable, captures from RTSP stream
   - Ensures photos are always saved

---

## System Status

When the camera node starts successfully, you'll see:
```
✓ Found 1 directories on SD card
✓ Using directory: 101SIYI_IMG
✓ Initial photo count: 0
✓ Camera stream connected
 SIYI A8 COMBINED NODE INITIALIZED
```

---

## Troubleshooting

### "Package 'video_cam' not found"
**Solution:** Build and source the workspace:
```bash
cd /home/astra-dev/astra/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```

### "SD VERIFICATION TIMEOUT"
This means the photo isn't appearing on the SD card. Check:
1. SD card is inserted in camera
2. Camera is powered on and connected
3. Ethernet cable is connected (192.168.144.25)
4. Camera settings allow SD card storage

**Note:** The system will automatically fall back to stream capture if SD fails.

### "Cannot connect to camera"
Check network connection:
```bash
ping 192.168.144.25
```
Should show responses with <5ms latency.

---

## File Locations

### Source Code:
- Launch file: `/home/astra-dev/astra/ros2_ws/src/video_cam/launch/siyi_camera2.launch.py`
- Main node: `/home/astra-dev/astra/ros2_ws/src/video_cam/video_cam/image_pub_siyi2.py`
- Helper scripts:
  - `/home/astra-dev/astra/ros2_ws/src/video_cam/launch_siyi.sh`
  - `/home/astra-dev/astra/ros2_ws/src/video_cam/capture_photo.sh`

### Data Directories:
- Camera feed: `/home/astra-dev/astra/ros2_ws/video_cam_data/camera_feed/`
- Mapping photos: `/home/astra-dev/astra/ros2_ws/video_cam_data/mapping_photos/`

---

## Advanced Usage

### Change Resolution
```bash
# 4K (default - 3840x2160)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# 2.7K (2704x1520)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# 1080P (1920x1080)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"
```

### View Live Stream
```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

### Monitor System
```bash
# Watch ROS2 nodes
ros2 node list

# Check camera status
ros2 topic echo /camera/status

# View latest photos
ls -lht /home/astra-dev/astra/ros2_ws/video_cam_data/camera_feed/ | head -5
```

---

## Success Indicators

### ✅ Build Success:
```
Finished <<< video_cam [X.XXs]
Summary: 1 package finished
```

### ✅ Launch Success:
```
SIYI A8 COMBINED NODE INITIALIZED
Camera IP: 192.168.144.25
Resolution: 4K
SD Card Directory: 101SIYI_IMG
✓ Camera stream connected
```

### ✅ Photo Capture Success (with SD card):
```
[1/3] Triggering 4K capture...
[2/3] Waiting for photo on SD card...
[3/3] Downloading from SD card...
✓ Saved: camera_feed/photo_4K_*.jpg
✓ Saved: mapping_photos/mapping_*.jpg
```

---

## System Architecture

```
SIYI A8 Camera (192.168.144.25)
    ↓
    ├─ RTSP Stream (port 8554) ──→ Live video
    ├─ HTTP API (port 82) ──────→ SD card access
    └─ UDP SDK (port 37260) ────→ Camera control
    ↓
ROS2 Node (image_pub_siyi2)
    ↓
    ├─ /camera/trigger ──→ Capture command
    ├─ /image_raw ──────→ Published images
    └─ /camera/status ──→ System status
    ↓
Local Storage
    ├─ camera_feed/ ──→ For detection/processing
    └─ mapping_photos/ ──→ For archival
```

---

## Need Help?

1. Check full documentation: `/home/astra-dev/astra/ros2_ws/src/video_cam/video_cam/QUICK_START.md`
2. Run setup check: `/home/astra-dev/astra/ros2_ws/src/video_cam/siyi_setup_check.sh`

---

**Last Updated:** January 15, 2026  
**Status:** ✅ Working - Build Complete
