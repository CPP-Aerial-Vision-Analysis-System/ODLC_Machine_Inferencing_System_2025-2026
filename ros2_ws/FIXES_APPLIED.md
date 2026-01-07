# Critical Fixes Applied to image_pub_siyi.py

## Date: January 7, 2026

### Issue #1: MEDIA_URL Undefined ✅ FIXED
- **Problem**: `media_command()` referenced `self.MEDIA_URL` which was never defined
- **Solution**: Removed unused `media_command()` method entirely (lines 404-421)
- **Status**: Fixed via deletion

### Issue #2: Duplicate cv_bridge Initialization ✅ FIXED  
- **Problem**: cv_bridge was initialized twice in `__init__` (lines 62-67 and 84-89)
- **Solution**: Removed second initialization block (lines 84-89)
- **Status**: Fixed

### Issue #3: Worker Thread for SD Capture ⚠️ PARTIALLY COMPLETE
- **Problem**: `camera_loop()` performs blocking HTTP downloads in 0.1s timer, blocking ROS executor
- **Solution Started**:
  - Added imports: `Thread, Event, Queue` from threading
  - Added worker thread infrastructure in `__init__`
  - Created `_sd_capture_worker()` method to handle blocking operations
  - Modified `camera_trigger_callback()` to queue requests instead of setting flag
  - Modified `auto_capture_callback()` to use queue
- **Still TODO**: Remove duplicate SD capture logic from `camera_loop()` (lines 714-772)
  - The camera_loop should ONLY stream RTSP frames
  - All SD capture should go through worker thread

### Issue #4: Altitude Logic Unclear ⚠️ NOT YET FIXED
- **Problem**: `ALT_THRESHOLD = -13.716` with `current_alt >= threshold` is confusing
- **Solution Needed**: 
  ```python
  # NED frame: negative altitude = higher up (Ardupilot convention)
  # This threshold means "at least 13.716m above home"
  MIN_REL_ALT_NED = -13.716  # meters (NED frame, negative = up)
  ```

### Issue #5: Hard-coded Configuration ⚠️ NOT YET FIXED
- **Problem**: Camera IP, paths, thresholds are hard-coded constants
- **Solution Needed**: Convert to ROS parameters:
  - `CAM_IP` → `~/camera_ip` (default: "192.168.144.25")
  - `ALT_THRESHOLD` → `~/min_altitude` (default: -13.716)
  - `photo_path` → `~/photo_save_dir`
  - `use_real_camera` → `~/use_real_camera` (default: True)
  - `RTSP URL` → `~/rtsp_url`

### Issue #6: GStreamer Subprocess Risk ⚠️ NOT YET FIXED
- **Problem**: `init_gstreamer_subprocess()` is risky and can zombie
- **Solution Needed**: Remove or guard with parameter `~/enable_gstreamer_fallback` (default: False)

## Summary
- ✅ 2 issues fixed (MEDIA_URL, duplicate cv_bridge)
- ⚠️ 1 issue partially fixed (worker thread added, but camera_loop not yet cleaned)
- ⚠️ 3 issues remaining (altitude docs, ROS parameters, GStreamer)

## Next Steps
1. Complete worker thread by removing lines 714-772 from camera_loop
2. Add altitude frame documentation
3. Add ROS parameters using declare_parameter()
4. Remove or guard GStreamer subprocess

## Rebuild Command
```bash
cd /home/astra-dev/astra/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```
