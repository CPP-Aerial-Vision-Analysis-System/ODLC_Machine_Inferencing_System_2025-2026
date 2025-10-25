# How to Run Video Camera and Object Detection System

**System:** NVIDIA Jetson Orin Nano  
**ROS2 Distribution:** Humble  
**Last Updated:** October 25, 2025

---

## Quick Start

### 1. Enable GPU Acceleration (CRITICAL - Do This First!)

Your Jetson has a GPU but PyTorch is using CPU only. Fix this for **5-10x performance boost**:

```bash
# Check GPU status
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam/video_cam
python3 check_gpu.py

# Follow GPU_SETUP_GUIDE.md for full instructions
# Quick fix: Install PyTorch with CUDA support from NVIDIA
# See: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
```

**Important:** Without GPU, object detection will be very slow (1-2 FPS vs 10-20 FPS with GPU).

---

### 2. Build the Package

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam --symlink-install
source install/setup.bash
```

---

### 3. Run the Camera Node

Terminal 1:
```bash
source ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/setup.bash
ros2 run video_cam image_pub
```

**Expected Output:**
```
[INFO] [siyi_a8_publisher]: Using real camera: True
[INFO] [siyi_a8_publisher]: Successfully connected to camera!
[INFO] [siyi_a8_publisher]: Camera Frame Publishing
```

---

### 4. Run Object Detection

Terminal 2 (with SAHI for small objects):
```bash
source ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/setup.bash
ros2 run video_cam object_detection_sahi
```

**Or** basic object detection:
```bash
ros2 run video_cam object_detection
```

**Expected Output (with GPU):**
```
[INFO] [sahi_object_detection_node]: 🚀 CUDA GPU Detected!
[INFO] [sahi_object_detection_node]:    Device: Orin
[INFO] [sahi_object_detection_node]: Using device: cuda:0
[INFO] [sahi_object_detection_node]: SAHI Object Detection Node Initialized
```

**Expected Output (CPU-only - needs GPU fix):**
```
[WARN] [sahi_object_detection_node]: ⚠️  No GPU detected, using CPU
[ERROR] [sahi_object_detection_node]:    ⚠️  JETSON DEVICE DETECTED BUT GPU NOT AVAILABLE!
[ERROR] [sahi_object_detection_node]:    Install PyTorch with CUDA support for GPU acceleration
```

---

## System Components

### 1. Camera Publisher (`image_pub`)
- Connects to SIYI A8 camera via RTSP
- Publishes frames to `/image_raw` topic
- Saves images to `camera_feed/` directory
- **Features:**
  - Multi-backend camera support (FFmpeg, GStreamer, default)
  - Automatic fallback mechanisms
  - cv_bridge compatibility layer
  - Real-time frame publishing

### 2. Object Detection - SAHI (`object_detection_sahi`)
- Optimized for **small object detection** (tents, people in aerial imagery)
- Uses YOLO with Slicing Aided Hyper Inference (SAHI)
- **GPU-accelerated** when properly configured
- Publishes detection results to `/sahi_detection_results`
- Saves annotated images to `detection_results_sahi/`

### 3. Object Detection - Standard (`object_detection`)
- General purpose object detection
- Uses YOLO + MobileNet
- **GPU-accelerated** when properly configured
- Publishes detection results to `/detection_results`
- Saves annotated images to `detection_results/`

---

## ROS2 Topics

### Published Topics
- `/image_raw` - Raw camera frames (Image)
- `/sahi_detection_results` - SAHI annotated frames (Image)
- `/sahi_detection_info` - Detection metadata (String)
- `/detection_results` - Standard detection frames (Image)
- `/detection_info` - Detection metadata (String)

### Subscribed Topics (Camera Node)
- `/camera/trigger` - Manual capture trigger (Bool)
- `/mavros/global_position/rel_alt` - Altitude for auto-enable (Float64)
- `/webcam/image_raw` - Simulation mode images (Image)

---

## Configuration

### Camera Settings (`image_pub_siyi.py`)
```python
use_real_camera = True  # Set to False for simulation
ALT_THRESHOLD = 13.716  # Minimum altitude to enable camera (meters)
```

### SAHI Detection Parameters
```bash
# Via ROS2 parameters
ros2 run video_cam object_detection_sahi \
    --ros-args \
    -p model_path:=yolo11s.pt \
    -p confidence_threshold:=0.15 \
    -p slice_height:=512 \
    -p slice_width:=512 \
    -p overlap_height_ratio:=0.3 \
    -p overlap_width_ratio:=0.3 \
    -p device:=auto  # auto, cuda:0, cpu, mps
```

### Model Selection
Available models (in `ros2_ws/` directory):
- `yolo11n.pt` - Nano (fastest, least accurate)
- `yolo11s.pt` - Small (balanced) **← Current default**
- `yolo11m.pt` - Medium (slower, more accurate)
- `yolo11l.pt` - Large (slowest, most accurate)

**Note:** Larger models require more GPU memory. On Jetson Orin Nano, stick with nano or small.

---

## Performance Optimization

### GPU Memory Management

If you get "CUDA out of memory" errors:

1. **Use smaller model:**
   ```bash
   # In object_detection_sahi.py, change:
   model_path='yolo11n.pt'  # Instead of yolo11s.pt
   ```

2. **Reduce slice size:**
   ```bash
   ros2 run video_cam object_detection_sahi \
       --ros-args \
       -p slice_height:=384 \
       -p slice_width:=384
   ```

3. **Clear GPU memory:**
   ```bash
   python3 -c "import torch; torch.cuda.empty_cache()"
   ```

### Monitor Performance

```bash
# Terminal 1: GPU monitoring
watch -n 1 nvidia-smi

# Terminal 2: Jetson stats
tegrastats

# Terminal 3: ROS2 topic frequency
ros2 topic hz /sahi_detection_results
```

---

## Troubleshooting

### Issue: Camera not connecting

**Symptoms:**
```
[ERROR] [siyi_a8_publisher]: Could not open video stream
```

**Solutions:**
1. Check camera IP is reachable:
   ```bash
   ping 192.168.144.25
   ```

2. Test RTSP stream directly:
   ```bash
   gst-launch-1.0 rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! fakesink
   ```

3. Check camera power and network connection

---

### Issue: GPU not detected

**Symptoms:**
```
[WARN] No GPU detected, using CPU
[ERROR] JETSON DEVICE DETECTED BUT GPU NOT AVAILABLE!
```

**Solution:**
```bash
# 1. Run diagnostic
python3 check_gpu.py

# 2. Follow GPU setup guide
cat GPU_SETUP_GUIDE.md

# 3. Install PyTorch with CUDA
# See: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
```

---

### Issue: cv_bridge / NumPy errors

**Symptoms:**
```
AttributeError: _ARRAY_API not found
```

**Solution:**
```bash
pip install "numpy<2" --force-reinstall
```

**Already Fixed:** Code now has fallback image conversion methods.

---

### Issue: Slow detection (< 5 FPS)

**Likely Cause:** Running on CPU instead of GPU

**Check:**
```bash
# Look for this in the logs:
[INFO] Using device: cpu  # ← Problem!
# Should be:
[INFO] Using device: cuda:0  # ← Correct
```

**Solution:** Fix GPU support (see GPU_SETUP_GUIDE.md)

---

### Issue: Package build errors

**Solution:**
```bash
# Clean and rebuild
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select video_cam --symlink-install
source install/setup.bash
```

---

## Directory Structure

```
video_cam/
├── video_cam/
│   ├── image_pub_siyi.py          # Camera publisher node
│   ├── object_detection.py        # Standard detection node
│   ├── object_detection_sahi.py   # SAHI detection node
│   ├── check_gpu.py               # GPU diagnostic tool
│   ├── install_pytorch_cuda.sh    # PyTorch CUDA installer
│   ├── Documentation.md           # Technical documentation
│   ├── GPU_SETUP_GUIDE.md         # GPU setup instructions
│   └── HOWTORUN.MD                # This file
├── launch/
│   └── sahi_detection.launch.py   # Launch file (future)
├── package.xml                     # ROS2 package manifest
└── setup.py                        # Python package setup
```

### Output Directories
- `install/video_cam/share/video_cam/camera_feed/` - Raw camera images
- `install/video_cam/share/video_cam/detection_results_sahi/` - SAHI detections
- `install/video_cam/share/video_cam/detection_results/` - Standard detections
- `install/video_cam/share/video_cam/mapping_photos/` - Triggered photos

---

## Common Workflows

### Workflow 1: Real-time Detection with Camera

```bash
# Terminal 1: Start camera
ros2 run video_cam image_pub

# Terminal 2: Start detection
ros2 run video_cam object_detection_sahi

# Terminal 3: Monitor performance
ros2 topic hz /sahi_detection_results
```

### Workflow 2: Process Existing Images

```bash
# 1. Copy images to camera_feed directory
cp /path/to/images/*.jpg \
   ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/video_cam/share/video_cam/camera_feed/

# 2. Run detection
ros2 run video_cam object_detection_sahi

# 3. Results saved to detection_results_sahi/
```

### Workflow 3: Simulation Mode (No Physical Camera)

```bash
# In image_pub_siyi.py, set:
use_real_camera = False

# Terminal 1: Publish test images to /webcam/image_raw
ros2 topic pub /webcam/image_raw sensor_msgs/msg/Image ...

# Terminal 2: Run camera node (republishes)
ros2 run video_cam image_pub

# Terminal 3: Run detection
ros2 run video_cam object_detection_sahi
```

---

## Performance Expectations

### With GPU (CUDA-enabled PyTorch):
- **Detection Speed:** 10-20 FPS
- **Frame Processing:** 50-100ms
- **GPU Usage:** 60-80%
- **Power Consumption:** ~15-20W

### With CPU (current state):
- **Detection Speed:** 1-2 FPS ⚠️
- **Frame Processing:** 500-1000ms ⚠️
- **CPU Usage:** 100% on all cores
- **Power Consumption:** ~10W

**⚠️ GPU acceleration provides 5-10x speedup!**

---

## Dependencies

### Required Packages (Already Installed):
- `rclpy` - ROS2 Python
- `sensor_msgs` - ROS2 messages
- `cv_bridge` - OpenCV/ROS bridge (with fallback)
- `opencv-python` - Computer vision
- `numpy<2` - Numerical computing
- `ultralytics` - YOLO
- `sahi` - Slicing inference
- `torch` - PyTorch (⚠️ needs CUDA version)
- `pillow` - Image processing

### Optional:
- `tensorflow` - For MobileNet (currently CPU-only)

---

## Safety and Best Practices

1. **Always test on ground before flight**
2. **Monitor GPU temperature** (use `tegrastats`)
3. **Set altitude threshold** appropriately for your flight area
4. **Backup detection results** regularly
5. **Use smaller models** (nano/small) to conserve power and memory
6. **Enable GPU** for real-time performance requirements

---

## Recent Changes (October 25, 2025)

### Camera Node (`image_pub_siyi`)
- ✅ Fixed NumPy 2.x compatibility with cv_bridge
- ✅ Added manual image conversion fallback
- ✅ Multi-backend camera support (FFmpeg priority)
- ✅ Improved error handling and logging
- ✅ Throttled log messages

### Object Detection
- ✅ Added GPU auto-detection with Jetson-specific checks
- ✅ Detailed error messages when GPU unavailable
- ✅ Automatic device selection (cuda:0, mps, or cpu)
- ✅ GPU memory monitoring
- ✅ Instructions for fixing GPU support

### Documentation
- ✅ Created GPU_SETUP_GUIDE.md
- ✅ Created check_gpu.py diagnostic tool
- ✅ Created install_pytorch_cuda.sh helper script
- ✅ Updated Documentation.md with all changes

---

## Next Steps

1. **PRIORITY:** Enable GPU acceleration (see GPU_SETUP_GUIDE.md)
2. Test camera connection at target deployment site
3. Calibrate detection confidence thresholds for your use case
4. Create launch files for simultaneous node startup
5. Integrate with mission planning/MAVROS
6. Implement real-time telemetry streaming

---

## Support and Resources

### Documentation Files:
- `Documentation.md` - Complete technical documentation
- `GPU_SETUP_GUIDE.md` - GPU acceleration setup
- `HOWTORUN.MD` - This file

### Diagnostic Tools:
- `python3 check_gpu.py` - GPU detection and benchmark
- `python3 image_pub_siyi.py` - Direct camera test
- `ros2 topic list` - Check active topics
- `ros2 topic echo /image_raw` - View camera feed
- `ros2 node list` - Check running nodes

### External Resources:
- **PyTorch for Jetson:** https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- **YOLO Documentation:** https://docs.ultralytics.com/
- **SAHI Documentation:** https://github.com/obss/sahi
- **ROS2 Humble Docs:** https://docs.ros.org/en/humble/

---

**System Status:** ✅ Functional (CPU mode) | ⚠️ GPU support needs configuration  
**Maintainer:** CPP Aerial Vision Analysis System Team  
**Last Tested:** October 25, 2025 on Jetson Orin Nano
