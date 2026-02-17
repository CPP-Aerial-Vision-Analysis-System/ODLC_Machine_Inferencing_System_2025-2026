# QUICK START: Full Cycle Image Processing

## 🚀 The Fastest Way to Run Everything

### 1️⃣ Test Connection (Do this first!)
```bash
cd ~/astra/ros2_ws/src/video_cam
./run_full_pipeline.sh test
```

Expected output:
```
✓ Camera is reachable at 192.168.144.25
✓ Camera API is responding
✓ ROS workspace is built
[TEST] All checks passed! System is ready.
```

---

### 2️⃣ Launch the Pipeline

**Terminal 1:**
```bash
cd ~/astra/ros2_ws/src/video_cam
./run_full_pipeline.sh launch
```

Wait for this message:
```
[INFO] Pipeline ready. Waiting for triggers...
```

---

### 3️⃣ Capture Images

**Terminal 2:**
```bash
cd ~/astra/ros2_ws/src/video_cam
./run_full_pipeline.sh trigger
```

Or for continuous captures every 2 seconds:
```bash
cd ~/astra/ros2_ws
source install/setup.bash
ros2 topic pub --rate 0.5 /camera/trigger std_msgs/msg/Bool "data: true"
```

---

### 4️⃣ (Optional) Run Object Detection

**Terminal 3:**
```bash
cd ~/astra/ros2_ws
source install/setup.bash
ros2 run detection object_detection_sahi
```

---

## 📊 Check Status

```bash
cd ~/astra/ros2_ws/src/video_cam
./run_full_pipeline.sh status
```

---

## 📁 Where Are My Images?

```bash
cd ~/astra/ros2_ws/video_cam_data/mapping_photos
ls -lth
```

Images are named like: `DSCF0001.JPG`, `DSCF0002.JPG`, etc.

---

## 🎛️ Change Resolution

```bash
cd ~/astra/ros2_ws
source install/setup.bash

# 4K (default - best quality)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# 2.7K (medium)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# 1080P (fast)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"
```

---

## 🔧 Troubleshooting

### Camera not responding?
```bash
# 1. Check Jetson IP
ip addr show eth0
# Should show: 192.168.144.100

# 2. Ping camera
ping 192.168.144.25

# 3. Restart camera (power cycle)
```

### Node won't start?
```bash
# Rebuild package
cd ~/astra/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
```

### Images not saving?
```bash
# Check disk space
df -h ~/astra/ros2_ws

# Check directory exists
ls -la ~/astra/ros2_ws/video_cam_data/mapping_photos/
```

---

## 📖 Full Documentation

For complete details, see: **HOW_TO_RUN_FULL_PIPELINE.md**

Topics covered:
- Complete architecture explanation
- All ROS topics and commands
- Configuration parameters
- Detailed troubleshooting
- Code walkthrough

---

## ⚡ Command Cheat Sheet

```bash
# === ESSENTIAL COMMANDS ===
./run_full_pipeline.sh test      # Test everything
./run_full_pipeline.sh launch    # Start camera node
./run_full_pipeline.sh trigger   # Take a picture
./run_full_pipeline.sh status    # Check system

# === MANUAL ROS COMMANDS ===
ros2 run video_cam siyi_unified_pipeline_refactored    # Launch node
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"  # Capture
ros2 topic echo /camera/status   # Monitor status
ros2 run rqt_image_view rqt_image_view /image_raw  # View images

# === FILE LOCATIONS ===
~/astra/ros2_ws/video_cam_data/mapping_photos/  # Images saved here
~/.ros/log/latest/                               # ROS logs
~/astra/ros2_ws/src/video_cam/                  # Source code
```

---

## 🎯 What Each Component Does

| Component | File | Purpose |
|-----------|------|---------|
| **Config** | `config.py` | All settings (IPs, timeouts, paths) |
| **Camera Interface** | `camera_interface.py` | Talks to camera (UDP/HTTP) |
| **Storage Manager** | `storage_manager.py` | Saves images safely |
| **Pipeline Orchestrator** | `pipeline_orchestrator.py` | Coordinates workflow |
| **ROS Node** | `siyi_node_refactored.py` | ROS 2 integration |

---

## 🌟 The 4-Phase Pipeline

Every capture goes through:

```
1. CAPTURE CONTROL    → Send UDP command to camera
2. SD CARD INDEXING   → Wait for image on SD card
3. DOWNLOAD & SAVE    → Get image via HTTP, save locally
4. PUBLISH            → Send to /image_raw topic
```

**Total time:** 3-5 seconds per image

---

## 💡 Pro Tips

1. **Always test connection first** - saves debugging time
2. **Use 4K resolution** - best for detection
3. **Don't spam triggers** - wait 5 seconds between captures
4. **Monitor disk space** - each 4K image is ~2-3MB
5. **Source workspace in every new terminal**

---

**Need help?** Check `HOW_TO_RUN_FULL_PIPELINE.md` for detailed troubleshooting!
