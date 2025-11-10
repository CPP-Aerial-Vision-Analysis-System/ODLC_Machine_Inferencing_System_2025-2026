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

# Quick fix: Install PyTorch with CUDA support from NVIDIA
# See: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
```

**Important:** Without GPU, object detection will be very slow (1-2 FPS vs 10-20 FPS with GPU).

---

### 2. run image capturing and image detection

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam 
source install/setup.bash
ros2 run video_cam image_pub
```

# for starting the camera
```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash
ros2 run video_cam object_detection_sahi
```

------
# if you are on Aro's linux, here are the commands(you are going to use Jazzy instead of humble)
```bash
   cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
   colcon build --packages-select video_cam --symlink-install
   source install/setup.bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 run video_cam object_detection_sahi
```

# for starting the camera
```bash
   cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
   colcon build --packages-select video_cam --symlink-install
   source install/setup.bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 run video_cam image_pub
```

---

### External Resources:
- **PyTorch for Jetson:** https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
- **YOLO Documentation:** https://docs.ultralytics.com/
- **SAHI Documentation:** https://github.com/obss/sahi
- **ROS2 Humble Docs:** https://docs.ros.org/en/humble/

---

**System Status:** Functional (CPU mode) | GPU support needs configuration  
**Maintainer:** CPP Aerial Vision Analysis System Team  
**Last Tested:** November 8, 2025 on Jetson Orin Nano
