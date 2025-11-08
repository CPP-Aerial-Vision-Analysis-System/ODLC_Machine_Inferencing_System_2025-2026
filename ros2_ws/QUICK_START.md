# Quick Start Guide

## Fix Build Error

The build is failing because ROS2 needs to be sourced first. Here are two ways to fix it:

### Option 1: Use the Build Script (Recommended)

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
./build.sh --video-cam-only
```

This script will:
- Automatically detect your ROS2 distribution (Jazzy/Humble/Iron)
- Source ROS2 setup
- Build the video_cam package
- Show you the next steps

### Option 2: Manual Build

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws

# Source ROS2 (choose your distribution)
source /opt/ros/jazzy/setup.bash    # For Jazzy
# OR
source /opt/ros/humble/setup.bash   # For Humble
# OR
source /opt/ros/iron/setup.bash     # For Iron

# Build the workspace
colcon build --packages-select video_cam --symlink-install

# Source the workspace
source install/setup.bash
```

## Run SAHI Object Detection

### Quick Run (Recommended)

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
./run_sahi.sh
```

### Manual Run

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash  # or humble/iron

# Source workspace
source install/setup.bash

# Run detection
ros2 run video_cam object_detection_sahi
```

## Build All Packages

```bash
cd ~/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
./build.sh
```

## Build with Custom Options

```bash
# Build specific packages
./build.sh --packages-select video_cam interfaces

# Build with symlink install
./build.sh --symlink-install

# Build video_cam only (shortcut)
./build.sh --video-cam-only
```

## Troubleshooting

### Error: "Could not find ROS2 installation"
- Make sure ROS2 is installed: `ls /opt/ros/`
- If ROS2 is in a different location, modify the build script

### Error: "colcon is not installed"
```bash
sudo apt install python3-colcon-common-extensions
```

### Error: "ament_cmake not found"
- Make sure you've sourced ROS2 before building
- Use the build script: `./build.sh`

