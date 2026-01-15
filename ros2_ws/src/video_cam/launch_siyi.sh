#!/bin/bash

# SIYI Camera Launch Script
# Convenience script to launch the SIYI A8 camera system

echo "=========================================="
echo "Starting SIYI A8 Camera System..."
echo "=========================================="

# Navigate to workspace
cd /home/astra-dev/astra/ros2_ws

# Source the workspace
source /home/astra-dev/astra/ros2_ws/install/setup.bash

# Launch the camera node
ros2 launch video_cam siyi_camera2.launch.py
