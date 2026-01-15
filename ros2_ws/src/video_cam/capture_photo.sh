#!/bin/bash

# SIYI Camera Photo Capture Script
# Triggers a 4K photo capture from the SIYI A8 camera

echo "=========================================="
echo "Triggering SIYI A8 Photo Capture..."
echo "=========================================="

# Source the workspace
source /home/astra-dev/astra/ros2_ws/install/setup.bash

# Trigger photo capture
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

echo ""
echo "Photo capture triggered!"
echo "Check the camera node output for status"
