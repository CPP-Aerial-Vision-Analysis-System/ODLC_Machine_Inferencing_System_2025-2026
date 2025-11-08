#!/bin/bash
# Quick run script for SAHI object detection
# This script sources ROS2, builds if needed, and runs the detection node

set -e  # Exit on error

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "SAHI Object Detection - Quick Run"
echo "=========================================="

# Detect ROS2 distribution
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO="jazzy"
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
    ROS_SETUP="/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    ROS_DISTRO="iron"
    ROS_SETUP="/opt/ros/iron/setup.bash"
else
    echo "ERROR: Could not find ROS2 installation!"
    echo "Please install ROS2 or source it manually:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Source ROS2
echo "Sourcing ROS2 setup: $ROS_SETUP"
source "$ROS_SETUP"

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo "Workspace not built yet. Building video_cam package..."
    colcon build --packages-select video_cam --symlink-install
fi

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Parse command line arguments for the node
NODE_ARGS=""

while [[ $# -gt 0 ]]; do
    NODE_ARGS="$NODE_ARGS $1"
    shift
done

echo "=========================================="
echo "Starting SAHI Object Detection Node..."
echo "=========================================="
echo ""

# Run the node
ros2 run video_cam object_detection_sahi $NODE_ARGS

