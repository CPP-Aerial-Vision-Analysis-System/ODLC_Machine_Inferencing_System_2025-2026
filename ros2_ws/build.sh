#!/bin/bash
# Build script for ROS2 workspace
# This script sources ROS2 and builds all packages

set -e  # Exit on error

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "ROS2 Workspace Build Script"
echo "=========================================="

# Detect ROS2 distribution
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO="jazzy"
    ROS_SETUP="/opt/ros/jazzy/setup.bash"
    echo "Detected ROS2 distribution: Jazzy"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
    ROS_SETUP="/opt/ros/humble/setup.bash"
    echo "Detected ROS2 distribution: Humble"
elif [ -f "/opt/ros/iron/setup.bash" ]; then
    ROS_DISTRO="iron"
    ROS_SETUP="/opt/ros/iron/setup.bash"
    echo "Detected ROS2 distribution: Iron"
else
    echo "ERROR: Could not find ROS2 installation!"
    echo "Please install ROS2 or source it manually:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Source ROS2
echo "Sourcing ROS2 setup: $ROS_SETUP"
source "$ROS_SETUP"

# Check if colcon is available
if ! command -v colcon &> /dev/null; then
    echo "ERROR: colcon is not installed!"
    echo "Install it with:"
    echo "  sudo apt install python3-colcon-common-extensions"
    exit 1
fi

# Parse command line arguments
PACKAGES_SELECT=""
SYMLINK_INSTALL=""
BUILD_ARGS=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --packages-select)
            PACKAGES_SELECT="--packages-select $2"
            shift 2
            ;;
        --symlink-install)
            SYMLINK_INSTALL="--symlink-install"
            shift
            ;;
        --video-cam-only)
            PACKAGES_SELECT="--packages-select video_cam"
            shift
            ;;
        *)
            BUILD_ARGS="$BUILD_ARGS $1"
            shift
            ;;
    esac
done

# Build command
BUILD_CMD="colcon build $PACKAGES_SELECT $SYMLINK_INSTALL $BUILD_ARGS"

echo "=========================================="
echo "Building ROS2 workspace..."
echo "Command: $BUILD_CMD"
echo "=========================================="

# Run the build
eval $BUILD_CMD

# Check build result
if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "Build completed successfully!"
    echo "=========================================="
    echo ""
    echo "To use the built packages, run:"
    echo "  source install/setup.bash"
    echo ""
    echo "To run the object detection node:"
    echo "  ros2 run video_cam object_detection_sahi"
    echo ""
else
    echo "=========================================="
    echo "Build failed!"
    echo "=========================================="
    exit 1
fi

