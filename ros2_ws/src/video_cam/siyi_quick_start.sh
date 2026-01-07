#!/bin/bash
# SIYI A8 mini Camera - Quick Start Script
# This script helps set up and test the SIYI camera integration

echo "=== SIYI A8 mini Camera Quick Start ==="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

echo "1. Checking prerequisites..."

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 not sourced!${NC}"
    echo "  Please run: source /opt/ros/humble/setup.bash"
    exit 1
else
    echo -e "${GREEN}✓ ROS2 $ROS_DISTRO detected${NC}"
fi

# Check if Python packages are installed
if python3 -c "import requests" 2>/dev/null; then
    echo -e "${GREEN}✓ requests package installed${NC}"
else
    echo -e "${YELLOW}⚠ requests package not found${NC}"
    echo "  Installing: pip install requests"
    pip install requests
fi

if python3 -c "import cv2" 2>/dev/null; then
    echo -e "${GREEN}✓ opencv-python installed${NC}"
else
    echo -e "${YELLOW}⚠ opencv-python not found${NC}"
    echo "  Installing: pip install opencv-python"
    pip install opencv-python
fi

echo ""
echo "2. Checking network connectivity to camera..."

# Check if camera is reachable
CAMERA_IP="192.168.144.25"
if ping -c 1 -W 2 $CAMERA_IP >/dev/null 2>&1; then
    echo -e "${GREEN}✓ Camera reachable at $CAMERA_IP${NC}"
    
    # Try to connect to media server
    if curl -s --max-time 2 "http://$CAMERA_IP:82/cgi-bin/media.cgi?cmd=getdirectories" >/dev/null 2>&1; then
        echo -e "${GREEN}✓ Camera media server responding${NC}"
    else
        echo -e "${YELLOW}⚠ Camera reachable but media server not responding${NC}"
        echo "  Check if camera is fully booted and SD card is inserted"
    fi
else
    echo -e "${YELLOW}⚠ Camera not reachable at $CAMERA_IP${NC}"
    echo "  This is normal if camera is not connected"
    echo "  Node will run in simulation mode"
fi

echo ""
echo "3. Network configuration tips:"
echo "  Current IP addresses:"
ip -brief addr show | grep -E "eth|enp"
echo ""
echo "  To set static IP on Jetson (if needed):"
echo "  sudo ip addr add 192.168.144.100/24 dev eth0"
echo "  sudo ip link set eth0 up"

echo ""
echo "4. Build and run instructions:"
echo ""
echo "  # Build the package"
echo "  cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws"
echo "  colcon build --packages-select video_cam"
echo ""
echo "  # Source the workspace"
echo "  source install/setup.bash"
echo ""
echo "  # Run the camera node"
echo "  ros2 run video_cam image_pub"
echo ""

echo "5. Photo storage locations:"
WS_DIR="/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws"
echo "  Regular photos: $WS_DIR/video_cam/camera_feed/"
echo "  Mapping photos: $WS_DIR/video_cam/mapping_photos/"
echo ""

echo -e "${GREEN}=== Setup check complete ===${NC}"
echo ""
echo "For detailed documentation, see:"
echo "  ros2_ws/src/video_cam/README_SIYI_CAMERA.md"
