#!/bin/bash
# SIYI Camera - Complete Usage Example
# This demonstrates all the new features

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║     SIYI Camera Control System - Usage Demo                   ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash

echo "📸 Starting camera node in background..."
ros2 run video_cam image_pub > /tmp/camera_node.log 2>&1 &
CAMERA_PID=$!
echo "   Node PID: $CAMERA_PID"
sleep 6

echo ""
echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 1: Manual Photo Capture"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 run video_cam camera_control take_photo"
ros2 run video_cam camera_control take_photo
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 2: Zoom Control - Zoom In"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 run video_cam camera_control zoom_in 5"
ros2 run video_cam camera_control zoom_in 5
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 3: Set Specific Zoom Level"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 run video_cam camera_control set_zoom 15"
ros2 run video_cam camera_control set_zoom 15
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 4: Zoom Out"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 run video_cam camera_control zoom_out 3"
ros2 run video_cam camera_control zoom_out 3
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 5: Check Camera Status"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 run video_cam camera_control get_status"
ros2 run video_cam camera_control get_status
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 6: Using ROS2 Service Directly"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 service call /camera/control interfaces/srv/CameraCommand \\"
echo "  \"{command: 'zoom_in', parameter: '2'}\""
timeout 3 ros2 service call /camera/control interfaces/srv/CameraCommand \
  "{command: 'zoom_in', parameter: '2'}"
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "EXAMPLE 7: Monitoring Zoom Level"
echo "══════════════════════════════════════════════════════════════════"
echo ""
echo "$ ros2 topic echo /camera/zoom_level --once"
timeout 2 ros2 topic echo /camera/zoom_level --once || echo "Topic data: (waiting for zoom change)"
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "Available Services"
echo "══════════════════════════════════════════════════════════════════"
echo ""
ros2 service list | grep camera
echo ""

echo "══════════════════════════════════════════════════════════════════"
echo "Available Topics"
echo "══════════════════════════════════════════════════════════════════"
echo ""
ros2 topic list | grep -E "(image|camera|zoom)"
echo ""

echo "Stopping camera node..."
kill $CAMERA_PID 2>/dev/null
wait $CAMERA_PID 2>/dev/null
echo "✓ Demo complete!"
echo ""

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║                    Summary of Commands                        ║"
echo "╠════════════════════════════════════════════════════════════════╣"
echo "║ • take_photo    - Capture photo immediately                   ║"
echo "║ • zoom_in N     - Zoom in by N steps                          ║"
echo "║ • zoom_out N    - Zoom out by N steps                         ║"
echo "║ • set_zoom N    - Set zoom to level N (1-30)                  ║"
echo "║ • get_status    - Get camera status                           ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""
echo "For more information, see:"
echo "  - CAMERA_CONTROL_GUIDE.md"
echo "  - README_SIYI_CAMERA.md"
