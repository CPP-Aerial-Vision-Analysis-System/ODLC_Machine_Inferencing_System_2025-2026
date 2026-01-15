#!/bin/bash

##############################################################################
# SIYI Unified Pipeline Quick Start
#
# This script provides easy launch options for the SIYI A8 Mini unified
# pipeline node.
#
# Usage:
#   ./siyi_unified_pipeline_start.sh [option]
#
# Options:
#   launch    - Launch using launch file (recommended)
#   run       - Run node directly
#   test      - Test camera connectivity
#   status    - Check pipeline status
#   trigger   - Manually trigger a photo capture
#   help      - Show this help message
##############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
CAMERA_IP="192.168.144.25"
RTSP_URL="rtsp://${CAMERA_IP}:8554/main.264"
HTTP_API="http://${CAMERA_IP}:82/cgi-bin/media.cgi/api/v1"

# Function: Print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function: Print header
print_header() {
    echo ""
    echo "======================================================================"
    echo "  SIYI A8 Mini Unified Pipeline"
    echo "======================================================================"
    echo ""
}

# Function: Test camera connectivity
test_camera() {
    print_header
    print_info "Testing SIYI camera connectivity..."
    echo ""
    
    # Test 1: Ping camera
    print_info "Test 1: Ping camera at ${CAMERA_IP}"
    if ping -c 1 -W 2 ${CAMERA_IP} &> /dev/null; then
        print_success "Camera is reachable"
    else
        print_error "Cannot reach camera at ${CAMERA_IP}"
        return 1
    fi
    echo ""
    
    # Test 2: HTTP API
    print_info "Test 2: HTTP API at port 82"
    if curl -s --connect-timeout 2 "${HTTP_API}/getdirectories?media_type=0" &> /dev/null; then
        print_success "HTTP API is accessible"
    else
        print_error "Cannot access HTTP API"
        return 1
    fi
    echo ""
    
    # Test 3: RTSP stream (optional, takes longer)
    print_info "Test 3: RTSP stream at port 8554"
    if timeout 2 ffprobe "${RTSP_URL}" &> /dev/null; then
        print_success "RTSP stream is accessible"
    else
        print_warning "RTSP stream test skipped or failed (install ffmpeg for test)"
    fi
    echo ""
    
    print_success "Camera connectivity tests completed!"
    echo ""
    
    # Show camera info
    print_info "Querying camera SD card info..."
    response=$(curl -s "${HTTP_API}/getdirectories?media_type=0")
    echo "${response}" | python3 -m json.tool 2>/dev/null || echo "${response}"
    echo ""
}

# Function: Launch pipeline using launch file
launch_pipeline() {
    print_header
    print_info "Launching SIYI Unified Pipeline..."
    echo ""
    
    # Source ROS2 environment
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    elif [ -f "../install/setup.bash" ]; then
        source ../install/setup.bash
    else
        print_error "Cannot find ROS2 workspace setup.bash"
        print_info "Please run from ros2_ws directory or build the workspace first"
        exit 1
    fi
    
    print_info "Starting pipeline with launch file..."
    ros2 launch video_cam siyi_unified_pipeline.launch.py
}

# Function: Run pipeline node directly
run_pipeline() {
    print_header
    print_info "Running SIYI Unified Pipeline node..."
    echo ""
    
    # Source ROS2 environment
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    elif [ -f "../install/setup.bash" ]; then
        source ../install/setup.bash
    else
        print_error "Cannot find ROS2 workspace setup.bash"
        exit 1
    fi
    
    print_info "Starting pipeline node..."
    ros2 run video_cam siyi_unified_pipeline
}

# Function: Check pipeline status
check_status() {
    print_header
    print_info "Checking SIYI Unified Pipeline status..."
    echo ""
    
    # Check if node is running
    print_info "Active ROS2 nodes:"
    if ros2 node list 2>/dev/null | grep -q "siyi_unified_pipeline"; then
        print_success "Pipeline node is running!"
        ros2 node list | grep siyi
    else
        print_warning "Pipeline node is not running"
    fi
    echo ""
    
    # Check topics
    print_info "Pipeline topics:"
    ros2 topic list 2>/dev/null | grep -E "(camera|image)" || print_warning "No pipeline topics found"
    echo ""
    
    # Check camera status topic
    print_info "Latest camera status:"
    timeout 2 ros2 topic echo --once /camera/status 2>/dev/null || print_warning "No status messages yet"
    echo ""
}

# Function: Trigger photo capture
trigger_capture() {
    print_header
    print_info "Triggering photo capture..."
    echo ""
    
    # Check if node is running
    if ! ros2 node list 2>/dev/null | grep -q "siyi_unified_pipeline"; then
        print_error "Pipeline node is not running!"
        print_info "Start the pipeline first with: $0 launch"
        exit 1
    fi
    
    print_info "Sending capture trigger..."
    ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
    
    print_success "Capture trigger sent!"
    print_info "Check /camera/status topic for capture progress"
    echo ""
    
    # Listen for status updates
    print_info "Listening for status updates (Ctrl+C to stop)..."
    ros2 topic echo /camera/status
}

# Function: Show help
show_help() {
    print_header
    echo "Usage: $0 [option]"
    echo ""
    echo "Options:"
    echo "  launch    - Launch pipeline using launch file (recommended)"
    echo "  run       - Run pipeline node directly"
    echo "  test      - Test camera connectivity and API"
    echo "  status    - Check if pipeline is running"
    echo "  trigger   - Manually trigger a photo capture"
    echo "  help      - Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 test                    # Test camera before starting"
    echo "  $0 launch                  # Start the pipeline"
    echo "  $0 trigger                 # Capture a photo"
    echo "  $0 status                  # Check pipeline status"
    echo ""
    echo "Topics:"
    echo "  Subscribed:"
    echo "    /camera/trigger           - Trigger capture (std_msgs/Bool)"
    echo "    /camera/set_resolution    - Set resolution (std_msgs/String)"
    echo ""
    echo "  Published:"
    echo "    /image_raw                - Live stream and captured images"
    echo "    /camera/status            - Camera status updates"
    echo "    /mavros/statustext/send   - MAVROS status messages"
    echo ""
}

# Main script logic
case "$1" in
    launch)
        launch_pipeline
        ;;
    run)
        run_pipeline
        ;;
    test)
        test_camera
        ;;
    status)
        check_status
        ;;
    trigger)
        trigger_capture
        ;;
    help|--help|-h|"")
        show_help
        ;;
    *)
        print_error "Unknown option: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
