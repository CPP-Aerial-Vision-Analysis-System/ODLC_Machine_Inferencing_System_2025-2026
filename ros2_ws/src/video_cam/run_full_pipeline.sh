#!/bin/bash
# Quick launcher for SIYI Full Cycle Image Processing Pipeline
# Usage: ./run_full_pipeline.sh [test|launch|trigger|status|help]

ROS_WS="$HOME/astra/ros2_ws"
CAMERA_IP="192.168.144.25"
CAMERA_PORT=82

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

function print_banner() {
    echo -e "${BLUE}"
    echo "╔═══════════════════════════════════════════════════════════╗"
    echo "║     SIYI A8 Mini - Full Cycle Image Processing           ║"
    echo "║     Camera → Download → Storage → ROS → Detection        ║"
    echo "╚═══════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

function test_connection() {
    echo -e "${YELLOW}[TEST] Checking camera connection...${NC}"
    
    # Test ping
    if ping -c 2 -W 2 $CAMERA_IP &> /dev/null; then
        echo -e "${GREEN}✓ Camera is reachable at $CAMERA_IP${NC}"
    else
        echo -e "${RED}✗ Cannot ping camera at $CAMERA_IP${NC}"
        echo "  Check: 1) Ethernet cable connected"
        echo "         2) Camera is powered on"
        echo "         3) Jetson IP is 192.168.144.100"
        return 1
    fi
    
    # Test HTTP API
    if curl -s --connect-timeout 3 "http://$CAMERA_IP:$CAMERA_PORT/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0" | grep -q '"success":true'; then
        echo -e "${GREEN}✓ Camera API is responding${NC}"
    else
        echo -e "${RED}✗ Camera API not responding${NC}"
        echo "  Check: Camera web interface should be accessible"
        return 1
    fi
    
    # Test ROS workspace
    if [ -d "$ROS_WS/install/video_cam" ]; then
        echo -e "${GREEN}✓ ROS workspace is built${NC}"
    else
        echo -e "${YELLOW}⚠ ROS workspace not built. Running build...${NC}"
        cd "$ROS_WS"
        colcon build --packages-select video_cam
        if [ $? -eq 0 ]; then
            echo -e "${GREEN}✓ Build successful${NC}"
        else
            echo -e "${RED}✗ Build failed${NC}"
            return 1
        fi
    fi
    
    echo -e "${GREEN}[TEST] All checks passed! System is ready.${NC}"
    return 0
}

function launch_pipeline() {
    echo -e "${YELLOW}[LAUNCH] Starting SIYI camera node...${NC}"
    echo ""
    echo "This will start the full pipeline:"
    echo "  • Camera interface (UDP + HTTP)"
    echo "  • Storage manager"
    echo "  • Pipeline orchestrator"
    echo "  • ROS 2 publishers/subscribers"
    echo ""
    echo "Press Ctrl+C to stop"
    echo ""
    
    cd "$ROS_WS"
    source install/setup.bash
    
    # Run the refactored node (latest version)
    ros2 run video_cam siyi_unified_pipeline_refactored
}

function trigger_capture() {
    echo -e "${YELLOW}[TRIGGER] Sending capture command...${NC}"
    
    cd "$ROS_WS"
    source install/setup.bash
    
    ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
    
    echo ""
    echo -e "${GREEN}✓ Capture triggered!${NC}"
    echo ""
    echo "Watch Terminal 1 for pipeline progress:"
    echo "  Phase 1: Capture command"
    echo "  Phase 2: SD card indexing"
    echo "  Phase 3: Download & save"
    echo "  Phase 4: Publish to /image_raw"
    echo ""
    echo "Expected duration: 3-5 seconds"
}

function check_status() {
    echo -e "${YELLOW}[STATUS] Checking system status...${NC}"
    echo ""
    
    cd "$ROS_WS"
    source install/setup.bash
    
    # Check if node is running
    if ros2 node list 2>/dev/null | grep -q "siyi"; then
        echo -e "${GREEN}✓ SIYI node is running${NC}"
        ros2 node list | grep siyi
    else
        echo -e "${RED}✗ SIYI node is NOT running${NC}"
    fi
    
    echo ""
    
    # Check topics
    echo "Active camera topics:"
    ros2 topic list 2>/dev/null | grep -E "(camera|image)" || echo "  No camera topics found"
    
    echo ""
    
    # Check latest images
    echo "Latest captured images:"
    if [ -d "$ROS_WS/video_cam_data/mapping_photos" ]; then
        ls -lth "$ROS_WS/video_cam_data/mapping_photos" | head -5 || echo "  No images yet"
    else
        echo "  Directory not created yet"
    fi
}

function show_help() {
    print_banner
    echo "Usage: $0 [COMMAND]"
    echo ""
    echo "Commands:"
    echo "  test      Test camera connection and system readiness"
    echo "  launch    Launch the full pipeline (use in Terminal 1)"
    echo "  trigger   Trigger a single image capture (use in Terminal 2)"
    echo "  status    Check system status and recent images"
    echo "  help      Show this help message"
    echo ""
    echo "Example Workflow:"
    echo "  Terminal 1: ./run_full_pipeline.sh launch"
    echo "  Terminal 2: ./run_full_pipeline.sh trigger"
    echo ""
    echo "For detailed documentation, see:"
    echo "  • HOW_TO_RUN_FULL_PIPELINE.md (complete guide)"
    echo "  • QUICK_START.md (quick reference)"
    echo ""
}

# Main script logic
case "$1" in
    test)
        print_banner
        test_connection
        ;;
    launch)
        print_banner
        launch_pipeline
        ;;
    trigger)
        trigger_capture
        ;;
    status)
        check_status
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        show_help
        exit 1
        ;;
esac
