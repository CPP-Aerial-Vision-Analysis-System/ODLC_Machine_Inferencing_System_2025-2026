#!/bin/bash

###############################################################################
# SIYI Camera System Setup Verification Script
# Checks network connectivity, camera access, ROS2 environment, and directories
###############################################################################

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# Camera configuration
CAMERA_IP="192.168.144.25"
CAMERA_HTTP_PORT="82"
CAMERA_UDP_PORT="37260"
EXPECTED_SUBNET="192.168.144"

# Counters
CHECKS_PASSED=0
CHECKS_FAILED=0

# Helper functions
print_header() {
    echo ""
    echo -e "${BOLD}=======================================================================${NC}"
    echo -e "${BOLD}$1${NC}"
    echo -e "${BOLD}=======================================================================${NC}"
    echo ""
}

print_check() {
    echo -e "${BLUE}[CHECK]${NC} $1"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
    ((CHECKS_PASSED++))
}

print_error() {
    echo -e "${RED}✗${NC} $1"
    ((CHECKS_FAILED++))
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${BLUE}ℹ${NC} $1"
}

###############################################################################
# CHECK 1: Network Interface
###############################################################################
check_network() {
    print_header "NETWORK CONFIGURATION"
    
    print_check "Checking Ethernet interface..."
    
    # Find active Ethernet interface
    ETH_INTERFACE=$(ip -o link show | grep -E 'eth|enp' | grep 'state UP' | awk '{print $2}' | sed 's/:$//' | head -1)
    
    if [ -z "$ETH_INTERFACE" ]; then
        print_error "No active Ethernet interface found"
        print_info "Available interfaces:"
        ip -o link show | awk '{print "  - " $2}' | sed 's/:$//'
        return 1
    fi
    
    print_success "Active interface: $ETH_INTERFACE"
    
    # Check IP address
    print_check "Checking IP address on $ETH_INTERFACE..."
    IP_ADDR=$(ip addr show $ETH_INTERFACE | grep 'inet ' | awk '{print $2}' | cut -d/ -f1)
    
    if [ -z "$IP_ADDR" ]; then
        print_error "No IP address assigned to $ETH_INTERFACE"
        print_info "Configure with: sudo nmtui"
        return 1
    fi
    
    print_success "IP Address: $IP_ADDR"
    
    # Check subnet
    print_check "Verifying subnet..."
    if [[ $IP_ADDR == ${EXPECTED_SUBNET}.* ]]; then
        print_success "Correct subnet: $EXPECTED_SUBNET.x"
    else
        print_error "Wrong subnet. Expected $EXPECTED_SUBNET.x, got $IP_ADDR"
        print_info "Camera is at $CAMERA_IP - you must be on the same subnet"
        return 1
    fi
    
    return 0
}

###############################################################################
# CHECK 2: Camera Connectivity
###############################################################################
check_camera_ping() {
    print_header "CAMERA CONNECTIVITY"
    
    print_check "Pinging camera at $CAMERA_IP..."
    
    if ping -c 3 -W 2 $CAMERA_IP > /dev/null 2>&1; then
        print_success "Camera is reachable via ping"
        
        # Show latency
        LATENCY=$(ping -c 3 -W 2 $CAMERA_IP | tail -1 | awk '{print $4}' | cut -d '/' -f 2)
        print_info "Average latency: ${LATENCY}ms"
    else
        print_error "Cannot ping camera at $CAMERA_IP"
        print_info "Troubleshooting steps:"
        print_info "  1. Check Ethernet cable is connected"
        print_info "  2. Verify camera is powered on"
        print_info "  3. Check IP configuration: ip addr"
        return 1
    fi
    
    return 0
}

###############################################################################
# CHECK 3: Camera HTTP API
###############################################################################
check_camera_api() {
    print_header "CAMERA HTTP API"
    
    print_check "Testing camera HTTP API at $CAMERA_IP:$CAMERA_HTTP_PORT..."
    
    API_URL="http://${CAMERA_IP}:${CAMERA_HTTP_PORT}/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0"
    
    if ! command -v curl &> /dev/null; then
        print_warning "curl not found, skipping API test"
        print_info "Install with: sudo apt install curl"
        return 0
    fi
    
    # Test API with timeout
    RESPONSE=$(curl -s --max-time 5 "$API_URL" 2>&1)
    CURL_EXIT=$?
    
    if [ $CURL_EXIT -eq 0 ]; then
        # Check if response contains "success":true
        if echo "$RESPONSE" | grep -q '"success":true'; then
            print_success "Camera HTTP API is accessible"
            
            # Parse and show directory count
            if command -v jq &> /dev/null; then
                DIR_COUNT=$(echo "$RESPONSE" | jq -r '.data.directories | length' 2>/dev/null)
                if [ ! -z "$DIR_COUNT" ] && [ "$DIR_COUNT" -ge 0 ]; then
                    print_info "Found $DIR_COUNT directories on SD card"
                fi
            fi
        else
            print_error "API returned error response"
            print_info "Response: $RESPONSE"
            return 1
        fi
    else
        print_error "Cannot connect to camera HTTP API"
        print_info "Error: $RESPONSE"
        print_info "Verify camera is on and SD card is inserted"
        return 1
    fi
    
    return 0
}

###############################################################################
# CHECK 4: SD Card
###############################################################################
check_sd_card() {
    print_header "CAMERA SD CARD"
    
    print_check "Checking SD card status..."
    
    if ! command -v curl &> /dev/null; then
        print_warning "curl not found, skipping SD card check"
        return 0
    fi
    
    API_URL="http://${CAMERA_IP}:${CAMERA_HTTP_PORT}/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0"
    RESPONSE=$(curl -s --max-time 5 "$API_URL" 2>&1)
    
    if echo "$RESPONSE" | grep -q '"success":true'; then
        if command -v jq &> /dev/null; then
            # Check if directories exist
            DIR_COUNT=$(echo "$RESPONSE" | jq -r '.data.directories | length' 2>/dev/null)
            
            if [ ! -z "$DIR_COUNT" ] && [ "$DIR_COUNT" -gt 0 ]; then
                print_success "SD card is present and accessible"
                
                # Show directory names
                echo "$RESPONSE" | jq -r '.data.directories[].path' 2>/dev/null | while read dir; do
                    print_info "  Directory: $dir"
                done
            else
                print_warning "SD card may be empty or not formatted"
                print_info "Format SD card in camera menu if needed"
            fi
        else
            print_success "SD card appears accessible (install jq for details)"
        fi
    else
        print_error "Cannot access SD card"
        print_info "Ensure SD card is inserted in camera"
        return 1
    fi
    
    return 0
}

###############################################################################
# CHECK 5: ROS2 Environment
###############################################################################
check_ros2() {
    print_header "ROS2 ENVIRONMENT"
    
    print_check "Checking ROS2 installation..."
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 environment not sourced"
        print_info "Source ROS2: source /opt/ros/humble/setup.bash"
        return 1
    fi
    
    print_success "ROS2 distro: $ROS_DISTRO"
    
    # Check workspace
    print_check "Checking workspace..."
    
    if [ -z "$AMENT_PREFIX_PATH" ]; then
        print_warning "Workspace not sourced"
        print_info "Source workspace: source install/setup.bash"
    else
        print_success "Workspace sourced"
    fi
    
    return 0
}

###############################################################################
# CHECK 6: Package Installation
###############################################################################
check_package() {
    print_header "VIDEO_CAM PACKAGE"
    
    print_check "Checking if video_cam package is built..."
    
    # Find workspace root
    if [ -d "$PWD/install/video_cam" ]; then
        INSTALL_DIR="$PWD/install/video_cam"
    elif [ -d "$PWD/../install/video_cam" ]; then
        INSTALL_DIR="$PWD/../install/video_cam"
    elif [ -d "$PWD/../../install/video_cam" ]; then
        INSTALL_DIR="$PWD/../../install/video_cam"
    else
        print_error "video_cam package not found"
        print_info "Build with: colcon build --packages-select video_cam"
        return 1
    fi
    
    print_success "Package installed: $INSTALL_DIR"
    
    # Check executables
    print_check "Checking executables..."
    
    EXECUTABLES=("image_pub_siyi2" "siyi2")
    for exec in "${EXECUTABLES[@]}"; do
        if [ -f "$INSTALL_DIR/lib/video_cam/$exec" ]; then
            print_success "  $exec: found"
        else
            print_error "  $exec: not found"
            print_info "Rebuild package: colcon build --packages-select video_cam"
        fi
    done
    
    return 0
}

###############################################################################
# CHECK 7: Python Dependencies
###############################################################################
check_python_deps() {
    print_header "PYTHON DEPENDENCIES"
    
    print_check "Checking Python packages..."
    
    REQUIRED_PACKAGES=("requests" "cv2" "numpy")
    PACKAGE_NAMES=("requests" "opencv-python" "numpy")
    
    for i in "${!REQUIRED_PACKAGES[@]}"; do
        PACKAGE="${REQUIRED_PACKAGES[$i]}"
        PACKAGE_NAME="${PACKAGE_NAMES[$i]}"
        
        if python3 -c "import $PACKAGE" 2>/dev/null; then
            print_success "  $PACKAGE_NAME: installed"
        else
            print_error "  $PACKAGE_NAME: not found"
            print_info "Install with: pip3 install $PACKAGE_NAME"
        fi
    done
    
    return 0
}

###############################################################################
# CHECK 8: Local Directories
###############################################################################
check_directories() {
    print_header "LOCAL STORAGE DIRECTORIES"
    
    print_check "Checking/creating storage directories..."
    
    # Try to find workspace
    WS_DIR=""
    if [ -d "$PWD/src/video_cam" ]; then
        WS_DIR="$PWD"
    elif [ -d "$PWD/../src/video_cam" ]; then
        WS_DIR="$(cd $PWD/.. && pwd)"
    elif [ -d "$PWD/../../src/video_cam" ]; then
        WS_DIR="$(cd $PWD/../.. && pwd)"
    fi
    
    if [ -z "$WS_DIR" ]; then
        print_warning "Could not determine workspace directory"
        print_info "Run this script from ros2_ws directory"
        return 0
    fi
    
    print_info "Workspace: $WS_DIR"
    
    # Create directories
    DIRS=(
        "$WS_DIR/video_cam_data/camera_feed"
        "$WS_DIR/video_cam_data/mapping_photos"
        "$WS_DIR/video_cam_data/downloaded_from_sd"
    )
    
    for dir in "${DIRS[@]}"; do
        if [ -d "$dir" ]; then
            print_success "  $(basename $(dirname $dir))/$(basename $dir): exists"
        else
            mkdir -p "$dir" 2>/dev/null
            if [ $? -eq 0 ]; then
                print_success "  $(basename $(dirname $dir))/$(basename $dir): created"
            else
                print_error "  $(basename $(dirname $dir))/$(basename $dir): failed to create"
            fi
        fi
    done
    
    return 0
}

###############################################################################
# MAIN
###############################################################################
main() {
    print_header "SIYI CAMERA SYSTEM SETUP VERIFICATION"
    echo -e "${BLUE}This script will verify your system is ready for SIYI camera operation${NC}"
    echo ""
    
    # Run all checks
    check_network
    check_camera_ping
    check_camera_api
    check_sd_card
    check_ros2
    check_package
    check_python_deps
    check_directories
    
    # Print summary
    print_header "VERIFICATION SUMMARY"
    
    echo -e "Total checks passed: ${GREEN}${CHECKS_PASSED}${NC}"
    echo -e "Total checks failed: ${RED}${CHECKS_FAILED}${NC}"
    echo ""
    
    if [ $CHECKS_FAILED -eq 0 ]; then
        echo -e "${GREEN}${BOLD}✓ ALL CHECKS PASSED - SYSTEM READY!${NC}"
        echo ""
        echo -e "${BLUE}Next steps:${NC}"
        echo "  1. Launch camera: ros2 launch video_cam siyi_camera2.launch.py"
        echo "  2. Trigger photo: ros2 topic pub --once /camera/trigger std_msgs/msg/Bool \"data: true\""
        echo ""
        exit 0
    else
        echo -e "${YELLOW}${BOLD}⚠ SOME CHECKS FAILED${NC}"
        echo ""
        echo -e "${BLUE}Please address the errors above before proceeding.${NC}"
        echo "Refer to SIYI_CAMERA_COMPLETE_GUIDE.md for troubleshooting."
        echo ""
        exit 1
    fi
}

# Run main
main
