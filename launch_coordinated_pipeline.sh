#!/bin/bash

# Coordinated Image Capture Pipeline Launcher
# This script helps launch all required nodes for the coordinated pipeline

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$SCRIPT_DIR/ros2_ws"

echo "================================================================"
echo "  COORDINATED IMAGE CAPTURE PIPELINE"
echo "================================================================"
echo ""
echo "This will launch the coordinated pipeline where main_controller"
echo "orchestrates: waypoint → capture → wait → detect → act"
echo ""
echo "Required nodes:"
echo "  1. MAVROS (connect to flight controller)"
echo "  2. SIYI Camera (siyi_unified_pipeline_new)"
echo "  3. Image Capture Coordinator (image_pub_final)"
echo "  4. Detection Node (new_od_aro)"
echo "  5. Main Controller (main_controller_aro)"
echo ""
echo "================================================================"
echo ""

# Check if workspace is built
if [ ! -d "$WS_DIR/install" ]; then
    echo "❌ ERROR: Workspace not built!"
    echo "Please run: cd $WS_DIR && colcon build"
    exit 1
fi

# Source workspace
cd "$WS_DIR"
source install/setup.bash

echo "✅ Workspace sourced"
echo ""
echo "Available launch options:"
echo ""
echo "  1) Launch ALL nodes in separate terminals (requires tmux)"
echo "  2) Launch individual nodes manually"
echo "  3) Show manual launch commands"
echo "  4) Exit"
echo ""
read -p "Select option (1-4): " OPTION

case $OPTION in
    1)
        # Check if tmux is available
        if ! command -v tmux &> /dev/null; then
            echo "❌ tmux not found. Please install: sudo apt install tmux"
            exit 1
        fi
        
        echo ""
        echo "Launching nodes in tmux session 'odlc_pipeline'..."
        echo ""
        
        # Create new tmux session
        tmux new-session -d -s odlc_pipeline -n mavros
        
        # Window 0: MAVROS
        tmux send-keys -t odlc_pipeline:mavros "cd $WS_DIR" C-m
        tmux send-keys -t odlc_pipeline:mavros "source install/setup.bash" C-m
        tmux send-keys -t odlc_pipeline:mavros "echo 'Starting MAVROS...'" C-m
        tmux send-keys -t odlc_pipeline:mavros "ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557" C-m
        
        # Window 1: SIYI Camera
        tmux new-window -t odlc_pipeline -n siyi
        tmux send-keys -t odlc_pipeline:siyi "cd $WS_DIR" C-m
        tmux send-keys -t odlc_pipeline:siyi "source install/setup.bash" C-m
        tmux send-keys -t odlc_pipeline:siyi "sleep 3" C-m
        tmux send-keys -t odlc_pipeline:siyi "echo 'Starting SIYI Camera...'" C-m
        tmux send-keys -t odlc_pipeline:siyi "ros2 run video_cam siyi_unified_pipeline_new" C-m
        
        # Window 2: Image Capture Coordinator
        tmux new-window -t odlc_pipeline -n capture
        tmux send-keys -t odlc_pipeline:capture "cd $WS_DIR" C-m
        tmux send-keys -t odlc_pipeline:capture "source install/setup.bash" C-m
        tmux send-keys -t odlc_pipeline:capture "sleep 5" C-m
        tmux send-keys -t odlc_pipeline:capture "echo 'Starting Image Capture Coordinator...'" C-m
        tmux send-keys -t odlc_pipeline:capture "ros2 run video_cam image_pub_final --ros-args -p coordinate_with_controller:=true -p capture_delay:=1.0" C-m
        
        # Window 3: Detection
        tmux new-window -t odlc_pipeline -n detection
        tmux send-keys -t odlc_pipeline:detection "cd $WS_DIR" C-m
        tmux send-keys -t odlc_pipeline:detection "source install/setup.bash" C-m
        tmux send-keys -t odlc_pipeline:detection "sleep 7" C-m
        tmux send-keys -t odlc_pipeline:detection "echo 'Starting Detection Node...'" C-m
        tmux send-keys -t odlc_pipeline:detection "ros2 run detection new_od_aro" C-m
        
        # Window 4: Main Controller
        tmux new-window -t odlc_pipeline -n controller
        tmux send-keys -t odlc_pipeline:controller "cd $WS_DIR" C-m
        tmux send-keys -t odlc_pipeline:controller "source install/setup.bash" C-m
        tmux send-keys -t odlc_pipeline:controller "sleep 10" C-m
        tmux send-keys -t odlc_pipeline:controller "echo 'Starting Main Controller...'" C-m
        tmux send-keys -t odlc_pipeline:controller "ros2 run main main_controller_aro" C-m
        
        # Window 5: Monitoring
        tmux new-window -t odlc_pipeline -n monitor
        tmux send-keys -t odlc_pipeline:monitor "cd $WS_DIR" C-m
        tmux send-keys -t odlc_pipeline:monitor "source install/setup.bash" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo ''" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '================================================================'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '  MONITORING WINDOW'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '================================================================'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo ''" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo 'Useful monitoring commands:'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '  ros2 topic echo /controller/state'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '  ros2 topic echo /waypoint_capture/status'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '  ros2 topic echo /camera/trigger'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '  ros2 topic list'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo '  ros2 node list'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo ''" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo 'Switch windows: Ctrl+b then window number (0-5)'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo 'Exit tmux: Ctrl+b then type :kill-session'" C-m
        tmux send-keys -t odlc_pipeline:monitor "echo ''" C-m
        
        # Attach to session
        echo "✅ All nodes launched in tmux session 'odlc_pipeline'"
        echo ""
        echo "Windows:"
        echo "  0: MAVROS"
        echo "  1: SIYI Camera"
        echo "  2: Image Capture Coordinator"
        echo "  3: Detection"
        echo "  4: Main Controller"
        echo "  5: Monitor (this window)"
        echo ""
        echo "Controls:"
        echo "  Switch windows: Ctrl+b then number (0-5)"
        echo "  Exit tmux: Ctrl+b then type :kill-session"
        echo ""
        echo "Attaching to session..."
        sleep 2
        tmux attach -t odlc_pipeline
        ;;
        
    2)
        echo ""
        echo "Launch each node in a separate terminal:"
        echo ""
        echo "Terminal 1 - MAVROS:"
        echo "  cd $WS_DIR"
        echo "  source install/setup.bash"
        echo "  ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557"
        echo ""
        echo "Terminal 2 - SIYI Camera:"
        echo "  cd $WS_DIR"
        echo "  source install/setup.bash"
        echo "  ros2 run video_cam siyi_unified_pipeline_new"
        echo ""
        echo "Terminal 3 - Image Capture Coordinator:"
        echo "  cd $WS_DIR"
        echo "  source install/setup.bash"
        echo "  ros2 run video_cam image_pub_final --ros-args -p coordinate_with_controller:=true"
        echo ""
        echo "Terminal 4 - Detection:"
        echo "  cd $WS_DIR"
        echo "  source install/setup.bash"
        echo "  ros2 run detection new_od_aro"
        echo ""
        echo "Terminal 5 - Main Controller:"
        echo "  cd $WS_DIR"
        echo "  source install/setup.bash"
        echo "  ros2 run main main_controller_aro"
        echo ""
        ;;
        
    3)
        echo ""
        echo "Manual launch commands (copy-paste ready):"
        echo ""
        echo "# Terminal 1 - MAVROS"
        echo "cd $WS_DIR && source install/setup.bash && ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14557"
        echo ""
        echo "# Terminal 2 - SIYI Camera"
        echo "cd $WS_DIR && source install/setup.bash && ros2 run video_cam siyi_unified_pipeline_new"
        echo ""
        echo "# Terminal 3 - Image Capture Coordinator"
        echo "cd $WS_DIR && source install/setup.bash && ros2 run video_cam image_pub_final --ros-args -p coordinate_with_controller:=true"
        echo ""
        echo "# Terminal 4 - Detection"
        echo "cd $WS_DIR && source install/setup.bash && ros2 run detection new_od_aro"
        echo ""
        echo "# Terminal 5 - Main Controller"
        echo "cd $WS_DIR && source install/setup.bash && ros2 run main main_controller_aro"
        echo ""
        ;;
        
    4)
        echo "Exiting..."
        exit 0
        ;;
        
    *)
        echo "Invalid option"
        exit 1
        ;;
esac

echo ""
echo "================================================================"
echo "For more information, see:"
echo "  $WS_DIR/COORDINATED_PIPELINE_GUIDE.md"
echo "================================================================"
