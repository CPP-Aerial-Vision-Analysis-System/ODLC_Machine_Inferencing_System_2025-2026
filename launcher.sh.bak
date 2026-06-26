#!/bin/bash

# ─────────────────────────────────────────────
# ASTRA Launcher
# Sources ROS 2 Humble + the local workspace
# before starting MAVROS and the mission stack.
# ─────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros2_ws"

# ── Source ROS 2 base ────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "[ERROR] /opt/ros/humble/setup.bash not found. Is ROS 2 Humble installed?"
    exit 1
fi

# ── Source the local workspace overlay ───────
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "[ERROR] Workspace overlay not found at $WS_DIR/install/setup.bash"
    echo "        Did you run 'colcon build' inside $WS_DIR?"
    exit 1
fi

# ── Configs ──────────────────────────────────
# Adjust fcu_url to match your hardware:
#   Serial (Pixhawk via USB):  /dev/ttyUSB0:115200  or  /dev/ttyACM0:115200
#   UDP (SITL / companion):    udp://:14552@localhost:14552
FCU_URL="${FCU_URL:-/dev/ttyUSB0:115200}"

MAVROS_CMD="ros2 launch mavros px4.launch fcu_url:=$FCU_URL"
# MAVROS_CMD="ros2 launch mavros apm.launch fcu_url:=udp://:14552@localhost:14552"

# Use the project's own tracker launch (main package)
TRACKER_CMD="ros2 launch main tracker.launch.xml"

MAX_WAIT=30
LOOP_DELAY=1

# ── Start MAVROS in background ───────────────
echo "[INFO] Launching MAVROS (fcu_url=$FCU_URL)..."
$MAVROS_CMD &
MAVROS_PID=$!

# ── Wait for /mavros/state heartbeat ─────────
echo "[INFO] Waiting for heartbeat on /mavros/state (up to ${MAX_WAIT}s)..."
HEARTBEAT=false
for (( i=0; i<MAX_WAIT; i++ )); do
    if ros2 topic echo --once /mavros/state > /dev/null 2>&1; then
        echo "[INFO] Heartbeat detected after ${i}s."
        HEARTBEAT=true
        break
    fi
    sleep $LOOP_DELAY
done

# ── Launch mission stack ──────────────────────
if $HEARTBEAT; then
    echo "[INFO] Launching mission stack..."
    $TRACKER_CMD
else
    echo "[WARN] No heartbeat received after ${MAX_WAIT}s — launching anyway."
    $TRACKER_CMD
fi

# ── Keep script alive until MAVROS exits ─────
wait $MAVROS_PID
