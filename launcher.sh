#!/bin/bash

# ─────────────────────────────────────────────
# ASTRA Launcher
# Sources ROS 2 Humble + the local workspace
# before starting MAVROS and the mission stack.
#
# Usage:
#   ./launcher.sh            # hardware (Pixhawk over serial)
#   ./launcher.sh sim        # SITL over UDP, real camera still attached
#
# Overrides (env):
#   FCU_URL=...        MAVROS connection string
#   MAVROS_LAUNCH=...  px4.launch / apm.launch
#   CAMERA_IP=...      SIYI camera address for the reachability check
#   ASTRA_RAW_TIME=1   show raw epoch timestamps
# ─────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/ros2_ws"

# ── Mode: hw (default) or sim ────────────────
MODE="${ASTRA_MODE:-hw}"
case "$1" in
    sim|--sim)  MODE="sim" ;;
    hw|--hw)    MODE="hw" ;;
    "")         ;;
    *)          echo "[ERROR] Unknown argument '$1'. Use 'sim' or 'hw'."; exit 1 ;;
esac

# ── Make ROS log timestamps readable (epoch -> HH:MM:SS) ──
# Set ASTRA_RAW_TIME=1 to disable and see raw epoch timestamps.
if [ -z "$ASTRA_RAW_TIME" ] && [ -x "$SCRIPT_DIR/scripts/humanize.sh" ]; then
    exec > >("$SCRIPT_DIR/scripts/humanize.sh") 2>&1
fi

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
# The flight stack is ArduPilot (Mission Planner / EKF3 / DigiCamCtrl),
# so apm.launch is the correct MAVROS launch file in both modes.
# Set MAVROS_LAUNCH=px4.launch to override.
MAVROS_LAUNCH="${MAVROS_LAUNCH:-apm.launch}"

if [ "$MODE" = "sim" ]; then
    # SITL over UDP. The camera is still the real SIYI on the payload network.
    FCU_URL="${FCU_URL:-udp://:14552@localhost:14552}"
else
    # Serial (Pixhawk via USB): /dev/ttyUSB0:115200 or /dev/ttyACM0:115200
    FCU_URL="${FCU_URL:-/dev/ttyUSB0:115200}"
fi

MAVROS_CMD="ros2 launch mavros $MAVROS_LAUNCH fcu_url:=$FCU_URL"

# Use the project's own tracker launch (main package).
# It starts waypoint_manager, siyi, the DigiCamCtrl trigger bridge,
# new_od, gpstest and main_controller.
TRACKER_CMD="ros2 launch main tracker.launch.xml"

CAMERA_IP="${CAMERA_IP:-192.168.144.25}"
MAX_WAIT=30
LOOP_DELAY=1

echo "[INFO] Mode: $MODE"

# ── Shut everything down together on Ctrl-C ──
MAVROS_PID=""
TRACKER_PID=""
shutdown() {
    echo ""
    echo "[INFO] Shutting down..."
    [ -n "$TRACKER_PID" ] && kill "$TRACKER_PID" 2>/dev/null
    [ -n "$MAVROS_PID" ]  && kill "$MAVROS_PID"  2>/dev/null
    wait 2>/dev/null
    echo "[INFO] Stopped."
}
trap shutdown INT TERM

# ── Camera reachability (non-fatal) ──────────
echo "[INFO] Checking SIYI camera at $CAMERA_IP..."
if ping -c 1 -W 1 "$CAMERA_IP" > /dev/null 2>&1; then
    echo "[INFO] Camera reachable."
else
    echo "[WARN] Camera at $CAMERA_IP did not respond to ping."
    echo "       Captures will fail until it is on the network."
fi

# ── Start MAVROS in background ───────────────
echo "[INFO] Launching MAVROS ($MAVROS_LAUNCH, fcu_url=$FCU_URL)..."
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

if ! $HEARTBEAT; then
    echo "[WARN] No heartbeat received after ${MAX_WAIT}s — launching anyway."
    if [ "$MODE" = "sim" ]; then
        echo "       Is SITL running and forwarding to $FCU_URL?"
    else
        echo "       Is the Pixhawk connected on $FCU_URL?"
    fi
    echo "       Without a heartbeat there is no statustext, so DigiCamCtrl"
    echo "       never reaches the trigger bridge and no photos are taken."
fi

# ── Launch mission stack ──────────────────────
echo "[INFO] Launching mission stack..."
$TRACKER_CMD &
TRACKER_PID=$!

echo "[INFO] Running. Ctrl-C to stop."
echo "[INFO] Watch captures with:  ros2 topic echo /camera/status"

# ── Keep script alive until either child exits ─
wait -n "$MAVROS_PID" "$TRACKER_PID"
shutdown
