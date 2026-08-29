#!/bin/bash
# ─────────────────────────────────────────────
# ASTRA status tap
# Sources ROS 2 Humble + the workspace overlay, then streams every
# ground-station message (send_ack) to this terminal.
#
# Local:  ./scripts/watch_status.sh
# Remote: ssh user@jetson '~/Documents/ODLC_Machine_Inferencing_System_2025-2026/scripts/watch_status.sh'
#
# Extra args are passed to watch_status.py (--send-only, --topic, --no-color).
# ─────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

source /opt/ros/humble/setup.bash || { echo "[ERROR] ROS 2 Humble not found."; exit 1; }
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"

exec python3 "$SCRIPT_DIR/watch_status.py" "$@"
