#!/usr/bin/env bash
# =============================================================================
#  container_jetson.sh  --  runs INSIDE the "jetson" container: build, wait for the simulator, ./launcher.sh sim
#
#  PURPOSE: what the "jetson" service of docker/compose.sim.yml does when it starts. This container is the
#  Jetson stand-in: the team's dev image (Ubuntu 22.04 + ROS 2 Humble + MAVROS + the Python ML stack, the
#  same versions as the Jetson) at 192.168.144.100, the real Jetson's address on the drone's network. Steps:
#    1. makes the mounted folders writable for the container user (fresh Docker volumes belong to root);
#    2. builds the repo's ros2_ws in this container's own volumes, when needed (_container_common.sh);
#    3. detector:=real only: makes sure the detector weights yolo26m.pt are there (compose_up.sh copies the
#       native simulator's file; otherwise they are downloaded once, ~40 MB);
#    4. waits until the simulator's MAVLink arrives on UDP 14552 (MAVProxy in the "sim" container sends it
#       here), so MAVROS gets a heartbeat right away instead of launcher.sh's 30 s timeout;
#    5. starts the team's code EXACTLY as on the drone:  cd <repo> && ./launcher.sh sim   (MAVROS with the
#       launcher's own fcu_url udp://:14552@localhost:14552, video_cam talking to the camera at
#       192.168.144.25 = the "sim" container, new_od, the DigiCamCtrl trigger, waypoint_manager,
#       main_controller). launcher.sh is not changed; only its documented TRACKER_CMD setting is used, the
#       same way simulation/scripts/demo.sh does: tracker.launch.xml through sim_bringup's
#       team_stack.launch.py, which only gives new_od a model file that exists off the Jetson.
#  Stop: Ctrl+C in compose_up.sh's window, or compose_down.sh (launcher.sh stops MAVROS and the stack itself).
#
#  Settings (environment variables; docker/compose.sim.yml fills them):
#    ASTRA_DETECTOR      real (default) | truth  (must match the sim container; compose passes the same value)
#    ASTRA_TRACKER_CMD   the mission stack command for launcher.sh ("" = the team_stack.launch.py line above);
#                        e.g. "ros2 launch main tracker.launch.xml" = the team's plain file (Jetson model path)
#    ASTRA_WAIT_FOR_SIM  seconds to wait for the simulator's MAVLink (default 1800: the first start of the
#                        sim container builds ros2_ws and downloads 3D models first; 0 = do not wait)
#    ASTRA_MAVROS_PORT   the UDP port MAVROS listens on (default 14552, launcher.sh's sim default)
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source-path=SCRIPTDIR source=_container_common.sh
source "$HERE/_container_common.sh"

DETECTOR="${ASTRA_DETECTOR:-real}"
WAIT_S="${ASTRA_WAIT_FOR_SIM:-1800}"
PORT="${ASTRA_MAVROS_PORT:-14552}"

say "ASTRA Jetson stand-in container \"jetson\" (address $(my_ip), user $(id -un))"

# ---- 1. folders + 2. build ------------------------------------------------------------------------
fix_ownership "$ASTRA_REPO/ros2_ws/build" "$ASTRA_REPO/ros2_ws/install" "$ASTRA_REPO/ros2_ws/log" \
  "$ASTRA_SIM_HOME" "$ASTRA_SIM_HOME/models" "$HOME/.config"
build_workspace jetson || warn "continuing with what was built (launcher.sh reports missing packages)"
[ -f "$ASTRA_REPO/ros2_ws/install/setup.bash" ] || die "ros2_ws/install is missing: the build above failed (see its log)"

# ---- 3. detector weights --------------------------------------------------------------------------
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$HOME/.config}"
MODEL="${ASTRA_DETECTOR_MODEL:-$ASTRA_SIM_HOME/models/yolo26m.pt}"
export ASTRA_DETECTOR_MODEL="$MODEL"
if [ "$DETECTOR" = real ] && [ ! -s "$MODEL" ]; then
  say "Downloading the detector weights $(basename "$MODEL") (about 40 MB, only the first time)"
  mkdir -p "$(dirname "$MODEL")"
  if (cd "$(dirname "$MODEL")" && timeout 900 python3 -c "from ultralytics import YOLO; YOLO('$(basename "$MODEL")')" >/dev/null 2>&1) \
     && [ -s "$MODEL" ]; then
    ok "saved $MODEL"
  else
    warn "could not get $MODEL: new_od will NOT detect anything (internet?). Use the answer-key detector instead:"
    warn "    bash simulation/docker/compose_up.sh --detector truth"
  fi
fi

# ---- 4. wait for the simulator's MAVLink -----------------------------------------------------------
if [ "$WAIT_S" -gt 0 ] 2>/dev/null; then
  say "Waiting for the simulator's MAVLink on UDP $PORT (up to $((WAIT_S / 60)) min; the sim container builds and boots first)"
  # pymavlink (installed in this image) listens on the port MAVROS will use, waits for one HEARTBEAT, and lets
  # the port go again before launcher.sh starts MAVROS on it.
  if python3 - "$PORT" "$WAIT_S" <<'PY'
import sys, time
from pymavlink import mavutil
port, limit = int(sys.argv[1]), float(sys.argv[2])
conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}')
start = last = time.time()
try:
    while time.time() - start < limit:
        if conn.wait_heartbeat(timeout=5):
            print(f'    MAVLink heartbeat from the simulator after {time.time() - start:.0f} s')
            sys.exit(0)
        if time.time() - last >= 60:
            last = time.time()
            print(f'    still waiting ({time.time() - start:.0f} s): is the "sim" container running? '
                  '(docker logs astra-sim)', flush=True)
    sys.exit(1)
finally:
    conn.close()
PY
  then
    ok "the simulator is talking: starting the team's code"
  else
    warn "no MAVLink from the simulator after $WAIT_S s; starting the team's code anyway (MAVROS keeps listening)"
  fi
fi

# ---- 5. the team's code, exactly as on the drone ----------------------------------------------------
export TRACKER_CMD="${ASTRA_TRACKER_CMD:-ros2 launch sim_bringup team_stack.launch.py detector:=$DETECTOR}"
say "cd $ASTRA_REPO && ./launcher.sh sim"
info "mission stack (TRACKER_CMD): $TRACKER_CMD"
cd "$ASTRA_REPO" || die "cannot enter $ASTRA_REPO"
exec ./launcher.sh sim
