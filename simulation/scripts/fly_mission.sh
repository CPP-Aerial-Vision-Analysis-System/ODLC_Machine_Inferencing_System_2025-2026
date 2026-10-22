#!/usr/bin/env bash
# =============================================================================
#  fly_mission.sh  --  upload the survey mission, wait for the autopilot, arm and start it
#
#  PURPOSE: what the ground crew does at the field, in one command (pymavlink, through MAVProxy's
#  ground-station output udp 14550):
#    1. upload the world's survey mission (like upload_mission.sh) and make a running MAVROS re-read it
#       (the team's waypoint_manager otherwise keeps the old mission and main_controller never holds),
#    2. wait until SITL has a GPS fix, a healthy EKF and passes its pre-arm checks (~30 s after boot;
#       "Arm: Gyros inconsistent" until then is the normal settling window),
#    3. switch to GUIDED and arm (retrying while the checks settle),
#    4. MAV_CMD_MISSION_START: AUTO mode, the drone takes off and flies the mission,
#    5. confirm the take-off; with --watch, keep printing the autopilot's messages until it lands.
#  The team's main_controller takes over from there (target waypoints, drops, RTL) when ./launcher.sh sim
#  runs. demo.sh does all of this for you, plus the team stack and the score.
#
#  HOW TO RUN:   bash fly_mission.sh                    -> runway mission
#                bash fly_mission.sh suas_field --watch
#                bash fly_mission.sh /path/my.waypoints
#  Options:      --url URL (default: the running simulator's ground-station port, udp 14550), --no-upload (fly what is already on the autopilot),
#                --watch (print messages until landed and disarmed)
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

TARGET="runway"
URL="udpin:127.0.0.1:$(astra_state_value gcs_port 14550)"   # the running simulator's ground-station port
UPLOAD=1
WATCH=0
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --url) shift; URL="${1:?--url needs a value}" ;;
    --no-upload) UPLOAD=0 ;;
    --watch) WATCH=1 ;;
    *) TARGET="$1" ;;
  esac
  shift
done
astra_load_env
astra_require_built

MISSION=""
if [ "$UPLOAD" = 1 ]; then
  if [ -f "$TARGET" ]; then
    MISSION="$(readlink -f "$TARGET")"
  else
    MISSION="$(python3 -m sim_bringup.sim_paths world "$TARGET" mission)"
    [ -n "$MISSION" ] || die "No mission file for world \"$TARGET\" (is sim_worlds built?)"
  fi
  say "Flying the mission $MISSION"
  ros2 run sim_bringup mission_tool --url "$URL" upload --mission "$MISSION" || die "the upload failed (see the messages above)"
  astra_mavros_repull || true
else
  say "Flying the mission that is already on the autopilot"
fi
# (no --mission: arm and start what is on the autopilot now; the upload above already happened)
ros2 run sim_bringup mission_tool --url "$URL" fly || die "the mission did not start (see the messages above)"
ok "The drone is flying the mission."
if [ "$WATCH" = 1 ]; then
  ros2 run sim_bringup mission_tool --url "$URL" watch
fi
