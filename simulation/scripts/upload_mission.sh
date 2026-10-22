#!/usr/bin/env bash
# =============================================================================
#  upload_mission.sh  --  upload a world's survey mission to the simulated autopilot
#
#  PURPOSE: does what Mission Planner's "Write WPs" button does at the field. It finds the world's
#  mission file (sim_worlds/missions/<world>_survey.waypoints, via sim_worlds/config/<world>.yaml),
#  checks it has what the team's code needs (TAKEOFF, DO_DIGICAM_CONTROL = start photos, survey
#  waypoints, RETURN_TO_LAUNCH last) and uploads it to ArduCopter SITL over MAVLink with
#  `ros2 run sim_bringup mission_tool upload`, through MAVProxy's ground-station output (udp 14550).
#
#  HOW TO RUN:   bash upload_mission.sh                  -> the runway mission
#                bash upload_mission.sh suas_field        -> the SUAS field mission
#                bash upload_mission.sh /path/my.waypoints
#  Options:      --url URL   another MAVLink connection (default: the running simulator's ground-station port, udp 14550)
#  Note: only one program can listen on udp 14550 at a time; close QGroundControl / Mission Planner first,
#  or give them another port.
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

TARGET="runway"
URL="udpin:127.0.0.1:$(astra_state_value gcs_port 14550)"   # the running simulator's ground-station port
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --url) shift; URL="${1:?--url needs a value}" ;;
    *) TARGET="$1" ;;
  esac
  shift
done
astra_load_env
astra_require_built

if [ -f "$TARGET" ]; then
  MISSION="$(readlink -f "$TARGET")"
else
  MISSION="$(python3 -m sim_bringup.sim_paths world "$TARGET" mission)"
  [ -n "$MISSION" ] || die "No mission file for world \"$TARGET\" (is sim_worlds built? see sim_worlds/config/$TARGET.yaml)"
fi
say "Uploading the mission $MISSION"
ros2 run sim_bringup mission_tool --url "$URL" upload --mission "$MISSION" || die "the upload failed (see the messages above)"
# The team stack (MAVROS -> waypoint_manager) does not notice an upload by itself: tell it.
astra_mavros_repull
