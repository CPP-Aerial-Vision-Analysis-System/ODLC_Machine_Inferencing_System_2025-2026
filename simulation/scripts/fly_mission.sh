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
#                bash fly_mission.sh suas_field_real --watch
#                bash fly_mission.sh /path/my.waypoints
#  Options:      --url URL (default: the running simulator's ground-station port, udp 14550), --no-upload (fly what is already on the autopilot),
#                --watch (print messages until landed and disarmed)
#  Recommended:  bash mission.sh fly [FILE]   checks the file first (and refuses a mission the team code cannot fly).
#  Exit codes:   0 = the drone took off (with --watch: and landed), 1 = failed (reason printed), 2 = command typed wrong.
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

TARGET="runway"
URL="udpin:127.0.0.1:$(astra_state_value gcs_port 14550)"   # the running simulator's ground-station port
URL_TYPED=""      # "typed" after --url: then no "is a simulator running here?" check (it may run elsewhere)
UPLOAD=1
WATCH=0
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --url) shift; [ -n "${1:-}" ] || die_usage "--url needs a value, e.g. udpin:127.0.0.1:14550"; URL="$1"; URL_TYPED=typed ;;
    --no-upload) UPLOAD=0 ;;
    --watch) WATCH=1 ;;
    -*) die_usage "unknown option \"$1\"" ;;
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
    [ -n "$MISSION" ] || die "No mission for \"$TARGET\": it is neither a file nor a world with a survey mission.
    Likely cause: a typo in the file or world name, or sim_worlds is not built.
    Fix: see the files and worlds you can use with  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" list"
  fi
  astra_require_sim_link "$URL" "$URL_TYPED"
  say "Flying the mission $MISSION"
  ros2 run sim_bringup mission_tool --url "$URL" upload --mission "$MISSION" || die "the upload failed (mission_tool's reason is above).
    Likely cause: the autopilot did not answer (the simulator is still starting, or it stopped) or it rejected an item.
    Fix: wait until  bash \"$ASTRA_SCRIPTS_DIR/status.sh\"  shows the autopilot, then try again"
  astra_mavros_repull || true
else
  astra_require_sim_link "$URL" "$URL_TYPED"
  say "Flying the mission that is already on the autopilot"
fi
# (no --mission: arm and start what is on the autopilot now; the upload above already happened)
ros2 run sim_bringup mission_tool --url "$URL" fly || die "the mission did not start (mission_tool's reason is above).
    Likely cause: the autopilot is still settling (GPS / EKF / pre-arm checks, normal for ~30-60 s after it starts),
    it refused to arm (the 'PreArm:' text above says why), or the mission has no TAKEOFF.
    Fix: wait a minute and run this again; check the mission with  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" check FILE"
ok "The drone is flying the mission."
if [ "$WATCH" = 1 ]; then
  ros2 run sim_bringup mission_tool --url "$URL" watch
fi
