#!/usr/bin/env bash
# =============================================================================
#  mp_actions.sh  --  the SIMULATOR buttons of the ASTRA tab in Mission Planner
#
#  PURPOSE: Mission Planner (on Windows) is the ground station. Its ASTRA tab (the plugin
#  simulation/windows/mission_planner_plugin/astra_gcs.cs) has four buttons that only make sense in the
#  simulator; they press the same ROS "buttons" the old ASTRA demo panel pressed. The plugin runs
#        wsl.exe -d Ubuntu-22.04 -- bash -lc "bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/mp_actions.sh ACTION"
#  and shows the LAST line this script prints in its event log.
#
#    take_photo     publish true on /camera/trigger          -> the team's camera node (video_cam) takes a photo
#    look_down      "gimbal_set_angles 0,-90" on /camera/command -> the gimbal points the camera straight down
#    look_forward   "gimbal_set_angles 0,0"   on /camera/command -> the gimbal points the camera forward
#    start_over     simulation/scripts/start_over.sh         -> put the whole run back to the start (see that script)
#
#  The ROS messages wait (up to 20 s) until somebody listens, so a press is never lost while the team stack starts.
#  Nothing here touches the autopilot: flying (arm, AUTO, RTL) stays with Mission Planner's own commands.
#
#  HOW TO RUN by hand:   bash simulation/scripts/mp_actions.sh take_photo
#  Exit code: 0 done, 1 ROS 2 could not be loaded or nobody listened, 2 unknown action.
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ACTION="${1:-}"
[ $# -gt 0 ] && shift

case "$ACTION" in
  take_photo|look_down|look_forward|start_over) ;;
  -h|--help|"") sed -n '2,24p' "$0"; exit 0 ;;
  *) echo "unknown action \"$ACTION\" (use take_photo, look_down, look_forward or start_over)"; exit 2 ;;
esac

if [ "$ACTION" = start_over ]; then
  exec bash "$HERE/start_over.sh" "$@"
fi

# ROS 2 Humble + this workspace, quietly (the same setup every simulator script uses).
# shellcheck source=../setup/sim_env.sh
source "$HERE/../setup/sim_env.sh" >/dev/null 2>&1 || true
command -v ros2 >/dev/null 2>&1 || { echo "cannot load ROS 2 (simulation/setup/sim_env.sh): is the workspace built?"; exit 1; }

publish() {   # publish <topic> <type> <yaml> <what it did>
  if timeout 25 ros2 topic pub -w 1 --once "$1" "$2" "$3" >/dev/null 2>&1; then
    echo "$4"
  else
    echo "nobody listened on $1 within 20 s (is the team stack running?)"
    return 1
  fi
}

case "$ACTION" in
  take_photo)   publish /camera/trigger std_msgs/msg/Bool "{data: true}" "photo requested on /camera/trigger" ;;
  look_down)    publish /camera/command std_msgs/msg/String "{data: 'gimbal_set_angles 0,-90'}" "camera pointed down (gimbal_set_angles 0,-90)" ;;
  look_forward) publish /camera/command std_msgs/msg/String "{data: 'gimbal_set_angles 0,0'}" "camera pointed forward (gimbal_set_angles 0,0)" ;;
esac
