#!/usr/bin/env bash
# =============================================================================
#  team_overlay.sh  --  run ANOTHER COPY of the team's Jetson code with the simulator's launch helpers
#
#  PURPOSE: demo.sh --team-repo DIR (profile key run.team_repo) runs a teammate's fork, another branch or this
#  year's code instead of the simulator's own copy. It goes into DIR and starts DIR's ./launcher.sh sim, exactly
#  like the real Jetson does. That launcher only sources DIR/ros2_ws/install, which knows the TEAM packages but
#  usually not the simulator's sim_bringup (team_stack.launch.py lives there). So demo.sh sets
#      TRACKER_CMD="bash <this file> --sim-ws <simulator ros2_ws> --team-ws DIR/ros2_ws -- ros2 launch sim_bringup ..."
#  and this script, started by DIR's launcher.sh, makes BOTH workspaces visible in the right order and then runs
#  the command:
#      1. the simulator's workspace first (only for sim_bringup / sim_plugins: team_stack.launch.py, sim_paths);
#      2. the team checkout's workspace LAST, so its packages (main, detection, video_cam, mapping ...) WIN over
#         the simulator's copies of the same packages (in ROS 2 the workspace sourced last comes first).
#  It uses each workspace's local_setup.bash (that workspace alone, without re-sourcing /opt/ros/humble), so the
#  order above is exactly the order of the search paths. Nothing is copied or changed in either checkout.
#
#  HOW TO RUN (demo.sh does it for you; by hand only to debug):
#      bash team_overlay.sh --sim-ws ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws \
#                           --team-ws ~/astra_fork/ros2_ws -- ros2 launch sim_bringup team_stack.launch.py
#      bash team_overlay.sh --sim-ws A --team-ws B --show     # only print which copy of each package wins
#  HOW DO I KNOW IT WORKED: it prints "[team_overlay] main comes from <team-ws>/install/main" before the
#  command starts (team_stack.log in the run folder).
#  EXIT CODES: the command's own exit code; 2 = bad usage or a workspace that is not built (message says the fix).
# =============================================================================
set -o pipefail

PROG="[team_overlay]"
SIM_WS=""; TEAM_WS=""; SHOW=0
die() { echo "$PROG ERROR: $*" >&2; exit 2; }

while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) sed -n '2,/^# =====/p' "$0" | sed '1d;$d;s/^# \{0,1\}//'; exit 0 ;;
    --sim-ws) shift; SIM_WS="${1:-}" ;;
    --team-ws) shift; TEAM_WS="${1:-}" ;;
    --show) SHOW=1 ;;
    --) shift; break ;;
    *) die "unknown option \"$1\". Usage: bash team_overlay.sh --sim-ws DIR --team-ws DIR -- COMMAND ..." ;;
  esac
  shift
done

[ -n "$SIM_WS" ] && [ -n "$TEAM_WS" ] || die "both --sim-ws and --team-ws are needed. Likely cause: started by hand \
without them. Fix: bash team_overlay.sh --sim-ws <simulator ros2_ws> --team-ws <team ros2_ws> -- ros2 launch ..."
for ws in "$SIM_WS" "$TEAM_WS"; do
  [ -f "$ws/install/local_setup.bash" ] || die "$ws/install/local_setup.bash is missing: that workspace is not built. \
Fix: source /opt/ros/humble/setup.bash && cd $ws && colcon build --symlink-install --packages-ignore ultralytics_ros"
done
[ "$SHOW" = 1 ] || [ $# -gt 0 ] || die "no command after --. Fix: add the command, e.g. -- ros2 launch sim_bringup team_stack.launch.py"

# ROS setup files read unset variables (AMENT_TRACE_SETUP_FILES ...): never source them under set -u.
set +u
# shellcheck disable=SC1091
source "$SIM_WS/install/local_setup.bash"
# shellcheck disable=SC1091
source "$TEAM_WS/install/local_setup.bash"

# Say where the important packages come from, so team_stack.log shows which code really runs.
for pkg in main detection video_cam mapping waypoint_mavros interfaces sim_bringup; do
  where="$(ros2 pkg prefix "$pkg" 2>/dev/null)"
  echo "$PROG $pkg comes from ${where:-(not found)}"
done
case "$(ros2 pkg prefix main 2>/dev/null)" in
  "$TEAM_WS"/*|"$(readlink -f "$TEAM_WS" 2>/dev/null)"/*) ;;
  *) echo "$PROG WARNING: the team package main does not come from $TEAM_WS. Consequence: the simulator's own copy \
of the team code runs, not the checkout you asked for. Fix: build the checkout (cd $TEAM_WS && colcon build --symlink-install)." ;;
esac
ros2 pkg prefix sim_bringup >/dev/null 2>&1 || die "sim_bringup is not found in $SIM_WS. Fix: build the simulator: \
bash simulation/setup/03_build.sh"
[ "$SHOW" = 0 ] || exit 0

exec "$@"
