#!/usr/bin/env bash
# =============================================================================
#  mission_planner.sh  --  open, close or check Mission Planner (on Windows) from the Ubuntu terminal
#
#  PURPOSE: the terminal twin of the demo panel's OPEN / CLOSE MISSION PLANNER buttons. Mission Planner is
#  the Windows ground-station program the team flies the real drone with; it can also watch the simulated
#  drone (simulation/README.md, "Watch it in Mission Planner"). Ubuntu runs inside WSL, and WSL's "interop"
#  lets Ubuntu start and stop Windows programs. This script runs the SAME Python helper as the panel,
#  ros2_ws/src/simulation/sim_demo/sim_demo/mission_planner.py (plain Python: no ROS, no build needed):
#    start   opens Mission Planner, unless it is already open. It stays open when this terminal closes.
#    stop    asks Mission Planner to close, like clicking its X; only if it is still open 5 s later is it
#            forced closed. Safe during a flight: the simulated autopilot has no ground-station failsafe.
#    status  says whether Mission Planner is running (exit code 0 = running, 3 = not running,
#            1 = cannot tell, for example outside WSL).
#  It never starts or stops anything of the simulator.
#
#  HOW TO RUN:   bash mission_planner.sh start | stop | status
#  Installed somewhere else?  MISSION_PLANNER_EXE="/mnt/e/Mission Planner/MissionPlanner.exe" bash mission_planner.sh start
#                (a Windows path such as "E:\Mission Planner\MissionPlanner.exe" works too)
#  NEXT STEP:    in Mission Planner set the connection box (top right) to UDP, click CONNECT, keep port 14550
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
CMD="${1:-}"
case "$CMD" in
  start|stop|status) ;;
  -h|--help) astra_usage "$0"; exit 0 ;;
  "") astra_usage "$0"; exit 2 ;;
  *) die "unknown command \"$CMD\" (use start, stop or status; see --help)" ;;
esac

# The helper lives in the sim_demo package of the same repo copy as this script (Windows or Linux copy).
SIM_DEMO_SRC="$ASTRA_SCRIPTS_DIR/../../ros2_ws/src/simulation/sim_demo"
[ -f "$SIM_DEMO_SRC/sim_demo/mission_planner.py" ] ||
  die "Missing $SIM_DEMO_SRC/sim_demo/mission_planner.py (is this a complete copy of the repo?)"

case "$CMD" in
  start)  say "Opening Mission Planner on Windows (this takes a few seconds)" ;;
  stop)   say "Closing Mission Planner on Windows" ;;
  status) say "Is Mission Planner running on Windows?" ;;
esac
# Run inside the package folder, so `python3 -m sim_demo.mission_planner` finds the module without a build
# (PYTHONDONTWRITEBYTECODE: no __pycache__ folders left in the repo copy).
# Capturing its output is safe: Mission Planner itself gets /dev/null as its output (see start() in the
# helper), so this returns as soon as the helper is done, not when Mission Planner closes.
MSG=$(cd "$SIM_DEMO_SRC" && PYTHONDONTWRITEBYTECODE=1 python3 -m sim_demo.mission_planner "$CMD" 2>&1)
RC=$?
case "$RC" in
  0) ok "$MSG" ;;
  3) info "$MSG" ;;              # status: not running (a normal answer, not an error)
  *) warn "$MSG" ;;
esac
exit "$RC"
