#!/usr/bin/env bash
# =============================================================================
#  start_over.sh  --  START OVER: fly the search mission again, without closing any window
#
#  PURPOSE: after one demo flight the payloads lie on the ground, the team's mission brain (main_controller)
#  has finished its sequence and old photos sit in the team's photo folder, so pressing UPLOAD MISSION + START
#  again is NOT a clean restart. This script puts EVERYTHING back to the start while Gazebo, RViz and the
#  demo panel stay open:
#    1. a flying drone lands first (LAND, or RTL with --land rtl) and disarms (it waits, with a time limit),
#    2. the team flight stack (./launcher.sh sim) stops; the old photos and detections move into the OLD run
#       folder, so the team photo folder is empty again,
#    3. a fresh drone with BOTH payloads stands on the start point; SITL + MAVProxy restart with the same
#       settings (the autopilot boots there, so GPS, EKF and Gazebo agree),
#    4. a NEW run folder (score report, photos, phases); the mission is uploaded; the team stack starts again
#       exactly like demo.sh starts it, and the script waits until it is ready,
#    5. with --fly: arm + AUTO, the drone flies the search again.
#  The work is done by ros2_ws/src/simulation/sim_demo/sim_demo/run_reset.py; the demo panel's START OVER
#  button runs this same script. If NO simulator is running, it starts a fresh one instead
#  (demo.sh --keep with --fly, demo.sh --manual without it).
#
#  HOW TO RUN:   bash start_over.sh                  reset, then wait for you (panel button or Mission Planner)
#                bash start_over.sh --fly            reset and fly the mission again right away
#                bash start_over.sh --fly --follow   ... and show the mission phases here until it lands
#  Options:      --world W              the world you expect. Start over keeps the windows open, so it restarts
#                                       the SAME world; another world needs stop_sim.sh, then demo.sh W
#                --mission FILE         upload another .waypoints mission (default: the one the last run flew)
#                --land land|rtl        how a flying drone comes down first (default land = straight down)
#                --land-timeout S       stop waiting for the landing after S seconds (default 300)
#                --no-team-stack        do not start ./launcher.sh sim again
#                --detector real|truth  detector of the new run (default: the same as before)
#  It takes about 2-3 minutes (mostly the autopilot's normal settling time and the team stack's start).
#  Log: ~/.astra_sim/logs/start_over_<time>.log      New run: ~/.astra_sim/runs/<time>_<world>/
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

FLY=0; FOLLOW=0; WORLD=""; PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --fly) FLY=1; PASS+=(--fly) ;;
    --follow) FOLLOW=1 ;;
    --world) shift; WORLD="${1:?--world needs a world name}"; PASS+=(--world "$WORLD") ;;
    --mission) shift; [ -f "${1:-}" ] || die "mission file not found: ${1:-}"; PASS+=(--mission "$(readlink -f "$1")") ;;
    --land) shift; case "${1:-}" in land|rtl) PASS+=(--land "$1") ;; *) die "--land must be land or rtl" ;; esac ;;
    --land-timeout) shift; case "${1:-}" in ''|*[!0-9]*) die "--land-timeout needs seconds" ;; esac; PASS+=(--land-timeout "$1") ;;
    --no-team-stack) PASS+=(--no-team-stack) ;;
    --detector) shift; case "${1:-}" in real|truth) PASS+=(--detector "$1") ;; *) die "--detector must be real or truth" ;; esac ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done

astra_load_env
astra_require_built
ros2 pkg prefix sim_demo >/dev/null 2>&1 || die "sim_demo is not built (it holds start over). Run setup/03_build.sh."

# No simulator? Then "start over" simply means "start": a fresh demo, kept open.
start_fresh() {
  local w="${WORLD:-runway}"
  say "No simulator is running, so there is nothing to start over: starting a fresh demo ($w) instead"
  if [ "$FLY" = 1 ]; then
    exec bash "$ASTRA_SCRIPTS_DIR/demo.sh" "$w" --keep
  fi
  exec bash "$ASTRA_SCRIPTS_DIR/demo.sh" "$w" --manual
}
SESSION="$(cat "$ASTRA_STATE_DIR/session" 2>/dev/null)"
[ -n "$SESSION" ] || SESSION="$(astra_state_value session "")"
if [ -z "$SESSION" ] || [ -z "$(astra_session_pids "$SESSION")" ]; then
  start_fresh
fi
# Everything start over starts carries the simulator's session tag, so stop_sim.sh stops it later too.
export ASTRA_SIM_SESSION="$SESSION"

LOG="$ASTRA_SIM_HOME/logs/start_over_$(date +%Y%m%d-%H%M%S).log"
mkdir -p "$(dirname "$LOG")"
say "START OVER: the windows stay open; the drone, the autopilot and the team code start fresh"
info "log: $LOG"
python3 -m sim_demo.run_reset reset "${PASS[@]}" 2>&1 | tee -a "$LOG"
RC=${PIPESTATUS[0]}
case "$RC" in
  0) ;;
  3) start_fresh ;;
  *) die "START OVER did not finish (the reason is above; full log: $LOG)" ;;
esac

RUN_DIR="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("run_dir", ""))' \
            "$ASTRA_STATE_DIR/current_run.json" 2>/dev/null)"
if [ "$FLY" = 0 ]; then
  echo
  ok "Ready to fly again. Either press UPLOAD MISSION + START in the demo panel, or use Mission Planner:"
  python3 -m sim_demo.run_reset mp-steps | sed 's/^/    /'
fi
if [ "$FOLLOW" = 1 ] && [ -n "$RUN_DIR" ] && [ -f "$RUN_DIR/phases.log" ]; then
  say "Mission phases (from $RUN_DIR/phases.log); this ends when the drone has landed (Ctrl+C only stops watching)"
  WATCHER="$(pgrep -u "$(id -u)" -f "sim_demo.run_reset watch-run --run-dir $RUN_DIR" | head -1)"
  if [ -n "$WATCHER" ]; then
    tail -n +1 -f --pid="$WATCHER" "$RUN_DIR/phases.log"
  else
    cat "$RUN_DIR/phases.log"
  fi
  [ -s "$RUN_DIR/score_report.md" ] && { say "SCORE REPORT ($RUN_DIR/score_report.md)"; head -40 "$RUN_DIR/score_report.md" | sed 's/^/    /'; }
fi
[ -n "$RUN_DIR" ] && info "everything of the new run goes to $RUN_DIR"
info "to run it again: bash $ASTRA_SCRIPTS_DIR/start_over.sh --fly   (or START OVER in the demo panel)"
