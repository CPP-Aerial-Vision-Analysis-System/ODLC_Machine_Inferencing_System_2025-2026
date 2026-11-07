#!/usr/bin/env bash
# =============================================================================
#  astra.sh  --  THE FRONT DOOR: one command for everything a teammate does with the simulator
#
#  PURPOSE: you only have to remember ONE script. Each word after "astra.sh" runs the script that does that job
#  (this file never does the work itself; it passes every other word on unchanged, so all options of the real
#  script work here too, and "astra.sh WORD --help" shows that script's own help).
#
#  HOW TO RUN (inside Ubuntu 22.04 / WSL, from the repo folder):   bash simulation/scripts/astra.sh WORD [options]
#
#   Run the simulator
#    demo [WORLD] [options]     the full automatic search demo: start, upload, arm, fly, score, close  (demo.sh)
#    manual [WORLD] [options]   Mission Planner mode: start everything, then YOU load, arm and fly; stays open
#                               (= demo.sh WORLD --manual)
#    keep [WORLD] [options]     automatic demo that stays open after the landing (= demo.sh WORLD --keep)
#    sim [WORLD] [options]      only the simulator, without the team's flight code (start_sim.sh)
#    stop [options]             stop the simulator and everything it started (stop_sim.sh)
#    over [options]             START OVER: fly again without closing the windows (start_over.sh)
#   Missions
#    mission check FILE         explain a mission file and check the team code can fly it      (mission.sh)
#    mission load FILE|WORLD    check + upload it to the running simulator (+ MAVROS re-read)
#    mission fly [FILE|WORLD]   (load FILE first) then arm and start AUTO
#    mission make WORLD         plan a new survey mission for a world;  mission list = the files you can load
#   While it runs
#    photo [options]            take ONE photo with the simulated camera (take_photo.sh)
#    status [options]           what is running, ports, camera, autopilot, ROS nodes (status.sh)
#    logs [options]             the run folders and their logs (logs.sh)
#    mp start|stop|status       open / close Mission Planner on Windows (mission_planner.sh)
#   Set up and check
#    build [options]            copy the repo to Linux and build it; run after every change (setup/03_build.sh)
#    check-ros                  is ROS 2 ready in this terminal? (setup/check_ros.sh)
#    jetson WORD [options]      the commands for the REAL drone's Jetson Orin Nano (simulation/jetson/jetson.sh).
#                               The Jetson is "an address this PC can SSH to": the USB-C cable address by default,
#                               or a name / IP on Ethernet, Wi-Fi or a LAN (no USB cable needed); see its --help.
#                               The team uses that board: run nothing on it without the owner's OK
#    help [WORD]                this list, or the help of one command
#
#  Examples:
#    bash simulation/scripts/astra.sh demo                       # runway world, about 8 minutes, closes by itself
#    bash simulation/scripts/astra.sh demo suas_field_real --detector truth
#    bash simulation/scripts/astra.sh demo runway --profile truth_detector
#    bash simulation/scripts/astra.sh manual                     # then in a second terminal:
#    bash simulation/scripts/astra.sh mission fly runway --watch
#    bash simulation/scripts/astra.sh mission check "C:\Users\me\Documents\Mission Planner\field.waypoints"
#    bash simulation/scripts/astra.sh stop
#  Every command is listed with copy-paste examples in docs/simulation/COMMANDS.md; every error message and its
#  fix in docs/simulation/ERRORS.md.
#  Exit code: the exit code of the script it ran; 2 = unknown word; 1 = that script is missing in this copy.
# =============================================================================
ASTRA_PROG=astra
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
SIM_DIR="$(cd "$ASTRA_SCRIPTS_DIR/.." && pwd)"

# word -> script (relative to simulation/) and the words it adds in front of yours
script_for() {
  case "$1" in
    demo)            echo "scripts/demo.sh" ;;
    manual)          echo "scripts/demo.sh" ;;
    keep)            echo "scripts/demo.sh" ;;
    sim|start)       echo "scripts/start_sim.sh" ;;
    stop)            echo "scripts/stop_sim.sh" ;;
    over|start-over) echo "scripts/start_over.sh" ;;
    mission|missions) echo "scripts/mission.sh" ;;
    fly)             echo "scripts/mission.sh" ;;
    photo)           echo "scripts/take_photo.sh" ;;
    status)          echo "scripts/status.sh" ;;
    logs|log)        echo "scripts/logs.sh" ;;
    mp|mission-planner) echo "scripts/mission_planner.sh" ;;
    build|rebuild)   echo "setup/03_build.sh" ;;
    check-ros)       echo "setup/check_ros.sh" ;;
    jetson)          echo "jetson/jetson.sh" ;;
    *)               return 1 ;;
  esac
}

# Which stream / step makes a script that may not be in an older copy yet (for the "missing" message).
missing_hint() {
  case "$1" in
    scripts/logs.sh) echo "logs.sh comes with the 'runs and logs' work; until it is in your copy read the newest run folder: ls -1dt ~/.astra_sim/runs/* | head -1" ;;
    jetson/jetson.sh) echo "jetson.sh comes with the Jetson work (simulation/jetson/; the plan is docs/jetson/HIL_DESIGN.md): update your checkout when it is merged.
    Until then do NOT start anything on the Jetson by hand: the team uses the board, and anything we run there needs the
    owner's OK and its own ROS_DOMAIN_ID (so our topics never mix with the team's). With jetson.sh the owner logs in
    himself (jetson.sh login opens a window for the password), then: jetson.sh check --host HOST" ;;
    *) echo "update your checkout (git pull) and run  bash \"$SIM_DIR/setup/03_build.sh\"" ;;
  esac
}

WORD="${1:-help}"
[ $# -gt 0 ] && shift

COMMANDS="demo, manual, keep, sim, stop, over, mission, photo, status, logs, mp, build, check-ros, jetson, help"
if [ "$WORD" = help ] || [ "$WORD" = -h ] || [ "$WORD" = --help ]; then
  if [ -n "${1:-}" ]; then
    rel="$(script_for "$1")" || die_usage "there is no command \"$1\", so there is no help for it (the commands are:
    $COMMANDS)"
    [ -f "$SIM_DIR/$rel" ] || die "the command \"$1\" needs simulation/$rel, which is not in this copy.
    Likely cause: an older copy of the repo. Fix: $(missing_hint "$rel")"
    exec bash "$SIM_DIR/$rel" --help
  fi
  astra_usage "$0"
  exit 0
fi

rel="$(script_for "$WORD")" || die_usage "unknown command \"$WORD\" (the commands are: $COMMANDS)"
target="$SIM_DIR/$rel"
[ -f "$target" ] || die "the command \"$WORD\" needs simulation/$rel, which is not in this copy.
    Likely cause: an older copy of the repo, or that part has not been added yet.
    Fix: $(missing_hint "$rel")"

# --manual / --keep go FIRST: an option at the end that misses its value (astra.sh manual --windows-gcs) must not
# swallow them (demo.sh takes the world from any position, so "demo.sh --manual runway" is the same run).
case "$WORD" in
  manual) exec bash "$target" --manual "$@" ;;
  keep)   exec bash "$target" --keep "$@" ;;
  fly)    exec bash "$target" fly "$@" ;;
  *)      exec bash "$target" "$@" ;;
esac
