#!/usr/bin/env bash
# =============================================================================
#  container_sim.sh  --  runs INSIDE the "sim" container: build, start the simulator, fly the demo
#
#  PURPOSE: what the "sim" service of docker/compose.sim.yml does when it starts (you normally never call it
#  yourself; simulation/docker/compose_up.sh starts the container, and the container runs this):
#    1. makes the mounted folders writable for the container user (fresh Docker volumes belong to root);
#    2. builds the repo's ros2_ws in this container's own volumes, when needed (_container_common.sh);
#    3. downloads the Gazebo Fuel 3D models the worlds use, once;
#    4. sends the autopilot's MAVLink to the Jetson stand-in at udp:192.168.144.100:14552, exactly where the
#       team's ./launcher.sh sim listens there (fcu_url udp://:14552@...). How: with sim.launch.py's
#       mavros_out argument when it has one (simulation plug-and-play settings), otherwise with MAVProxy's own
#       start-up file ~/.mavinit.scr ("output add udp:HOST:PORT"), which needs no change to the launch file;
#    5. starts the simulator with simulation/scripts/start_sim.sh --background: Gazebo, ArduCopter SITL,
#       MAVProxy, the SIYI camera emulator (this container's address IS the camera's, 192.168.144.25, so no
#       address alias is added), RViz, the demo panel and sim_demo's score recorder. Its log goes to
#       ~/.astra_sim/logs/sim_<time>.log (= <data folder>/logs on the host, normally ~/.astra_docker/logs);
#    6. ASTRA_DEMO=1 (default): the demo "conductor", i.e. the steps of simulation/scripts/demo.sh that are not
#       the Jetson's job: wait for the autopilot, upload the world's survey mission, wait until the team's code
#       in the "jetson" container is ready (sim_bringup stack_check, over ROS 2 across the two containers),
#       point the camera down, arm + AUTO, print the mission phases, wait for the landing, make sure the score
#       report is written and copy the photos into the run folder. Then (ASTRA_KEEP=0) the windows stay open
#       ASTRA_LINGER_S seconds and everything stops; compose then stops the "jetson" container too.
#       ASTRA_DEMO=0: nothing is flown; the simulator log is shown and you fly with the demo panel's buttons or
#       Mission Planner on Windows (MAVLink copy, see ASTRA_WINDOWS_GCS).
#  Stop: Ctrl+C in compose_up.sh's window, or compose_down.sh (this script passes the stop on, writes the score).
#
#  Settings (environment variables; docker/compose.sim.yml fills them from compose_up.sh's options):
#    ASTRA_SIM_WORLD      runway (default) | suas_field | a world file path inside the container
#    ASTRA_CAMERA         zr10 (default, 2K photos) | a8mini (4K photos)
#    ASTRA_DETECTOR       real (default: the team's new_od in the jetson container) | truth (answer-key stand-in)
#    ASTRA_SIM_OPTIONS    extra start_sim.sh options, e.g. "--headless"
#    ASTRA_SIM_ARGS       extra launch settings "name:=value ...", passed to sim.launch.py
#    ASTRA_JETSON_MAVLINK where the Jetson's MAVROS listens (default 192.168.144.100:14552)
#    ASTRA_WINDOWS_GCS    HOST:PORT of Mission Planner / QGroundControl on Windows ("" = none; compose_up.sh
#                         fills in the Windows host in WSL's NAT mode)
#    ASTRA_DEMO           1 (default) = fly the demo by itself | 0 = only start the simulator
#    ASTRA_KEEP           0 (default) | 1 = keep running after the landing
#    ASTRA_TIMEOUT_MIN    give the demo up after this many minutes (default 30)
#    ASTRA_LINGER_S       seconds the windows stay open after the landing (default 120; 0 when headless)
#    ASTRA_STACK_WAIT     seconds to wait for the team's code in the jetson container (default 1200: it may
#                         still be building ros2_ws the first time)
#    ASTRA_SCORE          1 (default) = record the run and write score_report.md; 0 = off
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source-path=SCRIPTDIR source=_container_common.sh
source "$HERE/_container_common.sh"

WORLD="${ASTRA_SIM_WORLD:-runway}"
CAMERA="${ASTRA_CAMERA:-zr10}"
DETECTOR="${ASTRA_DETECTOR:-real}"
JETSON="${ASTRA_JETSON_MAVLINK:-192.168.144.100:14552}"
WIN_GCS="${ASTRA_WINDOWS_GCS:-}"
SCORE="${ASTRA_SCORE:-1}"
DEMO="${ASTRA_DEMO:-1}"
KEEP="${ASTRA_KEEP:-0}"
TIMEOUT_MIN="${ASTRA_TIMEOUT_MIN:-30}"
LINGER_S="${ASTRA_LINGER_S:-120}"
STACK_WAIT="${ASTRA_STACK_WAIT:-1200}"
GCS_PORT=14550                       # MAVProxy's ground-station output inside this container (mission_tool)
read -r -a OPTIONS <<< "${ASTRA_SIM_OPTIONS:-}"
read -r -a EXTRA <<< "${ASTRA_SIM_ARGS:-}"

say "ASTRA simulator container \"sim\" (address $(my_ip), user $(id -un))"
info "world $WORLD | camera $CAMERA | detector $DETECTOR | MAVLink to the Jetson: $JETSON | demo flight: $([ "$DEMO" = 1 ] && echo yes || echo no)"

# No desktop to draw on (no X11 socket) -> no windows, instead of Gazebo/RViz failing to open.
HEADLESS=0
for o in "${OPTIONS[@]}"; do [ "$o" = --headless ] && HEADLESS=1; done
if [ "$HEADLESS" = 0 ] && [ ! -S "/tmp/.X11-unix/X${DISPLAY#*:}" ] && [ ! -S "${XDG_RUNTIME_DIR:-/nonexistent}/${WAYLAND_DISPLAY:-wayland-0}" ]; then
  warn "no desktop found (no /tmp/.X11-unix/X${DISPLAY#*:}): running without windows (--headless)"
  OPTIONS+=(--headless); HEADLESS=1
fi
[ "$HEADLESS" = 1 ] && LINGER_S=0

# ---- 1. folders + 2. build ------------------------------------------------------------------------
fix_ownership "$ASTRA_REPO/ros2_ws/build" "$ASTRA_REPO/ros2_ws/install" "$ASTRA_REPO/ros2_ws/log" \
  "$ASTRA_SIM_HOME" "$HOME/.gz" "$HOME/.gz/fuel"
build_workspace sim || warn "continuing with what was built (the simulator replaces or skips missing parts)"

# The simulation scripts' own settings: ROS 2 + this workspace + ArduPilot paths + WSLg graphics.
# shellcheck disable=SC1091
source "$ASTRA_REPO/simulation/setup/sim_env.sh"
[ "${ASTRA_WS_BUILT:-no}" = yes ] || die "ros2_ws/install is missing: the build above failed (see its log)"

# ---- 3. Fuel models -------------------------------------------------------------------------------
fetch_fuel_models

# ---- 4. MAVLink to the Jetson stand-in -------------------------------------------------------------
ARGS=("camera_model:=$CAMERA" "detector:=$DETECTOR")
LAUNCH_ARGS="$(timeout 120 ros2 launch sim_bringup sim.launch.py --show-args 2>/dev/null)"
has_launch_arg() { grep -q "^ *'$1':" <<< "$LAUNCH_ARGS"; }
rm -f "$HOME/.mavinit.scr"
if has_launch_arg mavros_out; then
  # sim.launch.py's own setting: MAVProxy's MAVROS output goes to the Jetson instead of 127.0.0.1:14552.
  ARGS+=("mavros_out:=udp:$JETSON")
  info "MAVProxy's MAVROS output -> udp:$JETSON (launch argument mavros_out)"
else
  # MAVProxy runs the commands in ~/.mavinit.scr every time it starts (MAVProxy 1.8.74 mavproxy.py,
  # run_startup_scripts); "output add" is the same command you would type in its console.
  printf '# Written by simulation/docker/container_sim.sh: MAVLink copy for the Jetson stand-in.\noutput add udp:%s\n' \
    "$JETSON" > "$HOME/.mavinit.scr"
  info "MAVProxy's extra output -> udp:$JETSON (via ~/.mavinit.scr; sim.launch.py has no mavros_out argument yet)"
fi
# Mission Planner / QGroundControl on Windows. sim.launch.py's automatic choice would pick this container's
# gateway (the WSL Ubuntu, not Windows), so it is either given explicitly or switched off.
if [ -n "$WIN_GCS" ]; then
  ARGS+=("windows_gcs_addr:=$WIN_GCS")
  info "Windows ground station copy -> udp:$WIN_GCS (Mission Planner: connection type UDP, port ${WIN_GCS##*:})"
else
  ARGS+=("windows_gcs:=false")
fi

# ---- 5. start the simulator ----------------------------------------------------------------------
RUN_DIR="$ASTRA_SIM_HOME/runs/$(date +%Y%m%d-%H%M%S)_$(basename "${WORLD%.*}")"
mkdir -p "$RUN_DIR" "$ASTRA_SIM_HOME/state"
printf '%s\n' "$RUN_DIR" > "$ASTRA_SIM_HOME/state/docker_run_dir"
if [ "$SCORE" = 1 ]; then
  ARGS+=("score:=true" "demo_run_dir:=$RUN_DIR")
fi
ARGS+=("${EXTRA[@]}")
info "run folder (score report, phases, photos): $RUN_DIR"

say "Starting the simulator: start_sim.sh $WORLD --no-camera-ip --background ${OPTIONS[*]} ${ARGS[*]}"
bash "$ASTRA_REPO/simulation/scripts/start_sim.sh" "$WORLD" --no-camera-ip --background "${OPTIONS[@]}" "${ARGS[@]}" \
  || die "start_sim.sh could not start the simulator (see above)"
# start_sim.sh wrote the launch's process group (PGID=) and its log file (LOG=) into state/start_sim.env.
SIM_ENV="$ASTRA_SIM_HOME/state/start_sim.env"
SIM_PGID="$(awk -F= '$1 == "PGID" {v = $2} END {print v}' "$SIM_ENV" 2>/dev/null)"
SIM_LOG="$(awk -F= '$1 == "LOG" {v = $2} END {print v}' "$SIM_ENV" 2>/dev/null)"
[ -n "$SIM_PGID" ] || die "start_sim.sh did not report the simulator's process (see $SIM_ENV)"
cp "$SIM_ENV" "$RUN_DIR/start_sim.env" 2>/dev/null

sim_alive() { kill -0 -- "-$SIM_PGID" 2>/dev/null; }

# stop_simulator : Ctrl+C to the whole launch, then harder, then stop_sim.sh sweeps up what is left.
stop_simulator() {
  if sim_alive; then
    info "stopping the simulator (a few seconds) ..."
    kill -INT -- "-$SIM_PGID" 2>/dev/null
    for _ in $(seq 1 25); do sim_alive || break; sleep 1; done
    sim_alive && { kill -TERM -- "-$SIM_PGID" 2>/dev/null; sleep 3; kill -KILL -- "-$SIM_PGID" 2>/dev/null; }
  fi
  bash "$ASTRA_REPO/simulation/scripts/stop_sim.sh" --quiet >/dev/null 2>&1 || true
}

# run_wait CMD... : run a command and wait for it, in a way a stop request (docker stop / Ctrl+C) interrupts
# immediately (bash only runs a trap after a FOREGROUND command ends, but `wait` returns at once).
run_wait() { "$@" & local pid=$!; wait "$pid"; }

# ---- 6. the demo conductor (the parts of demo.sh that are not the Jetson's job) ---------------------
MON_PID=""; FINISHED=0
PHOTO_DIR="$(python3 -m sim_bringup.sim_paths photo-dir 2>/dev/null)"

# finish : make sure the score report exists, copy the photos, print the report (once).
finish() {
  [ "$FINISHED" = 1 ] && return
  FINISHED=1
  [ -n "$MON_PID" ] && kill -INT "$MON_PID" 2>/dev/null
  if [ "$SCORE" = 1 ] && [ ! -s "$RUN_DIR/score_report.md" ] && sim_alive; then
    info "asking score_run to write the score report ..."
    timeout 25 ros2 service call /sim/demo/write_score std_srvs/srv/Trigger "{}" > "$RUN_DIR/write_score.log" 2>&1 || true
  fi
  if [ -n "$PHOTO_DIR" ] && compgen -G "$PHOTO_DIR/*" > /dev/null; then
    mkdir -p "$RUN_DIR/photos"
    cp -p "$PHOTO_DIR"/* "$RUN_DIR/photos/" 2>/dev/null
    local det_dir
    det_dir="$(dirname "$PHOTO_DIR")/detection_results_sahi"
    if compgen -G "$det_dir/*" > /dev/null; then mkdir -p "$RUN_DIR/detections"; cp -p "$det_dir"/* "$RUN_DIR/detections/" 2>/dev/null; fi
    info "photos copied to $RUN_DIR/photos ($(find "$RUN_DIR/photos" -type f | wc -l) files)"
  fi
  [ -n "$SIM_LOG" ] && cp "$SIM_LOG" "$RUN_DIR/sim.log" 2>/dev/null
  if [ -s "$RUN_DIR/score_report.md" ]; then
    say "SCORE REPORT ($RUN_DIR/score_report.md)"
    head -40 "$RUN_DIR/score_report.md" | sed 's/^/    /'
  elif [ "$SCORE" = 1 ]; then
    warn "no score report was written (see $RUN_DIR/sim.log)"
  fi
}

on_stop() {
  trap - INT TERM
  echo
  warn "Stop requested: finishing cleanly (score report, photos), then stopping the simulator"
  [ "$DEMO" = 1 ] && finish
  stop_simulator
  exit 130
}
trap on_stop INT TERM

# conduct_demo : demo.sh's steps 2-6 (its step 3, starting ./launcher.sh sim, happens in the jetson container).
# Returns 0 = landed, 3 = timed out, other = a step failed.
conduct_demo() {
  local url="udpin:127.0.0.1:$GCS_PORT" mission need=()
  mission="$(python3 -m sim_bringup.sim_paths world "$WORLD" mission 2>/dev/null)"
  if [ -z "$mission" ]; then
    warn "world \"$WORLD\" has no survey mission: nothing to fly. Fly by hand (demo panel / Mission Planner)."
    return 2
  fi
  say "DEMO 1/5  Waiting for the autopilot (GPS + EKF + pre-arm checks; the first 30-60 s are normal settling)"
  if ! run_wait ros2 run sim_bringup mission_tool --url "$url" --timeout 360 wait-ready; then
    warn "the simulated autopilot never became ready. Last lines of the simulator log ($SIM_LOG):"
    tail -25 "$SIM_LOG" 2>/dev/null | sed 's/^/    /'
    return 1
  fi
  say "DEMO 2/5  Uploading the survey mission (like Mission Planner): $mission"
  run_wait ros2 run sim_bringup mission_tool --url "$url" upload --mission "$mission" || return 1
  say "DEMO 3/5  Waiting for the team's code in the \"jetson\" container (MAVROS, video_cam, new_od, main_controller)"
  info "it starts there by itself: ./launcher.sh sim  (its lines start with \"astra-jetson\"; up to $((STACK_WAIT / 60)) min"
  info "the first time, because that container builds ros2_ws first)"
  [ "$DETECTOR" = real ] && need=(--need-detector)
  if ! run_wait ros2 run sim_bringup stack_check --timeout "$STACK_WAIT" --settle 20 "${need[@]}"; then
    warn "the team's code did not get ready. Is the jetson container running (docker logs astra-jetson)?"
    warn "If its nodes run but are not seen here, ROS 2 cannot talk across the containers (docs/simulation/DOCKER.md)."
    return 1
  fi
  # The camera emulator already looks straight down; say it again through the team's own camera command
  # (video_cam -> SIYI SDK), as the operator would before a survey.
  timeout 15 ros2 topic pub -w 1 --once /camera/command std_msgs/msg/String "{data: 'gimbal_set_angles 0,-90'}" >/dev/null 2>&1 || true
  say "DEMO 4/5  Watching the mission (phases appear below)"
  ros2 run sim_bringup phase_monitor --log "$RUN_DIR/phases.jsonl" --photo-dir "$PHOTO_DIR" --timeout $((TIMEOUT_MIN * 60)) &
  MON_PID=$!
  say "DEMO 5/5  Arming and starting the mission (AUTO)"
  run_wait ros2 run sim_bringup mission_tool --url "$url" fly || { warn "the mission did not start (see above)"; return 1; }
  say "The drone flies the mission by itself now: search lanes, photos, detections, targets, drops, home"
  while kill -0 "$MON_PID" 2>/dev/null; do
    sim_alive || { warn "the simulator stopped unexpectedly (see $SIM_LOG)"; return 1; }
    sleep 2
  done
  wait "$MON_PID"
}

if [ "$DEMO" = 1 ]; then
  info "simulator log: $SIM_LOG  (only the demo steps are shown here)"
  conduct_demo
  RESULT=$?
  case "$RESULT" in
    0) ok "The drone has landed: mission complete." ;;
    3) warn "The demo timed out after $TIMEOUT_MIN minutes." ;;
    *) warn "The demo did not finish (code $RESULT)." ;;
  esac
  # score_run writes the report about 8 s after the landing; give it a moment.
  for _ in $(seq 1 30); do [ -s "$RUN_DIR/score_report.md" ] && break; sim_alive || break; sleep 1; done
  finish
  if [ "$KEEP" = 1 ]; then
    ok "ASTRA_KEEP=1: everything keeps running. Stop it with:  bash simulation/docker/compose_down.sh"
  else
    if [ "$LINGER_S" -gt 0 ] 2>/dev/null && sim_alive; then
      info "the windows stay open for $LINGER_S s so you can look around (Ctrl+C closes everything now)"
      for _ in $(seq 1 "$LINGER_S"); do sim_alive || break; sleep 1; done
    fi
    trap - INT TERM
    stop_simulator
    say "Everything from this run is in $RUN_DIR"
    exit "$RESULT"
  fi
else
  info "showing the simulator log ($SIM_LOG); fly with the demo panel or Mission Planner"
  tail -n +1 -F "$SIM_LOG" 2>/dev/null &
fi

# Manual mode or ASTRA_KEEP=1: stay until the simulator ends (window closed, compose_down.sh, Ctrl+C).
while sim_alive; do sleep 2; done
say "The simulator has stopped."
exit 0
