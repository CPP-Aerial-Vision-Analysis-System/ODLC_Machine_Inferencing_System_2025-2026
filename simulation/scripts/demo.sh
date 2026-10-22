#!/usr/bin/env bash
# =============================================================================
#  demo.sh  --  THE FULL SEARCH DEMO, one command
#
#  PURPOSE: shows the whole search-and-deliver mission in the simulator, flown by the team's REAL code:
#    1. starts the simulator WITH its windows: Gazebo (3D view following the drone + a live camera POV
#       panel), RViz (camera POV + markers) and the demo panel (live POV, newest photo with detection boxes,
#       detection list with class / confidence / lat-lon, mission phase); sim_demo's score_run records the run;
#    2. waits until ArduCopter SITL has GPS and a settled EKF (the first ~30 s "Arm: Gyros inconsistent" is
#       normal), then uploads the world's survey mission (like Mission Planner);
#    3. starts the team's real flight stack exactly as on the drone: ./launcher.sh sim (MAVROS + video_cam +
#       new_od detection + DigiCamCtrl trigger + waypoint_manager + main_controller), from the Linux-side repo
#       copy, and waits until it is ready (MAVROS connected, mission read, detector model loaded);
#    4. arms and starts the mission (AUTO): take-off, search lanes over the hiker (mannequin) and the tent with
#       a photo every second, detections, main_controller inserts the target waypoints, the drone flies to each
#       target and the payloads fall in Gazebo, then it returns home (RTL) and lands;
#    5. prints every phase in this terminal as it happens (phase_monitor), and
#    6. at the end (landed, or Ctrl+C) makes sure the score report is written (detections vs the ground truth,
#       drop errors), copies the photos into the run folder and stops everything cleanly.
#  Everything of one run is in ~/.astra_sim/runs/<date>_<world>/ (logs, phases, photos, score_report.md).
#
#  HOW TO RUN:   bash demo.sh                          -> runway world (small and quick, about 6-8 minutes)
#                bash demo.sh suas_field                -> the SUAS 2026 field (longer)
#                bash demo.sh runway --detector truth   -> answer-key detector stand-in instead of the team model
#                bash demo.sh runway --camera a8mini    -> A8 mini profile, 4K photos
#  Options:      --detector real|truth   who finds the targets (default real = the team's new_od + yolo26m)
#                --camera zr10|a8mini    photo profile (default zr10 = 2K photos, last year's camera default)
#                --zoom survey|team      survey (default): photos keep the full 81 deg view the mission was
#                                        planned for; team: let video_cam's 5x zoom rack shrink the photos
#                --headless              no windows (for tests and servers)
#                --no-windows-gcs        no MAVLink copy for Mission Planner / QGroundControl on Windows (by default,
#                                        inside WSL, it goes to the Windows host: Mission Planner -> UDP -> port 14550)
#                --windows-gcs HOST[:PORT]  send that copy somewhere else (another laptop, mirrored networking)
#                --timeout MINUTES       give up after this long (default 30)
#                --keep                  leave everything running after the landing
#                --manual                MISSION PLANNER MODE (fly it like at the competition): start everything
#                                        (simulator, windows, team stack, the MAVLink copy for Mission Planner) and
#                                        then WAIT: nothing is uploaded, armed or started automatically. You do it
#                                        in Mission Planner (the steps are printed). Stays open like --keep.
#                --dry-run               check everything and print the plan, start nothing
#  Run it again: START OVER in the demo panel (click twice), or bash start_over.sh [--fly] (see start_over.sh).
#  Advanced:     --mission FILE (fly another .waypoints mission), --sitl-instance N, --gcs-port P,
#                --mavros-port P, --panel-port P (run beside another simulator), --launch-arg name:=value
#                (repeatable, passed to sim.launch.py). World "standin" = ardupilot_gazebo's runway, no targets
#                (needs --mission; for testing the chain when sim_worlds is not built).
#  Plug and play (swap one piece for your own; docs/simulation/PLUGINS.md and ARDUPILOT.md):
#                --profile NAME|FILE     a plug-in profile (simulation/profiles/NAME.yaml or your own file): which
#                                        detector, mission brain, autopilot, camera, world and extras to use
#                --set KEY=VALUE         change one profile setting (repeatable), e.g. --set camera.strict=true
#                --detector custom       your detector (needs a profile with detector.custom)
#                --sitl-version V        ArduCopter V from setup/fetch_sitl.sh (e.g. 4.7.1, latest)
#                --sitl-source DIR       ArduCopter built in an ArduPilot source tree (e.g. /opt/ardupilot)
#                --sitl-bin FILE         any arducopter program
#                --param-file FILE       extra parameters after the simulator's own (e.g. the real drone's .param;
#                                        hardware lines are filtered out), repeatable
#                --autopilot external --mavlink ADDR   an autopilot running elsewhere (another container / PC)
#                --mavros-out ADDR, --extra-out ADDR, --camera-bind IP, --fcu-url URL, --team-stack external
#                                        network endpoints for Docker / two computers
#                Options typed here win over the profile; --dry-run prints the profile's full plan.
#                Without any of these, demo.sh runs exactly as it always did (no profile is read).
#  Stop early:   Ctrl+C here (the score report is still written).
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

usage() { astra_usage "$0"; }

# ---- options ---------------------------------------------------------------------------------------
WORLD="runway"; DETECTOR="real"; CAMERA="zr10"; ZOOM="survey"; HEADLESS=0; TIMEOUT_MIN=30; KEEP=0; DRY=0
INSTANCE=0; GCS_PORT=14550; MAVROS_PORT=14552; PANEL_PORT=14553; LAUNCH_EXTRA=(); MISSION_OVERRIDE=""
WINDOWS_GCS=true; WINDOWS_GCS_ADDR=""
# Plug and play (package sim_plugins). PLUGIN=1 when a profile, --set or a plug-in option is given; then every
# typed option is ALSO recorded as a profile setting (TYPED), so profile_tool works out the whole plan in one
# place and what you type always wins over the profile. PLUGIN=0 = exactly the old behaviour, no profile read.
PROFILE=""; SETS=(); TYPED=(); PLUGIN=0; PARAM_FILES=""; EXTRA_OUTS=""
MANUAL=0          # --manual: Mission Planner mode (a person uploads, arms and starts; see start_over.sh too)
FIRST=1
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --detector) shift; DETECTOR="${1:-}"; TYPED+=("detector.kind=$DETECTOR"); [ "$DETECTOR" != custom ] || PLUGIN=1 ;;
    --camera) shift; CAMERA="${1:-}"; TYPED+=("camera.model=$CAMERA") ;;
    --zoom) shift; ZOOM="${1:-}"; TYPED+=("camera.zoom=$ZOOM") ;;
    --headless) HEADLESS=1; TYPED+=("run.headless=true") ;;
    --no-windows-gcs) WINDOWS_GCS=false; TYPED+=("network.windows_gcs=false") ;;
    --windows-gcs) shift; WINDOWS_GCS_ADDR="${1:-}"; [ -n "$WINDOWS_GCS_ADDR" ] || die "--windows-gcs needs HOST or HOST:PORT"
                   TYPED+=("network.windows_gcs_addr=$WINDOWS_GCS_ADDR") ;;
    --timeout) shift; TIMEOUT_MIN="${1:-30}"; TYPED+=("run.timeout_min=$TIMEOUT_MIN") ;;
    --keep) KEEP=1 ;;
    --manual) MANUAL=1; KEEP=1 ;;
    --dry-run) DRY=1 ;;
    --sitl-instance) shift; INSTANCE="${1:-0}"; TYPED+=("autopilot.instance=$INSTANCE") ;;
    --gcs-port) shift; GCS_PORT="${1:-}"; TYPED+=("network.gcs_port=$GCS_PORT") ;;
    --mavros-port) shift; MAVROS_PORT="${1:-}"; TYPED+=("network.mavros_port=$MAVROS_PORT") ;;
    --panel-port) shift; PANEL_PORT="${1:-}"; TYPED+=("network.panel_port=$PANEL_PORT") ;;
    --launch-arg) shift; LAUNCH_EXTRA+=("${1:-}") ;;
    --mission) shift; MISSION_OVERRIDE="${1:-}"; TYPED+=("world.mission=$MISSION_OVERRIDE") ;;
    # -- plug and play --------------------------------------------------------------------------------
    --profile) shift; PROFILE="${1:-}"; [ -n "$PROFILE" ] || die "--profile needs a profile name or a file"; PLUGIN=1 ;;
    --set) shift; [[ "${1:-}" == *=* ]] || die "--set needs KEY=VALUE (for example --set detector.kind=truth)"
           SETS+=("$1"); PLUGIN=1 ;;
    --sitl-version) shift; TYPED+=("autopilot.kind=sitl_binary" "autopilot.version=${1:-}"); PLUGIN=1 ;;
    --sitl-bin) shift; TYPED+=("autopilot.kind=sitl_binary" "autopilot.binary=${1:-}"); PLUGIN=1 ;;
    --sitl-source) shift; TYPED+=("autopilot.kind=sitl_source" "autopilot.source_dir=${1:-}"); PLUGIN=1 ;;
    --param-file) shift; PARAM_FILES="${PARAM_FILES:+$PARAM_FILES,}${1:-}"; PLUGIN=1 ;;
    --autopilot) shift; TYPED+=("autopilot.kind=${1:-}"); PLUGIN=1 ;;
    --mavlink) shift; TYPED+=("autopilot.external.mavlink=${1:-}"); PLUGIN=1 ;;
    --mavros-out) shift; TYPED+=("network.mavros_out=${1:-}"); PLUGIN=1 ;;
    --extra-out) shift; EXTRA_OUTS="${EXTRA_OUTS:+$EXTRA_OUTS,}${1:-}"; PLUGIN=1 ;;
    --camera-bind) shift; TYPED+=("network.camera_bind=${1:-}"); PLUGIN=1 ;;
    --fcu-url) shift; TYPED+=("network.fcu_url=${1:-}"); PLUGIN=1 ;;
    --team-stack) shift; TYPED+=("run.team_stack=${1:-}"); PLUGIN=1 ;;
    -*) die "unknown option $1 (see --help)" ;;
    *) if [ "$FIRST" = 1 ]; then WORLD="$1"; FIRST=0; TYPED+=("world.name=$1"); else die "unexpected argument \"$1\" (see --help)"; fi ;;
  esac
  shift
done
[ -z "$PARAM_FILES" ] || TYPED+=("autopilot.param_files=$PARAM_FILES")
[ -z "$EXTRA_OUTS" ] || TYPED+=("network.extra_outs=$EXTRA_OUTS")
# A person flies the Mission Planner mode: give them 3 hours instead of 30 minutes (unless --timeout was typed).
if [ "$MANUAL" = 1 ] && [[ " ${TYPED[*]} " != *" run.timeout_min="* ]]; then TIMEOUT_MIN=180; fi
if [ "$PLUGIN" = 0 ]; then
  case "$DETECTOR" in real|truth) ;; *) die "--detector must be real, truth or custom (custom needs --profile)" ;; esac
  case "$CAMERA" in zr10|a8mini) ;; *) die "--camera must be zr10 or a8mini" ;; esac
  case "$ZOOM" in survey|team) ;; *) die "--zoom must be survey or team" ;; esac
  for n in "$INSTANCE" "$GCS_PORT" "$MAVROS_PORT" "$PANEL_PORT" "$TIMEOUT_MIN"; do
    case "$n" in ''|*[!0-9]*) die "\"$n\" is not a whole number" ;; esac
  done
fi

say "ASTRA full search demo"
astra_load_env
astra_require_built

# ---- plug-in profile (only with --profile / --set / a plug-in option) ----------------------------------
# profile_tool reads the profile + your settings, checks every value and that every file / program / package it
# names exists here, and prints shell variables: WORLD DETECTOR CAMERA ZOOM HEADLESS TIMEOUT_MIN INSTANCE ports,
# MISSION_OVERRIDE MODEL_OVERRIDE FCU_URL_OVERRIDE TEAM_STACK_WHERE NEED_DETECTOR CAMERA_KIND CAMERA_IP and the
# PLUGIN_SIM_ARGS / PLUGIN_TEAM_ARGS / PLUGIN_STACK_ARGS lists. Without a profile these keep today's values.
PLUGIN_SIM_ARGS=(); PLUGIN_TEAM_ARGS=(); PLUGIN_STACK_ARGS=(); PLUGIN_SUMMARY=""; PLUGIN_DETECTOR_TEXT=""
MODEL_OVERRIDE=""; FCU_URL_OVERRIDE=""; TEAM_STACK_WHERE=local; CAMERA_KIND=siyi_emulator; CAMERA_IP=""
NEED_DETECTOR=0; [ "$DETECTOR" != real ] || NEED_DETECTOR=1
if [ "$PLUGIN" = 1 ]; then
  PT_ARGS=(shell --for demo)
  [ -z "$PROFILE" ] || PT_ARGS+=(--profile "$PROFILE")
  for s in "${SETS[@]}" "${TYPED[@]}"; do PT_ARGS+=(--set "$s"); done
  [ "$DRY" = 0 ] || PT_ARGS+=(--no-write)
  PLUGIN_VARS="$(python3 -m sim_plugins.profile_tool "${PT_ARGS[@]}")" \
    || die "the plug-in profile cannot be used (the reasons are above; sim_plugins not built? run setup/03_build.sh).
    Check it with:  python3 -m sim_plugins.profile_tool plan${PROFILE:+ --profile $PROFILE} --check"
  eval "$PLUGIN_VARS"
fi
words() { [ $# -eq 0 ] || printf ' %s' "$@"; }    # " a b c" (nothing at all for an empty list)

# ---- prerequisites (all checked before anything starts) --------------------------------------------
PROBLEMS=0
problem() { warn "$*"; PROBLEMS=$((PROBLEMS + 1)); }
for pkg in main video_cam detection mapping waypoint_mavros gps_ros2 wp_sender interfaces mavros; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || problem "the team package \"$pkg\" is not built/installed (run setup/03_build.sh)"
done
for pkg in sim_models siyi_emulator; do
  ros2 pkg prefix "$pkg" >/dev/null 2>&1 || problem "the simulation package \"$pkg\" is not built (run setup/03_build.sh)"
done
if [ "$WORLD" != standin ] && [ ! -f "$WORLD" ]; then
  ros2 pkg prefix sim_worlds >/dev/null 2>&1 || problem "the simulation package \"sim_worlds\" is not built (run setup/03_build.sh)"
fi
[ -f "$WORLD" ] && WORLD="$(readlink -f "$WORLD")"          # a world config .yaml / .sdf given as a file
WORLD_NAME="$(basename "${WORLD%.*}")"
ros2 pkg prefix sim_demo >/dev/null 2>&1 || warn "sim_demo is not built: no demo panel, no markers, no score report (the flight itself still works)"
LAUNCHER="$ASTRA_REPO/launcher.sh"
[ -f "$LAUNCHER" ] || problem "the team's launcher.sh is missing in $ASTRA_REPO"
if [ "$WORLD" = standin ]; then
  WORLD_FILE="(stand-in: ardupilot_gazebo iris_runway, no targets)"; MISSION=""; TRUTH=""
  [ -n "${ARDUPILOT_GAZEBO_DIR:-}" ] || problem "the stand-in world needs ardupilot_gazebo (run setup/02_setup_ardupilot.sh)"
else
  WORLD_FILE="$(python3 -m sim_bringup.sim_paths world "$WORLD" world_file 2>/dev/null)"
  MISSION="$(python3 -m sim_bringup.sim_paths world "$WORLD" mission 2>/dev/null)"
  TRUTH="$(python3 -m sim_bringup.sim_paths world "$WORLD" truth 2>/dev/null)"
  [ -n "$WORLD_FILE" ] || problem "world \"$WORLD\" has no world file (sim_worlds/config/$WORLD.yaml missing or broken)"
fi
if [ -n "$MISSION_OVERRIDE" ]; then
  [ -f "$MISSION_OVERRIDE" ] && MISSION="$(readlink -f "$MISSION_OVERRIDE")" || problem "mission file not found: $MISSION_OVERRIDE"
fi
[ -n "$MISSION" ] || problem "world \"$WORLD\" has no survey mission (the demo needs one; give one with --mission FILE)"
[ -n "$TRUTH" ] || warn "world \"$WORLD\" has no truth file: the score report cannot compare with the real target positions"
if [ -n "$MISSION" ]; then
  ros2 run sim_bringup mission_tool check-mission --mission "$MISSION" | sed 's/^/    /' || problem "the mission file does not have what the team code needs (see above)"
fi
MODEL="$(python3 -m sim_bringup.sim_paths model-path)"
[ -z "$MODEL_OVERRIDE" ] || MODEL="$MODEL_OVERRIDE"          # a profile's detector.model_path
if [ "$DETECTOR" = real ] && [ ! -s "$MODEL" ]; then
  warn "the detector weights $MODEL are missing; trying to download them now (needs internet, ~40 MB)"
  if [ "$DRY" = 0 ]; then
    mkdir -p "$(dirname "$MODEL")"
    (cd "$(dirname "$MODEL")" && timeout 600 python3 -c "from ultralytics import YOLO; YOLO('$(basename "$MODEL")')" >/dev/null 2>&1) || true
  fi
  [ -s "$MODEL" ] || problem "no detector weights: run setup/03_build.sh with internet, or use --detector truth"
fi
UDP_NOW=$(ss -lunH 2>/dev/null)
port_busy() { printf '%s\n' "$UDP_NOW" | grep -qE "[:.]$1\b"; }
port_busy "$GCS_PORT" && problem "udp $GCS_PORT is in use (a ground station like QGroundControl?): close it, or use --gcs-port"
# (a team stack that runs elsewhere, run.team_stack: external, may already be listening: that is expected)
if [ "$TEAM_STACK_WHERE" != external ]; then
  port_busy "$MAVROS_PORT" && problem "udp $MAVROS_PORT is in use (is ./launcher.sh sim / MAVROS already running?): stop it first"
fi
if [ "$PROBLEMS" -gt 0 ]; then
  die "$PROBLEMS problem(s) found; nothing was started. Fix them and run demo.sh again."
fi
ok "everything needed is installed and built"

# ---- the plan --------------------------------------------------------------------------------------
PHOTO_DIR="$(python3 -m sim_bringup.sim_paths photo-dir)"
STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="$ASTRA_SIM_HOME/runs/${STAMP}_${WORLD_NAME}"
MAX_ZOOM=""
[ "$ZOOM" = survey ] && MAX_ZOOM=1
case "$ZOOM" in survey|team) ;; *) MAX_ZOOM="$ZOOM" ;; esac   # a profile's camera.zoom number = the max zoom
SIM_ARGS=("world:=$WORLD" "camera_model:=$CAMERA" "detector:=$DETECTOR" "score:=true" "demo_run_dir:=$RUN_DIR"
          "sitl_instance:=$INSTANCE" "gcs_port:=$GCS_PORT" "mavros_port:=$MAVROS_PORT" "panel_port:=$PANEL_PORT")
[ -n "$MAX_ZOOM" ] && SIM_ARGS+=("camera_max_zoom:=$MAX_ZOOM")
[ "$HEADLESS" = 1 ] && SIM_ARGS+=("gui:=false" "rviz:=false" "panel:=false")
# The copy of the MAVLink for Mission Planner / QGroundControl on Windows (the launch finds the address itself).
SIM_ARGS+=("windows_gcs:=$WINDOWS_GCS")
[ -n "$WINDOWS_GCS_ADDR" ] && SIM_ARGS+=("windows_gcs_addr:=$WINDOWS_GCS_ADDR")
# A profile's other sim.launch.py arguments (autopilot, network, camera ...); --launch-arg still comes last.
SIM_ARGS+=("${PLUGIN_SIM_ARGS[@]}")
SIM_ARGS+=("${LAUNCH_EXTRA[@]}")
GCS_URL="udpin:127.0.0.1:$GCS_PORT"
TRACKER="ros2 launch sim_bringup team_stack.launch.py detector:=$DETECTOR$(words "${PLUGIN_TEAM_ARGS[@]}")"
TEAM_FCU_URL="${FCU_URL_OVERRIDE:-udp://:$MAVROS_PORT@localhost:$MAVROS_PORT}"
NEED=()
[ "$NEED_DETECTOR" != 1 ] || NEED=(--need-detector)
info "world      $WORLD ($WORLD_FILE)"
info "mission    $MISSION"
if [ -n "$PLUGIN_DETECTOR_TEXT" ]; then
  info "detector   $PLUGIN_DETECTOR_TEXT"
else
  info "detector   $DETECTOR$([ "$DETECTOR" = real ] && echo " (team new_od + $MODEL)" || echo " (sim_demo truth_detector stand-in)")"
fi
info "camera     $CAMERA, zoom: $ZOOM"
info "run folder $RUN_DIR"
info "photos     $PHOTO_DIR"
if [ "$PLUGIN" = 1 ]; then
  info "plug-in profile, slot by slot:"
  printf '%s
' "$PLUGIN_SUMMARY" | sed 's/^/      /'
fi
if [ "$DRY" = 1 ]; then
  say "DRY RUN: this is what would run (nothing was started)"
  echo "    1. ros2 launch sim_bringup sim.launch.py ${SIM_ARGS[*]}"
  echo "    2. ros2 run sim_bringup mission_tool --url $GCS_URL wait-ready"
  echo "    3. ros2 run sim_bringup mission_tool --url $GCS_URL upload --mission $MISSION"
  if [ "$TEAM_STACK_WHERE" = external ]; then
    echo "    4. (the team stack runs elsewhere: start it there with  TRACKER_CMD=\"$TRACKER\" ./launcher.sh sim)"
  else
    echo "    4. cd $ASTRA_REPO && TRACKER_CMD=\"$TRACKER\" FCU_URL=$TEAM_FCU_URL ./launcher.sh sim"
  fi
  echo "    5. ros2 run sim_bringup stack_check$(words "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}")"
  echo "    6. ros2 run sim_bringup phase_monitor --log $RUN_DIR/phases.jsonl"
  echo "    7. ros2 run sim_bringup mission_tool --url $GCS_URL fly        (arm + start, mission already uploaded)"
  echo "    8. wait for LANDED, score_report.md in the run folder, copy photos, stop everything"
  [ "$MANUAL" = 0 ] || echo "    --manual: steps 3 and 7 are YOURS, in Mission Planner (Write WPs, Arm, Auto); step 8 keeps everything open"
  exit 0
fi

# ---- start -----------------------------------------------------------------------------------------
if [ -s "$ASTRA_STATE_DIR/session" ] && [ -n "$(astra_session_pids "$(cat "$ASTRA_STATE_DIR/session")")" ]; then
  say "A simulator from before is still running: stopping it first"
  bash "$ASTRA_SCRIPTS_DIR/stop_sim.sh" --quiet || warn "something from the old run is still alive (see above)"
fi
astra_mem_check
if [ "$CAMERA_KIND" != none ]; then
  astra_ensure_camera_ip "${CAMERA_IP:-$ASTRA_CAMERA_IP}" || die "the camera address is needed: without it the team's video_cam cannot take photos"
fi
mkdir -p "$RUN_DIR"
# Old photos in the team folder would be re-processed and could send the drone to an old target: move them away.
astra_archive_photos "$PHOTO_DIR" "$STAMP"

astra_new_session demo
export ASTRA_RUN_DIR="$RUN_DIR"
printf 'SESSION=%s\nWORLD=%s\nRUN_DIR=%s\nSTARTED=%s\n' "$ASTRA_SIM_SESSION" "$WORLD" "$RUN_DIR" "$(date)" > "$RUN_DIR/demo.env"
printf 'MISSION=%s\nMANUAL=%s\n' "$MISSION" "$MANUAL" >> "$RUN_DIR/demo.env"     # (START OVER uploads the same mission)
NOMISSION=()
[ "$MANUAL" = 0 ] || NOMISSION=(--no-mission)      # Mission Planner writes the mission after the team stack is up
SIM_PG=""; TEAM_PG=""; MON_PID=""; FINISHED=0; INTERRUPTED=0

stop_group() {   # stop_group PGID NAME : Ctrl+C to a whole process group, then wait (up to 20 s)
  local pg="$1" name="$2"
  [ -n "$pg" ] || return 0
  kill -0 -- "-$pg" 2>/dev/null || return 0
  info "stopping the $name ..."
  kill -INT -- "-$pg" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- "-$pg" 2>/dev/null || return 0; sleep 1; done
  kill -TERM -- "-$pg" 2>/dev/null; sleep 3
  kill -KILL -- "-$pg" 2>/dev/null
  return 0
}

finish() {   # write the score, collect the photos, stop everything (called once)
  [ "$FINISHED" = 1 ] && return
  FINISHED=1
  trap - INT TERM
  [ -n "$MON_PID" ] && kill -INT "$MON_PID" 2>/dev/null
  say "Finishing the demo"
  if [ ! -s "$RUN_DIR/score_report.md" ] && [ -n "$SIM_PG" ] && kill -0 -- "-$SIM_PG" 2>/dev/null; then
    info "asking score_run to write the score report ..."
    timeout 25 ros2 service call /sim/demo/write_score std_srvs/srv/Trigger "{}" > "$RUN_DIR/write_score.log" 2>&1 || true
  fi
  if compgen -G "$PHOTO_DIR/*" > /dev/null; then
    mkdir -p "$RUN_DIR/photos"
    cp -p "$PHOTO_DIR"/* "$RUN_DIR/photos/" 2>/dev/null
    DET_DIR="$(dirname "$PHOTO_DIR")/detection_results_sahi"
    if compgen -G "$DET_DIR/*" > /dev/null; then mkdir -p "$RUN_DIR/detections"; cp -p "$DET_DIR"/* "$RUN_DIR/detections/" 2>/dev/null; fi
    info "photos copied to $RUN_DIR/photos ($(find "$RUN_DIR/photos" -type f | wc -l) files)"
  fi
  echo
  if [ -s "$RUN_DIR/score_report.md" ]; then
    say "SCORE REPORT ($RUN_DIR/score_report.md)"
    head -40 "$RUN_DIR/score_report.md" | sed 's/^/    /'
  else
    warn "no score report was written yet (it is written when the simulator stops; see $RUN_DIR/sim.log)"
  fi
  # START OVER (start_over.sh / the panel's button) has taken this simulator over: never close it from here.
  if [ "$(cat "$ASTRA_STATE_DIR/keep_open" 2>/dev/null)" = "$ASTRA_SIM_SESSION" ] && [ "$INTERRUPTED" = 0 ]; then
    KEEP=1
  fi
  if [ "$KEEP" = 1 ] && [ "$INTERRUPTED" = 0 ]; then
    ok "--keep: everything keeps running. Stop it later with:  bash $ASTRA_SCRIPTS_DIR/stop_sim.sh"
    info "fly it again: START OVER in the demo panel (click twice), or  bash $ASTRA_SCRIPTS_DIR/start_over.sh --fly"
  else
    if [ "$INTERRUPTED" = 0 ] && [ "$HEADLESS" = 0 ] && [ -t 0 ]; then
      # Give the audience a moment to look at the panel, the markers and the payloads on the ground.
      read -r -t 120 -p "    The windows stay open so you can look around. Press Enter to close everything (auto-close in 2 min) " _ || true
      echo
    fi
    if [ "$(cat "$ASTRA_STATE_DIR/keep_open" 2>/dev/null)" = "$ASTRA_SIM_SESSION" ] && [ "$INTERRUPTED" = 0 ]; then
      ok "START OVER took this simulator over meanwhile: it stays open (stop it with: bash $ASTRA_SCRIPTS_DIR/stop_sim.sh)"
      say "Everything from this run is in $RUN_DIR"
      return
    fi
    stop_group "$TEAM_PG" "team flight stack (launcher.sh)"
    stop_group "$SIM_PG" "simulator"
    bash "$ASTRA_SCRIPTS_DIR/stop_sim.sh" --quiet --session "$ASTRA_SIM_SESSION" || true
    [ -s "$RUN_DIR/score_report.md" ] && info "score report: $RUN_DIR/score_report.md"
  fi
  say "Everything from this run is in $RUN_DIR"
}
on_interrupt() { INTERRUPTED=1; echo; warn "Ctrl+C: stopping the demo (the score report is still written)"; finish; exit 130; }
trap on_interrupt INT TERM

say "STEP 1/6  Starting the simulator (Gazebo, SITL, camera emulator$([ "$HEADLESS" = 0 ] && echo ", windows"))"
setsid ros2 launch sim_bringup sim.launch.py "${SIM_ARGS[@]}" > "$RUN_DIR/sim.log" 2>&1 < /dev/null &
SIM_PG=$!
info "log: $RUN_DIR/sim.log"
[ "$HEADLESS" = 0 ] && info "Windows: Gazebo (3D view follows the drone, live camera top-right), RViz, and the demo panel."

say "STEP 2/6  Waiting for the autopilot (GPS + EKF + pre-arm checks; the first ~30-60 s are normal settling)"
if ! ros2 run sim_bringup mission_tool --url "$GCS_URL" --timeout 360 wait-ready; then
  warn "the simulated autopilot never became ready. Last lines of the simulator log:"
  tail -25 "$RUN_DIR/sim.log" | sed 's/^/    /'
  finish; exit 1
fi
# Where the launch sent the MAVLink copy for a ground station on Windows ('' = nowhere), for the audience.
WIN_GCS="$(astra_state_value windows_gcs "")"
if [ -n "$WIN_GCS" ] && [ "$MANUAL" = 1 ]; then
  info "Mission Planner on Windows can CONNECT now: connection type UDP, port ${WIN_GCS##*:} (MAVLink goes to" \
       "${WIN_GCS#udp:}). You fly the mission from it in step 5."
elif [ -n "$WIN_GCS" ]; then
  info "Mission Planner / QGroundControl on Windows can watch now: connection type UDP, port ${WIN_GCS##*:}" \
       "(MAVLink goes to ${WIN_GCS#udp:}). Watch only: no mode changes or mission uploads from it during the demo."
fi
if [ "$MANUAL" = 1 ]; then
  info "--manual: no automatic upload: YOU write the mission from Mission Planner (step 5 shows how)"
else
  info "uploading the survey mission (like Mission Planner)"
  if ! ros2 run sim_bringup mission_tool --url "$GCS_URL" upload --mission "$MISSION"; then
    finish; exit 1
  fi
fi

if [ "$TEAM_STACK_WHERE" = external ]; then
  # run.team_stack: external (a plug-in profile): the team stack runs in another container / on another PC.
  say "STEP 3/6  Waiting for the team's flight stack, which runs elsewhere (run.team_stack: external)"
  info "start it there now:  TRACKER_CMD=\"ros2 launch sim_bringup team_stack.launch.py detector:=$DETECTOR\" ./launcher.sh sim"
  info "(its MAVROS fcu_url must listen where network.mavros_out sends the MAVLink)"
  if ! ros2 run sim_bringup stack_check --timeout 600 --settle 20 "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}" "${NOMISSION[@]}"; then
    warn "the team stack that runs elsewhere did not get ready in 10 minutes"
    finish; exit 1
  fi
else
  say "STEP 3/6  Starting the team's flight stack: ./launcher.sh sim (MAVROS, video_cam, new_od, trigger, main_controller)"
  info "log: $RUN_DIR/team_stack.log"
  [ -z "$PLUGIN_DETECTOR_TEXT" ] || info "plug-in profile: detector $PLUGIN_DETECTOR_TEXT"
fi
# --settle 20: the team's main_controller sometimes dies ~10 s after start (a parameter read that times out is
# not handled, see simulation/README.md); then the team stack is started once more before anything flies.
for attempt in 1 2; do
  [ "$TEAM_STACK_WHERE" != external ] || break
  ( cd "$ASTRA_REPO" && TRACKER_CMD="$TRACKER" FCU_URL="$TEAM_FCU_URL" \
      exec setsid ./launcher.sh sim >> "$RUN_DIR/team_stack.log" 2>&1 < /dev/null ) &
  TEAM_PG=$!
  if ros2 run sim_bringup stack_check --timeout 300 --settle 20 "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}" "${NOMISSION[@]}"; then
    break
  fi
  if [ "$attempt" = 1 ]; then
    warn "the team stack is not ready (a node stopped or never started): starting it once more. Its log so far:"
    grep -aE 'process has died|Error|ERROR' "$RUN_DIR/team_stack.log" | tail -6 | sed 's/^/    /'
    stop_group "$TEAM_PG" "team flight stack (launcher.sh)"
    echo "===== demo.sh: team stack restarted $(date +%T) =====" >> "$RUN_DIR/team_stack.log"
    continue
  fi
  warn "the team stack did not get ready. Last lines of its log:"
  tail -25 "$RUN_DIR/team_stack.log" | sed 's/^/    /'
  finish; exit 1
done
# What START OVER (start_over.sh, sim_demo run_reset.py) needs to start the team stack again EXACTLY like this.
printf 'SESSION=%s\nTRACKER_CMD=%s\nFCU_URL=%s\nSTACK_CHECK_ARGS=%s\nTEAM_STACK_WHERE=%s\n' "$ASTRA_SIM_SESSION" \
  "$TRACKER" "$TEAM_FCU_URL" "$(words "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}")" "$TEAM_STACK_WHERE" \
  > "$ASTRA_STATE_DIR/team_stack.env"
# The camera emulator already starts looking straight down; say it again through the team's own camera
# command (video_cam -> SIYI SDK 0x0E), as the operator would before a survey.
timeout 15 ros2 topic pub -w 1 --once /camera/command std_msgs/msg/String "{data: 'gimbal_set_angles 0,-90'}" >/dev/null 2>&1 || true

say "STEP 4/6  Watching the mission (phases appear below)"
ros2 run sim_bringup phase_monitor --log "$RUN_DIR/phases.jsonl" --photo-dir "$PHOTO_DIR" --timeout $((TIMEOUT_MIN * 60)) &
MON_PID=$!

if [ "$MANUAL" = 1 ]; then
  say "STEP 5/6  YOUR TURN in Mission Planner: nothing is uploaded, armed or started automatically"
  python3 -m sim_demo.run_reset mp-steps --mission "$MISSION" --windows-gcs "$WIN_GCS" | sed 's/^/    /'
  info "This terminal shows every phase as it happens (it waits up to $TIMEOUT_MIN minutes for the flight)."
else
  say "STEP 5/6  Arming and starting the mission (AUTO)"
  if ! ros2 run sim_bringup mission_tool --url "$GCS_URL" fly; then
    warn "the mission did not start (see the messages above)"
    finish; exit 1
  fi
fi

say "STEP 6/6  The drone flies the mission by itself now: search lanes, photos, detections, targets, drops, home"
RESULT=0
while true; do
  if ! kill -0 "$MON_PID" 2>/dev/null; then
    wait "$MON_PID"; RESULT=$?
    break
  fi
  if ! kill -0 -- "-$SIM_PG" 2>/dev/null; then
    warn "the simulator stopped unexpectedly (see $RUN_DIR/sim.log)"; RESULT=1; break
  fi
  sleep 2
done
case "$RESULT" in
  0) ok "The drone has landed: mission complete." ;;
  3) warn "The demo timed out after $TIMEOUT_MIN minutes." ;;
  *) warn "The phase monitor ended with code $RESULT." ;;
esac
# score_run writes the report about 8 s after the landing; give it a moment.
for _ in $(seq 1 30); do [ -s "$RUN_DIR/score_report.md" ] && break; sleep 1; done
finish
exit "$RESULT"
