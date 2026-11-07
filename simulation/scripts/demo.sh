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
#                --panel                 also open the old ASTRA demo panel (the Qt window). It is OFF by default:
#                                        Mission Planner is the ground station (the camera picture is at
#                                        http://127.0.0.1:8081/pov.mjpg, see docs/simulation/MISSION_PLANNER.md)
#                --no-panel              (the default) do not open the ASTRA demo panel
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
#                --no-recorder           no live CSV recording (the flight recorder listens on UDP 14555 by default)
#                --recorder-port P       the flight recorder's own MAVProxy output port (default 14555 + 10 x the
#                                        --sitl-instance; 1024-65535, never a port the simulator itself uses)
#  Logs and data: every process logs into the run folder with a date and time on every line (demo.log = this
#                window, also the checks before the start; sim.log; team_stack.log); csv/ is written LIVE by
#                sim_demo flight_recorder (telemetry, events, phases, aims/releases from MAVLink) and completed at
#                the end (photos, detections, landing points); run_summary.md says what ran, the result, the phase
#                times and every warning/error with file:line (written at the landing, updated after the simulator
#                stopped). START OVER ends this run with its records. See: bash logs.sh --help and
#                docs/simulation/LOGS_AND_DATA.md.
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
#                --model FILE            other YOLO weights for the team's new_od (.pt; checked and loaded once
#                                        before the flight; name, size and SHA-256 go to <run>/run_meta/detector.json)
#                --team-repo DIR         run ANOTHER copy of the team's Jetson code (a teammate's fork, another
#                                        branch): DIR holds launcher.sh and a built ros2_ws (docs/simulation/SWAPPING.md)
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
#  Jetson in the loop (the team stack runs on the Jetson Orin Nano; docs/jetson/LINK.md, docs/jetson/HIL_DESIGN.md):
#                --jetson usb|ssh|virtual  usb = the USB-C cable (192.168.55.1); ssh = any address (--jetson-host);
#                                        virtual = the same container on this PC (no board). Log in first with
#                                        bash simulation/jetson/jetson.sh login
#                --jetson-host HOST, --jetson-user USER, --jetson-port N, --jetson-auth login|key   the target
#                --jetson-photos live|after  copy the Jetson's photos during the flight (default) or after it (slow Wi-Fi)
#                --jetson-tap on|off     copy the Jetson's ROS topics to this PC (default on: phases + score report)
#                --jetson-baud 115200|off  limit the MAVLink link to the TELEM2 serial speed (default off)
#                --jetson-pipeline auto|new|legacy  the camera pipeline in the container (auto = new when the image
#                                        has it, else legacy), --photo-pace auto|off|MBIT (login mode),
#                                        --jetson-like (virtual: limit the container to the Jetson's CPUs/memory)
#  What really ran: every run writes <run>/run_meta/autopilot.json (the ArduCopter version the autopilot itself
#                reported), detector.json (the model file: name, size, SHA-256) and team_stack.json (which copy
#                of the Jetson code, its git commit) with sim_plugins run_meta (docs/simulation/SWAPPING.md).
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
PANEL=0           # --panel: also open the old ASTRA demo panel (off by default: Mission Planner replaces it)
RECORDER=1; RECORDER_PORT=""      # sim_demo flight_recorder: live CSV from its own MAVProxy output (--no-recorder); "" = 14555 + 10 x instance
# --jetson (simulation/jetson/pc/hil_steps.sh): "" = the team stack runs on this PC, as always
JETSON=""; JETSON_HOST_OPT=""; JETSON_USER_OPT=""; JETSON_PORT_OPT=""; JETSON_AUTH_OPT=""; JETSON_PHOTOS=live
JETSON_TAP=on; JETSON_BAUD=off; JETSON_PACE=auto; JETSON_PIPELINE=auto; JETSON_LIKE=0
FIRST=1
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --detector) shift; DETECTOR="${1:-}"; TYPED+=("detector.kind=$DETECTOR"); [ "$DETECTOR" != custom ] || PLUGIN=1 ;;
    --camera) shift; CAMERA="${1:-}"; TYPED+=("camera.model=$CAMERA") ;;
    --zoom) shift; ZOOM="${1:-}"; TYPED+=("camera.zoom=$ZOOM") ;;
    --headless) HEADLESS=1; TYPED+=("run.headless=true") ;;
    --panel) PANEL=1 ;;
    --no-panel) PANEL=0 ;;
    --no-windows-gcs) WINDOWS_GCS=false; TYPED+=("network.windows_gcs=false") ;;
    --windows-gcs) shift; WINDOWS_GCS_ADDR="${1:-}"; [ -n "$WINDOWS_GCS_ADDR" ] || die_usage "--windows-gcs needs HOST or HOST:PORT, e.g. --windows-gcs 192.168.1.20:14550"
                   TYPED+=("network.windows_gcs_addr=$WINDOWS_GCS_ADDR") ;;
    --timeout) shift; TIMEOUT_MIN="${1:-30}"; TYPED+=("run.timeout_min=$TIMEOUT_MIN") ;;
    --keep) KEEP=1 ;;
    --manual) MANUAL=1; KEEP=1 ;;
    --dry-run) DRY=1 ;;
    --no-recorder) RECORDER=0 ;;
    --recorder-port) shift; RECORDER_PORT="${1:-}"
                     case "$RECORDER_PORT" in ''|*[!0-9]*) die_usage "--recorder-port needs a UDP port number, e.g. --recorder-port 14565" ;; esac ;;
    --sitl-instance) shift; INSTANCE="${1:-0}"; TYPED+=("autopilot.instance=$INSTANCE") ;;
    --gcs-port) shift; GCS_PORT="${1:-}"; TYPED+=("network.gcs_port=$GCS_PORT") ;;
    --mavros-port) shift; MAVROS_PORT="${1:-}"; TYPED+=("network.mavros_port=$MAVROS_PORT") ;;
    --panel-port) shift; PANEL_PORT="${1:-}"; TYPED+=("network.panel_port=$PANEL_PORT") ;;
    --launch-arg) shift; [[ "${1:-}" == *:=* ]] || die_usage "--launch-arg needs name:=value, e.g. --launch-arg panel:=true"; LAUNCH_EXTRA+=("$1") ;;
    --mission) shift; MISSION_OVERRIDE="${1:-}"; TYPED+=("world.mission=$MISSION_OVERRIDE") ;;
    # -- plug and play --------------------------------------------------------------------------------
    --profile) shift; PROFILE="${1:-}"; [ -n "$PROFILE" ] || die_usage "--profile needs a profile name or a file, e.g. --profile truth_detector"; PLUGIN=1 ;;
    --set) shift; [[ "${1:-}" == *=* ]] || die_usage "--set needs KEY=VALUE (for example --set detector.kind=truth)"
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
    --model) shift; [ -n "${1:-}" ] || die_usage "--model needs a weights file, e.g. --model ~/.astra_sim/models/my_detector.pt"
             TYPED+=("detector.model_path=$1"); PLUGIN=1 ;;
    --team-repo) shift; [ -n "${1:-}" ] || die_usage "--team-repo needs the folder of a team-code checkout, e.g. --team-repo ~/astra_fork"
                 TYPED+=("run.team_repo=$1"); PLUGIN=1 ;;
    --jetson) shift; JETSON="${1:-}" ;;
    --jetson-host) shift; JETSON_HOST_OPT="${1:-}" ;;
    --jetson-user) shift; JETSON_USER_OPT="${1:-}" ;;
    --jetson-port) shift; JETSON_PORT_OPT="${1:-}" ;;
    --jetson-auth) shift; JETSON_AUTH_OPT="${1:-}" ;;
    --jetson-photos) shift; JETSON_PHOTOS="${1:-}" ;;
    --jetson-tap) shift; JETSON_TAP="${1:-}" ;;
    --jetson-baud) shift; JETSON_BAUD="${1:-}" ;;
    --jetson-pipeline) shift; JETSON_PIPELINE="${1:-}" ;;
    --photo-pace) shift; JETSON_PACE="${1:-}" ;;
    --jetson-like) JETSON_LIKE=1 ;;
    -*) die_usage "unknown option \"$1\"" ;;
    *) if [ "$FIRST" = 1 ]; then WORLD="$1"; FIRST=0; TYPED+=("world.name=$1"); else die_usage "unexpected argument \"$1\" (the world comes first and only once, e.g. demo.sh suas_field --keep)"; fi ;;
  esac
  shift
done
# --jetson: check its options, pick the jetson_<link> profile (run.team_stack external) and the baud-limit file.
if [ -n "$JETSON" ]; then
  # shellcheck source=../jetson/pc/hil_steps.sh
  source "$ASTRA_SCRIPTS_DIR/../jetson/pc/hil_steps.sh"
  hil_options
elif [ -n "$JETSON_HOST_OPT$JETSON_USER_OPT$JETSON_PORT_OPT$JETSON_AUTH_OPT" ]; then
  die_usage "--jetson-host/--jetson-user/--jetson-port/--jetson-auth need --jetson usb|ssh|virtual"
fi
[ -z "$PARAM_FILES" ] || TYPED+=("autopilot.param_files=$PARAM_FILES")
[ -z "$EXTRA_OUTS" ] || TYPED+=("network.extra_outs=$EXTRA_OUTS")
# A person flies the Mission Planner mode: give them 3 hours instead of 30 minutes (unless --timeout was typed).
if [ "$MANUAL" = 1 ] && [[ " ${TYPED[*]} " != *" run.timeout_min="* ]]; then TIMEOUT_MIN=180; fi
if [ "$PLUGIN" = 0 ]; then
  case "$DETECTOR" in real|truth) ;; *) die_usage "--detector must be real, truth or custom (custom needs --profile), not \"$DETECTOR\"" ;; esac
  case "$CAMERA" in zr10|a8mini) ;; *) die_usage "--camera must be zr10 or a8mini, not \"$CAMERA\"" ;; esac
  case "$ZOOM" in survey|team) ;; *) die_usage "--zoom must be survey or team, not \"$ZOOM\"" ;; esac
  for n in "$INSTANCE" "$GCS_PORT" "$MAVROS_PORT" "$PANEL_PORT" "$TIMEOUT_MIN"; do
    case "$n" in ''|*[!0-9]*) die_usage "\"$n\" is not a whole number (--timeout, --sitl-instance and the --*-port options take numbers)" ;; esac
  done
fi

say "ASTRA full search demo"
astra_load_env
# stamp_lines: copy its input to its output with the date and time in front of every line (colour codes left out).
# Its own session (setsid) and ignored Ctrl+C / HUP: a Ctrl+C or a closed window never breaks the pipe the simulator
# and the team stack write their logs into (a broken pipe would kill them); it ends when they close their output.
# Its stderr goes nowhere: it must not hold this window's output open after demo.sh ended (--keep, START OVER),
# or a wrapper like  demo.sh | tee x.log  would wait until the simulator stops.
# shellcheck disable=SC2016  # (the awk program is meant to be in single quotes)
stamp_lines() { trap '' INT HUP; exec setsid awk '{ gsub(/\033\[[0-9;]*m/, ""); print strftime("%Y-%m-%d %H:%M:%S"), $0; fflush() }' 2>/dev/null; }
# Everything this window shows, from here on (also the checks below), goes into <run>/demo.log with a date and time
# per line; the terminal looks exactly the same. Until the run folder exists the copy goes to a hidden file in the
# runs folder, which is then renamed to <run>/demo.log (same disk: the open file follows the rename). The copy ignores
# Ctrl+C (trap ''), so the finishing steps after a Ctrl+C still reach the screen and the file. A start that stops
# before the run folder exists leaves its copy in ~/.astra_sim/logs/demo_not_started_<time>.log.
PRE_LOG=""
if [ "$DRY" = 0 ]; then
  mkdir -p "$ASTRA_SIM_HOME/runs" && PRE_LOG="$ASTRA_SIM_HOME/runs/.demo_window_$$.log"
  exec > >(trap '' INT TERM; tee >(stamp_lines >> "$PRE_LOG")) 2>&1
  demo_log_leftover() { [ -f "$PRE_LOG" ] && mv -f "$PRE_LOG" "$ASTRA_SIM_HOME/logs/demo_not_started_$(date +%Y%m%d-%H%M%S).log" 2>/dev/null; return 0; }
  trap demo_log_leftover EXIT
fi
astra_require_built

# ---- plug-in profile (only with --profile / --set / a plug-in option) ----------------------------------
# profile_tool reads the profile + your settings, checks every value and that every file / program / package it
# names exists here, and prints shell variables: WORLD DETECTOR CAMERA ZOOM HEADLESS TIMEOUT_MIN INSTANCE ports,
# MISSION_OVERRIDE MODEL_OVERRIDE FCU_URL_OVERRIDE TEAM_STACK_WHERE NEED_DETECTOR CAMERA_KIND CAMERA_IP and the
# PLUGIN_SIM_ARGS / PLUGIN_TEAM_ARGS / PLUGIN_STACK_ARGS lists. Without a profile these keep today's values.
PLUGIN_SIM_ARGS=(); PLUGIN_TEAM_ARGS=(); PLUGIN_STACK_ARGS=(); PLUGIN_SUMMARY=""; PLUGIN_DETECTOR_TEXT=""
MODEL_OVERRIDE=""; FCU_URL_OVERRIDE=""; TEAM_STACK_WHERE=local; CAMERA_KIND=siyi_emulator; CAMERA_IP=""
TEAM_REPO=""      # run.team_repo / --team-repo: another checkout of the team code ("" = $ASTRA_REPO, as always)
NEED_DETECTOR=0; [ "$DETECTOR" != real ] || NEED_DETECTOR=1
if [ "$PLUGIN" = 1 ]; then
  PT_ARGS=(shell --for demo)
  [ -z "$PROFILE" ] || PT_ARGS+=(--profile "$PROFILE")
  for s in "${SETS[@]}" "${TYPED[@]}"; do PT_ARGS+=(--set "$s"); done
  [ "$DRY" = 0 ] || PT_ARGS+=(--no-write)
  PLUGIN_VARS="$(python3 -m sim_plugins.profile_tool "${PT_ARGS[@]}")" \
    || die "the plug-in profile or swap option cannot be used; nothing was started.
    Likely cause: a file, program, ArduCopter version or team-code folder it names is missing or broken (each XX line
    above says which one and the exact fix), or sim_plugins is not built (then: run setup/03_build.sh).
    Check it again with:  python3 -m sim_plugins.profile_tool plan${PROFILE:+ --profile $PROFILE} --check   (guide: docs/simulation/SWAPPING.md)"
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
ros2 pkg prefix sim_demo >/dev/null 2>&1 || warn "sim_demo is not built. Consequence: no demo panel, no markers, no score report (the flight itself still works). Likely cause: its build failed or was skipped. Fix: bash \"$ASTRA_SETUP_DIR/03_build.sh\" (read its first error), then run the demo again"
TEAM_DIR="${TEAM_REPO:-$ASTRA_REPO}"             # the copy of the team code that flies (--team-repo)
LAUNCHER="$TEAM_DIR/launcher.sh"
[ -f "$LAUNCHER" ] || problem "the team's launcher.sh is missing in $TEAM_DIR"
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
[ -n "$TRUTH" ] || warn "world \"$WORLD\" has no truth file. Consequence: the score report cannot compare with the real target positions. Likely cause: its world config (sim_worlds/config/$WORLD.yaml) names no truth file, or it was not rebuilt. Fix: add the truth file to the world (docs/simulation/WORLDS.md) and run setup/03_build.sh; fine to ignore for a world without targets"
if [ -n "$MISSION" ]; then
  ros2 run sim_bringup mission_tool check-mission --mission "$MISSION" | sed 's/^/    /' || problem "the mission file does not have what the team code needs (see above)"
fi
MODEL="$(python3 -m sim_bringup.sim_paths model-path)"
[ -z "$MODEL_OVERRIDE" ] || MODEL="$MODEL_OVERRIDE"          # a profile's detector.model_path
if [ "$DETECTOR" = real ] && [ ! -s "$MODEL" ] && [ -z "$JETSON" ]; then     # (--jetson: the weights live on the Jetson)
  warn "the detector weights $MODEL are missing; trying to download them now (needs internet, ~40 MB)"
  if [ "$DRY" = 0 ]; then
    mkdir -p "$(dirname "$MODEL")"
    (cd "$(dirname "$MODEL")" && timeout 600 python3 -c "from ultralytics import YOLO; YOLO('$(basename "$MODEL")')" >/dev/null 2>&1) || true
  fi
  [ -s "$MODEL" ] || problem "no detector weights: run setup/03_build.sh with internet, or use --detector truth"
fi
# The weights file itself (format, really PyTorch, an .engine without TensorRT here ...): sim_plugins model_check,
# exit 3 = cannot be used. It is LOADED once more just before the flight (run_meta detector --load, below).
if [ "$DETECTOR" = real ] && [ -s "$MODEL" ] && [ -z "$JETSON" ]; then
  python3 -m sim_plugins.model_check --no-hash "$MODEL" 2>&1 | sed 's/^/    /'
  case "${PIPESTATUS[0]}" in
    0) ;;
    3) problem "the detector weights $MODEL cannot be used (reasons above). Fix: give other weights with --model FILE, or use --detector truth" ;;
    *) warn "could not check the detector weights (sim_plugins model_check did not run: run setup/03_build.sh). Consequence: a broken model is only noticed in the team stack log" ;;
  esac
fi
# The flight recorder's port: 14555, +10 per --sitl-instance (a second simulator beside the first gets its own); never a
# port the simulator itself uses (the recorder would take that program's MAVLink away).
[ -n "$RECORDER_PORT" ] || RECORDER_PORT=$((14555 + 10 * ${INSTANCE:-0}))
if [ "$RECORDER" = 1 ]; then
  if [ "$RECORDER_PORT" -lt 1024 ] || [ "$RECORDER_PORT" -gt 65535 ]; then
    die_usage "--recorder-port $RECORDER_PORT is not a usable UDP port (it must be 1024-65535, e.g. --recorder-port 14565)"
  fi
  for p in "$GCS_PORT" "$MAVROS_PORT" "$PANEL_PORT" 14554; do
    [ "$RECORDER_PORT" != "$p" ] || die_usage "--recorder-port $RECORDER_PORT is a port the simulator itself uses (ground station $GCS_PORT, MAVROS $MAVROS_PORT, panel $PANEL_PORT, Lattice bridge 14554): pick another one, e.g. --recorder-port 14565"
  done
fi
UDP_NOW=$(ss -lunH 2>/dev/null)
port_busy() { printf '%s\n' "$UDP_NOW" | grep -qE "[:.]$1\b"; }
port_busy "$GCS_PORT" && problem "udp $GCS_PORT is in use (a ground station like QGroundControl?): close it, or use --gcs-port"
# (a team stack that runs elsewhere, run.team_stack: external, may already be listening: that is expected)
if [ "$TEAM_STACK_WHERE" != external ]; then
  port_busy "$MAVROS_PORT" && problem "udp $MAVROS_PORT is in use (is ./launcher.sh sim / MAVROS already running?): stop it first"
fi
# The flight recorder only LISTENS (never needed for the flight): a busy port turns it off with a warning, nothing more.
if [ "$RECORDER" = 1 ] && port_busy "$RECORDER_PORT"; then
  warn "UDP $RECORDER_PORT (the flight recorder's port) is already in use. Consequence: this run records no live CSV" \
       "files (telemetry.csv stays empty; photos, detections and the run summary are still written at the end)." \
       "Likely cause: the flight recorder of an earlier run is still running. Fix: pkill -f sim_demo.flight_recorder" \
       "(or use another port with --recorder-port P), then run demo.sh again"
  RECORDER=0
fi
if [ "$PROBLEMS" -gt 0 ]; then
  die "$PROBLEMS problem(s) found (the WARNING lines above); nothing was started.
    Likely cause: a part that is not built yet, a missing file, or a port another program uses.
    Fix: do what each WARNING line says (most often:  bash \"$ASTRA_SETUP_DIR/03_build.sh\"), then run demo.sh again"
fi
ok "everything needed is installed and built"

# ---- the plan --------------------------------------------------------------------------------------
PHOTO_DIR="$(python3 -m sim_bringup.sim_paths photo-dir)"
# --team-repo: that checkout's video_cam saves into ITS OWN ros2_ws (video_cam's rule: the folder with install/ and
# src/), so the photo archive, the phase monitor and the simulator's score / truth detector look there.
[ -z "$TEAM_REPO" ] || PHOTO_DIR="$TEAM_DIR/ros2_ws/src/video_cam/mapping_photos"
STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="$ASTRA_SIM_HOME/runs/${STAMP}_${WORLD_NAME}"
MAX_ZOOM=""
[ "$ZOOM" = survey ] && MAX_ZOOM=1
case "$ZOOM" in survey|team) ;; *) MAX_ZOOM="$ZOOM" ;; esac   # a profile's camera.zoom number = the max zoom
SIM_ARGS=("world:=$WORLD" "camera_model:=$CAMERA" "detector:=$DETECTOR" "score:=true" "demo_run_dir:=$RUN_DIR"
          "sitl_instance:=$INSTANCE" "gcs_port:=$GCS_PORT" "mavros_port:=$MAVROS_PORT" "panel_port:=$PANEL_PORT")
[ -n "$MAX_ZOOM" ] && SIM_ARGS+=("camera_max_zoom:=$MAX_ZOOM")
[ "$HEADLESS" = 1 ] && SIM_ARGS+=("gui:=false" "rviz:=false" "panel:=false")
[ "$PANEL" = 0 ] && SIM_ARGS+=("panel:=false")        # --launch-arg panel:=true still wins (it comes last)
[ -z "$TEAM_REPO" ] || SIM_ARGS+=("photo_dir:=$PHOTO_DIR")   # (another checkout's photo folder, see above)
# The copy of the MAVLink for Mission Planner / QGroundControl on Windows (the launch finds the address itself).
SIM_ARGS+=("windows_gcs:=$WINDOWS_GCS")
[ -n "$WINDOWS_GCS_ADDR" ] && SIM_ARGS+=("windows_gcs_addr:=$WINDOWS_GCS_ADDR")
# A profile's other sim.launch.py arguments (autopilot, network, camera ...); --launch-arg still comes last.
SIM_ARGS+=("${PLUGIN_SIM_ARGS[@]}")
SIM_ARGS+=("${LAUNCH_EXTRA[@]}")
# The flight recorder's own copy of the MAVLink (one more MAVProxy output). It is ADDED to the extra_outs a profile or a
# --launch-arg gives (ros2 launch keeps only the LAST extra_outs:=), never replacing them.
RECORDER_OUT="udp:127.0.0.1:$RECORDER_PORT"
if [ "$RECORDER" = 1 ]; then
  LAST_OUTS=-1
  for i in "${!SIM_ARGS[@]}"; do case "${SIM_ARGS[$i]}" in extra_outs:=*) LAST_OUTS=$i ;; esac; done
  if [ "$LAST_OUTS" -ge 0 ]; then
    OUTS="${SIM_ARGS[$LAST_OUTS]#extra_outs:=}"; SIM_ARGS[LAST_OUTS]="extra_outs:=${OUTS:+$OUTS,}$RECORDER_OUT"
  else
    SIM_ARGS+=("extra_outs:=$RECORDER_OUT")
  fi
fi
GCS_URL="udpin:127.0.0.1:$GCS_PORT"
TRACKER="ros2 launch sim_bringup team_stack.launch.py detector:=$DETECTOR$(words "${PLUGIN_TEAM_ARGS[@]}")"
# --team-repo: THAT checkout's launcher.sh runs (like on the Jetson); sim_plugins' team_overlay.sh puts the simulator's
# sim_bringup under it and that checkout's team packages on top (it logs where each package comes from).
if [ -n "$TEAM_REPO" ]; then
  TRACKER="bash $ASTRA_REPO/ros2_ws/src/simulation/sim_plugins/scripts/team_overlay.sh --sim-ws $ASTRA_REPO/ros2_ws --team-ws $TEAM_DIR/ros2_ws -- $TRACKER"
fi
# MAVROS talks to SITL's SERIAL2 directly (TCP 5763 + 10 x instance), wired like the real Jetson on the Pixhawk's
# TELEM2: then ArduPilot passes the team code's status texts on to Mission Planner and the Lattice bridge, exactly like
# on the real drone (behind MAVProxy they never reached them). A profile can still set network.fcu_url.
TEAM_FCU_URL="${FCU_URL_OVERRIDE:-tcp://127.0.0.1:$((5763 + 10 * INSTANCE))}"
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
  if [ -n "$JETSON" ]; then
    echo "    4. (the team stack runs in the container on the $( [ "$JETSON" = virtual ] && echo 'virtual Jetson' || echo Jetson ): nothing starts here; see the Jetson lines at the end)"
  elif [ "$TEAM_STACK_WHERE" = external ]; then
    echo "    4. (the team stack runs elsewhere: start it there with  TRACKER_CMD=\"$TRACKER\" ./launcher.sh sim)"
  else
    echo "    4. cd $TEAM_DIR && TRACKER_CMD=\"$TRACKER\" FCU_URL=$TEAM_FCU_URL ./launcher.sh sim"
  fi
  if [ -n "$JETSON" ]; then echo "    5. (stack_check runs INSIDE the container: see the Jetson lines at the end)"
  else echo "    5. ros2 run sim_bringup stack_check$(words "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}")"; fi
  echo "    6. ros2 run sim_bringup phase_monitor --log $RUN_DIR/phases.jsonl"
  echo "    7. ros2 run sim_bringup mission_tool --url $GCS_URL fly        (arm + start, mission already uploaded)"
  echo "    8. wait for LANDED, score_report.md in the run folder, copy photos, stop everything"
  [ "$RECORDER" = 0 ] || echo "    (all along: python3 -m sim_demo.flight_recorder --url udpin:127.0.0.1:$RECORDER_PORT -> $RUN_DIR/csv/*.csv live;" \
                              "at the end csv_export + run_summary -> run_summary.md and ${ASTRA_SIM_HOME}/runs/runs_index.csv)"
  echo "    (and python3 -m sim_plugins.run_meta detector|autopilot|team-stack: $RUN_DIR/run_meta/*.json = what really ran)"
  [ "$MANUAL" = 0 ] || echo "    --manual: steps 3 and 7 are YOURS, in Mission Planner (Write WPs, Arm, Auto); step 8 keeps everything open"
  [ -z "$JETSON" ] || hil_dry_run_lines
  exit 0
fi
[ -z "$JETSON" ] || hil_preflight       # --jetson: is the board (or the virtual Jetson) ready? Stops here if not

# ---- start -----------------------------------------------------------------------------------------
if [ -s "$ASTRA_STATE_DIR/session" ] && [ -n "$(astra_session_pids "$(cat "$ASTRA_STATE_DIR/session")")" ]; then
  say "A simulator from before is still running: stopping it first"
  bash "$ASTRA_SCRIPTS_DIR/stop_sim.sh" --quiet || warn "something from the old run is still alive (see above). Consequence: two simulators may fight over the same ports and the new run can fail. Fix: if it does, stop everything with  bash \"$ASTRA_SCRIPTS_DIR/stop_sim.sh\" --all  and run the demo again"
fi
astra_mem_check
if [ "$CAMERA_KIND" != none ]; then
  astra_ensure_camera_ip "${CAMERA_IP:-$ASTRA_CAMERA_IP}" || die "the camera address ${CAMERA_IP:-$ASTRA_CAMERA_IP} is needed: without it the team's video_cam cannot take photos.
    Likely cause: no admin rights to add it (see the warning just above).
    Fix: run the command printed above once (sudo bash \"$ASTRA_SETUP_DIR/camera_ip_alias.sh\"), then run demo.sh again"
fi
mkdir -p "$RUN_DIR"
# This window's copy (see stamp_lines above) moves into the run folder now and keeps growing there as demo.log.
if [ -n "$PRE_LOG" ] && mv "$PRE_LOG" "$RUN_DIR/demo.log" 2>/dev/null; then
  info "this window is also saved to $RUN_DIR/demo.log (from the first check on)"
else
  warn "this window's copy could not be moved to $RUN_DIR/demo.log. Consequence: this run has no demo.log (every other" \
       "log and the CSV files are still written). Likely cause: the disk is full. Fix: df -h ~"
fi
# What really runs (sim_plugins run_meta; docs/simulation/SWAPPING.md): the detector first, because for the team's
# new_od it LOADS the weights once (~5-15 s) and stops here, before anything starts, when they cannot be used
# (exit 3). Any other failure of the recorder is only a warning: it never stops a flight.
RUN_META=(python3 -m sim_plugins.run_meta)
LOAD_TEST=(); [ "$DETECTOR" != real ] || [ -n "$JETSON" ] || LOAD_TEST=(--load)     # only the team's new_od loads weights (here)
"${RUN_META[@]}" detector --out "$RUN_DIR/run_meta/detector.json" --kind "$DETECTOR" --model "$MODEL" \
    --camera "$CAMERA" --text "${PLUGIN_DETECTOR_TEXT:-$DETECTOR}" "${LOAD_TEST[@]}" 2>&1 | sed 's/^/    /'
case "${PIPESTATUS[0]}" in
  0) ;;
  3) die "the detector weights $MODEL did not pass the load test (reasons above); nothing was started. Fix: give other weights with --model FILE, or use --detector truth" ;;
  *) warn "could not record the detector in $RUN_DIR/run_meta/detector.json (see above); the flight goes on" ;;
esac
# Old photos in the team folder would be re-processed and could send the drone to an old target: move them away.
astra_archive_photos "$PHOTO_DIR" "$STAMP"

astra_new_session demo
export ASTRA_RUN_DIR="$RUN_DIR"
printf 'SESSION=%s\nWORLD=%s\nRUN_DIR=%s\nSTARTED=%s\n' "$ASTRA_SIM_SESSION" "$WORLD" "$RUN_DIR" "$(date)" > "$RUN_DIR/demo.env"
printf 'MISSION=%s\nMANUAL=%s\n' "$MISSION" "$MANUAL" >> "$RUN_DIR/demo.env"     # (START OVER uploads the same mission)
# (for run_summary.md and runs_index.csv: what this run was started with)
printf 'DETECTOR=%s\nCAMERA=%s\nZOOM=%s\nPROFILE=%s\nTEAM_STACK=%s\nMODEL=%s\nRECORDER=%s\nCAMERA_KIND=%s\n' \
  "$DETECTOR" "$CAMERA" "$ZOOM" "${PROFILE:-default}" "$TEAM_STACK_WHERE" "$([ "$DETECTOR" = real ] && echo "$MODEL")" \
  "$([ "$RECORDER" = 1 ] && echo "udpin:127.0.0.1:$RECORDER_PORT")" "$CAMERA_KIND" >> "$RUN_DIR/demo.env"
# --jetson: where the team code runs, for the run summary ("Team code ran: jetson (HOST, usb|ssh|virtual)")
if [ -n "${JETSON:-}" ]; then
  printf 'TEAM_STACK=jetson\nJETSON_LINK=%s\nJETSON_HOST=%s\n' "$JETSON" \
    "${JETSON_HOST_OPT:-${JETSON_HOST:-$([ "$JETSON" = usb ] && echo 192.168.55.1 || echo "$JETSON")}}" >> "$RUN_DIR/demo.env"
fi
# Which git commit runs (run_summary.md, runs_index.csv): looked up NOW, in the background (git on the Windows drive
# takes seconds, more when the PC is busy), into run_meta/source.json; the end of the run then never waits for git.
# The Linux build copy has no .git: ASTRA_SOURCE_REPO, or ASTRA_WINDOWS_REPO (the .bat launchers), names the checkout.
( ASTRA_SOURCE_REPO="${ASTRA_SOURCE_REPO:-$([ -z "${ASTRA_WINDOWS_REPO:-}" ] || wslpath -u "$ASTRA_WINDOWS_REPO" 2>/dev/null)}" \
    python3 -m sim_demo.run_summary source "$RUN_DIR" --git-repo "$(cd "$ASTRA_SCRIPTS_DIR/../.." && pwd)" \
    > "$RUN_DIR/run_meta_source.log" 2>&1 < /dev/null & )
NOMISSION=()
[ "$MANUAL" = 0 ] || NOMISSION=(--no-mission)      # Mission Planner writes the mission after the team stack is up
SIM_PG=""; TEAM_PG=""; MON_PID=""; FINISHED=0; INTERRUPTED=0; RESULT=""; REC_PID=""
# The flight recorder (sim_demo flight_recorder) only LISTENS to its own MAVProxy output and writes csv/ live. It starts
# before the simulator so it hears the autopilot's boot banner. setsid: a Ctrl+C here must not stop it before finish()
# has collected everything; --parent-pid: it stops by itself (and writes a PARTIAL run summary) if this script vanishes.
if [ "$RECORDER" = 1 ]; then
  if python3 -c 'import sim_demo.flight_recorder' 2>/dev/null; then
    setsid python3 -m sim_demo.flight_recorder --run-dir "$RUN_DIR" --url "udpin:127.0.0.1:$RECORDER_PORT" \
        --mission "$MISSION" --parent-pid $$ > "$RUN_DIR/flight_recorder.log" 2>&1 < /dev/null &
    REC_PID=$!
    sleep 1.5                     # a recorder that cannot start (no pymavlink, port taken) ends within a second
    if kill -0 "$REC_PID" 2>/dev/null; then
      info "flight recorder: MAVLink on UDP $RECORDER_PORT -> $RUN_DIR/csv/ (live CSV), log flight_recorder.log"
    else
      warn "the flight recorder stopped right after it started, so this run records no live CSV files (the flight" \
           "is not affected; photos, detections and the run summary still come). Likely cause: pymavlink is missing" \
           "for $(command -v python3), or UDP $RECORDER_PORT is taken. Fix: read its reason below (all of it:" \
           "$RUN_DIR/flight_recorder.log), then run demo.sh again"
      tail -3 "$RUN_DIR/flight_recorder.log" 2>/dev/null | sed 's/^/    /'
      REC_PID=""
    fi
  else
    warn "sim_demo's flight recorder is not installed, so this run records no live CSV files (the flight is not" \
         "affected). Likely cause: the workspace was built before it existed. Fix: bash \"$ASTRA_SETUP_DIR/03_build.sh\""
  fi
fi

SUMMARY_T=""      # when the run summary was written: log lines stamped later (shutdown) are listed apart
finish_records() {   # [OUTCOME CODE] stop the flight recorder, complete csv/, write run_summary.md/.json + runs_index.csv row
  local outcome="${1:-}" code="${2:-}" src="${ASTRA_SOURCE_REPO:-}"
  if [ -n "$outcome" ]; then :
  elif [ "$INTERRUPTED" = 1 ]; then outcome="stopped with Ctrl+C"; code=130
  elif [ -z "$RESULT" ]; then outcome="stopped early: a step failed before the flight (see the warnings in demo.log)"; code=1
  else
    case "$RESULT" in
      0) outcome="landed: mission complete"; code=0 ;;
      3) outcome="timed out after $TIMEOUT_MIN minutes"; code=3 ;;
      *) outcome="ended with phase monitor code $RESULT"; code="$RESULT" ;;
    esac
  fi
  if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null
    for _ in $(seq 1 10); do kill -0 "$REC_PID" 2>/dev/null || break; sleep 1; done
    if kill -0 "$REC_PID" 2>/dev/null; then
      warn "the flight recorder did not stop within 10 s; stopping it hard. Consequence: the last second of csv/ may be missing."
      kill -TERM "$REC_PID" 2>/dev/null
    fi
  fi
  REC_PID=""
  # Which commit ran: normally run_meta/source.json (looked up at the start); this is only the fallback.
  [ -n "$src" ] || [ -z "${ASTRA_WINDOWS_REPO:-}" ] || src="$(wslpath -u "$ASTRA_WINDOWS_REPO" 2>/dev/null)"
  SUMMARY_T="$(date +%s)"
  if ASTRA_SOURCE_REPO="$src" python3 -m sim_demo.run_summary finish "$RUN_DIR" --live --result "$outcome" \
       --exit-code "$code" --logs-until "$SUMMARY_T" --git-repo "$(cd "$ASTRA_SCRIPTS_DIR/../.." && pwd)" \
       > "$RUN_DIR/run_summary.log" 2>&1; then
    say "RUN SUMMARY: $RUN_DIR/run_summary.md"
    sed 's/^/    /' "$RUN_DIR/run_summary.log"
    info "CSV files: $RUN_DIR/csv   (Windows: $(python3 -m sim_demo.run_summary winpath "$RUN_DIR/csv" 2>/dev/null))"
    info "look at this run again any time:  bash $ASTRA_SCRIPTS_DIR/logs.sh summary   (errors | csv | bundle)"
  else
    SUMMARY_T=""
    warn "the run summary could not be written. Consequence: no run_summary.md and no row in runs_index.csv for this" \
         "run (the CSV files written live are kept). Likely cause: a broken file in sim_demo (the Python error is the" \
         "last line below) or a full disk. Fix: bash \"$ASTRA_SCRIPTS_DIR/logs.sh\" summary $(basename "$RUN_DIR")" \
         "(all of its output: $RUN_DIR/run_summary.log; after a code change: bash \"$ASTRA_SETUP_DIR/03_build.sh\")"
    tail -5 "$RUN_DIR/run_summary.log" 2>/dev/null | sed 's/^/    /'
  fi
}
refresh_records() {  # after the simulator stopped: score_run rewrote its files then -> CSV + summary again, same result
  [ -n "$SUMMARY_T" ] || return 0
  if python3 -m sim_demo.run_summary finish "$RUN_DIR" --refresh >> "$RUN_DIR/run_summary.log" 2>&1; then
    info "run summary and CSV files updated with the score files score_run wrote when the simulator stopped"
  else
    warn "the run summary could not be updated after the simulator stopped. Consequence: run_summary.md and csv/" \
         "may miss detections finished after the landing. Fix: bash \"$ASTRA_SCRIPTS_DIR/logs.sh\" summary" \
         "$(basename "$RUN_DIR") --refresh   (its output: $RUN_DIR/run_summary.log)"
  fi
}
handover_records() {  # START OVER took the simulator over: end THIS run's records (the new run gets its own)
  local new name="a new run folder"
  new="$(grep -l "^PREVIOUS_RUN=$RUN_DIR\$" "$ASTRA_SIM_HOME"/runs/*/start_over.env 2>/dev/null | head -1)"
  [ -z "$new" ] || name="run $(basename "$(dirname "$new")")"
  finish_records "ended by START OVER (the flight goes on as $name)" 0
}

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
  # START OVER owns the old run's collection and the new recorder now. A late finish here could ask
  # the NEW score_run to write an unfinished report, or copy NEW photos into the old run folder.
  if taken_over; then
    info "START OVER owns this simulator now; its run recorder collects the results."
    handover_records
    return
  fi
  [ -n "$MON_PID" ] && kill -INT "$MON_PID" 2>/dev/null
  [ -z "$JETSON" ] || hil_finish      # --jetson: stop the container, copy its photos/logs here, close the link
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
    warn "no score report was written yet (it is written when the simulator stops; see $RUN_DIR/sim.log). Fix: write" \
         "it again from the recorded data:  ros2 run sim_demo score_run --rescore \"$RUN_DIR\""
  fi
  finish_records
  # START OVER (start_over.sh / the panel's button) has taken this simulator over: never close it from here.
  if [ "$(cat "$ASTRA_STATE_DIR/keep_open" 2>/dev/null)" = "$ASTRA_SIM_SESSION" ] && [ "$INTERRUPTED" = 0 ]; then
    KEEP=1
  fi
  if [ "$KEEP" = 1 ] && [ "$INTERRUPTED" = 0 ]; then
    ok "--keep: everything keeps running. Stop it later with:  bash $ASTRA_SCRIPTS_DIR/stop_sim.sh"
    if [ "$MANUAL" = 1 ]; then
      info "another run: START OVER in the demo panel (click twice) or  bash $ASTRA_SCRIPTS_DIR/start_over.sh ;" \
           "then do the Mission Planner steps 2-5 again (Load File, Write, Arm, Auto)"
    else
      info "fly it again: START OVER in the demo panel (click twice), or  bash $ASTRA_SCRIPTS_DIR/start_over.sh --fly"
    fi
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
    refresh_records
    [ -s "$RUN_DIR/score_report.md" ] && info "score report: $RUN_DIR/score_report.md"
  fi
  say "Everything from this run is in $RUN_DIR"
}
on_interrupt() { INTERRUPTED=1; echo; warn "Ctrl+C: stopping the demo (the score report is still written)"; finish; exit 130; }
trap on_interrupt INT TERM
# START OVER (start_over.sh, the demo panel's button) can take this simulator over at any moment: it stops the
# step this script waits for (wait-ready, stack_check, the phase monitor ...) and does the rest itself. Then this
# script must not retry, report an error or close anything: it says so and ends (everything keeps running).
taken_over() { [ "$(cat "$ASTRA_STATE_DIR/keep_open" 2>/dev/null)" = "$ASTRA_SIM_SESSION" ]; }
handed_over() {
  ok "START OVER has taken this simulator over (progress: demo panel / start_over.sh). This script ends here; everything keeps running."
  FINISHED=1
  trap - INT TERM
  handover_records          # this run's recorder stops and its summary is written; START OVER records the new run
  exit 0
}

say "STEP 1/6  Starting the simulator (Gazebo, SITL, camera emulator$([ "$HEADLESS" = 0 ] && echo ", windows"))"
# (through stamp_lines: every line of sim.log gets its date and time, also MAVProxy's, Gazebo's and tracebacks;
#  $! stays the launch's own pid = its process group, because the stamper is made inside that background job)
setsid ros2 launch sim_bringup sim.launch.py "${SIM_ARGS[@]}" > >(stamp_lines >> "$RUN_DIR/sim.log") 2>&1 < /dev/null &
SIM_PG=$!
info "log: $RUN_DIR/sim.log"
[ "$HEADLESS" = 0 ] && info "Windows: Gazebo (3D view follows the drone, live camera top-right) and RViz$([ "$PANEL" = 1 ] && echo ", and the demo panel"). Mission Planner: camera picture at http://127.0.0.1:8081/pov.mjpg"

say "STEP 2/6  Waiting for the autopilot (GPS + EKF + pre-arm checks; the first ~30-60 s are normal settling)"
if ! ros2 run sim_bringup mission_tool --url "$GCS_URL" --timeout 360 wait-ready; then
  taken_over && handed_over
  warn "the simulated autopilot never became ready (no GPS fix / EKF / pre-arm OK within 6 minutes). Likely cause: SITL or" \
       "Gazebo did not start (a port still used by an old run, or the 3D drawing failed). Fix: bash \"$ASTRA_SCRIPTS_DIR/stop_sim.sh\" --all," \
       "then run demo.sh again; if it repeats, read $RUN_DIR/sim.log. Last lines of the simulator log:"
  tail -25 "$RUN_DIR/sim.log" | sed 's/^/    /'
  finish; exit 1
fi
# Which ArduPilot really runs: its own boot banner in sim.log + AUTOPILOT_VERSION asked on the ground-station port
# (free between two mission_tool calls), the program and its parameter files -> run_meta/autopilot.json.
"${RUN_META[@]}" autopilot --out "$RUN_DIR/run_meta/autopilot.json" --sim-log "$RUN_DIR/sim.log" \
    --state "$ASTRA_STATE_DIR/current_launch.yaml" --session "$ASTRA_SIM_SESSION" --wait 5 \
    --mavlink "$GCS_URL" --mavlink-timeout 6 2>&1 | sed 's/^/    /' \
  || warn "could not record the autopilot version in $RUN_DIR/run_meta/autopilot.json (see above); the flight goes on"
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
    taken_over && handed_over
    finish; exit 1
  fi
fi

if [ -n "$JETSON" ]; then
  # --jetson: the team stack runs in the container on the Jetson (or the virtual Jetson): link, container, tap, ready?
  if ! hil_start; then taken_over && handed_over; finish; exit 1; fi
elif [ "$TEAM_STACK_WHERE" = external ]; then
  # run.team_stack: external (a plug-in profile): the team stack runs in another container / on another PC.
  say "STEP 3/6  Waiting for the team's flight stack, which runs elsewhere (run.team_stack: external)"
  info "start it there now:  TRACKER_CMD=\"ros2 launch sim_bringup team_stack.launch.py detector:=$DETECTOR\" ./launcher.sh sim"
  info "(its MAVROS fcu_url must listen where network.mavros_out sends the MAVLink)"
  if ! ros2 run sim_bringup stack_check --timeout 600 --settle 20 "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}" "${NOMISSION[@]}"; then
    taken_over && handed_over
    warn "the team stack that runs elsewhere (the Jetson / another container) did not get ready in 10 minutes. Likely" \
         "cause: it was not started there, its MAVROS cannot reach the autopilot (the link: network.mavros_out / FCU_URL)," \
         "or its ROS_DOMAIN_ID differs from this PC's (${ROS_DOMAIN_ID:-0}), so this PC cannot see its nodes. Fix: start it" \
         "with the command printed above, check that  ros2 node list  on this PC shows /mavros, then run the demo again"
    finish; exit 1
  fi
else
  say "STEP 3/6  Starting the team's flight stack: ./launcher.sh sim (MAVROS, video_cam, new_od, trigger, main_controller)"
  info "log: $RUN_DIR/team_stack.log"
  [ -z "$TEAM_REPO" ] || info "team code: $TEAM_DIR (--team-repo; its launcher.sh and its ros2_ws)"
  [ -z "$PLUGIN_DETECTOR_TEXT" ] || info "plug-in profile: detector $PLUGIN_DETECTOR_TEXT"
fi
# --settle 20: older team code's main_controller sometimes died ~10 s after start (a parameter read that timed
# out was not handled; fixed in October 2026, see ros2_ws/src/main/README.md). The check and the restart stay as a
# safety net: if a team node stops early, the team stack is started once more before anything flies.
for attempt in 1 2; do
  [ "$TEAM_STACK_WHERE" != external ] || break
  ( cd "$TEAM_DIR" && TRACKER_CMD="$TRACKER" FCU_URL="$TEAM_FCU_URL" \
      exec setsid ./launcher.sh sim > >(stamp_lines >> "$RUN_DIR/team_stack.log") 2>&1 < /dev/null ) &
  TEAM_PG=$!
  if ros2 run sim_bringup stack_check --timeout 300 --settle 20 "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}" "${NOMISSION[@]}"; then
    break
  fi
  taken_over && handed_over        # (start over restarts the team stack itself: never start a second one)
  if [ "$attempt" = 1 ]; then
    warn "the team stack is not ready (a node stopped or never started): starting it once more. Its log so far:"
    grep -aE 'process has died|Error|ERROR' "$RUN_DIR/team_stack.log" | tail -6 | sed 's/^/    /'
    stop_group "$TEAM_PG" "team flight stack (launcher.sh)"
    echo "===== demo.sh: team stack restarted $(date +%T) =====" >> "$RUN_DIR/team_stack.log"
    continue
  fi
  warn "the team stack did not get ready, even after one restart. Likely cause: a team node crashes at start (the" \
       "'process has died' lines in the log), MAVROS cannot reach SITL, or the detector model does not load. Fix: read" \
       "$RUN_DIR/team_stack.log from the first ERROR; check the model with --detector truth. Last lines of its log:"
  tail -25 "$RUN_DIR/team_stack.log" | sed 's/^/    /'
  finish; exit 1
done
# What START OVER (start_over.sh, sim_demo run_reset.py) needs to start the team stack again EXACTLY like this.
printf 'SESSION=%s\nTRACKER_CMD=%s\nFCU_URL=%s\nSTACK_CHECK_ARGS=%s\nTEAM_STACK_WHERE=%s\n' "$ASTRA_SIM_SESSION" \
  "$TRACKER" "$TEAM_FCU_URL" "$(words "${NEED[@]}" "${PLUGIN_STACK_ARGS[@]}")" "$TEAM_STACK_WHERE" \
  > "$ASTRA_STATE_DIR/team_stack.env"
[ -z "$JETSON" ] || hil_stack_env     # (--jetson: TEAM_STACK_WHERE=jetson; START OVER is not supported with it)
# Which copy of the Jetson code flies (folder, git commit, launcher.sh fingerprint) -> run_meta/team_stack.json.
if [ "$TEAM_STACK_WHERE" = local ]; then       # (a stack elsewhere, e.g. on a Jetson, records itself)
  "${RUN_META[@]}" team-stack --out "$RUN_DIR/run_meta/team_stack.json" --repo "$TEAM_DIR" --tracker-cmd "$TRACKER" \
      --fcu-url "$TEAM_FCU_URL" 2>&1 | sed 's/^/    /' \
    || warn "could not record the team code in $RUN_DIR/run_meta/team_stack.json (see above); the flight goes on"
fi
# The camera emulator already starts looking straight down; say it again through the team's own camera
# command (video_cam -> SIYI SDK 0x0E), as the operator would before a survey.
if [ -n "$JETSON" ]; then hil_look_down     # (typed inside the container on the Jetson, as the stack's own user)
else timeout 15 ros2 topic pub -w 1 --once /camera/command std_msgs/msg/String "{data: 'gimbal_set_angles 0,-90'}" >/dev/null 2>&1 || true; fi

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
    taken_over && handed_over
    warn "the mission did not start (mission_tool's reason is above). Likely cause: the autopilot refused to arm (a" \
         "'PreArm:' text above says why) or it did not switch to AUTO. Fix: bash \"$ASTRA_SCRIPTS_DIR/stop_sim.sh\", then run" \
         "demo.sh again; a mission problem shows with  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" check \"$MISSION\""
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
    warn "the simulator stopped unexpectedly. Likely cause: Gazebo or SITL crashed, or its window was closed. Fix: read" \
         "the end of $RUN_DIR/sim.log, then  bash \"$ASTRA_SCRIPTS_DIR/stop_sim.sh\"  and run demo.sh again"; RESULT=1; break
  fi
  # --jetson: a long link drop silences the tap (and the phases); then the landing is also watched over MAVLink
  if [ -n "$JETSON" ] && ! hil_tick; then kill -INT "$MON_PID" 2>/dev/null; RESULT=0; break; fi
  sleep 2
done
# START OVER stopped the phase monitor to watch the NEW run itself: not an error, and no report to wait for.
taken_over && handed_over
case "$RESULT" in
  0) ok "The drone has landed: mission complete." ;;
  3) warn "The demo timed out after $TIMEOUT_MIN minutes (the drone had not landed). Likely cause: the drone got stuck" \
          "(a hold that never ended, a mode change from a ground station) or nobody started the flight (--manual)." \
          "Fix: look at the last phases above / in $RUN_DIR/phases.jsonl; give more time with --timeout MINUTES" ;;
  *) warn "The phase monitor ended with code $RESULT (not a normal landing). Consequence: the score report may be" \
          "incomplete. Fix: read $RUN_DIR/phases.jsonl and $RUN_DIR/sim.log" ;;
esac
# score_run writes the report about 8 s after the landing; give it a moment.
for _ in $(seq 1 30); do [ -s "$RUN_DIR/score_report.md" ] && break; sleep 1; done
finish
exit "$RESULT"
