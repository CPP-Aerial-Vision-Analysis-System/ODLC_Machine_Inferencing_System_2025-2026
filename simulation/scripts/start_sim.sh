#!/usr/bin/env bash
# =============================================================================
#  start_sim.sh  --  start the ASTRA simulator (Gazebo + drone + SITL + camera emulator + windows)
#
#  PURPOSE: the everyday way to start the simulator WITHOUT the team's flight stack. It loads the
#  settings (setup/sim_env.sh), makes sure the camera address 192.168.144.25 exists on this PC, stops a
#  simulator that is still running from before, and runs
#      ros2 launch sim_bringup sim.launch.py world:=<world> [your extra name:=value settings]
#  You then start the team's code yourself in a second terminal (./launcher.sh sim), press the photo
#  button (take_photo.sh), upload/fly a mission (upload_mission.sh, fly_mission.sh), ...
#  For the complete automatic search demo use demo.sh instead.
#
#  HOW TO RUN:   bash start_sim.sh                         -> runway world (quick test field)
#                bash start_sim.sh suas_field              -> the SUAS 2026 competition field
#                bash start_sim.sh runway camera_model:=a8mini rviz:=false
#                bash start_sim.sh runway --background     -> start and give the terminal back
#                bash start_sim.sh runway --headless       -> no windows (= gui:=false rviz:=false panel:=false)
#                bash start_sim.sh runway --no-windows-gcs -> no MAVLink copy for Mission Planner on Windows
#  Plug and play (docs/simulation/PLUGINS.md):
#                bash start_sim.sh --profile ardupilot_version     -> a plug-in profile (simulation/profiles/*.yaml
#                                                                     or a path): autopilot, camera, world, extras ...
#                bash start_sim.sh --profile mine.yaml --set camera.strict=true   -> change one setting
#                bash start_sim.sh --profile mine.yaml --dry-run   -> print the plan and the command, start nothing
#                Settings you type (name:=value, the world, --headless) win over the profile. The profile also
#                prints the matching command for the team stack in the second terminal.
#  Mission Planner / QGroundControl on WINDOWS can watch the flight: inside WSL, MAVProxy also sends the MAVLink
#  to the Windows host (in Mission Planner: connection type UDP, CONNECT, port 14550). The log line
#  "[sim] Windows ground station" shows the address; another one: windows_gcs_addr:=IP:PORT (see README.md).
#  Like demo.sh, it starts the camera emulator with camera_max_zoom:=1 (photos keep the full 81 degree view the
#  survey missions are planned for, although video_cam asks for 5x); give camera_max_zoom:=30 for the real ZR10.
#  A new simulator run starts with an EMPTY team photo folder: photos of an earlier run are moved to
#  ~/.astra_sim/photo_archive/<time>/ (new_od would process them again). --keep-photos leaves them.
#  Windows that open: Gazebo (3D view following the drone + live camera), RViz, the demo panel.
#  To stop: Ctrl+C here (or bash stop_sim.sh). The log is in ~/.astra_sim/logs/.
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

usage() { astra_usage "$0"; }
WORLD="runway"
BACKGROUND=0
CAMERA_IP_STEP=1
KEEP_PHOTOS=0
EXTRA=()
FIRST=1
# Plug and play: PLUGIN=1 when --profile or --set is given (PLUGIN=0 = exactly the old behaviour).
PROFILE=""; SETS=(); PLUGIN=0; DRY=0; NEXT=""
for a in "$@"; do
  if [ -n "$NEXT" ]; then                      # the value of --profile / --set
    case "$NEXT" in profile) PROFILE="$a" ;; set) SETS+=("$a") ;; esac
    NEXT=""; continue
  fi
  case "$a" in
    -h|--help) usage; exit 0 ;;
    --background|-b) BACKGROUND=1 ;;
    --headless) EXTRA+=("gui:=false" "rviz:=false" "panel:=false") ;;
    --keep-photos) KEEP_PHOTOS=1 ;;
    --no-camera-ip) CAMERA_IP_STEP=0 ;;
    --no-windows-gcs) EXTRA+=("windows_gcs:=false") ;;
    --profile) NEXT=profile; PLUGIN=1 ;;
    --set) NEXT=set; PLUGIN=1 ;;
    --dry-run) DRY=1 ;;
    *:=*) EXTRA+=("$a") ;;
    -*) die "unknown option $a (see --help)" ;;
    *) if [ "$FIRST" = 1 ]; then WORLD="$a"; FIRST=0; else die "unexpected argument \"$a\" (settings look like name:=value)"; fi ;;
  esac
done
[ -z "$NEXT" ] || die "--$NEXT needs a value (see --help)"
for s in "${SETS[@]}"; do [[ "$s" == *=* ]] || die "--set needs KEY=VALUE, got \"$s\" (for example --set camera.model=a8mini)"; done

astra_load_env
astra_require_built

# ---- plug-in profile (only with --profile / --set) -----------------------------------------------------
# profile_tool checks the profile and prints WORLD, PLUGIN_SIM_ARGS (the sim.launch.py arguments it decides, never
# one you typed), PLUGIN_SUMMARY (slot by slot) and PLUGIN_TEAM_HINT (the team-stack command for terminal 2).
PLUGIN_SIM_ARGS=(); PLUGIN_SUMMARY=""; PLUGIN_TEAM_HINT=""; PLUGIN_CAMERA_IP=""
if [ "$PLUGIN" = 1 ]; then
  TYPED_NAMES=""
  for a in "${EXTRA[@]}"; do TYPED_NAMES="${TYPED_NAMES:+$TYPED_NAMES,}${a%%:=*}"; done
  PT_ARGS=(shell --for start_sim --typed "$TYPED_NAMES")
  [ -z "$PROFILE" ] || PT_ARGS+=(--profile "$PROFILE")
  for s in "${SETS[@]}"; do PT_ARGS+=(--set "$s"); done
  [ "$FIRST" = 1 ] || PT_ARGS+=(--set "world.name=$WORLD")
  [ "$DRY" = 0 ] || PT_ARGS+=(--no-write)
  PLUGIN_VARS="$(python3 -m sim_plugins.profile_tool "${PT_ARGS[@]}")" \
    || die "the plug-in profile cannot be used (the reasons are above; sim_plugins not built? run setup/03_build.sh)"
  eval "$PLUGIN_VARS"
  for a in "${PLUGIN_SIM_ARGS[@]}"; do
    case "$a" in camera_ip:=*) PLUGIN_CAMERA_IP="${a#camera_ip:=}" ;; esac
  done
  EXTRA=("${PLUGIN_SIM_ARGS[@]}" "${EXTRA[@]}")
fi

case "$WORLD" in
  runway|suas_field|standin) ;;
  *.sdf|*.yaml|*.yml) [ -f "$WORLD" ] || die "world file not found: $WORLD"; WORLD="$(readlink -f "$WORLD")" ;;
  *) warn "\"$WORLD\" is not one of runway / suas_field; trying it as a world name from sim_worlds/config" ;;
esac

emulator_on=1
zoom_given=0
for a in "${EXTRA[@]}"; do
  case "$a" in
    emulator:=false|emulator:=0) emulator_on=0 ;;
    camera_max_zoom:=*) zoom_given=1 ;;
  esac
done
# demo.sh's default "--zoom survey": the photos keep the full view the survey lanes are planned for.
[ "$zoom_given" = 1 ] || EXTRA+=("camera_max_zoom:=1")
CMD=(ros2 launch sim_bringup sim.launch.py "world:=$WORLD" "${EXTRA[@]}")

if [ "$DRY" = 1 ]; then
  say "DRY RUN: this is what would run (nothing was started)"
  [ -z "$PLUGIN_SUMMARY" ] || printf '%s\n' "$PLUGIN_SUMMARY" | sed 's/^/    /'
  echo "    1. ${CMD[*]}"
  # Step 2 is only printed (the team stack is started by hand): README.md's "OR BY HAND" command. A profile prints
  # the same command plus what it changes (sim_plugins plan.team_stack_hint; the default profile adds nothing).
  if [ -n "$PLUGIN_TEAM_HINT" ]; then
    echo "    2. (second terminal) cd $ASTRA_REPO && $PLUGIN_TEAM_HINT"
  else
    echo "    2. (second terminal) cd $ASTRA_REPO && TRACKER_CMD=\"ros2 launch sim_bringup team_stack.launch.py\" ./launcher.sh sim"
  fi
  exit 0
fi

# A simulator that is still running would block the ports: stop ours first.
if [ -s "$ASTRA_STATE_DIR/session" ] && [ -n "$(astra_session_pids "$(cat "$ASTRA_STATE_DIR/session")")" ]; then
  say "A simulator from before is still running: stopping it first"
  bash "$ASTRA_SCRIPTS_DIR/stop_sim.sh" --quiet || warn "something from the old run is still alive (see above)"
fi

astra_mem_check
if [ "$KEEP_PHOTOS" = 0 ]; then
  astra_archive_photos "$(python3 -m sim_bringup.sim_paths photo-dir 2>/dev/null)"
fi
if [ "$CAMERA_IP_STEP" = 1 ] && [ "$emulator_on" = 1 ]; then
  astra_ensure_camera_ip "${PLUGIN_CAMERA_IP:-$ASTRA_CAMERA_IP}" || warn "continuing without the camera address: video_cam will not reach the camera emulator"
fi

astra_new_session sim
LOG="$ASTRA_SIM_HOME/logs/sim_$(date +%Y%m%d-%H%M%S).log"
printf 'SESSION=%s\nWORLD=%s\nLOG=%s\nSTARTED=%s\nCOMMAND=%s\n' "$ASTRA_SIM_SESSION" "$WORLD" "$LOG" "$(date)" \
  "${CMD[*]}" > "$ASTRA_STATE_DIR/start_sim.env"

say "Starting the ASTRA simulator"
info "world:    $WORLD"
info "settings: ${EXTRA[*]:-(defaults)}"
info "log:      $LOG"
info "Gazebo partition $GZ_PARTITION, ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-0}, session $ASTRA_SIM_SESSION"
info "The first start takes 1-2 minutes (3D models load, SITL boots; wait ~30 s more before arming)."
if [ "$PLUGIN" = 1 ]; then
  info "plug-in profile, slot by slot:"
  printf '%s\n' "$PLUGIN_SUMMARY" | sed 's/^/      /'
  info "team stack for the second terminal:  cd $ASTRA_REPO && $PLUGIN_TEAM_HINT"
fi

if [ "$BACKGROUND" = 1 ]; then
  setsid "${CMD[@]}" > "$LOG" 2>&1 < /dev/null &
  echo "PGID=$!" >> "$ASTRA_STATE_DIR/start_sim.env"
  ok "started in the background (pid $!). Watch it:  tail -f $LOG"
  info "Stop it with:  bash $ASTRA_SCRIPTS_DIR/stop_sim.sh"
  exit 0
fi

info "Press Ctrl+C here to stop everything."
echo
# The launch runs in the FOREGROUND, so Ctrl+C reaches it directly and it shuts everything down in order.
# `tee -i` keeps writing the log while that happens; stop_sim.sh then sweeps up anything left over.
trap 'echo; say "Stopping the simulator (this takes a few seconds) ..."' INT TERM
"${CMD[@]}" > >(tee -i "$LOG") 2>&1
trap - INT TERM
bash "$ASTRA_SCRIPTS_DIR/stop_sim.sh" --quiet
say "The simulator has stopped. Log: $LOG"
