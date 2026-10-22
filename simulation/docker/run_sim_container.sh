#!/usr/bin/env bash
# =============================================================================
#  run_sim_container.sh  --  the WHOLE simulator + the team's code in ONE container (image astra-sim)
#
#  PURPOSE: the simplest Docker way to use the simulator: one container that is a complete copy of the native
#  setup of simulation/README.md (the astra-sim image holds ROS 2 Humble, Gazebo Harmonic, ArduCopter SITL,
#  MAVProxy AND the team's software), on Docker's default network. The camera address 192.168.144.25 is
#  added INSIDE the container (on its own loopback, so nothing changes on this computer). Windows appear on
#  the desktop (WSLg on Windows 11, X11 on Linux) and the graphics card is used (DirectX through /dev/dxg,
#  or /dev/dri, plus CUDA when the NVIDIA Container Toolkit is there).
#  For the "like the real drone" setup with a separate Jetson stand-in use compose_up.sh instead.
#
#  HOW TO RUN (inside the Ubuntu-22.04 WSL distro, or on Linux; image built with build_images.sh sim):
#      bash simulation/docker/run_sim_container.sh                 a terminal INSIDE the container: there
#                                                                   everything of simulation/README.md works
#                                                                   (repo at /ODLC_Machine_Inferencing_System_2025-2026)
#      bash simulation/docker/run_sim_container.sh --demo runway   build if needed, then the full search demo
#      bash simulation/docker/run_sim_container.sh --demo suas_field --detector truth --camera a8mini
#      bash simulation/docker/run_sim_container.sh -- gz sim --versions      run one command and leave
#  Options (before --demo / --):
#      --headless     no windows                 --no-gpu       no CUDA
#      --repo DIR     mount this repo folder (default: the Linux-side copy on Windows, else this repo)
#      --name NAME    container name (default astra-sim-solo)
#      --dry-run      print the docker command, run nothing
#  Everything after --demo goes to simulation/scripts/demo.sh (see its --help). Results: ~/.astra_docker/runs.
#  Inside the terminal, first time:  bash simulation/setup/03_build.sh   (builds ros2_ws into the container's volumes)
#  Stop: Ctrl+C (demo) / exit (terminal); or from another terminal  bash simulation/docker/compose_down.sh --solo
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_host_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_host_common.sh"

MODE=shell; HEADLESS=0; GPU=auto; REPO_ARG=""; NAME=astra-sim-solo; PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --demo) MODE=demo; shift; PASS=("$@"); break ;;
    --) MODE=cmd; shift; PASS=("$@"); break ;;
    --headless) HEADLESS=1 ;;
    --no-gpu) GPU=no ;;
    --repo) shift; REPO_ARG="${1:-}" ;;
    --name) shift; NAME="${1:-astra-sim-solo}" ;;
    --dry-run) ASTRA_DRY=1 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done
[ "$MODE" = cmd ] && [ "${#PASS[@]}" -eq 0 ] && die "-- needs a command after it"

astra_need_docker --no-compose
astra_require_images "astra-sim:latest sim"
if [ "$ASTRA_DRY" = 0 ]; then
  docker ps -q --filter "name=^$NAME\$" | grep -q . && die "a container named $NAME is already running (stop it: docker stop $NAME)"
  # Same build volumes as the compose "sim" container: never build into them twice at the same time.
  docker ps -q --filter 'name=^astra-sim$' | grep -q . && die "the compose simulator (astra-sim) is running; stop it first: bash $ASTRA_DOCKER_SCRIPTS/compose_down.sh"
fi
REPO_HOST="$(astra_pick_repo "$REPO_ARG")" || exit 1
[ -f "$REPO_HOST/simulation/docker/container_solo.sh" ] \
  || die "$REPO_HOST has no simulation/docker/container_solo.sh (an old copy): refresh it with  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only"
astra_prepare_data
astra_use_host_fuel

DISPLAY_MODE="$(astra_display_mode)"
[ "$HEADLESS" = 1 ] && DISPLAY_MODE=none
astra_gui_run_args "$DISPLAY_MODE"
astra_gpu_run_args "$DISPLAY_MODE" "$GPU"
[ "$DISPLAY_MODE" = x11 ] && astra_x11_allow
astra_mem_check 4000

R="$ASTRA_CONTAINER_REPO"
# Before a terminal or a command: make the fresh (root-owned) build volumes writable for the container user.
PREP='source simulation/docker/_container_common.sh && fix_ownership ros2_ws/build ros2_ws/install ros2_ws/log "$ASTRA_SIM_HOME" "$HOME/.gz/fuel" >/dev/null; exec "$@"'
RUN=(docker run --rm --name "$NAME" --hostname "$NAME" --init
     # NET_ADMIN: demo.sh / 03_build.sh add the camera address 192.168.144.25 to the container's own loopback.
     --cap-add NET_ADMIN
     # The host's shared memory: ROS 2 (Fast DDS) shared-memory transport, no 64 MB /dev/shm limit.
     --ipc host
     -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" -e "ASTRA_REPO=$R" -e PYTHONUNBUFFERED=1
     -e "ASTRA_BUILD=${ASTRA_BUILD:-auto}" -e "ASTRA_BUILD_JOBS=${ASTRA_BUILD_JOBS:-4}"
     -v "$REPO_HOST:$R" -w "$R"
     -v astra_sim_ws_build:"$R/ros2_ws/build" -v astra_sim_ws_install:"$R/ros2_ws/install" -v astra_sim_ws_log:"$R/ros2_ws/log"
     -v "$ASTRA_DOCKER_DATA:/home/ubuntu/.astra_sim"
     -v "${ASTRA_FUEL_DIR:-astra_gz_fuel}:/home/ubuntu/.gz/fuel"
     "${GUI_ARGS[@]}" "${GPU_ARGS[@]}")
[ -t 0 ] && RUN+=(-it)

case "$MODE" in
  demo)
    # demo.sh inside a container would send the Mission Planner copy to the container's gateway (this Ubuntu),
    # not to Windows: give it the Windows host unless an option about it was given.
    want_gcs=1
    for a in "${PASS[@]}"; do case "$a" in --windows-gcs|--no-windows-gcs) want_gcs=0 ;; esac; done
    if [ "$want_gcs" = 1 ]; then
      gcs="$(astra_windows_gcs)"
      if [ -n "$gcs" ]; then PASS+=(--windows-gcs "$gcs"); else PASS+=(--no-windows-gcs); fi
    fi
    [ "$DISPLAY_MODE" = none ] && PASS+=(--headless)
    say "One-container simulator: demo.sh ${PASS[*]}"
    RUN+=(astra-sim:latest bash "$R/simulation/docker/container_solo.sh" "${PASS[@]}") ;;
  cmd)
    RUN+=(astra-sim:latest bash -c "$PREP" _ "${PASS[@]}") ;;
  shell)
    say "A terminal inside the simulator container ($NAME). Type  exit  to leave."
    info "first time inside:  bash simulation/setup/03_build.sh      then: bash simulation/scripts/demo.sh runway"
    info "Mission Planner on Windows from inside: add  --windows-gcs $(astra_windows_gcs || true)  to demo.sh / start_sim.sh"
    RUN+=(astra-sim:latest bash -c "$PREP" _ bash) ;;
esac
info "repo $REPO_HOST -> $R | data $ASTRA_DOCKER_DATA | windows $DISPLAY_MODE | GPU ${GPU_ARGS[*]:-none}"
astra_run "${RUN[@]}"
