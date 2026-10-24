#!/usr/bin/env bash
# =============================================================================
#  run_jetson_container.sh  --  the Jetson stand-in on its own: the team's dev image (= the Jetson's software)
#
#  PURPOSE: a container with exactly the Jetson's software (image astra-dev: Ubuntu 22.04, ROS 2 Humble, MAVROS,
#  the Python ML stack, mavlink-routerd; the same versions as the real Jetson), with the repo mounted, so code
#  tested here runs on the drone unchanged. Three ways to connect it (--net):
#    astra   the Docker copy of the drone network: it gets 192.168.144.100 (the real Jetson's address) next to
#            a simulator started with  compose_up.sh --only sim  (the camera emulator at 192.168.144.25). Same as
#            the compose "jetson" container, but you are at the keyboard (start ./launcher.sh sim yourself,
#            restart one node, debug). Default when the astra_net network exists.
#    host    this computer's network: the team's code (in the Jetson's software) talks to the NATIVE simulator
#            (simulation/scripts/start_sim.sh): MAVROS gets MAVProxy's UDP 14552 of this computer, and the
#            camera emulator is on this computer's 192.168.144.25 loopback address.
#    bridge  Docker's normal network, no simulator: build and test code (colcon, pytest, ruff). Default otherwise.
#  What runs in it:
#    (default)    a terminal inside the container
#    --launcher   what the compose "jetson" container does: build ros2_ws, wait for the simulator's MAVLink,
#                 then ./launcher.sh sim (simulation/docker/container_jetson.sh)
#    -- CMD...    one command, e.g.  -- bash -c "cd ros2_ws && colcon test --packages-select main"
#
#  HOW TO RUN (inside the Ubuntu-22.04 WSL distro, or on Linux; image built with build_images.sh dev):
#      bash simulation/docker/run_jetson_container.sh                         terminal (astra or bridge network)
#      bash simulation/docker/compose_up.sh --only sim --manual   then   bash simulation/docker/run_jetson_container.sh --net astra --launcher
#      bash simulation/docker/run_jetson_container.sh --net host --launcher   against the native simulator
#  Options:  --net astra|host|bridge   --launcher   --detector real|truth   --no-gpu   --repo DIR
#            --name NAME (default astra-jetson-solo)   --dry-run
#  Inside the terminal, build once:  source /opt/ros/humble/setup.bash && cd ros2_ws && \
#      colcon build --symlink-install --parallel-workers 4 --packages-ignore ultralytics_ros && cd .. && ./launcher.sh sim
#  Stop: Ctrl+C / exit; or from another terminal  bash simulation/docker/compose_down.sh --solo
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_host_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_host_common.sh"

NET=auto; MODE=shell; GPU=auto; REPO_ARG=""; NAME=astra-jetson-solo; DETECTOR="${ASTRA_DETECTOR:-real}"; PASS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --net) shift; NET="${1:-}" ;;
    --launcher) MODE=launcher ;;
    --detector) shift; DETECTOR="${1:-}" ;;
    --no-gpu) GPU=no ;;
    --repo) shift; REPO_ARG="${1:-}" ;;
    --name) shift; NAME="${1:-astra-jetson-solo}" ;;
    --dry-run) ASTRA_DRY=1 ;;
    --) MODE=cmd; shift; PASS=("$@"); break ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done
case "$DETECTOR" in real|truth) ;; *) die "--detector must be real or truth" ;; esac
[ "$MODE" = cmd ] && [ "${#PASS[@]}" -eq 0 ] && die "-- needs a command after it"

astra_need_docker --no-compose
astra_require_images "astra-dev:latest dev"
if [ "$ASTRA_DRY" = 0 ]; then
  docker ps -q --filter "name=^$NAME\$" | grep -q . && die "a container named $NAME is already running (stop it: docker stop $NAME)"
  # Same build volumes and the same address as the compose "jetson" container: only one of them at a time.
  docker ps -q --filter 'name=^astra-jetson$' | grep -q . && die "the compose Jetson stand-in (astra-jetson) is running; stop it first (compose_down.sh), or use compose_up.sh --only sim"
fi
if [ "$NET" = auto ]; then
  if [ "$ASTRA_DRY" = 0 ] && docker network inspect "$ASTRA_NET" >/dev/null 2>&1; then NET=astra; else NET=bridge; fi
fi
case "$NET" in
  astra)
    [ "$ASTRA_DRY" = 1 ] || docker network inspect "$ASTRA_NET" >/dev/null 2>&1 \
      || die "the network $ASTRA_NET does not exist yet: start the simulator first:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --only sim --manual"
    NET_ARGS=(--network "$ASTRA_NET" --ip "$ASTRA_JETSON_IP" --hostname jetson)
    NET_TEXT="drone network $ASTRA_NET at $ASTRA_JETSON_IP (camera / simulator at $ASTRA_CAMERA_IP)" ;;
  host)
    NET_ARGS=(--network host)
    NET_TEXT="this computer's network (native simulator: MAVROS UDP 14552, camera $ASTRA_CAMERA_IP on the loopback)"
    astra_camera_ip_on_host || warn "this computer has no $ASTRA_CAMERA_IP yet: the native start_sim.sh / demo.sh adds it" ;;
  bridge)
    NET_ARGS=(--hostname jetson)
    NET_TEXT="Docker's normal network (no simulator: build and test only)" ;;
  *) die "--net must be astra, host or bridge" ;;
esac
REPO_HOST="$(astra_pick_repo "$REPO_ARG")" || exit 1
[ -f "$REPO_HOST/simulation/docker/_container_common.sh" ] \
  || die "$REPO_HOST has no simulation/docker/_container_common.sh (an old copy): refresh it with  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only"
astra_prepare_data

DISPLAY_MODE="$(astra_display_mode)"
astra_gui_run_args "$DISPLAY_MODE"
astra_gpu_run_args "$GPU"
[ "$DISPLAY_MODE" = x11 ] && astra_x11_allow

R="$ASTRA_CONTAINER_REPO"
# Before a terminal or a command: make the fresh (root-owned) build volumes writable for the container user.
PREP='source simulation/docker/_container_common.sh && fix_ownership ros2_ws/build ros2_ws/install ros2_ws/log "$ASTRA_SIM_HOME" >/dev/null; exec "$@"'
RUN=(docker run --rm --name "$NAME" --init --ipc host "${NET_ARGS[@]}"
     -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" -e ROS_LOCALHOST_ONLY=0 -e "ASTRA_REPO=$R" -e PYTHONUNBUFFERED=1
     -e "ASTRA_DETECTOR=$DETECTOR" -e "ASTRA_BUILD=${ASTRA_BUILD:-auto}" -e "ASTRA_BUILD_JOBS=${ASTRA_BUILD_JOBS:-4}"
     -v "$REPO_HOST:$R" -w "$R"
     -v astra_jetson_ws_build:"$R/ros2_ws/build" -v astra_jetson_ws_install:"$R/ros2_ws/install" -v astra_jetson_ws_log:"$R/ros2_ws/log"
     -v "$ASTRA_DOCKER_DATA:/home/ubuntu/.astra_sim"
     "${GUI_ARGS[@]}" "${GPU_ARGS[@]}")
[ -t 0 ] && RUN+=(-it)
case "$MODE" in
  launcher) RUN+=(astra-dev:latest bash "$R/simulation/docker/container_jetson.sh") ;;
  cmd)      RUN+=(astra-dev:latest bash -c "$PREP" _ "${PASS[@]}") ;;
  shell)
    say "A terminal inside the Jetson stand-in ($NAME). Type  exit  to leave."
    info "build once:  source /opt/ros/humble/setup.bash && cd ros2_ws && colcon build --symlink-install --parallel-workers 4 --packages-ignore ultralytics_ros && cd .."
    info "then:        TRACKER_CMD=\"ros2 launch sim_bringup team_stack.launch.py detector:=$DETECTOR\" ./launcher.sh sim"
    RUN+=(astra-dev:latest bash -c "$PREP" _ bash) ;;
esac
say "Jetson stand-in: $NET_TEXT"
info "repo $REPO_HOST -> $R | data $ASTRA_DOCKER_DATA | windows $DISPLAY_MODE | GPU ${GPU_ARGS[*]:-none}"
astra_run "${RUN[@]}"
