#!/usr/bin/env bash
# =============================================================================
#  compose_down.sh  --  stop and remove the ASTRA simulator containers (your builds and results stay)
#
#  PURPOSE: the clean "off switch" for compose_up.sh (and for the one-container scripts):
#    1. asks the "sim" container to stop: it writes the score report, copies the photos and shuts the simulator
#       down in order (it gets up to 45 s); the "jetson" container stops the team's code the same way;
#    2. removes both containers and the astra_net network (the Docker copy of the drone's 192.168.144.x net);
#    3. --solo:            also stops the containers of run_sim_container.sh and run_jetson_container.sh;
#    4. --wipe-builds:     also deletes the Docker volumes with the containers' colcon builds (the next start
#                          builds ros2_ws again; use it when a build is broken in a strange way);
#       --wipe-fuel:       also deletes the downloaded Gazebo 3D models volume (astra_gz_fuel);
#    5. --restore-camera-ip: gives this computer the camera address 192.168.144.25 back on its loopback, which
#                          compose_up.sh took away (the native start_sim.sh / demo.sh also do that by themselves).
#  Never deleted here: your runs and logs (~/.astra_docker) and the images (docker image rm astra-sim:latest ...).
#
#  HOW TO RUN:   bash simulation/docker/compose_down.sh
#  Options:      --solo  --wipe-builds  --wipe-fuel  --restore-camera-ip  --quiet  --dry-run
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_host_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_host_common.sh"

SOLO=0; WIPE_BUILDS=0; WIPE_FUEL=0; RESTORE_IP=0; QUIET=0
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --solo) SOLO=1 ;;
    --wipe-builds) WIPE_BUILDS=1 ;;
    --wipe-fuel) WIPE_FUEL=1 ;;
    --restore-camera-ip) RESTORE_IP=1 ;;
    --quiet|-q) QUIET=1 ;;
    --dry-run) ASTRA_DRY=1 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done
astra_need_docker
[ "$QUIET" = 1 ] || say "Stopping the ASTRA simulator containers (the sim container may need up to 45 s to finish cleanly)"

# The project name "astra" finds the containers; the base file is enough for "down".
astra_run docker compose -p astra -f "$ASTRA_REPO_HERE/docker/compose.sim.yml" down --timeout 45 --remove-orphans \
  || warn "docker compose down reported a problem (see above)"

if [ "$SOLO" = 1 ]; then
  for c in astra-sim-solo astra-jetson-solo; do
    if [ "$ASTRA_DRY" = 1 ] || docker ps -q --filter "name=^$c\$" | grep -q .; then
      [ "$QUIET" = 1 ] || info "stopping $c"
      astra_run docker stop -t 45 "$c" >/dev/null || true
    fi
  done
fi

if [ "$WIPE_BUILDS" = 1 ]; then
  [ "$QUIET" = 1 ] || info "deleting the colcon build volumes (the next start builds ros2_ws again)"
  for v in astra_sim_ws_build astra_sim_ws_install astra_sim_ws_log astra_jetson_ws_build astra_jetson_ws_install astra_jetson_ws_log; do
    astra_run docker volume rm -f "$v" >/dev/null || true
  done
fi
if [ "$WIPE_FUEL" = 1 ]; then
  [ "$QUIET" = 1 ] || info "deleting the Gazebo 3D model volume astra_gz_fuel"
  astra_run docker volume rm -f astra_gz_fuel >/dev/null || true
fi

if [ "$RESTORE_IP" = 1 ] && ! astra_camera_ip_on_host; then
  [ "$QUIET" = 1 ] || info "giving this computer the camera address $ASTRA_CAMERA_IP back (for the NATIVE simulator)"
  if [ "$ASTRA_DRY" = 1 ]; then
    astra_run sudo ip addr add "$ASTRA_CAMERA_IP/32" dev lo
  else
    { sudo -n ip addr add "$ASTRA_CAMERA_IP/32" dev lo 2>/dev/null; } \
      || { [ -n "${WSL_DISTRO_NAME:-}" ] && timeout 60 wsl.exe -d "$WSL_DISTRO_NAME" -u root -- ip addr add "$ASTRA_CAMERA_IP/32" dev lo >/dev/null 2>&1; } \
      || { [ -t 0 ] && sudo ip addr add "$ASTRA_CAMERA_IP/32" dev lo; } \
      || true
    if astra_camera_ip_on_host; then ok "camera address back"; rm -f "$ASTRA_DOCKER_DATA/state/native_camera_ip_removed"
    else warn "could not add it; the native start_sim.sh / demo.sh will try again when they start"; fi
  fi
fi

if [ "$QUIET" = 0 ]; then
  if [ "$ASTRA_DRY" = 0 ] && docker network inspect "$ASTRA_NET" >/dev/null 2>&1; then
    warn "the network $ASTRA_NET still exists (a container started by run_jetson_container.sh still uses it?)"
    info "see: docker network inspect $ASTRA_NET   |   stop those: bash $0 --solo"
  fi
  ok "stopped. Your runs and logs are still in $ASTRA_DOCKER_DATA"
fi
