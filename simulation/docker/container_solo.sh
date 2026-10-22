#!/usr/bin/env bash
# =============================================================================
#  container_solo.sh  --  runs INSIDE the one-container setup: build, then the normal demo.sh
#
#  PURPOSE: what `simulation/docker/run_sim_container.sh --demo ...` runs in its container (image astra-sim, which
#  holds the simulator AND the team's software). Everything happens in this one container, exactly like the
#  native setup of simulation/README.md:
#    1. makes the mounted folders writable for the container user (fresh Docker volumes belong to root);
#    2. builds ros2_ws in the container's own volumes when needed (_container_common.sh; same volumes as the
#       "sim" container of docker/compose.sim.yml, which builds the same packages);
#    3. downloads the Gazebo Fuel 3D models the worlds use, once;
#    4. runs simulation/scripts/demo.sh with your options. demo.sh adds the camera address 192.168.144.25 to
#       THIS CONTAINER's loopback (password-free sudo + the NET_ADMIN right the container was given), starts
#       Gazebo, SITL, the camera emulator, the windows, the team's ./launcher.sh sim, flies the mission and
#       writes the score report into ~/.astra_sim/runs (= ~/.astra_docker/runs on the host).
#  HOW TO RUN: not by hand; from the host:  bash simulation/docker/run_sim_container.sh --demo runway [options]
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source-path=SCRIPTDIR source=_container_common.sh
source "$HERE/_container_common.sh"

say "ASTRA one-container simulator (address $(my_ip), user $(id -un))"
fix_ownership "$ASTRA_REPO/ros2_ws/build" "$ASTRA_REPO/ros2_ws/install" "$ASTRA_REPO/ros2_ws/log" \
  "$ASTRA_SIM_HOME" "$ASTRA_SIM_HOME/models" "$HOME/.gz" "$HOME/.gz/fuel" "$HOME/.config"
build_workspace solo || warn "continuing with what was built (demo.sh lists what is missing)"
# shellcheck disable=SC1091
source "$ASTRA_REPO/simulation/setup/sim_env.sh"
fetch_fuel_models
say "simulation/scripts/demo.sh $*"
exec bash "$ASTRA_REPO/simulation/scripts/demo.sh" "$@"
