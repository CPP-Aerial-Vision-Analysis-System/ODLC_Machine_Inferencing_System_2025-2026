# shellcheck shell=bash
# =============================================================================
#  _container_common.sh  --  helpers for the scripts that run INSIDE the ASTRA containers (sourced, never run)
#
#  PURPOSE: the steps both containers of docker/compose.sim.yml do before they start their real job, written
#  once so "sim" (container_sim.sh) and "jetson" (container_jetson.sh) behave the same way:
#    * friendly messages (say / info / ok / warn / die), like the other simulation scripts;
#    * fix_ownership: make mounted folders writable for the container user "ubuntu" (Docker creates a brand-new
#      volume, or a missing host folder, owned by root; the image's user has password-free sudo for this);
#    * build_workspace: build the repo's ros2_ws with colcon INSIDE the container, into that container's own
#      Docker volumes (ros2_ws/build, install, log), only when needed (never built, or a source file changed),
#      with few parallel jobs so a laptop stays usable;
#    * my_ip: this container's address on the drone network (192.168.144.x);
#    * fetch_fuel_models: download the Gazebo Fuel 3D models the worlds use, once (kept in a volume).
#  Used by container_sim.sh ("sim"), container_jetson.sh ("jetson") and container_solo.sh (one container).
#  Settings (environment variables, set by docker/compose.sim.yml or the run_*_container.sh scripts):
#    ASTRA_REPO        the repo inside the container (default /ODLC_Machine_Inferencing_System_2025-2026)
#    ASTRA_BUILD       auto (default) | always | never | clean   when to run colcon (clean = wipe and rebuild)
#    ASTRA_BUILD_JOBS  packages / compiler jobs at the same time (default 4)
# =============================================================================

export ASTRA_REPO="${ASTRA_REPO:-/ODLC_Machine_Inferencing_System_2025-2026}"
export ASTRA_SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"

say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '\033[1;32m OK %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m !! %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31m XX %s\033[0m\n' "$*" >&2; exit 1; }

# my_ip : this container's IPv4 address on its first network (e.g. 192.168.144.25), or "?".
my_ip() {
  local ip
  ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  printf '%s\n' "${ip:-?}"
}

# fix_ownership DIR... : create each folder if needed and make it writable for the current user.
fix_ownership() {
  local d
  for d in "$@"; do
    [ -d "$d" ] || mkdir -p "$d" 2>/dev/null || sudo -n mkdir -p "$d" 2>/dev/null
    if [ ! -w "$d" ]; then
      if sudo -n chown "$(id -u):$(id -g)" "$d" 2>/dev/null; then
        info "made $d writable for $(id -un)"
      else
        warn "cannot write to $d (and sudo did not help): things stored there will fail"
      fi
    fi
  done
}

# workspace_needs_build STAMP : true when ros2_ws was never built here or a source file is newer than STAMP.
# Photos, detector output and Python caches inside src/ do not count (they change on every flight).
workspace_needs_build() {
  local stamp="$1" ws="$ASTRA_REPO/ros2_ws"
  [ -f "$ws/install/setup.bash" ] || return 0
  [ -f "$stamp" ] || return 0
  [ -n "$(find "$ws/src" \( -name mapping_photos -o -name detection_results_sahi -o -name camera_feed \
            -o -name __pycache__ -o -name .pytest_cache -o -name '*.egg-info' \) -prune \
            -o -type f -newer "$stamp" -print -quit 2>/dev/null)" ]
}

# build_workspace NAME : colcon build of ros2_ws for container NAME ("sim" or "jetson"), when needed.
# Same build as simulation/setup/03_build.sh (all packages except the unused vendored ultralytics_ros,
# --symlink-install), but into this container's volumes, so it never mixes with a native build.
build_workspace() {
  local name="$1" ws="$ASTRA_REPO/ros2_ws" jobs="${ASTRA_BUILD_JOBS:-4}" mode="${ASTRA_BUILD:-auto}"
  local stamp="$ws/install/.astra_docker_build_stamp" log rc
  [ -d "$ws/src" ] || die "no ros2_ws/src in $ASTRA_REPO: is the repo mounted? (docker/compose.sim.yml mounts it)"
  case "$mode" in
    never|no|0)
      [ -f "$ws/install/setup.bash" ] || die "ASTRA_BUILD=never, but ros2_ws was never built in this container"
      info "ASTRA_BUILD=never: using the existing build"
      return 0 ;;
    clean)
      say "ASTRA_BUILD=clean: deleting this container's old build (build/ install/ log/)"
      # The three folders are Docker volume mount points: empty them, never remove them.
      find "$ws/build" "$ws/install" "$ws/log" -mindepth 1 -maxdepth 1 -exec rm -rf {} + 2>/dev/null ;;
    always|auto) ;;
    *) warn "unknown ASTRA_BUILD=$mode (use auto, always, never or clean); using auto"; mode=auto ;;
  esac
  if [ "$mode" = auto ] && ! workspace_needs_build "$stamp"; then
    ok "ros2_ws is already built in this container and nothing changed (ASTRA_BUILD=always forces a build)"
    return 0
  fi
  mkdir -p "$ASTRA_SIM_HOME/logs"
  log="$ASTRA_SIM_HOME/logs/docker_build_${name}_$(date +%Y%m%d-%H%M%S).log"
  say "Building ros2_ws inside the $name container ($jobs jobs; the first time takes a few minutes)"
  info "log: $log   (inside the container; on the host: <ASTRA_DOCKER_DATA>/logs/)"
  (
    cd "$ws" || exit 1
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    export MAKEFLAGS="-j$jobs"
    nice -n 10 colcon build --symlink-install --parallel-workers "$jobs" --packages-ignore ultralytics_ros \
      --event-handlers console_cohesion- summary+
  ) > "$log" 2>&1
  rc=$?
  grep -aE '^(Summary|  [0-9]+ packages?)|Failed|Aborted' "$log" | sed 's/^/    /'
  if [ "$rc" -ne 0 ]; then
    warn "colcon reported failures (exit $rc); details in $log"
    [ -f "$ws/install/setup.bash" ] || die "nothing usable was built; see $log"
    return 1
  fi
  touch "$stamp"
  ok "ros2_ws built"
}

# fetch_fuel_models : the worlds use a few 3D models from Gazebo Fuel (the internet model library). Download
# them once (about 110 MB) into ~/.gz/fuel, which compose.sim.yml keeps in a volume or the host's ~/.gz/fuel.
# Needs ros2_ws built (sim_worlds) and internet; without them the world loads with fewer details.
fetch_fuel_models() {
  local fetch="$ASTRA_REPO/ros2_ws/src/simulation/sim_worlds/scripts/fetch_models.sh"
  [ -f "$fetch" ] || return 0
  bash "$fetch" --check >/dev/null 2>&1 && return 0
  say "Downloading the Gazebo Fuel 3D models the worlds use (about 110 MB, only the first time)"
  timeout 1800 bash "$fetch" || warn "some Fuel models could not be downloaded (internet?): the world loads with fewer details"
}
