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
#      with few parallel jobs so a laptop stays usable; in the containers that run Gazebo it then also makes
#      the real-terrain world's Gazebo models when that world's generator is in the repo (build_world_assets);
#    * display_available / prepare_runtime_dir: is there a desktop to draw on (X11 or Wayland socket)?
#    * my_ip: this container's address on the drone network (192.168.144.x);
#    * fetch_fuel_models: download the Gazebo Fuel 3D models the worlds use, once (kept in a volume);
#    * the hand-over files the two containers share in the data folder (see "Hand-over files" below).
#  Used by container_sim.sh ("sim"), container_jetson.sh ("jetson") and container_solo.sh (one container).
#  Settings (environment variables, set by docker/compose.sim.yml or the run_*_container.sh scripts):
#    ASTRA_REPO        the repo inside the container (default /ODLC_Machine_Inferencing_System_2025-2026)
#    ASTRA_BUILD       auto (default) | always | never | clean   when to run colcon (clean = wipe and rebuild)
#    ASTRA_BUILD_JOBS  packages / compiler jobs at the same time (default 4)
#
#  Hand-over files: both containers mount the same data folder at ~/.astra_sim, so its state/ folder is how they
#  tell each other things (plain small files, no network needed):
#    state/docker_mission_ready          sim -> jetson  the survey mission is uploaded: start the team's code now
#                                                        (the same order as demo.sh: upload first, then launcher.sh)
#    state/docker_team_restart.request   sim -> jetson  "start the team's code again" (a token), like demo.sh's
#    state/docker_team_restart.done      jetson -> sim   second attempt when a team node stopped; .done = done
#    state/team_stack.env                sim -> START OVER  TEAM_STACK_WHERE=external: the team's code runs in the
#                                                        jetson container (run_reset must not start it in "sim")
#    state/start_over_progress.jsonl     START OVER -> jetson  its steps; the jetson container stops the team's
#                                                        code at "stop_team" and starts it again at "team"
# =============================================================================

export ASTRA_REPO="${ASTRA_REPO:-/ODLC_Machine_Inferencing_System_2025-2026}"
export ASTRA_SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"
ASTRA_STATE_DIR="$ASTRA_SIM_HOME/state"

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

# display_available : true when a desktop can be drawn on (the X11 socket of $DISPLAY or a Wayland socket).
display_available() {
  local n="${DISPLAY#*:}"
  n="${n%%.*}"                                  # DISPLAY=:0 or :0.0 -> the socket /tmp/.X11-unix/X0
  [ -n "${DISPLAY:-}" ] && [ -S "/tmp/.X11-unix/X$n" ] && return 0
  [ -n "${WAYLAND_DISPLAY:-}" ] && [ -S "${XDG_RUNTIME_DIR:-/nonexistent}/$WAYLAND_DISPLAY" ]
}

# prepare_runtime_dir : Qt wants a private runtime folder; a Linux desktop setup points it into /tmp.
prepare_runtime_dir() {
  case "${XDG_RUNTIME_DIR:-}" in
    /tmp/*) [ -d "$XDG_RUNTIME_DIR" ] || mkdir -p -m 700 "$XDG_RUNTIME_DIR" 2>/dev/null ;;
  esac
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

# build_workspace NAME : colcon build of ros2_ws for container NAME ("sim", "jetson" or "solo"), when needed.
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
    case "$name" in sim|solo) build_world_assets ;; esac
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
  case "$name" in sim|solo) build_world_assets ;; esac
}

# build_world_assets : the real-terrain world (suas_field_real) gets its Gazebo models (terrain tiles, ground
# pictures, trees) made from the site pack in the repo AFTER the build, like simulation/setup/03_build.sh does
# natively:  python3 sim_worlds/scripts/build_real_field.py assets --install --if-needed  (1-2 min the first
# time, then nothing). Only when that generator stage exists in this repo; a failure is only a warning (the
# other worlds do not need it).
build_world_assets() {
  local src="$ASTRA_REPO/ros2_ws/src/simulation/sim_worlds" log
  [ -f "$src/scripts/build_real_field.py" ] && [ -f "$src/sim_worlds/realfield/assets_build.py" ] || return 0
  [ -d "$src/data/suas_field_real" ] || return 0
  log="$ASTRA_SIM_HOME/logs/docker_world_assets_$(date +%Y%m%d-%H%M%S).log"
  info "real-terrain world: making / checking its Gazebo models (build_real_field.py assets; log $log)"
  if ! (
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash && source "$ASTRA_REPO/ros2_ws/install/setup.bash" \
      && cd "$src" && nice -n 10 timeout 1800 python3 scripts/build_real_field.py assets --install --if-needed
  ) > "$log" 2>&1; then
    warn "the real-terrain world's models could not be made (see $log); the other worlds are not affected"
    return 0
  fi
  tail -1 "$log" | sed 's/^/    /'
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

# new_token : a value that is different every time (for the hand-over files).
new_token() { printf '%s-%s\n' "$(date +%s%N)" "$$"; }

# file_token FILE : the TOKEN=... value written in a hand-over file, or "".
file_token() { awk -F= '$1 == "TOKEN" {print $2; exit}' "$1" 2>/dev/null; }
