#!/usr/bin/env bash
# =============================================================================
#  build_images.sh  --  build the ASTRA Docker images (dev, sitl, sim) from docker/Dockerfile, laptop-friendly
#
#  PURPOSE: one command for the three images of docker/Dockerfile, built one after the other (each one reuses
#  the finished steps of the one before from Docker's cache), with:
#    * a limit on parallel C/C++ compile jobs (--jobs, default 4 = the Dockerfile's BUILD_JOBS) so building
#      ArduPilot and the Gazebo plugin does not freeze the laptop;
#    * PyTorch for CUDA when this computer has an NVIDIA card (--torch auto), so the team's detector in the
#      "jetson" container uses the graphics card like on the real Jetson (the Dockerfile's own default is the
#      CPU build, which stays the default for `docker build` and the VS Code dev container);
#    * your user id built in (USER_UID / USER_GID), so files the containers write into the mounted repo
#      belong to you;
#    * one log per image in ~/.astra_docker/logs/ and a quick test of every image at the end.
#  The images:  astra-dev:latest   the Jetson's software (Ubuntu 22.04, ROS 2 Humble, MAVROS, Python ML stack)
#                                  = the "jetson" container of docker/compose.sim.yml
#               astra-sitl:latest  dev + ArduCopter SITL 4.6.3 (the Pixhawk's firmware, as a program)
#               astra-sim:latest   sitl + Gazebo Harmonic + ardupilot_gazebo + everything the simulator needs
#                                  = the "sim" container
#  Docker Engine keeps them in /var/lib/docker; in our Windows setup that is inside the Ubuntu-22.04 virtual
#  disk on the D: drive (docs/simulation/DOCKER.md, "Disk space").
#
#  HOW TO RUN (inside the Ubuntu-22.04 WSL distro, or on a Linux PC; from any folder):
#      bash simulation/docker/build_images.sh            all three (first time about 1 hour, about 25 GB)
#      bash simulation/docker/build_images.sh sim        the simulator image (dev + sitl steps come with it)
#      bash simulation/docker/build_images.sh dev        only the Jetson stand-in
#  Options:  --jobs N               parallel compile jobs (default 4; a big desktop can take 16)
#            --torch auto|cpu|cuda  PyTorch flavour (auto = cuda when an NVIDIA card is visible)
#            --no-cache             build every step again (e.g. after a broken download)
#            --pull                 fetch a newer ROS base image first
#            --check                only test the images that exist (no build)
#            --no-check             do not test after building
#            --dry-run              print the docker commands, run nothing
#  Afterwards:  bash simulation/docker/compose_up.sh      (the full demo in containers)
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_host_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_host_common.sh"

WANT=(); JOBS=4; TORCH=auto; NO_CACHE=0; PULL=0; CHECK_ONLY=0; CHECK=1
while [ $# -gt 0 ]; do
  case "$1" in
    dev|sitl|sim) WANT+=("$1") ;;
    all) WANT=(dev sitl sim) ;;
    --jobs) shift; JOBS="${1:-}" ;;
    --torch) shift; TORCH="${1:-}" ;;
    --no-cache) NO_CACHE=1 ;;
    --pull) PULL=1 ;;
    --check) CHECK_ONLY=1 ;;
    --no-check) CHECK=0 ;;
    --dry-run) ASTRA_DRY=1 ;;
    -h|--help) astra_usage "$0"; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
  shift
done
[ "${#WANT[@]}" -gt 0 ] || WANT=(dev sitl sim)
case "$JOBS" in ''|*[!0-9]*|0) die "--jobs needs a whole number of 1 or more" ;; esac
# Build in the Dockerfile's order (dev -> sitl -> sim), whatever order the names were given in.
TARGETS=()
for t in dev sitl sim; do
  for w in "${WANT[@]}"; do [ "$w" = "$t" ] && { TARGETS+=("$t"); break; }; done
done

DOCKER_DIR="$ASTRA_REPO_HERE/docker"
[ -f "$DOCKER_DIR/Dockerfile" ] || die "docker/Dockerfile not found next to this script ($DOCKER_DIR)"
astra_need_docker --no-compose

# ---- check-only mode -------------------------------------------------------------------------------
# check_image TARGET : start the image once and test the pieces the simulator / launcher.sh need.
check_image() {
  local t="$1" img="astra-$1:latest" script
  if [ "$ASTRA_DRY" = 0 ] && ! astra_image_exists "$img"; then warn "$img does not exist (not built)"; return 1; fi
  # The test runs INSIDE the image; every line prints PASS or FAIL.
  script='
    t() { if eval "$2" >/dev/null 2>&1; then echo "    PASS  $1"; else echo "    FAIL  $1   ($2)"; FAILED=1; fi; }
    FAILED=0; set +u; source /opt/ros/humble/setup.bash
    t "ROS 2 Humble + MAVROS"                "ros2 pkg prefix mavros && ros2 pkg prefix mavros_extras"
    t "cv_bridge, vision_msgs"               "ros2 pkg prefix cv_bridge && ros2 pkg prefix vision_msgs"
    t "mavlink-routerd"                      "command -v mavlink-routerd"
    t "MAVProxy + pymavlink"                 "command -v mavproxy.py && python3 -c \"import pymavlink\""
    t "ping + ip (launcher.sh camera check)" "command -v ping && command -v ip"
    t "numpy < 2 (cv_bridge)"                "python3 -c \"import numpy; assert int(numpy.__version__[0]) < 2\""
    if python3 -c "import torch" 2>/dev/null; then
      echo "    info  torch $(python3 -c "import torch; print(torch.__version__)")"
      t "ultralytics + sahi"                 "python3 -c \"import ultralytics, sahi\""
    fi'
  case "$t" in
    sitl|sim) script="$script"'
    t "ArduCopter SITL binary"               "test -x /opt/ardupilot/build/sitl/bin/arducopter"
    t "SITL parameter files"                 "test -f /opt/ardupilot/Tools/autotest/default_params/copter.parm && test -f /opt/ardupilot/Tools/autotest/default_params/gazebo-iris.parm"' ;;
  esac
  case "$t" in
    sim) script="$script"'
    t "Gazebo Harmonic (gz sim 8)"           "gz sim --versions | grep -q \"^8\""
    t "ros_gz (Harmonic build)"              "ros2 pkg prefix ros_gz_sim && ros2 pkg prefix ros_gz_bridge"
    t "ArduPilot Gazebo plugin"              "test -f /opt/ardupilot_gazebo/build/libArduPilotPlugin.so"
    t "xacro + rviz2"                        "ros2 pkg prefix xacro && ros2 pkg prefix rviz2"
    t "Gazebo Python bindings"               "python3 -c \"import gz.transport13, gz.msgs10\""
    t "PyQt5 + Pillow (demo panel)"          "python3 -c \"import PyQt5.QtWidgets, PIL\""
    t "Mesa d3d12 driver (WSLg GPU)"         "test -f /usr/lib/x86_64-linux-gnu/dri/d3d12_dri.so"
    t "GStreamer"                            "command -v gst-launch-1.0"
    t "env: ARDUPILOT_SITL_BIN / GAZEBO_DIR" "test -x \"$ARDUPILOT_SITL_BIN\" && test -d \"$ARDUPILOT_GAZEBO_DIR/build\""' ;;
  esac
  script="$script"'
    exit $FAILED'
  say "Testing $img"
  if [ "$ASTRA_DRY" = 1 ]; then astra_run docker run --rm "$img" bash -c '<the PASS/FAIL tests>'; return 0; fi
  if docker run --rm "$img" bash -c "$script"; then ok "$img passed"; else warn "$img FAILED some tests (see above)"; return 1; fi
  # CUDA: only when the image has CUDA PyTorch and Docker can give containers the NVIDIA card.
  if [ "$(docker image inspect "$img" --format '{{index .Config.Labels "org.astra.torch"}}' 2>/dev/null)" = cuda ] \
     && astra_has_nvidia_runtime; then
    if docker run --rm --gpus all "$img" python3 -c "import torch, sys; print('    CUDA:', torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else ''); sys.exit(0 if torch.cuda.is_available() else 1)"; then
      ok "$img: PyTorch sees the graphics card"
    else
      warn "$img: PyTorch does NOT see the graphics card (docker run --rm --gpus all ubuntu:22.04 nvidia-smi works?)"
    fi
  fi
}

if [ "$CHECK_ONLY" = 1 ]; then
  RC=0
  for t in "${TARGETS[@]}"; do check_image "$t" || RC=1; done
  exit "$RC"
fi

# ---- settings ----------------------------------------------------------------------------------------
if [ "$TORCH" = auto ]; then
  if [ -e /usr/lib/wsl/lib/libcuda.so.1 ] || nvidia-smi -L >/dev/null 2>&1; then TORCH=cuda; else TORCH=cpu; fi
fi
case "$TORCH" in
  cpu)  TORCH_URL=https://download.pytorch.org/whl/cpu ;;
  # CUDA 12.6 = the Jetson's CUDA version and what the native simulator setup installs (01_install_humble.sh).
  cuda) TORCH_URL=https://download.pytorch.org/whl/cu126 ;;
  *) die "--torch must be auto, cpu or cuda" ;;
esac
for t in "${TARGETS[@]}"; do
  if [ "$t" = sim ] && [ "$(uname -m)" != x86_64 ]; then
    die "The sim image needs an amd64 (x86_64) computer: Gazebo's ROS packages are not made for $(uname -m)."
  fi
done
# The image's user gets your user and group id (only normal ids; system ids like 20 would clash in the image).
UID_ARGS=()
if [ "$(id -u)" -ge 1000 ] && [ "$(id -g)" -ge 1000 ]; then
  UID_ARGS=(--build-arg "USER_UID=$(id -u)" --build-arg "USER_GID=$(id -g)")
fi
COMMIT="$(git -C "$ASTRA_REPO_HERE" rev-parse --short HEAD 2>/dev/null || echo unknown)"

say "Building ${TARGETS[*]} (compile jobs $JOBS, PyTorch $TORCH, user id $(id -u))"
info "Dockerfile: $DOCKER_DIR/Dockerfile"
if [ "$ASTRA_DRY" = 0 ]; then
  ROOT_DIR="$(docker info --format '{{.DockerRootDir}}' 2>/dev/null)"
  FREE_GB="$(df -BG --output=avail "${ROOT_DIR:-/var/lib/docker}" 2>/dev/null | tail -1 | tr -dc '0-9')"
  info "images go to ${ROOT_DIR:-?} (${FREE_GB:-?} GB free there)"
  [ -n "$FREE_GB" ] && [ "$FREE_GB" -lt 40 ] && warn "less than 40 GB free: the three images need about 25 GB plus build space"
  astra_mem_check 3000
fi
mkdir -p "$ASTRA_DOCKER_DATA/logs"

# ---- build ---------------------------------------------------------------------------------------------
FIRST=1; RC=0; T0=$(date +%s)
for t in "${TARGETS[@]}"; do
  img="astra-$t:latest"
  log="$ASTRA_DOCKER_DATA/logs/build_${t}_$(date +%Y%m%d-%H%M%S).log"
  cmd=(docker build -f "$DOCKER_DIR/Dockerfile" --target "$t" -t "$img" --progress=plain
       --build-arg "BUILD_JOBS=$JOBS" --build-arg "TORCH_INDEX_URL=$TORCH_URL" "${UID_ARGS[@]}"
       --label "org.astra.torch=$TORCH" --label "org.astra.target=$t" --label "org.astra.repo-commit=$COMMIT")
  [ "$t" = sim ] && cmd+=(--platform linux/amd64)
  # --no-cache / --pull only for the FIRST image: the later ones reuse its fresh steps (sitl and sim contain
  # dev), instead of building everything a second and third time.
  if [ "$FIRST" = 1 ]; then
    [ "$NO_CACHE" = 1 ] && cmd+=(--no-cache)
    [ "$PULL" = 1 ] && cmd+=(--pull)
  fi
  cmd+=("$DOCKER_DIR")
  FIRST=0
  say "Image $img (target $t)"
  if [ "$ASTRA_DRY" = 1 ]; then astra_run "${cmd[@]}"; continue; fi
  info "log: $log   (the lines below are the build steps; Ctrl+C stops the build)"
  t1=$(date +%s)
  # Show only the step headers ("#12 [sim 3/9] RUN ...") and errors; everything goes into the log.
  "${cmd[@]}" 2>&1 | tee "$log" | grep --line-buffered -E '^#[0-9]+ \[[^]]+\] |ERROR|error:' | cut -c1-150
  rc=${PIPESTATUS[0]}
  if [ "$rc" -ne 0 ]; then
    warn "building $img FAILED (exit $rc). The end of the log:"
    tail -30 "$log" | sed 's/^/      /' >&2
    RC=1; break
  fi
  ok "$img built in $(( ($(date +%s) - t1) / 60 )) min"
done
[ "$ASTRA_DRY" = 1 ] && exit 0
[ "$RC" = 0 ] || die "stopped. Fix the problem shown above and run this again (finished steps are not repeated)."

docker image ls --filter 'reference=astra-*' --format '    {{.Repository}}:{{.Tag}}  {{.Size}}  ({{.CreatedSince}})'
info "total build time: $(( ($(date +%s) - T0) / 60 )) min"
if [ "$CHECK" = 1 ]; then
  for t in "${TARGETS[@]}"; do check_image "$t" || RC=1; done
fi
[ "$RC" = 0 ] && say "DONE. Next: bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh   (docs/simulation/DOCKER.md)"
exit "$RC"
