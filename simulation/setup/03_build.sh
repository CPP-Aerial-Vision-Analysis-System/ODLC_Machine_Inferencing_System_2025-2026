#!/usr/bin/env bash
# =============================================================================
#  03_build.sh  --  STEP 3: copy the repo to Linux and build everything (run again after every change)
#
#  PURPOSE: makes the simulator and the team's code runnable:
#    1. COPY: when this script lives on the Windows disk (/mnt/c, /mnt/d ...), it mirrors the whole repo to
#       a Linux-side copy, ~/ODLC_Machine_Inferencing_System_2025-2026 (no space in the path, a fast disk;
#       Gazebo and colcon misbehave on /mnt paths with spaces). Edit on Windows, then run this again:
#       the copy is overwritten (photos and build results in it are kept). Run from a Linux checkout it
#       builds in place instead.
#    2. BUILD: `colcon build --symlink-install` of ros2_ws: the team's packages (video_cam, detection, main,
#       mapping, waypoint_mavros, ...) and the simulation packages (sim_bringup, sim_models, sim_worlds,
#       siyi_emulator, sim_demo). The vendored ultralytics_ros is ignored (never used; needs PCL).
#    3. MODELS: downloads the Gazebo Fuel 3D models the worlds use (sim_worlds/scripts/fetch_models.sh),
#       and the detector weights yolo26m.pt for the team's real detector into ~/.astra_sim/models/
#       (the repo only has a Jetson-only TensorRT file).
#    4. CAMERA ADDRESS: checks that 192.168.144.25 exists on this PC (camera emulator) and adds it if not.
#
#  HOW TO RUN:   bash 03_build.sh                 (WITHOUT sudo; after steps 1 and 2)
#  Options:      --dest DIR        Linux-side copy (default ~/ODLC_Machine_Inferencing_System_2025-2026)
#                --no-sync         build the existing copy, do not copy from Windows first
#                --no-models       skip the Fuel models and the detector weights
#                --clean           delete the copy's build/install/log first (full rebuild)
#                --packages "a b"  build only these packages (plus what they need)
#                --workers N       packages built at the same time (default 4: keeps a laptop usable;
#                                  more is faster on a big desktop)
#                --add-to-bashrc   load setup/sim_env.sh in every new terminal
#  NEXT STEP:    bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/demo.sh runway
#  Exit codes:   0 = built (warnings may be printed), 1 = failed (the reason and the fix are printed),
#                2 = the command was typed wrong. Every message: docs/simulation/ERRORS.md ("03_build").
# =============================================================================
set -eo pipefail
say()  { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
# Every warning and error says what failed, the likely cause and the fix, tagged [03_build] (docs/simulation/ERRORS.md).
warn() { printf '\033[1;33m !! [03_build] WARNING: %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31m XX [03_build] ERROR: %s\033[0m\n' "$*" >&2; exit 1; }
die_usage() { printf '\033[1;31m XX [03_build] ERROR: %s. Fix: see the options with  bash "%s" --help\033[0m\n' "$*" "$0" >&2; exit 2; }

SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_REPO="$(cd "$SETUP_DIR/../.." && pwd)"
DEST=""
SYNC=1; MODELS=1; CLEAN=0; BASHRC=0; PACKAGES=""; WORKERS=4
while [ $# -gt 0 ]; do
  case "$1" in
    --dest) shift; [ -n "${1:-}" ] || die_usage "--dest needs a folder"; DEST="$1" ;;
    --workers) shift; case "${1:-}" in ''|*[!0-9]*) die_usage "--workers needs a whole number, e.g. --workers 4" ;; esac
               WORKERS="$1" ;;
    --no-sync) SYNC=0 ;;
    --no-models) MODELS=0 ;;
    --clean) CLEAN=1 ;;
    --packages) shift; [ -n "${1:-}" ] || die_usage "--packages needs a list, e.g. --packages \"sim_demo sim_bringup\""
                PACKAGES="$1" ;;
    --add-to-bashrc) BASHRC=1 ;;
    -h|--help) awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"; exit 0 ;;
    *) die_usage "unknown option \"$1\"" ;;
  esac
  shift
done
[ "$(id -u)" -ne 0 ] || die "this build must not run as root.
    Likely cause: it was started with sudo (or as the root user). The build belongs in your own home folder.
    Fix: run it again WITHOUT sudo:  bash \"$0\""
[ -f /opt/ros/humble/setup.bash ] || die "ROS 2 Humble is not installed (/opt/ros/humble/setup.bash is missing).
    Likely cause: setup step 1 has not run on this PC yet (or it failed).
    Fix: sudo bash \"$SETUP_DIR/01_install_humble.sh\"   then run this build again"

# ---- where to build --------------------------------------------------------------------------
case "$SRC_REPO" in
  /mnt/*) DEST="${DEST:-$HOME/ODLC_Machine_Inferencing_System_2025-2026}" ;;
  *)      DEST="${DEST:-$SRC_REPO}" ;;
esac
case "$DEST" in *" "*) die "the build folder \"$DEST\" contains a space.
    Likely cause: --dest points to a folder with a space, or this Linux checkout lives in one. Gazebo and colcon fail there.
    Fix: give a folder without spaces, e.g.  bash \"$0\" --dest \"\$HOME/ODLC_Machine_Inferencing_System_2025-2026\"" ;; esac
export ASTRA_SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"
mkdir -p "$ASTRA_SIM_HOME/logs"
LOG="$ASTRA_SIM_HOME/logs/build_$(date +%Y%m%d-%H%M%S).log"

# ---- 1. copy ---------------------------------------------------------------------------------
if [ "$SYNC" = 1 ] && [ "$SRC_REPO" != "$DEST" ]; then
  say "[1/4] Copying the repo from $SRC_REPO to $DEST"
  mkdir -p "$DEST"
  # --delete keeps the copy identical to Windows; the excluded folders (build results, photos, logs) are
  # never deleted or overwritten.
  rsync -a --delete \
    --exclude '.git/' --exclude '__pycache__/' --exclude '*.pyc' \
    --exclude 'ros2_ws/build/' --exclude 'ros2_ws/install/' --exclude 'ros2_ws/log/' \
    --exclude 'ros2_ws/logs/' --exclude 'ros2_ws/terrain/' --exclude 'ros2_ws/*.tlog' --exclude 'ros2_ws/*.bin' \
    --exclude 'ros2_ws/src/video_cam/mapping_photos/' --exclude 'ros2_ws/src/video_cam/detection_results_sahi/' \
    --exclude 'ros2_ws/src/video_cam/camera_feed/' --exclude 'camera_feed/' \
    "$SRC_REPO/" "$DEST/"
  info "copied ($(du -sh "$DEST" 2>/dev/null | cut -f1))."
else
  say "[1/4] Building in place: $DEST"
fi
[ -d "$DEST/ros2_ws/src" ] || die "there is no ros2_ws/src in $DEST, so there is nothing to build.
    Likely cause: --no-sync on a copy that was never made, or --dest points to the wrong folder.
    Fix: run it without --no-sync (it copies the repo first), or give the right folder with --dest"

# ---- 2. build --------------------------------------------------------------------------------
say "[2/4] Building ros2_ws with colcon (a few minutes the first time; log: $LOG)"
cd "$DEST/ros2_ws"
if [ "$CLEAN" = 1 ]; then info "removing build/ install/ log/"; rm -rf build install log; fi
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# --parallel-workers: colcon would otherwise build one package per CPU core at once (32 on some laptops),
# which can freeze WSL; MAKEFLAGS keeps each C++ package to the same number of compiler jobs.
export MAKEFLAGS="${MAKEFLAGS:--j$WORKERS}"
COLCON_ARGS=(--symlink-install --parallel-workers "$WORKERS" --packages-ignore ultralytics_ros
             --event-handlers console_cohesion- summary+)
if [ -n "$PACKAGES" ]; then
  # shellcheck disable=SC2206
  COLCON_ARGS+=(--packages-up-to $PACKAGES)
fi
set +e
colcon build "${COLCON_ARGS[@]}" 2>&1 | tee "$LOG" | grep -E '^(Starting|Finished|Failed|Aborted|Summary|  [0-9]+ packages?)|ERROR|error:' || true
BUILD_RC=${PIPESTATUS[0]}
set -e
if [ "$BUILD_RC" -ne 0 ]; then
  warn "colcon reported failures (exit $BUILD_RC): the packages that failed are listed above (\"Failed\")." \
       "Consequence: the simulator still starts as long as sim_bringup was built, but those packages are missing or old." \
       "Fix: read the first error in  $LOG  (search for 'error' or 'Failed'), fix it, and build again; one package:" \
       "bash \"$0\" --packages \"NAME\""
fi
[ -f "$DEST/ros2_ws/install/sim_bringup/share/sim_bringup/package.xml" ] || die "sim_bringup was not built, so nothing can start.
    Likely cause: a compile or Python error in a package sim_bringup needs (the 'Failed' lines above), or colcon missing.
    Fix: read the first error in  $LOG , fix it, then build again (a clean build: bash \"$0\" --clean)"
BUILT=""
for d in "$DEST/ros2_ws/install"/*/; do
  name="$(basename "$d")"
  [ -f "$d/share/$name/package.xml" ] && BUILT="$BUILT $name"
done
info "built packages:$BUILT"

# ---- 3. models -------------------------------------------------------------------------------
if [ "$MODELS" = 1 ]; then
  say "[3/4] 3D models and detector weights"
  FETCH="$DEST/ros2_ws/src/simulation/sim_worlds/scripts/fetch_models.sh"
  if [ -f "$FETCH" ]; then
    info "Gazebo Fuel models (trees, rocks, people ...) via sim_worlds/scripts/fetch_models.sh"
    timeout 1800 bash "$FETCH" || warn "some Gazebo Fuel models could not be downloaded. Likely cause: no internet, or" \
      "the Fuel server is slow. Consequence: the worlds still load, with fewer details (trees, rocks, people)." \
      "Fix: run this build again later with internet"
  else
    info "sim_worlds/scripts/fetch_models.sh not found: skipping the Fuel models"
  fi
  # the real-terrain world (suas_field_real): its Gazebo models (terrain tiles, ground pictures, trees, buildings)
  # are made from the site pack that is in the repo. About 1 minute the first time, then nothing (--if-needed).
  RF_SRC="$DEST/ros2_ws/src/simulation/sim_worlds"
  if [ -f "$RF_SRC/scripts/build_real_field.py" ] && [ -f "$RF_SRC/sim_worlds/realfield/assets_build.py" ] \
     && [ -d "$RF_SRC/data/suas_field_real" ]; then
    RF_LOG="$ASTRA_SIM_HOME/logs/real_field_assets_$(date +%Y%m%d-%H%M%S).log"
    mkdir -p "$ASTRA_SIM_HOME/logs"
    # the AI-sharper ground picture (every PC must have it, or its ground is softer than the reference PC's): its own small
    # Python environment (upscale_env.sh), then the picture (about 1 minute once); a failure only warns
    UP_PY="$HOME/.astra_sim/upscale_venv/bin/python"
    if [ ! -x "$UP_PY" ]; then
      info "making the picture sharpener (simulation/setup/upscale_env.sh; once, about 5 minutes)"
      bash "$DEST/simulation/setup/upscale_env.sh" >> "$RF_LOG" 2>&1 \
        || warn "the picture sharpener could not be set up (see $RF_LOG). Consequence: the real-terrain ground is softer than on the" \
                "reference PC and 'build_real_field.py verify' will say so. Fix: bash simulation/setup/upscale_env.sh (needs internet and a PyTorch with CUDA)"
    fi
    if [ -x "$UP_PY" ]; then
      ( cd "$RF_SRC" && nice -n 10 timeout 1800 "$UP_PY" scripts/build_real_field.py upscale --if-needed ) >> "$RF_LOG" 2>&1 \
        || warn "the sharper ground picture could not be made (see $RF_LOG); the ground will be softer than on the reference PC"
    fi
    info "real-terrain world: making / checking its Gazebo models (build_real_field.py assets; log $RF_LOG)"
    if ( set +u; source /opt/ros/humble/setup.bash && source "$DEST/ros2_ws/install/setup.bash" \
           && cd "$RF_SRC" && nice -n 10 timeout 1800 python3 scripts/build_real_field.py assets --install --if-needed ) \
         > "$RF_LOG" 2>&1; then
      tail -1 "$RF_LOG" | sed 's/^/    /'
      ( set +u; source /opt/ros/humble/setup.bash && source "$DEST/ros2_ws/install/setup.bash" && cd "$RF_SRC" \
          && python3 scripts/build_real_field.py verify ) 2>&1 | grep -E 'FAIL|verify\]' | sed 's/^/    /' \
        || true
    else
      warn "the real-terrain world's models could not be made. Likely cause: numpy or Pillow missing for python3, or" \
           "too little disk space. Consequence: only the world suas_field_real is affected (runway and suas_field" \
           "do not need them). Fix: read the end of $RF_LOG, then build again"
    fi
  fi
  # Blender: a dependency for INSPECTING the terrain (build_real_field.py blender); not needed to build or fly. Only reported here.
  if [ -f "$RF_SRC/scripts/build_real_field.py" ]; then
    ( cd "$RF_SRC" && python3 scripts/build_real_field.py blender --check ) 2>&1 | sed 's/^/    /' || true
  fi
  MODEL_DIR="$ASTRA_SIM_HOME/models"
  mkdir -p "$MODEL_DIR" "$HOME/.config"
  if [ -s "$MODEL_DIR/yolo26m.pt" ]; then
    info "detector weights already there: $MODEL_DIR/yolo26m.pt"
  else
    info "downloading the detector weights yolo26m.pt (about 40 MB) into $MODEL_DIR"
    if (cd "$MODEL_DIR" && YOLO_CONFIG_DIR="$HOME/.config" timeout 900 \
          python3 -c "from ultralytics import YOLO; YOLO('yolo26m.pt'); print('yolo26m.pt ready')"); then
      info "saved $MODEL_DIR/yolo26m.pt"
    else
      warn "could not download the detector weights yolo26m.pt. Likely cause: no internet, or the ultralytics Python" \
           "package is missing (setup step 1 installs it). Consequence: the team's real detector cannot run in the" \
           "simulator. Fix: run this build again with internet, or fly with the answer-key detector:" \
           "bash simulation/scripts/demo.sh runway --detector truth"
    fi
  fi
else
  say "[3/4] Skipping the models (--no-models)"
fi

# ---- 4. camera address -----------------------------------------------------------------------
say "[4/4] The camera address 192.168.144.25 (for the SIYI camera emulator)"
if [ -f "$DEST/simulation/scripts/_common.sh" ]; then
  # shellcheck disable=SC1091
  source "$DEST/simulation/scripts/_common.sh"
  if astra_camera_ip_ok; then
    info "already on this PC."
  else
    astra_ensure_camera_ip "$ASTRA_CAMERA_IP" || true
  fi
  if ! systemctl is-enabled astra-camera-ip.service >/dev/null 2>&1; then
    info "To keep it after a restart, run once:  sudo bash $DEST/simulation/setup/camera_ip_alias.sh"
    info "(or from Windows PowerShell: wsl -d ${WSL_DISTRO_NAME:-Ubuntu-22.04} -u root bash $DEST/simulation/setup/camera_ip_alias.sh)"
  fi
fi

if [ "$BASHRC" = 1 ]; then
  LINE="source $DEST/simulation/setup/sim_env.sh"
  if grep -qxF "$LINE" "$HOME/.bashrc" 2>/dev/null; then info "$HOME/.bashrc already loads sim_env.sh"; else
    printf '\n# ASTRA simulator settings (added by simulation/setup/03_build.sh)\n%s\n' "$LINE" >> "$HOME/.bashrc"
    info "added to $HOME/.bashrc: $LINE"
  fi
fi

say "DONE. Next:"
echo "    bash $DEST/simulation/scripts/demo.sh runway        # the full search demo"
echo "    bash $DEST/simulation/scripts/start_sim.sh runway   # only the simulator"
exit 0
