# shellcheck shell=bash
# =============================================================================
#  _host_common.sh  --  helpers for the Docker scripts that run on the HOST (sourced, never run)
#
#  PURPOSE: the checks and choices every host-side Docker script (build_images.sh, compose_up.sh,
#  compose_down.sh, run_sim_container.sh, run_jetson_container.sh, export/import_images.sh, check_host.sh)
#  needs, written once:
#    * friendly messages (say / info / ok / warn / die), the same style as the other simulation scripts;
#    * astra_need_docker: is "docker" really a Docker Engine (not the Windows Docker Desktop program), may this
#      user use it (the "docker" group), does Compose exist and is it new enough;
#    * astra_pick_repo: which repo folder to mount into the containers (on Windows, when the scripts are run from
#      a Windows drive /mnt/c, /mnt/d ..., the Linux-side copy ~/ODLC_Machine_Inferencing_System_2025-2026 that
#      simulation/setup/03_build.sh keeps, because Windows drives are slow and their paths often have spaces);
#    * astra_prepare_data: the host folder for runs, logs and detector weights (~/.astra_docker);
#    * astra_display_mode / astra_compose_files / astra_gui_run_args / astra_gpu_run_args: how containers show
#      windows (WSLg on Windows, X11 on Linux, none) and get the graphics card (DirectX through /dev/dxg,
#      /dev/dri, NVIDIA CUDA), for `docker compose` (which add-on files) and for `docker run` (which options);
#    * astra_check_image_user: the images have one user id built in; warn when it is not yours;
#    * astra_docker_inputs_hash / astra_check_image_fresh: a fingerprint of docker/Dockerfile + the requirements
#      files, stored in each image by build_images.sh, so an image older than those files is reported ("stale");
#    * astra_prepare_repo_mounts: create ros2_ws/build, install, log in the repo before Docker mounts the build
#      volumes there (Docker would create them owned by root, and a later native build could not write into them);
#    * astra_copy_differences: how many files of the Windows folder are newer than the Linux-side copy (tool
#      caches not counted; ASTRA_NO_COPY_CHECK=1 skips this 3-second look);
#    * astra_free_camera_ip: the native simulator gives this computer the camera address 192.168.144.25 on its
#      loopback; while that is there, the "sim" container (which IS 192.168.144.25 on the Docker network) gets
#      no internet and the host answers in its place, so it is taken away (the native scripts put it back);
#    * astra_subnet_conflict: is 192.168.144.0/24 already used here (another network, the real drone's)? Any
#      overlap counts, also a Docker network that covers it (192.168.144.0/20); astra_subnet_wider_route finds a
#      wider route (a VPN's 192.168.0.0/16) that does not block the containers but is worth knowing about;
#    * astra_build_uid: the user id build_images.sh builds into the images for you;
#    * astra_run: run a command, or only print it with --dry-run.
#  Nothing here knows about one particular computer: every path is found at run time.
# =============================================================================

ASTRA_DOCKER_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_REPO_HERE="$(cd "$ASTRA_DOCKER_SCRIPTS/../.." && pwd)"   # the repo these scripts belong to
ASTRA_LINUX_COPY="$HOME/ODLC_Machine_Inferencing_System_2025-2026"
ASTRA_CONTAINER_REPO=/ODLC_Machine_Inferencing_System_2025-2026
ASTRA_CONTAINER_HOME=/home/ubuntu                              # the images' user (docker/Dockerfile USERNAME)
ASTRA_CAMERA_IP=192.168.144.25                                 # real SIYI camera = the "sim" container
ASTRA_JETSON_IP=192.168.144.100                                # real Jetson eth0 = the "jetson" container
ASTRA_SUBNET=192.168.144.0/24
ASTRA_NET=astra_net
ASTRA_IMAGES="astra-dev:latest astra-sitl:latest astra-sim:latest"
export ASTRA_DOCKER_DATA="${ASTRA_DOCKER_DATA:-$HOME/.astra_docker}"
ASTRA_DRY="${ASTRA_DRY:-0}"

say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '\033[1;32m OK %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m !! %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31m XX %s\033[0m\n' "$*" >&2; exit 1; }

# astra_usage FILE : print the comment block at the top of FILE (the script's built-in help).
astra_usage() { awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$1"; }

# astra_run CMD... : run a command; with ASTRA_DRY=1 only print it (copy-paste ready).
astra_run() {
  if [ "$ASTRA_DRY" = 1 ]; then
    printf '    +'; printf ' %q' "$@"; printf '\n'
  else
    "$@"
  fi
}

astra_is_wsl() { grep -qi microsoft /proc/version 2>/dev/null; }

# astra_need_docker [--no-compose] : stop with a clear message when Docker Engine cannot be used.
astra_need_docker() {
  local where
  if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
    # The usual reason: Docker said "permission denied" and sudo seemed to help. It hides the real fix and moves
    # $HOME to /root, so the data folder and the Linux-side repo copy are looked for in the wrong place.
    warn "You started this with sudo. Run it as yourself (no sudo): with sudo the home folder is /root, so the data"
    warn "  folder and the Linux-side repo copy are looked for in the wrong place. Docker's \"permission denied\" is"
    warn "  fixed by the docker group instead:  sudo usermod -aG docker $SUDO_USER   then open a new login (see DOCKER.md)."
  fi
  where="$(command -v docker 2>/dev/null || true)"
  if [ -z "$where" ]; then
    die "Docker is not installed in this Ubuntu. Install Docker Engine once (admin rights):
    sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh
    (Windows, from PowerShell:  wsl -d ${WSL_DISTRO_NAME:-Ubuntu-22.04} -u root -- bash <that file> --user $(id -un))"
  fi
  case "$where" in
    /mnt/*) die "\"docker\" here is Docker Desktop's Windows program ($where), not a Docker Engine for Linux.
    Install Docker Engine inside this Ubuntu:  sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh
    then open a NEW terminal (type -a docker must show /usr/bin/docker first).
    (Or switch on Docker Desktop's WSL integration for this distro instead: docs/simulation/DOCKER.md, A6.)" ;;
  esac
  [ "$ASTRA_DRY" = 1 ] && return 0
  local err
  if ! err="$(docker info 2>&1 >/dev/null)"; then
    case "$err" in
      *"permission denied"*)
        die "You may not use Docker yet: your user is not in the \"docker\" group in THIS terminal.
    Windows: close every Ubuntu window, run  wsl --terminate ${WSL_DISTRO_NAME:-Ubuntu-22.04}  in PowerShell, open Ubuntu again.
    Linux: log out and in again (or restart).   Right now, for one command:  sg docker -c \"bash $0\"" ;;
      *) die "Docker is not running or not reachable:
    $err
    Start it:  sudo systemctl start docker     (logs: journalctl -u docker --no-pager | tail -40)" ;;
    esac
  fi
  [ "${1:-}" = --no-compose ] && return 0
  docker compose version >/dev/null 2>&1 || die "Docker Compose is missing (package docker-compose-plugin). Re-run install_docker_engine.sh."
  local v major minor
  v="$(docker compose version --short 2>/dev/null)"; v="${v#v}"
  major="${v%%.*}"; minor="$(printf '%s' "$v" | cut -d. -f2)"
  if [ "${major:-0}" -lt 2 ] 2>/dev/null || { [ "${major:-0}" -eq 2 ] && [ "${minor:-0}" -lt 20 ]; } 2>/dev/null; then
    die "Docker Compose $v is too old (2.20 or newer is needed). Install the current one from Docker's apt source:
    sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh"
  fi
}

astra_image_exists() { docker image inspect "$1" >/dev/null 2>&1; }

# astra_require_images "IMAGE TARGET"... : stop when an image is missing and say how to get it.
astra_require_images() {
  local pair img target missing=()
  [ "$ASTRA_DRY" = 1 ] && return 0
  for pair in "$@"; do
    img="${pair% *}"; target="${pair#* }"
    astra_image_exists "$img" || missing+=("$img (bash simulation/docker/build_images.sh $target)")
  done
  if [ "${#missing[@]}" -gt 0 ]; then
    warn "These Docker images are not on this computer yet:"
    for img in "${missing[@]}"; do warn "    $img"; done
    die "Build them first (the first build takes about an hour; afterwards only changed steps are rebuilt),
    or load images a teammate saved (docs/simulation/DOCKER.md, \"Copy images without rebuilding\").
    No Docker login is needed: these images are built on your PC and are not on Docker Hub."
  fi
}

# astra_image_uid IMAGE : the user id built into IMAGE (label org.astra.uid, set by build_images.sh), or "".
astra_image_uid() { astra_image_label "$1" org.astra.uid; }

# astra_build_uid : the user id build_images.sh builds into the images for THIS user (the same rule as there):
# your own id when your user AND group id are 1000 or more (normal accounts), otherwise 1000, the Dockerfile's
# default (small ids such as 20 or 500 belong to system groups inside the Ubuntu image and would clash).
astra_build_uid() {
  if [ "$(id -u)" -ge 1000 ] && [ "$(id -g)" -ge 1000 ]; then id -u; else echo 1000; fi
}

# astra_check_image_user IMAGE... : warn when an image's built-in user id is not this user's. Then files the
# containers write into the mounted repo belong to someone else, and an X11 desktop refuses their windows.
# Only says "rebuild" when a rebuild here would change it: for a user id below 1000 build_images.sh always uses
# 1000 (see astra_build_uid), so that case gets a plain explanation instead of advice that cannot help.
astra_check_image_user() {
  local img uid
  [ "$ASTRA_DRY" = 1 ] && return 0
  for img in "$@"; do
    uid="$(astra_image_uid "$img")"
    [ -z "$uid" ] && uid="$(docker run --rm --entrypoint id "$img" -u 2>/dev/null)"
    if [ -n "$uid" ] && [ "$uid" != "$(id -u)" ]; then
      if [ "$uid" != "$(astra_build_uid)" ]; then
        warn "$img was built for user id $uid, you are $(id -u) (images copied from another PC?)."
        warn "  It still runs; files it writes into the repo then belong to user id $uid. For a clean setup rebuild"
        warn "  with your id:  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh   (docs/simulation/DOCKER.md, troubleshooting)"
      else
        info "$img uses user id $uid; yours is $(id -u), below 1000, so the images always use 1000 (nothing to"
        info "  rebuild). Files the containers write into the repo belong to user id $uid."
      fi
      ASTRA_UID_MISMATCH="$uid"
    fi
  done
}

# astra_docker_inputs_hash : a short fingerprint (16 hex digits) of everything an image build reads from the repo:
# docker/Dockerfile and docker/requirements-*.txt. Line endings are ignored, so a Windows (CRLF) checkout gives the
# same value. build_images.sh stores it in every image (label org.astra.inputs).
astra_docker_inputs_hash() {
  local d="$ASTRA_REPO_HERE/docker"
  cat "$d/Dockerfile" "$d/requirements-dev.txt" "$d/requirements-ml.txt" 2>/dev/null | tr -d '\r' | sha256sum | cut -c1-16
}

# astra_image_label IMAGE LABEL : the value of one label of IMAGE ("" when the image or the label is missing).
astra_image_label() {
  local v
  v="$(docker image inspect "$1" --format "{{index .Config.Labels \"$2\"}}" 2>/dev/null)"
  [ "$v" = "<no value>" ] && v=""
  printf '%s\n' "$v"
}

# astra_check_image_fresh IMAGE... : warn when an image was built from OLDER docker/ files than this repo has now
# (somebody changed the Dockerfile or a requirements file since). The code itself is mounted, never baked in, so
# only these files make an image stale. Images built without build_images.sh carry no fingerprint: not judged.
astra_check_image_fresh() {
  local img want have
  [ "$ASTRA_DRY" = 1 ] && return 0
  want="$(astra_docker_inputs_hash)"
  for img in "$@"; do
    have="$(astra_image_label "$img" org.astra.inputs)"
    if [ -n "$have" ] && [ "$have" != "$want" ]; then
      warn "$img is older than docker/Dockerfile or docker/requirements-*.txt of this repo (they changed after it was built)."
      warn "  It can lack a package the code now needs. Rebuild it (unchanged steps are reused):  bash $ASTRA_DOCKER_SCRIPTS/build_images.sh"
    fi
  done
}

# astra_prepare_repo_mounts REPO : create ros2_ws/build, install and log in REPO before a container mounts its own
# build volumes there. If they are missing, Docker creates them itself, owned by root, and a later NATIVE colcon
# build in the same checkout fails with "Permission denied". Existing folders are never touched.
astra_prepare_repo_mounts() {
  local d
  for d in build install log; do
    [ -e "$1/ros2_ws/$d" ] || astra_run mkdir -p "$1/ros2_ws/$d"
  done
}

# astra_pick_repo [FOLDER] : print the repo folder to mount into the containers.
# When these scripts are run from a Windows drive (/mnt/c, /mnt/d ...: slow, often a space in the path), the
# Linux-side copy that simulation/setup/03_build.sh keeps (~/ODLC_Machine_Inferencing_System_2025-2026) is used
# instead when it exists. A repo cloned inside Linux (the recommended way) is used as it is.
astra_pick_repo() {
  local want="${1:-}"
  if [ -n "$want" ]; then
    [ -d "$want/ros2_ws/src" ] || die "--repo $want: no ros2_ws/src there"
    (cd "$want" && pwd); return 0
  fi
  case "$ASTRA_REPO_HERE" in
    /mnt/*)
      if [ -d "$ASTRA_LINUX_COPY/ros2_ws/src" ]; then
        # The copy only changes when somebody syncs it: say so when the Windows folder has newer files, otherwise
        # the containers silently run yesterday's code.
        local behind
        behind="$(astra_copy_differences)"
        if [ "${behind:-0}" -gt 0 ] 2>/dev/null; then
          warn "The Linux-side copy ($ASTRA_LINUX_COPY) is OLDER than this Windows folder: $behind file(s) changed here."
          warn "  The containers run the copy, so your latest edits would be missing. Add --sync (or run: bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only)."
        fi
        printf '%s\n' "$ASTRA_LINUX_COPY"
      else
        warn "No Linux-side copy ($ASTRA_LINUX_COPY) yet: mounting the Windows folder (works, but slower)."
        warn "Make the copy with:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only   (or setup/03_build.sh)"
        printf '%s\n' "$ASTRA_REPO_HERE"
      fi ;;
    *) printf '%s\n' "$ASTRA_REPO_HERE" ;;
  esac
}

# What the Linux-side copy never takes from the Windows folder (the same list as 03_build.sh's step 1): git data,
# caches, build results, flight logs, photos and detector output.
ASTRA_SYNC_EXCLUDES=(
  --exclude '.git/' --exclude '__pycache__/' --exclude '*.pyc'
  --exclude 'ros2_ws/build/' --exclude 'ros2_ws/install/' --exclude 'ros2_ws/log/'
  --exclude 'ros2_ws/logs/' --exclude 'ros2_ws/terrain/' --exclude 'ros2_ws/*.tlog' --exclude 'ros2_ws/*.bin'
  --exclude 'ros2_ws/src/video_cam/mapping_photos/' --exclude 'ros2_ws/src/video_cam/detection_results_sahi/'
  --exclude 'ros2_ws/src/video_cam/camera_feed/' --exclude 'camera_feed/'
)

# astra_sync_copy : refresh the Linux-side copy from the Windows folder (the same rsync as 03_build.sh's step 1,
# without its native colcon build). Build results, photos and logs in the copy are kept.
astra_sync_copy() {
  case "$ASTRA_REPO_HERE" in
    /mnt/*) ;;
    *) info "the repo is already on the Linux disk ($ASTRA_REPO_HERE): nothing to copy"; return 0 ;;
  esac
  command -v rsync >/dev/null 2>&1 || die "rsync is missing:  sudo apt-get install rsync"
  say "Copying the repo to the Linux disk: $ASTRA_REPO_HERE -> $ASTRA_LINUX_COPY"
  astra_run mkdir -p "$ASTRA_LINUX_COPY"
  astra_run rsync -a --delete "${ASTRA_SYNC_EXCLUDES[@]}" "$ASTRA_REPO_HERE/" "$ASTRA_LINUX_COPY/"
  [ "$ASTRA_DRY" = 1 ] || ok "copied ($(du -sh "$ASTRA_LINUX_COPY" 2>/dev/null | cut -f1))"
}

# astra_copy_differences : how many files astra_sync_copy would copy right now (new or changed in the Windows
# folder), "0" when the Linux-side copy is up to date or there is nothing to compare. Only looks (rsync --dry-run).
# Reading every file's date through the Windows drive costs about 3 s per start; ASTRA_NO_COPY_CHECK=1 skips it.
astra_copy_differences() {
  case "$ASTRA_REPO_HERE" in /mnt/*) ;; *) echo 0; return 0 ;; esac
  [ "${ASTRA_NO_COPY_CHECK:-0}" = 1 ] && { echo 0; return 0; }
  [ -d "$ASTRA_LINUX_COPY/ros2_ws/src" ] && command -v rsync >/dev/null 2>&1 || { echo 0; return 0; }
  rsync -a -n --itemize-changes "${ASTRA_SYNC_EXCLUDES[@]}" "$ASTRA_REPO_HERE/" "$ASTRA_LINUX_COPY/" 2>/dev/null \
    | astra_count_copy_changes
}

# astra_count_copy_changes : read rsync's --itemize-changes list on stdin and print how many FILES would be copied
# ("0" for none), not counting tool caches (.ruff_cache, .pytest_cache, .mypy_cache ...) and editor/agent folders
# (.vscode, .claude): they change all the time but never change what the containers run, so they must not make the
# "the Linux-side copy is OLDER" warning cry wolf. An rsync line looks like ">f+++++++++ path/to/file".
astra_count_copy_changes() {
  awk 'substr($0, 1, 2) == ">f" {
         p = $0; sub(/^[^ ]+ /, "", p)
         if (p !~ /(^|\/)(\.[^\/]*_cache|\.hypothesis|\.vscode|\.claude)\//) n++
       }
       END { print n + 0 }'
}

# astra_prepare_data : create the host data folder; reuse the native simulator's detector weights.
astra_prepare_data() {
  astra_run mkdir -p "$ASTRA_DOCKER_DATA/logs" "$ASTRA_DOCKER_DATA/models" "$ASTRA_DOCKER_DATA/runs" "$ASTRA_DOCKER_DATA/state"
  local native="$HOME/.astra_sim/models/yolo26m.pt"
  if [ ! -s "$ASTRA_DOCKER_DATA/models/yolo26m.pt" ] && [ -s "$native" ]; then
    astra_run cp -p "$native" "$ASTRA_DOCKER_DATA/models/yolo26m.pt"
    info "detector weights copied from the native simulator ($native)"
  fi
}

# astra_use_host_fuel : share the native Gazebo's 3D model cache (~/.gz/fuel) with the "sim" container when it
# exists (no second 110 MB download); otherwise compose.sim.yml keeps them in the Docker volume astra_gz_fuel.
astra_use_host_fuel() {
  if [ -z "${ASTRA_FUEL_DIR:-}" ] && [ -d "$HOME/.gz/fuel" ]; then
    export ASTRA_FUEL_DIR="$HOME/.gz/fuel"
  fi
}

# astra_display_mode : how containers can show windows here: wslg | x11 | none.
astra_display_mode() {
  if astra_is_wsl && [ -d /mnt/wslg ] && [ -d /tmp/.X11-unix ]; then
    echo wslg
  elif ! astra_is_wsl && [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    echo x11
  else
    echo none
  fi
}

# astra_has_nvidia_runtime : true when Docker can give containers the NVIDIA card (--gpus all).
astra_has_nvidia_runtime() {
  docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null | tr ' ' '\n' | grep -qx nvidia
}

# astra_render_gid : the group id that owns /dev/dri/renderD128 (Linux graphics), or "" when there is none.
astra_render_gid() {
  local node
  for node in /dev/dri/renderD128 /dev/dri/card0; do
    [ -e "$node" ] && { stat -c %g "$node"; return 0; }
  done
  return 0
}

# astra_compose_files MODE GPU : fill COMPOSE_FILES (-f options for docker compose) and COMPOSE_TEXT (a
# description) for display MODE (wslg | x11 | none) and GPU (auto | yes | no). See docker/compose.*.yml.
astra_compose_files() {
  local mode="$1" gpu="$2" d="$ASTRA_REPO_HERE/docker"
  COMPOSE_FILES=(-f "$d/compose.sim.yml"); COMPOSE_TEXT="compose.sim.yml"
  if astra_is_wsl; then
    # On Windows the WSLg file also carries the 3D path to the graphics card (/dev/dxg), so it is used even
    # without windows (--headless): Gazebo's camera still draws its pictures on the card.
    if [ -d /mnt/wslg ] && [ -e /dev/dxg ]; then
      COMPOSE_FILES+=(-f "$d/compose.wslg.yml"); COMPOSE_TEXT="$COMPOSE_TEXT + compose.wslg.yml"
    fi
  else
    if [ "$mode" = x11 ]; then
      COMPOSE_FILES+=(-f "$d/compose.linux.yml"); COMPOSE_TEXT="$COMPOSE_TEXT + compose.linux.yml"
    fi
    if [ -e /dev/dri ]; then
      export ASTRA_RENDER_GID="${ASTRA_RENDER_GID:-$(astra_render_gid)}"
      COMPOSE_FILES+=(-f "$d/compose.dri.yml"); COMPOSE_TEXT="$COMPOSE_TEXT + compose.dri.yml"
    fi
  fi
  if [ "$gpu" != no ] && { [ "$ASTRA_DRY" = 1 ] || astra_has_nvidia_runtime; }; then
    COMPOSE_FILES+=(-f "$d/compose.gpu.yml"); COMPOSE_TEXT="$COMPOSE_TEXT + compose.gpu.yml"
  elif [ "$gpu" = yes ]; then
    die "--gpu: Docker has no NVIDIA runtime (sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh --nvidia)"
  fi
}

# astra_gui_run_args MODE : fill the array GUI_ARGS with `docker run` options for windows + 3D drawing
# (the same as docker/compose.wslg.yml, compose.linux.yml and compose.dri.yml).
astra_gui_run_args() {
  GUI_ARGS=()
  if astra_is_wsl && [ -d /mnt/wslg ] && [ -e /dev/dxg ]; then
    GUI_ARGS=(-e "DISPLAY=${DISPLAY:-:0}" -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}"
              -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e PULSE_SERVER=unix:/mnt/wslg/PulseServer
              -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg
              -v /usr/lib/wsl:/usr/lib/wsl:ro --device /dev/dxg -e LD_LIBRARY_PATH=/usr/lib/wsl/lib
              -e GALLIUM_DRIVER=d3d12 -e "MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
              -e QT_X11_NO_MITSHM=1)
    return 0
  fi
  if [ "$1" = x11 ]; then
    GUI_ARGS=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1
              -e XDG_RUNTIME_DIR=/tmp/runtime-ubuntu)
  fi
  if ! astra_is_wsl && [ -e /dev/dri ]; then
    local gid="${ASTRA_RENDER_GID:-$(astra_render_gid)}"
    GUI_ARGS+=(--device /dev/dri --group-add "${gid:-video}")
  fi
}

# astra_gpu_run_args WANT : fill GPU_ARGS with `--gpus all ...` when WANT=yes|auto and Docker has the
# NVIDIA runtime (capabilities "all": CUDA + OpenGL; see docker/compose.gpu.yml).
astra_gpu_run_args() {
  GPU_ARGS=()
  case "$1" in no) return 0 ;; esac
  if [ "$ASTRA_DRY" = 1 ] || astra_has_nvidia_runtime; then
    GPU_ARGS=(--gpus all -e "NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all}")
  elif [ "$1" = yes ]; then
    die "--gpu was asked for, but Docker has no NVIDIA runtime (install_docker_engine.sh --nvidia installs it)"
  fi
}

# astra_x11_allow : let containers open windows on a Linux X11 desktop. The X server checks the user id of
# whoever connects: the containers run as the images' user, normally your own id (build_images.sh).
astra_x11_allow() {
  if ! command -v xhost >/dev/null 2>&1; then
    warn "xhost is missing (sudo apt-get install x11-xserver-utils): windows may be refused by the X server"
    return 0
  fi
  if [ -n "${ASTRA_UID_MISMATCH:-}" ]; then
    # The images' user is someone else's id: allow local programs of every user (only this PC's own users).
    warn "allowing every local user's programs on your X11 screen (xhost +local:), because of the user id above"
    astra_run xhost +local: >/dev/null 2>&1 || warn "xhost failed: windows may not open (no X11 desktop?)"
  else
    astra_run xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || warn "xhost failed: windows may not open (no X11 desktop?)"
  fi
}

# astra_windows_gcs : HOST:PORT where Mission Planner / QGroundControl on WINDOWS can receive MAVLink from a
# container ("" when unknown). In WSL's NAT networking the Windows host is this Ubuntu's default gateway.
astra_windows_gcs() {
  astra_is_wsl || return 0
  local mode gw
  mode="$(wslinfo --networking-mode 2>/dev/null || echo nat)"
  case "$mode" in
    nat) gw="$(ip -4 route show default 2>/dev/null | awk '{print $3; exit}')"
         [ -n "$gw" ] && printf '%s:14550\n' "$gw" ;;
    *) warn "WSL networking mode \"$mode\": no automatic Mission Planner copy (give --windows-gcs <Windows IP>:14550)" ;;
  esac
}

astra_camera_ip_on_host() { ip -4 addr show dev lo 2>/dev/null | grep -q "inet ${ASTRA_CAMERA_IP}/"; }

# astra_free_camera_ip : take the native simulator's camera address off this computer's loopback (see the top).
astra_free_camera_ip() {
  astra_camera_ip_on_host || return 0
  say "The native simulator's camera address $ASTRA_CAMERA_IP is on this computer (loopback): removing it"
  info "Why: on the Docker network the \"sim\" container IS $ASTRA_CAMERA_IP. While this computer also has that"
  info "address, the computer answers in the container's place and the container gets no internet."
  info "The native start_sim.sh / demo.sh add it back by themselves the next time they run."
  if [ "$ASTRA_DRY" = 1 ]; then
    astra_run sudo ip addr del "$ASTRA_CAMERA_IP/32" dev lo; return 0
  fi
  if sudo -n true 2>/dev/null; then
    sudo -n ip addr del "$ASTRA_CAMERA_IP/32" dev lo 2>/dev/null
  fi
  if astra_camera_ip_on_host && [ -n "${WSL_DISTRO_NAME:-}" ] && command -v wsl.exe >/dev/null 2>&1; then
    info "using WSL's own admin door (wsl.exe -u root), no password needed"
    timeout 60 wsl.exe -d "$WSL_DISTRO_NAME" -u root -- ip addr del "$ASTRA_CAMERA_IP/32" dev lo >/dev/null 2>&1
  fi
  if astra_camera_ip_on_host && [ -t 0 ]; then
    sudo ip addr del "$ASTRA_CAMERA_IP/32" dev lo
  fi
  if astra_camera_ip_on_host; then
    die "Could not remove $ASTRA_CAMERA_IP from the loopback. Do it once by hand, then start again:
    sudo ip addr del $ASTRA_CAMERA_IP/32 dev lo
    or from Windows PowerShell:  wsl -d ${WSL_DISTRO_NAME:-Ubuntu-22.04} -u root ip addr del $ASTRA_CAMERA_IP/32 dev lo"
  fi
  mkdir -p "$ASTRA_DOCKER_DATA/state" && date > "$ASTRA_DOCKER_DATA/state/native_camera_ip_removed"
  ok "removed"
}

# astra_ip_to_int A.B.C.D : the IPv4 address as one number, for the subnet arithmetic below.
astra_ip_to_int() {
  local IFS=. a b c d
  read -r a b c d <<< "$1"
  echo $(( (${a:-0} << 24) + (${b:-0} << 16) + (${c:-0} << 8) + ${d:-0} ))
}

# astra_cidr_overlap NET : true when NET (A.B.C.D/N, or a bare address = /32) shares at least one address with
# the drone network ASTRA_SUBNET (192.168.144.0/24), in either direction: 192.168.144.25/32 (inside it) and
# 192.168.128.0/17 or 192.168.144.0/20 (around it) both overlap; 192.168.145.0/24 or IPv6 addresses do not.
astra_cidr_overlap() {
  local net="${1%/*}" len="${1#*/}" mine="${ASTRA_SUBNET%/*}" mylen="${ASTRA_SUBNET#*/}" short mask
  [ "$len" = "$1" ] && len=32
  case "$len" in ''|*[!0-9]*) return 1 ;; esac
  case "$net" in *[!0-9.]*|*..*|.*) return 1 ;; esac
  case "$net" in *.*.*.*) ;; *) return 1 ;; esac
  short=$(( len < mylen ? len : mylen ))
  mask=$(( short == 0 ? 0 : (0xFFFFFFFF << (32 - short)) & 0xFFFFFFFF ))
  [ $(( $(astra_ip_to_int "$net") & mask )) -eq $(( $(astra_ip_to_int "$mine") & mask )) ]
}

# astra_subnet_conflict : print a description of what else uses 192.168.144.0/24 on this computer ("" = free).
# Docker cannot create astra_net when another Docker network overlaps it ("Pool overlaps with other one on this
# address space"; Docker's own address pool hands out 192.168.144.0/20 once its 172.x networks are used up), and a
# network card on that range (the real drone's network, a VPN) would answer in the containers' place; the team's
# code needs exactly these addresses. A wider route that merely COVERS the range (a 192.168.0.0/16 VPN route) does
# not block the containers: astra_subnet_wider_route reports that one separately.
astra_subnet_conflict() {
  local dst rest len nets n sub s
  while read -r dst rest; do
    [ -n "$dst" ] || continue
    len="${dst#*/}"; [ "$len" = "$dst" ] && len=32
    case "$len" in ''|*[!0-9]*) continue ;; esac
    if [ "$len" -ge "${ASTRA_SUBNET#*/}" ] && astra_cidr_overlap "$dst"; then
      printf 'a network card of this computer: %s %s\n' "$dst" "$rest"; break
    fi
  done < <(ip -4 route show 2>/dev/null | awk '$1 != "default" && $0 !~ / dev br-/')
  command -v docker >/dev/null 2>&1 || return 0
  nets="$(docker network ls --format '{{.Name}}' 2>/dev/null)"
  for n in $nets; do
    [ "$n" = "$ASTRA_NET" ] && continue
    sub="$(docker network inspect "$n" --format '{{range .IPAM.Config}}{{.Subnet}} {{end}}' 2>/dev/null)"
    for s in $sub; do
      if astra_cidr_overlap "$s"; then printf 'the Docker network "%s" (%s)\n' "$n" "${sub% }"; break; fi
    done
  done
}

# astra_subnet_wider_route : print a route of this computer that is WIDER than the drone network and covers it
# (for example "192.168.0.0/16 dev tun0", a VPN), "" when there is none. The containers still start (the Docker
# network's own /24 route is more exact and wins), but while they run this computer cannot reach the
# 192.168.144.x part of that wider network. Docker's own bridges (br-...) and the default route are ignored.
astra_subnet_wider_route() {
  local dst rest len
  while read -r dst rest; do
    [ -n "$dst" ] || continue
    len="${dst#*/}"; [ "$len" = "$dst" ] && continue
    case "$len" in ''|*[!0-9]*) continue ;; esac
    if [ "$len" -lt "${ASTRA_SUBNET#*/}" ] && [ "$len" -gt 0 ] && astra_cidr_overlap "$dst"; then
      printf '%s %s\n' "$dst" "$rest"; return 0
    fi
  done < <(ip -4 route show 2>/dev/null | awk '$1 != "default" && $0 !~ / dev br-/')
}

# astra_native_sim_running : true when a NATIVE (not containerised) Gazebo or SITL is running on this computer.
astra_native_sim_running() {
  local pid
  for pid in $(pgrep -f 'arducopter|gz sim' 2>/dev/null); do
    grep -qE 'docker|containerd' "/proc/$pid/cgroup" 2>/dev/null || return 0
  done
  return 1
}

# astra_mem_check MB : warn when less than MB megabytes of memory are free.
astra_mem_check() {
  local avail
  avail="$(free -m 2>/dev/null | awk '/^Mem:/{print $7}')"
  if [ -n "$avail" ] && [ "$avail" -lt "${1:-4000}" ]; then
    warn "Only ${avail} MB of memory is free; the simulator wants about ${1:-4000} MB. Close other programs (or stop"
    warn "the native simulator: bash simulation/scripts/stop_sim.sh) if it gets slow."
  fi
}

# astra_free_gb DIR : free space (whole GB) of the file system that holds DIR (or its nearest existing parent).
astra_free_gb() {
  local d="$1"
  while [ -n "$d" ] && [ ! -e "$d" ]; do d="$(dirname "$d")"; done
  df -BG --output=avail "${d:-/}" 2>/dev/null | tail -1 | tr -dc '0-9'
}
