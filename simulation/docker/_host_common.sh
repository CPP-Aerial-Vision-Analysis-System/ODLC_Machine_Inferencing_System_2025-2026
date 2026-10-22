# shellcheck shell=bash
# =============================================================================
#  _host_common.sh  --  helpers for the Docker scripts that run on the HOST (sourced, never run)
#
#  PURPOSE: the checks and choices every host-side Docker script (build_images.sh, compose_up.sh,
#  compose_down.sh, run_sim_container.sh, run_jetson_container.sh) needs, written once:
#    * friendly messages (say / info / ok / warn / die), the same style as the other simulation scripts;
#    * astra_need_docker: is "docker" really Docker Engine (not the Windows Docker Desktop shim), may this
#      user use it (the "docker" group), does Compose exist and is it new enough;
#    * astra_pick_repo: which repo folder to mount into the containers (on Windows the Linux-side copy
#      ~/ODLC_Machine_Inferencing_System_2025-2026, because the D: drive is slow and has a space in its path);
#    * astra_prepare_data: the host folder for runs, logs and detector weights (~/.astra_docker);
#    * astra_display_mode / astra_gui_run_args / astra_gpu_run_args: how containers show windows (WSLg on
#      Windows 11, X11 on Linux, none) and get the graphics card (DirectX via /dev/dxg, /dev/dri, NVIDIA CUDA);
#    * astra_free_camera_ip: the native simulator gives this computer the camera address 192.168.144.25 on
#      its loopback; while that is there, the "sim" container (which IS 192.168.144.25 on the Docker network)
#      gets no internet and the host answers in its place, so it is taken away (the native scripts put it
#      back by themselves the next time they run);
#    * astra_run: run a command, or only print it with --dry-run.
# =============================================================================

ASTRA_DOCKER_SCRIPTS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_REPO_HERE="$(cd "$ASTRA_DOCKER_SCRIPTS/../.." && pwd)"   # the repo these scripts belong to
ASTRA_LINUX_COPY="$HOME/ODLC_Machine_Inferencing_System_2025-2026"
ASTRA_CONTAINER_REPO=/ODLC_Machine_Inferencing_System_2025-2026
ASTRA_CAMERA_IP=192.168.144.25                                 # real SIYI camera = the "sim" container
ASTRA_JETSON_IP=192.168.144.100                                # real Jetson eth0 = the "jetson" container
ASTRA_NET=astra_net
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
  where="$(command -v docker 2>/dev/null || true)"
  if [ -z "$where" ]; then
    die "Docker is not installed in this Ubuntu. Install Docker Engine once (admin rights):
    sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh
    (from Windows PowerShell:  wsl -d ${WSL_DISTRO_NAME:-Ubuntu-22.04} -u root -- bash <that file> --user $(id -un))"
  fi
  case "$where" in
    /mnt/*) die "\"docker\" here is Docker Desktop's Windows program ($where), not Docker Engine.
    Install Docker Engine inside this Ubuntu:  sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh
    then open a NEW terminal (type -a docker must show /usr/bin/docker first)." ;;
  esac
  [ "$ASTRA_DRY" = 1 ] && return 0
  local err
  if ! err="$(docker info 2>&1 >/dev/null)"; then
    case "$err" in
      *"permission denied"*)
        die "You may not use Docker yet: your user is not in the \"docker\" group in THIS terminal.
    Close every Ubuntu window, run  wsl --terminate ${WSL_DISTRO_NAME:-Ubuntu-22.04}  in PowerShell, open Ubuntu again.
    (Right now, for one command:  sg docker -c \"bash $0\")" ;;
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
  if [ "${major:-0}" -lt 2 ] 2>/dev/null || { [ "${major:-0}" -eq 2 ] && [ "${minor:-0}" -lt 24 ]; } 2>/dev/null; then
    die "Docker Compose $v is too old (2.24 or newer is needed). Install the current one from Docker's apt source:
    sudo bash $ASTRA_DOCKER_SCRIPTS/install_docker_engine.sh"
  fi
}

astra_image_exists() { docker image inspect "$1" >/dev/null 2>&1; }

# astra_require_images "IMAGE TARGET"... : stop when an image is missing and say how to build it.
astra_require_images() {
  local pair img target missing=()
  [ "$ASTRA_DRY" = 1 ] && return 0
  for pair in "$@"; do
    img="${pair% *}"; target="${pair#* }"
    astra_image_exists "$img" || missing+=("$img (bash simulation/docker/build_images.sh $target)")
  done
  if [ "${#missing[@]}" -gt 0 ]; then
    warn "These Docker images are not built yet:"
    for img in "${missing[@]}"; do warn "    $img"; done
    die "Build them first (the first build takes about an hour; afterwards only changed steps are rebuilt)."
  fi
}

# astra_pick_repo [FOLDER] : print the repo folder to mount into the containers.
# On Windows the scripts usually live on D: (/mnt/d/..., slow, path with a space); the Linux-side copy that
# simulation/setup/03_build.sh keeps (~/ODLC_Machine_Inferencing_System_2025-2026) is used instead when it exists.
astra_pick_repo() {
  local want="${1:-}"
  if [ -n "$want" ]; then
    [ -d "$want/ros2_ws/src" ] || die "--repo $want: no ros2_ws/src there"
    (cd "$want" && pwd); return 0
  fi
  case "$ASTRA_REPO_HERE" in
    /mnt/*)
      if [ -d "$ASTRA_LINUX_COPY/ros2_ws/src" ]; then
        printf '%s\n' "$ASTRA_LINUX_COPY"
      else
        warn "No Linux-side copy ($ASTRA_LINUX_COPY) yet: mounting the Windows folder (works, but slower)."
        warn "Make the copy with:  bash $ASTRA_DOCKER_SCRIPTS/compose_up.sh --sync-only   (or setup/03_build.sh)"
        printf '%s\n' "$ASTRA_REPO_HERE"
      fi ;;
    *) printf '%s\n' "$ASTRA_REPO_HERE" ;;
  esac
}

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
  astra_run rsync -a --delete \
    --exclude '.git/' --exclude '__pycache__/' --exclude '*.pyc' \
    --exclude 'ros2_ws/build/' --exclude 'ros2_ws/install/' --exclude 'ros2_ws/log/' \
    --exclude 'ros2_ws/logs/' --exclude 'ros2_ws/terrain/' --exclude 'ros2_ws/*.tlog' --exclude 'ros2_ws/*.bin' \
    --exclude 'ros2_ws/src/video_cam/mapping_photos/' --exclude 'ros2_ws/src/video_cam/detection_results_sahi/' \
    --exclude 'ros2_ws/src/video_cam/camera_feed/' --exclude 'camera_feed/' \
    "$ASTRA_REPO_HERE/" "$ASTRA_LINUX_COPY/"
  [ "$ASTRA_DRY" = 1 ] || ok "copied ($(du -sh "$ASTRA_LINUX_COPY" 2>/dev/null | cut -f1))"
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
  elif [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    echo x11
  else
    echo none
  fi
}

# astra_has_nvidia_runtime : true when Docker can give containers the NVIDIA card (--gpus all).
astra_has_nvidia_runtime() {
  docker info --format '{{range $k, $v := .Runtimes}}{{$k}} {{end}}' 2>/dev/null | tr ' ' '\n' | grep -qx nvidia
}

# astra_gui_run_args MODE : fill the array GUI_ARGS with `docker run` options for windows + 3D drawing.
astra_gui_run_args() {
  GUI_ARGS=()
  case "$1" in
    wslg)
      GUI_ARGS=(-e "DISPLAY=${DISPLAY:-:0}" -e "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}"
                -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e PULSE_SERVER=unix:/mnt/wslg/PulseServer
                -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg
                -v /usr/lib/wsl:/usr/lib/wsl:ro --device /dev/dxg -e LD_LIBRARY_PATH=/usr/lib/wsl/lib
                -e GALLIUM_DRIVER=d3d12 -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA -e QT_X11_NO_MITSHM=1) ;;
    x11)
      GUI_ARGS=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -e QT_X11_NO_MITSHM=1
                -e XDG_RUNTIME_DIR=/tmp/runtime-ubuntu)
      [ -e /dev/dri ] && GUI_ARGS+=(--device /dev/dri) ;;
  esac
}

# astra_gpu_run_args MODE WANT : fill GPU_ARGS with `--gpus all ...` when WANT=yes|auto and Docker has the
# NVIDIA runtime (MODE x11 also asks for the OpenGL driver: capability "all").
astra_gpu_run_args() {
  GPU_ARGS=()
  case "$2" in no) return 0 ;; esac
  if astra_has_nvidia_runtime; then
    GPU_ARGS=(--gpus all -e "NVIDIA_DRIVER_CAPABILITIES=$([ "$1" = x11 ] && echo all || echo compute,utility)")
  elif [ "$2" = yes ]; then
    die "--gpu was asked for, but Docker has no NVIDIA runtime (install_docker_engine.sh --nvidia installs it)"
  fi
}

# astra_x11_allow : let containers of this user open windows on a Linux X11 desktop.
astra_x11_allow() {
  if command -v xhost >/dev/null 2>&1; then
    astra_run xhost "+SI:localuser:$(id -un)" >/dev/null 2>&1 || warn "xhost failed: windows may not open (no X11 desktop?)"
  else
    warn "xhost is missing (sudo apt-get install x11-xserver-utils): windows may be refused by the X server"
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
