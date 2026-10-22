# shellcheck shell=bash
# =============================================================================
#  sim_env.sh  --  the simulator's settings for THIS terminal (source it, do not run it)
#
#  PURPOSE: one line that prepares a terminal for the ASTRA simulator, the same way every time:
#    * finds the team repo you run from (the Linux-side copy ~/ODLC_Machine_Inferencing_System_2025-2026
#      made by 03_build.sh, preferred over the Windows copy on /mnt/d, which only holds the source),
#    * turns on ROS 2 Humble and the repo's workspace (ros2_ws/install), so `ros2 launch sim_bringup ...`
#      and the team's packages are found,
#    * tells Gazebo where the ArduPilot plugin and its models are (~/ardupilot_gazebo),
#    * makes Gazebo/RViz draw with the NVIDIA graphics card inside WSL2 (Mesa's d3d12 driver),
#    * keeps the simulator's Gazebo messages in their own "partition" (astra_sim), so another Gazebo
#      running on this PC never mixes in,
#    * points to SITL (~/sitl/arducopter) and MAVProxy (~/sitl/venv) from 02_setup_ardupilot.sh.
#  It never changes ROS_DOMAIN_ID (the team's flight stack and the simulator must use the same one; the
#  default 0 is what the drone uses). Every setting can be overridden by exporting it BEFORE sourcing.
#
#  HOW TO USE:   source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh
#  (all scripts in simulation/scripts do this for you)
# =============================================================================

_astra_setup_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_astra_repo_guess="$(cd "$_astra_setup_dir/../.." && pwd)"
_astra_linux_copy="$HOME/ODLC_Machine_Inferencing_System_2025-2026"

# ---- 1. which copy of the repo runs --------------------------------------------------------
# The Windows copy (/mnt/d/...) has a space in its path and a slow file system, so the simulator runs
# from the Linux-side copy that 03_build.sh keeps in sync. If this file is sourced from /mnt/..., use the
# Linux copy when it exists.
if [ -z "${ASTRA_REPO:-}" ]; then
  case "$_astra_repo_guess" in
    /mnt/*)
      if [ -d "$_astra_linux_copy/ros2_ws" ]; then ASTRA_REPO="$_astra_linux_copy"; else ASTRA_REPO="$_astra_repo_guess"; fi ;;
    *) ASTRA_REPO="$_astra_repo_guess" ;;
  esac
fi
export ASTRA_REPO
export ASTRA_SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"

# ---- 2. ROS 2 Humble + the workspace ---------------------------------------------------------
# ROS's setup files use unset variables; switch "nounset" off while sourcing them (and restore it).
_astra_nounset="$(set +o | grep nounset)"
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[sim_env] ROS 2 Humble is not installed (/opt/ros/humble). Run simulation/setup/01_install_humble.sh first."
fi
if [ -f "$ASTRA_REPO/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$ASTRA_REPO/ros2_ws/install/setup.bash"
  ASTRA_WS_BUILT=yes
else
  ASTRA_WS_BUILT=no
fi
export ASTRA_WS_BUILT
eval "$_astra_nounset"

# ---- 3. ArduPilot Gazebo plugin + SITL + MAVProxy -------------------------------------------
_astra_prepend() {   # _astra_prepend VAR DIR : put DIR in front of the colon list VAR (no duplicates)
  local current="${!1:-}"
  case ":$current:" in
    *":$2:"*) ;;
    *) export "$1"="$2${current:+:$current}" ;;
  esac
}
if [ -z "${ARDUPILOT_GAZEBO_DIR:-}" ]; then
  for _astra_d in "$HOME/ardupilot_gazebo" /opt/ardupilot_gazebo; do
    if [ -f "$_astra_d/build/libArduPilotPlugin.so" ]; then ARDUPILOT_GAZEBO_DIR="$_astra_d"; break; fi
  done
fi
if [ -n "${ARDUPILOT_GAZEBO_DIR:-}" ]; then
  export ARDUPILOT_GAZEBO_DIR
  _astra_prepend GZ_SIM_SYSTEM_PLUGIN_PATH "$ARDUPILOT_GAZEBO_DIR/build"
  _astra_prepend GZ_SIM_RESOURCE_PATH "$ARDUPILOT_GAZEBO_DIR/worlds"
  _astra_prepend GZ_SIM_RESOURCE_PATH "$ARDUPILOT_GAZEBO_DIR/models"
fi
export GZ_VERSION=harmonic
if [ -z "${ARDUPILOT_SITL_BIN:-}" ] && [ -x "$HOME/sitl/arducopter" ]; then export ARDUPILOT_SITL_BIN="$HOME/sitl/arducopter"; fi
if [ -z "${ASTRA_MAVPROXY:-}" ] && [ -x "$HOME/sitl/venv/bin/mavproxy.py" ]; then export ASTRA_MAVPROXY="$HOME/sitl/venv/bin/mavproxy.py"; fi
# The MAVProxy venv goes at the END of PATH: `python3` must stay the system Python that ROS uses.
if [ -d "$HOME/sitl/venv/bin" ]; then
  case ":$PATH:" in *":$HOME/sitl/venv/bin:"*) ;; *) export PATH="$PATH:$HOME/sitl/venv/bin" ;; esac
fi

# ---- 4. graphics (WSL2) and Gazebo messaging ------------------------------------------------
if grep -qi microsoft /proc/version 2>/dev/null; then
  # Without these two, WSL draws 3D on the CPU (llvmpipe) or on the Intel chip: slow.
  export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"
  # RViz's 3D view and Qt windows are more reliable on X11 than on Wayland inside WSLg.
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi
export GZ_PARTITION="${GZ_PARTITION:-astra_sim}"
export GZ_IP="${GZ_IP:-127.0.0.1}"

# ---- 5. small comforts ---------------------------------------------------------------------
# Ultralytics (the team's detector) wants a settings folder it can write; it adds "Ultralytics" itself,
# so this gives the standard ~/.config/Ultralytics/settings.json.
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$HOME/.config}"
mkdir -p "$YOLO_CONFIG_DIR" "$ASTRA_SIM_HOME" 2>/dev/null
# Python output appears in logs immediately (not in big delayed chunks).
export PYTHONUNBUFFERED=1

if [[ $- == *i* ]] || [ "${ASTRA_SIM_VERBOSE:-0}" = 1 ]; then
  echo "[sim_env] repo: $ASTRA_REPO (workspace built: $ASTRA_WS_BUILT) | ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} |" \
       "GZ_PARTITION=$GZ_PARTITION | plugin: ${ARDUPILOT_GAZEBO_DIR:-NOT FOUND} | state: $ASTRA_SIM_HOME"
fi
unset _astra_setup_dir _astra_repo_guess _astra_linux_copy _astra_nounset _astra_d
