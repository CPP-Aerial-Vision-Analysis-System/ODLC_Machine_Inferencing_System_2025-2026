#!/usr/bin/env bash
# =============================================================================
#  01_install_humble.sh  --  STEP 1: install everything ASTRA needs (run once)
#
#  PURPOSE: turns a fresh Ubuntu 22.04 ("Jammy") into an ASTRA development
#  machine. It installs ROS 2 Humble (the robot software framework the drone's
#  Jetson runs), Gazebo Harmonic (the 3D simulator) with the ROS <-> Gazebo
#  bridge built for it, MAVROS (the bridge between ROS 2 and the ArduPilot
#  autopilot), and every library the simulator, the ArduPilot Gazebo plugin and
#  the SIYI camera emulator need. It also tells Mesa to draw 3D graphics with the
#  NVIDIA card when running inside WSL2.
#
#  WHY HUMBLE + UBUNTU 22.04: the drone's Jetson Orin Nano runs JetPack 6.2,
#  which is Ubuntu 22.04 + ROS 2 Humble + Python 3.10. Developing on exactly the
#  same versions means code written here runs on the drone without changes.
#
#  HOW TO RUN:   sudo bash 01_install_humble.sh
#     (WSL user without a sudo password? From Windows PowerShell:
#      wsl -d Ubuntu-22.04 -u root bash /path/to/simulation/setup/01_install_humble.sh)
#  Safe to run again: it skips what is already installed. If the internet drops
#  halfway, run it again and it continues.
#  NEXT STEP:    bash 02_setup_ardupilot.sh      (WITHOUT sudo), then bash 03_build.sh
# =============================================================================
set -eo pipefail
export DEBIAN_FRONTEND=noninteractive

# If Ubuntu is updating itself in the background, wait up to 10 minutes for it
# instead of failing with "Unable to acquire the dpkg frontend lock".
apt-get() { command apt-get -o DPkg::Lock::Timeout=600 "$@"; }
say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

if [ "$(id -u)" -ne 0 ]; then
  echo "This script installs software, so it needs admin power."
  echo "Run it like this:   sudo bash $0"
  exit 1
fi
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "jammy" ]; then
  echo "ASTRA needs Ubuntu 22.04 (codename jammy), like the Jetson. You have: ${PRETTY_NAME:-unknown}."
  exit 1
fi

say "[1/8] Language settings (UTF-8) and basic tools"
apt-get update
apt-get install -y locales software-properties-common curl wget ca-certificates gnupg lsb-release \
  git rsync build-essential cmake mesa-utils iputils-ping net-tools
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
add-apt-repository -y -n universe          # -n = do not refresh yet; step 3 refreshes

say "[2/8] Adding the official ROS 2 and Gazebo software sources"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  if [ -z "${ROS_APT_SOURCE_VERSION}" ]; then
    echo "Could not ask GitHub for the newest ros2-apt-source version (internet or rate limit). Try again in a minute."
    exit 1
  fi
  curl -fsSL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  apt-get install -y /tmp/ros2-apt-source.deb
fi
# Gazebo Harmonic is not the default simulator of Humble (that is Fortress), so it
# comes from the Gazebo team's own software source (packages.osrfoundation.org).
if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
  curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable ${VERSION_CODENAME} main" \
    > /etc/apt/sources.list.d/gazebo-stable.list
fi

say "[3/8] Refreshing the software list and updating Ubuntu"
apt-get update
apt-get upgrade -y

say "[4/8] ROS 2 Humble Desktop (includes RViz) + build tools"
apt-get install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-vcstool

say "[5/8] Gazebo Harmonic + the ROS <-> Gazebo bridge built for Harmonic"
# ros-humble-ros-gzharmonic replaces ros-humble-ros-gz (the Fortress version); never install both.
apt-get install -y gz-harmonic ros-humble-ros-gzharmonic
# Python access to Gazebo's message system (used by the SIYI camera emulator and test tools).
apt-get install -y python3-gz-transport13 python3-gz-msgs10 || echo "WARNING: Gazebo Python bindings not found under these names."

say "[6/8] MAVROS (ROS 2 <-> ArduPilot) + the geoid files it needs for GPS heights"
# MAVROS is currently missing from the main ROS apt repo for Humble, so (exactly like
# docker/Dockerfile) it comes from a dated, frozen ROS snapshot, pinned so that ONLY the
# MAVROS packages are taken from there. The Jetson has 2.12.0; the snapshot carries 2.14.0
# (mavros_msgs only gained ObstacleDistance3D; no existing msg/srv changed).
MAVROS_SNAPSHOT=2026-08-07
if [ ! -f /etc/apt/sources.list.d/ros-snapshot.list ]; then
  curl -fsSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA' \
    | gpg --dearmor -o /usr/share/keyrings/ros-snapshots.gpg
  echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.gpg] http://snapshots.ros.org/humble/${MAVROS_SNAPSHOT}/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros-snapshot.list
  printf 'Package: *\nPin: origin snapshots.ros.org\nPin-Priority: 100\n\nPackage: ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs ros-humble-libmavconn\nPin: origin snapshots.ros.org\nPin-Priority: 1001\n' \
    > /etc/apt/preferences.d/ros-snapshot-mavros
  apt-get update
fi
apt-get install -y ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs geographiclib-tools
if [ ! -e /usr/share/GeographicLib/geoids/egm96-5.pgm ] && [ ! -e /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]; then
  bash /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
fi

say "[7/8] Extra ROS pieces, libraries for the ArduPilot plugin, video streaming and Python"
apt-get install -y \
  ros-humble-xacro ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-rqt-image-view ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
  ros-humble-tf2-ros ros-humble-tf2-tools ros-humble-rviz2 ros-humble-teleop-twist-keyboard \
  ros-humble-launch-testing ros-humble-launch-testing-ament-cmake ros-humble-geographic-msgs \
  ros-humble-image-view ros-humble-vision-msgs
# Test tools of the simulation scripts: shellcheck (script checker), xvfb (an invisible screen, so the
# Gazebo window / RViz / demo panel can be tested without opening windows on the desktop).
apt-get install -y shellcheck xvfb
# The ArduPilot Gazebo plugin (built in step 2) and the camera emulator's video stream:
apt-get install -y \
  libgz-sim8-dev rapidjson-dev libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
  gstreamer1.0-libav gstreamer1.0-gl gir1.2-gst-rtsp-server-1.0 libgstrtspserver-1.0-dev python3-gi
apt-get install -y \
  python3-pip python3-venv python3-numpy python3-scipy python3-yaml python3-pil python3-opencv \
  python3-pytest python3-serial python3-piexif python3-pyqt5 python3-watchdog python3-requests \
  python3-lxml python3-future python3-pexpect python3-empy iproute2

# The team's Python stack, same pins as docker/requirements-*.txt (= the Jetson). torch comes
# from PyTorch's CUDA 12.6 index so the detector can use this laptop's NVIDIA GPU.
# numpy must stay < 2: ROS Humble's cv_bridge is built against numpy 1.x.
REPO_DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../docker" 2>/dev/null && pwd || true)"
if [ -f "${REPO_DOCKER_DIR}/requirements-dev.txt" ]; then
  python3 -m pip install -r "${REPO_DOCKER_DIR}/requirements-dev.txt"
  python3 -m pip install --index-url https://download.pytorch.org/whl/cu126 torch==2.8.0 torchvision==0.23.0
  python3 -m pip install -r "${REPO_DOCKER_DIR}/requirements-ml.txt"
else
  echo "WARNING: docker/requirements-*.txt not found next to this script; skipping the Python ML stack."
fi

say "[8/8] Graphics card settings for WSL2, simulator networking, rosdep"
# Inside WSL2 the fast path to the NVIDIA card is Mesa's "d3d12" driver (it talks to
# Windows DirectX). Without these two lines Gazebo draws on the CPU ("llvmpipe"): slow.
cat > /etc/profile.d/astra_wsl_gpu.sh <<'EOF'
# ASTRA: draw 3D graphics with the NVIDIA card when running inside WSL2 (see simulation/setup/01_install_humble.sh)
if grep -qi microsoft /proc/version 2>/dev/null; then
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
fi
EOF
# profile.d is only read by login shells; also load it from every interactive bash shell.
if ! grep -q astra_wsl_gpu /etc/bash.bashrc; then
  printf '\n# ASTRA simulator: use the NVIDIA GPU in WSL2\n[ -f /etc/profile.d/astra_wsl_gpu.sh ] && . /etc/profile.d/astra_wsl_gpu.sh\n' >> /etc/bash.bashrc
fi
# The installer runs as root; make sure the normal user still owns their home folder.
if [ -n "${SUDO_USER:-}" ] && [ "${SUDO_USER}" != "root" ]; then
  chown -R "${SUDO_USER}:${SUDO_USER}" "/home/${SUDO_USER}" 2>/dev/null || true
fi
# The SIYI camera emulator serves the camera's media web page on port 82, like the real camera.
# Ports below 1024 normally need root; allow ports from 80 up for normal users (Docker
# containers already allow this by default).
echo 'net.ipv4.ip_unprivileged_port_start=80' > /etc/sysctl.d/60-astra-sim.conf
sysctl -p /etc/sysctl.d/60-astra-sim.conf || true
# The team's camera code talks to 192.168.144.25 (the real camera's address): give this PC that
# address on the loopback device, now and at every start (a small systemd service).
if [ -f "$(dirname "${BASH_SOURCE[0]}")/camera_ip_alias.sh" ]; then
  bash "$(dirname "${BASH_SOURCE[0]}")/camera_ip_alias.sh" || echo "WARNING: could not add the camera address (see above)"
fi
# Ultralytics (the team's detector) wants a settings folder in the user's home.
if [ -n "${SUDO_USER:-}" ] && [ "${SUDO_USER}" != "root" ]; then
  install -d -o "$SUDO_USER" -g "$SUDO_USER" "/home/${SUDO_USER}/.config" "/home/${SUDO_USER}/.config/Ultralytics"
fi
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init || true; fi

say "ALL DONE: ROS 2 Humble + Gazebo Harmonic + MAVROS are installed."
echo "Check:  source /opt/ros/humble/setup.bash && gz sim --versions && ros2 pkg prefix mavros"
echo "Next step:  bash 02_setup_ardupilot.sh   (WITHOUT sudo)"
