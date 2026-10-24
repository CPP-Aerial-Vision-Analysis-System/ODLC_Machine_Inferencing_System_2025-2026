#!/usr/bin/env bash
# =============================================================================
#  simulation/setup/check_ros.sh  --  "is ROS 2 ready in THIS terminal?" in plain words
#
#  PURPOSE: the most common first-day build error is
#      CMake Error ... Could not find a package configuration file provided by "ament_cmake"
#  (or "ament_cmake_python", "rosidl_default_generators" ...). It means the terminal running colcon/cmake does not
#  have ROS 2 Humble loaded: ROS is not installed there, or it is installed but "source /opt/ros/humble/setup.bash"
#  was never run in this terminal, or the build is being run somewhere ROS cannot live (Windows PowerShell, macOS).
#  This script checks each possibility, changes nothing, and tells you the ONE thing to do.
#
#  USAGE:   bash simulation/setup/check_ros.sh        (exit code 0 = ready, 1 = something to fix)
#  The normal build (bash simulation/setup/03_build.sh) loads ROS itself and stops with a clear message if it is
#  missing; use that instead of typing colcon by hand, or load ROS first (the line this script prints).
# =============================================================================
fails=0
ok()   { printf '  [ OK ]  %s\n' "$*"; }
bad()  { printf '  [FAIL]  %s\n' "$*"; fails=$((fails + 1)); }
tip()  { printf '          -> %s\n' "$*"; }
note() { printf '  [ .. ]  %s\n' "$*"; }

echo "ROS 2 Humble check for this terminal"
echo "------------------------------------"

# 1. Where are we? ROS 2 Humble builds on Ubuntu 22.04 (native, WSL2 or Docker): not on plain Windows or macOS.
case "$(uname -s 2>/dev/null)" in
  Linux) ;;
  Darwin) bad "this is macOS: ROS 2 Humble cannot be built here"
          tip "use the Docker dev container (VS Code: Reopen in Container, choose 'ASTRA ODLC dev'); docs/simulation/DOCKER.md"
          exit 1 ;;
  *)     bad "this is not Linux (it looks like Windows): ROS 2 Humble cannot be built here"
         tip "open an Ubuntu 22.04 terminal in WSL2:  wsl -d Ubuntu-22.04   (docs/simulation/DOCKER.md, step A1), or use the Docker dev container"
         exit 1 ;;
esac
if [ -r /etc/os-release ]; then
  . /etc/os-release
  if [ "$ID" = ubuntu ] && [ "$VERSION_ID" = "22.04" ]; then ok "Ubuntu 22.04 (what ROS 2 Humble needs)"
  else bad "this is ${PRETTY_NAME:-an unknown Linux}; ROS 2 Humble packages are built for Ubuntu 22.04"
       tip "use an Ubuntu 22.04 WSL distro (wsl --install -d Ubuntu-22.04) or the Docker dev container"; fi
fi
[ -f /.dockerenv ] && note "running inside a Docker container"
grep -qi microsoft /proc/version 2>/dev/null && note "running inside WSL2"

# 2. Is Humble installed?
if [ -f /opt/ros/humble/setup.bash ]; then
  ok "ROS 2 Humble is installed (/opt/ros/humble)"
else
  bad "ROS 2 Humble is NOT installed here (no /opt/ros/humble/setup.bash)"
  tip "install it once:  sudo bash simulation/setup/01_install_humble.sh        (or use the Docker dev container, which has it)"
  [ -d /opt/ros ] && tip "found other ROS versions in /opt/ros: $(ls /opt/ros | tr '\n' ' '): this project needs humble"
fi

# 3. Is it LOADED in this terminal? (installed but not sourced is the usual cause)
if [ -n "${ROS_DISTRO:-}" ]; then
  if [ "$ROS_DISTRO" = humble ]; then ok "ROS is loaded in this terminal (ROS_DISTRO=humble)"
  else bad "ROS_DISTRO is '$ROS_DISTRO', not humble: a different ROS is loaded"
       tip "open a fresh terminal, then:  source /opt/ros/humble/setup.bash"; fi
else
  bad "ROS is not loaded in this terminal (ROS_DISTRO is empty)"
  tip "run this, then build again:   source /opt/ros/humble/setup.bash"
  tip "make it automatic for every new terminal:   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
fi

# 4. Look at it the way CMake does: after loading ROS, can ament_cmake be found?
if [ -f /opt/ros/humble/setup.bash ]; then
  (set +u; . /opt/ros/humble/setup.bash >/dev/null 2>&1
   if ros2 pkg prefix ament_cmake >/dev/null 2>&1; then exit 0; else exit 1; fi)
  if [ $? -eq 0 ]; then ok "ament_cmake is found once ROS is loaded"
  else bad "ament_cmake is missing from this ROS install"
       tip "sudo apt-get install -y ros-humble-ament-cmake ros-humble-ament-cmake-python   (or re-run 01_install_humble.sh)"; fi
fi

# 5. The build tool and the folder it builds in.
if command -v colcon >/dev/null 2>&1; then ok "colcon is installed"
else bad "colcon is not installed"; tip "sudo apt-get install -y python3-colcon-common-extensions"; fi
case "$PWD" in *" "*) note "this folder's path has a space: $PWD  (builds are more reliable in a path without spaces; 03_build.sh copies the repo for you)";; esac
case "$PWD" in /mnt/[a-z]/*) note "you are on a Windows drive (/mnt/...): builds there are slow and fragile; use bash simulation/setup/03_build.sh";; esac

echo "------------------------------------"
if [ "$fails" -eq 0 ]; then echo "Everything needed for a ROS 2 build is in place in this terminal."; exit 0; fi
echo "$fails thing(s) to fix (see the arrows above), then run this check again."
exit 1
