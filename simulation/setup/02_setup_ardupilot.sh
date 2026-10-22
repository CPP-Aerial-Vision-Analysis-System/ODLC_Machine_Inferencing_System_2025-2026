#!/usr/bin/env bash
# =============================================================================
#  02_setup_ardupilot.sh  --  STEP 2: the simulated autopilot (run once)
#
#  PURPOSE: installs the pieces that make the simulator behave like the real
#  Pixhawk flight controller:
#    1. ArduCopter SITL ("software in the loop"): the same ArduPilot firmware the
#       Pixhawk runs, built to run on a PC. Downloaded prebuilt into ~/sitl.
#    2. The ArduPilot Gazebo plugin (github.com/ArduPilot/ardupilot_gazebo): the
#       glue that lets SITL fly a drone inside Gazebo Harmonic, built in
#       ~/ardupilot_gazebo against the Gazebo installed by step 1.
#    3. ArduPilot's default parameter files for an Iris quadcopter in Gazebo.
#    4. MAVProxy + pymavlink (a text ground station and the MAVLink Python
#       library) in their own Python environment, ~/sitl/venv.
#  With these, MAVROS and the team's flight code talk to SITL exactly the way
#  they talk to the real Pixhawk (MAVLink), so nothing in the flight code changes.
#
#  HOW TO RUN:   bash 02_setup_ardupilot.sh        (WITHOUT sudo; step 1 first)
#  Safe to run again: it skips what is already there.
# =============================================================================
set -eo pipefail
say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }

if [ "$(id -u)" -eq 0 ]; then echo "Run this WITHOUT sudo:  bash $0"; exit 1; fi
if [ ! -f /opt/ros/humble/setup.bash ]; then echo "ROS 2 Humble is missing: run step 1 first (sudo bash 01_install_humble.sh)."; exit 1; fi
source /opt/ros/humble/setup.bash

say "[1/4] ArduCopter SITL (the simulated Pixhawk) in ~/sitl"
# Same ArduCopter version as the real Pixhawk (4.6.3, see docker/Dockerfile ARDUPILOT_REF).
ARDUCOPTER_VERSION=4.6.3
mkdir -p "$HOME/sitl"
if [ ! -x "$HOME/sitl/arducopter" ]; then
  curl -fL -o "$HOME/sitl/arducopter" \
    "https://firmware.ardupilot.org/Copter/stable-${ARDUCOPTER_VERSION}/SITL_x86_64_linux_gnu/arducopter" \
  || curl -fL -o "$HOME/sitl/arducopter" \
    "https://firmware.ardupilot.org/Copter/stable/SITL_x86_64_linux_gnu/arducopter"
  chmod +x "$HOME/sitl/arducopter"
fi
# (No pipe here: "strings | grep -m1" made pipefail report a false failure when grep stopped early.)
VER_LINE=$(grep -a -o -m1 -E 'ArduCopter V[0-9]+\.[0-9]+\.[0-9]+[^ ]* \([0-9a-f]+\)' "$HOME/sitl/arducopter" || true)
if [ -n "$VER_LINE" ]; then echo "${VER_LINE%%$'\n'*}"; else echo "WARNING: could not read the ArduCopter version"; fi

say "[2/4] ArduPilot Gazebo plugin in ~/ardupilot_gazebo (built for Gazebo Harmonic)"
AG="$HOME/ardupilot_gazebo"
if [ ! -d "$AG/.git" ]; then
  git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$AG"
fi
export GZ_VERSION=harmonic
mkdir -p "$AG/build" && cd "$AG/build"
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo > cmake.log 2>&1 || { tail -30 cmake.log; echo "cmake FAILED (see $AG/build/cmake.log)"; exit 1; }
make -j"$(nproc)" 2>&1 | tail -3
ls -1 "$AG"/build/lib*.so
ldd "$AG/build/libArduPilotPlugin.so" | grep -m1 'libgz-sim8' || { echo "plugin is not linked against Gazebo Harmonic (gz-sim8)"; exit 1; }

say "[3/4] ArduPilot parameter files for an Iris in Gazebo"
cd "$HOME/sitl"
# Taken from the same release as the SITL binary (Copter-4.6.3), master only as a fallback.
RAW_TAG=https://raw.githubusercontent.com/ArduPilot/ardupilot/Copter-${ARDUCOPTER_VERSION}/Tools/autotest/default_params
RAW_MASTER=https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/autotest/default_params
for f in copter.parm gazebo-iris.parm; do
  [ -f "$f" ] || curl -fsSL -o "$f" "$RAW_TAG/$f" || curl -fsSL -o "$f" "$RAW_MASTER/$f"
done
cp -n "$AG/config/gazebo-iris-gimbal.parm" . 2>/dev/null || true
ls -1 ./*.parm

say "[4/4] MAVProxy + pymavlink in ~/sitl/venv"
[ -x "$HOME/sitl/venv/bin/python" ] || python3 -m venv "$HOME/sitl/venv"
"$HOME/sitl/venv/bin/python" -m pip install --quiet --upgrade pip
# "future": MAVProxy 1.8.74 imports future.builtins (rline.py) but does not declare it; without it
# mavproxy.py stops at start with "No module named 'future'".
"$HOME/sitl/venv/bin/python" -m pip install --quiet MAVProxy==1.8.74 pymavlink==2.4.49 "numpy<2" future==1.0.0
"$HOME/sitl/venv/bin/python" -c "import pymavlink; print('pymavlink', pymavlink.__version__)"
"$HOME/sitl/venv/bin/mavproxy.py" --help > /dev/null || { echo "mavproxy.py does not start (see the error above)"; exit 1; }
echo "mavproxy.py starts OK"

say "ALL DONE: SITL, the ArduPilot Gazebo plugin, parameters and MAVProxy are ready."
