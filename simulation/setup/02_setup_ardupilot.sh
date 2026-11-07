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
#  Exit codes:   0 = done, 1 = a step failed (which one, why and the fix are printed). docs/simulation/ERRORS.md
# =============================================================================
set -eo pipefail
say() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; STEP="$*"; }
# err WHAT WHY FIX : one error in the shared shape (what, likely cause, fix), then stop.
err() { trap - EXIT; printf '\033[1;31m XX [02_setup_ardupilot] ERROR: %s.\n    Likely cause: %s.\n    Fix: %s\033[0m\n' "$1" "$2" "$3" >&2; exit 1; }
STEP="start"
# Any other command that fails (set -e) still says which step it was and what to do.
# shellcheck disable=SC2154  # (rc is set inside the trap text itself)
trap 'rc=$?; [ $rc -eq 0 ] || printf "\033[1;31m XX [02_setup_ardupilot] ERROR: the step \"%s\" failed (exit %s; the reason is above).\n    Likely cause: no internet (a download or git clone), or a missing build tool.\n    Fix: check the internet and run it again (it skips what is done):  bash \"%s\"\033[0m\n" "$STEP" "$rc" "$0" >&2' EXIT
case "${1:-}" in -h|--help) trap - EXIT; awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"; exit 0 ;; esac

if [ "$(id -u)" -eq 0 ]; then err "this must not run as root" "it was started with sudo; SITL and the plugin belong in your own home folder" "run it WITHOUT sudo:  bash \"$0\""; fi
if [ ! -f /opt/ros/humble/setup.bash ]; then err "ROS 2 Humble is not installed (/opt/ros/humble/setup.bash is missing)" "setup step 1 has not run on this PC (or it failed)" "sudo bash \"$(dirname "$0")/01_install_humble.sh\"   then run this again"; fi
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
if [ -n "$VER_LINE" ]; then echo "${VER_LINE%%$'\n'*}"; else echo "[02_setup_ardupilot] WARNING: could not read the ArduCopter version in ~/sitl/arducopter. Consequence: none if it starts; if SITL fails later, delete ~/sitl/arducopter and run this again." >&2; fi

say "[2/4] ArduPilot Gazebo plugin in ~/ardupilot_gazebo (built for Gazebo Harmonic)"
AG="$HOME/ardupilot_gazebo"
if [ ! -d "$AG/.git" ]; then
  git clone --depth 1 https://github.com/ArduPilot/ardupilot_gazebo.git "$AG"
fi
export GZ_VERSION=harmonic
mkdir -p "$AG/build" && cd "$AG/build"
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo > cmake.log 2>&1 || { tail -30 cmake.log; err "cmake could not configure the ArduPilot Gazebo plugin (log: $AG/build/cmake.log)" "a Gazebo Harmonic development package is missing (step 1 installs them)" "sudo bash \"$(dirname "$0")/01_install_humble.sh\", then run this again"; }
make -j"$(nproc)" 2>&1 | tail -3
ls -1 "$AG"/build/lib*.so
ldd "$AG/build/libArduPilotPlugin.so" | grep -m1 'libgz-sim8' || err "the plugin is not linked against Gazebo Harmonic (gz-sim8)" "an older Gazebo (Garden / Fortress) was found when it was built" "rm -rf $AG/build, then run this again (it builds with GZ_VERSION=harmonic)"

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
"$HOME/sitl/venv/bin/mavproxy.py" --help > /dev/null || err "mavproxy.py does not start (its error is above)" "a Python package in ~/sitl/venv is missing or broken" "rm -rf ~/sitl/venv, then run this again"
echo "mavproxy.py starts OK"
trap - EXIT

say "ALL DONE: SITL, the ArduPilot Gazebo plugin, parameters and MAVProxy are ready."
