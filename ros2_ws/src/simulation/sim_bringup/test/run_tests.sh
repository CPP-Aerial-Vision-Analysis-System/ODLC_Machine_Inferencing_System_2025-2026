#!/usr/bin/env bash
# =============================================================================
#  sim_bringup/test/run_tests.sh  --  every quick check of sim_bringup and the simulation scripts
#
#  PURPOSE: one command a teammate runs after changing sim_bringup or simulation/scripts:
#    1. the pytest unit tests of sim_bringup (no simulator needed),
#    2. both launch files parse and list their arguments (ros2 launch ... --show-args),
#    3. bash -n and shellcheck on simulation/setup/*.sh and simulation/scripts/*.sh,
#    4. demo.sh --help and every script's --help work.
#  It never starts Gazebo or SITL (the headless acceptance test is in simulation/README.md "For developers").
#
#  HOW TO RUN:   bash ros2_ws/src/simulation/sim_bringup/test/run_tests.sh      (after 03_build.sh)
#  Exit code: 0 = all passed, 1 = something failed (listed at the end).
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
REPO="$(cd "$PKG/../../../.." && pwd)"
FAILED=()
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
check() { local name="$1"; shift; if "$@"; then echo " OK  $name"; else echo " XX  $name"; FAILED+=("$name"); fi; }

# shellcheck disable=SC1091
source "$REPO/simulation/setup/sim_env.sh"

step "1. pytest (sim_bringup/test)"
check "pytest" bash -c "cd /tmp && python3 -m pytest -q -p no:cacheprovider '$HERE'"

step "2. launch files parse"
check "sim.launch.py --show-args" bash -c "timeout 60 ros2 launch sim_bringup sim.launch.py --show-args > /dev/null"
check "team_stack.launch.py --show-args" bash -c "timeout 60 ros2 launch sim_bringup team_stack.launch.py --show-args > /dev/null"

step "3. shell scripts"
for f in "$REPO"/simulation/setup/*.sh "$REPO"/simulation/scripts/*.sh; do
  check "bash -n $(basename "$f")" bash -n "$f"
done
if command -v shellcheck > /dev/null; then
  check "shellcheck (warnings)" shellcheck -x -S warning "$REPO"/simulation/setup/*.sh "$REPO"/simulation/scripts/*.sh
else
  echo " --  shellcheck not installed (sudo apt install shellcheck)"
fi

step "4. --help of every script"
for f in demo start_sim stop_sim take_photo upload_mission fly_mission status; do
  check "$f.sh --help" bash -c "bash '$REPO/simulation/scripts/$f.sh' --help | grep -q PURPOSE"
done
check "03_build.sh --help" bash -c "bash '$REPO/simulation/setup/03_build.sh' --help | grep -q PURPOSE"

echo
if [ ${#FAILED[@]} -eq 0 ]; then echo "ALL CHECKS PASSED"; exit 0; fi
echo "FAILED: ${FAILED[*]}"
exit 1
