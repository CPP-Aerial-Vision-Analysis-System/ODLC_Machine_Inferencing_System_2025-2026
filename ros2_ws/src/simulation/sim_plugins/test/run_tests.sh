#!/usr/bin/env bash
# =============================================================================
#  sim_plugins/test/run_tests.sh  --  every quick check of the plug-and-play part of the simulator
#
#  PURPOSE: one command a teammate runs after changing sim_plugins, the profiles, the plug-in templates or the
#  scripts that use them. It never starts Gazebo, SITL or any ROS node:
#    1. pytest: sim_plugins (profiles, parameter files, autopilot slot, plans, contracts, check_slot with fakes,
#       the SIYI probe, the tracker filter, launch-file evaluation with a fake launch context, fetch_sitl.sh with
#       a fake curl) and the two templates in sim_plugin_examples;
#    2. every profile in simulation/profiles validates;
#    3. bash -n + shellcheck on the scripts this work touches (demo.sh, start_sim.sh, fetch_sitl.sh, this file);
#    4. fetch_sitl.sh --dry-run for a few versions (no internet needed);
#    5. THE DEFAULT PLAN EQUALS TODAY'S COMMANDS: demo.sh --dry-run and start_sim.sh --dry-run print exactly the
#       same commands with and without --profile default (needs the workspace built: setup/03_build.sh).
#  It tests the files of the repo copy it lives in (sim_plugins from source, not an old installed copy).
#
#  HOW TO RUN:   bash ros2_ws/src/simulation/sim_plugins/test/run_tests.sh       (inside Ubuntu 22.04 / WSL)
#  Exit code: 0 = all passed, 1 = something failed (listed at the end).
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG="$(cd "$HERE/.." && pwd)"
SIM_SRC="$(cd "$PKG/.." && pwd)"
REPO="$(cd "$PKG/../../../.." && pwd)"
FAILED=()
step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
check() { local name="$1"; shift; if "$@"; then echo " OK  $name"; else echo " XX  $name"; FAILED+=("$name"); fi; }

# ROS 2 + the workspace (for the launch-file tests and the dry runs); sim_plugins from THIS source tree.
# shellcheck disable=SC1091
source "$REPO/simulation/setup/sim_env.sh"
export PYTHONPATH="$PKG:$SIM_SRC/sim_bringup${PYTHONPATH:+:$PYTHONPATH}"
export ASTRA_PROFILES_DIR="$REPO/simulation/profiles"

step "1. pytest (sim_plugins/test and sim_plugin_examples)"
check "pytest sim_plugins" bash -c "cd /tmp && python3 -m pytest -q -p no:cacheprovider '$HERE'"
check "pytest sim_plugin_examples" bash -c "cd /tmp && python3 -m pytest -q -p no:cacheprovider '$SIM_SRC/sim_plugin_examples'"

step "2. every profile validates"
check "profile_tool validate" python3 -m sim_plugins.profile_tool validate

step "3. shell scripts"
SCRIPTS=("$REPO/simulation/scripts/demo.sh" "$REPO/simulation/scripts/start_sim.sh"
         "$REPO/simulation/setup/fetch_sitl.sh" "$HERE/run_tests.sh")
for f in "${SCRIPTS[@]}"; do check "bash -n $(basename "$f")" bash -n "$f"; done
if command -v shellcheck > /dev/null; then
  check "shellcheck (warnings)" shellcheck -x -S warning "${SCRIPTS[@]}"
else
  echo " --  shellcheck not installed (sudo apt install shellcheck)"
fi

step "4. fetch_sitl.sh --dry-run (no internet)"
for v in 4.6.3 4.7.1 stable beta latest Copter-4.6.3; do
  check "fetch_sitl.sh $v --dry-run" bash -c "bash '$REPO/simulation/setup/fetch_sitl.sh' $v --dry-run | grep -q 'dry run'"
done

step "5. the default plan equals today's commands (dry runs)"
# Only the numbered command lines are compared; the run folder's time stamp is blanked out.
commands() { grep -E '^    [0-9]\. ' | sed -E 's#/runs/[0-9-]+_#/runs/STAMP_#'; }
if [ "${ASTRA_WS_BUILT:-no}" = yes ] && ros2 pkg prefix sim_bringup > /dev/null 2>&1; then
  for world in runway suas_field; do
    a="$(bash "$REPO/simulation/scripts/demo.sh" "$world" --dry-run 2>/dev/null | commands)"
    b="$(bash "$REPO/simulation/scripts/demo.sh" "$world" --dry-run --profile default 2>/dev/null | commands)"
    check "demo.sh $world: --profile default == no profile" test -n "$a" -a "$a" = "$b"
    [ "$a" = "$b" ] || diff <(echo "$a") <(echo "$b") | sed 's/^/      /'
  done
  a="$(bash "$REPO/simulation/scripts/demo.sh" runway --detector truth --headless --dry-run 2>/dev/null | commands)"
  b="$(bash "$REPO/simulation/scripts/demo.sh" --profile truth_detector --headless --dry-run 2>/dev/null | commands)"
  check "demo.sh --detector truth == --profile truth_detector" test -n "$a" -a "$a" = "$b"
  a="$(bash "$REPO/simulation/scripts/start_sim.sh" runway --dry-run 2>/dev/null | commands)"
  b="$(bash "$REPO/simulation/scripts/start_sim.sh" --profile default --dry-run 2>/dev/null | commands)"
  check "start_sim.sh: --profile default == no profile" test -n "$a" -a "$a" = "$b"
else
  echo " --  skipped: the workspace is not built (bash simulation/setup/03_build.sh)"
fi

echo
if [ ${#FAILED[@]} -eq 0 ]; then echo "ALL CHECKS PASSED"; exit 0; fi
echo "FAILED: ${FAILED[*]}"
exit 1
