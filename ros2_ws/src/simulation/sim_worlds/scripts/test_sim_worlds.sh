#!/usr/bin/env bash
# sim_worlds/scripts/test_sim_worlds.sh -- PURPOSE: run every check of the sim_worlds package in one go and print
# PASS/FAIL per step:
#   1. unit tests (pytest: geo maths, survey lanes, mission shape, configs, truth files, assets, generators),
#   2. mission files: read back and checked against the shape last year's code needs,
#   3. Fuel models in the local cache (warning only: worlds still load without them, objects are missing),
#   4. each world loads HEADLESS: no Gazebo errors, contract systems loaded, real-time factor, GPS probes
#      (skipped with --quick, or when no `gz` command is available).
# usage:  test_sim_worlds.sh [--quick]        (source your ROS 2 Humble workspace first)
set -u
QUICK=0; [ "${1:-}" = "--quick" ] && QUICK=1
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$HERE/../package.xml" ]; then PKG="$(cd "$HERE/.." && pwd)"           # source tree
else PKG="$(ros2 pkg prefix --share sim_worlds 2>/dev/null)"; fi               # installed
TOOLS="$HERE"
export PYTHONPATH="$PKG${PYTHONPATH:+:$PYTHONPATH}"
fail=0
step() { echo; echo "=== $1"; }
result() { if [ "$1" -eq 0 ]; then echo "--> PASS: $2"; else echo "--> FAIL: $2"; fail=$((fail + 1)); fi; }

step "1. unit tests"
if [ -d "$PKG/test" ]; then
  python3 -m pytest -q "$PKG/test"; result $? "unit tests"
else
  echo "(no test folder in $PKG: run from the source tree for the unit tests)"
fi

step "2. mission files"
for w in runway suas_field; do
  python3 "$TOOLS/make_mission.py" "$w" --check "$PKG/missions/${w}_survey.waypoints" > /tmp/sim_worlds_mission_$w.txt 2>&1
  rc=$?; tail -3 /tmp/sim_worlds_mission_$w.txt; result $rc "mission $w"
done

step "3. Fuel models in the cache"
bash "$TOOLS/fetch_models.sh" --check | tail -1 || echo "(warning only: run fetch_models.sh to download them)"

if [ "$QUICK" -eq 1 ] || ! command -v gz >/dev/null 2>&1; then
  echo; echo "(skipping the headless world loads: --quick or no gz command)"
else
  for w in runway suas_field; do
    step "4. headless load of $w"
    timeout 300 python3 "$TOOLS/check_world.py" "$w" --seconds 10; result $? "world $w"
  done
fi
echo
if [ "$fail" -eq 0 ]; then echo "ALL CHECKS PASSED"; else echo "$fail CHECK(S) FAILED"; fi
exit "$fail"
