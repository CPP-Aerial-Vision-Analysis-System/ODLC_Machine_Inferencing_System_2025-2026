#!/usr/bin/env bash
# =============================================================================
#  run_tests.sh  --  run the siyi_emulator unit tests
#
#  PURPOSE: runs every pytest test of the package (protocol, commands, zoom,
#  SD card, media server, gimbal maths, the UDP+HTTP photo flow with a fake
#  Gazebo, and, when the team's video_cam is built in the same workspace, the
#  compatibility test that drives video_cam's real CameraInterface against the
#  emulator). Needs no Gazebo and no running ROS nodes; uses free ports on
#  127.0.0.1 only, so it never disturbs a running simulation.
#
#  HOW TO RUN (after colcon build and "source install/setup.bash"):
#      bash $(ros2 pkg prefix siyi_emulator)/share/siyi_emulator/scripts/run_tests.sh
#      bash run_tests.sh -k media -v        (extra arguments go to pytest)
# =============================================================================
set -eo pipefail

# Find the package's test folder: next to this script (source tree, or the installed
# share/siyi_emulator, which also carries the tests), or the source folder behind a
# colcon --symlink-install link. The tests import the siyi_emulator package that is
# on PYTHONPATH (source install/setup.bash first).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR=""
for candidate in "${HERE}/.." "$(readlink -f "${HERE}/run_tests.sh" 2>/dev/null | xargs -r dirname)/.."; do
  if [ -f "${candidate}/test/test_sdk_protocol.py" ] && [ -f "${candidate}/test/conftest.py" ]; then
    PKG_DIR="$(cd "${candidate}" && pwd)"
    break
  fi
done
if [ -z "${PKG_DIR}" ]; then
  echo "Could not find the siyi_emulator source folder with its tests (looked next to ${HERE})."
  echo "Run this script from the source tree: ros2_ws/src/simulation/siyi_emulator/scripts/run_tests.sh"
  exit 1
fi

cd "${PKG_DIR}"
export PYTHONDONTWRITEBYTECODE=1
echo "Running siyi_emulator tests in ${PKG_DIR}"
exec python3 -m pytest -p no:cacheprovider -q test/ "$@"
