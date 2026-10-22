#!/usr/bin/env bash
# =============================================================================
#  run_e2e_gazebo.sh  --  end-to-end test of the SIYI emulator in Gazebo
#
#  PURPOSE: starts the small Gazebo test world (test/fixtures/gimbal_stand.sdf:
#  a floating "drone" with the contract's gimbal + cameras and coloured boxes)
#  HEADLESS, starts the emulator on a development address that never collides
#  with a real simulation (127.0.0.3 by default), runs e2e_gazebo_check.py
#  (yaw/pitch directions, zoom, lock/follow, stabilisation, photo delivery over
#  UDP + HTTP), then stops everything it started (its own process groups only).
#  Photos, a contact sheet, report.json and all logs end up in the output folder.
#
#  HOW TO RUN (after colcon build + source install/setup.bash):
#      bash run_e2e_gazebo.sh [output_folder]                 (default /tmp/siyi_e2e)
#      OPTICAL_ZOOM=1 bash run_e2e_gazebo.sh /tmp/siyi_e2e_optical
#  Environment: BIND_IP (127.0.0.3), GZ_PARTITION (siyi_e2e), CAMERA_MODEL (zr10),
#  ARDUPILOT_GAZEBO (~/ardupilot_gazebo, only for OPTICAL_ZOOM=1).
# =============================================================================
set -uo pipefail

OUT="${1:-/tmp/siyi_e2e}"
BIND_IP="${BIND_IP:-127.0.0.3}"
CAMERA_MODEL="${CAMERA_MODEL:-zr10}"
OPTICAL_ZOOM="${OPTICAL_ZOOM:-0}"
export GZ_PARTITION="${GZ_PARTITION:-siyi_e2e}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIXTURE="${HERE}/../fixtures/gimbal_stand.sdf"

if ! command -v gz >/dev/null 2>&1; then echo "gz (Gazebo) not found: source /opt/ros/humble/setup.bash"; exit 2; fi
if ! command -v ros2 >/dev/null 2>&1; then echo "ros2 not found: source your workspace"; exit 2; fi
if [ ! -f "${FIXTURE}" ]; then echo "fixture world missing: ${FIXTURE}"; exit 2; fi

mkdir -p "${OUT}/world"
rm -f "${OUT}"/*.jpg "${OUT}/report.json"
python3 "${HERE}/make_ground_texture.py" "${OUT}/world/ground.png" >/dev/null || exit 2
cp "${FIXTURE}" "${OUT}/world/gimbal_stand.sdf"
EMU_EXTRA=()
CHECK_EXTRA=()
if [ "${OPTICAL_ZOOM}" = "1" ]; then
  AG="${ARDUPILOT_GAZEBO:-${HOME}/ardupilot_gazebo}"
  export GZ_SIM_SYSTEM_PLUGIN_PATH="${AG}/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
  python3 - "${OUT}/world/gimbal_stand.sdf" <<'EOF'
import sys
path = sys.argv[1]
text = open(path, encoding="utf-8").read()
plugin = ('<plugin filename="CameraZoomPlugin" name="CameraZoomPlugin">'
          '<topic>/sim/camera/photo/zoom</topic><max_zoom>20</max_zoom></plugin>')
marker = text.index("<!-- OPTICAL_ZOOM_PLUGIN")
end = text.index("-->", marker) + 3
open(path, "w", encoding="utf-8").write(text[:marker] + plugin + text[end:])
EOF
  EMU_EXTRA=(-p optical_zoom_topics:=/sim/camera/photo/zoom)
  CHECK_EXTRA=(--optical-zoom)
fi

PGIDS=()
cleanup() {
  for pg in "${PGIDS[@]}"; do kill -INT -- "-${pg}" 2>/dev/null; done
  sleep 2
  for pg in "${PGIDS[@]}"; do kill -KILL -- "-${pg}" 2>/dev/null; done
}
trap cleanup EXIT

echo "== Gazebo test world (headless, GZ_PARTITION=${GZ_PARTITION}) =="
setsid gz sim -s -r -v 2 "${OUT}/world/gimbal_stand.sdf" > "${OUT}/gz.log" 2>&1 &
PGIDS+=($!)
for _ in $(seq 1 90); do
  gz topic -l 2>/dev/null | grep -q "^/sim/camera/photo/image$" && break
  sleep 1
done
if ! gz topic -l 2>/dev/null | grep -q "^/sim/camera/photo/image$"; then
  echo "Gazebo did not come up (see ${OUT}/gz.log)"; exit 3
fi

echo "== SIYI emulator on ${BIND_IP} (${CAMERA_MODEL}) =="
setsid ros2 run siyi_emulator siyi_emulator --ros-args -p bind_ip:="${BIND_IP}" -p camera_model:="${CAMERA_MODEL}" \
  -p sd_card_dir:="${OUT}/sd_card" -p initial_pitch_deg:=0.0 "${EMU_EXTRA[@]}" > "${OUT}/emulator.log" 2>&1 &
PGIDS+=($!)
sleep 3

echo "== checks =="
python3 "${HERE}/e2e_gazebo_check.py" --ip "${BIND_IP}" --out "${OUT}" --world gimbal_stand "${CHECK_EXTRA[@]}"
RC=$?
echo "emulator log: ${OUT}/emulator.log   gazebo log: ${OUT}/gz.log"
exit ${RC}
