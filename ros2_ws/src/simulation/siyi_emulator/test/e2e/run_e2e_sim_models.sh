#!/usr/bin/env bash
# =============================================================================
#  run_e2e_sim_models.sh  --  the emulator against the REAL simulated drone
#
#  PURPOSE: generates the drone (sim_models iris_a8mini, xacro, camera_model
#  zr10, ArduPilot JSON port 9002+10*INSTANCE so it never collides with
#  another simulation; no SITL is started), spawns it HEADLESS in sim_models'
#  own test world (poles at known bearings), starts the emulator on a
#  development address (127.0.0.3), runs e2e_sim_models_check.py, and stops
#  only the processes it started. Skips (exit 0) when sim_models or the
#  ArduPilot Gazebo plugin is not available.
#
#  HOW TO RUN (after colcon build + source install/setup.bash):
#      bash run_e2e_sim_models.sh [output_folder]        (default /tmp/siyi_e2e_sim_models)
#  Environment: BIND_IP (127.0.0.3), INSTANCE (3), GZ_PARTITION (siyi_e2e_models),
#  SIM_MODELS_DIR (folder with models/ and test/worlds/; default: the installed
#  sim_models share folder), ARDUPILOT_GAZEBO (~/ardupilot_gazebo).
# =============================================================================
set -uo pipefail

OUT="${1:-/tmp/siyi_e2e_sim_models}"
BIND_IP="${BIND_IP:-127.0.0.3}"
INSTANCE="${INSTANCE:-3}"
export GZ_PARTITION="${GZ_PARTITION:-siyi_e2e_models}"
export GZ_IP="${GZ_IP:-127.0.0.1}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AG="${ARDUPILOT_GAZEBO:-${HOME}/ardupilot_gazebo}"
SM="${SIM_MODELS_DIR:-}"
if [ -z "${SM}" ]; then
  prefix="$(ros2 pkg prefix sim_models 2>/dev/null || true)"
  [ -n "${prefix}" ] && SM="${prefix}/share/sim_models"
fi
XACRO_FILE="${SM}/models/iris_a8mini/model.sdf.xacro"
WORLD_FILE="${SM}/test/worlds/sim_models_test.sdf"
if [ -z "${SM}" ] || [ ! -f "${XACRO_FILE}" ] || [ ! -f "${WORLD_FILE}" ]; then
  echo "SKIPPED: sim_models not found (build it, or set SIM_MODELS_DIR to a folder with models/ and test/worlds/)"
  exit 0
fi
if [ ! -d "${AG}/models" ] || [ ! -d "${AG}/build" ]; then
  echo "SKIPPED: ArduPilot Gazebo plugin not found in ${AG} (run simulation/setup/02_setup_ardupilot.sh)"
  exit 0
fi

mkdir -p "${OUT}"
rm -f "${OUT}"/*.jpg "${OUT}/report.json"
export GZ_SIM_RESOURCE_PATH="${SM}/models:${AG}/models:${AG}/worlds${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${AG}/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:${GZ_SIM_SYSTEM_PLUGIN_PATH}}"
xacro "${XACRO_FILE}" camera_model:=zr10 fdm_port_in:=$((9002 + 10 * INSTANCE)) -o "${OUT}/iris_a8mini.sdf" || exit 2
cp "${WORLD_FILE}" "${OUT}/world.sdf"

PGIDS=()
cleanup() {
  for pg in "${PGIDS[@]}"; do kill -INT -- "-${pg}" 2>/dev/null; done
  sleep 2
  for pg in "${PGIDS[@]}"; do kill -KILL -- "-${pg}" 2>/dev/null; done
}
trap cleanup EXIT

echo "== sim_models test world + iris_a8mini (headless, GZ_PARTITION=${GZ_PARTITION}) =="
setsid gz sim -s -r -v 2 "${OUT}/world.sdf" > "${OUT}/gz.log" 2>&1 &
PGIDS+=($!)
for _ in $(seq 1 90); do
  gz service -l 2>/dev/null | grep -q "^/world/sim_models_test/create$" && break
  sleep 1
done
timeout 60 ros2 run ros_gz_sim create -world sim_models_test -file "${OUT}/iris_a8mini.sdf" -name iris \
  -x 0 -y 0 -z 0.195 -Y 1.5708 > "${OUT}/spawn.log" 2>&1 || { echo "spawn failed (see ${OUT}/spawn.log)"; exit 3; }
for _ in $(seq 1 60); do
  gz topic -l 2>/dev/null | grep -q "^/sim/camera/photo/image$" && break
  sleep 1
done

echo "== SIYI emulator on ${BIND_IP} =="
setsid ros2 run siyi_emulator siyi_emulator --ros-args -p bind_ip:="${BIND_IP}" -p sd_card_dir:="${OUT}/sd_card" \
  -p initial_pitch_deg:=0.0 > "${OUT}/emulator.log" 2>&1 &
PGIDS+=($!)
sleep 4

python3 "${HERE}/e2e_sim_models_check.py" --ip "${BIND_IP}" --out "${OUT}"
RC=$?
if grep -q -E "WARN.*(odometry|joint)" "${OUT}/emulator.log"; then
  echo "emulator warnings about the drone's topics:"; grep -E "WARN" "${OUT}/emulator.log"
fi
echo "logs: ${OUT}/emulator.log ${OUT}/gz.log"
exit ${RC}
