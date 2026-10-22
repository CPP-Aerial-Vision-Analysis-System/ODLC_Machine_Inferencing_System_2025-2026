#!/usr/bin/env bash
# =============================================================================================
#  sim_models/test/acceptance/run_acceptance.sh -- full acceptance test of the simulated drone
#
#  PURPOSE: proves the iris_a8mini model works with the real autopilot software. It starts,
#  HEADLESS (no windows), everything the test needs, runs acceptance_driver.py, and stops what it
#  started (only its own process groups):
#    1. Gazebo server with test/worlds/sim_models_test.sdf (1 ms physics, coloured targets)
#    2. the drone, generated from model.sdf.xacro for this SITL instance, spawned as "iris"
#    3. the payload_release node
#    4. ArduCopter SITL (--model JSON -I<instance>) with copter.parm, gazebo-iris.parm and
#       sim_models/config/astra_sitl.parm
#  The driver then flies: arm (after the ~30 s settling window), take off to 15 m, hover 20 s,
#  30 m forward and back with the payloads on, gimbal steps + camera frames, one photo trigger,
#  camera load / real-time factor, payload drops on servos 11 and 13, land.
#  Results: <out>/results.json, <out>/summary.txt, images in <out>.
#
#  USAGE (source the workspace first, so `ros2 run sim_models ...` works):
#    bash run_acceptance.sh [--instance 1] [--camera-model zr10|a8mini] [--out DIR]
#                           [--tests flight,cruise,gimbal,photo,rtf,payload] [--keep]
#  ENVIRONMENT (defaults in brackets): GZ_PARTITION [sim_models_test], GZ_IP [127.0.0.1],
#    ROS_DOMAIN_ID [41], ARDUPILOT_SITL_BIN, ARDUPILOT_GAZEBO_DIR [~/ardupilot_gazebo],
#    ARDUPILOT_PARAMS_DIR (folder with copter.parm + gazebo-iris.parm) [~/sitl].
#  Exit code: 0 = every test passed, 1 = a test failed, 2 = setup problem.
# =============================================================================================
set -u

INSTANCE=1
CAMERA_MODEL=zr10
OUT=""
TESTS="flight,cruise,gimbal,photo,rtf,payload"
KEEP=0
while [ $# -gt 0 ]; do
  case "$1" in
    --instance) INSTANCE="$2"; shift 2 ;;
    --camera-model) CAMERA_MODEL="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --tests) TESTS="$2"; shift 2 ;;
    --keep) KEEP=1; shift ;;
    -h|--help) sed -n '2,30p' "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done

say() { printf '\033[1;36m[acceptance] %s\033[0m\n' "$*"; }
die() { printf '\033[1;31m[acceptance] %s\033[0m\n' "$*"; exit 2; }

# ---- environment -----------------------------------------------------------------------------
export GZ_PARTITION="${GZ_PARTITION:-sim_models_test}"   # keeps this test away from other sims
export GZ_IP="${GZ_IP:-127.0.0.1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-41}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-d3d12}"          # WSL2: render on the GPU
export MESA_D3D12_DEFAULT_ADAPTER_NAME="${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}"

command -v ros2 >/dev/null || die "ROS 2 is not sourced (source /opt/ros/humble/setup.bash and the workspace)"
SHARE="$(ros2 pkg prefix sim_models 2>/dev/null)/share/sim_models"
[ -f "$SHARE/models/iris_a8mini/model.sdf.xacro" ] || die "sim_models is not built/sourced"
APG="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
[ -f "$APG/build/libArduPilotPlugin.so" ] || die "ArduPilotPlugin not found in $APG/build (run 02_setup_ardupilot.sh)"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$APG/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$SHARE/models:$APG/models:$APG/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

SITL=""
for c in "${ARDUPILOT_SITL_BIN:-}" "$HOME/sitl/arducopter" /opt/ardupilot/build/sitl/bin/arducopter; do
  if [ -n "$c" ] && [ -x "$c" ]; then SITL="$c"; break; fi
done
[ -n "$SITL" ] || die "ArduCopter SITL binary not found (set ARDUPILOT_SITL_BIN)"
PDIR=""
for d in "${ARDUPILOT_PARAMS_DIR:-}" "$HOME/sitl" /opt/ardupilot/Tools/autotest/default_params; do
  if [ -n "$d" ] && [ -f "$d/copter.parm" ] && [ -f "$d/gazebo-iris.parm" ]; then PDIR="$d"; break; fi
done
[ -n "$PDIR" ] || die "copter.parm / gazebo-iris.parm not found (set ARDUPILOT_PARAMS_DIR)"
DEFAULTS="$PDIR/copter.parm,$PDIR/gazebo-iris.parm,$SHARE/config/astra_sitl.parm"

# Gazebo + SITL + the cameras need about 3 GB: wait (up to 10 min) instead of failing at once,
# because other simulators on the same machine may be finishing.
for i in $(seq 1 60); do
  AVAIL=$(free -m | awk '/^Mem:/{print $7}')
  [ "${AVAIL:-0}" -ge 3000 ] && break
  [ "$i" = 1 ] && say "only ${AVAIL} MB of memory available, waiting for 3000 MB ..."
  sleep 10
done
[ "${AVAIL:-0}" -ge 3000 ] || die "only ${AVAIL} MB of memory available (need 3000): close something and retry"

OUT="${OUT:-/tmp/sim_models_acceptance_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT/sitl"
WORLD="$SHARE/test/worlds/sim_models_test.sdf"
MAVPORT=$((5760 + 10 * INSTANCE))
FDMPORT=$((9002 + 10 * INSTANCE))
HOME_LOC="-35.363262,149.165237,584,0"   # = the test world's <spherical_coordinates>
say "out=$OUT instance=$INSTANCE (MAVLink tcp:$MAVPORT, JSON udp:$FDMPORT) camera=$CAMERA_MODEL"
say "GZ_PARTITION=$GZ_PARTITION ROS_DOMAIN_ID=$ROS_DOMAIN_ID SITL=$SITL"

# refuse to run if someone else uses our ports
if ss -ltn 2>/dev/null | grep -q ":$MAVPORT "; then die "TCP $MAVPORT is busy (another SITL -I$INSTANCE?)"; fi
if ss -lun 2>/dev/null | grep -q ":$FDMPORT "; then die "UDP $FDMPORT is busy (another Gazebo drone on this instance?)"; fi

# ---- process bookkeeping ---------------------------------------------------------------------------
# Every child is started with setsid, so it and everything it starts share one NEW session whose id
# is the child's pid (even if a program moves its helpers to another process group, as `gz sim`
# does). Cleanup signals exactly those sessions (pkill -s <id>), never anything else.
SIDS=()
start_bg() {  # start_bg <logfile> <command...>
  local log="$1"; shift
  setsid "$@" > "$log" 2>&1 < /dev/null &
  SIDS+=($!)
}
signal_sessions() {  # signal_sessions <SIGNAL>
  for s in "${SIDS[@]}"; do pkill "-$1" -s "$s" 2>/dev/null; done
}
cleanup() {
  [ "$KEEP" = 1 ] && { say "--keep: leaving processes running (sessions: ${SIDS[*]})"; return; }
  [ ${#SIDS[@]} -eq 0 ] && return
  say "stopping test processes (sessions ${SIDS[*]})"
  signal_sessions INT; sleep 3
  signal_sessions TERM; sleep 2
  signal_sessions KILL
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# ---- 1. model -----------------------------------------------------------------------------------
say "generating the model (camera $CAMERA_MODEL, ArduPilotPlugin port $FDMPORT)"
ros2 run sim_models generate_model_sdf --camera-model "$CAMERA_MODEL" --instance "$INSTANCE" \
    -o "$OUT/iris_a8mini.sdf" || die "model generation failed"

# ---- 2. Gazebo server (headless) -------------------------------------------------------------------
say "starting Gazebo (headless) with $WORLD"
start_bg "$OUT/gz.log" timeout 3600 gz sim -s -r -v 3 "$WORLD"
for i in $(seq 1 180); do
  gz service -l 2>/dev/null | grep -q "/world/sim_models_test/create" && break
  sleep 1
done
gz service -l 2>/dev/null | grep -q "/world/sim_models_test/create" || die "Gazebo did not start (see $OUT/gz.log)"

say "spawning the drone as 'iris' at the origin, facing north"
timeout 180 ros2 run ros_gz_sim create -world sim_models_test -file "$OUT/iris_a8mini.sdf" -name iris \
    -x 0 -y 0 -z 0.195 -Y 1.5708 > "$OUT/spawn.log" 2>&1 || die "spawn failed (see $OUT/spawn.log)"

# ---- 3. payload release node ------------------------------------------------------------------------
say "starting payload_release"
start_bg "$OUT/payload_release.log" timeout 3600 ros2 run sim_models payload_release --ros-args \
    -p world_origin_lat:=-35.363262 -p world_origin_lon:=149.165237 -p world_origin_alt:=584.0

# ---- 4. ArduCopter SITL -------------------------------------------------------------------------------
if [[ ",$TESTS," == *",flight,"* ]] || [[ ",$TESTS," == *",payload,"* ]] || [[ ",$TESTS," == *",rtf,"* ]] \
   || [[ ",$TESTS," == *",gimbal,"* ]] || [[ ",$TESTS," == *",photo,"* ]]; then
  say "starting ArduCopter SITL -I$INSTANCE"
  # SITL writes eeprom.bin and logs/ into the current folder: use a local temp folder (fast disk)
  SITL_DIR=$(mktemp -d /tmp/sim_models_sitl_XXXX)
  echo "$SITL_DIR" > "$OUT/sitl/workdir.txt"
  cd "$SITL_DIR"
  start_bg "$OUT/sitl.log" timeout 3600 "$SITL" --model JSON -I"$INSTANCE" \
      --speedup 1 --home "$HOME_LOC" --defaults "$DEFAULTS" -w
  cd - > /dev/null
fi

# ---- 5. the test driver ------------------------------------------------------------------------------
say "running acceptance_driver.py ($TESTS)"
timeout 3000 python3 "$SHARE/test/acceptance/acceptance_driver.py" \
    --mav "tcp:127.0.0.1:$MAVPORT" --world sim_models_test --model-sdf "$OUT/iris_a8mini.sdf" \
    --camera-model "$CAMERA_MODEL" --parm "$SHARE/config/astra_sitl.parm" --tests "$TESTS" \
    --payload-log "$OUT/payload_release.log" --out "$OUT"
RC=$?
say "driver exit code $RC; results in $OUT"
[ -f "$OUT/summary.txt" ] && cat "$OUT/summary.txt"
exit $RC
