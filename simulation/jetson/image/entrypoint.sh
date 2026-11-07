#!/bin/bash
# =============================================================================
#  /opt/astra/entrypoint.sh  --  how the Jetson-in-the-loop container starts (simulation/jetson/image/entrypoint.sh)
#
#  PURPOSE: start the team's flight software exactly as the drone does (the team's own ./launcher.sh, unchanged),
#  with only the two things that must differ in a simulator flight: where the autopilot is (FCU_URL) and how the
#  camera is reached. Modes (first argument; docs/jetson/HIL_DESIGN.md 8.2):
#    sim       (default) Jetson in the loop: the container has NO network of its own (--network none), so this adds
#              the camera's address 192.168.144.25 to the container's OWN loopback (nothing outside the container
#              changes), starts the relay (hil_relay.py container: MAVLink on 127.0.0.1:5763 and the camera on
#              192.168.144.25 -> the socket files the PC's SSH forwards end in), starts the read-only ROS tap, then
#              ./launcher.sh hw with FCU_URL=tcp://127.0.0.1:5763.
#    real      on the drone (no simulator): --network host; ./launcher.sh hw as today (MAVROS on /dev/ttyUSB0,
#              video_cam straight to the real camera on eth0). No relay, no tap, no extra address.
#    selftest  print and check what the image needs (GPU, TensorRT, packages, photo folder, model files).
#    warmup    build the TensorRT engine for this board (once; jetson.sh warmup).
#    shell     a bash shell as astra-dev with ROS and the workspace sourced.
#  Only the address step runs as root; everything that speaks ROS runs as ONE user, astra-dev, given the Jetson
#  user's ids (ASTRA_UID/ASTRA_GID), because Fast DDS between root and another user in one container delivers
#  nothing (HIL_DESIGN.md 8.6, VERIFIED). Errors say what failed, why and how to fix it; exit codes: 20 = the
#  safety guard refused (other network interfaces than lo), 21 = could not add the camera address, 22 = the socket
#  folder is missing, 23 = unknown mode.
# =============================================================================
set -o pipefail
MODE="${1:-${ASTRA_HIL_MODE:-sim}}"
[ $# -gt 0 ] && shift
CAMERA_IP="${CAMERA_IP:-192.168.144.25}"
RUN_ID="${ASTRA_RUN_ID:-manual-$(date +%Y%m%d-%H%M%S)}"
RUN_DIR="/data/runs/$RUN_ID"
SOCK_DIR="/run/astra_hil"
REPO=/home/astra-dev/astra
# shellcheck disable=SC1091
[ -f /opt/astra/image.env ] && . /opt/astra/image.env
FLAVOR="${ASTRA_FLAVOR:-${FLAVOR:-virtual}}"

say()  { printf '[entrypoint] %s\n' "$*"; }
fail() { local code="$1"; shift; printf '[entrypoint] ERROR: %s\n' "$*" >&2; exit "$code"; }

# give astra-dev the Jetson user's ids, so files in the mounted folders belong to that user
fix_ids() {
  local uid="${ASTRA_UID:-1000}" gid="${ASTRA_GID:-1000}"
  [ "$(id -u)" = 0 ] || return 0
  if [ "$(id -g astra-dev)" != "$gid" ]; then groupmod -o -g "$gid" astra-dev 2>/dev/null; fi
  if [ "$(id -u astra-dev)" != "$uid" ]; then usermod -o -u "$uid" astra-dev 2>/dev/null; fi
  chown astra-dev:astra-dev /home/astra-dev /run/astra_hil /data/runs 2>/dev/null
  chown -R astra-dev:astra-dev /home/astra-dev/.ros /home/astra-dev/.config 2>/dev/null
  mkdir -p "$RUN_DIR" && chown astra-dev:astra-dev "$RUN_DIR" 2>/dev/null
}

# run CMD as astra-dev (setpriv keeps the environment; HOME set explicitly)
as_stack() {
  if [ "$(id -u)" = 0 ]; then
    HOME=/home/astra-dev USER=astra-dev setpriv --reuid astra-dev --regid astra-dev --init-groups "$@"
  else
    "$@"
  fi
}

# the TRACKER_CMD per pipeline and flavor (HIL_DESIGN.md 8.2). launcher.sh itself stays unchanged.
pick_tracker() {
  local pipe="${ASTRA_PIPELINE:-auto}" prop
  prop=$(ls /opt/astra/extra/proposals/*.launch.py /opt/astra/extra/proposals/*.launch.xml 2>/dev/null | head -1)
  if [ "$pipe" = auto ]; then
    if [ -n "$prop" ] && [ -d "$REPO/ros2_ws/install/zr10_ros2" ]; then pipe=new; else pipe=legacy; fi
    [ "$pipe" = legacy ] && say "NOTE: the new video pipeline is not in this image yet (no proposed field launch file under simulation/video/proposals/ or no zr10_ros2 package when it was built), so the LEGACY (2025) photo pipeline runs. Rebuild the image once the video workflow has delivered them."
  fi
  if [ "$pipe" = new ]; then
    [ -n "$prop" ] || fail 24 "--pipeline new, but this image has no proposed field launch file (simulation/video/proposals/). Likely cause: the image was built before the video workflow delivered it. Fix: rebuild the image (jetson.sh image build), or start with --pipeline legacy"
    # the detector runs in its own container (jetson.sh start); the proposed file is asked to turn its part off
    export TRACKER_CMD="${TRACKER_CMD:-ros2 launch $prop detector:=external}"
  elif [ "$FLAVOR" = virtual ] && [ -z "${TRACKER_CMD:-}" ] && [ "${ASTRA_TENSORRT:-no}" != yes ]; then
    # the plain-ROS PC image has no TensorRT: the simulator's own override (the SAME tracker.launch.xml, the .pt on
    # PyTorch). The TensorRT image (default) runs the team's own launch file unchanged, like the board (below).
    export TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py detector:=real model_path:=/models/yolo26m.pt"
  fi
  # legacy + TensorRT image (the Jetson, and the PC's version-matched image): TRACKER_CMD stays UNSET = the team's own
  # "ros2 launch main tracker.launch.xml"; new_od finds ros2_ws/src/yolo26m.engine (a link into /models) by itself
  if [ "$pipe" = legacy ] && [ -z "${TRACKER_CMD:-}" ]; then
    if [ -s /models/yolo26m.engine ]; then
      say "detector: the team's new_od with the TensorRT engine /models/yolo26m.engine ($(grep -h '^GPU=\|^TRT=' /models/yolo26m.engine.meta 2>/dev/null | paste -sd' ' -))"
    else
      say "NOTE: no TensorRT engine in /models: the team's new_od builds it from yolo26m.pt at its first start (several minutes, as on a fresh board). Build it beforehand: jetson.sh warmup"
    fi
  fi
  ASTRA_PIPELINE_USED="$pipe"
  export ASTRA_PIPELINE_USED
}

start_stack() {   # runs launcher.sh in the foreground as astra-dev; passes SIGTERM on so it stops cleanly
  cd "$REPO" || fail 1 "the repo is missing in the image ($REPO). Likely cause: a broken build. Fix: rebuild the image"
  say "starting ./launcher.sh hw (FCU_URL=${FCU_URL:-launcher default}, TRACKER_CMD=${TRACKER_CMD:-team default: ros2 launch main tracker.launch.xml}, ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}, pipeline=$ASTRA_PIPELINE_USED)"
  # ASTRA_MODE is launcher.sh's own variable: never pass ours (HIL_DESIGN.md 5)
  as_stack env -u ASTRA_MODE bash -c 'set -o pipefail; ./launcher.sh hw 2>&1 | tee -a "$0"' "$RUN_DIR/team_stack.log" &
  STACK_PID=$!
}

stop_all() {
  say "stopping (SIGTERM): the team stack first, then the relay and the tap"
  if [ -n "${STACK_PID:-}" ]; then
    pkill -TERM -f "launcher.sh hw" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$STACK_PID" 2>/dev/null || break; sleep 1; done
    pkill -KILL -f "launcher.sh hw" 2>/dev/null
  fi
  [ -n "${TAP_PID:-}" ] && kill -TERM "$TAP_PID" 2>/dev/null
  [ -n "${RELAY_PID:-}" ] && kill -TERM "$RELAY_PID" 2>/dev/null
  wait 2>/dev/null
  say "stopped"
  exit 0
}

case "$MODE" in
  sim)
    # (1) safety guard: only lo may exist (else 192.168.144.25 could land on a real network and hide the real camera)
    others=""
    for nif in /sys/class/net/*; do
      nif="${nif##*/}"; [ "$nif" = lo ] || [ "$nif" = '*' ] || others="${others:+$others,}$nif"
    done
    [ -z "$others" ] || fail 20 "refusing to add $CAMERA_IP: this container has network interfaces besides lo ($others). Likely cause: it was started with --network host or on a Docker network. Adding the camera's address there could hide the real camera on the drone. Fix: start it with bash simulation/jetson/jetson.sh start --sim (it uses --network none)."
    # (2) the socket folder the PC's SSH forwards end in
    [ -d "$SOCK_DIR" ] || fail 22 "the socket folder $SOCK_DIR is not mounted. Likely cause: the container was started by hand. Fix: bash simulation/jetson/jetson.sh start --sim"
    # (3) the camera stand-in address on OUR loopback (needs NET_ADMIN inside this container only)
    if ! ip -4 addr show dev lo | grep -q "inet $CAMERA_IP/"; then
      ip addr add "$CAMERA_IP/32" dev lo 2>/dev/null \
        || fail 21 "could not add $CAMERA_IP to the container's own loopback. Likely cause: the container lacks --cap-add NET_ADMIN. Fix: start it with bash simulation/jetson/jetson.sh start --sim (it adds it)"
    fi
    fix_ids
    say "SIM mode, run $RUN_ID, flavor $FLAVOR, git ${GIT_SHA:-?}; camera stand-in $CAMERA_IP on lo; logs in $RUN_DIR"
    # (5) the relay as astra-dev (binding port 82 works: ip_unprivileged_port_start is 0 in a container)
    as_stack python3 /opt/astra/hil_relay.py container --sock-dir "$SOCK_DIR" --mav-listen 127.0.0.1:5763 \
        --cam-ip "$CAMERA_IP" --sdk-port 37260 --http-port 82 --rtsp-port 8554 \
        --idle-timeout 4 --redial 1 --gate 2.5 --http-idle 15 --gap-log 0.5 \
        --log "$RUN_DIR/relay.log" --status-file "$RUN_DIR/relay.status" &
    RELAY_PID=$!
    pick_tracker
    if [ "${ASTRA_TAP:-1}" = 1 ] && [ -f /opt/astra/ros_tap.py ]; then
      # the topic file's parts for the pipeline that runs ([common] + [new] or [legacy]), if the tap knows --sections
      sect=""
      if python3 /opt/astra/ros_tap.py send --help 2>/dev/null | grep -q -- '--parts'; then
        sect="--parts common,$([ "$ASTRA_PIPELINE_USED" = new ] && echo new || echo legacy)"
      fi
      # shellcheck disable=SC1091
      as_stack bash -c 'source /opt/ros/humble/setup.bash && source '"$REPO"'/ros2_ws/install/setup.bash && exec python3 /opt/astra/ros_tap.py send --listen unix:'"$SOCK_DIR"'/tap.sock --topics /opt/astra/tap_topics.txt '"$sect"' --nodes waypoint_manager,main_controller,siyi,mission_camera_trigger,new_od,gps_mavros_service_node,mavros,zr10_node,stream_node,detector_node --log '"$RUN_DIR"'/tap.log' &
      TAP_PID=$!
    elif [ "${ASTRA_TAP:-1}" = 1 ]; then
      say "NOTE: no ROS tap in this image (simulation/jetson/pc/ros_tap.py was not there when it was built): the PC sees the flight over MAVLink only."
    fi
    export FCU_URL="${FCU_URL:-tcp://127.0.0.1:5763}" CAMERA_IP
    trap stop_all TERM INT
    start_stack
    wait "$STACK_PID"; rc=$?
    say "the team stack ended (exit $rc)"
    stop_all
    ;;
  real)
    fix_ids
    say "REAL mode (on the drone): ./launcher.sh hw as today, run $RUN_ID"
    pick_tracker
    trap stop_all TERM INT
    start_stack
    wait "$STACK_PID"; stop_all
    ;;
  selftest)
    fix_ids
    as_stack bash -c 'source /opt/ros/humble/setup.bash && source '"$REPO"'/ros2_ws/install/setup.bash && exec python3 /opt/astra/selftest.py "$@"' selftest "$@"
    ;;
  warmup)
    fix_ids
    as_stack bash -c 'source /opt/ros/humble/setup.bash && source '"$REPO"'/ros2_ws/install/setup.bash && exec nice -n 19 ionice -c 3 python3 /opt/astra/warmup.py "$@"' warmup "$@"
    ;;
  shell|bash)
    fix_ids
    as_stack /opt/astra/in_stack.sh bash "$@"
    ;;
  *)
    fail 23 "unknown mode '$MODE'. Fix: use sim, real, selftest, warmup or shell"
    ;;
esac
