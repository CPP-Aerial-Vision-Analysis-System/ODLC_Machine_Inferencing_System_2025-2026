#!/usr/bin/env bash
# =============================================================================
#  status.sh  --  what is the simulator doing right now?
#
#  PURPOSE: one screen that answers "is it running, and is every piece talking?":
#    1. the recorded session and the world/ports the running launch resolved (state/current_launch.yaml),
#    2. the simulator processes (Gazebo, SITL, MAVProxy, camera emulator, bridge, panel, team stack ...),
#    3. the network ports (SITL 5760/9002 + 10*I, MAVProxy 14550/14552/14553, camera 37260/82),
#    4. the camera address 192.168.144.25 and the camera emulator's media server answer,
#    5. the autopilot (one status line over MAVLink, if udp 14550 is free),
#    6. the ROS 2 nodes that are up.
#  It only looks; it never starts or stops anything.
#
#  HOW TO RUN:   bash status.sh            (add --no-mavlink to skip step 5)
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
MAVLINK=1
case "${1:-}" in -h|--help) astra_usage "$0"; exit 0 ;; --no-mavlink) MAVLINK=0 ;; esac
astra_load_env

say "1. Session"
SESSION=$(cat "$ASTRA_STATE_DIR/session" 2>/dev/null)
info "recorded session: ${SESSION:-none}"
if [ -f "$ASTRA_STATE_DIR/current_launch.yaml" ]; then
  grep -E '^(started|world|world_file|camera_model|detector|sitl_instance|gcs_port|mavros_port|panel_port|photo_dir|mission|truth):' \
    "$ASTRA_STATE_DIR/current_launch.yaml" | sed 's/^/    /'
fi
INSTANCE=$(astra_state_value sitl_instance 0)
GCS=$(astra_state_value gcs_port 14550)
MAVROS=$(astra_state_value mavros_port 14552)
PANEL=$(astra_state_value panel_port 14553)

say "2. Simulator processes (this user)"
if [ -n "$SESSION" ]; then
  mapfile -t PIDS < <(astra_session_pids "$SESSION")
  info "${#PIDS[@]} process(es) with session tag $SESSION"
  for p in "${PIDS[@]}"; do
    c=$(astra_cmd_of "$p")
    case "$c" in
      *gz\ sim*|*arducopter*|*mavproxy*|*siyi_emulator*|*payload_release*|*parameter_bridge*|*rviz2*|*demo_panel*|*demo_markers*|*truth_detector*|*mavros_node*|*new_od*|*siyi\ *|*main_controller*|*launcher.sh*|*phase_monitor*|*ros2\ launch*)
        info "  $p  $c" ;;
    esac
  done
else
  info "no session recorded"
fi
for name in 'gz sim' arducopter mavproxy.py siyi_emulator; do
  n=$(pgrep -u "$(id -u)" -fc "$name" 2>/dev/null)
  info "$(printf '%-14s' "$name") running: ${n:-0}"
done

say "3. Ports (SITL instance $INSTANCE)"
s0=$((5760 + 10 * INSTANCE)); fdm=$((9002 + 10 * INSTANCE))
UDP=$(ss -lunH 2>/dev/null); TCP=$(ss -ltnH 2>/dev/null)
port_state() { if printf '%s\n' "$2" | grep -qE "[:.]$1\b"; then echo "in use"; else echo "free"; fi; }
info "TCP $s0 (SITL MAVLink):         $(port_state "$s0" "$TCP")"
info "UDP $fdm (Gazebo<->SITL link):  $(port_state "$fdm" "$UDP")"
info "UDP $GCS (ground station):     $(port_state "$GCS" "$UDP")   (bound only while a ground station/script listens)"
info "UDP $MAVROS (MAVROS):             $(port_state "$MAVROS" "$UDP")   (bound while ./launcher.sh sim runs)"
info "UDP $PANEL (demo panel):         $(port_state "$PANEL" "$UDP")"
info "UDP 37260 (camera SDK):         $(port_state 37260 "$UDP")"
info "TCP 82 (camera media server):   $(port_state 82 "$TCP")"

say "4. Camera address and emulator"
if astra_camera_ip_ok; then ok "$ASTRA_CAMERA_IP exists on this PC"; else warn "$ASTRA_CAMERA_IP is missing (run setup/camera_ip_alias.sh)"; fi
if command -v curl >/dev/null 2>&1; then
  ans=$(curl -s -m 3 "http://$ASTRA_CAMERA_IP:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0" 2>/dev/null)
  if [ -n "$ans" ]; then ok "camera media server answers: $(printf '%s' "$ans" | cut -c1-100)"; else info "camera media server: no answer"; fi
fi

if [ "$MAVLINK" = 1 ] && [ "${ASTRA_WS_BUILT:-no}" = yes ]; then
  say "5. Autopilot (MAVLink on udp $GCS)"
  if [ "$(port_state "$GCS" "$UDP")" = "in use" ]; then
    info "udp $GCS is taken by a ground station; skipping"
  else
    timeout 15 ros2 run sim_bringup mission_tool --url "udpin:127.0.0.1:$GCS" --timeout 6 status 2>&1 | tail -1 | sed 's/^/    /'
  fi
fi

if [ "${ASTRA_WS_BUILT:-no}" = yes ]; then
  say "6. ROS 2 nodes (ROS_DOMAIN_ID ${ROS_DOMAIN_ID:-0})"
  timeout 15 ros2 node list 2>/dev/null | sed 's/^/    /' | head -40
fi
