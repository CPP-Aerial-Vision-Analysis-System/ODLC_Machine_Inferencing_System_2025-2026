# shellcheck shell=bash
# =============================================================================
#  _common.sh  --  helpers shared by the simulation scripts (sourced by them; never run it by itself)
#
#  PURPOSE: the things every script in simulation/scripts needs, written once:
#    * friendly coloured messages (say / info / ok / warn / die),
#    * loading the simulator settings (setup/sim_env.sh) and checking the workspace is built,
#    * the memory check (the simulator needs about 3 GB free),
#    * the camera address 192.168.144.25 on this PC (needed by the camera emulator), added with admin
#      rights when it is missing (sudo without password, or WSL's root door `wsl.exe -u root`),
#    * "sessions": every process a script starts inherits ASTRA_SIM_SESSION=<id>, so stop_sim.sh can
#      find and stop exactly those processes and nothing else (never other people's simulations),
#    * small waiting helpers.
# =============================================================================

ASTRA_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASTRA_SETUP_DIR="$(cd "$ASTRA_SCRIPTS_DIR/../setup" && pwd)"
ASTRA_CAMERA_IP="${ASTRA_CAMERA_IP:-192.168.144.25}"
# Live code map (opt-in, simulation/codemap): only with ASTRA_TRACE=1, report which functions run (UDP 8091).
if [ "${ASTRA_TRACE:-0}" = 1 ] && [ -f "$ASTRA_SCRIPTS_DIR/../codemap/trace/astra_trace.sh" ]; then
  source "$ASTRA_SCRIPTS_DIR/../codemap/trace/astra_trace.sh" || true; fi

# Print a script's built-in help = its header comment (line 2 up to the first line that is not a comment).
astra_usage() { awk 'NR == 1 { next } /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$1"; }

# Every warning and error names the program it comes from, e.g. " XX [demo] ERROR: ...", so a teammate who finds it
# in a long log knows which command it was and can look it up in docs/simulation/ERRORS.md. ASTRA_PROG = the script's
# file name without .sh (a script may set it before sourcing this file). A message that already starts with "[" keeps
# its own tag. Shape of a good message: WHAT failed. Likely cause: WHY. Fix: the exact command.
ASTRA_PROG="${ASTRA_PROG:-$(basename "${0%.sh}")}"
_astra_tagged() { case "$2" in "["*) printf '%s' "$2" ;; *) printf '[%s] %s: %s' "$ASTRA_PROG" "$1" "$2" ;; esac; }

say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
info() { printf '    %s\n' "$*"; }
ok()   { printf '\033[1;32m OK %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m !! %s\033[0m\n' "$(_astra_tagged WARNING "$*")" >&2; }
die()  { printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "$*")" >&2; exit 1; }
# die_usage MESSAGE : a mistake in the command line (unknown option, missing value). Exit code 2 (1 = something
# failed, 2 = the command was typed wrong); the fix is always the script's own --help.
die_usage() {
  printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "$*. Fix: see the options with  bash \"$ASTRA_SCRIPTS_DIR/$ASTRA_PROG.sh\" --help")" >&2
  exit 2
}
# warn_more TEXT : one more indented line that belongs to the warning just printed (e.g. the exact fix command).
warn_more() { printf '\033[1;33m        %s\033[0m\n' "$*" >&2; }

# Load setup/sim_env.sh (ROS 2 Humble + workspace + Gazebo/ArduPilot paths + GPU settings).
astra_load_env() {
  # shellcheck disable=SC1091
  source "$ASTRA_SETUP_DIR/sim_env.sh"
  ASTRA_STATE_DIR="$ASTRA_SIM_HOME/state"
  mkdir -p "$ASTRA_STATE_DIR" "$ASTRA_SIM_HOME/logs"
}

# astra_state_value KEY DEFAULT : a value the running simulator wrote into state/current_launch.yaml
# (e.g. gcs_port, sitl_instance, camera_ip, mission), or DEFAULT when there is none.
astra_state_value() {
  local v
  v=$(awk -v k="$1:" '$1 == k {print $2}' "${ASTRA_SIM_HOME:-$HOME/.astra_sim}/state/current_launch.yaml" 2>/dev/null | tr -d "'\"")
  printf '%s\n' "${v:-$2}"
}

# Stop with a clear message when the workspace (or sim_bringup in it) is not built yet.
astra_require_built() {
  if [ "${ASTRA_WS_BUILT:-no}" != "yes" ] || ! ros2 pkg prefix sim_bringup >/dev/null 2>&1; then
    die "The simulator is not built yet (no $ASTRA_REPO/ros2_ws/install with sim_bringup).
    Likely cause: setup/03_build.sh has not run on this PC yet, or its build failed.
    Fix: bash \"$ASTRA_SETUP_DIR/03_build.sh\"   (then run this command again)"
  fi
}

# Warn when less than about 3 GB of memory is free (the simulator gets slow or is killed otherwise).
astra_mem_check() {
  local avail
  avail=$(free -m 2>/dev/null | awk '/^Mem:/{print $7}')
  if [ -n "$avail" ] && [ "$avail" -lt 3000 ]; then
    warn "Only ${avail} MB of memory is free; the simulator wants about 3000 MB. Consequence: Gazebo gets slow or" \
         "Linux stops a program. Fix: close other programs or another simulator (bash \"$ASTRA_SCRIPTS_DIR/stop_sim.sh\" --all)," \
         "then check with  free -m"
  fi
}

# True when the camera address exists on this PC.
astra_camera_ip_ok() {
  ip -4 addr show dev lo 2>/dev/null | grep -q "inet ${1:-$ASTRA_CAMERA_IP}/"
}

# Make sure 192.168.144.25 (the SIYI camera's address) exists on this PC; returns 1 if it could not be added.
astra_ensure_camera_ip() {
  local addr="${1:-$ASTRA_CAMERA_IP}"
  astra_camera_ip_ok "$addr" && return 0
  say "The camera address $addr is not on this PC yet: adding it (this needs admin rights)"
  if sudo -n true 2>/dev/null && sudo -n ip addr add "$addr/32" dev lo 2>/dev/null; then
    astra_camera_ip_ok "$addr" && { ok "added with sudo"; return 0; }
  fi
  if [ -n "${WSL_DISTRO_NAME:-}" ] && command -v wsl.exe >/dev/null 2>&1; then
    info "using WSL's own admin door (wsl.exe -u root), no password needed"
    timeout 60 wsl.exe -d "$WSL_DISTRO_NAME" -u root -- ip addr add "$addr/32" dev lo >/dev/null 2>&1
    astra_camera_ip_ok "$addr" && { ok "added"; return 0; }
  fi
  if [ -t 0 ] && sudo ip addr add "$addr/32" dev lo; then
    astra_camera_ip_ok "$addr" && { ok "added with sudo"; return 0; }
  fi
  warn "Could not add the camera address $addr to this PC. Likely cause: no admin rights (sudo wants a password and" \
       "WSL's root door did not work). Consequence: the team's video_cam cannot reach the camera emulator (no photos)." \
       "Fix, once (it then also comes back after every restart):"
  warn_more "sudo bash \"$ASTRA_SETUP_DIR/camera_ip_alias.sh\""
  warn_more "or from Windows PowerShell:  wsl -d ${WSL_DISTRO_NAME:-Ubuntu-22.04} -u root ip addr add $addr/32 dev lo"
  return 1
}

# Start a new session id (all child processes inherit it) and remember it for stop_sim.sh / status.sh.
astra_new_session() {
  local kind="${1:-sim}"
  ASTRA_SIM_SESSION="${kind}-$(date +%Y%m%d-%H%M%S)-$$"
  export ASTRA_SIM_SESSION
  printf '%s\n' "$ASTRA_SIM_SESSION" > "$ASTRA_STATE_DIR/session"
}

# The pid of this script and all its parents (never stop ourselves).
astra_ancestors() {
  local p=$$
  while [ -n "$p" ] && [ "$p" != "0" ] && [ "$p" != "1" ]; do
    echo "$p"
    p=$(awk '/^PPid:/{print $2}' "/proc/$p/status" 2>/dev/null)
  done
}

# List the pids of this user's processes that carry ASTRA_SIM_SESSION=<id> (any session when id is "*").
# The ros2 command-line daemon is skipped: it is shared by every `ros2` command and harmless.
astra_session_pids() {
  local id="$1" p env cmd skip
  skip=" $(astra_ancestors | tr '\n' ' ') "
  for p in $(pgrep -u "$(id -u)" . 2>/dev/null); do
    case "$skip" in *" $p "*) continue ;; esac
    # (the braces send the "Permission denied" / "No such file" of short-lived processes to /dev/null too)
    env=$( { tr '\0' '\n' < "/proc/$p/environ"; } 2>/dev/null ) || continue
    if [ "$id" = "*" ]; then
      printf '%s\n' "$env" | grep -q '^ASTRA_SIM_SESSION=' || continue
    else
      printf '%s\n' "$env" | grep -qx "ASTRA_SIM_SESSION=$id" || continue
    fi
    cmd=$( { tr '\0' ' ' < "/proc/$p/cmdline"; } 2>/dev/null ) || continue
    [ -n "$cmd" ] || continue                       # the process has just ended
    case "$cmd" in *ros2cli.daemon*) continue ;; esac
    echo "$p"
  done
}

# After a mission upload from our ground-station port: make a running MAVROS (the team's ./launcher.sh sim)
# read the mission again. MAVROS re-reads by itself only when it sees a ground station DOWNLOAD the mission,
# never after an upload, so the team's waypoint_manager would keep the old (or empty) mission: buffer_wp stays
# -1/-2, the photo loop never stops and main_controller never holds at the last waypoint. Harmless when MAVROS
# is not running (it reads the mission when it starts).
astra_mavros_repull() {
  local out
  # Call the service directly: `ros2 service call` waits until it discovers the service itself (a
  # `ros2 service list` check first can come back empty while the ros2 daemon is still starting).
  out=$(timeout 15 ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull "{}" 2>&1)
  if printf '%s\n' "$out" | grep -q 'success=True'; then
    ok "MAVROS re-read the mission (the team's waypoint_manager now has the new indices)"
  elif ! printf '%s\n' "$out" | grep -q 'response:'; then
    info "MAVROS is not running (no /mavros/mission/pull): it reads the mission when ./launcher.sh sim starts"
  else
    warn "MAVROS did not re-read the mission (its answer was not success). Likely cause: MAVROS is still starting or" \
         "busy talking to the autopilot. Consequence: the team's waypoint_manager keeps the OLD mission (photos never" \
         "stop, no hold, no drops). Fix: wait 10 s and run:"
    warn_more "ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull {}     (or restart ./launcher.sh sim)"
    return 1
  fi
  return 0
}

# astra_require_sim_running : stop with a clear message when no simulator runs on this PC: no process of the recorded
# session, and no SITL / camera emulator / Gazebo server at all (any user: a Docker container's processes count too).
astra_require_sim_running() {
  local session
  session="$(cat "${ASTRA_STATE_DIR:-$HOME/.astra_sim/state}/session" 2>/dev/null)"
  if [ -n "$session" ] && [ -n "$(astra_session_pids "$session")" ]; then return 0; fi
  # Exact process names (SITL, the camera emulator, the Gazebo server): a command line that only MENTIONS one of
  # these words (an editor, a grep, a log viewer) does not count as a running simulator.
  { pgrep -x arducopter || pgrep -x siyi_emulator || pgrep -f '^gz sim( |$)'; } >/dev/null 2>&1 && return 0
  die "No simulator is running, so there is no autopilot or camera to talk to.
    Likely cause: the simulator was not started yet, or it has stopped (read the end of the last run's sim.log).
    Fix: start it first, e.g.  bash \"$ASTRA_SCRIPTS_DIR/astra.sh\" manual   (= demo.sh runway --manual), then run this again"
}

# astra_require_sim_link URL [typed] : before a script talks MAVLink on URL (udpin:HOST:PORT, what upload_mission.sh /
# fly_mission.sh use), stop with a clear message when no simulator runs, or when another program already listens on
# that port (then the tool would only fail with "Address already in use"). Other URLs (tcp:, udpout:): no check.
# "typed" = the person gave this URL with --url: then only the port check runs, because the MAVLink source may be a
# simulator in Docker or on another PC, or a real autopilot, which this PC cannot see as a process.
astra_require_sim_link() {
  local url="$1" port
  case "$url" in udpin:*:*) port="${url##*:}" ;; *) return 0 ;; esac
  [ "${2:-}" = typed ] || astra_require_sim_running
  if ss -lunH 2>/dev/null | grep -qE "[:.]$port\b"; then
    die "UDP port $port (where this command listens for the autopilot's MAVLink) is already used by another program.
    Likely cause: QGroundControl or another ground station inside Ubuntu, or another mission/status command still running.
    Fix: close that program (find it with:  ss -lunp | grep $port), or give this command another link with --url"
  fi
}

# astra_archive_photos DIR [STAMP] : move the team's photos (and new_od's result files) of an earlier run to
# $ASTRA_SIM_HOME/photo_archive/<STAMP>/. Old photos would be processed again by new_od and could send the drone
# to an OLD target. The FILES are moved, never the folder itself, so a new_od that already watches it keeps working.
astra_archive_photos() {
  local dir="$1" stamp="${2:-$(date +%Y%m%d-%H%M%S)}" archive det
  [ -n "$dir" ] || return 0
  mkdir -p "$dir"
  det="$(dirname "$dir")/detection_results_sahi"
  if [ -z "$(ls -A "$dir" 2>/dev/null)" ] && [ -z "$(ls -A "$det" 2>/dev/null)" ]; then return 0; fi
  archive="$ASTRA_SIM_HOME/photo_archive/$stamp"
  mkdir -p "$archive/mapping_photos" "$archive/detection_results_sahi"
  find "$dir" -mindepth 1 -maxdepth 1 -exec mv -t "$archive/mapping_photos/" {} + 2>/dev/null
  [ -d "$det" ] && find "$det" -mindepth 1 -maxdepth 1 -exec mv -t "$archive/detection_results_sahi/" {} + 2>/dev/null
  info "photos from an earlier run were moved to $archive"
}

# Short command line of a pid (for messages).
astra_cmd_of() { { tr '\0' ' ' < "/proc/$1/cmdline"; } 2>/dev/null | cut -c1-110; }

# wait_until SECONDS COMMAND...   : run COMMAND every second until it succeeds (0) or time runs out (1).
astra_wait_until() {
  local limit="$1"; shift
  local start=$SECONDS
  while [ $((SECONDS - start)) -lt "$limit" ]; do
    "$@" && return 0
    sleep 1
  done
  return 1
}
