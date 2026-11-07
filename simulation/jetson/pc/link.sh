#!/usr/bin/env bash
# =============================================================================
#  simulation/jetson/pc/link.sh  --  the simulator's link to the Jetson (or to the virtual Jetson on this PC)
#
#  PURPOSE: give the team code on the Jetson its two "doors" to the simulator, exactly as on the drone it has a
#  serial cable to the Pixhawk and an Ethernet cable to the camera:
#    LINK A  the autopilot (SITL SERIAL2 = the Pixhawk's TELEM2 port, TCP 5763 + 10 x instance) and the read-only
#            ROS tap back to this PC (127.0.0.1:7461);
#    LINK B  the camera (the SIYI SDK on UDP 37260, the HTTP photo API on TCP 82, the RTSP video on TCP 8554 of the
#            camera emulator at 192.168.144.25).
#  Over the USB-C cable or any SSH address (--link usb | ssh), the doors are SSH "forwards" added to the connection
#  the PC opened (the owner's login window, or our own key connections). The PC opens every connection: WSL2 sits
#  behind NAT and cannot be dialled from outside. For the virtual Jetson (--link virtual) a local stand-in
#  (hil_relay.py pc-local) makes the same socket files, with no SSH at all.
#  After a drop (a pulled cable, Wi-Fi gone), link.sh notices, waits for the login (login mode: the owner presses
#  Enter in the login window and types the password; key mode: it reconnects by itself every 2 s), kills the old
#  SSH session on the Jetson (it would never notice the drop by itself), removes the stale socket files and adds the
#  forwards again (docs/jetson/HIL_DESIGN.md 9.4 and the reconnect steps of 15.3 g). It never closes the owner's
#  login window.
#
#  HOW TO RUN (from the repo folder, inside Ubuntu/WSL):
#    bash simulation/jetson/pc/link.sh start  [TARGET] [--run-dir DIR] [--instance I] [--camera zr10|a8mini]
#                                             [--photo-pace auto|off|MBIT] [--no-watch]
#    bash simulation/jetson/pc/link.sh status [TARGET] [--json]
#    bash simulation/jetson/pc/link.sh stop   [TARGET]
#    bash simulation/jetson/pc/link.sh drop SECONDS [TARGET] [--mode blackhole|close|glitch|cut]
#  TARGET = --link usb|ssh|virtual --host HOST --user USER --port N --auth login|key (or JETSON_LINK, JETSON_HOST ...;
#  default: the saved target, else the USB cable 192.168.55.1). demo.sh --jetson calls start and stop itself.
#  Drop modes (rehearse a lost link): virtual: blackhole (silent: bytes stop, sockets stay open) or close (every
#  connection closed); usb/ssh: glitch (freeze the PC's SSH for SECONDS, then go on: the Jetson sees silence, as
#  with a pulled cable) or cut (key mode only: freeze, then kill it: the full reconnect).
#  Files: $ASTRA_SIM_HOME/state/jetson_link.env (LINK HOST AUTH A B DROPS DOWN_S SINCE ...), RUN_DIR/jetson/link.log.
#  Exit codes: 0 OK, 1 a step failed, 2 typed wrong, 14 the link could not be opened (or is down: status),
#  17 not logged in (login mode: run jetson.sh login).
# =============================================================================
set -o pipefail
# shellcheck disable=SC2034  # (read by _common.sh: messages say "[link] ...")
ASTRA_PROG="link"
PC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
J_DIR="$(cd "$PC_DIR/.." && pwd)"
# shellcheck source=../lib/target.sh
source "${JETSON_TARGET_SH:-$J_DIR/lib/target.sh}"

RELAY="${JETSON_RELAY_PY:-$J_DIR/lib/hil_relay.py}"
PY="${JETSON_PY:-python3}"
SDK_PORT=37261; HTTP_PORT=37282; TAP_PORT=7461
CAMERA_IP="${ASTRA_CAMERA_IP:-192.168.144.25}"

# ---- options -------------------------------------------------------------------------------------------------------
CMD="${1:-}"; [ $# -gt 0 ] && shift
case "$CMD" in -h|--help|"") astra_usage "$0"; exit 0 ;; esac
DROP_S=""
if [ "$CMD" = drop ]; then DROP_S="${1:-}"; [ $# -gt 0 ] && shift; fi
jetson_target_load "$@"
RUN_DIR=""; INSTANCE=""; CAMERA="zr10"; PACE="auto"; WATCH=1; JSON=0; MODE=""
set -- "${JETSON_ARGS[@]}"
while [ $# -gt 0 ]; do
  case "$1" in
    --run-dir) RUN_DIR="${2:-}"; shift ;;
    --instance) INSTANCE="${2:-}"; shift ;;
    --camera) CAMERA="${2:-}"; shift ;;
    --photo-pace) PACE="${2:-}"; shift ;;
    --no-watch) WATCH=0 ;;
    --json) JSON=1 ;;
    --mode) MODE="${2:-}"; shift ;;
    -h|--help) astra_usage "$0"; exit 0 ;;
    *) jetson_die 2 "unknown option \"$1\" for link.sh $CMD. Fix: see  bash \"$PC_DIR/link.sh\" --help" ;;
  esac
  shift
done
case "$CMD" in start|stop|status|drop|_watch|_steps) ;; *) jetson_die 2 "unknown command \"$CMD\" (start, stop, status, drop). Fix: bash \"$PC_DIR/link.sh\" --help" ;; esac

ASTRA_SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"
STATE_DIR="$ASTRA_SIM_HOME/state"
STATE="$STATE_DIR/jetson_link.env"
CTL="$STATE_DIR/jetson_local.ctl"
mkdir -p "$STATE_DIR" "$ASTRA_SIM_HOME/logs"
[ -n "$INSTANCE" ] || INSTANCE="$(astra_state_value sitl_instance 0)"
case "$INSTANCE" in ''|*[!0-9]*) jetson_die 2 "--instance must be a number (got '$INSTANCE')" ;; esac
SITL_PORT=$((5763 + 10 * INSTANCE))

# The link log: the run folder when demo.sh gives one, else the logs folder.
if [ -n "$RUN_DIR" ]; then mkdir -p "$RUN_DIR/jetson"; LOG="$RUN_DIR/jetson/link.log"
else LOG="${LINK_LOG:-$ASTRA_SIM_HOME/logs/jetson_link.log}"; fi
log() { printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "$LOG"; }

# ---- the state file (KEY=VALUE, read by status, demo.sh and the run summary) ---------------------------------------
# state_get KEY [DEFAULT]
state_get() { local v; v=$(awk -F= -v k="$1" '$1 == k { sub(/^[^=]*=/, ""); print; exit }' "$STATE" 2>/dev/null); printf '%s\n' "${v:-${2:-}}"; }
# state_set KEY=VALUE ... : change some keys, keep the others (atomic rewrite)
state_set() {
  local tmp="$STATE.tmp$$" kv k
  touch "$STATE"
  cp "$STATE" "$tmp" 2>/dev/null || : > "$tmp"
  for kv in "$@"; do
    k="${kv%%=*}"
    grep -v "^$k=" "$tmp" > "$tmp.2" 2>/dev/null; mv -f "$tmp.2" "$tmp"
    printf '%s\n' "$kv" >> "$tmp"
  done
  mv -f "$tmp" "$STATE"
}

# ---- the reconnect steps (HIL_DESIGN.md 15.3 g), as shell text run ON the Jetson through that link's connection ------
# link_socks A|B : this link's socket files (never tap.sock: the tap sender owns it)
link_socks() { case "$1" in A) echo "mavlink.sock" ;; B) echo "cam_sdk.sock cam_http.sock cam_rtsp.sock" ;; esac; }
# link_specs A|B : this link's -R forwards (the doors on the Jetson)
link_specs() {
  local d="$JETSON_DATA"
  case "$1" in
    A) printf '%s\n' -R "$d/sock/mavlink.sock:127.0.0.1:$SITL_PORT" ;;
    B) printf '%s\n' -R "$d/sock/cam_sdk.sock:127.0.0.1:$SDK_PORT" -R "$d/sock/cam_http.sock:127.0.0.1:$HTTP_PORT" \
                     -R "$d/sock/cam_rtsp.sock:$CAMERA_IP:8554" ;;
  esac
}
# tap_specs : link A's -L forward (the tap's door on THIS PC, 127.0.0.1:7461). Added on its own, after the -R doors:
# after a drop the OLD ssh may still hold port 7461 for a few seconds (until its keepalive gives up), and that must
# never stop the autopilot door from coming back (found with a real sshd, 2026-10-31). It is retried until it works.
tap_specs() { printf '%s\n' -L "127.0.0.1:$TAP_PORT:$JETSON_DATA/sock/tap.sock"; }
# add_tap : 0 = the tap door is in place; 1 = not yet (logged; the watcher tries again)
add_tap() {
  local out specs=()
  mapfile -t specs < <(tap_specs)
  jetson_forward cancel A "${specs[@]}" >/dev/null 2>&1 || true
  if out=$(jetson_forward add A "${specs[@]}" 2>&1); then
    log "link A: tap forward added (${specs[*]})"; state_set "TAP=up"; return 0
  fi
  log "link A: the tap forward (PC port $TAP_PORT) is not in place yet: $out (an old connection may still hold the port; retrying)"
  state_set "TAP=pending"
  return 1
}
# remote_clean_cmd A|B : step 2. Kill the PREVIOUS connection's sshd only when it is still that very process (same pid
# AND same start time, an sshd/sshd-session of OUR user, not the current connection: the team logs in as the same
# user), then remove this link's stale socket files (sshd never replaces an old socket file by itself).
remote_clean_cmd() {
  local x="$1" d="$JETSON_DATA" f socks=""
  for f in $(link_socks "$x"); do socks+=" '$d/sock/$f'"; done
  # shellcheck disable=SC2016  # ($PPID, $p ... are meant for the remote shell)
  printf '%s' "mkdir -p '$d/sock' '$d/run' && chmod 700 '$d/sock'; cur=\$PPID; p=''; st=''; " \
    "[ -f '$d/run/link_$x.pid' ] && read -r p st < '$d/run/link_$x.pid'; c=\$(cat /proc/\$p/comm 2>/dev/null); " \
    "if [ -n \"\$p\" ] && [ \"\$p\" != \"\$cur\" ] && { [ \"\$c\" = sshd ] || [ \"\$c\" = sshd-session ]; } && " \
    "[ -O /proc/\$p ] && [ \"\$(awk '{print \$22}' /proc/\$p/stat 2>/dev/null)\" = \"\$st\" ]; then " \
    "kill \"\$p\" && echo \"killed old session \$p\"; fi; rm -f$socks"
}
# remote_record_cmd A|B : step 4. Remember this connection's sshd (pid + start time) for the next reconnect.
remote_record_cmd() {
  # shellcheck disable=SC2016
  printf '%s' "echo \"\$PPID \$(awk '{print \$22}' /proc/\$PPID/stat)\" > '$JETSON_DATA/run/link_$1.pid'; " \
    "cat /proc/\$PPID/comm"
}

# master_pid SOCKET : the pid of the SSH master on SOCKET ("" when none answers). ssh -O check prints
# "Master running (pid=1234)".
master_pid() {
  "${JETSON_SSH_BIN:-ssh}" -S "$1" -O check -o LogLevel=ERROR "$JETSON_DEST" 2>&1 | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | head -1
}

# connect_link A|B : steps 2-4 for one link. 0 = forwards in place.
connect_link() {
  local x="$1" out specs=()
  mapfile -t specs < <(link_specs "$x")
  out=$(jetson_ssh_x "$x" "$(remote_clean_cmd "$x")" 2>&1) || { log "link $x: clean step failed: $out"; return 1; }
  [ -z "$out" ] || log "link $x: $out"
  jetson_forward cancel "$x" "${specs[@]}" >/dev/null 2>&1 || true
  if ! out=$(jetson_forward add "$x" "${specs[@]}" 2>&1); then
    log "link $x: adding the forwards failed: $out"
    printf '%s\n' "$out" >&2
    return 1
  fi
  out=$(jetson_ssh_x "$x" "$(remote_record_cmd "$x")" 2>&1) || log "link $x: could not record the session pid: $out"
  log "link $x: forwards added (${specs[*]}); session process: ${out:-?}"
  [ "$x" != A ] || add_tap || true          # (the tap door: retried by the watcher when the port is still held)
  return 0
}
jetson_ssh_x() { if [ "$1" = B ]; then shift; jetson_ssh_b "$@"; else shift; jetson_ssh "$@"; fi; }

# ---- the PC-side relays (hil_relay.py, written by the JETSON-SIDE builder) ------------------------------------------
need_relay() {
  [ -f "$RELAY" ] && return 0
  jetson_die 14 "the relay program $RELAY is missing, so the link cannot start. Likely cause: the Jetson kit is not" \
    "complete in this checkout yet (hil_relay.py comes with simulation/jetson/lib). Fix: update the checkout (git pull)," \
    "or run the mirror+build step: bash simulation/setup/03_build.sh"
}
port_busy_tcp() { ss -ltnH 2>/dev/null | awk '{print $4}' | grep -qE "(^|[:.])$1\$"; }
alive() { [ -n "${1:-}" ] && kill -0 "$1" 2>/dev/null; }

start_pc_relays() {
  local pace jd="${RUN_DIR:+$RUN_DIR/jetson}"; jd="${jd:-$ASTRA_SIM_HOME/logs}"
  pace="$("$PY" "$PC_DIR/hil_pc.py" pace --mode "$PACE" --auth "$JETSON_AUTH" --camera "$CAMERA" \
           --speed-file "$JETSON_STATE/speed.env")" || exit 2
  for p in $SDK_PORT $HTTP_PORT; do
    port_busy_tcp "$p" && jetson_die 14 "TCP 127.0.0.1:$p (a relay port of the Jetson link) is already in use. Likely" \
      "cause: the link of an earlier run is still up. Fix: bash \"$PC_DIR/link.sh\" stop   (then start again)"
  done
  setsid "$PY" "$RELAY" pc-sdk --listen "127.0.0.1:$SDK_PORT" --camera "$CAMERA_IP:37260" --idle-timeout 4 \
      --log "$jd/pc_sdk.log" --status-file "$jd/pc_sdk.status" > /dev/null 2>> "$jd/pc_sdk.log" < /dev/null &
  local sdk=$!
  setsid "$PY" "$RELAY" pc-http --listen "127.0.0.1:$HTTP_PORT" --camera "$CAMERA_IP:82" --pace-mbit "$pace" \
      --http-idle 15 --log "$jd/pc_http.log" --status-file "$jd/pc_http.status" > /dev/null 2>> "$jd/pc_http.log" < /dev/null &
  local http=$!
  sleep 1
  alive "$sdk" && alive "$http" || jetson_die 14 "the PC-side camera relays stopped right after they started. Likely" \
    "cause: a port is taken or hil_relay.py failed. Fix: read $jd/pc_sdk.log and $jd/pc_http.log"
  state_set "SDK_PID=$sdk" "HTTP_PID=$http" "PACE_MBIT=$pace"
  log "PC relays: pc-sdk pid $sdk (127.0.0.1:$SDK_PORT -> $CAMERA_IP:37260), pc-http pid $http" \
      "(127.0.0.1:$HTTP_PORT -> $CAMERA_IP:82, pace ${pace} Mbit/s; 0 = none)"
  info "camera relays on the PC: SDK 127.0.0.1:$SDK_PORT, HTTP 127.0.0.1:$HTTP_PORT (photo pace ${pace} Mbit/s, 0 = none)"
}

start_virtual() {
  local jd="${RUN_DIR:+$RUN_DIR/jetson}"; jd="${jd:-$ASTRA_SIM_HOME/logs}"
  mkdir -p "$JETSON_DATA/sock" && chmod 700 "$JETSON_DATA/sock"
  rm -f "$CTL"
  setsid "$PY" "$RELAY" pc-local --sock-dir "$JETSON_DATA/sock" --sitl "127.0.0.1:$SITL_PORT" \
      --camera-sdk "$CAMERA_IP:37260" --camera-http "$CAMERA_IP:82" --camera-rtsp "$CAMERA_IP:8554" --control "$CTL" \
      --log "$jd/pc_local.log" --status-file "$jd/pc_local.status" > /dev/null 2>> "$jd/pc_local.log" < /dev/null &
  local pid=$!
  for _ in $(seq 1 20); do
    [ -S "$JETSON_DATA/sock/mavlink.sock" ] && [ -S "$JETSON_DATA/sock/cam_sdk.sock" ] && break
    alive "$pid" || break
    sleep 0.5
  done
  if ! alive "$pid" || [ ! -S "$JETSON_DATA/sock/mavlink.sock" ]; then
    jetson_die 14 "the virtual link (hil_relay.py pc-local) did not make its socket files in $JETSON_DATA/sock. Likely" \
      "cause: an error in the relay (read $jd/pc_local.log) or no write permission there. Fix: read that log, then" \
      "bash \"$PC_DIR/link.sh\" stop --link virtual and start again"
  fi
  state_set "LOCAL_PID=$pid" "A=up" "B=up"
  log "virtual link: hil_relay.py pc-local pid $pid, sockets in $JETSON_DATA/sock (SITL 127.0.0.1:$SITL_PORT)"
  ok "virtual link up: socket files in $JETSON_DATA/sock (no SSH; SITL SERIAL2 127.0.0.1:$SITL_PORT)"
}

# ---- the watcher: notices drops, waits for the login, reconnects (runs in the background during a flight) -----------
watch_loop() {
  local was_up=1 down_since=0 drops last_note=0 m_a m_b old_a old_b
  drops="$(state_get DROPS 0)"
  old_a="$(state_get MASTER_A)"; old_b="$(state_get MASTER_B)"
  trap 'log "watcher stopped"; exit 0' TERM INT
  log "watcher started (pid $$): checks the link every 2 s"
  while true; do
    # (sleep in the background and wait for it: bash runs the TERM trap only between commands, so a plain
    # "sleep 2" would delay link.sh stop by up to 2 s; "wait" is interrupted by the signal at once)
    sleep 2 & wait $!
    # the PC relays must stay up (restart one that died: a camera relay crash must not end the flight)
    # (in a subshell: a failure there prints and exits the subshell only, never the watcher)
    if [ "$JETSON_LINK" != virtual ]; then
      alive "$(state_get SDK_PID)" && alive "$(state_get HTTP_PID)" || { log "a PC relay stopped: restarting both";
        kill "$(state_get SDK_PID)" "$(state_get HTTP_PID)" 2>/dev/null; sleep 1; ( start_pc_relays ) >/dev/null 2>&1; }
    else
      alive "$(state_get LOCAL_PID)" || { log "the virtual link stand-in stopped: starting it again"; ( start_virtual ) >/dev/null 2>&1; }
      continue
    fi
    if jetson_logged_in; then
      m_a="$(master_pid "$JETSON_CM_A")"; m_b="$(master_pid "$JETSON_CM_B")"
      if [ "$was_up" = 1 ] && [ "$m_a" = "$old_a" ] && [ "$m_b" = "$old_b" ]; then
        [ "$(state_get TAP up)" = up ] || add_tap || true      # the tap door still waiting for its port
        continue
      fi
      # a new connection (the owner logged in again, or key mode reopened it): the reconnect steps for A and B
      if connect_link A && connect_link B; then
        if [ "$was_up" = 0 ]; then
          local down=$(( $(date +%s) - down_since ))
          drops=$((drops + 1))
          state_set "A=up" "B=up" "DROPS=$drops" "DOWN_S=$(( $(state_get DOWN_S 0) + down ))" "SINCE=$(date +%s)" \
                    "MASTER_A=$m_a" "MASTER_B=$m_b"
          log "link back after ${down} s (drop #$drops)"
          printf '\033[1;32m OK %s\033[0m\n' "[link] the Jetson link is back after ${down} s (drop #$drops)" >&2
        else
          state_set "A=up" "B=up" "MASTER_A=$m_a" "MASTER_B=$m_b"
          log "the SSH connection changed (new master $m_a): forwards added again"
        fi
        was_up=1; old_a="$m_a"; old_b="$m_b"
      fi
      continue
    fi
    # down
    if [ "$was_up" = 1 ]; then
      was_up=0; down_since=$(date +%s); last_note=0; old_a=""; old_b=""
      state_set "A=down" "B=down" "DOWN_SINCE=$down_since"
      log "LINK DOWN: no SSH connection to $JETSON_DEST (auth $JETSON_AUTH)"
      warn "the link to the Jetson ($JETSON_DEST) dropped. Consequence: the team stack hears no autopilot and no camera" \
           "until it is back; the drone flies on (AUTO) or holds (GUIDED) like a real drone with a silent companion."
      if [ "$JETSON_AUTH" = login ]; then
        warn_more "Fix: in the Jetson login window press Enter and type the password (or: bash \"$J_DIR/jetson.sh\" login)."
      else
        warn_more "link.sh reconnects by itself every 2 s (key mode). Check the cable / Wi-Fi if it does not come back."
      fi
    fi
    if [ $(( $(date +%s) - last_note )) -ge 30 ] && [ "$last_note" != 0 ]; then
      warn "the Jetson link has been down for $(( $(date +%s) - down_since )) s. Fix: $( [ "$JETSON_AUTH" = login ] && \
        echo "log in again in the Jetson login window (Enter, then the password)" || echo "check the cable / Wi-Fi")"
      last_note=$(date +%s)
    fi
    [ "$last_note" != 0 ] || last_note=$(date +%s)
  done
}

# ---- commands --------------------------------------------------------------------------------------------------------
cmd_start() {
  need_relay
  state_set "LINK=$JETSON_LINK" "HOST=$JETSON_HOST" "AUTH=$JETSON_AUTH" "USER=$JETSON_USER" "INSTANCE=$INSTANCE" \
            "SITL_PORT=$SITL_PORT" "RUN_DIR=$RUN_DIR" "DROPS=0" "DOWN_S=0" "SINCE=$(date +%s)" "A=down" "B=down" \
            "LOG=$LOG" "DATA=$JETSON_DATA"
  log "link start: $JETSON_LINK $JETSON_DEST (auth $JETSON_AUTH), SITL SERIAL2 127.0.0.1:$SITL_PORT"
  if [ "$JETSON_LINK" = virtual ]; then
    start_virtual
  else
    port_busy_tcp "$TAP_PORT" && jetson_die 14 "TCP 127.0.0.1:$TAP_PORT (the tap's forward) is already in use. Likely cause:" \
      "the link of an earlier run is still up. Fix: bash \"$PC_DIR/link.sh\" stop   (then start again)"
    if ! jetson_logged_in; then jetson_need_login; exit 17; fi
    jetson_resolve_data || jetson_die 14 "could not ask the Jetson for its home folder through the login. Likely cause:" \
      "the connection dropped just now. Fix: bash \"$J_DIR/jetson.sh\" check"
    state_set "DATA=$JETSON_DATA"
    start_pc_relays
    if ! connect_link A || ! connect_link B; then
      jetson_die 14 "the SSH forwards to the Jetson could not be added (the message above says why). Likely cause: the" \
        "Jetson's sshd does not allow forwarding (AllowStreamLocalForwarding / AllowTcpForwarding no), the folder" \
        "$JETSON_DATA/sock is missing, or the connection dropped. Fix: bash \"$J_DIR/jetson.sh\" check (row 12 tests" \
        "forwarding), then run this again; details: $LOG"
    fi
    state_set "A=up" "B=up" "MASTER_A=$(master_pid "$JETSON_CM_A")" "MASTER_B=$(master_pid "$JETSON_CM_B")"
    ok "Jetson link up over $JETSON_LINK ($JETSON_DEST, $JETSON_AUTH mode): autopilot, camera and tap forwards in place"
  fi
  if [ "$WATCH" = 1 ]; then
    # The watcher runs for the whole flight, so it must NOT keep this script's stdout/stderr: demo.sh reads
    # link.sh's output through a pipe ("| sed"), and a background process that still holds the pipe means the pipe
    # never ends, so demo.sh would wait forever (it did in the first virtual-Jetson flight, 2026-10-31 15:33). Its
    # messages go to the link log instead; demo.sh (hil_tick in hil_steps.sh) reports drops and returns in its
    # own terminal from the state file the watcher keeps ($STATE: A=up|down, DROPS, DOWN_S).
    setsid bash "$0" _watch "${ORIG_ARGS[@]}" < /dev/null >> "$LOG" 2>&1 &
    state_set "WATCH_PID=$!"
    info "link watcher pid $! (it notices drops and reconnects; log $LOG)"
  fi
}

cmd_stop() {
  local p
  p="$(state_get WATCH_PID)"; alive "$p" && kill "$p" 2>/dev/null
  if [ "$JETSON_LINK" = virtual ]; then
    p="$(state_get LOCAL_PID)"; alive "$p" && kill "$p" 2>/dev/null
  else
    if [ -n "$JETSON_DATA" ] || [ -n "$(state_get DATA)" ]; then
      JETSON_DATA="${JETSON_DATA:-$(state_get DATA)}"
      local specs=()
      for x in A B; do
        mapfile -t specs < <(link_specs "$x")
        jetson_forward cancel "$x" "${specs[@]}" >/dev/null 2>&1 || true
      done
      mapfile -t specs < <(tap_specs)
      jetson_forward cancel A "${specs[@]}" >/dev/null 2>&1 || true
    fi
    for k in SDK_PID HTTP_PID; do p="$(state_get "$k")"; alive "$p" && kill "$p" 2>/dev/null; done
  fi
  state_set "A=down" "B=down" "STOPPED=$(date +%s)" "WATCH_PID=" "SDK_PID=" "HTTP_PID=" "LOCAL_PID="
  log "link stop (drops $(state_get DROPS 0), down $(state_get DOWN_S 0) s)"
  ok "Jetson link stopped (forwards cancelled, PC relays stopped; the login window stays open)"
}

cmd_status() {
  local up=0 a b
  a="$(state_get A down)"; b="$(state_get B down)"
  if [ "$JETSON_LINK" = virtual ]; then
    alive "$(state_get LOCAL_PID)" && [ -S "$JETSON_DATA/sock/mavlink.sock" ] && up=1
  else
    jetson_logged_in && [ "$a" = up ] && [ "$b" = up ] && alive "$(state_get SDK_PID)" && up=1
  fi
  if [ "$JSON" = 1 ]; then
    printf '{"link": "%s", "host": "%s", "auth": "%s", "up": %s, "A": "%s", "B": "%s", "drops": %s, "down_s": %s, "watcher": %s}\n' \
      "$JETSON_LINK" "$JETSON_HOST" "$JETSON_AUTH" "$([ $up = 1 ] && echo true || echo false)" "$a" "$b" \
      "$(state_get DROPS 0)" "$(state_get DOWN_S 0)" "$(alive "$(state_get WATCH_PID)" && echo true || echo false)"
  else
    say "Jetson link ($JETSON_LINK, $JETSON_DEST, $JETSON_AUTH mode)"
    info "link A (autopilot + tap): $a    link B (camera): $b    drops: $(state_get DROPS 0) (down $(state_get DOWN_S 0) s)"
    info "watcher: $(alive "$(state_get WATCH_PID)" && echo running || echo 'not running')    log: $(state_get LOG "$LOG")"
    if [ "$up" = 1 ]; then ok "the link is up"; else warn "the link is down or not started. Fix: bash \"$PC_DIR/link.sh\" start (demo.sh --jetson does it), or log in again: bash \"$J_DIR/jetson.sh\" login"; fi
  fi
  [ "$up" = 1 ] || exit 14
}

cmd_drop() {
  case "$DROP_S" in ''|*[!0-9]*) jetson_die 2 "link.sh drop needs the number of seconds first, e.g. link.sh drop 20 --mode blackhole" ;; esac
  if [ "$JETSON_LINK" = virtual ]; then
    MODE="${MODE:-blackhole}"
    case "$MODE" in blackhole|close) ;; *) jetson_die 2 "the virtual link rehearses --mode blackhole or close (glitch / cut are for --link usb|ssh)" ;; esac
    need_relay
    log "rehearsal: drop $DROP_S s, mode $MODE"
    "$PY" "$RELAY" drop --control "$CTL" --seconds "$DROP_S" --mode "$MODE" || jetson_die 1 "the drop command failed (see above)"
    ok "virtual link: $MODE for $DROP_S s"
    return
  fi
  MODE="${MODE:-glitch}"
  local pids=() p
  for s in "$JETSON_CM_A" "$JETSON_CM_B"; do p="$(master_pid "$s")"; [ -n "$p" ] && pids+=("$p"); done
  mapfile -t pids < <(printf '%s\n' "${pids[@]}" | sort -u)
  [ "${#pids[@]}" -gt 0 ] && [ -n "${pids[0]}" ] || { jetson_need_login; exit 17; }
  case "$MODE" in
    glitch)
      log "rehearsal: glitch $DROP_S s (SIGSTOP the SSH master(s) ${pids[*]})"
      kill -STOP "${pids[@]}" || jetson_die 1 "could not freeze the SSH connection ${pids[*]}"
      info "SSH connection frozen for $DROP_S s (the Jetson sees silence, like a pulled cable)"
      sleep "$DROP_S"; kill -CONT "${pids[@]}"
      log "rehearsal: glitch over (SIGCONT)"; ok "the SSH connection runs again" ;;
    cut)
      [ "$JETSON_AUTH" = key ] || jetson_die 2 "--mode cut ends the connection; in login mode that is the owner's own login window. Use --mode glitch, or pull the cable for the real test (docs/jetson/LINK.md)"
      log "rehearsal: cut after $DROP_S s (SIGSTOP, then SIGKILL the SSH master(s) ${pids[*]})"
      kill -STOP "${pids[@]}"; sleep "$DROP_S"; kill -KILL "${pids[@]}"
      ok "SSH connection killed: the watcher reconnects with the key within a few seconds" ;;
    *) jetson_die 2 "unknown --mode $MODE for --link $JETSON_LINK (glitch or cut)" ;;
  esac
}

ORIG_ARGS=()
[ -n "$RUN_DIR" ] && ORIG_ARGS+=(--run-dir "$RUN_DIR")
ORIG_ARGS+=(--link "$JETSON_LINK" --host "$JETSON_HOST" --port "$JETSON_PORT" --auth "$JETSON_AUTH" --instance "$INSTANCE"
            --camera "$CAMERA" --photo-pace "$PACE")
[ -n "$JETSON_USER" ] && ORIG_ARGS+=(--user "$JETSON_USER")

case "$CMD" in
  start) cmd_start ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  drop) cmd_drop ;;
  _watch) JETSON_DATA="${JETSON_DATA:-$(state_get DATA)}"; watch_loop ;;
  # (tests only: print the reconnect steps of links A and B, one per line: clean A, record A, specs A, clean B ...)
  _steps) for x in A B; do remote_clean_cmd "$x"; echo; remote_record_cmd "$x"; echo
            { link_specs "$x"; [ "$x" != A ] || tap_specs; } | paste -sd' '; done ;;
esac
