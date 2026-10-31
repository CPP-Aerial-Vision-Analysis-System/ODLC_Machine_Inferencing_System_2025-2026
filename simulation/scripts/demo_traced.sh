#!/usr/bin/env bash
# =============================================================================
#  demo_traced.sh  --  THE SEARCH DEMO WITH THE LIVE CODE MAP (functions light up while they run)
#
#  PURPOSE: runs the normal visible search demo (demo.sh, with the same options) with the code map's live
#  tracers switched on, so a browser shows which file and function is working at every moment:
#    1. starts the code map server (simulation/codemap/server.py) in the background of THIS script (it lives
#       as long as this script): it listens for the tracers on UDP 127.0.0.1:8091, serves the page on
#       http://127.0.0.1:8090/ and records every event it sends into a recording file (replay it later, or
#       build the shareable artifact page from it with simulation/codemap/build_artifact.py);
#    2. runs  bash simulation/scripts/demo.sh <your options>  with ASTRA_TRACE=1 and PYTHONPATH starting with
#       simulation/codemap/trace of the copy that runs (the Linux-side copy ~/ODLC_Machine_Inferencing_System_2025-2026
#       made by setup/03_build.sh). Then every Python program of the run counts the calls of the team's
#       functions (trace/sitecustomize.py + trace/astra_trace.py), and every simulator bash script reports its
#       functions (trace/astra_trace.sh, switched on by _common.sh). The demo itself runs exactly as usual;
#    3. at the end (after the landing, after Ctrl+C, or after Enter with --keep / --manual) it stops the
#       server and copies the recording into the demo's run folder as codemap_trace.jsonl (and the server's
#       last /health answer, the calls it could not place, as codemap_health.json). It prints where the
#       recording is and the command that replays it later (server.py lists both default recording folders,
#       see --record-dir, by itself; any other folder with --recordings DIR).
#  Open http://127.0.0.1:8090/ in a browser while it runs (a browser on Windows works too: WSL forwards
#  localhost to Ubuntu).
#
#  HOW TO RUN:   bash demo_traced.sh                                  -> runway world, like demo.sh
#                bash demo_traced.sh runway --detector truth --keep   (every demo.sh option works)
#                bash demo_traced.sh --map-only                       -> only the server, for runs by hand
#  Options of this script (every other option goes to demo.sh):
#                --map-port N       port of the web page (default 8090)
#                --trace-port N     UDP port the tracers send to (default 8091)
#                --no-record        do not write a recording
#                --record-dir DIR   where the recording goes. Default: simulation/codemap/recordings of the
#                                   checkout this script lives in when that checkout is a git checkout (the
#                                   Windows one, /mnt/...); for the Linux-side copy (no .git, overwritten by
#                                   03_build.sh) ~/.astra_sim/codemap/recordings instead
#                --map-only         start only the server and print the lines that switch tracing on in your
#                                   own terminals (start_sim.sh, ./launcher.sh sim ...); Enter stops it
#  Needs: simulation/codemap/codemap.json (made by simulation/codemap/extract.py) to name the blocks; without
#  it the server still runs but cannot place the calls (see http://127.0.0.1:8090/health).
#  The tracers make the traced programs slower (Python: about 0.2-0.4 microseconds per call, real tests
#  +15-40 %; bash: about 15 microseconds per command, about 0.3 ms per call of one of our functions in a busy
#  loop): numbers in simulation/codemap/README.md ("How much slower does tracing make things?").
#  MAVProxy (busy, and not our code) is left untraced on purpose (ASTRA_TRACE_SKIP, see astra_trace.py).
# =============================================================================
set -o pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$HERE/_common.sh"

usage() { astra_usage "$0"; }
# A mistake in the command line: exit code 2 with the --help hint (_common.sh's die_usage; plain die if it is gone).
usage_error() { if declare -F die_usage > /dev/null; then die_usage "$*"; else die "$* (see --help)"; fi; }

MAP_PORT=8090; TRACE_PORT=8091; RECORD=1; RECORD_DIR=""; MAP_ONLY=0; DRY=0; KEEPISH=0; DEMO_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --map-port) shift; MAP_PORT="${1:-}" ;;
    --trace-port) shift; TRACE_PORT="${1:-}" ;;
    --no-record) RECORD=0 ;;
    --record-dir) shift; RECORD_DIR="${1:-}"; [ -n "$RECORD_DIR" ] || usage_error "--record-dir needs a folder" ;;
    --map-only) MAP_ONLY=1 ;;
    *) case "$1" in --dry-run) DRY=1 ;; --keep|--manual) KEEPISH=1 ;; esac
       DEMO_ARGS+=("$1") ;;
  esac
  shift
done
for n in "$MAP_PORT" "$TRACE_PORT"; do
  case "$n" in ''|*[!0-9]*) usage_error "--map-port and --trace-port need a port number, not \"$n\"" ;; esac
done

# ---- which copy runs ---------------------------------------------------------------------------------------
# Like setup/sim_env.sh: from the Windows checkout (/mnt/...) the simulator runs the Linux-side copy.
THIS_REPO="$(cd "$HERE/../.." && pwd)"
RUN_REPO="$THIS_REPO"
LINUX_COPY="$HOME/ODLC_Machine_Inferencing_System_2025-2026"
case "$THIS_REPO" in
  /mnt/*) if [ -d "$LINUX_COPY/ros2_ws" ]; then RUN_REPO="$LINUX_COPY"; fi ;;
esac
CODEMAP_DIR="$RUN_REPO/simulation/codemap"
TRACE_DIR="$CODEMAP_DIR/trace"
SERVER="$CODEMAP_DIR/server.py"
if [ ! -f "$TRACE_DIR/sitecustomize.py" ] || [ ! -f "$TRACE_DIR/astra_trace.sh" ] || [ ! -f "$SERVER" ]; then
  die "$RUN_REPO has no simulation/codemap (server.py + trace/) yet. Likely cause: this copy was made before the
    code map existed. Fix: bash \"$THIS_REPO/simulation/setup/03_build.sh\"   (then run this command again)"
fi
case "$TRACE_DIR" in
  *:*) die "the trace folder $TRACE_DIR has a \":\" in its path. Likely cause: an unusual folder name. Consequence:
    it cannot go on PYTHONPATH (\":\" separates entries). Fix: use a copy of the repo in a folder without \":\"" ;;
esac
[ -f "$CODEMAP_DIR/codemap.json" ] || warn "no $CODEMAP_DIR/codemap.json yet. Consequence: the page cannot name the" \
  "blocks and the calls cannot be placed. Fix: python3 \"$THIS_REPO/simulation/codemap/extract.py\", then" \
  "bash \"$THIS_REPO/simulation/setup/03_build.sh\""

SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"
if [ -z "$RECORD_DIR" ]; then
  if [ -e "$THIS_REPO/.git" ]; then RECORD_DIR="$THIS_REPO/simulation/codemap/recordings"
  else RECORD_DIR="$SIM_HOME/codemap/recordings"; fi
fi
STAMP="$(date +%Y%m%d-%H%M%S)"
REC_FILE=""
SERVER_ARGS=(--port "$MAP_PORT" --udp-port "$TRACE_PORT" --codemap "$CODEMAP_DIR/codemap.json")
if [ "$RECORD" = 1 ]; then
  REC_FILE="$RECORD_DIR/$STAMP.jsonl"
  SERVER_ARGS+=(--record "$REC_FILE")
fi
TRACE_ENV="ASTRA_TRACE=1 ASTRA_TRACE_ADDR=127.0.0.1:$TRACE_PORT PYTHONPATH=\"$TRACE_DIR\${PYTHONPATH:+:\$PYTHONPATH}\""

if [ "$DRY" = 1 ]; then
  say "DRY RUN of the traced demo: this is what would run (nothing was started)"
  echo "    A. python3 $SERVER ${SERVER_ARGS[*]}      (in the background; open http://127.0.0.1:$MAP_PORT/)"
  echo "    B. cd $RUN_REPO && $TRACE_ENV bash simulation/scripts/demo.sh ${DEMO_ARGS[*]}"
  echo "    C. stop the server; copy the recording into the run folder"
  echo "    demo.sh's own plan:"
  bash "$RUN_REPO/simulation/scripts/demo.sh" "${DEMO_ARGS[@]}"
  exit $?
fi

# ---- 1. the code map server ---------------------------------------------------------------------------------
map_healthy() {   # true when a code map server answers on MAP_PORT
  python3 - "$MAP_PORT" > /dev/null 2>&1 <<'PY'
import sys, urllib.request
urllib.request.urlopen('http://127.0.0.1:%s/health' % sys.argv[1], timeout=1).read()
PY
}
SERVER_PID=""; CLEANED=0; INTERRUPTED=0; OLD_RUN=""
cleanup() {      # stop the server we started; keep and point to the recording (runs once, at exit)
  [ "$CLEANED" = 1 ] && return 0
  CLEANED=1
  if [ -n "$SERVER_PID" ] && kill -0 "$SERVER_PID" 2>/dev/null; then
    info "stopping the code map server ..."
    kill -TERM "$SERVER_PID" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 "$SERVER_PID" 2>/dev/null || break; sleep 0.25; done
    kill -KILL "$SERVER_PID" 2>/dev/null
  fi
  if [ -n "$REC_FILE" ] && [ -s "$REC_FILE" ]; then
    local new_run
    new_run="$(ls -1dt "$SIM_HOME"/runs/*/ 2>/dev/null | head -1)"
    if [ -n "$new_run" ] && [ "$new_run" != "$OLD_RUN" ] && cp "$REC_FILE" "${new_run%/}/codemap_trace.jsonl" 2>/dev/null; then
      info "the recording is also in the run folder: ${new_run%/}/codemap_trace.jsonl"
      # server.py's last /health answer (which calls it could not place), written when it stopped
      cp "${REC_FILE%.jsonl}.health.json" "${new_run%/}/codemap_health.json" 2>/dev/null || true
    fi
    ok "live code map recording: $REC_FILE ($(wc -l < "$REC_FILE") events)"
    info "replay it later (then open http://127.0.0.1:8090/ and pick it in the Replay tab):"
    info "    python3 \"$THIS_REPO/simulation/codemap/server.py\" --recordings \"$RECORD_DIR\""
    info "or make the shareable page from it:"
    info "    python3 \"$THIS_REPO/simulation/codemap/build_artifact.py\" --recording \"$REC_FILE\""
  fi
  return 0
}
trap cleanup EXIT
trap 'INTERRUPTED=1' INT TERM      # Ctrl+C reaches demo.sh too; it finishes cleanly, then we clean up

if map_healthy; then
  warn "a code map server already answers on http://127.0.0.1:$MAP_PORT/: using it (it records only if it was" \
       "started with --record; this script will not stop it)"
  REC_FILE=""
else
  LOG_DIR="$SIM_HOME/logs"
  mkdir -p "$LOG_DIR"
  [ -z "$REC_FILE" ] || mkdir -p "$RECORD_DIR" || die "cannot create the recording folder $RECORD_DIR. Likely" \
    "cause: no write permission there. Fix: --record-dir <a folder you own>, or --no-record"
  SERVER_LOG="$LOG_DIR/codemap_server_$STAMP.log"
  say "Starting the live code map server"
  # Not traced itself, and not part of the simulator session (stop_sim.sh must never stop it).
  ( unset ASTRA_TRACE ASTRA_SIM_SESSION; exec python3 "$SERVER" "${SERVER_ARGS[@]}" ) > "$SERVER_LOG" 2>&1 < /dev/null &
  SERVER_PID=$!
  for _ in $(seq 1 40); do
    map_healthy && break
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
      SERVER_PID=""
      tail -5 "$SERVER_LOG" | sed 's/^/    /'
      die "the code map server did not start (its own message is above). Likely cause: another program already" \
          "uses port $MAP_PORT or UDP $TRACE_PORT. Fix: stop it, or choose free ports: --map-port N --trace-port N"
    fi
    sleep 0.25
  done
  map_healthy || die "the code map server does not answer on http://127.0.0.1:$MAP_PORT/ after 10 s. Likely cause:" \
    "a very busy PC or a firewall rule. Fix: read its log $SERVER_LOG, then run this command again"
  info "log: $SERVER_LOG"
  [ -z "$REC_FILE" ] || info "recording: $REC_FILE"
fi
say "LIVE CODE MAP: open http://127.0.0.1:$MAP_PORT/ in a browser (on Windows too)"

if [ "$MAP_ONLY" = 1 ]; then
  say "Only the code map server runs. To trace what you start by hand, run this first in that terminal:"
  echo "    export $TRACE_ENV"
  if [ -t 0 ]; then
    read -r -p "    Press Enter (or Ctrl+C) to stop the code map server " _ || true
  elif [ -n "$SERVER_PID" ]; then
    wait "$SERVER_PID"
  fi
  exit 0
fi

# ---- 2. the demo, traced ------------------------------------------------------------------------------------
OLD_RUN="$(ls -1dt "$SIM_HOME"/runs/*/ 2>/dev/null | head -1)"
export ASTRA_TRACE=1
export ASTRA_TRACE_ADDR="127.0.0.1:$TRACE_PORT"
export PYTHONPATH="$TRACE_DIR${PYTHONPATH:+:$PYTHONPATH}"
say "Running the demo with the tracers on:  bash simulation/scripts/demo.sh ${DEMO_ARGS[*]}"
cd "$RUN_REPO" || die "cannot enter $RUN_REPO"
bash "$RUN_REPO/simulation/scripts/demo.sh" "${DEMO_ARGS[@]}"
RC=$?

# ---- 3. the end ---------------------------------------------------------------------------------------------
if [ "$KEEPISH" = 1 ] && [ "$INTERRUPTED" = 0 ] && [ "$RC" = 0 ] && [ -n "$SERVER_PID" ] && [ -t 0 ]; then
  say "The simulator keeps running (--keep / --manual), and so does the live code map."
  read -r -p "    Press Enter to stop the live code map (the simulator keeps running) " _ || true
fi
exit "$RC"
