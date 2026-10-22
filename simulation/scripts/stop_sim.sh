#!/usr/bin/env bash
# =============================================================================
#  stop_sim.sh  --  stop the ASTRA simulator (and the team stack that demo.sh started)
#
#  PURPOSE: stops ONLY the simulator's own processes: Gazebo (server + window), ArduCopter SITL, MAVProxy,
#  the SIYI camera emulator, the payload release, the bridge, RViz, the demo panel/markers/truth detector,
#  and the team's flight stack when demo.sh started it. It finds them precisely: every process started by
#  start_sim.sh, demo.sh or sim.launch.py carries a session tag (environment variable ASTRA_SIM_SESSION),
#  and only processes of YOUR user with that tag are stopped. Other simulations on the PC (other users,
#  other WSL distros, Gazebo started by hand) are never touched.
#  Order: polite Ctrl+C first (launch files shut their programs down cleanly), then TERM, then KILL.
#
#  HOW TO RUN:   bash stop_sim.sh                 stop the simulator started last (state/session)
#                bash stop_sim.sh --all           stop every tagged simulator process of this user
#                bash stop_sim.sh --session ID    stop one session
#                bash stop_sim.sh --quiet         fewer messages
#  Exit code: 0 = nothing left running, 1 = something survived (its pid is printed).
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

QUIET=0
TARGET=""
while [ $# -gt 0 ]; do
  case "$1" in
    --all) TARGET="*" ;;
    --session) shift; TARGET="${1:-}" ;;
    --quiet|-q) QUIET=1 ;;
    -h|--help) astra_usage "$0"; exit 0 ;;
    *) die "unknown option $1 (use --all, --session ID, --quiet)" ;;
  esac
  shift
done
log() { [ "$QUIET" = 1 ] || say "$*"; }
note() { [ "$QUIET" = 1 ] || info "$*"; }

ASTRA_SIM_HOME="${ASTRA_SIM_HOME:-$HOME/.astra_sim}"
STATE="$ASTRA_SIM_HOME/state"
if [ -z "$TARGET" ]; then
  TARGET=$(cat "$STATE/session" 2>/dev/null)
  # a launch started by hand (ros2 launch sim_bringup sim.launch.py) records its own tag
  if [ -z "$TARGET" ] && [ -f "$STATE/current_launch.yaml" ]; then
    TARGET=$(awk '/^session:/{print $2}' "$STATE/current_launch.yaml" | tr -d "'\"")
  fi
fi
if [ -z "$TARGET" ]; then
  log "No simulator session is recorded (nothing started by start_sim.sh / demo.sh). Use --all to stop every tagged process."
  exit 0
fi

mapfile -t PIDS < <(astra_session_pids "$TARGET")
# Hand-started launches are not tagged themselves (only their children are): add them for --all.
if [ "$TARGET" = "*" ]; then
  while read -r p; do [ -n "$p" ] && PIDS+=("$p"); done < <(pgrep -u "$(id -u)" -f 'ros2 launch sim_bringup (sim|team_stack)\.launch\.py' 2>/dev/null)
fi
if [ ${#PIDS[@]} -eq 0 ]; then
  log "The simulator is not running (session ${TARGET}): nothing to stop."
  rm -f "$STATE/session"
  exit 0
fi

log "Stopping ${#PIDS[@]} simulator process(es) (session ${TARGET})"
alive() { kill -0 "$1" 2>/dev/null; }
count_alive() { local n=0 p; for p in "$@"; do alive "$p" && n=$((n + 1)); done; echo "$n"; }

# Step 1: Ctrl+C to launch files and launcher.sh (they stop their programs in order), then to the rest.
for p in "${PIDS[@]}"; do
  case "$(astra_cmd_of "$p")" in
    *"ros2 launch"*|*launcher.sh*|*demo.sh*) note "Ctrl+C -> $(astra_cmd_of "$p")"; kill -INT "$p" 2>/dev/null ;;
  esac
done
for _ in $(seq 1 12); do [ "$(count_alive "${PIDS[@]}")" -eq 0 ] && break; sleep 1; done
for p in "${PIDS[@]}"; do alive "$p" && kill -INT "$p" 2>/dev/null; done
for _ in $(seq 1 6); do [ "$(count_alive "${PIDS[@]}")" -eq 0 ] && break; sleep 1; done

# Step 2: whatever is left (including children started meanwhile) gets TERM, then KILL.
mapfile -t LEFT < <(astra_session_pids "$TARGET"; for p in "${PIDS[@]}"; do alive "$p" && echo "$p"; done)
if [ ${#LEFT[@]} -gt 0 ]; then
  for p in "${LEFT[@]}"; do alive "$p" && { note "stopping $(astra_cmd_of "$p")"; kill -TERM "$p" 2>/dev/null; }; done
  for _ in $(seq 1 5); do [ "$(count_alive "${LEFT[@]}")" -eq 0 ] && break; sleep 1; done
  for p in "${LEFT[@]}"; do alive "$p" && { note "forcing $(astra_cmd_of "$p")"; kill -KILL "$p" 2>/dev/null; }; done
  sleep 1
fi

sleep 1
mapfile -t FINAL < <(for p in $(astra_session_pids "$TARGET"); do alive "$p" && echo "$p"; done)
if [ ${#FINAL[@]} -eq 0 ]; then
  [ "$TARGET" != "*" ] && [ "$(cat "$STATE/session" 2>/dev/null)" = "$TARGET" ] && rm -f "$STATE/session"
  log "The simulator is stopped."
  exit 0
fi
warn "${#FINAL[@]} process(es) are still running:"
for p in "${FINAL[@]}"; do warn "  $p  $(astra_cmd_of "$p")"; done
exit 1
