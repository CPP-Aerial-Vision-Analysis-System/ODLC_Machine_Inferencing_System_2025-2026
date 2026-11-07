#!/usr/bin/env bash
# =============================================================================
#  mission.sh  --  everything about MISSION PLANS: check one, load it, fly it, make one, list them
#
#  PURPOSE: one command for the mission file a teammate made in Mission Planner (or QGroundControl):
#    check  explains the file in plain words (every item, its altitude in metres and feet, where the photos start
#           and stop, the hold point, the RTL, the lane speed) and checks it has what the team's flight code needs
#           (AGENTS.md section 3.4: home, TAKEOFF, DO_DIGICAM_CONTROL, a hold waypoint before RETURN_TO_LAUNCH) and
#           the SUAS 2026 altitude rule (150-400 ft above the ground). No simulator needed; works for the REAL
#           drone's mission files too. (The checker: ros2_ws/src/simulation/sim_demo/sim_demo/mission_check.py.)
#    load   checks it, then uploads it to the running simulator like Mission Planner's "Write WPs" (through
#           upload_mission.sh = mission_tool upload) and makes the team's MAVROS read it again, so the team's
#           waypoint_manager has the new indices. It refuses a mission with errors unless you add --force.
#    fly    arms the drone and starts the mission (AUTO) through fly_mission.sh; with a FILE it loads it first.
#    make   plans a new lawn-mower survey mission for a world (sim_worlds make_mission.py), then checks it.
#    list   shows the mission files you can load.
#
#  HOW TO RUN (inside Ubuntu; the simulator must run for load and fly, e.g. bash astra.sh manual):
#    bash mission.sh check FILE                 explain + check a mission file (exit 0 = flyable)
#    bash mission.sh load  FILE|WORLD           check, upload, MAVROS re-read
#    bash mission.sh fly   [FILE|WORLD]         (load FILE first) then arm + AUTO; no FILE = fly what is loaded
#    bash mission.sh make  WORLD [options]      new survey mission for runway / suas_field_real
#    bash mission.sh list                       the mission files of the worlds and the ones you made
#  FILE   a Mission Planner .waypoints file (first line "QGC WPL 110") or a QGroundControl .plan file (converted to
#         .waypoints in ~/.astra_sim/missions/ before the upload). A Windows path works too, in quotes:
#         "C:\Users\me\Documents\Mission Planner\field.waypoints"  or  "\\wsl.localhost\Ubuntu-22.04\home\me\m.waypoints"
#  WORLD  runway | suas_field_real : that world's own survey mission
#  Options:
#    check:     --strict (warnings count as errors), --quiet (no item table), --json, --home LAT,LON
#    load, fly: --force (upload even when the check found errors), --url URL (MAVLink link; default: the running
#               simulator's ground-station port, udp 14550)
#    fly:       --watch (print the autopilot's messages until it has landed)
#    make:      --altitude M, --speed MPS, --area NAME, --sidelap F, --frontlap F, --lead-in S, --photo-interval S,
#               --out FILE (default ~/.astra_sim/missions/<world>_survey_<time>.waypoints; the world's own mission
#               file in the repo is never overwritten)
#  Examples:
#    bash mission.sh check ~/Downloads/field.waypoints
#    bash mission.sh make runway --altitude 50 && bash mission.sh list
#    bash demo.sh runway --manual        (window 1)   then   bash mission.sh fly runway --watch   (window 2)
#  Exit codes: 0 = done, 1 = failed or the check found errors (the reason is printed), 2 = command typed wrong,
#              3 = the mission file or world cannot be used (missing, not a mission file, broken line, unknown world),
#              4 = the checker itself failed (a bug in mission_check.py, not in your mission: report it with the file).
#              Every message and its fix: docs/simulation/ERRORS.md
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

# The checker is pure Python and runs straight from the source tree (no build, no ROS needed).
CHECKER_PY="$ASTRA_SCRIPTS_DIR/../../ros2_ws/src/simulation/sim_demo/sim_demo/mission_check.py"
# die_file MESSAGE : the mission file cannot be used (exit code 3, like mission_check).
die_file() { printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "$*")" >&2; exit 3; }

run_checker() {   # run_checker ARGS... : the mission checker's exit code (0 ok, 1 errors, 2 usage, 3 unusable file)
  [ -f "$CHECKER_PY" ] || die "the mission checker is missing ($CHECKER_PY).
    Likely cause: an old or partial copy of the repo.
    Fix: update the checkout, then  bash \"$ASTRA_SETUP_DIR/03_build.sh\""
  python3 "$CHECKER_PY" "$@"
}

# resolve_mission TARGET : sets MISSION to the absolute path of a mission file, or of a world's survey mission.
resolve_mission() {
  local t="$1"
  # A Windows path -> the same file seen from Ubuntu: C:\... or C:/... -> /mnt/c/..., and the path Windows shows for a
  # file inside Ubuntu (\\wsl.localhost\Ubuntu-22.04\home\... or \\wsl$\...; what `mission.sh make` prints) -> /home/...
  case "$t" in
    [A-Za-z]:\\*|[A-Za-z]:/*|\\\\*|//wsl*)
      if command -v wslpath >/dev/null 2>&1; then t="$(wslpath -u "$t" 2>/dev/null || printf '%s' "$t")"; fi ;;
  esac
  if [ -f "$t" ]; then MISSION="$(readlink -f "$t")"; return 0; fi
  case "$t" in
    *.waypoints|*.plan|*.txt|*/*|*\\*)
      die_file "the mission file \"$1\" does not exist.
    Likely cause: a typo in the name, a Windows path without quotes (the backslashes get lost), or a file inside
    another WSL distribution (only this one, ${WSL_DISTRO_NAME:-Ubuntu-22.04}, can be read).
    Fix: check it with  ls \"$t\"   and put a Windows path in quotes: \"C:\\Users\\me\\field.waypoints\"" ;;
  esac
  astra_load_env
  astra_require_built
  MISSION="$(python3 -m sim_bringup.sim_paths world "$t" mission 2>/dev/null)"
  if [ -z "$MISSION" ] || [ ! -f "$MISSION" ]; then
    die_file "\"$1\" is neither a mission file nor a world with a survey mission.
    Likely cause: a typo, or the world has no mission (standin).
    Fix: see the choices with  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" list"
  fi
}

sim_session() { cat "${ASTRA_STATE_DIR:-$HOME/.astra_sim/state}/session" 2>/dev/null; }

# sim_home : the running simulator's home (LAT,LON,ALT,HEADING from state/current_launch.yaml), but ONLY when that
# file belongs to a simulator session whose processes run on this PC now. A stale file (another world ran last, or
# the simulator runs in Docker / on another PC) would turn a good mission into a false "the drone would fly away"
# error, so then nothing is printed and the checker only compares the waypoints with the file's own home.
sim_home() {
  local sess
  sess="$(astra_state_value session "")"
  [ -n "$sess" ] && [ -n "$(astra_session_pids "$sess")" ] || return 0
  astra_state_value home ""
}

CMD="${1:-}"
[ $# -gt 0 ] && shift
case "$CMD" in
  ''|-h|--help|help) astra_usage "$0"; exit 0 ;;
  check|load|fly|make|list) ;;
  *) die_usage "unknown command \"$CMD\" (the commands are: check, load, fly, make, list)" ;;
esac

# ---- check -------------------------------------------------------------------------------------------------------
if [ "$CMD" = check ]; then
  TARGET=""; PASS=()
  while [ $# -gt 0 ]; do
    case "$1" in
      -h|--help) astra_usage "$0"; exit 0 ;;
      --strict|--quiet|-q|--json) PASS+=("$1") ;;
      --home) shift; [ -n "${1:-}" ] || die_usage "--home needs LAT,LON"; PASS+=(--home "$1") ;;
      -*) die_usage "unknown option \"$1\" for check" ;;
      *) [ -z "$TARGET" ] || die_usage "check takes ONE mission file, got \"$TARGET\" and \"$1\""; TARGET="$1" ;;
    esac
    shift
  done
  [ -n "$TARGET" ] || die_usage "check needs a mission FILE (or a world name)"
  resolve_mission "$TARGET"
  run_checker "$MISSION" "${PASS[@]}"
  exit $?
fi

# ---- list --------------------------------------------------------------------------------------------------------
if [ "$CMD" = list ]; then
  case "${1:-}" in -h|--help) astra_usage "$0"; exit 0 ;; '') ;; *) die_usage "list takes no arguments" ;; esac
  astra_load_env
  say "Mission files you can load (bash mission.sh load FILE, or by world name)"
  SHARE=""
  if [ "${ASTRA_WS_BUILT:-no}" = yes ]; then SHARE="$(ros2 pkg prefix sim_worlds 2>/dev/null)/share/sim_worlds/missions"; fi
  [ -d "$SHARE" ] || SHARE="$ASTRA_REPO/ros2_ws/src/simulation/sim_worlds/missions"
  info "the worlds' own missions ($SHARE):"
  for f in "$SHARE"/*.waypoints; do [ -f "$f" ] && info "    $f"; done
  info "  by world name:  runway -> runway_survey.waypoints,  suas_field_real -> suas_field_real_survey.waypoints"
  MINE="$ASTRA_SIM_HOME/missions"
  info "missions made with 'mission.sh make' or converted from .plan ($MINE):"
  found=0
  for f in "$MINE"/*.waypoints; do [ -f "$f" ] && { info "    $f"; found=1; }; done
  [ "$found" = 1 ] || info "    (none yet: bash mission.sh make runway --altitude 50)"
  exit 0
fi

# ---- make --------------------------------------------------------------------------------------------------------
if [ "$CMD" = make ]; then
  WORLD=""; OUT=""; PASS=()
  while [ $# -gt 0 ]; do
    case "$1" in
      -h|--help) astra_usage "$0"; exit 0 ;;
      --out) shift; [ -n "${1:-}" ] || die_usage "--out needs a file name"; OUT="$1" ;;
      --altitude|--speed|--area|--sidelap|--frontlap|--lead-in|--photo-interval)
        [ -n "${2:-}" ] || die_usage "$1 needs a value"; PASS+=("$1" "$2"); shift ;;
      -*) die_usage "unknown option \"$1\" for make" ;;
      *) [ -z "$WORLD" ] || die_usage "make takes ONE world, got \"$WORLD\" and \"$1\""; WORLD="$1" ;;
    esac
    shift
  done
  [ -n "$WORLD" ] || die_usage "make needs a WORLD (runway or suas_field_real)"
  astra_load_env
  astra_require_built
  # A world is a name with a config in sim_worlds/config (or a config .yaml file): say so before Python does.
  if [ ! -f "$WORLD" ]; then
    WCFG="$(ros2 pkg prefix sim_worlds 2>/dev/null)/share/sim_worlds/config"
    KNOWN=""
    for f in "$WCFG"/*.yaml; do [ -f "$f" ] && KNOWN="$KNOWN $(basename "$f" .yaml)"; done
    [ -f "$WCFG/$WORLD.yaml" ] || die_file "there is no world called \"$WORLD\" (no $WCFG/$WORLD.yaml).
    Likely cause: a typo in the world name.
    Fix: use one of:${KNOWN:- (none found: run setup/03_build.sh)}"
  fi
  [ -n "$OUT" ] || OUT="$ASTRA_SIM_HOME/missions/${WORLD}_survey_$(date +%Y%m%d-%H%M%S).waypoints"
  mkdir -p "$(dirname "$OUT")" || die "cannot create the folder $(dirname "$OUT").
    Likely cause: no write permission there. Fix: give another place with --out FILE"
  say "Planning a survey mission for the world \"$WORLD\""
  ros2 run sim_worlds make_mission.py "$WORLD" "${PASS[@]}" --out "$OUT" || die "making the mission failed (the reason is above).
    Likely cause: an unknown world or area name, or a setting out of range.
    Fix: worlds are runway and suas_field_real; see every setting with  ros2 run sim_worlds make_mission.py --help"
  echo
  run_checker "$OUT" --quiet || warn "the new mission has problems (above): change the settings and make it again"
  echo
  ok "Made $OUT"
  info "load it:            bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" load \"$OUT\""
  info "a full demo with it: bash \"$ASTRA_SCRIPTS_DIR/demo.sh\" $WORLD --mission \"$OUT\""
  if command -v wslpath >/dev/null 2>&1; then
    info "Mission Planner on Windows (FLIGHT PLAN > Load WP File): $(wslpath -w "$OUT" 2>/dev/null)"
  fi
  exit 0
fi

# ---- load / fly --------------------------------------------------------------------------------------------------
TARGET=""; FORCE=0; WATCH=0; URL=""
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) astra_usage "$0"; exit 0 ;;
    --force) FORCE=1 ;;
    --watch) [ "$CMD" = fly ] || die_usage "--watch is an option of fly"; WATCH=1 ;;
    --url) shift; [ -n "${1:-}" ] || die_usage "--url needs a MAVLink address, e.g. udpin:127.0.0.1:14550"; URL="$1" ;;
    -*) die_usage "unknown option \"$1\" for $CMD" ;;
    *) [ -z "$TARGET" ] || die_usage "$CMD takes ONE mission, got \"$TARGET\" and \"$1\""; TARGET="$1" ;;
  esac
  shift
done
URL_ARGS=()
[ -z "$URL" ] || URL_ARGS=(--url "$URL")

load_mission() {   # check + (convert) + upload + MAVROS re-read, then remember what was loaded
  resolve_mission "$TARGET"
  astra_load_env
  astra_require_built
  astra_require_sim_link "${URL:-udpin:127.0.0.1:$(astra_state_value gcs_port 14550)}" "${URL:+typed}"
  local home="" upload="$MISSION" rc
  [ -n "$URL" ] || home="$(sim_home)"       # (with --url the autopilot may be anywhere: its home is not known here)
  local args=("$MISSION" --quiet)
  if [ -n "$home" ]; then
    args+=(--home "$home")
  else
    info "(the simulator's home is not known on this PC, so the check cannot tell whether the mission is for the" \
         "field the drone is on: look at the map in Mission Planner)"
  fi
  case "$MISSION" in
    *.plan|*.PLAN)
      mkdir -p "$ASTRA_SIM_HOME/missions"
      upload="$ASTRA_SIM_HOME/missions/$(basename "${MISSION%.*}").waypoints"
      args+=(--write-waypoints "$upload") ;;
  esac
  say "Checking the mission before the upload: $MISSION"
  run_checker "${args[@]}"
  rc=$?
  case "$rc" in
    0) ;;
    1) if [ "$FORCE" = 1 ]; then
         warn "--force: uploading although the check found errors (above). Consequence: the team code will probably" \
              "not take photos, hold or drop correctly with this mission."
       else
         die "the mission check found errors (listed above), so nothing was uploaded.
    Likely cause: the mission is missing something the team's flight code needs (see each ERROR line).
    Fix: correct it in Mission Planner and save it again (FLIGHT PLAN > Save WP File), or upload it anyway with --force"
       fi ;;
    3) exit 3 ;;
    *) die "the mission checker stopped with exit code $rc (see above).
    Likely cause: a problem in the checker itself. Fix: run  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" check \"$MISSION\"  and report it" ;;
  esac
  bash "$ASTRA_SCRIPTS_DIR/upload_mission.sh" "$upload" "${URL_ARGS[@]}" || exit $?
  printf 'SESSION=%s\nMISSION=%s\nUPLOADED=%s\nLOADED_AT=%s\n' "$(sim_session)" "$MISSION" "$upload" "$(date '+%F %T')" \
    > "$ASTRA_STATE_DIR/loaded_mission.env"
  ok "Mission loaded: $MISSION"
}

if [ "$CMD" = load ]; then
  [ -n "$TARGET" ] || die_usage "load needs a mission FILE or a WORLD name"
  load_mission
  info "next: fly it with  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" fly   (or in Mission Planner: Arm, then Auto)"
  exit 0
fi

# fly
astra_load_env
astra_require_built
if [ -n "$TARGET" ]; then
  load_mission
else
  # (first: is there a simulator at all? Then the warnings below are about the right thing.)
  astra_require_sim_link "${URL:-udpin:127.0.0.1:$(astra_state_value gcs_port 14550)}" "${URL:+typed}"
  loaded_session="$(sed -n 's/^SESSION=//p' "$ASTRA_STATE_DIR/loaded_mission.env" 2>/dev/null)"
  if [ -n "$loaded_session" ] && [ "$loaded_session" = "$(sim_session)" ]; then
    info "flying the mission loaded at $(sed -n 's/^LOADED_AT=//p' "$ASTRA_STATE_DIR/loaded_mission.env"):" \
         "$(sed -n 's/^MISSION=//p' "$ASTRA_STATE_DIR/loaded_mission.env")"
  else
    warn "no mission was loaded with mission.sh in this simulator session. Consequence: the autopilot flies whatever" \
         "mission it has stored (Mission Planner's, the demo's, or one from an earlier run)."
    warn_more "Fix if that is not what you want: load yours first with  bash \"$ASTRA_SCRIPTS_DIR/mission.sh\" fly FILE"
    if [ -t 0 ]; then         # a person is watching: give them a moment to stop it
      info "starting in 5 s (Ctrl+C stops here, before anything is armed)"
      sleep 5
    fi
  fi
fi
# Is the team's flight stack (MAVROS) up? Look for the process on this PC (any user: a Docker container's count too),
# then in the ROS graph. When it runs ELSEWHERE (the Jetson, another PC, another ROS_DOMAIN_ID: demo.sh recorded
# TEAM_STACK_WHERE=external for this session, or a --url points away) this PC cannot see it: only a hint then.
stack_env="$ASTRA_STATE_DIR/team_stack.env"
stack_where=""
if [ "$(sed -n 's/^SESSION=//p' "$stack_env" 2>/dev/null)" = "$(sim_session)" ]; then
  stack_where="$(sed -n 's/^TEAM_STACK_WHERE=//p' "$stack_env" 2>/dev/null)"
fi
if pgrep -x mavros_node >/dev/null 2>&1 || timeout 10 ros2 node list 2>/dev/null | grep -q '^/mavros'; then
  :
elif [ "$stack_where" = external ] || [ -n "$URL" ]; then
  info "(the team's MAVROS is not on this PC: it must run where the team stack runs, e.g. the Jetson or a container;" \
       "without it there are no photos and no drops)"
else
  warn "the team's flight stack (./launcher.sh sim, with MAVROS) was not found: no mavros_node process on this PC and" \
       "no /mavros node in ROS domain ${ROS_DOMAIN_ID:-0}. Consequence: the drone flies the lanes and comes home, but" \
       "takes no photos and drops nothing."
  warn_more "Fix: start it in another terminal (README 'Start pieces by hand'), or use  bash \"$ASTRA_SCRIPTS_DIR/astra.sh\" manual"
  warn_more "(If the team stack runs on the Jetson or in a container with another ROS_DOMAIN_ID, this PC cannot see it: ignore this.)"
fi
FLY_ARGS=(--no-upload "${URL_ARGS[@]}")
[ "$WATCH" = 0 ] || FLY_ARGS+=(--watch)
exec bash "$ASTRA_SCRIPTS_DIR/fly_mission.sh" "${FLY_ARGS[@]}"
