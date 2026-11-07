# shellcheck shell=bash
# =============================================================================
#  simulation/jetson/pc/hil_steps.sh  --  the "--jetson" steps of demo.sh (sourced by demo.sh; never run by itself)
#
#  PURPOSE: with  demo.sh ... --jetson usb | ssh | virtual  the team's flight software does NOT start on this PC: it
#  runs in the container astra-hil on the real Jetson Orin Nano (over the USB-C cable or any SSH address), or in the
#  same container on this PC (the "virtual Jetson"). Everything else of the demo stays exactly as it is (Gazebo,
#  SITL, the camera emulator, Mission Planner, the phases, the score report). These functions are the few steps that
#  differ (docs/jetson/HIL_DESIGN.md 9.3), kept here so the change in demo.sh stays small:
#    hil_options      after demo.sh read its options: check them, pick the jetson_<link> plug-in profile
#    hil_preflight    before the simulator starts: jetson.sh check --quick --for flight (logged in, image, engine,
#                     team not busy, disk, memory ...); any failure stops the demo with its fix
#    hil_start        STEP 3: archive the Jetson's old photos, measure the clock, open the link (link.sh), start the
#                     container (jetson.sh start --sim), read the software versions inside it (container_versions.py
#                     -> RUN_DIR/jetson/versions.env), the ROS tap, the live photo mirror, find which camera pipeline
#                     runs (the team nodes the tap reports), then wait until the team stack is ready (stack_check
#                     runs INSIDE the container; one restart if it fails, like today)
#    hil_look_down    the operator's "camera straight down" command, typed inside the container
#    hil_tick         called every 2 s while the drone flies: says when link.sh saw the link drop / come back
#                     (hil_link_news), and if the tap has been silent for 30 s (the link is down) it also watches the
#                     landing over MAVLink, so a long drop cannot hang the demo
#    hil_finish       at the end: stop the container, copy photos/detections/logs into the run folder (clock
#                     corrected), close the link (never the owner's login window), write RUN_DIR/jetson/summary.md
#  Variables demo.sh sets before calling (see its --jetson options): JETSON (usb|ssh|virtual), JETSON_HOST_OPT,
#  JETSON_USER_OPT, JETSON_PORT_OPT, JETSON_AUTH_OPT, JETSON_PHOTOS (live|after), JETSON_TAP (on|off),
#  JETSON_BAUD (115200|off), JETSON_PACE (auto|off|MBIT), JETSON_PIPELINE (auto|new|legacy), JETSON_LIKE (0|1),
#  and its own RUN_DIR, STAMP,
#  WORLD_NAME, INSTANCE, PHOTO_DIR, CAMERA, DETECTOR, GCS_PORT, NEED, PLUGIN_STACK_ARGS.
# =============================================================================

HIL_PC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HIL_J_DIR="$(cd "$HIL_PC_DIR/.." && pwd)"
HIL_JETSON_SH="${HIL_JETSON_SH:-$HIL_J_DIR/jetson.sh}"
HIL_PY="${JETSON_PY:-python3}"
HIL_T=()                       # the target options every Jetson command gets
HIL_STARTED=0; HIL_LINK_UP=0; HIL_TAP_PID=""; HIL_COLLECT_PID=""; HIL_WATCH_PID=""; HIL_RUN_ID=""; HIL_PIPE_USED=""
JETSON_LIKE="${JETSON_LIKE:-0}"
HIL_SILENT_NOTE=0
HIL_PARAM_KEEP="SIM_BAUDLIMIT_EN,SERIAL0_BAUD,SERIAL2_BAUD"

# hil_options : check the --jetson options and turn them into plug-in profile settings (called right after demo.sh's
# option loop). Sets PROFILE (jetson_<link>, unless --profile was given), PLUGIN=1 and the TYPED settings.
hil_options() {
  case "$JETSON" in usb|ssh|virtual) ;; *) die_usage "--jetson must be usb, ssh or virtual, not \"$JETSON\"" ;; esac
  [ "$JETSON" != ssh ] || [ -n "$JETSON_HOST_OPT" ] || [ -n "${JETSON_HOST:-}" ] || [ -s "${ASTRA_SIM_HOME:-$HOME/.astra_sim}/jetson/target.env" ] \
    || die_usage "--jetson ssh needs the Jetson's address: add --jetson-host HOST (an IP such as 192.168.144.20, or a name such as jetson.local)"
  case "$JETSON_PHOTOS" in live|after) ;; *) die_usage "--jetson-photos must be live or after, not \"$JETSON_PHOTOS\"" ;; esac
  case "$JETSON_TAP" in on|off) ;; *) die_usage "--jetson-tap must be on or off, not \"$JETSON_TAP\"" ;; esac
  case "$JETSON_BAUD" in 115200|off) ;; *) die_usage "--jetson-baud must be 115200 or off, not \"$JETSON_BAUD\"" ;; esac
  case "$JETSON_PIPELINE" in auto|legacy|new) ;; *) die_usage "--jetson-pipeline must be auto, new or legacy, not \"$JETSON_PIPELINE\"" ;; esac
  case "$JETSON_PACE" in auto|off) ;; ''|*[!0-9.]*) die_usage "--photo-pace must be auto, off or a number of Mbit/s, not \"$JETSON_PACE\"" ;; esac
  case "$DETECTOR" in truth|custom) die_usage "--detector $DETECTOR cannot be used with --jetson: that detector runs on this PC, and" \
    "main_controller on the Jetson could not hear it (the tap only copies Jetson -> PC). Use the team's detector (the default)" ;; esac
  HIL_T=(--link "$JETSON")
  [ -z "$JETSON_HOST_OPT" ] || HIL_T+=(--host "$JETSON_HOST_OPT")
  [ -z "$JETSON_USER_OPT" ] || HIL_T+=(--user "$JETSON_USER_OPT")
  [ -z "$JETSON_PORT_OPT" ] || HIL_T+=(--port "$JETSON_PORT_OPT")
  [ -z "$JETSON_AUTH_OPT" ] || HIL_T+=(--auth "$JETSON_AUTH_OPT")
  # the simulator side of a Jetson flight is a plug-in profile (simulation/profiles/jetson_<link>.yaml):
  # run.team_stack external (demo.sh never starts ./launcher.sh here: SITL SERIAL2 serves ONE client, the Jetson)
  [ -n "$PROFILE" ] || PROFILE="jetson_$JETSON"
  TYPED+=("run.team_stack=external")
  if [ "$JETSON_BAUD" = 115200 ]; then
    # like the Pixhawk's TELEM2 at 115200 baud (SITL SIM_BAUDLIMIT_EN; the safe parameter filter drops SIM_* and
    # SERIAL* lines, so they are kept by name)
    PARAM_FILES="${PARAM_FILES:+$PARAM_FILES,}$HIL_PC_DIR/hil_serial2_115200.parm"
    TYPED+=("autopilot.param_keep=$HIL_PARAM_KEEP")
  fi
  # shellcheck disable=SC2034  # (demo.sh reads PLUGIN: the profile is used)
  PLUGIN=1
}

# hil_jetson ARGS... : run jetson.sh with the target options; its exit code comes back.
hil_jetson() {
  local sub="$1"; shift
  bash "$HIL_JETSON_SH" "$sub" "${HIL_T[@]}" "$@"
}

# hil_dry_run_lines : the plan lines for demo.sh --dry-run
hil_dry_run_lines() {
  echo "    (Jetson in the loop, --jetson $JETSON${JETSON_HOST_OPT:+ --jetson-host $JETSON_HOST_OPT}: the team stack runs in the container on the $( [ "$JETSON" = virtual ] && echo 'virtual Jetson on this PC' || echo Jetson ))"
  echo "    0. bash $HIL_JETSON_SH check --quick --for flight ${HIL_T[*]}"
  echo "    4. instead of ./launcher.sh here: collect.sh clock start; link.sh start ${HIL_T[*]} --instance $INSTANCE;"
  echo "       jetson.sh start --sim --run-id <stamp>_$WORLD_NAME; ros_tap.py receive$( [ "$JETSON_TAP" = on ] || echo ' (off)' );"
  echo "       collect.sh --follow ($JETSON_PHOTOS); jetson.sh exec -- ros2 run sim_bringup stack_check ..."
  echo "    9. at the end: jetson.sh stop; collect.sh --final; link.sh stop; RUN_DIR/jetson/summary.md"
}

# hil_preflight : before anything starts. Stops demo.sh (die) with the check's own message when the board is not ready.
hil_preflight() {
  if [ ! -f "$HIL_JETSON_SH" ]; then
    die "the Jetson kit ($HIL_JETSON_SH) is missing, so --jetson cannot run.
    Likely cause: this checkout is older than the Jetson kit, or the Linux copy was not mirrored yet.
    Fix: bash \"$ASTRA_SETUP_DIR/03_build.sh\"  (it mirrors the repo), then run demo.sh again"
  fi
  say "Checking the Jetson before anything starts (jetson.sh check --quick --for flight)"
  local rc pipe=()
  [ "$JETSON_PIPELINE" = auto ] || pipe=(--pipeline "$JETSON_PIPELINE")
  hil_jetson check --quick --for flight "${pipe[@]}" 2>&1 | sed 's/^/    /'
  rc=${PIPESTATUS[0]}
  if [ "$rc" != 0 ]; then
    die "the Jetson is not ready for a flight (jetson.sh check ended with code $rc; the lines above say which step and how to fix it); nothing was started.
    Likely cause: not logged in (code 17: bash \"$HIL_JETSON_SH\" login), the image is not deployed (12), the engine is missing (19),
    or the team is using the board (16).
    Fix: do what the failed line above says, then run demo.sh again (docs/jetson/LINK.md has the link troubleshooting)"
  fi
  ok "the Jetson is ready ($JETSON)"
}

# hil_virtual_data : the virtual Jetson's data folder (the same rule as lib/target.sh)
hil_virtual_data() { printf '%s\n' "${JETSON_DATA_DIR:-${ASTRA_SIM_HOME:-$HOME/.astra_sim}/jetson_virtual}"; }

# hil_resolve_pipeline : which camera pipeline the container really runs (HIL_PIPE_USED). With --jetson-pipeline auto
# the container's entrypoint decides (new when the image has the proposed field launch file and zr10_ros2, else
# legacy). We look at what really runs, in this order:
#   1. the team node names the ROS tap reports (NODES= in $ASTRA_STATE_DIR/jetson_tap.env): zr10_camera /
#      zr10_stream = the new video pipeline, siyi / new_od / mission_camera_trigger = the legacy photo pipeline;
#   2. the entrypoint's own line "starting ./launcher.sh hw (... pipeline=new|legacy)" in the container output
#      (jetson.sh logs shows the LAST 200 lines, so this only works in the first second or two after the start);
#   3. the same text in team_stack.log, through jetson.sh exec (/data/runs is mounted).
# The first virtual flight (2026-10-31) showed that 3 alone never matches (the entrypoint line is not in
# team_stack.log) and cost 2.5 minutes of waiting, hence 1 and 2.
hil_pipeline_from_nodes() {   # prints new / legacy / nothing, from the tap's NODES= line
  local n
  n=",$(awk -F= '$1 == "NODES" { sub(/^[^=]*=/, ""); print }' "$ASTRA_STATE_DIR/jetson_tap.env" 2>/dev/null),"
  case "$n" in
    *,zr10_camera,*|*,zr10_stream,*) echo new ;;
    *,siyi,*|*,new_od,*|*,mission_camera_trigger,*) echo legacy ;;
  esac
}
hil_resolve_pipeline() {
  HIL_PIPE_USED="$JETSON_PIPELINE"
  [ "$JETSON_PIPELINE" = auto ] || return 0
  local p="" try
  for try in $(seq 1 45); do
    [ "$JETSON_TAP" != on ] || p=$(hil_pipeline_from_nodes)
    [ -n "$p" ] || [ "$try" -gt 3 ] || p=$(hil_jetson logs 2>/dev/null | grep -ao 'pipeline=[a-z]*' | tail -1)
    if [ -z "$p" ] && [ $((try % 10)) = 0 ]; then
      p=$(hil_jetson exec -- grep -ao 'pipeline=[a-z]*' "/data/runs/$HIL_RUN_ID/team_stack.log" 2>/dev/null | tail -1)
    fi
    p="${p#pipeline=}"
    case "$p" in new|legacy) break ;; esac
    sleep 2
  done
  case "$p" in
    new|legacy) HIL_PIPE_USED="$p" ;;
    *) HIL_PIPE_USED=legacy
       warn "could not tell which camera pipeline the container runs (no team camera node reported by the tap and no" \
            "'pipeline=' line found within about 90 s). Consequence: the readiness check assumes the legacy photo" \
            "pipeline. Fix: give it with --jetson-pipeline new|legacy" ;;
  esac
  info "camera pipeline in the container: $HIL_PIPE_USED"
}

# hil_stack_check_args : what stack_check inside the container waits for (depends on the camera pipeline)
hil_stack_check_args() {
  HIL_SC=(--timeout 300 --settle 20)
  if [ "${HIL_PIPE_USED:-$JETSON_PIPELINE}" = new ]; then
    # the new video pipeline: no photo nodes (siyi, trigger, new_od), the ZR10 controller and the stream node instead
    HIL_SC+=(--skip-node siyi --skip-node mission_camera_trigger --skip-node new_od --add-node zr10_camera --add-node zr10_stream)
  else
    HIL_SC+=("${NEED[@]}")
  fi
  HIL_SC+=("${PLUGIN_STACK_ARGS[@]}")
}

# hil_start : STEP 3 of demo.sh with --jetson. Returns 0 when the team stack on the Jetson is ready.
hil_start() {
  local jd="$RUN_DIR/jetson" pipe_opt=() rc attempt
  HIL_RUN_ID="${STAMP}_${WORLD_NAME}"
  mkdir -p "$jd"
  say "STEP 3/6  The team's flight stack on the Jetson ($JETSON): link, container, tap"
  info "log of the team stack: $RUN_DIR/team_stack.log (copied from the Jetson at the end); Jetson files: $jd/"
  # 1. the Jetson's photos of an earlier run go to its photo archive (they would be processed again)
  bash "$HIL_PC_DIR/collect.sh" archive "${HIL_T[@]}" --run-dir "$RUN_DIR" --stamp "$STAMP" 2>&1 | sed 's/^/    /'
  # 2. how far the Jetson's clock is from this PC's (photo times are corrected by it)
  bash "$HIL_PC_DIR/collect.sh" clock start "${HIL_T[@]}" --run-dir "$RUN_DIR" 2>&1 | sed 's/^/    /'
  # 3. the link: autopilot (SITL SERIAL2), camera, tap
  bash "$HIL_PC_DIR/link.sh" start "${HIL_T[@]}" --run-dir "$RUN_DIR" --instance "$INSTANCE" --camera "$CAMERA" \
       --photo-pace "$JETSON_PACE" 2>&1 | sed 's/^/    /'
  rc=${PIPESTATUS[0]}
  if [ "$rc" != 0 ]; then
    warn "the link to the Jetson could not be opened (the lines above say why). Fix: bash \"$HIL_PC_DIR/link.sh\" status" \
         "${HIL_T[*]}, then run demo.sh again (docs/jetson/LINK.md, troubleshooting)"
    return 14
  fi
  HIL_LINK_UP=1
  # 4. the container (SIM mode: the camera address only inside it, the relay, the tap sender, ./launcher.sh hw)
  pipe_opt=(--pipeline "$JETSON_PIPELINE")
  [ "$JETSON_LIKE" != 1 ] || pipe_opt+=(--jetson-like)     # (virtual: the optional Jetson-like CPU/memory limit)
  hil_jetson start --sim --run-id "$HIL_RUN_ID" "${pipe_opt[@]}" 2>&1 | sed 's/^/    /'
  rc=${PIPESTATUS[0]}
  if [ "$rc" != 0 ]; then
    warn "the team stack container did not start on the Jetson (jetson.sh start ended with code $rc; see above). Fix:" \
         "bash \"$HIL_JETSON_SH\" logs ${HIL_T[*]}, then run demo.sh again"
    return 13
  fi
  HIL_STARTED=1
  hil_jetson status --json > "$jd/status_start.json" 2>/dev/null || true
  # the software versions INSIDE the container (owner 12:32: "version for version" in every run summary); read-only,
  # sent on standard input so the image needs nothing extra; a failure only leaves the table empty
  if hil_jetson exec -- python3 - < "$HIL_PC_DIR/container_versions.py" > "$jd/versions.env" 2>> "$jd/versions.err"; then
    info "versions in the container: $(awk -F= '$1 ~ /^(OS|MAVROS|TORCH_CUDA|TENSORRT_PY|GPU)$/ { printf "%s=%s; ", $1, $2 }' "$jd/versions.env")"
  else
    warn "the container's software versions could not be read (see $jd/versions.err). Consequence: the run summary has no" \
         "version table. Fix: none needed for the flight; by hand: bash \"$HIL_JETSON_SH\" exec ${HIL_T[*]} -- python3 - < \"$HIL_PC_DIR/container_versions.py\""
  fi
  # 5. the ROS tap (Jetson -> PC, read-only) so phase_monitor and score_run work unchanged
  if [ "$JETSON_TAP" = on ]; then
    local ep="tcp:127.0.0.1:7461"
    [ "$JETSON" != virtual ] || ep="unix:$(hil_virtual_data)/sock/tap.sock"
    setsid "$HIL_PY" "$HIL_PC_DIR/ros_tap.py" receive --connect "$ep" --node-name jetson_tap --presence-namespace /jetson \
        --log "$jd/tap_receive.log" --status-file "$ASTRA_STATE_DIR/jetson_tap.env" > /dev/null 2>> "$jd/tap_receive.log" < /dev/null &
    HIL_TAP_PID=$!
    info "ROS tap receiver pid $HIL_TAP_PID ($ep -> node jetson_tap; presence nodes /jetson/<team node>)"
  else
    warn "--jetson-tap off: no topics are copied from the Jetson. Consequence: the phases in this terminal and the" \
         "score report's detections stay empty (Mission Planner and the CSV recorder still work over MAVLink)."
  fi
  # 6. the live photo mirror (photos + sidecars into the PC's photo folder, times corrected)
  if [ "$JETSON_PHOTOS" = live ]; then
    setsid bash "$HIL_PC_DIR/collect.sh" --follow "${HIL_T[@]}" --run-dir "$RUN_DIR" --photo-dir "$PHOTO_DIR" \
        > /dev/null 2>> "$jd/collect.log" < /dev/null &
    HIL_COLLECT_PID=$!
    info "live photo mirror pid $HIL_COLLECT_PID (Jetson photos -> $PHOTO_DIR every 3 s)"
  else
    info "--jetson-photos after: the photos are copied after the landing (no live per-photo table)"
  fi
  # 7. ready? stack_check INSIDE the container (it needs the team's node names and services); one restart if not
  hil_resolve_pipeline
  hil_stack_check_args
  for attempt in 1 2; do
    info "waiting for the team stack on the Jetson (stack_check inside the container, up to 5 minutes) ..."
    hil_jetson exec -- ros2 run sim_bringup stack_check "${HIL_SC[@]}" 2>&1 | sed 's/^/    /'
    rc=${PIPESTATUS[0]}
    [ "$rc" = 0 ] && break
    if [ "$attempt" = 1 ]; then
      warn "the team stack on the Jetson is not ready (stack_check code $rc): restarting the container once"
      hil_jetson restart --sim --run-id "$HIL_RUN_ID" "${pipe_opt[@]}" 2>&1 | sed 's/^/    /'
      continue
    fi
    warn "the team stack on the Jetson did not get ready, even after one restart. Likely cause: a team node crashes" \
         "at start, MAVROS cannot reach SITL through the link, or the detector model does not load. Fix: bash" \
         "\"$HIL_JETSON_SH\" logs ${HIL_T[*]} (the team stack log), and bash \"$HIL_PC_DIR/link.sh\" status ${HIL_T[*]}"
    return 15
  done
  ok "the team stack on the Jetson is ready ($JETSON)"
  return 0
}

# hil_stack_env : replace demo.sh's state/team_stack.env (what START OVER reads) with the Jetson's facts.
# START OVER is not supported with --jetson in this version (HIL_DESIGN.md 9.3): TEAM_STACK_WHERE=jetson says so.
hil_stack_env() {
  printf 'SESSION=%s\nTRACKER_CMD=%s\nFCU_URL=%s\nSTACK_CHECK_ARGS=%s\nTEAM_STACK_WHERE=jetson\nJETSON_LINK=%s\n' \
    "$ASTRA_SIM_SESSION" "(in the container on the Jetson: ./launcher.sh hw)" "tcp://127.0.0.1:5763 (in the container)" \
    "${HIL_SC[*]}" "$JETSON" > "$ASTRA_STATE_DIR/team_stack.env"
}

# hil_look_down : the operator's "camera straight down" (legacy photo pipeline: video_cam's /camera/command)
hil_look_down() {
  [ "${HIL_PIPE_USED:-$JETSON_PIPELINE}" = legacy ] || return 0
  timeout 30 bash "$HIL_JETSON_SH" exec "${HIL_T[@]}" -- ros2 topic pub -w 1 --once /camera/command std_msgs/msg/String \
    "{data: 'gimbal_set_angles 0,-90'}" >/dev/null 2>&1 || true
}

# hil_link_news : say in the demo terminal when link.sh's watcher saw the link drop or come back. The watcher itself
# writes only into RUN_DIR/jetson/link.log (a background process must never hold demo.sh's pipes), and keeps
# A=up|down, DROPS and DOWN_S in $ASTRA_STATE_DIR/jetson_link.env, which this reads.
HIL_LINK_DOWN_SAID=0
hil_link_news() {
  local f="$ASTRA_STATE_DIR/jetson_link.env" a
  [ "$HIL_LINK_UP" = 1 ] && [ -f "$f" ] || return 0
  a=$(awk -F= '$1 == "A" { print $2 }' "$f" 2>/dev/null)
  if [ "$a" = down ] && [ "$HIL_LINK_DOWN_SAID" = 0 ]; then
    HIL_LINK_DOWN_SAID=1
    warn "the link to the Jetson dropped (seen by link.sh; details: $RUN_DIR/jetson/link.log). Consequence: the team stack" \
         "hears no autopilot and no camera until it is back; the drone flies on (AUTO) or holds (GUIDED)."
    if [ "$(awk -F= '$1 == "AUTH" { print $2 }' "$f")" = key ]; then
      warn_more "It reconnects by itself (key mode). Check the cable / Wi-Fi if it does not come back."
    else
      warn_more "Fix: in the Jetson login window press Enter and type the password (or: bash \"$HIL_JETSON_SH\" login)."
    fi
  elif [ "$a" = up ] && [ "$HIL_LINK_DOWN_SAID" = 1 ]; then
    HIL_LINK_DOWN_SAID=0
    ok "the Jetson link is back (drop #$(awk -F= '$1 == "DROPS" { print $2 }' "$f"), $(awk -F= '$1 == "DOWN_S" { print $2 }' "$f") s down in total)"
  fi
}

# hil_tick : every 2 s while flying. Returns 10 when the landing was seen over MAVLink while the tap was silent.
hil_tick() {
  hil_link_news
  [ "$JETSON_TAP" = on ] || return 0
  local last now
  now=$(date +%s)
  last=$(awk -F= '$1 == "LAST_FRAME_UNIX" { print int($2) }' "$ASTRA_STATE_DIR/jetson_tap.env" 2>/dev/null)
  [ -n "$last" ] && [ "$last" != 0 ] || return 0
  if [ $((now - last)) -ge 30 ] && [ -z "$HIL_WATCH_PID" ]; then
    warn "the ROS tap from the Jetson has been silent for $((now - last)) s (the link is down?). Consequence: the phases" \
         "here pause. The landing is now also watched over MAVLink (mission_tool watch), so the demo cannot hang."
    ros2 run sim_bringup mission_tool --url "udpin:127.0.0.1:$GCS_PORT" watch > "$RUN_DIR/jetson/mavlink_watch.log" 2>&1 &
    HIL_WATCH_PID=$!
  fi
  if [ $((now - last)) -ge 60 ] && [ $((now - HIL_SILENT_NOTE)) -ge 60 ]; then
    HIL_SILENT_NOTE=$now
    warn "the Jetson link has been down for about $((now - last)) s. Fix: $( [ "${JETSON_AUTH_OPT:-login}" = key ] && \
      echo 'check the cable / Wi-Fi (key mode reconnects by itself)' || echo 'press Enter in the Jetson login window and type the password' )"
  fi
  if [ -n "$HIL_WATCH_PID" ] && ! kill -0 "$HIL_WATCH_PID" 2>/dev/null; then
    if wait "$HIL_WATCH_PID"; then
      ok "landed and disarmed (seen over MAVLink while the tap was silent)"
      return 10
    fi
    HIL_WATCH_PID=""
  fi
  return 0
}

# hil_finish : at the end (also after an early failure: only what was started is stopped)
hil_finish() {
  [ -n "$HIL_RUN_ID" ] || return 0
  local jd="$RUN_DIR/jetson" extra=()
  say "Jetson: stopping the team stack and collecting its files"
  [ -z "$HIL_WATCH_PID" ] || kill "$HIL_WATCH_PID" 2>/dev/null
  if [ "$HIL_STARTED" = 1 ]; then
    hil_jetson status --json > "$jd/status_end.json" 2>/dev/null || true
    hil_jetson stop 2>&1 | sed 's/^/    /'
  fi
  [ -z "$HIL_COLLECT_PID" ] || { kill "$HIL_COLLECT_PID" 2>/dev/null; wait "$HIL_COLLECT_PID" 2>/dev/null; }
  bash "$HIL_PC_DIR/collect.sh" clock end "${HIL_T[@]}" --run-dir "$RUN_DIR" 2>&1 | sed 's/^/    /'
  [ "$JETSON_PHOTOS" = after ] && extra=(--photo-dir "$PHOTO_DIR")
  if [ "$HIL_STARTED" = 1 ]; then
    bash "$HIL_PC_DIR/collect.sh" --final "${HIL_T[@]}" --run-dir "$RUN_DIR" --run-id "$HIL_RUN_ID" "${extra[@]}" 2>&1 | sed 's/^/    /'
  fi
  cp -f "$ASTRA_STATE_DIR/jetson_link.env" "$jd/link_state.env" 2>/dev/null
  cp -f "$ASTRA_STATE_DIR/jetson_tap.env" "$jd/tap_status.env" 2>/dev/null
  if [ "$HIL_LINK_UP" = 1 ]; then bash "$HIL_PC_DIR/link.sh" stop "${HIL_T[@]}" --run-dir "$RUN_DIR" 2>&1 | sed 's/^/    /'; fi
  [ -z "$HIL_TAP_PID" ] || kill "$HIL_TAP_PID" 2>/dev/null
  local connected=()
  if [ "${HIL_PIPE_USED:-$JETSON_PIPELINE}" = new ]; then
    connected+=("new video pipeline: zr10_ros2 + RTSP stream + yolo26 detector; how the detections reach main_controller is an open question for the team (no detection-driven drops unless their wiring exists)")
  else
    connected+=("legacy photo pipeline: video_cam (SIYI SDK + HTTP photos) -> new_od -> /image_detection -> main_controller (the team's own wiring)")
  fi
  "$HIL_PY" "$HIL_PC_DIR/hil_pc.py" summary --run-dir "$RUN_DIR" --link "$JETSON" --auth "$(awk -F= '$1=="AUTH"{print $2}' "$jd/link_state.env" 2>/dev/null)" \
      --host "$(awk -F= '$1=="HOST"{print $2}' "$jd/link_state.env" 2>/dev/null)" \
      --user "$(awk -F= '$1=="USER"{print $2}' "$jd/link_state.env" 2>/dev/null)" --photos "$JETSON_PHOTOS" \
      --pace "$(awk -F= '$1=="PACE_MBIT"{print $2}' "$jd/link_state.env" 2>/dev/null)" --baud "$JETSON_BAUD" --camera "$CAMERA" \
      --pipeline "${HIL_PIPE_USED:-$JETSON_PIPELINE}" "${connected[@]/#/--connected=}" > "$jd/summary.log" 2>&1 \
    && { say "JETSON RUN SUMMARY: $jd/summary.md"; sed -n '1,40p' "$jd/summary.md" | sed 's/^/    /'; } \
    || warn "the Jetson run summary could not be written (see $jd/summary.log). Fix: python3 $HIL_PC_DIR/hil_pc.py summary --run-dir \"$RUN_DIR\""
  HIL_RUN_ID=""
}
