#!/usr/bin/env bash
# =============================================================================
#  simulation/jetson/pc/collect.sh  --  bring the Jetson's photos, detections and logs into the PC's run folder
#
#  PURPOSE: during a Jetson-in-the-loop flight the team code saves its photos and logs ON the Jetson (in our own
#  folder ~/astra_hil, never the team's). The PC's score report, phase view and run summary look for them on the PC.
#  This script copies them over, so every existing PC tool works unchanged (docs/jetson/HIL_DESIGN.md 7 items 4-5):
#    clock start|end   measure how far the Jetson's clock is from the PC's (7 samples over SSH; the one with the
#                      shortest round trip counts) into RUN_DIR/jetson/clock.txt. Photo file times are shifted by it,
#                      because score_run matches photos to the drone's position by their file time.
#    --follow          (during the flight) every few seconds: copy new photos + their .json sidecars from the Jetson
#                      into a staging folder, shift each file's time to the PC clock, then move the sidecar and THEN
#                      the photo into the PC's photo folder (a rename: the photo watcher never sees a half file).
#                      In login mode the copy is speed-limited (rsync --bwlimit) so it never crowds out the autopilot.
#    --final           (after the landing) the whole photo folder -> RUN_DIR/photos (times corrected the same way),
#                      the detections -> RUN_DIR/detections, the container's launcher output -> RUN_DIR/team_stack.log
#                      (same name as a PC run), and the Jetson's own files (relay, tap, ROS logs, tegrastats, facts,
#                      power mode) -> RUN_DIR/jetson/.
#
#  HOW TO RUN (demo.sh --jetson does all of this itself):
#    bash simulation/jetson/pc/collect.sh clock start [TARGET] --run-dir DIR
#    bash simulation/jetson/pc/collect.sh --follow    [TARGET] --run-dir DIR --photo-dir PHOTO_DIR [--interval 3]
#    bash simulation/jetson/pc/collect.sh --final     [TARGET] --run-dir DIR --run-id ID [--photo-dir PHOTO_DIR]
#  TARGET = --link usb|ssh|virtual --host HOST --user USER --port N --auth login|key (as for every Jetson command).
#  --photo-dir with --final: also put the photos into the PC's photo folder (for --jetson-photos after).
#  Exit codes: 0 OK, 1 a copy failed (the message says which), 2 typed wrong, 17 not logged in.
# =============================================================================
set -o pipefail
# shellcheck disable=SC2034  # (read by _common.sh: messages say "[collect] ...")
ASTRA_PROG=collect
PC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
J_DIR="$(cd "$PC_DIR/.." && pwd)"
# shellcheck source=../lib/target.sh
source "${JETSON_TARGET_SH:-$J_DIR/lib/target.sh}"
PY="${JETSON_PY:-python3}"

MODE=""; WHICH=""
case "${1:-}" in
  -h|--help|"") astra_usage "$0"; exit 0 ;;
  clock) MODE=clock; WHICH="${2:-}"; if [ $# -ge 2 ]; then shift 2; else shift; fi ;;
  archive) MODE=archive; shift ;;
  --follow|follow) MODE=follow; shift ;;
  --final|final) MODE=final; shift ;;
  *) jetson_die 2 "unknown command \"$1\" (clock start|end, --follow, --final). Fix: bash \"$PC_DIR/collect.sh\" --help" ;;
esac
jetson_target_load "$@"
RUN_DIR=""; PHOTO_DIR=""; RUN_ID=""; INTERVAL=3; STAMP="$(date +%Y%m%d-%H%M%S)"
set -- "${JETSON_ARGS[@]}"
while [ $# -gt 0 ]; do
  case "$1" in
    --run-dir) RUN_DIR="${2:-}"; shift ;;
    --stamp) STAMP="${2:-}"; shift ;;
    --photo-dir) PHOTO_DIR="${2:-}"; shift ;;
    --run-id) RUN_ID="${2:-}"; shift ;;
    --interval) INTERVAL="${2:-3}"; shift ;;
    *) jetson_die 2 "unknown option \"$1\". Fix: bash \"$PC_DIR/collect.sh\" --help" ;;
  esac
  shift
done
[ -n "$RUN_DIR" ] || jetson_die 2 "--run-dir DIR is needed (the run folder, e.g. ~/.astra_sim/runs/<stamp>_runway)"
JD="$RUN_DIR/jetson"; mkdir -p "$JD"
CLOCK="$JD/clock.txt"
LOG="$JD/collect.log"
log() { printf '%s %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" >> "$LOG"; }

need_link() {
  [ "$JETSON_LINK" = virtual ] && return 0
  jetson_logged_in || { jetson_need_login; exit 17; }
  jetson_resolve_data || jetson_die 1 "could not ask the Jetson for its data folder. Likely cause: the connection" \
    "dropped. Fix: bash \"$J_DIR/jetson.sh\" check"
}

# ---- clock ----------------------------------------------------------------------------------------------------------
do_clock() {
  case "$WHICH" in start|end) ;; *) jetson_die 2 "collect.sh clock needs start or end, e.g. collect.sh clock start --run-dir DIR" ;; esac
  local samples="$JD/.clock_samples_$WHICH" t0 tj t1 out
  if [ "$JETSON_LINK" = virtual ]; then
    out=$("$PY" "$PC_DIR/hil_pc.py" clock --clock-file "$CLOCK" --which "$WHICH" --line "offset_s=0 rtt_ms=0") || return 1
    log "clock $WHICH: virtual Jetson = this PC's clock ($out)"; info "clock: virtual Jetson uses this PC's clock"; return 0
  fi
  need_link
  : > "$samples"
  for _ in 1 2 3 4 5 6 7; do
    t0=$(date +%s.%N); tj=$(jetson_ssh 'date +%s.%N' 2>/dev/null); t1=$(date +%s.%N)
    [ -n "$tj" ] && printf '%s %s %s\n' "$t0" "$tj" "$t1" >> "$samples"
  done
  if out=$("$PY" "$PC_DIR/hil_pc.py" clock --clock-file "$CLOCK" --which "$WHICH" --samples "$samples"); then
    log "clock $WHICH: $out"; info "clock $WHICH: $out (Jetson minus PC; photo times are corrected by it)"
  else
    warn "the Jetson's clock offset could not be measured ($WHICH). Consequence: photo file times are not corrected," \
         "so the score report's per-photo footprints may be weaker. Likely cause: the link dropped. Fix: bash \"$J_DIR/jetson.sh\" check"
    return 1
  fi
}

# ---- follow (live photo mirror) -------------------------------------------------------------------------------------
do_follow() {
  [ -n "$PHOTO_DIR" ] || jetson_die 2 "--follow needs --photo-dir (the PC's photo folder: python3 -m sim_bringup.sim_paths photo-dir)"
  need_link
  local staging="$JD/.mirror" state="$JD/.mirror_state.json" bw n fails=0
  mkdir -p "$staging" "$PHOTO_DIR"
  bw=$("$PY" "$PC_DIR/hil_pc.py" bwlimit --auth "$JETSON_AUTH" --speed-file "$JETSON_STATE/speed.env" 2>/dev/null || echo 0)
  local limit=(); [ "${bw:-0}" -gt 0 ] && limit=(--bwlimit="$bw")
  log "follow: $JETSON_DATA/photos/ -> $PHOTO_DIR every ${INTERVAL}s (bwlimit ${bw:-0} KB/s, 0 = none)"
  trap 'log "follow stopped"; exit 0' TERM INT
  while true; do
    if jetson_rsync -rt --timeout=20 "${limit[@]}" ":$JETSON_DATA/photos/" "$staging/" >> "$LOG" 2>&1; then
      fails=0
      n=$("$PY" "$PC_DIR/hil_pc.py" publish --staging "$staging" --dest "$PHOTO_DIR" --clock-file "$CLOCK" --state "$state" 2>>"$LOG")
      [ "${n:-0}" = 0 ] || log "follow: $n file(s) published into $PHOTO_DIR"
    else
      fails=$((fails + 1))
      # (the link may be down for a while: say so once per minute, keep trying)
      [ $((fails % 20)) = 1 ] && log "follow: the copy failed ($fails in a row; the link may be down): retrying"
    fi
    sleep "$INTERVAL"
  done
}

# ---- final copy -------------------------------------------------------------------------------------------------------
do_final() {
  [ -n "$RUN_ID" ] || jetson_die 2 "--final needs --run-id ID (the container's run folder name, normally <stamp>_<world>)"
  need_link
  local rc=0 staging="$JD/.final_photos" n
  mkdir -p "$staging" "$RUN_DIR/photos" "$RUN_DIR/detections"
  log "final: from $JETSON_LINK $JETSON_DEST:$JETSON_DATA (run $RUN_ID)"
  if jetson_rsync -rt --timeout=60 ":$JETSON_DATA/photos/" "$staging/" >> "$LOG" 2>&1; then
    n=$("$PY" "$PC_DIR/hil_pc.py" publish --staging "$staging" --dest "$RUN_DIR/photos" --clock-file "$CLOCK" \
          --state "$JD/.final_state.json" 2>>"$LOG")
    info "photos: $(find "$RUN_DIR/photos" -maxdepth 1 -type f -name '*.jp*g' | wc -l) in $RUN_DIR/photos (file times on the PC clock)"
    if [ -n "$PHOTO_DIR" ]; then
      "$PY" "$PC_DIR/hil_pc.py" publish --staging "$staging" --dest "$PHOTO_DIR" --clock-file "$CLOCK" \
          --state "$JD/.mirror_state.json" >/dev/null 2>>"$LOG"
    fi
    # the two staging copies (this one and the live mirror's) are only download buffers: the photos are now in
    # RUN_DIR/photos, so free the space (about 100 MB each for a runway flight, seen in the first virtual flight)
    rm -rf "$staging" "$JD/.mirror"
    log "final: staging folders removed (the photos are in $RUN_DIR/photos)"
  else
    warn "the Jetson's photos could not be copied (details: $LOG). Consequence: the run folder has no photos from the" \
         "Jetson. Fix: when the link is back: bash \"$PC_DIR/collect.sh\" --final --run-dir \"$RUN_DIR\" --run-id $RUN_ID"
    rc=1
  fi
  jetson_rsync -rt --timeout=60 ":$JETSON_DATA/detections/" "$RUN_DIR/detections/" >> "$LOG" 2>&1 \
    || { warn "the Jetson's detection files could not be copied (details: $LOG). Fix: run --final again later"; rc=1; }
  if jetson_rsync -rt --timeout=120 ":$JETSON_DATA/runs/$RUN_ID/" "$JD/" >> "$LOG" 2>&1; then
    if [ -f "$JD/team_stack.log" ]; then
      if [ -f "$RUN_DIR/team_stack.log" ]; then cat "$JD/team_stack.log" >> "$RUN_DIR/team_stack.log"; rm -f "$JD/team_stack.log"
      else mv -f "$JD/team_stack.log" "$RUN_DIR/team_stack.log"; fi
      info "the team stack's log (launcher.sh in the container): $RUN_DIR/team_stack.log"
    fi
  else
    warn "the Jetson's run files ($JETSON_DATA/runs/$RUN_ID) could not be copied (details: $LOG). Consequence: no" \
         "team_stack.log, relay or tap logs in the run folder. Fix: run --final again when the link is back"
    rc=1
  fi
  if [ -f "$J_DIR/jetson.sh" ]; then
    bash "$J_DIR/jetson.sh" logs --run "$RUN_ID" --link "$JETSON_LINK" --host "$JETSON_HOST" --port "$JETSON_PORT" \
         --auth "$JETSON_AUTH" ${JETSON_USER:+--user "$JETSON_USER"} > "$JD/container.log" 2>&1 || true
  fi
  log "final: done (rc $rc)"
  return "$rc"
}

# ---- archive: the Jetson's photos of an earlier run -> its photo archive (they would be processed again) -------------
do_archive() {
  need_link
  local d="$JETSON_DATA" out
  # shellcheck disable=SC2016  # (the $(...) runs on the Jetson)
  out=$(jetson_ssh "mkdir -p '$d/photos' '$d/detections'; if [ -n \"\$(ls -A '$d/photos' '$d/detections' 2>/dev/null)\" ]; then \
    a='$d/photo_archive/$STAMP'; mkdir -p \"\$a/photos\" \"\$a/detections\"; \
    find '$d/photos' -mindepth 1 -maxdepth 1 -exec mv -t \"\$a/photos/\" {} +; \
    find '$d/detections' -mindepth 1 -maxdepth 1 -exec mv -t \"\$a/detections/\" {} +; echo \"\$a\"; fi" 2>&1) \
    || { warn "the Jetson's old photos could not be archived ($out). Consequence: photos of an earlier run may be" \
              "processed again. Fix: bash \"$J_DIR/jetson.sh\" shell, then move $d/photos/* away"; return 1; }
  [ -z "$out" ] || { info "photos of an earlier run on the Jetson moved to $out"; log "archive: $out"; }
  return 0
}

case "$MODE" in
  archive) do_archive ;;
  clock) do_clock ;;
  follow) do_follow ;;
  final) do_final ;;
esac
