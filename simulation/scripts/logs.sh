#!/usr/bin/env bash
# =============================================================================
#  logs.sh  --  LOOK AT THE LOGS AND DATA OF SIMULATOR RUNS, one command
#
#  PURPOSE: every demo.sh run leaves a run folder ~/.astra_sim/runs/<date>-<time>_<world>/ with its logs,
#  CSV files (csv/), the score report and a run summary (run_summary.md). This script finds them for you:
#
#    bash logs.sh list [N]                   the last N runs (default 20): result, flight time, targets found
#    bash logs.sh summary [RUN] [--refresh]  print the run summary (what ran, result, phases, every problem);
#                                            --refresh first writes it again from the files (same result)
#    bash logs.sh errors [RUN] [--all]       every warning / error line of every log, with file:line
#                                            (--all also shows the lines the docs call normal)
#    bash logs.sh tail LOG [RUN] [-n N] [-f] the end of one log: LOG = sim | team_stack | demo | recorder |
#                                            any file name in the run folder (-f keeps following it)
#    bash logs.sh csv [RUN]                  where the CSV files are, as a Linux AND a Windows path, with rows
#    bash logs.sh export [RUN ...]           (re)write the CSV files of a run (also older runs)
#    bash logs.sh bundle [RUN] [--photos] [--max-mb N] [--out FILE.zip]
#                                            one zip of the run to send to a teammate: logs, CSV, reports;
#                                            no photos (unless --photos), no file over 20 MB, nothing that
#                                            looks like a password or key (by its name or its content)
#    bash logs.sh path [RUN]                 just the run folder (for cd "$(bash logs.sh path)")
#
#  RUN is the newest run when left out; otherwise a folder name (20261031-101431_runway), part of one
#  (101431), a number (1 = newest, 2 = the one before ...) or a full path.
#  A run that is STILL GOING (its flight recorder runs, or its logs changed in the last minute) is only looked at:
#  summary shows it without writing anything, export refuses, bundle zips it as it is (with a note).
#  How do I know it worked: "list" prints a table of runs; "summary" prints a page starting "# Run summary".
#  Everything is explained in docs/simulation/LOGS_AND_DATA.md.
# =============================================================================
set -o pipefail
# shellcheck source-path=SCRIPTDIR source=_common.sh
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"

usage() { astra_usage "$0"; }

RUNS_DIR="${ASTRA_SIM_HOME:-$HOME/.astra_sim}/runs"
# The helpers are plain Python (no ROS needed): use the installed sim_demo, else the source folder next to us.
SRC_PKG="$(cd "$ASTRA_SCRIPTS_DIR/../../ros2_ws/src/simulation/sim_demo" 2>/dev/null && pwd)"
PY="python3"
[ -x /usr/bin/python3 ] && PY=/usr/bin/python3
helper() {   # helper ARGS... : python3 -m sim_demo.run_summary ARGS
  PYTHONPATH="${SRC_PKG:+$SRC_PKG:}${PYTHONPATH:-}" PYTHONDONTWRITEBYTECODE=1 "$PY" -m sim_demo.run_summary "$@"
}
exporter() {
  PYTHONPATH="${SRC_PKG:+$SRC_PKG:}${PYTHONPATH:-}" PYTHONDONTWRITEBYTECODE=1 "$PY" -m sim_demo.csv_export "$@"
}

# find_run [RUN] : print the run folder for RUN (see the header), or stop with a clear message.
find_run() {
  local want="${1:-}" found=""
  [ -d "$RUNS_DIR" ] || die "There is no runs folder $RUNS_DIR yet.
    Likely cause: no simulator run has finished on this computer (or ASTRA_SIM_HOME points elsewhere).
    Fix: run a demo first:  bash \"$ASTRA_SCRIPTS_DIR/demo.sh\""
  if [ -z "$want" ] || [ "$want" = latest ]; then want=1; fi
  if [ -d "$want" ]; then (cd "$want" && pwd); return 0; fi
  if [ -d "$RUNS_DIR/$want" ]; then printf '%s\n' "$RUNS_DIR/$want"; return 0; fi
  # a short number (1-999) counts back from the newest run; anything else is a part of a folder name (e.g. 101431)
  if [[ "$want" =~ ^[1-9][0-9]{0,2}$ ]]; then
    found="$(find "$RUNS_DIR" -mindepth 1 -maxdepth 1 -type d | sort -r | sed -n "${want}p")"
  else
    found="$(find "$RUNS_DIR" -mindepth 1 -maxdepth 1 -type d -name "*$want*" | sort -r | head -1)"
  fi
  [ -n "$found" ] || die "No run matches \"$1\" in $RUNS_DIR.
    Likely cause: a typo, or the run is older and was deleted.
    Fix: see the runs with:  bash \"$ASTRA_SCRIPTS_DIR/logs.sh\" list"
  printf '%s\n' "$found"
}

# still_running RUN : true (and prints why into $WHY) when the run is still going (sim_demo run_summary running).
still_running() { local rc; WHY="$(helper running "$1" 2>/dev/null)"; rc=$?; WHY="${WHY#running: }"; return "$rc"; }

CMD="${1:-list}"
[ $# -gt 0 ] && shift
case "$CMD" in
  -h|--help|help) usage; exit 0 ;;
  list)
    N="${1:-20}"
    case "$N" in *[!0-9]*|'') die_usage "list takes a number of runs, e.g.  logs.sh list 50" ;; esac
    helper list --runs-dir "$RUNS_DIR" --limit "$N"
    ;;
  summary|show)
    REFRESH=0; RUN_ARG=""
    for a in "$@"; do case "$a" in --refresh) REFRESH=1 ;; -*) die_usage "unknown option $a" ;; *) RUN_ARG="$a" ;; esac; done
    RUN="$(find_run "$RUN_ARG")" || exit 1
    if [ "$REFRESH" = 1 ]; then
      if still_running "$RUN"; then
        die "$(basename "$RUN") is still going ($WHY), so its summary is not written again now.
    Likely cause: the simulator (or its flight recorder) is still running this run.
    Fix: wait for the end of the run (demo.sh writes the summary itself), then run this again"
      fi
      helper finish "$RUN" --refresh >&2 || die "The summary of $(basename "$RUN") could not be written again (the reason is above)."
    fi
    helper show "$RUN"
    ;;
  errors)
    ARGS=(); RUN_ARG=""
    for a in "$@"; do case "$a" in --all|--errors-only) ARGS+=("$a") ;; -*) die_usage "unknown option $a" ;; *) RUN_ARG="$a" ;; esac; done
    RUN="$(find_run "$RUN_ARG")" || exit 1
    helper errors "$RUN" "${ARGS[@]}"
    ;;
  tail)
    LOG="${1:-}"; [ -n "$LOG" ] || die_usage "tail needs a log name: sim, team_stack, demo, recorder or a file name"
    shift
    LINES=40; FOLLOW=0; RUN_ARG=""
    while [ $# -gt 0 ]; do
      case "$1" in
        -n) shift; LINES="${1:-}"
            case "$LINES" in ''|*[!0-9]*) die_usage "-n needs a number of lines, e.g.  logs.sh tail sim -n 100" ;; esac ;;
        -f) FOLLOW=1 ;;
        -*) die_usage "unknown option $1" ;;
        *) RUN_ARG="$1" ;;
      esac
      shift
    done
    RUN="$(find_run "$RUN_ARG")" || exit 1
    case "$LOG" in
      sim|team_stack|demo) FILE="$RUN/$LOG.log" ;;
      recorder|flight_recorder) FILE="$RUN/flight_recorder.log" ;;
      *) FILE="$RUN/$LOG" ;;
    esac
    [ -f "$FILE" ] || die "There is no $FILE.
    Likely cause: this run did not write that log (older runs have no demo.log or flight_recorder.log).
    Fix: see which files the run has:  ls \"$RUN\""
    info "$FILE"
    if [ "$FOLLOW" = 1 ]; then tail -n "$LINES" -f "$FILE"; else tail -n "$LINES" "$FILE"; fi
    ;;
  csv)
    RUN="$(find_run "${1:-}")" || exit 1
    if [ ! -d "$RUN/csv" ] && still_running "$RUN"; then
      die "$(basename "$RUN") is still going ($WHY) and has no csv/ folder.
    Likely cause: it runs without the flight recorder (--no-recorder, or its port was busy).
    Fix: wait for the end of the run (photos.csv, detections.csv and the rest are written then), then run this again"
    fi
    if [ ! -d "$RUN/csv" ]; then
      warn "This run has no csv/ folder yet (it ran before the CSV recorder existed). Writing it now from the run's files."
      exporter "$RUN" || die "The CSV files could not be written (the reason is above)."
    fi
    say "CSV files of $(basename "$RUN")"
    info "Linux:   $RUN/csv"
    WIN="$(helper winpath "$RUN/csv")"
    [ "$WIN" = "$RUN/csv" ] || info "Windows: $WIN      (paste it into the File Explorer address bar)"
    for f in "$RUN"/csv/*.csv; do
      [ -f "$f" ] || continue
      info "$(printf '%-24s %6s rows' "$(basename "$f")" "$(($(wc -l < "$f") - 1))")"
    done
    ;;
  export)
    [ $# -gt 0 ] || set -- ""
    DIRS=()
    for r in "$@"; do
      d="$(find_run "$r")" || exit 1
      if still_running "$d"; then
        die "$(basename "$d") is still going ($WHY): its CSV files are not rewritten while it runs.
    Likely cause: the newest run is the one flying right now (leave RUN out = the newest run).
    Fix: wait for its end (demo.sh completes csv/ itself), or name an older run:  bash \"$ASTRA_SCRIPTS_DIR/logs.sh\" list"
      fi
      DIRS+=("$d")
    done
    exporter "${DIRS[@]}"
    ;;
  bundle)
    ARGS=(); RUN_ARG=""
    while [ $# -gt 0 ]; do
      case "$1" in
        --photos) ARGS+=(--photos) ;;
        --max-mb|--out) ARGS+=("$1" "${2:-}"); shift ;;
        -*) die_usage "unknown option $1" ;;
        *) RUN_ARG="$1" ;;
      esac
      shift
    done
    RUN="$(find_run "$RUN_ARG")" || exit 1
    if still_running "$RUN"; then
      warn "$(basename "$RUN") is still going ($WHY). Consequence: the zip holds the files as they are now (no run" \
           "summary is written into a running run). Fix: for the complete run, zip it again after its end"
      ARGS+=(--note "the run was still going when this zip was made ($WHY): files may be incomplete")
    elif [ ! -f "$RUN/run_summary.md" ]; then
      helper finish "$RUN" --partial --no-index --note "written later by logs.sh bundle from the files on disk (the run never wrote one)" >/dev/null
    fi
    helper bundle "$RUN" "${ARGS[@]}"
    ;;
  path)
    find_run "${1:-}"
    ;;
  *) die_usage "unknown command \"$CMD\" (list, summary, errors, tail, csv, export, bundle, path)" ;;
esac
