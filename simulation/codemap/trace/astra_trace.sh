# shellcheck shell=bash
# =============================================================================
#  astra_trace.sh  --  the opt-in live FUNCTION tracer for the simulator's bash scripts (ASTRA code map)
#
#  PURPOSE: lets the code map web page (simulation/codemap, served by server.py) light up a bash function of
#  simulation/scripts (say, astra_new_session, finish, stop_group ...) at the moment it runs.
#    * It is sourced ONLY when ASTRA_TRACE=1, by the 3-line hook in simulation/scripts/_common.sh (which every
#      simulator script sources). Without ASTRA_TRACE=1 this file is never read and nothing changes.
#    * How it notices calls: `set -T` plus a DEBUG trap. Bash runs the trap before every command; the trap
#      compares the depth of the function call stack (${#FUNCNAME[@]}) with the depth it saw last time. A
#      deeper stack means "a function was just entered", so each call is reported ONCE, at its first command.
#      ${FUNCNAME[1]} is the function's name and ${BASH_SOURCE[1]} the file it was defined in.
#    * Only functions defined in files of the team repo are reported (the repo this file lives in, the
#      Linux-side copy ~/ODLC_Machine_Inferencing_System_2025-2026, any path with a folder of that name, or
#      exactly the folders in ASTRA_TRACE_ROOTS, separated by ":"). ROS's setup.bash helpers are skipped.
#    * How it reports: one small UDP message with JSON text to 127.0.0.1:8091 (ASTRA_TRACE_ADDR, numeric IPv4
#      HOST:PORT or localhost:PORT), written with bash's built-in /dev/udp/HOST/PORT, so no extra program runs:
#          {"pid": 1234, "sh": "/home/u/ODLC_.../simulation/scripts/_common.sh", "calls": [["say", 2]]}
#      (one message per file; the counts are the calls since the last message).
#    * Rate-limited exactly like the Python tracer (the code map's contract): AT MOST ONE FLUSH EVERY 100 ms PER
#      PROCESS. A flush is one message per script file that has new calls (almost always just one file). A call
#      that comes sooner is counted and waits. The waiting counts go out at the first command the script runs
#      once the 100 ms are over (or at the next function call), right before `exec` (which replaces the shell
#      without running the EXIT trap), and when the script exits (EXIT trap; only when the script has no EXIT
#      trap of its own: this file never replaces one). A quiet script therefore reports each call at once.
#    * Subshells ($( ), ( ), the parts of a pipeline) are processes of their own: a subshell starts with EMPTY
#      counts and its own 100 ms clock (so the parent's waiting counts are never sent twice), and, when it calls
#      one of our functions, it gets our EXIT trap too if it has none of its own (bash never runs the parent's EXIT
#      trap in a subshell; `trap -p EXIT` there shows the parent's text, so only "none" or "ours" is replaced).
#    * Never fails the script: every command here is guarded (works with set -e, set -u, set -o pipefail), the
#      trap always returns 0, `$?` and PIPESTATUS of the script are kept (bash restores them after a trap),
#      sending errors are silenced, and the trap is not installed when the script already has a DEBUG trap.
#      Every name it adds starts with _astra_tr / _ASTRA_TR_ (it can never overwrite a script's own variable),
#      and the helpers' working variables are local.
#    * What can still be lost: the counts of the last 100 ms of a script (or subshell) that has an EXIT trap of
#      its own, or that is killed; a script that runs `set -x` also prints the trap's commands.
#  Needs bash 4.2 or newer (Ubuntu 22.04 has 5.1); older bash versions simply skip tracing.
# =============================================================================

if [ "${ASTRA_TRACE:-0}" != 1 ] || [ -n "${_ASTRA_TR_LOADED:-}" ] || [ -z "${BASH_VERSINFO[0]:-}" ] \
   || [ "${BASH_VERSINFO[0]}" -lt 4 ] || { [ "${BASH_VERSINFO[0]}" -eq 4 ] && [ "${BASH_VERSINFO[1]}" -lt 2 ]; }; then
  return 0 2>/dev/null || exit 0
fi
_ASTRA_TR_LOADED=1

# ---- where to send: numeric IPv4 only (a host NAME would need a DNS look-up, which could block) ----------
_ASTRA_TR_HOST=${ASTRA_TRACE_ADDR:-127.0.0.1:8091}
_ASTRA_TR_PORT=${_ASTRA_TR_HOST##*:}
_ASTRA_TR_HOST=${_ASTRA_TR_HOST%:*}
if [ "$_ASTRA_TR_HOST" = localhost ]; then _ASTRA_TR_HOST=127.0.0.1; fi
case "$_ASTRA_TR_HOST" in ''|*[!0-9.]*) _ASTRA_TR_HOST="" ;; esac
case "$_ASTRA_TR_PORT" in ''|*[!0-9]*) _ASTRA_TR_HOST="" ;; esac
if [ -n "$_ASTRA_TR_HOST" ] && { [ "${#_ASTRA_TR_PORT}" -gt 5 ] || [ "$_ASTRA_TR_PORT" -lt 1 ] || [ "$_ASTRA_TR_PORT" -gt 65535 ]; }; then
  _ASTRA_TR_HOST=""
fi
if [ -z "$_ASTRA_TR_HOST" ]; then
  return 0 2>/dev/null || exit 0       # unusable ASTRA_TRACE_ADDR: no tracing at all (never an error)
fi

# ---- which files count as ours -------------------------------------------------------------------------
_ASTRA_TR_PWD0=$PWD                    # relative ${BASH_SOURCE[...]} paths are relative to where we started
_ASTRA_TR_ROOTS=()
_ASTRA_TR_SEGMENT=""
if [ -n "${ASTRA_TRACE_ROOTS:-}" ]; then
  _astra_tr_list=$ASTRA_TRACE_ROOTS
  while [ -n "$_astra_tr_list" ]; do
    _astra_tr_one=${_astra_tr_list%%:*}
    if [ "$_astra_tr_one" = "$_astra_tr_list" ]; then _astra_tr_list=""; else _astra_tr_list=${_astra_tr_list#*:}; fi
    _astra_tr_one=${_astra_tr_one%/}
    if [ -n "$_astra_tr_one" ]; then _ASTRA_TR_ROOTS+=("$_astra_tr_one"); fi
  done
  unset _astra_tr_list _astra_tr_one
else
  _astra_tr_here=${BASH_SOURCE[0]%/*}
  if [ "$_astra_tr_here" = "${BASH_SOURCE[0]}" ]; then _astra_tr_here=.; fi
  # <repo>/simulation/codemap/trace -> <repo>, both as written (logical) and with links resolved (physical).
  # Only when this folder really sits at <repo>/simulation/codemap/trace: a copy of this file somewhere else
  # (say /tmp/x) must never make "/" or another big folder a root (every script would count as ours).
  for _astra_tr_one in "$(cd "$_astra_tr_here" 2>/dev/null && pwd)" "$(cd "$_astra_tr_here" 2>/dev/null && pwd -P)"; do
    case "$_astra_tr_one" in
      */simulation/codemap/trace)
        _astra_tr_one=${_astra_tr_one%/simulation/codemap/trace}
        if [ -n "$_astra_tr_one" ]; then _ASTRA_TR_ROOTS+=("$_astra_tr_one"); fi ;;
    esac
  done
  _ASTRA_TR_ROOTS+=("${HOME:-/nonexistent}/ODLC_Machine_Inferencing_System_2025-2026")
  _ASTRA_TR_SEGMENT="/ODLC_Machine_Inferencing_System_2025-2026/"
  unset _astra_tr_here _astra_tr_one
fi

# ---- state ---------------------------------------------------------------------------------------------
declare -gA _ASTRA_TR_PEND=()          # "file<TAB>function" -> calls not sent yet (of process _ASTRA_TR_PID)
declare -gA _ASTRA_TR_OURS=()          # file as bash names it -> 1 (ours) / 0 (not ours): decided once per file
declare -gA _ASTRA_TR_PATH=()          # file as bash names it -> its clean absolute path (what is reported)
_ASTRA_TR_DEPTH=0                      # call-stack depth seen by the previous trap
_ASTRA_TR_HAS=0                        # 1 while _ASTRA_TR_PEND holds counts (part of the trap's one test)
_ASTRA_TR_LAST=0                       # time of this process's last flush in microseconds (0 = none yet)
_ASTRA_TR_NEXT=0                       # the earliest time of its next flush: _ASTRA_TR_LAST + 100 ms (0 = at once)
_ASTRA_TR_PID=${BASHPID:-$$}           # the process the waiting counts belong to (a subshell starts its own)
_ASTRA_TR_XPID=${BASHPID:-$$}          # the process whose EXIT trap was already taken care of (see the end)
# Our EXIT trap. It switches our DEBUG trap off FIRST: bash runs the DEBUG trap inside an EXIT trap too, and the
# "send what waits" check would then start a second flush in the middle of this last one.
_ASTRA_TR_EXITCMD='trap - DEBUG; _astra_tr_flush 1 || :'

# _astra_tr_is_ours FILE : true when FILE (an absolute path) belongs to the team repo.
_astra_tr_is_ours() {
  local _r
  case "$1" in */simulation/codemap/*) return 1 ;; esac      # the code map tools themselves never count
  for _r in "${_ASTRA_TR_ROOTS[@]}"; do
    case "$1" in "$_r"/*) return 0 ;; esac
  done
  if [ -n "$_ASTRA_TR_SEGMENT" ]; then
    case "$1" in *"$_ASTRA_TR_SEGMENT"*) return 0 ;; esac
  fi
  return 1
}

# _astra_tr_normpath /ABS/PATH : the same path with "." and ".." parts worked out, as text only (the file is
# not touched), into _ASTRA_TR_E. "/repo/simulation/scripts/../../../x/lib.sh" -> "/x/lib.sh": without this a
# file outside the repo, sourced through "..", would look as if it were inside.
_astra_tr_normpath() {
  local _rest=${1#/} _seg _out=""
  while [ -n "$_rest" ]; do
    _seg=${_rest%%/*}
    if [ "$_seg" = "$_rest" ]; then _rest=""; else _rest=${_rest#*/}; fi
    case "$_seg" in
      ''|.) ;;
      ..) _out=${_out%/*} ;;
      *) _out="$_out/$_seg" ;;
    esac
  done
  _ASTRA_TR_E=${_out:-/}
}

# _astra_tr_esc TEXT : print TEXT as the inside of a JSON string (no subprocess: parameter expansion only).
# The result goes into the global _ASTRA_TR_E (printing it would need a $( ) subshell for every call).
_astra_tr_esc() {
  local _e=$1
  _e=${_e//\\/\\\\}
  _e=${_e//\"/\\\"}
  _e=${_e//$'\n'/\\n}
  _e=${_e//$'\r'/\\r}
  _e=${_e//$'\t'/\\t}
  _ASTRA_TR_E=$_e
}

# _astra_tr_flush [force] : send this process's waiting counts, one message per file, when its last flush is at
# least 100 ms ago (the contract's rate limit) or right before `exec`; with "1" always (the EXIT trap).
_astra_tr_flush() {
  local _now _k _s _n _c _msg
  if [ "$_ASTRA_TR_HAS" != 1 ]; then return 0; fi               # nothing waits
  if [ "${BASHPID:-$$}" != "$_ASTRA_TR_PID" ]; then return 0; fi  # a subshell never sends its parent's counts
  if [ "${1:-0}" != 1 ]; then
    case "${BASH_COMMAND:-}" in
      exec|exec\ *) ;;              # the next command replaces the shell (no EXIT trap after it): send now
      *)
        _now=${EPOCHREALTIME:-}
        _now=${_now//[!0-9]/}
        if [ -z "$_now" ]; then _now=$(( SECONDS * 1000000 )); fi     # bash < 5.0: whole seconds only
        # less than 100 ms since this process's last flush: keep counting, send later (a clock that went
        # backwards, now < last, never blocks sending)
        if [ "$_now" -lt "$_ASTRA_TR_NEXT" ] && [ "$_now" -ge "$_ASTRA_TR_LAST" ]; then return 0; fi
        _ASTRA_TR_LAST=$_now
        _ASTRA_TR_NEXT=$(( _now + 100000 )) ;;
    esac
  fi
  local -A _bysrc=()
  for _k in "${!_ASTRA_TR_PEND[@]}"; do
    _s=${_k%%$'\t'*}
    _n=${_k#*$'\t'}
    _c=${_ASTRA_TR_PEND[$_k]:-0}
    if [ "$_c" = 0 ]; then continue; fi
    _astra_tr_esc "$_n"
    _bysrc[$_s]+=",[\"$_ASTRA_TR_E\",$_c]"
  done
  _ASTRA_TR_PEND=()
  _ASTRA_TR_HAS=0
  for _s in "${!_bysrc[@]}"; do
    _astra_tr_esc "$_s"
    _msg="{\"pid\":${BASHPID:-$$},\"sh\":\"$_ASTRA_TR_E\",\"calls\":[${_bysrc[$_s]#,}]}"
    { printf '%s' "$_msg" > "/dev/udp/$_ASTRA_TR_HOST/$_ASTRA_TR_PORT"; } 2>/dev/null || :
  done
  return 0
}

# _astra_tr_newproc : the first traced moment of a subshell (a new process). The waiting counts it inherited are
# its parent's (the parent sends them itself), so they are dropped here; the subshell gets its own 100 ms clock.
_astra_tr_newproc() {
  _ASTRA_TR_PID=${BASHPID:-$$}
  _ASTRA_TR_PEND=()
  _ASTRA_TR_HAS=0
  _ASTRA_TR_LAST=0
  _ASTRA_TR_NEXT=0
  return 0
}

# _astra_tr_exit_trap : once per subshell that counts a function: give it our EXIT trap (so its last counts are sent
# when it ends) unless it has one of its own. In a subshell `trap -p EXIT` shows the PARENT's EXIT trap (bash keeps
# the text but never runs it there) or the subshell's own one, so only "none" or "ours" is safe to (re)place.
_astra_tr_exit_trap() {
  local _t
  _ASTRA_TR_XPID=$_ASTRA_TR_PID
  _t=$(trap -p EXIT 2>/dev/null) || _t=x
  if [ -z "$_t" ] || [ "$_t" = "trap -- '$_ASTRA_TR_EXITCMD' EXIT" ]; then
    trap "$_ASTRA_TR_EXITCMD" EXIT 2>/dev/null || :
  fi
  return 0
}

# _astra_tr_entry NAME FILE : a function was just entered; count it when FILE is ours (sending is left to the hook).
_astra_tr_entry() {
  local _n=$1 _raw=$2 _s _ok _k _c
  case "$_n" in ''|main|source|_astra_tr_*) return 0 ;; esac   # "source" = a file being sourced
  case "$_raw" in ''|environment) return 0 ;; esac               # (functions inherited with export -f)
  _ok=${_ASTRA_TR_OURS[$_raw]:-}
  if [ -z "$_ok" ]; then                                         # first function seen from this file
    _s=$_raw
    case "$_s" in /*) ;; *) _s="$_ASTRA_TR_PWD0/$_s" ;; esac
    case "$_s" in */./*|*/../*|*/.|*/..) _astra_tr_normpath "$_s"; _s=$_ASTRA_TR_E ;; esac
    if _astra_tr_is_ours "$_s"; then _ok=1; else _ok=0; fi
    _ASTRA_TR_OURS[$_raw]=$_ok
    _ASTRA_TR_PATH[$_raw]=$_s
  fi
  if [ "$_ok" != 1 ]; then return 0; fi
  if [ "${BASHPID:-$$}" != "$_ASTRA_TR_PID" ]; then _astra_tr_newproc; fi      # we are in a new subshell
  if [ "$_ASTRA_TR_XPID" != "$_ASTRA_TR_PID" ]; then _astra_tr_exit_trap; fi   # ... that needs an EXIT trap
  _s=${_ASTRA_TR_PATH[$_raw]}
  _k="$_s"$'\t'"$_n"
  _c=${_ASTRA_TR_PEND[$_k]:-0}
  _ASTRA_TR_PEND[$_k]=$(( _c + 1 ))
  _ASTRA_TR_HAS=1
  return 0
}

# _astra_tr_hook : called by the DEBUG trap (1) when the call stack got deeper, i.e. a function was just entered,
# and (2) while counts wait (_ASTRA_TR_HAS=1), before every other command too. It counts an entered function, then,
# while counts wait, sends them once this process's 100 ms are over or right before `exec` (it checks the clock
# itself, so _astra_tr_flush, which decides for good, is only called when a flush may be due). In a subshell that
# has not counted anything yet it only drops the counts it inherited from its parent. Inside this helper
# ${FUNCNAME[0]} is the helper itself, so ${FUNCNAME[1]} / ${BASH_SOURCE[1]} are the entered function and its file.
_astra_tr_hook() {
  local _now
  if [ "${_ASTRA_TR_D:-0}" -gt "$_ASTRA_TR_DEPTH" ]; then
    _astra_tr_entry "${FUNCNAME[1]:-}" "${BASH_SOURCE[1]:-}" || :
  fi
  if [ "$_ASTRA_TR_HAS" != 1 ]; then return 0; fi
  if [ "${BASHPID:-$$}" != "$_ASTRA_TR_PID" ]; then _astra_tr_newproc; return 0; fi
  case "${BASH_COMMAND:-}" in
    exec|exec\ *) _astra_tr_flush 0 ;;
    *)
      _now=${EPOCHREALTIME:-}
      _now=${_now//[!0-9]/}
      if [ -z "$_now" ] || [ "$_now" -ge "$_ASTRA_TR_NEXT" ] || [ "$_now" -lt "$_ASTRA_TR_LAST" ]; then
        _astra_tr_flush 0
      fi ;;
  esac
  return 0
}

# ---- switch it on (never over a DEBUG trap the script already has) ----------------------------------------
# The trap itself is three short commands: (1) the depth now, (2) deeper than before (a function was entered) OR
# counts wait -> _astra_tr_hook, (3) remember the depth. Bash re-reads the trap text before EVERY command, so it is
# kept this short; the arithmetic (( )) form measured cheaper than [ ] tests (WSL, 2026-10-31). Measured cost: see
# simulation/codemap/README.md ("How much slower does tracing make things?"). `set -x` scripts would also print
# the trap's commands.
# ${FUNCNAME[@]+...}: at a script's top level FUNCNAME is UNSET, and plain ${#FUNCNAME[@]} would stop a
# `set -u` script with "FUNCNAME: unbound variable"; this form gives "" there (counted as depth 0).
if [ -z "$(trap -p DEBUG)" ]; then
  set -T
  trap '_ASTRA_TR_D=${FUNCNAME[@]+${#FUNCNAME[@]}}; ((${_ASTRA_TR_D:-0}>_ASTRA_TR_DEPTH||_ASTRA_TR_HAS)) && _astra_tr_hook; _ASTRA_TR_DEPTH=${_ASTRA_TR_D:-0}' DEBUG
  if [ -z "$(trap -p EXIT)" ]; then
    trap "$_ASTRA_TR_EXITCMD" EXIT
  fi
fi
return 0 2>/dev/null || :
