# shellcheck shell=bash
# =============================================================================
#  simulation/jetson/lib/target.sh  --  "which Jetson, and how do I reach it?" (sourced, never run by itself)
#
#  PURPOSE: the ONE place that knows how the PC reaches the Jetson Orin Nano. Every Jetson command (jetson.sh)
#  and the PC-side scripts (simulation/jetson/pc/*) source this file and call  jetson_target_load "$@" . It reads
#  the target (USB cable, any SSH address, or the "virtual Jetson" container on this PC) from, in this order:
#  the command-line options  >  the environment (JETSON_HOST ...)  >  the saved file ~/.astra_sim/jetson/target.env
#  >  the defaults (USB cable, 192.168.55.1, port 22, login mode). It NEVER connects by itself. It then offers small
#  functions to run a command on the Jetson through the owner's login window (SSH connection sharing: the owner
#  types the Jetson password in a visible window once; every command here reuses that connection and never sees
#  or stores a password), to copy files, and to add or remove SSH "forwards" (the doors the simulator link uses).
#
#  This is the contract of docs/jetson/HIL_DESIGN.md section 15.3 a: names of variables and functions here must not
#  change without updating that section and the PC-side scripts.
#
#  Variables set by jetson_target_load:
#    JETSON_LINK        usb | ssh | virtual
#    JETSON_HOST        192.168.55.1 (USB), any name/IP (ssh), "localhost" (virtual)
#    JETSON_USER        the Linux user on the Jetson ("" = not known yet: the login window asks)
#    JETSON_PORT        22
#    JETSON_AUTH        login (owner's window, the default) | key (optional SSH key, HIL_DESIGN.md 4.1)
#    JETSON_DEST        USER@HOST or HOST, what ssh is given
#    JETSON_CM          the login window's control socket (~/.ssh/cm-jetson, same as the lead's jetson_login.sh)
#    JETSON_CM_A/_B     the connection for commands + link A, and for link B + photos (login mode: both = JETSON_CM)
#    JETSON_KEY         ~/.astra_sim/jetson/id_ed25519 (key mode only)
#    JETSON_KNOWN_HOSTS ~/.astra_sim/jetson/known_hosts (the one known-hosts file of the kit)
#    JETSON_STATE       ~/.astra_sim/jetson (PC-side files of the kit)
#    JETSON_DATA        the Jetson data folder, absolute ($HOME/astra_hil of the Jetson user, saved by login/setup;
#                       "" until the first connection: call jetson_resolve_data; virtual: ~/.astra_sim/jetson_virtual)
#    JETSON_CONTAINER   astra-hil (board) | astra-hil-virtual (virtual)
#    JETSON_IMAGE       astra-hil:jetson (board) | astra-hil:virtual (virtual)
#    JETSON_SSH         bash array: the full command prefix to run a remote command through JETSON_CM_A (virtual: empty)
#    JETSON_SSH_B       the same through JETSON_CM_B
#    JETSON_RSYNC_RSH   the value for  rsync -e  (through JETSON_CM_B)
#    JETSON_ARGS        bash array: the arguments jetson_target_load did not use (the caller's own options)
#  Functions: jetson_target_load, jetson_logged_in, jetson_need_login, jetson_ssh, jetson_ssh_b, jetson_rsync,
#             jetson_forward add|cancel A|B -R SPEC|-L SPEC ..., jetson_die CODE MESSAGE, jetson_target_print env|json,
#             jetson_resolve_data, jetson_save_target.
#  Test hooks (tests only): JETSON_SSH_BIN (default ssh), JETSON_RSYNC_BIN (default rsync).
# =============================================================================

JETSON_LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JETSON_DIR="$(cd "$JETSON_LIB_DIR/.." && pwd)"
JETSON_REPO="$(cd "$JETSON_DIR/../.." && pwd)"
# Messages say "[jetson] ..." like every simulator script says "[demo] ...".
ASTRA_PROG="${ASTRA_PROG:-jetson}"
# shellcheck source=../../scripts/_common.sh
source "$JETSON_REPO/simulation/scripts/_common.sh"

JETSON_USB_HOST="192.168.55.1"          # the Jetson's own address on the USB-C cable (NVIDIA devkit user guide)

# die_usage MESSAGE : a typing mistake in a Jetson command (exit 2); the fix is jetson.sh's own --help.
die_usage() {
  printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "$*. Fix: see the commands and options with  bash \"$JETSON_DIR/jetson.sh\" --help")" >&2
  exit 2
}

# jetson_die CODE MESSAGE : like die (red " XX [jetson] ERROR: ..."), but with the exit code of HIL_DESIGN.md 15.4.
jetson_die() {
  local code="$1"; shift
  printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "$*")" >&2
  exit "$code"
}

# _jt_read_env FILE PREFIX : read KEY=VALUE lines of a file we wrote (no shell code is run) into _jt_file_<KEY>.
_jt_read_env() {
  local file="$1" line key val
  [ -f "$file" ] || return 0
  while IFS= read -r line || [ -n "$line" ]; do
    case "$line" in ''|'#'*) continue ;; *=*) ;; *) continue ;; esac
    key="${line%%=*}"; val="${line#*=}"
    val="${val%\"}"; val="${val#\"}"; val="${val%\'}"; val="${val#\'}"
    case "$key" in *[!A-Za-z0-9_]*|'') continue ;; esac
    printf -v "_jt_file_$key" '%s' "$val"
  done < "$file"
}

# jetson_target_load "$@" : parse the target options, fill the variables above. Unused arguments -> JETSON_ARGS.
jetson_target_load() {
  JETSON_STATE="${JETSON_STATE_DIR:-${ASTRA_SIM_HOME:-$HOME/.astra_sim}/jetson}"
  # 1. what the environment says (before we overwrite anything)
  local e_link="${JETSON_LINK:-}" e_host="${JETSON_HOST:-}" e_user="${JETSON_USER:-}" e_port="${JETSON_PORT:-}"
  local e_auth="${JETSON_AUTH:-}" e_data="${JETSON_DATA_DIR:-}"
  # 2. the options
  local o_link="" o_host="" o_user="" o_port="" o_auth=""
  JETSON_ARGS=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --link)  [ $# -ge 2 ] || jetson_die 2 "--link needs a value: usb, ssh or virtual. Fix: e.g. --link usb"; o_link="$2"; shift 2 ;;
      --link=*) o_link="${1#*=}"; shift ;;
      --host)  [ $# -ge 2 ] || jetson_die 2 "--host needs an address. Fix: e.g. --host 192.168.55.1 or --host jetson.local"; o_host="$2"; shift 2 ;;
      --host=*) o_host="${1#*=}"; shift ;;
      --user)  [ $# -ge 2 ] || jetson_die 2 "--user needs the Jetson user name. Fix: e.g. --user astra-dev"; o_user="$2"; shift 2 ;;
      --user=*) o_user="${1#*=}"; shift ;;
      --port)  [ $# -ge 2 ] || jetson_die 2 "--port needs a number. Fix: e.g. --port 22"; o_port="$2"; shift 2 ;;
      --port=*) o_port="${1#*=}"; shift ;;
      --auth)  [ $# -ge 2 ] || jetson_die 2 "--auth needs login or key. Fix: e.g. --auth login"; o_auth="$2"; shift 2 ;;
      --auth=*) o_auth="${1#*=}"; shift ;;
      *) JETSON_ARGS+=("$1"); shift ;;
    esac
  done
  # 3. the saved files: target.env (setup/login), login.env (the open login window's target)
  local _jt_file_JETSON_LINK="" _jt_file_JETSON_HOST="" _jt_file_JETSON_USER="" _jt_file_JETSON_PORT=""
  local _jt_file_JETSON_AUTH="" _jt_file_JETSON_DATA=""
  _jt_read_env "$JETSON_STATE/target.env"
  local f_link="$_jt_file_JETSON_LINK" f_host="$_jt_file_JETSON_HOST" f_user="$_jt_file_JETSON_USER"
  local f_port="$_jt_file_JETSON_PORT" f_auth="$_jt_file_JETSON_AUTH" f_data="$_jt_file_JETSON_DATA"

  # Pick: options > environment > file > defaults. A host given without a link means "ssh" (unless it is the USB one).
  JETSON_LINK="${o_link:-$e_link}"
  JETSON_HOST="${o_host:-$e_host}"
  if [ -z "$JETSON_LINK" ]; then
    if [ -n "$JETSON_HOST" ]; then
      if [ "$JETSON_HOST" = "$JETSON_USB_HOST" ]; then JETSON_LINK=usb; else JETSON_LINK=ssh; fi
    else
      # the saved board (usb or ssh); the virtual Jetson is never a saved default: it is always asked for with
      # --link virtual, so "jetson.sh check" on the day the board is plugged in checks the BOARD, not this PC
      JETSON_LINK="${f_link:-usb}"
      [ "$JETSON_LINK" = virtual ] && JETSON_LINK=usb
    fi
  fi
  case "$JETSON_LINK" in
    usb|ssh|virtual) ;;
    *) jetson_die 2 "unknown link '$JETSON_LINK'. Likely cause: a typing mistake. Fix: --link usb (USB-C cable), --link ssh --host HOST (Ethernet, Wi-Fi, LAN, Tailscale) or --link virtual (the container on this PC)" ;;
  esac
  if [ -z "$JETSON_HOST" ]; then
    case "$JETSON_LINK" in
      usb) JETSON_HOST="$JETSON_USB_HOST" ;;
      virtual) JETSON_HOST="localhost" ;;
      ssh) if [ "$f_link" = ssh ] && [ -n "$f_host" ]; then JETSON_HOST="$f_host"
           elif [ -n "$f_host" ] && [ "$f_host" != "$JETSON_USB_HOST" ]; then JETSON_HOST="$f_host"
           else jetson_die 2 "--link ssh needs the Jetson's address. Likely cause: no --host was given and none is saved yet. Fix: add --host HOST (an IP such as 192.168.144.20, or a name such as jetson.local), or find it with: bash \"$JETSON_DIR/jetson.sh\" find"
           fi ;;
    esac
  fi
  # the saved user/port/auth/data belong to the saved host only
  local same_host=0
  [ -n "$f_host" ] && [ "$f_host" = "$JETSON_HOST" ] && same_host=1
  [ "$same_host" = 1 ] || { f_user=""; f_port=""; f_data=""; }
  JETSON_USER="${o_user:-${e_user:-$f_user}}"
  JETSON_PORT="${o_port:-${e_port:-${f_port:-22}}}"
  JETSON_AUTH="${o_auth:-${e_auth:-${f_auth:-login}}}"
  case "$JETSON_PORT" in ''|*[!0-9]*) jetson_die 2 "--port must be a number (got '$JETSON_PORT'). Fix: e.g. --port 22" ;; esac
  case "$JETSON_AUTH" in
    login|key) ;;
    *) jetson_die 2 "unknown --auth '$JETSON_AUTH'. Fix: --auth login (you type the password in a window, the default) or --auth key (after jetson.sh setup --key)" ;;
  esac
  # the user of the open login window, when none is known otherwise (the window asked for it)
  if [ -z "$JETSON_USER" ] && [ "$JETSON_LINK" != virtual ]; then
    local _jt_file_HOST="" _jt_file_USER=""
    _jt_read_env "$JETSON_STATE/login.env"
    [ "$_jt_file_HOST" = "$JETSON_HOST" ] && JETSON_USER="$_jt_file_USER"
  fi

  JETSON_CM="${JETSON_CM_PATH:-$HOME/.ssh/cm-jetson}"
  JETSON_KEY="$JETSON_STATE/id_ed25519"
  JETSON_KNOWN_HOSTS="$JETSON_STATE/known_hosts"
  if [ "$JETSON_AUTH" = key ]; then
    JETSON_CM_A="$JETSON_STATE/cm-A"; JETSON_CM_B="$JETSON_STATE/cm-B"
  else
    JETSON_CM_A="$JETSON_CM"; JETSON_CM_B="$JETSON_CM"
  fi
  JETSON_DEST="${JETSON_USER:+$JETSON_USER@}$JETSON_HOST"

  if [ "$JETSON_LINK" = virtual ]; then
    JETSON_DATA="${e_data:-${ASTRA_SIM_HOME:-$HOME/.astra_sim}/jetson_virtual}"
    JETSON_CONTAINER="astra-hil-virtual"; JETSON_IMAGE="astra-hil:virtual"
    JETSON_SSH=(); JETSON_SSH_B=(); JETSON_RSYNC_RSH=""
  else
    if [ -n "$e_data" ]; then JETSON_DATA="$e_data"
    elif [ -n "$f_data" ]; then JETSON_DATA="$f_data"
    else JETSON_DATA=""                      # learned on the first connection: $HOME/astra_hil (jetson_resolve_data)
    fi
    # shellcheck disable=SC2034  # used by jetson.sh and the PC-side scripts
    JETSON_CONTAINER="astra-hil"
    # shellcheck disable=SC2034
    JETSON_IMAGE="astra-hil:jetson"
    # The ONE SSH option set of the kit (HIL_DESIGN.md 15.3 a). BatchMode: never ask for a password here.
    JETSON_SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=2 -o ServerAliveCountMax=3
                     -o "UserKnownHostsFile=$JETSON_KNOWN_HOSTS" -o StrictHostKeyChecking=yes
                     -o ExitOnForwardFailure=yes -o LogLevel=ERROR -p "$JETSON_PORT")
    local sshbin="${JETSON_SSH_BIN:-ssh}"
    JETSON_SSH=("$sshbin" "${JETSON_SSH_OPTS[@]}" -S "$JETSON_CM_A" -o ControlMaster=no "$JETSON_DEST")
    JETSON_SSH_B=("$sshbin" "${JETSON_SSH_OPTS[@]}" -S "$JETSON_CM_B" -o ControlMaster=no "$JETSON_DEST")
    local w q=""
    for w in "$sshbin" "${JETSON_SSH_OPTS[@]}" -S "$JETSON_CM_B" -o ControlMaster=no; do q+="$(printf '%q' "$w") "; done
    JETSON_RSYNC_RSH="${q% }"
  fi
  export JETSON_LINK JETSON_HOST JETSON_USER JETSON_PORT JETSON_AUTH
}

# jetson_cm_check SOCKET : 0 when an SSH master answers on SOCKET.
jetson_cm_check() {
  [ -S "$1" ] || return 1
  "${JETSON_SSH_BIN:-ssh}" -S "$1" -O check -o LogLevel=ERROR "$JETSON_DEST" >/dev/null 2>&1
}

# _jt_open_key_master SOCKET : key mode only, open our own connection with the key (no password, no window).
_jt_open_key_master() {
  [ -f "$JETSON_KEY" ] || return 1
  "${JETSON_SSH_BIN:-ssh}" "${JETSON_SSH_OPTS[@]}" -i "$JETSON_KEY" -o IdentitiesOnly=yes -o ControlMaster=yes \
      -o ControlPersist=yes -S "$1" -fN "$JETSON_DEST" 2>/dev/null
}

# jetson_logged_in : 0 when commands can run on the Jetson now (login mode: the owner's window is open; key mode:
# our connections are open or could be opened with the key). Always 0 for virtual.
jetson_logged_in() {
  [ "$JETSON_LINK" = virtual ] && return 0
  if [ "$JETSON_AUTH" = key ]; then
    jetson_cm_check "$JETSON_CM_A" || _jt_open_key_master "$JETSON_CM_A" || return 1
    jetson_cm_check "$JETSON_CM_B" || _jt_open_key_master "$JETSON_CM_B" || return 1
    return 0
  fi
  jetson_cm_check "$JETSON_CM"
}

# jetson_need_login : print the standard "Not logged in" message (owner's wording, entry 11:40); returns 17.
jetson_need_login() {
  local where="${JETSON_DEST:-$JETSON_HOST}"
  if [ "$JETSON_AUTH" = key ]; then
    printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "SSH login refused for $where with the key. Likely cause: the key was never added on the Jetson (or was removed), or the Jetson is not reachable. Fix: bash \"$JETSON_DIR/jetson.sh\" login (then: jetson.sh setup --key), or use login mode: add --auth login")" >&2
  else
    printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "Not logged in to $where. Likely cause: the login window is closed or its connection dropped. Fix: bash \"$JETSON_DIR/jetson.sh\" login (a window opens; type the Jetson password there; keep it open)")" >&2
  fi
  return 17
}

# jetson_ssh "COMMAND" : run one shell command on the Jetson through connection A (virtual: here, in bash).
# Stdin is passed through, so  jetson_ssh "bash -s" < script.sh  runs a local script remotely.
jetson_ssh() {
  if [ "$JETSON_LINK" = virtual ]; then bash -c "$*"; return; fi
  "${JETSON_SSH[@]}" "$*"
}
# jetson_ssh_b "COMMAND" : the same through connection B (photos, link B).
jetson_ssh_b() {
  if [ "$JETSON_LINK" = virtual ]; then bash -c "$*"; return; fi
  "${JETSON_SSH_B[@]}" "$*"
}
# jetson_rsync ARGS... : rsync through connection B; write remote paths as ":PATH" (the host is added here).
jetson_rsync() {
  local a args=()
  if [ "$JETSON_LINK" = virtual ]; then
    for a in "$@"; do case "$a" in :*) args+=("${a#:}") ;; *) args+=("$a") ;; esac; done
    "${JETSON_RSYNC_BIN:-rsync}" "${args[@]}"; return
  fi
  for a in "$@"; do case "$a" in :*) args+=("$JETSON_DEST$a") ;; *) args+=("$a") ;; esac; done
  "${JETSON_RSYNC_BIN:-rsync}" -e "$JETSON_RSYNC_RSH" "${args[@]}"
}

# jetson_forward add|cancel A|B -R SPEC [-L SPEC ...] : add or remove forwards on a running connection (ssh -O).
jetson_forward() {
  local op="$1" which="$2"; shift 2
  local sock; case "$which" in A) sock="$JETSON_CM_A" ;; B) sock="$JETSON_CM_B" ;; *) return 2 ;; esac
  local o; case "$op" in add) o=forward ;; cancel) o=cancel ;; *) return 2 ;; esac
  [ "$JETSON_LINK" = virtual ] && return 0
  "${JETSON_SSH_BIN:-ssh}" -S "$sock" -O "$o" -o LogLevel=ERROR -o ExitOnForwardFailure=yes "$@" "$JETSON_DEST"
}

# jetson_resolve_data : make sure JETSON_DATA is an absolute path (asks the Jetson for $HOME once if needed).
jetson_resolve_data() {
  [ -n "$JETSON_DATA" ] && return 0
  local h; h=$(jetson_ssh 'printf %s "$HOME"' 2>/dev/null) || return 1
  [ -n "$h" ] || return 1
  JETSON_DATA="$h/astra_hil"
  if [ -z "$JETSON_USER" ]; then
    JETSON_USER=$(jetson_ssh 'id -un' 2>/dev/null)
  fi
}

# jetson_save_target : remember the current target in ~/.astra_sim/jetson/target.env (no secrets in it).
jetson_save_target() {
  # the virtual Jetson is never saved as the target (it would hide the real board's settings); 1 = nothing saved
  [ "$JETSON_LINK" = virtual ] && return 1
  mkdir -p "$JETSON_STATE" && chmod 700 "$JETSON_STATE"
  {
    echo "# target.env -- written by simulation/jetson/jetson.sh (login/setup): the Jetson the kit talks to. Edit or delete freely."
    echo "JETSON_LINK=$JETSON_LINK"
    echo "JETSON_HOST=$JETSON_HOST"
    echo "JETSON_USER=$JETSON_USER"
    echo "JETSON_PORT=$JETSON_PORT"
    echo "JETSON_AUTH=$JETSON_AUTH"
    echo "JETSON_DATA=$JETSON_DATA"
  } > "$JETSON_STATE/target.env"
}

# jetson_target_print env|json : the target values for other programs (jetson.sh target).
jetson_target_print() {
  local fmt="${1:-env}" sshq="" w
  for w in "${JETSON_SSH[@]}"; do sshq+="$(printf '%q' "$w") "; done
  sshq="${sshq% }"
  local keys=(JETSON_LINK JETSON_HOST JETSON_USER JETSON_PORT JETSON_AUTH JETSON_DEST JETSON_CM JETSON_CM_A JETSON_CM_B
              JETSON_KEY JETSON_KNOWN_HOSTS JETSON_STATE JETSON_DATA JETSON_CONTAINER JETSON_IMAGE)
  local k
  if [ "$fmt" = json ]; then
    # shellcheck disable=SC2163
    for k in "${keys[@]}"; do export "$k"; done
    python3 - "$sshq" "$JETSON_RSYNC_RSH" "${keys[@]}" <<'PY'
import json, os, sys
sshq, rsh, keys = sys.argv[1], sys.argv[2], sys.argv[3:]
out = {k: os.environ.get(k, "") for k in keys}
out["JETSON_SSH"] = sshq
out["JETSON_RSYNC_RSH"] = rsh
print(json.dumps(out, indent=2))
PY
  else
    for k in "${keys[@]}"; do printf '%s=%s\n' "$k" "${!k}"; done
    printf 'JETSON_SSH=%s\n' "$sshq"
    printf 'JETSON_RSYNC_RSH=%s\n' "$JETSON_RSYNC_RSH"
  fi
}
