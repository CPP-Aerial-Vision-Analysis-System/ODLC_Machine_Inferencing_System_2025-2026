#!/usr/bin/env bash
# =============================================================================
#  jetson.sh  --  the Jetson Orin Nano kit: connect, check, set up, deploy and run the team's container on it
#
#  PURPOSE: one command for everything about the real Jetson (or the "virtual Jetson" container on this PC) in
#  "Jetson in the loop" flights (docs/jetson/HIL_DESIGN.md): the team's flight software runs in ONE Docker
#  container ON the Jetson, while Gazebo, ArduPilot SITL, the camera emulator and Mission Planner stay on this PC.
#  The PC reaches the Jetson by SSH only: over the USB-C cable (the Jetson is 192.168.55.1), or with no cable over
#  Ethernet, Wi-Fi, a LAN or Tailscale (any address the PC can SSH to). The owner logs in ONCE in a visible window
#  and types the Jetson password there himself; every command here reuses that login and never sees a password.
#  Nothing here changes the team's things on the board: our own names (astra-hil, ~/astra_hil, label
#  org.astra.owner=hil), heavy commands refuse while the team is using the board, the power mode is only reported.
#  Guide for people: docs/jetson/README.md (start with "Plug in and test").
#
#  HOW TO RUN (in the Ubuntu WSL terminal, from the repo folder):   bash simulation/jetson/jetson.sh COMMAND [options]
#    check   [--quick] [--deep] [--speed] [--pc] [--for check|deploy|warmup|flight] [--team-agreed]
#                         find the board and report every step with a fix (changes nothing)
#    login   [--here] [--wait S]   open the login window (type the Jetson password there; keep it open)
#    logout                         end the login (the same as closing the window)
#    setup   [--key|--remove-key] [--yes]   save the target, create ~/astra_hil on the Jetson, print sudo steps
#    find                           look for the Jetson on the usual addresses (USB, saved, .local names, Tailscale)
#    clock                          the Jetson's clock offset to this PC (offset_s=... rtt_ms=...)
#    status  [--json]               logged in? container running? image, engine, power mode, memory, disk
#    image   build [--flavor jetson|virtual] [--x86-base trt|ros] [--no-cache] [--dry-run] | status | prune
#                                   build the image ON THIS PC (arm64 for the board through emulation)
#    deploy  [--via registry|save] [--model FILE] [--registry-port N] [--team-agreed] [--dry-run]
#                                   put the image on the Jetson (only changed layers move), copy the model, selftest
#    warmup  [--yes] [--team-agreed]   build the TensorRT engine ON the Jetson (once, with the owner's OK);
#                                   --link virtual: this PC's own engine on its GPU (deploy --link virtual runs it)
#    start   --sim|--real [--run-id ID] [--pipeline new|legacy] [--domain N] [--localhost-only] [--team-root DIR]
#            [--archive-photos|--keep-photos] [--team-agreed] [--jetson-like] [--dry-run]     start the container
#                                   (--jetson-like: the virtual Jetson gets the board's 6 CPUs / 7.4 GB)
#    stop | restart                 stop (and start again) our containers
#    logs    [-f] [--run ID]        the container's output (or one run's team_stack.log)
#    exec    [--tty] -- CMD...      run CMD inside the container as the stack's user (exit 13: not running)
#    shell   [--root]               a terminal inside the container (as the stack's user)
#    paths | target [--format env|json]   the folders / the target settings, as KEY=VALUE lines
#  Every command takes the target the same way:  --link usb|ssh|virtual  --host HOST  --user USER  --port N
#  --auth login|key   (or JETSON_HOST, JETSON_USER, JETSON_PORT, JETSON_AUTH). Default: --link usb = 192.168.55.1,
#  --auth login. A --host without --link means --link ssh. What login/setup learn is saved in
#  ~/.astra_sim/jetson/target.env, so after the first time no option is needed.
#  Examples:  bash simulation/jetson/jetson.sh check                       (USB-C cable)
#             bash simulation/jetson/jetson.sh login --host jetson.local --user astra-dev   (no cable: Wi-Fi/LAN)
#             bash simulation/jetson/jetson.sh check --link virtual        (the container on this PC)
#  Exit codes (docs/jetson/HIL_DESIGN.md 15.4): 0 OK, 1 a step failed, 2 typed wrong, 3 host unreachable, 4 SSH
#  port closed, 5 key refused, 6 host key changed, 7 not a supported board, 8 Docker, 9 NVIDIA runtime, 10 disk,
#  11 memory, 12 image not deployed/old, 13 container not running, 14 link/forward, 15 stack not ready, 16 team
#  busy, 17 not logged in, 18 PC prerequisite missing, 19 TensorRT engine missing or for another board.
# =============================================================================
set -o pipefail
JETSON_SH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/jetson.sh"
# shellcheck source=lib/target.sh
source "$(dirname "$JETSON_SH")/lib/target.sh"

J="bash \"$JETSON_SH\""                         # how messages name this script (copy-paste ready)
LABEL="org.astra.owner=hil"
REGISTRY_IMAGE="registry:2.8.3"                  # pinned tag of the small image store on the PC (9.5)
REGISTRY_NAME="astra-hil-registry"
REGISTRY_PORT_DEFAULT=29517
L4T_TAG_DEFAULT="r36.4.0"                        # nvcr.io/nvidia/l4t-jetpack tag (fits L4T R36.4.x)
BINFMT_IMAGE="tonistiigi/binfmt:qemu-v10.2.3"
TRT_X86_BASE="nvcr.io/nvidia/tensorrt:24.08-py3"   # x86 layer 1: CUDA 12.6 + cuDNN 9.3 + TensorRT 10.3, like JetPack 6.2
PHOTO_SUB="ros2_ws/src/video_cam/mapping_photos"
DETECT_SUB="ros2_ws/src/video_cam/detection_results_sahi"
W_IN="/home/astra-dev/astra"                     # the repo inside the image (the drone's own layout)

# remote preamble: docker directly (docker group) or through passwordless sudo; never asks for a password
DKPRE='if docker version >/dev/null 2>&1; then DK=docker; else DK="sudo -n docker"; fi; dk() { $DK "$@"; };'

usage() { astra_usage "$JETSON_SH"; }
q() { printf '%q' "$1"; }                         # shell-quote one word for a remote command line

# ---------------------------------------------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------------------------------------------
pc_git_sha() { git -c safe.directory='*' -C "$JETSON_REPO" rev-parse --short=12 HEAD 2>/dev/null || echo unknown; }

# tcp_open HOST PORT [SECONDS] : 0 when a TCP connection can be opened
tcp_open() { timeout "${3:-4}" bash -c "exec 3<>/dev/tcp/$1/$2" 2>/dev/null; }

need_login() {
  [ "$JETSON_LINK" = virtual ] && return 0
  if ! jetson_logged_in; then jetson_need_login; exit 17; fi
  # the open login must be to the board we were asked for (the lead's window writes no login.env: then trust it)
  local _jt_file_HOST=""
  _jt_read_env "$JETSON_STATE/login.env"
  if [ -n "$_jt_file_HOST" ] && [ "$_jt_file_HOST" != "$JETSON_HOST" ] && [ "$JETSON_AUTH" = login ]; then
    jetson_die 17 "The open login window is logged in to $_jt_file_HOST, not to $JETSON_HOST. Likely cause: another board (or another address of it) was used before. Fix: close that login window (or $J logout), then $J login --host $JETSON_HOST"
  fi
  jetson_resolve_data || jetson_die 14 "Could not ask the Jetson for its home folder through the login. Likely cause: the connection just dropped. Fix: $J login, then try again"
}

# collect_facts FILE [--only busy] : run remote_facts.sh on the target, write its KEY=VALUE lines to FILE
collect_facts() {
  local out="$1"; shift
  local args=(--data "$JETSON_DATA" --image "$JETSON_IMAGE" --container "$JETSON_CONTAINER" "$@")
  local a qa=""
  for a in "${args[@]}"; do qa+=" $(q "$a")"; done
  if [ "$JETSON_LINK" = virtual ]; then
    bash "$JETSON_LIB_DIR/remote_facts.sh" "${args[@]}" > "$out" 2>/dev/null
  else
    jetson_ssh "bash -s --$qa" < "$JETSON_LIB_DIR/remote_facts.sh" > "$out" 2>/dev/null
  fi
}

judge() {   # judge FACTS_FILE [jetson_facts.py options...]
  local f="$1"; shift
  python3 "$JETSON_LIB_DIR/jetson_facts.py" judge --facts "$f" --link "$JETSON_LINK" --host "$JETSON_HOST" \
      --jetson-cmd "$J" "$@"
}

fact() { awk -F= -v k="$1" '$1 == k { sub(/^[^=]*=/, ""); v = $0 } END { print v }' "$2"; }

# team_gate PURPOSE [--team-agreed] : refuse (exit 16) while the team uses the board (HIL_DESIGN.md 9.6)
team_gate() {
  local purpose="$1"; shift
  [ "$JETSON_LINK" = virtual ] && return 0
  local f; f=$(mktemp)
  collect_facts "$f" --only busy || { rm -f "$f"; jetson_die 14 "Could not read the Jetson's state through the login. Likely cause: the connection dropped. Fix: $J login, then try again"; }
  judge "$f" --rows busy --for "$purpose" "$@" >/dev/null 2>&1
  local rc=$?
  if [ "$rc" != 0 ]; then
    judge "$f" --rows busy --for "$purpose" "$@" >&2
    rm -f "$f"; exit "$rc"
  fi
  rm -f "$f"
}

clock_measure() {   # prints "offset_s=X rtt_ms=Y pc_time=Z" (best of 7 samples)
  [ "$JETSON_LINK" = virtual ] && { echo "offset_s=0.000 rtt_ms=0.0 pc_time=$(date +%s.%N)"; return 0; }
  local i t0 t1 r lines=""
  for i in 1 2 3 4 5 6 7; do
    t0=$(date +%s.%N); r=$(jetson_ssh 'date +%s.%N' 2>/dev/null) || return 1; t1=$(date +%s.%N)
    lines+="$t0 $r $t1"$'\n'
  done
  printf '%s' "$lines" | awk 'NF == 3 { rtt = $3 - $1; if (best == "" || rtt < best) { best = rtt; off = $2 - ($1 + $3) / 2; pc = $3 } }
       END { if (best == "") exit 1; printf "offset_s=%.3f rtt_ms=%.1f pc_time=%.3f\n", off, best * 1000, pc }'
}

# ---------------------------------------------------------------------------------------------------------------
# login / logout / find
# ---------------------------------------------------------------------------------------------------------------
cmd_login() {
  local here=0 wait_s=120
  while [ $# -gt 0 ]; do
    case "$1" in
      --here) here=1 ;;
      --wait) wait_s="${2:-}"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die_usage "login: unknown option '$1'" ;;
    esac
    shift
  done
  if [ "$JETSON_LINK" = virtual ]; then ok "The virtual Jetson needs no login."; return 0; fi
  if jetson_cm_check "$JETSON_CM"; then
    ok "already logged in (the login window for ${JETSON_DEST} is open). Nothing to do."
    jetson_save_target; return 0
  fi
  mkdir -p "$JETSON_STATE" && chmod 700 "$JETSON_STATE"
  cp "$JETSON_LIB_DIR/login_window.sh" "$JETSON_STATE/login_window.sh"       # a path without spaces for Windows
  local lw="$JETSON_STATE/login_window.sh"
  local args=("$JETSON_DEST" "$JETSON_PORT" "$JETSON_CM" "$JETSON_KNOWN_HOSTS" "$JETSON_STATE")
  if [ "$here" = 1 ]; then
    say "Logging in here (this terminal becomes the login window; use another terminal for the other commands)"
    exec bash "$lw" "${args[@]}"
  fi
  say "Opening the Jetson login window for ${JETSON_DEST} (port $JETSON_PORT)"
  if [ -n "${WSL_DISTRO_NAME:-}" ] && command -v powershell.exe >/dev/null 2>&1; then
    local a list="'-d','$WSL_DISTRO_NAME','--','bash'"
    for a in "$lw" "${args[@]}"; do list+=",'${a//\'/\'\'}'"; done
    powershell.exe -NoProfile -Command "Start-Process -FilePath wsl.exe -ArgumentList $list" >/dev/null 2>&1 \
      || jetson_die 1 "Could not open a new window through Windows (powershell.exe Start-Process failed). Likely cause: Windows interop is off in this WSL. Fix: run the login in this terminal: $J login --here"
  elif [ -n "${DISPLAY:-}" ] && command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash "$lw" "${args[@]}" >/dev/null 2>&1 &
  else
    info "No window system found. Open a second terminal and run there:"
    info "  bash $(q "$lw") ${args[*]}"
  fi
  info "Type the Jetson password in the new window (the first time also 'yes' for its key). Keep that window open."
  local t=0
  while [ "$t" -lt "$wait_s" ]; do
    if jetson_cm_check "$JETSON_CM"; then
      # the window may have asked for the user name: take it from login.env
      local _jt_file_USER="" _jt_file_HOST=""
      _jt_read_env "$JETSON_STATE/login.env"
      [ -n "$_jt_file_USER" ] && JETSON_USER="$_jt_file_USER"
      JETSON_DEST="${JETSON_USER:+$JETSON_USER@}$JETSON_HOST"
      jetson_resolve_data
      jetson_save_target
      ok "logged in to $JETSON_DEST (the window must stay open; closing it ends the access)"
      info "Next: $J check"
      return 0
    fi
    sleep 2; t=$((t + 2))
    [ $((t % 20)) = 0 ] && info "still waiting for the login in the window ($t s of $wait_s s)..."
  done
  jetson_die 17 "No login to $JETSON_DEST within $wait_s s. Likely cause: the password was not typed yet, it was wrong, or the window could not reach the Jetson (it says why). Fix: finish the login in the window, then run $J check (or $J login --wait 300)"
}

cmd_logout() {
  [ "$JETSON_LINK" = virtual ] && { ok "The virtual Jetson has no login."; return 0; }
  if jetson_cm_check "$JETSON_CM"; then
    "${JETSON_SSH_BIN:-ssh}" -S "$JETSON_CM" -O exit -o LogLevel=ERROR "$JETSON_DEST" >/dev/null 2>&1
    ok "logged out (the login window shows that the connection ended)"
  else
    info "not logged in; nothing to do"
  fi
  if [ "$JETSON_AUTH" = key ]; then
    local s; for s in "$JETSON_CM_A" "$JETSON_CM_B"; do
      "${JETSON_SSH_BIN:-ssh}" -S "$s" -O exit -o LogLevel=ERROR "$JETSON_DEST" >/dev/null 2>&1; done
  fi
}

cmd_find() {
  say "Looking for the Jetson (SSH port 22) on the usual addresses"
  local cands=() found=0 h
  local _jt_file_JETSON_HOST=""
  _jt_read_env "$JETSON_STATE/target.env"
  [ -n "$_jt_file_JETSON_HOST" ] && cands+=("$_jt_file_JETSON_HOST")
  cands+=("$JETSON_USB_HOST" jetson.local suas-jetson.local ubuntu.local suas-jetson)
  if command -v tailscale.exe >/dev/null 2>&1; then
    while read -r ip name _; do
      case "$name" in *jetson*|ubuntu) cands+=("$ip") ;; esac
    done < <(timeout 5 tailscale.exe status 2>/dev/null | tr -d '\r' | awk '$1 ~ /^100\./ {print $1, $2}')
  fi
  local seen=" "
  for h in "${cands[@]}"; do
    case "$seen" in *" $h "*) continue ;; esac; seen+="$h "
    if tcp_open "$h" "$JETSON_PORT" 3; then ok "$h answers on port $JETSON_PORT"; found=1
    else info "$h: no answer"; fi
  done
  if [ "$found" = 0 ]; then
    warn "No Jetson answered. Consequence: no command can reach it yet. Fix: USB-C: plug the DATA cable into the Jetson's USB-C port, power it (barrel jack), wait 1 minute, run this again. Wi-Fi/LAN: on the Jetson run  hostname -I  and use that address with --host."
    return 3
  fi
  info "Use one with: $J login --host HOST --user USER"
}

# ---------------------------------------------------------------------------------------------------------------
# check (HIL_DESIGN.md 9.2)
# ---------------------------------------------------------------------------------------------------------------
windows_usb_view() {   # prints a short read-only line about Windows' view of the USB network (empty if unknown)
  [ "$JETSON_LINK" = usb ] || return 0
  command -v powershell.exe >/dev/null 2>&1 || return 0
  local ps1="$JETSON_DIR/pc/windows_link_check.ps1"
  if [ -f "$ps1" ]; then   # the PC side's read-only Windows view (KEY=VALUE lines), joined on one line
    timeout 25 powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w "$ps1" 2>/dev/null || echo "$ps1")" \
        -Target "$JETSON_HOST" -Port "$JETSON_PORT" 2>/dev/null | tr -d '\r' | paste -sd' ' -
    return 0
  fi
  timeout 20 powershell.exe -NoProfile -Command \
    "\$a = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | Where-Object { \$_.IPAddress -like '192.168.55.*' } | Select-Object -First 1;
     if (\$a) { \$p = Test-Connection -Count 1 -Quiet 192.168.55.1; \"WINDOWS_USB_ADAPTER=\$(\$a.InterfaceAlias) WINDOWS_USB_IP=\$(\$a.IPAddress) WINDOWS_PING=\$p\" }
     else { \$n = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { \$_.FriendlyName -match 'NDIS|RNDIS|L4T' } | Select-Object -First 1;
            \"WINDOWS_USB_ADAPTER=none WINDOWS_NDIS_DEVICE=\$(\$n.FriendlyName) WINDOWS_NDIS_STATUS=\$(\$n.Status)\" }" 2>/dev/null | tr -d '\r'
}

check_forwards() {   # row 12: a real -R and -L through the login (exit 14 on failure)
  local purpose="$1" port lport
  port=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
  lport=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
  # PC side: answers "pong:<what it got>" once
  python3 -c "
import socket,sys
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); s.bind(('127.0.0.1',$port)); s.listen(1); s.settimeout(15)
try:
    c,_=s.accept(); c.settimeout(5); c.sendall(b'pong:'+c.recv(16)); c.close()
except Exception: pass" &
  local srv=$!
  sleep 0.3
  local sock="$JETSON_DATA/sock/check.sock" sockl="$JETSON_DATA/sock/check_l.sock" got="" ok_r=0 ok_l=0
  jetson_ssh "mkdir -p $(q "$JETSON_DATA/sock") && chmod 700 $(q "$JETSON_DATA/sock") && rm -f $(q "$sock") $(q "$sockl")"
  if jetson_forward add A -R "$sock:127.0.0.1:$port" >/dev/null 2>&1; then
    got=$(jetson_ssh "python3 -c \"import socket; u=socket.socket(socket.AF_UNIX); u.settimeout(5); u.connect('$sock'); u.sendall(b'ping'); print(u.recv(32).decode())\"" 2>/dev/null)
    [ "$got" = "pong:ping" ] && ok_r=1
    jetson_forward cancel A -R "$sock:127.0.0.1:$port" >/dev/null 2>&1
  fi
  kill "$srv" 2>/dev/null; wait "$srv" 2>/dev/null
  # -L: a socket served on the Jetson, read from the PC
  jetson_ssh "nohup python3 -c \"
import socket,os
p='$sockl'; s=socket.socket(socket.AF_UNIX); s.bind(p); s.listen(1); s.settimeout(15)
try:
    c,_=s.accept(); c.sendall(b'from-jetson'); c.close()
finally:
    os.unlink(p)\" >/dev/null 2>&1 &"
  sleep 0.5
  if jetson_forward add A -L "127.0.0.1:$lport:$sockl" >/dev/null 2>&1; then
    got=$(python3 -c "import socket; c=socket.create_connection(('127.0.0.1',$lport),timeout=5); print(c.recv(32).decode())" 2>/dev/null)
    [ "$got" = "from-jetson" ] && ok_l=1
    jetson_forward cancel A -L "127.0.0.1:$lport:$sockl" >/dev/null 2>&1
  fi
  jetson_ssh "rm -f $(q "$sock") $(q "$sockl")" >/dev/null 2>&1
  local ok_reg=1
  if [ "$purpose" = deploy ]; then
    ok_reg=0
    local rp="${REGISTRY_PORT:-$REGISTRY_PORT_DEFAULT}"
    if jetson_forward add A -R "127.0.0.1:$rp:127.0.0.1:$rp" >/dev/null 2>&1; then
      ok_reg=1; jetson_forward cancel A -R "127.0.0.1:$rp:127.0.0.1:$rp" >/dev/null 2>&1
    fi
  fi
  if [ "$ok_r" = 1 ] && [ "$ok_l" = 1 ] && [ "$ok_reg" = 1 ]; then
    ok "[ 12] SSH forwards allowed: a byte went PC -> Jetson -> PC through -R and back through -L$([ "$purpose" = deploy ] && echo ", -R 127.0.0.1:${REGISTRY_PORT:-$REGISTRY_PORT_DEFAULT} works")"
    return 0
  fi
  printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "SSH forwards (check row 12): the forward test failed (-R socket: $([ $ok_r = 1 ] && echo ok || echo FAILED), -L socket: $([ $ok_l = 1 ] && echo ok || echo FAILED)$([ "$purpose" = deploy ] && echo ", -R registry port: $([ $ok_reg = 1 ] && echo ok || echo FAILED)")). Likely cause: the Jetson's sshd does not allow forwarding (AllowStreamLocalForwarding no / AllowTcpForwarding no / DisableForwarding yes in /etc/ssh/sshd_config), which the simulator link needs. Fix: the owner/team decides; on the Jetson: sudo grep -Ei 'forwarding' /etc/ssh/sshd_config /etc/ssh/sshd_config.d/*; deploy can still use --via save")" >&2
  return 14
}

cmd_check() {
  local quick=0 deep=0 speed=0 pc=0 purpose=check team=() pipeline=new
  while [ $# -gt 0 ]; do
    case "$1" in
      --quick) quick=1 ;; --deep) deep=1 ;; --speed) speed=1 ;; --pc) pc=1 ;;
      --for) purpose="${2:-}"; shift ;;
      --team-agreed) team=(--team-agreed) ;;
      --pipeline) pipeline="${2:-}"; shift ;;
      --registry-port) REGISTRY_PORT="${2:-}"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die_usage "check: unknown option '$1'" ;;
    esac
    shift
  done
  case "$purpose" in check|deploy|warmup|flight|start) ;; *) die_usage "check: --for must be check, deploy, warmup or flight" ;; esac
  [ "$pc" = 1 ] && { check_pc; return; }
  local fail=0
  say "Checking the Jetson: link $JETSON_LINK, host $JETSON_HOST, port $JETSON_PORT, login mode $JETSON_AUTH"
  if [ "$JETSON_LINK" != virtual ]; then
    # row 1 + 2: link up, SSH port
    local pinged=0 tcp=0 wv=""
    ping -c 1 -W 2 "$JETSON_HOST" >/dev/null 2>&1 && pinged=1
    tcp_open "$JETSON_HOST" "$JETSON_PORT" 5 && tcp=1
    [ "$JETSON_LINK" = usb ] && wv=$(windows_usb_view)
    if [ "$pinged" = 0 ] && [ "$tcp" = 0 ]; then
      [ -n "$wv" ] && info "Windows sees: $wv"
      if [ "$JETSON_LINK" = usb ]; then
        case "$wv" in
          *WINDOWS_PING=True*|PING=ok*|*" PING=ok"*) jetson_die 3 "link (check row 1): Windows reaches 192.168.55.1 but this Ubuntu (WSL) does not. Likely cause: WSL's network (NAT) does not route to the USB network adapter (HIL_DESIGN.md risk 1). Fix: tell the lead (a Windows-side SSH fallback exists in the design); meanwhile use Ethernet or Wi-Fi: $J check --host ADDRESS" ;;
          *WINDOWS_USB_ADAPTER=none*|USB_ADAPTER=no*|*" USB_ADAPTER=no"*) jetson_die 3 "link (check row 1): no answer from 192.168.55.1 and Windows has NO USB network adapter with a 192.168.55.x address. Likely cause: the USB-C cable is not a data cable, is not in the Jetson's USB-C port, the Jetson is off or still starting, or Windows has no driver for it (Device Manager: 'Remote NDIS' under 'Other devices'). Fix: plug a DATA USB-C cable into the Jetson's USB-C port, power the Jetson from its barrel jack, wait 1 minute, run this again; driver steps: docs/jetson/README.md, 'Windows does not see the Jetson'" ;;
        esac
        jetson_die 3 "link (check row 1): no answer from $JETSON_HOST (no ping, no SSH port). Likely cause: the USB-C cable is not plugged in (or is a charge-only cable), the Jetson is off, or it is still starting. Fix: plug a DATA USB-C cable into the Jetson's USB-C port, power it from the barrel jack, wait 1 minute, then run this again. Without a cable: $J check --host ADDRESS (find it: $J find)"
      fi
      jetson_die 3 "link (check row 1): cannot reach $JETSON_HOST (no answer to ping or TCP $JETSON_PORT). Likely cause: wrong address, the Jetson is off or on another network, or the Wi-Fi blocks device-to-device traffic (campus Wi-Fi often does). Fix: on the Jetson run  hostname -I  and use that address; try $J find; on campus Wi-Fi use the USB-C cable, an Ethernet cable, a phone hotspot or Tailscale"
    fi
    ok "[  1] link up: $JETSON_HOST answers ($([ $pinged = 1 ] && echo "ping ok" || echo "no ping, but TCP works"))${wv:+; Windows: $wv}"
    [ "$tcp" = 1 ] || jetson_die 4 "SSH port (check row 2): $JETSON_HOST answers but its SSH port $JETSON_PORT is closed. Likely cause: the Jetson is still starting, or its SSH server is off. Fix: wait 1 minute and try again; if it stays closed, on the Jetson (screen + keyboard, or the USB serial console): sudo systemctl enable --now ssh"
    ok "[  2] SSH port $JETSON_PORT open"
    # row 3 + 4: login, host key (the window checks the key)
    if ! jetson_logged_in; then
      jetson_need_login
      info "(Rows 3-16 need the login. Nothing was changed on the Jetson.)"
      exit 17
    fi
    need_login
    ok "[  3] logged in ($JETSON_AUTH mode, ${JETSON_DEST}); [  4] host key checked by the login window ($JETSON_KNOWN_HOSTS)"
  fi
  # rows 5-11, 13-16: one SSH call, judged on the PC
  local f; f=$(mktemp)
  collect_facts "$f" || { rm -f "$f"; jetson_die 14 "Could not read the facts through the login (remote_facts.sh failed). Likely cause: the connection dropped. Fix: $J login, then run this again"; }
  [ -s "$f" ] || { rm -f "$f"; jetson_die 14 "The Jetson sent no facts. Likely cause: the login dropped while checking. Fix: $J login, then run this again"; }
  if [ "$quick" = 0 ] || [ "$purpose" = flight ]; then
    local cm; cm=$(clock_measure) && {
      printf 'CLOCK_OFFSET_S=%s\nCLOCK_RTT_MS=%s\nPC_TIME_UNIX=%s\n' \
        "$(sed -n 's/.*offset_s=\([^ ]*\).*/\1/p' <<<"$cm")" "$(sed -n 's/.*rtt_ms=\([^ ]*\).*/\1/p' <<<"$cm")" \
        "$(sed -n 's/.*pc_time=\([^ ]*\).*/\1/p' <<<"$cm")" >> "$f"; }
  fi
  cp "$f" "$JETSON_STATE/last_facts.env" 2>/dev/null
  judge "$f" --for "$purpose" --pc-git-sha "$(pc_git_sha)" --pc-code-sha "$(code_sha)" --pipeline "$pipeline" "${team[@]}" || fail=$?
  # row 12: forwards (needs the login; not for virtual)
  if [ "$JETSON_LINK" != virtual ] && [ "$quick" = 0 ]; then
    check_forwards "$purpose" || { [ "$fail" = 0 ] && fail=14; }
  fi
  if [ "$speed" = 1 ] && [ "$JETSON_LINK" != virtual ]; then
    local t0 t1; t0=$(date +%s.%N)
    head -c 52428800 /dev/zero | jetson_ssh 'cat > /dev/null' && t1=$(date +%s.%N) && \
      ok "[ --] link speed: $(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.0f Mbit/s (50 MB in %.1f s)", 400/(b-a), b-a }') through the login connection"
  fi
  if [ "$deep" = 1 ] && [ "$(fact HIL_IMAGE_ID "$f")" != "" ]; then
    say "Deep check: selftest inside the deployed image (GPU test)"
    cmd_selftest || { [ "$fail" = 0 ] && fail=9; }
  fi
  rm -f "$f"
  echo
  if [ "$fail" = 0 ]; then
    case "$purpose" in
      flight|start) ok "ALL GOOD: the Jetson at $JETSON_HOST is ready to fly: bash simulation/scripts/demo.sh runway --jetson $JETSON_LINK${JETSON_LINK:+$([ "$JETSON_LINK" = ssh ] && echo " --jetson-host $JETSON_HOST")}" ;;
      *) if [ "$JETSON_LINK" = virtual ]; then ok "ALL GOOD: the virtual Jetson on this PC is ready"
         else ok "ALL GOOD: the Jetson at $JETSON_HOST is ready for \"$J deploy\""; fi ;;
    esac
  else
    printf '\033[1;31m XX %s\033[0m\n' "$(_astra_tagged ERROR "check found a problem (exit $fail): see the XX lines above, each says what to do. Nothing was changed on the Jetson.")" >&2
  fi
  return "$fail"
}

check_pc() {   # check --pc : prerequisites for "image build" on this PC
  local rc=0
  say "Checking this PC for image builds"
  if ! command -v docker >/dev/null 2>&1 || ! docker info >/dev/null 2>&1; then
    jetson_die 18 "Docker Engine is not running in this Ubuntu (WSL). Likely cause: it is not installed, or stopped. Fix: bash simulation/docker/install_docker_engine.sh   (never Docker Desktop: its disk is on C:)"
  fi
  local os; os=$(docker info --format '{{.OperatingSystem}}' 2>/dev/null)
  case "$os" in *"Docker Desktop"*) jetson_die 18 "docker talks to Docker Desktop ($os), whose disk is on C: (it filled up before). Likely cause: the Docker Desktop WSL integration is on. Fix: use the Docker Engine inside Ubuntu: bash simulation/docker/install_docker_engine.sh, and turn off Docker Desktop's WSL integration" ;; esac
  ok "Docker Engine: $(docker version --format '{{.Server.Version}}' 2>/dev/null) ($os), root $(docker info --format '{{.DockerRootDir}}' 2>/dev/null)"
  if docker buildx version >/dev/null 2>&1; then ok "buildx: $(docker buildx version | awk '{print $2}')"
  else warn "buildx is missing. Consequence: images cannot be built. Fix: sudo apt-get install docker-buildx-plugin"; rc=18; fi
  if docker run --rm --platform linux/arm64 ubuntu:22.04 uname -m 2>/dev/null | grep -q aarch64; then ok "arm64 emulation works (needed for the Jetson image)"
  else warn "arm64 emulation does not work right now. Consequence: \"image build --flavor jetson\" registers it first (docker run --privileged --rm $BINFMT_IMAGE --install arm64). Permanent fix (owner's choice): sudo apt-get install qemu-user-static binfmt-support"; fi
  local free; free=$(df -BG --output=avail "$(docker info --format '{{.DockerRootDir}}' 2>/dev/null || echo /var/lib/docker)" 2>/dev/null | tail -1 | tr -dc 0-9)
  if [ -n "$free" ] && [ "$free" -lt 60 ]; then warn "only ${free} GB free for Docker. Consequence: the arm64 build (~60 GB with the registry copy) may not fit. Fix: free disk space"; else ok "disk for Docker: ${free:-?} GB free"; fi
  for i in astra-dev:latest astra-hil:virtual astra-dev:jetson astra-hil:jetson "$REGISTRY_IMAGE"; do
    if docker image inspect "$i" >/dev/null 2>&1; then info "image $i: present"; else info "image $i: not built yet"; fi
  done
  return "$rc"
}

# ---------------------------------------------------------------------------------------------------------------
# setup
# ---------------------------------------------------------------------------------------------------------------
cmd_setup() {
  local key=0 rmkey=0 yes=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --key) key=1 ;; --remove-key) rmkey=1 ;; --yes|-y) yes=1 ;;
      -h|--help) usage; exit 0 ;;
      *) die_usage "setup: unknown option '$1'" ;;
    esac
    shift
  done
  need_login
  say "Setting up $([ "$JETSON_LINK" = virtual ] && echo "the virtual Jetson on this PC" || echo "${JETSON_DEST}") (only our own folder $JETSON_DATA)"
  local D="$JETSON_DATA"
  jetson_ssh "umask 077; mkdir -p $(q "$D")/sock $(q "$D")/run $(q "$D")/models $(q "$D")/photos $(q "$D")/detections $(q "$D")/photo_archive $(q "$D")/runs $(q "$D")/logs $(q "$D")/bin && chmod 700 $(q "$D")/sock && chmod 755 $(q "$D") $(q "$D")/photos $(q "$D")/detections $(q "$D")/runs $(q "$D")/models" \
    || jetson_die 1 "Could not create $D on the Jetson. Likely cause: the home folder is full or not writable. Fix: look on the board: df -h ~ ; ls -ld ~"
  ok "created $D/{sock,run,models,photos,detections,photo_archive,runs,logs,bin}"
  jetson_ssh "cat > $(q "$D/bin/astra_hil.sh") && chmod 755 $(q "$D/bin/astra_hil.sh")" < "$JETSON_LIB_DIR/astra_hil.sh" \
    && ok "installed $D/bin/astra_hil.sh (start/stop REAL mode on the drone without the PC)"
  local f; f=$(mktemp)
  collect_facts "$f"
  cp "$f" "$JETSON_STATE/board_facts.env" 2>/dev/null
  local doms acc user; doms=$(fact ROS_DOMAINS "$f"); acc=$(fact DOCKER_ACCESS "$f"); user=$(fact USER "$f")
  [ -n "$user" ] && JETSON_USER="$user"
  info "ROS domains in use on the board right now: ${doms:-none seen}. Our SIM container uses domain 77 (sealed, ROS_LOCALHOST_ONLY=1)."
  case ",$doms," in *,77,*) warn "domain 77 is in use on the board. Consequence: none for SIM (its network is sealed), but pick another for clarity: start --domain N" ;; esac
  if [ "$acc" = none ] && [ "$JETSON_LINK" != virtual ]; then
    warn "user $user cannot use Docker without a password. Consequence: the kit cannot run containers. Fix (the owner/team decides: the docker group is root-equivalent), typed by the owner in the login window:"
    warn_more "sudo usermod -aG docker $user      then close the login window and run: $J login"
  fi
  rm -f "$f"
  if [ "$key" = 1 ] && [ "$JETSON_LINK" != virtual ]; then
    if [ "$yes" = 0 ]; then
      echo "Key mode adds ONE line to ~/.ssh/authorized_keys of $JETSON_DEST (the team's account). The owner and the team must agree."
      read -r -p "Add the key now? [y/N] " a; case "$a" in y|Y|yes) ;; *) info "not added"; key=0 ;; esac
    fi
    if [ "$key" = 1 ]; then
      [ -f "$JETSON_KEY" ] || ssh-keygen -q -t ed25519 -N '' -C "astra-hil-kit@$(hostname)" -f "$JETSON_KEY"
      jetson_ssh "umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; grep -qxF $(q "$(cat "$JETSON_KEY.pub")") ~/.ssh/authorized_keys || echo $(q "$(cat "$JETSON_KEY.pub")") >> ~/.ssh/authorized_keys" \
        && ok "key added (comment astra-hil-kit@...); key mode: add --auth key, or it is saved now" && JETSON_AUTH=key
    fi
  fi
  if [ "$rmkey" = 1 ] && [ "$JETSON_LINK" != virtual ]; then
    jetson_ssh "[ -f ~/.ssh/authorized_keys ] && sed -i '/ astra-hil-kit@/d' ~/.ssh/authorized_keys" && ok "our key line was removed from the Jetson"
    JETSON_AUTH=login
  fi
  if [ "$JETSON_LINK" = virtual ]; then
    info "the saved board target ($JETSON_STATE/target.env) is left as it is: the virtual Jetson is always chosen with --link virtual"
    info "Next: $J image build --flavor virtual   then $J deploy --link virtual"
    return 0
  fi
  jetson_save_target
  ok "saved the target in $JETSON_STATE/target.env (link $JETSON_LINK, host $JETSON_HOST, user ${JETSON_USER:-?}, auth $JETSON_AUTH)"
  info "Next: $J check   then (on the PC) $J image build   then $J deploy"
}

# ---------------------------------------------------------------------------------------------------------------
# clock / status / paths / target
# ---------------------------------------------------------------------------------------------------------------
cmd_clock() {
  need_login
  local cm; cm=$(clock_measure) || jetson_die 14 "Could not read the Jetson's clock. Likely cause: the login dropped. Fix: $J login"
  echo "${cm% pc_time=*}"
}

cmd_status() {
  local json=0
  while [ $# -gt 0 ]; do case "$1" in --json) json=1 ;; -h|--help) usage; exit 0 ;; *) die_usage "status: unknown option '$1'" ;; esac; shift; done
  local logged=false rc=0
  if jetson_logged_in; then logged=true; else rc=17; fi
  local f; f=$(mktemp)
  if [ "$logged" = true ]; then
    jetson_resolve_data
    collect_facts "$f"
    local run_id uptime
    run_id=$(jetson_ssh "$DKPRE dk inspect -f '{{index .Config.Labels \"org.astra.run_id\"}}' $(q "$JETSON_CONTAINER") 2>/dev/null")
    uptime=$(jetson_ssh "$DKPRE s=\$(dk inspect -f '{{.State.StartedAt}}' $(q "$JETSON_CONTAINER") 2>/dev/null) && [ -n \"\$s\" ] && echo \$(( \$(date +%s) - \$(date -d \"\$s\" +%s) ))" 2>/dev/null)
    printf 'RUN_ID=%s\nUPTIME_S=%s\n' "$run_id" "$uptime" >> "$f"
  fi
  if [ "$json" = 1 ]; then
    local base="{}"
    [ "$logged" = true ] && base=$(python3 "$JETSON_LIB_DIR/jetson_facts.py" status-json --facts "$f" --link "$JETSON_LINK")
    python3 - "$base" "$JETSON_LINK" "$JETSON_AUTH" "$logged" "$(fact RUN_ID "$f" 2>/dev/null)" "$(fact UPTIME_S "$f" 2>/dev/null)" <<'PY'
import json, sys
base = json.loads(sys.argv[1])
out = {"link": sys.argv[2], "auth": sys.argv[3], "logged_in": sys.argv[4] == "true",
       "container": "missing", "image": "", "git_sha": "", "run_id": sys.argv[5], "uptime_s": None,
       "l4t": "", "power_mode": "", "clocks": "", "engine": "", "team_busy": None, "mem_avail_mb": None,
       "disk_free_gb": None}
out.update(base)
try:
    out["uptime_s"] = int(sys.argv[6])
except ValueError:
    pass
print(json.dumps(out))
PY
  else
    say "Jetson status ($JETSON_LINK, $JETSON_HOST)"
    if [ "$logged" = true ]; then
      info "logged in:   yes ($JETSON_AUTH mode, $JETSON_DEST)"
      info "container:   $(fact HIL_CONTAINER "$f" || true)${JETSON_CONTAINER:+ ($JETSON_CONTAINER)}  run: $(fact RUN_ID "$f")"
      info "image:       $JETSON_IMAGE git $(fact HIL_IMAGE_SHA "$f") built $(fact HIL_IMAGE_BUILT "$f")"
      info "engine:      $(fact ENGINE_PRESENT "$f") (L4T $(fact ENGINE_L4T "$f"), TensorRT $(fact ENGINE_TRT "$f"))"
      info "board:       L4T $(fact L4T "$f"), power mode $(fact NVPMODEL_MODE "$f"), clocks $(fact CLOCKS "$f")"
      info "memory/disk: $(fact MEM_AVAIL_MB "$f") MB free, $(fact DISK_DOCKER_FREE_GB "$f") GB free"
      local tp; tp=$(fact TEAM_PROCS "$f"); info "team:        ${tp:-no team programs seen}"
    else
      jetson_need_login || true
    fi
  fi
  rm -f "$f"
  return "$rc"
}

cmd_paths() {
  need_login
  printf 'JETSON_DATA=%s\nSOCK_DIR=%s/sock\nPHOTOS=%s/photos\nDETECTIONS=%s/detections\nRUNS=%s/runs\nMODELS=%s/models\n' \
    "$JETSON_DATA" "$JETSON_DATA" "$JETSON_DATA" "$JETSON_DATA" "$JETSON_DATA" "$JETSON_DATA"
}

cmd_target() {
  local fmt=env
  while [ $# -gt 0 ]; do case "$1" in --format) fmt="${2:-}"; shift ;; --format=*) fmt="${1#*=}" ;; *) die_usage "target: unknown option '$1'" ;; esac; shift; done
  export JETSON_LINK JETSON_HOST JETSON_USER JETSON_PORT JETSON_AUTH JETSON_DEST JETSON_CM JETSON_CM_A JETSON_CM_B \
         JETSON_KEY JETSON_KNOWN_HOSTS JETSON_STATE JETSON_DATA JETSON_CONTAINER JETSON_IMAGE
  jetson_target_print "$fmt"
}

# ---------------------------------------------------------------------------------------------------------------
# image build / status / prune (on THIS PC; heavy job: both locks)
# ---------------------------------------------------------------------------------------------------------------
take_heavy_locks() {
  [ "${ASTRA_LOCKS_HELD:-0}" = 1 ] && return 0
  [ "${DRY:-0}" = 1 ] && return 0
  info "waiting for the heavy-job locks (one simulator or image build at a time on this PC)..."
  exec 8>/mnt/wsl/astra_heavy.lock 2>/dev/null || exec 8>/tmp/astra_heavy.lock
  flock 8
  exec 9>/tmp/astra_sim.lock; flock 9
  export ASTRA_LOCKS_HELD=1
}

run() {   # run CMD... (prints it; with --dry-run only prints)
  printf '    $ %s\n' "$*"
  [ "${DRY:-0}" = 1 ] && return 0
  "$@"
}

stage_context() {   # stage_context DIR FLAVOR : copy the repo files listed in image/context.list into DIR
  local ctx="$1" flavor="$2"
  rm -rf "$ctx"; mkdir -p "$ctx/repo" "$ctx/opt" "$ctx/extra"
  local incl=() excl=() line
  while IFS= read -r line || [ -n "$line" ]; do
    line="${line%%#*}"; line="$(echo "$line" | sed 's/[[:space:]]*$//')"
    [ -n "$line" ] || continue
    case "$line" in -*) excl+=(--exclude "${line#- }") ;; *) incl+=("$line") ;; esac
  done < "$JETSON_DIR/image/context.list"
  local p
  for p in "${incl[@]}"; do
    [ -e "$JETSON_REPO/$p" ] || { warn "context.list names $p, which is not in the repo. Consequence: the image will miss it. Fix: correct simulation/jetson/image/context.list"; continue; }
    mkdir -p "$ctx/repo/$(dirname "$p")"
    rsync -a "${excl[@]}" "$JETSON_REPO/$p" "$ctx/repo/$(dirname "$p")/"
  done
  cp "$JETSON_DIR/image/"{entrypoint.sh,in_stack.sh,selftest.py,warmup.py} "$ctx/opt/"
  cp "$JETSON_LIB_DIR/hil_relay.py" "$ctx/opt/"
  if [ -f "$JETSON_DIR/pc/ros_tap.py" ]; then cp "$JETSON_DIR/pc/ros_tap.py" "$JETSON_DIR/pc/tap_topics.txt" "$ctx/opt/" 2>/dev/null
  else warn "simulation/jetson/pc/ros_tap.py is not there yet (PC-side builder). Consequence: this image has no ROS tap sender; the entrypoint says so and the PC sees MAVLink only. Fix: rebuild the image once it exists"; fi
  # the new video pipeline (owner 11:56-12:10): the proposed field launch file + the team's zr10 packages, if present
  [ -d "$JETSON_REPO/simulation/video/proposals" ] && cp -r "$JETSON_REPO/simulation/video/proposals" "$ctx/extra/proposals"
  if [ -n "${ASTRA_NEW_PIPELINE_SRC:-}" ] && [ -d "$ASTRA_NEW_PIPELINE_SRC" ]; then
    mkdir -p "$ctx/extra/new_pipeline"
    for p in zr10_ros2 zr10_stream; do
      [ -d "$ASTRA_NEW_PIPELINE_SRC/$p" ] && rsync -a --exclude build --exclude install --exclude log "$ASTRA_NEW_PIPELINE_SRC/$p" "$ctx/extra/new_pipeline/"
    done
  fi
  printf 'GIT_SHA=%s\nCODE_SHA=%s\nFLAVOR=%s\nBUILT=%s\n' "$(pc_git_sha)" "$(code_sha)" "$flavor" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$ctx/opt/image.env"
}

# code_sha : a 12-character fingerprint of exactly what goes INTO the image (the files of image/context.list minus its
# excludes, the /opt/astra files and image/Dockerfile), from the file CONTENTS only. The git commit alone cannot say
# whether an image is current (the working tree may hold uncommitted changes, and the Linux copy that demo.sh runs
# from has no .git), so `image build` stores this in the label org.astra.code_sha and `check` compares it with the
# code on this PC: the Windows folder and its Linux copy (03_build.sh mirrors it) give the same value.
code_sha() {
  local incl=() prune=() line p
  while IFS= read -r line || [ -n "$line" ]; do
    line="${line%%#*}"; line="$(echo "$line" | sed 's/[[:space:]]*$//')"
    [ -n "$line" ] || continue
    case "$line" in
      -*) p="${line#- }"
          case "$p" in
            */\*) prune+=(-o -path "*/${p%/\*}/*") ;;     # "folder/*": the folder's contents
            */)   prune+=(-o -path "*/${p%/}") ;;          # "folder/": the folder
            *)    prune+=(-o -name "$p") ;;               # a name pattern (files or folders)
          esac ;;
      *) incl+=("$line") ;;
    esac
  done < "$JETSON_DIR/image/context.list"
  (
    cd "$JETSON_REPO" || exit 1
    {
      for p in "${incl[@]}"; do
        [ -e "$p" ] || continue
        find "$p" \( -false "${prune[@]}" \) -prune -o -type f -print
      done
      cd "$JETSON_DIR" && printf '%s\n' image/Dockerfile image/entrypoint.sh image/in_stack.sh image/selftest.py \
        image/warmup.py lib/hil_relay.py pc/ros_tap.py pc/tap_topics.txt | while read -r f; do [ -f "$f" ] && echo "@jetson/$f"; done
    } | LC_ALL=C sort | while IFS= read -r f; do
      case "$f" in @jetson/*) (cd "$JETSON_DIR" && sha256sum "${f#@jetson/}") ;; *) sha256sum "$f" ;; esac
    done | sha256sum | cut -c1-12
  )
}

ensure_arm64_emulation() {
  if docker run --rm --platform linux/arm64 ubuntu:22.04 uname -m 2>/dev/null | grep -q aarch64; then
    ok "arm64 emulation works"; return 0
  fi
  say "arm64 emulation is not registered in this WSL session: registering it ($BINFMT_IMAGE, lasts until WSL restarts)"
  run docker run --privileged --rm "$BINFMT_IMAGE" --install arm64 >/dev/null || true
  [ "${DRY:-0}" = 1 ] && return 0
  docker run --rm --platform linux/arm64 ubuntu:22.04 uname -m 2>/dev/null | grep -q aarch64 \
    || jetson_die 18 "arm64 emulation still does not work. Likely cause: the binfmt registration was refused (no --privileged), or QEMU is missing. Fix (permanent, owner's choice): sudo apt-get install qemu-user-static binfmt-support   then run this again"
  ok "arm64 emulation registered"
}

ensure_registry() {   # the small image store on the PC, only on 127.0.0.1
  local port="${1:-$REGISTRY_PORT_DEFAULT}"
  if docker ps --format '{{.Names}}' | grep -qx "$REGISTRY_NAME"; then return 0; fi
  docker rm -f "$REGISTRY_NAME" >/dev/null 2>&1
  run docker run -d --name "$REGISTRY_NAME" --label "$LABEL" --restart unless-stopped -p "127.0.0.1:$port:5000" \
      -v astra-hil-registry:/var/lib/registry "$REGISTRY_IMAGE" >/dev/null \
    || jetson_die 18 "Could not start the image store ($REGISTRY_IMAGE) on 127.0.0.1:$port. Likely cause: the port is in use, or the image could not be downloaded. Fix: ss -ltn | grep $port ; or use --registry-port N"
}

cmd_image() {
  local sub="${1:-}"; shift || true
  case "$sub" in
    build) image_build "$@" ;;
    status) image_status ;;
    prune) image_prune "$@" ;;
    *) die_usage "image: use  image build | image status | image prune" ;;
  esac
}

image_build() {
  local flavor="" nocache=() l4t="$L4T_TAG_DEFAULT" x86base="${JETSON_X86_BASE:-trt}"
  DRY=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --flavor) flavor="${2:-}"; shift ;;
      --x86-base) x86base="${2:-}"; shift ;;
      --no-cache) nocache=(--no-cache) ;;
      --dry-run) DRY=1 ;;
      --l4t-tag) l4t="${2:-}"; shift ;;
      -h|--help) usage; exit 0 ;;
      *) die_usage "image build: unknown option '$1'" ;;
    esac
    shift
  done
  [ -n "$flavor" ] || { [ "$JETSON_LINK" = virtual ] && flavor=virtual || flavor=jetson; }
  case "$flavor" in jetson|virtual) ;; *) die_usage "image build: --flavor must be jetson or virtual" ;; esac
  case "$x86base" in trt|ros) ;; *) die_usage "image build: --x86-base must be trt (the Jetson's CUDA/TensorRT versions, default) or ros (plain ROS image)" ;; esac
  if [ "$DRY" = 0 ]; then
    command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1 || jetson_die 18 "Docker Engine is not running in this Ubuntu (WSL). Likely cause: not installed or stopped. Fix: bash simulation/docker/install_docker_engine.sh"
    docker buildx version >/dev/null 2>&1 || jetson_die 18 "Docker's build tool buildx is missing. Fix: sudo apt-get install docker-buildx-plugin"
  fi
  take_heavy_locks
  local sha csha; sha=$(pc_git_sha); csha=$(code_sha)
  local ctx="$JETSON_STATE/build_ctx/$flavor"
  export DOCKER_BUILDKIT=1
  if [ "$flavor" = virtual ]; then
    say "Image astra-hil:virtual (the virtual Jetson, x86, PC GPU) from git $sha, x86 base: $x86base"
    local dev_image=astra-dev:latest
    if [ "$x86base" = trt ]; then
      # owner 12:32: the PC container = the Jetson's software version for version (CUDA 12.6, cuDNN 9.3, TensorRT 10.3
      # on Ubuntu 22.04), so layer 1 is NVIDIA's TensorRT container instead of the plain ROS image
      say "Layer 1: $TRT_X86_BASE + ROS 2 Humble (x86, the Jetson's CUDA / TensorRT versions)"
      run docker build "${nocache[@]}" -f "$JETSON_DIR/image/Dockerfile.l4t-ros" --build-arg BASE_IMAGE="$TRT_X86_BASE" \
          --build-arg PURGE_NVIDIA_OPENCV=no -t astra-hil-trt-ros:24.08 "$JETSON_DIR/image" \
        || jetson_die 1 "Layer 1 (astra-hil-trt-ros:24.08) did not build. Likely cause: nvcr.io or packages.ros.org could not be reached. Fix: read the error above; or build on the plain ROS base: --x86-base ros"
      say "Layer 2: the team's dev stack (docker/Dockerfile target dev) on top, torch for CUDA 12.6"
      run docker build "${nocache[@]}" --target dev -f "$JETSON_REPO/docker/Dockerfile" \
          --build-arg BASE_IMAGE=astra-hil-trt-ros:24.08 --build-arg TORCH_INDEX_URL=https://download.pytorch.org/whl/cu126 \
          --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" --label org.astra.owner=hil \
          --label org.astra.flavor=virtual -t astra-dev:hil-x86 "$JETSON_REPO/docker" \
        || jetson_die 1 "Layer 2 (astra-dev:hil-x86) did not build. Fix: read the error above; or --x86-base ros"
      dev_image=astra-dev:hil-x86
    elif ! docker image inspect astra-dev:latest >/dev/null 2>&1 && [ "$DRY" = 0 ]; then
      say "Layer 2 (astra-dev:latest) is missing: building it with the existing build_images.sh"
      run bash "$JETSON_REPO/simulation/docker/build_images.sh" dev --torch auto || jetson_die 1 "Layer 2 (astra-dev:latest) did not build. Likely cause: a download failed. Fix: look at ~/.astra_docker/logs/, then run this again"
    fi
    say "Layer 3: the team stack built in"
    [ "$DRY" = 1 ] || stage_context "$ctx" virtual
    # TensorRT in the image: yes on the version-matched base (the Jetson's TensorRT 10.3), no on the plain ROS base
    local trt=no; [ "$x86base" = trt ] && trt=yes
    run docker build "${nocache[@]}" -f "$JETSON_DIR/image/Dockerfile" --build-arg DEV_IMAGE="$dev_image" \
        --build-arg FLAVOR=virtual --build-arg GIT_SHA="$sha" --build-arg CODE_SHA="$csha" --build-arg TENSORRT="$trt" \
        --label "org.astra.git_sha=$sha" --label "org.astra.code_sha=$csha" --label "org.astra.built=$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
        -t astra-hil:virtual -t "astra-hil:virtual-$csha" "$ctx" \
      || jetson_die 1 "Layer 3 (astra-hil:virtual) did not build. Likely cause: a team package does not build (colcon). Fix: read the error above; the build context is $ctx"
    ok "built astra-hil:virtual (also astra-hil:virtual-$csha; code $csha, git $sha). Next: $J deploy --link virtual"
    return 0
  fi
  say "Image astra-hil:jetson (arm64, for the Jetson) built ON THIS PC from git $sha"
  [ "$DRY" = 1 ] || ensure_arm64_emulation
  say "Layer 1: L4T JetPack $l4t + ROS 2 Humble (arm64)"
  run docker build "${nocache[@]}" --platform linux/arm64 -f "$JETSON_DIR/image/Dockerfile.l4t-ros" \
      --build-arg L4T_TAG="$l4t" -t "astra-hil-l4t-ros:$l4t" "$JETSON_DIR/image" \
    || jetson_die 1 "Layer 1 did not build. Likely cause: a download (nvcr.io / packages.ros.org) failed, or the OpenCV check in it failed. Fix: read the error above"
  say "Layer 2: the team's dev stack (docker/Dockerfile target dev) on top, Jetson torch"
  # shellcheck disable=SC1091
  local pins="$JETSON_DIR/image/jetson_pins.env" tidx="https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/"
  # shellcheck source=image/jetson_pins.env
  [ -f "$pins" ] && { . "$pins"; tidx="${TORCH_INDEX_URL:-$tidx}"; }
  run docker build "${nocache[@]}" --platform linux/arm64 --target dev -f "$JETSON_REPO/docker/Dockerfile" \
      --build-arg BASE_IMAGE="astra-hil-l4t-ros:$l4t" --build-arg TORCH_INDEX_URL="$tidx" \
      --build-arg TORCH_VERSION="${TORCH_VERSION:-2.8.0}" --build-arg TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.23.0}" \
      --build-arg TORCH_SHA256="${TORCH_SHA256:-}" --build-arg TORCHVISION_SHA256="${TORCHVISION_SHA256:-}" \
      --build-arg USER_UID=1000 --build-arg USER_GID=1000 \
      --label org.astra.flavor=jetson --label "org.astra.git_sha=$sha" --label "org.astra.built=$(date -u +%Y-%m-%dT%H:%M:%SZ)" -t astra-dev:jetson "$JETSON_REPO/docker" \
    || jetson_die 1 "Layer 2 (astra-dev:jetson) did not build. Likely cause: an arm64 package or the Jetson torch wheel failed. Fix: read the error above"
  say "Layer 3: the team stack built in"
  [ "$DRY" = 1 ] || stage_context "$ctx" jetson
  run docker build "${nocache[@]}" --platform linux/arm64 -f "$JETSON_DIR/image/Dockerfile" \
      --build-arg DEV_IMAGE=astra-dev:jetson --build-arg FLAVOR=jetson --build-arg GIT_SHA="$sha" \
      --build-arg CODE_SHA="$csha" --build-arg TENSORRT=yes \
      --label "org.astra.git_sha=$sha" --label "org.astra.code_sha=$csha" --label "org.astra.built=$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
      -t astra-hil:jetson -t "astra-hil:jetson-$csha" "$ctx" \
    || jetson_die 1 "Layer 3 (astra-hil:jetson) did not build. Fix: read the error above; the build context is $ctx"
  say "Into the image store on the PC (127.0.0.1:$REGISTRY_PORT_DEFAULT), for deploy"
  ensure_registry "$REGISTRY_PORT_DEFAULT"
  run docker tag "astra-hil:jetson-$csha" "127.0.0.1:$REGISTRY_PORT_DEFAULT/astra-hil:jetson-$csha"
  run docker push "127.0.0.1:$REGISTRY_PORT_DEFAULT/astra-hil:jetson-$csha" \
    || jetson_die 18 "Could not push into the image store on 127.0.0.1:$REGISTRY_PORT_DEFAULT. Fix: docker logs $REGISTRY_NAME"
  ok "built astra-hil:jetson (code $csha, git $sha) and stored it for deploy. Next: $J deploy"
}

image_status() {
  say "Images on this PC (ours carry the label $LABEL)"
  docker images --filter "label=$LABEL" --format '    {{.Repository}}:{{.Tag}}  {{.Size}}  {{.CreatedSince}}' 2>/dev/null
  for i in astra-dev:latest astra-dev:jetson; do docker image inspect "$i" >/dev/null 2>&1 && info "$i (layer 2): present"; done
  if [ "$JETSON_LINK" != virtual ] && jetson_logged_in; then
    jetson_resolve_data
    say "Images on the Jetson (ours only)"
    jetson_ssh "$DKPRE dk images --filter label=$LABEL --format '    {{.Repository}}:{{.Tag}}  {{.Size}}  {{.CreatedSince}}'"
  fi
}

image_prune() {   # removes OUR old images only (label org.astra.owner=hil), never docker system prune
  local where="${1:---jetson}"
  if [ "$JETSON_LINK" = virtual ] || [ "$where" = --pc ]; then
    say "Removing our old images on this PC (not the current astra-hil:virtual / astra-hil:jetson)"
    docker images --filter "label=$LABEL" --format '{{.Repository}}:{{.Tag}}' | grep -Ev ':(virtual|jetson)$' | xargs -r docker rmi
    return 0
  fi
  need_login
  say "Removing our old images on the Jetson (label $LABEL; the current astra-hil:jetson stays; nothing of the team's)"
  jetson_ssh "$DKPRE dk images --filter label=$LABEL --format '{{.Repository}}:{{.Tag}}' | grep -Ev '^astra-hil:jetson\$' | xargs -r \$DK rmi; dk image prune -f --filter label=$LABEL >/dev/null"
  ok "done"
}

# ---------------------------------------------------------------------------------------------------------------
# deploy / selftest / warmup
# ---------------------------------------------------------------------------------------------------------------
gpu_flag() { if [ "$JETSON_LINK" = virtual ]; then echo "--gpus all"; else echo "--runtime nvidia"; fi; }

cmd_selftest() {
  local D="$JETSON_DATA"
  jetson_ssh "$DKPRE dk run --rm --init --label $LABEL $(gpu_flag) --network none -e ASTRA_UID=\$(id -u) -e ASTRA_GID=\$(id -g) -v $(q "$D/models"):/models $(q "$JETSON_IMAGE") selftest"
}

cmd_deploy() {
  local via=registry model="" rport="$REGISTRY_PORT_DEFAULT" team=()
  DRY=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --via) via="${2:-}"; shift ;;
      --model) model="${2:-}"; shift ;;
      --registry-port) rport="${2:-}"; shift ;;
      --team-agreed) team=(--team-agreed) ;;
      --dry-run) DRY=1 ;;
      -h|--help) usage; exit 0 ;;
      *) die_usage "deploy: unknown option '$1'" ;;
    esac
    shift
  done
  case "$via" in registry|save) ;; build-on-board) jetson_die 2 "deploy --via build-on-board is the last resort of HIL_DESIGN.md 9.5 and is not built in this version. Fix: use --via registry (default) or --via save" ;; *) die_usage "deploy: --via must be registry or save" ;; esac
  [ -z "$model" ] || [ -f "$model" ] || jetson_die 2 "deploy: the model file $model does not exist. Fix: give an existing .pt file, or leave --model out"
  need_login
  local sha; sha=$(pc_git_sha)
  local D="$JETSON_DATA"
  if [ "$JETSON_LINK" != virtual ]; then
    [ "$DRY" = 1 ] || team_gate deploy "${team[@]}"
    # disk reserve (row 9 with the deploy size)
    local f; f=$(mktemp); collect_facts "$f"
    if [ "$DRY" = 0 ]; then
      judge "$f" --for deploy "${team[@]}" > "$f.out" 2>&1; local rc=$?
      if grep -q 'XX .*\[  9\]\|XX .*\[ 10\]' "$f.out"; then grep 'XX' "$f.out" >&2; rm -f "$f" "$f.out"; exit "$rc"; fi
      if grep -q 'XX .*\[  6\]\|XX .*\[  7\]' "$f.out"; then grep 'XX' "$f.out" >&2; rm -f "$f" "$f.out"; exit "$rc"; fi
      rm -f "$f.out"
    fi
    rm -f "$f"
    docker image inspect astra-hil:jetson >/dev/null 2>&1 || [ "$DRY" = 1 ] || jetson_die 12 "There is no astra-hil:jetson image on this PC yet. Likely cause: the image was never built. Fix: $J image build   (on this PC, first time 1-2 h), then $J deploy"
    local isha icode; isha=$(docker image inspect -f '{{index .Config.Labels "org.astra.code_sha"}}' astra-hil:jetson 2>/dev/null)
    icode=$(code_sha)
    [ -z "$isha" ] || [ "$isha" = "$icode" ] || warn "the image on this PC was built from code $isha, the code here is $icode (the team packages or the image files changed since). Consequence: the Jetson gets the older code. Fix: $J image build first"
    jetson_ssh "mkdir -p $(q "$D")/models $(q "$D")/sock"
    if [ "$via" = registry ]; then
      say "Moving the image through the login (only changed layers move; the Jetson needs no internet)"
      [ "$DRY" = 1 ] || ensure_registry "$rport"
      local tag="127.0.0.1:$rport/astra-hil:jetson-${isha:-$sha}"
      run docker tag astra-hil:jetson "$tag"
      run docker push "$tag" >/dev/null || jetson_die 18 "Could not put the image into the store on the PC. Fix: docker logs $REGISTRY_NAME"
      printf '    $ %s\n' "ssh -O forward -R 127.0.0.1:$rport:127.0.0.1:$rport (through the login)"
      if [ "$DRY" = 0 ]; then
        jetson_forward add A -R "127.0.0.1:$rport:127.0.0.1:$rport" >/dev/null 2>&1 \
          || jetson_die 14 "Could not open the registry door (-R 127.0.0.1:$rport) through the login. Likely cause: port $rport is in use on the Jetson, or sshd forbids forwarding. Fix: $J deploy --registry-port N, or $J deploy --via save"
        trap 'jetson_forward cancel A -R "127.0.0.1:'"$rport"':127.0.0.1:'"$rport"'" >/dev/null 2>&1' EXIT
      fi
      run jetson_ssh "$DKPRE dk pull $(q "$tag") && dk tag $(q "$tag") astra-hil:jetson && dk tag $(q "$tag") astra-hil:jetson-${isha:-$sha} && dk rmi $(q "$tag") >/dev/null" \
        || jetson_die 1 "The Jetson could not pull the image from the PC. Likely cause: the login dropped, or the Jetson's disk is full. Fix: $J check --for deploy, then try again, or $J deploy --via save"
      [ "$DRY" = 1 ] || { jetson_forward cancel A -R "127.0.0.1:$rport:127.0.0.1:$rport" >/dev/null 2>&1; trap - EXIT; }
    else
      say "Copying the whole image through the login (docker save | gzip | docker load)"
      if [ "$DRY" = 1 ]; then printf '    $ %s\n' "docker save astra-hil:jetson | gzip -1 | ssh ... 'gunzip | docker load'"
      else docker save astra-hil:jetson | gzip -1 | jetson_ssh "$DKPRE gunzip | dk load" \
          || jetson_die 1 "Copying the image failed. Likely cause: the login dropped or the disk is full. Fix: $J check --for deploy"
      fi
    fi
    ok "the image is on the Jetson"
  else
    say "Virtual Jetson: the image stays on this PC"
    mkdir -p "$D/models" "$D/sock" "$D/runs" "$D/photos" "$D/detections"
    docker image inspect astra-hil:virtual >/dev/null 2>&1 || [ "$DRY" = 1 ] || jetson_die 12 "There is no astra-hil:virtual image yet. Fix: $J image build --flavor virtual"
  fi
  # the model weights
  say "Model weights (yolo26m.pt)"
  local team_model; team_model="$(jetson_ssh "[ -s ~/astra/ros2_ws/src/yolo26m.pt ] && echo ~/astra/ros2_ws/src/yolo26m.pt" 2>/dev/null)"
  if [ -n "$model" ]; then
    run jetson_rsync -t "$model" ":$D/models/yolo26m.pt" && ok "copied $model"
  elif [ "$JETSON_LINK" != virtual ] && [ -n "$team_model" ]; then
    run jetson_ssh "cp -p $(q "$team_model") $(q "$D/models/yolo26m.pt")" && ok "copied the team's own file $team_model (read-only from their folder)"
  elif [ -s "${ASTRA_SIM_HOME:-$HOME/.astra_sim}/models/yolo26m.pt" ]; then
    run jetson_rsync -t "${ASTRA_SIM_HOME:-$HOME/.astra_sim}/models/yolo26m.pt" ":$D/models/yolo26m.pt" && ok "copied this PC's ~/.astra_sim/models/yolo26m.pt"
  else
    warn "no yolo26m.pt found (not given, not in the team's folder, not in ~/.astra_sim/models). Consequence: the legacy detector new_od has no model. Fix: $J deploy --model FILE"
  fi
  [ "$DRY" = 1 ] && { ok "dry run: nothing was changed"; return 0; }
  say "selftest inside the image on $([ "$JETSON_LINK" = virtual ] && echo "this PC" || echo "the Jetson")"
  local st; st=$(cmd_selftest 2>&1); local strc=$?
  printf '%s\n' "$st" | sed 's/^/    /'
  local trt msha
  trt=$(printf '%s\n' "$st" | sed -n 's/^TRT=//p' | head -1)
  msha=$(jetson_ssh "sha256sum $(q "$D/models/yolo26m.pt") 2>/dev/null | cut -c1-16")
  [ -n "${isha:-}" ] || isha=$(docker image inspect -f '{{index .Config.Labels "org.astra.code_sha"}}' "$JETSON_IMAGE" 2>/dev/null)
  jetson_ssh "printf 'IMAGE=%s\nCODE_SHA=%s\nGIT_SHA=%s\nDATE=%s\nMODEL_SHA=%s\nTRT=%s\nSELFTEST=%s\n' $(q "$JETSON_IMAGE") $(q "${isha:-}") $(q "$sha") $(q "$(date -u +%FT%TZ)") $(q "$msha") $(q "$trt") $(q "$([ $strc = 0 ] && echo ok || echo failed)") > $(q "$D/deployed.env")"
  [ "$strc" = 0 ] || jetson_die 9 "selftest failed inside the image (see the lines above). Likely cause: the GPU is not visible in the container (NVIDIA runtime) or a package is missing. Fix: $J check --deep; the lines above name the failing item"
  if [ "$JETSON_LINK" = virtual ]; then
    # this PC is ours (no team to disturb): build the PC's own TensorRT engine right away, as warmup does on the board
    cmd_warmup --yes || exit $?
    ok "deployed. Next: bash simulation/scripts/demo.sh runway --jetson virtual"
    return 0
  fi
  ok "deployed. Next: $J warmup   (once; it uses the GPU for several minutes; only when the team is not using the board)"
}

cmd_warmup() {
  local yes=0 team=()
  while [ $# -gt 0 ]; do
    case "$1" in --yes|-y) yes=1 ;; --team-agreed) team=(--team-agreed) ;; -h|--help) usage; exit 0 ;; *) die_usage "warmup: unknown option '$1'" ;; esac
    shift
  done
  need_login
  if [ "$JETSON_LINK" = virtual ]; then
    # The virtual Jetson builds ITS OWN engine on this PC's GPU from the same model file, with the same team code and
    # TensorRT 10.3 (owner 12:32: one engine per machine). No question, no team check: this PC is ours.
    local vid vtrt
    vid=$(docker image inspect -f '{{.Id}}' "$JETSON_IMAGE" 2>/dev/null)
    [ -n "$vid" ] || jetson_die 12 "$JETSON_IMAGE is not on this PC. Fix: $J image build --flavor virtual"
    vtrt=$(docker image inspect -f '{{index .Config.Labels "org.astra.tensorrt"}}' "$JETSON_IMAGE" 2>/dev/null)
    if [ "$vtrt" != yes ]; then
      ok "this virtual Jetson image has no TensorRT (built with --x86-base ros): new_od runs the .pt model with PyTorch on the PC GPU, so there is no engine to build (a difference from the field: $J image build --flavor virtual gives the TensorRT image)"
      return 0
    fi
    [ -s "$JETSON_DATA/models/yolo26m.pt" ] || jetson_die 12 "There is no model in $JETSON_DATA/models. Likely cause: deploy was not run, or found no yolo26m.pt. Fix: $J deploy --link virtual --model FILE"
    say "Building this PC's own TensorRT engine (TensorRT 10.3 like the Jetson, the team's own export code; a few minutes on the PC GPU)"
    docker run --rm --init --name "${JETSON_CONTAINER}-warmup" --label "$LABEL" --gpus all --network none \
        -e ASTRA_UID="$(id -u)" -e ASTRA_GID="$(id -g)" -e L4T=pc -v "$JETSON_DATA/models":/models "$JETSON_IMAGE" warmup \
      || jetson_die 19 "The engine build failed on this PC (see the lines above). Likely cause: the GPU is not visible to Docker, or the TensorRT export of the model failed. Fix: $J check --link virtual --deep; read the lines above"
    ok "engine ready: $JETSON_DATA/models/yolo26m.engine (+ .meta). The team's new_od uses it exactly as on the board"
    return 0
  fi
  if [ "$yes" = 0 ]; then
    read -r -p "Build the TensorRT engine now? It uses the Jetson's GPU for several minutes. The team must not be using the board. [y/N] " a
    case "$a" in y|Y|yes) ;; *) info "not started"; return 0 ;; esac
  fi
  team_gate warmup "${team[@]}"
  local D="$JETSON_DATA"
  jetson_ssh "[ -s $(q "$D/models/yolo26m.pt") ]" || jetson_die 12 "There is no model in $D/models on the Jetson. Fix: $J deploy (it copies yolo26m.pt)"
  say "Building the engine on the Jetson (low priority inside the container; --cpus 2 --memory 4g)"
  jetson_ssh "$DKPRE dk run --rm --init --name astra-hil-warmup --label $LABEL --runtime nvidia --network none --cpus 2 --memory 4g -e ASTRA_UID=\$(id -u) -e ASTRA_GID=\$(id -g) -e L4T=\$(head -1 /etc/nv_tegra_release | sed -n 's/^# R\([0-9]*\) .*REVISION: \([0-9.]*\),.*/R\1.\2/p') -v $(q "$D/models"):/models $(q "$JETSON_IMAGE") warmup" \
    || jetson_die 19 "The engine build failed (see above). Likely cause: not enough memory, or the TensorRT export of the model failed. Fix: $J check; read the lines above; the team's notes: ros2_ws/src/fixtensorRT.md"
  ok "engine built: $D/models/yolo26m.engine (+ .meta). Next: fly with bash simulation/scripts/demo.sh runway --jetson $JETSON_LINK"
}

# ---------------------------------------------------------------------------------------------------------------
# start / stop / restart / logs / exec / shell
# ---------------------------------------------------------------------------------------------------------------
# run_cmd_sim RUN_ID DOMAIN LOCALONLY PIPELINE : prints the remote docker run line of SIM mode (HIL_DESIGN.md 8.3)
run_cmd_sim() {
  local run_id="$1" dom="$2" lo="$3" pipe="$4" D="$JETSON_DATA" W="$W_IN"
  printf '%s' "dk run -d --init --name $(q "$JETSON_CONTAINER") --label $LABEL --label org.astra.run_id=$(q "$run_id") $(gpu_flag)"
  # limits: the board leaves room for itself (HIL_DESIGN.md 8.3); the PC only with --jetson-like (owner 12:32)
  local lim="--memory 5g --cpus 5"
  if [ "$JETSON_LINK" = virtual ]; then lim=""; [ "${JETSON_LIKE:-0}" = 1 ] && lim="--cpus 6 --memory 7400m"; fi
  printf '%s' " --network none --ipc shareable --cap-add NET_ADMIN --shm-size 1g $lim"
  printf '%s' " -e ASTRA_HIL_MODE=sim -e ASTRA_RUN_ID=$(q "$run_id") -e ASTRA_UID=\$(id -u) -e ASTRA_GID=\$(id -g)"
  printf '%s' " -e ROS_DOMAIN_ID=$(q "$dom") -e ROS_LOCALHOST_ONLY=$(q "$lo") -e ASTRA_PIPELINE=$(q "$pipe") -e ASTRA_LINK=$(q "$JETSON_LINK")"
  printf '%s' " -v $(q "$D/sock"):/run/astra_hil -v $(q "$D/models"):/models -v $(q "$D/runs"):/data/runs"
  printf '%s' " -v $(q "$D/photos"):$W/$PHOTO_SUB -v $(q "$D/detections"):$W/$DETECT_SUB $(q "$JETSON_IMAGE") sim"
}
run_cmd_real() {
  local run_id="$1" dom="$2" lo="$3" pipe="$4" troot="$5" D="$JETSON_DATA" W="$W_IN"
  printf '%s' "dk run -d --init --name astra-hil-real --label $LABEL --label org.astra.run_id=$(q "$run_id") --runtime nvidia --network host --ipc host"
  printf '%s' " --device /dev/ttyUSB0 --group-add dialout -e ASTRA_HIL_MODE=real -e ASTRA_RUN_ID=$(q "$run_id") -e ASTRA_UID=\$(id -u) -e ASTRA_GID=\$(id -g)"
  printf '%s' " -e ROS_DOMAIN_ID=$(q "$dom") ${lo:+-e ROS_LOCALHOST_ONLY=$(q "$lo")} -e ASTRA_PIPELINE=$(q "$pipe")"
  printf '%s' " -v $(q "$D/models"):/models -v $(q "$D/runs"):/data/runs"
  printf '%s' " -v $(q "$troot/$PHOTO_SUB"):$W/$PHOTO_SUB -v $(q "$troot/$DETECT_SUB"):$W/$DETECT_SUB $(q "$JETSON_IMAGE") real"
}
run_cmd_detector() {   # the new pipeline's detector, inside our container's private network + IPC (6.3)
  local img="$1" dcmd="$2" dom="$3"
  printf '%s' "dk run -d --init --name $(q "${JETSON_CONTAINER}-detector") --label $LABEL $(gpu_flag)"
  printf '%s' " --network container:$(q "$JETSON_CONTAINER") --ipc container:$(q "$JETSON_CONTAINER") -u \$(id -u):\$(id -g)"
  printf '%s' " -e ROS_DOMAIN_ID=$(q "$dom") -e ROS_LOCALHOST_ONLY=1 $(q "$img") $dcmd"
}

cmd_start() {
  local mode="" run_id="" dom="" lo="" pipe="${ASTRA_PIPELINE:-auto}" troot="" photos=refuse team=()
  DRY=0
  while [ $# -gt 0 ]; do
    case "$1" in
      --sim) mode=sim ;; --real) mode=real ;;
      --run-id) run_id="${2:-}"; shift ;;
      --domain) dom="${2:-}"; shift ;;
      --localhost-only) lo=1 ;;
      --pipeline) pipe="${2:-}"; shift ;;
      --team-root) troot="${2:-}"; shift ;;
      --archive-photos) photos=archive ;; --keep-photos) photos=keep ;;
      --team-agreed) team=(--team-agreed) ;;
      --jetson-like) JETSON_LIKE=1 ;;
      --dry-run) DRY=1 ;;
      -h|--help) usage; exit 0 ;;
      *) die_usage "start: unknown option '$1'" ;;
    esac
    shift
  done
  [ -n "$mode" ] || die_usage "start: say --sim (Jetson in the loop) or --real (on the drone, no simulator)"
  case "$pipe" in auto|new|legacy) ;; *) die_usage "start: --pipeline must be new, legacy or auto" ;; esac
  case "$dom" in ''|*[!0-9]*) [ -z "$dom" ] || die_usage "start: --domain must be a number" ;; esac
  [ -n "$run_id" ] || run_id="manual-$(date +%Y%m%d-%H%M%S)"
  need_login
  local D="$JETSON_DATA"
  if [ "$mode" = real ] && [ "$JETSON_LINK" = virtual ]; then jetson_die 2 "REAL mode runs on the drone's Jetson only (it opens /dev/ttyUSB0 and the real camera). Fix: start --sim for the virtual Jetson"; fi
  if [ "$DRY" = 1 ]; then
    if [ "$mode" = sim ]; then printf '%s\n' "$(run_cmd_sim "$run_id" "${dom:-77}" "${lo:-1}" "$pipe")"
    else printf '%s\n' "$(run_cmd_real "$run_id" "${dom:-0}" "$lo" "$pipe" "${troot:-/home/${JETSON_USER:-USER}/astra}")"; fi
    return 0
  fi
  team_gate start "${team[@]}"
  # already running? image there? engine (legacy only, board only)?
  local f; f=$(mktemp); collect_facts "$f"
  local st; st=$(fact HIL_CONTAINER "$f")
  [ "$st" = running ] && { rm -f "$f"; jetson_die 1 "Our container $JETSON_CONTAINER is already running. Likely cause: an earlier start or flight did not stop it. Fix: $J stop   (then start again)"; }
  [ -n "$(fact HIL_IMAGE_ID "$f")" ] || { rm -f "$f"; jetson_die 12 "$JETSON_IMAGE is not on $([ "$JETSON_LINK" = virtual ] && echo "this PC" || echo "the Jetson"). Fix: $([ "$JETSON_LINK" = virtual ] && echo "$J image build --flavor virtual" || echo "$J image build, then $J deploy")"; }
  if [ "$JETSON_LINK" != virtual ] && [ "$pipe" = legacy ]; then
    judge "$f" --for start --pipeline legacy "${team[@]}" > "$f.out" 2>&1
    if grep -q 'XX .*\[16b\]' "$f.out"; then grep 'XX .*\[16b\]' "$f.out" >&2; rm -f "$f" "$f.out"; exit 19; fi
    rm -f "$f.out"
  fi
  rm -f "$f"
  jetson_ssh "$DKPRE dk rm -f $(q "$JETSON_CONTAINER") $(q "${JETSON_CONTAINER}-detector") astra-hil-real >/dev/null 2>&1; mkdir -p $(q "$D")/sock $(q "$D")/runs/$(q "$run_id") $(q "$D")/photos $(q "$D")/detections $(q "$D")/run; chmod 700 $(q "$D")/sock" 2>/dev/null
  local line
  if [ "$mode" = sim ]; then
    line=$(run_cmd_sim "$run_id" "${dom:-77}" "${lo:-1}" "$pipe")
  else
    troot="${troot:-/home/$JETSON_USER/astra}"
    # REAL mode refuses next to a native team stack (both would open /dev/ttyUSB0) and with old photos (8.3)
    local n; n=$(jetson_ssh "ls -A $(q "$troot/$PHOTO_SUB") 2>/dev/null | wc -l")
    if [ "${n:-0}" -gt 0 ]; then
      case "$photos" in
        refuse) jetson_die 1 "the team's photo folder $troot/$PHOTO_SUB is not empty ($n files). Likely cause: photos of an earlier flight. Old photos would be processed again and could send the drone to an old target. Fix: add --archive-photos (moves them to $D/photo_archive/) or --keep-photos" ;;
        archive) jetson_ssh "a=$(q "$D/photo_archive")/\$(date +%Y%m%d-%H%M%S)_real; mkdir -p \"\$a\" && mv $(q "$troot/$PHOTO_SUB")/* \"\$a\"/ && echo \"archived to \$a\"" ;;
      esac
    fi
    line=$(run_cmd_real "$run_id" "${dom:-0}" "$lo" "$pipe" "$troot")
  fi
  say "Starting $([ "$mode" = sim ] && echo "SIM (Jetson in the loop)" || echo "REAL (on the drone)") mode, run $run_id"
  jetson_ssh "$DKPRE $line >/dev/null" || jetson_die 13 "docker run failed on $([ "$JETSON_LINK" = virtual ] && echo "this PC" || echo "the Jetson") (see above). Likely cause: the NVIDIA runtime is missing, or a folder could not be mounted. Fix: $J check"
  # tegrastats + power mode at the start (board only; report only)
  if [ "$JETSON_LINK" != virtual ]; then
    jetson_ssh "R=$(q "$D/runs/$run_id"); (sudo -n nvpmodel -q 2>/dev/null || nvpmodel -q 2>/dev/null) | sed 's/^/start: /' > \"\$R/power.txt\"; if command -v tegrastats >/dev/null; then nohup tegrastats --interval 1000 --logfile \"\$R/tegrastats.log\" >/dev/null 2>&1 & echo \$! > $(q "$D/run/tegrastats.pid"); fi" 2>/dev/null
  fi
  # the new pipeline's detector container (image and command from the video workflow; HIL_DESIGN.md 6.3)
  if [ "$mode" = sim ] && [ "$pipe" != legacy ] && [ -n "${JETSON_DETECTOR_IMAGE:-}" ]; then
    jetson_ssh "$DKPRE $(run_cmd_detector "$JETSON_DETECTOR_IMAGE" "${JETSON_DETECTOR_CMD:-}" "${dom:-77}") >/dev/null" \
      && ok "detector container ${JETSON_CONTAINER}-detector started ($JETSON_DETECTOR_IMAGE)" \
      || warn "the detector container did not start. Consequence: no yolo26 detections in this run. Fix: $J logs; check JETSON_DETECTOR_IMAGE"
  elif [ "$mode" = sim ] && [ "$pipe" != legacy ]; then
    info "no separate detector container: JETSON_DETECTOR_IMAGE is not set (the video workflow names the team's yolo26 image in docs/simulation/VIDEO_PIPELINE.md)."
  fi
  [ "$mode" = real ] && { ok "REAL mode started (container astra-hil-real). Logs: $J logs"; return 0; }
  # wait until the relay listens (contract 15.3 h: exit 0 = running + relay listening, per relay.status)
  local t=0 status_file="$D/runs/$run_id/relay.status"
  while [ "$t" -lt 40 ]; do
    if jetson_ssh "grep -q '^LISTEN=yes' $(q "$status_file") 2>/dev/null"; then
      ok "container $JETSON_CONTAINER runs and its relay listens (run $run_id; logs: $J logs)"
      return 0
    fi
    if ! jetson_ssh "$DKPRE [ \"\$(dk inspect -f '{{.State.Running}}' $(q "$JETSON_CONTAINER") 2>/dev/null)\" = true ]"; then
      jetson_ssh "$DKPRE dk logs --tail 30 $(q "$JETSON_CONTAINER") 2>&1" | sed 's/^/    /' >&2
      jetson_die 13 "the container $JETSON_CONTAINER stopped right after the start (its last lines are above). Likely cause: the entrypoint refused (wrong network mode, missing socket folder) or the image is broken. Fix: read the lines above; $J logs"
    fi
    sleep 1; t=$((t + 1))
  done
  jetson_die 13 "the container runs, but its relay did not say LISTEN=yes in $status_file within 40 s. Likely cause: the relay could not bind its addresses. Fix: $J logs; $J exec -- cat /data/runs/$run_id/relay.log"
}

cmd_stop() {
  need_login
  local D="$JETSON_DATA"
  say "Stopping our containers (label $LABEL) on $([ "$JETSON_LINK" = virtual ] && echo "this PC" || echo "the Jetson")"
  local run_id; run_id=$(jetson_ssh "$DKPRE dk inspect -f '{{index .Config.Labels \"org.astra.run_id\"}}' $(q "$JETSON_CONTAINER") 2>/dev/null")
  jetson_ssh "$DKPRE for c in $(q "${JETSON_CONTAINER}-detector") $(q "$JETSON_CONTAINER") astra-hil-real; do
      [ \"\$(dk inspect -f '{{index .Config.Labels \"org.astra.owner\"}}' \$c 2>/dev/null)\" = hil ] || continue
      dk stop -t 25 \$c >/dev/null 2>&1; dk logs \$c > $(q "$D/logs")/\$c.\$(date +%Y%m%d-%H%M%S).log 2>&1; dk rm \$c >/dev/null 2>&1 && echo \"stopped \$c\"
    done" | sed 's/^/    /'
  if [ "$JETSON_LINK" != virtual ]; then
    jetson_ssh "p=\$(cat $(q "$D/run/tegrastats.pid") 2>/dev/null); [ -n \"\$p\" ] && [ \"\$(cat /proc/\$p/comm 2>/dev/null)\" = tegrastats ] && kill \$p; rm -f $(q "$D/run/tegrastats.pid");
      [ -n $(q "$run_id") ] && [ -d $(q "$D/runs/$run_id") ] && (sudo -n nvpmodel -q 2>/dev/null || nvpmodel -q 2>/dev/null) | sed 's/^/end: /' >> $(q "$D/runs/$run_id/power.txt")" 2>/dev/null
  fi
  ok "stopped"
}

cmd_logs() {
  local follow=() run=""
  while [ $# -gt 0 ]; do
    case "$1" in -f|--follow) follow=(-f) ;; --run) run="${2:-}"; shift ;; -h|--help) usage; exit 0 ;; *) die_usage "logs: unknown option '$1'" ;; esac
    shift
  done
  need_login
  if [ -n "$run" ]; then
    jetson_ssh "${follow:+tail -f} ${follow:-cat} $(q "$JETSON_DATA/runs/$run/team_stack.log")" || jetson_die 1 "no log for run $run. Fix: ls the runs: $J exec -- ls /data/runs"
  else
    jetson_ssh "$DKPRE dk logs ${follow[*]} --tail 200 $(q "$JETSON_CONTAINER")" || jetson_die 13 "the container $JETSON_CONTAINER does not exist. Likely cause: it was never started, or it was stopped (stop saves its log in $JETSON_DATA/logs/). Fix: $J start --sim"
  fi
}

container_running() {
  jetson_ssh "$DKPRE [ \"\$(dk inspect -f '{{.State.Running}}' $(q "$JETSON_CONTAINER") 2>/dev/null)\" = true ]"
}

jetson_ssh_tty() {   # like jetson_ssh, with a terminal (for shell / exec --tty)
  if [ "$JETSON_LINK" = virtual ]; then bash -c "$*"; return; fi
  "${JETSON_SSH[0]}" -t "${JETSON_SSH[@]:1}" "$*"
}

cmd_exec() {
  local tty=0
  while [ $# -gt 0 ]; do
    case "$1" in --tty) tty=1 ;; --) shift; break ;; -h|--help) usage; exit 0 ;; *) break ;; esac
    shift
  done
  [ $# -gt 0 ] || die_usage "exec: give the command after --, e.g. exec -- ros2 node list"
  need_login
  container_running || jetson_die 13 "the container $JETSON_CONTAINER is not running. Likely cause: it was not started, or it stopped. Fix: $J start --sim (or look at $J logs)"
  local a qc=""
  for a in "$@"; do qc+=" $(q "$a")"; done
  # always as the stack's user: ROS commands as root cannot see the team stack (HIL_DESIGN.md 8.6)
  local it="-i"; [ "$tty" = 1 ] && it="-it"
  local line
  line="$DKPRE dk exec $it -u astra-dev -e HOME=/home/astra-dev -w $W_IN $(q "$JETSON_CONTAINER") /opt/astra/in_stack.sh$qc"
  if [ "$tty" = 1 ]; then jetson_ssh_tty "$line"; else jetson_ssh "$line"; fi
}

cmd_shell() {
  local root=0
  while [ $# -gt 0 ]; do case "$1" in --root) root=1 ;; -h|--help) usage; exit 0 ;; *) die_usage "shell: unknown option '$1'" ;; esac; shift; done
  need_login
  container_running || jetson_die 13 "the container $JETSON_CONTAINER is not running. Fix: $J start --sim"
  if [ "$root" = 1 ]; then
    warn "a root shell: ROS commands run as root cannot see the team stack (they only see root's own nodes). Consequence: use it for files and network only."
    jetson_ssh_tty "$DKPRE dk exec -it -u root $(q "$JETSON_CONTAINER") bash"
  else
    jetson_ssh_tty "$DKPRE dk exec -it -u astra-dev -e HOME=/home/astra-dev -w $W_IN $(q "$JETSON_CONTAINER") /opt/astra/in_stack.sh bash"
  fi
}

# ---------------------------------------------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------------------------------------------
CMD="${1:-help}"; [ $# -gt 0 ] && shift
case "$CMD" in -h|--help|help) usage; exit 0 ;; esac
jetson_target_load "$@"
set -- "${JETSON_ARGS[@]}"
mkdir -p "$JETSON_STATE" 2>/dev/null && chmod 700 "$JETSON_STATE" 2>/dev/null
case "$CMD" in
  login)   cmd_login "$@" ;;
  logout)  cmd_logout "$@" ;;
  check)   cmd_check "$@" ;;
  setup)   cmd_setup "$@" ;;
  find)    cmd_find "$@" ;;
  clock)   cmd_clock "$@" ;;
  status)  cmd_status "$@" ;;
  image)   cmd_image "$@" ;;
  deploy)  cmd_deploy "$@" ;;
  warmup)  cmd_warmup "$@" ;;
  selftest) need_login; cmd_selftest ;;
  start)   cmd_start "$@" ;;
  stop)    cmd_stop "$@" ;;
  restart) cmd_stop; cmd_start "$@" ;;
  logs)    cmd_logs "$@" ;;
  exec)    cmd_exec "$@" ;;
  shell)   cmd_shell "$@" ;;
  paths)   cmd_paths "$@" ;;
  target)  cmd_target "$@" ;;
  *) die_usage "unknown command '$CMD'" ;;
esac
