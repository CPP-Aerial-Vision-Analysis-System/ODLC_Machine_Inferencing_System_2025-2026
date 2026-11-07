#!/bin/bash
# =============================================================================
#  simulation/jetson/lib/login_window.sh  --  the visible "Jetson login" window (owner's rule, 2026-10-31 11:40)
#
#  PURPOSE: open ONE SSH login to the Jetson Orin Nano in a window the owner can see. The owner types the Jetson
#  password HERE himself. While this window stays open, every jetson.sh command (and the simulator link) reuses this
#  one login through SSH connection sharing ("ControlMaster", the socket ~/.ssh/cm-jetson), so no tool ever types,
#  sees or stores a password. Closing this window (or typing "exit" on the Jetson) ends all access at once.
#  Nothing is installed or written on the Jetson by this script. It never retries by itself: when the login fails or
#  the connection drops, it says why in plain words and waits for the owner to press Enter to try again.
#
#  It copies the behaviour of the lead's reference D:\suas_scratch\e2e\jetson_login.sh (same control socket, so
#  either window serves the kit) and adds a fast keepalive (a dropped cable is noticed in about 6 s, not 90 s) and the
#  kit's own known-hosts file.
#
#  Started by:  bash simulation/jetson/jetson.sh login   (opens this in a new window; --here runs it in this terminal)
#  By hand:     bash login_window.sh USER@HOST [PORT] [CONTROL_SOCKET] [KNOWN_HOSTS] [STATE_DIR]
#               (HOST alone: the window asks for the user name)
#  Writes:      STATE_DIR/login.env (USER=, HOST=, PORT=, CM=, KEEPALIVE_S=, STARTED=) while the login is open, so
#               jetson.sh knows which board this window is logged in to. Deleted when the window ends.
# =============================================================================

ADDR="${1:-}"
PORT="${2:-22}"
SOCK="${3:-$HOME/.ssh/cm-jetson}"
KNOWN="${4:-${ASTRA_SIM_HOME:-$HOME/.astra_sim}/jetson/known_hosts}"
STATE="${5:-${ASTRA_SIM_HOME:-$HOME/.astra_sim}/jetson}"
ERRF="$(mktemp)"
mkdir -p "$(dirname "$SOCK")" "$STATE" "$(dirname "$KNOWN")"
chmod 700 "$(dirname "$SOCK")" "$STATE" 2>/dev/null
printf '\033]0;Jetson login - keep this window open\007'

ask_target() {
  if [ -z "$ADDR" ]; then
    echo "Which Jetson? Type its address (192.168.55.1 over the USB-C cable, or its network name / IP):"
    read -r -p "Address: " ADDR
  fi
  case "$ADDR" in
    *@*) TARGET="$ADDR" ;;
    *)   echo "Jetson address: $ADDR"
         read -r -p "Your user name on that Jetson (the Linux login there, e.g. astra-dev): " JUSER
         TARGET="$JUSER@$ADDR" ;;
  esac
}

while true; do
  ask_target
  if [ -z "${TARGET%@*}" ] || [ -z "${TARGET#*@}" ]; then
    echo "[jetson] That is not USER@ADDRESS. Try again."; ADDR=""; continue
  fi
  rm -f "$SOCK"                                    # a stale socket of an old window would block a new login
  {
    echo "# login.env -- written by login_window.sh while the Jetson login window is open"
    echo "USER=${TARGET%@*}"
    echo "HOST=${TARGET#*@}"
    echo "PORT=$PORT"
    echo "CM=$SOCK"
    echo "KEEPALIVE_S=2"
    echo "STARTED=$(date +%s)"
  } > "$STATE/login.env"
  echo "=============================================================================="
  echo " Logging in to the Jetson as: $TARGET  (port $PORT)"
  echo " Type the Jetson password when asked. The tools never see it."
  echo " KEEP THIS WINDOW OPEN while you work with the Jetson (you may minimize it)."
  echo " Close the window, or type 'exit' below, to end all access to the Jetson."
  echo " (The first time, ssh asks you to accept the Jetson's key fingerprint: type yes.)"
  echo "=============================================================================="
  ssh -M -S "$SOCK" -o ControlPersist=no -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
      -o ConnectTimeout=10 -o "UserKnownHostsFile=$KNOWN" -o StrictHostKeyChecking=accept-new \
      -o NumberOfPasswordPrompts=3 -p "$PORT" "$TARGET" 2> >(tee "$ERRF" >&2)
  status=$?
  sleep 0.3                                        # let tee finish writing
  rm -f "$SOCK" "$STATE/login.env"
  if [ "$status" -eq 0 ]; then
    echo; echo "Logged out normally. The tools can no longer reach the Jetson."
  else
    echo
    echo "------------------------------------------------------------------------------"
    if grep -q "REMOTE HOST IDENTIFICATION HAS CHANGED" "$ERRF"; then
      echo "[jetson] The Jetson's key changed (another board at this address, or it was re-flashed)."
      echo "         Fix, if you trust this board:  ssh-keygen -f \"$KNOWN\" -R ${TARGET#*@}   then try again."
    elif grep -q "Permission denied (publickey)" "$ERRF"; then
      echo "[jetson] This Jetson does NOT accept passwords for '$TARGET' (keys only)."
      echo "         Fix: a teammate on the Jetson allows password login for this user, or adds your key."
    elif grep -q -i "Permission denied" "$ERRF"; then
      echo "[jetson] Wrong user name or password for '$TARGET' (the Jetson said 'Permission denied')."
      echo "         Fix: check the user name (what 'whoami' prints on the Jetson) and the password, then try again."
    elif grep -q -i -E "Connection refused" "$ERRF"; then
      echo "[jetson] ${TARGET#*@} answers, but its SSH port $PORT is closed."
      echo "         Likely cause: the Jetson is still starting, or SSH is off. Fix: wait 1 minute; on the Jetson:"
      echo "         sudo systemctl enable --now ssh"
    elif grep -q -i -E "timed out|No route|Could not resolve|Network is unreachable" "$ERRF"; then
      echo "[jetson] Could not reach ${TARGET#*@} (network)."
      echo "         USB-C cable: is it a DATA cable in the Jetson's USB-C port, is the Jetson powered (barrel jack),"
      echo "         did it finish starting (about 1 minute)? Wi-Fi/LAN: is the address right (jetson.sh find)?"
    elif grep -q -i -E "broken pipe|closed by remote|Timeout, server .* not responding|Connection reset" "$ERRF"; then
      echo "[jetson] The connection to the Jetson DROPPED (cable pulled, Wi-Fi lost, or the Jetson restarted)."
      echo "         Running flights keep flying (the autopilot does not need the Jetson to stay in the air)."
      echo "         Fix: check the cable / network, then press Enter and type the password to log in again."
    else
      echo "[jetson] The login ended with code $status. The message above says why."
    fi
    echo "------------------------------------------------------------------------------"
  fi
  read -r -p "Log in again? Enter = yes, n = no (closes the window), u = yes with another user name: " again
  case "$again" in
    n|N) break ;;
    u|U) ADDR="${TARGET#*@}" ;;                    # keep the address, ask the user name again
    *)   ADDR="$TARGET" ;;                         # same user and address
  esac
  : > "$ERRF"
done
rm -f "$ERRF" "$SOCK" "$STATE/login.env"
echo "The tools can no longer reach the Jetson. This window closes in 5 s."
sleep 5
