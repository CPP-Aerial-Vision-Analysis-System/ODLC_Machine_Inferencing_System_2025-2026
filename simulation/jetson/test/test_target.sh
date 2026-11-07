#!/usr/bin/env bash
# =============================================================================
#  simulation/jetson/test/test_target.sh  --  tests of lib/target.sh (the contract both builders code against)
#
#  PURPOSE: prove "which Jetson?" is answered the documented way (HIL_DESIGN.md 15.3 a): options > environment >
#  the saved file > defaults; a --host without --link means ssh (unless it is the USB address); the saved user/data
#  belong to the saved host only; login mode uses the login window's socket for A and B, key mode its own two; the
#  virtual link has no SSH at all; unused arguments are left in JETSON_ARGS. Never connects to anything.
#  HOW TO RUN (WSL or any Linux bash):  bash simulation/jetson/test/test_target.sh
# =============================================================================
set -u
T_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d /tmp/jet_target_test.XXXX)"
trap 'rm -rf "$TMP"' EXIT
PASS=0; FAIL=0
check() {   # check NAME "ACTUAL" "WANT"
  if [ "$2" = "$3" ]; then PASS=$((PASS + 1)); echo "PASS $1"; else FAIL=$((FAIL + 1)); echo "FAIL $1: got '$2', want '$3'"; fi
}
# each case runs in a subshell with a clean environment and its own state folder
load() {   # load "ENV_ASSIGNMENTS" ARGS... -> prints LINK HOST USER PORT AUTH CM_A CM_B DATA NSSH ARGS
  local envs="$1"; shift
  env -i HOME="$TMP/home" PATH="$PATH" JETSON_STATE_DIR="$TMP/state" $envs bash -c '
    source "'"$T_DIR"'/../lib/target.sh"
    jetson_target_load "$@"
    printf "%s|%s|%s|%s|%s|%s|%s|%s|%s|%s\n" "$JETSON_LINK" "$JETSON_HOST" "$JETSON_USER" "$JETSON_PORT" "$JETSON_AUTH" \
      "${JETSON_CM_A##*/}" "${JETSON_CM_B##*/}" "$JETSON_DATA" "${#JETSON_SSH[@]}" "${JETSON_ARGS[*]}"' _ "$@" 2>&1
}
mkdir -p "$TMP/state" "$TMP/home"

check "defaults = USB, login mode" "$(load "")" "usb|192.168.55.1||22|login|cm-jetson|cm-jetson||24|"
check "--host means ssh" "$(load "" --host jetson.local --user astra-dev)" "ssh|jetson.local|astra-dev|22|login|cm-jetson|cm-jetson||24|"
check "--host 192.168.55.1 stays usb" "$(load "" --host 192.168.55.1 | cut -d'|' -f1)" "usb"
check "other args are kept" "$(load "" check --quick --port 2222 --for flight | cut -d'|' -f4,10)" "2222|check --quick --for flight"
check "env JETSON_HOST" "$(load "JETSON_HOST=10.0.0.5 JETSON_USER=bob" | cut -d'|' -f1-3)" "ssh|10.0.0.5|bob"
check "option beats env" "$(load "JETSON_HOST=10.0.0.5" --host 10.0.0.6 | cut -d'|' -f2)" "10.0.0.6"
printf 'JETSON_LINK=ssh\nJETSON_HOST=192.168.144.20\nJETSON_USER=astra-dev\nJETSON_PORT=2200\nJETSON_AUTH=login\nJETSON_DATA=/home/astra-dev/astra_hil\n' > "$TMP/state/target.env"
check "saved target used" "$(load "" | cut -d'|' -f1-4,8)" "ssh|192.168.144.20|astra-dev|2200|/home/astra-dev/astra_hil"
check "--link ssh uses the saved ssh host" "$(load "" --link ssh | cut -d'|' -f2)" "192.168.144.20"
check "env beats the file" "$(load "JETSON_PORT=22" | cut -d'|' -f4)" "22"
check "saved user/data only for the saved host" "$(load "" --host other.local | cut -d'|' -f3,4,8)" "|22|"
check "--link usb ignores the saved ssh host" "$(load "" --link usb | cut -d'|' -f1,2)" "usb|192.168.55.1"
check "key mode: own sockets" "$(load "" --auth key | cut -d'|' -f5-7)" "key|cm-A|cm-B"
check "virtual: no ssh, local data" "$(load "" --link virtual | cut -d'|' -f1,2,9)" "virtual|localhost|0"
check "virtual data folder" "$(load "" --link virtual | cut -d'|' -f8)" "$TMP/home/.astra_sim/jetson_virtual"
# a saved "virtual" (written by an older kit) is never the default: plain commands mean the board (USB)
printf 'JETSON_LINK=virtual\nJETSON_HOST=localhost\nJETSON_USER=jarrod\n' > "$TMP/state/target.env"
check "saved virtual is not the default" "$(load "" | cut -d'|' -f1,2)" "usb|192.168.55.1"
# jetson_save_target refuses the virtual link (returns 1, file untouched)
printf 'JETSON_LINK=ssh\nJETSON_HOST=192.168.144.20\n' > "$TMP/state/target.env"
env -i HOME="$TMP/home" PATH="$PATH" JETSON_STATE_DIR="$TMP/state" bash -c 'source "'"$T_DIR"'/../lib/target.sh"; jetson_target_load --link virtual; jetson_save_target' >/dev/null 2>&1
check "virtual is never saved (exit 1)" "$?" "1"
check "virtual is never saved (file kept)" "$(grep -c '^JETSON_HOST=192.168.144.20$' "$TMP/state/target.env")" "1"
rm -f "$TMP/state/target.env"
printf 'USER=astra-obs-avd\nHOST=192.168.55.1\nPORT=22\n' > "$TMP/state/login.env"
check "user from the open login window" "$(load "" | cut -d'|' -f3)" "astra-obs-avd"
rm -f "$TMP/state/login.env"
out=$(load "" --link ssh)
check "ssh without a host is an error" "$(grep -c 'needs the Jetson' <<<"$out")" "1"
out=$(load "" --link warp)
check "unknown link is an error" "$(grep -c 'unknown link' <<<"$out")" "1"
out=$(load "" --port abc)
check "port must be a number" "$(grep -c 'must be a number' <<<"$out")" "1"
out=$(load "" --auth pw)
check "unknown auth is an error" "$(grep -c 'unknown --auth' <<<"$out")" "1"
# exit codes of those errors are 2 (typed wrong)
env -i HOME="$TMP/home" PATH="$PATH" JETSON_STATE_DIR="$TMP/state" bash -c 'source "'"$T_DIR"'/../lib/target.sh"; jetson_target_load --link warp' >/dev/null 2>&1
check "typed wrong -> exit 2" "$?" "2"
# the one SSH option set
opts=$(env -i HOME="$TMP/home" PATH="$PATH" JETSON_STATE_DIR="$TMP/state" bash -c 'source "'"$T_DIR"'/../lib/target.sh"; jetson_target_load --host h --user u; echo "${JETSON_SSH[*]}"')
for o in BatchMode=yes ConnectTimeout=5 ServerAliveInterval=2 ServerAliveCountMax=3 StrictHostKeyChecking=yes ExitOnForwardFailure=yes ControlMaster=no; do
  check "ssh option $o" "$(grep -c -- "$o" <<<"$opts")" "1"
done
check "ssh ends with the destination" "${opts##* }" "u@h"
echo
echo "test_target: $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
