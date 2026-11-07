#!/usr/bin/env bash
# =============================================================================
#  simulation/jetson/pc/test/test_pc_scripts.sh  --  tests of the PC side's shell scripts (no board, no simulator)
#
#  PURPOSE: check, inside Ubuntu/WSL and in a throwaway folder, the parts of link.sh, collect.sh and hil_steps.sh that
#  must be right before anyone plugs a Jetson in:
#    1. every script parses (bash -n), and shellcheck is clean when it is installed;
#    2. link.sh's forwards: the right socket files and PC ports for links A and B (SITL SERIAL2 = 5763 + 10 x instance);
#    3. link.sh's "kill the old session" step (HIL_DESIGN.md 15.3 g), the safety-critical part: it kills ONLY an
#       sshd of our user with the recorded pid AND start time, never the current connection, never another program,
#       never a process whose start time differs (a reused pid); it removes this link's socket files, never tap.sock;
#       and the record step writes the connection's own pid + start time;
#    4. collect.sh with the virtual link: the clock record, the archive of old photos, the final copy into a run folder
#       (photos + sidecars, detections, team_stack.log, the Jetson's run files);
#    5. hil_steps.sh's option checks (profile choice, run.team_stack external, the baud file + param_keep, refusals);
#    6. link.sh with the virtual link (hil_relay.py pc-local and a fake SITL SERIAL2 on 127.0.0.1:5833): socket files,
#       door A, status, a drop rehearsal, stop; and with its watcher ON, read through a pipe like demo.sh does, it
#       must end by itself (the watcher writes to the link log and never holds the caller's pipe).
#  Nothing here connects anywhere else; ASTRA_SIM_HOME points into the throwaway folder.
#  Run:  bash simulation/jetson/pc/test/test_pc_scripts.sh      (exit 0 = all passed)
# =============================================================================
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PC="$(cd "$HERE/.." && pwd)"
J="$(cd "$PC/.." && pwd)"
REPO="$(cd "$J/../.." && pwd)"
T="$(mktemp -d /tmp/jet_pc_test.XXXXXX)"
export ASTRA_SIM_HOME="$T/home"
mkdir -p "$ASTRA_SIM_HOME/state"
PASS=0; FAIL=0; KIDS=()
pass() { PASS=$((PASS + 1)); printf ' ok   %s\n' "$1"; }
fail() { FAIL=$((FAIL + 1)); printf ' FAIL %s\n' "$1"; }
check() { local name="$1"; shift; if "$@"; then pass "$name"; else fail "$name"; fi; }
cleanup() { for k in "${KIDS[@]}"; do kill "$k" 2>/dev/null; done; rm -rf "$T"; }
trap cleanup EXIT
starttime() { awk '{print $22}' "/proc/$1/stat"; }

echo "== 1. syntax"
for f in "$PC/link.sh" "$PC/collect.sh" "$PC/hil_steps.sh" "$HERE/test_pc_scripts.sh"; do
  check "bash -n $(basename "$f")" bash -n "$f"
done
if command -v shellcheck >/dev/null 2>&1; then
  check "shellcheck" shellcheck -x -S warning "$PC/link.sh" "$PC/collect.sh" "$PC/hil_steps.sh"
else
  echo " --   shellcheck not installed (skipped)"
fi

echo "== 2. link.sh forwards"
D="$T/D"
mapfile -t S < <(JETSON_DATA_DIR="$D" bash "$PC/link.sh" _steps --link ssh --host 10.0.0.9 --user jet --instance 2 2>&1)
check "6 step lines (clean/record/specs for A and B)" [ "${#S[@]}" = 6 ]
check "A: -R mavlink.sock -> SITL SERIAL2 of instance 2 (5783)" grep -q -- "-R $D/sock/mavlink.sock:127.0.0.1:5783" <<< "${S[2]}"
check "A: -L 7461 -> tap.sock" grep -q -- "-L 127.0.0.1:7461:$D/sock/tap.sock" <<< "${S[2]}"
has() { grep -q -- "$2" <<< "$1"; }          # has TEXT PATTERN
lacks() { ! grep -q -- "$2" <<< "$1"; }
check "B: SDK -> 127.0.0.1:37261" has "${S[5]}" "-R $D/sock/cam_sdk.sock:127.0.0.1:37261"
check "B: HTTP -> 127.0.0.1:37282" has "${S[5]}" "-R $D/sock/cam_http.sock:127.0.0.1:37282"
check "B: RTSP -> 192.168.144.25:8554" has "${S[5]}" "-R $D/sock/cam_rtsp.sock:192.168.144.25:8554"
check "B's clean step removes the three camera sockets" has "${S[3]}" "cam_sdk.sock' '$D/sock/cam_http.sock' '$D/sock/cam_rtsp.sock'"
check "B's clean step never touches tap.sock" lacks "${S[3]}" "tap.sock"
check "A's clean step never touches tap.sock" lacks "${S[0]}" "tap.sock"

echo "== 3. link.sh: kill only the recorded old sshd session"
CLEAN_A="${S[0]}"; RECORD_A="${S[1]}"
mkdir -p "$D/sock" "$D/run" "$T/bin"
cp /bin/sleep "$T/bin/sshd"                       # a program whose name (comm) is "sshd", owned by us
cp /bin/bash "$T/bin2_sshd" && mkdir -p "$T/b2" && mv "$T/bin2_sshd" "$T/b2/sshd"   # a bash named "sshd"
# (a) another program (comm "sleep") with the right pid + start time: never killed
sleep 300 & P=$!; KIDS+=("$P")
touch "$D/sock/mavlink.sock" "$D/sock/tap.sock"
echo "$P $(starttime "$P")" > "$D/run/link_A.pid"
bash -c "$CLEAN_A" >/dev/null 2>&1
check "a program that is not sshd is never killed" kill -0 "$P"
check "this link's socket file is removed" [ ! -e "$D/sock/mavlink.sock" ]
check "tap.sock (the tap sender's) is kept" [ -e "$D/sock/tap.sock" ]
# (b) an sshd of ours with the recorded pid + start time: killed
"$T/bin/sshd" 300 & F=$!; KIDS+=("$F")
echo "$F $(starttime "$F")" > "$D/run/link_A.pid"
out=$(bash -c "$CLEAN_A" 2>&1); sleep 0.3
check "the recorded old sshd session is killed ($out)" bash -c "! kill -0 $F 2>/dev/null"
# (c) an sshd with the same pid but another start time (a reused pid): never killed
"$T/bin/sshd" 300 & G=$!; KIDS+=("$G")
echo "$G 12345" > "$D/run/link_A.pid"
bash -c "$CLEAN_A" >/dev/null 2>&1
check "a reused pid (other start time) is never killed" kill -0 "$G"
# (d) the CURRENT connection (the clean command's own parent is the recorded sshd): never killed
"$T/b2/sshd" -c 'echo "$$ $(awk "{print \$22}" /proc/$$/stat)" > "'"$D"'/run/link_A.pid"; bash -c "$0"; echo alive' "$CLEAN_A" > "$T/cur.out" 2>&1
check "the current connection is never killed" grep -q alive "$T/cur.out"
# (e) the record step writes the connection's own pid + start time (its parent, here a bash named sshd)
"$T/b2/sshd" -c 'bash -c "$0" > /dev/null; echo "$$ $(awk "{print \$22}" /proc/$$/stat)"' "$RECORD_A" > "$T/rec.expect"
check "record step: pid + start time of the connection" diff -q "$T/rec.expect" "$D/run/link_A.pid"
kill "$G" 2>/dev/null

echo "== 4. collect.sh with the virtual link"
V="$T/V"; RUN="$T/runs/20261031-120000_runway"
mkdir -p "$V/photos" "$V/detections" "$V/runs/RUN1/ros_log" "$RUN"
printf 'jpeg' > "$V/photos/36.1 , -96.1.jpg"; printf '{}' > "$V/photos/36.1 , -96.1.json"
printf 'det' > "$V/detections/d1.jpg"; printf 'stack line\n' > "$V/runs/RUN1/team_stack.log"
printf 'MAV=up\n' > "$V/runs/RUN1/relay.status"
export JETSON_DATA_DIR="$V"
check "clock start (virtual = same clock)" bash "$PC/collect.sh" clock start --link virtual --run-dir "$RUN"
check "clock.txt says offset 0" grep -q '^offset_start_s=0.0000' "$RUN/jetson/clock.txt"
check "final copy" bash "$PC/collect.sh" --final --link virtual --run-dir "$RUN" --run-id RUN1
check "photo + sidecar in RUN/photos" bash -c "[ -f '$RUN/photos/36.1 , -96.1.jpg' ] && [ -f '$RUN/photos/36.1 , -96.1.json' ]"
check "the staging copies are removed after the final copy" bash -c "[ ! -e '$RUN/jetson/.final_photos' ] && [ ! -e '$RUN/jetson/.mirror' ]"
check "detections copied" [ -f "$RUN/detections/d1.jpg" ]
check "team_stack.log at the run folder's top (same name as a PC run)" grep -q 'stack line' "$RUN/team_stack.log"
check "the Jetson's run files in RUN/jetson" bash -c "[ -f '$RUN/jetson/relay.status' ] && [ -d '$RUN/jetson/ros_log' ]"
check "archive of old photos" bash "$PC/collect.sh" archive --link virtual --run-dir "$RUN" --stamp S1
check "photos folder empty, archive filled" bash -c "[ -z \"\$(ls -A '$V/photos')\" ] && [ -f '$V/photo_archive/S1/photos/36.1 , -96.1.jpg' ]"
unset JETSON_DATA_DIR

echo "== 5. hil_steps.sh option checks"
hs() {   # hs VAR=VALUE... : source hil_steps with demo.sh's defaults + the given values, run hil_options, print the result
  ( source "$REPO/simulation/scripts/_common.sh"
    JETSON=usb; JETSON_HOST_OPT=""; JETSON_USER_OPT=""; JETSON_PORT_OPT=""; JETSON_AUTH_OPT=""; JETSON_PHOTOS=live
    JETSON_TAP=on; JETSON_BAUD=off; JETSON_PACE=auto; JETSON_PIPELINE=legacy; DETECTOR=real; PROFILE=""; TYPED=()
    PARAM_FILES=""; PLUGIN=0
    for kv in "$@"; do eval "$kv"; done
    source "$PC/hil_steps.sh"
    hil_options
    echo "PROFILE=$PROFILE PLUGIN=$PLUGIN TYPED=${TYPED[*]} PARAM_FILES=$PARAM_FILES T=${HIL_T[*]}" ) 2>&1
}
out=$(hs JETSON=virtual)
check "virtual -> profile jetson_virtual + team_stack external" grep -q 'PROFILE=jetson_virtual PLUGIN=1 TYPED=run.team_stack=external' <<< "$out"
out=$(hs JETSON=ssh JETSON_HOST_OPT=jetson.local JETSON_USER_OPT=astra-dev)
check "ssh -> jetson_ssh, target --host --user" grep -q 'PROFILE=jetson_ssh .*T=--link ssh --host jetson.local --user astra-dev' <<< "$out"
out=$(hs JETSON_BAUD=115200)
check "--jetson-baud 115200 -> the .parm file" has "$out" "PARAM_FILES=$PC/hil_serial2_115200.parm"
check "--jetson-baud 115200 -> param_keep" has "$out" "autopilot.param_keep=SIM_BAUDLIMIT_EN,SERIAL0_BAUD,SERIAL2_BAUD"
out=$(hs PROFILE=my_profile)
check "a typed --profile is kept" grep -q 'PROFILE=my_profile' <<< "$out"
( hs DETECTOR=truth >/dev/null ); rc=$?
check "--detector truth with --jetson is refused (exit 2)" [ "$rc" = 2 ]
( ASTRA_SIM_HOME="$T/empty" hs JETSON=ssh >/dev/null ); rc=$?
check "--jetson ssh without a host is refused (exit 2)" [ "$rc" = 2 ]
( hs JETSON=serial >/dev/null ); rc=$?
check "--jetson serial is refused (exit 2)" [ "$rc" = 2 ]
( hs JETSON_PACE=fast >/dev/null ); rc=$?
check "--photo-pace fast is refused (exit 2)" [ "$rc" = 2 ]
# which camera pipeline runs, from the team node names the ROS tap reports (hil_pipeline_from_nodes)
pn() { ( source "$REPO/simulation/scripts/_common.sh"; ASTRA_STATE_DIR="$T/pn"; mkdir -p "$ASTRA_STATE_DIR"
         printf 'CONNECTED=1\nNODES=%s\n' "$1" > "$ASTRA_STATE_DIR/jetson_tap.env"; source "$PC/hil_steps.sh"; hil_pipeline_from_nodes ) 2>&1; }
check "tap nodes siyi,new_od -> legacy" [ "$(pn gps_mavros_service_node,main_controller,mavros,mission_camera_trigger,new_od,siyi,waypoint_manager)" = legacy ]
check "tap nodes zr10_camera,zr10_stream -> new" [ "$(pn main_controller,mavros,zr10_camera,zr10_stream)" = new ]
check "tap nodes without a camera node -> (unknown yet)" [ -z "$(pn mavros,waypoint_manager)" ]

echo "== 6. link.sh with the virtual link (hil_relay.py pc-local; a fake SITL SERIAL2 on 127.0.0.1:5833 = instance 7)"
if [ -f "$J/lib/hil_relay.py" ]; then
  python3 -c "
import socket, threading, time
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1); s.bind(('127.0.0.1',5833)); s.listen(8)
def serve(c):
    try:
        while True: c.sendall(b'hb;'); time.sleep(0.3)
    except OSError: pass
while True:
    c,_=s.accept(); threading.Thread(target=serve,args=(c,),daemon=True).start()" &
  KIDS+=("$!"); sleep 0.5
  VJ="$T/VJ"; VA=(--link virtual --instance 7 --run-dir "$T/vrun" --no-watch)
  check "link.sh start --link virtual" env JETSON_DATA_DIR="$VJ" bash "$PC/link.sh" start "${VA[@]}"
  check "the four socket files exist" bash -c "for f in mavlink cam_sdk cam_http cam_rtsp; do [ -S '$VJ/sock/'\$f.sock ] || exit 1; done"
  got=$(python3 -c "
import socket; u=socket.socket(socket.AF_UNIX); u.settimeout(5); u.connect('$VJ/sock/mavlink.sock'); print(u.recv(16))" 2>&1)
  check "door A reaches SITL SERIAL2 ($got)" has "$got" "hb;"
  check "link.sh status (up, exit 0)" env JETSON_DATA_DIR="$VJ" bash "$PC/link.sh" status "${VA[@]}"
  check "link.sh drop 2 --mode close" env JETSON_DATA_DIR="$VJ" bash "$PC/link.sh" drop 2 "${VA[@]}" --mode close
  lp=$(awk -F= '$1 == "LOCAL_PID" {print $2}' "$ASTRA_SIM_HOME/state/jetson_link.env")
  check "link.sh stop" env JETSON_DATA_DIR="$VJ" bash "$PC/link.sh" stop "${VA[@]}"
  sleep 0.5
  check "the local stand-in stopped" bash -c "! kill -0 ${lp:-999999} 2>/dev/null"
  ( JETSON_DATA_DIR="$VJ" bash "$PC/link.sh" status "${VA[@]}" >/dev/null 2>&1 ); rc=$?
  check "link.sh status after stop: exit 14" [ "$rc" = 14 ]
  # The watcher must never hold the caller's pipe: demo.sh reads link.sh through "| sed", and a watcher that kept
  # stderr made that pipe never end (the first virtual-Jetson flight hung there, 2026-10-31 15:33). With the watcher
  # ON, "link.sh start ... | cat" must end by itself; the outer timeout (exit 124) catches a hang.
  VW=(--link virtual --instance 7 --run-dir "$T/vrun2")
  timeout 25 env JETSON_DATA_DIR="$VJ" PCSH="$PC/link.sh" bash -c 'bash "$PCSH" start "$@" 2>&1 | cat > /dev/null; exit "${PIPESTATUS[0]}"' _ "${VW[@]}"
  rc=$?
  check "link.sh start with its watcher, read through a pipe, ends by itself (exit $rc; 124 = hung)" [ "$rc" = 0 ]
  wp=$(awk -F= '$1 == "WATCH_PID" {print $2}' "$ASTRA_SIM_HOME/state/jetson_link.env")
  check "the watcher runs (pid ${wp:-none})" bash -c "kill -0 ${wp:-999999}"
  for _ in 1 2 3 4 5 6 7 8 9 10; do grep -q "watcher started" "$T/vrun2/jetson/link.log" 2>/dev/null && break; sleep 0.5; done
  check "the watcher writes into the link log, not the caller's terminal" grep -q "watcher started" "$T/vrun2/jetson/link.log"
  check "link.sh stop (watcher on)" env JETSON_DATA_DIR="$VJ" bash "$PC/link.sh" stop "${VW[@]}"
  for _ in 1 2 3 4 5 6; do kill -0 "${wp:-999999}" 2>/dev/null || break; sleep 0.5; done
  check "the watcher stopped within 3 s" bash -c "! kill -0 ${wp:-999999} 2>/dev/null"
else
  echo " --   $J/lib/hil_relay.py not there yet (skipped)"
fi

echo
echo "passed: $PASS   failed: $FAIL"
[ "$FAIL" = 0 ]
