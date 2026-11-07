#!/usr/bin/env bash
# =============================================================================
#  simulation/jetson/test/test_jetson_cli.sh  --  the whole jetson.sh path against a FAKE Jetson (no board needed)
#
#  PURPOSE: prove the first-connection kit works end to end before the owner plugs in the real board: argument
#  handling, the check rows and their exit codes (unreachable host 3, closed SSH port 4, not logged in 17, team busy
#  16, no docker access 8, wrong JetPack 7, forwards refused 14, all good 0), setup, the deploy and start command
#  lines (dry runs), start/exec/stop exit codes, clock, status --json and target --format json. The "Jetson" is this
#  PC: test/fake/ssh runs the remote commands here with a fake home, fake board files (/etc/nv_tegra_release ...),
#  a fake process table and fake board tools (docker, nvpmodel, tegrastats, sudo without a password), and makes
#  REAL forwards for check row 12. Light: no Docker, no simulator, no locks needed. Runs in WSL (bash, python3).
#  HOW TO RUN:  bash simulation/jetson/test/test_jetson_cli.sh        (prints PASS/FAIL per test; exit 0 = all passed)
# =============================================================================
set -u
T_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
J="$T_DIR/../jetson.sh"
TMP="$(mktemp -d /tmp/jet_cli_test.XXXX)"
export FAKE_STATE="$TMP/state" FAKE_HOME="$TMP/home" FAKE_ROOT="$TMP/root" FAKE_PROC="$TMP/proc"
export JETSON_STATE_DIR="$TMP/pcstate" JETSON_CM_PATH="$TMP/cm-jetson" JETSON_SSH_BIN="$T_DIR/fake/ssh"
export ASTRA_SIM_HOME="$TMP/astra_sim"
unset JETSON_HOST JETSON_USER JETSON_PORT JETSON_AUTH JETSON_LINK
mkdir -p "$FAKE_STATE" "$FAKE_HOME" "$FAKE_PROC" "$FAKE_ROOT/etc/ssh" "$FAKE_ROOT/proc/device-tree" "$JETSON_STATE_DIR"
chmod +x "$T_DIR/fake/ssh" "$T_DIR/fake/board/"*
printf 'NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super\0' > "$FAKE_ROOT/proc/device-tree/model"
board_release() { echo "# R$1 (release), REVISION: $2, GCID: 42132812, BOARD: generic, EABI: aarch64" > "$FAKE_ROOT/etc/nv_tegra_release"; }
board_release 36 4.7
printf 'ID=ubuntu\nVERSION_ID="22.04"\n' > "$FAKE_ROOT/etc/os-release"
printf 'MemTotal:        7802880 kB\nMemAvailable:    4508672 kB\n' > "$FAKE_ROOT/proc/meminfo"
printf '< POWER_MODEL ID=0 NAME=15W >\n< POWER_MODEL ID=1 NAME=25W >\n< POWER_MODEL ID=2 NAME=MAXN_SUPER >\n' > "$FAKE_ROOT/etc/nvpmodel.conf"
# the fake board says aarch64 (remote_facts asks uname -m): a tiny uname wrapper on the fake board's PATH
cat > "$T_DIR/fake/board/uname" <<'EOF'
#!/bin/bash
# test/fake/board/uname -- PURPOSE: the fake Jetson is an aarch64 board (uname -m), whatever this PC is.
case "${1:-}" in -m) echo aarch64 ;; -r) echo 5.15.148-tegra ;; *) /bin/uname "$@" ;; esac
EOF
chmod +x "$T_DIR/fake/board/uname"
# a control socket file for "-S" (jetson_cm_check looks for a socket file)
python3 -c "import socket,sys; s=socket.socket(socket.AF_UNIX); s.bind(sys.argv[1])" "$JETSON_CM_PATH"
# an "SSH port" that answers (TCP) on this PC
SSH_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
python3 -c "
import socket,time
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); s.bind(('127.0.0.1',$SSH_PORT)); s.listen(50)
s.settimeout(600)
while True:
    try: c,_=s.accept(); c.close()
    except Exception: break" &
LISTENER=$!
CLOSED_PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
cleanup() {
  kill "$LISTENER" 2>/dev/null
  for f in "$FAKE_STATE"/fwd_*.pid; do [ -f "$f" ] && kill "$(cat "$f")" 2>/dev/null; done
  pkill -f "exec -a mavros_node" 2>/dev/null
  rm -rf "$TMP" "$T_DIR/fake/board/uname"
}
trap cleanup EXIT

PASS=0; FAIL=0
T=(--host 127.0.0.1 --port "$SSH_PORT" --user astra-dev)
run_j() { OUT=$(bash "$J" "$@" 2>&1); RC=$?; }
expect() {   # expect NAME WANT_RC [GREP_PATTERN]
  local name="$1" want="$2" pat="${3:-}"
  if [ "$RC" = "$want" ] && { [ -z "$pat" ] || grep -qE -- "$pat" <<<"$OUT"; }; then
    PASS=$((PASS + 1)); echo "PASS $name"
  else
    FAIL=$((FAIL + 1)); echo "FAIL $name (exit $RC, wanted $want${pat:+, pattern '$pat'})"; printf '%s\n' "$OUT" | tail -15 | sed 's/^/      /'
  fi
}

run_j --help;                      expect "help" 0 "HOW TO RUN"
run_j frobnicate;                  expect "unknown command -> 2" 2 "unknown command"
run_j check --link warp;           expect "unknown link -> 2" 2 "unknown link"
run_j check --link ssh;            expect "ssh without host -> 2" 2 "needs the Jetson's address"
run_j check --bogus "${T[@]}";     expect "unknown option -> 2" 2 "unknown option"
run_j check --host 192.0.2.1 --port 22;   expect "unreachable host -> 3" 3 "cannot reach 192.0.2.1"
run_j check --host 127.0.0.1 --port "$CLOSED_PORT"; expect "closed SSH port -> 4" 4 "SSH port .* is closed"
run_j check "${T[@]}";             expect "not logged in -> 17" 17 "Not logged in to astra-dev@127.0.0.1.*jetson.sh\" login"
touch "$FAKE_STATE/logged_in"
run_j check "${T[@]}";             expect "idle board, all good -> 0" 0 "ALL GOOD"
grep -q "\[ 12\] SSH forwards allowed" <<<"$OUT" && { PASS=$((PASS+1)); echo "PASS row 12 real forward test"; } || { FAIL=$((FAIL+1)); echo "FAIL row 12 real forward test"; }
grep -q "MAXN_SUPER" <<<"$OUT" && grep -q "sudo nvpmodel -m <ID" <<<"$OUT" && { PASS=$((PASS+1)); echo "PASS power mode reported, never changed"; } || { FAIL=$((FAIL+1)); echo "FAIL power mode row"; }
grep -q "clock offset" <<<"$OUT" && { PASS=$((PASS+1)); echo "PASS clock row"; } || { FAIL=$((FAIL+1)); echo "FAIL clock row"; }
touch "$FAKE_STATE/forward_fail"
run_j check "${T[@]}";             expect "forwards refused -> 14" 14 "forward test failed"
rm -f "$FAKE_STATE/forward_fail"
# the team is busy: a "mavros_node" in the fake process table
mkdir -p "$FAKE_PROC/1197"; printf '/opt/ros/humble/lib/mavros/mavros_node\0--ros-args\0' > "$FAKE_PROC/1197/cmdline"
run_j check "${T[@]}" --quick;     expect "team busy -> 16" 16 "the team is using the Jetson right now.*mavros_node pid 1197"
run_j check "${T[@]}" --quick --team-agreed; expect "team busy + --team-agreed -> 0" 0 "ALL GOOD"
run_j start --sim "${T[@]}";       expect "start refuses while busy -> 16" 16 "team is using"
rm -rf "$FAKE_PROC/1197"
touch "$FAKE_STATE/docker_none"
run_j check "${T[@]}" --quick;     expect "no docker access -> 8" 8 "sudo usermod -aG docker [a-z]"
rm -f "$FAKE_STATE/docker_none"
touch "$FAKE_STATE/no_nvidia"
run_j check "${T[@]}" --quick;     expect "no NVIDIA runtime -> 9" 9 "nvidia-container"
rm -f "$FAKE_STATE/no_nvidia"
board_release 35 4.1
run_j check "${T[@]}" --quick;     expect "wrong JetPack -> 7" 7 "R35.4.1"
board_release 36 4.7
run_j setup "${T[@]}";             expect "setup -> 0" 0 "saved the target"
[ -d "$FAKE_HOME/astra_hil/sock" ] && [ -x "$FAKE_HOME/astra_hil/bin/astra_hil.sh" ] && grep -q "JETSON_HOST=127.0.0.1" "$JETSON_STATE_DIR/target.env" \
  && { PASS=$((PASS+1)); echo "PASS setup made ~/astra_hil and target.env"; } || { FAIL=$((FAIL+1)); echo "FAIL setup files"; }
# after setup no option is needed: the saved target is used
run_j target;                      expect "saved target used" 0 "JETSON_HOST=127.0.0.1"
run_j target --format json;        expect "target json" 0 '"JETSON_DATA": ".*astra_hil"'
run_j clock;                       expect "clock" 0 "^offset_s=-?[0-9]+\.[0-9]+ rtt_ms="
run_j check --for flight --quick;  expect "flight check without an image -> 12" 12 "astra-hil:jetson is not on .* yet"
run_j start --sim --dry-run --run-id T1; expect "start --sim --dry-run" 0 "--network none --ipc shareable --cap-add NET_ADMIN.*-e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1"
grep -q -- "--runtime nvidia" <<<"$OUT" && grep -q -- "--label org.astra.owner=hil" <<<"$OUT" && { PASS=$((PASS+1)); echo "PASS sim line has the GPU runtime and our label"; } || { FAIL=$((FAIL+1)); echo "FAIL sim line flags"; }
run_j start --real --dry-run;      expect "start --real --dry-run" 0 "--network host --ipc host.*--device /dev/ttyUSB0.*ROS_DOMAIN_ID=0"
run_j start;                       expect "start without a mode -> 2" 2 "--sim"
run_j exec -- true;                expect "exec when not running -> 13" 13 "is not running"
echo "abc1234" > "$FAKE_STATE/image"
run_j start --sim --run-id T2;     expect "start --sim -> 0 (relay listening)" 0 "relay listens"
grep -q "docker run -d --init --name astra-hil" "$FAKE_STATE/docker.log" && { PASS=$((PASS+1)); echo "PASS docker run was sent"; } || { FAIL=$((FAIL+1)); echo "FAIL docker run not sent"; }
run_j start --sim --run-id T3;     expect "start twice -> 1" 1 "already running"
run_j exec -- bash -c 'exit 3';    expect "exec passes the exit code" 3
run_j exec -- echo hello-from-stack; expect "exec output" 0 "hello-from-stack"
run_j status --json;               expect "status --json" 0 '"logged_in": true'
python3 -c "import json,sys; d=json.loads(sys.argv[1].strip().splitlines()[-1]); assert d['container']=='running' and d['l4t']=='R36.4.7', d" "$OUT" \
  && { PASS=$((PASS+1)); echo "PASS status json content"; } || { FAIL=$((FAIL+1)); echo "FAIL status json content: $OUT"; }
run_j stop;                        expect "stop" 0 "stopped"
run_j deploy --dry-run --via registry; expect "deploy --dry-run (registry)" 0 "dk pull .*127.0.0.1:29517/astra-hil:jetson-"
run_j warmup --yes;                expect "warmup without a model -> 12" 12 "no model"
run_j login;                       expect "login when already logged in" 0 "already logged in"
rm -f "$FAKE_STATE/logged_in"
run_j status --json;               expect "status when logged out -> 17" 17 '"logged_in": false'
run_j image build --flavor nope;   expect "image build bad flavor -> 2" 2 "--flavor"

# ---- the virtual Jetson (revision 4): this PC's docker is the fake one; nothing reaches a real Docker -------------
VD="$ASTRA_SIM_HOME/jetson_virtual"
run_v() { OUT=$(PATH="$T_DIR/fake/board:$PATH" bash "$J" "$@" 2>&1); RC=$?; }
cp "$JETSON_STATE_DIR/target.env" "$TMP/target.before"
run_v setup --link virtual;        expect "setup --link virtual -> 0" 0 "left as it is"
cmp -s "$JETSON_STATE_DIR/target.env" "$TMP/target.before" && { PASS=$((PASS+1)); echo "PASS virtual setup keeps the saved board target"; } \
  || { FAIL=$((FAIL+1)); echo "FAIL virtual setup changed target.env"; }
run_v target;                      expect "plain commands still mean the board" 0 "JETSON_LINK=ssh"
rm -f "$FAKE_STATE/image_trt"
run_v warmup --link virtual;       expect "virtual warmup, image without TensorRT -> 0" 0 "has no TensorRT"
echo yes > "$FAKE_STATE/image_trt"
run_v warmup --link virtual;       expect "virtual warmup without a model -> 12" 12 "no model in .*jetson_virtual/models"
mkdir -p "$VD/models" && echo pt > "$VD/models/yolo26m.pt"
: > "$FAKE_STATE/docker.log"
run_v warmup --link virtual;       expect "virtual warmup with TensorRT -> 0" 0 "engine ready"
grep -q -- "--gpus all .*-e L4T=pc .*astra-hil:virtual warmup" "$FAKE_STATE/docker.log" \
  && { PASS=$((PASS+1)); echo "PASS virtual warmup: PC GPU, L4T=pc, the virtual image"; } || { FAIL=$((FAIL+1)); echo "FAIL virtual warmup line: $(cat "$FAKE_STATE/docker.log")"; }

echo
echo "test_jetson_cli: $PASS passed, $FAIL failed"
[ "$FAIL" = 0 ]
