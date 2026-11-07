#!/bin/bash
# =============================================================================
#  simulation/jetson/pc/test/test_link_sshd.sh  --  link.sh against REAL OpenSSH (no board needed)
#
#  PURPOSE: a throwaway ubuntu:22.04 container with openssh-server stands in for the Jetson (the same OpenSSH release
#  as JetPack 6's Ubuntu 22.04). Login mode is emulated with our own SSH master on a private control socket
#  (JETSON_CM_PATH), exactly what the owner's login window is. It proves, with real sshd behaviour:
#    1. link.sh adds its doors to a RUNNING shared connection (ssh -O forward with Unix-socket -R and a -L);
#    2. the reconnect step records the connection's own sshd (comm "sshd") through the shared connection;
#    3. door A carries SITL's bytes to the Jetson side, the tap -L carries the Jetson's bytes to the PC;
#    4. after a SILENT drop (the old connection frozen, the owner logs in again), the watcher kills the old stuck
#       sshd session, the relay end on the Jetson sees end-of-file, and a new connection to door A works;
#    5. the tap door comes back as soon as the old ssh lets go of PC port 7461;
#    6. link.sh stop closes every PC port it opened.
#  SITL is a fake TCP server on 127.0.0.1:5833 (instance 7), so a simulator running on this PC is never touched.
#  Needs: Docker Engine in WSL, the ubuntu:22.04 image, internet for apt inside the throwaway container. Light job (no
#  image build, no simulator); it removes its container and files at the end.
#  Run:  bash simulation/jetson/pc/test/test_link_sshd.sh      (exit 0 = all passed)
# =============================================================================
set -u
L="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/link.sh"
T=$(mktemp -d /tmp/jet_link.XXXX); chmod 700 "$T"; mkdir -p "$T/state" "$T/home/state"
export JETSON_STATE_DIR="$T/state" JETSON_CM_PATH="$T/cm" ASTRA_SIM_HOME="$T/home"
C=jetlinksshd_$$
PASS=0; FAIL=0; SITL=""; M1=""
pass() { PASS=$((PASS + 1)); printf ' ok   %s\n' "$1"; }
fail() { FAIL=$((FAIL + 1)); printf ' FAIL %s\n' "$1"; }
check() { local name="$1"; shift; if "$@"; then pass "$name"; else fail "$name"; fi; }
has() { grep -q -- "$2" <<< "$1"; }
cleanup() {
  [ -n "$M1" ] && kill -9 "$M1" 2>/dev/null
  ssh -S "$T/cm" -O exit x 2>/dev/null
  [ -n "$SITL" ] && kill "$SITL" 2>/dev/null
  pkill -f "link.sh _watch.*--instance 7" 2>/dev/null
  docker rm -f "$C" >/dev/null 2>&1; rm -rf "$T"
}
trap cleanup EXIT

ssh-keygen -q -t ed25519 -N '' -f "$T/key"
docker run -d --rm --name "$C" ubuntu:22.04 sleep 900 >/dev/null || { echo "docker run failed (is the Docker Engine running in WSL?)"; exit 3; }
docker exec "$C" bash -c 'apt-get update -qq >/dev/null && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq openssh-server python3 procps >/dev/null && mkdir -p /run/sshd' \
  || { echo "apt inside the throwaway container failed (no internet?)"; exit 4; }
docker exec "$C" bash -c 'useradd -m -s /bin/bash -u 1000 jet && mkdir -p /home/jet/.ssh /home/jet/astra_hil/sock /home/jet/astra_hil/run && chmod 700 /home/jet/astra_hil/sock'
docker cp "$T/key.pub" "$C":/home/jet/.ssh/authorized_keys
docker exec "$C" bash -c 'chown -R jet:jet /home/jet && chmod 600 /home/jet/.ssh/authorized_keys && /usr/sbin/sshd'
IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$C"); echo "Jetson stand-in at $IP"
sleep 1; ssh-keyscan -t ed25519 "$IP" > "$T/state/known_hosts" 2>/dev/null
open_master() { ssh -M -S "$T/cm" -o ControlPersist=no -fN -i "$T/key" -o UserKnownHostsFile="$T/state/known_hosts" \
                    -o StrictHostKeyChecking=yes -o ServerAliveInterval=2 -o ServerAliveCountMax=3 jet@"$IP"; }
jet_py() { docker exec -u jet "$C" python3 -c "$1" 2>&1; }
open_master || { echo "could not open the SSH master"; exit 5; }
# fake SITL SERIAL2 (instance 7 = TCP 5833): sends hb<n>; every 0.5 s to each client
python3 -c "
import socket, threading, time
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1); s.bind(('127.0.0.1',5833)); s.listen(8)
def serve(c):
    i=0
    try:
        while True:
            i+=1; c.sendall(b'hb%d;' % i); time.sleep(0.5)
    except OSError: pass
while True:
    c,_=s.accept(); threading.Thread(target=serve,args=(c,),daemon=True).start()" &
SITL=$!; sleep 0.5
A=(--link ssh --host "$IP" --user jet --auth login --instance 7 --run-dir "$T/run")
LOG="$T/run/jetson/link.log"

echo "== 1-3. start, doors, recorded session"
bash "$L" start "${A[@]}" > "$T/start.out" 2>&1; rc=$?
check "link.sh start (exit $rc)" [ "$rc" = 0 ]
comm=$(docker exec "$C" bash -c 'ps -o comm= -p $(cut -d" " -f1 /home/jet/astra_hil/run/link_A.pid)' 2>&1)
check "the recorded session process is the connection's sshd ($comm)" [ "$comm" = sshd ]
got=$(jet_py "import socket; u=socket.socket(socket.AF_UNIX); u.settimeout(5); u.connect('/home/jet/astra_hil/sock/mavlink.sock'); print(u.recv(64))")
check "door A: SITL's bytes reach the Jetson side ($got)" has "$got" "b'hb"
docker exec -d -u jet "$C" python3 -c "
import socket
s=socket.socket(socket.AF_UNIX); s.bind('/home/jet/astra_hil/sock/tap.sock'); s.listen(1); c,_=s.accept(); c.sendall(b'tap-frame'); c.close()"
sleep 1
got=$(python3 -c "import socket; c=socket.create_connection(('127.0.0.1',7461),timeout=5); print(c.recv(64))" 2>&1)
check "tap -L: the Jetson's bytes reach the PC ($got)" has "$got" "tap-frame"
bash "$L" status "${A[@]}" >/dev/null 2>&1; check "link.sh status: up (exit 0)" [ "$?" = 0 ]

echo "== 4. a silent drop, then a new login"
docker exec -d -u jet "$C" python3 -c "
import socket, time
u=socket.socket(socket.AF_UNIX); u.connect('/home/jet/astra_hil/sock/mavlink.sock'); f=open('/tmp/reader.log','w')
while True:
    d=u.recv(64); f.write('%.2f %r\n' % (time.time(), d)); f.flush()
    if not d: break"
sleep 2
M1=$(ssh -S "$T/cm" -O check jet@"$IP" 2>&1 | sed -n 's/.*pid=\([0-9]*\).*/\1/p')
OLD=$(docker exec "$C" cut -d' ' -f1 /home/jet/astra_hil/run/link_A.pid)
kill -STOP "$M1"; rm -f "$T/cm"; open_master
for _ in $(seq 1 20); do grep -q "link back\|connection changed" "$LOG" 2>/dev/null && break; sleep 1; done
sleep 2
check "the watcher saw the new login and re-added the doors" grep -q "link back\|connection changed" "$LOG"
check "the old stuck sshd session ($OLD) was killed" grep -q "killed old session $OLD" "$LOG"
last=$(docker exec "$C" tail -1 /tmp/reader.log 2>&1)
check "the Jetson-side reader on the old door saw end-of-file ($last)" has "$last" "b''"
got=$(jet_py "import socket; u=socket.socket(socket.AF_UNIX); u.settimeout(5); u.connect('/home/jet/astra_hil/sock/mavlink.sock'); print(u.recv(64))")
check "door A works again after the reconnect ($got)" has "$got" "b'hb"

echo "== 5. the old ssh lets go of port 7461: the tap door comes back"
kill -9 "$M1" 2>/dev/null; M1=""; sleep 5
docker exec -d -u jet "$C" python3 -c "
import socket, os
p='/home/jet/astra_hil/sock/tap.sock'
os.path.exists(p) and os.unlink(p)
s=socket.socket(socket.AF_UNIX); s.bind(p); s.listen(1); c,_=s.accept(); c.sendall(b'tap-frame-2'); c.close()"
sleep 1
got=$(python3 -c "import socket; c=socket.create_connection(('127.0.0.1',7461),timeout=5); print(c.recv(64))" 2>&1)
check "tap -L works again ($got)" has "$got" "tap-frame-2"

echo "== 6. stop"
bash "$L" stop "${A[@]}" >/dev/null 2>&1; check "link.sh stop (exit 0)" [ "$?" = 0 ]
sleep 0.5
check "PC ports 37261, 37282, 7461 closed" bash -c "! ss -ltn | grep -qE ':(37261|37282|7461) '"

echo; echo "link.log:"; sed 's/^/    /' "$LOG"
echo; echo "passed: $PASS   failed: $FAIL"
[ "$FAIL" = 0 ]
