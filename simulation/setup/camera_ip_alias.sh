#!/usr/bin/env bash
# =============================================================================
#  camera_ip_alias.sh  --  give this PC the SIYI camera's address (needs admin rights; run once)
#
#  PURPOSE: the team's camera code (video_cam) always talks to the camera at 192.168.144.25, the address the
#  real SIYI camera has on the drone's network. In the simulator the "camera" is the siyi_emulator program
#  on this PC, so this PC must ALSO answer on 192.168.144.25. This script:
#    1. adds 192.168.144.25 as an extra address of the loopback device (lo) right now, and
#    2. installs a tiny system service (astra-camera-ip.service) that adds it again at every start of
#       WSL/Ubuntu (only when systemd runs, as in the Ubuntu-22.04 WSL distro), and
#    3. lets normal users open port 82 (the camera's media web server port) by setting
#       net.ipv4.ip_unprivileged_port_start=80 (01_install_humble.sh does this too).
#  Nothing leaves this PC: a loopback address is not visible on the real network.
#  WSL note: all WSL distros share one network stack, so the address is seen by every distro.
#
#  HOW TO RUN:   sudo bash camera_ip_alias.sh            (inside Ubuntu)
#     or, from Windows PowerShell (no Ubuntu password needed):
#                wsl -d Ubuntu-22.04 -u root bash /path/to/simulation/setup/camera_ip_alias.sh
#  Undo:         sudo bash camera_ip_alias.sh --remove
#  Options:      --no-boot  only add the address now (no service)
# =============================================================================
set -eo pipefail
IP="${ASTRA_CAMERA_IP:-192.168.144.25}"
UNIT=/etc/systemd/system/astra-camera-ip.service
say() { printf '\033[1;36m==> %s\033[0m\n' "$*"; }

if [ "$(id -u)" -ne 0 ]; then
  echo "This changes the PC's network settings, so it needs admin power."
  echo "Run it like this:   sudo bash $0      (or from Windows: wsl -d Ubuntu-22.04 -u root bash <this file>)"
  exit 1
fi

if [ "${1:-}" = "--remove" ]; then
  say "Removing the camera address $IP"
  if [ -f "$UNIT" ]; then systemctl disable --now astra-camera-ip.service 2>/dev/null || true; rm -f "$UNIT"; systemctl daemon-reload 2>/dev/null || true; fi
  ip addr del "$IP/32" dev lo 2>/dev/null || true
  echo "Done. The simulator's camera emulator will fall back to 127.0.0.1 (video_cam then cannot reach it)."
  exit 0
fi

say "Adding the camera address $IP to the loopback device (lo)"
if ip -4 addr show dev lo | grep -q "inet $IP/"; then
  echo "    already there."
else
  ip addr add "$IP/32" dev lo
  echo "    added."
fi

say "Allowing normal users to open port 82 (the camera's media web server)"
echo 'net.ipv4.ip_unprivileged_port_start=80' > /etc/sysctl.d/60-astra-sim.conf
sysctl -q -p /etc/sysctl.d/60-astra-sim.conf || true
echo "    ip_unprivileged_port_start = $(sysctl -n net.ipv4.ip_unprivileged_port_start)"

if [ "${1:-}" != "--no-boot" ]; then
  if [ "$(ps -p 1 -o comm= 2>/dev/null)" = "systemd" ]; then
    say "Installing astra-camera-ip.service (adds the address at every start)"
    cat > "$UNIT" <<EOF
# Installed by simulation/setup/camera_ip_alias.sh (ASTRA simulator).
# Gives this PC the SIYI camera address $IP on the loopback device, so the camera emulator can answer
# the team's video_cam exactly like the real camera. Remove with: sudo bash camera_ip_alias.sh --remove
[Unit]
Description=ASTRA simulator: SIYI camera address $IP on the loopback device
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'ip -4 addr show dev lo | grep -q "inet $IP/" || ip addr add $IP/32 dev lo'
ExecStop=/bin/sh -c 'ip addr del $IP/32 dev lo 2>/dev/null || true'

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable astra-camera-ip.service >/dev/null 2>&1
    systemctl start astra-camera-ip.service || true
    echo "    enabled (systemctl status astra-camera-ip shows it)."
  else
    echo "    systemd is not running here, so the address is NOT re-added after a restart."
    echo "    start_sim.sh / demo.sh add it again when needed (they will ask for admin rights)."
  fi
fi
say "Done: the simulated camera can now be reached at $IP"
