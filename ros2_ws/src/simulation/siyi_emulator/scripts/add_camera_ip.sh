#!/usr/bin/env bash
# =============================================================================
#  add_camera_ip.sh  --  give this computer the SIYI camera's address
#
#  PURPOSE: the team's camera driver (video_cam) always talks to the camera at
#  192.168.144.25. In the simulator the camera is the siyi_emulator program on
#  THIS computer, so this computer must also answer to 192.168.144.25. This
#  script adds that address to the "loopback" network interface (lo), which only
#  this computer can reach, and allows normal users to open port 82 (the
#  camera's web server port). Safe to run again: it skips what is already done.
#
#  HOW TO RUN:   sudo bash add_camera_ip.sh            (192.168.144.25)
#                sudo bash add_camera_ip.sh 10.1.2.3   (another address)
#  UNDO:         sudo bash remove_camera_ip.sh
#  NOTE: the address disappears at reboot (or "wsl --shutdown"); run it again then.
# =============================================================================
set -euo pipefail

IP="${1:-192.168.144.25}"

if [ "$(id -u)" -ne 0 ]; then
  echo "This changes network settings, so it needs admin power. Run:  sudo bash $0 ${1:-}"
  exit 1
fi
if ! command -v ip >/dev/null 2>&1; then
  echo "The 'ip' command is missing. Install it with: sudo apt-get install -y iproute2"
  exit 1
fi

# 1. The camera address on the loopback interface (a /32 = exactly this one address).
if ip -4 addr show dev lo | grep -qw "inet ${IP}"; then
  echo "OK: ${IP} is already on the loopback interface."
else
  ip addr add "${IP}/32" dev lo
  echo "Added ${IP}/32 to the loopback interface."
fi

# 2. Port 82 is below 1024; let normal users use ports from 80 up (Docker does the same).
current="$(sysctl -n net.ipv4.ip_unprivileged_port_start 2>/dev/null || echo 1024)"
if [ "${current}" -gt 82 ]; then
  sysctl -w net.ipv4.ip_unprivileged_port_start=80 >/dev/null
  echo "Normal users may now open ports from 80 up (was ${current}). For a permanent setting see"
  echo "simulation/setup/01_install_humble.sh (/etc/sysctl.d/60-astra-sim.conf)."
else
  echo "OK: normal users may already open port 82 (ip_unprivileged_port_start=${current})."
fi

echo "Check:  ping -c1 ${IP}   and start the emulator:  ros2 run siyi_emulator siyi_emulator"
