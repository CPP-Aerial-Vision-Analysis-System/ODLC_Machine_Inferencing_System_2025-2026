#!/usr/bin/env bash
# =============================================================================
#  remove_camera_ip.sh  --  undo add_camera_ip.sh
#
#  PURPOSE: removes the SIYI camera address (default 192.168.144.25) from the
#  loopback interface again. Only do this when no emulator or simulation still
#  uses it (another WSL distro shares the same network!). Safe to run twice.
#
#  HOW TO RUN:   sudo bash remove_camera_ip.sh [address]
# =============================================================================
set -euo pipefail

IP="${1:-192.168.144.25}"

if [ "$(id -u)" -ne 0 ]; then
  echo "This changes network settings, so it needs admin power. Run:  sudo bash $0 ${1:-}"
  exit 1
fi
if ip -4 addr show dev lo | grep -qw "inet ${IP}"; then
  ip addr del "${IP}/32" dev lo
  echo "Removed ${IP} from the loopback interface."
else
  echo "OK: ${IP} was not on the loopback interface."
fi
