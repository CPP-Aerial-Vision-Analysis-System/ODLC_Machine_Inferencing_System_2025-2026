"""
siyi_emulator/netutil.py -- PURPOSE: decide which IP address the emulator listens on.

The team's video_cam talks to the camera at 192.168.144.25 (its default camera_ip). The emulator can
only answer there if that address exists on this computer, so the helper script
scripts/add_camera_ip.sh adds it to the loopback interface (sudo ip addr add 192.168.144.25/32 dev lo).
Rules (parameter bind_ip):
  * bind_ip set            -> use it as given;
  * bind_ip empty (default) and camera_ip exists here -> listen on camera_ip;
  * otherwise              -> listen on 127.0.0.1 and warn that video_cam then needs camera_ip:=127.0.0.1.
Standard library only.
"""

from __future__ import annotations

import errno
import socket
from typing import Optional, Tuple

FALLBACK_BIND_IP = "127.0.0.1"


def address_is_local(ip: str) -> bool:
    """True when ``ip`` is assigned to an interface of this computer (we can bind a socket to it).

    Uses a throw-away UDP bind on port 0: the kernel refuses (EADDRNOTAVAIL) an address it does
    not own, which is exactly the question. No root needed, nothing is sent.
    """
    if not ip:
        return False
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind((ip, 0))
        return True
    except OSError:
        return False


def resolve_bind_ip(camera_ip: str, bind_ip: str = "") -> Tuple[str, Optional[str]]:
    """Pick the listen address.

    Args:
        camera_ip: the address clients expect the camera at (default 192.168.144.25).
        bind_ip:   explicit override ('' = automatic).

    Returns:
        ``(ip, warning)`` where warning is None or a sentence to log once.
    """
    if bind_ip and bind_ip.strip():
        return bind_ip.strip(), None
    if address_is_local(camera_ip):
        return camera_ip, None
    warning = (
        f"camera address {camera_ip} does not exist on this computer, so the emulator listens on "
        f"{FALLBACK_BIND_IP} instead. The team's video_cam then needs '-p camera_ip:={FALLBACK_BIND_IP}'. "
        f"To keep video_cam's defaults, add the address once per boot: "
        f"sudo bash $(ros2 pkg prefix siyi_emulator)/share/siyi_emulator/scripts/add_camera_ip.sh"
    )
    return FALLBACK_BIND_IP, warning


def explain_bind_error(exc: OSError, ip: str, port: int, proto: str) -> str:
    """Turn a bind() failure into a sentence that says how to fix it."""
    code = getattr(exc, "errno", None)
    if code == errno.EACCES and port < 1024:
        return (f"no permission to listen on {proto} port {port}. Allow normal users to use ports from 80 up "
                f"(once): sudo sysctl -w net.ipv4.ip_unprivileged_port_start=80   "
                f"(simulation/setup/01_install_humble.sh makes it permanent)")
    if code == errno.EADDRINUSE:
        return (f"{proto} {ip}:{port} is already used by another program (another emulator, or a sim from "
                f"another WSL distro: both distros share one network). Stop it, or start this one with a "
                f"different bind_ip (e.g. 127.0.0.3) / port")
    if code == errno.EADDRNOTAVAIL:
        return (f"address {ip} does not exist on this computer. Run scripts/add_camera_ip.sh or set "
                f"bind_ip:=127.0.0.1")
    return f"cannot listen on {proto} {ip}:{port}: {exc}"
