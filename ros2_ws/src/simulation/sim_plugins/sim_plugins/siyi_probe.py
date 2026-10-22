"""
sim_plugins/siyi_probe.py -- PURPOSE: knock on a SIYI-compatible camera the way the team's video_cam does, to
check the CAMERA slot (the siyi_emulator, or a teammate's own camera program) from outside:

    probe_sdk(ip)     UDP 37260: send the SDK "request hardware ID" frame (command 0x02) and wait for the
                      camera's answer (a frame with the same command)
    probe_media(ip)   TCP 82: ask the media web API for its photo folders (getdirectories?media_type=0) and
                      check the answer is JSON with a "data" part

The SIYI SDK frame (SIYI "Gimbal Camera SDK" manual; video_cam/camera_interface.py builds the same):

    0x55 0x66 | CTRL (1) | DATA_LEN (2, little-endian) | SEQ (2, LE) | CMD_ID (1) | DATA | CRC16 (2, LE)

CTRL 0x01 = "please answer"; CRC16 = CRC-16/XMODEM (polynomial 0x1021, start 0) over every byte before it.
Example from the team's code: the photo command is always 55 66 01 01 00 00 00 0c 00 34 ce.
Only the Python standard library is used (socket, urllib, json), so it runs anywhere.
"""

from __future__ import annotations

import json
import socket
import struct
import urllib.error
import urllib.request
from typing import Optional, Tuple

STX = b'\x55\x66'
CMD_HARDWARE_ID = 0x02
SDK_PORT = 37260
MEDIA_PORT = 82
MEDIA_DIRS_PATH = '/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0'


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM of `data` (polynomial 0x1021, initial value 0, no reflection), as the SIYI SDK uses."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_frame(cmd: int, data: bytes = b'', seq: int = 0, need_ack: bool = True) -> bytes:
    """One SIYI SDK frame (see the module docstring)."""
    head = STX + bytes([0x01 if need_ack else 0x00]) + struct.pack('<HH', len(data), seq & 0xFFFF)
    body = head + bytes([cmd & 0xFF]) + data
    return body + struct.pack('<H', crc16_xmodem(body))


def parse_frame(buf: bytes) -> Optional[Tuple[int, bytes]]:
    """(command id, data) of the first valid frame in `buf`, or None (wrong start bytes, too short, bad CRC)."""
    start = buf.find(STX)
    if start < 0 or len(buf) - start < 10:
        return None
    frame = buf[start:]
    length = struct.unpack('<H', frame[3:5])[0]
    total = 8 + length + 2
    if len(frame) < total:
        return None
    body, crc = frame[:8 + length], struct.unpack('<H', frame[8 + length:total])[0]
    if crc16_xmodem(body) != crc:
        return None
    return frame[7], bytes(frame[8:8 + length])


def probe_sdk(ip: str, port: int = SDK_PORT, timeout: float = 2.0) -> Tuple[bool, str]:
    """Ask for the hardware ID over UDP. Returns (answered, plain-words result)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.settimeout(timeout)
            sock.sendto(build_frame(CMD_HARDWARE_ID), (ip, int(port)))
            while True:
                data, _ = sock.recvfrom(4096)
                parsed = parse_frame(data)
                if parsed and parsed[0] == CMD_HARDWARE_ID:
                    hw = parsed[1].decode('ascii', 'replace').strip('\x00 ')
                    return True, f'SIYI SDK answers on UDP {ip}:{port} (hardware id "{hw}")'
    except socket.timeout:
        return False, (f'no answer to the SDK hardware-id request on UDP {ip}:{port} within {timeout:g} s '
                       '(is the camera emulator running, and does this computer have that address? '
                       'simulation/setup/camera_ip_alias.sh)')
    except OSError as exc:
        return False, f'cannot talk to UDP {ip}:{port}: {exc}'


def probe_media(ip: str, port: int = MEDIA_PORT, timeout: float = 3.0) -> Tuple[bool, str]:
    """Ask the media web API for its photo folders. Returns (works, plain-words result)."""
    url = f'http://{ip}:{port}{MEDIA_DIRS_PATH}'
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:      # noqa: S310 - fixed local URL
            payload = json.loads(resp.read().decode('utf-8', 'replace'))
    except (urllib.error.URLError, OSError) as exc:
        return False, f'media web API {url} does not answer: {exc}'
    except ValueError:
        return False, f'media web API {url} answered, but not with JSON'
    if not isinstance(payload, dict) or 'data' not in payload:
        return False, f'media web API {url} answered JSON without a "data" part: {str(payload)[:120]}'
    return True, f'media web API answers on http://{ip}:{port} ({str(payload.get("data"))[:80]})'
