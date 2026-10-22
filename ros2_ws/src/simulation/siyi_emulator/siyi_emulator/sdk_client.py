"""
siyi_emulator/sdk_client.py -- PURPOSE: a tiny SIYI SDK client (UDP 37260 + media HTTP 82) for tests
and for poking a camera by hand. It works against the emulator AND a real SIYI camera, so students
can check what the hardware answers. Standard library only (socket + urllib).

Command line (executable ``siyi_client``; add --ip 127.0.0.3 etc. when the emulator listens elsewhere):

    ros2 run siyi_emulator siyi_client hwid                 # model check (0x02)
    ros2 run siyi_emulator siyi_client info                 # 0x0A system info
    ros2 run siyi_emulator siyi_client attitude             # 0x0D
    ros2 run siyi_emulator siyi_client angles 30 -45        # 0x0E yaw +30 (left), pitch -45
    ros2 run siyi_emulator siyi_client axis pitch -90       # 0x41
    ros2 run siyi_emulator siyi_client rotate 50 0 --seconds 1   # 0x07 for 1 s then stop
    ros2 run siyi_emulator siyi_client center               # 0x08
    ros2 run siyi_emulator siyi_client zoom 5               # 0x0F, then shows 0x18 after 3 s
    ros2 run siyi_emulator siyi_client mode lock            # 0x0C func 3 (then 0x19 / 0x0A)
    ros2 run siyi_emulator siyi_client photo --out p.jpg    # 0x0C + wait 0x0B + list + download
    ros2 run siyi_emulator siyi_client list                 # media listing
    ros2 run siyi_emulator siyi_client format --yes         # 0x48 (deletes every photo!)
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple
from urllib.parse import quote, urlencode, urlsplit
from urllib.request import urlopen

from . import sdk_protocol as sp


class SiyiClient:
    """Minimal SIYI SDK client.

    Args:
        ip:          camera address (default 192.168.144.25).
        port:        SDK UDP port (37260).
        media_port:  media HTTP port (82).
        timeout:     seconds to wait for each reply (2.0 like video_cam).
    """

    def __init__(self, ip: str = "192.168.144.25", port: int = 37260, media_port: int = 82,
                 timeout: float = 2.0) -> None:
        self.ip = ip
        self.port = int(port)
        self.media_port = int(media_port)
        self.timeout = float(timeout)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(self.timeout)
        self.seq = 0
        self.unsolicited: List[sp.Frame] = []     # frames received while waiting for something else

    # ------------------------------------------------------------------ plumbing
    def close(self) -> None:
        """Close the UDP socket."""
        self.sock.close()

    def __enter__(self) -> "SiyiClient":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def send(self, cmd_id: int, data: bytes = b"", need_ack: bool = True, seq: Optional[int] = None) -> None:
        """Send one frame (CTRL 0x01 when need_ack, else 0x00)."""
        if seq is None:
            seq = self.seq
            self.seq = (self.seq + 1) & 0xFFFF
        frame = sp.build_frame(cmd_id, data, seq, sp.CTRL_NEED_ACK if need_ack else 0x00)
        self.sock.sendto(frame, (self.ip, self.port))

    def wait_for(self, expect: Iterable[int], timeout: Optional[float] = None) -> Optional[sp.Frame]:
        """Wait for a frame whose cmd_id is in ``expect``; other frames are kept in ``unsolicited``."""
        expect = set(expect)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                return None
            for frame in sp.iter_frames(data):
                if frame.cmd_id in expect:
                    return frame
                self.unsolicited.append(frame)

    def request(self, cmd_id: int, data: bytes = b"", expect: Optional[Iterable[int]] = None,
                timeout: Optional[float] = None) -> Optional[sp.Frame]:
        """Send a command and return its reply frame (None after ``timeout``)."""
        self.send(cmd_id, data)
        return self.wait_for(expect or {cmd_id}, timeout)

    # ------------------------------------------------------------------ SDK commands
    def hardware_id(self) -> Optional[str]:
        """0x02 -> hardware ID string ('6B...' ZR10, '73...' A8 mini) or None."""
        reply = self.request(sp.CMD_HARDWARE_ID)
        return reply.data.decode("ascii", "ignore").strip("\x00").strip() if reply else None

    def firmware(self) -> Optional[Tuple[str, ...]]:
        """0x01 -> ('v0.3.4', 'v0.3.9', ...) or None."""
        reply = self.request(sp.CMD_FIRMWARE_VERSION)
        if not reply or len(reply.data) < 4:
            return None
        words = struct.unpack("<" + "I" * (len(reply.data) // 4), reply.data[: len(reply.data) // 4 * 4])
        return tuple(f"v{(w >> 16) & 0xFF}.{(w >> 8) & 0xFF}.{w & 0xFF}" for w in words)

    def system_info(self) -> Optional[Dict[str, int]]:
        """0x0A -> {'hdr', 'record_sta', 'mode', 'mounting', 'video_output', 'raw'} or None."""
        reply = self.request(sp.CMD_SYSTEM_INFO)
        if not reply or len(reply.data) < 5:
            return None
        d = reply.data
        return {"hdr": d[1], "record_sta": d[3], "mode": d[4], "mounting": d[5] if len(d) > 5 else -1,
                "video_output": d[6] if len(d) > 6 else -1, "length": len(d)}

    def attitude(self) -> Optional[Dict[str, float]]:
        """0x0D -> {'yaw','pitch','roll','yaw_rate','pitch_rate','roll_rate'} (deg, deg/s) or None."""
        reply = self.request(sp.CMD_GIMBAL_ATTITUDE)
        if not reply or len(reply.data) < 12:
            return None
        keys = ("yaw", "pitch", "roll", "yaw_rate", "pitch_rate", "roll_rate")
        return dict(zip(keys, sp.decode_attitude(reply.data)))

    def set_angles(self, yaw: float, pitch: float) -> Optional[Tuple[float, float, float]]:
        """0x0E -> the attitude reported in the ACK (yaw, pitch, roll) or None."""
        reply = self.request(sp.CMD_SET_ANGLES, sp.encode_set_angles_request(yaw, pitch))
        return sp.decode_angles(reply.data) if reply and len(reply.data) >= 6 else None

    def single_axis(self, axis: str, angle: float) -> Optional[Tuple[int, Tuple[float, float, float]]]:
        """0x41 (axis 'yaw' or 'pitch') -> (reply cmd_id, (yaw, pitch, roll)) or None. A8 mini replies 0x0E."""
        flag = 0 if axis.lower().startswith("y") else 1
        reply = self.request(sp.CMD_SINGLE_AXIS, struct.pack("<hB", sp.deg_to_raw(angle), flag),
                             expect={sp.CMD_SINGLE_AXIS, sp.CMD_SET_ANGLES})
        return (reply.cmd_id, sp.decode_angles(reply.data)) if reply and len(reply.data) >= 6 else None

    def _ok(self, reply: Optional[sp.Frame]) -> bool:
        return bool(reply) and (not reply.data or reply.data[0] == 1)

    def rotate(self, yaw_speed: int, pitch_speed: int) -> bool:
        """0x07 speeds -100..100 (yaw + = right, pitch + = up). 0, 0 stops."""
        return self._ok(self.request(sp.CMD_GIMBAL_ROTATION, struct.pack("<bb", int(yaw_speed), int(pitch_speed))))

    def center(self, pos: int = 1) -> bool:
        """0x08 one-key centre (pos 1) or 'centre downward' (pos 2)."""
        return self._ok(self.request(sp.CMD_CENTER, bytes([pos])))

    def absolute_zoom(self, zoom: float) -> bool:
        """0x0F absolute zoom (x)."""
        integer, tenth = sp.zoom_to_int_tenth(zoom)
        return self._ok(self.request(sp.CMD_ABSOLUTE_ZOOM, bytes([integer, tenth])))

    def current_zoom(self) -> Optional[float]:
        """0x18 -> zoom factor or None (a real A8 mini never answers)."""
        reply = self.request(sp.CMD_CURRENT_ZOOM)
        return sp.decode_zoom(reply.data) if reply and len(reply.data) >= 2 else None

    def zoom_range(self) -> Optional[float]:
        """0x16 -> max zoom or None."""
        reply = self.request(sp.CMD_ZOOM_RANGE)
        return sp.decode_zoom(reply.data) if reply and len(reply.data) >= 2 else None

    def manual_zoom(self, direction: int) -> Optional[float]:
        """0x05 (1 in, -1 out, 0 stop) -> current zoom or None."""
        reply = self.request(sp.CMD_MANUAL_ZOOM, struct.pack("<b", int(direction)))
        return sp.decode_zoom_x10(reply.data) if reply and len(reply.data) >= 2 else None

    def auto_focus(self, x: int = 960, y: int = 540) -> bool:
        """0x04 at a video-stream pixel."""
        return self._ok(self.request(sp.CMD_AUTO_FOCUS, struct.pack("<BHH", 1, x, y)))

    def manual_focus(self, direction: int) -> bool:
        """0x06 (1 far, -1 near, 0 stop)."""
        return self._ok(self.request(sp.CMD_MANUAL_FOCUS, struct.pack("<b", int(direction))))

    def gimbal_mode(self) -> Optional[str]:
        """0x19 -> 'lock' | 'follow' | 'fpv' or None."""
        reply = self.request(sp.CMD_GIMBAL_MODE)
        return sp.MODE_NAMES.get(reply.data[0], str(reply.data[0])) if reply and reply.data else None

    def set_mode(self, mode: str) -> None:
        """0x0C func 3/4/5 (never acknowledged)."""
        func = {"lock": sp.FUNC_LOCK_MODE, "follow": sp.FUNC_FOLLOW_MODE, "fpv": sp.FUNC_FPV_MODE}[mode.lower()]
        self.send(sp.CMD_CAPTURE, bytes([func]))

    def take_photo(self, feedback_timeout: float = 0.8, same_packet_as_video_cam: bool = True) -> Optional[int]:
        """0x0C func 0, then wait for the 0x0B push. Returns its info_type (0 = OK) or None.

        With ``same_packet_as_video_cam`` the exact constant packet of video_cam/config.py:131 is sent
        (SEQ 0 every time).
        """
        if same_packet_as_video_cam:
            self.sock.sendto(bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce"), (self.ip, self.port))
        else:
            self.send(sp.CMD_CAPTURE, bytes([sp.FUNC_TAKE_PHOTO]), need_ack=True)
        reply = self.wait_for({sp.CMD_FUNCTION_FEEDBACK, sp.CMD_CAPTURE}, feedback_timeout)
        return reply.data[0] if reply and reply.data else None

    def format_sd(self) -> None:
        """0x48 (never acknowledged): deletes every photo."""
        self.send(sp.CMD_FORMAT_SD, need_ack=False)

    # ------------------------------------------------------------------ media HTTP
    @property
    def media_root(self) -> str:
        """'http://ip:port'."""
        return f"http://{self.ip}:{self.media_port}"

    def http_json(self, api: str, **params: Any) -> Dict[str, Any]:
        """GET /cgi-bin/media.cgi/api/v1/<api>?params -> decoded JSON."""
        url = f"{self.media_root}/cgi-bin/media.cgi/api/v1/{api}?{urlencode(params)}"
        with urlopen(url, timeout=10) as resp:
            return json.loads(resp.read().decode("utf-8"))

    def directories(self) -> List[Dict[str, str]]:
        """getdirectories -> list of {'name', 'path'}."""
        doc = self.http_json("getdirectories", media_type=0)
        return doc.get("data", {}).get("directories", []) if doc.get("success") else []

    def media_list(self, path: Optional[str] = None, start: int = 0, count: int = 9999) -> List[Dict[str, Any]]:
        """getmedialist of ``path`` (default: the last directory, like video_cam)."""
        if path is None:
            dirs = self.directories()
            if not dirs:
                return []
            path = dirs[-1]["path"]
        doc = self.http_json("getmedialist", media_type=0, path=path, start=start, count=count)
        return doc.get("data", {}).get("list", []) if doc.get("success") else []

    def download(self, url_or_name: str) -> bytes:
        """Download a photo by its listing URL (any host is rewritten to ip:media_port) or file name."""
        if "://" in url_or_name:
            path = urlsplit(url_or_name).path
        else:
            dirs = self.directories()
            path = "/" + quote(f"{dirs[-1]['path']}/{url_or_name}") if dirs else "/" + url_or_name
        with urlopen(self.media_root + path, timeout=10) as resp:
            return resp.read()

    def take_and_download(self, listing_timeout: float = 15.0) -> Tuple[Optional[int], Optional[Dict[str, Any]], bytes]:
        """Do what video_cam does: photo, 0x0B, then poll the listing until a new file appears and fetch it.

        Returns (0x0B info_type or None, listing entry or None, JPEG bytes (empty if none)).
        """
        before = {item["name"] for item in self.media_list()}
        info = self.take_photo()
        deadline = time.monotonic() + listing_timeout
        while time.monotonic() < deadline:
            items = self.media_list()
            new = [item for item in items if item["name"] not in before]
            if new:
                return info, new[-1], self.download(new[-1]["url"])
            time.sleep(0.25)
        return info, None, b""


# ---------------------------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------------------------
def _print(obj: Any) -> None:
    print(json.dumps(obj, indent=2) if isinstance(obj, (dict, list)) else obj)


def main(argv: Optional[List[str]] = None) -> int:
    """``siyi_client`` command line (see the module docstring)."""
    p = argparse.ArgumentParser(prog="siyi_client", description="Talk to a SIYI camera (or the emulator).")
    p.add_argument("--ip", default="192.168.144.25")
    p.add_argument("--port", type=int, default=37260)
    p.add_argument("--media-port", type=int, default=82)
    p.add_argument("--timeout", type=float, default=2.0)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name in ("hwid", "firmware", "info", "attitude", "zoom-current", "zoom-range", "list", "dirs"):
        sub.add_parser(name)
    s = sub.add_parser("angles"); s.add_argument("yaw", type=float); s.add_argument("pitch", type=float)
    s = sub.add_parser("axis"); s.add_argument("axis", choices=["yaw", "pitch"]); s.add_argument("angle", type=float)
    s = sub.add_parser("rotate"); s.add_argument("yaw", type=int); s.add_argument("pitch", type=int)
    s.add_argument("--seconds", type=float, default=0.0, help="then send 0,0 (stop)")
    s = sub.add_parser("center"); s.add_argument("pos", type=int, nargs="?", default=1)
    s = sub.add_parser("zoom"); s.add_argument("zoom", type=float)
    s = sub.add_parser("zoom-manual"); s.add_argument("direction", choices=["in", "out", "stop"])
    s = sub.add_parser("mode"); s.add_argument("mode", nargs="?", choices=["lock", "follow", "fpv"])
    s = sub.add_parser("photo"); s.add_argument("--out", default="")
    s = sub.add_parser("download"); s.add_argument("name"); s.add_argument("--out", required=True)
    s = sub.add_parser("format"); s.add_argument("--yes", action="store_true")
    args = p.parse_args(argv)

    with SiyiClient(args.ip, args.port, args.media_port, args.timeout) as c:
        if args.cmd == "hwid":
            _print(c.hardware_id())
        elif args.cmd == "firmware":
            _print(c.firmware())
        elif args.cmd == "info":
            _print(c.system_info())
        elif args.cmd == "attitude":
            _print(c.attitude())
        elif args.cmd == "zoom-current":
            _print(c.current_zoom())
        elif args.cmd == "zoom-range":
            _print(c.zoom_range())
        elif args.cmd == "dirs":
            _print(c.directories())
        elif args.cmd == "list":
            _print(c.media_list())
        elif args.cmd == "angles":
            _print(c.set_angles(args.yaw, args.pitch))
        elif args.cmd == "axis":
            _print(c.single_axis(args.axis, args.angle))
        elif args.cmd == "rotate":
            _print(c.rotate(args.yaw, args.pitch))
            if args.seconds > 0:
                time.sleep(args.seconds)
                _print(c.rotate(0, 0))
        elif args.cmd == "center":
            _print(c.center(args.pos))
        elif args.cmd == "zoom":
            _print(c.absolute_zoom(args.zoom))
            time.sleep(3.0)
            _print({"zoom_after_3s": c.current_zoom()})
        elif args.cmd == "zoom-manual":
            _print(c.manual_zoom({"in": 1, "out": -1, "stop": 0}[args.direction]))
        elif args.cmd == "mode":
            if args.mode:
                c.set_mode(args.mode)
                time.sleep(0.2)
            _print({"0x19": c.gimbal_mode(), "0x0A": c.system_info()})
        elif args.cmd == "photo":
            info, item, data = c.take_and_download()
            _print({"feedback_0x0B": info, "file": item, "bytes": len(data)})
            if args.out and data:
                with open(args.out, "wb") as handle:
                    handle.write(data)
            return 0 if data else 1
        elif args.cmd == "download":
            data = c.download(args.name)
            with open(args.out, "wb") as handle:
                handle.write(data)
            _print({"bytes": len(data), "out": args.out})
        elif args.cmd == "format":
            if not args.yes:
                print("format deletes every photo on the card: add --yes", file=sys.stderr)
                return 1
            c.format_sd()
            _print("0x48 sent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
