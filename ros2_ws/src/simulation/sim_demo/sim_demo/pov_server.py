"""
sim_demo/pov_server.py -- PURPOSE: show the drone's camera picture inside MISSION PLANNER. The gimbal camera of the
simulated drone publishes its live picture as the ROS topic /sim/camera/image. Mission Planner cannot read a ROS
topic, but it CAN show an "MJPEG" web stream. This node turns the topic into one: it re-encodes the newest picture
as a JPEG a few times a second and serves it over plain HTTP, on this computer only:

    http://127.0.0.1:8081/pov.mjpg    the live stream (what Mission Planner opens)
    http://127.0.0.1:8081/pov.jpg     the newest single picture (open it in a browser to check)
    http://127.0.0.1:8081/            a small page with the picture and the three clicks for Mission Planner
    http://127.0.0.1:8081/health      one line: how many frames, how old the newest one is

In Mission Planner (Windows): right-click the blue-and-brown HUD -> Video -> "Set MJPEG Source" -> paste
http://127.0.0.1:8081/pov.mjpg -> OK. The picture replaces the HUD background (right-click -> Video -> "Start/Stop"
to toggle). Mission Planner on Windows reaches this program in WSL through localhost forwarding.

Why MJPEG: Mission Planner's GStreamer option needs GStreamer installed on Windows; MJPEG needs nothing.

GIMBAL ANGLES ON THE PICTURE (parameter `overlay`, on by default). A small dark box in a corner of every picture
says where the camera points, for example:

    gimbal pitch -90.0 deg, yaw 0.0 deg, roll 0.0 deg       <- where the camera points, against the horizon
    joints (vs drone): pitch -101.3, yaw 0.0, roll 4.2 deg   <- the three gimbal motors, measured in Gazebo

  * The numbers come from the simulator's measured gimbal joints (/sim/gimbal/joint_states, bridged from Gazebo's
    JointStatePublisher, 50 per second) and the drone's attitude (/sim/drone/odometry). They are NOT the commanded
    angles: if the gimbal lagged or fought, the box would show it.
  * Line 1 uses the SIYI signs: pitch and roll against the HORIZON (pitch -90 = straight down, + = up; roll + =
    right side down), yaw against the drone's NOSE (+ = left). These are the numbers the SIYI emulator sends the
    team's video_cam in its attitude reply (SDK 0x0D) while it stabilises the camera, which is its default
    (siyi_emulator parameter stabilize:=true). The formulas are the emulator's (siyi_emulator.gimbal_control
    GimbalModel.reported_attitude); a test checks that both give the same numbers. Started with stabilize:=false
    the emulator reports the raw joints instead (= line 2), while line 1 still shows the angles against the horizon.
  * Both split the angles in the gimbal's own motor order (yaw -> roll -> pitch), so the numbers stay steady while
    the camera looks straight down (a camera 0.4 deg past straight down reads pitch -90.4, roll 0). That order is
    our assumption: no SIYI document or real A8 mini log shows how the real camera reports angles near straight
    down (see siyi_emulator/README.md).
  * Line 2 is the raw joint angles, relative to the drone body: while the drone tilts in flight they move to keep
    line 1 steady (that is the stabiliser working).
  * No angle data for 2 s (simulator stopped, bridge missing) -> the box says so instead of showing old numbers.
    No drone attitude -> line 1 assumes a level drone and says so.

Safety and good manners:
  * It only READS the camera topic. The flight, the photos and the other nodes are not touched; a crash here changes
    nothing else.
  * It listens on 127.0.0.1 (this computer only) by default, serves at most MAX_CLIENTS viewers at once, and never
    blocks the ROS callback: the callback only encodes (a few milliseconds) and stores the newest picture.
  * Pictures are shrunk to `max_width` pixels and limited to `max_fps`, so a stream is about 100-300 KB/s.
  * While no picture has arrived yet (Gazebo still starting) it serves a grey "waiting for the camera" picture, and
    when the camera goes quiet it repeats the newest picture once a second, so a viewer never sees a dead connection.

The module imports ROS lazily and PIL/numpy only when a picture is encoded, so everything except the ROS node itself
is tested without ROS (test/test_pov_server.py).

Parameters of the node (ros2 run sim_demo pov_server --ros-args -p port:=8082):
    topic             /sim/camera/image         the camera picture (sensor_msgs/Image: rgb8, bgr8, rgba8, bgra8, mono8)
    host              127.0.0.1                 where to listen ("0.0.0.0" = every network of this computer; only for a lab)
    port              8081                      TCP port
    max_fps           10.0                      at most this many pictures per second
    quality           75                        JPEG quality 10..95
    max_width         960                       pictures wider than this are shrunk (0 = keep the size)
    overlay           true                      draw the gimbal angles on the picture (false = the bare picture)
    overlay_position  top_left                  where the angle box goes: top_left or bottom_left
    gimbal_topic      /sim/gimbal/joint_states  measured gimbal joints (sensor_msgs/JointState)
    odometry_topic    /sim/drone/odometry       drone attitude (nav_msgs/Odometry); "" = assume a level drone
"""

from __future__ import annotations

import functools
import io
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Sequence, Tuple

#: The multipart boundary word of the stream.
BOUNDARY = 'astraframe'
#: Most viewers at the same time (each one holds a thread).
MAX_CLIENTS = 8
#: How long a stream waits for a new picture before it repeats the newest one (keeps the connection alive).
KEEPALIVE_S = 1.0

_ENCODINGS = {'rgb8': (3, False), 'bgr8': (3, True), 'rgba8': (4, False), 'bgra8': (4, True), 'mono8': (1, False)}

#: Where the gimbal-angle box may go on the picture.
OVERLAY_POSITIONS = ('top_left', 'bottom_left')
#: Names of the three gimbal joints in the drone model (sim_models iris_a8mini), by role.
GIMBAL_JOINTS = {'yaw': 'gimbal_yaw', 'roll': 'gimbal_roll', 'pitch': 'gimbal_pitch'}
#: Angle data older than this many seconds counts as missing (the box then says so).
GIMBAL_STALE_S = 2.0


# ===================================================================================================
# Pictures
# ===================================================================================================
def encode_frame(width: int, height: int, encoding: str, step: int, data: bytes, *, quality: int = 75,
                 max_width: int = 960, overlay_lines: Optional[Sequence[str]] = None,
                 overlay_position: str = 'top_left') -> bytes:
    """A ROS image (width, height, encoding, row length in bytes, pixel bytes) as a JPEG.

    Supports rgb8, bgr8, rgba8, bgra8 and mono8. Rows may be padded (`step` larger than width x channels). The picture
    is shrunk to `max_width` pixels wide when it is wider. ValueError for an unsupported encoding or data that is
    too short for the stated size, so a corrupt message is skipped instead of producing garbage.
    `overlay_lines` (optional): lines of text drawn in a dark box at `overlay_position` after shrinking, so the text
    keeps the same size whatever the camera resolution (see draw_overlay). None or [] = the bare picture.
    """
    import numpy as np
    from PIL import Image

    if encoding not in _ENCODINGS:
        raise ValueError(f'unsupported image encoding {encoding!r} (use rgb8, bgr8, rgba8, bgra8 or mono8)')
    channels, swap = _ENCODINGS[encoding]
    if not (isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0):
        raise ValueError('the picture has no size')
    if not (isinstance(step, int) and step >= width * channels):
        raise ValueError('the row length is shorter than one row of pixels')
    if len(data) < step * height:
        raise ValueError(f'the picture holds {len(data)} bytes but {step * height} are needed')
    rows = np.frombuffer(data, dtype=np.uint8, count=step * height).reshape(height, step)[:, :width * channels]
    array = rows.reshape(height, width, channels) if channels > 1 else rows.reshape(height, width)
    if channels == 4:
        array = array[:, :, :3] if not swap else array[:, :, [2, 1, 0]]
    elif swap:
        array = array[:, :, ::-1]
    image = Image.fromarray(np.ascontiguousarray(array), 'L' if channels == 1 else 'RGB')
    if max_width and width > max_width:
        image = image.resize((max_width, max(1, round(height * max_width / width))), Image.BILINEAR)
    if overlay_lines:
        image = draw_overlay(image, overlay_lines, overlay_position)
    buffer = io.BytesIO()
    image.save(buffer, 'JPEG', quality=int(min(95, max(10, quality))))
    return buffer.getvalue()


@functools.lru_cache(maxsize=8)
def _font(size: int):
    """A readable font of about `size` pixels: DejaVu Sans Bold (Ubuntu), Arial Bold (Windows), else PIL's own."""
    from PIL import ImageFont

    for name in ('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 'DejaVuSans-Bold.ttf', 'arialbd.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()          # small but always there


def _text_box(draw, text: str, font=None) -> Tuple[int, int, int, int]:
    """(left, top, right, bottom) of `text` drawn at (0, 0).

    Pillow 9.0 (Ubuntu 22.04's python3-pil) can only measure TrueType fonts with textbbox and raises ValueError for
    its built-in bitmap font; then the older getsize() measures it. Newer Pillow measures every font with textbbox.
    """
    try:
        return draw.textbbox((0, 0), text, font=font)
    except (ValueError, AttributeError):
        from PIL import ImageFont

        width, height = (font or ImageFont.load_default()).getsize(text)
        return 0, 0, width, height


def draw_overlay(image, lines: Sequence[str], position: str = 'top_left'):
    """Draw `lines` of text, white on a half-transparent dark box, in a corner of a PIL image.

    The text height is 1/30 of the picture height (18 px on a 960 x 540 stream), so it stays readable when Mission
    Planner shrinks the picture into its HUD. position: 'top_left' (default) or 'bottom_left'; anything else counts
    as top_left. A grey (mono8) picture is turned into colour first. Returns the image that was drawn on.
    """
    from PIL import ImageDraw

    if image.mode != 'RGB':
        image = image.convert('RGB')
    width, height = image.size
    size = max(10, round(height / 30))
    font = _font(size)
    draw = ImageDraw.Draw(image, 'RGBA')                 # RGBA drawing on an RGB picture = see-through box
    boxes = [_text_box(draw, line, font) for line in lines]
    gap = max(2, size // 4)
    pad = max(3, size // 3)
    text_w = max(b[2] - b[0] for b in boxes)
    line_h = max(b[3] - b[1] for b in boxes) + gap
    box_w = min(width - 2, text_w + 2 * pad)
    box_h = min(height - 2, line_h * len(lines) - gap + 2 * pad)
    x0 = pad
    y0 = height - box_h - pad if position == 'bottom_left' else pad
    draw.rectangle([x0, y0, x0 + box_w, y0 + box_h], fill=(0, 0, 0, 150))
    for index, (line, box) in enumerate(zip(lines, boxes)):
        draw.text((x0 + pad - box[0], y0 + pad + index * line_h - box[1]), line, font=font, fill=(245, 245, 245, 255))
    return image


def placeholder_jpeg(text: str = 'waiting for the camera ...', size: Tuple[int, int] = (640, 360)) -> bytes:
    """A dark grey picture with a line of text, served while no real picture exists yet."""
    from PIL import Image, ImageDraw

    image = Image.new('RGB', size, (38, 42, 48))
    draw = ImageDraw.Draw(image)
    box = _text_box(draw, text)              # (draw.textbbox alone fails on Ubuntu's Pillow 9.0 with this font)
    draw.text(((size[0] - (box[2] - box[0])) / 2, (size[1] - (box[3] - box[1])) / 2), text, fill=(200, 205, 212))
    buffer = io.BytesIO()
    image.save(buffer, 'JPEG', quality=80)
    return buffer.getvalue()


# ===================================================================================================
# Gimbal angles (the text of the overlay). Plain math, no numpy, no ROS.
# ===================================================================================================
def _rot_x(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _rot_y(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(a: float):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _rot_quat(x: float, y: float, z: float, w: float):
    """Rotation matrix of an (x, y, z, w) quaternion (normalised here; a zero quaternion gives 'level')."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return _rot_z(0.0)
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def siyi_angles(yaw_joint: float, roll_joint: float, pitch_joint: float,
                drone_quat_xyzw: Optional[Tuple[float, float, float, float]] = None) -> Tuple[float, float, float]:
    """(yaw, pitch, roll) in DEGREES as the SIYI camera reports them, from the gimbal joints (radians) and the drone.

    Joints follow the drone model's chain body -> yaw (axis +Z) -> roll (axis +X) -> pitch (axis -Y) -> camera,
    so the camera's rotation in the body frame is Rz(yaw) Rx(roll) Ry(-pitch). drone_quat_xyzw = the drone's
    orientation in the world (ENU, the odometry quaternion); None = a level drone.
    Returned like the real camera: yaw = the yaw joint (relative to the nose, + = left); pitch and roll =
    against the horizon (pitch + = up, -90 = straight down, and smoothly on past -90; roll + = right side down).
    Pitch and roll are split in the gimbal's own yaw -> roll -> pitch order, R_world_cam = Rz(h) Rx(roll)
    Ry(-pitch), like siyi_emulator.gimbal_control.gimbal_earth_angles: steady at straight down (the old Z-Y-X
    split gave a random roll there) and only undefined at roll +-90 deg, far outside the gimbal's range.
    """
    body = _rot_quat(*drone_quat_xyzw) if drone_quat_xyzw is not None else _rot_z(0.0)
    cam = _mul(body, _mul(_rot_z(yaw_joint), _mul(_rot_x(roll_joint), _rot_y(-pitch_joint))))
    s = max(-1.0, min(1.0, cam[2][1]))
    roll = math.asin(s)
    pitch = -math.atan2(-cam[2][0], cam[2][2]) if abs(s) < 0.999999 else 0.0
    return math.degrees(yaw_joint), math.degrees(pitch), math.degrees(roll)


def _deg_text(value: float) -> str:
    """One decimal, and never '-0.0' (a tiny negative number would look like a real angle)."""
    return f'{round(value, 1) + 0.0:.1f}'


class GimbalReadout:
    """The newest measured gimbal joints and drone attitude, and the overlay text made from them (no ROS here).

    The ROS node feeds it from its callbacks (set_joints, set_attitude); encode_frame draws lines(). clock = a
    monotonic seconds function (a fake one in the tests). joint_names = role -> joint name in the drone model; a
    message name matches when its last part (after '::' or '/') is that name, so scoped names work too.
    """

    def __init__(self, clock=time.monotonic, stale_s: float = GIMBAL_STALE_S, joint_names=None):
        self._clock = clock
        self.stale_s = float(stale_s)
        self.joint_names = dict(joint_names or GIMBAL_JOINTS)
        self._joints: Optional[Tuple[float, float, float]] = None     # radians (yaw, roll, pitch)
        self._joints_at: Optional[float] = None
        self._quat: Optional[Tuple[float, float, float, float]] = None
        self._quat_at: Optional[float] = None

    def set_joints(self, names: Sequence[str], positions: Sequence[float]) -> bool:
        """Take one JointState (names + positions, radians). True when all three gimbal joints were in it."""
        found = {}
        for name, position in zip(names, positions):
            short = re.split(r'::|/', str(name))[-1]
            for role, joint in self.joint_names.items():
                if short == joint:
                    found[role] = float(position)
        if len(found) != 3 or not all(math.isfinite(v) for v in found.values()):
            return False
        self._joints = (found['yaw'], found['roll'], found['pitch'])
        self._joints_at = self._clock()
        return True

    def set_attitude(self, x: float, y: float, z: float, w: float) -> bool:
        """Take the drone orientation (world ENU quaternion). False (ignored) for a zero or non-finite one."""
        values = (float(x), float(y), float(z), float(w))
        if not all(math.isfinite(v) for v in values) or sum(v * v for v in values) < 1e-12:
            return False
        self._quat = values
        self._quat_at = self._clock()
        return True

    def lines(self) -> list:
        """The overlay text: two lines with the angles, or one line saying why there are none."""
        now = self._clock()
        if self._joints is None:
            return ['gimbal: no angle data yet']
        age = now - self._joints_at
        if age > self.stale_s:
            return [f'gimbal: no angle data for {age:.0f} s']
        yaw_j, roll_j, pitch_j = self._joints
        level = self._quat is None or now - self._quat_at > self.stale_s
        yaw, pitch, roll = siyi_angles(yaw_j, roll_j, pitch_j, None if level else self._quat)
        first = f'gimbal pitch {_deg_text(pitch)} deg, yaw {_deg_text(yaw)} deg, roll {_deg_text(roll)} deg'
        if level:
            first += ' (drone attitude unknown: level drone assumed)'
        second = (f'joints (vs drone): pitch {_deg_text(math.degrees(pitch_j))}, yaw {_deg_text(math.degrees(yaw_j))}, '
                  f'roll {_deg_text(math.degrees(roll_j))} deg')
        return [first, second]


class FrameStore:
    """The newest JPEG, shared between the ROS callback (writer) and the HTTP threads (readers)."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._seq = 0
        self._stamp: Optional[float] = None

    def put(self, jpeg: bytes) -> None:
        """Store a new newest picture and wake the waiting streams."""
        with self._cond:
            self._jpeg, self._seq, self._stamp = jpeg, self._seq + 1, self._clock()
            self._cond.notify_all()

    def latest(self) -> Tuple[int, Optional[bytes]]:
        """(sequence number, newest JPEG or None). The number grows with every put()."""
        with self._cond:
            return self._seq, self._jpeg

    def wait_newer(self, seq: int, timeout_s: float) -> Tuple[int, Optional[bytes]]:
        """Wait up to `timeout_s` for a picture newer than `seq`; returns the newest one either way."""
        deadline = self._clock() + max(0.0, timeout_s)
        with self._cond:
            while self._seq <= seq:
                remaining = deadline - self._clock()
                if remaining <= 0:
                    break
                self._cond.wait(remaining)
            return self._seq, self._jpeg

    def age_s(self) -> Optional[float]:
        """Seconds since the newest picture arrived, or None before the first one."""
        with self._cond:
            return None if self._stamp is None else max(0.0, self._clock() - self._stamp)

    @property
    def count(self) -> int:
        """How many pictures have been stored so far."""
        with self._cond:
            return self._seq


# ===================================================================================================
# The web server
# ===================================================================================================
def _page(url: str) -> bytes:
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ASTRA camera POV</title><style>
:root{{--bg:#f6f7f9;--fg:#1b2430;--mu:#5d6b7c;--card:#fff;--ln:#d5dbe3}}
@media (prefers-color-scheme:dark){{:root{{--bg:#11161d;--fg:#e6edf3;--mu:#9aa7b5;--card:#19212b;--ln:#2d3946}}}}
body{{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif}}
img{{max-width:100%;border:1px solid var(--ln);border-radius:8px;background:var(--card)}}
ol{{max-width:60ch}}code{{background:var(--card);border:1px solid var(--ln);padding:1px 6px;border-radius:4px}}
p{{color:var(--mu)}}</style></head><body><h1>ASTRA camera POV</h1>
<img src="/pov.mjpg" alt="live camera picture">
<p>This is the picture of the drone's gimbal camera. To see it inside <b>Mission Planner</b>:</p>
<ol><li>Right-click the HUD (the blue-and-brown artificial horizon).</li>
<li>Choose <b>Video</b> then <b>Set MJPEG Source</b>.</li>
<li>Paste <code>{url}/pov.mjpg</code> and press OK.</li></ol>
<p>Single picture: <a href="/pov.jpg">/pov.jpg</a> &middot; status: <a href="/health">/health</a></p></body></html>""".encode()


class PovServer:
    """Serve the pictures of a FrameStore over HTTP (see the module text). `start()` returns self."""

    def __init__(self, store: FrameStore, host: str = '127.0.0.1', port: int = 8081, *, keepalive_s: float = KEEPALIVE_S,
                 max_clients: int = MAX_CLIENTS):
        self.store = store
        self.host = host
        self.port = port
        self.keepalive_s = keepalive_s
        self.max_clients = max_clients
        self._clients = 0
        self._clients_lock = threading.Lock()
        self._placeholder: Optional[bytes] = None
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def url(self) -> str:
        """The address viewers use (this computer's loopback when listening on every address)."""
        shown = '127.0.0.1' if self.host in ('', '0.0.0.0') else self.host
        return f'http://{shown}:{self.port}'

    def start(self) -> 'PovServer':
        """Start listening in a background thread. OSError (with a plain-words hint) when the port is taken."""
        if self._httpd is not None:
            return self
        owner = self

        class Handler(_Handler):
            """The request handler of THIS server."""

            server_owner = owner

        try:
            self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        except OSError as exc:
            raise OSError(f'cannot listen on {self.host}:{self.port} ({exc.strerror or exc}). Another program (an '
                          f'earlier camera feed?) may be using the port: stop it, or choose another port with '
                          f'--ros-args -p port:=8082.') from exc
        self._httpd.daemon_threads = True
        self.port = self._httpd.server_address[1]
        self._thread = threading.Thread(target=self._httpd.serve_forever, kwargs={'poll_interval': 0.1},
                                        name='pov-server', daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        """Stop listening. Safe to call twice."""
        httpd, self._httpd = self._httpd, None
        if httpd is not None:
            httpd.shutdown()
            httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5)
            self._thread = None

    def placeholder(self) -> bytes:
        """The grey 'waiting' picture (made once)."""
        if self._placeholder is None:
            self._placeholder = placeholder_jpeg()
        return self._placeholder

    def enter(self) -> bool:
        """Reserve a viewer slot; False when the server is full."""
        with self._clients_lock:
            if self._clients >= self.max_clients:
                return False
            self._clients += 1
            return True

    def leave(self) -> None:
        """Give a viewer slot back."""
        with self._clients_lock:
            self._clients = max(0, self._clients - 1)

    @property
    def clients(self) -> int:
        """How many streams are open right now."""
        with self._clients_lock:
            return self._clients


class _Handler(BaseHTTPRequestHandler):
    """One request. `server_owner` is filled in by PovServer.start()."""

    server_owner: PovServer
    protocol_version = 'HTTP/1.0'                    # the stream ends when the connection closes: no chunking needed
    server_version = 'AstraPov'

    def log_message(self, fmt, *args) -> None:
        """Keep the console quiet: a stream would otherwise log every connection."""
        return None

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:                         # noqa: N802 (http.server's naming)
        """Route: the page, one picture, the stream, the status line."""
        owner = self.server_owner
        path = self.path.split('?')[0]
        try:
            if path == '/pov.mjpg':
                self._stream(owner)
            elif path == '/pov.jpg':
                _, jpeg = owner.store.latest()
                self._send(200, jpeg or owner.placeholder(), 'image/jpeg')
            elif path == '/health':
                age = owner.store.age_s()
                text = f'ok frames={owner.store.count} newest_age_s={"-" if age is None else f"{age:.1f}"} viewers={owner.clients}\n'
                self._send(200, text.encode(), 'text/plain')
            elif path in ('/', '/index.html'):
                self._send(200, _page(owner.url), 'text/html; charset=utf-8')
            else:
                self._send(404, b'not found\n', 'text/plain')
        except (BrokenPipeError, ConnectionError, TimeoutError):
            return                                    # the viewer closed the window: nothing to do

    def _stream(self, owner: PovServer) -> None:
        """Send pictures until the viewer goes away (multipart/x-mixed-replace)."""
        if not owner.enter():
            self._send(503, b'too many viewers\n', 'text/plain')
            return
        try:
            self.send_response(200)
            self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={BOUNDARY}')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Pragma', 'no-cache')
            self.end_headers()
            seq = -1
            while owner._httpd is not None:
                seq, jpeg = owner.store.wait_newer(seq, owner.keepalive_s)
                payload = jpeg or owner.placeholder()          # repeats the newest one when the camera is quiet
                self.wfile.write(f'--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(payload)}\r\n\r\n'.encode())
                self.wfile.write(payload)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        finally:
            owner.leave()


# ===================================================================================================
# The ROS node
# ===================================================================================================
def main(args=None) -> None:
    """`ros2 run sim_demo pov_server`: serve /sim/camera/image as an MJPEG stream (see the module text)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image, JointState

    rclpy.init(args=args)
    node = Node('pov_server')
    declared = {'topic': '/sim/camera/image', 'host': '127.0.0.1', 'port': 8081, 'max_fps': 10.0, 'quality': 75,
                'max_width': 960, 'overlay': True, 'overlay_position': 'top_left',
                'gimbal_topic': '/sim/gimbal/joint_states', 'odometry_topic': '/sim/drone/odometry'}
    for name, default in declared.items():
        node.declare_parameter(name, default)
    topic, host, port = (node.get_parameter(n).value for n in ('topic', 'host', 'port'))
    max_fps = float(node.get_parameter('max_fps').value)
    quality, max_width = int(node.get_parameter('quality').value), int(node.get_parameter('max_width').value)
    overlay = bool(node.get_parameter('overlay').value)
    position = str(node.get_parameter('overlay_position').value)
    if position not in OVERLAY_POSITIONS:
        node.get_logger().warning(f'overlay_position "{position}" is unknown (use {" or ".join(OVERLAY_POSITIONS)}); '
                                  'using top_left')
        position = 'top_left'
    min_gap = 1.0 / max_fps if math.isfinite(max_fps) and max_fps > 0 else 0.0
    store = FrameStore()
    server = PovServer(store, host, int(port))
    try:
        server.start()
    except OSError as exc:
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)
    state = {'last': 0.0, 'errors': 0, 'last_error_log': 0.0}
    readout = GimbalReadout()
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)   # newest only; works with any publisher

    def on_image(msg: Image) -> None:
        """Encode at most max_fps pictures per second; a bad picture is counted and skipped, never raised."""
        now = time.monotonic()
        if now - state['last'] < min_gap:
            return
        state['last'] = now
        try:
            store.put(encode_frame(msg.width, msg.height, msg.encoding, msg.step, bytes(msg.data),
                                   quality=quality, max_width=max_width,
                                   overlay_lines=readout.lines() if overlay else None, overlay_position=position))
        except Exception as exc:                                     # noqa: BLE001 - one bad picture must not stop the feed
            state['errors'] += 1
            if now - state['last_error_log'] > 30.0:
                state['last_error_log'] = now
                node.get_logger().warning(f'skipped a picture ({exc.__class__.__name__}: {exc}); {state["errors"]} so far')

    node.create_subscription(Image, topic, on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    if overlay:
        gimbal_topic = str(node.get_parameter('gimbal_topic').value)
        odometry_topic = str(node.get_parameter('odometry_topic').value)
        node.create_subscription(JointState, gimbal_topic, lambda m: readout.set_joints(m.name, m.position), sensor_qos)
        attitude_from = 'a level drone (odometry_topic is empty)'
        if odometry_topic:
            try:
                from nav_msgs.msg import Odometry
            except ImportError:                                      # nav_msgs missing: angles against a level drone
                Odometry = None
                attitude_from = 'a level drone (nav_msgs is not installed)'
            if Odometry is not None:
                node.create_subscription(Odometry, odometry_topic, lambda m: readout.set_attitude(
                    m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
                    m.pose.pose.orientation.w), sensor_qos)
                attitude_from = odometry_topic
        node.get_logger().info(f'gimbal angles drawn on the picture ({position}) from {gimbal_topic} and {attitude_from}; '
                               '-p overlay:=false shows the bare picture')
    node.get_logger().info(f'camera feed for Mission Planner: {server.url}/pov.mjpg  (from {topic}). In Mission Planner: '
                           f'right-click the HUD > Video > Set MJPEG Source > paste that address. Preview in a browser: {server.url}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
