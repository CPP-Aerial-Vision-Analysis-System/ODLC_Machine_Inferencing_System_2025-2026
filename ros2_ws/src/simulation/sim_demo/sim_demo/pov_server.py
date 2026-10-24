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
    topic      /sim/camera/image   the camera picture (sensor_msgs/Image: rgb8, bgr8, rgba8, bgra8 or mono8)
    host       127.0.0.1           where to listen ("0.0.0.0" = every network of this computer; only for a lab)
    port       8081                TCP port
    max_fps    10.0                at most this many pictures per second
    quality    75                  JPEG quality 10..95
    max_width  960                pictures wider than this are shrunk (0 = keep the size)
"""

from __future__ import annotations

import io
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple

#: The multipart boundary word of the stream.
BOUNDARY = 'astraframe'
#: Most viewers at the same time (each one holds a thread).
MAX_CLIENTS = 8
#: How long a stream waits for a new picture before it repeats the newest one (keeps the connection alive).
KEEPALIVE_S = 1.0

_ENCODINGS = {'rgb8': (3, False), 'bgr8': (3, True), 'rgba8': (4, False), 'bgra8': (4, True), 'mono8': (1, False)}


# ===================================================================================================
# Pictures
# ===================================================================================================
def encode_frame(width: int, height: int, encoding: str, step: int, data: bytes, *, quality: int = 75,
                 max_width: int = 960) -> bytes:
    """A ROS image (width, height, encoding, row length in bytes, pixel bytes) as a JPEG.

    Supports rgb8, bgr8, rgba8, bgra8 and mono8. Rows may be padded (`step` larger than width x channels). The picture
    is shrunk to `max_width` pixels wide when it is wider. ValueError for an unsupported encoding or data that is
    too short for the stated size, so a corrupt message is skipped instead of producing garbage.
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
    buffer = io.BytesIO()
    image.save(buffer, 'JPEG', quality=int(min(95, max(10, quality))))
    return buffer.getvalue()


def placeholder_jpeg(text: str = 'waiting for the camera ...', size: Tuple[int, int] = (640, 360)) -> bytes:
    """A dark grey picture with a line of text, served while no real picture exists yet."""
    from PIL import Image, ImageDraw

    image = Image.new('RGB', size, (38, 42, 48))
    draw = ImageDraw.Draw(image)
    box = draw.textbbox((0, 0), text)
    draw.text(((size[0] - (box[2] - box[0])) / 2, (size[1] - (box[3] - box[1])) / 2), text, fill=(200, 205, 212))
    buffer = io.BytesIO()
    image.save(buffer, 'JPEG', quality=80)
    return buffer.getvalue()


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
    from sensor_msgs.msg import Image

    rclpy.init(args=args)
    node = Node('pov_server')
    declared = {'topic': '/sim/camera/image', 'host': '127.0.0.1', 'port': 8081, 'max_fps': 10.0, 'quality': 75,
                'max_width': 960}
    for name, default in declared.items():
        node.declare_parameter(name, default)
    topic, host, port = (node.get_parameter(n).value for n in ('topic', 'host', 'port'))
    max_fps = float(node.get_parameter('max_fps').value)
    quality, max_width = int(node.get_parameter('quality').value), int(node.get_parameter('max_width').value)
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

    def on_image(msg: Image) -> None:
        """Encode at most max_fps pictures per second; a bad picture is counted and skipped, never raised."""
        now = time.monotonic()
        if now - state['last'] < min_gap:
            return
        state['last'] = now
        try:
            store.put(encode_frame(msg.width, msg.height, msg.encoding, msg.step, bytes(msg.data),
                                   quality=quality, max_width=max_width))
        except Exception as exc:                                     # noqa: BLE001 - one bad picture must not stop the feed
            state['errors'] += 1
            if now - state['last_error_log'] > 30.0:
                state['last_error_log'] = now
                node.get_logger().warning(f'skipped a picture ({exc.__class__.__name__}: {exc}); {state["errors"]} so far')

    node.create_subscription(Image, topic, on_image, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
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
