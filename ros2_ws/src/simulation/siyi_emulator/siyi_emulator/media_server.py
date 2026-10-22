"""
siyi_emulator/media_server.py -- PURPOSE: the camera's media web server (TCP port 82), the one
video_cam uses to find and download each photo after the shutter fires. It serves the emulated SD
card (sd_card.py) with exactly the requests and JSON shapes video_cam depends on
(video_cam.md section 4.4 and 19.B; video_cam camera_interface.py:512-601):

    GET /cgi-bin/media.cgi/api/v1/getdirectories?media_type=0
        -> {"code":200,"success":true,"message":"","data":{"directories":[{"name":D,"path":D}]}}
    GET /cgi-bin/media.cgi/api/v1/getmedialist?media_type=0&path=D&start=0&count=N
        -> {"code":200,"success":true,"message":"","data":{"list":[{"name":"IMG_0001.jpg",
            "url":"http://<host>/D/IMG_0001.jpg","size":<bytes>}, ...],"total":<all files>}}
           oldest first; start/count slice the list, "total" always counts every file
    GET /cgi-bin/media.cgi/api/v1/getmediacount?media_type=0&path=D
        -> {"code":200,"success":true,"message":"","data":{"count":N}}
    GET /D/IMG_0001.jpg   -> 200 image/jpeg bytes (404 if missing)
    GET /                 -> a small HTML page listing the card (handy in a web browser)

media_type=1 (videos) answers with empty lists. Errors use the SIYI shape with "success": false
and code 400 ("Invalid media type", "path not exist"), like SIYI's media API chapter.
The double slash of SIYI's documented base URL (http://host:82//cgi-bin/...) is accepted.

The "url" of each photo uses the Host header of the request (what the client used to reach us),
so the link always works for that client; video_cam also rewrites 192.168.144.25 -> camera_ip.
Standard library only (http.server), one thread per connection, HTTP/1.1 keep-alive.
"""

from __future__ import annotations

import html
import json
import logging
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, quote, unquote, urlsplit

from .sd_card import SdCard

API_PREFIX = "/cgi-bin/media.cgi/api/v1/"
MEDIA_TYPE_IMAGES = 0
MEDIA_TYPE_VIDEOS = 1
DEFAULT_LIST_COUNT = 1_000_000   # "everything" when the client gives no count


# ---------------------------------------------------------------------------------------------
# JSON builders (pure functions, unit-tested directly)
# ---------------------------------------------------------------------------------------------
def api_response(data: Optional[Dict[str, Any]] = None, success: bool = True, code: int = 200,
                 message: str = "") -> Dict[str, Any]:
    """The SIYI media API envelope: {"code", "data", "success", "message"}."""
    return {"code": code, "data": data if data is not None else {}, "success": success, "message": message}


def _parse_int(value: Optional[str], default: int) -> int:
    try:
        return int(str(value).strip())
    except (TypeError, ValueError):
        return default


def parse_media_type(value: Optional[str]) -> Optional[int]:
    """'0' -> 0 (images), '1' -> 1 (videos), anything else -> None (invalid). Missing -> images."""
    if value is None or str(value).strip() == "":
        return MEDIA_TYPE_IMAGES
    number = _parse_int(value, -1)
    return number if number in (MEDIA_TYPE_IMAGES, MEDIA_TYPE_VIDEOS) else None


def directories_response(sd: SdCard, media_type: Optional[int]) -> Dict[str, Any]:
    """Answer of getdirectories."""
    if media_type is None:
        return api_response({}, success=False, code=400, message="Invalid media type")
    if media_type == MEDIA_TYPE_VIDEOS:
        return api_response({"directories": []})
    return api_response({"directories": sd.directories()})


def media_list_response(sd: SdCard, media_type: Optional[int], path: str, start: int, count: int,
                        base_url: str) -> Dict[str, Any]:
    """Answer of getmedialist.

    Args:
        sd:         the card.
        media_type: 0 images, 1 videos, None invalid.
        path:       directory asked for (must be our photo directory).
        start:      index of the first item to return (>= 0).
        count:      how many items at most (video_cam asks 9999, or 1 to read "total").
        base_url:   'http://host:port' used to build each item's "url".
    """
    if media_type is None:
        return api_response({}, success=False, code=400, message="Invalid media type")
    if not sd.has_directory(path):
        return api_response({}, success=False, code=400, message="path not exist")
    if media_type == MEDIA_TYPE_VIDEOS:
        return api_response({"list": [], "total": 0})
    photos = sd.list_photos()
    start = max(0, int(start))
    count = max(0, int(count))
    items = [
        {"name": p.name, "url": f"{base_url}/{quote(p.rel_path)}", "size": p.size}
        for p in photos[start:start + count]
    ]
    return api_response({"list": items, "total": len(photos)})


def media_count_response(sd: SdCard, media_type: Optional[int], path: str) -> Dict[str, Any]:
    """Answer of getmediacount."""
    if media_type is None:
        return api_response({}, success=False, code=400, message="Invalid media type")
    if not sd.has_directory(path):
        return api_response({}, success=False, code=400, message="path not exist")
    if media_type == MEDIA_TYPE_VIDEOS:
        return api_response({"count": 0})
    return api_response({"count": sd.count()})


def index_page(sd: SdCard) -> str:
    """A tiny human-friendly HTML listing of the card (GET /)."""
    rows = "".join(
        f'<li><a href="/{quote(p.rel_path)}">{html.escape(p.name)}</a> ({p.size / 1e6:.2f} MB)</li>'
        for p in reversed(sd.list_photos())
    ) or "<li>(no photos yet)</li>"
    return (
        "<!doctype html><html><head><meta charset='utf-8'><title>SIYI emulator SD card</title></head>"
        "<body style='font-family:sans-serif'>"
        f"<h2>SIYI camera emulator: SD card</h2><p>Folder <b>{html.escape(sd.dir_name)}</b>, newest first. "
        f"On this computer: <code>{html.escape(str(sd.photo_dir))}</code></p>"
        f"<ul>{rows}</ul>"
        "<p>API: <a href='/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0'>getdirectories</a> | "
        f"<a href='/cgi-bin/media.cgi/api/v1/getmedialist?media_type=0&path={quote(sd.dir_name)}&start=0&count=9999'>"
        "getmedialist</a></p></body></html>"
    )


# ---------------------------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------------------------
class MediaServer:
    """Threaded HTTP server for the emulated SD card.

    Args:
        sd_card:   the :class:`SdCard` to serve.
        bind_ip:   address to listen on (e.g. 192.168.144.25 or 127.0.0.1).
        port:      TCP port (82 like the real camera; 0 = any free port, for tests).
        url_host:  host[:port] to put in photo URLs; '' = use the request's Host header.
        logger:    a ``logging.Logger``.
    """

    def __init__(self, sd_card: SdCard, bind_ip: str = "127.0.0.1", port: int = 82, url_host: str = "",
                 logger: Optional[logging.Logger] = None) -> None:
        self.sd = sd_card
        self.bind_ip = bind_ip
        self.requested_port = int(port)
        self.url_host = (url_host or "").strip()
        self.log = logger or logging.getLogger("siyi_emulator.media")
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self.requests_served = 0

    @property
    def port(self) -> int:
        """The TCP port actually in use (useful when 0 was requested)."""
        return self._server.server_address[1] if self._server else self.requested_port

    def start(self) -> None:
        """Bind and start serving in a background thread. Raises OSError if the port is unavailable."""
        handler = _make_handler(self)
        server = ThreadingHTTPServer((self.bind_ip, self.requested_port), handler)
        server.daemon_threads = True
        self._server = server
        self._thread = threading.Thread(target=server.serve_forever, name="siyi-media-http", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop serving and close the socket (safe to call twice)."""
        server, self._server = self._server, None
        if server is not None:
            server.shutdown()
            server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def base_url(self, host_header: Optional[str]) -> str:
        """'http://host[:port]' for photo URLs (url_host param > Host header > bind address)."""
        host = self.url_host or (host_header or "").strip() or f"{self.bind_ip}:{self.port}"
        return f"http://{host}"


def _make_handler(media: MediaServer):
    """Build the request-handler class bound to one MediaServer (http.server needs a class)."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"          # keep-alive, like video_cam's requests.Session wants
        server_version = "SIYI-camera-emulator/1.0"
        timeout = 120                          # s; an idle keep-alive connection is closed after this

        def log_message(self, fmt: str, *args: Any) -> None:  # route http.server logs to our logger
            media.log.debug("HTTP %s - %s", self.address_string(), fmt % args)

        def do_GET(self) -> None:  # noqa: N802 (http.server naming)
            self._handle(send_body=True)

        def do_HEAD(self) -> None:  # noqa: N802
            self._handle(send_body=False)

        # -------------------------------------------------------------- helpers
        def _send(self, status: int, body: bytes, content_type: str, send_body: bool) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            if send_body:
                self.wfile.write(body)

        def _send_json(self, payload: Dict[str, Any], send_body: bool) -> None:
            body = json.dumps(payload).encode("utf-8")
            self._send(200, body, "application/json", send_body)

        def _handle(self, send_body: bool) -> None:
            try:
                media.requests_served += 1
                parts = urlsplit(self.path)
                path = re.sub(r"/{2,}", "/", unquote(parts.path))
                query = {k: v[-1] for k, v in parse_qs(parts.query, keep_blank_values=True).items()}
                if path.startswith(API_PREFIX):
                    self._handle_api(path[len(API_PREFIX):].strip("/"), query, send_body)
                elif path in ("/", "/index.html"):
                    self._send(200, index_page(media.sd).encode("utf-8"), "text/html; charset=utf-8", send_body)
                else:
                    self._handle_file(path, send_body)
            except (BrokenPipeError, ConnectionResetError):
                pass                                  # the client went away; nothing to do
            except Exception as exc:                  # never let one request kill the server
                media.log.warning("media server error on %s: %s", self.path, exc)
                try:
                    self._send(500, b"internal error", "text/plain", send_body)
                except Exception:
                    pass

        def _handle_api(self, name: str, query: Dict[str, str], send_body: bool) -> None:
            media_type = parse_media_type(query.get("media_type"))
            if name == "getdirectories":
                payload = directories_response(media.sd, media_type)
            elif name == "getmedialist":
                payload = media_list_response(
                    media.sd, media_type, query.get("path", ""),
                    _parse_int(query.get("start"), 0), _parse_int(query.get("count"), DEFAULT_LIST_COUNT),
                    media.base_url(self.headers.get("Host")))
            elif name == "getmediacount":
                payload = media_count_response(media.sd, media_type, query.get("path", ""))
            else:
                payload = api_response({}, success=False, code=404, message=f"unknown api '{name}'")
            media.log.debug("HTTP API %s %s -> success=%s", name, query, payload["success"])
            self._send_json(payload, send_body)

        def _handle_file(self, path: str, send_body: bool) -> None:
            target = media.sd.resolve(path)
            if target is None:
                self._send(404, b"not found", "text/plain", send_body)
                return
            try:
                data = target.read_bytes()
            except OSError:
                self._send(404, b"not found", "text/plain", send_body)
                return
            ctype = "image/jpeg" if target.suffix.lower() in (".jpg", ".jpeg") else (
                "application/json" if target.suffix.lower() == ".json" else "application/octet-stream")
            media.log.debug("HTTP download %s (%d bytes)", target.name, len(data))
            self._send(200, data, ctype, send_body)

    return Handler
