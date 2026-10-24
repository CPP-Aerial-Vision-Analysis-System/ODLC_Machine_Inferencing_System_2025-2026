"""
lattice_bridge/fake_lattice.py -- PURPOSE: a small pretend Lattice that runs on this computer, so the whole bridge can
be tried (and shown to a teammate) with no Lattice account, no internet and no token. It speaks the same REST door
as the real thing:

    PUT /api/v1/entities          publish one entity. Answers 200 with {} when taken (also when it is silently
                                  discarded because an equal or older sourceUpdateTime is already held, exactly
                                  as Lattice does), 401 for a missing or wrong token, 400 for an entity that
                                  breaks the contract (the answer says what is wrong, in plain words).
    GET  /api/v1/entities/<id>    one entity of the current picture, or 404.
    GET  /entities.json           the whole current picture (what the map would show now).
    GET  /stats.json              counters: requests, accepted, discarded, refused, ...
    GET  /health                  "ok" (for scripts that wait until it is up).
    GET  /                        a LIVE MAP PAGE: the drone, the targets, the drop points and the areas drawn
                                  to scale, a table with the age of each entity, the counters. It reloads
                                  itself every 2 seconds. Open it in a browser while a flight runs.
    POST /control                 make the server misbehave on purpose, to test the bridge's recovery:
                                  {"down": true}                   answer 503 to everything
                                  {"delay_s": 5}                   wait 5 s before every answer
                                  {"fail_status": 500, "fail_count": 3}   the next 3 requests fail with 500
                                  {"down": false, "delay_s": 0}    back to normal

What it keeps is Lattice's own bookkeeping: an update only counts when its provenance.sourceUpdateTime is LATER than
the one held; `isLive: false` takes the entity off the picture (it stays in `history`); an entity is checked against
the same contract the bridge builds to (entities.find_problems), so a wrongly built entity is refused here first.

Tokens: with `token` set, `authorization: Bearer <token>` must match (and `anduril-sandbox-authorization` when a
`sandbox_token` is set). With no token the server is in OPEN mode: any non-empty bearer token is accepted (good for a
quick demo); the tests always set one. It listens on 127.0.0.1 by default so nobody else on the network can reach it.

Run it:   python3 -m lattice_bridge.fake_lattice --port 8089        (Ctrl-C stops it)
In code:  fake = FakeLattice(token='...').start();  fake.url;  fake.entities();  fake.stop()

Python standard library only (http.server, threading, json, html).
"""

from __future__ import annotations

import argparse
import html
import json
import math
import os
import signal
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import constants as C
from .entities import find_problems
from .sinks.rest_sink import ENTITIES_PATH, SANDBOX_HEADER
from .timeutil import parse_utc

MAX_BODY_BYTES = 1024 * 1024
PAGE_REFRESH_S = 2


def _utc_now() -> datetime:
    """The real clock, as an aware UTC datetime (tests pass their own fake clock instead)."""
    return datetime.now(timezone.utc)


def _positions(entity: Dict[str, Any]) -> Tuple[Optional[Tuple[float, float]], List[List[Tuple[float, float]]]]:
    """(point, polygons) of an entity: its location or point shape, and the rings of a polygon shape (lat, lon)."""
    def pair(node) -> Optional[Tuple[float, float]]:
        """(lat, lon) of a {'position': {...}} block, or None when it is missing or not numbers."""
        position = (node or {}).get('position') if isinstance(node, dict) else None
        if isinstance(position, dict):
            lat, lon = position.get('latitudeDegrees'), position.get('longitudeDegrees')
            if isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
                return float(lat), float(lon)
        return None

    point = pair(entity.get('location'))
    rings: List[List[Tuple[float, float]]] = []
    shape = entity.get('geoShape')
    if isinstance(shape, dict):
        if point is None:
            point = pair(shape.get('point'))
        polygon = shape.get('polygon')
        if isinstance(polygon, dict):
            for ring in polygon.get('rings') or []:
                corners = [pair(p) for p in (ring or {}).get('positions') or []]
                rings.append([c for c in corners if c is not None])
    return point, rings


class FakeLattice:
    """The pretend Lattice (see the module text). Thread safe; `start()` returns self."""

    def __init__(self, host: str = '127.0.0.1', port: int = 0, token: str = '', sandbox_token: str = '', *,
                 clock: Callable[[], datetime] = _utc_now):
        self.host = host
        self.port = port
        self.token = token
        self.sandbox_token = sandbox_token
        self._clock = clock
        self._lock = threading.Lock()
        self._picture: Dict[str, Dict[str, Any]] = {}
        self._received_at: Dict[str, float] = {}
        self._held_time: Dict[str, datetime] = {}
        self.history: List[Dict[str, Any]] = []
        self.counters: Dict[str, int] = {key: 0 for key in (
            'requests', 'accepted', 'discarded_older', 'went_offline', 'refused_auth', 'refused_contract',
            'refused_malformed', 'injected_failures', 'gets')}
        self._down = False
        self._delay_s = 0.0
        self._fail_status: Optional[int] = None
        self._fail_left = 0
        self._retry_after: Optional[float] = None
        self._server: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------------------------------ life cycle
    def start(self) -> 'FakeLattice':
        """Start listening in a background thread (port 0 picks a free port; see `url`)."""
        if self._server is not None:
            return self
        owner = self

        class Handler(_Handler):
            """The request handler of THIS server (it knows which FakeLattice it belongs to)."""

            fake = owner

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._server.daemon_threads = True
        self.port = self._server.server_address[1]
        self._thread = threading.Thread(target=self._server.serve_forever, kwargs={'poll_interval': 0.1},
                                        name='fake-lattice', daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        """Stop listening. Safe to call twice."""
        server, self._server = self._server, None
        if server is not None:
            server.shutdown()
            server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5)
            self._thread = None

    @property
    def url(self) -> str:
        """The address to give the bridge as LATTICE_ENDPOINT."""
        shown = '127.0.0.1' if self.host in ('', '0.0.0.0') else self.host
        return f'http://{shown}:{self.port}'

    def __enter__(self) -> 'FakeLattice':
        """`with FakeLattice() as fake:` starts it."""
        return self.start()

    def __exit__(self, *exc_info) -> None:
        """...and stops it again at the end of the block."""
        self.stop()

    # ------------------------------------------------------------------------------------------ what it knows
    def entities(self) -> Dict[str, Dict[str, Any]]:
        """The current picture: entityId -> entity (a copy)."""
        with self._lock:
            return json.loads(json.dumps(self._picture))

    def stats(self) -> Dict[str, Any]:
        """Counters plus the number of entities on the picture and in the history."""
        with self._lock:
            return {**self.counters, 'on_picture': len(self._picture), 'history': len(self.history)}

    # ------------------------------------------------------------------------------------------ misbehaving
    def configure(self, *, down: Optional[bool] = None, delay_s: Optional[float] = None,
                  fail_status: Optional[int] = None, fail_count: int = 1, retry_after_s: Optional[float] = None) -> None:
        """Change how the server behaves (see POST /control in the module text)."""
        with self._lock:
            if down is not None:
                self._down = bool(down)
            if delay_s is not None:
                self._delay_s = max(0.0, min(float(delay_s), 60.0))
            if fail_status is not None:
                self._fail_status = int(fail_status)
                self._fail_left = max(0, int(fail_count))
                self._retry_after = retry_after_s

    def _injected_failure(self) -> Tuple[float, Optional[int], Optional[float]]:
        """(delay to apply, status to answer instead of working, Retry-After) for the next request."""
        with self._lock:
            delay = self._delay_s
            if self._down:
                self.counters['injected_failures'] += 1
                return delay, 503, self._retry_after
            if self._fail_left > 0 and self._fail_status is not None:
                self._fail_left -= 1
                self.counters['injected_failures'] += 1
                return delay, self._fail_status, self._retry_after
            return delay, None, None

    # ------------------------------------------------------------------------------------------ publishing
    def check_auth(self, headers) -> bool:
        """True when the request carries the right token(s)."""
        sent = (headers.get('authorization') or '').strip()
        if not sent.lower().startswith('bearer ') or not sent[7:].strip():
            return False
        if self.token and sent != 'Bearer ' + self.token:
            return False
        if self.sandbox_token and (headers.get(SANDBOX_HEADER) or '').strip() != 'Bearer ' + self.sandbox_token:
            return False
        return True

    def publish(self, entity: Any) -> Tuple[int, Dict[str, Any]]:
        """Apply one published entity; returns (HTTP status, JSON answer)."""
        if not isinstance(entity, dict):
            with self._lock:
                self.counters['refused_malformed'] += 1
            return 400, {'message': 'the body must be one JSON object (an entity)'}
        now = self._clock()
        problems = find_problems(entity, now)
        if problems:
            with self._lock:
                self.counters['refused_contract'] += 1
            return 400, {'message': 'the entity breaks the contract: ' + '; '.join(problems[:5])}
        entity_id = entity['entityId']
        held_since = parse_utc(entity['provenance']['sourceUpdateTime'])
        with self._lock:
            held = self._held_time.get(entity_id)
            if held is not None and held_since <= held:
                self.counters['discarded_older'] += 1                    # Lattice: "will discard the update"
                return 200, {}
            self._held_time[entity_id] = held_since
            self.history.append(json.loads(json.dumps(entity)))
            if entity.get('isLive') is False:
                self._picture.pop(entity_id, None)
                self._received_at.pop(entity_id, None)
                self.counters['went_offline'] += 1
            else:
                self._picture[entity_id] = json.loads(json.dumps(entity))
                self._received_at[entity_id] = time.monotonic()
            self.counters['accepted'] += 1
        return 200, {}

    # ------------------------------------------------------------------------------------------ the map page
    def render_page(self) -> str:
        """The live map page (one self-contained HTML document)."""
        with self._lock:
            picture = list(self._picture.values())
            ages = {key: time.monotonic() - value for key, value in self._received_at.items()}
            counters = dict(self.counters)
        rows, shapes, lats, lons = [], [], [], []
        for entity in picture:
            point, rings = _positions(entity)
            for lat, lon in [p for ring in rings for p in ring] + ([point] if point else []):
                lats.append(lat)
                lons.append(lon)
        lat0 = sum(lats) / len(lats) if lats else 0.0
        east_scale = math.cos(math.radians(lat0))
        width, height, pad = 900.0, 560.0, 40.0
        east = [lon * east_scale for lon in lons] or [0.0]
        span_x = max(max(east) - min(east), 1e-9)
        span_y = max(max(lats or [0.0]) - min(lats or [0.0]), 1e-9)
        scale = min((width - 2 * pad) / span_x, (height - 2 * pad) / span_y)

        def xy(lat: float, lon: float) -> Tuple[float, float]:
            """Map (lat, lon) to a drawing position in pixels (east to the right, north up)."""
            return (pad + (lon * east_scale - min(east)) * scale, height - pad - (lat - min(lats or [0.0])) * scale)

        for entity in sorted(picture, key=lambda e: e['entityId']):
            ontology = entity.get('ontology') or {}
            template, kind = ontology.get('template', ''), ontology.get('platformType', '')
            name = html.escape((entity.get('aliases') or {}).get('name', entity['entityId']))
            point, rings = _positions(entity)
            for ring in rings:
                if len(ring) >= 3:
                    cls = 'control' if (entity.get('geoDetails') or {}).get('type') == 'GEO_TYPE_CONTROL_AREA' else 'area'
                    pts = ' '.join(f'{x:.1f},{y:.1f}' for x, y in (xy(la, lo) for la, lo in ring))
                    shapes.append(f'<polygon class="{cls}" points="{pts}"><title>{name}</title></polygon>')
            if point is not None:
                x, y = xy(*point)
                if template == 'TEMPLATE_ASSET':
                    shapes.append(f'<polygon class="uav" points="{x:.1f},{y - 11:.1f} {x - 9:.1f},{y + 8:.1f} {x + 9:.1f},{y + 8:.1f}"/>'
                                  f'<text x="{x + 12:.1f}" y="{y + 4:.1f}">{name}</text>')
                elif template == 'TEMPLATE_TRACK':
                    cls = 'tent' if kind == C.PLATFORM_TYPE_TENT else 'person'
                    shapes.append(f'<circle class="{cls}" cx="{x:.1f}" cy="{y:.1f}" r="8"/>'
                                  f'<text x="{x + 12:.1f}" y="{y + 4:.1f}">{name}</text>')
                else:
                    shapes.append(f'<rect class="drop" x="{x - 6:.1f}" y="{y - 6:.1f}" width="12" height="12" '
                                  f'transform="rotate(45 {x:.1f} {y:.1f})"/><text x="{x + 12:.1f}" y="{y + 4:.1f}">{name}</text>')
            shown = f'{point[0]:.6f}, {point[1]:.6f}' if point else '(area)'
            rows.append(f'<tr><td>{name}</td><td>{html.escape(template.replace("TEMPLATE_", "").title())}'
                        f'{" / " + html.escape(kind) if kind else ""}</td><td>{shown}</td>'
                        f'<td>{ages.get(entity["entityId"], 0.0):.0f} s ago</td></tr>')
        count_text = ' &middot; '.join(f'{k.replace("_", " ")}: {v}' for k, v in counters.items() if v)
        empty = '<p class="empty">Nothing on the picture yet. Start the bridge and fly.</p>' if not picture else ''
        return _PAGE.format(refresh=PAGE_REFRESH_S, shapes=''.join(shapes), rows=''.join(rows), empty=empty,
                            counters=count_text or 'no requests yet', width=int(width), height=int(height),
                            count=len(picture))


_PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="{refresh}"><title>Fake Lattice</title><style>
:root{{--bg:#f6f7f9;--fg:#1b2430;--mu:#5d6b7c;--card:#fff;--ln:#d5dbe3;--acc:#1f6feb;--ok:#1a7f37;--warn:#b35900;--bad:#c62828}}
@media (prefers-color-scheme:dark){{:root{{--bg:#11161d;--fg:#e6edf3;--mu:#9aa7b5;--card:#19212b;--ln:#2d3946;--acc:#58a6ff;--ok:#3fb950;--warn:#e3b341;--bad:#ff7b72}}}}
body{{margin:0;padding:16px;background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,sans-serif}}
h1{{font-size:1.2rem;margin:0 0 4px}}p.sub{{color:var(--mu);margin:0 0 12px}}
svg{{width:100%;max-width:900px;height:auto;background:var(--card);border:1px solid var(--ln);border-radius:8px}}
text{{fill:var(--fg);font-size:12px}}.area{{fill:rgba(31,111,235,.12);stroke:var(--acc);stroke-width:1.5}}
.control{{fill:none;stroke:var(--warn);stroke-width:2;stroke-dasharray:8 5}}.uav{{fill:var(--acc)}}
.person{{fill:var(--ok)}}.tent{{fill:var(--warn)}}.drop{{fill:var(--bad)}}
table{{border-collapse:collapse;width:100%;max-width:900px;margin-top:12px;background:var(--card)}}
td,th{{border:1px solid var(--ln);padding:4px 8px;text-align:left;font-size:13px}}.empty{{color:var(--mu)}}
</style></head><body><h1>Fake Lattice <small style="color:var(--mu)">(a pretend one, on this computer)</small></h1>
<p class="sub">{count} entities on the picture &middot; reloads every {refresh} s &middot; {counters}</p>
<svg viewBox="0 0 {width} {height}" role="img" aria-label="Map of the entities the bridge has published">{shapes}</svg>{empty}
<table><tr><th>Name</th><th>What</th><th>Where</th><th>Last update</th></tr>{rows}</table></body></html>"""


class _Handler(BaseHTTPRequestHandler):
    """One request. `fake` is filled in by FakeLattice.start()."""

    fake: FakeLattice
    protocol_version = 'HTTP/1.1'
    server_version = 'FakeLattice'

    def log_message(self, fmt, *args) -> None:
        """Keep the console quiet: one line per request would drown the bridge's own log."""
        return None

    def _send(self, status: int, body: bytes, content_type: str = 'application/json',
              extra: Optional[Dict[str, str]] = None) -> None:
        """Write one complete answer (status, headers, body)."""
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Connection', 'close')
        for key, value in (extra or {}).items():
            self.send_header(key, value)
        self.end_headers()
        if self.command != 'HEAD':
            self.wfile.write(body)

    def _json(self, status: int, payload: Any, extra: Optional[Dict[str, str]] = None) -> None:
        """Answer with a JSON body."""
        self._send(status, json.dumps(payload).encode('utf-8'), extra=extra)

    def _read_body(self) -> Optional[bytes]:
        """The request body, or None after answering 400/413 when it is missing or larger than MAX_BODY_BYTES."""
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            length = -1
        if length < 0 or length > MAX_BODY_BYTES:
            self._json(413 if length > 0 else 400, {'message': 'the body is missing or too large'})
            return None
        return self.rfile.read(length) if length else b''

    def do_PUT(self) -> None:                                      # noqa: N802 (http.server's naming)
        """PUT /api/v1/entities: count it, apply any injected failure, check the token, then publish."""
        fake = self.fake
        with fake._lock:
            fake.counters['requests'] += 1
        body = self._read_body()                                   # always read it first: answering with the body unread can reset the connection
        if body is None:
            return
        if self.path.split('?')[0] != ENTITIES_PATH:
            self._json(404, {'message': 'not found'})
            return
        delay, status, retry_after = fake._injected_failure()
        if delay:
            time.sleep(delay)
        if status is not None:
            extra = {'Retry-After': f'{retry_after:g}'} if retry_after else None
            self._json(status, {'message': 'injected failure'}, extra)
            return
        if not fake.check_auth(self.headers):
            with fake._lock:
                fake.counters['refused_auth'] += 1
            self._json(401, {'message': 'missing or wrong token'})
            return
        try:
            entity = json.loads(body.decode('utf-8'))
        except (UnicodeDecodeError, ValueError):
            with fake._lock:
                fake.counters['refused_malformed'] += 1
            self._json(400, {'message': 'the body is not valid JSON'})
            return
        code, payload = fake.publish(entity)
        self._json(code, payload)

    def do_GET(self) -> None:                                      # noqa: N802
        """The read-only pages: the map, the picture as JSON, one entity, the counters, /health."""
        fake = self.fake
        with fake._lock:
            fake.counters['gets'] += 1
        path = self.path.split('?')[0]
        if path == '/health':
            self._send(200, b'ok', 'text/plain')
        elif path in ('/', '/map'):
            self._send(200, fake.render_page().encode('utf-8'), 'text/html; charset=utf-8')
        elif path == '/entities.json':
            self._json(200, list(fake.entities().values()))
        elif path == '/stats.json':
            self._json(200, fake.stats())
        elif path.startswith(ENTITIES_PATH + '/'):
            found = fake.entities().get(path[len(ENTITIES_PATH) + 1:])
            self._json(200, found) if found else self._json(404, {'message': 'no such entity'})
        else:
            self._json(404, {'message': 'not found'})

    def do_POST(self) -> None:                                     # noqa: N802
        """POST /control: make the server misbehave on purpose (down, slow, a few failures)."""
        body = self._read_body()
        if body is None:
            return
        if self.path.split('?')[0] != '/control':
            self._json(404, {'message': 'not found'})
            return
        try:
            wanted = json.loads(body.decode('utf-8') or '{}')
            self.fake.configure(down=wanted.get('down'), delay_s=wanted.get('delay_s'),
                                fail_status=wanted.get('fail_status'), fail_count=wanted.get('fail_count', 1),
                                retry_after_s=wanted.get('retry_after_s'))
        except (UnicodeDecodeError, ValueError, TypeError, AttributeError):
            self._json(400, {'message': 'send a JSON object like {"down": true}'})
            return
        self._json(200, {'ok': True})


def main(argv: Optional[List[str]] = None) -> int:
    """Command line: run the fake Lattice until Ctrl-C."""
    parser = argparse.ArgumentParser(prog='fake_lattice', description='A pretend Lattice on this computer (see the file text).')
    parser.add_argument('--host', default='127.0.0.1', help='address to listen on (default: this computer only)')
    parser.add_argument('--port', type=int, default=8089, help='port to listen on (default 8089)')
    parser.add_argument('--token-env', default='LATTICE_TOKEN',
                        help='name of the environment variable that holds the token to require (default LATTICE_TOKEN); '
                             'when it is empty the server is in open mode')
    parser.add_argument('--sandbox-token-env', default='SANDBOXES_TOKEN',
                        help='name of the environment variable that holds the sandbox token to require (default SANDBOXES_TOKEN)')
    args = parser.parse_args(argv)
    fake = FakeLattice(args.host, args.port, os.environ.get(args.token_env, ''), os.environ.get(args.sandbox_token_env, ''))
    fake.start()
    mode = 'checks the token from ' + args.token_env if fake.token else f'OPEN mode (any non-empty bearer token; {args.token_env} is not set)'
    print(f'fake Lattice listening on {fake.url} - {mode}', flush=True)
    print(f'live map: {fake.url}/   (give {fake.url} to the bridge as its endpoint)', flush=True)
    stop = threading.Event()
    for name in ('SIGINT', 'SIGTERM'):
        if hasattr(signal, name):
            signal.signal(getattr(signal, name), lambda *_: stop.set())
    try:
        while not stop.wait(0.5):
            pass
    finally:
        fake.stop()
    print('fake Lattice stopped', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
