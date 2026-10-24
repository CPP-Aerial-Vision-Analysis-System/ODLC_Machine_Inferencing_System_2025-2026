"""
lattice_bridge/test/support_net.py -- PURPOSE: helpers shared by the sink, fake-Lattice and publisher tests:
a fake clock (so back-off and rate-limit tests need no real waiting), factories that make VALID entities with the
real builders, a tiny HTTP client for talking to the fake Lattice, and a polling helper for the few tests that
use real threads. Nothing here is part of the library. Test credentials are assembled at run time so no file
contains something that looks like a real token.
"""

import http.client
import json
import time
from dataclasses import dataclass
from datetime import timedelta
from typing import Any, Dict, Optional
from urllib.parse import urlsplit

from lattice_bridge.entities import build_area_geo, build_drop_point, build_target_track, build_uav_asset
from support import MISSION, NOW

UAV_ID = f'astra-{MISSION}-uav'

#: Made-up credentials for the fake Lattice. They are built from parts on purpose (see the module docstring).
GOOD_TOKEN = 'tok-' + 'a1b2c3d4' * 3
OTHER_TOKEN = 'tok-' + 'z9y8x7w6' * 3
SANDBOX_TOKEN = 'sbx-' + 'q1w2e3r4' * 3
AUTH_HEADERS = {'authorization': 'Bearer ' + GOOD_TOKEN}


class FakeClock:
    """Monotonic seconds and an aware UTC wall clock that move together, only when the test says so."""

    def __init__(self, wall=NOW, mono: float = 1000.0):
        self._wall0 = wall
        self._mono0 = mono
        self.t = mono
        self.sleeps = []

    def monotonic(self) -> float:
        """The monotonic reading (use as the Publisher's `clock`)."""
        return self.t

    def now(self):
        """The wall clock (use as the Publisher's `wall`)."""
        return self._wall0 + timedelta(seconds=self.t - self._mono0)

    def advance(self, seconds: float) -> None:
        """Let `seconds` pass."""
        self.t += seconds

    def sleep(self, seconds: float) -> None:
        """A sleep that only moves the fake time (use as the Publisher's `sleep`); remembers the request."""
        self.sleeps.append(seconds)
        self.advance(seconds)


def uav_at(clock: FakeClock, *, entity_id: str = UAV_ID, lat: float = 36.2143, lon: float = -96.0068,
           age_s: float = 0.0, ttl_s: Optional[float] = None, **kwargs) -> Dict[str, Any]:
    """A valid drone entity published 'now' on the fake clock (expires 120 s later, or `ttl_s` later when given).

    The GPS fix it describes was taken `age_s` seconds earlier (so sourceUpdateTime = now - age_s). Tests that keep
    the entity waiting longer than 120 s (a long outage) pass a big `ttl_s`, otherwise the publisher rightly drops
    it as expired.
    """
    when = clock.now()
    if ttl_s is not None and 'expiry_time' not in kwargs:
        kwargs['expiry_time'] = when + timedelta(seconds=ttl_s)
    return build_uav_asset(entity_id, lat, lon, fix_time=when - timedelta(seconds=age_s), now=when, **kwargs)


def target_at(clock: FakeClock, kind: str = 'person', number: int = 1, *, confidence: float = 0.9,
              lat: float = 36.2150, lon: float = -96.0062, expiry_hours: float = 8.0,
              **kwargs) -> Dict[str, Any]:
    """A valid target track measured 'now' on the fake clock (expires `expiry_hours` later)."""
    when = clock.now()
    return build_target_track(f'astra-{MISSION}-tgt-{kind}-{number}', kind, lat, lon, confidence,
                              measured_time=when, now=when, expiry_time=when + timedelta(hours=expiry_hours),
                              **kwargs)


def area_at(clock: FakeClock, name: str = 'search-box') -> Dict[str, Any]:
    """A valid search-area polygon defined 'now'."""
    when = clock.now()
    corners = [(36.2140, -96.0070), (36.2140, -96.0060), (36.2150, -96.0060), (36.2150, -96.0070)]
    return build_area_geo(f'astra-{MISSION}-area-{name}', name, corners, source_update_time=when, now=when,
                          expiry_time=when + timedelta(hours=12))


def drop_at(clock: FakeClock, kind: str = 'person', number: int = 1) -> Dict[str, Any]:
    """A valid drop point released 'now'."""
    when = clock.now()
    return build_drop_point(f'astra-{MISSION}-drop-{kind}-{number}', kind, 36.2145, -96.0065,
                            release_time=when, now=when, expiry_time=when + timedelta(hours=12))


def make_publisher(sink=None, clock=None, *, rng: float = 0.5, goodbye_default: bool = False, **config):
    """A Publisher wired to a MockSink and a FakeClock (nothing real: no thread, no sleeping, no jitter).

    `rng` is the constant "random" draw (0.5 = no jitter). Extra keyword arguments become PublisherConfig
    values. Returns (publisher, sink, clock).
    """
    from lattice_bridge.publisher import Publisher, PublisherConfig
    from lattice_bridge.sinks.mock_sink import MockSink
    clock = clock or FakeClock()
    sink = sink if sink is not None else MockSink()
    publisher = Publisher(sink, PublisherConfig(**config), clock=clock.monotonic, wall=clock.now,
                          sleep=clock.sleep, rng=lambda: rng, goodbye_default=goodbye_default)
    return publisher, sink, clock


def assert_accounting(health: Dict[str, Any]) -> None:
    """The Publisher's bookkeeping identity (module docstring of publisher.py): nothing is ever lost uncounted."""
    h = health
    assert h['submitted'] + h['goodbyes_queued'] == h['enqueued'] + h['unchanged'] + h['stale'], h
    assert h['enqueued'] == (h['sent'] + h['coalesced'] + h['dropped_overflow'] + h['dropped_expired']
                             + h['dropped_fatal'] + h['dropped_shutdown'] + h['queue_depth'] + h['inflight']), h
    assert h['dropped'] == (h['dropped_overflow'] + h['dropped_expired'] + h['dropped_fatal']
                            + h['dropped_shutdown']), h


def run_for(publisher, clock: FakeClock, seconds: float, step: float = 0.5) -> int:
    """Pump the publisher every `step` fake seconds for `seconds`; returns how many sink calls it made."""
    end = clock.t + seconds
    calls = 0
    while clock.t < end:
        calls += publisher.pump()
        clock.advance(step)
    return calls


def wait_until(predicate, timeout: float = 5.0, interval: float = 0.01) -> bool:
    """Poll `predicate()` until it is true or `timeout` real seconds pass; returns the last answer."""
    deadline = time.monotonic() + timeout
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return bool(predicate())
        time.sleep(interval)


@dataclass
class HttpResult:
    """What a request to the fake Lattice came back with."""
    status: int
    headers: Dict[str, str]
    body: bytes

    @property
    def json(self) -> Any:
        """The body parsed as JSON (fails the test with a clear error if it is not)."""
        return json.loads(self.body.decode('utf-8'))


def http_request(base_url: str, method: str, path: str, body: Optional[bytes] = None,
                 headers: Optional[Dict[str, str]] = None, timeout: float = 5.0) -> HttpResult:
    """One HTTP request with http.client (exact control over headers; no redirects, no retries)."""
    parts = urlsplit(base_url)
    connection = http.client.HTTPConnection(parts.hostname, parts.port, timeout=timeout)
    try:
        connection.request(method, path, body=body, headers=headers or {})
        response = connection.getresponse()
        data = response.read()
        return HttpResult(response.status, {k.lower(): v for k, v in response.getheaders()}, data)
    finally:
        connection.close()


def put_entity(base_url: str, entity: Any, *, headers: Optional[Dict[str, str]] = None,
               timeout: float = 5.0) -> HttpResult:
    """`PUT /api/v1/entities` with the standard test credentials and a JSON body."""
    merged = {'content-type': 'application/json', **AUTH_HEADERS, **(headers or {})}
    return http_request(base_url, 'PUT', '/api/v1/entities', json.dumps(entity).encode('utf-8'), merged, timeout)
