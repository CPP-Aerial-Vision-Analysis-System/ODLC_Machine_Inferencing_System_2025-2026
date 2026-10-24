"""
lattice_bridge/sinks/base.py -- PURPOSE: the contract every sink follows, in one place. A "sink" is where a
finished Lattice request goes: the real Lattice over REST (rest_sink.py), the fake local Lattice (also over REST),
an in-memory list for unit tests (mock_sink.py) or a recording file (jsonl_sink.py). The Publisher
(publisher.py) talks to all of them through BaseSink, so the code that decides WHAT to send never needs to
know WHERE it goes.

What a sink promises:
  * publish_entity(entity, t) returns None when the entity was delivered, and otherwise raises ONE of two typed
    errors, so the caller knows what to do without reading error text:
      RetryableSinkError  trouble that may go away by itself (link down, timeout, server 5xx, "slow down"):
                          try again later, with a growing pause.
      FatalSinkError      trying the SAME request again will not help (rejected content, bad credentials,
                          refused certificate). `systemic=True` says that EVERY request would fail the same way
                          (credentials, certificate, wrong address), so the caller should pause instead of
                          discarding the one request that happened to notice.
  * NO retries inside a sink. The Publisher owns the retry policy (one place to tune, one place to test), and a
    sink that also retried would multiply the attempts (the Lattice docs warn about exactly this).
  * Error text never contains a secret (token, header value). Sinks keep credentials private.
  * close() is safe to call twice.

BaseSink also answers the older, smaller `Sink` protocol of sinks/__init__.py (write(kind, payload, t) -> bool,
which never raises), so a BaseSink can be used wherever that protocol is expected.

Python standard library only (abc, dataclasses, logging): no ROS, no network.
"""

from __future__ import annotations

import abc
import logging
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, Optional

LOG = logging.getLogger('lattice_bridge.sink')

#: Longest piece of server text that is copied into an error message (a hostile or broken server must not
#: be able to flood a log).
MAX_ERROR_TEXT = 300


def short_text(text: Any, limit: int = MAX_ERROR_TEXT) -> str:
    """`text` as one printable line of at most `limit` characters (for error messages and logs)."""
    cleaned = ''.join(ch if ch.isprintable() else ' ' for ch in str(text)).strip()
    return cleaned if len(cleaned) <= limit else cleaned[:limit - 3] + '...'


class SinkError(Exception):
    """Base class of the errors a sink raises on purpose. `.status` is the HTTP status when there was one."""

    def __init__(self, message: str, *, status: Optional[int] = None):
        super().__init__(short_text(message))
        self.message = short_text(message)
        self.status = status


class RetryableSinkError(SinkError):
    """Trouble that may pass: connection refused or dropped, timeout, HTTP 5xx / 429 / 408 / 409 / 404.

    `retry_after_s` carries the server's own "come back in N seconds" hint when it sent one (the Retry-After
    header); the Publisher waits at least that long (up to its own cap).
    """

    def __init__(self, message: str, *, status: Optional[int] = None, retry_after_s: Optional[float] = None):
        super().__init__(message, status=status)
        self.retry_after_s = retry_after_s


class FatalSinkError(SinkError):
    """Sending the same request again will not help (the content or the credentials are refused).

    `systemic` is True when the cause is not this one request but the whole connection set-up (bad
    credentials, refused certificate, wrong address): then every other request would fail the same way, and
    the caller should pause and probe slowly instead of discarding requests one by one.
    """

    def __init__(self, message: str, *, status: Optional[int] = None, systemic: bool = False):
        super().__init__(message, status=status)
        self.systemic = bool(systemic)


class AuthSinkError(FatalSinkError):
    """HTTP 401 / 403: the token is missing, wrong or expired. Systemic: it hits every request."""

    def __init__(self, message: str, *, status: Optional[int] = None):
        super().__init__(message, status=status, systemic=True)


class ConfigSinkError(FatalSinkError):
    """The sink was set up wrongly (no endpoint, no token, a plain-http address for a remote host ...)."""

    def __init__(self, message: str):
        super().__init__(message, status=None, systemic=True)


@dataclass(frozen=True)
class ObjectInfo:
    """What a sink reports about an uploaded object (all text exactly as the server wrote it)."""
    path: str
    size_bytes: int
    checksum: Optional[str] = None
    last_updated_at: Optional[str] = None
    expiry_time: Optional[str] = None


class BaseSink(abc.ABC):
    """The abstract base every sink implements (see the module docstring for the promises)."""

    #: Short label for logs and the health report. Must never contain a secret.
    name = 'sink'

    @abc.abstractmethod
    def publish_entity(self, entity: Dict[str, Any], t: Optional[datetime] = None) -> None:
        """Deliver one entity (the body of one `PUT /api/v1/entities`).

        Returns None when it was delivered. Raises RetryableSinkError or FatalSinkError otherwise, and
        nothing else. `t` is the time of the request as an aware datetime (only recorders use it).
        """

    def upload_object(self, path: str, data: bytes, *, ttl_s: Optional[float] = None,
                      t: Optional[datetime] = None) -> ObjectInfo:
        """Store a file ("object") such as a target thumbnail. Sinks that cannot store files refuse it."""
        raise FatalSinkError(f'the {self.name} sink does not store objects')

    @abc.abstractmethod
    def close(self) -> None:
        """Flush and release everything. Safe to call more than once."""

    def describe(self) -> str:
        """A one-line, secret-free description for log lines (for example 'rest https://host')."""
        return self.name

    # ----------------------------------------------------------------------------- the older protocol
    def write(self, kind: str, payload: Any, t: Optional[datetime] = None) -> bool:
        """The phase-1 `Sink` protocol: True when delivered, False when not; never raises.

        Only `kind == 'entity'` is delivered by the typed sinks; any other kind is refused with False.
        """
        if kind != 'entity':
            return False
        try:
            self.publish_entity(payload, t)
            return True
        except SinkError as exc:
            LOG.warning('%s sink: entity not delivered: %s', self.name, exc)
        except Exception as exc:                                  # a sink bug must not break the promise
            LOG.warning('%s sink: unexpected %s: %s', self.name, type(exc).__name__, short_text(exc))
        return False

    def __enter__(self) -> 'BaseSink':
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()


class _WriteSinkAdapter(BaseSink):
    """Makes an object that only has write()/close() (the phase-1 protocol) usable as a BaseSink."""

    name = 'legacy'

    def __init__(self, inner: Any):
        self._inner = inner
        self.name = type(inner).__name__

    def publish_entity(self, entity: Dict[str, Any], t: Optional[datetime] = None) -> None:
        """Hand the entity to the inner `write`; a False answer becomes a FatalSinkError."""
        if not self._inner.write('entity', entity, t):
            raise FatalSinkError(f'the {self.name} sink refused the entity (see its own log)')

    def close(self) -> None:
        """Close the inner sink."""
        self._inner.close()


def adapt_sink(sink: Any) -> BaseSink:
    """Return `sink` itself when it is a BaseSink, wrap a write()/close() object, or raise TypeError."""
    if isinstance(sink, BaseSink):
        return sink
    if callable(getattr(sink, 'write', None)) and callable(getattr(sink, 'close', None)):
        return _WriteSinkAdapter(sink)
    raise TypeError(f'{type(sink).__name__} is not a sink: it needs publish_entity() or write()/close()')
