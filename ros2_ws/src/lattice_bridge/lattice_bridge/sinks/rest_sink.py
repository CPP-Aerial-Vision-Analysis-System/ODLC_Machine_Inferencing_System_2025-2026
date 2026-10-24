"""
lattice_bridge/sinks/rest_sink.py -- PURPOSE: send entities to a Lattice over its REST door: one
`PUT <endpoint>/api/v1/entities` per entity, with the token in an `authorization: Bearer ...` header (and, for a
Lattice Developer Program sandbox, the second token in `anduril-sandbox-authorization: Bearer ...`). The same
class talks to the real Lattice and to our fake local Lattice (fake_lattice.py), because both speak the same REST.

In plain words, what it does with each answer:

    HTTP 200 / 201 / 202 / 204          delivered.
    HTTP 401 / 403                      the token is missing, wrong or expired: AuthSinkError (affects every request).
    HTTP 400 / 413 / 422 (and other 4xx) this one entity is refused: FatalSinkError (the Publisher drops that
                                        version and carries on with the others).
    HTTP 3xx                            the address is wrong (a redirect would also leak the token to another host,
                                        so redirects are never followed): FatalSinkError for every request.
    HTTP 408 / 409 / 425 / 429 / 404 / 5xx, no answer, timeout, connection refused or dropped:
                                        RetryableSinkError (the Publisher backs off and tries again). If the server
                                        sent `Retry-After: N` seconds, that is passed on.
    a certificate that does not verify  FatalSinkError for every request (retrying cannot fix a certificate).

Rules this file keeps:
  * NO retries here. The Publisher owns the retry policy; a sink that also retried would multiply the attempts.
  * NEVER a secret in a message. Error text is cut to one short line and every token is blanked out of it, even
    if the server echoes it back.
  * Redirects are never followed and every request opens (and closes) its own connection: simple, and a half-dead
    connection can never poison the next request. At 1 update per second that costs nothing.
  * A plain `http://` address is accepted only for this computer or a private network (the fake Lattice, a
    lab). A remote host must use `https://`, because the token would otherwise cross the internet in clear text.
  * The certificate is always verified (the system's trusted list). There is no "ignore certificate errors" switch.

Python standard library only (http.client, ssl, json, ipaddress).
"""

from __future__ import annotations

import http.client
import ipaddress
import json
import math
import socket
import ssl
from datetime import datetime
from typing import Any, Dict, Optional
from urllib.parse import urlsplit

from .base import AuthSinkError, BaseSink, ConfigSinkError, FatalSinkError, RetryableSinkError, short_text

#: The one URL path entities are published to.
ENTITIES_PATH = '/api/v1/entities'
#: The header that carries the sandbox token (Lattice Developer Program sandboxes only).
SANDBOX_HEADER = 'anduril-sandbox-authorization'
#: Longest answer body that is read (a broken or hostile server must not make us read forever).
MAX_RESPONSE_BYTES = 64 * 1024
#: Longest Retry-After hint that is believed; the Publisher caps it again with its own limit.
MAX_RETRY_AFTER_S = 3600.0

_OK = frozenset({200, 201, 202, 204})
_RETRYABLE = frozenset({404, 408, 409, 425, 429})


def is_private_host(host: str) -> bool:
    """True for localhost, a loopback or private-network address (where plain http is acceptable)."""
    name = (host or '').strip().strip('[]').lower()
    if name in ('localhost', 'host.docker.internal') or name.endswith('.localhost'):
        return True
    try:
        address = ipaddress.ip_address(name)
    except ValueError:
        return False
    return address.is_loopback or address.is_private or address.is_link_local


def _header_safe(value: str) -> bool:
    """True when `value` can sit in an HTTP header (printable, one line)."""
    return bool(value) and all(32 <= ord(ch) < 127 for ch in value)


class RestSink(BaseSink):
    """Publish entities with `PUT /api/v1/entities` (see the module text for how answers are classified)."""

    name = 'rest'

    def __init__(self, endpoint: str, token: str, sandbox_token: str = '', *, timeout_s: float = 10.0):
        parts = urlsplit(endpoint or '')
        if parts.scheme not in ('http', 'https') or not parts.hostname:
            raise ConfigSinkError('the Lattice endpoint must look like https://host or http://host:port')
        if parts.scheme == 'http' and not is_private_host(parts.hostname):
            raise ConfigSinkError('a plain http:// address is only allowed for this computer or a private network; '
                                  'use https:// for a remote Lattice so the token is not sent in clear text')
        if not token:
            raise ConfigSinkError('no Lattice token is set (LATTICE_TOKEN)')
        if not _header_safe(token) or (sandbox_token and not _header_safe(sandbox_token)):
            raise ConfigSinkError('a Lattice token contains characters that cannot be sent in an HTTP header '
                                  '(a space-only value, a line break or a non-ASCII letter); check the secrets file')
        if not isinstance(timeout_s, (int, float)) or isinstance(timeout_s, bool) \
                or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ConfigSinkError('the request timeout must be a number of seconds greater than 0')
        try:
            self._port = parts.port
        except ValueError:
            raise ConfigSinkError('the Lattice endpoint has an invalid port number') from None
        self._scheme = parts.scheme
        self._host = parts.hostname
        self._timeout = float(timeout_s)
        self._secrets = [value for value in (token, sandbox_token) if value]
        self._headers = {'content-type': 'application/json', 'accept': 'application/json',
                         'authorization': 'Bearer ' + token}
        if sandbox_token:
            self._headers[SANDBOX_HEADER] = 'Bearer ' + sandbox_token
        self._context = ssl.create_default_context() if self._scheme == 'https' else None
        self._sandbox = bool(sandbox_token)
        shown_port = f':{self._port}' if self._port else ''
        self._label = f'{self._scheme}://{self._host}{shown_port}'

    # ------------------------------------------------------------------------------------------ the sink API
    def describe(self) -> str:
        """`rest https://host` (plus `sandbox` when a sandbox token is in use); never a token."""
        return f'rest {self._label}' + (' (sandbox)' if self._sandbox else '')

    def close(self) -> None:
        """Nothing stays open between requests, so there is nothing to release."""
        return None

    def publish_entity(self, entity: Dict[str, Any], t: Optional[datetime] = None) -> None:
        """PUT one entity. Returns None when delivered, raises RetryableSinkError / FatalSinkError otherwise."""
        try:
            body = json.dumps(entity, allow_nan=False, separators=(',', ':')).encode('utf-8')
        except (TypeError, ValueError) as exc:
            raise FatalSinkError(f'the entity cannot be written as JSON ({short_text(exc, 120)})') from None
        status, retry_after, text = self._request('PUT', ENTITIES_PATH, body)
        if status in _OK:
            return
        self._raise_for_status(status, retry_after, text)

    # ------------------------------------------------------------------------------------------ the details
    def _scrub(self, text: Any) -> str:
        """`text` as a short single line with every secret blanked out."""
        line = short_text(text)
        for secret in self._secrets:
            line = line.replace(secret, '***')
        return line

    def _connection(self) -> http.client.HTTPConnection:
        """A fresh connection (https verified against the system's trusted certificates)."""
        if self._scheme == 'https':
            return http.client.HTTPSConnection(self._host, self._port, timeout=self._timeout, context=self._context)
        return http.client.HTTPConnection(self._host, self._port, timeout=self._timeout)

    def _request(self, method: str, path: str, body: Optional[bytes]):
        """Do one HTTP request; returns (status, retry_after_s, short body text). Network trouble becomes a sink error."""
        connection = self._connection()
        try:
            connection.request(method, path, body=body, headers=self._headers)
            response = connection.getresponse()
            data = response.read(MAX_RESPONSE_BYTES)
            retry_after = self._parse_retry_after(response.getheader('Retry-After'))
            text = data.decode('utf-8', errors='replace')
            return response.status, retry_after, text
        except ssl.SSLCertVerificationError as exc:
            raise FatalSinkError(f'the certificate of {self._label} is not trusted ({self._scrub(exc)})',
                                 systemic=True) from None
        except ssl.SSLError as exc:
            raise RetryableSinkError(f'secure connection to {self._label} failed ({self._scrub(exc)})') from None
        except (socket.timeout, TimeoutError):
            raise RetryableSinkError(f'{self._label} did not answer within {self._timeout:g} s') from None
        except (OSError, http.client.HTTPException) as exc:
            raise RetryableSinkError(f'cannot reach {self._label} ({exc.__class__.__name__}: {self._scrub(exc)})') from None
        finally:
            try:
                connection.close()
            except Exception:                                   # closing must never hide the real answer
                pass

    @staticmethod
    def _parse_retry_after(value: Optional[str]) -> Optional[float]:
        """The `Retry-After` header as seconds, or None when absent, a date (not used) or nonsense."""
        if not value:
            return None
        try:
            seconds = float(value.strip())
        except ValueError:
            return None
        if not math.isfinite(seconds) or seconds <= 0:
            return None
        return min(seconds, MAX_RETRY_AFTER_S)

    def _raise_for_status(self, status: int, retry_after: Optional[float], text: str) -> None:
        """Turn a not-OK HTTP status into the right typed error (always raises)."""
        shown = self._scrub(text)
        suffix = f': {shown}' if shown else ''
        if status in (401, 403):
            raise AuthSinkError(f'HTTP {status}: the token was refused{suffix}', status=status)
        if 300 <= status < 400:
            raise FatalSinkError(f'HTTP {status}: {self._label} sent us somewhere else; check LATTICE_ENDPOINT '
                                 f'(redirects are never followed)', status=status, systemic=True)
        if status in _RETRYABLE or status >= 500:
            raise RetryableSinkError(f'HTTP {status}{suffix}', status=status, retry_after_s=retry_after)
        raise FatalSinkError(f'HTTP {status}: the entity was refused{suffix}', status=status)
