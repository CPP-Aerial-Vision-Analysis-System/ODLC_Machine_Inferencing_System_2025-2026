"""
lattice_bridge/sinks/__init__.py -- PURPOSE: the "sinks" of the bridge. A sink is where finished requests go:
a file (jsonl_sink, this phase), a fake local Lattice (phase 2), the real Lattice (phase 3). They all offer
the same two calls, written down here as the `Sink` protocol, so the code that decides WHAT to publish never
needs to know WHERE it goes:

    sink.write(kind, payload, t=None) -> bool     hand over one request; True = accepted, False = it
                                                  was not stored/sent (the reason is logged); never raises
                                                  for I/O or network trouble
    sink.close() -> None                          flush and release everything; safe to call twice

`kind` is a short lower-case word saying what the payload is (`entity` = the body of one
`PUT /api/v1/entities`), `payload` is plain JSON data and `t` is the time of the request as an aware datetime.
"""

from __future__ import annotations

from datetime import datetime
from typing import Any, Optional, Protocol, runtime_checkable


@runtime_checkable
class Sink(Protocol):
    """What every sink offers (see the module docstring)."""

    def write(self, kind: str, payload: Any, t: Optional[datetime] = None) -> bool:
        """Hand over one request. Returns True when it was accepted."""

    def close(self) -> None:
        """Flush and release resources. Safe to call more than once."""
