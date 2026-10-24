"""
lattice_bridge/logutil.py -- PURPOSE: the one place where the library writes log lines of its own for its
callers. The builders are pure (they do no I/O, so they cannot log), but some problems must be shouted about
LOUDLY, once: the main one is "the drone's height is being published as if height above sea level (AMSL) were
height above the ellipsoid (HAE), because nobody configured the geoid undulation", which puts the drone about
30 m off in Lattice. The bridge calls warn_if_altitude_approximate() once at start-up with its settings.

A warning is logged only the FIRST time per message key in a process, so a bridge that publishes once a second
does not fill the log. Thread safe, bounded (at most 100 different keys are remembered), never raises.

Python standard library only (logging, threading).
"""

from __future__ import annotations

import logging
import threading
from typing import Optional

from .entities import ALTITUDE_WARNING, altitude_is_approximate

LOG = logging.getLogger('lattice_bridge')

_MAX_KEYS = 100
_lock = threading.Lock()
_seen = set()


def warn_once(key: str, message: str, logger: Optional[logging.Logger] = None) -> bool:
    """Log `message` at WARNING level the first time `key` is seen in this process; True if it was logged."""
    with _lock:
        if key in _seen or len(_seen) >= _MAX_KEYS:
            return False
        _seen.add(key)
    try:
        (logger or LOG).warning('%s', message)
    except Exception:                                   # a broken log handler must never reach the caller
        return False
    return True


def warn_if_altitude_approximate(geoid_undulation_m: Optional[float], logger: Optional[logging.Logger] = None) -> bool:
    """Log the long altitude warning (once per process) when no geoid undulation is configured.

    `geoid_undulation_m` is the same setting the drone builder takes: None = not configured (warn), any
    number, including 0.0, = somebody decided (silent). Returns True when the warning was logged now.
    """
    if not altitude_is_approximate(geoid_undulation_m):
        return False
    return warn_once('altitude-approximate', ALTITUDE_WARNING, logger)


def reset_warnings() -> None:
    """Forget which warnings were logged (for tests, and for a bridge that restarts its mission)."""
    with _lock:
        _seen.clear()
