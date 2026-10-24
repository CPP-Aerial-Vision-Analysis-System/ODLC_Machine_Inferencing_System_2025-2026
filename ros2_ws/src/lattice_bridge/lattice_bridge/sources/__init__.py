"""
lattice_bridge/sources/__init__.py -- PURPOSE: the "sources" of the bridge. A source is where the facts come
from: the MAVLink stream of the telemetry radio (mavlink_source), the ROS topics of MAVROS when the bridge
runs on the Jetson itself (ros_source), or a recorded run folder from the simulator (replay_source). They all
offer the same few calls, written down here as the `Source` protocol, so the bridge core never needs to know
which one it is talking to:

    source.start()              open the connection / the ROS node. Raises SourceUnavailable when the source
                                can never work here (a library is missing, the run folder does not exist).
                                Transient trouble (nobody is sending yet) is NOT an error: the source keeps
                                trying and says so with SourceNotice events.
    source.poll(timeout_s)      wait up to timeout_s seconds and return the events that arrived (possibly an
                                empty list). Never raises for outside trouble; never blocks longer than
                                timeout_s plus a little.
    source.now()                the current time as an aware UTC datetime IN THE SOURCE'S TIME BASE. For live
                                sources that is the real clock; a replay returns the recorded time it has
                                reached, so every timing decision in the bridge (1 Hz publishing, "the link
                                went quiet") also works when a recording is played faster than real time.
    source.close()              release everything. Safe to call twice.
    source.describe()           one line for the log and the dry-run plan (never contains a secret).
    source.stats()              a dict of counters for the health line.

`altitude_reference` says what the DroneFix heights mean: 'amsl' (above mean sea level, needs the geoid
undulation to become Lattice's ellipsoid height) or 'hae' (already above the ellipsoid) or 'none'.

Nothing is imported here except the standard library, so importing the package stays cheap and never pulls in
ROS or pymavlink; each source module imports its own heavy library lazily, inside `start()`.
"""

from __future__ import annotations

from datetime import datetime
from typing import Dict, List, Protocol, runtime_checkable

from ..events import Event

#: Names the command line accepts for --source.
SOURCE_NAMES = ('mavlink', 'ros', 'replay')


class SourceUnavailable(RuntimeError):
    """The source cannot work on this computer; the message says what to install, fix or change."""


@runtime_checkable
class Source(Protocol):
    """What every source offers (see the module docstring)."""

    name: str
    altitude_reference: str

    def start(self) -> None:
        """Open the source. Raises SourceUnavailable when it can never work here."""

    def poll(self, timeout_s: float) -> List[Event]:
        """Wait up to `timeout_s` and return the events that arrived."""

    def now(self) -> datetime:
        """The current time (aware UTC) in this source's time base."""

    def close(self) -> None:
        """Release everything. Safe to call more than once."""

    def describe(self) -> str:
        """One line saying what this source reads (no secrets)."""

    def stats(self) -> Dict[str, int]:
        """Counters for the health line."""
