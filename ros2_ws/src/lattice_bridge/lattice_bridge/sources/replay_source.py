"""
lattice_bridge/sources/replay_source.py -- PURPOSE: play a recorded simulator run through the bridge, so a
teammate can try the bridge (and look at the result in the fake or the real Lattice) with no simulator running.
The simulator's demo.sh writes one folder per flight, ~/.astra_sim/runs/<date>-<time>_<world>/ ; this source
reads these files from it and produces the same events a live source would:

    photos/*.json    one sidecar per photo: where the drone was when the shutter fired (latitude, longitude,
                     height above sea level, height above home, compass heading, and how old the GPS fix was)
                     -> a DroneFix per photo. The camera runs only during the search, so this is the drone's
                     track while it searched.
    events.jsonl     the run's log: armed / disarmed (+ the mode), the team's STATUSTEXT lines (a `TGT ...`
                     report is decoded exactly as on the radio; "Servo 13 -> 1400" is a release), the
                     `aim` records (the position main_controller chose to fly to for each class) and the
                     `detection` records (their scores)
    phases.jsonl     "flight mode STABILIZE -> GUIDED" lines -> the flight mode shown on the drone
    run_data.json    the drops with their exact release positions, and the aims
    score_report.json  (optional) the scorer's choice per class: used for a confidence when nothing else has one

What is NOT in a run folder: the drone's position between the photos. So a replayed drone is visible while
the camera ran and at the two release points, and "goes stale" in between: the replay shows exactly what was
recorded and never fills the gaps with invented positions. (A MAVLink recording of the whole flight would need
a different source.)

Targets: a run recorded by an older team stack has no `TGT` lines (the Jetson-side reporter is new), so each
`aim` record is replayed as one target report, with the best detection score of that class up to that moment as
its confidence (rounded to whole percent, as the radio message would do). If the run does hold `TGT` lines,
those are used instead (never both, so nothing is counted twice).

Time: events are played at `speed` times real time (0 = as fast as possible: the virtual clock then advances
`step_s` seconds per poll). With time_mode "shift" (default) the whole run is moved so that it starts NOW (or at
`start_at`): a real Lattice only accepts entities whose expiry is in the future, and a recording from yesterday
would be refused. With "recorded" the original dates are kept (deterministic; for tests and the fake sink).
`now()` returns the virtual time, so the bridge's 1 Hz rate and "no data for 10 s" rules work at any speed.

Standard library only; reading a folder is the only I/O.
"""

from __future__ import annotations

import json
import logging
import math
import re
import time
from bisect import bisect_right
from dataclasses import dataclass, field, replace
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from .. import constants as C
from ..events import DroneFix, DropEvent, Event, SourceNotice, TargetSighting
from ..geo import is_real_number
from ..timeutil import from_unix_seconds
from . import SourceUnavailable
from .common import DEFAULT_DROP_CHANNELS, DEFAULT_DROP_OPEN_PWM, TextEventDecoder

LOG = logging.getLogger('lattice_bridge.replay')

_MODE_CHANGE = re.compile(r'flight mode (?P<old>[A-Za-z0-9_ ]{1,24}?) -> (?P<new>[A-Za-z0-9_]{1,24})')
_PRIORITY = {DroneFix: 0, TargetSighting: 1, DropEvent: 2, SourceNotice: 3}
#: A photo's GPS fix is at most this old when the sidecar says so (guards against a corrupt age_s).
_MAX_FIX_AGE_S = 5.0
TIME_MODES = ('shift', 'recorded')


@dataclass
class RunRecording:
    """Everything usable that was found in one run folder, as events in RECORDED time, sorted."""
    run_dir: str
    world: str
    events: List[Tuple[datetime, Event]]
    notes: List[str] = field(default_factory=list)
    counts: Dict[str, int] = field(default_factory=dict)

    @property
    def name(self) -> str:
        """The folder name (no path)."""
        return Path(self.run_dir).name

    @property
    def start(self) -> datetime:
        """Time of the first event."""
        return self.events[0][0]

    @property
    def end(self) -> datetime:
        """Time of the last event."""
        return self.events[-1][0]

    @property
    def label(self) -> str:
        """The world name reduced to lowercase letters and digits, for the mission id ('runway')."""
        return re.sub(r'[^a-z0-9]', '', self.world.lower()) or 'replay'


# ===================================================================================================
# Reading the folder
# ===================================================================================================

def _read_jsonl(path: Path, notes: List[str]) -> List[dict]:
    """The JSON objects of a .jsonl file; unreadable lines are skipped and mentioned in `notes`."""
    try:
        raw = path.read_bytes()
    except OSError:
        return []
    items, bad = [], 0
    for line in raw.splitlines():
        if not line.strip():
            continue
        try:
            obj = json.loads(line.decode('utf-8'))
        except (ValueError, UnicodeError):
            bad += 1
            continue
        if isinstance(obj, dict):
            items.append(obj)
        else:
            bad += 1
    if bad:
        notes.append(f'{path.name}: {bad} unreadable line(s) skipped')
    return items


def _read_json(path: Path, notes: List[str]) -> Optional[dict]:
    """A JSON object from a file, or None (and a note) when it is missing or unreadable."""
    try:
        obj = json.loads(path.read_text(encoding='utf-8'))
    except OSError:
        return None
    except (ValueError, UnicodeError):
        notes.append(f'{path.name}: not valid JSON, skipped')
        return None
    return obj if isinstance(obj, dict) else None


def _num(value) -> Optional[float]:
    """A finite float, or None."""
    return float(value) if is_real_number(value) else None


def _utc(seconds) -> Optional[datetime]:
    """Unix seconds -> aware UTC, or None when missing/garbage."""
    if not is_real_number(seconds):
        return None
    try:
        return from_unix_seconds(seconds)
    except ValueError:
        return None


class _StateTimeline:
    """Flight mode and armed flag as they were at any moment of the run (from events.jsonl and phases.jsonl)."""

    def __init__(self):
        self._modes: List[Tuple[datetime, str]] = []
        self._armed: List[Tuple[datetime, bool]] = []

    def add_mode(self, t: datetime, mode: str) -> None:
        """Record "the mode became `mode` at `t`"."""
        self._modes.append((t, mode))

    def add_armed(self, t: datetime, armed: bool) -> None:
        """Record "armed / disarmed at `t`"."""
        self._armed.append((t, armed))

    def freeze(self) -> None:
        """Sort once everything is added."""
        self._modes.sort(key=lambda item: item[0])
        self._armed.sort(key=lambda item: item[0])

    def at(self, t: datetime) -> Tuple[Optional[str], Optional[bool]]:
        """(mode, armed) in force at `t`; None where nothing was recorded yet."""
        mode = armed = None
        index = bisect_right([m[0] for m in self._modes], t)
        if index:
            mode = self._modes[index - 1][1]
        index = bisect_right([a[0] for a in self._armed], t)
        if index:
            armed = self._armed[index - 1][1]
        return mode, armed


def _fix_from_sidecar(data: dict, timeline: _StateTimeline) -> Optional[Tuple[datetime, DroneFix]]:
    """One photo sidecar -> (time, DroneFix), or None when it holds no usable position."""
    shutter = _utc(data.get('shutter_unix_s')) or _utc(data.get('trigger_unix_s'))
    position = data.get('position')
    if shutter is None or not isinstance(position, dict):
        return None
    age = _num(position.get('age_s'))
    age = min(max(age, 0.0), _MAX_FIX_AGE_S) if age is not None else 0.0
    mode, armed = timeline.at(shutter)
    try:
        fix = DroneFix(
            fix_time=shutter - timedelta(seconds=age), arrival=shutter,
            lat_deg=position.get('lat'), lon_deg=position.get('lon'),
            alt_amsl_m=_clean(position.get('alt_msl_m'), C.ALT_MIN_M, C.ALT_MAX_M),
            rel_alt_m=_clean(data.get('rel_alt_m'), C.ALT_MIN_M, C.ALT_MAX_M),
            heading_deg=_clean(data.get('heading_deg'), 0.0, 360.0),
            mode=mode, armed=armed, time_source='recorded')
    except ValueError:
        return None
    return shutter, fix


def _clean(value, low: float, high: float) -> Optional[float]:
    """A number inside [low, high], or None."""
    number = _num(value)
    return number if number is not None and low <= number <= high else None


def load_run(run_dir, *, drop_channels: Optional[Dict[int, str]] = None,
             drop_open_pwm: int = DEFAULT_DROP_OPEN_PWM) -> RunRecording:
    """Read a run folder into a RunRecording (see the module docstring for what is used).

    Raises SourceUnavailable when the folder does not exist or holds nothing usable. Missing optional files,
    unreadable lines and photos without a position are skipped and listed in `RunRecording.notes`.
    """
    root = Path(str(run_dir)).expanduser()
    if not root.is_dir():
        raise SourceUnavailable(f'replay: the run folder {str(root)!r} does not exist. Runs are written by the '
                                f'simulator to ~/.astra_sim/runs/<date>_<world>/ (see docs/lattice/BRIDGE.md).')
    notes: List[str] = []
    log_items = _read_jsonl(root / 'events.jsonl', notes)
    phase_items = _read_jsonl(root / 'phases.jsonl', notes)
    run_data = _read_json(root / 'run_data.json', notes) or {}
    score = _read_json(root / 'score_report.json', notes) or {}

    timeline = _StateTimeline()
    for item in log_items:
        t = _utc(item.get('t'))
        if t is not None and item.get('kind') in ('armed', 'disarmed'):
            timeline.add_armed(t, item.get('kind') == 'armed')
            if isinstance(item.get('mode'), str) and item['mode']:
                timeline.add_mode(t, item['mode'])
    for item in phase_items:
        t = _utc(item.get('time'))
        match = _MODE_CHANGE.fullmatch(str(item.get('text', ''))) if t is not None else None
        if match:
            timeline.add_mode(t, match.group('new'))
    timeline.freeze()

    timed: List[Tuple[datetime, Event]] = []
    counts = {'fixes': 0, 'sightings': 0, 'drops': 0}

    sidecars = sorted((root / 'photos').glob('*.json')) if (root / 'photos').is_dir() else []
    skipped = 0
    for path in sidecars:
        data = _read_json(path, notes)
        built = _fix_from_sidecar(data, timeline) if data else None
        if built is None:
            skipped += 1
            continue
        timed.append(built)
        counts['fixes'] += 1
    if skipped:
        notes.append(f'photos: {skipped} sidecar(s) without a usable position were skipped')
    timed.sort(key=lambda item: item[0])

    # --- drops: the exact release positions of run_data.json win over "Servo 13 -> 1400" lines ---------
    drops = run_data.get('drops') if isinstance(run_data.get('drops'), list) else []
    drop_events: List[Tuple[datetime, Event]] = []
    for item in drops:
        t = _utc(item.get('t')) if isinstance(item, dict) else None
        kind = item.get('cls') if isinstance(item, dict) else None
        if t is None or kind not in C.TARGET_KINDS:
            continue
        try:
            drop_events.append((t, DropEvent(kind, t, item.get('release_lat'), item.get('release_lon'))))
        except ValueError:
            notes.append('run_data.json: a drop without a valid position was skipped')
    drop_from_data = bool(drop_events)

    # --- statustext lines: TGT reports (and release lines when run_data.json had none) -----------------
    decoder = TextEventDecoder(drop_channels=drop_channels or dict(DEFAULT_DROP_CHANNELS),
                               drop_open_pwm=drop_open_pwm)
    text_events: List[Tuple[datetime, Event]] = []
    fixes_only = [(t, e) for t, e in timed if isinstance(e, DroneFix)]
    for item in log_items:
        if item.get('kind') != 'statustext' or not isinstance(item.get('text'), str):
            continue
        t = _utc(item.get('t'))
        if t is None:
            continue
        index = bisect_right([f[0] for f in fixes_only], t)
        last_fix = fixes_only[index - 1][1] if index else None
        for event in decoder.decode(item['text'], C.STATUSTEXT_SEVERITY_INFO, t, last_fix):
            if isinstance(event, DropEvent) and drop_from_data:
                continue
            text_events.append((t, event))
    has_tgt = any(isinstance(e, TargetSighting) for _, e in text_events)

    # --- aims: replayed as target reports when the run holds no TGT lines -----------------------------
    aim_events: List[Tuple[datetime, Event]] = []
    if not has_tgt:
        scores = _detection_scores(log_items)
        aims = [i for i in log_items if i.get('kind') == 'aim'] or [
            i for i in (run_data.get('aims') or []) if isinstance(i, dict)]
        choices = {c.get('class'): _num(c.get('confidence')) for c in (score.get('choices') or [])
                   if isinstance(c, dict)}
        for item in aims:
            t, kind = _utc(item.get('t')), item.get('cls')
            if t is None or kind not in C.TARGET_KINDS:
                continue
            confidence = _best_score_until(scores, kind, t)
            if confidence is None:
                confidence = choices.get(kind)
            if confidence is None:
                notes.append(f'aim for a {kind} has no detection score: not replayed')
                continue
            try:
                aim_events.append((t, TargetSighting(kind, item.get('lat'), item.get('lon'),
                                                     round(min(max(confidence, 0.0), 1.0), 2), t, origin='aim')))
            except ValueError:
                notes.append(f'aim for a {kind} has an invalid position: not replayed')

    # --- the drone at each release (a recorded position; keeps the drone on the map there) -------------
    release_fixes: List[Tuple[datetime, Event]] = []
    for t, event in drop_events:
        mode, armed = timeline.at(t)
        try:
            release_fixes.append((t, DroneFix(fix_time=t, arrival=t, lat_deg=event.lat_deg, lon_deg=event.lon_deg,
                                              mode=mode, armed=armed, time_source='recorded')))
        except ValueError:
            continue

    events = timed + text_events + aim_events + drop_events + release_fixes
    counts['fixes'] = sum(1 for _, e in events if isinstance(e, DroneFix))
    counts['sightings'] = sum(1 for _, e in events if isinstance(e, TargetSighting))
    counts['drops'] = sum(1 for _, e in events if isinstance(e, DropEvent))
    if not events:
        raise SourceUnavailable(f'replay: the run folder {root.name!r} holds nothing the bridge can use '
                                f'(no photo sidecars with a position, no TGT lines, no aims, no drops)')
    events.sort(key=lambda item: (item[0], _PRIORITY.get(type(item[1]), 9)))
    world = str(run_data.get('world') or root.name.split('_', 1)[-1] or 'replay')
    return RunRecording(str(root), world, events, notes, counts)


def _detection_scores(log_items: List[dict]) -> Dict[str, List[Tuple[datetime, float]]]:
    """{class: [(time, score), ...]} from the `detection` records of events.jsonl, sorted by time."""
    scores: Dict[str, List[Tuple[datetime, float]]] = {kind: [] for kind in C.TARGET_KINDS}
    for item in log_items:
        t = _utc(item.get('t'))
        if t is None or item.get('kind') != 'detection' or not isinstance(item.get('boxes'), list):
            continue
        for box in item['boxes']:
            if isinstance(box, dict) and box.get('cls') in scores and _num(box.get('score')) is not None:
                scores[box['cls']].append((t, float(box['score'])))
    for series in scores.values():
        series.sort(key=lambda entry: entry[0])
    return scores


def _best_score_until(scores: Dict[str, List[Tuple[datetime, float]]], kind: str, t: datetime) -> Optional[float]:
    """The highest score of class `kind` seen up to time `t` (None when there was none)."""
    values = [score for when, score in scores.get(kind, []) if when <= t]
    return max(values) if values else None


# ===================================================================================================
# Playing it
# ===================================================================================================

def shift_event(event: Event, delta: timedelta) -> Event:
    """A copy of `event` with every time moved by `delta`."""
    if isinstance(event, DroneFix):
        return replace(event, fix_time=event.fix_time + delta, arrival=event.arrival + delta)
    if isinstance(event, TargetSighting):
        return replace(event, measured_time=event.measured_time + delta)
    if isinstance(event, DropEvent):
        return replace(event, release_time=event.release_time + delta)
    return replace(event, time=event.time + delta)


class ReplaySource:
    """A Source that plays a RunRecording (see the module docstring).

    `speed` > 0 plays in (virtual) real time multiplied by speed; `speed` == 0 plays as fast as possible, one
    virtual `step_s` per poll. `start_at` pins the start of a "shift" replay (tests use it; the default is the
    clock's now). `clock`, `monotonic` and `sleep` are injectable.
    """

    name = 'replay'
    altitude_reference = 'amsl'

    def __init__(self, run_dir, *, speed: float = 1.0, time_mode: str = 'shift', start_at: Optional[datetime] = None,
                 step_s: float = 1.0, drop_channels: Optional[Dict[int, str]] = None,
                 drop_open_pwm: int = DEFAULT_DROP_OPEN_PWM, clock: Optional[Callable[[], datetime]] = None,
                 monotonic: Optional[Callable[[], float]] = None, sleep: Optional[Callable[[float], None]] = None):
        if not is_real_number(speed) or not 0.0 <= speed <= 10000.0:
            raise ValueError('speed must be a number from 0 (as fast as possible) to 10000')
        if time_mode not in TIME_MODES:
            raise ValueError(f'time mode must be one of {TIME_MODES}, got {str(time_mode)[:20]!r}')
        if not is_real_number(step_s) or not 0.01 <= step_s <= 3600.0:
            raise ValueError('step_s must be between 0.01 and 3600 seconds')
        self.run_dir = str(run_dir)
        self.speed = float(speed)
        self.time_mode = time_mode
        self.start_at = start_at
        self.step_s = float(step_s)
        self._drop_channels = drop_channels
        self._drop_open_pwm = drop_open_pwm
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._monotonic = monotonic or time.monotonic
        self._sleep = sleep or time.sleep
        self.recording: Optional[RunRecording] = None
        self.suggested_label: Optional[str] = None
        self._items: List[Tuple[datetime, Event]] = []      # shifted, sorted
        self._index = 0
        self._virtual_start: Optional[datetime] = None
        self._cursor: Optional[datetime] = None             # speed == 0: where virtual time is
        self._wall_start = 0.0
        self._announced = False
        self._ended = False
        self.counters = {'delivered': 0, 'fixes': 0, 'sightings': 0, 'drops': 0, 'total': 0}

    # ------------------------------------------------------------------------------------------ Source API
    def start(self) -> None:
        """Read the run folder (SourceUnavailable if it is missing or empty) and set the virtual clock."""
        self.recording = load_run(self.run_dir, drop_channels=self._drop_channels, drop_open_pwm=self._drop_open_pwm)
        for note in self.recording.notes:
            LOG.warning('replay %s: %s', self.recording.name, note)
        rec = self.recording
        if self.time_mode == 'recorded':
            delta, self._virtual_start = timedelta(0), rec.start
        else:
            begin = self.start_at or self._clock()
            delta, self._virtual_start = begin - rec.start, begin
        self._items = [(t + delta, shift_event(e, delta)) for t, e in rec.events]
        self._cursor = self._virtual_start
        self._wall_start = self._monotonic()
        self._index = 0
        self.suggested_label = rec.label
        self.counters['total'] = len(self._items)

    def now(self) -> datetime:
        """The virtual time the replay has reached."""
        if self._virtual_start is None:
            return self._clock()
        if self.speed > 0:
            return self._virtual_start + timedelta(seconds=(self._monotonic() - self._wall_start) * self.speed)
        return self._cursor

    def describe(self) -> str:
        """One line for the log: the folder name, size, speed and time handling (no full path)."""
        name = Path(self.run_dir).name
        if self.recording is None:
            return f'replay of run folder {name}'
        rec = self.recording
        duration = (rec.end - rec.start).total_seconds()
        speed = 'as fast as possible' if self.speed == 0 else f'{self.speed:g}x real time'
        return (f'replay of run {name} ({len(rec.events)} events over {duration:.0f} s, {speed}, '
                f'times {"moved to now" if self.time_mode == "shift" else "as recorded"})')

    def stats(self) -> Dict[str, int]:
        """Counters: events delivered so far by kind, and the total in the recording."""
        return dict(self.counters)

    def close(self) -> None:
        """Nothing to release; kept so every source can be closed the same way."""
        return None

    def poll(self, timeout_s: float) -> List[Event]:
        """Return the events that are due in virtual time (see the module docstring)."""
        if self._virtual_start is None:
            raise RuntimeError('ReplaySource.start() must be called first')
        events: List[Event] = []
        if not self._announced:
            self._announced = True
            events.append(SourceNotice('info', self.describe(), self.now()))
        if self._ended:
            self._sleep(min(max(timeout_s, 0.0), 0.1))
            return events
        if self.speed > 0:
            self._poll_realtime(events, timeout_s)
        else:
            self._cursor = self._cursor + timedelta(seconds=self.step_s)
            events.extend(self._take_due(self._cursor))
        if self._index >= len(self._items):
            self._ended = True
            events.append(SourceNotice('ended', f'replay of {self.recording.name} finished '
                                                f'({self.counters["delivered"]} events)', self.now()))
        return events

    # ------------------------------------------------------------------------------------------ internals
    def _poll_realtime(self, events: List[Event], timeout_s: float) -> None:
        """speed > 0: wait (up to timeout_s) for the next event to become due; return what is due."""
        deadline = self._monotonic() + max(0.0, timeout_s)
        while True:
            due = self._take_due(self.now())
            events.extend(due)
            if due or self._index >= len(self._items):
                return
            remaining = deadline - self._monotonic()
            if remaining <= 0:
                return
            until_next = (self._items[self._index][0] - self.now()).total_seconds() / self.speed
            self._sleep(max(0.0, min(remaining, until_next, 0.2)))

    def _take_due(self, until: datetime) -> List[Event]:
        """Events whose (shifted) time is <= `until`, in order."""
        due: List[Event] = []
        while self._index < len(self._items) and self._items[self._index][0] <= until:
            event = self._items[self._index][1]
            self._index += 1
            self.counters['delivered'] += 1
            if isinstance(event, DroneFix):
                self.counters['fixes'] += 1
            elif isinstance(event, TargetSighting):
                self.counters['sightings'] += 1
            elif isinstance(event, DropEvent):
                self.counters['drops'] += 1
            due.append(event)
        return due
