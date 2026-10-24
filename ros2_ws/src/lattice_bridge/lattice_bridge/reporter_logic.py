"""
lattice_bridge/reporter_logic.py -- PURPOSE: the DECISIONS of the Jetson-side target reporter, with no ROS, no
file access and no clock in it (the caller passes the time), so every rule can be tested with plain numbers.

The reporter (reporter_node.py) runs next to the team's flight nodes on the Jetson. It listens to the detector,
works out where each detected hiker or tent is on the ground, and sends the ground laptop a short radio text
(`TGT p 36.214935 -96.006203 95 12`, see report_codec.py). This module answers the two questions that matter:

  1. "Is this sighting worth a radio message?"   -> TargetReporter.offer()
  2. "May a message go out right now?"           -> TargetReporter.next_message()

The rules, written down on purpose (test_reporter_logic.py checks every line):
  * One list of KNOWN TARGETS (at most `max_targets`). A sighting belongs to a known target when it is the same
    kind (person / tent) and within `same_target_radius_m` (12 m, the same number the ground clusterer uses) of
    that target's best sighting. The nearest known target wins.
  * Each target remembers its BEST sighting: the one with the highest confidence (a tie keeps the older one).
    The message always carries ONE real sighting (its position and its confidence), never an average, so the
    place on the Lattice map is the place main_controller will aim at (it also flies to its best sighting).
  * A message is wanted when a target is
        NEW          the first sighting of a target                     -> always reported
        BETTER       its best confidence is at least `min_gain_points` percentage points above the one
                     the ground was last told                           -> reported again
        MOVED        its best sighting is more than `min_move_m` metres from the position the ground was
                     last told                                           -> reported again
    Everything else is quiet: a hiker seen in 20 photos must not cost 20 radio messages.
  * At most `max_messages_per_target` messages per target and `max_messages_total` in all (a runaway detector
    must not be able to flood the telemetry radio). The first message of a target is never starved by the
    per-target limit.
  * Sending is rate limited: at most one message per `min_interval_s` for ALL targets together. Wanted messages
    wait in a queue with at most ONE entry per target (a newer, better sighting replaces the waiting one), so
    the queue can never be longer than the target list. NEW goes before BETTER before MOVED; ties are served
    oldest first. "Reported" is recorded only when a message is really handed out by next_message().
  * Bad input never raises and never creates a message: an unknown kind, a NaN / infinite / out-of-range
    position, the (0, 0) "no GPS" position, a confidence outside 0..1 or <= 0 or below `min_confidence` is
    ignored with a reason (`Offer.reason`) and counted. A bad clock value (not a number) is a programming error
    and raises ValueError, like a naive datetime does in the rest of the package.
  * Memory is bounded: at most `max_targets` small records, at most that many waiting reports, a handful of
    counters.

Also here, because the node, the YAML file, the launch file and the docs must agree on them: PARAMETERS (every
setting of the node with its default, range and a plain-words explanation), clean_parameters() (turns raw ROS
parameter values into valid ones, replacing a bad value with the default and saying so), AimSettings (the five
settings that must equal main_controller's) and OnceGate (log a problem the first time only).

Pure Python (math, dataclasses, typing): no ROS, no I/O, no clock reads.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Dict, List, Mapping, Optional, Tuple

from . import constants as C
from .geo import haversine_m, is_real_number, require_lat_lon
from .report_codec import encode_tgt, next_seq

# ---------------------------------------------------------------------------------------------------
# The settings of the node: one table, used by the node, the YAML file, the docs and the tests
# ---------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class ParamSpec:
    """One setting: its name, default, type, allowed range and what it does (plain words).

    `kind` is 'float', 'int', 'bool' or 'str'. `low` / `high` are inclusive limits (None = no limit besides
    "a finite number"). `group` sorts the settings in the docs: 'what', 'aim', 'send' or 'ops'.
    """
    name: str
    default: Any
    kind: str
    low: Optional[float]
    high: Optional[float]
    group: str
    text: str


PARAMETERS: Tuple[ParamSpec, ...] = (
    # ---- what counts as a target -------------------------------------------------------------
    ParamSpec('min_confidence', 0.0, 'float', 0.0, 1.0, 'what',
              'Ignore a detection whose score is below this number (0.0 to 1.0). A score of 0 is always '
              'ignored. 0.0 copies main_controller, which has no limit of its own (the detector new_od '
              'already throws away everything under 0.25).'),
    ParamSpec('allow_photo_position_fallback', False, 'bool', None, None, 'what',
              'When a photo cannot be projected onto the ground (no sidecar file, no height or heading '
              'recorded ...), main_controller falls back to the DRONE position at the shutter. false = do '
              'not report such a detection, because Lattice must show the target place and never the '
              "drone's. true = report it anyway, like main_controller would aim there."),
    ParamSpec('photo_dir', '', 'str', None, None, 'what',
              'The folder with the photos and their .json sidecar files. "" = find it the way '
              'main_controller does: <workspace>/src/video_cam/mapping_photos.'),
    # ---- aiming: must be the same as main_controller's ---------------------------------------
    ParamSpec('aim_at_detection_box', True, 'bool', None, None, 'aim',
              "Same as main_controller's setting: true = project the middle of the detection box onto the "
              "ground; false = use the drone's own position."),
    ParamSpec('default_camera_pitch_deg', -90.0, 'float', None, None, 'aim',
              "Same as main_controller's: camera pitch (-90 = straight down) when the camera did not report it."),
    ParamSpec('default_camera_hfov_deg', 81.0, 'float', None, None, 'aim',
              "Same as main_controller's: field of view in degrees when a sidecar has none."),
    ParamSpec('target_ground_height_m', 0.0, 'float', None, None, 'aim',
              "Same as main_controller's: height of the ground under the targets above the launch point, metres."),
    ParamSpec('max_aim_offset_m', 150.0, 'float', None, None, 'aim',
              "Same as main_controller's: an aim point farther than this from the photo is not trusted, metres."),
    ParamSpec('follow_main_controller', True, 'bool', None, None, 'aim',
              'true = ask the running main_controller for the five aim settings above (once at start-up and '
              'then every follow_period_s) and use ITS values, so the Lattice picture always matches where '
              'the drone will really aim. false = use the values written here.'),
    ParamSpec('follow_period_s', 60.0, 'float', 0.0, 3600.0, 'aim',
              'How often to ask main_controller again, in seconds. 0 = ask only until the first answer.'),
    # ---- when to send ------------------------------------------------------------------------
    ParamSpec('same_target_radius_m', C.CLUSTER_MERGE_RADIUS_M, 'float', 0.5, 500.0, 'send',
              'Two sightings of the same kind closer than this (metres) are the SAME target. The ground '
              'program uses the same number.'),
    ParamSpec('min_gain_points', 5.0, 'float', 1.0, 100.0, 'send',
              'Send a target again when its best confidence is at least this many percentage points higher '
              'than the one the ground was told last (N).'),
    ParamSpec('min_move_m', 5.0, 'float', 0.5, 500.0, 'send',
              'Send a target again when its best sighting is more than this many metres away from the '
              'position the ground was told last (M).'),
    ParamSpec('min_interval_s', 2.0, 'float', 0.1, 600.0, 'send',
              'At least this many seconds between two radio messages, for all targets together. Values under '
              '1 are meant for tests only.'),
    ParamSpec('max_messages_per_target', 6, 'int', 1, 100, 'send',
              'At most this many radio messages for one target in one flight (K).'),
    ParamSpec('max_messages_total', 300, 'int', 1, 10000, 'send',
              'At most this many TGT radio messages in one flight, whatever happens (a safety net for the radio).'),
    ParamSpec('max_targets', C.CLUSTER_MAX_CLUSTERS, 'int', 1, 256, 'send',
              'How many different targets are remembered. A new target beyond this is ignored (logged once).'),
    # ---- operation -----------------------------------------------------------------------------
    ParamSpec('dry_run', False, 'bool', None, None, 'ops',
              'true = decide and log everything, but do NOT publish the radio messages. Use it for the first '
              'look on the real drone.'),
    ParamSpec('queue_size', 16, 'int', 1, 256, 'ops',
              'How many detector answers may wait to be processed. When more arrive the oldest are dropped '
              '(and counted).'),
    ParamSpec('summary_period_s', 60.0, 'float', 0.0, 3600.0, 'ops',
              'Seconds between two one-line status messages in the log. 0 = only at the end.'),
    ParamSpec('tick_period_s', 0.5, 'float', 0.02, 5.0, 'ops',
              'How often the node wakes up to process answers and send a message. Leave alone (tests use less).'),
)

PARAM_BY_NAME: Dict[str, ParamSpec] = {spec.name: spec for spec in PARAMETERS}
PARAM_GROUPS = ('what', 'aim', 'send', 'ops')
#: The five settings that main_controller also has; the reporter must use the same ones.
AIM_NAMES = ('aim_at_detection_box', 'default_camera_pitch_deg', 'default_camera_hfov_deg',
             'target_ground_height_m', 'max_aim_offset_m')


def _short(value) -> str:
    """A short, safe text for messages (a hostile value must not flood a log)."""
    return repr(value)[:30]


def clean_value(spec: ParamSpec, raw) -> Tuple[Any, str]:
    """Return (value, problem) for one raw parameter value.

    `problem` is '' when the raw value is fine. Otherwise `value` is the DEFAULT and `problem` says what was
    wrong, in one sentence. Numbers must be finite (NaN and infinity are refused: they slip through every
    ordinary comparison); a whole-number float such as 6.0 is accepted for an integer setting; `true` / `false`
    in text form are accepted for a yes/no setting (launch arguments arrive as text).
    """
    default = spec.default
    if spec.kind == 'bool':
        if isinstance(raw, bool):
            return raw, ''
        if isinstance(raw, str) and raw.strip().lower() in ('true', 'false'):
            return raw.strip().lower() == 'true', ''
        return default, f'{spec.name} must be true or false, got {_short(raw)}; using the default {default}'
    if spec.kind == 'str':
        if isinstance(raw, str):
            return raw.strip(), ''
        return default, f'{spec.name} must be text, got {_short(raw)}; using the default {default!r}'
    if spec.kind == 'int':
        if isinstance(raw, bool) or not is_real_number(raw) or float(raw) != math.floor(float(raw)):
            return default, f'{spec.name} must be a whole number, got {_short(raw)}; using the default {default}'
        value: Any = int(raw)
    else:
        if not is_real_number(raw):
            return default, f'{spec.name} must be a finite number, got {_short(raw)}; using the default {default}'
        value = float(raw)
    if (spec.low is not None and value < spec.low) or (spec.high is not None and value > spec.high):
        return default, (f'{spec.name} must be between {spec.low} and {spec.high}, got {value}; '
                         f'using the default {default}')
    return value, ''


def clean_parameters(raw: Mapping[str, Any]) -> Tuple[Dict[str, Any], List[str]]:
    """Turn raw ROS parameter values into a full set of valid ones: (values, problems).

    Every name of PARAMETERS is in `values`. A missing or bad entry becomes its default and adds one sentence
    to `problems` (a missing entry adds none). The node logs the problems and starts anyway: a mistyped number
    must never stop a passive helper from starting, and the defaults are the safe ones.
    """
    values: Dict[str, Any] = {}
    problems: List[str] = []
    for spec in PARAMETERS:
        if spec.name not in raw:
            values[spec.name] = spec.default
            continue
        value, problem = clean_value(spec, raw[spec.name])
        values[spec.name] = value
        if problem:
            problems.append(problem)
    return values, problems


@dataclass(frozen=True)
class ReporterSettings:
    """The settings that drive the decisions of TargetReporter (a subset of PARAMETERS).

    The defaults are the table's defaults. Construction raises ValueError for a value outside its range, so a
    TargetReporter can never run with a nonsense limit. `start_seq` is the first message counter (0 = a fresh
    flight).
    """
    min_confidence: float = 0.0
    same_target_radius_m: float = C.CLUSTER_MERGE_RADIUS_M
    min_gain_points: float = 5.0
    min_move_m: float = 5.0
    min_interval_s: float = 2.0
    max_messages_per_target: int = 6
    max_messages_total: int = 300
    max_targets: int = C.CLUSTER_MAX_CLUSTERS
    start_seq: int = 0

    def __post_init__(self):
        for item in fields(self):
            if item.name == 'start_seq':
                continue
            value, problem = clean_value(PARAM_BY_NAME[item.name], getattr(self, item.name))
            if problem:
                raise ValueError(problem)
            # A whole number given for a decimal setting (2 for 2.0) or the reverse (6.0 for 6) is turned into
            # the declared type, so the rest of the code always sees the same types.
            object.__setattr__(self, item.name, value)
        if isinstance(self.start_seq, bool) or not isinstance(self.start_seq, int) \
                or not 0 <= self.start_seq <= C.TGT_SEQ_MAX:
            raise ValueError(f'start_seq must be an integer from 0 to {C.TGT_SEQ_MAX}')

    @classmethod
    def from_values(cls, values: Mapping[str, Any]) -> 'ReporterSettings':
        """Build the settings from a cleaned parameter dict (see clean_parameters)."""
        names = [item.name for item in fields(cls) if item.name != 'start_seq']
        return cls(**{name: values[name] for name in names})


@dataclass(frozen=True)
class AimSettings:
    """The five aiming settings main_controller has; the reporter must use the same ones.

    from_values() follows main_controller's own rule (`_number_parameter` there): a number that is not a finite
    number is replaced by the default; the yes/no setting is read as true or false.
    """
    aim_at_detection_box: bool = True
    default_camera_pitch_deg: float = -90.0
    default_camera_hfov_deg: float = 81.0
    target_ground_height_m: float = 0.0
    max_aim_offset_m: float = 150.0

    @classmethod
    def from_values(cls, values: Mapping[str, Any]) -> Tuple['AimSettings', List[str]]:
        """Return (settings, problems) from raw values; a missing name keeps its default."""
        kwargs: Dict[str, Any] = {}
        problems: List[str] = []
        for name in AIM_NAMES:
            spec = PARAM_BY_NAME[name]
            if name not in values or values[name] is None:
                kwargs[name] = spec.default
                continue
            if spec.kind == 'bool':
                raw = values[name]
                kwargs[name] = bool(raw)          # main_controller: bool(parameter value)
                continue
            value, problem = clean_value(spec, values[name])
            kwargs[name] = value
            if problem:
                problems.append(problem)
        return cls(**kwargs), problems

    def describe(self) -> str:
        """One line for the log."""
        return (f'aim_at_detection_box={self.aim_at_detection_box}, pitch={self.default_camera_pitch_deg:g}, '
                f'hfov={self.default_camera_hfov_deg:g}, ground_height={self.target_ground_height_m:g} m, '
                f'max_offset={self.max_aim_offset_m:g} m')


class OnceGate:
    """first(key) is True the first time a key is seen and False afterwards (log a problem once).

    Remembers at most `limit` different keys; after that every new key counts as seen, so a flood of different
    problems cannot grow it (bounded memory).
    """

    def __init__(self, limit: int = 64):
        self._limit = int(limit)
        self._seen = set()

    def first(self, key) -> bool:
        """True only the first time `key` is passed in (and only while there is room to remember it)."""
        if key in self._seen or len(self._seen) >= self._limit:
            return False
        self._seen.add(key)
        return True


# ---------------------------------------------------------------------------------------------------
# The decisions
# ---------------------------------------------------------------------------------------------------

#: Why a message is wanted. The order is the order they are sent in when several wait.
WHY_NEW = 'new'
WHY_BETTER = 'better'
WHY_MOVED = 'moved'
_WHY_RANK = {WHY_NEW: 0, WHY_BETTER: 1, WHY_MOVED: 2}

#: The five answers of TargetReporter.offer().
OFFER_NEW = 'new'              # the first sighting of a target: a report is waiting to be sent
OFFER_IMPROVED = 'improved'    # a better sighting: a (new or replaced) report is waiting to be sent
OFFER_UNCHANGED = 'unchanged'  # nothing worth saying
OFFER_IGNORED = 'ignored'      # bad or unwanted data (see Offer.reason)
OFFER_CAPPED = 'capped'        # worth saying, but a message limit stops it


def percent(confidence: float) -> int:
    """The whole percent the radio message carries for a confidence 0..1 (the codec's own rounding)."""
    return int(math.floor(float(confidence) * 100.0 + 0.5))


def _require_time(value, name: str = 'now_s') -> float:
    """The clock value as a float; ValueError when it is not a real number (a programming error)."""
    if not is_real_number(value):
        raise ValueError(f'{name} must be a real number of seconds, got {_short(value)}')
    return float(value)


@dataclass(frozen=True)
class Offer:
    """The answer for one sighting. `target` is the kind's own number (person 1, tent 1 ...; 0 = none).

    `key` is a short code for an ignored sighting ('unknown_kind', 'bad_position', 'bad_confidence',
    'low_confidence', 'table_full'); '' otherwise.
    """
    action: str
    reason: str = ''
    target: int = 0
    key: str = ''


@dataclass(frozen=True)
class Outgoing:
    """A radio message handed out by next_message(): the text to send and what it stands for."""
    text: str
    kind: str
    number: int
    lat_deg: float
    lon_deg: float
    confidence_percent: int
    seq: int
    why: str
    messages_for_target: int


@dataclass(frozen=True)
class TargetInfo:
    """A read-only snapshot of one known target (for logs and tests)."""
    kind: str
    number: int
    lat_deg: float
    lon_deg: float
    confidence: float
    sightings: int
    messages: int
    reported_percent: Optional[int]
    waiting: Optional[str]


class _Target:
    """One known target: its best sighting, what the ground was told last, and its waiting report."""

    __slots__ = ('kind', 'number', 'lat', 'lon', 'conf', 'rep', 'messages', 'sightings', 'first_t', 'last_t',
                 'why', 'wait_since')

    def __init__(self, kind: str, number: int, lat: float, lon: float, conf: float, now: float):
        self.kind = kind
        self.number = number
        self.lat, self.lon, self.conf = lat, lon, conf
        self.rep: Optional[Tuple[float, float, int]] = None     # (lat, lon, percent) the ground was told
        self.messages = 0
        self.sightings = 1
        self.first_t = self.last_t = now
        self.why: Optional[str] = None                          # None = nothing waiting
        self.wait_since = now


class ReporterStats:
    """Plain counters (all integers, a fixed set of names, so memory cannot grow)."""

    _NAMES = ('offered', 'ignored', 'new', 'improved', 'unchanged', 'capped', 'sent', 'encode_errors')

    def __init__(self):
        self._counts = {name: 0 for name in self._NAMES}
        self._ignored_by_reason: Dict[str, int] = {}

    def bump(self, name: str) -> None:
        """Add one to a counter."""
        self._counts[name] += 1

    def bump_ignored(self, reason_key: str) -> None:
        """Count one ignored sighting under a short reason key (at most 16 different keys are kept)."""
        if reason_key in self._ignored_by_reason or len(self._ignored_by_reason) < 16:
            self._ignored_by_reason[reason_key] = self._ignored_by_reason.get(reason_key, 0) + 1
        self._counts['ignored'] += 1

    def as_dict(self) -> Dict[str, Any]:
        """A copy of all counters (the ignored ones under 'ignored_by_reason')."""
        out: Dict[str, Any] = dict(self._counts)
        out['ignored_by_reason'] = dict(self._ignored_by_reason)
        return out

    def __getitem__(self, name: str) -> int:
        return self._counts[name]


class TargetReporter:
    """Decide which sightings become radio messages, and when each message may go out.

    Typical use (the node does exactly this):

        reporter = TargetReporter(ReporterSettings(min_interval_s=2.0))
        reporter.offer('person', lat, lon, 0.95, now_s)       # for every accepted detection
        out = reporter.next_message(now_s)                    # on a timer; None = nothing to send now
        if out: publish(out.text)

    Not thread safe: call it from one thread (the node's executor).
    """

    def __init__(self, settings: Optional[ReporterSettings] = None):
        self.settings = settings or ReporterSettings()
        self.stats = ReporterStats()
        self._targets: List[_Target] = []
        self._counters = {kind: 0 for kind in C.TARGET_KINDS}
        self._seq = self.settings.start_seq
        self._last_sent_s: Optional[float] = None
        self._sent_total = 0

    # ---------------------------------------------------------------------------------- reading
    def __len__(self) -> int:
        return len(self._targets)

    @property
    def sent_total(self) -> int:
        """How many radio messages have been handed out so far."""
        return self._sent_total

    @property
    def next_seq_number(self) -> int:
        """The counter the next message will carry."""
        return self._seq

    def waiting(self) -> int:
        """How many reports are waiting for their turn (never more than there are targets)."""
        return sum(1 for target in self._targets if target.why is not None)

    def targets(self) -> List[TargetInfo]:
        """Snapshots of all known targets, ordered by kind then number."""
        order = {kind: index for index, kind in enumerate(C.TARGET_KINDS)}
        return [TargetInfo(t.kind, t.number, t.lat, t.lon, t.conf, t.sightings, t.messages,
                           None if t.rep is None else t.rep[2], t.why)
                for t in sorted(self._targets, key=lambda t: (order[t.kind], t.number))]

    def accepts_score(self, score) -> bool:
        """True when a detection score may become a report (finite, > 0, <= 1, not below min_confidence).

        The node asks this BEFORE it reads any photo file, so a sighting that would be ignored costs no I/O.
        """
        return (is_real_number(score) and 0.0 < score <= 1.0 and score >= self.settings.min_confidence)

    def summary_line(self) -> str:
        """One line for the log: targets, messages, waiting reports and why sightings were ignored."""
        counts = {kind: sum(1 for t in self._targets if t.kind == kind) for kind in C.TARGET_KINDS}
        ignored = self.stats.as_dict()['ignored_by_reason']
        why = ', '.join(f'{key} {value}' for key, value in sorted(ignored.items())) or 'none'
        return (f'targets: {counts[C.KIND_PERSON]} person, {counts[C.KIND_TENT]} tent | radio messages sent: '
                f'{self._sent_total} (limit {self.settings.max_messages_total}) | waiting: {self.waiting()} | '
                f'sightings ignored: {self.stats["ignored"]} ({why}) | capped: {self.stats["capped"]}')

    # ---------------------------------------------------------------------------------- offering
    def _ignore(self, key: str, reason: str) -> Offer:
        self.stats.bump_ignored(key)
        return Offer(OFFER_IGNORED, reason, 0, key)

    def offer(self, kind, lat_deg, lon_deg, confidence, now_s) -> Offer:
        """Take one sighting (kind, position, confidence 0..1, time in seconds) and decide what it means.

        Never raises for bad sighting data (it is ignored with a reason); raises ValueError only when `now_s`
        is not a number. See the module text for the rules.
        """
        now = _require_time(now_s)
        settings = self.settings
        self.stats.bump('offered')
        if kind not in C.TARGET_KINDS:
            return self._ignore('unknown_kind', f'unknown kind {_short(kind)}')
        try:
            lat, lon = require_lat_lon(lat_deg, lon_deg, 'target position')
        except ValueError as exc:
            return self._ignore('bad_position', str(exc))
        if not is_real_number(confidence) or not 0.0 <= confidence <= 1.0:
            return self._ignore('bad_confidence', f'confidence must be a number from 0 to 1, got {_short(confidence)}')
        conf = float(confidence)
        if conf <= 0.0 or conf < settings.min_confidence:
            return self._ignore('low_confidence', f'confidence {conf:.2f} is below the minimum')
        if self._sent_total >= settings.max_messages_total:
            self.stats.bump('capped')
            return Offer(OFFER_CAPPED, f'the limit of {settings.max_messages_total} messages is used up')

        target = self._nearest(kind, lat, lon)
        if target is None:
            if len(self._targets) >= settings.max_targets:
                return self._ignore('table_full', f'already {settings.max_targets} targets are known')
            self._counters[kind] += 1
            target = _Target(kind, self._counters[kind], lat, lon, conf, now)
            target.why = WHY_NEW
            self._targets.append(target)
            self.stats.bump('new')
            return Offer(OFFER_NEW, 'first sighting of this target', target.number)

        target.sightings += 1
        target.last_t = now
        if conf <= target.conf:
            self.stats.bump('unchanged')
            return Offer(OFFER_UNCHANGED, f'{percent(conf)}% is not better than the best sighting '
                         f'({percent(target.conf)}%)', target.number)
        target.lat, target.lon, target.conf = lat, lon, conf      # the better sighting becomes the best one
        return self._reevaluate(target, now)

    def _nearest(self, kind: str, lat: float, lon: float) -> Optional[_Target]:
        """The same-kind target closest to the point and within the radius, or None."""
        best: Optional[_Target] = None
        best_distance = 0.0
        for target in self._targets:
            if target.kind != kind:
                continue
            distance = haversine_m(lat, lon, target.lat, target.lon)
            if distance > self.settings.same_target_radius_m:
                continue
            if best is None or distance < best_distance or (distance == best_distance and target.number < best.number):
                best, best_distance = target, distance
        return best

    def _reevaluate(self, target: _Target, now: float) -> Offer:
        """After the best sighting changed: is a message wanted, and which kind? Sets or clears target.why."""
        settings = self.settings
        if target.rep is None:
            target.why = WHY_NEW           # the first message has not gone yet; it will carry the new best
            self.stats.bump('improved')
            return Offer(OFFER_IMPROVED, 'a better sighting replaces the report that is still waiting', target.number)
        rep_lat, rep_lon, rep_percent = target.rep
        gain = percent(target.conf) - rep_percent
        moved = haversine_m(target.lat, target.lon, rep_lat, rep_lon)
        if gain >= settings.min_gain_points:
            why = WHY_BETTER
        elif moved > settings.min_move_m:
            why = WHY_MOVED
        else:
            target.why = None              # a waiting report that is no longer worth sending is cancelled
            self.stats.bump('unchanged')
            return Offer(OFFER_UNCHANGED, f'+{gain} points and {moved:.1f} m: below the limits', target.number)
        if target.messages >= settings.max_messages_per_target:
            target.why = None
            self.stats.bump('capped')
            return Offer(OFFER_CAPPED, f'this target already has {target.messages} messages', target.number)
        if target.why is None:
            target.wait_since = now
        target.why = why
        self.stats.bump('improved')
        return Offer(OFFER_IMPROVED, f'{why}: +{gain} points, {moved:.1f} m from the last report', target.number)

    # ---------------------------------------------------------------------------------- sending
    def next_message(self, now_s) -> Optional[Outgoing]:
        """Hand out the next radio message if the rate limit allows, else None.

        Chooses the waiting report with the best rank (new, better, moved), oldest first. Only here a target
        counts as "reported" and the message counter moves on. Raises ValueError only for a bad `now_s`.
        """
        now = _require_time(now_s)
        settings = self.settings
        if self._sent_total >= settings.max_messages_total:
            for target in self._targets:
                target.why = None
            return None
        if self._last_sent_s is not None:
            gap = now - self._last_sent_s
            if gap < 0.0:                  # the clock went backwards: wait a full interval from this new time
                self._last_sent_s = now
                return None
            if gap < settings.min_interval_s:
                return None
        waiting = [t for t in self._targets if t.why is not None]
        if not waiting:
            return None
        order = {kind: index for index, kind in enumerate(C.TARGET_KINDS)}
        target = min(waiting, key=lambda t: (_WHY_RANK[t.why], t.wait_since, order[t.kind], t.number))
        why = target.why
        try:
            text = encode_tgt(target.kind, target.lat, target.lon, target.conf, self._seq)
            if len(text) > C.STATUSTEXT_MAX_CHARS or not text.isascii():
                raise ValueError('message too long or not ASCII')
        except ValueError:
            target.why = None              # cannot happen for validated input; never loop on it
            self.stats.bump('encode_errors')
            return None
        seq = self._seq
        target.rep = (target.lat, target.lon, percent(target.conf))
        target.messages += 1
        target.why = None
        self._seq = next_seq(self._seq)
        self._last_sent_s = now
        self._sent_total += 1
        self.stats.bump('sent')
        return Outgoing(text, target.kind, target.number, target.lat, target.lon, percent(target.conf), seq, why,
                        target.messages)
