"""
sim_demo/phases.py -- PURPOSE: work out which phase of the search mission the drone is in,
from the messages the real flight stack already sends, so the panel can light up
"Waiting > Taking off > Searching (lane 3 of 6) > Processing photos > Flying to target >
Dropping > Returning > Landed" and the scorer can time each phase.

Inputs (all optional; the tracker copes with any of them missing):
  * /mavros/state             armed flag and flight mode (AUTO, GUIDED, RTL, LAND ...)
  * /mavros/global_position/rel_alt   height above home
  * /mavros/mission/reached   which mission item was just reached
  * /mavros/statustext/recv   texts FROM the autopilot, e.g. "Mission: 2 DigiCamCtrl"
                              (ArduPilot prints "Mission: <n> <type>" for every item it starts)
  * /mavros/statustext/send   texts the team's nodes send TO the ground station, e.g.
                              main_controller "Reached last WP. Holding to finish image processing
                              before RTL", "Targets: tent@13 -> person@14. Diverting.",
                              "Over tent: loiter 6s then drop", "Servo 13 -> 1400", "RTL";
                              mapping trigger "Camera trigger STARTED".
  * the mission structure (mission_file.MissionSummary) for lane numbers.

This module has no ROS imports: feed it plain values and read `phase`, `detail`, `history`.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field

from .mission_file import MissionSummary

WAITING = 'Waiting'
TAKING_OFF = 'Taking off'
SEARCHING = 'Searching'
PROCESSING = 'Processing photos'
FLYING_TO_TARGET = 'Flying to target'
DROPPING = 'Dropping'
RETURNING = 'Returning'
LANDED = 'Landed'

PHASES = [WAITING, TAKING_OFF, SEARCHING, PROCESSING, FLYING_TO_TARGET, DROPPING, RETURNING,
          LANDED]

# Release servo channel -> payload class (main_controller.py: PERSON [11], TENT [13];
# the older two-servo wiring in capture_detect_drop.py used [9, 11] and [13, 14]).
DEFAULT_SERVO_CLASSES = {11: 'person', 13: 'tent', 9: 'person', 14: 'tent'}
OPEN_PWM_MIN = 1000     # a DO_SET_SERVO at or above this counts as "open" (team: 1400 open, 500 closed)

_RE_MISSION = re.compile(r'Mission:\s*(\d+)\s+(\S+)')
_RE_TARGETS = re.compile(r'Targets:\s*(.+?)\.?\s*Diverting', re.IGNORECASE)
_RE_TARGET_ITEM = re.compile(r'(person|tent)@(\d+)', re.IGNORECASE)
_RE_OVER = re.compile(r'Over\s+(person|tent)\b', re.IGNORECASE)
_RE_SERVO = re.compile(r'Servo\s+(\d+)\s*->\s*(\d+)', re.IGNORECASE)
_RE_RESUMED = re.compile(r'Resumed AUTO\s*->\s*WP\s*(\d+)', re.IGNORECASE)
_RE_CAPTURED = re.compile(r'Captured\s+\S+\s+image\s+#(\d+)', re.IGNORECASE)
_RE_ZOOM = re.compile(r'Zoom set to\s+([0-9.]+)x', re.IGNORECASE)


@dataclass
class PhaseEvent:
    """Something worth putting on the timeline.

    kind: 'phase' (data: phase, detail), 'drop' (data: cls, channel, pwm), 'release_closed',
    'over' (data: cls; main_controller arrived over that target and starts its loiter),
    'targets' (data: plan list of (cls, wp)), 'detected' (data: cls), 'photo' (data: number),
    'armed', 'disarmed', 'mode' (data: mode), 'zoom' (data: zoom).
    """

    t: float
    kind: str
    data: dict = field(default_factory=dict)


class PhaseTracker:
    """State machine: feed it messages with on_*() and read phase / detail / events."""

    def __init__(self, servo_classes: dict[int, str] | None = None):
        self.servo_classes = dict(DEFAULT_SERVO_CLASSES if servo_classes is None else servo_classes)
        self.phase = WAITING
        self.detail = 'waiting for the autopilot'
        self.events: list[PhaseEvent] = []
        self.summary: MissionSummary | None = None
        self.connected = False
        self.armed = False
        self.mode = ''
        self.rel_alt = 0.0
        self.max_rel_alt = 0.0
        self.last_reached = -1
        self.target_plan: list[tuple[str, int]] = []   # [(cls, wp_seq)] from main_controller
        self.current_target = ''
        self.drops: list[str] = []                     # classes released so far
        self.photos_captured = 0
        self.detected_classes: set[str] = set()
        self.zoom: float | None = None
        self.no_targets = False
        self._phase_since = 0.0
        self._flew = False
        # ArduPilot prints "Mission: N RTL" the moment it STARTS the mission's RTL item, which
        # happens right when the last search waypoint (or the last target) is reached - a
        # second BEFORE main_controller takes over (GUIDED). Only believe it if nobody takes
        # over within RTL_GRACE_S.
        self._pending_rtl_t: float | None = None

    RTL_GRACE_S = 4.0

    def _check_pending_rtl(self, t: float) -> None:
        if self._pending_rtl_t is None:
            return
        if self.mode != 'AUTO' or not self._flying():
            self._pending_rtl_t = None          # somebody took over (GUIDED, RTL mode ...)
        elif t - self._pending_rtl_t >= self.RTL_GRACE_S:
            self._pending_rtl_t = None
            if self.phase != RETURNING:
                self._set(t, RETURNING, 'flying home (mission RTL)')

    # ------------------------------------------------------------------ helpers
    @property
    def phase_index(self) -> int:
        """Position of the current phase in PHASES (0 = Waiting)."""
        return PHASES.index(self.phase)

    @property
    def history(self) -> list[PhaseEvent]:
        """Only the 'phase' events (phase changes), oldest first."""
        return [e for e in self.events if e.kind == 'phase']

    @property
    def phase_since(self) -> float:
        """Time [s] the current phase started (0.0 = still the initial Waiting phase)."""
        return self._phase_since

    def _set(self, t: float, phase: str, detail: str) -> None:
        if phase != self.phase:
            self.phase = phase
            self._phase_since = t
            self.detail = detail
            self.events.append(PhaseEvent(t, 'phase', {'phase': phase, 'detail': detail}))
        else:
            self.detail = detail

    def _lane_detail(self) -> str:
        s = self.summary
        if s and s.lane_count:
            if self.last_reached < s.survey_seqs[0]:
                return f'flying to the search area ({s.lane_count} lanes)'
            lane = s.lane_for_reached(self.last_reached)
            return f'lane {lane} of {s.lane_count}'
        if self.last_reached >= 0:
            return f'past waypoint {self.last_reached}'
        return 'flying the search pattern'

    def _flying(self) -> bool:
        return self.phase in (TAKING_OFF, SEARCHING, PROCESSING, FLYING_TO_TARGET, DROPPING,
                              RETURNING)

    # ------------------------------------------------------------------ inputs
    def set_mission(self, summary: MissionSummary | None) -> None:
        """Give the tracker the mission structure (lanes, takeoff and trigger items)."""
        self.summary = summary
        if self.phase == SEARCHING:
            self.detail = self._lane_detail()

    def on_state(self, t: float, connected: bool, armed: bool, mode: str) -> None:
        """Feed /mavros/state."""
        mode = (mode or '').upper()
        if armed and not self.armed:
            self.events.append(PhaseEvent(t, 'armed'))
        if self.armed and not armed:
            self.events.append(PhaseEvent(t, 'disarmed'))
        if mode != self.mode and mode:
            self.events.append(PhaseEvent(t, 'mode', {'mode': mode}))
        self.connected, self.mode = connected, mode
        was_armed, self.armed = self.armed, armed
        self._check_pending_rtl(t)
        if not connected and self.phase == WAITING:
            self.detail = 'waiting for the autopilot (MAVROS not connected)'
            return
        if armed:
            if self.phase in (WAITING, LANDED):
                if self.phase == LANDED:
                    self.drops, self.target_plan, self.no_targets = [], [], False
                self._set(t, TAKING_OFF, 'motors on - climbing')
            if mode == 'RTL' and self._flying() and self.phase != RETURNING:
                self._set(t, RETURNING, 'no targets found - flying home' if self.no_targets
                          else 'flying home (RTL)')
            elif mode == 'LAND' and self._flying():
                self._set(t, RETURNING, 'landing')
            elif mode == 'AUTO' and self.phase == DROPPING and self.target_plan:
                # main_controller switches back to AUTO after a drop to fly to the next target.
                remaining = [c for c, _ in self.target_plan if c not in self.drops]
                if remaining and self.current_target in self.drops:
                    self.current_target = remaining[0]
                    self._set(t, FLYING_TO_TARGET, f'to the {remaining[0]}')
        else:
            if was_armed and (self._flew or self.max_rel_alt > 2.0):
                self._set(t, LANDED, 'mission complete - motors off')
            elif self.phase == TAKING_OFF and not self._flew:
                self._set(t, WAITING, 'disarmed before take-off')
            elif self.phase == WAITING:
                self.detail = 'on the ground, motors off'

    def on_rel_alt(self, t: float, alt: float) -> None:
        """Feed /mavros/global_position/rel_alt (metres above home)."""
        self.rel_alt = alt
        self.max_rel_alt = max(self.max_rel_alt, alt)
        if alt > 2.0 and self.armed:
            self._flew = True
        self._check_pending_rtl(t)
        if self.phase == TAKING_OFF:
            target = self.summary.survey_alt if self.summary and self.summary.survey_alt else 0.0
            self.detail = f'climbing: {alt:.0f} m' + (f' of {target:.0f} m' if target else '')

    def on_reached(self, t: float, seq: int) -> None:
        """Feed /mavros/mission/reached (mavros_msgs/WaypointReached.wp_seq)."""
        self._check_pending_rtl(t)
        self.last_reached = seq
        s = self.summary
        if self.phase == TAKING_OFF and s is not None and seq >= max(s.takeoff_index, 1):
            self._set(t, SEARCHING, self._lane_detail())
        elif self.phase == SEARCHING:
            if s is not None and seq == s.last_nav_before_rtl:
                self.detail = 'end of the search pattern'
            else:
                self.detail = self._lane_detail()
        elif self.phase == FLYING_TO_TARGET:
            for cls, wp in self.target_plan:
                if wp == seq:
                    self.current_target = cls
                    self._set(t, DROPPING, f'over the {cls}: loiter, then release')

    def on_statustext(self, t: float, text: str) -> None:
        """Feed a STATUSTEXT from /mavros/statustext/recv or /mavros/statustext/send."""
        self._check_pending_rtl(t)
        txt = (text or '').strip()
        low = txt.lower()
        m = _RE_MISSION.search(txt)
        if m:
            kind = m.group(2).lower()
            if kind.startswith('takeoff') and self.phase in (WAITING, TAKING_OFF):
                self._set(t, TAKING_OFF, 'climbing')
            elif kind.startswith('digicam') and self.phase in (WAITING, TAKING_OFF, SEARCHING):
                self._set(t, SEARCHING, self._lane_detail())
            elif kind == 'wp' and self.phase == TAKING_OFF:
                self._set(t, SEARCHING, self._lane_detail())
            elif kind.startswith('rtl') and self._flying() and self.phase != RETURNING:
                self._pending_rtl_t = t        # see RTL_GRACE_S: main_controller may take over
            elif kind.startswith('land') and self._flying():
                self._set(t, RETURNING, 'landing')
            return
        if 'camera trigger started' in low and self.phase in (WAITING, TAKING_OFF, SEARCHING):
            self._set(t, SEARCHING, self._lane_detail())
            return
        if 'holding to finish image processing' in low or low.startswith('reached last wp'):
            self._set(t, PROCESSING, 'hovering until every photo is checked (max 90 s)')
            return
        if 'images processed' in low and 'selecting targets' in low:
            self._set(t, PROCESSING, 'all photos checked - choosing targets')
            return
        if 'processing timeout' in low:
            self._set(t, PROCESSING, 'detector too slow - choosing targets anyway')
            return
        if 'no valid targets' in low:
            self.no_targets = True
            self._set(t, RETURNING, 'no targets found - flying home')
            return
        m = _RE_TARGETS.search(txt)
        if m:
            self.target_plan = [(c.lower(), int(w)) for c, w in _RE_TARGET_ITEM.findall(m.group(1))]
            self.events.append(PhaseEvent(t, 'targets', {'plan': list(self.target_plan)}))
            if self.target_plan:
                self.current_target = self.target_plan[0][0]
                first = self.target_plan[0]
                self._set(t, FLYING_TO_TARGET, f'to the {first[0]} (WP {first[1]})')
            return
        m = _RE_RESUMED.search(txt)
        if m and self.phase in (PROCESSING, FLYING_TO_TARGET, DROPPING):
            wp = int(m.group(1))
            cls = next((c for c, w in self.target_plan if w == wp), self.current_target or 'target')
            self.current_target = cls
            self._set(t, FLYING_TO_TARGET, f'to the {cls} (WP {wp})')
            return
        m = _RE_OVER.search(txt)
        if m:
            self.current_target = m.group(1).lower()
            self.events.append(PhaseEvent(t, 'over', {'cls': self.current_target}))
            self._set(t, DROPPING, f'over the {self.current_target}: loiter, then release')
            return
        m = _RE_SERVO.search(txt)
        if m and 'failed' not in low:
            ch, pwm = int(m.group(1)), int(m.group(2))
            cls = self.servo_classes.get(ch, self.current_target or f'channel {ch}')
            if pwm >= OPEN_PWM_MIN:
                if cls not in self.drops:
                    self.drops.append(cls)
                self.events.append(PhaseEvent(t, 'drop', {'cls': cls, 'channel': ch, 'pwm': pwm}))
                remaining = [c for c, _ in self.target_plan if c not in self.drops]
                if self.phase == DROPPING and self.mode == 'AUTO' and remaining:
                    # main_controller sleeps inside its callbacks, so this text often arrives
                    # AFTER it already switched back to AUTO for the next target.
                    self.current_target = remaining[0]
                    self._set(t, FLYING_TO_TARGET, f'to the {remaining[0]} ({cls} payload released)')
                elif self.phase in (FLYING_TO_TARGET, PROCESSING, SEARCHING):
                    self._set(t, DROPPING, f'{cls} payload released')
                else:
                    self.detail = f'{cls} payload released'
            else:
                self.events.append(PhaseEvent(t, 'release_closed', {'cls': cls, 'channel': ch}))
            return
        if 'all targets done' in low:
            self.detail = 'all payloads released - RTL next'
            return
        if low == 'rtl' and self._flying():
            self._set(t, RETURNING, 'flying home (RTL)')
            return
        if low.startswith('detected '):
            cls = low.split(' ', 1)[1].strip()
            self.detected_classes.add(cls)
            self.events.append(PhaseEvent(t, 'detected', {'cls': cls}))
            return
        m = _RE_CAPTURED.search(txt)
        if m:
            self.photos_captured = max(self.photos_captured, int(m.group(1)))
            self.events.append(PhaseEvent(t, 'photo', {'number': int(m.group(1))}))
            return
        m = _RE_ZOOM.search(txt)
        if m:
            self.zoom = float(m.group(1))
            self.events.append(PhaseEvent(t, 'zoom', {'zoom': self.zoom}))
            return
        if 'land complete' in low and self.phase == RETURNING:
            self.detail = 'touched down'
