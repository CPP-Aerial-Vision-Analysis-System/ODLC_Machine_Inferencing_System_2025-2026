"""
sim_demo/flight_recorder.py -- PURPOSE: write the flight part of a run's CSV files LIVE, while
the drone flies, from nothing but the drone's own MAVLink messages.

It only LISTENS: it never sends a single byte to the autopilot, so it can never change the
flight. It reads its OWN copy of the MAVLink stream (in the simulator MAVProxy sends one to
UDP 127.0.0.1:14555 just for it; demo.sh adds that output), and every second it flushes:
    <run_dir>/csv/telemetry.csv          2 rows per second (--rate): position, height, heading,
                                         speeds, attitude, mode, armed, battery, GPS, waypoint
    <run_dir>/csv/events.csv             every STATUSTEXT (autopilot texts and team texts in
                                         separate columns, with severity and sender system /
                                         component) + arm, disarm, mode changes, waypoints
                                         reached, servo changes, MAVLink link lost / back
    <run_dir>/csv/phases.csv             when each mission phase started (sim_demo/phases.py
                                         worked out from the same messages)
    <run_dir>/csv/targets_and_drops.csv  per payload: where main_controller aims ("Aim ..."
                                         text) and when the release servo opened (SERVO_OUTPUT_RAW)
    <run_dir>/flight_recorder.json       what it heard (vehicle id, ArduPilot version, counts)
    <run_dir>/flight_recorder.pid        its process id while it runs (run_records.py stops it with
                                         this; logs.sh sees from it that the run is still going)
At the end of the run csv_export.py and run_summary.py add the photos, detections, landing
points and errors from the scorer. t_since_arm_s counts from the FIRST arming of the run (the
same zero as csv_export.py and the score report), also when the motors are armed again.

Because it only needs MAVLink it works the same way when the team's code runs on the real
Jetson (the PC then cannot see the Jetson's ROS topics) and against the real drone's telemetry
radio (a serial port, or a UDP copy from Mission Planner / MAVProxy):
    python3 -m sim_demo.flight_recorder --run-dir DIR                      (simulator, UDP 14555)
    python3 -m sim_demo.flight_recorder --run-dir DIR --url /dev/ttyUSB0 --baud 57600   (radio)
    ros2 run sim_demo flight_recorder --run-dir DIR --url udpin:0.0.0.0:14555
Stop it with Ctrl+C (or TERM): it closes the files cleanly. With --parent-pid it also stops
when that program (demo.sh) disappears, and then writes a partial run summary.

Exit codes: 0 stopped normally, 2 pymavlink is not installed, 3 the MAVLink connection could not
be opened (port busy / wrong address), 4 the run folder cannot be written, 5 bad options.
The core (FlightRecorder) is plain Python fed with message objects, unit-tested without pymavlink.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import sys
import time

from .phases import OPEN_PWM_MIN, PhaseTracker
from .run_csv import (DROPS_COLUMNS, EVENTS_COLUMNS, PHASE_CODES, PHASES_COLUMNS,
                      TELEMETRY_COLUMNS, LiveCsv, csv_dir, num, since, time_cells, utc_iso,
                      write_csv_atomic)
from .scoring import parse_aim_text

PROG = 'flight_recorder'
DEFAULT_PORT = 14555
DEFAULT_URL = f'udpin:127.0.0.1:{DEFAULT_PORT}'
EXIT_OK, EXIT_NO_PYMAVLINK, EXIT_NO_LINK, EXIT_NO_RUN_DIR, EXIT_BAD_ARGS = 0, 2, 3, 4, 5

MAV_TYPE_GCS = 6
MAV_AUTOPILOT_INVALID = 8
MAV_COMP_ID_AUTOPILOT1 = 1
MAV_COMP_ID_MISSIONPLANNER = 190
MAV_MODE_FLAG_SAFETY_ARMED = 128
UINT16_UNKNOWN = 65535

# ArduCopter flight modes (HEARTBEAT.custom_mode), ArduPilot ArduCopter/mode.h.
COPTER_MODES = {
    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 5: 'LOITER', 6: 'RTL',
    7: 'CIRCLE', 9: 'LAND', 11: 'DRIFT', 13: 'SPORT', 14: 'FLIP', 15: 'AUTOTUNE',
    16: 'POSHOLD', 17: 'BRAKE', 18: 'THROW', 19: 'AVOID_ADSB', 20: 'GUIDED_NOGPS',
    21: 'SMART_RTL', 22: 'FLOWHOLD', 23: 'FOLLOW', 24: 'ZIGZAG', 25: 'SYSTEMID',
    26: 'AUTOROTATE', 27: 'AUTO_RTL', 28: 'TURTLE',
}
# MAV_TYPE values flown by ArduCopter (quad, coaxial, helicopter, hexa, octo, tri, dodeca, deca).
COPTER_TYPES = {2, 3, 4, 13, 14, 15, 29, 35}
SEVERITY_NAMES = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 'WARNING', 'NOTICE', 'INFO',
                  'DEBUG']
GPS_FIX_NAMES = {0: 'no GPS', 1: 'no fix', 2: '2D fix', 3: '3D fix', 4: 'DGPS',
                 5: 'RTK float', 6: 'RTK fixed', 7: 'static', 8: 'PPP'}
LANDED_STATES = {1: 'on_ground', 2: 'in_air', 3: 'taking_off', 4: 'landing'}
# Release servo channel -> payload (main_controller.py: PERSON_SERVO_CHANNELS [11],
# TENT_SERVO_CHANNELS [13]). Change with --servo person:11,tent:13 when the wiring differs.
DEFAULT_SERVO_CLASSES = {11: 'person', 13: 'tent'}
SERVO_CHANGE_US = 50            # a servo output change smaller than this is noise, not an event
STATUSTEXT_CHUNK = 50           # MAVLink STATUSTEXT carries at most 50 characters per message
_RE_VERSION = re.compile(r'\bArdu(?:Copter|Plane|Rover|Sub)\s+V?(\d+\.\d+\.\d+[^\s(]*)')
_RE_SERVO_TEXT = re.compile(r'Servo\s+(\d+)\s*->\s*(\d+)', re.IGNORECASE)


def _stamp() -> str:
    """Local date and time at the start of every log line (flight_recorder.log is read after the run)."""
    return time.strftime('%Y-%m-%d %H:%M:%S')


def err(what: str, why: str = '', fix: str = '') -> None:
    """Print an error in the shared shape: WHAT failed, WHY (likely cause), HOW to fix it."""
    parts = [f'{_stamp()} [{PROG}] ERROR: {what}']
    if why:
        parts.append(f'Likely cause: {why}')
    if fix:
        parts.append(f'Fix: {fix}')
    print(' '.join(parts), file=sys.stderr, flush=True)


def warn(what: str, consequence: str = '', fix: str = '') -> None:
    """Print a warning: what happened, what it means for the run, how to fix it."""
    parts = [f'{_stamp()} [{PROG}] WARNING: {what}']
    if consequence:
        parts.append(f'Consequence: {consequence}')
    if fix:
        parts.append(f'Fix: {fix}')
    print(' '.join(parts), file=sys.stderr, flush=True)


def say(text: str) -> None:
    """A normal progress line (goes into flight_recorder.log)."""
    print(f'{_stamp()} [{PROG}] {text}', flush=True)


def mode_name(custom_mode: int, mav_type: int = 2) -> str:
    """HEARTBEAT custom_mode -> 'AUTO', 'GUIDED', ... (copters); 'MODE<n>' when unknown."""
    try:
        cm = int(custom_mode)
    except (TypeError, ValueError):
        return ''
    if mav_type in COPTER_TYPES and cm in COPTER_MODES:
        return COPTER_MODES[cm]
    return f'MODE{cm}'


def sender_of(src_system: int, src_component: int, vehicle_sys: int | None) -> str:
    """Who sent a STATUSTEXT: 'autopilot', 'gcs' (a ground station) or 'team' (the onboard
    computer: MAVROS forwarding the team nodes' /mavros/statustext/send)."""
    if src_component == MAV_COMP_ID_AUTOPILOT1 and (vehicle_sys is None or src_system == vehicle_sys):
        return 'autopilot'
    if src_system == 255 or src_component == MAV_COMP_ID_MISSIONPLANNER:
        return 'gcs'
    return 'team'


def parse_servo_classes(text: str) -> dict[int, str]:
    """'person:11,tent:13' -> {11: 'person', 13: 'tent'} (ValueError on bad input)."""
    out: dict[int, str] = {}
    for part in (text or '').split(','):
        part = part.strip()
        if not part:
            continue
        cls, _, ch = part.partition(':')
        if not cls.strip() or not ch.strip().isdigit():
            raise ValueError(f'"{part}" is not CLASS:CHANNEL (for example tent:13)')
        out[int(ch)] = cls.strip().lower()
    if not out:
        raise ValueError('no CLASS:CHANNEL pairs given')
    return out


def _text_of(msg, strip: bool = True) -> str:
    """The text of a STATUSTEXT (NUL padding removed; spaces at the ends removed unless strip=False)."""
    raw = getattr(msg, 'text', '')
    if isinstance(raw, (bytes, bytearray)):
        raw = bytes(raw).decode('utf-8', 'replace')
    text = str(raw).replace('\x00', '')
    return text.strip() if strip else text


def _deg(rad) -> float | None:
    try:
        return math.degrees(float(rad))
    except (TypeError, ValueError):
        return None


class FlightRecorder:
    """Turns MAVLink messages into the live CSV files (see the module docstring).

    Feed every received message to handle(msg, now) and call tick(now) often (at least a few
    times per second). `msg` needs get_type(), get_srcSystem(), get_srcComponent() and the
    MAVLink fields as attributes, exactly like a pymavlink message.
    """

    def __init__(self, run_dir: str, rate_hz: float = 2.0, servo_classes: dict | None = None,
                 open_pwm: int = OPEN_PWM_MIN, mission_summary=None, sysid: int | None = None,
                 link_timeout_s: float = 5.0, url: str = '', flush_every_s: float = 1.0,
                 meta_name: str = 'flight_recorder.json', quiet: bool = False):
        self.run_dir = run_dir
        self.dir = csv_dir(run_dir)
        # progress lines (flight_recorder.log); a .tlog replay is quiet: its lines would carry today's time
        self._say = (lambda _text: None) if quiet else say
        self._warn = (lambda *_a: None) if quiet else warn
        os.makedirs(self.dir, exist_ok=True)
        self.url = url
        self.period = 1.0 / max(0.2, min(float(rate_hz), 20.0))
        self.servo_classes = dict(DEFAULT_SERVO_CLASSES if servo_classes is None else servo_classes)
        self.open_pwm = int(open_pwm)
        self.link_timeout = float(link_timeout_s)
        self.telemetry = LiveCsv(os.path.join(self.dir, 'telemetry.csv'), TELEMETRY_COLUMNS,
                                 flush_every_s)
        self.events = LiveCsv(os.path.join(self.dir, 'events.csv'), EVENTS_COLUMNS, flush_every_s)
        self.phases = LiveCsv(os.path.join(self.dir, 'phases.csv'), PHASES_COLUMNS, flush_every_s)
        self.drops_path = os.path.join(self.dir, 'targets_and_drops.csv')
        self.meta_path = os.path.join(run_dir, meta_name)   # (a .tlog replay writes its own name)
        self.tracker = PhaseTracker(servo_classes=dict(self.servo_classes))
        if mission_summary is not None:
            self.tracker.set_mission(mission_summary)
        self._phase_seen = 0
        # vehicle state (None = not heard yet)
        self.vehicle_sys = sysid
        self.mav_type = 2
        self.armed = False
        self.mode = ''
        self.t_arm: float | None = None
        self.t_started: float | None = None
        self.last_hb: float | None = None
        self.first_msg: float | None = None
        self.link_ok = False
        self.pos: dict = {}
        self.pos_t: float | None = None
        self.hud: dict = {}
        self.att: dict = {}
        self.batt: dict = {}
        self.gps: dict = {}
        self.current_wp: int | None = None
        self.last_reached: int | None = None
        self.landed_state: int | None = None
        self.ardupilot_version = ''
        self.counts: dict[str, int] = {}
        self.servo_pwm: dict[int, int] = {}
        self.drops: dict[str, dict] = {}            # payload class -> targets_and_drops row
        self._chunks: dict[tuple, tuple[float, list[str]]] = {}
        self._next_row = 0.0
        self._next_meta = 0.0           # flight_recorder.json is refreshed every 10 s (fresh after a crash)
        self.closed = False

    # ------------------------------------------------------------------ helpers
    def _times(self, now: float) -> dict:
        return time_cells(now, self.t_arm)

    def _event(self, now: float, kind: str, detail: str = '', **cells) -> None:
        row = self._times(now)
        row.update(kind=kind, detail=detail, mission_phase=PHASE_CODES.get(self.tracker.phase, ''),
                   source='mavlink')
        row.update(cells)
        self.events.write(row)

    def _phase_row(self, now: float, code: str, name: str, detail: str) -> None:
        row = self._times(now)
        row.update(phase_code=code, phase=name, detail=detail, source='mavlink')
        self.phases.write(row)
        self._event(now, 'phase', f'{code}: {detail}')

    def _drain_phases(self, now: float) -> None:
        """Write the phase changes the tracker found since the last call."""
        evs = self.tracker.events
        for e in evs[self._phase_seen:]:
            if e.kind == 'phase':
                name = e.data.get('phase', '')
                self._phase_row(now, PHASE_CODES.get(name, name.upper()), name,
                                e.data.get('detail', ''))
        self._phase_seen = len(evs)

    def _is_vehicle(self, msg) -> bool:
        return (self.vehicle_sys is not None and msg.get_srcSystem() == self.vehicle_sys
                and msg.get_srcComponent() == MAV_COMP_ID_AUTOPILOT1)

    # ------------------------------------------------------------------ input
    def handle(self, msg, now: float) -> None:
        """Process one MAVLink message received at wall-clock time `now` [s since 1970]."""
        if self.closed or msg is None:
            return
        kind = msg.get_type()
        if kind == 'BAD_DATA':
            return
        if self.first_msg is None:
            self.first_msg = now
        self.counts[kind] = self.counts.get(kind, 0) + 1
        if kind == 'HEARTBEAT':
            self._heartbeat(msg, now)
            return
        if kind == 'STATUSTEXT':
            self._statustext(msg, now)
            return
        if not self._is_vehicle(msg):
            return
        handler = getattr(self, '_m_' + kind, None)
        if handler is not None:
            handler(msg, now)

    def _heartbeat(self, msg, now: float) -> None:
        if msg.get_srcComponent() != MAV_COMP_ID_AUTOPILOT1:
            return                          # MAVROS / ground stations send heartbeats too
        if getattr(msg, 'type', 0) == MAV_TYPE_GCS or getattr(msg, 'autopilot', 0) == MAV_AUTOPILOT_INVALID:
            return
        if self.vehicle_sys is None:
            self.vehicle_sys = msg.get_srcSystem()
            self._say(f'hearing the autopilot: system {self.vehicle_sys}, vehicle type {msg.type}')
            self._event(now, 'vehicle', f'autopilot found: system {self.vehicle_sys}, '
                        f'MAV_TYPE {msg.type}', src_system=self.vehicle_sys, src_component=1)
        if msg.get_srcSystem() != self.vehicle_sys:
            return
        self.mav_type = int(getattr(msg, 'type', 2))
        if not self.link_ok:
            if self.last_hb is not None:
                self._event(now, 'link_back', 'MAVLink from the autopilot is back')
                self._say('MAVLink from the autopilot is back')
            self.link_ok = True
        self.last_hb = now
        armed = bool(int(getattr(msg, 'base_mode', 0)) & MAV_MODE_FLAG_SAFETY_ARMED)
        mode = mode_name(getattr(msg, 'custom_mode', 0), self.mav_type)
        if mode != self.mode:
            if self.mode:
                self._event(now, 'mode', f'flight mode {self.mode} -> {mode}')
            else:
                self._event(now, 'mode', f'flight mode {mode}')
        if armed and not self.armed:
            if self.t_arm is None:      # the FIRST arming is the zero of t_since_arm_s (as in csv_export)
                self.t_arm = now
            self._event(now, 'armed', f'motors armed (mode {mode})')
            self._phase_row(now, 'ARMED', 'Armed', 'motors armed')
            self._say(f'armed in {mode}')
        elif self.armed and not armed:
            self._event(now, 'disarmed', f'motors disarmed (mode {mode})')
            self._say(f'disarmed (mode {mode})')
        self.armed, self.mode = armed, mode
        self.tracker.on_state(now, True, armed, mode)
        self._drain_phases(now)

    def _statustext(self, msg, now: float) -> None:
        text = _text_of(msg)
        sys_id, comp = msg.get_srcSystem(), msg.get_srcComponent()
        chunk_id = int(getattr(msg, 'id', 0) or 0)
        if chunk_id:
            # MAVLink 2 splits texts longer than 50 characters into numbered chunks. A chunk is FULL (more
            # follow) when its raw text uses all 50 characters, spaces included: measure before stripping.
            raw = _text_of(msg, strip=False)
            key = (sys_id, comp, chunk_id)
            t0, parts = self._chunks.get(key, (now, []))
            parts.append(raw)
            if len(raw) >= STATUSTEXT_CHUNK:
                self._chunks[key] = (t0, parts)
                return
            self._chunks.pop(key, None)
            text = ''.join(parts).strip()
        self._emit_text(now, sys_id, comp, int(getattr(msg, 'severity', 6)), text)

    def _emit_text(self, now: float, sys_id: int, comp: int, severity: int, text: str) -> None:
        if not text:
            return
        who = sender_of(sys_id, comp, self.vehicle_sys)
        sev_name = SEVERITY_NAMES[severity] if 0 <= severity < len(SEVERITY_NAMES) else ''
        cells = dict(src_system=sys_id, src_component=comp, sender=who, severity=severity,
                     severity_name=sev_name)
        if who == 'autopilot':
            cells['autopilot_text'] = text
        else:
            cells['team_text'] = text
        self._event(now, 'statustext', '', **cells)
        if who == 'autopilot' and not self.ardupilot_version:
            m = _RE_VERSION.search(text)
            if m:
                self.ardupilot_version = m.group(1)
                self._say(f'ArduPilot version: {self.ardupilot_version} ("{text}")')
                self._write_meta(now)
        aim = parse_aim_text(text)
        if aim is not None:
            cls, lat, lon = aim
            row = self._drop_row(cls)
            row.update(aim_wall_time_utc=utc_iso(now), aim_t_since_arm_s=since(now, self.t_arm),
                       aim_lat_deg=num(lat, 7), aim_lon_deg=num(lon, 7))
            self._write_drops()
        m = _RE_SERVO_TEXT.search(text)
        if m and 'failed' not in text.lower():
            ch, pwm = int(m.group(1)), int(m.group(2))
            if ch in self.servo_classes and pwm >= self.open_pwm:
                self._release(now, ch, pwm, 'team_text')
        self.tracker.on_statustext(now, text)
        self._drain_phases(now)

    def _m_GLOBAL_POSITION_INT(self, msg, now: float) -> None:
        hdg = int(getattr(msg, 'hdg', UINT16_UNKNOWN))
        self.pos = dict(lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt=msg.alt / 1000.0,
                        rel=msg.relative_alt / 1000.0,
                        hdg=None if hdg == UINT16_UNKNOWN else hdg / 100.0,
                        gs=math.hypot(msg.vx, msg.vy) / 100.0, climb=-msg.vz / 100.0)
        self.pos_t = now
        self.tracker.on_rel_alt(now, self.pos['rel'])
        self._drain_phases(now)

    def _m_VFR_HUD(self, msg, now: float) -> None:
        self.hud = dict(gs=float(msg.groundspeed), climb=float(msg.climb),
                        hdg=float(msg.heading), throttle=float(msg.throttle))

    def _m_ATTITUDE(self, msg, now: float) -> None:
        self.att = dict(roll=_deg(msg.roll), pitch=_deg(msg.pitch))

    def _m_SYS_STATUS(self, msg, now: float) -> None:
        mv = int(getattr(msg, 'voltage_battery', UINT16_UNKNOWN))
        pct = int(getattr(msg, 'battery_remaining', -1))
        self.batt = dict(v=None if mv in (UINT16_UNKNOWN, 0) else mv / 1000.0,
                         pct=None if pct < 0 else pct)

    def _m_GPS_RAW_INT(self, msg, now: float) -> None:
        sats = int(getattr(msg, 'satellites_visible', 255))
        self.gps = dict(fix=int(msg.fix_type), sats=None if sats == 255 else sats)

    def _m_MISSION_CURRENT(self, msg, now: float) -> None:
        seq = int(msg.seq)
        if seq != self.current_wp:
            self.current_wp = seq
            self._event(now, 'current_wp', f'current mission item {seq}')

    def _m_MISSION_ITEM_REACHED(self, msg, now: float) -> None:
        seq = int(msg.seq)
        if seq == self.last_reached:
            return                      # the autopilot repeats it a few times
        self.last_reached = seq
        self._event(now, 'wp_reached', f'reached mission item {seq}')
        self.tracker.on_reached(now, seq)
        self._drain_phases(now)

    def _m_EXTENDED_SYS_STATE(self, msg, now: float) -> None:
        self.landed_state = int(getattr(msg, 'landed_state', 0))

    def _m_SERVO_OUTPUT_RAW(self, msg, now: float) -> None:
        if int(getattr(msg, 'port', 0) or 0) != 0:
            return                      # port 1 = outputs 17-32
        for ch in sorted(self.servo_classes):
            raw = getattr(msg, f'servo{ch}_raw', None)
            if raw is None:
                continue
            raw = int(raw)
            last = self.servo_pwm.get(ch)
            self.servo_pwm[ch] = raw
            if last is None:
                continue                # the first sample is the starting value, not a change
            if abs(raw - last) < SERVO_CHANGE_US:
                continue
            cls = self.servo_classes[ch]
            self._event(now, 'servo', f'servo {ch} ({cls}): {last} -> {raw} us')
            if raw >= self.open_pwm > last:
                self._release(now, ch, raw, 'servo_output_raw')

    def _release(self, now: float, ch: int, pwm: int, how: str) -> None:
        """A release servo opened: remember when, where the drone was, and which channel."""
        cls = self.servo_classes.get(ch, f'channel {ch}')
        row = self._drop_row(cls)
        if row.get('release_wall_time_utc') and row.get('release_source') == 'servo_output_raw':
            return                      # already known from the autopilot's own servo output
        if row.get('release_wall_time_utc') and how == 'team_text':
            return                      # the team text arrives seconds late; keep the first time
        p = self.pos
        heard_text = row.get('release_source') == 'team_text'
        row.update(release_wall_time_utc=utc_iso(now), release_t_since_arm_s=since(now, self.t_arm),
                   release_source=how, servo_channel=ch, servo_open_pwm_us=pwm,
                   release_drone_lat_deg=num(p.get('lat'), 7), release_drone_lon_deg=num(p.get('lon'), 7),
                   release_height_above_home_m=num(p.get('rel'), 2))
        if heard_text:          # the team's "Servo N -> PWM" text came first; the autopilot's own output is exact
            self._say(f'{cls} release confirmed by the autopilot servo output (servo {ch} -> {pwm} us); that time is kept')
        else:
            self._say(f'{cls} payload released: servo {ch} -> {pwm} us ({how})')
        if how == 'team_text':
            self._event(now, 'release', f'{cls} payload: servo {ch} -> {pwm} us (team text)')
        self._write_drops()

    def _drop_row(self, cls: str) -> dict:
        return self.drops.setdefault(cls, {'payload_class': cls, 'source': 'flight_recorder'})

    def _write_drops(self) -> None:
        rows = [self.drops[c] for c in sorted(self.drops)]
        try:
            write_csv_atomic(self.drops_path, DROPS_COLUMNS, rows)
        except OSError as exc:
            self._warn(f'could not write {self.drops_path} ({exc})',
                 'the aim/release table is missing (telemetry and events keep recording)',
                 'check the disk space: df -h ~')

    def _write_meta(self, now: float | None = None) -> None:
        meta = dict(url=self.url, started_utc=utc_iso(self.t_started), stopped_utc='',
                    vehicle_system=self.vehicle_sys, ardupilot_version=self.ardupilot_version,
                    mav_type=self.mav_type, armed_utc=utc_iso(self.t_arm),
                    first_mavlink_utc=utc_iso(self.first_msg),
                    rows={'telemetry': self.telemetry.rows, 'events': self.events.rows,
                          'phases': self.phases.rows},
                    messages=dict(sorted(self.counts.items())),
                    servo_classes={str(k): v for k, v in self.servo_classes.items()},
                    rate_hz=round(1.0 / self.period, 2))
        if self.closed:
            meta['stopped_utc'] = utc_iso(now)
        try:
            tmp = self.meta_path + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(meta, f, indent=1)
            os.replace(tmp, self.meta_path)
        except OSError:
            pass                        # the CSV files matter more than this small summary

    # ------------------------------------------------------------------ clock
    def start(self, now: float) -> None:
        """Mark the start (writes the first event)."""
        self.t_started = now
        self._event(now, 'recorder_start', f'flight recorder listening on {self.url or "?"}')
        self._write_meta(now)

    def tick(self, now: float) -> None:
        """Call often: writes telemetry rows on time, watches the link, flushes the files."""
        if self.closed:
            return
        for key, (t0, parts) in list(self._chunks.items()):
            if now - t0 > 2.0:          # a chunked text whose last chunk never came
                self._chunks.pop(key, None)
                self._emit_text(now, key[0], key[1], 6, ''.join(parts).strip())
        if self.link_ok and self.last_hb is not None and now - self.last_hb > self.link_timeout:
            self.link_ok = False
            self._event(now, 'link_lost', f'no heartbeat from the autopilot for {self.link_timeout:.0f} s')
            self._warn(f'no heartbeat from the autopilot for {self.link_timeout:.0f} s',
                 'telemetry.csv pauses until MAVLink comes back (the flight itself is not affected)',
                 'if the simulator is still running, look for "MAVProxy outputs" in sim.log')
        if self.link_ok and now >= self._next_row:
            # Keep a steady beat (row times n x period apart); after a pause start a new beat from now.
            nxt = self._next_row + self.period
            self._next_row = nxt if nxt > now else now + self.period
            self.telemetry.write(self.telemetry_row(now))
        for f in (self.telemetry, self.events, self.phases):
            f.maybe_flush()
        if now >= self._next_meta:
            self._next_meta = now + 10.0
            self._write_meta(now)

    def telemetry_row(self, now: float) -> dict:
        """One telemetry.csv row from the newest value of every message."""
        p, h = self.pos, self.hud
        gs = h.get('gs') if h else p.get('gs')
        climb = h.get('climb') if h else p.get('climb')
        hdg = p.get('hdg') if p.get('hdg') is not None else h.get('hdg')
        fix = self.gps.get('fix')
        row = self._times(now)
        row.update(
            lat_deg=num(p.get('lat'), 7), lon_deg=num(p.get('lon'), 7),
            alt_msl_m=num(p.get('alt'), 2), height_above_home_m=num(p.get('rel'), 2),
            heading_deg=num(hdg, 1), groundspeed_mps=num(gs, 2), climb_rate_mps=num(climb, 2),
            roll_deg=num(self.att.get('roll'), 1), pitch_deg=num(self.att.get('pitch'), 1),
            throttle_pct=num(h.get('throttle'), 0), flight_mode=self.mode, armed=int(self.armed),
            landed_state=LANDED_STATES.get(self.landed_state, ''),
            battery_v=num(self.batt.get('v'), 2), battery_pct=num(self.batt.get('pct'), 0),
            gps_fix_type=num(fix, 0), gps_fix=GPS_FIX_NAMES.get(fix, ''),
            gps_satellites=num(self.gps.get('sats'), 0), current_wp=num(self.current_wp, 0),
            last_reached_wp=num(self.last_reached, 0),
            mission_phase=PHASE_CODES.get(self.tracker.phase, ''),
            position_age_s=since(now, self.pos_t))
        return row

    def close(self, now: float, why: str = '') -> None:
        """Write the last event, flush and close every file (safe to call twice)."""
        if self.closed:
            return
        self._event(now, 'recorder_stop', why or 'stopped')
        self.closed = True
        for f in (self.telemetry, self.events, self.phases):
            f.close()
        self._write_meta(now)


# ---------------------------------------------------------------------- program
def _mission_summary(path: str):
    """Lane structure of a .waypoints mission (for 'lane 2 of 3' in phases.csv), or None."""
    if not path:
        return None
    try:
        from .mission_file import analyze_items, load_waypoints_file
        return analyze_items(load_waypoints_file(path))
    except Exception as exc:  # noqa: BLE001 - lane numbers are a nicety
        warn(f'the mission file {path} could not be read ({exc})',
             'phases.csv shows no lane numbers (everything else is recorded)', '')
        return None


def _parent_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True                     # it exists, it is just not ours
    except OSError:
        return True
    return True


PID_FILE = 'flight_recorder.pid'


def _write_pid_file(run_dir: str, url: str) -> str:
    """<run_dir>/flight_recorder.pid: 'PID URL START' while this recorder runs ('' when it cannot be written)."""
    path = os.path.join(run_dir, PID_FILE)
    try:
        with open(path, 'w', encoding='utf-8') as f:
            f.write(f'{os.getpid()} {url} {utc_iso(time.time())}\n')
        return path
    except OSError:
        return ''


def _remove_pid_file(path: str) -> None:
    """Remove the pid file, but only if it is still ours (a newer recorder may have replaced it)."""
    if not path:
        return
    try:
        with open(path, encoding='utf-8') as f:
            ours = f.read().split()[:1] == [str(os.getpid())]
        if ours:                        # (removed after closing it: Windows cannot delete an open file)
            os.remove(path)
    except OSError:
        pass


def _why_parent_ended(run_dir: str) -> str:
    """The note for a partial summary when demo.sh disappeared: START OVER took over, or the window closed."""
    from .run_summary import read_env
    session = read_env(os.path.join(run_dir, 'demo.env')).get('SESSION', '')
    home = os.environ.get('ASTRA_SIM_HOME') or os.path.expanduser('~/.astra_sim')
    try:
        with open(os.path.join(home, 'state', 'keep_open'), encoding='utf-8') as f:
            keep_open = f.read().strip()
    except OSError:
        keep_open = ''
    if session and keep_open == session:
        return ('demo.sh handed this simulator to START OVER and ended (the flight continues in a new run '
                'folder); this summary was written by the flight recorder')
    return ('demo.sh ended without its normal finish (its window was closed, or it was killed); this '
            'partial summary was written by the flight recorder')


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog=PROG, description='Listen to MAVLink and write the live CSV files of one run '
        '(telemetry, events, phases, targets_and_drops). Never sends anything to the drone.')
    ap.add_argument('--run-dir', default=os.environ.get('ASTRA_RUN_DIR', ''),
                    help='the run folder (default $ASTRA_RUN_DIR); the files go into <run-dir>/csv')
    ap.add_argument('--url', default=DEFAULT_URL,
                    help=f'pymavlink connection: {DEFAULT_URL} (simulator), udpin:0.0.0.0:PORT, '
                         'tcp:HOST:PORT, or a serial port like /dev/ttyUSB0 (default %(default)s)')
    ap.add_argument('--baud', type=int, default=57600, help='serial speed for a radio (default 57600)')
    ap.add_argument('--rate', type=float, default=2.0, help='telemetry rows per second (default 2)')
    ap.add_argument('--servo', default='person:11,tent:13',
                    help='release servo channels as CLASS:CHANNEL,... (default %(default)s)')
    ap.add_argument('--open-pwm', type=int, default=OPEN_PWM_MIN,
                    help='a servo at or above this pulse width [us] counts as OPEN (default %(default)s)')
    ap.add_argument('--sysid', type=int, default=None, help='only this MAVLink system (default: the first autopilot heard)')
    ap.add_argument('--mission', default='', help='.waypoints file of the mission (lane numbers in phases.csv)')
    ap.add_argument('--parent-pid', type=int, default=0,
                    help='stop (and write a partial run summary) when this process ends (demo.sh passes its pid)')
    ap.add_argument('--no-mavlink-warn-s', type=float, default=90.0,
                    help='warn when nothing was heard for this long after the start (default 90)')
    ap.add_argument('--max-hours', type=float, default=8.0, help='stop by itself after this long (default 8)')
    return ap


def main(argv=None) -> int:
    """Entry point: `ros2 run sim_demo flight_recorder ...` / `python3 -m sim_demo.flight_recorder ...`."""
    args = build_parser().parse_args([a for a in (sys.argv[1:] if argv is None else argv)
                                      if not a.startswith('__')])     # ignore ROS's __node:= args
    if not args.run_dir:
        err('no run folder given.', 'neither --run-dir nor $ASTRA_RUN_DIR is set.',
            'python3 -m sim_demo.flight_recorder --run-dir ~/.astra_sim/runs/<run>')
        return EXIT_BAD_ARGS
    try:
        servo = parse_servo_classes(args.servo)
    except ValueError as exc:
        err(f'--servo {args.servo!r} cannot be used: {exc}.', 'a typo in the option.',
            'write it like --servo person:11,tent:13')
        return EXIT_BAD_ARGS
    run_dir = os.path.abspath(os.path.expanduser(args.run_dir))
    try:
        os.makedirs(csv_dir(run_dir), exist_ok=True)
    except OSError as exc:
        err(f'cannot create {csv_dir(run_dir)} ({exc}).', 'the disk is full or the folder is not yours.',
            'check with: df -h ~ ; ls -ld ' + run_dir)
        return EXIT_NO_RUN_DIR
    os.environ.setdefault('MAVLINK20', '1')     # parse MAVLink 2 (servo outputs 9-16 are MAVLink 2 fields)
    try:
        from pymavlink import mavutil
    except ImportError:
        err('the Python package pymavlink is not installed for ' + sys.executable + '.',
            'the simulator setup was not run with this Python (or a virtual environment is active).',
            'run setup/01_install_humble.sh again, or: /usr/bin/python3 -m pip install --user pymavlink')
        return EXIT_NO_PYMAVLINK
    try:
        conn = mavutil.mavlink_connection(args.url, baud=args.baud, autoreconnect=True)
    except Exception as exc:  # noqa: BLE001 - pymavlink raises many kinds for a bad address
        port = args.url.rsplit(':', 1)[-1]
        err(f'cannot open the MAVLink connection {args.url} ({exc}).',
            'another program (an older flight_recorder?) already listens on that port, or the '
            'address / serial port is wrong.',
            f'see who has it: ss -lunp | grep {port} ; stop it (pkill -f sim_demo.flight_recorder), '
            'or give another --url (and a matching MAVProxy output).')
        return EXIT_NO_LINK
    try:
        rec = FlightRecorder(run_dir, rate_hz=args.rate, servo_classes=servo, open_pwm=args.open_pwm,
                             mission_summary=_mission_summary(args.mission), sysid=args.sysid,
                             url=args.url)
    except OSError as exc:
        err(f'cannot create the CSV files in {csv_dir(run_dir)} ({exc}).',
            'the disk is full or the folder is not yours.', 'check with: df -h ~')
        return EXIT_NO_RUN_DIR
    stop = {'why': ''}

    def on_signal(signum, _frame):
        stop['why'] = f'stopped by signal {signal.Signals(signum).name}'

    # Background jobs of a script start with Ctrl+C IGNORED; set the handlers explicitly.
    for s in (signal.SIGINT, signal.SIGTERM, getattr(signal, 'SIGHUP', None)):
        if s is not None:
            signal.signal(s, on_signal)
    pid_file = _write_pid_file(run_dir, args.url)
    try:
        return _record(args, conn, rec, run_dir, stop)
    finally:
        _remove_pid_file(pid_file)


def _record(args, conn, rec: FlightRecorder, run_dir: str, stop: dict) -> int:
    """The listening loop of main(): until a signal, the parent's end or --max-hours."""
    t0 = time.time()
    rec.start(t0)
    say(f'listening (only) on {args.url}; writing {csv_dir(run_dir)} '
        f'({args.rate:g} telemetry rows/s, flushed every second)')
    warned_quiet = False
    next_parent_check = 0.0
    parent_gone = False
    while not stop['why']:
        try:
            msg = conn.recv_match(blocking=True, timeout=0.2)
        except (OSError, ValueError) as exc:     # a UDP "port unreachable" or a garbled packet
            msg = None
            if not stop['why']:
                time.sleep(0.2)
            _ = exc
        now = time.time()
        if msg is not None:
            try:
                rec.handle(msg, now)
            except Exception as exc:  # noqa: BLE001 - one odd message must never stop the recording
                warn(f'a {msg.get_type()} message could not be recorded ({exc})',
                     'that one message is missing from the CSV files', '')
        rec.tick(now)
        if not warned_quiet and rec.first_msg is None and now - t0 > args.no_mavlink_warn_s:
            warned_quiet = True
            warn(f'no MAVLink heard on {args.url} in {args.no_mavlink_warn_s:.0f} s',
                 'telemetry.csv, events.csv and phases.csv stay empty until it arrives',
                 'check that the simulator runs and that sim.log lists this port under '
                 '"MAVProxy outputs" (demo.sh adds it as extra_outs); on a radio check --url/--baud')
        if args.parent_pid and now >= next_parent_check:
            next_parent_check = now + 1.0
            if not _parent_alive(args.parent_pid):
                parent_gone = True
                stop['why'] = f'the program that started it (pid {args.parent_pid}) ended'
        if now - t0 > args.max_hours * 3600.0:
            stop['why'] = f'--max-hours {args.max_hours:g} reached'
    rec.close(time.time(), stop['why'])
    try:
        conn.close()
    except Exception:  # noqa: BLE001
        pass
    say(f'stopped ({stop["why"]}): {rec.telemetry.rows} telemetry rows, {rec.events.rows} events, '
        f'{rec.phases.rows} phase changes in {csv_dir(run_dir)}')
    if parent_gone:
        # demo.sh vanished without its normal ending (window closed, killed): leave a partial
        # summary so the run is never without one (it never replaces a complete one).
        try:
            from .run_summary import finish_run
            finish_run(run_dir, note=_why_parent_ended(run_dir), partial=True, live=True)
        except Exception as exc:  # noqa: BLE001
            warn(f'the partial run summary could not be written ({exc})',
                 'the run has CSV files but no run_summary.md',
                 f'write it by hand: python3 -m sim_demo.run_summary finish {run_dir}')
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
