"""
sim_demo/csv_export.py -- PURPOSE: turn the files of a finished run folder into the run's CSV files
(the same columns as the live flight recorder, see run_csv.py), for NEW runs and for OLD ones.

At the end of every demo.sh run it completes <run>/csv/ with what only the scorer knows:
    photos.csv             one row per photo (time, position, height above sea level from the drone's
                           MAVLink at the shutter time, the camera's own WGS-84 height, heading, zoom,
                           gimbal, how many detections, best class and confidence)
    detections.csv         one row per detection box (photo, class, confidence, box, photo position,
                           false alarm yes/no from the score report)
    targets_and_drops.csv  per payload: aim point, release (time, servo channel and pulse width, drone
                           position), landing point, the real target and the aim / drop errors
It keeps what the live flight recorder (flight_recorder.py) already wrote from MAVLink
(telemetry.csv, events.csv, phases.csv, the aim and release cells of targets_and_drops.csv). For an
OLDER run (before the recorder existed) it rebuilds those from the run's own files:
    events.csv   <- events.jsonl (score_run: autopilot and team texts, arm, disarm, waypoints, aims)
    phases.csv   <- phases.jsonl (phase_monitor)
    telemetry.csv <- a MAVLink log (.tlog) when you give one with --tlog (else only the header: the
                     run never recorded positions over time)
Rules for the recorded files (telemetry/events/phases.csv):
  * a file with rows from MAVLink (the live recorder, or an earlier --tlog replay) is KEPT;
  * a missing, empty or exporter-built events/phases.csv is (re)built from the .jsonl files every time;
  * --tlog replays the log into all three, but REFUSES (exit 1, says why) when telemetry.csv already has
    rows or when the log's times do not match the run; add --force to do it anyway;
  * --force without --tlog rebuilds events/phases.csv from the .jsonl files; telemetry.csv is never
    replaced by an empty file (without a .tlog there is nothing to rebuild it from).
The replay writes flight_recorder_tlog.json (the live recorder's flight_recorder.json is never touched).

How to run it (no ROS needed; it only reads files and writes <run>/csv/):
    ros2 run sim_demo export_csv ~/.astra_sim/runs/20261031-101431_runway
    python3 -m sim_demo.csv_export RUN_DIR [RUN_DIR ...] [--force] [--tlog FILE]
    py -3 ros2_ws\\src\\simulation\\sim_demo\\sim_demo\\csv_export.py \\\\wsl.localhost\\Ubuntu-22.04\\home\\<you>\\.astra_sim\\runs\\<run>
Exit codes: 0 = written, 1 = a run folder could not be exported or the request was refused (the reason
and the fix are printed), 2 = bad usage. Standard library only (csv, json); the .tlog replay needs pymavlink.
"""
from __future__ import annotations

import argparse
import bisect
import json
import os
import re
import shutil
import sys
import time

if __package__ in (None, ''):          # run as a plain file: python3 .../sim_demo/csv_export.py
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = 'sim_demo'           # noqa: A001 - lets the relative imports below work

from .run_csv import (DETECTIONS_COLUMNS, DROPS_COLUMNS, EVENTS_COLUMNS, PHASES_COLUMNS,  # noqa: E402
                      PHOTOS_COLUMNS, TELEMETRY_COLUMNS, count_rows, csv_dir, num, read_csv, since,
                      time_cells, to_float, utc_iso, write_csv_atomic)

PROG = 'export_csv'
EXIT_OK, EXIT_FAILED, EXIT_USAGE = 0, 1, 2
LIVE_FILES = ('telemetry.csv', 'events.csv', 'phases.csv')
TLOG_META = 'flight_recorder_tlog.json'   # what a .tlog replay heard (the live flight_recorder.json stays)
TLOG_MARGIN_S = 120.0                     # a .tlog must OVERLAP the run's own times (2 min for clocks/rounding);
                                          # runs here follow each other minutes apart, so more would let
                                          # the NEXT run's log in


class ExportRefused(Exception):
    """The exporter will not do what was asked, for a reason the person can fix: WHAT, WHY, FIX."""

    def __init__(self, what: str, why: str, fix: str):
        super().__init__(what)
        self.what, self.why, self.fix = what, why, fix
# Release servo channel -> payload (main_controller: PERSON_SERVO_CHANNELS [11], TENT_SERVO_CHANNELS [13]).
SERVO_CLASSES = {11: 'person', 13: 'tent'}
_RE_SERVO = re.compile(r'Servo\s+(\d+)\s*->\s*(\d+)', re.IGNORECASE)
_RE_AIM = re.compile(r'^\s*Aim\s+(person|tent)\s+([-+]?\d+(?:\.\d+)?)\s*,\s*([-+]?\d+(?:\.\d+)?)\s*$',
                     re.IGNORECASE)


def err(what: str, why: str = '', fix: str = '') -> None:
    """An error in the shared shape: WHAT failed. Likely cause: WHY. Fix: HOW."""
    parts = [f'[{PROG}] ERROR: {what}']
    if why:
        parts.append(f'Likely cause: {why}')
    if fix:
        parts.append(f'Fix: {fix}')
    print(' '.join(parts), file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------------------- reading
def load_json(path: str):
    """A JSON file's content, or None when it is missing or broken (a crashed run)."""
    try:
        with open(path, encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_jsonl(path: str) -> list[dict]:
    """Every JSON object of a .jsonl file (a half-written last line is skipped)."""
    out: list[dict] = []
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except ValueError:
                    continue
                if isinstance(obj, dict):
                    out.append(obj)
    except OSError:
        pass
    return out


def photo_latlon(name: str) -> tuple[float | None, float | None]:
    """'<lat> , <lon>.jpg' -> (lat, lon) (the team's photo name; (None, None) otherwise)."""
    stem = os.path.splitext(os.path.basename(name or ''))[0]
    if ' , ' not in stem:
        return None, None
    a, b = stem.split(' , ', 1)
    try:
        return float(a), float(b)
    except ValueError:
        return None, None


class RunFiles:
    """Everything the exporter reads from one run folder (each part may be missing)."""

    def __init__(self, run_dir: str):
        self.run_dir = run_dir
        self.run_data = load_json(os.path.join(run_dir, 'run_data.json')) or {}
        self.score = load_json(os.path.join(run_dir, 'score_report.json')) or {}
        self.events = load_jsonl(os.path.join(run_dir, 'events.jsonl'))
        self.phases = load_jsonl(os.path.join(run_dir, 'phases.jsonl'))
        self.recorder = load_json(os.path.join(run_dir, 'flight_recorder.json')) or {}
        self.csv = csv_dir(run_dir)

    def t_arm(self) -> float | None:
        """When the motors were armed [s since 1970] (live telemetry first, then score_run's files)."""
        rows = read_csv(os.path.join(self.csv, 'events.csv'))
        for r in rows:
            if r.get('kind') == 'armed':
                t = to_float(r.get('unix_time_s'))
                if t is not None:
                    return t
        for ev in self.events:
            if ev.get('kind') == 'armed':
                t = to_float(ev.get('t'))
                if t is not None:
                    return t
        return to_float(self.run_data.get('started')) if self.run_data else None

    def sidecar(self, photo: str) -> dict:
        """The camera's JSON next to a photo in <run>/photos ({} when there is none)."""
        stem = os.path.splitext(photo)[0]
        return load_json(os.path.join(self.run_dir, 'photos', stem + '.json')) or {}


# ---------------------------------------------------------------------------------------- events
def events_rows(rf: RunFiles, t_arm: float | None) -> list[dict]:
    """events.csv rows rebuilt from score_run's events.jsonl (older runs, or no live recorder)."""
    rows = []
    for ev in rf.events:
        t = to_float(ev.get('t'))
        kind = str(ev.get('kind', ''))
        row = time_cells(t, t_arm)
        row.update(source='events.jsonl', mission_phase=ev.get('phase', ''))
        if kind == 'statustext':
            text = str(ev.get('text', ''))
            if ev.get('source') == 'fcu':
                row.update(kind='statustext', sender='autopilot', autopilot_text=text)
            else:
                row.update(kind='statustext', sender='team', team_text=text)
        elif kind in ('armed', 'disarmed'):
            row.update(kind=kind, detail=f'motors {kind} (mode {ev.get("mode", "?")})')
        elif kind == 'reached':
            row.update(kind='wp_reached', detail=f'reached mission item {ev.get("seq")}')
        elif kind == 'aim':
            row.update(kind='aim', detail=f'main_controller aims the {ev.get("cls")} at '
                                          f'{ev.get("lat")}, {ev.get("lon")}')
        elif kind == 'start':
            row.update(kind='run_start', detail=f'score_run started recording (world {ev.get("world", "?")})')
        elif kind == 'report':
            row.update(kind='score_report', detail=f'score report written ({ev.get("reason", "")})')
        else:
            continue            # photos and detections have their own CSV files
        rows.append(row)
    return rows


def phases_rows(rf: RunFiles, t_arm: float | None) -> list[dict]:
    """phases.csv rows rebuilt from phase_monitor's phases.jsonl."""
    rows = []
    for ev in rf.phases:
        if ev.get('kind') != 'phase':
            continue
        t = to_float(ev.get('time'))
        row = time_cells(t, t_arm)
        code = str(ev.get('phase', ''))
        row.update(phase_code=code, phase=code.capitalize(), detail=ev.get('text', ''),
                   source='phases.jsonl')
        rows.append(row)
    return rows


# ---------------------------------------------------------------------------------------- photos
class Track:
    """csv/telemetry.csv as a time series: the value of a column at any moment, interpolated between the
    two rows around it (2 rows per second), so a photo gets the drone's height at its exact shutter time."""

    def __init__(self, rows: list[dict]):
        pts = sorted(((to_float(r.get('unix_time_s')), r) for r in rows), key=lambda p: p[0] or 0.0)
        pts = [p for p in pts if p[0] is not None]
        self.times = [p[0] for p in pts]
        self.rows = [p[1] for p in pts]

    def at(self, t: float | None, column: str, max_gap_s: float = 2.0) -> float | None:
        """The column's value at time t [s since 1970]; None when there is no row close enough."""
        if t is None or not self.times:
            return None
        i = bisect.bisect_left(self.times, t)
        left = next(((self.times[j], v) for j in range(i - 1, max(i - 4, -1), -1)
                     if (v := to_float(self.rows[j].get(column))) is not None), None)
        right = next(((self.times[j], v) for j in range(i, min(i + 3, len(self.times)))
                      if (v := to_float(self.rows[j].get(column))) is not None), None)
        if left and right and right[0] - left[0] <= max_gap_s:
            if right[0] == left[0]:
                return right[1]
            return left[1] + (right[1] - left[1]) * (t - left[0]) / (right[0] - left[0])
        near = min((p for p in (left, right) if p), key=lambda p: abs(p[0] - t), default=None)
        if near and abs(near[0] - t) <= max_gap_s / 2.0:
            return near[1]
        return None


def _frames(rf: RunFiles) -> list[dict]:
    """Every detector message of the run: run_data.json frames, else events.jsonl detections."""
    frames = rf.run_data.get('frames') if rf.run_data else None
    if frames:
        return list(frames)
    out = []
    for ev in rf.events:
        if ev.get('kind') != 'detection':
            continue
        out.append({'image_name': ev.get('image', ''), 'lat': ev.get('lat'), 'lon': ev.get('lon'),
                    'boxes': ev.get('boxes') or [], 'method': ev.get('method', ''),
                    'received': ev.get('t'), 'waypoint_index': None, 'source': 'events.jsonl'})
    return out


def _photos(rf: RunFiles) -> list[dict]:
    """Every photo: run_data.json photos, else events.jsonl photo events, else the photos/ folder."""
    if rf.run_data.get('photos'):
        return [dict(p, source='run_data.json') for p in rf.run_data['photos']]
    out = []
    for ev in rf.events:
        if ev.get('kind') == 'photo':
            out.append({'name': ev.get('name', ''), 't': ev.get('t'), 'lat': ev.get('lat'),
                        'lon': ev.get('lon'), 'pose_how': ev.get('pose', ''), 'zoom': ev.get('zoom'),
                        'footprint': ev.get('footprint'), 'source': 'events.jsonl'})
    if out:
        return out
    folder = os.path.join(rf.run_dir, 'photos')
    try:
        names = sorted(n for n in os.listdir(folder) if n.lower().endswith(('.jpg', '.jpeg', '.png')))
    except OSError:
        names = []
    for n in names:
        lat, lon = photo_latlon(n)
        try:
            mtime = os.path.getmtime(os.path.join(folder, n))
        except OSError:
            mtime = None
        out.append({'name': n, 't': mtime, 'lat': lat, 'lon': lon, 'source': 'photos folder'})
    return out


def photos_and_detections(rf: RunFiles, t_arm: float | None) -> tuple[list[dict], list[dict]]:
    """photos.csv rows and detections.csv rows."""
    frames = _frames(rf)
    false = {(fd.get('photo'), fd.get('class'), round(float(fd.get('confidence', 0.0)), 3))
             for fd in (rf.score.get('false_detections') or [])}
    scored = bool(rf.score)
    by_photo: dict[str, list[dict]] = {}
    checked: set[str] = set()
    det_rows = []
    for f in frames:
        name = f.get('image_name', '')
        checked.add(name)
        t = to_float(f.get('received'))
        lat, lon = to_float(f.get('lat')), to_float(f.get('lon'))
        has_gps = not (lat in (None, 0.0) and lon in (None, 0.0))
        for b in f.get('boxes') or []:
            conf = to_float(b.get('score'))
            row = time_cells(t, t_arm)
            row.update(photo=name, **{'class': b.get('cls', '')}, confidence=num(conf, 3),
                       box_cx_px=num(b.get('cx'), 1), box_cy_px=num(b.get('cy'), 1),
                       box_w_px=num(b.get('w'), 1), box_h_px=num(b.get('h'), 1),
                       photo_lat_deg=num(lat, 7) if has_gps else '',
                       photo_lon_deg=num(lon, 7) if has_gps else '',
                       method=f.get('method', ''), waypoint_index=num(f.get('waypoint_index'), 0),
                       source=f.get('source', 'run_data.json'))
            if scored:
                row['false_alarm'] = int((name, b.get('cls'), round(conf or 0.0, 3)) in false)
            det_rows.append(row)
            by_photo.setdefault(name, []).append(row)
    photo_rows = []
    # The drone's height above SEA LEVEL comes from its own MAVLink (telemetry.csv) at the shutter time. The
    # camera sidecar's "alt_msl_m" is NOT sea level: video_cam copies NavSatFix.altitude from MAVROS
    # /mavros/global_position/global, the height above the WGS-84 ellipsoid -> column alt_wgs84_m.
    track = Track(read_csv(os.path.join(rf.csv, 'telemetry.csv')))
    for p in sorted(_photos(rf), key=lambda p: to_float(p.get('t')) or 0.0):
        name = p.get('name', '')
        side = rf.sidecar(name)
        pos = side.get('position') or {}
        cam = side.get('camera') or {}
        gim = side.get('gimbal') or {}
        t = to_float(side.get('shutter_unix_s')) or to_float(p.get('t'))
        dets = by_photo.get(name, [])
        best = max(dets, key=lambda r: r['confidence'] or 0.0) if dets else None
        fp = p.get('footprint')
        rel = to_float(side.get('rel_alt_m'))
        row = time_cells(t, t_arm)
        row.update(photo=name,
                   lat_deg=num(to_float(pos.get('lat')) or to_float(p.get('lat')), 7),
                   lon_deg=num(to_float(pos.get('lon')) or to_float(p.get('lon')), 7),
                   alt_msl_m=num(track.at(t, 'alt_msl_m'), 2), alt_wgs84_m=num(pos.get('alt_msl_m'), 2),
                   height_above_home_m=num(rel if rel is not None else track.at(t, 'height_above_home_m'), 2),
                   heading_deg=num(side.get('heading_deg'), 1),
                   zoom_x=num(cam.get('zoom_x') if cam.get('zoom_x') is not None else p.get('zoom'), 2),
                   gimbal_pitch_deg=num(gim.get('pitch_deg'), 1), camera_model=cam.get('model', ''),
                   pose_how=p.get('pose_how', ''), checked_by_detector=int(name in checked),
                   detections=len(dets), best_class=best['class'] if best else '',
                   best_confidence=best['confidence'] if best else '',
                   footprint_en_m=(';'.join(f'{num(x, 2)},{num(y, 2)}' for x, y in fp) if fp else ''),
                   source=p.get('source', ''))
        photo_rows.append(row)
    return photo_rows, det_rows


# ---------------------------------------------------------------------------------------- drops
def origin_from_score(score: dict, world_name: str = '', worlds_dir: str = ''):
    """The world's GPS origin (to turn landing points in metres back into lat/lon), or None.

    First the world files (load_world); without them (e.g. on Windows) it is worked out from any
    entry of the score report that has both lat/lon AND east/north (the aims), with the same
    ArduPilot formula the scorer used, so the result is exact.
    """
    try:
        from .world_info import load_world
        if world_name:
            return load_world(world_name, worlds_dir).origin
    except Exception:  # noqa: BLE001 - no world files here: fall back to the score report
        pass
    from .geo import ARDUPILOT_M_PER_DEG, GeoOrigin, _ap_lon_scale
    for a in score.get('aims') or []:
        lat, lon, e, n = (to_float(a.get(k)) for k in ('lat', 'lon', 'east', 'north'))
        if None in (lat, lon, e, n):
            continue
        lat0 = lat - n / ARDUPILOT_M_PER_DEG
        lon0 = lon - e / ARDUPILOT_M_PER_DEG / _ap_lon_scale((lat0 + lat) / 2.0)
        return GeoOrigin(lat0, lon0)
    return None


def _texts(rf: RunFiles) -> list[tuple[float, str]]:
    """(time, text) of every status text of the run (live events.csv first, else events.jsonl)."""
    rows = read_csv(os.path.join(rf.csv, 'events.csv'))
    out = [(to_float(r.get('unix_time_s')) or 0.0, r.get('team_text') or r.get('autopilot_text') or '')
           for r in rows if r.get('kind') == 'statustext']
    if out:
        return out
    return [(to_float(ev.get('t')) or 0.0, str(ev.get('text', '')))
            for ev in rf.events if ev.get('kind') == 'statustext']


def drops_rows(rf: RunFiles, t_arm: float | None, origin=None) -> list[dict]:
    """targets_and_drops.csv rows: the live recorder's cells, completed with the scorer's numbers."""
    live = {r.get('payload_class', ''): dict(r)
            for r in read_csv(os.path.join(rf.csv, 'targets_and_drops.csv')) if r.get('payload_class')}
    rows: dict[str, dict] = {}

    def row_for(cls: str) -> dict:
        if cls not in rows:
            rows[cls] = live.get(cls) or {'payload_class': cls}
        return rows[cls]

    for cls in live:
        row_for(cls)
    # aim points: run_data.json "aims" (newest per class), else the "Aim ..." texts
    aims = list(rf.run_data.get('aims') or [])
    if not aims:
        for t, text in _texts(rf):
            m = _RE_AIM.match(text)
            if m:
                aims.append({'cls': m.group(1).lower(), 't': t, 'lat': float(m.group(2)),
                             'lon': float(m.group(3))})
    for a in sorted(aims, key=lambda a: to_float(a.get('t')) or 0.0):
        r = row_for(a.get('cls', '?'))
        if str(r.get('source', '')).startswith('flight_recorder') and r.get('aim_lat_deg'):
            continue            # the live recorder heard it itself
        t = to_float(a.get('t'))
        r.update(aim_wall_time_utc=utc_iso(t), aim_t_since_arm_s=since(t, t_arm),
                 aim_lat_deg=num(a.get('lat'), 7), aim_lon_deg=num(a.get('lon'), 7))
    # releases: score_run's drop records (time + drone position), servo channel from the team text
    servo_texts = []
    for t, text in _texts(rf):
        m = _RE_SERVO.search(text)
        if m and 'failed' not in text.lower():
            servo_texts.append((t, int(m.group(1)), int(m.group(2))))
    for d in rf.run_data.get('drops') or []:
        r = row_for(d.get('cls', '?'))
        if not r.get('release_wall_time_utc'):
            t = to_float(d.get('t'))
            r.update(release_wall_time_utc=utc_iso(t), release_t_since_arm_s=since(t, t_arm),
                     release_source='score_run')
            # the "Servo <ch> -> <pwm>" text of THIS payload's channel nearest in time
            near = sorted((s for s in servo_texts if t is not None and abs(s[0] - t) < 30.0 and s[2] >= 1000
                           and SERVO_CLASSES.get(s[1], '') == d.get('cls')), key=lambda s: abs(s[0] - t))
            if near:
                r.update(servo_channel=near[0][1], servo_open_pwm_us=near[0][2])
        if not r.get('release_drone_lat_deg'):
            r.update(release_drone_lat_deg=num(d.get('release_lat'), 7),
                     release_drone_lon_deg=num(d.get('release_lon'), 7))
    # landing points and errors: the score report
    targets = {t.get('name'): t for t in rf.score.get('targets') or []}
    for d in rf.score.get('drops') or []:
        r = row_for(d.get('class', '?'))
        e, n = to_float(d.get('east')), to_float(d.get('north'))
        r.update(landing_east_m=num(e, 2), landing_north_m=num(n, 2), landing_source=d.get('source', ''),
                 drop_error_m=num(d.get('error_m'), 2))
        if origin is not None and e is not None and n is not None:
            lat, lon, _ = origin.to_geo(e, n)
            r.update(landing_lat_deg=num(lat, 7), landing_lon_deg=num(lon, 7))
        if d.get('target'):
            r['truth_target'] = d['target']
    for a in rf.score.get('aims') or []:
        r = row_for(a.get('class', '?'))
        r['aim_error_m'] = num(a.get('error_m'), 2)
        if a.get('target') and not r.get('truth_target'):
            r['truth_target'] = a['target']
    for cls, r in rows.items():
        t = targets.get(r.get('truth_target'))
        if t is None:                   # no drop and no aim: the target of that class, if one
            same = [x for x in targets.values() if x.get('cls') == cls]
            t = same[0] if len(same) == 1 else None
            if t is not None:
                r['truth_target'] = t.get('name', '')
        if t is not None:
            r.update(truth_lat_deg=num(t.get('lat'), 7), truth_lon_deg=num(t.get('lon'), 7))
            if not r.get('aim_error_m'):
                r['aim_error_m'] = num(t.get('aim_error_m'), 2)
            if not r.get('drop_error_m'):
                r['drop_error_m'] = num(t.get('drop_error_m'), 2)
        if rf.score or rf.run_data:
            r['source'] = ('flight_recorder+score' if str(r.get('source', '')).startswith('flight_recorder')
                           else r.get('source') or 'score')
    return [rows[c] for c in sorted(rows)]


# ---------------------------------------------------------------------------------------- tlog
def replay_tlog_into(run_dir: str, tlog: str, meta_name: str = TLOG_META) -> dict:
    """Feed a MAVLink log (.tlog) through the live recorder: <run_dir>/csv/telemetry/events/phases.csv and
    <run_dir>/<meta_name>. Returns {'rows', 'messages', 'first', 'last'} (times in s since 1970, None if empty)."""
    os.environ.setdefault('MAVLINK20', '1')
    from pymavlink import mavutil

    from .flight_recorder import FlightRecorder
    log = mavutil.mavlink_connection(tlog, robust_parsing=True)
    rec = FlightRecorder(run_dir, url=f'tlog:{tlog}', flush_every_s=5.0, meta_name=meta_name, quiet=True)
    first = last = None
    n = 0
    t = 0.0
    while True:
        msg = log.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == 'BAD_DATA':
            continue
        t = float(getattr(msg, '_timestamp', 0.0) or 0.0)
        if first is None:
            rec.start(t)
            first = t
        last = t
        n += 1
        rec.handle(msg, t)
        rec.tick(t)
    rec.close(t, f'end of {os.path.basename(tlog)}')
    try:
        log.close()
    except Exception:  # noqa: BLE001 - closing a finished log file is not worth a message
        pass
    return {'rows': rec.telemetry.rows, 'messages': n, 'first': first, 'last': last}


def replay_tlog(run_dir: str, tlog: str) -> int:
    """replay_tlog_into() straight into run_dir (no checks). Returns the number of telemetry rows."""
    return replay_tlog_into(run_dir, tlog)['rows']


def run_time_span(rf: RunFiles) -> tuple[float | None, float | None, str]:
    """When the run happened: (first, last, how we know) in s since 1970, from its own files; else the time in
    the folder name (local time, so the check is only to the day); (None, None, '') when nothing tells."""
    ts = [v for src in (rf.run_data, rf.score) for k in ('started', 'ended') if (v := to_float(src.get(k)))]
    ts += [v for e in rf.phases if (v := to_float(e.get('time')))]
    ts += [v for e in rf.events if (v := to_float(e.get('t')))]
    if ts:
        return min(ts), max(ts), 'the run\'s own files'
    m = re.match(r'^(\d{8}-\d{6})', os.path.basename(os.path.normpath(rf.run_dir)))
    if m:
        try:
            t0 = time.mktime(time.strptime(m.group(1), '%Y%m%d-%H%M%S'))
            return t0 - 12 * 3600.0, t0 + 18 * 3600.0, 'the date in the folder name'
        except (ValueError, OverflowError):
            pass
    return None, None, ''


def _span_text(a: float | None, b: float | None) -> str:
    if a is None or b is None:
        return '?'
    return f'{time.strftime("%Y-%m-%d %H:%M", time.localtime(a))} to {time.strftime("%H:%M", time.localtime(b))}'


def _tlog_into_run(rf: RunFiles, tlog: str, force: bool) -> dict:
    """Replay a .tlog into rf's csv/ (via a temporary folder), after checking it belongs to this run."""
    if not os.path.isfile(tlog):
        raise ExportRefused(f'the MAVLink log {tlog} does not exist.', 'a typo in the path, or the file is on '
                            'another computer.', 'check it with  ls -l FILE  (Mission Planner keeps its logs in '
                            'Documents\\Mission Planner\\logs; MAVProxy writes mav.tlog in its state folder)')
    tmp = os.path.join(rf.csv, '.tlog_replay')
    shutil.rmtree(tmp, ignore_errors=True)
    try:
        info = replay_tlog_into(tmp, tlog)
        if not info['messages']:
            raise ExportRefused(f'{tlog} holds no MAVLink messages.', 'it is not a MAVLink log (.tlog), or it is '
                                'empty.', 'give the .tlog of this flight (Mission Planner: Ctrl+F > ... or its '
                                'logs folder; MAVProxy: mav.tlog)')
        a, b, how = run_time_span(rf)
        if a is not None and not force and (info['last'] < a - TLOG_MARGIN_S or info['first'] > b + TLOG_MARGIN_S):
            raise ExportRefused(
                f'{os.path.basename(tlog)} covers {_span_text(info["first"], info["last"])}, but this run '
                f'happened {_span_text(a, b)} (from {how}): it is not the log of this flight.',
                'another day\'s or another run\'s log was picked.',
                'give the .tlog of this flight; if you are sure it is the right one (e.g. a computer clock was '
                'wrong), add --force')
        if a is None:
            info['note'] = 'the run folder does not say when it ran, so the log could not be checked against it'
        for name in LIVE_FILES:
            os.replace(os.path.join(tmp, 'csv', name), os.path.join(rf.csv, name))
        # the replay's aim and release cells (release time from the autopilot's own servo output) too, unless the
        # live recorder already wrote them; the score's landing points and errors are added below as always
        drops_tmp, drops = os.path.join(tmp, 'csv', 'targets_and_drops.csv'), os.path.join(rf.csv, 'targets_and_drops.csv')
        live = [r for r in read_csv(drops) if str(r.get('source', '')).startswith('flight_recorder')]
        if os.path.isfile(drops_tmp) and (force or not live):
            os.replace(drops_tmp, drops)
        os.replace(os.path.join(tmp, TLOG_META), os.path.join(rf.run_dir, TLOG_META))
        return info
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _has_mavlink_rows(path: str) -> bool:
    """True when a CSV file holds rows written from MAVLink (the live recorder or a .tlog replay)."""
    return any(r.get('source') == 'mavlink' for r in read_csv(path))


# ---------------------------------------------------------------------------------------- main
def export_run(run_dir: str, force: bool = False, tlog: str = '', quiet: bool = False) -> dict:
    """Write the CSV files of one run folder (the rules are in the module docstring).

    Returns {'written': [...], 'kept': [...] (recorded files left as they are), 'notes': [...], 'counts': {...}}.

    Raises:
        FileNotFoundError: run_dir is not a folder.
        ExportRefused: --tlog would replace recorded rows (without force), or the .tlog is not of this run.
    """
    if not os.path.isdir(run_dir):
        raise FileNotFoundError(run_dir)
    out = {'written': [], 'kept': [], 'notes': []}
    rf = RunFiles(run_dir)
    os.makedirs(rf.csv, exist_ok=True)

    def path(name):
        return os.path.join(rf.csv, name)

    replayed = False
    if tlog:
        have = count_rows(path('telemetry.csv')) or 0
        if have and not force:
            raise ExportRefused(
                f'{path("telemetry.csv")} already has {have} rows (from the live flight recorder or an earlier '
                '--tlog replay); --tlog would replace them.', 'the run is already recorded.',
                'nothing to do if those rows are fine; to replace them with the replay, add --force')
        info = _tlog_into_run(rf, tlog, force)
        replayed = True
        out['written'] += list(LIVE_FILES)
        out['notes'].append(f'telemetry, events and phases replayed from {tlog} ({info["rows"]} telemetry rows, '
                            f'{info["messages"]} messages); what the replay heard: {TLOG_META}')
        if info.get('note'):
            out['notes'].append(info['note'])
    t_arm = rf.t_arm()
    for name, columns, builder in (('events.csv', EVENTS_COLUMNS, lambda: events_rows(rf, t_arm)),
                                   ('phases.csv', PHASES_COLUMNS, lambda: phases_rows(rf, t_arm))):
        if replayed:
            continue
        if not force and _has_mavlink_rows(path(name)):
            out['kept'].append(name)            # recorded from MAVLink: never rebuilt from the .jsonl files
            continue
        # missing, empty, or built earlier from the .jsonl files: build it again (they may have grown since)
        write_csv_atomic(path(name), columns, builder())
        out['written'].append(name)
    if not replayed:
        have = count_rows(path('telemetry.csv'))
        if have:
            out['kept'].append('telemetry.csv')
            if force:
                out['notes'].append(f'telemetry.csv kept ({have} rows): without --tlog there is nothing to '
                                    'rebuild it from')
        else:
            if have is None:
                write_csv_atomic(path('telemetry.csv'), TELEMETRY_COLUMNS, [])
                out['written'].append('telemetry.csv')
            out['notes'].append('telemetry.csv has only its header: this run has no recorded positions over '
                                'time (it ran before the flight recorder, or the recorder did not run). '
                                'Give a MAVLink log of the same flight with --tlog FILE to fill it.')
    photos, dets = photos_and_detections(rf, t_arm)
    write_csv_atomic(path('photos.csv'), PHOTOS_COLUMNS, photos)
    write_csv_atomic(path('detections.csv'), DETECTIONS_COLUMNS, dets)
    world = (rf.run_data.get('world') or rf.score.get('world') or '') if (rf.run_data or rf.score) else ''
    origin = origin_from_score(rf.score, world, rf.run_data.get('worlds_dir', '')) if rf.score else None
    write_csv_atomic(path('targets_and_drops.csv'), DROPS_COLUMNS, drops_rows(rf, t_arm, origin))
    out['written'] += ['photos.csv', 'detections.csv', 'targets_and_drops.csv']
    if not rf.run_data:
        out['notes'].append('no run_data.json (the scorer did not finish): photos and detections come '
                            'from events.jsonl or the photos folder')
    if not rf.score:
        out['notes'].append('no score_report.json: no landing points, errors or false-alarm flags')
    out['counts'] = {'photos': len(photos), 'detections': len(dets)}
    if not quiet:
        print(f'[{PROG}] {run_dir}: wrote {", ".join(out["written"])}'
              + (f'; kept the recorded {", ".join(out["kept"])}' if out['kept'] else '')
              + f' ({len(photos)} photos, {len(dets)} detections)')
        for n in out['notes']:
            print(f'[{PROG}]   note: {n}')
    return out


def build_parser() -> argparse.ArgumentParser:
    """Command-line options."""
    ap = argparse.ArgumentParser(prog=PROG, description='Write the CSV files (<run>/csv/) of one or more '
                                 'run folders from their run_data.json, events.jsonl and phases.jsonl.')
    ap.add_argument('run_dirs', nargs='+', metavar='RUN_DIR', help='run folder(s), e.g. ~/.astra_sim/runs/<run>')
    ap.add_argument('--force', action='store_true',
                    help='rebuild events/phases.csv from the .jsonl files even when the live recorder wrote them; '
                         'with --tlog: replace recorded rows and accept a log whose times do not match the run '
                         '(telemetry.csv is never replaced by an empty file)')
    ap.add_argument('--tlog', default='', help='a MAVLink log (.tlog) of the SAME flight: replayed through the '
                    'flight recorder into telemetry/events/phases.csv (refused when telemetry.csv already has rows)')
    ap.add_argument('--quiet', action='store_true', help='print errors only')
    return ap


def main(argv=None) -> int:
    """Entry point: `ros2 run sim_demo export_csv RUN_DIR` / `python3 -m sim_demo.csv_export RUN_DIR`."""
    args = build_parser().parse_args([a for a in (sys.argv[1:] if argv is None else argv)
                                      if not a.startswith('__')])
    if args.tlog and len(args.run_dirs) > 1:
        err('--tlog works with ONE run folder only.', 'a MAVLink log belongs to one flight.',
            'run the command once per run folder.')
        return EXIT_USAGE
    code = EXIT_OK
    for d in args.run_dirs:
        d = os.path.abspath(os.path.expanduser(d))
        try:
            export_run(d, force=args.force, tlog=args.tlog, quiet=args.quiet)
        except ExportRefused as exc:
            err(exc.what, exc.why, exc.fix)
            code = EXIT_FAILED
        except FileNotFoundError:
            err(f'the run folder {d} does not exist.', 'a typo in the name, or the run is on another computer.',
                'list the runs with:  bash simulation/scripts/logs.sh list')
            code = EXIT_FAILED
        except ImportError as exc:
            err(f'--tlog needs pymavlink ({exc}).', 'it is not installed for ' + sys.executable + '.',
                'python3 -m pip install --user pymavlink   (or run it inside the simulator\'s Ubuntu)')
            code = EXIT_FAILED
        except OSError as exc:
            err(f'cannot write the CSV files of {d} ({exc}).', 'the disk is full or the folder is not yours.',
                'check with:  df -h ~ ; ls -ld ' + d)
            code = EXIT_FAILED
    return code


if __name__ == '__main__':
    sys.exit(main())
