"""
sim_demo/run_csv.py -- PURPOSE: the column lists and the small writing helpers shared by every
CSV file of a run, so the live flight recorder (flight_recorder.py), the exporter for older runs
(csv_export.py) and the run summary (run_summary.py) all write EXACTLY the same columns.

Every run folder ~/.astra_sim/runs/<stamp>_<world>/ gets a csv/ folder:
    telemetry.csv          2-5 rows per second: where the drone is, how fast, mode, battery, GPS
    events.csv             every status text (autopilot and team texts in separate columns) and
                           every arm / disarm / mode change / waypoint reached / servo change
    phases.csv             when each mission phase started (TAKEOFF, SEARCH, HOLD, ...)
    photos.csv             one row per photo the camera saved
    detections.csv         one row per detection box (class, confidence, box, photo position)
    targets_and_drops.csv  one row per payload: aim point, release, landing point, errors
and ~/.astra_sim/runs/runs_index.csv gets one row per run.

Rules for the columns (so a spreadsheet or pandas can read them without guessing):
  * the unit is in the name: _m metres, _mps metres per second, _deg degrees, _s seconds,
    _v volts, _pct percent, _us microseconds (servo pulse width), _px pixels;
  * wall_time_utc is ISO 8601 in UTC with milliseconds ("2026-10-31T17:14:31.123Z"),
    unix_time_s is the same moment as seconds since 1970 (easy to subtract);
  * t_since_arm_s = seconds since the motors were armed (empty before arming);
  * an unknown value is an EMPTY cell (never 0, which would look like a real value);
  * yes/no columns hold 1 or 0.
Standard library only (csv, os, time): it must also run on a Windows PC without ROS.
"""
from __future__ import annotations

import csv
import math
import os
import time

CSV_DIR_NAME = 'csv'
INDEX_FILE_NAME = 'runs_index.csv'

TIME_COLUMNS = ['wall_time_utc', 'unix_time_s', 't_since_arm_s']

TELEMETRY_COLUMNS = TIME_COLUMNS + [
    'lat_deg', 'lon_deg', 'alt_msl_m', 'height_above_home_m', 'heading_deg',
    'groundspeed_mps', 'climb_rate_mps', 'roll_deg', 'pitch_deg', 'throttle_pct',
    'flight_mode', 'armed', 'landed_state', 'battery_v', 'battery_pct',
    'gps_fix_type', 'gps_fix', 'gps_satellites', 'current_wp', 'last_reached_wp',
    'mission_phase', 'position_age_s',
]

EVENTS_COLUMNS = TIME_COLUMNS + [
    'kind', 'src_system', 'src_component', 'sender', 'severity', 'severity_name',
    'autopilot_text', 'team_text', 'detail', 'mission_phase', 'source',
]

PHASES_COLUMNS = TIME_COLUMNS + ['phase_code', 'phase', 'detail', 'source']

# alt_msl_m = height above MEAN SEA LEVEL at the shutter time, from the drone's own MAVLink (telemetry.csv,
# interpolated); alt_wgs84_m = what the camera's sidecar calls "alt_msl_m": video_cam copies it from MAVROS
# /mavros/global_position/global, which is the height above the WGS-84 ELLIPSOID (MAVROS adds the geoid offset).
# On the Oklahoma field the two differ by about 29.6 m, so they get two different names here.
PHOTOS_COLUMNS = TIME_COLUMNS + [
    'photo', 'lat_deg', 'lon_deg', 'alt_msl_m', 'alt_wgs84_m', 'height_above_home_m', 'heading_deg',
    'zoom_x', 'gimbal_pitch_deg', 'camera_model', 'pose_how', 'checked_by_detector',
    'detections', 'best_class', 'best_confidence', 'footprint_en_m', 'source',
]

DETECTIONS_COLUMNS = TIME_COLUMNS + [
    'photo', 'class', 'confidence', 'box_cx_px', 'box_cy_px', 'box_w_px', 'box_h_px',
    'photo_lat_deg', 'photo_lon_deg', 'method', 'waypoint_index', 'false_alarm', 'source',
]

DROPS_COLUMNS = [
    'payload_class',
    'aim_wall_time_utc', 'aim_t_since_arm_s', 'aim_lat_deg', 'aim_lon_deg',
    'release_wall_time_utc', 'release_t_since_arm_s', 'release_source', 'servo_channel',
    'servo_open_pwm_us', 'release_drone_lat_deg', 'release_drone_lon_deg',
    'release_height_above_home_m',
    'landing_lat_deg', 'landing_lon_deg', 'landing_east_m', 'landing_north_m', 'landing_source',
    'truth_target', 'truth_lat_deg', 'truth_lon_deg', 'aim_error_m', 'drop_error_m', 'source',
]

RUNS_INDEX_COLUMNS = [
    'run', 'world', 'started_utc', 'duration_s', 'result', 'exit_code', 'profile', 'detector',
    'detector_model', 'detector_model_sha256', 'ardupilot_version', 'team_stack', 'git_commit',
    'git_dirty', 'targets_found', 'targets_total', 'tent_aim_error_m', 'tent_drop_error_m',
    'hiker_aim_error_m', 'hiker_drop_error_m', 'photos_taken', 'photos_checked',
    'warnings', 'errors', 'run_dir',
]

# Mission phase names of sim_demo/phases.py (PhaseTracker) -> the short codes that
# sim_bringup/phase_monitor.py prints and writes into phases.jsonl. Both write the same codes.
PHASE_CODES = {
    'Waiting': 'WAITING', 'Armed': 'ARMED', 'Taking off': 'TAKEOFF', 'Searching': 'SEARCH',
    'Processing photos': 'HOLD', 'Flying to target': 'TARGETS', 'Dropping': 'DROP',
    'Returning': 'RTL', 'Landed': 'LANDED',
}
PHASE_ORDER = ['WAITING', 'ARMED', 'TAKEOFF', 'SEARCH', 'HOLD', 'TARGETS', 'DROP', 'RTL', 'LANDED']


def utc_iso(t: float | None) -> str:
    """Seconds since 1970 -> '2026-10-31T17:14:31.123Z' ('' for None)."""
    if t is None:
        return ''
    try:
        t = float(t)
    except (TypeError, ValueError):
        return ''
    if not math.isfinite(t):
        return ''
    ms = int(round((t - math.floor(t)) * 1000.0))
    whole = math.floor(t)
    if ms >= 1000:                       # 0.9996 rounds up to the next second
        whole, ms = whole + 1, 0
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(whole)) + f'.{ms:03d}Z'


def num(value, digits: int = 3):
    """A number for a CSV cell: rounded float/int, or '' when unknown (None, NaN, text)."""
    if value is None:
        return ''
    if isinstance(value, (bool, int)):
        return int(value)                 # True -> 1, False -> 0, 7 -> 7
    try:
        f = float(value)
    except (TypeError, ValueError):
        return ''
    if not math.isfinite(f):
        return ''
    if digits <= 0:
        return int(round(f))
    return round(f, digits)


def since(t: float | None, t0: float | None, digits: int = 2):
    """Seconds from t0 to t ('' when either is unknown)."""
    if t is None or t0 is None:
        return ''
    return num(float(t) - float(t0), digits)


def time_cells(t: float | None, t_arm: float | None) -> dict:
    """The three time columns for one moment."""
    return {'wall_time_utc': utc_iso(t), 'unix_time_s': num(t, 3),
            't_since_arm_s': since(t, t_arm)}


def csv_dir(run_dir: str) -> str:
    """<run_dir>/csv (not created here)."""
    return os.path.join(run_dir, CSV_DIR_NAME)


def _fsync(f) -> None:
    """Ask the operating system to put the file's data on the disk NOW (survives a frozen WSL machine)."""
    try:
        os.fsync(f.fileno())
    except (OSError, ValueError, AttributeError):
        pass                              # a pipe, a closed file or a test double: flushing was enough


class LiveCsv:
    """A CSV file written row by row while the run happens.

    Every second (flush_every_s) the rows are handed to the operating system: they are then safe even if
    this program crashes a moment later. Every 5 seconds (fsync_every_s) they are also forced onto the
    disk, so even a frozen or killed WSL machine loses at most the last 5 seconds.
    """

    def __init__(self, path: str, columns: list[str], flush_every_s: float = 1.0,
                 clock=time.monotonic, fsync_every_s: float = 5.0):
        self.path = path
        self.columns = list(columns)
        self.rows = 0
        self._clock = clock
        self._flush_every = float(flush_every_s)
        self._fsync_every = float(fsync_every_s)
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        # newline='' is what the csv module wants (it writes its own \r\n line ends).
        self._f = open(path, 'w', encoding='utf-8', newline='')
        self._w = csv.DictWriter(self._f, fieldnames=self.columns, extrasaction='ignore')
        self._w.writeheader()
        self._f.flush()
        self._last_flush = self._last_fsync = self._clock()

    def write(self, row: dict) -> None:
        """Append one row (missing columns stay empty, unknown keys are ignored)."""
        if self._f is None:
            return
        self._w.writerow({k: ('' if row.get(k) is None else row.get(k)) for k in self.columns})
        self.rows += 1
        self.maybe_flush()

    def maybe_flush(self, force: bool = False) -> None:
        """Flush when a second has passed since the last flush (or always with force); fsync every 5 s."""
        if self._f is None:
            return
        now = self._clock()
        if force or now - self._last_flush >= self._flush_every:
            self._f.flush()
            self._last_flush = now
            if self._fsync_every > 0 and (force or now - self._last_fsync >= self._fsync_every):
                _fsync(self._f)
                self._last_fsync = now

    def close(self) -> None:
        """Flush, put on the disk and close (safe to call twice)."""
        if self._f is not None:
            try:
                self._f.flush()
                _fsync(self._f)
                self._f.close()
            finally:
                self._f = None


def write_csv_atomic(path: str, columns: list[str], rows: list[dict]) -> str:
    """Write a whole CSV file at once: into <path>.tmp first (put on the disk), then rename it over <path>.

    A reader (or a crash) never sees a half-written file. Returns the path.
    """
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(columns), extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: ('' if r.get(k) is None else r.get(k)) for k in columns})
        f.flush()
        _fsync(f)
    os.replace(tmp, path)
    return path


def read_csv(path: str) -> list[dict]:
    """All rows of a CSV file as dicts ([] when the file is missing or unreadable)."""
    try:
        with open(path, encoding='utf-8', newline='') as f:
            return list(csv.DictReader(f))
    except (OSError, csv.Error, UnicodeDecodeError):
        return []


def count_rows(path: str) -> int | None:
    """Number of data rows in a CSV file (None when it does not exist)."""
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding='utf-8', newline='') as f:
            return max(0, sum(1 for _ in csv.reader(f)) - 1)
    except (OSError, csv.Error, UnicodeDecodeError):
        return None


def to_float(value) -> float | None:
    """A CSV cell / JSON value -> float, or None when empty or not a number."""
    if value is None or value == '':
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None
