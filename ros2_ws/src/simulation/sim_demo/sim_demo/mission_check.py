#!/usr/bin/env python3
"""
sim_demo/mission_check.py -- PURPOSE: explain a mission file in plain words and check that the team's flight
code can fly it, BEFORE anyone uploads it (to the simulator or to the real Pixhawk).

It reads a mission made in Mission Planner (a ".waypoints" file, first line "QGC WPL 110") or in QGroundControl
(a ".plan" JSON file), and prints:
  * a table of every item: what it is, its altitude (metres and feet above home) and what the team code does
    there (lane start/end, where photos start = DO_DIGICAM_CONTROL, where photos stop = buffer_wp, the hold
    point where main_controller waits for the detector, the return-to-launch item),
  * the indices the team's waypoint_manager and main_controller will compute (exact copies of their rules,
    taken from sim_demo/mission_file.py, which copies waypoint_mavros/waypoint.py and main_controller.py),
  * the lane speed (DO_CHANGE_SPEED), the flight path length and a rough flight time,
  * ERRORS: things that stop the team code from doing its job, or that would crash the drone (no TAKEOFF, no
    DO_DIGICAM_CONTROL, no RETURN_TO_LAUNCH, a DO_ item where the hold point or buffer_wp must be, a search
    waypoint at ground level or below home, broken coordinates ...),
  * WARNINGS: things a person should look at (altitude outside the SUAS 2026 rule of 150-400 ft above the
    ground, altitudes not "above home", a waypoint far away from home, items after RTL ...).
Every problem says what is wrong, why it matters and how to fix it in Mission Planner.

It is pure Python (no ROS, no simulator, no pymavlink), so it also works on the ground laptop for the REAL
drone's mission files:
    python3 ros2_ws/src/simulation/sim_demo/sim_demo/mission_check.py my_mission.waypoints
    (Windows:  python ros2_ws\\src\\simulation\\sim_demo\\sim_demo\\mission_check.py my_mission.waypoints)
In the simulator it is used by simulation/scripts/mission.sh (check / load / fly) and installed as
`ros2 run sim_demo mission_check FILE`.

Exit codes (stable, used by mission.sh):
  0  the team code can fly it (warnings may still be printed)
  1  the mission has ERRORS (or WARNINGS with --strict): do not fly it as it is
  2  bad command line (unknown option, missing file name, bad --home)
  3  the file cannot be used: missing, unreadable, not a mission file, a broken line or a broken .plan structure
  4  the checker itself failed (a bug in this program, not in the mission): report it with the file
Every error of the program itself is printed to stderr in the shared shape
"[mission_check] ERROR: WHAT. Likely cause: WHY. Fix: HOW" (docs/simulation/ERRORS.md, section mission_check).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass, field

if __package__ in (None, ''):
    # Run as a plain file (python3 .../sim_demo/mission_check.py): make `import sim_demo` find this source tree.
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim_demo.mission_file import (MAV_CMD_DO_CHANGE_SPEED, MAV_CMD_DO_DIGICAM_CONTROL,  # noqa: E402
                                   MAV_CMD_DO_SET_SERVO, MAV_CMD_NAV_LAND, MAV_CMD_NAV_RETURN_TO_LAUNCH,
                                   MAV_CMD_NAV_SPLINE_WAYPOINT, MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_WAYPOINT,
                                   NAV_COMMANDS, MissionItem, MissionSummary, analyze_items,
                                   parse_waypoints_text)

PROG = 'mission_check'

# ---- exit codes (documented in the module docstring and docs/simulation/ERRORS.md) ------------------------------
EXIT_OK = 0
EXIT_PROBLEMS = 1
EXIT_USAGE = 2
EXIT_UNUSABLE = 3
EXIT_INTERNAL = 4

# ---- rules and constants ---------------------------------------------------------------------------------------
FT_PER_M = 1.0 / 0.3048
#: SUAS 2026 flight altitude rule: 150-400 ft above the ground (AGL) while flying the mission.
SUAS_MIN_FT = 150.0
SUAS_MAX_FT = 400.0
#: A flight item between TAKEOFF and RETURN_TO_LAUNCH at or below this height above home flies at ground level
#: (grass, fences, people): an ERROR, in the simulator and on the real drone. Negative heights are below home.
MIN_FLIGHT_HEIGHT_M = 5.0
#: Below this height (but above MIN_FLIGHT_HEIGHT_M) the search is too low even for a simulator test: trees, poles
#: and buildings are in the way and each photo covers only a few metres. From here up to 150 ft it is only the SUAS
#: rule that is broken ("fine for a simulator test").
LOW_TEST_HEIGHT_M = 20.0
#: main_controller.py flies the target waypoints it inserts at ALT = 49 m above home (about 161 ft).
TARGET_ALT_M = 49.0
#: main_controller.py waits at most this long at the hold point for the detector (PROCESSING_TIMEOUT_SECONDS).
HOLD_TIMEOUT_S = 90
#: ArduCopter's default WPNAV_SPEED (10 m/s), used for the time estimate when the mission sets no speed.
DEFAULT_SPEED_MPS = 10.0
#: A waypoint farther than this from home is probably for another field (or a typo in a coordinate).
FAR_FROM_HOME_KM = 5.0
#: The payload release servos main_controller drives (PERSON_SERVO_CHANNELS / TENT_SERVO_CHANNELS).
PAYLOAD_SERVO_CHANNELS = {11: 'person (hiker) payload', 13: 'tent payload'}
#: NAV commands that ArduPilot "reaches" (they produce MISSION_ITEM_REACHED, which the team code waits for).
REACHABLE_NAV = {16, 17, 18, 19, MAV_CMD_NAV_SPLINE_WAYPOINT}
#: Commands whose x/y are a latitude/longitude (subset of ArduPilot's AP_Mission::stored_in_location).
LOCATION_COMMANDS = {16, 17, 18, 19, 21, 22, 82}

FRAME_TEXT = {0: 'above sea level (frame 0)', 3: 'above home (frame 3)', 10: 'above terrain (frame 10)',
              5: 'above sea level, integer (frame 5)', 6: 'above home, integer (frame 6)',
              11: 'above terrain, integer (frame 11)', 2: 'mission frame (frame 2)'}
#: frames whose altitude means "above home" / "above terrain" / "above sea level"
REL_FRAMES, TERRAIN_FRAMES, MSL_FRAMES = {3, 6}, {10, 11}, {0, 5}

SPEED_TYPES = {0: 'airspeed', 1: 'ground speed', 2: 'climb speed', 3: 'descent speed'}


class MissionFormatError(ValueError):
    """The file is not a usable mission (wrong format, broken line, unsupported content)."""


@dataclass
class Report:
    """Everything the checker found out about one mission.

    Attributes:
        items: the mission items (item 0 = home), renumbered 0..N-1 in file order.
        source: the file name (for messages); kind: 'waypoints' (QGC WPL 110) or 'plan' (QGroundControl JSON).
        summary: the team-code indices (sim_demo.mission_file.MissionSummary).
        roles: one plain-words role per item ('' = none), e.g. 'lane 1 start'.
        heights_m: height above home of every NAV item with an altitude (None = not known / no altitude).
        errors / warnings / notes: plain-words findings (each says what, why, and the fix).
        speed_mps: lane speed from the first DO_CHANGE_SPEED (None = not set); path_m: flight path length.
    """

    items: list
    source: str = ''
    kind: str = 'waypoints'
    summary: MissionSummary = field(default_factory=MissionSummary)
    roles: list = field(default_factory=list)
    heights_m: list = field(default_factory=list)
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    notes: list = field(default_factory=list)
    speed_mps: float | None = None
    path_m: float = 0.0
    seq_renumbered: bool = False

    @property
    def ok(self) -> bool:
        """True when the team code can fly the mission (no errors)."""
        return not self.errors

    def to_dict(self) -> dict:
        """The report as plain data (for --json)."""
        s = self.summary
        if len(self.roles) != len(self.items):
            self.roles = item_roles(self)
        return {
            'source': self.source, 'kind': self.kind, 'count': len(self.items), 'ok': self.ok,
            'indices': {'takeoff_index': s.takeoff_index, 'digicam_index': s.digicam_index,
                        'buffer_wp': s.buffer_wp, 'last_before_rtl': s.last_before_rtl,
                        'last_nav_before_rtl': s.last_nav_before_rtl, 'rtl_index': s.rtl_index,
                        'num_waypoints': len(self.items), 'lanes': s.lane_count},
            'speed_mps': self.speed_mps, 'path_m': round(self.path_m, 1),
            'items': [{'seq': i, 'command': it.command, 'name': it.name, 'frame': it.frame,
                       'lat': it.lat, 'lon': it.lon, 'alt': it.alt,
                       'role': self.roles[i] if i < len(self.roles) else ''}
                      for i, it in enumerate(self.items)],
            'errors': self.errors, 'warnings': self.warnings, 'notes': self.notes,
        }


# =================================================================================================================
# Reading files
# =================================================================================================================

def _number(value, what: str) -> float:
    """A JSON number (QGroundControl writes NaN as null: that becomes 0)."""
    if value is None:
        return 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        raise MissionFormatError(f'{what} is not a number: {value!r}') from None


def _whole(value, what: str) -> int:
    """A JSON whole number such as a command id or a frame (NaN, infinity or a fraction cannot be one)."""
    v = _number(value, what)
    if not math.isfinite(v) or v != int(v):
        raise MissionFormatError(f'{what} is not a whole number: {value!r}')
    return int(v)


def _plan_simple_item(it, where: str) -> MissionItem:
    """One QGroundControl "SimpleItem" -> MissionItem (params[4..6] = lat, lon, alt)."""
    if not isinstance(it, dict):
        raise MissionFormatError(f'{where} is {it!r}, not a mission item (a JSON object with "command" and "params")')
    if 'command' not in it:
        raise MissionFormatError(f'{where} has no "command"')
    raw = it.get('params')
    if raw is None:
        raw = []
    if not isinstance(raw, list):
        raise MissionFormatError(f'{where} "params" is {raw!r}, not a list of 7 numbers')
    params = list(raw) + [0.0] * (7 - len(raw))
    p = [_number(v, f'{where} parameter {k + 1}') for k, v in enumerate(params[:7])]
    return MissionItem(seq=0, current=0, frame=_whole(it.get('frame', 3), f'{where} frame'),
                       command=_whole(it['command'], f'{where} command'), param1=p[0], param2=p[1],
                       param3=p[2], param4=p[3], lat=p[4], lon=p[5], alt=p[6],
                       autocontinue=1 if it.get('autoContinue', True) else 0)


def parse_plan_text(text: str) -> list:
    """Parse a QGroundControl .plan file (JSON) into MissionItems, item 0 = the planned home position.

    Simple items are copied as they are. A pattern ("ComplexItem": survey, corridor scan, structure scan) is
    expanded from the waypoints QGroundControl stored with it (TransectStyleComplexItem.Items); a pattern
    without them cannot be flown from the file and raises MissionFormatError with the fix.
    """
    try:
        doc = json.loads(text)
    except json.JSONDecodeError as exc:
        raise MissionFormatError(f'not a valid QGroundControl .plan file (the JSON is broken: {exc})') from None
    if not isinstance(doc, dict) or not isinstance(doc.get('mission'), dict):
        raise MissionFormatError('not a QGroundControl .plan file (no "mission" section)')
    mission = doc['mission']
    home = mission.get('plannedHomePosition') or [0.0, 0.0, 0.0]
    if not isinstance(home, list) or len(home) < 3:
        raise MissionFormatError('the .plan file has no usable "plannedHomePosition" [lat, lon, alt]')
    items = [MissionItem(seq=0, current=1, frame=0, command=MAV_CMD_NAV_WAYPOINT, param1=0.0, param2=0.0,
                         param3=0.0, param4=0.0, lat=_number(home[0], 'home latitude'),
                         lon=_number(home[1], 'home longitude'), alt=_number(home[2], 'home altitude'))]
    plan_items = mission.get('items') or []
    if not isinstance(plan_items, list):
        raise MissionFormatError('the .plan "mission" > "items" is not a list of mission items')
    for n, it in enumerate(plan_items, start=1):
        where = f'plan item {n}'
        kind = it.get('type') if isinstance(it, dict) else None
        if kind == 'SimpleItem':
            items.append(_plan_simple_item(it, where))
        elif kind == 'ComplexItem':
            pattern = it.get('complexItemType', 'pattern')
            transect = it.get('TransectStyleComplexItem') or {}
            if not isinstance(transect, dict):
                raise MissionFormatError(f'{where} ("{pattern}" pattern): "TransectStyleComplexItem" is not a JSON '
                                         'object')
            inner = transect.get('Items') or []
            if not isinstance(inner, list):
                raise MissionFormatError(f'{where} ("{pattern}" pattern): its stored waypoints ("Items") are not '
                                         'a list')
            if not inner:
                raise MissionFormatError(
                    f'{where} is a QGroundControl "{pattern}" pattern without its generated waypoints. Fix: open '
                    'the plan in QGroundControl 4.x and save it again (it then stores the waypoints), or draw the '
                    'lanes as simple waypoints, or make the mission in Mission Planner (.waypoints)')
            for k, sub in enumerate(inner, start=1):
                items.append(_plan_simple_item(sub, f'{where} ({pattern} waypoint {k})'))
        else:
            raise MissionFormatError(f'{where} has an unknown type {kind!r} (expected SimpleItem or ComplexItem)')
    for i, it in enumerate(items):
        it.seq = i
    return items


def load_mission(path: str) -> tuple:
    """Read a .waypoints (QGC WPL 110) or .plan (QGroundControl JSON) file.

    Returns (items, kind, renumbered) where kind is 'waypoints' or 'plan' and renumbered is True when the
    file's own sequence numbers were not 0, 1, 2 ... (the items are always renumbered in file order, which is
    what the autopilot receives). Raises OSError (cannot read) or MissionFormatError (not a usable mission:
    also for any structure the parsers did not expect, so a strange file never ends in a Python traceback).
    """
    with open(path, 'rb') as f:
        data = f.read()
    # utf-8-sig: Windows editors sometimes add a BOM; Notepad's "Unicode" saves UTF-16 (it starts with FF FE / FE FF).
    text = data.decode('utf-16') if data[:2] in (b'\xff\xfe', b'\xfe\xff') else data.decode('utf-8-sig')
    try:
        return _parse_mission_text(text)
    except MissionFormatError:
        raise
    except (TypeError, AttributeError, KeyError, IndexError, OverflowError, ValueError) as exc:
        raise MissionFormatError(f'the file has a structure this checker does not understand ({exc})') from None


def _parse_mission_text(text: str) -> tuple:
    """load_mission() without the file reading (see there)."""
    stripped = text.lstrip()
    if stripped.startswith('{'):
        return parse_plan_text(text), 'plan', False
    if not stripped:
        raise MissionFormatError('the file is empty')
    first = stripped.splitlines()[0].strip()
    if not first.upper().startswith('QGC WPL'):
        raise MissionFormatError(f'not a mission file: the first line is {first[:40]!r}, a Mission Planner file '
                                 'starts with "QGC WPL 110" and a QGroundControl file with "{"')
    if first.split()[-1] != '110':
        raise MissionFormatError(f'unsupported mission file version {first!r} (only "QGC WPL 110" is supported). '
                                 'Fix: save the mission again from Mission Planner')
    try:
        items = parse_waypoints_text(text)
    except ValueError as exc:
        raise MissionFormatError(str(exc)) from None
    if not items:
        raise MissionFormatError('the file has the "QGC WPL 110" header but no mission items')
    renumbered = any(it.seq != i for i, it in enumerate(items))
    for i, it in enumerate(items):
        it.seq = i
    return items, 'waypoints', renumbered


def write_waypoints(path: str, items: list) -> None:
    """Write items as a QGC WPL 110 file (the format Mission Planner and mission_tool read)."""
    lines = ['QGC WPL 110']
    for i, it in enumerate(items):
        lines.append('\t'.join([str(i), '1' if i == 0 else '0', str(it.frame), str(it.command)]
                               + [f'{v:.8f}' for v in (it.param1, it.param2, it.param3, it.param4, it.lat, it.lon)]
                               + [f'{it.alt:.6f}', str(int(it.autocontinue))]))
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines) + '\n')


# =================================================================================================================
# Checking
# =================================================================================================================

def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points (haversine)."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def _ft(m: float) -> str:
    return f'{m * FT_PER_M:.0f} ft'


def _has_position(it: MissionItem) -> bool:
    return it.command in LOCATION_COMMANDS and not (it.lat == 0.0 and it.lon == 0.0)


def height_above_home(it: MissionItem, home_alt_msl: float | None) -> float | None:
    """Height of a NAV item above home in metres (frame 3 / terrain frames as they are, MSL minus home)."""
    if it.command not in LOCATION_COMMANDS:
        return None
    if it.frame in REL_FRAMES or it.frame in TERRAIN_FRAMES:
        return it.alt
    if it.frame in MSL_FRAMES and home_alt_msl is not None:
        return it.alt - home_alt_msl
    return None


def check_mission(items: list, source: str = '', kind: str = 'waypoints', home: tuple | None = None,
                  renumbered: bool = False) -> Report:
    """Check a mission against what the team code and the SUAS 2026 rules need. Never raises for content.

    Args:
        items: MissionItems, item 0 = home (as Mission Planner writes it).
        source, kind: for messages ('waypoints' or 'plan').
        home: optional (lat, lon[, alt_msl]) of the place the drone will really fly from (the simulator's home):
            a mission whose waypoints are far from it is an ERROR (the drone would fly away).
        renumbered: the file's own sequence numbers were not 0, 1, 2 ... (a warning).
    Returns:
        Report.
    """
    rep = Report(items=items, source=source, kind=kind, seq_renumbered=renumbered)
    s = analyze_items(items)
    rep.summary = s
    n = len(items)
    err, warn, note = rep.errors.append, rep.warnings.append, rep.notes.append
    cmds = [it.command for it in items]

    # ---- home (item 0) ----------------------------------------------------------------------------------------
    home_item = items[0] if n else None
    home_is_home = bool(home_item and home_item.command == MAV_CMD_NAV_WAYPOINT and home_item.frame in MSL_FRAMES)
    home_alt = home_item.alt if home_is_home else None
    takeoff_at_0 = bool(n and items[0].command == MAV_CMD_NAV_TAKEOFF)
    if takeoff_at_0:
        err('item 0 is the TAKEOFF, but the autopilot treats item 0 as the HOME position and never flies it, so this '
            'mission has no TAKEOFF and AUTO cannot start. Likely cause: the file has no home line (written by hand '
            'or by another program). Fix: open it in Mission Planner and save it again (FLIGHT PLAN > Save WP File '
            'adds the home line as item 0), or add a home line "0 1 0 16 0 0 0 0 LAT LON ALT 1" as the first item')
    elif n and not home_is_home:
        warn(f'item 0 is {items[0].name} with frame {items[0].frame}, not the HOME line (WAYPOINT, frame 0). Why it '
             'matters: Mission Planner always writes home as item 0 and the team code counts it, so every index '
             'below would be off by one. Fix: save the mission from Mission Planner (Save WP File), which adds home')
    rep.heights_m = [height_above_home(it, home_alt) if i > 0 else None for i, it in enumerate(items)]

    # ---- broken numbers and coordinates ---------------------------------------------------------------------
    for i, it in enumerate(items):
        values = (it.param1, it.param2, it.param3, it.param4, it.lat, it.lon, it.alt)
        if any(math.isnan(v) or math.isinf(v) for v in values):
            err(f'item {i} ({it.name}) contains NaN or infinity. Why it matters: ArduPilot rejects or misreads '
                'it. Fix: re-enter that item in Mission Planner and save the file again')
            continue
        if it.command in LOCATION_COMMANDS and (abs(it.lat) > 90.0 or abs(it.lon) > 180.0):
            err(f'item {i} ({it.name}) has an impossible position {it.lat}, {it.lon}. Fix: correct the latitude '
                '(-90..90) and longitude (-180..180) in Mission Planner')
        elif (i > 0 and it.command in (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT)
              and it.lat == 0.0 and it.lon == 0.0):
            warn(f'item {i} ({it.name}) has latitude/longitude 0, 0. Why it matters: ArduCopter then stays where '
                 'it is for this item instead of flying somewhere; it is almost always a mistake. Fix: give the '
                 'waypoint its real position in Mission Planner')
        if i > 0 and it.autocontinue == 0:
            warn(f'item {i} ({it.name}) has autocontinue 0: the mission stops there and waits. Fix: set '
                 'autocontinue to 1 (Mission Planner does this by default)')
    if renumbered:
        warn('the sequence numbers in the file are not 0, 1, 2 ... in order. The tools upload the items in file '
             'order (renumbered), but Mission Planner may read the file differently. Fix: open the file in '
             'Mission Planner and save it again')

    # ---- take-off ----------------------------------------------------------------------------------------------
    # (a TAKEOFF at item 0 does not count: item 0 is home and is never flown; it has its own error above)
    takeoffs = [i for i, c in enumerate(cmds) if c == MAV_CMD_NAV_TAKEOFF and i > 0]
    if not takeoffs:
        if not takeoff_at_0:
            err('no TAKEOFF item. Why it matters: ArduCopter does not take off in AUTO without one, so the mission '
                'never starts. Fix: in Mission Planner add a TAKEOFF as the first item after home (right-click > '
                'Takeoff) with the take-off altitude')
    else:
        if len(takeoffs) > 1:
            warn(f'{len(takeoffs)} TAKEOFF items (at {takeoffs}); the team\'s waypoint_manager uses the LAST one '
                 f'({takeoffs[-1]}) for takeoff_index. Fix: keep only one TAKEOFF, right after home')
        if takeoffs[0] != 1:
            warn(f'the TAKEOFF is item {takeoffs[0]}, not item 1: the items before it run on the ground. Fix: move '
                 'TAKEOFF to be the first item after home')
        h = rep.heights_m[takeoffs[0]]
        if h is not None and h <= 0:
            err(f'the TAKEOFF altitude is {h:.1f} m: the drone would not climb. Fix: set the take-off altitude '
                '(for example the survey altitude) in Mission Planner')

    # ---- return to launch, hold point and buffer_wp ---------------------------------------------------------
    rtls = [i for i, c in enumerate(cmds) if c == MAV_CMD_NAV_RETURN_TO_LAUNCH]
    r = s.rtl_index
    if not rtls:
        err('no RETURN_TO_LAUNCH item. Why it matters: the team code finds everything from it (waypoint_manager\'s '
            'rtl_index stays -1): the photos never stop, main_controller never holds for the detector and cannot '
            'insert the target waypoints, so no payload is dropped. Fix: end the mission with RETURN_TO_LAUNCH '
            '(right-click > RTL in Mission Planner)')
    else:
        if len(rtls) > 1:
            err(f'{len(rtls)} RETURN_TO_LAUNCH items (at {rtls}). Why it matters: ArduPilot goes home at the FIRST '
                f'one, but the team code uses the LAST one ({rtls[-1]}), so the hold point and the photo stop are '
                'never reached. Fix: keep exactly one RETURN_TO_LAUNCH, as the last item')
        elif r != n - 1:
            after = ', '.join(f'{i} {items[i].name}' for i in range(r + 1, n))
            warn(f'RETURN_TO_LAUNCH (item {r}) is not the last item: the items after it ({after}) are never flown '
                 '(RTL lands the drone). Fix: delete them or move RETURN_TO_LAUNCH to the end')
        navs = [i for i in range(max(takeoffs or [0]) + 1, r) if cmds[i] in REACHABLE_NAV]
        if r < 3 or len(navs) < 2:
            err('the mission is too short: the team code needs TAKEOFF, the search waypoints, then two '
                'waypoints before RETURN_TO_LAUNCH (photos stop at the second-to-last one, the drone holds at the '
                'last one). Fix: add the search lanes and a hold waypoint before RTL in Mission Planner')
        else:
            hold, buf = items[r - 1], items[r - 2]
            if hold.command not in (NAV_COMMANDS - {20, 21, 22}):
                err(f'item {r - 1}, right before RETURN_TO_LAUNCH, is {hold.name}, not a WAYPOINT. Why it matters: '
                    'main_controller holds for the detector when ArduPilot REACHES that item and inserts the '
                    'target waypoints after it; DO_ items are never "reached". Fix: in Mission Planner put a '
                    'WAYPOINT (the hold point, e.g. over the middle of the search area) right before RTL')
            elif hold.command != MAV_CMD_NAV_WAYPOINT:
                warn(f'the hold point (item {r - 1}) is {hold.name}, not a plain WAYPOINT; the team mission uses a '
                     'WAYPOINT there (main_controller switches to GUIDED when it is reached anyway)')
            if buf.command not in REACHABLE_NAV:
                err(f'item {r - 2} (buffer_wp, two before RETURN_TO_LAUNCH) is {buf.name}, not a WAYPOINT. Why it '
                    'matters: the team\'s photo trigger stops when ArduPilot REACHES buffer_wp, and DO_ items are '
                    'never "reached", so the photos would never stop. Fix: make the item two before RTL the end '
                    'of the last search lane (a WAYPOINT); put DO_ items earlier')
        if s.last_nav_before_rtl >= 0 and s.last_before_rtl >= 0 and s.last_nav_before_rtl != s.last_before_rtl:
            note(f'main_controller would hold at item {s.last_nav_before_rtl} but insert targets after item '
                 f'{s.last_before_rtl}: they must be the same item (see the error about the item before RTL)')

    # ---- photos: DO_DIGICAM_CONTROL -------------------------------------------------------------------------------
    digicams = [i for i, c in enumerate(cmds) if c == MAV_CMD_DO_DIGICAM_CONTROL]
    if not digicams:
        err('no DO_DIGICAM_CONTROL item. Why it matters: ArduPilot sends "Mission: N DigiCamCtrl" when it gets '
            'there, and only that message starts the team\'s photo trigger (1 photo per second): without it there '
            'are no photos, no detections and no drops. Fix: in Mission Planner add a DO_DIGICAM_CONTROL right '
            'after the first search waypoint (the start of lane 1)')
    else:
        d = digicams[0]
        if len(digicams) > 1:
            warn(f'{len(digicams)} DO_DIGICAM_CONTROL items (at {digicams}): every "DigiCamCtrl" message (re)starts '
                 'the team\'s photo trigger. Fix: keep one, right after the first search waypoint')
        if takeoffs and d < takeoffs[0]:
            warn(f'DO_DIGICAM_CONTROL (item {d}) comes before the TAKEOFF: photos start on the ground. Fix: move it '
                 'right after the first search waypoint')
        if s.buffer_wp >= 0 and d >= s.buffer_wp:
            err(f'DO_DIGICAM_CONTROL (item {d}) comes at or after buffer_wp (item {s.buffer_wp}, where photos stop): '
                'the photos would stop before they start. Fix: move DO_DIGICAM_CONTROL to the start of lane 1')
        elif d >= 1 and items[d - 1].command == MAV_CMD_NAV_TAKEOFF:
            note(f'DO_DIGICAM_CONTROL (item {d}) comes right after the TAKEOFF: photos start while the drone '
                 'climbs and flies to the first waypoint (the team mission puts it after the lane-1 start '
                 'waypoint, so the camera set-up happens on the way into the search area)')

    # ---- servos: the payload channels belong to main_controller ---------------------------------------------
    for i, it in enumerate(items):
        if (it.command == MAV_CMD_DO_SET_SERVO and math.isfinite(it.param1)
                and int(it.param1) in PAYLOAD_SERVO_CHANNELS):
            warn(f'item {i} is DO_SET_SERVO on channel {int(it.param1)} ({PAYLOAD_SERVO_CHANNELS[int(it.param1)]}) '
                 f'to {it.param2:.0f} us: the mission itself would move a payload release at that point, before '
                 'main_controller has found a target. Fix: remove it; main_controller drops the payloads itself')
        if it.command == MAV_CMD_NAV_LAND:
            warn(f'item {i} is LAND: the team mission ends with RETURN_TO_LAUNCH (which lands at home), and '
                 'main_controller only works with RTL. Fix: replace LAND with RETURN_TO_LAUNCH')

    # ---- altitudes: frames and the SUAS rule ------------------------------------------------------------------
    first = (takeoffs[0] + 1) if takeoffs else 1
    last = (r - 1) if r > 0 else n - 1
    survey = [i for i in range(first, last + 1) if items[i].command in REACHABLE_NAV]
    odd_frames = sorted({items[i].frame for i in survey + takeoffs[:1] if items[i].frame not in REL_FRAMES})
    if odd_frames:
        names = ', '.join(FRAME_TEXT.get(f, f'frame {f}') for f in odd_frames)
        warn(f'some flight items use altitudes {names} instead of "above home" (frame 3, Mission Planner\'s '
             '"Relative"). The team code (and the target waypoints main_controller inserts, frame 3 at '
             f'{TARGET_ALT_M:.0f} m) thinks in heights above home; frame 0 depends on the field\'s height above sea '
             'level and frame 10 needs terrain data. Fix: set the altitude frame to Relative in Mission Planner')
    known = [(i, rep.heights_m[i]) for i in survey if rep.heights_m[i] is not None]
    # Ground level (or below home): the drone would fly the search in the grass. An ERROR, also in the simulator.
    ground = [(i, h) for i, h in known if h <= MIN_FLIGHT_HEIGHT_M]
    low = [(i, h) for i, h in known if h > MIN_FLIGHT_HEIGHT_M and h * FT_PER_M < SUAS_MIN_FT - 0.5]
    high = [(i, rep.heights_m[i]) for i in survey + takeoffs[:1] if rep.heights_m[i] is not None
            and rep.heights_m[i] * FT_PER_M > SUAS_MAX_FT + 0.5]
    band = f'{SUAS_MIN_FT * 0.3048:.1f}-{SUAS_MAX_FT * 0.3048:.1f} m'
    if ground:
        i, h = min(ground, key=lambda t: t[1])
        which = ', '.join(str(k) for k, _ in ground[:8]) + (' ...' if len(ground) > 8 else '')
        below = ' (a negative height is BELOW the home position)' if h < 0 else ''
        err(f'{len(ground)} flight item(s) between TAKEOFF and RETURN_TO_LAUNCH are at or below '
            f'{MIN_FLIGHT_HEIGHT_M:.0f} m above home (items {which}; lowest: item {i} at {h:.1f} m = {_ft(h)}){below}. '
            'Why it matters: the drone would fly the search at ground level or into the ground (grass, fences, '
            'people), in the simulator and on the real drone (for an altitude of exactly 0 ArduCopter may instead '
            'keep the height of the item before, which is just as wrong: the mission does not say how high to fly). '
            f'Fix: in Mission Planner set the Alt column of those items to the survey altitude ({band} = '
            f'{SUAS_MIN_FT:.0f}-{SUAS_MAX_FT:.0f} ft for SUAS 2026), altitude frame Relative, and save the file again')
    if low:
        i, h = min(low, key=lambda t: t[1])
        rule = (f'{len(low)} flight item(s) fly below the SUAS 2026 rule of {SUAS_MIN_FT:.0f}-{SUAS_MAX_FT:.0f} ft '
                f'above the ground (lowest: item {i} at {h:.1f} m = {_ft(h)}).')
        if h < LOW_TEST_HEIGHT_M:
            warn(f'{rule} That is too low even for a simulator test: trees, poles and buildings are in the way and '
                 'each photo covers only a few metres of ground. Fix: fly the search at '
                 f'{band} above the field (at least {LOW_TEST_HEIGHT_M:.0f} m for a quick test) and re-plan the lane '
                 'spacing for the new photo size')
        else:
            warn(f'{rule} Fine for a simulator test; at the competition fly at {band} above the field '
                 '(and re-plan the lane spacing for the new photo size)')
    if high:
        i, h = max(high, key=lambda t: t[1])
        warn(f'{len(high)} item(s) fly ABOVE {SUAS_MAX_FT:.0f} ft above the ground (highest: item {i} at {h:.1f} m '
             f'= {_ft(h)}): that breaks the SUAS 2026 rule and FAA Part 107. Fix: lower them to at most '
             f'{SUAS_MAX_FT * 0.3048:.1f} m')
    note(f'the target waypoints main_controller inserts fly at {TARGET_ALT_M:.0f} m ({_ft(TARGET_ALT_M)}) above home; '
         'RTL flies at the autopilot\'s RTL_ALT parameter (not part of the mission)')

    # ---- speed ---------------------------------------------------------------------------------------------------
    for i, it in enumerate(items):
        if it.command == MAV_CMD_DO_CHANGE_SPEED and it.param2 > 0:
            rep.speed_mps = it.param2
            if it.param2 > 15.0:
                warn(f'DO_CHANGE_SPEED (item {i}) sets {it.param2:g} m/s: with 1 photo per second the photos are '
                     f'{it.param2:.0f} m apart and may blur. Fix: fly the lanes at 4-8 m/s (the team mission: 5.5)')
            break
    if rep.speed_mps is None:
        note(f'no DO_CHANGE_SPEED: the lanes are flown at the autopilot\'s WPNAV_SPEED (ArduCopter default '
             f'{DEFAULT_SPEED_MPS:g} m/s = a photo every {DEFAULT_SPEED_MPS:g} m)')

    # ---- distances: path length and far-away waypoints ------------------------------------------------------
    ref = (home_item.lat, home_item.lon) if home_is_home and _has_position(home_item) else None
    points = [(it.lat, it.lon) for it in items[1:] if _has_position(it)]
    if ref:
        path = [ref] + points + ([ref] if rtls else [])
        rep.path_m = sum(distance_m(*a, *b) for a, b in zip(path, path[1:]))
        far = max((distance_m(*ref, *p) for p in points), default=0.0)
        if far > FAR_FROM_HOME_KM * 1000:
            warn(f'a waypoint is {far / 1000:.1f} km from the home position in the file. Is this mission for '
                 'another field, or is a coordinate mistyped? Fix: check the waypoints on the map in Mission Planner')
    if home is not None and points:
        far = max(distance_m(home[0], home[1], *p) for p in points)
        if far > FAR_FROM_HOME_KM * 1000:
            err(f'a waypoint is {far / 1000:.1f} km from where the drone starts ({home[0]:.6f}, {home[1]:.6f}): the '
                'drone would fly away. Why: the mission was made for another field. Fix: make the mission for this '
                'field (simulation/scripts/mission.sh make WORLD) or start the matching world')
    return rep


def item_roles(rep: Report) -> list:
    """A plain-words role for every item (what the team code does there)."""
    s, items = rep.summary, rep.items
    roles = [''] * len(items)
    for k, (a, b) in enumerate(s.lanes, start=1):
        if 0 <= a < len(items):
            roles[a] = f'lane {k} start'
        if 0 <= b < len(items) and b != a:
            roles[b] = f'lane {k} end'
    for i, it in enumerate(items):
        extra = []
        if i == 0 and it.command == MAV_CMD_NAV_WAYPOINT and it.frame in MSL_FRAMES:
            extra.append('HOME (ArduPilot replaces it with where the drone arms)')
        elif it.command == MAV_CMD_NAV_TAKEOFF:
            h = rep.heights_m[i] if i < len(rep.heights_m) else None
            extra.append('take off' + (f' and climb to {h:.0f} m' if h is not None else ''))
        elif it.command == MAV_CMD_DO_DIGICAM_CONTROL:
            extra.append(f'ArduPilot says "Mission: {i} DigiCamCtrl": the team trigger starts 1 photo per second')
        elif it.command == MAV_CMD_DO_CHANGE_SPEED and it.param2 > 0:
            kind = SPEED_TYPES.get(int(it.param1), 'speed') if math.isfinite(it.param1) else 'speed'
            extra.append(f'fly at {it.param2:g} m/s {kind} (a photo every {it.param2:.1f} m)')
        elif it.command == MAV_CMD_DO_SET_SERVO:
            extra.append(f'servo {it.param1:.0f} -> {it.param2:.0f} us')
        elif it.command == MAV_CMD_NAV_RETURN_TO_LAUNCH and i == s.rtl_index:
            extra.append('return home and land; main_controller inserts the target waypoints just before this')
        if i == s.buffer_wp and i > 0:
            extra.append('buffer_wp: photos STOP when this is reached')
        if i == s.last_before_rtl and i > 0:
            extra.append(f'HOLD: main_controller switches to GUIDED and waits (up to {HOLD_TIMEOUT_S} s) '
                         'for the detector')
        roles[i] = '; '.join(r for r in [roles[i]] + extra if r)
    return roles


# =================================================================================================================
# Printing
# =================================================================================================================

def format_report(rep: Report, quiet: bool = False) -> str:
    """The report as text for a person (see the module docstring)."""
    s, items = rep.summary, rep.items
    rep.roles = item_roles(rep)
    out = []
    kind = 'Mission Planner file (QGC WPL 110)' if rep.kind == 'waypoints' else 'QGroundControl plan (.plan JSON)'
    out.append(f'Mission {rep.source}: {kind}, {len(items)} items (item 0 = home)')
    if not quiet:
        out.append(f'  {"#":>3}  {"command":<19} {"height":<22} {"lat, lon":<24} what happens there')
        for i, it in enumerate(items):
            h = rep.heights_m[i] if i < len(rep.heights_m) else None
            if i == 0 and it.frame in MSL_FRAMES:
                height = f'{it.alt:.1f} m above sea'
            elif h is not None:
                height = f'{h:.1f} m ({_ft(h)})' + (' terrain' if it.frame in TERRAIN_FRAMES else '')
            else:
                height = '-'
            pos = f'{it.lat:.7f}, {it.lon:.7f}' if _has_position(it) else '-'
            out.append(f'  {i:>3}  {it.name:<19} {height:<22} {pos:<24} {rep.roles[i]}')
    out.append('')
    out.append('What the team code will compute from it (waypoint_manager / main_controller):')
    out.append(f'  takeoff_index {s.takeoff_index}, photos start at {s.digicam_index} (DO_DIGICAM_CONTROL), '
               f'buffer_wp {s.buffer_wp} (photos stop), hold {s.last_before_rtl} (last_before_rtl), '
               f'rtl_index {s.rtl_index}, num_waypoints {len(items)}')
    if s.rtl_index > 0:
        out.append(f'  after the search it inserts the targets at {s.rtl_index} (tent first) and {s.rtl_index + 1} '
                   f'(person); RETURN_TO_LAUNCH then moves to {s.rtl_index + 2}')
    alts = sorted(rep.heights_m[i] for i in s.survey_seqs if i < len(rep.heights_m) and rep.heights_m[i] is not None)
    line = f'  {s.lane_count} search lane(s)'
    if alts:
        mid = alts[len(alts) // 2]
        line += f' at {mid:.0f} m ({_ft(mid)}) above home'
    speed = rep.speed_mps or DEFAULT_SPEED_MPS
    line += (f', lane speed {rep.speed_mps:g} m/s' if rep.speed_mps
             else f', lane speed {DEFAULT_SPEED_MPS:g} m/s (default)')
    out.append(line)
    if rep.path_m > 0:
        out.append(f'  flight path about {rep.path_m / 1000:.2f} km: about {rep.path_m / speed / 60:.0f} min at '
                   f'{speed:g} m/s, plus up to {HOLD_TIMEOUT_S} s detector hold and ~15 s per payload drop')
    out.append('')
    for text in rep.errors:
        out.append(f'ERROR: {text}')
    for text in rep.warnings:
        out.append(f'WARNING: {text}')
    if not quiet:
        for text in rep.notes:
            out.append(f'note: {text}')
    if rep.errors or rep.warnings or (rep.notes and not quiet):
        out.append('')
    if rep.ok:
        out.append(f'RESULT: OK, the team code can fly this mission ({len(rep.warnings)} warning(s) to read)')
    else:
        out.append(f'RESULT: NOT READY, {len(rep.errors)} error(s): fix them before flying '
                   '(docs/simulation/ERRORS.md, "mission_check")')
    return '\n'.join(out)


def fail(what: str, why: str, fix: str, code: int) -> int:
    """Print one error in the shared shape (what, likely cause, fix) to stderr and return the exit code."""
    print(f'[{PROG}] ERROR: {what}. Likely cause: {why}. Fix: {fix}', file=sys.stderr, flush=True)
    return code


def parse_home(text: str) -> tuple:
    """'LAT,LON[,ALT[,HEADING]]' -> (lat, lon). Raises ValueError with a plain message."""
    parts = [p for p in text.replace(' ', '').split(',') if p]
    if len(parts) < 2:
        raise ValueError(f'"{text}" is not LAT,LON')
    lat, lon = float(parts[0]), float(parts[1])
    if abs(lat) > 90 or abs(lon) > 180:
        raise ValueError(f'"{text}" is not a position on Earth')
    return lat, lon


class _Parser(argparse.ArgumentParser):
    """argparse with command-line mistakes in the shared error shape (what, likely cause, fix) and exit code 2."""

    def error(self, message):
        # Name the command the way the person started it: `mission_check` (ros2 run / installed) or the .py file.
        argv0 = sys.argv[0] if sys.argv and sys.argv[0] else PROG
        cmd = f'python3 "{argv0}"' if argv0.endswith('.py') else PROG
        self.exit(EXIT_USAGE, f'[{PROG}] ERROR: {message}. Likely cause: a typo in the command, or the mission file '
                              f'name is missing. Fix: see the options and examples with  {cmd} --help   (the usual '
                              f'command is:  {cmd} my_mission.waypoints)\n')


def main(argv=None) -> int:
    """Command-line entry point (see the module docstring and --help)."""
    parser = _Parser(
        prog=PROG, formatter_class=argparse.RawDescriptionHelpFormatter,
        description='Explain a mission file (Mission Planner .waypoints or QGroundControl .plan) in plain words and '
                    'check that the team\'s flight code can fly it. Pure Python: works for real-drone missions too.',
        epilog='examples:\n'
               '  mission_check my_mission.waypoints\n'
               '  mission_check survey.plan --write-waypoints survey.waypoints   (convert for mission_tool)\n'
               '  mission_check field.waypoints --home 36.2143,-96.0068 --strict\n'
               'exit codes: 0 = OK (warnings allowed), 1 = errors (or warnings with --strict),\n'
               '            2 = bad command line, 3 = the file cannot be used (missing, not a mission, broken line),\n'
               '            4 = the checker itself failed (a bug: report it with the file)')
    parser.add_argument('file', help='the mission file (.waypoints = QGC WPL 110, or .plan = QGroundControl JSON)')
    parser.add_argument('--home', metavar='LAT,LON',
                        help='where the drone really starts (e.g. the simulator\'s home): far-away waypoints '
                             'become errors')
    parser.add_argument('--strict', action='store_true',
                        help='treat warnings as errors (exit 1), e.g. at the competition')
    parser.add_argument('--quiet', '-q', action='store_true',
                        help='no item table and no notes: only the indices and problems')
    parser.add_argument('--json', action='store_true', help='print the report as JSON instead of text')
    parser.add_argument('--write-waypoints', metavar='OUT',
                        help='also write the mission as a QGC WPL 110 file (converts a .plan for upload)')
    args = parser.parse_args(argv)

    home = None
    if args.home:
        try:
            home = parse_home(args.home)
        except ValueError as exc:
            return fail(f'--home {exc}', 'a typo in the position', 'give it as LAT,LON, e.g. --home 36.2143,-96.0068',
                        EXIT_USAGE)
    try:
        items, kind, renumbered = load_mission(args.file)
    except FileNotFoundError:
        return fail(f'the mission file "{args.file}" does not exist', 'a typo in the path, or the file is on '
                    'Windows and the path was not converted', 'check the name with  ls  (a Windows path like '
                    'C:\\Users\\me\\m.waypoints is /mnt/c/Users/me/m.waypoints inside WSL)', EXIT_UNUSABLE)
    except IsADirectoryError:
        return fail(f'"{args.file}" is a folder, not a mission file', 'the file name is missing',
                    'give the .waypoints / .plan file inside it', EXIT_UNUSABLE)
    except (OSError, UnicodeDecodeError) as exc:
        return fail(f'cannot read "{args.file}" ({exc})', 'no read permission, or it is not a text file',
                    'save the mission again from Mission Planner (Save WP File) and use that file', EXIT_UNUSABLE)
    except MissionFormatError as exc:
        return fail(f'"{args.file}" cannot be used as a mission: {exc}', 'it is not a Mission Planner / '
                    'QGroundControl mission file, or it was edited by hand',
                    'save the mission from Mission Planner (FLIGHT PLAN > Save WP File) and use that file',
                    EXIT_UNUSABLE)
    try:
        rep = check_mission(items, source=args.file, kind=kind, home=home, renumbered=renumbered)
        text = format_report(rep, quiet=args.quiet)
        report_json = json.dumps(rep.to_dict(), indent=2) if args.json else ''
    except Exception as exc:    # noqa: BLE001 -- never a bare traceback: exit 4 must not look like "errors found"
        return fail(f'the checker itself failed on "{args.file}" ({type(exc).__name__}: {exc})',
                    'a bug in mission_check.py for an unusual mission (the mission may be fine)',
                    'report it with the mission file; meanwhile open the file in Mission Planner and check it by eye',
                    EXIT_INTERNAL)
    print(report_json if args.json else text)
    if args.write_waypoints:
        try:
            write_waypoints(args.write_waypoints, items)
        except OSError as exc:
            return fail(f'cannot write {args.write_waypoints} ({exc})', 'the folder does not exist or is read-only',
                        'give another path with --write-waypoints', EXIT_UNUSABLE)
        if not args.json:
            print(f'wrote {args.write_waypoints} (QGC WPL 110, {len(items)} items)')
    if rep.errors or (args.strict and rep.warnings):
        if args.strict and rep.warnings and not rep.errors and not args.json:
            print('RESULT with --strict: the warnings count as errors (exit 1)')
        return EXIT_PROBLEMS
    return EXIT_OK


if __name__ == '__main__':
    sys.exit(main())
