"""
sim_worlds/mission.py -- PURPOSE: write and read ArduPilot mission files in the "QGC WPL 110" text format (what
MAVProxy `wp load` and Mission Planner read), build the search mission in EXACTLY the shape last year's flight
software expects, and check a mission against that shape.

The shape (read from the team code, see sim_worlds/README.md "Mission shape"):

  seq 0  HOME                    MAV_CMD_NAV_WAYPOINT, frame 0 (absolute). ArduPilot replaces it with the
                                 real home when the drone arms. MAVROS publishes it as item 0, so every
                                 index below is the same in this file, in ArduPilot and in /mavros/mission/waypoints.
  seq 1  NAV_TAKEOFF             climb to the survey altitude (frame 3 = metres above home)
  seq 2  NAV_WAYPOINT            start of the first survey lane
  seq 3  DO_DIGICAM_CONTROL      ArduPilot prints "Mission: 3 DigiCamCtrl"; mapping/do_digi_cam_trigger.py sees
                                 that STATUSTEXT and starts its 1 Hz /camera/trigger photo loop
 (seq 4  DO_CHANGE_SPEED)        optional: fly the lanes at a photo-friendly speed
  ...    NAV_WAYPOINT x N        the rest of the lane ends, zig-zag (lawn-mower)
  R-2    NAV_WAYPOINT            end of the LAST lane = waypoint_manager's buffer_wp (rtl_index - 2):
                                 the trigger node stops taking photos when ArduPilot reports reaching it
  R-1    NAV_WAYPOINT            hold point over the middle of the search area = last NAV item before RTL:
                                 main_controller.py switches to GUIDED here, waits for detection to finish,
                                 then inserts one waypoint per target at index R (before the RTL) and resumes AUTO
  R      NAV_RETURN_TO_LAUNCH    the controller derives rtl_index from this item

Only DO_ items sit between NAV items, never at R-1 or R-2: DO_ items never produce MISSION_ITEM_REACHED, so
the photo loop would never stop and the controller would never hold.
"""

import math
from dataclasses import dataclass

# --- MAVLink command numbers (MAV_CMD) and frames used here ---------------------------------------------------
NAV_WAYPOINT = 16
NAV_LOITER_UNLIM = 17
NAV_LOITER_TURNS = 18
NAV_LOITER_TIME = 19
NAV_RETURN_TO_LAUNCH = 20
NAV_LAND = 21
NAV_TAKEOFF = 22
DO_CHANGE_SPEED = 178
DO_SET_SERVO = 183
DO_DIGICAM_CONTROL = 203

#: The command ids main_controller.py counts as "physical" NAV items (main/src/main_controller.py:246).
NAV_COMMANDS = {16, 17, 18, 19, 20, 21, 22}

FRAME_GLOBAL = 0                 # MAV_FRAME_GLOBAL: altitude above mean sea level
FRAME_GLOBAL_RELATIVE_ALT = 3    # MAV_FRAME_GLOBAL_RELATIVE_ALT: altitude above home (what Mission Planner uses)

COMMAND_NAMES = {NAV_WAYPOINT: 'NAV_WAYPOINT', NAV_LOITER_UNLIM: 'NAV_LOITER_UNLIM', NAV_LOITER_TURNS: 'NAV_LOITER_TURNS',
                 NAV_LOITER_TIME: 'NAV_LOITER_TIME', NAV_RETURN_TO_LAUNCH: 'NAV_RETURN_TO_LAUNCH', NAV_LAND: 'NAV_LAND',
                 NAV_TAKEOFF: 'NAV_TAKEOFF', DO_CHANGE_SPEED: 'DO_CHANGE_SPEED', DO_SET_SERVO: 'DO_SET_SERVO',
                 DO_DIGICAM_CONTROL: 'DO_DIGICAM_CONTROL'}

HEADER = 'QGC WPL 110'


@dataclass
class MissionItem:
    """One line of a QGC WPL 110 file = one MAVLink MISSION_ITEM.

    seq: index (0 = home). current: 1 only for item 0. frame: MAV_FRAME (0 absolute, 3 relative to home).
    command: MAV_CMD number. param1..param4: command parameters. x, y, z: latitude (deg), longitude (deg),
    altitude (m) for NAV items; extra parameters (param5..7) for DO_ items. autocontinue: 1 = go on by itself.
    """

    seq: int
    current: int
    frame: int
    command: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    autocontinue: int = 1

    @property
    def name(self):
        """Readable command name, e.g. 'NAV_WAYPOINT'."""
        return COMMAND_NAMES.get(self.command, f'CMD_{self.command}')

    @property
    def is_nav(self):
        """True for the NAV commands main_controller.py counts (16..22)."""
        return self.command in NAV_COMMANDS

    def to_line(self):
        """The tab-separated text line, formatted like Mission Planner writes it."""
        return (f'{self.seq}\t{self.current}\t{self.frame}\t{self.command}\t'
                f'{self.param1:.8f}\t{self.param2:.8f}\t{self.param3:.8f}\t{self.param4:.8f}\t'
                f'{self.x:.8f}\t{self.y:.8f}\t{self.z:.6f}\t{self.autocontinue}')

    @classmethod
    def from_line(cls, line):
        """Parse one 12-field line. Raises ValueError with the line text when it is malformed."""
        f = line.split()
        if len(f) != 12:
            raise ValueError(f'mission line needs 12 fields, got {len(f)}: "{line.strip()}"')
        return cls(int(f[0]), int(f[1]), int(f[2]), int(f[3]), float(f[4]), float(f[5]), float(f[6]),
                   float(f[7]), float(f[8]), float(f[9]), float(f[10]), int(f[11]))


def write_waypoints(path, items):
    """Write items to `path` as a QGC WPL 110 file (no comments: Mission Planner does not allow them)."""
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(HEADER + '\n')
        for it in items:
            f.write(it.to_line() + '\n')


def read_waypoints(path):
    """Read a QGC WPL 110 file. Returns [MissionItem, ...]. Raises ValueError on a bad header or line."""
    items = []
    with open(path, encoding='utf-8') as f:
        lines = [ln for ln in f.read().splitlines()]
    if not lines or not lines[0].strip().startswith('QGC WPL 110'):
        raise ValueError(f'{path}: first line must be "{HEADER}"')
    for ln in lines[1:]:
        if not ln.strip() or ln.lstrip().startswith('#'):
            continue
        items.append(MissionItem.from_line(ln))
    return items


def build_survey_mission(home, survey_latlon, altitude_m, hold_latlon, speed_mps=None):
    """Build the team-shaped search mission (see the module text for the exact layout).

    home: sim_worlds.geo.Home (item 0). survey_latlon: [(lat, lon), ...] lane ends in flight order (at least
    2 points: start and end of the first lane). altitude_m: survey height above home [m]. hold_latlon:
    (lat, lon) of the hold point (normally the middle of the search area). speed_mps: optional survey speed
    [m/s]; when given a DO_CHANGE_SPEED item follows the DO_DIGICAM_CONTROL item.
    Returns [MissionItem, ...] with seq numbers 0..N.
    """
    if len(survey_latlon) < 2:
        raise ValueError('a survey needs at least 2 waypoints (start and end of a lane)')
    rel = FRAME_GLOBAL_RELATIVE_ALT
    items = [
        MissionItem(0, 1, FRAME_GLOBAL, NAV_WAYPOINT, x=home.lat, y=home.lon, z=home.alt),
        # lat/lon 0 = take off where the drone stands (ArduPilot uses the current position)
        MissionItem(1, 0, rel, NAV_TAKEOFF, z=altitude_m),
        MissionItem(2, 0, rel, NAV_WAYPOINT, x=survey_latlon[0][0], y=survey_latlon[0][1], z=altitude_m),
        # all-zero parameters: the trigger bridge only needs the STATUSTEXT, and zero x/y survive MAVProxy's
        # MISSION_ITEM -> MISSION_ITEM_INT conversion (it multiplies x/y by 1e7 for every item)
        MissionItem(3, 0, FRAME_GLOBAL, DO_DIGICAM_CONTROL),
    ]
    if speed_mps:
        # param1 = 1 (ground speed), param2 = speed m/s, param3 = -1 (throttle unchanged), param4 = 0 (absolute)
        items.append(MissionItem(len(items), 0, FRAME_GLOBAL, DO_CHANGE_SPEED, 1.0, float(speed_mps), -1.0, 0.0))
    for lat, lon in survey_latlon[1:]:
        items.append(MissionItem(len(items), 0, rel, NAV_WAYPOINT, x=lat, y=lon, z=altitude_m))
    items.append(MissionItem(len(items), 0, rel, NAV_WAYPOINT, x=hold_latlon[0], y=hold_latlon[1], z=altitude_m))
    items.append(MissionItem(len(items), 0, rel, NAV_RETURN_TO_LAUNCH))
    return items


def team_indices(items):
    """The indices last year's nodes would derive from this mission (exact replicas of their code).

    waypoint_manager (waypoint_mavros/waypoint.py waypoints_list): takeoff_index, next_after_takeoff,
    rtl_index, last_before_rtl = rtl_index - 1, buffer_wp = last_before_rtl - 1 (where the photo trigger stops).
    main_controller (main/src/main_controller.py _update_last_nav_before_rtl): last_nav_before_rtl (the hold
    trigger) and its own buffer_wp. Also: digicam_indices (every DO_DIGICAM_CONTROL), insert_index (where the
    controller inserts the first target waypoint = last_before_rtl + 1).
    Returns a dict of ints (-1 = not found, like the team code).
    """
    n = len(items)
    takeoff_index = next_after_takeoff = rtl_index = last_before_rtl = -1
    for i, it in enumerate(items):
        if it.command == NAV_TAKEOFF:
            takeoff_index = i
            if i + 1 < n:
                next_after_takeoff = i + 1
        if it.command == NAV_RETURN_TO_LAUNCH:
            rtl_index = i
            if i - 1 > 0:
                last_before_rtl = i - 1
    buffer_wp = last_before_rtl - 1
    last_nav, mc_buffer = -1, -1
    if rtl_index > 0 and n > 0:
        found = False
        for i in range(min(rtl_index, n) - 1, -1, -1):
            if items[i].command in NAV_COMMANDS:
                if not found:
                    last_nav, found = i, True
                else:
                    mc_buffer = i
                    break
    trigger_wp = last_nav if last_nav >= 0 else last_before_rtl
    return {'num_waypoints': n, 'takeoff_index': takeoff_index, 'next_after_takeoff': next_after_takeoff,
            'rtl_index': rtl_index, 'last_before_rtl': last_before_rtl, 'buffer_wp': buffer_wp,
            'last_nav_before_rtl': last_nav, 'hold_trigger_wp': trigger_wp, 'main_controller_buffer_wp': mc_buffer,
            'digicam_indices': [i for i, it in enumerate(items) if it.command == DO_DIGICAM_CONTROL],
            'insert_index': last_before_rtl + 1 if last_before_rtl >= 0 else -1}


def check_team_shape(items, alt_min=None, alt_max=None):
    """Check `items` against the shape last year's code needs. Returns a list of problems ([] = all good).

    alt_min / alt_max: optional allowed band [m above home] for every NAV_TAKEOFF / NAV_WAYPOINT altitude.
    """
    p = []
    if not items:
        return ['the mission is empty']
    for i, it in enumerate(items):
        if it.seq != i:
            p.append(f'item {i} has seq {it.seq} (must count 0, 1, 2, ...)')
    if not (items[0].command == NAV_WAYPOINT and items[0].frame == FRAME_GLOBAL):
        p.append('item 0 must be HOME (NAV_WAYPOINT, frame 0)')
    takeoffs = [i for i, it in enumerate(items) if it.command == NAV_TAKEOFF]
    rtls = [i for i, it in enumerate(items) if it.command == NAV_RETURN_TO_LAUNCH]
    if takeoffs != [1]:
        p.append(f'exactly one NAV_TAKEOFF at index 1 expected, found at {takeoffs}')
    if len(rtls) != 1 or rtls[0] != len(items) - 1:
        p.append(f'exactly one NAV_RETURN_TO_LAUNCH as the LAST item expected, found at {rtls}')
    if any(it.command == NAV_LAND for it in items):
        p.append('NAV_LAND found: the team mission ends with RTL, not LAND')
    idx = team_indices(items)
    r = idx['rtl_index']
    if r >= 3:
        if items[r - 1].command != NAV_WAYPOINT:
            p.append(f'item R-1 = {r - 1} must be a NAV_WAYPOINT (main_controller holds there), is {items[r - 1].name}')
        if items[r - 2].command != NAV_WAYPOINT:
            p.append(f'item R-2 = {r - 2} must be a NAV_WAYPOINT (waypoint_manager buffer_wp: photos stop there), '
                     f'is {items[r - 2].name}')
    else:
        p.append('the mission is too short: it needs at least TAKEOFF, two waypoints and RTL')
    if idx['hold_trigger_wp'] != idx['last_before_rtl']:
        p.append(f'main_controller would hold at {idx["hold_trigger_wp"]} but insert targets after '
                 f'{idx["last_before_rtl"]}: they must be the same item')
    digicams = idx['digicam_indices']
    if not digicams:
        p.append('no DO_DIGICAM_CONTROL item: the photo trigger would never start')
    else:
        d = digicams[0]
        if not (d >= 3 and items[d - 1].command == NAV_WAYPOINT and items[d - 2].command == NAV_TAKEOFF):
            p.append(f'the first DO_DIGICAM_CONTROL (index {d}) should follow TAKEOFF and the first survey waypoint')
        if idx['buffer_wp'] >= 0 and d >= idx['buffer_wp']:
            p.append('DO_DIGICAM_CONTROL comes after buffer_wp: photos would stop before they start')
    for it in items[1:]:
        if it.command in (NAV_WAYPOINT, NAV_TAKEOFF):
            if it.frame != FRAME_GLOBAL_RELATIVE_ALT:
                p.append(f'item {it.seq} ({it.name}) should use frame 3 (altitude above home)')
            if alt_min is not None and it.z < alt_min - 1e-6:
                p.append(f'item {it.seq} altitude {it.z:.1f} m is below the allowed {alt_min:.1f} m')
            if alt_max is not None and it.z > alt_max + 1e-6:
                p.append(f'item {it.seq} altitude {it.z:.1f} m is above the allowed {alt_max:.1f} m')
        if it.command == NAV_WAYPOINT and (abs(it.x) < 1e-9 or abs(it.y) < 1e-9):
            p.append(f'item {it.seq} NAV_WAYPOINT has latitude/longitude 0')
    return p


def describe(items, home=None):
    """A readable table of the mission (one line per item). With home (geo.Home) it also shows local x/y."""
    from sim_worlds import geo  # local import: keeps this module importable on its own
    idx = team_indices(items)
    roles = {idx['takeoff_index']: 'takeoff', idx['buffer_wp']: 'buffer_wp: photos STOP here',
             idx['last_before_rtl']: 'hold: main_controller waits here (GUIDED)', idx['rtl_index']: 'RTL'}
    for d in idx['digicam_indices']:
        roles[d] = 'photos START ("Mission: N DigiCamCtrl")'
    roles[0] = 'home'
    lines = [f'{"seq":>3}  {"command":<20} {"frame":>5}  {"lat":>12} {"lon":>13} {"alt":>7}  {"x m":>8} {"y m":>8}  role']
    for it in items:
        xy = ''
        if home is not None and it.command == NAV_WAYPOINT and it.seq > 0:
            x, y = geo.latlon_to_enu(it.x, it.y, home)
            xy = f'{x:8.1f} {y:8.1f}'
        else:
            xy = f'{"":8} {"":8}'
        extra = ''
        if it.command == DO_CHANGE_SPEED:
            extra = f' speed {it.param2:g} m/s'
        lines.append(f'{it.seq:>3}  {it.name:<20} {it.frame:>5}  {it.x:12.7f} {it.y:13.7f} {it.z:7.1f}  {xy}  '
                     f'{roles.get(it.seq, "")}{extra}')
    return '\n'.join(lines)


def path_length_m(items, home):
    """Horizontal length (m) of the flight from home through every NAV_WAYPOINT and back home (RTL)."""
    from sim_worlds import geo
    pts = [(0.0, 0.0)]
    for it in items[1:]:
        if it.command == NAV_WAYPOINT:
            pts.append(geo.latlon_to_enu(it.x, it.y, home))
    pts.append((0.0, 0.0))
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts[:-1], pts[1:]))
