"""
sim_demo/mission_file.py -- PURPOSE: read an ArduPilot mission file (".waypoints", the
"QGC WPL 110" text format that Mission Planner and MAVProxy write) and work out its
structure: which item is the takeoff, where the camera starts (DO_DIGICAM_CONTROL), which
waypoints form the search lanes, and which item is the return-to-launch.

The demo panel uses this to say "Searching (lane 3 of 6)" and to upload the mission.
The index rules copy the team's code so the panel agrees with it:
  * waypoint_mavros/waypoint.py: takeoff_index = NAV_TAKEOFF item, rtl_index = NAV_RETURN_TO_LAUNCH
    item, last_before_rtl = rtl_index - 1, buffer_wp = last_before_rtl - 1.
  * main/main_controller.py _update_last_nav_before_rtl(): the "trigger" is the last NAV
    command (16..22) before RTL; reaching it starts the GUIDED photo-processing hold.

File format (one item per line, TAB separated, first line "QGC WPL 110"):
    seq  current  frame  command  p1  p2  p3  p4  lat(x)  lon(y)  alt(z)  autocontinue
Line seq 0 is the home position.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# MAVLink command ids used in SUAS missions.
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_LOITER_UNLIM = 17
MAV_CMD_NAV_LOITER_TURNS = 18
MAV_CMD_NAV_LOITER_TIME = 19
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_SPLINE_WAYPOINT = 82
MAV_CMD_DO_CHANGE_SPEED = 178
MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_DIGICAM_CONTROL = 203
MAV_CMD_DO_MOUNT_CONTROL = 205
MAV_CMD_MISSION_START = 300

# Same set main_controller.py treats as "physical" navigation items.
NAV_COMMANDS = {16, 17, 18, 19, 20, 21, 22}

COMMAND_NAMES = {
    16: 'WAYPOINT', 17: 'LOITER_UNLIM', 18: 'LOITER_TURNS', 19: 'LOITER_TIME',
    20: 'RETURN_TO_LAUNCH', 21: 'LAND', 22: 'TAKEOFF', 82: 'SPLINE_WAYPOINT',
    178: 'DO_CHANGE_SPEED', 183: 'DO_SET_SERVO', 203: 'DO_DIGICAM_CONTROL',
    205: 'DO_MOUNT_CONTROL', 206: 'DO_SET_CAM_TRIGG_DIST',
}


@dataclass
class MissionItem:
    """One line of a .waypoints file (one MAVLink MISSION_ITEM)."""

    seq: int
    current: int
    frame: int
    command: int
    param1: float
    param2: float
    param3: float
    param4: float
    lat: float
    lon: float
    alt: float
    autocontinue: int = 1

    @property
    def name(self) -> str:
        """Readable command name, e.g. 'WAYPOINT'."""
        return COMMAND_NAMES.get(self.command, f'CMD_{self.command}')


@dataclass
class MissionSummary:
    """The structure of a survey mission (indices are mission seq numbers; -1 = absent).

    Attributes:
        takeoff_index, rtl_index, last_before_rtl, buffer_wp: as waypoint_manager computes them.
        last_nav_before_rtl: main_controller's trigger item (start of the processing hold).
        digicam_index: first DO_DIGICAM_CONTROL item (where photos start).
        survey_seqs: NAV_WAYPOINT seqs of the search pattern, in flight order.
        lanes: list of (start_seq, end_seq) pairs, one per search lane.
        survey_alt: altitude of the survey waypoints [m, relative to home] (0 if unknown).
        hold_index: the waypoint after the lanes where main_controller holds (-1 = none).
    """

    takeoff_index: int = -1
    rtl_index: int = -1
    last_before_rtl: int = -1
    buffer_wp: int = -1
    last_nav_before_rtl: int = -1
    digicam_index: int = -1
    hold_index: int = -1
    survey_seqs: list[int] = field(default_factory=list)
    lanes: list[tuple[int, int]] = field(default_factory=list)
    survey_alt: float = 0.0

    @property
    def lane_count(self) -> int:
        """Number of search lanes."""
        return len(self.lanes)

    def lane_for_reached(self, reached_seq: int) -> int:
        """Lane number (1-based) the drone is flying after it reached item `reached_seq`.

        Before the first survey waypoint it is heading to lane 1; after the last one it
        stays on the last lane. Returns 0 if the mission has no lanes.
        """
        if not self.lanes:
            return 0
        count = sum(1 for s in self.survey_seqs if s <= reached_seq)
        return min(len(self.lanes), count // 2 + 1)


def parse_waypoints_text(text: str) -> list[MissionItem]:
    """Parse the contents of a QGC WPL 110 file.

    Raises:
        ValueError: if the header is missing or a line is malformed.
    """
    lines = [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.strip().startswith('#')]
    if not lines or not lines[0].upper().startswith('QGC WPL'):
        raise ValueError('not a QGC WPL mission file (first line must be "QGC WPL 110")')
    items = []
    for n, ln in enumerate(lines[1:], start=2):
        parts = ln.replace('\t', ' ').split()
        if len(parts) < 12:
            raise ValueError(f'mission line {n} has {len(parts)} fields, expected 12: {ln!r}')
        try:
            items.append(MissionItem(
                seq=int(parts[0]), current=int(parts[1]), frame=int(parts[2]),
                command=int(parts[3]), param1=float(parts[4]), param2=float(parts[5]),
                param3=float(parts[6]), param4=float(parts[7]), lat=float(parts[8]),
                lon=float(parts[9]), alt=float(parts[10]), autocontinue=int(float(parts[11]))))
        except ValueError as exc:
            raise ValueError(f'mission line {n} is malformed: {ln!r} ({exc})') from exc
    return items


def load_waypoints_file(path: str) -> list[MissionItem]:
    """Read and parse a .waypoints file (see parse_waypoints_text)."""
    with open(path, encoding='utf-8') as f:
        return parse_waypoints_text(f.read())


def analyze(commands: list[int], alts: list[float] | None = None,
            hold_index: int = -1) -> MissionSummary:
    """Find the survey structure from the list of command ids (index = mission seq).

    Lanes: ArduPilot runs the DO items that follow a waypoint while it flies the NEXT leg, so a
    DO_DIGICAM_CONTROL right after waypoint W switches the camera on as the drone leaves W:
    W is the start of lane 1. After that the waypoints pair up as lane end / next lane start.
    sim_worlds missions end with a "hold" waypoint (the middle of the area, where main_controller
    waits for the detector); give its index as hold_index when known (config survey.planned.
    hold_wp), otherwise an odd waypoint left over at the end is taken as that hold point.

    Args:
        commands: MAV_CMD id of every mission item, item 0 = home.
        alts: optional altitude of every item (for survey_alt).
        hold_index: index of the hold waypoint (-1 = unknown).
    Returns:
        MissionSummary.
    """
    s = MissionSummary()
    for i, cmd in enumerate(commands):
        if cmd == MAV_CMD_NAV_TAKEOFF:
            s.takeoff_index = i
        if cmd == MAV_CMD_NAV_RETURN_TO_LAUNCH:
            s.rtl_index = i
            if i - 1 > 0:
                s.last_before_rtl = i - 1
        if cmd == MAV_CMD_DO_DIGICAM_CONTROL and s.digicam_index < 0:
            s.digicam_index = i
    if s.last_before_rtl >= 0:
        s.buffer_wp = s.last_before_rtl - 1
    end = s.rtl_index if s.rtl_index > 0 else len(commands)
    for i in range(min(end, len(commands)) - 1, -1, -1):
        if commands[i] in NAV_COMMANDS:
            s.last_nav_before_rtl = i
            break
    nav_wp = (MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_SPLINE_WAYPOINT)
    lane_start = -1
    if s.digicam_index > 0:          # the waypoint right before the camera switches on
        for i in range(s.digicam_index - 1, max(s.takeoff_index, 0), -1):
            if commands[i] in nav_wp:
                lane_start = i
                break
            if commands[i] in NAV_COMMANDS:
                break
    start = max(s.digicam_index, s.takeoff_index, 0) + 1
    stop = s.last_nav_before_rtl if s.last_nav_before_rtl >= 0 else end - 1
    seqs = ([lane_start] if lane_start >= 0 else []) + [
        i for i in range(start, stop + 1) if i < len(commands) and commands[i] in nav_wp]
    if hold_index >= 0:
        s.hold_index = hold_index
        seqs = [i for i in seqs if i < hold_index]
    elif len(seqs) > 1 and len(seqs) % 2 == 1:
        s.hold_index = seqs[-1]
        seqs = seqs[:-1]
    s.survey_seqs = seqs
    s.lanes = [(seqs[i], seqs[i + 1] if i + 1 < len(seqs) else seqs[i])
               for i in range(0, len(seqs), 2)]
    if alts and seqs:
        vals = [alts[i] for i in seqs if i < len(alts)]
        s.survey_alt = sorted(vals)[len(vals) // 2] if vals else 0.0
    return s


def analyze_items(items: list[MissionItem], hold_index: int = -1) -> MissionSummary:
    """analyze() for parsed MissionItems (uses each item's position in the list as its seq)."""
    return analyze([it.command for it in items], [it.alt for it in items], hold_index)


def survey_bounds_en(items: list[MissionItem], summary: MissionSummary, origin) -> list[tuple[float, float]]:
    """Rectangle (east, north) around the survey waypoints, as a 4-point polygon.

    Used as the "search area" outline when the world's truth file has none.
    `origin` is a geo.GeoOrigin. Returns [] if there are fewer than 2 survey points.
    """
    pts = []
    for seq in summary.survey_seqs:
        if seq < len(items) and not (items[seq].lat == 0.0 and items[seq].lon == 0.0):
            e, n, _ = origin.to_enu(items[seq].lat, items[seq].lon)
            pts.append((e, n))
    if len(pts) < 2:
        return []
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    x1, x2, y1, y2 = min(xs), max(xs), min(ys), max(ys)
    if math.isclose(x1, x2) or math.isclose(y1, y2):
        return []
    return [(x1, y1), (x2, y1), (x2, y2), (x1, y2)]
