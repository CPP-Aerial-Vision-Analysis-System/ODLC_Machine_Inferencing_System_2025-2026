"""
sim_bringup/mission_tool.py -- PURPOSE: a tiny MAVLink "ground station" for the simulator (what Mission Planner does
at the field: upload the survey mission, wait until the drone is ready, arm it and start the mission).

It talks to ArduCopter SITL through MAVProxy's ground-station output (UDP 127.0.0.1:14550 by default) with
pymavlink. The team's own code never arms or starts a mission (Mission Planner did that by hand), so the
demo needs this tool. Subcommands:

  upload  --mission FILE     upload a mission file (QGC WPL 110 ".waypoints", the Mission Planner format)
  wait-ready                 wait until SITL has a 3D GPS fix, a healthy EKF and passes its pre-arm checks
                             (the first ~30 s after boot "Arm: Gyros inconsistent" is normal settling)
  arm                        switch to GUIDED and arm, retrying while the pre-arm checks still settle
  start                      MAV_CMD_MISSION_START: switch to AUTO and fly the mission from the start
  fly     --mission FILE     upload + wait-ready + arm + start, then confirm the take-off
  watch                      print the autopilot's messages, mode and altitude until it lands and disarms
  status                     print one line: mode, armed, GPS, altitude, battery, mission item
  mode NAME | rtl | land | disarm      simple commands

Common options: --url (MAVLink connection, default udpin:127.0.0.1:14550), --timeout SECONDS.
Exit codes: 0 = done, 1 = failed/timed out (the reason is printed), 2 = bad usage.
"""

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

DEFAULT_URL = 'udpin:127.0.0.1:14550'
SOURCE_SYSTEM = 245          # our MAVLink system id (MAVProxy uses 255, MAVROS 1/191)
SOURCE_COMPONENT = 190       # MAV_COMP_ID_MISSIONPLANNER

#: Commands whose x/y are a latitude/longitude (ArduPilot's AP_Mission::stored_in_location list). For all
#: other commands x/y are plain parameters (e.g. DO_DIGICAM_CONTROL's "shoot" flag) and must NOT be scaled
#: by 1e7 in MISSION_ITEM_INT.
LOCATION_COMMANDS = {16, 17, 18, 19, 21, 22, 31, 82, 92, 179, 189, 191, 192, 195, 201, 84, 85, 94}

#: Human names of the commands that appear in survey missions (for printing).
COMMAND_NAMES = {16: 'WAYPOINT', 17: 'LOITER_UNLIM', 18: 'LOITER_TURNS', 19: 'LOITER_TIME',
                 20: 'RETURN_TO_LAUNCH', 21: 'LAND', 22: 'TAKEOFF', 177: 'DO_JUMP', 178: 'DO_CHANGE_SPEED',
                 183: 'DO_SET_SERVO', 203: 'DO_DIGICAM_CONTROL', 206: 'DO_SET_CAM_TRIGG_DIST',
                 205: 'DO_MOUNT_CONTROL', 1000: 'DO_GIMBAL_MANAGER_PITCHYAW'}


def say(text: str) -> None:
    """Print one tool message (flushed, so it appears immediately in logs)."""
    print(f'[mission] {text}', flush=True)


# ---------------------------------------------------------------------------------------------
# Mission files
# ---------------------------------------------------------------------------------------------

@dataclass
class MissionItem:
    """One line of a QGC WPL 110 file.

    seq, current (1 = current item), frame (MAV_FRAME: 0 global, 3 global relative alt, 2 mission),
    command (MAV_CMD), p1..p4 (command parameters), x/y (lat/lon in degrees for location commands,
    otherwise parameters 5/6), z (altitude in metres or parameter 7), autocontinue (1 = go on).
    """
    seq: int
    current: int
    frame: int
    command: int
    p1: float
    p2: float
    p3: float
    p4: float
    x: float
    y: float
    z: float
    autocontinue: int

    @property
    def name(self) -> str:
        """Readable command name, e.g. 'TAKEOFF'."""
        return COMMAND_NAMES.get(self.command, f'CMD_{self.command}')

    def x_int(self) -> int:
        """x for MISSION_ITEM_INT: lat*1e7 for location commands, else the plain integer value."""
        return int(round(self.x * 1e7)) if self.command in LOCATION_COMMANDS else int(round(self.x))

    def y_int(self) -> int:
        """y for MISSION_ITEM_INT: lon*1e7 for location commands, else the plain integer value."""
        return int(round(self.y * 1e7)) if self.command in LOCATION_COMMANDS else int(round(self.y))


def parse_waypoints(text: str) -> List[MissionItem]:
    """Parse a QGC WPL 110 mission text into MissionItems (item 0 = home, like Mission Planner writes it).

    Raises ValueError with a plain message for a wrong header or a broken line. Sequence numbers are
    renumbered 0..N-1 in file order (that is what the autopilot receives anyway).
    """
    lines = [ln.strip() for ln in text.splitlines() if ln.strip() and not ln.strip().startswith('#')]
    if not lines or not lines[0].startswith('QGC WPL'):
        raise ValueError('not a mission file: the first line must be "QGC WPL 110"')
    items: List[MissionItem] = []
    for number, line in enumerate(lines[1:], start=2):
        cols = line.replace(',', ' ').split()
        if len(cols) != 12:
            raise ValueError(f'mission line {number} has {len(cols)} columns, expected 12: "{line}"')
        try:
            items.append(MissionItem(seq=len(items), current=int(float(cols[1])), frame=int(float(cols[2])),
                                     command=int(float(cols[3])), p1=float(cols[4]), p2=float(cols[5]),
                                     p3=float(cols[6]), p4=float(cols[7]), x=float(cols[8]), y=float(cols[9]),
                                     z=float(cols[10]), autocontinue=int(float(cols[11]))))
        except ValueError:
            raise ValueError(f'mission line {number} contains something that is not a number: "{line}"') from None
    if not items:
        raise ValueError('the mission file has a header but no items')
    return items


def describe_mission(items: List[MissionItem]) -> Dict[str, object]:
    """Summarise a mission the way the team code sees it.

    Returns {'count', 'takeoff', 'digicam', 'rtl', 'nav_after_digicam', 'warnings'} where the indices are
    mission sequence numbers. The team's code needs: a TAKEOFF, a DO_DIGICAM_CONTROL (starts the 1 Hz
    photos), at least two NAV waypoints after it (photos stop at the second-to-last one, the processing hold
    happens at the last one) and RETURN_TO_LAUNCH as the last item.
    """
    def first(cmd):
        return next((i.seq for i in items if i.command == cmd), -1)
    takeoff, digicam, rtl = first(22), first(203), first(20)
    nav_after = sum(1 for i in items if i.seq > max(digicam, 0) and i.command == 16 and (rtl < 0 or i.seq < rtl))
    warnings = []
    if takeoff < 0:
        warnings.append('no TAKEOFF item: ArduCopter will not take off in AUTO')
    if digicam < 0:
        warnings.append('no DO_DIGICAM_CONTROL item: the team\'s trigger never starts taking photos')
    if rtl < 0:
        warnings.append('no RETURN_TO_LAUNCH item: main_controller cannot find where to insert target waypoints')
    elif rtl != items[-1].seq:
        warnings.append('RETURN_TO_LAUNCH is not the last item')
    if nav_after < 2:
        warnings.append('fewer than two WAYPOINTs after DO_DIGICAM_CONTROL: photos may never stop / no hold point')
    return {'count': len(items), 'takeoff': takeoff, 'digicam': digicam, 'rtl': rtl,
            'nav_after_digicam': nav_after, 'warnings': warnings}


# ---------------------------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------------------------

class Vehicle:
    """A pymavlink connection to the simulated drone plus the small amount of state the tool needs."""

    def __init__(self, url: str, timeout: float = 60.0):
        """Open `url` and wait (up to `timeout` s) for the autopilot's heartbeat. Raises RuntimeError."""
        from pymavlink import mavutil   # imported here so the file parser works without pymavlink
        self.mavutil = mavutil
        self.mavlink = mavutil.mavlink
        say(f'connecting to {url} ...')
        self.m = mavutil.mavlink_connection(url, source_system=SOURCE_SYSTEM, source_component=SOURCE_COMPONENT,
                                            dialect='ardupilotmega', autoreconnect=True)
        self.statustexts: List[str] = []
        self.last: Dict[str, object] = {}
        hb = self.wait_autopilot_heartbeat(timeout)
        if hb is None:
            raise RuntimeError(f'no autopilot heartbeat on {url} within {timeout:.0f} s. Is the simulator running '
                               '(simulation/scripts/start_sim.sh) and is nothing else using this port?')
        self.ts, self.tc = hb.get_srcSystem(), hb.get_srcComponent()
        say(f'connected to the autopilot (system {self.ts}, component {self.tc})')

    # -- receiving -------------------------------------------------------------------------------
    def wait_autopilot_heartbeat(self, timeout: float):
        """Wait for a HEARTBEAT from an ArduPilot autopilot (ignores GCS/onboard heartbeats)."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.m.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg is not None and msg.autopilot == self.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.last['HEARTBEAT'] = msg
                return msg
        return None

    def pump(self, seconds: float = 0.0, types: Optional[List[str]] = None):
        """Read messages for `seconds` (at least once), keep the newest of each type, collect STATUSTEXT.

        Returns the first message of one of `types` if given and seen, else None.
        """
        end = time.monotonic() + seconds
        while True:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                mtype = msg.get_type()
                if mtype == 'BAD_DATA':
                    pass
                elif msg.get_srcSystem() == getattr(self, 'ts', msg.get_srcSystem()):
                    self.last[mtype] = msg
                    if mtype == 'STATUSTEXT':
                        self.statustexts.append(msg.text)
                    if types and mtype in types:
                        return msg
            if time.monotonic() >= end:
                return None

    # -- state -----------------------------------------------------------------------------------
    @property
    def armed(self) -> bool:
        """True when the newest heartbeat says the motors are armed."""
        hb = self.last.get('HEARTBEAT')
        return bool(hb and hb.base_mode & self.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    @property
    def mode(self) -> str:
        """Current flight mode name (e.g. 'GUIDED'), from the newest heartbeat."""
        hb = self.last.get('HEARTBEAT')
        if hb is None:
            return '?'
        names = {v: k for k, v in self.m.mode_mapping().items()} if self.m.mode_mapping() else {}
        return names.get(hb.custom_mode, str(hb.custom_mode))

    def rel_alt_m(self) -> Optional[float]:
        """Altitude above home in metres (GLOBAL_POSITION_INT), or None if not received yet."""
        gp = self.last.get('GLOBAL_POSITION_INT')
        return gp.relative_alt / 1000.0 if gp else None

    def new_statustexts(self) -> List[str]:
        """Return and forget the STATUSTEXT lines collected since the last call."""
        out, self.statustexts = self.statustexts, []
        return out

    # -- commands --------------------------------------------------------------------------------
    def command(self, cmd: int, *params: float, wait: float = 5.0) -> Optional[int]:
        """Send COMMAND_LONG `cmd` with up to 7 params; return the COMMAND_ACK result (None = no answer)."""
        p = list(params) + [0.0] * (7 - len(params))
        self.m.mav.command_long_send(self.ts, self.tc, cmd, 0, *p)
        end = time.monotonic() + wait
        while time.monotonic() < end:
            ack = self.pump(0.3, types=['COMMAND_ACK'])
            if ack is not None and ack.command == cmd and getattr(ack, 'target_system', SOURCE_SYSTEM) in (0, SOURCE_SYSTEM):
                return ack.result
        return None

    def set_mode(self, name: str, wait: float = 8.0) -> bool:
        """Switch flight mode by name (GUIDED, AUTO, RTL, LAND, LOITER ...) and confirm it."""
        mapping = self.m.mode_mapping() or {}
        name = name.upper()
        if name not in mapping:
            say(f'unknown mode "{name}" (known: {", ".join(sorted(mapping))})')
            return False
        end = time.monotonic() + wait
        while time.monotonic() < end:
            self.m.set_mode(mapping[name])
            self.pump(1.0)
            if self.mode == name:
                return True
        return self.mode == name


# ---------------------------------------------------------------------------------------------
# The jobs
# ---------------------------------------------------------------------------------------------

def upload(v: Vehicle, items: List[MissionItem], timeout: float = 30.0) -> bool:
    """Upload a mission with the MAVLink mission protocol (MISSION_COUNT, then one item per request).

    Every request (MISSION_REQUEST_INT or the older MISSION_REQUEST) is answered with MISSION_ITEM_INT
    (lat/lon * 1e7 for location commands). Returns True when the autopilot answers MISSION_ACK "accepted"
    after all items.
    """
    mav = v.mavlink
    say(f'uploading {len(items)} mission items ...')
    v.m.mav.mission_count_send(v.ts, v.tc, len(items), mav.MAV_MISSION_TYPE_MISSION)
    sent = set()
    last_activity = time.monotonic()
    count_resends = 0
    while True:
        msg = v.pump(0.5, types=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'])
        now = time.monotonic()
        if msg is None:
            if not sent and now - last_activity > 3.0 and count_resends < 3:
                count_resends += 1
                v.m.mav.mission_count_send(v.ts, v.tc, len(items), mav.MAV_MISSION_TYPE_MISSION)
                last_activity = now
            elif now - last_activity > timeout:
                say(f'upload FAILED: the autopilot stopped asking for items ({len(sent)}/{len(items)} sent)')
                return False
            continue
        if getattr(msg, 'target_system', SOURCE_SYSTEM) not in (0, SOURCE_SYSTEM):
            continue                                    # meant for another ground station (e.g. MAVROS)
        last_activity = now
        if msg.get_type() == 'MISSION_ACK':
            if msg.type == mav.MAV_MISSION_ACCEPTED and len(sent) == len(items):
                say(f'upload OK: {len(items)} items accepted by the autopilot')
                return True
            if msg.type != mav.MAV_MISSION_ACCEPTED:
                reason = mav.enums['MAV_MISSION_RESULT'][msg.type].name if msg.type in mav.enums['MAV_MISSION_RESULT'] else msg.type
                say(f'upload FAILED: autopilot answered {reason}')
                return False
            continue
        seq = msg.seq
        if not 0 <= seq < len(items):
            say(f'autopilot asked for item {seq}, which does not exist')
            continue
        it = items[seq]
        # Always answer with MISSION_ITEM_INT (exact coordinates). ArduPilot accepts it for both request
        # types and warns "GCS should send MISSION_ITEM_INT" when it gets the old float MISSION_ITEM.
        v.m.mav.mission_item_int_send(v.ts, v.tc, seq, it.frame, it.command, 1 if seq == 0 else 0,
                                      it.autocontinue, it.p1, it.p2, it.p3, it.p4, it.x_int(), it.y_int(),
                                      it.z, mav.MAV_MISSION_TYPE_MISSION)
        sent.add(seq)


def wait_ready(v: Vehicle, timeout: float = 240.0, min_boot_s: float = 30.0) -> bool:
    """Wait until the autopilot can be armed: 3D GPS fix, healthy EKF, pre-arm checks passed, booted >= min_boot_s.

    Prints what it is still waiting for every 10 s (including the autopilot's own PreArm messages).
    """
    mav = v.mavlink
    ekf_needed = (mav.EKF_ATTITUDE | mav.EKF_VELOCITY_HORIZ | mav.EKF_POS_HORIZ_ABS | mav.EKF_POS_VERT_ABS)
    prearm_bit = getattr(mav, 'MAV_SYS_STATUS_PREARM_CHECK', 1 << 28)
    start = time.monotonic()
    last_print = -100.0
    prearm_msgs: List[str] = []
    while time.monotonic() - start < timeout:
        v.pump(1.0)
        for text in v.new_statustexts():
            if text.startswith(('PreArm', 'Arm')):
                prearm_msgs = (prearm_msgs + [text])[-3:]
        gps, ekf, sys_status = v.last.get('GPS_RAW_INT'), v.last.get('EKF_STATUS_REPORT'), v.last.get('SYS_STATUS')
        boot_ms = max((getattr(m, 'time_boot_ms', 0) for m in v.last.values()), default=0)
        checks = {
            'GPS 3D fix': bool(gps and gps.fix_type >= 3),
            'EKF healthy': bool(ekf and (ekf.flags & ekf_needed) == ekf_needed),
            f'booted {min_boot_s:.0f} s': boot_ms / 1000.0 >= min_boot_s,
        }
        if sys_status and sys_status.onboard_control_sensors_present & prearm_bit:
            checks['pre-arm checks'] = bool(sys_status.onboard_control_sensors_health & prearm_bit)
        if all(checks.values()):
            say(f'ready to arm after {time.monotonic() - start:.0f} s (autopilot up {boot_ms / 1000:.0f} s)')
            return True
        elapsed = time.monotonic() - start
        if elapsed - last_print >= 10.0:
            todo = ', '.join(k for k, ok in checks.items() if not ok)
            extra = f' | autopilot says: {prearm_msgs[-1]}' if prearm_msgs else ''
            say(f'waiting for: {todo} ({elapsed:.0f} s){extra}')
            last_print = elapsed
    say(f'NOT READY after {timeout:.0f} s')
    return False


def arm(v: Vehicle, timeout: float = 120.0) -> bool:
    """Switch to GUIDED and arm. Retries every 5 s while the autopilot refuses (and prints why)."""
    mav = v.mavlink
    if not v.set_mode('GUIDED'):
        say(f'could not switch to GUIDED (mode is {v.mode})')
        return False
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        v.new_statustexts()
        result = v.command(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1, wait=5.0)
        # The heartbeat that shows "armed" comes once per second: give it time before judging.
        settle = time.monotonic() + (4.0 if result == mav.MAV_RESULT_ACCEPTED else 1.0)
        while time.monotonic() < settle and not v.armed:
            v.pump(0.5)
        if v.armed:
            say('ARMED')
            return True
        why = [t for t in v.new_statustexts() if t.startswith(('PreArm', 'Arm'))]
        result_name = 'no answer' if result is None else mav.enums['MAV_RESULT'][result].name
        say(f'arming refused ({result_name}){": " + why[-1] if why else ""} -- retrying in 5 s '
            '(the first ~30 s after boot this is normal)')
        v.pump(4.0)
    say(f'could not arm within {timeout:.0f} s')
    return False


def start_mission(v: Vehicle) -> bool:
    """Send MAV_CMD_MISSION_START: ArduCopter switches to AUTO and starts at the first item.

    (A plain AUTO mode switch on the ground would wait for the pilot's throttle; MISSION_START does not.)
    """
    mav = v.mavlink
    result = v.command(mav.MAV_CMD_MISSION_START, 0, 0, wait=5.0)
    v.pump(1.5)
    if result == mav.MAV_RESULT_ACCEPTED and v.mode == 'AUTO':
        say('mission STARTED (mode AUTO)')
        return True
    name = 'no answer' if result is None else mav.enums['MAV_RESULT'][result].name
    say(f'mission start refused ({name}), mode is {v.mode}. Trying the AUTO mode switch instead.')
    return v.set_mode('AUTO')


def confirm_takeoff(v: Vehicle, timeout: float = 45.0, min_alt: float = 2.0) -> bool:
    """Wait until the drone is above `min_alt` metres (relative to home)."""
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        v.pump(1.0)
        alt = v.rel_alt_m()
        if alt is not None and alt >= min_alt:
            say(f'TAKEOFF confirmed: {alt:.1f} m and climbing')
            return True
        if not v.armed:
            say('the drone disarmed instead of taking off (see the autopilot messages above)')
            return False
    say(f'no takeoff seen within {timeout:.0f} s (altitude {v.rel_alt_m()})')
    return False


def watch(v: Vehicle, timeout: float = 3600.0) -> bool:
    """Print the autopilot's messages and mode/altitude changes until it has flown, landed and disarmed."""
    start = time.monotonic()
    flew, last_mode, last_line = False, None, -100.0
    while time.monotonic() - start < timeout:
        v.pump(1.0)
        for text in v.new_statustexts():
            say(f'autopilot: {text}')
        if v.mode != last_mode:
            say(f'mode -> {v.mode}')
            last_mode = v.mode
        alt = v.rel_alt_m()
        if alt is not None and alt > 3.0:
            flew = True
        if time.monotonic() - last_line >= 15.0 and alt is not None:
            mc = v.last.get('MISSION_CURRENT')
            say(f'altitude {alt:5.1f} m | mode {v.mode} | {"ARMED" if v.armed else "disarmed"}'
                f'{" | mission item " + str(mc.seq) if mc else ""}')
            last_line = time.monotonic()
        if flew and not v.armed:
            say('LANDED and disarmed: flight finished')
            return True
    say('watch timed out')
    return False


def status_line(v: Vehicle) -> str:
    """One-line summary of the vehicle state (after listening for ~2 s)."""
    v.pump(2.0)
    gps, bat, mc = v.last.get('GPS_RAW_INT'), v.last.get('SYS_STATUS'), v.last.get('MISSION_CURRENT')
    alt = v.rel_alt_m()
    parts = [f'mode {v.mode}', 'ARMED' if v.armed else 'disarmed',
             f'GPS fix {gps.fix_type} ({gps.satellites_visible} sats)' if gps else 'GPS ?',
             f'alt {alt:.1f} m' if alt is not None else 'alt ?',
             f'battery {bat.battery_remaining}%' if bat and bat.battery_remaining >= 0 else 'battery ?',
             f'mission item {mc.seq}' if mc else 'mission item ?']
    return ' | '.join(parts)


def main(argv=None) -> int:
    """Command-line entry point (see the module docstring)."""
    parser = argparse.ArgumentParser(prog='mission_tool', description=__doc__.split('\n\n')[0])
    parser.add_argument('--url', default=DEFAULT_URL, help=f'MAVLink connection (default {DEFAULT_URL})')
    parser.add_argument('--timeout', type=float, default=0.0, help='override the step timeout in seconds')
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('upload', help='upload a .waypoints mission')
    p.add_argument('--mission', required=True)
    sub.add_parser('wait-ready', help='wait until SITL can be armed').add_argument('--min-boot', type=float, default=30.0)
    sub.add_parser('arm', help='GUIDED + arm (with retries)')
    sub.add_parser('start', help='start the uploaded mission (AUTO)')
    p = sub.add_parser('fly', help='upload + wait-ready + arm + start + confirm takeoff')
    p.add_argument('--mission', default='', help='mission file ("" = fly the mission already on the autopilot)')
    p.add_argument('--min-boot', type=float, default=30.0)
    sub.add_parser('watch', help='print messages until landed and disarmed')
    sub.add_parser('status', help='one-line state')
    sub.add_parser('mode', help='switch flight mode').add_argument('name')
    sub.add_parser('rtl', help='return to launch')
    sub.add_parser('land', help='land here')
    sub.add_parser('disarm', help='disarm (only on the ground)')
    sub.add_parser('check-mission', help='only read and check a mission file').add_argument('--mission', required=True)
    args = parser.parse_args(argv)

    items: List[MissionItem] = []
    if getattr(args, 'mission', ''):
        try:
            with open(args.mission, 'r', encoding='utf-8') as f:
                items = parse_waypoints(f.read())
        except (OSError, ValueError) as exc:
            say(f'cannot use mission file {args.mission}: {exc}')
            return 1
        info = describe_mission(items)
        say(f'mission {args.mission}: {info["count"]} items (takeoff #{info["takeoff"]}, photos start #{info["digicam"]}, '
            f'RTL #{info["rtl"]}, {info["nav_after_digicam"]} survey waypoints after the photo start)')
        for w in info['warnings']:
            say(f'WARNING: {w}')
        if args.cmd == 'check-mission':
            return 0 if not info['warnings'] else 1

    try:
        v = Vehicle(args.url, timeout=args.timeout or 60.0)
    except Exception as exc:  # noqa: BLE001 - connection problems become one clear line
        say(f'ERROR: {exc}')
        return 1

    t = args.timeout or None
    ok = True
    if args.cmd == 'upload':
        ok = upload(v, items, timeout=t or 30.0)
    elif args.cmd == 'wait-ready':
        ok = wait_ready(v, timeout=t or 240.0, min_boot_s=args.min_boot)
    elif args.cmd == 'arm':
        ok = arm(v, timeout=t or 120.0)
    elif args.cmd == 'start':
        ok = start_mission(v)
    elif args.cmd == 'fly':
        ok = ((not items or upload(v, items)) and wait_ready(v, timeout=t or 240.0, min_boot_s=args.min_boot)
              and arm(v) and start_mission(v) and confirm_takeoff(v))
    elif args.cmd == 'watch':
        ok = watch(v, timeout=t or 3600.0)
    elif args.cmd == 'status':
        say(status_line(v))
    elif args.cmd == 'mode':
        ok = v.set_mode(args.name)
        say(f'mode is now {v.mode}')
    elif args.cmd in ('rtl', 'land'):
        ok = v.set_mode(args.cmd.upper())
        say(f'mode is now {v.mode}')
    elif args.cmd == 'disarm':
        result = v.command(v.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        v.pump(1.0)
        ok = not v.armed
        say('disarmed' if ok else f'disarm refused (result {result}); it only works on the ground')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
