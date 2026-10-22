"""
sim_plugins/params.py -- PURPOSE: use a REAL drone's ArduPilot parameter file in the simulator, safely.

Mission Planner can save every parameter of the Pixhawk into a .param file ("Config -> Full Parameter
List -> Save"). Loading that file into ArduCopter SITL makes the simulated autopilot fly the mission the
way the real one does (waypoint speed, waypoint radius, RTL height, failsafes ...). But some lines
describe the PHYSICAL drone and would break the simulator:
  * sensor calibrations and sensor IDs (INS_*, COMPASS_*, BARO*): SITL's sensors have other IDs, so the
    autopilot would refuse to arm ("3D Accel calibration needed");
  * wiring (SERIAL*, GPS*, BATT*, RC*, CAN*, BRD_*): SITL's GPS, battery and links are on other ports;
  * the airframe (FRAME_*, MOT_*, ATC_*, PSC_*, SERVO1..8_*): the Gazebo drone is an Iris quad X with its
    own motors and tuning; a hexacopter's frame or PIDs would make it crash;
  * a few values the simulator itself needs (astra_sitl.parm): payload servos 11/13 free, no geofence,
    no ground-station failsafe, arming in AUTO without RC sticks, real-time speed.
The "safe" filter (default) drops exactly those and keeps everything that describes the MISSION
(WPNAV_*, WP_*, RTL_*, LAND_*, LOIT_*, FS_* failsafes, ARMING_*, ...). param_filter "none" keeps every
line (experts). param_keep lists names to keep anyway.

Formats read: Mission Planner .param ("NAME,VALUE" or "NAME VALUE"), QGroundControl .params
("1<TAB>1<TAB>NAME<TAB>VALUE<TAB>TYPE"), and ArduPilot's own .parm files; '#' starts a comment.
Output: a SITL --defaults file with one "NAME VALUE" per line, every line far shorter than 90 characters
(ArduPilot reads defaults files in 100-byte pieces; see sim_models/config/astra_sitl.parm).
"""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

NAME_RE = re.compile(r'^[A-Z][A-Z0-9_]{0,15}$')

#: (regular expression, reason) of the parameter groups the safe filter drops. Checked in order.
DROP_RULES: List[Tuple[str, str]] = [
    # -- the simulator's own settings (sim_models/config/astra_sitl.parm and the Gazebo model) --------
    (r'^SIM_', 'simulator setting (comes from the simulator\'s own files)'),
    (r'^(SERVO11_FUNCTION|SERVO13_FUNCTION)$', 'payload release servo must stay free for DO_SET_SERVO '
                                               '(astra_sitl.parm)'),
    (r'^(FENCE_ENABLE|FS_GCS_ENABLE|AUTO_OPTIONS|PLND_ENABLED)$', 'the simulator needs its own value '
                                                                   '(astra_sitl.parm)'),
    (r'^MNT', 'gimbal driver: the SIYI camera stabilises itself (astra_sitl.parm MNT1_TYPE 0)'),
    (r'^(SYSID_|MAV_SYSID$|MAV_GCS_SYSID$)', 'MAVLink system id: MAVROS in the simulator talks to system 1'),
    # -- the airframe: the Gazebo drone is an Iris quad X with its own motors and tuning ---------------
    (r'^FRAME_', 'airframe (the Gazebo drone is an Iris quad X)'),
    (r'^SERVO[1-8]_', 'motor output (the Gazebo Iris uses outputs 1-4 for its motors)'),
    (r'^(MOT_|ATC_|PSC_|FFT_)', 'airframe tuning (motors / PIDs of another drone)'),
    # -- the physical Pixhawk: sensors, calibrations, wiring -------------------------------------------
    (r'^(INS_|COMPASS_|BARO|GND_|ARSPD|AHRS_ORIENTATION$|AHRS_TRIM_)', 'sensor setting or calibration'),
    (r'^(EK3_SRC|EK3_IMU_MASK$|EK2_IMU_MASK$)', 'sensor source selection'),
    (r'^(SERIAL|GPS|BATT|RC|RSSI_|CAN|BRD_|NTF_|RNGFND|FLOW_|PRX|BCN_|ADSB_|VISO|EAHRS|DID_|RPM|TEMP|EFI_|'
     r'GEN_|ICE_|OSD|VTX_|ESC_|BLH_|SERVO_|CAM|RELAY|BTN_|NET_|NMEA_|AIS_|TUNE)',
     'hardware wiring or device (the simulator has its own)'),
    (r'^SR\d+_', 'telemetry rate of a serial port (the simulator\'s links have their own)'),
    # -- read-only and statistics -------------------------------------------------------------------
    (r'^(STAT_|FORMAT_VERSION$|SYSID_SW_)', 'read-only / statistics'),
]
_COMPILED = [(re.compile(rx), why) for rx, why in DROP_RULES]

#: Values the simulator relies on, compared with the user's file to print a warning when the REAL drone
#: differs (those settings would matter in a real flight).
REAL_DRONE_CHECKS: Dict[str, Tuple[float, str]] = {
    'SERVO11_FUNCTION': (0.0, 'on the real drone DO_SET_SERVO on output 11 (person drop) would be refused'),
    'SERVO13_FUNCTION': (0.0, 'on the real drone DO_SET_SERVO on output 13 (tent drop) would be refused'),
}


@dataclass
class ParamLine:
    """One parameter from a file: name, value (as written) and where it was."""
    name: str
    value: str
    line: int


@dataclass
class ParamFileReport:
    """What happened to one input file.

    source   the file read
    output   the SITL defaults file written ('' when nothing was written)
    kept     parameters that go to SITL
    dropped  (name, reason) of the parameters the filter removed
    bad      (line number, text) of lines that are not "NAME VALUE"
    warnings plain-words warnings (e.g. a real-drone setting that would break a payload drop)
    """
    source: str
    output: str = ''
    kept: List[ParamLine] = field(default_factory=list)
    dropped: List[Tuple[str, str]] = field(default_factory=list)
    bad: List[Tuple[int, str]] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    def summary(self) -> str:
        """One line for the launch log / the plan."""
        return (f'{Path(self.source).name}: {len(self.kept)} parameter(s) used, {len(self.dropped)} dropped by '
                f'the filter, {len(self.bad)} unreadable line(s)' + (f' -> {self.output}' if self.output else ''))


def _is_number(text: str) -> bool:
    try:
        return math.isfinite(float(text))
    except ValueError:
        return False


def parse_param_text(text: str) -> Tuple[List[ParamLine], List[Tuple[int, str]]]:
    """Read a parameter file's text. Returns (parameters, bad lines). Later lines win for a repeated name."""
    found: Dict[str, ParamLine] = {}
    bad: List[Tuple[int, str]] = []
    for number, raw in enumerate(text.splitlines(), start=1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        cols = line.split('\t')
        if len(cols) >= 4 and cols[0].strip().isdigit() and cols[1].strip().isdigit():
            name, value = cols[2].strip(), cols[3].strip()          # QGroundControl .params
        else:
            parts = re.split(r'[,\s=]+', line)
            if len(parts) < 2:
                bad.append((number, raw.strip()))
                continue
            name, value = parts[0].strip(), parts[1].strip()
        name = name.upper()
        if not NAME_RE.match(name) or not _is_number(value):
            bad.append((number, raw.strip()))
            continue
        found[name] = ParamLine(name, value, number)
    return list(found.values()), bad


def drop_reason(name: str, mode: str = 'safe', keep: Iterable[str] = ()) -> str:
    """Why the filter drops this parameter ('' = it is kept)."""
    if mode == 'none' or name in set(keep):
        return ''
    for rx, why in _COMPILED:
        if rx.match(name):
            return why
    return ''


def filter_params(lines: Sequence[ParamLine], mode: str = 'safe', keep: Iterable[str] = ()
                  ) -> Tuple[List[ParamLine], List[Tuple[str, str]]]:
    """Split parameters into (kept, [(dropped name, reason)])."""
    keep = set(keep)
    kept, dropped = [], []
    for p in lines:
        why = drop_reason(p.name, mode, keep)
        if why:
            dropped.append((p.name, why))
        else:
            kept.append(p)
    return kept, dropped


def real_drone_warnings(lines: Sequence[ParamLine], mode: str = 'safe', keep: Iterable[str] = ()) -> List[str]:
    """Warnings for settings of the REAL drone that would break its mission (see REAL_DRONE_CHECKS)."""
    keep = set(keep)
    out = []
    for p in lines:
        if p.name in REAL_DRONE_CHECKS:
            want, why = REAL_DRONE_CHECKS[p.name]
            if float(p.value) != want:
                used = mode == 'none' or p.name in keep
                what = 'the simulated drop fails too' if used else f'the simulator keeps {want:g}'
                out.append(f'your file sets {p.name} = {p.value} ({what}): {why}')
    return out


def sitl_defaults_text(lines: Sequence[ParamLine], source: str) -> str:
    """A SITL --defaults file: a short header and one "NAME VALUE" per line (all lines < 90 characters)."""
    header = ['# Made by sim_plugins (params.py) from:',
              f'#   {Path(source).name[:70]}',
              '# One "NAME VALUE" per line. Rewritten at every start: edit the original file instead.']
    body = [f'{p.name} {p.value}' for p in lines]
    return '\n'.join(header + body) + '\n'


def output_name(source: str) -> str:
    """File name of the cleaned copy: <stem>_<8 hex of the full path>.sim.parm (no clashes, no spaces)."""
    stem = re.sub(r'[^A-Za-z0-9_.-]+', '_', Path(source).stem)[:40] or 'params'
    digest = hashlib.sha1(str(source).encode('utf-8')).hexdigest()[:8]
    return f'{stem}_{digest}.sim.parm'


def prepare_param_file(source: str, out_dir: Optional[Path], mode: str = 'safe', keep: Iterable[str] = (),
                       write: bool = True) -> ParamFileReport:
    """Read, filter and (when write) save one parameter file for SITL. Never raises for a bad file:
    problems are in the report (report.output stays '' when the file could not be read)."""
    report = ParamFileReport(source=str(source))
    try:
        text = Path(source).read_text(encoding='utf-8', errors='replace')
    except OSError as exc:
        report.warnings.append(f'cannot read {source}: {exc}')
        return report
    lines, report.bad = parse_param_text(text)
    report.kept, report.dropped = filter_params(lines, mode, keep)
    report.warnings += real_drone_warnings(lines, mode, keep)
    if not lines:
        report.warnings.append(f'{Path(source).name} contains no "NAME VALUE" lines')
    if write and out_dir is not None:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        out = out_dir / output_name(str(source))
        out.write_text(sitl_defaults_text(report.kept, str(source)), encoding='utf-8')
        report.output = str(out)
    return report


def prepare_param_files(sources: Sequence[str], out_dir: Optional[Path], mode: str = 'safe',
                        keep: Iterable[str] = (), write: bool = True) -> List[ParamFileReport]:
    """prepare_param_file() for each file, in order (later files win in SITL, like --defaults a,b,c)."""
    return [prepare_param_file(s, out_dir, mode, keep, write) for s in sources]
