"""
sim_bringup/sim_paths.py -- PURPOSE: find every file, folder and port number the simulator needs, in ONE place.

The launch file (launch/sim.launch.py), the scripts in simulation/scripts and the tests all ask this module
instead of hard-coding paths. It knows:
  * where the per-run state lives (~/.astra_sim: SITL's saved parameters, generated files, demo runs),
  * how a world name like "runway" turns into a world file + SITL home + spawn pose + mission + truth file
    (sim_worlds/config/<world>.yaml), and how a custom .sdf world is handled,
  * where ArduCopter SITL, its default parameter files, MAVProxy and the ArduPilot Gazebo plugin are,
  * the port numbers of a SITL instance (5760+10*I, 9002+10*I, ...),
  * where a ground station on WINDOWS (Mission Planner / QGroundControl) can be reached when the simulator
    runs inside WSL2 (the Windows host's address, looked up at every start because it changes),
  * the two camera profiles (zr10 = 2K photos, a8mini = 4K photos),
  * how to generate the drone model file with xacro, and the stand-ins used when a simulation package
    is not built yet (ardupilot_gazebo's iris_with_gimbal and iris_runway world),
  * which Gazebo topics are bridged to ROS 2 (/sim/camera/image, /sim/camera/photo, ...),
  * where the team's photo folder is (<team ros2_ws>/src/video_cam/mapping_photos).
Everything here is plain Python (no ROS node), so it can be unit-tested with pytest.
"""

import math
import os
import re
import shutil
import socket
import struct
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------------------------
# Fixed numbers of the simulation contract
# ---------------------------------------------------------------------------------------------

#: The address the real SIYI camera has on the drone's network. The team's video_cam node uses it
#: by default, so the emulator answers on it (a loopback alias makes the address exist on this PC).
DEFAULT_CAMERA_IP = '192.168.144.25'

#: Name of the drone entity in Gazebo (the contract: model "iris_a8mini" spawned as entity "iris").
DRONE_ENTITY = 'iris'

#: MAVProxy output ports (UDP, all on 127.0.0.1). 14550 = ground station inside Ubuntu (our scripts, or a
#: QGC/Mission Planner running in Linux), 14552 = MAVROS (the team's launcher.sh sim uses
#: udp://:14552@localhost:14552), 14553 = the demo panel's mission buttons. A ground station on Windows gets
#: its own extra output (see windows_gcs_output below).
DEFAULT_GCS_PORT = 14550
DEFAULT_MAVROS_PORT = 14552
DEFAULT_PANEL_PORT = 14553

#: Live video camera (A8 mini stream) and lens; the same for both camera profiles.
VIDEO_DEFAULTS = {'video_width': 1280, 'video_height': 720, 'video_fps': 25, 'hfov': 1.4137}

#: Photo camera profiles. zr10 is the default because last year's video_cam code falls back to the
#: SIYI ZR10 (2K stills, '2K' validation profile 2400x1300 minimum) and the detector is tuned for 2K.
CAMERA_PROFILES = {
    'zr10': {'photo_width': 2560, 'photo_height': 1440,
             'label': 'ZR10 profile: 2K photos 2560x1440 (last year\'s default camera model)'},
    'a8mini': {'photo_width': 3840, 'photo_height': 2160,
               'label': 'A8 mini profile: 4K photos 3840x2160'},
}

#: The ArduPilot "CMAC" test field: home of ardupilot_gazebo's iris_runway.sdf (used by the stand-in).
CMAC_HOME = '-35.363262,149.165237,584,0'

#: Where SITL and its parameter files can live (native install by 02_setup_ardupilot.sh, or Docker).
SITL_BINARY_CANDIDATES = ('~/sitl/arducopter', '/opt/ardupilot/build/sitl/bin/arducopter')
PARAM_DIR_CANDIDATES = ('~/sitl', '/opt/ardupilot/Tools/autotest/default_params')
ARDUPILOT_GAZEBO_CANDIDATES = ('~/ardupilot_gazebo', '/opt/ardupilot_gazebo')


def truthy(text) -> bool:
    """Return True for 'true', '1', 'yes', 'on' (any case), False for anything else.

    Launch arguments always arrive as strings, so this is how the launch file reads switches.
    """
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def expand(path: str) -> str:
    """Expand ~ and $VARIABLES in a path string (empty string stays empty)."""
    return os.path.expandvars(os.path.expanduser(path)) if path else ''


# ---------------------------------------------------------------------------------------------
# State folders
# ---------------------------------------------------------------------------------------------

def sim_home() -> Path:
    """Folder for everything the simulator writes at run time.

    $ASTRA_SIM_HOME if set, else ~/.astra_sim. Inside it:
      sitl_I<n>/   SITL instance n: eeprom.bin (saved parameters + mission), logs, MAVProxy state
      generated/   drone model, stand-in world and bridge list made for the current launch
      runs/        one folder per demo run (logs, phase log, score report)
      state/       which simulator session is running (used by stop_sim.sh)
      models/      the detector weights for the team's real detector (yolo26m.pt)
    """
    return Path(expand(os.environ.get('ASTRA_SIM_HOME', '') or '~/.astra_sim'))


def sitl_run_dir(instance: int, base: Optional[Path] = None) -> Path:
    """Working folder of SITL instance `instance` (eeprom.bin persists there between runs)."""
    return (base or sim_home()) / f'sitl_I{int(instance)}'


def generated_dir(base: Optional[Path] = None) -> Path:
    """Folder for files generated by the launch file (drone SDF, stand-in world, bridge YAML)."""
    return (base or sim_home()) / 'generated'


# ---------------------------------------------------------------------------------------------
# Ports
# ---------------------------------------------------------------------------------------------

def sitl_ports(instance: int) -> Dict[str, int]:
    """Port numbers of ArduCopter SITL instance I (SITL's -I option adds 10*I to every port).

    Returns a dict:
      serial0  TCP 5760+10I  MAVLink, SITL waits for the first client here (MAVProxy connects)
      serial1  TCP 5762+10I  MAVLink, free for an extra ground station
      serial2  TCP 5763+10I  MAVLink, free for an extra ground station
      fdm      UDP 9002+10I  JSON physics link: the ArduPilotPlugin in Gazebo binds it
      rc_in    UDP 5501+10I  RC input (MAVProxy --sitl), not used by this simulator
    """
    i = int(instance)
    if i < 0:
        raise ValueError(f'SITL instance must be >= 0, got {i}')
    off = 10 * i
    return {'serial0': 5760 + off, 'serial1': 5762 + off, 'serial2': 5763 + off,
            'fdm': 9002 + off, 'rc_in': 5501 + off}


# ---------------------------------------------------------------------------------------------
# A ground station on Windows (Mission Planner / QGroundControl) while the simulator runs in WSL2
# ---------------------------------------------------------------------------------------------

#: The UDP port Mission Planner and QGroundControl listen on when you pick a "UDP" connection.
DEFAULT_WINDOWS_GCS_PORT = 14550

#: The Windows ground station's port in WSL "mirrored" networking. There Windows and Ubuntu share 127.0.0.1,
#: and 14550 on it already belongs to our scripts' ground station (mission_tool listens on it). Two programs
#: cannot listen on one port, so Mission Planner gets the next number instead.
MIRRORED_WINDOWS_GCS_PORT = 14551


def running_in_wsl(proc_version: Optional[str] = None) -> bool:
    """True inside Windows' WSL2: its kernel version string (/proc/version) mentions Microsoft.

    proc_version: the text to check (the tests pass their own); default = read /proc/version now.
    """
    if proc_version is None:
        try:
            proc_version = Path('/proc/version').read_text(errors='ignore')
        except OSError:
            return False
    return 'microsoft' in proc_version.lower()


def wsl_networking_mode() -> str:
    """How WSL2 connects Ubuntu to Windows: 'nat' (the default), 'mirrored', ...; '' outside WSL.

    Asks `wslinfo --networking-mode` (WSL 2.0 and newer). Without that tool the 'loopback0' network
    interface decides, because only mirrored mode has it.
    """
    if not running_in_wsl():
        return ''
    try:
        out = subprocess.run(['wslinfo', '--networking-mode'], capture_output=True, text=True, timeout=5,
                             check=False)
        mode = out.stdout.strip().lower()
        if out.returncode == 0 and mode:
            return mode
    except (OSError, subprocess.SubprocessError):
        pass
    return 'mirrored' if Path('/sys/class/net/loopback0').exists() else 'nat'


def default_gateway(route_table: Optional[str] = None) -> str:
    """IPv4 address of this machine's default gateway, '' when there is none.

    In WSL2's NAT networking the default gateway IS the Windows host (its "vEthernet (WSL)" adapter), so this
    is where Mission Planner on Windows can be reached. Windows gives that adapter a new address after a
    restart, which is why the launch file looks it up at every start instead of storing it.
    route_table: the text of /proc/net/route (the tests pass their own); default = read it now. In that file
    the default route has Destination 00000000 and the addresses are little-endian hexadecimal numbers.
    """
    if route_table is None:
        try:
            route_table = Path('/proc/net/route').read_text()
        except OSError:
            return ''
    best = None                                     # (metric, address) of the best default route so far
    for line in route_table.splitlines()[1:]:       # the first line holds the column names
        f = line.split()                            # Iface Destination Gateway Flags RefCnt Use Metric ...
        if len(f) < 7 or f[1] != '00000000':
            continue
        try:
            flags, metric = int(f[3], 16), int(f[6])
            gateway = socket.inet_ntoa(struct.pack('<I', int(f[2], 16)))
        except (ValueError, struct.error, OSError):
            continue
        # 0x1 = the route is up, 0x2 = it goes through a gateway; with several, the lowest metric wins.
        if (flags & 0x3) == 0x3 and (best is None or metric < best[0]):
            best = (metric, gateway)
    return best[1] if best else ''


def parse_host_port(text: str, default_port: int) -> Tuple[str, int]:
    """Split 'HOST' or 'HOST:PORT' (e.g. '192.168.1.20' or '192.168.1.20:14550') into (host, port).

    Raises ValueError with a plain-words message for an empty host, a host with spaces or extra colons
    (MAVProxy's udp outputs take an IPv4 address or a name only) or a port outside 1-65535.
    """
    text = str(text).strip()
    host, sep, port_text = text.partition(':')
    host = host.strip()
    if not host or ' ' in host or ':' in port_text:
        raise ValueError(f'"{text}" is not HOST or HOST:PORT (for example 192.168.1.20:14550)')
    try:
        port = int(port_text) if sep else int(default_port)
    except ValueError:
        raise ValueError(f'"{text}": the port must be a whole number') from None
    if not 1 <= port <= 65535:
        raise ValueError(f'"{text}": the port must be between 1 and 65535')
    return host, port


def windows_gcs_output(enabled, address: str = '', local_ports: Sequence[int] = (),
                       mode: Optional[str] = None, gateway: Optional[str] = None) -> Tuple[str, List[str]]:
    """The extra MAVProxy output for a ground station on WINDOWS: ('udp:HOST:PORT', notes), or ('', notes).

    Why it is needed: in WSL2's default NAT networking Ubuntu is like a separate small computer behind
    Windows, so Mission Planner / QGroundControl on Windows never hear MAVProxy's 127.0.0.1 outputs. MAVProxy
    therefore sends one more copy of the drone's MAVLink to the Windows host itself. The ground station's
    answers come back the same way, so it can read parameters and send commands like on the real drone.
      enabled      the launch argument windows_gcs ('true' / 'false'); false = no extra output.
      address      windows_gcs_addr: '' = automatic; 'HOST' or 'HOST:PORT' = used as given, also outside WSL
                   (e.g. a teammate's laptop on the same network: 192.168.1.20:14550).
      local_ports  MAVProxy's own UDP outputs on 127.0.0.1 (ground station, MAVROS, panel): never doubled.
      mode         WSL networking mode, for the tests ('nat', 'mirrored', '' = not WSL); default = ask WSL now.
      gateway      default gateway, for the tests; default = read /proc/net/route now.
    Automatic choice: WSL NAT -> the default gateway (= Windows), port 14550; WSL mirrored -> 127.0.0.1:14551
    (Windows shares 127.0.0.1 with Ubuntu there, and 14550 is our scripts' port); not WSL -> nothing.
    The notes are plain-words lines for the launch log: what was chosen and why.
    """
    if not truthy(enabled):
        return '', ['windows_gcs:=false: no MAVLink copy for a ground station on Windows']
    if str(address).strip():
        try:
            host, port = parse_host_port(address, DEFAULT_WINDOWS_GCS_PORT)
        except ValueError as exc:
            return '', [f'windows_gcs_addr {exc}: no MAVLink copy for a ground station on Windows']
        why = 'windows_gcs_addr'
    else:
        mode = wsl_networking_mode() if mode is None else mode
        if not mode:
            return '', ['not running inside WSL: no automatic Windows ground station output '
                        '(windows_gcs_addr:=HOST:PORT sends a copy to any computer)']
        if mode == 'none':
            return '', ['WSL networking is switched off (networkingMode=none): no Windows ground station output']
        if mode == 'mirrored':
            host, port = '127.0.0.1', MIRRORED_WINDOWS_GCS_PORT
            why = ('WSL mirrored networking: Windows shares 127.0.0.1 with Ubuntu, and 14550 there is our '
                   'scripts\' port')
        else:
            host = default_gateway() if gateway is None else gateway
            if not host:
                return '', ['could not find the Windows host (no default route in /proc/net/route): no Windows '
                            'ground station output. Give it with windows_gcs_addr:=IP:14550']
            port = DEFAULT_WINDOWS_GCS_PORT
            why = f'the Windows host = the default gateway, WSL {mode.upper()} networking'
    if host in ('127.0.0.1', 'localhost') and port in {int(p) for p in local_ports}:
        return '', [f'Windows ground station {host}:{port} is one of MAVProxy\'s own outputs, so it is not '
                    'doubled: give windows_gcs_addr another port']
    out = f'udp:{host}:{port}'
    qgc = ('QGroundControl connects by itself' if port == DEFAULT_WINDOWS_GCS_PORT
           else f'QGroundControl: add a UDP comm link on port {port}')
    return out, [f'Windows ground station: MAVLink also goes to {out} ({why}). Mission Planner on Windows: '
                 f'connection type UDP -> CONNECT -> port {port}. {qgc}']


# ---------------------------------------------------------------------------------------------
# Worlds
# ---------------------------------------------------------------------------------------------

@dataclass
class WorldConfig:
    """Everything the launch file needs to know about one world.

    Attributes:
      name        short name ('runway', 'suas_field', or the file stem of a custom .sdf)
      world_file  absolute path of the Gazebo world (.sdf), '' if it could not be found
      home        SITL --home string "lat,lon,alt_msl_m,heading_deg" (must match the world's
                  <spherical_coordinates> so SITL's GPS and the Gazebo picture agree)
      spawn       drone spawn pose {'x','y','z' metres, 'yaw' radians} in the world (ENU) frame
      mission     absolute path of the survey mission (QGC WPL 110 .waypoints), '' if none
      truth       absolute path of the ground-truth YAML (target positions), '' if none
      config_file the YAML this came from ('' for a custom .sdf)
      worlds_dir  the sim_worlds folder the YAML belongs to (installed share, or the source folder when a
                  config YAML is given as a path); its models/ and worlds/ go on Gazebo's resource path
      standin     True when the stand-in world from ardupilot_gazebo is used
      notes       human-readable remarks collected while resolving (printed by the launch file)
    """
    name: str
    world_file: str = ''
    home: str = CMAC_HOME
    spawn: Dict[str, float] = field(default_factory=lambda: {'x': 0.0, 'y': 0.0, 'z': 0.2, 'yaw': 0.0})
    mission: str = ''
    truth: str = ''
    config_file: str = ''
    worlds_dir: str = ''
    standin: bool = False
    notes: List[str] = field(default_factory=list)


def parse_home(text: str) -> Tuple[float, float, float, float]:
    """Parse and check a SITL home string "lat,lon,alt_msl,heading".

    Returns (lat_deg, lon_deg, alt_m, heading_deg). Raises ValueError with a plain-words message
    when the text is malformed or out of range.
    """
    parts = [p.strip() for p in str(text).split(',')]
    if len(parts) != 4:
        raise ValueError(f'home "{text}" must be "lat,lon,alt_msl,heading_deg" (4 numbers)')
    try:
        lat, lon, alt, hdg = (float(p) for p in parts)
    except ValueError:
        raise ValueError(f'home "{text}" contains something that is not a number') from None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError(f'home "{text}": latitude/longitude out of range')
    return lat, lon, alt, hdg


def format_home(lat: float, lon: float, alt: float, heading: float = 0.0) -> str:
    """Build a SITL home string from numbers (inverse of parse_home)."""
    return f'{lat:.7f},{lon:.7f},{alt:.2f},{heading:g}'


_SPHERICAL_TAGS = ('latitude_deg', 'longitude_deg', 'elevation', 'heading_deg')


def read_spherical_coordinates(sdf_text: str) -> Optional[Tuple[float, float, float, float]]:
    """Read <spherical_coordinates> (lat, lon, elevation, heading_deg) from a world SDF text.

    Returns None when the world has no spherical coordinates. Used for custom worlds so that SITL's
    --home automatically matches the world (the GPS position and the Gazebo picture then agree).
    """
    block = re.search(r'<spherical_coordinates>(.*?)</spherical_coordinates>', sdf_text, re.S)
    if not block:
        return None
    values = []
    for tag in _SPHERICAL_TAGS:
        m = re.search(rf'<{tag}>\s*([-+0-9.eE]+)\s*</{tag}>', block.group(1))
        values.append(float(m.group(1)) if m else 0.0)
    return tuple(values)  # type: ignore[return-value]


def world_name_from_sdf(sdf_text: str, default: str = 'default') -> str:
    """Return the name attribute of the first <world name="..."> in an SDF text."""
    m = re.search(r'<world\s+name\s*=\s*["\']([^"\']+)["\']', sdf_text)
    return m.group(1) if m else default


def _read_spawn(raw, notes: List[str]) -> Dict[str, float]:
    """Turn the 'spawn' section of a world YAML into {'x','y','z','yaw'} (yaw in radians).

    Accepts 'yaw' (radians) or 'yaw_deg' (degrees). Missing keys get safe defaults and a note.
    """
    spawn = {'x': 0.0, 'y': 0.0, 'z': 0.2, 'yaw': 0.0}
    if not isinstance(raw, dict):
        if raw is not None:
            notes.append('spawn section is not a {x, y, z, yaw} mapping; spawning at the origin')
        return spawn
    for key in ('x', 'y', 'z'):
        if raw.get(key) is not None:
            spawn[key] = float(raw[key])
    if raw.get('yaw') is not None:
        spawn['yaw'] = float(raw['yaw'])
    elif raw.get('yaw_deg') is not None:
        spawn['yaw'] = math.radians(float(raw['yaw_deg']))
    return spawn


def _resolve_rel(path_text: str, roots: Sequence[Path]) -> str:
    """Resolve a path from a world YAML: absolute stays, relative is tried against each root."""
    if not path_text:
        return ''
    p = Path(expand(str(path_text)))
    if p.is_absolute():
        return str(p) if p.exists() else ''
    for root in roots:
        cand = root / p
        if cand.exists():
            return str(cand)
    return ''


def load_world_config(world: str, worlds_share: Optional[str]) -> WorldConfig:
    """Turn the launch argument world:= into a WorldConfig.

    Args:
      world:        'runway', 'suas_field' (any name that has sim_worlds/config/<name>.yaml),
                    a path to a world .sdf, or a path to a world config .yaml.
      worlds_share: the installed share folder of sim_worlds, or None when sim_worlds is not built.
    Returns a WorldConfig; problems are written into .notes (never raises for a missing file, so the
    launch file can print a clear message instead of a Python traceback).
    """
    world = (world or 'runway').strip()
    share = Path(worlds_share) if worlds_share else None

    # 1) a custom world file
    if world.endswith('.sdf'):
        sdf = Path(expand(world)).resolve()
        cfg = WorldConfig(name=sdf.stem, world_file=str(sdf) if sdf.is_file() else '')
        if not sdf.is_file():
            cfg.notes.append(f'world file not found: {sdf}')
            return cfg
        sc = read_spherical_coordinates(sdf.read_text(encoding='utf-8', errors='replace'))
        if sc:
            cfg.home = format_home(*sc)
            cfg.notes.append(f'custom world: SITL home taken from its <spherical_coordinates> ({cfg.home})')
        else:
            cfg.notes.append('custom world has no <spherical_coordinates>: SITL uses the CMAC home; '
                             'GPS positions will not match the picture')
        cfg.notes.append('custom world: no mission/truth files (pass them to the scripts by hand)')
        return cfg

    # 2) a world config YAML (given as a path, or found by name in sim_worlds/config)
    if world.endswith(('.yaml', '.yml')):
        yaml_path = Path(expand(world)).resolve()
    elif share is not None:
        yaml_path = share / 'config' / f'{world}.yaml'
    else:
        cfg = WorldConfig(name=world)
        cfg.notes.append('package sim_worlds is not built: no world config for '
                         f'"{world}" (build it with simulation/setup/03_build.sh)')
        return cfg

    cfg = WorldConfig(name=yaml_path.stem, config_file=str(yaml_path),
                      worlds_dir=str(share) if share is not None and not world.endswith(('.yaml', '.yml'))
                      else str(yaml_path.parent.parent))
    if not yaml_path.is_file():
        cfg.notes.append(f'world config not found: {yaml_path}')
        return cfg
    try:
        import yaml  # local import: keeps this module importable without PyYAML for basic helpers
        data = yaml.safe_load(yaml_path.read_text(encoding='utf-8')) or {}
    except Exception as exc:  # noqa: BLE001 - any YAML error becomes a note, not a crash
        cfg.notes.append(f'could not read {yaml_path}: {exc}')
        return cfg
    if not isinstance(data, dict):
        cfg.notes.append(f'{yaml_path} does not contain a mapping')
        return cfg

    # Relative paths in the YAML: a YAML given as a path resolves against its own folder first.
    if world.endswith(('.yaml', '.yml')):
        roots = [p for p in (yaml_path.parent.parent, yaml_path.parent, share) if p is not None]
    else:
        roots = [p for p in (share, yaml_path.parent.parent, yaml_path.parent) if p is not None]
    cfg.world_file = _resolve_rel(str(data.get('world_file', '') or ''), roots)
    if not cfg.world_file:
        cfg.notes.append(f'world_file "{data.get("world_file")}" from {yaml_path.name} not found')
    if data.get('home'):
        try:
            parse_home(str(data['home']))
            cfg.home = str(data['home']).replace(' ', '')
        except ValueError as exc:
            cfg.notes.append(f'{exc}; using the world file instead')
            data['home'] = None
    if not data.get('home') and cfg.world_file:
        sc = read_spherical_coordinates(Path(cfg.world_file).read_text(encoding='utf-8', errors='replace'))
        if sc:
            cfg.home = format_home(*sc)
    cfg.spawn = _read_spawn(data.get('spawn'), cfg.notes)
    cfg.mission = _resolve_rel(str(data.get('mission', '') or ''), roots)
    if data.get('mission') and not cfg.mission:
        cfg.notes.append(f'mission file "{data.get("mission")}" not found')
    cfg.truth = _resolve_rel(str(data.get('truth', '') or ''), roots)
    if data.get('truth') and not cfg.truth:
        cfg.notes.append(f'truth file "{data.get("truth")}" not found')
    return cfg


def standin_world_config(ardupilot_gazebo_dir: Optional[str], out_dir: Path) -> WorldConfig:
    """World used when sim_worlds is not built: a copy of ardupilot_gazebo's iris_runway.sdf.

    The copy has the built-in iris_with_gimbal removed, because the launch file spawns its own drone
    (named "iris", with the right SITL port). Home = CMAC, spawn = where iris_runway puts its drone.
    """
    cfg = WorldConfig(name='iris_runway', home=CMAC_HOME, standin=True,
                      spawn={'x': 0.0, 'y': 0.0, 'z': 0.195, 'yaw': math.pi / 2})
    if not ardupilot_gazebo_dir:
        cfg.notes.append('stand-in world unavailable: ardupilot_gazebo not found')
        return cfg
    src = Path(ardupilot_gazebo_dir) / 'worlds' / 'iris_runway.sdf'
    if not src.is_file():
        cfg.notes.append(f'stand-in world unavailable: {src} missing')
        return cfg
    out_dir.mkdir(parents=True, exist_ok=True)
    text = src.read_text(encoding='utf-8')
    text = re.sub(r'<include>\s*<uri>\s*model://iris_with_gimbal\s*</uri>.*?</include>', '', text, flags=re.S)
    out = out_dir / 'standin_iris_runway.sdf'
    out.write_text(text, encoding='utf-8')
    cfg.world_file = str(out)
    cfg.notes.append('STAND-IN world: ardupilot_gazebo iris_runway (no targets, no mission). '
                     'Build sim_worlds for the real runway / suas_field worlds.')
    return cfg


# ---------------------------------------------------------------------------------------------
# ArduPilot pieces
# ---------------------------------------------------------------------------------------------

def _first_existing(candidates: Sequence[str], want_exec: bool = False) -> Optional[str]:
    """Return the first candidate path that exists (and is executable when want_exec)."""
    for cand in candidates:
        if not cand:
            continue
        p = Path(expand(cand))
        if p.exists() and (not want_exec or os.access(p, os.X_OK)):
            return str(p)
    return None


def find_sitl_binary(explicit: str = '') -> Optional[str]:
    """Find ArduCopter SITL: explicit path, $ARDUPILOT_SITL_BIN, ~/sitl/arducopter, Docker path."""
    return _first_existing([explicit, os.environ.get('ARDUPILOT_SITL_BIN', ''),
                            *SITL_BINARY_CANDIDATES], want_exec=True)


def find_param_file(name: str) -> Optional[str]:
    """Find one ArduPilot default parameter file (e.g. 'copter.parm', 'gazebo-iris.parm').

    Looks in $ARDUPILOT_DEFAULT_PARAMS, ~/sitl (native setup) and the ArduPilot source tree (Docker).
    """
    dirs = [os.environ.get('ARDUPILOT_DEFAULT_PARAMS', ''), *PARAM_DIR_CANDIDATES]
    return _first_existing([str(Path(expand(d)) / name) for d in dirs if d])


def sitl_defaults(astra_parm: Optional[str]) -> Tuple[List[str], List[str]]:
    """The --defaults list for SITL, in the contract's order: copter.parm, gazebo-iris.parm, astra_sitl.parm.

    Later files win (ArduPilot reads them left to right). Returns (existing_files, notes).
    """
    files, notes = [], []
    for name in ('copter.parm', 'gazebo-iris.parm'):
        found = find_param_file(name)
        if found:
            files.append(found)
        else:
            notes.append(f'ArduPilot parameter file {name} not found (run simulation/setup/02_setup_ardupilot.sh)')
    if astra_parm and Path(astra_parm).is_file():
        files.append(astra_parm)
    else:
        notes.append('sim_models/config/astra_sitl.parm not found: payload servos 11/13 are not set up')
    return files, notes


def find_ardupilot_gazebo() -> Optional[str]:
    """Find the ArduPilot Gazebo plugin folder ($ARDUPILOT_GAZEBO_DIR, ~/ardupilot_gazebo, /opt/...).

    A folder counts only when it holds build/libArduPilotPlugin.so (the plugin is built).
    """
    for cand in (os.environ.get('ARDUPILOT_GAZEBO_DIR', ''), *ARDUPILOT_GAZEBO_CANDIDATES):
        if cand and (Path(expand(cand)) / 'build' / 'libArduPilotPlugin.so').is_file():
            return str(Path(expand(cand)))
    return None


def find_mavproxy(explicit: str = '') -> Optional[str]:
    """Find MAVProxy: explicit path, $ASTRA_MAVPROXY, ~/sitl/venv (native setup), then the PATH (Docker)."""
    found = _first_existing([explicit, os.environ.get('ASTRA_MAVPROXY', ''),
                             '~/sitl/venv/bin/mavproxy.py'], want_exec=True)
    return found or shutil.which('mavproxy.py')


# ---------------------------------------------------------------------------------------------
# Drone model
# ---------------------------------------------------------------------------------------------

def patch_fdm_port(sdf_text: str, port: int) -> str:
    """Set every <fdm_port_in> of an SDF text to `port` (SITL instance I needs 9002 + 10*I)."""
    return re.sub(r'<fdm_port_in>\s*\d+\s*</fdm_port_in>', f'<fdm_port_in>{int(port)}</fdm_port_in>', sdf_text)


def xacro_mappings(camera_model: str, instance: int) -> Dict[str, str]:
    """The xacro arguments of sim_models' model.sdf.xacro for a camera profile and SITL instance."""
    profile = CAMERA_PROFILES.get(camera_model)
    if profile is None:
        raise ValueError(f'camera_model must be one of {sorted(CAMERA_PROFILES)}, got "{camera_model}"')
    maps = {k: str(v) for k, v in VIDEO_DEFAULTS.items()}
    maps.update({'photo_width': str(profile['photo_width']), 'photo_height': str(profile['photo_height']),
                 'fdm_port_in': str(sitl_ports(instance)['fdm'])})
    return maps


def _mirror_model_folder(src_dir: Path, dst_dir: Path) -> None:
    """Make dst_dir look like src_dir (symlinks to meshes/, materials/, model.config, ...).

    The generated model.sdf is written into dst_dir; relative mesh paths inside it then still work.
    """
    dst_dir.mkdir(parents=True, exist_ok=True)
    for entry in src_dir.iterdir():
        if entry.name.startswith('model.sdf'):
            continue
        link = dst_dir / entry.name
        if link.is_symlink() or link.exists():
            if link.is_symlink() and os.path.realpath(link) == os.path.realpath(entry):
                continue
            if link.is_dir() and not link.is_symlink():
                shutil.rmtree(link)
            else:
                link.unlink()
        os.symlink(entry, link)


def generate_drone_sdf(models_share: Optional[str], camera_model: str, instance: int,
                       out_root: Path, explicit_model: str = '') -> Tuple[str, List[str]]:
    """Produce the drone model file to spawn and return (path, notes). '' path = nothing to spawn.

    Order of preference:
      1. explicit_model (model:= launch argument): an .sdf is used as-is (fdm port patched),
         an .xacro is run through xacro with the profile arguments;
      2. sim_models/models/iris_a8mini/model.sdf.xacro, run through xacro with photo_width/height
         (from camera_model), video size/fps, hfov and fdm_port_in;
      3. the pre-generated sim_models/models/iris_a8mini/model.sdf (fdm port patched; camera size is
         whatever it was generated with, normally 2560x1440);
    The result goes to out_root/iris_a8mini_<camera>_I<n>/model.sdf next to symlinks of the model's
    other files, so relative mesh paths keep working.
    """
    notes: List[str] = []
    port = sitl_ports(instance)['fdm']
    maps = xacro_mappings(camera_model, instance)

    candidates: List[Path] = []
    if explicit_model:
        candidates.append(Path(expand(explicit_model)))
    if models_share:
        mdir = Path(models_share) / 'models' / 'iris_a8mini'
        candidates += [mdir / 'model.sdf.xacro', mdir / 'model.sdf']
    src = next((c for c in candidates if c.is_file()), None)
    if src is None:
        if explicit_model:
            notes.append(f'model file not found: {explicit_model}')
        return '', notes

    out_dir = out_root / f'{src.parent.name}_{camera_model}_I{int(instance)}'
    _mirror_model_folder(src.parent, out_dir)
    out = out_dir / 'model.sdf'
    if src.name.endswith('.xacro'):
        try:
            import xacro  # provided by ros-humble-xacro
            doc = xacro.process_file(str(src), mappings=maps)
            text = doc.toprettyxml(indent='  ')
        except Exception as exc:  # noqa: BLE001 - report and fall back
            notes.append(f'xacro failed on {src}: {exc}')
            plain = src.with_name('model.sdf')
            if not plain.is_file():
                return '', notes
            notes.append(f'falling back to the pre-generated {plain} (default camera size)')
            text = plain.read_text(encoding='utf-8')
    else:
        text = src.read_text(encoding='utf-8')
        if src.parent.name == 'iris_a8mini' and camera_model != 'zr10':
            notes.append('using the pre-generated model.sdf: the photo size stays at its default')
    out.write_text(patch_fdm_port(text, port), encoding='utf-8')
    notes.append(f'drone model {src.name} -> {out} (camera {camera_model}, SITL port {port})')
    return str(out), notes


def standin_drone_sdf(ardupilot_gazebo_dir: Optional[str], instance: int, out_root: Path) -> Tuple[str, List[str]]:
    """Drone used when sim_models is not built: ardupilot_gazebo's iris_with_gimbal (fdm port patched).

    It has a 3-axis servo gimbal and a 640x480 camera, no photo camera and no payloads.
    """
    if not ardupilot_gazebo_dir:
        return '', ['stand-in drone unavailable: ardupilot_gazebo not found']
    src = Path(ardupilot_gazebo_dir) / 'models' / 'iris_with_gimbal' / 'model.sdf'
    if not src.is_file():
        return '', [f'stand-in drone unavailable: {src} missing']
    out_dir = out_root / f'standin_iris_with_gimbal_I{int(instance)}'
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / 'model.sdf'
    out.write_text(patch_fdm_port(src.read_text(encoding='utf-8'), sitl_ports(instance)['fdm']), encoding='utf-8')
    config = src.parent / 'model.config'
    if config.is_file():
        shutil.copyfile(config, out_dir / 'model.config')
    return str(out), ['STAND-IN drone: ardupilot_gazebo iris_with_gimbal (no photo camera, no payloads). '
                      'Build sim_models for the Iris + A8 mini model.']


# ---------------------------------------------------------------------------------------------
# ROS <-> Gazebo bridge
# ---------------------------------------------------------------------------------------------

def bridge_entries(standin: bool = False, world_name: str = '', entity: str = DRONE_ENTITY) -> List[Dict]:
    """The list of topics ros_gz_bridge's parameter_bridge carries (all Gazebo -> ROS).

    Real model (contract topics):
      /sim/camera/video/image        -> /sim/camera/image        live POV (sensor_msgs/Image)
      /sim/camera/video/camera_info  -> /sim/camera/camera_info  lens of the live camera
      /sim/camera/photo/image        -> /sim/camera/photo        each rendered photo (one per trigger)
      /sim/drone/odometry            -> /sim/drone/odometry      drone pose, world ENU (nav_msgs/Odometry)
      /sim/gimbal/joint_states       -> /sim/gimbal/joint_states gimbal angles (sensor_msgs/JointState)
    Stand-in drone: ardupilot_gazebo's gimbal camera topic -> /sim/camera/image (+ camera_info).
    Camera topics are 'lazy': they are only converted while somebody watches them.
    """
    def entry(gz, ros, ros_type, gz_type, lazy=False):
        return {'ros_topic_name': ros, 'gz_topic_name': gz, 'ros_type_name': ros_type,
                'gz_type_name': gz_type, 'direction': 'GZ_TO_ROS', 'lazy': lazy}

    img, info = ('sensor_msgs/msg/Image', 'gz.msgs.Image'), ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')
    if standin:
        base = f'/world/{world_name}/model/{entity}/model/gimbal/link/pitch_link/sensor/camera'
        return [entry(f'{base}/image', '/sim/camera/image', *img, lazy=True),
                entry(f'{base}/camera_info', '/sim/camera/camera_info', *info, lazy=True)]
    return [
        entry('/sim/camera/video/image', '/sim/camera/image', *img, lazy=True),
        entry('/sim/camera/video/camera_info', '/sim/camera/camera_info', *info, lazy=True),
        entry('/sim/camera/photo/image', '/sim/camera/photo', *img, lazy=True),
        entry('/sim/drone/odometry', '/sim/drone/odometry', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        entry('/sim/gimbal/joint_states', '/sim/gimbal/joint_states',
              'sensor_msgs/msg/JointState', 'gz.msgs.Model'),
    ]


def write_bridge_yaml(entries: List[Dict], out_path: Path) -> str:
    """Write a parameter_bridge config file (YAML list of topic entries) and return its path."""
    import yaml
    out_path.parent.mkdir(parents=True, exist_ok=True)
    header = ('# Generated by sim_bringup (sim_paths.write_bridge_yaml) for ros_gz_bridge parameter_bridge.\n'
              '# Each entry copies one Gazebo topic into ROS 2. Do not edit: it is rewritten at every launch.\n')
    out_path.write_text(header + yaml.safe_dump(entries, sort_keys=False), encoding='utf-8')
    return str(out_path)


# ---------------------------------------------------------------------------------------------
# The team's folders and the detector model
# ---------------------------------------------------------------------------------------------

def default_repo_dir() -> Path:
    """The Linux-side copy of the team repo ($ASTRA_REPO, else ~/ODLC_Machine_Inferencing_System_2025-2026)."""
    return Path(expand(os.environ.get('ASTRA_REPO', '') or '~/ODLC_Machine_Inferencing_System_2025-2026'))


def team_photo_dir(override: str = '') -> str:
    """The folder where the team's video_cam saves photos and new_od looks for them.

    Follows video_cam's own rule (video_cam.storage_manager.get_ros2_ws_directory, which video_cam,
    detection and main_controller all use): walk up from where the video_cam package is installed to
    the folder that holds both install/ and src/, then src/video_cam/mapping_photos. The video_cam module
    is NOT imported here on purpose: it imports OpenCV, and OpenCV's pip wheel sets
    QT_QPA_PLATFORM_PLUGIN_PATH to its own Qt plugins, which makes every Qt window started afterwards
    (Gazebo, RViz, the demo panel) abort. Falls back to the Linux-side repo copy.
    """
    if override:
        return expand(override)
    try:
        from ament_index_python.packages import get_package_prefix
        here = Path(get_package_prefix('video_cam')).resolve()
        for folder in [here, *here.parents][:10]:
            if (folder / 'install').is_dir() and (folder / 'src').is_dir():
                return str(folder / 'src' / 'video_cam' / 'mapping_photos')
    except Exception:  # noqa: BLE001 - team stack not built / not sourced: fall back quietly
        pass
    return str(default_repo_dir() / 'ros2_ws' / 'src' / 'video_cam' / 'mapping_photos')


def scrub_foreign_qt_env(environ=None) -> List[str]:
    """Remove Qt plugin variables that point into a Python wheel (e.g. OpenCV's cv2/qt/plugins).

    OpenCV's pip package sets QT_QPA_PLATFORM_PLUGIN_PATH / QT_QPA_FONTDIR when it is imported; system Qt
    programs that inherit them load the wrong plugins and abort ("Could not load the Qt platform plugin
    xcb ... even though it was found"). Returns the names that were removed.
    """
    env = os.environ if environ is None else environ
    removed = []
    for name in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_QPA_FONTDIR', 'QT_PLUGIN_PATH'):
        value = env.get(name, '')
        if value and ('site-packages' in value or 'dist-packages' in value):
            env.pop(name, None)
            removed.append(name)
    return removed


def team_detection_dir(photo_dir: str) -> str:
    """The detector's annotated-image folder next to the photo folder (detection_results_sahi)."""
    return str(Path(photo_dir).parent / 'detection_results_sahi')


def detector_model_path() -> str:
    """Weights file for the team's real detector in the simulator ($ASTRA_DETECTOR_MODEL or
    ~/.astra_sim/models/yolo26m.pt). Kept outside the repo so no Jetson-only .engine sits next to it
    (new_od would otherwise try to load that TensorRT file first)."""
    return expand(os.environ.get('ASTRA_DETECTOR_MODEL', '') or str(sim_home() / 'models' / 'yolo26m.pt'))


# ---------------------------------------------------------------------------------------------
# Small command line, so shell scripts can ask the same questions:
#   python3 -m sim_bringup.sim_paths world runway mission     -> path of the runway survey mission
#   python3 -m sim_bringup.sim_paths world suas_field truth   -> path of the truth file
#   python3 -m sim_bringup.sim_paths photo-dir | model-path | sim-home | sitl-ports 0
# Prints the answer (empty line when unknown) and exits 0; exits 2 on bad usage.
# ---------------------------------------------------------------------------------------------

def _cli(argv: Sequence[str]) -> int:
    """Answer one question for a shell script (see the comment above)."""
    if not argv:
        print('usage: python3 -m sim_bringup.sim_paths world NAME FIELD | photo-dir | model-path | sim-home | sitl-ports I')
        return 2
    what = argv[0]
    if what == 'world' and len(argv) == 3:
        share = None
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('sim_worlds')
        except Exception:  # noqa: BLE001 - sim_worlds not built
            share = None
        cfg = load_world_config(argv[1], share)
        value = {'mission': cfg.mission, 'truth': cfg.truth, 'world_file': cfg.world_file, 'home': cfg.home,
                 'name': cfg.name, 'config': cfg.config_file, 'worlds_dir': cfg.worlds_dir}.get(argv[2])
        if value is None:
            print(f'unknown field {argv[2]} (mission, truth, world_file, home, name, config, worlds_dir)')
            return 2
        print(value)
        return 0
    if what == 'photo-dir':
        print(team_photo_dir())
    elif what == 'model-path':
        print(detector_model_path())
    elif what == 'sim-home':
        print(sim_home())
    elif what == 'sitl-ports' and len(argv) == 2:
        print(' '.join(f'{k}={v}' for k, v in sitl_ports(int(argv[1])).items()))
    else:
        print(f'unknown question: {" ".join(argv)}')
        return 2
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_cli(sys.argv[1:]))
