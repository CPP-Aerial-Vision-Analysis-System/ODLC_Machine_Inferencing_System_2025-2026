"""
sim_plugins/autopilot.py -- PURPOSE: the AUTOPILOT slot. Decide which ArduCopter runs the simulated drone
and with which parameter files, from either a profile (the scripts' plan) or sim.launch.py's arguments.

The ways (profile key autopilot.kind; launch arguments autopilot / sitl_version / sitl_bin / sitl_source):

  default       ~/sitl/arducopter (ArduCopter 4.6.3, installed by simulation/setup/02_setup_ardupilot.sh),
                found exactly like before: $ARDUPILOT_SITL_BIN, ~/sitl/arducopter, Docker's /opt/ardupilot.
  version       ~/sitl/versions/<VERSION>/arducopter, downloaded by simulation/setup/fetch_sitl.sh VERSION
                (4.6.3, 4.7.1, latest ...), with that version's copter.parm / gazebo-iris.parm next to it.
  binary        any arducopter program you give by path (a teammate's build, a CI artifact ...).
  source        an ArduPilot source tree (Docker's /opt/ardupilot, ~/ardupilot): its build/sitl/bin/arducopter
                and its Tools/autotest/default_params.
  external      no SITL here: an autopilot running in another container / on another PC. MAVProxy picks
                up its MAVLink (mavlink_master), and the drone's ArduPilot plugin binds its physics port on
                fdm_addr (0.0.0.0 = every network) so the remote SITL can reach it. (The plugin answers to
                whoever sent the last packet, so only the remote SITL needs to know this computer's address:
                arducopter --sim-address=<this computer>.)

On top of any local SITL: extra parameter files (the real drone's Mission Planner .param), cleaned by
params.py, loaded AFTER astra_sitl.parm.

Run folders (SITL's working folder: eeprom.bin = its saved parameters and mission, logs, MAVProxy state):
every flavour other than the default gets its OWN folder (~/.astra_sim/sitl_I<I>_<tag>), so the saved
parameters of one version never leak into another. The default keeps ~/.astra_sim/sitl_I<I>, and so does
the old launch argument sitl_bin:=PATH on its own (it existed before profiles; its behaviour is unchanged).
A profile's autopilot.binary gets its own folder through an explicit sitl_run_dir (to_launch_args).
"""

from __future__ import annotations

import hashlib
import os
import re
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

from . import params as params_mod
from . import paths

#: Same search order as sim_bringup/sim_paths.py (used when sim_bringup cannot be imported).
SITL_BINARY_CANDIDATES = ('~/sitl/arducopter', '/opt/ardupilot/build/sitl/bin/arducopter')
PARAM_DIR_CANDIDATES = ('~/sitl', '/opt/ardupilot/Tools/autotest/default_params')
STOCK_PARAM_FILES = ('copter.parm', 'gazebo-iris.parm')

#: The launch arguments of sim.launch.py that belong to this slot, with their defaults (sitl_bin and
#: sitl_wipe existed before profiles; the others were added for the plug-and-play slots).
LAUNCH_DEFAULTS = OrderedDict([
    ('autopilot', 'sitl'), ('sitl_version', ''), ('sitl_bin', ''), ('sitl_source', ''),
    ('sitl_param_files', ''), ('sitl_param_filter', 'safe'), ('sitl_param_keep', ''), ('sitl_run_dir', ''),
    ('sitl_wipe', 'false'), ('mavlink_master', ''), ('fdm_addr', ''),
])

#: MAVProxy --retries for a TCP master in external mode: about one try per second, so ~5 minutes for the
#: remote autopilot to come up (MAVProxy's own default is 3 tries, then it exits).
EXTERNAL_TCP_RETRIES = 300

_VERSION_IN_BINARY = re.compile(rb'ArduCopter V(\d+\.\d+\.\d+[0-9A-Za-z.\-]*) \(([0-9a-f]{6,40})\)')


@dataclass
class AutopilotSettings:
    """The autopilot slot as plain values (from a profile or from launch arguments).

    mode           'sitl' (start ArduCopter SITL here) or 'external'
    version        '' or a fetch_sitl.sh version (4.7.1, latest ...)
    binary         '' or the path of an arducopter program
    source_dir     '' or an ArduPilot source tree
    param_files    extra parameter files (original files; cleaned at start)
    param_filter   'safe' or 'none'
    param_keep     parameter names the safe filter must keep
    run_dir        '' = automatic (see the module docstring)
    wipe           erase saved parameters + mission at start (SITL -w)
    instance       SITL instance I
    mavlink_master '' = local SITL tcp:127.0.0.1:5760+10I; external: the remote MAVLink address
    fdm_addr       '' = the drone model's own value (127.0.0.1); external: 0.0.0.0
    sim_address    external only: this computer's address seen from the remote SITL (for the printed command)
    """
    mode: str = 'sitl'
    version: str = ''
    binary: str = ''
    source_dir: str = ''
    param_files: List[str] = field(default_factory=list)
    param_filter: str = 'safe'
    param_keep: List[str] = field(default_factory=list)
    run_dir: str = ''
    wipe: bool = False
    instance: int = 0
    mavlink_master: str = ''
    fdm_addr: str = ''
    sim_address: str = ''

    def needs_plan(self) -> bool:
        """True when SITL's binary, parameter files or run folder differ from the old launch code path.

        sitl_bin, sitl_wipe and sitl_instance on their own are handled by sim.launch.py's unchanged code.
        """
        return bool(self.mode == 'external' or self.version or self.source_dir or self.param_files
                    or self.run_dir or self.param_filter != 'safe' or self.param_keep)

    def is_default(self) -> bool:
        """True when nothing at all differs from today's simulator."""
        return not (self.needs_plan() or self.binary or self.mavlink_master or self.fdm_addr or self.wipe)


def settings_from_profile(data: Dict) -> AutopilotSettings:
    """AutopilotSettings from a resolved profile's data (profiles.ResolvedProfile.data)."""
    ap = data['autopilot']
    s = AutopilotSettings(instance=int(ap['instance']), wipe=bool(ap['wipe']), param_filter=ap['param_filter'],
                          param_keep=list(ap['param_keep']), param_files=list(ap['param_files']))
    if ap['kind'] == 'external':
        s.mode = 'external'
        s.mavlink_master = ap['external']['mavlink']
        s.fdm_addr = ap['external']['fdm_addr'] or '0.0.0.0'
        s.sim_address = ap['external']['sim_address']
        s.param_files, s.param_keep, s.wipe, s.param_filter = [], [], False, 'safe'
    elif ap['kind'] == 'sitl_source':
        s.source_dir = ap['source_dir']
    else:
        s.version, s.binary = ap['version'], ap['binary']
    return s


def _split_list(text: str) -> List[str]:
    return [p.strip() for p in str(text or '').split(',') if p.strip()]


def settings_from_launch(cfg: Dict[str, str]) -> AutopilotSettings:
    """AutopilotSettings from sim.launch.py's argument strings (missing names get their defaults)."""
    def get(key: str) -> str:
        return str(cfg.get(key, LAUNCH_DEFAULTS.get(key, '')) or '').strip()

    mode = get('autopilot').lower() or 'sitl'
    mode = 'external' if mode in ('external', 'remote') else 'sitl'
    try:
        instance = int(cfg.get('sitl_instance', '0') or 0)
    except ValueError:
        instance = 0
    return AutopilotSettings(
        mode=mode, version=get('sitl_version'), binary=get('sitl_bin'), source_dir=get('sitl_source'),
        param_files=[paths.expand(p) for p in _split_list(get('sitl_param_files'))],
        param_filter=(get('sitl_param_filter').lower() or 'safe'),
        param_keep=[n.upper() for n in _split_list(get('sitl_param_keep'))],
        run_dir=paths.expand(get('sitl_run_dir')), wipe=paths.truthy(get('sitl_wipe') or 'false'),
        instance=instance, mavlink_master=get('mavlink_master'), fdm_addr=get('fdm_addr'))


def to_launch_args(s: AutopilotSettings) -> 'OrderedDict[str, str]':
    """The sim.launch.py arguments that select this autopilot (only those that differ from the defaults).

    A non-default flavour also gets sitl_run_dir, so the plan shows exactly which folder SITL will use (and
    a profile's binary gets its own folder, unlike the old sitl_bin:= on its own).
    """
    out: 'OrderedDict[str, str]' = OrderedDict()
    if s.mode == 'external':
        out['autopilot'] = 'external'
    if s.version:
        out['sitl_version'] = s.version
    if s.binary:
        out['sitl_bin'] = s.binary
    if s.source_dir:
        out['sitl_source'] = s.source_dir
    if s.param_files:
        out['sitl_param_files'] = ','.join(s.param_files)
    if s.param_filter != 'safe':
        out['sitl_param_filter'] = s.param_filter
    if s.param_keep:
        out['sitl_param_keep'] = ','.join(s.param_keep)
    if s.wipe:
        out['sitl_wipe'] = 'true'
    if s.mavlink_master:
        out['mavlink_master'] = s.mavlink_master
    if s.fdm_addr:
        out['fdm_addr'] = s.fdm_addr
    if s.run_dir:
        out['sitl_run_dir'] = s.run_dir
    elif flavour_tag(s, include_binary=True):
        out['sitl_run_dir'] = str(run_dir_for(s, include_binary=True))
    return out


# ---------------------------------------------------------------------------------------------
# Files and folders
# ---------------------------------------------------------------------------------------------

def _short_hash(text: str) -> str:
    return hashlib.sha1(text.encode('utf-8')).hexdigest()[:6]


def flavour_tag(s: AutopilotSettings, include_binary: bool = False) -> str:
    """A short folder tag for a non-default autopilot ('' for the default): v4.7.1, src-ardupilot-1a2b3c ...

    include_binary: also tag an explicit binary (profiles do; the old launch argument sitl_bin does not).
    """
    parts = []
    if s.mode == 'external':
        parts.append('external')
    elif s.source_dir:
        base = re.sub(r'[^A-Za-z0-9_.-]+', '_', Path(s.source_dir.rstrip('/')).name)[:20] or 'tree'
        parts.append(f'src-{base}-{_short_hash(s.source_dir)}')
    elif s.binary and include_binary:
        parts.append(f'bin-{_short_hash(s.binary)}')
    elif s.version and not s.binary:
        parts.append(f'v{s.version}')
    if s.param_files and s.mode != 'external':
        parts.append(f'params-{_short_hash(",".join(s.param_files) + s.param_filter + ",".join(s.param_keep))}')
    return '_'.join(parts)


def default_run_dir(instance: int) -> Path:
    """Today's SITL run folder (~/.astra_sim/sitl_I<I>), same as sim_paths.sitl_run_dir."""
    return paths.sim_home() / f'sitl_I{int(instance)}'


def run_dir_for(s: AutopilotSettings, include_binary: bool = False) -> Path:
    """The SITL run folder of these settings: s.run_dir, else today's folder or sitl_I<I>_<tag>."""
    if s.run_dir:
        return Path(s.run_dir)
    tag = flavour_tag(s, include_binary)
    return default_run_dir(s.instance) if not tag else paths.sim_home() / f'sitl_I{int(s.instance)}_{tag}'


def flavour_run_dir(s: AutopilotSettings) -> Path:
    """The run folder sim.launch.py uses for its arguments (sitl_bin on its own keeps today's folder)."""
    return run_dir_for(s, include_binary=False)


def version_dir(version: str) -> Path:
    """Where fetch_sitl.sh puts a version: ~/sitl/versions/<version>/."""
    return paths.sitl_versions_dir() / version


def source_binary(source_dir: str) -> Path:
    """The SITL program a built ArduPilot source tree has: <tree>/build/sitl/bin/arducopter."""
    return Path(paths.expand(source_dir)) / 'build' / 'sitl' / 'bin' / 'arducopter'


def source_params_dir(source_dir: str) -> Path:
    """ArduPilot's stock parameter files inside a source tree."""
    return Path(paths.expand(source_dir)) / 'Tools' / 'autotest' / 'default_params'


def default_binary() -> Optional[str]:
    """The default SITL search (sim_paths.find_sitl_binary): $ARDUPILOT_SITL_BIN, ~/sitl/arducopter, Docker."""
    try:
        from sim_bringup import sim_paths
        return sim_paths.find_sitl_binary('')
    except Exception:  # noqa: BLE001 - sim_bringup not importable here: same rules, local copy
        for cand in [os.environ.get('ARDUPILOT_SITL_BIN', ''), *SITL_BINARY_CANDIDATES]:
            if cand and os.access(paths.expand(cand), os.X_OK) and Path(paths.expand(cand)).is_file():
                return paths.expand(cand)
        return None


def default_param_file(name: str) -> Optional[str]:
    """The default stock-parameter search (sim_paths.find_param_file)."""
    try:
        from sim_bringup import sim_paths
        return sim_paths.find_param_file(name)
    except Exception:  # noqa: BLE001
        for d in [os.environ.get('ARDUPILOT_DEFAULT_PARAMS', ''), *PARAM_DIR_CANDIDATES]:
            if d and (Path(paths.expand(d)) / name).is_file():
                return str(Path(paths.expand(d)) / name)
        return None


def read_binary_version(path: str, limit_mb: int = 64) -> str:
    """The version string compiled into an arducopter program, e.g. '4.6.3 (3fc7011a)'; '' if not found."""
    try:
        with open(path, 'rb') as f:
            tail = b''
            read = 0
            while read < limit_mb * 1024 * 1024:
                chunk = f.read(1024 * 1024)
                if not chunk:
                    break
                read += len(chunk)
                m = _VERSION_IN_BINARY.search(tail + chunk)
                if m:
                    return f'{m.group(1).decode()} ({m.group(2).decode()})'
                tail = chunk[-200:]
    except OSError:
        return ''
    return ''


def is_elf(path: str) -> bool:
    """True when the file starts like a Linux program (ELF)."""
    try:
        with open(path, 'rb') as f:
            return f.read(4) == b'\x7fELF'
    except OSError:
        return False


# ---------------------------------------------------------------------------------------------
# The plan
# ---------------------------------------------------------------------------------------------

@dataclass
class AutopilotPlan:
    """What will run for the autopilot slot.

    mode           'sitl' or 'external'
    binary         the arducopter program ('' when none was found or external)
    how            plain words: where the binary comes from
    version        the version compiled into the binary ('' unknown)
    params_dir     folder searched first for copter.parm / gazebo-iris.parm ('' = the default search)
    param_reports  what happened to each extra parameter file
    run_dir        SITL's working folder (eeprom.bin = saved parameters + mission, logs)
    wipe           SITL starts with -w (erases eeprom.bin)
    mavlink_master where MAVProxy connects ('' = the local SITL)
    fdm_addr       the plugin's bind address ('' = unchanged, 127.0.0.1)
    errors         reasons it cannot start (each with a fix)
    notes          other remarks for the log
    """
    mode: str
    binary: str = ''
    how: str = ''
    version: str = ''
    params_dir: str = ''
    param_reports: List[params_mod.ParamFileReport] = field(default_factory=list)
    run_dir: str = ''
    wipe: bool = False
    mavlink_master: str = ''
    fdm_addr: str = ''
    errors: List[str] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def stock_param_files(self, finder: Callable[[str], Optional[str]] = None) -> List[str]:
        """copter.parm + gazebo-iris.parm: from params_dir when it has them, else the default search."""
        finder = finder or default_param_file
        out = []
        for name in STOCK_PARAM_FILES:
            mine = Path(self.params_dir) / name if self.params_dir else None
            if mine is not None and mine.is_file():
                out.append(str(mine))
                continue
            found = finder(name)
            if found:
                if self.params_dir:
                    self.notes.append(f'{name} is not in {self.params_dir}: using {found} instead')
                out.append(found)
            else:
                self.notes.append(f'ArduPilot parameter file {name} not found '
                                  '(run simulation/setup/02_setup_ardupilot.sh or fetch_sitl.sh)')
        return out

    def defaults_list(self, astra_parm: Optional[str], finder: Callable[[str], Optional[str]] = None) -> List[str]:
        """SITL's --defaults list: stock files, astra_sitl.parm, then the cleaned extra files (later wins)."""
        files = self.stock_param_files(finder)
        if astra_parm and Path(astra_parm).is_file():
            files.append(astra_parm)
        else:
            self.notes.append('sim_models/config/astra_sitl.parm not found: payload servos 11/13 are not set up')
        files += [r.output for r in self.param_reports if r.output]
        return files


def plan_autopilot(s: AutopilotSettings, params_out_dir: Optional[Path] = None, write_params: bool = True,
                   binary_finder: Callable[[], Optional[str]] = None) -> AutopilotPlan:
    """Decide the binary, parameter folder, extra parameter files and run folder for these settings.

    params_out_dir  where the cleaned parameter files go (default ~/.astra_sim/generated/params)
    write_params    False = only read and report (the dry-run plan), True = write the cleaned files
    binary_finder   the default SITL search (sim.launch.py passes sim_paths.find_sitl_binary)
    """
    plan = AutopilotPlan(mode=s.mode, wipe=s.wipe, run_dir=str(flavour_run_dir(s)), fdm_addr=s.fdm_addr)
    if s.mode == 'external':
        plan.how = 'external autopilot (not started here)'
        plan.mavlink_master = s.mavlink_master
        if not s.mavlink_master:
            plan.errors.append('autopilot:=external needs mavlink_master (e.g. tcp:192.168.1.50:5760): where '
                               'MAVProxy finds the autopilot\'s MAVLink')
        if not s.fdm_addr:
            plan.fdm_addr = '0.0.0.0'
        return plan
    if s.mavlink_master:
        plan.notes.append(f'MAVProxy connects to {s.mavlink_master} instead of the local SITL')
    plan.mavlink_master = s.mavlink_master
    if s.binary:
        cand = Path(paths.expand(s.binary))
        plan.how = f'the program you gave ({cand})'
        if cand.is_file() and os.access(cand, os.X_OK):
            plan.binary = str(cand)
        else:
            plan.errors.append(f'arducopter program not found or not executable: {cand} '
                               '(chmod +x it, or check the path)')
    elif s.source_dir:
        tree = Path(paths.expand(s.source_dir))
        cand = source_binary(str(tree))
        plan.how = f'ArduPilot source tree {tree}'
        plan.params_dir = str(source_params_dir(str(tree)))
        if not tree.is_dir():
            plan.errors.append(f'ArduPilot source tree not found: {tree}')
        elif cand.is_file() and os.access(cand, os.X_OK):
            plan.binary = str(cand)
        else:
            plan.errors.append(f'{tree} has no built SITL ({cand} missing). Build it once: cd {tree} && '
                               './waf configure --board sitl && ./waf copter')
    elif s.version:
        vdir = version_dir(s.version)
        cand = vdir / 'arducopter'
        plan.how = f'ArduCopter {s.version} from fetch_sitl.sh ({vdir})'
        plan.params_dir = str(vdir)
        if cand.is_file() and os.access(cand, os.X_OK):
            plan.binary = str(cand)
        else:
            plan.errors.append(f'ArduCopter {s.version} is not downloaded ({cand} missing). Get it with: '
                               f'bash simulation/setup/fetch_sitl.sh {s.version}')
    else:
        found = (binary_finder or default_binary)()
        plan.how = 'the installed default ($ARDUPILOT_SITL_BIN, ~/sitl/arducopter, /opt/ardupilot)'
        if found:
            plan.binary = found
        else:
            plan.errors.append('ArduCopter SITL not found ($ARDUPILOT_SITL_BIN, ~/sitl/arducopter, '
                               '/opt/ardupilot/build/sitl/bin/arducopter): run simulation/setup/02_setup_ardupilot.sh')
    if plan.binary:
        plan.version = read_binary_version(plan.binary)
        if not is_elf(plan.binary):
            plan.notes.append(f'{plan.binary} does not look like a Linux program (no ELF header)')
    out_dir = Path(params_out_dir) if params_out_dir else paths.generated_dir() / 'params'
    for src in s.param_files:
        if not Path(src).is_file():
            plan.errors.append(f'parameter file not found: {src}')
            continue
        report = params_mod.prepare_param_file(src, out_dir, s.param_filter, s.param_keep, write=write_params)
        plan.param_reports.append(report)
        plan.notes.append(report.summary())
        plan.notes += [f'{Path(src).name}: {w}' for w in report.warnings]
    return plan


def describe(plan: AutopilotPlan, s: AutopilotSettings) -> str:
    """One line for the plan / the launch log."""
    if plan.mode == 'external':
        return (f'external autopilot: MAVProxy <- {plan.mavlink_master or "?"}; drone plugin listens on '
                f'{plan.fdm_addr or "0.0.0.0"}:{9002 + 10 * s.instance}; run folder {plan.run_dir}')
    ver = f' = ArduCopter {plan.version}' if plan.version else ''
    extra = f' + {len(s.param_files)} parameter file(s) ({s.param_filter} filter)' if s.param_files else ''
    return f'SITL {plan.binary or "(not found)"}{ver}{extra}; run folder {plan.run_dir}'


def remote_command(s: AutopilotSettings, home: str, sim_address: str = '') -> str:
    """The command to start ArduCopter SITL on the OTHER computer for autopilot:=external.

    SITL sends its physics packets to --sim-address:9002+10*I (the plugin here answers the sender), and
    listens for MAVLink on TCP 5760+10*I (MAVProxy here connects with --master tcp:<that computer>:5760).
    """
    addr = sim_address or s.sim_address or '<this computer\'s IP>'
    return (f'arducopter --model JSON --sim-address={addr} -I{s.instance} --home {home} '
            '--defaults copter.parm,gazebo-iris.parm,astra_sitl.parm')


def mavproxy_master_args(master: str, external: bool) -> List[str]:
    """MAVProxy's --master (and, for a remote TCP autopilot, --retries) arguments."""
    out = ['--master', master]
    if external and master.startswith('tcp:'):
        out += ['--retries', str(EXTERNAL_TCP_RETRIES)]
    return out


# ---------------------------------------------------------------------------------------------
# The drone model: where its ArduPilot plugin binds the physics port
# ---------------------------------------------------------------------------------------------

_PLUGIN_RE = re.compile(r'(<plugin\b[^>]*filename\s*=\s*["\'][^"\']*ArduPilotPlugin[^"\']*["\'][^>]*>)(.*?)(</plugin>)',
                        re.S)


def patch_plugin_fdm_addr(sdf_text: str, fdm_addr: str) -> Tuple[str, bool]:
    """Set <fdm_addr> of the ArduPilotPlugin in a drone SDF (inserted when missing).

    Returns (new text, True if an ArduPilotPlugin was found). The plugin (ardupilot_gazebo, 2026) BINDS its
    JSON physics port (9002+10*I) on this address and answers whoever sends to it; 127.0.0.1 (the model's
    value) only accepts a SITL on the same computer, 0.0.0.0 accepts one on another computer/container.
    (<listen_addr> is deprecated in this plugin version: it prints a warning and is ignored.)
    """
    if not fdm_addr:
        return sdf_text, bool(_PLUGIN_RE.search(sdf_text))
    found = False

    def fix(m):
        nonlocal found
        found = True
        head, body, tail = m.group(1), m.group(2), m.group(3)
        if re.search(r'<fdm_addr>.*?</fdm_addr>', body, re.S):
            body = re.sub(r'<fdm_addr>.*?</fdm_addr>', f'<fdm_addr>{fdm_addr}</fdm_addr>', body, flags=re.S)
        else:
            body = f'\n      <fdm_addr>{fdm_addr}</fdm_addr>' + body
        return head + body + tail

    return _PLUGIN_RE.sub(fix, sdf_text), found
