"""
sim_plugins/plan.py -- PURPOSE: turn a resolved profile into what the scripts and launch files need:
launch arguments for sim.launch.py and team_stack.launch.py, extra stack_check options, the shell
variables demo.sh / start_sim.sh use, environment checks (is every file there?), and the printable plan.

The rule that keeps today's behaviour exactly: only settings that DIFFER from the built-in defaults
produce extra arguments. With no profile (or simulation/profiles/default.yaml) every PLUGIN_* list is
empty and the commands are byte for byte the ones the scripts ran before profiles existed
(test_plan.py and the dry-run comparison in test/run_tests.sh prove it).

Who uses what:
    demo_vars(rp)       demo.sh      (through `profile_tool shell --for demo`)
    start_sim_vars(rp)  start_sim.sh (through `profile_tool shell --for start_sim`)
    team_stack_hint()   start_sim.sh prints the command for the second terminal
    summary_lines(rp)   every plan printout ("which component fills which slot")
    check_environment() errors that would stop the start (missing binaries, packages, files)
"""

from __future__ import annotations

import hashlib
import os
import shlex
import shutil
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from . import autopilot as ap_mod
from . import paths
from .profiles import ResolvedProfile, dump_resolved

#: Default value of every sim.launch.py argument a profile can set. MUST equal sim.launch.py's ARGUMENTS
#: (test_plan.py reads sim.launch.py and compares).
SIM_LAUNCH_DEFAULTS: 'OrderedDict[str, str]' = OrderedDict([
    ('world', 'runway'), ('camera_model', 'zr10'), ('gui', 'true'), ('rviz', 'true'), ('panel', 'true'),
    ('detector', 'real'), ('sitl_instance', '0'), ('gcs_port', '14550'), ('mavros_port', '14552'),
    ('panel_port', '14553'), ('windows_gcs', 'true'), ('windows_gcs_addr', ''), ('emulator', 'true'),
    ('camera_ip', '192.168.144.25'), ('camera_max_zoom', ''), ('camera_bind', ''), ('camera_strict', ''),
    ('gcs_out', ''), ('mavros_out', ''), ('extra_outs', ''), ('profile', ''),
    *ap_mod.LAUNCH_DEFAULTS.items(),
])

#: sim.launch.py arguments demo.sh always builds itself (from its own variables), so they never go into
#: PLUGIN_SIM_ARGS for demo.sh.
DEMO_OWNED = frozenset({'world', 'camera_model', 'detector', 'score', 'demo_run_dir', 'sitl_instance', 'gcs_port',
                        'mavros_port', 'panel_port', 'camera_max_zoom', 'gui', 'rviz', 'panel', 'windows_gcs',
                        'windows_gcs_addr'})

DETECTOR_LAUNCH_VALUE = {'team': 'real', 'truth': 'truth', 'custom': 'custom'}

#: The team nodes a custom slot replaces (name used by stack_check, (package, executable) in tracker.launch.xml).
REPLACED_NODES = {'detector': ('new_od', ('detection', 'new_od')),
                  'mission_brain': ('main_controller', ('main', 'main_controller.py'))}


# ---------------------------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------------------------

def zoom_to_max_zoom(zoom) -> str:
    """camera.zoom -> sim.launch.py camera_max_zoom: survey -> '1', team -> '' (camera default), N -> 'N'."""
    if zoom == 'survey':
        return '1'
    if zoom == 'team':
        return ''
    return f'{float(zoom):g}'


def team_side_entries(rp: ResolvedProfile) -> List[Tuple[str, Dict]]:
    """(role, entry) of everything team_stack.launch.py must start from the profile (custom slots + extras)."""
    out = []
    for slot in ('detector', 'mission_brain'):
        if rp.get(f'{slot}.kind') == 'custom' and rp.get(f'{slot}.custom'):
            out.append((slot, rp.get(f'{slot}.custom')))
    out += [('extra', e) for e in rp.get('extras', []) if e['start_with'] == 'team_stack']
    return out


def sim_side_entries(rp: ResolvedProfile) -> List[Tuple[str, Dict]]:
    """(role, entry) of everything sim.launch.py must start from the profile (custom camera + extras)."""
    out = []
    if rp.get('camera.kind') == 'custom' and rp.get('camera.custom'):
        out.append(('camera', rp.get('camera.custom')))
    out += [('extra', e) for e in rp.get('extras', []) if e['start_with'] == 'simulator']
    return out


def needs_resolved_file(rp: ResolvedProfile) -> bool:
    """True when a launch file must read the profile itself (it has entries that are not simple arguments)."""
    return bool(team_side_entries(rp) or sim_side_entries(rp))


def resolved_file_path(rp: ResolvedProfile, out_dir: Optional[Path] = None) -> Path:
    """Where the resolved profile is written: <generated>/profile_<name>_<6 hex of its content>.yaml."""
    out_dir = Path(out_dir) if out_dir else paths.generated_dir()
    text = dump_resolved(rp)
    safe = ''.join(c if c.isalnum() or c in '_.-' else '_' for c in rp.name)[:40] or 'profile'
    return out_dir / f'profile_{safe}_{hashlib.sha1(text.encode("utf-8")).hexdigest()[:6]}.yaml'


def write_resolved_file(rp: ResolvedProfile, out_dir: Optional[Path] = None) -> Path:
    """Write the resolved profile (see resolved_file_path) and return its path."""
    path = resolved_file_path(rp, out_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(dump_resolved(rp), encoding='utf-8')
    return path


# ---------------------------------------------------------------------------------------------
# Launch arguments
# ---------------------------------------------------------------------------------------------

def sim_args_full(rp: ResolvedProfile) -> 'OrderedDict[str, str]':
    """Every sim.launch.py argument this profile decides (as launch strings), defaults included.

    camera_max_zoom follows the scripts' meaning of camera.zoom (survey = 1); profile:= is not included
    (the caller adds it when needs_resolved_file()).
    """
    d = rp.data
    net, cam = d['network'], d['camera']
    args: 'OrderedDict[str, str]' = OrderedDict()
    args['world'] = d['world']['name']
    args['camera_model'] = cam['model']
    args['detector'] = DETECTOR_LAUNCH_VALUE[d['detector']['kind']]
    args['sitl_instance'] = str(d['autopilot']['instance'])
    args['gcs_port'] = str(net['gcs_port'])
    args['mavros_port'] = str(net['mavros_port'])
    args['panel_port'] = str(net['panel_port'])
    args['windows_gcs'] = 'true' if net['windows_gcs'] else 'false'
    args['windows_gcs_addr'] = net['windows_gcs_addr']
    args['camera_max_zoom'] = zoom_to_max_zoom(cam['zoom'])
    headless = bool(d['run']['headless'])
    for window in ('gui', 'rviz', 'panel'):
        args[window] = 'false' if headless else 'true'
    args['emulator'] = 'true' if cam['kind'] == 'siyi_emulator' else 'false'
    args['camera_ip'] = net['camera_ip']
    args['camera_bind'] = net['camera_bind']
    args['camera_strict'] = 'true' if cam['strict'] else ''
    args['gcs_out'] = net['gcs_out']
    args['mavros_out'] = net['mavros_out']
    args['extra_outs'] = ','.join(net['extra_outs'])
    settings = ap_mod.settings_from_profile(d)
    for key, default in ap_mod.LAUNCH_DEFAULTS.items():
        args[key] = default
    args.update(ap_mod.to_launch_args(settings))
    return args


def nondefault(args: 'OrderedDict[str, str]', defaults: Dict[str, str] = None) -> 'OrderedDict[str, str]':
    """Only the arguments whose value differs from the launch file's default."""
    defaults = SIM_LAUNCH_DEFAULTS if defaults is None else defaults
    return OrderedDict((k, v) for k, v in args.items() if str(v) != str(defaults.get(k, '')))


def stack_check_args(rp: ResolvedProfile) -> List[str]:
    """Extra stack_check options: which team nodes are replaced (and which node to wait for instead)."""
    out: List[str] = []
    for slot, (team_node, _) in REPLACED_NODES.items():
        if rp.get(f'{slot}.kind') == 'custom':
            out += ['--skip-node', team_node]
            node = (rp.get(f'{slot}.custom') or {}).get('node_name', '')
            if node:
                out += ['--add-node', node]
    return out


def need_detector(rp: ResolvedProfile) -> bool:
    """True when demo.sh should wait for the detector's model (stack_check --need-detector)."""
    kind = rp.get('detector.kind')
    if kind == 'team':
        return True
    if kind == 'custom':
        return bool((rp.get('detector.custom') or {}).get('provides_health'))
    return False


def team_launch_args(rp: ResolvedProfile, resolved_file: str = '') -> List[str]:
    """team_stack.launch.py arguments beyond detector:=... (model path, batched, the resolved profile)."""
    out: List[str] = []
    if rp.get('detector.kind') == 'team':
        if rp.get('detector.model_path'):
            out.append(f'model_path:={rp.get("detector.model_path")}')
        if rp.get('detector.batched'):
            out.append('batched:=true')
    if rp.get('mission_brain.kind') == 'custom':
        out.append('mission_brain:=custom')
    if resolved_file and team_side_entries(rp):
        out.append(f'profile:={resolved_file}')
    return out


def default_fcu_url(mavros_port: int) -> str:
    """MAVROS's fcu_url that matches MAVProxy's MAVROS output (launcher.sh sim's default for 14552)."""
    return f'udp://:{mavros_port}@localhost:{mavros_port}'


# ---------------------------------------------------------------------------------------------
# Script variables
# ---------------------------------------------------------------------------------------------

@dataclass
class ShellVars:
    """Variables for a script: scalars (NAME=value) and arrays (NAME=(a b c)), printed shell-safe."""
    scalars: 'OrderedDict[str, str]' = field(default_factory=OrderedDict)
    arrays: 'OrderedDict[str, List[str]]' = field(default_factory=OrderedDict)

    def render(self) -> str:
        """Lines a bash script can `eval` (every value quoted with shlex.quote)."""
        lines = [f'{k}={shlex.quote(str(v))}' for k, v in self.scalars.items()]
        lines += [f'{k}=({" ".join(shlex.quote(str(x)) for x in v)})' for k, v in self.arrays.items()]
        return '\n'.join(lines) + '\n'


def demo_vars(rp: ResolvedProfile, resolved_file: str = '', summary: str = '') -> ShellVars:
    """Everything demo.sh needs from a profile (see simulation/scripts/demo.sh)."""
    d = rp.data
    net = d['network']
    sv = ShellVars()
    sc = sv.scalars
    sc['WORLD'] = d['world']['name']
    sc['DETECTOR'] = DETECTOR_LAUNCH_VALUE[d['detector']['kind']]
    sc['CAMERA'] = d['camera']['model']
    zoom = d['camera']['zoom']
    sc['ZOOM'] = zoom if isinstance(zoom, str) else f'{float(zoom):g}'
    sc['HEADLESS'] = '1' if d['run']['headless'] else '0'
    sc['TIMEOUT_MIN'] = str(d['run']['timeout_min'])
    sc['INSTANCE'] = str(d['autopilot']['instance'])
    sc['GCS_PORT'] = str(net['gcs_port'])
    sc['MAVROS_PORT'] = str(net['mavros_port'])
    sc['PANEL_PORT'] = str(net['panel_port'])
    sc['WINDOWS_GCS'] = 'true' if net['windows_gcs'] else 'false'
    sc['WINDOWS_GCS_ADDR'] = net['windows_gcs_addr']
    sc['MISSION_OVERRIDE'] = d['world']['mission']
    sc['MODEL_OVERRIDE'] = d['detector']['model_path'] if d['detector']['kind'] == 'team' else ''
    sc['FCU_URL_OVERRIDE'] = net['fcu_url']
    sc['TEAM_STACK_WHERE'] = d['run']['team_stack']
    sc['NEED_DETECTOR'] = '1' if need_detector(rp) else '0'
    sc['CAMERA_KIND'] = d['camera']['kind']
    sc['CAMERA_IP'] = net['camera_ip']
    sc['PLUGIN_PROFILE'] = rp.name
    sc['PLUGIN_PROFILE_FILE'] = resolved_file
    sc['PLUGIN_DETECTOR_TEXT'] = detector_text(rp)
    sc['PLUGIN_SUMMARY'] = summary
    sim = nondefault(sim_args_full(rp))
    plugin_sim = [f'{k}:={v}' for k, v in sim.items() if k not in DEMO_OWNED]
    if resolved_file and sim_side_entries(rp):
        plugin_sim.append(f'profile:={resolved_file}')
    sv.arrays['PLUGIN_SIM_ARGS'] = plugin_sim
    sv.arrays['PLUGIN_TEAM_ARGS'] = team_launch_args(rp, resolved_file)
    sv.arrays['PLUGIN_STACK_ARGS'] = stack_check_args(rp)
    return sv


def start_sim_vars(rp: ResolvedProfile, typed: Sequence[str] = (), resolved_file: str = '',
                   summary: str = '') -> ShellVars:
    """Everything start_sim.sh needs: WORLD and the extra launch arguments (never one the user typed)."""
    typed = set(typed)
    sv = ShellVars()
    sv.scalars['WORLD'] = rp.get('world.name')
    sv.scalars['PLUGIN_PROFILE'] = rp.name
    sv.scalars['PLUGIN_PROFILE_FILE'] = resolved_file
    sv.scalars['PLUGIN_SUMMARY'] = summary
    sv.scalars['PLUGIN_TEAM_HINT'] = team_stack_hint(rp, resolved_file)
    sim = nondefault(sim_args_full(rp))
    sim.pop('world', None)
    # camera_max_zoom: start_sim.sh adds camera_max_zoom:=1 itself unless one is given, so "survey" needs
    # nothing, "team" must say camera_max_zoom:= (empty = camera default) and a number says itself.
    zoom = zoom_to_max_zoom(rp.get('camera.zoom'))
    sim.pop('camera_max_zoom', None)
    out = [f'{k}:={v}' for k, v in sim.items() if k not in typed]
    if zoom != '1' and 'camera_max_zoom' not in typed:
        out.append(f'camera_max_zoom:={zoom}')
    if resolved_file and sim_side_entries(rp) and 'profile' not in typed:
        out.append(f'profile:={resolved_file}')
    sv.arrays['PLUGIN_SIM_ARGS'] = out
    return sv


#: The team-stack launch for terminal 2, exactly as simulation/README.md ("OR BY HAND") gives it. start_sim.sh
#: prints the same command without a profile, so the default profile's printout is identical to no profile.
TEAM_STACK_LAUNCH = 'ros2 launch sim_bringup team_stack.launch.py'


def team_stack_hint(rp: ResolvedProfile, resolved_file: str = '') -> str:
    """The command for terminal 2 (the team stack) that matches this profile.

    Only what differs from the defaults is added: the default profile gives exactly the README's command
    TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py" ./launcher.sh sim (detector:=real is
    team_stack.launch.py's own default, and launcher.sh sim's own FCU_URL matches MAVProxy's 14552 output).
    """
    kind = rp.get('detector.kind')
    args = ([] if kind == 'team' else [f'detector:={DETECTOR_LAUNCH_VALUE[kind]}']) + team_launch_args(rp, resolved_file)
    fcu = rp.get('network.fcu_url') or (default_fcu_url(rp.get('network.mavros_port'))
                                        if rp.get('network.mavros_port') != 14552 else '')
    env = f'FCU_URL={fcu} ' if fcu else ''
    return f'{env}TRACKER_CMD="{" ".join([TEAM_STACK_LAUNCH] + args)}" ./launcher.sh sim'


# ---------------------------------------------------------------------------------------------
# Human-readable plan
# ---------------------------------------------------------------------------------------------

def entry_text(entry: Dict) -> str:
    """'package/launch file (args)' in a few words."""
    if entry.get('launch'):
        what = f'{entry["package"]} {entry["launch"]}' if entry.get('package') else entry['launch']
    elif entry.get('executable'):
        what = f'ros2 run {entry["package"]} {entry["executable"]}'
    else:
        what = ' '.join(entry.get('command', []))
    args = entry.get('args') or {}
    return what + (f' ({", ".join(f"{k}={v}" for k, v in args.items())})' if args else '')


def detector_text(rp: ResolvedProfile) -> str:
    """The detector slot in words."""
    kind = rp.get('detector.kind')
    if kind == 'team':
        model = rp.get('detector.model_path') or '~/.astra_sim/models/yolo26m.pt'
        return f'team: detection/new_od (YOLO weights {model}{", batched" if rp.get("detector.batched") else ""})'
    if kind == 'truth':
        return 'truth: sim_demo truth_detector (answer-key stand-in; new_od idles on an empty folder)'
    return f'custom: {entry_text(rp.get("detector.custom") or {})} (replaces new_od)'


def summary_lines(rp: ResolvedProfile, env: Optional[Dict] = None) -> List[Tuple[str, str]]:
    """(slot, text) for every slot. env (from check_environment) adds what was found on this computer."""
    env = env or {}
    d = rp.data
    lines: List[Tuple[str, str]] = []
    mission = d['world']['mission'] or "the world's own survey mission"
    lines.append(('world', f'{d["world"]["name"]} (mission: {mission})'))
    settings = ap_mod.settings_from_profile(d)
    apl = env.get('autopilot_plan')
    if apl is not None:
        lines.append(('autopilot', ap_mod.describe(apl, settings)))
    else:
        kind = d['autopilot']['kind']
        if kind == 'external':
            text = f'external: MAVLink from {d["autopilot"]["external"]["mavlink"]}'
        elif kind == 'sitl_source':
            text = f'SITL from the ArduPilot source tree {d["autopilot"]["source_dir"]}'
        elif d['autopilot']['binary']:
            text = f'SITL program {d["autopilot"]["binary"]}'
        elif d['autopilot']['version']:
            text = f'SITL ArduCopter {d["autopilot"]["version"]} (~/sitl/versions/{d["autopilot"]["version"]})'
        else:
            text = 'SITL: the installed default (~/sitl/arducopter, ArduCopter 4.6.3)'
        if d['autopilot']['param_files']:
            text += (f' + {len(d["autopilot"]["param_files"])} parameter file(s) '
                     f'({d["autopilot"]["param_filter"]} filter)')
        lines.append(('autopilot', text + f'; run folder {ap_mod.run_dir_for(settings, include_binary=True)}'))
    cam = d['camera']
    if cam['kind'] == 'siyi_emulator':
        size = '2K photos' if cam['model'] == 'zr10' else '4K photos'
        zoom = cam['zoom'] if isinstance(cam['zoom'], str) else f'max {float(cam["zoom"]):g}x'
        zoom = {'survey': 'survey (max 1x)', 'team': 'team (camera default)'}.get(zoom, zoom)
        lines.append(('camera', f'siyi_emulator, {cam["model"]} profile ({size}), zoom {zoom}, '
                                f'{"strict" if cam["strict"] else "permissive"}'))
    elif cam['kind'] == 'custom':
        lines.append(('camera', f'custom: {entry_text(cam["custom"] or {})} ({cam["model"]} photo size)'))
    else:
        lines.append(('camera', 'none (no camera on the network: video_cam cannot take photos)'))
    lines.append(('detector', detector_text(rp)))
    if d['mission_brain']['kind'] == 'team':
        lines.append(('mission_brain', 'team: main/main_controller.py'))
    else:
        lines.append(('mission_brain', f'custom: {entry_text(d["mission_brain"]["custom"] or {})} '
                                       '(replaces main_controller)'))
    extras = d['extras']
    lines.append(('extras', '; '.join(f'{e["name"]}: {entry_text(e)} [with the '
                                      f'{"team stack" if e["start_with"] == "team_stack" else "simulator"}]'
                                      for e in extras) or 'none'))
    net = d['network']
    inst = int(d['autopilot']['instance'])
    master = d['autopilot']['external']['mavlink'] if d['autopilot']['kind'] == 'external' \
        else f'tcp:127.0.0.1:{5760 + 10 * inst}'
    outs = [f'ground station {net["gcs_out"] or "udp:127.0.0.1:%d" % net["gcs_port"]}',
            f'MAVROS {net["mavros_out"] or "udp:127.0.0.1:%d" % net["mavros_port"]}',
            f'demo panel udp:127.0.0.1:{net["panel_port"]}']
    outs += [f'extra {o}' for o in net['extra_outs']]
    if net['windows_gcs']:
        outs.append(f'Windows ground station {net["windows_gcs_addr"] or "(automatic inside WSL)"}')
    lines.append(('network', f'MAVProxy <- {master}; -> {", ".join(outs)}; camera emulator on '
                             f'{net["camera_ip"]}{" (listens on " + net["camera_bind"] + ")" if net["camera_bind"] else ""}'))
    fcu = net['fcu_url'] or default_fcu_url(net['mavros_port'])
    where = d['run']['team_stack']
    lines.append(('team stack', f'{"local: ./launcher.sh sim" if where == "local" else "external (started elsewhere; demo.sh only waits for it)"}'
                                f', MAVROS fcu_url {fcu}'))
    return lines


def render_summary(rp: ResolvedProfile, env: Optional[Dict] = None) -> str:
    """The slot table as text (the first part of every plan)."""
    src = rp.source or 'no profile file'
    head = f'PROFILE {rp.name}  ({src}{"; overrides: " + " ".join(rp.sets) if rp.sets else ""})'
    body = []
    for slot, text in summary_lines(rp, env):
        body.append(f'  {slot:13s} {text}')
        if slot == 'autopilot':
            # check_environment's notes are about the autopilot (parameter files, stock files): show them here.
            body += [f'  {"":13s} .. {n}' for n in (env or {}).get('notes', [])]
    warns = [f'  !! {w.text()}' for w in rp.warnings] + [f'  !! {w}' for w in (env or {}).get('warnings', [])]
    return '\n'.join([head] + body + warns)


# ---------------------------------------------------------------------------------------------
# Environment checks (does every file / program / package exist on THIS computer?)
# ---------------------------------------------------------------------------------------------

def _package_share(package: str) -> Optional[str]:
    """Installed share folder of a ROS package, None when not found, '?' when ROS is not available."""
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    except ImportError:
        return '?'
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        return None


def _package_prefix(package: str) -> Optional[str]:
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    except ImportError:
        return '?'
    try:
        return get_package_prefix(package)
    except PackageNotFoundError:
        return None


def resolve_entry_launch(entry: Dict, share_finder: Callable[[str], Optional[str]] = None) -> Tuple[str, str]:
    """(absolute launch file path, problem) of an entry's launch file ('' path when not found)."""
    share_finder = share_finder or _package_share
    launch = entry.get('launch', '')
    if not launch:
        return '', ''
    if launch.startswith('/') or os.path.isabs(launch):
        return (launch, '') if Path(launch).is_file() else ('', f'launch file not found: {launch}')
    share = share_finder(entry.get('package', ''))
    if share is None:
        return '', (f'ROS package "{entry.get("package")}" is not built/installed '
                    '(build it with simulation/setup/03_build.sh, then source the workspace)')
    if share == '?':
        return '', ''
    for cand in (Path(share) / 'launch' / launch, Path(share) / launch):
        if cand.is_file():
            return str(cand), ''
    return '', f'{launch} is not in {share}/launch (is it installed by setup.py data_files?)'


def entry_problems(role: str, entry: Dict, share_finder=None, prefix_finder=None) -> List[str]:
    """What is missing on this computer for one custom entry."""
    prefix_finder = prefix_finder or _package_prefix
    problems: List[str] = []
    label = f'{role} "{entry.get("name") or entry.get("package")}"'
    if entry.get('launch'):
        _, why = resolve_entry_launch(entry, share_finder)
        if why:
            problems.append(f'{label}: {why}')
    elif entry.get('executable'):
        prefix = prefix_finder(entry['package'])
        if prefix is None:
            problems.append(f'{label}: ROS package "{entry["package"]}" is not built/installed')
        elif prefix != '?' and not (Path(prefix) / 'lib' / entry['package'] / entry['executable']).exists():
            problems.append(f'{label}: {entry["package"]} has no executable "{entry["executable"]}" '
                            '(check entry_points in its setup.py)')
    elif entry.get('command'):
        prog = entry['command'][0]
        if not (Path(prog).is_file() or shutil.which(prog)):
            problems.append(f'{label}: program "{prog}" not found')
    return problems


def check_environment(rp: ResolvedProfile, share_finder=None, prefix_finder=None,
                      binary_finder=None) -> Dict:
    """Check this computer for everything the profile needs.

    Returns {'errors': [...], 'warnings': [...], 'notes': [...], 'autopilot_plan': AutopilotPlan}.
    Nothing is written (the parameter files are read and filtered only in memory).
    """
    errors: List[str] = []
    warnings: List[str] = []
    settings = ap_mod.settings_from_profile(rp.data)
    # The scripts give sim.launch.py an explicit sitl_run_dir (to_launch_args): plan with the same folder.
    settings.run_dir = settings.run_dir or str(ap_mod.run_dir_for(settings, include_binary=True))
    apl = ap_mod.plan_autopilot(settings, write_params=False, binary_finder=binary_finder)
    errors += apl.errors
    for role, entry in team_side_entries(rp) + sim_side_entries(rp):
        errors += entry_problems(role, entry, share_finder, prefix_finder)
    world = rp.get('world.name')
    if (world.endswith(('.sdf', '.yaml', '.yml')) or '/' in world) and not Path(world).is_file():
        errors.append(f'world file not found: {world}')
    mission = rp.get('world.mission')
    if mission and not Path(mission).is_file():
        errors.append(f'mission file not found: {mission}')
    model = rp.get('detector.model_path')
    if rp.get('detector.kind') == 'team' and model and not Path(model).is_file():
        errors.append(f'detector weights not found: {model}')
    return {'errors': errors, 'warnings': warnings, 'notes': list(apl.notes), 'autopilot_plan': apl}
