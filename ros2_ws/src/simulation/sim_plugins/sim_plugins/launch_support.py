"""
sim_plugins/launch_support.py -- PURPOSE: the helpers sim_bringup's launch files use to apply a plug-in profile:

    load_profile(ref)               read + check a profile given to a launch file as profile:=NAME_OR_PATH
    apply_profile_args(cfg, ...)    use the profile's value for every launch argument the user left at its
                                    default (what the user typed on the command line always wins)
    entry_actions(entry, role)      turn one profile entry (a custom detector, mission brain, camera or an
                                    extra) into launch actions: an included launch file, one node, or a program

Why here and not in the launch files: the same code serves sim.launch.py (custom camera, extras that start
with the simulator) and team_stack.launch.py (custom detector / mission brain, extras that start with the team
stack), and it can be unit-tested. Only entry_actions needs the ROS 2 `launch` libraries (imported inside it),
so everything else works in plain Python tests.
"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple

from . import plan as plan_mod
from . import profiles


def load_profile(ref: str) -> profiles.ResolvedProfile:
    """Read and check a profile for a launch file (a name in simulation/profiles or a path).

    Raises profiles.ProfileError with every problem in plain words (the launch file prints them and stops).
    """
    return profiles.resolve(str(ref).strip())


def apply_profile_args(cfg: Dict[str, str], defaults: Dict[str, str], profile_args: Dict[str, str]
                       ) -> List[Tuple[str, str, str]]:
    """Copy profile values into launch arguments that are still at their default value.

    cfg           the launch arguments as strings (changed in place)
    defaults      the launch file's default of each argument
    profile_args  the arguments the profile decides (plan.sim_args_full or team_args)
    Returns [(name, old value, new value)] of every argument that was changed (for the launch log).
    An argument the user typed (its value differs from the default) is never changed. Typing an argument
    with exactly its default value cannot be told apart from not typing it: then the profile's value is used.
    """
    changed = []
    for name, value in profile_args.items():
        if name not in cfg or name not in defaults:
            continue
        if str(cfg[name]) == str(defaults[name]) and str(value) != str(defaults[name]):
            changed.append((name, cfg[name], str(value)))
            cfg[name] = str(value)
    return changed


def team_args(rp: profiles.ResolvedProfile) -> Dict[str, str]:
    """The team_stack.launch.py arguments a profile decides (detector, mission_brain, model_path, batched)."""
    out = {'detector': plan_mod.DETECTOR_LAUNCH_VALUE[rp.get('detector.kind')],
           'mission_brain': rp.get('mission_brain.kind')}
    if rp.get('detector.kind') == 'team':
        if rp.get('detector.model_path'):
            out['model_path'] = rp.get('detector.model_path')
        out['batched'] = 'true' if rp.get('detector.batched') else 'false'
    return out


def _launch_text(value) -> str:
    """A launch-argument string from a YAML value (true/false for booleans, comma lists for lists)."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (list, tuple)):
        return ','.join(str(v) for v in value)
    return str(value)


def launch_source(path: str):
    """The right launch-description source for a launch file, chosen by its extension."""
    suffix = Path(path).suffix.lower()
    if path.endswith('.launch.py') or suffix == '.py':
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        return PythonLaunchDescriptionSource(path)
    if suffix == '.xml':
        from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
        return XMLLaunchDescriptionSource(path)
    if suffix in ('.yaml', '.yml'):
        from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
        return YAMLLaunchDescriptionSource(path)
    raise ValueError(f'{path}: a launch file must end in .launch.py, .py, .xml or .yaml')


def entry_actions(entry: Dict, role: str, tag: str = 'plugins',
                  share_finder: Optional[Callable[[str], Optional[str]]] = None) -> List:
    """Launch actions that start one profile entry (see schema.ENTRY_KEYS).

      launch      IncludeLaunchDescription(<package share>/launch/<file> or a path, args as launch arguments)
      executable  Node(package, executable, name=node_name, parameters=[args])
      command     ExecuteProcess(cmd=command)   (no shell: a list of words)
    A problem (package not built, launch file missing) becomes a clear ERROR line in the launch log and the
    entry is skipped: the rest of the simulator still starts, and check_slot / stack_check will say what is
    missing.
    """
    from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
    from launch_ros.actions import Node

    label = f'{role} "{entry.get("name") or entry.get("package") or "?"}"'
    text = plan_mod.entry_text(entry)
    if entry.get('launch'):
        path, why = plan_mod.resolve_entry_launch(entry, share_finder)
        if not path:
            return [LogInfo(msg=f'[{tag}] ERROR {label} not started: {why or "launch file not found"}')]
        try:
            source = launch_source(path)
        except (ValueError, ImportError) as exc:
            return [LogInfo(msg=f'[{tag}] ERROR {label} not started: {exc}')]
        args = {str(k): _launch_text(v) for k, v in (entry.get('args') or {}).items()}
        return [LogInfo(msg=f'[{tag}] starting {label}: {text}'),
                IncludeLaunchDescription(source, launch_arguments=list(args.items()))]
    if entry.get('executable'):
        params = [dict(entry['args'])] if entry.get('args') else []
        kwargs = {'name': entry['node_name']} if entry.get('node_name') else {}
        return [LogInfo(msg=f'[{tag}] starting {label}: {text}'),
                Node(package=entry['package'], executable=entry['executable'], output='screen',
                     parameters=params, **kwargs)]
    if entry.get('command'):
        name = ''.join(c if c.isalnum() or c in '_-' else '_' for c in (entry.get('name') or 'extra'))
        return [LogInfo(msg=f'[{tag}] starting {label}: {text}'),
                ExecuteProcess(cmd=[str(c) for c in entry['command']], name=name, output='screen')]
    return [LogInfo(msg=f'[{tag}] ERROR {label}: nothing to start (no launch, executable or command)')]
