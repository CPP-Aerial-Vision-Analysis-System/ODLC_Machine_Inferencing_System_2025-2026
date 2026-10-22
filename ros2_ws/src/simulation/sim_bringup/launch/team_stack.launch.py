"""
sim_bringup/launch/team_stack.launch.py -- PURPOSE: run the team's OWN mission stack (main/launch/tracker.launch.xml)
unchanged, with only the detector's input/model paths pointed at things that exist on this PC.

Why: tracker.launch.xml starts new_od, video_cam (siyi), the DigiCamCtrl trigger, waypoint_manager, gpstest and
main_controller with NO parameters. new_od's default model_path is the Jetson path
/home/astra-dev/astra/ros2_ws/src/yolo26m.pt, which does not exist on a laptop, so the real detector could never
load a model in the simulator. This wrapper includes tracker.launch.xml exactly as it is and adds global
parameters that only new_od declares (every other node of the file ignores them):
  model_path             the YOLO weights file (03_build.sh downloads yolo26m.pt to ~/.astra_sim/models/)
  use_batched_inference  false by default here: the team's batched path (detection/batched_inference.py,
                         _extract_yolo) unwraps a PyTorch .pt model into its inner DetectionModel and every
                         predict() call fails ("unexpected keyword argument 'imgsz'"); it only works with the
                         Jetson's TensorRT .engine. new_od's own SAHI path (the parameter's other value) works
                         with the .pt file. Pass batched:=true to test the team default anyway.
  camera_feed_path       only in detector:=truth mode (see below)

How it is used: ./launcher.sh sim runs this file instead of tracker.launch.xml when the environment variable
TRACKER_CMD says so; simulation/scripts/demo.sh sets
    TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py detector:=real"
Arguments:
  detector    real  (default) new_od loads model_path and detects targets in every new photo.
              truth new_od watches an always-empty folder, so it stays quiet; sim_demo's truth_detector
                    (started by sim.launch.py detector:=truth) is then the only publisher of /image_detection.
              custom new_od is NOT started (a copy of tracker.launch.xml without it is used); a teammate's
                    detector takes its place: the profile's detector.custom entry, or one you start yourself.
  mission_brain team (default) main_controller runs. custom = main_controller is NOT started; the profile's
                    mission_brain.custom entry (or one you start yourself) takes its place.
  model_path  weights for new_od (default: $ASTRA_DETECTOR_MODEL or ~/.astra_sim/models/yolo26m.pt).
              "" keeps the team's Jetson default.
  batched     false (default) = new_od's SAHI path (works with .pt); true = the team's batched default.
  debug       passed to tracker.launch.xml (true also starts image_view on /yolo_image).
  profile     a plug-in profile (sim_plugins; a name in simulation/profiles or a path). It fills in every
              argument above that was left at its default, and starts its custom detector / mission brain and
              its extras that start with the team stack. demo.sh passes the resolved profile file here.
With every argument at its default the launch is exactly what it was before plug-in profiles existed: the
team's tracker.launch.xml itself is included and sim_plugins is not even imported.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, GroupAction, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from sim_bringup import sim_paths as sp

#: The team nodes a custom slot replaces, as (pkg, exec) in tracker.launch.xml (= sim_plugins.tracker_filter).
REPLACED = {'detector': ('detection', 'new_od'), 'mission_brain': ('main', 'main_controller.py')}


def truth_mode_feed_dir() -> str:
    """An always-empty folder for new_od to watch in detector:=truth mode (created if missing)."""
    path = sp.sim_home() / 'truth_mode_empty_feed'
    path.mkdir(parents=True, exist_ok=True)
    return str(path)


def _fail(msg: str):
    """Stop this launch with a clear message (launcher.sh then stops MAVROS too, and demo.sh reports it)."""
    return [LogInfo(msg=f'[team_stack] ERROR: {msg}'), EmitEvent(event=Shutdown(reason=msg.splitlines()[0]))]


def _setup(context):
    """Resolve the arguments and build the include (runs once at launch time)."""
    cfg = {name: LaunchConfiguration(name).perform(context)
           for name in ('detector', 'mission_brain', 'model_path', 'debug', 'batched', 'profile')}
    actions = []

    # A plug-in profile first: it fills in every argument left at its default (typed arguments win).
    profile_rp = None
    if cfg['profile'].strip():
        try:
            from sim_plugins import launch_support, plan as plug_plan
        except ImportError:
            return _fail('profile:= needs the sim_plugins package: build it with simulation/setup/03_build.sh')
        try:
            profile_rp = launch_support.load_profile(cfg['profile'])
        except Exception as exc:  # noqa: BLE001 - sim_plugins.profiles.ProfileError: every problem in words
            return _fail(f'the profile "{cfg["profile"]}" cannot be used:\n{exc}')
        defaults = {'detector': 'real', 'mission_brain': 'team', 'model_path': sp.detector_model_path(),
                    'batched': 'false'}
        actions.append(LogInfo(msg=f'[team_stack] plug-in profile "{profile_rp.name}" ({profile_rp.source})'))
        for name, _, new in launch_support.apply_profile_args(cfg, defaults, launch_support.team_args(profile_rp)):
            actions.append(LogInfo(msg=f'[team_stack]   from the profile: {name}:={new}'))

    detector = cfg['detector'].strip().lower()
    mission_brain = cfg['mission_brain'].strip().lower() or 'team'
    model = sp.expand(cfg['model_path'].strip())
    debug = cfg['debug']
    batched = sp.truthy(cfg['batched'])
    if detector not in ('real', 'truth', 'custom'):
        actions.append(LogInfo(msg=f'[team_stack] detector "{detector}" is unknown (real, truth or custom); '
                                   'using real'))
        detector = 'real'
    if mission_brain not in ('team', 'custom'):
        actions.append(LogInfo(msg=f'[team_stack] mission_brain "{mission_brain}" is unknown (team or custom); '
                                   'using team'))
        mission_brain = 'team'
    try:
        tracker = os.path.join(get_package_share_directory('main'), 'launch', 'tracker.launch.xml')
    except PackageNotFoundError:
        return [LogInfo(msg='[team_stack] ERROR: the team package "main" is not built '
                            '(run simulation/setup/03_build.sh)')]

    params = []
    if detector != 'custom':
        if model and os.path.isfile(model):
            params.append(SetParameter(name='model_path', value=model))
            actions.append(LogInfo(msg=f'[team_stack] new_od model_path := {model}'))
        elif model:
            actions.append(LogInfo(msg=f'[team_stack] WARNING: detector model {model} not found. new_od keeps its '
                                       'Jetson default path and will NOT detect anything. Download the model with '
                                       'simulation/setup/03_build.sh, or run the demo with --detector truth.'))
        else:
            actions.append(LogInfo(msg='[team_stack] model_path:="" -> new_od keeps its own default'))
        if not batched:
            params.append(SetParameter(name='use_batched_inference', value=False))
            actions.append(LogInfo(msg='[team_stack] new_od use_batched_inference := false (SAHI path; the batched '
                                       'path cannot run a .pt model, see detection/batched_inference.py '
                                       '_extract_yolo)'))
    if detector == 'truth':
        empty = truth_mode_feed_dir()
        params.append(SetParameter(name='camera_feed_path', value=empty))
        actions.append(LogInfo(msg=f'[team_stack] detector:=truth -> new_od watches the empty folder {empty} '
                                   '(it stays quiet); sim_demo truth_detector publishes /image_detection'))

    # A custom detector / mission brain REPLACES the team node: include a copy of tracker.launch.xml without it.
    remove = [REPLACED[slot] for slot, on in (('detector', detector == 'custom'),
                                               ('mission_brain', mission_brain == 'custom')) if on]
    launch_file = tracker
    if remove:
        try:
            from sim_plugins import tracker_filter
        except ImportError:
            return _fail('detector:=custom / mission_brain:=custom need the sim_plugins package: build it with '
                         'simulation/setup/03_build.sh')
        try:
            launch_file, removed, notes = tracker_filter.write_filtered(tracker, remove, sp.generated_dir())
        except (OSError, ValueError) as exc:
            return _fail(f'could not make the copy of tracker.launch.xml: {exc}')
        actions.append(LogInfo(msg=f'[team_stack] NOT starting the team\'s {", ".join(removed) or "(nothing)"}: a '
                                   f'plug-in replaces it ({launch_file})'))
        actions += [LogInfo(msg=f'[team_stack] note: {n}') for n in notes]

    include = IncludeLaunchDescription(XMLLaunchDescriptionSource(launch_file),
                                       launch_arguments={'debug': debug}.items())
    # GroupAction(scoped=True) keeps the global parameters inside this group (the tracker's nodes).
    actions.append(GroupAction([*params, include], scoped=True))

    # The profile's own components (custom detector / mission brain, extras), OUTSIDE the group: new_od's
    # model_path & co. must not leak into them.
    entries = plug_plan.team_side_entries(profile_rp) if profile_rp is not None else []
    for role, entry in entries:
        actions.extend(launch_support.entry_actions(entry, role, tag='team_stack'))
    for slot, on in (('detector', detector == 'custom'), ('mission_brain', mission_brain == 'custom')):
        if on and not any(role == slot for role, _ in entries):
            actions.append(LogInfo(msg=f'[team_stack] WARNING: {slot}:=custom but no {slot}.custom entry in a '
                                       f'profile: start your {slot.replace("_", " ")} yourself (ros2 run ...)'))
    return actions


def generate_launch_description():
    """Entry point used by `ros2 launch` (and by ./launcher.sh through TRACKER_CMD)."""
    return LaunchDescription([
        DeclareLaunchArgument('detector', default_value='real',
                              description='real = new_od detects with model_path | truth = new_od idle '
                                          '(truth_detector) | custom = no new_od (a plug-in detector instead)'),
        DeclareLaunchArgument('mission_brain', default_value='team',
                              description='team = main_controller | custom = no main_controller (a plug-in '
                                          'mission brain instead)'),
        DeclareLaunchArgument('model_path', default_value=sp.detector_model_path(),
                              description='YOLO weights for new_od ("" keeps the team default)'),
        DeclareLaunchArgument('batched', default_value='false',
                              description='false = new_od SAHI path (works with .pt) | true = team batched default'),
        DeclareLaunchArgument('debug', default_value='false', description='tracker.launch.xml debug switch'),
        DeclareLaunchArgument('profile', default_value='',
                              description='a plug-in profile (simulation/profiles/<name>.yaml or a path): fills in '
                                          'the arguments left at their defaults and starts its plug-ins'),
        OpaqueFunction(function=_setup),
    ])
