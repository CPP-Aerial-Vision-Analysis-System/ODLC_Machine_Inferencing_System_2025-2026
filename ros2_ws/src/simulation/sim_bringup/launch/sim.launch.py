"""
sim_bringup/launch/sim.launch.py -- PURPOSE: start the whole ASTRA simulator with one command.

    ros2 launch sim_bringup sim.launch.py [world:=runway|suas_field|/path/world.sdf] [camera_model:=zr10|a8mini]
        [gui:=true] [rviz:=true] [panel:=true] [headless_rendering:=false] [detector:=real|truth]
        [sitl:=true] [sitl_instance:=0] [mavproxy:=true] [windows_gcs:=true] [emulator:=true] [payload:=true]
        [bridge:=true] [markers:=true] [follow:=true]   (and more, see `ros2 launch sim_bringup sim.launch.py --show-args`)

What it starts, in this order (every part can be switched off with its argument):
  1. Gazebo server (gz sim -s -r) with the world, and the Gazebo window (gz sim -g) with the world's own
     layout (its <gui> section) or ours (config/gazebo_gui.config): big 3D view + a live camera POV panel.
     The window is a separate program: closing it does NOT stop the simulation.
  2. The drone: sim_models' model.sdf.xacro is run through xacro (photo size from camera_model, SITL
     port from sitl_instance) and spawned as entity "iris" at the world's spawn pose.
  3. ArduCopter SITL (the Pixhawk's firmware on the PC), started only when the drone's ArduPilotPlugin
     has opened UDP 9002+10*I, in ~/.astra_sim/sitl_I<I> (eeprom.bin = saved parameters persists there).
  4. MAVProxy, started when SITL listens on TCP 5760+10*I, with three outputs:
     udp 127.0.0.1:14550 ground station / our scripts, :14552 MAVROS (./launcher.sh sim), :14553 demo panel,
     plus (inside WSL, windows_gcs:=true) a fourth copy for Mission Planner / QGroundControl on WINDOWS:
     udp <Windows host>:14550, the address found at every start (sim_paths.windows_gcs_output).
  5. The SIYI camera emulator (pretends to be the camera at 192.168.144.25; its SD card is
     ~/.astra_sim/sd_card_I<I>) and the payload release node.
  6. ros_gz_bridge: /sim/camera/image, /sim/camera/camera_info, /sim/camera/photo, /sim/drone/odometry,
     /sim/gimbal/joint_states into ROS 2; odom_tf (drone TF for RViz).
  7. The demo nodes, through sim_demo's own launch file (demo_nodes.launch.py): demo_markers (RViz
     markers), demo_panel (the demo window), truth_detector (only with detector:=truth) and score_run
     (only with score:=true: records the run, writes the score report when the drone lands).
  8. RViz with rviz/sim.rviz (big camera POV + 3D view with markers), and gz_follow, which makes the
     Gazebo 3D view follow the drone once the window and the drone are there.
When a simulation package is not built yet, the launch says so and uses a stand-in (ardupilot_gazebo's
iris_runway world / iris_with_gimbal drone) or skips that part, instead of crashing.
The team's flight stack is NOT started here: run ./launcher.sh sim (or simulation/scripts/demo.sh).

Plug-and-play (package sim_plugins; docs/simulation/PLUGINS.md and docs/simulation/ARDUPILOT.md): the
arguments at the end of ARGUMENTS swap one piece for another, and every one of them defaults to today's
behaviour (with all of them at their defaults the commands are exactly the ones before they existed):
  profile:=NAME|FILE     a plug-in profile: its values for every argument left at its default, plus its
                         custom camera and its extras that start with the simulator
  autopilot:=external, sitl_version:=4.7.1, sitl_source:=/opt/ardupilot, sitl_param_files:=my.param, ...
                         which ArduCopter flies the drone (see sim_plugins/autopilot.py)
  mavlink_master, gcs_out, mavros_out, extra_outs, camera_bind, fdm_addr
                         where MAVLink goes and where the camera / the physics link listen (Docker, 2 PCs)
sim_plugins is imported only when one of these arguments is used, so the default start never needs it.
"""

import os
import re
import socket
import time
import uuid
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim_bringup import sim_paths as sp
from sim_bringup.wait_then_exec import bound_ports

#: (name, default, description) of every launch argument. `--show-args` prints these.
ARGUMENTS = [
    ('world', 'runway', 'runway | suas_field | path to a world .sdf | path to a world config .yaml'),
    ('camera_model', 'zr10', 'zr10 (2K photos, last year\'s default) | a8mini (4K photos)'),
    ('gui', 'true', 'open the Gazebo window (false = server only, no window)'),
    ('rviz', 'true', 'open RViz with the camera POV layout'),
    ('panel', 'true', 'open the demo panel (sim_demo demo_panel)'),
    ('headless_rendering', 'false', 'render the cameras without any display (EGL); for servers/CI'),
    ('detector', 'real', 'real = the team\'s new_od finds targets | truth = sim_demo truth_detector stand-in'),
    ('markers', 'true', 'start sim_demo demo_markers (RViz markers: search area, targets, footprints, track)'),
    ('score', 'false', 'start sim_demo score_run (records the run, writes the score report when it lands)'),
    ('demo_run_dir', '', 'folder for the score report ("" = sim_demo default ~/astra_sim_runs/<world>_<time>)'),
    ('auto_start', 'false', 'true = the demo panel uploads + starts the mission by itself (demo.sh does it instead)'),
    ('sitl', 'true', 'start ArduCopter SITL'),
    ('sitl_instance', '0', 'SITL instance I: ports 5760+10*I (MAVLink) and 9002+10*I (Gazebo link)'),
    ('sitl_wipe', 'false', 'true = wipe SITL\'s saved parameters/mission (eeprom.bin) at start'),
    ('sitl_bin', '', 'path to an arducopter binary (default: $ARDUPILOT_SITL_BIN, ~/sitl/arducopter, Docker path)'),
    ('mavproxy', 'true', 'start MAVProxy (routes MAVLink to the ground station, MAVROS and the panel)'),
    ('mavproxy_bin', '', 'path to mavproxy.py (default: $ASTRA_MAVPROXY, ~/sitl/venv/bin, PATH)'),
    ('gcs_port', str(sp.DEFAULT_GCS_PORT), 'MAVProxy UDP output for a ground station / our scripts'),
    ('mavros_port', str(sp.DEFAULT_MAVROS_PORT), 'MAVProxy UDP output for MAVROS (launcher.sh sim uses 14552)'),
    ('panel_port', str(sp.DEFAULT_PANEL_PORT), 'MAVProxy UDP output for the demo panel\'s mission buttons'),
    ('windows_gcs', 'true', 'also send MAVLink to Mission Planner / QGroundControl on WINDOWS when the simulator '
                            'runs in WSL (address found at start; nothing outside WSL). false = off'),
    ('windows_gcs_addr', '', 'where that Windows ground station listens: "" = automatic (WSL NAT: the Windows '
                             'host, UDP 14550; WSL mirrored: 127.0.0.1:14551), or HOST / HOST:PORT'),
    ('emulator', 'true', 'start the SIYI camera emulator (siyi_emulator)'),
    ('camera_ip', sp.DEFAULT_CAMERA_IP, 'address the emulator pretends to be (the real camera\'s address)'),
    ('camera_max_zoom', '', 'emulator max zoom ("" = camera default: zr10 30x, a8mini 1x). 1 keeps photos at '
                            'the full 81 deg view although video_cam racks to 5x'),
    ('payload', 'true', 'start the payload release node (sim_models payload_release)'),
    ('bridge', 'true', 'start ros_gz_bridge (camera images, odometry, gimbal angles into ROS 2)'),
    ('follow', 'true', 'make the Gazebo 3D view follow the drone (needs gui:=true)'),
    ('follow_offset', '-12,0,6', 'chase-camera position relative to the drone, metres "x,y,z"'),
    ('gui_config', '', 'Gazebo window layout: "" = the world\'s own <gui> if it has one, else ours '
                       '(config/gazebo_gui.config: 3D view + camera POV); "astra" = ours; "default" = Gazebo\'s; '
                       'or a file'),
    ('gz_verbosity', '1', 'Gazebo console verbosity 0-4'),
    ('model', '', 'override the drone model file (.sdf or .sdf.xacro); default = sim_models iris_a8mini'),
    ('x', '', 'spawn x [m] override (default: the world config)'),
    ('y', '', 'spawn y [m] override'),
    ('z', '', 'spawn z [m] override'),
    ('yaw', '', 'spawn yaw [rad] override'),
    ('photo_dir', '', 'the team\'s photo folder for the panel ("" = <team ros2_ws>/src/video_cam/mapping_photos)'),
    ('run_dir', '', 'state folder ("" = $ASTRA_SIM_HOME or ~/.astra_sim)'),
    # ---- plug-and-play slots (sim_plugins). Every default = today's behaviour. ----------------------------
    ('profile', '', 'a plug-in profile (a name in simulation/profiles or a .yaml path): its values are used for '
                    'every argument left at its default; its custom camera and simulator extras start here'),
    ('autopilot', 'sitl', 'sitl = start ArduCopter SITL here | external = an autopilot running elsewhere '
                          '(another container / PC; needs mavlink_master)'),
    ('sitl_version', '', 'an ArduCopter version fetched by simulation/setup/fetch_sitl.sh VERSION '
                         '(~/sitl/versions/<VERSION>/arducopter); "" = the installed default (4.6.3)'),
    ('sitl_source', '', 'an ArduPilot source tree with a built SITL (build/sitl/bin/arducopter), e.g. /opt/ardupilot'),
    ('sitl_param_files', '', 'extra parameter files loaded after astra_sitl.parm, comma list (e.g. the real '
                             'drone\'s .param from Mission Planner); hardware lines are filtered out'),
    ('sitl_param_filter', 'safe', 'safe = drop hardware/airframe lines from sitl_param_files | none = keep all'),
    ('sitl_param_keep', '', 'parameter names the safe filter keeps anyway, comma list'),
    ('sitl_run_dir', '', 'SITL working folder ("" = ~/.astra_sim/sitl_I<I>; another version / tree / '
                         'parameter set gets its own sitl_I<I>_<tag>)'),
    ('mavlink_master', '', 'where MAVProxy picks up MAVLink ("" = the local SITL, tcp:127.0.0.1:5760+10*I)'),
    ('fdm_addr', '', 'address the drone\'s ArduPilot plugin binds its physics port on ("" = the model\'s '
                     '127.0.0.1; 0.0.0.0 = reachable from another computer, used by autopilot:=external)'),
    ('gcs_out', '', 'MAVProxy ground-station output ("" = udp:127.0.0.1:<gcs_port>, which the scripts use)'),
    ('mavros_out', '', 'MAVProxy MAVROS output ("" = udp:127.0.0.1:<mavros_port>; e.g. udp:172.18.0.3:14552 '
                       'for a MAVROS in another container)'),
    ('extra_outs', '', 'more MAVProxy outputs, comma list (e.g. udp:192.168.1.20:14550 for a ground station)'),
    ('camera_bind', '', 'address the camera emulator listens on ("" = camera_ip if this PC has it, else '
                        '127.0.0.1; 0.0.0.0 = every address, for containers)'),
    ('camera_strict', '', 'true = the emulator stays silent like a real A8 mini on commands it ignores '
                          '("" = the emulator\'s own setting, false)'),
]

#: The autopilot-slot arguments: when all are at their defaults, SITL and MAVProxy start exactly as before.
PLUGIN_AUTOPILOT_ARGS = ('autopilot', 'sitl_version', 'sitl_source', 'sitl_param_files', 'sitl_param_filter',
                         'sitl_param_keep', 'sitl_run_dir', 'mavlink_master', 'fdm_addr')


def share_or_none(package: str):
    """Installed share folder of a package, or None (no exception) when it is not built/sourced."""
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        return None


def executable_or_none(package: str, name: str):
    """Absolute path of an installed console script (lib/<package>/<name>), or None."""
    try:
        path = os.path.join(get_package_prefix(package), 'lib', package, name)
    except PackageNotFoundError:
        return None
    return path if os.access(path, os.X_OK) else None


def _prepend_env(name: str, paths):
    """SetEnvironmentVariable action that puts `paths` in front of the current value of `name`."""
    current = [p for p in os.environ.get(name, '').split(':') if p]
    merged = []
    for p in list(paths) + current:
        if p and p not in merged:
            merged.append(p)
    return SetEnvironmentVariable(name, ':'.join(merged))


def _gui_env():
    """Extra environment for windows (Gazebo GUI, RViz, panel).

    In WSL2, Qt programs are more reliable on X11 (xcb) than on Wayland, especially RViz's OGRE view.
    A QT_QPA_PLATFORM that the user already set (e.g. 'offscreen' in tests) is kept.
    """
    env = {}
    if os.environ.get('QT_QPA_PLATFORM'):
        env['QT_QPA_PLATFORM'] = os.environ['QT_QPA_PLATFORM']
    elif _running_in_wsl():
        env['QT_QPA_PLATFORM'] = 'xcb'
    return env


def _running_in_wsl() -> bool:
    """True inside Windows' WSL2 (its kernel version string mentions Microsoft)."""
    try:
        return 'microsoft' in Path('/proc/version').read_text(errors='ignore').lower()
    except OSError:
        return False


def _launch_setup(context):
    """Build every action from the resolved launch arguments (runs once when the launch starts)."""
    cfg = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGUMENTS}
    actions = []

    def say(msg):
        actions.append(LogInfo(msg=f'[sim] {msg}'))

    def fatal(msg):
        actions.append(LogInfo(msg=f'[sim] ERROR: {msg}'))
        actions.append(EmitEvent(event=Shutdown(reason=msg)))
        return actions

    # ---- 0. arguments --------------------------------------------------------------------------
    # A plug-in profile (sim_plugins) first: it fills in every argument the user left at its default.
    profile_rp = None
    if cfg['profile'].strip():
        try:
            from sim_plugins import launch_support, plan as plug_plan
        except ImportError:
            return fatal('profile:= needs the sim_plugins package: build it with simulation/setup/03_build.sh')
        try:
            profile_rp = launch_support.load_profile(cfg['profile'])
        except Exception as exc:  # noqa: BLE001 - sim_plugins.profiles.ProfileError: every problem in words
            return fatal(f'the profile "{cfg["profile"]}" cannot be used:\n{exc}')
        say(f'plug-in profile "{profile_rp.name}" ({profile_rp.source})')
        arg_defaults = {name: default for name, default, _ in ARGUMENTS}
        for name, _, new in launch_support.apply_profile_args(cfg, arg_defaults,
                                                              plug_plan.sim_args_full(profile_rp)):
            say(f'  from the profile: {name}:={new}')
        for warning in profile_rp.warnings:
            say(f'  profile warning: {warning.text()}')
    try:
        instance = int(cfg['sitl_instance'])
        ports = sp.sitl_ports(instance)
        gcs_port, mavros_port, panel_port = (int(cfg[k]) for k in ('gcs_port', 'mavros_port', 'panel_port'))
    except ValueError as exc:
        return fatal(f'bad number in the launch arguments: {exc}')
    camera = cfg['camera_model'].strip().lower().replace('-', '').replace('_', '')
    if camera not in sp.CAMERA_PROFILES:
        say(f'camera_model "{cfg["camera_model"]}" is unknown (use zr10 or a8mini); using zr10')
        camera = 'zr10'
    detector = cfg['detector'].strip().lower()
    if detector not in ('real', 'truth', 'custom'):
        say(f'detector "{cfg["detector"]}" is unknown (use real, truth or custom); using real')
        detector = 'real'
    # custom = a teammate's detector, started with the team stack (team_stack.launch.py). For the demo nodes it
    # counts as a real detector: no truth_detector, no answer-key banner.
    demo_detector = 'truth' if detector == 'truth' else 'real'
    # The autopilot slot (sim_plugins.autopilot), only when one of its arguments is not at its default.
    arg_defaults = {name: default for name, default, _ in ARGUMENTS}
    external = cfg['autopilot'].strip().lower() in ('external', 'remote')
    plug_ap = ap_settings = None
    if any(cfg[n].strip() != arg_defaults[n] for n in PLUGIN_AUTOPILOT_ARGS):
        try:
            from sim_plugins import autopilot as plug_ap
        except ImportError:
            return fatal('the autopilot arguments (autopilot, sitl_version, sitl_source, sitl_param_files, '
                         'mavlink_master, fdm_addr ...) need the sim_plugins package: build it with '
                         'simulation/setup/03_build.sh')
        ap_settings = plug_ap.settings_from_launch(cfg)
    fdm_addr = cfg['fdm_addr'].strip() or ('0.0.0.0' if external else '')
    gui = sp.truthy(cfg['gui'])
    # Windows started below must not inherit Qt plugin paths from a Python wheel (OpenCV sets them when
    # something imports cv2), or Gazebo / RViz / the panel abort with "Could not load the Qt platform plugin".
    for name in sp.scrub_foreign_qt_env():
        say(f'removed {name} from the environment (it pointed into a Python package, e.g. OpenCV\'s Qt)')
    state_home = Path(sp.expand(cfg['run_dir'])) if cfg['run_dir'] else sp.sim_home()
    gen_dir = sp.generated_dir(state_home)
    gen_dir.mkdir(parents=True, exist_ok=True)

    # ---- 1. which packages / tools are there ---------------------------------------------------
    shares = {pkg: share_or_none(pkg) for pkg in ('sim_models', 'sim_worlds', 'siyi_emulator', 'sim_demo',
                                                   'sim_bringup')}
    for pkg in ('sim_models', 'sim_worlds', 'siyi_emulator', 'sim_demo'):
        if not shares[pkg]:
            say(f'package {pkg} is not built: its part is replaced by a stand-in or skipped '
                '(build everything with simulation/setup/03_build.sh)')
    ag_dir = sp.find_ardupilot_gazebo()
    if not ag_dir:
        say('ArduPilot Gazebo plugin not found (~/ardupilot_gazebo/build or /opt/ardupilot_gazebo/build): '
            'the drone cannot talk to SITL. Run simulation/setup/02_setup_ardupilot.sh')

    # ---- 2. world ------------------------------------------------------------------------------
    world_cfg = sp.load_world_config(cfg['world'], shares['sim_worlds'])
    if not world_cfg.world_file and (shares['sim_worlds'] is None or cfg['world'] in ('standin', 'iris_runway')):
        for n in world_cfg.notes:
            say(n)
        world_cfg = sp.standin_world_config(ag_dir, gen_dir)
    for n in world_cfg.notes:
        say(n)

    # Gazebo looks for model:// folders in GZ_SIM_RESOURCE_PATH and for system plugins (the
    # ArduPilotPlugin) in GZ_SIM_SYSTEM_PLUGIN_PATH.
    resource = []
    if shares['sim_models']:
        resource.append(os.path.join(shares['sim_models'], 'models'))
    for wdir in dict.fromkeys(d for d in (world_cfg.worlds_dir, shares['sim_worlds']) if d):
        resource += [os.path.join(wdir, 'models'), os.path.join(wdir, 'worlds')]
    if ag_dir:
        resource += [os.path.join(ag_dir, 'models'), os.path.join(ag_dir, 'worlds')]
    actions.append(_prepend_env('GZ_SIM_RESOURCE_PATH', [p for p in resource if os.path.isdir(p)]))
    if ag_dir:
        actions.append(_prepend_env('GZ_SIM_SYSTEM_PLUGIN_PATH', [os.path.join(ag_dir, 'build')]))
    actions.append(SetEnvironmentVariable('GZ_VERSION', 'harmonic'))
    if not world_cfg.world_file:
        return fatal(f'no world file for world:={cfg["world"]}. Known worlds: runway, suas_field '
                     '(sim_worlds/config/<name>.yaml), or give a path to an .sdf file.')
    if ' ' in world_cfg.world_file:
        say('WARNING: the world path contains a space; Gazebo may not open it. Use the Linux-side copy '
            '(simulation/setup/03_build.sh) instead of /mnt/... paths.')
    world_name = sp.world_name_from_sdf(Path(world_cfg.world_file).read_text(encoding='utf-8', errors='replace'))
    spawn = dict(world_cfg.spawn)
    for key in ('x', 'y', 'z', 'yaw'):
        if cfg[key] != '':
            spawn[key] = float(cfg[key])
    say(f'world "{world_cfg.name}" ({world_name}): {world_cfg.world_file}')
    say(f'SITL home {world_cfg.home}; spawn x={spawn["x"]:.2f} y={spawn["y"]:.2f} z={spawn["z"]:.2f} '
        f'yaw={spawn["yaw"]:.3f} rad; mission {world_cfg.mission or "-"}; truth {world_cfg.truth or "-"}')

    # A second simulator with the same SITL instance would steal the ports: refuse early and clearly.
    if sp.truthy(cfg['sitl']):
        busy = []
        if ports['fdm'] in bound_ports('udp'):
            busy.append(f'UDP {ports["fdm"]}')
        if ports['serial0'] in bound_ports('tcp'):
            busy.append(f'TCP {ports["serial0"]}')
        if busy:
            return fatal(f'SITL instance {instance} ports are already in use ({", ".join(busy)}): another '
                         'simulator is still running. Stop it with simulation/scripts/stop_sim.sh, or start '
                         f'this one with sitl_instance:={instance + 1}.')

    # ---- 3. drone model ------------------------------------------------------------------------
    model_file, notes = ('', [])
    if shares['sim_models'] or cfg['model']:
        model_file, notes = sp.generate_drone_sdf(shares['sim_models'], camera, instance, gen_dir, cfg['model'])
    standin_model = False
    if not model_file:
        more = []
        model_file, more = sp.standin_drone_sdf(ag_dir, instance, gen_dir)
        notes += more
        standin_model = bool(model_file)
    for n in notes:
        say(n)
    if fdm_addr and model_file and plug_ap is not None:
        # The plugin binds its JSON physics port on <fdm_addr> (127.0.0.1 in the model): 0.0.0.0 lets a SITL on
        # another computer / container reach it. The plugin answers whoever sends, so nothing else changes.
        text, found = plug_ap.patch_plugin_fdm_addr(Path(model_file).read_text(encoding='utf-8'), fdm_addr)
        if found:
            Path(model_file).write_text(text, encoding='utf-8')
            say(f'drone ArduPilot plugin: physics port UDP {ports["fdm"]} bound on {fdm_addr}')
        else:
            say(f'WARNING: fdm_addr:={fdm_addr} but the drone model has no ArduPilotPlugin to change')
    say(f'camera profile: {sp.CAMERA_PROFILES[camera]["label"]}')

    # ---- 4. Gazebo server + window ---------------------------------------------------------------
    gz_server = ['gz', 'sim', '-s', '-r', '-v', cfg['gz_verbosity']]
    if sp.truthy(cfg['headless_rendering']):
        gz_server.append('--headless-rendering')
    gz_server.append(world_cfg.world_file)
    server = ExecuteProcess(cmd=gz_server, name='gz_server', output='screen',
                            sigterm_timeout='10', sigkill_timeout='5')
    actions.append(server)
    actions.append(RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
        LogInfo(msg='[sim] the Gazebo server stopped -> stopping the whole simulator'),
        EmitEvent(event=Shutdown(reason='Gazebo server exited'))])))

    if gui:
        gui_cmd = ['gz', 'sim', '-g', '-v', cfg['gz_verbosity']]
        layout = cfg['gui_config'].strip()
        world_has_gui = re.search(r'<gui[\s>]', Path(world_cfg.world_file).read_text(encoding='utf-8',
                                                                                    errors='replace')) is not None
        if layout in ('', 'world') and world_has_gui:
            say('Gazebo window: the world\'s own layout (its <gui> section)')
        elif layout in ('', 'world', 'astra'):
            ours = os.path.join(shares['sim_bringup'] or '', 'config', 'gazebo_gui.config')
            if os.path.isfile(ours) and standin_model:
                # The stand-in drone's camera publishes on a different topic: use a patched copy.
                text = Path(ours).read_text(encoding='utf-8').replace(
                    '/sim/camera/video/image', sp.bridge_entries(True, world_name)[0]['gz_topic_name'])
                ours = str(gen_dir / 'gazebo_gui_standin.config')
                Path(ours).write_text(text, encoding='utf-8')
            if os.path.isfile(ours):
                gui_cmd += ['--gui-config', ours]
        elif layout == 'default':
            stock = '/usr/share/gz/gz-sim8/gui/gui.config'      # Gazebo's own layout (ignores the world's)
            if os.path.isfile(stock):
                gui_cmd += ['--gui-config', stock]
        else:
            gui_cmd += ['--gui-config', sp.expand(layout)]
        window = ExecuteProcess(cmd=gui_cmd, name='gz_gui', output='screen', additional_env=_gui_env())
        actions.append(window)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=window, on_exit=[
            LogInfo(msg='[sim] the Gazebo window was closed; the simulation keeps running '
                        '(stop everything with Ctrl+C or simulation/scripts/stop_sim.sh)')])))

    # ---- 5. spawn the drone ---------------------------------------------------------------------
    if model_file:
        actions.append(Node(
            package='ros_gz_sim', executable='create', name='spawn_drone', output='screen',
            arguments=['-file', model_file, '-name', sp.DRONE_ENTITY,
                       '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(spawn['z']),
                       '-Y', str(spawn['yaw'])]))
    else:
        say('NO DRONE MODEL (neither sim_models nor ardupilot_gazebo found): nothing is spawned')

    waiter = executable_or_none('sim_bringup', 'wait_then_exec')

    # ---- 6. ArduCopter SITL ---------------------------------------------------------------------
    run_dir = sp.sitl_run_dir(instance, state_home)
    win_out = ''            # MAVProxy's extra output for a ground station on Windows ('' = none), set in step 7
    sitl_used = ''          # the arducopter program that was started (written into the state file)
    ap_plan = None          # sim_plugins AutopilotPlan: another version / source tree / parameter files / external
    if ap_settings is not None and ap_settings.needs_plan():
        if not ap_settings.run_dir:
            # Another version / tree / parameter set keeps its own saved parameters (eeprom.bin) in its own
            # folder, so switching back and forth never mixes them.
            tag = plug_ap.flavour_tag(ap_settings)
            ap_settings.run_dir = str(state_home / f'sitl_I{instance}_{tag}') if tag else str(run_dir)
        ap_plan = plug_ap.plan_autopilot(ap_settings, params_out_dir=gen_dir / 'params',
                                         binary_finder=lambda: sp.find_sitl_binary(''))
        run_dir = Path(ap_plan.run_dir)
        say(f'autopilot: {ap_plan.how}')
        if ap_plan.errors:
            return fatal('the autopilot cannot start: ' + ' | '.join(ap_plan.errors))

    def add_mavproxy(waiter_prefix):
        """Step 7: MAVProxy and its outputs. waiter_prefix = the wait_then_exec words ([] = start at once)."""
        nonlocal win_out
        mavproxy = sp.find_mavproxy(cfg['mavproxy_bin'])
        if not mavproxy:
            say('MAVProxy not found (~/sitl/venv/bin/mavproxy.py or mavproxy.py on PATH): '
                f'connect a ground station to TCP 127.0.0.1:{ports["serial0"]} yourself')
            return
        # Mission Planner / QGroundControl on WINDOWS cannot hear the 127.0.0.1 outputs (in WSL2's
        # NAT networking Ubuntu is a separate computer behind Windows): give them their own copy,
        # sent to the Windows host. The 127.0.0.1:gcs_port output our scripts use is unchanged.
        win_out, win_notes = sp.windows_gcs_output(cfg['windows_gcs'], cfg['windows_gcs_addr'],
                                                   local_ports=(gcs_port, mavros_port, panel_port))
        for n in win_notes:
            say(n)
        # Plug-and-play outputs (Docker / two computers); all default to today's 127.0.0.1 ports.
        master = cfg['mavlink_master'].strip() or f'tcp:127.0.0.1:{ports["serial0"]}'
        gcs_out = cfg['gcs_out'].strip() or f'udp:127.0.0.1:{gcs_port}'
        mavros_out = cfg['mavros_out'].strip() or f'udp:127.0.0.1:{mavros_port}'
        extra_outs = [o.strip() for o in cfg['extra_outs'].split(',') if o.strip()]
        master_args = plug_ap.mavproxy_master_args(master, external) if plug_ap else ['--master', master]
        mp_cmd = [mavproxy, *master_args,
                  '--out', gcs_out, '--out', mavros_out,
                  '--out', f'udp:127.0.0.1:{panel_port}', *[w for o in extra_outs for w in ('--out', o)],
                  *(['--out', win_out] if win_out else []),
                  '--daemon', '--state-basedir', str(run_dir), '--logfile', 'mav.tlog']
        run_dir.mkdir(parents=True, exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=[*waiter_prefix, *mp_cmd],
            cwd=str(run_dir), name='mavproxy', output='screen', sigterm_timeout='5', sigkill_timeout='3'))
        say(f'MAVProxy outputs: ground station {gcs_out}, MAVROS {mavros_out} '
            f'(fcu_url udp://:{mavros_port}@localhost:{mavros_port}), demo panel udp:127.0.0.1:{panel_port}'
            + ''.join(f', extra {o}' for o in extra_outs)
            + (f', Windows ground station {win_out}' if win_out else '')
            + (f'; MAVLink from {master}' if cfg['mavlink_master'].strip() else ''))

    if external:
        # autopilot:=external: ArduCopter runs in another container / on another PC. Nothing to start here
        # except MAVProxy, which picks up its MAVLink and shares it out exactly like for the local SITL.
        say(f'autopilot:=external: no SITL here. Start ArduCopter on the other computer, e.g.: '
            f'{plug_ap.remote_command(ap_settings, world_cfg.home)}')
        if sp.truthy(cfg['mavproxy']):
            add_mavproxy([])
        else:
            say(f'mavproxy:=false: connect your ground station / MAVROS to {ap_plan.mavlink_master} yourself')
    elif sp.truthy(cfg['sitl']):
        astra_parm = os.path.join(shares['sim_models'], 'config', 'astra_sitl.parm') if shares['sim_models'] else None
        if ap_plan is None:         # today's code path (also sitl_bin:= / sitl_wipe:= on their own)
            sitl_bin = sp.find_sitl_binary(cfg['sitl_bin'])
            defaults, notes = sp.sitl_defaults(astra_parm)
        else:                       # another version / source tree / extra parameter files (sim_plugins)
            sitl_bin = ap_plan.binary
            defaults = ap_plan.defaults_list(astra_parm, finder=sp.find_param_file)
            notes = list(ap_plan.notes)
            say(f'autopilot: {plug_ap.describe(ap_plan, ap_settings)}')
        for n in notes:
            say(n)
        if not sitl_bin:
            say('ArduCopter SITL not found ($ARDUPILOT_SITL_BIN, ~/sitl/arducopter, '
                '/opt/ardupilot/build/sitl/bin/arducopter): run simulation/setup/02_setup_ardupilot.sh')
        elif not waiter:
            say('sim_bringup wait_then_exec is not installed (rebuild sim_bringup): SITL not started')
        else:
            run_dir.mkdir(parents=True, exist_ok=True)
            sitl_used = sitl_bin
            sitl_cmd = [sitl_bin, '--model', 'JSON', '-I', str(instance), '--home', world_cfg.home]
            if defaults:
                sitl_cmd += ['--defaults', ','.join(defaults)]
            if sp.truthy(cfg['sitl_wipe']):
                sitl_cmd.append('-w')
            # SITL may only start once the drone's ArduPilotPlugin listens (Gazebo up + drone spawned).
            actions.append(ExecuteProcess(
                cmd=[waiter, '--udp-port', str(ports['fdm']), '--timeout', '420', '--label', 'sitl', '--', *sitl_cmd],
                cwd=str(run_dir), name='sitl', output='screen', sigterm_timeout='5', sigkill_timeout='5'))
            say(f'SITL instance {instance}: MAVLink TCP {ports["serial0"]}, Gazebo link UDP {ports["fdm"]}, '
                f'run folder {run_dir} (wait ~30 s after boot before arming: "Arm: Gyros inconsistent" is normal)')

            # ---- 7. MAVProxy ----------------------------------------------------------------------
            if sp.truthy(cfg['mavproxy']):
                add_mavproxy([waiter, '--tcp-port', str(ports['serial0']), '--timeout', '480', '--delay', '1',
                              '--label', 'mavproxy', '--'])
            else:
                say(f'mavproxy:=false: SITL waits for a ground station on TCP 127.0.0.1:{ports["serial0"]}')
    elif sp.truthy(cfg['mavproxy']):
        say('sitl:=false: MAVProxy is not started either (it has nothing to talk to)')

    # ---- 8. SIYI camera emulator + payload release ---------------------------------------------
    camera_ip = cfg['camera_ip'].strip() or sp.DEFAULT_CAMERA_IP
    if sp.truthy(cfg['emulator']):
        if executable_or_none('siyi_emulator', 'siyi_emulator'):
            # The emulator's own parameter file first, then our overrides (later entries win).
            emu_params = []
            emu_yaml = os.path.join(shares['siyi_emulator'] or '', 'config', 'siyi_emulator.yaml')
            if os.path.isfile(emu_yaml):
                emu_params.append(emu_yaml)
            overrides = {'camera_model': camera, 'camera_ip': camera_ip,
                         'sd_card_dir': str(state_home / f'sd_card_I{instance}')}
            if cfg['camera_max_zoom'].strip():
                try:
                    overrides['max_zoom'] = float(cfg['camera_max_zoom'])
                    say(f'camera max zoom := {overrides["max_zoom"]:g}x (photos keep '
                        f'{"the full view" if overrides["max_zoom"] <= 1 else "the zoomed view"})')
                except ValueError:
                    say(f'camera_max_zoom "{cfg["camera_max_zoom"]}" is not a number; using the camera default')
            # Plug-and-play (camera slot): where it listens (0.0.0.0 in containers) and the strict A8 mini mode.
            if cfg['camera_bind'].strip():
                overrides['bind_ip'] = cfg['camera_bind'].strip()
                say(f'camera emulator listens on {overrides["bind_ip"]} (camera_bind)')
            if cfg['camera_strict'].strip():
                overrides['strict'] = sp.truthy(cfg['camera_strict'])
                say(f'camera emulator strict mode := {str(overrides["strict"]).lower()}')
            emu_params.append(overrides)
            actions.append(Node(package='siyi_emulator', executable='siyi_emulator', name='siyi_emulator',
                                output='screen', parameters=emu_params))
            if not cfg['camera_bind'].strip() and not _address_exists(camera_ip):
                say(f'the camera address {camera_ip} does not exist on this PC: the emulator will use its '
                    'fallback address and the team\'s video_cam will NOT reach it. Add it with '
                    'simulation/setup/camera_ip_alias.sh (start_sim.sh and demo.sh do this for you).')
        else:
            say('siyi_emulator is not built: no SIYI camera on the network (video_cam captures will fail)')
    if sp.truthy(cfg['payload']):
        if executable_or_none('sim_models', 'payload_release'):
            actions.append(Node(package='sim_models', executable='payload_release', name='payload_release',
                                output='screen'))
        else:
            say('sim_models payload_release is not built: payloads will not fall')

    # ---- 9. ROS <-> Gazebo bridge + drone TF ----------------------------------------------------
    if sp.truthy(cfg['bridge']):
        entries = sp.bridge_entries(standin=standin_model, world_name=world_name)
        bridge_yaml = sp.write_bridge_yaml(entries, gen_dir / f'bridge_I{instance}.yaml')
        actions.append(Node(package='ros_gz_bridge', executable='parameter_bridge', name='sim_bridge',
                            output='screen', parameters=[{'config_file': bridge_yaml}]))
        say('bridge: ' + ', '.join(f'{e["gz_topic_name"]} -> {e["ros_topic_name"]}' for e in entries))
        if not standin_model:
            actions.append(Node(package='sim_bringup', executable='odom_tf', name='odom_tf', output='screen'))

    # ---- 10. demo nodes (sim_demo) --------------------------------------------------------------
    # sim_demo's own launch file knows its nodes' parameters: include it (panel, markers, truth_detector,
    # score_run). Only when that file is missing are the nodes started one by one with the basics.
    photo_dir = sp.team_photo_dir(cfg['photo_dir'])
    gui_env = _gui_env()
    demo_launch = os.path.join(shares['sim_demo'] or '', 'launch', 'demo_nodes.launch.py')
    if shares['sim_demo'] and os.path.isfile(demo_launch):
        demo_args = {
            'world': world_cfg.name, 'detector': demo_detector, 'panel': str(sp.truthy(cfg['panel'])).lower(),
            'markers': str(sp.truthy(cfg['markers'])).lower(), 'score': str(sp.truthy(cfg['score'])).lower(),
            'photo_dir': cfg['photo_dir'], 'run_dir': sp.expand(cfg['demo_run_dir']),
            'mavlink_url': f'udpin:127.0.0.1:{panel_port}', 'auto_start': str(sp.truthy(cfg['auto_start'])).lower(),
            'frame_id': 'world', 'worlds_dir': world_cfg.worlds_dir,
        }
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(demo_launch),
                                                launch_arguments=demo_args.items()))
        parts = [n for n, on in (('panel', sp.truthy(cfg['panel'])), ('markers', sp.truthy(cfg['markers'])),
                                 ('truth_detector', detector == 'truth'), ('score_run', sp.truthy(cfg['score'])))
                 if on]
        say(f'sim_demo: {", ".join(parts) or "nothing"} (sim_demo/launch/demo_nodes.launch.py)')
    elif shares['sim_demo']:
        basic = {'world': world_cfg.name, 'photo_dir': cfg['photo_dir'], 'detector': demo_detector}
        for exe, on in (('demo_markers', sp.truthy(cfg['markers'])), ('truth_detector', detector == 'truth'),
                        ('demo_panel', sp.truthy(cfg['panel'])), ('score_run', sp.truthy(cfg['score']))):
            if on and executable_or_none('sim_demo', exe):
                extra = dict(basic, run_dir=sp.expand(cfg['demo_run_dir'])) if exe == 'score_run' else dict(basic)
                if exe == 'demo_panel':
                    extra['mavlink_url'] = f'udpin:127.0.0.1:{panel_port}'
                actions.append(Node(package='sim_demo', executable=exe, name=exe, output='screen',
                                    parameters=[extra], additional_env=gui_env))
    if detector == 'truth':
        say('detector:=truth -> sim_demo truth_detector publishes /image_detection (the panel says so)')
    elif detector == 'custom':
        say('detector:=custom -> a teammate\'s detector publishes /image_detection (started with the team stack: '
            'team_stack.launch.py detector:=custom profile:=...)')
    say(f'team photo folder (the panel shows the newest photo): {photo_dir}')

    # ---- 10b. plug-in profile: custom camera + extras that start with the simulator ----------------
    if profile_rp is not None:
        for role, entry in plug_plan.sim_side_entries(profile_rp):
            actions.extend(launch_support.entry_actions(entry, role, tag='sim'))

    # ---- 11. RViz + Gazebo follow camera ---------------------------------------------------------
    if sp.truthy(cfg['rviz']):
        rviz_cfg = os.path.join(shares['sim_bringup'] or '', 'rviz', 'sim.rviz')
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                            arguments=['-d', rviz_cfg] if os.path.isfile(rviz_cfg) else [],
                            additional_env=gui_env))
    if gui and sp.truthy(cfg['follow']):
        follower = executable_or_none('sim_bringup', 'gz_follow')
        try:
            offset = [str(float(v)) for v in cfg['follow_offset'].split(',')]
            assert len(offset) == 3
        except (ValueError, AssertionError):
            say(f'follow_offset "{cfg["follow_offset"]}" is not "x,y,z"; using -12,0,6')
            offset = ['-12', '0', '6']
        if follower:
            actions.append(ExecuteProcess(cmd=[follower, '--entity', sp.DRONE_ENTITY, '--offset', *offset],
                                          name='gz_follow', output='screen'))

    # ---- 12. remember what was started (for status.sh, demo.sh and the scripts) -----------------
    _write_state(state_home, {
        'session': os.environ.get('ASTRA_SIM_SESSION', ''), 'started': time.strftime('%Y-%m-%d %H:%M:%S'),
        'world': world_cfg.name, 'world_name': world_name, 'world_file': world_cfg.world_file,
        'home': world_cfg.home, 'spawn': spawn, 'mission': world_cfg.mission, 'truth': world_cfg.truth,
        'worlds_dir': world_cfg.worlds_dir,
        'standin_world': world_cfg.standin, 'standin_model': standin_model, 'model_file': model_file,
        'camera_model': camera, 'detector': detector, 'sitl_instance': instance, 'ports': ports,
        'gcs_port': gcs_port, 'mavros_port': mavros_port, 'panel_port': panel_port, 'windows_gcs': win_out,
        'camera_ip': camera_ip,
        'sitl_run_dir': str(run_dir), 'photo_dir': photo_dir,
        'autopilot': 'external' if external else 'sitl', 'sitl_bin': sitl_used,
        'profile': profile_rp.source if profile_rp is not None else '',
        'gz_partition': os.environ.get('GZ_PARTITION', ''), 'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
    }, actions)
    return actions


def _address_exists(ip: str) -> bool:
    """True when `ip` is an address of this machine (we can bind a UDP socket to it)."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.bind((ip, 0))
        return True
    except OSError:
        return False


def _write_state(state_home: Path, info: dict, actions: list) -> None:
    """Write state/current_launch.yaml (what this launch resolved). Failure is only reported."""
    try:
        state = state_home / 'state'
        state.mkdir(parents=True, exist_ok=True)
        (state / 'current_launch.yaml').write_text(
            '# Written by sim_bringup sim.launch.py at start: what the running simulator uses.\n'
            + yaml.safe_dump(info, sort_keys=False), encoding='utf-8')
    except OSError as exc:
        actions.append(LogInfo(msg=f'[sim] could not write the state file: {exc}'))


def generate_launch_description():
    """Entry point used by `ros2 launch`."""
    decls = [DeclareLaunchArgument(name, default_value=default, description=desc)
             for name, default, desc in ARGUMENTS]
    # Every process started below inherits this tag; stop_sim.sh uses it to stop exactly this simulator.
    session = [] if os.environ.get('ASTRA_SIM_SESSION') else [
        SetEnvironmentVariable('ASTRA_SIM_SESSION', f'launch-{uuid.uuid4().hex[:12]}')]
    # Keep Gazebo's messages in their own "partition" so other Gazebo programs on this PC do not mix in.
    partition = [] if os.environ.get('GZ_PARTITION') else [SetEnvironmentVariable('GZ_PARTITION', 'astra_sim')]
    # Qt windows (Gazebo, RViz, demo panel) work best on X11 inside WSL2 (see _gui_env).
    qt = [SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')] if (_running_in_wsl() and
                                                                 not os.environ.get('QT_QPA_PLATFORM')) else []
    return LaunchDescription(decls + session + partition + qt + [OpaqueFunction(function=_launch_setup)])
