"""
sim_plugins/test/test_launch_files.py -- PURPOSE: evaluate sim_bringup's two launch files with a FAKE launch
context and recording stand-ins for ExecuteProcess / Node / SetParameter. Nothing is started (no Gazebo, no
SITL, no ROS graph): the tests only look at the commands the launch files WOULD run.

  * default arguments -> exactly today's SITL and MAVProxy commands, today's camera-emulator parameters and the
    team's own tracker.launch.xml, with sim_plugins made un-importable (so the default path provably does not
    need it);
  * every example profile, and the new arguments one by one (another ArduCopter version, real-drone parameter
    files, an external autopilot, Docker network outputs, camera bind / strict, custom detector / mission brain /
    camera / extras) -> the right changes and nothing else.
Included launch files are found with conftest.include_location(): their location is performed in the
LaunchContext (and the file is read), like the launch system does; `.location` alone is not the path in Humble.
Needs ROS 2 Humble's Python libraries (launch, launch_ros, launch_xml): skipped without them.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

from conftest import PROFILES, include_location, write_fake_arducopter

SIM = Path(__file__).resolve().parents[2]            # ros2_ws/src/simulation
SRC = SIM.parent
LAUNCH_DIR = SIM / 'sim_bringup' / 'launch'
EXAMPLE_PARAM = PROFILES / 'params' / 'example_real_drone.param'
RUNWAY_HOME = '36.2143000,-96.0068000,210.0,0'
WAITER = '/fake/lib/sim_bringup/wait_then_exec'


@pytest.fixture(autouse=True)
def need_ros_launch():
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    pytest.importorskip('launch_xml')


class Recorder:
    """What the launch file asked for: processes, nodes, parameters."""

    def __init__(self):
        self.procs, self.nodes, self.params = [], [], []

    def proc(self, label):
        """The command of the process whose wait_then_exec --label (or first word) is `label`."""
        for kw in self.procs:
            cmd = [str(c) for c in kw['cmd']]
            if label in cmd[:9] or Path(cmd[0]).name == label:
                return kw
        return None

    def node(self, executable):
        return next((kw for kw in self.nodes if kw.get('executable') == executable), None)


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def install_recorders(mod, rec: Recorder):
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node, SetParameter

    class RecExec(ExecuteProcess):
        def __init__(self, **kw):
            rec.procs.append(kw)
            super().__init__(**kw)

    class RecNode(Node):
        def __init__(self, **kw):
            rec.nodes.append(kw)
            super().__init__(**kw)

    class RecParam(SetParameter):
        def __init__(self, name, value, **kw):
            rec.params.append((name, value))
            super().__init__(name=name, value=value, **kw)

    for attr, cls in (('ExecuteProcess', RecExec), ('Node', RecNode), ('SetParameter', RecParam)):
        if hasattr(mod, attr):
            setattr(mod, attr, cls)


def messages(actions, ctx):
    from launch.actions import LogInfo
    from launch.utilities import perform_substitutions
    return '\n'.join(perform_substitutions(ctx, a.msg) for a in actions if isinstance(a, LogInfo))


def shutdown_requested(actions):
    from launch.actions import EmitEvent
    return any(isinstance(a, EmitEvent) for a in actions)


# ---- sim.launch.py -------------------------------------------------------------------------------------

def run_sim(monkeypatch, tmp_path, args=None, block_plugins=False):
    if block_plugins:
        for name in [m for m in sys.modules if m == 'sim_plugins' or m.startswith('sim_plugins.')] + ['sim_plugins']:
            monkeypatch.setitem(sys.modules, name, None)
    mod = load(LAUNCH_DIR / 'sim.launch.py', 'sim_launch_under_test')
    rec = Recorder()
    install_recorders(mod, rec)
    shares = {p: str(SIM / p) for p in ('sim_models', 'sim_worlds', 'siyi_emulator', 'sim_demo', 'sim_bringup')}
    monkeypatch.setattr(mod, 'share_or_none', lambda p: shares.get(p))
    monkeypatch.setattr(mod, 'executable_or_none', lambda p, n: f'/fake/lib/{p}/{n}')
    monkeypatch.setattr(mod, 'bound_ports', lambda proto: set())
    sp = mod.sp
    monkeypatch.setattr(sp, 'find_ardupilot_gazebo', lambda: None)
    monkeypatch.setattr(sp, 'find_sitl_binary', lambda explicit='': explicit or '/fake/sitl/arducopter')
    monkeypatch.setattr(sp, 'find_param_file', lambda name: f'/fake/sitl/{name}')
    monkeypatch.setattr(sp, 'find_mavproxy', lambda explicit='': '/fake/venv/bin/mavproxy.py')
    monkeypatch.setattr(sp, 'wsl_networking_mode', lambda: '')
    from launch import LaunchContext
    ctx = LaunchContext()
    args = dict(args or {})
    args.setdefault('run_dir', str(tmp_path / 'state'))
    for name, default, _ in mod.ARGUMENTS:
        ctx.launch_configurations[name] = str(args.get(name, default))
    actions = mod._launch_setup(ctx)
    return rec, actions, ctx


def todays_sitl_cmd(binary='/fake/sitl/arducopter', defaults=None):
    defaults = defaults or ['/fake/sitl/copter.parm', '/fake/sitl/gazebo-iris.parm',
                            str(SIM / 'sim_models' / 'config' / 'astra_sitl.parm')]
    return [WAITER, '--udp-port', '9002', '--timeout', '420', '--label', 'sitl', '--', binary, '--model', 'JSON',
            '-I', '0', '--home', RUNWAY_HOME, '--defaults', ','.join(defaults)]


def todays_mavproxy_cmd(run_dir, outs=('udp:127.0.0.1:14550', 'udp:127.0.0.1:14552', 'udp:127.0.0.1:14553')):
    cmd = [WAITER, '--tcp-port', '5760', '--timeout', '480', '--delay', '1', '--label', 'mavproxy', '--',
           '/fake/venv/bin/mavproxy.py', '--master', 'tcp:127.0.0.1:5760']
    for o in outs:
        cmd += ['--out', o]
    return cmd + ['--daemon', '--state-basedir', str(run_dir), '--logfile', 'mav.tlog']


def test_sim_default_is_todays_commands_without_sim_plugins(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, block_plugins=True)
    assert not shutdown_requested(actions), messages(actions, ctx)
    sitl = rec.proc('sitl')
    assert [str(c) for c in sitl['cmd']] == todays_sitl_cmd()
    assert sitl['cwd'] == str(tmp_path / 'state' / 'sitl_I0')
    mp = rec.proc('mavproxy')
    assert [str(c) for c in mp['cmd']] == todays_mavproxy_cmd(tmp_path / 'state' / 'sitl_I0')
    emu = rec.node('siyi_emulator')
    overrides = emu['parameters'][-1]
    assert overrides == {'camera_model': 'zr10', 'camera_ip': '192.168.144.25',
                         'sd_card_dir': str(tmp_path / 'state' / 'sd_card_I0')}
    state = (tmp_path / 'state' / 'state' / 'current_launch.yaml').read_text()
    assert 'autopilot: sitl' in state and "profile: ''" in state and 'detector: real' in state


def test_sim_another_version(sim_home, monkeypatch, tmp_path):
    vdir = sim_home['versions'] / '4.7.1'
    prog = write_fake_arducopter(vdir / 'arducopter', '4.7.1')
    (vdir / 'copter.parm').write_text('A 1\n')
    (vdir / 'gazebo-iris.parm').write_text('B 2\n')
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'sitl_version': '4.7.1'})
    assert not shutdown_requested(actions), messages(actions, ctx)
    sitl = rec.proc('sitl')
    assert [str(c) for c in sitl['cmd']] == todays_sitl_cmd(
        str(prog), [str(vdir / 'copter.parm'), str(vdir / 'gazebo-iris.parm'),
                    str(SIM / 'sim_models' / 'config' / 'astra_sitl.parm')])
    run_dir = tmp_path / 'state' / 'sitl_I0_v4.7.1'
    assert sitl['cwd'] == str(run_dir)
    assert [str(c) for c in rec.proc('mavproxy')['cmd']] == todays_mavproxy_cmd(run_dir)
    assert 'ArduCopter 4.7.1' in messages(actions, ctx)


def test_sim_version_not_fetched_stops_with_the_fix(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'sitl_version': '9.9.9'})
    assert shutdown_requested(actions) and 'fetch_sitl.sh 9.9.9' in messages(actions, ctx)
    assert rec.proc('sitl') is None


def test_sim_real_drone_param_file(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'sitl_param_files': str(EXAMPLE_PARAM)})
    assert not shutdown_requested(actions), messages(actions, ctx)
    cmd = [str(c) for c in rec.proc('sitl')['cmd']]
    defaults = cmd[cmd.index('--defaults') + 1].split(',')
    assert defaults[:3] == todays_sitl_cmd()[-1].split(',')        # today's files first ...
    cleaned = Path(defaults[3])                                      # ... the real drone's file last (it wins)
    assert cleaned.parent == tmp_path / 'state' / 'generated' / 'params' and cleaned.is_file()
    assert 'INS_ACCOFFS_X' not in cleaned.read_text() and 'WPNAV_SPEED 500' in cleaned.read_text()
    assert Path(rec.proc('sitl')['cwd']).name.startswith('sitl_I0_params-')


def test_sim_legacy_sitl_bin_keeps_todays_folder(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'sitl_bin': '/my/arducopter', 'sitl_wipe': 'true'},
                                block_plugins=True)
    cmd = [str(c) for c in rec.proc('sitl')['cmd']]
    assert cmd[8] == '/my/arducopter' and cmd[-1] == '-w'
    assert rec.proc('sitl')['cwd'] == str(tmp_path / 'state' / 'sitl_I0')


def test_sim_external_autopilot(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'autopilot': 'external',
                                                        'mavlink_master': 'tcp:10.0.0.9:5760'})
    assert not shutdown_requested(actions), messages(actions, ctx)
    assert rec.proc('sitl') is None                                   # no SITL here
    mp = [str(c) for c in rec.proc('mavproxy.py')['cmd']]
    assert mp[:5] == ['/fake/venv/bin/mavproxy.py', '--master', 'tcp:10.0.0.9:5760', '--retries', '300']
    assert WAITER not in mp
    model = tmp_path / 'state' / 'generated' / 'iris_a8mini_zr10_I0' / 'model.sdf'
    assert '<fdm_addr>0.0.0.0</fdm_addr>' in model.read_text()
    assert '--sim-address=' in messages(actions, ctx)          # the command for the other computer is printed
    assert 'autopilot: external' in (tmp_path / 'state' / 'state' / 'current_launch.yaml').read_text()


def test_sim_external_without_mavlink_master_stops(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'autopilot': 'external'})
    assert shutdown_requested(actions) and 'mavlink_master' in messages(actions, ctx)


def test_sim_network_outputs_and_camera(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {
        'mavros_out': 'udp:172.18.0.3:14552', 'extra_outs': 'udp:10.0.0.5:14550,udp:10.0.0.6:14550',
        'gcs_out': 'udp:127.0.0.1:14560', 'camera_bind': '0.0.0.0', 'camera_strict': 'true'})
    assert not shutdown_requested(actions)
    mp = [str(c) for c in rec.proc('mavproxy')['cmd']]
    outs = [mp[i + 1] for i, w in enumerate(mp) if w == '--out']
    assert outs == ['udp:127.0.0.1:14560', 'udp:172.18.0.3:14552', 'udp:127.0.0.1:14553', 'udp:10.0.0.5:14550',
                    'udp:10.0.0.6:14550']
    overrides = rec.node('siyi_emulator')['parameters'][-1]
    assert overrides['bind_ip'] == '0.0.0.0' and overrides['strict'] is True


def test_sim_profile_example_detector(sim_home, monkeypatch, tmp_path):
    from launch.actions import IncludeLaunchDescription
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'profile': 'example_detector'})
    assert not shutdown_requested(actions), messages(actions, ctx)
    demo = [a for a in actions if isinstance(a, IncludeLaunchDescription)
            and include_location(a, ctx).endswith('demo_nodes.launch.py')][0]
    assert dict(demo.launch_arguments)['detector'] == 'real'           # custom = "a real detector" for the panel
    text = messages(actions, ctx)
    assert 'from the profile: detector:=custom' in text and 'from the profile: camera_max_zoom:=1' in text
    assert 'detector: custom' in (tmp_path / 'state' / 'state' / 'current_launch.yaml').read_text()


def test_sim_profile_typed_argument_wins(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'profile': 'truth_detector', 'camera_model': 'a8mini'})
    assert rec.node('siyi_emulator')['parameters'][-1]['camera_model'] == 'a8mini'
    assert 'from the profile: detector:=truth' in messages(actions, ctx)


def test_sim_profile_custom_camera_and_extras(sim_home, monkeypatch, tmp_path):
    from launch.actions import ExecuteProcess
    prof = tmp_path / 'cam.yaml'
    prof.write_text('camera:\n  kind: custom\n  custom: {name: mycam, command: [python3, /opt/cam.py]}\n'
                    'extras:\n  - {name: simlog, command: [python3, /opt/simlog.py], start_with: simulator}\n'
                    '  - {name: teamlog, command: [python3, /opt/teamlog.py]}\n')
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'profile': str(prof)})
    assert not shutdown_requested(actions), messages(actions, ctx)
    assert rec.node('siyi_emulator') is None                            # emulator:=false from the profile
    started = ' '.join(messages(actions, ctx).split())
    assert 'starting camera "mycam"' in started and 'starting extra "simlog"' in started
    assert 'teamlog' not in started                                      # starts with the team stack instead
    # the two programs come from launch_support (plain ExecuteProcess, not the recorder of sim.launch.py's own)
    assert len([a for a in actions if type(a) is ExecuteProcess]) == 2


@pytest.mark.parametrize('name', ['default', 'truth_detector', 'example_detector', 'example_listener',
                                  'real_drone_params', 'external_autopilot', 'docker_split'])
def test_sim_every_example_profile_evaluates(sim_home, monkeypatch, tmp_path, name):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'profile': name})
    assert not shutdown_requested(actions), messages(actions, ctx)


@pytest.mark.parametrize('name, words', [('ardupilot_version', 'fetch_sitl.sh 4.7.1'),
                                         ('ardupilot_source', 'source tree not found')])
def test_sim_profiles_that_need_downloads_stop_clearly(sim_home, monkeypatch, tmp_path, name, words):
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'profile': name})
    assert shutdown_requested(actions) and words in messages(actions, ctx)


def test_sim_bad_profile_stops(sim_home, monkeypatch, tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('detecter: {kind: truth}\n')
    rec, actions, ctx = run_sim(monkeypatch, tmp_path, {'profile': str(bad)})
    assert shutdown_requested(actions) and 'did you mean "detector"' in messages(actions, ctx)


# ---- team_stack.launch.py ------------------------------------------------------------------------------

def run_team(monkeypatch, args=None, block_plugins=False):
    if block_plugins:
        for name in [m for m in sys.modules if m == 'sim_plugins' or m.startswith('sim_plugins.')] + ['sim_plugins']:
            monkeypatch.setitem(sys.modules, name, None)
    mod = load(LAUNCH_DIR / 'team_stack.launch.py', 'team_launch_under_test')
    rec = Recorder()
    install_recorders(mod, rec)

    def share(pkg):
        if pkg == 'main':
            return str(SRC / 'main')
        raise mod.PackageNotFoundError(pkg)
    monkeypatch.setattr(mod, 'get_package_share_directory', share)
    if not block_plugins:
        from sim_plugins import plan
        monkeypatch.setattr(plan, '_package_share', lambda pkg: str(SIM / 'sim_plugin_examples' / pkg))
    from launch import LaunchContext
    ctx = LaunchContext()
    defaults = {'detector': 'real', 'mission_brain': 'team', 'model_path': mod.sp.detector_model_path(),
                'debug': 'false', 'batched': 'false', 'profile': ''}
    defaults.update(args or {})
    for k, v in defaults.items():
        ctx.launch_configurations[k] = str(v)
    actions = mod._setup(ctx)
    return rec, actions, ctx


TEAM_TRACKER = SRC / 'main' / 'launch' / 'tracker.launch.xml'


def tracker_include_action(actions):
    """The IncludeLaunchDescription of the tracker file (team_stack.launch.py puts it in a scoped GroupAction)."""
    from launch.actions import GroupAction, IncludeLaunchDescription
    group = [a for a in actions if isinstance(a, GroupAction)][0]
    return [a for a in group.get_sub_entities() if isinstance(a, IncludeLaunchDescription)][0]


def tracker_include(actions, ctx):
    """The tracker launch file team_stack.launch.py includes, its location performed in ctx."""
    return include_location(tracker_include_action(actions), ctx)


def nodes_of(description, ctx):
    """'pkg/exec' of every node a loaded LaunchDescription would start (its pkg/exec performed in ctx)."""
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    from launch_ros.actions import Node

    def text(value):
        return perform_substitutions(ctx, normalize_to_list_of_substitutions(value))
    return [f'{text(e.node_package)}/{text(e.node_executable)}' for e in description.entities if isinstance(e, Node)]


def tracker_nodes(actions, ctx):
    """The nodes the INCLUDED tracker file would start, read by the launch system (comments do not count)."""
    include = tracker_include_action(actions)
    include_location(include, ctx)
    return nodes_of(include.launch_description_source.get_launch_description(ctx), ctx)


def team_tracker_nodes(ctx):
    """The nodes the team's own tracker.launch.xml starts (read from the team's file, so a team change is seen)."""
    from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
    return nodes_of(XMLLaunchDescriptionSource(str(TEAM_TRACKER)).get_launch_description(ctx), ctx)


def test_team_default_is_todays_launch_without_sim_plugins(sim_home, monkeypatch):
    rec, actions, ctx = run_team(monkeypatch, block_plugins=True)
    assert tracker_include(actions, ctx) == str(TEAM_TRACKER)
    assert 'detection/new_od' in tracker_nodes(actions, ctx) and 'main/main_controller.py' in tracker_nodes(actions, ctx)
    assert rec.params == [('use_batched_inference', False)]           # (no weights in the test home)
    assert not shutdown_requested(actions)


def test_team_default_with_model(sim_home, monkeypatch):
    model = sim_home['home'] / 'models' / 'yolo26m.pt'
    model.parent.mkdir(parents=True)
    model.write_bytes(b'weights')
    rec, actions, ctx = run_team(monkeypatch, block_plugins=True)
    assert rec.params == [('model_path', str(model)), ('use_batched_inference', False)]


def test_team_truth(sim_home, monkeypatch):
    rec, actions, ctx = run_team(monkeypatch, {'detector': 'truth'}, block_plugins=True)
    assert ('camera_feed_path', str(sim_home['home'] / 'truth_mode_empty_feed')) in rec.params
    assert tracker_include(actions, ctx).endswith('main/launch/tracker.launch.xml')


def test_team_custom_detector_without_profile(sim_home, monkeypatch):
    rec, actions, ctx = run_team(monkeypatch, {'detector': 'custom'})
    path = Path(tracker_include(actions, ctx))
    assert path.parent == sim_home['home'] / 'generated'               # a generated copy, the team file untouched
    team = team_tracker_nodes(ctx)
    assert tracker_nodes(actions, ctx) == [n for n in team if n != 'detection/new_od']   # only new_od is gone
    assert rec.params == []                                           # no new_od parameters at all
    assert 'start your detector yourself' in messages(actions, ctx)


def test_team_profile_example_detector(sim_home, monkeypatch):
    from launch.actions import IncludeLaunchDescription
    rec, actions, ctx = run_team(monkeypatch, {'profile': 'example_detector'})
    assert tracker_nodes(actions, ctx) == [n for n in team_tracker_nodes(ctx) if n != 'detection/new_od']
    plugin = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert plugin and include_location(plugin[0], ctx).endswith('example_detector/launch/example_detector.launch.py')
    assert 'from the profile: detector:=custom' in messages(actions, ctx)


def test_team_profile_custom_mission_brain(sim_home, monkeypatch, tmp_path):
    from launch_ros.actions import Node
    prof = tmp_path / 'brain.yaml'
    prof.write_text('mission_brain:\n  kind: custom\n  custom: {package: my_brain, executable: brain, '
                    'node_name: brain, args: {hold_s: 4}}\n')
    rec, actions, ctx = run_team(monkeypatch, {'profile': str(prof)})
    assert tracker_nodes(actions, ctx) == [n for n in team_tracker_nodes(ctx) if n != 'main/main_controller.py']
    nodes = [a for a in actions if isinstance(a, Node)]
    assert nodes and nodes[0].node_package == 'my_brain' and nodes[0].node_executable == 'brain'


def test_team_profile_extra(sim_home, monkeypatch):
    from launch_ros.actions import Node
    rec, actions, ctx = run_team(monkeypatch, {'profile': 'example_listener'})
    assert tracker_include(actions, ctx).endswith('main/launch/tracker.launch.xml')
    assert [a.node_executable for a in actions if isinstance(a, Node)] == ['example_listener']


def test_team_bad_profile_stops(sim_home, monkeypatch, tmp_path):
    rec, actions, ctx = run_team(monkeypatch, {'profile': str(tmp_path / 'missing.yaml')})
    assert shutdown_requested(actions) and 'not found' in messages(actions, ctx)
