"""
sim_plugins/test/test_plan.py -- PURPOSE: prove that a profile turns into exactly the right commands (plan.py):

  * the DEFAULT plan adds nothing: the demo.sh / start_sim.sh variables equal the scripts' own defaults (read
    from demo.sh itself), every plug-in argument list is empty, and plan.SIM_LAUNCH_DEFAULTS equals the
    defaults written in sim.launch.py (read from the launch file itself);
  * every example profile produces exactly its documented arguments;
  * the environment check finds missing programs / packages / files with fake finders (no ROS needed).
"""

import ast
import re
from pathlib import Path

import pytest

from sim_plugins import plan, profiles
from sim_plugins.plan import nondefault, sim_args_full

REPO = Path(__file__).resolve().parents[5]
SIM_LAUNCH = REPO / 'ros2_ws' / 'src' / 'simulation' / 'sim_bringup' / 'launch' / 'sim.launch.py'
DEMO_SH = REPO / 'simulation' / 'scripts' / 'demo.sh'


def resolve(name, *sets):
    return profiles.resolve(name, sets=list(sets), profiles_dir=REPO / 'simulation' / 'profiles')


def launch_file_defaults():
    """The (name -> default) table of sim.launch.py's ARGUMENTS, read from the file (no ROS import)."""
    from sim_bringup import sim_paths
    tree = ast.parse(SIM_LAUNCH.read_text(encoding='utf-8'))
    for node in tree.body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'ARGUMENTS':
            out = {}
            for elt in node.value.elts:
                name = ast.literal_eval(elt.elts[0])
                default = eval(compile(ast.Expression(elt.elts[1]), 'sim.launch.py', 'eval'),  # noqa: S307
                               {'sp': sim_paths, 'str': str})
                out[name] = default
            return out
    raise AssertionError('ARGUMENTS not found in sim.launch.py')


def demo_sh_defaults():
    """The option defaults demo.sh sets before reading its options (WORLD="runway"; DETECTOR="real"; ...)."""
    text = DEMO_SH.read_text(encoding='utf-8')
    block = text[text.index('# ---- options'):text.index('FIRST=1')]
    return dict(re.findall(r'\b([A-Z_]+)="?([^";()\s]*)"?(?=;|\s|$)', block))


# ---- defaults = today ---------------------------------------------------------------------------------

def test_plan_defaults_equal_the_launch_file():
    launch = launch_file_defaults()
    for name, default in plan.SIM_LAUNCH_DEFAULTS.items():
        assert name in launch, f'{name} is not an argument of sim.launch.py'
        assert launch[name] == default, f'{name}: plan says {default!r}, sim.launch.py says {launch[name]!r}'


def test_default_profile_adds_nothing_to_the_launch(sim_home):
    for rp in (profiles.resolve(None), resolve('default')):
        # camera_max_zoom:=1 is what the scripts ALWAYS pass for zoom "survey" (demo.sh and start_sim.sh)
        assert nondefault(sim_args_full(rp)) == {'camera_max_zoom': '1'}
        assert plan.team_launch_args(rp, '/x.yaml') == []
        assert plan.stack_check_args(rp) == []
        assert plan.need_detector(rp) is True
        assert not plan.needs_resolved_file(rp)


def test_default_demo_vars_equal_demo_sh_defaults(sim_home):
    sh = demo_sh_defaults()
    sv = plan.demo_vars(resolve('default'))
    for name in ('WORLD', 'DETECTOR', 'CAMERA', 'ZOOM', 'HEADLESS', 'TIMEOUT_MIN', 'INSTANCE', 'GCS_PORT',
                 'MAVROS_PORT', 'PANEL_PORT', 'WINDOWS_GCS', 'WINDOWS_GCS_ADDR', 'MISSION_OVERRIDE'):
        assert sv.scalars[name] == sh[name], name
    assert sv.scalars['MODEL_OVERRIDE'] == '' and sv.scalars['FCU_URL_OVERRIDE'] == ''
    assert sv.scalars['TEAM_STACK_WHERE'] == 'local' and sv.scalars['NEED_DETECTOR'] == '1'
    assert all(v == [] for v in sv.arrays.values()), sv.arrays


def test_default_start_sim_vars_add_nothing(sim_home):
    sv = plan.start_sim_vars(resolve('default'))
    assert sv.scalars['WORLD'] == 'runway' and sv.arrays['PLUGIN_SIM_ARGS'] == []
    # = simulation/README.md "OR BY HAND" and start_sim.sh --dry-run without a profile (byte for byte)
    assert sv.scalars['PLUGIN_TEAM_HINT'] == \
        'TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py" ./launcher.sh sim'
    start_sim = (REPO / 'simulation' / 'scripts' / 'start_sim.sh').read_text(encoding='utf-8')
    assert '&& ' + sv.scalars['PLUGIN_TEAM_HINT'].replace('"', '\\"') + '"' in start_sim


# ---- each example profile ------------------------------------------------------------------------------

def test_truth_detector_plan(sim_home):
    rp = resolve('truth_detector')
    sv = plan.demo_vars(rp)
    assert sv.scalars['DETECTOR'] == 'truth' and sv.scalars['NEED_DETECTOR'] == '0'
    assert sv.arrays['PLUGIN_SIM_ARGS'] == [] and sv.arrays['PLUGIN_TEAM_ARGS'] == []
    assert nondefault(sim_args_full(rp)) == {'detector': 'truth', 'camera_max_zoom': '1'}


def test_example_detector_plan(sim_home):
    rp = resolve('example_detector')
    path = str(plan.resolved_file_path(rp))
    sv = plan.demo_vars(rp, path)
    assert sv.scalars['DETECTOR'] == 'custom' and sv.scalars['NEED_DETECTOR'] == '1'
    assert sv.arrays['PLUGIN_TEAM_ARGS'] == [f'profile:={path}']
    assert sv.arrays['PLUGIN_STACK_ARGS'] == ['--skip-node', 'new_od', '--add-node', 'example_detector']
    assert sv.arrays['PLUGIN_SIM_ARGS'] == []
    assert plan.needs_resolved_file(rp) and plan.team_side_entries(rp)[0][0] == 'detector'
    assert 'custom: example_detector' in sv.scalars['PLUGIN_DETECTOR_TEXT']


def test_example_listener_plan(sim_home):
    rp = resolve('example_listener')
    sv = plan.demo_vars(rp, '/gen/p.yaml')
    assert sv.arrays['PLUGIN_TEAM_ARGS'] == ['profile:=/gen/p.yaml'] and sv.scalars['DETECTOR'] == 'real'
    assert sv.arrays['PLUGIN_STACK_ARGS'] == []


def test_ardupilot_version_plan(sim_home):
    sv = plan.demo_vars(resolve('ardupilot_version'))
    assert sv.arrays['PLUGIN_SIM_ARGS'] == ['sitl_version:=4.7.1',
                                            f'sitl_run_dir:={sim_home["home"] / "sitl_I0_v4.7.1"}']


def test_ardupilot_source_plan(sim_home):
    args = dict(a.split(':=', 1) for a in plan.demo_vars(resolve('ardupilot_source')).arrays['PLUGIN_SIM_ARGS'])
    assert args['sitl_source'] == '/opt/ardupilot'
    assert Path(args['sitl_run_dir']).name.startswith('sitl_I0_src-ardupilot-')


def test_external_autopilot_plan(sim_home):
    args = dict(a.split(':=', 1) for a in plan.demo_vars(resolve('external_autopilot')).arrays['PLUGIN_SIM_ARGS'])
    assert args == {'autopilot': 'external', 'mavlink_master': 'tcp:192.168.1.50:5760', 'fdm_addr': '0.0.0.0',
                    'sitl_run_dir': str(sim_home['home'] / 'sitl_I0_external')}


def test_real_drone_params_plan(sim_home):
    args = dict(a.split(':=', 1) for a in plan.demo_vars(resolve('real_drone_params')).arrays['PLUGIN_SIM_ARGS'])
    assert Path(args['sitl_param_files']).name == 'example_real_drone.param'
    assert Path(args['sitl_run_dir']).name.startswith('sitl_I0_params-')


def test_docker_split_plan(sim_home):
    sv = plan.demo_vars(resolve('docker_split'))
    assert sv.scalars['TEAM_STACK_WHERE'] == 'external' and sv.scalars['HEADLESS'] == '1'
    assert sv.scalars['WINDOWS_GCS'] == 'false'
    assert sorted(sv.arrays['PLUGIN_SIM_ARGS']) == ['camera_bind:=0.0.0.0', 'mavros_out:=udp:172.30.0.3:14552']


def test_mission_brain_custom_plan(sim_home, tmp_path):
    p = tmp_path / 'brain.yaml'
    p.write_text('mission_brain:\n  kind: custom\n  custom: {package: my_brain, executable: brain, node_name: brain}\n')
    rp = profiles.resolve(str(p))
    assert plan.team_launch_args(rp, '/gen/x.yaml') == ['mission_brain:=custom', 'profile:=/gen/x.yaml']
    assert plan.stack_check_args(rp) == ['--skip-node', 'main_controller', '--add-node', 'brain']


def test_custom_camera_is_started_by_the_simulator(sim_home, tmp_path):
    p = tmp_path / 'cam.yaml'
    p.write_text('camera:\n  kind: custom\n  custom: {command: [python3, /opt/cam.py]}\n')
    rp = profiles.resolve(str(p))
    full = nondefault(sim_args_full(rp))
    assert full['emulator'] == 'false'
    assert plan.sim_side_entries(rp)[0][0] == 'camera'
    sv = plan.demo_vars(rp, '/gen/c.yaml')
    assert 'profile:=/gen/c.yaml' in sv.arrays['PLUGIN_SIM_ARGS'] and 'emulator:=false' in sv.arrays['PLUGIN_SIM_ARGS']


def test_start_sim_typed_arguments_win(sim_home):
    rp = resolve('default', 'camera.model=a8mini', 'camera.zoom=team', 'autopilot.version=4.7.1')
    sv = plan.start_sim_vars(rp, typed=['camera_model'])
    args = sv.arrays['PLUGIN_SIM_ARGS']
    assert not any(a.startswith('camera_model:=') for a in args)     # the user typed it: never overridden
    assert 'camera_max_zoom:=' in args                               # "team" = the camera's own default
    assert 'sitl_version:=4.7.1' in args
    sv = plan.start_sim_vars(resolve('default', 'camera.zoom=2.5'))
    assert 'camera_max_zoom:=2.5' in sv.arrays['PLUGIN_SIM_ARGS']


def test_team_stack_hint_with_other_mavros_port(sim_home):
    hint = plan.team_stack_hint(resolve('default', 'network.mavros_port=14562'))
    assert hint.startswith('FCU_URL=udp://:14562@localhost:14562 TRACKER_CMD=')


def test_team_stack_hint_adds_only_what_differs(sim_home):
    assert plan.team_stack_hint(resolve('truth_detector')) == \
        'TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py detector:=truth" ./launcher.sh sim'
    hint = plan.team_stack_hint(resolve('example_detector'), '/gen/p.yaml')
    assert 'detector:=custom profile:=/gen/p.yaml' in hint


def test_summary_mentions_every_slot(sim_home):
    text = plan.render_summary(resolve('example_detector'))
    for slot in ('world', 'autopilot', 'camera', 'detector', 'mission_brain', 'extras', 'network', 'team stack'):
        assert re.search(rf'^\s+{slot}\s', text, re.M), slot


def test_resolved_file_is_stable_and_written(sim_home):
    rp = resolve('example_detector')
    a, b = plan.resolved_file_path(rp), plan.resolved_file_path(resolve('example_detector'))
    assert a == b and a.parent == sim_home['home'] / 'generated'
    written = plan.write_resolved_file(rp)
    assert written == a and written.is_file()
    assert profiles.resolve(str(written)).get('detector.kind') == 'custom'


# ---- environment checks with fakes ---------------------------------------------------------------------

def test_check_environment_finds_what_is_missing(sim_home, fake_arducopter):
    prog = fake_arducopter(sim_home['sitl'] / 'arducopter', '4.6.3')
    env = plan.check_environment(resolve('default'), binary_finder=lambda: str(prog))
    assert env['errors'] == [] and env['autopilot_plan'].binary == str(prog)
    env = plan.check_environment(resolve('ardupilot_version'), binary_finder=lambda: str(prog))
    assert any('fetch_sitl.sh 4.7.1' in e for e in env['errors'])
    env = plan.check_environment(resolve('example_detector'), share_finder=lambda pkg: None,
                                 binary_finder=lambda: str(prog))
    assert any('example_detector' in e and 'not built' in e for e in env['errors'])
    src = REPO / 'ros2_ws' / 'src' / 'simulation' / 'sim_plugin_examples' / 'example_detector'
    env = plan.check_environment(resolve('example_detector'), share_finder=lambda pkg: str(src),
                                 binary_finder=lambda: str(prog))
    assert env['errors'] == []


def test_check_environment_executable_and_command_entries(sim_home, fake_arducopter, tmp_path):
    prog = fake_arducopter(tmp_path / 'arducopter', '4.6.3')
    p = tmp_path / 'x.yaml'
    p.write_text('extras:\n  - {package: lg, executable: logger}\n  - {command: [definitely_not_a_program_xyz]}\n'
                 f'world: {{mission: {tmp_path / "missing.waypoints"}}}\n')
    rp = profiles.resolve(str(p))
    prefix = tmp_path / 'prefix'
    env = plan.check_environment(rp, prefix_finder=lambda pkg: str(prefix), binary_finder=lambda: str(prog))
    text = ' | '.join(env['errors'])
    assert 'no executable "logger"' in text and 'definitely_not_a_program_xyz' in text and 'mission file' in text


def test_zoom_to_max_zoom():
    assert plan.zoom_to_max_zoom('survey') == '1'
    assert plan.zoom_to_max_zoom('team') == ''
    assert plan.zoom_to_max_zoom(2.5) == '2.5'


def test_shell_vars_render_is_quoted():
    sv = plan.ShellVars()
    sv.scalars['A'] = "it's a test; rm -rf /"
    sv.arrays['B'] = ['x y', '$HOME']
    text = sv.render()
    assert "A='it'\"'\"'s a test; rm -rf /'" in text and "B=('x y' '$HOME')" in text


@pytest.mark.parametrize('name', ['default', 'truth_detector', 'example_detector', 'example_listener',
                                  'ardupilot_version', 'ardupilot_source', 'external_autopilot',
                                  'real_drone_params', 'docker_split'])
def test_every_profile_renders_a_plan(sim_home, name):
    rp = resolve(name)
    assert plan.render_summary(rp).startswith(f'PROFILE {name}')
    plan.demo_vars(rp, '/gen/p.yaml').render()
    plan.start_sim_vars(rp, (), '/gen/p.yaml').render()
