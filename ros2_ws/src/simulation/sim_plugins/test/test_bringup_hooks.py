"""
sim_plugins/test/test_bringup_hooks.py -- PURPOSE: unit tests of the small plug-and-play HOOKS that live inside
sim_bringup (the scripts and launch files call them there, but they exist for the plug-and-play slots):

  * stack_check --skip-node / --add-node: without them the check waits for exactly the team's nodes, as before;
    with them a plug-in replaces a team node (a custom detector replaces new_od ...);
  * every plug-and-play argument of sim.launch.py has a default that means "today's behaviour", has help text,
    and the autopilot-slot list that switches sim_plugins on names only real arguments;
  * team_stack.launch.py declares detector / mission_brain / profile with today's defaults.
The launch files are READ as text (ast), so these tests need no ROS installation.
"""

import ast
from pathlib import Path

from sim_bringup import stack_check

LAUNCH = Path(__file__).resolve().parents[2] / 'sim_bringup' / 'launch'

#: The arguments the plug-and-play work added to sim.launch.py, with the default that keeps today's behaviour.
PLUG_ARGS = {'profile': '', 'autopilot': 'sitl', 'sitl_version': '', 'sitl_source': '', 'sitl_param_files': '',
             'sitl_param_filter': 'safe', 'sitl_param_keep': '', 'sitl_run_dir': '', 'mavlink_master': '',
             'fdm_addr': '', 'gcs_out': '', 'mavros_out': '', 'extra_outs': '', 'camera_bind': '',
             'camera_strict': ''}


def literal_assignments(path: Path):
    """{name: value} of the module-level assignments of a launch file whose value is a plain literal."""
    out = {}
    for node in ast.parse(path.read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                out[node.targets[0].id] = node.value          # not a plain literal: keep the syntax tree
    return out


def sim_launch_arguments():
    """(name, default text or None, description) of sim.launch.py's ARGUMENTS (None = computed default)."""
    tree = literal_assignments(LAUNCH / 'sim.launch.py')['ARGUMENTS']
    rows = []
    for elt in tree.elts:
        name = ast.literal_eval(elt.elts[0])
        try:
            default = ast.literal_eval(elt.elts[1])
        except ValueError:
            default = None
        try:
            text = ast.literal_eval(elt.elts[2])
        except ValueError:                                  # an f-string or a computed text
            text = ast.unparse(elt.elts[2])
        rows.append((name, default, text))
    return rows


# ---- stack_check ----------------------------------------------------------------------------------------

def test_stack_check_waits_for_exactly_the_team_nodes_by_default():
    assert stack_check.wanted_nodes([], []) == list(stack_check.TEAM_NODES)


def test_stack_check_skip_and_add_node():
    out = stack_check.wanted_nodes(['new_od'], ['example_detector'])
    assert 'new_od' not in out and out[-1] == 'example_detector'
    assert len(out) == len(stack_check.TEAM_NODES)
    assert stack_check.wanted_nodes([], ['main_controller']) == list(stack_check.TEAM_NODES)   # no duplicate
    assert 'main_controller' not in stack_check.wanted_nodes(['main_controller'], [])


# ---- sim.launch.py --------------------------------------------------------------------------------------

def test_sim_launch_plug_and_play_arguments_default_to_today():
    rows = {name: (default, desc) for name, default, desc in sim_launch_arguments()}
    for name, default in PLUG_ARGS.items():
        assert name in rows, f'sim.launch.py has no argument {name}'
        assert rows[name][0] == default, f'{name}: default {rows[name][0]!r}, today = {default!r}'
        assert len(rows[name][1]) > 20, f'{name}: needs a description (ros2 launch ... --show-args)'


def test_sim_launch_autopilot_switch_names_real_arguments():
    names = {name for name, _, _ in sim_launch_arguments()}
    switch = literal_assignments(LAUNCH / 'sim.launch.py')['PLUGIN_AUTOPILOT_ARGS']
    assert set(switch) <= names and 'autopilot' in switch and 'mavlink_master' in switch


def test_sim_launch_imports_sim_plugins_only_when_asked():
    # The default start must not need sim_plugins: every import of it sits inside an `if` of _launch_setup.
    tree = ast.parse((LAUNCH / 'sim.launch.py').read_text(encoding='utf-8'))
    top_level = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    assert not any('sim_plugins' in ast.dump(n) for n in top_level)


# ---- team_stack.launch.py -------------------------------------------------------------------------------

def test_team_stack_declares_the_slots_with_todays_defaults():
    text = (LAUNCH / 'team_stack.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('detector', default_value='real'" in text
    assert "DeclareLaunchArgument('mission_brain', default_value='team'" in text
    assert "DeclareLaunchArgument('profile', default_value=''" in text
    tree = ast.parse(text)
    top_level = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    assert not any('sim_plugins' in ast.dump(n) for n in top_level)
