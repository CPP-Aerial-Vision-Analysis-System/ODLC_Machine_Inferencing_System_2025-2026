"""
sim_plugins/test/test_launch_support.py -- PURPOSE: unit tests of the launch-file helpers (launch_support.py):
a profile only fills launch arguments the user left at their defaults, the team-stack arguments a profile
decides, and (with ROS 2's launch libraries, skipped otherwise) that each kind of entry becomes the right
launch action: an include, a node with the right name and parameters, or a plain program.
"""

from pathlib import Path

import pytest

from sim_plugins import launch_support as ls
from sim_plugins import profiles

from conftest import PROFILES

EXAMPLES = Path(__file__).resolve().parents[2] / 'sim_plugin_examples'


def test_apply_profile_args_never_overrides_typed_values():
    cfg = {'world': 'suas_field', 'camera_model': 'zr10', 'detector': 'real', 'other': 'x'}
    defaults = {'world': 'runway', 'camera_model': 'zr10', 'detector': 'real'}
    changed = ls.apply_profile_args(cfg, defaults, {'world': 'runway', 'camera_model': 'a8mini',
                                                     'detector': 'real', 'unknown': 'y'})
    assert cfg == {'world': 'suas_field', 'camera_model': 'a8mini', 'detector': 'real', 'other': 'x'}
    assert changed == [('camera_model', 'zr10', 'a8mini')]


def test_team_args(sim_home):
    rp = profiles.resolve('example_detector', profiles_dir=PROFILES)
    assert ls.team_args(rp) == {'detector': 'custom', 'mission_brain': 'team'}
    rp = profiles.resolve(None, sets=['detector.model_path=/m/yolo.pt', 'detector.batched=true'])
    assert ls.team_args(rp) == {'detector': 'real', 'mission_brain': 'team', 'model_path': '/m/yolo.pt',
                                'batched': 'true'}


def test_load_profile_errors_are_profile_errors(sim_home):
    with pytest.raises(profiles.ProfileError):
        ls.load_profile('no_such_profile_anywhere')


def test_launch_text():
    assert ls._launch_text(True) == 'true' and ls._launch_text([1, 2]) == '1,2' and ls._launch_text(3.5) == '3.5'


# ---- with the ROS 2 launch libraries ------------------------------------------------------------------

def need_launch():
    """Skip a test when the ROS 2 launch libraries are not on the path (source /opt/ros/humble/setup.bash)."""
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')


def test_launch_source_by_extension():
    need_launch()
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    assert isinstance(ls.launch_source('/x/a.launch.py'), PythonLaunchDescriptionSource)
    assert type(ls.launch_source('/x/a.launch.xml')).__name__ == 'XMLLaunchDescriptionSource'
    with pytest.raises(ValueError, match='must end in'):
        ls.launch_source('/x/a.txt')


def test_entry_actions_launch_file():
    need_launch()
    from launch.actions import IncludeLaunchDescription
    entry = {'name': 'example_detector', 'package': 'example_detector', 'launch': 'example_detector.launch.py',
             'executable': '', 'command': [], 'args': {'min_blob_area_px': 150, 'debug': True}}
    actions = ls.entry_actions(entry, 'detector', share_finder=lambda pkg: str(EXAMPLES / pkg))
    include = [a for a in actions if isinstance(a, IncludeLaunchDescription)][0]
    assert include.launch_description_source.location.endswith('example_detector/launch/example_detector.launch.py')
    args = dict((k, v) for k, v in include.launch_arguments)
    assert args == {'min_blob_area_px': '150', 'debug': 'true'}


def test_entry_actions_missing_package_is_logged_not_raised():
    need_launch()
    from launch.actions import LogInfo
    entry = {'name': 'x', 'package': 'not_built', 'launch': 'x.launch.py', 'executable': '', 'command': [],
             'args': {}}
    actions = ls.entry_actions(entry, 'extra', share_finder=lambda pkg: None)
    assert len(actions) == 1 and isinstance(actions[0], LogInfo)


def test_entry_actions_executable_and_command():
    need_launch()
    from launch.actions import ExecuteProcess
    from launch_ros.actions import Node
    node_entry = {'name': 'lg', 'package': 'example_listener', 'executable': 'example_listener', 'launch': '',
                  'command': [], 'args': {'summary_period_s': 5.0}, 'node_name': 'example_listener'}
    node = [a for a in ls.entry_actions(node_entry, 'extra') if isinstance(a, Node)][0]
    assert node.node_package == 'example_listener' and node.node_executable == 'example_listener'
    cmd_entry = {'name': 'my logger!', 'package': '', 'executable': '', 'launch': '',
                 'command': ['python3', '/opt/log.py', '--fast'], 'args': {}}
    proc = [a for a in ls.entry_actions(cmd_entry, 'extra') if isinstance(a, ExecuteProcess)][0]
    assert len(proc.cmd) == 3
