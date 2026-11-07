"""
sim_bringup/test/test_gui_configs.py -- PURPOSE: check the Gazebo window layouts without starting Gazebo.
  * Every layout file in config/ is readable XML.
  * The camera-only window (gui_camera:=window) holds exactly one camera panel that listens to the drone's live
    camera topic.
  * The main layout without the camera panel is our normal layout with ONLY the camera panel taken out (so the
    two files cannot drift apart unnoticed).
  * sim_paths.layout_without_camera_panels, which the launch uses for gui_camera:=window/off when the window would
    otherwise use the WORLD's own layout (runway.sdf and suas_field_real.sdf have a <gui> section) or a gui_config file,
    keeps every other panel and setting (start view, light, follow-camera numbers) and removes only the camera.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from sim_bringup import sim_paths as sp

CONFIG = Path(__file__).resolve().parents[1] / 'config'
WORLDS = Path(__file__).resolve().parents[2] / 'sim_worlds' / 'worlds'
CAMERA_TOPIC = '/sim/camera/video/image'


def parse_layout(text):
    """A gz-gui layout has several top-level elements (window, plugins): wrap them in one root to read it."""
    body = text.split('?>', 1)[1] if text.lstrip().startswith('<?xml') else text
    return ET.fromstring(f'<layout>{body}</layout>')


def load(name):
    return parse_layout((CONFIG / name).read_text(encoding='utf-8'))


def plugins(root):
    return [(p.get('filename'), p.get('name')) for p in root.findall('plugin')]


@pytest.mark.parametrize('name', ['gazebo_gui.config', 'gazebo_gui_no_camera.config', 'gazebo_camera_window.config'])
def test_every_layout_is_readable(name):
    assert load(name).findall('plugin')


def test_the_camera_window_shows_only_the_live_camera():
    root = load('gazebo_camera_window.config')
    [(filename, _)] = plugins(root)
    assert filename == 'ImageDisplay'
    panel = root.find('plugin')
    assert panel.findtext('topic') == CAMERA_TOPIC and panel.findtext('topic_picker') == 'false'
    assert root.find('window') is not None


def test_the_main_layout_without_camera_is_the_normal_one_minus_the_camera_panel():
    normal = plugins(load('gazebo_gui.config'))
    without = plugins(load('gazebo_gui_no_camera.config'))
    assert [p for p in normal if p[0] != 'ImageDisplay'] == without
    assert any(p[0] == 'ImageDisplay' for p in normal) and not any(p[0] == 'ImageDisplay' for p in without)
    assert ('MinimalScene', '3D View') in without and ('CameraTracking', 'Camera Tracking') in without
    # and the shipped copy says the same as the generated one would
    generated, removed = sp.layout_without_camera_panels((CONFIG / 'gazebo_gui.config').read_text(encoding='utf-8'))
    assert removed == 1 and plugins(parse_layout(generated)) == without


@pytest.mark.parametrize('world', ['runway.sdf', 'suas_field_real.sdf'])
def test_a_world_layout_loses_only_its_camera_panel(world):
    # the demo worlds bring their own <gui>; gui_camera:=window must keep it and drop only the camera panel
    path = WORLDS / world
    if not path.is_file():
        pytest.skip(f'{path} not in this checkout')
    gui = ET.parse(path).getroot().find('.//gui')
    original = [(p.get('filename'), p.get('name')) for p in gui.findall('plugin')]
    text, removed = sp.layout_without_camera_panels(path.read_text(encoding='utf-8'), from_world=True)
    root = parse_layout(text)
    assert removed == 1
    assert plugins(root) == [p for p in original if p[0] != 'ImageDisplay']
    assert root.find('window') is None          # same window size as when Gazebo reads the world's <gui> itself
    # the world's own start view and follow-camera numbers survive unchanged
    for tag in ('camera_pose', 'ambient_light', 'background_color'):
        assert root.find(f"plugin[@filename='MinimalScene']/{tag}").text == gui.find(
            f"plugin[@filename='MinimalScene']/{tag}").text
    for tag in ('follow_offset', 'follow_pgain', 'follow_target'):
        assert root.find(f"plugin[@filename='CameraTracking']/{tag}").text == gui.find(
            f"plugin[@filename='CameraTracking']/{tag}").text


def test_a_custom_layout_file_loses_its_camera_panels_too():
    # gui_config:=<your file> with gui_camera:=off: no camera panel may stay behind silently
    custom = """<?xml version="1.0"?>
    <!-- a team member's own layout -->
    <window><width>800</width><height>600</height></window>
    <plugin filename="MinimalScene" name="3D View"><camera_pose>1 2 3 0 0 0</camera_pose></plugin>
    <plugin filename="ImageDisplay" name="cam A"><topic>/a</topic></plugin>
    <plugin filename="libImageDisplay.so" name="cam B"/>
    <plugin filename="WorldStats" name="World stats"/>
    """
    text, removed = sp.layout_without_camera_panels(custom)
    root = parse_layout(text)
    assert removed == 2
    assert plugins(root) == [('MinimalScene', '3D View'), ('WorldStats', 'World stats')]
    assert root.findtext('window/width') == '800'
    assert root.findtext("plugin[@filename='MinimalScene']/camera_pose") == '1 2 3 0 0 0'


def test_layouts_that_cannot_be_used_say_so():
    assert sp.layout_without_camera_panels(None) == (None, 0)
    assert sp.layout_without_camera_panels('<plugin filename="x">') == (None, 0)            # broken XML
    assert sp.layout_without_camera_panels('<sdf><world name="w"/></sdf>', from_world=True) == (None, 0)
    text, removed = sp.layout_without_camera_panels('<plugin filename="WorldStats" name="s"/>')
    assert removed == 0 and plugins(parse_layout(text)) == [('WorldStats', 's')]
