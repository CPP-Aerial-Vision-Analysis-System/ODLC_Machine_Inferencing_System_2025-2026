"""
sim_bringup/test/test_small_tools.py -- PURPOSE: unit tests for the small helpers that need no simulator: the SIYI
packet builder of photo_trigger (must produce exactly the bytes the team's video_cam sends), the health-text parser and
node-name check of stack_check, and the argument list of sim.launch.py (every argument the contract asks for exists,
plus the switch for the Windows ground station copy).
"""

import importlib.util
from pathlib import Path

import pytest

from sim_bringup.photo_trigger import siyi_packet
from sim_bringup.stack_check import TEAM_NODES, missing_nodes, parse_health


def test_take_photo_packet_matches_the_team_constant():
    # video_cam/config.py TAKE_PHOTO_COMMAND = 55 66 01 01 00 00 00 0c 00 34 ce
    assert siyi_packet(0x0C, b'\x00') == bytes.fromhex('556601010000000c0034ce')
    # SIYI SDK manual example: set gimbal angles yaw 0, pitch -90 -> 55 66 01 04 00 00 00 0E 00 00 7C FC 4F A4
    assert siyi_packet(0x0E, bytes.fromhex('00007cfc')) == bytes.fromhex('556601040000000e00007cfc4fa4')


def test_parse_health_text():
    h = parse_health('Healthy: True, ConsecErrors: 0, ModelLoaded: True, Format: pytorch')
    assert h == {'Healthy': 'True', 'ConsecErrors': '0', 'ModelLoaded': 'True', 'Format': 'pytorch'}
    assert parse_health('') == {}


def test_missing_nodes():
    running = ['/waypoint_manager', '/main_controller', '/siyi', '/mavros', '/mavros/sys']
    assert missing_nodes(running, TEAM_NODES) == ['mission_camera_trigger', 'new_od', 'gps_mavros_service_node']
    assert missing_nodes(['/' + n for n in TEAM_NODES], TEAM_NODES) == []


def load_sim_launch():
    """Import launch/sim.launch.py as a module (it is a file, not part of the Python package)."""
    pytest.importorskip('launch')
    pytest.importorskip('launch_ros')
    path = Path(__file__).resolve().parents[1] / 'launch' / 'sim.launch.py'
    spec = importlib.util.spec_from_file_location('sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_file_declares_the_contract_arguments():
    module = load_sim_launch()
    names = {name: default for name, default, _ in module.ARGUMENTS}
    expected = {'world': 'runway', 'camera_model': 'zr10', 'gui': 'true', 'rviz': 'true', 'panel': 'true',
                'headless_rendering': 'false', 'sitl': 'true', 'sitl_instance': '0', 'mavproxy': 'true',
                'emulator': 'true', 'payload': 'true', 'bridge': 'true', 'detector': 'real', 'markers': 'true'}
    for name, default in expected.items():
        assert names.get(name) == default, name
    assert module.generate_launch_description() is not None


def test_launch_file_has_the_windows_ground_station_switch():
    # on by default (Mission Planner on Windows can watch), automatic address, and a way to switch it off
    names = {name: default for name, default, _ in load_sim_launch().ARGUMENTS}
    assert names.get('windows_gcs') == 'true' and names.get('windows_gcs_addr') == ''
