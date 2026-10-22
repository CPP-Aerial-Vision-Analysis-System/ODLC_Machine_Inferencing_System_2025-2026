"""
sim_bringup/test/test_sim_paths.py -- PURPOSE: unit tests for sim_paths, the module that turns launch arguments into
files and port numbers. Runs with plain pytest (no ROS, no Gazebo): every file it needs is made in a temporary
folder (a fake sim_worlds share, a fake ardupilot_gazebo, a fake sim_models model).
"""

import math
import textwrap
from pathlib import Path

import pytest
import yaml

from sim_bringup import sim_paths as sp

WORLD_SDF = textwrap.dedent("""\
    <?xml version="1.0"?>
    <sdf version="1.9">
      <world name="astra_runway">
        <spherical_coordinates>
          <surface_model>EARTH_WGS84</surface_model>
          <latitude_deg>36.2118</latitude_deg>
          <longitude_deg>-96.0055</longitude_deg>
          <elevation>200.5</elevation>
          <heading_deg>0</heading_deg>
        </spherical_coordinates>
      </world>
    </sdf>
    """)


def make_worlds_share(tmp_path: Path, home='36.2118,-96.0055,200.5,0', with_mission=True) -> Path:
    """Create a fake sim_worlds share folder with config/runway.yaml, worlds/, missions/, truth/."""
    share = tmp_path / 'sim_worlds'
    for sub in ('config', 'worlds', 'missions', 'truth'):
        (share / sub).mkdir(parents=True)
    (share / 'worlds' / 'runway.sdf').write_text(WORLD_SDF)
    if with_mission:
        (share / 'missions' / 'runway_survey.waypoints').write_text('QGC WPL 110\n')
    (share / 'truth' / 'runway.yaml').write_text('targets: []\n')
    cfg = {'world_file': 'worlds/runway.sdf', 'home': home,
           'spawn': {'x': 1.5, 'y': -2.0, 'z': 0.3, 'yaw': 1.0},
           'mission': 'missions/runway_survey.waypoints', 'truth': 'truth/runway.yaml'}
    (share / 'config' / 'runway.yaml').write_text(yaml.safe_dump(cfg))
    return share


def test_truthy_accepts_the_usual_spellings():
    assert all(sp.truthy(v) for v in ('true', 'True', '1', 'yes', 'ON', ' true '))
    assert not any(sp.truthy(v) for v in ('false', '0', 'no', '', 'off', 'maybe'))


def test_sitl_ports_follow_the_10_per_instance_rule():
    assert sp.sitl_ports(0) == {'serial0': 5760, 'serial1': 5762, 'serial2': 5763, 'fdm': 9002, 'rc_in': 5501}
    p4 = sp.sitl_ports(4)
    assert p4['serial0'] == 5800 and p4['fdm'] == 9042
    with pytest.raises(ValueError):
        sp.sitl_ports(-1)


# /proc/net/route of WSL2 on the team laptop: default route via 172.24.128.1 (= the Windows host), little-endian hex.
ROUTE_TABLE = ('Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask\t\tMTU\tWindow\tIRTT\n'
               'eth0\t00000000\t018018AC\t0003\t0\t0\t0\t00000000\t0\t0\t0\n'
               'eth0\t000018AC\t00000000\t0001\t0\t0\t0\t00F0FFFF\t0\t0\t0\n')


def test_default_gateway_reads_the_route_table():
    assert sp.default_gateway(ROUTE_TABLE) == '172.24.128.1'
    assert sp.default_gateway('Iface\tDestination\tGateway\n') == ''
    # a second default route with a higher metric (e.g. a VPN) does not win
    two = ROUTE_TABLE + 'wlan0\t00000000\t0101A8C0\t0003\t0\t0\t600\t00000000\t0\t0\t0\n'
    assert sp.default_gateway(two) == '172.24.128.1'


def test_running_in_wsl_reads_the_kernel_name(monkeypatch):
    assert sp.running_in_wsl('Linux version 5.15.167.4-microsoft-standard-WSL2 (root@x) #1 SMP')
    assert not sp.running_in_wsl('Linux version 6.8.0-45-generic (buildd@lcy02-amd64-075) #45-Ubuntu SMP')
    monkeypatch.setattr(sp, 'running_in_wsl', lambda proc_version=None: False)
    assert sp.wsl_networking_mode() == ''


def test_parse_host_port_good_and_bad():
    assert sp.parse_host_port('192.168.1.20', 14550) == ('192.168.1.20', 14550)
    assert sp.parse_host_port(' 10.0.0.5:14600 ', 14550) == ('10.0.0.5', 14600)
    for bad in ('', ':14550', '1.2.3.4:', '1.2.3.4:x', '1.2.3.4:0', '1.2.3.4:70000', 'a b:1', '::1'):
        with pytest.raises(ValueError):
            sp.parse_host_port(bad, 14550)


def test_windows_gcs_output_automatic_choices():
    local = (14550, 14552, 14553)
    out, notes = sp.windows_gcs_output('true', '', local, mode='nat', gateway='172.24.128.1')
    assert out == 'udp:172.24.128.1:14550' and 'Mission Planner' in notes[0]
    # mirrored networking: Windows shares 127.0.0.1, where 14550 is our scripts' port -> 14551
    assert sp.windows_gcs_output('true', '', local, mode='mirrored')[0] == 'udp:127.0.0.1:14551'
    assert sp.windows_gcs_output('true', '', local, mode='')[0] == ''                    # not WSL
    assert sp.windows_gcs_output('true', '', local, mode='none')[0] == ''                # WSL without network
    assert sp.windows_gcs_output('true', '', local, mode='nat', gateway='')[0] == ''     # no default route
    out, notes = sp.windows_gcs_output('false', '', local, mode='nat', gateway='172.24.128.1')
    assert out == '' and 'windows_gcs:=false' in notes[0]


def test_windows_gcs_output_with_an_address_given():
    local = (14550, 14552, 14553)
    assert sp.windows_gcs_output('true', '192.168.1.20', local, mode='')[0] == 'udp:192.168.1.20:14550'
    assert sp.windows_gcs_output('true', '192.168.1.20:14600', local, mode='nat',
                                 gateway='172.24.128.1')[0] == 'udp:192.168.1.20:14600'
    out, notes = sp.windows_gcs_output('true', '127.0.0.1:14552', local, mode='nat', gateway='x')
    assert out == '' and 'not doubled' in notes[0]                          # would double the MAVROS output
    out, notes = sp.windows_gcs_output('true', 'nonsense:port', local, mode='nat', gateway='x')
    assert out == '' and 'windows_gcs_addr' in notes[0]


def test_parse_home_good_and_bad():
    assert sp.parse_home('36.2118,-96.0055,200.5,0') == (36.2118, -96.0055, 200.5, 0.0)
    for bad in ('1,2,3', 'a,b,c,d', '95,0,0,0', '0,190,0,0'):
        with pytest.raises(ValueError):
            sp.parse_home(bad)
    lat, lon, alt, hdg = sp.parse_home(sp.format_home(36.2118, -96.0055, 200.5, 0))
    assert (lat, lon, alt, hdg) == pytest.approx((36.2118, -96.0055, 200.5, 0.0))


def test_read_spherical_coordinates_and_world_name():
    assert sp.read_spherical_coordinates(WORLD_SDF) == pytest.approx((36.2118, -96.0055, 200.5, 0.0))
    assert sp.read_spherical_coordinates('<sdf><world name="x"/></sdf>') is None
    assert sp.world_name_from_sdf(WORLD_SDF) == 'astra_runway'
    assert sp.world_name_from_sdf('<sdf/>', default='d') == 'd'


def test_load_world_config_by_name(tmp_path):
    share = make_worlds_share(tmp_path)
    cfg = sp.load_world_config('runway', str(share))
    assert cfg.name == 'runway'
    assert cfg.world_file == str(share / 'worlds' / 'runway.sdf')
    assert cfg.home == '36.2118,-96.0055,200.5,0'
    assert cfg.spawn == {'x': 1.5, 'y': -2.0, 'z': 0.3, 'yaw': 1.0}
    assert cfg.mission.endswith('runway_survey.waypoints') and cfg.truth.endswith('runway.yaml')
    assert cfg.notes == []


def test_load_world_config_from_a_yaml_path_without_the_package(tmp_path):
    # sim_worlds not built yet: give the source folder's config YAML directly
    share = make_worlds_share(tmp_path)
    cfg = sp.load_world_config(str(share / 'config' / 'runway.yaml'), None)
    assert cfg.world_file == str(share / 'worlds' / 'runway.sdf')
    assert cfg.worlds_dir == str(share) and cfg.mission.endswith('runway_survey.waypoints')
    by_name = sp.load_world_config('runway', str(share))
    assert by_name.worlds_dir == str(share)


def test_load_world_config_takes_home_from_the_world_when_missing(tmp_path):
    share = make_worlds_share(tmp_path, home='')
    cfg = sp.load_world_config('runway', str(share))
    assert sp.parse_home(cfg.home)[:3] == pytest.approx((36.2118, -96.0055, 200.5))


def test_load_world_config_reports_missing_things(tmp_path):
    share = make_worlds_share(tmp_path, with_mission=False)
    cfg = sp.load_world_config('runway', str(share))
    assert cfg.mission == '' and any('mission' in n for n in cfg.notes)
    missing = sp.load_world_config('moon', str(share))
    assert missing.world_file == '' and any('not found' in n for n in missing.notes)
    no_pkg = sp.load_world_config('runway', None)
    assert no_pkg.world_file == '' and any('sim_worlds is not built' in n for n in no_pkg.notes)


def test_custom_sdf_world_uses_its_spherical_coordinates(tmp_path):
    sdf = tmp_path / 'my_field.sdf'
    sdf.write_text(WORLD_SDF)
    cfg = sp.load_world_config(str(sdf), None)
    assert cfg.name == 'my_field' and cfg.world_file == str(sdf.resolve())
    assert sp.parse_home(cfg.home)[:2] == pytest.approx((36.2118, -96.0055))
    gone = sp.load_world_config(str(tmp_path / 'nope.sdf'), None)
    assert gone.world_file == ''


def test_yaw_deg_is_converted_to_radians(tmp_path):
    notes = []
    assert sp._read_spawn({'yaw_deg': 90}, notes)['yaw'] == pytest.approx(math.pi / 2)
    assert sp._read_spawn('nonsense', notes) == {'x': 0.0, 'y': 0.0, 'z': 0.2, 'yaw': 0.0} and notes


def test_patch_fdm_port_rewrites_every_port():
    text = '<plugin><fdm_port_in>9002</fdm_port_in></plugin><x><fdm_port_in> 9012 </fdm_port_in></x>'
    out = sp.patch_fdm_port(text, 9042)
    assert out.count('<fdm_port_in>9042</fdm_port_in>') == 2 and '9002' not in out


def test_xacro_mappings_for_both_camera_profiles():
    zr10 = sp.xacro_mappings('zr10', 0)
    assert (zr10['photo_width'], zr10['photo_height'], zr10['fdm_port_in']) == ('2560', '1440', '9002')
    a8 = sp.xacro_mappings('a8mini', 2)
    assert (a8['photo_width'], a8['photo_height'], a8['fdm_port_in']) == ('3840', '2160', '9022')
    assert a8['video_width'] == '1280' and a8['hfov'] == '1.4137'
    with pytest.raises(ValueError):
        sp.xacro_mappings('gopro', 0)


def test_bridge_entries_real_and_standin(tmp_path):
    real = sp.bridge_entries()
    ros_topics = {e['ros_topic_name'] for e in real}
    assert {'/sim/camera/image', '/sim/camera/camera_info', '/sim/camera/photo', '/sim/drone/odometry'} <= ros_topics
    assert all(e['direction'] == 'GZ_TO_ROS' for e in real)
    stand = sp.bridge_entries(standin=True, world_name='iris_runway')
    assert stand[0]['gz_topic_name'].startswith('/world/iris_runway/model/iris/model/gimbal/')
    path = sp.write_bridge_yaml(real, tmp_path / 'b.yaml')
    assert yaml.safe_load(Path(path).read_text()) == real


def fake_ardupilot_gazebo(tmp_path: Path) -> Path:
    """Create a minimal ~/ardupilot_gazebo look-alike (world + iris_with_gimbal + plugin file)."""
    ag = tmp_path / 'ardupilot_gazebo'
    (ag / 'worlds').mkdir(parents=True)
    (ag / 'build').mkdir()
    (ag / 'build' / 'libArduPilotPlugin.so').write_text('')
    model = ag / 'models' / 'iris_with_gimbal'
    model.mkdir(parents=True)
    (model / 'model.sdf').write_text('<model><fdm_port_in>9002</fdm_port_in></model>')
    (model / 'model.config').write_text('<model/>')
    (ag / 'worlds' / 'iris_runway.sdf').write_text(
        '<sdf><world name="iris_runway"><include><uri>model://runway</uri></include>\n'
        '<include>\n  <uri>model://iris_with_gimbal</uri>\n  <pose>0 0 0.195 0 0 90</pose>\n</include>'
        '</world></sdf>')
    return ag


def test_standin_world_removes_the_builtin_drone(tmp_path):
    ag = fake_ardupilot_gazebo(tmp_path)
    cfg = sp.standin_world_config(str(ag), tmp_path / 'gen')
    text = Path(cfg.world_file).read_text()
    assert cfg.standin and 'iris_with_gimbal' not in text and 'model://runway' in text
    assert cfg.home == sp.CMAC_HOME and cfg.spawn['z'] == pytest.approx(0.195)
    assert sp.standin_world_config(None, tmp_path / 'gen').world_file == ''


def test_standin_drone_has_the_instance_port(tmp_path):
    ag = fake_ardupilot_gazebo(tmp_path)
    path, notes = sp.standin_drone_sdf(str(ag), 3, tmp_path / 'gen')
    assert '<fdm_port_in>9032</fdm_port_in>' in Path(path).read_text() and notes


def test_generate_drone_sdf_from_plain_model(tmp_path):
    share = tmp_path / 'sim_models'
    mdir = share / 'models' / 'iris_a8mini'
    (mdir / 'meshes').mkdir(parents=True)
    (mdir / 'model.sdf').write_text('<sdf><model name="iris_a8mini"><fdm_port_in>9002</fdm_port_in></model></sdf>')
    (mdir / 'model.config').write_text('<model/>')
    path, notes = sp.generate_drone_sdf(str(share), 'zr10', 1, tmp_path / 'gen')
    out = Path(path)
    assert out.name == 'model.sdf' and '<fdm_port_in>9012</fdm_port_in>' in out.read_text()
    assert (out.parent / 'meshes').is_symlink() and (out.parent / 'model.config').exists()
    # running it twice must not fail on the existing symlinks
    assert sp.generate_drone_sdf(str(share), 'zr10', 1, tmp_path / 'gen')[0] == path


def test_generate_drone_sdf_runs_xacro(tmp_path):
    pytest.importorskip('xacro')
    share = tmp_path / 'sim_models'
    mdir = share / 'models' / 'iris_a8mini'
    mdir.mkdir(parents=True)
    (mdir / 'model.sdf.xacro').write_text(textwrap.dedent("""\
        <?xml version="1.0"?>
        <sdf version="1.9" xmlns:xacro="http://www.ros.org/wiki/xacro">
          <xacro:arg name="photo_width" default="1"/>
          <xacro:arg name="photo_height" default="1"/>
          <xacro:arg name="fdm_port_in" default="9002"/>
          <model name="iris_a8mini">
            <photo w="$(arg photo_width)" h="$(arg photo_height)"/>
            <fdm_port_in>$(arg fdm_port_in)</fdm_port_in>
          </model>
        </sdf>
        """))
    path, notes = sp.generate_drone_sdf(str(share), 'a8mini', 0, tmp_path / 'gen')
    text = Path(path).read_text()
    assert 'w="3840"' in text and 'h="2160"' in text and '<fdm_port_in>9002</fdm_port_in>' in text


def test_missing_model_gives_empty_path(tmp_path):
    assert sp.generate_drone_sdf(None, 'zr10', 0, tmp_path)[0] == ''
    path, notes = sp.generate_drone_sdf(None, 'zr10', 0, tmp_path, explicit_model=str(tmp_path / 'none.sdf'))
    assert path == '' and notes


def test_sitl_defaults_order_and_notes(tmp_path, monkeypatch):
    params = tmp_path / 'params'
    params.mkdir()
    for name in ('copter.parm', 'gazebo-iris.parm'):
        (params / name).write_text('')
    astra = tmp_path / 'astra_sitl.parm'
    astra.write_text('')
    monkeypatch.setenv('ARDUPILOT_DEFAULT_PARAMS', str(params))
    files, notes = sp.sitl_defaults(str(astra))
    assert [Path(f).name for f in files] == ['copter.parm', 'gazebo-iris.parm', 'astra_sitl.parm'] and not notes
    files, notes = sp.sitl_defaults(None)
    assert len(files) == 2 and any('astra_sitl.parm' in n for n in notes)


def test_find_sitl_binary_prefers_the_environment(tmp_path, monkeypatch):
    fake = tmp_path / 'arducopter'
    fake.write_text('#!/bin/sh\n')
    fake.chmod(0o755)
    monkeypatch.setenv('ARDUPILOT_SITL_BIN', str(fake))
    assert sp.find_sitl_binary() == str(fake)
    assert sp.find_sitl_binary(str(fake)) == str(fake)


def test_state_folders_follow_astra_sim_home(tmp_path, monkeypatch):
    monkeypatch.setenv('ASTRA_SIM_HOME', str(tmp_path / 'state'))
    assert sp.sim_home() == tmp_path / 'state'
    assert sp.sitl_run_dir(4) == tmp_path / 'state' / 'sitl_I4'
    assert sp.detector_model_path() == str(tmp_path / 'state' / 'models' / 'yolo26m.pt')


def test_team_photo_dir_override_and_fallback(monkeypatch, tmp_path):
    assert sp.team_photo_dir('/x/y') == '/x/y'
    monkeypatch.setenv('ASTRA_REPO', str(tmp_path / 'repo'))
    path = sp.team_photo_dir()
    assert path.endswith('video_cam/mapping_photos')
    assert sp.team_detection_dir(path).endswith('video_cam/detection_results_sahi')


def test_scrub_foreign_qt_env_removes_wheel_plugin_paths():
    env = {'QT_QPA_PLATFORM_PLUGIN_PATH': '/usr/local/lib/python3.10/dist-packages/cv2/qt/plugins',
           'QT_QPA_FONTDIR': '/usr/local/lib/python3.10/dist-packages/cv2/qt/fonts',
           'QT_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins', 'OTHER': 'x'}
    removed = sp.scrub_foreign_qt_env(env)
    assert sorted(removed) == ['QT_QPA_FONTDIR', 'QT_QPA_PLATFORM_PLUGIN_PATH']
    assert env == {'QT_PLUGIN_PATH': '/usr/lib/x86_64-linux-gnu/qt5/plugins', 'OTHER': 'x'}


def test_team_photo_dir_does_not_import_opencv(monkeypatch):
    # importing the team's video_cam pulls in cv2, which poisons the Qt environment of every window
    import sys
    monkeypatch.delitem(sys.modules, 'video_cam.storage_manager', raising=False)
    sp.team_photo_dir()
    assert 'video_cam.storage_manager' not in sys.modules


def test_cli_answers(capsys, tmp_path, monkeypatch):
    monkeypatch.setenv('ASTRA_SIM_HOME', str(tmp_path))
    assert sp._cli(['sitl-ports', '1']) == 0
    assert 'serial0=5770' in capsys.readouterr().out
    assert sp._cli(['model-path']) == 0
    assert capsys.readouterr().out.strip().endswith('yolo26m.pt')
    assert sp._cli(['nonsense']) == 2
