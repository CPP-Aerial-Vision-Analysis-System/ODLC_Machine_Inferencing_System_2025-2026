"""
sim_plugins/test/test_autopilot.py -- PURPOSE: unit tests of the autopilot slot (autopilot.py): profile and launch
settings, which arguments a profile produces, the run-folder rules (the default and the old sitl_bin:= keep
today's folder), finding a fetched version / a source tree / a given binary (with clear errors when missing),
parameter files, reading the version inside a program, and the external-autopilot pieces (plugin fdm_addr,
MAVProxy --retries, the command for the other computer).
"""

import os
from pathlib import Path

import pytest

from sim_plugins import autopilot as ap
from sim_plugins import profiles

EXAMPLE_PARAM = Path(__file__).resolve().parents[5] / 'simulation' / 'profiles' / 'params' / 'example_real_drone.param'


def settings(*sets):
    return ap.settings_from_profile(profiles.resolve(None, sets=list(sets)).data)


def test_default_settings_are_default(sim_home):
    s = settings()
    assert s.is_default() and not s.needs_plan()
    assert ap.to_launch_args(s) == {}
    assert ap.flavour_run_dir(s) == sim_home['home'] / 'sitl_I0'


def test_launch_settings_defaults_are_default():
    s = ap.settings_from_launch({k: v for k, v in ap.LAUNCH_DEFAULTS.items()})
    assert s.is_default()
    s = ap.settings_from_launch({'sitl_bin': '/x/arducopter', 'sitl_wipe': 'true', 'sitl_instance': '2'})
    assert not s.needs_plan()            # sim.launch.py's unchanged code path handles these
    assert s.binary == '/x/arducopter' and s.wipe and s.instance == 2


def test_legacy_sitl_bin_keeps_todays_run_folder(sim_home):
    s = ap.settings_from_launch({'sitl_bin': '/x/arducopter'})
    assert ap.flavour_run_dir(s) == sim_home['home'] / 'sitl_I0'


def test_profile_binary_gets_its_own_run_folder(sim_home):
    s = settings('autopilot.binary=/x/arducopter')
    args = ap.to_launch_args(s)
    assert args['sitl_bin'] == '/x/arducopter'
    assert Path(args['sitl_run_dir']).name.startswith('sitl_I0_bin-')


def test_version_settings_and_folder(sim_home):
    s = settings('autopilot.version=4.7.1', 'autopilot.instance=1')
    assert s.needs_plan()
    args = ap.to_launch_args(s)
    assert args['sitl_version'] == '4.7.1'
    assert Path(args['sitl_run_dir']) == sim_home['home'] / 'sitl_I1_v4.7.1'


def test_source_and_params_tags(sim_home):
    s = settings('autopilot.kind=sitl_source', 'autopilot.source_dir=/opt/ardupilot',
                 f'autopilot.param_files={EXAMPLE_PARAM}')
    tag = ap.flavour_tag(s)
    assert tag.startswith('src-ardupilot-') and '_params-' in tag


def test_external_settings(sim_home):
    s = settings('autopilot.kind=external', 'autopilot.external.mavlink=tcp:10.0.0.9:5760')
    assert s.mode == 'external' and s.fdm_addr == '0.0.0.0'
    args = ap.to_launch_args(s)
    assert args['autopilot'] == 'external' and args['mavlink_master'] == 'tcp:10.0.0.9:5760'
    assert args['fdm_addr'] == '0.0.0.0' and Path(args['sitl_run_dir']).name == 'sitl_I0_external'
    plan = ap.plan_autopilot(s, write_params=False)
    assert plan.errors == [] and plan.mode == 'external'
    assert 'listens on 0.0.0.0:9002' in ap.describe(plan, s)


def test_plan_default_uses_the_finder(sim_home, fake_arducopter):
    prog = fake_arducopter(sim_home['sitl'] / 'arducopter', '4.6.3', '3fc7011a')
    plan = ap.plan_autopilot(settings(), binary_finder=lambda: str(prog))
    assert plan.binary == str(prog) and plan.version == '4.6.3 (3fc7011a)' and plan.errors == []
    plan = ap.plan_autopilot(settings(), binary_finder=lambda: None)
    assert '02_setup_ardupilot.sh' in plan.errors[0]


def test_plan_version(sim_home, fake_arducopter):
    s = settings('autopilot.version=4.7.1')
    plan = ap.plan_autopilot(s, write_params=False)
    assert 'fetch_sitl.sh 4.7.1' in plan.errors[0]             # not downloaded yet: says how to get it
    vdir = sim_home['versions'] / '4.7.1'
    fake_arducopter(vdir / 'arducopter', '4.7.1')
    (vdir / 'copter.parm').write_text('X 1\n')
    (vdir / 'gazebo-iris.parm').write_text('Y 2\n')
    plan = ap.plan_autopilot(s, write_params=False)
    assert plan.errors == [] and plan.binary == str(vdir / 'arducopter') and plan.version.startswith('4.7.1')
    files = plan.defaults_list(None, finder=lambda n: None)
    assert files[:2] == [str(vdir / 'copter.parm'), str(vdir / 'gazebo-iris.parm')]


def test_plan_version_falls_back_to_default_param_files(sim_home, fake_arducopter, tmp_path):
    vdir = sim_home['versions'] / 'latest'
    fake_arducopter(vdir / 'arducopter', '4.8.0-dev')
    plan = ap.plan_autopilot(settings('autopilot.version=latest'), write_params=False)
    astra = tmp_path / 'astra_sitl.parm'
    astra.write_text('A 1\n')
    files = plan.defaults_list(str(astra), finder=lambda n: f'/fallback/{n}')
    assert files == ['/fallback/copter.parm', '/fallback/gazebo-iris.parm', str(astra)]
    assert any('not in' in n for n in plan.notes)


def test_plan_source_tree(sim_home, fake_arducopter, tmp_path):
    tree = tmp_path / 'ardupilot'
    s = settings('autopilot.kind=sitl_source', f'autopilot.source_dir={tree}')
    assert 'not found' in ap.plan_autopilot(s).errors[0]
    tree.mkdir()
    assert './waf configure --board sitl' in ap.plan_autopilot(s).errors[0]
    fake_arducopter(tree / 'build' / 'sitl' / 'bin' / 'arducopter', '4.6.3')
    plan = ap.plan_autopilot(s)
    assert plan.errors == [] and plan.params_dir == str(tree / 'Tools' / 'autotest' / 'default_params')


def test_plan_binary(sim_home, fake_arducopter, tmp_path):
    prog = tmp_path / 'my_arducopter'
    s = settings(f'autopilot.binary={prog}')
    assert 'not found or not executable' in ap.plan_autopilot(s).errors[0]
    fake_arducopter(prog, '4.6.2')
    assert ap.plan_autopilot(s).binary == str(prog)


def test_plan_param_files(sim_home, fake_arducopter, tmp_path):
    prog = fake_arducopter(tmp_path / 'arducopter', '4.6.3')
    s = settings(f'autopilot.param_files={EXAMPLE_PARAM}')
    out = tmp_path / 'params_out'
    plan = ap.plan_autopilot(s, params_out_dir=out, binary_finder=lambda: str(prog))
    assert plan.errors == [] and len(plan.param_reports) == 1
    report = plan.param_reports[0]
    assert Path(report.output).parent == out and Path(report.output).is_file()
    files = plan.defaults_list(None, finder=lambda n: f'/sitl/{n}')
    assert files[-1] == report.output                          # the real drone's file comes LAST (it wins)
    # a dry plan reads but does not write
    dry = ap.plan_autopilot(s, params_out_dir=tmp_path / 'dry', write_params=False, binary_finder=lambda: str(prog))
    assert dry.param_reports[0].output == '' and not (tmp_path / 'dry').exists()
    missing = settings(f'autopilot.param_files={tmp_path / "nope.param"}')
    assert 'parameter file not found' in ap.plan_autopilot(missing, binary_finder=lambda: str(prog)).errors[0]


def test_read_binary_version_and_elf(tmp_path, fake_arducopter):
    prog = fake_arducopter(tmp_path / 'a', '4.6.3', '3fc7011a')
    assert ap.read_binary_version(str(prog)) == '4.6.3 (3fc7011a)'
    assert ap.is_elf(str(prog))
    other = tmp_path / 'b'
    other.write_text('#!/bin/sh\n')
    assert ap.read_binary_version(str(other)) == '' and not ap.is_elf(str(other))
    assert ap.read_binary_version(str(tmp_path / 'missing')) == ''


SDF = '''<model name="iris">
  <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
    <fdm_addr>127.0.0.1</fdm_addr>
    <fdm_port_in>9002</fdm_port_in>
  </plugin>
  <plugin name="other" filename="gz-sim-imu-system"/>
</model>'''


def test_patch_plugin_fdm_addr():
    text, found = ap.patch_plugin_fdm_addr(SDF, '0.0.0.0')
    assert found and '<fdm_addr>0.0.0.0</fdm_addr>' in text and '127.0.0.1' not in text
    no_addr = SDF.replace('    <fdm_addr>127.0.0.1</fdm_addr>\n', '')
    text, found = ap.patch_plugin_fdm_addr(no_addr, '0.0.0.0')
    assert found and text.count('<fdm_addr>0.0.0.0</fdm_addr>') == 1
    text, found = ap.patch_plugin_fdm_addr('<model/>', '0.0.0.0')
    assert not found and text == '<model/>'
    assert ap.patch_plugin_fdm_addr(SDF, '') == (SDF, True)


def test_patch_the_real_drone_model():
    xacro = Path(__file__).resolve().parents[2] / 'sim_models' / 'models' / 'iris_a8mini' / 'model.sdf.xacro'
    if not xacro.is_file():
        pytest.skip('sim_models model not present')
    text, found = ap.patch_plugin_fdm_addr(xacro.read_text(encoding='utf-8'), '0.0.0.0')
    assert found and '<fdm_addr>0.0.0.0</fdm_addr>' in text


def test_mavproxy_master_args_and_remote_command():
    assert ap.mavproxy_master_args('tcp:127.0.0.1:5760', external=False) == ['--master', 'tcp:127.0.0.1:5760']
    assert ap.mavproxy_master_args('tcp:10.0.0.9:5760', external=True) == \
        ['--master', 'tcp:10.0.0.9:5760', '--retries', str(ap.EXTERNAL_TCP_RETRIES)]
    assert ap.mavproxy_master_args('udpin:0.0.0.0:14560', external=True) == ['--master', 'udpin:0.0.0.0:14560']
    cmd = ap.remote_command(ap.AutopilotSettings(mode='external', instance=1), '36.2,-96.0,210,0', '10.0.0.2')
    assert '--sim-address=10.0.0.2' in cmd and '-I1' in cmd and '--home 36.2,-96.0,210,0' in cmd


@pytest.mark.skipif(os.name != 'posix', reason='executable bits are a POSIX thing')
def test_non_executable_binary_is_refused(sim_home, tmp_path):
    prog = tmp_path / 'arducopter'
    prog.write_bytes(b'\x7fELF')
    prog.chmod(0o644)
    assert ap.plan_autopilot(settings(f'autopilot.binary={prog}')).errors
