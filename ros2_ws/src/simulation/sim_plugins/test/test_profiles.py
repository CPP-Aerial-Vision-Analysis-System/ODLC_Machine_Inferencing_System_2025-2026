"""
sim_plugins/test/test_profiles.py -- PURPOSE: unit tests of profile reading, merging and checking (profiles.py,
schema.py): default.yaml is exactly the built-in defaults (= today's simulator), every example profile loads,
`extends` and `--set` merge in the documented order, relative paths start at the profile's folder, and every
mistake gives a plain-words error with a hint ("did you mean ...?").
"""

from pathlib import Path

import pytest

from sim_plugins import profiles, schema
from sim_plugins.profiles import ProfileError


def write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')
    return path


def errors_of(exc: ProfileError):
    return [p for p in exc.problems if p.severity == 'error']


# ---- the defaults ------------------------------------------------------------------------------------

def test_default_yaml_is_exactly_the_builtin_defaults(sim_home, profiles_dir):
    rp = profiles.resolve('default', profiles_dir=profiles_dir)
    assert profiles.without_meta(rp.data) == profiles.without_meta(schema.defaults())
    assert rp.is_default()
    assert rp.warnings == []


def test_no_profile_is_the_builtin_defaults(sim_home):
    rp = profiles.resolve(None)
    assert rp.is_default() and rp.name == '(built-in defaults)' and rp.sources == []


def test_defaults_are_todays_values():
    d = schema.defaults()
    assert d['world']['name'] == 'runway'
    assert d['autopilot']['kind'] == 'sitl_binary' and d['autopilot']['version'] == ''
    assert d['camera'] == {'kind': 'siyi_emulator', 'model': 'zr10', 'zoom': 'survey', 'strict': False, 'custom': None}
    assert d['detector']['kind'] == 'team' and d['mission_brain']['kind'] == 'team'
    assert (d['network']['gcs_port'], d['network']['mavros_port'], d['network']['panel_port']) == (14550, 14552, 14553)
    assert d['network']['camera_ip'] == '192.168.144.25'
    assert d['run'] == {'headless': False, 'timeout_min': 30, 'team_stack': 'local'}


@pytest.mark.parametrize('name', ['default', 'truth_detector', 'example_detector', 'example_listener',
                                  'ardupilot_version', 'ardupilot_source', 'external_autopilot',
                                  'real_drone_params', 'docker_split'])
def test_every_example_profile_loads(sim_home, profiles_dir, name):
    rp = profiles.resolve(name, profiles_dir=profiles_dir)
    assert rp.name == name
    assert rp.get('description')


def test_example_profiles_say_what_they_change(sim_home, profiles_dir):
    get = lambda n, k: profiles.resolve(n, profiles_dir=profiles_dir).get(k)   # noqa: E731
    assert get('truth_detector', 'detector.kind') == 'truth'
    assert get('example_detector', 'detector.kind') == 'custom'
    assert get('example_detector', 'detector.custom')['node_name'] == 'example_detector'
    assert get('ardupilot_version', 'autopilot.version') == '4.7.1'
    assert get('ardupilot_source', 'autopilot.source_dir') == '/opt/ardupilot'
    assert get('external_autopilot', 'autopilot.external.mavlink') == 'tcp:192.168.1.50:5760'
    assert get('example_listener', 'extras')[0]['executable'] == 'example_listener'
    params = get('real_drone_params', 'autopilot.param_files')
    assert len(params) == 1 and Path(params[0]).is_file()       # relative to the profile's folder


def test_list_profiles(profiles_dir):
    rows = profiles.list_profiles(profiles_dir)
    names = [r[0] for r in rows]
    assert 'default' in names and 'example_detector' in names
    assert all(problem == '' for _, _, problem in rows)


# ---- merging ------------------------------------------------------------------------------------------

def test_extends_then_file_then_sets(sim_home, tmp_path):
    write(tmp_path / 'base.yaml', 'profile: 1\nname: base\ncamera: {model: a8mini, strict: true}\n'
                                  'detector: {kind: truth}\n')
    write(tmp_path / 'child.yaml', 'profile: 1\nextends: base.yaml\ncamera: {strict: false}\n')
    rp = profiles.resolve(str(tmp_path / 'child.yaml'), sets=['detector.kind=team', 'run.timeout_min=12'])
    assert rp.get('camera.model') == 'a8mini'          # from the base
    assert rp.get('camera.strict') is False            # the child changed it
    assert rp.get('detector.kind') == 'team'           # the command line wins
    assert rp.get('run.timeout_min') == 12             # text "12" became a number
    assert rp.name == 'child' and len(rp.sources) == 2


def test_extends_by_name_from_the_profiles_folder(sim_home, tmp_path, profiles_dir):
    write(tmp_path / 'mine.yaml', 'profile: 1\nextends: truth_detector\ncamera: {model: a8mini}\n')
    rp = profiles.resolve(str(tmp_path / 'mine.yaml'), profiles_dir=profiles_dir)
    assert rp.get('detector.kind') == 'truth' and rp.get('camera.model') == 'a8mini'


def test_extends_circle_is_reported(sim_home, tmp_path):
    write(tmp_path / 'a.yaml', 'extends: b.yaml\n')
    write(tmp_path / 'b.yaml', 'extends: a.yaml\n')
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(str(tmp_path / 'a.yaml'))
    assert 'circle' in str(exc.value)


def test_lists_are_replaced_not_appended(sim_home, tmp_path):
    write(tmp_path / 'base.yaml', 'network: {extra_outs: [udp:10.0.0.1:14550]}\n')
    write(tmp_path / 'child.yaml', 'extends: base.yaml\nnetwork: {extra_outs: [udp:10.0.0.2:14550]}\n')
    assert profiles.resolve(str(tmp_path / 'child.yaml')).get('network.extra_outs') == ['udp:10.0.0.2:14550']


def test_world_shorthand_and_paths(sim_home, tmp_path):
    write(tmp_path / 'w.yaml', 'world: suas_field\n')
    assert profiles.resolve(str(tmp_path / 'w.yaml')).get('world.name') == 'suas_field'
    write(tmp_path / 'sub' / 'p.yaml', 'world: {name: ../worlds/my.sdf, mission: m.waypoints}\n')
    rp = profiles.resolve(str(tmp_path / 'sub' / 'p.yaml'))
    assert Path(rp.get('world.name')) == (tmp_path / 'worlds' / 'my.sdf')
    assert Path(rp.get('world.mission')) == (tmp_path / 'sub' / 'm.waypoints')


def test_set_values_stay_text_unless_list(sim_home):
    rp = profiles.resolve(None, sets=['network.windows_gcs_addr=10.0.0.5:14550', 'camera.zoom=2.5',
                                      'network.extra_outs=udp:1.2.3.4:14550,udp:1.2.3.5:14550',
                                      'camera.strict=yes', 'world.name=runway'])
    assert rp.get('network.windows_gcs_addr') == '10.0.0.5:14550'      # YAML 1.1 would make it a number
    assert rp.get('camera.zoom') == 2.5
    assert rp.get('network.extra_outs') == ['udp:1.2.3.4:14550', 'udp:1.2.3.5:14550']
    assert rp.get('camera.strict') is True


def test_set_inside_a_custom_entry_is_yaml(sim_home, profiles_dir):
    rp = profiles.resolve('example_detector', profiles_dir=profiles_dir,
                          sets=['detector.custom.args.min_blob_area_px=400', 'detector.custom.args.debug=true'])
    args = rp.get('detector.custom')['args']
    assert args['min_blob_area_px'] == 400 and args['debug'] is True
    assert rp.get('detector.custom')['package'] == 'example_detector'     # the rest of the entry is kept


def test_version_prefixes_are_removed(sim_home):
    assert profiles.resolve(None, sets=['autopilot.version=Copter-4.7.1']).get('autopilot.version') == '4.7.1'
    assert profiles.resolve(None, sets=['autopilot.version=V4.6.3']).get('autopilot.version') == '4.6.3'


def test_dump_resolved_round_trip(sim_home, tmp_path, profiles_dir):
    for name in ('example_detector', 'real_drone_params', 'external_autopilot'):
        rp = profiles.resolve(name, profiles_dir=profiles_dir)
        out = profiles.write_resolved(rp, tmp_path)
        again = profiles.resolve(str(out))
        assert profiles.without_meta(again.data) == profiles.without_meta(rp.data)


# ---- mistakes and their messages ---------------------------------------------------------------------

def test_unknown_key_gets_did_you_mean(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', 'detecter: {kind: truth}\ncamera: {modle: a8mini}\n')
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(str(tmp_path / 'p.yaml'))
    texts = [p.text() for p in errors_of(exc.value)]
    assert any('detecter' in t and 'did you mean "detector"' in t for t in texts)
    assert any('camera.modle' in t and 'did you mean "model"' in t for t in texts)


def test_all_errors_are_reported_at_once(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', 'camera: {model: a9}\nnetwork: {gcs_port: 99999, camera_ip: 192.168.1}\n'
                               'run: {headless: maybe}\n')
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(str(tmp_path / 'p.yaml'))
    where = {p.where for p in errors_of(exc.value)}
    assert {'camera.model', 'network.gcs_port', 'network.camera_ip', 'run.headless'} <= where
    assert '4 problem(s)' in str(exc.value)


def test_choice_aliases():
    rp = profiles.resolve(None, sets=['detector.kind=real', 'camera.model=A8-Mini', 'autopilot.kind=source',
                                      'autopilot.source_dir=/opt/ardupilot'])
    assert rp.get('detector.kind') == 'team' and rp.get('camera.model') == 'a8mini'
    assert rp.get('autopilot.kind') == 'sitl_source'


def test_custom_without_entry_is_an_error(sim_home):
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(None, sets=['detector.kind=custom'])
    assert 'detector.custom' in str(exc.value)


def test_entry_needs_exactly_one_start(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', 'detector:\n  kind: custom\n  custom: {package: x, launch: a.launch.py, '
                               'executable: b}\n')
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(str(tmp_path / 'p.yaml'))
    assert 'exactly ONE of launch, executable or command' in str(exc.value)


def test_entry_unknown_key_and_missing_package(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', 'extras:\n  - {executable: logger, nodename: x}\n')
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(str(tmp_path / 'p.yaml'))
    text = str(exc.value)
    assert 'nodename' in text and 'node_name' in text and 'needs its package' in text


def test_command_entry_relative_program(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', 'extras:\n  - {command: [./tools/log.py, --fast]}\n')
    entry = profiles.resolve(str(tmp_path / 'p.yaml')).get('extras')[0]
    assert Path(entry['command'][0]) == tmp_path / 'tools' / 'log.py'
    assert entry['name'] == 'log' and entry['start_with'] == 'team_stack'


def test_command_label_names_the_program_not_its_options():
    assert profiles.command_label(['python3', '/home/me/logger.py', '--fast']) == 'logger'
    assert profiles.command_label(['/usr/bin/python3.10', '-u', 'tools/rec.py']) == 'rec'
    assert profiles.command_label(['bash', '-c', 'echo hi']) == 'echo hi'       # (launch_support makes it echo_hi)
    assert profiles.command_label(['./tools/log.py', '--fast']) == 'log'
    assert profiles.command_label(['--only-options']) == '--only-options'


def test_autopilot_cross_checks(sim_home):
    with pytest.raises(ProfileError, match='source_dir is empty'):
        profiles.resolve(None, sets=['autopilot.kind=sitl_source'])
    with pytest.raises(ProfileError, match='MAVLink address is empty'):
        profiles.resolve(None, sets=['autopilot.kind=external'])
    with pytest.raises(ProfileError, match='version OR binary'):
        profiles.resolve(None, sets=['autopilot.version=4.7.1', 'autopilot.binary=/x/arducopter'])
    rp = profiles.resolve(None, sets=['autopilot.kind=external', 'autopilot.external.mavlink=tcp:10.0.0.9:5760',
                                      'autopilot.param_files=a.param'])
    assert any('external autopilot' in w.message for w in rp.warnings)


def test_bad_mavlink_addresses(sim_home):
    with pytest.raises(ProfileError, match='must start with one of'):
        profiles.resolve(None, sets=['network.mavros_out=172.18.0.3:14552'])
    with pytest.raises(ProfileError, match='between 1 and 65535'):
        profiles.resolve(None, sets=['network.mavros_out=udp:172.18.0.3:99999'])
    assert profiles.resolve(None, sets=['network.mavros_out=udp:astra-jetson:14552']).get('network.mavros_out') \
        == 'udp:astra-jetson:14552'


def test_ports_must_differ_and_gcs_out_warns(sim_home):
    with pytest.raises(ProfileError, match='must be different'):
        profiles.resolve(None, sets=['network.mavros_port=14550'])
    rp = profiles.resolve(None, sets=['network.gcs_out=udp:10.0.0.5:14550'])
    assert any(w.where == 'network.gcs_out' for w in rp.warnings)


def test_yaml_syntax_error_says_where(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', 'camera:\n  model: zr10\n kind: x\n')
    with pytest.raises(ProfileError) as exc:
        profiles.resolve(str(tmp_path / 'p.yaml'))
    assert 'not valid YAML' in str(exc.value) and 'line' in str(exc.value)


def test_not_a_mapping(sim_home, tmp_path):
    write(tmp_path / 'p.yaml', '- a\n- b\n')
    with pytest.raises(ProfileError, match='mapping'):
        profiles.resolve(str(tmp_path / 'p.yaml'))


def test_profile_not_found_lists_the_known_ones(sim_home, profiles_dir):
    with pytest.raises(ProfileError) as exc:
        profiles.resolve('truth_detectr', profiles_dir=profiles_dir)
    text = str(exc.value)
    assert 'did you mean "truth_detector"' in text and 'example_detector' in text


def test_bad_set_syntax():
    with pytest.raises(ProfileError, match='KEY=VALUE'):
        profiles.resolve(None, sets=['detector.kind'])


def test_custom_ignored_warning(sim_home, profiles_dir):
    rp = profiles.resolve('example_detector', profiles_dir=profiles_dir, sets=['detector.kind=team'])
    assert any('is ignored' in w.message for w in rp.warnings)


def test_team_stack_external_warns_about_team_side_plugins(sim_home, profiles_dir):
    rp = profiles.resolve('example_detector', profiles_dir=profiles_dir, sets=['run.team_stack=external'])
    assert any(w.where == 'run.team_stack' for w in rp.warnings)


def test_model_path_with_space_is_an_error(sim_home):
    with pytest.raises(ProfileError, match='contains a space'):
        profiles.resolve(None, sets=['detector.model_path=/home/me/my models/yolo.pt'])


def test_schema_lookup_and_keys():
    assert schema.lookup('detector.kind').default == 'team'
    with pytest.raises(KeyError):
        schema.lookup('detector.nope')
    keys = dict(schema.all_keys())
    assert 'autopilot.external.mavlink' in keys and 'network.mavros_out' in keys
    assert all(f.help for f in keys.values())        # every key is documented
