"""
lattice_bridge/test/test_bridge_config.py -- PURPOSE: prove the settings layer of the bridge: the table of settings
and the shipped profile file agree exactly, every value is checked (type, range, choices) with a message that says
what to fix, a mistyped YAML key is an error with a "did you mean", command line beats environment beats file beats
default, the secrets are read from the right places and never shown, a secrets file inside a git folder is refused,
and the world files become the right areas. Pure Python plus PyYAML; no network.
"""

import os
from datetime import datetime, timezone

import pytest
import yaml

from lattice_bridge import bridge_config as BC
from lattice_bridge import constants as C
from support_net import GOOD_TOKEN, OTHER_TOKEN, SANDBOX_TOKEN

PROFILE = BC.Path(__file__).resolve().parent.parent / 'config' / 'bridge.yaml'
START = datetime(2026, 10, 24, 15, 30, 12, tzinfo=timezone.utc)


def line(name, value, sep='='):
    """One `NAME=value` line, assembled at run time so no file contains a literal NAME=value pair."""
    return f'{name}{sep}{value}'


def write(path, text):
    path.write_text(text, encoding='utf-8')
    return str(path)


# ------------------------------------------------------------------------------------------ the table and the profile
def test_the_shipped_profile_lists_every_setting_with_its_default_and_nothing_else():
    tree = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    flat = BC._flatten(tree)
    assert sorted(flat) == sorted(opt.path for opt in BC.OPTIONS)
    values = BC.values_from_yaml(tree, 'bridge.yaml')
    default = BC.BridgeConfig()
    assert [o.path for o in BC.OPTIONS if getattr(BC.BridgeConfig(values), o.key) != getattr(default, o.key)] == []


def test_the_profile_says_where_secrets_belong_and_holds_none():
    text = PROFILE.read_text(encoding='utf-8')
    assert 'LATTICE_TOKEN' in text and 'Never put a token in this file' in text


def test_every_setting_has_a_plain_words_explanation_and_flags_are_unique():
    assert all(len(opt.doc) > 15 for opt in BC.OPTIONS)
    flags = [opt.cli for opt in BC.OPTIONS if opt.cli]
    assert len(flags) == len(set(flags)) and all(flag.startswith('--') for flag in flags)
    assert len({opt.path for opt in BC.OPTIONS}) == len(BC.OPTIONS)


def test_the_defaults_are_the_safe_ones():
    cfg = BC.BridgeConfig()
    assert cfg.sink == 'jsonl'                         # nothing leaves the computer unless somebody asks
    assert cfg.source == 'mavlink' and cfg.drone_stale_policy == 'stop'
    assert cfg.altitude_geoid_undulation_m is None     # unknown: the bridge warns instead of guessing
    assert cfg.rest_endpoint == '' and not hasattr(cfg, 'rest_token')


def test_a_config_is_read_only_and_with_values_checks_what_it_is_given():
    cfg = BC.BridgeConfig()
    with pytest.raises(AttributeError):
        cfg.sink = 'rest'
    with pytest.raises(AttributeError):
        cfg.no_such_setting
    assert cfg.with_values(drone_interval_s='2.5').drone_interval_s == 2.5
    with pytest.raises(BC.ConfigError):
        cfg.with_values(drone_interval_s=0)
    with pytest.raises(BC.ConfigError):
        cfg.with_values(no_such=1)
    with pytest.raises(KeyError):
        BC.BridgeConfig({'no_such': 1})
    assert cfg.as_dict()['mavlink.connection'] == 'udpin:127.0.0.1:14550'


# ------------------------------------------------------------------------------------------ checking one value
def opt(path):
    return BC._BY_PATH[path]


@pytest.mark.parametrize('path,given,expected', [
    ('drone.name', '  Hawk  ', 'Hawk'), ('mavlink.background_reader', False, False),
    ('source', 'MAVLINK', 'mavlink'), ('log.level', 'debug', 'DEBUG'),
    ('drone.interval_s', '2.5', 2.5), ('drone.interval_s', 3, 3.0), ('mavlink.vehicle_sysid', '7', 7),
    ('mavlink.vehicle_sysid', 7.0, 7), ('altitude.geoid_undulation_m', None, None),
    ('altitude.geoid_undulation_m', '-30', -30.0), ('mavlink.tgt_severities', [6, 5], [6, 5]),
    ('drops.channels', {'11': 'person', 13: 'tent'}, {11: 'person', 13: 'tent'}),
])
def test_good_values_come_back_in_the_right_type(path, given, expected):
    result = BC.coerce(opt(path), given)
    assert result == expected and type(result) is type(expected)


@pytest.mark.parametrize('path,bad', [
    ('drone.name', 5), ('mavlink.background_reader', 'yes'), ('mavlink.background_reader', 1), ('source', 'carrier-pigeon'),
    ('log.level', 'LOUD'), ('drone.interval_s', 'fast'), ('drone.interval_s', True), ('drone.interval_s', float('nan')),
    ('drone.interval_s', float('inf')), ('drone.interval_s', 0), ('drone.interval_s', 10 ** 6),
    ('mavlink.vehicle_sysid', 7.5), ('mavlink.vehicle_sysid', -1), ('altitude.geoid_undulation_m', 'deep'),
    ('mavlink.tgt_severities', []), ('mavlink.tgt_severities', [9]), ('mavlink.tgt_severities', 'info'),
    ('mavlink.tgt_severities', [True]), ('drops.channels', {}), ('drops.channels', {'x': 'person'}),
    ('drops.channels', {11: 'cat'}), ('drops.channels', {0: 'person'}), ('drops.channels', [11, 13]),
])
def test_bad_values_are_refused_with_the_name_of_the_setting(path, bad):
    with pytest.raises(BC.ConfigError) as caught:
        BC.coerce(opt(path), bad)
    assert path in str(caught.value)


# ------------------------------------------------------------------------------------------ YAML
def test_a_mistyped_key_is_an_error_with_a_hint(tmp_path):
    profile = write(tmp_path / 'p.yaml', 'drone:\n  stale_aftre_s: 5\n')
    with pytest.raises(BC.ConfigError) as caught:
        BC.resolve_config(yaml_path=profile)
    assert 'drone.stale_aftre_s' in str(caught.value) and 'Did you mean "drone.stale_after_s"' in str(caught.value)


def test_nested_and_dotted_keys_both_work(tmp_path):
    nested = write(tmp_path / 'a.yaml', 'ros:\n  topics:\n    fix: /my/fix\nmavlink:\n  connection: udpin:0.0.0.0:14551\n')
    assert BC.resolve_config(yaml_path=nested).ros_topics_fix == '/my/fix'
    dotted = write(tmp_path / 'b.yaml', "mavlink.connection: 'udpin:0.0.0.0:14552'\n")
    assert BC.resolve_config(yaml_path=dotted).mavlink_connection == 'udpin:0.0.0.0:14552'


def test_the_format_version_is_checked(tmp_path):
    assert BC.resolve_config(yaml_path=write(tmp_path / 'ok.yaml', 'profile: 1\nsink: mock\n')).sink == 'mock'
    with pytest.raises(BC.ConfigError, match='profile'):
        BC.resolve_config(yaml_path=write(tmp_path / 'bad.yaml', 'profile: 2\n'))


@pytest.mark.parametrize('text,fragment', [('- a\n- b\n', 'mapping'), ('a: [unclosed\n', 'not valid YAML')])
def test_a_profile_that_is_not_a_mapping_or_not_yaml_is_a_clear_error(tmp_path, text, fragment):
    with pytest.raises(BC.ConfigError, match=fragment):
        BC.resolve_config(yaml_path=write(tmp_path / 'p.yaml', text))


def test_a_missing_profile_is_a_clear_error(tmp_path):
    with pytest.raises(BC.ConfigError, match='cannot read'):
        BC.resolve_config(yaml_path=str(tmp_path / 'nope.yaml'))


def test_an_empty_profile_means_all_defaults(tmp_path):
    assert BC.resolve_config(yaml_path=write(tmp_path / 'e.yaml', '')).as_dict() == BC.BridgeConfig().as_dict()


# ------------------------------------------------------------------------------------------ who wins
def test_command_line_beats_environment_beats_file_beats_default(tmp_path):
    profile = write(tmp_path / 'p.yaml', 'sink: rest\nrest:\n  endpoint: from-file.example\n  timeout_s: 20\n')
    assert BC.resolve_config(yaml_path=profile, env={}).rest_endpoint == 'from-file.example'
    assert BC.resolve_config(yaml_path=profile, env={'LATTICE_ENDPOINT': 'from-env.example'}).rest_endpoint == 'from-env.example'
    cli = {'rest_endpoint': 'from-cli.example'}
    assert BC.resolve_config(yaml_path=profile, cli=cli, env={'LATTICE_ENDPOINT': 'from-env.example'}).rest_endpoint == 'from-cli.example'
    assert BC.resolve_config(yaml_path=profile, env={}).rest_timeout_s == 20.0
    assert BC.resolve_config(env={}).rest_timeout_s == 10.0


def test_command_line_flags_become_option_values():
    import argparse
    parser = argparse.ArgumentParser()
    BC.add_cli_options(parser)
    namespace = parser.parse_args(['--sink', 'mock', '--speed', '4', '--log-level', 'debug', '--stale-after', '20'])
    assert BC.values_from_cli(namespace) == {'sink': 'mock', 'replay_speed': 4.0, 'log_level': 'DEBUG', 'drone_stale_after_s': 20.0}
    assert BC.values_from_cli(parser.parse_args([])) == {}
    with pytest.raises(SystemExit):
        parser.parse_args(['--sink', 'pigeon'])


# ------------------------------------------------------------------------------------------ cross-checks
@pytest.mark.parametrize('changes,fragment', [
    ({'drone_offline_after_s': 5.0, 'drone_stale_after_s': 10.0}, 'offline_after_s'),
    ({'mission_id': 'Bad Id!'}, 'mission.id'),
    ({'source': 'replay'}, '--run-dir'),
    ({'sink': 'rest', 'source': 'replay', 'replay_run_dir': 'x', 'replay_speed': 5.0}, 'real time'),
    ({'sink': 'rest', 'rest_endpoint': 'http://host/with/a/path'}, 'without a path'),
    ({'mission_duration_s': 30 * 86400.0 - 1}, 'too long'),
])
def test_settings_that_do_not_fit_together_are_refused(changes, fragment):
    cfg = BC.BridgeConfig({key: value for key, value in changes.items()})
    with pytest.raises(BC.ConfigError) as caught:
        BC.check_config(cfg)
    assert fragment in str(caught.value)


def test_a_real_time_replay_into_lattice_is_allowed():
    BC.check_config(BC.BridgeConfig({'sink': 'rest', 'source': 'replay', 'replay_run_dir': 'x', 'replay_speed': 1.0}))


@pytest.mark.parametrize('given,expected', [
    ('lattice.example.com', 'https://lattice.example.com'), ('lattice.example.com:8443', 'https://lattice.example.com:8443'),
    ('http://127.0.0.1:8089', 'http://127.0.0.1:8089'), ('HTTPS://Host.Example/', 'https://host.example'),
    ('  my-env.lattice.example  ', 'https://my-env.lattice.example'), ('http://[::1]:8089', 'http://[::1]:8089'),
])
def test_endpoints_are_normalised(given, expected):
    assert BC.normalise_endpoint(given) == expected


@pytest.mark.parametrize('bad', ['', '   ', 'ftp://host', 'http://', 'https://user:pw@host', 'https://host/path',
                                 'https://host?x=1', 'https://host:99999', 'https://host:abc'])
def test_unusable_endpoints_are_refused_and_a_password_is_never_echoed(bad):
    with pytest.raises(BC.ConfigError) as caught:
        BC.normalise_endpoint(bad)
    assert 'pw' not in str(caught.value).replace('password', '')


# ------------------------------------------------------------------------------------------ secrets
def test_secrets_come_from_the_environment_and_never_show_in_text():
    env = {'LATTICE_ENDPOINT': 'lattice.example', 'LATTICE_TOKEN': GOOD_TOKEN, 'SANDBOXES_TOKEN': SANDBOX_TOKEN}
    secrets = BC.load_secrets(env)
    assert (secrets.endpoint, secrets.token, secrets.sandbox_token) == ('lattice.example', GOOD_TOKEN, SANDBOX_TOKEN)
    shown = repr(secrets) + str(secrets)
    assert GOOD_TOKEN not in shown and SANDBOX_TOKEN not in shown and 'token=set' in shown
    assert secrets.values() == [GOOD_TOKEN, SANDBOX_TOKEN]                    # the endpoint is not secret
    assert 'not set' in repr(BC.load_secrets({}))


def test_redact_blanks_every_secret_wherever_it_appears():
    secrets = BC.load_secrets({'LATTICE_TOKEN': GOOD_TOKEN, 'SANDBOXES_TOKEN': SANDBOX_TOKEN})
    text = f'sent {GOOD_TOKEN} and {SANDBOX_TOKEN} and {GOOD_TOKEN}'
    cleaned = BC.redact(text, secrets)
    assert GOOD_TOKEN not in cleaned and SANDBOX_TOKEN not in cleaned and cleaned.count('***') == 3
    assert BC.redact('nothing here', secrets) == 'nothing here'
    assert BC.redact('abc', BC.load_secrets({'LATTICE_TOKEN': 'abc'})) == 'abc'      # too short to be a secret: left alone


def test_a_secrets_file_fills_in_what_the_environment_lacks_but_never_overrides_it(tmp_path):
    body = '\n'.join([
        '# comment',
        '',
        'export ' + line('LATTICE_TOKEN', '"' + OTHER_TOKEN + '"'),         # "export" and quotes are accepted
        line('SANDBOXES_TOKEN', SANDBOX_TOKEN, ' = '),                       # spaces around the sign are accepted
        line('LATTICE_ENDPOINT', "'file.example'"),
        'UNRELATED=1',                                                       # a name we do not use is ignored
        'lower_case=x',
        ''])
    path = write(tmp_path / 'lattice.env', body)
    if os.name == 'posix':
        os.chmod(path, 0o600)
    secrets = BC.load_secrets({'LATTICE_TOKEN': GOOD_TOKEN}, path)
    assert secrets.token == GOOD_TOKEN                                          # the environment wins
    assert secrets.sandbox_token == SANDBOX_TOKEN and secrets.endpoint == 'file.example'
    assert secrets.source == 'environment + secrets file'
    assert BC.load_secrets({}, path).token == OTHER_TOKEN                       # quotes and "export" are accepted


def test_a_secrets_file_inside_a_git_folder_is_refused(tmp_path):
    repo = tmp_path / 'repo'
    (repo / '.git').mkdir(parents=True)
    path = write(repo / 'lattice.env', line('LATTICE_TOKEN', GOOD_TOKEN) + '\n')
    with pytest.raises(BC.ConfigError) as caught:
        BC.load_secrets({}, path)
    assert 'git repository' in str(caught.value) and GOOD_TOKEN not in str(caught.value)


def test_a_missing_secrets_file_is_a_clear_error(tmp_path):
    with pytest.raises(BC.ConfigError, match='cannot read the secrets file'):
        BC.load_secrets({}, str(tmp_path / 'nope.env'))


@pytest.mark.skipif(os.name != 'posix', reason='file modes are a POSIX idea')
def test_a_secrets_file_that_others_can_read_is_refused(tmp_path):
    path = write(tmp_path / 'lattice.env', line('LATTICE_TOKEN', GOOD_TOKEN) + '\n')
    os.chmod(path, 0o644)
    with pytest.raises(BC.ConfigError, match='chmod 600'):
        BC.load_secrets({}, path)


# ------------------------------------------------------------------------------------------ areas, mission, core settings
CONFIG = """\
home: "36.2143000,-96.0068000,210.0,0"
truth: truth/runway.yaml
search_areas:
  - {name: search_box, center: [25.0, 95.0], size: [120.0, 100.0], yaw: 0.0}
  - {name: second_box, center: [-100.0, 40.0], size: [60.0, 60.0], yaw: 0.5, note: the second one}
"""
TRUTH = """\
flight_boundary:
  latlon: [[36.2190, -96.0020], [36.2200, -96.0040], [36.2210, -96.0010]]
altitude_rule_agl_m: {min: 30.0, max: 120.0}
targets:
  - {cls: person, lat: 36.2151, lon: -96.0062}
"""


@pytest.fixture()
def world(tmp_path):
    (tmp_path / 'config').mkdir()
    (tmp_path / 'truth').mkdir()
    write(tmp_path / 'config' / 'runway.yaml', CONFIG)
    write(tmp_path / 'truth' / 'runway.yaml', TRUTH)
    return tmp_path


def test_areas_come_from_the_config_and_the_boundary_from_the_truth_file_it_names(world):
    specs = BC.load_areas(str(world / 'config' / 'runway.yaml'))
    assert [s.name for s in specs] == ['flight_boundary', 'search_box', 'second_box']
    assert specs[0].geo_type == C.GEO_TYPE_FLIGHT_BOUNDARY and '30.0-120.0 m' in specs[0].note
    assert specs[2].note == 'the second one' and all(len(s.corners_latlon) >= 3 for s in specs)


def test_the_answer_key_never_reaches_the_areas(world):
    specs = BC.load_areas(str(world / 'config' / 'runway.yaml'))
    flat = repr(specs)
    assert '36.2151' not in flat and '-96.0062' not in flat                       # the true target position


def test_an_explicit_truth_path_wins_and_a_world_without_truth_still_works(world, tmp_path):
    other = write(tmp_path / 'other_truth.yaml', 'flight_boundary:\n  latlon: [[36.0, -96.0], [36.1, -96.1], [36.2, -96.0]]\n')
    specs = BC.load_areas(str(world / 'config' / 'runway.yaml'), other)
    assert specs[0].corners_latlon[0] == (36.0, -96.0)
    bare = write(tmp_path / 'bare.yaml', CONFIG.replace('truth: truth/runway.yaml\n', ''))
    assert [s.name for s in BC.load_areas(bare)] == ['search_box', 'second_box']


@pytest.mark.parametrize('text,fragment', [
    ('home: "36.2,-96.0"\nsearch_areas: []\n', 'no search areas'),
    ('home: nonsense\nsearch_areas:\n  - {name: a, center: [0, 0], size: [1, 1]}\n', 'home'),
    ('home: "36.2,-96.0"\nsearch_areas:\n  - {center: [0, 0], size: [1, 1]}\n', 'needs a name'),
    ('home: "36.2,-96.0"\nsearch_areas:\n  - {name: a, center: [0, 0], size: [0, 5]}\n', 'greater than 0'),
])
def test_a_broken_world_config_is_a_clear_error(tmp_path, text, fragment):
    with pytest.raises(BC.ConfigError, match=fragment):
        BC.load_areas(write(tmp_path / 'c.yaml', text))


def test_a_broken_truth_boundary_is_a_clear_error(world, tmp_path):
    bad = write(tmp_path / 't.yaml', 'flight_boundary:\n  latlon: [[36.0]]\n')
    with pytest.raises(BC.ConfigError, match='flight boundary'):
        BC.load_areas(str(world / 'config' / 'runway.yaml'), bad)


def test_the_mission_id_is_the_configured_one_or_made_from_the_start_time():
    assert BC.resolve_mission_id(BC.BridgeConfig({'mission_id': 'field1'}), START) == 'field1'
    assert BC.resolve_mission_id(BC.BridgeConfig(), START) == 'm20261024t153012z'
    assert BC.resolve_mission_id(BC.BridgeConfig(), START, 'runway') == 'runway20261024t153012z'
    assert BC.resolve_mission_id(BC.BridgeConfig({'mission_label': 'Test 1'}), START, 'runway') == 'test120261024t153012z'


def test_the_core_gets_every_setting_it_needs():
    cfg = BC.BridgeConfig({'altitude_geoid_undulation_m': -30.0, 'drone_interval_s': 2.0, 'drone_name': 'Hawk',
                           'targets_min_confidence': 0.4, 'mission_duration_s': 1800.0, 'drone_stale_policy': 'not_live'})
    settings = BC.core_settings(cfg, 'field1')
    assert (settings.mission_id, settings.geoid_undulation_m, settings.drone_interval_s) == ('field1', -30.0, 2.0)
    assert (settings.uav_name, settings.min_confidence, settings.mission_duration_s) == ('Hawk', 0.4, 1800.0)
    assert settings.stale_policy == 'not_live' and settings.areas == ()
    settings.validate()
