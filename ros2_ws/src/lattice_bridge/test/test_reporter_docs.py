"""
lattice_bridge/test/test_reporter_docs.py -- PURPOSE: keep the reporter's settings file honest. The file
config/lattice_reporter.yaml promises "the values below ARE the defaults: delete a line and nothing changes" and
lists every setting with an explanation. This test proves it: the file names exactly the settings of
reporter_logic.PARAMETERS (none missing, none extra, none misspelled), under the node name the launch file uses,
with the same default values and types. It also checks that the simulator profile that starts the reporter still
points at this package's launch file. Needs PyYAML (skipped without it); no ROS.
"""

from pathlib import Path

import pytest

from lattice_bridge.reporter_logic import PARAMETERS

yaml = pytest.importorskip('yaml')

PKG = Path(__file__).resolve().parent.parent
CONFIG = PKG / 'config' / 'lattice_reporter.yaml'
LAUNCH = PKG / 'launch' / 'lattice_reporter.launch.py'
PROFILE = PKG.parent.parent.parent / 'simulation' / 'profiles' / 'lattice_mock.yaml'


def _settings():
    data = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    assert list(data) == ['lattice_reporter'], 'the file must hold the settings of node lattice_reporter only'
    return data['lattice_reporter']['ros__parameters']


def test_the_settings_file_lists_exactly_the_reporters_parameters():
    in_file = set(_settings())
    in_code = {spec.name for spec in PARAMETERS}
    assert in_file - in_code == set(), 'settings in the file that the reporter does not know (misspelled?)'
    assert in_code - in_file == set(), 'settings of the reporter missing from the file'


@pytest.mark.parametrize('spec', PARAMETERS, ids=lambda spec: spec.name)
def test_every_value_in_the_settings_file_is_the_built_in_default(spec):
    value = _settings()[spec.name]
    assert value == spec.default and type(value) is type(spec.default), (
        f'{spec.name}: the file says {value!r}, the code default is {spec.default!r}')


def test_the_launch_file_starts_the_node_under_the_name_the_settings_file_uses():
    text = LAUNCH.read_text(encoding='utf-8')
    assert "name='lattice_reporter'" in text and "'lattice_reporter.yaml'" in text


def test_every_number_launch_argument_is_a_float_setting_of_the_reporter():
    """min_confidence:=0.5 must override the setting of the same name, with the same type as the file's value."""
    text = LAUNCH.read_text(encoding='utf-8')
    specs = {spec.name: spec for spec in PARAMETERS}
    names = [name for name in ('min_confidence', 'min_move_m') if f"DeclareLaunchArgument('{name}'" in text]
    assert names == ['min_confidence', 'min_move_m']
    for name in names:
        assert isinstance(specs[name].default, float), name


@pytest.mark.skipif(not PROFILE.is_file(), reason='simulation/profiles is not next to this package (clean overlay)')
def test_the_simulator_profile_starts_the_reporter_with_the_team_stack():
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    reporters = [e for e in profile['extras'] if e.get('name') == 'lattice_reporter']
    assert len(reporters) == 1
    entry = reporters[0]
    assert entry['package'] == 'lattice_bridge' and entry['launch'] == LAUNCH.name
    assert entry['start_with'] == 'team_stack' and entry['node_name'] == 'lattice_reporter'
    specs = {spec.name: spec for spec in PARAMETERS}
    for name, value in entry.get('args', {}).items():                   # launch arguments it overrides
        assert name in specs and specs[name].low <= float(value) <= specs[name].high, name


@pytest.mark.skipif(not PROFILE.is_file(), reason='simulation/profiles is not next to this package (clean overlay)')
def test_the_bridge_listens_behind_a_ground_station_that_hears_the_jetsons_texts():
    """With MAVROS behind MAVProxy (this profile's fcu_url), the team's TGT and Servo texts reach the ground only on
    another autopilot port, so the bridge listens behind its own ground station on SERIAL1 (see the profile's
    header)."""
    profile = yaml.safe_load(PROFILE.read_text(encoding='utf-8'))
    extras = {e['name']: e for e in profile['extras']}
    radio = ' '.join(extras['lattice_ground_radio']['command'])
    assert '--master tcp:127.0.0.1:5762' in radio and '--out udp:127.0.0.1:14554' in radio
    assert '--source-system 254' in radio and '--tcp-port 5762' in radio
    assert 'udpin:127.0.0.1:14554' in extras['lattice_bridge']['command']
    assert extras['lattice_ground_radio']['start_with'] == 'simulator'
    # TEMPORARY WORKAROUND, locked in on purpose: MAVROS stays behind MAVProxy until mission_tool ignores MAVROS's
    # heartbeat (see the profile's header). Whoever fixes mission_tool and deletes the profile's fcu_url line, as
    # the header says, must change this assert too (and may then also replace the ground station by a MAVProxy
    # output on 14554, which works with MAVROS on SERIAL2: lattice run 20261031-094007_runway got all 8 TGT texts).
    assert profile['network']['fcu_url'] == 'udp://:14552@localhost:14552'
    # Nothing else may send to the bridge's port: with the ground station, a MAVProxy output on 14554 as well would
    # deliver every message twice.
    assert 'udp:127.0.0.1:14554' not in profile['network'].get('extra_outs', [])
