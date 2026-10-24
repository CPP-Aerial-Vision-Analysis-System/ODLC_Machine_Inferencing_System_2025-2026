"""
lattice_bridge/bridge_config.py -- PURPOSE: every setting of the bridge in ONE table, and the code that reads
them from the three places a person can put them: built-in defaults, a YAML profile (config/bridge.yaml), and the
command line. The table `OPTIONS` is the single source of truth: the defaults, the YAML checks, the --help text,
the documentation test and the dry-run plan are all made from it, so they cannot disagree.

Order of authority (later wins): built-in default < YAML profile < environment (endpoint only) < command line.
Unknown YAML keys are an ERROR with a "did you mean ...?" hint, because a mistyped key that is silently ignored
(`stale_aftre_s`) is how a safety setting quietly stays at its default.

Secrets (the Lattice endpoint and tokens) are different from settings. They come ONLY from environment variables
(LATTICE_ENDPOINT, LATTICE_TOKEN, SANDBOXES_TOKEN) or from a small KEY=VALUE file that lives OUTSIDE any git
repository (--secrets-file); there is deliberately no command-line flag for a token, because command lines are
visible to every user of the computer (`ps`). The Secrets object prints as "set"/"not set", and redact() scrubs a
secret out of any text before it can reach a log.

Plain Python plus PyYAML (imported only when a YAML file is read, with a clear message if it is missing).
"""

from __future__ import annotations

import difflib
import math
import os
import re
import stat
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple
from urllib.parse import urlparse

from . import constants as C
from .bridge_core import STALE_POLICIES, CoreSettings
from .ids import check_mission_id, make_mission_id
from .world_areas import AreaSpec, flight_boundary, search_areas

SOURCES = ('mavlink', 'ros', 'replay')
SINKS = ('rest', 'jsonl', 'mock')
ENV_ENDPOINT = 'LATTICE_ENDPOINT'
ENV_TOKEN = 'LATTICE_TOKEN'
ENV_SANDBOX_TOKEN = 'SANDBOXES_TOKEN'
SECRET_NAMES = (ENV_ENDPOINT, ENV_TOKEN, ENV_SANDBOX_TOKEN)

# Exit codes of the program (documented in --help and docs/lattice/BRIDGE.md).
EXIT_OK = 0
EXIT_USAGE = 2          # argparse's own code for a bad command line
EXIT_CONFIG = 3         # the settings are wrong (bad YAML, missing file, missing endpoint ...)
EXIT_SOURCE = 4         # the chosen source or sink cannot work here (library missing, run folder missing ...)
EXIT_INTERNAL = 5       # a bug: something nobody expected


class ConfigError(ValueError):
    """The settings are wrong; the message says which one and how to fix it (the program exits with EXIT_CONFIG)."""


@dataclass(frozen=True)
class Opt:
    """One setting. `path` is the dotted name in the YAML file; the program uses `key` (dots become underscores)."""
    path: str
    kind: str                       # str | int | float | bool | choice | opt_float | int_list | drop_map
    default: Any
    doc: str
    cli: str = ''                   # the matching command-line flag ('' = YAML only)
    choices: Tuple[str, ...] = ()
    low: Optional[float] = None
    high: Optional[float] = None

    @property
    def key(self) -> str:
        """The attribute name on BridgeConfig (`mavlink.connection` -> `mavlink_connection`)."""
        return self.path.replace('.', '_')


def _o(path, kind, default, doc, cli='', **kw) -> Opt:
    return Opt(path, kind, default, doc, cli, **kw)


#: Every setting of the bridge. Keep the YAML profile (config/bridge.yaml) in step: a test checks that it
#: mentions every `path` here, and that its values equal the defaults.
OPTIONS: Tuple[Opt, ...] = (
    _o('source', 'choice', 'mavlink', 'where the facts come from: mavlink (the telemetry stream, for the ground '
       'laptop), ros (MAVROS topics, for a bridge on the Jetson) or replay (a recorded simulator run)',
       '--source', choices=SOURCES),
    _o('sink', 'choice', 'jsonl', 'where the finished requests go: jsonl (a file; the safe default), mock (a '
       'built-in test sink) or rest (the real or fake Lattice over HTTP)', '--sink', choices=SINKS),
    _o('mission.id', 'str', '', 'name of this mission inside every entity id; empty = made from the start time and '
       'the label, so a later flight never overwrites an earlier one', '--mission-id'),
    _o('mission.label', 'str', '', 'short word put in front of the generated mission id (letters and digits); '
       'empty = the replayed world, or "m"', '--mission-label'),
    _o('mission.duration_s', 'float', 3600.0, 'planned length of the mission in seconds; targets expire this long '
       'after the bridge started plus the margin below', '--mission-duration', low=60.0, high=7 * 86400.0),
    _o('mission.target_margin_s', 'float', C.TARGET_EXPIRY_MARGIN_S, 'extra seconds a target stays on the map after '
       'the planned end (for the debrief)', low=0.0, high=7 * 86400.0),
    _o('jsonl.path', 'str', 'lattice_requests.jsonl', 'file the jsonl sink appends to (one request per line)',
       '--output'),
    _o('rest.endpoint', 'str', '', 'Lattice address, e.g. my-env.lattice.example or http://127.0.0.1:8765 for the '
       'fake Lattice; empty = take it from the LATTICE_ENDPOINT environment variable', '--endpoint'),
    _o('rest.timeout_s', 'float', 10.0, 'seconds before one HTTP request to Lattice is given up', low=0.5,
       high=120.0),
    _o('secrets.file', 'str', '', 'optional file with one NAME=value line for each of LATTICE_ENDPOINT, LATTICE_TOKEN and SANDBOXES_TOKEN; it '
       'must live OUTSIDE any git repository', '--secrets-file'),
    _o('altitude.geoid_undulation_m', 'opt_float', None, 'height of the geoid above the WGS84 ellipsoid at the field '
       '(about -30 in Oklahoma). Lattice wants height above the ELLIPSOID, MAVLink gives height above SEA LEVEL: '
       'HAE = AMSL + this. Empty/null = not configured: the sea-level height is published as it is and a loud '
       'warning is logged (0 is a deliberate choice)', '--geoid-undulation', low=-120.0, high=120.0),
    _o('altitude.ros_altitude_is', 'choice', 'hae', 'what NavSatFix.altitude means on the ROS topics: hae (above the '
       'ellipsoid, what MAVROS documents) or amsl (above sea level)', choices=('hae', 'amsl')),
    _o('drone.name', 'str', 'ASTRA UAV', 'name shown for the drone in Lattice'),
    _o('drone.interval_s', 'float', 1.0, 'seconds between drone updates (a new position is needed for each)',
       low=0.2, high=60.0),
    _o('drone.stale_after_s', 'float', 10.0, 'no position for this many seconds = the link is stale and the drone '
       'is no longer refreshed', '--stale-after', low=1.0, high=3600.0),
    _o('drone.stale_policy', 'choice', 'stop', 'what a stale link does: stop (just stop refreshing; the entry '
       'expires by itself after 2 minutes) or not_live (also send isLive=false after offline_after_s)',
       '--stale-policy', choices=STALE_POLICIES),
    _o('drone.offline_after_s', 'float', 30.0, 'with stale_policy not_live: seconds of silence before the '
       'isLive=false update', low=1.0, high=3600.0),
    _o('targets.min_confidence', 'float', 0.0, 'reports below this confidence (0..1) are ignored', low=0.0, high=1.0),
    _o('targets.merge_radius_m', 'float', C.CLUSTER_MERGE_RADIUS_M, 'two reports of the same kind closer than this '
       'are the same physical target', low=0.5, high=200.0),
    _o('targets.update_min_move_m', 'float', C.CLUSTER_UPDATE_MIN_MOVE_M, 'send a target update only if its '
       'position moved this far', low=0.0, high=100.0),
    _o('targets.update_min_conf_gain', 'float', C.CLUSTER_UPDATE_MIN_CONF_GAIN, 'or if its confidence grew this '
       'much (0..1)', low=0.0, high=1.0),
    _o('targets.max_clusters', 'int', C.CLUSTER_MAX_CLUSTERS, 'at most this many targets are tracked', low=1, high=1000),
    _o('areas.config', 'str', '', 'a simulator world file (sim_worlds config/<world>.yaml): its search areas are '
       'published once at start; empty = no areas', '--area-config'),
    _o('areas.truth', 'str', '', 'optional sim_worlds truth/<world>.yaml: ONLY its flight_boundary and altitude '
       'rule are read (never the target positions); empty = look for the file the config names',
       '--truth-config'),
    _o('areas.ttl_s', 'float', C.AREA_TTL_S, 'seconds the areas stay in Lattice (finite, never "no expiry")',
       low=60.0, high=7 * 86400.0),
    _o('drops.channels', 'drop_map', {11: C.KIND_PERSON, 13: C.KIND_TENT}, 'servo channel -> payload kind of the '
       'release line "Servo <channel> -> <pwm>" (main_controller.py: person 11, tent 13)'),
    _o('drops.open_pwm', 'int', 1400, 'the pulse width that means "released" (main_controller.py PULLEY_OPEN)',
       low=500, high=2500),
    _o('drops.ttl_s', 'float', C.AREA_TTL_S, 'seconds a drop point stays in Lattice', low=60.0, high=7 * 86400.0),
    _o('mavlink.connection', 'str', 'udpin:127.0.0.1:14550', 'pymavlink connection string: udpin:HOST:PORT '
       '(listen), tcp:HOST:PORT, or a serial device like /dev/ttyUSB0,57600', '--connection'),
    _o('mavlink.vehicle_sysid', 'int', 0, 'MAVLink system id of the drone; 0 = the first autopilot heard',
       low=0, high=255),
    _o('mavlink.time_mode', 'choice', 'auto', 'which clock stamps a position: auto (the autopilot clock while it '
       'agrees with this computer, else arrival time), sensor, or arrival', choices=('auto', 'sensor', 'arrival')),
    _o('mavlink.max_clock_skew_s', 'float', 2.0, 'auto mode: the autopilot clock is believed only while it is '
       'within this many seconds of the arrival time', low=0.1, high=55.0),
    _o('mavlink.tgt_severities', 'int_list', [C.STATUSTEXT_SEVERITY_INFO], 'STATUSTEXT severities accepted for TGT '
       'reports and release lines (6 = INFO, what the Jetson sends)'),
    _o('mavlink.silent_notice_s', 'float', 10.0, 'log a notice when nothing at all arrived for this long', low=1.0,
       high=3600.0),
    _o('mavlink.reconnect_after_silence_s', 'opt_float', None, 'reopen the connection after this much silence; '
       'empty/null = 30 s for tcp/serial, never for udp', low=1.0, high=3600.0),
    _o('mavlink.background_reader', 'bool', True, 'read the radio stream in its own thread so a slow sink can '
       'never make telemetry or target reports back up'),
    _o('ros.node_name', 'str', 'lattice_bridge_source', 'name of the ROS node the ros source creates'),
    _o('ros.topics.fix', 'str', '/mavros/global_position/global', 'sensor_msgs/NavSatFix'),
    _o('ros.topics.rel_alt', 'str', '/mavros/global_position/rel_alt', 'std_msgs/Float64, height above home'),
    _o('ros.topics.compass', 'str', '/mavros/global_position/compass_hdg', 'std_msgs/Float64, degrees'),
    _o('ros.topics.state', 'str', '/mavros/state', 'mavros_msgs/State: armed flag and flight mode'),
    _o('ros.topics.battery', 'str', '/mavros/battery', 'sensor_msgs/BatteryState'),
    _o('ros.topics.statustext_recv', 'str', '/mavros/statustext/recv', 'texts FROM the autopilot'),
    _o('ros.topics.statustext_send', 'str', '/mavros/statustext/send', 'texts the Jetson\'s nodes SEND (TGT, releases)'),
    _o('replay.run_dir', 'str', '', 'the run folder to replay, e.g. ~/.astra_sim/runs/<date>_<world>', '--run-dir'),
    _o('replay.speed', 'float', 1.0, 'play at this many times real time; 0 = as fast as possible', '--speed',
       low=0.0, high=10000.0),
    _o('replay.time_mode', 'choice', 'shift', 'shift = move the run so it starts now (a real Lattice refuses old '
       'times); recorded = keep the original dates', choices=('shift', 'recorded')),
    _o('replay.step_s', 'float', 1.0, 'speed 0 only: seconds of recorded time per step', low=0.01, high=3600.0),
    _o('log.level', 'choice', 'INFO', 'DEBUG, INFO, WARNING or ERROR', '--log-level',
       choices=('DEBUG', 'INFO', 'WARNING', 'ERROR')),
    _o('log.health_interval_s', 'float', 30.0, 'seconds between the one-line health messages; 0 = never',
       '--health-interval', low=0.0, high=86400.0),
)

_BY_PATH = {opt.path: opt for opt in OPTIONS}
_BY_KEY = {opt.key: opt for opt in OPTIONS}
assert len(_BY_PATH) == len(OPTIONS) == len(_BY_KEY), 'duplicate option in OPTIONS'


class BridgeConfig:
    """The resolved settings. Read them as attributes with underscores: `cfg.mavlink_connection`, `cfg.drone_interval_s`."""

    def __init__(self, values: Optional[Mapping[str, Any]] = None):
        merged = {opt.key: _copy_default(opt) for opt in OPTIONS}
        for key, value in (values or {}).items():
            if key not in merged:
                raise KeyError(key)
            merged[key] = value
        object.__setattr__(self, '_values', merged)

    def __getattr__(self, name: str) -> Any:
        try:
            return object.__getattribute__(self, '_values')[name]
        except KeyError:
            raise AttributeError(name) from None

    def __setattr__(self, name: str, value: Any) -> None:
        raise AttributeError('BridgeConfig is read-only; use with_values()')

    def with_values(self, **changes: Any) -> 'BridgeConfig':
        """A copy with some settings replaced (keys use underscores). Values are checked like any others."""
        merged = dict(object.__getattribute__(self, '_values'))
        for key, value in changes.items():
            if key not in _BY_KEY:
                raise ConfigError(f'unknown setting {key!r}')
            merged[key] = coerce(_BY_KEY[key], value)
        return BridgeConfig(merged)

    def as_dict(self) -> Dict[str, Any]:
        """{dotted path: value} for every setting (used by the plan and the tests)."""
        values = object.__getattribute__(self, '_values')
        return {opt.path: values[opt.key] for opt in OPTIONS}


def _copy_default(opt: Opt) -> Any:
    return dict(opt.default) if isinstance(opt.default, dict) else list(opt.default) if isinstance(opt.default, list) \
        else opt.default


# ===================================================================================================
# Checking and converting values
# ===================================================================================================

def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def coerce(opt: Opt, value: Any) -> Any:
    """Check `value` against the option's kind and range and return it in the right Python type, or raise ConfigError."""
    where = f'setting "{opt.path}"'
    kind = opt.kind
    if kind == 'str':
        if not isinstance(value, str):
            raise ConfigError(f'{where} must be text, got {value!r}')
        return value.strip()
    if kind == 'bool':
        if not isinstance(value, bool):
            raise ConfigError(f'{where} must be true or false, got {value!r}')
        return value
    if kind == 'choice':
        text = str(value).strip()
        text = text.upper() if opt.path == 'log.level' else text.lower()
        if text not in opt.choices:
            raise ConfigError(f'{where} must be one of {", ".join(opt.choices)}, got {value!r}')
        return text
    if kind in ('int', 'float', 'opt_float'):
        if value is None and kind == 'opt_float':
            return None
        if isinstance(value, str):
            try:
                value = float(value) if kind != 'int' else int(value)
            except ValueError:
                raise ConfigError(f'{where} must be a number, got {value!r}') from None
        if not _is_number(value) or (kind == 'int' and (isinstance(value, float) and value != int(value))):
            raise ConfigError(f'{where} must be a {"whole " if kind == "int" else ""}number, got {value!r}')
        number = int(value) if kind == 'int' else float(value)
        if opt.low is not None and number < opt.low or opt.high is not None and number > opt.high:
            raise ConfigError(f'{where} must be between {opt.low} and {opt.high}, got {number}')
        return number
    if kind == 'int_list':
        if not isinstance(value, (list, tuple)) or not value or not all(
                isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 7 for v in value):
            raise ConfigError(f'{where} must be a non-empty list of whole numbers from 0 to 7, got {value!r}')
        return list(value)
    if kind == 'drop_map':
        if not isinstance(value, Mapping) or not value:
            raise ConfigError(f'{where} must map channel numbers to person or tent, e.g. {{11: person, 13: tent}}')
        result: Dict[int, str] = {}
        for channel, payload in value.items():
            try:
                number = int(channel)
            except (TypeError, ValueError):
                raise ConfigError(f'{where}: channel {channel!r} is not a whole number') from None
            if not 1 <= number <= 99 or payload not in C.TARGET_KINDS:
                raise ConfigError(f'{where}: "{channel}: {payload}" must be a channel 1-99 and person or tent')
            result[number] = payload
        return result
    raise AssertionError(f'unknown option kind {kind}')


def _flatten(tree: Mapping[str, Any], prefix: str = '') -> Dict[str, Any]:
    """{'mavlink': {'connection': x}} -> {'mavlink.connection': x}; option paths are matched greedily."""
    flat: Dict[str, Any] = {}
    for key, value in tree.items():
        path = f'{prefix}{key}'
        if path in _BY_PATH:
            flat[path] = value
        elif isinstance(value, Mapping) and any(p.startswith(path + '.') for p in _BY_PATH):
            flat.update(_flatten(value, path + '.'))
        else:
            flat[path] = value                                  # unknown: reported by the caller
    return flat


def _suggest(path: str) -> str:
    close = difflib.get_close_matches(path, list(_BY_PATH), n=1, cutoff=0.6)
    return f' Did you mean "{close[0]}"?' if close else ''


# ===================================================================================================
# YAML, environment, command line
# ===================================================================================================

def import_yaml():
    """Import PyYAML or raise ConfigError saying how to get it (only YAML files and area files need it)."""
    try:
        import yaml
    except ImportError:
        raise ConfigError('reading YAML files needs PyYAML, which is not installed. On Ubuntu: '
                          '"sudo apt install python3-yaml" (ROS 2 installs it already), or '
                          '"python3 -m pip install pyyaml".') from None
    return yaml


def read_yaml_file(path: str, what: str) -> Dict[str, Any]:
    """Load a YAML file that must hold a mapping; every problem becomes a ConfigError naming the file."""
    yaml = import_yaml()
    try:
        text = Path(path).expanduser().read_text(encoding='utf-8')
    except OSError as exc:
        raise ConfigError(f'cannot read the {what} file "{path}": {exc.strerror or exc}') from None
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ConfigError(f'the {what} file "{path}" is not valid YAML: {str(exc).splitlines()[0] if str(exc) else exc}') from None
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f'the {what} file "{path}" must hold a YAML mapping (key: value lines)')
    return data


def values_from_yaml(tree: Mapping[str, Any], origin: str = 'the profile') -> Dict[str, Any]:
    """Check a parsed YAML profile and return {option key: converted value}. Unknown keys raise ConfigError."""
    values: Dict[str, Any] = {}
    unknown: List[str] = []
    for path, value in _flatten({k: v for k, v in tree.items() if k != 'profile'}).items():
        opt = _BY_PATH.get(path)
        if opt is None:
            unknown.append(path)
            continue
        values[opt.key] = coerce(opt, value)
    if unknown:
        raise ConfigError(f'{origin} has unknown setting(s): ' + '; '.join(f'"{p}".{_suggest(p)}' for p in unknown[:5]))
    if 'profile' in tree and tree['profile'] != 1:
        raise ConfigError(f'{origin}: "profile" must be 1 (the format version), got {tree["profile"]!r}')
    return values


def add_cli_options(parser) -> None:
    """Add one command-line flag per option that has one (the help text comes from the table)."""
    for opt in OPTIONS:
        if not opt.cli:
            continue
        kwargs: Dict[str, Any] = {'dest': 'opt_' + opt.key, 'default': None, 'metavar': opt.kind.upper().replace('OPT_', '')}
        if opt.choices:
            kwargs['choices'] = opt.choices if opt.path != 'log.level' else tuple(c.lower() for c in opt.choices) + opt.choices
            kwargs['metavar'] = '|'.join(opt.choices)
        parser.add_argument(opt.cli, help=f'{opt.doc} (default: {_show_default(opt)})', **kwargs)


def _show_default(opt: Opt) -> str:
    return 'not set' if opt.default in (None, '') else str(opt.default)


def values_from_cli(namespace) -> Dict[str, Any]:
    """{option key: converted value} for the flags that were actually given on the command line."""
    values: Dict[str, Any] = {}
    for opt in OPTIONS:
        if opt.cli and getattr(namespace, 'opt_' + opt.key, None) is not None:
            values[opt.key] = coerce(opt, getattr(namespace, 'opt_' + opt.key))
    return values


def resolve_config(*, yaml_path: str = '', cli: Optional[Mapping[str, Any]] = None,
                   env: Optional[Mapping[str, str]] = None) -> BridgeConfig:
    """Defaults < YAML profile < environment (the endpoint) < command line; then cross-checks between settings."""
    values: Dict[str, Any] = {}
    if yaml_path:
        values.update(values_from_yaml(read_yaml_file(yaml_path, 'profile'), f'the profile "{yaml_path}"'))
    environment = os.environ if env is None else env
    if environment.get(ENV_ENDPOINT) and not (cli or {}).get('rest_endpoint'):
        values['rest_endpoint'] = coerce(_BY_PATH['rest.endpoint'], environment[ENV_ENDPOINT])
    values.update(cli or {})
    cfg = BridgeConfig(values)
    check_config(cfg)
    return cfg


def check_config(cfg: BridgeConfig) -> None:
    """Cross-checks that need two settings at once. Raises ConfigError."""
    if cfg.drone_offline_after_s < cfg.drone_stale_after_s:
        raise ConfigError('"drone.offline_after_s" must not be smaller than "drone.stale_after_s"')
    if cfg.mission_id:
        try:
            check_mission_id(cfg.mission_id)
        except ValueError as exc:
            raise ConfigError(f'"mission.id": {exc}') from None
    if cfg.source == 'replay' and not cfg.replay_run_dir:
        raise ConfigError('--source replay needs a run folder: give --run-dir PATH (or "replay.run_dir")')
    if cfg.sink == 'rest' and cfg.source == 'replay' and cfg.replay_speed != 1.0:
        raise ConfigError('a replay into Lattice (--sink rest) must run in real time (--speed 1): Lattice refuses '
                          'positions time-stamped ahead of its own clock, and a faster replay runs ahead of it. '
                          'Use --sink jsonl or --sink mock for a fast replay.')
    if cfg.sink == 'rest':
        endpoint = cfg.rest_endpoint
        if endpoint:
            normalise_endpoint(endpoint)
    longest = cfg.mission_duration_s + cfg.mission_target_margin_s
    if longest + 3600 >= C.MAX_EXPIRY.total_seconds() or cfg.areas_ttl_s + 3600 >= C.MAX_EXPIRY.total_seconds():
        raise ConfigError('the mission duration plus margin (or the area lifetime) is too long: Lattice entities '
                          f'may live less than {C.LATTICE_MAX_EXPIRY.days} days')


def normalise_endpoint(value: str) -> str:
    """'host', 'host:port' or 'http(s)://host[:port]' -> 'https://host[:port]' / the URL given. ConfigError if unusable.

    A scheme-less address means https (that is how the Lattice quickstart builds its URL). Credentials inside the
    address (user:password@host) are refused: a secret must never be part of a setting that gets printed.
    """
    text = value.strip()
    if not text:
        raise ConfigError('the Lattice endpoint is empty')
    parsed = urlparse(text if '://' in text else 'https://' + text)
    if parsed.scheme not in ('http', 'https') or not parsed.hostname:
        raise ConfigError(f'the Lattice endpoint "{_safe(text)}" is not a host name or an http(s) address')
    if parsed.username or parsed.password:
        raise ConfigError('the Lattice endpoint must not contain a user name or password; use the token variables')
    if parsed.path not in ('', '/') or parsed.query or parsed.fragment:
        raise ConfigError('the Lattice endpoint must be only a host (and port), without a path')
    try:
        port = parsed.port
    except ValueError:
        raise ConfigError('the Lattice endpoint has an invalid port number') from None
    host = parsed.hostname if ':' not in parsed.hostname else f'[{parsed.hostname}]'
    return f'{parsed.scheme}://{host}' + (f':{port}' if port else '')


def _safe(text: str) -> str:
    return text if len(text) <= 60 else text[:57] + '...'


# ===================================================================================================
# Secrets
# ===================================================================================================

_SECRET_LINE = re.compile(r'\s*(?:export\s+)?(?P<key>[A-Z_]+)\s*=\s*(?P<value>.*?)\s*$')


@dataclass(frozen=True)
class Secrets:
    """The endpoint and tokens. repr()/str() show only whether each is set, never the value."""
    endpoint: str = ''
    token: str = ''
    sandbox_token: str = ''
    source: str = 'environment'

    def __repr__(self) -> str:
        return (f'Secrets(endpoint={"set" if self.endpoint else "not set"}, token={"set" if self.token else "not set"}, '
                f'sandbox_token={"set" if self.sandbox_token else "not set"})')

    __str__ = __repr__

    def values(self) -> List[str]:
        """The secret strings that must never appear in a log (only real tokens; an endpoint is not secret)."""
        return [v for v in (self.token, self.sandbox_token) if v]


def _inside_git_repository(path: Path) -> bool:
    for parent in [path.parent] + list(path.parent.parents):
        if (parent / '.git').exists():
            return True
    return False


def load_secrets(env: Optional[Mapping[str, str]] = None, secrets_file: str = '') -> Secrets:
    """Read the secrets from the environment, then fill what is missing from `secrets_file`.

    The file holds lines like `LATTICE_TOKEN=<the token>` (a leading `export ` and quotes are accepted, `#` starts a
    comment). It is refused when it sits inside a git repository (so it can never be committed by accident) and a
    warning-worthy mode (readable by others) is reported through the returned object's `source` text.
    """
    environment = os.environ if env is None else env
    found = {name: str(environment.get(name, '')).strip() for name in SECRET_NAMES}
    source = 'environment'
    if secrets_file:
        path = Path(secrets_file).expanduser()
        try:
            resolved = path.resolve()
            info = resolved.stat()
        except OSError as exc:
            raise ConfigError(f'cannot read the secrets file "{secrets_file}": {exc.strerror or exc}') from None
        if _inside_git_repository(resolved):
            raise ConfigError(f'the secrets file "{secrets_file}" is inside a git repository, where it could be '
                              f'committed by accident. Move it outside (for example ~/.config/astra/lattice.env).')
        if os.name == 'posix' and info.st_mode & (stat.S_IRGRP | stat.S_IROTH):
            raise ConfigError(f'the secrets file "{secrets_file}" can be read by other users; run '
                              f'"chmod 600 {secrets_file}" first')
        try:
            lines = resolved.read_text(encoding='utf-8').splitlines()
        except (OSError, UnicodeError) as exc:
            raise ConfigError(f'cannot read the secrets file "{secrets_file}": {exc}') from None
        for line in lines:
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            match = _SECRET_LINE.fullmatch(line)
            if match and match.group('key') in SECRET_NAMES and not found[match.group('key')]:
                value = match.group('value')
                if len(value) >= 2 and value[0] == value[-1] and value[0] in '"\'':
                    value = value[1:-1]
                found[match.group('key')] = value
        source = 'environment + secrets file'
    return Secrets(found[ENV_ENDPOINT], found[ENV_TOKEN], found[ENV_SANDBOX_TOKEN], source)


def redact(text: str, secrets: Secrets) -> str:
    """Replace every secret value inside `text` by '***' (for log lines and error messages)."""
    for value in secrets.values():
        if len(value) >= 6:
            text = text.replace(value, '***')
    return text


# ===================================================================================================
# From settings to the objects the bridge needs
# ===================================================================================================

def load_areas(config_path: str, truth_path: str = '') -> List[AreaSpec]:
    """The search areas (and the flight boundary, if its file is found) described by a sim_worlds world.

    `config_path` is config/<world>.yaml (search areas as rectangles around home). The flight boundary lives in
    truth/<world>.yaml: `truth_path`, or the file the config names under `truth:` (looked up next to the config
    folder). Only `flight_boundary` and `altitude_rule_agl_m` are taken from the truth file: the target
    positions in it are the answer key and must never reach Lattice.
    """
    config = read_yaml_file(config_path, 'area config')
    try:
        specs = list(search_areas(config))
    except ValueError as exc:
        raise ConfigError(f'the area config "{config_path}" is not usable: {exc}') from None
    truth_file = truth_path
    if not truth_file and isinstance(config.get('truth'), str):
        base = Path(config_path).expanduser().resolve().parent
        for candidate in (base.parent / config['truth'], base / config['truth']):
            if candidate.is_file():
                truth_file = str(candidate)
                break
    if truth_file:
        truth = read_yaml_file(truth_file, 'truth')
        try:
            boundary = flight_boundary({k: truth[k] for k in ('flight_boundary', 'altitude_rule_agl_m') if k in truth})
        except ValueError as exc:
            raise ConfigError(f'the flight boundary in "{truth_file}" is not usable: {exc}') from None
        if boundary is not None:
            specs.insert(0, boundary)
    if not specs:
        raise ConfigError(f'the area config "{config_path}" holds no search areas')
    return specs


def resolve_mission_id(cfg: BridgeConfig, start_time: datetime, suggested_label: Optional[str] = None) -> str:
    """The mission id: the configured one, or a fresh one made from the start time and a label."""
    if cfg.mission_id:
        return check_mission_id(cfg.mission_id)
    return make_mission_id(start_time, cfg.mission_label or suggested_label or None)


def core_settings(cfg: BridgeConfig, mission_id: str, areas: Sequence[AreaSpec] = ()) -> CoreSettings:
    """The CoreSettings for BridgeCore, from a BridgeConfig."""
    return CoreSettings(
        mission_id=mission_id, geoid_undulation_m=cfg.altitude_geoid_undulation_m,
        drone_interval_s=cfg.drone_interval_s, stale_after_s=cfg.drone_stale_after_s,
        stale_policy=cfg.drone_stale_policy, offline_after_s=cfg.drone_offline_after_s,
        mission_duration_s=cfg.mission_duration_s, target_margin_s=cfg.mission_target_margin_s,
        area_ttl_s=cfg.areas_ttl_s, drop_ttl_s=cfg.drops_ttl_s, min_confidence=cfg.targets_min_confidence,
        merge_radius_m=cfg.targets_merge_radius_m, update_min_move_m=cfg.targets_update_min_move_m,
        update_min_conf_gain=cfg.targets_update_min_conf_gain, max_clusters=cfg.targets_max_clusters,
        uav_name=cfg.drone_name, areas=tuple(areas))


@dataclass
class Plan:
    """What a dry run prints: the lines of the plan and whether everything needed is available."""
    lines: List[str] = field(default_factory=list)
    problems: List[str] = field(default_factory=list)

    def text(self) -> str:
        """The plan as one string."""
        return '\n'.join(self.lines)
