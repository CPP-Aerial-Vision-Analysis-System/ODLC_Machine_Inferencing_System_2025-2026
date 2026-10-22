"""
sim_plugins/profiles.py -- PURPOSE: read a plug-in profile (a small YAML file), check every value, and
produce the complete settings the simulator will use. Mistakes are reported in plain words, all at once,
with a hint how to fix each one (and "did you mean ...?" for typos).

How a profile becomes settings (later wins):
    1. the built-in defaults (schema.defaults() = exactly today's simulator),
    2. the profile it `extends:` (recursively, e.g. default),
    3. the profile file itself,
    4. command-line overrides  `--set detector.kind=truth`  (demo.sh / start_sim.sh flags become these).
Relative paths inside a profile are relative to THAT profile's folder, so a profile can sit next to its
parameter file and say `param_files: [my_drone.param]`. Paths given on the command line are relative to
the current folder.

Main entry point:  resolve(profile='example_detector', sets=['camera.model=a8mini'])  -> ResolvedProfile
A profile is found by name in simulation/profiles/ (paths.profiles_dir()) or by path.
"""

from __future__ import annotations

import copy
import difflib
import ipaddress
import os
import posixpath
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from . import paths, schema
from .schema import ENTRY_KEYS, SCHEMA, Field

#: MAVProxy output kinds (`--out`), and the master kinds (`--master`).
MAVOUT_KINDS = ('udp', 'udpout', 'udpbcast', 'tcp', 'tcpin')
MAVURL_KINDS = ('tcp', 'tcpin', 'udp', 'udpin', 'udpout', 'udpbcast')
VERSION_RE = re.compile(r'^[0-9A-Za-z][0-9A-Za-z._-]{0,31}$')
PARAM_NAME_RE = re.compile(r'^[A-Z][A-Z0-9_]{0,15}$')


# ---------------------------------------------------------------------------------------------
# Problems
# ---------------------------------------------------------------------------------------------

@dataclass
class Problem:
    """One thing wrong (or worth a warning) in a profile.

    where     the key, e.g. 'detector.kind' or 'extras[1].launch'
    message   what is wrong, in plain words
    hint      how to fix it ('' = no hint)
    severity  'error' (stops the start) or 'warning' (printed, the start continues)
    source    the file it came from ('' = command line or merged result)
    """
    where: str
    message: str
    hint: str = ''
    severity: str = 'error'
    source: str = ''

    def text(self) -> str:
        """One or two printable lines."""
        origin = f' ({Path(self.source).name})' if self.source else ''
        line = f'{self.where}{origin}: {self.message}'
        return line + (f'\n      -> {self.hint}' if self.hint else '')


class ProfileError(Exception):
    """A profile could not be used. `.problems` lists every error (and the warnings found so far)."""

    def __init__(self, problems: Sequence[Problem]):
        self.problems = list(problems)
        errors = [p for p in self.problems if p.severity == 'error']
        super().__init__(f'{len(errors)} problem(s) in the profile')

    def __str__(self) -> str:
        errors = [p for p in self.problems if p.severity == 'error']
        lines = [f'{len(errors)} problem(s) in the profile:']
        lines += ['  - ' + p.text() for p in errors]
        warns = [p for p in self.problems if p.severity != 'error']
        if warns:
            lines.append('and warning(s):')
            lines += ['  - ' + p.text() for p in warns]
        return '\n'.join(lines)


def _did_you_mean(word: str, options: Iterable[str]) -> str:
    """'did you mean X?' for the closest option, or ''."""
    close = difflib.get_close_matches(str(word), list(options), n=1, cutoff=0.6)
    return f'did you mean "{close[0]}"?' if close else ''


# ---------------------------------------------------------------------------------------------
# The resolved result
# ---------------------------------------------------------------------------------------------

@dataclass
class ResolvedProfile:
    """The complete settings after defaults + extends + file + overrides, every value checked.

    data      the nested settings dict (same shape as schema.defaults())
    name      the profile's name (its `name:` or the file name; '(built-in defaults)' without a file)
    sources   the files read, base first (empty without a profile)
    warnings  things worth telling the user (the start continues)
    sets      the command-line overrides that were applied ("detector.kind=truth", ...)
    """
    data: Dict[str, Any]
    name: str = '(built-in defaults)'
    sources: List[str] = field(default_factory=list)
    warnings: List[Problem] = field(default_factory=list)
    sets: List[str] = field(default_factory=list)

    def get(self, dotted: str, default: Any = None) -> Any:
        """The value at a dotted key like 'detector.kind' (default when missing)."""
        node: Any = self.data
        for part in dotted.split('.'):
            if not isinstance(node, dict) or part not in node:
                return default
            node = node[part]
        return node

    @property
    def source(self) -> str:
        """The profile file that was asked for ('' for the built-in defaults)."""
        return self.sources[-1] if self.sources else ''

    def is_default(self) -> bool:
        """True when the settings are exactly the built-in defaults (today's behaviour)."""
        return without_meta(self.data) == without_meta(schema.defaults())


def without_meta(data: Dict[str, Any]) -> Dict[str, Any]:
    """A copy without the descriptive keys (profile, name, description, extends): only what changes behaviour."""
    out = copy.deepcopy(data)
    for key in ('profile', 'name', 'description', 'extends'):
        out.pop(key, None)
    return out


# ---------------------------------------------------------------------------------------------
# Finding and reading files
# ---------------------------------------------------------------------------------------------

def find_profile(ref: str, base_dir: Optional[Path] = None, profiles_dir: Optional[Path] = None) -> Path:
    """Turn a profile reference into a file path.

    ref: a path (absolute, ~, or relative to base_dir / the current folder) or a NAME of a file in the
    profiles folder ('truth_detector' or 'truth_detector.yaml'). Raises ProfileError when nothing matches.
    """
    ref = str(ref).strip()
    if not ref:
        raise ProfileError([Problem('profile', 'empty profile name')])
    pdir = Path(profiles_dir) if profiles_dir else paths.profiles_dir()
    expanded = Path(paths.expand(ref))
    candidates: List[Path] = []
    if expanded.is_absolute():
        candidates.append(expanded)
    else:
        if base_dir is not None:
            candidates.append(Path(base_dir) / expanded)
        candidates.append(Path.cwd() / expanded)
        candidates.append(pdir / expanded)
    if not ref.endswith(('.yaml', '.yml')):
        candidates += [c.with_name(c.name + '.yaml') for c in list(candidates)]
    for cand in candidates:
        if cand.is_file():
            return cand.resolve()
    known = sorted(p.stem for p in pdir.glob('*.yaml')) if pdir.is_dir() else []
    hint = _did_you_mean(Path(ref).stem, known)
    hint = (hint + ' ' if hint else '') + (f'Profiles in {pdir}: {", ".join(known)}' if known
                                          else f'(no profiles found in {pdir})')
    raise ProfileError([Problem('profile', f'profile "{ref}" not found', hint)])


def load_yaml_file(path: Path) -> Dict[str, Any]:
    """Read one profile file; it must be a YAML mapping. Raises ProfileError with the YAML error in words."""
    try:
        import yaml
    except ImportError:                                   # pragma: no cover - PyYAML is a dependency
        raise ProfileError([Problem('profile', 'PyYAML is not installed', 'sudo apt install python3-yaml')])
    try:
        text = Path(path).read_text(encoding='utf-8')
    except OSError as exc:
        raise ProfileError([Problem('profile', f'cannot read {path}: {exc}')]) from None
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        mark = getattr(exc, 'problem_mark', None)
        where = f'line {mark.line + 1}, column {mark.column + 1}' if mark else 'somewhere'
        raise ProfileError([Problem('profile', f'{Path(path).name} is not valid YAML ({where}): '
                                    f'{getattr(exc, "problem", exc)}',
                                    'check the indentation (spaces, not tabs) and the colons',
                                    source=str(path))]) from None
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ProfileError([Problem('profile', f'{Path(path).name} must contain "key: value" lines (a mapping)',
                                    source=str(path))])
    return data


# ---------------------------------------------------------------------------------------------
# Checking one value
# ---------------------------------------------------------------------------------------------

def _as_bool(value: Any) -> Optional[bool]:
    """True/False from YAML booleans and the usual words; None when it is not a yes/no value."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        low = value.strip().lower()
        if low in ('true', 'yes', 'on', '1'):
            return True
        if low in ('false', 'no', 'off', '0'):
            return False
    return None


def _resolve_path(text: str, base_dir: Optional[Path]) -> str:
    """Expand ~ and $VAR; a relative path is taken relative to base_dir (the profile's folder).

    Linux paths (starting with /) are normalised with posixpath so they keep their forward slashes even
    when the unit tests run on another operating system.
    """
    text = str(text).strip()
    if not text:
        return ''
    p = paths.expand(text)
    if not p.startswith('/') and not os.path.isabs(p) and base_dir is not None:
        p = os.path.join(str(base_dir), p)
    return posixpath.normpath(p) if p.startswith('/') else os.path.normpath(p)


def _looks_like_path(text: str) -> bool:
    """True for things like ./x, ~/x, /x, $HOME/x, a/b (a file somewhere) - not for a bare file name."""
    return any(c in text for c in ('/', '\\')) or text.startswith(('~', '$', '.'))


def _check_host_port(text: str) -> Optional[str]:
    """None when 'HOST:PORT' is fine, else the reason."""
    host, sep, port = text.rpartition(':')
    if not sep or not host or ' ' in host:
        return 'needs HOST:PORT'
    try:
        num = int(port)
    except ValueError:
        return 'the port must be a whole number'
    if not 1 <= num <= 65535:
        return 'the port must be between 1 and 65535'
    return None


def _check_mavlink_address(text: str, kinds: Tuple[str, ...]) -> Optional[str]:
    """None when a MAVProxy address like udp:127.0.0.1:14552 is fine, else the reason."""
    if text.startswith('/dev/'):
        return None                                        # a serial device (rare, but MAVProxy takes it)
    kind, sep, rest = text.partition(':')
    if not sep or kind not in kinds:
        return f'must start with one of {", ".join(k + ":" for k in kinds)} (or be a /dev/... serial port)'
    why = _check_host_port(rest)
    return f'"{rest}" {why}' if why else None


def check_value(spec: Field, value: Any, where: str, base_dir: Optional[Path],
                problems: List[Problem], source: str = '') -> Any:
    """Check and normalise one value against its Field; append Problems; return the clean value."""
    def bad(message, hint=''):
        problems.append(Problem(where, message, hint or spec.help, source=source))
        return copy.deepcopy(spec.default)

    kind = spec.kind
    if value is None and kind in ('str', 'path', 'mavout', 'mavurl', 'ip', 'hostport', 'version'):
        return ''
    if value is None and kind in ('paths', 'mavouts', 'names', 'entries'):
        return []
    if value is None and kind == 'entry':
        return None
    if kind == 'str':
        if isinstance(value, (dict, list)):
            return bad('must be a single word or sentence, not a list or mapping')
        return str(value)
    if kind == 'bool':
        b = _as_bool(value)
        return bad(f'"{value}" is not true or false') if b is None else b
    if kind in ('int', 'port'):
        if isinstance(value, bool):
            return bad(f'{value} is not a number')
        try:
            num = int(str(value).strip())
        except ValueError:
            return bad(f'"{value}" is not a whole number')
        low, high = (1, 65535) if kind == 'port' else (spec.minimum, spec.maximum)
        if not low <= num <= high:
            return bad(f'{num} is outside {low}..{high}')
        return num
    if kind == 'choice':
        text = str(value).strip().lower()
        text = spec.aliases.get(text, text)
        if text in spec.choices:
            return text
        hint = _did_you_mean(text, list(spec.choices) + list(spec.aliases))
        return bad(f'"{value}" is not one of: {", ".join(spec.choices)}', hint or spec.help)
    if kind == 'zoom':
        text = str(value).strip().lower()
        if text in ('survey', 'team'):
            return text
        try:
            num = float(text.rstrip('x'))
        except ValueError:
            return bad(f'"{value}" is not survey, team or a number', spec.help)
        if not 0 < num <= 30:
            return bad(f'zoom {num:g} must be between 0 and 30')
        return num
    if kind == 'path':
        if isinstance(value, (dict, list)):
            return bad('must be one path')
        return _resolve_path(str(value), base_dir)
    if kind == 'paths':
        items = value if isinstance(value, list) else [s for s in str(value).split(',')]
        out = []
        for i, item in enumerate(items):
            if isinstance(item, (dict, list)) or item is None:
                problems.append(Problem(f'{where}[{i}]', 'must be a path', spec.help, source=source))
                continue
            if str(item).strip():
                out.append(_resolve_path(str(item), base_dir))
        return out
    if kind == 'names':
        items = value if isinstance(value, list) else [s for s in str(value).split(',')]
        out = []
        for item in items:
            name = str(item).strip().upper()
            if not name:
                continue
            if not PARAM_NAME_RE.match(name):
                problems.append(Problem(where, f'"{item}" is not an ArduPilot parameter name',
                                        'names are 1-16 characters: capital letters, digits and _',
                                        source=source))
                continue
            out.append(name)
        return out
    if kind in ('mavout', 'mavurl'):
        text = str(value).strip()
        if not text:
            return ''
        why = _check_mavlink_address(text, MAVOUT_KINDS if kind == 'mavout' else MAVURL_KINDS)
        return bad(f'"{text}" {why}', 'example: udp:127.0.0.1:14552') if why else text
    if kind == 'mavouts':
        items = value if isinstance(value, list) else [s for s in str(value).split(',')]
        out = []
        for i, item in enumerate(items):
            text = str(item).strip()
            if not text:
                continue
            why = _check_mavlink_address(text, MAVOUT_KINDS)
            if why:
                problems.append(Problem(f'{where}[{i}]', f'"{text}" {why}', 'example: udp:192.168.1.20:14550',
                                        source=source))
            else:
                out.append(text)
        return out
    if kind == 'ip':
        text = str(value).strip()
        if not text:
            return ''
        try:
            ipaddress.IPv4Address(text)
        except ValueError:
            return bad(f'"{text}" is not an IPv4 address like 192.168.144.25 (or 0.0.0.0)')
        return text
    if kind == 'hostport':
        text = str(value).strip()
        if not text:
            return ''
        host, sep, port = text.partition(':')
        if not host or ' ' in host or (sep and _check_host_port(text)):
            return bad(f'"{text}" is not HOST or HOST:PORT', 'example: 192.168.1.20:14550')
        return text
    if kind == 'version':
        text = str(value).strip()
        if not text:
            return ''
        for prefix in ('Copter-', 'copter-', 'ArduCopter-', 'V', 'v'):
            if text.startswith(prefix) and len(text) > len(prefix) and text[len(prefix)].isdigit():
                text = text[len(prefix):]
                break
        if not VERSION_RE.match(text):
            return bad(f'"{value}" is not a version like 4.7.1, latest, stable or beta')
        return text
    if kind == 'entry':
        return check_entry(value, where, base_dir, problems, source)
    if kind == 'entries':
        if not isinstance(value, list):
            return bad('must be a list of entries (each starts with "- ")')
        return [e for e in (check_entry(v, f'{where}[{i}]', base_dir, problems, source, extras=True)
                            for i, v in enumerate(value)) if e is not None]
    raise ValueError(f'unknown schema kind {kind}')          # pragma: no cover - programming error


def check_entry(value: Any, where: str, base_dir: Optional[Path], problems: List[Problem],
                source: str = '', extras: bool = False) -> Optional[Dict[str, Any]]:
    """Check one custom-component entry (see schema.ENTRY_KEYS); returns the clean entry or None."""
    if value is None:
        return None
    if not isinstance(value, dict):
        problems.append(Problem(where, 'must be a mapping with package / launch / executable / command / args',
                                source=source))
        return None
    entry: Dict[str, Any] = {'name': '', 'package': '', 'launch': '', 'executable': '', 'command': [],
                             'args': {}, 'node_name': '', 'provides_health': False,
                             'start_with': 'team_stack'}
    for key, raw in value.items():
        if key not in ENTRY_KEYS:
            problems.append(Problem(f'{where}.{key}', 'unknown key', _did_you_mean(key, ENTRY_KEYS) or
                                    'allowed: ' + ', '.join(ENTRY_KEYS), source=source))
            continue
        if key in ('name', 'package', 'executable', 'node_name'):
            entry[key] = '' if raw is None else str(raw).strip()
        elif key == 'launch':
            text = '' if raw is None else str(raw).strip()
            entry[key] = _resolve_path(text, base_dir) if text and _looks_like_path(text) else text
        elif key == 'command':
            items = raw if isinstance(raw, list) else ([] if raw is None else [raw])
            cmd = [str(c) for c in items]
            if cmd and _looks_like_path(cmd[0]) and not cmd[0].startswith('/'):
                cmd[0] = _resolve_path(cmd[0], base_dir)
            entry[key] = cmd
        elif key == 'args':
            if raw is None:
                raw = {}
            if not isinstance(raw, dict):
                problems.append(Problem(f'{where}.args', 'must be a mapping (name: value)', source=source))
                continue
            entry[key] = {str(k): v for k, v in raw.items()}
        elif key == 'provides_health':
            b = _as_bool(raw)
            if b is None:
                problems.append(Problem(f'{where}.provides_health', f'"{raw}" is not true or false',
                                        source=source))
            else:
                entry[key] = b
        elif key == 'start_with':
            text = str(raw).strip().lower()
            if text not in ('team_stack', 'simulator'):
                problems.append(Problem(f'{where}.start_with', f'"{raw}" is not team_stack or simulator',
                                        source=source))
            else:
                entry[key] = text
    starts = [k for k in ('launch', 'executable', 'command') if entry[k]]
    if len(starts) != 1:
        problems.append(Problem(where, 'give exactly ONE of launch, executable or command '
                                       f'(found {", ".join(starts) or "none"})',
                                'launch: my_file.launch.py (with package), executable: my_node (with package), '
                                'or command: [python3, /path/to/script.py]', source=source))
    if entry['executable'] and not entry['package']:
        problems.append(Problem(f'{where}.package', 'an executable needs its package',
                                'ros2 run <package> <executable>', source=source))
    if entry['launch'] and not entry['package'] and not _looks_like_path(entry['launch']):
        problems.append(Problem(f'{where}.package', f'the launch file "{entry["launch"]}" needs its package '
                                '(or give a path to the file)', source=source))
    if not extras and entry['start_with'] != 'team_stack':
        problems.append(Problem(f'{where}.start_with', 'only extras have start_with', severity='warning',
                                source=source))
    if not entry['name']:
        entry['name'] = entry['package'] or (Path(entry['command'][-1]).stem if entry['command'] else 'extra')
    return entry


# ---------------------------------------------------------------------------------------------
# Checking a whole (partial) profile
# ---------------------------------------------------------------------------------------------

def check_tree(raw: Dict[str, Any], table: Dict[str, Any], prefix: str, base_dir: Optional[Path],
               problems: List[Problem], source: str = '') -> Dict[str, Any]:
    """Check a (partial) profile mapping against a schema (sub)table. Returns only the keys that were given,
    cleaned. Unknown keys become problems with a "did you mean" hint."""
    out: Dict[str, Any] = {}
    for key, value in raw.items():
        dotted = f'{prefix}{key}'
        if key not in table:
            options = list(table)
            problems.append(Problem(dotted, 'unknown key', _did_you_mean(key, options) or
                                    'allowed here: ' + ', '.join(options), source=source))
            continue
        spec = table[key]
        if isinstance(spec, dict):
            if dotted == 'world' and isinstance(value, str):
                value = {'name': value}                    # "world: runway" is short for world.name
            if value is None:
                continue
            if not isinstance(value, dict):
                problems.append(Problem(dotted, 'must be a section (indented "key: value" lines below it)',
                                        source=source))
                continue
            out[key] = check_tree(value, spec, dotted + '.', base_dir, problems, source)
        else:
            if dotted == 'world.name':
                text = '' if value is None else str(value).strip()
                looks = text.endswith(('.sdf', '.yaml', '.yml')) or _looks_like_path(text)
                out[key] = _resolve_path(text, base_dir) if looks else (text or 'runway')
                continue
            out[key] = check_value(spec, value, dotted, base_dir, problems, source)
    return out


def merge(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    """Deep merge: mappings merge key by key; anything else (lists too) is replaced by the overlay."""
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict) and key not in ('custom', 'args'):
            out[key] = merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def parse_set(text: str) -> Tuple[str, Any]:
    """Split a command-line override 'section.key=value' into (dotted key, value).

    The value stays TEXT (check_value turns "true" / "5" into the right type for its key), because YAML would
    turn some addresses and times into numbers ("10:30" is 630 in YAML 1.1). Only a value that starts with
    [ or { is read as YAML (a list or a mapping, e.g. --set 'extras=[...]'), and null / ~ mean "nothing".
    """
    key, sep, raw = str(text).partition('=')
    key = key.strip()
    if not sep or not key:
        raise ProfileError([Problem('--set', f'"{text}" is not KEY=VALUE', 'example: --set detector.kind=truth')])
    raw = raw.strip()
    if raw.lower() in ('null', '~'):
        return key, None
    if raw[:1] in ('[', '{'):
        return key, _yaml_value(raw)
    return key, raw


def _yaml_value(raw: str) -> Any:
    """A YAML value from text (the text itself when it is not valid YAML)."""
    import yaml
    try:
        return yaml.safe_load(raw) if raw else ''
    except yaml.YAMLError:
        return raw


def _nest(dotted: str, value: Any) -> Dict[str, Any]:
    """'a.b.c', 1 -> {'a': {'b': {'c': 1}}}."""
    out: Dict[str, Any] = {}
    node = out
    parts = dotted.split('.')
    for part in parts[:-1]:
        node[part] = {}
        node = node[part]
    node[parts[-1]] = value
    return out


def _overrides_tree(sets: Sequence[str], problems: List[Problem]
                    ) -> Tuple[Dict[str, Any], List[Tuple[str, List[str], Any]]]:
    """Turn ['detector.kind=truth', ...] into one checked partial profile (paths relative to the cwd).

    Returns (partial profile, entry_sets). entry_sets are overrides INSIDE a custom entry, like
    'detector.custom.args.threshold=0.5': (slot, ['args', 'threshold'], 0.5); resolve() applies them to
    the entry the profile already has.
    """
    total: Dict[str, Any] = {}
    entry_sets: List[Tuple[str, List[str], Any]] = []
    for text in sets:
        try:
            dotted, value = parse_set(text)
        except ProfileError as exc:
            problems.extend(exc.problems)
            continue
        parts = dotted.split('.')
        if len(parts) >= 3 and parts[1] == 'custom' and parts[0] in ('detector', 'mission_brain', 'camera'):
            # Inside an entry (e.g. detector.custom.args.threshold=0.5) the value becomes a ROS parameter or
            # a launch argument: read it as YAML so 0.5 is a number and true a boolean.
            entry_sets.append((parts[0], parts[2:], _yaml_value(value) if isinstance(value, str) else value))
            continue
        total = merge(total, _nest(dotted, value))
    return check_tree(total, SCHEMA, '', Path.cwd(), problems, source=''), entry_sets


def cross_checks(data: Dict[str, Any], problems: List[Problem]) -> None:
    """Checks that need several keys together (run on the merged settings)."""
    for slot in ('detector', 'mission_brain', 'camera'):
        kind = data[slot]['kind']
        entry = data[slot].get('custom')
        if kind == 'custom' and not entry:
            problems.append(Problem(f'{slot}.custom', f'{slot}.kind is custom but there is no {slot}.custom entry',
                                    f'add {slot}.custom with package + launch (or executable / command)'))
        if kind != 'custom' and entry:
            problems.append(Problem(f'{slot}.custom', f'is ignored because {slot}.kind is "{kind}"',
                                    f'set {slot}.kind: custom to use it', severity='warning'))
    ap = data['autopilot']
    if ap['kind'] == 'sitl_source' and not ap['source_dir']:
        problems.append(Problem('autopilot.source_dir', 'autopilot.kind is sitl_source but source_dir is empty',
                                'e.g. source_dir: /opt/ardupilot'))
    if ap['kind'] == 'external' and not ap['external']['mavlink']:
        problems.append(Problem('autopilot.external.mavlink', 'autopilot.kind is external but its MAVLink '
                                'address is empty', 'e.g. mavlink: tcp:192.168.1.50:5760'))
    if ap['kind'] == 'sitl_binary' and ap['version'] and ap['binary']:
        problems.append(Problem('autopilot.binary', 'give version OR binary, not both',
                                'binary = one exact file; version = ~/sitl/versions/<version>/arducopter'))
    if ap['kind'] != 'sitl_binary' and (ap['version'] or ap['binary']):
        problems.append(Problem('autopilot.version', f'version/binary are ignored for autopilot.kind '
                                f'{ap["kind"]}', severity='warning'))
    if ap['kind'] == 'external' and (ap['param_files'] or ap['wipe']):
        problems.append(Problem('autopilot.param_files', 'param_files/wipe do nothing for an external autopilot '
                                '(set the parameters on that autopilot)', severity='warning'))
    net = data['network']
    ports = {'gcs_port': net['gcs_port'], 'mavros_port': net['mavros_port'], 'panel_port': net['panel_port']}
    if len(set(ports.values())) != len(ports):
        problems.append(Problem('network', f'gcs_port, mavros_port and panel_port must be different ({ports})'))
    if net['gcs_out'] and net['gcs_out'] != f'udp:127.0.0.1:{net["gcs_port"]}':
        problems.append(Problem('network.gcs_out', f'moves the output our scripts listen on (udp 127.0.0.1:'
                                f'{net["gcs_port"]}): demo.sh, upload_mission.sh and fly_mission.sh will not reach '
                                'the autopilot', 'to ADD a ground station somewhere else, use network.extra_outs',
                                severity='warning'))
    for slot, team_node in (('detector', 'new_od'), ('mission_brain', 'main_controller')):
        entry = data[slot].get('custom') or {}
        if data[slot]['kind'] == 'custom' and entry and not entry.get('node_name'):
            problems.append(Problem(f'{slot}.custom.node_name', f'not given: demo.sh cannot wait for your node '
                                    f'(it only stops waiting for {team_node})',
                                    'add node_name: <the ROS node name your program creates>', severity='warning'))
    model = data['detector']['model_path']
    if ' ' in model:
        problems.append(Problem('detector.model_path', f'"{model}" contains a space: ./launcher.sh cannot pass it '
                                'to the team stack', 'move the weights to a folder without spaces'))
    if data['run']['team_stack'] == 'external':
        team_side = [s for s in ('detector', 'mission_brain') if data[s]['kind'] == 'custom']
        team_side += [e['name'] for e in data['extras'] if e['start_with'] == 'team_stack']
        if team_side:
            problems.append(Problem('run.team_stack', 'the team stack runs elsewhere, so demo.sh does not start '
                                    f'{", ".join(team_side)}; start them with that team stack '
                                    '(team_stack.launch.py profile:=<this profile>)', severity='warning'))
    names = [e['name'] for e in data['extras']]
    for dup in sorted({n for n in names if names.count(n) > 1}):
        problems.append(Problem('extras', f'two extras are both called "{dup}"', 'give each a different name:',
                                severity='warning'))


def resolve(profile: Optional[str] = None, sets: Sequence[str] = (), profiles_dir: Optional[Path] = None,
            base_dir: Optional[Path] = None) -> ResolvedProfile:
    """Build the complete, checked settings (see the module docstring for the order).

    profile:      a profile name or path ('' / None = built-in defaults only)
    sets:         command-line overrides like 'camera.model=a8mini'
    profiles_dir: where profile names are looked up (default paths.profiles_dir())
    Raises ProfileError listing EVERY error; warnings are in the result's .warnings.
    """
    problems: List[Problem] = []
    data = schema.defaults()
    sources: List[str] = []
    name = '(built-in defaults)'
    if profile:
        chain: List[Tuple[Path, Dict[str, Any]]] = []
        path = find_profile(profile, base_dir, profiles_dir)
        while True:
            if any(p == path for p, _ in chain):
                cycle = ' -> '.join(p.name for p, _ in chain) + f' -> {path.name}'
                raise ProfileError([Problem('extends', f'profiles extend each other in a circle: {cycle}')])
            raw = load_yaml_file(path)
            chain.append((path, raw))
            parent = raw.get('extends')
            if not parent:
                break
            try:
                path = find_profile(str(parent), path.parent, profiles_dir)
            except ProfileError as exc:
                for p in exc.problems:
                    p.where, p.source = 'extends', str(chain[-1][0])
                raise
        for path, raw in reversed(chain):                     # base first
            clean = check_tree(raw, SCHEMA, '', path.parent, problems, source=str(path))
            data = merge(data, clean)
            sources.append(str(path))
        top_path, top_raw = chain[0]
        name = str(top_raw.get('name') or top_path.stem)
        data['name'] = name
    if sets:
        clean, entry_sets = _overrides_tree(sets, problems)
        data = merge(data, clean)
        for slot, parts, value in entry_sets:
            raw = copy.deepcopy(data[slot].get('custom') or {})
            node = raw
            for part in parts[:-1]:
                if not isinstance(node.get(part), dict):
                    node[part] = {}
                node = node[part]
            node[parts[-1]] = value
            data[slot]['custom'] = check_entry(raw, f'{slot}.custom', Path.cwd(), problems)
    cross_checks(data, problems)
    errors = [p for p in problems if p.severity == 'error']
    if errors:
        raise ProfileError(problems)
    return ResolvedProfile(data=data, name=name, sources=sources,
                           warnings=[p for p in problems if p.severity != 'error'], sets=list(sets))


def dump_resolved(rp: ResolvedProfile) -> str:
    """The resolved settings as a self-contained YAML text (no extends; absolute paths), for the launch files."""
    import yaml
    data = copy.deepcopy(rp.data)
    data['extends'] = ''
    header = ('# RESOLVED plug-in profile, written by sim_plugins (profile_tool) for the launch files.\n'
              f'# Made from: {", ".join(rp.sources) or "built-in defaults"}'
              + (f'; overrides: {" ".join(rp.sets)}' if rp.sets else '') + '\n'
              '# Do not edit: it is rewritten at every start. Edit the profile it was made from.\n')
    return header + yaml.safe_dump(data, sort_keys=False, default_flow_style=False)


def write_resolved(rp: ResolvedProfile, out_dir: Optional[Path] = None, stem: str = '') -> Path:
    """Write dump_resolved() to <generated>/profile_<stem>.yaml and return the path."""
    out_dir = Path(out_dir) if out_dir else paths.generated_dir()
    out_dir.mkdir(parents=True, exist_ok=True)
    safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', stem or rp.name or 'profile')
    out = out_dir / f'profile_{safe}.yaml'
    out.write_text(dump_resolved(rp), encoding='utf-8')
    return out


def list_profiles(profiles_dir: Optional[Path] = None) -> List[Tuple[str, str, str]]:
    """(name, description, problem) of every profile in the profiles folder ('' problem = it loads fine)."""
    pdir = Path(profiles_dir) if profiles_dir else paths.profiles_dir()
    out = []
    for path in sorted(pdir.glob('*.yaml')):
        try:
            rp = resolve(str(path), profiles_dir=pdir)
            out.append((path.stem, str(rp.get('description', '')), ''))
        except ProfileError as exc:
            out.append((path.stem, '', str(exc).splitlines()[0]))
    return out
