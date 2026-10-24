"""
sim_plugins/doc_blocks.py -- PURPOSE: write the GENERATED parts of docs/simulation/PLUGINS.md from the code, so the
page can never drift away from what the tools really do:

    reference   every profile key: its default (= today's simulator), the values it accepts and what it does
                (from schema.py), plus the keys of an "entry" (schema.ENTRY_KEYS)
    contracts   every slot's contract as tables: topics, services, files, ports, rules (from contracts.py)

In the Markdown file each part sits between two marker lines (an HTML comment, invisible on GitHub):

    <!-- BEGIN GENERATED reference: python3 -m sim_plugins.profile_tool docs --write -->
    ...
    <!-- END GENERATED reference -->

`profile_tool docs --write` rewrites only what is between the markers (everything else on the page is written
by hand and kept), `profile_tool docs --check` exits 1 when a part is out of date. test_doc_blocks.py runs the
check, so a change to schema.py or contracts.py without `docs --write` fails the tests (and says how to fix it).
"""

from __future__ import annotations

import re
from typing import Any, Dict, List, Sequence, Tuple

from . import contracts, schema

#: The generated parts, in the order they appear on the page.
BLOCKS = ('reference', 'contracts')

#: The slots in the order the contracts are shown (the ones teammates replace most often first).
CONTRACT_ORDER = ('detector', 'mission_brain', 'camera', 'autopilot', 'world', 'extras', 'network')

#: Plain words for each kind of value (schema.Field.kind).
KIND_WORDS = {
    'str': 'text',
    'bool': '`true` or `false`',
    'port': 'a port number 1..65535',
    'zoom': '`survey`, `team`, or a number (the largest zoom, above 0 and up to 30)',
    'path': 'a file path (a relative path starts at the profile\'s folder)',
    'paths': 'a list of file paths, e.g. `[my.param]` (relative = from the profile\'s folder)',
    'names': 'a list of ArduPilot parameter names, e.g. `[ATC_RAT_RLL_P]`',
    'mavout': 'a MAVProxy output: `udp:HOST:PORT` (or `udpout:`, `udpbcast:`, `tcp:`, `tcpin:`)',
    'mavouts': 'a list of MAVProxy outputs, e.g. `[udp:192.168.1.20:14550]`',
    'mavurl': 'a MAVLink address: `tcp:HOST:PORT`, `udpin:0.0.0.0:PORT`, ... (or a `/dev/...` serial port)',
    'ip': 'an IPv4 address (`0.0.0.0` = every address of this computer)',
    'hostport': '`HOST` or `HOST:PORT`',
    'version': 'a version name: `4.7.1`, `stable`, `beta`, `latest` ... (`Copter-4.7.1` and `V4.7.1` work too)',
    'entry': 'an entry: a mapping with the entry keys below',
    'entries': 'a list of entries (each starts with `- `)',
}


def begin_marker(name: str) -> str:
    """The line that opens a generated part."""
    return f'<!-- BEGIN GENERATED {name}: python3 -m sim_plugins.profile_tool docs --write -->'


def end_marker(name: str) -> str:
    """The line that closes a generated part."""
    return f'<!-- END GENERATED {name} -->'


def _esc(text: str) -> str:
    """Make text safe inside a Markdown table cell (| would start a new column)."""
    return str(text).replace('|', '\\|').replace('\n', ' ')


def render_value(value: Any) -> str:
    """A default value the way it is written in a profile (YAML)."""
    if value is None:
        return '`null`'
    if isinstance(value, bool):
        return f'`{"true" if value else "false"}`'
    if isinstance(value, (list, tuple)):
        return '`[' + ', '.join(str(v) for v in value) + ']`'
    if value == '':
        return '`""`'
    return f'`{value}`'


def accepts(field: schema.Field) -> str:
    """What values a key takes, in words."""
    if field.kind == 'choice':
        text = 'one of ' + ', '.join(f'`{c}`' for c in field.choices)
        if field.aliases:
            text += ' (also: ' + ', '.join(f'`{a}` = `{c}`' for a, c in field.aliases.items()) + ')'
        return text
    if field.kind == 'int':
        return f'a whole number {field.minimum}..{field.maximum}'
    return KIND_WORDS.get(field.kind, field.kind)


def reference_block() -> str:
    """Every profile key as tables, one table per section (the profile reference)."""
    out: List[str] = [
        'The **Default** column is today\'s simulator (`simulation/profiles/default.yaml` writes every key out).',
        'A profile only needs the keys it changes. The dotted name (for example `camera.model`) is what',
        '`--set KEY=VALUE` takes on the command line.',
        '',
    ]
    groups: Dict[str, List[Tuple[str, schema.Field]]] = {}
    for dotted, field in schema.all_keys():
        section = dotted.split('.')[0] if dotted.split('.')[0] in schema.SECTIONS else '(the file itself)'
        if dotted == 'extras':
            section = 'extras'
        groups.setdefault(section, []).append((dotted, field))
    order = ['(the file itself)'] + [s for s in schema.SCHEMA if s in groups and s != '(the file itself)']
    for section in order:
        title = 'The file itself' if section == '(the file itself)' else f'`{section}`'
        out += [f'#### {title}', '', '| Key | Default | Accepts | What it does |', '|---|---|---|---|']
        for dotted, field in groups[section]:
            out.append(f'| `{dotted}` | {_esc(render_value(field.default))} | {_esc(accepts(field))} | '
                       f'{_esc(field.help)} |')
        out.append('')
    out += ['#### Entry keys', '',
            'An **entry** says how to start one program: `detector.custom`, `mission_brain.custom`, `camera.custom`',
            'and every item of `extras`. Give exactly ONE of `launch`, `executable` or `command`.', '',
            '| Key | What it does |', '|---|---|']
    out += [f'| `{key}` | {_esc(text)} |' for key, text in schema.ENTRY_KEYS.items()]
    return '\n'.join(out)


def contracts_block() -> str:
    """Every slot's contract: an anchor (#detector, #camera ...), a heading and the tables of contracts.py."""
    out: List[str] = []
    for slot in CONTRACT_ORDER:
        c = contracts.contract(slot)
        out += [f'<a id="{slot}"></a>', '', f'### {c.title}', '', contracts.render_markdown(c).rstrip(), '']
    missing = [s for s in contracts.CONTRACTS if s not in CONTRACT_ORDER]
    for slot in missing:                                     # a new slot shows up even before it is ordered
        c = contracts.contract(slot)
        out += [f'<a id="{slot}"></a>', '', f'### {c.title}', '', contracts.render_markdown(c).rstrip(), '']
    return '\n'.join(out).rstrip()


GENERATORS = {'reference': reference_block, 'contracts': contracts_block}


def generate(name: str) -> str:
    """The body of one generated part (without its marker lines)."""
    return GENERATORS[name]()


def _pattern(name: str):
    return re.compile(re.escape(begin_marker(name)) + r'\n(.*?)\n?' + re.escape(end_marker(name)), re.S)


def replace_blocks(text: str, names: Sequence[str] = BLOCKS) -> Tuple[str, List[str], List[str]]:
    """Put the generated parts between their markers.

    Returns (new text, names of the parts that changed, names whose markers are missing). Text outside the
    markers is never changed.
    """
    changed, missing = [], []
    for name in names:
        pattern = _pattern(name)
        match = pattern.search(text)
        if not match:
            missing.append(name)
            continue
        body = generate(name)
        if match.group(1).strip('\n') != body:
            changed.append(name)
        block = f'{begin_marker(name)}\n{body}\n{end_marker(name)}'
        text = text[:match.start()] + block + text[match.end():]
    return text, changed, missing


def out_of_date(text: str, names: Sequence[str] = BLOCKS) -> List[str]:
    """Names of the generated parts that are missing or differ from what the code says now."""
    _, changed, missing = replace_blocks(text, names)
    return missing + changed
