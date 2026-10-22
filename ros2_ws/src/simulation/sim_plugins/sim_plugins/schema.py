"""
sim_plugins/schema.py -- PURPOSE: every key a profile may contain, with its default value, its allowed
values and one line of help. ONE table drives everything: the validator (profiles.py), the built-in
defaults (= exactly today's simulator behaviour), and the profile reference in docs/simulation/PLUGINS.md.

A profile is a YAML file with these sections (a profile only needs the keys it changes):

    world          which world to fly in (+ optionally another survey mission)
    autopilot      which ArduCopter: an installed binary / a version / a source tree / an external one
    camera         siyi_emulator (zr10 or a8mini profile), your own camera program, or none
    detector       team (new_od), truth (answer-key stand-in) or custom (your detector)
    mission_brain  team (main_controller) or custom (your mission logic)
    extras         a list of extra launch files / nodes / programs to start
    network        ports and addresses (MAVProxy outputs, camera address; Docker setups)
    run            demo.sh settings: headless, timeout, where the team stack runs

The DEFAULTS built from this table must stay EXACTLY today's behaviour; test_profiles.py checks that
simulation/profiles/default.yaml says the same thing.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from typing import Any, Dict, Tuple


@dataclass(frozen=True)
class Field:
    """One profile key.

    Attributes:
      default   the value used when no profile sets the key (today's behaviour)
      kind      how the value is checked: str, bool, int, port, choice, zoom, path, paths, names,
                mavout, mavouts, mavurl, ip, hostport, version, entry, entries
      choices   allowed values for kind 'choice'
      aliases   other spellings that mean a choice (e.g. 'real' -> 'team'); compared lower-case
      minimum   smallest allowed number (kind int)
      maximum   largest allowed number (kind int)
      help      one plain-words line (shown in errors and in the docs)
    """
    default: Any
    kind: str
    choices: Tuple[str, ...] = ()
    aliases: Dict[str, str] = field(default_factory=dict)
    minimum: int = 0
    maximum: int = 1_000_000
    help: str = ''


#: The keys of a "custom component" entry (detector.custom, mission_brain.custom, camera.custom, extras[i]).
#: Exactly ONE of launch / executable / command says what to start.
ENTRY_KEYS: Dict[str, str] = {
    'name': 'a short label for messages (extras); default = the package or program name',
    'package': 'the ROS 2 package that holds the launch file or the executable',
    'launch': 'a launch file: a name inside <package>/share/<package>/launch/, or a path to the file',
    'executable': 'OR one node: `ros2 run <package> <executable>`',
    'command': 'OR any program, as a list: ["python3", "/home/me/logger.py", "--fast"] (no shell)',
    'args': 'a mapping: launch arguments (launch) or ROS parameters (executable)',
    'node_name': 'the ROS node name it creates (stack_check waits for it; check_slot inspects it)',
    'provides_health': 'detector only: true = it serves /sahi/get_health like new_od (stack_check waits '
                       'for "ModelLoaded: True")',
    'start_with': 'extras only: team_stack (default, starts with ./launcher.sh sim) or simulator '
                  '(starts with sim.launch.py)',
}

#: The full profile table. Nested dicts are sections; Field objects are keys.
SCHEMA: Dict[str, Any] = {
    'profile': Field(1, 'int', minimum=1, maximum=1, help='format version of the file (always 1)'),
    'name': Field('', 'str', help='a short name shown in the plan (default: the file name)'),
    'description': Field('', 'str', help='one sentence: what this profile is for'),
    'extends': Field('', 'str', help='start from another profile (a name in simulation/profiles or a path), '
                                     'then change only what this file says'),
    'world': {
        'name': Field('runway', 'str', help='runway | suas_field | suas_field_real (when built) | any name with '
                                            'sim_worlds/config/<name>.yaml | a path to a .sdf or a world .yaml'),
        'mission': Field('', 'path', help='another survey mission (.waypoints) instead of the world\'s own'),
    },
    'autopilot': {
        'kind': Field('sitl_binary', 'choice', choices=('sitl_binary', 'sitl_source', 'external'),
                      aliases={'binary': 'sitl_binary', 'sitl': 'sitl_binary', 'source': 'sitl_source',
                               'remote': 'external'},
                      help='sitl_binary = a prebuilt arducopter | sitl_source = an ArduPilot source tree | '
                           'external = an autopilot running elsewhere (another container or PC)'),
        'version': Field('', 'version', help='sitl_binary: "" = the installed default (~/sitl/arducopter, '
                                             '4.6.3); 4.7.1 / latest / ... = ~/sitl/versions/<v>/arducopter '
                                             '(get it with simulation/setup/fetch_sitl.sh <v>)'),
        'binary': Field('', 'path', help='sitl_binary: the path of an arducopter program (wins over version)'),
        'source_dir': Field('', 'path', help='sitl_source: an ArduPilot source tree, e.g. /opt/ardupilot (Docker '
                                             'sitl image) or ~/ardupilot; it must be built '
                                             '(./waf configure --board sitl && ./waf copter)'),
        'param_files': Field([], 'paths', help='extra parameter files layered on top (e.g. the real drone\'s '
                                               '.param saved from Mission Planner); hardware settings are '
                                               'filtered out (see param_filter)'),
        'param_filter': Field('safe', 'choice', choices=('safe', 'none'),
                              help='safe = drop hardware and airframe settings that would break the '
                                   'simulator | none = use every line (experts)'),
        'param_keep': Field([], 'names', help='parameter names to keep even though the safe filter would '
                                              'drop them, e.g. [ATC_RAT_RLL_P]'),
        'wipe': Field(False, 'bool', help='true = erase SITL\'s saved parameters and mission at every start'),
        'instance': Field(0, 'int', minimum=0, maximum=20,
                          help='SITL instance I: ports 5760+10*I (MAVLink) and 9002+10*I (Gazebo link)'),
        'external': {
            'mavlink': Field('', 'mavurl', help='external: where MAVProxy picks up the autopilot\'s MAVLink, '
                                                'e.g. tcp:192.168.1.50:5760 or udpin:0.0.0.0:14560'),
            'fdm_addr': Field('0.0.0.0', 'ip', help='external: the address the drone\'s ArduPilot plugin binds its '
                                                    'physics port (UDP 9002+10*I) on; 0.0.0.0 = every network, so a '
                                                    'SITL on another computer/container can reach it'),
            'sim_address': Field('', 'str', help='external: this computer\'s address as the remote SITL sees it '
                                                 '(only used to print the command for the other computer)'),
        },
    },
    'camera': {
        'kind': Field('siyi_emulator', 'choice', choices=('siyi_emulator', 'custom', 'none'),
                      aliases={'emulator': 'siyi_emulator', 'siyi': 'siyi_emulator', 'off': 'none'},
                      help='siyi_emulator = the SIYI camera emulator | custom = your own camera program | '
                           'none = no camera on the network'),
        'model': Field('zr10', 'choice', choices=('zr10', 'a8mini'),
                       aliases={'a8-mini': 'a8mini', 'a8_mini': 'a8mini', 'a8': 'a8mini', 'zr-10': 'zr10'},
                       help='zr10 = 2K photos 2560x1440 (last year\'s default) | a8mini = 4K photos 3840x2160'),
        'zoom': Field('survey', 'zoom', help='survey = photos keep the full 81 degree view (max zoom 1x) | '
                                             'team = video_cam\'s 5x zoom rack works | a number = max zoom'),
        'strict': Field(False, 'bool', help='true = the emulator stays silent on commands a real A8 mini '
                                            'ignores (0x04 0x06 0x16 0x18 0x19); false = answers everything'),
        'custom': Field(None, 'entry', help='camera.kind custom: what to start instead of siyi_emulator'),
    },
    'detector': {
        'kind': Field('team', 'choice', choices=('team', 'truth', 'custom'),
                      aliases={'real': 'team', 'new_od': 'team', 'answer_key': 'truth', 'stand-in': 'truth'},
                      help='team = detection/new_od (YOLO) | truth = sim_demo truth_detector (answer key) | '
                           'custom = your detector (replaces new_od)'),
        'model_path': Field('', 'path', help='team: the YOLO weights ("" = ~/.astra_sim/models/yolo26m.pt or '
                                             '$ASTRA_DETECTOR_MODEL)'),
        'batched': Field(False, 'bool', help='team: true = the team\'s batched inference path (needs a TensorRT '
                                             'engine); false = SAHI path (works with .pt)'),
        'custom': Field(None, 'entry', help='detector.kind custom: your detector (see ENTRY_KEYS)'),
    },
    'mission_brain': {
        'kind': Field('team', 'choice', choices=('team', 'custom'),
                      aliases={'main_controller': 'team', 'real': 'team'},
                      help='team = main/main_controller.py | custom = your mission logic (replaces it)'),
        'custom': Field(None, 'entry', help='mission_brain.kind custom: your node (see ENTRY_KEYS)'),
    },
    'extras': Field([], 'entries', help='extra things to start: a list of entries (see ENTRY_KEYS)'),
    'network': {
        'gcs_port': Field(14550, 'port', help='MAVProxy UDP output on 127.0.0.1 for a ground station / our scripts'),
        'mavros_port': Field(14552, 'port', help='MAVProxy UDP output for MAVROS (./launcher.sh sim)'),
        'panel_port': Field(14553, 'port', help='MAVProxy UDP output for the demo panel\'s mission buttons'),
        'gcs_out': Field('', 'mavout', help='"" = udp:127.0.0.1:<gcs_port>; or another MAVProxy output. Careful: '
                                            'demo.sh / upload_mission.sh / fly_mission.sh listen on '
                                            'udp 127.0.0.1:<gcs_port>; to ADD a ground station use extra_outs'),
        'mavros_out': Field('', 'mavout', help='"" = udp:127.0.0.1:<mavros_port>; e.g. udp:172.18.0.3:14552 = a '
                                               'MAVROS in another container'),
        'extra_outs': Field([], 'mavouts', help='more MAVProxy outputs, e.g. [udp:192.168.1.20:14550]'),
        'windows_gcs': Field(True, 'bool', help='inside WSL also send MAVLink to Mission Planner on Windows'),
        'windows_gcs_addr': Field('', 'hostport', help='"" = automatic; HOST or HOST:PORT = send it there'),
        'camera_ip': Field('192.168.144.25', 'ip', help='the address the camera emulator pretends to be'),
        'camera_bind': Field('', 'ip', help='"" = camera_ip if this computer has it, else 127.0.0.1; '
                                            '0.0.0.0 = listen on every address (containers)'),
        'fcu_url': Field('', 'str', help='"" = udp://:<mavros_port>@localhost:<mavros_port>; MAVROS\'s '
                                         'fcu_url for ./launcher.sh sim (demo.sh)'),
    },
    'run': {
        'headless': Field(False, 'bool', help='true = no windows (Gazebo, RViz, demo panel)'),
        'timeout_min': Field(30, 'int', minimum=1, maximum=24 * 60, help='demo.sh gives up after this long'),
        'team_stack': Field('local', 'choice', choices=('local', 'external'),
                            help='local = demo.sh starts ./launcher.sh sim | external = the team stack runs '
                                 'somewhere else (another container/PC); demo.sh only waits for it'),
    },
}

#: The top-level keys that are sections (mappings of Fields).
SECTIONS = tuple(k for k, v in SCHEMA.items() if isinstance(v, dict))


def _defaults_of(table: Dict[str, Any]) -> Dict[str, Any]:
    """The nested default dict of a (sub)table."""
    out: Dict[str, Any] = {}
    for key, spec in table.items():
        out[key] = _defaults_of(spec) if isinstance(spec, dict) else copy.deepcopy(spec.default)
    return out


def defaults() -> Dict[str, Any]:
    """A fresh copy of the built-in defaults: exactly how the simulator behaves without any profile."""
    return _defaults_of(SCHEMA)


def lookup(dotted: str):
    """The Field (or section dict) at a dotted key like 'detector.kind'; raises KeyError if unknown."""
    node: Any = SCHEMA
    for part in dotted.split('.'):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(dotted)
        node = node[part]
    return node


def all_keys(table: Dict[str, Any] = None, prefix: str = ''):
    """Every dotted key with its Field, in table order (used by the docs and the error suggestions)."""
    table = SCHEMA if table is None else table
    for key, spec in table.items():
        dotted = f'{prefix}{key}'
        if isinstance(spec, dict):
            yield from all_keys(spec, dotted + '.')
        else:
            yield dotted, spec
