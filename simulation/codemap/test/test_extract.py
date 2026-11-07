"""
simulation/codemap/test/test_extract.py -- PURPOSE: tests of simulation/codemap/extract.py, the program that reads
the whole repository and writes codemap.json (the map the code-map explorer draws).

What is tested, in plain words:
  * the small YAML reader used for systems.yaml (and that the real systems.yaml reads the same as PyYAML would);
  * Python: the block ids follow the contract (Class.method, outer.inner, never <locals>, decorators, a property's
    getter and setter merged into one block), and every trace_index line number is exactly the co_firstlineno
    Python itself gives the function (checked against real compiled code);
  * importable names and the colcon install paths in trace_index (py_modules, py_files, programs);
  * ROS entry points from setup.py and CMakeLists.txt; launch files (.launch.py and .launch.xml);
  * calls (same module, self.method, through an import, through an attribute that always holds one class,
    class creation) and what is NOT guessed (unknown objects, shadowed names);
  * ROS topics and services: literal names, constants, relative names, parameter defaults, message types, the
    publishing method, the subscription callback, and topic owners from systems.yaml;
  * bash (functions, here-documents, here-strings, line continuations, sourced helpers, scripts that run
    scripts, ros2 run / launch / topic pub, trap handlers), C# (classes, methods, overloads, properties) and
    Windows batch files;
  * packages and systems (flat and recursive folders, test folders, Python sub-packages, unassigned files);
  * explanations: merging, unmatched ids, broken files, docstring and test-name fall-backs, coverage numbers;
  * the output is deterministic and the --check option fails when it should;
  * a smoke test on the REAL repository (no duplicate ids, every file in a package and a system, key facts), and
    for every real Python file: each function's trace_index line compared with the code Python itself compiles.

Run (Windows or Ubuntu, Python 3.10+, no ROS), from the repo root:
    python -m pytest -q -p no:cacheprovider simulation/codemap/test/test_extract.py
"""

from __future__ import annotations

import json
import re
import textwrap
import time
import types
from pathlib import Path

import pytest

import extract

REPO = Path(__file__).resolve().parents[3]

# ---------------------------------------------------------------------------------------------------------
# helpers: a tiny pretend repository
# ---------------------------------------------------------------------------------------------------------

SYSTEMS = """
# test systems file
schema: 1
systems:
  - key: stack
    label: The stack
    summary: >-
      The pretend team stack. It has
      two lines.
    packages:
      - ros2_ws/src/*
      - scripts
      - tools/**
  - key: sim
    label: Simulator
    summary: The pretend simulator.
    packages:
      - simulation/scripts
      - simulation/windows/plugin
  - key: tests
    label: Tests
    packages:
      - "**/test"
  - key: other
    label: Other
    packages:
      - .
externals:
  - key: mavros
    label: MAVROS
    system: stack
    summary: The translator.
    ros_packages: [mavros]
    process_match: ['\\bmavros_node\\b']
  - key: gazebo
    label: Gazebo
    system: sim
    ros_packages: [ros_gz_sim]
    programs: [gz]
    launch_names: [gz_server]
topic_owners:
  - prefix: /mavros/
    owner: ext:mavros
  - prefix: /camera/
    parent: sys:sim
links:
  - from: ext:gazebo
    to: ext:mavros
    kind: data
    label: "a pretend link: UDP 1"
  - from: ext:gazebo
    to: topic:/sim/clock
    kind: publishes
"""

PKG_XML = """<?xml version="1.0"?>
<!-- {name}/package.xml PURPOSE: the pretend package {name}. -->
<package format="3"><name>{name}</name><description>Pretend {name}.</description>
<export><build_type>{build}</build_type></export></package>
"""


def write_files(root: Path, files: dict):
    for rel, text in files.items():
        p = root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(textwrap.dedent(text).lstrip('\n') if isinstance(text, str) else '', encoding='utf-8',
                     newline='\n')


def make_repo(tmp_path: Path, files: dict, systems: str = SYSTEMS, explanations: dict | None = None):
    repo = tmp_path / 'repo'
    write_files(repo, files)
    (tmp_path / 'systems.yaml').write_text(systems, encoding='utf-8')
    expl = tmp_path / 'expl'
    expl.mkdir(exist_ok=True)
    for name, data in (explanations or {}).items():
        (expl / name).write_text(data if isinstance(data, str) else json.dumps(data), encoding='utf-8')
    return repo


def build(tmp_path: Path, files: dict, systems: str = SYSTEMS, explanations: dict | None = None, **kw):
    repo = make_repo(tmp_path, files, systems, explanations)
    cm, report = extract.build_codemap(repo, tmp_path / 'systems.yaml', tmp_path / 'expl',
                                       generated='2026-01-01T00:00:00Z', commit='test', **kw)
    nodes = {n['id']: n for n in cm['nodes']}
    edges = {(e['from'], e['to'], e['kind']) for e in cm['edges']}
    labels = {(e['from'], e['to'], e['kind']): e.get('label', '') for e in cm['edges']}
    return types.SimpleNamespace(cm=cm, report=report, nodes=nodes, edges=edges, labels=labels, repo=repo)


def ros_python_pkg(name: str, modules: dict, console: list[str] = (), build_type: str = 'ament_python') -> dict:
    """Files of an ament_python package at ros2_ws/src/<name> with modules in <name>/<name>/."""
    base = f'ros2_ws/src/{name}'
    files = {f'{base}/package.xml': PKG_XML.format(name=name, build=build_type),
             f'{base}/{name}/__init__.py': '',
             f'{base}/setup.py': "from setuptools import setup\n"
                                 "setup(name=%r, entry_points={'console_scripts': [%s]})\n"
                                 % (name, ', '.join(repr(c) for c in console)),
             f'{base}/resource/{name}': ''}
    for rel, text in modules.items():
        files[f'{base}/{name}/{rel}'] = text
    return files


# ---------------------------------------------------------------------------------------------------------
# the YAML reader
# ---------------------------------------------------------------------------------------------------------

def test_yaml_subset_reads_maps_lists_quotes_flow_lists_and_folded_blocks():
    data = extract.parse_yaml_subset(textwrap.dedent("""
        # a comment
        a: plain text   # trailing comment
        b: "quoted: with colon"
        c: 'single ''quote'''
        d: [x, 'y z', "w"]
        e: []
        f: >-
          folded line one
          line two

          second paragraph
        g:
          - one
          - key: k1
            val: v1
          - "**/test"
        h:
          inner: 1
        .: dot key
    """))
    assert data['a'] == 'plain text'
    assert data['b'] == 'quoted: with colon'
    assert data['c'] == "single 'quote'"
    assert data['d'] == ['x', 'y z', 'w'] and data['e'] == []
    assert data['f'] == 'folded line one line two\nsecond paragraph'
    assert data['g'] == ['one', {'key': 'k1', 'val': 'v1'}, '**/test']
    assert data['h'] == {'inner': '1'} and data['.'] == 'dot key'


def test_yaml_subset_reports_the_line_of_a_problem():
    with pytest.raises(extract.YamlError, match='line 2'):
        extract.parse_yaml_subset('a: 1\n  b: 2\n', 'x.yaml')
    with pytest.raises(extract.YamlError, match='twice'):
        extract.parse_yaml_subset('a: 1\na: 2\n')


def _stringify(x):
    if isinstance(x, dict):
        return {str(k): _stringify(v) for k, v in x.items()}
    if isinstance(x, list):
        return [_stringify(v) for v in x]
    if isinstance(x, bool):                 # the small reader keeps "false" as text; extract.py reads it as a yes/no
        return 'true' if x else 'false'
    return x if isinstance(x, str) or x is None else str(x)


def test_real_systems_yaml_reads_the_same_as_pyyaml():
    yaml = pytest.importorskip('yaml')
    text = (REPO / 'simulation' / 'codemap' / 'systems.yaml').read_text(encoding='utf-8')
    mine = extract.parse_yaml_subset(text, 'systems.yaml')
    assert _stringify(yaml.safe_load(text)) == mine
    assert len(mine['systems']) in range(8, 20)
    keys = {s['key'] for s in mine['systems']}
    assert {'team_stack', 'simulator', 'ground_station', 'tests', 'other'} <= keys
    assert {e['key'] for e in mine['externals']} >= {'gazebo', 'ardupilot_sitl', 'mavproxy', 'mavros',
                                                     'mission_planner', 'lattice_cloud'}
    for s in mine['systems'] + mine['externals']:
        assert len(s.get('summary', '').split('. ')) >= 2, f'{s["key"]}: the summary should be 3-5 sentences'


# ---------------------------------------------------------------------------------------------------------
# Python: ids, qualified names and trace_index line numbers
# ---------------------------------------------------------------------------------------------------------

PY_SAMPLE = '''
"""tools/sample.py -- PURPOSE: a sample module with every kind of definition.

Second paragraph (not in the summary).
"""
import functools


def top(a, b=2):
    """Add two numbers."""
    return a + b


@functools.lru_cache()
@staticmethod
def decorated():
    return 1


class Widget:
    """A widget."""

    def __init__(self):
        self.size = 1

    @property
    def value(self):
        return self.size

    @value.setter
    def value(self, v):
        self.size = v

    def method(self):
        def inner(x):
            return x * 2
        return inner(3)

    class Nested:
        def deep(self):
            return 0


def outer():
    class Local:
        def meth(self):
            return 5
    return Local


async def later():
    return None
'''


def test_python_ids_follow_the_contract(tmp_path):
    r = build(tmp_path, {'tools/sample.py': PY_SAMPLE})
    p = 'tools/sample.py'
    ids = {nid for nid in r.nodes if nid.startswith(f'fn:{p}::')}
    expect = {'top', 'decorated', 'Widget', 'Widget.__init__', 'Widget.value', 'Widget.method', 'Widget.method.inner',
              'Widget.Nested', 'Widget.Nested.deep', 'outer', 'outer.Local', 'outer.Local.meth', 'later'}
    assert ids == {f'fn:{p}::{q}' for q in expect}
    assert not any('<locals>' in nid for nid in r.nodes)
    kind = {q: r.nodes[f'fn:{p}::{q}']['kind'] for q in expect}
    assert kind['Widget'] == 'class' and kind['Widget.method'] == 'method' and kind['top'] == 'function'
    assert kind['Widget.method.inner'] == 'function' and kind['Widget.Nested.deep'] == 'method'
    assert r.nodes[f'fn:{p}::Widget.method.inner']['parent'] == f'fn:{p}::Widget.method'
    assert r.nodes[f'fn:{p}::Widget']['parent'] == f'mod:{p}'
    top = r.nodes[f'fn:{p}::top']
    assert top['signature'] == 'top(a, b=2)' and top['doc'] == 'Add two numbers.'
    assert top['explain'] == 'Add two numbers.' + extract.DOCSTRING_MARK
    assert 'return a + b' in top['source']
    assert r.nodes[f'fn:{p}::later']['signature'].startswith('async later(')
    # the property's getter and setter are ONE block that shows both definitions
    prop = r.nodes[f'fn:{p}::Widget.value']
    assert 'also defined at line' in prop['source'] and prop['source'].count('def value') == 2
    mod = r.nodes[f'mod:{p}']
    assert mod['summary'] == 'A sample module with every kind of definition.'
    assert mod['lang'] == 'python' and mod['line'] == 1 and mod['end_line'] == len(PY_SAMPLE.strip('\n').split('\n'))


def _code_objects(code, prefix=''):
    """(qualified name without <locals>, co_firstlineno) of every def/class code object, like the tracer sees."""
    for const in code.co_consts:
        if isinstance(const, types.CodeType):
            if const.co_name.startswith('<'):             # lambdas, comprehensions
                continue
            qual = f'{prefix}.{const.co_name}' if prefix else const.co_name
            yield qual, const.co_firstlineno
            yield from _code_objects(const, qual)


def test_trace_index_lines_are_python_s_own_co_firstlineno(tmp_path):
    r = build(tmp_path, {'tools/sample.py': PY_SAMPLE})
    mod_id = 'mod:tools/sample.py'
    table = r.cm['trace_index']['functions'][mod_id]
    code = compile(textwrap.dedent(PY_SAMPLE).lstrip('\n'), 'sample.py', 'exec')
    seen = list(_code_objects(code))
    assert len(seen) == 14                               # 13 blocks, the property has two code objects
    for qual, line in seen:
        assert table[str(line)] == f'fn:tools/sample.py::{qual}', (qual, line)
    # decorated: the key is the FIRST decorator's line, as Python reports it
    src_lines = textwrap.dedent(PY_SAMPLE).lstrip('\n').split('\n')
    assert src_lines[int([k for k, v in table.items() if v.endswith('::decorated')][0]) - 1].startswith('@functools')
    assert r.cm['trace_index']['py_files']['tools/sample.py'] == mod_id
    assert r.cm['trace_index']['py_modules']['sample'] == mod_id


def test_importable_names_and_colcon_install_paths(tmp_path):
    files = ros_python_pkg('camera', {'node.py': 'def main():\n    pass\n', 'sub/__init__.py': '',
                                      'sub/deep.py': 'X = 1\n'}, console=['cam = camera.node:main'])
    files.update({
        'ros2_ws/src/brain/package.xml': PKG_XML.format(name='brain', build='ament_cmake'),
        'ros2_ws/src/brain/CMakeLists.txt': """
            project(brain)
            install(PROGRAMS src/brain_main.py DESTINATION lib/${PROJECT_NAME})
            install(PROGRAMS scripts/tool.py DESTINATION lib/${PROJECT_NAME} RENAME tool)
            install(FILES src/helper.py DESTINATION lib/${PROJECT_NAME})
            ament_python_install_package(${PROJECT_NAME})
        """,
        'ros2_ws/src/brain/src/brain_main.py': 'import helper\n\ndef run():\n    helper.go()\n',
        'ros2_ws/src/brain/src/helper.py': 'def go():\n    return 1\n',
        'ros2_ws/src/brain/scripts/tool.py': 'print(1)\n',
        'ros2_ws/src/brain/brain/__init__.py': '',
        'ros2_ws/src/brain/brain/logic.py': 'def f():\n    return 1\n',
    })
    r = build(tmp_path, files)
    ti = r.cm['trace_index']
    assert ti['py_modules']['camera.node'] == 'mod:ros2_ws/src/camera/camera/node.py'
    assert ti['py_modules']['camera.sub.deep'] == 'mod:ros2_ws/src/camera/camera/sub/deep.py'
    assert ti['py_modules']['camera'] == 'mod:ros2_ws/src/camera/camera/__init__.py'
    assert ti['py_modules']['helper'] == 'mod:ros2_ws/src/brain/src/helper.py'       # a script's sibling
    assert ti['py_modules']['brain.logic'] == 'mod:ros2_ws/src/brain/brain/logic.py'
    pf = ti['py_files']
    assert pf['ros2_ws/build/camera/camera/node.py'] == 'mod:ros2_ws/src/camera/camera/node.py'
    assert pf['ros2_ws/install/camera/lib/python3.10/site-packages/camera/node.py'] == \
        'mod:ros2_ws/src/camera/camera/node.py'
    assert pf['ros2_ws/install/brain/lib/brain/brain_main.py'] == 'mod:ros2_ws/src/brain/src/brain_main.py'
    assert pf['ros2_ws/install/brain/lib/brain/tool'] == 'mod:ros2_ws/src/brain/scripts/tool.py'
    assert pf['ros2_ws/install/brain/lib/brain/helper.py'] == 'mod:ros2_ws/src/brain/src/helper.py'
    assert pf['ros2_ws/install/brain/local/lib/python3.10/dist-packages/brain/logic.py'] == \
        'mod:ros2_ws/src/brain/brain/logic.py'
    assert ti['programs']['ros2_ws/install/camera/lib/camera/cam'] == 'mod:ros2_ws/src/camera/camera/node.py'
    # entry points: on the function a console script names, and on the module of every program
    assert r.nodes['fn:ros2_ws/src/camera/camera/node.py::main']['entry_point'] == 'ros2 run camera cam'
    assert r.nodes['fn:ros2_ws/src/camera/camera/node.py::main']['called_when'].startswith('When you start')
    assert r.nodes['mod:ros2_ws/src/brain/src/brain_main.py']['entry_point'] == 'ros2 run brain brain_main.py'
    assert r.nodes['mod:ros2_ws/src/brain/scripts/tool.py']['entry_point'] == 'ros2 run brain tool'
    # a script importing its sibling: import edge and call through the module name
    assert ('mod:ros2_ws/src/brain/src/brain_main.py', 'mod:ros2_ws/src/brain/src/helper.py', 'imports') in r.edges
    assert ('fn:ros2_ws/src/brain/src/brain_main.py::run', 'fn:ros2_ws/src/brain/src/helper.py::go', 'calls') in r.edges


# ---------------------------------------------------------------------------------------------------------
# calls
# ---------------------------------------------------------------------------------------------------------

def test_calls_are_found_only_when_clear(tmp_path):
    lib = '''
        """tools/lib.py -- PURPOSE: a library."""
        def helper():
            return 1

        class Engine:
            def start(self):
                return helper()
    '''
    app = '''
        """tools/app.py -- PURPOSE: uses the library."""
        import lib
        from lib import helper, Engine as Motor


        def local_fn():
            return 2


        class Base:
            def ping(self):
                return 1


        class Car(Base):
            def __init__(self):
                self.engine = Motor()
                self.other = something_unknown()

            def drive(self):
                self.engine.start()
                self.other.start()
                self.ping()
                super().ping()
                local_fn()
                lib.helper()
                helper()

            def shadowed(self, local_fn):
                local_fn()
                m = Motor()
                m.start()
                obj.start()


        if __name__ == '__main__':
            Car().drive()
    '''
    r = build(tmp_path, {'tools/lib.py': lib, 'tools/app.py': app})
    A, L = 'fn:tools/app.py::', 'fn:tools/lib.py::'
    e = r.edges
    assert (A + 'Car.__init__', L + 'Engine', 'calls') in e
    assert r.labels[(A + 'Car.__init__', L + 'Engine', 'calls')] == 'creates'
    assert (A + 'Car.drive', L + 'Engine.start', 'calls') in e          # self.engine always holds an Engine
    assert (A + 'Car.drive', A + 'Base.ping', 'calls') in e             # self.ping inherited + super().ping
    assert (A + 'Car.drive', A + 'local_fn', 'calls') in e
    assert (A + 'Car.drive', L + 'helper', 'calls') in e                # lib.helper() and helper()
    assert (A + 'Car.shadowed', A + 'local_fn', 'calls') not in e       # the parameter hides the function
    assert (A + 'Car.shadowed', L + 'Engine.start', 'calls') in e       # m = Motor(); m.start()
    assert (L + 'Engine.start', L + 'helper', 'calls') in e
    assert ('mod:tools/app.py', A + 'Car', 'calls') in e                # module code under __main__
    assert ('mod:tools/app.py', 'mod:tools/lib.py', 'imports') in e
    # nothing is guessed for objects of unknown type
    starts = [x for x in e if x[1] == L + 'Engine.start']
    assert {x[0] for x in starts} == {A + 'Car.drive', A + 'Car.shadowed'}


# ---------------------------------------------------------------------------------------------------------
# ROS topics and services
# ---------------------------------------------------------------------------------------------------------

def test_topics_services_types_callbacks_and_owners(tmp_path):
    node = '''
        """talker/node.py -- PURPOSE: a pretend ROS node."""
        from rclpy.node import Node
        from std_msgs.msg import String, Bool
        from mavros_msgs.srv import SetMode
        from .names import STATUS_TOPIC

        LOCAL = '/local/topic'


        class Talker(Node):
            TOPIC_C = 'class/const'

            def __init__(self):
                super().__init__('talker')
                self.pub = self.create_publisher(String, STATUS_TOPIC, 10)
                self.raw = self.create_publisher(Bool, 'image_raw', 10)
                self.loc = self.create_publisher(String, LOCAL, 10)
                self.cam = self.declare_parameter('cam_topic', '/camera/image').value
                self.create_subscription(Bool, self.cam, self.on_image, 10)
                self.create_subscription(String, self.TOPIC_C, lambda m: self.on_text(m.data), 10)
                self.mode = self.create_client(SetMode, '/mavros/set_mode')
                self.create_service(SetMode, '~/private', self.on_private)
                self.create_timer(2.0, self.tick)

            def say(self, text):
                self.pub.publish(String(data=text))

            def on_image(self, msg):
                self.raw.publish(msg)

            def on_text(self, text):
                pass

            def on_private(self, req, res):
                return res

            def tick(self):
                self.mode.call_async(SetMode.Request())


        def main():
            node = Talker()
            sub = node.create_subscription
            sub(String, '/mavros/state', print, 10)
    '''
    files = ros_python_pkg('talker', {'node.py': node, 'names.py': "STATUS_TOPIC = '/mavros/statustext/send'\n"},
                           console=['talk = talker.node:main'])
    r = build(tmp_path, files)
    F = 'fn:ros2_ws/src/talker/talker/node.py::'
    e, lab = r.edges, r.labels
    assert (F + 'Talker.say', 'topic:/mavros/statustext/send', 'publishes') in e      # constant from another module
    assert lab[(F + 'Talker.say', 'topic:/mavros/statustext/send', 'publishes')] == 'std_msgs/msg/String'
    assert (F + 'Talker.on_image', 'topic:/image_raw', 'publishes') in e              # relative name gets a /
    assert (F + 'Talker', 'topic:/local/topic', 'publishes') in e                     # never published: the class
    assert (F + 'Talker.on_image', 'topic:/camera/image', 'subscribes') in e          # a parameter's default
    assert 'default of the parameter cam_topic' in lab[(F + 'Talker.on_image', 'topic:/camera/image', 'subscribes')]
    assert (F + 'Talker.on_text', 'topic:/class/const', 'subscribes') in e            # lambda -> the method it calls
    assert (F + 'Talker.tick', 'topic:/mavros/set_mode', 'calls') in e                # the client is used in tick
    assert lab[(F + 'Talker.tick', 'topic:/mavros/set_mode', 'calls')] == 'service client mavros_msgs/srv/SetMode'
    assert (F + 'main', 'topic:/mavros/state', 'subscribes') in e                     # "sub = ...create_subscription"
    assert 'topic:/private' not in r.nodes and 'topic:~/private' not in r.nodes        # private names are skipped
    # MAVROS (systems.yaml topic_owners) answers the /mavros/ names our code only uses from one side
    assert ('ext:mavros', 'topic:/mavros/state', 'publishes') in e
    assert ('ext:mavros', 'topic:/mavros/set_mode', 'subscribes') in e
    assert ('ext:mavros', 'topic:/mavros/statustext/send', 'subscribes') in e
    assert r.nodes['topic:/mavros/state']['parent'] == 'ext:mavros'
    assert r.nodes['topic:/camera/image']['parent'] == 'sys:sim'                      # pinned by "parent:"
    assert r.nodes['topic:/image_raw']['parent'] == 'sys:stack'
    assert 'Type: std_msgs/msg/Bool' in r.nodes['topic:/image_raw']['summary']
    assert r.nodes['topic:/mavros/set_mode']['summary'].startswith('ROS 2 service')
    assert r.nodes['topic:/image_raw']['explain'].endswith(extract.GENERATED_MARK)
    # called_when found in the code: subscription callbacks, timers, entry points
    assert r.nodes[F + 'Talker.on_image']['called_when'] == 'Every time a message arrives on /camera/image.'
    assert r.nodes[F + 'Talker.tick']['called_when'] == 'Every 2 seconds (a ROS timer).'
    # links from systems.yaml, including a topic that only a link names
    assert ('ext:gazebo', 'ext:mavros', 'data') in e
    assert lab[('ext:gazebo', 'ext:mavros', 'data')] == 'a pretend link: UDP 1'
    assert ('ext:gazebo', 'topic:/sim/clock', 'publishes') in e
    assert r.nodes['topic:/sim/clock']['parent'] == 'ext:gazebo'
    assert r.nodes['ext:mavros']['process_match'] == ['\\bmavros_node\\b']


# ---------------------------------------------------------------------------------------------------------
# launch files
# ---------------------------------------------------------------------------------------------------------

def test_launch_files_python_and_xml(tmp_path):
    files = ros_python_pkg('camera', {'node.py': 'def main():\n    pass\n'}, console=['cam = camera.node:main'])
    files.update(ros_python_pkg('other', {'x.py': 'def main():\n    pass\n'}, console=['x = other.x:main']))
    files.update({
        'ros2_ws/src/camera/launch/team.launch.xml': """
            <launch>
              <node pkg="camera" exec="cam"/>
              <!-- <node pkg="other" exec="x"/> -->
              <include file="$(find-pkg-share mavros)/launch/apm.launch"/>
            </launch>
        """,
        'ros2_ws/src/other/launch/team.launch.xml': '<launch><node pkg="other" exec="x"/></launch>\n',
        'ros2_ws/src/camera/launch/sim.launch.py': """
            import os
            from ament_index_python.packages import get_package_share_directory
            from launch.actions import ExecuteProcess
            from launch_ros.actions import Node

            def generate_launch_description():
                tracker = os.path.join(get_package_share_directory('camera'), 'launch', 'team.launch.xml')
                return [Node(package='other', executable='x'), ExecuteProcess(cmd=['gz', 'sim'], name='gz_server'),
                        Node(package='ros_gz_sim', executable='create'), Node(package='nowhere', executable='y')]
        """,
    })
    r = build(tmp_path, files)
    xml, py = 'mod:ros2_ws/src/camera/launch/team.launch.xml', 'mod:ros2_ws/src/camera/launch/sim.launch.py'
    assert (xml, 'mod:ros2_ws/src/camera/camera/node.py', 'launches') in r.edges
    assert (xml, 'mod:ros2_ws/src/other/other/x.py', 'launches') not in r.edges        # commented out
    assert (xml, 'ext:mavros', 'launches') in r.edges
    assert (py, 'mod:ros2_ws/src/other/other/x.py', 'launches') in r.edges
    assert (py, 'ext:gazebo', 'launches') in r.edges                     # ExecuteProcess name + ros_gz_sim
    assert (py, xml, 'launches') in r.edges                              # the right one of two team.launch.xml
    assert (py, 'mod:ros2_ws/src/other/launch/team.launch.xml', 'launches') not in r.edges


# ---------------------------------------------------------------------------------------------------------
# bash
# ---------------------------------------------------------------------------------------------------------

COMMON_SH = """
#!/usr/bin/env bash
# _common.sh -- PURPOSE: helpers every script uses.
say() { printf '==> %s\\n' "$*"; }
"""

DEMO_SH = """
#!/usr/bin/env bash
# =========================================================
#  demo.sh -- PURPOSE: the pretend demo.
#  It runs everything.
# =========================================================
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/_common.sh"
read -r -a OPTS <<< "${EXTRA:-}"

# cleanup : stop everything (comment above the function)
cleanup() {
  say "stopping"
  bash "$HERE/stop.sh" --all
}
trap cleanup EXIT INT

function multi {
  local x
  x="$(printf '%s' "a \\
     b")"
  cat <<'EOF'
fake_in_heredoc() {
  echo nothing
}
EOF
  echo "please run other.sh now"
  ros2 run camera cam --ros-args -p a:=1
  ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "{data: true}"
  python3 -m camera.node
  TRACKER="${TRACKER:-ros2 launch mavros apm.launch}"
}

after_continuation() { :; }
cleanup
multi
"""


def test_bash_functions_heredocs_here_strings_and_continuations(tmp_path):
    files = ros_python_pkg('camera', {'node.py': 'def main():\n    pass\n'}, console=['cam = camera.node:main'])
    files.update({'simulation/scripts/_common.sh': COMMON_SH, 'simulation/scripts/demo.sh': DEMO_SH,
                  'simulation/scripts/stop.sh': '#!/bin/bash\n# stop.sh -- PURPOSE: stop.\necho stop\n',
                  'simulation/scripts/other.sh': '#!/bin/bash\necho other\n'})
    r = build(tmp_path, files)
    D, C = 'fn:simulation/scripts/demo.sh::', 'fn:simulation/scripts/_common.sh::'
    funcs = {n for n in r.nodes if n.startswith(D)}
    assert funcs == {D + 'cleanup', D + 'multi', D + 'after_continuation'}             # not the heredoc's text
    text = textwrap.dedent(DEMO_SH).lstrip('\n').split('\n')
    for name in ('cleanup', 'multi', 'after_continuation'):
        n = r.nodes[D + name]
        assert text[n['line'] - 1].lstrip().startswith((name, 'function ' + name)), name
        assert text[n['end_line'] - 1].rstrip().endswith('}'), name
    assert r.nodes[D + 'cleanup']['doc'] == 'cleanup : stop everything (comment above the function)'
    assert r.nodes['mod:simulation/scripts/demo.sh']['summary'] == 'The pretend demo. It runs everything.'
    e = r.edges
    assert (D + 'cleanup', C + 'say', 'calls') in e                                    # function of a sourced file
    assert ('mod:simulation/scripts/demo.sh', 'mod:simulation/scripts/_common.sh', 'imports') in e
    assert (D + 'cleanup', 'mod:simulation/scripts/stop.sh', 'runs') in e
    assert ('mod:simulation/scripts/demo.sh', D + 'cleanup', 'calls') in e
    assert r.labels[('mod:simulation/scripts/demo.sh', D + 'cleanup', 'calls')] in ('', 'trap handler')
    assert r.nodes[D + 'cleanup']['called_when'].startswith('When the script gets the signal(s) EXIT INT')
    assert (D + 'multi', 'mod:ros2_ws/src/camera/camera/node.py', 'runs') in e          # ros2 run + python3 -m
    assert (D + 'multi', 'topic:/camera/trigger', 'publishes') in e
    assert (D + 'multi', 'ext:mavros', 'runs') in e                                    # from a VAR="${VAR:-...}" value
    assert not any(x[1] == 'mod:simulation/scripts/other.sh' for x in e)               # only a printed message
    assert r.cm['trace_index']['sh_files']['simulation/scripts/demo.sh'] == 'mod:simulation/scripts/demo.sh'


def test_bash_command_splitter_and_script_paths():
    cmds = extract.bash_split_commands('a=1 sudo -E foo x 2>/dev/null | bar "$(baz q)" && { qux; }\n')
    names = [extract.bash_command_words(w)[0][:1] for _l, w in cmds]
    assert ['baz'] in names and ['foo'] in names and ['bar'] in names and ['qux'] in names
    foo = next(w for _l, w in cmds if 'foo' in w)
    assert '/dev/null' not in foo                                                       # redirect target dropped
    assert extract._strip_expansions('"$(cd "$(dirname "$0")" && pwd)/_common.sh"') == '/_common.sh'
    assert extract._strip_expansions('${DIR}/../setup/x.sh') == '/../setup/x.sh'


# ---------------------------------------------------------------------------------------------------------
# C#, PowerShell and batch files
# ---------------------------------------------------------------------------------------------------------

CS = """
/*
 * plugin.cs -- PURPOSE: a pretend Mission Planner plugin.
 */
using System;

namespace Pretend
{
    /// <summary>The phases.</summary>
    public enum Phase { A, B }

    public class Info
    {
        public string Kind = "x";
        public string Word { get { return Kind == "person" ? "hiker" : "tent"; } }
        public int Twice => Count() * 2;
        int Count() { return 1; }
    }

    public class Plugin : Base
    {
        // void Commented() { }
        private const string Url = "http://127.0.0.1:8081/pov.mjpg";
        public Plugin() : base(1) { Start(); }

        /// <summary>Starts it.</summary>
        public override bool Start()
        {
            var i = new Info();
            string s = "not a method(";
            AddEvent("a", 1);
            Helper.Run();
            return true;
        }

        private void AddEvent(string text, int level) { AddEvent(text, level, DateTime.Now); }
        private void AddEvent(string text, int level, DateTime when) { }

        private void RunSimAction(string action)
        {
            var args = "-- bash -lc \\"bash ~/repo/simulation/scripts/act.sh " + action + "\\"";
        }

        private class Helper
        {
            public static void Run() { }
        }
    }
}
"""


def test_csharp_classes_methods_overloads_and_properties(tmp_path):
    files = {'simulation/windows/plugin/plugin.cs': CS, 'simulation/scripts/act.sh': '#!/bin/bash\necho act\n'}
    r = build(tmp_path, files)
    P = 'fn:simulation/windows/plugin/plugin.cs::'
    ids = {n[len(P):] for n in r.nodes if n.startswith(P)}
    assert ids == {'Phase', 'Info', 'Info.Word', 'Info.Twice', 'Info.Count', 'Plugin', 'Plugin.Plugin', 'Plugin.Start',
                   'Plugin.AddEvent', 'Plugin.RunSimAction', 'Plugin.Helper', 'Plugin.Helper.Run'}
    assert r.nodes[P + 'Plugin.Helper']['parent'] == P + 'Plugin'
    assert r.nodes[P + 'Plugin.Start']['kind'] == 'method' and r.nodes[P + 'Plugin']['kind'] == 'class'
    assert r.nodes[P + 'Plugin.Start']['doc'] == 'Starts it.'
    assert 'also defined at line' in r.nodes[P + 'Plugin.AddEvent']['source']             # overloads: one block
    e = r.edges
    assert (P + 'Plugin.Start', P + 'Info', 'calls') in e                                 # new Info()
    assert (P + 'Plugin.Start', P + 'Plugin.AddEvent', 'calls') in e
    assert (P + 'Plugin.Start', P + 'Plugin.Helper.Run', 'calls') in e
    assert (P + 'Plugin.Plugin', P + 'Plugin.Start', 'calls') in e
    assert (P + 'Info.Twice', P + 'Info.Count', 'calls') in e
    assert (P + 'Plugin.RunSimAction', 'mod:simulation/scripts/act.sh', 'runs') in e
    assert r.nodes['mod:simulation/windows/plugin/plugin.cs']['summary'] == 'A pretend Mission Planner plugin.'


def test_batch_and_powershell_files(tmp_path):
    files = {
        'simulation/windows/plugin/Start.bat': (
            '@echo off\nrem  Start.bat -- PURPOSE: start the demo.\nrem\n'
            'echo   bash simulation/scripts/act.sh is NOT run by this echo\n'
            'wsl.exe -d Ubuntu-22.04 -- bash -lc "cd ~/repo || exit 1; bash simulation/scripts/act.sh --keep"\n'),
        'simulation/windows/plugin/install.ps1': (
            "<#\ninstall.ps1 -- PURPOSE: copy the plugin.\n#>\n"
            "function Copy-It { param($x) Write-Host $x }\n"
            "$source = Join-Path $PSScriptRoot 'plugin.cs'\n"),
        'simulation/windows/plugin/plugin.cs': 'public class X { }\n',
        'simulation/scripts/act.sh': '#!/bin/bash\necho act\n',
    }
    r = build(tmp_path, files)
    bat, ps = 'mod:simulation/windows/plugin/Start.bat', 'mod:simulation/windows/plugin/install.ps1'
    assert (bat, 'mod:simulation/scripts/act.sh', 'runs') in r.edges
    assert len([x for x in r.edges if x[0] == bat]) == 1
    assert r.nodes[bat]['summary'] == 'Start the demo.'
    assert 'fn:simulation/windows/plugin/install.ps1::Copy-It' in r.nodes
    assert (ps, 'mod:simulation/windows/plugin/plugin.cs', 'data') in r.edges
    assert r.nodes[ps]['summary'] == 'Copy the plugin.'


# ---------------------------------------------------------------------------------------------------------
# packages, systems and every kind of file
# ---------------------------------------------------------------------------------------------------------

def test_packages_systems_and_file_kinds(tmp_path):
    files = ros_python_pkg('lattice', {'sinks/__init__.py': '', 'sinks/rest.py': 'def send():\n    pass\n'})
    files.update({
        'ros2_ws/src/lattice/test/test_rest.py': 'def test_sends_one_entity_per_target():\n    assert True\n',
        'ros2_ws/src/lattice/README.md': '<!--\nlattice/README.md -- PURPOSE: the guide to lattice.\n-->\n# Lattice\n',
        'ros2_ws/src/lattice/lattice/old.py.bak': 'x = 1\n',
        'ros2_ws/src/lattice/docs/pic.png': '',
        'scripts/run.sh': '#!/bin/bash\necho run\n',
        'scripts/sub/deeper.sh': '#!/bin/bash\necho deeper\n',          # "scripts" is flat: not claimed
        'tools/a/b/c.py': 'X = 1\n',                                    # "tools/**" is recursive
        'notes.md': 'Some notes.\n\nMore.\n',
        'page.mhtml': 'skipped',
        'LOCAL_NOTES.md': 'private notes, never in the map',
    })
    r = build(tmp_path, files)
    n = r.nodes
    assert n['pkg:ros2_ws/src/lattice']['parent'] == 'sys:stack'
    assert n['pkg:ros2_ws/src/lattice']['summary'] == 'The pretend package lattice.'
    assert n['pkg:ros2_ws/src/lattice/lattice/sinks']['parent'] == 'sys:stack'          # sub-package inherits
    assert n['pkg:ros2_ws/src/lattice/lattice/sinks']['label'] == 'lattice.sinks'
    assert n['mod:ros2_ws/src/lattice/lattice/sinks/rest.py']['parent'] == 'pkg:ros2_ws/src/lattice/lattice/sinks'
    assert n['pkg:ros2_ws/src/lattice/test']['parent'] == 'sys:tests' and n['pkg:ros2_ws/src/lattice/test']['is_test']
    t = n['fn:ros2_ws/src/lattice/test/test_rest.py::test_sends_one_entity_per_target']
    assert t['is_test'] and t['explain'] == 'Test: sends one entity per target.'
    assert n['mod:tools/a/b/c.py']['parent'] == 'pkg:tools'
    assert n['mod:ros2_ws/src/lattice/README.md']['lang'] == 'markdown'
    assert n['mod:ros2_ws/src/lattice/README.md']['summary'] == 'The guide to lattice.'
    assert n['mod:ros2_ws/src/lattice/lattice/old.py.bak']['lang'] == 'backup'
    assert n['mod:ros2_ws/src/lattice/lattice/old.py.bak']['explain'].startswith('An old copy of old.py')
    assert n['mod:ros2_ws/src/lattice/docs/pic.png']['lang'] == 'image'
    assert 'line' not in n['mod:ros2_ws/src/lattice/docs/pic.png']
    assert n['mod:ros2_ws/src/lattice/resource/lattice']['lang'] == 'marker'
    assert n['mod:notes.md']['parent'] == 'pkg:.' and n['mod:notes.md']['summary'] == 'Some notes.'
    assert 'mod:page.mhtml' not in n and 'mod:LOCAL_NOTES.md' not in n
    # a file nobody claims is reported (and lands in "other" so the map stays whole)
    assert r.report['unassigned'] == ['scripts/sub/deeper.sh']
    assert n['mod:scripts/sub/deeper.sh']['parent'] == 'pkg:scripts/sub'
    assert n['pkg:scripts/sub']['parent'] == 'sys:other'


WIRED_SYSTEMS = SYSTEMS.replace("""  - key: tests
    label: Tests""", """  - key: logs
    label: Run logs
    summary: The pretend run logs. Two sentences.
    packages:
      - hil/**
    files:
      - ros2_ws/src/demo/demo/recorder.py
      - ros2_ws/src/demo/demo/run_*.py
      - simulation/scripts/logs.sh
      - simulation/scripts/not_yet.sh
  - key: tests
    label: Tests""").replace("""links:
""", """package_info:
  ros2_ws/src/demo/demo@logs:
    label: "demo: the recorder"
links:
  - id: radio
    from: ext:gazebo
    to: ext:mavros
    kind: mavlink
    label: the pretend radio
    wire: mav
    arrow: both
    status: current
    title: The radio
    via: UDP 14550 · Windows host
    code: [simulation/scripts/logs.sh]
    explain: Telemetry down, commands up.
  - id: wire_only
    map: false
    wire_from: sim
    wire_to: rail
    wire: ros
    status: being built
    path: 300,10 300,90
    label_at: 5,6 start stack
    offset: -4
""") + """wiring:
  title: The pretend wiring
  intro: Two boxes and a rail.
  size: 800 600
  kinds:
    - key: mav
      label: MAVLink
      text: The autopilot's language.
  statuses:
    - key: current
      label: works today
  zones:
    - id: pc
      label: THE PC
      at: 0 0 800 600
  boxes:
    - id: sim
      title: Gazebo
      node: ext:gazebo
      at: 10 10 200 80
      lines:
        - "physics: one step at a time"
    - id: mav
      title: MAVROS
      node: ext:mavros
      at: 400 10 200 80
      status: legacy
    - id: rail
      title: ROS 2
      shape: rail
      at: 300 100 10 300
    - id: later
      title: Not built yet
      node: mod:does/not/exist.py
      at: 10 200 100 50
  steps:
    - title: One
      text: The radio talks.
      wires: [radio]
      boxes: [sim, mav]
"""


def test_file_claims_inheritance_and_the_wiring_section(tmp_path):
    files = ros_python_pkg('demo', {'recorder.py': 'def record():\n    pass\n', 'run_summary.py': 'X = 1\n',
                                    'panel.py': 'Y = 2\n'})
    files.update({'simulation/scripts/logs.sh': '#!/bin/bash\n# logs.sh -- PURPOSE: show the runs (UDP 14550).\n',
                  'simulation/scripts/demo.sh': '#!/bin/bash\necho demo\n',
                  'hil/tool.sh': '#!/bin/bash\necho hil\n'})
    # a ROS package inside "hil/**" that nothing names: it belongs to the same system
    files.update({k.replace('ros2_ws/src/inner', 'hil/inner'): v
                  for k, v in ros_python_pkg('inner', {'a.py': 'Z = 3\n'}).items()})
    r = build(tmp_path, files, systems=WIRED_SYSTEMS)
    n, e = r.nodes, r.edges
    # claimed files: one package per (folder, system), the rest of the folder stays where it was
    grp = 'pkg:ros2_ws/src/demo/demo@logs'
    assert n['mod:ros2_ws/src/demo/demo/recorder.py']['parent'] == grp
    assert n['mod:ros2_ws/src/demo/demo/run_summary.py']['parent'] == grp
    assert n['mod:ros2_ws/src/demo/demo/panel.py']['parent'] == 'pkg:ros2_ws/src/demo'
    assert n[grp]['parent'] == 'sys:logs' and n[grp]['label'] == 'demo: the recorder'
    assert n[grp]['path'] == 'ros2_ws/src/demo/demo' and 'recorder.py' in n[grp]['summary']
    assert n['mod:simulation/scripts/logs.sh']['parent'] == 'pkg:simulation/scripts@logs'
    assert n['pkg:simulation/scripts@logs']['parent'] == 'sys:logs'
    assert n['mod:simulation/scripts/demo.sh']['parent'] == 'pkg:simulation/scripts'
    assert any('simulation/scripts/not_yet.sh' in w and 'does not exist' in w for w in r.report['warnings'])
    # the ROS package inside hil/** joined the "logs" system, its own files stay in it
    assert n['pkg:hil/inner']['parent'] == 'sys:logs' and n['mod:hil/inner/inner/a.py']['parent'] == 'pkg:hil/inner'
    assert n['mod:hil/tool.sh']['parent'] == 'pkg:hil'
    assert r.report['unassigned'] == [] and r.report['problems'] == []
    # links: a wire with map: false is not a map edge; a normal link is both
    assert ('ext:gazebo', 'ext:mavros', 'mavlink') in e
    assert not any(x[0] == 'sim' or x[1] == 'rail' for x in e)
    w = r.cm['wiring']
    assert w['title'] == 'The pretend wiring' and w['width'] == 800 and w['height'] == 600
    assert [b['id'] for b in w['boxes']] == ['sim', 'mav', 'rail', 'later']
    assert w['boxes'][0]['members'] == ['ext:gazebo'] and w['boxes'][0]['lines'] == ['physics: one step at a time']
    assert w['boxes'][1]['status'] == 'legacy' and w['boxes'][2]['shape'] == 'rail'
    radio = next(x for x in w['wires'] if x['id'] == 'radio')
    assert (radio['from'], radio['to'], radio['kind'], radio['arrow']) == ('sim', 'mav', 'mav', 'both')
    assert radio['via'] == 'UDP 14550 · Windows host' and radio['code'] == ['simulation/scripts/logs.sh']
    assert radio['map_from'] == 'ext:gazebo' and radio['label'] == 'the pretend radio'
    only = next(x for x in w['wires'] if x['id'] == 'wire_only')
    assert (only['from'], only['to'], only['status']) == ('sim', 'rail', 'being built') and 'map_from' not in only
    assert only['path'] == [[300.0, 10.0], [300.0, 90.0]] and only['label_at'] == [5.0, 6.0, 'start', 'stack']
    assert only['offset'] == -4.0
    assert w['steps'] == [{'title': 'One', 'text': 'The radio talks.', 'wires': ['radio'], 'boxes': ['sim', 'mav']}]
    assert any('box later opens "mod:does/not/exist.py"' in x for x in r.report['warnings'])
    assert r.report['stats']['wiring view wires'] == 2
    # the output file keeps the wiring, one box / wire per line
    text = extract.dumps_codemap(r.cm)
    assert json.loads(text)['wiring'] == w
    assert sum(1 for ln in text.split('\n') if ln.startswith('{"') and '"title":"' in ln and '"x":' in ln) == 4


@pytest.mark.parametrize('change, words', [
    (('    wire_to: rail\n', '    wire_to: nowhere\n'), 'no box holds its "to" end'),
    (('  - id: wire_only\n', '  - id: radio\n'), 'used twice'),
    (('    wire: ros\n', '    wire: smoke\n'), 'wire "smoke" is not one of'),
    (('    status: being built\n', '    status: someday\n'), 'status "someday"'),
    (('      wires: [radio]\n', '      wires: [radio, ghost]\n'), 'names the wire "ghost"'),
    (('      boxes: [sim, mav]\n', '      boxes: [sim, ufo]\n'), 'names the box "ufo"'),
    (('    path: 300,10 300,90\n', '    path: 300,10 300\n'), 'path must be pairs'),
    (('    label_at: 5,6 start stack\n', '    label_at: 5,6 sideways\n'), 'label_at must be'),
    (('      at: 400 10 200 80\n', '      at: 400 10\n'), 'needs an id and "at: x y width height"'),
])
def test_wiring_mistakes_are_problems(tmp_path, change, words):
    systems = WIRED_SYSTEMS.replace(*change, 1)
    assert systems != WIRED_SYSTEMS
    r = build(tmp_path, {'simulation/scripts/logs.sh': '#!/bin/bash\n'}, systems=systems)
    assert any(words in p for p in r.report['problems']), r.report['problems']


def test_a_file_two_systems_claim_is_a_problem(tmp_path):
    systems = WIRED_SYSTEMS.replace('      - simulation/windows/plugin\n',
                                    '      - simulation/windows/plugin\n    files:\n      - simulation/scripts/logs.sh\n')
    assert systems != WIRED_SYSTEMS
    r = build(tmp_path, {'simulation/scripts/logs.sh': '#!/bin/bash\n'}, systems=systems)
    assert any('claimed by two systems' in p for p in r.report['problems'])


def test_a_package_two_patterns_claim_equally_is_a_problem(tmp_path):
    systems = SYSTEMS.replace('      - "**/test"', '      - "**/test"\n      - ros2_ws/src/*')
    r = build(tmp_path, ros_python_pkg('alpha', {'a.py': 'X = 1\n'}), systems=systems)
    assert any('matches two systems equally well' in p for p in r.report['problems'])


# ---------------------------------------------------------------------------------------------------------
# explanations and coverage
# ---------------------------------------------------------------------------------------------------------

def test_explanations_merge_fallbacks_and_coverage(tmp_path):
    files = {
        'tools/m.py': '''
            """tools/m.py -- PURPOSE: a module."""
            def explained():
                return 1

            def documented():
                """Does one thing.

                More words that are not copied."""

            def bare():
                return 3
        ''',
        'tools/test/test_m.py': 'def test_explained_returns_one():\n    assert True\n',
    }
    expl = {
        'a.json': {'area': 'tools', 'entries': {
            'fn:tools/m.py::explained': {'explain': 'Gives back the number one.', 'called_when': 'Never, really.',
                                         'inputs': 'Nothing.', 'outputs': '1', 'effects': 'None.', 'junk': 'x'},
            'pkg:tools': {'explain': 'The tools folder.', 'how_it_fits': 'It holds the tools.'},
            'fn:tools/m.py::does_not_exist': {'explain': 'Nothing matches me.'},
        }},
        'b.json': '{ this is not json',
    }
    r = build(tmp_path, files, explanations=expl)
    n = r.nodes
    ex = n['fn:tools/m.py::explained']
    assert ex['explain'] == 'Gives back the number one.' and ex['called_when'] == 'Never, really.'
    assert ex['inputs'] == 'Nothing.' and ex['outputs'] == '1' and ex['effects'] == 'None.' and 'junk' not in ex
    assert n['pkg:tools']['how_it_fits'] == 'It holds the tools.'
    assert n['fn:tools/m.py::documented']['explain'] == 'Does one thing.' + extract.DOCSTRING_MARK
    assert 'explain' not in n['fn:tools/m.py::bare']
    assert n['fn:tools/test/test_m.py::test_explained_returns_one']['explain'] == 'Test: explained returns one.'
    assert r.report['unmatched_explanations'] == ['a.json: fn:tools/m.py::does_not_exist']
    assert any('b.json' in w for w in r.report['warnings'])
    cov = r.report['coverage']['by_kind']['function']
    assert cov == {'total': 4, 'explanations': 1, 'docstring': 1, 'test name': 1, 'none': 1}
    assert r.report['coverage']['by_system']['sys:stack']['explanations'] >= 2
    # two of the four pretend systems have a summary in systems.yaml, the other two have none
    assert r.report['coverage']['by_kind']['system'] == {'total': 4, 'systems.yaml': 2, 'none': 2}


# ---------------------------------------------------------------------------------------------------------
# output, determinism and the command line
# ---------------------------------------------------------------------------------------------------------

def test_output_is_deterministic_and_one_block_per_line(tmp_path):
    files = {'tools/sample.py': PY_SAMPLE, 'simulation/scripts/demo.sh': DEMO_SH,
             'simulation/scripts/_common.sh': COMMON_SH, 'simulation/windows/plugin/plugin.cs': CS}
    a = build(tmp_path / 'a', files)
    b = build(tmp_path / 'b', files)
    text_a, text_b = extract.dumps_codemap(a.cm), extract.dumps_codemap(b.cm)
    assert text_a == text_b
    data = json.loads(text_a)
    assert data == json.loads(json.dumps(a.cm))
    assert [n['id'] for n in data['nodes']] == sorted(n['id'] for n in data['nodes'])
    node_lines = [ln for ln in text_a.split('\n') if ln.startswith('{"') and '"kind":' in ln and '"id":' in ln]
    assert len(node_lines) == len(data['nodes'])
    assert set(data) == {'schema', 'generated', 'commit', 'nodes', 'edges', 'trace_index'}
    assert data['generated'] == '2026-01-01T00:00:00Z' and data['schema'] == 1


def test_command_line_check_no_source_and_out_file(tmp_path, capsys):
    repo = make_repo(tmp_path, {'tools/sample.py': PY_SAMPLE})
    out = repo / 'simulation' / 'codemap' / 'codemap.json'
    args = ['--repo', str(repo), '--systems', str(tmp_path / 'systems.yaml'), '--explanations', str(tmp_path / 'expl')]
    assert extract.main(args + ['--check', '--quiet']) == 0
    data = json.loads(out.read_text(encoding='utf-8'))
    assert any('source' in n for n in data['nodes'])
    assert 'mod:simulation/codemap/codemap.json' not in {n['id'] for n in data['nodes']}   # never maps itself
    assert extract.main(args + ['--no-source', '--quiet']) == 0
    data = json.loads(out.read_text(encoding='utf-8'))
    assert not any('source' in n for n in data['nodes'])
    write_files(repo, {'stray/unclaimed.py': 'X = 1\n'})
    assert extract.main(args + ['--check', '--quiet']) == 1
    assert 'stray/unclaimed.py' in capsys.readouterr().out


def test_one_odd_file_is_a_problem_not_a_crash(tmp_path, monkeypatch):
    real_clean = extract.bash_clean

    def fussy_clean(text):
        if 'BOOM' in text:
            raise RuntimeError('an odd file')
        return real_clean(text)
    monkeypatch.setattr(extract, 'bash_clean', fussy_clean)
    r = build(tmp_path, {'scripts/ok.sh': 'f() { :; }\n', 'scripts/odd.sh': '# BOOM\n', 'tools/sample.py': PY_SAMPLE})
    assert 'fn:scripts/ok.sh::f' in r.nodes and 'fn:tools/sample.py::top' in r.nodes
    assert 'mod:scripts/odd.sh' in r.nodes                          # the file still has its block
    assert any(p.startswith('scripts/odd.sh: bash_file failed') and 'an odd file' in p for p in r.report['problems'])


def test_small_text_helpers():
    assert extract.purpose_summary('x.py -- PURPOSE: does a thing.\nStill the same paragraph.\n\nNot this.') == \
        'Does a thing. Still the same paragraph.'
    assert extract.purpose_summary('demo.sh -- THE DEMO\n\nPURPOSE: shows it.') == 'Shows it.'
    assert extract.first_paragraph('') is None
    long = 'word ' * 400
    assert len(extract.first_paragraph(long)) <= extract.SUMMARY_CAP + 4
    assert extract.test_sentence('TestPayloadLogic.test_opens_at_1400') == 'Test (payload logic): opens at 1400.'
    assert extract.cap_text('a\n' * 3000, 100).endswith('more characters not shown)')
    assert extract.classify('a/b.py') == 'python' and extract.classify('a/x.launch.xml') == 'xml'
    assert extract.classify('a/new_od.py.bak2') == 'backup' and extract.classify('servo_wiggle.py\'') == 'backup'
    assert extract.classify('Lattice.mhtml') is None and extract.classify('CMakeLists.txt') == 'cmake'
    assert extract.glob_to_regex('**/test').match('test') and extract.glob_to_regex('**/test').match('a/b/test')
    assert not extract.glob_to_regex('ros2_ws/src/*').match('ros2_ws/src/a/b')


# ---------------------------------------------------------------------------------------------------------
# the real repository
# ---------------------------------------------------------------------------------------------------------

@pytest.fixture(scope='module')
def real():
    started = time.time()          # files saved after this moment (someone else editing) may differ from the map
    cm, report = extract.build_codemap(REPO, include_source=False, generated='2026-01-01T00:00:00Z', commit='test')
    nodes = {n['id']: n for n in cm['nodes']}
    edges = {(e['from'], e['to'], e['kind']) for e in cm['edges']}
    return types.SimpleNamespace(cm=cm, report=report, nodes=nodes, edges=edges, started=started)


def test_real_repo_follows_the_contract(real):
    assert real.report['duplicates'] == []
    assert real.report['unassigned'] == []
    assert real.report['problems'] == []
    prefixes = {'system': 'sys:', 'external': 'ext:', 'package': 'pkg:', 'module': 'mod:', 'class': 'fn:',
                'function': 'fn:', 'method': 'fn:', 'topic': 'topic:'}
    for nid, n in real.nodes.items():
        assert {'id', 'kind', 'label', 'parent'} <= set(n), nid
        assert n['kind'] in extract.NODE_KINDS and nid.startswith(prefixes[n['kind']]), nid
        assert n['parent'] is None or n['parent'] in real.nodes, nid
        if n['kind'] == 'module':
            assert real.nodes[n['parent']]['kind'] == 'package', nid
            assert nid == f'mod:{n["path"]}' and '\\' not in nid
        if n['kind'] == 'package':
            assert real.nodes[n['parent']]['kind'] == 'system', nid
        if n['kind'] == 'system':
            assert n['parent'] is None
    for e in real.cm['edges']:
        assert e['kind'] in extract.EDGE_KINDS and e['from'] in real.nodes and e['to'] in real.nodes
    assert len([n for n in real.nodes.values() if n['kind'] == 'system']) in range(8, 20)
    ti = real.cm['trace_index']
    assert {'py_modules', 'py_files', 'functions'} <= set(ti)
    for mod_id, table in ti['functions'].items():
        assert mod_id in real.nodes
        for line, fid in table.items():
            assert line.isdigit() and fid in real.nodes and fid.startswith(f'fn:{mod_id[4:]}::')


def test_real_repo_trace_index_matches_python_s_own_code_objects(real):
    """For EVERY Python file of the repo: compile it (nothing runs) and check that each def / class code object the
    tracer can report (its co_firstlineno) leads, through trace_index, to the block with the same qualified name
    (Python's own name without the ".<locals>" parts). This is exactly the lookup server.py does at run time."""
    ti = real.cm['trace_index']
    checked = files = 0
    for rel, mod_id in ti['py_files'].items():
        path = REPO / rel
        if rel.startswith(('ros2_ws/build/', 'ros2_ws/install/')) or not path.is_file():
            continue                                         # the colcon copies point to the same blocks
        if path.stat().st_mtime >= real.started - 2:
            continue                                         # saved while the tests ran: the map saw an older text
        try:
            code = compile(path.read_text(encoding='utf-8-sig', errors='replace'), rel, 'exec')
        except SyntaxError:
            continue                                         # a file Python cannot compile never runs either
        files += 1
        table = ti['functions'].get(mod_id, {})
        for qual, line in _code_objects(code):
            assert table.get(str(line)) == f'fn:{rel}::{qual}', (rel, line, qual)
            checked += 1
    assert files >= 300 and checked >= 3000, (files, checked)


def test_real_repo_key_facts(real):
    n, e = real.nodes, real.edges
    mc = 'ros2_ws/src/main/src/main_controller.py'
    assert f'fn:{mc}::MainController.image_result_cb' in n
    assert (f'fn:{mc}::MainController.image_result_cb', 'topic:/image_detection', 'subscribes') in e
    assert any(x[1] == 'topic:/image_detection' and x[2] == 'publishes' and 'detection/detection/new_od.py' in x[0]
               for x in e)
    assert ('mod:ros2_ws/src/main/launch/tracker.launch.xml', f'mod:{mc}', 'launches') in e
    assert ('mod:launcher.sh', 'mod:ros2_ws/src/main/launch/tracker.launch.xml', 'runs') in e
    assert ('mod:launcher.sh', 'ext:mavros', 'runs') in e
    assert ('ext:mavros', 'topic:/mavros/state', 'publishes') in e
    assert ('mod:ros2_ws/src/simulation/sim_bringup/launch/sim.launch.py', 'ext:gazebo', 'launches') in e
    assert n['mod:launcher.sh']['parent'] == 'pkg:.' and n['pkg:.']['parent'] == 'sys:team_stack'
    assert n[f'mod:{mc}']['entry_point'] == 'ros2 run main main_controller.py'
    assert real.cm['trace_index']['py_files']['ros2_ws/install/main/lib/main/main_controller.py'] == f'mod:{mc}'
    assert real.cm['trace_index']['py_modules']['sim_bringup.mission_tool'] == \
        'mod:ros2_ws/src/simulation/sim_bringup/sim_bringup/mission_tool.py'
    assert 'mod:LOCAL_NOTES.md' not in n and 'mod:simulation/codemap/codemap.json' not in n
    assert 'fn:simulation/windows/mission_planner_plugin/astra_gcs.cs::AstraPlugin.RunSimAction' in n


def _system_of(nodes, nid):
    while nid and nodes[nid]['kind'] != 'system':
        nid = nodes[nid]['parent']
    return nid


def test_real_repo_architecture_of_today(real):
    """The map shows today's architecture (owner's decisions of 2026-10-31): the new video pipeline (team code, not in
    this repo), last year's photo path marked legacy, run logs, commands and docs, plug-and-play, Jetson in the loop,
    Lattice handed to the team, the Docker doctor. Files that other work is still building are checked when present."""
    n, e = real.nodes, real.edges
    keys = {nid[4:] for nid, x in n.items() if x['kind'] == 'system'}
    assert {'team_stack', 'video_pipeline', 'camera_detection', 'run_logs', 'commands_docs', 'plug_and_play',
            'jetson_hil', 'lattice', 'docker_setup', 'sim_world', 'autopilot', 'codemap'} <= keys
    assert n['sys:camera_detection']['label'].startswith('Legacy (2025) photo pipeline')
    assert 'not in this repo' in n['sys:video_pipeline']['summary'].lower() and 'proposal' in n['sys:video_pipeline']['summary']
    assert 'handed' in n['sys:lattice']['summary'] and 'HANDOFF.md' in n['sys:lattice']['summary']
    for pkg in ('video_cam', 'mapping', 'detection'):
        assert 'legacy 2025 photo pipeline' in n[f'pkg:ros2_ws/src/{pkg}']['label']
        assert _system_of(n, f'pkg:ros2_ws/src/{pkg}') == 'sys:camera_detection'
    # the team's new video pipeline: code not in this repo, drawn as outside programs of their own system
    for ext in ('zr10_node', 'zr10_stream', 'team_detector'):
        assert n[f'ext:{ext}']['parent'] == 'sys:video_pipeline' and 'not in this repo' in n[f'ext:{ext}']['summary'].lower()
    assert ('ext:zr10_stream', 'topic:/zr10/image_raw', 'publishes') in e
    assert ('ext:team_detector', 'topic:/zr10/image_raw', 'subscribes') in e
    assert n['topic:/zr10/image_raw']['parent'] == 'sys:video_pipeline'
    where = {
        'ros2_ws/src/simulation/sim_demo/sim_demo/flight_recorder.py': 'run_logs',
        'ros2_ws/src/simulation/sim_demo/sim_demo/csv_export.py': 'run_logs',
        'ros2_ws/src/simulation/sim_demo/sim_demo/run_summary.py': 'run_logs',
        'simulation/scripts/logs.sh': 'run_logs',
        'docs/simulation/LOGS_AND_DATA.md': 'run_logs',
        'simulation/scripts/astra.sh': 'commands_docs',
        'simulation/scripts/mission.sh': 'commands_docs',
        'ros2_ws/src/simulation/sim_demo/sim_demo/mission_check.py': 'commands_docs',
        'simulation/scripts/check_docs.py': 'commands_docs',
        'docs/README.md': 'commands_docs',
        'docs/simulation/COMMANDS.md': 'commands_docs',
        'docs/simulation/ERRORS.md': 'commands_docs',
        'docs/simulation/SWAPPING.md': 'plug_and_play',
        'ros2_ws/src/simulation/sim_plugins/sim_plugins/profiles.py': 'plug_and_play',
        'simulation/profiles/default.yaml': 'plug_and_play',
        'simulation/docker/check_host.sh': 'docker_setup',
        'simulation/docker/check_host.ps1': 'docker_setup',
        'docs/lattice/HANDOFF.md': 'lattice',
        'simulation/profiles/lattice_mock.yaml': 'lattice',
        'simulation/jetson/jetson.sh': 'jetson_hil',
        'simulation/jetson/lib/hil_relay.py': 'jetson_hil',
        'simulation/profiles/jetson_virtual.yaml': 'jetson_hil',
        'docs/jetson/HIL_DESIGN.md': 'jetson_hil',
        'docs/simulation/VIDEO_PIPELINE.md': 'video_pipeline',
        'ros2_ws/src/simulation/sim_demo/sim_demo/pov_server.py': 'demo_scoring',
        'simulation/windows/mission_planner_plugin/astra_gcs.cs': 'ground_station',
        'ros2_ws/src/simulation/siyi_emulator/siyi_emulator/emulator.py': 'sim_world',
    }
    checked = 0
    for rel, key in where.items():
        path = REPO / rel
        if not path.is_file() or path.stat().st_mtime >= real.started - 2 or f'mod:{rel}' not in n:
            continue                          # still being built (or appeared while the tests ran)
        assert _system_of(n, f'mod:{rel}') == f'sys:{key}', rel
        checked += 1
    assert checked >= 20, checked


def test_real_repo_wiring_view_shows_today_s_wires(real):
    w = real.cm['wiring']
    assert real.report['problems'] == []
    boxes = {b['id']: b for b in w['boxes']}
    wires = {x['id']: x for x in w['wires']}
    want = {'serial2': 'TCP 5763', 'serial0': 'TCP 5760', 'mp_link': 'UDP 14550', 'tool_link': 'UDP 14550',
            'recorder_link': 'UDP 14555', 'lattice_link': 'UDP 14554', 'physics': 'UDP 9002', 'zr10_sdk': 'UDP 37260',
            'rtsp': 'TCP 8554', 'pov_mp': ':8081', 'tracer': ':8091', 'page': ':8090', 'relay_mav': 'SSH -R',
            'relay_cam': 'SSH -R', 'image_raw': '/zr10/image_raw', 'tap': '7461'}
    for wid, words in want.items():
        assert words in wires[wid].get('via', ''), (wid, wires[wid].get('via'))
    status = {'serial2': 'current', 'mp_link': 'current', 'recorder_link': 'current', 'lattice_link': 'optional',
              'zr10_sdk': 'being built', 'rtsp': 'being built', 'relay_mav': 'being built', 'relay_cam': 'being built',
              'adapter_main': 'proposal', 'det_adapter': 'proposal', 'legacy_sdk': 'legacy', 'legacy_http': 'legacy',
              'legacy_trigger': 'legacy', 'legacy_folder': 'legacy', 'legacy_newod': 'legacy'}
    for wid, st in status.items():
        assert wires[wid]['status'] == st, wid
    for bid in ('trigger', 'siyi', 'folder', 'newod'):
        assert boxes[bid]['status'] == 'legacy' and 'LEGACY' in boxes[bid].get('tag', '')
    assert boxes['adapter']['status'] == 'proposal' and boxes['relay']['status'] == 'being built'
    for bid in ('zr10node', 'zr10stream'):
        assert 'NOT IN REPO' in boxes[bid]['tag']
    assert {x['kind'] for x in w['wires']} == set(extract.WIRE_KINDS)
    for x in w['wires']:
        assert x['from'] in boxes and x['to'] in boxes and x['from'] != x['to'], x['id']
    for step in w['steps']:
        assert step['text'] and (step['wires'] or step['boxes'])
        assert all(s in wires for s in step['wires']) and all(b in boxes for b in step['boxes'])
    assert len(w['steps']) >= 10
    for b in w['boxes']:                       # a box whose program is in the map can open it
        if b.get('node'):
            assert b['node'] in real.nodes, b['id']


def test_real_repo_every_wire_port_is_in_the_code_it_names(real):
    """Honesty check: each port number written on a wire (except our proposals) appears, as a whole number, in at
    least one of the repo files the wire lists under "code"; every listed file exists."""
    port = re.compile(r'(?<![\d.])(\d{2,5})(?![\d.])')
    checked = 0
    for x in real.cm['wiring']['wires']:
        ports = [p for p in port.findall(x.get('via', '')) if int(p) >= 80]
        if x['status'] == 'proposal' or not ports:
            continue
        assert x.get('code'), f'wire {x["id"]} shows a port but names no file where it was checked'
        texts = []
        for rel in x['code']:
            assert (REPO / rel).is_file(), (x['id'], rel)
            texts.append((REPO / rel).read_text(encoding='utf-8', errors='replace'))
        for p in ports:
            assert any(re.search(r'(?<![\d.])' + p + r'(?!\d)', t) for t in texts), (x['id'], p, x['code'])
            checked += 1
    assert checked >= 18, checked
