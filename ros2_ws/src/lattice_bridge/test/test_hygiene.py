"""
lattice_bridge/test/test_hygiene.py -- PURPOSE: the rules of this package that are about the CODE and the FILES,
not about behaviour, checked mechanically so they cannot rot:

  * flight safety: the core library imports only the Python standard library, and the pure modules do no I/O,
    no printing, no sleeping, no threads and never read the clock;
  * team rules: every file starts with a purpose paragraph;
  * public repo: no tokens, personal e-mail addresses or machine-specific paths in any file of the package
    or in docs/lattice;
  * packaging: package.xml, setup.py, the resource marker and __version__ agree, and every sub-package has an
    __init__.py (otherwise colcon would silently leave it out of the install).
"""

import ast
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
LIB = PKG / 'lattice_bridge'
REPO_DOCS = PKG.parents[2] / 'docs' / 'lattice'

PURE_MODULES = ['constants', 'timeutil', 'geo', 'ids', 'entities', 'clustering', 'report_codec', 'world_areas',
                'reporter_logic']
STDLIB_FOR_PURE = {'__future__', 'math', 'numbers', 're', 'json', 'dataclasses', 'datetime', 'typing'}

#: The PROGRAMS (phase 2) talk to ROS or to MAVLink, so each may import exactly the extra modules listed for it
#: here (key = path inside the lattice_bridge/ folder, e.g. 'reporter_node.py'). Every other file is the core library and may
#: import the Python standard library only. Add a line here, with a reason, when a new program needs ROS.
OUTSIDE_IMPORTS_ALLOWED = {
    # the Jetson-side target reporter: ROS 2 node, the detector's message, the MAVROS text message, and the
    # team's helper that reads another node's parameters without blocking
    'reporter_node.py': {'rclpy', 'interfaces', 'mavros_msgs', 'rcl_interfaces', 'wp_sender'},
    # the bridge's settings file reader: PyYAML (part of ROS 2 Humble and of the Jetson image)
    'bridge_config.py': {'yaml'},
    # the ground-side MAVLink reader: pymavlink (installed with MAVProxy; imported only here)
    'sources/mavlink_source.py': {'pymavlink'},
    # the bridge reading ROS topics on one machine (for a bridge that later runs on the Jetson)
    'sources/ros_source.py': {'rclpy', 'mavros_msgs', 'sensor_msgs', 'std_msgs'},
}


def lib_files():
    return sorted(LIB.rglob('*.py'))


def imported_top_level_names(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name.split('.')[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split('.')[0])
    return names


# ------------------------------------------------------------------------------------ no new dependencies
@pytest.mark.parametrize('path', lib_files(), ids=lambda p: str(p.relative_to(PKG)))
def test_the_library_imports_only_the_standard_library(path):
    stdlib = set(sys.stdlib_module_names)
    allowed = OUTSIDE_IMPORTS_ALLOWED.get(path.relative_to(LIB).as_posix(), set())
    outside = {n for n in imported_top_level_names(path) if n not in stdlib and n not in allowed}
    assert not outside, f'{path.name} imports non-standard-library modules: {sorted(outside)}'


@pytest.mark.parametrize('name', PURE_MODULES)
def test_pure_modules_use_only_a_tiny_set_of_modules(name):
    extra = imported_top_level_names(LIB / f'{name}.py') - STDLIB_FOR_PURE
    assert not extra, f'{name}.py imports {sorted(extra)}'


@pytest.mark.parametrize('name', PURE_MODULES)
def test_pure_modules_do_no_io_printing_sleeping_or_clock_reads(name):
    tree = ast.parse((LIB / f'{name}.py').read_text(encoding='utf-8'))
    banned_names = {'open', 'print', 'input', 'exec', 'eval', 'compile', '__import__', 'breakpoint'}
    banned_attrs = {'now', 'utcnow', 'today', 'sleep', 'time', 'monotonic', 'perf_counter', 'getenv', 'environ',
                    'read_text', 'write_text', 'urlopen', 'system', 'popen', 'Popen'}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            func = node.func
            if isinstance(func, ast.Name):
                assert func.id not in banned_names, f'{name}.py line {node.lineno}: {func.id}()'
            elif isinstance(func, ast.Attribute):
                assert func.attr not in banned_attrs, f'{name}.py line {node.lineno}: .{func.attr}()'


def test_importing_the_package_loads_no_ros_network_or_third_party_modules():
    """A fresh interpreter that imports lattice_bridge must not have pulled in rclpy, YAML, numpy, sockets ..."""
    code = '\n'.join([
        'import sys, lattice_bridge',
        'bad = {"rclpy", "yaml", "numpy", "pymavlink", "mavros_msgs", "cv2", "torch", "requests", "socket",',
        '       "subprocess", "multiprocessing", "asyncio", "ssl"}',
        'found = sorted(bad & set(sys.modules))',
        'print("LOADED", found)',
        'sys.exit(1 if found else 0)',
    ])
    env = dict(os.environ)
    if os.environ.get('LATTICE_BRIDGE_USE_INSTALLED') != '1':
        env['PYTHONPATH'] = os.pathsep.join([str(PKG), env.get('PYTHONPATH', '')])
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, env=env,
                            cwd=str(HERE))
    assert result.returncode == 0, result.stdout + result.stderr


# ------------------------------------------------------------------------------------ purpose paragraphs
def purpose_files():
    files = [p for p in PKG.rglob('*') if p.is_file() and '__pycache__' not in p.parts
             and p.suffix in {'.py', '.md', '.yaml', '.yml', '.xml', '.cfg'}]
    files.append(PKG / 'resource' / 'lattice_bridge')
    return sorted(set(files))


@pytest.mark.parametrize('path', purpose_files(), ids=lambda p: str(p.relative_to(PKG)))
def test_every_file_starts_with_a_purpose_paragraph(path):
    if path.name == 'lattice_bridge' and path.parent.name == 'resource':
        pytest.skip('the ament resource marker must stay empty')
    head = ''.join(path.read_text(encoding='utf-8').splitlines(keepends=True)[:14])
    assert 'PURPOSE' in head, f'{path.relative_to(PKG)} has no PURPOSE paragraph in its first lines'


@pytest.mark.parametrize('path', [p for p in lib_files() + sorted(HERE.glob('*.py'))],
                         ids=lambda p: str(p.relative_to(PKG)))
def test_python_files_have_a_module_docstring_that_names_the_file(path):
    doc = ast.get_docstring(ast.parse(path.read_text(encoding='utf-8'))) or ''
    assert doc.startswith(f'lattice_bridge/{path.relative_to(PKG).as_posix()} -- PURPOSE:') or \
        doc.startswith(f'{path.relative_to(PKG).as_posix()} -- PURPOSE:'), path.name


@pytest.mark.parametrize('path', lib_files(), ids=lambda p: str(p.relative_to(PKG)))
def test_public_functions_and_classes_have_docstrings(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and not node.name.startswith('_'):
            assert ast.get_docstring(node), f'{path.name}: {node.name} has no docstring'


# ------------------------------------------------------------------------------------ public repo
FORBIDDEN = [
    (r'[A-Za-z]:\\(?:Users|suas_scratch|Projects)', 'a Windows path'),
    (r'/mnt/[a-z](?:/|$)', 'a WSL drive path'),
    (r'/home/(?!user\b|<)[a-z0-9_]+/', 'a personal home folder'),
    (r'(?i)jarrod', 'a personal name'),
    (r'@(?!example\.com\b)[A-Za-z0-9-]+\.(?:com|org|net|edu|io)\b', 'an e-mail address other than @example.com'),
    (r'Bearer\s+[A-Za-z0-9._~+/=-]{16,}', 'a bearer token'),
    (r'eyJ[A-Za-z0-9_-]{10,}', 'a JWT'),
    # (an ALL-CAPS name with an underscore, such as 'LATTICE_TOKEN', is the NAME of an environment variable)
    (r'(?i)(?:token|secret|password|api[_-]?key)\s*[:=]\s*["\'](?!(?-i:[A-Z][A-Z0-9]*_[A-Z0-9_]+)["\'])[^"\'$<{]{8,}["\']', 'a literal secret'),
    (r'(?i)\b[0-9a-f]{32,}\b', 'a long hex string (key?)'),
]


def public_files():
    roots = [PKG] + ([REPO_DOCS] if REPO_DOCS.is_dir() else [])
    files = []
    for root in roots:
        files += [p for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts
                  and p.suffix in {'.py', '.md', '.yaml', '.yml', '.xml', '.cfg', '.json', '.txt', '.sh'}]
    return sorted(files)


def test_there_are_files_to_scan():
    assert len(public_files()) > 20


@pytest.mark.parametrize('pattern, what', FORBIDDEN)
def test_no_secrets_personal_data_or_machine_paths_in_committed_files(pattern, what):
    regex = re.compile(pattern)
    offenders = []
    for path in public_files():
        if path.name == 'test_hygiene.py':
            continue                                      # this file spells the patterns out
        for number, line in enumerate(path.read_text(encoding='utf-8', errors='replace').splitlines(), start=1):
            if regex.search(line):
                offenders.append(f'{path.name}:{number}')
    assert not offenders, f'{what} found in {offenders[:8]}'


def test_the_maintainer_address_is_the_shared_placeholder():
    text = (PKG / 'package.xml').read_text(encoding='utf-8')
    assert 'astra-sim@example.com' in text
    assert 'astra-sim@example.com' in (PKG / 'setup.py').read_text(encoding='utf-8')


def test_secrets_are_only_ever_named_never_valued():
    """The docs may mention LATTICE_TOKEN and friends by name; no file may assign them a value."""
    pattern = re.compile(r'(LATTICE_TOKEN|SANDBOXES_TOKEN|ENVIRONMENT_TOKEN|LATTICE_ENDPOINT)\s*=\s*[^\s<$"\',]')
    for path in public_files():
        if path.name == 'test_hygiene.py':
            continue
        for number, line in enumerate(path.read_text(encoding='utf-8', errors='replace').splitlines(), start=1):
            assert not pattern.search(line), f'{path.name}:{number}'


# ------------------------------------------------------------------------------------ packaging
def test_package_xml_setup_py_and_version_agree():
    root = ET.parse(PKG / 'package.xml').getroot()
    assert root.findtext('name') == 'lattice_bridge'
    xml_version = root.findtext('version')
    setup_text = (PKG / 'setup.py').read_text(encoding='utf-8')
    assert f"version='{xml_version}'" in setup_text
    init_text = (LIB / '__init__.py').read_text(encoding='utf-8')
    assert f"__version__ = '{xml_version}'" in init_text
    assert root.find('export/build_type').text == 'ament_python'
    assert root.find('buildtool_depend').text == 'ament_python'


#: ROS packages the phase 2 programs may name in package.xml (all part of ROS 2 Humble or of this repo). Nothing
#: that needs pip or apt beyond ROS itself belongs here: that is the "no new dependencies" rule.
ALLOWED_EXEC_DEPENDS = {'rclpy', 'interfaces', 'mavros_msgs', 'main', 'wp_sender', 'launch', 'launch_ros',
                        'ament_index_python'}


def test_package_xml_adds_no_runtime_dependencies():
    root = ET.parse(PKG / 'package.xml').getroot()
    assert not root.findall('depend') and not root.findall('build_depend')
    assert {e.text for e in root.findall('exec_depend')} <= ALLOWED_EXEC_DEPENDS
    assert {e.text for e in root.findall('test_depend')} <= {'python3-pytest', 'python3-yaml'}


def test_resource_marker_and_data_files_exist():
    assert (PKG / 'resource' / 'lattice_bridge').is_file()
    assert (PKG / 'README.md').is_file()
    setup_text = (PKG / 'setup.py').read_text(encoding='utf-8')
    for name in ("'resource/' + package_name", "'package.xml'", "'README.md'", "'/launch'", "'/config'"):
        assert name in setup_text


def test_every_package_folder_has_an_init_file():
    for folder in {p.parent for p in LIB.rglob('*.py')}:
        assert (folder / '__init__.py').is_file(), f'{folder.relative_to(PKG)} has no __init__.py'


#: The only programs this package installs (console scripts). Each is OFF unless somebody starts it.
ALLOWED_CONSOLE_SCRIPTS = {'lattice_reporter'}


def test_only_the_known_programs_are_installed_and_nothing_launches_the_bridge():
    setup_text = (PKG / 'setup.py').read_text(encoding='utf-8')
    block = re.search(r"'console_scripts':\s*\[(.*?)\]", setup_text, flags=re.S).group(1)
    names = {item.split('=')[0].strip() for item in re.findall(r"'([^']+)'", block)}
    assert names <= ALLOWED_CONSOLE_SCRIPTS, f'unexpected programs: {sorted(names - ALLOWED_CONSOLE_SCRIPTS)}'
    repo = PKG.parents[2]
    for launch in (repo / 'launcher.sh', repo / 'ros2_ws' / 'src' / 'main' / 'launch' / 'tracker.launch.xml'):
        if launch.is_file():
            assert 'lattice' not in launch.read_text(encoding='utf-8', errors='replace').lower(), launch.name


def test_the_installed_copy_is_the_one_under_test_when_asked():
    if os.environ.get('LATTICE_BRIDGE_USE_INSTALLED') != '1':
        pytest.skip('source-tree run')
    import lattice_bridge
    location = Path(lattice_bridge.__file__).resolve()
    assert PKG not in location.parents, f'expected the INSTALLED package, got {location}'
    assert 'install' in location.parts
