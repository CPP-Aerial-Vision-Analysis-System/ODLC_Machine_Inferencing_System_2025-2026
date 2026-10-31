"""
simulation/docker/test/test_line_endings.py -- PURPOSE: guard the repository's line-ending rules (.gitattributes) that
keep a Windows git clone usable in Linux and Docker. Git for Windows writes CRLF line endings by default; a CRLF
shell script fails with "$'\\r': command not found" and a CRLF "#!/usr/bin/env python3" line (main_controller.py,
the sim_worlds tools) with "python3\\r: No such file or directory". These tests ask git itself (git check-attr,
git ls-files --eol, git grep --cached) and need no Docker. Run from the repository (Linux or Windows):
    python3 -m pytest -q -p no:cacheprovider simulation/docker/test/test_line_endings.py
"""

from pathlib import Path
import subprocess

import pytest


REPO = Path(__file__).resolve().parents[3]


def git(*args):
    """Run git in the repository and return its output (skips the test when git or the checkout is missing)."""
    try:
        result = subprocess.run(['git', '-C', str(REPO), *args], text=True, capture_output=True, timeout=120)
    except FileNotFoundError:
        pytest.skip('git is not installed')
    if result.returncode != 0:
        pytest.skip(f'not a usable git checkout: {result.stderr.strip()[:100]}')
    return result.stdout


def attrs(path):
    """{attribute: value} of git check-attr text eol for PATH."""
    out = git('check-attr', 'text', 'eol', '--', path)
    return {line.split(': ')[1]: line.split(': ')[2] for line in out.splitlines()}


@pytest.mark.parametrize('path', [
    'ros2_ws/src/main/src/main_controller.py',               # started through its "#!" line by ros2 launch
    'ros2_ws/src/simulation/sim_worlds/scripts/make_mission.py',
    'docker/Dockerfile',
    'launcher.sh',
    'simulation/docker/check_host.sh',
])
def test_linux_files_check_out_with_lf_everywhere(path):
    """Every text file used in Linux is written with LF, also where core.autocrlf=true (Git for Windows)."""
    a = attrs(path)
    assert a['eol'] == 'lf', a
    assert a['text'] in ('set', 'auto'), a


def test_windows_launchers_keep_crlf():
    """Double-click .bat launchers stay CRLF (cmd.exe needs it)."""
    assert attrs('simulation/windows/Start Search Demo.bat')['eol'] == 'crlf'


def test_binary_files_are_never_converted():
    """Pictures, PDFs and model weights are binary: no line-ending conversion can damage them."""
    for path in ('docs/example.png', 'x/photo.jpg', 'x/manual.pdf', 'x/weights.pt', 'x/model.onnx'):
        assert attrs(path)['text'] == 'unset', path


def test_no_shell_script_is_committed_with_crlf():
    """A shell script committed with CRLF would be CRLF in every clone, even on Linux."""
    bad = [line for line in git('ls-files', '--eol', '--', '*.sh', '*.bash').splitlines()
           if line.split()[0] in ('i/crlf', 'i/mixed')]
    assert bad == []


def test_every_lf_script_gets_lf_on_checkout():
    """Each committed file with a '#!' line that is stored with LF is checked out with LF on every computer."""
    scripts = [p for p in git('grep', '--cached', '-I', '-l', '-e', '^#!', '--', '.', ':(exclude)*.md',
                              ':(exclude)*.txt', ':(exclude)*.html', ':(exclude)*.mhtml').splitlines() if p]
    assert scripts, 'no scripts found'
    wrong = []
    for line in git('ls-files', '--eol', '--', *scripts).splitlines():
        meta, path = line.split('\t', 1)
        if meta.split()[0] == 'i/lf' and 'eol=lf' not in meta:
            wrong.append(path)
    assert wrong == []
