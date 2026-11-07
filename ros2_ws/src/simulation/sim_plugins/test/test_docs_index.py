"""
sim_plugins/test/test_docs_index.py -- PURPOSE: keep the repo's documentation findable. It runs the docs checker
simulation/scripts/check_docs.py on the real repo, so pytest and `colcon test` fail when
  * a Markdown file is missing from the documentation index docs/README.md,
  * a link in the index, or a relative link inside any doc, points to a file (or #section) that does not exist,
  * a doc names a script under simulation/scripts/, simulation/jetson/ (or setup/, docker/, windows/) that
    does not exist.
It also tests the checker itself on small pretend repos in a temporary folder (each rule fails when it should,
a planned doc or script under "## Being written" does not, code examples and allow-listed files are skipped).

If the first test fails: read its message (each line says the file, what is wrong and how to fix it), fix the
doc or docs/README.md, and run  python3 simulation/scripts/check_docs.py  until it prints "[check_docs] OK".
Needs only the Python standard library and pytest (no ROS); it lives next to test_doc_blocks.py, the other
doc test of this package.
"""

import importlib.util
import subprocess
import sys

import pytest

from conftest import REPO

CHECKER = REPO / 'simulation' / 'scripts' / 'check_docs.py'


def _load_checker():
    spec = importlib.util.spec_from_file_location('check_docs', CHECKER)
    module = importlib.util.module_from_spec(spec)
    sys.modules['check_docs'] = module          # dataclasses look their module up while the class is made
    spec.loader.exec_module(module)
    return module


check_docs = _load_checker()


# ------------------------------------------------------------------------------------------- the real repo

def test_the_real_repo_docs_are_indexed_and_links_work():
    assert CHECKER.is_file(), 'simulation/scripts/check_docs.py is missing'
    assert (REPO / 'docs' / 'README.md').is_file(), 'the documentation index docs/README.md is missing'
    report = check_docs.check(REPO)
    assert report.ok, ('the docs checker found problems (fix them, then run python3 '
                       'simulation/scripts/check_docs.py):\n' + '\n'.join(p.text() for p in report.problems))
    assert len(report.docs) >= 50, f'only {len(report.docs)} Markdown files found: is REPO right? ({REPO})'


def test_the_command_line_says_ok_and_exits_0():
    result = subprocess.run([sys.executable, str(CHECKER)], capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.strip().splitlines()[-1].startswith('[check_docs] OK:')


# ------------------------------------------------------------------------------------------- a pretend repo

def _repo(tmp_path, index_body, files=None):
    """A tiny repo (no git: the checker walks the folder) with docs/README.md = index_body plus `files`."""
    (tmp_path / 'simulation' / 'scripts').mkdir(parents=True)
    (tmp_path / 'ros2_ws').mkdir()
    (tmp_path / 'docs').mkdir()
    (tmp_path / 'simulation' / 'scripts' / 'demo.sh').write_text('#!/bin/bash\n', encoding='utf-8')
    (tmp_path / 'docs' / 'README.md').write_text('# Index\n\n' + index_body, encoding='utf-8')
    for rel, text in (files or {}).items():
        path = tmp_path / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding='utf-8')
    return tmp_path


def _problems(root):
    return [p.text() for p in check_docs.check(root).problems]


def test_a_complete_index_passes(tmp_path):
    root = _repo(tmp_path, '* [`README.md`](../README.md)\n* [guide](guide.md#run-it)\n',
                 {'README.md': '# Repo\nSee [the guide](docs/guide.md).\n',
                  'docs/guide.md': '# Guide\n\n## Run it\n\nbash simulation/scripts/demo.sh\n'})
    assert _problems(root) == []


def test_a_doc_missing_from_the_index_fails(tmp_path):
    root = _repo(tmp_path, '* [`README.md`](../README.md)\n',
                 {'README.md': '# Repo\n', 'docs/forgotten.md': '# Nobody links me\n'})
    problems = _problems(root)
    assert len(problems) == 1
    assert 'docs/forgotten.md' in problems[0] and 'not listed in the documentation index' in problems[0]
    assert 'Fix:' in problems[0] and 'Likely cause:' in problems[0]


def test_a_broken_link_in_the_index_fails(tmp_path):
    root = _repo(tmp_path, '* [`README.md`](../README.md)\n* [old](simulation/OLD.md)\n', {'README.md': '# R\n'})
    problems = _problems(root)
    assert len(problems) == 1 and "'simulation/OLD.md'" in problems[0] and 'does not exist' in problems[0]


def test_a_broken_relative_link_inside_a_doc_fails(tmp_path):
    root = _repo(tmp_path, '* [`README.md`](../README.md)\n',
                 {'README.md': '# R\n\nRead [the setup](docs/SETUP.md) and [this](#nowhere).\n'})
    problems = _problems(root)
    assert any('README.md:3' in p and "'docs/SETUP.md'" in p for p in problems), problems
    assert any("'#nowhere'" in p and 'section' in p for p in problems), problems
    assert len(problems) == 2


def test_a_planned_doc_under_being_written_does_not_fail(tmp_path):
    index = ('* [`README.md`](../README.md)\n\n## Being written\n\n* [`docs/jetson/README.md`](jetson/README.md)\n'
             '\n## Keeping the docs up to date\n\n* [rules](#being-written)\n')
    root = _repo(tmp_path, index, {'README.md': '# R\n'})
    assert _problems(root) == []
    # ... but the same missing link OUTSIDE that section is an error
    root2 = _repo(tmp_path / 'b', '* [`docs/jetson/README.md`](jetson/README.md)\n* [r](../README.md)\n',
                  {'README.md': '# R\n'})
    assert len(_problems(root2)) == 1


def test_another_doc_may_link_a_planned_doc_while_it_is_written(tmp_path):
    files = {'README.md': '# R\n', 'docs/simulation/PLUGINS.md': '# P\nThe short way: [SWAPPING](SWAPPING.md).\n'}
    index = ('* [r](../README.md) [p](simulation/PLUGINS.md)\n\n## Being written\n\n'
             '* [`docs/simulation/SWAPPING.md`](simulation/SWAPPING.md)\n')
    report = check_docs.check(_repo(tmp_path, index, files))
    assert report.ok, [p.text() for p in report.problems]
    assert any('docs/simulation/PLUGINS.md:2' in n and 'docs/simulation/SWAPPING.md' in n for n in report.notes)
    # ... and once the index no longer lists it under "Being written", the same link is an error again
    root2 = _repo(tmp_path / 'b', '* [r](../README.md) [p](simulation/PLUGINS.md)\n', files)
    problems = _problems(root2)
    assert len(problems) == 1 and "'SWAPPING.md'" in problems[0] and 'Being written' in problems[0], problems


def test_a_planned_doc_that_appeared_counts_as_indexed_with_a_note(tmp_path):
    index = '* [r](../README.md)\n\n## Being written\n\n* [`docs/jetson/LINK.md`](jetson/LINK.md)\n'
    root = _repo(tmp_path, index, {'README.md': '# R\n', 'docs/jetson/LINK.md': '# Link\n'})
    report = check_docs.check(root)
    assert report.ok and any('docs/jetson/LINK.md now exists' in n for n in report.notes)


def test_a_named_script_that_does_not_exist_fails(tmp_path):
    root = _repo(tmp_path, '* [r](../README.md)\n',
                 {'README.md': '# R\n\n```bash\nbash simulation/scripts/demo.sh\n'
                               'bash simulation/jetson/flash_jetson.sh\n```\n'
                               'or `simulation\\scripts\\nothere.ps1`.\n'})
    problems = _problems(root)
    assert any('simulation/jetson/flash_jetson.sh' in p for p in problems), problems
    assert any('simulation/scripts/nothere.ps1' in p for p in problems), problems
    assert not any('demo.sh' in p for p in problems) and len(problems) == 2


def test_a_planned_script_under_being_written_does_not_fail(tmp_path):
    index = ('* [r](../README.md)\n\n## Being written\n\n* the Jetson kit: `bash simulation/jetson/jetson.sh check`\n'
             '\n## Keeping the docs up to date\n\nRun `bash simulation/jetson/jetson.sh check`.\n')
    root = _repo(tmp_path, index, {'README.md': '# R\n'})
    problems = _problems(root)
    # only the mention OUTSIDE "Being written" (the last line of the index) is an error
    assert len(problems) == 1 and 'simulation/jetson/jetson.sh' in problems[0], problems
    assert 'docs/README.md:11' in problems[0], problems


def test_links_in_code_and_comments_and_web_links_are_not_checked(tmp_path):
    body = ('# R\n<!-- [hidden](nothere.md) -->\n`[code](nothere.md)`\n\n```\n[fenced](nothere.md)\n```\n'
            '[web](https://example.com/x.md) [mail](mailto:a@example.com)\n')
    root = _repo(tmp_path, '* [r](../README.md)\n', {'README.md': body})
    assert _problems(root) == []


def test_allow_listed_and_gitignored_files_need_no_index_line(tmp_path):
    root = _repo(tmp_path, '* [r](../README.md)\n',
                 {'README.md': '# R\n',
                  '.gitignore': 'LOCAL_NOTES.md\nros2_ws/build/\nGUIDE.md\n',
                  'LOCAL_NOTES.md': '[broken](nothere.md)\n',
                  'notes/GUIDE.md': '# ignored by .gitignore\n',
                  'ros2_ws/src/ultralytics_ros/README.md': '[upstream](docs/nothere.md)\n',
                  'ros2_ws/build/pkg/README.md': '# build output\n'})
    assert _problems(root) == []


def test_links_with_spaces_and_anchors_resolve(tmp_path):
    index = '* [notes](../things%20to%20test.md) [s](<../things to test.md#step-1>) [a](../A.md#31-hosts-and-links)\n'
    root = _repo(tmp_path, index, {'things to test.md': '# T\n## Step 1\n',
                                   'A.md': '# A\n### 3.1 Hosts and links\n'})
    assert _problems(root) == []


@pytest.mark.parametrize('heading, slug', [
    ('3. How to check your work', '3-how-to-check-your-work'),
    ('4.7 demo.sh, run folders, score report', '47-demosh-run-folders-score-report'),
    ('Do I need a `docker login`?', 'do-i-need-a-docker-login'),
    ('The [plug-in](PLUGINS.md) guide', 'the-plug-in-guide'),
])
def test_heading_anchors_follow_github(heading, slug):
    assert check_docs.slugify(heading) == slug


def test_repeated_headings_get_numbered_anchors():
    anchors = check_docs.anchors_of('# A\n## Run\n## Run\n<a id="no-login"></a>\n')
    assert {'a', 'run', 'run-1', 'no-login'} <= anchors


def test_missing_index_exits_2(tmp_path, capsys):
    (tmp_path / 'simulation').mkdir()
    (tmp_path / 'ros2_ws').mkdir()
    assert check_docs.main(['--root', str(tmp_path)]) == 2
    err = capsys.readouterr().err
    assert 'ERROR' in err and 'Likely cause' in err and 'Fix:' in err
