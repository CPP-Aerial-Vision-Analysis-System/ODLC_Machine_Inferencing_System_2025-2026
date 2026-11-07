"""
sim_plugins/test/test_team_repo.py -- PURPOSE: unit tests of the check of ANOTHER COPY of the team's Jetson code
(team_repo.py, profile key run.team_repo / demo.sh --team-repo): a missing folder, a folder with a space, no
launcher.sh, an old launcher without TRACKER_CMD / FCU_URL, an unbuilt or half-built ros2_ws and a missing
tracker.launch.xml each stop the run with the fix; a renamed node or a changed status text is a warning; and the
REAL team sources in this repo contain every node name and status text the simulator watches (so the lists in
team_repo.py cannot drift away from the team code without a failing test).
"""

from pathlib import Path

from sim_plugins import team_repo as tr

REPO = Path(__file__).resolve().parents[5]

LAUNCHER = '#!/bin/bash\nFCU_URL="${FCU_URL:-x}"\nTRACKER_CMD="${TRACKER_CMD:-ros2 launch main tracker.launch.xml}"\n'


def fake_checkout(root: Path, launcher: str = LAUNCHER, built=tr.TEAM_PACKAGES, node_names: bool = True) -> Path:
    """A minimal checkout: launcher.sh, ros2_ws/install/<pkg>, tracker.launch.xml and package sources."""
    (root / 'ros2_ws' / 'src' / 'main' / 'launch').mkdir(parents=True)
    (root / 'launcher.sh').write_text(launcher, encoding='utf-8')
    (root / 'ros2_ws' / 'src' / 'main' / 'launch' / 'tracker.launch.xml').write_text('<launch/>', encoding='utf-8')
    install = root / 'ros2_ws' / 'install'
    install.mkdir(parents=True)
    (install / 'setup.bash').write_text('# colcon\n', encoding='utf-8')
    for pkg in built:
        (install / pkg).mkdir()
    texts = {pkg: [] for pkg in tr.NODE_OF_PACKAGE}
    for needle, pkg, _ in tr.STATUS_TEXTS:
        texts.setdefault(pkg, []).append(needle)
    for pkg, node in tr.NODE_OF_PACKAGE.items():
        folder = root / 'ros2_ws' / 'src' / pkg / pkg
        folder.mkdir(parents=True, exist_ok=True)
        name = node if node_names else node + '_v2'
        body = f"super().__init__('{name}')\n" + ''.join(f"log('{t} ...')\n" for t in texts.get(pkg, []))
        (folder / 'node.py').write_text(body, encoding='utf-8')
    return root


def test_empty_setting_checks_nothing():
    assert tr.check('') == ([], [], [])


def test_missing_folder_says_how_to_clone(tmp_path):
    errors, _, _ = tr.check(str(tmp_path / 'astra_fork'))
    assert len(errors) == 1 and 'does not exist' in errors[0] and 'git clone' in errors[0]


def test_space_in_folder_is_an_error(tmp_path):
    root = fake_checkout(tmp_path / 'my fork')
    errors, _, _ = tr.check(str(root))
    assert any('contains a space' in e and 'Fix:' in e for e in errors)


def test_a_complete_checkout_passes(tmp_path):
    errors, warnings, notes = tr.check(str(fake_checkout(tmp_path / 'fork')))
    assert errors == [] and warnings == []
    assert 'team code from' in notes[0]


def test_no_launcher(tmp_path):
    root = fake_checkout(tmp_path / 'fork')
    (root / 'launcher.sh').unlink()
    errors, _, _ = tr.check(str(root))
    assert any('has no launcher.sh' in e for e in errors)


def test_an_old_launcher_without_the_simulator_hooks(tmp_path):
    root = fake_checkout(tmp_path / 'fork', launcher='#!/bin/bash\nros2 launch main tracker.launch.xml\n')
    errors, _, _ = tr.check(str(root))
    assert any('TRACKER_CMD' in e and 'cp ' in e for e in errors)
    assert any('FCU_URL' in e for e in errors)


def test_unbuilt_and_half_built_workspaces(tmp_path):
    root = fake_checkout(tmp_path / 'a')
    (root / 'ros2_ws' / 'install' / 'setup.bash').unlink()
    errors, _, _ = tr.check(str(root))
    assert any('is not built' in e and 'colcon build' in e for e in errors)
    root = fake_checkout(tmp_path / 'b', built=('main', 'interfaces'))
    errors, _, _ = tr.check(str(root))
    assert any('has no video_cam' in e and '--packages-up-to' in e for e in errors)


def test_missing_tracker_launch(tmp_path):
    root = fake_checkout(tmp_path / 'fork')
    (root / 'ros2_ws' / 'src' / 'main' / 'launch' / 'tracker.launch.xml').unlink()
    errors, _, _ = tr.check(str(root))
    assert any('tracker.launch.xml is missing' in e for e in errors)


def test_renamed_nodes_are_warnings(tmp_path):
    errors, warnings, _ = tr.check(str(fake_checkout(tmp_path / 'fork', node_names=False)))
    assert errors == []
    assert any('"main_controller" was not found' in w and 'stack_check' in w for w in warnings)


def test_the_real_team_sources_have_every_node_name_and_status_text():
    _, warnings, _ = tr.check(str(REPO))
    about_sources = [w for w in warnings if 'was not found in' in w]
    assert about_sources == [], about_sources
