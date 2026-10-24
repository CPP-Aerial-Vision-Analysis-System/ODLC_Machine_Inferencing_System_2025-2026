"""
lattice_bridge/test/support.py -- PURPOSE: small helpers shared by the lattice_bridge tests: the fixed test
time, the golden-file comparison, the scenario data loader and a deterministic random generator of valid
inputs. Nothing here is part of the library.

Golden files (test/golden/*.json) are complete, hand-reviewed example requests. A golden test builds the same
entity again and demands an exact match. When a change is INTENDED, regenerate with

    LATTICE_UPDATE_GOLDEN=1 python3 -m pytest -q test/test_golden.py

and then review `git diff test/golden` against the Lattice docs before committing: the diff is the review.
"""

import json
import os
import random
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
GOLDEN = HERE / 'golden'

NOW = datetime(2026, 10, 24, 15, 30, 0, tzinfo=timezone.utc)
MISSION = 'runway20261024t153000z'
HOUR = timedelta(hours=1)


def assert_golden(name: str, entity) -> None:
    """Compare `entity` with golden/<name>.json (or rewrite it when LATTICE_UPDATE_GOLDEN=1)."""
    path = GOLDEN / f'{name}.json'
    actual = json.loads(json.dumps(entity, allow_nan=False))
    if os.environ.get('LATTICE_UPDATE_GOLDEN') == '1':
        GOLDEN.mkdir(exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='\n') as handle:
            handle.write(json.dumps(actual, indent=2) + '\n')
        return
    if not path.exists():
        pytest.fail(f'golden file {path.name} is missing; run with LATTICE_UPDATE_GOLDEN=1 and review it')
    expected = json.loads(path.read_text(encoding='utf-8'))
    assert actual == expected, f'{path.name} differs from the builder output (review, then regenerate)'


def scenarios() -> dict:
    """The committed copy of the simulator worlds' true positions (see scenario_data.py)."""
    from scenario_data import load_fixture
    return load_fixture()


def rng(seed: int = 1234) -> random.Random:
    """A seeded random generator: property tests are random but repeatable."""
    return random.Random(seed)


def random_position(r: random.Random):
    """A random valid (lat, lon) away from (0, 0) and from the poles."""
    return r.uniform(-80.0, 80.0) or 1.0, r.uniform(-179.0, 179.0) or 1.0
