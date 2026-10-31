"""
simulation/codemap/test/conftest.py -- PURPOSE: shared pytest set-up for the code-map tests (pytest reads this
file by itself before the test files in this folder). It does two small things:
  1. puts simulation/codemap on Python's import path, so a test can simply write `import extract` (or
     `import server`, `import build_artifact`) instead of loading the file by its path;
  2. stops Python from writing __pycache__ folders into the repository while the tests run
     (the same as running with PYTHONDONTWRITEBYTECODE=1).
It deliberately does NOT put simulation/codemap/trace on the import path: that folder holds the tracer's
sitecustomize.py, which must only ever be loaded on purpose (ASTRA_TRACE=1), never by accident.

Run the tests (Windows or Ubuntu, Python 3.10+, no ROS needed), from the repo root:
    python -m pytest -q -p no:cacheprovider simulation/codemap/test
"""

import sys
from pathlib import Path

sys.dont_write_bytecode = True

CODEMAP_DIR = Path(__file__).resolve().parents[1]      # simulation/codemap
if str(CODEMAP_DIR) not in sys.path:
    sys.path.insert(0, str(CODEMAP_DIR))
