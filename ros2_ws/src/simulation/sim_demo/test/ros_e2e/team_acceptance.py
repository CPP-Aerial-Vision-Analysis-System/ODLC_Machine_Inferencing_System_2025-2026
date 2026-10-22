"""
sim_demo/test/ros_e2e/team_acceptance.py -- PURPOSE: prove that what truth_detector publishes is
accepted by the TEAM'S OWN mission controller code, not just by our reading of it. It loads the
team's main/src/main_controller.py (read-only, from the repo) and runs its real
MainController.image_result_cb on each /image_detection message, using a tiny stand-in for
`self` (constructing the real node would block until MAVROS and waypoint_manager services
exist). Afterwards `best()` tells what main_controller would fly to.

Only `wp_sender.parameter` is stubbed if the team's wp_sender package is not built here (the
callback never uses it). Usage: see fake_mission.py.
"""
from __future__ import annotations

import importlib.util
import os
import sys
import types

# The repo root is 6 folders above this file (ros_e2e/test/sim_demo/simulation/src/ros2_ws), so the
# helper works from any clone; ASTRA_TEAM_REPO still overrides it.
DEFAULT_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), *(['..'] * 6)))


def load_main_controller(repo: str = ''):
    """Import the team's main_controller.py as a module (stubbing wp_sender if needed)."""
    repo = repo or os.environ.get('ASTRA_TEAM_REPO', DEFAULT_REPO)
    path = os.path.join(repo, 'ros2_ws', 'src', 'main', 'src', 'main_controller.py')
    try:
        import wp_sender.parameter  # noqa: F401 - the real one, if built
    except ImportError:
        pkg = types.ModuleType('wp_sender')
        mod = types.ModuleType('wp_sender.parameter')

        class ParameterManager:  # never used by image_result_cb
            pass

        mod.ParameterManager = ParameterManager
        pkg.parameter = mod
        sys.modules['wp_sender'] = pkg
        sys.modules['wp_sender.parameter'] = mod
    spec = importlib.util.spec_from_file_location('team_main_controller', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TeamCallbackRunner:
    """Feeds messages to the team's real image_result_cb and remembers what it decided."""

    def __init__(self, repo: str = ''):
        self.mc = load_main_controller(repo)
        self.acks: list[str] = []
        self.logs: list[str] = []
        logs = self.logs
        acks = self.acks

        class _Logger:
            def info(self, text):
                logs.append(str(text))

            def warn(self, text):
                logs.append(str(text))

            warning = warn
            error = warn

        class _Self:
            pass

        self.fake = _Self()
        self.fake.processed_image_names = set()
        self.fake.detections = {
            'person': self.mc.Detection_Object(type='person', confidence=0, latitude=0.0, longitude=0.0),
            'tent': self.mc.Detection_Object(type='tent', confidence=0, latitude=0.0, longitude=0.0),
        }
        self.fake.get_logger = lambda: _Logger()
        self.fake.send_ack = lambda text: acks.append(str(text))

    def feed(self, msg) -> None:
        """Run MainController.image_result_cb(self, msg) with the stand-in self."""
        self.mc.MainController.image_result_cb(self.fake, msg)

    def best(self) -> dict:
        """{'person': (confidence, lat, lon), 'tent': (...)} as main_controller stores them."""
        return {k: (float(v.confidence), float(v.lat), float(v.long))
                for k, v in self.fake.detections.items()}

    @property
    def processed(self) -> int:
        """How many distinct photos main_controller counts as processed."""
        return len(self.fake.processed_image_names)
