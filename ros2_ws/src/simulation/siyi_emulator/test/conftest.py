"""
test/conftest.py -- PURPOSE: shared pytest fixtures for the emulator tests: a FakeGzLink that behaves
like Gazebo (answers each photo trigger with a synthetic frame after a chosen delay, records the
gimbal joint targets, echoes them back as "measured" joint states, and can inject drone odometry),
and a factory that starts a real SiyiEmulator on 127.0.0.1 with free ports and a temporary SD card.
"""

import logging
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

try:  # tests normally use the installed package (source install/setup.bash) ...
    import siyi_emulator  # noqa: F401
except ImportError:  # ... but also work straight from the source tree
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from siyi_emulator.emulator import EmulatorConfig, SiyiEmulator  # noqa: E402
from siyi_emulator.gz_link import JointSample, NullGzLink, OdomSample, RawImage  # noqa: E402

# The compatibility test drives last year's real video_cam code. When video_cam is not built in the
# same workspace, leave that file out at collection time. (A module-level pytest.importorskip there
# made pytest 6.2.5, the Ubuntu 22.04 version, collect NO tests at all from this folder.)
import importlib.util  # noqa: E402

collect_ignore = []
if importlib.util.find_spec("video_cam") is None:
    collect_ignore.append("test_video_cam_compat.py")


def synthetic_rgb(width, height):
    """A frame with a texture and a white 40x40 square in the centre (to check digital zoom)."""
    rng = np.random.default_rng(3)
    img = rng.integers(0, 120, size=(height, width, 3), dtype=np.uint8)
    cy, cx = height // 2, width // 2
    img[cy - 20: cy + 20, cx - 20: cx + 20] = 255
    return img


class FakeGzLink(NullGzLink):
    """Stands in for GzLink: same callbacks, no Gazebo."""

    def __init__(self, width=2560, height=1440, frame_delay_s=0.05, send_frames=True):
        super().__init__("fake gazebo")
        self.available = True
        self.reason = "fake gazebo"
        self.width, self.height = width, height
        self.frame_delay_s = frame_delay_s
        self.send_frames = send_frames
        self.joint_targets = []
        self.zoom_commands = []
        self.sim_time = 1.0
        self.echo_joints = True
        self._rgb = synthetic_rgb(width, height)

    def start(self):
        return True

    def publish_joint_targets(self, yaw, roll, pitch):
        self.joint_targets.append((yaw, roll, pitch))
        self.sim_time += 0.02
        if self.echo_joints and self.on_joint_states is not None:
            self.on_joint_states(JointSample(self.sim_time, {"yaw": yaw, "roll": roll, "pitch": pitch},
                                             {"yaw": 0.0, "roll": 0.0, "pitch": 0.0}))

    def publish_optical_zoom(self, value):
        self.zoom_commands.append(value)

    def trigger_photo(self):
        self.triggers_sent += 1
        if not self.send_frames:
            return True

        def deliver():
            time.sleep(self.frame_delay_s)
            if self.on_photo is not None:
                self.on_photo(RawImage(self.sim_time + 0.01, self.width, self.height, self.width * 3, 3,
                                       self._rgb.tobytes()))

        threading.Thread(target=deliver, daemon=True).start()
        return True

    def send_odometry(self, quat_xyzw, position=(0.0, 0.0, 30.0)):
        self.sim_time += 0.02
        if self.on_odometry is not None:
            self.on_odometry(OdomSample(self.sim_time, position, quat_xyzw))


@pytest.fixture()
def start_emulator(tmp_path):
    """Factory: start_emulator(gz=None, **config_overrides) -> running SiyiEmulator (stopped afterwards)."""
    running = []

    def _start(gz=None, **overrides):
        values = dict(bind_ip="127.0.0.1", ctrl_port=0, media_port=0, sd_card_dir=str(tmp_path / "sd"),
                      initial_pitch_deg=0.0)
        values.update(overrides)
        config, unknown = EmulatorConfig().updated(values)
        assert not unknown, unknown
        emu = SiyiEmulator(config, gz_link=gz if gz is not None else NullGzLink("tests"),
                           logger=logging.getLogger("siyi_emulator.test"))
        emu.start()
        running.append(emu)
        return emu

    yield _start
    for emu in running:
        emu.stop()
