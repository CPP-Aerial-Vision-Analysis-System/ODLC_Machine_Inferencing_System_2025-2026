"""Purpose: check real SIYI node orientation parameters with camera networking and disk storage disabled."""

import os
import sys
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('mavros_msgs')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from video_cam import siyi_node  # noqa: E402


@pytest.mark.parametrize('value,rotate,expected', [
    ('90', False, 90.0), ('90.5', False, 90.5),
    ('90', True, 270.0), ('-90', False, 270.0),
    ('true', False, 0.0), ('typo', False, 0.0),
])
def test_orientation_override_starts_camera_node(monkeypatch, value, rotate, expected):
    """Verify integer overrides no longer prevent startup or lose the saved-image rotation."""
    monkeypatch.setattr(siyi_node, 'StorageManager', lambda *args, **kwargs: None)
    rclpy.init(domain_id=140 + os.getpid() % 20, args=[
        '--ros-args', '-p', 'use_real_camera:=false', '-p',
        f'image_top_direction_deg:={value}', '-p', f'rotate_180:={str(rotate).lower()}'])
    node = None
    try:
        node = siyi_node.SIYINode()
        assert node.camera is None
        assert node.image_top_direction_deg == expected
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
