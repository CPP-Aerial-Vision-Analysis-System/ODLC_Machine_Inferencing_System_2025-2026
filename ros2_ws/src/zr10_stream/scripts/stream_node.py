#!/usr/bin/env python3
"""Publish decoded ZR10 frames for a detector without a disk polling stage."""
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from frame_source import FrameSource, png_bytes


class StreamNode(Node):
    def __init__(self):
        super().__init__('zr10_stream')
        def param(name, default):
            return self.declare_parameter(name, default).value
        url = param('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')
        self.width = param('width', 1280)
        self.height = param('height', 720)
        fps = param('fps', 10.0)
        self.max_age = param('max_frame_age_sec', 2.0)
        self.frame_id = param('frame_id', 'zr10_camera')
        self.directory = Path(param('snapshot_directory', 'stream_snapshots')).expanduser()
        if not url.startswith('rtsp://'):
            raise ValueError('rtsp_url must start with rtsp://')
        if not (1 <= self.width <= 4096 and 1 <= self.height <= 2160):
            raise ValueError('resolution outside supported bounds')
        if not (math.isfinite(fps) and 0 < fps <= 60 and
                math.isfinite(self.max_age) and 0 < self.max_age <= 60):
            raise ValueError('fps or max_frame_age_sec outside supported bounds')
        # Reliable publisher also permits a best-effort detector subscriber.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Image, 'zr10/image_raw', qos)
        self.service = self.create_service(Trigger, 'zr10/save_frame', self.save_frame)
        command = ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-nostdin',
                   '-rtsp_transport', 'tcp', '-i', url, '-an',
                   '-vf', f'scale={self.width}:{self.height},fps={fps}',
                   '-pix_fmt', 'rgb24', '-f', 'rawvideo', 'pipe:1']
        self.source = FrameSource(command, self.width * self.height * 3)
        self.last_sequence = 0
        self.last_warning = 0.0
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)
        self.source.start()

    def fresh_frame(self):
        frame, error = self.source.get()
        if frame is None or time.monotonic() - frame[1] > self.max_age:
            raise RuntimeError(error or 'latest frame is stale')
        return frame

    def publish_frame(self):
        try:
            sequence, _, rgb = self.fresh_frame()
        except RuntimeError as exc:
            if time.monotonic() - self.last_warning > 5:
                self.get_logger().warning(str(exc))
                self.last_warning = time.monotonic()
            return
        if sequence == self.last_sequence:
            return
        msg = Image()
        # Host publication time; camera exposure time is unavailable over this path.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.width, msg.height = self.width, self.height
        msg.encoding, msg.is_bigendian, msg.step = 'rgb8', 0, self.width * 3
        msg.data = rgb
        self.publisher.publish(msg)
        self.last_sequence = sequence

    def save_frame(self, request, response):
        try:
            _, _, rgb = self.fresh_frame()
            self.directory.mkdir(parents=True, exist_ok=True)
            path = self.directory / f'frame_{uuid.uuid4().hex}.png'
            data = png_bytes(self.width, self.height, rgb)
            with path.open('xb') as output:
                output.write(data)
            response.success, response.message = True, str(path.resolve())
        except (OSError, RuntimeError, ValueError) as exc:
            response.success, response.message = False, str(exc)
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = StreamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.source.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
