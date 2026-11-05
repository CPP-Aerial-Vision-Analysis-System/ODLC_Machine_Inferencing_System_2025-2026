"""Frame matching and JPEG preparation, independent from HTTP and ROS callbacks."""
from collections import OrderedDict
from datetime import datetime, timezone
import hashlib
from io import BytesIO
import math
import threading
import time

from PIL import Image, ImageDraw, ImageFont


def font(size):
    for path in ('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/System/Library/Fonts/Supplemental/Arial.ttf', 'arial.ttf'):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


def encode(image, quality=88):
    out = BytesIO()
    image.save(out, 'JPEG', quality=quality)
    return out.getvalue()


def prepare(image, info, camera):
    image = image.convert('RGB').copy()
    width, height = image.size
    draw = ImageDraw.Draw(image)
    size = max(12, round(width / 100))
    small = font(size)
    objects = []
    for item in info.get('objects', []):
        try:
            confidence = float(item['confidence'])
            box = [float(v) for v in item['bbox']]
            if len(box) != 4 or not all(math.isfinite(v) for v in box + [confidence]) or not 0 <= confidence <= 1:
                continue
            x1, y1, x2, y2 = box
            x1, x2 = max(0, min(width - 1, x1)), max(0, min(width - 1, x2))
            y1, y2 = max(0, min(height - 1, y1)), max(0, min(height - 1, y2))
            if x2 <= x1 or y2 <= y1:
                continue
            objects.append(dict(label=str(item.get('yolo_class', item.get('class', 'object')))[:80],
                                confidence=confidence, bbox=[x1, y1, x2, y2]))
        except (TypeError, KeyError, ValueError):
            continue
    stamp = info['stamp']
    captured = stamp['sec'] + stamp['nanosec'] / 1e9
    key = f"{info.get('frame_id', camera)}:{stamp['sec']}:{stamp['nanosec']}"
    short = hashlib.sha256(key.encode()).hexdigest()[:8].upper()

    def label(x, y, text, color='#eaf6f5'):
        bounds = draw.textbbox((0, 0), text, font=small)
        tw, th = bounds[2], bounds[3] - bounds[1]
        x = max(0, min(x, width - tw - 12))
        y = max(0, min(y, height - th - 12))
        draw.rectangle((x, y, x + tw + 12, y + th + 10), fill='#102126')
        draw.text((x + 6, y + 4 - bounds[1]), text, font=small, fill=color)

    for item in objects:
        x1, y1, x2, y2 = item['bbox']
        draw.rectangle((x1, y1, x2, y2), outline='#6de5be', width=max(2, width // 500))
        label(x1, max(34, y1 - size - 14), f"{item['label']}  {item['confidence']:.0%}", '#6de5be')
    label(12, 12, datetime.fromtimestamp(captured, timezone.utc).strftime('%Y-%m-%d  %H:%M:%S UTC'))
    summary = f"MAX CONF {max(o['confidence'] for o in objects):.0%}" if objects else 'NO DETECTIONS'
    tw = draw.textlength(summary, font=small)
    label(width - tw - 24, 12, summary)
    label(12, height - size - 24, f'{camera}  /  {short}')
    thumb = image.copy()
    thumb.thumbnail((288, 162))
    return encode(image), encode(thumb, 76), dict(captured=captured, frame_key=key,
        camera=camera, objects=objects, width=width, height=height, reference=short,
        processing_sec=info.get('processing_sec'))


class Pipeline:
    def __init__(self, archive, camera='ZR10', interval=1.0, stale_after=5.0, clock=time.monotonic):
        self.archive, self.camera, self.interval = archive, camera, interval
        self.stale_after, self.clock = stale_after, clock
        self.condition = threading.Condition()
        self.raw = OrderedDict()
        self.infos = OrderedDict()
        self.pending = None
        self.latest = None
        self.sequence = 0
        self.last_input = None
        self.last_result = None
        self.last_saved = None
        self.last_saved_key = None
        self.error = ''
        self.source_error = ''
        self.stop = threading.Event()
        self.threads = []

    @staticmethod
    def key(stamp):
        return int(stamp['sec']), int(stamp['nanosec'])

    def raw_frame(self, image, stamp):
        key = self.key(stamp)
        with self.condition:
            self.last_input = self.clock()
            self.raw[key] = image
            while len(self.raw) > 24:
                self.raw.popitem(last=False)
            if key in self.infos:
                self.pending = (image, self.infos.pop(key))
                self.condition.notify_all()

    def detection(self, info):
        key = self.key(info['stamp'])
        with self.condition:
            if key in self.raw:
                self.pending = (self.raw[key], info)
                self.condition.notify_all()
            else:
                self.infos[key] = info
                while len(self.infos) > 24:
                    self.infos.popitem(last=False)

    def start(self):
        for target in (self._process, self._capture):
            thread = threading.Thread(target=target, daemon=True)
            thread.start()
            self.threads.append(thread)

    def _process(self):
        while not self.stop.is_set():
            with self.condition:
                self.condition.wait_for(lambda: self.pending is not None or self.stop.is_set(), timeout=1)
                if self.stop.is_set():
                    break
                work, self.pending = self.pending, None
            if work is None:
                continue
            try:
                result = prepare(*work, self.camera)
                with self.condition:
                    self.latest = result
                    self.last_result = self.clock()
                    self.sequence += 1
                    self.source_error = ''
                    self.condition.notify_all()
            except Exception as exc:
                self.source_error = f'Cannot prepare frame: {exc}'

    def capture_once(self):
        with self.condition:
            result = self.latest
            fresh = self.last_result is not None and self.clock() - self.last_result < self.stale_after
        if result and fresh and result[2]['frame_key'] != self.last_saved_key:
            try:
                self.archive.save(*result)
                self.last_saved_key = result[2]['frame_key']
                self.last_saved = time.time()
                self.error = ''
            except (OSError, ValueError, RuntimeError) as exc:
                self.error = str(exc)

    def _capture(self):
        deadline = time.monotonic() + self.interval
        while not self.stop.wait(max(0, deadline - time.monotonic())):
            try:
                self.capture_once()
            except Exception as exc:
                self.error = f'Cannot save photo: {exc}'
            deadline += self.interval
            if deadline < time.monotonic():
                deadline = time.monotonic() + self.interval

    def status(self):
        with self.condition:
            now = self.clock()
            input_age = None if self.last_input is None else now - self.last_input
            result_age = None if self.last_result is None else now - self.last_result
            state = 'live' if result_age is not None and result_age < self.stale_after else 'waiting'
            if state != 'live' and input_age is not None:
                state = 'detection_stale' if input_age < self.stale_after else 'disconnected'
            metadata = self.latest[2] if self.latest else None
            return dict(state=state, input_age_sec=input_age, result_age_sec=result_age,
                        sequence=self.sequence, metadata=metadata, capture_interval=self.interval,
                        last_saved=self.last_saved, error=self.error, source_error=self.source_error)

    def close(self):
        self.stop.set()
        with self.condition:
            self.condition.notify_all()
        for thread in self.threads:
            thread.join(timeout=3)


def demo_source(pipeline):
    """Synthetic test frames. No camera or model is used."""
    i = 0
    while not pipeline.stop.is_set():
        now = time.time_ns()
        stamp = dict(sec=now // 10**9, nanosec=now % 10**9)
        image = Image.new('RGB', (1280, 720), '#46594a')
        d = ImageDraw.Draw(image)
        for row in range(8):
            for col in range(10):
                colors = ('#687d58', '#526945', '#8a8a5c', '#4c6958')
                x, y = col * 160 - 90, row * 110 - 50
                d.polygon([(x,y),(x+150,y+12),(x+132,y+105),(x-18,y+93)], fill=colors[(row * 3 + col) % 4])
        d.line([(0,630),(520,360),(1280,120)], fill='#b5ac8b', width=35)
        d.line([(0,630),(520,360),(1280,120)], fill='#cec5a8', width=23)
        x = int(570 + math.sin(i / 18) * 170)
        d.ellipse((x+9,303,x+23,317), fill='#e5b891')
        d.rounded_rectangle((x+5,318,x+28,348), radius=5, fill='#dc654a')
        d.line((x+11,347,x+8,369), fill='#1e3237', width=7)
        d.line((x+24,347,x+27,369), fill='#1e3237', width=7)
        d.polygon([(880,430),(940,390),(1010,440)], fill='#dbbe78')
        d.polygon([(880,430),(940,390),(928,450)], fill='#b68e51')
        d.text((535,650), 'SYNTHETIC DEMO · NOT A CAMERA FEED', font=font(19), fill='#ffffff')
        objects = [dict(yolo_class='person', confidence=round(.88 + .08 * math.sin(i / 15), 3), bbox=[x-4,296,x+38,378]),
                   dict(yolo_class='tent', confidence=.74, bbox=[872,384,1016,455])]
        if i % 80 >= 60:
            objects = []
        pipeline.raw_frame(image, stamp)
        pipeline.detection(dict(stamp=stamp, frame_id='demo_camera', objects=objects, processing_sec=.042))
        i += 1
        pipeline.stop.wait(.2)


def ros_source(pipeline, image_topic, info_topic):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image as RosImage
    from std_msgs.msg import String
    import json

    rclpy.init(args=[])
    node = Node('astra_viewer')
    bridge = CvBridge()

    def raw(message):
        try:
            pixels = bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            pipeline.raw_frame(Image.fromarray(pixels.copy()),
                dict(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec))
        except Exception as exc:
            pipeline.source_error = f'Camera conversion failed: {exc}'

    def info(message):
        try:
            pipeline.detection(json.loads(message.data))
        except Exception as exc:
            pipeline.source_error = f'Detection message rejected: {exc}'

    subscriptions = [node.create_subscription(RosImage, image_topic, raw, qos_profile_sensor_data),
                     node.create_subscription(String, info_topic, info, 10)]
    try:
        while not pipeline.stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
