"""
sim_demo/detections.py -- PURPOSE: turn the team detector's output message into simple Python
records (and build that message for the truth_detector stand-in), so the panel, markers and
scorer never have to dig through nested ROS message fields.

The message (team package `interfaces`, file msg/ImageResult.msg), as detection/new_od.py fills it:
    topic /image_detection, QoS RELIABLE depth 10
    header                 stamp = publish time, frame_id 'camera'
    detections             vision_msgs/Detection2DArray with ONLY person/tent boxes:
                             bbox.center.position.x/.y = box centre [px of the photo],
                             bbox.size_x/.size_y = box size [px],
                             results[0].hypothesis.class_id = "0" person / "1" tent,
                             results[0].hypothesis.score = confidence 0..1
    masks                  always []
    image_name             the photo's file name, e.g. "38.315339 , -76.548108.jpg"
    timestamp              wall-clock ISO string
    num_detections         count of ALL categorised detections (incl. "object")
    saved_to               path of the annotated JPEG
    method                 'sahi+yolo' / 'sahi+yolo+tensorrt' (truth_detector: 'truth_detector ...')
    slice_size, overlap    "640x640", "0.15x0.15"
    waypoint_index         last /mavros/mission/reached seq
    latitude, longitude    float32, parsed from the file name = drone GPS when the photo was taken
    classes, confidences, areas, descriptions   parallel arrays over ALL detections

main_controller.py reads class_id through {"0": "person", "1": "tent"} and keeps, per class,
the single highest score it has ever seen (strictly greater wins), flying to that message's
latitude/longitude. best_per_class() reproduces that choice.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from datetime import datetime

# Same map as main/main_controller.py line 367 and detection/new_od.py CLASS_ID.
CLASS_ID_TO_NAME = {'0': 'person', '1': 'tent'}
NAME_TO_CLASS_ID = {'person': '0', 'tent': '1'}
DETECTION_TOPIC = '/image_detection'
TRUTH_METHOD_PREFIX = 'truth_detector'


@dataclass
class Box:
    """One detection box in photo pixels.

    Attributes:
        cls: 'person' or 'tent' (or 'class <id>' for an unknown id).
        score: confidence 0..1.
        cx, cy, w, h: box centre and size in pixels of the full-resolution photo.
    """

    cls: str
    score: float
    cx: float
    cy: float
    w: float
    h: float

    @property
    def xyxy(self) -> tuple[float, float, float, float]:
        """(x1, y1, x2, y2) corners in pixels."""
        return (self.cx - self.w / 2.0, self.cy - self.h / 2.0,
                self.cx + self.w / 2.0, self.cy + self.h / 2.0)


@dataclass
class DetectionFrame:
    """Everything one ImageResult message says about one photo.

    Attributes:
        image_name: photo file name (matches a file in the photo folder).
        lat, lon: the message's latitude/longitude (0.0 if the name had no GPS).
        boxes: person/tent boxes (what main_controller acts on).
        method: detector method string ('sahi+yolo', 'truth_detector (stand-in)', ...).
        num_detections: count of all categorised detections (incl. 'object').
        classes, confidences: the informational parallel arrays.
        waypoint_index: last mission item reached when it was published.
        received: wall-clock time this program received it [s].
        stamp: header stamp [s] (0 if unset).
        saved_to: annotated image path from the detector.
    """

    image_name: str
    lat: float
    lon: float
    boxes: list[Box] = field(default_factory=list)
    method: str = ''
    num_detections: int = 0
    classes: list[str] = field(default_factory=list)
    confidences: list[float] = field(default_factory=list)
    waypoint_index: int = 0
    received: float = 0.0
    stamp: float = 0.0
    saved_to: str = ''

    @property
    def is_stand_in(self) -> bool:
        """True if this result came from the truth_detector stand-in (not the AI model)."""
        return self.method.startswith(TRUTH_METHOD_PREFIX)

    @property
    def has_gps(self) -> bool:
        """False when the detector had no GPS in the file name (lat = lon = 0)."""
        return not (self.lat == 0.0 and self.lon == 0.0)


def image_result_to_frame(msg, received: float | None = None) -> DetectionFrame:
    """Convert an interfaces/msg/ImageResult (or any object with the same fields) to a frame.

    Robust: missing or odd fields become defaults instead of exceptions.
    """
    boxes: list[Box] = []
    try:
        dets = msg.detections.detections
    except AttributeError:
        dets = []
    for d in dets:
        try:
            cx = float(d.bbox.center.position.x)
            cy = float(d.bbox.center.position.y)
        except AttributeError:   # vision_msgs < 4 (Foxy) had center.x / center.y
            cx = float(getattr(d.bbox.center, 'x', 0.0))
            cy = float(getattr(d.bbox.center, 'y', 0.0))
        w, h = float(d.bbox.size_x), float(d.bbox.size_y)
        for r in getattr(d, 'results', []):
            hyp = getattr(r, 'hypothesis', r)
            class_id = str(getattr(hyp, 'class_id', ''))
            cls = CLASS_ID_TO_NAME.get(class_id, f'class {class_id}')
            boxes.append(Box(cls, float(getattr(hyp, 'score', 0.0)), cx, cy, w, h))
    stamp = 0.0
    try:
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
    except AttributeError:
        pass
    return DetectionFrame(
        image_name=str(getattr(msg, 'image_name', '') or ''),
        lat=float(getattr(msg, 'latitude', 0.0) or 0.0),
        lon=float(getattr(msg, 'longitude', 0.0) or 0.0),
        boxes=boxes,
        method=str(getattr(msg, 'method', '') or ''),
        num_detections=int(getattr(msg, 'num_detections', len(boxes)) or 0),
        classes=[str(c) for c in getattr(msg, 'classes', [])],
        confidences=[float(c) for c in getattr(msg, 'confidences', [])],
        waypoint_index=int(getattr(msg, 'waypoint_index', 0) or 0),
        received=time.time() if received is None else received,
        stamp=stamp,
        saved_to=str(getattr(msg, 'saved_to', '') or ''),
    )


def frame_to_image_result(frame: DetectionFrame, image_result_cls=None, detection2d_cls=None,
                          hypothesis_cls=None, array_cls=None, stamp=None, slice_size: str = '',
                          overlap: str = ''):
    """Build an interfaces/msg/ImageResult from a frame, filled exactly like new_od does.

    The message classes are imported here (not at module import) so the pure parts of this
    module work without ROS. Callers may pass the classes in (tests do).

    Args:
        frame: the detection data. stamp: builtin_interfaces/Time for the headers.
    Returns:
        an ImageResult message ready to publish on /image_detection.
    """
    if image_result_cls is None:
        from interfaces.msg import ImageResult as image_result_cls  # noqa: N813
    if detection2d_cls is None or hypothesis_cls is None or array_cls is None:
        from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
        detection2d_cls = detection2d_cls or Detection2D
        hypothesis_cls = hypothesis_cls or ObjectHypothesisWithPose
        array_cls = array_cls or Detection2DArray
    msg = image_result_cls()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = 'camera'
    msg.image_name = frame.image_name
    # new_od writes datetime.now().isoformat(); do the same.
    msg.timestamp = datetime.fromtimestamp(frame.received or time.time()).isoformat()
    msg.saved_to = frame.saved_to
    msg.method = frame.method
    msg.slice_size = slice_size
    msg.overlap = overlap
    msg.waypoint_index = int(frame.waypoint_index)
    msg.latitude = float(frame.lat)
    msg.longitude = float(frame.lon)
    arr = array_cls()
    arr.header = msg.header
    classes, confidences, areas, descriptions = [], [], [], []
    for b in frame.boxes:
        classes.append(b.cls)
        confidences.append(float(b.score))
        areas.append(float(b.w * b.h))
        descriptions.append(b.cls)
        class_id = NAME_TO_CLASS_ID.get(b.cls)
        if class_id is None:   # like new_od: only person/tent go into the Detection2DArray
            continue
        d = detection2d_cls()
        d.header = msg.header
        d.bbox.center.position.x = float(b.cx)
        d.bbox.center.position.y = float(b.cy)
        d.bbox.size_x = float(b.w)
        d.bbox.size_y = float(b.h)
        hyp = hypothesis_cls()
        hyp.hypothesis.class_id = class_id
        hyp.hypothesis.score = float(b.score)
        d.results.append(hyp)
        arr.detections.append(d)
    msg.detections = arr
    msg.masks = []
    msg.num_detections = len(frame.boxes)
    msg.classes = classes
    msg.confidences = confidences
    msg.areas = areas
    msg.descriptions = descriptions
    return msg


def best_per_class(frames: list[DetectionFrame]) -> dict[str, tuple[float, DetectionFrame, Box]]:
    """What main_controller would fly to: per class, the highest-scoring box ever seen.

    Mirrors main_controller.image_result_cb: frames in arrival order, a box replaces the
    current best only if its score is STRICTLY greater.

    Returns:
        {'person': (score, frame, box), 'tent': (...)} for the classes seen.
    """
    best: dict[str, tuple[float, DetectionFrame, Box]] = {}
    for f in frames:
        for b in f.boxes:
            if b.cls not in ('person', 'tent'):
                continue
            if b.cls not in best or b.score > best[b.cls][0]:
                best[b.cls] = (b.score, f, b)
    return best
