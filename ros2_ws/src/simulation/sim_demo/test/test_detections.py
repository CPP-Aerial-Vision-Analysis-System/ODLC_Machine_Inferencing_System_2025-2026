"""
sim_demo/test/test_detections.py -- PURPOSE: unit tests of the ImageResult <-> record
conversion (detections.py) and of best_per_class(), which must pick exactly what the team's
main_controller picks. Uses tiny stand-in message classes so it runs without ROS; when the
real `interfaces` and `vision_msgs` packages are importable it also round-trips the REAL
message types and checks every field main_controller reads.
"""
from types import SimpleNamespace as NS

import pytest

from sim_demo.detections import (Box, DetectionFrame, best_per_class, frame_to_image_result,
                                 image_result_to_frame)


# ---- minimal fake message classes with the same field layout as the ROS types -------------
class _Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakeHyp:
    def __init__(self):
        self.hypothesis = _Obj(class_id='', score=0.0)


class FakeDet:
    def __init__(self):
        self.header = None
        self.bbox = _Obj(center=_Obj(position=_Obj(x=0.0, y=0.0), theta=0.0), size_x=0.0, size_y=0.0)
        self.results = []


class FakeArr:
    def __init__(self):
        self.header = None
        self.detections = []


class FakeImageResult:
    def __init__(self):
        self.header = _Obj(stamp=_Obj(sec=0, nanosec=0), frame_id='')
        self.detections = FakeArr()
        for k, v in dict(masks=[], image_name='', timestamp='', num_detections=0, saved_to='',
                         method='', slice_size='', overlap='', waypoint_index=0, longitude=0.0,
                         latitude=0.0, classes=[], confidences=[], areas=[], descriptions=[]).items():
            setattr(self, k, v)


def _frame():
    return DetectionFrame(image_name='38.315339 , -76.548108.jpg', lat=38.315339, lon=-76.548108,
                          boxes=[Box('tent', 0.91, 1000, 500, 80, 60), Box('person', 0.77, 200, 300, 20, 50)],
                          method='truth_detector (stand-in)', waypoint_index=7, received=1.0e9)


def test_build_and_read_back_with_fakes():
    msg = frame_to_image_result(_frame(), FakeImageResult, FakeDet, FakeHyp, FakeArr)
    assert msg.header.frame_id == 'camera'
    assert msg.image_name == '38.315339 , -76.548108.jpg'
    assert msg.latitude == pytest.approx(38.315339) and msg.longitude == pytest.approx(-76.548108)
    ids = [d.results[0].hypothesis.class_id for d in msg.detections.detections]
    assert ids == ['1', '0']                                     # tent "1", person "0"
    assert msg.num_detections == 2 and msg.classes == ['tent', 'person']
    back = image_result_to_frame(msg, received=5.0)
    assert [(b.cls, b.score, b.cx, b.cy, b.w, b.h) for b in back.boxes] == [
        ('tent', 0.91, 1000, 500, 80, 60), ('person', 0.77, 200, 300, 20, 50)]
    assert back.is_stand_in and back.has_gps and back.waypoint_index == 7


def test_unknown_class_ids_are_kept_but_labelled():
    msg = FakeImageResult()
    d = FakeDet()
    h = FakeHyp()
    h.hypothesis.class_id, h.hypothesis.score = '2', 0.5
    d.results.append(h)
    msg.detections.detections.append(d)
    f = image_result_to_frame(msg)
    assert f.boxes[0].cls == 'class 2'
    assert best_per_class([f]) == {}                  # main_controller ignores non-targets


def test_robust_to_missing_fields():
    f = image_result_to_frame(NS(image_name='x.jpg'))
    assert f.image_name == 'x.jpg' and f.boxes == [] and not f.has_gps


def test_best_per_class_matches_main_controller():
    f1 = DetectionFrame('a.jpg', 1, 1, [Box('tent', 0.6, 0, 0, 1, 1)])
    f2 = DetectionFrame('b.jpg', 2, 2, [Box('tent', 0.9, 0, 0, 1, 1), Box('person', 0.4, 0, 0, 1, 1)])
    f3 = DetectionFrame('c.jpg', 3, 3, [Box('tent', 0.9, 0, 0, 1, 1)])     # tie: first one wins
    best = best_per_class([f1, f2, f3])
    assert best['tent'][1].image_name == 'b.jpg' and best['person'][0] == 0.4


def test_real_message_types_if_available():
    pytest.importorskip('interfaces.msg')
    pytest.importorskip('vision_msgs.msg')
    from interfaces.msg import ImageResult
    msg = frame_to_image_result(_frame())
    assert isinstance(msg, ImageResult)
    # Exactly the fields main_controller.image_result_cb reads:
    assert msg.image_name.endswith('.jpg')
    for det in msg.detections.detections:
        assert det.results[0].hypothesis.class_id in ('0', '1')
        assert 0.0 < det.results[0].hypothesis.score <= 1.0
        assert det.bbox.center.position.x > 0 and det.bbox.size_x > 0
    assert isinstance(msg.latitude, float) and isinstance(msg.longitude, float)
    back = image_result_to_frame(msg)
    assert [b.cls for b in back.boxes] == ['tent', 'person']
