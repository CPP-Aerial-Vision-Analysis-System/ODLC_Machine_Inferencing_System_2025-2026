import tempfile
import time
import unittest
from io import BytesIO
from pathlib import Path
from unittest.mock import patch
from PIL import Image
from astra_viewer.storage import Archive
from astra_viewer.pipeline import Pipeline, prepare


def metadata(key, confidence=None, captured=1):
    return dict(frame_key=key, captured=captured, camera='test', width=640, height=360,
                objects=[] if confidence is None else [dict(label='person', confidence=confidence)])


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.archive = Archive(self.tmp.name, max_bytes=100, reserve_bytes=0)

    def tearDown(self):
        self.archive.db.close()
        self.tmp.cleanup()

    def save(self, key, confidence=None, captured=1):
        return self.archive.save(b'jpeg-data', b'thumb-data!', metadata(key, confidence, captured))

    def test_cleanup_prefers_no_detection_then_low_confidence_and_protects(self):
        high = self.save('high', .95, 1)
        low = self.save('low', .32, 2)
        empty = self.save('empty', None, 3)
        protected = self.save('protected', None, 0)
        self.archive.protect(protected, True)
        self.archive.max_bytes = 80
        self.save('new', .9, 4)
        self.assertIsNone(self.archive.get(empty))
        self.assertIsNotNone(self.archive.get(low))
        self.save('newer', .9, 5)
        self.assertIsNone(self.archive.get(low))
        self.assertIsNotNone(self.archive.get(high))
        self.assertTrue(self.archive.get(protected)['protected'])
        self.assertFalse((Path(self.tmp.name) / (empty + '.jpg')).exists())

    def test_full_protected_archive_stops_saving_without_deleting_protected(self):
        ident = self.save('one', .9)
        self.archive.protect(ident, True)
        self.archive.max_bytes = 20
        with self.assertRaises(OSError):
            self.save('two')
        self.assertIsNotNone(self.archive.read(ident))
        self.assertEqual(self.archive.status()['count'], 1)

    def test_disk_reserve_triggers_cleanup_even_under_archive_limit(self):
        ident = self.save('old')
        self.archive.reserve_bytes = 100
        usage = type('Usage', (), {'free': 1000})()
        low = type('Usage', (), {'free': 105})()
        with patch('astra_viewer.storage.shutil.disk_usage', side_effect=[low, usage]):
            self.save('new')
        self.assertIsNone(self.archive.get(ident))

    def test_cursor_ties_and_duplicate_frame_are_stable(self):
        ids = [self.save(str(i), .8, 5) for i in range(4)]
        first = self.archive.list(limit=2)
        second = self.archive.list(before=first['next'], limit=2)
        self.assertEqual({p['id'] for p in first['photos'] + second['photos']}, set(ids))
        self.assertIsNone(second['next'])
        self.assertIsNone(self.save('0'))
        self.assertEqual(self.archive.status()['count'], 4)

    def test_protection_persists_after_restart(self):
        ident = self.save('persist')
        self.archive.protect(ident, True)
        self.archive.db.close()
        self.archive = Archive(self.tmp.name, reserve_bytes=0)
        self.assertTrue(self.archive.get(ident)['protected'])


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.archive = Archive(self.tmp.name, max_bytes=10000000, reserve_bytes=0)
        self.now = 10
        self.pipeline = Pipeline(self.archive, clock=lambda:self.now)

    def tearDown(self):
        self.pipeline.close()
        self.archive.db.close()
        self.tmp.cleanup()

    def test_annotations_are_embedded_and_empty_detections_are_valid(self):
        source = Image.new('RGB', (640, 360), 'gray')
        info = dict(stamp=dict(sec=100, nanosec=0), frame_id='cam', objects=[])
        jpeg, thumb, meta = prepare(source, info, 'ZR10')
        self.assertEqual(meta['objects'], [])
        self.assertEqual(Image.open(BytesIO(jpeg)).size, (640, 360))
        self.assertLessEqual(Image.open(BytesIO(thumb)).width, 288)
        self.assertNotEqual(Image.open(BytesIO(jpeg)).getpixel((15,15)), (128,128,128))
        self.assertEqual(source.getpixel((15,15)), (128,128,128))

    def test_only_matching_timestamps_are_processed_even_when_info_arrives_first(self):
        image = Image.new('RGB',(640,360))
        self.pipeline.detection(dict(stamp=dict(sec=2,nanosec=1), objects=[]))
        self.pipeline.raw_frame(image, dict(sec=1,nanosec=1))
        self.assertIsNone(self.pipeline.pending)
        self.pipeline.raw_frame(image, dict(sec=2,nanosec=1))
        self.assertEqual(self.pipeline.pending[1]['stamp']['sec'], 2)
        for i in range(100):
            self.pipeline.raw_frame(image, dict(sec=i,nanosec=0))
        self.assertLessEqual(len(self.pipeline.raw), 24)

    def test_capture_does_not_duplicate_a_frame_or_save_stale_results(self):
        self.pipeline.latest = (b'jpeg', b'thumb', metadata('one'))
        self.pipeline.last_result = self.now
        self.pipeline.capture_once()
        self.pipeline.capture_once()
        self.assertEqual(self.archive.status()['count'], 1)
        self.pipeline.latest = (b'jpeg', b'thumb', metadata('two'))
        self.now += 6
        self.pipeline.capture_once()
        self.assertEqual(self.archive.status()['count'], 1)
        self.pipeline.last_result = self.now
        self.pipeline.capture_once()
        self.assertEqual(self.archive.status()['count'], 2)

    def test_staleness_distinguishes_camera_loss_from_detection_loss(self):
        self.assertEqual(self.pipeline.status()['state'], 'waiting')
        self.pipeline.last_input = self.now
        self.assertEqual(self.pipeline.status()['state'], 'detection_stale')
        self.pipeline.last_result = self.now
        self.assertEqual(self.pipeline.status()['state'], 'live')
        self.now += 6
        self.assertEqual(self.pipeline.status()['state'], 'disconnected')

    def test_background_worker_saves_real_jpeg(self):
        self.pipeline.interval = .1
        self.pipeline.start()
        self.pipeline.raw_frame(Image.new('RGB',(640,360)), dict(sec=1,nanosec=0))
        self.pipeline.detection(dict(stamp=dict(sec=1,nanosec=0),objects=[]))
        deadline=time.monotonic()+3
        while not self.archive.list()['photos'] and time.monotonic()<deadline:
            time.sleep(.02)
        photo=self.archive.list()['photos'][0]
        self.assertEqual(Image.open(BytesIO(self.archive.read(photo['id']))).size,(640,360))


if __name__ == '__main__':
    unittest.main()
