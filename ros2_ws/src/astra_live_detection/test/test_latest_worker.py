import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from astra_live_detection.latest_worker import LatestWorker


class WorkerTests(unittest.TestCase):
    def until(self, predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.005)
        self.fail('worker did not reach expected state')

    def test_slow_inference_replaces_pending_with_latest(self):
        entered, release = threading.Event(), threading.Event()
        processed = []
        def infer(message):
            processed.append(message)
            if message == 1:
                entered.set()
                release.wait(2)
            return message
        worker = LatestWorker(infer)
        worker.start()
        try:
            worker.submit(1)
            self.assertTrue(entered.wait(1))
            worker.submit(2)
            worker.submit(3)
            worker.submit(4)
            release.set()
            self.until(lambda: worker.status()['processed'] == 2)
            self.assertEqual(processed, [1, 4])
            self.assertEqual(worker.take_result(), 4)
            self.assertIsNone(worker.take_result())
            self.assertEqual(worker.status()['replaced'], 2)
        finally:
            release.set()
            worker.close()

    def test_pause_discards_inflight_and_pending_then_resumes(self):
        entered, release = threading.Event(), threading.Event()
        def infer(message):
            if message == 1:
                entered.set()
                release.wait(2)
            return message
        worker = LatestWorker(infer)
        worker.start()
        try:
            worker.submit(1)
            self.assertTrue(entered.wait(1))
            worker.submit(2)
            worker.set_enabled(False)
            worker.submit(3)
            release.set()
            self.until(lambda: worker.status()['processed'] == 1)
            self.assertIsNone(worker.take_result())
            worker.set_enabled(True)
            worker.submit(4)
            self.until(lambda: worker.status()['processed'] == 2)
            self.assertEqual(worker.take_result(), 4)
        finally:
            release.set()
            worker.close()

    def test_exception_is_reported_and_worker_recovers(self):
        def infer(message):
            if message == 1:
                raise ValueError('inference failed')
            return message
        worker = LatestWorker(infer)
        worker.start()
        try:
            worker.submit(1)
            self.until(lambda: worker.status()['errors'] == 1)
            self.assertEqual(worker.status()['last_error'], 'inference failed')
            self.assertIsNone(worker.take_result())
            worker.submit(2)
            self.until(lambda: worker.status()['processed'] == 1)
            self.assertEqual(worker.status()['last_error'], '')
            self.assertEqual(worker.take_result(), 2)
        finally:
            worker.close()

    def test_expired_pending_frame_is_dropped(self):
        worker = LatestWorker(lambda value: value, max_age=.01)
        worker.submit(1)
        time.sleep(.02)
        worker.start()
        try:
            self.until(lambda: worker.status()['stale'] == 1)
            self.assertEqual(worker.status()['processed'], 0)
        finally:
            worker.close()


if __name__ == '__main__':
    unittest.main()
