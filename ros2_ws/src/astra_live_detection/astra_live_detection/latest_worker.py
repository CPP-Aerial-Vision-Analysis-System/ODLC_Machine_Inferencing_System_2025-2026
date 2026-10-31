"""One inference in flight, one pending frame, one completed result."""
import threading
import time


class LatestWorker:
    def __init__(self, process, max_age=2.0):
        self.process, self.max_age = process, max_age
        self.condition = threading.Condition()
        self.pending = self.result = None
        self.enabled, self.closed = True, False
        self.epoch = 0
        self.stats = dict(received=0, replaced=0, processed=0, errors=0, stale=0)
        self.last_error = ''
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def submit(self, message):
        with self.condition:
            self.stats['received'] += 1
            if not self.enabled or self.closed:
                return
            if self.pending is not None:
                self.stats['replaced'] += 1
            self.pending = (self.epoch, time.monotonic(), message)
            self.condition.notify()

    def set_enabled(self, enabled):
        with self.condition:
            self.enabled = enabled
            self.epoch += 1
            self.pending = self.result = None

    def take_result(self):
        with self.condition:
            result, self.result = self.result, None
            return result

    def status(self):
        with self.condition:
            return dict(self.stats, enabled=self.enabled, last_error=self.last_error,
                        worker_alive=self.thread.is_alive(), pending=self.pending is not None)

    def close(self):
        with self.condition:
            self.closed = True
            self.pending = self.result = None
            self.condition.notify()
        # GPU inference cannot safely be interrupted; wait before destroying ROS.
        self.thread.join()

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.closed or self.pending is not None)
                if self.closed:
                    return
                epoch, arrived, message = self.pending
                self.pending = None
            if time.monotonic() - arrived > self.max_age:
                with self.condition:
                    self.stats['stale'] += 1
                continue
            try:
                output = self.process(message)
                with self.condition:
                    self.stats['processed'] += 1
                    self.last_error = ''
                    if self.enabled and not self.closed and self.epoch == epoch:
                        self.result = output
            except Exception as exc:
                with self.condition:
                    self.stats['errors'] += 1
                    self.last_error = str(exc)
