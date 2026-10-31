"""Bounded latest-frame reader. No ROS or image library dependencies."""
import os
import selectors
import struct
import subprocess
import threading
import time
import zlib


def png_bytes(width, height, rgb):
    if len(rgb) != width * height * 3:
        raise ValueError('incorrect RGB frame length')
    def chunk(kind, data):
        return (struct.pack('!I', len(data)) + kind + data +
                struct.pack('!I', zlib.crc32(kind + data) & 0xffffffff))
    rows = b''.join(b'\0' + rgb[i:i + width * 3]
                    for i in range(0, len(rgb), width * 3))
    return (b'\x89PNG\r\n\x1a\n' +
            chunk(b'IHDR', struct.pack('!2I5B', width, height, 8, 2, 0, 0, 0)) +
            chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b''))


class FrameSource:
    def __init__(self, command, frame_size, timeout=10.0):
        self.command, self.frame_size, self.timeout = command, frame_size, timeout
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.sequence = 0
        self.error = 'waiting for stream'
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def get(self):
        with self.lock:
            return self.latest, self.error

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=4)
        if self.thread.is_alive():
            raise RuntimeError('stream reader did not stop')

    def _run(self):
        while not self.stop_event.is_set():
            process = None
            try:
                process = subprocess.Popen(self.command, stdout=subprocess.PIPE,
                                           stderr=subprocess.DEVNULL, bufsize=0)
                with selectors.DefaultSelector() as selector:
                    selector.register(process.stdout, selectors.EVENT_READ)
                    pending = bytearray()
                    deadline = time.monotonic() + self.timeout
                    while not self.stop_event.is_set():
                        if time.monotonic() > deadline:
                            raise RuntimeError('stream frame timeout')
                        if not selector.select(0.1):
                            continue
                        part = os.read(process.stdout.fileno(),
                                       min(65536, self.frame_size - len(pending)))
                        if not part:
                            raise RuntimeError('decoder exited or stream ended')
                        pending.extend(part)
                        if len(pending) == self.frame_size:
                            with self.lock:
                                self.sequence += 1
                                self.latest = (self.sequence, time.monotonic(), bytes(pending))
                                self.error = ''
                            pending.clear()
                            deadline = time.monotonic() + self.timeout
            except (OSError, RuntimeError) as exc:
                with self.lock:
                    self.error = str(exc)
                    self.latest = None
            finally:
                if process is not None:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=1)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                    process.stdout.close()
            self.stop_event.wait(1)
