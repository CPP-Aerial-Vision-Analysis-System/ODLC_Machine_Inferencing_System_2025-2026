import shutil
import struct
import sys
import time
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from frame_source import FrameSource, png_bytes


class FramesTest(unittest.TestCase):
    def wait_frame(self, source):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            frame, _ = source.get()
            if frame:
                return frame
            time.sleep(0.02)
        self.fail('no frame received')

    def test_partial_reads_and_latest_only(self):
        code = ('import os,time\n'
                'for i in range(40):\n'
                ' os.write(1,bytes([i])*2);time.sleep(.002)\n'
                ' os.write(1,bytes([i])*4);time.sleep(.002)\n'
                'time.sleep(10)\n')
        source = FrameSource([sys.executable, '-c', code], 6)
        source.start()
        try:
            self.wait_frame(source)
            time.sleep(.3)
            frame, _ = source.get()
            self.assertEqual(frame[0], 40)
            self.assertEqual(frame[2], bytes([39]) * 6)
        finally:
            source.close()
        self.assertFalse(source.thread.is_alive())

    def test_stall_discards_stale_frame_and_reconnects(self):
        source = FrameSource([sys.executable, '-c',
                              'import os,time;os.write(1,b"abcdef");time.sleep(10)'],
                             6, timeout=.15)
        source.start()
        try:
            first = self.wait_frame(source)
            time.sleep(.3)
            self.assertIsNone(source.get()[0])
            second = self.wait_frame(source)
            self.assertGreater(second[0], first[0])
        finally:
            source.close()

    def test_missing_decoder(self):
        source = FrameSource(['/nonexistent/zr10_ffmpeg'], 6)
        source.start()
        try:
            time.sleep(.1)
            self.assertIsNone(source.get()[0])
            self.assertIn('No such file', source.get()[1])
        finally:
            source.close()

    @unittest.skipUnless(shutil.which('ffmpeg'), 'FFmpeg not installed')
    def test_actual_ffmpeg_rgb_and_png(self):
        source = FrameSource(['ffmpeg', '-nostdin', '-loglevel', 'error', '-re',
                              '-f', 'lavfi', '-i', 'color=c=red:s=16x16:r=10',
                              '-pix_fmt', 'rgb24', '-f', 'rawvideo', 'pipe:1'], 768)
        source.start()
        try:
            rgb = self.wait_frame(source)[2]
            self.assertGreater(rgb[0], 240)
            self.assertLess(rgb[1], 10)
            png = png_bytes(16, 16, rgb)
            self.assertEqual(png[:8], b'\x89PNG\r\n\x1a\n')
            position, compressed = 8, b''
            while position < len(png):
                size = struct.unpack('!I', png[position:position + 4])[0]
                kind = png[position + 4:position + 8]
                data = png[position + 8:position + 8 + size]
                crc = struct.unpack('!I', png[position + 8 + size:position + 12 + size])[0]
                self.assertEqual(crc, zlib.crc32(kind + data) & 0xffffffff)
                if kind == b'IDAT':
                    compressed += data
                position += 12 + size
            rows = zlib.decompress(compressed)
            self.assertEqual(b''.join(rows[i + 1:i + 49] for i in range(0, len(rows), 49)), rgb)
        finally:
            source.close()


if __name__ == '__main__':
    unittest.main()
