"""Local-network HTTP application. Run from the project root with -m astra_viewer.server."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import threading
from urllib.parse import parse_qs, urlsplit

from .pipeline import Pipeline, demo_source, ros_source
from .storage import Archive

STATIC = Path(__file__).parent / 'static'
PHOTO = re.compile(r'^/api/photos/([a-f0-9]{32})/(image|thumbnail|download|protect)$')


def handler_for(pipeline, archive, demo):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def log_message(self, fmt, *args):
            # Suppress one-second polling noise; retain errors.
            if args and str(args[1] if len(args) > 1 else '').startswith(('4', '5')):
                super().log_message(fmt, *args)

        def send(self, data, status=200, mime='application/json', extra=None):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' blob:; style-src 'self'; script-src 'self'; connect-src 'self'; frame-ancestors 'none'")
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.end_headers()
            self.wfile.write(data)

        def js(self, data, status=200):
            self.send(json.dumps(data, allow_nan=False).encode(), status)

        def do_GET(self):
            try:
                self.get()
            except (BrokenPipeError, ConnectionResetError, TimeoutError):
                pass
            except (ValueError, KeyError):
                self.js({'error': 'Invalid request'}, 400)

        def get(self):
            url = urlsplit(self.path)
            if url.path in ('/', '/app.js', '/style.css'):
                name, mime = {'/': ('index.html', 'text/html; charset=utf-8'),
                              '/app.js': ('app.js', 'text/javascript'), '/style.css': ('style.css', 'text/css')}[url.path]
                self.send((STATIC / name).read_bytes(), mime=mime)
            elif url.path == '/api/status':
                self.js(dict(**pipeline.status(), archive=archive.status(), demo=demo, camera=pipeline.camera))
            elif url.path == '/api/photos':
                query = parse_qs(url.query)
                limit = max(1, min(100, int(query.get('limit', ['60'])[0])))
                self.js(archive.list(query.get('before', [None])[0], limit))
            elif url.path == '/api/live.jpg':
                with pipeline.condition:
                    jpeg = pipeline.latest[0] if pipeline.latest else None
                if jpeg:
                    self.send(jpeg, mime='image/jpeg')
                else:
                    self.js({'error': 'Waiting for a matched detection frame'}, 503)
            elif match := PHOTO.fullmatch(url.path):
                ident, action = match.groups()
                if action == 'protect':
                    return self.js({'error': 'Use POST'}, 405)
                data = archive.read(ident, action == 'thumbnail')
                if data is None:
                    return self.js({'error': 'This photo is no longer in the archive'}, 404)
                extra = {'Content-Disposition': f'attachment; filename="astra-{ident}.jpg"'} if action == 'download' else None
                self.send(data, mime='image/jpeg', extra=extra)
            else:
                self.js({'error': 'Not found'}, 404)

        def do_POST(self):
            try:
                # JSON + origin check prevents cross-site form submissions on a LAN.
                origin = self.headers.get('Origin')
                if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                    return self.js({'error': 'Origin not allowed'}, 403)
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    return self.js({'error': 'JSON body required'}, 415)
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 1024:
                    return self.js({'error': 'Invalid body length'}, 400)
                body = json.loads(self.rfile.read(length))
                match = PHOTO.fullmatch(urlsplit(self.path).path)
                if not match or match[2] != 'protect':
                    return self.js({'error': 'Not found'}, 404)
                if type(body.get('protected')) is not bool:
                    return self.js({'error': 'protected must be true or false'}, 400)
                if not archive.protect(match[1], body['protected']):
                    return self.js({'error': 'Photo no longer exists'}, 404)
                self.js(archive.get(match[1]))
            except (ValueError, TypeError, AttributeError):
                self.js({'error': 'Invalid request'}, 400)
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description='ASTRA Photo Booth — local camera/detection viewer')
    parser.add_argument('--demo', action='store_true', help='Use synthetic frames; no ROS needed')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--archive', default=None)
    parser.add_argument('--max-gb', type=float, default=10)
    parser.add_argument('--reserve-gb', type=float, default=2)
    parser.add_argument('--interval', type=float, default=1)
    parser.add_argument('--camera', default='ZR10')
    parser.add_argument('--image-topic', default='/zr10/image_raw')
    parser.add_argument('--info-topic', default='/detection/info')
    args = parser.parse_args()
    import math
    if not all(math.isfinite(v) for v in (args.max_gb, args.reserve_gb, args.interval)) or args.max_gb <= 0 or args.reserve_gb < 0 or args.interval < .1:
        parser.error('Use a positive max-gb, nonnegative reserve-gb, and interval >= 0.1 seconds')
    if not args.demo:
        try:
            import rclpy
            import cv_bridge
            import sensor_msgs.msg
            import std_msgs.msg
        except ImportError as exc:
            parser.error(f'ROS environment unavailable ({exc}). Source ROS Humble and your workspace, or use --demo.')
    directory = args.archive or str(Path.home() / '.astra' / ('demo-photos' if args.demo else 'photos'))
    archive = Archive(directory, int(args.max_gb * 1024**3), int(args.reserve_gb * 1024**3))
    pipeline = Pipeline(archive, 'DEMO' if args.demo else args.camera, args.interval)
    httpd = ThreadingHTTPServer((args.host, args.port), handler_for(pipeline, archive, args.demo))
    httpd.daemon_threads = True
    pipeline.start()

    def source():
        try:
            if args.demo:
                demo_source(pipeline)
            else:
                ros_source(pipeline, args.image_topic, args.info_topic)
        except Exception as exc:
            pipeline.source_error = f'Source stopped: {exc}'
            print(pipeline.source_error, flush=True)

    threading.Thread(target=source, daemon=True).start()
    print(f'ASTRA {"DEMO" if args.demo else "ROS"} viewer: http://localhost:{args.port}', flush=True)
    print(f'Other computers: http://<this computer LAN address>:{args.port}', flush=True)
    print(f'Archive: {archive.root}\nPress Ctrl+C to stop.', flush=True)
    try:
        httpd.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.close()
        httpd.server_close()


if __name__ == '__main__':
    main()
