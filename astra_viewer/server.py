"""Local-network viewer for live ROS detections and camera SD-card photos."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import re
import threading
from urllib.parse import parse_qs, urlsplit

from .camera_media import CameraMedia, CameraMediaError
from .pipeline import Pipeline, demo_source, ros_source

STATIC = Path(__file__).parent / 'static'
MEDIA_IMAGE = re.compile(r'^/api/media/image/([a-f0-9]{32})$')


def handler_for(pipeline, media, demo):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.settimeout(10)

        def log_message(self, fmt, *args):
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
            except CameraMediaError as exc:
                self.js({'error': str(exc)}, 502)

        def get(self):
            url = urlsplit(self.path)
            if url.path in ('/', '/app.js', '/style.css'):
                name, mime = {'/': ('index.html', 'text/html; charset=utf-8'),
                              '/app.js': ('app.js', 'text/javascript'), '/style.css': ('style.css', 'text/css')}[url.path]
                self.send((STATIC / name).read_bytes(), mime=mime)
            elif url.path == '/api/status':
                client_id = parse_qs(url.query).get('client_id', [None])[0]
                controls = pipeline.controls.status(client_id) if pipeline.controls else {'available': False, 'dry_run': None, 'armed': False, 'reason': 'Camera controls are not connected.'}
                self.js(dict(**pipeline.status(), demo=demo, camera=pipeline.camera,
                             recent_limit=pipeline.recent_limit, controls=controls))
            elif url.path == '/api/live.jpg':
                query = parse_qs(url.query)
                requested = query.get('sequence', [None])[0]
                jpeg = pipeline.result_for(requested)
                if jpeg:
                    self.send(jpeg, mime='image/jpeg')
                else:
                    self.js({'error': 'Matched photo frame is no longer available'}, 503)
            elif url.path == '/api/live.mjpg':
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=astra-frame')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('X-Content-Type-Options', 'nosniff')
                self.send_header('Connection', 'close')
                self.end_headers()
                sequence = 0
                while not pipeline.stop.is_set():
                    frame, sequence = pipeline.wait_preview(sequence, 2.0)
                    if frame is None:
                        continue
                    self.wfile.write(b'--astra-frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
            elif url.path == '/api/media/directories':
                self.js({'directories': media.list_directories()})
            elif url.path == '/api/media/photos':
                query = parse_qs(url.query)
                self.js(media.list_photos(query.get('directory', [''])[0],
                                          int(query.get('start', ['0'])[0]),
                                          int(query.get('count', ['40'])[0])))
            elif match := MEDIA_IMAGE.fullmatch(url.path):
                thumbnail = parse_qs(url.query).get('thumbnail', ['0'])[0] == '1'
                data = media.image(match[1], thumbnail)
                download = parse_qs(url.query).get('download', ['0'])[0] == '1'
                extra = {'Content-Disposition': f'attachment; filename="zr10-{match[1]}.jpg"'} if download and not thumbnail else None
                self.send(data, mime='image/jpeg', extra=extra)
            else:
                self.js({'error': 'Not found'}, 404)

        def do_POST(self):
            try:
                origin = self.headers.get('Origin')
                if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                    return self.js({'error': 'Origin not allowed'}, 403)
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    return self.js({'error': 'JSON body required'}, 415)
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    return self.js({'error': 'Invalid body length'}, 400)
                body = json.loads(self.rfile.read(length))
                if not isinstance(body, dict) or not pipeline.controls:
                    return self.js({'error': 'Camera controls are not connected'}, 503)
                path = urlsplit(self.path).path
                if path == '/api/control/stop':
                    pipeline.controls.stop(body.get('client_id'))
                    return self.js({'success': True})
                if path == '/api/control/arm':
                    armed = body.get('armed')
                    if type(armed) is not bool:
                        return self.js({'error': 'armed must be true or false'}, 400)
                    return self.js(pipeline.controls.arm(body.get('client_id'), armed))
                if path == '/api/control/motion':
                    pipeline.controls.motion(body.get('client_id'), body.get('yaw', 0), body.get('pitch', 0), body.get('zoom', 0), body.get('focus', 0))
                    return self.js({'success': True})
                if path == '/api/control/action':
                    return self.js(pipeline.controls.action(body.get('client_id'), body.get('action')))
                if path == '/api/control/zoom':
                    pipeline.controls.absolute_zoom_to(body.get('client_id'), body.get('zoom'))
                    return self.js({'success': True})
                if path == '/api/control/autofocus-point':
                    pipeline.controls.autofocus_point(body.get('client_id'), body.get('x'), body.get('y'), body.get('width'), body.get('height'))
                    return self.js({'success': True})
                return self.js({'error': 'Not found'}, 404)
            except Exception as exc:
                from .controls import ControlError
                if isinstance(exc, ControlError):
                    return self.js({'error': str(exc)}, 409)
                if isinstance(exc, (ValueError, TypeError, AttributeError)):
                    return self.js({'error': 'Invalid request'}, 400)
                if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
                    return
                self.js({'error': 'Camera command failed'}, 502)

    return Handler


def main():
    parser = argparse.ArgumentParser(description='ASTRA Photo Booth — local camera/detection viewer')
    parser.add_argument('--demo', action='store_true', help='Use synthetic frames; no ROS needed')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--interval', type=float, default=1)
    parser.add_argument('--recent-limit', type=int, default=60, help='Recent photos retained in each browser tab memory')
    parser.add_argument('--preview-fps', type=float, default=5.0, help='Low-resolution browser preview rate (1–10 FPS)')
    parser.add_argument('--camera', default='ZR10')
    parser.add_argument('--camera-ip', default='192.168.144.25')
    parser.add_argument('--media-port', type=int, default=82)
    parser.add_argument('--enable-controller', action='store_true', help='Allow the browser to use live camera controls; also requires the ROS node dry_run:=false')
    parser.add_argument('--image-topic', default='/zr10/image_raw')
    parser.add_argument('--info-topic', default='/detection/info')
    args = parser.parse_args()
    import math
    if not math.isfinite(args.interval) or args.interval < .1 or not math.isfinite(args.preview_fps) or not 1 <= args.preview_fps <= 10 or not 1 <= args.recent_limit <= 240:
        parser.error('Use interval >= 0.1 seconds, preview-fps between 1 and 10, and recent-limit between 1 and 240')
    if not args.demo:
        try:
            import rclpy, cv_bridge, sensor_msgs.msg, std_msgs.msg
        except ImportError as exc:
            parser.error(f'ROS environment unavailable ({exc}). Source ROS Humble and your workspace, or use --demo.')
    pipeline = Pipeline(None, 'DEMO' if args.demo else args.camera, args.interval, recent_limit=args.recent_limit, preview_fps=args.preview_fps)
    media = CameraMedia(args.camera_ip, args.media_port)
    httpd = ThreadingHTTPServer((args.host, args.port), handler_for(pipeline, media, args.demo))
    httpd.daemon_threads = True
    pipeline.start()

    def source():
        try:
            if args.demo:
                demo_source(pipeline)
            else:
                ros_source(pipeline, args.image_topic, args.info_topic, args.enable_controller)
        except Exception as exc:
            pipeline.source_error = f'Source stopped: {exc}'
            print(pipeline.source_error, flush=True)

    threading.Thread(target=source, daemon=True).start()
    print(f'ASTRA {"DEMO" if args.demo else "ROS"} viewer: http://localhost:{args.port}', flush=True)
    print(f'Other computers: http://<Jetson LAN address>:{args.port}', flush=True)
    print(f'Recent photos are kept in each browser tab memory (limit {args.recent_limit}); camera SD photos are streamed on demand. Preview capped at {args.preview_fps:g} FPS.', flush=True)
    if args.enable_controller:
        print('Controller access requested; it remains unavailable unless /zr10_camera reports dry_run=false.', flush=True)
    print('Press Ctrl+C to stop.', flush=True)
    try:
        httpd.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        if pipeline.controls:
            pipeline.controls.stop()
        pipeline.close()
        httpd.server_close()


if __name__ == '__main__':
    main()
