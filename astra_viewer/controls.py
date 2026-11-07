"""Guarded bridge from the LAN UI to the existing ZR10 ROS controls."""
import math
import threading
import time


class ControlError(RuntimeError):
    pass


class ControlBridge:
    ACTIONS = {'photo': 'take_photo', 'autofocus': 'autofocus',
               'record': 'toggle_recording', 'center': 'center_gimbal'}

    def __init__(self, node, enabled=False):
        from geometry_msgs.msg import Vector3
        from rcl_interfaces.srv import GetParameters
        from sensor_msgs.msg import RegionOfInterest
        from std_msgs.msg import Float32, Int8
        from std_srvs.srv import Trigger

        self.node = node
        self.trigger_type = Trigger
        self.roi_type = RegionOfInterest
        self.float32_type = Float32
        self.int8_type = Int8
        self.enabled = bool(enabled)
        self.lock = threading.RLock()
        self.clients = {key: node.create_client(Trigger, f'/{name}') for key, name in self.ACTIONS.items()}
        self.params = node.create_client(GetParameters, '/zr10_camera/get_parameters')
        self.gimbal = node.create_publisher(Vector3, '/gimbal_speed', 10)
        self.zoom = node.create_publisher(Int8, '/zoom_direction', 10)
        self.focus = node.create_publisher(Int8, '/focus_direction', 10)
        self.absolute_zoom = node.create_publisher(Float32, '/absolute_zoom', 10)
        self.focus_point_publisher = node.create_publisher(RegionOfInterest, '/autofocus_point', 10)
        self.dry_run = None
        self.api_version = None
        self.last_status_request = 0.0
        self.armed = set()
        self.armed_last = {}
        self.owner = None
        self.last_motion = 0.0

    def poll(self):
        """Refresh the dry_run state from the camera node while ROS is spinning."""
        now = time.monotonic()
        if not self.enabled or now - self.last_status_request < 2 or not self.params.service_is_ready():
            return
        self.last_status_request = now
        from rcl_interfaces.srv import GetParameters
        request = GetParameters.Request()
        request.names = ['dry_run', 'controller_api_version']
        future = self.params.call_async(request)

        def done(result):
            try:
                values = result.result().values
                with self.lock:
                    self.dry_run = bool(values[0].bool_value) if values else None
                    self.api_version = int(values[1].integer_value) if len(values) > 1 and values[1].type == 2 else None
            except Exception:
                with self.lock:
                    self.dry_run = None
        future.add_done_callback(done)

    def status(self, client_id=None):
        with self.lock:
            services_ready = all(client.service_is_ready() for client in self.clients.values())
            ready = self.enabled and self.dry_run is False and self.api_version == 2 and services_ready
            reason = ''
            if not self.enabled:
                reason = 'Controller is disabled in the viewer settings.'
            elif self.dry_run is None:
                reason = 'Waiting for the ZR10 control node status.'
            elif self.dry_run:
                reason = 'ZR10 control node is in dry-run mode.'
            elif self.api_version != 2:
                reason = 'ZR10 control package needs the updated controller support.'
            elif not services_ready:
                reason = 'ZR10 control services are not all available.'
            return {'available': ready, 'dry_run': self.dry_run,
                    'armed': (client_id in self.armed) if client_id else bool(self.armed),
                    'api_version': self.api_version, 'reason': reason}

    def _require_ready(self, client_id):
        status = self.status()
        if not status['available']:
            raise ControlError(status['reason'] or 'Camera controls are unavailable')
        if client_id not in self.armed:
            raise ControlError('Arm the controller before sending camera commands')
        self.armed_last[client_id] = time.monotonic()

    @staticmethod
    def _client_id(client_id):
        if not isinstance(client_id, str) or not 8 <= len(client_id) <= 80:
            raise ControlError('Invalid controller session')
        return client_id

    def arm(self, client_id, armed):
        client_id = self._client_id(client_id)
        if armed and not self.status()['available']:
            raise ControlError(self.status()['reason'] or 'Camera controls are unavailable')
        with self.lock:
            if armed:
                self.armed.add(client_id)
                self.armed_last[client_id] = time.monotonic()
            else:
                self.armed.discard(client_id)
                self.armed_last.pop(client_id, None)
                self._stop_locked(client_id)
        return self.status()

    def _call(self, client, timeout=2.0):
        if not client.service_is_ready():
            raise ControlError('Camera command service is not available')
        event = threading.Event()
        future = client.call_async(self.trigger_type.Request())
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            raise ControlError('Camera command timed out')
        try:
            result = future.result()
        except Exception as exc:
            raise ControlError('Camera command failed') from exc
        return {'success': bool(result.success), 'message': str(result.message)[:200]}

    def action(self, client_id, action):
        self._client_id(client_id)
        if action not in self.ACTIONS:
            raise ControlError('Unknown camera action')
        with self.lock:
            self._require_ready(client_id)
            return self._call(self.clients[action])

    def motion(self, client_id, yaw, pitch, zoom, focus=0):
        from geometry_msgs.msg import Vector3
        client_id = self._client_id(client_id)
        try:
            yaw, pitch = float(yaw), float(pitch)
            zoom = int(zoom)
            focus = int(focus)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ControlError('Invalid motion values') from exc
        if not math.isfinite(yaw) or not math.isfinite(pitch) or not -100 <= yaw <= 100 or not -100 <= pitch <= 100 or zoom not in (-1, 0, 1) or focus not in (-1, 0, 1):
            raise ControlError('Motion values are outside the allowed range')
        with self.lock:
            self._require_ready(client_id)
            now = time.monotonic()
            if self.owner not in (None, client_id) and now - self.last_motion < 0.35:
                raise ControlError('Another device currently has camera control')
            self.owner = client_id
            self.last_motion = now
            gimbal = Vector3()
            gimbal.x, gimbal.y = yaw, pitch
            self.gimbal.publish(gimbal)
            zoom_msg = self.int8_type()
            zoom_msg.data = zoom
            self.zoom.publish(zoom_msg)
            focus_msg = self.int8_type()
            focus_msg.data = focus
            self.focus.publish(focus_msg)


    def absolute_zoom_to(self, client_id, zoom):
        self._client_id(client_id)
        try:
            value = float(zoom)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ControlError('Invalid zoom value') from exc
        if not math.isfinite(value) or not 1.0 <= value <= 30.0:
            raise ControlError('Zoom must be between 1x and 30x')
        with self.lock:
            self._require_ready(client_id)
            message = self.float32_type()
            message.data = value
            self.absolute_zoom.publish(message)

    def autofocus_point(self, client_id, x, y, width, height):
        self._client_id(client_id)
        try:
            x, y, width, height = (int(x), int(y), int(width), int(height))
        except (TypeError, ValueError, OverflowError) as exc:
            raise ControlError('Invalid focus point') from exc
        if width < 1 or height < 1 or not 0 <= x < width or not 0 <= y < height or width > 65535 or height > 65535:
            raise ControlError('Focus point is outside the live image')
        with self.lock:
            self._require_ready(client_id)
            roi = self.roi_type()
            roi.x_offset, roi.y_offset = x, y
            self.focus_point_publisher.publish(roi)

    def _stop_locked(self, client_id=None):
        if self.owner is not None and client_id not in (None, self.owner):
            return
        from geometry_msgs.msg import Vector3
        try:
            self.gimbal.publish(Vector3())
            zoom_msg = self.int8_type()
            zoom_msg.data = 0
            self.zoom.publish(zoom_msg)
            focus_msg = self.int8_type()
            focus_msg.data = 0
            self.focus.publish(focus_msg)
        finally:
            self.owner = None
            self.last_motion = 0.0

    def stop(self, client_id=None):
        with self.lock:
            self._stop_locked(client_id)

    def watchdog(self):
        with self.lock:
            now = time.monotonic()
            if self.owner is not None and now - self.last_motion > 0.35:
                self._stop_locked(self.owner)
            expired = [client_id for client_id, last in self.armed_last.items() if now - last > 30.0]
            for client_id in expired:
                self.armed.discard(client_id)
                self.armed_last.pop(client_id, None)
                self._stop_locked(client_id)
