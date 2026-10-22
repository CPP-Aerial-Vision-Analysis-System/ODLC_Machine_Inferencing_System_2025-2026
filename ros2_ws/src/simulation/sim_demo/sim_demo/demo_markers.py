"""
sim_demo/demo_markers.py -- PURPOSE: draw the search mission in RViz. Publishes one
visualization_msgs/MarkerArray on /sim/demo/markers (twice a second, latched) with:
    search_area   yellow outline of the search area (truth file, else the survey lanes' box)
    targets       the real hiker (red) and tent (orange) with labels, from the answer key
    footprints    translucent blue patches: the ground each photo covered
    detections    spheres + labels where the detector says it saw a person/tent
    best          the detection main_controller will fly to, per class (big, labelled)
    route         the mission route from the autopilot (white line + waypoint dots)
    track         where the drone actually flew (cyan line), and the drone itself (arrow)
    drops         where each payload was released (magenta) and where it landed (Gazebo)
All markers use one fixed frame (parameter frame_id, default "world") = Gazebo world ENU with
its origin at the SITL home, so they line up with the simulator. GPS is converted with the
world's home from sim_worlds config/<world>.yaml.

Run:  ros2 run sim_demo demo_markers --ros-args -p world:=runway
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .camera_model import DEFAULT_HFOV_RAD
from .detections import DETECTION_TOPIC, best_per_class, image_result_to_frame
from .mission_file import analyze_items, load_waypoints_file, survey_bounds_en
from .payloads import GzPayloadSubscriber, PayloadTracker
from .phases import PhaseTracker
from .photo_geometry import PhotoGeometry
from .photo_info import PhotoInfoIndex
from .photos import PhotoWatcher, resolve_photo_dir
from .ros_common import QOS_DETECTIONS, QOS_LATCHED, FlightData
from .world_info import load_world

COLORS = {
    'person': (1.0, 0.25, 0.25), 'tent': (1.0, 0.6, 0.1), 'area': (1.0, 0.9, 0.2),
    'footprint': (0.3, 0.7, 1.0), 'track': (0.2, 0.95, 1.0), 'route': (0.95, 0.95, 0.95),
    'drop': (1.0, 0.2, 1.0), 'landed': (0.6, 0.2, 1.0), 'drone': (0.2, 1.0, 0.4),
}


class DemoMarkersNode(Node):
    """ROS node publishing the RViz picture of the mission."""

    def __init__(self):
        super().__init__('demo_markers')
        p = self.declare_parameter
        p('world', 'runway')
        p('worlds_dir', '')
        p('truth_file', '')
        p('photo_dir', '')
        p('frame_id', 'world')
        p('publish_rate_hz', 2.0)
        p('hfov_rad', DEFAULT_HFOV_RAD)
        p('zoom', 0.0)
        p('gimbal_pitch_deg', -90.0)
        p('max_footprints', 800)
        p('max_detection_markers', 60)
        p('track_min_step_m', 1.0)
        p('gz_payload_tracking', True)
        gp = self.get_parameter
        self.frame_id = str(gp('frame_id').value)
        self.max_fp = int(gp('max_footprints').value)
        self.max_det = int(gp('max_detection_markers').value)
        self.track_step = float(gp('track_min_step_m').value)
        log = self.get_logger()
        self.world = None
        try:
            self.world = load_world(str(gp('world').value), str(gp('worlds_dir').value),
                                    str(gp('truth_file').value))
            for w in self.world.warnings:
                log.warn(w)
        except Exception as exc:  # noqa: BLE001
            log.error(f'Could not load the world ({exc}); only the drone track will be drawn '
                      '(relative to the first GPS fix).')
        self.mission_items = []
        # Every search area as (name, polygon); the truth file's first, else the survey's box.
        self.areas = list(self.world.search_areas) if self.world else []
        if self.world and self.world.mission_path:
            try:
                self.mission_items = load_waypoints_file(self.world.mission_path)
                if not self.areas:
                    box = survey_bounds_en(self.mission_items,
                                           analyze_items(self.mission_items, self.world.hold_index),
                                           self.world.origin)
                    if box:
                        self.areas = [('survey lanes', box)]
            except Exception as exc:  # noqa: BLE001
                log.warn(f'Mission file not readable ({exc}); route comes from MAVROS only.')
        self.origin = self.world.origin if self.world else None
        self.phases = PhaseTracker()
        self.photo_info = PhotoInfoIndex(self.origin) if self.origin else None
        self.flight = FlightData(self, zoom_fixed=float(gp('zoom').value),
                                 on_statustext=self._on_text, photo_info=self.photo_info)
        self.photo_dir, how = resolve_photo_dir(str(gp('photo_dir').value))
        self.watcher = PhotoWatcher(self.photo_dir, report_existing=False)
        log.info(f'Photo footprints from {self.photo_dir} ({how}); frame "{self.frame_id}"')
        self.geometry = None
        if self.origin is not None:
            self.geometry = PhotoGeometry(self.origin, self.flight, float(gp('hfov_rad').value),
                                          float(gp('gimbal_pitch_deg').value),
                                          photo_info=self.photo_info)
        self.footprints: list[list[tuple[float, float]]] = []
        self.frames = []
        self.track: list[tuple[float, float, float]] = []
        self.drops: list[tuple[str, float, float]] = []     # (cls, east, north)
        self.payloads = PayloadTracker()
        if bool(gp('gz_payload_tracking').value):
            GzPayloadSubscriber(self.payloads, log=lambda s: log.info(s)).start()

        from interfaces.msg import ImageResult
        from visualization_msgs.msg import Marker, MarkerArray
        self.Marker, self.MarkerArray = Marker, MarkerArray
        self.create_subscription(ImageResult, DETECTION_TOPIC, self._on_detection, QOS_DETECTIONS)
        self.pub = self.create_publisher(MarkerArray, '/sim/demo/markers', QOS_LATCHED)
        self.create_timer(0.5, self._poll_photos)
        self.create_timer(1.0 / max(0.2, float(gp('publish_rate_hz').value)), self._publish)
        self._cleared = False

    # ------------------------------------------------------------------ inputs
    def _en(self, lat: float, lon: float) -> tuple[float, float]:
        if self.origin is None:
            from .geo import GeoOrigin
            self.origin = GeoOrigin(lat, lon, 0.0)
        e, n, _ = self.origin.to_enu(lat, lon)
        return e, n

    def _on_text(self, t: float, text: str, source: str) -> None:
        before = len([e for e in self.phases.events if e.kind == 'drop'])
        self.phases.on_statustext(t, text)
        drops = [e for e in self.phases.events if e.kind == 'drop']
        if len(drops) > before:
            cls = drops[-1].data.get('cls', '?')
            # main_controller's "Servo" text arrives late (it sleeps in its callback); the drone
            # hovered over the target since its "Over <cls>" text: take the position from then.
            overs = [o.t for o in self.phases.events if o.kind == 'over' and o.data.get('cls') == cls]
            t_release = min(t, overs[-1] + 3.0) if overs else t
            with self.flight.lock:
                s = self.flight.history.at_time(t_release, max_gap_s=5.0)
            if s is not None:
                e, n = self._en(s.lat, s.lon)
                self.drops.append((cls, e, n))

    def _on_detection(self, msg) -> None:
        self.frames.append(image_result_to_frame(msg))
        if len(self.frames) > 5000:
            del self.frames[:1000]

    def _poll_photos(self) -> None:
        for ev in self.watcher.poll():
            if self.geometry is None:
                continue
            sol = self.geometry.solve(ev)
            if sol.footprint:
                self.footprints.append(sol.footprint)
                if len(self.footprints) > self.max_fp:
                    del self.footprints[: len(self.footprints) - self.max_fp]
        snap = self.flight.snapshot()
        if snap['lat'] is not None:
            e, n = self._en(snap['lat'], snap['lon'])
            z = snap['rel_alt'] if not math.isnan(snap['rel_alt']) else 0.0
            if not self.track or math.dist(self.track[-1], (e, n, z)) >= self.track_step:
                self.track.append((e, n, z))
                if len(self.track) > 20000:
                    del self.track[:5000]

    # ------------------------------------------------------------------ markers
    def _marker(self, ns: str, mid: int, mtype: int, color, alpha: float = 1.0):
        m = self.Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, self.Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b = color
        m.color.a = alpha
        m.scale.x = m.scale.y = m.scale.z = 1.0
        return m

    def _pt(self, x: float, y: float, z: float = 0.0):
        from geometry_msgs.msg import Point
        return Point(x=float(x), y=float(y), z=float(z))

    def _text(self, ns: str, mid: int, x: float, y: float, z: float, text: str, color, size=2.0):
        m = self._marker(ns, mid, self.Marker.TEXT_VIEW_FACING, color)
        m.pose.position = self._pt(x, y, z)
        m.scale.z = size
        m.text = text
        return m

    def _publish(self) -> None:
        M = self.Marker
        arr = self.MarkerArray()
        if not self._cleared:
            clear = M()
            clear.action = M.DELETEALL
            arr.markers.append(clear)
            self._cleared = True
        # Search area outlines.
        for k, (name, poly) in enumerate(self.areas):
            m = self._marker('search_area', 2 * k, M.LINE_STRIP, COLORS['area'], 0.9)
            m.scale.x = 0.6
            m.points = [self._pt(x, y, 0.08) for x, y in list(poly) + list(poly[:1])]
            arr.markers.append(m)
            cx = sum(p[0] for p in poly) / len(poly)
            ymax = max(p[1] for p in poly)
            arr.markers.append(self._text('search_area', 2 * k + 1, cx, ymax + 4, 1.0,
                                          f'SEARCH AREA ({name})', COLORS['area'], 3.0))
        # Truth targets.
        if self.world:
            for i, t in enumerate(self.world.targets):
                col = COLORS.get(t.cls, (1, 1, 1))
                body = self._marker('targets', 3 * i, M.CUBE, col, 0.95)
                body.pose.position = self._pt(t.east, t.north, t.up + t.size[2] / 2)
                yaw = math.radians(t.yaw_deg)
                body.pose.orientation.z, body.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                body.scale.x, body.scale.y, body.scale.z = t.size
                ring = self._marker('targets', 3 * i + 1, M.CYLINDER, col, 0.25)
                ring.pose.position = self._pt(t.east, t.north, 0.03)
                ring.scale.x = ring.scale.y = 8.0
                ring.scale.z = 0.05
                label = ('HIKER' if t.cls == 'person' else 'TENT') + f' ({t.name})'
                arr.markers += [body, ring,
                                self._text('targets', 3 * i + 2, t.east, t.north, 5.0, label, col)]
        # Photo footprints (all faint, newest brighter).
        if self.footprints:
            tri = self._marker('footprints', 0, M.TRIANGLE_LIST, COLORS['footprint'], 0.10)
            lines = self._marker('footprints', 1, M.LINE_LIST, COLORS['footprint'], 0.45)
            lines.scale.x = 0.15
            for fp in self.footprints:
                a, b, c, d = [self._pt(x, y, 0.05) for x, y in fp]
                tri.points += [a, b, c, a, c, d]
                lines.points += [a, b, b, c, c, d, d, a]
            last = self._marker('footprints', 2, M.LINE_STRIP, COLORS['footprint'], 1.0)
            last.scale.x = 0.4
            last.points = [self._pt(x, y, 0.1) for x, y in self.footprints[-1] + self.footprints[-1][:1]]
            arr.markers += [tri, lines, last]
        # Detections (newest N) and the best per class.
        dets = [(f, b) for f in self.frames if f.has_gps for b in f.boxes]
        for i, (f, b) in enumerate(dets[-self.max_det:]):
            e, n = self._en(f.lat, f.lon)
            col = COLORS.get(b.cls, (1, 1, 1))
            s = self._marker('detections', i, M.SPHERE, col, 0.7)
            s.pose.position = self._pt(e, n, 1.0)
            s.scale.x = s.scale.y = s.scale.z = 1.2
            arr.markers.append(s)
        for j, (cls, (score, f, _b)) in enumerate(sorted(best_per_class(self.frames).items())):
            if not f.has_gps:
                continue
            e, n = self._en(f.lat, f.lon)
            col = COLORS.get(cls, (1, 1, 1))
            s = self._marker('best', 2 * j, M.SPHERE, col, 1.0)
            s.pose.position = self._pt(e, n, 1.5)
            s.scale.x = s.scale.y = s.scale.z = 2.5
            arr.markers += [s, self._text('best', 2 * j + 1, e, n, 6.0,
                                          f'BEST {cls.upper()} {score:.2f} (flies here)', col, 2.2)]
        # Mission route (from the autopilot if known, else from the file).
        with self.flight.lock:
            wps = list(self.flight.waypoints)
        if not wps and self.mission_items:
            wps = [(it.command, it.lat, it.lon, it.alt) for it in self.mission_items]
        pts = [(self._en(la, lo), alt) for cmd, la, lo, alt in wps[1:]
               if not (la == 0.0 and lo == 0.0) and cmd in (16, 17, 18, 19, 21, 22, 82)]
        if len(pts) >= 2:
            route = self._marker('route', 0, M.LINE_STRIP, COLORS['route'], 0.6)
            route.scale.x = 0.2
            route.points = [self._pt(e, n, alt) for (e, n), alt in pts]
            dots = self._marker('route', 1, M.SPHERE_LIST, COLORS['route'], 0.9)
            dots.scale.x = dots.scale.y = dots.scale.z = 1.0
            dots.points = list(route.points)
            arr.markers += [route, dots]
        # Track and drone.
        if len(self.track) >= 2:
            tr = self._marker('track', 0, M.LINE_STRIP, COLORS['track'], 0.9)
            tr.scale.x = 0.3
            tr.points = [self._pt(*p) for p in self.track]
            arr.markers.append(tr)
        snap = self.flight.snapshot()
        if self.track:
            dr = self._marker('drone', 0, M.ARROW, COLORS['drone'], 1.0)
            x, y, z = self.track[-1]
            dr.pose.position = self._pt(x, y, z)
            hdg = snap['heading'] if not math.isnan(snap['heading']) else 0.0
            yaw = math.radians(90.0 - hdg)
            dr.pose.orientation.z, dr.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            dr.scale.x, dr.scale.y, dr.scale.z = 4.0, 1.2, 1.2
            arr.markers.append(dr)
        # Drops.
        for k, (cls, e, n) in enumerate(self.drops):
            m = self._marker('drops', 2 * k, M.CYLINDER, COLORS['drop'], 0.9)
            m.pose.position = self._pt(e, n, 0.1)
            m.scale.x = m.scale.y = 1.5
            m.scale.z = 0.2
            arr.markers += [m, self._text('drops', 2 * k + 1, e, n, 3.0, f'{cls} payload released',
                                          COLORS['drop'], 1.6)]
        for k, (name, (t, x, y, z)) in enumerate(sorted(self.payloads.landing_points().items())):
            m = self._marker('landed', k, M.SPHERE, COLORS['landed'], 1.0)
            m.pose.position = self._pt(x, y, max(0.2, z))
            m.scale.x = m.scale.y = m.scale.z = 1.0
            arr.markers.append(m)
        self.pub.publish(arr)


def main(args=None):
    """Entry point: ros2 run sim_demo demo_markers."""
    rclpy.init(args=args)
    node = DemoMarkersNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
