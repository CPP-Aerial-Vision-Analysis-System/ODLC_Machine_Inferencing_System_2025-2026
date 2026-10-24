"""
sim_demo/score_run.py -- PURPOSE: record one simulated search mission and write its score
report. While the mission runs it listens to the detector (/image_detection), the autopilot
(MAVROS), the team's status texts and the photo folder; when the drone has landed (or when
you stop it, or call the /sim/demo/write_score service) it writes into the run folder:
    score_report.md    human-readable report (targets found, confidences, aim, photo GPS and
                       drop errors, detection quality, time line)
    score_report.json  the same numbers for scripts
    run_data.json      every input the score was computed from (to re-score later)
    events.jsonl       a running log of everything that happened (one JSON object per line)

The AIM ERROR (October 2026) comes from main_controller's own status text
"Aim <class> <lat>,<lon>": the point it really flies to and drops on. Runs recorded before that
have no "aims" in run_data.json; --rescore then finds the texts in the run's events.jsonl.

Run during a mission:
    ros2 run sim_demo score_run --ros-args -p world:=runway -p run_dir:=/tmp/run1
Re-score a finished run offline (no ROS needed, e.g. after fixing the truth file):
    ros2 run sim_demo score_run --rescore /tmp/run1 [--world runway] [--truth file.yaml]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import asdict

from .camera_model import DEFAULT_HFOV_RAD
from .detections import DETECTION_TOPIC, Box, DetectionFrame, image_result_to_frame
from .mission_file import analyze_items, load_waypoints_file
from .phases import PhaseTracker
from .scoring import (AimRecord, DropRecord, PhotoRecord, latest_aims, parse_aim_text,
                      report_markdown, score_run_data, write_reports)
from .world_info import load_world

# Two "Aim" texts for the same class and position closer together than this are one text heard
# twice (e.g. on /mavros/statustext/send and again through the autopilot).
AIM_DUPLICATE_WINDOW_S = 5.0


def add_aim(aims: list, t: float, text: str) -> AimRecord | None:
    """Append the AimRecord of an "Aim <class> <lat>,<lon>" text to `aims` (skipping repeats).

    Returns:
        the new AimRecord, or None when the text is not an aim text or a repeat.
    """
    parsed = parse_aim_text(text)
    if parsed is None:
        return None
    cls, lat, lon = parsed
    for a in aims:
        same_point = a.cls == cls and a.lat == lat and a.lon == lon
        if same_point and abs(a.t - t) < AIM_DUPLICATE_WINDOW_S:
            return None
    rec = AimRecord(cls, float(t), lat, lon)
    aims.append(rec)
    return rec


def aims_from_events(events_path: str) -> list[AimRecord]:
    """The "Aim" texts recorded in a run's events.jsonl (for runs without "aims" in run_data)."""
    aims: list[AimRecord] = []
    try:
        with open(events_path, encoding='utf-8') as f:
            for line in f:
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue                      # a half-written last line
                if isinstance(ev, dict) and ev.get('kind') == 'statustext':
                    add_aim(aims, float(ev.get('t', 0.0)), str(ev.get('text', '')))
    except OSError:
        pass                                      # no events file: no aims
    return aims


def default_run_dir(world: str) -> str:
    """Run folder: $ASTRA_RUN_DIR if set, else ~/astra_sim_runs/<world>_<date>_<time>."""
    env = os.environ.get('ASTRA_RUN_DIR', '')
    if env:
        return os.path.abspath(os.path.expanduser(env))
    stamp = time.strftime('%Y%m%d_%H%M%S')
    return os.path.join(os.path.expanduser('~'), 'astra_sim_runs', f'{world}_{stamp}')


def frame_from_dict(d: dict) -> DetectionFrame:
    """Rebuild a DetectionFrame saved in run_data.json."""
    boxes = [Box(**b) for b in d.get('boxes', [])]
    keep = {k: v for k, v in d.items() if k != 'boxes'}
    return DetectionFrame(boxes=boxes, **keep)


def draw_map(run_dir: str, info, frames, photos, drops, track, title: str,
             aims: list[AimRecord] | None = None) -> str:
    """Draw mission_map.png into run_dir (see mission_map.py). Returns the file name or ''.

    The "chosen" rings are where main_controller aims (its "Aim" texts) when they are known,
    else the photo position of the best detection (where the old code flew to).
    """
    try:
        from .detections import best_per_class
        from .mission_map import draw_mission_map
        o = info.origin
        dets = [(b.cls, *o.to_enu(f.lat, f.lon)[:2], b.score) for f in frames if f.has_gps
                for b in f.boxes]
        aimed = latest_aims(aims)
        choices = [(cls, *o.to_enu(a.lat, a.lon)[:2]) for cls, a in aimed.items()]
        choices += [(cls, *o.to_enu(f.lat, f.lon)[:2])
                    for cls, (_s, f, _b) in best_per_class(frames).items()
                    if f.has_gps and cls not in aimed]
        drop_pts = []
        for d in drops:
            if d.landing_en is not None:
                drop_pts.append((d.cls, d.landing_en[0], d.landing_en[1], 'landed'))
            elif d.release_lat is not None:
                drop_pts.append((d.cls, *o.to_enu(d.release_lat, d.release_lon)[:2], 'released'))
        draw_mission_map(os.path.join(run_dir, 'mission_map.png'),
                         [(t.name, t.cls, t.east, t.north) for t in info.targets],
                         info.search_areas, [p.footprint for p in photos if p.footprint],
                         [tuple(p) for p in track], dets, choices, drop_pts, title)
        return 'mission_map.png'
    except Exception as exc:  # noqa: BLE001 - the report must not depend on the picture
        print(f'score_run: mission map not drawn ({exc})', file=sys.stderr)
        return ''


def rescore(run_dir: str, world: str = '', truth: str = '', worlds_dir: str = '') -> str:
    """Recompute the report of a recorded run from its run_data.json. Returns the .md path."""
    with open(os.path.join(run_dir, 'run_data.json'), encoding='utf-8') as f:
        data = json.load(f)
    info = load_world(world or data['world'], worlds_dir or data.get('worlds_dir', ''),
                      truth or data.get('truth_file', ''))
    frames = [frame_from_dict(d) for d in data['frames']]
    photos = [PhotoRecord(**p) for p in data['photos']]
    drops = []
    for d in data['drops']:
        if d.get('landing_en') is not None:
            d = dict(d, landing_en=tuple(d['landing_en']))
        drops.append(DropRecord(**d))
    if 'aims' in data:
        aims = [AimRecord(**a) for a in data['aims']]
    else:                       # recorded before October 2026: look for the texts in events.jsonl
        aims = aims_from_events(os.path.join(run_dir, 'events.jsonl'))
    rs = score_run_data(info.name, info.origin, info.targets, frames, photos, drops,
                        [tuple(x) for x in data['timeline']], data['started'], data['ended'],
                        data.get('detector', 'real'), notes=data.get('notes', []), aims=aims)
    rs.map_file = draw_map(run_dir, info, frames, photos, drops, data.get('track', []),
                           f'{info.name}: mission map', aims=aims)
    md, _ = write_reports(rs, run_dir)
    return md


def _ros_main(args=None) -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    from .payloads import GzPayloadSubscriber, PayloadTracker
    from .photo_geometry import PhotoGeometry
    from .photo_info import PhotoInfoIndex
    from .photos import PhotoWatcher, resolve_photo_dir
    from .ros_common import QOS_DETECTIONS, FlightData, detector_publishers

    class ScoreRunNode(Node):
        """ROS node: records the run and writes the score report."""

        def __init__(self):
            super().__init__('score_run')
            p = self.declare_parameter
            p('world', 'runway')
            p('worlds_dir', '')
            p('truth_file', '')
            p('photo_dir', '')
            p('run_dir', '')                  # '' = $ASTRA_RUN_DIR or ~/astra_sim_runs/<world>_<time>
            p('detector', 'auto')             # label: auto / real / truth
            p('hfov_rad', DEFAULT_HFOV_RAD)
            p('zoom', 0.0)
            p('gimbal_pitch_deg', -90.0)
            p('finish_on_landed', True)       # write the report when the drone lands
            p('linger_s', 8.0)                # wait this long after landing (late detections)
            p('gz_payload_tracking', True)    # measure payload landing points in Gazebo
            p('footprint_margin_m', 2.0)
            p('geo_match_radius_m', 50.0)
            gp = self.get_parameter
            self.world_name = str(gp('world').value)
            self.worlds_dir = str(gp('worlds_dir').value)
            self.truth_file = str(gp('truth_file').value)
            self.run_dir = str(gp('run_dir').value) or default_run_dir(self.world_name)
            os.makedirs(self.run_dir, exist_ok=True)
            self.detector_param = str(gp('detector').value).lower()
            self.finish_on_landed = bool(gp('finish_on_landed').value)
            self.linger = float(gp('linger_s').value)
            self.margin = float(gp('footprint_margin_m').value)
            self.radius = float(gp('geo_match_radius_m').value)
            log = self.get_logger()
            self.world = None
            try:
                self.world = load_world(self.world_name, self.worlds_dir, self.truth_file)
                for w in self.world.warnings:
                    log.warn(w)
            except Exception as exc:  # noqa: BLE001
                log.error(f'Could not load the world ({exc}): the report will have no targets.')
            self.events_path = os.path.join(self.run_dir, 'events.jsonl')
            self._events = open(self.events_path, 'a', encoding='utf-8')
            self.t_start = time.time()
            self.t_armed = None
            self.t_landed = None
            self.written_for = None
            self.phases = PhaseTracker()
            if self.world and self.world.mission_path:
                try:     # lane numbers in the time line ("Searching - lane 2 of 3")
                    self.phases.set_mission(analyze_items(load_waypoints_file(self.world.mission_path),
                                                          self.world.hold_index))
                except Exception as exc:  # noqa: BLE001 - the report works without it
                    log.warn(f'Mission file not readable ({exc}): no lane numbers in the time line')
            self.photo_info = PhotoInfoIndex(self.world.origin) if self.world else None
            self.flight = FlightData(self, zoom_fixed=float(gp('zoom').value),
                                     on_statustext=self._on_text, on_state=self._on_state,
                                     on_reached=self._on_reached, on_rel_alt=self._on_rel_alt,
                                     photo_info=self.photo_info)
            self.photo_dir, how = resolve_photo_dir(str(gp('photo_dir').value))
            self.watcher = PhotoWatcher(self.photo_dir, report_existing=True)
            self.geometry = (PhotoGeometry(self.world.origin, self.flight,
                                           float(gp('hfov_rad').value),
                                           float(gp('gimbal_pitch_deg').value),
                                           photo_info=self.photo_info)
                             if self.world else None)
            self.frames: list[DetectionFrame] = []
            self.photos: list[PhotoRecord] = []
            self.aims: list[AimRecord] = []   # main_controller's "Aim <class> <lat>,<lon>" texts
            self.preexisting = 0
            self.detector_nodes: set[str] = set()
            self.payloads = PayloadTracker()
            self.gz_ok = False
            if bool(gp('gz_payload_tracking').value):
                self.gz_ok = GzPayloadSubscriber(self.payloads, log=lambda s: log.info(s)).start()
            from interfaces.msg import ImageResult
            from std_srvs.srv import Trigger
            self.create_subscription(ImageResult, DETECTION_TOPIC, self._on_detection,
                                     QOS_DETECTIONS)
            self.create_service(Trigger, '/sim/demo/write_score', self._srv_write)
            self.create_timer(0.5, self._tick)
            self.create_timer(3.0, self._who_detects)
            log.info(f'Recording the run into {self.run_dir} (photos: {self.photo_dir}, {how})')
            self._event('start', world=self.world_name, photo_dir=self.photo_dir)

        # -------------------------------------------------------------- recording
        def _event(self, kind: str, **data) -> None:
            try:
                self._events.write(json.dumps(dict(t=round(time.time(), 3), kind=kind, **data)) + '\n')
                self._events.flush()
            except (OSError, ValueError):
                pass

        def _on_text(self, t, text, source):
            self.phases.on_statustext(t, text)
            self._event('statustext', source=source, text=text, phase=self.phases.phase)
            aim = add_aim(self.aims, t, text)       # where main_controller will drop
            if aim is not None:
                self._event('aim', cls=aim.cls, lat=aim.lat, lon=aim.lon, source=source)

        def _on_state(self, t, connected, armed, mode):
            was_armed = self.phases.armed
            self.phases.on_state(t, connected, armed, mode)
            if armed and not was_armed:
                self.t_armed = self.t_armed or t
                self.t_landed = None
                self._event('armed', mode=mode)
            if was_armed and not armed:
                self._event('disarmed', mode=mode, phase=self.phases.phase)
                if self.phases.phase == 'Landed':
                    self.t_landed = t

        def _on_reached(self, t, seq):
            self.phases.on_reached(t, seq)
            self._event('reached', seq=seq, phase=self.phases.phase)

        def _on_rel_alt(self, t, alt):
            self.phases.on_rel_alt(t, alt)

        def _on_detection(self, msg):
            f = image_result_to_frame(msg)
            self.frames.append(f)
            self._event('detection', image=f.image_name, lat=f.lat, lon=f.lon, method=f.method,
                        boxes=[asdict(b) for b in f.boxes])

        def _who_detects(self):
            self.detector_nodes.update(detector_publishers(self, DETECTION_TOPIC))

        def _tick(self):
            for ev in self.watcher.poll():
                if ev.preexisting:
                    self.preexisting += 1
                    continue
                rec = PhotoRecord(ev.name, ev.mtime, ev.lat, ev.lon)
                if self.geometry is not None:
                    sol = self.geometry.solve(ev)
                    rec.footprint = sol.footprint
                    rec.pose_how = sol.how
                    rec.zoom = sol.intr.zoom
                self.photos.append(rec)
                self._event('photo', name=ev.name, lat=ev.lat, lon=ev.lon, pose=rec.pose_how,
                            zoom=rec.zoom, footprint=rec.footprint)
            if (self.finish_on_landed and self.t_landed is not None
                    and time.time() - self.t_landed >= self.linger
                    and self.written_for != self.t_landed):
                self.written_for = self.t_landed
                self.write('the drone landed')

        # -------------------------------------------------------------- report
        def detector_label(self) -> str:
            if self.detector_param in ('real', 'truth'):
                return self.detector_param
            if any(f.is_stand_in for f in self.frames) or 'truth_detector' in self.detector_nodes:
                return 'truth'
            return 'real'

        def collect(self) -> dict:
            started = self.t_armed or self.t_start
            ended = self.t_landed or time.time()
            drops = []
            landings = self.payloads.landing_points()
            for e in self.phases.events:
                if e.kind != 'drop':
                    continue
                cls = e.data.get('cls', '?')
                # Where was the drone when the servo opened? main_controller hovers over the
                # target from "Over <cls>: loiter ..." until the release, and its "Servo" text
                # comes late, so look the position up just after the last "Over <cls>" text.
                overs = [o.t for o in self.phases.events
                         if o.kind == 'over' and o.data.get('cls') == cls and o.t <= e.t]
                t_release = min(e.t, overs[-1] + 3.0) if overs else e.t
                with self.flight.lock:
                    s = self.flight.history.at_time(t_release, max_gap_s=5.0)
                landing = None
                # Each class has one payload. Its landing may be timed BEFORE the drop event:
                # main_controller sends the "Servo N -> 1400" text only after the sleeps in its
                # callback (up to ~10 s after the servo really opened), so allow 30 s.
                for name, (tl, x, y, _z) in landings.items():
                    if cls in name and tl >= e.t - 30.0:
                        landing = (x, y)
                drops.append(DropRecord(cls, e.t, s.lat if s else None, s.lon if s else None,
                                        landing))
            timeline = []
            for e in self.phases.events:
                if e.kind == 'phase':
                    timeline.append((e.t, f'{e.data["phase"]} - {e.data["detail"]}'))
                elif e.kind == 'drop':
                    timeline.append((e.t, f'{e.data.get("cls")} payload released '
                                          f'(servo {e.data.get("channel")})'))
                elif e.kind == 'targets':
                    plan = ' -> '.join(f'{c}@WP{w}' for c, w in e.data.get('plan', []))
                    timeline.append((e.t, f'main_controller chose: {plan}'))
                elif e.kind in ('armed', 'disarmed'):
                    timeline.append((e.t, e.kind))
            seen = set()
            for f in self.frames:
                for b in f.boxes:
                    if b.cls not in seen:
                        seen.add(b.cls)
                        timeline.append((f.received, f'first {b.cls} detection ({b.score:.2f}) in '
                                                     f'{f.image_name}'))
            timeline.sort(key=lambda x: x[0])
            notes = []
            if self.preexisting:
                notes.append(f'{self.preexisting} photos were already in the photo folder when '
                             'recording started; they are not counted.')
            if not self.gz_ok:
                notes.append('Gazebo payload poses were not available; drop errors use the '
                             "drone's position at release.")
            track = []                      # drone track in world metres, a point every >= 2 m
            if self.world is not None:
                with self.flight.lock:
                    samples = self.flight.history.samples(started - 5.0, ended + 5.0)
                for s in samples:
                    e, n, _ = self.world.origin.to_enu(s.lat, s.lon)
                    if not track or math.hypot(e - track[-1][0], n - track[-1][1]) >= 2.0:
                        track.append((round(e, 2), round(n, 2)))
            return dict(world=self.world.name if self.world else self.world_name,
                        worlds_dir=self.worlds_dir, truth_file=self.truth_file,
                        detector=self.detector_label(), started=started, ended=ended,
                        frames=[asdict(f) for f in self.frames],
                        photos=[asdict(p) for p in self.photos],
                        drops=[asdict(d) for d in drops], timeline=timeline, notes=notes,
                        track=track, aims=[asdict(a) for a in self.aims])

        def write(self, reason: str) -> str:
            data = self.collect()
            with open(os.path.join(self.run_dir, 'run_data.json'), 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=1)
            if self.world is None:
                self.get_logger().error('No world loaded: run data saved, no score computed.')
                return ''
            frames = [frame_from_dict(d) for d in data['frames']]
            drops = [DropRecord(**d) for d in data['drops']]
            aims = [AimRecord(**a) for a in data['aims']]
            rs = score_run_data(self.world.name, self.world.origin, self.world.targets, frames,
                                self.photos, drops, data['timeline'], data['started'],
                                data['ended'], data['detector'], self.margin, self.radius,
                                data['notes'], aims=aims)
            rs.map_file = draw_map(self.run_dir, self.world, frames, self.photos, drops,
                                   data['track'], f'{self.world.name}: mission map '
                                   f'({rs.detector} detector)', aims=aims)
            md, js = write_reports(rs, self.run_dir)
            self.get_logger().info(f'Score report written ({reason}): {md}')
            for line in report_markdown(rs).splitlines()[:16]:
                self.get_logger().info(line)
            self._event('report', md=md, json=js, reason=reason)
            return md

        def _srv_write(self, request, response):
            try:
                response.message = self.write('requested') or 'no world loaded'
                response.success = bool(response.message.endswith('.md'))
            except Exception as exc:  # noqa: BLE001
                response.success, response.message = False, str(exc)
            return response

    rclpy.init(args=args)
    node = ScoreRunNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node.written_for is None or node.frames:
                node.write('shutdown')
        except Exception as exc:  # noqa: BLE001
            print(f'score_run: could not write the report at shutdown: {exc}', file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None) -> None:
    """Entry point: `ros2 run sim_demo score_run` (ROS node) or `... --rescore DIR` (offline)."""
    argv = sys.argv[1:] if args is None else list(args)
    if '--rescore' in argv:
        ap = argparse.ArgumentParser(prog='score_run', description='Re-score a recorded run.')
        ap.add_argument('--rescore', required=True, metavar='RUN_DIR')
        ap.add_argument('--world', default='')
        ap.add_argument('--worlds-dir', default='')
        ap.add_argument('--truth', default='')
        ns = ap.parse_args([a for a in argv if not a.startswith('__')])
        md = rescore(ns.rescore, ns.world, ns.truth, ns.worlds_dir)
        print(open(md, encoding='utf-8').read())
        print(f'Written: {md}')
        return
    _ros_main(args)


if __name__ == '__main__':
    main()
