"""
sim_bringup/phase_monitor.py -- PURPOSE: tell the person running the demo, in plain words, what the drone is doing.

It listens to the same ROS 2 topics a ground station would see and prints one line per mission phase:
  ARMED -> TAKEOFF -> SEARCH (lanes + a photo every second) -> HOLD (detector finishes) -> TARGETS (target
  waypoints inserted) -> DROP (payload released) -> RTL (return home) -> LANDED
plus every detection (class, confidence, lat/lon), a photo counter and a short status line every 20 s.
When the drone has flown, landed and disarmed it prints a summary and exits with code 0, which is how
simulation/scripts/demo.sh knows the demo is finished.

Where the information comes from (all published by MAVROS or by the team's own nodes, nothing simulated):
  /mavros/state                 mode + armed                        (mavros_msgs/State)
  /mavros/extended_state        landed / in air                     (mavros_msgs/ExtendedState)
  /mavros/global_position/rel_alt   altitude above home, m          (std_msgs/Float64)
  /mavros/statustext/recv       autopilot texts: "Mission: 3 DigiCamCtrl", "Mission: 1 Takeoff", ...
  /mavros/statustext/send       team node texts: "Camera trigger STARTED", "SUCCESS: Captured 2K image #4",
                                "Detected person", "Targets: tent@9 -> person@10. Diverting.", "Servo 13 -> 1400"
  /image_detection              the detector's results (interfaces/ImageResult), when interfaces is built
  the team photo folder         counts the photos video_cam saved (<team ros2_ws>/src/video_cam/mapping_photos)

Usage:
  ros2 run sim_bringup phase_monitor [--log FILE.jsonl] [--photo-dir DIR] [--timeout SECONDS] [--no-exit]
Exit codes: 0 = flown, landed and disarmed; 3 = timeout; 130 = Ctrl+C.
"""

import argparse
import json
import os
import re
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

#: The phases in order, with the sentence printed when the phase starts.
PHASES: List[Tuple[str, str]] = [
    ('WAITING', 'waiting for the drone to be armed'),
    ('ARMED', 'motors armed, the mission is starting'),
    ('TAKEOFF', 'taking off to the search altitude'),
    ('SEARCH', 'flying the search lanes; the camera takes a photo every second'),
    ('HOLD', 'last search waypoint reached: hovering while the detector finishes the photos'),
    ('TARGETS', 'target waypoints inserted into the mission: flying to the targets'),
    ('DROP', 'over a target: releasing a payload'),
    ('RTL', 'returning home (RTL)'),
    ('LANDED', 'landed and disarmed: the demo is finished'),
]
PHASE_INDEX = {name: i for i, (name, _) in enumerate(PHASES)}

#: servo channel -> payload name (main_controller: channel 11 = person/hiker bottle, 13 = tent beacon)
SERVO_PAYLOADS = {11: 'person (hiker) payload', 13: 'tent payload'}


@dataclass
class Event:
    """One thing worth printing. kind: 'phase', 'detection', 'photo', 'info', 'warn'."""
    kind: str
    text: str
    phase: str = ''


@dataclass
class MissionState:
    """Pure-Python state machine fed with texts and numbers (unit-tested without ROS).

    Every on_* method returns the list of Events to print. The ROS node only forwards messages here.
    """
    phase: str = 'WAITING'
    armed: bool = False
    mode: str = ''
    airborne: bool = False
    rel_alt: Optional[float] = None
    landed_state: int = 0              # mavros ExtendedState: 1 on ground, 2 in air, 3 takeoff, 4 landing
    photos: int = 0
    detections: Dict[str, List[float]] = field(default_factory=lambda: {'person': [], 'tent': []})
    drops: List[str] = field(default_factory=list)
    targets_text: str = ''
    started: float = field(default_factory=time.monotonic)
    finished: bool = False
    # time the autopilot announced the mission's RETURN_TO_LAUNCH item; it only counts as the RTL phase if the
    # drone is still in AUTO a few seconds later (main_controller normally switches to GUIDED to hold first)
    pending_rtl: Optional[float] = None

    # -- helpers ---------------------------------------------------------------------------------
    def _advance(self, new: str, extra: str = '') -> List[Event]:
        """Move forward to phase `new` (never backwards, except DROP<->TARGETS); return the phase event."""
        if new == self.phase:
            return []
        old_i, new_i = PHASE_INDEX[self.phase], PHASE_INDEX[new]
        if new_i < old_i and not (self.phase == 'DROP' and new == 'TARGETS'):
            return []
        self.phase = new
        sentence = dict(PHASES)[new]
        return [Event('phase', f'{sentence}{" (" + extra + ")" if extra else ""}', phase=new)]

    # -- inputs ----------------------------------------------------------------------------------
    def on_state(self, armed: bool, mode: str, now: Optional[float] = None) -> List[Event]:
        """/mavros/state (about 1 Hz): arming, mode changes (RTL mode starts the RTL phase)."""
        now = time.monotonic() if now is None else now
        events: List[Event] = []
        if mode and mode != self.mode:
            if self.mode:
                events.append(Event('info', f'flight mode {self.mode} -> {mode}'))
            self.mode = mode
            if mode == 'RTL' and self.phase not in ('WAITING', 'LANDED'):
                events += self._advance('RTL')
        if self.pending_rtl is not None:
            if mode != 'AUTO':
                self.pending_rtl = None                 # main_controller took over (GUIDED hold)
            elif now - self.pending_rtl >= 3.0:
                self.pending_rtl = None                 # the mission's own RTL item is really flying
                events += self._advance('RTL', 'mission RETURN_TO_LAUNCH item')
        if armed and not self.armed:
            events += self._advance('ARMED')
        if not armed and self.armed:
            events.append(Event('info', 'motors disarmed'))
        self.armed = armed
        events += self._check_landed()
        return events

    def on_altitude(self, rel_alt: float) -> List[Event]:
        """/mavros/global_position/rel_alt: notices take-off (above 3 m) and the landing."""
        self.rel_alt = rel_alt
        events: List[Event] = []
        if rel_alt > 3.0 and self.armed and not self.airborne:
            self.airborne = True
            events += self._advance('TAKEOFF', f'{rel_alt:.0f} m')
        events += self._check_landed()
        return events

    def on_landed_state(self, landed_state: int) -> List[Event]:
        """/mavros/extended_state landed_state (2 = in air marks the flight as airborne)."""
        self.landed_state = landed_state
        if landed_state == 2:
            self.airborne = True
        return self._check_landed()

    def _check_landed(self) -> List[Event]:
        """Finished = has flown, is disarmed, and is on the ground (landed_state 1 or altitude < 1.5 m)."""
        if self.finished or not self.airborne or self.armed:
            return []
        on_ground = self.landed_state == 1 or (self.rel_alt is not None and self.rel_alt < 1.5)
        if not on_ground:
            return []
        self.finished = True
        return self._advance('LANDED')

    def on_autopilot_text(self, text: str, now: Optional[float] = None) -> List[Event]:
        """/mavros/statustext/recv: ArduPilot's own messages ("Mission: 3 DigiCamCtrl", ...)."""
        now = time.monotonic() if now is None else now
        events: List[Event] = []
        m = re.match(r'Mission:\s*(\d+)\s+(\w+)', text)
        if m:
            item, kind = int(m.group(1)), m.group(2)
            if kind == 'Takeoff':
                events += self._advance('TAKEOFF', f'mission item {item}')
            elif kind == 'DigiCamCtrl':
                events += self._advance('SEARCH', f'DigiCamCtrl at mission item {item}')
            elif kind in ('WP', 'SplineWP') and self.phase in ('SEARCH', 'TARGETS'):
                events.append(Event('info', f'heading for mission item {item} ({kind})'))
            elif kind == 'RTL':
                # ArduPilot announces the RTL item the moment the last waypoint is reached, even when
                # main_controller switches to GUIDED right then: decide in on_state.
                self.pending_rtl = now
            return events
        if text.startswith(('PreArm', 'Arm:')):
            return [Event('warn', f'autopilot: {text}')]
        lowered = text.lower()
        if any(word in lowered for word in ('disarming', 'land complete', 'crash', 'failsafe', 'ekf')):
            events.append(Event('info', f'autopilot: {text}'))
        return events

    def on_team_text(self, text: str) -> List[Event]:
        """/mavros/statustext/send: what the team's nodes tell the ground station."""
        events: List[Event] = []
        if 'Camera trigger STARTED' in text:
            events += self._advance('SEARCH', 'the team\'s trigger started the photos')
        elif text.startswith('SUCCESS: Captured'):
            pass                                        # counted from the photo folder instead
        elif text.startswith('Detected '):
            events.append(Event('info', f'main_controller: {text}'))
        elif text.startswith('Reached last WP') or 'Holding to finish' in text:
            events += self._advance('HOLD')
        elif text.startswith(('All ', 'Processing timeout')) and 'images' in text:
            events.append(Event('info', f'main_controller: {text}'))
        elif text.startswith('Targets:'):
            self.targets_text = text
            events += self._advance('TARGETS', text)
        elif text.startswith('No valid targets'):
            events.append(Event('warn', f'main_controller: {text}'))
        elif text.startswith('Over '):
            events += self._advance('DROP', text)
        elif m := re.match(r'Servo\s+(\d+)\s*->\s*(\d+)', text):
            ch, pwm = int(m.group(1)), int(m.group(2))
            if pwm >= 1200:
                what = SERVO_PAYLOADS.get(ch, f'servo {ch}')
                self.drops.append(what)
                events.append(Event('info', f'PAYLOAD RELEASED: {what} (servo {ch} -> {pwm} us); watch it fall in Gazebo'))
        elif text.startswith('Servo') and 'FAILED' in text:
            events.append(Event('warn', f'main_controller: {text}'))
        elif text.strip() == 'RTL':
            events += self._advance('RTL')
        elif text.startswith(('FAILED', 'WARN')):
            events.append(Event('warn', f'team: {text}'))
        return events

    def on_detection(self, image: str, classes_scores: List[Tuple[str, float]], lat: float, lon: float) -> List[Event]:
        """/image_detection: one line per detected person/tent."""
        events = []
        for cls, score in classes_scores:
            self.detections.setdefault(cls, []).append(score)
            where = f'{lat:.6f}, {lon:.6f}' if (lat or lon) else 'NO GPS (0, 0)'
            events.append(Event('detection', f'{cls.upper():6s} {score * 100:4.0f}% at {where}  (photo {image})'))
        return events

    def on_photo_count(self, count: int) -> List[Event]:
        """Photo-folder size: one line for the first photo, then every 10th."""
        if count <= self.photos:
            self.photos = count
            return []
        first = self.photos == 0
        self.photos = count
        if first or count % 10 == 0:
            return [Event('photo', f'photos saved by the team\'s camera node: {count}')]
        return []

    def summary(self) -> List[str]:
        """Lines printed at the end of the demo."""
        mins = (time.monotonic() - self.started) / 60.0
        det = ', '.join(f'{k}: {len(v)} (best {max(v) * 100:.0f}%)' for k, v in self.detections.items() if v) or 'none'
        return [f'flight time {mins:.1f} min, last phase {self.phase}',
                f'photos: {self.photos}', f'detections: {det}',
                f'targets: {self.targets_text or "none inserted"}',
                f'payloads released: {", ".join(self.drops) if self.drops else "none"}']


# ---------------------------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------------------------

def _count_photos(folder: str) -> int:
    """Number of finished photos in `folder` (temporary *.tmp.jpg files are not counted)."""
    try:
        return sum(1 for n in os.listdir(folder)
                   if n.lower().endswith(('.jpg', '.jpeg', '.png')) and '.tmp.' not in n)
    except OSError:
        return 0


def main(argv=None) -> int:
    """Command-line entry point: run the monitor node until the drone has landed (or Ctrl+C / timeout)."""
    parser = argparse.ArgumentParser(prog='phase_monitor', description=__doc__.split('\n\n')[0])
    parser.add_argument('--log', default='', help='also append every event as a JSON line to this file')
    parser.add_argument('--photo-dir', default='', help='team photo folder ("" = video_cam\'s own rule)')
    parser.add_argument('--timeout', type=float, default=0.0, help='exit with code 3 after this many seconds (0 = never)')
    parser.add_argument('--no-exit', action='store_true', help='keep running after landing')
    args, ros_args = parser.parse_known_args(argv if argv is not None else sys.argv[1:])

    import rclpy
    from mavros_msgs.msg import ExtendedState, State, StatusText
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Float64

    from sim_bringup.sim_paths import team_photo_dir

    try:
        from interfaces.msg import ImageResult       # the team's detection message
    except ImportError:
        ImageResult = None

    photo_dir = team_photo_dir(args.photo_dir)
    state = MissionState()
    log_file = open(args.log, 'a', encoding='utf-8') if args.log else None
    exit_code = {'value': None}

    def emit(events: List[Event]) -> None:
        for ev in events:
            stamp = time.strftime('%H:%M:%S')
            if ev.kind == 'phase':
                n = PHASE_INDEX[ev.phase]
                line = f'[{stamp}] ===== PHASE {n}/{len(PHASES) - 1}: {ev.phase} ===== {ev.text}'
            elif ev.kind == 'detection':
                line = f'[{stamp}]   DETECTION  {ev.text}'
            elif ev.kind == 'warn':
                line = f'[{stamp}]   (!) {ev.text}'
            else:
                line = f'[{stamp}]   {ev.text}'
            print(line, flush=True)
            if log_file:
                log_file.write(json.dumps({'time': time.time(), 'kind': ev.kind, 'phase': state.phase,
                                           'text': ev.text}) + '\n')
                log_file.flush()
            if ev.kind == 'phase' and ev.phase == 'LANDED' and not args.no_exit:
                exit_code['value'] = 0

    rclpy.init(args=ros_args)
    node = Node('phase_monitor')
    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(State, '/mavros/state', lambda m: emit(state.on_state(m.armed, m.mode)), reliable)
    node.create_subscription(ExtendedState, '/mavros/extended_state',
                             lambda m: emit(state.on_landed_state(m.landed_state)), reliable)
    node.create_subscription(Float64, '/mavros/global_position/rel_alt',
                             lambda m: emit(state.on_altitude(m.data)), qos_profile_sensor_data)
    node.create_subscription(StatusText, '/mavros/statustext/recv',
                             lambda m: emit(state.on_autopilot_text(m.text)), qos_profile_sensor_data)
    node.create_subscription(StatusText, '/mavros/statustext/send',
                             lambda m: emit(state.on_team_text(m.text)), reliable)
    if ImageResult is not None:
        names = {'0': 'person', '1': 'tent'}

        def on_result(msg):
            found = []
            for det in msg.detections.detections:
                for hyp in det.results:
                    cls = names.get(hyp.hypothesis.class_id)
                    if cls:
                        found.append((cls, float(hyp.hypothesis.score)))
            emit(state.on_detection(msg.image_name, found, float(msg.latitude), float(msg.longitude)))
        node.create_subscription(ImageResult, '/image_detection', on_result, reliable)
    else:
        print('[phase_monitor] interfaces (the team package) is not built: detections are not shown here', flush=True)

    node.create_timer(2.0, lambda: emit(state.on_photo_count(_count_photos(photo_dir))))
    last_status = {'t': time.monotonic()}

    def status_tick():
        if time.monotonic() - last_status['t'] >= 20.0 and state.phase not in ('WAITING', 'LANDED'):
            alt = f'{state.rel_alt:.0f} m' if state.rel_alt is not None else '? m'
            dets = sum(len(v) for v in state.detections.values())
            print(f'[{time.strftime("%H:%M:%S")}]   status: {state.phase} | mode {state.mode or "?"} | alt {alt} | '
                  f'photos {state.photos} | detections {dets}', flush=True)
            last_status['t'] = time.monotonic()
    node.create_timer(5.0, status_tick)

    # The team's main_controller (holds, picks targets, drops, RTL) can crash (a parameter read that timed out
    # is not handled): say so loudly once, otherwise the audience waits for drops that never come.
    controller = {'seen': False, 'warned': False}

    def controller_tick():
        names = {n for n, _ in node.get_node_names_and_namespaces()}
        if 'main_controller' in names:
            controller['seen'] = True
        elif controller['seen'] and not controller['warned']:
            controller['warned'] = True
            emit([Event('warn', 'main_controller is NOT RUNNING any more (team code crashed, see the team stack '
                                'log): no target waypoints or drops; the drone finishes the survey and returns home')])
    node.create_timer(5.0, controller_tick)

    print(f'[phase_monitor] watching the mission (photo folder {photo_dir})', flush=True)
    emit([Event('phase', dict(PHASES)['WAITING'], phase='WAITING')])
    start = time.monotonic()
    try:
        while rclpy.ok() and exit_code['value'] is None:
            rclpy.spin_once(node, timeout_sec=0.5)
            if args.timeout and time.monotonic() - start > args.timeout:
                print(f'[phase_monitor] timeout after {args.timeout:.0f} s (phase {state.phase})', flush=True)
                exit_code['value'] = 3
    except KeyboardInterrupt:
        exit_code['value'] = 130
    print('[phase_monitor] ----- summary -----', flush=True)
    for line in state.summary():
        print(f'[phase_monitor]   {line}', flush=True)
    if log_file:
        log_file.write(json.dumps({'time': time.time(), 'kind': 'summary', 'lines': state.summary()}) + '\n')
        log_file.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return exit_code['value'] if exit_code['value'] is not None else 0


if __name__ == '__main__':
    sys.exit(main())
