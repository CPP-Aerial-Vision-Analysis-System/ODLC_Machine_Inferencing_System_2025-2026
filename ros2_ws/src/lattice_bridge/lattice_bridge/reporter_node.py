"""
lattice_bridge/reporter_node.py -- PURPOSE: the ROS 2 node `lattice_reporter` that runs ONBOARD the drone (on the
Jetson) next to the team's flight nodes and tells the ground about every hiker and tent the detector finds, as
short radio texts (`TGT p 36.214935 -96.006203 95 12`, see report_codec.py) sent through MAVROS as MAVLink
STATUSTEXT. On the ground the lattice_bridge program decodes them and puts the targets on the Lattice map.

THIS NODE IS OFF UNLESS YOU START IT. launcher.sh and tracker.launch.xml know nothing about it. Start it by hand
(`ros2 run lattice_bridge lattice_reporter`, or `ros2 launch lattice_bridge lattice_reporter.launch.py`) or with
the simulator profile simulation/profiles/lattice_reporter.yaml. All it does with the flight is listen.

How it works, in one picture:

    /image_detection  (RELIABLE, depth 10, from new_od)
          |  message callback: copy a few numbers into a plain object, put it in a small queue. Nothing else.
          v
    queue of photo results (at most `queue_size`; the oldest are dropped and counted)
          |  timer, every `tick_period_s`: for each detection, read the photo's sidecar and project the box
          |  centre onto the ground with the team's OWN code (target_locator.py), then ask TargetReporter
          v  (reporter_logic.py) whether it is new / clearly better / moved
    TargetReporter ---- at most one message per `min_interval_s`, K per target, N in all --->
          v
    /mavros/statustext/send  (mavros_msgs/StatusText, severity 6 = INFO)  ->  MAVROS  ->  telemetry radio

Why this cannot hurt the flight:
  * It is a separate process. No flight node imports it, and it needs nothing from the flight nodes except the
    messages they already publish. It only ADDS messages to /mavros/statustext/send (like the team nodes do).
  * No callback waits: the message callback only copies numbers; files are read from the timer, at most
    PROCESS_BUDGET_S per tick; the queue, the target list and every counter have fixed upper limits.
  * Every failure is caught and logged once (counted afterwards), never raised. A bad photo name, a missing or
    broken sidecar, a NaN position or score, a full queue: the detection is skipped and the node goes on.
  * Its only calls to a flight node are one read-only "get parameters" request to main_controller every
    `follow_period_s` seconds (see follow_main_controller), made without waiting.
  * `dry_run: true` does everything except publish.

Parameters: see PARAMETERS in reporter_logic.py, config/lattice_reporter.yaml (every one explained) and
docs/lattice/REPORTER.md.
"""

from __future__ import annotations

import sys
import time
from collections import deque
from typing import Dict, NamedTuple

import rclpy
from interfaces.msg import ImageResult
from mavros_msgs.msg import StatusText
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from . import constants as C
from .geo import is_real_number
from .reporter_logic import (
    AIM_NAMES, PARAMETERS, AimSettings, OnceGate, ReporterSettings, TargetReporter, clean_parameters,
)
from .target_locator import (
    KIND_BY_CLASS_ID, PhotoJob, TargetLocator, TeamModuleError, default_photo_dir, extract_job,
    find_team_module, load_team_module,
)

NODE_NAME = 'lattice_reporter'
DETECTION_TOPIC = '/image_detection'
STATUSTEXT_TOPIC = '/mavros/statustext/send'
#: Never spend longer than this in one timer tick on reading photo files (seconds).
PROCESS_BUDGET_S = 0.05
#: With follow_main_controller on, detections wait at most this long for main_controller's answer (seconds).
FOLLOW_WAIT_S = 10.0
#: Skips that are normal and not worth a log line (they are still counted).
SILENT_SKIPS = frozenset({'low_score'})
_MAX_SKIP_KEYS = 32


class _Queued(NamedTuple):
    """A photo result waiting to be processed, with the time it arrived (steady clock, seconds)."""
    received_s: float
    job: PhotoJob


class ReporterNode(Node):
    """Listens to the detector and sends TGT radio messages. See the module text.

    `context` and `parameter_overrides` exist for the tests (a private ROS domain, parameters without a file);
    `team_module_path` lets a test point at another copy of the team's target_geolocation.py.
    """

    def __init__(self, context=None, parameter_overrides=None, team_module_path=None):
        # The team's projection code first: without it there is nothing to do, and failing here (before any
        # ROS object exists) keeps the error message simple.
        how = 'given by the caller'
        if team_module_path is None:
            team_module_path, how = find_team_module()
            if team_module_path is None:
                raise TeamModuleError(how)
        self._geo = load_team_module(team_module_path)
        super().__init__(NODE_NAME, context=context, parameter_overrides=parameter_overrides)
        log = self.get_logger()
        self._started_s = time.monotonic()
        self._once = OnceGate()

        raw = {}
        for spec in PARAMETERS:
            self.declare_parameter(spec.name, spec.default,
                                   ParameterDescriptor(description=spec.text, dynamic_typing=True))
            raw[spec.name] = self.get_parameter(spec.name).value
        values, problems = clean_parameters(raw)
        for problem in problems:
            log.warn(problem)
        self.values = values
        self._dry_run = bool(values['dry_run'])
        self._summary_period_s = float(values['summary_period_s'])
        self._last_summary_s = self._started_s

        aim, _ = AimSettings.from_values(values)
        photo_dir = values['photo_dir'] or default_photo_dir(team_module_path)
        self._locator = TargetLocator(self._geo, photo_dir, aim, values['allow_photo_position_fallback'])
        self._reporter = TargetReporter(ReporterSettings.from_values(values))
        self._jobs: deque = deque(maxlen=int(values['queue_size']))
        self.counts: Dict[str, int] = {'photos': 0, 'photos_processed': 0, 'photos_dropped': 0, 'published': 0,
                                       'publish_errors': 0, 'errors': 0}
        self.skips: Dict[str, int] = {}

        self._pub = self.create_publisher(StatusText, STATUSTEXT_TOPIC, 10)
        # RELIABLE depth 10, exactly like main_controller (a BEST_EFFORT listener would also work, but the
        # task asks for the same QoS as the real consumer).
        self.create_subscription(ImageResult, DETECTION_TOPIC, self._on_detection,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self._timer = self.create_timer(float(values['tick_period_s']), self._tick)

        self._follow = bool(values['follow_main_controller'])
        self._aim_synced = False
        self._follower = None
        if self._follow:
            self._start_following(float(values['follow_period_s']))

        log.info(
            f'lattice_reporter started ({"DRY RUN: nothing will be published" if self._dry_run else "publishing"}'
            f' on {STATUSTEXT_TOPIC}); team code: {how}; photos: {photo_dir}; aim: {aim.describe()}; '
            f'at most one message per {self._reporter.settings.min_interval_s:g} s, '
            f'{self._reporter.settings.max_messages_per_target} per target')

    # ---------------------------------------------------------------------------------- reading main_controller
    def _start_following(self, period_s: float) -> None:
        """Ask main_controller for its aim settings (without waiting), now and every `period_s` seconds."""
        try:
            from wp_sender.parameter import AsyncParameterReader
        except ImportError as exc:
            self.get_logger().warn(f'follow_main_controller is on, but wp_sender cannot be imported ({exc}): '
                                   f'using the aim settings of this node')
            self._follow = False
            return
        self._follower = AsyncParameterReader(self, 'main_controller', list(AIM_NAMES),
                                              self._on_main_controller_values)
        self._follower.request('start-up')
        if period_s > 0:
            self.create_timer(period_s, lambda: self._follower.request('periodic refresh'))

    def _on_main_controller_values(self, values) -> None:
        """main_controller answered: use its aim settings from now on (runs on this node's executor)."""
        try:
            aim, problems = AimSettings.from_values(values)
            for problem in problems:
                if self._once.first(('follow', problem)):
                    self.get_logger().warn(f"main_controller's {problem}")
            if aim != self._locator.aim:
                self._locator.set_aim(aim)
                self.get_logger().info(f"using main_controller's aim settings: {aim.describe()}")
            elif not self._aim_synced:
                self.get_logger().info(f"main_controller's aim settings equal ours: {aim.describe()}")
            self._aim_synced = True
        except Exception as exc:  # noqa: BLE001 - never let a surprise reach the executor
            self._error_once('follow', exc)

    def _ready(self, now_s: float) -> bool:
        """May queued photos be processed? Not until main_controller answered (at most FOLLOW_WAIT_S)."""
        if not self._follow or self._aim_synced:
            return True
        if now_s - self._started_s >= FOLLOW_WAIT_S:
            if self._once.first('follow-timeout'):
                self.get_logger().warn(f'main_controller did not answer within {FOLLOW_WAIT_S:g} s: using the '
                                       f'aim settings of this node: {self._locator.aim.describe()}')
            return True
        return False

    # ---------------------------------------------------------------------------------- the message callback
    def _on_detection(self, msg) -> None:
        """Copy the numbers out of the message and queue them. Nothing else: no files, no maths, no waiting."""
        try:
            job, malformed = extract_job(msg)
            self.counts['photos'] += 1
            if malformed:
                self._skip('bad_message', f'{malformed} detection(s) of {job.image_name[:40]!r} could not be read')
            if len(self._jobs) == self._jobs.maxlen:
                self.counts['photos_dropped'] += 1
                if self._once.first('queue-full'):
                    self.get_logger().warn(f'more than {self._jobs.maxlen} detector answers are waiting: the '
                                           f'oldest are dropped (counted) from now on')
            self._jobs.append(_Queued(time.monotonic(), job))
        except Exception as exc:  # noqa: BLE001
            self._error_once('callback', exc)

    # ---------------------------------------------------------------------------------- the timer
    def _tick(self) -> None:
        """Process waiting photo results (within a time budget), then send at most one radio message."""
        now = time.monotonic()
        try:
            if self._jobs and self._ready(now):
                self._process_jobs()
            self._send_next(now)
            if self._summary_period_s > 0 and now - self._last_summary_s >= self._summary_period_s:
                self._last_summary_s = now
                self.log_summary()
        except Exception as exc:  # noqa: BLE001
            self._error_once('tick', exc)

    def _process_jobs(self) -> None:
        """Work through queued photo results until the queue is empty or PROCESS_BUDGET_S is used up."""
        deadline = time.monotonic() + PROCESS_BUDGET_S
        while self._jobs and time.monotonic() < deadline:
            queued = self._jobs.popleft()
            self.counts['photos_processed'] += 1
            try:
                self._handle(queued)
            except Exception as exc:  # noqa: BLE001 - one bad photo must not stop the others
                self._skip('job_error', f'processing {queued.job.image_name[:40]!r} failed: '
                                        f'{type(exc).__name__}: {exc}')

    def _handle(self, queued: _Queued) -> None:
        """Turn the detections of one photo into sightings (the same filters as main_controller)."""
        job = queued.job
        reporter = self._reporter
        photo = None
        for detection in job.detections:
            kind = KIND_BY_CLASS_ID.get(detection.class_id)
            if kind is None:
                continue                               # not a mission target class: ignored, like main_controller
            if not is_real_number(detection.score):
                self._skip('bad_score', f'a detection in {job.image_name[:40]!r} has a score that is not a '
                                        f'number ({detection.score})')
                continue
            if not reporter.accepts_score(detection.score):
                if detection.score > 1.0:
                    self._skip('score_over_1', f'a detection in {job.image_name[:40]!r} has a score above 1 '
                                               f'({detection.score}); the radio message cannot carry it')
                else:
                    self._skip('low_score', 'score too low')
                continue
            if photo is None:
                photo = self._locator.open_photo(job)
            located = self._locator.locate(photo, detection.u, detection.v)
            if not located.ok:
                self._skip(located.skip_key, f'{located.skip_text} (photo {job.image_name[:40]!r})')
                continue
            offer = reporter.offer(kind, located.lat, located.lon, detection.score, queued.received_s)
            if offer.action in ('new', 'improved'):
                self.get_logger().info(
                    f'{kind} {offer.target}: {offer.action} ({offer.reason}); score {detection.score:.2f} at '
                    f'{located.lat:.6f}, {located.lon:.6f} in {job.image_name[:40]!r}: {located.note}')
            elif offer.action == 'ignored':
                self._skip(offer.key or 'ignored', f'{kind} sighting ignored: {offer.reason}')
            elif offer.action == 'capped':
                self._skip('capped', f'{kind} {offer.target}: {offer.reason}')

    def _send_next(self, now_s: float) -> None:
        """Publish the next TGT message if one is due (rate limit) and somebody (MAVROS) is listening."""
        reporter = self._reporter
        if reporter.waiting() == 0:
            return
        if not self._dry_run and self._pub.get_subscription_count() == 0:
            if self._once.first('no-listener'):
                self.get_logger().warn(f'nobody listens on {STATUSTEXT_TOPIC} (is MAVROS running?): the radio '
                                       f'messages wait until somebody does')
            return
        out = reporter.next_message(now_s)
        if out is None:
            return
        if self._dry_run:
            self.get_logger().info(f'[dry run] would send "{out.text}" ({out.why}: {out.kind} {out.number}, '
                                   f'message {out.messages_for_target} for it)')
            return
        message = StatusText()
        message.severity = C.STATUSTEXT_SEVERITY_INFO
        message.text = out.text
        try:
            self._pub.publish(message)
        except Exception as exc:  # noqa: BLE001
            self.counts['publish_errors'] += 1
            self._error_once('publish', exc)
            return
        self.counts['published'] += 1
        self.get_logger().info(f'radio: {out.text}  ({out.why}: {out.kind} {out.number}, '
                               f'message {out.messages_for_target} for it)')

    # ---------------------------------------------------------------------------------- bookkeeping
    @property
    def reporter(self) -> TargetReporter:
        """The decision object (for tests and tools)."""
        return self._reporter

    @property
    def locator(self) -> TargetLocator:
        """The position finder (for tests and tools)."""
        return self._locator

    def _skip(self, key: str, text: str) -> None:
        """Count a skipped detection under `key`; log the first one of each kind (unless the kind is silent)."""
        if key in self.skips or len(self.skips) < _MAX_SKIP_KEYS:
            self.skips[key] = self.skips.get(key, 0) + 1
        if key not in SILENT_SKIPS and self._once.first(('skip', key)):
            self.get_logger().warn(f'skipping a detection ({key}): {text}. Further cases of this kind are only '
                                   f'counted; the totals are in the status line.')

    def _error_once(self, where: str, exc: BaseException) -> None:
        """Count an unexpected error and log the first one per place."""
        self.counts['errors'] += 1
        if self._once.first(('error', where)):
            self.get_logger().error(f'unexpected error in the {where} step, ignored: {type(exc).__name__}: {exc}')

    def status_line(self) -> str:
        """One line: targets, messages, photos, skips. For the log and the tests."""
        c = self.counts
        skipped = ', '.join(f'{key} {value}' for key, value in sorted(self.skips.items())) or 'none'
        return (f'{self._reporter.summary_line()} | photos: {c["photos"]} received, {c["photos_processed"]} '
                f'processed, {c["photos_dropped"]} dropped | published: {c["published"]} | detections skipped: '
                f'{skipped} | errors: {c["errors"]}')

    def log_summary(self, final: bool = False) -> None:
        """Write the status line to the log."""
        self.get_logger().info(('final ' if final else '') + 'status: ' + self.status_line())


def main(args=None) -> None:
    """Entry point: `ros2 run lattice_bridge lattice_reporter`. Exit code 1 when the team code cannot be found."""
    try:
        from rclpy.executors import ExternalShutdownException
    except ImportError:                                            # older rclpy
        ExternalShutdownException = KeyboardInterrupt              # noqa: N806
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ReporterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):         # Ctrl+C, or the launch system stopping us
        pass
    except TeamModuleError as exc:
        print(f'lattice_reporter: cannot start: {exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.log_summary(final=True)
            except Exception:  # noqa: BLE001 - the context may already be gone
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
