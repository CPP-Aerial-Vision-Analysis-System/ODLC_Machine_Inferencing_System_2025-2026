"""
lattice_bridge/sources/ros_source.py -- PURPOSE: the same events as the MAVLink source, but read from the ROS 2
topics that MAVROS publishes, for a bridge that runs ON the Jetson (Option B of docs/lattice/LATTICE_PLAN.md:
only useful if the drone has an IP link to the internet or to a local Lattice node, because in flight the
Jetson's only link to the ground is the MAVLink radio).

Topics it listens to (all by default; names can be changed in the bridge profile):

    /mavros/global_position/global       sensor_msgs/NavSatFix     the position (one DroneFix per message)
    /mavros/global_position/rel_alt      std_msgs/Float64          height above home
    /mavros/global_position/compass_hdg  std_msgs/Float64          compass heading, degrees
    /mavros/state                        mavros_msgs/State         armed flag and flight mode
    /mavros/battery                      sensor_msgs/BatteryState  charge as a fraction 0..1
    /mavros/statustext/recv              mavros_msgs/StatusText    texts FROM the autopilot
    /mavros/statustext/send              mavros_msgs/StatusText    texts the Jetson's own nodes SEND; this is where
                                                                    the `TGT` reports and the "Servo 13 -> 1400"
                                                                    release lines appear (they are sent to the
                                                                    autopilot, which never echoes them back on /recv)

FLIGHT SAFETY (the rules from docs/lattice/LATTICE_PLAN.md section 4): this is a separate process with its own
ROS node. Every callback only converts the message and puts the result into a bounded queue: no sleeping, no
network, no file access, no waiting for anything, and an exception inside a callback is caught and counted so it
can never stop the executor. Subscriptions use the "sensor" QoS (best effort), which is compatible with every
MAVROS and team publisher (a best-effort subscriber can listen to both best-effort and reliable publishers, the
other way round it could not) and, being a subscriber only, it cannot change anything for the publishers.

HEIGHT: MAVROS converts the autopilot's height above sea level to height above the WGS84 ELLIPSOID before it
publishes the NavSatFix (it uses a geoid model it loads at start-up). So this source delivers `alt_hae_m` and
the bridge must NOT add the geoid undulation again (docs/lattice/DATA_CONTRACT.md section 4). If the bench shows
that your MAVROS publishes sea-level height, set `altitude_is: amsl` and the correction is applied as usual.

rclpy and the message packages are imported only in start(); every handler below works with plain objects that
have the right attributes, so the whole conversion is unit-tested without ROS.
"""

from __future__ import annotations

import logging
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Tuple

from .. import constants as C
from ..events import Event, SourceNotice
from ..geo import is_real_number, require_lat_lon
from ..timeutil import from_unix_seconds
from . import SourceUnavailable
from .common import ClockJudge, DroneStateAccumulator, EventQueue, TextEventDecoder

LOG = logging.getLogger('lattice_bridge.ros')

#: MAVROS builds the mode text from the autopilot's custom mode ('AUTO', 'GUIDED', 'CMODE(11)' ...).
_MODE_TEXT = re.compile(r'[A-Za-z0-9_() .-]{1,24}')


@dataclass
class RosTopics:
    """The topic names (defaults are MAVROS' standard ones)."""
    fix: str = '/mavros/global_position/global'
    rel_alt: str = '/mavros/global_position/rel_alt'
    compass: str = '/mavros/global_position/compass_hdg'
    state: str = '/mavros/state'
    battery: str = '/mavros/battery'
    statustext_recv: str = '/mavros/statustext/recv'
    statustext_send: str = '/mavros/statustext/send'


def ros_stamp_to_utc(stamp) -> Optional[datetime]:
    """A ROS time (builtin_interfaces/Time with sec and nanosec) as aware UTC, or None for "no stamp" (0) or junk."""
    try:
        seconds, nanoseconds = int(stamp.sec), int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if seconds == 0 and nanoseconds == 0:
        return None
    try:
        return from_unix_seconds(seconds + nanoseconds / 1e9)
    except ValueError:
        return None


class RosSource:
    """A Source that listens to MAVROS topics (see the module docstring).

    `altitude_is` says what NavSatFix.altitude means: 'hae' (above the ellipsoid, MAVROS' documented behaviour)
    or 'amsl' (above sea level). `node_factory(name)` is injectable for tests; the default creates a real
    rclpy node in start().
    """

    name = 'ros'

    def __init__(self, *, topics: Optional[RosTopics] = None, node_name: str = 'lattice_bridge_source',
                 altitude_is: str = 'hae', time_mode: str = 'auto', max_clock_skew_s: float = 2.0,
                 tgt_severities: Tuple[int, ...] = (C.STATUSTEXT_SEVERITY_INFO,),
                 drop_channels: Optional[Dict[int, str]] = None, drop_open_pwm: int = 1400,
                 rel_alt_max_age_s: float = 5.0, queue_size: int = 1000,
                 ros_args: Optional[List[str]] = None, clock: Optional[Callable[[], datetime]] = None):
        if altitude_is not in ('hae', 'amsl'):
            raise ValueError("altitude_is must be 'hae' or 'amsl'")
        self.topics = topics or RosTopics()
        self.node_name = node_name
        self.altitude_is = altitude_is
        self.altitude_reference = altitude_is
        self.rel_alt_max_age_s = float(rel_alt_max_age_s)
        self.ros_args = list(ros_args) if ros_args else None
        self._clock = clock or (lambda: datetime.now(timezone.utc))
        self._judge = ClockJudge(time_mode, max_clock_skew_s)
        self.accumulator = DroneStateAccumulator()
        self.texts = TextEventDecoder(severities=tgt_severities, drop_channels=drop_channels,
                                      drop_open_pwm=drop_open_pwm)
        self.queue = EventQueue(queue_size)
        self._rel_alt: Optional[Tuple[float, datetime]] = None
        self._last_fix_time: Optional[datetime] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._node: Any = None
        self._executor: Any = None
        self._rclpy: Any = None
        self._owns_rclpy = False
        self._last_error_log = -1e9
        self.counters = {'fixes': 0, 'no_fix': 0, 'bad_fixes': 0, 'fixes_not_newer': 0, 'texts': 0,
                         'callback_errors': 0}

    # ------------------------------------------------------------------------------------------ handlers
    # Each one takes a message object (duck-typed) and the arrival time; none of them blocks or does I/O.

    def handle_fix(self, msg, arrival: Optional[datetime] = None) -> None:
        """NavSatFix -> DroneFix. A message without a GPS fix, or with a bad position, is counted and dropped."""
        arrival = arrival or self._clock()
        if int(msg.status.status) < 0:                      # NavSatStatus.STATUS_NO_FIX = -1
            self.counters['no_fix'] += 1
            return
        try:
            lat, lon = require_lat_lon(float(msg.latitude), float(msg.longitude), 'drone position')
        except ValueError:
            self.counters['bad_fixes'] += 1
            return
        fix_time, used_stamp = self._judge.choose(ros_stamp_to_utc(msg.header.stamp), arrival)
        if self._last_fix_time is not None and fix_time <= self._last_fix_time:
            self.counters['fixes_not_newer'] += 1
            return
        altitude = float(msg.altitude)
        altitude = altitude if is_real_number(altitude) and C.ALT_MIN_M <= altitude <= C.ALT_MAX_M else None
        rel_alt = None
        if self._rel_alt is not None and 0.0 <= (arrival - self._rel_alt[1]).total_seconds() <= self.rel_alt_max_age_s:
            rel_alt = self._rel_alt[0]
        fix = self.accumulator.build_fix(
            fix_time=fix_time, arrival=arrival, lat_deg=lat, lon_deg=lon,
            alt_hae_m=altitude if self.altitude_is == 'hae' else None,
            alt_amsl_m=altitude if self.altitude_is == 'amsl' else None,
            rel_alt_m=rel_alt, time_source='ros_stamp' if used_stamp else 'arrival')
        self._last_fix_time = fix_time
        self.counters['fixes'] += 1
        self.queue.put(fix)

    def handle_rel_alt(self, msg, arrival: Optional[datetime] = None) -> None:
        """Float64 height above home (metres)."""
        value = float(msg.data)
        if is_real_number(value) and C.ALT_MIN_M <= value <= C.ALT_MAX_M:
            self._rel_alt = (value, arrival or self._clock())

    def handle_compass(self, msg, arrival: Optional[datetime] = None) -> None:
        """Float64 compass heading in degrees (0 = north, clockwise)."""
        value = float(msg.data)
        if is_real_number(value) and 0.0 <= value <= 360.0:
            self.accumulator.set_heading(value % 360.0, arrival or self._clock())

    def handle_state(self, msg, arrival: Optional[datetime] = None) -> None:
        """mavros_msgs/State -> flight mode and armed flag (both unknown while MAVROS is not connected)."""
        arrival = arrival or self._clock()
        if not bool(msg.connected):
            self.accumulator.set_mode(None, None, arrival)
            return
        mode = str(msg.mode).strip() if msg.mode is not None else ''
        self.accumulator.set_mode(mode if _MODE_TEXT.fullmatch(mode) else None, bool(msg.armed), arrival)

    def handle_battery(self, msg, arrival: Optional[datetime] = None) -> None:
        """BatteryState.percentage is a fraction 0..1 (NaN when unknown); stored as percent."""
        value = float(msg.percentage)
        if is_real_number(value) and 0.0 <= value <= 1.0:
            self.accumulator.set_battery(value * 100.0, arrival or self._clock())

    def handle_statustext(self, msg, arrival: Optional[datetime] = None) -> None:
        """mavros_msgs/StatusText -> `TGT` reports and release lines (everything else is ignored)."""
        arrival = arrival or self._clock()
        self.counters['texts'] += 1
        for event in self.texts.decode(str(msg.text), int(msg.severity), arrival, self.accumulator.last_fix):
            self.queue.put(event)

    # ------------------------------------------------------------------------------------------ Source API
    def start(self) -> None:
        """Import rclpy and the message types, create the node and its subscriptions, start the spin thread."""
        try:
            import rclpy
            from mavros_msgs.msg import State, StatusText
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import BatteryState, NavSatFix
            from std_msgs.msg import Float64
        except ImportError as exc:
            raise SourceUnavailable(
                f'the ROS source needs ROS 2 Humble with rclpy and mavros_msgs ({exc}). Run '
                '"source /opt/ros/humble/setup.bash" first, or use --source mavlink / --source replay.') from None
        self._rclpy = rclpy
        if not rclpy.ok():
            self._init_rclpy(rclpy)
        self._node = rclpy.create_node(self.node_name)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        wiring = [
            (NavSatFix, self.topics.fix, self.handle_fix), (Float64, self.topics.rel_alt, self.handle_rel_alt),
            (Float64, self.topics.compass, self.handle_compass), (State, self.topics.state, self.handle_state),
            (BatteryState, self.topics.battery, self.handle_battery),
            (StatusText, self.topics.statustext_recv, self.handle_statustext),
            (StatusText, self.topics.statustext_send, self.handle_statustext),
        ]
        for message_type, topic, handler in wiring:
            self._node.create_subscription(message_type, topic, self._guarded(handler), qos)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, name='lattice-bridge-ros', daemon=True)
        self._thread.start()
        self.queue.put(SourceNotice('connected', f'subscribed to {self.topics.fix} and 6 more ROS topics',
                                    self._clock()))

    def poll(self, timeout_s: float) -> List[Event]:
        """The events the callbacks queued, waiting up to `timeout_s` for the first one."""
        return self.queue.get_many(500, timeout_s)

    def now(self) -> datetime:
        """The current time (aware UTC)."""
        return self._clock()

    def describe(self) -> str:
        """One line for the log."""
        return (f'ROS 2 topics {self.topics.fix}, {self.topics.state}, {self.topics.statustext_send} ... '
                f'(heights are {"ellipsoid" if self.altitude_is == "hae" else "sea-level"} heights)')

    def stats(self) -> Dict[str, int]:
        """Counters of the source and the text decoder."""
        merged = dict(self.counters)
        merged.update(self.texts.stats)
        merged.update({'queue_dropped_fixes': self.queue.dropped_fixes, 'queue_dropped_other': self.queue.dropped_other})
        return merged

    def close(self) -> None:
        """Stop spinning and tear the node down. Safe to call twice."""
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=3.0)
        for action in (lambda: self._executor.shutdown(), lambda: self._node.destroy_node()):
            try:
                action()
            except Exception:                               # tearing down a half-built node may fail; keep going
                pass
        self._executor = self._node = None
        if self._owns_rclpy and self._rclpy is not None:
            try:
                self._rclpy.shutdown()
            except Exception:
                pass
            self._owns_rclpy = False

    # ------------------------------------------------------------------------------------------ internals
    def _init_rclpy(self, rclpy) -> None:
        """rclpy.init WITHOUT its own signal handlers: the bridge handles SIGINT/SIGTERM itself, in order."""
        try:
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(args=self.ros_args, signal_handler_options=SignalHandlerOptions.NO)
        except (ImportError, TypeError):
            rclpy.init(args=self.ros_args)
        self._owns_rclpy = True

    def _guarded(self, handler: Callable[[Any], None]) -> Callable[[Any], None]:
        """Wrap a handler so that an exception in it is counted and logged, never allowed to stop the executor."""
        def callback(msg):
            """The ROS subscription callback: run `handler`, counting and logging any error it raises."""
            try:
                handler(msg)
            except Exception as exc:
                self.counters['callback_errors'] += 1
                now = time.monotonic()
                if now - self._last_error_log >= 30.0:
                    LOG.error('ROS callback %s failed (%s: %s); the message was dropped', handler.__name__,
                              exc.__class__.__name__, exc)
                    self._last_error_log = now
        return callback

    def _spin(self) -> None:
        while not self._stop.is_set():
            try:
                self._executor.spin_once(timeout_sec=0.2)
            except Exception as exc:
                if self._stop.is_set():
                    return
                LOG.error('the ROS executor failed (%s: %s); retrying', exc.__class__.__name__, exc)
                self._stop.wait(0.5)
