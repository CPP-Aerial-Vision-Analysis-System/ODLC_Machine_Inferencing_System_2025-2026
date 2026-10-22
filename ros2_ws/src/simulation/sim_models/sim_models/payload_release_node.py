"""
sim_models/payload_release_node.py -- PURPOSE: the "release mechanism" of the simulated drone, as a
ROS 2 node. When ArduPilot SITL opens a payload servo (MAV_CMD_DO_SET_SERVO 11 or 13 to 1400 us,
exactly what last year's main_controller.py sends), this node sees the servo value in Gazebo and
tells the DetachableJoint that holds that payload to let go, so the payload visibly falls. It logs
the release, confirms the detachment and reports where the payload landed.

Started with:  ros2 run sim_models payload_release   (scripts/payload_release.py calls main())

Data flow. Gazebo topics are read and written directly with the gz-transport Python bindings
(gz.transport13 / gz.msgs10, part of gz-harmonic), so no ros_gz_bridge is needed:
    gz  /sim/payload/servo11            gz.msgs.Double   servo 11 pulse width [us] -> person release
    gz  /sim/payload/servo13            gz.msgs.Double   servo 13 pulse width [us] -> tent release
    gz  /sim/payload/person/detach      gz.msgs.Empty    published on release (DetachableJoint)
    gz  /sim/payload/tent/detach        gz.msgs.Empty
    gz  /sim/payload/<name>/state       gz.msgs.StringMsg "attached"/"detached" (confirmation)
    gz  /sim/payload/<name>/odometry    gz.msgs.Odometry world ENU position of the payload (landing)
    gz  /sim/drone/odometry             gz.msgs.Odometry drone position (logged at the release)
    ROS /sim/payload/events             std_msgs/String  one JSON line per event: "released",
                                        "detached", "landed", "attached" (a new payload is held
                                        again after Gazebo restarted; the node re-arms)
                                        (reliable + transient local, depth 20)

Why gz-transport and not a bridge? ArduPilotPlugin republishes every servo channel on every physics
step (1000 messages per second). gz-transport can throttle a subscription (SubscribeOptions
msgs_per_sec) before Python ever sees the messages; a bridge would push all of them through ROS.

The release DECISION is in sim_models.payload_logic (ServoReleaseDetector, LandingDetector), which
is unit tested without ROS or Gazebo. The Gazebo side is hidden behind GzLink, so tests can feed
fake values (see test/test_payload_release_node.py).
"""

import json
import math
import threading
import time

from sim_models.payload_logic import (
    OPEN_PWM_US,
    LandingDetector,
    ServoReleaseDetector,
    enu_to_geodetic,
)

# Default SITL servo output of each known payload (main_controller.py:24-25).
DEFAULT_SERVOS = {"person": 11, "tent": 13}


class GzLink:
    """Thin wrapper around gz.transport13 that hands plain Python values to callbacks.

    Every method returns True on success. Message conversion:
        Double    -> float (msg.data)
        Odometry  -> (t_sim, x, y, z)  t_sim from the header stamp [s], position [m]
        StringMsg -> str
    """

    def __init__(self):
        """Create the gz-transport node. Raises ImportError if the bindings are missing."""
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node, SubscribeOptions

        self._types = {
            "Double": Double,
            "Empty": Empty,
            "Odometry": Odometry,
            "StringMsg": StringMsg,
        }
        self._options_cls = SubscribeOptions
        self._node = Node()
        self._publishers = []  # keep references alive

    def _options(self, rate_hz):
        opts = self._options_cls()
        if rate_hz and rate_hz > 0:
            opts.msgs_per_sec = int(math.ceil(rate_hz))
        return opts

    def subscribe_double(self, topic, callback, rate_hz=0.0):
        """callback(value: float) for each gz.msgs.Double (at most rate_hz per second)."""
        return bool(
            self._node.subscribe(
                self._types["Double"],
                topic,
                lambda m: callback(float(m.data)),
                self._options(rate_hz),
            )
        )

    def subscribe_odometry(self, topic, callback, rate_hz=0.0):
        """callback((t_sim, x, y, z)) for each gz.msgs.Odometry (at most rate_hz per second)."""

        def convert(m):
            t = (
                m.header.stamp.sec + m.header.stamp.nsec * 1e-9
                if m.HasField("header")
                else math.nan
            )
            p = m.pose.position
            callback((t, p.x, p.y, p.z))

        return bool(
            self._node.subscribe(self._types["Odometry"], topic, convert, self._options(rate_hz))
        )

    def subscribe_string(self, topic, callback):
        """callback(text: str) for each gz.msgs.StringMsg."""
        return bool(
            self._node.subscribe(
                self._types["StringMsg"], topic, lambda m: callback(str(m.data)), self._options(0)
            )
        )

    def advertise_empty(self, topic):
        """Return a function that publishes one gz.msgs.Empty on `topic` (None on failure)."""
        pub = self._node.advertise(topic, self._types["Empty"])
        if pub is None or not pub.valid():
            return None
        self._publishers.append(pub)
        empty_type = self._types["Empty"]
        return lambda: pub.publish(empty_type())


class PayloadChannel:
    """Everything the node tracks for ONE payload (thread-safe inbox for the gz callbacks)."""

    def __init__(self, name, servo, topics, detector, landing):
        """
        Args:
            name: payload name ('person', 'tent', ...).
            servo: ArduPilot SITL servo output number (11, 13, ...).
            topics: dict with keys servo, detach, odometry, state (gz topic names).
            detector: a ServoReleaseDetector.  landing: a LandingDetector.
        """
        self.name = name
        self.servo = servo
        self.topics = topics
        self.detector = detector
        self.landing = landing
        self.publish_detach = None
        self.detach_sent = 0  # how many detach messages were published
        self.last_detach_wall = 0.0  # time.monotonic() of the last detach message
        self.state = None  # last text on the DetachableJoint state topic
        self.last_logged_pwm = None
        self.last_position = None  # (t, x, y, z) world, latest odometry
        self.warned = set()
        self._lock = threading.Lock()
        self._pwm_inbox = []
        self._odom_inbox = []
        self._state_inbox = []

    # ---- called from gz-transport threads ----
    def on_pwm(self, value):
        with self._lock:
            self._pwm_inbox.append((time.monotonic(), value))

    def on_odometry(self, sample):
        with self._lock:
            self._odom_inbox.append(sample)

    def on_state(self, text):
        with self._lock:
            self._state_inbox.append(text)

    # ---- called from the ROS timer ----
    def take_inbox(self):
        """Return and clear (pwm samples, odometry samples, state texts)."""
        with self._lock:
            pwm, odom, state = self._pwm_inbox, self._odom_inbox, self._state_inbox
            self._pwm_inbox, self._odom_inbox, self._state_inbox = [], [], []
        return pwm, odom, state


def _make_node_class():
    """Build the rclpy Node subclass lazily, so this module imports without ROS (for tests/tools)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class PayloadReleaseNode(Node):
        """ROS 2 node 'payload_release': drops a payload in Gazebo when its SITL servo opens.

        Parameters (name: default, unit):
            payloads: ['person', 'tent']            payload names to watch
            <name>.servo: 11 (person) / 13 (tent)    SITL servo output number
            <name>.servo_topic: /sim/payload/servo<servo>
            <name>.detach_topic: /sim/payload/<name>/detach
            <name>.odometry_topic: /sim/payload/<name>/odometry
            <name>.state_topic: /sim/payload/<name>/state
            open_threshold_us: 1400.0 [us]           servo value that means OPEN
            min_open_time_s: 0.0 [s]                 servo must stay open this long first
            servo_sample_rate_hz: 50.0 [Hz]          throttle of the 1 kHz servo streams
            drone_odometry_topic: /sim/drone/odometry
            events_topic: /sim/payload/events
            landing_rest_speed: 0.05 [m/s], landing_rest_time_s: 1.0 [s]
            world_origin_lat / _lon [deg], world_origin_alt [m]: NaN = unknown (then events carry
                ENU metres only; set them to the world's <spherical_coordinates> to add lat/lon)
            detach_repeats: 3                        detach messages sent per release (0.2 s apart,
                                                     until the joint reports "detached")
        """

        def __init__(self, gz_link=None):
            """
            Args:
                gz_link: object with the GzLink methods (tests pass a fake). None = real GzLink.
            """
            super().__init__("payload_release")
            self._declare_parameters()
            p = self.get_parameter
            self.open_threshold_us = float(p("open_threshold_us").value)
            self.min_open_time_s = float(p("min_open_time_s").value)
            self.sample_rate_hz = float(p("servo_sample_rate_hz").value)
            self.detach_repeats = max(1, int(p("detach_repeats").value))
            self.origin = (
                float(p("world_origin_lat").value),
                float(p("world_origin_lon").value),
                float(p("world_origin_alt").value),
            )
            qos = QoSProfile(
                depth=20,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.events_pub = self.create_publisher(String, p("events_topic").value, qos)
            self._string_cls = String
            self.drone_position = None  # (t, x, y, z) latest drone odometry
            self._drone_lock = threading.Lock()
            self._drone_warned = False
            self.channels = self._make_channels()
            self.gz = gz_link if gz_link is not None else GzLink()
            self._connect_gazebo()
            self.timer = self.create_timer(0.02, self._tick)  # 50 Hz
            for ch in self.channels:
                self.get_logger().info(
                    f'watching payload "{ch.name}": servo {ch.servo} on {ch.topics["servo"]}, '
                    f"opens at >= {self.open_threshold_us:.0f} us, detach on {ch.topics['detach']}. "
                    f"(Until ArduPilot first commands servo {ch.servo} it reads 1000 us = closed, "
                    f"or nothing before SITL connects: both are normal.)"
                )

        # ------------------------------------------------------------ setup
        def _declare_parameters(self):
            d = self.declare_parameter
            d("payloads", ["person", "tent"])
            d("open_threshold_us", OPEN_PWM_US)
            d("min_open_time_s", 0.0)
            d("servo_sample_rate_hz", 50.0)
            d("drone_odometry_topic", "/sim/drone/odometry")
            d("events_topic", "/sim/payload/events")
            d("landing_rest_speed", 0.05)
            d("landing_rest_time_s", 1.0)
            d("world_origin_lat", math.nan)
            d("world_origin_lon", math.nan)
            d("world_origin_alt", math.nan)
            d("detach_repeats", 3)

        def _make_channels(self):
            channels = []
            names = [str(n) for n in self.get_parameter("payloads").value if str(n)]
            for name in names:
                default_servo = DEFAULT_SERVOS.get(name, 0)
                servo = int(self.declare_parameter(f"{name}.servo", default_servo).value)
                if servo <= 0:
                    self.get_logger().error(
                        f'payload "{name}" has no servo number: set the parameter {name}.servo '
                        f"(SITL output 1..16). Skipping it."
                    )
                    continue
                topics = {
                    "servo": self.declare_parameter(
                        f"{name}.servo_topic", f"/sim/payload/servo{servo}"
                    ).value,
                    "detach": self.declare_parameter(
                        f"{name}.detach_topic", f"/sim/payload/{name}/detach"
                    ).value,
                    "odometry": self.declare_parameter(
                        f"{name}.odometry_topic", f"/sim/payload/{name}/odometry"
                    ).value,
                    "state": self.declare_parameter(
                        f"{name}.state_topic", f"/sim/payload/{name}/state"
                    ).value,
                }
                detector = ServoReleaseDetector(self.open_threshold_us, self.min_open_time_s)
                landing = LandingDetector(
                    float(self.get_parameter("landing_rest_speed").value),
                    float(self.get_parameter("landing_rest_time_s").value),
                )
                channels.append(PayloadChannel(name, servo, topics, detector, landing))
            if not channels:
                self.get_logger().warning("no payloads configured: nothing to do")
            return channels

        def _connect_gazebo(self):
            log = self.get_logger()
            drone_topic = self.get_parameter("drone_odometry_topic").value
            if not self.gz.subscribe_odometry(drone_topic, self._on_drone_odometry, 10.0):
                log.warning(
                    f"could not subscribe to {drone_topic} (drone position will be unknown)"
                )
            for ch in self.channels:
                ok = self.gz.subscribe_double(ch.topics["servo"], ch.on_pwm, self.sample_rate_hz)
                ok &= self.gz.subscribe_odometry(ch.topics["odometry"], ch.on_odometry, 20.0)
                ok &= self.gz.subscribe_string(ch.topics["state"], ch.on_state)
                ch.publish_detach = self.gz.advertise_empty(ch.topics["detach"])
                if not ok or ch.publish_detach is None:
                    log.error(
                        f'payload "{ch.name}": could not connect all Gazebo topics {ch.topics}'
                    )

        def _on_drone_odometry(self, sample):
            with self._drone_lock:
                self.drone_position = sample

        # ------------------------------------------------------------ main loop
        def _tick(self):
            for ch in self.channels:
                pwm_samples, odom_samples, states = ch.take_inbox()
                for sample in odom_samples:
                    ch.last_position = sample
                    if ch.landing.update(*sample):
                        self._on_landed(ch)
                for t_wall, pwm in pwm_samples:
                    self._log_pwm_change(ch, pwm)
                    if ch.detector.update(pwm, t_wall):
                        self._release(ch, pwm)
                for text in states:
                    self._on_state(ch, text)
                self._repeat_detach(ch)

        def _log_pwm_change(self, ch, pwm):
            rounded = int(round(pwm))
            if rounded != ch.last_logged_pwm:
                state = "OPEN" if ch.detector.is_open(pwm) else "closed"
                before = "-" if ch.last_logged_pwm is None else f"{ch.last_logged_pwm} us"
                self.get_logger().info(
                    f"servo {ch.servo} ({ch.name}): {before} -> {rounded} us ({state})"
                )
                ch.last_logged_pwm = rounded

        def _release(self, ch, pwm):
            with self._drone_lock:
                drone = self.drone_position
            start = ch.last_position or drone
            if ch.last_position is None and "no_payload_odom" not in ch.warned:
                ch.warned.add("no_payload_odom")
                self.get_logger().warning(
                    f'payload "{ch.name}": no position on {ch.topics["odometry"]} yet, so the landing '
                    f"point cannot be measured (using the drone position as the release point)"
                )
            if drone is None and not self._drone_warned:
                self._drone_warned = True
                self.get_logger().warning(
                    f"no drone odometry on {self.get_parameter('drone_odometry_topic').value}"
                )
            if start is not None:
                ch.landing.start(*start)
            self._send_detach(ch)
            where = self._fmt_position(drone)
            self.get_logger().info(
                f'RELEASE payload "{ch.name}": servo {ch.servo} = {pwm:.0f} us '
                f"(>= {self.open_threshold_us:.0f} us). Drone at {where}. "
                f"Sent detach on {ch.topics['detach']}."
            )
            self._publish_event("released", ch, pwm_us=round(pwm, 1), drone=drone)

        def _send_detach(self, ch):
            if ch.publish_detach is None:
                if "no_detach_pub" not in ch.warned:
                    ch.warned.add("no_detach_pub")
                    self.get_logger().error(
                        f'payload "{ch.name}": no publisher for {ch.topics["detach"]}'
                    )
                return
            ch.publish_detach()
            ch.detach_sent += 1
            ch.last_detach_wall = time.monotonic()

        def _repeat_detach(self, ch):
            """Resend the detach (0.2 s apart, detach_repeats in total) until the joint confirms."""
            if not ch.detector.released or ch.state == "detached":
                return
            if (
                0 < ch.detach_sent < self.detach_repeats
                and time.monotonic() - ch.last_detach_wall >= 0.2
            ):
                self._send_detach(ch)

        def _on_state(self, ch, text):
            text = text.strip().lower()
            if text == ch.state:
                return
            ch.state = text
            if text == "detached":
                self.get_logger().info(f'payload "{ch.name}" is DETACHED: it is falling')
                self._publish_event("detached", ch)
            elif text == "attached" and ch.detector.released:
                # A new payload is held again (Gazebo was restarted or the drone re-spawned while
                # this node kept running): forget the old drop so the new payload can fall too.
                ch.detector = ServoReleaseDetector(self.open_threshold_us, self.min_open_time_s)
                ch.landing = LandingDetector(ch.landing.rest_speed, ch.landing.rest_time_s)
                ch.detach_sent = 0
                self.get_logger().info(
                    f'payload "{ch.name}" is ATTACHED again (new simulation?): ready to drop it'
                )
                self._publish_event("attached", ch)
            elif ch.detector.released:
                self.get_logger().warning(
                    f'payload "{ch.name}" state is "{text}" after the release'
                )

        def _on_landed(self, ch):
            t, x, y, z = ch.landing.landed_point
            summary = ch.landing.fall_summary()
            self.get_logger().info(
                f'payload "{ch.name}" LANDED at {self._fmt_position((t, x, y, z))} after falling '
                f"{summary['fall_height_m']:.1f} m in {summary['fall_time_s']:.1f} s "
                f"(drift {summary['drift_m']:.2f} m from the release point)"
            )
            self._publish_event("landed", ch, position=(t, x, y, z), **summary)

        # ------------------------------------------------------------ output helpers
        def _geo(self, x, y, z):
            if any(math.isnan(v) for v in self.origin):
                return None
            return enu_to_geodetic(x, y, z, *self.origin)

        def _fmt_position(self, sample):
            if sample is None:
                return "unknown position"
            _, x, y, z = sample
            text = f"E {x:.2f} N {y:.2f} U {z:.2f} m"
            geo = self._geo(x, y, z)
            if geo:
                text += f" (lat {geo[0]:.7f} lon {geo[1]:.7f})"
            return text

        def _publish_event(self, event, ch, position=None, drone=None, **extra):
            data = {
                "event": event,
                "payload": ch.name,
                "servo": ch.servo,
                "wall_time": round(time.time(), 3),
            }
            pos = position or ch.last_position
            if event == "released":
                pos = ch.landing.start_point or pos
            for key, sample in (("position", pos), ("drone", drone)):
                if sample is None:
                    continue
                t, x, y, z = sample
                data[f"{key}_enu"] = [round(x, 3), round(y, 3), round(z, 3)]
                if not math.isnan(t):
                    data[f"{key}_sim_time"] = round(t, 3)
                geo = self._geo(x, y, z)
                if geo:
                    data[f"{key}_lat"] = round(geo[0], 8)
                    data[f"{key}_lon"] = round(geo[1], 8)
                    data[f"{key}_alt"] = round(geo[2], 3)
            data.update(extra)
            msg = self._string_cls()
            msg.data = json.dumps(data)
            self.events_pub.publish(msg)

    return rclpy, PayloadReleaseNode


def main(args=None):
    """Entry point: run the payload_release node until Ctrl-C. Returns the exit code."""
    rclpy, PayloadReleaseNode = _make_node_class()
    rclpy.init(args=args)
    code = 0
    node = None
    try:
        node = PayloadReleaseNode()
        rclpy.spin(node)
    except ImportError as exc:
        print(
            "[payload_release] ERROR: the Gazebo Python bindings are missing "
            f"({exc}). Install them with: sudo apt install python3-gz-transport13 "
            "python3-gz-msgs10   (they come with gz-harmonic).",
            flush=True,
        )
        code = 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
