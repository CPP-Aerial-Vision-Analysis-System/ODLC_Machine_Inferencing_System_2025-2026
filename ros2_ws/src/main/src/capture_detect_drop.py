#!/usr/bin/env python3
"""Capture -> detect -> drop bench test (no mission, no waypoints).

What it does, in order:

  1. Triggers the SIYI camera at 1 Hz by publishing /camera/trigger, exactly
     the way mapping/do_digi_cam_trigger.py does during a survey. new_od picks
     each new file out of mapping_photos on its own (inotify) and publishes an
     interfaces/ImageResult on /image_detection -- so detection here runs
     through the identical path main_controller sees in flight.
  2. Keeps the single highest-confidence person and the single highest-
     confidence tent, logging when and where each new best was seen.
  3. Stops capturing, waits for the images it triggered to finish processing,
     then waits post_capture_wait seconds.
  4. Announces and performs the payload release for the first target, sends
     "done", then announces and performs the release for the second target.

Deliberately NOT here: waypoints, mode changes, RTL, any mission logic. The
vehicle is never commanded to move -- the only thing sent to the FCU is
MAV_CMD_DO_SET_SERVO (and statustext, for the ground station log).

Run:
    source /opt/ros/humble/setup.bash
    source ~/astra/ros2_ws/install/setup.bash
    ros2 run main capture_detect_drop.py

    # or without rebuilding:
    python3 ~/astra/ros2_ws/src/main/src/capture_detect_drop.py

    # 60 s capture window, stop as soon as both targets are seen:
    ros2 run main capture_detect_drop.py --ros-args \
        -p capture_seconds:=60.0 -p stop_when_both_found:=true

    # dry run -- full sequence, no servo commands actually sent:
    ros2 run main capture_detect_drop.py --ros-args -p dry_run:=true

    # only drive the channels that actually move on this airframe:
    ros2 run main capture_detect_drop.py --ros-args \
        -p person_servos:="[11]" -p tent_servos:="[13]"
"""

import json
import os
import threading
import time
from datetime import datetime

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from interfaces.msg import ImageResult
from mavros_msgs.msg import StatusText
from mavros_msgs.srv import CommandLong
from std_msgs.msg import Bool, String

# Servo channels per target -- same wiring as main_controller.py.
# Each target releases via TWO servos.
PERSON_SERVO_CHANNELS = [9, 11]
TENT_SERVO_CHANNELS = [13, 14]

# Pulley PWM positions (same for every release servo). The servos travel
# 500-1400us on this airframe, so OPEN is the high end and CLOSE the low end --
# the same high=open / low=closed sense the old 1900/1400 pair had.
# If the payload turns out to release at 500 instead, swap them at runtime:
#   -p open_pwm:=500 -p close_pwm:=1400
PULLEY_OPEN = 1400       # released / open
PULLEY_CLOSE = 500       # closed

# new_od publishes these class ids on ImageResult.detections; anything else
# ("object") is informational only and is not a mission target.
CLASS_BY_ID = {"0": "person", "1": "tent"}

MAV_CMD_DO_SET_SERVO = 183


class Target:
    """Best-so-far detection of one class."""

    def __init__(self, name):
        self.name = name
        self.confidence = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.image_name = ""
        self.wall_time = ""        # when we logged it
        self.image_timestamp = ""  # new_od's own timestamp for that image
        self.hits = 0              # how many times this class was seen at all

    @property
    def found(self):
        return self.confidence > 0.0

    def as_dict(self):
        return {
            "type": self.name,
            "found": self.found,
            "confidence": round(self.confidence, 4),
            "latitude": self.lat,
            "longitude": self.lon,
            "image_name": self.image_name,
            "detected_at": self.wall_time,
            "image_timestamp": self.image_timestamp,
            "total_sightings": self.hits,
        }


class CaptureDetectDrop(Node):

    def __init__(self):
        super().__init__("capture_detect_drop")

        # -- Parameters -------------------------------------------------
        self.declare_parameter("capture_seconds", 30.0)
        self.declare_parameter("capture_period", 1.0)
        self.declare_parameter("stop_when_both_found", False)
        self.declare_parameter("post_capture_wait", 10.0)
        self.declare_parameter("processing_timeout", 90.0)
        self.declare_parameter("servo_open_seconds", 3.0)
        self.declare_parameter("close_after_release", True)
        self.declare_parameter("inter_drop_wait", 0.0)
        self.declare_parameter("drop_order", ["tent", "person"])
        # Release channels, defaulting to main_controller.py's flight wiring.
        # Override to test only the channels that actually move, e.g.
        #   -p person_servos:="[11]" -p tent_servos:="[13]"
        self.declare_parameter("person_servos", PERSON_SERVO_CHANNELS)
        self.declare_parameter("tent_servos", TENT_SERVO_CHANNELS)
        self.declare_parameter("open_pwm", PULLEY_OPEN)
        self.declare_parameter("close_pwm", PULLEY_CLOSE)
        self.declare_parameter("drop_if_not_detected", True)
        self.declare_parameter("autofocus", True)
        self.declare_parameter("autofocus_period", 5.0)
        self.declare_parameter("camera_feed_path", "")
        self.declare_parameter("log_path", "")
        self.declare_parameter("dry_run", False)

        self.capture_seconds = float(self.get_parameter("capture_seconds").value)
        self.capture_period = float(self.get_parameter("capture_period").value)
        self.stop_when_both_found = bool(self.get_parameter("stop_when_both_found").value)
        self.post_capture_wait = float(self.get_parameter("post_capture_wait").value)
        self.processing_timeout = float(self.get_parameter("processing_timeout").value)
        self.servo_open_seconds = float(self.get_parameter("servo_open_seconds").value)
        self.close_after_release = bool(self.get_parameter("close_after_release").value)
        self.inter_drop_wait = float(self.get_parameter("inter_drop_wait").value)
        self.drop_order = [str(s).lower() for s in self.get_parameter("drop_order").value]
        self.drop_if_not_detected = bool(self.get_parameter("drop_if_not_detected").value)
        self.use_autofocus = bool(self.get_parameter("autofocus").value)
        self.autofocus_period = float(self.get_parameter("autofocus_period").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.camera_feed_path = (self.get_parameter("camera_feed_path").value
                                 or self._resolve_camera_feed_path())
        self.log_path = self.get_parameter("log_path").value or os.path.join(
            os.path.dirname(self.camera_feed_path),
            "capture_detect_drop_{:%Y%m%d_%H%M%S}.json".format(datetime.now()))

        bad = [t for t in self.drop_order if t not in ("tent", "person")]
        if bad:
            raise ValueError("drop_order may only contain 'tent'/'person', got %s" % bad)

        self.open_pwm = int(self.get_parameter("open_pwm").value)
        self.close_pwm = int(self.get_parameter("close_pwm").value)
        self.servos = {
            "person": [int(c) for c in self.get_parameter("person_servos").value],
            "tent": [int(c) for c in self.get_parameter("tent_servos").value],
        }
        for name, channels in self.servos.items():
            if not channels:
                self.get_logger().warn(
                    "%s has no servo channels configured; its release will be "
                    "announced but nothing will actuate" % name)

        # -- State ------------------------------------------------------
        self._lock = threading.Lock()
        self.targets = {"person": Target("person"), "tent": Target("tent")}
        self.detection_log = []          # every person/tent sighting, in order
        self.processed_images = set()    # image_name values seen on /image_detection
        self.images_at_start = set()     # files already in the folder before we began
        self.triggers_sent = 0
        self.started_at = datetime.now().isoformat(timespec="seconds")

        cb = ReentrantCallbackGroup()

        # -- Publishers -------------------------------------------------
        self.trigger_pub = self.create_publisher(Bool, "/camera/trigger", 10)
        self.camera_command_pub = self.create_publisher(String, "/camera/command", 10)
        self.status_pub = self.create_publisher(StatusText, "/mavros/statustext/send", 10)

        # -- Subscribers ------------------------------------------------
        # RELIABLE + depth 10 matches new_od's publisher QoS. A mismatch here is
        # silent: the subscription simply never receives anything.
        detection_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ImageResult, "/image_detection",
                                 self.image_result_cb, detection_qos,
                                 callback_group=cb)

        # -- Clients ----------------------------------------------------
        self.command_client = self.create_client(
            CommandLong, "/mavros/cmd/command", callback_group=cb)

        # Timers exist only while capturing; None means "not running".
        self.capture_timer = None
        self.autofocus_timer = None

        self.get_logger().info("Camera feed: %s" % self.camera_feed_path)
        self.get_logger().info("Run log:     %s" % self.log_path)
        if self.dry_run:
            self.get_logger().warn("DRY RUN: servo commands will be logged, not sent")

        self._thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._thread.start()

    # -- Sequence -------------------------------------------------------

    def _run_sequence(self):
        try:
            self._wait_for_command_service()
            self._capture_phase()
            self._wait_for_processing()
            self._report_targets()

            self.send_ack("Waiting %.0fs before release" % self.post_capture_wait)
            self.get_logger().info("Waiting %.0fs..." % self.post_capture_wait)
            time.sleep(self.post_capture_wait)

            for i, name in enumerate(self.drop_order):
                if i > 0 and self.inter_drop_wait > 0:
                    time.sleep(self.inter_drop_wait)
                self._release(name)

            self.send_ack("Sequence complete")
            self.get_logger().info("Sequence complete.")
        except Exception as exc:
            self.get_logger().error("Sequence failed: %s" % exc)
            import traceback
            self.get_logger().error(traceback.format_exc())
        finally:
            self._write_log()
            # Let the last statustext reach the wire before we tear down.
            time.sleep(1.0)
            if rclpy.ok():
                rclpy.shutdown()

    def _wait_for_command_service(self):
        """Wait briefly for MAVROS, but never block the run on it.

        With no Pixhawk on /dev/ttyUSB0 the service never appears. The capture
        and detection half of this test is still worth running in that state,
        so warn and continue rather than spinning here forever.
        """
        if self.dry_run:
            return
        if self.command_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("/mavros/cmd/command ready")
            return
        self.get_logger().warn(
            "/mavros/cmd/command NOT available after 10s -- is MAVROS connected "
            "to the FCU? Continuing anyway; servo commands will fail until it "
            "appears.")

    def _capture_phase(self):
        # Snapshot what is already on disk, so the drain check below waits only
        # on images THIS run produced, not on leftovers from earlier runs.
        self.images_at_start = self._list_images()
        self.get_logger().info(
            "%d image(s) already in folder; ignoring those" % len(self.images_at_start))

        # Rack the lens to capture zoom and focus it: the same single idempotent
        # command do_digi_cam_trigger sends on DigiCamCtrl. One command, not a
        # separate zoom + focus, so the two cannot race on siyi_node's worker.
        self.camera_command_pub.publish(String(data="capture_setup"))
        self.get_logger().info("Requested capture_setup (zoom + focus)")

        window = ("until both targets found" if self.capture_seconds <= 0
                  else "%.0fs" % self.capture_seconds)
        self.send_ack("Capture STARTED @ %.1fHz (%s)"
                      % (1.0 / self.capture_period, window))
        self.get_logger().info("Capturing every %.1fs (%s)"
                               % (self.capture_period, window))

        self.capture_timer = self.create_timer(self.capture_period, self._trigger_camera)
        if self.use_autofocus:
            self.autofocus_timer = self.create_timer(
                self.autofocus_period, self._request_autofocus)

        start = time.time()
        while rclpy.ok():
            elapsed = time.time() - start
            if self.capture_seconds > 0 and elapsed >= self.capture_seconds:
                break
            if self.stop_when_both_found and self._both_found():
                self.get_logger().info("Both targets found after %.1fs" % elapsed)
                break
            time.sleep(0.1)

        self._stop_timers()
        self.send_ack("Capture STOPPED after %d triggers" % self.triggers_sent)
        self.get_logger().info("Capture stopped. %d triggers sent." % self.triggers_sent)

    def _wait_for_processing(self):
        """Block until every image captured in this run has come back on
        /image_detection, or until processing_timeout expires.

        Mirrors main_controller's pre-divert hold: pick targets only once
        detection has seen every frame, so 'highest confidence' is not decided
        by whichever images happened to finish first.
        """
        deadline = time.time() + self.processing_timeout
        last_remaining = None
        while rclpy.ok() and time.time() < deadline:
            ours = self._list_images() - self.images_at_start
            with self._lock:
                done = len(ours & self.processed_images)
            remaining = len(ours) - done
            if remaining != last_remaining:
                self.get_logger().info(
                    "Processing: %d/%d done, %d remaining" % (done, len(ours), remaining))
                last_remaining = remaining
            if ours and remaining <= 0:
                self.send_ack("All %d images processed" % len(ours))
                return
            time.sleep(1.0)

        if not (self._list_images() - self.images_at_start):
            self.get_logger().warn(
                "No new images were captured. Is the SIYI camera reachable and "
                "is the siyi node running?")
            self.send_ack("WARNING: no images captured")
        else:
            self.get_logger().warn(
                "Processing timeout after %.0fs; continuing with whatever finished"
                % self.processing_timeout)
            self.send_ack("Processing timeout; continuing")

    def _report_targets(self):
        self.get_logger().info("---- Detection summary ----")
        for name in ("person", "tent"):
            t = self.targets[name]
            if t.found:
                self.get_logger().info(
                    "BEST %s: conf=%.3f at lat=%.7f, lon=%.7f "
                    "(image '%s', seen %s, %d sighting(s))"
                    % (name.upper(), t.confidence, t.lat, t.lon,
                       t.image_name, t.wall_time, t.hits))
                self.send_ack("Best %s: %.2f @ %.6f,%.6f"
                              % (name, t.confidence, t.lat, t.lon))
            else:
                self.get_logger().warn("BEST %s: none detected" % name.upper())
                self.send_ack("No %s detected" % name)
        self.get_logger().info("---------------------------")

    def _release(self, name):
        """Announce, actuate that target's two servos, then confirm done."""
        t = self.targets[name]
        if not t.found and not self.drop_if_not_detected:
            self.get_logger().warn("Skipping %s release: never detected" % name)
            self.send_ack("Skipping %s release (not detected)" % name)
            return

        where = ("%.6f,%.6f" % (t.lat, t.lon)) if t.found else "location unknown"
        if not t.found:
            self.get_logger().warn(
                "No %s was detected -- releasing anyway "
                "(drop_if_not_detected=true)" % name)

        channels = self.servos[name]
        self.send_ack("Releasing payload: %s (%s)" % (name, where))
        self.get_logger().info(
            "RELEASING %s payload -- servos %s -> %dus, hold %.0fs"
            % (name.upper(), channels, self.open_pwm, self.servo_open_seconds))

        for ch in channels:
            self.move_servo(ch, self.open_pwm)
        time.sleep(self.servo_open_seconds)

        if self.close_after_release:
            for ch in channels:
                self.move_servo(ch, self.close_pwm)
            # Give the close commands a moment on the wire before "done".
            time.sleep(0.5)

        self.send_ack("done: %s payload released" % name)
        self.get_logger().info("DONE -- %s payload released" % name)

    # -- Callbacks ------------------------------------------------------

    def image_result_cb(self, msg):
        """One processed image from new_od. Keep the best person and tent."""
        if msg.image_name:
            with self._lock:
                self.processed_images.add(msg.image_name)

        if not msg.detections.detections:
            self.get_logger().info("%s: no objects detected" % msg.image_name)
            return

        self.get_logger().info("%s: %d object(s) detected"
                               % (msg.image_name, len(msg.detections.detections)))

        for detection in msg.detections.detections:
            for result in detection.results:
                obj_class = CLASS_BY_ID.get(result.hypothesis.class_id)
                if obj_class is None:
                    continue  # not an actionable target
                conf = float(result.hypothesis.score)
                now = datetime.now().isoformat(timespec="seconds")

                # ImageResult.latitude/longitude is the GPS fix siyi_node baked
                # into the image FILENAME -- where the aircraft was when the
                # shutter fired, not the ground position of the object. Same
                # approximation main_controller flies on.
                lat, lon = float(msg.latitude), float(msg.longitude)

                with self._lock:
                    t = self.targets[obj_class]
                    t.hits += 1
                    improved = conf > t.confidence
                    old = t.confidence
                    self.detection_log.append({
                        "type": obj_class, "confidence": round(conf, 4),
                        "latitude": lat, "longitude": lon,
                        "image_name": msg.image_name, "detected_at": now,
                        "image_timestamp": msg.timestamp,
                        "new_best": improved,
                    })
                    if improved:
                        t.confidence, t.lat, t.lon = conf, lat, lon
                        t.image_name = msg.image_name
                        t.wall_time = now
                        t.image_timestamp = msg.timestamp
                    current_best = t.confidence

                if lat == 0.0 and lon == 0.0:
                    self.get_logger().warn(
                        "%s at (0.0, 0.0) -- no GPS in filename '%s'; "
                        "location is not usable" % (obj_class, msg.image_name))

                if improved:
                    self.get_logger().info(
                        "NEW BEST %s: %.3f -> %.3f | lat=%.7f lon=%.7f | "
                        "image '%s' | at %s"
                        % (obj_class, old, conf, lat, lon, msg.image_name, now))
                    self.send_ack("Detected %s %.2f" % (obj_class, conf))
                else:
                    self.get_logger().info(
                        "%s conf=%.3f (best stays %.3f)"
                        % (obj_class, conf, current_best))

    def _trigger_camera(self):
        self.trigger_pub.publish(Bool(data=True))
        self.triggers_sent += 1
        self.get_logger().info("Triggering camera... (#%d)" % self.triggers_sent)

    def _request_autofocus(self):
        self.camera_command_pub.publish(String(data="autofocus"))
        self.get_logger().info("Autofocus requested")

    # -- MAVLink --------------------------------------------------------

    def move_servo(self, channel, pwm):
        """MAV_CMD_DO_SET_SERVO via MAVROS -- the same release path
        main_controller.py and payload.py use."""
        if self.dry_run:
            self.get_logger().info("[DRY RUN] servo %d -> %dus" % (channel, pwm))
            return
        try:
            request = CommandLong.Request()
            request.broadcast = False
            request.command = MAV_CMD_DO_SET_SERVO
            request.confirmation = 0
            request.param1 = float(channel)
            request.param2 = float(pwm)
            request.param3 = 0.0
            request.param4 = 0.0
            request.param5 = 0.0
            request.param6 = 0.0
            request.param7 = 0.0

            # Fire-and-forget, as in main_controller: the result comes back on
            # the done-callback. Never spin on the future here.
            future = self.command_client.call_async(request)
            future.add_done_callback(
                lambda f, c=channel, p=pwm: self._on_servo_result(f, c, p))
        except Exception as exc:
            self.get_logger().error("Servo service call failed: %s" % exc)

    def _on_servo_result(self, future, channel, pwm):
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info("[SERVO] Channel %d moved to %dus" % (channel, pwm))
            else:
                self.get_logger().warn("[SERVO] Failed to move channel %d" % channel)
                self.send_ack("Servo %d move FAILED" % channel)
        except Exception as exc:
            self.get_logger().error("Servo result error: %s" % exc)

    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_pub.publish(msg)
        self.get_logger().info("Status: %s" % text)

    # -- Helpers --------------------------------------------------------

    def _both_found(self):
        with self._lock:
            return all(t.found for t in self.targets.values())

    def _list_images(self):
        try:
            return set(f for f in os.listdir(self.camera_feed_path)
                       if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")))
        except OSError as exc:
            self.get_logger().warn("Cannot read %s: %s" % (self.camera_feed_path, exc))
            return set()

    def _stop_timers(self):
        for attr in ("capture_timer", "autofocus_timer"):
            timer = getattr(self, attr)
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                setattr(self, attr, None)

    def _resolve_camera_feed_path(self):
        """Same walk main_controller.py does: climb to the directory holding
        both install/ and src/, then src/video_cam/mapping_photos."""
        search_dir = os.path.dirname(os.path.abspath(__file__))
        ws = None
        for _ in range(10):
            if (os.path.exists(os.path.join(search_dir, "install")) and
                    os.path.exists(os.path.join(search_dir, "src"))):
                ws = search_dir
                break
            parent = os.path.dirname(search_dir)
            if (os.path.exists(os.path.join(parent, "install")) and
                    os.path.exists(os.path.join(parent, "src"))):
                ws = parent
                break
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break
        if ws is None:
            ws = os.path.expanduser("~/astra/ros2_ws")
        return os.path.join(ws, "src", "video_cam", "mapping_photos")

    def _write_log(self):
        with self._lock:
            payload = {
                "run_started": self.started_at,
                "run_finished": datetime.now().isoformat(timespec="seconds"),
                "triggers_sent": self.triggers_sent,
                "images_processed": len(self.processed_images),
                "dry_run": self.dry_run,
                "drop_order": self.drop_order,
                "best": dict((n, t.as_dict()) for n, t in self.targets.items()),
                "all_sightings": self.detection_log,
            }
        try:
            with open(self.log_path, "w") as fh:
                json.dump(payload, fh, indent=2)
            self.get_logger().info("Run log written to %s" % self.log_path)
        except OSError as exc:
            self.get_logger().error("Could not write log %s: %s" % (self.log_path, exc))


def main(args=None):
    rclpy.init(args=args)
    node = CaptureDetectDrop()
    # Multi-threaded: the sequence runs on its own thread and sleeps, while the
    # capture timer and /image_detection callbacks must keep firing.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted -- stopping capture")
        node._stop_timers()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
