#!/usr/bin/env python3
"""
Mission controller: hold after the survey, aim at the targets, drop the payloads, go home.

main_controller.py -- PURPOSE: the mission "brain" that runs on the drone's Jetson (ROS node
main_controller). ArduPilot flies the survey mission by itself; this node watches it and takes
over at the end:
  1. SURVEY: it remembers the best "person" and the best "tent" the detector reports (new_od,
     topic /image_detection) and WHERE ON THE GROUND each one is (see "Drop aiming" below).
  2. PROCESSING: at the last survey waypoint it switches to GUIDED (hover) and waits until every
     photo has been checked by the detector (at most PROCESSING_TIMEOUT_SECONDS).
  3. VISITING: it inserts one waypoint over each target (tent first, then person) right before
     the mission's RTL (service /addWaypoint of waypoint_manager), points the mission at the
     first one and switches back to AUTO.
  4. DROPPING: over each target it hovers (GUIDED) LOITER_SECONDS, opens that target's release
     servo for SERVO_OPEN_SECONDS (MAV_CMD_DO_SET_SERVO), and flies on; after the last one it
     returns home (RTL).

Fixed in October 2026 (found in the Gazebo simulator):
  * Drop aiming: the target position is no longer the drone's GPS position when the photo was
    taken (that missed by 14-25 m), but the ground point under the CENTRE OF THE DETECTION BOX.
    video_cam writes a JSON "sidecar" next to every photo (height above home, heading, field of
    view, zoom, gimbal angles at the shutter) and target_geolocation.py projects the box centre
    onto the ground. A photo without a usable sidecar falls back to the old behaviour (its own
    position); a photo without GPS is ignored instead of sending the drone to lat/lon 0,0.
  * No crash on a slow parameter read: the mission indices from waypoint_manager are read with
    wp_sender.parameter.AsyncParameterReader (never blocks, keeps the old values and retries when
    a read fails). The old blocking read spun the executor inside a callback, timed out,
    returned None and killed this node (2 of 4 simulator runs).
  * Nothing sleeps inside a callback any more: the 4 s + 1 s before resuming AUTO, the 6 s
    loiter, the 3 s servo opening and the 6 s before RTL are one-shot timers
    (timed_steps.StepRunner), so the node keeps handling messages and service answers meanwhile.
    Same timings as before.

Parameters (ros2 run main main_controller.py --ros-args -p name:=value):
  aim_at_detection_box      true   false = old behaviour (fly to the photo's own position)
  default_camera_pitch_deg  -90.0  camera pitch when the camera did not report its attitude
  default_camera_hfov_deg   81.0   field of view when a sidecar has none
  target_ground_height_m    0.0    ground height under the targets above HOME (heights are
                                   measured from home)
  max_aim_offset_m          150.0  an aim point farther than this from the photo is not trusted
How the photo is turned relative to the drone is video_cam's parameter image_top_direction_deg;
it arrives here inside each photo's sidecar.

Talks to: /image_detection (in), /mavros/mission/waypoints + /mavros/mission/reached (in),
/parameter_events + /waypoint_manager/get_parameters (mission indices), /addWaypoint (out),
/mavros/set_mode, /mavros/mission/set_current, /mavros/cmd/command, /mavros/statustext/send (out).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from interfaces.msg import ImageResult
from mavros_msgs.srv import CommandLong, SetMode, WaypointSetCurrent, WaypointPull
from mavros_msgs.msg import WaypointReached, VfrHud, StatusText, WaypointList, StatusText
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Bool
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterEvent

from cv_bridge import CvBridge

from interfaces.srv import GetGPSData, AddWaypoint, DelWaypoint
from wp_sender.parameter import AsyncParameterReader

import time, cv2, math, sys, os, subprocess

# Helpers next to this file (installed beside it by main/CMakeLists.txt).
import target_geolocation as geo
from timed_steps import StepRunner, call_later

ALT = 49      # in meters (this is ~55ft)

# Each target releases via TWO servos.
PERSON_SERVO_CHANNELS = [11]
TENT_SERVO_CHANNELS = [13]

# Pulley PWM positions (same for every release servo).
PULLEY_OPEN = 1400       # released / open   (bench-verified 500-1400 travel)
PULLEY_CLOSE = 500       # closed
SERVO_OPEN_SECONDS = 3.0  # hold OPEN this long before closing back

# Upper bound on the GUIDED "wait for detection to finish" hold. Without it the
# hold is unbounded: a wedged new_od, or a stale image on disk that will never be
# re-announced on /image_detection, leaves the aircraft loitering until a battery
# or GCS failsafe fires. On timeout we divert with whatever finished -- and if
# that is nothing, _divert_to_targets() already falls through to RTL.
PROCESSING_TIMEOUT_SECONDS = 90.0

# Waits of the divert: let the full-mission push land on the FCU, then set the
# current waypoint, then resume AUTO (same values as the old time.sleep calls).
PUSH_SETTLE_SECONDS = 4.0
SET_CURRENT_TO_AUTO_SECONDS = 1.0

# The mission indices waypoint_manager publishes as parameters (read together).
MISSION_INDEX_PARAMS = ['num_waypoints', 'takeoff_index', 'rtl_index', 'next_after_takeoff',
                        'last_before_rtl']

class Detection_Object:
    def __init__(self, type, confidence, latitude, longitude):
        self.type = type          # person or tent
        self.confidence = confidence     
        # self.waypoint_index = waypoint_index # index > 0
        self.lat = latitude       # where we will fly to drop (the aim point)
        self.long = longitude
        self.image_name = ''      # photo it was seen in
        self.aim_note = ''        # how the aim point was computed (for the log)
        
class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Parameters of the drop aiming (see the module docstring).
        self.declare_parameter('aim_at_detection_box', True)
        self.declare_parameter('default_camera_pitch_deg', -90.0)
        self.declare_parameter('default_camera_hfov_deg', 81.0)
        self.declare_parameter('target_ground_height_m', 0.0)
        self.declare_parameter('max_aim_offset_m', 150.0)
        self.aim_at_box = bool(self.get_parameter('aim_at_detection_box').value)
        self.default_pitch = float(self.get_parameter('default_camera_pitch_deg').value)
        self.default_hfov = float(self.get_parameter('default_camera_hfov_deg').value)
        self.ground_height = float(self.get_parameter('target_ground_height_m').value)
        self.max_aim_offset = float(self.get_parameter('max_aim_offset_m').value)

        # Subscribers
        detection_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(ImageResult, "/image_detection", self.image_result_cb, detection_qos)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_cb, 10)
        self.create_subscription(WaypointReached, "/mavros/mission/reached", self.update_waypoint_reached, 1)
        self.create_subscription(ParameterEvent, "/parameter_events", self.parameter_event_cb, 10)

        # Publishers
        self.status_publisher = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # Clients
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Set mode service not available, waiting ...")
        self.add_wp_client = self.create_client(AddWaypoint, "/addWaypoint")
        while not self.add_wp_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for add waypoint service ..s.")
        
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_current_client = self.create_client(WaypointSetCurrent, "/mavros/mission/set_current")

        # variables
        self.num_waypoints = 0
        self.last_before_rtl = 0
        self.next_after_takeoff = 0
        self.takeoff_index = 0
        self.rtl_index = 0
        self.lap = 0
        self.waypoint_reached = 0
        self.human_wp = -1
        self.tent_wp = -1
        self.wait_to_send_wp = True # wait to send new waypoints until reaching last_before_rtl
        self.last_nav_before_rtl = -1  # last physical nav waypoint before RTL
        self.buffer_wp = -1            # NAV waypoint before trigger, for GUIDED hold
        self.processed_image_names = set()  # image filenames received via /image_detection
        self.waiting_for_processing = False  # True when in GUIDED waiting for processing
        self.auto_resumed = False      # set True after one-time AUTO resume; prevents re-triggering
        self.processing_check_timer = None
        self.processing_started_at = 0.0  # wall time the GUIDED processing hold began
        self.mission_phase = "survey"   # survey -> processing -> visiting -> done
        self.visit_plan = []            # ordered targets to drop on (tent first, then person)
        self.visit_idx = 0              # how many targets dropped so far
        self.LOITER_SECONDS = 6.0       # loiter over each target + pre-RTL wait
        self.camera_feed_path = self._resolve_camera_feed_path()
        # Timed sequences (resume AUTO, loiter-and-drop) run here, on one-shot timers.
        self.steps = StepRunner(self)

        # Mission indices: read WITHOUT blocking, re-read on every waypoint_manager parameter
        # event; a failed read keeps the previous values and is retried (see the docstring).
        self.index_reader = AsyncParameterReader(
            self, 'waypoint_manager', MISSION_INDEX_PARAMS, self._on_mission_indices)
        self.index_reader.request('start-up')

        self.waypoints = []
        self.detections = {
            "person": Detection_Object(type="person", confidence=0, latitude=0.0, longitude=0.0),
            "tent": Detection_Object(type="tent", confidence=0, latitude=0.0, longitude=0.0)
        }

        while not self.command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for command service ...")
        # Startup self-test: visibly actuate BOTH targets' servos every launch
        # so we can confirm they are alive before the mission starts.
        # self.get_logger().info("Startup: actuating all servos...")
        # self.actuate_servos(TENT_SERVO_CHANNELS)
        # self.actuate_servos(PERSON_SERVO_CHANNELS)
        # self.get_logger().info("Startup servo test complete.")  # startup servo test disabled

    def fetch_mission_indices(self):
        """Ask for a fresh read of the mission indices (never blocks; see _on_mission_indices)."""
        self.index_reader.request('mission indices may have changed')
        
    def _on_mission_indices(self, values):
        """New mission indices from waypoint_manager (runs on this node's executor)."""
        new = {name: int(values[name]) for name in MISSION_INDEX_PARAMS}
        old = {'num_waypoints': self.num_waypoints, 'takeoff_index': self.takeoff_index,
               'rtl_index': self.rtl_index, 'next_after_takeoff': self.next_after_takeoff,
               'last_before_rtl': self.last_before_rtl}
        self.num_waypoints = new['num_waypoints']
        self.takeoff_index = new['takeoff_index']
        self.rtl_index = new['rtl_index']
        self.next_after_takeoff = new['next_after_takeoff']
        self.last_before_rtl = new['last_before_rtl']
        if new != old:
            self.get_logger().info(
                f"Mission indices: takeoff={self.takeoff_index}, rtl={self.rtl_index}, "
                f"last_before_rtl={self.last_before_rtl} ({self.num_waypoints} items)")
        # The hold trigger depends on rtl_index: recompute it if the mission list is already here
        # (the list may have arrived before these values).
        self._update_last_nav_before_rtl()

    def parameter_event_cb(self, msg: ParameterEvent):
        if msg.node == "/waypoint_manager":
            for changed_param in msg.changed_parameters:
                name = changed_param.name
                value = changed_param.value

                if name in {"num_waypoints", "takeoff_index", "rtl_index", "next_after_takeoff", "last_before_rtl"}:
                    #self.get_logger().info(f"[Param Update] {name} changed")
                    self.fetch_mission_indices()
                    break
    
    def update_waypoint_reached(self, msg):
        self.waypoint_reached = msg.wp_seq      # latest waypoint index reached

        # Trigger = last physical NAV waypoint before RTL.
        trigger_wp = self.last_nav_before_rtl if self.last_nav_before_rtl >= 0 else self.last_before_rtl

        # PHASE survey -> processing: reached the last waypoint before RTL.
        # Hold in GUIDED (do NOT RTL) until EVERY captured image has been
        # processed by object detection; only then do we pick targets.
        if self.mission_phase == "survey" and self.waypoint_reached == trigger_wp:
            self.mission_phase = "processing"
            self.send_ack("Reached last WP. Holding to finish image processing before RTL")
            self.get_logger().info("Holding (GUIDED) at last WP; waiting for all images to be processed")
            self.change_mode("GUIDED")
            self.waiting_for_processing = True
            self.processing_started_at = time.time()
            if self.processing_check_timer is None:
                self.processing_check_timer = self.create_timer(2.0, self._check_all_images_processed)
            return

        # PHASE visiting: arrive over each planned target in turn; loiter, drop.
        if self.mission_phase == "visiting" and self.visit_idx < len(self.visit_plan):
            if self.steps.busy:
                # A drop (or the resume) is still running: a repeated "reached" must not start
                # a second one (the old code was blocked in time.sleep here instead).
                return
            target = self.visit_plan[self.visit_idx]
            if self.waypoint_reached == target["wp"]:
                self._loiter_and_drop(target)

    def _divert_to_targets(self):
        """Run once, after all images are processed. Build the ordered target
        list (1 highest-confidence TENT first, then 1 highest-confidence PERSON;
        only those actually detected), inject a waypoint for each, fly to the
        first. If nothing was found, RTL straight away."""
        if self.mission_phase != "processing":
            return

        order = [
            ("tent",   TENT_SERVO_CHANNELS),
            ("person", PERSON_SERVO_CHANNELS),
        ]
        self.visit_plan = []
        for obj_type, servos in order:
            if self.valid_detection(obj_type):
                d = self.detections[obj_type]
                self.visit_plan.append({
                    "type": obj_type, "lat": d.lat, "lon": d.long, "alt": ALT,
                    "servos": servos, "wp": -1,
                })
                self.get_logger().info(
                    f"Target {obj_type}: conf {d.confidence:.2f} in '{d.image_name}', aim "
                    f"{d.lat:.7f}, {d.long:.7f} ({d.aim_note})")

        if not self.visit_plan:
            self.send_ack("No valid targets found. Returning to launch.")
            self.get_logger().info("No valid targets; commanding RTL")
            self.mission_phase = "done"
            self.change_mode("RTL")
            return

        # Mission indices in VISIT order: tent=base+1, person=base+2, ...
        base = self.last_before_rtl
        for i, t in enumerate(self.visit_plan):
            t["wp"] = base + 1 + i

        # waypoint_manager INSERTS at the index (pushing later items down), so to
        # land visit_plan[0] at base+1 we must insert the list in REVERSE order.
        inject = [{"lat": t["lat"], "lon": t["lon"], "alt": t["alt"], "index": base + 1}
                  for t in reversed(self.visit_plan)]
        self.send_waypoint_data(inject)

        # Where exactly we aim, for the ground station (fits the 50-character STATUSTEXT limit).
        for t in self.visit_plan:
            self.send_ack(f"Aim {t['type']} {t['lat']:.7f},{t['lon']:.7f}")
        names = " -> ".join(f"{t['type']}@{t['wp']}" for t in self.visit_plan)
        self.send_ack(f"Targets: {names}. Diverting.")
        self.get_logger().info(f"Visit plan: {names}")

        self.mission_phase = "visiting"
        self.visit_idx = 0
        self.last_before_rtl = -1
        # Point the vehicle at the first target and resume AUTO.
        self.divert_and_resume(self.visit_plan[0]["wp"])

    def _loiter_and_drop(self, target):
        """Over a target: loiter (GUIDED) LOITER_SECONDS, actuate its servo, then
        continue to the next target -- or wait LOITER_SECONDS and RTL if last.

        Runs as timed steps (never sleeps): loiter -> open -> SERVO_OPEN_SECONDS
        -> close -> next target (AUTO), or -> LOITER_SECONDS -> RTL after the last.
        """
        self.get_logger().info(
            f"Arrived over {target['type']} (WP {target['wp']}). "
            f"Loitering {self.LOITER_SECONDS:.0f}s, then dropping.")
        self.send_ack(f"Over {target['type']}: loiter {self.LOITER_SECONDS:.0f}s then drop")
        self.change_mode("GUIDED")              # hold position above the target
        is_last = self.visit_idx + 1 >= len(self.visit_plan)

        def after_close():
            self.visit_idx += 1
            if not is_last:
                self.change_mode("AUTO")        # fly to the next target
            else:
                self.send_ack(f"All targets done. Waiting {self.LOITER_SECONDS:.0f}s then RTL.")

        def go_home():
            self.mission_phase = "done"
            self.change_mode("RTL")
            self.send_ack("RTL")

        servos = target["servos"]
        steps = [
            (self.LOITER_SECONDS, 'open release servo', lambda: self.open_servos(servos)),
            (SERVO_OPEN_SECONDS, 'close release servo', lambda: self.close_servos(servos)),
            (0.0, 'next target or wait', after_close),
        ]
        if is_last:
            steps.append((self.LOITER_SECONDS, 'RTL', go_home))   # final wait, then home
        self.steps.start(f"drop on the {target['type']}", steps)

    def valid_detection(self, type):
        if type in self.detections:
            if self.detections[type].confidence > 0:
                return True
        return False
        
    def waypoints_cb(self, msg: WaypointList):
        self.waypoints = msg.waypoints
        self._update_last_nav_before_rtl()

    def _update_last_nav_before_rtl(self):
        """Find the last actual NAV waypoint index before RTL, and the buffer
        waypoint (one NAV waypoint before that) used for GUIDED processing hold.
        DigiCamCtrl and other DO_ commands don't trigger WaypointReached,
        so we need the index of the last physical navigation waypoint."""
        NAV_COMMANDS = {16, 17, 18, 19, 20, 21, 22}  # NAV_WAYPOINT, NAV_LOITER_*, NAV_RETURN_TO_LAUNCH, NAV_TAKEOFF
        previous = (self.last_nav_before_rtl, self.buffer_wp)
        self.last_nav_before_rtl = -1
        self.buffer_wp = -1
        if self.rtl_index > 0 and len(self.waypoints) > 0:
            found_last = False
            # The parameter can briefly describe an older mission while a new
            # WaypointList is arriving, so never index beyond the received list.
            for i in range(min(self.rtl_index, len(self.waypoints)) - 1, -1, -1):
                if self.waypoints[i].command in NAV_COMMANDS:
                    if not found_last:
                        self.last_nav_before_rtl = i
                        found_last = True
                    else:
                        self.buffer_wp = i
                        break
        if (self.last_nav_before_rtl, self.buffer_wp) == previous:
            return                       # unchanged: do not repeat the log lines
        if self.last_nav_before_rtl >= 0:
            self.get_logger().info(
                f"Last nav WP before RTL: index {self.last_nav_before_rtl} "
                f"(last_before_rtl={self.last_before_rtl}, rtl={self.rtl_index})"
            )

        if self.buffer_wp >= 0:
            self.get_logger().info(f"Buffer WP (GUIDED processing hold): index {self.buffer_wp}")
    
    def _resolve_camera_feed_path(self):
        """Resolve camera_feed folder path."""
        current_file = os.path.abspath(__file__)
        search_dir = os.path.dirname(current_file)
        ros2_ws_dir = None
        for _ in range(10):
            if (os.path.exists(os.path.join(search_dir, "install")) and
                    os.path.exists(os.path.join(search_dir, "src"))):
                ros2_ws_dir = search_dir
                break
            parent = os.path.dirname(search_dir)
            if (os.path.exists(os.path.join(parent, "install")) and
                    os.path.exists(os.path.join(parent, "src"))):
                ros2_ws_dir = parent
                break
            search_dir = os.path.dirname(search_dir)
            if search_dir == "/":
                break
        if ros2_ws_dir and os.path.exists(os.path.join(ros2_ws_dir, "src")):
            ros2_ws_dir = os.path.join(ros2_ws_dir, "src")
        if ros2_ws_dir is None:
            ros2_ws_dir = "/astra/ros2_ws/src"
        path = os.path.join(ros2_ws_dir, "video_cam", "mapping_photos")
        return path

    def _check_all_images_processed(self):
        """Timer during the 'processing' hold: once every captured image has been
        processed by object detection, stop waiting and divert to the targets."""
        if self.mission_phase != "processing":
            if self.processing_check_timer:
                self.processing_check_timer.cancel()
                self.processing_check_timer = None
            return
        # Checked BEFORE anything that can return early (a missing camera-feed
        # path used to skip the rest of this callback forever), so the hold is
        # bounded no matter which way the check below fails.
        elapsed = time.time() - self.processing_started_at
        timed_out = elapsed >= PROCESSING_TIMEOUT_SECONDS

        try:
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path not found: {self.camera_feed_path}")
                if not timed_out:
                    return

            image_files = set(
                f for f in os.listdir(self.camera_feed_path)
                if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp'))
            )
            total = len(image_files)
            processed = len(self.processed_image_names & image_files)
            remaining = total - processed
            self.get_logger().info(f"Processing check: {processed}/{total} images done, {remaining} remaining")

            if remaining <= 0:
                self.send_ack(f"All {total} images processed. Selecting targets.")
            elif timed_out:
                # Divert with whatever finished rather than loiter indefinitely.
                self.get_logger().warn(
                    f"Processing timeout after {elapsed:.0f}s: {processed}/{total} "
                    f"done, {remaining} never processed. Diverting anyway.")
                self.send_ack(
                    f"Processing timeout ({processed}/{total}). Diverting anyway.")
            else:
                return

            if self.processing_check_timer:
                self.processing_check_timer.cancel()
                self.processing_check_timer = None
            self.waiting_for_processing = False
            self._divert_to_targets()
        except Exception as e:
            self.get_logger().error(f"Error checking processing status: {e}")

    def get_waypoint(self, waypoint_index):     # return copy of an old waypoint given index
        if 0 <= waypoint_index < len(self.waypoints):
            wp = self.waypoints[waypoint_index]
            lat = wp.x_lat
            lon = wp.y_long
            alt = wp.z_alt
            return lat, lon, alt
        else:
            self.get_logger().warn(f"Waypoint index {waypoint_index} out of range")
            return None

    def locate_detection(self, msg, detection):
        """Where to drop for one detection box: an AimResult, or None if the photo has no GPS.

        Uses the photo's sidecar (written by video_cam at the shutter) to project the box centre
        onto the ground (target_geolocation.aim_at_box). Falls back to the photo's own position
        when that is not possible, exactly like the old code did for every detection.
        """
        image_path = os.path.join(self.camera_feed_path, msg.image_name or '')
        meta = geo.read_photo_sidecar(image_path) if msg.image_name else None
        # Prefer the exact numbers: the sidecar's float64 fix, else the 6-decimal file name, and
        # only last the float32 copy in the message (0.4-0.7 m rounding).
        name_lat, name_lon = self._latlon_from_name(msg.image_name)
        if name_lat is None:
            name_lat, name_lon = float(msg.latitude), float(msg.longitude)
        position = geo.photo_position(meta, name_lat, name_lon)
        if position is None:
            return None
        photo_lat, photo_lon = position
        if not self.aim_at_box:
            return geo.AimResult(photo_lat, photo_lon, 'photo',
                                 note='photo position (aim_at_detection_box is false)')
        u = float(detection.bbox.center.position.x)
        v = float(detection.bbox.center.position.y)
        return geo.aim_at_box(meta, photo_lat, photo_lon, u, v, geo.jpeg_size(image_path),
                              default_pitch_deg=self.default_pitch,
                              default_hfov_deg=self.default_hfov,
                              ground_height_m=self.ground_height,
                              max_offset_m=self.max_aim_offset)

    @staticmethod
    def _latlon_from_name(image_name):
        """(lat, lon) from a '<lat> , <lon>.jpg' photo name (same rule as detection/geotag.py)."""
        stem = os.path.splitext(os.path.basename(image_name or ''))[0]
        if ' , ' not in stem:
            return None, None
        lat_s, lon_s = stem.split(' , ', 1)
        try:
            return float(lat_s), float(lon_s)
        except ValueError:
            return None, None

    def image_result_cb(self, msg):
         # Track all processed image names for processing completion check
        if msg.image_name:
            self.processed_image_names.add(msg.image_name)

        if msg.detections.detections:
            self.get_logger().info(f"{len(msg.detections.detections)} object(s) detected!")
            
            #self.get_logger().info(f"{msg}")
            for detection in msg.detections.detections:
                for result in detection.results:
                    obj_id = result.hypothesis.class_id
                    obj_class = {"0": "person", "1": "tent"}.get(obj_id)
                    if obj_class is None:
                        # Ignore classes that are not actionable mission targets.
                        continue
                    obj_conf = result.hypothesis.score

                    if obj_class in self.detections:        # only works if obj_class is saved as 'person' or 'tent'    // TODO: DOUBLE CHECK THIS
                        if obj_conf > self.detections[obj_class].confidence:        # get highest conf
                            aim = self.locate_detection(msg, detection)
                            if aim is None:
                                # No GPS in the photo: there is nowhere to fly to. (The old code
                                # stored 0,0 and would have sent the drone there.)
                                self.get_logger().warn(
                                    f"Ignoring {obj_class} ({obj_conf:.2f}) in "
                                    f"'{msg.image_name}': the photo has no GPS position")
                                continue
                            self.get_logger().info(f"Updating {obj_class}: old_conf={self.detections[obj_class].confidence:.2f}, new_conf={obj_conf:.2f}")
                            self.send_ack(f"Detected {obj_class}")
                            # self.move_servo(11,1400)   # debug: fired on every new best detection
                            # self.move_servo(11,500)
                            # update conf
                            self.detections[obj_class].confidence = obj_conf
                            # update wp_index
                            # self.detections[obj_class].waypoint_index = msg.waypoint_index
                            self.detections[obj_class].lat = aim.lat
                            self.detections[obj_class].long = aim.lon
                            self.detections[obj_class].image_name = msg.image_name
                            self.detections[obj_class].aim_note = aim.note
                            self.get_logger().info(f"Obj at long: {self.detections[obj_class].long}, lat: {self.detections[obj_class].lat}")
                            self.get_logger().info(f"  aimed at the {aim.note}")
        else:
            self.get_logger().info("No objects detected.")
            self.send_ack("No objects detected.")

    def change_mode(self, mode):
        # set_mode service should already be ready from self._wait_for_services
        self.get_logger().info(f"Setting mode to {mode}...")
        try:
            req = SetMode.Request()
            req.custom_mode = mode
            # Fire-and-forget: never block (this runs inside a subscription
            # callback under a single-threaded executor — spinning here deadlocks).
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(lambda f, m=mode: self._on_mode_result(f, m))
        except Exception as e:
            self.get_logger().error(str(e))

    def _on_mode_result(self, future, mode):
        try:
            response = future.result()
            if response is not None and response.mode_sent:
                self.get_logger().info(f"Mode changed to {mode}")
            else:
                self.get_logger().error(f"Failed to change mode to {mode}")
        except Exception as e:
            self.get_logger().error(f"set_mode result error: {e}")

    def send_waypoint_data(self, wp_list):
        # AddWaypoint is scalar (one waypoint per call); send one request each.
        self.get_logger().info(f"Sending {len(wp_list)} waypoints")

        for i, wp in enumerate(wp_list):
            try:
                req = AddWaypoint.Request()
                req.command = 16  # NAV_WAYPOINT
                req.latitude = float(wp["lat"])
                req.longitude = float(wp["lon"])
                req.altitude = float(wp["alt"])
                req.index = int(wp["index"])
                req.channel = 0
                req.pwm = 0

                self.get_logger().info(
                    f"Waypoint {i + 1}: lat={wp['lat']}, lon={wp['lon']}, "
                    f"alt={wp['alt']}, index={wp['index']}"
                )

                self.add_wp_client.call_async(req)
            except Exception as e:
                self.get_logger().error(f"Error sending waypoint {i + 1}: {str(e)}")
    
    def set_current_wp(self, seq):
        try:
            req = WaypointSetCurrent.Request()
            req.wp_seq = int(seq)
            self.set_current_client.call_async(req)
            self.get_logger().info(f"Set current waypoint -> {seq}")
            self.send_ack(f"Set current WP -> {seq}")
        except Exception as e:
            self.get_logger().error(f"set_current failed: {e}")

    def divert_and_resume(self, target_seq):
        # Let the full-mission push land on the FCU, then point the vehicle at
        # the first divert waypoint and resume AUTO so it flies there instead of
        # continuing the RTL. Timed steps, never sleeps: the node keeps handling
        # the /addWaypoint answers and parameter events meanwhile.
        if target_seq is None or target_seq < 0:
            return

        def resume():
            self.change_mode("AUTO")
            self.send_ack(f"Resumed AUTO -> WP {target_seq}")

        self.steps.start('resume AUTO', [
            (PUSH_SETTLE_SECONDS, 'set current waypoint', lambda: self.set_current_wp(target_seq)),
            (SET_CURRENT_TO_AUTO_SECONDS, 'resume AUTO', resume),
        ])

    def open_servos(self, channels):
        """Open the given release servos (all together)."""
        self.get_logger().info(
            f"Opening servos {channels} for {SERVO_OPEN_SECONDS:.0f}s")
        for ch in channels:
            self.move_servo(ch, PULLEY_OPEN)

    def close_servos(self, channels):
        """Close the given release servos (all together)."""
        for ch in channels:
            self.move_servo(ch, PULLEY_CLOSE)

    def actuate_servos(self, channels):
        """Open the given release servos, hold OPEN for SERVO_OPEN_SECONDS,
        then close them back. All servos open together and close together.
        Never blocks: the close is a one-shot timer."""
        self.get_logger().info(
            f"Opening servos {channels} -> hold {SERVO_OPEN_SECONDS:.0f}s -> close")
        self.open_servos(channels)
        call_later(self, SERVO_OPEN_SECONDS, lambda: self.close_servos(channels))

    def move_servo(self, channel, pwm):
        try:
            # Sending request to move servo
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.confirmation = 0
            request.param1 = float(channel)
            request.param2 = float(pwm)
            request.param3 = float(0)
            request.param4 = float(0)
            request.param5 = float(0)
            request.param6 = float(0)
            request.param7 = float(0)

            # Fire-and-forget: do NOT spin here. This runs inside a subscription
            # callback under a single-threaded executor, so spin_until_future_complete
            # would deadlock the node. call_async already puts the command on the wire.
            future = self.command_client.call_async(request)
            future.add_done_callback(lambda f, c=channel, p=pwm: self._on_servo_result(f, c, p))

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def _on_servo_result(self, future, channel, pwm):
        try:
            response = future.result()
            if response is not None and response.success:
                self.get_logger().info(f"[SERVO] Channel {channel} moved to {pwm}us")
                self.send_ack(f"Servo {channel} -> {pwm}")
            else:
                self.get_logger().warn(f"[SERVO] Failed to move channel {channel}")
                self.send_ack(f"Servo {channel} move FAILED")
        except Exception as e:
            self.get_logger().error(f"Servo result error: {e}")

    def send_ack(self, text):
        msg = StatusText()
        msg.severity = 6  # INFO
        msg.text = text
        self.status_publisher.publish(msg)
        self.get_logger().info(f"Status: {text}")

if __name__ == "__main__":
    rclpy.init()
    node = MainController()
    rclpy.spin(node)
