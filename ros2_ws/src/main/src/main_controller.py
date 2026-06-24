#!/usr/bin/env python3

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
from wp_sender.parameter import ParameterManager

import time, cv2, math, sys, os, subprocess

ALT = 16.8      # in meters (this is ~55ft)

# Each target releases via TWO servos.
PERSON_SERVO_CHANNELS = [9, 11]
TENT_SERVO_CHANNELS = [13, 14]

# Pulley PWM positions (same for every release servo).
PULLEY_OPEN = 1900       # released / open   #1050
PULLEY_CLOSE = 1400      # closed            #850
SERVO_OPEN_SECONDS = 3.0  # hold OPEN this long before closing back

class Detection_Object:
    def __init__(self, type, confidence, latitude, longitude):
        self.type = type          # person or tent
        self.confidence = confidence     
        # self.waypoint_index = waypoint_index # index > 0
        self.lat = latitude
        self.long = longitude
        
class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

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
        self.mission_phase = "survey"   # survey -> processing -> visiting -> done
        self.visit_plan = []            # ordered targets to drop on (tent first, then person)
        self.visit_idx = 0              # how many targets dropped so far
        self.LOITER_SECONDS = 3.0       # loiter over each target + pre-RTL wait
        self.camera_feed_path = self._resolve_camera_feed_path()
        self.param_manager = ParameterManager()

        self.fetch_mission_indices()

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
        wp_params = ['num_waypoints', 'takeoff_index', 'rtl_index', 'next_after_takeoff', 'last_before_rtl']
        params = self.param_manager.get_param(self.param_manager.waypoint_client, list_params=wp_params)
        num_waypoints, takeoff_index, rtl_index, next_after_takeoff, last_before_rtl = params.values()
        
        self.num_waypoints = int(num_waypoints)
        self.takeoff_index = int(takeoff_index)
        self.rtl_index = int(rtl_index)
        self.next_after_takeoff = int(next_after_takeoff)
        self.last_before_rtl = int(last_before_rtl)

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
            if self.processing_check_timer is None:
                self.processing_check_timer = self.create_timer(2.0, self._check_all_images_processed)
            return

        # PHASE visiting: arrive over each planned target in turn; loiter, drop.
        if self.mission_phase == "visiting" and self.visit_idx < len(self.visit_plan):
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
        continue to the next target -- or wait LOITER_SECONDS and RTL if last."""
        self.get_logger().info(
            f"Arrived over {target['type']} (WP {target['wp']}). "
            f"Loitering {self.LOITER_SECONDS:.0f}s, then dropping.")
        self.send_ack(f"Over {target['type']}: loiter {self.LOITER_SECONDS:.0f}s then drop")
        self.change_mode("GUIDED")              # hold position above the target
        time.sleep(self.LOITER_SECONDS)         # loiter over the target
        self.actuate_servos(target["servos"])
        self.visit_idx += 1

        if self.visit_idx < len(self.visit_plan):
            self.change_mode("AUTO")            # fly to the next target
        else:
            self.send_ack(f"All targets done. Waiting {self.LOITER_SECONDS:.0f}s then RTL.")
            time.sleep(self.LOITER_SECONDS)     # final wait
            self.mission_phase = "done"
            self.change_mode("RTL")
            self.send_ack("RTL")

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
        self.last_nav_before_rtl = -1
        self.buffer_wp = -1
        if self.rtl_index > 0 and len(self.waypoints) > 0:
            found_last = False
            for i in range(self.rtl_index - 1, -1, -1):
                if self.waypoints[i].command in NAV_COMMANDS:
                    if not found_last:
                        self.last_nav_before_rtl = i
                        found_last = True
                    else:
                        self.buffer_wp = i
                        break
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
        try:
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path not found: {self.camera_feed_path}")
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
                if self.processing_check_timer:
                    self.processing_check_timer.cancel()
                    self.processing_check_timer = None
                self.waiting_for_processing = False
                self._divert_to_targets()
        except Exception as e:
            self.get_logger().error(f"Error checking processing status: {e}")

    def get_waypoint(self, waypoint_index):     # return copy of an old waypoint given index
        if 0 < waypoint_index < len(self.waypoints):
            wp = self.waypoints[waypoint_index]
            lat = wp.x_lat
            lon = wp.y_long
            alt = wp.z_alt
            return lat, lon, alt
        else:
            self.get_logger().warn(f"Waypoint index {waypoint_index} out of range")
            return None

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
                    if obj_id == "0":
                        obj_class = "person"
                    elif obj_id == "1":
                        obj_class = "tent"
                    obj_conf = result.hypothesis.score

                    if obj_class in self.detections:        # only works if obj_class is saved as 'person' or 'tent'    // TODO: DOUBLE CHECK THIS
                        if obj_conf > self.detections[obj_class].confidence:        # get highest conf
                            self.get_logger().info(f"Updating {obj_class}: old_conf={self.detections[obj_class].confidence:.2f}, new_conf={obj_conf:.2f}")
                            self.send_ack(f"Detected {obj_class}")
                            # update conf
                            self.detections[obj_class].confidence = obj_conf
                            # update wp_index
                            # self.detections[obj_class].waypoint_index = msg.waypoint_index
                            self.detections[obj_class].lat = msg.latitude
                            self.detections[obj_class].long = msg.longitude
                            self.get_logger().info(f"Obj at long: {self.detections[obj_class].long}, lat: {self.detections[obj_class].lat}")
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
        # continuing the RTL.
        if target_seq is None or target_seq < 0:
            return
        time.sleep(4)
        self.set_current_wp(target_seq)
        time.sleep(1)
        self.change_mode("AUTO")
        self.send_ack(f"Resumed AUTO -> WP {target_seq}")

    def actuate_servos(self, channels):
        """Open the given release servos, hold OPEN for SERVO_OPEN_SECONDS,
        then close them back. All servos open together and close together."""
        self.get_logger().info(
            f"Opening servos {channels} -> hold {SERVO_OPEN_SECONDS:.0f}s -> close")
        for ch in channels:
            self.move_servo(ch, PULLEY_OPEN)
        time.sleep(SERVO_OPEN_SECONDS)
        for ch in channels:
            self.move_servo(ch, PULLEY_CLOSE)

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