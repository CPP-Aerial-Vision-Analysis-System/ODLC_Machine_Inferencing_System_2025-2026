#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ultralytics_ros.msg import ImageResult
from mavros_msgs.srv import SetMode, WaypointSetCurrent, WaypointPull
from mavros_msgs.msg import WaypointReached, VfrHud, StatusText, WaypointList, StatusText
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Bool
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterEvent

from cv_bridge import CvBridge

from interfaces.srv import GetGPSData, AddWaypoint, DelWaypoint
from wp_sender.parameter import ParameterManager

import time, cv2, math, sys, os, subprocess

from mavros_msgs.msg import StatusText
msg.severity = StatusText.NOTICE

ALT = 16.8      # in meters (this is ~55ft)

# Servo channels and PWM for payload drops (each item held by 2 servos)
HUMAN_SERVO_CHANNEL_1 = 9
HUMAN_SERVO_CHANNEL_2 = 10
HUMAN_SERVOS_PWM = 1500

TENT_SERVO_CHANNEL_1 = 11
TENT_SERVO_CHANNEL_2 = 12
TENT_SERVOS_PWM = 1500

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

        # TODO: use reliable QoS for all the subscribers, but make sure they are mavros compatible
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

        # variables
        self.last_before_rtl = 0
        self.next_after_takeoff = 0
        self.takeoff_index = 0
        self.rtl_index = 0
        # self.lap = 0
        self.waypoint_reached = 0
        self.wait_to_send_wp = True
        self.last_nav_before_rtl = -1  # last physical nav waypoint before RTL
        self.buffer_wp = -1            # NAV waypoint before trigger, for GUIDED hold
        self.processed_image_names = set()  # image filenames received via /image_detection
        self.waiting_for_processing = False  # True when in GUIDED waiting for processing
        self.auto_resumed = False      # set True after one-time AUTO resume; prevents re-triggering
        self.processing_check_timer = None
        self.camera_feed_path = self._resolve_camera_feed_path()
        self.param_manager = ParameterManager()
        self.trigger_wp = -1

        self.fetch_mission_indices()

        self.waypoints = []
        self.detections = {
            "person": Detection_Object(type="person", confidence=0, latitude=0.0, longitude=0.0),
            "tent": Detection_Object(type="tent", confidence=0, latitude=0.0, longitude=0.0) 
        }

    # update the waypoint list that you have even if one of the parameters is changed. (looks like its overworking by changing everything once one of the waypoints is changed, chould only change that one if it will improve our workflow).
    def fetch_mission_indices(self):
        wp_params = ['num_waypoints', 'takeoff_index', 'rtl_index', 'next_after_takeoff', 'last_before_rtl']
        params = self.param_manager.get_param(self.param_manager.waypoint_client, list_params=wp_params)
        num_waypoints, takeoff_index, rtl_index, next_after_takeoff, last_before_rtl = params.values()

        self.num_waypoints = int(num_waypoints)
        self.takeoff_index = int(takeoff_index)
        self.rtl_index = int(rtl_index)
        self.next_after_takeoff = int(next_after_takeoff)
        self.last_before_rtl = int(last_before_rtl)

    # if waypoint_manager is called, check update the name and value with the changed_parameters, if they are one of these, then call fetch_mission_indices() 
    def parameter_event_cb(self, msg: ParameterEvent):
        if msg.node == "/waypoint_manager":
            for changed_param in msg.changed_parameters:
                name = changed_param.name
                value = changed_param.value

                if name in {"num_waypoints", "takeoff_index", "rtl_index", "next_after_takeoff", "last_before_rtl"}:
                    self.fetch_mission_indices()
                    break

    # called when /mavros/mission/reached is triggered, get the wp_sequence, check if we reached buffer_wp, switch to guided, starts a timer and calls _check_all_imgaes_processed()
    def update_waypoint_reached(self, msg):
        self.waypoint_reached = msg.wp_seq      # store latest waypoint index   
        # Buffer waypoint: switch to GUIDED to wait for image processing to finish

        if (self.buffer_wp >= 0
                and self.waypoint_reached == self.buffer_wp
                and not self.waiting_for_processing
                and not self.auto_resumed):
            self.get_logger().info(f"Reached buffer WP {self.buffer_wp}, switching to GUIDED for processing wait")
            self.send_ack(f"Buffer WP {self.buffer_wp}: GUIDED hold for image processing")
            self.change_mode("GUIDED")
            self.waiting_for_processing = True
            if self.processing_check_timer is None:
                self.processing_check_timer = self.create_timer(2.0, self._check_all_images_processed)

        if self.last_nav_before_rtl >= 0:
            trigger_wp = self.last_nav_before_rtl
        else:
            trigger_wp = self.last_before_rtl

        if self.waypoint_reached == trigger_wp and self.wait_to_send_wp:

            invalid_detection_message = f"No valid object was detected."
            both_detected_message     = f"Both person and tent detected!"
            person_detected_message   = f"Only person detected!"
            tent_detected_message     = f"Only a tent was detected!"

            if  (self.valid_detection("person") and self.valid_detection("tent")):
                person_lat = self.detections["person"].lat
                person_lon = self.detections["person"].long

                tent_lat = self.detections["tent"].lat
                tent_lon = self.detections["tent"].long

                
                self.get_logger().info(both_detected_message)
                self.send_ack(both_detected_message)

                insert_base = self.last_before_rtl + 1

                # Mission sequence: fly to tent → release both tent servos → fly to person → release both human servos
                self.send_waypoint_data([
                    {"command": 16,  "lat": tent_lat, "lon": tent_lon, "alt": ALT, "index": insert_base},
                    {"command": 183, "channel": TENT_SERVO_CHANNEL_1, "pwm": TENT_SERVOS_PWM, "index": insert_base + 1},
                    {"command": 183, "channel": TENT_SERVO_CHANNEL_2, "pwm": TENT_SERVOS_PWM, "index": insert_base + 2},
                    {"command": 16,  "lat": person_lat, "lon": person_lon, "alt": ALT, "index": insert_base + 3},
                    {"command": 183, "channel": HUMAN_SERVO_CHANNEL_1, "pwm": HUMAN_SERVOS_PWM, "index": insert_base + 4},
                    {"command": 183, "channel": HUMAN_SERVO_CHANNEL_2, "pwm": HUMAN_SERVOS_PWM, "index": insert_base + 5},
                ])
                self.wait_to_send_wp = False
                self.send_ack(f"Mission: tent drop → person drop (all in AUTO)")
                self.get_logger().info(f"Waypoints+servos sent. last_before_rtl was: {self.last_before_rtl}")
                self.last_before_rtl = -1

            elif (self.valid_detection("person")):
                insert_base = self.last_before_rtl + 1
                person_lat = self.detections["person"].lat
                person_lon = self.detections["person"].long

                self.get_logger().info(person_detected_message)
                self.send_ack(person_detected_message)

                self.send_waypoint_data([
                    {"command": 16,  "lat": person_lat, "lon": person_lon, "alt": ALT, "index": insert_base},
                    {"command": 183, "channel": HUMAN_SERVO_CHANNEL_1, "pwm": HUMAN_SERVOS_PWM, "index": insert_base + 1},
                    {"command": 183, "channel": HUMAN_SERVO_CHANNEL_2, "pwm": HUMAN_SERVOS_PWM, "index": insert_base + 2},
                ])
                self.wait_to_send_wp = False
                self.last_before_rtl = -1

            elif self.valid_detection("tent"):
                tent_lat = self.detections["tent"].lat
                tent_lon = self.detections["tent"].long

                self.get_logger().info(tent_detected_message)
                self.send_ack(tent_detected_message)

                self.send_waypoint_data([
                    {"command": 16,  "lat": tent_lat, "lon": tent_lon, "alt": ALT, "index": insert_base},
                    {"command": 183, "channel": TENT_SERVO_CHANNEL_1, "pwm": TENT_SERVOS_PWM, "index": insert_base + 1},
                    {"command": 183, "channel": TENT_SERVO_CHANNEL_2, "pwm": TENT_SERVOS_PWM, "index": insert_base + 2},
                ])
                self.wait_to_send_wp = False
                self.last_before_rtl = -1
            else:
                self.get_logger().info(invalid_detection_message)
                self.send_ack(invalid_detection_message)
                return

            self.wait_to_send_wp = False
            self.last_before_rtl = -1
            
        
    def valid_detection(self, obj_type):
        return obj_type in self.detections and self.detections[obj_type].confidence > 0
        
    def waypoints_cb(self, msg: WaypointList):
        self.waypoints = msg.waypoints
        self._update_last_nav_before_rtl()

    def _update_last_nav_before_rtl(self):
        # Find the last actual NAV waypoint index before RTL, and the buffer waypoint, used for GUIDED processing hold.
        # DigiCamCtrl and other DO_ commands don't trigger WaypointReached, so we need the index of the last physical navigation waypoint.
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
        
    # called by update_waypoint_reached()
    def _check_all_images_processed(self):
        # check if the images are processed, if so, switches to AUTO (continue mission)
        if self.auto_resumed or not self.waiting_for_processing:
            if self.processing_check_timer:
                self.processing_check_timer.cancel()
                self.processing_check_timer = None
            return

        try:
            if not os.path.exists(self.camera_feed_path):
                self.get_logger().warn(f"Camera feed path not found: {self.camera_feed_path}")
                return

            image_files = set()
            for f in os.listdir(self.camera_feed_path):
                if f.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp')):
                    image_files.add(f)

            total = len(image_files)
            processed = len(self.processed_image_names & image_files)
            remaining = total - processed

            self.get_logger().info(f"Processing check: {processed}/{total} images done, {remaining} remaining")

            if remaining <= 0:
                self.get_logger().info("All images processed! Switching to AUTO (one-time)")
                self.send_ack(f"All {total} images processed, resuming AUTO")
                self.change_mode("AUTO")
                self.auto_resumed = True
                self.waiting_for_processing = False
                if self.processing_check_timer:
                    self.processing_check_timer.cancel()
                    self.processing_check_timer = None
        except Exception as e:
            self.get_logger().error(f"Error checking processing status: {e}")

        if self.buffer_wp >= 0:
            self.get_logger().info(f"Buffer WP (GUIDED processing hold): index {self.buffer_wp}")
    
    # nobody is calling this method
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
            future = self.set_mode_client.call_async(req)
            # rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.mode:
                self.get_logger().info(f"Mode changed to {mode}")
            else:
                self.get_logger().error("Failed to change mode")
        except Exception as e:
            self.get_logger().error(str(e))

    def send_waypoint_data(self, wp_list):
        # Send mission items (NAV waypoints + DO_SET_SERVO) to the waypoint manager.

        self.get_logger().info(f"Sending {len(wp_list)} mission items")

        try:
            for i, wp in enumerate(wp_list):
                req = AddWaypoint.Request()
                req.command = int(wp['command'])
                req.index = int(wp['index'])

                if wp['command'] == 183:
                    req.channel = int(wp['channel'])
                    req.pwm = int(wp['pwm'])
                    req.latitude = 0.0
                    req.longitude = 0.0
                    req.altitude = 0.0
                    self.get_logger().info(
                        f"  Item {i + 1}: DO_SET_SERVO ch={wp['channel']}, "
                        f"pwm={wp['pwm']}, index={wp['index']}"
                    )
                else:
                    req.latitude = float(wp['lat'])
                    req.longitude = float(wp['lon'])
                    req.altitude = float(wp['alt'])
                    req.channel = 0
                    req.pwm = 0
                    self.get_logger().info(
                        f"  Item {i + 1}: NAV_WAYPOINT lat={wp['lat']}, "
                        f"lon={wp['lon']}, alt={wp['alt']}, index={wp['index']}"
                    )

                future = self.add_wp_client.call_async(req)

            self.get_logger().info("All mission items sent")
        except Exception as e:
            self.get_logger().error(f"Error sending mission items: {str(e)}")


    def send_ack(self, text):
        msg = StatusText()

        msg.severity = StatusText.NOTICE
        msg.text = text

        self.status_publisher.publish(msg)

        self.get_logger().info(f"Status: {text}")

if __name__ == "__main__":
    rclpy.init()
    node = MainController()
    rclpy.spin(node)    