#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ultralytics_ros.msg import ImageResult
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

# HARD CODED SERVO CHANNEL AND PWM VALUES FOR HUMAN AND TENT OBJECTS (WILL BE CHANGED TO THE RIGHT VALUES LATER)
HUMAN_SERVO_CHANNEL_1 = 9
HUMAN_SERVO_CHANNEL_2 = 10
HUMAN_SERVOS_PWM= 1500

TENT_SERVO_CHANNEL_1 = 11
TENT_SERVO_CHANNEL_2 = 12
TENT_SERVOS_PWM= 1500

class Detection_Object:
    def __init__(self, type, confidence, waypoint_index):
        self.type = type          # person or tent
        self.confidence = confidence     
        self.waypoint_index = waypoint_index # index > 0
        
class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        # Subscribers
        self.create_subscription(ImageResult, "/image_detection", self.image_result_cb, 1)
        self.create_subscription(WaypointList, "/mavros/mission/waypoints", self.waypoints_cb, 1)
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
        self.param_manager = ParameterManager()

        self.fetch_mission_indices()

        self.waypoints = []
        self.detections = {
            "person": Detection_Object(type="person", confidence=0, waypoint_index=0),
            "tent": Detection_Object(type="tent", confidence=0, waypoint_index=0) 
        }

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
        self.waypoint_reached = msg.wp_seq      # store latest waypoint index   

        # UNCOMMENT TO TEST DATA RECEIVED FROM /image_detection
        if self.waypoint_reached == self.last_before_rtl and (self.valid_detection("person") and self.valid_detection("tent") and self.wait_to_send_wp):
            person_lat, person_lon, person_alt = self.get_waypoint(self.detections["person"].waypoint_index)
            tent_lat, tent_lon, tent_alt = self.get_waypoint(self.detections["tent"].waypoint_index)
            # Update new_wp for both detections (MIGHT WORK LMAO)
            self.get_logger().info(f"last before rtl: {self.last_before_rtl}")
            self.human_wp = self.last_before_rtl + 1
            self.tent_wp = self.last_before_rtl + 2
            
            self.send_waypoint_data([
                {"lat": person_lat, "lon": person_lon, "alt": person_alt, "index": self.last_before_rtl + 1},
                {"lat": tent_lat, "lon": tent_lon, "alt": tent_alt, "index": self.last_before_rtl + 1}
            ])
            self.wait_to_send_wp = False
            self.get_logger().info(f"last before rtl: {self.last_before_rtl}")
            self.last_before_rtl = -1

        elif self.waypoint_reached == self.last_before_rtl and (self.valid_detection("person") or self.valid_detection("tent")) and self.wait_to_send_wp:
            # If only one detection is valid, send that object waypoint
            if self.valid_detection("person"):
                person_lat, person_lon, person_alt = self.get_waypoint(self.detections["person"].waypoint_index)
                self.human_wp = self.last_before_rtl + 1
                self.get_logger().info("Only person was detected")
                self.get_logger().info(f"last before rtl: {self.last_before_rtl}")
                self.send_waypoint_data([
                    {"lat": person_lat, "lon": person_lon, "alt": person_alt, "index": self.last_before_rtl + 1}
                ])
                self.wait_to_send_wp = False
                self.get_logger().info(f"after before rtl: {self.last_before_rtl}")
                self.last_before_rtl = -1

            if self.valid_detection("tent"):
                tent_lat, tent_lon, tent_alt = self.get_waypoint(self.detections["tent"].waypoint_index)
                self.tent_wp = self.last_before_rtl + 1
                self.send_waypoint_data([
                    {"lat": tent_lat, "lon": tent_lon, "alt": tent_alt, "index": self.last_before_rtl + 1}
                ])
                self.wait_to_send_wp = False
                self.get_logger().info(f"after before rtl: {self.last_before_rtl}")
                self.last_before_rtl = -1
        
        if self.waypoint_reached == self.human_wp:
            self.get_logger().info("Reached human waypoint, activating servo...")
            self.send_ack("Reached human waypoint, activating servo")
            self.change_mode("GUIDED")
            self.move_human_servo() # Placeholder when testing out in simulation
            # self.move_servo(HUMAN_SERVO_CHANNEL_1, HUMAN_SERVOS_PWM)
            # time.sleep(2)
            # self.move_servo(HUMAN_SERVO_CHANNEL_2, HUMAN_SERVOS_PWM)
            # time.sleep(2)
            self.change_mode("AUTO")
        if self.waypoint_reached == self.tent_wp:
            self.get_logger().info("Reached tent waypoint, activating servo...")
            self.send_ack("Reached tent waypoint, activating servo")
            self.change_mode("GUIDED")
            self.move_tent_servo()
            # self.move_servo(TENT_SERVO_CHANNEL_1, TENT_SERVOS_PWM)
            # time.sleep(2)
            # self.move_servo(TENT_SERVO_CHANNEL_2, TENT_SERVOS_PWM)
            # time.sleep(2)
            self.change_mode("AUTO")
            
        
    def valid_detection(self, type):
        if type in self.detections:
            if self.detections[type].confidence > 0:
                return True
        return False
        
    def waypoints_cb(self, msg: WaypointList):
        self.waypoints = msg.waypoints

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
                            self.send_ack(f"Detected {obj_class} at waypoint {msg.waypoint_index}")
                            # update conf
                            self.detections[obj_class].confidence = obj_conf
                            # update wp_index
                            self.detections[obj_class].waypoint_index = msg.waypoint_index
        else:
            self.get_logger().info("No objects detected.")

    def change_mode(self, mode):
        # set_mode service should already be ready from self._wait_for_services
        self.get_logger().info(f"Setting mode to {mode}...")
        try:
            req = SetMode.Request()
            req.custom_mode = mode
            future = self.set_mode_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()
            
            if response.mode_sent:
                self.get_logger().info(f"Mode changed to {mode}")
            else:
                self.get_logger().error("Failed to change mode")
        except Exception as e:
            self.get_logger().error(str(e))

    def send_waypoint_data(self, wp_list):
        self.get_logger().info(f"Sending {len(wp_list)} waypoints")

        try:
            req = AddWaypoint.Request()

            # Extract coordinates for all waypoints
            req.latitude = [float(wp["lat"]) for wp in wp_list]
            req.longitude = [float(wp["lon"]) for wp in wp_list]
            req.altitude = [float(wp["alt"]) for wp in wp_list]
            req.index = [int(wp["index"]) for wp in wp_list]

            # Log waypoints being added
            for i, wp in enumerate(wp_list):
                self.get_logger().info(
                    f"Waypoint {i + 1}: lat={wp['lat']}, lon={wp['lon']}, "
                    f"alt={wp['alt']}, index={wp['index']}"
                )

            future = self.add_wp_client.call_async(req)
            #rclpy.spin_until_future_complete(self, future) PLEASE COMMENT THIS OUT ON GOD
            if future.result() is not None:
                self.get_logger().info("All waypoints sent successfully")
            else:
                self.get_logger().warn("Failed to add waypoints")
        except Exception as e:
            self.get_logger().error(f"Error sending waypoints: {str(e)}")

    def move_human_servo(self):
        pass

    def move_tent_servo(self):
        pass
    
    def move_servo(self, channel, pwm):
        try:
            # Sending request to move servo
            request = CommandLong.Request()
            request.broadcast = False
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.confirmation = 0
            request.param1 = channel
            request.param2 = pwm
            request.param3 = 0
            request.param4 = 0
            request.param5 = 0
            request.param6 = 0
            request.param7 = 0

            # Get the response from the service
            future = self.command_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response.success:
                self.get_logger().info(f"[SERVO] Channel {channel} moved to {pwm}μs")
                self.send_status(f"Servo {channel} -> {pwm}")
            else:
                self.get_logger().warn(f"[SERVO] Failed to move channel {channel}")
                self.send_status(f"Servo {channel} move FAILED")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

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