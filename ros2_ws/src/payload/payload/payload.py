#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import math
# import Jetson.GPIO as GPIOe
from mavros_msgs.srv import CommandLong, SetMode
from mavros_msgs.msg import RCIn, StatusText
from std_msgs.msg import Float64

SERVO_BOTTLE = 9       # AUX1 = Servo 9
SERVO_BEACON = 10     # AUX2 = Servo 10
PULLEY_OPEN_BOTTLE = 1900       #1050
PULLEY_CLOSE_BOTTLE = 1400      #850
PULLEY_OPEN_BEACON = 1900      #1050
PULLEY_CLOSE_BEACON = 1400 


# #HARD-CODE PARAMS
# NUM_CYCLES = 5
# OPEN_TIME = 1.1  # Time to open pulley in seconds
# CLOSE_TIME = 2  # Time to close pulley in seconds
# PULLEY_RADIUS = 1.3 # inches   #1.22
# PULLEY_RADIUS_FT = PULLEY_RADIUS / 12.0
# CIRCUMFERENCE_FT = 2*math.pi * PULLEY_RADIUS_FT
# DROP_INTERRUPT_FT = 47 #45
# TICKS_MAX = 36  # around 18 full rotations
# # GPIO pin configuration
# LIMIT_SWITCH_PIN = 29          # Physical pin on Jetson board (BOARD mode)
# HALL_SENSOR_PIN = 15

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # Service clients
        self.command_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.wait_for_services()

        # Status publisher
        self.status_pub = self.create_publisher(StatusText, "/mavros/statustext/send", 10)

        self.last_status_time = 0
        self.status_interval = 5  # Throttle interval in seconds

        # GPIO setup
        # GPIO.setmode(GPIO.BOARD)
        # # GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN)
        # # GPIO.setup(HALL_SENSOR_PIN, GPIO.IN)
        # rclpy.on_shutdown(self.cleanup_gpio) #may work, IDK lol

        # self.last_hall_state = GPIO.input(HALL_SENSOR_PIN)
        # self.tick_count = 0
        # self.drop_distance_ft = 0.0
        # self.alt = 0

        # Altitude subscriber
        # self.altitude_sub = self.create_subscription(
        #     Float64,
        #     "/mavros/global_position/rel_alt",
        #     self.altitude_callback,
        #     10)
        # self.altitude_sub  # prevent unused variable warning

        # self.get_logger().info(f"✅ GPIO initialized. Monitoring pin {LIMIT_SWITCH_PIN} for limit switch.")

    def wait_for_services(self):
        clients = [
            ('/mavros/cmd/command', self.command_client),
            ('/mavros/set_mode', self.set_mode),
        ]
        for name, client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting...')
                
    # def cleanup_gpio(self):
    #     GPIO.cleanup()
    #     self.get_logger().info("GPIO cleaned up.")

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

    # def wait_for_limit_switch(self):
    #     self.get_logger().info("Waiting for limit switch release...")
    #     self.send_status("Waiting for limit switch release...")
    #     if GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
    #         self.get_logger().info(f"{GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH}")
    #         self.get_logger().info("Limit switch STILL PRESSED. Proceeding...")
    #         self.send_status("Limit switch STILL PRESSED.")
    #         return True
    #     else:
    #         self.get_logger().info("Limit switch RELEASED. Proceeding...")
    #         self.send_status("Limit switch RELEASED.")
    #         return False


    # Change flight mode
    # def change_mode(self, mode):
    #     self.get_logger().info(f"Setting mode to {mode}...")
    #     try:
    #         request = SetMode.Request()
    #         request.custom_mode = mode

    #         future = self.set_mode.call_async(request)
    #         rclpy.spin_until_future_complete(self, future)
    #         response = future.result()

    #         if response.mode_sent:
    #             self.get_logger().info(f"Mode changed to {mode}")
    #         else:
    #             self.get_logger().error("Failed to change mode")
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {e}")

    def send_status(self, text, throttle=False):
        now = time.time()
        if not throttle or (now - self.last_status_time > self.status_interval):
            status_msg = StatusText()
            status_msg.severity = 6  # NOTICE
            status_msg.text = text
            self.status_pub.publish(status_msg)
            self.last_status_time = now

    # def count_rotations(self):
    #     """Counts rising edges on hall sensor. Call this repeatedly in your main loop to update count."""
    #     current_state = GPIO.input(HALL_SENSOR_PIN)
    #     if self.last_hall_state == GPIO.LOW and current_state == GPIO.HIGH:
    #         self.tick_count += 1
    #     self.last_hall_state = current_state

    # Listens to altitude updates
    # def altitude_callback(self, msg):
    #     self.alt = msg.data
    #     self.get_logger().info(f"Altitude updated: {self.alt} meters")
        
    def run_sequence(self):
        self.move_servo(SERVO_BOTTLE, PULLEY_OPEN_BOTTLE)
        self.move_servo(SERVO_BEACON, PULLEY_OPEN_BEACON)
        # time.sleep(3)
        # self.move_servo(SERVO_BOTTLE, PULLEY_CLOSE_BOTTLE)
        # self.move_servo(SERVO_BEACON, PULLEY_CLOSE_BEACON)
     
     


def main(args=None):
    rclpy.init()
    servo_controller = ServoController()
    servo_controller.run_sequence()
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()