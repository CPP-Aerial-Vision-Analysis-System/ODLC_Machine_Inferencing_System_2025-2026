#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import time
import math
import Jetson.GPIO as GPIO
from mavros_msgs.srv import CommandLong, SetMode
from mavros_msgs.msg import RCIn, StatusText
from std_msgs.msg import Float64

SERVO_PULLEY = 9       # AUX1 = Servo 9
SERVO_SCISSOR = 10     # AUX2 = Servo 10
PULLEY_OPEN = 925       #1050
PULLEY_CLOSE = 725      #850
SCISSOR_OPEN = 1800
SCISSOR_CUT = 800

#HARD-CODE PARAMS
NUM_CYCLES = 5
OPEN_TIME = 1.1  # Time to open pulley in seconds
CLOSE_TIME = 2  # Time to close pulley in seconds
PULLEY_RADIUS = 1.3 # inches   #1.22
PULLEY_RADIUS_FT = PULLEY_RADIUS / 12.0
CIRCUMFERENCE_FT = 2*math.pi * PULLEY_RADIUS_FT
DROP_INTERRUPT_FT = 47 #45
TICKS_MAX = 36  # around 18 full rotations
# GPIO pin configuration
LIMIT_SWITCH_PIN = 29          # Physical pin on Jetson board (BOARD mode)
HALL_SENSOR_PIN = 15

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
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LIMIT_SWITCH_PIN, GPIO.IN)
        GPIO.setup(HALL_SENSOR_PIN, GPIO.IN)
        rclpy.on_shutdown(self.cleanup_gpio) #may work, IDK lol

        self.last_hall_state = GPIO.input(HALL_SENSOR_PIN)
        self.tick_count = 0
        self.drop_distance_ft = 0.0
        self.alt = 0

        # Altitude subscriber
        self.altitude_sub = self.create_subscription(
            Float64,
            "/mavros/global_position/rel_alt",
            self.altitude_callback,
            10)
        self.altitude_sub  # prevent unused variable warning

        self.get_logger().info(f"✅ GPIO initialized. Monitoring pin {LIMIT_SWITCH_PIN} for limit switch.")

    def wait_for_services(self):
        clients = [
            ('/mavros/cmd/command', self.command_client),
            ('/mavros/set_mode', self.set_mode),
        ]
        for name, client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting...')
                
    def cleanup_gpio(self):
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up.")

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

    def wait_for_limit_switch(self):
        self.get_logger().info("Waiting for limit switch release...")
        self.send_status("Waiting for limit switch release...")
        if GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH:
            self.get_logger().info(f"{GPIO.input(LIMIT_SWITCH_PIN) == GPIO.HIGH}")
            self.get_logger().info("Limit switch STILL PRESSED. Proceeding...")
            self.send_status("Limit switch STILL PRESSED.")
            return True
        else:
            self.get_logger().info("Limit switch RELEASED. Proceeding...")
            self.send_status("Limit switch RELEASED.")
            return False


    # Change flight mode
    def change_mode(self, mode):
        self.get_logger().info(f"Setting mode to {mode}...")
        try:
            request = SetMode.Request()
            request.custom_mode = mode

            future = self.set_mode.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response.mode_sent:
                self.get_logger().info(f"Mode changed to {mode}")
            else:
                self.get_logger().error("Failed to change mode")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def send_status(self, text, throttle=False):
        now = time.time()
        if not throttle or (now - self.last_status_time > self.status_interval):
            status_msg = StatusText()
            status_msg.severity = 6  # NOTICE
            status_msg.text = text
            self.status_pub.publish(status_msg)
            self.last_status_time = now

    def count_rotations(self):
        """Counts rising edges on hall sensor. Call this repeatedly in your main loop to update count."""
        current_state = GPIO.input(HALL_SENSOR_PIN)
        if self.last_hall_state == GPIO.LOW and current_state == GPIO.HIGH:
            self.tick_count += 1
        self.last_hall_state = current_state

    # Listens to altitude updates
    def altitude_callback(self, msg):
        self.alt = msg.data
        self.get_logger().info(f"Altitude updated: {self.alt} meters")
        
    def run_sequence(self):
        self.tick_count = 0
        self.drop_distance_ft = 0

        for i in range(NUM_CYCLES):
            ticks_count_cycle_start = self.tick_count
            open = OPEN_TIME
            if(i == NUM_CYCLES - 1):
                try:
                    height = self.alt * 3.28 
                    if height < 25:
                        height = 57

                except:
                    height = 57
                drop_feet = height - DROP_INTERRUPT_FT
                num_revs_needed = drop_feet / CIRCUMFERENCE_FT
                ticks_needed = int(num_revs_needed * 2)  # 2 ticks per revolution
                open = 0.7
                self.get_logger().info(
            f"Last cycle: Altitude={height:.2f} ft, "
            f"Target drop={drop_feet:.2f} ft, "
            f"Num revs needed={num_revs_needed:.2f}, "
            f"Ticks needed={ticks_needed:.2f}, "
            )
            else:
                ticks_needed = TICKS_MAX
            self.get_logger().info(f"Cycle {i+1}/5: Opening pulley")
            self.move_servo(SERVO_PULLEY, PULLEY_OPEN)
            start = time.time()
            while time.time() - start < open:
                self.count_rotations()
                ticks_this_cycle = self.tick_count - ticks_count_cycle_start
                if ticks_this_cycle >= ticks_needed:
                    break
                time.sleep(0.001)  # Adjust sleep time as needed for your application

            self.move_servo(SERVO_PULLEY, PULLEY_CLOSE)
            self.get_logger().info(f"Cycle {i+1}/3: Closing pulley")
            time.sleep(CLOSE_TIME)

            ticks_this_cycle = self.tick_count - ticks_count_cycle_start
            cycle_revolutions = ticks_this_cycle / 2.0  # 2 ticks per revolution
            cycle_drop = cycle_revolutions * CIRCUMFERENCE_FT
            self.drop_distance_ft += cycle_drop

            status = (
                f"[CYCLE {i+1}] Hall rotations: {cycle_revolutions}, "
                f"[CYCLE {i+1}] Drop Distance:  {cycle_drop:.2f} ft"
                f"Total Drop Distance: {self.drop_distance_ft:.2f} ft"
            )
            print(status)
            self.get_logger().info(status)
            self.send_status(status)

        self.get_logger().info("SEQUENCE DONE. Waiting 5s before scissors cut...")
        self.send_status("SEQUENCE. Waiting 5s for scissors cut...")
        time.sleep(5)

        self.move_servo(SERVO_SCISSOR, SCISSOR_CUT)
        self.get_logger().info("Scissors cut.")
        self.send_status("Scissors cut.")
        time.sleep(1)

        # Wait for limit switch to be released before AUTO mode
        while self.wait_for_limit_switch():
                self.get_logger().info("Limit Switch NOT RELEASED. Initiating Cut Sequence.")
                self.send_status("Limit Switch NOT RELEASED. Initiating Cut Sequence.")
                self.move_servo(SERVO_SCISSOR, SCISSOR_OPEN)
                time.sleep(3)
                self.move_servo(SERVO_SCISSOR, SCISSOR_CUT)
                time.sleep(3)
        message = "Limit switch released. Proceeding to AUTO in 5 secs."
        self.send_status(message)
        time.sleep(5)
        self.change_mode("AUTO")
        self.send_status("Changed to AUTO mode.")

if __name__ == '__main__':
    rclpy.init()
    servo_controller = ServoController()
    servo_controller.run_sequence()
    servo_controller.destroy_node()
    rclpy.shutdown()