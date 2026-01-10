'''
Instructions:

Take a photo
ros2 run video_cam camera_control take_photo

Zoom in 2 steps
ros2 run video_cam camera_control zoom_in 2

Set zoom level to 10
ros2 run video_cam camera_control set_zoom 10

Ask camera for status
ros2 run video_cam camera_control get_status

'''


#!/usr/bin/env python3
"""
SIYI Camera Control Client
Provides command-line interface for camera control
"""
import rclpy
from rclpy.node import Node
from interfaces.srv import CameraCommand
import sys

class CameraControlClient(Node):
    def __init__(self):
        super().__init__('camera_control_client')
        self.client = self.create_client(CameraCommand, '/camera/control')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for camera control service...')
    
    def send_command(self, command, parameter=""):
        request = CameraCommand.Request()
        request.command = command
        request.parameter = parameter
        
        self.get_logger().info(f'Sending command: {command} {parameter}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✓ Success: {response.message}')
            else:
                self.get_logger().error(f'✗ Failed: {response.message}')
            return response.success
        else:
            self.get_logger().error('Service call failed')
            return False

def print_usage():
    print("""
SIYI Camera Control - Usage:

Commands:
  take_photo                  - Capture a photo immediately
  zoom_in [steps]            - Zoom in (default: 1 step)
  zoom_out [steps]           - Zoom out (default: 1 step)
  set_zoom <level>           - Set zoom to specific level (1-30)
  get_status                  - Get camera status

Examples:
  python3 camera_control_client.py take_photo
  python3 camera_control_client.py zoom_in 2
  python3 camera_control_client.py set_zoom 10
  python3 camera_control_client.py get_status

Or with ROS2 run:
  ros2 run video_cam camera_control take_photo
  ros2 run video_cam camera_control zoom_in 5
""")

def main(args=None):
    if len(sys.argv) < 2:
        print_usage()
        return
    
    rclpy.init(args=args)
    client = CameraControlClient()
    
    command = sys.argv[1].lower()
    parameter = sys.argv[2] if len(sys.argv) > 2 else ""
    
    try:
        client.send_command(command, parameter)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
