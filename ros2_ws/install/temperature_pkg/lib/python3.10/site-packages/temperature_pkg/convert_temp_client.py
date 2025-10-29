import sys
import rclpy
from rclpy.node import Node
from temperature_interfaces.srv import ConvertTemperature

class ConvertClient(Node):
    def __init__(self):
        super().__init__('convert_client')
        self.cli = self.create_client(ConvertTemperature, 'convert_temperature')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')

    def send_request(self, value: float, conversion: str):
        req = ConvertTemperature.Request()
        req.input_temp = value
        req.conversion = conversion
        return self.cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ConvertClient()

    # Parse CLI arguments
    if len(sys.argv) < 3:
        node.get_logger().error("Usage: ros2 run temperature_pkg convert_client <value> <FtoC|CtoF>")
        return

    value = float(sys.argv[1])
    conversion = sys.argv[2]

    future = node.send_request(value, conversion)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Converted result: {future.result().converted_temp}")
    else:
        node.get_logger().error("Service call failed")

    node.destroy_node()
    rclpy.shutdown()
