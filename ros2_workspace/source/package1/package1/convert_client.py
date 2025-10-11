import sys
import rclpy
from rclpy.node import Node
from temperature_interface.srv import ConvertTemperature

class ConvertClient(Node):
    def __init__(self):
        super().__init__('convert_client')
        self.cli = self.create_client(ConvertTemperature, 'convert_temperature')
        self.get_logger().info('Waiting for /convert_temperature …')
        self.cli.wait_for_service()

    def do_request(self, value: float, unit: str):
        req = ConvertTemperature.Request()
        req.value = float(value)
        req.unit  = unit
        return self.cli.call_async(req)

def main(argv=None):
    rclpy.init(args=argv)
    node = ConvertClient()

    # allow: ros2 run package1 convert_client 72.5 F
    value = float(sys.argv[1]) if len(sys.argv) > 1 else 72.5
    unit  = sys.argv[2] if len(sys.argv) > 2 else 'F'

    future = node.do_request(value, unit)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        res = future.result()
        node.get_logger().info(f"Converted: {res.converted_value:.2f} {res.converted_unit}")
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
