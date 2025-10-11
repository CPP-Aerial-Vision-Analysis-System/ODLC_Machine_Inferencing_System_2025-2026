import rclpy
from rclpy.node import Node

from temperature_interface.srv import ConvertTemperature

class ConvertServer(Node):
    def __init__(self):
        super().__init__('convert_server')
        self.srv = self.create_service(ConvertTemperature, 'convert_temperature', self.handle_convert)
        self.get_logger().info('Convert server ready on /convert_temperature')

    def handle_convert(self, request, response):
        val, unit = request.value, request.unit.upper()
        if unit == 'F':
            response.converted_value = (val - 32.0) * 5.0/9.0
            response.converted_unit  = 'C'
        
        elif unit == 'C':
            response.converted_value = val * 9.0/5.0 + 32.0
            response.converted_unit  = 'F'
        
        else:
            # “invalid unit”: return same value and tell caller via log
            self.get_logger().warn(f"Unknown unit '{request.unit}', use 'F' or 'C'")
            response.converted_value = val
            response.converted_unit  = unit
        self.get_logger().info(f"{val} {unit} -> {response.converted_value:.2f} {response.converted_unit}")
        return response

def main():
    rclpy.init()
    node = ConvertServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()