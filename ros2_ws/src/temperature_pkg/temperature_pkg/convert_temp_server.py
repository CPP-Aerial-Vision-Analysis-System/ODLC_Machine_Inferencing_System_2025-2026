import rclpy
from rclpy.node import Node
from temperature_interfaces.srv import ConvertTemperature  # import from your interface pkg


class ConvertTemperatureServer(Node):

    def __init__(self):
        super().__init__('convert_temperature_server')
        # Create service
        self.srv = self.create_service(
            ConvertTemperature,
            'convert_temperature',
            self.convert_callback
        )
        self.get_logger().info("Temperature Converter Service Ready.")

    def convert_callback(self, request, response):
        if request.conversion_type == "FtoC":
            response.converted_temp = (request.input_temp - 32.0) * 5.0/9.0
            self.get_logger().info(
                f"Converting {request.input_temp}F -> {response.converted_temp:.2f}C"
            )
        elif request.conversion_type == "CtoF":
            response.converted_temp = (request.input_temp * 9.0/5.0) + 32.0
            self.get_logger().info(
                f"Converting {request.input_temp}C -> {response.converted_temp:.2f}F"
            )
        else:
            self.get_logger().warn("Unknown conversion type, returning input unchanged")
            response.converted_temp = request.input_temp
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ConvertTemperatureServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
