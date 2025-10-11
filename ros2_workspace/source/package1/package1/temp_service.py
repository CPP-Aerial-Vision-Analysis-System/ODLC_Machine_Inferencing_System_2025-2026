# package1/temp_service.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32

class TemperatureService(Node):
    def __init__(self):
        super().__init__('temperature_service')

        self._last_temp = 0.0  

        # Subscribe to the temperature topic to keep the latest value
        self.create_subscription(Float32, 'temperature', self._temp_cb, 10)

        # Provide a Trigger service that returns the latest temperature
        self.create_service(Trigger, 'get_temperature', self._handle_get_temp)

        # Startup log you asked for
        self.get_logger().info(
            'Temperature service is up. '
            'Subscribed to /temperature and offering /temperature_service/get_temperature'
        )

    def _temp_cb(self, msg: Float32):
        self._last_temp = float(msg.data)

    def _handle_get_temp(self, request, response):
        if self._last_temp is None:
            response.success = False
            response.message = 'No temperature received yet'
        else:
            response.success = True
            response.message = f'{self._last_temp:.2f}'
        self.get_logger().info(
            f'Service called -> success={response.success}, message="{response.message}"'
        )
        return response


def main():
    rclpy.init()
    node = TemperatureService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
