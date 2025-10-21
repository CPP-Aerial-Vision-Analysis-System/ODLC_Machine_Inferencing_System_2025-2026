import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue

class ParameterManager(Node):
    def __init__(self):
        super().__init__('parameter_manager')

        self.tracknode_client = self.create_client(GetParameters, "tracker_node/get_parameters")
        self.waypoint_client = self.create_client(GetParameters, "waypoint_manager/get_parameters")

        self._wait_for_services()

        # Wait for all services to be available
    def _wait_for_services(self):
        clients = [
            ('tracker_node/get_parameters', self.tracknode_client),
            ('waypoint_manager/get_parameters', self.waypoint_client),
        ]
        for name, client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting...')

    def get_param(self, client, list_params, bool_value=None, integer_value=None, double_value=None, string_value=None, bool_array_value=None
                    integer_array_value=None, double_array_value=None, string_array_value=None):
        req = GetParameters.Request()
        req.names = list_params

        try:
        # call service
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            response = future.result().values[0]
            
            if bool_value:
                return response.bool_value
            elif integer_value:
                return response.integer_value
            elif double_value:
                return response.double_value
            elif string_value:
                return response.string_value
            elif bool_value_array:
                return response.bool_value_array
            elif integer_array_value:
                return response.integer_array_value
            elif double_array_value:
                return response.double_array_value
            elif string_array_value:
                return response.string_array_value
            else:
                return "could not get parameter"

        except Exception as e:
            self.get_logger().info(f"{e}")

def main():
    rclpy.init()

    param_manager = ParameterManager()
    response = param_manager.get_param(param_manager.waypoint_client, list_params=['num_waypoints'], integer_value=True)
    print(response)

    param_manager.destroy_node()
    rclpy.shutdown()
