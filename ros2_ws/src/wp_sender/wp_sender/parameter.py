import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue

class ParameterManager(Node):
    def __init__(self):
        super().__init__('parameter_manager')

        self.waypoint_client = self.create_client(GetParameters, "waypoint_manager/get_parameters")

        self._wait_for_services()

        # Wait for all services to be available
    def _wait_for_services(self):
        clients = [('waypoint_manager/get_parameters', self.waypoint_client),]
        for name, client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting...')

    def get_param(self, client, list_params):
        req = GetParameters.Request()
        req.names = list_params

        try:
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result is None or not hasattr(result, 'values'):
                self.get_logger().error("No result or invalid response")
                return None

            # Build a dict of {param_name: python_value}
            params = {}
            for name, val in zip(list_params, result.values):
                # Determine actual type and extract it
                if val.type == 1:   # BOOL
                    params[name] = val.bool_value
                elif val.type == 2: # INTEGER
                    params[name] = val.integer_value
                elif val.type == 3: # DOUBLE
                    params[name] = val.double_value
                elif val.type == 4: # STRING
                    params[name] = val.string_value
                elif val.type == 5: # BYTE_ARRAY (rare)
                    params[name] = bytes(val.byte_array_value)
                elif val.type == 6: # BOOL_ARRAY
                    params[name] = list(val.bool_array_value)
                elif val.type == 7: # INTEGER_ARRAY
                    params[name] = list(val.integer_array_value)
                elif val.type == 8: # DOUBLE_ARRAY
                    params[name] = list(val.double_array_value)
                elif val.type == 9: # STRING_ARRAY
                    params[name] = list(val.string_array_value)
                else:
                    params[name] = None

            return params  # dict of all parameters

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None

def main():
    rclpy.init()

    param_manager = ParameterManager()
    response = param_manager.get_param(param_manager.waypoint_client, list_params=['num_waypoints', 'takeoff_index', 'rtl_index', 'next_after_takeoff', 'last_before_rtl'])
    print(response)

    param_manager.destroy_node()
    rclpy.shutdown()
