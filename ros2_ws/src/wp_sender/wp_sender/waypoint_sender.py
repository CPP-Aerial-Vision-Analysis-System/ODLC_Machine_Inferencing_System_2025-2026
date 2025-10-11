import rclpy
from rclpy.node import Node
from interfaces.srv import AddWaypoint, DelWaypoint, UpdateMission

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')
        self.client = self.create_client(AddWaypoint, "/addWaypoint")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available')

    # returns response
    def send_request(self, long, lat, alt, index):
        self.get_logger().info("add wp function called")
        try:
            request = AddWaypoint.Request()
            request.latitude = lat
            request.longitude = long
            request.altitude = alt
            request.index = index

            response = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            return response.result()
        except Exception as e:
            self.get_logger().info(f"service call failed: {e}")

def main(args=None):
    rclpy.init()
    wp_client = WaypointClient()
    response = wp_client.send_request(long=30.2, lat=10.5, alt=4.2, index=0)
    print(response)

    wp_client.destroy_node()
    rclpy.shutdown()

    