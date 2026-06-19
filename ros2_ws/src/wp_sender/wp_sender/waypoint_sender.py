import rclpy
from rclpy.node import Node
from interfaces.srv import AddWaypoint, DelWaypoint, UpdateMission

class WaypointClient(Node):
    def __init__(self):
        super().__init__('waypoint_client')

        # Add Way Point Client
        self.add_wp_client = self.create_client(AddWaypoint, "/addWaypoint")

        # Delete Way Point Client
        self.del_wp_client = self.create_client(DelWaypoint, "/delWaypoint")

        # Update Mission Client
        #self.update_client = self.create_client(UpdateMission, "/updateMission")

        self._wait_for_services()

    # Wait for all services to be available
    def _wait_for_services(self):
        clients = [
            ('/addWaypoint', self.add_wp_client),
            ('/delWaypoint', self.del_wp_client),
            #('/updateMission', self.update_client)
        ]
        for name, client in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{name} service not available, waiting...')
    

    def send_AddWP_request(self, long, lat, alt, index):
        self.get_logger().info("AddWayPoint function called")
        try:
            request = AddWaypoint.Request()
            request.command = 16  # NAV_WAYPOINT
            request.latitude = lat
            request.longitude = long
            request.altitude = alt
            request.index = index
            request.channel = 0
            request.pwm = 0

            response = self.add_wp_client.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            return response.result()
        except Exception as e:
            self.get_logger().info(f"AddWayPoint service call failed: {e}")

    def send_ServoWP_request(self, channel, pwm, index):
        """Insert a DO_SET_SERVO mission item at the given index."""
        self.get_logger().info(f"Adding DO_SET_SERVO: ch={channel}, pwm={pwm}, index={index}")
        try:
            request = AddWaypoint.Request()
            request.command = 183  # MAV_CMD_DO_SET_SERVO
            request.latitude = 0.0
            request.longitude = 0.0
            request.altitude = 0.0
            request.index = index
            request.channel = channel
            request.pwm = pwm

            response = self.add_wp_client.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            return response.result()
        except Exception as e:
            self.get_logger().info(f"ServoWP service call failed: {e}")
    
    # DelWayPoint returns response
    def send_DelWP_request(self, index):
        self.get_logger().info("DelWayPoint function called")
        try:
            request = DelWaypoint.Request()
            request.index = index

            response = self.del_wp_client.call_async(request)
            rclpy.spin_until_future_complete(self, response)
            return response.result()
        except Exception as e:
            self.get_logger().info(f"DelWayPoint service call failed: {e}")

        

def main(args=None):
    rclpy.init()
    wp_client = WaypointClient()
    response1 = wp_client.send_AddWP_request(long=30.2, lat=10.5, alt=4.2, index=0)
    print(response1)

    response2 = wp_client.send_DelWP_request(index=0)
    print(response2)

    wp_client.destroy_node()
    rclpy.shutdown()

    