import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class MinimalPublisher(Node):

    def __init__(self):
        # Publisher
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # store "current temperature" value
        self.current_temperature = 67

        # service 
        self.srv = self.create_service(Trigger, 'get_temperature_callback', self.get_temperature_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Current Temperature is: 67 degrees Farenheit %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

        # update current temperature value
        self.current_temperature = f"{67 + (self.i % 5)} degrees Farenheit"
    
    def get_temperature_callback(self, request, response):
        response.success = True
        response.message = f"The current temperature is {self.current_temperature}"
        self.get_logger().info(f'Service called, returning: "{response.message}"')
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
