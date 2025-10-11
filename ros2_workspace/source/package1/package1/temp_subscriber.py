# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__("temperature_subscriber")

        # Subscribe to the "temperature" topic
        # Queue size = 10 (same as publisher)
        self.sub = self.create_subscription(
            Float32,          # message type
            "temperature",    # topic name
            self.listener_callback,  # callback when msg arrives
            10                # queue size
        )

    def listener_callback(self, msg):
        # msg.data is the float value sent by publisher
        self.get_logger().info(f"Received temperature: {msg.data} °F")

def main():
    rclpy.init()
    node = TemperatureSubscriber()

    # Keep spinning so it listens continuously
    rclpy.spin(node)

    # Cleanup when done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()