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

import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32 #float32

def main():
    rclpy.init()
    node = Node("temperature_publisher")
    pub = node.create_publisher(Float32, "temperature", 10)

    try:
        while rclpy.ok():
            msg = Float32()
            msg.data = 72.5  # pretend temperature value
            pub.publish(msg)

            node.get_logger().info(f"Published temperature: {msg.data} °F")
     # Wait briefly so subscribers can connect
            time.sleep(1)
    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == "__main__": #makes it so that this only runs if this file is ran directly
    main()

