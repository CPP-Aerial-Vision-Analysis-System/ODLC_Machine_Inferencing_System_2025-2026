import rclpy
from rclpy.node import Node

from mavros_msgs.msg import StatusText
from rclpy.qos import QoSProfile, qos_profile_sensor_data

import os

class KillNode(Node):
    
    def __init__(self):
        super().__init__('Kill_node')
        #create a subscriber to listen for kill commands
        self.command_listener = self.create_subscription(
            StatusText,
            '/mavros/statustext/recv',
            self.listener_callback,
            qos_profile_sensor_data)
        self.command_listener  # prevent unused variable warning

        #create a publisher to send kill commands
        self.message_sender = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        self.get_logger().info('KillNode initialized and listening for shutdown commands.')     


    def listener_callback(self, msg):
        if "systemid" in msg.text.lower(): #shutdown command received
            self.get_logger().warn('Jetson Shutdown Triggered. Shutting down...')
            self.send_back("Shutdown command received.") #send feedback to gcs
            #self.shutdown_nodes() #shutdown all nodes
            self.shutdown_jetson() #shutdown the jetson

    
    def send_back(self, text):
        # feedback to gcs (mission planner in messages tab)
        msg = StatusText()
        msg.severity = 6  # INFO severity level. 6 = notice/info
        msg.text = text
        self.message_sender.publish(msg) #publish the message

    
    #def shutdown_nodes(self):
        #shutdown all nodes
    #    self.get_logger().info('Shutting down all ROS2 nodes...')

    #    node = rclpy.create_node('node_name_getter') #temporary node to get list of nodes
    #   node_names = [name for name, namespace in node.get_node_names_and_namespaces()] #get list of node names
    #   node.destroy_node() #destroy the temporary node

    #   for node_name in node_names:
    #       if node_name != self.get_name(): #don't kill self
    #           self.get_logger().info(f'Shutting down node: {node_name}')
    #           try:
    #                subprocess.run(['pkill', '-f', node_name], check=True)
    #           except subprocess.CalledProcessError as e:
    #                self.get_logger().warn(f'Failed to kill node {node_name}: {e}')
                

    def shutdown_jetson(self):
        self.get_logger().info('Executing Jetson shutdown command...')
        password = "UAV_Lab"
        os.system(f"echo {password} | sudo -S shutdown -h now") #shutdown the jetson
    
def main(args=None):
    rclpy.init(args=args)

    kill_node = KillNode()

    rclpy.spin(kill_node)

    kill_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()