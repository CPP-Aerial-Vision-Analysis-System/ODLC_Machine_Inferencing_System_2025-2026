#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from mavros_msgs.srv import ParamPull
from mavros_msgs.msg import StatusText
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool
import os
import shutil

class KillNode(Node):
    
    def __init__(self):
        super().__init__('Kill_node')

        # create a publisher to send kill commands
        self.message_sender = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # Instance variables for photo transfering
        self.src_folder = "/home/astra-dev/astra/ros2_ws/src/video_cam/mapping_photos"
        self.dst_folder = "/home/astra-dev/astra/ros2_ws/src/video_cam/camera_feed"
        
        self.get_logger().info('KillNode initialized and listening for shutdown commands.')     
        self.send_back("Killnode is here")

        # create a client to get a value of a parameter
        self.param_get_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        while not self.param_get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('param_get service not available, waiting...')
        
        self.get_logger().info('KillNode initialized and listening for shutdown commands.')  
        self.previous_value = None
        self.param_pull()  # Pull all parameters first

    def listener_callback(self, msg):
        if "systemid" in msg.text.lower(): #shutdown command received
            self.get_logger().warn('Jetson Shutdown Triggered. Shutting down...')
            self.send_back("Shutdown command received.") #send feedback to gcs
            #self.shutdown_nodes() #shutdown all nodes
            # First transfers all the photos
            self.transfer_photos()
            self.get_logger().info("Transfering completed")
            self.send_back("Transfering Completed, shutting down now")
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
                
    def transfer_photos(self):
        os.makedirs(self.dst_folder, exist_ok = True)
        
        for file in os.listdir(self.src_folder):
            if file.lower().endswith((".png", ".jpg", ".jpeg")):
                src_path = os.path.join(self.src_folder, file)
                dst_path = os.path.join(self.dst_folder, file)
                shutil.move(src_path, dst_path)

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