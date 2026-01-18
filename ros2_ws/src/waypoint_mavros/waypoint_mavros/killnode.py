#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from mavros_msgs.srv import ParamPull
from mavros_msgs.msg import StatusText
from rclpy.qos import QoSProfile, qos_profile_sensor_data

#from rclpy.qos import QoSProfile, qos_profile_sensor_data

import os

class KillNode(Node):
    
    def __init__(self):
        super().__init__('Kill_node')

        # create a publisher to send kill commands
        self.message_sender = self.create_publisher(StatusText, '/mavros/statustext/send', 10)

        # create a client to pull all the parameters
        self.param_pull_client = self.create_client(ParamPull, '/mavros/param/pull')
        while not self.param_pull_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('param_pull service not available, waiting...')

        # create a client to get a value of a parameter
        self.param_get_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        while not self.param_get_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('param_get service not available, waiting...')
        
        self.get_logger().info('KillNode initialized and listening for shutdown commands.')  
        self.previous_value = None
        self.param_pull()  # Pull all parameters first

    def param_pull(self):
        req = ParamPull.Request()
        req.force_pull = True  # Force pull all parameters

        try:
            future = self.param_pull_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()

            if result.success:
                self.get_logger().info("Parameter pull successful.")
                self.get_parameter_value()  # Start checking the parameter value
            else:
                self.get_logger().error("Parameter pull failed.")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

    def get_parameter_value(self):
        req = GetParameters.Request()
        req.names = ["SERVO9_FUNCTION"] # might change this to another unused parameter

        try:
            future = self.param_get_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()

            if result is None:
                self.get_logger().error("No result from param_get service")
                return
            if len(result.values) == 0:
                self.get_logger().warn(f'Parameter SERVO9_FUNCTION not found on /mavros/param')
                return
    
            self.get_logger().info(f"SERVO9_FUNCTION value: {result.values[0].integer_value}")
            if self.previous_value is None:
                self.previous_value = result.values[0].integer_value
            elif self.previous_value != result.values[0].integer_value:
                self.get_logger().info(f"SERVO9_FUNCTION changed from {self.previous_value} to {result.values[0].integer_value}")
                self.get_logger().warn('Jetson Shutdown Triggered. Shutting down...')
                self.send_back("Shutdown command received.") #send feedback to gcs
                #self.shutdown_jetson() #shutdown the jetson
                return
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return

        self.get_parameter_value()  # Call again to keep checking
        
    # def listener_callback(self, msg):
    #     self.get_logger().info(f'Received command: {msg.text}')
    #     if "shutdown" in msg.text.lower(): #shutdown command received
    #         self.get_logger().warn('Jetson Shutdown Triggered. Shutting down...')
    #         self.send_back("Shutdown command received.") #send feedback to gcs
    #         #self.shutdown_nodes() #shutdown all nodes
    #         #self.shutdown_jetson() #shutdown the jetson
    
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