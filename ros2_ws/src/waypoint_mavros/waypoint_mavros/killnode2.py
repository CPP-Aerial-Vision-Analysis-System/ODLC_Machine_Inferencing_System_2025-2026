#!/usr/bin/env python3

"""
Enhanced Kill Node for Remote System Shutdown

This node monitors a specific MAVROS parameter (SERVO9_FUNCTION) for changes
and triggers a system shutdown when the parameter value changes. This allows
remote shutdown capability via ground control station.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from mavros_msgs.srv import ParamPull
from mavros_msgs.msg import StatusText
import os
import time

# Constants
MONITORED_PARAMETER = "SERVO9_FUNCTION"
PARAM_CHECK_INTERVAL = 2.0  # seconds
SERVICE_TIMEOUT = 1.0
MAX_SERVICE_RETRIES = 5
STATUS_INFO_SEVERITY = 6


class KillNode(Node):
    """
    Monitor MAVROS parameters and trigger system shutdown on parameter change.
    
    This node continuously monitors a specific parameter and when it detects
    a change, it initiates a graceful shutdown sequence including notification
    to the ground control station.
    """
    
    def __init__(self):
        super().__init__('kill_node')

        # State variables
        self.previous_value = None
        self.shutdown_initiated = False
        self.param_pull_success = False
        
        # Debounce state for safety
        self.trigger_detected_at = None
        self.trigger_confirmed_value = None
        self.debounce_duration = 3.0  # seconds - require 3 seconds of consistent change
        
        # Setup publisher for status messages
        self._setup_publisher()
        
        # Setup service clients
        self._setup_service_clients()
        
        # Create timer for periodic parameter checking
        self.param_check_timer = self.create_timer(
            PARAM_CHECK_INTERVAL, 
            self.check_parameter_callback
        )
        
        self.get_logger().info('KillNode initialized and monitoring for shutdown commands')
        
        # Initial parameter pull
        self.pull_parameters()

    def _setup_publisher(self):
        """Setup publisher for sending status messages to GCS."""
        self.message_sender = self.create_publisher(
            StatusText, 
            '/mavros/statustext/send', 
            10
        )

    def _setup_service_clients(self):
        """Setup and wait for MAVROS service clients."""
        # Parameter pull service client
        self.param_pull_client = self.create_client(ParamPull, '/mavros/param/pull')
        if not self._wait_for_service(self.param_pull_client, 'param_pull'):
            self.get_logger().error('Failed to connect to param_pull service')
            return

        # Parameter get service client
        self.param_get_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        if not self._wait_for_service(self.param_get_client, 'param_get'):
            self.get_logger().error('Failed to connect to param_get service')
            return

    def _wait_for_service(self, client, service_name, max_retries=MAX_SERVICE_RETRIES):
        """
        Wait for a service to become available with timeout and retries.
        
        Args:
            client: The service client to wait for.
            service_name (str): Name of the service for logging.
            max_retries (int): Maximum number of retry attempts.
            
        Returns:
            bool: True if service is available, False otherwise.
        """
        retry_count = 0
        while not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            retry_count += 1
            if retry_count >= max_retries:
                self.get_logger().error(
                    f'{service_name} service not available after {max_retries} attempts'
                )
                return False
            self.get_logger().info(
                f'{service_name} service not available, waiting... '
                f'(attempt {retry_count}/{max_retries})'
            )
        
        self.get_logger().info(f'{service_name} service is ready')
        return True

    def pull_parameters(self):
        """
        Pull all parameters from the autopilot asynchronously.
        
        This forces a synchronization of all parameters from the flight controller
        to ensure we have the latest values.
        """
        req = ParamPull.Request()
        req.force_pull = True

        try:
            future = self.param_pull_client.call_async(req)
            
            def done_callback(fut):
                try:
                    result = fut.result()
                    if result and result.success:
                        self.get_logger().info(
                            f"Successfully pulled {result.param_received} parameters from autopilot"
                        )
                        self.param_pull_success = True
                    else:
                        self.get_logger().error("Parameter pull failed")
                        self.param_pull_success = False
                except Exception as e:
                    self.get_logger().error(f"Exception in param pull callback: {e}")
                    self.param_pull_success = False
            
            future.add_done_callback(done_callback)

        except Exception as e:
            self.get_logger().error(f"Exception during parameter pull: {e}")
            self.param_pull_success = False

    def get_parameter_value(self, callback):
        """
        Get the current value of the monitored parameter asynchronously.
        
        Args:
            callback: Function to call with result (value or None if failed).
        """
        req = GetParameters.Request()
        req.names = [MONITORED_PARAMETER]

        try:
            future = self.param_get_client.call_async(req)
            
            def done_callback(fut):
                try:
                    result = fut.result()
                    
                    if result is None:
                        self.get_logger().error("No result from param_get service")
                        callback(None)
                        return
                        
                    if len(result.values) == 0:
                        self.get_logger().warn(
                            f'Parameter {MONITORED_PARAMETER} not found in autopilot parameters'
                        )
                        callback(None)
                        return
                    
                    current_value = result.values[0].integer_value
                    callback(current_value)
                    
                except Exception as e:
                    self.get_logger().error(f"Exception in param get callback: {e}")
                    callback(None)
            
            future.add_done_callback(done_callback)

        except Exception as e:
            self.get_logger().error(f"Exception during parameter get: {e}")
            callback(None)

    def check_parameter_callback(self):
        """
        Timer callback to periodically check parameter value with debounce.
        
        This is called at regular intervals to monitor for parameter changes.
        Debounce logic ensures we don't trigger on transient glitches.
        """
        # Skip if shutdown already initiated
        if self.shutdown_initiated:
            return
            
        # Skip if initial parameter pull hasn't succeeded yet
        if not self.param_pull_success:
            self.get_logger().warn("Waiting for initial parameter pull to complete")
            self.pull_parameters()
            return
        
        # Get current parameter value asynchronously
        def on_value_received(current_value):
            if current_value is None:
                # Reset debounce on read failure
                self.trigger_detected_at = None
                self.trigger_confirmed_value = None
                return
            
            # Log current value periodically (every 30 seconds)
            if int(time.time()) % 30 == 0:
                self.get_logger().info(f"{MONITORED_PARAMETER} current value: {current_value}")
            
            # Initialize previous value on first check
            if self.previous_value is None:
                self.previous_value = current_value
                self.get_logger().info(
                    f"Monitoring {MONITORED_PARAMETER} (initial value: {current_value})"
                )
                return
            
            # Check if value changed back to original
            if current_value == self.previous_value:
                # Value is normal - reset debounce
                if self.trigger_detected_at is not None:
                    self.get_logger().info(
                        f"Parameter returned to normal value {current_value} - canceling shutdown"
                    )
                    self.trigger_detected_at = None
                    self.trigger_confirmed_value = None
                return
            
            # Value is different from baseline
            current_time = time.time()
            
            # First detection of change
            if self.trigger_detected_at is None:
                self.trigger_detected_at = current_time
                self.trigger_confirmed_value = current_value
                self.get_logger().warn(
                    f"Shutdown trigger detected: {MONITORED_PARAMETER} changed from "
                    f"{self.previous_value} to {current_value}. "
                    f"Confirming for {self.debounce_duration}s..."
                )
                self.send_status_message(
                    f"Shutdown trigger detected - confirming..."
                )
                return
            
            # Check if value changed again (flickering)
            if current_value != self.trigger_confirmed_value:
                self.get_logger().warn(
                    f"Parameter value flickering ({self.trigger_confirmed_value} → {current_value}). "
                    "Resetting debounce."
                )
                self.trigger_detected_at = current_time
                self.trigger_confirmed_value = current_value
                return
            
            # Check if debounce period has elapsed
            elapsed = current_time - self.trigger_detected_at
            if elapsed >= self.debounce_duration:
                # Confirmed - initiate shutdown
                self.get_logger().warn(
                    f"{MONITORED_PARAMETER} confirmed changed from {self.previous_value} "
                    f"to {current_value} for {elapsed:.1f}s"
                )
                self.get_logger().warn('Shutdown trigger confirmed!')
                self.initiate_shutdown()
            else:
                # Still waiting for confirmation
                remaining = self.debounce_duration - elapsed
                if int(elapsed) % 1 == 0:  # Log every second
                    self.get_logger().info(
                        f"Shutdown trigger holding... {remaining:.1f}s remaining"
                    )
        
        self.get_parameter_value(callback=on_value_received)
            
    def initiate_shutdown(self):
        """
        Initiate the system shutdown sequence.
        
        This function:
        1. Sets the shutdown flag to prevent multiple shutdown attempts
        2. Sends acknowledgment to ground control station
        3. Gives time for message to be transmitted
        4. Triggers the system shutdown
        """
        if self.shutdown_initiated:
            return
            
        self.shutdown_initiated = True
        
        # Send feedback to GCS
        self.send_status_message("Shutdown command received. System shutting down...")
        
        # Give time for message to reach GCS
        time.sleep(2.0)
        
        # Trigger shutdown
        self.get_logger().warn('Initiating system shutdown...')
        self.shutdown_system()

    def send_status_message(self, text):
        """
        Send a status message to the ground control station.
        
        Args:
            text (str): Message text to send.
        """
        msg = StatusText()
        msg.severity = STATUS_INFO_SEVERITY  # INFO/NOTICE level
        msg.text = text
        self.message_sender.publish(msg)
        self.get_logger().info(f"Status message sent: {text}")

    def shutdown_system(self):
        """
        Execute the system shutdown command.
        
        This uses a privileged command to shutdown the Jetson. The password
        should ideally be stored more securely or the script should be run
        with appropriate sudo permissions configured via sudoers file.
        """
        self.get_logger().warn('Executing system shutdown command...')
        
        try:
            # Note: For production, configure sudoers to allow shutdown without password
            # or use systemd service with proper permissions
            password = "UAV_Lab"
            shutdown_cmd = f"echo {password} | sudo -S shutdown -h now"
            
            # Execute shutdown
            os.system(shutdown_cmd)
            
        except Exception as e:
            self.get_logger().error(f"Failed to execute shutdown: {e}")
            self.send_status_message(f"Shutdown failed: {e}")


def main(args=None):
    """Entry point for the kill node."""
    rclpy.init(args=args)

    kill_node = KillNode()

    try:
        rclpy.spin(kill_node)
    except KeyboardInterrupt:
        kill_node.get_logger().info('KillNode interrupted by user')
    except Exception as e:
        kill_node.get_logger().error(f'KillNode error: {e}')
    finally:
        # Cleanup
        kill_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
