"""
sim_bringup/odom_tf.py -- PURPOSE: publish the drone's position as a TF frame so RViz can draw and follow it.

Gazebo's OdometryPublisher (on the drone model) gives /sim/drone/odometry (bridged to ROS as
nav_msgs/Odometry, 50 Hz): the drone's pose in the Gazebo world frame (x = east, y = north, z = up).
RViz needs TF frames instead. This small node:
  * broadcasts one TF per odometry message: <odometry header frame> -> <odometry child frame>
    (sim_models' iris_a8mini: "world" -> "base_link"), and
  * if the odometry frame is NOT already the fixed frame (e.g. a model using the OdometryPublisher
    default "iris/odom"), publishes once a static, identity TF <map_frame> -> <odometry frame>, because
    the Gazebo odometry frame IS the world frame. RViz uses "world" as its fixed frame, the same frame
    sim_demo's markers (search area, targets, photo footprints) are drawn in.

Topics:   in  /sim/drone/odometry (nav_msgs/Odometry)       out  /tf, /tf_static
Parameters (all optional):
  odom_topic         default /sim/drone/odometry
  map_frame          default world  (fixed frame of the RViz layout and the demo markers)
  odom_frame         default ''     ('' = use the odometry message's header.frame_id)
  base_frame         default ''     ('' = use the odometry message's child_frame_id)
If no odometry arrives (e.g. the stand-in drone has no OdometryPublisher) it warns every 15 s and waits.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class OdomTf(Node):
    """Odometry -> TF converter node (see the module docstring)."""

    def __init__(self):
        super().__init__('odom_tf')
        self.odom_topic = self.declare_parameter('odom_topic', '/sim/drone/odometry').value
        self.map_frame = self.declare_parameter('map_frame', 'world').value
        self.odom_frame = self.declare_parameter('odom_frame', '').value
        self.base_frame = self.declare_parameter('base_frame', '').value
        self.tf_pub = TransformBroadcaster(self)
        self.static_pub = StaticTransformBroadcaster(self)
        self.static_sent_for = None          # odometry frame the static map->odom TF was sent for
        self.got_message = False
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, qos_profile_sensor_data)
        self.create_timer(15.0, self.check_alive)
        self.get_logger().info(f'publishing TF from {self.odom_topic} (fixed frame "{self.map_frame}")')

    def on_odom(self, msg: Odometry) -> None:
        """Turn one odometry message into a TF (and the one-time static map -> odom TF)."""
        parent = self.odom_frame or msg.header.frame_id or 'odom'
        child = self.base_frame or msg.child_frame_id or 'base_link'
        if self.static_sent_for != parent and parent != self.map_frame:
            st = TransformStamped()
            st.header.stamp = self.get_clock().now().to_msg()
            st.header.frame_id = self.map_frame
            st.child_frame_id = parent
            st.transform.rotation.w = 1.0     # identity: the Gazebo odometry frame is the world frame
            self.static_pub.sendTransform(st)
            self.static_sent_for = parent
            self.get_logger().info(f'static TF {self.map_frame} -> {parent} (identity); drone frame "{child}"')
        t = TransformStamped()
        # Stamp with our own clock: the team nodes and RViz run on wall time (use_sim_time is false).
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = q
        self.tf_pub.sendTransform(t)
        self.got_message = True

    def check_alive(self) -> None:
        """Warn (once per 15 s) while no odometry has arrived."""
        if not self.got_message:
            self.get_logger().warn(f'no odometry on {self.odom_topic} yet (is the bridge running and the drone spawned?)')


def main(args=None) -> None:
    """Run the node until Ctrl+C."""
    rclpy.init(args=args)
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
