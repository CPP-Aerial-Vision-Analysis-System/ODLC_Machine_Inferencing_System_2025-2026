# Safe first build and dry-run tests

This is an **unverified** development revision. It has not been built against ROS 2 Humble or tested on Jetson hardware.

The `dry_run` parameter defaults to `true`; service calls and topic callbacks do not send camera control packets, and media sync is disabled. The ZR10 constructor still creates a UDP socket. The mode is not a substitute for an independent hardware safety interlock.

## Build on Ubuntu 22.04 with ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
cd ~/astra_ws
colcon build --packages-select zr10_ros2 --symlink-install
source install/setup.bash
ros2 run zr10_ros2 zr10_node --ros-args --params-file ~/astra_ws/src/zr10_ros2/config/zr10.yaml
```

In a second terminal (source the same ROS environment and workspace):

```bash
ros2 service list
ros2 service call /take_photo std_srvs/srv/Trigger '{}'
ros2 topic pub --once /zoom_direction std_msgs/msg/Int8 '{data: 1}'
ros2 topic pub --once /gimbal_speed geometry_msgs/msg/Vector3 '{x: 10.0, y: 0.0, z: 0.0}'
```

Service replies should state `DRY RUN: no camera command sent`. Topic callbacks still update internal motion state and exercise the watchdog. To observe it, inspect node logs.

Do not set `dry_run: false` until the camera is safely secured, propellers are disabled, the latest driver is reconciled, and a real-device test plan is approved. A successful UDP send does not prove camera acknowledgment or movement stop.
