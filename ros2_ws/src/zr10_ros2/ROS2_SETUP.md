# Project ASTRA: ZR10 ROS 2 Humble wrapper

This package wraps the **uploaded** UDP driver unchanged except for include paths.
It is **not hardware-verified** and has **not been built against a ROS 2 Humble installation** here.

## Requirements
- Ubuntu 22.04 with ROS 2 Humble installed and sourced; check the Jetson's actual OS / ROS distro.
- `sudo apt install python3-colcon-common-extensions ros-humble-rclcpp ros-humble-std-srvs ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs libcurl4-openssl-dev nlohmann-json3-dev`
- Verify camera-facing Jetson interface can reach `192.168.144.25` without changing the default route.

## Install/build
```bash
mkdir -p ~/astra_ws/src
# Copy this entire zr10_ros2 folder into ~/astra_ws/src/zr10_ros2
cd ~/astra_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select zr10_ros2
source install/setup.bash
ros2 run zr10_ros2 zr10_node --ros-args --params-file ~/astra_ws/src/zr10_ros2/config/zr10.yaml
```

## Separate terminal (source environment in each new terminal)
```bash
source /opt/ros/humble/setup.bash
source ~/astra_ws/install/setup.bash
ros2 service list
ros2 service call /take_photo std_srvs/srv/Trigger '{}'
ros2 service call /autofocus std_srvs/srv/Trigger '{}'
ros2 service call /center_gimbal std_srvs/srv/Trigger '{}'
ros2 service call /toggle_recording std_srvs/srv/Trigger '{}'
ros2 service call /stop_zoom std_srvs/srv/Trigger '{}'
ros2 service call /stop_gimbal std_srvs/srv/Trigger '{}'
ros2 topic pub --once /zoom_direction std_msgs/msg/Int8 '{data: 1}'
ros2 topic pub --once /zoom_direction std_msgs/msg/Int8 '{data: 0}'
ros2 topic pub --once /gimbal_speed geometry_msgs/msg/Vector3 '{x: 20.0, y: 0.0, z: 0.0}'
ros2 topic pub --once /gimbal_speed geometry_msgs/msg/Vector3 '{x: 0.0, y: 0.0, z: 0.0}'
```

## Critical behavior
- `Trigger.success=true` means **UDP send succeeded**, not that camera operation succeeded. No ACK receive loop exists in uploaded `zr10.cpp`.
- `toggle_recording` is a toggle, NOT an idempotent start/stop command. Do not call it blindly.
- Motion commands are **speed** commands. The 0.5-second watchdog stops them unless refreshed. For continuous motion, publish at e.g. 10 Hz; stop explicitly when done.
- Camera driver does not implement `setZoom`, absolute pan/tilt, manual focus, or feedback. Do not advertise these as implemented.
- Photo sync is disabled by default. `media.cpp` only downloads JPEGs, not recorded video; verify the camera HTTP API and destination first.
- The uploaded README/PROTOCAL.md document features not present in uploaded source. Reconcile protocol version and autofocus payload with actual firmware before flight.
- No ROS 2 image publisher or RTSP relay in this initial package. Old `main.cpp` launched FFmpeg independently; it has not been ported.
- Use an independent hardware/failsafe stop mechanism: the watchdog cannot stop motion if the Jetson process, network, or power fails.
- Replace placeholder package maintainer email and license before sharing/releasing.
