# SIYI Terminal Control Guide (Unified Node)

This guide uses one runtime node only:

```bash
ros2 run video_cam siyi
```

That node is `siyi_unified_pipeline` and now exposes:
- Existing topics (like `/camera/trigger`)
- Unified service API: `/camera/command` (`interfaces/srv/CameraCommand`)

## 1) Build

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select interfaces video_cam
```

## 2) Start the single SIYI node

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash
ros2 run video_cam siyi
```

## 3) Existing capture topic (unchanged)

```bash
ros2 topic pub /camera/trigger std_msgs/msg/Bool "data: true" --once
```

## 4) Unified service call format

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'COMMAND_NAME', parameter: 'VALUE'}"
```

## 5) Service commands (one by one)

### Capture

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'capture', parameter: '4K'}"
```

### Autofocus

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'autofocus', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'autofocus', parameter: '640,360'}"
```

### Zoom and focus

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_manual', parameter: 'in'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_manual', parameter: 'out'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_manual', parameter: 'stop'}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_absolute', parameter: '4.5'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_auto', parameter: '8.0'}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_range', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'zoom_current', parameter: ''}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'focus_manual', parameter: 'far'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'focus_manual', parameter: 'near'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'focus_manual', parameter: 'stop'}"
```

### Gimbal controls

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_rotate', parameter: '30,-20'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_stop', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_center', parameter: ''}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_attitude', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_set_angles', parameter: '15,-25'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_set_axis', parameter: 'yaw,10'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_set_axis', parameter: 'pitch,-30'}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_mode_get', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_mode_set', parameter: 'lock'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_mode_set', parameter: 'follow'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'gimbal_mode_set', parameter: 'fpv'}"
```

### Laser controls

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_distance', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_target', parameter: ''}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_state_get', parameter: ''}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_state_set', parameter: 'on'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_state_set', parameter: 'off'}"

ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_stream', parameter: 'enable,4'}"
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'laser_stream', parameter: 'disable'}"
```

### SD card format

```bash
ros2 service call /camera/command interfaces/srv/CameraCommand "{command: 'sd_format', parameter: 'yes'}"
```

## Notes

- The service response `message` is JSON with detailed result fields.
- `sd_format` is destructive and requires `parameter: 'yes'`.
- If capture pipeline is active, some direct control commands can return a busy error to avoid SDK socket conflicts.
