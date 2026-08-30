# SIYI Unified Pipeline - Quick Start Guide

This guide shows how to capture images from the SIYI A8 Mini camera and automatically process them for object detection.

## Quick Start (2 Terminals)

### Terminal 1: Launch Camera Pipeline
```bash
ros2 run video_cam siyi
```

### Terminal 2: Trigger Image Capture
```bash
ros2 topic pub /camera/trigger std_msgs/msg/Bool "data: true" --once
```

## Changing Resolution

```bash
# Set to 4K (default)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"
```

## Monitoring Status

```bash
# Watch camera status
ros2 topic echo /camera/status

# Check node is running
ros2 node list | grep siyi
```

ros2 topic echo /rosout | grep -i "siyi\|phase\|pipeline\|capture\|download\|index"

ros2 topic echo /mavros/statustext/send

## Use these exactly (all currently supported command names in /camera/command):
```bash

ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'capture 4K'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'autofocus'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_manual in'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_absolute 4.5'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_auto 8.0'" # this is what i use max is 30.0
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_range'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_current'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'focus_manual far'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'sd_format yes'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_absolute 1.0'"

# What DO_DIGICAM_CONTROL sends: rack to config.py CAPTURE_ZOOM_X, then focus
# per FOCUS_MODE. Idempotent -- a lens already at that zoom is left alone.
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'capture_setup'"
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'zoom_capture'"  # rack only, no focus

```

### Useful valid parameter alternatives:
```bash

autofocus supports "640,360" (x,y touch point).
zoom_manual supports in, out, stop.
focus_manual supports far, near, stop.
gimbal_set_axis supports yaw,angle or pitch,angle.
gimbal_mode_set supports lock, follow, fpv.
laser_state_set supports on/off (also true/false, 1/0, enable/disable).
laser_stream supports "", "disable", "4", or "enable,4".
capture supports "" (defaults to 4K), or 4K / 2.7K / 1080P.
capture_setup / zoom_capture take no parameter; the zoom comes from CAPTURE_ZOOM_X.
```
Gimbal Control

All gimbal commands are sent to /camera/command as a String topic. The node must be running (ros2 run video_cam siyi) before sending any command.
Build first (run once after any code change)

cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash

Terminal 1: Start the node

ros2 run video_cam siyi

Terminal 2: Send gimbal commands

Continuous rotation — moves at a speed from -100 to 100 (positive yaw = right, positive pitch = up):

ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_rotate 30, -20'"

Stop rotation:

ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_stop'"

Center gimbal (returns to forward-level position):

ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_center'"

Set absolute angles — yaw range ±135°, pitch range -90° to +25°:

# Example: yaw 45° right, pitch 30° down
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_angles 45, -30'"

# Straight down
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_angles 0, -90'"

# Return to level forward
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_angles 0, 0'"

Set a single axis — move only yaw or only pitch:

# Yaw only
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_axis yaw, 90'"

# Pitch only
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_axis pitch, -45'"

Set gimbal mode:

# Lock mode — gimbal holds its absolute position regardless of aircraft movement
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_mode_set lock'"

# Follow mode — gimbal yaw follows the aircraft heading
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_mode_set follow'"

# FPV mode — gimbal matches aircraft pitch and roll
ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_mode_set fpv'"

Command reference table
Command 	Arguments 	Range 	Description
gimbal_rotate 	yaw_speed, pitch_speed 	-100 to 100 	Continuous rotation at speed
gimbal_stop 	none 	— 	Stop all rotation
gimbal_center 	mode (optional, default 1) 	1, 2, 4 	Return to center
gimbal_set_angles 	yaw_deg, pitch_deg 	yaw ±135°, pitch -90°..+25° 	Absolute angle
gimbal_set_axis 	yaw|pitch, angle_deg 	same as above 	Single axis
gimbal_mode_set 	lock|follow|fpv 	— 	Set stabilisation mode
Monitoring Status

# Watch camera status
ros2 topic echo /camera/status

# Watch live image stream
ros2 topic hz /image_raw

# Check node is running
ros2 node list | grep siyi

Quick Command Reference:

# Launch camera pipeline
ros2 run video_cam siyi

# Trigger capture
ros2 topic pub /camera/trigger std_msgs/msg/Bool "data: true" --once

