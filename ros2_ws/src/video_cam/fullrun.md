# SIYI Unified Pipeline - Quick Start Guide

This guide shows how to capture images from the SIYI A8 Mini camera and automatically process them for object detection.

## Quick Start (2 Terminals)

### Terminal 1: Launch Camera Pipeline
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build 
source install/setup.bash
ros2 run video_cam siyi
```

### Terminal 2: Trigger Image Capture
```bash
# Wait for Terminal 1 to show "Pipeline ready. Waiting for triggers..."
# Then trigger a capture:
ros2 topic pub /camera/trigger std_msgs/msg/Bool "data: true" --once
```

## What Happens (4-Phase Pipeline)

When you trigger a capture, the unified pipeline executes:

1. **Phase 1 - Capture Control:** Sends command to camera to capture 4K image to SD card
2. **Phase 2 - SD Indexing:** Polls camera's SD card until new image appears
3. **Phase 3 - Download:** Downloads image from SD card with verification
4. **Phase 4 - Publish:** Publishes image to ROS `/image_raw` topic

**Total Time:** 3-5 seconds per capture


## Using the Convenience Script

Alternatively, use the provided script for easier operation:

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam

# Test camera connectivity
./siyi_unified_pipeline_start.sh test

# Launch pipeline
./siyi_unified_pipeline_start.sh launch

# Trigger capture (in another terminal)
./siyi_unified_pipeline_start.sh trigger

# Check status
./siyi_unified_pipeline_start.sh status
```

## Changing Resolution

```bash
# Set to 4K (default)
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# not set for now 
# # Set to 2.7K
# ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# # Set to 1080P
# ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"
```

## Monitoring Status

```bash
# Watch camera status
ros2 topic echo /camera/status

# Watch live image stream
ros2 topic hz /image_raw

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

#  # all the gimbal and lazer commands are not working (for now)
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_rotate 30, -20'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_stop'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_center'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_attitude'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_angles 15,-25'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_set_axis yaw,10'" 
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_mode_get'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'gimbal_mode_set lock'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'laser_distance'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'laser_target'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'laser_state_get'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'laser_state_set on'"
# ros2 topic pub --once /camera/command std_msgs/msg/String "data: 'laser_stream enable,4'"

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

# Launch detection
ros2 run detection object_detection_sahi