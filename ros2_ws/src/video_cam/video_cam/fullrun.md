# SIYI Unified Pipeline - Quick Start Guide

This guide shows how to capture images from the SIYI A8 Mini camera and automatically process them for object detection.

## Quick Start (2 Terminals)

### Terminal 1: Launch Camera Pipeline
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build 
source install/setup.bash
ros2 run video_cam siyi_unified_pipeline
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

## Where Images Are Saved

Images are automatically saved to THREE locations:

```
ros2_ws/src/video_cam/
├── downloaded_images/          # Original SD card filename (DSCF0001.JPG)
├── camera_feed/               # Timestamped for processing
└── mapping_photos/            # Timestamped for mapping + detection
```

**Example filenames:**
- `downloaded_images/DSCF0001.JPG`
- `camera_feed/photo_20260116-143022_DSCF0001.JPG`
- `mapping_photos/mapping_photo_20260116-143022_DSCF0001.JPG`

## Optional: Automatic Object Detection

### Terminal 3: Launch Detection (Optional)
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash
ros2 run detection object_detection_sahi
```

The detection node automatically:
- Monitors `mapping_photos/` directory
- Detects new images every 2 seconds
- Processes with SAHI + YOLO26
- Saves results to `src/detection/detection_results_sahi/`

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

# Set to 2.7K
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '2.7K'"

# Set to 1080P
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '1080P'"
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

**Quick Command Reference:**
```bash
# Launch camera pipeline
ros2 run video_cam siyi_unified_pipeline

# Trigger capture
ros2 topic pub /camera/trigger std_msgs/msg/Bool "data: true" --once

# Launch detection
ros2 run detection object_detection_sahi
```