# Astra Package - Capture & Detect Node

Combined node that merges camera capture and object detection functionality.

## Overview

The `astra` package contains `capture_detect.py`, which combines:
- **siyiUnifiedWorking.py**: SIYI A8 Mini camera control, image capture, SD card management
- **new_od.py**: SAHI+YOLO object detection

This single node handles the complete pipeline from image capture to detection.

## Features

✅ **Camera Control**
- RTSP video streaming
- Triggered 4K image capture
- SD card management
- Altitude-based enable/disable

✅ **Storage Management**
- Saves to multiple directories: `downloaded_images`, `camera_feed`, `mapping_photos`
- Automatic detection of new images
- Queue-based processing

✅ **Object Detection**
- SAHI sliced inference for small objects
- YOLO26 model support (confidence threshold: 25%)
- Configurable slice size (640x640) and overlap (25%)
- Multi-threaded detection processing

✅ **ROS2 Integration**
- Publishes raw images (`/image_raw`)
- Publishes detection results (`/detections`, `/image_detection`)
- Subscribes to capture triggers (`/camera/trigger`)
- Status updates to MAVROS

## Installation

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select astra
source install/setup.bash
```

## Usage

### Basic Launch

```bash
# Run the capture and detect node
ros2 run astra capture_detect
```

### With Parameters

```bash
ros2 run astra capture_detect --ros-args \
  -p model_path:=yolo26x.pt \
  -p confidence_threshold:=0.25 \
  -p slice_height:=640 \
  -p slice_width:=640 \
  -p overlap_height_ratio:=0.25 \
  -p device:=cuda:0
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `model_path` | string | `yolo26x.pt` | Path to YOLO model file |
| `confidence_threshold` | float | `0.25` | Minimum confidence (25% filters false positives) |
| `slice_height` | int | `640` | SAHI slice height in pixels |
| `slice_width` | int | `640` | SAHI slice width in pixels |
| `overlap_height_ratio` | float | `0.25` | Vertical overlap ratio for SAHI |
| `overlap_width_ratio` | float | `0.25` | Horizontal overlap ratio for SAHI |
| `detection_check_interval` | float | `2.0` | Seconds between detection checks |
| `device` | string | `auto` | Device for inference (`cpu`, `cuda:0`, `auto`) |
| `stream_rate` | float | `10.0` | Hz for video streaming |

## Topics

### Published Topics

- `/image_raw` (sensor_msgs/Image) - Raw camera images
- `/detections` (vision_msgs/Detection2DArray) - Object detections
- `/image_detection` (interfaces/ImageResult) - Detection summary
- `/camera/status` (std_msgs/String) - Camera status updates
- `/mavros/statustext/send` (mavros_msgs/StatusText) - Status messages

### Subscribed Topics

- `/camera/trigger` (std_msgs/Bool) - Trigger image capture
- `/camera/set_resolution` (std_msgs/String) - Set resolution (4K, 2.7K, 1080P)
- `/mavros/global_position/rel_alt` (std_msgs/Float64) - Altitude for auto-enable

## Workflow

1. **Initialization**
   - Connects to SIYI camera via RTSP (192.168.144.25)
   - Loads YOLO detection model
   - Starts worker thread for detection
   - Creates storage directories

2. **Streaming Mode** (Default)
   - Publishes live video stream at 10 Hz
   - Monitors for capture triggers

3. **Capture Pipeline** (On trigger)
   - Phase 1: Send capture command to camera via UDP
   - Phase 2: Wait for image to appear on SD card
   - Phase 3: Download from SD to Jetson via HTTP
   - Phase 4: Save to local directories (3 locations)
   - Phase 5: Publish to ROS topics
   - Phase 6: Queue for object detection

4. **Detection Pipeline** (Automatic)
   - Worker thread processes queue asynchronously
   - Runs SAHI sliced inference
   - Annotates images with bounding boxes
     - TENT labels: top-left corner (orange)
     - PERSON labels: top-right corner (green)
   - Saves detection results
   - Publishes to ROS topics

## Directory Structure

```
ros2_ws/
├── video_cam/
│   ├── downloaded_images/    # Archived captures
│   ├── camera_feed/          # Images for detection
│   └── mapping_photos/       # Images for mapping
└── src/detection/
    └── detection_results_sahi/  # Annotated detection results
```

## Detection Configuration

The node uses improved detection settings:
- **Confidence: 25%** - Filters out poor detections (previously 5%)
- **Overlap: 25%** - Better boundary detection (previously 15%)
- **Slice size: 640x640** - Optimal for small object detection

These settings significantly reduce false positives while maintaining good detection accuracy.

## Examples

### Trigger Capture
```bash
ros2 topic pub --once /camera/trigger std_msgs/Bool "data: true"
```

### Monitor Detection Results
```bash
ros2 topic echo /image_detection
```

### Check Node Status
```bash
ros2 node info /capture_detect_node
```

### Change Resolution
```bash
ros2 topic pub --once /camera/set_resolution std_msgs/String "data: '4K'"
```

### View Live Stream
```bash
ros2 run rqt_image_view rqt_image_view /image_raw
```

## Performance

- **Capture latency**: ~1-2 seconds (camera → Jetson)
- **Detection time**: 
  - CPU: ~30-60 seconds per 4K image
  - GPU (CUDA): ~5-10 seconds per 4K image
- **Throughput**: Asynchronous - capture continues while detecting

## Troubleshooting

### Camera Not Connecting
```
✗ Failed to connect to camera video stream
```
**Solution**: 
- Check network: `ping 192.168.144.25`
- Verify camera is powered on
- Check network interface is on 192.168.144.x subnet

### Model Not Loading
```
Failed to load detection model
```
**Solution**: 
- Verify model file: `ls ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/yolo26x.pt`
- Install SAHI: `pip install sahi`
- Install Ultralytics: `pip install ultralytics`

### Queue Full Warning
```
Detection queue full, skipping: IMG_XXX.jpg
```
**Solution**: Normal behavior - detection is slower than capture rate. Images are processed as capacity allows.

### Low Detection Quality
**Solutions**:
- Current settings (25% confidence, 25% overlap) are already optimized
- To catch more objects: decrease `confidence_threshold` (not recommended)
- To filter more: increase `confidence_threshold` to 0.30 or higher

## Comparison with Separate Nodes

### Before (Two Separate Nodes)
```bash
# Terminal 1: Camera capture
ros2 run video_cam siyiUnifiedWorking

# Terminal 2: Object detection
ros2 run detection new_od
```

### After (Single Combined Node)
```bash
# Single terminal: Both capture and detection
ros2 run astra capture_detect
```

**Advantages**:
- ✅ Single process - easier to manage
- ✅ Guaranteed synchronization
- ✅ Shared state and resources
- ✅ Lower overhead

**Trade-offs**:
- ⚠️ If detection crashes, capture also stops
- ⚠️ Less flexibility for independent configuration

## Dependencies

- ROS2 Humble
- Python 3.8+
- OpenCV with FFmpeg/GStreamer support
- SAHI: `pip install sahi`
- Ultralytics YOLO: `pip install ultralytics`
- PyTorch (for GPU acceleration): `pip install torch torchvision`
- cv_bridge (ROS package)
- requests: `pip install requests`

## Node Lifecycle

The node can be shut down gracefully using:
```bash
# Ctrl+C works but is not ideal
# Better: Use ROS2 lifecycle commands if implementing lifecycle management
```

## Notes

- Saves images to same locations as `siyiUnifiedWorking` for compatibility
- Detection runs asynchronously - capture is not blocked
- Images are automatically queued for detection after capture
- Original `siyiUnifiedWorking.py` and `new_od.py` remain unmodified
- Label positioning prevents overlaps (tents left, persons right)

## Package Location

```
ros2_ws/src/astra/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── astra
├── astra/
│   ├── __init__.py
│   └── capture_detect.py
└── README.md
```

## Author

Created by merging functionality from:
- `siyiUnifiedWorking.py` (Camera capture)
- `new_od.py` (Object detection with improvements)

January 2026
