# Quick Start Guide - Astra Capture & Detect

## ✅ Installation Complete!

The `capture_detect` node has been successfully created in `/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/astra/`

## Running the Node

### Option 1: Using ros2 run (After sourcing)

```bash
# Open a NEW terminal (important!)
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash
ros2 run astra capture_detect
```

### Option 2: Direct execution

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash
/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/install/astra/lib/astra/capture_detect
```

## Testing the Node

### Trigger a Capture
```bash
# In another terminal:
ros2 topic pub --once /camera/trigger std_msgs/Bool "data: true"
```

### Monitor Detections
```bash
ros2 topic echo /image_detection
```

### View Statistics
```bash
ros2 topic hz /detections
```

## What Was Created

```
ros2_ws/src/astra/
├── package.xml              # ROS2 package metadata
├── setup.py                 # Python package setup
├── setup.cfg                # Setup configuration
├── README.md                # Full documentation
├── QUICKSTART.md            # This file
├── resource/
│   └── astra                # Package marker
└── astra/
    ├── __init__.py          # Package init
    └── capture_detect.py    # Main node (1100+ lines)
```

## Key Features Merged

### From siyiUnifiedWorking.py:
✅ SIYI A8 Mini camera control
✅ RTSP video streaming
✅ SD card image capture
✅ HTTP download from camera
✅ Multi-directory storage (3 locations)
✅ Altitude-based enable/disable

### From new_od.py:
✅ SAHI sliced inference
✅ YOLO26 object detection
✅ Improved thresholds (25% confidence)
✅ Smart label positioning (no overlaps)
✅ Multi-threaded processing
✅ Detection result publishing

## Configuration Applied

The node uses optimized detection settings:
- **Confidence threshold**: 25% (filters false positives)
- **Slice size**: 640x640 pixels
- **Overlap ratio**: 25%
- **Device**: Auto-detect (GPU if available, else CPU)

## Directories Created

Images are saved to:
- `video_cam/downloaded_images/` - Archived captures
- `video_cam/camera_feed/` - Images for detection
- `video_cam/mapping_photos/` - Images for mapping
- `src/detection/detection_results_sahi/` - Annotated results

## Troubleshooting

### "Package 'astra' not found"
Solution: Make sure you open a **NEW terminal** after building, then source:
```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash
ros2 run astra capture_detect
```

### Camera not connecting
Solution: Check camera network:
```bash
ping 192.168.144.25
```

### Model not found
Solution: Verify model exists:
```bash
ls ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/yolo26x.pt
```

## Rebuild if Needed

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select astra
source install/setup.bash
```

## Original Files Preserved

✅ `video_cam/video_cam/siyiUnifiedWorking.py` - NOT modified
✅ `detection/detection/new_od.py` - NOT modified

The new node is completely separate and can run alongside or replace the original nodes.

## Support

See full documentation in `README.md` for:
- Complete parameter list
- Topic descriptions
- Advanced configuration
- Performance tuning
- Dependencies

---

**Status**: ✅ Ready to use!  
**Package**: astra  
**Node**: capture_detect  
**Location**: ros2_ws/src/astra/  

To start: `ros2 run astra capture_detect`
