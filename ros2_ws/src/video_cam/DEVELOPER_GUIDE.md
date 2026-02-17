# Quick Developer Reference - Refactored SIYI Pipeline

## Architecture at a Glance

```
┌─────────────────────────────────────────────────────────────────┐
│                         SIYINode                                │
│                    (ROS2 Integration)                           │
│  - Handles ROS topics, parameters, callbacks                    │
│  - Altitude-based camera enable/disable                         │
│  - Video stream publishing                                      │
└─────────────────┬───────────────────┬───────────────────────────┘
                  │                   │
                  ▼                   ▼
    ┌─────────────────────┐  ┌──────────────────────┐
    │ PipelineOrchestrator│  │   StorageManager     │
    │  (State Machine)    │  │  (File Operations)   │
    │                     │  │                      │
    │ - 4-phase pipeline  │  │ - Atomic writes      │
    │ - Error recovery    │  │ - Verification       │
    │ - SD polling        │  │ - Tracking state     │
    └──────────┬──────────┘  └──────────────────────┘
               │
               ▼
    ┌──────────────────────┐
    │   CameraInterface    │
    │ (Hardware Protocol)  │
    │                      │
    │ - SDK/UDP control    │
    │ - HTTP API           │
    │ - RTSP stream        │
    └──────────────────────┘
```

## Component Responsibilities

### config.py
**What:** All constants and configuration  
**When to edit:** Changing timeouts, thresholds, or hardware addresses  
**Example:**
```python
CAPTURE_TIMEOUT_SECONDS = 15.0  # Increase if camera is slow
MIN_FREE_SPACE_MB = 50          # Increase for safety margin
```

### camera_interface.py
**What:** Camera communication only  
**When to edit:** Changing camera protocol, adding new commands  
**Does NOT:** Handle ROS, files, or state machine  
**Key Pattern:**
```python
camera = CameraInterface("192.168.144.25")
camera.send_capture_command('4K')
files = camera.get_media_list(directory)
image_bytes = camera.download_image(url)
img = camera.decode_image(image_bytes)
camera.close()
```

### storage_manager.py
**What:** File I/O and persistence  
**When to edit:** Changing file structure, verification logic  
**Does NOT:** Handle ROS or camera communication  
**Key Pattern:**
```python
storage = StorageManager(workspace_root)
storage.check_disk_space(required_mb=100)
filepath = storage.save_image('test.jpg', img, '4K')
is_valid = storage.verify_file(filepath, '4K')
state = storage.load_tracking_state()
```

### pipeline_orchestrator.py
**What:** State machine and pipeline coordination  
**When to edit:** Changing pipeline phases, error handling logic  
**Does NOT:** Handle ROS topics or implement camera/storage operations  
**Key Pattern:**
```python
pipeline = PipelineOrchestrator(camera, storage)
pipeline.initialize_sd_card()
pipeline.set_resolution('4K')

if not pipeline.is_busy():
    success = pipeline.execute_pipeline()
    stats = pipeline.get_stats()
```

### siyi_node_refactored.py
**What:** ROS2 wrapper - topics, parameters, callbacks  
**When to edit:** Adding new ROS topics, changing ROS behavior  
**Does NOT:** Implement camera protocol, file I/O, or pipeline logic  
**Key Pattern:**
```python
node = SIYINode()  # Creates camera, storage, pipeline
# Components automatically initialized
# ROS topics automatically set up
rclpy.spin(node)
```

## Common Tasks

### Adding a New Resolution

1. **Update config.py:**
```python
PHOTO_RESOLUTIONS = {
    '4K': 0x00,
    '2.7K': 0x01,
    '1080P': 0x02,
    '720P': 0x03,  # NEW
}

CAPTURE_COMMANDS = {
    # ... existing ...
    '720P': bytes.fromhex("55 66 01 01 00 00 00 0c 03 37 ce"),  # NEW
}

RESOLUTION_SPECS = {
    # ... existing ...
    '720P': {  # NEW
        'min_width': 1200,
        'min_height': 600,
        'min_file_size': 15000,
    }
}
```

2. **That's it!** The rest of the code automatically supports it.

### Changing Polling Behavior

**Edit `config.py` only:**
```python
SD_POLL_INTERVAL_INITIAL = 0.5  # Start polling every 0.5s
SD_POLL_INTERVAL_MAX = 2.0      # Max backoff to 2s
```

### Adding Camera Status Logging

**Edit `camera_interface.py`:**
```python
def send_capture_command(self, resolution: str = '4K') -> bool:
    # ... existing code ...
    self._log('info', f"Sending capture command for {resolution}")
    # ... rest of method ...
```

### Adding Disk Space Alert

**Edit `storage_manager.py`:**
```python
def check_disk_space(self, required_mb: float = MIN_FREE_SPACE_MB) -> bool:
    # ... existing code ...
    if free_mb < required_mb * 1.5:  # 150% threshold
        self._log('warn', f"Disk space low: {free_mb:.1f}MB")
    # ... rest of method ...
```

### Adding Pipeline Phase

**Edit `pipeline_orchestrator.py`:**
```python
def execute_pipeline(self) -> bool:
    # ... existing phases ...
    
    # Phase 5: NEW - Post-processing
    check_timeout()
    with self.state_lock:
        self.pipeline_state = CaptureState.POST_PROCESSING
    
    if not self._phase5_post_process(filename, img):
        raise PipelineError("Phase 5 failed: Post-processing error")
    
    # ... rest of method ...

def _phase5_post_process(self, filename: str, img: np.ndarray) -> bool:
    """Phase 5: Apply post-processing"""
    # Your logic here
    return True
```

### Adding ROS Topic

**Edit `siyi_node_refactored.py`:**
```python
def __init__(self):
    # ... existing code ...
    
    # Add new publisher
    self.new_pub = self.create_publisher(
        YourMsgType, '/your/topic', 10)
    
    # Add new subscriber
    self.create_subscription(
        YourMsgType, '/your/input', self.your_callback, 10)

def your_callback(self, msg: YourMsgType):
    """Handle your new topic"""
    # Your logic here
    pass
```

## Testing Individual Components

### Test Camera Interface
```python
#!/usr/bin/env python3
from video_cam.camera_interface import CameraInterface

camera = CameraInterface(camera_ip="192.168.144.25")

# Test capture
success = camera.send_capture_command('4K')
print(f"Capture success: {success}")

# Test SD card query
dirs = camera.get_directories()
print(f"Directories: {dirs}")

if dirs:
    files = camera.get_media_list(dirs[0]['path'])
    print(f"Files: {len(files)}")

camera.close()
```

### Test Storage Manager
```python
#!/usr/bin/env python3
import numpy as np
from video_cam.storage_manager import StorageManager

storage = StorageManager('/path/to/workspace')

# Test disk space
free_mb = storage.get_free_space_mb()
print(f"Free space: {free_mb:.1f}MB")

# Test save
test_img = np.zeros((100, 100, 3), dtype=np.uint8)
filepath = storage.save_image('test.jpg', test_img, '4K')
print(f"Saved: {filepath}")

# Test verify
is_valid = storage.verify_file(filepath, '4K')
print(f"Valid: {is_valid}")
```

### Test Pipeline Orchestrator
```python
#!/usr/bin/env python3
from video_cam.camera_interface import CameraInterface
from video_cam.storage_manager import StorageManager
from video_cam.pipeline_orchestrator import PipelineOrchestrator

camera = CameraInterface()
storage = StorageManager('/path/to/workspace')
pipeline = PipelineOrchestrator(camera, storage)

# Initialize
pipeline.initialize_sd_card()

# Check state
print(f"State: {pipeline.get_state()}")
print(f"Busy: {pipeline.is_busy()}")

# Execute (if not busy)
if not pipeline.is_busy():
    success = pipeline.execute_pipeline()
    stats = pipeline.get_stats()
    print(f"Success: {success}")
    print(f"Stats: {stats}")

camera.close()
```

## Debugging Tips

### Enable Debug Logging

Modify the logger parameter when creating components:

```python
import logging

# Create a custom logger
logger = logging.getLogger('SIYI')
logger.setLevel(logging.DEBUG)
handler = logging.StreamHandler()
handler.setFormatter(logging.Formatter('%(levelname)s: %(message)s'))
logger.addHandler(handler)

# Pass to components
camera = CameraInterface(logger=logger)
storage = StorageManager(workspace_root, logger=logger)
pipeline = PipelineOrchestrator(camera, storage, logger=logger)
```

### Check Pipeline State

```python
stats = pipeline.get_stats()
print(f"""
State: {stats['state']}
Photos captured: {stats['photo_count']}
Files downloaded: {stats['downloaded_files']}
Current directory: {stats['current_directory']}
Resolution: {stats['resolution']}
""")
```

### Inspect Tracking State

```bash
# View tracking state file
cat /path/to/workspace/video_cam/downloaded_images/.tracking_state.json | jq
```

## Error Handling

Each component raises specific exceptions:

```python
from video_cam.camera_interface import CameraConnectionError
from video_cam.storage_manager import StorageError
from video_cam.pipeline_orchestrator import PipelineError

try:
    pipeline.execute_pipeline()
except CameraConnectionError as e:
    print(f"Camera error: {e}")
except StorageError as e:
    print(f"Storage error: {e}")
except PipelineError as e:
    print(f"Pipeline error: {e}")
```

## Lock Hierarchy (Prevent Deadlocks)

Always acquire locks in this order:

1. `config_lock` (camera enable flag)
2. `state_lock` (pipeline state)
3. `download_lock` (tracking set)

**Never** acquire in reverse order or skip levels.

## Performance Tips

1. **Video stream:** Adjust `STREAM_RATE_HZ` in config.py
2. **SD polling:** Adjust `SD_POLL_INTERVAL_*` in config.py
3. **HTTP timeout:** Adjust `HTTP_TIMEOUT_SECONDS` in config.py
4. **Disk checks:** Reduce frequency in `_pipeline_loop()`

## Common Gotchas

1. **Port conflicts:** Only one camera instance per IP:port
2. **File permissions:** Ensure write access to workspace directory
3. **Network issues:** Camera must be on same network
4. **RTSP stream:** May timeout if network unstable
5. **SD card full:** Pipeline will fail, free space on camera

## Quick Command Reference

```bash
# Build package
cd /path/to/ros2_ws
colcon build --packages-select video_cam
source install/setup.bash

# Run refactored node
ros2 run video_cam siyi_unified_pipeline_refactored

# Run with parameters
ros2 run video_cam siyi_unified_pipeline_refactored --ros-args \
    -p use_real_camera:=true \
    -p camera_ip:=192.168.144.25 \
    -p min_altitude_agl:=10.0

# Trigger capture
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

# Set resolution
ros2 topic pub --once /camera/set_resolution std_msgs/msg/String "data: '4K'"

# Monitor topics
ros2 topic echo /image_raw
ros2 topic echo /camera/status
ros2 topic echo /camera/disk_free_mb
```

---

**Quick Start:** Just read the "Component Responsibilities" section and you're good to go!
