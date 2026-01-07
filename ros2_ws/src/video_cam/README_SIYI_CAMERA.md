# SIYI A8 mini Camera Integration for ROS2

## Overview
This ROS2 node enables photo capture from the SIYI A8 mini camera using the camera's SDK and HTTP media server. The implementation follows the SIYI A8 mini User Manual v1.6 and v1.8 specifications.

## Hardware Requirements
- SIYI A8 mini camera with microSD (TF) card inserted
- Jetson Orin Nano (or compatible system)
- Ethernet connection between camera and Jetson
- SIYI "Gimbal Ethernet to RJ45" harness cable

## Network Configuration
The camera uses the following default network settings:
- **Camera IP**: `192.168.144.25`
- **Control Port (UDP)**: `37260` (for SDK commands)
- **Media Server Port (HTTP)**: `82` (for photo downloads)

Ensure your Jetson's Ethernet interface is configured to be on the same network subnet (e.g., `192.168.144.x`).

## How It Works

### Photo Capture Workflow
1. **Trigger Photo**: Jetson sends SDK command (UDP packet) to camera
2. **Save to SD**: Camera writes 4K JPEG to microSD card
3. **Wait for Photo**: Jetson polls camera's HTTP media server
4. **Download**: Jetson downloads JPEG over Ethernet
5. **Process**: Photo is saved locally and published as ROS2 Image message

### SDK Command
The node sends a "Take Picture" command using the SIYI SDK protocol:
```python
TAKE_PIC_PKT = bytes.fromhex("55 66 01 01 00 00 00 0c 00 34 ce")
```
This is CMD_ID `0x0C` with `func_type=0` (Take a picture).

### Media Server API
The camera exposes an HTTP interface at `http://192.168.144.25:82/cgi-bin/media.cgi` with the following commands:
- `getdirectories`: List photo/video folders on SD card
- `getmediacount`: Count files in a folder
- `getmedialist`: List files with download URLs

## Installation

### 1. Install Dependencies
```bash
cd /path/to/ros2_ws/src/video_cam
pip install -r requirements.txt
```

Required Python packages:
- `requests>=2.28.0` (HTTP client for media server)
- `opencv-python>=4.8.0` (Image processing)
- `numpy>=1.24.0` (Array operations)

### 2. Build the Package
```bash
cd /path/to/ros2_ws
colcon build --packages-select video_cam
```

### 3. Source the Workspace
```bash
source install/setup.bash
```

## Usage

### Run the Node
```bash
ros2 run video_cam image_pub
```

### What to Expect

#### With SIYI Camera Connected
```
[INFO] [siyi_a8_publisher]: SIYI A8 mini camera initialized successfully
[INFO] [siyi_a8_publisher]: Current photo directory: /DCIM/2026_01_07
[INFO] [siyi_a8_publisher]: Initial photo count: 5
[INFO] [siyi_a8_publisher]: Capturing photo from SIYI camera...
[INFO] [siyi_a8_publisher]: Photo trigger sent to camera
[INFO] [siyi_a8_publisher]: New photo available: http://192.168.144.25/.../photo_123.jpg
[INFO] [siyi_a8_publisher]: Photo downloaded successfully, size: (3840, 2160, 3)
[INFO] [siyi_a8_publisher]: Photo saved: .../camera_feed/photo_20260107-120530.jpg
[INFO] [siyi_a8_publisher]: Image published to /image_raw
```

#### Without Camera (Simulation Mode)
```
[ERROR] [siyi_a8_publisher]: Camera connection error: Connection to 192.168.144.25 timed out
[WARN] [siyi_a8_publisher]: Camera not available. Will use simulation mode
[INFO] [siyi_a8_publisher]: Status: SIYI camera not available - using simulation mode
```

The node will wait for images on the `/webcam/image_raw` topic for simulation purposes.

## ROS2 Topics

### Published Topics
- `/image_raw` (sensor_msgs/Image): Captured images (either from SIYI camera or simulation)
- `/mavros/statustext/send` (mavros_msgs/StatusText): Status messages for MAVLink integration

### Subscribed Topics
- `/camera/trigger` (std_msgs/Bool): Manual trigger to capture and save a mapping photo
- `/mavros/global_position/rel_alt` (std_msgs/Float64): Altitude-based camera enable/disable
- `/webcam/image_raw` (sensor_msgs/Image): Simulation images (fallback mode)

## Configuration

### Photo Storage
Photos are saved to two directories:
- **Regular captures**: `ros2_ws/video_cam/camera_feed/`
- **Mapping captures**: `ros2_ws/video_cam/mapping_photos/`

### Capture Interval
The node captures photos every **5 seconds** by default. Modify this in the code:
```python
self.timer = self.create_timer(5.0, self.camera_loop)  # Change 5.0 to desired seconds
```

### Altitude Threshold
Camera is enabled when altitude exceeds **13.716 meters** (45 feet). Modify:
```python
self.ALT_THRESHOLD = 13.716  # Change to desired altitude in meters
```

## Troubleshooting

### Camera Not Found
1. **Check Network Connection**:
   ```bash
   ping 192.168.144.25
   ```
   If ping fails, check Ethernet cable and network configuration.

2. **Check Camera IP**:
   Some cameras may use different IPs. Check camera settings or documentation.

3. **Verify SD Card**:
   The camera MUST have a microSD card inserted. Without it, photo capture will fail.

### Photo Download Timeout
If photos take too long to download, increase the timeout:
```python
def wait_for_new_photo(self, timeout_s=10):  # Increase from 10 to 15 or 20
```

### Network Configuration
To configure Jetson's Ethernet interface:
```bash
# Check current network interfaces
ip addr show

# Configure static IP (example for eth0)
sudo ip addr add 192.168.144.100/24 dev eth0
sudo ip link set eth0 up
```

Or edit `/etc/netplan/` configuration for persistent settings.

## Waypoint-Based Capture

### Option A: Jetson Triggers Photos (Recommended)
The node can be extended to listen for MAVLink waypoint events:
```python
from pymavlink import mavutil

m = mavutil.mavlink_connection("udp:0.0.0.0:14550")
m.wait_heartbeat()

# Listen for waypoint reached events
while True:
    msg = m.recv_match(type=["MISSION_ITEM_REACHED"], blocking=True)
    # Trigger photo capture
    self.capture_photo = True
```

### Option B: ArduPilot Triggers via UART
Connect the camera's UART to Pixhawk for ArduPilot to control camera functions directly. In this mode, Jetson only downloads and processes photos after they're captured.

## Performance Notes

### Photo Resolution
The SIYI A8 mini captures photos up to 4K (3840x2160). Download time over Ethernet depends on:
- Network bandwidth (~100 Mbps typical)
- JPEG compression (typically 2-5 MB per photo)
- Expected download time: **1-3 seconds per photo**

### Capture Rate Limitations
- **Camera capture time**: ~1-2 seconds
- **Download time**: ~1-3 seconds
- **Processing time**: <0.5 seconds
- **Practical maximum rate**: ~1 photo every 5 seconds

## Integration with Detection Pipeline

To integrate with object detection:
```bash
# Terminal 1: Run camera node
ros2 run video_cam image_pub

# Terminal 2: Run detection node
ros2 run detection object_detection_node

# Detection node will subscribe to /image_raw topic
```

## References
- SIYI A8 mini User Manual v1.6 (SDK and Media Server documentation)
- SIYI A8 mini User Manual v1.8 (UART and ArduPilot integration)
- Documentation located in: `ros2_ws/src/video_cam/siyi_documentations/`

## Support
For issues or questions, check:
1. Camera documentation in `siyi_documentations/` folder
2. ROS2 logs: `ros2_ws/log/` directory
3. Network connectivity: `ping 192.168.144.25`
