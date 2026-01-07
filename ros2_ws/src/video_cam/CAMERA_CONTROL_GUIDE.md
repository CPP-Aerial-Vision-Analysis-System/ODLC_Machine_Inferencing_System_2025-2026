# SIYI Camera Control System - Complete Guide

## 🎯 Overview

This system provides comprehensive control over the SIYI A8 mini camera with:
- **Automatic photo capture** (every N seconds)
- **Waypoint-based capture** (via MAVLink integration)
- **Manual control** (take photos on command)
- **Zoom control** (zoom in/out via commands)
- **Single-responsibility methods** (each method does one thing)

## 📋 What's New

### ✅ Refactored Code Structure
All methods now follow single-responsibility principle:

**Camera Initialization (split into 3 methods):**
- `test_camera_connectivity()` - Tests Ethernet connection only
- `discover_photo_directory()` - Finds SD card photo directory only
- `initialize_photo_count()` - Gets initial photo count only
- `initialize_camera()` - Orchestrates the above three methods

**Photo Capture (clear separation):**
- `take_picture()` - Sends trigger command only
- `wait_for_new_photo()` - Polls for new photos only
- `download_photo()` - Downloads image only
- `capture_and_download_photo()` - Orchestrates complete workflow

### ✅ Camera Control Service
New service `/camera/control` accepts commands:
- `take_photo` - Capture photo immediately
- `zoom_in [steps]` - Zoom in by N steps
- `zoom_out [steps]` - Zoom out by N steps
- `set_zoom <level>` - Set specific zoom level (1-30x)
- `get_status` - Get camera status

### ✅ Command-Line Interface
New executable: `camera_control`

## 🚀 Quick Start

### 1. Build the System
```bash
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws

# Build both packages
colcon build --packages-select interfaces video_cam

# Source the workspace
source install/setup.bash
```

### 2. Start the Camera Node
```bash
# Terminal 1: Start camera node
ros2 run video_cam image_pub
```

You should see:
```
[INFO] [siyi_a8_publisher]: SIYI A8 Publisher initialized
[INFO] [siyi_a8_publisher]: Camera control service available at /camera/control
```

### 3. Control the Camera
```bash
# Terminal 2: Send commands

# Take a photo immediately
ros2 run video_cam camera_control take_photo

# Zoom in by 5 steps
ros2 run video_cam camera_control zoom_in 5

# Set zoom to 10x
ros2 run video_cam camera_control set_zoom 10

# Zoom out by 3 steps
ros2 run video_cam camera_control zoom_out 3

# Check camera status
ros2 run video_cam camera_control get_status
```

## 📡 ROS2 Service Interface

### Using ros2 service call
```bash
# Take a photo
ros2 service call /camera/control interfaces/srv/CameraCommand \
  "{command: 'take_photo', parameter: ''}"

# Zoom in by 2 steps
ros2 service call /camera/control interfaces/srv/CameraCommand \
  "{command: 'zoom_in', parameter: '2'}"

# Set zoom to 15x
ros2 service call /camera/control interfaces/srv/CameraCommand \
  "{command: 'set_zoom', parameter: '15'}"

# Get status
ros2 service call /camera/control interfaces/srv/CameraCommand \
  "{command: 'get_status', parameter: ''}"
```

### Response Format
```yaml
success: true
message: "Photo captured and saved to /path/to/photo.jpg"
```

## 🔧 Code Architecture

### Method Responsibilities

#### Initialization Methods
```python
initialize_camera()              # Orchestrator - calls the three below
├── test_camera_connectivity()   # Tests Ethernet connection
├── discover_photo_directory()   # Finds SD card directory
└── initialize_photo_count()     # Gets initial count
```

#### Photo Capture Methods
```python
capture_and_download_photo()     # Orchestrator - full workflow
├── take_picture()               # Sends UDP trigger command
├── wait_for_new_photo()         # Polls media server
└── download_photo()             # Downloads via HTTP
```

#### Camera Control Methods
```python
camera_control_callback()        # Service callback - routes commands
├── take_picture() + workflow    # For take_photo command
├── zoom_in()                    # Increases zoom
├── zoom_out()                   # Decreases zoom
└── set_zoom_level()             # Sets specific zoom
```

### Why This Structure?
Each method has **one clear responsibility**, making it:
- ✅ Easier to test
- ✅ Easier to debug
- ✅ Easier to modify
- ✅ Easier to understand
- ✅ More maintainable

## 📊 Topics & Services

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | `sensor_msgs/Image` | Captured photos |
| `/camera/zoom_level` | `std_msgs/Int32` | Current zoom level |
| `/mavros/statustext/send` | `mavros_msgs/StatusText` | Status messages |

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/trigger` | `std_msgs/Bool` | Manual trigger |
| `/mavros/global_position/rel_alt` | `std_msgs/Float64` | Altitude |
| `/webcam/image_raw` | `sensor_msgs/Image` | Simulation |

### Services
| Service | Type | Description |
|---------|------|-------------|
| `/camera/control` | `interfaces/srv/CameraCommand` | Camera commands |

## 🧪 Testing

### Run Full System Test
```bash
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
./src/video_cam/test_camera_system.sh
```

This will:
1. Build packages
2. Source workspace
3. Start camera node
4. Test all services
5. Verify everything works

### Manual Testing
```bash
# Terminal 1: Start node
ros2 run video_cam image_pub

# Terminal 2: List services
ros2 service list | grep camera

# Terminal 3: Monitor topics
ros2 topic echo /camera/zoom_level

# Terminal 4: Send commands
ros2 run video_cam camera_control zoom_in 5
```

## 📸 Photo Capture Modes

### 1. Automatic (Timer-Based)
Default: Every 5 seconds
```python
# In image_pub_siyi.py
self.timer = self.create_timer(5.0, self.camera_loop)
```

### 2. Waypoint-Based
Via `/camera/trigger` topic:
```bash
ros2 topic pub /camera/trigger std_msgs/Bool "data: true" --once
```

### 3. Manual Control
Via camera control service:
```bash
ros2 run video_cam camera_control take_photo
```

### 4. Altitude-Based
Automatically enabled above 13.716m (45 feet)

## 🔍 Camera Control Examples

### Python Script Example
```python
import rclpy
from rclpy.node import Node
from interfaces.srv import CameraCommand

class MyCameraController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.client = self.create_client(CameraCommand, '/camera/control')
        
    def take_photo(self):
        request = CameraCommand.Request()
        request.command = 'take_photo'
        request.parameter = ''
        future = self.client.call_async(request)
        return future

# Use it:
controller = MyCameraController()
result = controller.take_photo()
```

### Launch File Example
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_cam',
            executable='image_pub',
            name='camera_node',
            output='screen'
        )
    ])
```

## 🎮 Zoom Control Details

### Zoom Levels
- **Range**: 1x to 30x (camera-dependent)
- **Default**: 1x (no zoom)
- **Command**: Uses SIYI SDK CMD_ID 0x05

### Zoom Commands
```bash
# Zoom in gradually
ros2 run video_cam camera_control zoom_in 1
ros2 run video_cam camera_control zoom_in 1
ros2 run video_cam camera_control zoom_in 1

# Jump to specific zoom
ros2 run video_cam camera_control set_zoom 15

# Zoom out to baseline
ros2 run video_cam camera_control set_zoom 1
```

### Monitor Zoom Level
```bash
ros2 topic echo /camera/zoom_level
```

## 🐛 Troubleshooting

### Camera Not Connecting
```bash
# Check network
ping 192.168.144.25

# Check service is available
ros2 service list | grep camera

# Check node is running
ros2 node list | grep siyi
```

### Service Call Fails
```bash
# Verify service definition
ros2 interface show interfaces/srv/CameraCommand

# Check for error messages
ros2 run video_cam image_pub  # Check logs
```

### Build Errors
```bash
# Clean and rebuild
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
rm -rf build/interfaces build/video_cam install/interfaces install/video_cam
colcon build --packages-select interfaces video_cam
source install/setup.bash
```

## 📁 File Structure
```
ros2_ws/
├── src/
│   ├── interfaces/
│   │   ├── srv/
│   │   │   └── CameraCommand.srv          # Service definition
│   │   └── CMakeLists.txt                 # Updated with new service
│   └── video_cam/
│       ├── video_cam/
│       │   ├── image_pub_siyi.py          # Main camera node (refactored)
│       │   └── camera_control_client.py   # Command-line client
│       ├── test_camera_system.sh          # Full system test
│       ├── setup.py                       # Updated entry points
│       └── package.xml                    # Dependencies
└── install/
    └── video_cam/
        └── lib/video_cam/
            ├── image_pub                  # Camera node executable
            └── camera_control             # Control client executable
```

## ✅ Verification Checklist

- [x] Code refactored for single responsibility
- [x] Camera control service implemented
- [x] Zoom control working
- [x] Manual photo capture working
- [x] Command-line interface created
- [x] Builds without errors
- [x] Sources correctly
- [x] Node starts properly
- [x] Services accessible
- [x] Documentation complete

## 🚀 Next Steps

### Integration with Mission Planning
```python
# In your mission planner node:
from interfaces.srv import CameraCommand

# At each waypoint:
camera_client.call_async(
    CameraCommand.Request(command='take_photo', parameter='')
)
```

### Custom Zoom Profiles
```python
# Create zoom sequence for different targets
zoom_sequence = [1, 5, 10, 15, 10, 5, 1]

for zoom in zoom_sequence:
    camera_client.call_async(
        CameraCommand.Request(command='set_zoom', parameter=str(zoom))
    )
    time.sleep(2)  # Wait between zooms
```

## 📝 Summary

The SIYI camera control system is now:
- ✅ **Fully functional** with proper method separation
- ✅ **Remotely controllable** via ROS2 services
- ✅ **Easy to use** with command-line tools
- ✅ **Well-tested** and documented
- ✅ **Production-ready** for deployment

All requirements met! 🎉
