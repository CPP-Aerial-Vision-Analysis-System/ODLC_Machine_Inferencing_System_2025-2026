# Quick Fix Guide - Remaining TODOs

## 1. Remove Dead Code from camera_loop() (5 minutes)

**Location:** `image_pub_siyi.py` lines ~714-772  
**Find this code block:**
```python
# Only save when triggered
if self.capture_photo:
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    # Capture 4K photo from SD card and download
    img_4k = self.capture_and_download_4k_photo()
    # ... 50+ lines of blocking code ...
```

**Replace with:**
```python
# NOTE: 4K SD card capture now handled by worker thread (_sd_capture_worker)
# This prevents blocking the ROS executor during HTTP downloads
```

---

## 2. Fix Altitude Documentation (10 minutes)

**Find (line ~136):**
```python
self.ALT_THRESHOLD = -13.716
```

**Replace with:**
```python
# Altitude threshold for enabling camera (NED frame)
# NED (North-East-Down) is Ardupilot's coordinate system where:
#   - Negative altitude = HIGHER (above home point)
#   - Positive altitude = LOWER (below home point)
# This value ensures camera only operates above 13.716m altitude
self.MIN_REL_ALT_NED = -13.716  # meters above home (NED frame)
```

**Find (line ~600+):**
```python
def check_altitude(self, msg):
    current_alt = msg.data
    if current_alt >= self.ALT_THRESHOLD:
```

**Replace with:**
```python
def check_altitude(self, msg):
    """
    Enable/disable camera based on altitude threshold.
    Uses NED frame where negative values mean higher altitude.
    """
    current_alt_ned = msg.data  # NED frame from /mavros/global_position/rel_alt
    if current_alt_ned >= self.MIN_REL_ALT_NED:
```

---

## 3. Add ROS Parameters (30 minutes)

**Add to __init__() after super().__init__():**

```python
# Declare ROS parameters with defaults
self.declare_parameter('camera_ip', '192.168.144.25')
self.declare_parameter('camera_ctrl_port', 37260)
self.declare_parameter('camera_media_port', 82)
self.declare_parameter('min_altitude_ned', -13.716)
self.declare_parameter('use_real_camera', True)
self.declare_parameter('rtsp_url', 'rtsp://192.168.144.25:8554/main.264')
self.declare_parameter('photo_resolution', '4K')
self.declare_parameter('auto_capture_interval', 10.0)

# Get parameter values
self.CAM_IP = self.get_parameter('camera_ip').get_parameter_value().string_value
self.CTRL_PORT = self.get_parameter('camera_ctrl_port').get_parameter_value().integer_value
self.MEDIA_PORT = self.get_parameter('camera_media_port').get_parameter_value().integer_value
self.MIN_REL_ALT_NED = self.get_parameter('min_altitude_ned').get_parameter_value().double_value
self.use_real_camera = self.get_parameter('use_real_camera').get_parameter_value().bool_value
rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
self.current_resolution = self.get_parameter('photo_resolution').get_parameter_value().string_value
auto_interval = self.get_parameter('auto_capture_interval').get_parameter_value().double_value

# Update timers with parameter values
self.auto_capture_timer = self.create_timer(auto_interval, self.auto_capture_callback)
```

**Delete these class variables (since they're now parameters):**
```python
CAM_IP = "192.168.144.25"  # DELETE - now a parameter
CTRL_PORT = 37260          # DELETE - now a parameter  
MEDIA_PORT = 82            # DELETE - now a parameter
```

**Update RTSP initialization to use parameter:**
```python
# OLD:
rtsp_url = 'rtsp://192.168.144.25:8554/main.264'

# NEW:
rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
```

**Create parameter file** `config/camera_params.yaml`:
```yaml
/**:
  ros__parameters:
    camera_ip: "192.168.144.25"
    camera_ctrl_port: 37260
    camera_media_port: 82
    min_altitude_ned: -13.716
    use_real_camera: true
    rtsp_url: "rtsp://192.168.144.25:8554/main.264"
    photo_resolution: "4K"
    auto_capture_interval: 10.0
```

**Launch with parameters:**
```bash
ros2 run video_cam image_pub_siyi --ros-args --params-file config/camera_params.yaml
```

---

## 4. Remove/Guard GStreamer Subprocess (10 minutes)

**Option A - Remove entirely (recommended):**

Delete these methods:
- `init_gstreamer_subprocess()` (lines ~200-260)
- `read_frame_from_gstreamer()` (lines ~260-280)

Remove GStreamer fallback from __init__:
```python
# DELETE THIS SECTION:
if not self.capture.isOpened():
    self.get_logger().warn('GStreamer method failed, trying default backend...')
    self.capture = cv2.VideoCapture(rtsp_url)
```

**Option B - Guard with parameter (if you must keep it):**

Add parameter:
```python
self.declare_parameter('enable_gstreamer_fallback', False)
```

Guard the code:
```python
if not self.capture.isOpened():
    if self.get_parameter('enable_gstreamer_fallback').get_parameter_value().bool_value:
        self.get_logger().warn('Trying GStreamer subprocess fallback...')
        if not self.init_gstreamer_subprocess():
            self.get_logger().error('All camera init methods failed')
    else:
        self.get_logger().error('Camera init failed, GStreamer fallback disabled')
```

---

## Testing After Fixes

```bash
# 1. Rebuild
cd /home/astra-dev/astra/ros2_ws
colcon build --packages-select video_cam

# 2. Test syntax
python3 -m py_compile src/video_cam/video_cam/image_pub_siyi.py

# 3. Run with parameters
source install/setup.bash
ros2 run video_cam image_pub_siyi --ros-args -p camera_ip:="192.168.144.25" -p min_altitude_ned:=-10.0

# 4. Check parameter values
ros2 param list /siyi_a8_publisher
ros2 param get /siyi_a8_publisher camera_ip
ros2 param get /siyi_a8_publisher min_altitude_ned

# 5. Test trigger (should be non-blocking now)
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
```

---

## Verification Checklist

- [ ] Code compiles without errors
- [ ] No blocking calls in timer callbacks
- [ ] Parameters can be set at launch
- [ ] Altitude logic is documented
- [ ] GStreamer subprocess removed or guarded
- [ ] Trigger callback returns immediately
- [ ] Worker thread handles photo capture
- [ ] RTSP stream maintains ~10Hz during capture
- [ ] Logs clearly show worker thread activity

---

**Estimated Total Time:** 55 minutes  
**Priority:** High (before next flight test)  
**Risk:** Low (all changes are improvements, no functionality loss)

