# Quick Reference: Fixed Camera Topics

## 🎯 The Problem
**RTSP stream was overwriting 4K captures** because both published to same topic.

## ✅ The Solution
Split into two topics with proper semantics.

---

## New Topic Structure

```
/camera/live     → RTSP firehose (10Hz continuous)
/camera/capture  → 4K events (on trigger)
/camera/status   → Status messages
```

---

## How to Subscribe

### For Live Viewing (rviz, GUI):
```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(
    Image,
    '/camera/live',
    self.callback,
    qos_profile_sensor_data
)
```

### For Detection/Processing (YOLO, mapping):
```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(
    Image,
    '/camera/capture',
    self.callback,
    qos_profile_sensor_data
)
```

---

## Testing Commands

```bash
# List topics
ros2 topic list | grep camera

# Monitor live stream rate
ros2 topic hz /camera/live

# Trigger capture
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

# Watch for capture (blocks until message)
ros2 topic echo /camera/capture --once

# Check both simultaneously
ros2 topic hz /camera/live &
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
```

---

## What Changed in image_pub_siyi.py

### Publishers:
- ❌ `self.publisher` (removed)
- ✅ `self.live_pub` → `/camera/live`
- ✅ `self.capture_pub` → `/camera/capture`

### QoS:
- ✅ Both use `qos_profile_sensor_data`

### Publishing:
- RTSP stream → `self.live_pub.publish()` (in camera_loop)
- 4K captures → `self.capture_pub.publish()` (in camera_loop, after worker prepares)

---

## Nodes That Need Updates

Update these to subscribe to correct topic:

1. **Detection nodes** → Subscribe to `/camera/capture`
   - `ultralytics_ros` (YOLO)
   - `mapping` node
   - Any ML inference nodes

2. **Viewing nodes** → Subscribe to `/camera/live`
   - rviz image display
   - Web interface
   - Recording/logging nodes (if continuous)

3. **Status monitoring** → Subscribe to `/camera/status`
   - Ground station
   - Health monitors

---

## Quick Migration Guide

### Old Code:
```python
self.create_subscription(Image, 'image_raw', self.callback, 10)
```

### New Code (for detection):
```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(
    Image,
    '/camera/capture',  # ← Changed
    self.callback,
    qos_profile_sensor_data  # ← Changed
)
```

### New Code (for viewing):
```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(
    Image,
    '/camera/live',  # ← Changed
    self.callback,
    qos_profile_sensor_data  # ← Changed
)
```

---

## Expected Behavior

### /camera/live:
- Publishes at ~10 Hz continuously
- Lower resolution (RTSP stream quality)
- For real-time viewing
- Never blocks

### /camera/capture:
- Publishes only when triggered
- High resolution (4K from SD card)
- For detection/mapping
- May have 1-3 second delay (SD download)

---

## Troubleshooting

### "No data on /camera/capture"
1. Check trigger was sent: `ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"`
2. Check camera node logs for "Worker: Starting 4K photo capture"
3. Verify camera is above altitude threshold

### "QoS mismatch warning"
Make sure subscriber uses `qos_profile_sensor_data`

### "Still getting low-res images in detection"
You're probably still subscribed to `/camera/live` instead of `/camera/capture`

---

**Build:** ✅ Success  
**Ready:** ✅ Testing  
**Docs:** ✅ Updated

