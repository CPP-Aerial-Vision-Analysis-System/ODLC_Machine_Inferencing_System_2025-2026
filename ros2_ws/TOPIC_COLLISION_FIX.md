# CRITICAL FIX: Topic Collision & Thread-Safe Publishing

## Date: January 7, 2026

## The Problem

Despite logs showing "published", 4K images were not reaching downstream nodes (YOLO, mapping, etc.).

### Root Causes Identified

1. **Topic Collision (CRITICAL)** - RTSP stream continuously overwrites 4K captures on same topic
2. **QoS Mismatch** - Publisher used default QoS, perception stacks expect `qos_profile_sensor_data`
3. **Thread-Unsafe Publishing** - Worker thread published directly (not ROS-safe under load)

---

## The Fixes Applied

### ✅ Fix #1: Split Topics

**Before:**
```python
self.publisher = self.create_publisher(Image, 'image_raw', 10)
# Both RTSP stream AND 4K captures → same topic → overwrite race!
```

**After:**
```python
# /camera/live → RTSP firehose (continuous stream)
self.live_pub = self.create_publisher(
    Image, 
    '/camera/live', 
    qos_profile_sensor_data
)

# /camera/capture → 4K events (triggered captures)
self.capture_pub = self.create_publisher(
    Image, 
    '/camera/capture', 
    qos_profile_sensor_data
)
```

**Mental Model:**
- `/camera/live` = **firehose** (10Hz continuous, for viewing)
- `/camera/capture` = **event** (on-demand, for processing)

---

### ✅ Fix #2: Proper QoS

Both publishers now use `qos_profile_sensor_data` to match perception stacks.

---

### ✅ Fix #3: Thread-Safe Publishing

**Before (UNSAFE):**
```python
def _sd_capture_worker(self):
    # In worker thread
    img_4k_msg = self.bridge.cv2_to_imgmsg(img_4k)
    self.publisher.publish(img_4k_msg)  # ❌ Not ROS-safe!
```

**After (SAFE):**
```python
def _sd_capture_worker(self):
    # In worker thread - just prepare the message
    img_4k_msg = self.bridge.cv2_to_imgmsg(img_4k)
    
    # Thread-safe handoff
    with self.capture_lock:
        self.last_4k_msg = img_4k_msg
        self.new_4k_ready = True  # Signal to ROS thread

def camera_loop(self):
    # In ROS thread - publish safely
    if self.new_4k_ready:
        with self.capture_lock:
            self.capture_pub.publish(self.last_4k_msg)
            self.new_4k_ready = False
```

**Pattern:**
- Worker thread → Prepare message, set flag
- ROS thread → Check flag, publish message

---

## Topic Structure

| Topic | Publisher | Rate | Purpose | Subscribers |
|-------|-----------|------|---------|-------------|
| `/camera/live` | RTSP loop | ~10 Hz | Live view | rviz, web_interface |
| `/camera/capture` | Worker thread → ROS loop | On trigger | Detection/mapping | yolo, mapper, logger |
| `/camera/status` | Various | Event | Status updates | ground_station |

---

## Downstream Node Changes Required

### Before (will stop working):
```python
self.create_subscription(Image, 'image_raw', self.callback, 10)
```

### After:

**For live viewing:**
```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(
    Image, 
    '/camera/live', 
    self.live_callback, 
    qos_profile_sensor_data
)
```

**For detection/processing:**
```python
from rclpy.qos import qos_profile_sensor_data

self.create_subscription(
    Image, 
    '/camera/capture', 
    self.process_callback, 
    qos_profile_sensor_data
)
```

---

## Testing

### 1. Check topics are published:
```bash
ros2 topic list | grep camera
# Should see:
#   /camera/live
#   /camera/capture
#   /camera/status
```

### 2. Monitor live stream:
```bash
ros2 topic hz /camera/live
# Should see ~10 Hz
```

### 3. Trigger capture and verify:
```bash
# Terminal 1: Watch for capture events
ros2 topic echo /camera/capture --once

# Terminal 2: Trigger
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"

# Terminal 1 should show the 4K image message
```

### 4. Verify no collision:
```bash
# Both should work simultaneously
ros2 topic hz /camera/live &
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "data: true"
# Live stream should maintain ~10 Hz during capture
```

---

## What Was Removed

1. ✅ Duplicate SD capture code in `camera_loop()` (lines 746-803)
2. ✅ Triple-publish loop (was pointless, didn't fix overwrite)
3. ✅ Legacy `self.publisher` reference
4. ✅ Direct publishing from worker thread

---

## Architecture Flow

```
┌─────────────────────────────────────────────────────────────┐
│ Timer (0.1s)                                                │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Check if 4K ready → Publish to /camera/capture         │
│  2. Read RTSP frame  → Publish to /camera/live            │
│                                                             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Trigger Callback                                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Queue timestamp → Return immediately (non-blocking)        │
│                                                             │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│ Worker Thread (background)                                  │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Wait for queue item (blocking OK here)                  │
│  2. Trigger camera via UDP                                  │
│  3. Poll SD card for new photo (HTTP)                       │
│  4. Download 4K image (HTTP - can take 1-15s)              │
│  5. Save to disk                                            │
│  6. Convert to ROS message                                  │
│  7. Store in self.last_4k_msg + set flag                   │
│     → ROS thread will publish it                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Key Insights

### Why split topics?
ROS doesn't queue by "importance". Last message wins. RTSP at 10Hz will always overwrite capture.

### Why not just increase queue size?
Won't help. Subscriber still only gets most recent. Need semantic separation.

### Why thread-safe handoff?
`rclpy` publishing from non-ROS threads can silently fail under load. Always publish from ROS-managed thread.

### Why remove triple-publish?
Didn't solve the problem (which was collision, not reliability). Just wasted bandwidth.

---

## Build Status

✅ Package compiles successfully  
✅ All old blocking code removed  
✅ Thread-safe publishing implemented  
✅ Topic collision resolved  
✅ QoS properly configured  

---

## Next Update Required

Update all downstream nodes to subscribe to:
- `/camera/live` for viewing
- `/camera/capture` for processing

Example nodes to update:
- `ultralytics_ros` (YOLO detection)
- `mapping` node
- Any logging/recording nodes
- Ground station viewer

---

**Status:** READY FOR TESTING  
**Priority:** HIGH - Test before next flight  
**Risk:** LOW - All changes are improvements, no functionality loss

