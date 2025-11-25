# Object Detection SAHI Upgrade Documentation

**Document Created:** November 25, 2025  
**Purpose:** Compare and document differences between the basic (`detection/`) and advanced (`video_cam/`) versions of `object_detection_sahi.py`

---

## Executive Summary

Two versions of `object_detection_sahi.py` exist in this workspace:

| Version | Location | Lines | Status |
|---------|----------|-------|--------|
| **Basic** | `/detection/detection/` | 887 | Original, simpler implementation |
| **Advanced** | `/video_cam/video_cam/` | 1,292 | Production-ready with enterprise features |

**Size Difference:** +405 lines (+45% more code)

---

## Key Architectural Differences

### 1. **Documentation & Code Organization**

#### Basic Version (`detection/`)
- Simple docstring
- No inline comments explaining architecture
- No constants defined

#### Advanced Version (`video_cam/`)
```python
# Extensive docstring explaining:
- Current Architecture (SAHI + YOLO pipeline)
- Detection Pipeline (4-step process)
- Configuration rationale
- Inline comments throughout code

# Constants for maintainability:
DEFAULT_CONFIDENCE_THRESHOLD = 0.15
DEFAULT_SLICE_SIZE = 512
DEFAULT_OVERLAP = 0.3
DEFAULT_CHECK_INTERVAL = 2.0
MAX_SEARCH_DEPTH = 10
```

---

### 2. **Import Differences**

#### Basic Version
```python
from ament_index_python.packages import get_package_share_directory
# No async, no type hints, no advanced ROS2 features
```

#### Advanced Version
```python
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, Future  # Async processing
from typing import List, Dict, Optional, Tuple  # Type safety
import gc  # Memory management
from rclpy.parameter import Parameter  # Dynamic reconfiguration
from rclpy.qos import QoSProfile, ReliabilityPolicy  # Reliable messaging
from mavros_msgs.msg import WaypointReached  # Mission integration
```

**Added Capabilities:**
- Asynchronous/parallel processing
- Type annotations for better IDE support
- Dynamic parameter updates at runtime
- GPS/waypoint integration
- Advanced memory management

---

### 3. **Directory Structure & Path Management**

#### Basic Version
```python
# Hardcoded fallback path
if ros2_ws_dir is None:
    ros2_ws_dir = "/astra/ros2_ws/src"

# Monitors: mapping_photos/
# Saves to: detection_results/
```

#### Advanced Version
```python
# Uses helper function: get_video_cam_directory()
# Flexible search with MAX_SEARCH_DEPTH
# Environment variable support: ROS2_WS_PATH
# Better error handling with OSError exceptions

# Monitors: camera_feed/
# Saves to: detection_results_sahi/
```

**Migration Note:** Different folder names mean the two versions **won't process the same images** unless paths are aligned.

---

### 4. **ROS2 Parameters**

#### Basic Version (8 parameters)
```python
model_path
confidence_threshold
slice_height
slice_width
overlap_height_ratio
overlap_width_ratio
check_interval
device
```

#### Advanced Version (15 parameters)
```python
# All basic parameters PLUS:
max_images_per_cycle = 5           # Batch processing limit
max_camera_feed_images = 100       # Auto-cleanup threshold
min_detection_area = 100           # Filter tiny noise
max_detection_area = 1000000       # Filter huge false positives
min_aspect_ratio = 0.1             # Shape filtering
max_aspect_ratio = 10.0            # Shape filtering
enable_gpu_memory_cleanup = True   # Periodic GPU cache clearing
```

**Benefits:**
- Prevents memory leaks on Jetson/GPU devices
- Automatic cleanup of old images
- Better false-positive filtering
- Configurable batch sizes for performance tuning

---

### 5. **Parameter Validation**

#### Basic Version
- No validation
- Uses parameters as-is

#### Advanced Version
```python
def _load_and_validate_parameters(self) -> None:
    """Load and validate all parameters"""
    # Validates confidence_threshold (0 < x <= 1.0)
    # Validates slice sizes (minimum 64x64)
    # Validates overlap ratios (0 <= x < 1.0)
    # Validates check_interval (>= 0.1)
    # Validates max_images_per_cycle (>= 1)
    # Logs warnings and applies safe defaults
```

---

### 6. **ROS2 Communication**

#### Basic Version
```python
# Standard QoS (depth=10)
self.publisher = self.create_publisher(Image, '/sahi_detection_results', 10)
self.detection_publisher = self.create_publisher(String, '/sahi_detection_info', 10)
self.detection_pub = self.create_publisher(ImageResult, '/image_detections', 10)
```

#### Advanced Version
```python
# Custom QoS with RELIABLE policy
qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)
self.publisher = self.create_publisher(Image, '/sahi_detection_results', qos_profile)
# ... all publishers use qos_profile
```

**Benefit:** Guaranteed message delivery, no dropped detections in poor network conditions.

---

### 7. **Processing State Management**

#### Basic Version
```python
self.processed_images = set()  # Simple set of filenames
```

#### Advanced Version
```python
self.processed_images: Dict[str, float] = {}  # Filename -> timestamp mapping
self.processing_queue: List[str] = []         # Queue for batch processing
self.processing_lock = False                   # Concurrency control
```

**Benefits:**
- Track **when** each image was processed (useful for debugging)
- Queue management for batch processing
- Thread-safe processing with lock

---

### 8. **Asynchronous Processing**

#### Basic Version
- Synchronous only
- Blocks on each image
- No parallelism

#### Advanced Version
```python
# Thread pool for parallel processing
self.thread_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="sahi_worker")
self.active_futures: List[Future] = []

# In check_for_new_images():
future = self.thread_pool.submit(self._process_image_safe, image_path)
self.active_futures.append(future)

# Cleanup completed futures
self.active_futures = [f for f in self.active_futures if not f.done()]
```

**Benefits:**
- Process multiple images in parallel (2 workers)
- Non-blocking detection loop
- Better throughput for high-frequency image feeds

---

### 9. **Statistics Tracking**

#### Basic Version (5 stats)
```python
self.stats = {
    'total_images_processed': 0,
    'total_detections': 0,
    'total_tents': 0,
    'total_people': 0,
    'avg_processing_time': 0.0
}
```

#### Advanced Version (8 stats + health monitoring)
```python
self.stats = {
    'total_images_processed': 0,
    'total_detections': 0,
    'total_tents': 0,
    'total_people': 0,
    'avg_processing_time': 0.0,
    'last_processing_time': 0.0,    # Track most recent
    'node_start_time': time.time(),  # Calculate uptime
    'errors': 0                      # Error counting
}

self.health_status = {
    'is_healthy': True,
    'last_successful_detection': None,
    'consecutive_errors': 0
}
```

---

### 10. **Services (ROS2 Services)**

#### Basic Version
- No services

#### Advanced Version
```python
from std_srvs.srv import Trigger

# Statistics service
self.stats_service = self.create_service(
    Trigger, 'sahi/get_statistics', self._get_statistics_service
)

# Health check service
self.health_service = self.create_service(
    Trigger, 'sahi/get_health', self._get_health_service
)
```

**Usage:**
```bash
ros2 service call /sahi/get_statistics std_srvs/srv/Trigger
ros2 service call /sahi/get_health std_srvs/srv/Trigger
```

---

### 11. **Dynamic Reconfiguration**

#### Basic Version
- No runtime parameter changes

#### Advanced Version
```python
self.add_on_set_parameters_callback(self._parameter_callback)

def _parameter_callback(self, params: List[Parameter]):
    """Handle parameter changes at runtime"""
    # Supports changing:
    # - confidence_threshold
    # - check_interval (recreates timer)
    # - max_images_per_cycle
    # Validates before applying
    # Returns success/failure with reason
```

**Usage:**
```bash
ros2 param set /sahi_object_detection_node confidence_threshold 0.2
ros2 param set /sahi_object_detection_node max_images_per_cycle 10
```

---

### 12. **GPU Memory Management**

#### Basic Version
```python
# One-time cache clear before model load
torch.cuda.empty_cache()
```

#### Advanced Version
```python
# Aggressive initial cleanup
gc.collect()
torch.cuda.empty_cache()
torch.cuda.synchronize()
torch.cuda.set_per_process_memory_fraction(0.8, 0)

# Periodic cleanup timer (every 30 seconds)
if self.enable_gpu_memory_cleanup and self.device.startswith('cuda'):
    self.gpu_cleanup_timer = self.create_timer(30.0, self._periodic_gpu_cleanup)

def _periodic_gpu_cleanup(self):
    gc.collect()
    torch.cuda.empty_cache()
    # Logs memory usage
```

**Critical for:**
- Jetson devices with limited GPU memory
- Long-running detection sessions
- Prevents out-of-memory crashes

---

### 13. **Image Cleanup**

#### Basic Version
- No automatic cleanup
- Images accumulate indefinitely

#### Advanced Version
```python
def _cleanup_old_images(self) -> None:
    """Remove old images from camera_feed if limit is exceeded"""
    if len(image_files) > self.max_camera_feed_images:
        # Remove oldest images
        # Update processed_images dict
        # Log cleanup action
```

**Prevents:**
- Disk space exhaustion
- Directory listing slowdown
- Stale image processing

---

### 14. **Batch Processing**

#### Basic Version
```python
# Process all new images immediately
for image_file in image_files:
    if image_file not in self.processed_images:
        self.process_image(image_path)
```

#### Advanced Version
```python
# Process in configurable batches
new_images = [fname for fname in images if fname not in self.processed_images]
batch = new_images[:self.max_images_per_cycle]

for image_file, mtime in batch:
    future = self.thread_pool.submit(self._process_image_safe, image_path)
```

**Benefits:**
- Controlled resource usage
- Prevents CPU/GPU overload
- Smoother performance with high image rates

---

### 15. **Error Handling & Health Monitoring**

#### Basic Version
```python
try:
    # Process image
except Exception as e:
    self.get_logger().error(f"Error: {e}")
```

#### Advanced Version
```python
def _process_image_safe(self, image_path: str) -> None:
    """Wrapper with comprehensive error handling"""
    try:
        self.process_image(image_path)
        # Update health on success
        self.health_status['last_successful_detection'] = time.time()
        self.health_status['consecutive_errors'] = 0
        self.health_status['is_healthy'] = True
    except Exception as e:
        self.stats['errors'] += 1
        self.health_status['consecutive_errors'] += 1
        if self.health_status['consecutive_errors'] > 5:
            self.health_status['is_healthy'] = False
```

**Benefits:**
- Track error patterns
- Automatic health status updates
- Degraded operation detection

---

### 16. **Detection Filtering**

#### Basic Version
```python
def _categorize_detection():
    # Basic person/tent detection
    # Simple confidence thresholds
```

#### Advanced Version
```python
def _categorize_detection():
    # Calculates: width, height, area, aspect_ratio
    
    # Area filtering
    if area < self.min_detection_area or area > self.max_detection_area:
        return None
    
    # Aspect ratio filtering
    if aspect_ratio < self.min_aspect_ratio or aspect_ratio > self.max_aspect_ratio:
        return None
    
    # Tent detection: only 'kite' and 'umbrella' (more conservative)
    # Mannequin detection: added 'doll' category
```

**Benefits:**
- Reduces false positives
- Filters noise/artifacts
- More robust tent detection strategy

---

### 17. **Waypoint Integration**

#### Basic Version
```python
# No waypoint tracking
```

#### Advanced Version
```python
self.waypoint_reached = 0
self.create_subscription(
    WaypointReached, 
    "/mavros/mission/reached", 
    self.waypoint_reached_cb, 
    10
)

# In publish_results:
image_result_msg.waypoint_index = self.waypoint_reached
```

**Use Case:** Associate detections with GPS waypoints for mission replay/analysis.

---

### 18. **Model Path Resolution**

#### Basic Version
```python
# Simple check
if not os.path.exists(model_path):
    alt_path = os.path.join('/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws', self.model_path)
```

#### Advanced Version
```python
# Multi-location search:
# 1. Check video_cam_directory
# 2. Check package directory (__file__ location)
# 3. Check current working directory
# 4. Fall back to Ultralytics download

if not os.path.isabs(model_path):
    video_cam_model_path = os.path.join(video_cam_dir, model_path)
    if os.path.exists(video_cam_model_path):
        model_path = video_cam_model_path
    elif os.path.exists(os.path.join(os.path.dirname(__file__), model_path)):
        model_path = os.path.join(os.path.dirname(__file__), model_path)
```

---

### 19. **Shutdown Handling**

#### Basic Version
```python
except KeyboardInterrupt:
    node.get_logger().info("Shutting down...")
finally:
    node._log_statistics()
    node.destroy_node()
    rclpy.shutdown()
```

#### Advanced Version
```python
except KeyboardInterrupt:
    node.get_logger().info("Shutting down...")
finally:
    node.get_logger().info("Shutting down...")
    node._log_statistics()
    
    # Cleanup thread pool executor
    if hasattr(node, 'executor'):
        node.executor.shutdown(wait=True, timeout=30.0)
    
    node.destroy_node()
    rclpy.shutdown()
```

**Benefit:** Graceful thread cleanup, no orphaned workers.

---

## Feature Comparison Matrix

| Feature | Basic (`detection/`) | Advanced (`video_cam/`) |
|---------|---------------------|------------------------|
| **Lines of Code** | 887 | 1,292 (+45%) |
| **Async Processing** | ❌ | ✅ ThreadPoolExecutor |
| **Type Hints** | ❌ | ✅ Full typing |
| **QoS Profiles** | ❌ Default | ✅ RELIABLE |
| **Parameter Validation** | ❌ | ✅ Comprehensive |
| **Dynamic Reconfiguration** | ❌ | ✅ Runtime params |
| **Health Monitoring** | ❌ | ✅ Health status + services |
| **GPU Memory Cleanup** | ⚠️ Basic | ✅ Periodic + aggressive |
| **Image Cleanup** | ❌ | ✅ Auto-delete old images |
| **Batch Processing** | ❌ | ✅ Configurable batches |
| **Error Tracking** | ⚠️ Basic | ✅ Counters + health |
| **Statistics Services** | ❌ | ✅ ROS2 services |
| **Waypoint Integration** | ❌ | ✅ MAVROS integration |
| **Detection Filtering** | ⚠️ Basic | ✅ Area + aspect ratio |
| **Documentation** | ⚠️ Minimal | ✅ Extensive |
| **Constants/Config** | ❌ | ✅ Named constants |

---

## Performance Implications

### Basic Version
- **Pros:** Simpler, easier to understand, lower memory overhead
- **Cons:** Blocks on processing, no parallelism, unlimited disk usage

### Advanced Version
- **Pros:** Parallel processing, automatic cleanup, better error recovery
- **Cons:** More complex, higher initial memory overhead, requires understanding of async patterns

### Recommended Use Cases

| Scenario | Recommended Version |
|----------|---------------------|
| **Development/Testing** | Basic (simpler debugging) |
| **Production Deployment** | Advanced (robustness) |
| **Jetson/GPU Constrained** | Advanced (memory management) |
| **High Image Rate (>1 Hz)** | Advanced (batch + async) |
| **Mission-Critical** | Advanced (health monitoring) |
| **Long-Running (>1 hour)** | Advanced (cleanup + memory) |
| **Simple Demo** | Basic (less overhead) |

---

## Migration Guide

### From Basic → Advanced

1. **Update directory paths:**
   ```bash
   # Basic uses: mapping_photos/
   # Advanced uses: camera_feed/
   
   # Option 1: Symlink
   ln -s mapping_photos camera_feed
   
   # Option 2: Change advanced code to use mapping_photos
   ```

2. **Add new parameters to launch files:**
   ```python
   'max_images_per_cycle': 5,
   'max_camera_feed_images': 100,
   'min_detection_area': 100,
   'max_detection_area': 1000000,
   'min_aspect_ratio': 0.1,
   'max_aspect_ratio': 10.0,
   'enable_gpu_memory_cleanup': True,
   ```

3. **Update package dependencies:**
   ```xml
   <!-- package.xml -->
   <depend>mavros_msgs</depend>
   <depend>std_srvs</depend>
   ```

4. **Test services:**
   ```bash
   ros2 service call /sahi/get_statistics std_srvs/srv/Trigger
   ros2 service call /sahi/get_health std_srvs/srv/Trigger
   ```

5. **Monitor health:**
   ```bash
   ros2 param get /sahi_object_detection_node
   ros2 topic echo /sahi_detection_info
   ```

---

## Backward Compatibility

The advanced version is **mostly backward compatible** with basic usage:

✅ **Compatible:**
- Same core 8 parameters work
- Same topics published
- Same detection logic (SAHI + YOLO)

⚠️ **Requires Changes:**
- Different folder paths (`mapping_photos` vs `camera_feed`)
- Different message imports (if used externally)

❌ **Not Compatible:**
- Cannot run both versions simultaneously (same node name)
- Different published topic formats (ImageResult structure)

---

## Recommendations

### For New Projects
→ **Use Advanced Version** (`video_cam/video_cam/`)
- More robust
- Production-ready
- Better long-term maintainability

### For Existing Basic Projects
→ **Evaluate Migration Based On:**
- Is memory management an issue? → Migrate
- Need health monitoring? → Migrate
- Processing high image rates? → Migrate
- Simple proof-of-concept? → Keep basic

### For Jetson Deployment
→ **Strongly Recommend Advanced Version**
- GPU memory cleanup critical
- Health monitoring valuable
- Batch processing prevents overload

---

## Testing Checklist

When switching versions, verify:

- [ ] Model loads correctly
- [ ] Images are detected in correct directory
- [ ] Detections are saved to correct output folder
- [ ] GPU memory usage is stable over 30+ minutes
- [ ] Parameters can be changed at runtime (advanced only)
- [ ] Services respond correctly (advanced only)
- [ ] Node recovers from errors
- [ ] Statistics are accurate
- [ ] Waypoint integration works (if using MAVROS)

---

## Quick Reference Commands

### Running Basic Version
```bash
ros2 run detection object_detection_sahi --ros-args -p slice_height:=256
```

### Running Advanced Version
```bash
ros2 run video_cam object_detection_sahi --ros-args \
  -p slice_height:=256 \
  -p max_images_per_cycle:=3 \
  -p enable_gpu_memory_cleanup:=true
```

### Checking Health (Advanced Only)
```bash
ros2 service call /sahi/get_health std_srvs/srv/Trigger
```

### Changing Parameters at Runtime (Advanced Only)
```bash
ros2 param set /sahi_object_detection_node confidence_threshold 0.2
```

---

## Conclusion

The **advanced version** (`video_cam/`) represents a significant evolution with 405 additional lines focused on:
- Production robustness
- Resource management
- Monitoring and observability
- Performance optimization

Choose the version that matches your deployment complexity and requirements.

---

**Last Updated:** November 25, 2025  
**Maintainer:** ODLC Machine Inferencing System Team
