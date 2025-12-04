# Object Detection SAHI (ODS) Documentation

## Methods Reference

### Standalone Functions

#### `get_detection_directory() -> str`

**Purpose**: Finds or creates the `detection` directory in the ROS2 workspace.

**How it works**:
1. Starts from the current file's location
2. Searches upward in the directory tree
3. Looks for `install/` and `src/` directories (signs of a ROS2 workspace)
4. If found, returns `ros2_ws/detection/`
5. If not found, uses environment variable `ROS2_WS_PATH` or defaults to `~/ros2_ws`
6. Creates the directory if it doesn't exist

**Returns**: Path to `detection` directory

**Raises**: `OSError` if directory cannot be created

---

### Class: SAHIObjectDetectionNode

#### `__init__(self)`

**Purpose**: Initializes the ROS2 node and sets up all components.

**What it does**:
1. Creates ROS2 node with name `'sahi_object_detection_node'`
2. Declares all parameters (model path, thresholds, slice sizes, etc.)
3. Validates parameters and sets defaults if invalid
4. Sets up ROS2 publishers and subscribers
5. Creates directories for camera feed and detection results
6. Auto-detects best available device (GPU/CPU)
7. Initializes the SAHI detection model
8. Sets up thread pool for parallel processing
9. Creates timer to check for new images
10. Sets up ROS2 services for statistics and health
11. Initializes statistics and health tracking

**Key Initializations**:
- `self.detection_model`: The SAHI model (None until loaded)
- `self.processed_images`: Dictionary tracking processed images with timestamps
- `self.executor`: Thread pool for async image processing
- `self.stats`: Dictionary with detection statistics
- `self.health_status`: Dictionary with node health information

---

#### `_load_and_validate_parameters(self) -> None`

**Purpose**: Loads all parameters from ROS2 and validates them.

**Validation Rules**:
- `confidence_threshold`: Must be between 0 and 1
- `slice_height/width`: Minimum 64 pixels
- `overlap_*_ratio`: Must be between 0 and 1
- `check_interval`: Minimum 0.1 seconds
- `max_images_per_cycle`: Minimum 1

**What happens**: If a parameter is invalid, it logs a warning and uses the default value.

---

#### `waypoint_reached_cb(self, msg: WaypointReached) -> None`

**Purpose**: Callback function that receives waypoint reached messages from the drone.

**What it does**: Updates `self.waypoint_reached` with the current waypoint index. This is included in detection results to track which waypoint the detection occurred at.

**Parameters**:
- `msg`: WaypointReached message containing waypoint sequence number

---

#### `_parameter_callback(self, params: List[Parameter]) -> SetParametersResult`

**Purpose**: Handles dynamic parameter changes at runtime (without restarting the node).

**Supported Parameters**:
- `confidence_threshold`: Updates detection sensitivity
- `check_interval`: Updates timer frequency (restarts timer)
- `max_images_per_cycle`: Updates batch size

**Returns**: `SetParametersResult` indicating success or failure with reason

**Example Usage**:
```bash
ros2 param set /sahi_object_detection_node confidence_threshold 0.25
```

---

#### `_get_statistics_service(self, request, response)`

**Purpose**: ROS2 service callback that returns detection statistics.

**Returns**:
- `response.success`: Always `True`
- `response.message`: String containing:
  - Total images processed
  - Total detections
  - Total tents and people
  - Average processing time
  - Last processing time
  - Error count
  - Node uptime

---

#### `_get_health_service(self, request, response)`

**Purpose**: ROS2 service callback that returns node health status.

**Returns**:
- `response.success`: `True` if healthy, `False` if unhealthy
- `response.message`: String containing:
  - Health status (healthy/unhealthy)
  - Consecutive error count
  - Last successful detection timestamp
  - Model initialization status

**Health Logic**: Node is considered unhealthy if there are more than 5 consecutive errors.

---

#### `_periodic_gpu_cleanup(self) -> None`

**Purpose**: Periodically cleans up GPU memory to prevent memory leaks.

**What it does**:
1. Runs every 30 seconds (if GPU cleanup is enabled)
2. Calls Python garbage collector
3. Empties CUDA cache
4. Logs current GPU memory usage (debug level)

**Why needed**: GPU memory can accumulate over time, especially on devices with limited memory like Jetson.

---

#### `_get_device(self) -> str`

**Purpose**: Auto-detects the best available device for running the model.

**Priority Order**:
1. **NVIDIA GPU (CUDA)**: For Linux, Windows, Jetson devices
2. **Apple Silicon GPU (MPS)**: For Mac M1/M2/M3
3. **CPU**: Fallback if no GPU available

**What it does**:
- Checks if PyTorch CUDA is available
- Checks if Apple MPS is available
- Logs device information (GPU name, memory, etc.)
- Provides helpful error messages if Jetson GPU is not available

**Returns**: Device string (`'cuda:0'`, `'mps'`, or `'cpu'`)

---

#### `initialize_sahi_model(self) -> bool`

**Purpose**: Loads and initializes the SAHI detection model with YOLO.

**What it does**:
1. Checks if SAHI and YOLO are available
2. Resolves model path (checks multiple locations):
   - `detection/` directory
   - Package directory
   - Current working directory
   - Downloads from Ultralytics if not found
3. If using GPU, optimizes memory settings:
   - Limits GPU memory usage to 80%
   - Enables cuDNN benchmarking
   - Logs available GPU memory
4. Creates `AutoDetectionModel` with:
   - Model type: `'yolov8'` (works for YOLO v8-11)
   - Model path: The YOLO model file
   - Confidence threshold: From parameters
   - Device: Auto-detected or specified
5. Verifies model is on correct device

**Returns**: `True` if successful, `False` if failed

**Error Handling**: Logs detailed error messages and suggests fixes (e.g., try CPU if GPU fails).

---

#### `check_for_new_images(self) -> None`

**Purpose**: Main timer callback that checks for new images and processes them.

**What it does**:
1. Checks if camera feed directory exists
2. Cleans up old images if limit exceeded
3. Scans directory for image files (`.jpg`, `.jpeg`, `.png`, `.bmp`)
4. Gets file modification times (for proper ordering)
5. Filters out already processed images
6. Processes new images in batches (up to `max_images_per_cycle`)
7. Submits images to thread pool for parallel processing
8. Tracks processed images with timestamps

**Called by**: Timer (every `check_interval` seconds)

**Error Handling**: Catches file system errors and updates health status.

---

#### `_cleanup_old_images(self) -> None`

**Purpose**: Removes old images from `camera_feed/` directory to prevent disk space issues.

**What it does**:
1. Gets all image files with timestamps
2. Sorts by modification time (oldest first)
3. If count exceeds `max_camera_feed_images`:
   - Removes oldest images
   - Removes them from processed images tracking
4. Logs how many images were removed

**When it runs**: Called by `check_for_new_images()` before processing

**Disabled if**: `max_camera_feed_images` is 0 or negative

---

#### `_process_image_safe(self, image_path: str) -> None`

**Purpose**: Wrapper around `process_image()` with error handling and health tracking.

**What it does**:
1. Calls `process_image()` in a try-except block
2. On success:
   - Updates last successful detection timestamp
   - Resets consecutive error count
   - Marks node as healthy
3. On error:
   - Logs error
   - Increments error statistics
   - Increments consecutive error count
   - Marks node as unhealthy if errors > 5

**Why needed**: Provides safe execution in thread pool and tracks health status.

---

#### `process_image(self, image_path: str) -> None`

**Purpose**: Processes a single image through the complete detection pipeline.

**What it does**:
1. Loads image using OpenCV
2. Logs image dimensions
3. Runs SAHI detection (`detect_objects_sahi()`)
4. Measures processing time
5. Annotates image with bounding boxes (`annotate_frame()`)
6. Publishes results to ROS2 topics (`publish_results()`)
7. Updates statistics:
   - Total images processed
   - Total detections
   - Total tents and people
   - Average processing time

**Error Handling**: Catches image loading errors and OpenCV errors separately.

---

#### `detect_objects_sahi(self, frame: np.ndarray) -> List[Dict]`

**Purpose**: Core detection function that uses SAHI to detect objects in an image.

**What it does**:
1. Converts image from BGR to RGB (OpenCV uses BGR, PyTorch uses RGB)
2. Calls SAHI's `get_sliced_prediction()`:
   - Slices image into overlapping patches
   - Runs YOLO on each slice
   - Merges results with NMS (Non-Maximum Suppression)
   - Uses IOS (Intersection Over Smaller area) for merging
3. Converts SAHI results to internal format:
   - Extracts bounding boxes
   - Extracts class names and confidence scores
   - Categorizes detections (`_categorize_detection()`)
4. Applies additional filtering (`_filter_detections()`)
5. Cleans up GPU memory if enabled

**Parameters**:
- `frame`: Image as numpy array (BGR format)

**Returns**: List of detection dictionaries, each containing:
- `'class'`: `'person'` or `'tent'`
- `'yolo_class'`: Original YOLO class name
- `'confidence'`: Detection confidence (0.0-1.0)
- `'bbox'`: Bounding box `[x1, y1, x2, y2]`
- `'description'`: Human-readable description
- `'method'`: `'sahi+yolo11s'`
- `'area'`: Bounding box area in pixels

**Error Handling**: Catches SAHI and PyTorch errors, logs with traceback.

---

#### `_categorize_detection(self, class_name: str, confidence: float, bbox: List[int], frame) -> Optional[Dict]`

**Purpose**: Categorizes YOLO detections into target classes (person/tent) and applies filtering.

**What it does**:
1. Calculates bounding box area and aspect ratio
2. Applies area filtering:
   - Rejects if area < `min_detection_area`
   - Rejects if area > `max_detection_area`
3. Applies aspect ratio filtering:
   - Rejects if ratio < `min_aspect_ratio` or > `max_aspect_ratio`
4. Categorizes detections:
   - **Person**: Direct `'person'` class (confidence > 0.25)
   - **Mannequin**: `'doll'` class mapped to person (with size/aspect validation)
   - **Tent**: `'kite'` or `'umbrella'` classes mapped to tent (with size/aspect validation)
5. Returns detection dictionary or `None` if filtered out

**Parameters**:
- `class_name`: YOLO class name (e.g., `'person'`, `'kite'`, `'umbrella'`)
- `confidence`: Detection confidence score
- `bbox`: Bounding box `[x1, y1, x2, y2]`
- `frame`: Original image (unused, kept for compatibility)

**Returns**: Detection dictionary or `None` if not a target class or filtered

**Why needed**: YOLO detects many classes, but we only care about people and tents. This function maps relevant classes and filters out invalid detections.

---

#### `_filter_detections(self, detections: List[Dict]) -> List[Dict]`

**Purpose**: Removes duplicate and overlapping detections using Non-Maximum Suppression (NMS).

**What it does**:
1. Sorts detections by confidence (highest first)
2. Separates detections by class (person vs tent)
3. For each class:
   - Applies NMS to remove overlapping detections
   - Uses IoU (Intersection over Union) to detect overlaps
   - Keeps highest confidence detection when overlaps occur
4. Combines filtered results from both classes

**Parameters**:
- `detections`: List of detection dictionaries

**Returns**: Filtered list with duplicates removed

**Why needed**: SAHI can produce multiple detections of the same object (from overlapping slices). NMS removes these duplicates.

---

#### `_apply_nms(self, detections: List[Dict], overlap_threshold: float = 0.3) -> List[Dict]`

**Purpose**: Applies Non-Maximum Suppression to a list of detections.

**Algorithm**:
1. Sorts detections by confidence (already sorted)
2. Takes highest confidence detection
3. Removes all detections that overlap significantly (IoU > threshold)
4. Repeats with remaining detections

**Parameters**:
- `detections`: List of detection dictionaries (sorted by confidence)
- `overlap_threshold`: IoU threshold (default 0.3 = 30% overlap)

**Returns**: List of detections with overlaps removed

**Why needed**: Prevents the same object from being detected multiple times.

---

#### `_calculate_iou(self, bbox1: List[int], bbox2: List[int]) -> float`

**Purpose**: Calculates Intersection over Union (IoU) of two bounding boxes.

**IoU Formula**:
```
IoU = Intersection Area / Union Area
```

**What it does**:
1. Calculates intersection rectangle
2. Calculates area of intersection
3. Calculates area of union (area1 + area2 - intersection)
4. Returns ratio (0.0 if no overlap, 1.0 if identical)

**Parameters**:
- `bbox1`: First bounding box `[x1, y1, x2, y2]`
- `bbox2`: Second bounding box `[x1, y1, x2, y2]`

**Returns**: IoU value between 0.0 and 1.0

**Why needed**: Used by NMS to determine if two detections are of the same object.

---

#### `annotate_frame(self, frame: np.ndarray, detections: List[Dict], processing_time: float) -> np.ndarray`

**Purpose**: Draws bounding boxes and labels on the image.

**What it does**:
1. Creates a copy of the original image
2. For each detection:
   - Chooses color (Green for person, Yellow for tent)
   - Draws bounding box rectangle
   - Draws text background (colored box)
   - Draws labels:
     - "TARGET: PERSON" or "TARGET: TENT"
     - "YOLO: {class} ({confidence})"
3. Adds header text:
   - Detection method and count
   - Processing time
4. Adds footer text:
   - SAHI mode information (slice size, overlap)

**Parameters**:
- `frame`: Original image as numpy array
- `detections`: List of detection dictionaries
- `processing_time`: Time taken for detection in seconds

**Returns**: Annotated image as numpy array

**Visual Style**:
- Green boxes and labels for people
- Yellow boxes and labels for tents
- White text on colored backgrounds
- Header and footer with detection info

---

#### `publish_results(self, annotated_frame: np.ndarray, detections: List[Dict], image_path: str) -> None`

**Purpose**: Publishes detection results to ROS2 topics and saves annotated image.

**What it does**:
1. Converts annotated image to ROS2 Image message
2. Publishes to `/sahi_detection_results` topic
3. Saves annotated image to `detection_results_sahi/` directory
   - Filename: `sahi_detected_{original_name}.{ext}`
4. Creates detection info dictionary with metadata
5. Creates `ImageResult` message with:
   - Image metadata (name, timestamp, path)
   - Detection count
   - Method and configuration
   - Waypoint index
   - Detection array (vision_msgs format)
   - Object summaries (classes, confidences, areas, descriptions)
6. Publishes `ImageResult` to `/image_detections` topic
7. Publishes detection info string to `/sahi_detection_info` topic

**Parameters**:
- `annotated_frame`: Image with bounding boxes drawn
- `detections`: List of detection dictionaries
- `image_path`: Path to original image file

**Error Handling**: Catches OpenCV and file system errors, updates error statistics.

---

#### `_log_statistics(self) -> None`

**Purpose**: Logs detection statistics to console.

**What it logs**:
- Total images processed
- Total detections
- Total tents and people detected
- Average processing time
- Last processing time
- Error count
- Node uptime

**When called**:
- Periodically after processing images
- On node shutdown
- Can be triggered via statistics service

---

#### `main(args=None)`

**Purpose**: Entry point for the ROS2 node.

**What it does**:
1. Checks if required dependencies are available (SAHI, YOLO)
2. Exits with error message if dependencies missing
3. Initializes ROS2
4. Creates `SAHIObjectDetectionNode` instance
5. Spins the node (keeps it running)
6. On shutdown:
   - Logs final statistics
   - Shuts down thread pool executor
   - Destroys node
   - Shuts down ROS2

**Error Handling**: Handles KeyboardInterrupt gracefully for clean shutdown.

---

## Usage Examples

### Basic Launch

```bash
# Launch with default parameters
ros2 launch detection detection_complete.launch.py
```

### Custom Parameters

```bash
# Launch with custom confidence threshold and slice size
ros2 launch detection detection_complete.launch.py \
    confidence_threshold:=0.25 \
    slice_height:=640 \
    slice_width:=640 \
    device:=cuda:0
```

### Runtime Parameter Changes

```bash
# Change confidence threshold while running
ros2 param set /sahi_object_detection_node confidence_threshold 0.30

# Change check interval
ros2 param set /sahi_object_detection_node check_interval 1.0

# Change batch size
ros2 param set /sahi_object_detection_node max_images_per_cycle 10
```

### Query Statistics

```bash
# Get detection statistics
ros2 service call /sahi/get_statistics std_srvs/srv/Trigger

# Check node health
ros2 service call /sahi/get_health std_srvs/srv/Trigger
```

### Monitor Topics

```bash
# View annotated images
ros2 topic echo /sahi_detection_results

# View detection info
ros2 topic echo /sahi_detection_info

# View structured results
ros2 topic echo /image_detections
```

---

## Troubleshooting

### Model Not Loading
- **Check model path**: Ensure `yolo11s.pt` exists in expected location
- **Check dependencies**: Run `pip install sahi ultralytics torch`
- **Try CPU**: If GPU fails, try `device:=cpu`

### No Detections
- **Lower confidence threshold**: Try `confidence_threshold:=0.10`
- **Check image quality**: Ensure images are clear and objects are visible
- **Adjust slice size**: Smaller slices (e.g., 256x256) for very small objects

### GPU Memory Issues
- **Enable cleanup**: `enable_gpu_memory_cleanup:=True`
- **Reduce batch size**: `max_images_per_cycle:=1`
- **Use smaller model**: Try `yolo11n.pt` (nano) instead of `yolo11s.pt` (small)

### Slow Processing
- **Use GPU**: Ensure `device:=cuda:0` or `device:=auto`
- **Reduce slice size**: Smaller slices process faster
- **Reduce overlap**: Lower overlap ratio (e.g., 0.2) processes faster

---

## File Structure

```
object_detection_sahi.py
├── Constants (lines 69-74)
├── get_detection_directory() (lines 77-120)
├── SAHIObjectDetectionNode class (lines 122-1244)
│   ├── __init__() - Initialization
│   ├── _load_and_validate_parameters() - Parameter setup
│   ├── waypoint_reached_cb() - Waypoint callback
│   ├── _parameter_callback() - Dynamic reconfiguration
│   ├── _get_statistics_service() - Statistics service
│   ├── _get_health_service() - Health service
│   ├── _periodic_gpu_cleanup() - GPU memory management
│   ├── _get_device() - Device detection
│   ├── initialize_sahi_model() - Model loading
│   ├── check_for_new_images() - Main processing loop
│   ├── _cleanup_old_images() - Image cleanup
│   ├── _process_image_safe() - Safe image processing wrapper
│   ├── process_image() - Single image processing
│   ├── detect_objects_sahi() - Core detection
│   ├── _categorize_detection() - Detection categorization
│   ├── _filter_detections() - Duplicate removal
│   ├── _apply_nms() - Non-Maximum Suppression
│   ├── _calculate_iou() - IoU calculation
│   ├── annotate_frame() - Image annotation
│   ├── publish_results() - Result publishing
│   └── _log_statistics() - Statistics logging
└── main() - Entry point (lines 1247-1281)
```

---

## Summary

The `object_detection_sahi.py` file implements a complete object detection system for aerial imagery. It:

1. **Monitors** a directory for new images
2. **Processes** images using SAHI + YOLO11
3. **Detects** small tents and people
4. **Annotates** images with bounding boxes
5. **Publishes** results to ROS2 topics
6. **Saves** annotated images to disk
7. **Tracks** statistics and health
8. **Supports** dynamic configuration

The system is designed to be robust, efficient, and easy to configure for different use cases.

