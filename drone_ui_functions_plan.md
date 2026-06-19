# Autonomous Drone Flight Controller - UI Function Specification & System Analysis

This document provides a comprehensive analysis of the ROS 2 packages and nodes in the drone flight controller workspace, lists all parameters and arguments for each node, and defines the specifications of functions/controls that can be exposed through a Graphic User Interface (UI).

---

## 1. System Architecture & Package Overview

The project is a ROS 2-based autonomous drone payload-drop and mapping system. The codebase is divided into several packages:

| Package | Node / Executable | Language | Purpose |
| :--- | :--- | :--- | :--- |
| **`main`** | `main_controller` | Python | Coordinates mission logic: subscribes to target detections, controls flight modes (AUTO, GUIDED, RTL), manages guided holds, dynamically commands payload servo releases, and enqueues waypoints. |
| **`video_cam`** | `siyi` (`siyi_node.py`) | Python | Driver for the SIYI A8 gimbal camera. Supports video streaming, gimbal control (yaw/pitch speeds, mode, centering), zoom (manual, auto, range, current), and capturing geotagged photos. |
| **`detection`** | `new_od` (`new_od.py`) | Python | Performs object detection (targets: `person` and `tent`) on the captured camera images using a YOLO model with optional Slicing Aided Hyper Inference (SAHI) for small/highly zoomed targets. |
| **`mapping`** | `trigger` (`do_digi_cam_trigger.py`) | Python | Listens to Pixhawk camera trigger commands (`DigiCamCtrl`) and publishes manual capture triggers at 1Hz until the mission's buffer waypoint is reached. |
| | `mapping` (`mapping.py`) | Python | Stitches captured mapping images incrementally using ORB or SIFT keypoints and affine/homography partial estimation to produce a composite orthomosaic panorama. |
| **`waypoint_mavros`** | `waypoint` (`waypoint.py`) | Python | Interfaces with MAVROS/APM to manage the drone's mission waypoints, support dynamic waypoint addition/deletion, and set flight modes. |
| | `killnode` (`killnode.py`) | Python | Monitors shutdown events, copies captured photos from temporary folders to persistent directories, and shuts down the onboard computer. |
| **`gps_ros2`** | `gpstest` (`gps.py`) | Python | Subscribes to MAVROS state, global position, and compass heading. Exposes a service `/get_drone_data` to share GPS/Yaw state with other nodes. |
| **`wp_sender`** | `waypoint_client` / `parameter_manager` | Python | Utility package helper scripts for adding/deleting waypoints and querying parameters from the MAVROS waypoint manager. |
| **`interfaces`** | Custom Messages & Services | C++ / Python | Defines custom data structures: `ImageResult`, `YoloResult` messages, and `AddWaypoint`, `DelWaypoint`, `GetGPSData`, `CameraCommand`, `UpdateMission` services. |

---

## 2. Comprehensive Parameter & Argument Directory

Each node exposes custom ROS 2 parameters that tune behavior, algorithms, camera settings, and limits. These parameters can be read or modified dynamically or at start-up.

### A. `new_od` (Object Detection Node)
* **`model_path`** (string, default: `'yolo26m.engine'`): Path to the YOLO model file (TensorRT engine `.engine` or ONNX/PyTorch file).
* **`model_format`** (int, default: `MODEL_FORMAT_AUTO`): Format format of the model.
* **`auto_convert_tensorrt`** (bool, default: `True`): Auto-convert ONNX to TensorRT `.engine` on start-up.
* **`tensorrt_workspace`** (int, default: `4`): TensorRT workspace limit (in GB).
* **`confidence_threshold`** (float, default: `0.15`): Threshold above which detections are counted.
* **`slice_height` / `slice_width`** (int, default: `512`): Patch sizes for SAHI sliced inference.
* **`overlap_height_ratio` / `overlap_width_ratio`** (float, default: `0.3`): Overlap ratio between adjacent slices.
* **`check_interval`** (float, default: `2.0`): File polling rate (in seconds) to check for new images.
* **`device`** (string, default: `'auto'`): Compute hardware (options: `'cuda:0'`, `'mps'`, `'cpu'`).
* **`max_images_per_cycle`** (int, default: `5`): Maximum images processed in a single evaluation tick.
* **`max_camera_feed_images`** (int, default: `10000`): Max storage buffer size for images.
* **`min_detection_area` / `max_detection_area`** (int, default: `25` / `1000000`): Filter out bounding boxes by pixel area bounds.
* **`min_aspect_ratio` / `max_aspect_ratio`** (float, default: `0.1` / `10.0`): Aspect ratio limits of detected objects.
* **`enable_gpu_memory_cleanup`** (bool, default: `True`): Force clean CUDA cache on cycles.
* **`camera_feed_path`** (string, default: `''`): Path to directory where raw camera frames are stored.
* **`detection_results_path`** (string, default: `''`): Path to save annotated result images.
* **`use_batched_inference`** (bool, default: `False`): Run sliced inference in batches.
* **`batch_size`** (int, default: `8`): Sliced inference batch size.

### B. `siyi` (Camera Driver Node)
* **`use_real_camera`** (bool, default: `True`): Connect to a physical SIYI A8 camera. False activates simulation fallback (reading `/camera/image`).
* **`min_altitude_agl`** (double, default: `0.0`): Min relative altitude (m) required to activate the camera (prevents capturing ground frames).
* **`camera_ip`** (string, default: `'192.168.144.25'`): RTSP and control IP of the camera.
* **`ctrl_port`** (int, default: `80`): TCP control port.
* **`media_port`** (int, default: `8554`): RTSP video streaming port.
* **`http_timeout_sec` / `capture_timeout_sec`** (double, default: `5.0` / `10.0`): Network timeouts for downloads and commands.
* **`min_free_space_mb`** (int, default: `100`): Minimum disk space required to capture.
* **`resolution`** (string, default: `'4K'`): Capture photo dimensions (choices: `'4K'`, `'2.7K'`, `'1080P'`).
* **`rotate_180`** (bool, default: `False`): Flip frames upside down (needed if mounted inverted).

### C. `waypoint` (MAVROS Waypoint Manager Node)
* **`num_waypoints`** (int, default: `0`): Total waypoints in the current mission.
* **`takeoff_index`** (int, default: `-1`): Mission item index of the takeoff command.
* **`rtl_index`** (int, default: `-1`): Mission item index of the Return-To-Launch command.
* **`next_after_takeoff`** (int, default: `-1`): First navigation waypoint index.
* **`last_before_rtl`** (int, default: `-1`): Final navigation waypoint index before RTL.
* **`buffer_wp`** (int, default: `-1`): Target waypoint index for the guided-hold processing pause.

### D. `mapping` (Orthomosaic stitching node)
* **`use_sift`** (bool, default: `False`): Use SIFT algorithm (fallback is ORB keypoints).
* **`downscale_factor`** (float, default: `0.5`): Rescaling ratio of inputs (e.g. 0.5 reduces memory usage).
* **`max_frames`** (int, default: `150`): Max frames stitched in a single run.
* **`min_matches`** (int, default: `6`): Minimum matching descriptors required.
* **`ratio_test`** (float, default: `0.8`): Nearest-neighbor distance ratio threshold.
* **`blend_method`** (string, default: `'distance'`): Blending algorithm (options: `'distance'`, `'multiband'`).
* **`ransac_thresh`** (float, default: `3.0`): Maximum reprojection threshold for partial affine.
* **`max_canvas_width` / `max_canvas_height`** (int, default: `8192`): Stitching canvas boundaries.
* **`save_intermediates`** (bool, default: `False`): Save intermediate panoramas at steps.
* **`intermediate_interval`** (int, default: `10`): Number of frames between intermediate saves.
* **`input_directory` / `output_directory`** (string, default: `''`): Source and destination directories.

---

## 3. UI Function Map (What the UI Can Control)

This maps code endpoints (topics, services, parameters) to specific user interface controls. The UI should organize these functions into categories.

### Category 1: Global Process & Launcher Controls
These actions control the execution state of the ROS 2 lifecycle and stack.

* **Launch Full Mission Stack**: Triggers `launcher.sh` which launches:
  * `mavros` (Pixhawk link)
  * `main` package tracker (waypoint, gps, siyi camera, and trigger nodes).
* **Launch/Restart Individual Nodes**:
  * Run Object Detection: `ros2 run detection new_od` (with parameter overrides)
  * Run Stitching Mapping: `ros2 run mapping mapping`
  * Run SIYI Camera Driver: `ros2 run video_cam siyi`
  * Run Waypoint Manager: `ros2 run waypoint_mavros waypoint`
* **Shutdown Stack / Jetson Node**:
  * Publish to `/camera/command` (`"sd_format yes"` or custom shutdown message) or invoke a shutdown command to `Kill_node` which triggers the safe transfer of photos (`shutil.move` from `/mapping_photos` to `/camera_feed`) and runs `sudo shutdown -h now`.

### Category 2: Flight Mode & Waypoint Controls
Actions that interact with `waypoint` node and MAVROS.

* **Set Flight Mode**: (Combobox to select mode, invokes service client `/mavros/set_mode`)
  * `AUTO` (resumes autonomous mission flight)
  * `GUIDED` (hovers in place to process images or wait for user inputs)
  * `RTL` (abrupt return-to-launch home)
* **Waypoint Mission Editor**:
  * **Fetch Mission indices**: Pulls current values for `takeoff_index`, `rtl_index`, `next_after_takeoff`, `last_before_rtl`, and `buffer_wp`.
  * **Add Waypoint**: Invokes `/addWaypoint` (`AddWaypoint.srv`) with `latitude`, `longitude`, `altitude`, `index`, and command code.
  * **Delete Waypoint**: Invokes `/delWaypoint` (`DelWaypoint.srv`) with the index.
  * **Push/Pull Mission**: Triggers `/mavros/mission/push` and `/mavros/mission/pull` to sync with the physical Pixhawk flight controller.
  * **Insert Drop Coordinates**: Manually push drop mission items (NAV_WAYPOINT + DO_SET_SERVO) if machine inferencing fails.

### Category 3: Camera & Gimbal Controls
Actions that write directly to the `/camera/command` (`std_msgs/msg/String`) topic as JSON or text commands.

* **Manual Shutter Trigger**: Publishes `True` to `/camera/trigger`.
* **Zoom Controls**:
  * Zoom In/Out/Stop: Send `zoom_manual in`, `zoom_manual out`, or `zoom_manual stop`.
  * Absolute Zoom: Send `zoom_absolute <multiplier>` (e.g. `zoom_absolute 4.5`).
  * Get Current Zoom: Send `zoom_current` (reads feedback).
* **Gimbal Direction Controls**:
  * Manual Rotate: Send `gimbal_rotate <yaw_speed>,<pitch_speed>` (e.g. `gimbal_rotate 15,-20`).
  * Center Gimbal: Send `gimbal_center`.
  * Stop Gimbal: Send `gimbal_stop`.
  * Set Angles: Send `gimbal_set_angles <yaw_deg>,<pitch_deg>`.
  * Set Axis Angle: Send `gimbal_set_axis <yaw|pitch|roll>,<deg>`.
  * Get/Set Gimbal Mode: Send `gimbal_mode_get` or `gimbal_mode_set <lock|follow|fpv>`.
* **Focus Controls**:
  * Autofocus: Send `autofocus` or `autofocus <touch_x>,<touch_y>`.
  * Manual Focus: Send `focus_manual <far|near|stop>`.
* **SD Card Control**:
  * Format SD Card: Send `sd_format yes`.

### Category 4: Object Detection & SAHI Configuration
Controls that modify `new_od` parameters on-the-fly.

* **Detection Switch**: Enable/disable active inferencing.
* **YOLO Confidence Slider**: Adjust `confidence_threshold` (0.0 to 1.0).
* **SAHI Patch Control**: Adjust `slice_height` and `slice_width` sliders (useful to detect smaller objects from high altitudes).
* **Device Selector**: Toggles device string between `'cuda:0'` (GPU for speed) and `'cpu'` (for power saving or fallback).
* **Min/Max Detection Size filter**: Adjust `min_detection_area` and `max_detection_area` to filter out non-target objects (like birds, trees).

### Category 5: Incremental Mapping & Stitching Controls
Interactions with the `mapping` node.

* **Stitcher Control**: Starts `mapping` node stitching.
* **Algorithm Selector**: Toggle between SIFT (`use_sift = True`) and ORB (`use_sift = False`).
* **Scale down slider**: Adjust `downscale_factor` to manage Jetson RAM usage.
* **Blend Selector**: Choose between `distance` and `multiband` blending.
* **Save Intermediate Images**: Checkbox for `save_intermediates`.

---

## 4. Suggested additions for the UI (Missed Features)

These features are not fully exposed in the current scripts but would make the user interface premium, interactive, and functionally complete.

### A. Live Video & Overlay Panel
* **Live RTSP Camera Feed**: Stream raw video from the SIYI camera (`rtsp://192.168.144.25:8554/main.264`).
* **Live Bounding Box Overlay**: Show the real-time detections (`/sahi_detection_results` or `/image_raw`) on top of the live feed.
* **Gimbal Virtual Joystick**: A touchscreen virtual joystick that publishes `gimbal_rotate` speeds on drag.
* **Focus Point Selection**: Allow users to click on the video feed to set focus coordinates (calling `autofocus <touch_x>,<touch_y>`).

### B. Map & Target Plotting Dashboard
* **GPS Plot Map**: Visual map (using OpenStreetMap or similar offline mapping) showing:
  * The drone's live coordinate location (from `/mavros/global_position/global`).
  * The mission flight path (rendered from `/mavros/mission/waypoints`).
  * Real-time detected object markers: Plot `person` and `tent` coordinates as custom pins instantly when published to `/image_detections` (`ImageResult.msg`).
* **Coordinates Target Override**: If the user sees a target visually on the map that the model missed, let them click and drop a custom waypoint pin, triggering a `/addWaypoint` service call to queue a payload release.

### C. Drop Mechanism Control & Telemetry
* **Servo Manual Release Buttons**: Buttons to manually trigger `CommandLong` (command 183) for channels 9 (bottle drop) and 10 (beacon drop) with high PWM (1900μs) or low PWM (1400μs).
* **Payload Status Indicators**: Displays if payload has dropped or is still locked, based on whether the main controller has completed the waypoint-reached drop sequence.
* **AGL Altitude Alert**: A telemetry widget showing the AGL altitude (`/mavros/global_position/rel_alt`) with a warning color if the drone falls below the `min_altitude_agl` (camera disabled).

### D. System Diagnostics & Resource Monitors
* **Jetson Performance Widget**: Displays GPU/CPU load, RAM usage, and core temperatures.
* **Disk Space Alert**: Integrates `/camera/disk_free_mb` data to show remaining storage with a warning if it approaches `min_free_space_mb`.
* **MAVROS Heartbeat Indicator**: A status light (Green/Red) indicating Pixhawk connection state (derived from `/mavros/state` subscription).
* **GCS Status Logger**: Text console displaying status text acknowledgments (`/mavros/statustext/send` and `recv`).
