# ASTRA camera-to-detection code walkthrough

Updated October 31, 2026. This guide describes the implemented code and the
Jetson test results shared in this chat.

## 1. What the system does

The ZR10 camera provides an RTSP video stream. The Jetson decodes it, wraps
frames in ROS 2 Image messages, and publishes them. A separate ROS node
subscribes to those images and runs the repository's SAHI/YOLO inference code.
Detection outputs are published for other subscribers to consume.

Normal streaming and inference require no image files on disk. Saving a PNG
is an explicit snapshot service call.

```mermaid
flowchart LR
    A[ZR10 camera] -->|RTSP over TCP| B[FFmpeg decoder]
    B -->|Raw RGB bytes through a pipe| C[zr10_stream]
    C -->|/zr10/image_raw| D[astra_live_detection]
    D --> E[Latest pending frame worker]
    E --> F[SAHI slices + YOLO inference]
    F --> G[ROS result publication]
    G --> H[/detection/results]
    G --> I[/detection/annotated]
    G --> J[/detection/info]
```

## 2. Workspace and code map

Jetson workspace:

```text
/home/astra-dev/astra_camera_detection/astra_ws/
├── CODE_WALKTHROUGH.md
├── CAMERA_DETECTION.md
└── src/
    ├── zr10_stream/
    │   ├── scripts/frame_source.py
    │   ├── scripts/stream_node.py
    │   └── launch/stream.launch.py
    ├── astra_live_detection/
    │   ├── astra_live_detection/
    │   │   ├── live_node.py
    │   │   ├── latest_worker.py
    │   │   ├── model_manager.py
    │   │   ├── detection_processor.py
    │   │   ├── annotation.py
    │   │   └── gpu_utils.py
    │   ├── launch/camera_detection.launch.py
    │   └── UPSTREAM.json
    └── zr10_ros2/
        └── src/zr10_node.cpp
```

`frame_source.py`, `stream_node.py`, `live_node.py`, `latest_worker.py` and the
new launch files implement the live integration. `model_manager.py`,
`detection_processor.py`, `annotation.py`, and `gpu_utils.py` came from the
local ODLC GitHub clone with internal imports renamed and trailing whitespace cleaned in the model manager. `UPSTREAM.json` records
the original commit and file hashes. The existing weights and detection rules
were reused; no new model was trained.

`zr10_ros2` is the previously selected camera command package recovered from
the user's ZIP. The successful stream/detection test did not start it. The
original GitHub `detection` and `video_cam` packages are preserved.

Read code under `src/`. The `build/` and `install/` directories contain build
outputs and installed links, not the primary source files.

## 3. ROS terms in this project

| Term | Meaning | Our example |
| --- | --- | --- |
| Package | Installable collection of code and launch files | `zr10_stream` |
| Node | Running ROS component | `/zr10_stream` |
| Publisher | Sends messages on a topic | Camera node publishes images |
| Subscriber | Receives messages on a topic | Detection node receives images |
| Topic | Named stream of typed messages | `/zr10/image_raw` |
| Callback | Function invoked for an event | `receive()` runs on image arrival |
| Service | Named request/response operation | `/zr10/save_frame` |
| Parameter | Configuration value | `fps`, `model_path` |
| Launch file | Starts/configures nodes | `camera_detection.launch.py` |
| Executor | Runs ROS callbacks | `rclpy.spin(node)` |

A topic does not guarantee that a consumer processed each image. A service
response only confirms the behavior documented by that particular service.

## 4. Camera capture: frame_source.py

`FrameSource` owns a background reader thread and an FFmpeg subprocess.

1. `_run()` starts FFmpeg with `subprocess.Popen`, without a shell.
2. FFmpeg connects to RTSP and writes raw RGB frames to standard output.
3. A selector waits for readable bytes without blocking indefinitely on a read.
4. Partial reads accumulate until `width * height * 3` bytes are available.
5. A complete frame replaces `latest`, under a lock. It stores a sequence
   number, host monotonic arrival time and immutable image bytes.
6. EOF, process startup failure or ten seconds without a complete frame marks
   the source as failed, clears its latest frame, and retries after one second.

The stream node calls `get()` to obtain the latest frame. `close()` signals
shutdown and joins the reader. The reader terminates/reaps its decoder process,
escalating from termination to forced termination if necessary.

`png_bytes()` encodes RGB data as a PNG using Python's standard library.
It is used for snapshots, not the normal streaming path.

The bounded reader buffer does not eliminate latency inside the camera,
network, FFmpeg decoder, or ROS transport.

## 5. Camera publication: stream_node.py

`StreamNode.__init__()` declares startup parameters, validates them, creates
the image publisher and snapshot service, configures FFmpeg, starts a timer,
and starts the reader thread.

The FFmpeg arguments select RTSP over TCP, the first decoded output video
path, scaling/FPS filtering, and `rgb24` raw output. The configured dimensions
are output dimensions; scaling can stretch an image if its aspect ratio differs.

`fresh_frame()` rejects a missing frame or one older than
`max_frame_age_sec` (default two seconds, measured since host decode completion).

`publish_frame()` runs on the ROS timer:

1. Fetch a fresh frame.
2. Skip it if its sequence was already published.
3. Create `sensor_msgs/msg/Image` with `rgb8` encoding.
4. Set the host publication timestamp and `zr10_camera` frame ID.
5. Publish to `zr10/image_raw`.

At 1280 x 720, each frame contains 2,764,800 RGB pixel bytes. A configured
2 FPS means up to roughly two publications per second, not a guaranteed
end-to-end throughput or latency.

`save_frame()` saves the latest fresh frame with a unique PNG filename and
returns its absolute path. A relative `snapshot_directory` is resolved from the
process's working directory, not automatically from the workspace. Encoding
the PNG happens in the service callback and can briefly delay publication.

## 6. Detection subscription: live_node.py

`LiveDetection.__init__()` validates the model path/settings and loads the
model through the reused `load_sahi_model()`. Missing dependencies or a model
that cannot load prevent startup; no placeholder detector is substituted.

It creates the image subscription, three output publishers, three services,
the latest-frame worker and a result-publication timer.

`receive(message)` is deliberately short:

```python
self.last_received = time.monotonic()
self.worker.submit(message)
```

It records input arrival and hands off the message. Inference does not run
inside the subscription callback, keeping ROS callbacks available for more
images, service requests and publication.

## 7. Work scheduling: latest_worker.py

`LatestWorker` retains at most one waiting input, runs one inference at a time,
and retains at most one completed output awaiting publication.

Example: inference is processing frame A while B arrives, then C arrives.
C replaces pending B. After A finishes, C is processed. B is counted as replaced.
This bounds the application backlog; it does not guarantee every frame is used.
ROS and decoder buffers are separate from this single pending slot.

`_run()` drops a pending frame if its time waiting in the worker exceeds
`max_pending_age_sec`. That age check is not based on the image header and
does not impose a timeout on an already running inference.

`set_enabled()` changes an epoch counter and clears pending/completed work.
An old in-flight inference may finish, but its result is discarded if its epoch
no longer matches. Pausing does not stop camera publication or cancel GPU work.

`close()` waits for the current inference to return. It has no forced timeout
for a hung model call.

## 8. Inference and annotation

`LiveDetection.process(message)`:

1. Converts ROS RGB input to OpenCV `bgr8` through `CvBridge`.
2. Calls the reused `run_sahi_detection()`.
3. Measures conversion/inference duration.
4. Calls `annotate_frame()` to draw boxes/labels on a copy.
5. Returns the original input header, detections, annotated image and duration.

Inside `detection_processor.py`, SAHI converts BGR to RGB and divides the image
into overlapping slices. Defaults are 640 x 640 slices and 20% overlap in each
direction. YOLO predicts objects; SAHI merges slice predictions. The shared
code disables the additional full-image prediction pass.

The existing filters retain detections above confidence 0.3, area between
25 and 1,000,000 pixels, and width/height ratio between 0.1 and 10. Confidence,
slice size and overlap are startup parameters; area/aspect limits are currently
fixed arguments in the adapter.

The shared categorizer labels person/person-like, tent-like and other objects.
Tent-like categories are heuristics based on existing class mappings, not
proof a trained tent detector has identified a tent. The standard result message
retains the original YOLO class; JSON includes both that class and the category.

The `processing_sec` measurement excludes annotation, waiting in the worker,
transport and camera latency. It is not end-to-end latency.

## 9. Published message contracts

| Topic at root namespace | Type | Publisher | Subscriber/use |
| --- | --- | --- | --- |
| `/zr10/image_raw` | `sensor_msgs/msg/Image` | `zr10_stream` | Detector or image viewer |
| `/detection/results` | `vision_msgs/msg/Detection2DArray` | `astra_live_detection` | Structured detection consumer |
| `/detection/annotated` | `sensor_msgs/msg/Image` | `astra_live_detection` | Image viewer |
| `/detection/info` | `std_msgs/msg/String` | `astra_live_detection` | JSON inspection/logging |

The source uses relative names, so a namespace can prefix them. The launch
argument `image_topic` remaps both stream output and detection input together.

`publish_result()` checks for completed work every 0.05 seconds. That 20 Hz
check is not a claim of 20 FPS inference. It builds output messages and publishes
them only when a completed result exists. All outputs retain the input header.
An empty detection array represents successful inference with no retained objects.
An exception is reported as an error, rather than a fabricated empty success.

### Image

```yaml
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: zr10_camera
height: 720
width: 1280
encoding: rgb8
is_bigendian: 0
step: 3840
data: ...
```

Annotated output uses `bgr8`. `step` is the number of bytes per row.

### Structured detection

Each `Detection2DArray` contains a header and zero or more `Detection2D` entries.
Each entry includes a hypothesis and a bounding box, for example:

```yaml
results:
  - hypothesis:
      class_id: person
      score: 0.94
bbox:
  center:
    position: {x: 790.0, y: 248.0}
  size_x: 300.0
  size_y: 496.0
```

JSON boxes use `[x1, y1, x2, y2]`; ROS boxes use center plus width/height:

```text
center_x = (x1 + x2) / 2
center_y = (y1 + y2) / 2
size_x = x2 - x1
size_y = y2 - y1
```

These are image pixels, with image origin at the top left. The default 3D pose
fields are unset placeholders, not measured positions. No geographic location,
distance or camera calibration is inferred. A confidence score is a model score,
not a measured accuracy percentage. Detection IDs are not assigned; there is
no persistent object tracking in this implementation.

### JSON details

The ROS message is a String whose `data` field contains JSON:

```json
{
  "frame_id": "zr10_camera",
  "stamp": {"sec": 123, "nanosec": 456},
  "processing_sec": 0.43,
  "objects": [{
    "class": "person",
    "yolo_class": "person",
    "confidence": 0.94,
    "bbox": [640, 0, 940, 496],
    "description": "person",
    "method": "sahi+yolo",
    "area": 148800,
    "is_target": true
  }]
}
```

The ROS command line renders messages as YAML. It may truncate long strings;
use `--full-length` to see complete JSON. Consumers should parse `data` as JSON.

## 10. QoS and ROS environment

Camera publisher: reliable, volatile, depth one. Detection input: the ROS
sensor-data profile, which is best effort (its transport queue is separate
from the worker's one pending slot). A reliable publisher can serve a
best-effort subscriber. Detection outputs: reliable with depth one and default
volatile durability. Old results are not deliberately latched for new viewers.

The verified setup used:

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Domain ID 0 groups these ROS participants for discovery. It is not an IP address
or authentication mechanism. ROS network connectivity and compatible discovery
settings still matter. Both test terminals ran on the Jetson over SSH.

The tested camera input URL was the configured default:
`rtsp://192.168.144.25:8554/main.264`. The Jetson's SSH addresses
(`10.110.186.158` on the network, or `192.168.55.1` for the observed USB link)
are distinct from the camera address and may depend on network/interface state.

## 11. Services

| Name | Type | Request | Response |
| --- | --- | --- | --- |
| `/zr10/save_frame` | `std_srvs/srv/Trigger` | `{}` | Success plus saved PNG path, or failure reason |
| `/detection/set_enabled` | `std_srvs/srv/SetBool` | `{data: false}` / `{data: true}` | Pause/resume acknowledgment |
| `/detection/get_statistics` | `std_srvs/srv/Trigger` | `{}` | JSON counters/state |
| `/detection/get_health` | `std_srvs/srv/Trigger` | `{}` | JSON state and input/result age |

The detector services exist only while the detector node is running. Health
success requires a live worker, no current worker/publication error, and input
received within five seconds. It can succeed while paused and does not prove
recent successful inference; inspect `result_age_sec` and `processed` too.

| Counter/field | Meaning |
| --- | --- |
| `received` | Image callbacks received, including while paused |
| `replaced` | Waiting inputs overwritten by newer arrivals |
| `processed` | Successful worker calls, including results later discarded during pause |
| `errors` | Worker exceptions; publication failures are reported separately |
| `stale` | Pending inputs discarded for excessive worker waiting time |
| `enabled` | Whether new inference work is accepted |
| `pending` | Whether an input is waiting, not whether inference is in flight |
| `last_error` | Most recent worker error, cleared after a successful call |
| `publish_error` | Current output conversion/publication error, health response only |

Snapshot saves are stream-resolution PNGs. Full-resolution camera SD-card photos
use the separate camera command node and media-download implementation.

## 12. Launch modes and copy/paste commands

Source this in every Jetson terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/astra_camera_detection/astra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

Publisher only (the mode requested after the walk-through):

```bash
ros2 launch zr10_stream stream.launch.py fps:=2.0
```

Camera plus detector:

```bash
ros2 launch astra_live_detection camera_detection.launch.py \
  model_path:="/home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/detection/detection/yolo11s.pt" \
  device:=cuda:0 fps:=2.0
```

Detector only, subscribing to a camera publisher already running:

```bash
ros2 launch astra_live_detection camera_detection.launch.py \
  start_stream:=false image_topic:=/zr10/image_raw \
  model_path:="/home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/detection/detection/yolo11s.pt" \
  device:=cuda:0
```

Run only one camera publisher and one detector for this test. Stop a combined
launch before replacing it with publisher-only mode. Stopping a separate
detector-only launch leaves the separate publisher running.

Observe results in another terminal:

```bash
ros2 topic info /zr10/image_raw --verbose
ros2 topic info /detection/info --verbose
ros2 topic echo /detection/info std_msgs/msg/String --full-length
```

Service examples:

```bash
ros2 service call /detection/get_health std_srvs/srv/Trigger '{}'
ros2 service call /detection/get_statistics std_srvs/srv/Trigger '{}'
ros2 service call /detection/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /detection/set_enabled std_srvs/srv/SetBool '{data: true}'
ros2 service call /zr10/save_frame std_srvs/srv/Trigger '{}'
```

On a graphical Jetson desktop, `rqt_image_view` can display
`/detection/annotated`. A plain SSH terminal does not itself provide a graphical
display. The pipeline does not automatically save annotated frames.

## 13. What was verified on the Jetson

Evidence comes from the user's build output, service responses and topic logs.

| Check | Observed outcome |
| --- | --- |
| ROS package build | Stream and live detection packages built |
| Dependencies | `rclpy`, `cv_bridge`, PyTorch, SAHI and Ultralytics imported |
| GPU availability | `torch.cuda.is_available()` returned true |
| Model startup | Loader reported PyTorch model loaded via SAHI with `device:=cuda:0` |
| Camera-to-detector flow | Image publisher/subscriber discovered; received/processed counters advanced |
| Detection outputs | `Detection2DArray` and full JSON received in another terminal |
| Health/statistics | Responses succeeded; sampled worker and publication errors were zero |
| Snapshot | Success returned `/home/astra-dev/stream_snapshots/frame_f32dadb60e1740c281ef761960c0b5ba.png` |
| Duplicate launches | Two complete launches were found; older launch and children exited; one complete launch remained |
| Repeat walk-through | Initially no person predictions, then repeated high-scoring person predictions, then no person predictions near the end |

The final pasted walk-through sample showed person scores around 0.92–0.95
during the central visible interval and processing times mostly 0.40–0.45 seconds.
Its timestamps no longer repeated as they had in the duplicate-launch run.
These are observations from a short test, not an accuracy or latency benchmark.
Some frames included an extra partial person box; visual review is still needed.

The source executable bit was initially missing, causing ROS launch to reject
`stream_node.py` despite a successful build. `chmod +x` corrected the Jetson
source and the local downloadable ZIP was fixed too.

Eight local camera-free tests passed: four reader/PNG tests and four worker
tests. The separate real-ROS synthetic-image smoke-test script exists but has
not been reported as run; manual Jetson tests above are separate evidence.

## 14. Remaining work and scope

* Visually inspect annotated images and evaluate box/class accuracy, including
  extra partial person boxes and other unexpected labels.
* Manually verify pause/resume on the Jetson; it is covered by local worker tests
  but has not been confirmed in the user's hardware session.
* Open the saved snapshot and verify its visual content; service save success
  alone does not establish image quality.
* Test camera network loss/recovery and orderly shutdown on the Jetson.
* Measure sustained rates, dropped/replaced frames, memory and end-to-end latency
  under the intended flight conditions.

The live pipeline has no GPS association, camera calibration, 3D localization,
persistent object tracking, flight control or payload actuation. The original
mission controller expects `/image_detection` with a custom
`ultralytics_ros/ImageResult` schema. It is not connected to the standard result
topic merely by matching a topic name.

The combined launch can optionally start `zr10_ros2` camera command control
with `start_camera_control:=true`; its `dry_run` defaults to true. Camera
motion/photo/recording commands were not part of the verified live test.
Avoid launching competing command controllers against the same camera.

Startup parameters are read once by these nodes. Use the detection enable
service for runtime pause/resume; changing an advertised parameter value alone
does not reconfigure the decoder or reload the model.

The code and documentation changes remain local; this chat has not committed
or pushed them to GitHub.
