# Live camera input for detection

The recovered `astra_ws.zip` is the previously selected camera workspace.
Its `zr10_ros2` sources are preserved. The new companion `zr10_stream` package
decodes the camera's RTSP stream directly to a ROS image topic. It does not
wait for an SD-card download or scan a folder before delivering frames.

## Jetson setup

Use the Jetson's existing compatible ROS installation (Humble shown):

```bash
source /opt/ros/humble/setup.bash
cd ~/astra_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select zr10_stream --symlink-install
source install/setup.bash
ros2 launch zr10_stream stream.launch.py
```

FFmpeg must be installed (`sudo apt-get install ffmpeg`). Build/run the
existing `zr10_ros2` package separately using its supplied instructions if
camera commands or full-resolution photo downloading are needed. The stream
node itself sends no gimbal, recording, or photo commands. Its launch does not
start the camera command node. Do not run competing command controllers.

## Detection contract

* Default topic: `/zr10/image_raw`, type `sensor_msgs/msg/Image`.
* Encoding: `rgb8`; default output: 1280 x 720 at up to 10 frames/second.
* Header stamp: host publication time, **not camera exposure time**.
* Frame ID: `zr10_camera`; no calibration or geographic position is implied.
* Publisher: reliable, volatile, depth 1. A detector can subscribe with reliable
  or best-effort reliability and should use depth 1.
* Decode runs in a separate thread, retaining one latest complete frame.
  Publication skips replaced frames. Decoder and network buffering can still
  add latency; this has not been measured on the camera.
* The detector should retain one latest pending image, run inference outside
  its subscription callback, and retain the input header on its output. That
  prevents a slow model from accumulating an unbounded inference queue.

For a detector already subscribing to `/camera/image_raw`:

```bash
ros2 launch zr10_stream stream.launch.py image_topic:=/camera/image_raw fps:=5.0
```

The local GitHub clone has since been located. Its shared SAHI/YOLO inference
code is now connected through `astra_live_detection`. See
[the integrated camera/detection guide](CAMERA_DETECTION.md) for launch commands,
topics, services and remaining Jetson acceptance checks.

## Images while streaming

```bash
ros2 service call /zr10/save_frame std_srvs/srv/Trigger '{}'
```

This writes the latest fresh streamed frame as a PNG and returns its absolute
path. The response fails when no fresh frame exists. These snapshots have the
configured stream resolution; they are distinct from full-resolution SD-card
photos obtained through the original camera node's `take_photo` and media sync.
Saving a frame can briefly delay publication because the service encodes PNG in
the ROS executor. It is intended for occasional snapshots, not continuous logging.
No files are saved during ordinary streaming.

Configurable startup arguments: `rtsp_url`, `width`, `height`, `fps`, `frame_id`,
`image_topic`, `snapshot_directory`, and `max_frame_age_sec` (default 2).
Choose a width/height with the camera's aspect ratio to avoid stretching.
The decoder reconnects after EOF/error or ten seconds without a complete frame,
with a one-second retry delay. Shutdown terminates and reaps its decoder child.
FFmpeg diagnostics are suppressed; recurring stream warnings indicate an input
failure. Check RTSP reachability with FFmpeg separately if troubleshooting.

## Verification and remaining acceptance checks

Camera-free reader tests:

```bash
python3 src/zr10_stream/tests/test_frame_source.py
```

These exercise fragmented pipe reads, latest-frame replacement, stale-frame
discard, reconnection, missing FFmpeg, bounded shutdown, and real FFmpeg
synthetic-video decoding plus PNG pixel/CRC validation.

ROS/Jetson acceptance checklist (current results are recorded in
[CODE_WALKTHROUGH.md](CODE_WALKTHROUGH.md)):

1. Build the package; run `ros2 topic hz /zr10/image_raw` and inspect the image
   with an image viewer. Confirm color order and orientation.
2. Save a frame during streaming and open the returned PNG.
3. Disconnect/reconnect camera networking; verify warnings, rejected stale
   snapshots, recovery and no repeated stale publications.
4. Start the actual detector against the configured image topic; verify its
   outputs use the input header and measure end-to-end latency/memory under load.
5. Stop with Ctrl+C and verify no decoder remains.

Manual Jetson testing has since confirmed live frames reaching inference and
actual detection result publication. Camera-free tests alone did not establish
that; the subsequent hardware observations are documented in the walkthrough.

ROS references: [Image message definition](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Image.html)
and [QoS compatibility](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html).
