# ASTRA Photo Booth

A local-network browser interface for the existing ZR10 → ROS 2 → SAHI/YOLO pipeline.
The Jetson runs the camera, detector and this viewer. Other computers open the same
address in a browser. Viewing and downloading work without installing ROS on those computers.
The demo server also runs on Windows, macOS and Linux with Python 3.10+ and Pillow.

## Test the UI on your Mac now

From the folder containing `astra_viewer`:

```bash
python3 -m astra_viewer.server --demo
```

Open **http://localhost:8080**. If Pillow is missing, install with
`python3 -m pip install -r astra_viewer/requirements.txt`. For Windows, use `py`
in place of `python3`. Demo images/detections are synthetic and clearly labeled;
they go to a separate archive, never mixed with camera photos.

## Run on the Jetson with your existing camera/detection installation

Copy this release folder to the Jetson and extract it to `~/astra_photo_booth`.
The release contains `astra_viewer/` plus the existing `astra_ws/` packages.
No model weights are included. **Keep one camera publisher and one detector running.**

If the previously tested `~/astra_camera_detection/astra_ws` installation is still
available, no ROS rebuild is needed. Install the viewer's one additional dependency:

```bash
sudo apt-get install python3-pil
```

**Terminal 1 — existing camera and detection:**

```bash
source /opt/ros/humble/setup.bash
source ~/astra_camera_detection/astra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch astra_live_detection camera_detection.launch.py \
  model_path:="/home/astra-dev/Documents/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/detection/detection/yolo11s.pt" \
  device:=cuda:0 fps:=2.0
```

Skip Terminal 1's launch if that same pipeline is already running. Adjust the model
path if it has moved. Camera-only mode does not produce detection messages for this UI.

**Terminal 2 — viewer:**

```bash
source /opt/ros/humble/setup.bash
source ~/astra_camera_detection/astra_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
cd ~/astra_photo_booth
python3 -m astra_viewer.server
```

The viewer prints its archive path and listening port. On the Jetson open
**http://localhost:8080**. On your Mac, Windows or Linux computer on the same
network open **http://JETSON_LAN_IP:8080**. Run `hostname -I` on the Jetson to find
its current LAN address; the old address may have changed. Use the Jetson's address,
not the ZR10 camera's address. Port 8080 must be reachable between the computers;
some guest/campus Wi-Fi networks block device-to-device connections.

The default listener is `0.0.0.0`, allowing LAN clients. Use `--host 127.0.0.1`
for access only from the host computer. This version is for a trusted local network,
with no sign-in; do not forward it to the internet.

### Fresh ROS installation / new ASTRA checkout

The local ASTRA repository uses `ros2_ws/`, while the downloadable release uses
`astra_ws/`. In the applicable workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/zr10_stream src/astra_live_detection --ignore-src -r -y
colcon build --packages-select zr10_stream astra_live_detection --symlink-install
source install/setup.bash
```

Use this workspace's `install/setup.bash` in both terminals instead of the old
workspace path. Keep the existing working Jetson PyTorch/SAHI/Ultralytics environment.
The viewer does not install or change those dependencies. See the included
`CAMERA_DETECTION.md` and `CODE_WALKTHROUGH.md` for the original integration.

## Views and browsing

- **Live:** annotated live feed, with newest thumbnails on the right.
- **Photos:** starts on the latest saved photo. Browse with thumbnails or arrow
  buttons/left-right keys; download the full annotated stream-resolution JPEG.
- **Split:** live feed and selected photo side by side; panels stack on narrow windows.
- Selecting a photo pauses **Follow latest**; new arrivals do not change your selection.
  **Follow latest** returns to the newest photo and scrolls the strip to the right.
- **Protect** keeps a photo from automatic cleanup, including after a service restart.
- **Load older** loads older pages. While browsing the history window, the gallery
  stays in place; Follow latest resumes incoming thumbnails. At most 240 thumbnails
  are held in the browser. Photos are still stored on the Jetson if a browser closes.

## Capture, annotation and storage behavior

The viewer matches `/zr10/image_raw` images to `/detection/info` JSON by the exact
ROS header timestamp. It uses your existing detections and does not run another
model. Detection can finish slower than video: the displayed annotated frame rate
is therefore the detection output rate, with a browser refresh ceiling near 2 FPS.
This first version favors correct image/box alignment over displaying boxes from
an older image on a newer video frame.

Every second, the viewer saves the newest fresh matched result, **including frames
with no detections**. It will not save the same frame twice or fabricate photos
while the source is paused/offline. If detection is slower than one result per
second, fewer unique photos can be saved; the source must supply fresh results to
achieve a sustained one-photo-per-second cadence. No backlog is accumulated.

Annotations are baked into the saved JPEG and download:

- Each box: object class and confidence.
- Top left: source ROS timestamp in UTC (host publication time in the existing
  camera node; not verified camera exposure time).
- Top right: highest object confidence, or **NO DETECTIONS**.
- Bottom left: camera name and frame reference, also shown above the selected photo.

The UI displays times in the viewing computer's local timezone. A confidence score
is a model score, not measured detection accuracy. No GPS/altitude is invented.
Snapshots use the video-stream resolution, not full-resolution SD-card photos.

Defaults: **10 GiB photo archive**, **2 GiB minimum free disk space**, one-second
capture interval. JPEG and thumbnail sizes count toward the photo limit; SQLite
metadata and other system files use additional space. Settings can be changed:

```bash
python3 -m astra_viewer.server --max-gb 20 --reserve-gb 4 \
  --archive ~/astra_photos --camera ZR10 --interval 1
```

When a save would cross either limit, automatic cleanup removes **unprotected** photos:
1. Photos with no detections, oldest first.
2. Photos whose highest object score is below 50%, oldest first.
3. Photos scoring 50–79%, oldest first.
4. Photos scoring 80% or higher, oldest first, only if still needed.

Thus a photo with one strong detection is retained longer even if it contains
other low-score detections. Cleanup permanently removes the JPEG and thumbnail;
protect important captures. If protected photos fill the limit, saving stops with
an on-screen message instead of removing them. Cleanup runs on capture attempts;
it cannot free unrelated files or guarantee a free-space reserve against other apps.
Only run one viewer process per archive directory.

Custom topic names: `--image-topic /your/image --info-topic /your/info`.
The existing camera node should run at 2 FPS or higher; exact matching keeps a
bounded 24-frame cache. Exceptionally slow detection may require lowering the
camera publish rate to keep its corresponding image in that cache.

## Verification

```bash
python3 -m unittest discover -s astra_viewer/tests -v
node --check astra_viewer/static/app.js
```

Local tests cover cleanup priority, protected-photo persistence, disk pressure,
pagination, duplicate/stale capture suppression, timestamp matching, annotations,
and background capture. Browser checks cover views, older-photo selection,
protection, downloads and narrow/desktop layouts. The ROS adapter and actual LAN
access still need testing on your Jetson; ROS is unavailable on this Mac.

Suggested Jetson check: start camera/detection once, open the viewer on two computers,
walk through the camera view, inspect/download a photo, protect an older photo,
then stop/restart detection and confirm the feed status and capture behavior.
