# ASTRA Photo Booth

The browser viewer connects to the existing ROS camera and detection topics. The
Jetson runs the ROS pipeline and viewer; other devices on the same reachable network
open the Jetson's address in a browser. Client computers do not need ROS installed.

## Current branch status

The pushed GUI baseline has Live, Photos, and Split views and saves captures to the
Jetson. The local follow-up draft on `shrihan/photo-booth-feedback` adds the
Controller view, browser-memory recent captures, Camera SD browsing, selected-photo
downloads, and a capped low-resolution live preview. The follow-up is uncommitted,
unpushed, and has not been Jetson-tested. See [PHOTO_BOOTH_TESTING_NOTES.md](../PHOTO_BOOTH_TESTING_NOTES.md)
for its test plan and limits.

In the follow-up draft, recent photos are held separately in each browser tab's
memory and disappear when that tab closes. Camera SD photos remain on the camera
and load on demand. The viewer does not write a photo archive to the Jetson. The
Controller defaults to unavailable until the ROS camera node is running with live
commands enabled and the viewer starts with `--enable-controller`. The browser also
requires an explicit enable action. Do not enable real controls until you are ready
to test the camera safely.

## Run a UI demo on macOS, Windows, or Linux

From the repository root:

```bash
python3 -m pip install -r astra_viewer/requirements.txt
python3 -m astra_viewer.server --demo
```

On Windows, use `py` in place of `python3`. Open `http://localhost:8080`. Demo
images and detections are synthetic; camera controls and Camera SD browsing are not
available in demo mode.

## Jetson prerequisites

The camera and detector must publish `/zr10/image_raw` and `/detection/info`. Build
the updated ROS packages from this checkout and source the resulting workspace. For
the local repository workspace, the build starts in `ros2_ws`:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to astra_live_detection zr10_ros2 --symlink-install
source install/setup.bash
```

Start the camera/detection pipeline once using the model path and device settings
that already work on this Jetson. The launch defaults do not start the control node;
it can be included with `start_camera_control:=true` and remains in `dry_run:=true`
unless explicitly changed. The viewer itself runs separately:

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 -m astra_viewer.server --camera-ip 192.168.144.25
```

Install Pillow if needed (`sudo apt-get install python3-pil`). On another device,
open `http://JETSON_ADDRESS:8080`, using the Jetson's reachable network address,
not the camera's address. Port 8080 must be allowed between devices.

To expose the Controller UI, start the viewer with `--enable-controller` and run
the updated ROS camera control node with `dry_run:=false`. That setting sends real
commands to the camera. Start with the camera secured and test only brief movements.
The UI's motion watchdog sends stop commands, but UDP sends do not confirm what the
camera executed. See the test notes before enabling live controls.

## Controls and limits

The draft exposes the camera functions currently wired into our ROS driver: gimbal
movement, relative and absolute zoom, manual and point autofocus, autofocus, photo,
center, and recording toggle. The UI cannot confirm camera acknowledgements or
recording state. Some ZR10 functions and settings are not implemented. SD-card
browsing depends on the camera media API and still needs hardware verification.

The browser preview defaults to a maximum of 5 FPS at 960 by 540. Detection boxes
come from the latest inference result and may lag the moving video. The actual
detector rate and compute load are unchanged by this viewer preview cap. These
defaults are not a measured smoothness or power guarantee.
