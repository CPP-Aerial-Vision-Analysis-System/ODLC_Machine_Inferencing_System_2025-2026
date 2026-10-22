# main: the mission controller

`src/main_controller.py` (ROS node `main_controller`) is the part of our code that takes over at the
end of the search: ArduPilot flies the survey by itself, and when it reaches the last survey waypoint
this node

1. switches to **GUIDED** (hover) and waits until the detector has checked every photo (at most 90 s),
2. picks the best **tent** and the best **person** (highest confidence),
3. inserts one waypoint over each of them before the mission's RTL (`/addWaypoint` of
   `waypoint_manager`), points the mission at the first one and switches back to **AUTO**,
4. over each target hovers 6 s, opens that target's release servo for 3 s (servo 13 = tent,
   11 = person) and flies on; after the last one waits 6 s and returns home (**RTL**).

It is started by `launch/tracker.launch.xml` (through `./launcher.sh`).

## What changed in October 2026 (found in the Gazebo simulator)

| Problem | What you saw | What it does now |
|---|---|---|
| **Drop aim** | The payload landed where the DRONE was when the photo was taken: 14-25 m from the target. | It drops on the point of the ground under the **centre of the detection box**. The math is in `src/target_geolocation.py`; the numbers it needs (height, heading, field of view, zoom, gimbal angles at the shutter) come from the small `.json` file video_cam writes next to every photo. In the simulator the aim is now within about 1-2 m. |
| **Crash** | `main_controller` died ("AttributeError: 'NoneType' object has no attribute 'values'") right after start-up or at take-off, in 2 of 4 runs. Nobody was left to drop. | The mission indices are read from `waypoint_manager` **without ever blocking** (`wp_sender.parameter.AsyncParameterReader`). A read that fails keeps the old values, says why in the log, and is tried again. |
| **Frozen node** | `time.sleep` inside callbacks: for up to 15 s nothing else was handled. | Every wait is a one-shot timer (`src/timed_steps.py`). Same timings as before. |
| **No GPS** | A photo without a GPS name gave a target at latitude/longitude 0,0. | Such a detection is ignored (with a warning). |

### How the drop aim works, in 5 steps

1. The detector says: "tent, 87 %, box centre at pixel (572, 702) in photo `36.215153 , -96.006786.jpg`".
2. `main_controller` opens that photo's sidecar `36.215153 , -96.006786.json`: the drone was 30 m
   above home, facing west (270 deg), the camera looked straight down with an 81 deg field of view.
3. The photo is 2560 pixels wide, so 1 pixel at the centre covers `30 m / 1499 px` = 2 cm of ground.
   The box is 708 pixels LEFT of the centre and 18 pixels ABOVE it: 14.2 m to the left, 0.4 m ahead.
4. Facing west, "left" is south and "ahead" is west: the tent is 14.2 m south and 0.4 m west of the
   drone's position when it took the photo.
5. That offset is added to the photo's GPS position (with ArduPilot's own formula), and that is
   the waypoint the drone flies to and drops on.

If a photo has no sidecar (or no height / heading, or the zoom was moving), it falls back to the
old behaviour and says so in the log ("photo position (no sidecar for this photo)").

### Parameters

`ros2 run main main_controller.py --ros-args -p name:=value`

| Name | Default | Meaning |
|---|---|---|
| `aim_at_detection_box` | `true` | `false` = old behaviour (fly to the photo's position) |
| `default_camera_pitch_deg` | `-90.0` | camera pitch when the camera did not report its angles (straight down) |
| `default_camera_hfov_deg` | `81.0` | field of view when a sidecar has none |
| `target_ground_height_m` | `0.0` | ground height under the targets above HOME (heights are measured from home) |
| `max_aim_offset_m` | `150.0` | an aim point farther than this from the photo is not trusted |

Which way a photo is turned relative to the drone is set on the CAMERA node:
`video_cam` parameter `image_top_direction_deg` (0 = top of the photo toward the nose; check this on
the real drone before trusting the drops, see `video_cam/HOW_OUR_CAMERA_WORKS.md`).

### What it tells the ground station (STATUSTEXT)

Unchanged texts: `Reached last WP. Holding ...`, `All N images processed. Selecting targets.`,
`Targets: tent@11 -> person@12. Diverting.`, `Resumed AUTO -> WP 11`, `Over tent: loiter 6s then
drop`, `Servo 13 -> 1400`, `All targets done ...`, `RTL`, `Detected tent`.
New: `Aim tent 36.2150261,-96.0068000` (one per target, the exact drop point).

## Tests

```bash
python3 -m pytest ros2_ws/src/main/test -v      # no ROS needed
```
`test_target_geolocation.py` also replays the best hiker and tent detections of three simulator runs
recorded before the fix: the old aim missed by 14-25 m, the new one must land within 2 m.
