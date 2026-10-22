# mapping: the survey photo trigger

`mapping/do_digi_cam_trigger.py` runs the ROS node `mission_camera_trigger` (executable `trigger`).
When ArduPilot starts the mission's `DO_DIGICAM_CONTROL` item it says `Mission: <n> DigiCamCtrl`;
the node then asks the camera to set its lens up once (`capture_setup`), publishes `/camera/trigger`
every second (one photo each) and an `autofocus` request every 5 seconds, and stops both when the
drone reaches waypoint `buffer_wp` (from `waypoint_manager`).

(`mapping/mapping.py` is an old, broken stitcher that is not launched.)

## October 2026

The two periods are now ROS parameters, with the old values as defaults:

| Parameter | Default | Meaning |
|---|---|---|
| `capture_period_s` | `1.0` | seconds between photo requests |
| `autofocus_period_s` | `5.0` | seconds between autofocus requests; `0` switches the loop off |

```bash
ros2 run mapping trigger --ros-args -p autofocus_period_s:=0.0
```

You normally do not need to change them: the camera node (`video_cam`) no longer loses photos while
the lens focuses, and it ignores autofocus by itself on cameras that cannot do it (A8 mini). Slow
the photos down (`capture_period_s`) only if the detector cannot keep up on the Jetson.
