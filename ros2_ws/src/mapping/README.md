<!-- Purpose: explain survey photo triggers, timing parameters, and isolated ROS tests. -->
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
ros2 run mapping trigger --ros-args -p autofocus_period_s:=0
```

Whole numbers and decimals both work (`:=0` or `:=0.0`, `:=2` or `:=2.0`). (Normally ROS 2 refuses
a whole number for a decimal parameter and the node would not start at all; this node accepts both.
A value that is not a number, like `:=true`, is reported in the log and the default is used.)
A `capture_period_s` below 0.1 s counts as 0.1 s.

You normally do not need to change them: the camera node (`video_cam`) no longer loses photos while
the lens focuses, and it ignores autofocus by itself on cameras that cannot do it (A8 mini). Slow
the photos down (`capture_period_s`) only if the detector cannot keep up on the Jetson.

## Tests

```bash
source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash
python3 -m pytest ros2_ws/src/mapping/test/test_trigger_node.py -v
```

They start the REAL node with rclpy (ROS 2 must be sourced, otherwise they are skipped) in a private
ROS domain, so they never talk to a running simulator or drone. They check the default periods,
the parameters (including whole numbers), that a repeated DigiCamCtrl starts only ONE photo timer,
that reaching `buffer_wp` stops the photos and the autofocus, and what the node publishes.
