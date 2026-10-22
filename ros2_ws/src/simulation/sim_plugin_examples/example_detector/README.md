# example_detector: a template detector you copy

A tiny detector that does on the outside **exactly** what the team's `detection/new_od` does, so the
mission code (`main_controller`) and the demo panel use it without a single change:

1. it watches the team photo folder `ros2_ws/src/video_cam/mapping_photos/` (the folder `video_cam`
   writes; `new_od` calls it `camera_feed_path`) for new photos named `"<lat> , <lon>.jpg"`;
2. it looks for the hiker (class `"0"`, person) and the tent (class `"1"`) in each photo; here by
   **colour** (`example_detector/blob_logic.py`), which is the part you replace with your own model;
3. it publishes **one** `interfaces/ImageResult` per photo on `/image_detection` (RELIABLE QoS, depth 10),
   also when nothing was found;
4. it answers `/sahi/get_health` with `ModelLoaded: True` like `new_od`, so `demo.sh` can wait for it.

The full rules are the detector contract in
[docs/simulation/PLUGINS.md](../../../../../docs/simulation/PLUGINS.md#detector).

| File | What it is |
|---|---|
| `example_detector/blob_logic.py` | the detection "brain": plain Python + OpenCV, no ROS. **Change this.** |
| `example_detector/node.py` | the ROS 2 node: folder in, `/image_detection` out. Usually unchanged. |
| `launch/example_detector.launch.py` | starts the node with its settings as launch arguments |
| `test/test_blob_logic.py` | unit tests of the brain (no ROS needed) |

## Copy -> rename -> change -> plug in -> check

**1. Copy** (next to the other simulation packages, or anywhere under `ros2_ws/src`):

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/simulation/sim_plugin_examples   # or the Windows copy
cp -r example_detector my_detector
```

**2. Rename** `example_detector` to `my_detector` in these places (the folder names too):

| Where | What to change |
|---|---|
| `my_detector/example_detector/` (folder) | rename the folder to `my_detector/my_detector/` |
| `resource/example_detector` (empty file) | rename to `resource/my_detector` |
| `package.xml` | `<name>my_detector</name>` |
| `setup.py` | `package_name = 'my_detector'` and the entry point `'my_detector = my_detector.node:main'` |
| `setup.cfg` | both paths: `$base/lib/my_detector` |
| `launch/example_detector.launch.py` | rename to `my_detector.launch.py`; inside: `package='my_detector', executable='my_detector', name='my_detector'` |
| `node.py` | `super().__init__('my_detector')` and `method_name` default, e.g. `'my_detector v1'` |

**3. Change** `blob_logic.detect()` (or write your own function) so it returns a list of `Detection`
(`class_id` "0"/"1", `score` 0..1, box centre `cx, cy` and size `w, h` in pixels of the **full** photo).
Keep the tests green: `python3 -m pytest test -q`.

**4. Build and plug in** with a profile. Copy `simulation/profiles/example_detector.yaml` to
`my_detector.yaml` and change the three names:

```yaml
profile: 1
name: my_detector
detector:
  kind: custom
  custom:
    package: my_detector
    launch: my_detector.launch.py
    node_name: my_detector
    provides_health: true
```

```bash
bash simulation/setup/03_build.sh --workers 4               # builds my_detector too
python3 -m sim_plugins.profile_tool plan --profile ./my_detector.yaml --check   # what will run
bash simulation/scripts/demo.sh --profile ./my_detector.yaml
```

`new_od` is then **not** started (a copy of the team's `tracker.launch.xml` without it is used) and
`demo.sh` waits for your node instead.

**5. Check** that it keeps the contract while the simulator runs (second terminal):

```bash
ros2 run sim_plugins check_slot --slot detector --take-photo
```

It lists every rule with OK / XX and ends with `RESULT: PASS` or `RESULT: FAIL`.

## Running it by hand

```bash
ros2 run example_detector example_detector                       # the team photo folder
ros2 run example_detector example_detector --ros-args -p camera_feed_path:=/tmp/photos -p min_blob_area_px:=300
ros2 launch example_detector example_detector.launch.py min_blob_area_px:=300
```

Colour ranges are parameters too (`person_hsv_low`, `person_hsv_high`, `tent_hsv_low`, `tent_hsv_high`,
OpenCV HSV `[hue 0-179, saturation 0-255, value 0-255]`); the defaults are placeholders.

## Troubleshooting

| What you see | Why | Fix |
|---|---|---|
| `main_controller` waits 90 s at the last waypoint | your node skipped some photos or published `image_name` with a folder | one message per photo, `image_name` = the file name only |
| the drop goes to the photo centre, not the target | box in the wrong pixels (a resized copy) | scale boxes back to the full photo (see `detect()`) |
| the panel shows "STAND-IN" | your `method` starts with `truth_detector` | use your own name |
| `check_slot` says BEST_EFFORT | your publisher's QoS | `QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)` |
| `demo.sh` waits for `new_od` | the profile has no `node_name` | add `node_name: <your node name>` |
| `ModuleNotFoundError: interfaces` | the team packages are not built / sourced | `bash simulation/setup/03_build.sh`, then `source simulation/setup/sim_env.sh` |
