<!-- PURPOSE: explain how to replace one simulator component with a YAML profile, check its interface, and maintain the generated profile and slot references. Runtime validation status is recorded in docs/HANDOFF.md. -->

# Replace a simulator component

A profile chooses the world, autopilot, camera, detector, mission brain and extra programs for one run.
It changes only simulator wiring: the team's flight launch remains the reference, and replacing a
detector or mission brain filters a generated copy of that launch file. The team's original file is
kept intact. Profiles and templates are still being validated; see [the handoff](../HANDOFF.md).

Run these commands from the Linux checkout after [building the simulator](../../simulation/README.md)
and sourcing `simulation/setup/sim_env.sh`:

```bash
python3 -m sim_plugins.profile_tool list
python3 -m sim_plugins.profile_tool plan --profile truth_detector --check
bash simulation/scripts/demo.sh --profile truth_detector --headless
bash simulation/scripts/stop_sim.sh
```

`plan` starts nothing. `--check` also checks that the required packages, binaries and files exist.
To inspect the full script command without starting a run, use
`bash simulation/scripts/demo.sh --profile truth_detector --dry-run`.

## Make a profile

Copy [default.yaml](../../simulation/profiles/default.yaml), or write just the keys you need. For example:

```yaml
# PURPOSE: run the answer-key detector with the camera's 4K still-image profile.
extends: truth_detector
camera:
  model: a8mini
```

Save it as `my_profile.yaml`, then run:

```bash
python3 -m sim_plugins.profile_tool validate ./my_profile.yaml
python3 -m sim_plugins.profile_tool resolve --profile ./my_profile.yaml
bash simulation/scripts/demo.sh --profile ./my_profile.yaml --headless
```

Settings merge in this order: built-in defaults, the `extends` parent, your file, then `--set KEY=VALUE`
overrides. Script options you type also take precedence over the profile. Paths in profile path fields
are relative to that profile's folder; `~` and environment variables are expanded. Raw `command` entries
are argument lists with no shell: give script arguments as their actual paths. Only the command's first
word is resolved as a relative executable path.

For an ad hoc change, use `--set camera.model=a8mini`. For a repeatable experiment, save the profile.
`default` keeps the normal demo settings. Other shipped examples are listed in
[the profiles README](../../simulation/profiles/README.md), including extra listeners, ArduPilot versions,
parameter files and split-container wiring. See [ARDUPILOT.md](ARDUPILOT.md) for autopilot details.

## Test new recognition weights

The shortest path for an Ultralytics model compatible with the team's SAHI pipeline is to supply its
weights. Keep them in local model storage outside this public repo:

```bash
source simulation/setup/sim_env.sh
export ASTRA_DETECTOR_MODEL="$HOME/.astra_sim/models/my_detector.pt"
bash simulation/scripts/demo.sh runway --headless
```

This changes the laptop wrapper's model path for that process environment. For a saved experiment:

```yaml
# PURPOSE: test locally installed recognition weights through the team's existing detector.
detector:
  kind: team
  model_path: ~/.astra_sim/models/my_detector.pt
```

Save it as `my_model.yaml`; then run this sequence:

```bash
python3 -m sim_plugins.profile_tool plan --profile ./my_model.yaml --check
bash simulation/scripts/demo.sh --profile ./my_model.yaml --headless
bash simulation/scripts/stop_sim.sh
```

The existing `detection_processor.CLASS_MAP` recognizes model labels `person`, `mannequin`, `doll`,
`umbrella` and `kite`; the last two become tent targets. A model whose label is literally `tent` is
currently treated as an ordinary object. Use a custom adapter for a new label vocabulary rather than
assuming the weight file alone gives the mission the right classes. Test score thresholds, aerial
image scale and the exported model format too. The simulation wrapper uses SAHI by default;
`detector.batched: true` selects the team's TensorRT-oriented path and needs separate verification.

A Jetson `.engine` file is tied to its runtime/hardware. Use portable supported weights or rebuild the
engine for the target machine; a simulation adapter is usually easier for a different ML framework.
Use the same world, mission, camera, zoom and truth settings for model comparisons. Check saved photos,
precision/recall, all-photos-processed counts, false alarms and both drop errors in the run report.

For mission plumbing without learned inference, use `--detector truth`. Its answer-key detections are
labelled in the panel/report. The team's detector process still starts against an empty feed and may
load weights; truth mode is not itself a guarantee that every process avoids GPU use. For a custom
CPU-only detector, select your CPU implementation with `detector.kind: custom` and verify it separately.

## Replace the detector or mission brain

Start with [example_detector](../../ros2_ws/src/simulation/sim_plugin_examples/example_detector/README.md)
or [example_listener](../../ros2_ws/src/simulation/sim_plugin_examples/example_listener/README.md).
The example detector is a simple colour-based template for the generated worlds; it is not the team's
YOLO detector. Build your package before selecting it in a profile.

```yaml
# PURPOSE: select a replacement detector while keeping the team's mission brain.
detector:
  kind: custom
  custom:
    package: example_detector
    launch: example_detector.launch.py
    node_name: example_detector
    provides_health: true
```

An entry starts exactly one launch file, one ROS executable, or one raw command. `args` holds launch
arguments for `launch`, or ROS parameters for `executable`. For a custom mission brain, use the same
entry shape under `mission_brain`, with `kind: custom`. `extras` adds programs alongside the selected
components; its `start_with` chooses the simulator or team-stack process group.

The detector must report every saved photo, including photos with no targets, and use the exact photo
file name. Publish `/image_detection` with reliable QoS, class IDs `"0"` for person and `"1"` for tent,
and box coordinates in full-photo pixels. Run one detector and one mission brain at a time. The full
contracts below give the remaining fields, services and ownership rules.

## Check a running component

Start a run with `--keep` (or use `start_sim.sh` and the printed second-terminal command), then check
the node that actually owns the slot:

```bash
ros2 run sim_plugins check_slot --slot detector --node example_detector --take-photo
ros2 run sim_plugins check_slot --slot mission_brain --node main_controller
```

Read all failures and warnings. A graph check proves topic types and QoS; a sampled message checks
fields and photo names. A passing interface check still needs a full mission and score report to
prove useful behaviour. The checker exits nonzero on failure. Stop your simulator when finished.

For a hand-started custom stack, follow the exact command printed by
`bash simulation/scripts/start_sim.sh --profile ./my_profile.yaml --dry-run`; it passes the resolved
profile to the launches that need custom entries. Direct `ros2 launch ... profile:=...` fills arguments
still at their defaults. An explicitly typed value equal to a launch default cannot be distinguished
from an omitted value; use the scripts and `--set` when overriding a profile back to that default.

## Profile reference

These tables are generated from `sim_plugins/schema.py`. After changing the schema or contracts, run
`python3 -m sim_plugins.profile_tool docs --write`; `docs --check` and the unit tests detect stale blocks.

<!-- BEGIN GENERATED reference: python3 -m sim_plugins.profile_tool docs --write -->
The **Default** column is today's simulator (`simulation/profiles/default.yaml` writes every key out).
A profile only needs the keys it changes. The dotted name (for example `camera.model`) is what
`--set KEY=VALUE` takes on the command line.

#### The file itself

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `profile` | `1` | a whole number 1..1 | format version of the file (always 1) |
| `name` | `""` | text | a short name shown in the plan (default: the file name) |
| `description` | `""` | text | one sentence: what this profile is for |
| `extends` | `""` | text | start from another profile (a name in simulation/profiles or a path), then change only what this file says |

#### `world`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `world.name` | `runway` | text | runway \| suas_field \| suas_field_real (when built) \| any name with sim_worlds/config/<name>.yaml \| a path to a .sdf or a world .yaml |
| `world.mission` | `""` | a file path (a relative path starts at the profile's folder) | another survey mission (.waypoints) instead of the world's own |

#### `autopilot`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `autopilot.kind` | `sitl_binary` | one of `sitl_binary`, `sitl_source`, `external` (also: `binary` = `sitl_binary`, `sitl` = `sitl_binary`, `source` = `sitl_source`, `remote` = `external`) | sitl_binary = a prebuilt arducopter \| sitl_source = an ArduPilot source tree \| external = an autopilot running elsewhere (another container or PC) |
| `autopilot.version` | `""` | a version name: `4.7.1`, `stable`, `beta`, `latest` ... (`Copter-4.7.1` and `V4.7.1` work too) | sitl_binary: "" = the installed default (~/sitl/arducopter, 4.6.3); 4.7.1 / latest / ... = ~/sitl/versions/<v>/arducopter (get it with simulation/setup/fetch_sitl.sh <v>) |
| `autopilot.binary` | `""` | a file path (a relative path starts at the profile's folder) | sitl_binary: the path of an arducopter program (wins over version) |
| `autopilot.source_dir` | `""` | a file path (a relative path starts at the profile's folder) | sitl_source: an ArduPilot source tree, e.g. /opt/ardupilot (Docker sitl image) or ~/ardupilot; it must be built (./waf configure --board sitl && ./waf copter) |
| `autopilot.param_files` | `[]` | a list of file paths, e.g. `[my.param]` (relative = from the profile's folder) | extra parameter files layered on top (e.g. the real drone's .param saved from Mission Planner); hardware settings are filtered out (see param_filter) |
| `autopilot.param_filter` | `safe` | one of `safe`, `none` | safe = drop hardware and airframe settings that would break the simulator \| none = use every line (experts) |
| `autopilot.param_keep` | `[]` | a list of ArduPilot parameter names, e.g. `[ATC_RAT_RLL_P]` | parameter names to keep even though the safe filter would drop them, e.g. [ATC_RAT_RLL_P] |
| `autopilot.wipe` | `false` | `true` or `false` | true = erase SITL's saved parameters and mission at every start |
| `autopilot.instance` | `0` | a whole number 0..20 | SITL instance I: ports 5760+10*I (MAVLink) and 9002+10*I (Gazebo link) |
| `autopilot.external.mavlink` | `""` | a MAVLink address: `tcp:HOST:PORT`, `udpin:0.0.0.0:PORT`, ... (or a `/dev/...` serial port) | external: where MAVProxy picks up the autopilot's MAVLink, e.g. tcp:192.168.1.50:5760 or udpin:0.0.0.0:14560 |
| `autopilot.external.fdm_addr` | `0.0.0.0` | an IPv4 address (`0.0.0.0` = every address of this computer) | external: the address the drone's ArduPilot plugin binds its physics port (UDP 9002+10*I) on; 0.0.0.0 = every network, so a SITL on another computer/container can reach it |
| `autopilot.external.sim_address` | `""` | text | external: this computer's address as the remote SITL sees it (only used to print the command for the other computer) |

#### `camera`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `camera.kind` | `siyi_emulator` | one of `siyi_emulator`, `custom`, `none` (also: `emulator` = `siyi_emulator`, `siyi` = `siyi_emulator`, `off` = `none`) | siyi_emulator = the SIYI camera emulator \| custom = your own camera program \| none = no camera on the network |
| `camera.model` | `zr10` | one of `zr10`, `a8mini` (also: `a8-mini` = `a8mini`, `a8_mini` = `a8mini`, `a8` = `a8mini`, `zr-10` = `zr10`) | zr10 = 2K photos 2560x1440 (last year's default) \| a8mini = 4K photos 3840x2160 |
| `camera.zoom` | `survey` | `survey`, `team`, or a number (the largest zoom, above 0 and up to 30) | survey = photos keep the full 81 degree view (max zoom 1x) \| team = video_cam's 5x zoom rack works \| a number = max zoom |
| `camera.strict` | `false` | `true` or `false` | true = the emulator stays silent on commands a real A8 mini ignores (0x04 0x06 0x16 0x18 0x19); false = answers everything |
| `camera.custom` | `null` | an entry: a mapping with the entry keys below | camera.kind custom: what to start instead of siyi_emulator |

#### `detector`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `detector.kind` | `team` | one of `team`, `truth`, `custom` (also: `real` = `team`, `new_od` = `team`, `answer_key` = `truth`, `stand-in` = `truth`) | team = detection/new_od (YOLO) \| truth = sim_demo truth_detector (answer key) \| custom = your detector (replaces new_od) |
| `detector.model_path` | `""` | a file path (a relative path starts at the profile's folder) | team: the YOLO weights ("" = ~/.astra_sim/models/yolo26m.pt or $ASTRA_DETECTOR_MODEL) |
| `detector.batched` | `false` | `true` or `false` | team: true = the team's batched inference path (needs a TensorRT engine); false = SAHI path (works with .pt) |
| `detector.custom` | `null` | an entry: a mapping with the entry keys below | detector.kind custom: your detector (see ENTRY_KEYS) |

#### `mission_brain`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `mission_brain.kind` | `team` | one of `team`, `custom` (also: `main_controller` = `team`, `real` = `team`) | team = main/main_controller.py \| custom = your mission logic (replaces it) |
| `mission_brain.custom` | `null` | an entry: a mapping with the entry keys below | mission_brain.kind custom: your node (see ENTRY_KEYS) |

#### `extras`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `extras` | `[]` | a list of entries (each starts with `- `) | extra things to start: a list of entries (see ENTRY_KEYS) |

#### `network`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `network.gcs_port` | `14550` | a port number 1..65535 | MAVProxy UDP output on 127.0.0.1 for a ground station / our scripts |
| `network.mavros_port` | `14552` | a port number 1..65535 | MAVProxy UDP output for MAVROS (./launcher.sh sim) |
| `network.panel_port` | `14553` | a port number 1..65535 | MAVProxy UDP output for the demo panel's mission buttons |
| `network.gcs_out` | `""` | a MAVProxy output: `udp:HOST:PORT` (or `udpout:`, `udpbcast:`, `tcp:`, `tcpin:`) | "" = udp:127.0.0.1:<gcs_port>; or another MAVProxy output. Careful: demo.sh / upload_mission.sh / fly_mission.sh listen on udp 127.0.0.1:<gcs_port>; to ADD a ground station use extra_outs |
| `network.mavros_out` | `""` | a MAVProxy output: `udp:HOST:PORT` (or `udpout:`, `udpbcast:`, `tcp:`, `tcpin:`) | "" = udp:127.0.0.1:<mavros_port>; e.g. udp:172.18.0.3:14552 = a MAVROS in another container |
| `network.extra_outs` | `[]` | a list of MAVProxy outputs, e.g. `[udp:192.168.1.20:14550]` | more MAVProxy outputs, e.g. [udp:192.168.1.20:14550] |
| `network.windows_gcs` | `true` | `true` or `false` | inside WSL also send MAVLink to Mission Planner on Windows |
| `network.windows_gcs_addr` | `""` | `HOST` or `HOST:PORT` | "" = automatic; HOST or HOST:PORT = send it there |
| `network.camera_ip` | `192.168.144.25` | an IPv4 address (`0.0.0.0` = every address of this computer) | the address the camera emulator pretends to be |
| `network.camera_bind` | `""` | an IPv4 address (`0.0.0.0` = every address of this computer) | "" = camera_ip if this computer has it, else 127.0.0.1; 0.0.0.0 = listen on every address (containers) |
| `network.fcu_url` | `""` | text | "" = udp://:<mavros_port>@localhost:<mavros_port>; MAVROS's fcu_url for ./launcher.sh sim (demo.sh) |

#### `run`

| Key | Default | Accepts | What it does |
|---|---|---|---|
| `run.headless` | `false` | `true` or `false` | true = no windows (Gazebo, RViz, demo panel) |
| `run.timeout_min` | `30` | a whole number 1..1440 | demo.sh gives up after this long |
| `run.team_stack` | `local` | one of `local`, `external` | local = demo.sh starts ./launcher.sh sim \| external = the team stack runs somewhere else (another container/PC); demo.sh only waits for it |

#### Entry keys

An **entry** says how to start one program: `detector.custom`, `mission_brain.custom`, `camera.custom`
and every item of `extras`. Give exactly ONE of `launch`, `executable` or `command`.

| Key | What it does |
|---|---|
| `name` | a short label for messages (extras); default = the package or program name |
| `package` | the ROS 2 package that holds the launch file or the executable |
| `launch` | a launch file: a name inside <package>/share/<package>/launch/, or a path to the file |
| `executable` | OR one node: `ros2 run <package> <executable>` |
| `command` | OR any program, as a list: ["python3", "/home/me/logger.py", "--fast"] (no shell) |
| `args` | a mapping: launch arguments (launch) or ROS parameters (executable) |
| `node_name` | the ROS node name it creates (stack_check waits for it; check_slot inspects it) |
| `provides_health` | detector only: true = it serves /sahi/get_health like new_od (stack_check waits for "ModelLoaded: True") |
| `start_with` | extras only: team_stack (default, starts with ./launcher.sh sim) or simulator (starts with sim.launch.py) |
<!-- END GENERATED reference -->

## Slot contracts

These tables are generated from `sim_plugins/contracts.py`. Optional simulator observations can help
the panel and scoring tools, while the required interfaces let the selected components work together.

<!-- BEGIN GENERATED contracts: python3 -m sim_plugins.profile_tool docs --write -->
<a id="detector"></a>

### Detector (finds the hiker and the tent in the photos)

**Default:** team: detection/new_od (YOLO26m on 640 px tiles), started by main/launch/tracker.launch.xml

**Alternatives:** truth (sim_demo truth_detector, the answer key) | custom (your detector: replaces new_od)

Reads every new photo that video_cam saves in the team photo folder and publishes ONE interfaces/ImageResult per photo on /image_detection. main_controller uses those messages to choose where to drop.

Node name the stack waits for: `new_od` (a custom component says its own with `node_name`).

| Topic | Type | The component ... | QoS | Required | Notes |
|---|---|---|---|---|---|
| `/image_detection` | `interfaces/msg/ImageResult` | publishes | reliable | yes | one message per photo (RELIABLE, depth 10, like new_od); main_controller subscribes RELIABLE |
| `/sahi_detection_info` | `std_msgs/msg/String` | publishes | any | no | optional JSON per photo: image, timestamp, num_people, num_tents, num_objects, total_detections, method |
| `/mavros/mission/reached` | `mavros_msgs/msg/WaypointReached` | subscribes to | any | no | optional: new_od copies the last reached item into ImageResult.waypoint_index |
| `/sim/camera/photo` | `sensor_msgs/msg/Image` | subscribes to | any | no | SIMULATOR ONLY, optional extra input: every photo the camera emulator renders (the photo FILE stays the real input: main_controller matches results by file name) |

| Service | Type | The component is the ... | Required | Notes |
|---|---|---|---|---|
| `/sahi/get_health` | `std_srvs/srv/Trigger` | server | no | recommended: message "Healthy: True, ConsecErrors: 0, ModelLoaded: True, Format: <name>"; stack_check --need-detector waits for ModelLoaded: True |
| `/sahi/get_statistics` | `std_srvs/srv/Trigger` | server | no | optional |

| File / folder | The component ... | Notes |
|---|---|---|
| `<ws>/src/video_cam/mapping_photos/"<lat> , <lon>.jpg"` | reads | the photos (6 decimals, e.g. "36.215026 , -96.006800.jpg"); "IMG_NNNN.jpg" / "sim_*.jpg" when there was no GPS fix. Written atomically as "<name>.tmp.jpg" then renamed: ignore *.tmp.* |
| `<ws>/src/video_cam/mapping_photos/"<lat> , <lon>.json"` | reads | optional sidecar (format astra.photo_metadata.v1): height above home, heading, gimbal angles, zoom, field of view at the shutter. Not an image: skip .json files |
| `<ws>/src/video_cam/detection_results_sahi/` | writes | optional annotated images (demo.sh copies them into the run folder) |

Rules:

1. Publish exactly ONE ImageResult per photo file, ALSO when nothing was found: at the last survey waypoint main_controller waits (up to 90 s) until every photo name in the folder was reported.
2. image_name = the photo's file name exactly as in the folder (no folder part).
3. detections = vision_msgs/Detection2DArray with ONLY persons and tents: results[0].hypothesis.class_id "0" = person, "1" = tent; .score = confidence 0..1; bbox.center.position.x/.y = box centre and bbox.size_x/.size_y = box size, in pixels of the FULL-SIZE photo.
4. latitude/longitude = the GPS in the file name (float32); 0.0 when the name has none.
5. method = your detector's name. It must NOT start with "truth_detector" (the demo panel would show the answer-key banner).
6. header.frame_id "camera"; timestamp = ISO wall-clock text; num_detections; classes / confidences / areas / descriptions = parallel lists over every detection (information only).
7. Run ONE detector at a time: main_controller cannot tell two detectors apart.
8. main_controller keeps the highest score per class (strictly higher wins) and aims at that box's centre (using the sidecar); a photo without GPS is ignored.

<a id="mission_brain"></a>

### Mission brain (decides where to drop, flies there, drops, goes home)

**Default:** team: main/src/main_controller.py (node main_controller)

**Alternatives:** custom (your mission logic: replaces main_controller)

ArduPilot flies the uploaded survey by itself (AUTO). The mission brain listens to the detector, and after the last survey waypoint inserts one waypoint per target, flies there, opens the release servos and sends the drone home.

Node name the stack waits for: `main_controller` (a custom component says its own with `node_name`).

| Topic | Type | The component ... | QoS | Required | Notes |
|---|---|---|---|---|---|
| `/image_detection` | `interfaces/msg/ImageResult` | subscribes to | reliable | yes | the detector's results (see the detector contract) |
| `/mavros/mission/waypoints` | `mavros_msgs/msg/WaypointList` | subscribes to | any | yes | the mission on the autopilot (find the last NAV item before RTL) |
| `/mavros/mission/reached` | `mavros_msgs/msg/WaypointReached` | subscribes to | any | yes | which mission item was just reached (wp_seq) |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | subscribes to | any | no | optional: waypoint_manager publishes the mission indices as its parameters |
| `/mavros/statustext/send` | `mavros_msgs/msg/StatusText` | publishes | any | no | recommended: short texts (max 50 characters) that Mission Planner and the phase monitor show |

| Service | Type | The component is the ... | Required | Notes |
|---|---|---|---|---|
| `/addWaypoint` | `interfaces/srv/AddWaypoint` | client | yes | waypoint_manager INSERTS one item at `index` and pushes the whole mission (command 16 = NAV_WAYPOINT, altitude relative to home) |
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | client | yes | custom_mode GUIDED / AUTO / RTL |
| `/mavros/mission/set_current` | `mavros_msgs/srv/WaypointSetCurrent` | client | yes | point the mission at the first inserted target |
| `/mavros/cmd/command` | `mavros_msgs/srv/CommandLong` | client | yes | MAV_CMD_DO_SET_SERVO (183): param1 = output, param2 = PWM |
| `/waypoint_manager/get_parameters` | `rcl_interfaces/srv/GetParameters` | client | no | optional: num_waypoints, takeoff_index, rtl_index, next_after_takeoff, last_before_rtl |

| File / folder | The component ... | Notes |
|---|---|---|
| `<ws>/src/video_cam/mapping_photos/` | reads | optional: which photos exist (to know when the detector is done) |
| `<ws>/src/video_cam/mapping_photos/"<lat> , <lon>.json"` | reads | optional: the photo's sidecar, to turn a box centre into latitude/longitude |

Rules:

1. Never arm, take off or upload the survey: the ground station (demo.sh / Mission Planner) does that.
2. At the last survey waypoint switch to GUIDED (hover) and wait for the detector (main_controller: until every photo was reported, at most 90 s).
3. Insert one NAV_WAYPOINT (16) per target right before the mission's RETURN_TO_LAUNCH item (main_controller: index = last_before_rtl + 1, tent first, altitude 49 m), set the current item to the first target, then AUTO.
4. Over each target: GUIDED, hold, DO_SET_SERVO output 13 (tent) or 11 (person) to 1400 us (open) for 3 s, then 500 us (closed). After the last target: RTL.
5. Phase monitor texts (optional): "Reached last WP ...", "All N images processed ..." or "Processing timeout ...", "Targets: ...", "No valid targets ...", "Over tent|person ...", "Servo 13 -> 1400", "RTL".
6. Run ONE mission brain at a time (two would fight over the flight mode).

<a id="camera"></a>

### Camera (the SIYI gimbal camera on the drone's network)

**Default:** siyi_emulator: pretends to be the SIYI camera at 192.168.144.25 (profiles zr10 = 2K photos, a8mini = 4K; strict = silent like a real A8 mini on commands it does not support)

**Alternatives:** custom (your camera program) | none (no camera)

The team's video_cam talks to "the camera" exactly like on the drone: SIYI SDK over UDP and the media web API over HTTP. In the simulator the camera renders its photos in Gazebo and moves the Gazebo gimbal.

Node name the stack waits for: `siyi_emulator` (a custom component says its own with `node_name`).

| Topic | Type | The component ... | QoS | Required | Notes |
|---|---|---|---|---|---|
| `/sim/camera/photo_info` | `std_msgs/msg/String` | publishes | reliable | no | optional (demo tools): JSON record of every photo (RELIABLE + TRANSIENT_LOCAL, depth 10) |
| `/sim/camera/emulator_state` | `std_msgs/msg/String` | publishes | any | no | optional (demo panel): JSON zoom + gimbal, 2 Hz |

| Network | The component ... | Notes |
|---|---|---|
| `UDP <camera_ip>:37260` | listens | SIYI SDK: frames 55 66 \| ctrl \| len \| seq \| cmd \| data \| CRC16-XMODEM; video_cam sends 0x02 (hardware id), 0x0A (system info), 0x0C (photo), 0x0D/0x0E (gimbal attitude / angles), 0x0F (zoom), 0x05/0x07/0x08, 0x18 (zoom read-back) and 0x04/0x06 (focus; an A8 mini ignores 0x04 0x06 0x16 0x18 0x19) |
| `TCP <camera_ip>:82` | listens | HTTP media API: GET /cgi-bin/media.cgi/api/v1/getdirectories?media_type=0, getmedialist?media_type=0&path=<dir>&start=0&count=<n>, then GET each photo url (JPEG) |
| `gz /sim/camera/photo/trigger -> /sim/camera/photo/image` | talks to | Gazebo (gz-transport, partition $GZ_PARTITION): one gz.msgs.Boolean = one rendered photo |
| `gz /sim/gimbal/{yaw,roll,pitch}/cmd_pos` | talks to | gz.msgs.Double, radians; yaw + = left, pitch -pi/2 = straight down; /sim/drone/odometry for stabilising |

Rules:

1. Photo size = the camera profile: zr10 2560x1440, a8mini 3840x2160 (video_cam checks the size).
2. After 0x0C the new photo must appear in getmedialist within 15 s (video_cam polls every 0.5 s).
3. Send the 0x0B "photo OK" feedback within 0.8 s, or nothing (silence = success for video_cam).
4. Listen on camera_ip (192.168.144.25 is added to this computer by simulation/setup/camera_ip_alias.sh), or on camera_bind (0.0.0.0 in containers).

<a id="autopilot"></a>

### Autopilot (the Pixhawk firmware: ArduCopter)

**Default:** ArduCopter SITL 4.6.3 (~/sitl/arducopter) + copter.parm, gazebo-iris.parm, astra_sitl.parm

**Alternatives:** another version (fetch_sitl.sh), a source build (/opt/ardupilot), extra parameter files, or external (another container / PC)

Flies the drone in Gazebo through the ArduPilot plugin's JSON physics link and talks MAVLink to MAVProxy, which copies it to MAVROS (the team stack), our scripts and the demo panel.

| Topic | Type | The component ... | QoS | Required | Notes |
|---|---|---|---|---|---|
| `/mavros/state` | `mavros_msgs/msg/State` | publishes | any | yes | published by MAVROS once ./launcher.sh sim runs: connected must become true |

| Network | The component ... | Notes |
|---|---|---|
| `UDP 9002+10*I (Gazebo computer)` | talks to | JSON physics link: the drone's ArduPilotPlugin listens (lock-step); SITL sends servo outputs |
| `TCP 5760+10*I` | listens | SITL serial0: MAVProxy connects here (5762/5763+10*I are spare) |
| `UDP 127.0.0.1:14550 / 14552 / 14553` | talks to | MAVProxy outputs: ground station + our scripts / MAVROS / demo panel (+ Windows ground station) |

Rules:

1. Behave like ArduCopter: modes GUIDED, AUTO and RTL; MISSION_ITEM_REACHED; the text "Mission: <n> DigiCamCtrl" when the DO_DIGICAM_CONTROL item starts (it starts the photos).
2. Accept MAV_CMD_DO_SET_SERVO on outputs 11 and 13 (SERVO11/13_FUNCTION 0).
3. Arm and take off in AUTO without RC sticks (AUTO_OPTIONS 3); no geofence, no GCS failsafe in the sim.
4. Home = the world's home (sim_worlds/config/<world>.yaml), so GPS and the Gazebo picture agree.
5. Wait ~30 s after boot before arming: "Arm: Gyros inconsistent" until then is normal.

<a id="world"></a>

### World (the field, the targets and the survey mission)

**Default:** runway (sim_worlds/config/runway.yaml)

**Alternatives:** suas_field | suas_field_real (real terrain; being built) | any .sdf + config .yaml

A Gazebo world plus one config file that tells the bringup where the world file, the SITL home, the spawn pose, the survey mission and the answer key (truth) are.

| Topic | Type | The component ... | QoS | Required | Notes |
|---|---|---|---|---|---|
| `/sim/drone/odometry` | `nav_msgs/msg/Odometry` | publishes | any | yes | bridged from Gazebo while the simulator runs (shows the world is loaded with the drone) |
| `/sim/camera/image` | `sensor_msgs/msg/Image` | publishes | any | no | live camera (bridge) |

| File / folder | The component ... | Notes |
|---|---|---|
| `sim_worlds/config/<name>.yaml` | reads | world_file, home "lat,lon,alt_msl,heading" (= the SDF's <spherical_coordinates>), spawn {x,y,z,yaw}, mission (.waypoints), truth (answer key) |
| `<world>.sdf` | reads | systems Physics (max_step_size 0.001), UserCommands, SceneBroadcaster, Sensors (ogre2), Imu, NavSat; <spherical_coordinates> EARTH_WGS84 ENU |
| `missions/<world>_survey.waypoints` | reads | QGC WPL 110: TAKEOFF, DO_DIGICAM_CONTROL, survey waypoints, RETURN_TO_LAUNCH last (check: ros2 run sim_bringup mission_tool check-mission --mission FILE) |

Rules:

1. Any name with sim_worlds/config/<name>.yaml works as world.name (suas_field_real too, once it exists).
2. A bare .sdf works as well: SITL home comes from its <spherical_coordinates>; give the mission with world.mission (demo.sh --mission).

<a id="extras"></a>

### Extras (anything else you want running: a logger, a monitor, a test driver)

**Default:** nothing

**Alternatives:** launch files, single nodes or plain programs, started with the team stack or the simulator

Extras have no fixed contract. They should only listen, unless they replace something on purpose.

Rules:

1. Do not publish on a topic another slot owns (/image_detection, /camera/trigger, /mavros/...) unless that is what you are testing.
2. Subscribe to /mavros topics with BEST_EFFORT QoS: it receives from every publisher.
3. An extra with start_with: team_stack starts and stops with ./launcher.sh sim; start_with: simulator starts with sim.launch.py.

<a id="network"></a>

### Network (ports and addresses; Docker and multi-computer setups)

**Default:** everything on 127.0.0.1 inside one computer / container

**Alternatives:** MAVProxy outputs to other hosts (mavros_out, gcs_out, extra_outs), the camera emulator on another address (camera_bind), an external autopilot (mavlink_master, fdm_addr)

Where MAVLink goes and where the camera listens. The defaults are today's values.

| Network | The component ... | Notes |
|---|---|---|
| `MAVProxy --master tcp:127.0.0.1:5760+10*I` | talks to | launch argument mavlink_master |
| `MAVProxy --out udp:127.0.0.1:14550` | talks to | gcs_port / gcs_out: our scripts (mission_tool) |
| `MAVProxy --out udp:127.0.0.1:14552` | talks to | mavros_port / mavros_out: MAVROS (launcher.sh sim fcu_url udp://:14552@localhost:14552) |
| `MAVProxy --out udp:127.0.0.1:14553` | talks to | panel_port: demo panel buttons (the panel listens on 127.0.0.1) |
| `MAVProxy --out udp:<Windows host>:14550` | talks to | windows_gcs / windows_gcs_addr |
| `siyi_emulator UDP 37260 + TCP 82 on 192.168.144.25` | listens | camera_ip / camera_bind |
| `Gazebo plugin UDP 9002+10*I on 127.0.0.1` | listens | fdm_addr (0.0.0.0 for an external SITL) |
<!-- END GENERATED contracts -->
