<!--
AGENTS.md -- PURPOSE: the first file an AI coding assistant (Codex, Cursor, Copilot, Claude Code through
CLAUDE.md, and others) reads in this repo. It says what the repo is, where everything lives and how finished
each part is, how the real drone software and the Gazebo digital twin work (exact nodes, topics, services,
ports, files), how to build, run and test, the rules for changing code here, short recipes and known traps.
Human guides are linked, not copied. Facts were checked against the code on 2026-10-24 (local branch
gazebo-sim tracks origin/jarrod-gazebo-sim; reviewed work is committed locally without pushing). Parts
marked WIP still need the runtime checks in HANDOFF.md. Keep this file under ~700 lines and
update it in the same change as the code it describes.
-->

# AGENTS.md: guide for AI coding assistants (ASTRA ODLC + Gazebo digital twin)

**Start here:** the current status, the uncommitted work and the ordered task list are in
[`docs/HANDOFF.md`](docs/HANDOFF.md). If `LOCAL_NOTES.md` exists at the repo root, read it too
(machine-specific notes for the owner's laptop; never committed).

Status date: **2026-10-24**. **WIP** = work in progress, verify in the code before relying on it.
Commands below run **inside Ubuntu 22.04** (native, or the WSL2 distro `Ubuntu-22.04` on Windows) unless
they say PowerShell. `<repo>` = the repo root; `~/ODLC_Machine_Inferencing_System_2025-2026` = the
Linux-side copy that `simulation/setup/03_build.sh` makes (section 5).

## 1. What this repo is

- Flight + vision software of team ASTRA (GitHub org CPP-Aerial-Vision-Analysis-System) for **SUAS 2026**:
  search a field for a **"hiker" (a mannequin) and a tent**, drop one payload on each, return home (RTL).
- The drone: **Jetson Orin Nano** (JetPack 6.2 = Ubuntu 22.04, **ROS 2 Humble**, Python 3.10) runs the team's
  ROS 2 nodes; a **Pixhawk with ArduCopter 4.6.3** flies; a **SIYI A8 mini** gimbal camera sits at
  **192.168.144.25** (SIYI SDK on UDP 37260, HTTP media API on port 82); **MAVROS** links ROS 2 and ArduPilot.
- A **Gazebo Harmonic digital twin** (ArduCopter SITL 4.6.3 + a SIYI camera emulator at the real camera
  address) runs the team's code **unchanged** (`./launcher.sh sim`) and scores each simulated mission.

## 2. Repo map

Status words: **FLOWN** = part of the flight stack on the real drone; **FLOWN\*** = flown code with
October 2026 fixes that are verified only in the simulator (not yet flown); **SIM** = simulator only, works
(verified in the full demo); **WIP**; **DEAD** = broken / unused / experimental, do not build on it;
**TOOL** = bench or operator tool; **DOC** = notes.

### Top level

| Path | What | Status |
|---|---|---|
| `launcher.sh` | THE entry point on the Jetson: `./launcher.sh` (hw) or `./launcher.sh sim`; sources Humble + `ros2_ws/install`, starts MAVROS, then `$TRACKER_CMD` (default `ros2 launch main tracker.launch.xml`) | FLOWN (the sim only added the `TRACKER_CMD` env override) |
| `ros2_ws/src/` | colcon workspace sources: team packages + `simulation/` packages (tables below) | |
| `simulation/` | the simulator's human side: `README.md`, `setup/` (install, build, env), `scripts/` (demo, start/stop, mission, photo), `profiles/` (plug-in profiles), `docker/` (container helpers), `windows/` (double-click `.bat` launchers) | SIM; `profiles/`, `docker/`, `windows/` WIP |
| `docker/` | `Dockerfile` (targets `dev`, `sitl`, `sim`), headless `compose.sim.yml` + GPU, Linux, WSLg and DRI add-ons, `requirements-dev.txt`, `requirements-ml.txt` | `dev` = team dev container; compose + `sim` additions WIP |
| `.devcontainer/` | `devcontainer.json` (VS Code, image target `dev`), `sim/devcontainer.json` (image `astra-sim`) | dev: used; sim: WIP |
| `.github/workflows/` | `lint-and-format.yml` (`ruff check ros2_ws/src` = the only CI gate that can fail; format check advisory), `ros2-build.yml` (colcon build/test, `continue-on-error`), `validation.yml` (manual placeholder) | CI |
| `scripts/` | `check_versions.sh` (diff container vs Jetson versions), `humanize.sh` (readable launcher timestamps), `watch_status.py`/`.sh` (operator tap of STATUSTEXT + `/camera/status` over SSH) | TOOL, FLOWN |
| `servo_test/`, `servo_wiggle.py` | pymavlink servo bench scripts on the serial port (MAVROS stopped); `servo_wiggle.py'` is a stray copy | TOOL |
| `requirements_jetson.txt` (+ `.lock.txt`) | authoritative Jetson pip pins: torch 2.8.0, torchvision 0.23.0, numpy 1.26.4 (**< 2**), opencv-python 4.10.0.84, ultralytics 8.4.75, sahi 0.11.36, watchdog 6.0.0 | FLOWN |
| `ruff.toml`, `.gitattributes`, `.gitignore` | ruff rules E9/F63/F7/F82, py310, line 100; `*.sh` LF, `*.bat` CRLF; ignores build outputs, photos, `*.pt *.onnx *.engine *.trt` | |
| `COLCON_IGNORE` (root, empty) | colcon never crawls the repo root: build inside `ros2_ws` | |
| `README.md` | dev container, Docker targets, "Simulation (Gazebo)" section | DOC |
| `HowTos.md` (SSH to the Jetson), `things to test.md`, `GtihubLearning.md`, `LearnC++ROS2/`, `drone_ui_functions_plan.md` (stale), `*.mhtml` (saved Anduril Lattice SDK pages) | team notes / reference | DOC |
| `launcher.sh.bak` | old launcher | DEAD |
| `docs/` | `HANDOFF.md` = ordered work/evidence; `simulation/` = human setup, architecture, model/plugin, SITL, world, Docker and operator guides | DOC; unverified runtime flows explicitly marked |

### Team packages (`ros2_ws/src/`)

| Package (build type) | Executable -> ROS node | Status | Notes |
|---|---|---|---|
| `main` (ament_cmake, Python scripts) | `main_controller.py` -> `main_controller` | FLOWN\* | mission brain (hold, aim, divert, drop, RTL). `launch/tracker.launch.xml` = THE flight launch. `src/target_geolocation.py` (drop aiming) + `src/timed_steps.py` (non-blocking timers) are installed next to the script. `capture_detect_drop.py` = bench TOOL (stale servo defaults). `old_main_controller.py`, `msg/` = DEAD |
| `video_cam` (ament_python) | `siyi` -> `siyi` | FLOWN\* | team SIYI driver: `camera_interface.py` (SDK + HTTP), `pipeline_orchestrator.py`, `storage_manager.py`, `config.py`, `capture_queue.py` + `photo_metadata.py` (Oct 2026). `launch/siyi_unified_pipeline.launch.py` is NOT used by the flight launch (defaults `resolution:=4K`). `*.bak*` DEAD. Vendor PDFs in `siyi_documentations/` |
| `detection` (ament_python) | `new_od` -> `new_od` | FLOWN | YOLO26m on 640 px tiles: `model_manager.py`, `batched_inference.py`, `detection_processor.py` (class map), `geotag.py` (lat/lon from file name), `annotation.py`, `gpu_utils.py`. `Documentation/`, `*.bak*` stale |
| `mapping` (ament_python) | `trigger` -> `mission_camera_trigger` | FLOWN\* | `do_digi_cam_trigger.py`: DigiCamCtrl -> 1 Hz photos. `mapping` executable (`mapping.py` stitcher) DEAD: crashes at start, not launched |
| `waypoint_mavros` (ament_python) | `waypoint` -> `waypoint_manager` | FLOWN\* | mission copy, index parameters, `/addWaypoint`; `mission_sync.py` (Oct 2026, pure Python). `killnode` DEAD and dangerous (plaintext sudo password, shuts the PC down) |
| `wp_sender` (ament_python) | library `wp_sender.parameter` (+ `param_tracking.py`); `send_wp`, `test_param` CLI test clients | FLOWN\* (library) | `AsyncParameterReader` (used by main_controller, never blocks); `ParameterManager` = blocking, CLI only |
| `gps_ros2` (ament_python) | `gpstest` -> `gps_mavros_service_node` | FLOWN | sets ALL MAVLink streams to 1 Hz once (`/mavros/set_stream_rate`); serves `/get_drone_data` (no client) |
| `interfaces` (ament_cmake, rosidl) | msg `ImageResult`, `YoloResult`; srv `AddWaypoint`, `DelWaypoint`, `UpdateMission`, `GetGPSData`, `CameraCommand` | FLOWN | `ImageResult` + `AddWaypoint` are used; the rest are unused. All message definitions live HERE |
| `payload` (ament_python) | `payload` -> `servo_controller` | DEAD | superseded (channels 9/10, 1900/1400 us); drops are in main_controller |
| `mainCPP` (package `maincpp`, C++) | `main_controller` (stale C++ port, SAME node name), `siyi_wipe_sd` | DEAD, not launched | `siyi_wipe_sd` formats the camera SD card when RC-out 12 >= 1800 us while disarmed: never run on a dev machine |
| `gps_cpp` (C++) | `gps_node` | DEAD | learning stub |
| `comms/` | not a package (no package.xml): mavlink-router wrapper, `command_listener.py` (COMMAND_LONG 31004 reboot / 31005 shutdown), `ground_sender.py`, `system_controller.py` (plaintext sudo) | DEAD | never launched; never run on a dev machine |
| `ultralytics_ros/` | vendored, has `COLCON_IGNORE` | DEAD | |
| loose files | `yolo26m.engine` (Jetson-only TensorRT, force-committed), `yolo26m.onnx` (unused), `yolo_model_downloader.py`, `ThingsToDo.md`, `HowToHandleMemoryIssues.md`, `fixtensorRT.md` | | |

### Simulation packages (`ros2_ws/src/simulation/`, maintainer astra-sim@example.com)

| Package (build type) | Executables | Status | Job |
|---|---|---|---|
| `sim_models` (ament_cmake) | `payload_release`, `generate_model_sdf` | SIM | drone `models/iris_a8mini/` (`model.sdf.xacro` + pre-generated `model.sdf`), `config/astra_sitl.parm`, payload release logic |
| `sim_worlds` (ament_cmake) | `ros2 run sim_worlds <tool>`: `world_info.py`, `make_mission.py`, `check_world.py`, `render_preview.py`, `build_runway.py`, `build_suas_field.py`, `fetch_models.sh`, `test_sim_worlds.sh` | SIM; real-terrain parts WIP | `worlds/`, `config/`, `missions/`, `truth/`, `models/` (all GENERATED by the build scripts); `sim_worlds/realfield/` + `scripts/build_real_field.py` + `scripts/fetch_real_field_data.{sh,py}` + `data/suas_field_real/` = WIP |
| `siyi_emulator` (ament_python) | `siyi_emulator`, `siyi_client` | SIM | pretends to be the SIYI camera at 192.168.144.25 |
| `sim_bringup` (ament_python) | `wait_then_exec`, `gz_follow`, `odom_tf`, `mission_tool`, `phase_monitor`, `stack_check`, `photo_trigger` | SIM | `launch/sim.launch.py` (the simulator), `launch/team_stack.launch.py` (team stack wrapper), `sim_bringup/sim_paths.py` (every path/port answer; CLI `python3 -m sim_bringup.sim_paths ...`) |
| `sim_demo` (ament_python) | `demo_panel`, `truth_detector`, `demo_markers`, `score_run` | SIM (+ `run_reset.py` WIP) | what you watch + the score report; `run_reset.py` = START OVER |
| `sim_plugins` (ament_python) | `profile_tool`, `check_slot` | WIP | plug-and-play profiles, slot contracts, conformance checker |
| `sim_plugin_examples/{example_detector,example_listener}` (ament_python) | `example_detector`, `example_listener` | WIP | copy-and-rename templates |

## 3. How the real system works

### 3.1 Hosts and links

| Host | Runs | Links |
|---|---|---|
| Ground laptop | Mission Planner (upload mission, arm, start AUTO, show STATUSTEXT); SSH terminal: `./launcher.sh`, `./scripts/watch_status.sh` | MAVLink over a telemetry radio to the Pixhawk; SSH to the Jetson |
| Pixhawk | ArduCopter 4.6.3: flies the AUTO mission; modes GUIDED / AUTO / RTL; PWM outputs 11 (person drop) and 13 (tent drop) | USB serial `/dev/ttyUSB0:115200` to the Jetson |
| Jetson Orin Nano (eth0 192.168.144.100/24) | MAVROS + the 6 team nodes of `tracker.launch.xml` | Ethernet to the camera |
| SIYI A8 mini (192.168.144.25) | gimbal + camera firmware, photos on its microSD | UDP 37260 (SDK), HTTP 82 (media); RTSP 8554 exists, unused. The code asks the model (SDK 0x02) and falls back to the **ZR10** profile (2K stills) |

### 3.2 Nodes (all from `launcher.sh` -> `tracker.launch.xml`; no per-node parameters; `use_sim_time` false)

| Node | Package / executable | Job |
|---|---|---|
| `mavros` | `mavros` / `apm.launch` | MAVLink <-> ROS 2. `fcu_url` = `/dev/ttyUSB0:115200` (hw) or `udp://:14552@localhost:14552` (sim); env overrides `FCU_URL`, `MAVROS_LAUNCH` |
| `waypoint_manager` | `waypoint_mavros` / `waypoint` | mirrors the FCU mission, publishes its indices as its own parameters `num_waypoints, takeoff_index, next_after_takeoff, rtl_index, last_before_rtl, buffer_wp`, serves `/addWaypoint`, re-pulls the mission on `Flight plan received`, on arming and at start-up |
| `gps_mavros_service_node` | `gps_ros2` / `gpstest` | `/mavros/set_stream_rate {stream_id 0, rate 1, on}` once (all telemetry 1 Hz) |
| `mission_camera_trigger` | `mapping` / `trigger` | on `Mission: <n> DigiCamCtrl`: `capture_setup` once, `autofocus` every 5 s, `/camera/trigger` every 1 s; stops at `buffer_wp` |
| `siyi` | `video_cam` / `siyi` | every trigger -> one photo via SDK + HTTP, saved with the GPS fix as its name + a JSON sidecar |
| `new_od` | `detection` / `new_od` | watches the photo folder, YOLO26m, publishes `/image_detection` |
| `main_controller` | `main` / `main_controller.py` | best person + best tent, hold, insert target waypoints, drop, RTL |

### 3.3 Interfaces (exact names; QoS "sensor" = best effort, a number = reliable depth)

| Name | Type | From -> to | Notes |
|---|---|---|---|
| `/mavros/statustext/recv` | `mavros_msgs/StatusText`, sensor | ArduPilot -> trigger, waypoint_manager | `Mission: 3 DigiCamCtrl` starts photos; `Flight plan received` = new mission |
| `/mavros/statustext/send` | `mavros_msgs/StatusText`, severity 6, 10 | trigger, siyi, main_controller, waypoint_manager -> GCS | MAVLink cuts text at 50 chars; the simulator PARSES these texts (section 7) |
| `/mavros/state` | `mavros_msgs/State`, 10 | -> waypoint_manager, gps node, launcher | connected, armed, mode |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList`, 10 | -> waypoint_manager, main_controller | the mission as the FCU has it |
| `/mavros/mission/reached` | `mavros_msgs/WaypointReached` | -> trigger (stop at `buffer_wp`), main_controller (hold/drop), waypoint_manager, new_od | `wp_seq` = mission index (item 0 = home); DO_ items never produce it |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix`, sensor | -> siyi (photo name), gps node | ~1 Hz after gpstest |
| `/mavros/global_position/rel_alt`, `/mavros/global_position/compass_hdg` | `std_msgs/Float64`, sensor | -> siyi (sidecar height above home, heading; focus pass at 15 m), gps node (heading) | |
| `/camera/trigger` | `std_msgs/Bool` (true), 10 | trigger (1 Hz) -> siyi | also the sim panel's TAKE PHOTO |
| `/camera/command` | `std_msgs/String`, 10 | trigger -> siyi | `capture_setup`, `autofocus`, `gimbal_set_angles 0,-90`, `zoom_absolute ...`, `sd_format yes` (dangerous) |
| `/camera/status`, `/camera/disk_free_mb`, `/image_raw` | `String`, `Float64`, `sensor_msgs/Image` | siyi -> watch_status.py / nobody | `/image_raw` = full decoded photo |
| `/image_detection` | `interfaces/msg/ImageResult`, **RELIABLE depth 10** | new_od -> main_controller | one per photo; `class_id` "0" person, "1" tent; bbox centre in photo pixels; float32 lat/lon from the file name |
| `/sahi_detection_results`, `/sahi_detection_info` | `Image`, `String` JSON | new_od -> nobody | |
| `/parameter_events` | `rcl_interfaces/ParameterEvent` | waypoint_manager -> trigger (`buffer_wp`), main_controller (5 indices) | |
| `/addWaypoint` (srv) | `interfaces/srv/AddWaypoint` | main_controller -> waypoint_manager | request `{command 16, latitude, longitude, altitude 49.0, index, channel 0, pwm 0}`; inserts NAV_WAYPOINT frame 3 (relative alt), `param1` 3 s hold, pushes the WHOLE list; always answers success |
| `/waypoint_manager/get_parameters` (srv) | `rcl_interfaces/srv/GetParameters` | trigger, main_controller -> waypoint_manager | async reads of the index parameters |
| `/mavros/mission/pull`, `/push` (srv) | `WaypointPull`, `WaypointPush {start_index 0}` | waypoint_manager -> MAVROS | push after every insert, result not awaited |
| `/mavros/mission/set_current`, `/mavros/set_mode`, `/mavros/cmd/command` (srv) | `WaypointSetCurrent`, `SetMode {custom_mode}`, `CommandLong {command 183, param1 channel, param2 pwm}` | main_controller -> MAVROS | GUIDED / AUTO / RTL; 183 = MAV_CMD_DO_SET_SERVO |
| `/get_drone_data`, `/sahi/get_statistics`, `/sahi/get_health` (srv) | `GetGPSData`, `std_srvs/Trigger` | gps node, new_od serve | health text `... ModelLoaded: True ...` |
| UDP `192.168.144.25:37260` | SIYI SDK frames `55 66 \| ctrl \| len \| seq \| cmd \| data \| CRC16` (XMODEM) | siyi -> camera | shutter = fixed packet `55 66 01 01 00 00 00 0c 00 34 ce` (cmd 0x0C); 0x0B feedback awaited 0.8 s |
| HTTP `192.168.144.25:82` | `/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0`, `.../getmedialist?media_type=0&path=<dir>&start=0&count=9999`, then GET the file URL | siyi -> camera | poll every 0.5 s up to 15 s, download timeout 10 s |
| photo folder | `<ws>/src/video_cam/mapping_photos/"<lat:.6f> , <lon:.6f>.jpg"` + `<same>.json` sidecar (`astra.photo_metadata.v1`) | siyi writes; new_od, main_controller read | `<ws>` = first parent of the installed module with `install/` and `src/` (= `ros2_ws`); atomic `.tmp.jpg` + rename; note the spaces around the comma |
| detection output | `<ws>/src/video_cam/detection_results_sahi/sahi_detected_<name>` | new_od writes | annotated copies |

```
 Mission Planner ==radio, MAVLink==> Pixhawk (ArduCopter 4.6.3) ==PWM 11 / 13==> release servos
                                        || USB serial /dev/ttyUSB0:115200
 Jetson ================================||===============================================================
   mavros <-----------------------------++   publishes statustext/recv, mission/reached, mission/waypoints,
     |                                        global_position/*, state; serves set_mode, cmd/command, mission/*
     +--"Mission: 3 DigiCamCtrl"--> mission_camera_trigger --/camera/trigger 1 Hz + /camera/command--> siyi
     +--mission/waypoints--> waypoint_manager --params (buffer_wp, rtl_index ...)--> trigger, main_controller
                                 ^ /addWaypoint             siyi --UDP 37260 SDK + HTTP 82--> SIYI camera .25
                                 |                           |  "<lat> , <lon>.jpg" + .json sidecar
   main_controller <--/image_detection-- new_od <--inotify-- <ws>/src/video_cam/mapping_photos/
     +--set_mode GUIDED/AUTO/RTL, mission/set_current, cmd/command 183 (DO_SET_SERVO)--> mavros
   gps_mavros_service_node --set_stream_rate (all 1 Hz)--> mavros
   trigger, siyi, main_controller, waypoint_manager --/mavros/statustext/send--> Mission Planner messages
```

### 3.4 Mission flow, step by step

Mission layout the code depends on (indices = MAVLink `seq`; example = sim runway mission, R = 11):
`0 HOME, 1 NAV_TAKEOFF, 2 WP lane start, 3 DO_DIGICAM_CONTROL, 4 DO_CHANGE_SPEED (optional), ... lane WPs,
R-2 last lane end (= buffer_wp: photos stop), R-1 hold point (last NAV before RTL), R NAV_RETURN_TO_LAUNCH`.

1. Pilot uploads the AUTO mission in Mission Planner, then on the Jetson (SSH) runs `./launcher.sh`.
2. `launcher.sh`: pipes output through `scripts/humanize.sh` (unless `ASTRA_RAW_TIME=1`), sources
   `/opt/ros/humble` + `ros2_ws/install`, pings `CAMERA_IP` (warning only), starts MAVROS, makes up to 30
   tries of `ros2 topic echo --once /mavros/state` (any message passes: not a real heartbeat check), starts
   `$TRACKER_CMD`; when either child exits it stops both (no restart, no systemd).
3. Start-up: `waypoint_manager` waits for `connected`, pulls the mission, derives the indices (runway:
   takeoff 1, next_after_takeoff 2, rtl 11, last_before_rtl 10, buffer_wp 9, num_waypoints 12);
   `new_od` loads the model in the background (TensorRT `.engine` next to `model_path` if present);
   `siyi` asks the model (0x02), checks the SD card (0x0A), lists the media folder; `main_controller` blocks
   until `/mavros/set_mode`, `/addWaypoint`, `/mavros/cmd/command` exist and reads the indices asynchronously.
4. Pilot arms and switches to AUTO (the Jetson never arms or takes off). ArduPilot climbs and flies; at
   item 3 it sends `Mission: 3 DigiCamCtrl`.
5. `mission_camera_trigger`: `capture_setup` (zoom rack to `CAPTURE_ZOOM_X` 5.0, then focus, `FOCUS_MODE`
   `infinity`), autofocus every 5 s (skipped on an A8 mini: no autofocus), `/camera/trigger` every 1 s.
6. `siyi` per trigger: queue (max 5 waiting, the oldest is dropped and counted) -> GPS name taken -> shutter
   0x0C -> poll `getmedialist` -> HTTP download -> size check (2K >= 2400x1300, 4K >= 3000x1600) -> save
   photo + sidecar -> STATUSTEXT `SUCCESS: Captured <res> image #<n>`.
7. `new_od`: new file (watchdog/inotify + periodic rescan) -> 640x640 tiles, 15 % overlap, confidence 0.25
   -> class map: `person`/`mannequin`/`doll` -> "0" person, `umbrella`/`kite` -> "1" tent (stock COCO model,
   not fine-tuned) -> one `ImageResult` per photo (also empty ones).
8. `main_controller` keeps the highest-confidence person and tent and their AIM point: the box centre
   projected to the ground with the sidecar (height, heading, field of view, zoom, gimbal pitch,
   `image_top_direction_deg`) by `target_geolocation.aim_at_box`; no sidecar -> the photo's own position;
   no GPS in the photo -> ignored.
9. `/mavros/mission/reached == buffer_wp` (R-2) -> trigger stops (`Camera trigger STOPPED`).
10. Reached the last NAV item before RTL (R-1) -> `Reached last WP. Holding ...`, mode **GUIDED**; every 2 s
    it compares the image files in `mapping_photos` with the names seen on `/image_detection`; it goes on
    when all are processed or after 90 s.
11. Divert: plan = tent first, then person (only classes with confidence > 0). Nothing -> `No valid targets
    found. Returning to launch.` + RTL. Else one `/addWaypoint` per target, sent in reverse with index
    `last_before_rtl + 1` (tent lands at R, person at R+1, RTL moves to R+2); STATUSTEXT `Aim tent <lat>,<lon>`
    and `Targets: tent@11 -> person@12. Diverting.`; after 4 s `set_current` to the first target, 1 s later
    **AUTO** (`Resumed AUTO -> WP 11`).
12. Over each target (`reached == target wp`): `Over tent: loiter 6s then drop`, **GUIDED**, 6 s,
    `DO_SET_SERVO` channel **13 tent / 11 person** at **1400 us (open)**, 3 s, **500 us (closed)**, then AUTO
    to the next target.
13. After the last drop: `All targets done. Waiting 6s then RTL.`, 6 s, **RTL**, `RTL`. ArduPilot flies home
    and lands. Nothing builds a map on board.

| Phase | Mode | Who is in charge |
|---|---|---|
| upload, arm, start | -> AUTO | pilot in Mission Planner (sim: `mission_tool` / panel / Mission Planner with `--manual`) |
| take-off, survey lanes, photos | AUTO | ArduPilot flies; trigger + siyi + new_od work alongside |
| hold for the detector (<= 90 s) | GUIDED | main_controller |
| fly to targets | AUTO | ArduPilot, on the waypoints main_controller inserted |
| each drop (6 s + 3 s) | GUIDED | main_controller |
| home | RTL | main_controller commands it; ArduPilot lands |

Constants (`main_controller.py` unless noted): `ALT` 49 m relative to home; `PROCESSING_TIMEOUT_SECONDS` 90;
`PUSH_SETTLE_SECONDS` 4; `SET_CURRENT_TO_AUTO_SECONDS` 1; `LOITER_SECONDS` 6; `SERVO_OPEN_SECONDS` 3;
`PULLEY_OPEN` 1400 / `PULLEY_CLOSE` 500; `PERSON_SERVO_CHANNELS [11]`, `TENT_SERVO_CHANNELS [13]`; trigger
`capture_period_s` 1.0, `autofocus_period_s` 5.0 (parameters); video_cam `MAX_PENDING_CAPTURES` 5,
`CAPTURE_ZOOM_X` 5.0. main_controller parameters: `aim_at_detection_box` true, `default_camera_pitch_deg` -90,
`default_camera_hfov_deg` 81, `target_ground_height_m` 0, `max_aim_offset_m` 150; video_cam parameter
`image_top_direction_deg` 0 (= top of the photo toward the nose; must be checked on the real drone).

## 4. How the simulator works

### 4.1 The two doors

The team code touches the world only through **MAVLink** (MAVROS) and the **SIYI camera protocol** at
192.168.144.25. The simulator opens exactly those two doors, so `./launcher.sh sim` runs unchanged:

```
 Ubuntu 22.04 (native or WSL2)                                  GZ_PARTITION=astra_sim, ROS_DOMAIN_ID 0
 sim.launch.py ------------------------------------------------------------------------------------------
  gz sim -s -r <world>.sdf (+ window gz sim -g)     drone entity "iris" = sim_models iris_a8mini (xacro)
   ArduPilotPlugin <==UDP 9002+10I, JSON, lock-step==> arducopter --model JSON -I<I> --home <config home>
                                                        --defaults copter.parm,gazebo-iris.parm,astra_sitl.parm
                                                         | TCP 5760+10I (serial0; 5762/5763+10I spare)
                                                       mavproxy.py --daemon  --out udp:127.0.0.1:14550 (scripts/GCS)
                                                         --out :14552 (MAVROS)  --out :14553 (demo panel)
                                                         [--out <Windows host>:14550, only inside WSL]
   gz /sim/gimbal/*/cmd_pos, /sim/camera/photo/trigger <-- siyi_emulator on 192.168.144.25 (UDP 37260, TCP 82)
   gz /sim/camera/photo/image --> siyi_emulator --> SD card ~/.astra_sim/sd_card_I<I>/101SIYI_IMG/IMG_NNNN.jpg
   gz /sim/payload/servo11|13 --> payload_release --> gz /sim/payload/person|tent/detach (the payload falls)
   ros_gz_bridge, odom_tf, sim_demo (demo_panel, demo_markers, score_run [, truth_detector]), rviz2, gz_follow
 ./launcher.sh sim  with TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py detector:=real" --------
   MAVROS fcu_url udp://:14552@localhost:14552 + the team's tracker.launch.xml UNCHANGED
   video_cam talks SDK + HTTP to 192.168.144.25 (a loopback alias on this PC) = the emulator
```

`team_stack.launch.py` includes `tracker.launch.xml` as is and only adds global parameters that only
`new_od` declares: `model_path` (default `~/.astra_sim/models/yolo26m.pt` or `$ASTRA_DETECTOR_MODEL`; the team
default is a Jetson-only file), `use_batched_inference` false (the team's batched path only works with the
Jetson's TensorRT `.engine`; `batched:=true` tests it), and with `detector:=truth` a `camera_feed_path` that is
always empty (`~/.astra_sim/truth_mode_empty_feed`). Arguments: `detector real|truth|custom`, `mission_brain team|custom`, `model_path`, `batched`,
`debug`, `profile`.

### 4.2 Ports and addresses (I = SITL instance, default 0)

| What | Where | Used by |
|---|---|---|
| SITL MAVLink | TCP `5760+10I` (serial0) | MAVProxy (SITL waits for it) |
| SITL spare MAVLink | TCP `5762+10I`, `5763+10I` | second ground station, e.g. Mission Planner by TCP 5762 (keeps a scored run's rates clean) |
| Physics link | UDP `9002+10I` (JSON, lock-step) | ArduPilotPlugin <-> SITL |
| Ground station / scripts | UDP `127.0.0.1:14550` | `mission_tool` (`--url udpin:127.0.0.1:14550`), or one GCS inside Ubuntu |
| MAVROS | UDP `127.0.0.1:14552` | `./launcher.sh sim` |
| Demo panel buttons | UDP `127.0.0.1:14553` | `demo_panel` |
| Windows GCS (WSL only) | UDP `<default gateway>:14550` (NAT) or `127.0.0.1:14551` (mirrored networking) | Mission Planner / QGC on Windows; off: `windows_gcs:=false` / `--no-windows-gcs`; elsewhere: `windows_gcs_addr:=HOST:PORT` |
| SIYI camera | `192.168.144.25` UDP 37260 + TCP 82 | video_cam -> siyi_emulator (browse `http://192.168.144.25:82/`) |
| Gazebo transport | `GZ_PARTITION=astra_sim`, `GZ_IP=127.0.0.1` (set by `sim_env.sh`) | keeps other Gazebos out |

Two simulators at once: `start_sim.sh runway sitl_instance:=1 gcs_port:=14560 mavros_port:=14562
panel_port:=14563` with `GZ_PARTITION=other`, then `FCU_URL=udp://:14562@localhost:14562 ./launcher.sh sim`.

### 4.3 Simulator-only topics (`/sim/...`)

| Name | Kind / type | Notes |
|---|---|---|
| `/sim/camera/image`, `/sim/camera/camera_info` | ROS (bridged from gz `/sim/camera/video/image`, `.../camera_info`) | live POV 1280x720 @ 25 Hz, HFOV 1.4137 rad (81 deg); lazy bridge |
| `/sim/camera/photo` | ROS (from gz `/sim/camera/photo/image`) | every rendered photo; gz trigger `/sim/camera/photo/trigger` (gz.msgs.Boolean) |
| `/sim/drone/odometry` | ROS `nav_msgs/Odometry` (from gz) | world ENU, origin = SITL home |
| `/sim/gimbal/joint_states` | ROS `sensor_msgs/JointState` (from gz.msgs.Model) | gimbal angles |
| `/sim/gimbal/{yaw,roll,pitch}/cmd_pos` | gz.msgs.Double, rad | yaw + = LEFT, roll + = right side down, pitch -pi/2 = straight down; limits yaw +-160, roll +-30, pitch -135..+45 deg |
| `/sim/camera/photo_info`, `/sim/camera/emulator_state` | ROS `std_msgs/String` JSON (photo_info RELIABLE + TRANSIENT_LOCAL) | emulator's exact record per photo / zoom + gimbal state |
| `/sim/payload/servo11`, `/servo13` | gz.msgs.Double, us | ArduPilotPlugin copies of SITL outputs 11 / 13 |
| `/sim/payload/{person,tent}/detach`, `/odometry` | gz Empty / Odometry | release + landing point |
| `/sim/payload/events` | ROS `std_msgs/String` JSON | released / detached / landed / attached |
| `/sim/demo/markers` | ROS `visualization_msgs/MarkerArray` (latched, frame `world`) | RViz layers |
| `/sim/demo/write_score` | ROS srv `std_srvs/Trigger` | score_run writes the report now |

### 4.4 Drone, camera emulator, payloads

- `sim_models/models/iris_a8mini`: ArduPilot Iris (motors on SITL channels 1-4) + SIYI A8 mini 3-axis
  gimbal under the nose + a live camera and a triggered photo camera: **2560x1440** with `camera_model:=zr10`
  (default, mirrors video_cam's ZR10 fallback) or **3840x2160** with `a8mini`. Spawned as entity `iris`.
- `config/astra_sitl.parm` (loaded after ArduPilot's `copter.parm` + `gazebo-iris.parm`): `SERVO11_FUNCTION 0`,
  `SERVO13_FUNCTION 0` (DO_SET_SERVO needs disabled outputs), MIN/TRIM 500, MAX 1900, `MNT1_TYPE 0`,
  `AUTO_OPTIONS 3` (arm + take off in AUTO without sticks), `FENCE_ENABLE 0`, `FS_GCS_ENABLE 0`,
  `PLND_ENABLED 0`, battery failsafes warn only, `SIM_SPEEDUP 1` (team code uses wall-clock sleeps). Every line
  < 90 characters (ArduPilot reads 100-byte chunks; `test_astra_sitl_parm.py` checks it).
- Payloads: person = 0.5 L water bottle on servo 11, tent = GPS beacon box on servo 13, held by
  DetachableJoints; `payload_release` detaches when the servo reaches 1400 us.
- `siyi_emulator`: answers the SDK and the media API like the real camera; `hwid` per camera model (ZR10
  `6B...`, A8 mini `73...`); on 0x0C it triggers one Gazebo frame, sends 0x0B within 0.5 s, applies digital
  zoom, writes `IMG_NNNN.jpg` + `IMG_NNNN.json`; no frame in 10 s -> a labelled test pattern. demo.sh /
  start_sim.sh start it with `camera_max_zoom:=1` (`--zoom survey`, default) so photos keep the 81 deg view
  the lanes were planned for; video_cam then logs `Could not confirm 5.0x after 3 attempts` (expected).
  `--zoom team` allows the 5x rack. Hand tool: `ros2 run siyi_emulator siyi_client hwid|angles 30 -45|photo --out /tmp/p.jpg`.

### 4.5 Worlds (`sim_worlds`, all files generated by its scripts)

| World | What | Mission |
|---|---|---|
| `runway` (default) | `build_runway.py` seed 7: pad, runway, 120 x 100 m search box with 1 hiker + 1 tent + distractors | `missions/runway_survey.waypoints`: 30 m, 3 lanes, 5.5 m/s, 12 items, DigiCamCtrl at 3, buffer_wp 9, hold 10, RTL 11; lane 1 starts 16 s early (`lead_in_s`) for the camera set-up |
| `suas_field` | `build_suas_field.py` seed 2026: SUAS 2026 field (Skyway Range, Oklahoma), Search Boundaries 1 and 2, each 1 hiker + 1 tent + distractors | `suas_field_survey.waypoints` (SB1, 50 m) + `suas_field_sb2_survey.waypoints` |
| `standin` | ardupilot_gazebo's runway, no targets | needs `--mission FILE` |
| `suas_field_real` | **WIP**: real terrain (USGS 3DEP lidar, NAIP 2023, OSM, CC0 textures); generator + site pack only, no world/config/mission/truth yet | `docs/simulation/WORLDS.md` (being written) |

`config/<world>.yaml`: `world_name`, `world_file`, `home "lat,lon,alt_msl,heading"` (SITL `--home`; MUST equal
the world's `<spherical_coordinates>`; both worlds: `36.2143000,-96.0068000,210.0,0`), `spawn` (yaw 1.5708 =
nose north), `mission`, `truth`, `search_areas`, `survey`, `gui`. `truth/<world>.yaml` = answer key (targets,
distractors, ArduPilot-convention lat/lon). Raw GIS downloads go to `~/.astra_sim/world_data` (never committed).

### 4.6 What you watch (`sim_demo`)

- `demo_panel` (Qt): status pills (MAVROS, CAMERA, PHOTOS, DETECTOR REAL/STAND-IN), phase strip, live camera,
  newest photo with boxes, detection list + what main_controller will fly to, telemetry, events, buttons
  **TAKE PHOTO** (`/camera/trigger`), **LOOK DOWN / LOOK FORWARD** (`/camera/command gimbal_set_angles 0,-90 / 0,0`),
  **UPLOAD MISSION + START** (MAVLink on 14553, fallback MAVROS services, then asks MAVROS to re-pull),
  **RTL**, **OPEN / CLOSE MISSION PLANNER** (Windows, via WSL interop `tasklist.exe` / `taskkill.exe`);
  **START OVER** (confirm with a second click; unit-tested, runtime acceptance pending in 4.11).
- `truth_detector` (`--detector truth`): publishes the same `ImageResult` on `/image_detection` from the
  answer key + photo pose (`geolocation:=projected` reports target positions); the panel and report say so.
- `demo_markers` (RViz), `score_run` (writes the report when the drone has landed, ~8 s after).
- Gazebo window: chase view (`gz_follow`, `/gui/follow`), live camera panel; RViz: camera + markers.

### 4.7 demo.sh, run folders, score report

`demo.sh`: 1 start `sim.launch.py` (log `sim.log`) -> 2 `mission_tool wait-ready` (GPS + EKF + pre-arm;
~30-60 s of `Arm: Gyros inconsistent` is normal) + upload -> 3 `./launcher.sh sim` (log `team_stack.log`) +
`stack_check --settle 20` (MAVROS connected, the 6 team nodes up, `buffer_wp >= 0`, model loaded; one restart
of the team stack if not) -> 4 `phase_monitor` -> 5 `mission_tool fly` (arm + AUTO) -> 6 wait for LANDED,
score report, copy photos, stop everything (`--keep` leaves it running). Phases: WAITING, ARMED, TAKEOFF,
SEARCH, HOLD, TARGETS, DROP, RTL, LANDED. Old team photos are moved to `~/.astra_sim/photo_archive/<time>/`
first (new_od would re-process them and could pick an old target).

Run folder `~/.astra_sim/runs/<YYYYmmdd-HHMMSS>_<world>/`: `demo.env`, `sim.log`, `team_stack.log`,
`phases.jsonl`, `events.jsonl`, `photos/` (+ `.json` sidecars), `detections/`, `score_report.md` / `.json`,
`run_data.json` (re-score: `ros2 run sim_demo score_run --rescore DIR`), `mission_map.png`. The report lists
targets found, best confidence, location error, **drop error** (payload resting point in Gazebo vs truth),
per-photo precision/recall, false alarms, time line. The **AIM ERROR** column =
main_controller's `Aim <class> <lat>,<lon>` STATUSTEXT vs truth (`sim_demo/scoring.py`, `test/test_score_aim.py`).

### 4.8 Plug-and-play profiles and slots (WIP)

- Slots: `world`, `autopilot`, `camera`, `detector`, `mission_brain`, `extras` (+ `network`, `run` settings).
  A profile is YAML in `simulation/profiles/NAME.yaml` (or any path) holding only the lines that differ
  from `default.yaml`; `extends:` and `--set key=value` merge more. Shipped: `default`, `truth_detector`,
  `example_detector`, `example_listener`, `ardupilot_version` (4.7.1), `ardupilot_source`,
  `real_drone_params` (+ `params/example_real_drone.param`), `external_autopilot`, `docker_split`.
- Use: `demo.sh --profile NAME|FILE [--set K=V] [--dry-run]`, `start_sim.sh --profile ...`, launch arg
  `profile:=`. With no plug-in option every script and launch file runs exactly as before (sim_plugins is
  not even imported). A custom detector / brain runs instead of `new_od` / `main_controller.py` through a
  filtered copy of `tracker.launch.xml` (`sim_plugins/tracker_filter.py`).
- Tools: `python3 -m sim_plugins.profile_tool list | validate [--check] | plan --profile X --check | resolve |
  contract --slot detector [--markdown] | shell --for demo|start_sim`; `ros2 run sim_plugins check_slot
  --slot detector|mission_brain|camera|autopilot|world|extras [--take-photo]` checks a running component
  against `sim_plugins/contracts.py`. Guide: `docs/simulation/PLUGINS.md`; generated references are checked by tests.

### 4.9 Pluggable ArduPilot (WIP)

Default: `~/sitl/arducopter` = ArduCopter 4.6.3 (`02_setup_ardupilot.sh`). Other choices (demo.sh options /
launch args): `--sitl-version V` / `sitl_version:=V` after `bash simulation/setup/fetch_sitl.sh V|stable|beta|latest`
(-> `~/sitl/versions/V/{arducopter,copter.parm,gazebo-iris.parm,SOURCE.txt}`; x86_64 only); `--sitl-source DIR`
(built ArduPilot tree, e.g. `/opt/ardupilot` in the `sitl` image); `--sitl-bin FILE`; `--param-file FILE`
(e.g. the real drone's Mission Planner `.param`; hardware lines dropped by the `safe` filter; repeatable);
`--autopilot external --mavlink tcp:HOST:5760` (autopilot on another PC/container; `fdm_addr:=`). Guide:
`docs/simulation/ARDUPILOT.md` (alternate firmware/runtime acceptance still pending).

### 4.10 Docker (WIP: compose files parse, images never built or run as of 2026-10-24)

- Images from `docker/Dockerfile`: `astra-dev` (target `dev`: Humble ros-base, MAVROS from snapshot
  2026-08-07, mavlink-routerd, Python ML stack; = the Jetson's software), `astra-sitl` (+ SITL built from
  `Copter-4.6.3` in `/opt/ardupilot`), `astra-sim` (+ Gazebo Harmonic, ardupilot_gazebo pinned commit,
  MAVProxy, PyQt5, Mesa d3d12; amd64 only). Build: `bash simulation/docker/build_images.sh [dev|sitl|sim]
  [--jobs 4] [--torch auto|cpu|cuda]` (first time ~1 h, ~25 GB; logs `~/.astra_docker/logs/`).
- Engine: `simulation/docker/install_docker_engine.sh` installs Docker **Engine** + NVIDIA Container Toolkit
  inside the Ubuntu-22.04 distro (images stay on that distro's disk); do not also turn on Docker Desktop's
  WSL integration for it.
- Like the real drone (`docker/compose.sim.yml`, network `astra_net` 192.168.144.0/24, gateway .1):
  container `sim` (image astra-sim) at **192.168.144.25** = the camera address, runs
  `simulation/docker/container_sim.sh` (Gazebo, SITL, MAVProxy -> `192.168.144.100:14552`, emulator, panel,
  score); container `jetson` (image astra-dev) at **192.168.144.100** = the Jetson's eth0, runs
  `container_jetson.sh` -> `./launcher.sh sim` unchanged. Both mount the same repo; separate colcon volumes
  (`astra_sim_ws_*`, `astra_jetson_ws_*`); host data `~/.astra_docker` (mounted as `~/.astra_sim`);
  `ROS_DOMAIN_ID` 0, `ROS_LOCALHOST_ONLY` 0, `ipc: host`. Add-on files (being reworked 2026-10-24; in the
  working tree `compose.sim.yml` alone is headless): `compose.wslg.yml` (WSL2: windows via WSLg, 3D via
  `/dev/dxg` + Mesa d3d12), `compose.linux.yml` (Linux desktop X11), `compose.dri.yml` (Linux `/dev/dri`
  graphics), `compose.gpu.yml` (NVIDIA card: CUDA + OpenGL via the Container Toolkit); `compose_up.sh` picks them.
- Run: `bash simulation/docker/compose_up.sh [world] [--detector real|truth] [--camera zr10|a8mini]
  [--headless] [--manual] [--keep] [--no-gpu]`; stop: `compose_down.sh [--restore-camera-ip] [--solo]`;
  one container: `run_sim_container.sh [--demo runway ...]`; Jetson stand-in alone:
  `run_jetson_container.sh --net astra|host|bridge [--launcher]`; VS Code: `.devcontainer/sim`.
- Another PC: `docs/simulation/DOCKER.md` includes source setup and explicit image save/load commands.
  Export/import wrapper scripts and clean-copy runtime acceptance remain Task 6; GPU argument tests pass.

### 4.11 START OVER and Mission Planner mode (WIP: unit-tested, never flown)

- `bash simulation/scripts/start_over.sh [--fly] [--follow] [--mission FILE] [--land land|rtl]
  [--detector real|truth] [--no-team-stack]` (~2-3 min, windows stay open): lands, stops the team stack,
  moves photos into the OLD run folder, removes the drone **while SITL still runs** (lock-step: stopping SITL
  first freezes Gazebo), stops SITL + MAVProxy, spawns a fresh drone with both payloads, restarts SITL +
  MAVProxy with the same command lines, new run folder, uploads, restarts the team stack, `--fly` arms + AUTO.
  Code: `sim_demo/run_reset.py` (`reset`, `watch-run`, `guard`, `mp-steps`; lock `~/.astra_sim/state/start_over.lock`).
  The panel button confirms with two clicks within 5 s, runs `start_over.sh` in the background, and shows
  progress in Events. Concurrent attempts preserve active progress; reopening the panel restores lockout.
  Reset stops only session-owned processes and refuses drone removal unless landing is verified (or the
  drone is confirmed absent). Demo handover leaves scoring/photos to the reset, preventing mixed runs.
- `demo.sh --manual`: starts everything, uploads/arms nothing; prints Mission Planner steps (connect UDP
  14550, Load WP File, Write WPs, Arm, Auto); timeout 180 min; stays open.
- `simulation/windows/*.bat` (double-click on Windows, distro `Ubuntu-22.04`): `Rebuild.bat` (03_build.sh),
  `Start Search Demo.bat` (`demo.sh runway --keep`), `Start for Mission Planner.bat` (`--manual`),
  `Start Over.bat` (`start_over.sh --fly --follow`), `Stop Simulation.bat` (`stop_sim.sh`).

## 5. Build, run and test

### 5.1 One-time setup (Ubuntu 22.04; ~30 min + downloads)

```bash
sudo bash simulation/setup/01_install_humble.sh   # Humble desktop, Gazebo Harmonic + ros-humble-ros-gzharmonic,
                                                  # MAVROS (snapshot pin), Python stack, WSL GPU vars, port 82, camera alias
bash simulation/setup/02_setup_ardupilot.sh       # NO sudo: ~/sitl/arducopter 4.6.3, ~/ardupilot_gazebo, ~/sitl/venv (MAVProxy)
bash simulation/setup/03_build.sh --workers 4     # copy to ~/ODLC_Machine_Inferencing_System_2025-2026 + colcon build + models
```

No sudo password in WSL? From PowerShell: `wsl -d Ubuntu-22.04 -u root bash "<repo path in WSL>/simulation/setup/01_install_humble.sh"`
(same for `camera_ip_alias.sh`). Jetson / plain dev container: `cd ros2_ws && colcon build --symlink-install`.

### 5.2 Building: why the Linux-side copy

On Windows the checkout on the Windows disk is the master copy (edit there; a checkout on a Linux disk builds
in place). `/mnt/<drive>` paths are slow and often contain spaces, which Gazebo and colcon do not
tolerate, so `03_build.sh` rsyncs the repo (`--delete`; skips `.git`,
`ros2_ws/{build,install,log}`, photo folders) to **`~/ODLC_Machine_Inferencing_System_2025-2026`** and runs
`colcon build --symlink-install --parallel-workers N --packages-ignore ultralytics_ros` there (refuses a build
folder with spaces). **Edits made inside the Linux copy are overwritten by the next 03_build.sh.** Options:
`--no-sync`, `--no-models`, `--clean`, `--packages "a b"` (= `--packages-up-to`), `--workers N` (default 4),
`--add-to-bashrc`. It also downloads Gazebo Fuel models (`sim_worlds/scripts/fetch_models.sh`, ~110 MB) and
`yolo26m.pt` (~40 MB) into `~/.astra_sim/models/`, and adds the camera address. Log: `~/.astra_sim/logs/build_<time>.log`.
In a terminal: `source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh` (ROS +
workspace + GPU + Gazebo paths; never changes `ROS_DOMAIN_ID`).

### 5.3 Running

```bash
S=~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts
bash $S/demo.sh runway [--headless] [--detector real|truth] [--camera zr10|a8mini] [--zoom survey|team]
                       [--keep] [--manual] [--dry-run] [--timeout MIN] [--mission FILE] [--no-windows-gcs]
                       [--windows-gcs HOST[:PORT]] [--profile NAME|FILE] [--set K=V] [--launch-arg name:=value]
# by hand: terminal 1 simulator, terminal 2 team code, terminal 3 ground crew
bash $S/start_sim.sh runway [name:=value ...] [--headless] [--background] [--keep-photos] [--profile ...] [--dry-run]
cd ~/ODLC_Machine_Inferencing_System_2025-2026 && source simulation/setup/sim_env.sh && \
  TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py" ./launcher.sh sim     # wait for "Model ready"
bash $S/fly_mission.sh runway [--watch]      # upload + MAVROS re-read, wait for settling, arm, AUTO
bash $S/take_photo.sh [--direct]             # one photo (team topic, or straight to the emulator)
bash $S/upload_mission.sh [world|file]   bash $S/status.sh   bash $S/stop_sim.sh [--all]
bash $S/start_over.sh [--fly]            bash $S/mission_planner.sh start|stop|status   # exit 0 running, 3 not
ros2 launch sim_bringup sim.launch.py --show-args   # world, camera_model, gui/rviz/panel, detector, sitl_instance,
                                                    # gcs/mavros/panel_port, windows_gcs, emulator, model, x/y/z/yaw ...
# real drone (on the Jetson over SSH, after colcon build in ros2_ws): mission uploaded first, photo folder empty
./launcher.sh                    # hw: MAVROS on /dev/ttyUSB0:115200 (FCU_URL=... to override)
./scripts/watch_status.sh        # every STATUSTEXT + /camera/status; or: ros2 topic echo /camera/status
# headless SITL without Gazebo or camera (root README, image astra-sitl): in one terminal
#   sim_vehicle.py -v ArduCopter --no-rebuild --out=udp:127.0.0.1:14552     then: ./launcher.sh sim
```

Every script in `simulation/scripts/` (and `03_build.sh`, `fetch_sitl.sh`, the user-facing
`simulation/docker/*.sh`) prints its header with `--help`. `stop_sim.sh` stops only processes carrying this user's
`ASTRA_SIM_SESSION` tag (never other simulations). Mission Planner (Windows): connection UDP, port 14550
(or TCP 127.0.0.1:5762); only watch during demo.sh, `Read` is fine, `Write`/mode changes take over.

### 5.4 Tests

Pure-Python suites run with plain pytest from the package folder (no build needed for most; source
`sim_env.sh` first in the Linux copy so ROS and `interfaces` import). Verified 2026-10-24 in Ubuntu-22.04:
19 packages build; 932 functional tests pass across these suites plus Docker helper tests. Exclude the
template lint tests and heavy acceptance/e2e directories as shown in HANDOFF section 3.

| Package | Command (from the package folder unless shown) | Result |
|---|---|---|
| `main` | `python3 -m pytest -q test` (also `colcon test`: `ament_add_pytest_test` in CMakeLists) | 51 pass |
| `video_cam` | `python3 -m pytest -q test` | 24 pass |
| `waypoint_mavros` | `python3 -m pytest -q test/test_mission_sync.py` | 15 pass |
| `wp_sender` | `python3 -m pytest -q test --ignore=test/test_flake8.py --ignore=test/test_pep257.py --ignore=test/test_copyright.py` | 17 pass |
| `mapping` | same lint exclusions, `python3 -m pytest -q test ...` | 20 pass (real rclpy node in an isolated domain) |
| `detection` | `python3 -m pytest -q test/test_latlon_filename.py` | 6 pass |
| lint-only (ament templates) | `test_flake8.py`, `test_pep257.py`, `test_copyright.py` in detection, mapping, payload, waypoint_mavros, wp_sender (payload has no functional test) | pre-existing failures; excluded from functional suites |
| `sim_bringup` | `python3 -m pytest -q test`; `bash test/run_tests.sh` (+ launch args, shellcheck) | 55 pass |
| `sim_models` | `python3 -m pytest -q test --ignore=test/acceptance`; flight acceptance `test/acceptance/run_acceptance.sh` (heavy) | 63 pass |
| `sim_worlds` | `python3 -m pytest -q test`; `ros2 run sim_worlds test_sim_worlds.sh [--quick]` | 62 pass (includes synthetic terrain regressions) |
| `siyi_emulator` | `python3 -m pytest -q test --ignore=test/e2e` (~45 s, loopback ports); `scripts/run_tests.sh` | 147 pass |
| `sim_demo` | `QT_QPA_PLATFORM=offscreen python3 -m pytest -q test --ignore=test/ros_e2e`; fake ROS e2e is separate | 234 pass |
| `sim_plugins` (runtime WIP) | `python3 -m pytest -q -p no:cacheprovider test` (whole repo layout + ROS sourced) | 219 pass, including generated documentation freshness |
| `example_detector` / `example_listener` (runtime WIP) | `python3 -m pytest -q test` | 10 / 3 pass |
| Docker host helpers | from repo: `python3 -m pytest -q simulation/docker/test` (no daemon needed) | 6 pass |
| whole repo (CI gate) | `ruff check ros2_ws/src` from `<repo>` | clean |
| colcon way | `cd ros2_ws && colcon test --packages-select <pkg> --event-handlers console_direct+ && colcon test-result --verbose` | needs a built workspace |

Add `-p no:cacheprovider` (and `PYTHONDONTWRITEBYTECODE=1`) to keep caches out of the repo.

### 5.5 Time, resources, logs

| Item | Value |
|---|---|
| install / first build / rebuild | ~30 min / a few min / ~1-2 min |
| autopilot settling after SITL boot | ~30-60 s (`Arm: Gyros inconsistent`, `PreArm: ...` are normal; scripts wait) |
| runway demo | ~5 min of flight (6-10 min in total); verified run 2026-10-22: 4:59 |
| START OVER | ~2-3 min |
| memory | simulator ~3 GB + detector ~3 GB; scripts warn below 3000 MB free (`free -m`) |
| Gazebo real-time factor | 0.7-0.9 is normal; < 0.5: check `glxinfo -B` says `D3D12 (NVIDIA ...)` |

Outputs: run folders `~/.astra_sim/runs/` (4.7); `~/.astra_sim/logs/` (start_sim, build, start_over);
`~/.astra_sim/sitl_I<I>/` (`eeprom.bin` = saved parameters + mission, `mav.tlog`); `~/.astra_sim/sd_card_I<I>/`;
`~/.astra_sim/state/` (`current_launch.yaml` = what the running simulator resolved, `session`, `team_stack.env`);
`~/.astra_sim/generated/` (drone SDF, bridge list); ROS logs `~/.ros/log`; team photos
`~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/video_cam/mapping_photos/`. `$ASTRA_SIM_HOME`
overrides `~/.astra_sim`. Docker: `~/.astra_docker`.

## 6. Rules for changing code here

1. **The team code must keep running unchanged on the real drone AND in the simulator.** The simulator
   adapts to the team code, never the reverse. Never put simulator-only names (`/sim/...`, emulator ports,
   `~/.astra_sim`) into team packages; the only sim hook in team files is `launcher.sh`'s `TRACKER_CMD`.
2. **Default behaviour stays exactly as today.** New behaviour goes behind a declared ROS parameter, launch
   argument, script option or profile key whose default reproduces the current behaviour.
3. **Purpose paragraph at the top of every file you create or touch** (add one if missing), before imports:
   Python module docstring `<file> -- PURPOSE: ...`; shell: a `#` block right after the shebang (scripts print
   it as `--help`); XML/SDF/launch XML: `<!-- -->`; YAML/.parm/CMake: `#`; `.bat`: `rem`; Markdown: an HTML comment.
4. Comments explain WHY; docstrings on functions; plain words for student teammates (explain each term once,
   copy-paste commands, troubleshooting tables). Team style: Python rclpy, ament_python (ament_cmake only
   where it already is), last year's package names.
5. **Tests with every change.** Put logic in ROS-free modules (like `mission_sync.py`, `param_tracking.py`,
   `capture_queue.py`, `photo_metadata.py`, `target_geolocation.py`, `timed_steps.py`) and test them with pytest.
6. **Never block a callback** (single-threaded executors): no `time.sleep`, `spin_until_future_complete` or
   waiting loops in callbacks; use `call_async` + done callbacks, timers, `timed_steps.StepRunner`,
   `wp_sender.parameter.AsyncParameterReader`. A blocking read is what crashed main_controller.
7. Message and service definitions live in `interfaces` only (`main/msg/` is dead). Keep `/image_detection`
   RELIABLE depth 10 (a BEST_EFFORT publisher never reaches main_controller).
8. **Never commit** raw data (GIS downloads, photos, tlogs), build outputs (`ros2_ws/build|install|log`),
   model weights (`*.pt *.onnx *.engine`), secrets, personal e-mail addresses or machine-specific absolute paths
   (derive paths at run time: `$HOME`, `sim_paths`, the script's own folder). The repo is **public**.
   New packages use maintainer e-mail **astra-sim@example.com** (do not copy the addresses in older package.xml files).
9. Line endings (`.gitattributes`): `*.sh` LF (bash fails on CRLF), `*.bat` CRLF.
10. `ruff check ros2_ws/src` must stay clean (the CI gate).
11. Git: commit, push, rebase or switch branches only when your human asks; branches are per person
    (`aro`, `jarrod-gazebo-sim`, ...). Never run `killnode`, `comms/system_controller.py`, `siyi_wipe_sd` or the
    `sd_format` camera command on a development machine.
12. Never make the simulator run faster than real time (`SIM_SPEEDUP 1`): the team code sleeps on the wall clock.

## 7. Common tasks (recipes)

**Add a detector (simulator, WIP mechanism).** 1) Copy `ros2_ws/src/simulation/sim_plugin_examples/example_detector`
to a new package and rename package/module/node/launch. 2) Meet the detector contract
(`sim_plugins/contracts.py`): for every new photo in `<ws>/src/video_cam/mapping_photos` publish exactly one
`interfaces/msg/ImageResult` on `/image_detection` (RELIABLE, depth 10), also when nothing is found
(main_controller waits for every photo); `image_name` = file name; per box `results[0].hypothesis.class_id`
"0" person / "1" tent and `score`; `bbox.center.position.x/y` = box centre in photo pixels (used for aiming);
`latitude/longitude` = values from the file name; optional `/sahi/get_health` answering `ModelLoaded: True`.
3) Copy `simulation/profiles/example_detector.yaml`, set `detector.custom.package`, `launch` (or `executable`),
`node_name`, `provides_health`. 4) `python3 -m sim_plugins.profile_tool plan --profile my.yaml --check`, then
`bash simulation/scripts/demo.sh --profile my.yaml`, and while it runs `ros2 run sim_plugins check_slot --slot
detector --take-photo`. 5) On the real drone a new detector must be added to `main/launch/tracker.launch.xml`
in place of `new_od` (profiles are simulator-only).

**Change the mission.** Simulator missions are QGC WPL 110 files `sim_worlds/missions/<world>_survey.waypoints`
made by `ros2 run sim_worlds make_mission.py <world> [--altitude 35 --sidelap --frontlap --speed --area
--out FILE]` from the `survey:` block of `config/<world>.yaml`. Keep the team shape (3.4: TAKEOFF at 1,
DO_DIGICAM_CONTROL after the first lane waypoint, R-2 and R-1 NAV_WAYPOINTs, RTL last, NAV items frame 3,
DO items with x/y 0). Check: `ros2 run sim_bringup mission_tool check-mission --mission FILE`; fly:
`demo.sh runway --mission FILE`. Real missions: same shape in Mission Planner, with a lead-in before the
search area for the camera set-up (~15 s in the sim, ~30 s on an A8 mini).

**Change or add a world.** Never hand-edit `worlds/*.sdf`, `config/`, `truth/`: edit
`sim_worlds/scripts/build_runway.py` / `build_suas_field.py` and re-run them (`--seed`, `--assets fuel|primitive`).
A new world needs `worlds/<w>.sdf`, `config/<w>.yaml` (home == `<spherical_coordinates>`), a mission and a truth
file; check `ros2 run sim_worlds check_world.py <w>`; run `demo.sh <w>`. Quick try: `start_sim.sh /path/world.sdf`
(its spherical coordinates become SITL's home) + `--mission FILE`.

**Add a simulator feature.** Code in a package under `ros2_ws/src/simulation/` (ament_python, maintainer
astra-sim@example.com); topics under `/sim/`; run-time files under `sim_paths.sim_home()`; paths/ports from
`sim_bringup.sim_paths`; start it from `sim.launch.py` behind a launch argument whose default keeps today's
behaviour (or as a profile `extras` entry); shell entry points in `simulation/scripts/` sourcing `_common.sh`
(sessions, `say/info/warn/die`, camera address); tests; document it in the package README and
`simulation/README.md`. Never change a team package for a simulator feature.

**Change a team node safely.** Keep node names (stack_check, check_slot, the panel use them), topic/service
names, types and QoS, default parameter values, the photo folder and file name format, and the STATUSTEXT
texts the simulator parses: `Mission: N DigiCamCtrl`, `Camera trigger STARTED`, `Captured <res> image #N`,
`Zoom set to Xx`, `Detected <class>`, `Reached last WP`/`Holding to finish`, `Targets: tent@N -> person@M.
Diverting.`, `Resumed AUTO -> WP N`, `Over <class>`, `Servo C -> PWM`, `RTL` (`sim_bringup/phase_monitor.py`,
`sim_demo/phases.py`). Then: unit tests, `03_build.sh`, `demo.sh runway --headless`, compare `score_report.md`
with an earlier run. Think about the real drone: serial MAVROS link, 1 Hz telemetry, the A8 mini ignores SDK
0x04/0x06/0x16/0x18/0x19, the real photo orientation (`image_top_direction_deg`), sloped ground.

**Add a test.** `test/test_<thing>.py` in the package (ament_python packages: picked up by `colcon test`;
`main` is ament_cmake: also add `ament_add_pytest_test(<name> test/<file>.py)` to its `CMakeLists.txt`).
Test the pure module, not the node; fakes over real processes.

**Run one package's tests.** `cd ros2_ws/src/<pkg> && python3 -m pytest -q -p no:cacheprovider test`
(sim packages: `ros2_ws/src/simulation/<pkg>`), or the colcon line in 5.4.

## 8. Gotchas

| Trap | What to do |
|---|---|
| WSL2 draws 3D on the CPU (slow, RTF < 0.5) | `GALLIUM_DRIVER=d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` (set by `sim_env.sh` and `/etc/profile.d/astra_wsl_gpu.sh`); `glxinfo -B` must say D3D12 (NVIDIA). `QT_QPA_PLATFORM=xcb` in WSL. Windows need WSLg (Windows 11) |
| video_cam: `Camera did not answer the 0x0A ...` / emulator on 127.0.0.1 | the camera address is missing: `sudo bash simulation/setup/camera_ip_alias.sh` (adds `192.168.144.25/32` on `lo` + `astra-camera-ip.service`; all WSL distros share it) |
| emulator cannot open port 82 | `net.ipv4.ip_unprivileged_port_start=80` (`/etc/sysctl.d/60-astra-sim.conf`, set by setup) |
| Docker `sim` container and the native alias both claim .25 | `compose_up.sh` removes the native alias; `compose_down.sh --restore-camera-ip` puts it back |
| repo path with a space (the Windows checkout on the team laptop has one) / slow `/mnt` | quote every path (`bash "<checkout in WSL>/simulation/setup/03_build.sh"`); build and run only in `~/ODLC_Machine_Inferencing_System_2025-2026`; edit on Windows, re-run 03_build.sh |
| MAVROS packages | come from the pinned ROS snapshot 2026-08-07 (2.14.0; the Jetson has 2.12.0, same msgs): keep `/etc/apt/preferences.d/ros-snapshot-mavros`, do not replace MAVROS |
| numpy | must stay **< 2** (1.26.4): cv_bridge and the Jetson torch wheel break (`_ARRAY_API not found`); never `pip install -U` the ML stack |
| `Arm: Gyros inconsistent` for 30-60 s | normal SITL settling; wait (scripts do) |
| MAVROS keeps an old mission after an upload | MAVROS re-reads only when a ground station DOWNLOADS the mission, never after an upload; waypoint_manager now re-pulls on `Flight plan received` / arming, the scripts call the pull too; by hand `ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull {}`; symptom `buffer_wp = -2`, photos never stop |
| old photos in `mapping_photos` | new_od re-processes them and main_controller may pick an old target: sim scripts archive them; on the drone empty the folder before each flight |
| batched detector path with `.pt` weights fails | sim uses `use_batched_inference:=false`; the batched path needs the Jetson TensorRT engine |
| 5x zoom shrinks photo footprints 5x | sim default `--zoom survey` (max zoom 1); plan real lanes for the zoom you fly |
| SITL parameters changed at run time (e.g. from Mission Planner) persist in `eeprom.bin` and beat the `.parm` defaults | start with `sitl_wipe:=true` once |
| Mission Planner over UDP 14550 changes stream rates for the shared link | for comparable scored runs connect by TCP 127.0.0.1:5762; one GCS per port; firewall rule for UDP 14550 inbound; Windows address changes after reboot (looked up at every start) |
| ports busy (`udp 14550/14552 is in use`, SITL ports in use) | another simulator / MAVROS / GCS runs: `stop_sim.sh [--all]` or other ports (`sitl_instance:=1`, `--gcs-port`) |
| one heavy job at a time on a laptop | one simulator, Docker build or full colcon build at a time; check `free -m` (>= 4000 MB available before a sim). Agents sharing the team laptop serialize heavy jobs with `flock /mnt/wsl/astra_heavy.lock` (shared by all WSL distros) and simulator runs also with `/tmp/astra_sim.lock` |
| driving WSL from Windows (agents) | put Linux commands in a script file and run `wsl -d Ubuntu-22.04 -- bash /mnt/<drive>/<path>/script.sh` (PowerShell mangles inline quoting; Git Bash rewrites `/tmp` paths given to wsl.exe). Background processes started inside a short `wsl` call die when it returns: host long jobs with `Start-Process wsl.exe -WindowStyle Hidden -ArgumentList '-d','Ubuntu-22.04','--','bash','<script>'` and poll its log |
| `set -u` scripts + ROS setup files | `AMENT_TRACE_SETUP_FILES: unbound variable`: `set +u` while sourcing (sim_env.sh does) |
| ROS 2 parameter types follow the default value | `:=3` normally fails for a `3.0` default. Controller aiming numbers, trigger periods and SIYI `image_top_direction_deg` now accept either type and fall back on nonnumeric input; ROS regression tests cover startup/type handling |
| colcon `Failed to find ... package.sh` | build with dependencies: `03_build.sh --packages "x"` (`--packages-up-to`) |
| closing the Gazebo window | does not stop the simulation (separate `gz sim -g`); use `stop_sim.sh` |
| `ROS_DOMAIN_ID` | simulator and team stack must match (default 0, like the drone); `sim_env.sh` never changes it |
| STATUSTEXT longer than 50 characters | cut by MAVLink (the hold message is) |
| harmless noise | MAVProxy `time moved backwards`, `Failed to download /SRTM3/...`; `libEGL ... renderD128`; `gz_frame_id ... not defined`; `131072x1 screen size is bogus` |

## 9. Status and known issues (2026-10-24)

**Finished and verified**
- Task 0/1 review: 19 packages built, 932 functional tests passed, Ruff and shell syntax clean.
  New run `20261024-100602_runway`: real YOLO, 2/2 targets, 94/94 photos, no dropped triggers,
  tent/hiker drop errors 0.32/0.54 m, 288.3 s, controller healthy, landed and all processes stopped.
- Full search demo with the team's real code and real YOLO on the runway world, re-verified 2026-10-22:
  2 of 2 targets (hiker 0.95, tent 0.85), 97 photos taken and all checked, drops 0.8 m (tent) and 1.3 m
  (hiker) from the targets (Gazebo landing points), RTL, landed, 4:59 of flight.
- Four team-code fixes (commit 81a7950, unit-tested, measured in 3+3 headless runs): main_controller crash on
  a timed-out parameter read (crashes 1/3 runs -> 0/3); stale mission after an upload (`mission_sync.py`);
  drop error 14 m and 24 m -> 0.05-1.3 m (`target_geolocation.py` + sidecars); dropped photo triggers
  (31-36 photos with ~50 triggers lost -> 92-131 photos, none lost). **Not yet flown on the real drone**; an independent review for the real drone (slow serial
  MAVROS link, real A8 mini firmware, photo orientation, sloped ground) is still to be finished.
- Simulator packages sim_models, sim_worlds (runway, suas_field), siyi_emulator, sim_bringup, sim_demo:
  unit tests green (5.4); Mission Planner on Windows over UDP 14550 (WSL NAT; mirrored networking untested).

**Work in progress (next acceptance tasks)**
- Plug-and-play: 219 tests pass, example detector/listener 10/3; generated PLUGINS reference is checked.
  Custom detectors, alternate SITL and external autopilot still need recorded simulator runs (Task 4).
- Docker (`docker/compose*.yml`, `simulation/docker/`, `.devcontainer/sim`): compose files parse, images never
  built or run; DOCKER.md documents another-PC setup and the remaining Task 6 acceptance checks.
- `suas_field_real`: generator (`sim_worlds/realfield/`, `scripts/build_real_field.py`) + site pack
  (`sim_worlds/data/suas_field_real/`) only; no world/config/mission/truth. WORLDS.md lists missing stages,
  tree-window/pole-height mismatches and data restrictions; 18 new synthetic tests cover preparation fixes.
- START OVER (`run_reset.py`, `start_over.sh`) + Mission Planner mode (`demo.sh --manual`, `simulation/windows/`):
  unit-tested, never flown; panel START OVER, lock/ownership/landing safeguards and handover regression tests added.
- Human guides now cover setup/editing, architecture, models/profiles, SITL, worlds, Docker, Mission Planner,
  Google Earth, troubleshooting and changes. Clean-machine tutorial and closed-book quiz remain Tasks 7/8.

**Open problems**
- Scoring now records corrected aim separately from legacy photo-GPS location error; old JSON remains compatible.
  `demo.sh` still retries the team stack once if a node dies, so inspect logs for hidden restarts.
- Task 2 independent review identified pending fixes: mission-pull futures lack a timeout; an inaccessible photo
  folder can defeat the processing deadline; geolocation needs nonfinite GPS/pixel rejection. No fixes claimed yet.
- Aiming assumes one ground plane above home and near-zero stabilized roll; real sloped terrain, camera mounting,
  image orientation and FOV need calibration. The 5 s parameter read is local ROS, not a serial mission upload.
- `launcher.sh` heartbeat wait accepts any `/mavros/state` message and has no per-attempt timeout.
- Open risks from last year's code: the two `/addWaypoint` full-mission pushes are not awaited (a rejected
  push could shift target indices); one high-confidence false positive decides a drop; tents are found only as
  COCO `umbrella`/`kite` (no fine-tuned model); telemetry at 1 Hz (GPS up to ~1 s old in photo names).
- Unknown on the real airframe: servo outputs 11/13 and 1400/500 us, the camera model that flies, the gimbal
  and photo orientation (`image_top_direction_deg`).
- Team lint tests (ament flake8/pep257) fail (pre-existing); CI `colcon` jobs cannot fail; `validation.yml` is a placeholder.

## 10. Where to read more

| Topic | File |
|---|---|
| Simulator: run, what you see, Mission Planner, ports, scripts, troubleshooting | `simulation/README.md` |
| Architecture, plug-ins/models, SITL, Docker, terrain, operator flow and troubleshooting | `docs/simulation/` (index in `simulation/README.md`) |
| Profiles | `simulation/profiles/README.md`, `default.yaml` (every key explained) |
| Mission brain, drop aiming, parameters | `ros2_ws/src/main/README.md` |
| Camera driver, SDK, sidecar, orientation | `ros2_ws/src/video_cam/HOW_OUR_CAMERA_WORKS.md`, `video_cam/video_cam/config.py`, `photo_metadata.py`, vendor PDFs in `video_cam/siyi_documentations/` |
| Photo trigger, mission sync | `ros2_ws/src/mapping/README.md`, `ros2_ws/src/waypoint_mavros/README.md` |
| Detector | `ros2_ws/src/detection/detection/README.md` |
| Simulator packages | `README.md` in `sim_bringup`, `sim_models`, `sim_worlds`, `siyi_emulator`, `sim_demo`, `sim_plugins`, `sim_plugin_examples` (+ each example) |
| Dev container, Docker targets | `README.md` (root), `docker/Dockerfile` header, `docker/compose.sim.yml` header |
| Every script's options | `bash <script> --help` (prints its header), or read the header of the scripts without `--help` (`01_install_humble.sh`, `02_setup_ardupilot.sh`, `camera_ip_alias.sh`, `sim_env.sh`) |
