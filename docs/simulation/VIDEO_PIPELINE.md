<!--
docs/simulation/VIDEO_PIPELINE.md -- PURPOSE: the design for running the team's NEW camera system (the "video
pipeline": zr10_ros2 camera control, the zr10_stream RTSP stream -> /zr10/image_raw, and the yolo26 detector) in the
Gazebo digital twin exactly as in the field, unchanged, from a folder the user points to. It says what the field
pipeline is, the three configurations of the twin, every connection (what, from, to, transport, port, rate), how the
simulated ZR10 camera serves its RTSP video at rtsp://192.168.144.25:8554/main.264 and answers the SIYI SDK, how the
team's nodes and detector start on this PC (natively and in the "virtual Jetson" container) and on the real Jetson,
where the team code comes from and how to update it, what the mission logic consumes (nothing yet: the twin runs a
clearly labelled PROPOSAL so demos complete), what every run logs, every remaining difference from the field, the
build plan for two engineers (EMULATOR side and PIPELINE side) with the contract between them, the exact changes the
Jetson design needs, the field-safety findings for the team (proposals P1-P10), what the live code map must add, and
the decisions still open. Status: DESIGN, revision 2 (2026-10-31, after the review); nothing in it is built.
Facts are tagged VERIFIED (a command was run, section 17), SOURCE (read in code or a manual) or EXPECTED (not tested).
-->

# The team's new video pipeline in the twin: design

**Status: DESIGN, revision 2 (2026-10-31, after the review). Nothing here is built yet.** It includes the owner's
instructions up to 15:14 on 2026-10-31. Two read-only studies came first: one read the team's new code and the
board's probe, the other read the simulator's camera side and measured RTSP servers on this PC. Their reports are
kept with the workflow's notes, not in the repo. A reviewer then checked revision 1 against the code; the table
below says what changed. Section 17 lists the evidence.

### What changed in revision 2 (the review)

| Review finding | What the design says now | Section |
|---|---|---|
| Refuted: "the board's launcher.sh runs the field launch file through its documented `TRACKER_CMD`" | The board's `launcher.sh` (commit `6beb5f1`, line 73) **hard-codes** `TRACKER_CMD` and ignores the environment. Only this repo's copy (line 78, added by commit `7298dc7` on 2026-10-22) honours it. The team needs a one-line change to adopt the launch file (team proposal **P7**). The twin, the virtual Jetson and HIL run this repo's copy: that is a difference from the field. A `6beb5f1` team checkout is refused for the video configuration, with a clear error | 2.1, 8.1, 8.2, 9, 12 (item 9), 14, 15.3 |
| Refuted: "without the photo folder the hold waits 90 s" | With the board's `main_controller` (`6beb5f1`) a missing `mapping_photos` folder makes the hold **never end**: the drone hovers until a failsafe. The proposed launch file now creates the folder (it never deletes anything), and **P8** asks the team to adopt this repo's hold fix | 2.2, 10.2, 12 (item 24), 15.3 |
| Refuted: "the adapter calls the team's own `target_geolocation.aim_at_box`, the same projection `main_controller` uses" | `target_geolocation.py` exists only in this repo (commit `81a7950`), not at `6beb5f1`. The proposal package now carries its own labelled copy, so it needs nothing from the team's `main` package. Neither `main_controller` projects a video detection | 10.3 |
| Contradictory detector user rule | ONE rule, in one table (contract C11): proposals on = the patched image, `--user` (P4), reliable image QoS (P9), GPU; proposals off and `board` = the team's image as root, like the board. `--user` counts as a proposal. The same rule holds natively, in the virtual Jetson and on the board | 3, 8.3, 11.2, 12, 14, 15.1 |
| "`--user` so the pictures arrive" was untested for 2.7 MB best-effort images | **Tested on this PC** (VERIFIED, same user): with the team's QoS only 24 and 11 of 40 pictures arrived; with a reliable subscriber or a bigger shared-memory segment, 40 of 40. New team proposal **P9**; `board` and `--no-proposals` reproduce the loss | 11.2, 12 (item 20), 15.3, 17 |
| Refuted: detector "not in git" | The detector is a clone of `github.com/llaammbbddaa/objectDetection`; the running build is the LOCAL branch `codex/ros2-image-subscriber` (`b02cac6`, not pushed) plus uncommitted changes. The topic mode the board uses exists only in those changes. `zr10_stream` is the only part with no git | 2.1, 9, 15.1 (D4) |
| Wrong CRC figures | 1-byte commands 1.55 % (first bad sequence 38); gimbal speed `0x07`, the STOP included, **1.94 %** (first bad 43); centre `0x08` first bad 5 (VERIFIED three times) | 7, 15.3, 17 |
| Refuted: "ffmpeg's 10 s timeout" | The 10 s is `stream_node`'s own frame deadline (`frame_source.py` line 25); ffmpeg has no timeout. An unpowered camera gives an EXPECTED ~4 s retry cycle | 6.7, 12 (item 17), 16 |
| Refuted: frame matching used `total` | Arrival time = log time (or CSV time) minus `latency` (= queue wait + total). The "0.4 m" bound is gone; matching aligns the whole frame sequence and records its ambiguity per row | 11.3, 11.4 |
| Overclaim: "the delay does not move the aim point" | A fixed delay removes the average only; `stream_node`'s 0.5 s timer leaves about ±0.25 s (±1.4 m at 5.5 m/s). New team proposal **P10** (stamp each picture when ffmpeg delivered it) | 6.5, 10.3, 12 (item 30) |
| `board` configuration incomplete | `team_stack.launch.py` adds `model_path` (the `.pt`) and `use_batched_inference` false: `new_od` runs the SAHI path, not the board's TensorRT engine. Root-owned result files are given back to the WSL user after the run | 8.1, 12 (item 15) |
| `pov_server` and the Gazebo camera panel read the 1080p30 camera too | `pov_server` gets a 640x360 preview from the emulator; A5 measures the real-time factor with the Gazebo camera panel and `pov_server` running | 5 (rows 20-21), 6.4, 6.8, 13.4 |
| `CameraZoomPlugin` clamps at 0.2 rad | The xacro must set `<max_zoom>` 20 (at least 16.9); the emulator sends the 1x command at start | 6.3, 13.2 (C5), 13.4 (A3) |
| The manual's main-stream examples use H.265 | The twin default stays H.264 (owner 12:25), but every stream check runs with H.265 too; D3 asks the owner again | 6.2, 13.4, 15.1 |
| Build, launch and stop details | exact colcon package paths + `--log-base`; absolute `TRACKER_CMD` paths without spaces or `~`; no `-t` on the detector, stopped with `docker kill --signal=INT` by a small wrapper; leftover containers removed | 8, 9 |
| Plan out of date (owner 15:14) | The acceptance test is now a full visible flight with the VIRTUAL Jetson (J1, first); real-board steps are documented only; who applies the patches to other streams' files, and a manual path until then | 13.3, 13.4, 13.6 |
| A1 ran 60 s = the RTSP session timeout | A1 runs 5 minutes; A1b checks reconnects; the stream keeps the VERIFIED time-stamping method | 6.4, 13.4 |
| `mapping_photos` "present and empty" wrote into the team checkout | It is the runtime folder of the team copy the simulator runs, which `demo.sh` already archives before every run; `video.root` (the team's video code) is never written | 10.2 |
| (user request) the live code map | Section 18 lists the new folders, programs and links the code-map refresh must add | 18 |

Tags used below:

- **VERIFIED**: someone ran a command on this PC and saw the result (section 17 says which).
- **SOURCE**: read in the team's code, in our code, or in the SIYI ZR10 manual v1.7 (the team keeps the PDF in
  `ros2_ws/src/video_cam/siyi_documentations/`).
- **PROBE**: seen on the real Jetson during the owner-approved read-only probe of 2026-10-31 11:47.
- **EXPECTED**: what we think will happen. It is not tested yet.

Related designs: [Jetson in the loop](../jetson/HIL_DESIGN.md) (it must stay consistent with this page; section 14
lists the changes it needs), [logs and data of a run](LOGS_AND_DATA.md), [plug-and-play profiles](PLUGINS.md),
[swapping the team code](SWAPPING.md), the [camera emulator](../../ros2_ws/src/simulation/siyi_emulator/README.md).

## 0. In one minute

- **The field camera system is the team's new video pipeline** (owner, 11:56 to 12:32). It has three parts. First,
  `zr10_node` steers the SIYI ZR10 camera with SDK commands. Second, `stream_node` pulls the camera's live video with
  ffmpeg and publishes it as pictures on the ROS topic `/zr10/image_raw` (1280x720, 2 per second). Third, the
  `yolo26` detector subscribes to that topic and finds objects. Last year's photo path is now the
  "legacy (2025) photo pipeline". It stays available as an option, but it is off by default.
- **The twin runs the team's code unchanged.** The simulated camera at `192.168.144.25` gets a real RTSP video server
  on TCP 8554, path `/main.264`. Its pictures come from the Gazebo gimbal camera, with the ZR10 lens and the ZR10
  zoom. The team's own `stream_node` and its own ffmpeg command pull that stream, exactly as on the drone.
- **The server is GStreamer's gst-rtsp-server, inside the camera emulator.** It is already installed on this PC. It
  was tested with the team's exact ffmpeg command on the exact URL: 2.1 pictures per second, no bad pictures
  (VERIFIED). It encodes H.264 at 1920x1080, 30 frames per second, 2000 kbit/s. Those values come from the ZR10
  manual's documented choices. The real camera's exact defaults were not measured, and the owner said not to
  measure them (12:25). This is listed as a difference. The manual's own main-stream examples use H.265, so every
  stream check runs with H.265 as well (6.2).
- **Nothing in the team's code passes the new detections to the mission logic yet** (SOURCE). `main_controller`
  only listens to `/image_detection`. Today's yolo26 detector publishes no topic at all: it writes CSV and PNG files.
  Nothing points the camera at the ground either. So the twin offers both:
  - **`--no-proposals`** shows today's truth: the drone flies the survey, finds no targets, and returns home (RTL)
    without dropping anything.
  - **the default** runs our **PROPOSAL** (owner, 12:25), clearly labelled "proposal, not adopted by the team". A
    small node points the gimbal down when the survey starts. The detector also publishes its boxes on a topic. An
    adapter turns each box into the message `main_controller` already understands, with a ground position worked
    out from the drone's position and the gimbal angles at the moment the picture was taken. So demos complete with
    drops. The detector then also runs as the normal user and subscribes reliably (team proposals P4 and P9); with
    `--no-proposals` it runs as root with the team's QoS, exactly like the board (section 3 has the one table).
- **Field-safety findings for the team** (each written as an exact proposal, 15.3): the board's `launcher.sh` ignores
  `TRACKER_CMD`, so the new pipeline cannot start at boot without a one-line change (P7); without the photo folder the
  board's `main_controller` never ends its hold and the drone hovers until a failsafe (P8); with the team's QoS many
  2.7 MB pictures never reach the detector, even for the same user (VERIFIED on this PC: only 24 and 11 of 40
  arrived; P9), and a root container gets none at all (P4); the gimbal STOP command has a bad checksum 1.94 % of the time (P1).
- **The acceptance test (owner 15:14) is a full visible flight with the VIRTUAL Jetson** running the video
  configuration on this PC. The real-board steps are written down for later and not run.
- **Detection runs on the GPU with TensorRT** (owner, 12:25). On the Jetson the TensorRT "engine" (the model compiled
  for one GPU) is built on the board, once, with the owner's OK. On this PC the detector uses TensorRT inside Docker
  with the RTX 4060. If Docker cannot reach the GPU, it uses ONNX Runtime with CUDA, and the CPU only as the last
  choice. Every run records which one ran. The team's current CPU-only detector image is kept as an option, because
  it is what the board runs today.
- **The team code comes from a folder you point to** (`video.root`, laid out like the Jetson's home folder). It is
  never copied into this repo, and the twin never writes into it.
- **Every run logs the pipeline:** every picture the camera sent (with the drone pose, gimbal angles and zoom), the
  stream health (frame rate, gaps, restarts, latency), every detector frame and every box. These become CSV files
  and a "Video pipeline" block in the run summary.
- **Two engineers can build it in parallel.** The EMULATOR side builds the RTSP stream and the ZR10 behaviour. The
  PIPELINE side builds the team-code runner, the proposals, the GPU detector, the logging and the docs. Section 13
  has the contract between them.

## 1. Words used here

| Word | Meaning |
|---|---|
| RTSP | "Real Time Streaming Protocol". A program asks a camera for its live video at an address like `rtsp://192.168.144.25:8554/main.264` |
| RTSP over TCP ("interleaved") | the video travels inside the same TCP connection as the RTSP requests. The team's ffmpeg asks for this (`-rtsp_transport tcp`), so only TCP port 8554 is needed |
| H.264 / H.265 | two common ways to compress video. The ZR10 can send either one (SOURCE: manual p.50) |
| frame | one picture of the video. "30 fps" = 30 frames per second |
| GOP / keyframe | a keyframe is a complete picture. The frames between keyframes only store changes. GOP = the number of frames from one keyframe to the next. A new viewer must wait for a keyframe |
| encoder | the part that compresses frames: `x264` (software, on the CPU) or `NVENC` (the NVIDIA card's video chip) |
| gst-rtsp-server | the RTSP server library of GStreamer (a media toolkit). It is installed on this PC (VERIFIED) |
| ffmpeg | a video tool. The team's `stream_node` runs it to pull and decode the stream |
| `/zr10/image_raw` | the ROS topic where `stream_node` publishes each decoded picture (`sensor_msgs/Image`, rgb8, 1280x720) |
| DDS, Fast DDS | the system that carries ROS 2 messages between programs. On one computer it may use "shared memory" (a memory area both programs open) |
| QoS "best effort" | a message may be lost and is not sent again. The detector subscribes this way (`SensorDataQoS`). A 2.7 MB picture travels in about 43 pieces; if one piece is lost, the whole picture is lost |
| QoS "reliable" | lost pieces are sent again until they arrive. Team proposal P9 makes the detector subscribe this way |
| shared-memory segment | the memory area Fast DDS uses to pass messages between programs on one computer; 0.5 MB by default, smaller than one picture |
| P1-P10 | our proposals for the team's own files (15.3). Applied nowhere; the team decides |
| ONNX | a file format for a trained model (`yolo26.onnx`) |
| TensorRT, engine | NVIDIA's fast inference library. An engine is the model compiled for ONE GPU and ONE TensorRT version, so each machine builds its own engine from the same ONNX file |
| backend | what runs the model: `tensorrt`, `ort_cuda` (ONNX Runtime on CUDA) or `opencv_cpu` (the team's current code) |
| proposal | code WE wrote for the team to look at (`simulation/video/proposals/`). The team has not adopted it. The twin runs it so demos complete, and labels it everywhere |
| `video.root` | a folder on this PC laid out like the Jetson's home folder, holding the team's video code (section 9) |
| virtual Jetson | the Jetson's container running on this PC (see the [Jetson design](../jetson/HIL_DESIGN.md)) |
| HIL | "hardware in the loop": the real Jetson runs the team's software against the simulator on the PC |
| the board | the team's real Jetson Orin Nano, as seen in the read-only probe of 2026-10-31 11:47 |

## 2. The field pipeline: what the team runs and what the owner decided

### 2.1 What runs on the board (PROBE, 11:47; started by hand, all on ROS_DOMAIN_ID 0)

| Piece | Exact command on the board | Built from |
|---|---|---|
| camera control `zr10_node` (C++, node name `zr10_camera`) | `ros2 run zr10_ros2 zr10_node --ros-args -p dry_run:=false` (no parameter file: all other settings are the code defaults) | `~/astra_ws/src/zr10_ros2` (the copy whose `CMakeLists.txt` line 17 is `target_link_libraries(zr10_node CURL::libcurl)`) |
| stream `stream_node` (Python + ffmpeg, node name `zr10_stream`) | `ros2 launch zr10_stream stream.launch.py fps:=2.0`, which runs `ffmpeg -hide_banner -loglevel error -nostdin -rtsp_transport tcp -i rtsp://192.168.144.25:8554/main.264 -an -vf scale=1280:720,fps=2.0 -pix_fmt rgb24 -f rawvideo pipe:1` | `~/astra_camera_detection/astra_ws/src/zr10_stream` |
| detector (C++ `yolo26_ros detector_node`, in Docker) | `docker run --rm -it --network host --ipc host -e ROS_DOMAIN_ID=0 -v /home/astra-dev/astra_camera_detection/astra_ws_d/objectDetection:/data yolo26-ros-subscriber ros2 run yolo26_ros detector_node --ros-args -p input_mode:=topic -p image_topic:=/zr10/image_raw -p model_path:=/opt/model/yolo26.onnx -p output_dir:=camera_results` (runs as root, no GPU flags) | image `yolo26-ros-subscriber`, built on the board at 11:33 from `~/astra_camera_detection/astra_ws_d/objectDetection/Dockerfile` |
| also running | the boot-time `launcher.sh` + `tracker.launch.xml` (MAVROS, waypoint_manager, gps node, and the now-obsolete photo nodes `siyi`, `trigger`, `new_od`, plus main_controller), and a diagnostic `ros2 topic hz /zr10/image_raw` in a second container | the team repo at commit `6beb5f1` |

Where each part comes from (VERIFIED read-only on the 11:51 snapshot with `git --no-optional-locks`; PROBE for the
board's own folders):

| Part | In git? | What the board ran |
|---|---|---|
| `zr10_ros2` | yes: the team's ASTRA repo, branch `video-cam` (`f496beb`). The board's copy `~/astra_ws/src/zr10_ros2` has no `.git`. Its `.cpp` files are identical to `video-cam`; only `CMakeLists.txt` line 17 differs. `video-cam` has `PRIVATE CURL::libcurl Threads::Threads`, which the team's own integration note says fails on ROS 2 Humble (SOURCE). A second copy under `~/astra_camera_detection/astra_ws/src/` has that same PRIVATE line | the build in `/home/astra-dev/astra_ws/install` (PROBE), i.e. from the copy with the plain line 17 |
| `zr10_stream` | **no git anywhere we know** | `~/astra_camera_detection/astra_ws/install` |
| detector (`yolo26_ros`) | yes: a clone of `https://github.com/llaammbbddaa/objectDetection`. It sits on the LOCAL branch `codex/ros2-image-subscriber` at `b02cac6` "init" (a local commit of 2026-10-31 10:34, not on `origin`), PLUS uncommitted changes: modified `CMakeLists.txt`, `Dockerfile`, `package.xml`, `readme.md`, `launch/feed_test.launch.py`, `src/detector_node.cpp`, `src/video_feeder_node.cpp`, `tests/VALIDATION.md`, and two new files `include/yolo26_ros/image_message.hpp`, `tests/topic_integration.py`. (git also lists `entrypoint.sh`, a file-mode change only, and `README.md`, which clashes with `readme.md` on Windows; both are probably artefacts of copying to Windows.) **The topic mode the board uses (`input_mode`, `image_topic`) exists only in these uncommitted changes:** the committed `b02cac6` always subscribes `/camera/image_raw` | the image built at 11:33 from that uncommitted tree |
| `launcher.sh` + the mission stack | yes: `~/astra` is the team repo at `6beb5f1` (PROBE). The probe ran only `git log`, so local changes on the board are unknown. **At `6beb5f1`, `launcher.sh` line 73 hard-codes `TRACKER_CMD="ros2 launch main tracker.launch.xml"` and ignores the environment.** The override `TRACKER_CMD="${TRACKER_CMD:-...}"` exists only in THIS repo's `launcher.sh` (line 78, added by commit `7298dc7`, 2026-10-22, the digital-twin commit). Apart from that line and a comment, the two files are identical | `launcher.sh` -> MAVROS + `tracker.launch.xml` since boot |

### 2.2 What the team code does and does not do (SOURCE, both studies)

| Question | Answer |
|---|---|
| What does `zr10_node` send to the camera during a mission? | **Nothing**, until it shuts down. It only sends when a person calls one of its services (`/take_photo`, `/autofocus`, `/toggle_recording`, `/stop_zoom`, `/stop_gimbal`, `/center_gimbal`) or publishes on `/zoom_direction` (Int8) or `/gimbal_speed` (Vector3). No team node does. It never reads the camera's replies |
| What replaces last year's photo trigger? | Nothing. The video streams all the time, and the detector processes every picture it receives |
| Who points the camera at the ground? | Nobody. `zr10_node` has no "set angles" command: it only has speed control and "centre" (forward) |
| What does the detector output? | **Files only**: `camera_results/csv/frameN.csv` (columns `x1,y1,x2,y2,confidence,class_id,class_name,resolutionX,resolutionY,detection_time`) and `camera_results/annotated_images/annotated_frameN.png`, plus one log line per frame. No topic. Stock COCO classes (person = 0; there is no tent or mannequin class). No time, no position |
| What does `main_controller` consume? | Only `/image_detection` (`interfaces/msg/ImageResult`, class_id `"0"` person / `"1"` tent, latitude/longitude). Only the obsolete `new_od` publishes it |
| So what happens with the new pipeline only? | EXPECTED from the code: the drone flies the survey and holds in GUIDED at the last waypoint. What happens next depends on the photo folder `<ws>/src/video_cam/mapping_photos`: **present and empty** -> `All 0 images processed. Selecting targets.`, then `No valid targets found. Returning to launch.`, home with no drops. **Old photos in it** -> nobody processes them, so after 90 s `Processing timeout (0/N). Diverting anyway.`, then the same. **Missing** -> with the board's `main_controller` (`6beb5f1`) **the hold never ends**: before 90 s it waits; after 90 s it falls through to `os.listdir()` (line 316), which raises `FileNotFoundError` inside the `try`; the `except` (line 342) only logs `Error checking processing status` and this repeats every 2 s, so the drone hovers until a failsafe or the pilot takes over. (This repo's `main_controller` ends after 90 s: `processing_hold.py`, commit `94ccf0b`.) On the board today the folder exists because the boot launcher's `siyi` node (its storage manager) and `new_od` create it; the folder is git-ignored (`.gitignore` line 45). A launch file without those two nodes must create it (10.2, P8) |
| Does the detector get every picture? | No. VERIFIED on this PC with the team's QoS: even between two programs of the same user, only 24 and 11 of 40 pictures arrived (11.2). Between a root container and a normal user, nothing arrives at all (VERIFIED by the Jetson design) |

### 2.3 The owner's decisions this design follows

| When | Decision | What it means here |
|---|---|---|
| 11:16 | run exactly as in the field; list every remaining difference | section 12 |
| 11:56, 12:04 | the field pipeline is the new video pipeline; the photo path is obsolete, kept as a labelled legacy option, off by default | three configurations (section 3), default `video` |
| 12:00 | the camera controller is `zr10_ros2`; the emulator must answer everything it sends | section 7 |
| 12:10 | write a proposed field launch file (and, if needed, an adapter proposal) under `simulation/video/proposals/`; the twin runs it; "the team has not adopted it yet" is a difference | sections 8.2 and 10 |
| 12:25 | detection runs on the GPU with TensorRT (the current CPU container is work in progress); the camera publishes images on a topic and the detection package subscribes to it; run the proposal in the twin so demos complete, clearly labelled; the camera is the **SIYI ZR10**; **no measurement on the real camera**: use the manual's documented stream settings and list "defaults not measured"; pick the detector backend automatically per machine and record it | sections 6, 8.3, 10, 11, 12 |
| 12:32 | the PC container must be the Jetson's software version for version (one Dockerfile, two base layers; x86 base e.g. NGC TensorRT 24.08) | sections 8.3 and 14 |
| 12:40, 12:50 | the owner's demo has priority over heavy jobs while its flag file exists (now gone) | every build and flight in 13.3 checks the flag and holds both locks |
| 15:14 | no real Jetson for now: build and PROVE everything with the virtual Jetson; **a full visible simulator flight with the virtual Jetson is the acceptance test**; keep the real-board steps documented with exact commands and expected outputs, but do not run them | 13.3 (step 5, J1 first), 13.4, 14 |

## 3. The three configurations of the twin

| Configuration | What runs on the "Jetson" side | Who feeds `main_controller` | When to use it |
|---|---|---|---|
| **`video`** (the default once its acceptance tests pass) | the proposed field launch file: `zr10_node`, the `zr10_stream` launch, the detector, waypoint_manager, the gps node, main_controller. No photo nodes | with proposals ON (default): our labelled adapter. With `--no-proposals`: nobody (RTL, no drops) | the field system |
| **`board`** | exactly what the board ran at 11:47: the unchanged `tracker.launch.xml` (photo path) PLUS the new pipeline started like the team starts it by hand (CPU detector image) | `new_od`, from photos (as on the board today) | comparing with the board; "both side by side exactly as on the board" |
| **`legacy`** | today's simulator: `tracker.launch.xml` only (siyi, trigger, new_od ...) | `new_od` | the 2025 photo pipeline, for old comparisons |

Two more choices go with the `video` configuration:

- **`video.proposals: on|off`** (default `on`). On = every proposal of ours, all labelled "proposal": the
  gimbal-pointing node, the detection topic, the adapter, the GPU detector patches, the detector as the normal user
  (P4) and its reliable image subscription (P9). Off = the team's code as it is today, started the way the board
  starts it. **`--user` (P4) and the reliable QoS (P9) count as proposals: they are on exactly when proposals are on.**
  The proposed launch file itself, and its photo-folder step (10.2), run in both: without the launch file the video
  configuration cannot start at all, and without the folder step the board's `main_controller` could hover for ever.
- **`video.detector_backend: auto|tensorrt|ort_cuda|opencv_cpu`** (default `auto`). It needs proposals on (only the
  patched image has more than one backend). `opencv_cpu` with proposals on is the team's CPU code path inside the
  patched image.

**The detector settings per configuration (contract C11: the ONE rule; `video.sh`, the launch file, `jetson.sh` and
every doc follow this table):**

| Configuration | Detector image | User | Backend, GPU flags | Image QoS (how it subscribes `/zr10/image_raw`) | What reaches `main_controller` |
|---|---|---|---|---|---|
| `video`, proposals on (the default once step 5 of 13.3 passes) | `astra-video-detector:<flavor>` (team source + our labelled patch set) | `--user UID:GID` (P4) | `auto` (TensorRT where it can), `--gpus all` (PC) / `--runtime nvidia` (Jetson) | reliable, depth 5 (P9, patch parameter `image_qos:=reliable`) | the adapter's `/image_detection` |
| `video --no-proposals` | the team's `yolo26-ros-subscriber`, built unchanged from their folder | root (no `--user`), as on the board | `opencv_cpu`, no GPU flags | the team's `SensorDataQoS` (best effort) | nothing (RTL, no drops) |
| `board` | the team's `yolo26-ros-subscriber` | root, as on the board | `opencv_cpu`, no GPU flags | the team's `SensorDataQoS` | `new_od`, from photos |
| `legacy` | no detector container | - | - | - | `new_od` |

- **The same rule holds everywhere:** natively in WSL, in the virtual Jetson and on the real Jetson (HIL). In the
  containers "the normal user" is the stack's user (`ASTRA_UID`/`ASTRA_GID`).
- **EXPECTED consequence:** with proposals off (and in `board`) the detector probably receives NO pictures (root
  container + user publisher, 11.2), exactly as the board probably does today. The run summary says so in plain
  words; that is "today's truth", not a twin bug.
- **Overrides** (`--detector-user board|match`, `--detector-backend B`, `--detector-qos sensor_data|reliable`) change
  one item for a test. Each one is written to `video/commands.txt` and listed in `video/differences.txt` as a
  difference from the configuration's default. `--detector-backend` other than `opencv_cpu` with proposals off stops
  with: `[video] ERROR: --detector-backend tensorrt needs the patched detector, but proposals are off. Why: the team's
  image has only the OpenCV CPU path. Fix: drop --no-proposals, or use --detector-backend opencv_cpu.`

In the `board` and `legacy` configurations the old video_cam `siyi` node points the gimbal down and racks the zoom to
5x when the survey starts. Because the video comes from the same gimbal camera, the stream then shows the zoomed,
downward view too. That is what the board would do today.

## 4. The picture (this PC, configuration `video`, proposals on)

```
WSL2 Ubuntu-22.04 (one ROS graph, ROS_DOMAIN_ID 0, like the board)
 gz sim (Gazebo)                                         ArduCopter SITL <-- TCP 5763 (SERIAL2) -- MAVROS
   drone "iris": gimbal + ZR10 video camera 1920x1080 @ 30 fps, HFOV 61.5 deg at 1x (7 deg at 10x)
        | gz-transport /sim/camera/video/image (raw RGB, in-process)
        v
 siyi_emulator  (the simulated ZR10 at 192.168.144.25 on WSL's loopback alias)
   SDK  UDP 37260 <---------- zr10_node (on demand; nothing during a mission)
                  <---------- [proposal] gimbal_pointing: 0x0E "look down" at the survey start
                  <---------- [proposal] detection_adapter: 0x0D attitude + 0x18 zoom polls
   HTTP TCP 82    (photo API; zr10_node uses it only if sync_photos is on, which the board leaves off)
   RTSP TCP 8554  /main.264  H.264 1920x1080 30 fps 2000 kbit/s  (gst-rtsp-server + x264, new)
   ROS /sim/camera/preview 640x360 <= 10 Hz (new) ----------------> pov_server -> Mission Planner HUD
        |                          writes <run>/video/stream_frames.csv (pose, gimbal, zoom per frame)
        v  RTSP over TCP (team's own ffmpeg: decode, scale to 1280x720, keep 2 fps)
 stream_node (team, unchanged) --/zr10/image_raw (rgb8 1280x720, <= 2 Hz)--> detector (Docker)
                                                     [proposal] --user + reliable subscription (P4, P9)
                                                                               |  files: <run>/video/yolo26/camera_results/
                                                     [proposal] /yolo26/detections (vision_msgs)
                                                                               v
                         [proposal] detection_adapter: box + drone pose + gimbal angles -> ground lat/lon
                                                                               | /image_detection (interfaces/ImageResult)
                                                                               v
 main_controller (team, unchanged) -> hold, targets, drops, RTL      waypoint_manager, gps node (team, unchanged)
   (the launch file's prepare_photo_folder step makes sure mapping_photos exists, so the hold can end: 10.2)
 score_run, phase_monitor, flight_recorder (PC tools, unchanged inputs + the new video files)
 All of it is started by THIS repo's ./launcher.sh through TRACKER_CMD (the board's launcher ignores TRACKER_CMD: P7)
```

## 5. Every connection

"Rate" is what flows during a normal flight. "On demand" means only when someone asks.

| # | What | From | To | Transport | Address / port | Rate |
|---|---|---|---|---|---|---|
| 1 | live video (RTSP control + the H.264 video in the same connection) | team `stream_node`'s ffmpeg | emulator's RTSP server | TCP (RTSP, RTP interleaved) | `192.168.144.25:8554`, path `/main.264` | the camera's main stream, all the time: 30 fps, 2000 kbit/s (twin default) |
| 2 | rendered pictures | Gazebo video camera | emulator stream thread | gz-transport, same computer | topic `/sim/camera/video/image` (raw RGB, 6.2 MB per 1080p frame) | 30 Hz of simulated time |
| 3 | decoded pictures | ffmpeg | `stream_node` | pipe (stdout) | raw rgb24 1280x720 = 2,764,800 bytes per frame | 2 fps |
| 4 | pictures for the detector | `stream_node` | detector (+ the optional passive monitor, row 16) | ROS topic, DDS | `/zr10/image_raw`, `sensor_msgs/Image` rgb8; publisher RELIABLE/VOLATILE/depth 1; the team's detector subscribes BEST_EFFORT depth 5 (`SensorDataQoS`); with proposals RELIABLE depth 5 (P9). Best effort loses many of these 2.7 MB pictures even for the same user (VERIFIED, 11.2) | up to 2 Hz published; fewer received |
| 5 | detector results | detector | files | disk | `<run>/video/yolo26/camera_results/csv/frameN.csv` + `annotated_images/annotated_frameN.png` | one of each per processed frame |
| 6 | detector log | detector | log file | stdout | one line per frame: `[frameN.png] detections=K detect=..ms queue_wait=..ms total=..ms latency=..ms queue_depth=..`, a summary at shutdown | per frame |
| 7 | [proposal] detection topic | patched detector | adapter (+ score tools) | ROS topic | `/yolo26/detections`, `vision_msgs/Detection2DArray`, header copied from the input picture, RELIABLE depth 10 | one per processed frame (also empty ones) |
| 8 | [proposal] detections for the mission | adapter | `main_controller` (+ `score_run`, `phase_monitor`) | ROS topic | `/image_detection`, `interfaces/msg/ImageResult`, RELIABLE depth 10 | only frames with a person or tent stand-in box |
| 9 | camera commands | `zr10_node` | emulator SDK | UDP | `192.168.144.25:37260` from an ephemeral port | on demand; a stop frame 0.5 s after the last motion command; two stop frames at shutdown |
| 10 | camera replies | emulator | the sender's ephemeral port | UDP | | one reply per request that has one (zr10_node never reads them) |
| 11 | [proposal] look down | `gimbal_pointing` | emulator SDK | UDP | `192.168.144.25:37260` | 0x0E once at `Mission: N DigiCamCtrl`, checked with 0x0D, at most 3 tries |
| 12 | [proposal] gimbal angles and zoom | `detection_adapter` | emulator SDK | UDP | `192.168.144.25:37260` | 0x0D at 10 Hz, 0x18 at 2 Hz |
| 13 | photo API | `zr10_node` (only when `sync_photos` is true) and the legacy `siyi` node | emulator HTTP | TCP (HTTP) | `192.168.144.25:82`, `/cgi-bin/media.cgi/api/v1/...` | every 2 s when used (off on the board) |
| 14 | drone pose for the adapter | MAVROS | adapter | ROS topics | `/mavros/global_position/global`, `/mavros/global_position/rel_alt`, `/mavros/global_position/compass_hdg` | 1 Hz (the team's gps node sets every stream to 1 Hz) |
| 15 | emulator -> Gazebo | emulator | Gazebo | gz-transport | `/sim/gimbal/{yaw,roll,pitch}/cmd_pos` (50 Hz), the zoom plugin's field-of-view topic (on change), `/sim/camera/photo/trigger` (on a photo command) | |
| 16 | passive stream monitor (optional) | `/zr10/image_raw` | `sim_video video_monitor` | ROS topic | `SensorDataQoS` (best effort), like the board's `ros2 topic hz` | records the time of every message it gets; it is itself a best-effort reader, so it sees losses too; `--reliable` gives an exact published count for a diagnostic run (recorded as a difference); on by default only on this PC (11.2) |
| 17 | stream records | emulator | files | disk | `<run>/video/stream_frames.csv`, `rtsp_sessions.csv`, `stream_server.json` | 30 rows per second |
| 18 | emulator state | emulator | PC tools | ROS topic | `/sim/camera/emulator_state` (JSON), with new stream fields | 2 Hz |
| 19 | MAVLink | MAVROS | SITL SERIAL2 | TCP | `127.0.0.1:5763` | unchanged |
| 20 | Mission Planner camera view | `pov_server` | Mission Planner HUD | HTTP MJPEG | `127.0.0.1:8081/pov.mjpg` | unchanged output. Its input today is the ROS topic `/sim/camera/image`, which `ros_gz_bridge` converts lazily from `/sim/camera/video/image` (`sim_paths.py` lines 719-720). At 1080p30 that would push about 187 MB/s through the bridge and rclpy, so with the stream on `sim.launch.py` sets `pov_server`'s own `topic` parameter to `/sim/camera/preview`: a 640x360 rgb8 picture at up to 10 Hz that the emulator publishes from the frames it already has (C12). `pov_server`'s code does not change |
| 21 | Gazebo window camera panel ("Camera POV (live)", or `gui_camera:=window`) | Gazebo video camera | Gazebo GUI | gz-transport (the GUI reads the gz topic directly, no ROS) | `/sim/camera/video/image` | 30 Hz at 1080p when the panel is shown; its cost is measured by A5; fallback: the gz preview topic or `gui_camera:=off` (6.8) |

On the real Jetson (HIL), rows 1, 9, 11, 12 and 13 go through the Jetson design's relay (section 14). Rows 4, 7, 8
and 14 stay inside the Jetson container.

## 6. (a) The RTSP stream from the simulated gimbal camera

### 6.1 Which server, and why

| Option | Verdict | Why |
|---|---|---|
| **gst-rtsp-server, inside the camera emulator** | **chosen** | Already installed (`libgstrtspserver-1.0-0 1.20.1`, `python3-gi`, also installed by `simulation/setup/01_install_humble.sh`), LGPL. VERIFIED: it bound `192.168.144.25:8554`, and the team's exact ffmpeg command (Ubuntu ffmpeg 4.4.2) on `rtsp://192.168.144.25:8554/main.264` got 17 frames in 8 s (2.12 fps), 0 bad frames. It runs in the same process as the frame source, so there is no extra hop and no "no stream yet" window. A GStreamer client sees 12-22 ms from picture to screen (VERIFIED). It sits in the process that already owns the camera address and knows the zoom and gimbal state |
| mediamtx v1.21.3 (MIT, one downloaded program) | fallback | It works too (VERIFIED, ~2 % CPU), but it needs a separate publisher process. A reader that comes too early gets "404 Not Found" (VERIFIED). It adds ~35 ms. It also opens a Media-over-QUIC server on every address unless `moq: no` is set (VERIFIED). Use it only if a browser (WebRTC) view is wanted later |
| ffmpeg alone | impossible | ffmpeg's RTSP "listen" flag exists only for input; it cannot serve readers (VERIFIED with `ffmpeg -h`) |
| ardupilot_gazebo `GstCameraPlugin` | rejected | It hard-codes 800 kbit/s and a GOP of 10, encodes inside the physics process (that can slow the simulation), and still needs an RTSP server in front (SOURCE) |

The stream code goes into the `siyi_emulator` package as one module. It runs as a thread of the emulator process and
has a clean interface, so it can be moved into its own process later if the frame copies ever slow the emulator's
SDK loop (risk 3).

### 6.2 Stream settings

The owner said: no measurement on the real camera; use the manual's documented settings, make them configurable,
and list "exact defaults not measured" as a difference (12:25).

| Setting | Real ZR10 (manual v1.7) | Twin default | Profile key (launch argument) |
|---|---|---|---|
| address | `rtsp://192.168.144.25:8554/main.264` (the ZR10 keeps the "old" address, p.93-94) | same | `camera.stream.port` (`stream_port`), `camera.stream.path` (`stream_path`) |
| codec | H.264 or H.265, settable (p.50); the default is not published. **Both of the manual's main-stream configuration examples (p.63, SDK 0x21) set `VideoEncType` 2 = H.265**: "Main Stream to HD Resolution" (1280x720, 1500 kbit/s) and "to Ultra HD" (1920x1080, 2000 kbit/s). So `.264` in the path is weak evidence | **H.264** (owner 12:25: "H.264, 1920x1080 or 1280x720"). Every stream check (A1, A2) runs with `h265` too: `x265enc` and `nvh265enc` are present in WSL (VERIFIED). The team's ffmpeg decodes both, but H.265 costs the Jetson's CPU much more to decode, so the choice matters for the field timing (D3) | `camera.stream.codec` `h264\|h265` |
| resolution | 1920x1080 or 1280x720 (p.50); default not published | **1920x1080** (the larger of the two documented choices) | `camera.stream.width`, `height` |
| frame rate | not published for the stream; recording is 30 fps (p.15) | **30 fps**, steady by the wall clock (6.4) | `camera.stream.fps` |
| bitrate | settable in kbit/s; the manual's examples: 1500 kbit/s at 1280x720 and 2000 kbit/s at 1920x1080 (p.63, both H.265) | **2000 kbit/s**, constant | `camera.stream.bitrate_kbps` |
| GOP (keyframe interval) | not published | **30 frames (1 s)**, no B-frames; SPS/PPS sent before every keyframe so a late viewer starts fast | `camera.stream.gop` |
| encoder | camera hardware | **x264** `ultrafast` + `zerolatency` (VERIFIED cost: 84 % of one core at 1080p30); `nvh264` (NVENC) optional (VERIFIED in WSL: 30 % of one core, 4-9 % of the encoder chip) | `camera.stream.encoder` `x264\|nvh264` |
| viewers at once | up to 4 (p.21) | up to 4; a fifth is refused | `camera.stream.max_clients` |
| extra delay | glass-to-network delay not published | **0 ms** extra (the twin's own delay is recorded, 6.5) | `camera.stream.extra_delay_ms` |
| render size | - | same as the stream; may be set lower if Gazebo slows down (6.8) | `camera.stream.render_width`, `render_height` |

Only `/main.264` is served. Other paths get "404 Not Found", which is what the real ZR10 is EXPECTED to answer
(manual p.94: the `video0/1/2` paths belong to newer cameras). The zr10_ros2 README mentions `/video1`; that is a
question for the team (section 15.2).

The emulator writes the settings it really used to `$ASTRA_SIM_HOME/state/video_stream.env` (contract C2), so the
pipeline side, the run summary and `video.sh check` can read them.

### 6.3 The picture: lens, zoom, gimbal, power-on pose

- **Lens (owner 12:25: "the camera is the SIYI ZR10", ZR10 everywhere by default).** Horizontal field of view
  61.5 degrees at 1x and 7 degrees at 10x optical (manual p.15). Today both Gazebo cameras use the A8 mini's 81
  degrees. The video camera and the photo camera both get the ZR10 lens. The field of view for a zoom `z` is
  `HFOV(z) = 2 x atan(tan(61.5 deg / 2) / z)`. That gives 6.8 degrees at 10x, which matches the manual's "7 deg".
  `--lens a8mini` keeps the old 81 degrees, to compare with old runs. **The survey missions were planned for 81
  degrees.** At 30 m height a 61.5-degree camera looking straight down covers 35.7 m across instead of 51.2 m, so the
  lanes must be re-planned for the ZR10, or the search leaves gaps (a task for the worlds' owners; it is listed in
  12).
- **Zoom in the live picture.** Today the emulator zooms photos only. Now a zoom command changes the video too:
  - from 1x to 10x **optically**, by changing the video camera's field of view (ardupilot_gazebo's
    `CameraZoomPlugin`; it is built on this PC, and the emulator already knows how to drive it: SOURCE,
    `optical_zoom_topics`). Two plugin facts matter (SOURCE, `~/ardupilot_gazebo/src/CameraZoomPlugin.cc` in WSL):
    the plugin sets `HFOV = refHfov / command` with a FIXED `refHfov` of 2.0 rad (114.6 degrees, line 131, line
    428), not the camera's own field of view, and it clamps the command to `[1, max_zoom]` with a default
    `max_zoom` of 10 (line 140, only a `gzwarn` "has been clamped" when it does). The emulator already sends
    `2.0 rad / HFOV(z)` (`photo.py` `zoom_plugin_command`): 1.863 at 1x and **16.83 at 10x**. So the xacro must give
    the video camera's plugin **`<max_zoom>20</max_zoom>`** (at least 16.9). With the default 10 the picture stops
    at 0.2 rad = 11.5 degrees, which the ZR10 lens reaches at only 5.9x, and A3 would fail with nothing but a
    warning. The emulator must also send the 1x command when it starts: without any command the plugin drives the
    camera to 2.0 rad (114.6 degrees);
  - from 10x to 30x **digitally**, by cutting out the middle of each frame and scaling it back up before encoding.
    That matches the ZR10's "30x hybrid".
  The zoom speed stays the emulator's 4x per second. The real speed is unknown (12).
- **Maximum zoom.** In the `video` and `board` configurations the camera has its real range (30x). The legacy
  demo's `--zoom survey` cap (1x) stays only in `legacy`.
- **Gimbal.** As today: speed 100 = 90 degrees per second (a guess; the real scale is unknown), stabilised against
  the drone's tilt, commandable yaw +-135 degrees, pitch -90 to +25 degrees.
- **Power-on pose.** A SIYI gimbal is EXPECTED to start centred, looking forward (pitch 0). Today the emulator starts
  looking straight down, because "nobody else points it in the sim". In `video` and `board` it starts forward
  (`initial_pitch_deg=0`, an existing emulator setting). In `legacy` it stays down. With proposals off, nothing in
  the team code points the camera down, so the stream shows the horizon, as it would at the field (section 10).

### 6.4 How a frame travels inside the emulator

1. The emulator subscribes to the Gazebo video camera through gz-transport, the same way it already gets photos.
   The stream's frames never go through ROS, so the stream itself causes no 187 MB/s conversion of 1080p frames.
   (Two other viewers of the same camera would: `pov_server` reads it through the lazy ROS bridge, and the Gazebo
   window's camera panel reads the gz topic. `pov_server` therefore gets a small preview instead, and A5 measures the
   panel: rows 20-21 of section 5, 6.8.)
2. A pacing loop runs at 30 ticks per second on the monotonic clock. Each tick takes the newest Gazebo frame,
   applies the digital zoom crop if the zoom is above 10x, and pushes the frame into GStreamer. **Time stamps: the
   VERIFIED bench method is kept** (`appsrc is-live=true format=time do-timestamp=true`: GStreamer stamps each frame
   with the pipeline's running time when it is pushed, like a camera stamping by its own clock). Explicit
   `tick / 30 s` stamps would be a different method and would need the bench test again. If Gazebo has not produced
   a new frame since the last tick (the simulation runs slower than real time), the last frame is sent again and
   counted as `repeated`. A real camera also never slows its stream down.
3. GStreamer runs `appsrc ! videoconvert ! x264enc ! rtph264pay` (or `x265enc ! rtph265pay` for `h265`). One shared
   media (`set_shared(True)`, as on the bench) serves all viewers, like the camera's one encoder. The media starts
   when the first viewer connects and is unprepared after the last one leaves; the next viewer gets a fresh `appsrc`
   through the `media-configure` signal, and the pacing loop pushes into whichever `appsrc` is current (none = no
   viewer: frames are dropped there). A1b checks this reconnect path.
4. Every pushed frame (only while at least one viewer plays) becomes one row of `<run>/video/stream_frames.csv`: the
   frame number, wall and simulated time, the Gazebo frame's age, `repeated`, the drone pose, the gimbal angles, the
   zoom and the field of view. A 64x36 grey thumbnail (2,304 bytes, about 4 MB per minute) goes into
   `stream_thumbs.u8` (used to match detector frames, 11.3).
5. With `extra_delay_ms` above 0, frames wait in a small ring buffer first (to match a real delay, if one is ever
   known).

### 6.5 Latency budget (from light reaching the camera to the detector's result)

| Stage | Field (real ZR10, Jetson) | Twin on this PC | Note |
|---|---|---|---|
| picture taken -> leaves the camera | unknown (EXPECTED 0.1-0.2 s for SIYI; anecdotal) | Gazebo render (up to one frame, 33 ms) + copy + x264: EXPECTED 40-80 ms; measured in every run (`gz_frame_age_ms`) | difference 3 in section 12 |
| network | Ethernet, under 1 ms | loopback, under 1 ms (HIL: the SSH link, 1-200 ms) | |
| team ffmpeg: buffer, decode, scale, keep 2 fps | about 0.6 s (same command; the Jetson's CPU is slower) | **580-593 ms** median (VERIFIED, 95 % under 603 ms) | most of the delay is inside the team's own ffmpeg command, and the field has it too |
| wait for `stream_node`'s 0.5 s timer | 0-0.5 s, spread evenly | 0-0.5 s | the timer and ffmpeg are not in step. The message is stamped at the timer tick (`stream_node.py` line 68), so this wait is INSIDE the picture's age at the stamp |
| `msg.data = bytes` in `stream_node` | EXPECTED 0.4-0.6 s (that line is why the board's stream node used 93 % of a core) | **196-202 ms** (VERIFIED) | it runs AFTER the stamp (line 72), so it delays the arrival but not the stamp; team proposal P3 removes it |
| DDS to the detector | a few ms, if it arrives at all (11.2, Q4) | a few ms | |
| detector queue + 4 tiles + PNG/CSV | unknown on the Orin (CPU today, GPU target) | EXPECTED 0.05-0.3 s depending on the backend | measured per frame from the detector log |
| **total** | **EXPECTED 1.2-2.5 s** | **EXPECTED 0.9-1.5 s** | the twin measures it in every run (11.3) |

At 5.5 m/s, 1 s of delay is 5.5 m of flight. The proposal adapter subtracts a fixed `delay` from the stamp (section
10.3). That removes the AVERAGE age of the picture, not its spread: the timer wait alone varies evenly over 0-0.5 s,
so about ±0.25 s remains around the average, which is about **±1.4 m along the flight path at 5.5 m/s** (plus
ffmpeg's own small spread: VERIFIED p50 592 ms, p95 603 ms on this PC). Team proposal P10 removes the timer part:
`FrameSource` already records when ffmpeg delivered each picture (`frame_source.py` line 70), and `stream_node` could
stamp the message with that time instead of the timer tick. The twin measures the real spread in every run (11.3).

### 6.6 Start and stop with the simulator

- `sim.launch.py` starts the stream together with the emulator when the launch argument `rtsp:=true` is set (with
  the profile key `camera.stream.enabled`). It is on in `video` and `board` and off in `legacy`, so today's default
  behaviour stays the same until the default configuration changes to `video`.
- `stop_sim.sh` stops the emulator, and with it the stream. The team's `stream_node` then sees the connection refused
  and retries every second (its own loop). Nothing is published meanwhile, as at the field with the camera off.
- START OVER removes and respawns the drone. During those seconds there are no Gazebo frames. The stream keeps
  sending the last frame (counted as repeated), then a labelled test pattern after 10 s. It goes back to live
  pictures by itself when the new drone's camera publishes.

### 6.7 Failure behaviour

| Situation | What the twin does | What the team's `stream_node` sees |
|---|---|---|
| emulator or simulator not running | port 8554 closed | ffmpeg exits at once ("connection refused"); `FrameSource` waits 1 s and starts it again (a ~1 s cycle); a warning at most every 5 s; nothing published. (At the field, EXPECTED: a camera with no power never answers ARP, so ffmpeg's connect fails with "No route to host" after about 3 s, then 1 s wait: a ~4 s cycle. ffmpeg itself is started with no timeout option. The 10 s limit is `stream_node`'s own frame deadline, `FrameSource(timeout=10.0)` in `frame_source.py` line 25, which ends ffmpeg with `stream frame timeout` (line 59) when a connection is open but no full picture arrives for 10 s. Either way it only changes start-up timing: difference 17) |
| Gazebo frames stop | the last frame is repeated; after 10 s a test pattern that says "NO GAZEBO FRAMES"; a warning in `sim.log`; counted in `stream_server.json` | pictures keep coming, as from a real camera |
| simulation slower than real time | frames repeated to keep 30 fps by the wall clock; counted | a steady stream |
| the camera address is missing | the emulator falls back to `127.0.0.1` as it does today, with this warning: `[siyi_emulator] WARNING: RTSP listens on 127.0.0.1:8554 because 192.168.144.25 is not on this computer. The team's stream_node uses rtsp://192.168.144.25:8554/main.264 and will NOT reach it. Fix: sudo bash simulation/setup/camera_ip_alias.sh` | connection refused, retries |
| port 8554 already used | `[siyi_emulator] ERROR: cannot listen on 192.168.144.25:8554: the port is in use. Likely cause: another simulator or RTSP server runs. Fix: bash simulation/scripts/stop_sim.sh --all, then start again` | refused |
| encoder missing | the emulator runs without the stream: `[siyi_emulator] ERROR: the H.264 encoder x264enc is missing, so there is no RTSP stream. Fix: sudo apt install gstreamer1.0-plugins-ugly` (NVENC asked for but missing: a warning, then x264) | refused |
| a fifth viewer | refused ("503 Service Unavailable") | - |
| a slow viewer (HIL on Wi-Fi) | after a bounded backlog per viewer, frames for that viewer are dropped and counted per session in `rtsp_sessions.csv` | gaps; if no full picture reaches `stream_node` for 10 s, its `FrameSource` deadline ends ffmpeg and starts it again |

### 6.8 What it costs on this PC

EXPECTED, from the VERIFIED parts: x264 at 1080p30 uses about 0.85 of one CPU core (of 32 threads), plus about
0.06 of a core for the frame copies. NVENC needs about 0.3 of a core. **Not measured yet:** how much Gazebo slows
down when it renders the video camera at 1920x1080 and 30 fps (the real-time factor).

Two other readers of that camera must be counted too, because a normal visible demo runs them:

- **`pov_server`** (the Mission Planner HUD picture) reads the ROS topic `/sim/camera/image`, which the bridge
  converts lazily from the same gz topic. At 1080p30 that is about 187 MB/s through `ros_gz_bridge` and rclpy. So
  with the stream on, the emulator publishes `/sim/camera/preview` (640x360 rgb8, up to 10 Hz, made from the frames
  it already has), and `sim.launch.py` points `pov_server`'s existing `topic` parameter at it (C12). `pov_server`
  sends at most 10 pictures per second anyway (its `max_fps`).
- **The Gazebo window's camera panel** reads `/sim/camera/video/image` directly over gz-transport (copy into the GUI
  process + drawing). A5 measures the real-time factor WITH the panel and `pov_server` running. If the panel costs
  too much, the GUI configs show a gz preview topic the emulator also publishes (EXPECTED to work: the emulator
  already publishes on gz-transport from Python), or the demo uses `gui_camera:=window`/`off`.

If the real-time factor still drops below 0.7, the fallback is to render at 1280x720 and let the encoder scale up to
the stream size (`render_width/render_height`). The team's ffmpeg scales every frame down to 1280x720 anyway, so the
pictures the detector gets stay almost the same. Owner decision D9.

## 7. (b) The ZR10 SDK commands (and HTTP) the emulator must answer

The emulator already answers every SDK command and HTTP request that `zr10_ros2` can send (SOURCE: `zr10.cpp` lines
267-374 compared with the emulator's `command_handler.py` lines 102-122). What changes is what the commands do to the
VIDEO, plus a few details:

| Sent by | Command (SDK cmd, data) | Real ZR10 reply (manual) | Emulator today | Change |
|---|---|---|---|---|
| `zr10_node` `/take_photo` | 0x0C `00` | no ACK; later a 0x0B feedback | renders a 2560x1440 photo onto the SD card, sends 0x0B | photo camera gets the ZR10 lens |
| `/toggle_recording` | 0x0C `02` | no ACK; the camera records MP4 (12 Mbit/s H.265) | toggles a flag only | none needed (no team code reads videos); optional later: write an MP4 from the same stream |
| `/autofocus` | 0x04 `01` (1-byte form) | ACK `sta` | answers `01`, picture always sharp | the ZR10 model must answer 0x04, 0x06, 0x16, 0x18 and 0x19 (today's "strict" A8 mini mode stays silent on them; it stays for `a8mini` only) |
| `/zoom_direction` 1 / -1 / 0, `/stop_zoom`, the watchdog | 0x05 `01` / `FF` / `00` | ACK zoom x 10 (uint16) | zoom moves at 4x per second, photos only | the zoom drives the LIVE picture (6.3); real range 30x in `video`/`board` |
| `/gimbal_speed` (yaw, pitch -100..100), `/stop_gimbal`, the watchdog | 0x07 `yaw pitch` (int8) | ACK `sta` | 100 = 90 deg/s | none (real scale unknown) |
| `/center_gimbal` | 0x08 `01` | ACK `sta` | yaw 0, pitch 0 | none |
| at shutdown | 0x05 `00`, 0x07 `00 00` | ACKs | handled | none |
| [proposal] `gimbal_pointing` | 0x0E set angles (yaw 0, pitch -90), then 0x0D to check | ACK with the angles | implemented | none |
| [proposal] `detection_adapter` | 0x0D attitude (10 Hz), 0x18 current zoom (2 Hz) | attitude; zoom | implemented | none; 0x18 must answer in the ZR10 model (row 3) |
| nobody today | 0x20 / 0x21 stream settings | the codec, size, bitrate, fps | ignored | optional: answer 0x20 with the stream settings in use (6.2), accept 0x21 without changing anything |

Two details matter for fidelity:

- **The team's CRC typo.** One value in `zr10.cpp`'s checksum table (line 34, entry 106) is wrong: `0xCDCE` should
  be `0xCDEC`. Whenever the running checksum hits that entry, the packet's checksum is wrong. Over all 65,536
  sequence numbers (VERIFIED three times: the study, the reviewer, and
  `D:\suas_scratch\video_pipeline\crc_recheck.py`, which builds every packet exactly like `ZR10::makePacket`):

  | Command | Bad sequence numbers of 65,536 | First bad sequence |
  |---|---|---|
  | 1-byte commands: photo `0x0C 00`, record `0x0C 02`, autofocus `0x04 01`, zoom `0x05 01`/`FF`/`00` (the zoom STOP included) | 1,017-1,021 (1.55 %) | 38 |
  | gimbal speed `0x07 yaw pitch`, **the gimbal STOP `0x07 00 00` included** | 1,267-1,273 depending on the speeds (1.94 %); the STOP: 1,269 | 43 |
  | centre `0x08 01` | 1,019 (1.55 %) | 5 |

  `zr10_node` counts one sequence number per packet from 0 at its start, so which packet is hit depends on the
  order of commands. **The gimbal STOP is the most affected command**, which makes P1 more urgent: `zr10_node`'s
  watchdog sends ONE stop frame 0.5 s after the last motion command and then marks the gimbal as stopped (it never
  reads the camera's reply, SOURCE `zr10_node.cpp` lines 91-108), so a dropped stop frame leaves the gimbal turning
  at the last speed until someone sends another command. The emulator already drops frames with a bad checksum, as
  the real camera very probably does (not
  verified on hardware). It keeps doing so, which reproduces the field behaviour. New: it counts them
  (`sdk_bad_crc` in `/sim/camera/emulator_state`) and writes the first 20 of them in hex to
  `<run>/video/sdk_bad_crc.log`, so the run summary can show the effect of team proposal P1.
- **Replies go back to the sender's port**, as today. `zr10_node` never reads them; on the board its socket held 768
  unread bytes (PROBE). That is harmless.

**HTTP (TCP 82):** no change. `zr10_node`'s photo sync is off on the board (`sync_photos` defaults to false and is not
set). If a teammate turns it on, the emulator already answers `getdirectories`, `getmediacount`, `getmedialist` and
the file download. One small difference: the real ZR10 pads its JPEG files with zeros after the end marker, and
`zr10_node` accepts both forms (SOURCE: `media.cpp` lines 74-104).

## 8. (c) Starting the team's nodes next to the existing team stack

Common rules for every mode:

- **Unchanged commands.** The team's command lines are used word for word. The only edits are the ones in the table
  of 8.3, and every one of them is written to `<run>/video/commands.txt`, each with the reason.
- **Their own folders are never written.** Builds go to `~/.astra_sim/video/ws/` (8.6 and section 9). The detector's
  `/data` folder is the run folder `<run>/video/yolo26/`, not the team's `objectDetection` folder. (Their
  entrypoint creates a link to the model in `/data`, and the results land there.) Programs started with a relative
  output path run with the working directory `<run>/video/cwd/`.
- **Start order** (like the board's last restart): the camera control `zr10_node`, then the detector, then the
  stream.
- **Stop order:** first the detector with `docker kill --signal=INT <name>` (= the team's Ctrl+C, so it prints its
  summary lines), up to 10 s, then `docker stop -t 5`; then the stream and `zr10_node` (it sends its two stop
  frames), then the rest. The detector runs WITHOUT `-t`: under a launch file or a script there is no terminal, the
  docker CLI may not pass a SIGINT on to a container with a TTY, and ROS writes its log lines to unbuffered stderr
  anyway, so nothing is lost. (The board ran it with `-it` from a person's terminal: difference 22.)
- **Leftover containers:** before starting, a stopped or running container named `astra-video-detector` WITH our
  label `org.astra.owner=video` is removed (`[video] NOTE: removed a leftover detector container from an earlier
  run`). One with that name but WITHOUT our label is never touched: `[video] ERROR: a container named
  astra-video-detector exists and is not ours (no label org.astra.owner=video). Why: our tools only stop their own
  containers. Fix: docker rm -f astra-video-detector if you know it is safe, then start again.`
- **Root-owned results:** with proposals off (and in `board`) the detector runs as root, so its result files in
  `<run>/video/yolo26/` belong to root and the WSL user could not delete the run folder. `video.sh collect` gives
  them back after the detector has stopped, with the same image and no sudo: `docker run --rm --label
  org.astra.owner=video -v <run>/video/yolo26:/data --entrypoint chown yolo26-ros-subscriber -R UID:GID /data`.

### 8.1 On this PC, natively in WSL (the normal demo)

**Configuration `video`.** demo.sh starts the simulator with the stream on, uploads the mission, and runs THIS
repo's `./launcher.sh sim` (unchanged; it honours `TRACKER_CMD`, line 78) with this `TRACKER_CMD` (PLANNED; the
overlay makes three workspaces visible: the simulator's, the team checkout's, and the video workspace with the
team's video packages and our proposals). demo.sh builds the line with every path already expanded:

```bash
# R = the Linux copy of this repo (e.g. /home/jarrod/ODLC_Machine_Inferencing_System_2025-2026), T = the team checkout
# (= R unless --team-repo), V = $HOME/.astra_sim/video/ws, RUN = the run folder. All absolute, none with a space.
TRACKER_CMD="bash $R/simulation/video/lib/video_overlay.sh --sim-ws $R/ros2_ws --team-ws $T/ros2_ws --video-ws $V -- \
  ros2 launch $R/simulation/video/proposals/field_video.launch.xml detector:=docker detector_image:=<image> \
  detector_data:=$RUN/video/yolo26 detector_user:=match image_qos:=reliable proposals:=true"
```

Why absolute paths: `launcher.sh` runs `$TRACKER_CMD &` unquoted (line 138). The shell splits it at spaces, does
not remove quotes and does not expand `~`. So a relative path would break when `launcher.sh` runs from a
`--team-repo` checkout, `~` would stay a literal `~`, and a path with a space would be cut in two. demo.sh refuses
such a path: `[video] ERROR: the path "<p>" contains a space. Why: launcher.sh splits TRACKER_CMD at spaces (it runs
$TRACKER_CMD unquoted), so the command would break. Fix: use a folder without spaces (the Linux copy
~/ODLC_Machine_Inferencing_System_2025-2026 has none).`

**The launcher must honour `TRACKER_CMD`.** The board's `launcher.sh` (`6beb5f1`, line 73) does not: it always runs
`tracker.launch.xml`, i.e. the obsolete photo nodes. With `--team-repo` pointing at a checkout like that, `video.sh`
and demo.sh stop before flying: `[video] ERROR: <dir>/launcher.sh ignores TRACKER_CMD (it sets
TRACKER_CMD="ros2 launch main tracker.launch.xml" itself, like the board's launcher at 6beb5f1). Why: the video
configuration starts its launch file through TRACKER_CMD; this launcher would start the obsolete photo nodes
instead, and the readiness check would then time out waiting for zr10_camera. Fix: use a copy of that checkout with
the one-line change of team proposal P7 (TRACKER_CMD="${TRACKER_CMD:-...}"), or drop --team-repo.` The test is
whether the file contains `${TRACKER_CMD:-`. The existing `team_repo.py` check only looks for the word `TRACKER_CMD`,
which the old launcher also contains, so it lets that checkout through: a patch for its owner is in 13.5.

The readiness check uses the node list of the configuration. `stack_check` already has the options this needs:
`--skip-node siyi --skip-node mission_camera_trigger --skip-node new_od --add-node zr10_camera --add-node zr10_stream
--add-node detector_node` (plus `--add-node detection_adapter --add-node gimbal_pointing` with proposals on). `/sahi/get_health` does not
exist here, so instead of `--need-detector` the check waits for the detector's "ready" log line (contract C9).

**Configuration `board`.** The team stack starts as today (`TRACKER_CMD` = `team_stack.launch.py`, which includes
the unchanged `tracker.launch.xml`). Note what that wrapper adds: two global parameters, `model_path` (the PC's
`yolo26m.pt`) and `use_batched_inference` false. So natively in WSL `new_od` runs the `.pt` SAHI path on PyTorch,
not the board's TensorRT engine with batched inference (difference 15). (The virtual Jetson's TensorRT image runs
the unchanged `tracker.launch.xml` with an engine built for this PC's GPU: HIL design, revision 4.) Then
`video.sh start --pipeline board --run-dir <run>` starts the new pipeline the way the team starts it by hand on the
board (the detector as root, the team's QoS: section 3):

```bash
cd <run>/video/cwd
ros2 run zr10_ros2 zr10_node --ros-args -p dry_run:=false                      # log: <run>/video/zr10_node.log
docker run --rm --name astra-video-detector --label org.astra.owner=video \
  --network host --ipc host -e ROS_DOMAIN_ID=0 -v <run>/video/yolo26:/data yolo26-ros-subscriber \
  ros2 run yolo26_ros detector_node --ros-args -p input_mode:=topic -p image_topic:=/zr10/image_raw \
  -p model_path:=/opt/model/yolo26.onnx -p output_dir:=camera_results         # log: <run>/video/detector.log
ros2 launch zr10_stream stream.launch.py fps:=2.0                               # log: <run>/video/stream.log
```

(The board ran the detector with `-it` from a terminal; here it has no `-t` and is stopped with `docker kill
--signal=INT`, see the common rules above. After the run, `video.sh collect` gives the root-owned result files back
to the WSL user.)

`ROS_DOMAIN_ID=0` is the twin's domain too (the simulator never changes it, like the board). If a user runs the twin
on another domain, that number is passed instead (difference 21).

### 8.2 The proposed field launch file (owner 12:10)

`simulation/video/proposals/field_video.launch.xml` (the file name may change; the arguments below are the contract,
C8). It is XML like the team's `tracker.launch.xml`. It starts, in this order:

- `prepare_photo_folder` (a small script of the proposal package, 10.2): it makes sure the photo folder that
  `main_controller` checks at the end of the survey exists, finding it the same way `main_controller` does (from
  `$(find-pkg-prefix main)`, the first parent folder with both `install/` and `src/`, then
  `src/video_cam/mapping_photos`; argument `photo_dir` overrides it). It never deletes anything. If the folder holds
  old photos it prints `[photo_folder] WARNING: <dir> holds N old photos. main_controller will wait 90 s for them at
  the end of the survey, because nothing processes photos in the video pipeline. Empty it before the flight (move the
  photos elsewhere).`;
- `zr10_ros2 zr10_node` with `dry_run` false;
- the `zr10_stream` launch with `fps` 2.0 (include of `$(find-pkg-share zr10_stream)/launch/stream.launch.py`);
- the detector, depending on `detector:=docker|external|none`:
  - `docker`: an `executable` that runs `run_detector.sh` (proposal package), a 30-line wrapper that removes a
    leftover container of ours, runs the team's `docker run` with the flags of 8.3 (named, labelled, no `-t`), and
    on SIGINT or SIGTERM runs `docker kill --signal=INT <name>`, waits up to 10 s for the team's summary lines, then
    `docker stop -t 5`. The launch must not escalate before that: its default is SIGTERM 5 s after SIGINT, from the
    launch configuration `sigterm_timeout` (Humble's XML `<executable>` has no attribute for it; SOURCE
    `launch/actions/execute_local.py` lines 90-93). So the wrapper's `executable` sits in a scoped `<group>` with
    `<let name="sigterm_timeout" value="15"/>`. (Launch XML has no shutdown event handler, hence the wrapper.) It
    prints its exact `docker run` line first, so `commands.txt` holds it;
  - `external`: nothing, because someone else starts it (the Jetson container does this, section 14);
  - `none`: no detector at all;
- `waypoint_mavros waypoint`, `gps_ros2 gpstest`, `main main_controller.py`, copied exactly as `tracker.launch.xml`
  declares them (same names, no parameters);
- with `proposals:=true` also the two proposal nodes (section 10).

It does NOT start `video_cam siyi`, `mapping trigger` or `detection new_od`. Arguments: `detector`,
`detector_image`, `detector_data`, `detector_user` (`board` = root like the board, `match` = `--user UID:GID` like the
team's own readme: P4), `detector_backend`, `image_qos` (`sensor_data` = the team's QoS, `reliable` = P9),
`domain_id` (passed to the container; default `$(env ROS_DOMAIN_ID 0)`), `stream_fps` (2.0), `photo_dir`,
`proposals`. The twin always passes every detector argument explicitly, following the table of section 3: proposals
on -> `detector_image:=astra-video-detector:<flavor> detector_user:=match image_qos:=reliable`; proposals off ->
`detector_image:=yolo26-ros-subscriber detector_user:=board image_qos:=sensor_data detector_backend:=opencv_cpu`.
With the team's image the wrapper adds none of the patch parameters (`backend`, `image_qos`, `detections_topic`), so
the team's command stays word for word. The defaults written in the file are the proposed field target (proposals
on, the Jetson image, TensorRT), because they only matter on the drone.

**How it gets started, on the twin and on the board.** This repo's `launcher.sh` honours `TRACKER_CMD` (line 78),
so the twin, the virtual Jetson and HIL start the file through it (8.1, 8.4). **The board's `launcher.sh` does not**
(`6beb5f1`, line 73, 2.1): it always runs `tracker.launch.xml`. Owner 11:16's "launcher.sh unchanged" cannot hold for
the new pipeline on the board: today's boot path starts the obsolete photo nodes, so something in it must change.
The proposals' README offers the team one change, **P7**: copy `field_video.launch.xml` into `main/launch/`, copy the
`astra_video_proposals` folder into `ros2_ws/src/`, rebuild, and change `launcher.sh` line 73 to

```bash
TRACKER_CMD="${TRACKER_CMD:-ros2 launch main field_video.launch.xml}"
```

That boots the field pipeline with no environment variable, and the simulator can still override it. (The smaller
form `TRACKER_CMD="${TRACKER_CMD:-ros2 launch main tracker.launch.xml}"`, as in this repo, only adds the hook; then
the team must set `TRACKER_CMD` wherever their boot service starts `launcher.sh`, which we do not know: T9.)

### 8.3 The detector container

The two columns follow the one rule of section 3 (C11).

| Item | Proposals OFF: `video --no-proposals` and `board` (= the board today) | Proposals ON: `video`, the default (= the proposed field target) |
|---|---|---|
| image | the team's `yolo26-ros-subscriber`, built from their Dockerfile unchanged, from the folder the user points to (today: the board's uncommitted tree, 2.1; on this PC: amd64) | `astra-video-detector:<flavor>`: the team's source + our labelled patch set (`simulation/video/proposals/detector_gpu/`) on a TensorRT base: PC `nvcr.io/nvidia/tensorrt:24.08-py3` (TensorRT 10.3, CUDA 12.6, Ubuntu 22.04: EXPECTED, the builder checks the versions), Jetson `nvcr.io/nvidia/l4t-jetpack:r36.4.0`. The same Dockerfile with two base layers (owner 12:32) |
| model | built into the image at build time by the latest Ultralytics (not pinned): stock COCO `yolo26n` | ONE `yolo26.onnx`, exported once on the PC with the team's own export command and a pinned Ultralytics version, SHA-256 recorded, mounted read-only at `/opt/model/yolo26.onnx`, so the team's `model_path` stays word for word. Every machine builds its own engine from that same file |
| backend | OpenCV DNN on the CPU (the only one in the team's image) | `auto`: (1) **TensorRT** when the container sees a GPU (engine cached in `~/.astra_sim/models/yolo26/` on the PC, `/models` on the Jetson; built on the Jetson only in the owner-approved warmup step); (2) **`ort_cuda`**, on this PC only, when Docker cannot reach the GPU: our wrapper runs the same ONNX with ONNX Runtime CUDA natively in WSL, with the team's exact pre- and post-processing (tested against the C++ output on the same frames); (3) **`opencv_cpu`**, the team's code path, as the last choice |
| GPU flags | none | PC `--gpus all`; Jetson `--runtime nvidia` |
| user | **root** (no `--user`), as on the board | **`--user UID:GID`** (team proposal P4, the form of the team's own readme). Needed for ANY picture to arrive (root vs user: 11.2) |
| image QoS | the team's `SensorDataQoS` (best effort): many 2.7 MB pictures are lost even for the same user (VERIFIED, 11.2) | **reliable, depth 5** (team proposal P9, patch parameter `image_qos:=reliable`): 40 of 40 arrived in the same test |
| keyboard | the board: `-it`; the twin: none (no `-t`, stopped with `docker kill --signal=INT`, 8 common rules) | the same: none |
| name, label | the board: none; the twin: `--name astra-video-detector --label org.astra.owner=video` (so our tools only ever stop our own container) | the same |
| `/data` | the board: the team's `objectDetection` folder; the twin: `<run>/video/yolo26/` (root-owned files given back after the run) | `<run>/video/yolo26/` |
| outputs | files only | the same files, PLUS (patch) `/yolo26/detections` and one start-up line `backend=... device=... engine=... precision=... model_sha256=...` (C9) |

The patch set has only clearly separated changes, as the owner asked (12:25). Each one is a `.patch` file with a
README: (1) a `backend` parameter, default `opencv_cpu`, so an unpatched command line behaves exactly as today; (2)
the TensorRT backend, using the same letterbox tiles, the same output parsing, the same confidence 0.25 and NMS 0.45;
(3) an optional `detections_topic` publisher; (4) the backend start-up line; (5) `Dockerfile.gpu` with `ARG BASE`;
(6) an `image_qos` parameter, default `sensor_data` (= the team's `SensorDataQoS`, so an unpatched command line
behaves exactly as today), `reliable` = RELIABLE depth 5 (P9). It is applied to a COPY of the team's folder at build
time, never to their folder. The TensorRT precision (FP16 or FP32) is a decision for the team (D6); the twin default
is FP16, the usual choice on a Jetson, and it is recorded.

### 8.4 In the "virtual Jetson" container on this PC

**This is the acceptance test now (owner 15:14).** The Jetson design's container (`astra-hil-virtual`: sealed
network, ROS domain 77, `ROS_LOCALHOST_ONLY=1`, started with `--ipc shareable`: `jetson.sh`'s `run_cmd_sim` already
passes it, line 933) runs the image's `./launcher.sh` with `TRACKER_CMD` = the proposed launch file and
`detector:=external`. That `launcher.sh` is THIS repo's copy (layer 3 copies the repo), so it honours `TRACKER_CMD`;
the board's would not (difference 9). Its image also carries `zr10_ros2`, `zr10_stream`, the proposal package
(`astra_video_proposals`, built) and `ffmpeg`. The detector runs as a second container
(`astra-hil-virtual-detector`, `jetson.sh`'s `${JETSON_CONTAINER}-detector`) that joins its network and IPC
(`--network container:astra-hil-virtual --ipc container:astra-hil-virtual`), with the settings of section 3: proposals
on = the patched image, the stack's user, `--gpus all`, TensorRT with an engine built for this PC's GPU, reliable QoS;
proposals off = the team's image as root, no GPU flags. The camera traffic (RTSP 8554, SDK 37260, HTTP 82) goes
through the relay's `pc-local` role to the emulator on WSL's `192.168.144.25`. Section 14 lists what that design
must add. With the owner's optional "Jetson-like" limits (6 CPUs, about 7.4 GB of memory, owner 12:32), the timing
gets closer to the board, but it is still not the board's.

### 8.5 On the real Jetson

The same as the virtual Jetson, but on the board through the SSH relay, with the arm64 image built off the board.
The engine is built on the board once (owner's OK, team idle). **Not run now** (owner 15:14: no real Jetson for the
moment); the steps stay documented with exact commands in the Jetson design. Details and the exact changes: section
14.

### 8.6 What this PC needs

| Need | For | Status | How |
|---|---|---|---|
| `ffmpeg` | `zr10_stream` natively | NOT installed in WSL (VERIFIED) | `sudo apt install ffmpeg` (owner, D8). The virtual Jetson image gets it from apt |
| `nlohmann-json3-dev` | building `zr10_ros2` natively | NOT installed in WSL (VERIFIED) | `sudo apt install nlohmann-json3-dev` (owner, D8); the dev image already has it |
| `libcurl4-openssl-dev`, std_srvs, geometry_msgs, sensor_msgs | `zr10_ros2` | installed (VERIFIED) | - |
| GStreamer + gst-rtsp-server + x264enc (+ x265enc, nvh264enc, nvh265enc) | the stream | installed (VERIFIED) | already in `01_install_humble.sh` |
| Docker Engine with GPU access in WSL | TensorRT detector | the Docker Engine is installed; GPU inside a container not tested yet | `simulation/docker/install_docker_engine.sh` installs the NVIDIA Container Toolkit; allowed by the owner (12:25) |

Without the two apt packages, the pipeline can still run in the virtual Jetson container (it needs no sudo).

## 9. (d) Where the team code comes from, and how to update it

**One pointer: `video.root`** (`--video-root DIR`, profile key `video.root`). It is a folder laid out like the
board's home folder:

```
<video.root>/astra_ws/src/zr10_ros2/                                  camera control (the copy that builds on Humble)
<video.root>/astra_camera_detection/astra_ws/src/zr10_stream/         the stream node
<video.root>/astra_camera_detection/astra_ws_d/objectDetection/       the detector (Dockerfile, sources)
```

Each part can also be pointed to on its own: `video.zr10_ros2`, `video.zr10_stream`, `video.detector_dir`
(`--zr10-ros2 DIR`, `--zr10-stream DIR`, `--detector-dir DIR`). Where the folder can come from:

1. a copy of the board's two folders (`~/astra_ws`, `~/astra_camera_detection`), made with the owner's and the
   team's OK. A read-only snapshot taken during the probe has exactly this layout;
2. the team's git, where it exists (2.1): `zr10_ros2` on the ASTRA repo, branch `video-cam` (its `CMakeLists.txt`
   fails on Humble until the team applies proposal P2); the detector on `github.com/llaammbbddaa/objectDetection`,
   **but no pushed branch runs the board's command**: the board's build is the local branch
   `codex/ros2-image-subscriber` plus uncommitted changes, and the committed code subscribes `/camera/image_raw`
   whatever `image_topic` says. `zr10_stream` is in no git we know. Decision D4 asks the team to commit and push both;
3. any teammate's checkout with the same layout.

What the twin does with it (`video.sh check` and `video.sh build`, PLANNED):

- **Read-only.** It builds the two packages by their exact folders, from a neutral working folder, so colcon finds
  nothing else and writes nothing next to the team's files:

  ```bash
  cd ~/.astra_sim/video/ws && source /opt/ros/humble/setup.bash && \
  colcon --log-base ~/.astra_sim/video/ws/log build \
    --base-paths <video.root>/astra_ws/src/zr10_ros2 <video.root>/astra_camera_detection/astra_ws/src/zr10_stream \
    --packages-select zr10_ros2 zr10_stream \
    --build-base ~/.astra_sim/video/ws/build --install-base ~/.astra_sim/video/ws/install
  ```

  Why exact folders: a base path at `video.root` would also find the second `zr10_ros2` (the PRIVATE copy under
  `astra_camera_detection/astra_ws/src/`), `yolo26_ros` and `astra_live_detection`; two packages with one name stop
  colcon. `--log-base` keeps colcon's `log/` out of the current folder. No `--symlink-install`: with symlinks, Python
  writes its `__pycache__` next to the team's scripts (the board's own copy has one). The proposal package
  `astra_video_proposals` is built the same way into the same install folder. The detector image is built from a
  COPY of their folder. A test checks that the team folders' checksums are the same before and after.
- **Checks with clear errors:** each package folder exists with its `package.xml`; `zr10_ros2/CMakeLists.txt` line 17
  is not the PRIVATE form (otherwise: `[video] ERROR: <path>/CMakeLists.txt line 17 uses "PRIVATE CURL::libcurl
  Threads::Threads", which does not build on ROS 2 Humble (it mixes CMake's plain and keyword forms). Likely cause:
  this is the video-cam branch copy. Fix: point --zr10-ros2 at the copy from the Jetson's ~/astra_ws, or ask the team
  to apply proposal P2.`); `ffmpeg` and `nlohmann-json3-dev` are installed for native runs; the detector image exists.
- **Records what ran:** for each part, its path; the git remote, branch and commit, and `dirty: yes|no` with the list
  of changed and untracked files (read with `git --no-optional-locks status --porcelain`, which writes nothing), or
  `not in git`; and a SHA-256 over its files. Today that gives `zr10_ros2: not in git (same .cpp as video-cam f496beb,
  CMakeLists line 17 differs)`, `zr10_stream: not in git`, `detector: llaammbbddaa/objectDetection
  codex/ros2-image-subscriber b02cac6, dirty: yes (8 modified, 2 untracked), local branch not on origin`. Also the
  detector image ID, the patch set's checksum, the model file's SHA-256 and the backend. All of this goes into
  `<run>/run_meta/video.json` (schema `astra.run_meta.video.v1`) and the run summary.
- **Updating:** `git pull` in the team checkout (or copy the board's folders again), then `video.sh build`. Before
  each run, `video.sh check` compares the checksums with the last build and stops with `[video] ERROR: the team's
  zr10_stream changed since the last build. Fix: bash simulation/video/video.sh build` if they differ.
- **Main team stack:** `run.team_repo` (`--team-repo DIR`) works as before for `legacy` and `board`. Without it the
  twin runs this repo's newer team copy, which has the October 2026 fixes (`processing_hold.py`,
  `target_geolocation.py` and others: difference 14). **A checkout of the board's commit `6beb5f1` cannot run the
  `video` configuration as it is**: its `launcher.sh` ignores `TRACKER_CMD` (8.1), so it would start the photo nodes
  instead of the launch file, and `video.sh` refuses it with the error of 8.1. To fly the board's main stack with the
  video pipeline anyway, make a copy of that checkout that YOU own, apply P7's one line to its `launcher.sh`, and point
  `--team-repo` at the copy. The run records `launcher.sh`'s SHA-256 (the run metadata already does) and lists
  "launcher.sh = 6beb5f1 + P7" as a difference. (With the board's `main_controller` the photo-folder step of the
  launch file matters most: 10.2.)

## 10. (e) What the mission logic consumes

### 10.1 In plain words

**The team's new pipeline does not feed `main_controller`.** `main_controller` only listens to `/image_detection`.
The new detector publishes nothing on it; it writes files. Nothing gives a detection a ground position. Nothing
points the camera down. With only the team's code, a new-pipeline mission flies the survey and comes home without
dropping (section 2.2). The team has not decided how to connect them (owner, 12:25).

### 10.2 What the twin does in each configuration

| Configuration | Detections reach `main_controller`? | Expected mission | Score report |
|---|---|---|---|
| `video`, proposals on (default) | yes, through our labelled adapter | survey, hold, targets, drops, RTL | targets, aim error and drop error as today, plus the per-frame video scores (11.6); every page says "PROPOSAL (not adopted by the team): gimbal pointing, detection topic, adapter, GPU detector" |
| `video --no-proposals` | no | survey, then `No valid targets found`, RTL, no drops | 0 targets; the detector scored from its files only (EXPECTED: few or none, because it runs as root like the board, 11.2); "mission link: not connected (the team's code today)" and how many pictures the detector received, with the likely cause |
| `board` | yes, from the old photo path (`new_od`) | as today's simulator; the new pipeline runs alongside and only writes files | as today, plus the video scores |
| `legacy` | yes (`new_od`) | as today | as today |

**The photo folder: a field-safety item.** At the end of the survey `main_controller` holds in GUIDED and counts the
photos in `<team ros2_ws>/src/video_cam/mapping_photos` (SOURCE, both versions):

| The folder is ... | Board's `main_controller` (`6beb5f1`) | This repo's `main_controller` |
|---|---|---|
| present and empty | `All 0 images processed`, at once | the same |
| holding old photos (nobody processes them in the video pipeline) | `Processing timeout (0/N)` after 90 s, then on | the same |
| **missing** | **the hold never ends** (lines 306-342: after 90 s `os.listdir` raises `FileNotFoundError` inside the `try`; the `except` only logs `Error checking processing status`, every 2 s). The drone hovers at the last waypoint until a failsafe or the pilot | ends after 90 s (`processing_hold.py`, commit `94ccf0b`) |

On the board today the folder exists only because the boot launcher's `siyi` node and `new_od` create it (it is
git-ignored). The proposed launch file removes both. So:

- **The proposal handles it.** The launch file's first step, `prepare_photo_folder` (8.2), creates the folder exactly
  where `main_controller` looks, never deletes anything, and warns when old photos are in it. The proposals' README
  and `TEAM_PATCHES.md` say why (with the line numbers above), and **P8** asks the team to adopt this repo's
  `processing_hold.py` fix as well, so a missing folder can never keep the drone hovering.
- **In the twin**, the folder is the runtime photo folder of the team copy the simulator runs: this repo's Linux copy,
  or the `--team-repo` checkout. `demo.sh` already archives old photos there before every run
  (`astra_archive_photos`, as today), and the launch file's step creates the folder when it is missing. The team's
  video code (`video.root`) is never written. Because the twin runs the same launch file, it behaves like the field
  would with the proposal; without the proposal's step it would not (difference 24).

### 10.3 The proposal (`simulation/video/proposals/`, owner 12:10 and 12:25)

A small ROS 2 package, `astra_video_proposals` (ament_python, team style), with two nodes, two small scripts
(`prepare_photo_folder`, `run_detector.sh`) and its own copy of the ground projection, plus the detector patch set and
a README. **It needs nothing from the team's `main` package**: it depends only on packages the board already has at
`6beb5f1` (`interfaces` for `ImageResult`, `vision_msgs`, `sensor_msgs`, `mavros_msgs`, `std_msgs`). The team adopts
it by copying the folder into their `ros2_ws/src`, plus the launch file into `main/launch/` with P7 (8.2), and
rebuilding. The twin builds it into the video workspace, never into this repo's `ros2_ws`.

**`gimbal_pointing`: who points the camera down.** It listens to `/mavros/statustext/recv`. On `Mission: N
DigiCamCtrl` (the survey's camera item, still in the mission) it sends SDK 0x0E (yaw 0, pitch -90) once, like last
year's `capture_setup`. It checks the result with 0x0D and tries at most 3 times. Zoom stays at 1x. The team could
instead add a "set angles" service to `zr10_node` (question T2).

**`detection_adapter`: detections into the message `main_controller` already reads.**

- Inputs: `/yolo26/detections` (one message per frame, header = the picture's `stream_node` time); MAVROS position,
  height above home and compass heading (1 Hz, kept in a 10 s buffer); the gimbal attitude from SDK 0x0D (10 Hz)
  and the zoom from 0x18 (2 Hz), on its own UDP socket.
- The picture's real time is `header.stamp - delay`. `delay` is a parameter (default 1.0 s, EXPECTED from 6.5). The
  twin measures the real value in every run (11.3) and the summary prints it, so the parameter can be tuned. A fixed
  delay removes only the average: about ±0.25 s remains from `stream_node`'s timer (±1.4 m along the path at 5.5 m/s)
  until the team adopts P10 (6.5).
- For each box of a stand-in class, it builds the same fields video_cam's photo sidecar has (height above home,
  heading, gimbal pitch and yaw, zoom, field of view from 6.3, picture size 1280x720), interpolated to the picture's
  time, and projects the box centre onto flat ground at home height with
  `astra_video_proposals/geolocation.py`. **That file is a labelled copy of this repo's
  `ros2_ws/src/main/src/target_geolocation.py`** (pure Python, no ROS imports; added here by commit `81a7950`, so the
  board's `6beb5f1` does not have it). Its header names the source file and its SHA-256, and a test in the twin
  compares the two, so a change to one is noticed. **Neither version of `main_controller` projects a video
  detection:** the board's (`6beb5f1`, lines 383-384) stores `msg.latitude`/`msg.longitude` as they come; this repo's
  projects only photos that have a sidecar file and otherwise also uses the message's latitude/longitude. So the
  adapter's projection is the only one in the video path, with either version.
- Classes (stand-ins until the team trains a model with mannequin and tent classes): COCO `person` -> `"0"`; COCO
  `umbrella` and `kite` -> `"1"` (the map last year's `new_od` used). The map is a parameter.
- Output: one `interfaces/msg/ImageResult` on `/image_detection` (RELIABLE depth 10) per frame that has at least one
  mapped box. `latitude`/`longitude` hold the projected ground position. `image_name` is `zr10_frame_<stamp>`,
  deliberately NOT in the photo-name form `"<lat> , <lon>"`. With no photo sidecar for that name, `main_controller`
  aims at the message's own latitude/longitude, i.e. at the adapter's projected point. That holds for this repo's
  version (SOURCE: `locate_detection` -> `aim_at_box` falls back to the "photo position" when there is no sidecar)
  and for the board's version at `6beb5f1`, which stores `msg.latitude` and `msg.longitude` directly (SOURCE).
  Frames with no mapped box are only logged, not sent: `main_controller` sends the text `No objects detected.` for
  every empty message, which would flood the ground station at 2 per second.
- Limits, stated in the README: latitude and longitude are `float32` in the message (about 0.4-0.7 m of rounding, as
  with photos); the drone's position comes at 1 Hz (interpolated); flat ground; the best single box per class wins
  (`main_controller`'s existing rule); a box seen while the gimbal is still moving is skipped (0x0D changes faster
  than 5 deg/s).
- It logs every decision to `<run>/video/adapter.csv` (11.4).

### 10.4 The questions for the team (copied into the proposals' README)

- T1: How should detections reach `main_controller`: the proposed `/yolo26/detections` + adapter, the detector
  publishing `ImageResult` itself, or something else?
- T2: Who points the camera at the ground, and when? Our proposal sends 0x0E at the survey start. Is there a pre-flight
  step with the SIYI app, a fixed angle, or a `zr10_node` service to come?
- T3: Where should target positions come from: the drone pose plus gimbal angles at the picture's time (our
  proposal), or something else? Which delay should be subtracted from `stream_node`'s publish time?
- T4: Which classes stand for the mannequin and the tent, and which model flies (stock COCO `yolo26n` today)?
- T5: Is 2 fps the field value for `stream_node` (the launch default is 10)?
- T9: How does `launcher.sh` start at boot on the flight Jetson (a systemd service, a desktop autostart, by hand)?
  P7 depends on it.
- T10: Is `mapping_photos` emptied before each flight today? With the new pipeline, old photos there make the end of
  the survey wait 90 s, and a missing folder makes the board's `main_controller` hover for ever (10.2).

## 11. (f) Logging: what every run records from the new pipeline

### 11.1 Files in the run folder

| File | Written by | What |
|---|---|---|
| `video/stream_frames.csv` (+ `stream_thumbs.u8`) | emulator | one row per frame sent: `frame, unix_time_s, wall_time_utc, sim_time_s, gz_frame_age_ms, repeated, late_ms, enu_e_m, enu_n_m, enu_u_m, drone_roll_deg, drone_pitch_deg, drone_yaw_deg, gimbal_yaw_deg, gimbal_pitch_deg, gimbal_roll_deg, zoom_x, hfov_deg, digital_crop, clients` (+ a 64x36 grey thumbnail per frame); rows only while at least one viewer plays |
| `video/rtsp_sessions.csv` | emulator | one row per viewer: address, transport, connect / play / end time, frames and bytes sent, dropped for a slow viewer, why it ended |
| `video/stream_server.json` | emulator | the settings really used, totals, repeated and late frames, the longest gap, test-pattern seconds, warnings |
| `video/sdk_bad_crc.log` | emulator | the first 20 SDK frames with a bad checksum (hex), and the count |
| `video/commands.txt` | `video.sh` / demo.sh | every command line used, and each place where it differs from the board, with the reason |
| `video/zr10_node.log`, `stream.log`, `detector.log` | `video.sh` (`board`) or split out of `team_stack.log` by its `[node-name]` prefixes (`video`) | the three programs' own output |
| `video/yolo26/camera_results/csv/frameN.csv`, `annotated_images/annotated_frameN.png` | the team's detector | its normal outputs. Note the size: EXPECTED 1-3 MB per PNG, at 2 per second (`logs.sh bundle` should leave them out by default) |
| `video/detector_backend.json` | `video.sh`, from the detector's start-up line | backend, device, engine checksum, precision, model SHA-256, image ID |
| `video/monitor.csv` | `sim_video video_monitor` (optional) | one row per `/zr10/image_raw` message: arrival time, header time, gap since the last one |
| `video/adapter.csv` | [proposal] adapter | per box: picture time, the pose used and its age, gimbal angles and their age, zoom, class, confidence, box centre, projected lat/lon, sent or why not |
| `video/video_summary.json`, `video/differences.txt` | `sim_video video_export` | the summary block (11.5) and this run's list of differences from the field (one item per line) |
| `run_meta/video.json` | `video.sh` | where the team code came from (section 9) |
| `csv/video_stream.csv`, `csv/video_detector_frames.csv`, `csv/video_detections.csv` | `sim_video video_export` | the spreadsheet files (11.4) |

### 11.2 Stream health: where each number comes from

| Number | Source | Works on the real Jetson? |
|---|---|---|
| frames sent, frame rate, repeated and late frames, gaps, viewers | emulator records (`stream_frames.csv`, `rtsp_sessions.csv`) | yes (the server is on the PC) |
| ffmpeg restarts and missing stream | `stream_node`'s warnings: `waiting for stream`, `decoder exited or stream ended`, `stream frame timeout`, `latest frame is stale` | yes (collected log) |
| `/zr10/image_raw` rate and gaps | `video_monitor` (a passive best-effort subscriber, like the board's `ros2 topic hz`), on by default on this PC only. It loses pictures too, so its rate is "what a best-effort reader gets", not "what was published". On the Jetson it is off: a second subscriber to 2.7 MB pictures costs the board CPU. (The board itself had a `ros2 topic hz` subscriber running.) | off by default |
| frames received / processed / dropped by the detector | its per-frame log lines, its warnings ("dropping oldest") and its shutdown `Topic summary: received=... dropped=...` | yes |
| frames published but never received | estimated: 2 per second for the time the stream was fresh (no `stream_node` warning), minus the detector's `received`. A diagnostic run with `video_monitor --reliable` gives the exact published count (it adds one reliable reader: recorded as a difference) | estimated |
| latency inside the detector | its own `latency=` field (from its subscription callback to the result: queue wait + processing) | yes |
| camera-to-detector latency | frame matching (11.3) | yes (clock offset corrected) |

**Two effects decide whether the detector gets pictures at all. Watch both from the very first run.**

1. **Root and user.** On the board the detector container runs as root, with `--ipc host`, and the stream node runs
   as the normal user `astra-dev`. The Jetson design VERIFIED on this PC that Fast DDS delivered NOTHING between root
   and a normal user in one container, in either direction, within 10 s (small messages). The board's detector also
   used only 0.4 % of a CPU core over 7.8 minutes (PROBE), and the copy of its results folder taken at 11:51 (eleven
   minutes after it started) had no CSV files. (That copy may have skipped files owned by root.) The likely meaning
   (EXPECTED, not proven): **on the board, the detector receives no pictures at all.**
2. **Big best-effort pictures, even for the same user** (VERIFIED on this PC today, section 17). Two programs of the
   SAME user, ROS 2 Humble's default Fast DDS (shared memory + UDP, as on the board), 1280x720 rgb8 pictures
   (2,764,800 bytes) at 2 Hz, published like `stream_node` (RELIABLE, depth 1) and received like the detector
   (`SensorDataQoS`: best effort, depth 5; a Python subscriber stood in for the C++ detector, same middleware):

   | Test (40 pictures each) | Arrived |
   |---|---|
   | the team's QoS, default transports, run 1 / run 2 | **24 / 11** |
   | the team's QoS, UDP only (no shared memory) | **5** |
   | a RELIABLE depth-5 subscriber, default transports | **40** (median age 16 ms) |
   | the team's best-effort QoS with an 8 MB shared-memory segment (default 0.5 MB) | **40** (median age 6 ms) |

   A picture travels in about 43 pieces; the default segment holds far less than one picture, and with best effort
   one lost piece loses the whole picture. EXPECTED on the board too (same Fast DDS defaults; the loss rate depends
   on CPU and timing, and the C++ detector was not part of this test). So even with `--user`, the team's QoS throws
   away many pictures: team proposal **P9** (subscribe reliably, or give Fast DDS a bigger segment).

**What each configuration shows** (the one rule of section 3): `board` and `video --no-proposals` reproduce both
effects (root, the team's QoS); `video` with proposals runs `--user` (P4) and reliable QoS (P9). The summary always
states received against published and names the likely cause, for example `detector received 0 of ~620 pictures in
312 s (root container + user publisher: the shared-memory trap; team proposal P4)` or `detector received 270 of ~620
pictures (best-effort 2.7 MB pictures are lost in Fast DDS; team proposal P9)`. The first PC test of the real
combination (the team's C++ detector in Docker, root vs `--user`, team QoS vs reliable) is step 3 of 13.3.

### 11.3 Matching a detector frame to the camera frame it came from

The detector's CSV has no time and the frame number restarts with every container, so the twin matches by time and by
picture:

1. **Arrival time** of detector frame N (when its subscription callback ran) = the time of its log line minus its
   logged `latency`. SOURCE `detector_node.cpp` lines 141 and 161-163: `latency = done - arrival` = `queue_wait` +
   `total`, while `total = done - dequeued`. (Using `total` would be off by the queue wait, which can reach seconds
   on the CPU backend.) The log line's time is the ROS console stamp, written right after `done`. If the log has no
   stamp, the CSV file's modification time is used instead (it is written just before `done`). On the Jetson, the
   times are corrected by the measured clock offset, as the Jetson design already does for photos.
2. **Candidates:** the camera frames sent between (arrival - 2.5 s) and (arrival - 0.3 s).
3. **Picture score:** the distance between each candidate's stored 64x36 grey thumbnail and the same thumbnail made
   from `annotated_frameN.png` (the thin boxes barely change it). **One frame alone is often ambiguous:** over uniform
   grass seen from 30 m, neighbouring 30 fps frames differ by only a fraction of a thumbnail pixel (the reviewer
   estimated 0.16 px at 32x18), after H.264, scaling and the box overlays.
4. **So the matcher aligns the whole sequence.** Consecutive detector frames come from ffmpeg's 2 fps selection, so
   they are about 15 camera frames apart (between 8 and 45 when `stream_node`'s timer skipped or doubled one). A
   monotonic alignment (dynamic programming) over all detector frames of the run uses that spacing and the scene's
   drift over 0.5 s (about 2.75 m at 5.5 m/s, a few thumbnail pixels) to pin each frame.
5. **Each row records how sure it is:** the method (`thumb_seq`, or `time` when no PNG exists: arrival minus the
   median delay of the matched frames), the best and second-best distance, the number of candidates within 10 % of
   the best (`match_ambiguity`), and the time-only estimate. No accuracy bound is claimed in advance: the summary
   reports the share of ambiguous rows and the spread between the picture and the time estimates.

This gives every detector frame the drone pose, gimbal angles and zoom of its picture, plus the real camera-to-detector
latency and its spread (6.5). The adapter's `delay` parameter is checked against it.

### 11.4 CSV files for the Runs & logs work (same rules as [LOGS_AND_DATA](LOGS_AND_DATA.md))

They use the same time columns (`wall_time_utc`, `unix_time_s`, `t_since_arm_s`), the unit in each column name, and an
empty cell for unknown values.

- `csv/video_stream.csv`, one row per second: `server_fps, repeated_frames, late_frames, clients, sent_kbps,
  topic_hz` (empty without the monitor), `detector_received, detector_processed, detector_queue_drops,
  stream_warnings, ffmpeg_restarts, mission_phase`.
- `csv/video_detector_frames.csv`, one row per frame the detector processed: `detector_frame, detections, detect_ms,
  queue_wait_ms, total_ms, latency_ms, queue_depth, backend, arrival_unix_time_s, stream_frame, match_method,
  match_best, match_second, match_ambiguity, stream_frame_by_time, camera_to_detector_s, lat_deg,
  lon_deg, height_above_home_m, heading_deg, gimbal_pitch_deg, gimbal_yaw_deg, zoom_x, hfov_deg, targets_in_view,
  mission_phase`.
- `csv/video_detections.csv`, one row per box: `detector_frame, stream_frame, class_id` (COCO), `class_name,
  confidence, box_x1_px, box_y1_px, box_x2_px, box_y2_px, box_cx_px, box_cy_px, image_w_px, image_h_px,
  detection_time_ms, near_target` (`hiker`, `tent` or empty), `target_distance_px, false_alarm` (1, 0, or empty),
  `sent_to_mission` (1 when the adapter sent it).

### 11.5 Run summary, run index and `demo.env`

- `run_summary.md` gets a **"Video pipeline"** section, and `run_summary.json` a `video` key (schema
  `astra.video_summary.v1`): the configuration and whether proposals ran (labelled); the sources (path, commit,
  checksum) and the detector image, model and **backend** (with "not TensorRT" as a difference when that is true);
  the stream (codec, size, fps, bitrate, GOP, encoder, "settings documented, not measured on the real camera"),
  frames sent, repeated, longest gap, viewers; `stream_node` restarts and warnings; the detector's frames received,
  processed and dropped, and **received against published (estimated) with the likely cause** (11.2); latency
  (median and 95 %), first frame time; the camera-to-detector latency and its spread; the share of ambiguous frame
  matches; the detector's settings (image, user, QoS) and which of them differ from the board; boxes by class;
  **the mission link** in one sentence (for example `main_controller received 0 detections: the team's code has no
  link yet (configuration video, proposals off)`); and the video items of the "differences from the field" list
  (from `video/differences.txt`).
- `runs_index.csv` gets the columns `pipeline`, `proposals`, `detector_backend`, `video_frames_processed`,
  `person_boxes`, `hiker_recall`.
- `demo.env` gets `VIDEO_PIPELINE`, `VIDEO_PROPOSALS`, `VIDEO_ROOT`, `VIDEO_DETECTOR_BACKEND`, `VIDEO_STREAM`
  (for example `h264 1920x1080@30 2000kbps gop30 x264`).

### 11.6 Scoring and phases

- **New per-frame scores** (from `video_detector_frames.csv`, the truth file and the matched poses): a target is "in
  view" when its truth position, projected with the frame's pose, gimbal angles and field of view, falls inside the
  picture. The scores are: hiker detected (a `person` box within max(box size, 3 % of the picture width) of the
  projected point) over hiker-in-view frames; boxes elsewhere = false alarms per minute of search; the time of the
  first hiker detection; the tent shown for information only (the stock model has no tent class); the
  camera-to-detector latency; frames lost.
- **Mission scores** stay as they are (targets, aim error from the `Aim ...` texts, drop error). With proposals on they
  work unchanged, because the adapter publishes `/image_detection`.
- **Phases:** `Mission: N DigiCamCtrl` (from ArduPilot) still marks the start of the search. `Camera trigger STARTED`
  and `Captured ... image #N` never appear with the new pipeline. Hold, targets, drops and RTL come from
  `main_controller`'s texts as today. The demo window's photo counter becomes "frames to the detector / detections".
- **Mission Planner** (the ASTRA plugin parses team texts): with the new pipeline there are no camera texts, so it
  shows no camera activity. That is the field truth, and this design asks for no plugin change.

## 12. (g) What still differs from the field

The twin runs the team's code unchanged. These things still differ. Each run writes the ones that apply to
`video/differences.txt` (with the configuration's own items: for example 11 only with proposals on, 15 only in
`board`).

1. **The stream's exact settings were not measured on the real camera** (owner, 12:25). The twin uses the manual's
   documented choices: H.264, 1920x1080, 30 fps, 2000 kbit/s, GOP 1 s. The real codec, size, bitrate, GOP and H.264
   profile may differ; the manual's own main-stream examples use **H.265** (6.2). That changes the Jetson's decode
   work (H.265 costs its CPU much more), the link's bandwidth and the compression artefacts.
2. **The pictures are rendered by Gazebo,** not taken through a real lens: no real exposure, motion blur, haze, sun or
   lens distortion; simple target models.
3. **The camera's own delay** (picture to network) is unknown. The twin's is measured (tens of ms); no extra delay is
   added unless `extra_delay_ms` is set.
4. **The zoom speed and the gimbal's degrees per second** are guesses (4x per second; speed 100 = 90 deg/s).
5. **The power-on pose** is assumed to be forward (EXPECTED for SIYI); the field's pointing procedure is unknown (T2).
6. **Focus** is always sharp; the real lens may be soft after a zoom.
7. **Checksum handling:** the twin drops the team's bad-checksum packets as it expects the real camera to (not
   verified on hardware). The gimbal STOP is hit 1.94 % of the time (7).
8. **Recording** (0x0C function 2) only changes a flag; no MP4 is written. The real camera pads its JPEGs with zeros.
9. **`launcher.sh`:** the twin, the virtual Jetson and HIL run THIS repo's `launcher.sh`, which honours
   `TRACKER_CMD` (line 78). The board's (`6beb5f1`, line 73) ignores it and always starts `tracker.launch.xml`.
   Otherwise the two files are identical. So on the board the proposed launch file cannot start until the team
   applies P7 (8.2).
10. **The launch:** the proposed field launch file has not been adopted by the team. On the board today the pieces are
    started by hand, and the boot launcher still starts the obsolete photo nodes (that is the `board` configuration).
11. **The proposals** are ours, not the team's: the gimbal pointing, the detection topic, the adapter (with its own copy
    of `target_geolocation.py`), the GPU detector patches, the detector as the normal user (P4) and its reliable
    subscription (P9). With them on, the drops depend on code the team has not adopted, and the detector gets
    pictures that the field (today) would lose (items 19, 20).
12. **The detector backend:** TensorRT is the field target. On this PC it runs on an RTX 4060 (another GPU, another
    engine), or ONNX Runtime CUDA, or the CPU. The summary says which one ran. The board today runs the CPU image.
13. **The model file:** one ONNX exported on the PC with a pinned Ultralytics version. The board's image exported its
    own at build time with whatever Ultralytics version was current (not pinned).
14. **The team code version:** the twin runs the copies the user points at. The board runs `zr10_ros2` from
    `~/astra_ws`, the detector built from an uncommitted local tree (2.1), and the main stack at `6beb5f1` (local
    changes unknown). The simulator's own team copy is newer (the October 2026 fixes, among them `processing_hold.py`
    and `target_geolocation.py`) unless `--team-repo` points at another checkout; a `6beb5f1` checkout needs P7 for
    the video configuration (9).
15. **The `board` configuration's `new_od`:** natively in WSL, `team_stack.launch.py` sets `model_path` (the PC's
    `yolo26m.pt`) and `use_batched_inference` false, so `new_od` runs the `.pt` SAHI path on PyTorch, not the board's
    TensorRT engine with batched inference. (The virtual Jetson's TensorRT image runs the unchanged
    `tracker.launch.xml` with an engine built for this PC's GPU.)
16. **CPU, GPU and architecture:** this PC's i9 (amd64) and RTX 4060 against the Orin (arm64). The stream node's slow
    line costs 0.2 s here and about 0.5 s on the board. The detector's speed, queueing and drops differ, and so may
    the share of pictures Fast DDS loses (item 20). Only the real Jetson gives the field timing.
17. **"Camera off"** looks different: here the connection is refused at once (about a 1 s retry cycle). At the field,
    EXPECTED: an unpowered camera never answers ARP, so ffmpeg's connect fails after about 3 s (about a 4 s cycle);
    if a connection opens but no full picture arrives, `stream_node`'s own 10 s frame deadline ends ffmpeg (about an
    11 s cycle).
18. **The network path:** on this PC, a loopback address (no Ethernet). On the real Jetson, the SSH relay over USB,
    Ethernet or Wi-Fi instead of the camera's own Ethernet cable.
19. **Users and DDS:** one rule (section 3). Proposals off and `board`: the detector runs as root, as on the board.
    Proposals on: as the normal user (P4), which the board does not do.
20. **Lost pictures:** with the team's QoS, many 2.7 MB pictures never reach the detector, even between programs of
    one user (VERIFIED on this PC: 24 and 11 of 40). Proposals off and `board` reproduce that; with proposals on the
    detector subscribes reliably (P9), so it gets pictures the field (today) would lose. The share lost depends on the
    machine, so the twin's number is not the board's.
21. **ROS domain:** native runs use domain 0, like the board. The virtual and real Jetson use domain 77 with
    `ROS_LOCALHOST_ONLY=1` (the Jetson design), so the team's `-e ROS_DOMAIN_ID=0` becomes 77 there.
22. **Folders and flags:** the detector's `/data` is the run folder, not the team's `objectDetection` folder; `-it`
    becomes no TTY at all, and the detector is stopped with `docker kill --signal=INT` (8); our container gets a name
    and a label; root-owned result files are given back to the WSL user after the run; builds use copy-install
    instead of `--symlink-install`.
23. **Extra listeners:** on this PC the optional `video_monitor` subscribes to `/zr10/image_raw` (best effort, like the
    board's `ros2 topic hz`; `--reliable` only in diagnostic runs). `pov_server` reads the emulator's 640x360 preview,
    not the camera stream.
24. **The photo folder:** the proposed launch file creates `mapping_photos` when it is missing (on the board today the
    boot launcher's `siyi` and `new_od` create it), and the twin archives old photos before each run. Without the
    launch file's step, a fresh flight Jetson running the board's `main_controller` would hover at the end of the
    survey until a failsafe (10.2); this repo's `main_controller` would wait 90 s. Old photos left in the folder cost
    a 90 s wait in both.
25. **Frame pacing:** when the simulation runs slower than real time, the stream repeats frames to stay at 30 fps
    (counted). The scene itself then moves in simulated time.
26. **The lens and the missions:** the twin now uses the ZR10 lens (61.5 degrees), but the survey missions were planned
    for 81 degrees until they are re-planned.
27. **ffmpeg version:** the twin uses Ubuntu 22.04's ffmpeg 4.4.2. The board's version was not probed.
28. **Only `/main.264` is served;** a sub-stream or other paths are not.
29. **The `board` configuration** has two camera controllers (`siyi` and `zr10_node`) on one camera, as on the board
    today.
30. **Aim timing:** the adapter subtracts a fixed delay from `stream_node`'s stamp; about ±0.25 s (±1.4 m at 5.5 m/s)
    remains because the stamp is the timer tick, not the picture's arrival (6.5, P10).
31. **The detector's source:** the board's image was built from an uncommitted local tree (2.1). The twin builds from
    the folder the user points to (today the 11:51 snapshot of that tree); once the team commits and pushes (D4), the
    run records a commit instead of "dirty".

## 13. (h) The build plan

### 13.1 Who builds what

**EMULATOR side (engineer A): the stream and the ZR10 behaviour.**

| File | Change |
|---|---|
| `ros2_ws/src/simulation/siyi_emulator/siyi_emulator/rtsp_stream.py` (new) | the gst-rtsp-server (shared media, `appsrc do-timestamp=true` as on the bench, a fresh `appsrc` per `media-configure`), the gz-transport frame source, the pacing loop, the digital zoom crop, the frame records and 64x36 thumbnails, the H.264 and H.265 pipelines, the 640x360 preview topic for `pov_server` (C12) |
| `siyi_emulator/stream_settings.py` (new, pure Python) | settings and checks, the GStreamer pipeline text, the zoom-to-field-of-view math, the CSV rows (unit-tested without GStreamer) |
| `siyi_emulator/emulator.py`, `node.py`, `camera_state.py`, `command_handler.py` | start and stop the stream with the emulator; share the zoom and gimbal state; the 1x zoom command at start (6.3); ZR10 answers to 0x04/0x06/0x16/0x18/0x19; the bad-checksum counter; the new `emulator_state` fields; the state file `video_stream.env`; optional 0x20/0x21 |
| `ros2_ws/src/simulation/sim_models/models/iris_a8mini/model.sdf.xacro` (+ the pre-generated `model.sdf`) | the ZR10 lens on both cameras; the video size and rate as arguments; `CameraZoomPlugin` on the video camera with **`<max_zoom>20</max_zoom>`** (6.3) |
| `ros2_ws/src/simulation/sim_bringup/sim_bringup/sim_paths.py`, `launch/sim.launch.py` | launch arguments `rtsp`, `stream_port`, `stream_path`, `stream_codec`, `stream_width`, `stream_height`, `stream_fps`, `stream_kbps`, `stream_gop`, `stream_encoder`, `stream_render_width`, `stream_render_height`, `stream_extra_delay_ms`, `lens`, `gimbal_start`, `video_record_dir`; with `rtsp:=true`, `pov_server`'s `topic` becomes `/sim/camera/preview` (C12); defaults keep today's behaviour |
| tests | `siyi_emulator/test/test_stream_settings.py` (pure); `test_rtsp_stream.py` (a synthetic source; skipped without GStreamer); an end-to-end script that runs the team's exact ffmpeg command in a throwaway `ubuntu:22.04` container against the stream |
| docs | `siyi_emulator` README ("The RTSP video stream"), the `sim_models` and `sim_bringup` READMEs, AGENTS.md sections 4.2, 4.3 and 4.4 (port 8554), TROUBLESHOOTING rows |

**PIPELINE side (engineer B): starting the team's nodes unchanged, the proposals, the logging, the docs.**

| Part | Files |
|---|---|
| B1: run the team code | `simulation/video/video.sh` (new front door: `check`, `build`, `start`, `stop`, `status`, `collect`, `logs`; options `--pipeline`, `--video-root`, `--zr10-ros2`, `--zr10-stream`, `--detector-dir`, `--detector-backend`, `--detector-user`, `--detector-qos`, `--model`, `--run-dir`), `simulation/video/lib/video_overlay.sh` (three workspaces), the board-configuration start and stop, the exact colcon command of 9, the `launcher.sh` check of 8.1 (`${TRACKER_CMD:-`), the git `dirty` records, the chown of root-owned results |
| B2: the proposals | `simulation/video/proposals/README.md`, `field_video.launch.xml`, `astra_video_proposals/` (the `gimbal_pointing` and `detection_adapter` nodes, `prepare_photo_folder`, `run_detector.sh`, `geolocation.py` = the labelled copy of `target_geolocation.py` with its drift test, pytest tests of the pure parts), `ADAPTER.md` (the design of 10.3 and the questions T1-T5, T9, T10), `TEAM_PATCHES.md` (P1-P10 of 15.3, the field-safety items P7, P8, P9, P1 first) |
| B3: the GPU detector | `simulation/video/proposals/detector_gpu/` (the patch files incl. `image_qos`, `Dockerfile.gpu`, README, the `ort_cuda` wrapper with its comparison test against the team's C++ output), the ONNX export with a pinned Ultralytics version, the engine cache; built together with the Jetson work (owner 12:32: one Dockerfile, two base layers) |
| B4: logging and scoring helpers | a new package `ros2_ws/src/simulation/sim_video/` (ament_python, maintainer astra-sim@example.com): `video_monitor`, `video_export` (the CSV files and `video_summary.json`), pure modules `detector_log.py`, `frame_match.py`, `video_sources.py`, `video_summary.py`, with tests |
| B5: docs | this page's "How to run" (13.6) made real, `docs/simulation/COMMANDS.md`, `ERRORS.md` (every `[video]` message), `LOGS_AND_DATA.md` (11.1-11.5), `SWAPPING.md` (pointing the twin at team video code), `TROUBLESHOOTING.md`, AGENTS.md, and the `docs/README.md` index (the new READMEs) |

If a third engineer is free, B3 is the part to give them. It depends only on the contract C9.

### 13.2 The contract between the two sides

| # | What | Exact form |
|---|---|---|
| C1 | the RTSP endpoint | `rtsp://<camera_bind>:<stream_port>/<stream_path>` = `rtsp://192.168.144.25:8554/main.264`; TCP interleaved (UDP allowed too); SPS/PPS before every keyframe; the first frame within 2 s of PLAY; a steady `stream_fps` by the wall clock; at most `max_clients` viewers; 404 for other paths |
| C2 | the settings really used | `$ASTRA_SIM_HOME/state/video_stream.env`: `URL, BIND, CODEC, WIDTH, HEIGHT, FPS, BITRATE_KBPS, GOP, ENCODER, PROFILE, RENDER_WIDTH, RENDER_HEIGHT, EXTRA_DELAY_MS, MAX_CLIENTS, LENS, HFOV_1X_DEG, OPTICAL_MAX, MAX_ZOOM, MEASURED=0, RECORD_DIR, STARTED, PID` (written at start, removed at stop) |
| C3 | the emulator's records | in `video_record_dir` (demo.sh passes `<run>/video`; without it: `~/.astra_sim/video/<start time>/`): `stream_frames.csv` (columns of 11.1, in this order; rows only while a viewer plays), `stream_thumbs.u8` (2,304 bytes = 64x36 grey per row, same order), `rtsp_sessions.csv`, `stream_server.json`, `sdk_bad_crc.log` |
| C4 | `/sim/camera/emulator_state` | adds `stream: {clients, frames, repeated, late, kbps}`, `hfov_deg`, `zoom_x`, `sdk_bad_crc` |
| C5 | camera behaviour | answers of section 7; `initial_pitch_deg` from `gimbal_start` (`forward` = 0, `down` = -90); max zoom = the model's own (30) unless `camera_max_zoom` is set; the 1x zoom command sent at start; the xacro's `CameraZoomPlugin` `<max_zoom>` 20 (it must be at least 16.9, 6.3) |
| C6 | launch arguments | the names in 13.1 (`sim.launch.py --show-args` lists them); B's demo.sh change passes them per configuration |
| C7 | the boundary | the pipeline side talks to the simulated camera ONLY through the camera protocol (RTSP, SDK, HTTP), like the field. It reads C2, C3 and C4 only for logging and scoring, outside the team code |
| C8 | the proposed launch file | its arguments in 8.2 (incl. `detector_user`, `image_qos`, `photo_dir`; the values per configuration come from C11); node names `zr10_camera`, `zr10_stream`, `detector_node`, `gimbal_pointing`, `detection_adapter` (and the team's three); the `prepare_photo_folder` step runs first in every configuration |
| C9 | the detector | images `yolo26-ros-subscriber` (team, unchanged) and `astra-video-detector:pc\|jetson` (patched), label `org.astra.owner=video` (HIL: `org.astra.owner=hil`); container name `astra-video-detector` (HIL: `jetson.sh`'s `${JETSON_CONTAINER}-detector` = `astra-hil-detector` on the board, `astra-hil-virtual-detector` on this PC); no `-t`; stopped with `docker kill --signal=INT`, 10 s, `docker stop -t 5`; start-up line `backend=<tensorrt\|ort_cuda\|opencv_cpu> device=<name> engine=<sha256\|-> precision=<fp16\|fp32\|-> model_sha256=<sha256> image_qos=<sensor_data\|reliable>` (patched image only); "ready" line = the team's existing `Ready: subscribing <topic>` (`detector_node.cpp`, in the uncommitted tree); topic `/yolo26/detections` |
| C10 | the run folder | the files of 11.1 and the CSV columns of 11.4 (the Runs & logs work reads these names) |
| C11 | the detector settings per configuration | the table in section 3: image, user, backend and GPU flags, image QoS. `video.sh`, the launch file's arguments, `jetson.sh` (`JETSON_DETECTOR_IMAGE`, `JETSON_DETECTOR_CMD`, its `-u` and GPU flags) and every doc follow it; an override is recorded as a difference |
| C12 | the preview for `pov_server` | with `rtsp:=true` the emulator publishes ROS `/sim/camera/preview` (`sensor_msgs/Image` rgb8, 640x360, up to 10 Hz, BEST_EFFORT depth 1) from the frames it already has; `sim.launch.py` sets `pov_server`'s `topic` parameter to it; a gz copy of the same topic exists only if A5 needs it for the Gazebo panel |

### 13.3 Order: what can start at once

Every simulator run and Docker build below holds both locks for its whole duration and waits while the owner's demo
flag file exists (owner 12:40).

1. **Both, day 1:** agree on C1-C12 (this page).
2. **A** builds the stream with synthetic frames first, and tests it with the team's exact ffmpeg command in a
   throwaway container (no Gazebo needed): 5 minutes (A1), reconnects (A1b), H.264 and H.265. Then the Gazebo
   source, the lens, the zoom (with `<max_zoom>` 20), the pacing, the records and the `pov_server` preview. Then one
   visible simulator run with the team's ffmpeg command pulling the stream, which also measures the real-time factor
   at 1080p30 with the Gazebo camera panel and `pov_server` running (A5).
3. **B** starts at the same time against a stand-in RTSP source on `192.168.144.25:8554/main.264` (a GStreamer test
   pattern, like the studies' bench tool). B1: check and build the team code without touching it. Run `zr10_node`,
   `stream_node` and the team's detector image (built from their Dockerfile: a heavy job, so both locks), natively
   and in a container. **Early question to answer here, as a 2x2 test of 60 s each:** the team's C++ detector in
   Docker as root and with `--user`, each with the team's QoS and with the patched reliable QoS; count received
   against published (`video_monitor --reliable` for the exact count). This is the PC evidence for the board's idle
   detector and for P4 and P9 (11.2); the result goes into this page's evidence. Then B2, B3 and B4.
4. **Patches to other streams' files (13.5).** B writes them as exact patches in its report; **the lead applies
   them** (or hands them to the files' owners) BEFORE step 5, because B cannot fly B4/B5 through `demo.sh` without
   them. Until then B flies with the manual path of 13.6 (`start_sim.sh` with `name:=value` launch settings, the
   mission upload, `TRACKER_CMD` typed by hand), which needs no change to anyone's file.
5. **Acceptance (owner 15:14): J1 first.** A full visible flight with the VIRTUAL Jetson running the `video`
   configuration (with the Jetson stream: its image carries `zr10_ros2`, `zr10_stream` and the proposal package; the
   detector container joins `astra-hil-virtual`). Then J2 (`--no-proposals` in the virtual Jetson), and the native
   flights `video`, `video --no-proposals` and `board`, all visible. Compare them, then write the docs.
6. **Documented only, not run** (owner 15:14: no real Jetson for now): the real-board steps (the arm64 image moved
   onto the board, the on-board TensorRT engine warmup for yolo26, the first HIL flight), with exact commands and
   expected outputs in the Jetson design (section 14 lists what it needs from here).

The owner's rule on making `video` the default: switch the default (demo.sh, profiles, the docs) only after step 5
passes. Until then the default stays as today.

### 13.4 Acceptance checks (how you know it worked)

| # | Check | It worked when |
|---|---|---|
| A1 | the team's exact ffmpeg command against the twin's stream, with `codec` `h264` and again with `h265` | 2.0 +- 0.1 frames per second for **5 minutes** (longer than gst-rtsp-server's 60 s session timeout, VERIFIED default), 0 bad frames, the exit code 0 after it is stopped |
| A1b | stop that ffmpeg and start it again, three times | each time the first picture arrives within 2 s (the media is prepared again with a fresh `appsrc`) |
| A2 | `ffprobe -rtsp_transport tcp rtsp://192.168.144.25:8554/main.264`, for both codecs | it prints `h264` (or `hevc`), `1920x1080`, `30 fps`, and keyframes 1 s apart (with `-show_frames`) |
| A3 | zoom 0x05 in, then stop | `hfov_deg` in `emulator_state` falls from 61.5 to 6.8 (+- 0.1) degrees at 10x; the picture is visibly zoomed; `sim.log` has no CameraZoomPlugin `has been clamped` warning; past 10x `digital_crop` is above 1 |
| A4 | gimbal 0x07 and 0x0E | the video turns; the angles in `stream_frames.csv` follow |
| A5 | real-time factor with the 1080p30 video camera | 0.7 or more in a visible runway run WITH the Gazebo camera panel and `pov_server` (the Mission Planner HUD) running, as in a normal demo (otherwise the 6.8 steps, then the D9 fallback) |
| A6 | four viewers, then a fifth | four get pictures; the fifth is refused |
| A7 | a packet with a bad checksum | ignored, counted in `sdk_bad_crc`, written to `sdk_bad_crc.log` |
| A8 | stop and start the simulator while `stream_node` runs | `stream_node` warns, retries, and publishes again within 15 s of the restart |
| B1 | `video.sh check --video-root <snapshot>` | `[video] OK` with the three parts listed and the detector shown as `dirty: yes`; pointed at the `video-cam` CMakeLists it stops with the line-17 error; with `--team-repo` at a `6beb5f1` checkout it stops with the `launcher.sh ignores TRACKER_CMD` error |
| B2 | `video.sh build` | `zr10_node` and `stream_node.py` are in `~/.astra_sim/video/ws/install`; no `log/` folder appeared in the current folder; the team folders' checksums are unchanged |
| B3 | `board` configuration | the same processes and arguments as the probe (compare `commands.txt` with 2.1; the listed differences only); `/zr10/image_raw` at about 2 Hz; the detector's result (pictures received against published, or none) written in the summary in plain words; after `video.sh collect` the WSL user can delete the run folder |
| B4 | `video` with proposals, runway | `Detected person` and/or `Detected tent`, `Aim ...`, drops, RTL, landed; the summary labels the proposals and names the backend, the user and the QoS |
| B5 | `video --no-proposals` | survey, `No valid targets found. Returning to launch.`, RTL, no drops; the summary says "mission link: not connected" and how many pictures the detector received, with the likely cause |
| B6 | backend `auto` on this PC | `detector_backend.json` says `tensorrt` (or says why not and which backend ran instead) |
| B7 | the CSV files and the summary | the three CSV files have rows; `run_summary.md` has the "Video pipeline" section; `check_docs.py` passes |
| B8 | the photo-folder step | with `mapping_photos` removed before the start, the launch file creates it and the hold ends at once with `All 0 images processed` (with this repo's team copy, and with a `6beb5f1` + P7 copy via `--team-repo`); with one old photo in it, the `[photo_folder] WARNING` appears and the hold lasts 90 s |
| B9 | stopping | after the launch is stopped (Ctrl+C), the detector printed its `Summary:` and `Topic summary:` lines, and no `astra-video-detector` container is left (`docker ps -a`) |
| **J1** | **virtual Jetson, `video` configuration, proposals on (THE acceptance test, owner 15:14)** | a full visible flight: survey, detections, `Aim ...`, drops, RTL, landed; the detector container `astra-hil-virtual-detector` joined to `astra-hil-virtual` with the stack's user, TensorRT and reliable QoS; the run summary has the "Video pipeline" section and lists the differences, `launcher.sh` (item 9) among them |
| J2 | virtual Jetson, `--no-proposals` | survey, RTL, no drops; the detector runs as root with no GPU flags; the summary states pictures received (EXPECTED none) and the cause |

### 13.5 Changes needed in files other work owns (to send as exact patches, not edits)

B writes each one as an exact patch in its report. **The lead applies them** (or hands them to the files' owners)
before step 5 of 13.3; B flies with the manual path of 13.6 until then.

- `simulation/scripts/demo.sh`: options `--pipeline video|board|legacy`, `--no-proposals`, `--video-root DIR`,
  `--detector-backend B`, `--detector-user board|match`, `--detector-qos sensor_data|reliable`, `--stream WxH@FPS`,
  `--stream-codec`, `--stream-kbps`; it passes the launch arguments (C6), builds `TRACKER_CMD` with absolute paths and
  refuses paths with spaces (8.1), checks the team checkout's `launcher.sh` for `${TRACKER_CMD:-` (8.1), passes the
  detector arguments of C11, uses the node lists for `stack_check`, calls `video.sh start` (`board`) and `video.sh
  collect`, and writes the `demo.env` keys (11.5).
- `simulation/scripts/astra.sh`: a `video` subcommand that calls `simulation/video/video.sh`.
- `ros2_ws/src/simulation/sim_plugins/` (schema, profiles, plan, `team_repo.py`): a new `video:` profile section
  (`pipeline, proposals, root, zr10_ros2, zr10_stream, detector_dir, detector_image, detector_backend, detector_user,
  detector_qos, model, stream_fps, monitor`) and `camera.stream.*`, `camera.lens`, `camera.gimbal_start`; the
  team-stack node list per configuration. Shipped profiles `video.yaml`, `video_board.yaml`, `legacy_photo.yaml`
  follow once the schema takes them. **`team_repo.py` line 100:** it tests `if var not in ltext` for `TRACKER_CMD`,
  which the board's old `launcher.sh` passes (it assigns `TRACKER_CMD` itself). Test for the override instead:

  ```python
  # team_repo.py, the launcher check: a launcher honours TRACKER_CMD only with the ${TRACKER_CMD:-...} form
  for var, why in (('${TRACKER_CMD:-', 'without it the launcher always starts tracker.launch.xml ...'),
                   ('${FCU_URL:-', 'without it MAVROS cannot be pointed at the simulated autopilot')):
  ```

  (the error text then names `TRACKER_CMD`/`FCU_URL` as today; the board's `6beb5f1` launcher has `${FCU_URL:-`, so
  only the `TRACKER_CMD` line fails for it).
- `simulation/jetson/jetson.sh` and the Jetson design: section 14 (C11 for the detector container).
- `sim_demo` (the Runs & logs work): `run_summary` reads `video/video_summary.json` and `video/differences.txt`;
  `csv_export` calls `sim_video video_export`; `score_run` adds the per-frame scores (11.6).
- `simulation/setup/01_install_humble.sh`: add `ffmpeg nlohmann-json3-dev` to its apt list (the owner runs it once).
- `docker/Dockerfile` (`dev` target): add `ffmpeg` (`zr10_stream` in the virtual Jetson).
- The Jetson design and kit: section 14.

### 13.6 How to run it once it is built (PLANNED: none of these commands exist yet)

```bash
# 1. Once: point the twin at the team's video code, check it and build it (never writes into that folder)
bash simulation/video/video.sh check --video-root ~/team_video
bash simulation/video/video.sh build --video-root ~/team_video
# 2. Fly the field configuration (proposals on, so the demo completes)
bash simulation/scripts/demo.sh runway --pipeline video --video-root ~/team_video
# 3. The team's code exactly as it is today (no proposals: expect RTL with no drops)
bash simulation/scripts/demo.sh runway --pipeline video --no-proposals --video-root ~/team_video
# 4. Both pipelines side by side, exactly as on the board today
bash simulation/scripts/demo.sh runway --pipeline board --video-root ~/team_video
```

How you will know it worked: the demo window prints `[video] stream rtsp://192.168.144.25:8554/main.264: h264
1920x1080@30 (documented settings, not measured)`, then `[video] /zr10/image_raw: 2.0 Hz`, then `[video] detector
backend: tensorrt (user: you, QoS: reliable, PROPOSAL)`. At the end, `run_summary.md` has a "Video pipeline" section
and `csv/video_detections.csv` has rows.

**13.6b The manual path (works before the `demo.sh` patches are applied; PLANNED, needs B1-B2 built):**

```bash
# Terminal 1: the simulator with the stream on (start_sim.sh already passes name:=value settings to sim.launch.py)
bash simulation/scripts/start_sim.sh runway rtsp:=true lens:=zr10 gimbal_start:=forward
# Terminal 2: upload the mission as demo.sh would, then the team stack through THIS repo's launcher.sh
bash simulation/scripts/upload_mission.sh            # the runway mission
R=$HOME/ODLC_Machine_Inferencing_System_2025-2026; V=$HOME/.astra_sim/video/ws; RUN=$HOME/.astra_sim/runs/manual_$(date +%H%M%S)
mkdir -p "$RUN/video/yolo26" && cd "$R" && FCU_URL=tcp://127.0.0.1:5763 TRACKER_CMD="bash $R/simulation/video/lib/video_overlay.sh \
  --sim-ws $R/ros2_ws --team-ws $R/ros2_ws --video-ws $V -- ros2 launch $R/simulation/video/proposals/field_video.launch.xml \
  detector:=docker detector_image:=astra-video-detector:pc detector_data:=$RUN/video/yolo26 detector_user:=match \
  image_qos:=reliable proposals:=true" ./launcher.sh sim
```

How you know it worked: terminal 2 shows `[zr10_stream]` without `waiting for stream` after a few seconds, the
detector's `Ready: subscribing /zr10/image_raw`, then `[frame1.png] detections=...` lines; the flight is started from
Mission Planner or `fly_mission.sh` as usual.

## 14. For the Jetson design (the exact changes `docs/jetson/HIL_DESIGN.md` needs)

These follow from the studies, the review, and the owner's entries of 12:25, 12:32 and 15:14. The Jetson stream owns
that design and its files (`simulation/jetson/**`, `docs/jetson/**`). Below is what to change, section by section.
Since 15:14 the virtual Jetson is the acceptance test, so items 2, 9, 11, 14 and 15 are needed for J1; the board
steps (3, 5) stay documented and are not run.

1. **Section 6.3, the detector row.** Replace `astra_live_detection live_node` and the `detection/results`,
   `detection/annotated` and `detection/info` topics. The board runs the C++ `yolo26_ros detector_node` in the image
   `yolo26-ros-subscriber`. It publishes **no topics**: it writes `/data/camera_results/{csv,annotated_images}` and
   logs one line per frame. It runs on the CPU only, as root, with `--rm -it --network host --ipc host -e
   ROS_DOMAIN_ID=0 -v ~/astra_camera_detection/astra_ws_d/objectDetection:/data` (PROBE). `astra_live_detection` is
   in the snapshot but does not run. The image was built from an uncommitted local tree (2.1).
2. **The detector in HIL (sections 6.3, 8.3, 8.6, risk 23): follow C11, the one rule of section 3.** Today
   `jetson.sh`'s `run_cmd_detector` (line 950) always adds `-u $(id -u):$(id -g)` and the GPU flag, and HIL_DESIGN
   8.6 / risk 23 say "always the stack's user". Change it to:
   - **proposals on (the field target, the default):** `JETSON_DETECTOR_IMAGE=astra-video-detector:jetson` (L4T r36.4
     base, arm64, built off the board like the team-stack image; `astra-video-detector:pc` on the virtual Jetson),
     `--runtime nvidia` (board) / `--gpus all` (virtual), `--network container:<c> --ipc container:<c>`,
     `-u $ASTRA_UID:$ASTRA_GID` (the stack's user, P4), `-e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1`, no `-t` or
     `-it`, `-v D/runs/<id>/yolo26:/data`, `-v D/models:/models` (the engine cache), the shared `yolo26.onnx` mounted
     read-only at `/opt/model/yolo26.onnx`, and `JETSON_DETECTOR_CMD="ros2 run yolo26_ros detector_node --ros-args
     -p input_mode:=topic -p image_topic:=/zr10/image_raw -p model_path:=/opt/model/yolo26.onnx -p
     output_dir:=camera_results -p backend:=tensorrt -p detections_topic:=/yolo26/detections -p image_qos:=reliable"`.
   - **proposals off (the team's code as it is):** the team's own `yolo26-ros-subscriber` image as it is on the board
     (recorded by image ID), **as root (no `-u`), no GPU flags**, the same network, IPC, domain and `/data` flags, and
     the team's command word for word (the first four `-p` above only). This reproduces the board's root/user trap on
     purpose (EXPECTED: no pictures arrive; the run summary says so). Risk 23's fallback (a Fast DDS profile without
     shared memory) is NOT applied then, because the board has none.
   - Stop: `docker kill --signal=INT <c>-detector`, wait up to 10 s for its summary lines, then `docker stop -t 5`.
     Its log (`docker logs -f`) goes to `RUN_DIR/video/detector.log`. With proposals off, give the root-owned files in
     `D/runs/<id>/yolo26/` back to the Jetson user afterwards (`docker run --rm --label org.astra.owner=hil -v
     ...:/data --entrypoint chown <image> -R UID:GID /data`).
3. **Warmup (section 9.5).** The yolo26 TensorRT engine is built on the board in the same owner-approved `warmup`
   step as the legacy engine: once, with the team idle, from the shared ONNX, into `D/models/yolo26/` with a `.meta`
   (TensorRT version, L4T release, GPU, ONNX SHA-256, precision). **`start` refuses a missing or mismatched yolo26
   engine (exit 19) only when the detector backend is `tensorrt` (proposals on).** With proposals off the team's CPU
   image needs no engine, so the row is SKIP. On the virtual Jetson, `warmup --link virtual` builds this PC's engine
   the same way. This replaces the open point "whether the yolo26 detector needs a first-run step".
4. **The tap (section 7, `simulation/jetson/pc/tap_topics.txt`).** In `[new]`, remove `/detection/results` and
   `/detection/info` (no program publishes them). Add `/yolo26/detections` (`vision_msgs/msg/Detection2DArray`) and,
   with proposals on, `/image_detection` (`interfaces/msg/ImageResult`, from the adapter), so `score_run` and
   `phase_monitor` work as they do today. In `[nodes]`, remove `astra_live_detection`; add `gimbal_pointing` and
   `detection_adapter`. Keep `zr10_camera`, `zr10_stream` and `detector_node`.
5. **Detector files to the PC.** `collect.sh --follow` copies `D/runs/<id>/yolo26/camera_results/csv/` during the
   flight (tiny files, every 3 s) and `annotated_images/` only after the landing (EXPECTED 1-3 MB per PNG at 2 per
   second, which is too much to copy live over Wi-Fi). File times are corrected by the clock offset, as for photos
   (section 11), because the frame matching of 11.3 uses them; the detector's log lines (with their console stamps)
   are copied too, because 11.3 prefers them.
6. **Bandwidth (section 6, "Bandwidth and latency").** The relay carries the camera's MAIN stream: 1920x1080, 30 fps,
   2000 kbit/s (twin default; about 2 Mbit/s plus RTSP framing; the same with `h265`). It is not 1280x720 at 2 fps:
   the scaling and the 2 fps selection happen inside ffmpeg on the Jetson, which is the field's decode work.
7. **Head-of-line delay (sections 4.1 and 15.3 g).** In key mode, put `cam_rtsp.sock` on its own connection (`cm-C`),
   so video bytes can never sit in front of `zr10_node`'s small SDK stop frames on `cm-B`. In login mode everything
   shares one TCP connection anyway. So `hil_relay.py` measures the SDK round-trip time (its PING frames) while the
   video flows, writes `SDK_RTT_MAX_MS` to `relay.status`, and the run summary prints it. Add an acceptance check:
   with the stream running, a stop frame arrives within 50 ms on USB.
8. **The image (section 8.1, layer 3).** `zr10_ros2` and `zr10_stream` come from `video.root` (section 9 here), copied
   into the build context at `image build` time (never into the repo) and built with colcon (copy-install, the exact
   package folders and `--packages-select zr10_ros2 zr10_stream`, as section 9 describes). Also build the proposal
   package `astra_video_proposals` from `simulation/video/proposals/` (the entrypoint's `pick_tracker` looks for
   `/opt/astra/extra/proposals/*.launch.xml`, and the launch file needs the package's `prepare_photo_folder`; with
   `detector:=external` it does not need `run_detector.sh`), and install `ffmpeg`, `nlohmann-json3-dev` and
   `libcurl4-openssl-dev` from apt. `image build` refuses the `video-cam` CMakeLists with the line-17 error of section
   9. Record the sources' checksums and `dirty` states in an image label (`org.astra.video_src_sha`). The x86 flavour
   uses the same layers on the owner's x86 base (12:32).
9. **`TRACKER_CMD` and `launcher.sh` (section 8.2).** For the new pipeline: `ros2 launch
   /home/astra-dev/astra/simulation/video/proposals/field_video.launch.xml detector:=external proposals:=true`
   (`proposals:=false` for the team's code as it is). The path follows wherever the image puts `simulation/video/`;
   it must be absolute and without spaces (8.1). **The image's `launcher.sh` is THIS repo's copy** (= the board's
   `6beb5f1` file plus the `${TRACKER_CMD:-...}` line), because the board's ignores `TRACKER_CMD`: add that to 12.1
   (item 11 below). In REAL mode on the drone the same holds until the team adopts P7.
10. **Readiness (section 7 item 2).** `stack_check` inside the container uses the video node list of 8.1 and the
    detector's `Ready: subscribing` line instead of `/sahi/get_health`.
11. **Differences (section 12.1).** Add items 1, 3, 4, 5, 9, 10, 11, 12, 13, 19, 20, 24, 26 and 30 of section 12
    here, and change item 5: the detector container follows C11 (the stack's user with proposals on; root, like the
    board, with proposals off).
12. **CPU (risk 25).** The stream node still takes about one core (93 % on the board) until the team applies P3. The
    detector container has no CPU limit, as on the board.
13. **Never the real camera:** the existing guard covers RTSP too. Inside the sealed container, `192.168.144.25:8554`
    is the relay, never the board's Ethernet camera.
14. **`--ipc shareable` (section 8.3).** `jetson.sh`'s `run_cmd_sim` already passes `--ipc shareable` for the board
    AND the virtual Jetson (line 933), which the detector's `--ipc container:...` needs (Docker's default is
    `private`). HIL_DESIGN 8.3's VIRTUAL line shows `...` there: please write the flag out, so the doc matches the
    code.
15. **The photo folder in HIL.** SIM mode mounts `D/photos` onto the team's `mapping_photos` path, so the folder
    exists; the launch file's `prepare_photo_folder` step finds it there and only warns when old photos are in it.
    REAL mode mounts the team's own folder; if it is missing on a fresh board, the step creates it (10.2), so the
    board's `main_controller` cannot hover for ever at the end of the survey.
16. **The real-board steps stay documented, not run** (owner 15:14): the first deploy of the arm64 image, the yolo26
    engine warmup (item 3), the first HIL flight, each with the exact command and the expected output, in the Jetson
    design's sections 9 and 16.

The picture for the real Jetson:

```
PC (WSL)                                                    Jetson (container astra-hil, network "none", own lo)
 emulator 192.168.144.25 :8554 RTSP <-- ssh -R cam_rtsp.sock <-- relay 192.168.144.25:8554 <-- stream_node's ffmpeg
          :37260 SDK  <-- pc-sdk <-- ssh -R cam_sdk.sock <------ relay 192.168.144.25:37260 <-- zr10_node, proposals
          :82 HTTP    <-- pc-http <-- ssh -R cam_http.sock <---- relay 192.168.144.25:82 (unused unless sync_photos)
 SITL 5763 <------------------------ ssh -R mavlink.sock <------ relay 127.0.0.1:5763 <-- MAVROS
 ros_tap receive <------------------ ssh -L tap.sock <---------- ros_tap send (/image_detection, /yolo26/detections ...)
 collect.sh (rsync) <---------------------------------------- D/runs/<id>/yolo26/camera_results/, logs
                                                             container astra-hil-detector (joins network + IPC; C11):
                                                               /zr10/image_raw -> TensorRT -> files + /yolo26/detections
```

## 15. (i) Decisions and questions still open

### 15.1 Owner decisions

| # | Decision | Recommendation |
|---|---|---|
| D1 | Make `video` (proposals on) the demo default once the checks of 13.4 pass, even though its drops rely on our proposal | yes, as the owner asked on 12:04 and 12:25; every page labels the proposals |
| D2 | Ask the team T1-T5 (section 10.4) and send them the proposals folder and P1-P5 | yes |
| D3 | The stream defaults while the real ones stay unmeasured: H.264 (your 12:25 entry), 1920x1080, 30 fps, 2000 kbit/s, GOP 1 s. **New evidence:** both of the manual's main-stream examples set H.265 (6.2), and H.265 costs the Jetson's CPU much more to decode. Keep H.264 as the default, or switch the default to H.265? | keep H.264 (your entry) and run every stream check with H.265 too; switch only if the team confirms H.265 on their camera |
| D4 | Ask the team to (a) commit and PUSH the detector's local branch `codex/ros2-image-subscriber` (today the board's build is `b02cac6`, a local commit, plus uncommitted changes, and only those changes have the topic mode the board uses), (b) put `zr10_stream` in git (it is in none we know), (c) push the `zr10_ros2` CMake fix (P2) to `video-cam`, so the twin can name commits instead of copied folders | yes |
| D5 | Copy the board's folders (`~/astra_ws`, `~/astra_camera_detection`) to this PC as `video.root` (read-only on the board, with the team's agreement), or keep using the 11:51 snapshot | snapshot until the team's git has them |
| D6 | TensorRT precision for the twin and the Jetson (FP16 or FP32), until the team chooses | FP16, recorded in every run |
| D7 | The detector settings rule (C11, section 3): `--user` (P4) and the reliable QoS (P9) count as proposals, so they are on exactly when proposals are on; `video --no-proposals` and `board` run the team's image as root with the team's QoS, like the board (EXPECTED: no pictures arrive). The same in the virtual Jetson and HIL | yes, this rule; overrides only for tests, recorded as differences |
| D8 | Install `ffmpeg` and `nlohmann-json3-dev` in WSL (needs your sudo password), or run the pipeline only in the virtual Jetson | install (two packages) |
| D9 | If 1080p30 rendering slows Gazebo below a real-time factor of 0.7: render at 1280x720 and scale up to 1920x1080 in the encoder | yes, recorded as a difference |
| D10 | The ZR10 lens also for the legacy photo pipeline, and re-planning the survey missions for 61.5 degrees | lens: yes (owner 12:25, ZR10 everywhere); missions: ask the worlds' owners |
| D11 | Set up GPU access for the WSL Docker Engine (NVIDIA Container Toolkit), allowed by the owner on 12:25 | yes, first thing for B3 (the Jetson design reports `--gpus all` as VERIFIED since) |
| D12 | Send the field-safety proposals to the team now, before anything else: P8 (the hold that never ends without the photo folder), P7 (the boot launcher cannot start the new pipeline), P9 (many pictures lost on the way to the detector), P1 (the gimbal STOP's bad checksum) | yes, as one short note with the line numbers |

Settled by the owner (no longer open): the camera is the ZR10; no measurement on the real camera or its stream;
detection on the GPU with TensorRT; the PC container is the Jetson's software version for version; no real Jetson
for now, the virtual Jetson flight is the acceptance test (15:14).

### 15.2 Questions for the team

T1-T5, T9 and T10 in section 10.4, plus:

- T6: `zr10_ros2`'s README names `rtsp://.../video1`, while `zr10_stream` uses `/main.264`. Which one does the field
  use? (The twin serves `/main.264` only.)
- T7: On the board, does the detector receive any pictures? (11.2: probably not, because of root against the normal
  user; and with the same user, many are lost with the current QoS.) The detector's shutdown line
  `Topic summary: received=... dropped=...` answers it.
- T8: Will the boot launcher keep starting the old photo nodes next to the new pipeline (two controllers on one
  camera)?
- T11: Which codec and size does the camera's main stream really use (H.264 or H.265)? The SIYI app or SDK 0x20
  shows it (we do not touch the real camera; owner 12:25).

### 15.3 Proposals for the team (written up in `simulation/video/proposals/TEAM_PATCHES.md`; applied nowhere)

| # | File | Change | Why | Needed to run? |
|---|---|---|---|---|
| P1 | `zr10_ros2/src/zr10.cpp` line 34 | `0xCDCE` -> `0xCDEC` (the third value on that line) | a checksum table typo: 1.55 % of the 1-byte commands and **1.94 % of the gimbal speed commands, the gimbal STOP included**, carry a bad checksum and are dropped by the camera; a dropped watchdog stop leaves the gimbal turning (7) | no (field safety) |
| P2 | `zr10_ros2/CMakeLists.txt` line 17 on `video-cam` | `target_link_libraries(zr10_node CURL::libcurl)`, as in `~/astra_ws` | the PRIVATE form does not build on Humble | yes, for a fresh clone |
| P3 | `zr10_stream/scripts/stream_node.py` line 72 | `msg.data = array.array('B', rgb)` (+ `import array`) | 196 ms -> 0.1 ms per frame on this PC (VERIFIED); frees most of the board's 93 % core | no (speed) |
| P4 | the detector's `docker run` | add `--user "$(id -u):$(id -g)"`, as the team's own readme does | root-owned output files, and no pictures at all between a root container and a user publisher (11.2) | yes, for any picture to arrive (EXPECTED on the board) |
| P5 | the detector `Dockerfile` line 76 | pin the Ultralytics version, and keep `yolo26.onnx` as a versioned file | the same model on every build | no |
| P6 | the proposals of section 10.3 (with the labelled copy of `target_geolocation.py`), the launch file of 8.2 (with its photo-folder step) and the GPU patch set of 8.3 | adopt, change or replace | the mission link, the camera pointing, TensorRT | for drops and for the GPU rule |
| P7 | `launcher.sh` line 73 (board, `6beb5f1`) | after copying `field_video.launch.xml` into `main/launch/` and `astra_video_proposals` into `ros2_ws/src/` and rebuilding: `TRACKER_CMD="${TRACKER_CMD:-ros2 launch main field_video.launch.xml}"` | the board's launcher ignores `TRACKER_CMD` and always starts the obsolete photo nodes; this line boots the field pipeline with no environment variable and keeps the simulator's hook (8.2) | **yes**, to start the new pipeline at boot |
| P8 | `main/src/main_controller.py` `_check_all_images_processed` (board lines 306-342) | adopt this repo's `processing_hold.py` and the hold code that uses it (commit `94ccf0b`), or at least end the hold when the folder is missing after the timeout | without `mapping_photos` the hold never ends: `os.listdir` raises every 2 s, the `except` only logs it, and the drone hovers until a failsafe (10.2). The launch file's folder step avoids it; P8 makes it impossible | **yes (field safety)** |
| P9 | the detector's image subscription (`detector_node.cpp`, `rclcpp::SensorDataQoS()` in the topic mode) | subscribe RELIABLE, depth 5 (our patch: parameter `image_qos:=reliable`), or ship a Fast DDS profile with a bigger shared-memory segment (8 MB worked) | with best effort, 2.7 MB pictures are lost even between programs of one user: 24 and 11 of 40 arrived on this PC; reliable: 40 of 40 (VERIFIED, 11.2) | yes, for every picture to arrive (EXPECTED on the board) |
| P10 | `zr10_stream/scripts/stream_node.py` line 68 | stamp the message with the time ffmpeg delivered the picture (`FrameSource` already stores it in `latest[1]`, `frame_source.py` line 70), converted from the monotonic clock: `now - (time.monotonic() - frame_time)` | the stamp is the timer tick today, 0-0.5 s after the picture arrived; that spread (±1.4 m at 5.5 m/s) cannot be corrected afterwards (6.5) | no (aim accuracy) |

## 16. Risks and fallbacks

| # | Risk | Sign | Fallback |
|---|---|---|---|
| 1 | `CameraZoomPlugin` does not change the video camera's field of view in Gazebo Harmonic | A3 fails | digital zoom over the whole 1-30x range (the picture gets soft above about 3x), listed as a difference |
| 2 | 1080p30 rendering lowers the real-time factor | A5 below 0.7 | D9 (render at 720p, encode at 1080p); NVENC to save CPU |
| 3 | copying frames in Python slows the emulator's SDK loop | SDK replies late; late frames in `stream_frames.csv` | move the stream into its own process (the module interface allows it), sharing the state via `emulator_state` |
| 4 | Docker cannot reach the GPU in WSL | B6 shows `ort_cuda` or `opencv_cpu` | the backend chain; D11; recorded as a difference |
| 5 | TensorRT 10.3 cannot parse the end-to-end YOLO26 ONNX (opset 12) | the engine build fails | export with `END2END=False` (the team's Dockerfile already supports it) and let the patched code do the NMS; or Ultralytics' own engine export on that machine |
| 6 | the `ort_cuda` wrapper gives different boxes from the team's C++ code | its comparison test fails | fix the pre- or post-processing; until then use `opencv_cpu`, not a different program |
| 7 | pictures never reach the detector, or most are lost (root vs user; best-effort 2.7 MB pictures) | `detector received 0 of ~N` or a low share | that is what `board` and `--no-proposals` should show if the board has it (C11); with proposals the detector runs with `--user` and reliable QoS (P4, P9); if pictures are still lost, a Fast DDS profile with a bigger shared-memory segment (VERIFIED to help, 11.2) |
| 8 | the adapter's aim is off | large aim error in the score | tune `delay` with the measured camera-to-detector latency (11.3); check gimbal and heading conventions with the photo path's tests |
| 9 | `No objects detected.` floods the ground station | many texts in `events.csv` | the adapter already sends only frames with mapped boxes (10.3) |
| 10 | the annotated PNGs fill the disk | run folders of more than 1 GB | measure in the first run; `logs.sh bundle` leaves them out; an option to keep only frames with detections (a difference, because the team keeps all) |
| 11 | the team changes its code or its commands | `video.sh check` checksum warnings; `commands.txt` no longer matches a new probe | update `video.root`, rebuild; this page's tables are updated from the team's README |
| 12 | late joiners wait for a keyframe | the first frame after connect takes up to 1 s | well inside `stream_node`'s own 10 s frame deadline (`FrameSource`); a real camera behaves the same |
| 13 | `CameraZoomPlugin` clamps the zoom | A3: the field of view stops at 11.5 degrees; `has been clamped` in `sim.log` | `<max_zoom>20</max_zoom>` in the xacro (6.3) |
| 14 | a detector container is left over after a crash or a killed launch | the next start fails with a name clash | the wrapper and `video.sh` remove a leftover container with our label first; one without our label stops the start with a clear error (8) |
| 15 | `TRACKER_CMD` breaks on a path with a space, a `~` or a relative path | the launch never starts; `launcher.sh` prints a `ros2 launch` usage error | demo.sh builds absolute paths and refuses spaces (8.1) |
| 16 | the frame matching stays ambiguous over uniform ground | many rows with `match_ambiguity` above 1 | the sequence alignment of 11.3; the summary reports the share; the scores that need exact poses use only unambiguous rows |

## 17. Evidence and sources

**VERIFIED on this PC (2026-10-31), by the two read-only studies:**

- gst-rtsp-server bound `192.168.144.25:8554` (`ss`: `LISTEN 0 5 192.168.144.25:8554 ... python3`). The team's exact
  ffmpeg command (Ubuntu `ffmpeg 7:4.4.2-0ubuntu0.22.04.1` in a throwaway `ubuntu:22.04` container) on
  `rtsp://192.168.144.25:8554/main.264` got `frames 17, fps 2.12, bad 0, lat_ms_p50 592`.
- Latency from picture to ffmpeg's output: 580-593 ms median through gst-rtsp-server and 614-628 ms through mediamtx
  v1.21.3, for x264 and NVENC alike. A GStreamer client on the same server: 12-22 ms median.
- Encoder cost (100 % = one core of 32): x264 ultrafast/zerolatency 720p30 40.9 %, 1080p30 84.1 %; NVENC (nvh264enc in
  WSL) 720p30 14.5 %, 1080p30 30.0 %, encoder use 4-9 %.
- `ffmpeg -h`: RTSP `listen` is a demuxer (input) option only. mediamtx v1.21.3: checksum OK, MIT licence, MoQ server
  on `:8892/:8893` unless `moq: no`; an early reader gets `404 Not Found`.
- `msg.data = bytes` for one 1280x720 rgb8 frame in rclpy: 196.3 ms (three runs: 199.9 / 203.9 / 201.7 ms in the other
  study); `array.array('B', ...)`: 0.0-0.1 ms.
- CMake on ROS 2 Humble with a minimal package: the plain `target_link_libraries` form configures; adding the PRIVATE
  form after `ament_target_dependencies` fails ("The plain signature for target_link_libraries has already been
  used"). (The build check of the two real `zr10_ros2` copies did not run: the heavy lock was busy.)
- The `zr10.cpp` CRC table: one mismatch, entry 106 (`0xcdce` should be `0xcdec`). Bad-checksum sequence numbers per
  65,536 (recomputed for revision 2 with `python D:\suas_scratch\video_pipeline\crc_recheck.py <zr10.cpp>`, matching
  the study and the reviewer): photo 1,019, record 1,021, autofocus 1,019, zoom in/out/stop 1,017 (1.55 %, first
  bad 38); gimbal stop `0x07 00 00` 1,269, gimbal speeds 1,267-1,273 (1.94 %, first bad 43); centre 1,019 (first
  bad 5). Seven of the manual's example packets are byte-identical to what `zr10_node` builds at sequence 0.
- WSL: `ffmpeg` not installed, `nlohmann-json3-dev` not installed, `libcurl4-openssl-dev` installed;
  `GstRtspServer OK GStreamer 1.20.3`; `x264enc`, `nvh264enc` and `openh264enc` present.
- From the Jetson design (its section 17 item 7): root and uid 1000 in one container heard nothing from each other
  over Fast DDS within 10 s, in either direction.

**VERIFIED for revision 2 (2026-10-31, about 15:40-16:00; light read-only checks, no simulator, no Docker build):**

- **Big pictures over Fast DDS, same user** (`D:\suas_scratch\e2e\vid_designfix_dds.py` + `vid_designfix_dds.sh`,
  `vid_designfix_dds2.sh`, `vid_designfix_dds3.sh`, run with `MSYS_NO_PATHCONV=1 wsl -d Ubuntu-22.04 -- bash
  /mnt/d/suas_scratch/e2e/<script>`): user `jarrod` uid 1000, `rmw_fastrtps_cpp`, ROS domain 93, `net.core.rmem_max`
  212992, 40 pictures of 1280x720 rgb8 (2,764,800 bytes) at 2 Hz, publisher RELIABLE/VOLATILE depth 1 (like
  `stream_node`), subscriber in a second process. Output: `SUB received=24` and `SUB received=11 ... ids=[1, 3, 4, 5,
  10, 11, 17, 20, 22, 28, 31]` with `qos_profile_sensor_data` (the detector's QoS); `SUB received=5` with UDP only;
  `SUB received=40 ... age_ms_p50=15.97` with a RELIABLE depth-5 subscriber; `SUB received=40 ... age_ms_p50=6.42`
  with the best-effort subscriber and an 8 MB shared-memory segment (profiles `vid_designfix_udp_only.xml`,
  `vid_designfix_shm8m.xml`). The subscriber was Python (`rclpy`), standing in for the C++ detector.
- `CameraZoomPlugin.cc` (`/home/jarrod/ardupilot_gazebo/src/`, read with `vid_designfix_probe.sh` and
  `vid_designfix_zoom.sh`): `refHfov{2.0}` line 131, `maxZoom{10.0}` line 140, the clamp and `gzwarn` lines 420-426,
  `goalHfov = refHfov / clampedZoomCmd` line 428; `max_zoom` read from the SDF line 350.
- GStreamer in WSL: `x264enc`, `nvh264enc`, `x265enc`, `nvh265enc`, `rtph264pay`, `rtph265pay` present;
  `GstRtspServer.RTSPSession` default timeout 60 s; `ffmpeg` still not installed; `192.168.144.25/32` on `lo`.
- Humble's launch XML: `ExecuteProcess.parse` reads no `sigterm_timeout` attribute; the timeout is the launch
  configuration `sigterm_timeout`, default 5 (`launch/actions/execute_local.py` lines 90-93).
- The detector's git state, read-only in the snapshot (`git --no-optional-locks log/branch -vv/remote -v/status/diff
  --stat`): `* codex/ros2-image-subscriber b02cac6 init` (2026-10-31 10:34 -0700), remote
  `https://github.com/llaammbbddaa/objectDetection.git` with `master`, `ros2_implementation`, `yolo26`; 10 entries
  modified (220+/37-, `entrypoint.sh` mode only), 2 untracked; the diff of `src/detector_node.cpp` adds `input_mode`,
  `image_topic` and `Ready: subscribing`. No `.git` in `astra_ws`, `zr10_ros2` or `zr10_stream`.
- `git show 6beb5f1:launcher.sh` line 73 `TRACKER_CMD="ros2 launch main tracker.launch.xml"`; this repo's
  `launcher.sh` line 78 `TRACKER_CMD="${TRACKER_CMD:-ros2 launch main tracker.launch.xml}"` and line 138
  `$TRACKER_CMD &`; `diff` between them: that line and a 5-line comment only.
- `git show 6beb5f1:ros2_ws/src/main/src/main_controller.py`: lines 306-342 (the hold), 383-384 (`msg.latitude`
  stored as is); `git grep makedirs 6beb5f1`: `new_od.py` line 129 and `video_cam/storage_manager.py` create
  `mapping_photos`; `.gitignore` line 45; `git ls-tree 6beb5f1` has no `target_geolocation.py` or
  `processing_hold.py` (added here by `81a7950` and `94ccf0b`).

**PROBE (the board, 2026-10-31 11:47, read-only):** the processes and command lines of 2.1; ROS_DOMAIN_ID 0, Fast
DDS; `stream_node` 93 % of one core, ffmpeg 31.5 %, the detector 0.4 % over 7.8 min, GPU 0 %; the Jetson's Ethernet
is `192.168.144.20`, the camera answers at `192.168.144.25`.

**SOURCE:** the team's code in the board snapshot (`zr10_node.cpp`, `zr10.cpp`, `media.cpp`, `stream_node.py`,
`frame_source.py`, `stream.launch.py`, `detector_node.cpp`, `yolo26_detector.cpp`, `entrypoint.sh`, `Dockerfile`,
`readme.md`, `ROS2_INSTRUCTIONS.md`); the team repo at `6beb5f1` (`main_controller.py`, `new_od.py`,
`tracker.launch.xml`, `ImageResult.msg`); this repo (`main_controller.py` `locate_detection` and `image_result_cb`,
`target_geolocation.py` `aim_at_box`, `processing_hold.py`, the emulator's `emulator.py` / `command_handler.py` /
`camera_state.py`, `model.sdf.xacro`, `sim_paths.py`, `stack_check.py`); the SIYI ZR10 User Manual v1.7 (pages 14-15,
21, 42-71, 93-94); ArduPilot's SIYI ZR10 page (the `main.264` address).

**Not done:** no simulator run, no Docker build, no change to any team file, no contact with the Jetson or its camera.
Not yet tested: the root-vs-user and QoS combination with the team's real C++ detector in Docker (13.3 step 3), the
1080p30 real-time factor (A5), the build of the two real `zr10_ros2` copies (the heavy lock was busy during the study).

## 18. For the live code map (what the code-map refresh must add once this is built)

The user asked that the live code map (`simulation/codemap/`, another stream's folder) stays up to date with the new
architecture. Nothing here is built yet, so nothing is added now. When the video work lands, the lead's code-map
refresh needs these, so `extract.py --check` passes and the map shows the new path:

| What | Where it goes in the map | Note |
|---|---|---|
| `simulation/video/` (new top-level folder: `video.sh`, `lib/`, `proposals/`) | system "Camera and target detection" (or "Simulator control" for `video.sh`) | `systems.yaml` needs a line for the new folder, or `extract.py --check` fails |
| `simulation/video/proposals/astra_video_proposals/` (a ROS package: `gimbal_pointing`, `detection_adapter`, `prepare_photo_folder`, `run_detector.sh`, `geolocation.py`) | "Camera and target detection", labelled PROPOSAL | `geolocation.py` is a copy of `main/src/target_geolocation.py`: show the link |
| `ros2_ws/src/simulation/sim_video/` (`video_monitor`, `video_export`, the pure modules) | the run-logging / scoring system | a ROS package, owned by its `package.xml` |
| `siyi_emulator/rtsp_stream.py`, `stream_settings.py` | "Simulated world" (the camera emulator) | the RTSP server lives inside the emulator process |
| outside programs: `ffmpeg` (started by `stream_node`), the detector container (`yolo26-ros-subscriber` / `astra-video-detector`, process pattern `detector_node`) | externals | the team's video code lives OUTSIDE the repo (`video.root`): `zr10_node`, `stream_node.py` |
| links: RTSP TCP 8554 (emulator -> ffmpeg), SDK UDP 37260 (`zr10_node`, proposals -> emulator), `/zr10/image_raw` (`stream_node` -> detector), `/yolo26/detections` (detector -> adapter), `/image_detection` (adapter -> `main_controller`), `/sim/camera/preview` (emulator -> `pov_server`) | links / topic owners | the photo-path links stay, marked "legacy (2025) photo pipeline" |
