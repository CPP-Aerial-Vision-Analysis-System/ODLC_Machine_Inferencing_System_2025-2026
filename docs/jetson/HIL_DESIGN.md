<!--
docs/jetson/HIL_DESIGN.md -- PURPOSE: the design of "Jetson in the loop" (hardware in the loop, HIL): the team's
flight software (MAVROS + the 6 nodes of tracker.launch.xml, with the YOLO detector on the GPU) runs in ONE Docker
container on the real Jetson Orin Nano, while Gazebo, ArduCopter SITL, MAVProxy, the SIYI camera emulator and
Mission Planner stay on the PC. Since the owner's entries of 11:56-12:04 the field camera path is the team's new
video pipeline (zr10_ros2 + RTSP stream + yolo26 detector); last year's photo path is kept as a "legacy" option.
It says what runs where, every connection (who opens it, which port), how the owner
logs in (his own password, typed in a visible window; the tools reuse that one login), how the PC sees what the
Jetson does, the container image and its modes (sim / real / virtual), how the image is built OFF the board and
moved onto it, the owner's commands, clocks, what the "virtual Jetson" on the PC proves, what still differs from the
field, the risks and fallbacks, and the exact files two engineers build in parallel (JETSON-SIDE and PC-SIDE) with
the contract between them. Status: DESIGN, revision 2 (2026-10-31, after the technical review); nothing in it is
built yet. Verified facts are marked VERIFIED with the command that showed them (section 17); facts read in code or
documentation are marked SOURCE (section 18); everything else is the plan, or an EXPECTED result still to be tested.
The user guide (plug in and test) will be docs/jetson/README.md, written by the JETSON-SIDE builder.
-->

# Jetson in the loop: design

**Status: revision 4 (2026-10-31, see "What changed in revision 4"); revision 3 said: The JETSON-SIDE half is built (see "What changed in revision 3" and the guide
`docs/jetson/README.md`); the PC-SIDE half (`simulation/jetson/pc/`) is built by its own builder. Originally:
DESIGN, revision 2, after the technical review.** Two builders
make the JETSON-SIDE and the PC-SIDE parts at the same time (section 15). The JETSON-SIDE builder starts with the
first-connection kit (`jetson.sh login`, `check`, `setup`), because that is what the owner wants first. Facts marked
**VERIFIED** were checked on this PC today (section 17 has the commands and outputs). Facts marked **SOURCE** were
read in code or documentation (section 18). Everything else is the plan, or an **EXPECTED** result that still has to
be tested. Nothing was run on any Jetson by this design work (owner update 11:22); the board facts come from the
read-only live probe the owner allowed at 11:47 (`D:\suas_scratch\coord\jetson_probe.md`).

**The field camera pipeline changed during this revision (owner entries 11:52 to 12:10).** The field system is now
the team's NEW video pipeline: `zr10_ros2` (camera control, SIYI SDK), `zr10_stream` (RTSP stream ->
`/zr10/image_raw`) and the yolo26 detector in its own Docker image, started in the twin by the video workflow's
proposed field launch file (12:10). Last year's photo path (video_cam `siyi` shutter +
HTTP download, `mapping_photos/`, `new_od` on photos) is now the **"legacy (2025) photo pipeline"**: kept working as
an option, off by default. Section 6.3 says how this design carries the new pipeline (the relay also passes RTSP on
TCP 8554; the detector container joins our container's private network). The simulator side of the new pipeline
(the RTSP stream from Gazebo, the ZR10 SDK answers, the pipeline in the virtual Jetson) is designed by the separate
video-pipeline workflow in `docs/simulation/VIDEO_PIPELINE.md`; this design must stay consistent with it. Where
sections 6-7 and 9-12 below talk about photos, `new_od`, `/image_detection` or the TensorRT `.engine`, they describe
the legacy option unless they say otherwise.

The guide for people (plug the board in, log in, run one command, read the answer) will be `docs/jetson/README.md`.
It starts with "Plug in and test".

## What changed in revision 2 (the review, and the owner's entries 11:16 to 11:40)

| Review finding or owner entry | What the design does now | Section |
|---|---|---|
| 11:16: the separate "jetson-kit" agent was stopped; the JETSON stream builds the kit | the JETSON-SIDE builder owns and creates `jetson.sh`, `lib/**`, `test/**`, `docs/jetson/README.md`; no waiting rule | 15.0, 16 |
| 11:40: login by the owner, not by keys | `jetson.sh login` opens a visible window (`ssh -M -S`); every command reuses it; key login is optional | 4.1 |
| 11:31: build images OFF the board | the arm64 image is built on the PC (QEMU) and pulled by the Jetson from a registry on the PC through the SSH login (only changed layers move); the TensorRT engine is a separate step, once, with the owner's OK | 9.5 |
| 11:31: "team busy?" refusal; our own names | `check`, `deploy`, `warmup`, `start` refuse while the team uses the board; everything is `astra-hil` / `~/astra_hil` / label `org.astra.owner=hil` | 9.6, 15.3 |
| 11:22: ROS_DOMAIN_ID | SIM is sealed (no network, own IPC: VERIFIED) and also uses domain 77 + `ROS_LOCALHOST_ONLY=1` (11:52); REAL uses the team's domain; `--domain N` on a shared bench board | 8.5 |
| 11:16 item 5: power mode | `check` and every run summary report `nvpmodel -q`, the board's mode list and the clock state; `tegrastats` is recorded; nothing is changed | 9.2, 9.7 |
| 11:16 item 7: differences from the field | a fixed list, carried by the README and every run summary | 12.1 |
| Refuted: "the relay re-dials after a link drop" | the Jetson's sshd does not notice a pulled cable (VERIFIED: no end-of-file after 19 s). New rules: idle timeouts with PING frames, a gate that drops MAVROS bytes while the link is stale, and link.sh kills the old sshd session before it reconnects (VERIFIED: end-of-file 0.15 s after the kill) | 5, 6, 9.4, 15.3 |
| Refuted: "the clock only affects logs" | score_run matches photos by their file times; collect.sh now shifts the mirrored photos' times by the measured offset; warning at 0.5 s | 11 |
| Refuted: "score_run and phase_monitor run unchanged with the tap" | the topic list is now derived from `ros_common.FlightData` and phase_monitor; node presence is forwarded so phase_monitor's "main_controller is NOT RUNNING" warning keeps working | 7 |
| Refuted: "`sudo nvpmodel -m 2` (MAXN SUPER)" | removed; mode numbers differ between boards and releases (SOURCE); the owner chooses | 9.2 |
| Refuted: "binfmt handler VERIFIED" | it is not ours and not durable (VERIFIED: registered from outside the distro, no package); the image build now has its own "arm64 emulation" step | 9.5, 17 |
| Contract too thin | `lib/target.sh` names, `jetson.sh target` output, the command lines of every relay and tap role, frames with PING, the reconnect steps | 15.3 |
| Minor findings | `launcher.sh hw` + `ASTRA_HIL_MODE`; `exec` as the stack's user (VERIFIED trap); `.25` guard; OpenCV, torch hash, build_images.sh patch; new check rows; camera profile stated (VERIFIED sizes); REAL mode uses the team's own photo folder | 8, 9, 13, 15.6 |
| 11:52 live probe of the real board | L4T R36.4.7 (an r36.4 base fits), TensorRT Python bindings present, Docker by flag `--runtime nvidia`, MAXN_SUPER, ~4.3 GB free, eth0 = `192.168.144.20` (not `.100`); the team's stack runs on domain 0 since boot: our SIM container uses domain 77 + `ROS_LOCALHOST_ONLY=1` inside its sealed network | 8.5, 9.6, 10.2 |
| 11:56-12:10: the field pipeline is the new video pipeline; the photo path is legacy; the twin runs the video workflow's proposed field launch file | the relay also carries RTSP (TCP 8554); the unchanged launcher.sh runs the proposed launch file through `TRACKER_CMD`; the detector image runs as a second container inside our container's private network and IPC (no Docker socket in the container); the tap carries the new detector topics; the photo path stays as the legacy option; how detections reach main_controller is an open question for the owner/team | 6.3, 7, 8.2, 12.1, 15.3, 15.5 |

## What changed in revision 3 (the JETSON-SIDE builder, 2026-10-31 afternoon)

The JETSON-SIDE half is now BUILT (`simulation/jetson/jetson.sh`, `lib/**`, `image/**`, `test/**`,
`docs/jetson/README.md`). Where building it showed this design was wrong or incomplete, the design was corrected here;
each row names the section that changed. Owner entries 12:25 and 12:32 arrived while building.

| What changed | Why | Section |
|---|---|---|
| `JETSON_DATA` is `""` until the first connection, then `$HOME/astra_hil` of the Jetson user as the Jetson reports it (`jetson_resolve_data`), saved by login/setup | the user's home is not always `/home/<user>`; the fake-board test showed the guess fails | 15.3 a |
| `target.sh` also sets `JETSON_DEST`, `JETSON_CONTAINER` (`astra-hil` / `astra-hil-virtual`), `JETSON_IMAGE`, and offers `jetson_die`, `jetson_cm_check`, `jetson_resolve_data`, `jetson_save_target`, `jetson_target_print`; test hooks `JETSON_SSH_BIN`, `JETSON_RSYNC_BIN`, `JETSON_STATE_DIR`, `JETSON_CM_PATH`, `JETSON_DATA_DIR` | the PC side and the tests need them | 15.3 a |
| `relay.status` has more keys: `LISTEN=yes|no` (what `start` waits for), `STARTED`, `MAV_CLIENT`, `MAV_DROPS`, `MAV_DOWN_S` (time SITL was silent, across re-dials), `SDK_DROPPED`, `RTSP_ACTIVE`; `MAX_GAP_S` is the largest gap INSIDE one connection (photo delays), drops are counted apart; `--rtsp-port 0` turns RTSP off | the run summary needs drops and gaps apart (the relay test showed the gap rule alone misses a drop) | 15.3 d |
| the entrypoint passes `--parts common,new|legacy` to `ros_tap.py send` (the PC side's option name), only if the tap knows it | consistent with the PC-side ros_tap.py | 7, 15.3 e |
| the container's pipeline is `ASTRA_PIPELINE=auto|new|legacy`; `auto` = new only when the image holds the proposed field launch file AND `zr10_ros2`, else the legacy photo pipeline with a log note. Today neither exists (`simulation/video/proposals/` not written yet), so images built today run legacy | do not pretend the new pipeline runs | 8.2 |
| entrypoint exit codes: 20 guard, 21 could not add the address, 22 socket folder missing, 23 unknown mode, 24 `--pipeline new` without the proposed launch file | clear errors | 15.4 |
| the detector container of the new pipeline is started by `jetson.sh start` only when `JETSON_DETECTOR_IMAGE` (and `JETSON_DETECTOR_CMD`) are set; the video workflow names them | the image and command are not known yet | 6.3, 8.3 |
| **x86 layer 1 (owner 12:32): `nvcr.io/nvidia/tensorrt:24.08-py3` (Ubuntu 22.04, CUDA 12.6, cuDNN 9.3, TensorRT 10.3) + ROS 2 Humble**, built by the SAME `Dockerfile.l4t-ros` (now `ARG BASE_IMAGE`, apt arch from dpkg); layer 2 = `astra-dev:hil-x86`; `--x86-base ros` keeps the old plain-ROS base | the PC container must be the Jetson's software version for version | 8.1, 9.5 |
| layer 2 for arm64 (and x86 trt) is built by `jetson.sh image build` with a direct `docker build --target dev -f docker/Dockerfile --build-arg BASE_IMAGE=... --build-arg TORCH_INDEX_URL=...`; **PATCH P1 to build_images.sh is dropped** (P2, the torch hashes, stays a proposed patch); the Jetson torch hashes are recorded in `image/jetson_pins.env` (torch `62a1beee9f2f1470...`, read from the index page) | no edit of another stream's file was needed | 8.1, 15.6 |
| the virtual Jetson has no memory/CPU limit by default; `start --jetson-like` gives it the board's 6 CPUs / 7.4 GB (owner 12:32); the board keeps `--memory 5g --cpus 5` | a PC torch process needs more host memory than the board's | 8.3 |
| the engine row is FAIL (exit 19) only for the legacy pipeline (new_od needs it); with the new pipeline it is a WARN (need unknown, 6.3) | do not block a flight on an unknown need | 9.2 row 16 |
| check row 1 uses the PC side's `pc/windows_link_check.ps1` (`-Target HOST -Port N`) when present, else a short read-only inline query | the PC side delivered it | 9.2 |
| `deploy --via build-on-board` is not built in this version (exit 2 says so) | last resort only | 9.5 |
| `jetson.sh login --here` runs the login in the current terminal when no window can be opened; the window writes `~/.astra_sim/jetson/login.env` (which board it is logged in to), and a command refuses (17) when the open login is to another host | plain Linux without a desktop; a wrong-board guard | 4.1 |
| two more differences from the field: MAVROS 2.14.0 in the image vs 2.12.0 on the board (no apt source has 2.12.0); the PC flavor runs the `.pt` on the RTX 4060 | honest list | 12.1 |
| evidence of what was built and tested | | 17 |

## What changed in revision 4 (the JETSON-SIDE builder, 2026-10-31 15:14 onward: "no real Jetson for now")

The owner (15:14): the board cannot be connected; build and PROVE everything with the virtual Jetson; a full visible
simulator flight with it is the acceptance test; keep the real-board steps ready and documented. Changes:

| What changed | Why | Section |
|---|---|---|
| **The virtual Jetson runs the team's own `tracker.launch.xml` unchanged** (`TRACKER_CMD` unset, exactly as on the board) when its image has TensorRT (the default `--x86-base trt` image). Its new_od finds `ros2_ws/src/yolo26m.engine` (a link into `/models`) by itself. The `team_stack.launch.py ... model_path:=/models/yolo26m.pt` override remains only for the plain-ROS image (`--x86-base ros`, no TensorRT). This replaces revision 3's "the PC flavor runs the `.pt` on the RTX 4060" | owner 12:25 item 5 (TensorRT in Docker on the RTX 4060 when it works; it does: `--gpus all` is VERIFIED) and 12:32 (the same software, one engine per machine from the same model) | 8.2, 12.1 |
| **`jetson.sh warmup --link virtual`** builds the PC's own engine on its GPU with `warmup.py` (the team's export code, FP16, dynamic batch 2), `L4T=pc` in the `.meta`; **`deploy --link virtual` runs it** (no prompt, no team check: the PC is ours). `warmup.py` now also compares the GPU name before calling an engine up to date | an engine belongs to one GPU; the virtual flight must never build it during the flight | 9.5 |
| **Code fingerprint `org.astra.code_sha`** (12 hex characters of the SHA-256 over the contents of every file that goes into layer 3: `image/context.list` minus its excludes, `image/Dockerfile`, the `/opt/astra` files). `image build` labels it and tags `astra-hil:<flavor>-<code>`; `check` row 16 compares it with the PC's code (`--pc-code-sha`) and falls back to the git commit only for images without the label; `deploy` warns with it; `deployed.env` records `CODE_SHA=` | the git commit cannot say whether an image is current: the working tree holds uncommitted changes, and the Linux copy demo.sh runs from has no `.git` (its git sha is `unknown`) | 8.1, 9.2, 15.3 |
| layer 3 takes `ARG TENSORRT=yes|no` (label `org.astra.tensorrt`, env `ASTRA_TENSORRT`, `image.env TENSORRT=` + `TENSORRT_VERSION=`); with `yes` the build fails unless `import tensorrt` works in the image | the entrypoint and check must know which detector path the image can run | 8.1, 8.2 |
| `remote_facts.sh` reports `HIL_IMAGE_CODE`, `HIL_IMAGE_TRT` and `GPU_NAME`; check row 16b judges the virtual Jetson's engine (missing, other GPU, other L4T = FAIL 19 for the legacy pipeline, fix `warmup --link virtual`); without TensorRT in the image the row is SKIP | the virtual flight is the acceptance test now | 9.2 |
| **Layer 3 installs `onnx==1.14.0` + `protobuf==3.20.3`** (the team's own pins) in TensorRT images; selftest checks `import onnx` | VERIFIED bug: layer 2's apt `python3-onnx` 1.7.0 cannot be imported next to numpy 1.26.4, so every TensorRT export (warmup, or new_od's first start) failed, on the PC and, with the same layer 2, on the board (section 17 item 15) | 8.1, 9.5, 13 |
| the virtual Jetson is never saved as the default target (`setup --link virtual` leaves `target.env` alone; a saved `JETSON_LINK=virtual` is read as `usb`) | otherwise `jetson.sh check` on the day the board is plugged in would check this PC and say ALL GOOD | 15.3 a |
| MAVROS 2.12.0: every ROS 2 Humble snapshot was listed (snapshots.ros.org, 2026-10-31): 2025-08-15 has 2.10.1, the next one (2026-01-07) has 2.14.0, the live repository 2.16.0. An exact 2.12.0 needs MAVROS built from its source tag inside layer 3 (not done: longer builds, another place to fail; owner decision D-new) | owner 15:14 lists MAVROS 2.12 among the versions to match | 12.1, 15.5 |

## 0. In one minute

- **What moves to the Jetson:** the team's flight software: the unchanged `./launcher.sh` starts MAVROS and the
  mission stack through its `TRACKER_CMD`. With the new video pipeline (the default) that is the video workflow's
  proposed field launch file (`zr10_ros2` camera control, `zr10_stream` RTSP -> `/zr10/image_raw`, waypoint_manager,
  the gps node, main_controller; owner 12:10); with the legacy photo option it is the team's own `tracker.launch.xml`
  (waypoint_manager, gps node, mission_camera_trigger, siyi = video_cam, new_od, main_controller). It runs inside
  **one Docker container named `astra-hil`**, plus the team's yolo26 detector image as a second container
  (`astra-hil-detector`) inside the same private network (6.3).
- **What stays on the PC:** Gazebo, ArduCopter SITL (it plays the Pixhawk), MAVProxy, the SIYI camera emulator (it
  plays the camera), the score report, Mission Planner.
- **The link is "an address the PC can SSH to".** USB-C cable (the Jetson is `192.168.55.1`), an Ethernet cable,
  Wi-Fi, a LAN or Tailscale: all the same to the software. **The PC opens every connection** (WSL2 sits behind NAT
  and cannot accept incoming ones). Only the Jetson's normal SSH port 22 is used.
- **Logging in (owner's rule, 11:40):** `jetson.sh login` opens a visible window. The owner types the Jetson password
  there himself. While that window stays open, every other command reuses that one login (SSH connection sharing).
  No tool ever types, stores or passes a password. A key login is an option (it lets the link come back by itself
  after a drop), never a requirement.
- **How the team code finds the "autopilot" and the "camera" with no code change:** inside the container's own
  network (and only there) the address `192.168.144.25` exists. A small relay passes the camera traffic (SDK UDP
  37260, the RTSP video stream on TCP 8554, HTTP 82) and the MAVLink through SSH forwards to the camera emulator and
  to SITL on the PC. The Jetson's own network settings are not touched, and the real camera is never addressed.
- **How the PC sees the result:** MAVLink as today (Mission Planner, the CSV flight recorder), plus a one-way,
  read-only "tap" that copies a list of ROS topics (detections, MAVROS state and position, status texts, camera
  commands) and the names of the running team nodes from the container to the PC, so `phase_monitor` and `score_run`
  work unchanged. During and after the flight, photos, logs and detections are copied into the run folder.
- **The image is built OFF the board.** The PC builds the arm64 image (through QEMU emulation) and the Jetson pulls it
  from a small registry on the PC, through the SSH login. Only changed layers move. The one thing built ON the board
  is the TensorRT engine: a separate step (`jetson.sh warmup`), done once, with the owner's OK, while the team is not
  using the board.
- **Never in the team's way:** our own names (`astra-hil`, `~/astra_hil`, label `org.astra.owner=hil`), a "team
  busy?" refusal before anything heavy, our ROS graph sealed inside the container, a disk reserve kept free for the
  team, power mode reported but never changed.
- **The same container runs on this PC** ("virtual Jetson", x86 build of the same image definition), so everything
  except the cable, the arm64 build and the Jetson GPU is tested without the board.
- **The owner's commands** (folder `simulation/jetson/`, below `J=simulation/jetson`): `bash $J/jetson.sh login`,
  `bash $J/jetson.sh check` (board found? what is missing?), once `jetson.sh image build`, `jetson.sh deploy` and
  `jetson.sh warmup`, then fly with `bash simulation/scripts/demo.sh runway --jetson usb` (or `--jetson ssh
  --jetson-host HOST`, or `--jetson virtual`). The camera pipeline is the simulator's default (the new video
  pipeline once VIDEO_PIPELINE.md's work is in; the legacy photo pipeline through the simulator's legacy option).
- **Who builds what:** the JETSON stream builds all of it, including the first-connection kit. The JETSON-SIDE builder
  delivers `login` + `check` + `setup` first (section 16).

## 1. Words used here

| Word | Meaning |
|---|---|
| Jetson | the NVIDIA Jetson Orin Nano Developer Kit (8 GB) that flies on the drone. JetPack 6.2.x = Ubuntu 22.04 + Jetson Linux (L4T) R36.4.x |
| HIL | "hardware in the loop": the real Jetson runs the flight software against the simulator |
| the PC | this Windows 11 laptop; the simulator runs inside its WSL2 distro `Ubuntu-22.04` |
| WSL NAT | WSL2's default network: Linux sits behind the Windows host, like a home network behind a router. Linux can open connections OUT; nobody outside can open one IN without extra Windows setup |
| link | how the PC reaches the Jetson: `usb` (USB-C cable, `192.168.55.1`), `ssh` (any address: Ethernet, Wi-Fi, LAN, Tailscale), `virtual` (the container on this PC, no board) |
| login window | a visible terminal window running `ssh -M -S ~/.ssh/cm-jetson USER@HOST`, where the owner types the Jetson password. It is the "control master" |
| control master / connection sharing | one SSH connection that other `ssh -S <socket>` commands reuse without logging in again. Closing the window ends all of them |
| login mode / key mode | login mode (default) = everything goes through the owner's login window. Key mode (optional) = the PC has its own SSH key on the board and opens its own two connections |
| forward | an extra channel inside an SSH connection. `-R` makes a door on the Jetson that leads to the PC; `-L` makes a door on the PC that leads to the Jetson. `ssh -S ... -O forward` adds one to a running connection, `-O cancel` removes it |
| Unix socket | a "network door" that is a file (`mavlink.sock`). Docker can hand a folder of them to a container |
| network namespace | each container's own private network: its own `lo` (loopback), its own addresses. What is added there is invisible to the Jetson itself |
| relay | a small program that copies bytes between two doors (`hil_relay.py`) |
| idle timeout | "if nothing has arrived for N seconds, the other side is gone": close and dial again |
| PING frame | a tiny frame each side sends every second, so silence really means a broken link |
| gate | the relay drops (does not keep) MAVROS's bytes while the autopilot side is silent, so old commands are never sent late |
| tap | the one-way copy of some ROS topics and of the team's node names from the Jetson container to the PC (`ros_tap.py`) |
| presence node | an empty ROS node on the PC named like a team node on the Jetson (`/jetson/main_controller`), so tools that look for node names still see it |
| team busy | the team is using the board (their programs run, or the GPU is busy): our heavy commands refuse to start |
| registry | a small image store (`registry:2` container) on the PC that the Jetson's Docker pulls from, through the SSH login |
| SERIAL2 | SITL's third serial port, TCP 5763 on the PC. It is wired like the Pixhawk's TELEM2, where the real Jetson is plugged in (commit 6d90c07) |
| image / container | an image is the packed software (like an installer); a container is one running copy of it |

## 2. What runs where, and why

| Piece | Where | Why there |
|---|---|---|
| Gazebo (world, drone, gimbal, cameras, payloads) | PC (WSL) | needs the RTX 4060 for rendering |
| ArduCopter SITL 4.6.3 | PC (WSL) | it is the Pixhawk stand-in, locked step by step to Gazebo over UDP 9002. On the drone the autopilot is a separate board too |
| MAVProxy, mission_tool, flight_recorder, Mission Planner | PC (WSL / Windows) | ground side, same as at the field |
| SIYI camera emulator (`192.168.144.25` on WSL's `lo`) | PC (WSL) | it renders the photos from Gazebo |
| score_run, phase_monitor, demo markers | PC (WSL) | they need Gazebo (payload landing points) and the emulator's photo records |
| the login window | PC (a visible window) | the owner types the Jetson password there |
| MAVROS + the mission stack (`./launcher.sh`, unchanged; `TRACKER_CMD` = the proposed field launch file with `zr10_node` + `stream_node`, or the legacy tracker.launch.xml) | **Jetson, in the container `astra-hil`** | this is what the field Jetson runs; the detector uses the Jetson GPU |
| the yolo26 detector (team image) | **Jetson, container `astra-hil-detector`**, inside `astra-hil`'s private network | the team runs it in its own image (6.3) |
| `hil_relay.py container`, `ros_tap.py send` | Jetson, same container | they give the team code its two doors (autopilot, camera) and feed the tap |
| `hil_relay.py pc-sdk` + `pc-http`, `ros_tap.py receive`, `link.sh`, `collect.sh` | PC (WSL) | the PC ends of the doors |
| the image build (arm64) and the registry | PC (WSL Docker Engine) | builds off the board (owner, 11:31) |
| the TensorRT engine build (`warmup`) | Jetson, once | an engine belongs to one GPU and one TensorRT version |

The owner's words "run the simulation with the SITL that runs on Jetson, aka the detection model runs on the
Jetson" are read as: **the detection model (and the whole team stack) runs on the Jetson; SITL stays on the PC**.
The code supports that reading: SITL is the autopilot stand-in, and the real Jetson never runs the autopilot.
Moving SITL to the Jetson would put Gazebo's lock-step physics link (UDP 9002, every physics step) on the cable.

## 3. The picture

```
PC: Windows 11
  Mission Planner + ASTRA plugin ...... UDP 14550 from MAVProxy (as today)
  LOGIN WINDOW (visible) .............. ssh -M -S ~/.ssh/cm-jetson USER@HOST   <- the owner types the password here
  USB network adapter "Remote NDIS" ... 192.168.55.100 (only with the USB-C cable)

PC: WSL2 Ubuntu-22.04 (NAT)
  gz sim <== UDP 9002 lock-step ==> arducopter SITL
  SITL TCP 5760 (serial0) -> MAVProxy -> UDP 14550 / 14555 ...                 (as today)
  SITL TCP 5763 (SERIAL2 = TELEM2, the Jetson's port) <------------------ [A] -R mavlink.sock
  camera emulator on 192.168.144.25 (WSL lo alias): UDP 37260 SDK, TCP 82 HTTP, TCP 8554 RTSP (new pipeline,
  served by the video-pipeline workflow's stream)
     ^-- hil_relay.py pc-sdk   127.0.0.1:37261  frames <-> UDP 37260 <------- [B] -R cam_sdk.sock
     ^-- hil_relay.py pc-http  127.0.0.1:37282  byte copy (+ pacing) <------- [B] -R cam_http.sock
     ^-- (ssh connects 192.168.144.25:8554 directly) <---------------------- [B] -R cam_rtsp.sock
  ros_tap.py receive: connects 127.0.0.1:7461 ---------------------------> [A] -L tap.sock
     -> republishes the allow-listed topics + presence nodes /jetson/<team node>
  score_run, phase_monitor, flight_recorder: unchanged
  link.sh adds the forwards [A] and [B] to the SSH connection(s) the PC opened:
     login mode: [A] and [B] ride on the owner's one login connection
     key mode:   [A] on connection cm-A, [B] on connection cm-B (photos never delay MAVLink)

========== SSH over TCP 22 on the Jetson (USB-C / Ethernet / Wi-Fi / LAN / Tailscale), opened by the PC ==========

JETSON (its own network untouched: eth0 192.168.144.20 on the board today, l4tbr0 192.168.55.1 over USB)
  sshd: makes the -R socket files in ~/astra_hil/sock/ and connects -L to tap.sock
  container astra-hil (--runtime nvidia --network none --cap-add NET_ADMIN, own IPC "shareable", only "lo")
    its OWN lo: 127.0.0.1 + 192.168.144.25 (the camera stand-in address, only inside)
    hil_relay.py container:  TCP 127.0.0.1:5763 -------------> mavlink.sock   [A]  (idle 4 s -> dial again)
                             UDP 192.168.144.25:37260 <------> cam_sdk.sock   [B]  (frames + PING every 1 s)
                             TCP 192.168.144.25:82 ----------> cam_http.sock  [B]
                             TCP 192.168.144.25:8554 --------> cam_rtsp.sock  [B]  (RTSP over TCP)
    ./launcher.sh hw (unchanged) with FCU_URL=tcp://127.0.0.1:5763   (the drone runs ./launcher.sh = hw too)
      mavros + TRACKER_CMD:
        new pipeline (default): the PROPOSED field launch file: zr10_node (SDK + HTTP to .25), zr10_stream
          (ffmpeg -rtsp_transport tcp rtsp://192.168.144.25:8554/main.264 -> /zr10/image_raw), waypoint_manager,
          gps node, main_controller (its detector part off: the detector runs in astra-hil-detector)
        legacy option: tracker.launch.xml UNCHANGED: waypoint_manager, gpstest, trigger, siyi, new_od, main_controller
    ros_tap.py send: listens on tap.sock, copies the allow-list + node names to the PC (one way)
  container astra-hil-detector (team's yolo26 image; --network container:astra-hil --ipc container:astra-hil)
    /zr10/image_raw -> detection/results, detection/info (same private lo and shared memory as astra-hil)
```

## 4. Every connection

"Opens it" = which side starts the connection. Every row that crosses the cable is opened by the PC.

| # | What | From | To | Transport | Opens it | Port / socket |
|---|---|---|---|---|---|---|
| 1 | The login (owner) | login window (`ssh -M` in WSL) | Jetson `sshd` | SSH over TCP | PC | Jetson TCP 22; control socket `~/.ssh/cm-jetson` in WSL |
| 2 | Commands (check, setup, start, stop, exec, status, logs, deploy) | `jetson.sh` (`ssh -S ~/.ssh/cm-jetson`) | Jetson `sshd` | inside row 1 (key mode: inside cm-A) | PC | - |
| 3 | LINK A forwards (MAVLink + tap) | `link.sh` (`ssh -S ... -O forward`) | Jetson `sshd` | inside row 1 (key mode: connection cm-A) | PC | - |
| 4 | MAVLink (autopilot) | MAVROS in the container | SITL SERIAL2 on the PC | TCP in the container -> Unix socket -> LINK A -> TCP on the PC | MAVROS dials the relay; the relay dials `mavlink.sock`; sshd + the PC's ssh dial SITL | container `127.0.0.1:5763` -> Jetson `~/astra_hil/sock/mavlink.sock` -> PC `127.0.0.1:5763+10I` |
| 5 | ROS tap (topics + node names) | `ros_tap.py send` in the container | `ros_tap.py receive` on the PC | Unix socket <- LINK A (`-L`) <- TCP on the PC | PC (`receive` connects) | PC `127.0.0.1:7461` -> Jetson `~/astra_hil/sock/tap.sock` |
| 6 | LINK B forwards (camera) | `link.sh` | Jetson `sshd` | inside row 1 (key mode: its own connection cm-B, so photos never delay MAVLink) | PC | - |
| 7 | SIYI SDK (gimbal, zoom, shutter, feedback) | video_cam in the container | emulator on the PC | UDP in the container -> frames -> Unix socket -> LINK B -> TCP -> UDP on the PC | video_cam sends; the PC opened the forward | container UDP `192.168.144.25:37260` -> `cam_sdk.sock` -> PC `127.0.0.1:37261` (`pc-sdk`) -> PC UDP `192.168.144.25:37260` |
| 8 | SIYI HTTP media API (photo list, photo download) | video_cam (legacy) and `zr10_node` (its `media_base` `http://192.168.144.25:82/cgi-bin/media.cgi`, for full-size photos) | emulator | TCP in the container -> Unix socket -> LINK B -> TCP on the PC | the client dials; the PC opened the forward | container TCP `192.168.144.25:82` -> `cam_http.sock` -> PC `127.0.0.1:37282` (`pc-http`) -> PC TCP `192.168.144.25:82` |
| 8b | RTSP video stream (new pipeline) | `zr10_stream` (ffmpeg `-rtsp_transport tcp`, `rtsp://192.168.144.25:8554/main.264`) | the simulator's RTSP stream on the PC (video-pipeline workflow) | TCP in the container -> Unix socket -> LINK B -> TCP on the PC (RTP inside the RTSP TCP connection, so no UDP) | ffmpeg dials; the PC opened the forward | container TCP `192.168.144.25:8554` -> `cam_rtsp.sock` -> PC TCP `192.168.144.25:8554` |
| 9 | Live photo mirror (during the flight) + final copy (after it) | PC `collect.sh` (`rsync -e "ssh -S ..."`) | Jetson `~/astra_hil/photos/`, `runs/<id>/` ... | inside row 1 (key mode: cm-B) | PC | - |
| 10 | Deploy: image layers | Jetson `dockerd` (`docker pull 127.0.0.1:29517/...`) | the registry container on the PC | TCP on the Jetson's loopback -> `-R 127.0.0.1:29517` inside row 1 -> PC `127.0.0.1:29517` | the PC opened the forward; the Jetson's dockerd dials its own loopback | Jetson `127.0.0.1:29517` (only during `deploy`) -> PC `127.0.0.1:29517` |
| 11 | Image build downloads | PC Docker (WSL) | nvcr.io, packages.ros.org, snapshots.ros.org, pypi.jetson-ai-lab.io, pypi.org, github.com, Docker Hub | HTTPS | PC | the PC's internet; the Jetson needs no internet |
| 12 | ROS 2 inside the container | team nodes, MAVROS, the new pipeline, the detector container, the tap sender | each other | DDS (Fast DDS, domain 77 on the board, `ROS_LOCALHOST_ONLY=1`) on the container's own `lo` and its own shared memory | - | never leaves the container (8.5) |
| 13 | ROS 2 on the PC | simulator nodes, tap receiver | each other | DDS, domain 0 | - | never leaves WSL |
| 14 | Unchanged PC wires | Gazebo <-> SITL UDP 9002; SITL TCP 5760 <-> MAVProxy; MAVProxy -> UDP 14550/14555 + the Windows host's 14550; emulator <-> Gazebo transport; POV MJPEG 8081 | | | | as in AGENTS.md 4.2 |
| 15 | REAL mode on the drone (no simulator) | MAVROS / video_cam in the container (`--network host`) | Pixhawk / real SIYI camera | serial `/dev/ttyUSB0:115200`; UDP 37260 + TCP 82 on eth0 `192.168.144.0/24` | Jetson | exactly as the native `./launcher.sh` today |
| 16 | Rescue console (fallback) | Windows terminal (PuTTY) | Jetson USB serial gadget | USB serial (COM port), 115200 8N1 | PC | only when the network part fails |

Nothing new listens on the Jetson's network: sshd on 22 as today, plus `127.0.0.1:29517` on the Jetson's loopback
only while `deploy` runs. Nothing new listens on the PC's network either: `127.0.0.1:37261`, `127.0.0.1:37282`,
`127.0.0.1:7461` and `127.0.0.1:29517` are inside WSL only (VERIFIED free today, section 17 item 10).

### 4.1 Logging in, and what it means for the link

**Login mode (the default, owner's rule 11:40).** `bash $J/jetson.sh login [--host HOST --user USER
--port N]` opens a new visible terminal window (from WSL through `powershell.exe Start-Process wsl.exe -d
$WSL_DISTRO_NAME -- bash simulation/jetson/lib/login_window.sh USER@HOST PORT`). That window runs, in the foreground:

```
ssh -M -S ~/.ssh/cm-jetson -o ControlPersist=no -o ServerAliveInterval=2 -o ServerAliveCountMax=3 \
    -o ConnectTimeout=10 -o UserKnownHostsFile=~/.astra_sim/jetson/known_hosts -o StrictHostKeyChecking=accept-new \
    -p PORT USER@HOST
```

The owner types the Jetson password there. He then sees a normal shell on the Jetson, which he may also use.
`jetson.sh login` waits up to 120 s until `ssh -S ~/.ssh/cm-jetson -O check USER@HOST` answers, then prints `OK:
logged in to USER@HOST`. `login_window.sh` copies the behaviour of the lead's reference
`D:\suas_scratch\e2e\jetson_login.sh` (same control socket path, so either window serves the kit), adds the fast
keepalive above, and, when the connection ends, says why and offers "press Enter to log in again". It never retries
by itself and never stores anything. (The reference window uses `ServerAliveInterval=30`: with it, a dropped link is
noticed on the PC after about 90 s instead of about 6 s; `check` says which keepalive the open login has.)

- Every other command runs through that login: `ssh -S ~/.ssh/cm-jetson -o ControlMaster=no ... USER@HOST CMD`.
  When the window is not open, every command stops with `[jetson] ERROR: Not logged in to USER@HOST. Likely cause:
  the login window is closed or its connection dropped. Fix: bash $J/jetson.sh login (a window
  opens; type the Jetson password there; keep it open).` and exit code 17.
- **Closing the window ends all access at once**, including the link of a running flight. (Typing `exit` in that
  shell may wait for open forwards to finish; closing the window does not.) `jetson.sh logout` does the same as
  closing it (`ssh -O exit`).
- **One TCP connection carries everything** (commands, MAVLink, camera, tap, photo mirror). A photo can then sit in
  front of MAVLink bytes. Two measures keep that small: `hil_relay.py pc-http` paces the photo download (option
  `--photo-pace auto|off|MBIT`; `auto` in login mode = 60 % of the throughput `check --speed` measured, at most
  80 Mbit/s, and never below 1.5 times what the camera profile needs, i.e. 29 Mbit/s for a8mini), and the live photo
  mirror uses `rsync --bwlimit` (default 40 % of the measured throughput). If the link is too slow for both, `check`
  says so and the flight uses `--jetson-photos after`. How long MAVLink still waits is measured, not assumed: the
  relay logs every gap over 0.5 s in the MAVLink stream, and the run summary prints the largest one (EXPECTED: a few
  ms on USB).
- **After a link drop the owner logs in again** (Enter in the window, then the password). Until then the drone
  flies on as with a silent companion computer (9.4).
- **sudo:** the tools never type a sudo password either. A step that needs sudo (docker group, power mode, clock) is
  printed for the owner to type in the login window, or is skipped with a clear note. A command may use `sudo -n`
  (works only where the team allowed passwordless sudo) and treats "a password is required" as "not available".

**Key mode (optional, the owner's choice per board).** `jetson.sh setup --key` makes `~/.astra_sim/jetson/id_ed25519`
(no passphrase) and, through the open login window, adds ONE line to the Jetson user's `~/.ssh/authorized_keys`.
That changes the team's account, so it needs the owner's and the team's OK; `jetson.sh setup --remove-key` takes the
line out again. In key mode the kit opens its own two connections with the key (`~/.astra_sim/jetson/cm-A` for
commands + LINK A, `cm-B` for LINK B + the photo mirror; `ControlPersist=yes`). Photos then never delay MAVLink, no
pacing is needed, and after a drop `link.sh` reconnects by itself every 2 s.

**Virtual:** no SSH at all; `hil_relay.py pc-local` makes the socket files itself (12).

## 5. (a) MAVLink: MAVROS on the Jetson to SITL on the PC

**Decision: keep SITL SERIAL2 exactly as today (a TCP server on 5763 inside WSL). `link.sh` adds a reverse forward
`-R ~/astra_hil/sock/mavlink.sock:127.0.0.1:5763` to the SSH connection the PC opened. Inside the container,
`hil_relay.py` listens on `127.0.0.1:5763` and passes each byte to that socket. MAVROS uses
`FCU_URL=tcp://127.0.0.1:5763`, the very URL the native simulator uses today.**

Why this and not the other ideas:

| Idea | Verdict | Reason |
|---|---|---|
| SITL `--serial2=tcpclient:<jetson>:5763` + MAVROS `tcp-l://0.0.0.0:5763` | rejected | SOURCE (ArduPilot `AP_HAL_SITL/UARTDriver.cpp`, Copter-4.6.3): a `tcpclient` port tries to connect 3 times, 1 s apart, then **SITL exits** (`exit(1)`); and after the connection closes it **never dials again**. So the Jetson would have to be ready before SITL starts, and one cable wiggle in flight would cut the Jetson off for the rest of the flight. On the drone, a serial link that comes back works again |
| MAVROS dials the PC through a Windows port proxy (`netsh interface portproxy`) | rejected | changes the owner's Windows settings, needs an inbound firewall rule, the WSL address changes at every reboot |
| UDP (SITL `udpclient:<jetson>:PORT`, MAVROS `udp://0.0.0.0:PORT@`) | plan B only | works through NAT (the PC sends first) and survives drops, but needs a published UDP port on the container, a second way in besides SSH, no encryption, and a change in `sim.launch.py`. Kept as the fallback if MAVLink through SSH ever proves too slow (13, risk 11) |
| **SITL TCP server (unchanged) + `-R` to a Unix socket + relay in the container** | **chosen** | SITL and `sim.launch.py` stay as they are; the PC opens the connection (NAT is no problem); one door (SSH 22) for everything, encrypted; SITL's TCP server takes a new client after the old one closed (SOURCE: `_check_connection()` returns at once while a client is connected and calls `accept()` again after end-of-file), so reconnecting works; MAVROS keeps today's URL |

Details the builders must keep:

- **ArduPilot routing stays as today.** MAVROS is connected to SERIAL2 exactly like the native simulator since
  commit 6d90c07, so ArduPilot forwards the team's status texts to MAVProxy, Mission Planner and the flight
  recorder. Nothing changes on the PC side of SITL.
- **Sticky MAVROS side.** The container relay keeps MAVROS's TCP connection open when the link drops. MAVROS then
  sees what it sees on the drone when the serial link is interrupted: heartbeats stop (`/mavros/state connected`
  goes false after its timeout) and come back on the same connection. Without this, MAVROS would wait for its own
  reconnect timer (SOURCE: `mavros_router.hpp`, ros2 branch: closed endpoints are retried every **30 s**).
- **Idle timeout on the autopilot side.** If no byte has come from `mavlink.sock` for **4 s**, the relay closes that
  connection and dials the socket again every 1 s. This is safe because ArduPilot sends HEARTBEAT every 1000 ms on
  every MAVLink port, whatever the stream rates are (SOURCE: `GCS_Common.cpp`, Copter-4.6.3:
  `set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT, 1000)`; HEARTBEAT is not part of any SRx_ stream).
  The idle rule is needed because the Jetson's sshd does NOT notice a pulled cable (VERIFIED, section 17 item 6), so
  the relay would otherwise wait on a dead connection forever.
- **The gate: no late commands.** MAVROS's bytes go to SITL only while the autopilot side is fresh (a byte from SITL
  within the last 2.5 s). Otherwise they are dropped and counted (`relay.status`: `MAV_DROPPED_BYTES`), and nothing
  is kept for later: a kept `set_mode` or `DO_SET_SERVO` sent after the reconnect could mean a late payload drop.
  This matches a real serial link, which loses bytes while it is broken. (Bytes already inside the SSH connection
  when it stalls can still arrive late if the connection itself recovers within ~6 s: listed in 12.1.)
- **SITL instance:** the PC end points at `127.0.0.1:5763+10*I`; the container side is always `127.0.0.1:5763`.
- **Only one client:** SITL SERIAL2 serves one connection at a time and takes the next one only after the previous
  one ended (SOURCE above). In HIL mode the PC must not start its own MAVROS (`demo.sh --jetson` never starts
  `./launcher.sh` on the PC). After a drop the old PC-side `ssh` exits (its keepalive fails), which closes its
  connection to SITL, so SITL is ready for the new one.
- **Stale socket files and the old session:** VERIFIED (section 17, item 4): sshd does not remove an old socket file
  (default `StreamLocalBindUnlink no`); the next `-R` then fails with "remote port forwarding failed". VERIFIED (item
  6): the old sshd session also stays alive after a silent drop and keeps its open channels. So on every (re)connect
  `link.sh` first kills the old session (its PID was recorded when it started; only if it is an `sshd` process of
  our own user), then removes its own socket files, then adds the forwards (exact steps in 15.3 g). We do NOT change
  the Jetson's `sshd_config`.
- **launcher.sh mode:** the container runs `./launcher.sh hw` with `FCU_URL=tcp://127.0.0.1:5763`, i.e. the drone's
  own mode (`./launcher.sh` with no argument = hw). launcher.sh's `hw` and `sim` branches differ only in the default
  `FCU_URL` and the printed mode label (SOURCE: launcher.sh lines 28-34, 65-71); with `FCU_URL` set they behave the
  same. The container's own mode variable is `ASTRA_HIL_MODE`, never `ASTRA_MODE`, because launcher.sh reads
  `ASTRA_MODE` itself (line 28); the entrypoint also unsets `ASTRA_MODE` before calling launcher.sh.

## 6. (b) The camera: 192.168.144.25 inside the container only

**Decision: the container runs with `--network none --cap-add NET_ADMIN`. At start its entrypoint adds
`192.168.144.25/32` to the container's OWN loopback. `hil_relay.py` listens there on UDP 37260 and TCP 82 and passes
the traffic through LINK B to the emulator on the PC. video_cam's `CAMERA_IP` stays `192.168.144.25`.**

- **No change to the team code, no change to the Jetson's network settings.** The address exists only inside the
  container's private network. VERIFIED (section 17, items 1-3): a `--network none` container got
  `192.168.144.25/32` on its own `lo`; a non-root process (uid 1000) bound UDP 37260 and TCP 82 on it; a SIYI shutter
  packet and an HTTP request got their answers; and a relay on `192.168.144.25:82` reached a Unix socket made on the
  host with mode 0600 (what sshd makes), as container root and as uid 1000.
- **Safety guard for the real drone:** before it adds the address, the entrypoint checks that the container's only
  network interface is `lo` (`ls /sys/class/net`). If there is any other (the container was started by mistake with
  `--network host` or a Docker network), it refuses: `[entrypoint] ERROR: refusing to add 192.168.144.25: this
  container has network interfaces besides lo (eth0). Likely cause: it was started with --network host or on a Docker
  network. Adding the camera's address there could hide the real camera on the drone. Fix: start it with
  bash $J/jetson.sh start --sim (it uses --network none).` (exit 20). With `--network host` and
  `NET_ADMIN`, the address would otherwise land on the Jetson's real loopback.
- **Why not a Docker network carrying 192.168.144.0/24 on the Jetson?** It would add a route for `192.168.144.0/24`
  on the Jetson host, the same subnet as eth0 on the drone (Jetson `.100`, camera `.25`). That changes the Jetson's
  routing and can hide the real camera. The private-loopback way changes nothing outside the container.
- **HTTP (TCP 82)** is a plain byte copy: one forwarded connection per HTTP connection. On the PC the forward ends at
  `hil_relay.py pc-http` (`127.0.0.1:37282`), which connects to `192.168.144.25:82` (the emulator on WSL's `lo`) and
  can pace the photo bytes (4.1). A pipe with no bytes in either direction for 15 s is closed (cleanup only;
  video_cam's own timeouts are shorter).
- **SDK (UDP 37260)** must cross a byte stream, so each datagram is packed into a small frame (15.3 f). The container
  end remembers which program sent it (a 1-byte client number), so replies (0x0B feedback, angles, the model answer
  0x02) go back to the right sender. The PC end (`pc-sdk`) sends each datagram to the emulator from one UDP socket
  per client number and packs the replies. **Both ends send a PING frame every 1 s; 4 s without any frame = the link
  is gone:** the container end closes and dials `cam_sdk.sock` again, the PC end closes and waits for the next
  connection.
- **`launcher.sh` pings `CAMERA_IP`** (warning only). Inside the container the address is local, so the ping
  answers. In REAL mode it pings the real camera as today.

### Bandwidth and latency

**New video pipeline (the field default):** the RTSP H.264 stream at 1280x720 and 2 frames per second (the team's
setting, live probe 11:47) needs only a few Mbit/s (EXPECTED, not measured; `check --speed` and the run summary
record the real rate). It fits every link in the table below, Wi-Fi included. Full-size photo downloads by
`zr10_node` (HTTP 82) come on top only when the team code asks for them.

**Legacy photo pipeline:** VERIFIED (section 17, items 5 and 9): with the `zr10` profile, simulated photos are
2560x1440 JPEG, **1.27-1.37 MiB** each (average 1322 KiB), **93-124 per runway flight**, about one per second: **~11
Mbit/s** from the PC to the Jetson during the search lanes. With **`--camera a8mini`** (3840x2160) one earlier runway
run had 95 photos of **2.24-2.49 MiB** (average 2354 KiB): **~19 Mbit/s** at one photo per second. The camera the
team's code drives now is the ZR10 (`zr10_ros2`, owner 12:00), so legacy HIL runs use `zr10` (its real 2K photo
size); `a8mini` stays available as the heavier load test. video_cam polls the photo list every 0.5 s for up to 15 s
and allows 10 s per download (AGENTS.md 3.3), and keeps at most 5 triggers waiting (the oldest is dropped).

| Link | Expected throughput | Expected round trip | Verdict for the legacy photo load (zr10 11 Mbit/s, a8mini 19 Mbit/s down, the same again up with the live mirror) |
|---|---|---|---|
| USB-C (USB network gadget, RNDIS on Windows) | 100-300 Mbit/s (EXPECTED; `check --speed` measures it) | < 1-2 ms | plenty |
| Ethernet cable PC <-> Jetson | ~900 Mbit/s | < 1 ms | plenty |
| Wi-Fi 5 GHz, good signal | 50-300 Mbit/s | 2-10 ms, spikes 50-200 ms | OK with `--photo-pace auto`; below ~80 Mbit/s measured: `--jetson-photos after` |
| Wi-Fi 2.4 GHz / busy campus Wi-Fi | 5-40 Mbit/s | 5-30 ms, spikes > 200 ms | not enough for a8mini: photos queue and video_cam drops some; zr10 may work with `--jetson-photos after`. (The new pipeline's stream is EXPECTED to fit) |

In key mode, MAVLink and the camera use two separate SSH connections, so a photo never sits in front of MAVLink
bytes. In login mode they share one connection and the pacing of 4.1 applies. MAVLink itself needs only a few kbit/s
(the gps node sets every stream to 1 Hz).

### (b2) Camera modes

| Mode | Where | How video_cam reaches "the camera" | Status |
|---|---|---|---|
| **SIM** | HIL: Jetson + PC simulator, or the virtual Jetson | `192.168.144.25` on the container's own `lo` -> `hil_relay.py` -> LINK B -> emulator on the PC | **build now** |
| **REAL** | on the drone, no simulator | container with `--network host`: video_cam talks straight to the real camera at `192.168.144.25` on eth0, MAVROS to `/dev/ttyUSB0:115200`. No relay, no tap, no extra address. The native `./launcher.sh` (no Docker) keeps working exactly as today | **must keep working** (the container REAL mode is new and optional) |
| MIRROR | HIL with the real camera on the Jetson's eth0 | photos from the simulator, gimbal/zoom commands copied to the real camera | **LATER, not built** (section 14) |

The camera path is its own replaceable piece: its own forwards (LINK B), its own socket names, its own relay roles
(`container` SDK/HTTP/RTSP part, `pc-sdk`, `pc-http`). Swapping it (for a mirror, or for the real camera) touches
nothing in the MAVLink path.

### 6.3 The new video pipeline (the field default since owner entries 11:56-12:04)

What the team runs on the board now (live probe 11:47; sources in `D:\suas_scratch\astra_new_pipeline\20261031-1151\`,
read-only; SOURCE where quoted), next to the unchanged launcher.sh stack:

| Piece | What it does | How it reaches the camera | What HIL does with it |
|---|---|---|---|
| `zr10_ros2` `zr10_node` (C++) | camera control: gimbal, zoom, photos (SIYI SDK) | UDP `192.168.144.25:37260` (`camera_port: 37260`) and HTTP `http://192.168.144.25:82/cgi-bin/media.cgi` (`media_base`) (SOURCE: `config/zr10.yaml`) | runs in `astra-hil`; its SDK and HTTP go through the relay exactly like video_cam's (6) |
| `zr10_stream` `stream_node` (Python + ffmpeg) | decodes the camera's video to `/zr10/image_raw` (`sensor_msgs/Image`, rgb8, 1280x720; the team runs 2 fps) | `ffmpeg ... -rtsp_transport tcp -i rtsp://192.168.144.25:8554/main.264` (SOURCE: `scripts/stream_node.py` line 41): RTSP with the video inside the same TCP connection | runs in `astra-hil`; the relay listens on `192.168.144.25:8554` and passes the connection to `cam_rtsp.sock` -> LINK B -> the simulator's RTSP stream at `192.168.144.25:8554` on the PC. A plain byte copy, like HTTP; no UDP is needed because of `-rtsp_transport tcp` |
| yolo26 detector (`astra_live_detection` `live_node`, in the team's Docker image `yolo26-ros-subscriber`, `--network host`, domain 0 on the board) | reads `/zr10/image_raw`, publishes `detection/results` (`vision_msgs/Detection2DArray`), `detection/annotated` (`Image`), `detection/info` (`String`) (SOURCE: `live_node.py` lines 59-61) | none (ROS only) | runs as a second container `astra-hil-detector` that JOINS `astra-hil`'s network and IPC namespaces (`--network container:astra-hil --ipc container:astra-hil`), so it shares the private `lo` and shared memory and nothing else; same user id as the stack (`-u $ASTRA_UID:$ASTRA_GID`, because of the shared-memory trap of 8.6); our name and label; the same `ROS_DOMAIN_ID`/`ROS_LOCALHOST_ONLY` as `astra-hil` |

Consequences for the rest of the design:

- **Relay:** one more TCP listener in the container (`192.168.144.25:8554` -> `cam_rtsp.sock`), one more forward on
  LINK B (`-R ~/astra_hil/sock/cam_rtsp.sock:192.168.144.25:8554`, which ssh connects directly; no pacing needed).
  `pc-local` (virtual) does the same. Contract: 15.3 c, d, g.
- **IPC:** `astra-hil` is started with `--ipc shareable` instead of the default `private`, so the detector container
  can join it. It is still its own namespace, separate from the Jetson's (SOURCE: Docker `--ipc` reference:
  "shareable: own private IPC namespace, with a possibility to share it with other containers"; VERIFIED only for
  `private`, item 7). The network stays `none`: the detector shares `astra-hil`'s `lo`, nothing else.
- **What the container runs (owner entry 12:10):** the team has no launch file for the new pipeline yet (on the
  board its pieces are started by hand, while the boot-time launcher still starts the obsolete photo nodes). The
  video-pipeline workflow writes a PROPOSED field launch file under `simulation/video/proposals/` (never inside the
  team packages): `zr10_node`, the `zr10_stream` launch, the yolo26 detector as the team runs it, waypoint_manager,
  the gps node and main_controller, WITHOUT the obsolete photo nodes, started by launcher.sh through its documented
  `TRACKER_CMD`. The twin's new-pipeline configuration runs that file, so HIL does too: the entrypoint runs
  `./launcher.sh hw` unchanged with `TRACKER_CMD` = that proposed launch file (the image copies
  `simulation/video/proposals/`). "The team has not adopted it yet" is a difference from the field (12.1).
- **The detector inside HIL:** if the proposed launch file starts the detector with `docker run ...` (an
  `ExecuteProcess`), that cannot run inside `astra-hil` without giving the container the Jetson's Docker socket
  (= root on the board), which this design refuses. So `jetson.sh start` starts the detector container itself (same
  image, same command and flags, except `--network container:astra-hil --ipc container:astra-hil` instead of
  `--network host`), and the launch file is started with its detector part switched off. **Request to the video
  workflow:** give the proposed launch file an argument for that (e.g. `detector:=external`). The exact commands, the
  detector image (the team's image as it is on the board, or a copy built off the board from the team's Dockerfile)
  and the packaging come from `docs/simulation/VIDEO_PIPELINE.md` and the proposals' README, so the virtual Jetson and
  the board run the same thing.
- **Tap:** carries the new detector topics (7); never `/zr10/image_raw` or `/detection/annotated` (images).
- **Scoring:** score_run, phase_monitor and the recorder must learn the new pipeline's outputs (owner 12:04); that is
  the video-pipeline workflow's and the PC tools' work. This design only guarantees that those topics reach the PC
  through the tap. Note for them: `stream_node` stamps frames with the Jetson's publication time, "not camera
  exposure time" (SOURCE: STREAMING.md), so the PC tools should use arrival time or the clock offset of 11.
- **Open question (owner 12:04, not invented here):** how the yolo26 detections reach the mission logic
  (main_controller's hold, target waypoints, drops). If nothing in the team code connects them yet, the HIL flight
  can show detections but no detection-driven drops; the run summary says so plainly.
- **GPU first run:** the legacy `new_od` needs its TensorRT `.engine` (9.5 warmup). Whether the yolo26 ONNX detector
  needs a first-run step on the GPU (e.g. a TensorRT cache) is not known yet; if it does, it goes into `warmup` with
  the same rules (once, the owner's OK, the team idle).
- **Legacy photo pipeline:** unchanged design (video_cam + HTTP downloads + photo folder + `new_od` + collect.sh's
  photo mirror and time correction), kept as an option.

## 7. (c) How the PC sees what the Jetson does

The PC's tools need: status texts, flight state and position, detections, camera commands, photos, logs, and which
team nodes are alive. Plan, in order of preference:

1. **MAVLink on the PC (already there, nothing new).** Every status text of the team code reaches the PC through
   ArduPilot's routing from SERIAL2 (row 4 of section 4). Mission Planner shows them; `sim_demo/flight_recorder.py`
   writes the CSV files from MAVLink only (SOURCE: its docstring says it was made for the case "when the team's code
   runs on the real Jetson (the PC then cannot see the Jetson's ROS topics)"). `mission_tool` arms and starts the
   mission over MAVLink as today.
2. **Readiness check on the Jetson.** `stack_check` needs the team's node names and two services
   (`/waypoint_manager/get_parameters`, `/sahi/get_health`). So the PC runs it INSIDE the container: `jetson.sh exec
   -- ros2 run sim_bringup stack_check --timeout 300 --settle 20 --need-detector` (sim_bringup is built into the
   image; it is pure Python). The exit code comes back over SSH. The same way, demo.sh's "look down" command becomes
   `jetson.sh exec -- ros2 topic pub -w 1 --once /camera/command std_msgs/msg/String "{data: 'gimbal_set_angles
   0,-90'}"`. **`exec` always runs as the stack's own user** (`docker exec -u astra-dev -e HOME=/home/astra-dev`):
   VERIFIED (section 17 item 7), Fast DDS between root and another user in the same container delivered NOTHING in
   10 s, in both directions, while the same user got the message at once. A ROS command run as root would not see the
   team stack.
3. **The ROS tap: a one-way, read-only copy of an allow-list of topics, Jetson -> PC.** It is needed because
   `score_run` (the score report) and `phase_monitor` (the live phases in the terminal) are ROS nodes that must stay
   on the PC (they need Gazebo's payload landing points and the emulator's photo records). Direction: **only
   Jetson -> PC**. The PC never publishes into the Jetson's ROS graph, so the Jetson's software sees exactly what it
   sees on the drone (where no PC ROS exists).

   **Allow-list (contract; derived from what the PC tools subscribe to: `ros_common.FlightData` lines 103-118, which
   score_run uses, phase_monitor lines 299-319, and score_run's detection subscription).** The tap copies each
   publisher's own QoS, so the PC side matches it the way it does today:

   | Topic | Type | Used on the PC by |
   |---|---|---|
   | `/image_detection` | `interfaces/msg/ImageResult` | score_run, phase_monitor |
   | `/mavros/state` | `mavros_msgs/msg/State` | FlightData (score_run), phase_monitor |
   | `/mavros/extended_state` | `mavros_msgs/msg/ExtendedState` | phase_monitor |
   | `/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | FlightData (photo geometry) |
   | `/mavros/global_position/rel_alt` | `std_msgs/msg/Float64` | FlightData, phase_monitor |
   | `/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | FlightData |
   | `/mavros/vfr_hud` | `mavros_msgs/msg/VfrHud` | FlightData (ground speed, climb) |
   | `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | FlightData |
   | `/mavros/mission/reached`, `/mavros/mission/waypoints` | `mavros_msgs/msg/WaypointReached`, `WaypointList` | FlightData |
   | `/mavros/statustext/recv`, `/mavros/statustext/send` | `mavros_msgs/msg/StatusText` | FlightData, phase_monitor (team texts are on `send`) |
   | `/camera/command` | `std_msgs/msg/String` | FlightData (zoom and gimbal history; published by the team code, e.g. `main/src/capture_detect_drop.py`, and by the look-down command) |
   | `/camera/status`, `/camera/disk_free_mb` | `std_msgs/msg/String`, `std_msgs/msg/Float64` | FlightData, operator view |
   | `/detection/results` (new pipeline) | `vision_msgs/msg/Detection2DArray` | the PC tools, once they learn the new pipeline (6.3) |
   | `/detection/info` (new pipeline) | `std_msgs/msg/String` | the same |
   | the `zr10_node` status topics (new pipeline; names from VIDEO_PIPELINE.md) | small messages only | operator view, scoring |

   `tap_topics.txt` has three parts (`[common]`: MAVROS and status texts; `[new]`; `[legacy]`: `/image_detection`,
   `/camera/*`), and the entrypoint passes the parts of the pipeline that runs. It is the one place to add a topic.

   Not copied: the emulator's `/sim/camera/*` topics (they are made on the PC already), `/zr10/image_raw` and
   `/detection/annotated` (raw and annotated video frames: 1280x720 rgb8 is ~2.7 MB per frame), `/image_raw` and
   `/sahi_detection_results` (full images), `/parameter_events`, `/rosout`, anything else. Size: well below
   50 kbit/s (EXPECTED; the receiver logs the real rate). The PC end is a node named `jetson_tap`; score_run's "who
   publishes detections" field will show that name (the PC builder checks what the report prints). score_run and
   FlightData stamp messages with the PC's own clock when they arrive (SOURCE: `time.time()` in the callbacks), so the
   Jetson's clock does not enter here; the tap adds only its transport delay.

   **Node names.** A topic copy cannot carry node names, and phase_monitor's warning "main_controller is NOT RUNNING
   any more" looks for the name `main_controller` in the PC's ROS graph (SOURCE: `get_node_names_and_namespaces()`,
   phase_monitor line 340). So the sender also sends the container's node list every 2 s (frame NODES, 15.3 e), and
   the receiver keeps one empty **presence node** per team node in the namespace `/jetson` (`/jetson/main_controller`,
   `/jetson/new_od`, ...; the list is stack_check's `TEAM_NODES` plus `mavros`, plus the new pipeline's node names
   from VIDEO_PIPELINE.md). phase_monitor compares names only,
   so its warning works unchanged. A presence node is removed only after two node lists in a row without that name;
   during a link drop the presence nodes stay (the state is unknown, and a drop is reported separately), so a drop
   never raises a false "crashed" warning. The PC builder greps the PC tools for other node-name checks and lists
   them.

   **Why not zenoh-bridge-ros2dds or Fast DDS over TCP?** zenoh-bridge-ros2dds "relies on CycloneDDS and has been
   tested with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`" (SOURCE), while the team stack runs Fast DDS, the Humble
   default; switching the container's RMW would make it differ from the drone. Fast DDS TCP needs XML profiles for
   every node on both sides. The tap is a few hundred lines of plain rclpy (`raw=True` subscriptions + publishing the
   serialized bytes), uses the same SSH door, and is easy to test.
4. **Photos (legacy photo pipeline, and any photo files `zr10_node` saves).** score_run and phase_monitor also watch
   the team's photo folder. During the flight `collect.sh
   --follow` mirrors the Jetson's photo folder (JPEG + JSON sidecar) into the PC's photo folder (`python3 -m
   sim_bringup.sim_paths photo-dir`) every 3 s (`--jetson-photos live`, the default for USB and Ethernet). It copies
   into a staging folder first, **corrects each file's modification time by the measured clock offset (11)**, then
   moves the sidecar and then the JPEG into the watched folder (a rename keeps the corrected time, and the watcher
   never sees a half-written file). On slow Wi-Fi, `--jetson-photos after` copies them only after the landing (the
   live report then has no per-photo table until `score_run --rescore`; the PC builder checks what rescore can fill in
   and documents it).
5. **After the flight: `collect.sh` copies everything into the run folder** (15.3 b lists the names): photos +
   sidecars to `RUN_DIR/photos/` (times corrected the same way) and annotated detections to `RUN_DIR/detections/` (as
   today), the container's launcher output to `RUN_DIR/team_stack.log` (same name as a PC run, so every existing tool
   finds it), and the Jetson's extras to `RUN_DIR/jetson/` (container log, ROS logs, relay and tap logs, the board's
   facts, the power mode, `tegrastats` samples taken during the flight, the clock offset, the link log, the run
   summary).

## 8. (d) The container

### 8.1 One image definition, built on the PC for two machines

The team stack image has three layers. Layer 2 is the EXISTING `docker/Dockerfile` target `dev` (`astra-dev`, "= the
Jetson's software"), which already takes `BASE_IMAGE` and `TORCH_INDEX_URL` as build arguments (SOURCE: lines 42
and 93 of the current file). **No second copy of the ROS / MAVROS / Python stack is written.** All layers are built on
the PC; the arm64 ones through QEMU emulation (9.5).

| Layer | x86 build (virtual Jetson on this PC) | arm64 build (real Jetson), built on the PC | Defined in |
|---|---|---|---|
| 1. base OS + GPU libraries + ROS 2 Humble | `ros:humble-ros-base-jammy` (already on this PC) | `astra-hil-l4t-ros:r36.4.0` = `nvcr.io/nvidia/l4t-jetpack:r36.4.0` (CUDA, cuDNN, TensorRT, VPI inside the image, 5.22 GB compressed: SOURCE, NGC; Ubuntu 22.04 with CUDA 12.6, cuDNN 9.3, TensorRT 10.3: EXPECTED, printed by `selftest`) + ROS 2 Humble ros-base from packages.ros.org (arm64) + rosdep + colcon | new `simulation/jetson/image/Dockerfile.l4t-ros` |
| 2. the team's dev stack | `astra-dev:latest` (`build_images.sh dev`, torch from `download.pytorch.org/whl/cu126` for the RTX 4060) | `astra-dev:jetson` = `build_images.sh dev --platform linux/arm64 --base astra-hil-l4t-ros:r36.4.0 --torch-index https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/ --tag jetson`: torch 2.8.0 / torchvision 0.23.0 Jetson builds, numpy < 2, ultralytics, sahi, MAVROS from the snapshot, mavlink-routerd | existing `docker/Dockerfile` (unchanged) + PATCH P1 to `simulation/docker/build_images.sh` (15.6) |
| 3. the team stack, built in | `astra-hil:virtual` | `astra-hil:jetson` (also tagged `astra-hil:jetson-<git sha>`) | new `simulation/jetson/image/Dockerfile` (`ARG DEV_IMAGE`) |

Layer 3 makes the image **self-contained**, so "putting the container on the Jetson" is all it takes (no repo
checkout, no colcon on the Jetson):

- the repo is laid out at **`/home/astra-dev/astra`**, the drone's own layout, so new_od's default `model_path`
  `/home/astra-dev/astra/ros2_ws/src/yolo26m.pt` is right without any override;
- copied in: `launcher.sh`, `scripts/humanize.sh`, and `ros2_ws/src/` packages `main video_cam detection mapping
  waypoint_mavros wp_sender gps_ros2 interfaces simulation/sim_bringup` (list in `image/context.list`), plus the new
  pipeline's packages `zr10_ros2` and `zr10_stream` (and `ffmpeg` from apt) from wherever VIDEO_PIPELINE.md takes them
  (the team keeps them outside this repo: `~/astra_ws` on the board, branch `video-cam` of the team ASTRA repo) and the
  proposed field launch file from `simulation/video/proposals/`; never
  copied: `*.pt *.onnx *.engine` (the force-committed `ros2_ws/src/yolo26m.engine` was built for one board's
  TensorRT; an engine must be built on the board that runs it), photos, build outputs, `.git`;
- `colcon build` (copy install, not symlink) of exactly those packages;
- `ros2_ws/src/yolo26m.pt` and `yolo26m.engine` are symlinks into `/models` (a folder on the Jetson, mounted at run
  time), so the weights and the board-built engine live outside the image;
- layer 2's user (`ubuntu`, SOURCE: `ARG USERNAME=ubuntu`) is renamed to `astra-dev` with home `/home/astra-dev`;
  the entrypoint gives it the Jetson user's ids at start (8.6);
- `/opt/astra/`: `entrypoint.sh`, `in_stack.sh`, `selftest.py`, `warmup.py`, `hil_relay.py` (from
  `simulation/jetson/lib/`), `ros_tap.py` and `tap_topics.txt` (from `simulation/jetson/pc/`), `paths.env`,
  `image.env`;
- labels `org.astra.owner=hil`, `org.astra.git_sha`, `org.astra.code_sha` (the code fingerprint, revision 4),
  `org.astra.tensorrt` (`yes` / `no`), `org.astra.built`, `org.astra.flavor` (`jetson` / `virtual`).

**Pinned versions for the arm64 build.** The Jetson torch wheel is pinned by its sha256, so a wheel re-published
under the same version number cannot change the image without anyone noticing (an earlier wheel with the same
version needed `libcudss.so.0`, which JetPack 6.2 lacks; SOURCE: reviewer's check of the NVIDIA forum). The current
`torch-2.8.0-cp310` aarch64 wheel's sha256 starts with `62a1beee9f2f1470` (reviewer's reading, 2026-10-31); builder 1
downloads it once on the PC, records the full hashes of torch and torchvision in
`simulation/jetson/image/jetson_pins.env`, and passes them through PATCH P2 (15.6). Its libraries (cudart, cusparse,
cufft, cufile, curand, cublas/cublasLt, cudnn9) are all in l4t-jetpack (reviewer's check; `selftest` proves it with
`import torch`).

**Diagnostic only: a CPU build on the board.** For "is the board connected and does the stack start at all", the
kit may run layer 2 built on the plain `ros:humble-ros-base-jammy` (multi-arch) with CPU torch and
`TRACKER_CMD=... team_stack.launch.py detector:=real ...`. **That is a diagnostic step, never the deliverable**: a
Jetson flight on the CPU does not count (owner 11:16 item 2). On the board the image is the L4T/TensorRT one above,
and `TRACKER_CMD` is either unset (legacy: the team's own tracker.launch.xml) or the proposed field launch file of the
new pipeline (8.2), never a simulator override.

### 8.2 Same entrypoint, same settings

`ENTRYPOINT ["/opt/astra/entrypoint.sh"]` (with `docker run --init`). Modes (first argument): `sim` (default),
`real`, `shell`, `selftest`, `warmup`. The team stack is ALWAYS started by `./launcher.sh` itself.

| Variable | Default | Meaning |
|---|---|---|
| `ASTRA_HIL_MODE` | `sim` | `sim` = HIL (address + relays + tap), `real` = on the drone (host network, serial). Not `ASTRA_MODE`: launcher.sh reads that one itself |
| `ASTRA_FLAVOR` | from the image label | `jetson` or `virtual`; picks the detector command below |
| `FCU_URL` | `sim`: `tcp://127.0.0.1:5763`; `real`: unset = launcher's own `/dev/ttyUSB0:115200` | MAVROS connection (launcher.sh) |
| `CAMERA_IP` | `192.168.144.25` | camera address (launcher's ping; the relay's listen address in `sim`) |
| `TRACKER_CMD` | **new pipeline (default), both flavors:** the video workflow's proposed field launch file with its detector part off (6.3, owner 12:10). **Legacy photo pipeline:** `jetson`: **unset** = the team's `ros2 launch main tracker.launch.xml` (TensorRT engine, `use_batched_inference` true, as the team ships it); `virtual` with TensorRT in the image (the default, revision 4): **unset** too, with this PC's own engine in `/models`; `virtual` built with `--x86-base ros` (no TensorRT): `ros2 launch sim_bringup team_stack.launch.py detector:=real model_path:=/models/yolo26m.pt` (the simulator's only override) | mission stack command (launcher.sh, which stays unchanged) |
| `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY` | `sim`: `77` and `1` (board and virtual alike); `real`: the team's domain (`0` on the board today) and unset | 8.5 |
| `ASTRA_RUN_ID` | `manual-<time>` | run folder name under `/data/runs/` (the PC passes `<stamp>_<world>`) |
| `ASTRA_TAP` | `1` in `sim`, `0` in `real` | start `ros_tap.py send` |
| `ASTRA_UID`, `ASTRA_GID` | `1000` | the ids the in-container user `astra-dev` gets (= the Jetson SSH user, so the photo files belong to them) |
| `ASTRA_RAW_TIME` | unset | passed to launcher.sh |

`sim` start, step by step: (1) refuse unless `lo` is the only interface (6); (2) check `/run/astra_hil` is there;
(3) as root, add `${CAMERA_IP}/32` to `lo` (error with the fix "start it with --cap-add NET_ADMIN (jetson.sh start
does)" if not allowed); (4) give `astra-dev` the ids `ASTRA_UID`/`ASTRA_GID`; (5) from here on everything runs as
`astra-dev` (`setpriv --reuid --regid --init-groups`): start `hil_relay.py container` (log
`/data/runs/<id>/relay.log`; binding port 82 as a normal user works because `ip_unprivileged_port_start` is 0 inside,
VERIFIED item 1), start `ros_tap.py send`, then `cd /home/astra-dev/astra && env -u ASTRA_MODE
FCU_URL=tcp://127.0.0.1:5763 ./launcher.sh hw 2>&1 | tee -a /data/runs/<id>/team_stack.log`. On SIGTERM: pass it to
launcher.sh (it stops MAVROS and the stack), stop the relay and the tap, exit 0.

`selftest` prints and checks: `torch.cuda.is_available()`, `import tensorrt` and its version (jetson flavor), numpy <
2, the OpenCV versions it finds (Python `cv2` and the `libopencv*` packages; see risk 17), `ros2 pkg prefix` for the 9
packages + mavros, the photo folder the team code resolves (`python3 -m sim_bringup.sim_paths photo-dir`), that
`/models/yolo26m.pt` exists, and whether `/models/yolo26m.engine` + `.meta` match this board.

`warmup` builds the TensorRT engine with the team's own `detection.model_manager.resolve_model_path` and new_od's
default arguments (slice 640, workspace 1, `max_batch` = `batch_size` 2 because batched inference is on), writes
`/models/yolo26m.engine` and `yolo26m.engine.meta` (TensorRT version, L4T release, GPU name, time). It runs its own
work under `nice -n 19 ionice -c 3` INSIDE the container (a `nice` in front of `docker run` would not help: the
daemon starts the container, not the CLI). The GPU part cannot be made "low priority", which is why warmup runs only
when the team is not using the board (9.6) and only with the owner's OK (9.5).

### 8.3 Run commands per mode (what `jetson.sh` types)

`D` = the Jetson data folder `~/astra_hil` (absolute path saved by `setup`). `W` = `/home/astra-dev/astra/ros2_ws`.
Every container carries `--label org.astra.owner=hil`; our commands only ever stop or remove containers and images
with that label.

```
# SIM (HIL) on the Jetson
docker run -d --init --name astra-hil --label org.astra.owner=hil --runtime nvidia --network none --ipc shareable \
  --cap-add NET_ADMIN --shm-size 1g --memory 5g --cpus 5 \
  -e ASTRA_HIL_MODE=sim -e ASTRA_RUN_ID=$RUN_ID -e ASTRA_UID=$(id -u) -e ASTRA_GID=$(id -g) \
  -e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1 \
  -v $D/sock:/run/astra_hil -v $D/models:/models -v $D/runs:/data/runs \
  -v $D/photos:$W/src/video_cam/mapping_photos -v $D/detections:$W/src/video_cam/detection_results_sahi \
  astra-hil:jetson sim

# the new pipeline's detector, inside astra-hil's private network and IPC (image and command per VIDEO_PIPELINE.md)
docker run -d --init --name astra-hil-detector --label org.astra.owner=hil --runtime nvidia \
  --network container:astra-hil --ipc container:astra-hil -u $(id -u):$(id -g) \
  -e ROS_DOMAIN_ID=77 -e ROS_LOCALHOST_ONLY=1 <detector image> <the team's detector command>

# REAL on the drone (no simulator): like the native ./launcher.sh, with the TEAM'S OWN folders
docker run -d --init --name astra-hil-real --label org.astra.owner=hil --runtime nvidia --network host --ipc host \
  --device /dev/ttyUSB0 --group-add dialout -e ASTRA_HIL_MODE=real -e ASTRA_UID=$(id -u) -e ASTRA_GID=$(id -g) \
  -v $D/models:/models -v $D/runs:/data/runs \
  -v $TEAM_ROOT/ros2_ws/src/video_cam/mapping_photos:$W/src/video_cam/mapping_photos \
  -v $TEAM_ROOT/ros2_ws/src/video_cam/detection_results_sahi:$W/src/video_cam/detection_results_sahi \
  astra-hil:jetson real

# VIRTUAL on this PC (WSL Docker Engine): SIM mode, x86 image, the PC GPU
docker run -d --init --name astra-hil-virtual --label org.astra.owner=hil --gpus all --network none \
  --cap-add NET_ADMIN --shm-size 1g ... -v ~/.astra_sim/jetson_virtual/sock:/run/astra_hil ... astra-hil:virtual sim

# WARMUP on the Jetson (once; 9.5)
docker run --rm --init --name astra-hil-warmup --label org.astra.owner=hil --runtime nvidia --network none \
  --cpus 2 --memory 4g -e ASTRA_UID=$(id -u) -e ASTRA_GID=$(id -g) -v $D/models:/models astra-hil:jetson warmup
```

- The memory and CPU limits in SIM leave room for the board itself (7.6 GB shared by CPU and GPU, ~5.5 GB free at idle
  per the owner's guide); the builder tunes them with `tegrastats` on the first flight and notes the numbers.
- **REAL mode uses the team's own photo and detection folders** (`TEAM_ROOT`, default `/home/<user>/astra`, option
  `--team-root DIR`), so operators find the photos where they always look and the team's `killnode.py` folder moves
  work as today. (SIM never touches the team's folders: it uses `~/astra_hil/photos`.) The builder confirms the
  folder list with `sim_paths photo-dir` and the team code (`camera_feed` too, if video_cam uses it).
- The exact photo folder path inside the image (`$W/src/video_cam/mapping_photos` per AGENTS.md 3.3) is confirmed by
  the build with `sim_paths photo-dir` and written into `/opt/astra/paths.env` (15.3 b).
- REAL mode refuses to start while a native `launcher.sh`/MAVROS or any other ROS 2 process of the team runs on the
  Jetson (both would open `/dev/ttyUSB0`, and both would be on the same DDS domain), and refuses when the photo folder
  is not empty unless `--archive-photos` or `--keep-photos` is given (old photos would be re-processed and could send
  the drone to an old target; the owner picks the default, 15.5).

### 8.4 How it relates to the docker_split profile and the compose "jetson" container

Today `docker/compose.sim.yml` has a "jetson" container (image `astra-dev`, repo mounted, colcon at start,
`./launcher.sh sim` at `192.168.144.100` on the Docker network `astra_net`, MAVLink from MAVProxy on UDP 14552).
**Converge, do not duplicate:**

- the HIL image is built FROM the same `astra-dev` target (layer 2), so versions match by construction;
- the HIL container and the compose container are two ways to run the same software: compose = both containers on
  one PC on a fake drone network (good for CI); HIL = the stack on a real board through SSH. They are not merged now
  (the docker stream owns those files);
- later, when the docker stream is done, the compose "jetson" service can use `astra-hil:virtual` (no colcon at start)
  and `docker_split.yaml` can point at it. This is a suggestion for the docker stream, not part of this work.

### 8.5 ROS 2 isolation and ROS_DOMAIN_ID (owner entry 11:22)

- **SIM (HIL and virtual): sealed, and on its own domain anyway.** The SIM container has `--network none` (only `lo`)
  and its own IPC namespace (`shareable`, so only our detector container can join it; never `--ipc host`), so its own
  `/dev/shm` is a private tmpfs. VERIFIED (section 17, item 7, with Docker's default `private`): `IpcMode private`,
  `/dev/shm` a fresh 64 MiB tmpfs, interfaces `lo` only. Fast DDS can therefore reach neither the Jetson's host, nor
  the team's nodes, nor other containers, nor the network, by UDP or by shared memory. The tap is the only way out,
  one way, through a socket file. Because the team's whole stack runs on the board on domain 0 with network discovery
  on, all day (live probe 11:47), SIM also uses **`ROS_DOMAIN_ID=77` and `ROS_LOCALHOST_ONLY=1`** (owner entries 11:22
  and 11:52): two independent locks, at no cost (the team code does not depend on the domain number; listed in 12.1).
  `check` confirms that no process on the board uses domain 77 (the facts read each ROS process's
  `/proc/<pid>/environ`); `--domain N` picks another. The virtual Jetson uses the same values, so it equals the board.
- **REAL: the team's domain.** REAL mode uses `--network host --ipc host`, so it shares the board's DDS world. That is
  correct on the drone (it IS the team's stack), so it uses the team's domain (0 on the board today, the field
  value; `setup` saves what the facts show). It refuses to start while any team ROS process runs (8.3). On a shared
  bench board the owner may pass `--domain N` and/or `--localhost-only` (`ROS_LOCALHOST_ONLY=1`) for a test.
- **The PC:** the tap receiver publishes on the PC's domain 0 inside WSL, as all simulator nodes do today.

### 8.6 One user inside the container

Everything in the container that speaks ROS runs as ONE user: `astra-dev`, with the Jetson SSH user's uid and gid
(`ASTRA_UID`/`ASTRA_GID`; on the PC the WSL user's). Only the address step (`ip addr add`) runs as root. VERIFIED
(section 17 item 7): root and uid 1000 in one container could not hear each other over Fast DDS in either direction
(nothing within 10 s), while the same user always could. So:

- `jetson.sh exec` = `docker exec -u astra-dev -e HOME=/home/astra-dev -w /home/astra-dev/astra astra-hil bash -lc
  'source the ROS setup and the workspace; CMD'`;
- `jetson.sh shell` opens the same user (`--root` exists for debugging and prints: "ROS commands run as root cannot
  see the team stack");
- the tap sender, `stack_check`, the look-down command and `selftest`'s ROS parts run as `astra-dev`;
- `tap.sock` is made by the tap sender as `astra-dev` = the Jetson SSH user, so sshd (the same user) can connect to it
  for the `-L` forward; mode 0600 is enough.

## 9. (e) The owner's steps

### 9.1 The commands

All of them run in the Ubuntu (WSL) terminal from the repo folder, with `J=simulation/jetson` (the full path is
written out in `docs/jetson/README.md`). Every one takes the target the same way: `--link usb|ssh|virtual`, `--host
HOST`, `--user USER`, `--port N`, `--auth login|key` (or `JETSON_HOST`, `JETSON_USER`, `JETSON_PORT`, `JETSON_AUTH`),
default `--link usb` = host `192.168.55.1`, `--auth login`. What `login`/`setup` learn is saved in
`~/.astra_sim/jetson/target.env`, so after the first time no option is needed.

| When | Command | What it does |
|---|---|---|
| right after plugging in | `bash $J/jetson.sh check` | everything that works WITHOUT a login: link up, SSH port, the Windows USB adapter; then "Not logged in: run jetson.sh login" |
| each session | `bash $J/jetson.sh login` | opens the login window (4.1); the owner types the password there; keep it open |
| then | `bash $J/jetson.sh check` | finds the board and reports each step with a fix (9.2). Changes nothing |
| once | `bash $J/jetson.sh setup` | saves the target, creates `~/astra_hil/` on the Jetson (only our folder), installs `~/astra_hil/bin/astra_hil.sh` (REAL mode without the PC), reads the team's ROS domain, prints the sudo steps the owner must type himself (docker group). `--key` / `--remove-key`: key mode (4.1) |
| once, and after code changes (PC only, heavy job) | `bash $J/jetson.sh image build` | builds the three layers on the PC (arm64 through QEMU; first time 1-2 h EXPECTED) and pushes `astra-hil:jetson` into the registry on the PC (9.5). `--flavor virtual` builds the PC's own image |
| once, and after image builds | `bash $J/jetson.sh deploy` | the Jetson pulls the image from the PC through the login (only changed layers move), copies the model weights, runs `selftest` (9.5). Refuses while the team is busy |
| once per board / JetPack / model, with the owner's OK | `bash $J/jetson.sh warmup` | builds the TensorRT engine on the Jetson (several minutes of GPU). Asks first, refuses while the team is busy |
| each flight | `bash simulation/scripts/demo.sh runway --jetson usb` | the normal demo, with the team stack on the Jetson (9.3); the camera pipeline is the simulator's default (6.3) |
| without the USB cable | `bash simulation/scripts/demo.sh runway --jetson ssh --jetson-host jetson.local` (any IP or name) | the same over Ethernet / Wi-Fi / LAN / Tailscale |
| without the board | `bash simulation/scripts/demo.sh runway --jetson virtual` | the same containers on this PC |
| any time | `jetson.sh status [--json]`, `logs [-f] [--run ID]`, `stop`, `shell`, `exec -- CMD`, `clock`, `find`, `target`, `logout` | look, stop, a terminal inside the container (as the stack's user), one command in it, the clock offset, search the usual addresses, print the target settings, end the login |
| on the drone | `jetson.sh start --real` (from the PC), or on the Jetson itself `~/astra_hil/bin/astra_hil.sh start real` | the container in REAL mode; the native `./launcher.sh` stays possible |

### 9.2 What `jetson.sh check` reports

One line per step, OK / WARN / FAIL, and for each FAIL: what failed, the likely cause and the exact fix, in the style
of `_common.sh` (`[jetson] ERROR: ... Likely cause: ... Fix: ...`). It stops at the first step that blocks the next
ones. One SSH call runs `lib/remote_facts.sh` on the Jetson (read-only, prints `KEY=VALUE` lines);
`lib/jetson_facts.py` judges them (pure Python, unit-tested with recorded samples). `--for deploy|warmup|flight`
picks the thresholds; `--quick` skips the slow rows.

| # | Step | Checked how | Typical fix printed |
|---|---|---|---|
| 1 | link up | `ping` + TCP 22 from WSL; for `usb`, also Windows' view through `powershell.exe` (read-only: is there a "Remote NDIS" adapter with `192.168.55.100`? does Windows' own `ping 192.168.55.1` work?) | plug the USB-C cable into the Jetson's USB-C port (a data cable), power on, wait 1 min; driver steps (10.1); Windows reaches it but WSL does not: 13, risk 1 |
| 2 | SSH port | TCP 22 answers | wait for boot; `sudo systemctl enable --now ssh` on the board (rescue console 10.1) |
| 3 | logged in | login mode: `ssh -S ~/.ssh/cm-jetson -O check`; key mode: `ssh -o BatchMode=yes` with the key | "Not logged in: run jetson.sh login (a window opens; type the Jetson password there; keep it open)" (exit 17). Team notes name two users (`astra-dev` on the flight Jetson, `astra-obs-avd` on `suas-jetson`); the owner picks |
| 4 | host key | `~/.astra_sim/jetson/known_hosts` (checked by the login window; a changed key shows ssh's own warning there) | another board at the same address: `ssh-keygen -f ~/.astra_sim/jetson/known_hosts -R 192.168.55.1`, then log in again |
| 5 | board + JetPack | `uname -m` = aarch64, `/proc/device-tree/model`, `/etc/nv_tegra_release` = R36 revision 4.x, Ubuntu 22.04 | the image is for L4T R36.4 (JetPack 6.2.x); other releases: reflash, or rebuild with another base tag |
| 6 | Docker without a password | `docker version` as the user (docker group), or `sudo -n docker version` (passwordless sudo) | the owner/team decides (the docker group is root-equivalent) and types in the login window: `sudo usermod -aG docker $USER`, then logs in again (the owner's guide noted the user was not in the group) |
| 7 | NVIDIA runtime | `docker info` lists `nvidia`; `nvidia-ctk --version`; `--deep`: GPU test in the deployed image | `sudo apt install nvidia-container` (SOURCE: NGC page), typed by the owner |
| 8 | **team busy?** | 9.6 | ask the team; run again when they agree and have stopped (exit 16) |
| 9 | disk, with a reserve for the team | free space of Docker's root folder and of `~`, minus what the command will use (deploy ~15 GB EXPECTED, a flight ~1 GB), must stay above the reserve = max(100 GB, 25 % of the disk) (owner may change: `JETSON_DISK_RESERVE_GB`) | delete our own old images (`jetson.sh image prune`, only `org.astra.owner=hil`) and run folders; never `docker system prune` |
| 10 | memory | `MemAvailable`: WARN < 4 GB, FAIL < 2.5 GB (CPU and GPU share 7.6 GB) | close the desktop apps, or run headless for the test |
| 11 | CUDA / TensorRT the container needs | host L4T R36.4.x (the container brings its own CUDA / cuDNN / TensorRT, SOURCE: NGC; the host gives the GPU driver through the NVIDIA runtime); `--deep`: `selftest` in the image | JetPack update: `jetson.sh warmup` again |
| 12 | **SSH forwards allowed** | a real test through the login: `-O forward -R ~/astra_hil/sock/check.sock:<a test listener on the PC>`, one byte there and back; then `-L` to a test socket on the Jetson; then `-R 127.0.0.1:29517` (registry) with `--for deploy`; everything removed afterwards | a hardened `sshd_config` (`AllowStreamLocalForwarding no`, `AllowTcpForwarding no`, `DisableForwarding yes`) blocks the link: the team/owner decides; otherwise fail early here, not in flight |
| 13 | **power mode and clocks (report only)** | `nvpmodel -q` (via `sudo -n` where allowed, else the unprivileged read; EXPECTED output "NV Power Mode: NAME" + the id), the board's own mode list from `/etc/nvpmodel.conf` (`POWER_MODEL ID=... NAME=...`), `jetson_clocks --show` (root per the docs; else from sysfs: CPU `scaling_min_freq` = `scaling_max_freq` and GPU devfreq `min_freq` = `max_freq` mean "clocks locked"; paths EXPECTED, confirmed on the board), `tegrastats` available | none. Printed: "This is the board's current mode. The owner picks the field mode. To see the modes: grep POWER_MODEL /etc/nvpmodel.conf. To change it (owner, in the login window): sudo nvpmodel -m <ID from that list>." Mode numbers differ between boards and releases (SOURCE: the R36.4 guide lists 0 = 15W, 1 = 7W, 2 = MaxN for the Orin Nano 8GB, MaxN only with the "super" flash configuration; the owner's guide recorded 25 W as mode 1 on his board), so no fixed number is ever printed |
| 14 | clock offset | PC time vs Jetson time over SSH (11): WARN above 0.5 s | none needed for scoring (collect.sh corrects); to set the clock, the owner types in the login window: `sudo date -s @<PC time printed>` |
| 15 | ports on the Jetson | `ss -ltn` (in the facts): `127.0.0.1:29517` free (deploy only) | `--registry-port N` |
| 16 | image + engine | `astra-hil:jetson` present, its code fingerprint vs the PC's (git sha only for images without one); `/models/yolo26m.engine.meta` matches TensorRT/L4T/GPU; our container running? | `jetson.sh deploy`; `jetson.sh warmup` (exit 19) |
| PC | `check --pc` (for `image build`) | Docker Engine in WSL (not Docker Desktop), `docker buildx version`, arm64 emulation (9.5), the registry image, free disk on the PC (~60 GB for the arm64 layers + the registry copy, EXPECTED) | `install_docker_engine.sh`; `sudo apt-get install docker-buildx-plugin`; the arm64 step of 9.5 |
| - | `--speed` (optional) | round trip and throughput: 50 MB through the login connection | below the camera profile's need x 1.5: `--jetson-photos after`, prefer USB/Ethernet |

With `--link virtual` there is no board: rows 1-4, 12 and 15 are skipped, and the rest are checked against this PC
(Docker Engine, the NVIDIA runtime, the image, disk, memory).

**How do I know it worked?** The last line says `ALL GOOD: the Jetson at 192.168.55.1 is ready for "jetson.sh
deploy"` (or `... ready to fly: demo.sh --jetson usb`) and the exit code is 0.

### 9.3 What `demo.sh --jetson` does differently

Only step 3 (the team stack) and the end change. Steps 1-2 (simulator, wait, upload) and 4-6 (phases, arm, fly, wait
for landing, score) are the same commands as today.

1. **Before anything starts:** `jetson.sh check --quick --for flight` (logged in, image deployed and matching the PC's
   git sha, engine present and matching the board, team not busy, disk reserve, memory, no `astra-hil` container
   already running, clock offset measured). Any failure stops demo.sh before the simulator starts, with the fix.
   `--detector truth|custom` with `--jetson` is refused (the answer-key detector runs on the PC and main_controller on
   the Jetson could not hear it through a one-way tap); `START OVER` is not supported with `--jetson` in this
   version.
2. Steps 1-2 as today (simulator, `wait-ready`, upload).
3. **STEP 3 (Jetson):** archive the Jetson's old photos (`~/astra_hil/photo_archive/<stamp>/`), `link.sh start` (the
   forwards, or the local link for `virtual`; it also starts `pc-sdk` and `pc-http`), `jetson.sh start --sim --run-id
   <stamp>_<world>` (it also starts the detector container of the new pipeline, `tegrastats` logging on the Jetson,
   and records the power mode), `ros_tap.py receive`, `collect.sh --follow`, then the remote `stack_check` with
   `--add-node` for the new pipeline's nodes (one restart of the containers if it fails, like today's one retry).
   `team_stack.env` records `TEAM_STACK_WHERE=jetson`.
4. Look down: the remote `ros2 topic pub` (7). Then phase_monitor, fly, wait: unchanged.
5. **finish():** `jetson.sh stop` (launcher.sh shuts the stack down cleanly; `tegrastats` stops), final `collect.sh`
   (with the clock correction), the end clock offset, `link.sh stop` (cancels our forwards, stops `pc-sdk`/`pc-http`;
   it never closes the owner's login window), stop the tap, write `RUN_DIR/jetson/summary.md` (9.7), then the usual
   photo copy, score report and shutdown.

### 9.4 When the link drops in the air

It must behave like the real drone losing its Jetson: **ArduPilot keeps flying.** In AUTO it flies the mission; in
GUIDED (the detector hold, the 6 s drop loiter) it holds position until the Jetson is back, exactly as the real
Pixhawk would with a silent companion (no companion failsafe is configured; `FS_GCS_ENABLE 0` in our SITL parameter
file, VERIFIED item 11).

**What really happens on the Jetson when the cable is pulled** (VERIFIED shape, section 17 item 6): the Jetson's
per-connection sshd does not notice. `ClientAliveInterval` is 0 by default (VERIFIED), so it sends nothing and waits.
It keeps its `-R` listeners and its open channels until TCP gives up, which takes up to about 15 minutes when it has
data to send and forever when it has none. In the experiment, a relay connected to `mavlink.sock` saw NO end-of-file
for 19 s after the PC's ssh went silent; removing the socket file and opening a new `-R` worked, but the old relay
connection stayed stuck. So the design never waits for the Jetson to notice:

- **Container relay:** no byte from SITL for 4 s -> close and dial `mavlink.sock` again every 1 s (5). The gate drops
  MAVROS's bytes while SITL is silent, so no stale command is sent after the reconnect. SDK: PING frames, same 4 s
  rule (6). HTTP: video_cam's own timeouts end the requests; the relay cleans up idle pipes after 15 s.
- **Tap:** PING frames every 1 s; the receiver closes and reconnects after 5 s of silence; the sender lets a new
  client replace the old one at once.
- **PC side:** the SSH keepalive (`ServerAliveInterval 2`, `ServerAliveCountMax 3`) notices in about 6 s. That ssh
  process exits, which closes its connections to SITL, `pc-sdk` and `pc-http` (SITL is then ready for the next
  client).
- **Getting back:** key mode: `link.sh` reopens its connections every 2 s by itself. Login mode: the login window says
  "the connection to the Jetson dropped: press Enter and type the password to log in again"; `link.sh` waits for the
  login and prints the same reminder every 30 s in the demo terminal.
- **Every (re)connect first kills the previous connection's sshd on the Jetson** (its PID was recorded with `echo
  $PPID` when the forwards were added; it is killed only if it is an `sshd`/`sshd-session` process of our own user
  and not the current connection; 15.3 g), then removes the stale socket files, then adds the forwards again. Killing
  the old sshd ends its listeners and channels: the relay sees end-of-file at once (VERIFIED: 0.15 s after the kill)
  and dials into the new forward, where data flows again (VERIFIED: the second dial got the PC server's data).
- **MAVROS** sees what it sees on the drone with an interrupted serial link: heartbeats stop (`connected` false after
  its timeout) and come back on the same connection.
- **demo.sh:** if the tap is silent for more than 30 s it also watches the landing over MAVLink (`mission_tool watch`),
  so a long drop cannot hang the demo; if the link is down for more than 60 s it prints what to do (log in again /
  check the cable); the run summary lists every drop (`RUN_DIR/jetson/link.log`, count and total time down) and the
  bytes the gate dropped.

**Rehearsing it.** The virtual Jetson rehearses both kinds of loss with `link.sh drop 20 --mode blackhole` (the local
stand-in stops passing bytes but keeps every socket open: a silent link, which exercises the idle timeouts) and
`--mode close` (all connections closed: a clean loss). Over SSH, `link.sh drop 20 --mode glitch` freezes the PC's ssh
for 20 s (SIGSTOP, then SIGCONT; the master's process number comes from `ssh -O check`, which prints it; the Jetson
sees silence, as with a pulled cable, then the connection resumes) and, in key mode, `--mode cut` freezes it and then
kills it (the full reconnect, including the kill of the old session). **A real cable pull
is part of the on-board acceptance** (16, step 5): the USB gadget's real behaviour can only be proven on the board.

### 9.5 Building the image on the PC, moving it onto the Jetson, building the engine

**Build (PC, heavy job: both locks, one at a time with simulator runs):** `jetson.sh image build`

1. PC prerequisites (`check --pc`): Docker Engine in WSL, buildx, enough disk.
2. **arm64 emulation, made sure every time.** The `aarch64` handler that exists in WSL today is not ours and does not
   last: VERIFIED (section 17 item 8) it has interpreter `/usr/bin/qemu-aarch64` with flags `POCF`, but that file does
   not exist in the distro, no `qemu-user*` or `binfmt-support` package is installed, and `/mnt/wsl/docker-desktop`
   exists, so it was most likely registered by Docker Desktop and disappears when WSL restarts without it. So every
   `image build --flavor jetson` first checks that emulation works (`docker run --rm --platform linux/arm64
   ubuntu:22.04 uname -m` prints `aarch64`) and, if it does not, registers it for this WSL session with `docker run
   --privileged --rm tonistiigi/binfmt:qemu-v10.2.3 --install arm64` (a pinned, published tag: SOURCE, Docker Hub,
   2026-06-08), printing what it does. The registration lasts until WSL restarts, and the step runs again on every
   build, so it is never missing. **Permanent option (owner's choice):** `sudo apt install qemu-user-static
   binfmt-support` in WSL (Ubuntu's QEMU 6.2, older) registers the handler at every WSL start, because this WSL runs
   systemd (VERIFIED: PID 1 is systemd, `/etc/wsl.conf` has `systemd=true`). Exit 18 with the fix when neither works.
3. Layer 1: `docker build --platform linux/arm64 -f simulation/jetson/image/Dockerfile.l4t-ros -t
   astra-hil-l4t-ros:r36.4.0 ...`.
4. Layer 2: `bash simulation/docker/build_images.sh dev --platform linux/arm64 --base astra-hil-l4t-ros:r36.4.0
   --torch-index https://pypi.jetson-ai-lab.io/jp6/cu126/+simple/ --tag jetson --no-check` (PATCH P1; no second copy
   of the build logic, labels, proxy arguments or logs).
5. Layer 3: `docker build --platform linux/arm64 -f simulation/jetson/image/Dockerfile --build-arg
   DEV_IMAGE=astra-dev:jetson -t astra-hil:jetson-<sha> -t astra-hil:jetson <context>`.
6. Push into the PC registry: container `astra-hil-registry` (image `registry:2`, pinned tag chosen by the builder,
   volume `astra-hil-registry`, published ONLY on `127.0.0.1:29517`), `docker push
   127.0.0.1:29517/astra-hil:jetson-<sha>`.

EXPECTED times (not measured): first build 1-2 h (apt and colcon under QEMU are slow); after a code change only
layer 3 (colcon under QEMU, ~10-20 min). Port 29517 was chosen because it is not used anywhere in the repo, nothing
listens on it in WSL, and it is below Linux's temporary port range 32768-60999 (VERIFIED item 10), so an outgoing
connection can never hold it by chance; `check` row 15 confirms it is free on the Jetson.

**Deploy (default: the registry through the login):** `jetson.sh deploy`

| Way | How | Size / time (EXPECTED) | When |
|---|---|---|---|
| **registry through the login (default)** | `-O forward -R 127.0.0.1:29517:127.0.0.1:29517` on the login connection, then on the Jetson `docker pull 127.0.0.1:29517/astra-hil:jetson-<sha>` + `docker tag ... astra-hil:jetson`, then `-O cancel`. Docker treats registries on 127.0.0.0/8 as allowed without TLS (SOURCE: dockerd docs), so the Jetson needs no setting changed | only changed layers move. The first time all layers, including the 5.2 GB l4t-jetpack base: USB 5-15 min. If the team already pulled `nvcr.io/nvidia/l4t-jetpack:r36.4.0` on the board, those identical layers are skipped | always (no internet needed on the Jetson) |
| `--via save` (fallback) | `docker save astra-hil:jetson \| gzip -1 \| ssh -S ... 'gunzip \| docker load'` | the whole image every time (~7-9 GB compressed): USB 4-12 min, Wi-Fi 20-60 min | when the forward test (check row 12) fails |
| `--via build-on-board` (last resort) | rsync the source subset to `~/astra_hil/src/` and build there, with a capped builder (`docker buildx create --driver docker-container --driver-opt cpuset-cpus=4-5 --driver-opt memory=2g`, `colcon --parallel-workers 1`, `MAKEFLAGS=-j1`), only in an idle window the team agreed to | 20-40 min, heavy load on the board | only if the PC cannot build arm64 at all |

`deploy` also takes the model weights: `--model FILE`, else the team's own file already on the Jetson
(`/home/<user>/astra/ros2_ws/src/yolo26m.pt`, copied, read-only from their folder), else the PC's
`~/.astra_sim/models/yolo26m.pt` (it says which one it used), then runs `selftest`, and writes
`~/astra_hil/deployed.env` (image, code fingerprint, git sha, date, model checksum). **On the board it does not build
the engine** (the virtual Jetson's deploy does: revision 4). It ends with:
"Next: jetson.sh warmup (once; it uses the GPU for several minutes; only when the team is not using the board)".
`deploy` refuses while the team is busy (9.6) and when the disk reserve would be crossed (row 9).

**Warmup (the one build ON the board):** `jetson.sh warmup` asks "Build the TensorRT engine now? It uses the Jetson's
GPU for several minutes. The team must not be using the board. [y/N]" (`--yes` skips the question for scripts),
refuses while the team is busy, and runs the WARMUP command of 8.3 (`--cpus 2 --memory 4g`, the work itself under
`nice -n 19 ionice -c 3` inside). It is needed once per board, JetPack/TensorRT version and model file (`start`
compares `yolo26m.engine.meta` and refuses with exit 19 when it does not match: a flight never builds the engine).

### 9.6 "Team busy?": refuse instead of disturbing the team (owner entries 11:22, 11:31)

`check` (row 8), `deploy`, `warmup` and `start` look first, read-only, through `remote_facts.sh`:

- **team programs:** processes matching `mavros_node`, `ros2 launch`, `tracker.launch`, `new_od`, `main_controller`,
  `siyi`, `gpstest`, `waypoint`, `trtexec`, `python3 ... detect`, of any user, **not inside our own containers**
  (a process is ours when its `/proc/<pid>/cgroup` names the id of a container with the label `org.astra.owner=hil`);
- **GPU load:** three `tegrastats` samples 1 s apart; `GR3D_FREQ` above 20 % on average = busy;
- **other containers:** running containers without our label are listed; one using more than 10 % CPU = busy;
- **people:** other logged-in users (`who`, without our own sessions) = WARN only;
- **memory:** below the row 10 floor = refuse (never overridable).

Refusal (exit 16): `[jetson] ERROR: the team is using the Jetson right now (found: new_od pid 1234 user astra-dev, GPU
85 %). Likely cause: the team is running their stack or a test. Fix: ask the team; when they agree and have stopped,
run the command again. Only if the owner and the team agreed to share the board: add --team-agreed.` `--team-agreed`
skips the programs/GPU/containers part only.

The live probe (11:47) found the team's flight stack running since boot and the new camera pipeline under test, so
on today's board every heavy command WILL refuse until the team stops its stack for an agreed window. That is
intended: two MAVROS/tracker stacks and two detectors on one 7.4 GiB board would disturb the team, and HIL must not.

### 9.7 The run summary (`RUN_DIR/jetson/summary.md`, also printed at the end of demo.sh)

- link (`usb`/`ssh`/`virtual`), login or key mode, host, user;
- image tag and git sha, engine meta (TensorRT, L4T, GPU), model checksum;
- **power mode at the start and at the end** (name and id from `nvpmodel -q`, or "unknown: needs sudo"), the clocks
  state (locked / not locked / unknown), and from `tegrastats`: GPU load average and peak during the search lanes,
  peak temperature, lowest free memory;
- clock offset at the start and the end, and the correction applied to the photos;
- link drops (count, total time down), bytes dropped by the gate, the largest MAVLink gap, photos mirrored, photos
  late or missing;
- **"Differences from the field"**: the fixed list of 12.1, plus what this run added (e.g. "camera profile zr10, not
  the A8 mini's 4K", "Wi-Fi link", "photos copied after the flight", "--team-agreed used").

## 10. The link choices (USB, Ethernet, Wi-Fi, LAN, Tailscale)

### 10.1 USB-C cable (`--jetson usb`, the default)

- SOURCE (NVIDIA devkit user guide): the devkit's USB-C port is "for data only" and in device mode the Jetson offers a
  "USB Ethernet (RNDIS) device for a local network between the PC and Jetson, with Jetson at 192.168.55.1" and a "USB
  Serial device for serial terminal access". Power comes from the barrel jack only. The PC gets `192.168.55.100` by
  DHCP (SOURCE: forum threads; EXPECTED here). On the Jetson the bridge is `l4tbr0`, made by
  `nv-l4t-usb-device-mode.service` (scripts in `/opt/nvidia/l4t-usb-device-mode/`; SOURCE: forum).
- **On Windows 11** (EXPECTED, confirmed by `check` on the first plug-in): a network adapter "Remote NDIS Compatible
  Device" (Windows' own driver) and a "USB Serial Device (COMn)" under Ports; maybe a small drive `L4T-README`. If
  the adapter shows under "Other devices" without a driver: Device Manager -> right-click -> Update driver -> Browse
  my computer -> Let me pick -> Network adapters -> Microsoft -> "Remote NDIS Compatible Device". This is a driver
  choice the OWNER makes (we do not change Windows settings).
- **From WSL (NAT):** WSL sends to its gateway (the Windows host), Windows routes `192.168.55.0/24` out of the USB
  adapter. Connections OUT of WSL work through NAT (SOURCE: Microsoft WSL networking page); nothing needs to come IN.
  EXPECTED to work; `check` step 1 proves it.
- **Rescue console:** the COM port (PuTTY, 115200 8N1) gives a login prompt when SSH or the network part fails.
- Known JetPack 6 trap (SOURCE: forum): `l4tbr0` missing on some CUSTOM carrier boards with a wrong device tree;
  NVIDIA says the official devkit (carrier P3768, which the owner's board is) is not affected.

### 10.2 No USB cable: `--jetson ssh --jetson-host HOST`

The software is identical: only `HOST` differs, and the owner logs in to that address. `jetson.sh find` tries, in
order: the saved target, `192.168.55.1`, `<hostname>.local` (mDNS through Windows' resolver: WSL here uses DNS
tunneling, VERIFIED nameserver `10.255.255.254`; whether `.local` resolves is EXPECTED, not tested), the Tailscale
name if Windows' `tailscale.exe status` lists it (the owner's guide: `suas-jetson`, `100.94.164.33`; the probe found
this laptop logged out of Tailscale today).

| Link | Address | What changes |
|---|---|---|
| Ethernet cable PC <-> Jetson RJ45 | the Jetson's Ethernet (`enP8p1s0` = `192.168.144.20/24` on the board today per the live probe; AGENTS.md says `.100`; check with `ip -4 addr` on the board) | the owner gives the PC's Ethernet adapter a fixed address in the same subnet, e.g. `192.168.144.50/24` (**never `.25`** (the camera), never `.20` or `.100`). The real camera sits on that network too: the PC must not answer for `.25` there. Steps written for the owner in `docs/jetson/README.md`; we do not change the adapter. Fast and steady: the best fallback when USB fails |
| Wi-Fi / LAN | an IP or a name; campus Wi-Fi addresses change (the guide's `10.110.142.92` is a generic pool address and no longer answered, per the probe) | latency 2-30 ms with spikes; photo throughput may be too low for a8mini (6); **campus Wi-Fi often blocks device-to-device traffic** ("client isolation"): then SSH itself fails -> use Tailscale, a phone hotspot, or Ethernet. Firewalls: only TCP 22 on the Jetson is needed; nothing inbound on the PC. In login mode everything shares one connection, so `--photo-pace auto` matters most here |
| Tailscale | `suas-jetson` / `100.94.164.33` (owner's board) | works from anywhere with internet; adds the internet round trip; photos at 19 Mbit/s (a8mini) need a good uplink; prefer `--jetson-photos after` |

Clear errors (15.4): "cannot reach HOST (no answer to ping or TCP 22)", "HOST answers but SSH port 22 is closed", "Not
logged in to USER@HOST (run jetson.sh login)", "SSH login refused for USER@HOST with the key (run jetson.sh setup
--key again, or use login mode)", "the host key of HOST changed", "the forward test failed (sshd does not allow
forwarding)", each with the likely cause and the exact fix.

## 11. (f) Clocks and time

- **Two clocks.** The Jetson's clock and the PC's clock are separate. SITL's time and its simulated GPS time come
  from the PC. Over USB only, the Jetson has no internet time (NTP); its clock can be wrong after a power cycle.
- **What the flight itself does not care about:** photo names are the GPS position (`"<lat> , <lon>.jpg"`), not a
  time. main_controller's 90 s hold, the 45 s mission-pull deadline and the step timers use the monotonic clock or
  relative wall-clock sleeps (SOURCE: AGENTS.md 3.4, 9), so an offset does not matter. `use_sim_time` stays false
  everywhere (SITL runs at real time, `SIM_SPEEDUP 1`); no `/clock` crosses the link. MAVROS's time sync with the
  autopilot works as on the drone. Topics arriving through the tap are stamped by the PC tools with the PC clock (7).
- **What the clock DOES affect (corrected in revision 2):**
  1. **score_run's photo geometry** uses each photo file's modification time against pose and emulator records
     stamped with the PC clock: `pose_history.capture_pose` searches `file_time - 60 s` to `file_time + 1 s`, and
     `photo_info.match` accepts emulator records from `file_time - window` (30 s by default) to `file_time + 1 s` (SOURCE:
     `photo_geometry.solve`, `pose_history.py` lines 162-168, `photo_info.py` line 126). A Jetson clock more than
     about 1 s behind the PC would push the right records outside the window and weaken the photo footprints and the
     per-photo precision in the report.
  2. **photos.csv** (`csv_export`) prints each photo's time from the sidecar's `shutter_unix_s`, which video_cam
     writes with the Jetson's clock, against the arming time from the PC (SOURCE: `csv_export.py` line 263).
  3. the ROS log times and the Jetson's own log files.
- **Measuring the offset:** `jetson.sh clock` (also inside `check`, and at the start and the end of every flight):
  seven samples over SSH, each `t0 = PC time; Jetson date +%s.%N; t1 = PC time`; `offset_s = jetson - (t0 + t1) / 2`
  from the sample with the shortest round trip; uncertainty = half that round trip (USB: ~1 ms, EXPECTED). Written to
  `RUN_DIR/jetson/clock.txt` (`offset_start_s`, `offset_end_s`, `rtt_ms`); `offset_s > 0` means the Jetson is ahead.
- **Correcting it:** `collect.sh` sets each mirrored photo's (and sidecar's) modification time to `jetson_mtime -
  offset_s` before it moves the file into the watched folder (7, item 4), and does the same for the final copy into
  `RUN_DIR/photos/`. If the start and end offsets differ by more than 0.2 s, it interpolates by time and notes it.
  The sidecar files' contents stay exactly as the team code wrote them; `csv_export` applies the offset from
  `RUN_DIR/jetson/clock.txt` when that file exists (a small PC-SIDE change, 15.2).
- **Warning threshold: 0.5 s** (was 2 s): above it `check` and the run summary say so, even though the photos are
  corrected, because the team's own log and sidecar times are then off by that much.
- **Setting the Jetson's clock** needs sudo, so the tools never do it: `check` prints the exact command for the owner
  to type in the login window (`sudo date -s @<PC time>`), or "connect it to the internet once (NTP)".
- **Builds:** the image is built on the PC, and the default deploy talks plain HTTP to the PC's registry through the
  tunnel, so a wrong Jetson clock cannot break `deploy` through certificate errors. Only `--via build-on-board`
  downloads over HTTPS on the board: there `check` turns an offset above 60 s into an error.

## 12. (g) What the virtual Jetson proves, and what only the board proves

The virtual Jetson is the SAME image definition (x86 flavor), started by the SAME `jetson.sh start`, with the SAME
container settings (`--network none`, `NET_ADMIN`, the socket folder, the variables, the one in-container user) and
the SAME relay and tap. Only two things are replaced: the x86 build instead of the arm64 one, and `hil_relay.py
pc-local` (it makes the three socket files itself, standing in for sshd + `pc-sdk` + `pc-http`) instead of SSH.

| Proven by the virtual Jetson | Only the real board can prove |
|---|---|
| the image layout, entrypoint, modes, variables, mounts, `selftest`, `stack_check` over `exec` as the stack's user | the USB network gadget on Windows 11 and WSL's NAT route to `192.168.55.1` |
| `192.168.144.25` in the container + the SDK and HTTP relays (frames, PING): photos, gimbal, zoom | the owner's login window, sudo, the docker group, sshd's forwarding settings on the board |
| MAVROS over the relay to SITL SERIAL2; team texts in Mission Planner | the arm64 build running: ROS 2 arm64 packages, MAVROS from the snapshot on arm64, jetson-ai-lab torch, `import tensorrt`, two OpenCVs side by side (risk 17) |
| the tap: phase_monitor and score_run unchanged on the PC, presence nodes, collect with the clock correction fills the run folder | TensorRT engine build on the Orin, batched inference speed, whether the detector finishes within the 90 s hold (HANDOFF D6) |
| silent and clean link losses (`link.sh drop 20 --mode blackhole` / `close`): idle timeouts, gate, re-dial, MAVROS heartbeats back, the flight goes on | memory pressure in 7.6 GB shared CPU/GPU memory; power mode and heat (`tegrastats`) |
| `demo.sh --jetson` end to end with `--camera a8mini`, timings of the photo pipeline through the relay | real Wi-Fi / USB throughput and latency, a pulled cable, the Jetson's clock |
| the reconnect steps of 15.3 g against a real OpenSSH server (the throwaway sshd container of section 17 items 4 and 6) | the NVIDIA container runtime on JetPack 6.2.x with an r36.4.0 image |

### 12.1 Differences from the field (the list `docs/jetson/README.md` and every run summary carry)

The Jetson-in-the-loop flight runs the team's software as at the field (owner 11:16). What still differs:

1. **The autopilot is ArduCopter 4.6.3 SITL on the PC, not the Pixhawk.** Same firmware version; simulated sensors,
   physics and GPS (Gazebo world); simulated time runs at real speed.
2. **The MAVLink link is TCP through SSH (USB network, Ethernet or Wi-Fi), not TELEM2 serial.** MAVROS uses
   `FCU_URL=tcp://127.0.0.1:5763` instead of `/dev/ttyUSB0:115200` (launcher.sh runs in `hw` mode in both). There is
   no 115200-baud limit unless the PC side turns on SITL's limiter: `SIM_BAUDLIMIT_EN` exists in 4.6.3 (SOURCE:
   `SITL.cpp`, default 0) and limits a port to its `SERIALn_BAUD`; our parameter file sets neither it nor
   `SERIAL2_BAUD` today (VERIFIED item 11). The PC-SIDE builder adds `demo.sh --jetson-baud 115200|off` through the
   existing extra-parameter-file mechanism (`autopilot.param_files`); the owner decides the default (15.5).
3. **Link glitches behave differently.** A broken serial link loses bytes. Through SSH, bytes already sent can arrive
   up to ~6 s late if the connection recovers by itself; the gate limits this to what MAVROS sent in the first 2.5 s
   of the glitch. A long drop ends the connection (9.4).
4. **The camera is the emulator on the PC, reached through the relay.** The SIYI protocol, the RTSP stream and the
   full-size photos are real protocols; the video is rendered by Gazebo and encoded on the PC (VIDEO_PIPELINE.md)
   instead of by the camera, and the stream, the SDK and the downloads go through the SSH link (photos paced in login
   mode) instead of the camera's own Ethernet port, so latency and download times differ from the real camera's
   (neither has been measured on the real camera).
5. **Camera pipeline and launch file:** the new video pipeline (ZR10 control, 1280x720 at 2 fps) is started by the
   video workflow's PROPOSED field launch file, which **the team has not adopted yet** (on the board today the pieces
   are started by hand and the boot-time launcher still starts the obsolete photo nodes; owner 12:10). The detector
   container joins our container's private network instead of `--network host` (6.3). The legacy photo option uses
   the `zr10` photo profile (2560x1440); `a8mini` is only a heavier load test.
6. **ROS domain 77 with `ROS_LOCALHOST_ONLY=1`** instead of the field's domain 0 (8.5); the team code does not depend
   on the number.
7. **The Jetson's clock** may differ from the PC's; photo file times are corrected, sidecar contents and message
   stamps are not (11).
8. **The board's setup** may differ from flight day: desktop running vs headless, power mode, other programs, the
   containers' memory and CPU limits (all recorded in the run summary).
9. **Files go to our own folder** (`~/astra_hil/photos`, not the team's `mapping_photos`) in SIM mode, so the team's
   folders are never touched.
10. **ROS graph:** sealed inside the container; nothing on the drone's network would see it either, but there the
    team's other tools on the Jetson host could. The detector runs inside our container's network instead of
    `--network host`.
11. **Mission wiring of the new detections:** unknown until the team says how the yolo26 detections reach
    main_controller (6.3); the run summary states what was and was not connected.
12. **MAVROS 2.14.0 in the image vs 2.12.0 natively on the board** (revision 3): no apt source carries 2.12.0 (the
    snapshot 2026-08-07 has 2.14.0; between them mavros_msgs only gained one message). Version for version would need
    a MAVROS source build in layer 2.
13. **The virtual Jetson (PC)** (revision 4: with the default TensorRT image the legacy detector runs a TensorRT engine
    built on the PC's GPU, not the `.pt`; the rest of this item describes the plain-ROS image): x86 + RTX 4060 instead
    of the Orin GPU; the legacy detector runs the `.pt` model on
    the PC GPU (owner 12:25 item 5: TensorRT inside Docker on the PC is the next step; the image now has TensorRT 10.3).

## 13. (h) Risks and fallbacks

| # | Risk | Sign | Fallback |
|---|---|---|---|
| 1 | WSL (NAT) cannot reach `192.168.55.1` although Windows can | `check` step 1: Windows ping OK, WSL ping/TCP fails | run the login and the forwards with Windows' own `ssh.exe` through WSL interop (to build only if needed): Windows reaches SITL in WSL through `localhost` forwarding; the camera HTTP target then needs a small WSL relay on `0.0.0.0`; or (owner's OK, `.wslconfig` change) mirrored networking; or Ethernet/Wi-Fi |
| 2 | no RNDIS driver on Windows | Device Manager "Other devices" | the driver pick in 10.1 (owner); or Ethernet cable |
| 3 | USB gadget not up on the board | no adapter at all | rescue console (COM port) or a screen: `systemctl status nv-l4t-usb-device-mode`; Ethernet / Wi-Fi meanwhile |
| 4 | ~~MAVROS arm64 packages missing~~ **resolved** | - | the snapshot `humble/2026-08-07` has arm64 `ros-humble-mavros` 2.14.0 and the main ROS repo now has arm64 2.16.0 (SOURCE: reviewer's check, 2026-10-31). Fallback if it ever breaks: build MAVROS from source in layer 3 |
| 5 | `import tensorrt` / ultralytics export fails in the image | `selftest` / `warmup` | **mostly resolved:** `nvidia-tensorrt-dev` -> `tensorrt` 10.3.0.30 -> `python3-libnvinfer-dev` are inside l4t-jetpack (SOURCE: reviewer's check), so `import tensorrt` should work in the container even though the host has no TensorRT Python bindings (owner 11:31). Left: ultralytics' export helpers (`onnxslim`; auto-install is off): add them to layer 3 if `warmup` asks; the team's `fixtensorRT.md` notes |
| 6 | first engine build is slow (minutes) | first `stack_check` waits for "ModelLoaded" | `warmup` is a separate step; `start` refuses without a matching engine (exit 19), so a flight never builds it |
| 7 | memory: desktop + container + engine | `check` WARN, OOM in logs | headless during tests (the owner types `sudo systemctl isolate multi-user.target`, back with `graphical.target`); the container limits of 8.3 |
| 8 | link drops in flight | `link.log`, MAVROS "connection lost" | 9.4: ArduPilot flies on; idle timeouts + kill of the old session + reconnect; landing watched over MAVLink |
| 9 | Wi-Fi client isolation / slow Wi-Fi | SSH fails / photos dropped | Tailscale, hotspot, Ethernet; `--jetson-photos after`; zr10 for quick runs |
| 10 | stale socket files block the forward | "remote port forwarding failed" | `rm -f` after killing the old session, before every forward (VERIFIED) |
| 11 | photos delay MAVLink on one shared connection (login mode) | the run summary's largest MAVLink gap | pacing (4.1); key mode (two connections); plan B for MAVLink only: UDP (section 5 table) |
| 12 | wrong Jetson clock | `check` WARN | collect.sh corrects photo times (11); the owner sets the clock |
| 13 | two Jetsons / two users in team notes (`astra-dev`, `astra-obs-avd`) | login refused | the owner says which board and user (15.5); saved in `target.env` |
| 14 | the drone's native install and the container both on `/dev/ttyUSB0` | serial port busy | REAL mode refuses to start while a native MAVROS / team ROS process runs |
| 15 | a JetPack update changes TensorRT | engine fails to load | `yolo26m.engine.meta` check at `start` (exit 19), `jetson.sh warmup` |
| 16 | the force-committed `yolo26m.engine` gets into the image | wrong-version engine | `context.list` excludes `*.engine`; `selftest` checks the engine's meta |
| 17 | **two OpenCVs in the arm64 image** | apt conflict while building layer 2, or `cv_bridge` errors | `l4t-jetpack:r36.4.0` installs `nvidia-opencv-dev` (NVIDIA's OpenCV 4.8.0, which Provides/Replaces/Conflicts Ubuntu's `libopencv-*-dev`; SOURCE: reviewer's reading of the image history), while `ros-humble-cv-bridge` (arm64) needs `libopencv-dev`, the `libopencv-*4.5d` runtime libraries and `python3-opencv`. The names do not collide, so apt should install both, but that is untested, and the image would hold two OpenCVs where the drone has Ubuntu's 4.5.4 only. **First step of the layer 1 work:** `apt-get install -s ros-humble-cv-bridge` on top of l4t-jetpack (a dry run). Fallback: purge `nvidia-opencv*` in layer 1, so the image matches the drone (Ubuntu 4.5.4 for C++ + pip opencv-python, as in docker/Dockerfile's header) |
| 18 | **a re-published torch wheel** with the same version | `selftest` import errors (e.g. `libcudss.so.0` missing) | hashes pinned (8.1, PATCH P2); the build fails on a hash mismatch instead of shipping a broken image |
| 19 | **arm64 emulation missing or too old** | `image build` exit 18, or QEMU crashes in a build step | the ensure step of 9.5 (tonistiigi/binfmt pinned to QEMU 10.2.3); permanent option apt `qemu-user-static`; last resort `--via build-on-board` in an agreed idle window |
| 20 | the owner's login drops during a flight (login mode) | link down, reminder every 30 s | the owner logs in again; the drone holds or flies on meanwhile; key mode if this happens often |
| 21 | no docker access without a password on the board | `check` row 6 | the team decides (docker group or passwordless sudo for docker); without it the kit cannot run containers |
| 22 | the team is using the board (today: their stack runs since boot) | `check` row 8 (exit 16) | wait for an agreed window where the team stops it; `--team-agreed` only with the owner's and team's OK |
| 23 | the detector container runs as another user than the stack | no detections reach the stack (the shared-memory trap of 8.6, VERIFIED) | `-u $ASTRA_UID:$ASTRA_GID` on the detector container; if its image cannot run as that user, give it a Fast DDS profile without shared memory (UDP on the private `lo` only) and say so in the summary |
| 24 | RTSP through the relay adds latency or stalls | frame gaps in `/zr10/image_raw` (stream node log), detector rate in the summary | USB/Ethernet; the stream is small (6.3); the relay's TCP copy has no buffering of its own; ffmpeg's own reconnect handles a dropped stream after a link drop (to confirm in the virtual drop test) |
| 25 | the board's CPU: the team's stream node took one core at 93 % in the live probe | high load in `tegrastats` | expected and field-like (it is the field pipeline); the container CPU limit leaves room; recorded in the summary |
| 26 | how the yolo26 detections reach main_controller is unknown | no detection-driven waypoints or drops in a new-pipeline flight | owner/team question (6.3); the summary says it plainly; the legacy option still exercises the old wiring |
| 27 | the new pipeline's sources live outside this repo (`~/astra_ws` on the board, team branch `video-cam`) | image build cannot find them | VIDEO_PIPELINE.md names the source; `image build` refuses with the fix when it is missing |
| 28 | (revision 4, VERIFIED) the TensorRT export needs a working `onnx`; apt's 1.7.0 breaks with numpy 1.26 | warmup fails with `module 'numpy' has no attribute 'object'`; selftest's ONNX row FAIL | layer 3 pins the team's `onnx==1.14.0` + `protobuf==3.20.3`; on the arm64 image the onnx 1.14.0 aarch64 wheel is EXPECTED from PyPI (not yet built) |
| 29 | (revision 4) a virtual image older than the code (someone edited a team package after the build) | check row 16 FAIL 12 `(the PC has code XXXX)` before the flight | `image build --flavor virtual` (only layer 3 rebuilds) |

## 14. Later: real camera mirroring (NOT built now)

The owner wants, later, the REAL SIYI camera plugged into the Jetson to move together with the simulated one: gimbal
and zoom commands copied to the real camera, while the photos still come from the simulator.

- **Where it plugs in:** in the SDK part of the relay's container end (`hil_relay.py container`), which sees every
  SIYI SDK frame video_cam sends. A mirror is a "tee": a copy of selected frames (gimbal angles 0x0E, gimbal speed
  0x07, zoom 0x0F / 0x05, center 0x08; NEVER the shutter 0x0C, the SD format, or the photo list) goes to a third
  socket `cam_mirror.sock`. The answers from the real camera are read and thrown away (the simulator stays the camera
  the team code believes).
- **The address clash it must solve:** inside the container `192.168.144.25` is the stand-in, so the container cannot
  reach the REAL camera, which is also `192.168.144.25`, on the Jetson's eth0. So the mirror must live OUTSIDE the
  container's network: a small second container (`astra-hil-mirror`, `--network host`, our label) or a host process
  that listens on `cam_mirror.sock` and sends to the real `192.168.144.25:37260` on eth0. The team container never
  gets eth0 in SIM mode (the guard of section 6 enforces it), so its `.25` and the real `.25` never meet.
- What it will need then: a `--camera-mirror on` option (default off), the list of frames to copy, a rate limit, and
  a check that the real camera answers (`0x01` firmware) before mirroring.

## 15. File plan and the contract between the two halves

### 15.0 Who builds what (owner entry 11:16)

The separate "jetson-kit" agent was stopped before it wrote anything, so there is no kit to wait for and no waiting
rule. **The JETSON-SIDE builder owns and creates `jetson.sh` (in `$J` = `simulation/jetson/`), `simulation/jetson/lib/**`,
`simulation/jetson/image/**`, `simulation/jetson/test/**` and `docs/jetson/README.md`.** Its first deliverable is the
first-connection kit, in this order, because that is what the owner wants first: `lib/target.sh` (the contract of
15.3 a, so the PC side can source it), `lib/login_window.sh` + `jetson.sh login|logout`, `jetson.sh check`
(remote_facts + jetson_facts + tests), `jetson.sh setup`, `status`, and the README's "Plug in and test". **The
PC-SIDE builder owns `simulation/jetson/pc/**` and `docs/jetson/FLYING.md`,** plus the small edits listed in 15.2.
Neither builder edits the other's files; the contract (15.3) is what they code against. A builder that needs a change
in the other's file writes it as a PATCH in its report.

### 15.1 JETSON-SIDE (builder 1): the kit, the image, the relay

| File | New/changed | What |
|---|---|---|
| `$J/jetson.sh` (`J=simulation/jetson`) | new | all subcommands of 9.1: `login`, `logout`, `check [--quick] [--deep] [--speed] [--pc] [--for deploy\|warmup\|flight] [--team-agreed]`, `setup [--key\|--remove-key] [--user]`, `image build [--flavor jetson\|virtual] [--no-cache]`, `image status`, `image prune`, `deploy [--via registry\|save\|build-on-board] [--model FILE] [--registry-port N] [--team-agreed]`, `warmup [--yes] [--team-agreed]`, `start --sim\|--real [--run-id ID] [--archive-photos\|--keep-photos] [--domain N] [--localhost-only] [--team-root DIR] [--team-agreed]`, `stop`, `restart`, `status [--json]`, `logs [-f] [--run ID]`, `exec [--tty] -- CMD...`, `shell [--root]`, `paths`, `target [--format env\|json]`, `clock`, `find`; every one takes `--link --host --user --port --auth` |
| `simulation/jetson/lib/target.sh` | new, FIRST | the contract of 15.3 a; sourced by `jetson.sh` and by the PC side; `[jetson]` messages through `simulation/scripts/_common.sh` |
| `simulation/jetson/lib/login_window.sh` | new | the visible login window of 4.1 (behaviour of `D:\suas_scratch\e2e\jetson_login.sh`, plus the fast keepalive and "press Enter to log in again") |
| `simulation/jetson/lib/remote_facts.sh` + `lib/jetson_facts.py` | new | read-only `KEY=VALUE` facts from the Jetson (board, L4T, docker, runtime, disk, memory, team-busy evidence, power mode and its list, clocks, ports, image, engine meta, clock sample), judged by a pure-Python module (testable on Windows Python) |
| `simulation/jetson/lib/hil_relay.py` | new | all relay roles of 15.3 d; standard library only |
| `simulation/jetson/lib/astra_hil.sh` | new | installed by `setup` to `~/astra_hil/bin/` on the Jetson: `start real\|sim`, `stop`, `status`, `logs` without the PC repo (for the drone) |
| `simulation/jetson/image/Dockerfile.l4t-ros` | new | layer 1 for arm64: `nvcr.io/nvidia/l4t-jetpack:r36.4.0` + ROS 2 Humble ros-base + rosdep init + colcon; first step: the OpenCV dry run of risk 17 |
| `simulation/jetson/image/Dockerfile` | new | layer 3: `ARG DEV_IMAGE`; `/home/astra-dev/astra` layout, copy of `context.list`, the new pipeline's packages + ffmpeg (6.3), colcon build, model symlinks, user rename, `/opt/astra/*`, labels, `ENTRYPOINT` |
| the detector container (in `jetson.sh start`) | part of `jetson.sh` | `astra-hil-detector` joined to `astra-hil` (8.3); image and command from VIDEO_PIPELINE.md; on the board the image arrives through the same registry (or is the team's own image, per the owner's decision 15.5) |
| `simulation/jetson/image/context.list`, `jetson_pins.env` | new | the repo paths copied into the build context (and the excludes); the pinned torch/torchvision hashes |
| `simulation/jetson/image/entrypoint.sh`, `in_stack.sh`, `selftest.py`, `warmup.py` | new | 8.2 |
| `simulation/jetson/test/` | new | `test_hil_relay.py` (loopback sockets: frames, PING, client ids, HTTP, sticky MAVLink, idle re-dial, gate, blackhole/close drops), `test_jetson_facts.py` (recorded samples: the owner's board per his guide, wrong L4T, no docker group, low disk, no nvidia runtime, team busy, sudo not available), `test_target.sh` (option/env/file precedence), shellcheck |
| `docs/jetson/README.md` | new | starts with "Plug in and test" (plug in, `check`, `login`, `check`, what each line means); then setup, image build, deploy, warmup, Ethernet / Wi-Fi / Tailscale, REAL mode, "Differences from the field" (12.1), troubleshooting; links to `FLYING.md` |
| PATCH P1, P2 (15.6) | patches | `build_images.sh` jetson options; torch hashes in `docker/Dockerfile` |

### 15.2 PC-SIDE (builder 2): the simulator end

| File | New/changed | What |
|---|---|---|
| `simulation/jetson/pc/link.sh` | new | `start\|stop\|status\|drop SECONDS [--mode blackhole\|close\|glitch\|cut]`: the reconnect steps of 15.3 g, the forwards (or `hil_relay.py pc-local` for `virtual`), starts/stops `pc-sdk` and `pc-http`, waits for the login after a drop (reminder every 30 s), writes `$ASTRA_SIM_HOME/state/jetson_link.env` and `RUN_DIR/jetson/link.log` |
| `simulation/jetson/pc/ros_tap.py` + `tap_topics.txt` | new | the tap, both roles (15.3 e) and the allow-list of section 7 |
| `simulation/jetson/pc/collect.sh` | new | `--follow` (live mirror into `sim_paths photo-dir`, staging + clock correction + rename, `--bwlimit` in login mode) and the final copy into `RUN_DIR` (15.3 b); clock samples at start and end |
| `simulation/jetson/pc/hil_steps.sh` | new | functions demo.sh sources for the `--jetson` path (preflight, start, ready, look-down, finish, summary of 9.7), so the demo.sh change stays small |
| `simulation/jetson/pc/windows_link_check.ps1` | new | read-only Windows view for `check` step 1 (USB adapter, its address, Windows' ping/TCP 22); called through `powershell.exe` by `jetson.sh check` |
| `simulation/jetson/pc/hil_serial2_115200.parm` | new | `SIM_BAUDLIMIT_EN 1`, `SERIAL2_BAUD 115` for `--jetson-baud 115200` (12.1 item 2); the builder checks that the parameter filter lets `SIM_*` through |
| `simulation/scripts/demo.sh` | small edit (shared file) | options `--jetson usb\|ssh\|virtual`, `--jetson-host`, `--jetson-user`, `--jetson-port`, `--jetson-photos live\|after`, `--jetson-tap on\|off`, `--jetson-baud 115200\|off`, `--photo-pace auto\|off\|MBIT`; with none of them demo.sh is exactly as today |
| `ros2_ws/src/simulation/sim_demo/sim_demo/csv_export.py` (+ its test) | small edit | apply `offset_s` from `RUN_DIR/jetson/clock.txt` to sidecar `shutter_unix_s` when the file exists (11) |
| `simulation/jetson/pc/test/` | new | tap frames + QoS mapping + NODES/presence logic, collect file lists and the time correction, hil_steps option parsing, link.sh reconnect steps against the throwaway sshd container (light, no lock); one virtual-Jetson flight (heavy, both locks, visible window) |
| `docs/jetson/FLYING.md` | new | flying with `demo.sh --jetson`, the run folder, the summary, link drops, Wi-Fi and photo modes |
| `simulation/README.md`, `AGENTS.md` | small edits (shared) | a short "Jetson in the loop" pointer; the new docs go into `docs/README.md` through the docs integrator |

### 15.3 The contract (both builders code against this)

**a) The target: `simulation/jetson/lib/target.sh` (JETSON-SIDE writes it first; PC-SIDE sources it).**
`jetson_target_load "$@"` parses `--link usb|ssh|virtual`, `--host`, `--user`, `--port`, `--auth login|key`
(options > environment `JETSON_*` > `~/.astra_sim/jetson/target.env` > defaults), never connects, leaves the other
arguments in the array `JETSON_ARGS`, and sets:

| Variable | Example / default | Meaning |
|---|---|---|
| `JETSON_LINK` | `usb` | `usb` / `ssh` / `virtual` |
| `JETSON_HOST`, `JETSON_USER`, `JETSON_PORT` | `192.168.55.1`, (saved), `22` | the target |
| `JETSON_AUTH` | `login` | `login` (owner's window) or `key` |
| `JETSON_CM` | `~/.ssh/cm-jetson` | the login window's control socket |
| `JETSON_CM_A`, `JETSON_CM_B` | login mode: both `= JETSON_CM`; key mode: `~/.astra_sim/jetson/cm-A`, `cm-B` | the connection for LINK A + commands, and for LINK B + the photo mirror |
| `JETSON_KEY` | `~/.astra_sim/jetson/id_ed25519` | key mode only |
| `JETSON_KNOWN_HOSTS` | `~/.astra_sim/jetson/known_hosts` | the one known-hosts file |
| `JETSON_DATA` | `""` until the first connection, then `$HOME/astra_hil` as the Jetson reports it (`jetson_resolve_data`; saved by login/setup; virtual: `~/.astra_sim/jetson_virtual`) (revision 3) | `D` |
| `JETSON_DEST`, `JETSON_CONTAINER`, `JETSON_IMAGE` | `astra-dev@192.168.55.1`, `astra-hil` (virtual `astra-hil-virtual`), `astra-hil:jetson` (virtual `astra-hil:virtual`) | what ssh is given; our container and image names (revision 3) |
| `JETSON_STATE` | `~/.astra_sim/jetson` | PC-side files |
| `JETSON_SSH`, `JETSON_SSH_B` | bash arrays | the full command prefix to run a remote command through `JETSON_CM_A` / `JETSON_CM_B` (empty for `virtual`) |
| `JETSON_RSYNC_RSH` | string | the `rsync -e` value (through `JETSON_CM_B`) |

The ONE SSH option set (in `target.sh` only): `-o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=2 -o
ServerAliveCountMax=3 -o UserKnownHostsFile=$JETSON_KNOWN_HOSTS -o StrictHostKeyChecking=yes -o
ExitOnForwardFailure=yes -o LogLevel=ERROR -p $JETSON_PORT`, plus `-S $JETSON_CM_A -o ControlMaster=no` for every
command. Key mode masters add `-i $JETSON_KEY -o IdentitiesOnly=yes -o ControlMaster=yes -o ControlPersist=yes -fN`.
Functions: `jetson_logged_in` (0 when `-O check` answers on `JETSON_CM_A` and `JETSON_CM_B`; key mode: opens the
masters when needed), `jetson_need_login` (prints the standard "Not logged in" error, returns 17), `jetson_ssh CMD...`
(through A; for `virtual` runs locally), `jetson_ssh_b CMD...`, `jetson_rsync ARGS...`, `jetson_forward add|cancel
A|B SPEC...` (`ssh -S ... -O forward|cancel` with `-R`/`-L` specs). `jetson.sh target [--format env|json]` prints the
same values as `KEY=VALUE` lines (`JETSON_SSH` as one shell-quoted string) for Python and other readers.

**b) Folders.** On the Jetson (`D` = `JETSON_DATA`): `D/sock/` (mode 0700; `mavlink.sock`, `cam_sdk.sock`,
`cam_http.sock` made by sshd for the PC's `-R`; `tap.sock` made by `ros_tap.py send`; `check.sock` only during
`check`), `D/run/` (`link_A.pid`, `link_B.pid`, `tegrastats.pid`), `D/models/` (`yolo26m.pt`, `yolo26m.engine`,
`.meta`), `D/photos/`, `D/detections/`, `D/photo_archive/<stamp>/`, `D/runs/<run_id>/` (`team_stack.log`,
`relay.log`, `relay.status`, `tap.log`, `ros_log/`, `tegrastats.log`, `facts.env`, `power.txt`), `D/src/` (only for
`--via build-on-board`), `D/bin/astra_hil.sh`, `D/logs/`, `D/deployed.env`. Inside the container: `/run/astra_hil` =
`D/sock`, `/models`, `/data/runs`, the photo and detection folders as in 8.3, `/opt/astra/paths.env` (`WS=`,
`PHOTO_DIR=`, `DETECTION_DIR=`), `/opt/astra/image.env` (`GIT_SHA=`, `CODE_SHA=`, `FLAVOR=`, `BUILT=`, `TENSORRT=`, `TENSORRT_VERSION=`). On the PC:
`$ASTRA_SIM_HOME/jetson/` (`target.env`, `known_hosts`, `login.env`, key mode: `id_ed25519`, `cm-A`, `cm-B`),
`~/.ssh/cm-jetson` (login mode), `$ASTRA_SIM_HOME/jetson_virtual/` (the virtual Jetson's `D`),
`$ASTRA_SIM_HOME/state/jetson_link.env` (`LINK=`, `HOST=`, `AUTH=`, `A=up|down`, `B=up|down`, `DROPS=`, `DOWN_S=`,
`SINCE=`), `$ASTRA_SIM_HOME/state/jetson_tap.env`, `$ASTRA_SIM_HOME/state/jetson_local.ctl` (virtual drop control).
Run folder: `RUN_DIR/team_stack.log`, `RUN_DIR/photos/`, `RUN_DIR/detections/` (same names as a PC run) and
`RUN_DIR/jetson/{container.log, relay.log, relay.status, tap.log, tap_receive.log, ros_log/, tegrastats.log,
facts.env, power.txt, clock.txt, link.log, image.txt, summary.md}`.

**c) Ports.** PC (WSL, loopback only): `127.0.0.1:37261` (`pc-sdk`), `127.0.0.1:37282` (`pc-http`),
`127.0.0.1:7461` (`-L` to `tap.sock`), `127.0.0.1:29517` (registry), plus today's `5763+10I`,
`192.168.144.25:37260/82` and the video-pipeline workflow's RTSP stream `192.168.144.25:8554`. Container:
`127.0.0.1:5763` TCP, `192.168.144.25:82` TCP, `192.168.144.25:8554` TCP, `192.168.144.25:37260` UDP. Jetson host:
nothing new except `127.0.0.1:29517` during `deploy`. Socket files: `cam_rtsp.sock` joins the B group in b) and g).

**d) `hil_relay.py` command lines (JETSON-SIDE writes it; the entrypoint starts `container`, the PC side starts the
others).** Every role: `--log FILE`, `--status-file FILE` (`KEY=VALUE`, rewritten on change), SIGTERM = clean stop
(exit 0). Exit codes: 2 typed wrong, 3 cannot listen (address in use, or `192.168.144.25` not on `lo`), 4 socket
folder missing.

```
hil_relay.py container --sock-dir /run/astra_hil --mav-listen 127.0.0.1:5763 \
    --cam-ip 192.168.144.25 --sdk-port 37260 --http-port 82 --rtsp-port 8554 \
    --idle-timeout 4 --redial 1 --gate 2.5 --http-idle 15 --gap-log 0.5
hil_relay.py pc-sdk   --listen 127.0.0.1:37261 --camera 192.168.144.25:37260 --idle-timeout 4
hil_relay.py pc-http  --listen 127.0.0.1:37282 --camera 192.168.144.25:82 --pace-mbit 0|N --http-idle 15
hil_relay.py pc-local --sock-dir DIR --sitl 127.0.0.1:5763 --camera-sdk 192.168.144.25:37260 \
    --camera-http 192.168.144.25:82 --camera-rtsp 192.168.144.25:8554 \
    --control $ASTRA_SIM_HOME/state/jetson_local.ctl
hil_relay.py drop     --control $ASTRA_SIM_HOME/state/jetson_local.ctl --seconds N --mode blackhole|close
```

Rules: MAVLink side sticky toward MAVROS; idle timeout 4 s toward SITL, re-dial every 1 s; the gate (forward MAVROS
bytes only while a SITL byte arrived within 2.5 s; otherwise drop and count; never buffer across re-dials); every
MAVLink gap over 0.5 s is logged. The RTSP listener is a plain TCP copy like HTTP (one socket connection per
client connection). `pc-local` makes `mavlink.sock`, `cam_sdk.sock`, `cam_http.sock`, `cam_rtsp.sock` itself
(removing stale ones first) and does the work of sshd + `pc-sdk` + `pc-http`. `relay.status` keys: `MAV=up|down`, `SDK=up|down`,
`HTTP_ACTIVE=n`, `REDIALS=n`, `MAV_DROPPED_BYTES=n`, `MAX_GAP_S=x`; revision 3 adds `LISTEN=yes|no` (all listeners
bound; `jetson.sh start` waits for it), `STARTED=unix time`, `MAV_CLIENT=connected|none`, `MAV_DROPS=n` and
`MAV_DOWN_S=x` (SITL silent end to end, across re-dials; `MAX_GAP_S` is the largest gap inside ONE connection),
`SDK_DROPPED=n` (datagrams while the SDK link was down), `RTSP_ACTIVE=n`. `--rtsp-port 0` turns the RTSP listener off.
pc-local's status has `LISTEN`, `MAV_CONNS`, `HTTP_ACTIVE`, `RTSP_ACTIVE`.

**e) `ros_tap.py` command lines and stream (PC-SIDE writes it; the JETSON-SIDE image copies it to `/opt/astra/` and
the entrypoint starts `send` as `astra-dev`).**

```
ros_tap.py send    --listen unix:/run/astra_hil/tap.sock --topics /opt/astra/tap_topics.txt \
    --nodes waypoint_manager,main_controller,siyi,mission_camera_trigger,new_od,gps_mavros_service_node,mavros,<the new pipeline's nodes> \
    --log /data/runs/<id>/tap.log
ros_tap.py receive --connect tcp:127.0.0.1:7461 | unix:<virtual D>/sock/tap.sock \
    --node-name jetson_tap --presence-namespace /jetson \
    --log RUN_DIR/jetson/tap_receive.log --status-file $ASTRA_SIM_HOME/state/jetson_tap.env
```

In `virtual` mode there is no `-L`, so the receiver connects to the socket file directly (`unix:`). `tap_topics.txt`:
one `topic type` pair per line, `#` comments. Stream on `tap.sock` / `127.0.0.1:7461`: frames `u32 big-endian length`
+ `u8 kind` + payload: kind 0 HELLO (JSON `{"proto": "astra-tap/2", "ros_distro": "humble", "time_unix": ...}`), kind
1 ANNOUNCE (JSON `{"id": n, "topic": ..., "type": "pkg/msg/Name", "qos": {"reliability": "reliable|best_effort",
"durability": "volatile|transient_local", "depth": n}}`), kind 2 DATA (`u16 id` + the CDR bytes rclpy serialized), kind
3 PING (every 1 s, both ways), kind 4 NODES (JSON list of `[name, namespace]` of the container's graph, filtered to
`--nodes`, every 2 s). One client at a time; a new client replaces the old one and gets HELLO + all ANNOUNCEs again.
The receiver reconnects after 5 s without any frame. Exit codes: 2 typed wrong, 3 cannot listen/connect at start, 4
bad topic file.

**f) SDK frame and MAVLink.** SDK frame (on `cam_sdk.sock` and `127.0.0.1:37261`): `u16 big-endian N` (N >= 1) + `u8
client_id` + `N-1` bytes of one UDP datagram. `client_id` 0-254: the container end gives each new (address, port) of
a sender an id (the oldest reused); the PC end keeps one UDP socket per id and wraps replies with the same id.
`client_id` 255 = PING (N = 1, no payload), sent every 1 s by both ends; 4 s without any frame = gone. MAVLink: a plain
byte stream (no framing).

**g) The reconnect steps (link.sh; every start and every return after a drop), per link A and B:**

1. make sure the connection exists (`jetson_logged_in`; login mode: wait for the owner, reminder every 30 s);
2. clean, through that connection (it runs as our user on the Jetson):
   `cur=$PPID; read -r p st < D/run/link_X.pid 2>/dev/null; c=$(cat /proc/$p/comm 2>/dev/null); if [ -n "$p" ] && [
   "$p" != "$cur" ] && { [ "$c" = sshd ] || [ "$c" = sshd-session ]; } && [ -O /proc/$p ] && [ "$(awk '{print $22}'
   /proc/$p/stat)" = "$st" ]; then kill "$p"; fi; rm -f <this link's socket files>` (A:
   `mavlink.sock`; B: `cam_sdk.sock`, `cam_http.sock`, `cam_rtsp.sock`; never `tap.sock`, which the sender owns). The
   start-time comparison makes sure a re-used process number can never point at someone else's SSH session (the team
   logs in as the same user: the live probe saw 10 sessions of `astra-dev`);
3. `jetson_forward cancel X <this link's specs>` (errors ignored), then `jetson_forward add X ...`: A = `-R
   D/sock/mavlink.sock:127.0.0.1:<5763+10I> -L 127.0.0.1:7461:D/sock/tap.sock`; B = `-R
   D/sock/cam_sdk.sock:127.0.0.1:37261 -R D/sock/cam_http.sock:127.0.0.1:37282 -R
   D/sock/cam_rtsp.sock:192.168.144.25:8554`;
4. record: `echo "$PPID $(awk '{print $22}' /proc/$PPID/stat)" > D/run/link_X.pid` through the same connection (the
   shell's parent is that connection's sshd; VERIFIED for a plain session: `comm=sshd`, owner = our user, cmdline
   `sshd: jet@notty`, and `-O` was true for it; EXPECTED the same for a shared connection, which the builder confirms
   with the probe container).

`link.sh stop` cancels our forwards and stops `pc-sdk`/`pc-http`; it never closes the owner's window.

**h) Commands the PC side calls** (all take `--link --host --user --port --auth`): `jetson.sh check --quick --for
flight`, `jetson.sh start --sim --run-id ID` (exit 0 = the container runs and its relay is listening, per
`relay.status`; it does not wait for the team stack), `jetson.sh exec -- CMD...` (CMD's exit code; 13 when the
container is not running; runs as `astra-dev`), `jetson.sh stop`, `jetson.sh clock` (prints `offset_s=... rtt_ms=...`),
`jetson.sh status --json` (`{"link", "auth", "logged_in", "container": "running|stopped|missing", "image",
"git_sha", "run_id", "uptime_s", "l4t", "power_mode", "clocks", "engine": "ok|missing|mismatch", "team_busy",
"mem_avail_mb", "disk_free_gb"}`), `jetson.sh paths` (`KEY=VALUE`: `JETSON_DATA`, `SOCK_DIR`, `PHOTOS`, `DETECTIONS`,
`RUNS`), `jetson.sh target`.

### 15.4 Exit codes of `jetson.sh` (and the errors they come with)

The PC side needs "0 or not 0" plus the printed message, and for `exec` the fixed code 13.

| Code | Meaning | The message says how to fix it |
|---|---|---|
| 0 | OK | |
| 1 | a step failed (the message names it) | |
| 2 | the command was typed wrong | `--help` |
| 3 | host unreachable (no ping, no TCP 22) | cable / power / address / `find` |
| 4 | host answers, SSH port 22 closed | boot finished? sshd on? |
| 5 | SSH login refused with the key (key mode) | `setup --key` again, or login mode |
| 6 | host key changed | the `ssh-keygen -R` line |
| 7 | not a supported board (not aarch64 / not L4T R36.4 / not 22.04) | which release was found |
| 8 | Docker missing, stopped, or no access without a password | the `usermod -aG docker` line for the owner |
| 9 | NVIDIA runtime missing / GPU test failed | `apt install nvidia-container` for the owner |
| 10 | not enough disk (the reserve would be crossed) | sizes, `image prune` |
| 11 | not enough memory | close apps / headless |
| 12 | image not deployed (or older than the PC's code, for `start`) | `image build`, `deploy` |
| 13 | container not running / exited at start | `logs` |
| 14 | link (forward) could not be opened, or did not come back | `link.sh status`, cable, check row 12 |
| 15 | team stack not ready (`stack_check` failed) | `logs`, `RUN_DIR/team_stack.log` |
| 16 | the team is using the board (refused) | 9.6 |
| 17 | not logged in | `jetson.sh login` |
| 18 | a PC prerequisite is missing (Docker Engine, buildx, arm64 emulation, registry) | 9.5 |
| 19 | TensorRT engine missing or built for another board / JetPack / model | `jetson.sh warmup` |

The container entrypoint uses 20 for the safety guard of section 6. Warnings never change the exit code (clock
offset, power mode, desktop memory, slow link).

### 15.5 Decisions the owner makes (do not guess)

1. Which Jetson and which user (`suas-jetson` with `astra-obs-avd`, or the flight Jetson with `astra-dev`), whether we
   may use it, and when (an idle window the team agreed to).
2. Login mode (default) or key mode (adds one line to the team account's `authorized_keys`).
3. Docker access without a password on the board (docker group, root-equivalent; or passwordless sudo for docker):
   the team decides and types it.
4. The field power mode (the tools only report it).
5. The disk reserve (default max(100 GB, 25 %)).
6. REAL mode: the photo-folder policy (refuse / archive / keep by default) and the team's root folder.
7. `--jetson-baud 115200` (like TELEM2) or `off` as the default.
8. Optional: the permanent arm64 emulation in WSL (`apt install qemu-user-static binfmt-support`) instead of the
   automatic registration before each build; mirrored WSL networking only if risk 1 happens.
9. Any Windows-side change (RNDIS driver pick, Ethernet adapter address): the owner does it from the written steps.
10. The new video pipeline in HIL (6.3): whether the team adopts the proposed field launch file (until then it is a
    listed difference), and the detector image (the team's `yolo26-ros-subscriber` as it is on the board, or a copy
    we build off the board from the team's Dockerfile). Shared with the video-pipeline workflow (VIDEO_PIPELINE.md).
11. The open question for the team: how the yolo26 detections reach main_controller (6.3).
12. (revision 4) MAVROS exactly 2.12.0 in the image: only possible by building MAVROS 2.12.0 (+ mavros_msgs,
    libmavconn) from the source tag inside layer 3, because no apt repository or snapshot carries it. It makes the
    x86 build ~10-15 minutes longer and the arm64 build (QEMU) much longer. Until decided, 2.14.0 stays a listed
    difference (between the two, mavros_msgs only gained ObstacleDistance3D).

### 15.6 Patches to files other streams own

The docker stream is listed as done in `D:\suas_scratch\coord\streams_done.txt` (11:50, commit 0e49505), so its
files are free: builder 1 re-reads them and applies these changes itself as small surgical edits (or writes them as
exact PATCHes in its report if the lead says otherwise). `BASE_IMAGE`, `TORCH_INDEX_URL` and `TORCH_VERSION` must
stay as they are (they exist today: lines 42, 93 and 94 of `docker/Dockerfile`).

- **Revision 3: P1 is dropped.** `jetson.sh image build` builds layer 2 with a direct `docker build --target dev -f
  docker/Dockerfile --build-arg BASE_IMAGE=... --build-arg TORCH_INDEX_URL=...` (arm64 and the x86 TensorRT base), so
  build_images.sh stays unchanged. P2 stays a proposed patch (in the builder's report); the hashes are in
  `simulation/jetson/image/jetson_pins.env`. The original P1 text, for the record:
- ~~**P1 `simulation/docker/build_images.sh`:**~~ new options `--platform P` (e.g. `linux/arm64`; only the `dev` target
  allowed when it is not the PC's own platform), `--base IMAGE` (passed as `--build-arg BASE_IMAGE=`; the expired-key
  date check then looks at that image), `--torch-index URL` (overrides `--torch`; label `org.astra.torch=custom`),
  `--tag T` (images `astra-<target>:T` instead of `:latest`); with a foreign platform, the end-of-build check skips
  the GPU test (it runs under QEMU without a GPU).
- **P2 `docker/Dockerfile`:** optional build arguments `TORCH_SHA256` and `TORCHVISION_SHA256`; when set, torch and
  torchvision are installed from a generated requirements file with `--hash=sha256:...` (pip's hash-checking mode);
  empty = today's behaviour.
- No patch is needed in `sim_bringup` (the gazebo-pov stream is done; phase_monitor stays unchanged thanks to the
  presence nodes) nor in the team packages: the team code runs unchanged.

## 16. Build order and acceptance

1. **JETSON-SIDE, first (the owner's priority):** `lib/target.sh`, `login_window.sh`, `jetson.sh login|logout|check|
   setup|status|find|clock`, the facts check with tests, README "Plug in and test" -> **ready for the owner's first
   plug-in**: `check` reports every row with a fix, nothing is installed on the board.
2. **Then in parallel.** JETSON-SIDE: layer 1 + 3 Dockerfiles (OpenCV dry run first), `selftest`, `warmup`,
   `hil_relay.py` with tests, P1/P2, `image build`, `deploy`, `warmup`, `start/stop/logs/exec/shell/paths/target`.
   PC-SIDE: `link.sh`, `ros_tap.py` + `tap_topics.txt`, `collect.sh`, `hil_steps.sh`, `windows_link_check.ps1`, the
   demo.sh options, the `csv_export` clock offset, tests (including the reconnect steps against the throwaway sshd
   container), `FLYING.md`.
3. Virtual image `astra-hil:virtual` built on this PC (heavy job, both locks).
4. **ONE visible flight: `demo.sh runway --jetson virtual`** with the simulator's default camera pipeline (window
   visible, both locks). If the video-pipeline workflow's RTSP stream and ZR10 answers are not in yet, this flight
   uses the legacy photo option (`zr10` profile) and the new-pipeline flight follows as soon as they are. Accept
   when: the report shows the detections made in the container (legacy: the targets found by new_od and the drops;
   new pipeline: `/detection/results` received on the PC, and drops only if the team's wiring exists, 6.3);
   `RUN_DIR/jetson/` is filled (summary with the "Differences from the field" list); Mission Planner showed the team
   texts; and one `link.sh drop 20 --mode blackhole` during the search lanes ended with the relay re-dialing (MAVLink,
   SDK, and the RTSP stream coming back), MAVROS connected again and the flight completing (the summary shows the
   drop, the gate's dropped bytes and no late commands).
5. **On the board (the owner, with the team's OK, in an idle window where the team's own stack is stopped):** `login`,
   `check`, `setup`, `image build` (PC, arm64), `deploy`, `warmup` (if the pipeline needs it), then `demo.sh runway
   --jetson usb`. Accept when the same report appears,
   `tegrastats.log` shows GPU load during the lanes, the power mode is recorded at start and end, and **a real cable
   pull** passes: pull the USB-C cable for about 20 s during the search lanes, plug it back in, log in again in the
   window (login mode); the link comes back, `link.log` shows the old session killed and the relay re-dial, MAVROS is
   connected again, and the flight completes. Then one short `check` + flight with `--jetson ssh --jetson-host HOST`
   over Ethernet or Wi-Fi, to prove the no-USB path.

## 17. Evidence (what was verified today, 2026-10-31, on this PC)

Scripts and logs are in `D:\suas_scratch\jetson_design\` (throwaway; not part of the repo). Nothing was run on a
Jetson.

1. **Camera address in a private container network** (`jet_netns_probe.sh`, `jet_netns_probe2.sh`, log
   `jet_netns_probe.log`): image `ros:humble-ros-base-jammy`, `docker run --network none --cap-add NET_ADMIN`; a
   busybox sidecar sharing its network ran `ip addr add 192.168.144.25/32 dev lo`; output `inet 192.168.144.25/32
   scope global lo`; `ip_unprivileged_port_start` = 0 inside; as uid 1000: `UDP ok, reply 556601010000000c0034ce` and
   `TCP ok: b'GET /'`.
2. **ROS 2 in that container:** `ros2 topic pub` + `ros2 topic echo --once /probe` printed `data: hello`, exit 0, with
   `ROS_LOCALHOST_ONLY=1` and `=0` (one user).
3. **Unix sockets across the container boundary** (`jet_sockdir_probe.sh`, log `jet_sockdir_probe.log`): a socket made
   on the WSL host with mode `srw-------` (as sshd makes them, `StreamLocalBindMask 0177`) in a folder mounted into a
   `--network none` container; a relay in the container on `192.168.144.25:82` passed an HTTP request to it and back:
   `client got: b'HTTP/1.0 200 OK ... host-socket-got:GET /cgi-bin/media.c'` as uid 0 and as uid 1000.
4. **The forward shape with real OpenSSH** (`jet_ssh_r_probe.sh`, log `jet_ssh_r_probe.log`): client OpenSSH 8.9p1 in
   WSL, server OpenSSH from Ubuntu 22.04 (the same release as JetPack 6's Ubuntu) in a throwaway container. `sshd -T`:
   `streamlocalbindmask 0177`, `allowstreamlocalforwarding yes`, `streamlocalbindunlink no`, `clientaliveinterval 0`.
   `-R .../mavlink.sock:127.0.0.1:15763` -> on the "Jetson" `jetson side got: b'sitl:heartbeat'`; `-L
   127.0.0.1:17461:.../tap.sock` -> on the PC `pc side got: b'tap-frame-from-jetson'`; after killing the tunnel the
   socket file stayed and the next `-R` failed: `Error: remote port forwarding failed for listen path ...`, `ssh exit
   255`; after `rm -f` the reconnect worked: `jetson side got after reconnect: b'sitl:again'`.
5. **Photo sizes and this PC** (`jet_measure.sh`, log `jet_measure.log`): runs `20261031-091316_runway` to
   `20261031-101431_runway` (zr10): 93-124 photos, average 1322-1323 KiB, min 1296, max 1398, size (2560, 1440). WSL is
   in NAT mode (`eth0 172.24.134.79/20`, `default via 172.24.128.1`), DNS `nameserver 10.255.255.254` (DNS tunneling).
   Docker Engine `29.9.1`, runtimes `nvidia runc`, root `/var/lib/docker`, buildx `v0.38.0`. Tools: `ssh`
   (OpenSSH_8.9p1), `rsync 3.2.7`; missing: `socat`, `zstd`, `autossh`, `sshd` (so the design uses only Python
   standard library relays and gzip). (Revision 1 also called the `aarch64` binfmt entry VERIFIED; item 8 corrects
   that.)
6. **A silent link on the Jetson side, and the fix** (`jet_halfopen_probe.sh`, log `jet_halfopen_probe.log`; same
   throwaway sshd as item 4, `clientaliveinterval 0`, `tcpkeepalive yes`): the link session's remote command recorded
   `$PPID` = `3515`, `comm=sshd owner=jet cmdline=sshd: jet@notty`. A relay stand-in (uid 1000) on `mavlink.sock`
   received the PC server's data (`got b'hb1;'`). The PC ssh was frozen with SIGSTOP at `...917.96` (silent link, no
   FIN/RST): 19 s later the relay log still had no end-of-file and `3515 jet sshd: jet@notty` was still alive. `rm -f`
   + a new `-R` session: worked ("session 2 alive: yes"), but the relay stayed stuck on its first connection. Then
   `kill 3515` by the recorded PID, through a second session, after checking `comm` = sshd and `-O` (ours): `killed
   3515` at `...937.02`; relay log: `...937.17 EOF on dial #1`, `...937.17 dialed #2`, `got b'hb2;'` (the new session's
   data). (`ss` was not installed in that container, so its "none" line means nothing.)
7. **Docker IPC and Fast DDS between users** (`jet_ipc_dds_probe.sh`, log `jet_ipc_dds_probe.log`; a fresh `--network
   none` `ros:humble-ros-base-jammy` container per pair, plain rclpy scripts): `IpcMode: private ShmSize: 67108864`,
   `/dev/shm tmpfs ... size=65536k`, interfaces `lo` only. Publisher root -> listener uid 1000: `received: NOTHING
   after 10.0 s`; uid 1000 -> root: `NOTHING after 10.0 s`; uid 1000 -> uid 1000: `received: from-1000 after 0.0 s`;
   root -> root: `received: from-root after 0.2 s`. (A first attempt with the `ros2` CLI was discarded: its daemon is
   shared between users and failed on its own.)
8. **The arm64 handler in WSL** (`jet_info_probe.sh`, log `jet_info_probe.log`): `/proc/sys/fs/binfmt_misc/aarch64`
   `enabled`, `interpreter /usr/bin/qemu-aarch64`, `flags: POCF` (plus `arm mips64 mips64le ppc64le riscv64 s390x`
   entries); `ls /usr/bin/qemu-aarch64*`: no such file; no `qemu-user*` / `binfmt-support` package; `/usr/lib/binfmt.d`
   has only `python3.10.conf`; `/mnt/wsl/docker-desktop` exists; PID 1 = `systemd`, `/etc/wsl.conf` `systemd=true`;
   `systemd-binfmt` `static / inactive`. Images on the WSL Docker Engine: `ros:humble-ros-base-jammy`, `ubuntu:22.04`,
   `hello-world`, `busybox:1.36` (no `astra-*` image built yet); 924 GB free under `/var/lib/docker`.
9. **4K photo sizes** (same probe): run `20261024-091528_runway` (a8mini, 3840 wide): `photos=95 avg=2354KiB
   min=2295KiB max=2547KiB`.
10. **Ports** (same probe and a repo search): nothing listens on `29517`, `7461`, `37261`, `37282` in WSL; none of
    `29517` / `37282` appears in the repo; temporary port range `32768 60999`.
11. **Our SITL parameter file** (`sim_models/config/astra_sitl.parm`, same probe): no `SERIAL2_*`, `SIM_BAUD*` or
    `SR2_*` lines; line 60 `FS_GCS_ENABLE 0`.
12. **The new pipeline's sources** (read, not run; `D:\suas_scratch\astra_new_pipeline\20261031-1151\
    astra_camera_detection\astra_ws\`): `zr10_stream/scripts/stream_node.py` line 41 `'-rtsp_transport', 'tcp', '-i',
    url` with default `rtsp://192.168.144.25:8554/main.264`; `zr10_ros2/config/zr10.yaml` `camera_port: 37260`,
    `media_base: "http://192.168.144.25:82/cgi-bin/media.cgi"`; `astra_live_detection/live_node.py` lines 59-61
    publishers `detection/results` (`Detection2DArray`), `detection/annotated` (`Image`), `detection/info` (`String`);
    `STREAMING.md`: `/zr10/image_raw` rgb8 1280x720, "Header stamp: host publication time, not camera exposure time".
    Board facts (L4T R36.4.7, domain 0, eth0 `192.168.144.20`, the detector container's `--network host`, the team
    stack running) are the live probe's (`D:\suas_scratch\coord\jetson_probe.md`, 11:47), not measured by this work.

Read in code (not run): ArduPilot `libraries/AP_HAL_SITL/UARTDriver.cpp` at `Copter-4.6.3` (tcpclient: 3 tries, then
`exit(1)`, no re-dial; the TCP server serves one client and accepts again after end-of-file; the baud limiter uses
`_uart_baudrate * 0.1` bytes/s when `telem_baudlimit_enable` is set); `libraries/SITL/SITL.cpp` (`BAUDLIMIT_EN`,
index 28, default 0, "SITL enable bandwidth limitting on telemetry ports"); `libraries/GCS_MAVLink/GCS_Common.cpp`
(`set_mavlink_message_id_interval(MAVLINK_MSG_ID_HEARTBEAT, 1000)` in `initialise_message_intervals_from_streamrates`;
HEARTBEAT not in any stream); MAVROS `mavros_router.hpp` (ros2 branch: endpoint reconnect every 30 s); libmavconn URL
forms; our repo: `launcher.sh` lines 28-34 and 65-71, `sim_demo/ros_common.py` lines 103-118, `sim_bringup/phase_monitor.py`
lines 299-347, `sim_demo/photo_geometry.py`, `pose_history.py` lines 139-179, `photo_info.py` lines 110-140,
`photos.py` (the watcher), `csv_export.py` line 263, `score_run.py` (arrival times), `sim_bringup/stack_check.py`
(`TEAM_NODES`), `simulation/docker/build_images.sh` (options, labels, checks), `docker/Dockerfile` (`ARG BASE_IMAGE`
line 42, `TORCH_INDEX_URL` 93, `TORCH_VERSION` 94, `USERNAME=ubuntu` 96), `sim_plugins/autopilot.py`
(`autopilot.param_files`). Facts from the reviewer's own checks that were not repeated here are marked "reviewer's
check" where they are used (MAVROS arm64 packages, l4t-jetpack's OpenCV and TensorRT Python packages, the torch
wheel's libraries and hash prefix).

**Evidence added in revision 4 (2026-10-31 afternoon, this PC, no board):**

13. **MAVROS in the ROS snapshots** (`D:\suas_scratch\e2e\jet_snap_probe.sh`, `jet_snap_probe2.sh`, read-only web
    requests): snapshots.ros.org/humble lists ... `2025-06-18/ 2025-08-15/ 2026-01-07/ 2026-03-29/ ...`; 2025-08-15
    amd64 + arm64 `ros-humble-mavros 2.10.1-1jammy.20250728...`; 2026-01-07 `2.14.0-1jammy.20251228...`
    (arm64 `...20251229...`); the live repository `ros-humble-mavros 2.16.0-1jammy.20260927.165913`. The board has
    `2.12.0-1jammy.20251017.223025` (live probe). No apt source has 2.12.0.
14. **The x86 TensorRT image builds** (the PC-side builder's job `jet_pcflight.sh`, log `jet_pcflight_image.log`,
    15:18-15:31, using these image files before the revision-4 edits): layer 2 on `astra-hil-trt-ros:24.08` and layer 3
    built; colcon `Summary: 9 packages finished [15.0s]` (the for-loop exit-status bug of the 12:37 attempt is fixed).
    Its selftest: `torch 2.8.0+cu126 (CUDA 12.6), NVIDIA GeForce RTX 4060 Laptop GPU`, `TensorRT (Python): 10.3.0`,
    `numpy: 1.26.4`, `OpenCV: python cv2 4.10.0; ... libopencv-core4.5d=4.5.4+dfsg-9ubuntu4`, all team packages +
    mavros + mavros_msgs; the container's versions: `OS=Ubuntu 22.04.4 LTS; MAVROS=2.14.0; TORCH_CUDA=12.6`.
15. **The TensorRT export failed in that image** (same job, step F3 `warmup --link virtual`): `import onnx` ->
    `/usr/lib/python3/dist-packages/onnx/mapping.py ... np.dtype(np.object)` -> `AttributeError: module 'numpy' has no
    attribute 'object'`; `jet_onnx_probe.sh`: `python3-onnx 1.7.0+dfsg-3ubuntu2`, pip `onnx 1.7.0`, `protobuf 3.12.4`,
    `numpy 1.26.4`, `ultralytics 8.4.75` (its ONNX export asks for `onnx>=1.12.0,<2.0.0`, and with autoinstall off
    `check_requirements` only warns). The same layer 2 is under the Jetson image, so warmup on the BOARD would have
    failed the same way. Fix in layer 3 (TensorRT images): the team's own pins `onnx==1.14.0`
    (`ros2_ws/src/ultralytics_ros/requirements.txt`) and `protobuf==3.20.3` (`ros2_ws/src/fixtensorRT.md`); selftest
    has a new "ONNX (for the TensorRT export)" row.
16. **Static checks of the image files** (`jet_lint_docker.sh`, `jet_lint_l4t.sh`): `docker buildx build --check`:
    layer 1 x86 (TensorRT 24.08 base) and layer 3 (on `astra-dev:hil-x86`): `Check complete, no warnings found.`;
    layer 1 arm64: the base `nvcr.io/nvidia/l4t-jetpack:r36.4.0` resolves (only `InvalidBaseImagePlatform`: the base
    is arm64-only, as intended); NGC tags: `r36.4.0 exists`, `r36.4.3 not found`, `r36.4.4 not found`, `r36.3.0
    exists`. hadolint v2.12.0: DL3006 (FROM with an ARG), DL3002 (last USER root: the entrypoint drops to astra-dev
    itself), DL3048 (label keys with `_`), SC2086 (`$pkgs` is split on purpose); nothing else.
17. **The code fingerprint** (`jet_codesha.sh`): the Windows folder twice and an rsync copy made like 03_build.sh's
    mirror all gave `c0cc2af252c9` (3.7 s on `/mnt/d`).

## 18. Sources

- NVIDIA Jetson Orin Nano Developer Kit user guide, hardware layout (USB-C device mode, RNDIS, 192.168.55.1, USB
  serial, power jack): https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/hardware_layout.html
- NVIDIA Jetson Linux R36.4 developer guide, platform power and performance (Orin Nano 8GB modes 0 = 15W, 1 = 7W,
  2 = MaxN, MaxN only with the "super" flash configuration; `sudo /usr/sbin/nvpmodel -q`):
  https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/SD/PlatformPowerAndPerformance/JetsonOrinNanoSeriesJetsonOrinNxSeriesAndJetsonAgxOrinSeries.html
- RidgeRun, Jetson Orin performance tuning (`sudo jetson_clocks --show`, `sudo nvpmodel -q`):
  https://developer.ridgerun.com/wiki/index.php/NVIDIA_Jetson_Orin/JetPack_5.0/Performance_Tuning/Maximizing_Performance
- NVIDIA forum, "JP6 has no IP over USB 192.168.55.1 anymore" (custom carriers vs the devkit, device tree):
  https://forums.developer.nvidia.com/t/jp6-has-no-ip-over-usb-192-168-55-1-anymore/296062
- NVIDIA forum, USB device mode scripts and service (`/opt/nvidia/l4t-usb-device-mode/`,
  `nv-l4t-usb-device-mode.service`): https://forums.developer.nvidia.com/t/usb-device-port-configuration/240909
- NVIDIA forum, host address 192.168.55.100 on the USB network:
  https://forums.developer.nvidia.com/t/how-to-use-a603s-usb-b-port-in-device-mode/276011
- NVIDIA forum, Windows + USB-C SSH troubleshooting (cable, driver, ping first):
  https://forums.developer.nvidia.com/t/windows-is-killing-ssh-connection/181670
- NVIDIA NGC, l4t-jetpack container (contents, r36.4.0 tag, 5.22 GB compressed, `--runtime nvidia`, `apt install
  nvidia-container`): https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-jetpack
- NVIDIA NGC, l4t-base (since r34.1 CUDA/cuDNN/TensorRT are not brought from the host):
  https://catalog.ngc.nvidia.com/orgs/nvidia/containers/l4t-base
- dusty-nv jetson-containers (community ROS images for L4T; considered, not used as the base):
  https://github.com/dusty-nv/jetson-containers and https://hub.docker.com/r/dustynv/ros/tags
- Jetson AI Lab pip index for JetPack 6 / CUDA 12.6 (torch 2.8.0, torchvision 0.23.0; already the team's
  `requirements_jetson.txt` index): https://pypi.jetson-ai-lab.io/jp6/cu126
- Ultralytics, NVIDIA Jetson guide (TensorRT is the fastest format on Jetson; export on the device):
  https://docs.ultralytics.com/guides/nvidia-jetson/
- NVIDIA forum, torch/torchvision on JetPack 6.2.x Orin Nano (torch 2.8.0 / torchvision 0.23.0 wheels):
  https://forums.developer.nvidia.com/t/jetson-orin-nano-issues-running-yolo-ultralytics-due-to-pytorch-torchvision-compatibility-jetpack-6-x-cuda-12-6/365683
- Microsoft, WSL networking (NAT, reaching other machines from Linux, `netsh portproxy` for inbound, mirrored mode,
  DNS tunneling, Hyper-V firewall): https://learn.microsoft.com/en-us/windows/wsl/networking
- WSL issue, Docker ports with mirrored mode: https://github.com/microsoft/WSL/issues/10683
- ArduPilot SITL serial options (`tcp:PORT:wait`, `tcpclient:IP:PORT`, `udpclient:IP:PORT`, SERIAL1/2 = TCP
  5762/5763): https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html and
  https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- ArduPilot source, Copter-4.6.3: SITL UART driver (tcpclient behaviour, one TCP client at a time, baud limiter):
  https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/libraries/AP_HAL_SITL/UARTDriver.cpp ; SITL parameters
  (`SIM_BAUDLIMIT_EN`): https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/libraries/SITL/SITL.cpp ; MAVLink
  message intervals (HEARTBEAT 1000 ms on every channel):
  https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/libraries/GCS_MAVLink/GCS_Common.cpp
- MAVROS, libmavconn URL schemes: https://github.com/mavlink/mavros/blob/ros2/libmavconn/README.md
- MAVROS, router reconnect timer (30 s):
  https://github.com/mavlink/mavros/blob/ros2/mavros/include/mavros/mavros_router.hpp and
  https://github.com/mavlink/mavros/blob/ros2/mavros/src/lib/mavros_router.cpp
- OpenSSH manual pages: `ssh -M -S`, `-O check|forward|cancel|exit`, `-R remote_socket:host:hostport`, `-L`:
  https://man.openbsd.org/ssh ; `ControlMaster`, `ControlPath`, `ControlPersist`, `ExitOnForwardFailure`,
  `ServerAliveInterval`: https://man.openbsd.org/ssh_config ; `ClientAliveInterval` (default 0), `TCPKeepAlive`,
  `StreamLocalBindUnlink`, `StreamLocalBindMask`, `AllowStreamLocalForwarding`, `DisableForwarding`:
  https://man.openbsd.org/sshd_config
- OpenSSH mailing list, `StreamLocalBindUnlink` must be set on the server for `-R` (we avoid it with `rm -f`):
  https://lists.mindrot.org/pipermail/openssh-unix-dev/2016-April/035062.html
- zenoh-bridge-ros2dds (considered for the ROS bridge, not chosen: tested with CycloneDDS):
  https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds
- Docker: `docker image save` / `load`: https://docs.docker.com/reference/cli/docker/image/save/ and
  https://docs.docker.com/reference/cli/docker/image/load/ ; multi-platform builds (buildx, QEMU):
  https://docs.docker.com/build/building/multi-platform/ ; registries on 127.0.0.0/8 treated as insecure (plain HTTP
  allowed) by default: https://docs.docker.com/reference/cli/dockerd/#insecure-registries ; the `--ipc` modes of
  `docker run`: https://docs.docker.com/reference/cli/docker/container/run/#ipc
- tonistiigi/binfmt (QEMU registration for buildx): https://github.com/tonistiigi/binfmt ; tags (`qemu-v10.2.3`,
  2026-06-08): https://hub.docker.com/r/tonistiigi/binfmt/tags
- The owner's own Jetson guide (read-only reference, team imaging repo, not copied) and the read-only probe report
  `D:\suas_scratch\coord\jetson_probe.md` (board not reachable on 2026-10-31; last known facts from 3 Oct: JetPack
  6.2.1 / L4T R36.4.4, 7.6 GB usable / ~5.5 GB free, Docker 28 with NVIDIA support, user not in the docker group,
  431 GB free, 25 W mode, USB-C 192.168.55.1, Tailscale `suas-jetson`). The repo's `docker/Dockerfile` header records
  L4T R36.4.7 read off a Jetson on 2026-10-10: `jetson.sh check` reports what the connected board really runs.
- The lead's login reference: `D:\suas_scratch\e2e\jetson_login.sh` (owner entry 11:40).
