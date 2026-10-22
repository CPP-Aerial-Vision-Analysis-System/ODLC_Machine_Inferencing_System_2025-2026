# siyi_emulator: a pretend SIYI camera for the simulator

Last year's camera driver (`video_cam`, executable `siyi`) talks to the real SIYI gimbal camera over the
network. In the simulator there is no real camera, so this package **pretends to be one**. It answers
on the same address and ports as the real camera, so `video_cam` runs **unchanged, with its default
parameters**, and gets real photos rendered by Gazebo.

```
 video_cam (team code, unchanged)              siyi_emulator (this package)                    Gazebo
 ----------------------------------            --------------------------------------         -------------------------
 UDP 192.168.144.25:37260  SIYI SDK  <------>  answers commands, moves the gimbal   ------->  /sim/gimbal/*/cmd_pos
                                                (stabilised, 90 deg/s)               <-------  /sim/gimbal/joint_states
                                                                                     <-------  /sim/drone/odometry
 0x0C "take photo"          ------------>      triggers ONE photo frame            ------->  /sim/camera/photo/trigger
 0x0B "photo OK" (< 0.8 s)  <------------      zoom (crop) + JPEG -> SD card       <-------  /sim/camera/photo/image
 TCP 192.168.144.25:82  media API  <------>    lists + serves IMG_NNNN.jpg
```

## Quick start (copy and paste)

```bash
# once per boot: give this computer the camera's address (needs your password)
sudo bash $(ros2 pkg prefix siyi_emulator)/share/siyi_emulator/scripts/add_camera_ip.sh

# start the emulator (Gazebo with the drone should already be running)
ros2 run siyi_emulator siyi_emulator

# in another terminal: the team's camera driver, exactly as in flight
ros2 run video_cam siyi
ros2 topic pub --once /camera/trigger std_msgs/msg/Bool "{data: true}"
#   -> the photo appears in <workspace>/src/video_cam/mapping_photos/
```

Poke the camera by hand (works against a real SIYI camera too):

```bash
ros2 run siyi_emulator siyi_client hwid            # "6B0000000001" = ZR10, "73..." = A8 mini
ros2 run siyi_emulator siyi_client angles 30 -45   # yaw 30 deg LEFT, pitch 45 deg down
ros2 run siyi_emulator siyi_client photo --out /tmp/p.jpg
```

Open `http://192.168.144.25:82/` in a browser to see the emulated SD card.

## How a photo flows (and why the timings matter)

1. `video_cam` sends `0x0C` (take photo) and waits **0.8 s** for a `0x0B` answer.
2. The emulator publishes one message on `/sim/camera/photo/trigger`; the triggered Gazebo camera
   renders exactly one frame on `/sim/camera/photo/image`.
3. As soon as the frame arrives (or after `feedback_deadline_s` = 0.5 s at the latest) the emulator
   pushes `0x0B info_type=0` ("photo captured"), so `video_cam` never hits its timeout.
4. The current zoom is applied (centre crop + resize = digital zoom), the JPEG (quality 92) is written
   atomically to the emulated SD card as `101SIYI_IMG/IMG_NNNN.jpg`, plus `IMG_NNNN.json` with the camera
   pose, gimbal angles, zoom and field of view.
5. `video_cam` waits 0.5 s, polls `getmedialist` every 0.5 s, sees the new file, downloads it over HTTP,
   checks it (size, 2K/4K profile) and saves it as `"<lat> , <lon>.jpg"` for the detector.

If Gazebo sends no frame within `photo_timeout_s` (10 s; the trigger is re-sent after 3 s), the emulator
saves a clearly labelled **test pattern** instead ("NO GAZEBO PHOTO FRAME") so the problem is visible,
or, with `test_pattern_fallback:=false`, loses the photo like a real camera would. A render normally takes
0.05-0.3 s; on a busy computer with software rendering it took over 3 s, which is why the wait is long
(video_cam keeps polling the listing for 15 s, so a late photo is still found).

## Strict or permissive (the fidelity switch)

| `strict` | what happens | use it for |
|---|---|---|
| `false` (default) | every command is answered | demos: captures flow at 1 Hz |
| `true` | silent on 0x04, 0x06, 0x16, 0x18, 0x19, exactly like a real **A8 mini** (SIYI SDK support table) | seeing what `video_cam` does on an A8 mini: 2 s timeouts, "Lens busy" dropped triggers |

`0x0C` and `0x48` are never acknowledged (also true on the real camera).

What the team's `video_cam` does with `camera_model:=a8mini strict:=true` (measured): it detects
`A8MINI`, saves 4K photos, but `capture_setup` spends about 22 s in three unconfirmed zoom attempts
("camera reports Nonex") and an unacknowledged focus, and 5 of 7 triggers sent meanwhile are dropped
with "Lens busy (zoom/focus) - dropping capture request".

## Camera models

| `camera_model` | hardware ID | photo size | zoom | 0x41 reply | 0x0A length |
|---|---|---|---|---|---|
| `zr10` (default) | `6B...` | 2560x1440 (video_cam "2K") | 1-30x (10x optical) | 0x41 | 8 |
| `a8mini` | `73...` | 3840x2160 (video_cam "4K") | none in 4K mode (`max_zoom:=6` to allow) | **0x0E** | 7 |

Both: yaw -135..+135 deg, pitch -90..+25 deg (commands are clamped), slew 90 deg/s. The Gazebo photo
camera must render the same size (the bringup generates the drone with `photo_width/photo_height`);
otherwise the frame is resized and a warning is printed once.

## Interface

### SIYI SDK, UDP `camera_ip:37260` (frames `55 66 | ctrl | len | seq | cmd | data | CRC16-XMODEM`)

Replies echo the request SEQ and use CTRL = 0x02. All numbers little-endian. Angles in 0.1 deg.

| cmd | name | request data | reply data | strict |
|---|---|---|---|---|
| 0x01 | firmware version | - | uint32 x3 (ZR10) / x2 (A8 mini) | answered |
| 0x02 | hardware ID | - | 12 ASCII chars, first two = model ('6B' / '73') | answered |
| 0x04 | auto focus | `<BHH` 1, x, y | `01` | **silent** |
| 0x05 | manual zoom | `<b` 1 in / 0 stop / -1 out | `<H` zoom*10 (now) | answered |
| 0x06 | manual focus | `<b` 1 / 0 / -1 | `01` | **silent** |
| 0x07 | gimbal speed | `<bb` yaw, pitch -100..100 (yaw + = RIGHT) | `01` | answered |
| 0x08 | centre | `<B` 1 centre, 2 centre-down | `01` | answered |
| 0x0A | system info | - | `[0]0 [1]hdr [2]0 [3]record_sta [4]mode [5]mount [6]video [7]linkage` | answered |
| 0x0B | function feedback | (pushed by the camera) | `<B` 0 photo OK, 1 photo failed (no TF card) | - |
| 0x0C | photo / record / mode | `<B` func: 0 photo, 2 record, 3 lock, 4 follow, 5 fpv, 9 look down | none (0x0B after a photo) | - |
| 0x0D | attitude | - | `<hhhhhh` yaw, pitch, roll, 3 rates (0.1 deg/s) | answered |
| 0x0E | set angles | `<hh` yaw, pitch | `<hhh` CURRENT yaw, pitch, roll | answered |
| 0x0F | absolute zoom | `<BB` integer, tenths | `01` | answered |
| 0x16 | max zoom | - | `<BB` integer, tenths | **silent** |
| 0x18 | current zoom | - | `<BB` integer, tenths | **silent** |
| 0x19 | gimbal mode | - | `<B` 0 lock, 1 follow, 2 fpv | **silent** |
| 0x41 | single axis | `<hB` angle, 0 yaw / 1 pitch | as 0x0E (cmd 0x0E on the A8 mini) | answered |
| 0x48 | format SD card | - | none; deletes every photo | - |

Signs: yaw + = camera turns LEFT, pitch + = up, pitch -90 = straight down. Reported yaw is relative to
the drone (encoder); reported pitch/roll are relative to the horizon (camera IMU). Unknown or broken
frames are ignored with a one-time warning; nothing can crash the emulator.

### Media HTTP, TCP `camera_ip:82`

| request | answer |
|---|---|
| `GET /cgi-bin/media.cgi/api/v1/getdirectories?media_type=0` | `{"code":200,"success":true,"message":"","data":{"directories":[{"name":"101SIYI_IMG","path":"101SIYI_IMG"}]}}` |
| `GET .../getmedialist?media_type=0&path=101SIYI_IMG&start=0&count=9999` | `{"code":200,"success":true,"message":"","data":{"list":[{"name":"IMG_0001.jpg","url":"http://<host>/101SIYI_IMG/IMG_0001.jpg","size":1234567}],"total":1}}` (oldest first) |
| `GET .../getmediacount?media_type=0&path=101SIYI_IMG` | `{"code":200,"success":true,"message":"","data":{"count":1}}` |
| `GET /101SIYI_IMG/IMG_0001.jpg` | the JPEG (`image/jpeg`), 404 if missing |
| `GET /` | a small web page listing the card |

Wrong `media_type` or path: `"success": false`, code 400, "Invalid media type" / "path not exist".
`media_type=1` (videos) gives empty lists. One directory for the whole run (video_cam only watches the last one).

### Gazebo topics (gz-transport, no ROS bridge needed)

| topic (parameter) | type | direction |
|---|---|---|
| `/sim/gimbal/yaw/cmd_pos`, `/roll/`, `/pitch/` (`gz_*_cmd_topic`) | gz.msgs.Double, rad | out, 50 Hz |
| `/sim/camera/photo/trigger` (`gz_photo_trigger_topic`) | gz.msgs.Boolean | out, one per photo |
| `/sim/gimbal/joint_states` (`gz_joint_states_topic`) | gz.msgs.Model | in (measured angles for 0x0D) |
| `/sim/drone/odometry` (`gz_odometry_topic`) | gz.msgs.Odometry | in (attitude for stabilisation) |
| `/sim/camera/photo/image` (`gz_photo_image_topic`) | gz.msgs.Image | in |
| `/sim/camera/photo/camera_info` (`gz_photo_camera_info_topic`) | gz.msgs.CameraInfo | in (metadata) |
| `optical_zoom_topics` (empty by default) | gz.msgs.Double | out, CameraZoomPlugin command |

### ROS topics (simulator-only extras)

| topic | type | what |
|---|---|---|
| `/sim/camera/photo_info` | std_msgs/String (JSON) | one per saved photo: SD name, URL, camera position and look direction (ENU), gimbal angles, zoom, field of view. Reliable + transient local. |
| `/sim/camera/emulator_state` | std_msgs/String (JSON) | 2 Hz: model, gimbal attitude, zoom, Gazebo health, photo count |

### Parameters (`ros2 param describe /siyi_emulator <name>` explains each one)

| name | default | unit / meaning |
|---|---|---|
| `camera_model` | `zr10` | `zr10` or `a8mini` |
| `strict` | `false` | silent where the real A8 mini is silent |
| `max_zoom` | `0.0` | > 0 overrides the model's max zoom (x) |
| `photo_width`, `photo_height` | `0` | > 0 overrides the photo size (px) |
| `camera_ip` | `192.168.144.25` | where clients expect the camera |
| `bind_ip` | `""` | listen address; empty = `camera_ip` if it exists here, else `127.0.0.1` (+ warning) |
| `ctrl_port` | `37260` | UDP SDK port |
| `media_port` | `82` | TCP media port |
| `media_url_host` | `""` | host in photo URLs; empty = the request's Host header |
| `sd_card_dir` | `~/.ros/siyi_emulator/sd_card` | emulated SD card folder |
| `photo_dir_name` | `101SIYI_IMG` | the one photo directory |
| `clear_sd_on_start` | `true` | empty card at every start |
| `restart_numbering_after_format` | `false` | IMG_0001 again after 0x48 |
| `sd_card_present` | `true` | `false` = "no TF card" |
| `jpeg_quality` | `92` | 1-100 |
| `write_sidecar` | `true` | IMG_NNNN.json next to each photo |
| `photo_timeout_s` | `10.0` | s to wait for the Gazebo frame |
| `retrigger_after_s` | `3.0` | s before the trigger is re-sent (max 2x) |
| `feedback_deadline_s` | `0.5` | s; 0x0B is pushed no later than this |
| `test_pattern_fallback` | `true` | no frame -> labelled test pattern |
| `initial_mode` | `follow` | `lock` / `follow` / `fpv` |
| `initial_yaw_deg` | `0.0` | deg, + = left |
| `initial_pitch_deg` | `-90.0` | deg; -90 = straight down (nobody else points the camera in the sim) |
| `max_slew_dps` | `90.0` | deg/s |
| `max_rate_dps` | `90.0` | deg/s for a 0x07 speed of 100 |
| `rate_yaw_sign` | `-1.0` | SIYI speed + = right -> joint + = left |
| `stabilize` | `true` | keep the camera level when the drone tilts |
| `control_rate_hz` | `50.0` | Hz, joint target rate |
| `zoom_speed_x_per_s` | `4.0` | zoom travel speed |
| `use_gazebo` | `true` | `false` = no Gazebo (test patterns) |
| `gz_partition` | `""` | GZ_PARTITION; empty = environment |
| `gz_*_topic` | contract names | see the Gazebo table |
| `yaw_joint`, `roll_joint`, `pitch_joint` | `gimbal_yaw`, `gimbal_roll`, `gimbal_pitch` | joint names in the drone SDF (scoped names like `iris::gimbal_yaw` also match) |
| `optical_zoom_topics` | `""` | comma list of CameraZoomPlugin topics |
| `hfov_rad` | `1.4137` | lens HFOV of the Gazebo photo camera at 1x |
| `optical_zoom_ref_hfov_rad` | `2.0` | CameraZoomPlugin's built-in reference HFOV |
| `log_commands` | `false` | log every SDK command at INFO |
| `publish_photo_info` | `true` | publish `/sim/camera/photo_info` |
| `publish_state_hz` | `2.0` | rate of `/sim/camera/emulator_state` (0 = off) |

Launch with a file or overrides:

```bash
ros2 launch siyi_emulator siyi_emulator.launch.py camera_model:=a8mini strict:=true
```

## Networking (the 192.168.144.25 address and port 82)

* `video_cam` uses `192.168.144.25` by default. `scripts/add_camera_ip.sh` adds it to the loopback
  interface (`ip addr add 192.168.144.25/32 dev lo`, safe to run twice; `scripts/remove_camera_ip.sh`
  undoes it). It is lost at reboot / `wsl --shutdown`.
* Port 82 is below 1024. `simulation/setup/01_install_humble.sh` sets
  `net.ipv4.ip_unprivileged_port_start=80` so normal users may use it; the helper script sets it too.
* Without the address the emulator listens on `127.0.0.1` and says so; then start video_cam with
  `-p camera_ip:=127.0.0.1` (it rewrites URLs itself).
* On WSL2 both Linux distros share ONE network: two emulators on the same address and port collide.
  For development use another loopback address, e.g. `bind_ip:=127.0.0.3` (nothing to add, all of
  127.0.0.0/8 is local).
* In a Docker container the address must be added inside the container (`--cap-add NET_ADMIN`, then
  `add_camera_ip.sh`), or run the emulator with its fallback 127.0.0.1 and video_cam with
  `-p camera_ip:=127.0.0.1`. Containers already allow port 82 for normal users.

## Optical zoom (optional, more realistic for the ZR10)

By default every zoom is digital (crop), which is what an A8 mini does. A ZR10 has 10x real optics.
If the drone model attaches ardupilot_gazebo's `CameraZoomPlugin` to the photo camera, e.g.

```xml
<plugin filename="CameraZoomPlugin" name="CameraZoomPlugin">
  <topic>/sim/camera/photo/zoom</topic>
  <max_zoom>20</max_zoom>
</plugin>
```

start the emulator with `optical_zoom_topics:=/sim/camera/photo/zoom`: zoom up to 10x then changes the
Gazebo camera's field of view (sharp), only the rest is cropped. The plugin divides a fixed 2.0 rad (not
the camera's HFOV) by the command, and jumps to 2.0 rad at start if it gets no command; the emulator
compensates (`optical_zoom_ref_hfov_rad`) and re-sends the command every second.

## Tests

```bash
SHARE=$(ros2 pkg prefix siyi_emulator)/share/siyi_emulator
bash $SHARE/scripts/run_tests.sh                       # unit tests (no Gazebo; free ports on 127.0.0.1)
bash $SHARE/test/e2e/run_e2e_gazebo.sh                 # Gazebo test stand + 34 photo checks (127.0.0.3)
OPTICAL_ZOOM=1 bash $SHARE/test/e2e/run_e2e_gazebo.sh  # same, with CameraZoomPlugin (real optical zoom)
bash $SHARE/test/e2e/run_e2e_sim_models.sh             # against the real sim_models drone (skips if absent)
```

All Gazebo tests run headless on the development address 127.0.0.3 and their own GZ_PARTITION, so
they never disturb a running simulation; they stop only what they started.

* `test/test_*.py` (pytest): frames and CRC (video_cam's two packets bit-exact), every command's reply,
  zoom state machine, SD card and media JSON, gimbal IK and level-camera stabilisation, the full
  UDP+HTTP photo flow with a fake Gazebo (slow renders, lost triggers, no TF card, strict mode,
  garbage input), the ROS node, and `test_video_cam_compat.py`, which runs the team's **real**
  `CameraInterface` and `PipelineOrchestrator` against the emulator (skipped if video_cam is not built
  in the same workspace).
* `test/e2e/run_e2e_gazebo.sh` + `test/fixtures/gimbal_stand.sdf`: a floating "drone" with the contract's
  gimbal and cameras among coloured boxes. Checks "+yaw turns left", "-90 looks straight down", 0x0D,
  0x07 direction, 3x zoom, lock/follow modes and a level horizon while the drone is rolled 20 deg and
  pitched 10 deg; saves every photo plus `e2e_contact_sheet.jpg` to look at.
* `test/e2e/run_e2e_sim_models.sh`: the same idea with the real `iris_a8mini` parked in sim_models'
  test world (poles due north and 30 deg west of north).

## Troubleshooting

| symptom | cause and fix |
|---|---|
| `cannot start ... UDP 192.168.144.25:37260 is already used` | another emulator (maybe in the other WSL distro) runs. Stop it, or use `bind_ip:=127.0.0.3`. |
| `no permission to listen on TCP port 82` | run `sudo sysctl -w net.ipv4.ip_unprivileged_port_start=80` (or the setup script). |
| `camera address 192.168.144.25 does not exist on this computer` | run `scripts/add_camera_ip.sh`, or give video_cam `-p camera_ip:=127.0.0.1`. |
| video_cam: `Camera model detection failed - assuming ZR10` | the emulator is not reachable: check the line "SIYI camera emulator ready ..." and the address. |
| photos show "SIYI EMULATOR TEST PATTERN" | Gazebo gave no photo frame: is the drone spawned, same `GZ_PARTITION`, photo camera triggered on `/sim/camera/photo/trigger`? `gz topic -l | grep sim/camera`. If the log says "a Gazebo photo frame arrived ... late", the computer is too busy: raise `photo_timeout_s`. |
| photos take seconds, Gazebo is slow | headless Gazebo in WSL2 may render on the CPU (`gz` log: `libEGL warning: failed to open /dev/dri/renderD128: Permission denied ... falling back to kms_swrast`). Normal renders still take 0.05-0.3 s; under heavy load they took > 3 s. |
| is the emulator healthy? | `ros2 topic echo --full-length /sim/camera/emulator_state`: Gazebo link, odometry/joint ages, gimbal, zoom, and `photo_frames` (received / dropped / stale / timeouts). |
| warning `no drone odometry ... after 15 s` | the drone's OdometryPublisher must publish `/sim/drone/odometry` with `<dimensions>3</dimensions>` (else roll/pitch are not compensated). |
| warning `joint_states has no joint for ...` | set `yaw_joint/roll_joint/pitch_joint` to the joint names in the drone SDF. |
| photos are blurry / show a small area | the camera is zoomed (video_cam racks to 5x on `capture_setup`); see "Known hazards". |

## Known hazards worth knowing (team code behaviour the emulator makes visible)

* `video_cam` racks the zoom to `CAPTURE_ZOOM_X = 5.0` when the mission's DO_DIGICAM_CONTROL fires
  (`config.py:59`, `siyi_node.py:612-618`). With the default ZR10 profile every survey photo then covers
  1/5 of the width (19.4 deg instead of 81 deg: a 26 m wide strip at 75 m height instead of 128 m).
  Plan the lane spacing for that, enable optical zoom, or
  start the emulator with `max_zoom:=1.0` (video_cam then retries 3x and logs "Could not confirm 5.0x").
* After `sd_format yes` (0x48) the list is shorter than video_cam's `last_photo_count`, so it ignores
  new photos until the count passes the old one (`camera_interface.py:672`). Seen live: the photo after a
  format ended with "Timeout after 15.0s / Phase 2 failed" although the emulator had saved it in 0.1 s.
* video_cam names photos `"<lat> , <lon>.jpg"` (6 decimals): while the drone hovers, every new photo
  overwrites the previous one (seen live with a constant GPS fix: three captures, one file).
* `restart_numbering_after_format:=true` makes new photos reuse names video_cam has marked as seen.

## Files

| file | what it does |
|---|---|
| `siyi_emulator/sdk_protocol.py` | frames, CRC-16/XMODEM, payload encoders/decoders |
| `siyi_emulator/camera_state.py` | ZR10 / A8 mini profiles, zoom state machine, 0x01/0x02/0x0A bytes |
| `siyi_emulator/gimbal_control.py` | gimbal modes, slew limit, stabilisation, inverse kinematics |
| `siyi_emulator/command_handler.py` | what each SDK command does and answers |
| `siyi_emulator/sd_card.py` | the emulated SD card folder |
| `siyi_emulator/media_server.py` | the port-82 web server |
| `siyi_emulator/photo.py` | Gazebo frame -> zoomed JPEG, test pattern, zoom maths |
| `siyi_emulator/gz_link.py` | Gazebo connection (gz-transport Python) |
| `siyi_emulator/emulator.py` | everything together; stand-alone command line |
| `siyi_emulator/node.py` | ROS 2 node (parameters, topics) |
| `siyi_emulator/sdk_client.py` | tiny SDK client + `siyi_client` command line |
| `siyi_emulator/netutil.py` | which address to listen on, friendly bind errors |
| `launch/siyi_emulator.launch.py`, `config/siyi_emulator.yaml` | start it from a launch file |
| `scripts/add_camera_ip.sh`, `scripts/remove_camera_ip.sh`, `scripts/run_tests.sh` | helpers |
| `test/test_*.py`, `test/conftest.py` | pytest unit + loopback + ROS node + video_cam compatibility tests |
| `test/fixtures/gimbal_stand.sdf` | Gazebo test world (floating drone with the contract's gimbal, coloured boxes) |
| `test/e2e/run_e2e_gazebo.sh`, `e2e_gazebo_check.py`, `make_ground_texture.py` | end-to-end photo checks in Gazebo |
| `test/e2e/run_e2e_sim_models.sh`, `e2e_sim_models_check.py` | the same against the real sim_models drone |
