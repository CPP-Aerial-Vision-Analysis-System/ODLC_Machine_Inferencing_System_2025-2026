# ASTRA Gazebo simulator

A virtual copy of our drone (an Iris quadcopter with the SIYI camera on a gimbal) that flies in Gazebo with
the **real** Pixhawk firmware (ArduCopter SITL) and runs **our real flight code** (`./launcher.sh sim`):
MAVROS, video_cam, the detector, the DigiCamCtrl trigger and main_controller. You watch the camera's
point of view while it searches for the hiker (the mannequin) and the tent, finds them, drops the payloads
and flies home.

## Run it

```bash
# ONE TIME (Windows 11 + WSL, Ubuntu 22.04): install, then build. Commands run inside Ubuntu-22.04.
sudo bash simulation/setup/01_install_humble.sh   # ROS 2 Humble, Gazebo, MAVROS, Python stack (~30 min)
bash simulation/setup/02_setup_ardupilot.sh       # ArduCopter SITL 4.6.3, ArduPilot Gazebo plugin, MAVProxy
bash simulation/setup/03_build.sh                 # copies the repo to ~/ODLC_Machine_Inferencing_System_2025-2026 and builds it

# THE FULL SEARCH DEMO (one command; windows open, phases print here, score report at the end)
bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/demo.sh runway

# OR BY HAND: terminal 1 = simulator, terminal 2 = the team code, terminal 3 = the ground crew
bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/start_sim.sh runway      # --headless = no windows
cd ~/ODLC_Machine_Inferencing_System_2025-2026 && source simulation/setup/sim_env.sh && \
  TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py" ./launcher.sh sim
bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/fly_mission.sh runway    # upload, wait ~30 s, arm, AUTO
bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/take_photo.sh           # one photo by hand, any time
```

`TRACKER_CMD=...` runs the team's own `tracker.launch.xml` and only gives `new_od` the PC's YOLO weights (its
default is the Jetson's file, so without it the detector finds nothing). Wait until terminal 2 shows
`Model ready` before flying. `upload_mission.sh` / `fly_mission.sh` tell the running MAVROS to read the new
mission (MAVROS does not notice an upload by itself; the team's waypoint_manager would keep the old one).

No sudo password in WSL? Run the root steps from Windows PowerShell instead:
`wsl -d Ubuntu-22.04 -u root bash "/mnt/d/<path to the repo>/simulation/setup/01_install_humble.sh"`.
After editing code on Windows, run `03_build.sh` again: it copies your changes and rebuilds.

---

## What you will see

| Window | What is in it |
|---|---|
| **Gazebo** | The 3D world. The view **follows the drone** like a chase camera. Top right: **Camera POV (live)**, the drone's gimbal camera (1280x720, 25 frames per second). Bottom right: simulation time and the real-time factor (0.7-0.9 is normal). |
| **Demo panel** (sim_demo) | Big live camera view, the **newest photo with detection boxes**, the list of **detections** (class, confidence, latitude/longitude), the **mission phase**, and buttons (TAKE PHOTO, look down, upload/start mission, open/close Mission Planner). |
| **RViz** | The live camera and the newest photo, plus a 3D map: search area, the true target positions, detections, photo footprints, the drone's track. |
| **This terminal** (demo.sh) | One line per phase: ARMED, TAKEOFF, SEARCH, HOLD, TARGETS, DROP, RTL, LANDED, every detection, photo counts. At the end: the score report. |

A demo run on the runway world takes about 6-10 minutes. Everything from one run (logs, photos, phase log,
`score_report.md`) is in `~/.astra_sim/runs/<date>_<world>/`.

## Watch it in Mission Planner (Windows)

Mission Planner on Windows can watch the simulated drone just like the real one: map, HUD, flight mode,
mission. There is nothing extra to start. When the simulator runs inside WSL, MAVProxy also sends the drone's
messages to Windows, on UDP port 14550 (Mission Planner's usual port). The simulator log says where it sends
them: `[sim] Windows ground station: MAVLink also goes to udp:172.24.128.1:14550 ...`. demo.sh also prints
`Mission Planner / QGroundControl on Windows can watch now` once the autopilot is ready.

1. Start the simulator (`start_sim.sh runway`) or the demo (`demo.sh runway`) like always.
2. Open **Mission Planner** on Windows. Any of these three ways: double-click its icon; or press
   **OPEN MISSION PLANNER** in the demo panel (bottom middle, in Controls); or type
   `bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/mission_planner.sh start` in Ubuntu.
   Its window needs 10-30 s to appear.
3. Top right, next to the **CONNECT** button, change the connection box (it says `AUTO` or `COM...`) to **UDP**.
4. Click **CONNECT**. It asks for a port: keep **14550** and press OK.
5. Wait a few seconds. The first time, Mission Planner also downloads the autopilot's settings (a progress
   window, about 10-30 s).

What you will see:

| Where in Mission Planner | What |
|---|---|
| Map (DATA screen) | the drone on the field (the runway world is at lat 36.2143, lon -96.0068), its track and home |
| HUD (top left) | artificial horizon, altitude, speed, heading, battery, GPS, and the flight mode and ARMED / DISARMED |
| Flight mode during demo.sh | `STABILIZE` while it waits, `GUIDED` to arm, `AUTO` for take-off and the search lanes, `GUIDED` while main_controller holds and adds the targets, `AUTO` to fly to them, `GUIDED` for each drop, `RTL` home (it lands in RTL) |
| Messages tab (below the HUD) | the autopilot's text messages, for example `Mission: 3 DigiCamCtrl` |
| PLAN screen, **Read** button | the mission on the map, including the target waypoints main_controller adds |

Rules while demo.sh is flying:

* **Only watch.** Mission Planner is a real ground station and its buttons really work. Do not change the mode,
  arm or disarm, use "Fly to here", or press **Write** on the PLAN screen during demo.sh, unless you really
  want to take over: the team's code is flying, and a mode change interrupts it. **Read** is fine.
* Mission Planner tells the autopilot how often to send each kind of data (its telemetry rates). Over UDP it
  shares one line with the team's code, which then gets data at those rates too, so a scored run is not
  exactly comparable with a run without Mission Planner. **For a run whose score you want to compare,
  connect by TCP instead:** connection box **TCP**, CONNECT, host `127.0.0.1`, port **5762**. That is the
  autopilot's spare line, with its own rates (Windows reaches Ubuntu's TCP ports on 127.0.0.1 by itself).
* Closing Mission Planner during a flight is safe: the simulated autopilot has no ground-station failsafe
  (`FS_GCS_ENABLE 0` in `sim_models/config/astra_sitl.parm`).
* QGroundControl works the same way: it listens on UDP 14550 and connects by itself. Use one ground station at
  a time, because only one program can listen on a port.

**Open and close it from Ubuntu.** Ubuntu runs inside WSL, and WSL can start Windows programs ("interop"),
so the simulator side can open and close Mission Planner for you:

* **In the demo panel** (bottom middle, Controls card): **OPEN MISSION PLANNER** and **CLOSE MISSION PLANNER**.
  Next to the CONTROLS title a light says **Mission Planner: running** (green) or **not running** (grey); it is
  checked every 2 s. Under the buttons the panel reminds you of the next step: `UDP -> CONNECT -> port 14550`.
  Only the button that makes sense can be clicked (OPEN while it is closed, CLOSE while it is open).
* **In a terminal:** `bash simulation/scripts/mission_planner.sh start`, `... stop` or `... status`.
* **Open** starts `C:\Program Files (x86)\Mission Planner\MissionPlanner.exe`, never a second copy, and it stays
  open when the panel or the terminal closes. Installed somewhere else? Set `MISSION_PLANNER_EXE` (for example
  `export MISSION_PLANNER_EXE="/mnt/e/Mission Planner/MissionPlanner.exe"`; a Windows path works too).
* **Close** asks Mission Planner to close, like clicking its X (it can save its settings). If it is still open
  after 5 s, it is forced closed. Safe during a flight (see the rules above).
* How it works: `tasklist.exe` (Windows' task list) says whether `MissionPlanner.exe` runs, the panel starts
  `MissionPlanner.exe` itself, and `taskkill.exe` closes it. The code is
  `ros2_ws/src/simulation/sim_demo/sim_demo/mission_planner.py`; the panel and the script share it.
* Grey buttons mean this computer cannot do it: hold the mouse over them to read why (not inside WSL, for
  example on the Jetson; Mission Planner not installed; or WSL interop switched off).

**Switch it off:** `start_sim.sh runway --no-windows-gcs` or `demo.sh runway --no-windows-gcs` (the launch
argument is `windows_gcs:=false`). **Send it somewhere else**, for example to a teammate's laptop on the same
Wi-Fi: `start_sim.sh runway windows_gcs_addr:=192.168.1.20:14550` or `demo.sh runway --windows-gcs 192.168.1.20:14550`.

**How it works.** WSL normally uses "NAT" networking: Ubuntu is like a small separate computer inside Windows,
with its own address (172.something). Seen from Ubuntu, Windows is the "default gateway" (`ip route` shows it
after `default via`). At every start the launch file reads that address and gives MAVProxy one more output,
`--out udp:<Windows address>:14550`. The 127.0.0.1:14550 output that our own scripts use inside Ubuntu stays as
it is. Windows can give WSL a new address after a restart; because the address is looked up at every start,
nothing needs editing. Mission Planner's answers (settings downloads, commands) travel back the same way.

**Another way: mirrored networking.** Put this into `C:\Users\<you>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

Then run `wsl --shutdown` in PowerShell (this stops everything that runs in WSL) and open Ubuntu again.
Windows and Ubuntu now share their addresses, including `127.0.0.1`. The simulator notices this and sends the
Windows copy to `127.0.0.1:14551` instead, so **use port 14551 in Mission Planner**. (Port 14550 on the shared
`127.0.0.1` belongs to our scripts, and two programs cannot listen on the same port.) This has **not been
tested on the team laptop yet**, which uses NAT. If it does not work, delete the line and run `wsl --shutdown`
again to go back to NAT.

| Problem in Mission Planner | What to do |
|---|---|
| `No heartbeat packets received` | 1. Is the simulator running, and does its log show `[sim] Windows ground station: ...`? (If not: it was started with `--no-windows-gcs`, or not inside WSL.) 2. The Windows Firewall must let UDP 14550 in. Run this once in an **admin** PowerShell: `New-NetFirewallRule -DisplayName "ASTRA sim: MAVLink UDP 14550 from WSL" -Direction Inbound -Protocol UDP -LocalPort 14550 -Action Allow` (the team laptop already has this rule). 3. If Windows once asked "allow Mission Planner?" and you clicked Cancel, it made a **block** rule, and block rules beat allow rules: Windows Security, Firewall & network protection, Allow an app through firewall, tick Mission Planner for Private and Public. |
| It worked yesterday but not today | Windows gave WSL a new address after a restart. Restart the simulator, because the address is looked up at start. Compare `ip route` in Ubuntu (the address after `default via`) with the `[sim] Windows ground station` log line. |
| `Only one usage of each socket address` | Another program is already listening on UDP 14550 (QGroundControl, a second Mission Planner): close it. |
| The map is grey | Mission Planner has no internet for the map pictures. The drone, HUD and mode still work. |
| The panel's OPEN / CLOSE MISSION PLANNER buttons are grey | Hold the mouse over them to read why: not inside WSL, Mission Planner not in `C:\Program Files (x86)\Mission Planner\` (set `MISSION_PLANNER_EXE`), or WSL interop is off (`[interop] enabled=true` in `/etc/wsl.conf`). `bash simulation/scripts/mission_planner.sh status` says the same in a terminal. |
| Check without Mission Planner | Close Mission Planner, then in PowerShell run: `$u=New-Object Net.Sockets.UdpClient(14550);$e=New-Object Net.IPEndPoint([Net.IPAddress]::Any,0);$n=0;$t=Get-Date;while(((Get-Date)-$t).TotalSeconds -lt 10){if($u.Available){[void]$u.Receive([ref]$e);$n++}else{Start-Sleep -Milliseconds 5}};$u.Close();"$n packets from $e"`. A running simulator gives about 800 packets in 10 s. |

## How the pieces connect

```
                         Windows 11 (WSL2, Ubuntu-22.04)
 +-------------------------------- SIMULATOR (sim_bringup sim.launch.py) ---------------------------------+
 |                                                                                                         |
 |  Gazebo server (gz sim -s)  --- world: runway / suas_field (sim_worlds), drone "iris" (sim_models)      |
 |    | ArduPilotPlugin  <== UDP 9002 (JSON physics, lock-step) ==>  ArduCopter SITL (the Pixhawk firmware)|
 |    | gimbal joints, live camera, triggered photo camera, payloads                 | TCP 5760            |
 |    |                                                                          MAVProxy                 |
 |    | gz-transport topics /sim/...                                 udp 14550 /  udp 14552 |  \ udp 14553  |
 |    |                                                     ground station  /           |   \ demo panel |
 |    +-- siyi_emulator: the SIYI camera at 192.168.144.25 (UDP 37260 SDK + HTTP 82 media server)        |
 |    +-- payload_release: servo 11 / 13 open  ->  the payload falls                                     |
 |    +-- ros_gz_bridge: /sim/camera/image, /sim/camera/photo, /sim/drone/odometry -> ROS 2              |
 |    +-- Gazebo window, RViz, demo panel, demo markers, score_run (sim_demo)                            |
 +----------------------------------------------------------------|----------------|--------------------+
                                                                  | MAVLink        | SIYI SDK + HTTP
 +------------------------ TEAM FLIGHT STACK (./launcher.sh sim, unchanged) -------|--------------------+
 |  MAVROS (apm.launch, fcu_url udp://:14552@localhost:14552)                       |                    |
 |  mapping/trigger: "Mission: N DigiCamCtrl" -> /camera/trigger every second       |                    |
 |  video_cam (siyi): /camera/trigger -> take photo -> download -> mapping_photos/"<lat> , <lon>.jpg"    |
 |  detection/new_od: YOLO on every new photo -> /image_detection                                       |
 |  main_controller: last waypoint -> hold -> insert target waypoints -> fly there -> DO_SET_SERVO -> RTL |
 |  waypoint_manager, gpstest                                                                            |
 +-----------------------------------------------------------------------------------------------------+
   demo.sh = start the simulator -> wait for SITL -> upload mission -> start the team stack -> arm + AUTO
             -> phase_monitor prints the phases -> landed -> score report -> stop everything
```

The team's code cannot tell it is in a simulator: it talks MAVLink to "the Pixhawk" (SITL) and the SIYI
camera protocol to "the camera" (the emulator) at the real camera's address. MAVProxy also sends a fourth
copy of the MAVLink to Windows, for Mission Planner / QGroundControl there (see "Watch it in Mission Planner").

## What each program does

| Program | Package | Job |
|---|---|---|
| `gz sim -s` / `gz sim -g` | Gazebo Harmonic | physics + cameras / the window (separate: closing the window does not stop the simulation) |
| `ros_gz_sim create` | ros_gz | spawns the drone model, generated with xacro (camera size from the camera profile) |
| `arducopter --model JSON` | ArduPilot | the flight controller firmware, run on the PC; started only when the drone's plugin is listening |
| `mavproxy.py --daemon` | MAVProxy | copies MAVLink between SITL and three UDP ports inside Ubuntu (ground station, MAVROS, demo panel), plus one copy for a ground station on Windows |
| `siyi_emulator` | siyi_emulator | answers the SIYI SDK and media web API like the real camera; moves the gimbal; renders photos in Gazebo |
| `payload_release` | sim_models | detaches the person/tent payload when SITL opens servo 11 / 13 |
| `parameter_bridge` | ros_gz_bridge | Gazebo topics into ROS 2 (camera images, odometry, gimbal angles) |
| `odom_tf`, `gz_follow`, `wait_then_exec` | sim_bringup | drone TF for RViz, Gazebo chase camera, start order |
| `demo_panel`, `demo_markers`, `truth_detector`, `score_run` | sim_demo | demo window, RViz markers, optional answer-key detector, score report |
| `./launcher.sh sim` | team code | MAVROS + `tracker.launch.xml` (video_cam, new_od, trigger, waypoint_manager, gpstest, main_controller) |
| `mission_tool`, `phase_monitor`, `stack_check`, `photo_trigger` | sim_bringup | ground-station steps, phase printing, readiness check, photo button (used by the scripts) |

## Ports, addresses and topics

| What | Where | Used by |
|---|---|---|
| SITL MAVLink | TCP `5760 + 10*I` (I = SITL instance, default 0) | MAVProxy |
| SITL extra MAVLink | TCP `5762 + 10*I`, `5763 + 10*I` | a second ground station (Mission Planner / QGC can connect by TCP) |
| Gazebo <-> SITL physics | UDP `9002 + 10*I` | ArduPilotPlugin <-> SITL |
| Ground station (inside Ubuntu) | UDP `127.0.0.1:14550` (MAVProxy output) | our scripts (mission_tool), or a QGroundControl running in Ubuntu (one at a time) |
| Ground station on Windows | UDP `<Windows host>:14550` (MAVProxy output, only inside WSL; the address is the default gateway, found at every start) | Mission Planner / QGroundControl on Windows. Off: `windows_gcs:=false` / `--no-windows-gcs`; elsewhere: `windows_gcs_addr:=HOST:PORT` |
| MAVROS | UDP `127.0.0.1:14552` | `./launcher.sh sim` (`fcu_url udp://:14552@localhost:14552`) |
| Demo panel buttons | UDP `127.0.0.1:14553` | sim_demo demo_panel |
| SIYI camera | `192.168.144.25` UDP `37260` (SDK), TCP `82` (media) | the team's video_cam -> siyi_emulator |
| ROS 2 topics | `/sim/camera/image`, `/sim/camera/camera_info`, `/sim/camera/photo`, `/sim/drone/odometry`, `/sim/gimbal/joint_states`, `/sim/demo/markers` | panel, RViz |
| Team topics | `/camera/trigger` (the photo button), `/camera/command`, `/camera/status`, `/image_detection`, `/mavros/...` | the team stack |
| Gazebo topics | `/sim/...` (gimbal commands, cameras, odometry, payload servos) | inside the simulator |

`GZ_PARTITION=astra_sim` keeps the simulator's Gazebo traffic apart from any other Gazebo on the PC.
`ROS_DOMAIN_ID` is not changed (0, like the drone); the simulator and the team stack must use the same one.

## Worlds and missions

| World | What | Mission |
|---|---|---|
| `runway` | small and fast: pad, runway, a 120 x 100 m search box with one hiker (mannequin) and one tent | `sim_worlds/missions/runway_survey.waypoints`, 30 m altitude, 3 lanes (lane 1 starts 88 m early: camera set-up) |
| `suas_field` | the SUAS 2026 field (Skyway Range, Oklahoma) with its search area | `sim_worlds/missions/suas_field_survey.waypoints` |
| `standin` | ardupilot_gazebo's runway, no targets; only for testing when sim_worlds is not built | give one with `--mission FILE` |

Each world has `sim_worlds/config/<world>.yaml`: world file, SITL home (must equal the world's
`<spherical_coordinates>`), spawn pose, mission, ground-truth file. The missions have what the team code
needs: TAKEOFF, `DO_DIGICAM_CONTROL` (ArduPilot then sends "Mission: N DigiCamCtrl", which starts the team's
1 Hz photos), survey waypoints, and RETURN_TO_LAUNCH as the last item (main_controller inserts the target
waypoints right before it).

## Camera profiles and zoom

| `camera_model` / `--camera` | Photos | Why |
|---|---|---|
| `zr10` (default) | 2560 x 1440 ("2K") | last year's video_cam falls back to the SIYI ZR10 profile, and the detector is tuned for 2K |
| `a8mini` | 3840 x 2160 ("4K") | the A8 mini's own still size; slower to render and detect |

The live camera is always 1280 x 720 at 25 Hz, 81 degrees wide.
**Zoom:** when the mission's DigiCamCtrl starts, the team's video_cam racks the lens to **5x**
(`CAPTURE_ZOOM_X = 5.0`). With 5x zoom a photo only covers 1/5 of the width, and the survey lanes (planned
for the full 81-degree view) would miss most of the ground. `demo.sh` and `start_sim.sh` therefore start the
camera emulator with `max_zoom 1` (`--zoom survey`, the default; `start_sim.sh ... camera_max_zoom:=30` for the
real ZR10): video_cam logs "Could not confirm 5.0x after 3 attempts" and the photos keep the full view.
`--zoom team` keeps the 5x behaviour (fly lower lanes, or higher).
**Camera set-up time:** after "Mission: N DigiCamCtrl" video_cam spends about 15 s racking the zoom and focusing
and drops every photo request meanwhile (on a real A8 mini about 30 s). The runway mission therefore starts
lane 1 88 m (16 s) before the search box (`lead_in_s: 16` in `sim_worlds/config/runway.yaml`), so no part of
the box is flown without photos. Plan the team's real missions the same way.

## Who finds the targets: `--detector real` or `truth`

* `real` (default): the team's `new_od` with the YOLO weights `yolo26m.pt` (downloaded by `03_build.sh` into
  `~/.astra_sim/models/`; the repo only has a Jetson-only TensorRT file). This is the honest test of the model.
* `truth`: sim_demo's `truth_detector` publishes the same `/image_detection` messages, computed from the
  world's answer key and the camera pose of each photo. Use it to demonstrate the mission logic even when
  the model misses the simulated targets. The panel and the score report say clearly that it is on.
  (`new_od` then watches an empty folder and stays quiet.)

## The scripts (`simulation/scripts/`)

| Script | What it does |
|---|---|
| `demo.sh [world] [--detector real\|truth] [--camera zr10\|a8mini] [--headless] [--no-windows-gcs]` | the whole search demo (see the top) |
| `start_sim.sh [world] [name:=value ...] [--headless] [--background] [--keep-photos] [--no-windows-gcs]` | only the simulator (windows included unless `--headless`); moves old team photos to `~/.astra_sim/photo_archive/` |
| `stop_sim.sh [--all]` | stops exactly the simulator's processes (and the team stack demo.sh started) |
| `take_photo.sh [--direct]` | one photo: `/camera/trigger` (team way) or straight to the camera emulator |
| `upload_mission.sh [world\|file]` | uploads the survey mission (like Mission Planner) and makes a running MAVROS re-read it |
| `fly_mission.sh [world\|file] [--watch]` | upload (+ MAVROS re-read), wait for the settling window, arm, start the mission (AUTO) |
| `status.sh` | what is running, which ports are open, the autopilot's state, the ROS nodes |
| `mission_planner.sh start\|stop\|status` | opens or closes Mission Planner on Windows (the demo panel's OPEN / CLOSE MISSION PLANNER buttons, for the terminal), or says whether it runs (exit code 0 = running, 3 = not running) |

Every script explains itself with `--help`. Settings for a terminal: `source simulation/setup/sim_env.sh`.
Launch arguments of the simulator: `ros2 launch sim_bringup sim.launch.py --show-args` (see
`ros2_ws/src/simulation/sim_bringup/README.md`).

## Files and folders

| Path | What |
|---|---|
| `~/ODLC_Machine_Inferencing_System_2025-2026/` | the Linux-side copy that runs (made by `03_build.sh`; edit on Windows, re-run 03_build.sh) |
| `~/.astra_sim/runs/<date>_<world>/` | one demo run: `sim.log`, `team_stack.log`, `phases.jsonl`, `photos/`, `score_report.md` |
| `~/.astra_sim/sitl_I<I>/` | SITL's saved parameters and mission (`eeprom.bin`), MAVProxy log `mav.tlog` |
| `~/.astra_sim/sd_card_I<I>/` | the camera emulator's SD card |
| `~/.astra_sim/logs/` | `start_sim.sh` and `03_build.sh` logs |
| `<copy>/ros2_ws/src/video_cam/mapping_photos/` | the team's photo folder (what new_od reads). From Windows: `\\wsl.localhost\Ubuntu-22.04\home\<you>\ODLC_Machine_Inferencing_System_2025-2026\ros2_ws\src\video_cam\mapping_photos` |

## Docker (sim target)

`docker build -t astra-sim --target sim --platform linux/amd64 docker/` gives an image with everything
above (ROS 2 Humble, Gazebo Harmonic, SITL built from ArduPilot 4.6.3, ardupilot_gazebo, MAVProxy, xacro,
RViz, PyQt5, Gazebo Python bindings). Run the simulator and the team stack in that ONE container (ports and
the camera address stay inside it). With a display:

* **Windows 11**, from a shell of the Ubuntu-22.04 distro (Docker Desktop with WSL integration on), in the repo:
  ```bash
  docker run -it --rm --cap-add NET_ADMIN \
    -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR -e PULSE_SERVER \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
    --device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    -e GALLIUM_DRIVER=d3d12 -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
    -v "$PWD":/ODLC_Machine_Inferencing_System_2025-2026 -w /ODLC_Machine_Inferencing_System_2025-2026 astra-sim
  ```
* **Linux (X11):** `xhost +SI:localuser:$(id -un)`, then the same command with only
  `-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix` for the display (+ `--gpus all` with the NVIDIA Container
  Toolkit, or `--device /dev/dri`).

Inside: `bash simulation/setup/03_build.sh && bash simulation/scripts/demo.sh runway`. `--cap-add NET_ADMIN`
lets the scripts add 192.168.144.25 inside the container (the image's user has passwordless sudo).
Do not use `--network host` here: the camera address and port 82 then belong to the host.

## Troubleshooting

| You see | What it means / what to do |
|---|---|
| `PreArm: ...` / `Arm: Gyros inconsistent` for 30-60 s after start | **Normal.** SITL's sensors settle. The scripts wait and retry; by hand, wait ~30 s before arming. |
| MAVProxy: `Warning, time moved backwards. Restarting timer.` | Harmless: WSL corrected its clock. |
| MAVProxy: `Failed to download /SRTM3/filelist_python` | Harmless: MAVProxy's terrain download; the simulator does not need it. |
| `mavproxy.py`: `No module named 'future'` | Old setup: run `02_setup_ardupilot.sh` again (it now installs `future`), or `~/sitl/venv/bin/pip install future`. |
| `[sim] ERROR: SITL instance 0 ports are already in use` | Another simulator still runs: `bash stop_sim.sh` (or `--all`), or start with `sitl_instance:=1`. |
| demo.sh: `udp 14550 is in use` | A ground station running inside Ubuntu (or another script) listens there: close it (or `--gcs-port 14560`). Mission Planner on Windows does not cause this. |
| demo.sh: `udp 14552 is in use` | Another `./launcher.sh sim` (MAVROS) is running: stop it first. |
| `The camera address 192.168.144.25 is not on this PC` / video_cam: `Camera did not answer the 0x0A ...` | Run once `sudo bash simulation/setup/camera_ip_alias.sh` (or from PowerShell `wsl -d Ubuntu-22.04 -u root bash .../camera_ip_alias.sh`). It also comes back after restarts. |
| `sudo: a password is required` | The WSL user has no password. Use `wsl -d Ubuntu-22.04 -u root ...` from PowerShell; the scripts do this themselves for the camera address. |
| new_od: `Model file not found` / `Model failed to load` | The weights are missing: `bash 03_build.sh` (downloads `yolo26m.pt`), or use `--detector truth`. |
| team log: `Lens busy (zoom/focus) - dropping capture request` | Normal for the team code: the lens worker (zoom rack + autofocus every 5 s) blocks some 1 Hz triggers. |
| team log: `Could not confirm 5.0x after 3 attempts` | Expected with `--zoom survey` (the camera's zoom is limited to 1x on purpose). |
| photos cover only a small area | `--zoom team` (5x). Use the default `--zoom survey`. |
| Gazebo is slow, real-time factor < 0.5 | Close other programs; check the GPU: `glxinfo -B` must say `D3D12 (NVIDIA ...)`. `sim_env.sh` sets `GALLIUM_DRIVER=d3d12` and `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`. |
| `libEGL warning: failed to open /dev/dri/renderD128` / `falling back to kms_swrast` | Harmless: Gazebo tries another door first; its log still shows the NVIDIA GPU. |
| `gz_frame_id ... not defined in SDF` | Harmless warning from the ArduPilot Iris model files. |
| The Gazebo view does not follow the drone | It starts once the window and the drone exist. By hand: right-click the drone -> Follow. |
| A window does not open in WSL | `echo $DISPLAY` must show `:0`; restart WSL (`wsl --shutdown` in PowerShell). Windows need WSLg (Windows 11). |
| `Only N MB of memory is free` | The simulator wants ~3 GB (+ ~3 GB for the detector). Close programs, or give WSL more memory in `%UserProfile%\.wslconfig` (`[wsl2] memory=12GB`). |
| demo.sh: `the team stack did not get ready` | Read `team_stack.log` in the run folder: MAVROS not connected (another MAVROS on 14552?), waypoint_manager has no mission, or main_controller still waits for a service. |
| team log: `waypoint_manager: Takeoff Index: -1 ... RTL Index: -1` / `buffer_wp = -2` after an upload | The mission was uploaded after the team stack started and MAVROS did not re-read it. Use `upload_mission.sh` / `fly_mission.sh` (they make MAVROS re-read it), or `ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull {}`. Without it the photos never stop and main_controller never holds. |
| team log: main_controller `get_parameters timed out after 10s`, then `AttributeError: 'NoneType' object has no attribute 'values'` | A bug in the team code (main_controller.py `fetch_mission_indices` does not handle a parameter read that timed out); seen right after start and once at take-off. main_controller is dead (phase_monitor says so): no targets, no drops. demo.sh checks for 20 s that the team nodes stay alive and restarts the team stack once. By hand: wait ~20 s after `Model ready`, check `ros2 node list` shows `/main_controller`, else restart `./launcher.sh sim`. |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | A script with `set -u` sourced ROS: switch `set -u` off while sourcing (sim_env.sh does). |
| colcon: `Failed to find the following files: .../package.sh` | A package was built alone before the packages it needs: run `03_build.sh` (or `colcon build --packages-up-to <pkg>`). |
| `your 131072x1 screen size is bogus` | Harmless message of `free`/`ps` in WSL without a terminal. |
| Mission Planner / QGroundControl on Windows sees nothing | See "Watch it in Mission Planner (Windows)" above: check the `[sim] Windows ground station` log line and the firewall rule for UDP 14550. |

## For developers

* Unit tests: `cd ros2_ws/src/simulation/sim_bringup && python3 -m pytest -q test` (each simulation package has
  its own tests; `colcon test` runs them all).
* Start pieces one by one: `start_sim.sh runway sitl:=false` (world only), `emulator:=false`, `panel:=false`, ...
* Without any window (laptop on battery, a server, tests): `start_sim.sh runway --headless`, `demo.sh runway --headless`.
  A picture of the demo panel anyway: `QT_QPA_PLATFORM=offscreen ros2 run sim_demo demo_panel --ros-args
  -p world:=runway -p screenshot_path:=/tmp/panel.png -p screenshot_after_s:=60.0 -p quit_after_screenshot:=true`.
* Use your own drone file: `start_sim.sh runway model:=/path/my_drone.sdf.xacro`; your own world:
  `start_sim.sh /path/my_world.sdf` (its `<spherical_coordinates>` become SITL's home).
* Two simulators at once: `start_sim.sh runway sitl_instance:=1 gcs_port:=14560 mavros_port:=14562 panel_port:=14563`
  and `GZ_PARTITION=other`, then `FCU_URL=udp://:14562@localhost:14562 ./launcher.sh sim`.
* What changed in the team's files: `launcher.sh` reads `TRACKER_CMD` from the environment (default unchanged:
  `ros2 launch main tracker.launch.xml`). demo.sh sets it to `ros2 launch sim_bringup team_stack.launch.py`,
  which runs the same tracker.launch.xml and only gives new_od a model file that exists on the PC.
