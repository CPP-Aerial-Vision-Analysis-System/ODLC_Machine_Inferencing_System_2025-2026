<!-- PURPOSE: help operators diagnose build, startup, camera, detector, mission, GUI and ownership problems using the simulator's existing logs and status tools. -->

# Find the failing link

Start with the latest run's `sim.log` and `team_stack.log` under `~/.astra_sim/runs/`, then run
`bash simulation/scripts/status.sh --no-mavlink`. The status command inspects the current session,
resolved settings, processes, network endpoints and ROS nodes without opening the ground-tools UDP
port. Use a sourced Ubuntu terminal in the Linux checkout.

Every error message names its command in brackets (`XX [demo] ERROR: ...`) and says what failed, the
likely cause and the exact fix. [ERRORS](ERRORS.md) lists every message by command with its meaning and fix;
[COMMANDS](COMMANDS.md) lists every command (`bash simulation/scripts/astra.sh help`) with copy-paste examples.

For a controlled retry, stop your recorded session with `bash simulation/scripts/stop_sim.sh`, stop
any launcher you started by hand in its own terminal, and rerun the original command. Closing Gazebo's
window alone leaves its server running. Use one heavy simulator/build/Docker/generator job at a time;
do not rebuild the mirror underneath a running mission.

## Build and environment

| Symptom | Check and remedy |
|---|---|
| `Could not find a package configuration file provided by "ament_cmake"` (or `ament_cmake_python`, `rosidl_default_generators`) | ROS 2 Humble is not loaded in the terminal that runs colcon/cmake. Run `bash simulation/setup/check_ros.sh`: it says which case you are in. Usual fixes: `source /opt/ros/humble/setup.bash` (ROS installed but not loaded; add it to `~/.bashrc`); install ROS with `sudo bash simulation/setup/01_install_humble.sh` (not installed); or use the Docker dev container / an Ubuntu 22.04 WSL terminal (plain Windows PowerShell and macOS cannot build ROS 2 Humble). `03_build.sh` loads ROS itself and stops with a clear message when it is missing. |
| Changes on Windows have no effect | Run the Windows checkout's `simulation/setup/03_build.sh` again, then source the Linux environment. The mirror is a build/run copy. |
| Build path has spaces or lies under `/mnt/...` | Use `03_build.sh`; it mirrors a Windows checkout to the Linux home directory. Do not invoke colcon in the Windows folder. |
| ROS packages/`package.sh` are missing | Build all dependencies with `03_build.sh`, or use `--packages "package_name"` (packages-up-to). Re-source `sim_env.sh`. |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | Temporarily disable shell `set -u` around ROS setup; `sim_env.sh` already does this. |
| NumPy/cv_bridge `_ARRAY_API` error | Restore the project's pinned Python environment; its ROS/Jetson stack expects NumPy below 2. Avoid indiscriminate pip upgrades. |
| MAVProxy lacks `future` | Rerun `02_setup_ardupilot.sh` to repair the SITL virtual environment. |
| Different WSL distro has different packages | Use `Ubuntu-22.04` for Humble/Gazebo runs. Read ignored local notes for machine-specific tooling. |

`03_build.sh --no-models` skips downloads; it does not provide missing assets or weights. Missing-model
errors require a real supported weights file or a suitable test detector, not a silent path fallback.
Do not replace pinned MAVROS packages simply to remove a warning.

## Startup and ports

| Symptom | Check and remedy |
|---|---|
| `Arm: Gyros inconsistent` immediately after boot | Allow the normal 30–60-second settling period. The mission tools wait/retry; persistent failures need log inspection. |
| SITL ports already in use | Another session may still run. Inspect status and stop its recorded session. Use different instance/ports only for an intentionally separate setup. |
| UDP 14550 is occupied | A Linux GCS or another ground tool holds the script's port. Run one client at a time. Windows NAT output is a separate endpoint. |
| UDP 14552 is occupied | Another MAVROS/launcher is listening. Stop it from the terminal/session that started it. |
| `the team stack did not get ready` | Inspect `team_stack.log` for missing services, MAVROS connectivity, mission indices, node exits and detector model health. |
| A command worked from Ubuntu but fails through Windows | Put commands in a `.sh` file and run it through PowerShell's `wsl -d Ubuntu-22.04 -- bash ...`; avoid nested quoting and Git Bash path rewriting. |

SITL's saved `eeprom.bin` may override a newly edited default parameter file. A controlled test can use
`demo.sh ... --set autopilot.wipe=true`; this resets that simulated autopilot's saved state. Consult
[ARDUPILOT](ARDUPILOT.md) before changing parameter layering or firmware.

## Camera and photos

| Symptom | Check and remedy |
|---|---|
| Camera does not answer at `192.168.144.25` | Inspect address/HTTP status. Native simulation needs `sudo bash simulation/setup/camera_ip_alias.sh`; split Docker owns the address inside its network instead. |
| Emulator cannot bind HTTP port 82 | Rerun/read setup's low-port configuration. Check another process is not holding the port. |
| `Could not confirm 5.0x after 3 attempts` | Expected with default `--zoom survey`, which caps the emulator at 1x to match the lanes. |
| Photo footprint is much narrower than planned | `--zoom team` allows 5x. Use survey mode or plan spacing/height for the chosen zoom. |
| `Photo queue full` | The camera pipeline cannot keep up; requests are being dropped. Inspect shutter/index/download timings and lens timeouts. Treat this as a run-quality failure. |
| Old `Lens busy ... dropping capture request` messages | That is the old camera path. Rebuild current sources; the revised node queues requests. |
| Sidecar absent or invalid | The controller falls back to the photo GPS position; inspect write errors and the matching JSON stem. The detector must ignore `.json` and temporary image files. |
| Photos exist but some detections never arrive | Check exact file-name matching, atomic saves and whether the adapter publishes one result even for an empty photo. Old photos are archived before normal runs. |

The camera profile chooses expected still size; it does not issue a resolution-changing SIYI shutter
command. `zr10` is the simulator's 2K default; `a8mini` uses 4K. The real camera model, lens field of
view, digital zoom behavior and mounted photo orientation need airframe verification.

## Detection, missions and scores

| Symptom | Check and remedy |
|---|---|
| `Model file not found` / `Model failed to load` | Check `ASTRA_DETECTOR_MODEL` or profile `detector.model_path` points to supported weights. The Linux wrapper otherwise uses `~/.astra_sim/models/yolo26m.pt`. |
| Model loads but tent targets never appear | Check label mapping: existing team code maps `umbrella`/`kite` to tent; a literal `tent` label needs a compatible adapter/mapping. See [PLUGINS](PLUGINS.md). |
| Custom detections visible but controller ignores them | Verify reliable QoS, `ImageResult`, class IDs `"0"`/`"1"`, GPS, box center/size in full-photo pixels and exact image names. Use `check_slot --slot detector --node NODE --take-photo`. |
| Two sets of detections or competing mode changes | Ensure the custom slot replaces its team node and only one detector/brain owns the slot. |
| `buffer_wp = -2` / RTL index unknown after upload | Wait for the mission refresh log, confirm the mission contains expected landmarks, inspect pull errors. The convenience upload script can explicitly re-request MAVROS pull. |
| Parameter-read warning while controller remains alive | The asynchronous reader keeps old values and retries. Verify a later successful read; a warning alone does not prove current indices. |
| Old `NoneType ... values` controller crash | Rebuild the October parameter-reader fix. Check `/main_controller` stays present to the end. |
| Aim error differs from photo-GPS error | Expected: the detector reports shutter-position GPS, while the controller projects the target box. Compare the report's labelled metrics. |
| Drop error is labelled as release position | Payload landing pose was unavailable; that fallback cannot certify simulated impact accuracy. Inspect payload events/odometry. |
| A good runway score but a poor real-field score | Check terrain height, datum/home, FOV, orientation, timestamp/telemetry age, target rendering and altitude. Flat-ground calibration does not establish sloped-ground accuracy. |

`--detector truth` checks mission plumbing with answer-key detections, and the report labels it. It does
not measure learned-model performance. The normal truth path keeps `new_od` pointed at an empty feed;
that process can still load a model. For deliberate CPU-only inference, use a verified CPU adapter.

## GUI and graphics

| Symptom | Check and remedy |
|---|---|
| Gazebo is slow / real-time factor below 0.5 | Check renderer with `glxinfo -B`, close other heavy jobs and inspect memory. Under WSL the project configures Mesa D3D12 and the NVIDIA adapter preference. |
| The Gazebo window (3D world view) did not open, `Failed to create OpenGL context` or `D3D12: Removing Device` in `sim.log` | Seen on 2026-11-07 in 2 of 3 real-terrain flights: the window's process crashed while the server and RViz were also starting. Since then `sim.launch.py` opens it again after a crash (up to 4 tries, `[sim] the Gazebo window crashed ... opening it again`), and `demo.sh` waits up to 90 s for the window and warns loudly if it never appears. If all tries fail: `wsl --shutdown` from Windows PowerShell, update the NVIDIA driver, start again. Closing the window by hand is respected (it does not reopen). |
| The drone disarms right after a payload release and the run stalls in phase DROP; `sim.log` has `ODE INTERNAL ERROR 1: assertion "aabbBound >= dMinIntExact ..." failed in collide()` | The Gazebo physics server crashed at the payload release (seen once in four real-terrain flights on 2026-11-07; the other three, and 25 box drops from 49 m around the same spot, were fine, so it is not the terrain). The run is lost: `bash simulation/scripts/stop_sim.sh`, start again. If it repeats, save `sim.log` (the line `iris::iris_with_standoffs ... body:collision is a duplicate` just before it) for the maintainers. |
| WSL GUI does not open | Check WSLg/display availability and `DISPLAY`. Source `sim_env.sh`; read [DOCKER](DOCKER.md) for container display wiring. |
| Chase camera does not follow | Wait for the GUI/drone to exist, or use Gazebo's Follow action. |
| Mission Planner sees no telemetry | Check its selected port, launch log, Windows firewall and duplicate listeners. See [MISSION_PLANNER](MISSION_PLANNER.md). |
| OPEN/CLOSE MISSION PLANNER is disabled | Inspect the tooltip: WSL interop, installation location and executable availability. Set `MISSION_PLANNER_EXE` for a nonstandard installation. |
| The "Drone camera (live)" panel in the Gazebo window flashes to an outside view | The window shows old pictures; the camera data is fine (RViz and Mission Planner show it). Try `--launch-arg gui_camera:=window` (the camera gets its own small window). [Details](#the-camera-panel-in-the-gazebo-window-flashes) below. |
| Gimbal roll in photo sidecars jumps (e.g. -177 then +176) while pitch sits at -90 | Fixed 2026-10-31 in `siyi_emulator` (angle split in the gimbal's own yaw, roll, pitch order; that order is our assumption, see the [siyi_emulator README](../../ros2_ws/src/simulation/siyi_emulator/README.md)). Rebuild with `03_build.sh`; new sidecars read roll near 0. |

### The camera panel in the Gazebo window flashes

**What you see.** The "Drone camera (live)" panel in the top-right corner of the Gazebo window keeps switching
between the real camera picture (grass seen from above) and other pictures: the take-off pad seen from behind and
above, a darker copy of it, or a low view of the horizon and the runway. RViz and Mission Planner show the same
camera without trouble.

**What was measured (2026-10-31).** The wrong pictures were never on the camera topic. They are pictures the
Gazebo window drew earlier itself: frames of its own 3D view (the "outside" chase view of the pad), darker copies
of them, and the camera's first frames from before the gimbal turned down. It is not the gimbal and not the
camera data:

| Check | How | Result |
|---|---|---|
| Gimbal fights or shakes? | publishers of `/sim/gimbal/{yaw,roll,pitch}/cmd_pos`; joint angles 50 times a second in flight | one publisher (the SIYI emulator, 50 Hz); the joints only move to cancel the drone's tilt, never back and forth |
| Two cameras on one topic? | publishers of `/sim/camera/video/image`; size, time stamp and likeness of every frame | one publisher (Gazebo); all frames 1280 x 720, in order, no jumps |
| Panel listening to two topics? | the panel's settings (`topic_picker` false) and `gz topic -i` | one topic; its subscribers are the Gazebo window and the ROS bridge |
| What the window really draws | the window's own pixels read through X11 (the way `xwd` does) 12-15 times a second; a read counts as wrong when it looks like none of the camera frames of the 1.5 s before it (correlation below 0.6) | run `20261031-101431` (normal settings): 91 of 449 reads wrong during the search, 131 of 719 in the minute after it (hold, targets, drops). Run `20261031-095801`: 0 of 898 wrong during climb and search, then 352 of 599 after the drops and the landing. Run `20261031-092448`: 0 of 1736 wrong in 105 s of flight. The fault comes and goes. |
| Drawing on Qt's main thread (`gui_env:=QSG_RENDER_LOOP=basic`) | run `20261031-095801` was started with it | does not stop it (352 of 599 reads wrong late in that run) |
| The panel next to a camera-only window, read in the SAME seconds | run `20261031-111300` (normal settings) plus a second Gazebo program that holds only the camera panel (the layout `gui_camera:=window` uses). The two pictures were read one right after the other, 7 times a second each, for 7 minutes (take-off until 2 minutes after landing) | panel in the main window: 674 of 2925 reads looked like none of the camera-only window's reads of the 1.5 s around them (sky and horizon during take-off and search, a low view of the runway on the way home, the pad from the side after landing). Camera-only window: never sky; after landing 0 of 905 reads unlike the camera topic, while the panel had 594 of 906 in the same seconds |

**Why it happens.** Measured: the wrong pictures come from inside the main Gazebo program, the one that has the
3D view and the panel together. A second Gazebo program on the same screen, with only the camera in it, showed the
right picture in the same seconds. Not measured, only our best guess: the panel and the 3D view share this
laptop's graphics driver (Mesa's D3D12 driver inside WSL2), and the driver sometimes hands the panel a piece of
picture memory that still holds an old 3D-view picture. Nobody has looked inside the driver.
(An earlier version of this page said a camera-only window "at the same time" stayed clean, before this was
measured. That claim was wrong: that window was read at a different minute of the flight, and its capture
crashed. The same-seconds row above replaces it.)

**What to do.** Show the camera in its own small Gazebo window, a separate program with no 3D view in it:

```bash
bash simulation/scripts/demo.sh runway --launch-arg gui_camera:=window
```

The main Gazebo window keeps its usual layout (for runway and suas_field_real: the world's own layout) without the
camera panel, and a second small window, "Drone camera (live)", shows the camera; drag it where you like.
`gui_camera:=off` leaves the camera out of Gazebo (RViz and Mission Planner still show it). `gui_camera:=panel`
(the default) is the old single window. What has been checked: the camera-only window stayed right for a whole
flight (the same-seconds row above), and `gui_camera:=window` opens the two windows as described (main window
1200 x 1000 with the world's layout and no panel, camera window 700 x 460). A whole flight started WITH
`gui_camera:=window` has not been watched yet, and the fault comes and goes, so one clean flight proves little.

**Check it yourself.** During the search the camera looks straight down, so its picture never contains sky. Watch
the camera picture for one minute of the search: a picture with sky (or the pad seen from the side) is the fault.

**The gimbal angles on the camera picture.** `sim_demo pov_server` (the camera feed for Mission Planner,
`http://127.0.0.1:8081/`) writes two lines in the top-left corner of every frame. Line 1 is where the camera
points: pitch and roll against the horizon (-90 = straight down), yaw against the drone's nose (+ = left). These
are the numbers the SIYI emulator sends the team's code while it keeps the camera level (its normal setting).
Line 2 is the three gimbal motors measured in Gazebo, relative to the drone; while the drone tilts they move so
that line 1 stays steady. `--ros-args -p overlay:=false` turns the box off.

`libEGL` device-probe warnings, Gazebo `gz_frame_id` warnings, MAVProxy `time moved backwards` and SRTM
download messages have appeared in working runs. Judge them alongside renderer/telemetry/score evidence;
do not dismiss a missing camera frame or failed startup merely because another warning is familiar.

## Keep useful evidence

Record the command/profile, world, model, firmware identity, camera/zoom settings, first error and run
folder. Confirm both target outcomes, processed-photo counts, final landing and controller survival.
Keep logs/photos/raw GIS data local; commit a short sanitized report or test that reproduces the bug.
Follow [the handoff](../HANDOFF.md) for the ordered acceptance checks and unresolved decisions.
