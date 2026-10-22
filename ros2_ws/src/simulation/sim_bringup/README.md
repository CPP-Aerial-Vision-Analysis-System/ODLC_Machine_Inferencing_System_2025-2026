# sim_bringup: the "start button" of the ASTRA simulator

`sim_bringup` starts the whole Gazebo simulator with one command and holds the small helper tools the
scripts in `simulation/scripts/` use (mission upload and arming, the phase monitor, readiness checks, the
photo button). It does not contain the drone, the worlds or the camera: those are the packages
`sim_models`, `sim_worlds`, `siyi_emulator` and `sim_demo`. The full guide (setup, the demo, troubleshooting)
is [`simulation/README.md`](../../../../simulation/README.md).

```bash
ros2 launch sim_bringup sim.launch.py                       # runway world, Gazebo window + RViz + demo panel
ros2 launch sim_bringup sim.launch.py world:=suas_field camera_model:=a8mini
ros2 launch sim_bringup sim.launch.py gui:=false rviz:=false panel:=false   # headless (no windows)
ros2 launch sim_bringup sim.launch.py --show-args           # every argument with its default
```

## What `sim.launch.py` starts (in this order)

| # | Process | Why | Switch |
|---|---|---|---|
| 1 | `gz sim -s -r <world>` (Gazebo server) | runs the physics and renders the cameras | always |
| 1 | `gz sim -g` (Gazebo window) with `config/gazebo_gui.config` | big 3D view + live camera POV panel | `gui` |
| 2 | `ros_gz_sim create` | spawns the drone (entity `iris`) from the xacro-generated SDF | always |
| 3 | `arducopter --model JSON -I<I> --home <world home>` | the Pixhawk firmware (ArduCopter SITL), started by `wait_then_exec` only when the drone's plugin opened UDP 9002+10I | `sitl` |
| 4 | `mavproxy.py --daemon` | routes MAVLink to 14550 / 14552 / 14553 (see ports), plus a copy for Mission Planner / QGC on Windows | `mavproxy`, `windows_gcs` |
| 5 | `siyi_emulator` | the SIYI camera on 192.168.144.25 (UDP 37260 + HTTP 82) | `emulator` |
| 5 | `payload_release` (sim_models) | drops the payloads when servo 11 / 13 opens | `payload` |
| 6 | `parameter_bridge` + `odom_tf` | camera images, odometry, gimbal angles into ROS 2; drone TF | `bridge` |
| 7 | `demo_markers`, `demo_panel`, `truth_detector` (sim_demo) | RViz markers, the demo window, the optional truth detector | `markers`, `panel`, `detector:=truth` |
| 8 | `rviz2 -d rviz/sim.rviz` | big camera POV + 3D view with the markers | `rviz` |
| 8 | `gz_follow` | makes the Gazebo 3D view follow the drone (`/gui/follow`) | `follow` (needs `gui`) |

A missing simulation package never stops the launch: it prints `[sim] package X is not built ...` and uses a
stand-in (ardupilot_gazebo's `iris_runway` world / `iris_with_gimbal` drone) or skips that part.
The team's own flight stack is **not** started here: run `./launcher.sh sim`, or `simulation/scripts/demo.sh`.

## Launch arguments (`sim.launch.py`)

| Argument | Default | Meaning |
|---|---|---|
| `world` | `runway` | `runway`, `suas_field` (any `sim_worlds/config/<name>.yaml`), a path to an `.sdf`, or a world config `.yaml` |
| `camera_model` | `zr10` | `zr10` = 2K photos 2560x1440 (last year's default) / `a8mini` = 4K photos 3840x2160 |
| `gui` / `rviz` / `panel` | `true` | the three windows |
| `headless_rendering` | `false` | render cameras with EGL and no display (servers, CI) |
| `detector` | `real` | `real` = the team's `new_od` / `truth` = start sim_demo `truth_detector` |
| `markers` | `true` | start sim_demo `demo_markers` |
| `sitl` / `sitl_instance` / `sitl_wipe` | `true` / `0` / `false` | SITL on ports 5760+10I and 9002+10I; `sitl_wipe` erases saved parameters + mission |
| `sitl_bin` / `mavproxy_bin` | `""` | override the binaries (default search below) |
| `mavproxy` | `true` | start MAVProxy |
| `gcs_port` / `mavros_port` / `panel_port` | `14550` / `14552` / `14553` | MAVProxy UDP outputs |
| `windows_gcs` | `true` | inside WSL, also send MAVLink to Mission Planner / QGroundControl on Windows (address found at start: WSL NAT = the default gateway, UDP 14550; mirrored = `127.0.0.1:14551`); nothing outside WSL; `false` = off |
| `windows_gcs_addr` | `""` | send that copy to `HOST` or `HOST:PORT` instead (also works outside WSL, e.g. a laptop on the network) |
| `emulator` / `camera_ip` | `true` / `192.168.144.25` | SIYI camera emulator and the address it pretends to be |
| `payload` / `bridge` | `true` | payload release node / ROS bridge |
| `follow` / `follow_offset` | `true` / `-12,0,6` | Gazebo chase camera and its offset (metres behind/left/above) |
| `gui_config` | `""` | Gazebo window layout: `""` = ours, `default` = Gazebo's, or a file |
| `gz_verbosity` | `1` | Gazebo console verbosity 0-4 |
| `model` | `""` | another drone file (`.sdf` or `.sdf.xacro`) |
| `x` `y` `z` `yaw` | `""` | spawn pose override (metres, radians) |
| `photo_dir` | `""` | the team photo folder the panel watches (default `<team ros2_ws>/src/video_cam/mapping_photos`) |
| `run_dir` | `""` | state folder (default `$ASTRA_SIM_HOME` or `~/.astra_sim`) |

Search order: SITL `$ARDUPILOT_SITL_BIN`, `~/sitl/arducopter`, `/opt/ardupilot/build/sitl/bin/arducopter`;
parameter files `$ARDUPILOT_DEFAULT_PARAMS`, `~/sitl`, `/opt/ardupilot/Tools/autotest/default_params`;
plugin `$ARDUPILOT_GAZEBO_DIR`, `~/ardupilot_gazebo`, `/opt/ardupilot_gazebo`; MAVProxy `$ASTRA_MAVPROXY`,
`~/sitl/venv/bin/mavproxy.py`, `mavproxy.py` on the PATH.

## `team_stack.launch.py`

Runs the team's `main/launch/tracker.launch.xml` **unchanged**, adding only global parameters that the team's
`new_od` declares: `model_path` (the detector weights, default `~/.astra_sim/models/yolo26m.pt`, because the
team default is a Jetson-only path) and, with `detector:=truth`, `camera_feed_path` set to an always-empty
folder so `new_od` stays quiet while sim_demo's `truth_detector` publishes `/image_detection`.
`./launcher.sh sim` uses it when `TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py detector:=real"`
(demo.sh sets this).

## Ports, topics and files

| Kind | Name | Direction | Notes |
|---|---|---|---|
| TCP | 5760+10I | SITL serial0 | MAVProxy connects here (SITL waits for it) |
| UDP | 9002+10I | Gazebo plugin <-> SITL | the JSON physics link, lock-step |
| UDP | 127.0.0.1:14550 | MAVProxy out | ground station inside Ubuntu: our scripts (mission_tool) **or** a QGC running in Ubuntu (one at a time) |
| UDP | 127.0.0.1:14552 | MAVProxy out | MAVROS: `fcu_url udp://:14552@localhost:14552` (`./launcher.sh sim`) |
| UDP | 127.0.0.1:14553 | MAVProxy out | sim_demo demo panel (mission buttons) |
| UDP | `<Windows host>`:14550 | MAVProxy out | Mission Planner / QGC on Windows (`windows_gcs`, only inside WSL; `sim_paths.windows_gcs_output`) |
| UDP/TCP | 192.168.144.25:37260 / :82 | siyi_emulator | SIYI SDK / media web server, as the real camera |
| ROS topic | `/sim/camera/image`, `/sim/camera/camera_info` | out (bridge) | live POV 1280x720 @ 25 Hz |
| ROS topic | `/sim/camera/photo` | out (bridge) | every photo the camera emulator renders |
| ROS topic | `/sim/drone/odometry`, `/sim/gimbal/joint_states` | out (bridge) | drone pose (world ENU), gimbal angles |
| ROS topic | `/tf`, `/tf_static` | out (odom_tf) | `map` -> `iris/odom` (identity) -> drone frame |
| file | `~/.astra_sim/sitl_I<I>/` | SITL + MAVProxy | `eeprom.bin` (saved parameters and mission), `mav.tlog`, logs |
| file | `~/.astra_sim/generated/` | launch | generated drone SDF, stand-in world, bridge list |
| file | `~/.astra_sim/state/current_launch.yaml` | launch | what the running simulator resolved (world, home, mission, truth, ports) |

## Helper commands (`ros2 run sim_bringup <name>`)

| Command | What it does | Main options |
|---|---|---|
| `wait_then_exec` | waits for a UDP/TCP port or a file, then becomes the given program | `--udp-port`, `--tcp-port`, `--file`, `--timeout`, `-- PROGRAM ...` |
| `gz_follow` | makes the Gazebo 3D view follow the drone | `--entity iris`, `--offset -12 0 6` |
| `odom_tf` | `/sim/drone/odometry` -> TF for RViz | params `odom_topic`, `map_frame` |
| `mission_tool` | MAVLink ground station: `upload`, `wait-ready`, `arm`, `start`, `fly`, `watch`, `status`, `mode`, `rtl`, `land`, `disarm`, `check-mission` | `--url udpin:127.0.0.1:14550` |
| `phase_monitor` | prints the mission phases (ARMED, TAKEOFF, SEARCH, HOLD, TARGETS, DROP, RTL, LANDED), detections, photo count; exits 0 when landed | `--log`, `--timeout` |
| `stack_check` | waits until MAVROS + the team nodes are up and waypoint_manager has the mission | `--timeout`, `--need-detector` |
| `photo_trigger` | one photo: `/camera/trigger` (team way) or straight to the emulator (`--direct`) | `--camera-ip`, `--timeout` |

`python3 -m sim_bringup.sim_paths world runway mission` prints the runway mission path (also `truth`,
`world_file`, `home`), and `... photo-dir`, `... model-path`, `... sitl-ports 0` answer the other questions
the shell scripts have.

## Tests

```bash
cd ros2_ws/src/simulation/sim_bringup && python3 -m pytest -q test     # unit tests, no simulator needed
bash test/run_tests.sh                                                  # + launch argument check + shellcheck
```
`test/gen_rviz_window_state.py` regenerates the RViz dock layout of `rviz/sim.rviz`.
