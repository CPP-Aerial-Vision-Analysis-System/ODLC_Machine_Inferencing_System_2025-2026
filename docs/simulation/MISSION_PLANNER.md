<!-- PURPOSE: explain how to observe or manually fly the digital twin with Mission Planner, select the right MAVLink connection, and reset/stop a simulator session. -->

# Fly and inspect with Mission Planner

Mission Planner can connect to SITL as a ground station. In an automatic demo it is an observer;
in `--manual` mode you upload the mission, arm and start AUTO. The team's controller then handles
detections, target waypoints, drops and RTL. START OVER/manual acceptance flights remain tracked in
[the handoff](../HANDOFF.md); these instructions describe the implemented flow.

## Observe an automatic demo

Start `bash simulation/scripts/demo.sh runway --keep` from the Linux checkout. On Windows, open Mission
Planner normally, use the panel's OPEN MISSION PLANNER button, or run
`bash simulation/scripts/mission_planner.sh start` through WSL.

| Connection | Mission Planner settings | When to use it |
|---|---|---|
| Spare SITL TCP link | TCP, host `127.0.0.1`, port `5762` | Comparable scored runs on the same WSL host |
| WSL NAT output | UDP, CONNECT, port `14550` | Usual Windows connection; launch log prints host address |
| WSL mirrored output | UDP, CONNECT, port `14551` | Only if WSL uses mirrored networking; not yet verified here |

Use the actual port from the launch log if the SITL instance or network profile changes. Spare TCP
ports add `10*I` for instance `I`. Inside Linux, UDP 14550 belongs to the demo's ground tools: two
programs cannot listen on it concurrently. A remote machine uses the WSL host's reachable address,
not its own `127.0.0.1`; forwarding/firewall configuration then depends on that network.

Mission Planner may request different telemetry rates. On the forwarded UDP stream this can change
rates seen by other consumers; use the spare TCP link when comparing runs. The DATA screen shows HUD,
position, mode and messages. PLAN → Read downloads the mission, including newly inserted target points.
Writing a mission, changing mode or arming/disarming takes control of the simulated vehicle, so reserve
those actions for the manual flow or deliberate intervention.

## The ASTRA tab and the drone camera (the ASTRA plugin)

Mission Planner is the ground station. A small **plugin** makes it show everything the old ASTRA panel showed. A plugin
is one file that Mission Planner reads when it starts:
[`simulation/windows/mission_planner_plugin/astra_gcs.cs`](../../simulation/windows/mission_planner_plugin/astra_gcs.cs).

**Install it** (once, and again after it changes):

* the easy way: open Mission Planner with `bash simulation/scripts/mission_planner.sh start` (or the simulator scripts
  that open it). That copies the plugin into Mission Planner's `plugins` folder first;
* by hand, in Windows PowerShell: `powershell -ExecutionPolicy Bypass -File simulation\windows\install_mp_plugin.ps1`,
  then close and reopen Mission Planner. Uninstall: the same with `-Remove`.

**What you see** on the **DATA** screen:

1. **The drone camera in the HUD.** As soon as the simulator runs, the blue-and-brown horizon background becomes the
   live gimbal camera picture. Nothing to click. When the simulator closes, the normal horizon comes back. Right-click
   the HUD, then **ASTRA camera**, to switch it off or change the address.
2. **The ASTRA tab** (bottom left, chosen automatically):
   * the **phase strip**: Waiting, Take-off, Search, Process, Targets, Drop, Return, Landed (the current one is blue,
     finished ones are ticked);
   * one **sentence** saying what the drone does right now ("Searching: waypoint 7, 30 m, 6.0 m/s ...");
   * big **readouts**: flight mode, height, speed, battery, GPS, distance home, time in the air, waypoint;
   * a **HIKER** card and a **TENT** card: how often it was detected, the target position the flight code chose, and
     when the payload was released (green = done);
   * the **event log**: the autopilot's and the team code's messages, newest first, warnings in amber;
   * buttons: **ARM + START MISSION** and **RETURN HOME (RTL)** (both ask "are you sure?" first). In the simulator also
     **TAKE PHOTO**, **CAMERA DOWN**, **CAMERA FORWARD** and **START OVER** (they run `simulation/scripts/mp_actions.sh`
     through WSL, the same ROS buttons the old panel pressed).
3. **Map markers** for the chosen targets (red = hiker, orange = tent) and green dots where the payloads were released.

Where the information comes from: only what Mission Planner already receives over MAVLink (the autopilot, and the team
code's status texts such as "Detected tent", "Aim tent 36.2149350,-96.0069430" and "Servo 13 -> 1400"). So it works
the same with the real drone over the telemetry radio. Only the camera picture and the simulator buttons use the
simulator.

Check it from outside: the plugin writes `Documents\Mission Planner\astra_plugin_status.json` every 2 s (phase, camera
pictures shown, detections, last error). No ASTRA tab at all? Run
`powershell -ExecutionPolicy Bypass -File simulation\windows\check_mp_plugin.ps1`: it compiles the plugin against your
Mission Planner and prints the error with a line number.

## The old ASTRA demo panel is off by default

`demo.sh` no longer opens the Qt "ASTRA Simulation" window: the ASTRA tab above does its job. Bring the old window back
with `bash simulation/scripts/demo.sh runway --panel` (it still works and is unchanged).

## The camera stream behind the picture

The simulator starts a small program, `pov_server`, next to the other windows. It turns the gimbal camera's live
picture (the ROS topic `/sim/camera/image`) into an **MJPEG web stream** at `http://127.0.0.1:8081/pov.mjpg`, which the
ASTRA plugin shows in the HUD. Open <http://127.0.0.1:8081/> in any browser on Windows to see it on its own (a grey
"waiting for the camera" picture means Gazebo has not produced a frame yet). Without the plugin, Mission Planner can
still show it by hand: right-click the HUD, **Video**, **Set MJPEG Source**, paste the address, OK.

Good to know:

* It only **reads** the camera topic, so it cannot change the flight. If it crashes, nothing else notices.
* It listens on `127.0.0.1` (this computer) only, and reaches Mission Planner on Windows through WSL's localhost
  forwarding. Other computers cannot see it unless you set `-p host:=0.0.0.0` yourself.
* Pictures are shrunk to 960 pixels wide and limited to 10 per second (about 100-300 KB/s).
* Turn it off with `pov_server:=false`, or move it with `pov_port:=8082` (a launch argument of `sim.launch.py`).
  Settings of the program itself (`ros2 run sim_demo pov_server --ros-args -p max_fps:=5.0`): `topic`, `host`,
  `port`, `max_fps`, `quality`, `max_width`.
* Quick checks: `curl http://127.0.0.1:8081/health` prints how many pictures it has and how old the newest is.

## Manual flight

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026
source simulation/setup/sim_env.sh
bash simulation/scripts/demo.sh runway --manual
```

On a native Linux checkout use its root instead of the mirror path. The script starts the simulator,
camera, score recorder and team stack, then prints the steps and mission file. It does not upload or
arm. It keeps the session open and defaults to a three-hour operator timeout.

1. Connect Mission Planner and wait for the vehicle to finish its startup/parameter download.
2. Open PLAN, load the printed `.waypoints` file, and write it to the vehicle. Confirm the loaded route.
3. Wait for GPS/EKF/pre-arm checks. Initial `Arm: Gyros inconsistent` can take 30–60 seconds to clear.
4. Arm the vehicle, then select AUTO/start the mission as prompted by Mission Planner.
5. Watch the full search, processing hold, target visits, drops and RTL/landing; read the new run's score.

The script prints the mission's Windows path when it can derive it. Otherwise Windows Explorer can
browse `\\wsl.localhost\Ubuntu-22.04\home\<linux-user>\...`, or you can copy the generated mission into
an operator-accessible folder. Keep HOME and mission coordinates from the selected world.

`waypoint_manager` refreshes the mission after upload text and on arming. To exercise that behavior,
upload directly through Mission Planner or `mission_tool`; the convenience `upload_mission.sh` and
`fly_mission.sh` additionally request MAVROS re-read. Inspect `team_stack.log` for the correct indices
and `Re-read the mission...`, rather than assuming an upload was observed.

For a separate MAVLink client that bypasses the shell re-read helper, after the manual demo starts:

```bash
MISSION="$(python3 -m sim_bringup.sim_paths world runway mission)"
ros2 run sim_bringup mission_tool --url udpin:127.0.0.1:14550 upload --mission "$MISSION"
ros2 run sim_bringup mission_tool --url udpin:127.0.0.1:14550 fly
```

Do not run another Linux client on that port at the same time. These commands affect the running
simulator; use its configured GCS port if it differs from 14550.

## Start over

Uploading the survey again does not reattach payloads or reset the mission controller. Use:

```bash
bash simulation/scripts/start_over.sh --fly --follow
```

The reset lands an airborne vehicle first (LAND by default, `--land rtl` for RTL), waits for disarm,
archives the old photos, resets the drone/SITL/team stack, and creates a fresh run folder. It keeps the
same world/camera/detector settings and windows. `--land-timeout SECONDS` bounds the landing wait.
Without `--fly`, reset stops at readiness for you to upload/start through Mission Planner or the panel.
The panel's START OVER action requires a second click within its confirmation window and invokes the
same reset script. Changing worlds requires stopping the session and starting another demo.

When no simulator is running, `start_over.sh` starts a fresh kept-open demo (`--fly`) or manual demo
(without it); `--no-fresh` instead fails if there is nothing to reset. To stop your session, run
`bash simulation/scripts/stop_sim.sh`. A launcher started by hand belongs to its own terminal: stop it
there too. Reset logs are under `~/.astra_sim/logs/start_over_<time>.log`.

## Connection problems

Read the launch's `[sim] Windows ground station` line: NAT host addresses change after WSL restarts.
`--no-windows-gcs` disables that output. Override it with
`demo.sh runway --windows-gcs HOST:PORT` or `start_sim.sh runway windows_gcs_addr:=HOST:PORT`.
Allow Mission Planner's UDP traffic through Windows Firewall if it is blocked; keep the rule scoped
to the appropriate trusted network. Verify another Mission Planner/QGroundControl instance is not
holding the same port. A grey map can be an internet/basemap issue while telemetry still works.

The panel uses WSL interop for OPEN/CLOSE MISSION PLANNER; on native Linux those buttons are unavailable.
Set `MISSION_PLANNER_EXE` to a nonstandard installation's executable. `mission_planner.sh status`
returns 0 while running and 3 when not running. See [TROUBLESHOOTING](TROUBLESHOOTING.md) for simulator
readiness, camera and process checks.
