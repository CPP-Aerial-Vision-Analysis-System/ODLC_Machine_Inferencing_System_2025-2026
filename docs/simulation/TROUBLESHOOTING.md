<!-- PURPOSE: help operators diagnose build, startup, camera, detector, mission, GUI and ownership problems using the simulator's existing logs and status tools. -->

# Find the failing link

Start with the latest run's `sim.log` and `team_stack.log` under `~/.astra_sim/runs/`, then run
`bash simulation/scripts/status.sh --no-mavlink`. The status command inspects the current session,
resolved settings, processes, network endpoints and ROS nodes without opening the ground-tools UDP
port. Use a sourced Ubuntu terminal in the Linux checkout.

For a controlled retry, stop your recorded session with `bash simulation/scripts/stop_sim.sh`, stop
any launcher you started by hand in its own terminal, and rerun the original command. Closing Gazebo's
window alone leaves its server running. Use one heavy simulator/build/Docker/generator job at a time;
do not rebuild the mirror underneath a running mission.

## Build and environment

| Symptom | Check and remedy |
|---|---|
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
| WSL GUI does not open | Check WSLg/display availability and `DISPLAY`. Source `sim_env.sh`; read [DOCKER](DOCKER.md) for container display wiring. |
| Chase camera does not follow | Wait for the GUI/drone to exist, or use Gazebo's Follow action. |
| Mission Planner sees no telemetry | Check its selected port, launch log, Windows firewall and duplicate listeners. See [MISSION_PLANNER](MISSION_PLANNER.md). |
| OPEN/CLOSE MISSION PLANNER is disabled | Inspect the tooltip: WSL interop, installation location and executable availability. Set `MISSION_PLANNER_EXE` for a nonstandard installation. |

`libEGL` device-probe warnings, Gazebo `gz_frame_id` warnings, MAVProxy `time moved backwards` and SRTM
download messages have appeared in working runs. Judge them alongside renderer/telemetry/score evidence;
do not dismiss a missing camera frame or failed startup merely because another warning is familiar.

## Keep useful evidence

Record the command/profile, world, model, firmware identity, camera/zoom settings, first error and run
folder. Confirm both target outcomes, processed-photo counts, final landing and controller survival.
Keep logs/photos/raw GIS data local; commit a short sanitized report or test that reproduces the bug.
Follow [the handoff](../HANDOFF.md) for the ordered acceptance checks and unresolved decisions.
