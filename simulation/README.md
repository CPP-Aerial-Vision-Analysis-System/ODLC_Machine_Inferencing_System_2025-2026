<!-- PURPOSE: be the front door for running, changing and testing the ASTRA Gazebo digital twin, with short operator recipes and links to the detailed guides. -->

# ASTRA Gazebo simulator

Run the team's mission software against a virtual drone: Gazebo Harmonic supplies physics and camera
images, ArduCopter SITL replaces the Pixhawk, and a SIYI camera emulator answers the real camera protocol.
The demo searches for a hiker and tent, drops both payloads, returns home, and writes a score report.
The current evidence and unfinished validation are in [the handoff](../docs/HANDOFF.md).

## Run it

In Ubuntu 22.04, start at the checkout root. On Windows use the `Ubuntu-22.04` WSL distro.

```bash
sudo bash simulation/setup/01_install_humble.sh
bash simulation/setup/02_setup_ardupilot.sh
bash simulation/setup/03_build.sh --workers 4
cd ~/ODLC_Machine_Inferencing_System_2025-2026
source simulation/setup/sim_env.sh
bash simulation/scripts/demo.sh runway
# Gazebo and RViz open; watch it in Mission Planner (ASTRA plugin). No screen? Add --headless.
# Read ~/.astra_sim/runs/<date>-<time>_runway/score_report.md when it lands.
# Use --detector truth to test mission logic with answer-key detections.
bash simulation/scripts/stop_sim.sh
```

On a native Linux checkout, `03_build.sh` builds in place: stay in that checkout instead of changing
to the mirror directory. When started from a Windows-mounted checkout, the script mirrors it to the
Linux directory above and builds there. Edit the Windows checkout, then run its `03_build.sh` again;
edits made directly to the mirror are overwritten. Never run colcon in the Windows `D:` checkout.

The real-terrain field is `suas_field_real` (needs the one-time `03_build.sh`, which makes its models):

```bash
bash simulation/scripts/demo.sh suas_field_real            # automatic: search Search Boundary 1, score report at the end
bash simulation/scripts/demo.sh suas_field_real --manual   # Mission Planner: connect UDP 14550, ASTRA tab, PLAN > Load WP file
```

Heights there are derived from the real ground (58.7 m / 55.8 m above the pad). The AI-sharper ground is made by `03_build.sh`; `python3 scripts/build_real_field.py verify` proves your PC builds the same world as the
reference PC, and `... blender` opens it in Blender (see
[WORLDS](../docs/simulation/WORLDS.md#will-another-pc-build-exactly-this-world-verify)).

Keep one simulator, full build, Docker build or large world-generation job running at a time. Stop the
simulator before rebuilding. A headless run still renders camera images and needs graphics support;
`truth` removes model inference from the mission test, while the team detector node still starts and
may load its model. `truth` is not a claim about model accuracy or a guarantee of a GPU-free process tree.

On Windows, [simulation/windows](windows/) contains `Rebuild.bat`, `Start Search Demo.bat`,
`Start for Mission Planner.bat`, `Start Over.bat` and `Stop Simulation.bat`. They use `Ubuntu-22.04`.
For automation, put Linux commands in a `.sh` file and invoke it from PowerShell with
`wsl -d Ubuntu-22.04 -- bash "<script path as seen by WSL>"`; avoid nested inline shell quoting.
Machine-specific paths and instructions belong in ignored `LOCAL_NOTES.md`.

## Which guide do I need?

| I want to... | Read |
|---|---|
| Find any doc in the repo (the list of every doc, and the rules for keeping them up to date) | [docs/README.md](../docs/README.md) |
| Understand nodes, topics, ports and mission flow | [ARCHITECTURE](../docs/simulation/ARCHITECTURE.md) |
| Swap ArduPilot, the Jetson code or the detection model with one command (and know it worked) | [SWAPPING](../docs/simulation/SWAPPING.md) |
| Test another recognition model or plug in a new node | [PLUGINS](../docs/simulation/PLUGINS.md) |
| Change SITL version, source build, parameters or remote connection | [ARDUPILOT](../docs/simulation/ARDUPILOT.md) |
| Watch, upload, arm and fly through Mission Planner and its ASTRA plugin (camera in the HUD); restart a run | [MISSION_PLANNER](../docs/simulation/MISSION_PLANNER.md) |
| Build a world; run, check and fly the real-terrain field (`suas_field_real`), its accuracy and the AI-sharper ground | [WORLDS](../docs/simulation/WORLDS.md#real-terrain-world-suas_field_real) |
| Run the team stack on the Jetson Orin Nano (USB-C cable or plain SSH) or on the virtual Jetson container while the simulator stays here: `demo.sh runway --jetson usb\|ssh\|virtual` | [Jetson guide](../docs/jetson/README.md), [the link](../docs/jetson/LINK.md), [flying](../docs/jetson/FLYING.md) |
| Run containers or set up another PC (no Docker login needed: [details](../docs/simulation/DOCKER.md#no-login)); any Docker error: run the doctor `bash simulation/docker/check_host.sh` (Windows side: `simulation\docker\check_host.ps1`) and see [its table](../docs/simulation/DOCKER.md#troubleshooting) | [DOCKER](../docs/simulation/DOCKER.md) |
| Compare locations with Google Earth | [GOOGLE_EARTH](../docs/simulation/GOOGLE_EARTH.md) |
| Diagnose a failure | [TROUBLESHOOTING](../docs/simulation/TROUBLESHOOTING.md) |
| Review changes to the team's software | [WHAT_CHANGED](../docs/simulation/WHAT_CHANGED.md) |
| Continue development as an assistant | [AGENTS.md](../AGENTS.md), then [HANDOFF](../docs/HANDOFF.md) |

## Test a recognition model

For weights supported by the team's existing Ultralytics/SAHI pipeline, place the model outside the
public checkout and run:

```bash
source simulation/setup/sim_env.sh
export ASTRA_DETECTOR_MODEL="$HOME/.astra_sim/models/my_detector.pt"
bash simulation/scripts/demo.sh runway --headless
```

The model must produce classes the team's [class mapping](../ros2_ws/src/detection/detection/detection_processor.py)
understands. A successful load alone does not establish compatible class IDs or useful detections. The
simulation wrapper uses the SAHI path (`batched:=false`); a Jetson TensorRT `.engine` is not portable PC
weights. To make the choice explicit in a repeatable profile, use
`--set detector.model_path="$HOME/.astra_sim/models/my_detector.pt"`.

For a different inference library, input format or label mapping, write a detector adapter that emits
`interfaces/msg/ImageResult`, and select it through `detector.kind: custom`. The
[plug-in guide](../docs/simulation/PLUGINS.md) shows the template, required message fields, QoS, readiness
service and conformance checks. Rebuild after adding the package. Check the saved photos and report;
compare `real` runs with `real` runs under the same camera, world, mission and model settings.

## Start pieces by hand

OR BY HAND: terminal 1 runs the simulator, terminal 2 the team stack, terminal 3 the ground-station steps.
Source `simulation/setup/sim_env.sh` in each Ubuntu terminal.

```bash
# Terminal 1
bash simulation/scripts/start_sim.sh runway
# Terminal 2, from the Linux checkout root
FCU_URL=tcp://127.0.0.1:5763 TRACKER_CMD="ros2 launch sim_bringup team_stack.launch.py" ./launcher.sh sim
# Terminal 3: wait until the model is ready, then upload, arm and fly
bash simulation/scripts/fly_mission.sh runway --watch
# Stop the manually started launcher with Ctrl+C in terminal 2, then:
bash simulation/scripts/stop_sim.sh
```

With profiles, use the matching terminal-2 command printed by `start_sim.sh --profile NAME --dry-run`.
The wrapper includes the team's flight launch and supplies a laptop model path. On hardware the usual
`./launcher.sh` still selects the serial MAVROS connection and the team's defaults.

## Watch and score a run

The GUI has a Gazebo chase view and RViz markers. **Mission Planner with the ASTRA plugin is the ground
station**: its HUD shows the drone camera (the `pov_server` stream, `http://127.0.0.1:8081/pov.mjpg`) and its
ASTRA tab shows the mission phases, detections, chosen targets, drops and buttons
([MISSION_PLANNER](../docs/simulation/MISSION_PLANNER.md)). The old Qt demo panel opens only with
`demo.sh --panel`. `--headless` opens no Gazebo or RViz window. A runway run normally takes
several minutes plus autopilot/model startup. `Arm: Gyros inconsistent` during initial settling is expected.

Each run is saved under `~/.astra_sim/runs/<date>-<time>_<world>/` (for example `20261031-092448_runway`):
`sim.log`, `team_stack.log`, `phases.jsonl`,
`events.jsonl`, copied photos/sidecars, detections, `run_data.json` and Markdown/JSON score reports.
The report distinguishes projected **aim error**, old photo-GPS error, and actual payload **drop error**.
Inspect whether a drop used a Gazebo landing point or the release-position fallback before comparing it.
`--detector truth` is labelled as an answer-key run in the report (and in the old panel).
Every line of `demo.log` (this window, from the first check), `sim.log` and `team_stack.log` starts with its date and
time. Every run also has spreadsheet files in `csv/` (telemetry, events, phases,
photos, detections, targets and drops; written live from MAVLink by the flight recorder on UDP 14555) and
`run_summary.md` (what ran, the result, phase times, every warning/error with file:line); `~/.astra_sim/runs/runs_index.csv`
has one row per run. Look at them with `bash simulation/scripts/logs.sh` ([LOGS_AND_DATA](../docs/simulation/LOGS_AND_DATA.md)).

The default camera profile is `zr10` (2560x1440 stills); `--camera a8mini` requests the emulator's 4K profile.
`--zoom survey` keeps the full 81-degree view used to plan the lanes. `--zoom team` allows the team's 5x
rack and needs matching survey spacing. Photos now queue during lens work; queue-full drops are failures
to investigate. `Could not confirm 5.0x` is expected when survey mode caps the emulator at 1x.

## Useful commands

| Command | Purpose |
|---|---|
| `bash simulation/scripts/astra.sh help` | The front door: one command for everything below (`astra.sh demo`, `manual`, `stop`, `mission ...`); every command with examples: [COMMANDS](../docs/simulation/COMMANDS.md), every error message and its fix: [ERRORS](../docs/simulation/ERRORS.md) |
| `bash simulation/scripts/mission.sh check FILE` | Explain a Mission Planner `.waypoints` / QGroundControl `.plan` file and check that the team code can fly it (no simulator needed; real-drone files too) |
| `bash simulation/scripts/mission.sh fly FILE --watch` | Check, upload (+ MAVROS re-read), arm and fly a mission file in the running simulator (`load FILE` only uploads; `make WORLD` plans a new one) |
| `bash simulation/scripts/demo.sh runway --keep` | Fly automatically and keep the simulator open |
| `bash simulation/scripts/demo.sh runway --manual` | Let Mission Planner upload, arm and start |
| `bash simulation/scripts/start_over.sh --fly --follow` | Land if necessary, reset the current world, and fly again; runtime validation pending |
| `bash simulation/scripts/take_photo.sh` | Trigger the team camera path |
| `bash simulation/scripts/take_photo.sh --direct` | Trigger the emulator directly |
| `bash simulation/scripts/upload_mission.sh runway` | Upload only; also request MAVROS re-read |
| `bash simulation/scripts/status.sh --no-mavlink` | Inspect state without opening the GCS UDP port |
| `bash simulation/scripts/logs.sh summary` | The newest run's summary; also `list`, `errors`, `tail sim`, `csv` (Windows path too), `export`, `bundle` ([LOGS_AND_DATA](../docs/simulation/LOGS_AND_DATA.md)) |
| `bash simulation/scripts/demo.sh runway --panel` | Also open the old Qt demo panel (off by default) |
| `bash simulation/scripts/mission_planner.sh start` | Open Mission Planner through WSL interop (also installs or updates the ASTRA plugin) |
| `bash simulation/scripts/stop_sim.sh` | Stop the recorded simulator session |
| `ros2 launch sim_bringup sim.launch.py --show-args` | Inspect all launch settings |
| `python3 simulation/scripts/check_docs.py` | Check that every doc is in the index `docs/README.md` and no doc link is broken |

Use `--help` on the scripts for their exact options. `--all` on the stop script also targets older simulator
sessions; use the ordinary session stop during development. A launcher started by hand should be stopped
from its own terminal. Closing the Gazebo window alone leaves its server running.

## Edit and verify

Change sources in the master checkout, stop the simulator, and rerun `simulation/setup/03_build.sh` from
that checkout. Use `--packages "sim_plugins"` to build a package with its dependencies or the full build
for integration changes. Source the Linux environment again before testing.

For example, run a package's functional tests with the existing lint exclusions:

```bash
cd ros2_ws/src/simulation/sim_plugins
python3 -m pytest -q test --ignore=test/test_flake8.py --ignore=test/test_pep257.py --ignore=test/test_copyright.py
```

For integration changes, run a complete demo (`--headless` only on a computer without a screen) and check its
score report and live-node evidence.
See [HANDOFF section 3](../docs/HANDOFF.md#3-how-to-check-your-work) for the required package set.
Keep new files' purpose comments, tests and matching documentation together; a new, moved or renamed doc
also gets its line in [docs/README.md](../docs/README.md), then run `python3 simulation/scripts/check_docs.py`.
Keep model weights, photos, GIS downloads, run folders, secrets, personal addresses and local machine paths
out of this public repo.
