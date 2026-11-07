<!--
docs/README.md -- PURPOSE: the documentation index of the ASTRA repo. It lists EVERY Markdown file in the repo
(where it is, what it is for in one plain sentence, and who it is for), grouped by what a reader wants to do,
plus the rules for keeping the docs up to date. The checker simulation/scripts/check_docs.py fails when a doc
is missing from this page, when a link here or inside any doc is broken, or when a doc names a script that
does not exist; the test ros2_ws/src/simulation/sim_plugins/test/test_docs_index.py runs it. When you add,
move or rename a doc, update this page in the same change.
-->

# ASTRA documentation index

This page lists **every document in this repo**: where it is, what it is for, and who should read it.
ASTRA is our team's software for the RoboNation **SUAS 2026** competition. The real drone is an Iris-class
quadcopter with a **Jetson Orin Nano** (ROS 2 Humble) running our flight code, a **Pixhawk** with ArduCopter
4.6.3, and a **SIYI A8 mini** gimbal camera. The repo also holds a **digital twin**: the same flight code
flying a simulated drone in Gazebo, with Mission Planner as the ground station.

Click a path to open the doc. Paths are written from the top folder of the repo.

> **Where do I start?** The five most important docs:
>
> 1. [`README.md`](../README.md): what this repo is, and how to open the team's development container.
> 2. [`simulation/README.md`](../simulation/README.md): run the digital twin (one demo command, then what you see).
> 3. [`docs/simulation/MISSION_PLANNER.md`](simulation/MISSION_PLANNER.md): Mission Planner, the ASTRA plugin tab
>    and the drone camera in its HUD: how you watch and fly a mission.
> 4. [`docs/simulation/ARCHITECTURE.md`](simulation/ARCHITECTURE.md): how the pieces fit together (programs,
>    ports, topics, the mission step by step).
> 5. [`docs/HANDOFF.md`](HANDOFF.md): what is finished, what was proven, and what to do next.
>
> Looking for a command? [`docs/simulation/COMMANDS.md`](simulation/COMMANDS.md) has every one.
> Something broke? Read [`docs/simulation/TROUBLESHOOTING.md`](simulation/TROUBLESHOOTING.md).
> The complete technical map of the repo (every node, topic, port and file) is [`AGENTS.md`](../AGENTS.md).

Words used below: **operator** = the person running the simulator or the ground station; **developer** = a
teammate changing code; **reviewer** = someone checking a change before it flies.

## Start here

| Doc | What it is for | Who it is for |
|---|---|---|
| [`README.md`](../README.md) | The front page of the repo: the development container that copies the Jetson's software, Docker targets, and a short pointer to the simulator. | everyone, first |
| [`docs/HANDOFF.md`](HANDOFF.md) | The hand-over file: the state of every part, the evidence (run folders, numbers), the ordered task list and the rules. | teammates and AI assistants continuing the work |
| `docs/README.md` (this page) | The list of every doc and the rules for keeping the docs up to date. | everyone |

## Fly the simulator

| Doc | What it is for | Who it is for |
|---|---|---|
| [`simulation/README.md`](../simulation/README.md) | The simulator's front door: install, build, run the demo, start pieces by hand, read the score, useful commands. | operators, new teammates |
| [`docs/simulation/ARCHITECTURE.md`](simulation/ARCHITECTURE.md) | How the digital twin works: which program talks to which, the ports, the interfaces to keep stable, one automatic run step by step. | developers |
| [`docs/simulation/WHAT_CHANGED.md`](simulation/WHAT_CHANGED.md) | What the simulator work added, which bugs in the team's flight code it found and fixed, and what still has to be proven on the real drone. | reviewers, the team |
| [`docs/simulation/VIDEO_PIPELINE.md`](simulation/VIDEO_PIPELINE.md) | Design (not built yet): how the twin runs the team's new camera system (ZR10 control, RTSP video stream, yolo26 detector) unchanged and as in the field: every connection, the simulated stream, what reaches the mission logic, what each run logs, what still differs, the build plan and the open decisions; also the field-safety findings for the team (proposals P1-P10: the boot launcher ignores TRACKER_CMD, the end-of-survey hold that never ends without the photo folder, pictures lost on the way to the detector). | developers, reviewers, the owner |

## Mission Planner and the ASTRA plugin

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/MISSION_PLANNER.md`](simulation/MISSION_PLANNER.md) | Mission Planner as the ground station: which connection to pick, the ASTRA plugin tab (phases, hiker and tent cards, buttons), the drone camera in the HUD (the `pov_server` stream on port 8081), flying by hand, START OVER. | operators, pilots |

The plugin itself is one file, [`simulation/windows/mission_planner_plugin/astra_gcs.cs`](../simulation/windows/mission_planner_plugin/astra_gcs.cs)
(its top comment explains it). The old Qt "ASTRA Simulation" demo panel is off by default; `demo.sh --panel`
brings it back (described in [`sim_demo`'s README](../ros2_ws/src/simulation/sim_demo/README.md)).

## Jetson

The real drone's computer is a **Jetson Orin Nano Developer Kit** (the team notes say JetPack 6.2 = Ubuntu
22.04, L4T R36.4, ROS 2 Humble). Until the Jetson guides listed under [Being written](#being-written) exist,
the facts are in [`AGENTS.md` section 3.1](../AGENTS.md#31-hosts-and-links) (hosts, links, addresses) and the
Jetson's start script is `launcher.sh` at the repo root.

The laptop reaches the Jetson through **any address it can SSH to**: the USB-C cable (the Jetson's USB
address is `192.168.55.1`) or, with no cable at all, Ethernet, Wi-Fi or the team's network (a name such as
`jetson.local`, or its IP address: `hostname -I` on the Jetson prints it). SSH works the same over both; the
Jetson kit that is being written takes that address as its target, so the USB cable is optional.

| Doc | What it is for | Who it is for |
|---|---|---|
| [`HowTos.md`](../HowTos.md) | Team note: how to log in to the Jetson over SSH (and `hostname -I` to find its address). | teammates at the drone |
| [`docs/jetson/FLYING.md`](jetson/FLYING.md) | Jetson-in-the-loop flight procedure and operator checks for the existing kit. | pilots and simulator operators |
| [`ros2_ws/src/video_cam/video_cam/siyiInstructions.md`](../ros2_ws/src/video_cam/video_cam/siyiInstructions.md) | Team note: check that the Jetson sees the SIYI camera on the network (ip, ping, HTTP API, SDK). | teammates at the drone |
| [`ros2_ws/src/fixtensorRT.md`](../ros2_ws/src/fixtensorRT.md) | Team note: fix the protobuf / ONNX / TensorRT version clash on the Jetson. | whoever sets up the Jetson's AI stack |
| [`ros2_ws/src/ThingsToDo.md`](../ros2_ws/src/ThingsToDo.md) | Team note: the same protobuf / TensorRT fix, written as a to-do list. | whoever sets up the Jetson's AI stack |
| [`ros2_ws/src/HowToHandleMemoryIssues.md`](../ros2_ws/src/HowToHandleMemoryIssues.md) | Team note (one line): free memory by stopping a stuck process with `pkill -f`. | teammates at the drone |

## Plug-and-play and swapping

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/SWAPPING.md`](simulation/SWAPPING.md) | **Start here to swap something.** Swap one of the three big pieces of the twin (the ArduPilot autopilot, the Jetson code, the detection model) with one copy-paste command each, how to know it worked, how to go back, and the common errors. | anyone testing another ArduPilot, code branch or model |
| [`docs/simulation/PLUGINS.md`](simulation/PLUGINS.md) | Replace one piece of the simulator (detector, mission brain, camera, autopilot, world, extras) with a YAML profile, and check that the new piece behaves like the old one. Parts of it are generated from the code. | developers testing a new model or node |
| [`simulation/profiles/README.md`](../simulation/profiles/README.md) | The list of ready-made profiles and what each one changes. | developers |
| [`ros2_ws/src/simulation/sim_plugin_examples/README.md`](../ros2_ws/src/simulation/sim_plugin_examples/README.md) | The two copy-and-rename templates for your own detector or extra program. | developers |
| [`ros2_ws/src/simulation/sim_plugin_examples/example_detector/README.md`](../ros2_ws/src/simulation/sim_plugin_examples/example_detector/README.md) | A template detector that reads the team's photos and publishes `/image_detection` exactly like `new_od`. | developers writing a detector |
| [`ros2_ws/src/simulation/sim_plugin_examples/example_listener/README.md`](../ros2_ws/src/simulation/sim_plugin_examples/example_listener/README.md) | A template "extra" program that only listens and logs a flight summary. | developers writing a monitor or logger |

## Runs, logs and CSV

Every simulator run writes a run folder `~/.astra_sim/runs/<date>-<time>_<world>/` (logs, phases, photos,
detections, the score report). Until the guide listed under [Being written](#being-written) exists, the run
folder is explained in [`simulation/README.md`, "Watch and score a run"](../simulation/README.md#watch-and-score-a-run)
and [`AGENTS.md` section 4.7](../AGENTS.md#47-demosh-run-folders-score-report); the score report's fields are
in the [`sim_demo` README](../ros2_ws/src/simulation/sim_demo/README.md).

## Commands and errors

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/COMMANDS.md`](simulation/COMMANDS.md) | Every command of the digital twin in one place, through one front door (`simulation/scripts/astra.sh`): run, stop, start over, photos, mission plans, status, logs, setup, the Jetson, and the exit codes. | operators, developers |
| [`docs/simulation/ERRORS.md`](simulation/ERRORS.md) | Every error and warning message the simulator's commands print, grouped by command: what it means and exactly how to fix it (search the page for the `[program]` name at the start of the message). | operators, developers |
| [`docs/simulation/TROUBLESHOOTING.md`](simulation/TROUBLESHOOTING.md) | Find the failing link when a run goes wrong: build, start-up, camera, detector, mission, windows, stuck processes. | operators, developers |

Every script in `simulation/scripts/` prints its own help: `bash simulation/scripts/demo.sh --help`.
The docs checker is `simulation/scripts/check_docs.py` (see [Keeping the docs up to date](#keeping-the-docs-up-to-date)).

## Code map (watch the code run)

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/CODE_MAP.md`](simulation/CODE_MAP.md) | The short guide to the code map: a web page that draws all our software as boxes you can open, and lights up every function while the simulator flies (`bash simulation/scripts/demo_traced.sh`, then http://127.0.0.1:8090/). | everyone, new teammates |
| [`simulation/codemap/README.md`](../simulation/codemap/README.md) | The code map's full reference: rebuilding the map, explanations, the shareable artifact page, how the tracer stays off on the real drone, what tracing costs, every file, the ids and file formats, tests. | developers |

## Worlds and missions

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/SUAS_2027.md`](simulation/SUAS_2027.md) | The supplied 2027 primer mapped to separate Blender/Gazebo training cases, private truth, local diagnostics, generation commands and remaining acceptance work. | terrain, simulator and ML developers |
| [`docs/simulation/DETECTION.md`](simulation/DETECTION.md) | What the simulator's image detector really is (generic COCO weights), why trees were taken for hikers and tents (measured), the opt-in physical size filter, and the exact steps to connect and check your own detection model. | everyone working on detection |
| [`docs/simulation/WHY_THESE_CHOICES.md`](simulation/WHY_THESE_CHOICES.md) | Why each tool is used (WSL2, ROS 2 Humble, Docker, Gazebo, ArduPilot SITL, MAVROS, Mission Planner that always opens, real terrain, Blender, AI upscale, mesh physics, locks), the reference PC, and what was deliberately removed. | everyone, before changing a tool or a rule |
| [`docs/simulation/WORLDS.md`](simulation/WORLDS.md) | The simulated fields (`runway` and the real-terrain `suas_field_real`), the rules every world must follow, how to run, regenerate and check them, the accuracy measurements, and the terrain data and its licences. | developers changing worlds or missions |
| [`docs/simulation/GOOGLE_EARTH.md`](simulation/GOOGLE_EARTH.md) | Compare simulator locations with Google Earth without copying Google's imagery into the repo. | anyone checking positions |
| [`ros2_ws/src/simulation/sim_worlds/data/suas_field_real/LICENSES.md`](../ros2_ws/src/simulation/sim_worlds/data/suas_field_real/LICENSES.md) | Where every file of the real-field site pack comes from and how to credit it (generated). | developers, reviewers of data licences |

## ArduPilot

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/ARDUPILOT.md`](simulation/ARDUPILOT.md) | How the simulator picks the ArduCopter SITL version, a source build, extra parameter files, or an autopilot on another computer. | developers |

## Docker

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/simulation/DOCKER.md`](simulation/DOCKER.md) | Run the simulator and a Jetson stand-in in containers, set up another PC, and fix every known Docker error (no Docker login needed). | developers on another PC |

The development container (the Jetson's software on any laptop) is explained in the root [`README.md`](../README.md).

## Lattice

The owner handed the Lattice work to the team on 2026-10-31. **Start with
[`docs/lattice/HANDOFF.md`](lattice/HANDOFF.md).**

| Doc | What it is for | Who it is for |
|---|---|---|
| [`docs/lattice/HANDOFF.md`](lattice/HANDOFF.md) | **Start here for Lattice.** The hand-over note (2026-10-31): what is built and proven, where the work stopped, what to do next in order, the commands to check it still works, and the safety rules. | teammates taking over the Lattice work |
| [`docs/lattice/README.md`](lattice/README.md) | The front door of the Anduril Lattice work: what Lattice is, which file to read, how far each phase got. | everyone curious about Lattice |
| [`docs/lattice/LATTICE_PLAN.md`](lattice/LATTICE_PLAN.md) | The engineering plan: what we publish to Lattice, where the bridge runs, phases with "done when" checks. | developers, reviewers |
| [`docs/lattice/DATA_CONTRACT.md`](lattice/DATA_CONTRACT.md) | The exact data we send to Lattice, field by field, and where each value comes from. | developers, reviewers |
| [`docs/lattice/BRIDGE.md`](lattice/BRIDGE.md) | How to run the Lattice bridge step by step, with the simulator and a pretend Lattice (no account needed). | operators, developers |
| [`ros2_ws/src/lattice_bridge/README.md`](../ros2_ws/src/lattice_bridge/README.md) | The `lattice_bridge` package: its interface, safety rules, examples and tests. | developers |
| [`ros2_ws/src/lattice_bridge/test/golden/README.md`](../ros2_ws/src/lattice_bridge/test/golden/README.md) | The "golden" example requests the tests compare against, and how to change them safely. | developers |

## The team's flight-code packages

These run on the real drone (and unchanged in the simulator). The newer READMEs (main, mapping,
waypoint_mavros, HOW_OUR_CAMERA_WORKS) match the code; the older team notes are kept as written.

| Doc | What it is for | Who it is for |
|---|---|---|
| [`ros2_ws/src/main/README.md`](../ros2_ws/src/main/README.md) | The mission controller `main_controller`: the hold, the aiming at a target, the drops, RTL, its settings and tests. | flight-code developers |
| [`ros2_ws/src/mapping/README.md`](../ros2_ws/src/mapping/README.md) | The survey photo trigger `mission_camera_trigger`: when photos start and stop, its timing settings, its tests. | flight-code developers |
| [`ros2_ws/src/waypoint_mavros/README.md`](../ros2_ws/src/waypoint_mavros/README.md) | `waypoint_manager`: the copy of the mission, its index parameters, `/addWaypoint`, keeping the mission fresh. | flight-code developers |
| [`ros2_ws/src/video_cam/HOW_OUR_CAMERA_WORKS.md`](../ros2_ws/src/video_cam/HOW_OUR_CAMERA_WORKS.md) | The SIYI camera driver: how a photo is taken and saved, the JSON "sidecar" next to each photo, the photo orientation test. | flight-code developers |
| [`ros2_ws/src/video_cam/fullrun.md`](../ros2_ws/src/video_cam/fullrun.md) | Team note: an older two-terminal quick start for the camera pipeline plus detection (not the flight launch). | camera developers |
| [`ros2_ws/src/video_cam/video_cam/QUICK_START.md`](../ros2_ws/src/video_cam/video_cam/QUICK_START.md) | Team note: the SIYI camera start guide (January 2026). | camera developers |
| [`ros2_ws/src/detection/detection/README.md`](../ros2_ws/src/detection/detection/README.md) | How to run and stop the detector node `new_od`. | ML / detector developers |
| [`ros2_ws/src/detection/Documentation/README.md`](../ros2_ws/src/detection/Documentation/README.md) | Team note (older): the detection package's two detection systems; some of its links point to files that were deleted. | ML / detector developers |
| [`ros2_ws/src/detection/Documentation/Documentation.md`](../ros2_ws/src/detection/Documentation/Documentation.md) | Team note (November 2025): image capture and detection workflow. | ML / detector developers |
| [`ros2_ws/src/detection/Documentation/EXECUTION_FLOW.md`](../ros2_ws/src/detection/Documentation/EXECUTION_FLOW.md) | Team note (older): what happens step by step in the old `detection_complete.launch.py`. | ML / detector developers |
| [`ros2_ws/src/detection/Documentation/ods_documentation.md`](../ros2_ws/src/detection/Documentation/ods_documentation.md) | Team note (older): function-by-function reference of the SAHI detection code. | ML / detector developers |
| [`ros2_ws/src/detection/Documentation/od_upgrade.md`](../ros2_ws/src/detection/Documentation/od_upgrade.md) | Team note (November 2025): differences between two versions of the SAHI detector. | ML / detector developers |
| [`ros2_ws/src/detection/Documentation/DocForML.md`](../ros2_ws/src/detection/Documentation/DocForML.md) | Team note: ML to-do list (training images, hard negatives, SAHI settings). | ML developers |
| [`ros2_ws/src/detection/Documentation/TasksForML.md`](../ros2_ws/src/detection/Documentation/TasksForML.md) | Team note: making training images with a Blender script. | ML developers |

## The simulator packages

Each package of the digital twin (under `ros2_ws/src/simulation/`) has a README with its programs, topics,
settings and tests.

| Doc | What it is for | Who it is for |
|---|---|---|
| [`ros2_ws/src/simulation/sim_bringup/README.md`](../ros2_ws/src/simulation/sim_bringup/README.md) | The simulator's start button (`sim.launch.py`) and the helper tools the scripts use (mission upload, phases, readiness checks). | simulator developers |
| [`ros2_ws/src/simulation/sim_models/README.md`](../ros2_ws/src/simulation/sim_models/README.md) | The simulated drone: Iris + SIYI A8 mini gimbal camera + two droppable payloads, and the extra SITL settings. | simulator developers |
| [`ros2_ws/src/simulation/sim_worlds/README.md`](../ros2_ws/src/simulation/sim_worlds/README.md) | The simulated fields, their search missions and their answer keys (truth files), and the tools that build them. | simulator developers |
| [`ros2_ws/src/simulation/sim_worlds/hooks/README.md`](../ros2_ws/src/simulation/sim_worlds/hooks/README.md) | Why the environment hook exists (it tells Gazebo where the world models are). | simulator developers |
| [`ros2_ws/src/simulation/siyi_emulator/README.md`](../ros2_ws/src/simulation/siyi_emulator/README.md) | The pretend SIYI camera that answers the real camera protocol, so the team's camera driver runs unchanged. | simulator developers |
| [`ros2_ws/src/simulation/sim_demo/README.md`](../ros2_ws/src/simulation/sim_demo/README.md) | What you watch and the score: the old demo panel, the answer-key detector, RViz markers, the score report, START OVER, the camera stream for Mission Planner. | simulator developers, operators |
| [`ros2_ws/src/simulation/sim_plugins/README.md`](../ros2_ws/src/simulation/sim_plugins/README.md) | The plug-and-play package's commands and modules (the user guide is PLUGINS.md). | simulator developers |

## Team notes and plans

Personal notes and plans written by teammates. They are kept as written (not checked against the code).

| Doc | What it is for | Who it is for |
|---|---|---|
| [`things to test.md`](../things%20to%20test.md) | Team note: tests to do on the real hardware (photos per minute, and more). | the team |
| [`drone_ui_functions_plan.md`](../drone_ui_functions_plan.md) | Team plan (stale): an analysis of the ROS 2 nodes and a proposed operator UI. | the team |
| [`GtihubLearning.md`](../GtihubLearning.md) | Team note: how git stores files (commits, trees, blobs). | anyone learning git |
| [`LearnC++ROS2/oneohone.md`](../LearnC++ROS2/oneohone.md) | Team note: a first C++ ROS 2 package, step by step. | anyone learning C++ ROS 2 |

## For AI assistants

| Doc | What it is for | Who it is for |
|---|---|---|
| [`AGENTS.md`](../AGENTS.md) | The complete guide for AI coding assistants: the repo map, how the real drone and the simulator work, build/run/test, the rules, recipes and traps. People can read it too. | AI coding assistants, developers |
| [`CLAUDE.md`](../CLAUDE.md) | Claude Code's entry file: it only imports `AGENTS.md`. | Claude Code |

## Third-party files (not ours)

[`ros2_ws/src/ultralytics_ros/README.md`](../ros2_ws/src/ultralytics_ros/README.md) belongs to a vendored copy
of the Ultralytics ROS package (it has `COLCON_IGNORE`, so it is never built). Its other Markdown files are
skipped by the checker on purpose (the allow-list in `simulation/scripts/check_docs.py` says why).

## Being written

These docs are planned by work that is running now. They do not exist yet; the checker accepts them here.
When one is added, move its line into its group above and delete it here.

* Jetson: [`docs/jetson/README.md`](jetson/README.md) (set up and run the Jetson Orin Nano: the
  first-connection kit, `jetson.sh check`, then setup and deploy),
  [`docs/jetson/HIL_DESIGN.md`](jetson/HIL_DESIGN.md) (the real Jetson in the loop with the simulator),
  [`docs/jetson/LINK.md`](jetson/LINK.md) (how the laptop reaches the Jetson, over the USB-C cable or plain
  SSH on Ethernet or Wi-Fi with no cable, what changes on Wi-Fi, and how the Jetson is wired to the Pixhawk).
* Runs, logs and CSV: [`docs/simulation/LOGS_AND_DATA.md`](simulation/LOGS_AND_DATA.md) (every log and data file of a
  run, including the CSV files the flight recorder writes during the flight).

## Keeping the docs up to date

These rules keep this page true. Follow them in the **same change** as the code or doc you touch.

1. **You add, move or rename a doc:** add or change its line on this page (a link with its path, one plain
   sentence on what it is for, who it is for), and fix every link that pointed to the old place.
2. **You change a command, a script name, an option, a port, a topic or a default:** update every doc that
   names it, and `AGENTS.md` if it is there. Find them with `git grep -n "old name"`.
3. **Every doc starts with a PURPOSE comment** (`<!-- PURPOSE: ... -->`), uses plain words, gives copy-paste
   commands, and says how you know a step worked.
4. **Links are relative** (for example `../simulation/README.md`), so they work on GitHub and in an editor.
5. **A planned doc** (and a planned script you want to name already) goes under [Being written](#being-written);
   when it exists, move it to its group. Everywhere else, a link or script name must point to a real file.
6. **A third-party or private Markdown file** that should not be listed goes on the `NOT_INDEXED` allow-list
   at the top of `simulation/scripts/check_docs.py`, with the reason.
7. **Then run the checker** and fix every line it prints:

```bash
python3 simulation/scripts/check_docs.py                  # Ubuntu / WSL, from the repo folder
python simulation\scripts\check_docs.py                   # Windows PowerShell, from the repo folder
python3 -m pytest -q -p no:cacheprovider ros2_ws/src/simulation/sim_plugins/test/test_docs_index.py
```

How you know it worked: the checker's last line starts with `[check_docs] OK:` and the test says `passed`.
If it prints `[check_docs] ERROR: ...`, each line says which file and line, what is wrong, the likely cause
and the fix.
