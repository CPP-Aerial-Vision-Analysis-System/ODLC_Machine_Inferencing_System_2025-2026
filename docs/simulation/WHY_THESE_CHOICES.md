<!-- PURPOSE: explain, in plain words, WHY the simulator uses each tool and rule it uses (WSL2, Ubuntu 22.04, ROS 2 Humble, Docker, Gazebo Harmonic, ArduPilot SITL, MAVROS, Mission Planner, real terrain data, Blender, AI upscaling, mesh physics, the locks) and what was deliberately removed, so a new teammate does not undo a decision by accident. -->

# Why we use what we use

Every tool in the simulator is there for a reason. This page gives the reason in plain words, says what you
would lose by swapping the tool for something else, and says honestly where something is still work in
progress. If you want to *change* a choice, read its section first and then change the section too.

The one sentence behind all of it: **the simulator must behave like the real aircraft and the real field, so that a
result seen here can be trusted there.** The real drone comes first; the simulator serves it.

## The reference PC (what "the same result" means)

Results in the README ("flight lands after about 7.5 minutes, hiker found ...") were produced on this machine. Another
PC reproduces them when it matches this table closely enough; a weaker GPU or less memory only changes speed.

| Part | Reference PC |
|---|---|
| Windows | Windows 11 Home (10.0.26200) |
| Linux inside it | WSL2, distro `Ubuntu-22.04` (Python 3.10) |
| Graphics | NVIDIA GeForce RTX 4060 Laptop GPU, 8 GB; torch 2.8.0 with CUDA 12.6 for the detector and the optional upscaler |
| Robot software | ROS 2 Humble, Gazebo Harmonic, ArduCopter SITL **4.6.3** (the Pixhawk's firmware version) |
| Ground station | Mission Planner on Windows with the ASTRA plugin |
| Repo | cloned on the Windows disk; `03_build.sh` makes the Linux-side copy `~/ODLC_Machine_Inferencing_System_2025-2026` |

## Why Windows + WSL2 + Ubuntu 22.04

* **Mission Planner is a Windows program** and it is the ground station the team flies the real drone with. The
  simulator has to run beside it on the same PC.
* **ROS 2 Humble is built for Ubuntu 22.04.** WSL2 gives a real Ubuntu 22.04 (and GPU access for Gazebo and the
  detector) without a second computer or a dual boot. The Linux side talks to Windows over normal networking
  (Mission Planner listens on UDP 14550).
* **The Linux-side copy of the repo is on purpose.** Gazebo and `colcon` misbehave on `/mnt/d/...` paths with spaces and
  are slow there, so `03_build.sh` mirrors the repo to a fast Linux folder and builds there. Edit on Windows, build again.

## Why ROS 2 Humble

The team's flight software (`launcher.sh sim`: MAVROS, `video_cam`, `new_od`, `waypoint_manager`, `main_controller`)
is ROS 2 Humble code that also runs on the Jetson Orin Nano (JetPack 6 / Ubuntu 22.04). The simulator runs **that same
code**, unchanged, so a simulated flight is a test of the real software, not of a copy. Changing the ROS release would
mean changing the team's code.

## Why Docker (and where we do not use it yet)

Docker is used where **the same software must run identically on different computers**:

* **The team's dev container** (`docker/`): everyone builds and tests the ROS workspace with the same compilers, the same
  Python packages and the same ROS release, whatever their laptop has installed. "Works on my machine" stops being an answer.
* **The Jetson.** The team stack runs in a container on the real Jetson (and in a *virtual Jetson* container on a PC), so
  the software that flies is the software that was tested. See [the Jetson guide](../jetson/README.md).
* **Pinned, repeatable images** replace a long list of manual installs for a new teammate.

Where Docker is **not** the validated path today: the whole Gazebo simulator is run and validated **natively in WSL2**
(the README steps). The container version of the simulator exists ([DOCKER.md](DOCKER.md)) but its full flight is still
marked as work in progress there; use the native path to reproduce results. Native also gives Gazebo the GPU and the
Windows screen directly, which containers on a laptop make harder.

## Why Gazebo Harmonic

* It is the simulator the **ArduPilot Gazebo plugin** supports for ArduCopter SITL (physics step in lock-step with the
  autopilot).
* It renders camera images (ogre2) that go through the same SIYI-emulator and photo pipeline the real camera does, so the
  detector sees what a camera image looks like, with real exposure, shadows and perspective.
* It simulates payload release and landing physics, so a drop is scored by where the payload really lands.

## Why ArduPilot SITL 4.6.3 and MAVROS on SERIAL2

* **SITL runs the real ArduCopter firmware code** that is in the Pixhawk, at the version the aircraft flies (4.6.3), with
  its real parameters, modes and failsafes; we are not testing a toy autopilot.
* **MAVROS is connected to SITL's SERIAL2** (TCP 5763), exactly as the Jetson is wired to the Pixhawk's TELEM2. That is
  why the team code's status messages ("Detected tent", "Aim ...") reach Mission Planner just like in the aircraft.
* MAVProxy fans the MAVLink out (UDP 14550 for Mission Planner, others for tools); only the ports differ from the aircraft.

## Why Mission Planner ALWAYS opens

Mission Planner is the ground station of the real mission, so it is part of the system under test: the map, the HUD,
the mission upload, the status texts and the drone camera (the ASTRA tab) are what the operator uses. Letting people skip
it created runs that looked fine without it and failed with it, and "it works on my PC" answers that could not be
checked by the next person. Therefore **`demo.sh`, `start_sim.sh` and `compose_up.sh` always open Mission Planner and
always feed it MAVLink on UDP 14550; there is no switch to turn that off** (the old `--no-windows-gcs` option and the
profile field `network.windows_gcs` were removed). If a PC cannot open it (no Windows, not installed), the start says so
loudly and tells you how to connect by hand; it does not pretend everything is fine.

## Why the real terrain (and why the old flat world was deleted)

The competition field is not flat: the ground rises 2 to 12 m across Search Boundary 1, trees stand in lines and clumps,
there are dirt patches, a pond, tracks and buildings. For a camera at 47 m above the ground those things decide what the
detector sees (shadows, bare soil that looks like a tent, tall grass hiding a mannequin) and what the survey height must be
(150 ft above the **highest** ground). The old procedural `suas_field` was a flat field with invented trees; testing on it
would have told us nothing true about the real field, and keeping it around invited people to run "the old way" and
compare numbers that were not comparable. It was **deleted on 2026-11-07**. The small `runway` world stays as a fast
test field (about 7 minutes), not as a model of the competition site.

The terrain is built **only from public data** with clear licences ([WORLDS.md](WORLDS.md#data-permissions-and-public-repository-rules)):
USGS lidar (ground and tree heights), the USDA NAIP aerial photo (colour and tree crowns), OpenStreetMap and Microsoft
building outlines, CC0 ground detail. Google imagery is used only to *look at* the layout, never as texture (Google's terms
do not allow it).

## Why mesh physics, merged trees and a 2-million-triangle budget

* **Mesh collision (not a heightmap)** because a landing drone or a falling payload must touch exactly the ground that is
  drawn; the box-drop test proves they agree to 3 cm in the search areas.
* **Trees are merged into a few big meshes** because 2,000 separate tree models took 30 s and 5.5 GB to load, the merged
  version 3.5 s. Under about 2 million triangles keeps the real-time factor near 1.0 on the reference laptop (0.999 measured).
* **The real tree crowns are textured from the aerial photo** so that from above a tree looks like the photo's tree, and the
  trees do not cast their own Gazebo shadows because the photo already contains the real ones.

## Why Blender (and what it is NOT used for)

Blender is the *authoring and inspection* tool: it reads the same generated meshes and pictures Gazebo loads, lights them
with the real sun, and renders views so people can judge and compare the terrain with Mission Planner screenshots without
starting the whole simulator. Gazebo, not Blender, is what the detector sees in a flight. (The separate SUAS 2027 training
generator, [SUAS_2027.md](SUAS_2027.md), also uses Blender; that is independent of the 2026 field.)

## Why the optional AI upscale (and why it is optional)

The aerial photo has 25 cm pixels; the camera sees 2 to 3 cm per pixel, so the plain photo looks smeared. Real-ESRGAN invents
plausible fine texture (grass blades, slab cracks), which makes the ground look like a camera image. Because invented texture
could change what a detector does, it is **optional**, kept outside git, and has to be checked by comparing false-alarm counts of
two flights before it is trusted.

## Why one heavy job at a time (the locks)

The laptop has one GPU and limited memory. A build, a world generation and a flight at the same time make Gazebo's real-time
factor drop and results become unreproducible. Heavy jobs take `/mnt/wsl/astra_heavy.lock` (and simulator runs also
`/tmp/astra_sim.lock`), so they queue instead of fighting.

## Why the simulator always shows its windows

Gazebo, RViz and Mission Planner are opened visibly on purpose: a person watching a run notices what a log line does not
(a drone flying into a tree line, the camera pointing wrong). `--headless` exists only for tests and servers with no screen.

## What was removed on purpose

| Removed | Why |
|---|---|
| The flat `suas_field` world, its generator `build_suas_field.py`, its creek model, its missions and tests | replaced by the real-terrain world; keeping it let PCs run "the old way" |
| `--no-windows-gcs`, profile field `network.windows_gcs` | Mission Planner always opens and is always fed |
| The leaf-camouflage tree texture | trees now use the photo's own crown pixels |

See also: [ARCHITECTURE](ARCHITECTURE.md) (how the parts talk), [WORLDS](WORLDS.md) (the real-terrain world in detail),
[MISSION_PLANNER](MISSION_PLANNER.md) (flying with it), [DOCKER](DOCKER.md) (containers).
