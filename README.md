<!-- PURPOSE: introduce the team development environment, point to the documentation index (docs/README.md) and direct simulator users to setup, architecture, model integration and operator guides. -->
# ASTRA ODLC — Dev Environment (ROS 2 Humble, Python + C++)

**All documentation: [`docs/README.md`](docs/README.md)** lists every doc in this repo, where it is, what it is
for and who it is for, plus the rules for keeping the docs up to date (when you add, move or rename a doc, or
change a command or port, update the doc and that index, then run `python3 simulation/scripts/check_docs.py`).

For the Gazebo digital twin, start with [the simulator run/edit guide](simulation/README.md).
It links to [architecture](docs/simulation/ARCHITECTURE.md),
[recognition model testing and plug-ins](docs/simulation/PLUGINS.md),
[SITL connections](docs/simulation/ARDUPILOT.md), and the remaining operator guides.
Mission Planner with the ASTRA plugin is the ground station: [MISSION_PLANNER](docs/simulation/MISSION_PLANNER.md).
Assistants continuing development should read [AGENTS.md](AGENTS.md), then [the handoff](docs/HANDOFF.md).

The dev container is built from our own `docker/Dockerfile`. It mirrors the
Jetson (Ubuntu 22.04, Python 3.10, ROS 2 Humble, same Python package versions)
and runs natively on Mac (Apple Silicon or Intel), Windows and Linux.

| Target | Who | What's in it |
|---|---|---|
| `dev` (default) | Everyone | ROS 2 Humble, MAVROS, C++ toolchain, ONNX Runtime, Python ML stack (CPU) |
| `sitl` | Anyone testing mission logic | `dev` + ArduPilot SITL (headless, no Gazebo) |
| `sim` | Simulator workstation or WSL 2 (x86 only) | `sitl` + Gazebo Harmonic + ardupilot_gazebo |

All versions are `ARG`s at the top of `docker/Dockerfile` — change one and rebuild.

## Prerequisites

1. **Docker Desktop** (Mac/Windows) or Docker Engine (Linux):
   [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)
   *No Docker account or login is needed* (Docker Desktop's sign-in screen can be skipped, and every image and
   download is public). The one case where `docker login` helps is Docker Hub's anonymous pull limit on a shared
   network, such as a university Wi-Fi. Details: [Do I need a login?](docs/simulation/DOCKER.md#no-login)
   **If Docker says "pull access denied ... repository does not exist or may require 'docker login'"**, the image
   just has not been built on your PC yet (our images are built locally, not downloaded). Build it once with
   `docker build -t astra-sitl --target sitl docker/`, or `bash simulation/docker/build_images.sh` for all of them.
   In VS Code, "Reopen in Container" offers two dev containers: **ASTRA ODLC dev** works straight away;
   **ASTRA simulator** needs `build_images.sh sim` first.
   **Any other Docker error? Run the doctor first.** It checks everything a first run trips over (Docker not
   running, the wrong `docker` program, missing rights, Windows line endings from a Windows clone, disk, clock,
   network, graphics, old images) and prints the exact command for each problem:
   `bash simulation/docker/check_host.sh` in the Ubuntu terminal, and on Windows
   `powershell -ExecutionPolicy Bypass -File simulation\docker\check_host.ps1`.
   Every message and its fix: [Docker troubleshooting](docs/simulation/DOCKER.md#troubleshooting).
   Tip: clone the repo **inside Ubuntu** (WSL), not with Git for Windows, so scripts keep Linux line endings.
2. **VS Code** + the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
3. **GUI windows (optional, only for `image_view` etc.)**
   - **Mac:** install [XQuartz](https://www.xquartz.org/). In XQuartz → Settings → Security,
     tick "Allow connections from network clients", restart XQuartz, then run `xhost +localhost`.
   - **Windows:** install [XLaunch (VcXsrv)](https://sourceforge.net/p/vcxsrv/wiki/VcXsrv%20%26%20Win10/).
     Launch it before opening the container, change the display number from `-1` to `0`,
     and press **Next** through the rest.

## Open the dev container

```bash
git clone https://github.com/CPP-Aerial-Vision-Analysis-System/ODLC_Machine_Inferencing_System_2025-2026.git
```

Open the folder in VS Code → **Reopen in Container** (or Ctrl/Cmd+Shift+P →
"Dev Containers: Reopen in Container"). The first build takes ~10 minutes.

## Build and run the workspace

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash     # load ROS 2 first (the dev container does this for you)
colcon build --symlink-install
source install/setup.bash
```

**Error "Could not find a package configuration file provided by `ament_cmake`"?** ROS 2 is not loaded in the
terminal you are building in. Run `bash simulation/setup/check_ros.sh`: it checks whether you are on Ubuntu 22.04
(or in the dev container), whether ROS 2 Humble is installed, and whether it is loaded in this terminal, and it
prints the one fix. ROS 2 Humble cannot be built in plain Windows PowerShell or on macOS: use the Docker dev
container or an Ubuntu 22.04 WSL terminal. (The simulator's `bash simulation/setup/03_build.sh` loads ROS by itself.)

Check the container matches the Jetson (run the same script on both and compare):

```bash
./scripts/check_versions.sh
```

## Simulation (SITL)

Build the `sitl` image once, from the repo root on your host:

```bash
docker build -t astra-sitl --target sitl docker/
docker run -it --rm --shm-size=1g -v "$PWD":/ODLC_Machine_Inferencing_System_2025-2026 astra-sitl
```

Inside it, terminal 1 — start the simulated drone (MAVProxy forwards it to MAVROS on 14552):

```bash
sim_vehicle.py -v ArduCopter --no-rebuild --out=udp:127.0.0.1:14552
```

Terminal 2 — run the mission stack against it:

```bash
./launcher.sh sim
```

For the complete Gazebo mission, camera emulator and scoring, use the native setup below or follow
[the Docker guide](docs/simulation/DOCKER.md). The container integration is still being validated.

## Simulation (Gazebo): the real SUAS field, flown from Mission Planner

A full digital twin of the competition site: our Iris + SIYI ZR10 camera in Gazebo Harmonic over the **real terrain**
(USGS lidar ground, NAIP aerial photo, the real trees and buildings), flown by ArduCopter SITL 4.6.3 (the Pixhawk's
firmware) with **the same team flight stack** (`./launcher.sh sim`) over MAVROS. **Mission Planner on Windows opens
every time** (there is no option to skip it), shows the drone, the mission and the camera picture, and a score report
is written at the end. *Why each tool was chosen:* [docs/simulation/WHY_THESE_CHOICES.md](docs/simulation/WHY_THESE_CHOICES.md).
Everything else about the simulator: [`simulation/README.md`](simulation/README.md). The index of every doc: [docs/README.md](docs/README.md).

> **What changed (2026-11-07):** the old flat `suas_field` world is **deleted** (use `suas_field_real`; the small `runway`
> test world stays); **Mission Planner always opens** (`--no-windows-gcs` and the profile field `network.windows_gcs` are gone);
> the real-terrain world exists end to end, with an optional AI-sharper ground. If an old note of yours says
> `demo.sh suas_field`, use `demo.sh suas_field_real`.

### What you need (the reference PC)

Windows 11 + WSL2 with the `Ubuntu-22.04` distro, an NVIDIA GPU with a current Windows driver (reference: RTX 4060 Laptop,
8 GB), about 25 GB free disk, internet for the first build, and **Mission Planner installed on Windows** (see
[MISSION_PLANNER.md](docs/simulation/MISSION_PLANNER.md) for the install and the ASTRA plugin). Details and the reasons:
[WHY_THESE_CHOICES.md](docs/simulation/WHY_THESE_CHOICES.md#the-reference-pc-what-the-same-result-means).

### Step 1 - one-time setup (about 30-60 minutes, mostly downloads)

Clone the repo on the Windows disk, then in an **Ubuntu-22.04** terminal, from the repo folder (`/mnt/d/...`):

```bash
sudo bash simulation/setup/01_install_humble.sh        # ROS 2 Humble, Gazebo Harmonic, tools
bash simulation/setup/02_setup_ardupilot.sh            # ArduCopter SITL 4.6.3
bash simulation/setup/03_build.sh --workers 4          # copies the repo to ~/ODLC_Machine_Inferencing_System_2025-2026,
                                                       # builds it, downloads the Gazebo models and the detector weights,
                                                       # and MAKES THE REAL-TERRAIN MODELS (about 1 minute)
```

Every later change: edit on Windows, run `03_build.sh` again (it overwrites the Linux copy; never edit the copy).
Optional sharper ground (needs the GPU; about 5 minutes once):

```bash
bash simulation/setup/upscale_env.sh                                              # the upscaler, in its own venv
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/simulation/sim_worlds
~/.astra_sim/upscale_venv/bin/python scripts/build_real_field.py upscale          # about 25-45 s
python3 scripts/build_real_field.py assets --install                              # paint the ground from it (about 2 min)
```

### Step 2 - fly it (automatic search of Search Boundary 1)

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026
source simulation/setup/sim_env.sh
bash simulation/scripts/demo.sh suas_field_real
```

What you will see, in order (reference PC): **Mission Planner opens**; Gazebo and RViz open; "ready to arm" after about 80 s;
the drone arms, takes off to **58.7 m above the pad** (about 47-56 m above the ground), flies 3 lanes at 8.5 m/s taking a photo
every second (about 110 photos), the detector finds the targets, the drone flies to them, drops, returns and lands
(about **7.5 minutes** in the air). Every phase prints in the terminal; the run folder `~/.astra_sim/runs/<date>_suas_field_real/`
holds `score_report.md`, the photos with detection boxes, CSV logs and the mission map. In Mission Planner the drone, the lanes
and the status texts appear, and the ASTRA tab shows the camera.

| Reference result (2026-11-07, real detector `new_od` + yolo26m, seed 2026) | Value |
|---|---|
| Flight | landed, 7.4 min, 110 photos |
| Hiker (Search Boundary 1) | found, confidence 0.92; payload landed about 16 m from it |
| Tent | the pick can be a false alarm on bright bare ground (first flight: 222 m error); expect tent precision to be low (see [WORLDS.md](docs/simulation/WORLDS.md#evidence-2026-11-07)) |
| Search Boundary 2 | not flown by the contract mission (use `suas_field_real_sb2_survey.waypoints`) |

Exact numbers vary a little between runs (timing, detector); the structure above must be the same.

### Step 3 - fly it yourself from Mission Planner (the competition procedure)

```bash
bash simulation/scripts/demo.sh suas_field_real --manual        # or: simulation\windows\Start for Mission Planner.bat suas_field_real
```

Mission Planner opens by itself. In it: (1) top right choose **UDP**, **CONNECT**, port **14550**; (2) **PLAN → Load WP file** →
`ros2_ws/src/simulation/sim_worlds/missions/suas_field_real_survey.waypoints` (Search Boundary 1) or
`suas_field_real_sb2_survey.waypoints` (Search Boundary 2) → **Write**; (3) arm, set **AUTO**. The **ASTRA** tab shows the
drone camera. The team code then handles detections, drops and RTL. Full guide and troubleshooting:
[MISSION_PLANNER.md](docs/simulation/MISSION_PLANNER.md).

### Step 4 - check that your PC matches ours

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/simulation/sim_worlds
python3 scripts/build_real_field.py check                    # offline: expect "all checks passed" (about 25 PASS lines)
ros2 run sim_worlds check_world.py suas_field_real --seconds 20   # Gazebo: expect RESULT: PASS, real-time factor about 1.0, 0 [Err]
python3 -m pytest -q -p no:cacheprovider test                # the package tests
```

If Mission Planner's map and the camera picture seem to disagree: the terrain is placed within 1-2 m of the real
site (measured), the usual cause is the 0.3-1.7 s delay of the position the map and the photo tags use, plus the map being
another imagery date; see [WORLDS.md](docs/simulation/WORLDS.md#why-the-drone-icon-in-mission-planner-can-seem-to-be-somewhere-else-than-the-camera-picture).

### The small test world and stopping

```bash
bash simulation/scripts/demo.sh runway          # the tiny quick test field (about 7 minutes); Mission Planner opens here too
bash simulation/scripts/stop_sim.sh             # stop everything
```

The simulator lives in `simulation/` (setup and scripts) and `ros2_ws/src/simulation/` (packages
`sim_bringup`, `sim_models`, `sim_worlds`, `siyi_emulator`, `sim_demo`, `sim_plugins`).
`launcher.sh` accepts `TRACKER_CMD` from the environment (default unchanged). Team-code fixes and their
remaining real-aircraft validation are described in [WHAT_CHANGED](docs/simulation/WHAT_CHANGED.md).


# How to wipe out all the docker images (mac)

* osascript -e 'quit app "Docker Desktop"'

### Paste all this

* docker stop $(docker ps -aq) 2>/dev/null || true
* docker rm -f $(docker ps -aq) 2>/dev/null || true
* docker rmi -f $(docker images -aq) 2>/dev/null || true
* docker volume rm $(docker volume ls -q) 2>/dev/null || true
* docker network rm $(docker network ls -q) 2>/dev/null || true

### Vhen go nuklear

* docker system prune -a --volumes -f

### Verify

* docker ps -a
* docker images
* docker volume ls
* docker network ls

### Reset docker and clean leftovers

* docker builder prune -a -f
* docker buildx prune -a -f

