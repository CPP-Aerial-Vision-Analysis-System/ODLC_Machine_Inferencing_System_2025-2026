<!-- PURPOSE: introduce the team development environment and direct simulator users to setup, architecture, model integration and operator guides. -->
# ASTRA ODLC — Dev Environment (ROS 2 Humble, Python + C++)

For the Gazebo digital twin, start with [the simulator run/edit guide](simulation/README.md).
It links to [architecture](docs/simulation/ARCHITECTURE.md),
[recognition model testing and plug-ins](docs/simulation/PLUGINS.md),
[SITL connections](docs/simulation/ARDUPILOT.md), and the remaining operator guides.
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
colcon build --symlink-install
source install/setup.bash
```

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

## Simulation (Gazebo)

A full digital twin: our Iris + SIYI camera in Gazebo Harmonic, flown by ArduCopter SITL 4.6.3 (the
Pixhawk's firmware), with **the same team flight stack** (`./launcher.sh sim`) talking to it through MAVROS
and to a SIYI camera emulator at the real camera address. You watch the camera's point of view while the
drone searches for the hiker and the tent, detects them, drops the payloads and flies home; a score report
is written at the end. Everything is in [`simulation/README.md`](simulation/README.md).

```bash
# one time, from the checkout inside Ubuntu 22.04
sudo bash simulation/setup/01_install_humble.sh
bash simulation/setup/02_setup_ardupilot.sh
bash simulation/setup/03_build.sh --workers 4
# Only Windows-mounted checkouts create the mirror; native Linux stays in its checkout.
# After a Windows-mounted build: cd ~/ODLC_Machine_Inferencing_System_2025-2026
source simulation/setup/sim_env.sh
bash simulation/scripts/demo.sh runway
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

