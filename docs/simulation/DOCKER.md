<!--
DOCKER.md -- PURPOSE: explain the ASTRA simulator containers, give a reproducible setup for another Windows
or Linux PC, show how to build or transfer images and run the current scripts, and separate checked host-side
behavior from container flights that still need validation.
-->

# Run the simulator in Docker

## Run it on another PC

Status on 2026-10-24: **container integration is work in progress**. Compose configuration and host helper
tests have been checked; image builds, a containerized flight, GUI behavior and the clean-copy procedure below
still need Task 6 validation. These commands describe the current scripts, not a completed acceptance run.

Use an **x86_64/amd64 PC**. The `sim` image is not an ARM/Jetson image. It contains Ubuntu 22.04, ROS 2 Humble,
Gazebo Harmonic, ArduCopter SITL and the supporting tools. The host does not need ROS installed for the Docker
route. Allow at least the build helper's recommended **40 GB free** for images and temporary build files;
the script estimates about 25 GB of images. Keep only one simulator or image build running at a time.

### A1. Windows: prepare Ubuntu 22.04 in WSL 2

In an administrator PowerShell terminal:

```powershell
wsl --install -d Ubuntu-22.04
wsl --update
wsl --list --verbose
```

Finish Ubuntu's first-run account setup. `Ubuntu-22.04` should show version `2`; if needed, use
`wsl --set-version Ubuntu-22.04 2`. Then open its terminal with `wsl -d Ubuntu-22.04`.
See [Microsoft's WSL commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands).

Run the Linux blocks below **inside that Ubuntu terminal**. When automating from PowerShell, save Linux
commands in an LF `.sh` file and invoke `wsl -d Ubuntu-22.04 -- bash <script-path-in-WSL>`; avoid inline shell
quoting. Keep the checkout in the Linux home directory, where builds are fast and paths have no spaces.

### A2. Clone the branch

On Windows use the Ubuntu terminal above; on a native Linux PC use an Ubuntu 22.04 terminal:

```bash
sudo apt-get update
sudo apt-get install -y git ca-certificates rsync python3-venv
git clone -b jarrod-gazebo-sim https://github.com/CPP-Aerial-Vision-Analysis-System/ODLC_Machine_Inferencing_System_2025-2026.git
cd ODLC_Machine_Inferencing_System_2025-2026
```

The working tree described by this guide may contain changes newer than the published branch. Record
`git rev-parse HEAD` when validating a fresh clone. Images contain dependencies; **the source checkout is
mounted at run time**, so copying images alone does not copy the drone software.

### A3. Choose Docker Engine or Docker Desktop

The supported setup script installs **Docker Engine inside Ubuntu**:

```bash
ps -p 1 -o comm=
sudo bash simulation/docker/install_docker_engine.sh
```

The first command must print `systemd` for this installer. If WSL prints something else, edit
`/etc/wsl.conf` with `sudoedit` and add or update the `[boot]` setting `systemd=true`, preserving its other
settings. Restart that distro from PowerShell with `wsl --terminate Ubuntu-22.04`, reopen Ubuntu, and check
again. The installer stops with this explanation rather than running without systemd.

It installs Engine, Compose and Buildx, sets bounded Docker logs, adds your account to the Docker group and
optionally installs NVIDIA Container Toolkit when a compatible card is visible. On native Linux, log out
and back in so the new group applies. On Windows, close Ubuntu and run `wsl --terminate Ubuntu-22.04` from
PowerShell, then reopen Ubuntu and return to the checkout.

For a CPU-only PC, the explicit installation choice is:

```bash
sudo bash simulation/docker/install_docker_engine.sh --no-nvidia
```

If choosing **Docker Desktop** instead, skip this Engine installer and follow [A6](#a6-docker-desktop-on-windows).
Keep one Docker installation responsible for this distro. Check the selected installation from Ubuntu:

```bash
command -v docker
docker info
docker compose version
docker run --rm hello-world
```

The helper requires Compose **2.20 or newer**. For NVIDIA, install a working host GPU driver first, then check:

```bash
docker run --rm --gpus all ubuntu:22.04 nvidia-smi
```

### A4. Build images, or load a teammate's images

From the checkout, choose one build:

```bash
# NVIDIA or automatic detection of the available card:
bash simulation/docker/build_images.sh --jobs 4 --torch auto

# CPU detector, no CUDA requirement:
bash simulation/docker/build_images.sh --jobs 4 --torch cpu
```

Run one choice, not both. The helper builds `astra-dev:latest`, `astra-sitl:latest` and `astra-sim:latest`
sequentially and checks each image. Initial duration and disk use are estimates until measured on the new PC.
Logs are in `~/.astra_docker/logs/build_<target>_<time>.log`. To check existing images without rebuilding:

```bash
bash simulation/docker/build_images.sh --check
```

For transferred images use the save/load procedure below instead of building. Build locally when the source
PC's user ID differs: the images run as their built-in user, and bind-mounted files and Linux X11 access
depend on that ID. This is a software stand-in for the Jetson; hardware-specific TensorRT engines are not
portable to this PC.

### A5. Start, recognize success, stop

First print the selected settings and validate the Compose configuration without starting containers:

```bash
bash simulation/docker/compose_up.sh runway --headless --dry-run
```

Then choose a flight:

```bash
# Team YOLO detector, GPU when available:
bash simulation/docker/compose_up.sh runway --headless

# Team YOLO detector on CPU:
bash simulation/docker/compose_up.sh runway --headless --no-gpu

# Faster integration smoke check using the labeled answer-key stand-in:
bash simulation/docker/compose_up.sh runway --headless --no-gpu --detector truth
```

The `truth` option proves integration with a stand-in detector; it does not test YOLO. CPU YOLO may build a
photo backlog. To show windows, omit `--headless`; WSL needs WSLg, native Linux needs a working X11/XWayland
desktop. `--no-gpu` disables NVIDIA requests for CUDA; it can still use WSLg or `/dev/dri` for scene rendering.

A successful run should report autopilot readiness, mission upload, team-stack readiness, AUTO flight, both
payload releases, RTL and landing. Check `score_report.md` in `~/.astra_docker/runs/<time>_runway/` for **2/2
targets** and both drop errors, plus the photos and process logs. A zero exit code alone is insufficient.
These success criteria remain to be demonstrated in Docker.

```bash
bash simulation/docker/compose_down.sh --restore-camera-ip --solo
```

Use this stop command after checks, including interrupted runs. `--solo` also stops the two documented
single-container names; results and images are retained. `--wipe-builds` and `--wipe-fuel` are recovery
options that delete cached build/model volumes, not routine shutdown options.

### A6. Docker Desktop on Windows

Use the [official Docker Desktop WSL instructions](https://docs.docker.com/desktop/features/wsl/): select
the WSL 2 backend and enable integration for **Ubuntu-22.04** under Resources / WSL Integration. Docker advises
removing distro-installed Engine/CLI before switching to Desktop. Do not run this repo's Engine installer
on that same setup. Verify `docker info` and `docker compose version` from Ubuntu before A4.

This backend is a portability target, **not a validated configuration yet**. The helpers reject a `docker`
command located under `/mnt/` to catch the Windows CLI; a usable Linux CLI must be visible in the integrated
distro. WSLg mounts and CUDA need their own checks even when `hello-world` works. Follow Docker's settings
for image storage location; Engine-in-WSL stores images in that distro's filesystem instead.

## Copy images without rebuilding

`export_images.sh` and `import_images.sh` are planned for Task 6 and **do not exist yet**. The current manual
equivalent uses standard Docker commands. On the source PC, from a folder with enough free space:

```bash
set -o pipefail
docker image inspect astra-dev:latest astra-sitl:latest astra-sim:latest > astra-image-metadata.json
docker save astra-dev:latest astra-sitl:latest astra-sim:latest | gzip -1 > astra-images.tar.gz
sha256sum astra-images.tar.gz > astra-images.tar.gz.sha256
```

Transfer the archive, checksum and metadata to the destination PC. In that folder:

```bash
set -o pipefail
sha256sum -c astra-images.tar.gz.sha256
gzip -dc astra-images.tar.gz | docker load
docker image ls
```

The `pipefail` setting reports a failed save or decompression even if the pipeline's last command succeeds.
Check the archive checksum before loading. The metadata records image labels, including the
source commit and PyTorch flavor; retain it with the archive. Run `build_images.sh --check` from the checkout
after loading, then A5. Image export/import, differing user IDs and a save/load round trip remain untested.

## What runs where

| Component | Container/address | Entry point |
|---|---|---|
| Gazebo, SITL, MAVProxy, SIYI emulator, score/panel | `sim`, `192.168.144.25` | `simulation/docker/container_sim.sh` |
| MAVROS and the team's ROS nodes | `jetson`, `192.168.144.100` | `container_jetson.sh` → `./launcher.sh sim` |
| Source code and team photo files | Same checkout mounted in both | `/ODLC_Machine_Inferencing_System_2025-2026` |
| Dependencies | Images `astra-sim` and `astra-dev` | `docker/Dockerfile`, targets `sim` and `dev` |
| ROS builds | Separate named Docker volumes | `astra_sim_ws_*`, `astra_jetson_ws_*` |
| Run reports, models, state | Shared host `~/.astra_docker` | Mounted as `/home/ubuntu/.astra_sim` |
| Fuel model cache | Host cache or `astra_gz_fuel` volume | `/home/ubuntu/.gz/fuel` |

The fixed `astra_net` subnet reproduces the camera network. MAVProxy sends MAVLink to the Jetson stand-in's
UDP 14552. Both containers use matching `ROS_DOMAIN_ID`, `ROS_LOCALHOST_ONLY=0` and host IPC. The native camera
alias is removed before Compose starts because it conflicts with the `sim` container's address; shutdown
can restore it. See [architecture](ARCHITECTURE.md) and [ArduPilot configuration](ARDUPILOT.md).

Compose chooses additive files: base `compose.sim.yml` is headless; `compose.wslg.yml` adds WSLg/DirectX;
`compose.linux.yml` adds X11; `compose.dri.yml` adds Linux render devices/groups; `compose.gpu.yml` requests
NVIDIA CUDA/OpenGL. Do not copy host device paths from one platform's override to another.

The new shared-state helpers describe mission-ready/restart tokens, but the current container entry points
do not yet implement that complete handover. **START OVER in the split-container setup remains pending**;
do not infer support from the host script's header. Profiles are described in [PLUGINS.md](PLUGINS.md);
the current Compose wrapper has its own detector/camera options and does not accept `--profile`.

## Other workflows and source edits

```bash
# Everything in one simulator container:
bash simulation/docker/run_sim_container.sh --demo runway --headless

# Jetson stand-in connected to a native simulator:
bash simulation/docker/run_jetson_container.sh --net host --launcher

# Two containers, but let Mission Planner/panel control the flight:
bash simulation/docker/compose_up.sh runway --manual
```

Run one workflow at a time. `run_sim_container.sh --no-gpu --demo runway --headless` applies the CPU choice
to the wrapper; arguments after `--demo` go to `demo.sh`. A Windows checkout on `/mnt/<drive>` uses the Linux
mirror by default. Refresh it with `compose_up.sh --sync-only` after editing; stale mirrors otherwise hide
your changes. Native ROS builds still use `simulation/setup/03_build.sh`; the containers build their own
volumes via `_container_common.sh`. Never build with colcon inside a Windows drive checkout.

Changing ROS/package source usually requires container workspace rebuild only. `ASTRA_BUILD=always` forces
that rebuild; `--rebuild` requests a clean container workspace. Changes to Dockerfile or dependency pins
require image rebuilds. Keep `numpy < 2` for ROS `cv_bridge` and preserve hardware defaults in team packages.

## Troubleshooting

| Symptom | Check/action |
|---|---|
| Docker socket permission denied | Reopen the login session after installation; check `id` for `docker` membership. |
| Engine is unreachable | `docker info`; Engine setup: `sudo systemctl start docker`; Desktop setup: start Desktop. |
| Missing image | Build the named target or load all required image tags; no transfer helper exists yet. |
| Windows edit has no effect | Check reported mounted repo; refresh the Linux mirror with `--sync-only`. |
| Fixed subnet already used | Inspect `ip -4 route` and `docker network ls`; resolve the conflicting network before running. |
| No windows | Verify `$DISPLAY`, X11/WSLg sockets and the selected overrides; first try `--headless`. |
| CUDA unavailable | Check the NVIDIA runtime and the image's PyTorch flavor; `--no-gpu` uses the CPU detector. |
| Slow camera/rendering | Check the render device/driver; headless cameras still render a 3D scene. |
| Team nodes invisible across containers | Match ROS domain, inspect `docker logs astra-jetson`, check DDS networking/IPC. |
| Files owned by another user | Compare host `id -u` with image user; rebuild for this user's ID. |
| Disk space low | Inspect `docker system df`; retain run evidence and avoid indiscriminate volume deletion. |

For ongoing logs use `docker logs -f astra-sim` and `docker logs -f astra-jetson`. After a foreground run the
wrapper copies container logs into its run folder. Broader checks live in [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

## Maintainer checks still required

Task 6 must build/check the images, fly one-container and split-container demos, verify WSLg briefly, run a
CPU fallback, and follow only this guide from a clean source copy. It must also implement image transfer
helpers and test their checksum/save/load round trip. Record source/image versions and score reports, then
update [HANDOFF.md](../HANDOFF.md), `AGENTS.md` and this status. Do not mark Docker complete after parsing YAML.

Host regressions, which do not start Docker, run with:

```bash
python3 -m pytest -q -p no:cacheprovider simulation/docker/test
```
