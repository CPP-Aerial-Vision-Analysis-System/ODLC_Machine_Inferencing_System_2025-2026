<!--
DOCKER.md -- PURPOSE: explain the ASTRA simulator containers, give a reproducible setup for another Windows
or Linux PC, show how to build or transfer images and run the current scripts, list every known error message
with its fix (Troubleshooting), and separate checked host-side behavior from container flights that still
need validation.
-->

# Run the simulator in Docker

## Step 0: let the doctor check your computer first

Most first-day Docker problems are not in our code: Docker is not running, the wrong `docker` program answers,
your user may not use Docker yet, a Windows clone gave the scripts Windows line endings, the disk is full, the
clock is wrong, a school network blocks a download ... Two small scripts look for all of these, change nothing,
and print **exactly what to type** for every problem they find:

1. **On Windows**, in PowerShell, in the repository folder (checks Windows, WSL, Docker Desktop, git, memory):

   ```powershell
   powershell -ExecutionPolicy Bypass -File simulation\docker\check_host.ps1
   ```

2. **Inside Ubuntu** (the WSL terminal on Windows, or a Linux PC), in the repository folder:

   ```bash
   bash simulation/docker/check_host.sh             # about 30 seconds (up to a minute on a Windows drive)
   bash simulation/docker/check_host.sh --deep      # also starts small test containers (after the images exist)
   ```

Each line starts with `[ OK ]` (fine), `[ .. ]` (good to know), `[WARN]` (works, but read it) or `[FAIL]` (fix
this first). Under every `[WARN]` and `[FAIL]` an arrow `->` shows the command to type. Fix the `[FAIL]` lines,
run the check again, and go on when it says "Nothing blocks ...". The [Troubleshooting](#troubleshooting) table
below lists every message with its meaning.

## Run it on another PC

Status on 2026-10-24: **container integration is work in progress**. Compose configuration and host helper
tests have been checked; image builds, a containerized flight, GUI behavior and the clean-copy procedure below
still need Task 6 validation. These commands describe the current scripts, not a completed acceptance run.

Use an **x86_64/amd64 PC**. The `sim` image is not an ARM/Jetson image. It contains Ubuntu 22.04, ROS 2 Humble,
Gazebo Harmonic, ArduCopter SITL and the supporting tools. The host does not need ROS installed for the Docker
route. Allow at least the build helper's recommended **40 GB free** for images and temporary build files;
the script estimates about 25 GB of images. Keep only one simulator or image build running at a time.

### Do I need a login or an account? No (with two small exceptions)

<a id="no-login"></a>

**No Docker login, no GitHub login and no NVIDIA account is needed to build or run the simulator.** This was
checked by reading the Dockerfile, the compose files and the scripts on 2026-10-24; it still has to be confirmed
by a clean-PC build (see "Maintainer checks still required" below). The base image is Docker's public
`ros:humble-ros-base-jammy`, nothing is pulled from a private registry, and the repository contains no
tokens, passwords or secrets.

| Step | Login needed? | What to know |
|---|---|---|
| Clone this repository | **No** | It is public. Only *pushing* needs a GitHub account with write access. |
| Pull the base image `ros:humble-ros-base-jammy` | **No** | It is a public Docker Hub image. See the rate limit below. |
| Everything else the build downloads | **No** | Ubuntu and ROS packages (`apt`), Gazebo packages (`packages.osrfoundation.org`), GitHub clones and releases (mavlink-router, ArduPilot, ardupilot_gazebo, onnxruntime), PyTorch wheels (`download.pytorch.org`) and one key from `keyserver.ubuntu.com`. All public. |
| Docker Engine inside WSL or Linux | **No** | `install_docker_engine.sh` never asks for an account. |
| Docker Desktop (Windows or Mac) | **No**, by default | The sign-in screen can be skipped. It becomes mandatory only if your school or company enforces it (see [Docker's page](https://docs.docker.com/desktop/enterprise/enforce-sign-in/)). The Engine route in A3 avoids Desktop altogether. |
| NVIDIA GPU in containers | **No NVIDIA account** | The NVIDIA Container Toolkit comes from a public apt repository. You only need a recent NVIDIA driver on the host. We do not use NVIDIA's NGC registry (`nvcr.io`). |
| Detector weights `yolo26m.pt` (about 40 MB) | **No** | Downloaded automatically from Ultralytics' public release the first time the real detector runs. Offline? Use `--detector truth` (the answer-key detector) or copy the file to `~/.astra_sim/models/yolo26m.pt`. |
| Gazebo 3D models (about 110 MB) | **No** | Downloaded once from the public Gazebo Fuel library, then kept in a volume. |
| Inside the running containers | **No password** | There is no SSH or VNC server. Open a shell with `docker exec -it astra-sim bash` (or `astra-jetson`, or `astra-sim-solo` for the single-container route). You are the user `ubuntu`, who has passwordless `sudo`. |

**The two exceptions, when `docker login` is useful:**

1. **Docker Hub's anonymous pull limit.** Without logging in, Docker Hub allows **100 pulls per 6 hours per IP
   address** (per IPv6 /64 block); a free Docker account allows 200 per 6 hours
   ([Docker's limits page](https://docs.docker.com/docker-hub/usage/pulls/)). Our build pulls only one base
   image, so you will only meet the limit on a **shared network** such as a university or lab Wi-Fi where many
   people appear as one address. The error says `toomanyrequests: You have reached your unauthenticated pull
   rate limit`. Fix: create a free account at hub.docker.com, then run `docker login` (use an access token, not
   your password), or wait 6 hours, or use a teammate's saved images (see "Copy images without rebuilding").
2. **Publishing images to a registry (optional, not needed to run anything).** Pushing to GitHub's
   `ghcr.io` needs a GitHub personal access token with the `write:packages` permission:
   `echo "$TOKEN" | docker login ghcr.io -u <your-github-name> --password-stdin`. Never put a token in a file
   that is committed to git: this repository is public.

#### "Login required" or "repository does not exist": almost always NOT a login problem

The error `pull access denied for astra-sim, repository does not exist or may require 'docker login'` (or
`unauthorized: authentication required`) means **the image has not been built on this PC yet**. The images
`astra-dev`, `astra-sitl` and `astra-sim` exist only on a computer where somebody built them (step A4) or loaded
them (`docker load`). They are **not** published on Docker Hub, so when anything asks Docker for one that is
missing, Docker looks for it on Docker Hub, finds nothing and words the failure as a possible login problem.

You can meet it in three ways. All have the same cure, **build the images once** (about 1 hour, about 25 GB):

```bash
bash simulation/docker/build_images.sh          # all three, one after the other   (or: ... sim   for just the simulator)
```

| Where you saw it | Why | What happens now |
|---|---|---|
| VS Code "Reopen in Container" and you picked **ASTRA simulator (Gazebo + SITL)** | that dev container uses the prebuilt image `astra-sim:latest` | it now stops first with a plain message that says to build the image (`simulation/docker/require_image.sh`). The other choice, **ASTRA ODLC dev**, builds itself the first time and needs no prior step. |
| `docker run astra-sim` / `astra-sitl` / `astra-dev` | the image does not exist yet | build it first (the root README's `docker build -t astra-sitl --target sitl docker/` does exactly that) |
| `docker compose -f docker/compose.sim.yml up` typed by hand | older versions of these files first tried to download the images, printed `unauthorized: authentication required`, then built them anyway | `pull_policy: never` is set now, so Compose goes straight to building. The helper `simulation/docker/compose_up.sh` checks for the images first and tells you how to build them. |

**If the failing name is `ros:humble-ros-base-jammy` (the public base image) instead,** you have a different
problem: a stale or broken Docker login. Docker Hub rejects expired credentials even for public images. Run
`docker logout`, then try again; also look for a `credsStore` line in `~/.docker/config.json` that names a
program which is not installed (for example `desktop.exe` in WSL without Docker Desktop) and delete that line.
If the message says `toomanyrequests`, it is the anonymous rate limit described above.

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
GIT_LFS_SKIP_SMUDGE=1 git clone -b jarrod-gazebo-sim https://github.com/CPP-Aerial-Vision-Analysis-System/ODLC_Machine_Inferencing_System_2025-2026.git
cd ODLC_Machine_Inferencing_System_2025-2026
```

Why clone **inside Ubuntu** and not with Git on Windows: Git for Windows is installed with `core.autocrlf=true`,
which writes Windows line endings (CRLF) into every file. Bash and Linux then fail with `$'\r': command not found`
or `/usr/bin/env: 'python3\r': No such file or directory` (for example when `main_controller.py` starts). The
repository's `.gitattributes` now forces Linux line endings for these files even on Windows, but a clone made
before that, or one copied around, can still have them: `bash simulation/docker/check_host.sh` finds them and
`bash simulation/docker/check_host.sh --fix-line-endings` repairs them (your own edits are kept).

Why `GIT_LFS_SKIP_SMUDGE=1`: one folder we never use (`ros2_ws/src/ultralytics_ros/ros2bag/`, a 1.3 GB recorded
ROS bag) is stored with Git LFS. Where Git LFS is installed (Git for Windows has it), `git clone` would download
it, or stop with `Smudge error ... Clone succeeded, but checkout failed` when the server cannot give it. The
setting makes git skip it; nothing in the build or the simulator needs that file. If your clone already stopped
with that error, do **not** use the command git suggests (`git restore --source=HEAD :/`): it puts the files back
but leaves git's list of files empty, so your next commit would delete almost the whole repository. Use the
command in the [Troubleshooting](#troubleshooting) row "Smudge error" instead; afterwards `git status` shows nothing.

One file is the exception the doctor knows about: `ros2_ws/src/ultralytics_ros/script/tracker_node.py` was saved
in the repository itself with Windows line endings (not by your clone), so it would fail with `python3\r` on every
computer. Nothing uses it: the containers and `03_build.sh` skip that package (`--packages-ignore ultralytics_ros`),
so the doctor and `--fix-line-endings` leave it alone on purpose.

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
Logs are in `~/.astra_docker/logs/build_<target>_<time>.log`.

What the helper does for you (so you do not have to remember it):

- it insists on Docker's current builder, **BuildKit** (the `buildx` plugin). Without it, `docker build` quietly
  falls back to the old builder, and that one made the `sitl` and `sim` images run as `root` (checked on
  2026-10-31; the Dockerfile now also protects against it);
- it writes labels into each image: your user id, the PyTorch flavor, the repo commit and a fingerprint of
  `docker/Dockerfile` + `docker/requirements-*.txt`. `compose_up.sh`, the two `run_*_container.sh` scripts and
  `check_host.sh` compare that fingerprint with the repo and warn "older than docker/Dockerfile" when somebody
  changed those files since your build. Code changes never need an image rebuild (the code is mounted);
- it passes your proxy settings (`http_proxy`, `https_proxy`, `no_proxy` ...) into the build when they are set;
- it downloads a fresh ROS base image when yours is from before June 2025 (older copies still trust the ROS
  signing key that expired then, and the build would stop at its first step with `EXPKEYSIG`).

To check existing images without rebuilding:

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

First run the doctor (Step 0 at the top): `bash simulation/docker/check_host.sh` inside Ubuntu, and on Windows
`powershell -ExecutionPolicy Bypass -File simulation\docker\check_host.ps1`. It finds most rows of this table by
itself. Find your message in the left column (the exact words Docker, bash or git print), then do what the right
column says. "Ubuntu terminal" means the WSL window on Windows (`wsl -d Ubuntu-22.04`) or a terminal on a Linux PC.

**Getting the code**

| Message or symptom | What it means and what to do |
|---|---|
| `$'\r': command not found`, `bash\r: No such file or directory`, `/usr/bin/env: 'python3\r': No such file or directory`, or `syntax error near unexpected token $'do\r'` | The files have Windows line endings (CRLF), usually from `git clone` with Git for Windows. Repair them (keeps your edits): `bash simulation/docker/check_host.sh --fix-line-endings`, then `bash simulation/docker/compose_up.sh --sync-only` if your checkout is on a Windows drive. Better: clone inside Ubuntu (A2). The repo's `.gitattributes` keeps new clones LF. |
| `Smudge error`, `Clone succeeded, but checkout failed`, `This repository is over its data quota`, or the clone downloads 1.3 GB | An unused recorded ROS bag (`ros2_ws/src/ultralytics_ros/ros2bag/`) is kept in Git LFS. Clone with `GIT_LFS_SKIP_SMUDGE=1 git clone ...` (PowerShell: `$env:GIT_LFS_SKIP_SMUDGE=1; git clone ...`); nothing needs that file. To repair a clone that already failed, inside its folder: `GIT_LFS_SKIP_SMUDGE=1 git restore --source=HEAD --staged --worktree :/` (PowerShell: `$env:GIT_LFS_SKIP_SMUDGE=1; git restore --source=HEAD --staged --worktree :/`), then `git status` must show nothing. Do **not** use git's own hint `git restore --source=HEAD :/`: it leaves hundreds of files marked "deleted" (`D`), and a commit would delete them. |
| `fatal: detected dubious ownership in repository at '...'`, or the doctor says git "refuses to read" the folder | The folder belongs to another user (a checkout on a Windows drive opened from Ubuntu, or a copy made with `sudo`), and git protects you from it. Allow this one folder: `git config --global --add safe.directory "<the folder>"` (the doctor prints the exact line). |
| `/usr/bin/env: 'python3\r'` for `ros2_ws/src/ultralytics_ros/script/tracker_node.py` | That one file is stored WITH Windows line endings in the repository itself. Nothing builds it (`--packages-ignore ultralytics_ros`). Only if you need it: `sed -i 's/\r$//' ros2_ws/src/ultralytics_ros/script/tracker_node.py` and commit that. |
| `Filename too long` during `git clone` on Windows | The folder path is too long for Windows: `git config --global core.longpaths true`, or clone into a short folder (`C:\src`), or inside Ubuntu. |
| git says files are "modified" right after cloning, but `git diff` shows nothing | Old line-ending rules for a few older `.md` files; fixed by the current `.gitattributes`: `git pull`, then `git status` again. Do not commit such "changes". |
| The checkout is inside OneDrive (`C:\Users\<you>\OneDrive\...`) | OneDrive locks and "online-only" files break git and builds: clone inside Ubuntu or into a folder outside OneDrive. |

**Docker itself**

| Message or symptom | What it means and what to do |
|---|---|
| `The command 'docker' could not be found in this WSL 2 distro. We recommend to activate the WSL integration in Docker Desktop settings.` | The `docker` that answers is Docker Desktop's Windows helper. Either install Docker Engine inside Ubuntu (`sudo bash simulation/docker/install_docker_engine.sh`, A3) or turn on Docker Desktop -> Settings -> Resources -> WSL integration -> Ubuntu-22.04 -> Apply & restart (A6). Never both. |
| `docker: command not found` | Docker is not installed in this Ubuntu: `sudo bash simulation/docker/install_docker_engine.sh` (A3). |
| `Please enable the Virtual Machine Platform Windows feature and ensure virtualization is enabled in the BIOS` (error `0x80370102`), Docker Desktop's `Virtualization support not detected`, or `check_host.ps1` says "virtualization is switched OFF in the BIOS/UEFI" | The processor's virtualization switch is off, so WSL2 and Docker cannot start at all. Restart the PC, open the BIOS/UEFI setup (often F2, F10, Del or Esc right after switching on) and turn on "Intel VT-x" / "Intel Virtualization Technology" or "AMD-V" / "SVM Mode"; save and restart. Then, in an administrator PowerShell: `wsl --install --no-distribution` (switches on the Windows part) and restart once more. |
| `check_host.ps1` says "Docker Desktop uses its old Hyper-V engine, not WSL 2" | Docker Desktop runs in its own virtual machine instead of WSL 2: no WSL integration, no Linux windows for the containers. Docker Desktop -> Settings -> General -> tick **Use the WSL 2 based engine** -> Apply & restart. (Not needed if you use Docker Engine inside Ubuntu.) |
| The doctor says `DOCKER_HOST=... is set` or `docker context "..." is active`; or your images "are missing" although you built them; or `Cannot connect to the Docker daemon at tcp://...` | `docker` is talking to a different engine (another computer, Docker Desktop, a remote server) than the one in this Ubuntu. Back to the normal one: `unset DOCKER_HOST` (and delete the `export DOCKER_HOST=...` line from `~/.bashrc` if it is there), then `docker context use default`. |
| `Cannot connect to the Docker daemon at unix:///var/run/docker.sock. Is the docker daemon running?` | Docker Engine is installed but not running: `sudo systemctl start docker`. With Docker Desktop: start Docker Desktop and wait for "Engine running". If `ps -p 1 -o comm=` does not print `systemd`, turn systemd on (A3). |
| `error during connect: ... open //./pipe/dockerDesktopLinuxEngine: The system cannot find the file specified` (in PowerShell) | Docker Desktop is not running, and you typed the command in Windows. Run the ASTRA scripts in the Ubuntu terminal instead. |
| `permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock` | Your Linux user may not use Docker yet. Once: `sudo usermod -aG docker $USER`, then close every Ubuntu window and run `wsl --terminate Ubuntu-22.04` in PowerShell (Linux PC: log out and in). Do **not** use `sudo` with the ASTRA scripts: it moves the data folder to `/root`. (This is your Linux session, not a Docker account.) |
| `ERROR: BuildKit is enabled but the buildx component is missing or broken`, or `DEPRECATED: The legacy builder is deprecated` | The build tool buildx is missing. Docker's own packages: `sudo apt-get install docker-buildx-plugin`; Ubuntu's `docker.io`: `sudo apt-get install docker-buildx`. The old builder makes the sitl/sim images run as root, so `build_images.sh` refuses to build without it. |
| `docker: 'compose' is not a docker command`, or `unknown shorthand flag: 'f' in -f` | Docker Compose is missing: `sudo apt-get install docker-compose-plugin` (or re-run `install_docker_engine.sh`). The old separate `docker-compose` (version 1) is not enough; 2.20 or newer is needed. |
| `error getting credentials - err: exec: "docker-credential-desktop.exe": executable file not found in $PATH` | `~/.docker/config.json` names a password helper that is not there (left over from Docker Desktop). No login is needed: `check_host.sh` prints a one-line command that removes the `credsStore` setting. |
| `toomanyrequests: ... unauthenticated pull rate limit` | Docker Hub's anonymous limit on a shared network: `docker login` with a free account, wait 6 hours, or load saved images. See [Do I need a login?](#no-login). |
| `pull access denied for astra-sim` (or `astra-dev`, `astra-sitl`), or `repository does not exist or may require 'docker login'`, or `No such image: astra-sim:latest` | Not a login problem: these images are built locally, not published. Build or load them first (A4). Details: [the login section](#no-login). |
| `The "HOME" variable is not set. Defaulting to a blank string.` | `docker compose` was typed in Windows PowerShell. Run the scripts in the Ubuntu terminal (`bash simulation/docker/compose_up.sh`). |
| The doctor says "a Docker network astra_net exists that compose did not make", or `network astra_net was found but has incorrect label com.docker.compose.network` | An old hand-made network blocks compose: `bash simulation/docker/compose_down.sh --solo`, then `docker network rm astra_net`. `compose_up.sh` makes the right one. |
| `Conflict. The container name "/astra-sim" is already in use` | A container left from another start: `bash simulation/docker/compose_down.sh --solo` (or `docker rm -f astra-sim`). |
| `Pool overlaps with other one on this address space`, or the doctor says the drone network 192.168.144.0/24 is already used | Something else uses 192.168.144.x (a network card on the real drone's network, a VPN, another Docker network, also a bigger one such as `192.168.144.0/20` that Docker gives out by itself once its usual 172.x addresses are used up): unplug / switch it off, or `docker network rm <that network>`. The containers need exactly these addresses. |
| The doctor says "a wider network route covers the drone network" (for example `192.168.0.0/16 dev tun0`) | A VPN or a big home/school network includes the 192.168.144.x addresses. This does not stop the containers: they start and work. Only while they run, this computer cannot reach 192.168.144.x on that other network. Fine for the simulator; if it IS the real drone's network, unplug it while you use the containers. |

**Building the images** (`build_images.sh`)

| Message or symptom | What it means and what to do |
|---|---|
| `EXPKEYSIG F42ED6FBAB17C654 Open Robotics`, or `The repository 'http://packages.ros.org/ros2/ubuntu jammy InRelease' is not signed` | Your saved ROS base image is older than June 2025 and trusts an expired key. `build_images.sh` now fetches a fresh one by itself; by hand: `docker pull ros:humble-ros-base-jammy`. |
| `Release file for ... is not valid yet (invalid for another ...)` | The Linux clock is behind (common in WSL after the laptop slept): `sudo hwclock -s`, or `wsl --shutdown` in PowerShell and open Ubuntu again. |
| `Temporary failure resolving 'archive.ubuntu.com'`, `Could not resolve host: github.com` | No internet or broken name lookup (DNS), often a VPN. Disconnect the VPN, or add `dnsTunneling=true` under `[wsl2]` in `%UserProfile%\.wslconfig` and `wsl --shutdown`. For containers only: add `"dns": ["8.8.8.8", "1.1.1.1"]` to `/etc/docker/daemon.json`, `sudo systemctl restart docker`. |
| `SSL certificate problem: self-signed certificate in certificate chain`, `certificate verify failed` | The network (school/company) inspects HTTPS. Build on another network (home, phone hotspot), or ask IT for their root certificate. |
| A download stops in `install_geographiclib_datasets.sh` (SourceForge), `git clone` (GitHub) or `pip` | The download site had a hiccup: run `build_images.sh` again; finished steps are not repeated. |
| `c++: fatal error: Killed signal terminated program cc1plus`, or the build stops with `exit code: 137` | The build ran out of memory. Use fewer compile jobs: `bash simulation/docker/build_images.sh --jobs 2`, close programs, or give WSL more memory (`memory=12GB` under `[wsl2]` in `%UserProfile%\.wslconfig`, then `wsl --shutdown`). |
| `no space left on device` | The disk that holds Docker's images is full. See `docker system df`; `docker builder prune` removes old build data. On Windows the images live inside the Ubuntu-22.04 virtual disk (Engine) or Docker Desktop's disk (Settings -> Resources -> Advanced). |
| `ERROR: failed to solve: ... no match for platform in manifest` or "The sim image needs an amd64 (x86_64) computer" | The simulator image needs an Intel/AMD PC; an ARM PC or Apple Silicon Mac can only build the `dev` image. |
| The doctor or `compose_up.sh` says an image "is OLDER than docker/Dockerfile or docker/requirements-*.txt" | Somebody changed those files after you built: `bash simulation/docker/build_images.sh` (only changed steps run). |
| The doctor says an image "runs as root" | It was built without buildx (the old builder). Install buildx (row above) and rebuild with `build_images.sh`. |
| The doctor says an image "was built for user id 4242, you are 1001" | The image came from another PC (or another account). It works, but files it writes into the repo belong to that other id. Rebuild for yourself: `bash simulation/docker/build_images.sh`. |
| The doctor says an image "uses user id 1000; yours (...) is below 1000" | Only a note. Accounts with an id below 1000 are unusual (system accounts); `build_images.sh` then always uses 1000, so a rebuild would not change anything. Files the containers write belong to user 1000. |

**Running** (`compose_up.sh`, `run_sim_container.sh`, VS Code)

| Message or symptom | What it means and what to do |
|---|---|
| `could not select device driver "nvidia" with capabilities: [[gpu]]` | Docker has no NVIDIA support: `sudo bash simulation/docker/install_docker_engine.sh --nvidia`, or start with `--no-gpu`. |
| `error gathering device information while adding custom device "/dev/dxg"` | The WSLg add-on file was used outside WSL2 (or WSL is too old). Let `compose_up.sh` choose the files, or in PowerShell `wsl --update`. |
| No windows appear; `cannot open display`, `qt.qpa.xcb: could not connect to display` | WSL: the doctor checks WSLg; if `/tmp/.X11-unix/X0` is missing, run `sudo ln -sf /mnt/wslg/.X11-unix/X0 /tmp/.X11-unix/X0`; old WSL: `wsl --update`. Linux: `sudo apt-get install x11-xserver-utils` (xhost), run from the desktop's own terminal. Or use `--headless`. |
| `bash simulation/...` typed in PowerShell opens the wrong Linux, or "No such file" | `bash` in PowerShell starts the DEFAULT WSL distro, which may not be Ubuntu-22.04 (`wsl -l -v` shows it with a `*`). Open the right one: `wsl -d Ubuntu-22.04`. |
| VS Code "Reopen in Container" for **ASTRA simulator** fails in `initializeCommand` | Open the folder FROM Ubuntu (`cd ~/ODLC_Machine_Inferencing_System_2025-2026 && code .`) and set `"dev.containers.executeInWSL": true`; then build the image once (`build_images.sh sim`). |
| In the team dev container (**ASTRA ODLC dev**), `image_view` or another window says `cannot open display: host.docker.internal:0.0` or `Could not connect to display` | That container sends windows to an X server PROGRAM on the host: XQuartz on a Mac, VcXsrv/XLaunch on Windows (start XLaunch with "Disable access control" ticked). The name `host.docker.internal` does not exist for Docker Engine on Linux or inside Ubuntu on Windows, so there it cannot work as it is. Use the **ASTRA simulator** dev container or `run_sim_container.sh` instead: they show windows on your own desktop (WSLg or X11). |
| A native `colcon build` says `Permission denied` in `ros2_ws/build`, `install` or `log` after a Docker run | Docker created those folders owned by root (fixed: the scripts now create them first). Repair once: `sudo chown -R $(id -u):$(id -g) ros2_ws/build ros2_ws/install ros2_ws/log`. |
| `bind: address already in use` (`run_jetson_container.sh --net host`), or ports 14550/14552/5760/8081 busy | A native simulator or MAVROS runs on this computer: `bash simulation/scripts/stop_sim.sh`, or use the split setup (`compose_up.sh`), whose containers have their own network. |
| Mission Planner shows no camera picture with the Docker simulator | Known gap: the camera stream (`http://127.0.0.1:8081/pov.mjpg`) listens only inside the sim container (sim.launch.py has no setting for its address yet). Use the native simulator for the Mission Planner camera. |
| Windows edit has no effect | The containers run the Linux-side copy: `compose_up.sh` now warns when it is older; refresh with `bash simulation/docker/compose_up.sh --sync-only` (or start with `--sync`). |
| `compose_up.sh` (or `run_sim_container.sh`, `run_jetson_container.sh`) started from a Windows drive waits about 3 seconds before doing anything | It is comparing the Windows folder with the Linux-side copy (the warning in the row above). To skip that look: `export ASTRA_NO_COPY_CHECK=1`; then remember `--sync` yourself after editing on Windows. Tool caches (`.ruff_cache`, `.pytest_cache` ...) are not counted as changes. |
| Engine is unreachable | `docker info`; Engine setup: `sudo systemctl start docker`; Desktop setup: start Desktop. |
| Missing image | Build the named target or load all required image tags; no transfer helper exists yet. |
| No windows | Verify `$DISPLAY`, X11/WSLg sockets and the selected overrides; first try `--headless`. |
| CUDA unavailable | Check the NVIDIA runtime and the image's PyTorch flavor (`check_host.sh` shows both); `--no-gpu` uses the CPU detector. |
| Slow camera/rendering | Check the render device/driver; headless cameras still render a 3D scene. |
| Team nodes invisible across containers | Match ROS domain, inspect `docker logs astra-jetson`, check DDS networking/IPC. |
| Files owned by another user | Compare host `id -u` with the image's user (`check_host.sh` shows it); rebuild for this user's ID. |
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

They include the doctor (`test_check_host.py`: stand-in `docker`/`ip` programs and throwaway git repositories,
no daemon) and the line-ending rules (`test_line_endings.py`, which also runs on Windows Python). Checked on
2026-10-31 on the team laptop: 30 pass in Ubuntu-22.04; on Windows Python the 9 line-ending tests pass and the
Linux-only ones are skipped. `check_host.sh` and `check_host.ps1` were run on that laptop (no `[FAIL]`), and
both reported a Windows-style clone (`core.autocrlf=true`) as failing with `main_controller.py` named.
