# =============================================================================
# ASTRA ODLC — development image (Python + C++, ROS 2 Humble)
# =============================================================================
# One Dockerfile, three targets. Docker only builds the target you ask for,
# so nobody downloads Gazebo unless they build `sim`.
#
#   dev   (default)  ROS 2 Humble + MAVROS + Python and C++ toolchains.
#                    Everyone uses this (VS Code "Reopen in Container").
#   sitl             dev + ArduPilot SITL, headless. Lets `./launcher.sh sim`
#                    run on a laptop. Works on Mac, Windows and Linux.
#   sim              sitl + Gazebo Harmonic + ardupilot_gazebo plugin.
#                    Dedicated sim computer only (amd64 / x86).
#
# Build (from the repo root):
#   docker build -t astra-dev          --target dev  docker/
#   docker build -t astra-sitl         --target sitl docker/
#   docker build -t astra-sim          --target sim  --platform linux/amd64 docker/
# or all three with the laptop-friendly helper (same commands, plus the options
# below picked for you): bash simulation/docker/build_images.sh
#
# Every version lives in the ARGs below. To switch one, change it here or pass
# --build-arg NAME=value, then rebuild.
#
# BUILD_JOBS (default 4) = how many C/C++ files are compiled at the same time
# (mavlink-router, ArduPilot SITL, the ArduPilot Gazebo plugin). Without a limit
# the compilers start one job per CPU core and can freeze a laptop or a WSL VM;
# on a big desktop pass e.g. --build-arg BUILD_JOBS=16 to build faster.
#
# Running the Gazebo simulator + the team's code in containers (the "sim" image
# and the "dev" image as a Jetson stand-in on the drone's 192.168.144.x network):
# docker/compose.sim.yml and docs/simulation/DOCKER.md.
#
# Versions were read off the Jetson on 2026-10-10 (scripts/check_versions.sh):
#   Jetson Orin Nano | JetPack 6.2 / L4T R36.4.7 | Ubuntu 22.04.5 | Python 3.10.12
#   CUDA 12.6 | cuDNN 9.3.0 | TensorRT 10.3.0      (GPU-only — not in this image)
#   ROS 2 Humble (ros-base) | Fast DDS (default RMW) | MAVROS 2.12.0
#   OpenCV: Ubuntu 4.5.4 for C++ (same as cv_bridge) + pip opencv-python 4.10 for Python
#   mavlink-router v4-16-g2362c62 | Pixhawk: ArduCopter 4.6.3
# =============================================================================

ARG ROS_DISTRO=humble
ARG BASE_IMAGE=ros:${ROS_DISTRO}-ros-base-jammy
# Parallel compile jobs for every C/C++ build in this file (see the header).
ARG BUILD_JOBS=4

# -----------------------------------------------------------------------------
# toolchain — compilers and basic tools shared by every stage
# -----------------------------------------------------------------------------
FROM ${BASE_IMAGE} AS toolchain

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# gcc 11.4 / cmake 3.22 are the Ubuntu 22.04 defaults — same as the Jetson.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config git curl ca-certificates gnupg \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# mavlink-router — same commit as the Jetson's /usr/bin/mavlink-routerd.
# comms/mavlink_router.py launches it.
# -----------------------------------------------------------------------------
FROM toolchain AS mavlink-router

ARG MAVLINK_ROUTER_COMMIT=2362c620f483
ARG BUILD_JOBS

RUN apt-get update && apt-get install -y --no-install-recommends meson ninja-build \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/mavlink-router/mavlink-router.git /src/mavlink-router \
    && cd /src/mavlink-router \
    && git checkout ${MAVLINK_ROUTER_COMMIT} \
    && git submodule update --init --recursive \
    && meson setup build . -Dsystemdsystemunitdir=/usr/lib/systemd/system \
    && ninja -C build -j "${BUILD_JOBS:-4}"

# -----------------------------------------------------------------------------
# dev
# -----------------------------------------------------------------------------
FROM toolchain AS dev

ARG ROS_DISTRO
# MAVROS is currently missing from the main ROS apt repo for Humble, so it is
# pulled from a dated, frozen ROS snapshot. The Jetson has 2.12.0, which no
# snapshot carries; 2.14.0 is the closest. Between the two, mavros_msgs only
# gained one new message (ObstacleDistance3D) — no existing msg/srv changed.
ARG MAVROS_SNAPSHOT=2026-08-07
ARG ONNXRUNTIME_VERSION=1.22.0
# Python ML stack. Set WITH_PY_ML=false once detection is fully C++ (~1 GB smaller).
ARG WITH_PY_ML=true
# CPU builds: no NVIDIA GPU on Macs or normal Windows PCs. For a Linux/Windows
# machine with an NVIDIA GPU use e.g. https://download.pytorch.org/whl/cu126
ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cpu
ARG TORCH_VERSION=2.8.0
ARG TORCHVISION_VERSION=0.23.0
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

ENV PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# --- Dev tools + libraries -----------------------------------------------------
# The python3-* packages are the apt versions the Jetson's system Python has
# (requests 2.25.1, pyserial 3.5, lxml 4.8.0, onnx, ...).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ccache gdb clang-format clang-tidy \
        sudo less nano bash-completion \
        python3-pip python3-dev python3-pytest \
        python3-requests python3-serial python3-lxml python3-future python3-pexpect python3-empy python3-onnx \
        libcurl4-openssl-dev nlohmann-json3-dev \
        geographiclib-tools \
    && rm -rf /var/lib/apt/lists/*

# --- ROS packages our workspace uses -------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-vision-msgs \
        ros-${ROS_DISTRO}-image-view \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-image-transport \
        ros-${ROS_DISTRO}-message-filters \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-eigen \
        ros-${ROS_DISTRO}-geometry-msgs \
        ros-${ROS_DISTRO}-visualization-msgs \
        ros-${ROS_DISTRO}-ament-cmake-gtest \
        ros-${ROS_DISTRO}-ament-lint-auto \
        ros-${ROS_DISTRO}-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

# --- MAVROS (from the snapshot repo, pinned so nothing else comes from it) ----
RUN curl -fsSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA' \
        | gpg --dearmor -o /usr/share/keyrings/ros-snapshots.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-snapshots.gpg] http://snapshots.ros.org/${ROS_DISTRO}/${MAVROS_SNAPSHOT}/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros-snapshot.list \
    && printf 'Package: *\nPin: origin snapshots.ros.org\nPin-Priority: 100\n\nPackage: ros-%s-mavros ros-%s-mavros-extras ros-%s-mavros-msgs ros-%s-libmavconn\nPin: origin snapshots.ros.org\nPin-Priority: 1001\n' \
        ${ROS_DISTRO} ${ROS_DISTRO} ${ROS_DISTRO} ${ROS_DISTRO} > /etc/apt/preferences.d/ros-snapshot-mavros \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-mavros \
        ros-${ROS_DISTRO}-mavros-extras \
    && /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh \
    && rm -rf /var/lib/apt/lists/*

# --- mavlink-routerd, built in the stage above --------------------------------
COPY --from=mavlink-router /src/mavlink-router/build/src/mavlink-routerd /usr/bin/mavlink-routerd

# --- ONNX Runtime C++ (CPU) — inference backend for C++ nodes off-Jetson ------
# The Jetson uses TensorRT instead; both load the same .onnx model.
RUN case "$(dpkg --print-architecture)" in \
        amd64) ORT_ARCH=x64 ;; arm64) ORT_ARCH=aarch64 ;; \
        *) echo "unsupported arch" >&2; exit 1 ;; \
    esac \
    && curl -fsSL "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION}.tgz" \
        | tar -xz -C /opt \
    && mv /opt/onnxruntime-linux-${ORT_ARCH}-${ONNXRUNTIME_VERSION} /opt/onnxruntime \
    `# The tarball's CMake config expects lib64/ and include/onnxruntime/; the files ship in lib/ and include/.` \
    && ln -s lib /opt/onnxruntime/lib64 \
    && ln -s . /opt/onnxruntime/include/onnxruntime \
    && echo /opt/onnxruntime/lib > /etc/ld.so.conf.d/onnxruntime.conf && ldconfig
ENV CMAKE_PREFIX_PATH=/opt/onnxruntime

# --- Python packages ----------------------------------------------------------
COPY requirements-dev.txt requirements-ml.txt /tmp/pip/
RUN python3 -m pip install -r /tmp/pip/requirements-dev.txt \
    && if [ "${WITH_PY_ML}" = "true" ]; then \
        python3 -m pip install --index-url "${TORCH_INDEX_URL}" \
            "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" \
        && python3 -m pip install -r /tmp/pip/requirements-ml.txt ; \
    fi \
    && rm -rf /tmp/pip

# --- Network tools the Jetson has ------------------------------------------------
# ./launcher.sh pings the camera (192.168.144.25) before it starts; without `ping`
# that check always says "did not respond". `ip` shows the container's address
# (192.168.144.100 when this image is the Jetson stand-in of docker/compose.sim.yml).
# Its own small layer, late in the stage, so adding it rebuilds almost nothing.
RUN apt-get update && apt-get install -y --no-install-recommends iputils-ping iproute2 \
    && rm -rf /var/lib/apt/lists/*

# --- User ---------------------------------------------------------------------
# dialout = serial access to the Pixhawk (/dev/ttyUSB0) on Linux hosts.
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && usermod -aG video,dialout ${USERNAME} \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Ultralytics must never pip-install things on its own (would break the pins).
# ccache wraps every C/C++ compile that colcon/CMake runs.
ENV YOLO_AUTOINSTALL=false \
    CMAKE_C_COMPILER_LAUNCHER=ccache \
    CMAKE_CXX_COMPILER_LAUNCHER=ccache \
    RCUTILS_COLORIZED_OUTPUT=1

USER ${USERNAME}
WORKDIR /home/${USERNAME}
RUN mkdir -p ~/.cache/ccache ~/.config \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

CMD ["bash"]

# -----------------------------------------------------------------------------
# sitl — dev + ArduPilot SITL (headless, no Gazebo)
# -----------------------------------------------------------------------------
FROM dev AS sitl

# Match the ArduCopter version flashed on the Pixhawk (4.6.3 as of 2026-09-15).
ARG ARDUPILOT_REF=Copter-4.6.3
ARG BUILD_JOBS

USER root
# empy / pexpect / future (apt) and MAVProxy (pip) are already in dev.
# The binary lands in /opt/ardupilot/build/sitl/bin/arducopter: the third place the simulator's launcher
# looks (after $ARDUPILOT_SITL_BIN and ~/sitl/arducopter, see ros2_ws/src/simulation/sim_bringup/sim_paths.py).
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && ./waf configure --board sitl \
    && ./waf -j "${BUILD_JOBS:-4}" copter \
    && chown -R ${USERNAME}:${USERNAME} /opt/ardupilot

ENV PATH=/opt/ardupilot/Tools/autotest:${PATH}
USER ${USERNAME}

# -----------------------------------------------------------------------------
# sim — sitl + Gazebo Harmonic (amd64 only; for the dedicated sim computer)
#
# Everything the Gazebo simulator of simulation/README.md needs (the packages in
# ros2_ws/src/simulation: sim_bringup, sim_models, sim_worlds, siyi_emulator,
# sim_demo): Gazebo Harmonic + ros_gz, the ArduPilot Gazebo plugin, ArduCopter SITL
# and MAVProxy (from sitl/dev), xacro, RViz, PyQt5, the Gazebo Python bindings,
# GStreamer, Mesa (incl. the d3d12 driver WSLg uses to draw on the NVIDIA card),
# fonts and network tools. The repo is NOT copied in: it is mounted at run time
# and its ros2_ws is built inside the container, so code changes need no rebuild.
#
# Two ways to run it (details: docs/simulation/DOCKER.md):
#   1. RECOMMENDED, two containers like the real drone (docker/compose.sim.yml):
#        bash simulation/docker/compose_up.sh
#      "sim" (this image) = Gazebo + SITL + camera emulator + demo panel at
#      192.168.144.25 (the real SIYI camera's address), and "jetson" (the dev image)
#      = the team's ./launcher.sh sim at 192.168.144.100 (the real Jetson's address).
#   2. Everything in ONE container (sim + team stack):
#        bash simulation/docker/run_sim_container.sh --demo runway
#      or the VS Code dev container .devcontainer/sim. What those run, by hand
#      (Windows 11 / WSLg, from a shell of the Ubuntu-22.04 WSL distro, in the repo folder):
#        docker run -it --rm --cap-add NET_ADMIN --shm-size=1g \
#          -e DISPLAY -e WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir -e PULSE_SERVER \
#          -v /tmp/.X11-unix:/tmp/.X11-unix -v /mnt/wslg:/mnt/wslg \
#          --device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
#          -e GALLIUM_DRIVER=d3d12 -e MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
#          -v "$PWD":/ODLC_Machine_Inferencing_System_2025-2026 -w /ODLC_Machine_Inferencing_System_2025-2026 astra-sim
#      Linux (X11): xhost +SI:localuser:$(id -un); then the same docker run with only
#          -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix (+ --gpus all -e NVIDIA_DRIVER_CAPABILITIES=all
#          with the NVIDIA Container Toolkit, or --device /dev/dri for Intel/AMD).
#      Inside:  bash simulation/setup/03_build.sh && bash simulation/scripts/demo.sh runway
#      --cap-add NET_ADMIN lets the scripts add 192.168.144.25 inside the container (passwordless
#      sudo); port 82 needs no extra setting (Docker lets containers open low ports).
# -----------------------------------------------------------------------------
FROM sitl AS sim

ARG ARDUPILOT_GAZEBO_REF=main
# The exact ardupilot_gazebo commit the native simulator was verified with (2026-10-10). "" = newest on
# ARDUPILOT_GAZEBO_REF. Pinned so a rebuild next month gets the same plugin, not whatever main is then.
ARG ARDUPILOT_GAZEBO_COMMIT=082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
ARG BUILD_JOBS

USER root
RUN if [ "$(dpkg --print-architecture)" != "amd64" ]; then \
        echo "The sim target is amd64-only: ros-${ROS_DISTRO}-ros-gzharmonic is not packaged for arm64." >&2; exit 1; \
    fi \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-${ROS_DISTRO}-ros-gzharmonic \
        libgz-sim8-dev rapidjson-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

# What the simulation packages need on top (see simulation/README.md):
#   xacro (drone model generation), rviz2 (camera POV layout), PyQt5 + Pillow (demo panel),
#   Gazebo Python bindings (camera emulator, payload release, demo markers talk gz-transport),
#   OpenCV/numpy/yaml for Python (also when WITH_PY_ML=false), iproute2 (camera address alias),
#   ping (launcher.sh checks the camera), procps/psmisc (pgrep/killall: stop_sim.sh, health checks),
#   Mesa = OpenGL/EGL drivers incl. d3d12_dri.so (WSLg draws Gazebo/RViz on the NVIDIA card through it)
#   + glxinfo (mesa-utils) and xdpyinfo (x11-utils) to check the display, fonts (window text),
#   GStreamer tools and base/good plugins (the plugin's optional GStreamer camera), xvfb (tests).
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-rviz2 \
        python3-pyqt5 python3-pil python3-numpy python3-yaml python3-opencv \
        python3-gz-transport13 python3-gz-msgs10 \
        iproute2 iputils-ping procps psmisc rsync \
        mesa-utils libgl1-mesa-dri libgl1 libegl1 libgles2 libglx-mesa0 libegl-mesa0 x11-utils \
        fonts-dejavu-core fonts-liberation \
        gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
        xvfb shellcheck \
    && rm -rf /var/lib/apt/lists/*

# The ArduPilot Gazebo plugin (the drone's link to SITL), built in /opt/ardupilot_gazebo/build: the second
# place sim_bringup/sim_paths.py looks (after $ARDUPILOT_GAZEBO_DIR and ~/ardupilot_gazebo).
# GZ_VERSION=harmonic tells the plugin's CMake which Gazebo to build against (the ENV line below only
# takes effect after this step; simulation/setup/02_setup_ardupilot.sh exports it the same way).
RUN git clone --branch ${ARDUPILOT_GAZEBO_REF} https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo \
    && cd /opt/ardupilot_gazebo \
    && if [ -n "${ARDUPILOT_GAZEBO_COMMIT}" ]; then git checkout --detach "${ARDUPILOT_GAZEBO_COMMIT}"; fi \
    && GZ_VERSION=harmonic cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && cmake --build /opt/ardupilot_gazebo/build -j "${BUILD_JOBS:-4}" \
    && chown -R ${USERNAME}:${USERNAME} /opt/ardupilot_gazebo

# Where simulation/setup/sim_env.sh and sim_bringup find ArduPilot's pieces in this image.
ENV GZ_VERSION=harmonic \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds \
    ARDUPILOT_GAZEBO_DIR=/opt/ardupilot_gazebo \
    ARDUPILOT_SITL_BIN=/opt/ardupilot/build/sitl/bin/arducopter \
    ARDUPILOT_DEFAULT_PARAMS=/opt/ardupilot/Tools/autotest/default_params \
    QT_X11_NO_MITSHM=1
USER ${USERNAME}
