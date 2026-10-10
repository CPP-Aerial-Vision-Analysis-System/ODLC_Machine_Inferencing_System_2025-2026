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
#
# Every version lives in the ARGs below. To switch one, change it here or pass
# --build-arg NAME=value, then rebuild.
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

RUN apt-get update && apt-get install -y --no-install-recommends meson ninja-build \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/mavlink-router/mavlink-router.git /src/mavlink-router \
    && cd /src/mavlink-router \
    && git checkout ${MAVLINK_ROUTER_COMMIT} \
    && git submodule update --init --recursive \
    && meson setup build . -Dsystemdsystemunitdir=/usr/lib/systemd/system \
    && ninja -C build

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

USER root
# empy / pexpect / future (apt) and MAVProxy (pip) are already in dev.
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /opt/ardupilot \
    && cd /opt/ardupilot \
    && ./waf configure --board sitl \
    && ./waf copter \
    && chown -R ${USERNAME}:${USERNAME} /opt/ardupilot

ENV PATH=/opt/ardupilot/Tools/autotest:${PATH}
USER ${USERNAME}

# -----------------------------------------------------------------------------
# sim — sitl + Gazebo Harmonic (amd64 only; for the dedicated sim computer)
# -----------------------------------------------------------------------------
FROM sitl AS sim

ARG ARDUPILOT_GAZEBO_REF=main

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

RUN git clone --depth 1 --branch ${ARDUPILOT_GAZEBO_REF} https://github.com/ArduPilot/ardupilot_gazebo.git /opt/ardupilot_gazebo \
    && cmake -S /opt/ardupilot_gazebo -B /opt/ardupilot_gazebo/build -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && cmake --build /opt/ardupilot_gazebo/build -j"$(nproc)" \
    && chown -R ${USERNAME}:${USERNAME} /opt/ardupilot_gazebo

ENV GZ_VERSION=harmonic \
    GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_gazebo/build \
    GZ_SIM_RESOURCE_PATH=/opt/ardupilot_gazebo/models:/opt/ardupilot_gazebo/worlds
USER ${USERNAME}
