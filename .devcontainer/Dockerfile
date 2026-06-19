# Cross-platform dev container image (works identically on macOS and Windows).
# A Docker image is built for Linux regardless of the host OS, so a single
# Dockerfile is all that is needed.

FROM joestrada1022/suas-sim:ros2-gazebo

ENV DEBIAN_FRONTEND=noninteractive

USER root

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

RUN usermod --append --groups video $USERNAME

# Install ROS deps (as root)
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-view \
        ros-humble-pcl-* && \
    rm -rf /var/lib/apt/lists/*

# Switch to user after system installs
USER $USERNAME
WORKDIR /home/$USERNAME

# Clone ultralytics ROS
RUN mkdir -p ~/ultra_ws/src && \
    cd ~/ultra_ws/src && \
    git clone -b humble-devel https://github.com/CPP-Aerial-Vision-Analysis-System/ultralytics_ros.git

# Install Python deps (CPU-only torch — no GPU/CUDA in this container)
RUN python3 -m pip install --no-cache-dir torch torchvision \
        --index-url https://download.pytorch.org/whl/cpu && \
    python3 -m pip install --no-cache-dir sahi ultralytics && \
    python3 -m pip install --no-cache-dir "numpy<2"

# Build workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd ~/ultra_ws && colcon build"
