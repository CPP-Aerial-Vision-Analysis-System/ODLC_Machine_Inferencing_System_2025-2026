FROM joestrada1022/suas-sim:ros2-gazebo

USER root

# Create vscode user directory
RUN mkdir -p /home/vscode && chmod 777 /home/vscode

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


USER $USERNAME

RUN sudo usermod --append --groups video $USERNAME

# Combine apt commands to reduce layers and clean cache
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-view \
        ros-humble-pcl-* && \
    sudo apt clean && sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/ultra_ws/src && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/ultra_ws/; colcon build"

# Install ultralytics with pip cache cleanup
RUN cd ~/ultra_ws/src && \
    git clone -b humble-devel https://github.com/CPP-Aerial-Vision-Analysis-System/ultralytics_ros.git && \
    cd ultralytics_ros && \
    python3 -m pip install --no-cache-dir -r requirements.txt
    