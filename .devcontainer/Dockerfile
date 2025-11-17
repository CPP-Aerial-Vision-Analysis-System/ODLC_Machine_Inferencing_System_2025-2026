FROM joestrada1022/suas-sim:ros2-gazebo

USER root

# --- Neutralize VS Code's injected .bash_history mount ---
RUN set -eux; \
    mkdir -p /home/vscode; \
    rm -f /home/vscode/.bash_history; \
    mkdir -p /home/vscode/.bash_history; \
    chmod 777 /home/vscode /home/vscode/.bash_history
# --- end neutralizer ---

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
RUN sudo apt update && sudo apt upgrade -y

RUN sudo apt install -y ros-humble-cv-bridge \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-view \
        ros-humble-pcl-*


RUN mkdir -p ~/ultra_ws/src && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/ultra_ws/; colcon build"


RUN cd ~/ultra_ws/src && \
    git clone -b humble-devel https://github.com/CPP-Aerial-Vision-Analysis-System/ultralytics_ros.git && \
    cd ultralytics_ros && \
    python3 -m pip install -r requirements.txt






# Install Python dependencies for video_cam package (SAHI, etc.)
# Install core dependencies directly in Dockerfile for better build caching
RUN pip3 install --no-cache-dir \
    torch>=2.0.0 \
    torchvision>=0.15.0 \
    sahi>=0.11.0 \
    opencv-python>=4.8.0 \
    opencv-contrib-python>=4.8.0 \
    Pillow>=10.0.0 \
    numpy>=1.24.0 \
    pandas>=2.0.0 \
    matplotlib>=3.7.0

# Verify critical installations
RUN python3 -c "import sahi; import torch; print('✓ Core dependencies (sahi, torch) installed successfully')" || \
    echo "⚠ Warning: Some dependencies may not be available - will install via requirements file on startup"

