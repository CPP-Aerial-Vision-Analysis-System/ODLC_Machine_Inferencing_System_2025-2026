# this is a janky way of doing things but it works. there might be issues in the forseeable future
# but for setup this is enough
FROM joestrada1022/suas-sim:ros2-gazebo 

USER root

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Install and update everything (so nothing breaks)
RUN sudo apt update && sudo apt upgrade -y


RUN sudo apt install -y ros-humble-cv-bridge \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-view \
        ros-humble-pcl-*


# Initialize colcon workspace
RUN mkdir -p ~/ultra_ws/src && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/ultra_ws/; colcon build"

# Clone repository and install using requirements.txt
RUN cd ~/ultra_ws/src && \
    git clone -b humble-devel https://github.com/CPP-Aerial-Vision-Analysis-System/ultralytics_ros.git && \
    cd ultralytics_ros && \
    python3 -m pip install -r requirements.txt


# remove thing because its supposed to be a submodule
RUN rm -rf ~/ultra_ws