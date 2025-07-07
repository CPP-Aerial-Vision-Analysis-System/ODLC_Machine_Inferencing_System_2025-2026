FROM osrf/ros:humble-desktop-full

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG COPTER_TAG=Copter-4.0.4

# Create user and grant sudo
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
    groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

RUN apt-get update && apt-get install -y \
    sudo \
    python3-pip \
    python3-venv \
    python3-setuptools \
    python3-wheel \
    python3-matplotlib \
    python3-serial \
    python3-wxgtk4.0 \
    python3-lxml \
    python3-scipy \
    python3-opencv \
    python3-numpy \
    rapidjson-dev \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set user
USER $USERNAME

# Add to video group (if needed)
RUN sudo usermod --append --groups video $USERNAME

# Update system and install git
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install -y git

# Setup ROS 2 in .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Update rosdep
RUN rosdep update

################################
## MAVROS installation ##
################################
RUN sudo apt-get install -y \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras

# GeographicLib required for MAVROS
USER root
WORKDIR /tmp
RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh
USER $USERNAME

################################
## Gazebo Garden installation
################################

RUN sudo apt-get update && \
    sudo apt-get -y install libopencv-dev libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

RUN sudo apt-get update && \
    sudo apt-get -y install curl lsb-release gnupg

RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN sudo apt-get update && \ 
    sudo apt-get -y install gz-garden

ENV GZ_VERSION=garden

################################
## MAVProxy installation ##
################################
RUN pip3 install --user --no-cache-dir MAVProxy

################################
## ArduPilot SITL setup ##
################################
WORKDIR /home/${USERNAME}

ENV USER=${USERNAME}

RUN git clone https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    git submodule update --init --recursive && \
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    . ~/.profile && \
    ./waf configure --board sitl && \
    ./waf copter


#############################
## Ardupilot Gazebo Plugin ##
#############################

WORKDIR /home/${USERNAME}

RUN git clone https://github.com/ArduPilot/ardupilot_gazebo && \
    cd ardupilot_gazebo && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    make -j$(nproc)

WORKDIR /home/${USERNAME}/ardupilot

RUN echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
RUN echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc