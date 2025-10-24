# this is a janky way of doing things but it works. there might be issues in the forseeable future
# but for setup this is enough
FROM joestrada1022/suas-sim:ros2-gazebo 

USER root

# Install and update everything (so nothing breaks)
RUN apt-get update && apt-get upgrade -y


RUN apt-get install -y ros-humble-cv-bridge \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-view \
        ros-humble-pcl-*


# remove apt stuff to save space (can alway sudo apt update to get these back)
RUN rm -r /var/lib/apt/lists/*

USER ubuntu

# Initialize colcon workspace
RUN mkdir -p ~/ultra_ws/src && \
    /bin/bash -c "source /opt/ros/humble/setup.bash; cd ~/ultra_ws/; colcon build"

# Clone repository and install using requirements.txt
RUN cd ~/ultra_ws/src && \
    git clone -b humble-devel https://github.com/CPP-Aerial-Vision-Analysis-System/ultralytics_ros.git && \
    cd ultralytics_ros && \
    python3 -m pip install -r requirements.txt