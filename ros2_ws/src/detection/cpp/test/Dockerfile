FROM yolo26-ros:latest
USER root
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libopencv-dev python3-opencv
WORKDIR /work
