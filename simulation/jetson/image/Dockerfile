# =============================================================================
#  simulation/jetson/image/Dockerfile  --  layer 3 of the Jetson-in-the-loop image: the team stack, built in
#
#  PURPOSE: turn the team's development image (layer 2 = the EXISTING docker/Dockerfile target "dev", built for the
#  PC as astra-dev:latest or for the Jetson as astra-dev:jetson on top of the L4T/TensorRT layer 1) into a
#  SELF-CONTAINED image of the team's flight software, so "putting the container on the Jetson" is all it takes:
#    * the repo laid out at /home/astra-dev/astra, the drone's own layout (new_od's default model_path
#      /home/astra-dev/astra/ros2_ws/src/yolo26m.pt is right with no override);
#    * the team packages (image/context.list) built with colcon, exactly as on the drone; launcher.sh unchanged;
#    * the model files are NOT in the image: ros2_ws/src/yolo26m.pt and yolo26m.engine point into /models, a folder
#      on the Jetson mounted at run time (an engine must be built on the board that runs it: jetson.sh warmup);
#    * layer 2's user "ubuntu" renamed to astra-dev; the entrypoint gives it the Jetson user's ids at start;
#    * /opt/astra: entrypoint.sh (modes sim / real / shell / selftest / warmup), in_stack.sh, selftest.py,
#      warmup.py, hil_relay.py (the camera and MAVLink relay), ros_tap.py + tap_topics.txt (the read-only ROS tap,
#      when the PC side has written them), paths.env (photo folders, checked here), image.env (git sha, flavor);
#    * labels org.astra.owner=hil (our commands only ever stop or remove images and containers with it),
#      org.astra.git_sha, org.astra.built, org.astra.flavor.
#  The build context is NOT the repo: `jetson.sh image build` stages it (repo/ = the files of context.list,
#  opt/ = the /opt/astra files, extra/ = the new video pipeline's proposed launch file and packages when present).
#  Build (through jetson.sh, which also builds layers 1-2 when needed):
#      bash simulation/jetson/jetson.sh image build --flavor virtual     (this PC: astra-hil:virtual)
#      bash simulation/jetson/jetson.sh image build                      (the Jetson: astra-hil:jetson, arm64)
#  Design: docs/jetson/HIL_DESIGN.md section 8.
# =============================================================================
ARG DEV_IMAGE=astra-dev:latest
FROM ${DEV_IMAGE}

ARG FLAVOR=virtual
ARG GIT_SHA=unknown
# CODE_SHA: the fingerprint of the files that go into this image (jetson.sh code_sha); `check` compares it with the PC
ARG CODE_SHA=unknown
# TENSORRT=yes: layer 1 brings TensorRT for Python (the Jetson's L4T base, or the PC's TensorRT 10.3 base); the team's
# new_od then runs a TensorRT engine exactly as on the drone. no: the plain ROS base (new_od runs the .pt on PyTorch).
ARG TENSORRT=no
ARG ROS_DISTRO=humble
ARG COLCON_WORKERS=4

USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# --- tools the stack needs here that layer 2 does not have ----------------------------------------------------------
# ffmpeg: the new video pipeline's stream node pulls the RTSP stream with it (zr10_stream). util-linux brings setpriv
# (the entrypoint drops from root to astra-dev with it). procps: ps/pgrep for launcher.sh and stack checks.
RUN apt-get update && apt-get install -y --no-install-recommends ffmpeg util-linux procps \
    && rm -rf /var/lib/apt/lists/*

# --- ONNX for the TensorRT export (images with TensorRT only) ---------------------------------------------------------
# The team's new_od builds its engine through Ultralytics, which first writes an ONNX file and needs "import onnx"
# (onnx >= 1.12). Layer 2's apt python3-onnx is 1.7.0 and cannot even be imported next to numpy 1.26.4 (it uses
# np.object, removed in numpy 1.24): VERIFIED 2026-10-31, warmup failed with "module 'numpy' has no attribute
# 'object'". So the team's OWN pins go in: onnx==1.14.0 (ros2_ws/src/ultralytics_ros/requirements.txt) and
# protobuf==3.20.3 (ros2_ws/src/fixtensorRT.md, "mandatory"). numpy stays 1.26.4 (checked below).
ARG ONNX_VERSION=1.14.0
ARG PROTOBUF_VERSION=3.20.3
RUN if [ "${TENSORRT}" = yes ]; then \
        python3 -m pip install --no-cache-dir "onnx==${ONNX_VERSION}" "protobuf==${PROTOBUF_VERSION}" "numpy==1.26.4" \
        && python3 -c "import onnx, numpy; print('onnx', onnx.__version__, 'numpy', numpy.__version__)"; \
    fi

# --- the user: layer 2's "ubuntu" becomes astra-dev (the drone's user), no sudo inside -----------------------------
RUN if id ubuntu >/dev/null 2>&1; then \
        usermod -l astra-dev -d /home/astra-dev -m ubuntu && groupmod -n astra-dev ubuntu ; \
    fi \
    && rm -f /etc/sudoers.d/ubuntu /etc/sudoers.d/astra-dev \
    && id astra-dev

# --- the team repo (only the files of context.list) ----------------------------------------------------------------
COPY --chown=astra-dev:astra-dev repo/ /home/astra-dev/astra/
COPY extra/ /opt/astra/extra/
# the new video pipeline's packages (zr10_ros2, zr10_stream), when `image build` found them (HIL_DESIGN.md 6.3)
RUN if [ -d /opt/astra/extra/new_pipeline ] && [ -n "$(ls -A /opt/astra/extra/new_pipeline)" ]; then \
        cp -r /opt/astra/extra/new_pipeline/* /home/astra-dev/astra/ros2_ws/src/ \
        && chown -R astra-dev:astra-dev /home/astra-dev/astra/ros2_ws/src ; \
    fi

USER astra-dev
WORKDIR /home/astra-dev/astra/ros2_ws
# colcon: copy install (not symlink), Release, the listed packages plus the new pipeline's when present.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && pkgs="interfaces video_cam main detection mapping waypoint_mavros wp_sender gps_ros2 sim_bringup" \
    && for p in zr10_ros2 zr10_stream; do if [ -d "src/$p" ]; then pkgs="$pkgs $p"; fi; done \
    && echo "colcon packages: $pkgs" \
    && MAKEFLAGS="-j${COLCON_WORKERS}" colcon build --packages-select $pkgs --parallel-workers "${COLCON_WORKERS}" \
         --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_cohesion+ \
    && rm -rf build log \
    && mkdir -p src/video_cam/mapping_photos src/video_cam/detection_results_sahi src/video_cam/camera_feed \
    && ln -sfn /models/yolo26m.pt src/yolo26m.pt \
    && ln -sfn /models/yolo26m.engine src/yolo26m.engine

# --- /opt/astra: entrypoint, relay, tap, checks; the photo folders confirmed with the simulator's own rule ----------
USER root
COPY opt/ /opt/astra/
RUN chmod 755 /opt/astra/*.sh /opt/astra/*.py \
    && source /opt/ros/${ROS_DISTRO}/setup.bash && source /home/astra-dev/astra/ros2_ws/install/setup.bash \
    && photo="$(python3 -m sim_bringup.sim_paths photo-dir)" \
    && want=/home/astra-dev/astra/ros2_ws/src/video_cam/mapping_photos \
    && if [ "$photo" != "$want" ]; then \
         echo "ERROR: the team code resolves the photo folder to $photo, not $want; jetson.sh mounts $want." >&2; exit 1; fi \
    && printf 'WS=/home/astra-dev/astra/ros2_ws\nPHOTO_DIR=%s\nDETECTION_DIR=%s\n' \
         "$photo" "/home/astra-dev/astra/ros2_ws/src/video_cam/detection_results_sahi" > /opt/astra/paths.env \
    && mkdir -p /run/astra_hil /models /data/runs \
    && chown astra-dev:astra-dev /run/astra_hil /models /data/runs \
    && if [ "${TENSORRT}" = yes ]; then \
         trtv="$(python3 -c 'import tensorrt; print(tensorrt.__version__)')" \
         || { echo "ERROR: TENSORRT=yes but 'import tensorrt' fails in this image. Likely cause: layer 1 is not the TensorRT/L4T base. Fix: jetson.sh image build (default --x86-base trt)" >&2; exit 1; }; \
         echo "TensorRT for Python: $trtv"; printf 'TENSORRT=yes\nTENSORRT_VERSION=%s\n' "$trtv" >> /opt/astra/image.env; \
       else printf 'TENSORRT=no\n' >> /opt/astra/image.env; fi \
    && cat /opt/astra/paths.env /opt/astra/image.env

LABEL org.astra.owner="hil" \
      org.astra.flavor="${FLAVOR}" \
      org.astra.git_sha="${GIT_SHA}" \
      org.astra.code_sha="${CODE_SHA}" \
      org.astra.tensorrt="${TENSORRT}" \
      org.opencontainers.image.title="astra-hil" \
      org.opencontainers.image.description="ASTRA team flight stack for Jetson-in-the-loop (docs/jetson/HIL_DESIGN.md)"
ENV ASTRA_FLAVOR=${FLAVOR} \
    ASTRA_TENSORRT=${TENSORRT} \
    ASTRA_HIL_MODE=sim \
    YOLO_AUTOINSTALL=false
WORKDIR /home/astra-dev/astra
# The entrypoint starts as root only to add the camera stand-in address to the container's OWN loopback, then runs
# everything as astra-dev. `docker run --init` is used by jetson.sh so signals reach launcher.sh.
ENTRYPOINT ["/opt/astra/entrypoint.sh"]
CMD ["sim"]
