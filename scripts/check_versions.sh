#!/usr/bin/env bash
# Print the versions that must match between the dev container and the Jetson.
# Run in both places and diff the output:
#   ./scripts/check_versions.sh > versions_$(hostname).txt

row() { printf '%-22s %s\n' "$1" "${2:-not installed}"; }
pyver() { python3 -c "import $1; print($1.__version__)" 2>/dev/null; }
debver() { dpkg-query -W -f='${Version}' "$1" 2>/dev/null; }

row "machine"         "$(uname -m)"
row "os"              "$(. /etc/os-release && echo "$PRETTY_NAME")"
row "jetpack (l4t)"   "$(head -1 /etc/nv_tegra_release 2>/dev/null)"
row "ros distro"      "${ROS_DISTRO}"
row "mavros"          "$(debver "ros-${ROS_DISTRO}-mavros")"
row "cv_bridge"       "$(debver "ros-${ROS_DISTRO}-cv-bridge")"
row "mavlink-router"  "$(mavlink-routerd --version 2>/dev/null | head -1 | awk '{print $NF}')"
row "rmw"             "${RMW_IMPLEMENTATION:-default (fastrtps)}"
echo "--- C++"
row "gcc"             "$(gcc -dumpfullversion 2>/dev/null)"
row "cmake"           "$(cmake --version 2>/dev/null | head -1 | awk '{print $3}')"
row "opencv (C++)"    "$(pkg-config --modversion opencv4 2>/dev/null)"
row "tensorrt"        "$(debver libnvinfer-dev)"
row "onnxruntime"     "$(cat /opt/onnxruntime/VERSION_NUMBER 2>/dev/null)"
row "cuda"            "$(dpkg-query -W -f='${Version}\n' 'cuda-cudart-1[0-9]-*' 2>/dev/null | grep -v '^$' | head -1)"
echo "--- Python"
row "python"          "$(python3 -c 'import platform; print(platform.python_version())')"
row "numpy"           "$(pyver numpy)"
row "opencv (python)" "$(pyver cv2)"
row "torch"           "$(python3 -c 'import torch; print(torch.__version__, "cuda" if torch.cuda.is_available() else "cpu")' 2>/dev/null)"
row "torchvision"     "$(pyver torchvision)"
row "ultralytics"     "$(python3 -c 'from importlib.metadata import version; print(version("ultralytics"))' 2>/dev/null)"
row "sahi"            "$(pyver sahi)"
row "pymavlink"       "$(python3 -c 'from importlib.metadata import version; print(version("pymavlink"))' 2>/dev/null)"
row "MAVProxy"        "$(python3 -c 'from importlib.metadata import version; print(version("MAVProxy"))' 2>/dev/null)"
row "watchdog"        "$(python3 -c 'from importlib.metadata import version; print(version("watchdog"))' 2>/dev/null)"
