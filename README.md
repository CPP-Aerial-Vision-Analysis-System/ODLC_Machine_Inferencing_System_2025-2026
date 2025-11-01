# Remote Development with ROS2 and GUI Support on Windows

## Prerequisites

### 1. Install the Remote Development Extension Pack

Download and install the Remote Development Extension Pack for VSCode:

🔗 [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

### 2. Set Up XLaunch for GUI Support

To use GUI applications on Windows, install **XLaunch**:

🔗 [Download XLaunch (VcXsrv)](https://sourceforge.net/p/vcxsrv/wiki/VcXsrv%20%26%20Win10/)

Once installed:
- Launch **XLaunch** before opening the devcontainer.
- When prompted for the display number, **change it from `-1` to `0`**.
- Press **Next** through the remaining steps without changing any other settings.

---

## Cloning DevContainer

### 1. Clone Repo
```bash
git clone https://github.com/CPP-Aerial-Vision-Analysis-System/ODLC_Machine_Inferencing_System_2025-2026.git
```

### 2. Download Docker Desktop (on linux download docker engine)

🔗 [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)

### 3. Download the image (can skip to step 4. will auto-download there)

In the terminal of Docker Desktop, download the Docker image:
```bash
docker pull joestrada1022/suas-sim:ros2-gazebo
```

### 4. Reopen in container

In VS Code:
- if prompted, you can press open when it asks you if you want to open the devcontainer.
- if you miss it or something, open the command pallete using ctrl + shift + p and press Reopen in Container

---

## Working Inside the Devcontainer

### 1. Source the Workspace

Once inside the devcontainer, run:

```bash
cd ~/ardu_ws
source install/setup.bash
````

### 2. Launch the Simulation

Run the following command to launch everything:

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### 3. Connect Mavros

Run the following command to get a mavproxy client running
```bash
mavproxy.py --master udp:127.0.0.1:1455 --out 127.0.0.1:14555
```

Connect Mavros to the mavproxy instance

```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@"
```

## Connecting Mission Planner

### 1. Forward Port 5762 in VSCode

* Press `Ctrl + J` to open the VSCode terminal panel.
* Locate the **Ports** section.
* **Add port `5762`** to forward it from the devcontainer.

### 2. Connect in Mission Planner

* In **Mission Planner**, choose **TCP** as the connection type.
* Use the following settings:

  * **IP Address:** `127.0.0.1`
  * **Port:** `5762`

---


