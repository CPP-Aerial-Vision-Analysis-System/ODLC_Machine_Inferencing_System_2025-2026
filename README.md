# Remote Development with ROS2 and GUI Support on Windows

## Prerequisites

### 1. Install the Remote Development Extension Pack

Download and install the Remote Development Extension Pack for VSCode:

 [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)

### 2. Set Up XLaunch for GUI Support

To use GUI applications on Windows, install **XLaunch**:

 [Download XLaunch (VcXsrv)](https://sourceforge.net/p/vcxsrv/wiki/VcXsrv%20%26%20Win10/)

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

 [Download Docker Desktop](https://www.docker.com/products/docker-desktop/)

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

### 3. Make camera face downwards (optional)

#### 3a. Open a mavproxy terminal
```bash
mavproxy.py --master=127.0.0.1:14550 --out=127.0.0.1:14552
```

#### 3b. Run the following RC overrides in the mavprxoy terminal to move gimbal in simulation
```bash
rc 6 1500 # neutral roll
rc 7 1300 # pitch down
rc 8 1500 # neutral yaw
```

#### 3c. Open a heartbeat terminal
```bash
ros2 launch mavros apm.launch fcu_url:=udp://:14552@localhost:14552
```


# How to wipe out all the docker images (mac)

* osascript -e 'quit app "Docker Desktop"'

### Paste all this

* docker stop $(docker ps -aq) 2>/dev/null || true
* docker rm -f $(docker ps -aq) 2>/dev/null || true
* docker rmi -f $(docker images -aq) 2>/dev/null || true
* docker volume rm $(docker volume ls -q) 2>/dev/null || true
* docker network rm $(docker network ls -q) 2>/dev/null || true

### Vhen go nuklear

* docker system prune -a --volumes -f

### Verify

* docker ps -a
* docker images
* docker volume ls
* docker network ls

### Reset docker and clean leftovers

* docker builder prune -a -f
* docker buildx prune -a -f

