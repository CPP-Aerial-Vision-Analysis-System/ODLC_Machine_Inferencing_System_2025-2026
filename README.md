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

---

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