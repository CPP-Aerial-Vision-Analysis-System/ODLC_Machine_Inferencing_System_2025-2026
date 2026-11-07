<!-- PURPOSE: the step-by-step guide to swapping the three big pieces of the digital twin: the ArduPilot autopilot (another version, a source build, the real drone's parameters, an autopilot on another computer), the Jetson code (another copy or branch of the team's ROS 2 code) and the detection model (other YOLO weights, a TensorRT engine, the answer key, another detector node). For each swap: the copy-paste commands, how to know it worked, how to go back, and the common errors with their fixes. Written so a 5th grader can follow it. -->

# Swap a piece of the digital twin: ArduPilot, the Jetson code, the detection model

The simulator is like a toy drone made of three big pieces you can pull out and replace:

| Piece | What it is on the real drone | What runs by default in the simulator |
|---|---|---|
| **1. The autopilot** | the Pixhawk flight controller with **ArduCopter 4.6.3** | ArduCopter 4.6.3 SITL (the same firmware, as a Linux program): `~/sitl/arducopter` |
| **2. The Jetson code** | the team's ROS 2 code on the **Jetson Orin Nano** (`./launcher.sh` + `tracker.launch.xml`) | the simulator's own copy of the team code: `~/ODLC_Machine_Inferencing_System_2025-2026` |
| **3. The detection model** | YOLO weights used by the team's detector `new_od` | `~/.astra_sim/models/yolo26m.pt` |

Each swap is **one command**. You can always go back by running the same command **without** the swap option.
Nothing in the team's code is changed by a swap.

The words you will see:

* **SITL** = "software in the loop": the real ArduCopter firmware compiled as a normal Linux program.
* **profile** = a small YAML file that remembers a swap, so you do not have to type it every time
  ([PLUGINS.md](PLUGINS.md) explains every setting).
* **run folder** = `~/.astra_sim/runs/<date>-<time>_<world>/`: everything one flight wrote.
* **dry run** = `--dry-run`: check everything and print the plan, but start nothing. Always try it first.

## Before you start (once per terminal)

Open the Ubuntu terminal (Windows: Start menu > "Ubuntu 22.04") and type:

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026
source simulation/setup/sim_env.sh
```

**How do I know it worked?** `ros2 pkg prefix sim_plugins` prints a folder (not an error).

Did you change files on Windows? Copy them into Ubuntu and rebuild first (this takes 1-3 minutes):

```bash
bash "/mnt/d/Projects ext/SUAS/gazebo/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/03_build.sh"
```

(Use your own Windows folder; the quotes matter because the folder name has a space.)

## Every swap in one line

| I want to fly with ... | Command (add `--dry-run` to only check) | Back to the default |
|---|---|---|
| another ArduCopter version | `bash simulation/scripts/demo.sh runway --sitl-version 4.7.1` | leave out `--sitl-version` |
| my own ArduCopter build | `bash simulation/scripts/demo.sh runway --sitl-source ~/ardupilot` | leave out `--sitl-source` |
| any arducopter program | `bash simulation/scripts/demo.sh runway --sitl-bin ~/my/arducopter` | leave out `--sitl-bin` |
| the real drone's parameters | `bash simulation/scripts/demo.sh runway --param-file ~/params/astra.param` | leave out `--param-file` |
| an autopilot on another computer | `bash simulation/scripts/demo.sh --profile ./my_external.yaml` | leave out `--profile` |
| a teammate's copy of the Jetson code | `bash simulation/scripts/demo.sh runway --team-repo ~/astra_fork` | leave out `--team-repo` |
| other YOLO weights | `bash simulation/scripts/demo.sh runway --model ~/.astra_sim/models/my_detector.pt` | leave out `--model` |
| the answer key instead of YOLO | `bash simulation/scripts/demo.sh runway --detector truth` | leave out `--detector` |
| a completely different detector | `bash simulation/scripts/demo.sh runway --profile example_detector` | leave out `--profile` |

Swaps can be combined, for example `--team-repo ~/astra_fork --model ~/.astra_sim/models/my_detector.pt`.
The same settings work in a profile file (`--profile my_swaps.yaml`), and `start_sim.sh` takes `--profile` too.

## How to know which pieces really flew (every run)

Every `demo.sh` run writes three small files into its run folder, `run_meta/`:

| File | What it says | Where the answer comes from |
|---|---|---|
| `run_meta/autopilot.json` | which ArduCopter really flew: `reported.version` (e.g. `4.6.3`), `reported.git`, the program's path, size and SHA-256, every `--defaults` parameter file with its SHA-256 | the autopilot's OWN boot message (`AP: ArduCopter V4.6.3 (3fc7011a)` in `sim.log`), the MAVLink message `AUTOPILOT_VERSION`, and the running program |
| `run_meta/detector.json` | which detector flew: `kind` (`real`, `truth`, `custom`) and for the team's `new_od` the model file's `name`, `path`, `size_mb`, `sha256`, `format`, and the result of the load test | `sim_plugins model_check` (it loads the model once before the flight) |
| `run_meta/team_stack.json` | which copy of the Jetson code flew: the folder, its git `commit` / `branch` / `dirty`, the SHA-256 of its `launcher.sh`, `TRACKER_CMD` and `FCU_URL` | the folder demo.sh started `./launcher.sh sim` in |

Show them for the newest run:

```bash
R=$(ls -td ~/.astra_sim/runs/*/ | head -1); echo "$R"; cat "$R"run_meta/*.json
```

The run summary of the Runs & logs tools reads the same files. Every file has `"schema":
"astra.run_meta.<autopilot|detector|team_stack>.v1"`; new keys may be added later, old keys keep their meaning.

The simulator's Linux copy has no `.git` folder, so `team_stack.json` cannot see its commit. To record the commit
of the Windows checkout it was copied from, set `ASTRA_SOURCE_REPO` before `demo.sh`, for example
`export ASTRA_SOURCE_REPO="/mnt/d/Projects ext/SUAS/gazebo/ODLC_Machine_Inferencing_System_2025-2026"`.

---

## Swap 1: the autopilot (ArduPilot SITL)

The autopilot flies the drone. The team code never talks to it directly: only through MAVROS (MAVLink). So any
ArduCopter that flies `GUIDED`, `AUTO` and `RTL` missions works without touching the team code.
Full background: [ARDUPILOT.md](ARDUPILOT.md).

### 1a. Another ArduCopter version

1. See which versions exist (needs internet):

   ```bash
   bash simulation/setup/fetch_sitl.sh --list
   ```

2. Download one (it goes to `~/sitl/versions/4.7.1/`; the default 4.6.3 is never touched):

   ```bash
   bash simulation/setup/fetch_sitl.sh 4.7.1
   ```

3. Check, then fly:

   ```bash
   bash simulation/scripts/demo.sh runway --sitl-version 4.7.1 --dry-run
   bash simulation/scripts/demo.sh runway --sitl-version 4.7.1
   ```

   The same as a profile: `bash simulation/scripts/demo.sh --profile ardupilot_version`.

**How do I know it worked?** The dry run's plan has the line
`autopilot  SITL /home/<you>/sitl/versions/4.7.1/arducopter = ArduCopter 4.7.1 (<git>); run folder ...`. After the
flight, `run_meta/autopilot.json` says `"version": "4.7.1"` under `reported`.

### 1b. Your own ArduCopter build (from source)

1. Build SITL once inside your ArduPilot folder (ArduPilot's own instructions install the tools first):

   ```bash
   git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
   cd ~/ardupilot && ./waf configure --board sitl && ./waf copter
   cd ~/ODLC_Machine_Inferencing_System_2025-2026
   ```

2. Fly it (the simulator takes `~/ardupilot/build/sitl/bin/arducopter` and the parameter files of that tree):

   ```bash
   bash simulation/scripts/demo.sh runway --sitl-source ~/ardupilot --dry-run
   bash simulation/scripts/demo.sh runway --sitl-source ~/ardupilot
   ```

   Only a program, no tree? `--sitl-bin ~/path/to/arducopter`.

**How do I know it worked?** The plan's autopilot line names `.../ardupilot/build/sitl/bin/arducopter` and the
version inside that program (`= ArduCopter 4.x.y (<git>)`); `run_meta/autopilot.json` shows the same version and the
git hash of your build.

### 1c. The real drone's parameter file

1. In Mission Planner (connected to the real Pixhawk): **CONFIG > Full Parameter List > Save to file**. Save it
   as `astra.param`.
2. Copy it into Ubuntu, into a folder without spaces (Windows' D: drive is `/mnt/d/` inside Ubuntu):

   ```bash
   mkdir -p ~/params && cp "/mnt/d/<where you saved it>/astra.param" ~/params/
   ```

3. Check, then fly:

   ```bash
   bash simulation/scripts/demo.sh runway --param-file ~/params/astra.param --dry-run
   bash simulation/scripts/demo.sh runway --param-file ~/params/astra.param
   ```

   The same as a profile: copy [real_drone_params.yaml](../../simulation/profiles/real_drone_params.yaml) and
   change its `param_files` line.

What happens: the simulator first loads its own settings (stock Copter, Gazebo Iris, `astra_sitl.parm`), then your
file on top. The **safe filter** drops the lines that describe the PHYSICAL drone (sensor calibration, motors,
serial ports, battery, frame, the payload servos), because they would break the simulated drone. It keeps the
lines about HOW the drone flies the mission (`WPNAV_SPEED`, `RTL_ALT`, failsafes ...). The plan prints how many
lines were kept, dropped and why.

Every line of the file must be `NAME,VALUE` (Mission Planner), `NAME VALUE` or the QGroundControl format. A line
that is not (a sentence, a value that is not a number, a value longer than 32 characters) **stops the start** and
the message names the line numbers: a bad line usually means the wrong file. The copy SITL reads keeps every line
far below 90 characters (ArduPilot reads parameter files in 100-byte pieces and silently breaks longer lines).

**How do I know it worked?** Under the autopilot line the plan shows for example
`.. astra.param: 10 parameter(s) used, 17 dropped by the filter, 0 unreadable line(s)` (the example file gives
exactly these numbers); `run_meta/autopilot.json` lists the cleaned copy under `process.defaults` with its SHA-256.

**Never commit** the real drone's parameter file (the repo is public): keep it in `~/params/`.

### 1d. An autopilot on another computer or container

Copy [external_autopilot.yaml](../../simulation/profiles/external_autopilot.yaml), change the two addresses, start
ArduCopter SITL on the other computer, then:

```bash
python3 -m sim_plugins.profile_tool plan --profile ./my_external.yaml --check
bash simulation/scripts/demo.sh --profile ./my_external.yaml
```

The commands the other computer needs are in [ARDUPILOT.md](ARDUPILOT.md#an-autopilot-somewhere-else).
`run_meta/autopilot.json` then has `"kind": "external"` and the version from `AUTOPILOT_VERSION`.

### Back to the default autopilot

Run `demo.sh` without `--sitl-version`, `--sitl-source`, `--sitl-bin`, `--param-file` or `--profile`. Each
autopilot choice keeps its OWN SITL state folder (the plan prints it: `~/.astra_sim/sitl_I0` for the default,
`sitl_I0_v4.7.1`, `sitl_I0_params-<hash>` ...), so a swap never leaves its parameters behind in the default.
Parameters changed DURING a run (for example from Mission Planner) are saved in that folder's `eeprom.bin` and come
back next time; clear them once with `bash simulation/scripts/demo.sh runway --set autopilot.wipe=true`.
**How do I know?** `run_meta/autopilot.json` says `"version": "4.6.3"` again.

### Autopilot: common errors and fixes

| The message says | Why | Fix |
|---|---|---|
| `ArduCopter 4.7.1 is not downloaded (... missing): that version was never fetched on this computer` | the version folder does not exist yet | `bash simulation/setup/fetch_sitl.sh 4.7.1` (internet), or see the list with `--list` |
| `arducopter program not found or not executable: ... does not exist` | a typo, or the file is still on Windows | `ls -l` the path; a source build puts it in `<tree>/build/sitl/bin/arducopter` |
| `... exists but may not be run` | the file lost its "may run" flag (copied from Windows) | `chmod +x <file>` |
| `... has no built SITL (... missing): the tree is not built yet` | `--sitl-source` points at a tree that was never built | `cd <tree> && ./waf configure --board sitl && ./waf copter` |
| `does not look like a Linux program (no ELF header)` | a Windows `.exe` or a broken download | download again; SITL programs only run on x86_64 Linux (a Jetson needs a source build) |
| `the folder says ArduCopter 4.7.1, but the program says 4.6.3` | someone copied another program into that folder | `bash simulation/setup/fetch_sitl.sh 4.7.1 --force` |
| `parameter file ... has N unreadable line(s): line 7: "..." (...)` | not a parameter file, or a typo | fix or delete those lines (or put `#` in front), or save the file again from Mission Planner |
| `parameter file not found: ...` | wrong path (a path in a profile starts at the profile's folder) | `ls -l` it, give the full path |
| `the simulated autopilot never became ready` | the new firmware does not arm, or a parameter from your file blocks arming | read the end of `sim.log` in the run folder; try without `--param-file`; `--set autopilot.wipe=true` |

---

## Swap 2: the Jetson code

"The Jetson code" is everything the real Jetson Orin Nano runs: `./launcher.sh` starts **MAVROS** and then the
six team nodes of `ros2_ws/src/main/launch/tracker.launch.xml`. You can fly **another copy** of it in the simulator:
a teammate's fork, another branch, an older version to compare, or (later) this year's code.

### What must stay the same (the rules)

The simulator touches the Jetson code only through **two doors**, and it watches a few names. Keep these and any
copy of the code runs in the simulator unchanged:

1. **Door 1, MAVLink through MAVROS**: the same MAVROS topics and services (`/mavros/state`, `/mavros/statustext/*`,
   `/mavros/mission/*`, `/mavros/set_mode`, `/mavros/cmd/command`, ...). The team nodes never talk MAVLink directly.
2. **Door 2, the SIYI camera protocol** at `192.168.144.25` (SDK on UDP 37260, photos over HTTP port 82). The
   simulator's camera emulator answers there like the real camera.
3. **The node names** the simulator waits for: `waypoint_manager`, `gps_mavros_service_node`,
   `mission_camera_trigger`, `siyi`, `new_od`, `main_controller` (and `mavros`).
4. **The status texts** the simulator reads to follow the mission: `Mission: N DigiCamCtrl`, `Camera trigger STARTED`,
   `Captured <res> image #N`, `Zoom set to Xx`, `Detected <class>`, `Reached last WP` / `Holding to finish`,
   `Targets: tent@N -> person@M. Diverting.`, `Resumed AUTO -> WP N`, `Over <class>`, `Servo C -> PWM`, `RTL`.
   (Changing one does not break the flight, only the simulator's phase display and score.)
5. **The photo folder and names**: `ros2_ws/src/video_cam/mapping_photos/"<lat> , <lon>.jpg"`.
6. **`launcher.sh` reads `TRACKER_CMD` and `FCU_URL`** (the simulator's only hook), and the `interfaces` messages
   (`ImageResult`, `AddWaypoint`) stay the same.

The full list with types: `python3 -m sim_plugins.profile_tool contract --slot team_stack` (also in
[PLUGINS.md](PLUGINS.md#team_stack)). The checks below test all of it for you.

### 2a. A teammate's fork or another branch

1. Get the code into Ubuntu, in a folder **without spaces**, and build it:

   ```bash
   git clone -b <branch name> <the fork's GitHub URL> ~/astra_fork
   source /opt/ros/humble/setup.bash
   cd ~/astra_fork/ros2_ws && colcon build --symlink-install --packages-ignore ultralytics_ros
   cd ~/ODLC_Machine_Inferencing_System_2025-2026 && source simulation/setup/sim_env.sh
   ```

2. Check it (this looks at the folder: `launcher.sh`, the build, `tracker.launch.xml`, the node names and status
   texts in its sources):

   ```bash
   python3 -m sim_plugins.profile_tool plan --set run.team_repo=~/astra_fork --check
   ```

3. Fly it:

   ```bash
   bash simulation/scripts/demo.sh runway --team-repo ~/astra_fork --dry-run
   bash simulation/scripts/demo.sh runway --team-repo ~/astra_fork
   ```

   The same as a profile: copy [team_fork.yaml](../../simulation/profiles/team_fork.yaml) and change `team_repo`.

What happens: demo.sh goes into `~/astra_fork` and starts **its** `./launcher.sh sim`, exactly like the Jetson does.
A small helper, `ros2_ws/src/simulation/sim_plugins/scripts/team_overlay.sh`, makes the simulator's launch helper
(`sim_bringup/team_stack.launch.py`) visible to it while the fork's own team packages win over the simulator's
copies. MAVROS is pointed at the simulated Pixhawk (`FCU_URL=tcp://127.0.0.1:5763`, like the Jetson on TELEM2).
That copy saves its photos in **its own** `ros2_ws/src/video_cam/mapping_photos/`, so demo.sh empties, watches and
scores that folder. (START OVER restarts the simulator's own copy, not the fork: start a new `demo.sh` instead.)

**How do I know it worked?**

* `team_stack.log` in the run folder starts with lines like
  `[team_overlay] main comes from /home/<you>/astra_fork/ros2_ws/install/main`;
* `run_meta/team_stack.json` says `"repo": "/home/<you>/astra_fork"` and its git commit;
* while it flies, `ros2 run sim_plugins check_slot --slot team_stack` (second terminal) ends with `RESULT: PASS`.

### 2b. Replace only ONE node (a new detector or a new mission brain)

If the copy renames or replaces one node (for example a new detector called `astra_detector`), do not fork the whole
launch: tell the simulator which node replaces which, with a profile `detector: {kind: custom, custom: {...}}` or
`mission_brain: {kind: custom, ...}` ([PLUGINS.md](PLUGINS.md#replace-the-detector-or-mission-brain)). The
simulator starts the team's flight launch without the replaced node and waits for your node's name instead.

### 2c. Check a running Jetson code

Start a run with `--keep` (or use the run that is flying), then in a second terminal:

```bash
source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh
ros2 run sim_plugins check_slot --slot team_stack
ros2 run sim_plugins check_slot --slot team_stack --skip-node new_od --add-node astra_detector   # one node replaced
```

**How do I know it worked?** The last line says `RESULT: PASS (... check(s) passed, ... warning(s))` and the exit
code is 0. A line starting with `XX` names what is missing and the fix; `!!` lines are warnings.

### 2d. The Jetson code on the real Jetson (or the virtual Jetson container)

The team code can also run **on the real Jetson Orin Nano** while the simulator runs on this PC ("Jetson in the
loop"), reached over its USB-C cable (`192.168.55.1`) **or** plain SSH over Ethernet / Wi-Fi / a LAN (a name such
as `jetson.local` or an IP address), or in the same container on this PC with no board ("virtual Jetson"). The
Jetson stream is building that kit now; its design is [HIL_DESIGN.md](../jetson/HIL_DESIGN.md). In that design
(not built yet, so check `--help` once it exists):

* you log in yourself: the kit's `jetson.sh login` opens a terminal window where YOU type the Jetson password and
  leave the window open; the other kit commands reuse that connection (no password is typed, stored or passed by
  the tools; closing the window ends the access);
* then the kit's `jetson.sh check` (in the Jetson kit's folder) and `jetson.sh deploy`, then
  `bash simulation/scripts/demo.sh runway --jetson usb` (or `--jetson ssh --jetson-host HOST`, or
  `--jetson virtual`);
* `deploy` copies the team code from the checkout on this PC into a container image on the Jetson, so **the code
  that flies there is the code you deployed**: to fly a fork or a branch on the Jetson, deploy from that checkout,
  and deploy again after every change;
* the same rules as above decide whether that copy works (two doors, node names, status texts); the PC opens every
  connection (SSH), because WSL2 sits behind a NAT and cannot accept new connections from the Jetson;
* `--detector truth` or a custom detector cannot be combined with `--jetson` (the detector runs on the Jetson).

`run.team_stack: external` (profile [docker_split.yaml](../../simulation/profiles/docker_split.yaml)) is the
general version of the same idea: demo.sh starts only the simulator and waits for the team nodes to appear, where
ever they run; `check_slot --slot team_stack` checks them the same way.

### Back to the default Jetson code

Run `demo.sh` without `--team-repo` (and without a profile that sets `run.team_repo`).
**How do I know?** `run_meta/team_stack.json` says `"repo": "/home/<you>/ODLC_Machine_Inferencing_System_2025-2026"`.

### Jetson code: common errors and fixes

| The message says | Why | Fix |
|---|---|---|
| `the team code folder ... does not exist` | not cloned yet, or a typo | `git clone <url> ~/astra_fork` |
| `... contains a space` | colcon and `./launcher.sh` do not work in folders with spaces | clone it into your Linux home, e.g. `~/astra_fork` |
| `... has no launcher.sh` | you gave a sub-folder | give the folder that holds `launcher.sh` and `ros2_ws/` |
| `launcher.sh does not read the environment variable TRACKER_CMD (an older launcher)` | the fork's launcher is older than the simulator hook | `cp ~/ODLC_Machine_Inferencing_System_2025-2026/launcher.sh ~/astra_fork/launcher.sh` |
| `.../ros2_ws is not built (no install/setup.bash)` or `install has no video_cam` | the fork was never (fully) built | `cd ~/astra_fork/ros2_ws && colcon build --symlink-install --packages-ignore ultralytics_ros` |
| `the node name "main_controller" was not found` (warning) | the fork renamed a node | keep the name, or use a custom slot with `node_name` (2b); otherwise demo.sh waits 5 minutes and stops |
| `the status text "RTL" was not found` (warning) | the fork changed a text | the flight still works; the phase display / score miss that step |
| `[team_overlay] WARNING: the team package main does not come from ...` | the fork's build is missing `main` | build the fork (see above) |
| `the team stack did not get ready` | a node crashed or never started | read `team_stack.log` in the run folder (look for `process has died`) |

---

## Swap 3: the detection model

The team's detector `new_od` (package `detection`) reads every new photo, cuts it into 640 px tiles, runs YOLO and
publishes one `/image_detection` message per photo. You can change the **weights** it loads, or replace the whole
**detector**.

Which YOLO classes count (the team's `detection_processor.py` `CLASS_MAP`): `person`, `mannequin`, `doll` become a
**person** (class id `"0"`); `umbrella` and `kite` become a **tent** (class id `"1"`). A model whose class is
literally called `tent` is ignored by the team code: name that class `umbrella` when you export, or change
`CLASS_MAP` in the team code (a team-code change: test it on the real drone too).

### 3a. Other YOLO weights (a .pt file)

1. Copy your weights into Ubuntu (a folder without spaces, NOT the repo: model files are never committed):

   ```bash
   mkdir -p ~/.astra_sim/models && cp "/mnt/d/<where it is>/my_detector.pt" ~/.astra_sim/models/
   ```

2. Check the file and load it once (5-15 seconds):

   ```bash
   python3 -m sim_plugins.model_check ~/.astra_sim/models/my_detector.pt --load
   ```

   **How do I know it worked?** Lines starting with ` OK` and ` ..`, exit code 0. On the team laptop the stock
   model gave `loaded in 3.8 s on cuda:0 (NVIDIA GeForce RTX 4060 Laptop GPU): 80 class(es), 19 ms per 640 px tile,
   ~0.3 s per 2560x1440 photo (16 tiles)` and `classes the team code uses: person <- person; tent <- umbrella, kite`.
   ` !!` lines are warnings (read them), ` XX` lines are errors (exit code 3) and each says the fix.

3. Fly it:

   ```bash
   bash simulation/scripts/demo.sh runway --model ~/.astra_sim/models/my_detector.pt --dry-run
   bash simulation/scripts/demo.sh runway --model ~/.astra_sim/models/my_detector.pt
   ```

   The same as a profile: copy [custom_model.yaml](../../simulation/profiles/custom_model.yaml) and change
   `model_path`. Or for one terminal: `export ASTRA_DETECTOR_MODEL=~/.astra_sim/models/my_detector.pt`.

**How do I know it worked?**

* before the simulator starts, demo.sh prints the model check (` OK model my_detector.pt: pytorch, ... MB,
  sha256 ...`) and stops right there if the model cannot be used;
* `team_stack.log` says `[team_stack] new_od model_path := /home/<you>/.astra_sim/models/my_detector.pt`;
* `run_meta/detector.json` has `"name": "my_detector.pt"` and its `sha256`; compare it with
  `sha256sum ~/.astra_sim/models/my_detector.pt` (the same 64 characters = the same file);
* `score_report.md` shows what this model found.

### 3b. A TensorRT .engine (the Jetson's fast format)

A `.engine` file only runs where **TensorRT** is installed and only on the GPU and TensorRT version it was built for
(normally the Jetson Orin Nano itself: an engine made on one board, or for another JetPack, does not load on
another). This PC has no TensorRT, so:

* `--model my_detector.engine` is **refused** here with that reason, unless a `my_detector.pt` sits next to it
  (then it warns: new_od would fall back to the `.pt`, so the `.pt` flies, not the engine);
* on the Jetson you do not have to make the engine by hand: give new_od the `.pt`; when TensorRT is there and no
  `.engine` with the same name exists, new_od itself (`detection/model_manager.py`) builds `my_detector.engine`
  next to the `.pt` with the team's own settings on its first start (this takes minutes), and loads the engine
  from then on (`model_format` auto loads an existing `.engine` FIRST);
* in the Jetson kit's design, `jetson.sh deploy --model my_detector.pt` copies the weights to the Jetson and builds
  the engine before the flight (`warmup`), so the first flight does not wait for it;
* the team's batched detector path needs the engine (`use_batched_inference`, profile key `detector.batched`):
  Jetson only.

So on this PC fly the `.pt`, and test the `.engine` on the Jetson (2d). After a JetPack update, delete the old
`.engine` so it is built again.

### 3c. The answer key instead of YOLO (truth detector)

```bash
bash simulation/scripts/demo.sh runway --detector truth
```

The simulator's `truth_detector` knows where the hiker and the tent are and reports them like `new_od` would. Use
it to test the mission logic (waypoints, drops, RTL) without any model. `run_meta/detector.json` says
`"kind": "truth"`; the score report's detector line says `TRUTH STAND-IN (truth_detector, NOT the AI model)`.

### 3d. A completely different detector node

Copy the template [example_detector](../../ros2_ws/src/simulation/sim_plugin_examples/example_detector/README.md)
(a small colour-blob detector), rename it, put your own code inside, and select it with a profile like
[example_detector.yaml](../../simulation/profiles/example_detector.yaml):

```bash
bash simulation/scripts/demo.sh runway --profile example_detector --dry-run
bash simulation/scripts/demo.sh runway --profile example_detector
ros2 run sim_plugins check_slot --slot detector --take-photo        # second terminal, while it flies
```

Your detector must, for **every** new photo in `ros2_ws/src/video_cam/mapping_photos/` (also when it finds
nothing), publish exactly one `interfaces/msg/ImageResult` on `/image_detection` with **RELIABLE** QoS, depth 10;
`image_name` = the photo's file name; per box `class_id` `"0"` (person) or `"1"` (tent), a `score`, the box centre in
photo pixels, and the latitude / longitude from the file name. The full contract:
`python3 -m sim_plugins.profile_tool contract --slot detector` ([PLUGINS.md](PLUGINS.md#detector)).

### No GPU? (CPU fallback)

When CUDA does not work (for example an old Windows NVIDIA driver in WSL), YOLO runs on the CPU. The load test then
warns with the **measured** time per tile and the expected time per photo. On the team laptop the stock model took
333 ms per tile on the CPU (`--device cpu`, ~5.3 s per photo) against 19-24 ms on the RTX 4060 (~0.3 s per photo):
about 14 times slower (other computers: typically 10-30 times, an estimate). The camera takes one photo per second,
so `new_od` falls behind;
`main_controller` waits at most 90 s at the last waypoint and then flies with what was checked. Fix: update the
Windows NVIDIA driver, then `nvidia-smi` inside Ubuntu must show the RTX card.

### Back to the default model

Run `demo.sh` without `--model` (and `unset ASTRA_DETECTOR_MODEL` if you exported it).
**How do I know?** `run_meta/detector.json` says `"name": "yolo26m.pt"`.

### Detection model: common errors and fixes

| The message says | Why | Fix |
|---|---|---|
| `the model file ... does not exist` | typo, or still on Windows | copy it into `~/.astra_sim/models/` (step 1) |
| `the model path "..." contains a space` | `./launcher.sh` splits the path at the space | move it to a folder without spaces |
| `... is empty (0 bytes)` / `does not look like PyTorch weights (it starts with a web page (HTML))` | a failed or wrong download | download / export again; check with `model_check --load` |
| `the extension ".h5" is not a model format new_od loads` | another framework's file | export to `.pt` with Ultralytics |
| `... is a TensorRT engine, but this computer has no TensorRT` | engines only run on the Jetson | give the `.pt` here; use the `.engine` on the Jetson (3b) |
| `... .engine sits next to ... .pt: new_od loads the .engine FIRST` (warning) | an engine with the same name is in that folder | move the engine away, or give the `.pt` from its own folder |
| `the model ... did not load: ...` | not a YOLO detection model, or made with a much newer/older Ultralytics | export again with Ultralytics 8.4.x as `.pt` |
| `the model has no class the team code turns into a PERSON / TENT` (warning) | other class names | see the class rule at the top of Swap 3 |
| `the detector runs on the CPU (...)` (warning) | no CUDA | see "No GPU?" above |
| `the detector weights ... did not pass the load test; nothing was started` | the load test failed (reasons above it) | fix the model, or fly `--detector truth` |

---

## What was proven, and how

Unit tests check every command and error on this page: `ros2_ws/src/simulation/sim_plugins/test/`
(`test_model_check.py`, `test_run_meta.py`, `test_team_repo.py`, `test_team_overlay.py`, `test_plan.py`,
`test_params.py`, `test_autopilot.py`, `test_check_slot.py`, `test_profiles.py`). On 2026-10-31 they passed on
Windows (258 tests, 56 skipped because they need ROS or bash) and in Ubuntu-22.04 with ROS (315 tests).

| Swap | Proven by a full simulator flight | Checked by dry run + tests only |
|---|---|---|
| other YOLO weights (`--model`) | **yes**: run `20261031-112518_runway` flew a renamed copy of the weights (`swap_test_yolo26m_copy.pt`): demo.sh loaded it before the start (19 ms per tile on the RTX 4060), `team_stack.log` shows `new_od model_path := .../swap_test_yolo26m_copy.pt`, 2 of 2 targets found, drops 4.6 m (tent) and 1.4 m (hiker), `run_meta/detector.json` holds its name and SHA-256 | |
| another copy of the Jetson code (`--team-repo`) | **the wiring**: the same run started the team code through `--team-repo` and `team_overlay.sh` (it logged where each package came from); it pointed at the simulator's own copy, so a DIFFERENT fork winning over the simulator's copy is proven only by `test_team_overlay.py` (fake workspaces, real `ros2 pkg prefix`) | a real fork: not flown yet |
| "what ran" records (`run_meta/*.json`) | **yes**: the same run and two other default demo runs that morning (`20261031-110240_runway`, `20261031-111300_runway`) wrote all three files; `autopilot.json` has the banner `ArduCopter V4.6.3 (3fc7011a)` and the `AUTOPILOT_VERSION` answer | |
| another ArduCopter version, a source build, any program | | dry run: the plan and the clear errors (4.7.1 not downloaded on this PC, a missing program, an unbuilt tree) |
| the real drone's parameter file | | dry run: the example file's plan; a file with bad lines is refused with the line numbers |
| an autopilot on another computer | | dry run of `external_autopilot` |
| the answer key, the example detector | | dry runs only on 2026-10-31 (the example detector still needs its first recorded flight) |
| a TensorRT `.engine` | | refused on this PC with the reason (no TensorRT); the Jetson part is in the Jetson kit's design |
| the Jetson code on the real Jetson | | not built yet (Jetson kit, `docs/jetson/HIL_DESIGN.md`) |
