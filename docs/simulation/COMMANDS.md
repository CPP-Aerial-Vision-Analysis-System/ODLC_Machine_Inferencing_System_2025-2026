<!--
docs/simulation/COMMANDS.md -- PURPOSE: every command a teammate needs for the ASTRA digital twin, in one place, with
copy-paste examples and "how do I know it worked": run a demo (automatic, Mission Planner mode, kept open, another
world, a profile), start only the simulator, stop, start over, take a photo, check / load / fly / make a mission plan,
status, logs, setup and the Jetson commands. The front door is simulation/scripts/astra.sh, which only calls the real
scripts. Every error message and its fix is in ERRORS.md next to this file.
-->

# Every command in one place

You only have to remember **one** command: `astra.sh`. The word after it says what you want, and it runs the
script that does that job (it never does the work itself, so every option of that script works through it too).

```bash
bash simulation/scripts/astra.sh help            # the list of words
bash simulation/scripts/astra.sh help demo       # the full help of one word (here: demo.sh --help)
```

Every script also answers `--help` by itself (`bash simulation/scripts/mission.sh --help`). When something goes
wrong, the message says **what** failed, the **likely cause** and the **fix** (the exact command), and starts with
the program's name, for example `XX [mission] ERROR: ...`. Look the message up in [ERRORS.md](ERRORS.md).

## Before you start: open the right terminal

All commands run in **Ubuntu 22.04** (on Windows: the WSL distro `Ubuntu-22.04`).

1. Windows: open PowerShell and type `wsl -d Ubuntu-22.04`. Linux: open a terminal.
2. Go to the repo copy the simulator runs from and load its settings:

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026
source simulation/setup/sim_env.sh
```

**How do I know it worked:** `echo $ASTRA_WS_BUILT` prints `yes`. It prints `no`? Build first:
[Set up and build](#set-up-and-build). (The scripts load the settings themselves; `source` is only needed for
`ros2 ...` commands you type by hand.)

The commands below are written from that folder (`bash simulation/scripts/...`). From the Windows checkout
(`/mnt/d/...`) they work the same way: they load the built Linux copy's settings and programs.

## The front door: astra.sh

| Word | What it does | It runs |
|---|---|---|
| `demo [WORLD] [options]` | The full automatic search demo: start, upload, arm, fly, score, close by itself | `demo.sh` |
| `manual [WORLD] [options]` | Mission Planner mode: start everything, then **you** load, arm and fly; stays open | `demo.sh WORLD --manual` |
| `keep [WORLD] [options]` | Automatic demo that stays open after the landing | `demo.sh WORLD --keep` |
| `sim [WORLD] [options]` | Only the simulator, without the team's flight code | `start_sim.sh` |
| `stop [options]` | Stop the simulator and everything it started | `stop_sim.sh` |
| `over [options]` | START OVER: fly again without closing the windows | `start_over.sh` |
| `mission check FILE` | Explain a mission file and check that the team code can fly it | `mission.sh` |
| `mission load FILE\|WORLD` | Check it, upload it to the running simulator, make MAVROS read it again | `mission.sh` |
| `mission fly [FILE\|WORLD]` | (Load FILE first, then) arm and start the mission (AUTO) | `mission.sh` |
| `mission make WORLD` | Plan a new survey mission for a world | `mission.sh` |
| `mission list` | The mission files you can load | `mission.sh` |
| `photo [options]` | Take one photo with the simulated camera | `take_photo.sh` |
| `status [options]` | What is running: processes, ports, camera, autopilot, ROS nodes | `status.sh` |
| `logs [list\|summary\|errors\|tail\|csv\|bundle ...]` | The run folders, their logs, CSV files and summaries | `logs.sh` |
| `mp start\|stop\|status` | Open / close Mission Planner on Windows (also installs the ASTRA plugin) | `mission_planner.sh` |
| `build [options]` | Copy the repo to Linux and build it; run it after every change | `setup/03_build.sh` |
| `check-ros` | Is ROS 2 ready in this terminal? | `setup/check_ros.sh` |
| `jetson WORD [options]` | The commands for the real drone's Jetson Orin Nano (USB cable or plain SSH) | `jetson.sh` (comes with the Jetson work) |

`fly` alone is short for `mission fly`; `start` is the same as `sim`, `start-over` the same as `over`.

## Run the simulator

Only **one** simulator runs at a time on a PC. A new one stops the old one first.

### The automatic demo

```bash
bash simulation/scripts/astra.sh demo                      # runway world, about 6-8 minutes, closes by itself
bash simulation/scripts/astra.sh demo suas_field_real           # the SUAS 2026 field on real terrain (longer)
bash simulation/scripts/astra.sh demo runway --detector truth   # the answer-key detector instead of the team model
bash simulation/scripts/astra.sh demo runway --camera a8mini    # A8 mini profile, 4K photos
bash simulation/scripts/astra.sh demo runway --headless         # no windows (tests, servers)
```

**How do I know it worked:** the terminal prints `STEP 1/6` ... `STEP 6/6`, then the phases (`TAKEOFF`, `SEARCH`,
`HOLD`, `TARGETS`, `DROP`, `RTL`, `LANDED`), then `OK The drone has landed: mission complete.` and the score report.
Everything of the run is in `~/.astra_sim/runs/<date>-<time>_<world>/` (`score_report.md`, `phases.jsonl`,
`sim.log`, `team_stack.log`, photos).

### Mission Planner mode (fly it yourself)

```bash
bash simulation/scripts/astra.sh manual                    # = demo.sh runway --manual
```

It starts the simulator, the windows and the team's flight code, then **waits**: nothing is uploaded, armed or
started. Do it in Mission Planner (the steps are printed; see [MISSION_PLANNER](MISSION_PLANNER.md)), or from a
second Ubuntu terminal with the mission commands below. It stays open after the landing: stop it with
`bash simulation/scripts/astra.sh stop`.

### Keep it open, another world, a profile

```bash
bash simulation/scripts/astra.sh keep                      # automatic, stays open after the landing
bash simulation/scripts/astra.sh demo suas_field_real --keep    # another world, kept open
bash simulation/scripts/astra.sh demo runway --profile truth_detector              # a plug-in profile
bash simulation/scripts/astra.sh demo runway --profile truth_detector --dry-run    # print the plan, start nothing
bash simulation/scripts/astra.sh demo runway --mission ~/.astra_sim/missions/my.waypoints   # another mission
```

Profiles are listed in [simulation/profiles/README.md](../../simulation/profiles/README.md) and explained in
[PLUGINS](PLUGINS.md). Every demo option: `bash simulation/scripts/astra.sh help demo`.

### Only the simulator

```bash
bash simulation/scripts/astra.sh sim runway                # Ctrl+C stops it
bash simulation/scripts/astra.sh sim runway --background   # give the terminal back; stop with astra.sh stop
```

Then start the team code by hand in a second terminal (the command is in
[simulation/README.md, "Start pieces by hand"](../../simulation/README.md#start-pieces-by-hand)).

### Stop everything, start over

```bash
bash simulation/scripts/astra.sh stop                      # stops only YOUR simulator session
bash simulation/scripts/astra.sh stop --all                # also older sessions of your user
bash simulation/scripts/astra.sh over --fly --follow       # land, reset the world and fly again (windows stay open)
```

**How do I know it worked:** `stop` ends with `The simulator is stopped.` (exit code 0). If a process survives it
says which one and how to stop it.

## Mission plans

A mission plan is the list of waypoints the autopilot flies. The team's flight code needs a certain shape
(AGENTS.md section 3.4): **home**, **TAKEOFF**, the search lanes with **DO_DIGICAM_CONTROL** after the first lane
start (photos start there), the last lane end (`buffer_wp`: photos stop), a **hold waypoint** (the drone waits
there for the detector), then **RETURN_TO_LAUNCH**. The SUAS 2026 rules want 150-400 ft (45.7-121.9 m) above the
ground. `mission.sh` checks all of that before anything is uploaded.

Files that work:

* **Mission Planner** `.waypoints` files (first line `QGC WPL 110`): FLIGHT PLAN > Save WP File.
* **QGroundControl** `.plan` files: simple items and survey patterns that QGroundControl saved with their
  waypoints. They are converted to `.waypoints` in `~/.astra_sim/missions/` before the upload.
* A Windows path works when you put it in quotes: `"C:\Users\me\Documents\Mission Planner\field.waypoints"`, and
  so does the Windows name of a file inside Ubuntu (what `mission make` prints for Mission Planner):
  `"\\wsl.localhost\Ubuntu-22.04\home\me\.astra_sim\missions\runway_survey_20261031-120000.waypoints"`.
* Mission Planner "Unicode" (UTF-16) files and files with Windows line ends (CRLF) work too.

### Check a mission (no simulator needed)

```bash
bash simulation/scripts/astra.sh mission check ~/Downloads/field.waypoints
bash simulation/scripts/astra.sh mission check "C:\Users\me\Documents\Mission Planner\field.waypoints"
bash simulation/scripts/astra.sh mission check runway                  # a world's own mission
bash simulation/scripts/astra.sh mission check field.waypoints --strict   # warnings count as errors (competition)
```

It prints every item (altitude in metres and feet, what the team code does there), the indices the team code will
compute (`takeoff_index`, photos start, `buffer_wp`, hold, `rtl_index`), the lane speed, the path length and a
rough flight time, then the **ERROR** and **WARNING** lines, each with the fix in Mission Planner.

**How do I know it worked:** the last line says `RESULT: OK, the team code can fly this mission` (exit code 0) or
`RESULT: NOT READY, N error(s)` (exit code 1). Exit code 3 = the file cannot be read as a mission; exit code 4 = the
checker itself failed (a bug: report it with the file).

Heights are checked too: a flight item at 5 m or lower above home (or below home) is an **ERROR** (the drone would
fly the search in the grass); below 20 m is "too low even for a simulator test"; 20 m up to 150 ft is only a SUAS
warning ("fine for a simulator test": the runway world flies at 30 m = 98 ft).

The checker is pure Python, so it also checks the **real drone's** mission files on any laptop, even Windows
without WSL:

```bash
python3 ros2_ws/src/simulation/sim_demo/sim_demo/mission_check.py field.waypoints     # Linux / macOS / WSL
python ros2_ws\src\simulation\sim_demo\sim_demo\mission_check.py field.waypoints      # Windows PowerShell
ros2 run sim_demo mission_check field.waypoints                                       # after a build
python3 ros2_ws/src/simulation/sim_demo/sim_demo/mission_check.py survey.plan --write-waypoints survey.waypoints
```

### Load and fly a mission (simulator running)

Window 1 starts the simulator in Mission Planner mode; window 2 loads and flies:

```bash
bash simulation/scripts/astra.sh manual                                     # window 1
bash simulation/scripts/astra.sh mission load "D:\missions\field.waypoints" # window 2: check + upload + MAVROS re-read
bash simulation/scripts/astra.sh mission fly --watch                        # window 2: arm + AUTO, print until landed
```

Or in one step: `bash simulation/scripts/astra.sh mission fly "D:\missions\field.waypoints" --watch`.

* `load` refuses a mission with errors; `--force` uploads it anyway (it warns what will go wrong).
* After the upload it asks MAVROS to read the mission again, so the team's `waypoint_manager` has the new indices
  (it prints `OK MAVROS re-read the mission`).
* `fly` without a file flies what is on the autopilot; it warns when you did not load a mission in this session.

**How do I know it worked:** `load` ends with `OK Mission loaded: <file>`; `fly` prints `ARMED`,
`mission STARTED (mode AUTO)`, `TAKEOFF confirmed`, and with `--watch` every autopilot message until it has landed.
The demo window shows the phases at the same time.

Instead of `mission.sh` you can also load the file in Mission Planner (FLIGHT PLAN > Load WP File > Write WPs) and
press Arm and Auto there ([MISSION_PLANNER](MISSION_PLANNER.md)).

### Make a mission for a world

```bash
bash simulation/scripts/astra.sh mission make runway --altitude 50          # 50 m = 164 ft, inside the SUAS band
bash simulation/scripts/astra.sh mission make suas_field_real --speed 6 --out ~/.astra_sim/missions/field_6ms.waypoints
bash simulation/scripts/astra.sh mission list                               # the files you can load
```

It plans a lawn-mower survey (sim_worlds `make_mission.py`), checks it, and prints the commands to load it, to fly
a full demo with it, and the Windows path for Mission Planner's "Load WP File". The worlds' own mission files in
the repo are never overwritten (new files go to `~/.astra_sim/missions/` unless you give `--out`).

## While it runs

```bash
bash simulation/scripts/astra.sh photo                 # one photo through the team's camera code
bash simulation/scripts/astra.sh photo --direct        # straight from the camera emulator
bash simulation/scripts/astra.sh status                # processes, ports, camera address, autopilot, ROS nodes
bash simulation/scripts/astra.sh status --no-mavlink   # the same without opening the ground-station port
bash simulation/scripts/astra.sh mp start              # open Mission Planner on Windows (WSL only)
```

Logs and data of the runs (`logs.sh`; RUN = the newest run when left out):

```bash
bash simulation/scripts/astra.sh logs list             # the last runs: result, flight time, targets found
bash simulation/scripts/astra.sh logs summary          # what ran, the result, the phases, every problem
bash simulation/scripts/astra.sh logs errors           # every warning / error line of every log, with file:line
bash simulation/scripts/astra.sh logs tail sim -f      # follow the end of one log (sim, team_stack, demo ...)
bash simulation/scripts/astra.sh logs bundle           # one zip of the run to send to a teammate
```

Every file of a run folder is explained in [LOGS_AND_DATA](LOGS_AND_DATA.md).

## Set up and build

Once per PC, in this order (about 30 minutes plus downloads):

```bash
sudo bash simulation/setup/01_install_humble.sh     # ROS 2 Humble, Gazebo Harmonic, MAVROS, Python stack
bash simulation/setup/02_setup_ardupilot.sh         # NO sudo: ArduCopter SITL 4.6.3, the Gazebo plugin, MAVProxy
bash simulation/setup/03_build.sh --workers 4       # copy the repo to Linux and build it
```

After every change on Windows: `bash simulation/scripts/astra.sh build` (= `03_build.sh`; one package:
`--packages "sim_demo"`). Other helpers:

| Command | What it does |
|---|---|
| `bash simulation/setup/check_ros.sh` | Says in plain words whether ROS 2 is ready in this terminal (exit 0 = yes) |
| `sudo bash simulation/setup/camera_ip_alias.sh` | Gives this PC the camera address 192.168.144.25 (now and after every restart) |
| `bash simulation/setup/fetch_sitl.sh 4.7.1` | Downloads another ArduCopter version (`--list` shows them; see [ARDUPILOT](ARDUPILOT.md)) |
| `python3 simulation/scripts/check_docs.py` | Checks that every doc is in the [index](../README.md) and no doc link is broken |

**How do I know it worked:** `03_build.sh` ends with `DONE. Next:` (exit code 0); warnings above it say what is
missing and how to get it.

## The real drone's Jetson

The Jetson Orin Nano on the drone is "an address this PC can SSH to": the USB-C cable address by default, or a
name / IP on Ethernet, Wi-Fi or a LAN (no USB cable needed). The Jetson commands come with the Jetson work
(`simulation/jetson/`, guide `docs/jetson/README.md`, both being written now; the plan is
[docs/jetson/HIL_DESIGN.md](../jetson/HIL_DESIGN.md)); through the front door they are
`bash simulation/scripts/astra.sh jetson WORD [options]`, and `bash simulation/scripts/astra.sh help jetson` shows
their help once they are in your copy. The owner logs in himself: `jetson login` opens a window where he types the
Jetson password, and the other Jetson commands use that open login (`jetson check --host HOST` first).

**Until then, do not start anything on the Jetson by hand.** The team uses that board: anything we run there needs
the owner's OK and its own `ROS_DOMAIN_ID`, so our ROS topics never mix with the team's.

## Exit codes (the same for every command)

| Code | Meaning |
|---|---|
| 0 | Done (warnings may be printed: read them) |
| 1 | Something failed; the message says what, why, and the fix |
| 2 | The command was typed wrong (unknown option, missing value); the message points to `--help` |
| 3 | `mission` / `mission_check` only: the mission file (or world) cannot be used |
| 4 | `mission check` / `mission_check` only: the checker itself failed (a bug, not your mission): report it with the file |
| 130 | You pressed Ctrl+C |

`astra.sh` returns the exit code of the script it ran.

## Where things are written

| What | Where |
|---|---|
| Everything of one run (logs, phases, photos, score report) | `~/.astra_sim/runs/<date>-<time>_<world>/` |
| Missions made with `mission make` or converted from `.plan` | `~/.astra_sim/missions/` |
| The mission loaded in this simulator session | `~/.astra_sim/state/loaded_mission.env` |
| Build and `start_sim` logs | `~/.astra_sim/logs/` |
| What the running simulator resolved (world, ports, home) | `~/.astra_sim/state/current_launch.yaml` |
