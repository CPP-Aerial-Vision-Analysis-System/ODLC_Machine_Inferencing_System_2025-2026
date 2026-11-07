<!--
docs/simulation/ERRORS.md -- PURPOSE: every error and warning message of the simulator's commands (simulation/scripts,
simulation/setup and the mission checker), grouped by command: what the message means and exactly how to fix it.
Each message on screen starts with the program's name in brackets, e.g. "XX [mission] ERROR: ...": search this page
for that name and the first words after "ERROR:" or "WARNING:". The commands themselves are in COMMANDS.md.
-->

# Every error message and its fix

Every message the simulator's commands print has the same shape:

```text
 XX [demo] ERROR: the simulated autopilot never became ready ...      <- WHAT failed (and which command said it)
    Likely cause: SITL or Gazebo did not start ...                    <- WHY (the most likely reason)
    Fix: bash simulation/scripts/stop_sim.sh --all, then run ...      <- HOW to fix it (copy-paste)
 !! [mission] WARNING: ... Consequence: ...                           <- a warning says what will go wrong
```

`XX` = error (the command stops), `!!` = warning (it goes on), `==>` and `OK` = normal progress.
**Find your message:** search this page (Ctrl+F) for the name in brackets, then for the first words after
`ERROR:` or `WARNING:`. Paths in the real message are full paths; here they are shortened.

Exit codes, the same for every command ([COMMANDS](COMMANDS.md#exit-codes-the-same-for-every-command)):
`0` done, `1` failed, `2` typed wrong, `3` (mission tools) the mission file cannot be used, `4` (mission checker) the
checker itself failed, `130` Ctrl+C.

Contents: [every script](#messages-every-simulator-script-can-print) · [astra](#astra-the-front-door) ·
[mission](#mission-missionsh) · [mission_check](#mission_check-the-mission-checker) ·
[upload_mission / fly_mission](#upload_mission-and-fly_mission) · [demo](#demo-demosh) ·
[start_sim](#start_sim) · [stop_sim](#stop_sim) · [start_over](#start_over) ·
[status / take_photo](#status-and-take_photo) · [03_build](#03_build) · [sim_env](#sim_env) ·
[01_install_humble](#01_install_humble) · [02_setup_ardupilot](#02_setup_ardupilot) ·
[camera_ip_alias / check_ros](#camera_ip_alias-and-check_ros) · [fetch_sitl](#fetch_sitl) ·
[other programs](#messages-of-other-programs)

## Messages every simulator script can print

These come from the shared helpers (`simulation/scripts/_common.sh`), so the name in brackets is the script you
ran (for example `[demo]`, `[mission]`, `[upload_mission]`).

| Message starts with | What it means | How to fix it |
|---|---|---|
| `ERROR: unknown option "..." ... Fix: see the options with ... --help` (exit 2) | You typed an option this command does not have, or forgot a value after an option. | Run the command with `--help` and copy the option from there. |
| `ERROR: The simulator is not built yet (no .../ros2_ws/install with sim_bringup)` | The build (`03_build.sh`) never ran on this PC, or it failed before `sim_bringup`. | `bash simulation/setup/03_build.sh`, then run your command again. |
| `ERROR: No simulator is running, so there is no autopilot or camera to talk to.` | A command that needs the running simulator (mission load / fly, upload, fly, photo) found no simulator: no process of the recorded session, and no `arducopter`, `siyi_emulator` or `gz sim` server process on this PC (a Docker simulator's processes count too). With `--url` this check is skipped (the autopilot may then be on another PC). | Start one first: `bash simulation/scripts/astra.sh manual` (window 1), then run your command in window 2. If it stopped by itself, read the end of the last run's `sim.log`. |
| `ERROR: UDP port 14550 (where this command listens for the autopilot's MAVLink) is already used by another program.` | Only one program can listen on that port: QGroundControl inside Ubuntu, or another `mission`/`status` command still running. (Mission Planner on **Windows** does not count: it gets its own copy.) | Close that program; find it with `ss -lunp \| grep 14550`. Or give the command another link with `--url udpin:127.0.0.1:PORT`. |
| `WARNING: Only N MB of memory is free; the simulator wants about 3000 MB.` | Gazebo may get slow, or Linux may stop a program. | Close other programs or another simulator (`bash simulation/scripts/stop_sim.sh --all`); check with `free -m`. |
| `WARNING: Could not add the camera address 192.168.144.25 to this PC.` | The team's `video_cam` talks to the camera at that address; without it there are no photos. Adding it needs admin rights. | Once: `sudo bash simulation/setup/camera_ip_alias.sh`, or from Windows PowerShell `wsl -d Ubuntu-22.04 -u root ip addr add 192.168.144.25/32 dev lo`. |
| `WARNING: MAVROS did not re-read the mission (its answer was not success).` | The upload worked, but the team's `waypoint_manager` still has the OLD mission: photos never stop, no hold, no drops. | Wait 10 s, then `ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull {}` (or restart `./launcher.sh sim`). |
| `MAVROS is not running (no /mavros/mission/pull)` (an info line, not an error) | The team stack is not started; MAVROS reads the mission itself when it starts. | Nothing to fix, unless you expected the team stack to run. |

## astra (the front door)

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[astra] ERROR: unknown command "..." (the commands are: demo, manual, ...)` (exit 2) | The word after `astra.sh` is not one of its commands. | `bash simulation/scripts/astra.sh help` lists them. |
| `[astra] ERROR: the command "logs" needs .../logs.sh, which is not in this copy.` | That part comes with the "runs and logs" work and is not in your copy yet. | Update your copy (`git pull`, then `03_build.sh`). Until then: `ls -1dt ~/.astra_sim/runs/* \| head -1` shows the newest run folder. |
| `[astra] ERROR: there is no command "...", so there is no help for it` (exit 2) | `astra.sh help WORD` with a word that is not a command. | `bash simulation/scripts/astra.sh help` lists the commands. |
| `[astra] ERROR: the command "jetson" needs .../jetson.sh, which is not in this copy.` | The Jetson commands come with the Jetson work and are not in your copy yet (plan: [docs/jetson/HIL_DESIGN.md](../jetson/HIL_DESIGN.md)). | Update your copy when it is merged. Until then do **not** start anything on the Jetson by hand: the team uses the board, and anything we run there needs the owner's OK and its own `ROS_DOMAIN_ID`. With `jetson.sh` the owner logs in himself (`jetson.sh login`), then `jetson.sh check --host HOST`. |
| `[astra] ERROR: the command "..." needs simulation/..., which is not in this copy.` (any other) | An old or partial copy of the repo. | `git pull`, then `bash simulation/setup/03_build.sh`. |

Every other message after `astra.sh WORD` comes from the script that word runs: look under that script's name.

## mission (mission.sh)

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[mission] ERROR: unknown command "..."` (exit 2) | The word after `mission.sh` must be `check`, `load`, `fly`, `make` or `list`. | `bash simulation/scripts/mission.sh --help`. |
| `[mission] ERROR: check needs a mission FILE` / `load needs a mission FILE or a WORLD name` / `make needs a WORLD` (exit 2) | The file or world is missing on the command line. | Add it, e.g. `mission.sh check ~/Downloads/field.waypoints`, `mission.sh make runway`. |
| `[mission] ERROR: ... takes ONE mission, got "A" and "B"` (exit 2) | Two file names, often a Windows path with spaces but **without quotes**. | Put the path in quotes: `"C:\Users\me\Documents\Mission Planner\field.waypoints"`. |
| `[mission] ERROR: the mission file "..." does not exist.` (exit 3) | A typo in the name, a Windows path without quotes (the backslashes get lost), or a `\\wsl.localhost\OTHER-DISTRO\...` path of another WSL distribution (only this one can be read). | Check it with `ls "FILE"`; put a Windows path in quotes (`C:\...` and `\\wsl.localhost\Ubuntu-22.04\...` both work). |
| `[mission] ERROR: "..." is neither a mission file nor a world with a survey mission.` (exit 3) | Not a file, and not a world name (`runway`, `suas_field`). | `bash simulation/scripts/mission.sh list` shows what you can use. |
| `[mission] ERROR: there is no world called "..."` (exit 3, `make`) | A typo in the world name. | Use one of the names the message lists (`runway`, `suas_field`). |
| `[mission] ERROR: the mission check found errors (listed above), so nothing was uploaded.` (exit 1) | The checker found something the team's flight code cannot fly (each `ERROR:` line above says what). | Fix it in Mission Planner and save again (FLIGHT PLAN > Save WP File). Upload anyway only for a test: add `--force`. |
| `[mission] WARNING: --force: uploading although the check found errors` | You chose to upload a mission with errors. The team code will probably not take photos, hold or drop correctly. | Only for tests. |
| `[mission] ERROR: the mission checker is missing (...mission_check.py)` | An old or partial copy of the repo. | Update the checkout, then `bash simulation/setup/03_build.sh`. |
| `[mission] ERROR: the mission checker stopped with exit code N` | A problem in the checker itself (exit code 4 = a bug in `mission_check.py`, not in your mission). | Run `bash simulation/scripts/mission.sh check FILE` to see its message, and report it with the file. |
| `(the simulator's home is not known on this PC, so the check cannot tell whether the mission is for the field the drone is on ...)` (an info line, `load`) | The simulator's recorded home belongs to a session that is not running on this PC (it runs in Docker or elsewhere, a different world ran last), or you gave `--url`. The "drone would fly away" check is skipped rather than guessed. | Look at the mission on the map in Mission Planner before you fly it. |
| `[mission] ERROR: making the mission failed (the reason is above).` | `make_mission.py` refused a setting (an area name, a value out of range). | See every setting: `ros2 run sim_worlds make_mission.py --help`. |
| `[mission] ERROR: cannot create the folder ...` | No write permission where `--out` points. | Give another place with `--out FILE`. |
| `[mission] WARNING: the new mission has problems (above)` | `make` wrote the file, but the check found errors. | Change the settings (altitude, speed) and make it again. |
| `[mission] WARNING: no mission was loaded with mission.sh in this simulator session.` | `fly` without a file flies whatever the autopilot has stored (Mission Planner's, the demo's, an older one). It waits 5 s first when you watch. | Press Ctrl+C within 5 s and load yours: `mission.sh fly FILE`. |
| `[mission] WARNING: the team's flight stack (./launcher.sh sim, with MAVROS) was not found: no mavros_node process on this PC and no /mavros node in ROS domain N.` | The drone flies the lanes and comes home, but takes no photos and drops nothing. | Start the simulator with `astra.sh manual` (it starts the team stack), or start `./launcher.sh sim` by hand. If the team stack runs on the Jetson or in a container with another `ROS_DOMAIN_ID`, this PC cannot see it: ignore the warning. |
| `(the team's MAVROS is not on this PC: it must run where the team stack runs ...)` (an info line, `fly`) | This session's team stack runs elsewhere (demo.sh recorded `TEAM_STACK_WHERE=external`), or you gave `--url`. | Make sure the team stack is running there; without it there are no photos and no drops. |

## mission_check (the mission checker)

`mission_check.py` (run by `mission.sh check / load / fly`, or by itself) prints a report that ends with
`RESULT: OK ...` (exit 0) or `RESULT: NOT READY, N error(s)` (exit 1). With `--strict`, warnings count as errors.

### The file cannot be used (exit 3)

These print one line: `[mission_check] ERROR: <what>. Likely cause: <why>. Fix: <how>`.

| Message contains | What it means | How to fix it |
|---|---|---|
| `the mission file "..." does not exist` | Wrong path. Inside WSL a Windows file `C:\Users\me\m.waypoints` is `/mnt/c/Users/me/m.waypoints`. | Check the name with `ls`. |
| `is a folder, not a mission file` | The file name is missing. | Give the `.waypoints` / `.plan` file inside the folder. |
| `cannot read "..."` | No read permission, or not a text file. | Save the mission again from Mission Planner and use that file. |
| `the file is empty` / `not a mission file: the first line is ...` | Not a Mission Planner file (first line `QGC WPL 110`) or a QGroundControl file (starts with `{`). | Save the mission from Mission Planner (FLIGHT PLAN > Save WP File). |
| `unsupported mission file version` | Another `QGC WPL` version than 110. | Save it again from Mission Planner. |
| `has the "QGC WPL 110" header but no mission items` | Only the first line. | Save the full mission again. |
| `mission line N has K fields, expected 12` / `mission line N is malformed` | A broken line (hand edit, cut file). | Open it in Mission Planner and save it again, or fix line N. |
| `not a valid QGroundControl .plan file (the JSON is broken ...)` / `no "mission" section` / `no usable "plannedHomePosition"` | A broken or non-mission `.plan` file. | Save the plan again from QGroundControl. |
| `is a QGroundControl "survey" pattern without its generated waypoints` | QGroundControl stored only the pattern's outline. | Open it in QGroundControl 4.x and save again, draw the lanes as simple waypoints, or make the mission in Mission Planner. |
| `has an unknown type` / `is not a number` / `is not a whole number` | A `.plan` item the checker does not know, or a command / frame that is not a whole number. | Make the mission from simple items, or in Mission Planner. |
| `is ..., not a mission item` / `"params" is ..., not a list of 7 numbers` / `"TransectStyleComplexItem" is not a JSON object` / `its stored waypoints ("Items") are not a list` / `"items" is not a list of mission items` | A `.plan` file with a broken structure (edited by hand, or written by another program). | Save the plan again from QGroundControl, or make the mission in Mission Planner. |
| `the file has a structure this checker does not understand (...)` | Anything else the parsers did not expect (never a Python traceback). | Save the mission again from Mission Planner and use that file. |
| `--home "..." is not LAT,LON` (exit 2) | A typo in `--home`. | `--home 36.2143,-96.0068`. |
| `the following arguments are required: file` / `unrecognized arguments: ...` (exit 2) | The mission file name is missing, or an option is mistyped. The message ends with the `--help` command to run. | `mission_check FILE` (or `python3 .../mission_check.py FILE`); see `--help`. |
| `cannot write ... (--write-waypoints)` | The output folder does not exist or is read-only. | Give another path. |
| `the checker itself failed on "..." (...)` (exit 4) | A bug in `mission_check.py` for an unusual mission; the mission itself may be fine. | Report it with the mission file; meanwhile check the mission by eye in Mission Planner. |

### ERROR lines in the report (the team code cannot fly it; exit 1)

| ERROR line starts with | Why it matters | How to fix it in Mission Planner |
|---|---|---|
| `no TAKEOFF item` | ArduCopter does not take off in AUTO without one. | Add TAKEOFF as the first item after home (right-click > Takeoff). |
| `item 0 is the TAKEOFF, but the autopilot treats item 0 as the HOME position` | The file has no home line (written by hand or by another program): the autopilot never flies item 0, so there is no TAKEOFF and AUTO cannot start. | Open it in Mission Planner and save it again (Save WP File adds home), or add a home line `0 1 0 16 0 0 0 0 LAT LON ALT 1` first. |
| `the TAKEOFF altitude is 0.0 m` | The drone would not climb. | Set the take-off altitude. |
| `N flight item(s) between TAKEOFF and RETURN_TO_LAUNCH are at or below 5 m above home` | The drone would fly the search at ground level or into the ground (a negative height is below home), in the simulator and on the real drone. For exactly 0 ArduCopter may keep the height of the item before instead: just as wrong. | Set the Alt column of those items to the survey altitude (45.7-121.9 m = 150-400 ft for SUAS 2026), frame Relative. |
| `no RETURN_TO_LAUNCH item` | The team code finds everything from it: photos never stop, no hold, no target waypoints, no drops. | End the mission with RETURN_TO_LAUNCH (right-click > RTL). |
| `N RETURN_TO_LAUNCH items` | ArduPilot goes home at the first one, the team code uses the last one. | Keep exactly one, as the last item. |
| `the mission is too short` | The team code needs TAKEOFF, the search waypoints, then two waypoints before RTL. | Add the lanes and a hold waypoint before RTL. |
| `item N, right before RETURN_TO_LAUNCH, is DO_..., not a WAYPOINT` | `main_controller` holds when that item is REACHED; DO_ items are never reached. | Put a WAYPOINT (the hold point, e.g. the middle of the search area) right before RTL. |
| `item N (buffer_wp, two before RETURN_TO_LAUNCH) is ..., not a WAYPOINT` | The photo trigger stops when buffer_wp is reached; a DO_ item is never reached. | Make the item two before RTL the end of the last lane; put DO_ items earlier. |
| `no DO_DIGICAM_CONTROL item` | Only ArduPilot's `Mission: N DigiCamCtrl` message starts the team's photos. | Add DO_DIGICAM_CONTROL right after the first search waypoint. |
| `DO_DIGICAM_CONTROL (item N) comes at or after buffer_wp` | Photos would stop before they start. | Move it to the start of lane 1. |
| `item N (...) contains NaN or infinity` / `has an impossible position` | ArduPilot rejects or misreads it. | Re-enter that item; latitude -90..90, longitude -180..180. |
| `a waypoint is N km from where the drone starts` | Only with `--home` (mission.sh load passes the simulator's home): the mission is for another field, the drone would fly away. | `mission.sh make WORLD` for this field, or start the matching world. |

### WARNING lines in the report (it can fly; read them)

| WARNING line starts with | What it means | Fix |
|---|---|---|
| `N flight item(s) fly below the SUAS 2026 rule of 150-400 ft ... Fine for a simulator test` | Lowest item 20 m or higher: fine for a simulator test (the runway world flies at 30 m = 98 ft). | At the competition fly 45.7-121.9 m above the field, and re-plan the lane spacing. |
| `N flight item(s) fly below the SUAS 2026 rule ... That is too low even for a simulator test` | Lowest item between 5 and 20 m: trees, poles and buildings are in the way, and each photo covers only a few metres. | Fly 45.7-121.9 m above the field (at least 20 m for a quick test) and re-plan the lane spacing. |
| `N item(s) fly ABOVE 400 ft` | Breaks the SUAS 2026 rule and FAA Part 107. | Lower them to at most 121.9 m. |
| `some flight items use altitudes above sea level (frame 0) / above terrain (frame 10)` | The team code thinks in heights above home (frame 3, Mission Planner "Relative"). | Set the altitude frame to Relative. |
| `item 0 is ..., not the HOME line` | Every index would be off by one. | Save the mission from Mission Planner (it adds home). |
| `N TAKEOFF items` / `the TAKEOFF is item N, not item 1` | Items before TAKEOFF run on the ground; the team code uses the last TAKEOFF. | One TAKEOFF, right after home. |
| `RETURN_TO_LAUNCH (item N) is not the last item` | Items after RTL are never flown. | Delete them or move RTL to the end. |
| `the hold point (item N) is LOITER_..., not a plain WAYPOINT` | It works; the team mission uses a WAYPOINT there. | Optional. |
| `N DO_DIGICAM_CONTROL items` / `DO_DIGICAM_CONTROL (item N) comes before the TAKEOFF` | Every DigiCamCtrl (re)starts the photos; before TAKEOFF they start on the ground. | Keep one, right after the first search waypoint. |
| `item N (...) has latitude/longitude 0, 0` | ArduCopter stays where it is for that item: almost always a mistake. | Give the waypoint its real position. |
| `item N (...) has autocontinue 0` | The mission stops there and waits. | Set autocontinue to 1. |
| `the sequence numbers in the file are not 0, 1, 2 ...` | The tools upload in file order; Mission Planner may read it differently. | Open and save it again in Mission Planner. |
| `item N is DO_SET_SERVO on channel 11 / 13` | The mission itself would open a payload release before a target is found. | Remove it; `main_controller` drops the payloads. |
| `item N is LAND` | The team code works with RETURN_TO_LAUNCH only. | Replace LAND with RTL. |
| `DO_CHANGE_SPEED (item N) sets N m/s` | Above 15 m/s the photos (1 per second) are far apart and may blur. | 4-8 m/s (the team mission: 5.5). |
| `a waypoint is N km from the home position in the file` | Another field, or a mistyped coordinate. | Check the waypoints on the map. |

## upload_mission and fly_mission

`mission.sh load` / `fly` run these two; you can also run them yourself.

| Message starts with | What it means | How to fix it |
|---|---|---|
| `ERROR: No mission for "...": it is neither a file nor a world with a survey mission.` | A typo in the file or world name, or sim_worlds not built. | `bash simulation/scripts/mission.sh list`. |
| `ERROR: the upload failed (mission_tool's reason is above).` | The autopilot did not answer (still starting, or stopped) or it rejected an item. | Wait until `bash simulation/scripts/status.sh` shows the autopilot; check the file with `mission.sh check FILE`; try again. |
| `[fly_mission] ERROR: the mission did not start (mission_tool's reason is above).` | The autopilot is still settling (30-60 s after it starts is normal), it refused to arm (a `PreArm:` line says why), or the mission has no TAKEOFF. | Wait a minute and run it again; check the mission with `mission.sh check FILE`. |

Plus the [shared messages](#messages-every-simulator-script-can-print) (no simulator, port 14550 busy, MAVROS re-read).

## demo (demo.sh)

Before anything starts, `demo.sh` checks everything and lists each problem as a `WARNING`, then stops with
`ERROR: N problem(s) found (the WARNING lines above); nothing was started.` Fix each warning, then run it again.

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[demo] ERROR: unknown option` / `unexpected argument` / `--camera must be ...` / `--detector must be ...` / `--zoom must be ...` / `"..." is not a whole number` / `--profile needs ...` / `--set needs KEY=VALUE` / `--windows-gcs needs HOST` / `--model needs a weights file` / `--team-repo needs the folder ...` / `--launch-arg needs name:=value` (exit 2) | Typed wrong. The world comes first and only once. | `bash simulation/scripts/demo.sh --help`. |
| `[demo] ERROR: the plug-in profile cannot be used` | A typo in the profile name or a `--set`, a file the profile names is missing, or sim_plugins not built. | `python3 -m sim_plugins.profile_tool plan --profile NAME --check` says which line. |
| `WARNING: the team package "..." is not built/installed` / `the simulation package "..." is not built` | The build did not finish. | `bash simulation/setup/03_build.sh`. |
| `WARNING: the team's launcher.sh is missing in ...` | A broken copy (or a wrong `--team-repo`). | Update the copy, or check the `--team-repo` folder. |
| `WARNING: world "..." has no world file` / `has no survey mission` / `mission file not found` | A typo in the world name or `--mission` path. | Worlds: `runway`, `suas_field`; missions: `mission.sh list`. |
| `WARNING: the mission file does not have what the team code needs` | The mission check failed. | `bash simulation/scripts/mission.sh check FILE` explains each problem. |
| `WARNING: no detector weights` / `the detector weights ... cannot be used` | `yolo26m.pt` is missing (no internet during the build) or the file is broken. | `03_build.sh` with internet, `--model FILE`, or `--detector truth`. |
| `WARNING: udp 14550 is in use` / `udp 14552 is in use` | A ground station inside Ubuntu, or an old MAVROS / launcher. | Close it (`ss -lunp`), or `--gcs-port P`. |
| `WARNING: UDP 14555 (the flight recorder's port) is already in use` | No live CSV files this run (the flight is not affected). | `pkill -f sim_demo.flight_recorder`, or `--recorder-port P`. |
| `WARNING: sim_demo is not built. Consequence: no demo panel, no markers, no score report` | Its build failed or was skipped; the flight itself still works. | `bash simulation/setup/03_build.sh` (read its first error), then the demo again. |
| `WARNING: world "..." has no truth file. Consequence: the score report cannot compare ...` | The world config (`sim_worlds/config/WORLD.yaml`) names no truth file, or it was not rebuilt. | Add the truth file to the world ([WORLDS](WORLDS.md)) and run `03_build.sh`; fine to ignore for a world without targets. |
| `[demo] ERROR: the camera address 192.168.144.25 is needed` | It could not be added (no admin rights). | `sudo bash simulation/setup/camera_ip_alias.sh` once, then the demo again. |
| `WARNING: the simulated autopilot never became ready` | SITL or Gazebo did not start (an old run still holds a port, or 3D drawing failed). The last 25 lines of `sim.log` follow. | `bash simulation/scripts/stop_sim.sh --all`, then the demo again; read `sim.log` if it repeats. |
| `WARNING: the team stack is not ready (a node stopped or never started): starting it once more.` | One automatic retry. | Nothing, unless the next message comes. |
| `WARNING: the team stack did not get ready, even after one restart.` | A team node crashes at start, MAVROS cannot reach SITL, or the model does not load. | Read `team_stack.log` from the first ERROR; try `--detector truth` to rule out the model. |
| `WARNING: the team stack that runs elsewhere (the Jetson / another container) did not get ready in 10 minutes.` (then exit 1) | With `run.team_stack: external` the team stack runs on another machine: it was not started there, its MAVROS cannot reach the autopilot (`network.mavros_out` / `FCU_URL`), or its `ROS_DOMAIN_ID` differs from this PC's, so this PC cannot see its nodes. | Start it with the command demo.sh printed, check that `ros2 node list` on this PC shows `/mavros`, then run the demo again. |
| `WARNING: the mission did not start (mission_tool's reason is above).` | The autopilot refused to arm (`PreArm:` line) or did not switch to AUTO. | Stop and start again; check the mission with `mission.sh check`. |
| `WARNING: the simulator stopped unexpectedly.` | Gazebo or SITL crashed, or its window was closed. | Read the end of `sim.log`, `stop_sim.sh`, then the demo again. |
| `WARNING: The demo timed out after N minutes (the drone had not landed).` | The drone got stuck, or nobody started the flight (`--manual`). | Look at the last phases; give more time with `--timeout MINUTES`. |
| `WARNING: The phase monitor ended with code N (not a normal landing).` | The score report may be incomplete. | Read `phases.jsonl` and `sim.log` in the run folder. |
| `WARNING: no score report was written yet` | `score_run` did not finish its report. | `ros2 run sim_demo score_run --rescore RUN_FOLDER`. |
| `WARNING: something from the old run is still alive (see above). Consequence: two simulators may fight over the same ports` | An earlier simulator did not stop completely; the new run may fail. | If it does: `bash simulation/scripts/stop_sim.sh --all`, then the demo again. |
| `WARNING: Ctrl+C: stopping the demo` | You pressed Ctrl+C; the score report is still written (exit 130). | Nothing. |

## start_sim

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[start_sim] ERROR: unknown option` / `unexpected argument` / `--profile needs a value` / `--set needs KEY=VALUE` (exit 2) | Typed wrong. Settings look like `name:=value` (e.g. `camera_model:=a8mini`). | `bash simulation/scripts/start_sim.sh --help`. |
| `[start_sim] ERROR: the plug-in profile cannot be used` | Same as in demo. | `python3 -m sim_plugins.profile_tool plan --profile NAME --check`. |
| `[start_sim] ERROR: the world file "..." does not exist.` | A typo in the `.sdf` / `.yaml` path. | `ls FILE`, or use `runway` / `suas_field`. |
| `[start_sim] WARNING: "..." is not one of runway / suas_field` | It tries it as a world name; a wrong name stops the launch with an error in the log. | Check the spelling. |
| `[start_sim] WARNING: continuing without the camera address` | No photos (video_cam cannot reach the emulator). | The two commands printed just above it. |

## stop_sim

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[stop_sim] ERROR: --session needs a session id` / `unknown option` (exit 2) | Typed wrong. | `bash simulation/scripts/status.sh` shows the session id. |
| `[stop_sim] WARNING: N simulator process(es) did not stop.` (exit 1) | A program is stuck (often in the graphics driver). Its ports stay busy. | `bash simulation/scripts/stop_sim.sh --all` again; if it stays, `kill -9` the pids it lists. |
| `No simulator session is recorded` / `The simulator is not running ... nothing to stop` | Not an error: nothing to do. | Use `--all` to also look for older sessions. |

## start_over

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[start_over] ERROR: --world needs`, `--land must be land or rtl`, `--land-timeout needs whole seconds`, `unknown option` (exit 2) | Typed wrong. | `bash simulation/scripts/start_over.sh --help`. |
| `[start_over] ERROR: the mission file "..." does not exist.` | A typo after `--mission`. | `ls FILE`; `mission.sh list`. |
| `[start_over] ERROR: the package sim_demo is not built` | START OVER lives in sim_demo. | `bash simulation/setup/03_build.sh`. |
| `[start_over] ERROR: no simulator is running, so there is nothing to start over.` | Only with `--no-fresh` (the panel button); without it a fresh demo starts. | `bash simulation/scripts/demo.sh runway --keep` (or `--manual`). |
| `[start_over] ERROR: START OVER did not finish` | The drone did not land in time, the autopilot did not restart, or the team stack did not get ready. | Read the log it names; to start clean: `stop_sim.sh`, then `demo.sh`. |

## status and take_photo

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[status] ERROR: unknown option` (exit 2) | The only option is `--no-mavlink`. | `bash simulation/scripts/status.sh --no-mavlink`. |
| `[status] WARNING: the camera address 192.168.144.25 is missing on this PC.` | No photos until it exists. | `sudo bash simulation/setup/camera_ip_alias.sh` (demo and start_sim also add it). |
| `[take_photo] ERROR: No simulator is running` | There is no camera to press. | Start the simulator first (`astra.sh sim` or `astra.sh manual`). |
| `[photo] ...` lines | Messages of `photo_trigger` (sim_bringup): they say which path it took and why a photo did not come. | `bash simulation/scripts/take_photo.sh --direct` skips the team code; see [TROUBLESHOOTING](TROUBLESHOOTING.md#camera-and-photos). |

## 03_build

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[03_build] ERROR: unknown option`, `--workers needs a whole number`, `--dest needs a folder`, `--packages needs a list` (exit 2) | Typed wrong. | `bash simulation/setup/03_build.sh --help`. |
| `[03_build] ERROR: this build must not run as root.` | It was started with `sudo`. | Run it again without `sudo`. |
| `[03_build] ERROR: ROS 2 Humble is not installed` | Step 1 has not run (or failed). | `sudo bash simulation/setup/01_install_humble.sh`. |
| `[03_build] ERROR: the build folder "..." contains a space.` | Gazebo and colcon fail in such folders. | `--dest "$HOME/ODLC_Machine_Inferencing_System_2025-2026"`. |
| `[03_build] ERROR: there is no ros2_ws/src in ...` | `--no-sync` on a copy that was never made, or a wrong `--dest`. | Run it without `--no-sync`. |
| `[03_build] WARNING: colcon reported failures` | Some packages did not build ("Failed" lines). The simulator may still start. | Read the first error in the build log it names; build one package with `--packages "NAME"`. |
| `[03_build] ERROR: sim_bringup was not built, so nothing can start.` | A compile or Python error in a package it needs. | Read the first error in the log; a clean build: `--clean`. |
| `[03_build] WARNING: some Gazebo Fuel models could not be downloaded.` | The worlds still load, with fewer details. | Run the build again later with internet. |
| `[03_build] WARNING: could not download the detector weights yolo26m.pt.` | The team's real detector cannot run in the simulator. | Build again with internet, or fly with `--detector truth`. |

## sim_env

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[sim_env] WARNING: ROS 2 Humble is not installed` | No `ros2` command and no simulator script works in this terminal. | `sudo bash simulation/setup/01_install_humble.sh`. |

## 01_install_humble

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[01_install_humble] ERROR: this script installs software, so it needs admin rights.` | Started without `sudo`. | `sudo bash simulation/setup/01_install_humble.sh` (no password: `wsl -d Ubuntu-22.04 -u root bash <file>` from PowerShell). |
| `[01_install_humble] ERROR: ASTRA needs Ubuntu 22.04 (codename jammy)` | Another Ubuntu version; Humble is built for 22.04 only. | `wsl --install -d Ubuntu-22.04` and run it there. |
| `[01_install_humble] ERROR: could not ask GitHub for the newest ros2-apt-source version.` | No internet, or GitHub's limit of 60 questions per hour. | Wait and run it again. |
| `[01_install_humble] ERROR: the step "[N/8] ..." failed` | A command in that step failed (its own message is above): usually no internet, a software source that did not answer, or apt busy. | Check the internet and run it again; it continues where it stopped. |
| `[01_install_humble] WARNING: the Gazebo Python bindings ... were not found` / `docker/requirements-*.txt not found` / `could not add the camera address` | Each says its consequence. | Each says its fix (run again later; run it from the repo; `camera_ip_alias.sh`). |

## 02_setup_ardupilot

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[02_setup_ardupilot] ERROR: this must not run as root.` | Started with `sudo`. | `bash simulation/setup/02_setup_ardupilot.sh` without `sudo`. |
| `[02_setup_ardupilot] ERROR: ROS 2 Humble is not installed` | Step 1 has not run. | `sudo bash simulation/setup/01_install_humble.sh`. |
| `[02_setup_ardupilot] ERROR: cmake could not configure the ArduPilot Gazebo plugin` | A Gazebo Harmonic development package is missing. | Run step 1 again, then this. |
| `[02_setup_ardupilot] ERROR: the plugin is not linked against Gazebo Harmonic (gz-sim8)` | An older Gazebo was found when it was built. | `rm -rf ~/ardupilot_gazebo/build`, then run it again. |
| `[02_setup_ardupilot] ERROR: mavproxy.py does not start` | A broken Python package in `~/sitl/venv`. | `rm -rf ~/sitl/venv`, then run it again. |
| `[02_setup_ardupilot] ERROR: the step "[N/4] ..." failed` | A download, `git clone` or build in that step failed (its message is above). | Check the internet and run it again; it skips what is done. |
| `[02_setup_ardupilot] WARNING: could not read the ArduCopter version` | Harmless if SITL starts. | If SITL fails later: delete `~/sitl/arducopter` and run it again. |

## camera_ip_alias and check_ros

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[camera_ip_alias] ERROR: this changes the PC's network settings, so it needs admin rights.` | Started without `sudo`. | `sudo bash simulation/setup/camera_ip_alias.sh`. |
| `[camera_ip_alias] ERROR: could not add 192.168.144.25 to the loopback device.` | This environment does not allow network changes (some containers). | Run it with admin rights, or start the container with `--cap-add NET_ADMIN`. |
| `[camera_ip_alias] ERROR: unknown option` (exit 2) | The options are `--remove` and `--no-boot`. | `--help`. |
| `check_ros.sh`: `[FAIL] ...` lines with `->` arrows | Each `[FAIL]` names one missing piece; the arrow under it is the fix (exit 1). | Do what the arrow says, then run the check again. |

## fetch_sitl

| Message starts with | What it means | How to fix it |
|---|---|---|
| `[fetch_sitl] ERROR: which version?` / `is not a version name` / `unknown option` / `... needs ...` (exit 2) | Typed wrong. | `bash simulation/setup/fetch_sitl.sh --help`; versions: `--list`. |
| `[fetch_sitl] ERROR: ArduCopter X was not found on the firmware server` | A version that does not exist, or no internet. | `bash simulation/setup/fetch_sitl.sh --list`. |
| `[fetch_sitl] ERROR: could not read the list of versions` | No internet, or the server is down. | Try again later. |
| `[fetch_sitl] ERROR: this computer is ...: ready-made SITL programs exist only for x86_64` | An ARM computer (for example a Mac or a Jetson). | Build SITL from source and use `--from-file` (see [ARDUPILOT](ARDUPILOT.md)). |
| `[fetch_sitl] ERROR: the downloaded file is not a Linux program` | The server sent an error page, or `--from-file` names the wrong file. | Run it again with `--force`. |
| `[fetch_sitl] ERROR: cannot create ...` / `cannot copy ...` / `cannot move the download ...` | No write permission, a full disk, or the simulator still uses the folder. | `df -h ~`; stop the simulator; try again. |
| `[fetch_sitl] WARNING: ... could not be downloaded` | A parameter file was taken from `~/sitl` instead (usually fine). | Run again with internet, or `--params-ref TAG`. |

## Messages of other programs

Some messages come from programs the scripts run. They use the same shape and their own name:

| Name in brackets | Program | Where it is explained |
|---|---|---|
| `[mission]` lines without `ERROR:` (`connecting ...`, `waiting for: GPS 3D fix ...`) | `mission_tool` (sim_bringup): upload, arm, AUTO, watch | [sim_bringup README](../../ros2_ws/src/simulation/sim_bringup/README.md); waiting 30-60 s for GPS / EKF / pre-arm is normal. |
| `[stack_check]` | Checks the team stack is up (MAVROS, the 6 nodes, the model) | [TROUBLESHOOTING](TROUBLESHOOTING.md#startup-and-ports) |
| `[photo]` | `photo_trigger` (take_photo.sh) | [TROUBLESHOOTING](TROUBLESHOOTING.md#camera-and-photos) |
| `[check_docs]` | The docs checker | [docs/README.md](../README.md#keeping-the-docs-up-to-date) |
| profile messages | `sim_plugins.profile_tool` | [PLUGINS](PLUGINS.md) |
| `[logs]` | `logs.sh` (`astra.sh logs`): the run folders, logs and CSV files | `bash simulation/scripts/logs.sh --help`, [LOGS_AND_DATA](LOGS_AND_DATA.md) |
| `[jetson]` | The Jetson commands (being written by the Jetson work) | Their `--help`; their guide will be listed in [docs/README.md](../README.md). |
| Docker messages | `simulation/docker/*.sh` | [DOCKER](DOCKER.md#troubleshooting) |

Still stuck? [TROUBLESHOOTING](TROUBLESHOOTING.md) follows a failure from the symptom to the failing link.
