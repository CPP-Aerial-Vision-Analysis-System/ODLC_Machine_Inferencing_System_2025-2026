<!--
docs/simulation/LOGS_AND_DATA.md -- PURPOSE: the guide to everything a simulator run writes down: the run folder, every
log file, the CSV files (what each column means and its unit), the run summary, the table of all runs, the logs.sh
command, the exporter for older runs, START OVER runs, and what to do when something is missing. Written so a 5th
grader can follow each step. Code: ros2_ws/src/simulation/sim_demo/sim_demo/{flight_recorder,csv_export,run_summary,
run_records,run_csv}.py, simulation/scripts/logs.sh and the recording hooks of simulation/scripts/demo.sh.
-->
# Logs and data of a run

Every time you run the demo (`bash simulation/scripts/demo.sh`), the computer writes down **everything that
happened** in one folder: what the drone did (as spreadsheet files), what every program said (log files), the
score, and a one-page summary. This page tells you where those files are, what is inside, and how to look at them.

**Short version:**

```bash
bash simulation/scripts/logs.sh list        # all runs, newest first
bash simulation/scripts/logs.sh summary     # the one-page summary of the newest run
bash simulation/scripts/logs.sh csv         # where the spreadsheet (CSV) files are, also as a Windows path
bash simulation/scripts/logs.sh errors      # every warning and error line, with file and line number
bash simulation/scripts/logs.sh bundle      # one zip file to send to a teammate
```

(Type these in the Ubuntu window, in the repo folder. From anywhere else, use the full path:
`bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/logs.sh list`.)

## 1. Where is the run folder?

Each run gets its own folder, named after the date, the time and the world:

```
~/.astra_sim/runs/20261031-110240_runway/
```

`~` means your home folder in Ubuntu. From **Windows**, open the File Explorer and paste this into the address bar
(change the user name and the run name):

```
\\wsl.localhost\Ubuntu-22.04\home\<your user>\.astra_sim\runs\
```

`bash simulation/scripts/logs.sh csv` prints the exact Windows path of the newest run for you.

**How do I know it worked:** `bash simulation/scripts/logs.sh list` prints a table with one line per run.

## 2. What is in the run folder?

Every log line starts with its **date and time** (`2026-10-31 11:26:39 ...`), so you can line up what different
programs did at the same moment. demo.sh adds that time to every line of `demo.log`, `sim.log` and `team_stack.log`
(also to lines the programs print without a time, like MAVProxy's, Gazebo's and Python tracebacks).

| File | Who writes it | What it is |
|---|---|---|
| `demo.log` | `demo.sh` | everything the demo window showed, from the very first check (also the warnings printed before the start) |
| `demo.env` | `demo.sh` | how the run was started: world, mission, detector, camera, profile, where the team code ran |
| `sim.log` | the simulator (`sim.launch.py`) | Gazebo, ArduCopter SITL, MAVProxy, the camera emulator, the scorer |
| `team_stack.log` | the team's code (`./launcher.sh sim`) | MAVROS and the 6 team nodes |
| `flight_recorder.log` | the flight recorder | what the recorder heard and wrote |
| `flight_recorder.json` | the flight recorder | the autopilot it heard (system id, ArduPilot version), how many messages of each kind, how many rows |
| `flight_recorder.pid` | the flight recorder | only while it runs: its process number (`logs.sh` sees from it that the run is still going) |
| `phases.jsonl` | `phase_monitor` | every mission phase, photo and detection line printed in the demo window |
| `events.jsonl` | `score_run` | every status text, arm/disarm, waypoint, photo and detection the scorer saw |
| `run_data.json` | `score_run` | everything the score was computed from (re-score: `ros2 run sim_demo score_run --rescore DIR`) |
| `score_report.md` / `.json` | `score_run` | the score: targets found, confidence, aim error, drop error, detection quality |
| `mission_map.png` | `score_run` | a map of the search area, the photo footprints, the track, the detections and the drops |
| `run_meta/*.json` | `sim_plugins run_meta` | which pieces REALLY ran: the ArduCopter version, the detector model (with SHA-256), the copy of the team code (see [SWAPPING](SWAPPING.md)) |
| `photos/`, `detections/` | `demo.sh` at the end | the camera's photos (+ a `.json` "sidecar" per photo) and the detector's annotated photos |
| `csv/` | flight recorder + exporter | the spreadsheet files (section 3) |
| `run_summary.md` / `.json` | `run_summary` at the end | the one-page summary (section 4) |
| `run_summary.log` | `demo.sh` | what the summary step printed (look here if the summary is missing) |
| `start_over.env`, `sitl.log`, `mavproxy.log`, `helpers.log`, `phases.log` | START OVER | only in a run started by START OVER (section 4b) |
| `jetson/` | the Jetson tools | only when the team code ran on the Jetson: `facts.env` (power mode ...), `tegrastats.log`, ... |
| `flight_recorder_tlog.json` | `export_csv --tlog` | what a replayed MAVLink log held (section 8) |

## 3. The CSV files (`csv/`)

CSV files open in Excel, LibreOffice Calc, Google Sheets or Python (`pandas.read_csv`). Rules for every file:

* **The unit is in the column name:** `_m` metres, `_mps` metres per second, `_deg` degrees, `_s` seconds, `_v`
  volts, `_pct` percent, `_us` microseconds (servo pulse width), `_px` pixels.
* **Time** is in three columns: `wall_time_utc` (like `2026-10-31T18:03:57.182Z`, world time UTC), `unix_time_s`
  (seconds since 1970: easy to subtract) and `t_since_arm_s` (seconds since the motors were FIRST armed in this run;
  arming again later does not restart it). Before arming the live files leave `t_since_arm_s` empty; files made
  afterwards by the exporter show a negative number.
* **An unknown value is an empty cell**, never 0 (a 0 would look like a real value).
* Yes/no columns hold `1` or `0`.

### How they are made

1. **Live, while the drone flies:** the **flight recorder** (`sim_demo flight_recorder`) listens to the drone's
   MAVLink messages on its own port, UDP `127.0.0.1:14555` (demo.sh asks MAVProxy for this extra copy). It writes
   `telemetry.csv`, `events.csv`, `phases.csv` and the aim/release part of `targets_and_drops.csv`. Every second it
   hands the new rows to Linux (safe if the recorder itself crashes) and every 5 seconds it forces them onto the disk
   (safe even if the whole Ubuntu machine freezes: at most the last 5 seconds are lost). It **only listens**: it never
   sends anything to the drone, so it cannot change the flight.
2. **At the end:** the **exporter** (`csv_export`) adds `photos.csv`, `detections.csv` and the landing points and
   errors in `targets_and_drops.csv` from the scorer's files. It never overwrites what the recorder wrote live. It runs
   once at the landing and once more after the simulator has stopped (the scorer writes its final files then, so a
   detection that finished after the landing is in the CSV files too).

Because the recorder needs only MAVLink, it works the same when the team's code runs **on the real Jetson** (over the
USB cable or plain SSH over Ethernet/Wi-Fi: the PC then cannot see the Jetson's ROS topics, but it still hears the
autopilot, which passes the team's status texts on), and later with the **real drone's telemetry radio**:

```bash
# simulator (what demo.sh does)
python3 -m sim_demo.flight_recorder --run-dir ~/.astra_sim/runs/my_test --url udpin:127.0.0.1:14555
# a telemetry radio on a USB port
python3 -m sim_demo.flight_recorder --run-dir ~/flight_logs/field_day_1 --url /dev/ttyUSB0 --baud 57600
# a copy from Mission Planner (Ctrl+F > MAVLink > forward to UDP 14555) or another MAVProxy
python3 -m sim_demo.flight_recorder --run-dir ~/flight_logs/field_day_1 --url udpin:0.0.0.0:14555
```

Stop it with Ctrl+C: it closes the files cleanly.

### `telemetry.csv`: where the drone is, 2 rows per second

| Column | Meaning |
|---|---|
| `lat_deg`, `lon_deg` | position (GPS) |
| `alt_msl_m` | height above sea level |
| `height_above_home_m` | height above the take-off point |
| `heading_deg` | which way the nose points (0 = north, 90 = east) |
| `groundspeed_mps`, `climb_rate_mps` | speed over the ground; climbing (+) or sinking (-) |
| `roll_deg`, `pitch_deg`, `throttle_pct` | how the drone is tilted, how hard the motors work |
| `flight_mode`, `armed` | AUTO, GUIDED, RTL, LAND ...; motors on (1) or off (0) |
| `landed_state` | on_ground / in_air / taking_off / landing (empty if the autopilot does not send it) |
| `battery_v`, `battery_pct` | battery voltage and charge |
| `gps_fix_type`, `gps_fix`, `gps_satellites` | GPS quality as a number and in words, satellites seen |
| `current_wp`, `last_reached_wp` | the mission item the drone flies to, the last one it reached |
| `mission_phase` | WAITING, ARMED, TAKEOFF, SEARCH, HOLD, TARGETS, DROP, RTL, LANDED |
| `position_age_s` | how old the position is (a big number = the position stopped coming) |

### `events.csv`: everything that happened, one row each

`kind` says what it is: `statustext` (a text message), `armed`, `disarmed`, `mode` (flight mode change),
`current_wp`, `wp_reached`, `servo` (a release servo moved), `release`, `phase`, `link_lost` / `link_back` (no
MAVLink for 5 s / back again), `vehicle`, `recorder_start` / `recorder_stop`. For text messages:

| Column | Meaning |
|---|---|
| `autopilot_text` | a text from ArduPilot itself (e.g. `PreArm: ...`, `Mission: 3 DigiCamCtrl`) |
| `team_text` | a text from the team's code on the Jetson (e.g. `Detected tent`, `Aim tent 36.2150261,-96.0067984`) |
| `sender` | `autopilot`, `team` or `gcs` (a ground station such as Mission Planner) |
| `src_system`, `src_component` | the MAVLink address of the sender (autopilot = 1/1; MAVROS = 1/191 or 1/240) |
| `severity`, `severity_name` | 0 EMERGENCY ... 3 ERROR, 4 WARNING ... 6 INFO, 7 DEBUG |

### `phases.csv`, `photos.csv`, `detections.csv`, `targets_and_drops.csv`

* `phases.csv`: when each mission phase started (`phase_code`, `phase`, `detail`). The live file uses the same
  phase rules as the score report's time line (`sim_demo/phases.py`); the demo window's phase lines
  (`phases.jsonl`, `phase_monitor`) use slightly different ones. They agree within a second for HOLD, TARGETS, DROP,
  RTL and LANDED, but SEARCH starts about 20 s earlier in `phases.csv` (at the first waypoint after take-off, not at
  the camera-trigger item), and `phases.csv` has a row for EVERY change (TARGETS and DROP once per target). The
  arming time comes from the autopilot's heartbeat (once per second), so it can be up to about 1.5 s after the
  autopilot's `Arming motors` text.
* `photos.csv`: one row per photo: position, `height_above_home_m`, `heading_deg`, `zoom_x`, `gimbal_pitch_deg`,
  `camera_model` (from the camera's sidecar `.json`), `checked_by_detector`, `detections`, `best_class`,
  `best_confidence`, `footprint_en_m` (the ground the photo covers: corner points "east,north" in metres from home).
  **Two heights, on purpose:** `alt_msl_m` is the height above **sea level** at the shutter time, taken from the
  drone's own MAVLink (`telemetry.csv`, worked out between the two rows around that moment; empty without telemetry).
  `alt_wgs84_m` is the height the camera wrote into its sidecar. The team's `video_cam` calls it `alt_msl_m`, but it
  copies it from MAVROS (`/mavros/global_position/global`), which gives the height above the **WGS-84 ellipsoid** (a
  mathematical shape of the Earth), not above sea level. On the Oklahoma field the two differ by about **29.6 m**
  (example: `alt_msl_m` 240.17, `alt_wgs84_m` 210.6 for the same photo). Use `alt_msl_m` or `height_above_home_m`.
* `detections.csv`: one row per detection box: `photo`, `class` (person / tent), `confidence` (0 to 1), the box
  (`box_cx_px`, `box_cy_px`, `box_w_px`, `box_h_px`), where the photo was taken (`photo_lat_deg`, `photo_lon_deg`),
  `waypoint_index`, `false_alarm` (1 = the score report says no real target was there).
* `targets_and_drops.csv`: one row per payload (`person` = the water bottle for the hiker, `tent` = the beacon):
  where `main_controller` aimed (`aim_*`), when the servo opened (`release_*`, `servo_channel`, `servo_open_pwm_us`,
  where the drone was), where the payload landed (`landing_*`), the real target (`truth_*`) and the errors
  `aim_error_m` and `drop_error_m` (the same numbers as `score_report.md`).

## 4. The run summary (`run_summary.md`)

At the end of every run demo.sh writes `run_summary.md` (for people) and `run_summary.json` (for programs) and
prints where they are. The summary says:

* **what ran:** the git commit (and whether there were uncommitted changes), the profile, the world, the ArduPilot
  version, where the team code ran (this PC or the Jetson, over USB or SSH), the detector, its model file with
  SHA-256 and the **detection backend** (for example "PyTorch on CUDA (RTX 4060)"; the field runs TensorRT on the
  Jetson, so anything else is listed as a difference from the field);
* **the result** (landed, timed out, stopped with Ctrl+C, failed early) and the score;
* **how long each phase took**;
* **every warning and error line** found in the logs, grouped (the same line 50 times is one row with "50 times"),
  with `file:line` so you can open the log right there. Lines the docs call normal (like the zoom readback warning
  with `--zoom survey`) are listed apart. Lines written **after the run ended** (programs being stopped print many
  "process has died" lines) are not counted; one sentence names how many there were and the first ones;
* **the Jetson** (only when the team code ran on it, over the USB cable or plain SSH): its power mode and
  `jetson_clocks` (from `jetson/facts.env`) and how busy it was during the flight (GPU, CPU, memory, temperature, from
  `jetson/tegrastats.log`). For the Jetson tools: write `JETSON_HOST=<address>`, `JETSON_LINK=usb` or `ssh` and
  `TEAM_STACK=jetson` into `demo.env`, and `facts.env` (`KEY=VALUE` lines; keys with NVP, POWER or CLOCK in the name
  are shown as the power setup), `tegrastats.log` and `differences.txt` (one item per line) into `<run>/jetson/`;
* **what still differs from the field**: the Jetson tools' own list (`jetson/differences.txt`) when there is one,
  else a short list (for example "the team code ran on this PC, not on the Jetson", "the autopilot is SITL");
* **where every file is**, with the number of rows of each CSV file and its Windows path.

**When it is written:** at the landing (you see it in the demo window before "Press Enter"), and once more after the
simulator has stopped, with the scorer's final files (the result stays the same).

If the run **crashes or its window is closed**, the flight recorder notices that demo.sh is gone and writes a
**PARTIAL** summary (it says so at the top, and why). A partial summary never replaces a complete one. (Checked: the
recorder runs in its own session, and such a process keeps running when the `wsl.exe` that started it ends; a plain
background process does not. Closing a real demo window was not tested.)

**The git commit:** the simulator runs from a Linux copy of the repo without `.git`, so the summary asks git in the
checkout the copy came from. It finds it in `ASTRA_SOURCE_REPO`, in `ASTRA_WINDOWS_REPO` (set by the Windows `.bat`
launchers), or in the copy's `.astra_source` file (written by `setup/03_build.sh` once that change is in). Otherwise
tell it once per window:
`export ASTRA_SOURCE_REPO="/mnt/d/Projects ext/SUAS/gazebo/ODLC_Machine_Inferencing_System_2025-2026"` (without it the
summary says "unknown" and why). "with uncommitted changes" counts changed files AND new files git does not ignore.

## 4b. Runs started by START OVER

START OVER (the demo panel's button, or `bash simulation/scripts/start_over.sh`) puts a fresh drone on the runway and
starts a **new run folder** while the windows stay open.

* The run that was flying ends with its own records: demo.sh stops its flight recorder and writes its summary with the
  result `ended by START OVER (the flight goes on as run ...)`.
* The new run folder has `start_over.env` (it names the run it continues, `PREVIOUS_RUN=`); its summary takes the
  settings it does not have (detector, camera, profile) from that run.
* Its own flight recorder and its summary at the landing come from START OVER itself (`run_reset.py` calls
  `sim_demo run_records`). **Until that small change to `run_reset.py` is applied (it is waiting for the lead), a
  START OVER run has no live CSV and no summary**; make them afterwards with `bash simulation/scripts/logs.sh summary`
  (photos, detections and drops are complete; `telemetry.csv` stays empty).
* Its `sim.log` stays in the FIRST run's folder (the simulator itself was not restarted), and START OVER's own logs
  (`sitl.log`, `mavproxy.log`, `team_stack.log`, ...) have no date on every line yet.

## 5. The table of all runs (`runs_index.csv`)

`~/.astra_sim/runs/runs_index.csv` has **one row per run**: run, world, start time, duration, result, exit code,
profile, detector and model (+ SHA-256), ArduPilot version, where the team code ran, git commit, targets found,
aim and drop errors for the tent and the hiker, photos, number of warnings and errors, run folder. Open it in Excel to
compare runs. Writing a summary again for the same run replaces its row (never two rows for one run).

## 6. The `logs.sh` command

| Command | What it does |
|---|---|
| `logs.sh list [N]` | the last N runs (default 20): result (`STILL RUNNING` for a run that is going), flight time, targets found, CSV yes/no, summary yes/no |
| `logs.sh summary [RUN] [--refresh]` | print the run summary (writes one first if the run never wrote one and is over; `--refresh` writes it again from the files, same result) |
| `logs.sh errors [RUN] [--all] [--errors-only]` | every warning/error line of every log as `file:line text` |
| `logs.sh tail LOG [RUN] [-n N] [-f]` | the end of a log: `sim`, `team_stack`, `demo`, `recorder` or a file name; `-f` keeps following it |
| `logs.sh csv [RUN]` | the CSV folder as a Linux and a Windows path, with the rows of each file |
| `logs.sh export [RUN ...]` | (re)write the CSV files of a run, also older runs |
| `logs.sh bundle [RUN] [--photos] [--max-mb N] [--out FILE.zip]` | one zip for a teammate (section 7) |
| `logs.sh path [RUN]` | just the run folder: `cd "$(bash simulation/scripts/logs.sh path)"` |

`RUN` is the newest run when you leave it out. Otherwise: the folder name (`20261031-110240_runway`), a part of it
(`110240`), a number (`1` = newest, `2` = the one before) or a full path.

**A run that is still going** (its flight recorder runs, or one of its logs changed in the last minute) is only
looked at, never changed: `summary` shows it with "RUN IN PROGRESS" on top and writes nothing, `export` and
`summary --refresh` refuse (they say why), `bundle` zips the files as they are, with a note in the zip.

## 7. Sending a run to a teammate

```bash
bash simulation/scripts/logs.sh bundle
```

It makes `~/.astra_sim/bundles/<run>.zip` and prints its Windows path. Inside: the logs, the CSV files, the reports,
the summary and a `BUNDLE_MANIFEST.txt` that lists what was left out and why. Left out unless you ask:

* the photos (`--photos` adds them; they are big);
* any file bigger than 20 MB (`--max-mb 100` raises the limit);
* **never**: files whose name looks like a secret (token, password, key, `.pem`, SSH keys like `id_ed25519`,
  `known_hosts`, `target.env`), and text files with a line that looks like a private key, a token or
  `password = ...`. The manifest names each one and the line, so you can look at it yourself.

## 8. CSV files for older runs (and on Windows)

Runs made before the flight recorder existed have no `csv/` folder. Make one:

```bash
bash simulation/scripts/logs.sh export 20261031-101431_runway       # one run
ros2 run sim_demo export_csv ~/.astra_sim/runs/20261024-125414_runway
python3 -m sim_demo.csv_export ~/.astra_sim/runs/*_runway           # many at once (no ROS needed)
```

On **Windows** (Python 3, no ROS), from the repo folder:

```powershell
py -3 ros2_ws\src\simulation\sim_demo\sim_demo\csv_export.py "\\wsl.localhost\Ubuntu-22.04\home\<your user>\.astra_sim\runs\20261031-101431_runway"
```

An older run has no positions over time, so its `telemetry.csv` has only the header line. If you have a MAVLink log
of the same flight (a `.tlog` from Mission Planner or MAVProxy), add `--tlog FILE` and the exporter replays it through
the flight recorder to fill `telemetry.csv`, `events.csv` and `phases.csv`:

```bash
python3 -m sim_demo.csv_export ~/.astra_sim/runs/20261024-125414_runway --tlog ~/flight_logs/2026-10-24.tlog
```

* It works also after an earlier export (a header-only `telemetry.csv` is filled).
* It **refuses** (and says why) when `telemetry.csv` already has rows (the live recorder or an earlier replay wrote
  them), or when the log's times do not match the run ("covers 2026-10-31 10:02 to 10:15, but this run happened ...":
  another day's log). Add `--force` when you are sure.
* What the replay heard goes to `flight_recorder_tlog.json`; the live `flight_recorder.json` is never changed.
* `--force` without `--tlog` rebuilds `events.csv` / `phases.csv` from the run's `.jsonl` files; it never empties a
  `telemetry.csv` that has rows (without a log there is nothing to rebuild it from).

**How do I know it worked:** the command prints `wrote events.csv, phases.csv, ... (93 photos, 28 detections)`
(with `--tlog`: `note: telemetry, events and phases replayed from ... (566 telemetry rows ...)`).

## 9. Options of `demo.sh`

| Option | What it does |
|---|---|
| (nothing) | the flight recorder runs on UDP 14555 (14565 with `--sitl-instance 1`, and so on), the summary is written at the end |
| `--no-recorder` | no live CSV recording (the CSV files of photos/detections/drops and the summary are still written) |
| `--recorder-port P` | the recorder listens on UDP P instead (1024-65535; a port the simulator itself uses is refused) |

demo.sh checks one and a half seconds after starting the recorder that it still runs; if not, it says so (with the
recorder's own reason) and the flight goes on without live CSV.

## 10. When something is missing

| What you see | Why | Fix |
|---|---|---|
| `[demo] WARNING: UDP 14555 (the flight recorder's port) is already in use` | the recorder of an earlier run is still running | `pkill -f sim_demo.flight_recorder`, then run demo.sh again (or `--recorder-port 14565`) |
| `telemetry.csv` has only its header | the recorder did not run (older run, `--no-recorder`, the port was busy, a START OVER run before the `run_reset.py` change) | for an older run that is normal; with a `.tlog` of that flight: `python3 -m sim_demo.csv_export RUN --tlog FILE` |
| `[demo] WARNING: the flight recorder stopped right after it started` | pymavlink missing for that Python, or the port taken | the lines under the warning (all: `flight_recorder.log`); `/usr/bin/python3 -m pip install --user pymavlink` |
| `[demo] ERROR: --recorder-port ... is a port the simulator itself uses` | the port is the ground station's, MAVROS's, the panel's or the Lattice bridge's | another port, e.g. `--recorder-port 14565` |
| `[export_csv] ERROR: .../telemetry.csv already has N rows ...; --tlog would replace them` | the run is already recorded | nothing to do; to replace it anyway add `--force` |
| `[export_csv] ERROR: X.tlog covers ..., but this run happened ...` | the log is from another flight | the `.tlog` of this flight; `--force` if you are sure |
| `[logs] ERROR: ... is still going (...)` | `export` / `summary --refresh` on the run that is flying now | wait for its end, or name an older run (`logs.sh list`) |
| the summary says `ended by START OVER` | START OVER took the simulator over during this run | normal: the flight went on in the run it names |
| `[flight_recorder] WARNING: no MAVLink heard on udpin:127.0.0.1:14555 in 90 s` | MAVProxy has no output to 14555 | look for `extra udp:127.0.0.1:14555` in the `MAVProxy outputs` line of `sim.log`; rebuild with `bash simulation/setup/03_build.sh` |
| `[flight_recorder] WARNING: no heartbeat from the autopilot for 5 s` | the simulator stopped or froze | read the end of `sim.log` (`logs.sh tail sim`) |
| `[flight_recorder] ERROR: the Python package pymavlink is not installed` | the setup was not run for this Python | `/usr/bin/python3 -m pip install --user pymavlink` |
| `[demo] WARNING: the run summary could not be written` | a broken file in sim_demo (the Python error is printed under the warning), or the disk is full | read `run_summary.log` in the run folder; then `bash simulation/scripts/logs.sh summary RUN` |
| the summary says `git ... unknown` | the Linux copy of the repo has no `.git` | `export ASTRA_SOURCE_REPO=...` (section 4) before demo.sh |
| `[logs] ERROR: No run matches "..."` | a typo, or the run was deleted | `bash simulation/scripts/logs.sh list` |

Exit codes: `flight_recorder` 0 stopped normally, 2 pymavlink missing, 3 the MAVLink connection could not be opened,
4 the run folder cannot be written, 5 bad options; `csv_export`, `run_summary` and `run_records` 0 done, 1 failed or
refused (reason and fix printed), 2 bad usage; `run_summary running RUN` 0 = still going, 1 = over.
