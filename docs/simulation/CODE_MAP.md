<!-- PURPOSE: a short, plain guide for teammates to the ASTRA code map: what it shows, its Wiring view (how the programs talk, with every port), how to watch the team's code run live during a simulator flight, where a run is saved and how to replay it, how to open the shareable page, how to keep the map up to date, and what tracing changes. The full reference is simulation/codemap/README.md. -->

# The code map: watch our code run

The **code map** is a web page that draws all of our software as boxes you can open, like folders:

1. **Systems**: the big parts, such as the *team flight code*, the *team's new video pipeline*, last
   year's *legacy (2025) photo pipeline*, the *simulator*, the *run logs*, *Jetson in the loop*, the
   *ground station*, and the programs we did not write (Gazebo, ArduPilot, MAVProxy, MAVROS, Mission
   Planner, and the team's camera and detection programs, whose code is not in this repo).
2. **Packages** inside a system (for example `main`, `video_cam`, `detection`).
3. **Files** inside a package (for example `main_controller.py`).
4. **Functions** inside a file (for example `MainController.image_result_cb`).

Click a box once to read **what it does in plain words**, who calls it, what it sends and receives,
and its source code. Click it again (or press Enter) to open it.

While the simulator flies, every function that runs **flashes green**. The file, the package and the
system it belongs to glow too, and count the calls. So you can watch the drone's brain work: photos
being taken, the detector finding the hiker, the mission controller picking a target, the drop.

## The Wiring view: how the programs talk

Press **Wiring** (above the drawing, next to *Drill down* and *Overview*). Now you see one picture of the
whole twin: each program is a box, and each line is a connection, with the **port written on it**.

- **The colour says what kind of message travels there:** orange = MAVLink (the autopilot's language),
  blue = ROS 2, purple = Gazebo, pink = the camera, teal = web pages, brown = files.
- **The line style says how finished it is:** solid = works today; dashed = being built right now;
  dotted with a PROPOSAL tag = our idea for the team, not adopted by them; grey with a LEGACY 2025 tag =
  last year's photo pipeline (still in the repo, no longer used at the field).
- **Click a box** to read what the program does and what it stands for on the real drone. **Click a
  line** to see its port and the files where we checked that port.
- **Press "Play the mission"** to follow one flight step by step (13 steps). "Show everything" or Esc
  clears the lights.
- Three things you can learn here at a glance: MAVROS has its own autopilot port (TCP 5763, SERIAL2, like
  the Jetson on the Pixhawk's TELEM2), the team's camera video arrives over RTSP (TCP 8554) and goes to the
  detector on the ROS topic `/zr10/image_raw`, and how the detections then reach `main_controller` is still
  the team's decision (the dotted proposal).

The picture is made from `simulation/codemap/systems.yaml`. To change a box or a line, edit that file and
run `python3 simulation/codemap/extract.py --check` (how: [`simulation/codemap/README.md`](../../simulation/codemap/README.md#where-a-file-goes-systemsyaml)).
Tip: add `#wiring` to the address to open it directly, for example http://127.0.0.1:8090/#wiring .

## Watch it live (on the team laptop)

You need the simulator set up already (see [`simulation/README.md`](../../simulation/README.md)).

1. Open an **Ubuntu-22.04** terminal.
2. Run the traced demo (it is the normal search demo, plus the code map):

   ```bash
   bash ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/scripts/demo_traced.sh
   ```

   Every option of `demo.sh` works here too, for example `demo_traced.sh runway --detector truth`.
3. Open **http://127.0.0.1:8090/** in a browser. A browser on Windows works too.
4. What you should see: a green **"Live connected"** label at the top, and boxes lighting up as the
   drone takes off, takes photos and flies to the targets.
5. When the drone has landed, the demo stops by itself. The run was **recorded**. The last lines in
   the terminal say where the recording is and how to replay it. For the command in step 2 (the Linux
   copy) the recording is saved in two places:
   - `~/.astra_sim/codemap/recordings/<date-time>.jsonl` (with `<date-time>.health.json` next to it), and
   - a copy in the run's own folder, `~/.astra_sim/runs/<time>_<world>/codemap_trace.jsonl` (its time is a few seconds later).

   (If you started `demo_traced.sh` from the Windows checkout, `/mnt/d/...`, the first one goes to
   `simulation/codemap/recordings/` of that checkout instead.)
6. To play it again later, start the map server and open the **Replay** tab:

   ```bash
   python3 ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/codemap/server.py
   ```

   Then open http://127.0.0.1:8090/ . The Replay tab lists the recordings of both folders above:
   `simulation/codemap/recordings/` and `~/.astra_sim/codemap/recordings/`. A recording in any other
   folder: add `--recordings <that folder>` to the command.

## Look at it without the simulator

- **The shareable page:** `simulation/codemap/dist/astra_code_map.html` is one file with the whole map
  and one recorded flight inside. Open it in a browser (double-click). It replays the flight; it never
  needs the internet except for its fonts and its drawing library.
- **Just the map:** in an Ubuntu terminal, `python3 simulation/codemap/server.py`, then open
  http://127.0.0.1:8090/ . Nothing lights up live (no tracers run), but the Replay tab plays the old
  recordings of both recording folders (step 6 above).

## Keep the map up to date

The map is made by reading the code. After you add, rename or move a file or a function, run this from
the repo folder (Windows or Ubuntu):

```bash
python3 simulation/codemap/extract.py --check
```

It writes `simulation/codemap/codemap.json` and prints how many boxes and links it found. The last lines
list problems (a file with no package, a name used twice); "exit code 0" means none.

If the page shows a box with **no plain explanation**, you can add one: put a line for its id in one of
the `simulation/codemap/explanations/*.json` files (how: [`simulation/codemap/README.md`](../../simulation/codemap/README.md#explanations-the-plain-words)),
then run `extract.py` again. The last lines it prints must not list any "explanation ids that match no block"
(that means an id is misspelled, or its function was renamed).

Two things you will see in the explanations:

- Boxes of **last year's photo path** (`video_cam`, the `mapping` photo trigger, `detection`/`new_od`, and the
  simulator helpers made for them, such as `photo_trigger` and `truth_detector`) start with
  **"Legacy (2025) photo pipeline"**. Since 2026-10-31 the field camera and detection path is the team's new
  video pipeline (the ZR10 camera with `zr10_ros2`, `zr10_stream` and a detection package); the old code stays
  in the repo for the legacy option.
- The explanations of the code map's own code are in `simulation/codemap/explanations/codemap.json`. Until
  2026-10-31 a `.gitignore` line hid that file from git by mistake; it is fixed now, so remember to add the file
  to git the next time the code map is committed.

To make a new shareable page from the newest recording (it looks in both recording folders, and its last
lines name the recording it used):

```bash
python3 simulation/codemap/build_artifact.py
```

For one recording you choose: add `--recording <path to the .jsonl file>`.

## Is this safe for the real drone?

Yes. The tracing is **off unless you switch it on**, and only the simulator scripts can switch it on:

- It only starts when the setting `ASTRA_TRACE=1` is present. `demo_traced.sh` sets it. `launcher.sh`,
  the flight code's ROS packages and the Jetson's Docker files never set it, and do not even mention it.
  (A search of the whole repo on 2026-10-31 found the word only in the code map's own files, the
  simulator scripts `demo_traced.sh` and `_common.sh`, this guide, and one label in the simulator's
  `sim_demo/run_summary.py` that only names the recording file; it sets nothing.)
- No team file was changed for the code map. The only hook is three lines in the **simulator's**
  `simulation/scripts/_common.sh`, and they do nothing without `ASTRA_TRACE=1`.
- When it is on, it only counts calls and sends a tiny message at most every 0.1 s to this laptop
  (`127.0.0.1`). It never waits: a message that cannot go at once is dropped.

What tracing *does* change, only while it is on:

- The traced programs run a little slower (numbers in
  [`simulation/codemap/README.md`](../../simulation/codemap/README.md#how-much-slower-does-tracing-make-things)).
- One rare case: a Python function that calls itself again and again, about 1,000 levels deep (that is
  Python's limit), gets Python's "too deep" error (`RecursionError`) 1 level sooner, and that part of the
  program stops lighting up on the map. Code that never goes that deep is not affected.

## If something looks wrong

| What you see | What to do |
|---|---|
| The page says **"Waiting"** and nothing lights up | The server runs but no traced program sends. Start the flight with `demo_traced.sh` (not `demo.sh`). |
| The page does not open at all | The server is not running. Start `demo_traced.sh`, or `python3 simulation/codemap/server.py`. |
| `the code map server did not start ... another program already uses port 8090 or UDP 8091` | Another code map server runs. Close it, or pick free ports: `demo_traced.sh --map-port 8095 --trace-port 8096`, then open http://127.0.0.1:8095/ . |
| Some function you know ran never lights up | The map is older than the code. Run `python3 simulation/codemap/extract.py --check`, then `bash simulation/setup/03_build.sh`. The list of calls the server could not place is at http://127.0.0.1:8090/health while it runs. After a run it is in `~/.astra_sim/codemap/recordings/<date-time>.health.json` (a run started from the Windows checkout: `simulation/codemap/recordings/<date-time>.health.json`), and in the run's folder as `codemap_health.json`. |
| The Replay tab does not list a run | The server lists only `simulation/codemap/recordings/` and `~/.astra_sim/codemap/recordings/`. For another folder: `python3 simulation/codemap/server.py --recordings <that folder>`. The server must be the new one: after updating the code, run `bash simulation/setup/03_build.sh` once. |
