<!-- PURPOSE: the reference guide of the ASTRA code map (simulation/codemap): what it is, its Wiring view (how the programs talk, with every port), how to watch the team's code run live during a simulator flight, how to rebuild the map and the shareable artifact page, how systems.yaml places files and draws the wiring, how the tracer stays off on the real drone, what tracing costs, what every file in this folder is, the shared contract (ids, file formats, live events), the tests, and what was verified on 2026-10-31. The short teammate guide is docs/simulation/CODE_MAP.md. -->

# ASTRA code map

The code map is a web page that shows **all of our software as boxes you can open**, one level at a time:

| Level | Example | What a box is |
|---|---|---|
| 1. Systems | *Team flight software (Jetson)*, *Team's new video pipeline*, *Legacy (2025) photo pipeline*, *Simulated world*, *Simulator control*, *Run logs*, *Jetson in the loop*, *Ground station*, ... plus the programs we did not write: Gazebo, ArduPilot SITL, MAVProxy, MAVROS, Mission Planner, Lattice, and the team's zr10_node, zr10_stream and detection package (their code is not in this repo) | a big part of the project (17 systems, 11 outside programs) |
| 2. Packages | `ros2_ws/src/main`, `ros2_ws/src/video_cam`, `simulation/scripts` | a ROS package or a folder |
| 3. Files | `main_controller.py`, `demo.sh`, `sim.launch.py` | one source file (Python, bash, C#, launch, XML, YAML ...) |
| 4. Functions | `MainController.image_result_cb`, `SIYINode.camera_trigger_callback` | one function or method (classes are drawn as boxes around their methods) |

Lines between boxes say how they are connected: a function **calls** another, a file **imports** another, a node
**publishes** or **subscribes** to a ROS topic, a launch file **launches** a program, a script **runs** one,
programs talk **MAVLink** or **HTTP**.

Click a box once: the side panel says **what it does in plain words**, when it is called, its inputs, outputs and
side effects, who calls it and what it calls, and shows its source code. Click it again (or press Enter): it opens.

While the simulator flies with tracing on, **every function that runs flashes green**, and its file, package and
system glow and count the calls. Programs that are running get a RUNNING badge. A run is recorded, so it can be
replayed later, or embedded in one shareable page (the "artifact").

## The Wiring view: how the programs talk

The **Wiring** button (next to *Drill down* and *Overview*) shows one drawing of the whole twin as it is today:
every program is a box (Mission Planner, MAVProxy, ArduPilot SITL, Gazebo, MAVROS, the team's nodes, the team's new
video pipeline, the simulated ZR10 camera, the Jetson relay, the CSV flight recorder, the referees, the code map
itself), and every connection is a wire with its **transport and port** written on it (for example *TCP 5763 ·
SERIAL2*, *UDP 14555*, *RTSP · TCP 8554 · /main.264*). Open it directly with `#wiring` at the end of the address
(`http://127.0.0.1:8090/#wiring`, or `dist/astra_code_map.html#wiring`).

| What you see | What it means |
|---|---|
| wire colour | the kind of message: orange MAVLink, blue ROS 2, purple Gazebo, pink the camera (SIYI SDK, RTSP video, HTTP photos), teal web pages and the code map, brown files |
| solid line or border | works in the twin today |
| dashed | being built right now (the virtual Jetson's relay, the simulated ZR10's RTSP stream, the team's video code in the twin) |
| dotted, tag PROPOSAL | our proposal for the team (how detections reach main_controller); the team has not adopted it |
| grey, tag LEGACY 2025 | last year's photo pipeline (photo trigger, siyi, the photo folder, new_od): still what today's default demo runs, no longer the field system |
| tag TEAM CODE · NOT IN REPO | the team's new video pipeline: zr10_node, zr10_stream, the detection package |
| the blue rail | the ROS 2 graph; a short line onto it means "takes part in ROS 2" (click it for the topics) |

Click a box: the panel says what it is, what it stands for on the real drone, every wire it uses, and offers
"Open ... in the code map". Click a wire: its transport and port, what travels on it, and the files in which the
port was checked. The colour buttons above the drawing light every wire of one kind. **Play the mission** follows
one flight in 13 steps (Back, Next, the step numbers, Show everything; Esc clears). While a traced run plays, a box
glows green when its programs' code runs and shows RUNNING when its program runs.

The drawing is **made from `systems.yaml`**, not drawn in the page: every link there with `wire:` is a wire, and the
`wiring:` section places the boxes and holds the tour (see "Where a file goes" below). `extract.py` copies both into
`codemap.json` as `"wiring"` and checks them (`--check` fails on a wire no box can hold, an unknown kind or status, a
wire id used twice, or a tour step that names something that does not exist). A test checks that every port written
on a wire (except proposals) appears in the files listed under its `code:`.

## Watch it live

On the team laptop (Ubuntu-22.04 terminal, simulator already set up, see [`simulation/README.md`](../README.md)):

```bash
bash simulation/scripts/demo_traced.sh          # from the repo folder (Windows checkout or Linux copy)
```

Then open **http://127.0.0.1:8090/** in a browser (a browser on Windows works too: WSL forwards localhost).

What happens, step by step:

1. `demo_traced.sh` starts the code map server `server.py` (it listens for the tracers on UDP `127.0.0.1:8091`,
   serves the page on TCP `127.0.0.1:8090`, and records the run).
2. It runs the normal search demo, `simulation/scripts/demo.sh`, with `ASTRA_TRACE=1` and
   `simulation/codemap/trace` at the front of `PYTHONPATH` (of the Linux-side copy
   `~/ODLC_Machine_Inferencing_System_2025-2026`, which is what the simulator runs). The demo itself runs exactly
   as usual. Every option of `demo.sh` works: `demo_traced.sh runway --detector truth --keep`.
3. Every Python program of the run counts the calls of our functions and sends them to the server every 0.1 s;
   every simulator bash script reports its functions too.
4. The page shows a green **Live connected** label and the boxes light up.
5. After the landing (or Ctrl+C) the server stops. The recording is `<YYYYmmdd-HHMMSS>.jsonl`, with
   `<same>.health.json` next to it (what the server could not place), in one of two folders:
   - `~/.astra_sim/codemap/recordings/` when `demo_traced.sh` ran from the Linux-side copy
     (`~/ODLC_Machine_Inferencing_System_2025-2026`, which has no `.git` and is overwritten by `03_build.sh`);
   - `simulation/codemap/recordings/` of the checkout when it ran from a git checkout (the Windows one, `/mnt/...`).

   Both files are also copied into the demo's run folder as `codemap_trace.jsonl` and `codemap_health.json`. The
   last lines of the script name the recording and the command that replays it.
6. Replay a run later: `python3 simulation/codemap/server.py`, open http://127.0.0.1:8090/, **Replay** tab. Without
   `--recordings` the server lists BOTH folders above; `--recordings DIR` lists DIR instead.

Options of `demo_traced.sh` (every other option goes to `demo.sh`): `--map-port N` (default 8090),
`--trace-port N` (default 8091), `--no-record`, `--record-dir DIR`, `--map-only` (start only the server and print
the `export ...` line that switches tracing on in your own terminals, for runs started by hand with
`start_sim.sh` and `./launcher.sh sim`), `--dry-run`, `--help`.

## Look without the simulator

| You want | Do this |
|---|---|
| The map and old recordings in a browser | `python3 simulation/codemap/server.py`, open http://127.0.0.1:8090/, **Replay** tab. It lists `simulation/codemap/recordings/` and `~/.astra_sim/codemap/recordings/` (runs started from the Linux-side copy; `$ASTRA_SIM_HOME/codemap/recordings` when `ASTRA_SIM_HOME` is set); `--recordings DIR` lists DIR instead. |
| One file you can send to anyone | open `simulation/codemap/dist/astra_code_map.html` (made by `build_artifact.py`, below). It has the map and one recorded run inside, replays it by itself, and makes no network requests except its fonts and its layout library (it has a built-in layout when that library cannot load). |
| Test the page with small sample data | `cd simulation/codemap/web && python3 -m http.server 8000`, open http://127.0.0.1:8000/ (shows a SAMPLE DATA label) |

## Rebuild the map

```bash
python3 simulation/codemap/extract.py --check        # Windows: py -3 simulation\codemap\extract.py --check
```

It reads every file of the repo (no ROS needed, standard library only, about 10 s) and writes
`simulation/codemap/codemap.json`. It prints the number of boxes and links of each kind, how many boxes have a
plain explanation (per kind and per system), and notes. `--check` makes it fail (exit 1) when a file belongs to
no package or system, or an id repeats. Other options: `--no-source` (leave the source code out), `--out PATH`,
`--quiet`.

**When:** after you add, move or rename files or functions. A traced run against an old map still works: a
function whose file changed is placed by its name when the name is unique in that file; truly new functions are
listed as "could not place" in `/health` and in `<recording>.health.json`. Then run `bash simulation/setup/03_build.sh`
so the Linux copy (what the simulator runs) gets the new map.

### Where a file goes (systems.yaml)

`systems.yaml` is written by hand: the 17 systems with a plain summary each, which folders ("packages") belong to
which system, the outside programs (with the patterns that recognise them in the process list) and the network
links between the big programs. A folder with a `package.xml` is a ROS package and owns everything under it;
`some/folder` owns only its own files, `some/folder/**` its sub-folders too. A new top-level folder needs a line
there, or `extract.py --check` fails and names the file.

Three more rules (added 2026-10-31):

- **A ROS package inside a `some/folder/**` package** that no line names belongs to that folder's system. That is
  how folders that are still being built (`simulation/video/**`, `simulation/jetson/**`) already land in the right
  system: a package another workflow adds there tomorrow passes `--check` without a new line. A folder named there
  that has no files yet is only a note.
- **`files:`** claims single files for a system even when they live in another system's folder, for example the
  flight recorder (`sim_demo/flight_recorder.py`) for *Run logs* while the rest of `sim_demo` stays in *Demo
  screens and scoring*. The claimed files of one folder become one package block, id `pkg:<folder>@<system key>`
  (its label and summary come from `package_info: "<folder>@<key>"`). `*` and `?` work but never cross a `/`
  (`simulation/profiles/jetson_*.yaml`). A claimed file that does not exist yet is only a note; a file claimed by
  two systems is a problem.
- **Wires of the Wiring view** are the `links:` that have `wire:` (the kind of message: `mav`, `ros`, `gz`, `cam`,
  `web`, `files`), an `id` (the tour uses it), `via` (transport and port, written on the wire; parts separated by
  ` · ` are stacked when the wire is short), `status` (`current`, `being built`, `proposal`, `legacy`, `optional`),
  `code` (the repo files where the port was checked), `explain` (plain words), and optionally `arrow`
  (`to`, `from`, `both`), `wire_from` / `wire_to` (draw it from / to this box), `path` (corner points `"x,y x,y"`),
  `label_at` (`"x,y [start|middle|end] [stack]"`) and `offset` (move a straight wire sideways). `map: false` makes a
  wire that is not also a line of the block diagrams (the code already shows that link, or an end is not a block).
  A wire goes from the box whose `node` / `members` hold the link's `from` id to the box that holds its `to` id. The
  `wiring:` section has `size`, `kinds`, `statuses`, `zones` and `boxes` (`at: "x y width height"` on a drawing of
  `size` pixels; `node` = the block "Open in the code map" goes to; `members` = the map ids that light the box live;
  `tag`, `lines`, `chips`, `where`, `stands_for`, `explain`, `shape: rail`) and the tour `steps` (title, text, the
  wires and boxes each step lights).

### Explanations: the plain words

The text in the side panel comes from `explanations/<area>.json` (one file per area: `flight`, `vision`,
`camera_plugins`, `world`, `sim_demo`, `tooling`, `lattice`, and `codemap` for this folder's own code). When a box has no entry there, the extractor uses its
docstring, then its test name, then a generated sentence. Format:

```json
{"area": "Team flight code",
 "entries": {
   "fn:ros2_ws/src/main/src/main_controller.py::MainController.image_result_cb": {
     "explain": "1-4 plain sentences: exactly what it does (required)",
     "called_when": "when it runs", "inputs": "...", "outputs": "...",
     "effects": "topics published, MAVLink sent, files written, programs started",
     "how_it_fits": "for files and packages: its role in the whole system"}}}
```

Only `explain` is required. An entry whose id is not in the map is reported by `extract.py` (a renamed function)
under "explanation ids that match no block"; that list does not fail `--check`, so read the last lines.

Conventions used in the texts (2026-10-31): blocks of last year's photo path (`ros2_ws/src/video_cam`, `mapping`,
`detection`, and the simulator helpers `photo_trigger.py`, `truth_detector.py`, `example_detector`,
`test_video_cam_compat.py`) start with "Legacy (2025) photo pipeline" (files and classes add "obsolete for the field
since 2026-10-31, kept for the legacy option"), because the field camera and detection path is now the team's new
video pipeline. Code another workflow is still building is described in a later pass. `explanations/codemap.json`
(this folder's own code) is hand-written and must be committed: `.gitignore` ignores only `/codemap.json`, the
generated map in this folder (before 2026-10-31 the unanchored line `codemap.json` hid the explanations file too).

## Make the shareable page (artifact)

```bash
python3 simulation/codemap/extract.py --check        # fresh map first
python3 simulation/codemap/build_artifact.py         # the newest recording of BOTH recording folders
python3 simulation/codemap/build_artifact.py --recording simulation/codemap/recordings/20261031-110232.jsonl
```

Without `--recording` it embeds the newest `*.jsonl` of `simulation/codemap/recordings/` and
`~/.astra_sim/codemap/recordings/`, and its last lines print the full path of the recording it used.

It writes `simulation/codemap/dist/astra_code_map.html`: `web/index.html` with the map and one recording inside
(`window.ASTRA_CODEMAP`, `window.ASTRA_TRACE`) and without the outer `<html>/<head>/<body>` tags (a Claude
artifact adds its own). It must stay at or under 15 MB: the recording is downsampled first (running-program lists
kept only when they change; calls merged into 0.4, 0.8, 1.6, 3.2 s ... windows until at most 3000 events and 3 MB;
call totals are added up, never dropped), then, only if needed, source code is reduced step by step. Options:
`--codemap`, `--recording PATH|none`, `--out`, `--max-bytes`, `--max-events`, `--max-trace-bytes`, `--no-source`.
When the page sees the embedded data it makes **no network calls at all**, replays the run (play, pause, speed,
scrubber) and says how to watch live.

## How the tracer stays off on the real drone

- **Off unless `ASTRA_TRACE=1`.** `trace/sitecustomize.py` only runs when its folder is on `PYTHONPATH`, and even
  then it does nothing else unless `ASTRA_TRACE` is exactly `1` (no imports, no thread, no profiler, no network).
- **Nothing on the drone sets it.** `launcher.sh`, the flight code's packages under `ros2_ws/src` (everything
  outside `ros2_ws/src/simulation`), `docker/` and the Jetson container scripts never set `ASTRA_TRACE` and never
  mention it or the trace folder. A search of the whole repo for `ASTRA_TRACE` and `codemap/trace` (2026-10-31,
  12:26) found them only in `simulation/codemap/**`, `simulation/scripts/demo_traced.sh`,
  `simulation/scripts/_common.sh` (the hook), `docs/simulation/CODE_MAP.md`, and one label string in the simulator's
  `ros2_ws/src/simulation/sim_demo/sim_demo/run_summary.py` (line 85, `'the live code map trace (only with
  ASTRA_TRACE=1)'`, which describes the `codemap_trace.jsonl` file and sets nothing). No team file was changed for
  the code map.
- **The only hook** is 3 lines in the simulator's `simulation/scripts/_common.sh`: with `ASTRA_TRACE=1` it sources
  `trace/astra_trace.sh`; without it, nothing.
- **When on, what it guarantees:** sending is non-blocking UDP to `127.0.0.1`, and a message that cannot go at once
  is dropped (it never waits); at most one flush per process every 100 ms; only files of this repo (or its
  `ros2_ws` install folders) are reported, never the code map's own files; it chains to the system's own
  `sitecustomize` (Ubuntu's apport hook), so Python starts as before. Python: the profiler catches exactly the two
  errors it can cause itself (`RecursionError`, `MemoryError`) and nothing else, so an exception that belongs to the
  program (say, from its own signal handler) is never swallowed; the rest of the tracer catches every `Exception`
  (a broken tracer only stops reporting). Bash: every command is guarded (works with `set -euo pipefail`), the trap
  always returns 0 and keeps `$?` and `PIPESTATUS`, and it never replaces an EXIT or DEBUG trap the script has.
- **When on, what it can still change** (both come from `sys.setprofile`, the only way Python 3.10 offers to see
  every call):
  - the programs run slower (next section);
  - the profiler is one more Python frame, so a traced program has 1 frame less of the recursion limit (1,000 by
    default). Measured with `sys.setrecursionlimit(300)` and a function calling itself until `RecursionError`: WSL
    Python 3.10 plain 298 levels, traced 297 (before this round's fix 296); Windows Python 3.11 plain 298, traced
    297. A function that compares something on every level (most real code) reached the same depth traced as plain
    in WSL (297 = 297), and 1 less on Windows (297 vs 298);
  - when an exception leaves the profiler anyway (that `RecursionError` at the very last frame, or the program's own
    exception raised by its signal handler at that instant, like `KeyboardInterrupt` from Ctrl+C), Python passes it
    to the program as usual and switches the profiler off for that thread: the program goes on exactly as untraced,
    but that thread reports no more calls;
  - bash: the counts of the last 100 ms of a script (or subshell) that has an EXIT trap of its own, or that is
    killed, are lost; `set -x` scripts also print the trap's commands.
- **Busy outside programs are skipped:** `ASTRA_TRACE_SKIP` (default `mavproxy.py,mavproxy`) names programs that
  never get the profiler even with tracing on.

Settings (environment variables): `ASTRA_TRACE=1` (on), `ASTRA_TRACE_ADDR` (default `127.0.0.1:8091`),
`ASTRA_TRACE_ROOTS` (folders whose files count; default: this repo, `~/ODLC_Machine_Inferencing_System_2025-2026`
and any path with a folder of that name), `ASTRA_TRACE_SKIP` (see above).

## How much slower does tracing make things?

Measured on the team laptop (WSL, Python 3.10, bash 5.1) on 2026-10-31, again after this round's tracer changes
(details and method at the top of `trace/astra_trace.py`; scripts `D:\suas_scratch\e2e\cm_bench3.sh` and
`cm_fix_bench.sh`):

| What | Plain | Traced |
|---|---|---|
| Python, 1,000,000 calls of a tiny function (worst case) | 0.060 s | 0.327 s when the code is not ours (skipped), 0.483 s when it is ours (counted) |
| The 83 ROS-free tests of `ros2_ws/src/main` | 1.92-1.94 s | 2.49-2.55 s (about +30 %) |
| Python start-up (`python3 -c pass`) | 14.9 ms | 16.0 ms with the folder on `PYTHONPATH` but off, 18.2 ms on |
| `mavproxy.py --help` | 185-219 ms | 195-225 ms (skipped by default), 318-343 ms if forced |
| Bash, a loop of about 20,000 plain commands | 26-28 ms | 326-336 ms (about +15 µs per command; the tracer before this round: 363-373 ms) |
| Bash, 3,000 calls of a tiny function in a loop that does nothing else (worst case) | 11-13 ms | 842-891 ms (about +0.28 ms per call; the tracer before this round: 457-494 ms) |

The Python numbers did not change with this round's profiler change (whole body inside the `try`): in the same
session the tracer before it gave 0.312-0.323 s / 0.487-0.506 s for the first row. The bash tracer now sends at most
once every 100 ms per process (the contract); while counts wait, every command checks the clock, which is why a loop
that does nothing but call functions got slower, while plain commands got cheaper (a shorter trap text).

**In a whole flight** (runway demo, real YOLO detector, traced run `20261031-110240_runway` against six untraced
runway runs of the same day): flight 307.4 s (untraced 287.1-333.3 s), photos 92 (untraced 93-95), detector hold
2.0 s (untraced 2.0 s every time), first detection of the tent 66.3 s / hiker 77.4 s (untraced 60.2-78.5 s /
70.2-92.5 s), both targets found and both payloads dropped (tent 1.37 m, hiker 4.36 m from the target; untraced
0.19-2.68 m and 1.1-5.16 m). So no difference showed at the mission level. The server itself used about 2 % of one
CPU core (one `top` snapshot during the search).

## Every file in this folder

| File | What it is |
|---|---|
| `extract.py` | the static extractor: reads the repo, writes `codemap.json` (boxes, links, explanations, `trace_index`) |
| `systems.yaml` | written by hand: the systems, which folders (and single claimed files) belong to which, the outside programs, the network links, and the Wiring view (boxes, wires, the mission tour) |
| `explanations/*.json` | plain-language explanations per area, merged by `extract.py` |
| `codemap.json` | the generated map (about 12 MB with source code); re-make it, never edit it |
| `trace/sitecustomize.py` | the opt-in switch Python runs at start-up; does nothing without `ASTRA_TRACE=1` |
| `trace/astra_trace.py` | the Python call tracer (profiler hook, counting, UDP sending, cost table) |
| `trace/astra_trace.sh` | the bash function tracer (DEBUG trap), sourced by `_common.sh` only with `ASTRA_TRACE=1` |
| `server.py` | the localhost collector and web server (UDP 8091 in, HTTP 8090 out, recordings, `/health`) |
| `web/index.html` | the explorer page with its three views, Drill down, Overview and Wiring (one file: inline CSS and JavaScript, dagre layout from cdnjs, fonts from Google Fonts) |
| `web/sample_codemap.json`, `web/sample_recording.jsonl` | small sample data for testing the page (the recording is synthetic) |
| `web/make_sample.py` | makes the two sample files from real files of the repo; the sample map carries the real Wiring view of `systems.yaml` (made by `extract.build_wiring_section`), so run it again after you change the wiring |
| `web/run_selftest.js`, `web/dom_smoke.js` | node tests of the page: logic self-test; a fake-browser run (also `--codemap PATH` and `--artifact PATH` for real data) |
| `build_artifact.py` | makes `dist/astra_code_map.html` (map + one recording, at most 15 MB) |
| `dist/astra_code_map.html` | the generated shareable page |
| `recordings/<stamp>.jsonl`, `<stamp>.health.json` | recorded live runs and the server's last `/health` answer for each (runs that `demo_traced.sh` started from the Linux-side copy are saved in `~/.astra_sim/codemap/recordings/` instead; `server.py` and `build_artifact.py` look in both) |
| `test/conftest.py`, `test/test_extract.py`, `test/test_live.py`, `test/test_build_artifact.py` | the pytest tests (no ROS needed) |
| `README.md` | this guide |

Outside this folder: `simulation/scripts/demo_traced.sh` (the traced demo) and the 3-line hook in
`simulation/scripts/_common.sh`. The short guide for teammates is [`docs/simulation/CODE_MAP.md`](../../docs/simulation/CODE_MAP.md).

## The shared contract

Ids (exact strings; paths are relative to the repo root, with `/`):

| Kind | Id | Example |
|---|---|---|
| system | `sys:<key>` | `sys:team_stack` |
| outside program | `ext:<key>` | `ext:gazebo`, `ext:ardupilot_sitl`, `ext:mavproxy`, `ext:mavros`, `ext:mission_planner`, `ext:lattice_cloud`, `ext:zr10_node`, `ext:zr10_stream`, `ext:team_detector` |
| package | `pkg:<folder>`, or `pkg:<folder>@<system key>` for the files one system claims from a folder | `pkg:ros2_ws/src/main`, `pkg:ros2_ws/src/simulation/sim_demo/sim_demo@run_logs` |
| file | `mod:<path>` | `mod:ros2_ws/src/main/src/main_controller.py` |
| function | `fn:<path>::<qualname>` | `fn:ros2_ws/src/main/src/main_controller.py::MainController.image_result_cb`; nested function `outer.inner`; bash `fn:<path>::<name>`; C# `fn:<path>::<Class>.<Method>` |
| topic | `topic:<ROS name>` | `topic:/image_detection` |

- `codemap.json` = `{"schema": 1, "generated", "commit", "nodes": [...], "edges": [...], "trace_index": {...}}`.
  A node has `id`, `kind` (system, external, package, module, class, function, method, topic), `label`, `parent`,
  and may have `path`, `lang`, `line`, `end_line`, `signature`, `doc`, `source`, `is_test`, `entry_point` and the
  explanation fields. An edge has `from`, `to`, `kind` (calls, imports, publishes, subscribes, launches, runs,
  mavlink, http, data) and maybe `label`. `trace_index` lets the server turn a tracer message into an id:
  `py_modules` (dotted module name -> file id), `py_files` (path -> file id), `functions` (file id -> first line of
  the def, decorators included -> function id).
- `codemap.json` may also have `"wiring"` (only when `systems.yaml` has a `wiring:` section; readers that do not
  know it can ignore it): `{"schema": 1, "title", "intro", "width", "height", "kinds": [{key, label, text}],
  "statuses": [...], "zones": [{id, label, note, x, y, w, h}], "boxes": [{id, title, x, y, w, h, status, members,
  node?, tag?, lines?, chips?, where?, stands_for?, explain?, shape?}], "wires": [{id, from, to (box ids), kind,
  status, arrow, title, via?, explain?, code?, path?, label_at?, offset?, map_from?, map_to?, label?}], "steps":
  [{title, text, wires, boxes}]}`. `build_artifact.py` keeps it in the artifact.
- Tracer -> server, UDP `127.0.0.1:8091`, JSON, at most every 100 ms per process:
  Python `{"pid", "proc", "calls": [[module __name__, file, first line, name, count], ...]}`;
  bash `{"pid", "sh": script path, "calls": [[function name, count], ...]}`.
  How both tracers keep the rate: at most one flush per process every 100 ms, plus one last flush when the program
  ends (Python `atexit`; bash EXIT trap, and right before `exec`). A Python flush is one datagram (split only above
  about 60 kB); a bash flush is one datagram per script file with new calls, because a bash message names one
  script. A bash subshell is a process of its own (its own pid, counts and 100 ms clock).
- Server -> page, `http://127.0.0.1:8090`: `GET /`, `/codemap.json`, `/events` (Server-Sent Events:
  `event: calls` with `{"t", "calls": {fn id: count}}` every 200 ms when not empty; `event: procs` with
  `{"t", "running": [mod or ext ids]}` every 2 s), `/recordings`, `/recordings/<file>`, `/health`.
- A recording line = one event plus its name: `{"event": "calls", "t": ..., "calls": {...}}` or
  `{"event": "procs", "t": ..., "running": [...]}`.

## Tests

```bash
python -m pytest -q -p no:cacheprovider simulation/codemap/test     # Windows or Ubuntu, no ROS
node simulation/codemap/web/run_selftest.js                         # page logic (node 18+)
node simulation/codemap/web/dom_smoke.js                            # page in a fake browser
node simulation/codemap/web/dom_smoke.js --artifact simulation/codemap/dist/astra_code_map.html
node simulation/codemap/web/dom_smoke.js --codemap simulation/codemap/codemap.json
```

What the Wiring view's tests cover: `test_extract.py` (file claims, packages inside `dir/**`, the wiring section and
every kind of mistake in it, the real repo's architecture of today, and that every port on a wire is in the files it
names), `run_selftest.js` (routes with right angles only, label placement, the model, what lights up, the sample's
real wiring), `dom_smoke.js` (every box and wire drawn and clickable, legacy / proposal / being-built styles, kind
buttons, the tour, keys, live glow and RUNNING, "open in the code map", `#wiring`; with `--codemap` / `--artifact`
also on the real map).

## Troubleshooting

| What you see | Why, and what to do |
|---|---|
| Page says **Waiting**, nothing glows | No traced program sends. Start the flight with `demo_traced.sh`, or `export` the line `demo_traced.sh --map-only` prints in the terminal that starts it. |
| `the code map server did not start ... uses port 8090 or UDP 8091` | Another server runs. Stop it, or `--map-port N --trace-port N`. |
| `has no simulation/codemap (server.py + trace/) yet` | The Linux copy is older than the code map: `bash simulation/setup/03_build.sh`. |
| Many "could not place" calls in `/health` | `received.unknown_generated_calls` are files colcon/rosidl generated (never in the map, harmless). The rest: files changed after `extract.py` ran. Re-run `extract.py --check`, then `03_build.sh`. `codemap.placed_by_name_only` above 0 means the same. After a run the same list is in `<recording>.health.json` (in `~/.astra_sim/codemap/recordings/` or `simulation/codemap/recordings/`, see "Watch it live") and in the run folder as `codemap_health.json`. |
| The Replay tab does not list a run | `server.py` lists `simulation/codemap/recordings/` and `~/.astra_sim/codemap/recordings/` only when started without `--recordings`; for another folder use `--recordings DIR`. A copy of `server.py` older than this rule (the Linux copy until its next `03_build.sh`) lists only the first folder: run `bash simulation/setup/03_build.sh`. |
| The artifact is over 15 MB | `build_artifact.py` reduces source code by itself; with `--max-bytes` lower, or `--no-source`, it gets smaller. |

## Verified on 2026-10-31

Traced visible runway demo (`D:\suas_scratch\e2e\cm_traced_run.sh`, both simulator locks held, Linux copy synced
and built first), run folder `~/.astra_sim/runs/20261031-110240_runway`, recording
`recordings/20261031-110232.jsonl`:

- `demo_traced.sh runway` exit 0; mission complete (both targets found, both payloads dropped, RTL, landed).
- `/health` answered from Windows (`Invoke-WebRequest http://127.0.0.1:8090/health` -> 200) every 5 s during the run.
- 20 s of `/events` saved from Windows with `curl.exe` during the search: 99 `calls` events, 10 `procs` events,
  317 different functions, among them `MainController.image_result_cb`, `MainController.locate_detection`,
  `SAHIObjectDetectionNode._process_image` (new_od), `SIYINode.camera_trigger_callback` (video_cam),
  `WaypointManager._repull_tick`, `MissionCameraTrigger.trigger_camera` and the sim_demo nodes.
- Whole run: 3,071 events (2,792 calls, 279 procs) over 600 s, 18.6 MB; 2,248,810 calls placed on **860 different
  functions**; every recorded id exists in the map. Per system (functions seen live / functions in the map):
  demo screens and scoring 228/563, simulated world 227/711, camera and target detection 160/189, simulator control
  133/467, team flight software 111/183, install and Docker 1/102.
- 119,380 calls (5 %) could not be placed: 97,510 in `sim_demo/flight_recorder.py`, which another person edited
  after the map was made (its lines moved; the server now places such calls by name), 19,584 in the class rosidl
  generates for `interfaces/msg/ImageResult` and 304 in colcon's own setup helpers (generated, never in the map),
  1 in a new `demo.sh` function.
- First artifact: built from the map of that day's run (7,110 blocks, 9,973 links, all source code kept) and that
  recording (3,071 events downsampled to 245: calls merged into 3.2 s windows, totals kept): 14,166,643 bytes; all
  three inline scripts passed `node --check`, and `dom_smoke.js --artifact` replayed it (12 checks, no errors, no
  network).

### Fix round after the final review (2026-10-31, about 11:50-13:00)

What changed: (1) runs that `demo_traced.sh` starts from the Linux-side copy are recorded in
`~/.astra_sim/codemap/recordings/`; `server.py` (without `--recordings`) and `build_artifact.py` (without
`--recording`) now look there too, and this README and `docs/simulation/CODE_MAP.md` say where a run is saved and how
to replay it; (2) the Python profiler's whole body is inside its `try`, which catches only `RecursionError` and
`MemoryError` (before: the first lines were outside it, the rest caught every `Exception`); the safety wording is
exact now (here, in `trace/astra_trace.py`, `trace/sitecustomize.py`, the teammate guide and the explanations);
(3) the bash tracer keeps the contract's rate (at most one flush per 100 ms per process; before: up to 20 at once),
gives every subshell its own counts (before, a subshell could send counts its parent was still holding, so they
were counted twice), sends right before `exec`, and its trap text is shorter; (4) the "who mentions ASTRA_TRACE"
search above was re-run; (5) the page in a real browser (first time) showed one layout bug, now fixed in
`web/index.html`: the side panel's link lists (for example the list of systems) were as wide as their longest
name, so at a phone width of 390 px the whole page was 499 px wide and could scroll sideways (one CSS rule: the
list's single column may now shrink, long names end in "...").

Checked (each with the command or script named; the scripts are in `D:\suas_scratch\e2e\`):

- `python simulation/codemap/extract.py --check` (Windows): exit 0, 7,623 blocks and 10,837 links at commit abf71e3 (the shared tree grew while this round ran); no explanation entry without a block.
- `python -m pytest -q -p no:cacheprovider simulation/codemap/test`: Windows Python 3.11 + Git Bash 5.2: 123 passed, 5 skipped (70 s);
  WSL Python 3.10 + bash 5.1 (`cm_fix_wsl_tests.sh`): 125 passed, 3 skipped (92 s; one earlier full run had 1 failure because another agent deleted a file in `simulation/jetson/` while the extractor read the repo, and the re-run passed).
- `node web/run_selftest.js`: 180 of 180 checks passed; `node web/dom_smoke.js`: 50 of 50; `node web/dom_smoke.js --artifact
  dist/astra_code_map.html`: 12 of 12 (7,623 blocks, 245 events).
- Recursion (`cm_fix_recursion.py`, `cm_fix_recursion2.py`): the numbers in "How the tracer stays off"; a traced
  program that stops just above its last frame keeps its profiler on (WSL and Windows).
- The profiler's errors (`cm_fix_oldcheck.py`): the tracer before this round got 2 of 4 cases wrong (a
  `RecursionError` on its first line reached the program; the program's own exception was swallowed); now 4 of 4
  right. Real signals (`cm_fix_signals.sh`, 6 rounds of 2 s, a `Tick` exception from the program's own signal
  handler every 1 ms): before, 2 and 1 of about 2,000 exceptions were lost in 2 of the 6 rounds; now 0 in all 6.
- Bash tracer on real scripts (`cm_fix_realbash.sh`: `demo.sh runway --detector truth --dry-run`, which checks and
  prints the plan but starts nothing): the same exit code and the same output traced and untraced; 13 datagrams from
  4 processes, all shaped `{"pid", "sh", "calls"}`; the smallest gap between two flushes of one process 0.131 s
  (only each process's last flush, at exit, may come sooner).
- Costs: the table in "How much slower does tracing make things?" (re-measured in this round).
- Artifact rebuilt from the fresh map and the same recording: 14,838,637 bytes (at most 15 MB). The map no longer fits with all its source code, so the builder kept function source up to 60 lines and left out whole-file source (stage "short-source"); the 3,071 recorded events became 245 (3.2 s windows) with the call total kept (2,248,810 calls on 860 functions, 0 unknown ids); all three inline scripts pass `node --check` (`cm_fix_artifact_check.py`).
- Real browser, Microsoft Edge 156.0.4314.50 (headless, a throwaway profile). Artifact page (`cm_fix_artifact_check.py`
  and `cm_fix_overflow_probe.py`; the probe loads the page inside an iframe of the exact width, because headless Edge
  will not make a window narrower than about 500 px): it boots by itself (status "Replay 1x", breadcrumb "All
  systems", the replay bar and the "Just ran" list fill in); no error from the page in the console; page width equals
  the screen width at 1280, 390 and 320 px (before the CSS fix: 499 px at 390 px); screenshots looked right (header,
  diagram, replay bar, side panel; at 320 px everything wraps and long texts end in "..."), in the dark and, at
  1280 px, also in the light colours (`--blink-settings=preferredColorScheme=1`). Live page
  (`cm_fix_live_edge.py`: `server.py` on free ports with the real map, a fake tracer message for
  `MainController.image_result_cb` every 0.5 s): status "Live connected", "39 calls seen", the function in the page,
  the diagram drawn (12 blocks), 0 calls the server could not place.

Not verified: no new traced simulator flight was flown in this round (the review asked not to repeat it unless a
fix needs it); the bash rate limit and the profiler change are checked by the tests and probes above, not in a full
flight. The Linux-side copy that the simulator runs still has the older tracer, server and `demo_traced.sh` until
its next `bash simulation/setup/03_build.sh`. The page was looked at in Edge only (not Firefox or Safari), not on a
real phone, and not inside a Claude artifact frame. On a big screen the first view ("All systems") is drawn very
wide, so "Fit" makes the 12 boxes small: zoom in with + (not changed in this round).
