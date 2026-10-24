<!--
docs/HANDOFF.md -- PURPOSE: the hand-over file for whoever continues this work next (a teammate, or an AI coding
assistant such as Codex or Claude Code). It says exactly where every part of the Gazebo digital twin stands on
2026-10-24, what was verified and committed, and the ordered task list with "done when" checks. Read AGENTS.md
first (how the repo works), then this file (what to do next). Update the status lines here as tasks finish.
-->

# Hand-over: where the simulator work stands and what to do next

Status date: **2026-10-24**. Local branch `gazebo-sim` tracks **`origin/jarrod-gazebo-sim`**.
The review commits are local; no push was requested. There is **no pull request, on purpose**: do not open one, do not
merge into `aro` or `main`, and push to `jarrod-gazebo-sim` only when the owner asks.

Read in this order: `AGENTS.md` (how everything works) -> this file -> `LOCAL_NOTES.md` if it exists
(machine-specific notes, never committed) -> the README of the package you are changing.

## 1. State right now

| Part | State | Evidence |
|---|---|---|
| Full search demo (runway world, shared team stack, real YOLO) | **Works** | 2026-10-24 run `20261024-100602_runway`: 2/2 targets; 94/94 photos processed; 94 triggers served, none dropped; tent/hiker drops 0.32/0.54 m, aim errors 0.30/0.57 m; 288.3 s flight, landed; no controller failure/traceback, all processes stopped |
| Four team-code bug fixes (commit `81a7950`) | **Done in the sim, not flown on the real drone** | unit tests green; 3+3 headless runs (see AGENTS.md section 9) |
| Simulator core (sim_models, sim_worlds runway/suas_field, siyi_emulator, sim_bringup, sim_demo, Mission Planner buttons) | **Works** | unit tests green |
| Plug-and-play slots + pluggable ArduPilot | **Unit-tested; runtime WIP** | sim_plugins 219, example_detector 10 and example_listener 3 pass; generated PLUGINS reference current; alternate profiles/SITL not flown |
| Docker (sim container + Jetson stand-in) | **WIP** | Docker Engine installed in the WSL distro, compose files parse, **images never built or run** |
| Real-terrain world `suas_field_real` | **WIP** | generator + site pack; 18 synthetic preparation regressions added (62 world tests pass); **no world/config/mission/truth yet**, missing build stages and numerical gaps documented in WORLDS |
| START OVER + Mission Planner manual mode | **Unit-tested; runtime WIP** | sim_demo 234 pass; reset progress/ownership/verified-landing and run-handover safeguards reviewed; reset/manual flows **not flown** |
| `docs/simulation/` (human docs) | **Written; clean-machine acceptance pending** | 9 guides plus front-door README, model-testing/setup commands and generated reference; relative links, script paths, CLI help and profile validation checked |
| `AGENTS.md` / `CLAUDE.md` | **Reviewed guide; quiz pending** | refreshed code/status/test counts; 30-question closed-book quiz remains Task 8 |

### Task 0/1 review and verification

Reviewed the interrupted run's uncommitted diff and grouped it into team-code parameters, corrected-aim
scoring, reset/panel behavior, plug-in contracts/docs, terrain preparation, Docker helpers and shared guides.
Fixed the Docker GPU-option caller regression, empty/sparse Sentinel-mask NaNs, reset lock/progress races,
unsafe reset cleanup/landing handling and cross-run collection. Corrected the example detector's invalid
test area assumption with boundary tests. Added ROS numeric-parameter tests and synthetic terrain coverage.

Section 3 validation: **19 packages built; 932 functional tests passed**, Ruff clean, shell syntax clean.
The full real-detector run is recorded above. Docs commands/links were checked, and nine profiles validate.
Raw run data and local automation remain outside Git. Task 2 is next; subsequent work runs **in series**
at the owner's request, with one heavy job at a time.

## 2. Task list (in this order)

Each task: what to do, then **Done when**. Rules for all tasks are in section 4.

1. **DONE: review and commit the uncommitted work** (above).
   Done when: `git status` is clean, every package's tests pass, one commit per area, each message saying
   what was verified.
2. **Team-code fixes: finish and review for the real drone.**
   Status: **next**. Read-only review found a mission-pull future without a deadline, inaccessible-photo-folder
   processing timeout that can loop forever, and missing nonfinite GPS/pixel rejection. Terrain is a single
   ground plane; roll/mounting/FOV/image orientation, metadata ages and serial upload acknowledgments remain
   hardware decisions. The local ROS parameter read's 5 s deadline is not a serial MAVLink transaction.
   (a) Robustness tests: make the MAVROS parameter read fail/time out once and show main_controller survives
   with the old indices; upload a mission WITHOUT the simulator's re-read helper (`simulation/scripts/_common.sh`)
   and show main_controller still uses the right indices; decide whether the helper stays.
   (b) Independent review of the diff `git diff cfc8d7c -- ros2_ws/src/{main,video_cam,waypoint_mavros,wp_sender,mapping}`
   for: the maths in `main/src/target_geolocation.py` (frames, degrees/radians, gimbal pitch sign, altitude
   reference, sloped ground: the real search areas have ~9-10 m of relief), timeouts tuned for the sim vs a
   115200-baud serial MAVROS link, real SIYI A8 mini firmware answers vs the emulator, thread safety, blocking
   calls in callbacks. Fix what is real, each with a unit test.
   (c) Write a short report of before/after numbers (two headless runway demos).
   Done when: tests for each fix, two clean headless demos (2/2 targets, drops about 1 m, no dropped photos,
   main_controller alive to the end), review findings fixed or written down as open decisions.
3. **START OVER and the Mission Planner manual mode: fly them.**
   In the simulator: full run -> `start_over.sh --fly` -> a second FULL run (both payloads re-attached and
   dropped again, new run folder and score report); start over while airborne (lands safely first); the
   manual flow (`demo.sh --manual`, upload/arm/AUTO from a separate MAVLink client or Mission Planner) with no
   re-read helper; the panel START OVER button; read every `simulation/windows/*.bat` for quoting.
   Done when: all three flows pass with run-folder evidence; never kills processes it did not start.
4. **Plug-and-play: docs + validate in the simulator.**
   Status: docs and unit checks done; all profile/SITL runtime acceptance checks remain.
   Write `docs/simulation/PLUGINS.md` (makes the last failing test pass; it embeds blocks generated by
   `sim_plugins/doc_blocks.py`) and `docs/simulation/ARDUPILOT.md`. Then fly: default profile (= today's demo),
   `truth_detector`, `example_detector` (does main_controller accept its detections? the panel show them?),
   `check_slot` passing the right node and failing a wrong one, another SITL version via
   `simulation/setup/fetch_sitl.sh` (e.g. 4.7.x), an extra parameter file (e.g. a changed WPNAV_SPEED),
   external autopilot mode if feasible.
   Done when: all sim_plugins tests pass and each profile has a recorded run.
5. **Real-terrain world `suas_field_real`.**
   Finish the generator and apply the terrain research fixes: (1) drop lidar "haze" (keep points below
   DTM + 35 m or ignore tops above ~31 m: no fake 45 m trees); (2) photo-only trees from DeepForest crowns or a
   6-8 m window (target ~130 trees in search box 1, ~75 in search box 2, ~3,100 in the flight boundary); (3) the
   3 wooden poles in search box 1 (10.2 m tall, 0.3 m wide; pad-frame (250.6,-246.4), (268.6,-245.4),
   (357.6,-246.4)); (4) subtract the Sentinel-2 L2A +1000 offset before NDVI; (5) building heights from the
   lidar; Microsoft footprints licence = CDLA Permissive 2.0; (6) survey altitudes: SB1 57.7-124.3 m and SB2
   54.8-122.0 m above the pad (150 ft over the highest ground, 400 ft over the lowest); (7) under ~2 M
   triangles, mesh (not heightmap) ground collision; (8) no OK Aerial 2025 pixels in the repo (no redistribution
   licence; the repo is public). Generate world .sdf, config, truth, mission; load in Gazebo (real-time factor
   >= 0.8), drop-test boxes on the ground, fly `demo.sh suas_field_real --headless`, measure drop errors on
   sloped ground; write `docs/simulation/WORLDS.md`.
   Done when: the demo flies the real field end to end with a score report and the numbers above check out.
6. **Docker, including "run it on another PC".**
   Build the images (`simulation/docker/build_images.sh`), run the headless demo inside the sim container,
   run the two-container compose (sim at 192.168.144.25, Jetson stand-in at 192.168.144.100 running
   `./launcher.sh sim` unchanged), a short GUI check through WSLg. Write `docs/simulation/DOCKER.md` starting
   with a copy-paste **"Run it on another PC"** guide: Windows + WSL2 (Docker Engine in WSL, or Docker Desktop),
   native Linux with and without an NVIDIA GPU (CPU fallback), `git clone -b jarrod-gazebo-sim ...`, build OR
   copy pre-built images with new `simulation/docker/export_images.sh` / `import_images.sh` (docker save/load +
   checksum), run, "it works if you see...", troubleshooting; optional ghcr.io publishing as instructions only.
   Portability test: from a clean copy (`git ls-files -co --exclude-standard | rsync --files-from=- ...` into a
   temp folder) follow only the guide; one run without the GPU; export -> rmi -> import round trip.
   Done when: the containerised demo runs from a clean copy following only the guide.
7. **Human docs.** Create `docs/simulation/`: ARCHITECTURE.md, PLUGINS.md, ARDUPILOT.md, DOCKER.md, WORLDS.md,
   MISSION_PLANNER.md, GOOGLE_EARTH.md (Google's terms: no Google imagery/3D as textures or offline data; a live
   basemap or Earth Pro comparison only), TROUBLESHOOTING.md, WHAT_CHANGED.md. Make `simulation/README.md` the
   front door with a 10-line "Run it" box. Every code file starts with a purpose paragraph.
   Done when: a new teammate can follow the docs from scratch (test it) and every command matches the scripts.
   Status: all nine guides and the 10-line front-door run box written; links, flags, help and generated blocks
   checked. Clean-machine walkthrough remains pending; do not count unfinished Docker/terrain flows as passed.
8. **Keep AGENTS.md true.** After each task, update the matching AGENTS.md section and section 9 (status).
   Test it closed-book: give a fresh assistant only AGENTS.md and the 30-question quiz; fix every miss.
   Done when: the quiz scores 90% or more.

Optional later: an "Open in Google Maps" button in the demo panel (no key needed); an optional Google map tab
(needs the owner's own API key and billing); explainer web pages (outside the repo, see LOCAL_NOTES.md).

## 3. How to check your work

```bash
# inside Ubuntu 22.04 (WSL distro Ubuntu-22.04 on the owner's laptop)
bash "<repo>/simulation/setup/03_build.sh" --workers 4        # mirror to ~/ODLC_Machine_Inferencing_System_2025-2026 and build
cd ~/ODLC_Machine_Inferencing_System_2025-2026 && source simulation/setup/sim_env.sh
cd ros2_ws/src/<package> && python3 -m pytest -q test --ignore=test/test_flake8.py --ignore=test/test_pep257.py --ignore=test/test_copyright.py
bash simulation/scripts/demo.sh runway --headless           # ~6 min; score report in ~/.astra_sim/runs/<date>_runway/score_report.md
bash simulation/scripts/stop_sim.sh                         # always stop what you start
```

Verified: sim_bringup 55, sim_demo 234, sim_models 63, sim_worlds 62, siyi_emulator 147, main 51,
video_cam 24, waypoint_mavros 15, wp_sender 17, mapping 20, detection 6, sim_plugins 219,
example_detector 10, example_listener 3, Docker helpers 6: **932 passing**.
For sim_models exclude `test/acceptance`, siyi_emulator `test/e2e`, sim_demo `test/ros_e2e`; these are
separate integration jobs. Use `QT_QPA_PLATFORM=offscreen` for panel tests. Docker helper tests run with
`python3 -m pytest -q simulation/docker/test` from the repo root. Also run `ruff check ros2_ws/src`,
`bash -n` on changed shell scripts, and `python3 -m sim_plugins.profile_tool docs --check`.

## 4. Rules (do not break these)

- The team's flight code must keep running **unchanged on the real drone** and in the simulator; default
  behaviour stays the same unless a task says otherwise; never weaken real-hardware behaviour to please the sim.
- Every file starts with a short purpose paragraph; comments explain why; beginner-friendly docs (the team are
  students); tests with every change.
- The repo is **public**: no raw data, build outputs, secrets, personal e-mail addresses or machine-specific
  paths in committed files (maintainer e-mail `astra-sim@example.com`; derive paths at run time).
- Only **one heavy job** (Gazebo, SITL, demo, docker build, big generator run) at a time on a laptop; stop
  everything you start.
- Build in the Linux-side copy (`03_build.sh`), never with colcon inside the Windows `D:` folder.
- No pull request, no merge into `aro`/`main`; push to `jarrod-gazebo-sim` only when the owner asks.
