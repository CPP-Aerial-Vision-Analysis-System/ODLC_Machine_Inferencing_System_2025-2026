<!--
docs/HANDOFF.md -- PURPOSE: the hand-over file for whoever continues this work next (a teammate, or an AI coding
assistant such as Codex or Claude Code). It says exactly where every part of the Gazebo digital twin stands on
2026-10-24 (section 0 adds the work committed up to 2026-10-31 and what is in progress), what was verified and
committed, and the ordered task list with "done when" checks. Read AGENTS.md first (how the repo works), then
this file (what to do next). Update the status lines here as tasks finish.
-->

# Hand-over: where the simulator work stands and what to do next

Status date: **2026-10-24**, with the later additions (to 2026-10-31) in section 0. Local branch `gazebo-sim`
tracks **`origin/jarrod-gazebo-sim`**.
The review commits are local; no push was requested. There is **no pull request, on purpose**: do not open one, do not
merge into `aro` or `main`, and push to `jarrod-gazebo-sim` only when the owner asks.

Read in this order: `AGENTS.md` (how everything works) -> this file -> `LOCAL_NOTES.md` if it exists
(machine-specific notes, never committed) -> the README of the package you are changing. Every doc of the
repo is listed in the documentation index [`docs/README.md`](README.md).

## 0. Since the 2026-10-24 status (committed work up to 2026-10-31)

### 2026-11-07: SUAS 2027 terrain and perception preparation (current request)

The owner's latest request prioritizes realistic environments and tests using newly available Blender
and the supplied *SUAS 2027 Primer & Task Ideas*. Read [SUAS_2027.md](simulation/SUAS_2027.md) for source-page
traceability, unconfirmed competition details, generation commands and test cases. Existing uncommitted
Jetson, documentation, code-map and plug-in work was preserved.

Added separate `sim_worlds/challenge2027/` recipe/packaging/evaluator modules and the Blender generator:
10-acre synthetic coastal search area, rolling collision terrain, articulated colored mannequins and
suitcases, bystanders/clutter, harbor props and six competing landing candidates. Five seeded cases:
baseline, occluded, low contrast, low sun and no target. Generated assets stay outside Git. Public task
geometry is separated from private target/validity truth. 2026 flight defaults and truth validation remain
unchanged. LZ dimensions/slope and all candidate validity labels are local training assumptions.

Verified 2026-11-07: prescribed WSL `03_build.sh --packages sim_worlds --workers 4 --no-models` passed;
`sim_worlds` **103 tests passed**, including 41 new recipe/evaluator/packaging regressions; full
`ruff check ros2_ws/src` clean; docs checker passes (72 Markdown files). The existing SciPy/numpy
compatibility warning remains. Blender 5.2.3 smoke checks pass for all four human poses, clothing materials
and terrain vertex normals. Fixed two discovered export issues: deferred transforms inflated a boot;
missing OBJ normals crashed Harmonic/DART before physics. Both now have Blender regressions.

Baseline `suas2027_baseline_2027` generated: **207,344 triangles**, 10.0028-acre search, survey 57 m above
home with at least 50.5266 m terrain clearance. Inspected Blender overview/detail/50 m previews and actual
Gazebo overview/detail/50 m camera images. Visible isolated Gazebo check: 10.065 simulated seconds,
21.65 s wall time including load, median real-time factor **0.9994**, no server errors; origin/clearing/
sloping-ground drop probes agree with the collision surface to less than **0.01 mm** at their final poses
(the slope sphere rolls downhill). Evidence: external bundle `checks/gazebo_check.json`, logs and images;
these numerical results are scene-only, without SITL or a running detector. Generated files and local
automation are outside Git. This slice is **uncommitted**; existing unrelated work remains uncommitted too.

Preparation slice **verified**: build and world tests pass; baseline Blender export and aerial/detail
previews inspected; meshes/materials load in visible Gazebo; terrain drop probes agree with the exported
surface; docs checker passes. These do not establish model accuracy or autonomous mission completion.
Next: actual camera/model benchmark on held-out seeds; mapping/official report format; terrain-aware LZ
proposals and 2027 mission orchestration; delivery retardant/AGL/telemetry tests. Keep the 2026 baseline.

#### First visible 2027-world flight, with Mission Planner (2026-11-07)

Run `20261107-092830_suas2027_baseline_2027`: full WSL build **20 packages**, 103 world tests pass;
unchanged team stack, real YOLO26m on CUDA, ArduCopter 4.6.3. Took off, surveyed five lanes, processed the
remaining images in a 22 s hold, released both legacy payloads, returned and landed; **10:50** from arming.
No controller traceback/restart. All simulator processes and the passive observer stopped; an initially
reported leftover process had exited when checked again. Mission Planner remains open for the operator.
The private shell runner logged exit 127 **after** the flight (`ue: command not found`): changing its panel
flag while Bash was reading that active file shifted the subsequent read position. Raw stderr is retained;
the mission's own run summary records landed/exit 0. This is a harness bookkeeping fault, not evidence of
an autopilot crash; do not describe the wrapper itself as a clean pass or edit an active runner again.

The owner reconfirmed that **Mission Planner + ASTRA plugin replaces the Python/Qt demo panel**. The test
initially passed `--panel` by mistake; closed only that owned panel cleanly during flight and opened the
documented ground station. Verified ASTRA tab selected, live telemetry/camera, target coordinates, both
release cards and Return phase. Future test command uses `--no-panel`; direct scene launches need
`panel:=false`. This was an automatic MAVLink-driven test observed in Mission Planner, not a manual
Mission Planner upload/arm acceptance test.

| Check | Measured result / limitation |
|---|---|
| Hiker recognition/aim | Legacy score finds 1/1 hiker, confidence **0.907**, chosen aim **2.22 m** from truth. The per-photo score uses coarse class/footprint matching, not instance-level validation. |
| Hard negatives / mission behavior | **157 false tent boxes**, no tent target in this world; a 0.85-confidence false tent caused an extra drop about 190.7 m from the hiker. The 2026 brain is not a one-payload 2027 mission. |
| Image accounting | **315** successful camera captures / emulator events, zero queue-full drops, but **258 distinct files**, all 258 processed. Investigate repeated GPS filenames/timing; do not describe 258/258 as proof that all 315 captures survived separately. |
| Actual delivery | Hiker bottle released/detached, but never confirmed resting. A later physical sample showed ground contact, **131.98 m from the hiker and moving 2.00 m/s**. It rolls on the sloped ground. **The report's 2.2 m drop value is release-position fallback, NOT final delivery accuracy.** |
| Actual terrain clearance at release | Passive world-odometry observer: hiker **45.846 m AGL**, false tent **45.559 m AGL**, compared with the primer's 45.72 m floor. These use the drone model origin, not a real-aircraft altitude certification. |
| Timing / warnings | Observed sim/wall-time ratio approximately **0.744** over the observer interval (full stack, not the scene-only 0.9994 check). Twelve MAVROS time-synchronizer reset errors were logged; controller stayed alive. Mission Planner's plugin reports a status-subscription retry warning, while fallback updates populated the cards. |

Evidence stays in the private run folder: `score_report.md`, `run_summary.md`, `run_data.json`, CSV/logs,
`terrain_observer.jsonl`, `person_payload_pose_samples.json`, `suas2027_flight_diagnostics.json`; native
Mission Planner screenshot and copied summary artifacts are kept outside Git. **Flight completed; 2027
mission/delivery acceptance did not pass.** Next decisions: one-target mission behavior and hard-negative
model validation; a realistic retarded payload with terrain contact/rolling; distinguish release versus
resting-delivery results in scoring; image filename accounting and sufficient AGL margin. No fixes to the
team flight code were made in this test.

### Work recorded through 2026-10-31

| Commit | What changed | Evidence recorded |
|---|---|---|
| `94ccf0b` | Task 2 (below) is **committed**: the three real-drone safety fixes in the team code | the tests and runs listed under "Task 2" |
| `2c9a75e` ... `df3b660` | Anduril Lattice bridge (package `lattice_bridge`, pretend local Lattice, `lattice_mock` profile) | [`docs/lattice/README.md`](lattice/README.md), [`docs/lattice/BRIDGE.md`](lattice/BRIDGE.md) |
| `4eb462a` | `pov_server`: the drone camera as an MJPEG stream at `http://127.0.0.1:8081/pov.mjpg`, started by `sim.launch.py` | unit tests (no live run in that commit) |
| `83c2ae4` | `demo.sh`: the old Qt demo panel is **off by default** (`--panel` brings it back); Mission Planner is the ground station | |
| `593bb96` | Mission Planner **ASTRA plugin** (`simulation/windows/mission_planner_plugin/astra_gcs.cs`): ASTRA tab, camera in the HUD, map markers | camera shown in the HUD for a whole demo flight |
| `6d90c07` | MAVROS talks to SITL **SERIAL2 (TCP 5763)** by default, wired like the real Jetson on the Pixhawk's TELEM2, so ArduPilot forwards the team's status texts to every ground station | run `20261031-092448_runway`: phases, detections, targets and both drops shown in Mission Planner; drops 1.0 m and 1.1 m |
| `0ddda83` | ASTRA plugin 1.1: keeps the ASTRA tab first, turns its sentence amber after 10 s without telemetry, retries the status-text subscription | compiles against Mission Planner 1.3.83 (`check_mp_plugin.ps1`); not yet watched in a flight in that commit |
| `f139036` | **Lattice handed to the team**: the owner stopped the AI-assisted Lattice work; the team continues from [`docs/lattice/HANDOFF.md`](lattice/HANDOFF.md) | the proof results recorded in that hand-over note |

Work in progress on 2026-10-31 (not committed; the lead reviews and commits each part separately): the
documentation index [`docs/README.md`](README.md) with its checker `simulation/scripts/check_docs.py` (and the
test `sim_plugins/test/test_docs_index.py`); the **Jetson-in-the-loop** kit (the team code on the real Jetson
Orin Nano, the simulator on the laptop; the laptop reaches the Jetson over the USB-C cable or over plain SSH on
Ethernet or Wi-Fi); swapping one twin piece for the real one; run logs and CSV files; one list of commands and
errors; the Docker doctor; the Gazebo camera window; the code map. Their guides are
listed under "Being written" in the index until they exist. The tables in sections 1-3 below are the
2026-10-24 record and were not re-run.

## 1. State right now

| Part | State | Evidence |
|---|---|---|
| Full search demo (runway world, shared team stack, real YOLO) | **Works** | 2026-10-24 run `20261024-100602_runway`: 2/2 targets; 94/94 photos processed; 94 triggers served, none dropped; tent/hiker drops 0.32/0.54 m, aim errors 0.30/0.57 m; 288.3 s flight, landed; no controller failure/traceback, all processes stopped |
| Four team-code bug fixes (commit `81a7950`) + Task 2 real-drone hardening (commit `94ccf0b`) | **Done in the sim, not flown on the real drone** | 3 review findings fixed with before/after tests; robustness and no-helper upload shown in the simulator; 2 demos 2/2 (drops 1.1/0.5 and 0.6/1.0 m); 6 open hardware/design decisions (Task 2 section below) |
| Simulator core (sim_models, sim_worlds runway/suas_field_real, siyi_emulator, sim_bringup, sim_demo, Mission Planner buttons) | **Works** | unit tests green |
| Plug-and-play slots + pluggable ArduPilot | **Unit-tested; runtime WIP** | sim_plugins 219, example_detector 10 and example_listener 3 pass; generated PLUGINS reference current; alternate profiles/SITL not flown |
| Docker (sim container + Jetson stand-in) | **WIP** | Docker Engine installed in the WSL distro, compose files parse, **images never built or run** |
| Real-terrain world `suas_field_real` | **WIP** | generator + site pack; 18 synthetic preparation regressions added (62 world tests pass); **no world/config/mission/truth yet**, missing build stages and numerical gaps documented in WORLDS |
| START OVER + Mission Planner manual mode | **Unit-tested; runtime WIP** | sim_demo 234 pass; reset progress/ownership/verified-landing and run-handover safeguards reviewed; reset/manual flows **not flown** |
| `docs/simulation/` (human docs) | **Written; clean-machine acceptance pending** | 9 guides plus front-door README, model-testing/setup commands and generated reference; relative links, script paths, CLI help and profile validation checked. Since 2026-10-31 every doc is listed in `docs/README.md` and `simulation/scripts/check_docs.py` checks the index, links and script names |
| `AGENTS.md` / `CLAUDE.md` | **Reviewed guide; quiz pending** | refreshed code/status/test counts; 30-question closed-book quiz remains Task 8 |

### Task 0/1 review and verification

Reviewed the interrupted run's uncommitted diff and grouped it into team-code parameters, corrected-aim
scoring, reset/panel behavior, plug-in contracts/docs, terrain preparation, Docker helpers and shared guides.
Fixed the Docker GPU-option caller regression, empty/sparse Sentinel-mask NaNs, reset lock/progress races,
unsafe reset cleanup/landing handling and cross-run collection. Corrected the example detector's invalid
test area assumption with boundary tests. Added ROS numeric-parameter tests and synthetic terrain coverage.

Section 3 validation: **19 packages built; 932 functional tests passed**, Ruff clean, shell syntax clean.
The full real-detector run is recorded above. Docs commands/links were checked, and nine profiles validate.
Raw run data and local automation remain outside Git. Task 2 is done (below, committed in `94ccf0b`); subsequent work
runs **in series** at the owner's request, with one heavy job at a time, and simulator runs are VISIBLE.

### Task 2: team code made safe for the real drone (2026-10-24; committed in `94ccf0b`)

**Three fixes**, each with tests that fail on the committed code (the new tests run against a copy of `HEAD`:
52 failures, and 2 test modules cannot even import) and pass after the fix:

| # | Problem found by the review | Fix | Tests (file::name) |
|---|---|---|---|
| 1 | A `/mavros/mission/pull` that MAVROS never answers kept waypoint_manager "in flight" for ever; every later re-read, even the one at arming, queued behind it (the team code could fly an OLD mission) | `waypoint_mavros/mission_sync.py`: `pull_timeout_s` 45 s (longer than MAVROS 2's own 30 s `LIST_TIMEOUT`, so a slow serial link is never cut off), `ACTION_TIMEOUT`. `waypoint.py` `_abandon_pull`: forgets the request, keeps the last mission copy and its parameters, logs `MAVROS did not answer the mission re-read within 45 s ... Keeping the last mission copy (12 items, rtl_index=11)`, retries (max 3); a later upload/arming always re-reads | `test_mission_sync.py::test_a_pull_without_answer_times_out_and_is_retried` (+3 more), `test_waypoint_node.py::test_a_pull_that_never_answers_times_out_and_keeps_the_last_mission` (+2; the real node with a pull client that never answers) |
| 2 | Missing/unreadable photo folder: the hold check raised on every 2 s tick before it reached the 90 s limit, so the drone hovered in GUIDED until a failsafe; the hold used the wall clock | new `main/src/processing_hold.py` (`list_photo_names` never raises, `decide` always ends at the limit); `main_controller.py`: `time.monotonic()`, folder problem logged once, `Processing timeout: images unreadable. Diverting.`, `_end_processing_hold` sends RTL if the divert itself fails | `test_processing_hold.py` (7), `test_controller_safety.py::test_missing_photo_folder_still_ends_the_hold_at_the_timeout`, `::test_unreadable_photo_folder_still_ends_the_hold_at_the_timeout`, `::test_folder_problem_is_logged_once_not_every_tick`, `::test_wall_clock_jump_back_does_not_stretch_the_hold`, `::test_a_failing_divert_goes_home_instead_of_hovering` |
| 3 | NaN/inf pass every `<`/`>` check: a photo named `nan , nan.jpg`, a NaN box centre, a NaN sidecar value or `-p target_ground_height_m:=.nan` became a NaN aim point and a NaN waypoint | `target_geolocation.py`: `is_finite_number`, `valid_latlon`, every input checked, `aim_at_box` returns None for a bad position or box, final check of every answer. `main_controller.py`: such a detection is ignored with the reason, non-finite scores ignored, `_number_parameter` refuses `.nan/.inf`, `_divert_to_targets` checks again. `waypoint.py`: `/addWaypoint` refuses NaN/out-of-range/(0,0) items. `video_cam/siyi_node.py`: such a fix gives no GPS photo name | `test_target_geolocation.py::test_projection_refuses_nonfinite_inputs`, `::test_aim_refuses_nonfinite_position_or_box`, `::test_aim_never_returns_nonfinite_numbers` (+3), `test_controller_safety.py::test_nonfinite_gps_never_becomes_a_target`, `::test_nonfinite_box_centre_never_becomes_a_target`, `::test_nonfinite_score_never_becomes_a_target`, `::test_divert_refuses_a_target_without_a_valid_position`, `test_number_parameters.py` (`.nan`, `.inf`), `test_waypoint_node.py::test_a_nonfinite_target_waypoint_is_never_uploaded`, `video_cam/test/test_gps_filename.py` |

Rule kept for (3): a non-finite sidecar height/heading/FOV/zoom/gimbal value counts as "not recorded" (the
existing fallback to the photo's own, valid position); a non-finite GPS position, box centre or score drops
the detection. Good data gives exactly the old behaviour. Tests: main 113, waypoint_mavros 27, video_cam 32,
wp_sender 17, mapping 20 pass on the built copy and again on the final source (+ detection 6); `ruff check
ros2_ws/src` clean. The simulator packages are unchanged and were not re-run (their Task 0/1 counts stand).
Runtime probe: `main_controller -p target_ground_height_m:=.nan` (and `max_aim_offset_m:=.inf`) starts and logs
"is not a number: using the default".

**(a) Robustness.** main_controller reads NO MAVROS/FCU parameter: its only parameter read is
waypoint_manager's index parameters over local ROS (5 s deadline), which is what was failed on purpose.
* `main/test/test_controller_robustness.py` (new; ~20 s): the real MainController node next to fake MAVROS
  services and a fake waypoint_manager in a private ROS domain; one read answered after 6.5 s (> 5 s) and one
  incomplete answer: node alive, old indices kept while failing, warning logged, retried by itself, new
  indices taken ("again after 1 failed attempt"). It passes before and after (the October fix already holds;
  this is the requested proof).
* Simulator, visible run (`20261024-110536_runway_stale_robust`, script kept outside the repo):
  a short decoy mission (8 items, RTL 7) was on the autopilot when the team stack started; the real runway
  mission (12 items) was then uploaded with `mission_tool upload` ONLY (no `/mavros/mission/pull` by anyone but
  the team code). waypoint_manager saw `Flight plan received`, re-read it 3 s later (indices 8/7/6/5 ->
  12/11/10/9) and main_controller and the photo trigger followed; the photos stopped at WP 9 and the hold was
  at WP 10 (the decoy would have given 5 / 6). Twice waypoint_manager was frozen (`kill -STOP`) for 13 s while
  main_controller was made to re-read its indices (on the ground and in flight during the survey): each time
  `Could not read ... no answer within 5 s ... Keeping the previous values`, then `again after 1 failed
  attempt(s)`; no process died, main_controller held, diverted, dropped (tent/hiker 1.05/0.35 m, aim 1.04/0.38
  m), RTL, landed; 101/101 photos, none dropped, 299 s.

**Re-read helper (`astra_mavros_repull` in `simulation/scripts/_common.sh`): keep it**, as a harmless speed-up
with a clear operator message in `upload_mission.sh` / `fly_mission.sh`; it is no longer needed for
correctness (the stale-mission run above and demo.sh never call it, so they keep testing the team code's own
re-read).

**(b) Review of `target_geolocation.py` and the timeouts.**
* Checked and correct: east/north/up offsets; compass headings; every trig call in radians; photo right =
  down x forward (= east when facing north and looking down); SIYI pitch sign (-90 = down); SIYI yaw sign
  (+ = left, as the emulator: verify on the bench, D2); ArduPilot's own lat/lon offset formula;
  `image_top_direction_deg` (code and tests match video_cam's meaning; only a docstring was ambiguous, now
  clarified). Height = `rel_alt` (above HOME = where the drone was armed) minus `target_ground_height_m`
  (also above home): consistent, NOT AMSL (`alt_msl_m` is recorded in the sidecar but unused).
* Sloped ground (`main/test/test_target_geolocation.py` pins these formulas). The projection uses ONE flat
  ground at `target_ground_height_m`. A ground height wrong by `dh` moves the aim by `dh x tan(ray angle)`,
  at ANY flight height (81 deg photo, 16:9, zoom 1x; the 5x team zoom divides every number by 5):

  | dh | centre | half-way to the side | side edge | top edge | corner |
  |---|---|---|---|---|---|
  | 2.5 m | 0.0 m | 1.1 m | 2.1 m | 1.2 m | 2.4 m |
  | 4.8 m | 0.0 m | 2.0 m | 4.1 m | 2.3 m | 4.7 m |
  | 9.6 m | 0.0 m | 4.1 m | 8.2 m | 4.6 m | 9.4 m |
  | 12.0 m | 0.0 m | 5.1 m | 10.2 m | 5.8 m | 11.8 m |

  Real field (from the Task 5 survey altitudes: 150 ft over the highest ground, 400 ft over the lowest):
  SB1 ground ~2.4-12.0 m and SB2 ~0.1-9.1 m above the pad. With the default 0, a corner detection is off by
  up to 11.7 m (SB1) / 8.9 m (SB2); with the parameter set to the area's mean height by up to 4.7 / 4.4 m
  (2.0 / 1.9 m half-way to the side).
* Gimbal not exactly straight down while -90 is assumed (no attitude report): centre error = height x
  tan(error): 1 deg -> 0.5 m (30 m sim), 0.8 m (150 ft), 1.3 m (250 ft), 2.1 m (400 ft); 2 deg -> 1.0 / 1.6 /
  2.7 / 4.3 m; 5 deg -> 2.6 / 4.0 / 6.7 / 10.7 m. When the gimbal REPORTS -85 the code uses it: then only
  `dh x tan(5 deg)` remains (0.8 m for dh 9.6 m). Roll is not modelled (same size, across track). Sim photos
  reported -89.6..-90.
* Metadata age: sim median 0.14 s, max 0.24 s (94 photos of run `20261024-100602_runway`); real drone:
  `gpstest` sets every stream to 1 Hz, so the fix can be ~1 s old = up to 5.5 m along track at 5.5 m/s (not
  compensated; D4).
* Timeouts vs a 115200-baud serial link: AsyncParameterReader 5 s = local ROS, fine; mission-pull deadline
  45 s > MAVROS's 30 s; repull delay 2 s, own-push quiet 15 s fine; the target upload is not awaited before
  `set_current` (fixed 4 s; D5); the 90 s hold assumes the Jetson keeps up (D6).
* Blocking calls / threads: main_controller and waypoint_manager run single-threaded executors and wait only
  on timers. Left unchanged and noted: `waypoint.py push_waypoints` waits without limit for
  `/mavros/mission/push` inside the `/addWaypoint` callback (only if MAVROS is gone); `pull_waypoints`,
  `delete_waypoint`, `change_mode`, `clear_waypoints` are blocking or broken but unused (`/delWaypoint` has no
  caller). video_cam's capture threads were not re-audited (October fix). new_od logs one error per 3 s
  scan if the folder becomes unreadable and video_cam reports a failed save per photo: bounded, unchanged.

**(c) Demos** (before = run `20261024-100602_runway`, the October fixes only: 2/2, 94/94 photos, none dropped,
drops tent/hiker 0.32/0.54 m, aim 0.30/0.57 m, 288 s):

| Run | How | Targets | Photos taken / checked | Triggers served / dropped | Drop tent / hiker | Aim tent / hiker | Flight | main_controller |
|---|---|---|---|---|---|---|---|---|
| `20261024-104711_runway` | headless (started before the owner's "visible" instruction) | 2/2 | 95 / 95 | 96 / 0 | 1.1 / 0.5 m | 1.1 / 0.5 m | 4:51 | alive (RTL sent, no death/restart) |
| `20261024-105800_runway` | visible (Gazebo + RViz + panel, closed itself) | 2/2 | 101 / 101 | 101 / 0 | 0.6 / 1.0 m | 0.6 / 1.0 m | 4:58 | alive (RTL sent, no death/restart) |

A third run (`20261024-105438_runway`, headless) was stopped during autopilot settling when the owner asked
for visible runs; it has no results. Only the first demo ran headless.

**Open decisions (hardware tests or design choices; nothing was guessed in the code):**

| # | Question | Numbers | Options |
|---|---|---|---|
| D1 | Ground height under the targets on sloped terrain (`target_ground_height_m`, default 0 = the launch point's height) | see the table above: up to ~12 m at a photo corner with 0, ~4-5 m with the area mean | (a) set the parameter to each area's mean height above the launch point (no code; needs the launch point's elevation on the day); (b) onboard terrain grid (public USGS 3DEP DEM, downloaded to the Jetson, never committed) + ray/terrain intersection (code + data; best); (c) prefer the detection nearest the photo centre for each target (selection change); (d) survey at the 5x zoom (5x smaller, many more lanes); (e) a confirmation pass over the aim point before dropping |
| D2 | Does the A8 mini report its gimbal attitude (0x0D) at the shutter, in which frame/sign (yaw + = left?), and is roll needed? | 0.8 m per degree at 150 ft, 2.1 m per degree at 400 ft if -90 is assumed wrongly | (a) bench test: gimbal at -90, read 0x0D, photograph a grid from a known height; (b) add roll to the projection (small change); (c) a fixed mounting-offset parameter from a calibration flight |
| D3 | Photo orientation (`image_top_direction_deg`, video_cam) | a wrong value turns every offset about the photo centre (180 = twice the offset) | marker test from `video_cam/HOW_OUR_CAMERA_WORKS.md` before trusting the drops |
| D4 | Age of the GPS/heading used for each photo | up to ~1 s at 1 Hz = 5.5 m along track at 5.5 m/s; A8 mini shutter delay unknown | (a) raise only the position stream on the Jetson's serial port (5-10 Hz: ~3-4 % of 115200 baud); (b) record velocity and extrapolate to the shutter; (c) fly slower; measure the shutter delay |
| D5 | The two `/addWaypoint` full-mission pushes are not awaited; `set_current` follows after a fixed 4 s | a ~14-item push should take well under 1 s on a clean 115200 link (estimate, not measured), but MAVROS retries a lost item after 1 s (3 tries); a refused `set_current` goes unnoticed | (a) wait for the push results before `set_current`, timeout -> RTL; (b) a longer fixed wait; (c) keep and test on the real link |
| D6 | Detector speed on the Jetson vs the 90 s hold | photos still unchecked after 90 s are ignored | measure new_od on the Orin Nano with the flight camera's photo size before changing `PROCESSING_TIMEOUT_SECONDS` |

## 2. Task list (in this order)

Each task: what to do, then **Done when**. Rules for all tasks are in section 4.

1. **DONE: review and commit the uncommitted work** (above).
   Done when: `git status` is clean, every package's tests pass, one commit per area, each message saying
   what was verified.
2. **Team-code fixes: finish and review for the real drone.**
   Status: **DONE in the simulator (committed in `94ccf0b`)**: the three review findings (mission-pull
   deadline, never-ending processing hold, NaN/inf rejection) are fixed with before/after tests, (a)-(c) are
   recorded in "Task 2" above, and what needs the real hardware or a design choice is listed there as open
   decisions D1-D6 (sloped ground, gimbal attitude/roll, photo orientation, metadata age, upload
   acknowledgement, detector speed). Nothing of it has flown on the real drone.
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
   re-read helper; the START OVER button (the ASTRA tab's in Mission Planner, and the old panel's with
   `--panel`); read every `simulation/windows/*.bat` for quoting.
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
   copy pre-built images with two new scripts still to be written in `simulation/docker/`, `export_images.sh`
   and `import_images.sh` (docker save/load + checksum), run, "it works if you see...", troubleshooting; optional ghcr.io publishing as instructions only.
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
bash simulation/scripts/demo.sh runway                      # VISIBLE (owner's rule), ~6 min; score report in ~/.astra_sim/runs/<date>-<time>_runway/score_report.md
bash simulation/scripts/stop_sim.sh                         # always stop what you start
```

Verified: sim_bringup 55, sim_demo 234, sim_models 63, sim_worlds 62, siyi_emulator 147, main 113,
video_cam 32, waypoint_mavros 27, wp_sender 17, mapping 20, detection 6, sim_plugins 219,
example_detector 10, example_listener 3, Docker helpers 6: **1014 passing** (Task 2 re-ran the team
packages; the simulator counts are from Task 0/1, unchanged code).
For sim_models exclude `test/acceptance`, siyi_emulator `test/e2e`, sim_demo `test/ros_e2e`; these are
separate integration jobs. Use `QT_QPA_PLATFORM=offscreen` for panel tests. Docker helper tests run with
`python3 -m pytest -q simulation/docker/test` from the repo root. Also run `ruff check ros2_ws/src`,
`bash -n` on changed shell scripts, `python3 -m sim_plugins.profile_tool docs --check`, and, when you touched
any doc, command, script name or port, `python3 simulation/scripts/check_docs.py` (the documentation index).

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
