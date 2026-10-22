# sim_worlds: the simulated fields for the SUAS search demo

This package holds the **places the simulated drone flies over**: two Gazebo Harmonic worlds, the **search
mission** for each one (a file Mission Planner and MAVProxy can load), and a **truth file** that says where the
hiker (the SUAS mannequin) and the tent really are, so a run can be scored automatically.

| world | what it is | search area | survey height |
|---|---|---|---|
| `runway` | small and fast test field near the real SUAS site | one 120 x 100 m box, 1 hiker + 1 tent, 60-120 m from the pad | 30 m |
| `suas_field` | the SUAS 2026 field (Skyway Range, Tulsa) from the handbook GPS numbers | Search Boundaries 1 and 2, 1 hiker + 1 tent each | 50 m (164 ft) |

The drone itself is **not** in these worlds: the bringup spawns it (model `iris`) where the config says.

---

## Quick start (copy and paste)

```bash
# 0. once: download the realistic 3D models (mannequin, people, cars, trees) into Gazebo's cache (~110 MB)
ros2 run sim_worlds fetch_models.sh

# 1. build (from your workspace folder, e.g. ~/ros2_ws) and load it
colcon build --symlink-install --packages-select sim_worlds
source install/setup.bash              # also adds share/sim_worlds/models to GZ_SIM_RESOURCE_PATH

# 2. look at a world WITH a window (the full demo is simulation/scripts/demo.sh, owned by the bringup)
gz sim -r "$(ros2 run sim_worlds world_info.py runway world_file)"

# 3. check a world headless in ~30 s: no errors, systems loaded, real-time factor, GPS = SITL home
ros2 run sim_worlds check_world.py runway

# 4. print or re-make the search mission
ros2 run sim_worlds make_mission.py --check "$(ros2 run sim_worlds world_info.py runway mission)" runway
ros2 run sim_worlds make_mission.py runway --altitude 35 --out /tmp/runway_35m.waypoints
```

What you should see in step 2: grass, a concrete pad with an **H** in the middle, a runway, a green canopy, and
(after the drone is spawned by the bringup) a big **"Drone camera (live)"** picture in the top-right corner.

---

## What is in the package

```
sim_worlds/
  worlds/runway.sdf, worlds/suas_field.sdf      the worlds (GENERATED, do not edit by hand)
  config/runway.yaml, config/suas_field.yaml    what the bringup needs: world file, SITL home, spawn, mission, truth
  missions/<world>_survey.waypoints            search missions, QGC WPL 110 (Mission Planner / MAVProxy "wp load")
  missions/suas_field_sb2_survey.waypoints      extra mission for Search Boundary 2
  truth/<world>.yaml                           where every target and distractor really is (for scoring)
  models/                                      ground, pad, runway, tent, grass patches, creek (our own models)
  sim_worlds/                                  Python helpers (importable): geo, survey, mission, planner,
                                               world_config, truth, gzrun, worldgen/ (the generators' parts)
  scripts/                                     tools: build_runway.py, build_suas_field.py, make_mission.py,
                                               world_info.py, check_world.py, render_preview.py, fetch_models.sh,
                                               test_sim_worlds.sh
  test/                                        unit tests (pytest)
  hooks/sim_worlds.dsv.in                      adds share/sim_worlds/models to GZ_SIM_RESOURCE_PATH
```

---

## Interface table

### Files other packages read (paths relative to `$(ros2 pkg prefix --share sim_worlds)`)

| file | read by | content |
|---|---|---|
| `config/<world>.yaml` | bringup, demo, scorer | contract fields below + `search_areas`, `survey`, `gui` |
| `worlds/<world>.sdf` | `gz sim` | the world (world name = `<world>`) |
| `missions/<world>_survey.waypoints` | bringup (MAVProxy `wp load`) | the search mission, team shape (see "Mission shape") |
| `truth/<world>.yaml` | `sim_demo` (truth_detector, score_run, demo_markers) | targets + distractors with GPS, local xyz, yaw, size |

### `config/<world>.yaml` fields

| field | example (runway) | meaning |
|---|---|---|
| `world_name` | `runway` | `<world name>` in the SDF; gz service names use it (`/world/runway/create`) |
| `world_file` | `worlds/runway.sdf` | the world to load |
| `home` | `"36.2143000,-96.0068000,210.0,0"` | lat, lon, alt (m above sea level), heading (deg) for **SITL `--home`**; equal to the world's `<spherical_coordinates>` so GPS and Gazebo agree |
| `spawn` | `{x: 0, y: 0, z: 0.25, yaw: 1.5708}` | drone start pose, metres and radians (ENU); yaw 1.5708 = nose north (heading 0) |
| `mission` | `missions/runway_survey.waypoints` | the contract mission |
| `truth` | `truth/runway.yaml` | the ground truth |
| `search_areas` | name, center, size, yaw | rectangles in local metres; `size = [along the lanes, across]` |
| `survey` | altitude_m, sidelap, frontlap, photo_interval_s, speed_mps, camera, planned | how the mission was made; `make_mission.py` re-reads it |
| `gui` | follow_target `iris`, follow_offset, camera_topic | what the world GUI expects from the bringup |

### `truth/<world>.yaml` fields

| field | meaning |
|---|---|
| `home` | world origin = pad centre = SITL home (`lat`, `lon`, `alt_msl`, `heading`) |
| `classes` | detector class names and ids: `person: "0"`, `tent: "1"` (as `detection/new_od.py` writes them) |
| `search_areas[]` | `name`, `center`, `size`, `yaw`, `corners_xy`, `corners_latlon` |
| `targets[]` | the hiker (`class: person`, `kind: mannequin`, `pose`) and the tent (`class: tent`) of each area |
| `distractors[]` | standing people (`class: person`!), vehicles, debris, bushes, flight-line objects |
| per entry | `id` (Gazebo model name), `x y z yaw` (m, rad, object origin), `lat lon` (**ArduPilot convention**), `lat_wgs84 lon_wgs84`, `alt_msl`, `size [sx sy sz]` (box in the object frame), `center` (world xyz of the box centre), `center_z`, `area`, `distance_from_home_m`, `model` |
| suas_field only | `flight_boundary` (latlon + local), `altitude_rule_agl_m` {45.7, 121.9}, `runway_1`, `runway_2_pad`, `creek_centreline_xy` |

**Two GPS conventions, on purpose.** `lat/lon` is what ArduPilot SITL (so MAVROS `/mavros/global_position/global`
and the photo names `"<lat> , <lon>.jpg"`) reports directly above the point: SITL turns Gazebo metres into GPS
with its own spherical formulas (`Location::offset`). `lat_wgs84/lon_wgs84` is the exact WGS84 value a Gazebo
NavSat sensor reports. Inside these worlds they differ by less than about 1 m. Compare detections with `lat/lon`.

### Gazebo interfaces used by the worlds

| name | type | direction | notes |
|---|---|---|---|
| `/sim/camera/video/image` | gz.msgs.Image | shown by the GUI | live drone camera (sim_models); the "Drone camera (live)" panel |
| `/gui/follow` | gz service, gz.msgs.StringMsg -> Boolean | called by the bringup with `iris` | starts the 3D view following the drone (offset -12, 0, 6 m in the drone frame, gain 0.02 from the world GUI config). gz-gui 8 does **not** start following from `<follow_target>` alone |
| world systems | Physics, UserCommands, SceneBroadcaster, Sensors (ogre2), Imu, NavSat | | physics step 1 ms, real-time factor 1 (ArduPilot lock-step) |
| `GZ_SIM_RESOURCE_PATH` | environment | set by the hook | must contain `share/sim_worlds/models` |
| Fuel cache | `~/.gz/fuel` (or `$GZ_FUEL_CACHE_PATH`) | filled by `fetch_models.sh` | pinned versions of 17 Fuel models |

### Python API (import from any package; only PyYAML needed)

| module | main functions |
|---|---|
| `sim_worlds.geo` | `Home.parse()`, `Home.sitl_string()`, `enu_to_latlon(x, y, home, convention='ardupilot'|'wgs84')`, `latlon_to_enu()`, `enu_to_wgs84()`, `wgs84_to_enu()`, `heading_to_yaw()`, polygon helpers |
| `sim_worlds.survey` | `footprint(alt)`, `Rect`, `plan_lanes(rect, alt, sidelap)`, `survey_speed(alt, frontlap, photo_interval_s)` |
| `sim_worlds.mission` | `read_waypoints()`, `write_waypoints()`, `build_survey_mission()`, `team_indices()`, `check_team_shape()`, `describe()` |
| `sim_worlds.planner` | `SurveySettings`, `plan_survey_mission(home, rect, settings)`, `settings_from_config()`, `rect_from_area()` |
| `sim_worlds.world_config` | `load_world_config('runway')` -> WorldConfig (absolute paths), `available_worlds()`, `share_dir()` |
| `sim_worlds.truth` | `load_truth()`, `targets(truth, 'person')`, `validate_truth()`, `nearest_target(truth, lat, lon)`, `CLASS_IDS` |
| `sim_worlds.gzrun` | `HeadlessGazebo` (safe headless server in its own partition), `inject()`, `navsat_model_sdf()` |

### Tools (`ros2 run sim_worlds <tool>`)

| tool | does | main options |
|---|---|---|
| `world_info.py` | prints config facts for shell scripts | `<world> [world_file|home|spawn|spawn_x..|mission|truth|models]`, `--list` |
| `make_mission.py` | (re)makes or checks a mission | `<world> [--altitude --sidelap --frontlap --photo-interval --speed --area --out]`, `--check FILE` |
| `check_world.py` | headless load test + real-time factor + GPS probes | `<world|file.sdf> [--seconds 12] [--json FILE]` |
| `render_preview.py` | pictures from cameras you place (no drone, no window) | `--world --out --camera NAME:x,y,z,r,p,y[,w,h,hfov] | NAME:x,y,z@tx,ty,tz[,w,h,hfov]` |
| `fetch_models.sh` | pre-downloads the Fuel models | `--check`, `--list` |
| `build_runway.py`, `build_suas_field.py` | regenerate a world + its config, truth, missions | `--seed`, `--assets fuel|primitive`, `--altitude`, `--textures`, `--out-dir` |
| `test_sim_worlds.sh` | every check in one go | `--quick` (no Gazebo) |

---

## Mission shape (why the missions look exactly like this)

Last year's flight code (`main/src/main_controller.py`, `waypoint_mavros/waypoint.py`,
`mapping/do_digi_cam_trigger.py`) derives everything from the **positions** of items in the mission, so the
generated missions follow the team's own layout `TAKEOFF, WP, DO_DIGICAM_CONTROL, WP ..., WP, WP, RTL`:

| seq (runway) | item | role in the team code |
|---|---|---|
| 0 | HOME (NAV_WAYPOINT, frame 0) | replaced by ArduPilot at arming; MAVROS publishes it as item 0, so indices match everywhere |
| 1 | NAV_TAKEOFF 30 m | `takeoff_index` |
| 2 | NAV_WAYPOINT, start of lane 1 (the corner nearest home) | |
| 3 | **DO_DIGICAM_CONTROL** | ArduPilot prints **`Mission: 3 DigiCamCtrl`**; the trigger node starts its 1 Hz `/camera/trigger` photo loop |
| 4 | DO_CHANGE_SPEED 5.5 m/s | lane speed (photo overlap); optional (`--speed 0` removes it) |
| 5 ... 8 | NAV_WAYPOINT lane ends | lawn-mower |
| 9 = R-2 | NAV_WAYPOINT, end of the LAST lane | `buffer_wp = rtl_index - 2`: the photo loop **stops** when it is reached |
| 10 = R-1 | NAV_WAYPOINT, middle of the search area | `last_before_rtl`: main_controller switches to **GUIDED** here, waits until every photo is processed, then inserts one waypoint per target at index R (49 m) and resumes AUTO |
| 11 = R | NAV_RETURN_TO_LAUNCH | `rtl_index` |

Rules the checker (`mission.check_team_shape`) enforces: one TAKEOFF at 1, RTL last, R-1 and R-2 are
NAV_WAYPOINTs (DO_ items never produce MISSION_ITEM_REACHED), the DigiCamCtrl follows TAKEOFF + the first lane
waypoint, all NAV items in frame 3 (metres above home) inside the allowed altitude band. DO_ items carry all-zero
x/y because MAVProxy multiplies x/y by 1e7 for every item when it uploads. The `.waypoints` files have no comments
because Mission Planner does not allow them; this table is their documentation.

---

## Survey maths (ARCHITECTURE.md section 8)

Camera straight down, nose (and gimbal, follow mode) along the lane, 2560 x 1440 photos, 81 deg lens:
`footprint width = 2 h tan(40.5 deg) = 1.708 h` across the lane, `height = width x 1440/2560` along it.
Lanes are spaced at most `width x (1 - sidelap)` and spread so the outer photo strips end exactly at the area's
edges. Speed = `footprint height x (1 - frontlap) / photo interval` (one photo every 2 s is what the real
pipeline delivers although the trigger asks every 1 s), rounded down to 0.5 m/s, at most 10 m/s.

| world | height | footprint | cm/px | lanes | spacing | speed | survey path |
|---|---|---|---|---|---|---|---|
| runway | 30 m | 51.2 x 28.8 m | 2.0 | 3 x 120 m | 24.4 m | 5.5 m/s | 409 m |
| suas_field SB1 | 50 m | 85.4 x 48.0 m | 3.3 | 3 x 262 m (east-west) | 31.1 m | 9.5 m/s | 848 m |
| suas_field SB2 | 50 m | 85.4 x 48.0 m | 3.3 | 3 x 259 m (north-south) | 31.4 m | 9.5 m/s | 840 m |

Why 30 m in the runway world: it is a quick test field, below the SUAS 150 ft floor on purpose (short climb,
lots of overlap, the hiker is ~90 px long). The competition field flies 50 m (inside 46-122 m; the team drops
from 49 m). If the camera node zooms (last year's `capture_setup` racks the lens to 5x), the footprint shrinks
5x and these lanes no longer cover the area: re-plan with a narrower `hfov` or keep 1x.

---

## The worlds in detail

**runway** (`scripts/build_runway.py`, seed 7). Home lat 36.2143, lon -96.0068, ground 210 m MSL (next to the real
field). Pad (12.2 m, H) at the origin, asphalt runway 152 x 12 m east-west 32 m south, flight line south-west
(canopy, table, chairs, the pilot, a pickup). Search box centre (25, 95) m, 120 x 100 m, cones and striped poles
at its corners. Inside: the hiker (Rescue Randy mannequin lying on its back, a small bush next to it), an open
pop-up tent, an SUV, 3 bushes, a barrel, 2 drums, a carton, a tarp, 2 logs, a pallet, a tyre, dirt patches.
A tree line beyond the north edge, scattered trees, dry-grass patches.

**suas_field** (`scripts/build_suas_field.py`, seed 2026). The SUAS 2026 site from the Team Handbook: flight
boundary (11 points), Search Boundary 1 (36.211822-36.213147 N, 96.005545-96.002627 W, 262 x 147 m) and 2
(36.213469-36.215795 N, 96.004286-96.002635 W, 148 x 259 m), Runway 1 (020 deg) and Runway 2 = the VTOL pad = the
origin. Each search boundary: one hiker (lying face up/down, sitting or standing; the two never share a pose;
"open", "near a bush" or "next to a vehicle", as the rules say "surrounded/covered by bushes/trees/vehicles"),
one open pop-up tent, 1-3 vehicles, 1-3 standing people (the detector calls them "person" too), 5-8 bushes and
~30 pieces of debris. A creek with trees along the north, tree lines west and south-east.

---

## Assets: realistic models from Gazebo Fuel, with offline fallbacks

The worlds reference Fuel **mesh files** at **pinned versions** inside our own static models
(`sim_worlds/worldgen/assets.py`), because the Fuel models themselves are not usable as-is (some are not static,
lift their link in the air, or pull in extra plugins, and none lies down). Every placement was checked in
side-view renders.

| object | Fuel model (version) | licence |
|---|---|---|
| hiker lying / standing | OpenRobotics / Rescue Randy (3), lying = rotated 90 deg | CC-BY 4.0 |
| hiker sitting | OpenRobotics / Rescue Randy Sitting (8) | CC0 |
| standing people | Standing person (3), Walking person (3), Casual female (4), FemaleVisitor (2) | CC0, CC-BY 4.0 |
| vehicles | Hatchback blue (5), SUV (4), Pickup (4) | CC0 |
| trees and bushes | Oak tree (7), Pine Tree (6) (needles tinted darker); bushes = low wide oaks | CC0 |
| props | Construction Cone (3), Construction Barrel (4), 55gal_Drum (1), foldable_chair (3), foldable_table (4), tent (4) = flight-line canopy | CC0, CC-BY 4.0 |

CC-BY credits: models by **Open Robotics** (Rescue Randy, FemaleVisitor, 55gal_Drum, foldable_chair,
foldable_table, tent), from https://fuel.gazebosim.org.

Made by us (procedural, `worldgen/primitives.py`, `textures.py`): the **open pop-up dome tent** (the search
target; the handbook: "in its open state and of the pop-up variant"; Fuel has no such tent), debris (logs,
plywood, roof sheets, lumber, tarps, tyres, bins, coolers, cartons, mattresses, pallets), the ground, the pad,
the runway and the grass/dirt patches. Not used on purpose: Fuel "Hatchback" (wheel textures missing) and
"Cardboard box" (texture path broken).

Offline: `build_runway.py --assets primitive` (or `build_suas_field.py`) writes the same layout from simple shapes
only (mannequin from capsules, box cars, ellipsoid bushes); no download needed. If a Fuel model is missing at run
time, the world still loads and only that object is missing (Gazebo prints an error).

---

## Re-generate a world

```bash
cd <repo>/ros2_ws/src/simulation/sim_worlds            # the SOURCE folder (scripts write into it)
python3 scripts/build_runway.py                          # same seed -> exactly the same files
python3 scripts/build_suas_field.py --seed 7             # a different layout (targets elsewhere)
python3 scripts/build_runway.py --hiker-pose sitting     # lying_face_up | lying_face_down | sitting | standing
python3 scripts/build_runway.py --textures               # re-paint the shared pictures (fixed seed 2026)
python3 -m pytest -q test                                # the committed worlds must equal the generator output
```

Needs numpy and Pillow (PyYAML for the missions). **The ground**: one seamless 4096 x 4096 grass picture covering
160 x 160 m (3.9 cm per pixel, about what the camera resolves from 30-50 m) repeats over a 12 km square. It only
has small-scale variation (tufts, 1-10 m darker/drier spots) and deliberately WEAK big blotches, because big
blotches would show the 160 m repeat as a regular pattern from high up. The big dry and greener areas are patch
pictures (soft, dithered edges) laid on top at random places, so they never repeat. Seen from the drone (30-50 m)
the ground looks like a natural pasture; from very high up (300 m+) it looks evenly olive with random patches.
(Tried and rejected: several tile pictures with shared borders laid out at random: the shared borders formed a
visible grid. Gazebo heightmap texture blending: ogre2 only draws the first texture.)

---

## Testing

```bash
ros2 run sim_worlds test_sim_worlds.sh            # unit tests + missions + Fuel cache + headless loads
ros2 run sim_worlds test_sim_worlds.sh --quick    # without Gazebo
ros2 run sim_worlds render_preview.py --world "$(ros2 run sim_worlds world_info.py runway world_file)" \
    --out /tmp/prev --camera "box:25,95,50,0,1.5708,1.5708"     # a 2560x1440 photo from 50 m over the search box
```

## Troubleshooting

| symptom | cause and fix |
|---|---|
| world loads but people/cars/trees are missing, `Unable to find file ... fuel.gazebosim.org` | models not in the cache and no internet: `ros2 run sim_worlds fetch_models.sh` while online, or use a `--assets primitive` world |
| first start hangs for minutes | Gazebo is downloading the Fuel models: run `fetch_models.sh` once beforehand |
| `Unable to find uri[model://sim_worlds_ground]` | `GZ_SIM_RESOURCE_PATH` misses `share/sim_worlds/models`: `source install/setup.bash` (the hook adds it) |
| the drone's GPS is ~200 m off / home is wrong | SITL `--home` must be `world_info.py <world> home` (equal to the world's `<spherical_coordinates>`) |
| photos never stop after the survey | the mission's item R-2 must be a NAV_WAYPOINT (run `make_mission.py --check`) and `waypoint_manager` must be running |
| no "Drone camera (live)" picture | the drone model (sim_models) is not spawned yet or does not publish `/sim/camera/video/image` |
| the 3D view does not follow the drone | the bringup must call `/gui/follow` with `iris` after the drone exists |
| strict XML tools reject an edited world | never put `--` inside `<!-- -->` comments (the generators avoid it) |
