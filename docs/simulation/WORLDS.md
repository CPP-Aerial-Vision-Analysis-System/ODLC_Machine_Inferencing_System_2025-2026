<!--
WORLDS.md -- PURPOSE: explain generated simulator worlds, their coordinate/mission contracts, how to edit
and check the working synthetic fields, and the real-terrain data pipeline and outstanding acceptance work.
-->

# Worlds, terrain and missions

2026-11-07 addition: [SUAS 2027 training environments](SUAS_2027.md) documents a separate opt-in Blender
generator for fictional coastal terrain, colored mannequins/suitcases, clutter and landing-zone candidates.
It produces an external bundle; the 2026 worlds and their default flight behavior stay as described here.
Its local evaluator is independent of the person-and-tent truth validator and the 2026 score.

Status on 2026-11-07: `runway` and `suas_field` are generated working worlds. **`suas_field_real` now exists end to
end** (all four generator stages, offline checks, Gazebo load check, box-drop ground check and one complete scored
flight with the team's real detector, see [Real-terrain world](#real-terrain-world-suas_field_real)). It is a first
accepted version, not the final one: trees and ground sharpness are still being improved, and it has not been compared
with the owner's Mission Planner screenshots yet.

Start with the [simulator README](../../simulation/README.md), then [architecture](ARCHITECTURE.md) for how
Gazebo, SITL and the team's code fit together. [Profiles](PLUGINS.md) select a world without changing flight
code; [Docker](DOCKER.md) covers container paths.

## Choose a world

| Name | Ground/targets | Mission and status |
|---|---|---|
| `runway` | Small synthetic field, 120 × 100 m search area, one hiker and one tent | 30 m survey; short default regression demo, flown in SITL |
| `suas_field` | Synthetic SUAS site using handbook boundary/runway coordinates; two search areas | 50 m surveys, one mission per search area; generated and unit tested |
| `standin` | ArduPilot Gazebo runway, no ASTRA search targets | Supply a mission explicitly; useful for bringup |
| `suas_field_real` | Real USGS lidar ground, NAIP 2023 aerial colour, the real trees and buildings, the three real poles; two search areas | One legal survey height per area (58.7 m and 55.8 m above the pad); flown once end to end; see [Real-terrain world](#real-terrain-world-suas_field_real) |

From the **built Linux checkout**, after sourcing its environment:

```bash
source simulation/setup/sim_env.sh
ros2 run sim_worlds world_info.py --list
ros2 run sim_worlds world_info.py runway mission
bash simulation/scripts/demo.sh runway            # add --headless only on a computer without a screen
bash simulation/scripts/stop_sim.sh
```

Only run a single heavy job at a time, including generators, builds and simulator checks. On the shared WSL
laptop, runners acquire `/mnt/wsl/astra_heavy.lock`, then `/tmp/astra_sim.lock` for simulator runs. Build from
the editing checkout with `bash simulation/setup/03_build.sh --workers 4`; Windows-drive checkouts mirror to
Linux. `LOCAL_NOTES.md` holds the owner's automation details and stays uncommitted.

## The contract every runnable world must meet

The package root is `ros2_ws/src/simulation/sim_worlds`; installed resources live under the package's ROS
share directory. A world needs these generated files:

| File | Consumer and required meaning |
|---|---|
| `worlds/<name>.sdf` | Gazebo scene; `<world name>` agrees with the config |
| `config/<name>.yaml` | Bringup paths, SITL home, spawn pose, mission, truth, camera/survey settings |
| `truth/<name>.yaml` | Target IDs/classes, poses, bounding boxes and GPS; scorer's answer key |
| `missions/<name>_survey.waypoints` | QGC WPL 110 mission compatible with the team's index logic |
| `models/` or model cache | Geometry/textures resolved by `GZ_SIM_RESOURCE_PATH` and Fuel |

The SDF `<spherical_coordinates>` and SITL `--home` must agree in latitude, longitude, altitude and heading.
World `x` is east, `y` north and `z` up (ENU); spawn yaw is in radians, with `π/2` pointing north. MAVLink frame
3 mission altitudes are **metres above home**, not local height above uneven ground. Keep both references
explicit when adding terrain.

Truth files retain two GPS conventions. `lat/lon` match ArduPilot's spherical offset arithmetic so detections
from team photo names compare fairly. `lat_wgs84/lon_wgs84` are the exact WGS84 tangent-plane conversion used
for real map coordinates. Use `sim_worlds.geo` helpers; do not silently mix these conventions.

The team expects HOME at 0, TAKEOFF at 1, a first lane waypoint followed by DigiCamCtrl, survey lane waypoints,
two final NAV waypoints and RTL. `buffer_wp = rtl_index - 2` ends photos; `last_before_rtl` holds for detection
completion before target insertion. DO items do not generate waypoint-reached messages. Keep all NAV survey
items in frame 3 and validate generated missions before flying.

## Edit or regenerate the working worlds

Edit generators, not their generated SDF/YAML/mission files. From the package source folder:

```bash
python3 scripts/build_runway.py --help
python3 scripts/build_suas_field.py --help
python3 scripts/build_runway.py --seed 7
python3 scripts/build_suas_field.py --seed 2026
python3 -m pytest -q -p no:cacheprovider test
```

The same seed should regenerate the same layout. `--assets primitive` uses simple shapes instead of Fuel
meshes. `--textures` regenerates shared procedural textures; it changes more than target placement, so review
the resulting images and diff. For Fuel assets, fetch the pinned models once:

```bash
bash scripts/fetch_models.sh
```

Rebuild/sync through `03_build.sh`, then check the world from the Linux build checkout:

```bash
source simulation/setup/sim_env.sh
ros2 run sim_worlds make_mission.py --check "$(ros2 run sim_worlds world_info.py runway mission)" runway
ros2 run sim_worlds check_world.py runway --seconds 12
```

The Gazebo check is a heavy job. It checks load errors, systems, simulation speed and coordinate probes;
follow it with a full scored demo for behavior. From the package source folder, use
`bash scripts/test_sim_worlds.sh --quick` for the package's non-Gazebo
checks. See [MISSION_PLANNER.md](MISSION_PLANNER.md) for operator mission handling.

### Survey geometry

For a nadir camera at local ground clearance `h`, horizontal field of view `f` gives width
`2 h tan(f/2)`; image aspect ratio gives the along-track footprint. Lane spacing accounts for sidelap; speed
and photo interval account for frontlap. The defaults use 81° and 2560 × 1440 photos. A 5× zoom cuts footprint
dimensions by about five, so a mission planned for 1× cannot claim the same coverage at 5×. The demo's survey
zoom cap preserves that planned view; it does not change hardware camera defaults.

## Real-field pipeline: what exists

`sim_worlds/realfield/` keeps GIS preparation separate from runtime geometry. This lets a future teammate fly
from compact derived assets without installing GIS libraries or downloading raw point clouds.

| Stage | Input → output | Code/status |
|---|---|---|
| `prepare` | Private raw cache → aligned `.npy` rasters, feature JSON, tree CSV | `prepare.py`, `trees_detect.py`; implemented, synthetic regressions pass, full new data pass pending |
| `pack` | Prepared grids → `data/suas_field_real/` compact derived site pack | `pack.py`, `sitepack.py`; implemented; tracked pack predates the latest numerical fixes |
| `upscale` (optional) | Site pack → AI-sharpened ground picture of the search areas (cache, not in git) | `upscale.py`; Real-ESRGAN x4 on the GPU, about 25 s; used by `assets` when it exists |
| `assets` | Site pack → terrain/tree/building Gazebo models and baked textures | `assets_build.py` with `terrain.py`, `trees.py`, `texture.py`, `structures.py`; about 40 s (standard) to 2 min (hd) |
| `world` | Site pack → SDF, config, truth and missions | `world_build.py`; seconds |
| `check`, `preview` | Pack/generated models → validation and pictures | `checks.py` (offline PASS/FAIL list), `previews.py` (overview, hillshade, per-area top views, altitude profiles) |

`03_build.sh` (native) and the Docker helper both run `build_real_field.py assets --install --if-needed` after the
build, so the models are made once and kept up to date; a failure there only warns (the other worlds do not need
them). The world, config, truth and mission files are small and are committed.

### Prepare data without changing ROS's Python

These are maintainer commands, run from the package source folder. Raw data belongs in
`~/.astra_sim/world_data/suas_field_real/raw/`, never in Git. `ASTRA_WORLD_DATA` selects another cache root.

```bash
sudo apt-get install -y python3-venv
bash scripts/fetch_real_field_data.sh --setup-venv
bash scripts/fetch_real_field_data.sh --list
bash scripts/fetch_real_field_data.sh --optional
bash scripts/fetch_real_field_data.sh --check
"$HOME/.astra_sim/gis_venv/bin/python" scripts/build_real_field.py prepare
"$HOME/.astra_sim/gis_venv/bin/python" scripts/build_real_field.py pack
```

These downloads/generation steps are substantial; run them serially with other heavy work. The wrapper
creates a separate GIS environment with rasterio, pyproj, laspy/lazrs, scipy, numpy, Pillow and PyYAML; it does
not upgrade ROS's numpy or torch. Required layers download by default; `--optional` adds optional layers,
some of which may be copy-only. `--from <folder>` imports existing inputs and still verifies checksums.
`--record` changes the manifest and is for deliberate, reviewed source updates only.

The manifest is `data/real_field_manifest.json`: file hashes for ordinary downloads, decoded-pixel hashes
for raster windows and sorted-feature hashes for vector data. Do not replace a mismatch with a new checksum
just to make a failing download pass. Inspect source date, coverage and provenance first.

### Research fixes and evidence still needed

The numerical code now includes these changes; the rebuilt pack/world must still prove their effect:

| Requirement | Current implementation / Task 5 check |
|---|---|
| Reject floating lidar haze | Drop points above local DTM + 35 m before DSM maxima; reject tree tops over 31 m. Check no false 45 m trees survive. |
| Plausible tree counts | Lidar envelope plus photo-crown peaks and Meta/young-lidar confirmation; measure roughly 130 trees in SB1, 75 in SB2, 3,100 in the flight boundary. Current photo peak window is 10 m; handoff's DeepForest or 6–8 m target remains to reconcile with counts. |
| Three SB1 poles | Source coordinates are in `site.MANUAL_STRUCTURES`; research pad-frame points `(250.6,-246.4)`, `(268.6,-245.4)`, `(357.6,-246.4)`. Diameters 0.3 m, heights 10.2/10.2/10.1 m. Reconcile final 10.2 m requirement for the third pole. |
| Sentinel reflectance | Remove the +1000 L2A storage offset before NDVI and ratios; require enough grass samples, otherwise preserve the site fallback. |
| Buildings | Prefer median lidar roof height over footprint/tag defaults; exclude roofs from tree detection. |
| Legal survey altitude | Derive one above-home altitude per area from maximum/minimum terrain, then apply 150–400 ft AGL limits. Expected feasible intervals: SB1 57.7–124.3 m, SB2 54.8–122.0 m above pad; check against rebuilt terrain. |
| Ground physics/performance | Mesh collision, not heightmap; under about 2 million triangles; check visual/collision agreement with box drops and real-time factor at least 0.8. |
| Sloped-ground mission | Generate all contract files, fly end to end, record actual payload landing error and score report. |

The code also defines a 1 m altitude margin and 122 m above-home cap; apply the tighter of geometric and
configured limits without confusing above-home with AGL. Main-controller geolocation uses `rel_alt_m` minus
a configured constant ground height (zero by default), not a spatial terrain intersection. Terrain slope is
also a flight-code review issue; a detailed map alone does not correct the aiming math.

## Real-terrain world (`suas_field_real`)

### Run it (also with Mission Planner)

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026          # the Linux checkout (03_build.sh made it)
source simulation/setup/sim_env.sh
bash simulation/scripts/demo.sh suas_field_real           # Gazebo + RViz open; searches Search Boundary 1
bash simulation/scripts/demo.sh suas_field_real --manual  # Mission Planner mode: you upload, arm and start AUTO
```

The first `03_build.sh` makes the models (`~/ODLC.../ros2_ws/install/sim_worlds/share/sim_worlds/models/rf_*`). If
you only changed the generator, re-make them with `python3 scripts/build_real_field.py assets --install` from the
package folder (add `--quality standard` for half the graphics memory in the search areas). Mission Planner connects
exactly as for the other worlds ([MISSION_PLANNER](MISSION_PLANNER.md)): UDP 14550, ASTRA tab, camera in the HUD. The
contract mission is `missions/suas_field_real_survey.waypoints` (Search Boundary 1, 58.7 m above the pad);
`missions/suas_field_real_sb2_survey.waypoints` is the same for Search Boundary 2 (55.8 m). In Mission Planner use
PLAN → Load WP file to load either one.

Heights are **metres above the pad** (frame 3), and the ground rises 2 to 12 m across Search Boundary 1, so the
generator derives the height from the real ground: at least 150 ft above the highest ground under the area and at
most 400 ft above the lowest, and it spaces the lanes for the camera height over the HIGHEST ground (the smallest
photo footprint). `check` verifies this for every mission.

### Change the targets, regenerate, check

```bash
cd ros2_ws/src/simulation/sim_worlds
python3 scripts/build_real_field.py world --seed 7                       # other target places, same terrain
python3 scripts/build_real_field.py world --hiker-poses sitting,standing --tent-colours red,blue
python3 scripts/build_real_field.py check                                # offline PASS/FAIL list (no Gazebo)
python3 scripts/build_real_field.py preview --out /tmp/rf_preview        # pictures to look at
ros2 run sim_worlds check_world.py suas_field_real --seconds 20          # Gazebo load, speed, GPS probes (heavy job)
~/.astra_sim/upscale_venv/bin/python scripts/build_real_field.py upscale && python3 scripts/build_real_field.py assets --install   # sharper ground (after upscale_env.sh)
```

Targets are placed only on open, level (under 12 degrees) ground of class grass, dry grass, bare soil or gravel, clear
of every real crown and the forest canopy, and never on the three real poles. Everything stands on the lidar ground
height under it. The sun is the sun of the aerial photo (15 September, 11:00 local), so photo shadows and Gazebo
shadows point the same way.

### The optional sharper ground (AI upscale)

The aerial photo has 25 cm pixels; the drone camera sees 2 to 3 cm per pixel. `upscale` runs Real-ESRGAN x4plus
(BSD-3 licence, GPU, about 25 s) over the search areas and the pad and stores the result in
`~/.astra_sim/world_data/suas_field_real/upscaled/` (never in git). `assets` paints the ground tiles from it when it
exists (`--upscale off` ignores it). The AI invents plausible fine texture (grass blades, slab cracks); it adds no
objects. Because invented texture could trigger a detector, compare the false-alarm counts of two flights (with and
without) before trusting it. One-time environment: `bash simulation/setup/upscale_env.sh` (its own virtual
environment; ROS's Python is untouched).

### Evidence (2026-11-07)

| Check | Result |
|---|---|
| `build_real_field.py check` | all PASS: 1.69 M triangles to draw (under 2 M), mesh physics, home equals pack home, targets on open level ground, both missions legal |
| `check_world.py` (headless Gazebo, 20 s) | no `[Err]` lines, all six systems loaded, real-time factor median 0.999 (min 0.65), GPS probe at the origin exact |
| Box drops on the physics ground (28 points) | boxes rest on the lidar ground within 3.4 cm in the search areas and at the pad; 32 cm at a far point with the coarse 0.6 m tolerance; **outside the flight boundary there is no physics ground by design** (a box dropped at 700 m east fell through) |
| Scored flight, real detector (`demo.sh suas_field_real`) | landed after 7.4 min; hiker found (confidence 0.92, payload landed 16 m from it); the tent pick was a false alarm at the west edge (222 m error); Search Boundary 2 is not part of the contract mission. Detection quality per photo: person recall 0.86, precision 0.25; tent recall 0.80, precision 0.04 (515 tent false alarms in 110 photos: white dirt patches and bright ground look like tents to the model) |
| Georeference against independent map data | OpenStreetMap roads and the paved strip lie on the photo's pavement to within 1 to 2 m (roof outlines shift because tall roofs lean in aerial photos); distances between handbook points agree with WGS84 geodesics to 0.2 % (the check itself uses a spherical formula) |
| End-to-end positional accuracy (flight photos matched to the ground picture by SIFT features) | across the flight direction 0 to 2 m; **along the flight direction the photo's GPS tag is 5 to 8 m off, and the sign flips with the flight direction**: the tag is the last GPS fix, which was 0.3 to 1.7 s old at the shutter (`position.age_s` in each photo's json) at 8.5 m/s. The terrain is not shifted; the timestamped GPS is late. See the next section |

### Why the drone icon in Mission Planner can seem to be "somewhere else" than the camera picture

1. **Timing.** Mission Planner draws the drone at the last position it received (MAVLink position stream), and the
   photo tag uses the last GPS fix too; at 8.5 m/s one second of delay is 8.5 m. In the 2026-11-07 flight the
   fix was 0.2 s old on median and 2.3 s at worst. Fly slower (`DO_CHANGE_SPEED`) when comparing by eye.
2. **The map under Mission Planner is not the photo the world is made from.** Mission Planner's map (Google or Bing
   tiles) has its own georeference error (often a few metres) and is from another date than NAIP (18 Sept 2023).
   Trees, ponds and dirt patches change between dates; the world follows the 2017 lidar and the 2023 photo.
3. **The 3D trees are drawn on top of a ground picture whose tree crowns were painted out.** From above a crown is a
   modelled blob, not the satellite's pixels (being improved).
4. Real error budget of the world itself: about 1 to 2 m (lidar, NAIP and OpenStreetMap agree to that level).

To compare properly, put Mission Planner screenshots (flight area, each search boundary, close views of the pond,
tracks and tree lines) in `D:\suas_scratch\terrain_ref\` and compare them with `build_real_field.py preview`.

### Data caveats that remain

- The lidar is from 2017 (three soil piles were levelled and a wetland dug since; both are patched in the DTM, see
  `site.DTM_EDITS`; the wetland depth of 0.8 m is an estimate).
- Building heights fall back to defaults when the lidar has none ("0 with a lidar height" in the assets log).
- A wire between the three poles is likely but unconfirmed and is not drawn; the poles alone are.

## Data permissions and public repository rules

| Source | Use in the pipeline | Attribution/constraint |
|---|---|---|
| USGS 3DEP DTM/lidar | Elevation, roof/canopy heights | Public-domain source; retain USGS credit and provenance |
| USDA NAIP 2023 | Ground color and crown observations | Public-domain source; retain date/credit |
| OpenStreetMap | Roads, water, runway, pipeline/building vectors | ODbL 1.0; preserve attribution and relevant database obligations |
| Microsoft Global ML Building Footprints | Building outlines | CDLA Permissive 2.0, distinct from OSM's license |
| Meta/WRI canopy height | Confirmation/height estimates | CC-BY 4.0; retain attribution |
| Copernicus Sentinel-2 | Grass reflectance ratios | Preserve Sentinel attribution and scene metadata |
| Poly Haven | Fine ground textures | CC0; retain source provenance |
| OK Aerial 2025 / OKMaps | Internal evergreen classification mask only | No written redistribution license recorded; **no source pixels in Git, generated world textures or published images** |

The existing compact pack is derived data with its own `LICENSES.md` and source entries in `pack.json`.
Keep raw downloads, prepared large rasters, run photos/tlogs, model weights, generated bulky assets, secrets,
personal email and machine-specific paths out of commits. Regenerating the pack requires reviewing both
provenance and what pixels are included. Correct license labels do not establish permission to redistribute
an otherwise unlicensed source.

Google imagery or Google 3D is not an input to these textures or offline models; see
[GOOGLE_EARTH.md](GOOGLE_EARTH.md) for the separate live comparison workflow.

## Tests and acceptance records

The current `sim_worlds` suite contains the original world/mission checks plus synthetic research regressions:

```bash
python3 -m pytest -q -p no:cacheprovider ros2_ws/src/simulation/sim_worlds/test
```

Synthetic tests exercise Sentinel offsets and empty/sparse masks, lidar haze before maxima, roof heights and
triangulation, tree confirmation/roof exclusion, pole geometry and license metadata. They do not establish
real-data tree counts, flight performance, terrain alignment or drop accuracy.

Task 5 evidence must retain generator settings, counts, height ranges, model triangle totals, Gazebo speed,
ground-drop results and the real-field mission's score report. Update [HANDOFF.md](../HANDOFF.md), matching
`AGENTS.md` sections and this status only after those checks pass.
