<!--
WORLDS.md -- PURPOSE: explain generated simulator worlds, their coordinate/mission contracts, how to edit
and check the working synthetic fields, and the real-terrain data pipeline and outstanding acceptance work.
-->

# Worlds, terrain and missions

Status on 2026-10-24: `runway` and `suas_field` are generated working worlds. **`suas_field_real` is still work
in progress**: source preparation, a compact site pack and partial geometry helpers exist; it has no completed
world/config/truth/mission or end-to-end flight evidence. Its CLI advertises unfinished stages. Do not use the
presence of an argparse subcommand as evidence that its implementation exists.

Start with the [simulator README](../../simulation/README.md), then [architecture](ARCHITECTURE.md) for how
Gazebo, SITL and the team's code fit together. [Profiles](PLUGINS.md) select a world without changing flight
code; [Docker](DOCKER.md) covers container paths.

## Choose a world

| Name | Ground/targets | Mission and status |
|---|---|---|
| `runway` | Small synthetic field, 120 × 100 m search area, one hiker and one tent | 30 m survey; short default regression demo, flown in SITL |
| `suas_field` | Synthetic SUAS site using handbook boundary/runway coordinates; two search areas | 50 m surveys, one mission per search area; generated and unit tested |
| `standin` | ArduPilot Gazebo runway, no ASTRA search targets | Supply a mission explicitly; useful for bringup |
| `suas_field_real` | Intended real elevation, aerial-derived color, vegetation and structures | Generator incomplete; Task 5 acceptance pending |

From the **built Linux checkout**, after sourcing its environment:

```bash
source simulation/setup/sim_env.sh
ros2 run sim_worlds world_info.py --list
ros2 run sim_worlds world_info.py runway mission
bash simulation/scripts/demo.sh runway --headless
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
| `assets` | Site pack → terrain/tree/building Gazebo models and baked textures | Geometry helpers exist (`terrain.py`, `trees.py`, `structures.py`, `texture.py`); `assets_build.py` is missing |
| `world` | Site pack → SDF, config, truth and missions | CLI dispatch exists; `world_build.py` is missing |
| `check`, `preview` | Pack/generated models → validation and pictures | CLI dispatch exists; `checks.py` and `previews.py` are missing |

The advertised `all` stage needs those missing modules too. `03_build.sh` currently does not generate a
finished real field. The Docker helper guards on the existence of `assets_build.py`; that guard skips the
unfinished stage rather than proving the assets exist. Some script headers describe planned final behavior;
the table above records the present implementation.

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

Do not run the following until their missing implementation and installation work is completed:
`assets --install`, `world`, `check`, `preview`, `all`, and `demo.sh suas_field_real --headless`. Their intended
sequence is prepare → pack → assets → world → offline checks → Gazebo ground/performance checks → scored flight.

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
