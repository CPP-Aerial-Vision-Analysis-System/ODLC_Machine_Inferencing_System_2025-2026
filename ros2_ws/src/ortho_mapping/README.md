# ortho_mapping — post-flight batch orthomosaic builder

Builds **one final, georeferenced, north-up map image** from all the photos
`video_cam` captured during the mapping mission. It runs **after** the flight,
when no more images will be taken: it collects everything in
`ros2_ws/src/video_cam/mapping_photos/`, projects each photo onto the ground
using its GPS position + heading + altitude, and blends them into a single
mosaic with a `.jgw` world file (loadable in QGIS / Mission Planner / Google
Earth as a georeferenced overlay).

The old `mapping` package (feature-based incremental stitcher) is **not used**
and is superseded by this package.

---

## How to run

### 1. On the Jetson, as a ROS2 node (normal mission flow)

```bash
cd ~/ros2_ws
colcon build --packages-select ortho_mapping video_cam
source install/setup.bash

# start the node (it stays passive until told to map)
ros2 launch ortho_mapping ortho_mapping.launch.py

# after the flight, when all images are downloaded:
ros2 topic pub --once /mapping/command std_msgs/msg/String "data: MAP_START"
```

Triggers (any of):
- `MAP_START` on `/mapping/command` (also accepts `START`, `STITCH_START`)
- a GCS statustext containing `MAP_START` on `/mavros/statustext/recv`
- `autostart:=true` launch argument (maps immediately on startup)

Progress and the result are published on `/mapping/status` and echoed to the
GCS through `/mavros/statustext/send`. Output goes to
`ros2_ws/src/video_cam/mapping_output/`:

```
mapped_20260714-153012.jpg          the map
mapped_20260714-153012.jgw          world file (EPSG:4326 georeference)
mapped_20260714-153012_report.json  per-image status, coverage, skip reasons
```

### 2. Anywhere, without ROS (laptop, copied SD folder)

```bash
python3 -m ortho_mapping.offline_map --dir /path/to/mapping_photos
# common knobs:
python3 -m ortho_mapping.offline_map --dir ./photos --alt 25 --gsd-cm 4 --no-refine
```

### 3. Old photos with NO sidecars — `gps_mosaic.py` (GPS-only fallback)

For folders captured **before** sidecars existed (only the
`"<lat> , <lon>.jpg"` filename is available — no altitude, heading, or
gimbal data), use the standalone single-file tool. It needs only
`numpy` + `opencv-python`, so it runs on any laptop without building the
workspace:

```bash
cd ros2_ws/src/ortho_mapping/ortho_mapping
python3 gps_mosaic.py --dir /path/to/old_photos --altitude 30
```

- **Scale** comes from `--altitude` (your mapping flight AGL) + the A8 mini
  FOV. This is the reliable path: bland aerial terrain (bare dirt, furrows,
  tire tracks) defeats feature-based auto-scaling, so **set `--altitude` to
  the real flight height**.
- **Heading** defaults to north-up (nadir gimbal, drone flying north-up or
  gimbal north-locked). If the whole map looks rotated/smeared, pass
  `--heading <deg>` (CW from north).
- **`--auto`** (opt-in) tries to recover scale *and* heading from image
  overlap by matching neighbouring photos against their GPS offsets. It
  self-guards: if the matches disagree (bland terrain) it falls back to
  `--altitude`. Worth trying on textured scenes; ignore on bare ground.
- **`--blend center`** (default) makes each output pixel come from the single
  most-centred photo → **sharp** map with visible tile seams. `--blend
  feather` averages overlaps → smooth but blurs where GPS scatter misaligns
  frames.

Output is the same trio: `<name>.jpg` + `.jgw` world file + `_report.json`
(in `<dir>/../mapping_output/`). Verified on the 30-image
`ros2_ws/src/mapping/mappingImages/` set: 85% coverage, ~1.3 cm/px source,
readable field detail (tractor-till pattern, plastic-sheet rows, dirt road).

**When to use which:** new flights → the ROS node (§1) reads sidecars
automatically. Old sidecar-less folders → `offline_map` (§2) still works if
you accept its GPS-track heading, but `gps_mosaic.py` (§3) is purpose-built
for the no-metadata case and gives you the altitude/heading/blend knobs that
matter when there's nothing but GPS to go on.

Useful parameters (same names as the ROS parameters):

| Parameter | Default | Meaning |
|---|---|---|
| `gsd_cm` | 3.0 | output resolution, cm of ground per pixel |
| `default_altitude_agl` | 30.0 | AGL (m) assumed for images with **no sidecar** — set this to the real mapping altitude when running on old folders |
| `heading_source` | `auto` | `auto` = sidecar compass, else GPS-track bearing; also `sidecar` / `track` / `fixed` |
| `yaw_offset_deg` | 0.0 | mounting correction if the whole map looks rotated |
| `refine` | true | ECC seam refinement, bounded to `max_refine_shift_m` (3 m) |
| `max_canvas_mp` | 120 | GSD auto-coarsens rather than exceed this canvas size |

---

## How it works (and what was found while researching)

### Capture side (existing, `video_cam`)
`siyi_node` saves each triggered capture as `"<lat> , <lon>.jpg"` into
`mapping_photos/` (GPS snapshotted at shutter time). That filename gives
position only — a map also needs **altitude, heading, and camera attitude**
per photo. `siyi_node` therefore now writes a JSON **sidecar**
(`"<lat> , <lon>.json"`) next to every image:

```json
{"version": 1, "timestamp_unix": 1720900000.1,
 "latitude": 38.315386, "longitude": -76.550875,
 "rel_alt_m": 30.2, "compass_hdg_deg": 274.3,
 "gimbal": {"yaw_deg": 0.1, "pitch_deg": -89.8, "roll_deg": 0.2},
 "resolution": "4K", "rotate_180_applied": true}
```

Sources: `/mavros/global_position/global` (lat/lon),
`/mavros/global_position/rel_alt` (AGL), `/mavros/global_position/compass_hdg`
(heading, new subscription), and the SIYI SDK gimbal-attitude request (read
under the camera control lock right before the shutter). Sidecar writing is
best-effort — it can never fail a capture — and `new_od`'s watcher ignores
non-image extensions, so detection is unaffected. Disable with
`record_metadata:=false`.

**Old folders without sidecars still map**: position comes from the filename,
altitude from `default_altitude_agl`, and heading from the **GPS track**
(bearing from each photo to the next — valid because the camera flies
top-forward on lawnmower legs).

### Mapping approach: GPS direct georeferencing (+ bounded visual refinement)

Approaches considered:

1. **Feature-only stitching** (the old `mapping` package): weak features on
   grass/asphalt and drift accumulation over 50+ nadir images; no
   georeference. Rejected.
2. **Full photogrammetry** (OpenDroneMap `--fast-orthophoto` / WebODM): best
   quality but heavy (SfM over all frames, Docker, tens of minutes on an
   Orin Nano sharing RAM with YOLO). Kept as an optional *ground-station*
   post-process — the same folder + sidecars feeds it directly.
3. **GPS direct georeferencing** (chosen): each image's 4 corners are
   ray-cast from the camera pose (lat/lon/alt + heading + gimbal attitude,
   SIYI A8 mini intrinsics: 3840x2160, 81° HFOV) onto the ground plane, then
   the image is perspective-warped onto a metric north-up canvas and
   feather-blended. O(1) per image, deterministic, never diverges, output is
   inherently georeferenced. Accuracy = GPS/compass accuracy (~2–5 m
   absolute).
4. **Hybrid refinement** (also implemented, `refine`, default on): before
   blending, a translation-only ECC alignment against the already-painted
   overlap corrects GPS jitter, but is *bounded to ±3 m* — errors cannot
   accumulate because every image stays anchored to GPS.

Pipeline (`mapper.build_map`):
1. scan folder → pose per image (sidecar > filename fallback), sorted by time
2. assign headings (sidecar compass / GPS-track bearing)
3. ray-cast footprints → size a canvas (auto-coarsening GSD above the MP cap)
4. warp + feather-blend each image (pre-shrunk to canvas scale for speed),
   optional ECC refinement
5. crop to painted area → JPEG + `.jgw` world file + JSON report

Everything runs on packages already on the Jetson (`numpy 1.26`,
`opencv 4.10`) — **zero new dependencies**. Expected runtime for ~100 4K
photos: on the order of 1–2 minutes on the Orin Nano.

### Flight-plan notes (coverage is a flight problem, not a software problem)
- The capture pipeline sustains ~1 photo / 2–3 s. At 30 m AGL the along-track
  footprint is ~29 m, so keep ground speed ≲ 10 m/s on mapping legs.
- Side-lap: leg spacing ≤ ~35 m at 30 m AGL (footprint width ~51 m).
- The report's `coverage_of_bounding_box` shows gaps immediately after the run.
- If the finished map looks uniformly rotated, set `yaw_offset_deg`.
- `verify`: run `pytest tests/unit/test_ortho_*.py` from the repo root
  (42 tests — geometry, metadata, and full synthetic-mission runs).
