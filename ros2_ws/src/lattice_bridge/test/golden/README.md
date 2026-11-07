<!--
lattice_bridge/test/golden/README.md -- PURPOSE: explain the golden files in this folder (complete example
requests, one per kind of entity we publish), how they were checked by hand against the Lattice documentation,
and how to change them safely. test_golden.py makes the builders reproduce them exactly, so the list and the
review checklist here must stay in step with the files.
-->

# Golden files: exactly what we send to Lattice

Each `*.json` file is the body of one `PUT /api/v1/entities` request, produced by a builder with fixed inputs
(`test/test_golden.py` lists the inputs). The test fails if the builder output differs by even one character of
meaning. When a difference is **intended**, regenerate (`LATTICE_UPDATE_GOLDEN=1 python3 -m pytest -q
test/test_golden.py`) and **read the diff against the Lattice docs before committing**: the diff is the review.

| File | What it shows | Built with |
|---|---|---|
| `uav_full.json` | the drone with every optional part: height (HAE from AMSL + geoid value, AGL), heading, velocity, flight mode, battery and link health | `build_uav_asset(...)` |
| `uav_amsl_as_hae.json` | the drone when **no geoid value is configured**: AMSL is published as HAE and the description says so | `build_uav_asset(...)` without `geoid_undulation_m` |
| `uav_minimal.json` | the drone with only a GPS fix: nothing else is invented | `build_uav_asset(...)` |
| `uav_goodbye.json` | the shutdown update: `isLive` false, a newer time, still a complete entity | `build_uav_asset(..., is_live=False)` |
| `track_person.json` | the hiker: a track at the projected target position, confidence 95 %, 7 sightings | `build_target_track(...)` |
| `track_tent.json` | the tent: same shape, other class and numbers | `build_target_track(...)` |
| `area_runway_search_box.json` | the runway world's search box as a polygon (from `config/runway.yaml`) | `build_area_geo(...)` |
| `area_suas_search_boundary_2.json` | the suas_field_real world's second search boundary (rotated rectangle) | `build_area_geo(...)` |
| `area_suas_flight_boundary.json` | the suas_field_real flight boundary: an 11-corner polygon of type `GEO_TYPE_CONTROL_AREA`, with the 150-400 ft altitude rule in the description | `build_area_geo(..., geo_type=...)` |
| `drop_person.json` | where the person payload was released: a geo point | `build_drop_point(...)` |
| `drop_tent.json` | the tent payload release, with a height | `build_drop_point(...)` |

## Hand review (2026-10-24)

Reviewed by the agent that wrote the code, against (1) the live REST reference
`https://developer.anduril.com/reference/rest/entities/publish-entity` (the old address `docs.anduril.com`
now redirects there), (2) the live guide `.../guides/entities/publish`, and (3) the saved pages "Entities
overview" and "Quickstart" in the repo root. A second person should repeat this check before the sandbox phase.

| Check | Result |
|---|---|
| Every entity has `entityId`, `isLive`, `expiryTime` (future, under 30 days), `provenance.integrationName`, `aliases.name` | yes, all 11 files |
| Assets and tracks have `ontology.template`, `ontology.platformType`, `location.position` (lat/lon) and `milView` (disposition + environment) | yes |
| Geo-entities have `ontology.template = TEMPLATE_GEO`, `geoDetails.type` and exactly one `geoShape`; no `location`, no `milView`, no `noExpiry` | yes |
| Every key is camelCase (`platformType`, not `platform_type`) | yes. The saved Entities page shows `platform_type` in its ontology examples, but the Quickstart and the REST reference use `platformType`, and the REST JSON is camelCase |
| Every enum word appears in the REST reference (`TEMPLATE_*`, `DISPOSITION_*`, `ENVIRONMENT_*`, `GEO_TYPE_*`, `HEALTH_STATUS_*`) | yes |
| Times are UTC text with `Z` (the docs' examples have 6 fraction digits too) | yes |
| Polygon: `geoShape.polygon.rings[].positions[].position.{latitudeDegrees,longitudeDegrees}`, first point = last point, counter-clockwise (RFC 7946, which the reference links) | yes. The reference says "only canonical representations accepted" and does not state a winding order; counter-clockwise is the RFC rule |
| `health`: `healthStatus` + `components[]` with `id`, `name`, `health`, `messages[]{status,message}`; `connectionStatus` left out because Lattice sets it from the update stream | yes (shape taken from the reference and the guide's drone example "Battery at 87% charge.") |
| `status.platformActivity`, `location.attitudeEnu` (x,y,z,w; ENU), `location.velocityEnu` (e,n,u), `location.speedMps`, `tracked.trackQualityWrapper` (0-15), `tracked.lastMeasurementTime` | all exist in the reference with these names |
| Heading quaternion: heading 45 gives z 0.382683, w 0.92388 | matches the guide's example (`z` 0.3827, `w` 0.9239 for velocity 50 east + 50 north) |
| Open points (not errors, but unconfirmed): `Person` / `Tent` platform types (Schema Registry), `ENVIRONMENT_SURFACE` vs `ENVIRONMENT_LAND` for people, whether a sandbox accepts `trackQualityWrapper` from an integration | listed in `docs/lattice/DATA_CONTRACT.md`, section 9 |
