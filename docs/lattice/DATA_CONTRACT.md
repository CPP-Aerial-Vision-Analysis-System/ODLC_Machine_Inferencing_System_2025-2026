<!--
docs/lattice/DATA_CONTRACT.md -- PURPOSE: the exact contract between ASTRA and Anduril Lattice, written for
students and reviewers: which entities we publish, every field of each one (with where the value comes from and
why), the enum words, the altitude and time rules, the TGT radio message, the target-merging rules, how the saved
Lattice pages and the live documentation differ, and the points that still need confirming. The code is in
ros2_ws/src/lattice_bridge (constants.py is the single source of the words); the complete example requests are
its test/golden/*.json files. Checked against the live Lattice documentation on 2026-10-24.
-->

# Lattice data contract (what ASTRA publishes)

Plain-words summary: we send Lattice five kinds of small JSON messages. Each message describes one **entity**
(one thing on the shared map). This page says exactly what is in each message and why. If you change what we
send, change `constants.py` / `entities.py`, regenerate the golden files, and update this page.

## 1. Where the rules come from

| Source | What we took from it |
|---|---|
| Live REST reference "PublishEntity" (`https://developer.anduril.com/reference/rest/entities/publish-entity`) | the 39 top-level keys, every enum word, the shape of `location`, `health`, `status`, `tracked`, `geoShape`, `media`; "expiry must be in the future, less than 30 days"; "`sourceUpdateTime` must change with every change or the update is discarded" |
| Live guide "Publish entities" (`.../guides/entities/publish`) | the drone example (health components, `velocityEnu`, `attitudeEnu`), "Lattice sets `connectionStatus`; offline after one minute without updates" |
| Saved pages in the repo root (Entities overview, Quickstart) | the three entity shapes and their required components, the example JSON, the `PUT https://$LATTICE_ENDPOINT/api/v1/entities` call with `authorization: Bearer $ENVIRONMENT_TOKEN` and `anduril-sandbox-authorization: Bearer $SANDBOXES_TOKEN`; success is HTTP 200 with `{}` |

**Where the saved pages and the live documentation differ (live wins):**

| Topic | Saved page | Live docs | What we do |
|---|---|---|---|
| Address of the documentation | `docs.anduril.com` | `docs.anduril.com` now redirects (HTTP 301) to `developer.anduril.com` | use `developer.anduril.com` |
| Key spelling of `platformType` | Entities overview shows `"platform_type"` in its ontology examples | Quickstart and REST reference use `platformType`; the REST JSON is camelCase everywhere | camelCase everywhere; `validate_entity` rejects snake_case keys |
| Health, status, quality | not on the saved pages | `health` (assets only), `status.platformActivity`, `tracked.trackQualityWrapper`, `location.attitudeEnu` exist | used for battery / link / mode / heading / confidence (section 3) |
| Confidence | not mentioned | the entity model has **no** confidence field | description text + track quality 0-15 |
| `platformType` values | free examples (`UAV`, `Radar`, `Submarine`, `Airplane`, `Animal`) | "high-level type in natural language", no list; the Schema Registry page does not mention ontology values | `Person` / `Tent` kept in one place, **to be confirmed** |

## 2. Rules that hold for every entity

| Rule | How it is enforced |
|---|---|
| Required parts: `entityId`, `isLive`, `expiryTime`, `provenance.integrationName`, `aliases.name` (+ `provenance.dataType`, `provenance.sourceUpdateTime`, which every Lattice example sets) | builders always set them; `validate_entity` checks them |
| `provenance.integrationName` is always `astra_odlc`; `dataType` is `uav_telemetry`, `odlc_detection`, `mission_area` or `payload_drop` | `constants.py` |
| Times are UTC text with a Z: `2026-10-24T15:30:00.000000Z` | `timeutil.format_utc`; naive datetimes raise `ValueError` |
| `sourceUpdateTime` is the time of the **measurement** (GPS fix, photo, release, definition of the area), never "now" for old data | each builder's time argument |
| `expiryTime` is in the future and under 30 days (we allow at most 29, leaving room for clock differences); `noExpiry` is never used | `_check_times`; `validate_entity` |
| Missing optional inputs leave the part out; nothing is invented | `None` means "leave out" |
| Bad numbers (NaN, infinity, out of range, `(0, 0)` = no GPS fix) raise `ValueError` | `geo.require_*` |
| An update that should delete an entity is `isLive: false` with a **new** `sourceUpdateTime` (Lattice ignores an update whose time did not change) | `is_live=False` |
| IDs are `astra-<mission>-...` and the mission id contains the start time to the second, so a later flight never overwrites an earlier one | `ids.py` |
| Secrets (`LATTICE_ENDPOINT`, `LATTICE_TOKEN`, `SANDBOXES_TOKEN`) come only from environment variables or a file outside git, never into entities, logs or recordings | phase 2/3; `test_hygiene.py` scans the repo |

## 3. The five kinds of message

### 3.1 Our drone (asset)

```json
{
  "entityId": "astra-runway20261024t153000z-uav",
  "description": "ASTRA UAV telemetry (MAVLink)",
  "isLive": true,
  "expiryTime": "2026-10-24T15:32:00.000000Z",
  "aliases": {"name": "ASTRA UAV"},
  "provenance": {"integrationName": "astra_odlc", "dataType": "uav_telemetry",
                 "sourceUpdateTime": "2026-10-24T15:29:59.000000Z"},
  "ontology": {"template": "TEMPLATE_ASSET", "platformType": "UAV"},
  "location": {"position": {"latitudeDegrees": 36.2143, "longitudeDegrees": -96.0068,
                            "altitudeHaeMeters": 230.0, "altitudeAglMeters": 49.0},
               "attitudeEnu": {"x": 0.0, "y": 0.0, "z": 0.382683, "w": 0.92388}},
  "milView": {"disposition": "DISPOSITION_FRIENDLY", "environment": "ENVIRONMENT_AIR"},
  "health": {"healthStatus": "HEALTH_STATUS_HEALTHY", "components": ["... see uav_full.json ..."]},
  "status": {"platformActivity": "AUTO"}
}
```

(shortened; the full file is `ros2_ws/src/lattice_bridge/test/golden/uav_full.json`)

| Field | Value and source | Why |
|---|---|---|
| `entityId` | `astra-<mission>-uav` | one drone entity per flight |
| `expiryTime` | publish time + 120 s, pushed forward by every update | the drone disappears about 2 minutes after the link dies |
| `sourceUpdateTime` | the **fix** time of the GPS position | Lattice wants the measurement time |
| `ontology` | `TEMPLATE_ASSET`, `platformType` `UAV` | the saved Entities page uses `UAV` for a drone |
| `milView` | `DISPOSITION_FRIENDLY`, `ENVIRONMENT_AIR` | ours, in the air |
| `location.position.latitudeDegrees/longitudeDegrees` | `/mavros/global_position/global` (phase 2: MAVLink `GLOBAL_POSITION_INT`) | |
| `altitudeHaeMeters` | **AMSL + geoid undulation N** (section 4) | Lattice wants height above the ellipsoid |
| `altitudeAglMeters` | MAVLink `relative_alt` = height above **home** | not terrain height; approximate on slopes |
| `location.attitudeEnu` | compass heading -> ENU quaternion (north 0.7071/0.7071, east 0/1) | the heading arrow |
| `location.velocityEnu`, `speedMps` | optional; MAVLink gives north, east, down, we convert to east, north, up | |
| `health.components` | `battery-0` (warn below 30 %, fail below 15 %), `telemetry-link` (warn after 3 s without telemetry, fail after 10 s) | the guide's drone example uses exactly this shape |
| `health.healthStatus` | the worst component | "typically a roll-up of component health" |
| `health.connectionStatus` | **not set** | "Lattice sets it online when updates arrive and offline after one minute without" |
| `status.platformActivity` | flight mode, e.g. `AUTO`, `GUIDED (disarmed)`, `DISARMED` | human-readable activity; no component exists for "mode" |
| goodbye | `isLive: false`, new time | deletes the entity at shutdown (`uav_goodbye.json`) |

### 3.2 A detected hiker or tent (track)

| Field | Value and source | Why |
|---|---|---|
| `entityId` | `astra-<mission>-tgt-<person\|tent>-<n>`, `n` counts from 1 per kind | stable: the same physical target keeps its id |
| `ontology` | `TEMPLATE_TRACK`, `platformType` `Person` or `Tent` | **to be confirmed** in the Schema Registry |
| `milView` | `DISPOSITION_NEUTRAL`, `ENVIRONMENT_SURFACE` | nobody's enemy, on the ground (`ENVIRONMENT_LAND` also exists; one line in `constants.py`) |
| `location.position` | the **projected target position** (`main/src/target_geolocation.py`), never the drone's; no altitude unless known | |
| `description` | `Person detected by ASTRA ODLC: confidence 95%, 7 sighting(s)` | the exact confidence and the number of photos |
| `tracked.trackQualityWrapper` | `round(confidence * 15)`, 0-15 | the closest supported number; an **approximation** |
| `tracked.lastMeasurementTime`, `provenance.sourceUpdateTime` | measurement (photo) time | |
| `createdTime` | time of the first sighting | "when the entity was first known to the producer" |
| `expiryTime` | planned mission end + 6 h (recommended), at most 29 days | stays on the map for the debrief |
| merged away | `isLive: false` with the merging photo's time | `state_track_kwargs(state, measured_time=...)` |

### 3.3 Search areas and the flight boundary (geo-entity polygon)

| Field | Value and source | Why |
|---|---|---|
| `entityId` | `astra-<mission>-area-<name with - instead of _>` | |
| `ontology` | `TEMPLATE_GEO` (no `milView`, no `location`) | the saved page: geo-entities need `geoDetails` and `geoShape` |
| `geoDetails.type` | search areas `GEO_TYPE_GENERAL`; flight boundary `GEO_TYPE_CONTROL_AREA` | |
| `geoShape.polygon.rings[0].positions[].position` | corners in latitude/longitude; the ring is **closed** and **counter-clockwise** | the reference: "first and last point must be the same", "only canonical representations accepted" (RFC 7946 rule: counter-clockwise outer ring) |
| corners | `config/<world>.yaml` `search_areas` (local metres, converted with ArduPilot's own formula) and `truth/<world>.yaml` `flight_boundary` | `world_areas.py`; matches the generator's own corner lists to a few millimetres |
| `expiryTime` | publish time + 12 h; no `noExpiry` | |
| `description` | the area's note; for the boundary `flight boundary: fly 45.7-121.9 m above ground` | the SUAS 150-400 ft rule |

### 3.4 Drop points (geo-entity point)

`TEMPLATE_GEO`, `GEO_TYPE_GENERAL`, `geoShape.point.position` = the drone's position at the release (that is
what `main_controller` knows, not where the payload came to rest), optional `altitudeHaeMeters`,
`sourceUpdateTime` = release time, id `astra-<mission>-drop-<person|tent>-<n>`.

## 4. Altitude, the part that is easy to get wrong

* The MAVLink message `GLOBAL_POSITION_INT.alt` (what the ground bridge will read from MAVProxy in phase 2)
  is height above **mean sea level (AMSL)**. Be careful with MAVROS: its ROS topic
  `/mavros/global_position/global` is documented to convert that value to ellipsoid height itself (it applies
  its own geoid correction). **This must be verified on the bench before phase 2 chooses its source.** The
  builder's `alt_amsl_m` must only ever be fed true AMSL; if a source already delivers ellipsoid height, pass
  `geoid_undulation_m=0.0` explicitly so nothing is corrected twice.
* Lattice wants height above the **WGS84 ellipsoid (HAE)**. HAE = AMSL + N, where N is the geoid undulation
  at that place. In Oklahoma N is roughly -30 m (look the exact value up with an EGM96 / EGM2008 calculator
  for the field; do not copy this number blindly).
* N is a setting: `geoid_undulation_m` (default `None` = "not configured"). With `None` the AMSL value is
  published as HAE, the drone's description says `altitude: AMSL used as HAE (geoid undulation not
  configured)`, and `altitude_is_approximate(None)` is `True`, and the bridge calls `logutil.warn_if_altitude_approximate` at start-up, which logs `ALTITUDE_WARNING` once. An explicit
  `0.0` counts as a decision.
* `altitudeAglMeters` comes from `relative_alt` (above home). It is not the height above the terrain below the
  drone; the real search areas have 9-10 m of relief.

## 5. Time rules

| Rule | Detail |
|---|---|
| Format | UTC, `YYYY-MM-DDTHH:MM:SS.ffffffZ` (the docs' own examples) |
| Zones | aware datetimes of any zone are converted; naive datetimes raise `ValueError` |
| `sourceUpdateTime` | measurement time; must **change** on every update; `time_advanced` in a clustering `Decision` says whether it did |
| Future times | accepted up to 60 s ahead (clock differences), refused beyond |
| `expiryTime` | after `now`, at most 29 days later |
| Drone | `now + 120 s` on every update |

## 6. The `TGT` radio message (Jetson -> ground)

```
TGT <p|t> <latitude> <longitude> <confidence 0-100> <seq>        TGT p 36.214935 -96.006203 95 12
```

| Part | Rule |
|---|---|
| carrier | MAVLink `STATUSTEXT` (severity INFO = 6) through `/mavros/statustext/send`; MAVLink cuts text at 50 characters |
| `TGT` | fixed word; anything else is not ours and is ignored |
| `p` / `t` | person (hiker) / tent |
| latitude, longitude | decimal degrees with exactly 6 decimals (about 11 cm), no leading zeros, no `+`, no `-0.000000`; `(0, 0)` is refused (it means "no GPS fix") |
| confidence | whole percent 0-100 |
| `seq` | counter 0-999999, then 0 again |
| length | at most 43 characters |
| decoder | untrusted input: strict canonical form only, `None` for anything else, never raises, never evaluates or formats the text, trailing NUL padding tolerated; fuzz-tested |

One message per accepted target change; the sending side (phase 2) is off by default.

## 7. How repeated sightings become one entity

Same kind and within 12 m of the target's current position = same target. Reported confidence = the best
seen. Reported position = mean of all sightings weighted by confidence squared. Quiet updates: only when the
position moved 1 m, or confidence rose by 0.02, or two targets merged (the older id survives; the other gets a
goodbye update). At most 64 targets. Details and reasons: `clustering.py` and the package README.
Limitation: a false positive that is replaced later by the real, higher-confidence target 40 m away stays as a
separate track (the Jetson reports "best target" changes, not retractions); phase 2 should decide whether to
retire the old one.

## 8. Enum words we use (all exist in the live reference)

| Where | Words |
|---|---|
| `ontology.template` | `TEMPLATE_ASSET`, `TEMPLATE_TRACK`, `TEMPLATE_GEO` |
| `milView.disposition` | `DISPOSITION_FRIENDLY` (drone), `DISPOSITION_NEUTRAL` (targets) |
| `milView.environment` | `ENVIRONMENT_AIR` (drone), `ENVIRONMENT_SURFACE` (targets) |
| `geoDetails.type` | `GEO_TYPE_GENERAL` (search areas, drops), `GEO_TYPE_CONTROL_AREA` (flight boundary) |
| `health` | `HEALTH_STATUS_HEALTHY`, `HEALTH_STATUS_WARN`, `HEALTH_STATUS_FAIL` |
| `ontology.platformType` | `UAV`, `Person`, `Tent` (free text; Person and Tent to be confirmed) |
| `provenance.integrationName` | `astra_odlc` |

## 9. To confirm before the sandbox phase

| Open point | Why it matters | Who / how |
|---|---|---|
| `Person` and `Tent` as `platformType` | the Schema Registry may define preferred names, and the Lattice UI may pick icons from them | check the Registry in the sandbox |
| `ENVIRONMENT_SURFACE` vs `ENVIRONMENT_LAND` for people and tents | affects symbols and filters | look at the Lattice UI in the sandbox |
| `tracked.trackQualityWrapper` accepted from an integration | it may be reserved for sensor fusion | a test publish; if refused, remove it (one line in `build_target_track`) |
| polygon winding order | the reference does not state one; we follow RFC 7946 | a test publish of `area_*.json` and a look at the map |
| exact geoid undulation N for the field | the altitude on the map is wrong by about 30 m without it | EGM calculator, then set the profile value |
| phase 2: which MAVLink altitude feeds the builder | `GLOBAL_POSITION_INT.alt` is AMSL; MAVROS' NavSatFix altitude is documented to be ellipsoid height (verify on the bench) | decide in the MAVLink source and keep a test |
| description length limit | none is documented; ours are under 300 characters | observe in the sandbox |
