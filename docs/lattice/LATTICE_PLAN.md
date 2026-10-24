<!--
docs/lattice/LATTICE_PLAN.md -- PURPOSE: the engineering plan for integrating Anduril Lattice (the Lattice SDK)
into the ASTRA codebase: what we publish, where the bridge runs, the package design, how we test it in the
Gazebo twin without a Lattice account, the phases with "done when" checks, and the open decisions. Based on the
seven Lattice documentation pages saved in the repo root (*.mhtml, added in cfc8d7c) and docs.anduril.com.
Phase 1 (the core library, package ros2_ws/src/lattice_bridge) was built on 2026-10-24; the other phases are
not built yet. Update this file as phases finish.
-->

# Lattice integration plan

Status: **plan, 2026-10-24; phase 1 (core library) built and tested the same day**. Owner to confirm the open
decisions in section 9 before Phase 2. The exact data contract as built is in [`DATA_CONTRACT.md`](DATA_CONTRACT.md).

## 1. What Lattice is, in our terms

Lattice is Anduril's common operating picture (COP) and tasking platform. Integrations talk to it through
four APIs (REST and gRPC; SDK v2+ clients are generated from the REST endpoints):

| Lattice API | What it is | What ASTRA would use it for |
|---|---|---|
| **Entities** | Everything on the map is an *entity*: a bag of components (`entityId`, `isLive`, `expiryTime` < 30 days, `provenance.integrationName` + `sourceUpdateTime`, `aliases.name`, `ontology.template`, `location`, `milView`, ...). Three shapes: **asset** (ours, controllable), **track** (seen by a sensor), **geo-entity** (a shape on the map). Updates only register when `provenance.sourceUpdateTime` changes. | our drone = asset; hiker and tent detections = tracks; search/flight boundaries and drop points = geo-entities |
| **Tasks** | Sequential commands to a *taskable agent*. An asset advertises what it accepts in `taskCatalog.taskDefinitions` (`type.googleapis.com/...`, e.g. `anduril.tasks.v2.VisualId`); lifecycle `STATUS_SENT` -> updates -> `STATUS_DONE_OK` / `STATUS_DONE_NOT_OK`. | later, optional: let an operator task the drone ("investigate here") with a human-confirm gate |
| **Objects** | Files (< 1 GiB) on Lattice's CDN, `path` + SHA-256 checksum + optional TTL; **REST only**; linked to an entity's media component (track thumbnails). | the photo crop that shows each detected target |
| **Video** | Live feeds via SRT ingress streams. | optional: the SIYI RTSP feed (8554, unused today) re-streamed via MediaMTX, like Anduril's Raspberry Pi camera sample |

Closest Anduril samples to copy patterns from: **Raspberry Pi camera** (1 Hz asset publish + video ingress +
health + custom tasks, Python gRPC), **Task an asset** (custom task proto in the Schema Registry), **Entity
thumbnail** (Objects + entity media), **AIS integration** (periodic track updates, gRPC and REST variants).

## 2. Why, and the honest unknown

No public SUAS 2026/2027 rule found (search on 2026-10-24) requires Lattice; the saved docs came with Areg's
commit `cfc8d7c` ("new docker, lattice docs"). So the design assumes Lattice is a **team capability that must
never put the mission at risk**: if Lattice is down, slow or unreachable, the drone flies exactly as today.
If a rule or sponsor requirement turns up, only the publish set (section 3) changes, not the architecture.

## 3. What we publish (data contract)

| Lattice entity | Shape / key components | Source in our stack | Rate / lifetime |
|---|---|---|---|
| `astra-<mission>-uav`: our drone | asset: `ontology {TEMPLATE_ASSET, platformType "UAV"}`, `milView {DISPOSITION_FRIENDLY, ENVIRONMENT_AIR}`, `location.position` (lat, lon, `altitudeHaeMeters` from AMSL + geoid, `altitudeAglMeters`), heading, `health` (battery, link) + `status.platformActivity` (flight mode) | `/mavros/global_position/global`, `rel_alt`, `compass_hdg`, `/mavros/state`, battery | 1 Hz; rolling `expiryTime` now + 2 min (disappears soon after the link dies); `isLive false` on shutdown |
| `astra-<mission>-tgt-person-<n>`, `astra-<mission>-tgt-tent-<n>`: targets | track: `ontology {TEMPLATE_TRACK, platformType "Person"/"Tent"}` (confirm names in the Registry), `milView {DISPOSITION_NEUTRAL or UNKNOWN}`, `location` = the **projected target position** (`main/src/target_geolocation.py`), confidence, class, photo time | `/image_detection` + geolocation, or main_controller's chosen targets | on change only; stable IDs from position clustering (same target = same ID); expiry end of mission + margin |
| `astra-<mission>-area-<name>` (search areas, flight boundary) | geo-entity polygons (`TEMPLATE_GEO`, `GEO_TYPE_GENERAL` / `GEO_TYPE_CONTROL_AREA`) | mission/world config (`sim_worlds/config/*.yaml`) | once per mission; long expiry (avoid `noExpiry` except truly static areas) |
| `astra-<mission>-drop-<person\|tent>-<n>` | geo point at the release position | main_controller drop event | on drop |
| thumbnail objects | `astra/<mission>/<entityId>.jpg` (crop of the best photo, TTL = mission), linked from the track's media component | photo + bbox | once per target, re-upload only when a better photo arrives |

Rules of the contract: `provenance.integrationName = "astra_odlc"`, `dataType` per publisher, `sourceUpdateTime`
= the **measurement** time in UTC (photo time for targets, fix time for the drone), never "now" for old data;
handle missing components (the docs require tolerating partial entities); IDs never reused across missions
(prefix with a mission id).

## 4. Where the bridge runs (the key architecture decision)

In flight the Jetson's only link to the ground is the **MAVLink telemetry radio** (Mission Planner); Lattice
sandboxes are in the cloud. So:

```
 Jetson (flight-critical, unchanged)                   Ground laptop (internet)            Lattice
 main_controller / new_od --> "astra report" ----MAVLink radio----> lattice_bridge ---gRPC/REST---> Entities
 MAVROS telemetry ------------------------------------------------> (MAVProxy out)                  Objects
```

- **Option A (recommended default): bridge on the ground laptop.** Telemetry comes from a MAVProxy/Mission
  Planner UDP output (the simulator already fans MAVLink out the same way). Target reports cross the radio as
  compact MAVLink messages the Jetson sends: one `STATUSTEXT` per target in a fixed machine-readable form
  (`TGT p 36.214935 -96.006203 95 12`, 43 characters at most, under the 50-char limit; last number = a counter) or `NAMED_VALUE_FLOAT` triplets. Photos/thumbnails
  are uploaded after landing (or over an IP radio if we ever add one).
- **Option B: bridge on the Jetson**, only if the drone gets an IP link to the internet or to a local Lattice
  node. Same package, different source adapter (ROS topics directly).
- Either way the bridge is a **separate process** with bounded queues, never called from flight nodes, no
  blocking in ROS callbacks, and its failure changes nothing in flight.

## 5. Package design (fits our plug-and-play style)

New ament_python package `ros2_ws/src/lattice_bridge` (team style: rclpy, purpose header in every file):

| Module | Job |
|---|---|
| `entities.py` | **pure functions** that build entity dicts from plain inputs (`build_uav_asset`, `build_target_track`, `build_area_geo`, `build_drop_point`): no I/O, fully unit-testable |
| `ids.py`, `clustering.py` | stable entity IDs; merge repeat detections of the same target (distance threshold, best confidence wins) |
| `sources/` | `ros_source.py` (topics on the same machine), `mavlink_source.py` (pymavlink on the ground: telemetry + `TGT` reports), `replay_source.py` (a recorded run folder from `~/.astra_sim/runs/`) |
| `sinks/` | `grpc_sink.py` (Lattice SDK, recommended for telemetry), `rest_sink.py` (Objects uploads, which are REST-only), `jsonl_sink.py` (records every request: offline mode and test oracle), `mock_sink.py` |
| `publisher.py` | rate limiting, de-duplication (only publish when content or `sourceUpdateTime` changes), retry with exponential backoff + jitter, an offline buffer that flushes newest-first on reconnect (per "Retry connections" / "Connect offline") |
| `bridge_node.py` | ROS 2 node + CLI (`ros2 run lattice_bridge bridge --source mavlink --sink grpc`), params from a YAML profile |
| `report_codec.py` (shared with the Jetson side) | encode/decode the `TGT` radio message, with tests |
| built in phase 1 besides the above | `constants.py` (every word and number in one place), `timeutil.py` (UTC rules), `geo.py` (number checks, polygons, heading), `world_areas.py` (simulator world files -> area corners), `sinks/__init__.py` (the `Sink` protocol) |

Jetson side: a tiny addition in main_controller (or a separate `lattice_reporter` node listening to its
decisions) that emits the `TGT` message when it accepts a target; behind a parameter, default **off**, so flight
behaviour is unchanged unless enabled.

Secrets: endpoint and tokens from environment variables or a file outside git (`LATTICE_ENDPOINT`,
`LATTICE_TOKEN`, sandbox header token `SANDBOXES_TOKEN`; newer SDK OAuth `client_id`/`client_secret`). Add the
file pattern to `.gitignore`; never commit them (public repo). Pin the SDK version in
`docker/requirements-*.txt` after confirming the package name (`anduril-lattice-sdk` per the current GitHub
README; an older page says `lattice-sdk-python`).

## 6. Testing without a Lattice account (the simulator is the advantage)

1. **Contract tests** (unit): every builder output has the required components, `expiryTime` < 30 days,
   valid enums, UTC timestamps, monotonic `sourceUpdateTime`; golden JSON files reviewed against the docs.
2. **Mock Lattice** (`lattice_bridge/test/mock_lattice.py`): a local HTTP server implementing `PUT
   /api/v1/entities`, object upload/get, and auth-header checks, plus a page listing entities on a map. Lets
   teammates develop with no token.
3. **Gazebo end-to-end**: a plug-and-play profile `simulation/profiles/lattice_mock.yaml` (an `extras` slot
   that starts the bridge with `--source mavlink --sink mock` on the sim's MAVLink output). Done when one
   headless demo shows: the UAV asset updates at ~1 Hz all flight, exactly one track per real target,
   track position within 2 m of the truth file, both search areas and both drop points present, nothing
   published after `isLive false`; and the demo's score is identical with the bridge on and off.
4. **Fault tests**: kill the mock mid-flight, delay it 5 s, return 500s: the flight result is unchanged and the
   bridge catches up from its buffer.
5. **Sandbox**: the same profile with `--sink grpc` against a Lattice Developer Program sandbox; verify in the
   Lattice UI Entity Explorer (search `aliases.name`).

## 7. Phases

| Phase | Work | Done when |
|---|---|---|
| 0. Decide + access | confirm why (rule / sponsor / team goal) with Areg; request Lattice Developer Program sandbox access; decide Option A or B; confirm SDK package + version; check the Registry names for `platformType` values | written answers in section 9 |
| 1. Core library | `entities.py`, `ids.py`, `clustering.py`, `report_codec.py`, `jsonl_sink`, contract tests | unit tests green; golden files reviewed. **DONE 2026-10-24**: package `lattice_bridge`, about 1,100 tests (also from a clean `colcon build`), 11 golden files reviewed against the live docs; findings in `DATA_CONTRACT.md` |
| 2. Mock + sim | mock Lattice, `mavlink_source`, `bridge_node`, `lattice_mock.yaml` profile, Jetson `TGT` reporter behind a default-off parameter | section 6 steps 3-4 pass in the twin |
| 3. Sandbox | `grpc_sink` with retries/offline buffer, secrets handling, a replay of a recorded sim run into the sandbox | entities visible and correct in the Lattice UI |
| 4. Thumbnails + areas | `rest_sink` Objects upload + media link; geo-entities from the mission config | thumbnails show in Lattice; areas drawn correctly |
| 5. Video (optional) | MediaMTX re-streams the SIYI RTSP feed to a Lattice SRT ingress (ground side or Jetson side, bandwidth permitting) | live feed visible next to the UAV asset |
| 6. Tasking (optional, gated) | advertise a custom `Investigate` task in `taskCatalog`; `ListenAsAgent`; turn an accepted task into a **suggested** waypoint that a human confirms in the GCS before anything moves; report task status | operator task -> confirmed waypoint -> `STATUS_DONE_OK` in the twin; impossible to move the drone without the confirm |
| 7. Field | network plan at the range (ground laptop internet), behaviour with no internet (record + replay later), checklist, docs | dry run on the real drone in a tethered/bench test |

Docs to add alongside: `docs/lattice/README.md` (setup, profiles, troubleshooting), a section in `AGENTS.md`,
and the 7 saved `*.mhtml` pages moved to `docs/lattice/reference/` (with Areg's OK; they are his files).

### Phase 8: several ground stations in different places (requested 2026-10-24; starts AFTER the ArduPilot / Mission Planner demo)

Goal: more than one ground control station (GCS), at different locations, can follow and work with the same
drone, with Lattice as the shared picture that connects them. Phases 1-2 must exist first (the bridge, the
entity builders, the fake Lattice); this phase adds the multi-site layer on top.

**Who talks to what (design to confirm with the owner and Areg):**

| Role | What it is | How it connects |
|---|---|---|
| Radio GCS site | a laptop with the telemetry radio (or a link to it) that actually hears the drone; runs Mission Planner and the `lattice_bridge` | MAVLink from the radio; Lattice over the internet |
| Remote GCS site | a laptop or screen at another place, with no radio | Lattice only: sees the drone, targets and areas; may be allowed to send tasks (phase 6 rules) |
| Drone (Jetson) | unchanged | MAVLink radio only; never talks to Lattice in flight |

**Design points (each becomes a test):**
1. **One drone, one entity.** Several radio sites may hear the same telemetry. Each publishes with its own
   `provenance.integrationName` (`astra_odlc.<site>`) and every site also publishes a **ground-station asset**
   (`astra-gcs-<site>`: location, link quality, last-heard time). A small rule decides which site's data is the
   *active* source for the drone entity (best link, with hysteresis so it does not flip back and forth), so Lattice
   never shows two drones or a jumping one. Merge by measurement time, not arrival time.
2. **Hand-over.** When the active radio site loses the drone, another site takes over within a set time; the
   drone entity shows the switch (a visible "heard by" field or a short log), and nothing is lost: the sites keep
   their offline buffers and flush newest-first when the internet returns.
3. **Who is in command.** MAVLink lets several GCS programs connect, but only one person should arm, change
   mode or upload a mission. Lattice tasks need an explicit "in command" state (one site holds it; changes are
   deliberate and logged) and every flight-affecting task still ends in a human confirm in the GCS (phase 6).
   Remote sites are **view-only by default**.
4. **Identity and secrets.** One Lattice credential per site, kept outside git, revocable per site; the bridge
   logs which site did what.
5. **Link realism.** Radio links drop, lag and have low bandwidth; the internet between sites does too.
   The simulator gets a **link emulator** (a small proxy between MAVProxy and each site's bridge that adds
   delay, loss, bandwidth limits and outages per site), so hand-over is tested before any field day.

**Simulator test (done when all pass, with the flight unaffected):** three simulated sites on different MAVProxy
outputs (for example *Tent A* near the field, *Tower B* on the far side with a weaker link, *Remote C* with
Lattice only); fly the demo; (a) Lattice shows exactly one drone entity all flight; (b) cut Tent A's link
mid-flight and Tower B takes over within the set time; (c) restore Tent A and the data stays consistent, no
duplicate targets; (d) Remote C sees everything but a task from it is refused unless it holds command;
(e) the mission score is the same with the multi-site layer on or off.

**Order of work:** finish the ArduPilot / Mission Planner demo and the gimbal fix first; let the phase 1-2 build
finish; then design review of this section with the owner (open decisions 6-8 below), then build.

## 8. Risks

| Risk | Mitigation |
|---|---|
| Lattice failure affects flight | separate process, default-off reporter, no shared callbacks, fault tests in the twin |
| Radio bandwidth / 50-char STATUSTEXT | compact `TGT` format at most once per target change; telemetry already flows |
| Stale or duplicated targets | clustering + stable IDs + measurement-time `sourceUpdateTime` |
| Token leaks in a public repo | env vars / ignored file, pre-commit grep, never in logs |
| SDK breaking changes (v1 -> v2 moved to REST-generated clients) | pin versions; our builders produce plain dicts so only the sink touches the SDK |
| No internet at the range | `jsonl_sink` records everything; replay into Lattice afterwards |
| Wrong ontology names | confirm in the Schema Registry before Phase 3 |

## 9. Open decisions (owner / Areg)

1. Why Lattice: competition rule, sponsor ask, or team goal? (Changes the publish set and priority.)
2. Do we have, or will we request, Lattice Developer Program sandbox access? Who holds the tokens?
3. Option A (ground bridge, recommended) or B (Jetson bridge, needs an IP link in flight)?
4. Is tasking (Phase 6) wanted at all, given it touches flight control? Recommendation: only with a human
   confirm in the GCS, and only after Phases 1-4.
5. Video: worth the radio/Jetson budget, or skip?
6. (Phase 8) Which ground stations, and where? Names and places of each site, which ones have a telemetry radio
   and which are remote (Lattice only), and who runs each laptop.
7. (Phase 8) Who may be "in command" (arm, change mode, upload a mission)? Recommendation: one named site at a time,
   changes logged, remote sites view-only unless explicitly given command.
8. (Phase 8) What internet does each site have at the range, and what is the fallback when it drops?
   Recommendation: every site records locally and catches up afterwards.
