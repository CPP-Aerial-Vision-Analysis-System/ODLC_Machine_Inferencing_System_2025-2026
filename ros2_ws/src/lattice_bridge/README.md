<!--
lattice_bridge/README.md -- PURPOSE: the guide to the lattice_bridge package for students and teammates: what
Anduril Lattice is in plain words, what this package does (phase 1: the core library), the whole public
interface in one table, the rules that keep the flight safe, copy-paste examples (they are run by the tests),
how to run the tests, and what is NOT built yet. The engineering plan is docs/lattice/LATTICE_PLAN.md and the
exact JSON we send is in docs/lattice/DATA_CONTRACT.md.
-->

# lattice_bridge: showing the ASTRA mission in Anduril Lattice

**Status: phases 1 and 2 of 7 are built and tested; phase 2 (bridge, pretend Lattice, Jetson-side reporter) ran end
to end in the Gazebo twin on 2026-10-31 ([`docs/lattice/BRIDGE.md`](../../../docs/lattice/BRIDGE.md)). Nothing talks
to a real Lattice yet, nothing is started by `launcher.sh`, and no flight node knows this package exists.**

## 1. What is this, in plain words

**Anduril Lattice** is a shared map. A team (or a judge, or an operator) opens it in a web browser and sees
everything that matters on one screen: our drone moving, the hiker and the tent when the camera finds them,
the search areas, the places where the payloads were dropped. Software talks to Lattice by sending it small
JSON messages called **entities**.

An **entity** is one thing on that map, written as a list of named parts ("components"). Lattice has three
shapes of entity, and we use all three:

| Shape | What it is | Our use |
|---|---|---|
| **asset** (`TEMPLATE_ASSET`) | a vehicle we control | the drone |
| **track** (`TEMPLATE_TRACK`) | something a sensor saw | the hiker and the tent we detected |
| **geo-entity** (`TEMPLATE_GEO`) | a shape drawn on the map | search areas, the flight boundary, drop points |

**This package builds those messages correctly, checks them, merges repeated sightings of the same target
into one entity, and defines the tiny radio message that carries a target report from the drone to the
ground.** That is all it does so far (phase 1). Sending to a (fake or real) Lattice is phase 2 and 3; see
[`docs/lattice/LATTICE_PLAN.md`](../../../docs/lattice/LATTICE_PLAN.md).

### The safety promise

Lattice is an extra. If Lattice, the internet or this code breaks, **the drone must fly exactly as it does
today.** So:

* this package is **separate**: no flight node imports it, `launcher.sh` and `tracker.launch.xml` never start
  it, and a test (`test_hygiene.py`) checks that;
* it uses **only the Python standard library** (no new packages to install on the Jetson);
* the builders are **pure functions**: numbers in, a dict out. They never open a file, never touch the
  network, never read the clock, never sleep. A test reads the source code to prove it;
* bad input raises a clear `ValueError` (or, for radio text and detections, is ignored with a reason). Nothing
  half-wrong is ever produced, and nothing is made up.

## 2. Quick start (copy and paste; the tests run this block)

```python
from datetime import datetime, timedelta, timezone

from lattice_bridge import (
    JsonlSink, TargetClusterer, build_target_track, build_uav_asset, decode_tgt, encode_tgt, make_mission_id,
    uav_entity_id, validate_entity,
)

# Times are ALWAYS timezone-aware. In real code: now = datetime.now(timezone.utc)
now = datetime(2026, 10, 24, 15, 30, 0, tzinfo=timezone.utc)
mission = make_mission_id(now, 'runway')                  # 'runway20261024t153000z' - never reused by a later flight

# 1. The drone. Missing inputs (None) are simply left out; nothing is invented.
drone = build_uav_asset(
    uav_entity_id(mission), 36.2143, -96.0068,
    fix_time=now - timedelta(seconds=1),                  # when the GPS fix was taken -> provenance.sourceUpdateTime
    now=now,                                              # only used to check the expiry
    alt_amsl_m=260.0, geoid_undulation_m=-29.0,           # height above sea level + a configured geoid value
    rel_alt_m=49.0, heading_deg=90.0, mode='AUTO', armed=True, battery_percent=87.0, link_age_s=0.8,
)
validate_entity(drone, now)                               # raises EntityValidationError if a Lattice rule is broken
print(drone['location']['position'])                      # altitudeHaeMeters is 260.0 - 29.0 = 231.0

# 2. Targets: feed every detection to the clusterer; it decides create / update / no_change.
clusterer = TargetClusterer(mission)
for seconds, lat_error in [(0, 0.0), (1, 0.00002), (2, -0.00001)]:      # three photos of ONE hiker (2 m apart)
    decision = clusterer.feed('person', 36.21513 + lat_error, -96.00621, 0.9, now + timedelta(seconds=seconds))
    if decision.should_publish:
        track = build_target_track(now=now + timedelta(seconds=seconds), expiry_time=now + timedelta(hours=8),
                                   **decision.track_kwargs())
        validate_entity(track, now + timedelta(seconds=seconds))
        print(decision.action, track['entityId'])         # create ... then (maybe) update, always the same id
assert len(clusterer) == 1

# 3. The radio message (Jetson -> ground), 50 characters at most, strict on the way in.
text = encode_tgt('person', 36.214935, -96.006203, 0.95, 12)
print(text)                                               # TGT p 36.214935 -96.006203 95 12
assert decode_tgt(text).lat_deg == 36.214935
assert decode_tgt('TGT p 36.2 -96.0 95 12') is None       # anything that is not exactly the format is ignored

# 4. Record every request in a file (offline mode and test oracle).
with JsonlSink('lattice_requests.jsonl') as sink:
    sink.write('entity', drone, now)                      # one line: {"kind":"entity","payload":{...},"t":"..."}
```

## 3. The public interface (everything importable from `lattice_bridge`)

| Module | Name | What it does |
|---|---|---|
| `entities` | `build_uav_asset(entity_id, lat, lon, *, fix_time, now, ...)` | the drone as an **asset**; altitude, heading, speed, mode, battery and link health are optional |
| | `build_target_track(entity_id, kind, lat, lon, confidence, *, measured_time, now, expiry_time, ...)` | a hiker or tent as a **track** at the projected target position |
| | `build_area_geo(entity_id, name, corners_latlon, *, source_update_time, now, expiry_time, ...)` | a search area or the flight boundary as a closed, counter-clockwise **polygon** |
| | `build_drop_point(entity_id, kind, lat, lon, *, release_time, now, expiry_time, ...)` | where a payload was released, as a **geo point** |
| | `validate_entity(entity, now)` | the contract checker; raises `EntityValidationError` (a `ValueError`) listing every problem |
| | `find_problems(entity, now)` | the same checks, returned as a list of sentences (empty = valid) |
| | `EntityValidationError` | the exception; `.problems` holds the list |
| | `hae_from_amsl(amsl_m, geoid_undulation_m)` | height above the ellipsoid = height above sea level + geoid value |
| | `altitude_is_approximate(geoid_undulation_m)` | `True` when no geoid value is configured (`warn_if_altitude_approximate` in `logutil` logs the warning once) |
| | `ALTITUDE_WARNING`, `ALTITUDE_NOTE` | the long text to log, and the short text that goes into the drone's `description` |
| | `confidence_to_track_quality(confidence)` | detector confidence 0..1 -> Lattice track quality 0..15 |
| `clustering` | `TargetClusterer(mission_id, ...)` | `.feed(kind, lat, lon, confidence, measured_time)` -> `Decision`; `.targets()`, `.get(id)`, `len()` |
| | `Decision` | `.action`, `.should_publish`, `.entity_id`, `.state`, `.time_advanced`, `.retired`, `.retired_ids`, `.reason`, `.track_kwargs()` |
| | `TargetState` | frozen snapshot of one target (id, kind, number, position, best confidence, sightings, first/last seen) |
| | `state_track_kwargs(state, measured_time=None)` | keyword arguments for `build_target_track` from a `TargetState` (use it for goodbye updates of merged targets; pass the merging photo's time) |
| | `ACTION_CREATE`, `ACTION_UPDATE`, `ACTION_NO_CHANGE`, `ACTION_IGNORED` | the four possible `Decision.action` words |
| `report_codec` | `encode_tgt(kind, lat, lon, confidence, seq)` | build the radio text `TGT p 36.214935 -96.006203 95 12` (never over 50 characters) |
| | `decode_tgt(text)` | read it back into a `TgtReport`, or `None` for anything else; never raises |
| | `TgtReport` | `.kind`, `.lat_deg`, `.lon_deg`, `.confidence_percent`, `.confidence`, `.seq` |
| | `next_seq(seq)` | the message counter after `seq` (wraps from 999999 to 0) |
| `ids` | `make_mission_id(start_time, label=None)` | `'runway20261024t153000z'`: unique per flight |
| | `uav_entity_id`, `target_entity_id`, `drop_entity_id`, `area_entity_id` | the stable entity ids (`astra-<mission>-uav`, `...-tgt-person-1`, ...) |
| | `parse_entity_id(entity_id)` | read an id back into its parts, or `None` |
| | `target_display_name(kind, number)` | `'Person 1 (ASTRA)'`, the name shown in the Lattice UI |
| `world_areas` | `search_areas(world_config)`, `flight_boundary(truth)` | read the simulator's world files into `AreaSpec` lists |
| | `AreaSpec` | `.name`, `.note`, `.corners_latlon`, `.geo_type` |
| `sinks.jsonl_sink` | `JsonlSink(path)` | `.write(kind, payload, t=None)` -> `bool`, `.close()`: the append-only recorder |
| | `read_records(path)`, `latest_by_entity_id(path)`, `Record` | read a recording back; "what does Lattice hold now?" |
| `sinks` | `Sink` | the two-method protocol every sink (file now, fake Lattice and real Lattice later) offers |
| `logutil` | `warn_if_altitude_approximate(geoid_undulation_m)`, `warn_once(key, message)` | log a loud warning **once per process** (the bridge calls the first at start-up with its settings) |
| `timeutil` | `format_utc(dt)`, `parse_utc(text)`, `from_unix_seconds(s)` | UTC text with a Z, and back; naive datetimes are refused |
| `constants` | `INTEGRATION_NAME`, the enum words, `PLATFORM_TYPE_*`, limits | every fixed word and number, in **one** file |
| `geo` | `require_lat_lon`, `haversine_m`, `offset_latlon`, `clean_polygon_ring`, ... | number checks and small geometry (not re-exported; import from `lattice_bridge.geo`) |

## 4. Ideas you need (each explained once)

| Word | Meaning |
|---|---|
| **entity id** | the name Lattice knows a thing by. Publish the same id again = update that thing; a new id = a new thing. Ours look like `astra-runway20261024t153000z-tgt-person-1` (mission id inside, so a later flight never overwrites an old one). |
| **expiryTime** | when Lattice deletes the entity by itself. It must be in the future and less than 30 days away (we allow at most 29). The drone gets *now + 120 s* and is refreshed with every update, so it disappears about two minutes after the radio link dies. Targets live until the planned end of the mission plus a few hours. |
| **sourceUpdateTime** | when the *measurement* was made (the GPS fix, the photo), **not** when we sent it. Lattice only accepts an update when this time changes. Stale data keeps its true old time. |
| **UTC time with Z** | `2026-10-24T15:30:00.000000Z`. Every time we write looks like this. A Python time **without** a time zone ("naive") is refused everywhere, because nobody can tell which zone it meant. |
| **AMSL / HAE** | MAVLink (the drone) reports height above **mean sea level**. Lattice wants height above the **WGS84 ellipsoid** (a smooth mathematical Earth). They differ by the **geoid undulation** N: *HAE = AMSL + N*. In Oklahoma N is roughly **-30 m**, so using AMSL as HAE puts the drone about 30 m too high. N is a setting (`geoid_undulation_m`); we never guess it. If it is not set, the AMSL value is used and the drone's `description` says so (and `warn_if_altitude_approximate()` logs `ALTITUDE_WARNING` once). |
| **AGL** | height above ground. We fill `altitudeAglMeters` from MAVLink's `relative_alt`, which is height above the **take-off point (home)**, *not* above the ground under the drone. On sloped ground it is only an approximation. |
| **track quality** | Lattice has no "confidence" field. We write the confidence in the `description` text, and also map it to `tracked.trackQualityWrapper` (a score 0..15; confidence 0.95 -> 14). That mapping is an approximation. |
| **goodbye update** | an entity published with `isLive = false` makes Lattice delete it. `is_live=False` builds one (used at shutdown and for targets that were merged). Give it a **new** time (the shutdown time): Lattice ignores an update whose `sourceUpdateTime` did not change. |
| **golden file** | a complete example request saved in `test/golden/`, checked by hand against the Lattice docs. A test demands that the code still produces exactly it. |

## 5. What we publish (short form; the full table is in `docs/lattice/DATA_CONTRACT.md`)

| Entity | Shape | Id | Position | Time |
|---|---|---|---|---|
| our drone | asset, `UAV`, friendly, air | `astra-<mission>-uav` | GPS fix; HAE from AMSL + N; AGL from `relative_alt`; heading as a quaternion | updated ~1 Hz, expiry now + 120 s, `sourceUpdateTime` = fix time |
| hiker / tent | track, `Person` / `Tent`, neutral, surface | `astra-<mission>-tgt-<person\|tent>-<n>` | the **projected target position** (never the drone's) | `sourceUpdateTime` = measurement time; expiry = mission end + margin |
| search areas, flight boundary | geo-entity polygon (`GEO_TYPE_GENERAL`, `GEO_TYPE_CONTROL_AREA`) | `astra-<mission>-area-<name>` | corners from the world / mission files | once; expiry 12 h |
| drop points | geo-entity point | `astra-<mission>-drop-<person\|tent>-<n>` | the drone's position at the release | on release |

`Person` and `Tent` (and `UAV`) live only in `constants.py` and are marked **"confirm in the Lattice Schema
Registry"**: the REST reference calls `platformType` free text, but the Registry may define preferred names.

## 6. The target radio message

`TGT <p|t> <lat> <lon> <confidence 0-100> <seq>`, for example `TGT p 36.214935 -96.006203 95 12`.
At most 43 characters (the MAVLink limit is 50). One message per accepted target change, sent by the Jetson as
a STATUSTEXT with severity INFO (6) through `/mavros/statustext/send`. (The sending side is phase 2 and will be
**off by default**.) The decoder treats the text as **untrusted radio input**: it accepts exactly this
format and returns `None` for anything else; it never raises and never evaluates or formats the text.

## 7. How target clustering decides (and why)

| Rule | Value | Why |
|---|---|---|
| same target | same kind and within **12 m** of the target's current position | one photo's position error (GPS up to ~1 s old at ~10 m/s) is a few metres; real targets are 100+ m apart |
| reported confidence | the **highest** ever seen | "best confidence wins" |
| reported position | mean of all sightings, each weighted by **confidence squared** | averaging many photos cancels random error; squaring keeps a poor sighting from pulling hard |
| publish again when | moved >= 1 m, or confidence +0.02, or two targets merged | a hiker seen in 20 photos should not cause 20 updates |
| two clusters grow together | they merge; the older id survives and the other is reported in `Decision.retired` | the result then does not depend on the order the photos arrived in |
| bad data | ignored, with a reason in `Decision.reason` | the bridge must never crash on one bad detection |
| memory | at most 64 clusters, a few numbers each | bounded, as flight-adjacent code must be |

## 8. Run the tests

```bash
# from the repo root, inside Ubuntu 22.04 (WSL) - no ROS and no build needed:
cd ros2_ws/src/lattice_bridge
python3 -m pytest -q -p no:cacheprovider test

# the way the build system does it (what a clean machine sees):
cd ros2_ws && colcon build --packages-select lattice_bridge && source install/setup.bash
LATTICE_BRIDGE_USE_INSTALLED=1 python3 -m pytest -q -p no:cacheprovider src/lattice_bridge/test
# (run it from ros2_ws, not from inside the package folder: python would then import the source copy
#  instead of the installed one; colcon test --packages-select lattice_bridge works as well)
```

| Test file | What it proves |
|---|---|
| `test_golden.py` | each builder still produces the reviewed example (`test/golden/*.json`) |
| `test_entities_*.py`, `test_validate_entity.py` | every field and error of the builders and of the contract checker, plus random valid / garbage inputs |
| `test_clustering.py`, `test_scenarios.py` | merging rules, order independence, and the **real hiker / tent / decoy / area positions** of the simulator worlds |
| `test_report_codec.py` | exact format, round trips, the 50-character limit, and more than 80 000 fuzzed messages |
| `test_time_handling.py`, `test_timeutil.py` | naive times refused, UTC text everywhere, expiry rules |
| `test_logutil.py` | the altitude warning is logged once, and only when no geoid value is configured |
| `test_jsonl_sink.py` | one line per request, append-only, never raises, bounded, thread safe, torn last line |
| `test_hygiene.py` | standard library only, no I/O or clock in the pure modules, purpose paragraphs, no secrets or personal paths in the public repo, packaging agrees |
| `test_docs.py` | the examples in this README really run, and the interface table lists every public name |

Changed what we send on purpose? Regenerate the golden files and **review the diff against the Lattice docs**:

```bash
LATTICE_UPDATE_GOLDEN=1 python3 -m pytest -q -p no:cacheprovider test/test_golden.py
git diff test/golden
```

The simulator worlds' positions used by the scenario tests are copied into `test/data/scenario_truth.yaml`
(so the tests also run from a clean copy of this package). After regenerating a world:
`python3 test/scenario_data.py --write`.

## 9. Troubleshooting

| You see | Why | Fix |
|---|---|---|
| `ValueError: ... is a naive datetime` | a `datetime` without a time zone | use `datetime.now(timezone.utc)` or `datetime(..., tzinfo=timezone.utc)` |
| `ValueError: expiry_time must be in the future` | the expiry is not after `now` | for the drone leave `expiry_time` out (default now + 120 s); for others pass mission end + margin |
| `ValueError: expiry_time is more than 29 days away` | Lattice refuses 30+ days | use a nearer time |
| `ValueError: ... no GPS fix` | latitude and longitude are both 0 | the autopilot has no fix yet; wait and publish nothing |
| `EntityValidationError: ... not camelCase` | a key like `platform_type` | the REST JSON uses `platformType`; use the builders |
| drone height looks 30 m too high in Lattice | AMSL used as HAE (no geoid value set) | set `geoid_undulation_m` (about -30 for the Skyway Range; look it up with an EGM96 / EGM2008 calculator) |
| `Decision.action == 'ignored'` | the detection was bad (see `Decision.reason`) | check kind, lat/lon, confidence 0..1 |
| `JsonlSink.write(...)` returns `False` | bad content, disk problem or size limit; a warning is logged | look at the log; the program keeps running by design |
| the freshness test fails after regenerating a world | `test/data/scenario_truth.yaml` is a copy | `python3 test/scenario_data.py --write` |

## 10. Files

| Path | What |
|---|---|
| `lattice_bridge/constants.py` | every fixed word and number |
| `lattice_bridge/entities.py` | the four builders + the contract checker |
| `lattice_bridge/clustering.py` | `TargetClusterer` |
| `lattice_bridge/report_codec.py` | the `TGT` radio message |
| `lattice_bridge/ids.py`, `timeutil.py`, `geo.py`, `world_areas.py`, `logutil.py` | ids, times, geometry, world-file adapter, log-once helper |
| `lattice_bridge/sinks/` | `Sink` protocol and the JSONL recorder |
| `test/` | tests, `golden/` examples, `data/scenario_truth.yaml` |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | ament_python packaging (no console scripts yet) |

## 11. Not built yet (see the plan)

Phase 2 (the fake local Lattice, the MAVLink source, the bridge program, the `lattice_mock` simulator profile and the
Jetson-side `TGT` reporter, default off) is built and ran end to end in the Gazebo twin on 2026-10-31: how to run it,
the numbers, and what that did and did not prove are in [`docs/lattice/BRIDGE.md`](../../../docs/lattice/BRIDGE.md). Phase 3: the real Lattice sink with
retries and an offline buffer. Phases 4-7: thumbnails, video, tasking (only with a human confirm) and field use. Open
questions for the team (why Lattice, who holds the tokens, ground or Jetson bridge) are in section 9 of
`docs/lattice/LATTICE_PLAN.md`.
