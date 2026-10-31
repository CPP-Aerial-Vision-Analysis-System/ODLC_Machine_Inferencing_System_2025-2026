<!--
docs/lattice/README.md -- PURPOSE: the front door of the Lattice work: what Lattice is in two sentences, which
file to read for what, how far the work has got (phase by phase), the copy-paste commands to check it, and
the safety promise. Written for students; the engineering plan is LATTICE_PLAN.md, the exact data we send is
DATA_CONTRACT.md, and the code is the ros2_ws/src/lattice_bridge package.
-->

# Anduril Lattice in the ASTRA project

> **Taking over the Lattice work? Start with [HANDOFF.md](HANDOFF.md)** (2026-10-31): what is built and proven,
> where the work stopped, and what to do next.

**Lattice** is a shared map and tasking program made by Anduril. If our drone, the hiker and the tent we find,
the search areas and the drop points appear on it, everyone on the team (and anyone we choose to share it with)
can watch the mission from one browser tab. This folder holds the plan and the facts; the code lives in
[`ros2_ws/src/lattice_bridge`](../../ros2_ws/src/lattice_bridge/README.md).

> **Safety promise.** Lattice is an extra. If Lattice, the internet or the bridge is down, slow or buggy, the
> drone flies exactly as it does today. The bridge is a separate program, it is never started by
> `launcher.sh` or `tracker.launch.xml`, and the part that would run on the Jetson is off by default.

## Which file do I read?

| I want to ... | Read |
|---|---|
| **run** the bridge (with the simulator and a pretend Lattice, or from a recording) | [`BRIDGE.md`](BRIDGE.md) |
| understand why and how we connect to Lattice, and in which phases | [`LATTICE_PLAN.md`](LATTICE_PLAN.md) |
| know exactly what JSON we send, field by field, and where the numbers come from | [`DATA_CONTRACT.md`](DATA_CONTRACT.md) |
| use or change the code | the package [README](../../ros2_ws/src/lattice_bridge/README.md) (interface table, examples, tests) |
| see example requests | `ros2_ws/src/lattice_bridge/test/golden/*.json` (with the hand-review notes in the README next to them) |
| read the Lattice documentation we saved | the `*.mhtml` pages in the repo root (live version: `https://developer.anduril.com`) |

## Where the work stands (2026-10-31)

| Phase | Work | State |
|---|---|---|
| 0 | decide why, get sandbox access | open: the questions are in `LATTICE_PLAN.md` section 9; no account or token exists yet |
| 1 | **core library**: entity builders and checker, stable ids, target clustering, radio message codec, JSONL recorder | **done and tested** (package `lattice_bridge`) |
| 2 | fake local Lattice, MAVLink source, bridge program, simulator profile, Jetson-side reporter (default off) | **built, tested (also in WSL with ROS 2) and run end to end in the Gazebo twin**: run `20261031-094837_runway` met the Lattice checks of `LATTICE_PLAN.md` section 6 step 3 for the real targets, but not step 3 as written (visible run, one search area in that world, score comparable not identical; numbers in [`BRIDGE.md`](BRIDGE.md), "What is proven"). During that demo Mission Planner gets none of the team's texts (MAVROS behind MAVProxy). Not yet: the Jetson itself, a real radio, the in-flight fault tests (step 4) |
| 3 | real Lattice sink with retries and an offline buffer | not started; needs the account |
| 4-7 | thumbnails and areas, video, tasking (with a human confirm), field use | not started |

Phases 1 and 2 need **no Lattice account**: everything is tested against recordings and a fake Lattice in the
Gazebo twin.

## Check it yourself (copy and paste, Ubuntu 22.04 or WSL, no ROS needed)

```bash
cd ros2_ws/src/lattice_bridge
python3 -m pytest -q -p no:cacheprovider test        # about 1,930 tests, under a minute (a few are skipped without ROS 2)
```

The same suite from a clean build, the way the build system sees the package:

```bash
cd ros2_ws && colcon build --packages-select lattice_bridge && source install/setup.bash
LATTICE_BRIDGE_USE_INSTALLED=1 python3 -m pytest -q -p no:cacheprovider src/lattice_bridge/test
```

## Things we found out while building phase 1

* The Lattice documentation moved: `docs.anduril.com` now redirects to `developer.anduril.com`.
* The saved Entities page writes `platform_type`; the REST reference and the Quickstart write `platformType`.
  The REST JSON is camelCase, and our checker rejects snake_case keys.
* Lattice has no "confidence" field. We write it into the description and also map it to `trackQualityWrapper`
  (0-15). Assets can report `health` (battery, link) and `status.platformActivity` (flight mode).
* MAVLink altitude is above sea level, Lattice wants height above the ellipsoid: about 30 m difference in
  Oklahoma. It is a setting, never a guess (`DATA_CONTRACT.md` section 4).
* Points still to confirm before the sandbox phase are listed in `DATA_CONTRACT.md` section 9.
