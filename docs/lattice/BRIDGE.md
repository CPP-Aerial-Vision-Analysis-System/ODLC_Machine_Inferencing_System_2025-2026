<!--
docs/lattice/BRIDGE.md -- PURPOSE: how to run the Lattice bridge, step by step, for a student who has never seen
it: try it with the simulator and a pretend Lattice (no account, nothing leaves the computer), replay a recorded
flight, check a setup without sending anything, keep the tokens safe, what happens when something breaks, and
exactly what has and has not been proven so far.
-->

# The Lattice bridge: how to run it

The **bridge** is a small program that runs on the **ground laptop**. It listens to the same radio messages
that Mission Planner shows, and it puts four kinds of things on the shared Lattice map:

| On the map | Where it comes from |
|---|---|
| our **drone** (about once a second) | the drone's position messages |
| the **hiker** and the **tent** we found | the short `TGT ...` notes the Jetson sends over the radio |
| the **search areas** and the **flight boundary** | the simulator's world file |
| the two **drop points** | the "payload released" message |

> **Safety promise.** The bridge only *listens*. It is never started by `launcher.sh` or the flight launch
> files. If it crashes, hangs or the internet dies, the drone flies exactly the same.

## Try it in 3 steps (no Lattice account, nothing leaves this computer)

You need the simulator working (see [`simulation/README.md`](../../simulation/README.md)) and the package built:

```bash
cd ros2_ws && colcon build --packages-select lattice_bridge && source install/setup.bash
pip install pymavlink            # only once; the bridge reads the radio stream with it
```

**1. Start the simulator with the Lattice profile**

```bash
bash simulation/scripts/demo.sh --profile lattice_mock
```

That starts the normal demo **and** two extra programs: a *pretend Lattice* and the bridge
([`simulation/profiles/lattice_mock.yaml`](../../simulation/profiles/lattice_mock.yaml)).

**2. Open the pretend Lattice in a browser:** <http://127.0.0.1:8089/>

**3. Watch.** After the drone arms you should see, within a few seconds:

* a blue **triangle** (the drone) that moves,
* the **search box** (and, in worlds that have one, the dashed **flight boundary**),
* a green **circle** for the hiker and an orange one for the tent once they are reported,
* a red **diamond** where each payload was dropped,
* a table underneath with how many seconds ago each thing was updated.

When the flight ends and you stop the demo, the drone disappears from the map (the bridge tells Lattice
"this drone is no longer live").

> **Honest status.** Steps 1-3 were *not* run together with Gazebo yet. Each half was proven separately (see
> "What is proven" at the end). The first real run is the next job.

## Try it without the simulator (replay a recorded flight)

Every simulator flight is saved in `~/.astra_sim/runs/<date>_<world>/`. The bridge can play one back:

```bash
# terminal 1: the pretend Lattice
ros2 run lattice_bridge fake_lattice --port 8089

# terminal 2: play the recording in real time (a replay into Lattice must be real time, --speed 1)
export LATTICE_TOKEN="<any text>"              # the pretend Lattice accepts any token (type any word)
ros2 run lattice_bridge bridge --source replay --run-dir ~/.astra_sim/runs/<one of your runs> \
    --sink rest --endpoint http://127.0.0.1:8089 --area-config world:runway --geoid-undulation -30
```

Want it fast and written to a file instead? Use the file sink; every update is kept:

```bash
ros2 run lattice_bridge bridge --source replay --run-dir ~/.astra_sim/runs/<run> --speed 0 \
    --sink jsonl --output /tmp/lattice_requests.jsonl --area-config world:runway --geoid-undulation -30
```

A recorded run only has the drone's position while the camera was taking photos, so the replayed drone is
visible during the search and at the drops, and "goes quiet" in between. The replay never invents positions.

## Check a setup without sending anything: `--dry-run`

```bash
ros2 run lattice_bridge bridge --source mavlink --sink rest --endpoint http://127.0.0.1:8089 --dry-run
```

It prints what *would* happen and ends with `verdict: ready` or `verdict: NOT READY: <what is missing>`.
Nothing is sent, the radio is not opened. Do this before every real use.

## The pieces (so you know where to look)

```
 source ──events──▶ bridge core ──entities──▶ publisher ──requests──▶ sink
 (radio / ROS /     (decides what            (queue, 1 per second     (Lattice over HTTP /
  a recording)       Lattice should hear)     per thing, retries,      a file / a test sink)
                                              keeps data while offline)
```

| Piece | File in `ros2_ws/src/lattice_bridge/lattice_bridge/` | One-line job |
|---|---|---|
| source: radio | `sources/mavlink_source.py` | read the MAVLink stream, turn it into drone / target / drop events |
| source: recording | `sources/replay_source.py` | play a saved simulator run |
| source: ROS | `sources/ros_source.py` | read MAVROS topics (for a bridge running on the Jetson; needs ROS) |
| core | `bridge_core.py` | one hiker = one entity; a quiet radio stops updates; goodbye at the end |
| publisher | `publisher.py` | never blocks; latest data wins; waits 1, 2, 4 ... 60 s after a failure |
| sink: Lattice | `sinks/rest_sink.py` | `PUT /api/v1/entities` with the token; sorts every answer |
| sink: file | `sinks/jsonl_sink.py` | one request per line, for tests and offline use |
| pretend Lattice | `fake_lattice.py` | the same HTTP door, plus the live map page |
| the program | `bridge_node.py` | joins them, reads the settings, shuts down in order |

## Settings

Every setting has a built-in default. Change one with a flag (`--stale-after 20`), or put several in a
profile file and use `--profile my.yaml`. Start from
[`config/bridge.yaml`](../../ros2_ws/src/lattice_bridge/config/bridge.yaml): it lists *every* setting with an
explanation. **Later wins:** built-in default, then the profile, then `LATTICE_ENDPOINT`, then the command line.

A mistyped key in a profile (`stale_aftre_s`) is an **error with a hint** ("did you mean `drone.stale_after_s`?"),
never silently ignored.

The settings you will touch first:

| Setting (flag) | Why |
|---|---|
| `--source mavlink\|replay\|ros` | where the facts come from |
| `--sink rest\|jsonl\|mock` | where they go (`jsonl` writes a file and is the safe default) |
| `--connection udpin:127.0.0.1:14554` | which MAVLink port to listen on (**one program per port**) |
| `--area-config world:runway` | which world's search areas to draw (`world:suas_field` has a flight boundary) |
| `--geoid-undulation -30` | height correction for the field. **Without it the drone is drawn about 30 m too high or low**; the bridge warns loudly |
| `--stale-after 10` | seconds of radio silence before the drone stops being refreshed |
| `--mission-id` / `--mission-label` | the name inside every id, so a later flight never overwrites an earlier one |

## Secrets: tokens

A Lattice needs an address and one or two **tokens**. Rules:

* They come **only** from environment variables (`LATTICE_ENDPOINT`, `LATTICE_TOKEN`, `SANDBOXES_TOKEN`) or from
  a small file you give with `--secrets-file`. There is **no flag for a token**: command lines can be seen by every
  user of the computer.
* The secrets file has one `NAME=value` line per secret and **must live outside any git folder**. The bridge
  refuses a file inside a repository (so it can never be committed by accident) and, on Linux, a file others can read
  (`chmod 600` it).
* Tokens never appear in logs, plans or error messages; every log line is scrubbed.
* **Never** put a real token into a file in this repository. It is public.

## What happens when something breaks

| What breaks | What the bridge does | The drone |
|---|---|---|
| Lattice or the internet is down | keeps the newest version of each thing, retries after 1, 2, 4 ... 60 s, sends it when the link is back | unaffected |
| the token is wrong or expired | says so once, then tries **once a minute** (no flood), keeps the data | unaffected |
| one entity is refused (400) | drops that version, logs which one, carries on with the rest | unaffected |
| the radio goes quiet | after `--stale-after` seconds stops refreshing the drone (never repeats an old position); it fades from Lattice by itself in about 2 minutes | unaffected |
| the radio comes back | resumes at once | unaffected |
| one weird message arrives | counted and skipped | unaffected |
| the bridge crashes or you press Ctrl-C | on Ctrl-C: final "not live" update, sends what is waiting (up to 10 s), then exits | unaffected |

## Exit codes

`0` fine · `2` bad command line · `3` bad settings or a missing address / token · `4` the source or output cannot work
here (pymavlink missing, run folder or output file not usable) · `5` a bug (the traceback is in the log).

## Trouble

| You see | Do this |
|---|---|
| `that UDP port is already in use` | only one program can listen on a port. Give the bridge its own MAVProxy output (`network.extra_outs`, as `lattice_mock.yaml` does) |
| `the MAVLink source needs the Python package pymavlink` | `python3 -m pip install pymavlink` |
| `AMSL is published as HAE ...` warning | add `--geoid-undulation -30` (about -30 near the Skyway Range) |
| `a replay into Lattice (--sink rest) must run in real time` | use `--speed 1`, or send the fast replay to `--sink jsonl` |
| `no Lattice address` / `LATTICE_TOKEN is not set` | set the variables (see Secrets); `--dry-run` tells you which |
| the map stays empty | is the demo flying? Is the bridge's log showing `health: drone=fresh`? Is port 14554 in the profile's `extra_outs`? |

## What is proven, and what is not (2026-10-24)

**Proven by automated tests** (about 1,900, on Windows Python; they also run from a clean build in WSL):

* every entity we send passes the contract checker, and exact example JSON files were reviewed against the live
  Lattice documentation;
* the REST sink against the pretend Lattice over **real sockets**: delivered, 401 (no retry flood), 400 (drop only
  that version), 429 with `Retry-After`, 503, a slow server, a dead port, a redirect, secrets never in messages;
* the bridge program end to end: a replayed run into a file and into the pretend Lattice (real time), the dry run, all
  exit codes;
* the MAVLink decoder with **real pymavlink messages** (units, "heading unknown", battery -1, (0,0), an autopilot
  clock that is far off, a reboot, other vehicles, the ground station and Jetson heartbeats) and a real UDP frame.

**Not proven yet** (be honest with Areg):

* it has **not** run together with the Gazebo simulator (the Windows/WSL test machine's WSL service was
  unavailable during this work). That is the next job;
* it has **never talked to a real Lattice**: no account or token exists. Names such as `Person` / `Tent` are still to
  be confirmed in the sandbox ([`DATA_CONTRACT.md`](DATA_CONTRACT.md) section 9);
* the Jetson-side reporter (`lattice_reporter`) has unit tests but has not been run on ROS 2 or on the Jetson;
* the ROS source (`--source ros`) has the same status.

More: [`LATTICE_PLAN.md`](LATTICE_PLAN.md) for the phases, [`DATA_CONTRACT.md`](DATA_CONTRACT.md) for the exact data,
the package [README](../../ros2_ws/src/lattice_bridge/README.md) for the code.
