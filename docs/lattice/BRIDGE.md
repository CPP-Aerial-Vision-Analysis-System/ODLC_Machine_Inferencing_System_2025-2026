<!--
docs/lattice/BRIDGE.md -- PURPOSE: how to run the Lattice bridge, step by step, for a student who has never seen
it: try it with the simulator and a pretend Lattice (no account, nothing leaves the computer), the Jetson-side
target reporter, replay a recorded flight, check a setup without sending anything, keep the tokens safe, what
happens when something breaks, and exactly what has and has not been proven so far (with run folders and numbers).
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

You need the simulator working (see [`simulation/README.md`](../../simulation/README.md)) and the workspace built
with `bash simulation/setup/03_build.sh` (it builds this package too; pymavlink and MAVProxy come with the setup).

**1. Start the simulator with the Lattice profile**

```bash
bash simulation/scripts/demo.sh runway --profile lattice_mock
```

That starts the normal demo **and** four extra programs
([`simulation/profiles/lattice_mock.yaml`](../../simulation/profiles/lattice_mock.yaml)):

| Program | Pretends to be | Job |
|---|---|---|
| `lattice_reporter` | software on the **Jetson** | starts with the team's flight stack; sends one short `TGT ...` radio text per hiker / tent the detector finds |
| `lattice_ground_radio` | the **telemetry radio + Mission Planner** | a second ground station (MAVProxy) on the autopilot's spare port: it hears everything the real radio would hear, including the Jetson's texts, and passes it to the bridge on UDP 14554 |
| `lattice_bridge` | the bridge on the **ground laptop** | turns that stream into Lattice entities |
| `fake_lattice` | **Lattice** | takes the entities, draws the map, and writes every request to `<run folder>/lattice/fake_lattice_journal.jsonl` |

**2. Open the pretend Lattice in a browser:** <http://127.0.0.1:8089/>

**3. Watch.** After the drone arms you should see, within a few seconds:

* a blue **triangle** (the drone) that moves,
* the **search box** (and, in worlds that have one, the dashed **flight boundary**),
* a green **circle** for the hiker and an orange one for the tent once they are reported (about a minute into the
  search),
* a red **diamond** where each payload was dropped,
* a table underneath with how many seconds ago each thing was updated.

When the flight ends and the demo closes, the drone disappears from the map (the bridge tells Lattice
"this drone is no longer live"; the pretend Lattice waits 4 seconds for that last message before it stops).

**Afterwards:** everything the pretend Lattice received is in `~/.astra_sim/runs/<date>_runway/lattice/`
(`fake_lattice_journal.jsonl`, one line per update request with the time it arrived and the answer it got, then a
last `stopped` line with the counters), the bridge's lines are in that run's
`sim.log` (search for `lattice_bridge`), and the reporter's lines are in its `team_stack.log` (search for
`radio: TGT`). Other pages of the pretend Lattice while it runs: `/entities.json` (the map now), `/history.json`
(every update it took), `/stats.json` (counters).

> **Why a second ground station?** Because `lattice_mock` keeps **MAVROS behind MAVProxy** (UDP 14552, the
> profile's `network.fcu_url` line; the reason is in Trouble, the `mission start refused` row). The `TGT` texts and
> the `Servo 13 -> 1400` drop texts are sent by the team's nodes through MAVROS, so they reach the autopilot through
> MAVProxy, and MAVProxy does not copy one client's messages to its other outputs (we know this from reading
> MAVProxy's code, not from a run). A bridge on an ordinary MAVProxy output would then see the drone but never a
> target or a drop. ArduPilot passes a message on to every *other* port where it has heard a ground station (never
> back to the port it came from), so a second ground station on the autopilot's spare port SERIAL1 hears them, the
> way the real radio hears the Jetson on TELEM2.
> With MAVROS on SERIAL2 (the simulator's default) an ordinary MAVProxy output **does** get the texts: the first
> lattice run (`20261031-094007_runway`) was wired exactly like that and its bridge received all 8 of the reporter's
> texts. The profile's header explains its wiring choices line by line.

> **Mission Planner during this demo.** Connect it on **UDP 14550** (TCP 5762 is taken by `lattice_ground_radio`).
> It shows the drone (mode, height, battery, GPS), but with MAVROS behind MAVProxy it gets **none of the team's
> texts** (`Detected ...`, `Aim ...`, `Targets: ...`, `Servo 13 -> ...`, `TGT ...`), so the ASTRA plugin's target
> cards, target and drop markers, and the phases it reads from those texts stay empty. The pretend Lattice map
> (<http://127.0.0.1:8089/>) is the place to watch the targets in this demo. This ends when the profile can drop its
> `fcu_url` line.

## The Jetson side: the target reporter

The **reporter** (`ros2 run lattice_bridge lattice_reporter`, file `lattice_bridge/reporter_node.py`) is the only
piece that would run on the drone. It listens to the detector (`/image_detection`), finds where each hiker or tent is
on the ground with the **team's own code** (`main/src/target_geolocation.py`, the same function main_controller aims
with), and sends a short text through MAVROS, for example `TGT p 36.215133 -96.006219 95 1` (p = person, t = tent,
latitude, longitude, confidence in percent, message counter).

* **It is OFF on the real drone.** `launcher.sh` and `tracker.launch.xml` do not start it. In the simulator the
  `lattice_mock` profile starts it with the flight stack.
* **First look on the real drone:** run it with `dry_run` so it decides and logs but sends nothing:
  `ros2 launch lattice_bridge lattice_reporter.launch.py dry_run:=true` (in a second terminal after `./launcher.sh`).
* **Settings:** every one is explained in
  [`config/lattice_reporter.yaml`](../../ros2_ws/src/lattice_bridge/config/lattice_reporter.yaml). Two can also be
  given to the launch file: `min_confidence:=0.5` (ignore weaker detections; default 0.0, like main_controller) and
  `min_move_m:=1.5` (send a target again when its best sighting moved that far; default 5). The simulator profile uses
  both values. 0.5 removes the stock detector's **weakest** false "tent" calls but **not all** of them: without a
  filter (run `094007`) three false tent tracks at 30-36 % reached the map, and the real tent was first reported at
  33 % and 39 % (81-86 % later); with 0.5 (run `094837`) a tarp at 56 % still became a tent track, and the real tent
  was reported at 63 % then 84 %. A higher value also holds back a target's first weak sightings. 1.5 m keeps the
  Lattice track close to the point main_controller will aim at. **The team should choose the real-drone values.**
* **Radio budget:** at most one text every 2 s, 6 per target, 300 per flight.

## The flight stays safe

Nothing in this setup is allowed to change the flight: the reporter only adds texts, the bridge and the pretend
Lattice run on the "ground", and if any of them crashes the drone flies the same. In the complete run below
(`20261031-094837_runway`) the team's mission (photos, detections, targets, drops, RTL) ran as in every other demo.

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
| `that UDP port is already in use` | only one program can listen on a port, and exactly one stream must feed the bridge's port. In `lattice_mock.yaml` that stream is `lattice_ground_radio`'s `--out udp:127.0.0.1:14554`, so do not also add a MAVProxy output on 14554. In your own profile with MAVROS on SERIAL2 (no `fcu_url` line), a MAVProxy output (`network.extra_outs: [udp:127.0.0.1:14554]`) also works and gets the Jetson's texts (run `094007`); with MAVROS behind MAVProxy it would miss every `TGT` and drop text (see "Why a second ground station?") |
| `the MAVLink source needs the Python package pymavlink` | `python3 -m pip install pymavlink` |
| `AMSL is published as HAE ...` warning | add `--geoid-undulation -30` (about -30 near the Skyway Range) |
| `a replay into Lattice (--sink rest) must run in real time` | use `--speed 1`, or send the fast replay to `--sink jsonl` |
| `no Lattice address` / `LATTICE_TOKEN is not set` | set the variables (see Secrets); `--dry-run` tells you which |
| the map stays empty | is the demo flying? Is the bridge's log showing `health: drone=fresh`? Did `lattice_ground_radio` print `Connect tcp:127.0.0.1:5762` in `sim.log` (TCP 5762 must be free: no Mission Planner on TCP 5762 during this demo)? |
| the drone is on the map but never a hiker, a tent or a drop | the Jetson's texts do not reach the bridge. The bridge must listen behind a ground station on a different autopilot port than the one MAVROS uses (with MAVROS behind MAVProxy: not a MAVProxy output; see "Why a second ground station?"). Check `radio: TGT` in `team_stack.log` (the reporter sent it) and `tgt_reports=` in the bridge's `health:` line (the bridge got it) |
| before take-off and after landing the drone is refreshed only once a minute | normal: an entity whose content did not change (a parked drone) is re-sent at half its 2-minute lifetime; in flight the position changes and it goes out about once a second |
| extra "Tent" tracks on the map | false alarms of the stock detector (rubble, tarps). Raising the reporter's `min_confidence` removes the weakest ones only: 0.5 (the simulator profile) removed the 30-36 % ones of run `094007`, but a tarp at 56 % still became a track in run `094837`. A higher value also holds back a real target's first weak sightings (the real tent started at 33 % in run `094007`). There is no setting that removes all of them; the team chooses the real-drone value |
| Mission Planner shows the drone but no target cards, markers or team messages during this demo | expected with `lattice_mock`: MAVROS sits behind MAVProxy, so the team's texts reach only the second ground station (see "Mission Planner during this demo"). Watch the targets on <http://127.0.0.1:8089/> |
| demo.sh stops right after arming: `mission start refused (MAV_RESULT_ACCEPTED), mode is STABILIZE` | with MAVROS on the autopilot's SERIAL2 (the simulator default since 2026-10-31), ArduPilot also forwards MAVROS's own heartbeat to UDP 14550 and `mission_tool` mistakes it for the autopilot's. `lattice_mock` keeps MAVROS behind MAVProxy (`network.fcu_url`) until `mission_tool` is fixed |

## What is proven, and what is not (2026-10-31)

**Proven in the Gazebo twin** (WSL2 Ubuntu 22.04, ArduCopter SITL 4.6.3, the team's real flight stack with the real
YOLO detector, runway world, visible runs). Numbers come from the pretend Lattice's journal, the run's
`phases.jsonl` and `score_report.md`, and the truth file `sim_worlds/truth/runway.yaml`.

Run `~/.astra_sim/runs/20261031-094837_runway` (`demo.sh runway --profile lattice_mock`), a complete mission:
2 of 2 targets found, both payloads dropped (1.2 m from the tent, 2.5 m from the hiker), RTL, landed, demo.sh exit 0.
At the end demo.sh's own stop check printed `1 process(es) are still running: 13685` with an empty command line; a
search for the simulator's programs right after the demo found none, and the same warning appears in demos without
the bridge (`20261031-092448_runway`, `20261031-095801_runway`), so it is probably an existing quirk of the stop
check, not something the bridge left behind.

Step 3 as written in the plan is **not fully met**: it asks for a *headless* run (this one was visible, as the owner
wants every simulator run), *both* search areas (the runway world has only one) and an *identical* score with the
bridge on and off (it was comparable, see the last row). Its Lattice checks were met for the real targets:

| Check (LATTICE_PLAN.md section 6, step 3) | Result |
|---|---|
| the drone is updated about once a second all flight | **yes**: 277 updates in the 287 s from arming to landing (0.97 per second), median interval 1.00 s. The 3 gaps longer than 1.5 s all fell while the drone stood still: 3.0 s on the pad right after arming (16:50:13.5-16:50:16.5 UTC, the same position and height at both ends), 2.0 s in the hover after the hold (16:52:46.8-48.9 UTC, 30 m up) and 4.0 s in the tent drop loiter (16:53:05.9-09.9 UTC, 49 m up): the publisher does not repeat identical content. Before take-off and after landing a parked drone is refreshed every 60 s. Every update's `sourceUpdateTime` was newer than the last; the fix was 0.28 s old on arrival (median, at most 0.59 s). Flight modes, height above the ellipsoid (180-229 m) and above home (0-49 m) were shown |
| the Jetson's `TGT` texts travel over MAVLink to the bridge | **yes**: the reporter sent 5 texts (`radio: TGT ...` in `team_stack.log`); the bridge received all 5 (`tgt_reports=5`) |
| exactly one track per real target | **yes for the real targets**: one hiker track and one tent track, never a duplicate. A third track, "tent" at 56 %, was the detector calling a tarp (distractor `box_tarp_5`, 0.2 m away) a tent; main_controller chose the real tent (84 %). The detector's false alarms reach Lattice as tracks unless the reporter's `min_confidence` filters them |
| track positions within 2 m of the truth | **yes**: hiker 1.33 m, tent 0.16 m (main_controller's own aim points in the same run: 2.6 m and 1.1 m; the bridge averages the reports) |
| the search areas and both drop points are present | **yes**: the runway world's one search area (`search-box`; this world has no flight boundary) and two drop points (1.13 m from the tent, 2.54 m from the hiker: the drone's position at the release) |
| nothing is published after the final `isLive false` | **yes**: the demo's stop signal came at 16:57:09.71 UTC, the drone's goodbye arrived 0.10 s later (16:57:09.82), then nothing; the pretend Lattice stopped 4 s later (`--linger-s 4`). Without the linger the goodbye would have found the pretend Lattice already gone |
| nothing was refused | **yes**: 293 requests, 293 accepted, 0 refused (contract, token, malformed), 0 discarded |
| the demo's score is the same with the bridge on and off | **comparable, not identical**: runs differ by chance. This run 2/2, drops 1.2 / 2.5 m; the same day without the bridge: 2/2, drops 2.1 / 0.4 m (`20261031-091316_runway`) and 2/2, drops 1.0 / 1.1 m (`20261031-092448_runway`) |

Run `20261031-094007_runway` (the first try) already had the pretend Lattice with its journal and the 4 s linger.
It was wired differently: MAVROS on the autopilot's SERIAL2 (the simulator default, `fcu_url=tcp://127.0.0.1:5763`
in `team_stack.log`), the bridge on an ordinary MAVProxy output (`--out udp:127.0.0.1:14554` in `sim.log`), and the
reporter with its default settings (no confidence filter, re-send after a 5 m move). demo.sh ended early, soon after
take-off. The console line with its reason was not kept (the run script's log was overwritten by the next run); what
was kept fits `mission_tool` misreading the flight mode (see Trouble): no drop, the demo closed 2:14 after arming,
and the main MAVProxy logged `Detected vehicle 1:191` (MAVROS's heartbeat). The drone kept flying its search on its
own until the demo closed. This run still proved:

* the Jetson's texts reach a bridge on an **ordinary MAVProxy output when MAVROS is on SERIAL2**: the reporter sent
  8 `TGT` texts (messages 0-7 in `team_stack.log`), the bridge received all 8 (`tgt_reports=8`) and made 5 tracks
  ("targets 5 created / 3 updated");
* drone updates about once a second in AUTO (median interval 1.00 s);
* the linger matters: the stop signal came at 16:44:10.44 UTC and the goodbye arrived 0.12 s later; the pretend
  Lattice took it, stopped 4 s later, and nothing came after the goodbye; 142 of 142 requests accepted.

It also found two problems that the profile has fixed since (run `094837` above is the result):

* three false "tent" tracks at 30-36 % from detector false alarms (the profile now passes `min_confidence 0.5`,
  which removes such weak ones but not all, see the reporter section);
* a hiker track 3.26 m from the truth, from a single report, because the reporter re-sent a target only after a 5 m
  move (the profile now passes `min_move_m 1.5`).

Separately, the first WSL test run with ROS 2 sourced (not a simulator run) had 10 log-checking tests fail that pass
on Windows: ROS's pytest plugins change every new Python logger (now fixed in `test/conftest.py`).

**Proven by automated tests** (1,931 on Windows Python; 1,938 in WSL with ROS 2 Humble sourced, from the source tree
and from a `colcon build` install, including the ROS-only parity test of the reporter's target positions against
main_controller's real `locate_detection`):

* every entity we send passes the contract checker, and exact example JSON files were reviewed against the live
  Lattice documentation;
* the REST sink against the pretend Lattice over **real sockets**: delivered, 401 (no retry flood), 400 (drop only
  that version), 429 with `Retry-After`, 503, a slow server, a dead port, a redirect, secrets never in messages;
* the bridge program end to end: a replayed run into a file and into the pretend Lattice (real time), the dry run, all
  exit codes;
* the MAVLink decoder with **real pymavlink messages** (units, "heading unknown", battery -1, (0,0), an autopilot
  clock that is far off, a reboot, other vehicles, the ground station and Jetson heartbeats) and a real UDP frame.

**Not proven yet** (be honest with Areg):

* it has **never talked to a real Lattice**: no account or token exists. Names such as `Person` / `Tent` are still to
  be confirmed in the sandbox ([`DATA_CONTRACT.md`](DATA_CONTRACT.md) section 9). Whether real Lattice shows a parked
  drone as offline between its 60-second refreshes is also unknown;
* the reporter ran on ROS 2 in the simulator, **not on the Jetson**, and never over a real telemetry radio (the
  simulator's "radio" is a local TCP port: no delay, no loss);
* one complete run only: the fault tests of section 6 step 4 (pretend Lattice down, slow, answering 500 during a
  flight) were not run in Gazebo (they are unit-tested with real sockets);
* only the runway world (one search area, no flight boundary); `suas_field` was not flown with the bridge;
* the ROS source (`--source ros`) has still not run against a live MAVROS;
* during a `lattice_mock` demo Mission Planner gets none of the team's texts (MAVROS behind MAVProxy, see "Mission
  Planner during this demo"), so the bridge and the ASTRA Mission Planner plugin have never shown the targets
  side by side;
* the pretend Lattice's journal was tightened after these runs (refused PUTs get a line too, `t` is the arrival time
  even with an injected delay, an update still being answered at the stop is never lost): unit-tested only. The two
  runs above had no refused request and no injected delay, so their journals are not affected.

More: [`LATTICE_PLAN.md`](LATTICE_PLAN.md) for the phases, [`DATA_CONTRACT.md`](DATA_CONTRACT.md) for the exact data,
the package [README](../../ros2_ws/src/lattice_bridge/README.md) for the code.
