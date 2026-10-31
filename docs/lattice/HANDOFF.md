<!--
docs/lattice/HANDOFF.md -- PURPOSE: the hand-over note for the team members who take over the Lattice work on
2026-10-31. It says in plain words what is built and proven, where the work stopped (including a change that was in
progress), what to do next in order, the commands to check that everything still works, and the rules (no tokens in
the repo, the real drone comes first). Read this first, then README.md (front door), BRIDGE.md (how to run it) and
LATTICE_PLAN.md (the full plan and the open decisions).
-->

# Lattice: hand-over to the team (2026-10-31)

The owner stopped the AI-assisted Lattice work on 2026-10-31 and handed it to the team. Everything that existed at
that moment is committed on the `gazebo-sim` branch (the commit that adds this file). Nothing is lost and nothing
was left half-installed: the code is consistent and its tests pass (see "Check it still works").

## 1. What Lattice is for (two sentences)

Lattice (made by Anduril) is a shared live map. Our ground laptop sends it the drone's position, the hiker and tent we
find, the search areas and the drop points, so anyone with access can watch the mission in a browser.

## 2. What is built

| Piece | Where | What it does | State |
|---|---|---|---|
| Core library | `ros2_ws/src/lattice_bridge/lattice_bridge/` | turns drone data into Lattice "entities" (drone, targets, areas, drops) | done, tested |
| Bridge program | `bridge_node.py` (`ros2 run lattice_bridge bridge ...`) | reads the drone's MAVLink stream on the ground and publishes to Lattice | done, proven in the Gazebo twin |
| REST sink | `sinks/rest_sink.py` | sends `PUT /api/v1/entities` to a Lattice server, token only from an environment variable | done, tested against the fake server only |
| Fake Lattice | `fake_lattice.py` (`ros2 run lattice_bridge fake_lattice --port 8089`) | a pretend Lattice on your own PC: checks every field, live map at http://127.0.0.1:8089/, journal of every request | done, tested |
| Jetson reporter | `reporter_node.py`, `launch/lattice_reporter.launch.py`, `config/lattice_reporter.yaml` | on the Jetson: turns detections into short radio texts `TGT p <lat> <lon> <conf> <seq>` sent through MAVROS | done; **OFF on the real drone** unless someone starts it |
| Simulator profile | `simulation/profiles/lattice_mock.yaml` | one command starts the fake Lattice, the bridge and the reporter next to the normal demo | done |
| Docs | `docs/lattice/README.md`, `BRIDGE.md`, `DATA_CONTRACT.md`, `LATTICE_PLAN.md` | front door, how to run, the exact data, the plan | up to date as of this commit |

## 3. What is proven (and what is not)

Proven in the Gazebo twin (run `~/.astra_sim/runs/20261031-094837_runway`, `demo.sh runway --profile lattice_mock`,
a complete mission with 2 of 2 targets and both drops):

* the drone appeared on the fake Lattice about once a second (277 updates in 287 s);
* exactly one track per real target: the hiker 1.33 m and the tent 0.16 m from the true position;
* the search area and both drop points were there; nothing arrived after the final "not live" update;
* 293 of 293 requests accepted; the Jetson reporter sent 5 TGT texts and the bridge received all 5.

NOT proven yet:

* **a real Lattice server** (needs the sandbox account and token; the team lead has it). The REST sink has only talked
  to the fake server;
* the reporter on the **real** Jetson and the real telemetry radio (1 Hz telemetry, radio drop-outs);
* "score identical with the bridge on and off": only comparable (2/2 targets both ways, slightly different drop
  errors), and only one search area exists in the runway world;
* one detector false alarm (a tarp called a tent at 56 %) still became a track with `min_confidence` 0.5.

## 4. Where the work stopped

1. **A small change was in progress when the work was stopped.** The last fixer was making the fake Lattice's journal
   record the moment each request *arrived* (not when the line was written) and refuse writes after the server
   stopped (`fake_lattice.py`: `received=` argument, `_closed` flag). The tests pass with it (1937 passed, 9 skipped on
   Windows Python), but nobody reviewed that last change: read `git show <this commit> -- ros2_ws/src/lattice_bridge/lattice_bridge/fake_lattice.py`.
2. **Why `lattice_mock` looks complicated right now.** Since 2026-10-31 the simulator connects MAVROS to SITL's
   SERIAL2 (TCP 5763), like the real Jetson on TELEM2, so the team's texts reach every ground station. That change made
   ArduPilot also forward MAVROS's own heartbeat, and the demo tool `mission_tool` sometimes mistook it for the
   autopilot's (it read "disarmed" or "STABILIZE" and stopped a demo after 12 s). To be safe, `lattice_mock` pins MAVROS
   behind MAVProxy (`network.fcu_url: udp://:14552@localhost:14552`) and adds a second ground station on TCP 5762 for the
   bridge. Side effect: during a `lattice_mock` demo Mission Planner shows the drone but **none of the team's texts**.
   The `mission_tool` bug is being fixed by the simulator side (see the git log for "mission_tool"); once that is in,
   the simpler setup works: delete `network.fcu_url` and the second ground station from the profile and give the bridge
   a plain MAVProxy output again (`network.extra_outs: ["udp:127.0.0.1:14554"]`). Then update
   `test/test_reporter_docs.py`, which currently checks the pinned setup.
3. **Phase 8, several ground stations in different places, was never started.** The owner asked for it on 2026-10-24
   ("connect to multiple different GCS stations in different locations that the drone will be able to communicate
   with"). The planned first step was a design note `docs/lattice/MULTI_GCS.md`: each station with its own radio and
   bridge, all feeding ONE drone entity and the same target tracks; hand-over when a station loses the radio; exactly one
   station "in command" at a time; what happens without internet at a station; the MAVLink system/component ids. See
   LATTICE_PLAN.md section 7, Phase 8, and the open decisions 6-8 in section 9.

## 5. What to do next (suggested order)

1. Run the checks below and read BRIDGE.md once, end to end.
2. Review the in-progress `fake_lattice.py` change (point 4.1).
3. When the `mission_tool` fix is in, simplify `lattice_mock` (point 4.2) and fly it once to confirm Mission Planner
   shows the team's texts again while the bridge still gets the TGT texts.
4. Get the Lattice sandbox token from the team lead and do the first real-server test (LATTICE_PLAN.md section 6 and
   BRIDGE.md, "secrets"): token in an environment variable only.
5. Decide the open questions in LATTICE_PLAN.md section 9 (who owns the account, what to share, the real-drone
   `min_confidence`; evidence so far: the real tent was first reported at 33-39 % in one run and 63 % in another, a tarp
   false alarm reached 56 %).
6. Phase 8 (several ground stations): design note first, then build.

## 6. Check it still works

On Windows (no ROS needed for the library tests):

```bash
cd ros2_ws/src/lattice_bridge
python -m pytest -q -p no:cacheprovider test
```

Expected at hand-over: `1937 passed, 9 skipped` (the skipped ones need ROS 2).

In WSL Ubuntu-22.04 with ROS 2 sourced (all tests, including the ROS ones; last full result 1938 passed, 2 skipped):

```bash
source ~/ODLC_Machine_Inferencing_System_2025-2026/simulation/setup/sim_env.sh
cd ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/lattice_bridge
/usr/bin/python3 -m pytest -q -p no:cacheprovider test
```

The whole thing in the simulator (opens the windows; watch the live map in a browser):

```bash
bash simulation/scripts/demo.sh runway --profile lattice_mock
# then open http://127.0.0.1:8089/ ; afterwards the evidence is in <run folder>/lattice/fake_lattice_journal.jsonl
```

## 7. Rules that keep it safe

* **No token ever goes into a file in this repository** (the repo is public). Tokens come only from environment
  variables (`LATTICE_TOKEN`, `LATTICE_ENDPOINT`, `SANDBOXES_TOKEN`); the bridge scrubs them from its logs.
* The bridge only **listens** to the drone: it never sends commands, so a Lattice problem cannot change the flight.
* The Jetson reporter is **off by default** on the real drone.
* Do not change the team's flight code for Lattice; the bridge reads what the drone already says.
* Every code file starts with a PURPOSE paragraph; docs stay simple enough for a 5th grader; when you add or move a
  doc, list it in `docs/README.md` (the documentation index).
