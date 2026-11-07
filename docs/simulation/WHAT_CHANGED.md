<!-- PURPOSE: summarize the simulator additions and team-code fixes, distinguish measured simulator results from hardware validation, and point reviewers to code/tests and open decisions. -->

# What changed and what has been proved

The digital twin adds a Gazebo/SITL/camera environment around the team's mission software. It also exposed
bugs in that software. The hardware entry point remains `./launcher.sh`; `./launcher.sh sim` selects the
MAVROS simulation connection (the simulator's scripts pass `FCU_URL=tcp://127.0.0.1:5763`, SITL's SERIAL2,
wired like the Jetson on the Pixhawk's TELEM2), and `TRACKER_CMD` lets the simulator use its wrapper launch.
Hardware validation is a separate requirement: these changes have not been certified by flight on the real drone.
The current acceptance state is maintained in [HANDOFF](../HANDOFF.md), with the system map in
[AGENTS.md](../../AGENTS.md).

## Simulator additions

| Area | Resulting behavior | Where to review |
|---|---|---|
| Drone/world/camera | Gazebo Harmonic Iris with gimbal, SIYI protocol emulator and generated search worlds | `sim_models`, `sim_worlds`, `siyi_emulator` under `ros2_ws/src/simulation/` |
| Runtime tools | Start/stop, mission upload/fly, readiness and phase monitoring | `sim_bringup`, `simulation/scripts/` |
| Observation | RViz markers, per-run score reports, the old Qt panel (off in `demo.sh` by default since `83c2ae4`; `--panel` brings it back) | `sim_demo` |
| Ground station | Mission Planner is the ground station: the ASTRA plugin adds a tab (phases, hiker/tent cards, buttons), the drone camera in the HUD and map markers (`593bb96`; version 1.1 in `0ddda83` keeps the tab first and warns in amber when telemetry stops); the camera picture comes from `pov_server`, an MJPEG stream on `http://127.0.0.1:8081/pov.mjpg` (`4eb462a`) | `simulation/windows/mission_planner_plugin/astra_gcs.cs`, `sim_demo/pov_server.py`; [MISSION_PLANNER](MISSION_PLANNER.md) |
| MAVROS wiring | MAVROS talks to SITL SERIAL2 (TCP 5763) like the Jetson on TELEM2, so ArduPilot forwards the team's status texts to every ground station (`6d90c07`) | `simulation/scripts/demo.sh`, `sim_plugins/plan.py`; [ARCHITECTURE](ARCHITECTURE.md) |
| Lattice | A bridge that reports the drone and its targets to Anduril Lattice, tested against a pretend local Lattice; handed to the team on 2026-10-31 (`f139036`) | `ros2_ws/src/lattice_bridge/`; start with [docs/lattice/HANDOFF.md](../lattice/HANDOFF.md) |
| Swappable components | Profiles, custom adapters, ArduPilot selection, slot checker | `sim_plugins`, `simulation/profiles/`, [PLUGINS](PLUGINS.md) |
| Repeat/operator flow | Manual Mission Planner startup, START OVER and Windows launchers | `run_reset.py`, `start_over.sh`, `simulation/windows/`; [MISSION_PLANNER](MISSION_PLANNER.md) |
| Portability | Container helpers and display/network overlays | `docker/`, `simulation/docker/`; [DOCKER](DOCKER.md) |
| Real terrain | Source manifest, preparation/generation code and site pack | `sim_worlds/realfield/`; [WORLDS](WORLDS.md) |

Implemented code and passing unit tests are not the same as a completed runtime acceptance test. Profiles,
reset/manual flows, real-terrain generation and Docker each have explicit remaining checks in the handoff.

## Team-code fixes

| Trigger before the fix | Revised behavior | Main code and tests |
|---|---|---|
| Remote mission-index parameter read failed; controller dereferenced `None` and died | Nonblocking reads keep old indices, log failures and retry | `wp_sender/parameter.py`, `param_tracking.py`, parameter-reader tests |
| GCS uploaded a mission after the stack started; MAVROS/mission nodes kept an older copy | waypoint_manager requests a mission pull on upload text, startup and arming; indices derive from the new list | `waypoint_mavros/mission_sync.py`, `waypoint.py`, `test_mission_sync.py` |
| Controller aimed at drone GPS when a photo was taken | Camera sidecars record pose/lens values; controller projects the chosen box center to a ground aim point | `video_cam/photo_metadata.py`, `main/src/target_geolocation.py`, their tests |
| Lens housekeeping/capture overlap discarded triggers | Bounded thread-safe photo queue, shorter SDK lock holds and camera capability checks | `video_cam/capture_queue.py`, `siyi_node.py`, queue/metadata tests |
| Long sleeps blocked controller callbacks | One-shot timed steps keep callbacks active through hold/drop/resume | `main/src/timed_steps.py`, `test_timed_steps.py` |
| Decimal ROS parameters supplied as whole numbers prevented node startup | Numeric settings accept both integer and floating values, with logged fallback for wrong types | controller/camera/trigger parameter handling and trigger tests |
| Report's location metric showed detector GPS rather than corrected target aim | Separate aim, photo-GPS and drop metrics, using controller `Aim ...` STATUSTEXT | `sim_demo/scoring.py`, `score_run.py`, `test_score_aim.py` |
| A mission pull MAVROS never answers kept waypoint_manager waiting for ever (it could fly an old mission) | The pull is abandoned after 45 s, the last mission copy is kept, up to 3 retries (commit `94ccf0b`) | `waypoint_mavros/mission_sync.py`, `waypoint.py`, `test_mission_sync.py`, `test_waypoint_node.py` |
| An unreadable photo folder made the detector hold never end (hover in GUIDED) | The hold always ends at 90 s on a monotonic clock; a failed divert sends RTL (`94ccf0b`) | `main/src/processing_hold.py`, `main_controller.py`, `test_processing_hold.py`, `test_controller_safety.py` |
| NaN/infinite GPS, box or score values could become a NaN waypoint | Rejected at every step; `/addWaypoint` refuses NaN, infinite and (0,0) targets (`94ccf0b`) | `main/src/target_geolocation.py`, `waypoint.py`, `video_cam/siyi_node.py`, their tests |

Photo names and published message interfaces stay compatible with the team pipeline. No simulation-only
service is required by the team's hardware nodes. The simulator supplies its own model path and disables
the TensorRT-specific batched detector path for portable `.pt` testing.

## Existing measured evidence

The historical four-fix study recorded 3 before and 3 after headless runs: controller timeouts went from
one crash in three runs to none in three; earlier drops 14–25 m away improved to roughly 0.05–1.3 m;
31–36 saved photos with about 50 lost triggers improved to 92–131 saved photos with none reported lost.
Those are recorded simulator observations, not hardware performance guarantees.

The handoff records a later 2026-10-22 runway run with real YOLO/team code: 2/2 targets, 97 photos all
processed, drops approximately 0.8 m and 1.3 m, RTL and landing in about five minutes. After the three
real-drone safety fixes (`94ccf0b`) two more runway runs gave 2/2 targets with drops of about 1 m and no lost
photos (runs `20261024-104711_runway`, `20261024-105800_runway`, table in the handoff). The commit that moved
MAVROS to SITL SERIAL2 (`6d90c07`) records run `20261031-092448_runway`: phases, detections, target positions
and both drops shown in Mission Planner, score drops 1.0 m and 1.1 m. New live review runs must record their
own results; these historical numbers should not be relabelled as current validation.

## Real-airframe decisions still to close

Projection assumes a near-level stabilized gimbal and a chosen ground plane. A single
`target_ground_height_m` cannot model varying terrain; the real search boxes have substantial relief.
Verify home/elevation datum, drop altitude and local ground height before treating a runway score as
evidence about the real field. Metadata age, shutter delay and real navigation error also affect aiming.

Measure the actual camera's FOV, zoom scale, mounted image orientation, gimbal/mount frame and firmware
responses. Confirm servo channels, travel and PWM positions against the airframe. Tests against the
emulator do not establish those hardware facts. The raw SIYI yaw sign is consistent with
[ArduPilot 4.6.3's SIYI driver](https://github.com/ArduPilot/ardupilot/blob/Copter-4.6.3/libraries/AP_Mount/AP_Mount_Siyi.cpp),
which negates raw yaw into its NED frame; roll/order/mounting assumptions still need the actual camera.

Mission uploads and mode/servo commands need real-link review: the legacy full-mission pushes are not
acknowledged before the controller's fixed settling delay. Slow links or rejection can invalidate its
assumed indices. A single high-confidence false positive can still select a drop; the existing model
maps umbrella/kite labels to tent. These remain review/calibration decisions unless the handoff records
an implemented, tested resolution.

## Reproduce a review

Read the affected package guide, inspect `git diff cfc8d7c --` for that package, run its functional tests
in the Linux build copy, and run the integration flow required by the task's Done when check. Save a
sanitized before/after table with model, firmware, world, camera settings, target outcomes, photo counts,
drop metric source and final-node state. Keep raw run data and machine paths out of the public commit.
