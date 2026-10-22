# sim_plugins: plug-and-play slots for the ASTRA simulator

`sim_plugins` lets a teammate swap **one piece** of the simulation for their own (the autopilot, the
camera, the detector, the mission brain, the world, extra programs) with a small YAML **profile**, and
then **check** that the new piece talks exactly like the one it replaces.

The user guide is [docs/simulation/PLUGINS.md](../../../../docs/simulation/PLUGINS.md) (and
[ARDUPILOT.md](../../../../docs/simulation/ARDUPILOT.md) for the autopilot). This page is for people who
change `sim_plugins` itself.

## Commands

```bash
ros2 run sim_plugins profile_tool list                                  # the profiles in simulation/profiles
ros2 run sim_plugins profile_tool validate [--check] [PROFILE ...]      # check profiles (all without a name)
ros2 run sim_plugins profile_tool plan --profile NAME [--set K=V] [--check] [--for demo|start_sim]
ros2 run sim_plugins profile_tool resolve --profile NAME                # the complete settings as YAML
ros2 run sim_plugins profile_tool contract [--slot detector] [--markdown]
ros2 run sim_plugins profile_tool shell --for demo|start_sim ...        # used by demo.sh / start_sim.sh
ros2 run sim_plugins check_slot --slot detector|mission_brain|camera|autopilot|world|extras [--node N]
```

Every `ros2 run sim_plugins profile_tool ...` also works as `python3 -m sim_plugins.profile_tool ...`.

## Modules

| Module | Job | Needs ROS? |
|---|---|---|
| `schema.py` | every profile key: default (= today's simulator), allowed values, one line of help | no |
| `profiles.py` | find, read, merge (`extends`, `--set`) and check a profile; plain-words errors with "did you mean" | no |
| `params.py` | read Mission Planner / QGC parameter files and drop the lines that would break the simulator | no |
| `autopilot.py` | the autopilot slot: which `arducopter`, which parameter files, which run folder, external mode | no |
| `plan.py` | a profile -> launch arguments, script variables, environment checks, the printable plan | no |
| `contracts.py` | the written contract of every slot (topics, types, QoS, services, files, ports, rules) | no |
| `tracker_filter.py` | a copy of the team's `tracker.launch.xml` without the node a plug-in replaces | no |
| `siyi_probe.py` | knocks on a SIYI camera (UDP SDK 0x02, HTTP media API) for the camera check | no |
| `launch_support.py` | profile entries -> launch actions (used by `sim.launch.py` and `team_stack.launch.py`) | launch |
| `check_slot.py` | the conformance checker over a `Graph` (live `RosGraph`, or `FakeGraph` in tests) | rclpy (main only) |
| `profile_tool.py` | the command line | no |

The rule that keeps today's behaviour: **only settings that differ from the defaults produce
arguments**. With no profile (or `default.yaml`) the scripts and launch files run exactly the commands
they ran before `sim_plugins` existed (`test/test_plan.py`, `test/test_launch_files.py`).

## Tests

```bash
cd /tmp && python3 -m pytest -q -p no:cacheprovider \
  ~/ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src/simulation/sim_plugins/test
```

The launch-file tests need ROS 2 Humble sourced (`source simulation/setup/sim_env.sh`); they evaluate the
launch files with a fake launch context and start nothing. `test/run_tests.sh` runs everything
(pytest, launch evaluation, shellcheck, fetch_sitl.sh dry runs, the demo/start_sim dry-run comparison).
