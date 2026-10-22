# simulation/profiles: plug-in profiles

A **profile** is a small YAML file that says which piece fills each **slot** of the simulator
(the world, the autopilot, the camera, the detector, the mission brain, extra programs, network
addresses). A profile only needs the lines it changes; everything else stays as the simulator is
today (`default.yaml` writes every setting out).

| Profile | What it changes |
|---|---|
| `default.yaml` | nothing: today's simulator, every setting written out and explained (copy it to start your own) |
| `truth_detector.yaml` | the answer-key detector instead of the team's YOLO model (= `demo.sh --detector truth`) |
| `example_detector.yaml` | the template detector `sim_plugin_examples/example_detector` instead of `new_od` |
| `example_listener.yaml` | adds the template "extra" `sim_plugin_examples/example_listener` (it only listens) |
| `ardupilot_version.yaml` | ArduCopter 4.7.1 (get it first: `bash simulation/setup/fetch_sitl.sh 4.7.1`) |
| `ardupilot_source.yaml` | ArduCopter built from an ArduPilot source tree (`/opt/ardupilot` in the Docker sitl image) |
| `real_drone_params.yaml` | the real drone's Mission Planner `.param` file on top (`params/example_real_drone.param` is only an example) |
| `external_autopilot.yaml` | EXAMPLE: the autopilot runs on another computer / container |
| `docker_split.yaml` | EXAMPLE: the team stack runs in another container (network settings) |

Use one:

```bash
bash simulation/scripts/demo.sh --profile truth_detector
bash simulation/scripts/start_sim.sh --profile ardupilot_version
bash simulation/scripts/demo.sh --profile ./my_profile.yaml --set camera.model=a8mini --dry-run
```

Check one (and see exactly what it will start):

```bash
python3 -m sim_plugins.profile_tool list
python3 -m sim_plugins.profile_tool plan --profile example_detector --check
```

The whole story (every slot's contract, every key, the templates, the checker, FAQ):
[docs/simulation/PLUGINS.md](../../docs/simulation/PLUGINS.md). The autopilot slot in depth:
[docs/simulation/ARDUPILOT.md](../../docs/simulation/ARDUPILOT.md).
