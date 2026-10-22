# sim_plugin_examples: templates to plug your own code into the simulator

Two small, heavily commented ROS 2 packages to **copy** when you want to replace or add a piece of the
simulation. Both are built by `simulation/setup/03_build.sh` like every other package, so they are ready
to try at once.

| Template | Slot | What it shows | Try it |
|---|---|---|---|
| [`example_detector`](example_detector/README.md) | detector | reads the team's photos exactly like `new_od` and publishes the same `interfaces/ImageResult` on `/image_detection` (here with a simple colour-blob rule) | `bash simulation/scripts/demo.sh --profile example_detector` |
| [`example_listener`](example_listener/README.md) | extras | listens to telemetry, mission progress, shutter presses and detections, and logs a summary | `bash simulation/scripts/demo.sh --profile example_listener` |

Every template README has the same five steps: **copy -> rename -> change -> plug in (a profile) ->
check (`ros2 run sim_plugins check_slot ...`)**. The rules each slot must follow (topics, message types,
QoS, files) are in [docs/simulation/PLUGINS.md](../../../../docs/simulation/PLUGINS.md).
