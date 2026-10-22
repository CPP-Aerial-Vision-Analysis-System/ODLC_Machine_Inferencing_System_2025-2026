# example_listener: a template "extra" you copy

An **extra** is anything you want running next to the simulation that is not one of the fixed slots: a
logger, a monitor, a test driver. This template only **listens** and never publishes, so it can never
disturb the mission. Every few seconds it logs a summary of the flight:

```
flight     connected=True armed=True modes=STABILIZE > AUTO > GUIDED armed 182 s, max height 30.4 m, flown 612 m
mission    reached items: [1, 2, 4, 5, 6, 7, 8, 9, 10]
camera     shutter presses: 41
detections photos reported: 41; found: person x3, tent x2; best: person 0.91 (36.21 , -96.00.jpg), tent 0.84 (...)
messages   Reached last WP. Holding to finish image | Targets: tent@11 -> person@12. Diverting.
```

and when it stops it saves the same as JSON (`~/.astra_sim/example_listener_summary.json`).

| File | What it is |
|---|---|
| `example_listener/summary.py` | the bookkeeping (plain Python, no ROS). **Change this** to record what you need. |
| `example_listener/node.py` | the ROS 2 node: subscribes, feeds the summary, prints and saves it |
| `test/test_summary.py` | unit tests (no ROS needed) |

Topics it listens to (QoS BEST_EFFORT for `/mavros/*`, so it hears every publisher; RELIABLE for
`/image_detection`, like `main_controller`): `/mavros/state`, `/mavros/global_position/global`,
`/mavros/global_position/rel_alt`, `/mavros/mission/reached`, `/mavros/statustext/recv`, `/camera/trigger`,
`/image_detection`.

## Copy -> rename -> change -> plug in -> check

**1. Copy:** `cp -r example_listener my_logger` (inside `ros2_ws/src/...`).

**2. Rename** `example_listener` to `my_logger` in: the inner folder `my_logger/example_listener/` ->
`my_logger/my_logger/`, `resource/example_listener` -> `resource/my_logger`, `package.xml` `<name>`,
`setup.py` (`package_name` and the entry point `'my_logger = my_logger.node:main'`), `setup.cfg` (both
paths) and `super().__init__('my_logger')` in `node.py`.

**3. Change** `summary.py` (what to remember) and the subscriptions in `node.py` (what to listen to). Keep
`python3 -m pytest test -q` green. Rule for extras: **listen only**, unless replacing something on purpose.

**4. Build and plug in** with a profile's `extras` list:

```yaml
profile: 1
name: my_logger
extras:
  - name: my_logger
    package: my_logger
    executable: my_logger            # or launch: my_logger.launch.py, or command: [python3, /path/script.py]
    node_name: my_logger
    start_with: team_stack           # starts/stops with ./launcher.sh sim; "simulator" = with sim.launch.py
    args: {summary_period_s: 5.0}    # ROS parameters
```

```bash
bash simulation/setup/03_build.sh --workers 4
bash simulation/scripts/demo.sh --profile ./my_logger.yaml
```

A profile can add an extra to another profile: `extends: truth_detector` plus the `extras:` list.

**5. Check** while it runs: `ros2 run sim_plugins check_slot --slot extras --node my_logger` (it warns
if your extra publishes on a topic another slot owns).

## Troubleshooting

| What you see | Why | Fix |
|---|---|---|
| nothing about `/mavros/...` arrives | a RELIABLE subscription to a BEST_EFFORT publisher | subscribe with `qos_profile_sensor_data` |
| no JSON file at the end | the node was killed hard (not Ctrl+C) | stop the demo normally; or write the file from the timer too |
| `extras[0]: give exactly ONE of launch, executable or command` | the profile entry | keep only one of the three |
