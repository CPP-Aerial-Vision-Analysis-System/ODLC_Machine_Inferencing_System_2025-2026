<!-- PURPOSE: explain how the simulator selects ArduCopter SITL binaries, source builds, parameter layers and remote autopilots, with repeatable checks and limits. -->

# Connect or change ArduCopter SITL

The default is ArduCopter **4.6.3**, matching the project's Pixhawk version. SITL runs that flight
controller firmware on Linux; Gazebo's ArduPilot plugin exchanges sensor/servo data with it through a
JSON physics link. Changing SITL does not require changing the team's mission nodes.
See [the architecture](ARCHITECTURE.md) for MAVProxy/MAVROS wiring.

Commands below run from the Linux checkout after `source simulation/setup/sim_env.sh`. Stop the
current simulator before changing firmware or rebuilding. Every alternate configuration still needs
its own full mission and score report; the available examples do not imply runtime certification.

## Use the default

```bash
bash simulation/setup/02_setup_ardupilot.sh
bash simulation/scripts/demo.sh runway --headless
```

Setup installs `~/sitl/arducopter`, stock defaults, MAVProxy and the Gazebo plugin. The launch selects
the binary and config from runtime paths, creates a SITL state directory, and starts it only after
the drone's physics plugin is listening. Saved `eeprom.bin` parameters can override defaults on later
runs. For a controlled experiment, use a fresh state directory or request `--set autopilot.wipe=true`.

## Try another downloaded version

```bash
bash simulation/setup/fetch_sitl.sh --list
bash simulation/setup/fetch_sitl.sh 4.7.1 --dry-run
bash simulation/setup/fetch_sitl.sh 4.7.1
python3 -m sim_plugins.profile_tool plan --profile ardupilot_version --check
bash simulation/scripts/demo.sh runway --sitl-version 4.7.1 --headless
```

`4.7.1` is the shipped example, not a statement about the latest release. `--list` queries the firmware
server; unavailable versions fail clearly. `stable`, `beta` and `latest` are moving aliases, so preserve
`SOURCE.txt` and the actual firmware identity for comparisons. The fetch script keeps the default binary
untouched and writes `~/sitl/versions/V/{arducopter,copter.parm,gazebo-iris.parm,SOURCE.txt}`.

Downloads are for x86_64 Linux. A Jetson/ARM machine needs an appropriate source build. Stock parameter
downloads can fall back to the installed files if unavailable; read the fetch warnings and provenance.
`--force` re-fetches an installed version; `--params-ref TAG` chooses stock defaults explicitly.

Equivalent persistent profile:

```yaml
# PURPOSE: compare a specific ArduCopter release with the default simulator setup.
autopilot:
  kind: sitl_binary
  version: "4.7.1"
```

## Use a binary or source build

```bash
bash simulation/scripts/demo.sh runway --sitl-bin "$HOME/sitl/custom/arducopter" --headless
# A source tree whose SITL binary is already built:
bash simulation/scripts/demo.sh runway --sitl-source "$HOME/ardupilot" --headless
```

For the source-tree option, build ArduPilot using its own dependency/install instructions, then run
`./waf configure --board sitl` and `./waf copter` inside that tree. The ASTRA script selects the existing
`build/sitl/bin/arducopter`; it does not compile ArduPilot for you. Source defaults are taken from the
tree where available. `fetch_sitl.sh LABEL --from-file BINARY --params-ref TAG` can register a teammate's
binary under a named version without replacing `~/sitl/arducopter`.

With `start_sim.sh`, use launch settings such as `sitl_version:=4.7.1` or `sitl_source:=...`, or a profile.
Its `--dry-run` prints the matching team-stack command for terminal 2.

## Layer mission parameters

```bash
bash simulation/scripts/demo.sh runway \
  --param-file simulation/profiles/params/example_real_drone.param --headless
```

That file is a teaching example, not the real aircraft's saved configuration. For your own local file,
use another `--param-file FILE`; the option repeats. Mission Planner `.param`, ArduPilot `.parm` and
QGroundControl `.params` formats are accepted. Stock Copter defaults, Gazebo Iris defaults and
`sim_models/config/astra_sitl.parm` come before the filtered extra layers; later values win.

The default `safe` filter removes simulator-critical settings, physical sensor calibration/wiring,
airframe/motor tuning and protected payload-servo settings. It keeps mission settings such as
`WPNAV_SPEED`, radius, RTL height and many failsafes. It does not reproduce the real airframe's dynamics.
Read the plan/launch summary for kept, removed and unreadable lines. Preserve the original file locally;
do not commit a raw aircraft parameter dump.

Profiles expose `autopilot.param_files`, `param_filter` and `param_keep`. `param_filter: none` bypasses
the filter; `param_keep` exempts particular names. Those are deliberate expert overrides and can prevent
the simulated Iris from arming or flying. Do not copy simulator failsafe/servo defaults onto hardware.

## An autopilot somewhere else

The [external profile](../../simulation/profiles/external_autopilot.yaml) is a wiring example. Set its
MAVLink endpoint to the remote SITL and its simulator address to this Gazebo machine's reachable address.

```bash
python3 -m sim_plugins.profile_tool plan --profile external_autopilot --check
bash simulation/scripts/demo.sh --profile external_autopilot --dry-run
# After changing the example addresses and starting the remote SITL:
bash simulation/scripts/demo.sh --profile ./my_external.yaml --headless
```

On the remote computer, SITL needs `--model JSON --sim-address=GAZEBO_HOST`, the same instance and home,
and compatible stock/Iris/ASTRA defaults. Copy the exact home and settings shown by the launch log. The
Gazebo plugin must listen on a reachable `fdm_addr` (external mode defaults to `0.0.0.0`), with UDP
`9002+10*I` reachable on Gazebo's machine. MAVProxy here needs TCP `5760+10*I` reachable on the SITL host.
Both machines must agree on model, instance, frame and home. External mode starts no local SITL.

This setup is for a JSON-compatible autopilot simulation. A physical Pixhawk requires a separate HIL
design; selecting `external` alone does not turn the Gazebo model into supported HIL hardware.

## Prove a configuration

First validate/plan the profile and inspect the resolved defaults. Then fly one complete headless runway
run, check MAVROS connectivity, mission refresh, both detections/drops, final landing and the score
report. Save the profile, actual binary identity, parameter provenance, camera/model settings and run
folder in your experiment notes. `check_slot --slot autopilot` checks the running interface after the
team stack starts; it does not prove flight quality. Stop all processes you started before the next run.
