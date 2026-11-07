<!--
docs/jetson/FLYING.md -- PURPOSE: how to fly the simulator demo with the team's software running on the Jetson Orin
Nano (or on the "virtual Jetson" container on this PC): the one demo.sh command and its --jetson options, what happens
step by step, what lands in the run folder (photos, logs, RUN_DIR/jetson/summary.md with the "Differences from the
field" list), the photo modes for slow links, and what is not supported yet. For the cable / network itself see
docs/jetson/LINK.md; for the first connection (login, check, setup, image, deploy) see docs/jetson/README.md; the
design is docs/jetson/HIL_DESIGN.md (section 9.3).
-->

# Flying with the Jetson in the loop

**What it is:** the normal simulator demo, but the team's flight software (MAVROS + the team nodes + the detector)
runs in the container `astra-hil` on the real Jetson Orin Nano, talking to the simulated autopilot and camera on
this PC exactly as it talks to the Pixhawk and the SIYI camera on the drone. Gazebo, ArduCopter SITL, the camera
emulator, Mission Planner, the phases and the score report stay on the PC and work as always.

## Before the first flight (once)

1. The Jetson is connected and checked: docs/jetson/LINK.md, then docs/jetson/README.md ("Plug in and test").
2. The image is on the Jetson and the detector engine is built: `jetson.sh image build`, `jetson.sh deploy`,
   `jetson.sh warmup` (README). For the virtual Jetson: `jetson.sh image build --flavor virtual`, then
   `jetson.sh deploy --link virtual` (the model file and a self-test of the image on this PC's GPU).
   No board connected (owner, 2026-10-31 15:14: "work without it for now")? Fly the virtual Jetson: it is the
   acceptance test until the board is back, and every step below is the same.
3. The team is **not** using the board right now (the commands refuse otherwise, exit 16).

## Fly

In the Ubuntu (WSL) terminal, from the repo folder:

```bash
bash simulation/jetson/jetson.sh login                                          # once per session; keep the window open
bash simulation/scripts/demo.sh runway --jetson usb                             # USB-C cable
bash simulation/scripts/demo.sh runway --jetson ssh --jetson-host 192.168.144.20  # no cable: Ethernet / Wi-Fi / LAN
bash simulation/scripts/demo.sh runway --jetson virtual                         # no board: the container on this PC
```

Add `--dry-run` to see the plan without starting anything.

**How do I know it worked?** The terminal shows `OK the Jetson is ready`, then `OK Jetson link up ...`, then
`OK the team stack on the Jetson is ready`, then the phases (take-off, lanes, detections, drops, landing) as in a
normal demo. Mission Planner shows the team's status texts. At the end: `JETSON RUN SUMMARY: .../jetson/summary.md`.

### The --jetson options

| Option | Default | What it does |
|---|---|---|
| `--jetson usb\|ssh\|virtual` | (off) | where the team stack runs |
| `--jetson-host HOST`, `--jetson-user USER`, `--jetson-port N`, `--jetson-auth login\|key` | the saved target | the Jetson's address and login (LINK.md) |
| `--jetson-photos live\|after` | `live` | copy the Jetson's photos during the flight, or only after it (slow Wi-Fi) |
| `--jetson-tap on\|off` | `on` | copy the Jetson's ROS topics to the PC (needed for the phases and the score report) |
| `--jetson-baud 115200\|off` | `off` | slow the autopilot link down to the drone's 115200-baud TELEM2 serial cable (the owner decides the default) |
| `--jetson-pipeline auto\|new\|legacy` | `auto` | the camera pipeline in the container: the new video pipeline when the image has it, else the legacy photo pipeline |
| `--photo-pace auto\|off\|MBIT` | `auto` | in login mode, how fast photo downloads may go (LINK.md section 4) |
| `--jetson-like` | off | virtual Jetson only: limit the container to the Jetson's CPUs and memory |

Every other demo.sh option works as usual (`--camera`, `--timeout`, `--manual`, `--keep`, ...). Not allowed with
`--jetson`: `--detector truth|custom` (that detector would run on the PC, where the team code on the Jetson cannot
hear it).

## What happens, step by step

1. **Before anything starts:** `jetson.sh check --quick --for flight` (logged in, image matches the PC's code, engine
   present, team not busy, disk, memory). Any problem stops the demo here with its fix.
2. **Simulator, autopilot, mission upload:** exactly as in a normal demo.
3. **The Jetson (instead of `./launcher.sh` on the PC):** old photos on the Jetson are archived; the Jetson's clock
   offset is measured; `link.sh start` adds the doors (autopilot, camera, tap); `jetson.sh start --sim` starts the
   container; the ROS tap and the live photo copy start; `stack_check` runs inside the container until the team stack
   is ready (one restart if it is not, like a normal demo).
4. **Look down, arm, fly, watch:** as always. The "camera straight down" command is typed inside the container.
5. **At the end:** the container stops (the team stack shuts down cleanly); photos, detections and logs are copied
   into the run folder with their times moved to the PC clock; the link's doors are removed (your login window stays
   open); the run summary is written; then the usual score report and shutdown.

## What lands in the run folder (`~/.astra_sim/runs/<date>_<world>/`)

| File | What |
|---|---|
| `team_stack.log` | the team stack's output from the container (same name as a PC run, so every log tool finds it) |
| `photos/`, `detections/` | the Jetson's photos (+ `.json` sidecars) and detection images, file times on the PC clock |
| `jetson/summary.md` | **the Jetson run summary**: link, image, the detector backend that really ran (read from new_od's own log lines: TensorRT or PyTorch, on which GPU), the version table **this run's container vs the field Jetson** (OS, CUDA, cuDNN, TensorRT, ROS 2, MAVROS, PyTorch, Ultralytics, numpy, OpenCV, GPU; "Same?" per row), power mode at start and end, tegrastats (GPU load, temperature, memory), clock offsets, link drops, photos, what was connected, **Differences from the field** |
| `jetson/versions.env` | the versions read INSIDE the running container at the start (`simulation/jetson/pc/container_versions.py`, read-only) |
| `jetson/differences.txt` | the same "Differences from the field" list, one per line (the run summary `run_summary.md` uses it) |
| `jetson/clock.txt` | the Jetson's clock minus the PC's, at the start and the end |
| `jetson/link.log`, `link_state.env` | every door added, every drop and reconnect |
| `jetson/relay.log`, `relay.status`, `tap.log`, `ros_log/`, `tegrastats.log`, `facts.env`, `power.txt`, `container.log` | the Jetson's own files |
| `jetson/tap_receive.log`, `tap_status.env`, `pc_sdk.log`, `pc_http.log`, `collect.log` | the PC side of the link |

## Differences from the field

Every summary carries the fixed list of HIL_DESIGN.md section 12.1 (SITL instead of the Pixhawk, MAVLink over SSH
instead of the serial cable, the camera emulator through the relay, the proposed launch file the team has not adopted
yet, ROS domain 77, the Jetson clock, the board's setup, our own photo folder, the sealed ROS graph, the unknown
mission wiring of the new detections), plus what the run itself changed (virtual Jetson, plain SSH instead of USB,
the legacy photo pipeline, photos after the flight, no baud limit, a detector that did not run on TensorRT).

## Rehearse a lost link during a flight

While a `--jetson virtual` demo flies, in a second Ubuntu terminal:

```bash
bash simulation/jetson/pc/link.sh drop 20 --link virtual --mode blackhole    # 20 s of silence, sockets stay open
```

How do I know it worked? `<run folder>/jetson/relay.log` says `nothing from SITL for 4 s: the link is gone, dialing
again` and then `link back after 20.0 s of silence`; the team stack log shows MAVROS reading the autopilot's version
again; the flight goes on; `jetson/summary.md` lists `autopilot silent ...: 1 time(s), 20.0 s` and the bytes the gate
dropped. With the real board use `--link usb --mode glitch` (LINK.md section 6), or pull the cable.

## What was verified (2026-10-31, virtual Jetson; no board was connected)

One visible flight, `bash simulation/scripts/demo.sh runway --jetson virtual`, run folder
`~/.astra_sim/runs/20261031-153222_runway` (image `astra-hil:virtual` on the TensorRT 24.08 base, legacy photo
pipeline, a 20 s blackhole rehearsal 15 s into the search lanes):

- the team stack in the container connected to SITL SERIAL2 through the relay, `stack_check` passed inside the
  container, the phases and detections reached this PC through the tap (3118 messages), and the drone **landed with
  2 of 2 targets found and both payloads dropped** (tent 1.7 m, hiker 5.1 m from the target; 73 photos, all checked
  by the detector; 5:22 of flight);
- the link drop: the relay saw SITL silent for 20.0 s, dialed again 4 times, its gate dropped 5620 bytes MAVROS sent
  meanwhile (no late commands), MAVROS reconnected and the mission went on;
- during the drop the team's camera node logged `FAILED: Capture shutter/index error` (the camera was unreachable, as
  on a drone whose camera cable is loose) and the phase view printed a false `motors disarmed` (MAVROS's default
  while it hears no autopilot; a fix for `phase_monitor` was proposed). The hiker target was chosen from a photo taken
  within seconds after the link came back, whose GPS name was 22 m off (likely cause, not proven: a position that was
  still old right after the reconnect): a real effect of a lost link, worth knowing at the field;
- the detector ran **PyTorch on the RTX 4060**, not TensorRT: in that image the TensorRT export failed (`import onnx`
  found Ubuntu's onnx 1.7, too old for numpy 1.26). The image definition was fixed afterwards (onnx 1.14 from the
  team's pins); a flight with the PC's own TensorRT engine is still to be recorded;
- `jetson/summary.md` listed the versions (CUDA 12.6, TensorRT 10.3, cuDNN 9.3 in the image but 9.10 loaded by
  PyTorch, MAVROS 2.14.0 vs the board's 2.12.0) and the differences from the field.

Not verified yet: anything with the real board (the USB-C or SSH link, the owner's login window, a real cable pull,
the arm64 image, the board's TensorRT engine, tegrastats), the new video pipeline in the container, and whether
Mission Planner showed the team's texts during this flight (nobody watched it).

## Not supported yet

- **START OVER** (the demo panel's button / `start_over.sh`) with `--jetson`: run the demo again instead.
- Mirroring gimbal and zoom commands to the REAL camera on the Jetson (HIL_DESIGN.md section 14: later).
