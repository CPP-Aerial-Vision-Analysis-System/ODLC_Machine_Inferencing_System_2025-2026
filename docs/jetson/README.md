<!--
docs/jetson/README.md -- PURPOSE: the owner's and the team's guide to the Jetson Orin Nano kit (simulation/jetson/):
how to plug the Jetson into this PC (USB-C cable) or reach it with no cable (Ethernet, Wi-Fi, LAN, Tailscale), log in,
check it with one command, set it up, build the team-stack container ON THE PC, put it on the Jetson, build the
TensorRT engine once, fly the simulator with the team's software running ON the Jetson, look at it, stop it, update it
after code changes, and fix every error message the kit can print. Written step by step, with copy-paste commands and
"how do I know it worked". The design behind it: docs/jetson/HIL_DESIGN.md. Flying with demo.sh --jetson (the PC
side: link, tap, run folder, summary): docs/jetson/FLYING.md.
-->

# The Jetson kit: plug in, check, deploy, fly

**What this is.** On the real drone the team's software (MAVROS + the mission nodes + the detector) runs on a
**Jetson Orin Nano**. This kit lets you run **that same software, in a Docker container, on the real Jetson**, while
the simulator (Gazebo, the ArduPilot autopilot, the camera emulator, Mission Planner) stays on this PC. The Jetson
then "flies" the simulated drone exactly as it would fly the real one. The same container also runs **on this PC**
as a "virtual Jetson", so you can test everything without the board.

**How the PC reaches the Jetson.** Only by **SSH** (the normal remote login). Either:
- the **USB-C cable** (the Jetson is then always at `192.168.55.1`), or
- **no cable**: any address the PC can SSH to: an Ethernet cable, Wi-Fi, a LAN, or Tailscale (`--host ADDRESS`).

**You type the Jetson password yourself, once, in a window.** The kit never sees, types or stores a password. While
that window stays open, every command reuses that one login. Close the window = all access ends.

**The team comes first.** The kit only uses its own names (`astra-hil`, the folder `~/astra_hil`, the label
`org.astra.owner=hil`), refuses heavy work while the team is using the board, never changes the power mode, and keeps
disk space free for the team.

In every command below, run it in the **Ubuntu (WSL) terminal**, from the repo folder:

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026        # or: cd "/mnt/d/Projects ext/SUAS/gazebo/ODLC_Machine_Inferencing_System_2025-2026"
```

> **No Jetson with you?** Go straight to [section 1b: the virtual Jetson](#1b-no-board-the-virtual-jetson-on-this-pc-3-commands).
> It runs the same container on this PC, with the Jetson's CUDA / TensorRT versions, and flies the same simulator.
> Section 1 is for the day the board is plugged in.

---

## 1. Plug in and test (the first time, about 5 minutes)

### Step 1: plug in

1. Give the Jetson power from its **round barrel jack** (the USB-C port does NOT power it).
2. Take a **USB-C data cable** (a "charge only" cable will not work). Plug one end into the **Jetson's USB-C port**
   and the other end into this laptop.
3. Switch the Jetson on and **wait 1 minute** (it has to finish starting).

**How do I know it worked?** Windows plays the "device connected" sound. In Device Manager, under "Network adapters",
a "Remote NDIS Compatible Device" appears (and under "Ports" a "USB Serial Device (COMn)").

### Step 2: the first check (no login needed yet)

```bash
bash simulation/jetson/jetson.sh check
```

**How do I know it worked?** You see green ` OK` lines for `link up` and `SSH port 22 open`, then a red line
`Not logged in to 192.168.55.1 ... Fix: bash ".../jetson.sh" login`. That is correct: you have not logged in yet.
If you see a red line about the link instead, look it up in [Troubleshooting](#10-troubleshooting-every-message).

### Step 3: log in (a window opens)

```bash
bash simulation/jetson/jetson.sh login --user astra-dev        # your Linux user name on that Jetson
```

1. A **new window** opens: "Jetson login - keep this window open".
2. The first time, ssh shows the Jetson's key fingerprint: type `yes`.
3. **Type the Jetson password** in that window and press Enter.
4. **Leave the window open** (you may minimize it). You can also type commands on the Jetson in it.

**How do I know it worked?** The window shows the Jetson's command prompt, and the first terminal prints
` OK logged in to astra-dev@192.168.55.1`. The user and address are saved, so next time `jetson.sh login` is enough.

### Step 4: the full check

```bash
bash simulation/jetson/jetson.sh check
```

It looks at everything (and changes nothing): the board and its JetPack, Docker, the NVIDIA runtime, "is the team
using the board?", disk, memory, CUDA/TensorRT, a real test of the SSH doors the simulator needs, the power mode, the
clock, and our image and engine.

**How do I know it worked?** The last line is green:
` OK ALL GOOD: the Jetson at 192.168.55.1 is ready for "jetson.sh deploy"`, and nothing above is red.

**A red line `the team is using the Jetson right now` (exit 16) is normal** while the team's own stack runs on the
board (today it runs from boot). The kit will not start anything heavy then. Ask the team for a time when they stop
their stack, then run the check again.

What each line means:

| Row | Line | Good when |
|---|---|---|
| 1 | link up | the Jetson answers (ping or TCP) |
| 2 | SSH port 22 open | SSH answers |
| 3-4 | logged in, host key | your login window is open |
| 5 | board + JetPack | Jetson (aarch64), L4T R36.4.x, Ubuntu 22.04 |
| 6 | Docker without a password | your user may use Docker (docker group, or passwordless sudo) |
| 7 | NVIDIA container runtime | `nvidia` is listed (the container uses the GPU through it) |
| 8 | team busy? | no team programs, GPU idle, no busy containers |
| 9 | disk | enough free space, AND a reserve stays free for the team (default: 100 GB or 25 % of the disk, the bigger) |
| 10 | memory | at least 4 GB free (CPU and GPU share 7.6 GB) |
| 11 | CUDA / TensorRT | the host is L4T R36.4 (the container brings its own CUDA/cuDNN/TensorRT) |
| 12 | SSH forwards allowed | one byte really went PC -> Jetson -> PC through the doors the simulator link uses |
| 13 | power mode and clocks | **report only**: you pick the field mode yourself (the line says how) |
| 14 | clock offset | the Jetson's clock is within 0.5 s of the PC's |
| 15 | registry port | `127.0.0.1:29517` free on the Jetson (used for a moment by deploy) |
| 16 | image, engine | after deploy/warmup: our image is there and matches the PC's code; the engine fits this board |

Extra options: `--quick` (skip the slow rows), `--speed` (measure the link: 50 MB through the login), `--deep` (run
the selftest inside the deployed image), `--for deploy|warmup|flight` (the thresholds for that step), `--pc` (check
this PC for image builds).

---

## 1b. No board: the virtual Jetson on this PC (3 commands)

The "virtual Jetson" is the SAME container, built for this PC (x86) on NVIDIA's TensorRT image, so it has the Jetson's
CUDA 12.6, cuDNN 9.3 and TensorRT 10.3 (section 11). It runs the team's unchanged `./launcher.sh` and
`tracker.launch.xml`, its detector runs a TensorRT engine on the PC's graphics card, and it talks to the simulator
through the same relay as on the board. Only the cable and the Jetson's own GPU are missing.

```bash
bash simulation/jetson/jetson.sh image build --flavor virtual   # 1. build it (first time ~20-30 min, later minutes)
bash simulation/jetson/jetson.sh deploy --link virtual          # 2. model + selftest + this PC's TensorRT engine
bash simulation/scripts/demo.sh runway --jetson virtual         # 3. fly (Gazebo opens; the team stack is in the container)
```

1. **Build** is a heavy job: it waits while a simulator or another build runs, then builds three layers (section 4).
   **How do I know it worked?** The last line is ` OK built astra-hil:virtual (also astra-hil:virtual-<code>; code
   <code>, git <commit>)`. The `<code>` is a fingerprint of the team code and image files that went in.
2. **Deploy** copies `yolo26m.pt` into `~/.astra_sim/jetson_virtual/models/`, runs the selftest inside the image
   (GPU, TensorRT, every team package, the photo folder), then builds THIS PC's own TensorRT engine with the team's
   own export code (a few minutes; the same step is `warmup` on the board). **How do I know it worked?** The selftest
   lines say ` OK`, then `[warmup] done in N s: /models/yolo26m.engine`, then ` OK deployed. Next: ... demo.sh runway
   --jetson virtual`.
3. **Fly.** demo.sh first runs `jetson.sh check --quick --for flight --link virtual`, then starts the simulator, the
   virtual link, the container and the ROS tap, and flies the mission. **How do I know it worked?** The phases appear
   in the terminal as usual (TAKEOFF, SEARCH, ...), the drone lands, and the run folder has `jetson/summary.md` and
   `team_stack.log` (FLYING.md explains both).

**After you change the team's code** (or anything in `simulation/jetson/image/`): run step 1 again (only the changed
layer is rebuilt). If you forget, `check` says ` XX ... [ 16] image ... (the PC has code XXXX)` and demo.sh stops
before anything starts.

Extra: `--jetson-like` on demo.sh (or `jetson.sh start --sim --link virtual --jetson-like`) limits the container to
the board's 6 CPUs and 7.4 GB of memory. The PC's graphics card is still much faster than the Jetson's.

## 1c. The first session with the real board (later): the whole list

Nothing below has run on a real board yet (owner, 15:14: no board for now). Do it in this order the day the board is
plugged in, with the team's OK for steps 5-8 (they use the GPU and disk). Each line says what you should see.

| # | Command | You should see (last line) |
|---|---|---|
| 1 | `bash simulation/jetson/jetson.sh check` | ` OK [  1] link up`, ` OK [  2] SSH port 22 open`, then ` XX ... Not logged in` (normal) |
| 2 | `bash simulation/jetson/jetson.sh login --user astra-dev` | a window opens; type the password there; ` OK logged in to astra-dev@192.168.55.1` |
| 3 | `bash simulation/jetson/jetson.sh check` | ` OK ALL GOOD: the Jetson at 192.168.55.1 is ready for "jetson.sh deploy"` (or ` XX ... team is using the Jetson` = ask the team) |
| 4 | `bash simulation/jetson/jetson.sh setup` | ` OK saved the target ...` |
| 5 | `bash simulation/jetson/jetson.sh image build` (on the PC; 1-2 h the first time) | ` OK built astra-hil:jetson (code <code>, git <commit>) and stored it for deploy` |
| 6 | `bash simulation/jetson/jetson.sh deploy` | selftest lines ` OK ...`, then ` OK deployed. Next: ... warmup` |
| 7 | `bash simulation/jetson/jetson.sh warmup` (answer `y`) | `[warmup] done in N s: /models/yolo26m.engine + /models/yolo26m.engine.meta`, then ` OK engine built: ...` |
| 8 | `bash simulation/jetson/jetson.sh check --for flight` | ` OK ALL GOOD: the Jetson at 192.168.55.1 is ready to fly: bash simulation/scripts/demo.sh runway --jetson usb` |
| 9 | `bash simulation/scripts/demo.sh runway --jetson usb` | the phases in the terminal, the landing, and `RUN_DIR/jetson/summary.md` (FLYING.md) |
| 10 | `bash simulation/jetson/jetson.sh stop`, then close the login window | ` OK stopped` |

If a step prints a red ` XX` line, it says what failed, why and what to type; the same lines are in
[Troubleshooting](#10-troubleshooting-every-message). Without the USB cable, add `--host ADDRESS` to step 1 and 2
(section 2) and fly with `--jetson ssh --jetson-host ADDRESS`.

---

## 2. No USB cable: SSH over Ethernet, Wi-Fi, a LAN or Tailscale

Everything works the same; only the address changes. Give it once with `--host` (it is saved after the login):

```bash
bash simulation/jetson/jetson.sh find                                   # tries the usual addresses and names
bash simulation/jetson/jetson.sh login --host jetson.local --user astra-dev   # or an IP, e.g. --host 192.168.144.20
bash simulation/jetson/jetson.sh check
```

**Finding the address:** on the Jetson (screen + keyboard, or the login window of another link) type
`hostname -I`. Tailscale: the name or `100.x.y.z` address from the Tailscale app.

| Link | What to know |
|---|---|
| **Ethernet cable** PC <-> Jetson | The Jetson's Ethernet is `192.168.144.20` on the board today (check with `ip -4 addr` on it). Give the PC's Ethernet adapter a fixed address in the same network **yourself** (Windows: Settings -> Network -> Ethernet -> IP assignment -> Edit -> Manual -> IPv4 `192.168.144.50`, mask `255.255.255.0`). **Never use `.25`** (that is the camera) and never `.20` / `.100`. Fast and steady: the best choice when USB does not work |
| **Wi-Fi / LAN** | Latency 2-30 ms with spikes; slower photo/stream transfers. **Campus Wi-Fi often blocks device-to-device traffic** ("client isolation"): then SSH itself fails: use the USB cable, an Ethernet cable, a phone hotspot or Tailscale. Addresses on campus Wi-Fi change: run `find` or `hostname -I` again. Firewalls: only TCP port 22 on the Jetson is needed; nothing comes IN to the PC |
| **Tailscale** | Works from anywhere with internet; slower (the internet round trip). The PC and the Jetson must both be logged in to the same tailnet |

The kit only ever opens connections FROM the PC (WSL sits behind Windows like a home network behind a router and
cannot accept incoming ones).

---

## 3. One-time setup on the Jetson

```bash
bash simulation/jetson/jetson.sh setup
```

It creates **only our own folder** `~/astra_hil/` on the Jetson (sockets, models, photos, runs, logs), installs
`~/astra_hil/bin/astra_hil.sh` (to run the container on the drone without the PC), reads which ROS domains the board
uses, and saves the target on the PC (`~/.astra_sim/jetson/target.env`).

**Docker without a password.** If check row 6 is red, the owner and the team decide (the `docker` group gives root
rights on the board), and the owner types in the login window: `sudo usermod -aG docker astra-dev`, then closes the
login window and logs in again. The kit never types sudo passwords.

**Key login (optional).** `jetson.sh setup --key` adds ONE line (comment `astra-hil-kit@...`) to the Jetson user's
`~/.ssh/authorized_keys` (the owner and the team must agree), so the link can reconnect by itself after a drop. Undo
with `jetson.sh setup --remove-key`. Without it, everything works through your login window.

**How do I know it worked?** ` OK saved the target ...` and `jetson.sh check` shows no red line about the folder.

---

## 4. Build the container image (on THIS PC, not on the Jetson)

```bash
bash simulation/jetson/jetson.sh image build                    # the Jetson image (arm64), first time 1-2 hours
bash simulation/jetson/jetson.sh image build --flavor virtual   # the PC's "virtual Jetson" image (x86)
```

- The Jetson image is built on the PC through emulation (the build makes sure arm64 emulation works first). Nothing
  heavy runs on the board. It is a **heavy job**: it waits until no simulator or other build runs.
- The image has three layers: (1) NVIDIA's L4T JetPack container (CUDA 12.6, cuDNN 9.3, TensorRT 10.3) + ROS 2 Humble,
  (2) the team's dev stack from `docker/Dockerfile` (MAVROS, Python packages, torch for the Jetson), (3) the team's
  code built in (`launcher.sh` unchanged, the team packages, `/opt/astra` helpers). The PC image uses the SAME layers
  2 and 3 on a PC base with the same CUDA/cuDNN/TensorRT versions (`nvcr.io/nvidia/tensorrt:24.08-py3`); add
  `--x86-base ros` to build on the plain ROS image instead.
- When it is done, the Jetson image is stored in a small image store on the PC (`127.0.0.1:29517`) for deploy.

**How do I know it worked?** ` OK built astra-hil:jetson (code <code>, git <commit>) and stored it for deploy`.
`jetson.sh image status` lists the images.

**The code fingerprint.** Every image carries a label `org.astra.code_sha`: a 12-character fingerprint of the exact
files that went into it (the team packages of `image/context.list`, `launcher.sh`, the files of
`simulation/jetson/image/`, the relay and the tap). `check` computes the same fingerprint from the code on the PC and
says when the image is older. It works with uncommitted changes and in the Linux copy (which has no git history),
where a git commit could not tell.

---

## 5. Put it on the Jetson (deploy)

```bash
bash simulation/jetson/jetson.sh deploy
```

- The Jetson pulls the image from the PC **through your login** (only changed layers move; the Jetson needs no
  internet). If the SSH doors are not allowed (check row 12), use `deploy --via save` (copies the whole image).
- It copies the model weights (`--model FILE`, else the team's own `~/astra/ros2_ws/src/yolo26m.pt` on the Jetson, read
  only, else this PC's `~/.astra_sim/models/yolo26m.pt`), then runs a **selftest** inside the image (GPU, TensorRT,
  packages, photo folder, model).
- It refuses while the team is using the board (exit 16), and when the disk reserve would be crossed (exit 10).

**How do I know it worked?** The selftest lines are ` OK`, and the last line is
` OK deployed. Next: ... warmup`.

## 6. Build the TensorRT engine on the Jetson (once)

```bash
bash simulation/jetson/jetson.sh warmup
```

It asks first ("It uses the Jetson's GPU for several minutes. The team must not be using the board. [y/N]"). An
engine belongs to ONE board, ONE TensorRT version and ONE model file, so it is built ON the board, once per board /
JetPack / model, with the team's own code and settings (the same engine new_od would build at its first start). A
flight never builds it: `start` refuses without a matching engine (exit 19) when the legacy detector needs it.

**How do I know it worked?** ` OK engine built: ~/astra_hil/models/yolo26m.engine (+ .meta)`. The `.meta` file
records the TensorRT version, the L4T release, the GPU, the model's checksum and the time. Run it again after a
JetPack update or a new model: it rebuilds only when one of them changed.

**The virtual Jetson** builds its own engine the same way, on the PC's graphics card (`deploy --link virtual` does it,
or `jetson.sh warmup --link virtual`; no question asked, nobody to disturb). Its `.meta` says `L4T=pc` and the PC's
GPU; an engine from the board is never used on the PC, or the other way round.

---

## 7. Fly

```bash
bash simulation/scripts/demo.sh runway --jetson usb                                  # USB-C cable
bash simulation/scripts/demo.sh runway --jetson ssh --jetson-host 192.168.144.20     # no cable
bash simulation/scripts/demo.sh runway --jetson virtual                              # no board: the PC container
```

demo.sh checks the Jetson first, starts the simulator, opens the link, starts the container on the Jetson, flies,
collects everything into the run folder and writes `RUN_DIR/jetson/summary.md`. The whole flight side (link drops,
Wi-Fi photo modes, the run folder, the summary) is in **[FLYING.md](FLYING.md)**.

## 8. Look, stop, run commands inside

```bash
bash simulation/jetson/jetson.sh status            # logged in? container? image, engine, power mode, memory, disk
bash simulation/jetson/jetson.sh status --json     # the same for programs
bash simulation/jetson/jetson.sh logs -f           # the container's output (Ctrl+C to stop watching)
bash simulation/jetson/jetson.sh logs --run 20261031-120000_runway   # one run's team_stack.log
bash simulation/jetson/jetson.sh exec -- ros2 node list               # one command inside, as the stack's user
bash simulation/jetson/jetson.sh shell                                 # a terminal inside the container
bash simulation/jetson/jetson.sh stop                                  # stop our containers (only ours)
bash simulation/jetson/jetson.sh clock                                 # offset_s=... rtt_ms=...
bash simulation/jetson/jetson.sh logout                                # end the login (= closing the window)
```

`exec` and `shell` always run as the stack's own user (`astra-dev`): ROS commands run as root cannot see the team's
nodes. `shell --root` exists for files and network only.

Starting by hand (demo.sh normally does this): `jetson.sh start --sim --run-id ID` (Jetson in the loop) or
`jetson.sh start --real` (on the drone; see section 9). `--dry-run` prints the exact `docker run` line.

## 9. After code changes, and on the drone

**After you change the team's code:** `jetson.sh image build` (only layer 3 is rebuilt, about 10-20 minutes under
emulation), then `jetson.sh deploy` (only the changed layer moves). `check --for flight` tells you when the image on
the Jetson is older than the PC's code (it compares the code fingerprint of section 4, not the git commit).

**REAL mode (the drone, no simulator):** `jetson.sh start --real` from the PC, or on the Jetson itself
`~/astra_hil/bin/astra_hil.sh start real`. It runs the same container with the drone's settings: host network,
MAVROS on `/dev/ttyUSB0`, the real camera on Ethernet, the team's own photo folders, the team's ROS domain. It refuses
while the team's native stack runs (both would open `/dev/ttyUSB0`) and when the photo folder has old photos (add
`--archive-photos` or `--keep-photos`). The native `./launcher.sh` without Docker keeps working exactly as before.

---

## 10. Troubleshooting (every message)

Every error line looks like ` XX [jetson] ERROR: WHAT. Likely cause: WHY. Fix: WHAT TO TYPE`. The exit code tells
programs what happened.

| Exit | Message starts with | What to do |
|---|---|---|
| 2 | `unknown command` / `unknown option` / `unknown link` / `--link ssh needs the Jetson's address` / `--port must be a number` | a typing mistake: `bash simulation/jetson/jetson.sh --help` |
| 3 | `link (check row 1): no answer from 192.168.55.1 and Windows has NO USB network adapter` | USB-C **data** cable into the Jetson's USB-C port, power from the barrel jack, wait 1 minute. Still nothing: Device Manager -> "Other devices" -> the unknown device -> Update driver -> Browse my computer -> Let me pick -> Network adapters -> Microsoft -> "Remote NDIS Compatible Device" (the owner does this) |
| 3 | `link (check row 1): Windows reaches 192.168.55.1 but this Ubuntu (WSL) does not` | WSL's network does not route to the USB adapter (design risk 1). Tell the lead; meanwhile use Ethernet/Wi-Fi with `--host` |
| 3 | `link (check row 1): cannot reach HOST (no answer to ping or TCP 22)` | wrong address, Jetson off, or the Wi-Fi blocks devices: `jetson.sh find`, `hostname -I` on the Jetson, USB/Ethernet/hotspot/Tailscale |
| 4 | `SSH port (check row 2): HOST answers but its SSH port 22 is closed` | wait 1 minute; on the Jetson: `sudo systemctl enable --now ssh` |
| 17 | `Not logged in to USER@HOST` | `bash simulation/jetson/jetson.sh login` (a window opens; type the password; keep it open) |
| 17 | `The open login window is logged in to X, not to Y` | close that window (or `jetson.sh logout`), then `jetson.sh login --host Y` |
| 17 | `No login to USER@HOST within 120 s` | finish the login in the window (it says why it failed), then `jetson.sh check` |
| 5 | `SSH login refused for USER@HOST with the key` | key mode only: `jetson.sh login` then `jetson.sh setup --key`, or use `--auth login` |
| - | login window: `The Jetson's key changed` | another board at that address or a re-flash: `ssh-keygen -f ~/.astra_sim/jetson/known_hosts -R HOST`, then log in again |
| - | login window: `Wrong user name or password` / `does NOT accept passwords` | check the user (`whoami` on the Jetson) and the password; or the team allows password login |
| - | login window: `The connection to the Jetson DROPPED` | check the cable/network, press Enter, type the password. A running flight keeps flying |
| 7 | `[  5] board + JetPack: ... L4T R35...` | the image needs JetPack 6.2.x (L4T R36.4): re-flash, or rebuild on another base tag |
| 8 | `[  6] Docker without a password: ... cannot use it without a password` | owner/team decide; owner types `sudo usermod -aG docker USER` in the login window, closes it, logs in again |
| 8 | `[  6] Docker: docker is not installed` | owner installs it: `sudo apt-get install docker.io nvidia-container` |
| 9 | `[  7] NVIDIA container runtime: runtimes: runc` | owner types: `sudo apt-get install nvidia-container` then `sudo systemctl restart docker` |
| 9 | `selftest failed inside the image` | `jetson.sh check --deep`; the selftest line marked FAIL names the item |
| 10 | `[  9] disk ...: using this space would leave less than the reserve` | `jetson.sh image prune` (only OUR images); never `docker system prune`; or `JETSON_DISK_RESERVE_GB=N` |
| 11 | `[ 10] memory: ... too little memory` | close programs on the board, or headless for the test (owner: `sudo systemctl isolate multi-user.target`) |
| 12 | `[ 16] image: astra-hil:jetson is not on ... yet` / `There is no astra-hil:jetson image on this PC yet` | `jetson.sh image build`, then `jetson.sh deploy` |
| 12 | `[ 16] image: ... (the PC has code XXXX)` / `... older (or newer) than the code on the PC` | the team code or the image files changed since the build: `jetson.sh image build`, then `jetson.sh deploy` (virtual: `image build --flavor virtual`, then `deploy --link virtual`) |
| 12 | `There is no model in ~/astra_hil/models` | `jetson.sh deploy` (or `deploy --model FILE`) |
| 12 | `There is no model in ~/.astra_sim/jetson_virtual/models` (virtual warmup) | `jetson.sh deploy --link virtual --model FILE` (or put `yolo26m.pt` in `~/.astra_sim/models/`: `03_build.sh` downloads it) |
| - | warning `the image on this PC was built from code X, the code here is Y` (deploy) | the Jetson would get older code: `jetson.sh image build` first |
| 13 | `the container astra-hil is not running` | `jetson.sh start --sim`, or read `jetson.sh logs` |
| 13 | `the container ... stopped right after the start` | its last lines are printed: the entrypoint says why (see the entrypoint rows below) |
| 13 | `the container runs, but its relay did not say LISTEN=yes` | `jetson.sh logs`; `jetson.sh exec -- cat /data/runs/ID/relay.log` |
| 1 | `Our container astra-hil is already running` | `jetson.sh stop`, then start again |
| 1 | `the team's photo folder ... is not empty` (REAL mode) | `--archive-photos` (moves them to `~/astra_hil/photo_archive/`) or `--keep-photos` |
| 14 | `SSH forwards (check row 12): the forward test failed` | the Jetson's sshd forbids forwarding (`AllowStreamLocalForwarding` / `AllowTcpForwarding` / `DisableForwarding`): owner/team decide; deploy can use `--via save` |
| 14 | `Could not open the registry door (-R 127.0.0.1:29517)` | `deploy --registry-port N` or `deploy --via save` |
| 14 | `Could not read the facts through the login` / `The Jetson sent no facts` | the login dropped: `jetson.sh login`, then again |
| 15 | `team stack not ready` (demo.sh) | `jetson.sh logs`, `RUN_DIR/team_stack.log` (see FLYING.md) |
| 16 | `the team is using the Jetson right now (found: ...)` | ask the team; when they agree and have stopped, run again. Only with the owner's and the team's agreement: `--team-agreed` |
| 18 | `Docker Engine is not running in this Ubuntu (WSL)` / `docker talks to Docker Desktop` | `bash simulation/docker/install_docker_engine.sh` (never Docker Desktop: its disk is on C:) |
| 18 | `Docker's build tool buildx is missing` | `sudo apt-get install docker-buildx-plugin` |
| 18 | `arm64 emulation still does not work` | permanent fix (owner's choice): `sudo apt-get install qemu-user-static binfmt-support` |
| 18 | `Could not start the image store` / `Could not push into the image store` | `docker logs astra-hil-registry`; port in use: `ss -ltn \| grep 29517` |
| 19 | `[16b] TensorRT engine: yolo26m.engine missing / built for L4T ... / another model file` | `jetson.sh warmup` (once; the team must not be using the board) |
| 19 | `[16b] TensorRT engine: ... built for another GPU (...)` or `missing` with `--link virtual` | the virtual Jetson needs ITS OWN engine: `jetson.sh warmup --link virtual` (a few minutes on the PC's GPU) |
| 19 | `The engine build failed` | not enough memory, or the TensorRT export failed: read the lines above; team notes `ros2_ws/src/fixtensorRT.md` |
| 19 | `The engine build failed on this PC` (virtual) | `jetson.sh check --link virtual --deep` (is the GPU visible in Docker?); read the `[warmup]` lines above |
| 1 | `Layer 1 / 2 / 3 ... did not build` | read the build error above it; layer 1 needs nvcr.io + packages.ros.org; layer 3 = a team package does not build with colcon |
| 1 | build: `TENSORRT=yes but 'import tensorrt' fails in this image` | layer 1 is not the TensorRT/L4T base: build without `--x86-base ros` (the default is the TensorRT base) |
| - | entrypoint note `no TensorRT engine in /models: the team's new_od builds it ... at its first start` | not an error: the first start takes several minutes longer. Build it beforehand: `jetson.sh warmup` (virtual: `--link virtual`) |
| - | entrypoint note `the new video pipeline is not in this image yet ... the LEGACY (2025) photo pipeline runs` | expected until the video workflow delivers `simulation/video/proposals/` and the zr10 packages; then rebuild the image |
| 20 | entrypoint: `refusing to add 192.168.144.25: this container has network interfaces besides lo` | the container was started with `--network host` by hand: use `jetson.sh start --sim` (protects the real camera on the drone) |
| 21 | entrypoint: `could not add 192.168.144.25 to the container's own loopback` | missing `--cap-add NET_ADMIN`: use `jetson.sh start --sim` |
| 22 | entrypoint: `the socket folder /run/astra_hil is not mounted` | started by hand: use `jetson.sh start --sim` |
| 24 | entrypoint: `--pipeline new, but this image has no proposed field launch file` | rebuild the image after the video workflow delivered `simulation/video/proposals/`, or `--pipeline legacy` |
| 3/4 | relay: `cannot listen` / `the socket folder ... does not exist` | the container was started by hand or twice: `jetson.sh stop`, `jetson.sh start --sim` |

Warnings (` !! [jetson] WARNING: ...`) never stop anything; each says the consequence and the fix (clock offset,
memory below 4 GB, power mode unknown without sudo, image older than the code, team busy with `--team-agreed`).

---

## 11. PC vs Jetson: versions (owner, 12:32: "the Jetson's software, version for version")

ONE image definition, two base layers; everything above the base is identical. What each machine really has is
printed by `jetson.sh deploy` (selftest) and recorded in every run summary.

| Item | Jetson (board, live probe 2026-10-31 11:47) | Jetson image `astra-hil:jetson` (arm64) | PC image `astra-hil:virtual` (x86, `--x86-base trt`) |
|---|---|---|---|
| OS | Ubuntu 22.04.5 | Ubuntu 22.04 (l4t-jetpack r36.4.0) | Ubuntu 22.04.4 LTS (tensorrt 24.08) |
| L4T / driver | R36.4.7 | uses the host's driver | the PC's NVIDIA driver (RTX 4060 Laptop GPU) |
| CUDA | 12.6.11 | 12.6 (inside the image) | 12.6 (`cuda-cudart-12-6 12.6.37-1`) |
| cuDNN | 9.3.0 | 9.3 (inside the image) | 9.3.0 (`libcudnn9-cuda-12 9.3.0.75-1`); PyTorch's own wheel brings cuDNN 9.10.2 |
| TensorRT | 10.3.0 (+ Python) | 10.3 (inside the image) | 10.3.0 (`libnvinfer10 10.3.0.26-1+cuda12.5`, Python `tensorrt 10.3.0`) |
| ROS 2 | Humble ros-base | Humble ros-base | Humble ros-base |
| MAVROS | 2.12.0 (native) | 2.14.0 (snapshot 2026-08-07; 2.12.0 is in no apt snapshot) | 2.14.0 (`ros-humble-mavros 2.14.0-1jammy.20260804`, `ros-humble-mavlink 2026.9.9`) |
| PyTorch | in the team's user site (new_od runs natively) | 2.8.0 Jetson wheel (pinned by sha256, `image/jetson_pins.env`) | 2.8.0+cu126 (CUDA 12.6 wheel) |
| numpy / OpenCV / Ultralytics / SAHI | 1.26.4 / 4.10.0.84 / 8.4.75 / 0.11.36 (team pins) | the same pins | 1.26.4 / 4.10.0 / 8.4.75 / 0.11.36 |
| ONNX / protobuf (TensorRT export) | not recorded | 1.14.0 / 3.20.3 (team pins; EXPECTED, arm64 not built yet) | 1.14.0 / 3.20.3 |
| Python | 3.10.12 | 3.10 | 3.10.12 |

The PC column was VERIFIED on 2026-10-31 inside the built `astra-hil:virtual` (code `bba6e6131dc9`): the versions
step of `D:\suas_scratch\e2e\jet_accept.sh` and the deploy selftest. The Jetson-image column is the plan (the arm64
image has not been built yet). The PC's TensorRT engine was built in 267 s on the RTX 4060 by `deploy --link virtual`
(`.meta`: `TRT=10.3.0 L4T=pc GPU=NVIDIA GeForce RTX 4060 Laptop GPU MAX_BATCH=2 SLICE=640`).

Performance is proven only on the real Jetson (the RTX 4060 is much faster, and an engine is optimised per GPU).
`jetson.sh start --sim --link virtual --jetson-like` gives the PC container the board's 6 CPUs and 7.4 GB memory.

## 12. Differences from the field (also in every run summary)

The Jetson-in-the-loop flight runs the team's software as at the field. What still differs:

1. **The autopilot is ArduCopter 4.6.3 SITL on the PC**, not the Pixhawk (same firmware version; simulated sensors).
2. **MAVLink goes over TCP through SSH**, not the TELEM2 serial cable (`FCU_URL=tcp://127.0.0.1:5763` instead of
   `/dev/ttyUSB0:115200`; launcher.sh runs in the drone's own `hw` mode). No 115200-baud limit unless demo.sh turns
   on SITL's limiter (FLYING.md).
3. **Link glitches behave differently**: a broken serial cable loses bytes; through SSH, bytes can arrive up to ~6 s
   late if the connection recovers by itself. The relay's gate drops what MAVROS sends while the autopilot is silent.
4. **The camera is the emulator on the PC**, reached through the relay (real SIYI protocol, real RTSP stream / full-size
   photos), so latency and download times differ from the real camera's.
5. **Camera pipeline**: the new video pipeline runs through the video workflow's PROPOSED field launch file, which the
   team has not adopted yet; until that file and the zr10 packages are in the image, the container runs the
   **legacy (2025) photo pipeline** and says so in its log.
6. **ROS domain 77 with `ROS_LOCALHOST_ONLY=1`** instead of the field's domain 0 (the team code does not depend on it).
7. **The Jetson's clock** may differ from the PC's; photo file times are corrected, sidecar contents are not.
8. **The board's setup** may differ from flight day (desktop running, power mode, other programs, the container's
   5 GB / 5 CPU limits): recorded in the run summary.
9. **Files go to our own folder** (`~/astra_hil/photos`), never the team's `mapping_photos`, in SIM mode.
10. **The ROS graph is sealed inside the container** (`--network none`); the detector container joins it instead of
    using `--network host`.
11. **How detections reach main_controller** with the new pipeline is not decided by the team yet (proposal only).
12. **MAVROS 2.14.0 in the container vs 2.12.0 natively on the board** (no apt source has 2.12.0; between them
    mavros_msgs only gained one new message).
13. **On the PC (virtual Jetson)**: x86 + RTX 4060 instead of the Orin GPU. The detector runs a TensorRT 10.3 engine
    built on the PC's GPU from the same model file with the team's own export code (FP16, dynamic batch 2, like the
    board), so its speed and exact numbers differ from the Orin's engine. PyTorch in the PC image brings its own cuDNN
    (pip wheel), next to the system cuDNN 9.3 that TensorRT uses. No USB link, no real cable pull. With
    `--x86-base ros` (no TensorRT) the detector runs the `.pt` model on PyTorch instead.
14. **MAVROS version**: exactly 2.12.0 would need MAVROS built from its source tag inside the image (no apt
    repository or snapshot carries 2.12.0: the snapshots jump from 2.10.1 to 2.14.0, checked 2026-10-31). Not done;
    the owner decides whether it is worth a longer build.

## 13. What is proven, and what only the board can prove

| Proven without the board (2026-10-31) | Only the real board can prove |
|---|---|
| every `check` row and its exit code against a fake Jetson (`test/test_jetson_cli.sh`: 50 checks, including a real forward test through socket files and the virtual Jetson's setup/warmup) | the USB-C network on Windows 11 and WSL's route to `192.168.55.1` |
| the version-matched PC image (TensorRT 10.3 / CUDA 12.6 / cuDNN 9.3, Ubuntu 22.04) built, its selftest all OK, the PC's own TensorRT engine built by `deploy --link virtual` (267 s), `check --for flight --link virtual` ALL GOOD, and in the container the team's unchanged `tracker.launch.xml` with new_od printing `TensorRT model loaded (batched path, SAHI bypassed)` (2026-10-31 16:04, `D:\suas_scratch\e2e\jet_accept.log`) | the arm64 image built and running on the Orin, its `import tensorrt` and engine build there |
| the judging of the facts with the owner's board as recorded (`test/test_jetson_facts.py`) | the owner's login window against the real sshd; sudo; the docker group |
| the relay rules: frames, PING, client ids, HTTP, sticky MAVLink, idle re-dial, the gate, a virtual blackhole drop (`test/test_hil_relay.py`) | the arm64 image running (built through emulation; not yet built), `import tensorrt` on the Orin |
| the PC image built and started as the virtual Jetson (see section 11 / the build log) | the TensorRT engine build, detector speed, memory pressure in 7.6 GB, power mode and heat |
| | real Wi-Fi / USB throughput and latency, a pulled cable, the Jetson's clock |

## 14. Files

| File | What it is |
|---|---|
| `simulation/jetson/jetson.sh` | the command for everything above |
| `simulation/jetson/lib/target.sh` | "which Jetson and how to reach it" (options, saved target, the one SSH option set; the PC-side scripts use it too) |
| `simulation/jetson/lib/login_window.sh` | the visible login window |
| `simulation/jetson/lib/remote_facts.sh`, `lib/jetson_facts.py` | the read-only facts of the board, and their judge (check) |
| `simulation/jetson/lib/hil_relay.py` | the byte relays (MAVLink, SIYI SDK, HTTP, RTSP; container and PC ends; the virtual link) |
| `simulation/jetson/lib/astra_hil.sh` | installed on the Jetson by setup: REAL mode without the PC |
| `simulation/jetson/image/` | the image: `Dockerfile` (layer 3), `Dockerfile.l4t-ros` (layer 1, arm64 and x86), `entrypoint.sh`, `in_stack.sh`, `selftest.py`, `warmup.py`, `context.list`, `jetson_pins.env` |
| `simulation/jetson/test/` | the tests (run without a board): `test_jetson_cli.sh`, `test_jetson_facts.py`, `test_hil_relay.py`, `fake/` |
| `simulation/jetson/pc/` | the PC side of a flight (link, tap, collect, demo.sh steps): see FLYING.md |
| `~/.astra_sim/jetson/` (PC) | `target.env`, `known_hosts`, `login.env`, `last_facts.env`, build contexts |
| `~/astra_hil/` (Jetson) | our only folder on the board |

Run the tests:

```bash
bash simulation/jetson/test/test_jetson_cli.sh
/usr/bin/python3 -m pytest -q simulation/jetson/test/          # in WSL (the relay tests need Linux)
```
