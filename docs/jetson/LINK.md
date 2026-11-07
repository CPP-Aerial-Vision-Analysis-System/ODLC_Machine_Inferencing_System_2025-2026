<!--
docs/jetson/LINK.md -- PURPOSE: the link between this PC and the Jetson Orin Nano, explained simply: what the link is,
how to connect with the USB-C cable, how to connect WITHOUT it (Ethernet, Wi-Fi, a LAN, Tailscale), what changes on
Wi-Fi, what simulation/jetson/pc/link.sh does during a flight and when the link drops, and a troubleshooting table
(each problem: what you see, why, the exact fix). Written for the owner and teammates, step by step. The design behind
it is docs/jetson/HIL_DESIGN.md (sections 4, 5, 6, 9.4 and 10); the first-connection kit (jetson.sh login / check /
setup) is described in docs/jetson/README.md; flying with the Jetson is docs/jetson/FLYING.md.
-->

# The link to the Jetson (USB cable, or just SSH)

**In one sentence:** the Jetson is "an address this PC can SSH to". The USB-C cable is one way to get such an
address (the Jetson is then `192.168.55.1`); an Ethernet cable, Wi-Fi, the lab network or Tailscale work exactly the
same. Every Jetson command takes the address the same way, so nothing else changes.

**No Jetson connected today?** Use the **virtual Jetson**: the same container on this PC, with a local stand-in for
the link (no cable, no SSH, no password). It tests everything in this guide except the cable itself:

```bash
bash simulation/jetson/jetson.sh image build --flavor virtual    # once (long the first time; later only what changed)
bash simulation/jetson/jetson.sh deploy --link virtual           # the model file + a self-test of the image
bash simulation/scripts/demo.sh runway --jetson virtual          # the full visible flight
```

How do I know it worked? The demo prints ` OK virtual link up: socket files in ...jetson_virtual/sock`, then
` OK the team stack on the Jetson is ready (virtual)`, and at the end `JETSON RUN SUMMARY: .../jetson/summary.md`.
The link log is `<run folder>/jetson/link.log`, exactly as with the board. The steps with the real board (section 2
or 3) stay the same; only `--jetson virtual` becomes `--jetson usb` or `--jetson ssh --jetson-host HOST`.

All commands below are typed in the **Ubuntu (WSL) terminal**, in the repo folder:

```bash
cd ~/ODLC_Machine_Inferencing_System_2025-2026      # (or wherever your Linux copy of the repo is)
J=simulation/jetson                                 # short name used below
```

## 1. The picture

```
 THIS PC (Windows 11 + Ubuntu in WSL)                          JETSON ORIN NANO
 ┌───────────────────────────────────────┐                     ┌──────────────────────────────┐
 │ Gazebo, ArduCopter SITL, camera        │   ONE SSH login     │ container "astra-hil":       │
 │ emulator, Mission Planner              │ ==================> │   MAVROS + the team nodes    │
 │                                        │  (TCP port 22 on    │   + the detector (GPU)       │
 │ link.sh adds "doors" to that login:    │   the Jetson; the   │                              │
 │   door A: autopilot (SITL SERIAL2)     │   PC always dials)  │ it finds "the autopilot" at  │
 │           + the read-only ROS tap back │                     │ 127.0.0.1:5763 and "the      │
 │   door B: camera (SDK, HTTP, RTSP)     │                     │ camera" at 192.168.144.25,   │
 └───────────────────────────────────────┘                     │ exactly like on the drone    │
                                                                └──────────────────────────────┘
```

- **The PC opens every connection.** Ubuntu in WSL sits behind Windows like a home network behind a router: it can
  call out, but nobody can call in without changing Windows settings. So the PC dials the Jetson's SSH port (22),
  and everything else rides inside that one connection.
- **Only one port on the Jetson is used: SSH (22).** Nothing new listens on the Jetson's network, and nothing on the
  PC has to accept connections from outside.
- **The Jetson's own network settings are never changed.** The camera address `192.168.144.25` exists only inside
  our container.
- **You log in yourself.** `jetson.sh login` opens a window; you type the Jetson password there; keep the window
  open. All tools reuse that one login and never see your password. Closing the window ends all access.

## 2. Connect with the USB-C cable (`--jetson usb`, the default)

**Step 1 - plug in.**
1. Power the Jetson from its **barrel jack** (the round power plug). The USB-C port does not power it.
2. Plug a **USB-C data cable** into the Jetson's **USB-C port** (next to the barrel jack, not one of the USB-A ports)
   and into the PC. Some cheap cables only charge: if nothing happens in step 2, try another cable.
3. Wait about **one minute** after the Jetson has started.

**Step 2 - does Windows see it?** In Windows PowerShell (copy-paste, it only reads):

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File simulation\jetson\pc\windows_link_check.ps1 -Target 192.168.55.1
```

How do I know it worked? You see `USB_ADAPTER=yes`, `USB_ADAPTER_IP=192.168.55.100`, `PING=ok` and `TCP22=ok`.
The last line (`HINT=...`) always says what to do next. In Device Manager the Jetson shows as a network adapter
"Remote NDIS Compatible Device" and a "USB Serial Device (COMn)" under Ports.

**Step 3 - does Ubuntu see it?** In the Ubuntu terminal:

```bash
ping -c 3 192.168.55.1
```

How do I know it worked? Three lines `64 bytes from 192.168.55.1 ...`. (If Windows reached it in step 2 but Ubuntu
does not, see "Windows reaches it but Ubuntu does not" in section 7.)

**Step 4 - log in and check:**

```bash
bash $J/jetson.sh login        # a window opens: type the Jetson user name (e.g. astra-dev) and password there; keep it open
bash $J/jetson.sh check        # one line per step, OK / WARN / FAIL, each FAIL with its fix
```

How do I know it worked? The last line of `check` says `ALL GOOD` and the exit code is 0 (`echo $?` prints `0`).

## 3. Connect WITHOUT the USB cable (`--jetson ssh --jetson-host HOST`)

Everything is the same; you only give the Jetson's address. Any of these works:

| Way | The address to use | What to do first |
|---|---|---|
| Ethernet cable, PC <-> Jetson | the Jetson's Ethernet address (on the board today `192.168.144.20`; check on the Jetson with `ip -4 addr`) | give the PC's Ethernet adapter a fixed address in the same network (below) |
| Wi-Fi or the lab LAN | the Jetson's IP (e.g. `10.0.0.42`) or its name (e.g. `suas-jetson.local`) | both on the same network; find the address (below) |
| Tailscale | the Tailscale name or `100.x.y.z` address (the owner's guide: `suas-jetson`) | Tailscale logged in on both |

Then, for example:

```bash
bash $J/jetson.sh login --host 192.168.144.20      # the window asks for the user name and password
bash $J/jetson.sh check --host 192.168.144.20
bash simulation/scripts/demo.sh runway --jetson ssh --jetson-host 192.168.144.20
```

After the first successful `login`/`setup` the address is remembered (`~/.astra_sim/jetson/target.env`), so later
commands need no `--host`.

**Finding the address.**
- `bash $J/jetson.sh find` tries the saved address, `192.168.55.1`, `<name>.local` and Tailscale for you.
- On the Jetson itself (screen + keyboard, or the USB serial console): `hostname -I` prints its addresses.
- Your router's or hotspot's list of connected devices.
- `.local` names work only when both are on the same network and the network allows it (not tested on this PC yet).

**Ethernet cable: the PC's fixed address (the owner does this on Windows; we never change Windows settings).**
1. Windows Settings -> Network & internet -> Ethernet -> (the adapter the cable is in).
2. IP assignment -> Edit -> **Manual** -> IPv4 **On**.
3. IP address `192.168.144.50`, Subnet mask `255.255.255.0` (or prefix length `24`), Gateway empty -> Save.
4. **Never** use `.25` (that is the camera's address), `.20` or `.100` (the Jetson's).
5. To undo: the same page, IP assignment -> **Automatic (DHCP)**.

How do I know it worked? `ping -c 3 192.168.144.20` from Ubuntu answers.

## 4. What changes on Wi-Fi

| | USB-C cable | Ethernet cable | Good 5 GHz Wi-Fi | Busy / 2.4 GHz / campus Wi-Fi |
|---|---|---|---|---|
| Speed (EXPECTED; `jetson.sh check --speed` measures it) | 100-300 Mbit/s | ~900 Mbit/s | 50-300 Mbit/s | 5-40 Mbit/s |
| Delay | < 2 ms | < 1 ms | 2-10 ms, spikes 50-200 ms | 5-30 ms, spikes > 200 ms |
| Verdict | plenty | plenty | fine; photos are paced | often too slow for live photos |

- **The autopilot link needs very little** (a few kbit/s: the team code asks for 1 message per second per stream).
- **The camera is the big user.** The new video pipeline's stream (1280x720 at 2 frames per second) needs only a few
  Mbit/s (EXPECTED, not measured yet). The legacy photo pipeline downloads one photo per second: ~11 Mbit/s with the
  ZR10's 2K photos (measured sizes 1.3 MiB each), ~19 Mbit/s with the A8 mini's 4K photos.
- **In login mode everything shares one connection**, so a photo can sit in front of autopilot messages. Two things
  keep that short: `--photo-pace auto` (photo downloads use at most 60 % of the measured speed, never more than
  80 Mbit/s, never less than 1.5 x what the camera needs) and a speed limit on the live photo copy. Key mode (two
  connections) avoids it completely: see the Jetson guide, `jetson.sh setup --key`.
- **Slow Wi-Fi:** fly with `--jetson-photos after` (photos are copied after the landing instead of during the flight).
- **Firewalls:** only TCP port 22 on the Jetson must be reachable. Nothing on the PC needs to be opened.
- **Campus Wi-Fi often blocks device-to-device traffic** ("client isolation"): then even `ping` fails. Use a phone
  hotspot, Tailscale, or an Ethernet cable.
- A dropped Wi-Fi is handled like a pulled cable (section 6).

## 5. What `link.sh` does during a flight

You normally never call it: `demo.sh --jetson ...` starts and stops it. By hand:

```bash
bash $J/pc/link.sh start  --link usb          # or --link ssh --host HOST, or --link virtual
bash $J/pc/link.sh status --link usb          # A (autopilot + tap) and B (camera): up/down, drops so far
bash $J/pc/link.sh stop   --link usb          # removes the doors; never closes your login window
```

The doors it adds to your login (all inside the one SSH connection the PC opened):

| Door | On the Jetson (socket file in `~/astra_hil/sock/`) | On the PC (inside Ubuntu only) | What uses it |
|---|---|---|---|
| A | `mavlink.sock` | SITL SERIAL2, `127.0.0.1:5763` (+10 per `--sitl-instance`) | MAVROS (the Pixhawk's TELEM2 port on the drone) |
| A | `tap.sock` (made by the tap in the container) | `127.0.0.1:7461` | the read-only ROS tap back to the PC |
| B | `cam_sdk.sock` | `127.0.0.1:37261` -> camera emulator UDP 37260 | gimbal, zoom, shutter (SIYI SDK) |
| B | `cam_http.sock` | `127.0.0.1:37282` -> camera emulator TCP 82 | photo list and download (HTTP) |
| B | `cam_rtsp.sock` | camera emulator `192.168.144.25:8554` | the video stream (RTSP over TCP) |

For the virtual Jetson (`--link virtual`) there is no SSH: a local stand-in makes the same socket files in
`~/.astra_sim/jetson_virtual/sock/`, so the container cannot tell the difference.

**How do I know it worked?** `link.sh status` ends with ` OK the link is up` and exit code 0. During a demo the
log is `<run folder>/jetson/link.log`.

## 6. When the link drops (cable pulled, Wi-Fi gone)

It behaves like the real drone losing its Jetson: **the autopilot keeps flying.** In AUTO it flies the mission; in
GUIDED (a detection hold, the drop loiter) it holds position until the Jetson is back.

What you see and what to do:

1. After about 6 s the demo terminal says `the link to the Jetson dropped` (and about once a minute
   `the Jetson link has been down for about N s`). link.sh's watcher writes every step into
   `<run folder>/jetson/link.log`; the demo terminal reads its state, so the watcher never holds the demo's output.
2. **Login mode:** the login window says the connection dropped. Plug the cable back in (or wait for Wi-Fi), then in
   that window press **Enter** and type the password. **Key mode:** nothing to do; it reconnects every 2 s by itself.
3. As soon as the login is back, link.sh ends the Jetson's old, stuck SSH session (the Jetson never notices a pulled
   cable by itself), removes the old socket files and adds the doors again. The terminal says
   `the Jetson link is back (drop #N, M s down in total)`. The relay in the container dials again by itself (it
   gives up on a connection after 4 s of silence and retries every second), so MAVROS gets its heartbeats back on
   its same connection.
4. While the link is down, commands MAVROS tried to send are **dropped, not kept** (so a late payload drop can never
   happen), exactly like a broken serial cable loses bytes.
5. If the tap stays silent for 30 s, the demo also watches the landing over MAVLink, so it cannot hang.
6. The run summary (`<run folder>/jetson/summary.md`) lists every drop, the time down and the dropped bytes.

**Rehearse it** (safe, no cable pulling):

```bash
bash $J/pc/link.sh drop 20 --link virtual --mode blackhole   # virtual: bytes stop for 20 s, sockets stay open
bash $J/pc/link.sh drop 20 --link virtual --mode close       # virtual: every connection closed
bash $J/pc/link.sh drop 20 --link usb --mode glitch          # board: your SSH connection frozen for 20 s
bash $J/pc/link.sh drop 20 --link usb --auth key --mode cut  # board, key mode only: frozen, then killed
```

## 7. Troubleshooting

| What you see | Likely cause | Fix |
|---|---|---|
| `windows_link_check` says `USB_ADAPTER=no` | wrong port, a charge-only cable, Jetson not booted | the Jetson's USB-C port (not USB-A), a data cable, power from the barrel jack, wait 1 min |
| `USB_UNKNOWN_DEVICE=yes`, Device Manager shows it under "Other devices" | Windows picked no driver for the USB network | owner, once: Device Manager -> right-click the device -> Update driver -> Browse my computer -> Let me pick -> Network adapters -> Microsoft -> "Remote NDIS Compatible Device" |
| adapter there but `USB_ADAPTER_STATUS` is not `Up` | the Jetson is still booting, or the cable wiggled | unplug and plug the USB-C cable again; wait 1 min |
| **Windows reaches it but Ubuntu does not** (`TCP22=ok`, but `ping 192.168.55.1` fails in Ubuntu) | WSL's network does not route to the USB network | try `ping` again after 10 s (the adapter may still be starting). Fallback: an Ethernet cable or Wi-Fi (section 3). Changing WSL to "mirrored" networking is the owner's choice: it edits `.wslconfig` and needs a WSL restart, so only when no simulator runs (HIL_DESIGN.md 13, risk 1) |
| `cannot reach HOST (no answer to ping or TCP 22)` (exit 3) | wrong address, different networks, campus Wi-Fi isolation | `bash $J/jetson.sh find`; check the address on the Jetson (`hostname -I`); use a hotspot, Tailscale or Ethernet |
| `HOST answers but SSH port 22 is closed` (exit 4) | the Jetson has not finished booting, or its SSH server is off | wait 1 min; on the Jetson (screen or USB serial console): `sudo systemctl enable --now ssh` |
| `Not logged in to USER@HOST` (exit 17) | the login window is closed or its connection dropped | `bash $J/jetson.sh login`, type the password in the window, keep it open |
| ssh in the login window warns that the **host key changed** | another board at the same address, or the Jetson was reflashed | `ssh-keygen -f ~/.astra_sim/jetson/known_hosts -R 192.168.55.1` (the address you use), then log in again |
| `the SSH forwards to the Jetson could not be added` (exit 14) | the Jetson's SSH server forbids forwarding, or `~/astra_hil/sock` is missing | `bash $J/jetson.sh check` (row 12 tests forwarding); `bash $J/jetson.sh setup` makes the folder. A forbidding `sshd_config` is the team's decision |
| `TCP 127.0.0.1:37261 / 37282 / 7461 ... is already in use` (exit 14) | the link of an earlier run is still up | `bash $J/pc/link.sh stop --link usb` (or the link you used), then start again |
| demo.sh stops right after `link watcher pid N ...` and never goes on | a link.sh older than 2026-10-31 15:40, whose watcher kept the demo's output pipe open | `bash simulation/setup/03_build.sh` (mirrors the fixed link.sh), then run the demo again; to rescue the running demo: `kill N` (the flight goes on without the watcher) |
| `the relay program .../hil_relay.py is missing` (exit 14) | an old checkout, or the Linux copy is not mirrored | `bash simulation/setup/03_build.sh` (it mirrors and builds), then try again |
| the link drops again and again | weak Wi-Fi, a loose cable, a USB hub | a direct cable, 5 GHz Wi-Fi, key mode (reconnects by itself) |
| photos arrive late or some are missing (legacy photo pipeline) | the link is too slow for one photo per second | `--jetson-photos after`; `bash $J/jetson.sh check --speed`; USB or Ethernet instead of Wi-Fi |
| MAVROS on the Jetson never connects, SITL shows no client on SERIAL2 | another MAVROS on this PC took SITL SERIAL2 (it serves ONE client) | never run `./launcher.sh` on the PC during a Jetson flight (`demo.sh --jetson` never does); stop it, then `link.sh stop` / `start` |
| `.local` name does not resolve | the network or Windows' resolver does not pass mDNS | use the IP address instead |

**Where to look:** `<run folder>/jetson/link.log` (drops, reconnects, every door added), `pc_sdk.log`, `pc_http.log`
(the camera relays on the PC), `collect.log` (the photo copy), `tap_receive.log` (the ROS tap), and
`~/.astra_sim/state/jetson_link.env` (the link's state right now). Outside a demo the link log is
`~/.astra_sim/logs/jetson_link.log`.

**Rescue console** (when SSH or the network part fails): the USB-C cable also gives a serial console. Windows shows
it as "USB Serial Device (COMn)"; open it with PuTTY at 115200 baud, 8N1, and press Enter for a login prompt.

## 8. Exit codes of the PC-side scripts

| Code | Meaning |
|---|---|
| 0 | OK |
| 1 | a step failed (the message names it) |
| 2 | the command was typed wrong (`--help` lists the options) |
| 14 | the link could not be opened, or is down (`link.sh status`) |
| 17 | not logged in (`jetson.sh login`) |

`jetson.sh` has its own full list (docs/jetson/README.md and HIL_DESIGN.md 15.4).

## 9. What only the owner does

We never change Windows settings, the Jetson's settings or the team's account. These are written steps for the owner:

- the RNDIS driver pick on Windows (section 7), once per PC;
- the PC's fixed Ethernet address (section 3), only for the Ethernet cable;
- any WSL networking change (`.wslconfig`), only if "Windows reaches it but Ubuntu does not" cannot be solved otherwise;
- key mode (`jetson.sh setup --key`): it adds one line to the Jetson user's `authorized_keys`, so it needs the owner's
  and the team's OK;
- setting the Jetson's clock (`sudo date -s @<PC time>`, typed in the login window) when `check` warns about it.
