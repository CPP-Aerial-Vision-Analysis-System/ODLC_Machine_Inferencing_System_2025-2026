# ZR10 Jetson controller

C++17 camera controls, automatic photo capture, HTTP photo retrieval, and optional
continuous LAN video relay. **Hardware is not currently available for physical
verification.** The code compiles on macOS and passes local camera-simulator
tests; Jetson, optics, SD performance, firmware, and real RTSP need verification.

## Build on the Jetson

Requires GCC 8+, CMake 3.10+, libcurl, nlohmann JSON; FFmpeg for video relay.

```sh
sudo apt update
sudo apt install build-essential cmake libcurl4-openssl-dev nlohmann-json3-dev ffmpeg
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
python3 tests/integration_test.py ./build/zr10
```

Older Nano/JetPack Ubuntu 18.04 may use GCC 7: install `g++-8`, configure with
`-DCMAKE_CXX_COMPILER=g++-8`. For CMake <3.13 use `mkdir build`, `cd build`,
`cmake ..`, `cmake --build .`, and `ctest`. GCC 8 filesystem linking is handled.

## Connect and run

Power the ZR10, insert its SD card, and connect Ethernet. Put the Jetson's
camera-facing interface on `192.168.144.0/24` (for example `192.168.144.30/24`),
preserving your Internet default route. Camera defaults: **192.168.144.25:37260 UDP**.

```sh
./build/zr10 --output ./photos
```

Commands: `photo`, `autofocus`, `zoomin`, `zoomout`, `zoomstop`, `help`, `quit`.
Press Enter after a command. Ctrl-C shuts down; closed stdin permits headless use.

| Behavior | Default | Option |
| --- | --- | --- |
| Automatic capture | Every 1 s | `--photo-interval` (0 disables) |
| Photo scan | Every 2 s | `--sync-interval` |
| Startup autofocus | Once after 2 s | `--startup-delay` |
| Autofocus cooldown | 2 s | `--af-cooldown` |
| Autofocus after zoom stops | After 0.5 s | `--zoom-settle` |
| Automatic zoom stop | After 5 s | `--zoom-max` |

There is **no periodic autofocus**. Manual/zoom focus requests coalesce during
cooldown and wait until zoom stops. Capture does not wait for autofocus.
Use `--af-x` and `--af-y` for a point within the actual stream resolution;
default `(960,540)` is the center of 1920×1080. `--help` lists endpoint overrides.

Photos download onto the Jetson; `--output` can be a mounted LAN share. The first
scan includes existing SD photos. Full remote URLs are encoded into unique local
paths; logs show original names. Completed downloads are remembered across
restarts. Slow scans may exceed two seconds; they never overlap or block camera
controls. Run one instance per output directory. See [protocol details](PROTOCOL.md)
for HTTP assumptions, retries, file identity, legacy autofocus, and limitations.

## Continuous video

A client with a route to the camera can open `rtsp://192.168.144.25:8554/video1`
(main stream) or `rtsp://192.168.144.25:8554/video2` (substream).
To relay through the Jetson, replace the example address with your LAN receiver:

```sh
./build/zr10 --video-output 'udp://192.168.1.50:5600?pkt_size=1316'
# On the receiving computer (allow UDP port 5600):
ffplay -fflags nobuffer 'udp://0.0.0.0:5600'
```

FFmpeg copies compressed video into MPEG-TS and restarts after failures with a
3-second delay. It must support RTSP/TCP and `-rw_timeout`. `--rtsp` overrides the
source. `--video-output capture.ts` saves locally without overwriting/resuming
existing files. Relay is opt-in because it needs a receiver address; direct RTSP
remains available. No automatic receiver discovery or SD recording controls.

## First hardware test

Start with `--photo-interval 0`. Check manual capture, focus coordinates/ACKs,
zoom/stop, HTTP downloads, and video. Then enable 1-second capture and check
actual files and focus quality. A sent photo command does not prove a saved
photo, and sustained 1-photo/s performance is unverified.

Implementation follows the supplied **ZR10 User Manual v1.7**, sections
3.5.1–3.5.7, including SDK framing and CRC. [Protocol and test notes](PROTOCOL.md)
contain the details and [official references](https://www.siyi.biz/en/product/tri-axis-single-camera-gimbal/zr10/download/).
