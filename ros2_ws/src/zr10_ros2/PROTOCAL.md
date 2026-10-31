# Protocol, media behavior, and verification

Implemented from the supplied **ZR10 User Manual v1.7**:

- Section 3.5.1: `55 66`, CTRL, payload length (LE16), sequence (LE16), command,
  payload, CRC (LE16). CRC covers header plus payload, polynomial `0x1021`,
  initial value `0`, no reflection or final XOR.
- Sections 3.5.2–3.5.5: photo `0x0C / 00` has **no ACK**. Focus `0x04` uses
  `01 + touch_x LE16 + touch_y LE16`; zoom `0x05` uses signed byte `1`, `-1`, `0`.
  Focus and zoom request ACKs. Replies must match sequence and command, have the
  ACK flag and correct CRC/length. Timeouts are logged; commands are not blindly
  resent. `0x0B` function feedback is also logged.
- **Set `--af-x` / `--af-y` to a point within your actual stream resolution.**
  Defaults `(960,540)` are the center of a 1920×1080 stream, not an assertion of
  the camera's configured resolution. `--legacy-af` sends only `01` for older
  firmware using the earlier one-byte focus command; use only after confirming
  that firmware's documentation.
- Section 3.5.7: base `http://192.168.144.25:82//cgi-bin/media.cgi`, GET
  `/api/v1/getmediacount?media_type=0&path=` and
  `/api/v1/getmedialist?media_type=0&path=&start=0&count=100`.
  Blank path selects all photo directories. The manual shows GET parameters as
  objects; this implementation serializes them as URL query parameters and
  appends the API path to the given base. Firmware compatibility must be checked.
- Listings are paginated. The worker follows each returned absolute HTTP(S) URL,
  requires HTTP 200, downloads to `.part`, checks JPEG start/end markers, then
  atomically renames. Failure leaves the photo eligible for a later scan.
  Transfers have a 3-second connection timeout and 15-second total timeout.
- Each remote URL is encoded losslessly as hexadecimal path components under
  `--output`, ending in `.jpg`. This prevents path traversal and collisions
  between same-named files in different camera directories. Logs show the
  original photo name and local path. Completed files provide persistent
  deduplication, including after restart. Clearing files causes redownload.

Photo capture, SDK receive handling, and the media worker operate independently.
Scans never overlap. Slow downloads can extend a scan beyond two seconds; the
next scan starts after it finishes. A changing inventory is rescanned from the
start, so files displaced between pages are picked up on subsequent scans.
Remote URLs are treated as stable identities: replacing SD-card files under
identical URLs requires a fresh output directory. JPEG markers detect common
incomplete writes, but are not a full decoder or proof that a remote file has
stopped changing. Existing completed local files are assumed intact.

## Verification and hardware checks

Included tests cover published SDK CRC vectors, invalid/truncated packets,
real loopback ACK handling, automatic/manual capture, startup/zoom/manual focus
without periodic focus, zoom stop on exit, paginated downloads, slow-download
isolation, incomplete JPEG retry, and persistent deduplication. The simulator
requires permission to bind local UDP and HTTP sockets. It does not emulate
optics, SD-card write throughput, firmware quirks, or RTSP video.

When hardware arrives, first use `--photo-interval 0`: confirm manual capture,
focus coordinates and ACKs, zoom/stop, HTTP URLs, and video playback. Then enable
1-second capture and check actual new files and focus quality. A photo command
being sent is **not proof of a saved photo**; the SDK supplies no per-photo ACK,
and downloaded files cannot be reliably matched one-to-one to capture commands.
The manual does not establish sustained one-photo-per-second performance.

Official references: [SIYI ZR10 downloads](https://www.siyi.biz/en/product/tri-axis-single-camera-gimbal/zr10/download/)
and [SIYI v1.5 manual with earlier protocol examples](https://siyi.biz/siyi_file/ZR10/ZR10%20User%20Manual%20v1.5.pdf).
The supplied v1.7 PDF is the implementation reference; it is not redistributed.
