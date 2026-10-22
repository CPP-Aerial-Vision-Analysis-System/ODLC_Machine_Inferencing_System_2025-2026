# How our camera works

## October 2026 update: no more lost photos, and a "sidecar" for every photo

In plain words, this is what happens now when someone asks for a photo:

1. **A photo request arrives.** During the survey `mapping/do_digi_cam_trigger.py` publishes
   `/camera/trigger` (true) once per second. `siyi_node.py` puts every request into a
   **waiting line** (`video_cam/capture_queue.py`, oldest first, at most 5 waiting).
   Before, the node threw a request away whenever the lens was zooming or focusing:
   in one simulator run 49 of 83 requests were lost.
2. **The photo is taken as soon as the camera is free.** A small timer (every 0.1 s) takes the
   oldest request and starts a capture, one at a time. Zooming and focusing no longer block
   photos: they hold the camera only for the milliseconds of each SDK command and wait for the
   lens to settle WITHOUT holding it. (A photo during a focus pass can be a little soft; a lost
   photo is a hole in the search.) Only if the camera really cannot keep up for a long time is
   the OLDEST request dropped, and the log says so ("Photo queue full ...").
3. **Right before the shutter** the node takes the drone's GPS position for the file name
   (`"<lat> , <lon>.jpg"`, unchanged) and asks the gimbal for its angles (SDK `0x0D`).
4. **A sidecar file is written next to the photo**: `"<lat> , <lon>.json"` with the drone's exact
   position, height above home, compass heading, the camera's field of view, zoom, gimbal angles
   and which way the photo is turned relative to the drone (`video_cam/photo_metadata.py`
   explains every field). `main_controller` uses it to drop the payload ON the target, not on the
   spot where the drone was when it took the photo.
5. Capture, indexing and download then work exactly as described below.

Camera models: the A8 mini cannot autofocus, focus by hand or report its zoom (SIYI's own SDK
table). `config.CAMERA_CAPABILITIES` lists this, and the node simply does not send those commands
to an A8 mini (each one used to cost a 2 second timeout).

New settings:

| Where | Name | Default | Meaning |
|---|---|---|---|
| ROS parameter of `siyi` | `image_top_direction_deg` | 0 | where the TOP of a photo points, clockwise from the drone's nose, when the camera looks straight down (0 = nose; 180 = camera mounted backwards). Check it on the real drone: a marker 5 m in front of the nose must be at the top of the photo for 0. |
| `config.py` | `CAMERA_HFOV_DEG` | 81 | photo field of view per model, used to turn pixels into metres. Measure the ZR10 / ZR30 values. |
| `config.py` | `MAX_PENDING_CAPTURES` | 5 | photo requests that may wait |
| `config.py` | `WRITE_PHOTO_SIDECAR` | True | write the `.json` next to every photo |

Watch it work: every 30 s the node logs a line like
`Photo stats: 94 triggers: 94 served, 0 dropped (queue full), ...; 94 photos saved`.

## The original notes (still true for the capture itself)

mission planner publishes /camera/trigger

siyi_node.py camera_trigger_callback puts the request in capture_queue (it used to call capture_requested.set)
    a 0.1 s timer (_service_capture_queue) starts the capture thread
        this calls pipelineOrchestrator's capture_and_index, then download_and_save

Capture
    cameraInterface sends a UDP command 192.168.144.25:37260 which is the camrea sdk port
        the command is 0x0c (capture specific command for the siya8)

wait for 0.5s (can probably be removed)

Indexing
    Poll every 0.5s, we send a HTTP get request to 192.168.144.25:82/getmedialist which looks for files
        when new files are found it sends a confirmation log

Downlading
    we send a HTTP get image request to the 192.168.144.25:82 with the image name that we found from the previous 
        we decode it( turn jpeg to raw pixel data so that the program can use)
        atomic write (file saving technique that makes sure that nothing is corrupted)
        verifying dimensions


advantages

when we are not using video streaming and switch to udp/http pipeline, 
we are droping from 7Mbs to 0.5Mbs

on top of that, image_pub is not only streaming, but its also recording? which is very very memory consuming. This is more suitable for a surveilance camera and adds additional pressure on the jetson. 
ough “pixel data moved” per second at 10 Hz:

1080p: ~118.7 MiB/s (2 copies)

4K: ~474.6 MiB/s (2 copies)
This is continuous.

with the new version you save a lot of continuous memory bandwidth and “constant buffer activity,” even if peak RAM isn’t wildly different.

Streaming (image_pub.py) CPU costs (continuous):

H.264 decode (software) + colorspace conversions

cv_bridge conversion

ROS publish/serialization

JPEG encode + disk write every tick (cv2.imwrite)

Trigger pipeline CPU costs (burst only):

HTTP requests + polling

JPEG decode once (cv2.imdecode)

a mean-brightness check (np.mean(img)) once

JPEG write once

publish once

If you’re also running YOLO/TensorRT on the same Jetson, removing the always-on stream decode/encode usually frees up enough CPU/RAM bandwidth to make the whole system more stable.