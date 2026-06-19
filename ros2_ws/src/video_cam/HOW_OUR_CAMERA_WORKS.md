mission planner publishes /camera/trigger

siyi_node.py camera_trigger_callback and capture_requested.set
    this calls pipeline_loop
        this calls pipelineOrchestrator's execute_pipeline

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