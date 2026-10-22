"""
example_detector -- PURPOSE: a TEMPLATE detector for the ASTRA simulator's plug-and-play detector slot.

    blob_logic   the detection "brain" (colour blobs), plain Python + OpenCV: replace it with your model
    node         the ROS 2 node: photo folder in, interfaces/ImageResult on /image_detection out (like new_od)

Start it alone:   ros2 run example_detector example_detector
In the demo:      bash simulation/scripts/demo.sh --profile example_detector
Check it:         ros2 run sim_plugins check_slot --slot detector --take-photo
"""
