#!/usr/bin/env python3
"""
payload_release.py -- PURPOSE: start the payload_release node, the simulated drone's drop mechanism.
When ArduPilot SITL opens servo 11 (person drop) or 13 (tent drop), the node detaches that payload
in Gazebo so it falls, and logs the release and the landing point.

Installed as:  ros2 run sim_models payload_release [--ros-args -p open_threshold_us:=1400.0 ...]
All the logic lives in sim_models/payload_release_node.py (parameters and topics are listed there
and in README.md); this file only calls its main().
"""

import sys

from sim_models.payload_release_node import main

if __name__ == "__main__":
    sys.exit(main())
