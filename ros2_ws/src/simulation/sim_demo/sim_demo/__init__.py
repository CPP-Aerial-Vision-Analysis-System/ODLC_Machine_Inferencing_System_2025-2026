"""
sim_demo/__init__.py -- PURPOSE: marks sim_demo as a Python package.

sim_demo holds the tools a person WATCHES during a simulated ASTRA search mission:
the demo panel, the optional truth_detector stand-in, the RViz markers and the scorer.
The pure-math modules (geo, camera_model, photos, world_info, mission_file, detections,
phases, pose_history, truth_model, scoring, payloads) do not need ROS, so they can be
unit-tested anywhere with plain pytest.
"""

__version__ = '0.1.0'
