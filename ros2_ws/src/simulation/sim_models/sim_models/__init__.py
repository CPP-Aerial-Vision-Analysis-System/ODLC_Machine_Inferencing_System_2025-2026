"""
sim_models -- PURPOSE: Python helpers of the sim_models package (the simulated ASTRA drone).

Modules:
    payload_logic    decide WHEN to drop a payload (servo pulse width -> release) and WHEN a
                     dropped payload has landed; ENU -> latitude/longitude conversion.
                     Pure Python: no ROS, no Gazebo, so it is easy to unit test.
    model_generator  turn models/iris_a8mini/model.sdf.xacro into a plain SDF file with the
                     arguments you choose (camera profile, SITL instance port, ...).

The programs that use them are scripts/payload_release.py (ros2 run sim_models payload_release)
and scripts/generate_model_sdf.py (ros2 run sim_models generate_model_sdf).
"""
