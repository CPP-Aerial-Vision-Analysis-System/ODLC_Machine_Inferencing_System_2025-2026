"""
example_listener -- PURPOSE: a TEMPLATE "extra" for the ASTRA simulator: it only listens to the flight and logs a
summary (summary.py = the bookkeeping, plain Python; node.py = the ROS 2 node).

Start it alone:   ros2 run example_listener example_listener
In the demo:      bash simulation/scripts/demo.sh --profile example_listener
Check it:         ros2 run sim_plugins check_slot --slot extras --node example_listener
"""
