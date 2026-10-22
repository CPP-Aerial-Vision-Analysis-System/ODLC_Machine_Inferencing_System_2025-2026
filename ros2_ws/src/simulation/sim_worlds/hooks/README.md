# hooks/

`sim_worlds.dsv.in` is an **environment hook**: when you `source install/setup.bash`, it adds
`<install>/share/sim_worlds/models` to the `GZ_SIM_RESOURCE_PATH` variable, so Gazebo can find the models the
worlds use (`model://sim_worlds_ground`, `model://sim_worlds_pad`, ...).

The `.dsv` format (one `type;VARIABLE;value` per line) does not allow comments, which is why this note exists.
CMakeLists.txt registers the hook with `ament_environment_hooks(...)`.
