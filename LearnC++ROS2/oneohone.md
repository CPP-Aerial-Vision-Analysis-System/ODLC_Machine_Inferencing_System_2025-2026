
Step 0 — Sanity-check your shell

In the container:

source /opt/ros/humble/setup.bash
source ~/ultra_ws/install/setup.bash
cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
source install/setup.bash        # fails harmlessly the first time

Then confirm the deps you're about to declare actually exist:

ros2 interface show mavros_msgs/srv/StreamRate
ros2 interface show interfaces/srv/GetGPSData

If interfaces isn't found, build it first: colcon build --packages-select interfaces, then re-source. Do this before creating your package — if a dependency is missing you want to know now, not inside a wall of CMake errors.

Put those source lines in ~/.bashrc so every new terminal has them. Forgetting to source is the #1 cause of "package not found."

Step 1 — Generate the package

cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws/src
ros2 pkg create --build-type ament_cmake --dependencies rclcpp interfaces mavros_msgs sensor_msgs std_msgs  gps_cpp

New package — leave gps_ros2 alone so it keeps flying while you work.

Now open the generated package.xml and CMakeLists.txt and find where those five dependencies landed. They're in both files. Knowing that is what saves you when you need to add a sixth.

Step 2 — Read the contract before writing code

cat ../src/interfaces/srv/GetGPSData.srv

Notice what's above vs below the ---: request vs response. GetGPSData's request is empty. Also skim gps_ros2/gps_ros2/gps.py once more — you're reimplementing it exactly, same node name and topics.

Step 3 — Compile an empty node FIRST

~15 lines: include rclcpp.hpp, a class inheriting rclcpp::Node that only sets its name, and a main that inits, spins, shuts down. No callbacks, no logic.

Then wire it into CMakeLists.txt — you need add_executable, ament_target_dependencies, and install(TARGETS ... DESTINATION lib/${PROJECT_NAME}). Copy the shape from mainCPP/CMakeLists.txt:18-29; it's a working example of exactly this, already in your repo.

cd /ODLC_Machine_Inferencing_System_2025-2026/ros2_ws
colcon build --packages-select gps_cpp
source install/setup.bash
ros2 run gps_cpp gps_node

Don't move on until that runs. Build problems on an empty file take ten minutes. The same problems on a finished file take two hours, because you can't tell CMake errors from code errors.

Always use --packages-select gps_cpp. A full workspace build in this container pulls in detection and torch and will cost you real time on every iteration.

Step 4 — Add one piece at a time, build after each

One subscriber → the other two → heartbeat timer → service server → service client.

Build after every one. ROS template errors run 300 lines; ten new lines means you know where it came from, a hundred means you're reading spew. When it does explode, read the first error only — the rest are usually fallout.

The trivia that would otherwise cost you hours

- Headers are snake_case: NavSatFix → sensor_msgs/msg/nav_sat_fix.hpp
- qos_profile_sensor_data → rclcpp::SensorDataQoS(). Get this wrong on the GPS topics and you silently receive nothing.
- Messages arrive as shared pointers: msg->connected, not msg.connected
- Subscriber callback: std::bind(&Class::cb, this, _1). Service callback takes two args, so _1, _2
- Python's None → std::optional<T>

Three traps, in the order you'll hit them

1. It compiles, runs, and receives nothing. You didn't store create_subscription<T>(...) in a member variable, so it was destroyed when the constructor ended. Everything — subscriptions, timers, services, clients — must be held in a member. This will get you at least once.

2. It hangs on startup at set_stream_rate. The Python uses rclpy.spin_until_future_complete() in the constructor. Do not translate that literally — in C++ the node isn't spinning yet, so it waits forever. You need async_send_request with a callback. Read fetch_mission_indices() at mainCPP/src/main_controller.cpp:151 first; it's the pattern you want.

3. interfaces/srv/get_gps_data.hpp: No such file. Missing from CMakeLists.txt or package.xml — it must be in both.

Step 5 — Prove it against the Python

Same node name and service, so it's a straight swap. Record the Python's answer first:

ros2 run gps_ros2 gpstest                                      # terminal 1
ros2 service call /get_drone_data interfaces/srv/GetGPSData    # terminal 2

Kill it, run gps_cpp gps_node, call again. Same numbers = done. Only then point tracker.launch.xml at the new package.

Two container-specific gotchas

Never run both at once. Same node name, same service — you'd get two nodes fighting over /get_drone_data and confusing results. Kill one before starting the other.

build/, install/, and log/ land on your Mac through the bind mount. Check they're gitignored before you commit, or you'll add thousands of files.

---

Write it and bring me what you get, working or not — your error output will teach you more than my finished file would. If one piece stalls you, ask about that piece.
