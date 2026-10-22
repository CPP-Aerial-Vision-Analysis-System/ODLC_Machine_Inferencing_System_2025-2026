# sim_worlds/CMakeLists.txt - PURPOSE: install the simulated worlds (worlds/, models/, config/, missions/, truth/)
# into share/sim_worlds, the Python helpers (sim_worlds/*) as an importable package, the command-line tools
# (scripts/*) for `ros2 run sim_worlds <tool>`, an environment hook that adds share/sim_worlds/models to
# GZ_SIM_RESOURCE_PATH (so Gazebo finds model://sim_worlds_*), and the unit tests (colcon test).
cmake_minimum_required(VERSION 3.8)
project(sim_worlds NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python package: sim_worlds.geo / survey / mission / planner / world_config / truth / gzrun / worldgen
ament_python_install_package(${PROJECT_NAME})

# command-line tools:  ros2 run sim_worlds make_mission.py runway
install(PROGRAMS
  scripts/build_runway.py
  scripts/build_suas_field.py
  scripts/make_mission.py
  scripts/world_info.py
  scripts/render_preview.py
  scripts/check_world.py
  scripts/fetch_models.sh
  scripts/test_sim_worlds.sh
  DESTINATION lib/${PROJECT_NAME})

# data: the worlds and everything they reference
install(DIRECTORY worlds models config missions truth
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

# GZ_SIM_RESOURCE_PATH += <install>/share/sim_worlds/models  (applied when the workspace is sourced)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(sim_worlds_unit_tests test TIMEOUT 600)
endif()

ament_package()
