# sim_models/CMakeLists.txt -- PURPOSE: build/install rules of the sim_models package (the simulated drone).
# There is nothing to compile. It installs:
#   share/sim_models/models/iris_a8mini/...   the Gazebo model (xacro source, default model.sdf, meshes)
#   share/sim_models/config/astra_sitl.parm   extra ArduCopter SITL parameters for this drone
#   share/sim_models/launch/*.launch.py       launch file for the payload_release node
#   share/sim_models/test/...                 test world + acceptance test scripts
#   lib/sim_models/payload_release            node: drops a payload when its SITL servo opens
#   lib/sim_models/generate_model_sdf         tool: model.sdf.xacro -> model.sdf with chosen arguments
#   python module sim_models                  shared helpers (release logic, model generator)
# and an environment hook, so sourcing install/setup.bash puts share/sim_models/models on
# GZ_SIM_RESOURCE_PATH (then Gazebo finds model://iris_a8mini by itself).
cmake_minimum_required(VERSION 3.8)
project(sim_models NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Python helper module: import sim_models.payload_logic, sim_models.model_generator, ...
ament_python_install_package(${PROJECT_NAME})

# Programs started with:  ros2 run sim_models <name>
install(PROGRAMS scripts/payload_release.py
  DESTINATION lib/${PROJECT_NAME} RENAME payload_release)
install(PROGRAMS scripts/generate_model_sdf.py
  DESTINATION lib/${PROJECT_NAME} RENAME generate_model_sdf)

# Data folders
install(DIRECTORY models config launch
  DESTINATION share/${PROJECT_NAME})

# Test world and acceptance tests (so they also run from an installed workspace)
install(DIRECTORY test/worlds test/acceptance
  DESTINATION share/${PROJECT_NAME}/test
  USE_SOURCE_PERMISSIONS)

# Sourcing the workspace prepends share/sim_models/models to GZ_SIM_RESOURCE_PATH.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

if(BUILD_TESTING)
  # Unit tests (no simulator needed): colcon test --packages-select sim_models
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_payload_logic test/test_payload_logic.py TIMEOUT 60)
  ament_add_pytest_test(test_model_generation test/test_model_generation.py TIMEOUT 180)
  ament_add_pytest_test(test_astra_sitl_parm test/test_astra_sitl_parm.py TIMEOUT 60)
  ament_add_pytest_test(test_payload_release_node test/test_payload_release_node.py TIMEOUT 60)
  ament_add_pytest_test(test_camera_geometry test/test_camera_geometry.py TIMEOUT 120)
endif()

ament_package()
