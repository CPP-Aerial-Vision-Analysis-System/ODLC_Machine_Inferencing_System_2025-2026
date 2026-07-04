cmake_minimum_required(VERSION 3.8)
project(comms)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)

# Shared, dependency-free mission-state store. Both command_listener and
# system_controller must agree on the file path; bake in a default alongside the
# sources (overridable at runtime via COMMS_STATE_FILE).
add_library(comms_mission_state src/mission_state.cpp)
target_include_directories(comms_mission_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(comms_mission_state PUBLIC
  COMMS_STATE_FILE_DEFAULT="${CMAKE_CURRENT_SOURCE_DIR}/mission_state.json")

# Non-MAVLink executables.
add_executable(system_controller src/system_controller.cpp)
target_link_libraries(system_controller comms_mission_state)

add_executable(mavlink_router src/mavlink_router.cpp)

# MAVLink-dependent executables (built only when the MAVLink C headers exist).
find_package(mavlink QUIET)
if(mavlink_FOUND)
  add_executable(command_listener src/command_listener.cpp)
  target_link_libraries(command_listener comms_mission_state)
  target_include_directories(command_listener PRIVATE ${mavlink_INCLUDE_DIRS})

  add_executable(ground_sender src/ground_sender.cpp)
  target_include_directories(ground_sender PRIVATE ${mavlink_INCLUDE_DIRS})

  install(TARGETS command_listener ground_sender
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "mavlink package not found: skipping command_listener and ground_sender. "
    "Install ros-${ROS_DISTRO}-mavlink (or mavros) to build them.")
endif()

install(TARGETS
  system_controller
  mavlink_router
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
