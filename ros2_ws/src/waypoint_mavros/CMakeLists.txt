cmake_minimum_required(VERSION 3.8)
project(waypoint_mavros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(interfaces REQUIRED)

add_executable(waypoint src/waypoint.cpp)
ament_target_dependencies(waypoint
  rclcpp
  mavros_msgs
  interfaces
)

add_executable(killnode src/killnode.cpp)
ament_target_dependencies(killnode
  rclcpp
  mavros_msgs
  rcl_interfaces
)

install(TARGETS
  waypoint
  killnode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
