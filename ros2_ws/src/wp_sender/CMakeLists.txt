cmake_minimum_required(VERSION 3.8)
project(wp_sender)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(send_wp src/waypoint_sender.cpp)
ament_target_dependencies(send_wp
  rclcpp
  interfaces
)

add_executable(test_param src/parameter.cpp)
ament_target_dependencies(test_param
  rclcpp
  rcl_interfaces
)

install(TARGETS
  send_wp
  test_param
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
