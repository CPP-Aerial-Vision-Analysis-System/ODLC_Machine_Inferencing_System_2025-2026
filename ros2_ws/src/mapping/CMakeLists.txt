cmake_minimum_required(VERSION 3.8)
project(mapping)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(trigger src/do_digi_cam_trigger.cpp)
ament_target_dependencies(trigger
  rclcpp
  std_msgs
  mavros_msgs
  rcl_interfaces
)

add_executable(mapping src/mapping.cpp)
ament_target_dependencies(mapping
  rclcpp
  mavros_msgs
  OpenCV
)

install(TARGETS
  trigger
  mapping
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
