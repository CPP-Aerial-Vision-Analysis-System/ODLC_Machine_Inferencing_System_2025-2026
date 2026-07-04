cmake_minimum_required(VERSION 3.8)
project(video_cam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(siyi src/siyi_node.cpp)
ament_target_dependencies(siyi
  rclcpp
  sensor_msgs
  std_msgs
  mavros_msgs
  cv_bridge
  OpenCV
)
target_link_libraries(siyi CURL::libcurl nlohmann_json::nlohmann_json)

install(TARGETS
  siyi
  DESTINATION lib/${PROJECT_NAME}
)

# Retain the Python utility modules (config, logging_utils, storage_manager)
# that the Python `detection` package imports from `video_cam`.
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
