cmake_minimum_required(VERSION 3.8)
project(gps)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(gps src/gps.cpp)
ament_target_dependencies(gps rclcpp std_msgs mavros_msgs sensor_msgs)

install(TARGETS
  gps
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()