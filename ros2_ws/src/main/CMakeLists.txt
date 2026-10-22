# main/CMakeLists.txt -- PURPOSE: build/install rules of the "main" package: installs the mission
# controller (src/main_controller.py), the bench tool (src/capture_detect_drop.py), the helper
# modules main_controller imports (src/target_geolocation.py = drop aiming, src/timed_steps.py =
# non-blocking timed steps; installed NEXT TO main_controller.py so a plain `import` finds them),
# the launch file (launch/tracker.launch.xml) and registers the helpers' unit tests (test/).
cmake_minimum_required(VERSION 3.8)
project(main)

set(CMAKE_CXX_STANDARD 17)

if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
  set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(OpenCV REQUIRED)

# Messages are now defined in the interfaces package, not here

install(PROGRAMS
  src/main_controller.py
  src/capture_detect_drop.py
  DESTINATION lib/${PROJECT_NAME}
)

# Helper modules imported by main_controller.py. They must sit in the same folder as the script
# (Python puts the script's own folder first on its import path).
install(FILES
  src/target_geolocation.py
  src/timed_steps.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # Unit tests of the helper modules (plain pytest; they also run without ROS:
  #   python3 -m pytest ros2_ws/src/main/test -v). Skipped quietly if ament_cmake_pytest is missing.
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_target_geolocation test/test_target_geolocation.py)
    ament_add_pytest_test(test_timed_steps test/test_timed_steps.py)
  endif()
endif()

ament_package()
