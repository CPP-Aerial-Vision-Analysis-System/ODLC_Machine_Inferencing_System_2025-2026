cmake_minimum_required(VERSION 3.16)
project(detection_annotation LANGUAGES CXX)

# Independent during the incremental migration; the ROS Python package is unchanged.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
add_library(detection_annotation src/annotation.cpp)
target_compile_features(detection_annotation PUBLIC cxx_std_17)
target_include_directories(detection_annotation PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(detection_annotation PUBLIC ${OpenCV_LIBS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(detection_annotation PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(annotation_test test/annotation_test.cpp)
  target_link_libraries(annotation_test PRIVATE detection_annotation)
  add_test(NAME annotation_behavior COMMAND annotation_test ${CMAKE_CURRENT_BINARY_DIR}/test-output)
endif()
