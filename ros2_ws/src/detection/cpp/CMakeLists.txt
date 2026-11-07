cmake_minimum_required(VERSION 3.17)
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

# A native diagnostic executable, independent of ROS and Python.
add_executable(check_gpu src/check_gpu.cpp)
target_compile_features(check_gpu PRIVATE cxx_std_17)
target_include_directories(check_gpu PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(check_gpu PRIVATE ${OpenCV_LIBS})
find_package(Threads REQUIRED)
target_link_libraries(check_gpu PRIVATE Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(check_gpu PRIVATE -Wall -Wextra -Wpedantic)
endif()

# CUDAToolkit does not require enabling CUDA or compiling a .cu source.
option(DETECTION_ENABLE_CUDA_DIAGNOSTICS "Use CUDA Runtime/cuBLAS when available" ON)
if(DETECTION_ENABLE_CUDA_DIAGNOSTICS)
  find_package(CUDAToolkit QUIET)
endif()
if(DETECTION_ENABLE_CUDA_DIAGNOSTICS AND CUDAToolkit_FOUND)
  target_compile_definitions(check_gpu PRIVATE DETECTION_CHECK_GPU_CUDA=1)
  target_link_libraries(check_gpu PRIVATE CUDA::cudart)
  if(TARGET CUDA::cublas)
    target_compile_definitions(check_gpu PRIVATE DETECTION_CHECK_GPU_CUBLAS=1)
    target_link_libraries(check_gpu PRIVATE CUDA::cublas)
  endif()
  message(STATUS "check_gpu: CUDA Runtime enabled")
else()
  message(STATUS "check_gpu: built without CUDA; NVIDIA device access will be reported as untested")
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(annotation_test test/annotation_test.cpp)
  target_link_libraries(annotation_test PRIVATE detection_annotation)
  add_test(NAME annotation_behavior COMMAND annotation_test ${CMAKE_CURRENT_BINARY_DIR}/test-output)
  add_test(NAME check_gpu_cli COMMAND ${CMAKE_COMMAND}
    -DPROGRAM=$<TARGET_FILE:check_gpu>
    -DTEST_DIR=${CMAKE_CURRENT_BINARY_DIR}/gpu-test-output
    -P ${CMAKE_CURRENT_SOURCE_DIR}/test/check_gpu_test.cmake)
  set_tests_properties(check_gpu_cli PROPERTIES TIMEOUT 30)
endif()
