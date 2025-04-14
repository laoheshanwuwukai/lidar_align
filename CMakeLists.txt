cmake_minimum_required(VERSION 2.8.3)
project(lidar_align)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE release)
endif()


# NLOPT is frustratingly inconsistent in the name of its cmake file so we use our own
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(gflags REQUIRED)
set(CMAKE_FIND_DEBUG_MODE ON)
find_package(NLOPT REQUIRED)

include_directories(
  include
  ${NLOPT_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library( lidar_align_common
  src/sensors.cpp
  src/aligner.cpp
  src/loader.cpp
)
target_link_libraries( lidar_align_common
  ${NLOPT_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)


add_executable(lidar_align_node
  # src/lidar_align_node.cpp
  src/test.cpp
)
target_link_libraries(lidar_align_node
  lidar_align_common
  gflags_shared
)


install(DIRECTORY include/lidar_align
  DESTINATION include
  FILES_MATCHING
    PATTERN "*.h"
    PATTERN "*.hpp"
)
