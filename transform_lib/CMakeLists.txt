cmake_minimum_required(VERSION 3.0.2)
project(transform_lib)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES transform_lib
 CATKIN_DEPENDS geometry_msgs roscpp tf2_ros
 DEPENDS Eigen
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(transform_lib src/transform_lib/transform_functions.cpp src/transform_lib/quaternion_conversions.cpp src/transform_lib/eigen_transform_conversions.cpp src/transform_lib/nan_check.cpp)

target_link_libraries(transform_lib ${catkin_LIBRARIES})

install(DIRECTORY include/transform_lib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
