cmake_minimum_required(VERSION 2.8.0)
project(handa_to_rosbag)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


cs_add_library(handa_to_rosbag
  src/library/handa_to_rosbag.cpp
  src/library/conversions.cpp
)

cs_add_executable(handa_to_rosbag_node
  src/handa_to_rosbag_node.cpp
)

target_link_libraries(handa_to_rosbag_node handa_to_rosbag ${catkin_LIBRARIES})

cs_install()
cs_export()
