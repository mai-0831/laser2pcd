cmake_minimum_required(VERSION 3.0.2)
project(laser2pcd)

find_package(PCL 1.8 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS
  laser_geometry
  roscpp
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
)

set(CMAKE_CXX_FLAGS "-O3 -g -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 11)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(laser2pcd_v2 src/laser2pcd_v2.cpp)
target_link_libraries(laser2pcd_v2
   ${catkin_LIBRARIES}
)


add_executable(passthrough_filter src/passthrough_filter.cpp)
target_link_libraries(passthrough_filter
   ${catkin_LIBRARIES}
)

