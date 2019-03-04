cmake_minimum_required(VERSION 3.5.1)

project(pointcloud_streaming)

add_compile_options(-std=c++14 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

#find_package(PCL 1.8 REQUIRED)

add_definitions(${PCL_DEFINITIONS})


catkin_package(
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pointcloud_streaming main.cpp)
target_link_libraries(pointcloud_streaming ${catkin_LIBRARIES} ${PCL_LIBRARIES})
