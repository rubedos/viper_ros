cmake_minimum_required(VERSION 3.5.1)

project(disparity_streaming)

add_compile_options(-std=c++14 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  stereo_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(disparity_streaming main.cpp)
target_link_libraries(disparity_streaming ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
