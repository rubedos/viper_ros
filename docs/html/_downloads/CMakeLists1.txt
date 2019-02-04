cmake_minimum_required(VERSION 3.5.1)

project(laserscan_streaming)

add_compile_options(-std=c++14 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laserscan_streaming main.cpp)
target_link_libraries(laserscan_streaming ${catkin_LIBRARIES})
