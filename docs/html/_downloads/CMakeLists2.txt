cmake_minimum_required(VERSION 3.5.1)

project(image_streaming)

add_compile_options(-std=c++14 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
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

add_executable(image_streaming main.cpp)
target_link_libraries(image_streaming ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
