cmake_minimum_required(VERSION 3.5.1)

project(cvm_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    sensor_msgs
    std_msgs
    stereo_msgs
    message_generation
    roslint
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    StereoImage.msg
)

generate_messages(
  DEPENDENCIES
    sensor_msgs
    std_msgs
    stereo_msgs
)

catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    message_runtime
    std_msgs
    stereo_msgs
)
