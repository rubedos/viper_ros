cmake_minimum_required(VERSION 3.5.1)

project(cvm_follow_me_sample)

add_compile_options(-std=c++14 -O3)

# Project variables
set(PROJECT_SRC     ${PROJECT_SOURCE_DIR}/src/FollowMeWindow.cpp)
set(PROJECT_HDR     ${PROJECT_SOURCE_DIR}/include/FollowMeWindow.h)
set(PROJECT_UI      ${PROJECT_SOURCE_DIR}/ui/FollowMeWindow.ui)
set(PROJECT_BIN_SRC ${PROJECT_SOURCE_DIR}/src/main.cpp)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cvm_msgs
  cv_bridge
  message_filters
  dynamic_reconfigure
)
find_package(Qt5 REQUIRED 
  Widgets REQUIRED
)
find_package(OpenCV REQUIRED)

# Manual MOC generation
qt5_wrap_cpp(PROJECT_HDR_MOC ${PROJECT_HDR})
qt5_wrap_ui(PROJECT_UI_MOC   ${PROJECT_UI})

catkin_package(
  LIBRARIES
)

include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
)

add_library(rubedo.robotics.${PROJECT_NAME} SHARED
  ${PROJECT_SRC}
  ${PROJECT_HDR_MOC}
  ${PROJECT_UI_MOC}
)

target_link_libraries(rubedo.robotics.${PROJECT_NAME}
  Qt5::Widgets 
  ${OpenCV_LIBRARIES} 
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME} ${PROJECT_BIN_SRC})
target_link_libraries(${PROJECT_NAME} rubedo.robotics.${PROJECT_NAME})

set(FILE "${CMAKE_CURRENT_SOURCE_DIR}/code")
if(NOT EXISTS "${FILE}")
  execute_process(COMMAND mkdir -p ${FILE})
endif()

install(DIRECTORY
  code
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  src
  include
  ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/code
)

install(FILES
  CMakeLists.txt
  package.xml
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/code
)

install(TARGETS
  rubedo.robotics.${PROJECT_NAME}
  ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

