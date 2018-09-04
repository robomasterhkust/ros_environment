cmake_minimum_required(VERSION 2.8.3)
project(flir_reader)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    roscpp
    std_msgs
    cv_bridge
)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)


FIND_PATH(FLYCAP_INCLUDE_DIR FlyCapture2.h
    /usr/include/flycapture
    /usr/local/include/flycapture
)

message(STATUS "Flycapture include path: ${POINTGREY_INCLUDE_DIR}")

FIND_LIBRARY(FLYCAP_LIBRARY NAMES libflycapture.so
   /usr/lib
)

message(STATUS "Flycapture library: ${FLYCAP_LIBRARY}")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS
)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

set(${PROJECT_NAME}_DETECTION_SOURCES
    src/main.cpp
    src/CamBase.cpp
    src/FlirCam.cpp
)


add_executable(flir_reader_node ${${PROJECT_NAME}_DETECTION_SOURCES})

target_link_libraries(flir_reader_node
    ${catkin_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBS}
    ${FLYCAP_LIBRARY}
)
