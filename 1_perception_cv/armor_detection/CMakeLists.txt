cmake_minimum_required(VERSION 2.8.3)
project(rm_cv)

set (USE_MVSDK true)

if (USE_MVSDK)
add_definitions(-DUSE_MVSDK)
endif (USE_MVSDK)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  genmsg
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
  LIBRARIES rm_cv
)

include_directories(
    include/rm_cv
    include/mvux_camera
    ${catkin_INCLUDE_DIRS}
)

if (USE_MVSDK)
set(${PROJECT_NAME}_SOURCES
    src/ArmorDetection.cpp
    src/Settings.cpp
    src/V4LCamDriver.cpp
    src/ArmorTracker.cpp
    src/main.cpp
    src/StopWatch.cpp
    src/Camera.cpp
    src/cvThreadPool.cpp
    src/ROSHandle.cpp
    src/GimbalController.cpp
    src/serial.cpp
    src/interface.cpp
    src/mvCamera.cpp
)
else(USE_MVSDK)
set(${PROJECT_NAME}_SOURCES
    src/ArmorDetection.cpp
    src/Settings.cpp
    src/V4LCamDriver.cpp
    src/ArmorTracker.cpp
    src/main.cpp
    src/StopWatch.cpp
    src/Camera.cpp
    src/cvThreadPool.cpp
    src/ROSHandle.cpp
    src/GimbalController.cpp
    src/serial.cpp
    src/interface.cpp
)
endif(USE_MVSDK)

add_executable(armor_detection_node ${${PROJECT_NAME}_SOURCES})


target_link_libraries(armor_detection_node
   ${catkin_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${OpenCV_LIBS}
)

if (USE_MVSDK)
FIND_LIBRARY(MVSDK ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(armor_detection_node
  MVSDK
)
endif(USE_MVSDK)
