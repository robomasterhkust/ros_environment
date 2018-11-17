cmake_minimum_required(VERSION 2.8.3)
project(rm_cv)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE Debug)

set (WITH_MINDVISION false) #whether compile with MindVision camera support
set (WITH_FLYCAP false) #whether compile with FLIR camera support

if (WITH_MINDVISION)
add_definitions(-DWITH_MVSDK)
endif (WITH_MINDVISION)

if (WITH_FLYCAP)
add_definitions(-DWITH_FLYCAP)
endif (WITH_FLYCAP)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  genmsg
  cv_bridge
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")

#flir camera related
if (WITH_FLYCAP)
FIND_PATH(POINTGREY_INCLUDE_DIR FlyCapture2.h
    /usr/include/flycapture
    /usr/local/include/flycapture
)
message(STATUS "Found POINTGREYCamera: ${POINTGREY_INCLUDE_DIR}")

FIND_LIBRARY(POINTGREY_LIBRARY NAMES libflycapture.so
   /usr/lib
)
message(STATUS "Found POINTGREYCamera library: ${POINTGREY_LIBRARY}")

if (POINTGREY_INCLUDE_DIR AND POINTGREY_LIBRARY)
   set(POINTGREY_FOUND TRUE)
   set(POINTGREY_LIBRARIES ${POINTGREY_LIBRARY})
else (POINTGREY_INCLUDE_DIR AND POINTGREY_LIBRARY)
   set(POINTGREY_FOUND FALSE)
endif (POINTGREY_INCLUDE_DIR AND POINTGREY_LIBRARY)

if (POINTGREY_FOUND)
      message(STATUS "Found LibPOINTGREY: ${POINTGREY_LIBRARIES}")
else (POINTGREY_FOUND)
           message(STATUS "don't find LibPOINTGREY")
endif (POINTGREY_FOUND)
endif (WITH_FLYCAP)



add_message_files(
  FILES
  ArmorRecord.msg
  vertice.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs sensor_msgs
  DEPENDS
)

include_directories(
    src
    ${catkin_INCLUDE_DIRS}
)

link_directories(${PROJECT_SOURCE_DIR}/lib)


#armor_detection_node
add_executable(armor_detection_node
    src/detection/ArmorDetection.cpp
    src/detection/Settings.cpp
    src/detection/main.cpp
    src/detection/StopWatch.cpp
    src/detection/Camera.cpp
    src/detection/cvThreadPool.cpp
    src/detection/ROSInterface.cpp
    src/detection/ROSCamIn.cpp
)

target_link_libraries(armor_detection_node
   ${catkin_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   ${OpenCV_LIBS}
)

add_dependencies(armor_detection_node
${${PROJECT_NAME}_EXPORTED_TARGETS}
 ${catkin_EXPORTED_TARGETS} )

#camera_reader
add_executable(cam_reader ${${PROJECT_NAME}_DETECTION_SOURCES}
    src/cameraDriver/main.cpp
    src/cameraDriver/CamBase.cpp
    src/cameraDriver/V4LCamDriver.cpp
)

target_link_libraries(cam_reader
    ${catkin_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBS}
    ${POINTGREY_LIBRARIES}
)

#tracking_node
add_executable( tracking_node
    src/tracking/tracking_node.cpp
)

target_link_libraries( tracking_node
    ${catkin_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBS}
)

if (WITH_MINDVISION)
target_sources(cam_reader PUBLIC src/cameraDriver/mvCamera.cpp)

target_link_libraries(cam_reader
  MVSDK
)
endif(WITH_MINDVISION)

if(WITH_FLYCAP)
target_sources(cam_reader PUBLIC src/cameraDriver/FlycapCam.cpp)

target_link_libraries(cam_reader
  ${POINTGREY_LIBRARIES}
)
endif(WITH_FLYCAP)
