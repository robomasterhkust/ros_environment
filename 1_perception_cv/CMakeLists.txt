cmake_minimum_required(VERSION 2.8.3)
project(rm_cv)

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

FIND_LIBRARY(MVSDK ${PROJECT_SOURCE_DIR}/lib)

aux_source_directory(./src DIR_SRCS)
add_executable(${PROJECT_NAME}_node ${DIR_SRCS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${Boost_SYSTEM_LIBRARY}
   MVSDK
   ${OpenCV_LIBS}
)
