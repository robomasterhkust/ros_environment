cmake_minimum_required(VERSION 2.8.3)
project(calib_image_saver)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC -fopenmp")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
     INCLUDE_DIRS include
     LIBRARIES
     CATKIN_DEPENDS image_transport roscpp sensor_msgs std_msgs
#     DEPENDS system_lib
    )
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

add_executable(stereoImageSaver    src/stereoImageSaver.cc    src/chessboard/Chessboard.cc    )
target_link_libraries(stereoImageSaver  dw  ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(singleImageSaver    src/singleImageSaver.cc    src/chessboard/Chessboard.cc    )
target_link_libraries(singleImageSaver  dw  ${catkin_LIBRARIES} ${OpenCV_LIBS})
