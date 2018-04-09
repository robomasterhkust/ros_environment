cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project( rune )
#####
# Enables link_directories() treat paths relative
# to the source dir.
if(POLICY CMP0015)
    cmake_policy(SET CMP0015 NEW)
endif(POLICY CMP0015)

#####
# Enables project() command manages VERSION variables.
if(POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif(POLICY CMP0048)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  genmsg
)

include_directories("${PROJECT_SOURCE_DIR}")
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src SRC) 

find_package(OpenCV REQUIRED)

#find_package( CUDA QUIET REQUIRED)
#INCLUDE(FindCUDA)
#INCLUDE_DIRECTORIES(/usr/local/cuda/include/)
 #set  (CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; -03 -gencode arch=compute_22,code=sm_22)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
add_executable( rune_node  ${SRC})
target_link_libraries( rune_node -lpthread -lrt  ${OpenCV_LIBS})
# set(CMAKE_BUILD_TYPE Release)
