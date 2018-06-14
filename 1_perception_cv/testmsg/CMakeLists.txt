cmake_minimum_required(VERSION 2.8.3)
project(testcvmsg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rm_cv
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES testmsg
#  CATKIN_DEPENDS rm_cv
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
