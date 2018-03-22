cmake_minimum_required(VERSION 3.8)
project(ros_environment)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        armor_recognition_publisher/src/armor_recognition_publisher_node.cpp)

add_executable(ros_environment ${SOURCE_FILES})