cmake_minimum_required(VERSION 2.8.3)
project(wheel_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    sensor_msgs
    nav_msgs
    message_generation
    message_runtime
    )
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   steeringAngle.msg
   wheelSpeeds.msg
   vehicleSpeeds.msg
)
generate_messages(
    DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    nav_msgs
)
catkin_package(
        CATKIN_DEPENDS
        message_runtime
        std_msgs
)
