# MAVMaster
MAVLink communication for ENTERPRIZE. Writen by Edward Zhang, integrated by Beck Pang on March 23rd.

## Software Requirements

 - [boost](http://www.boost.org/)
 - [MAVLink](https://mavlink.io/en/) for MAVLink header generation

##ROS Nodes
 - mavmaster: converts `mavlink_attitude_t` received through serial port `dev/ttyUSB0` with `system_id` 21 and `component_id` 78 to ROS message `geometry_msgs/Twist` @ topic `Attitude`; converts ROS message `geometry_msgs/Twist` @ topic `cv_result` to `mavlink_attitude_t`
 - fakecv: publishes a fake cv result @ ROS topic `cv_result` using ROS message `geometry_msgs/Twist`
## Compilation Flags
 - -lboost\_system
 - -lpthread
