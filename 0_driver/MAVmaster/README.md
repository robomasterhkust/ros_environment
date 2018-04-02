# MAVMaster
MAVLink communication for ENTERPRIZE.
## Software Requirements

 - [boost](http://www.boost.org/)
 - [MAVLink](https://mavlink.io/en/) for MAVLink header generation

## ROS Nodes
 - mavmaster: converts `mavlink_attitude_t` received through serial port `dev/ttyUSB0` with `system_id` 21 and `component_id` 78 to ROS message `geometry_msgs/Twist` @ topic `Attitude`; converts ROS message `geometry_msgs/Twist` @ topic `cv_result` to `mavlink_attitude_t`
 - armor: publishes a fake cv result @ ROS topic `cv_result` using ROS message `geometry_msgs/Point`
 - rune: publishes a fake rune detection result @ ROS topic `rune_result` using ROS message `geometry_msgs/Point`
 - autoaim: converts cv result to angular velocity of gimbal `gimbal_ang_vel` @ ROS message `geometry_msgs/Vector3` using fake PID
 - listener: a fake mavmaster node
## Compilation Flags
 - -lboost\_system
 - -pthread
