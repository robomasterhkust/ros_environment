# ros_environment
ROS src packages that support the computer vision team's result

## Get started
1. install Robotics Operating System Kinetic in your Ubuntu computer
http://wiki.ros.org/ROS/Installation

2. cd ~/catkin_ws

3. git clone --recursive git@github.com:robomasterhkust/ros_environment.git src

4. cd cv_camera && git clone git@github.com:OTL/cv_camera.git

5. catkin_make

## Start the program
6. roscore

7. cd src/armor_recognition_publisher/launch

8. roslaunch armor_recognition.launch


## Reference
My TA GAO Wenliang's repository for ELEC 5660 code setup:

https://github.com/gaowenliang/ELEC5660_lab_code
