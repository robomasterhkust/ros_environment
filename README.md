# ros_environment
ROS src packages that support the computer vision team's result

System: Ubuntu 16.04 LTS

## Get started
1. install Robotics Operating System Kinetic in your Ubuntu computer
http://wiki.ros.org/ROS/Installation

2. cd ~/catkin_ws

3. git clone --recursive git@github.com:robomasterhkust/ros_environment.git src

4. cd cv_camera && git clone git@github.com:OTL/cv_camera.git

5. catkin_make

## Toolchain
1. sudo apt-get install openocd minicom -y

2. install pixhawk toolchain
To install the development toolchain:

Download ubuntu_sim_nuttx.sh.
  Run the script in a bash shell:
  source ubuntu_sim_nuttx.sh
  You may need to acknowledge some prompts as the script progresses.
  Restart the computer on completion.

## Start the program
1. roscore

2. cd src/armor_recognition_publisher/launch

3. roslaunch armor_recognition.launch


## Reference
My TA GAO Wenliang's repository for ELEC 5660 code setup:

https://github.com/gaowenliang/ELEC5660_lab_code
