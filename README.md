# ros_environment
ROS src packages that support the computer vision team's result

System: Ubuntu 16.04 LTS

## Get started
1. install Robotics Operating System Kinetic in your Ubuntu computer
http://wiki.ros.org/ROS/Installation

2. `cd ~ && mkdir ws && cd ws`

3. `git clone --recursive git@github.com:robomasterhkust/ros_environment.git src`

4. `cd cv_camera && git clone git@github.com:OTL/cv_camera.git`

5. `cd ~/ws && catkin_make`

## Toolchain
1. install the following dependency for fast development
`sudo apt-get install openocd minicom -y`

`sudo apt-get install terminator -y`

`sudo apt-get install cmake -y`

`sudo apt-get install vim -y`

`sudo apt-get install htop`

2. install pixhawk toolchain
To install the development toolchain:
https://dev.px4.io/en/setup/dev_env_linux.html

3. set up the .bashrc for speed up the development process

`vim ~/.bashrc`

and add the following:

`alias cmk='cd ~/ws && catkin_make -j4 -l4 && source devel/setup.bash && cd -'`

`alias ll='ls -alF'`
`alias la='ls -A'`
`alias l='ls -cF'`
`source ~/ws/devel/setup.bash`

and `source ~/.bashrc` after saving the vim file.

4. install CV related dependences
openCV, Eigen, PCL, Ceres

## Start the program
1. roscore

2. cd src/armor_recognition_publisher/launch

3. roslaunch armor_recognition.launch


## Reference
My TA GAO Wenliang's repository for ELEC 5660 code setup:

https://github.com/gaowenliang/ELEC5660_lab_code
