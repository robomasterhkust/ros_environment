# ros_environment
ROS src packages that support the computer vision team's result

System: Ubuntu 16.04 LTS

ROS Version: Kinetic

## Attention
1. Never add CMakeList.txt to git repo.
2. Never add CMakeList.txt to git ignore.
3. There will be reminder that CMakeList.txt is not tracked in git. Just leave it there. Every computer has different src CMakeList.txt which linked to the place where ros is installed. 

## Software architecture

- modules
  - 0_driver
    - external library
  - 1_perception_cv
  - 2_perception_others
  - 3_estimator
  - 4_control
  - 5_planning
  - 6_decision
- tools

## Get started
1. install Robotics Operating System Kinetic in your Ubuntu computer
http://wiki.ros.org/ROS/Installation

2. `cd ~ && mkdir ws && cd ws`

3. `git clone --recursive git@github.com:robomasterhkust/ros_environment.git src`

4. `git submodule update`

5. `sudo cp 1_perception_cv/RMComputerVision/lib/libMVSDK.so /usr/lib`

6. `cd ~/ws && catkin_make`

## Toolchain
1. install the following dependency for fast development
`sudo apt-get install openocd minicom -y`

`sudo apt-get install terminator cmake vim htop libmuparser-dev -y`

2. set up the .bashrc for speed up the development process

`vim ~/.bashrc`

and add the following:

`alias cmk='cd ~/ws && catkin_make -j4 -l4 && source devel/setup.bash && cd -'`

`alias ll='ls -alF'`

`alias la='ls -A'`

`alias l='ls -cF'`

`source ~/ws/devel/setup.bash`

and `source ~/.bashrc` after saving the vim file.

3. install DJI-SDK
http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started

4. install CV related dependences

openCV, Eigen, PCL, Ceres

5. install RMOC open sourced dependency
https://github.com/robomasterhkust/RoboRTS

Reference: My TA GAO Wenliang's repository for ELEC 5660 code setup:

https://github.com/gaowenliang/ELEC5660_lab_code

## Multi-machine debugging
Two ros machines, one Intel Nuc, one TX2.

Follow the guide line here: https://askubuntu.com/questions/22835/how-to-network-two-ubuntu-computers-using-ethernet-without-a-router.

Set the ip address of nuc to be `10.0.0.2`, tx2 to be `10.0.0.1`. Nuc is the ros master.

`export ROS_MASTER_URI=http://10.0.0.2:11311` on both computers.

To debug using another ROS machine, using multimachine by modifying the `/etc/hosts` and setting three environment variables in the `~/.bashrc`

On both machines, add the ip address of the itself and the slave in the `/etc/hosts`, for instance, like this:

```
127.0.0.1       localhost
127.0.1.1       desktop
10.0.0.2   	rmsoldier#
10.0.0.1   	tegra-ubuntu
```


Then launch roscore in the master machine, and all nodes don't need to launch roscore and can receive topics from each others.
