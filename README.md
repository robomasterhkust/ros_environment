# ros_environment
ROS source code that for visual servo controller and localization

System: Ubuntu 16.04 LTS

ROS Version: Kinetic

Hardware: NVIDIA Jeston TX2

## Software architecture
- modules
  - 0_driver
  - 1_perception_cv
  - 2_perception_others
  - 3_estimator
  - 4_planning
  - 5_decision
  - 6_control
- tools

Most used
- 0_driver: CAN driver; dynamic reconfigure; rqt_multiplot.
- 1_perception_cv: RMComputerVision, the visual frontend.
- 3_estimator: wheel_odom, wheel_driver.
- 6_control: the visual servo control backend, handling control and target velocity estimation.


## Get started
If you compile the source code for the first time, start with the minimum code necessary: The CAN driver, the 1_perception_cv, and 6_control.

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

`sudo apt-get install terminator cmake vim htop libmuparser-dev ntp* -y`

install these dependency for the visualization tool rqt_multiplot
`sudo apt-get install ros-kinetic-rqt libqwt-dev libqwt-qt5-dev ros-kinetic-rqt-multiplot -y`

2. set up the .bashrc for speed up the development process

`vim ~/.bashrc`

and add the following:

`alias cmk='cd ~/ws && catkin_make -j4 -l4 && source devel/setup.bash && cd -'`

`alias ll='ls -alF'`

`alias la='ls -A'`

`alias l='ls -cF'`

`source ~/ws/devel/setup.bash`

and `source ~/.bashrc` after saving the vim file.

3. install Computer vision related dependences

openCV, Eigen

Reference I: RMOC open sourced repository
https://github.com/robomasterhkust/RoboRTS

Reference II: My TA GAO Wenliang's repository for visual inertial localization

https://github.com/gaowenliang

## Frame definition(right handed, all z axises heading upward)
1. World frame. At t = 0, x axis points to the starting direction of gimbal gunner, origin is at the starting rotation center of gimbal.
2. Chassis frame. Any time instant, x axis points to current direction of soldier heading, origin is at the current camera position on the chassis.
3. Transition frame. At any time instant, x axis points to current direction of soldier heading, origin is at the current rotation center of gimbal.
4. Gimbal frame. At any time instant, x axis points to current direction of gimbal gunner, origin is at the current rotation center of gimbal.

## For NVIDIA TX2
1. flycapture has one extra step to configure, see https://www.ptgrey.com/tan/10699
2. for rqt_multiplot, `sudo apt install libqwt-qt5-dev libqwt5-qt4 -y`, and in the running sequence, use `rosrun rqt_multiplot rqt_multiplot --force-discover`


## Network setting
-- rmsoldierX uses ip 192.168.1.22X in ASUS wifi network. All usernames are victory.
-- copy the file tools/lsusb.sh to /etc

## Attention
1. Never add CMakeList.txt to git repo.
2. Never add CMakeList.txt to git ignore.
3. There will be reminder that CMakeList.txt is not tracked in git. Just leave it there. Every computer has different src CMakeList.txt which linked to the place where ros is installed.

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
