# Estimator
The modules to integrate multiple sensors. Currently the loosely coupled was untested, and can find reference in /docs

## CAN ROS Dependency
- realtime_tools
- ros_canopen
- ros_control

## IMU/A3 ROS Dependency
- DJI-SDK https://github.com/dji-sdk/Onboard-SDK
- DJI-SDK-ROS http://wiki.ros.org/dji_sdk

To make it works, follow the tutorial on http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started

To finish the DJI SDK requirement, add the following line to the ~/.bashrc
```bash
export DJIROS_APPID='1005202'
export DJIROS_ENCKEY='e7666379497e235364d54c3ecdf1370cbfa009f72e15f6a64451f38e1e5b83ba'
```
