This package mainly consist of two parts: drivers to read image frames, and the detector of armor plates

# TODOs: 
- This todo list
- This readme
- debugging strange behaviours
- parameter tweaking
- recognizing graphics painted on armors
- additional detection methods (most likely dig into machine learning)
- more fail-safe exception handling, e.g. when some camera can't be read suddenly

## Environment Setup
Add following commands to ~/.bashrc if you want it permanent, these commands also require `source ~/.bashrc` to take effect in your terminal
1. `export $ROS_HOME={desired configuration directory}`
1. `source {catkin_workspace}\devel\setup.bash`

## To build and run:
1. In your {catkin_workspace}\src directory run `git clone git@github.com:robomasterhkust/RMComputerVision.git`
2. `catkin_make --pkg rm_cv`
5. copy the default setting files from the folder stableConfigs\ to your $ROS_HOME directory
7. `Reconfig the setting files according to your need`

## About the CMakeLists.txt
Just take care with these lines
```
set (WITH_MINDVISION true) #whether compile with MindVision camera support
set (WITH_FLYCAP true) #whether compile with FLIR camera support

```

## Configurations and Parameters
This ROS node will try to read configurations stored in .xml formats in your $ROS_HOME directory, please specify this variable to make sure you know where the configuration files are.
`To start with, please copy all the contents in stableConfigs/ to your $ROS_HOME directory, the program should run if a webcam or a MindVision industrial camera is connected`
