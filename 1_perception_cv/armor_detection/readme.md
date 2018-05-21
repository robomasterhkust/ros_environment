This is a standalone ROS package

to support thr Mindvision camera, copy lib/libMVSDK.so to your usr/local directory
otherwise, set USE_MVSDK in CMakeLists.txt to false

To build or run:
1. copy this to your {catkin_workspace}\src
2. catkin_make in your {catkin_workspace}
3. add "source {catkin_workspace}\devel\setup.bash" to your ~/.bashrc
4. add "export $ROS_HOME={desired configuration directory}" to your ~/.bashrc
5. use roslaunch to start the launch file under the launch folder
6. first time launching probably won't success, check and edit the configuration xml files in the directory in step 4, or just copy the preconfigured setting files from the folder stableConfigs\ 
7. roslaunch again

the current cmake configuration supports debugging, notice that some bugs may not appear when debugging (e.g. forbidden interaction with UI in other threads)

About the setting.xml and CamConfigs files:
-setting.xml is quite easy to understand, to setup a new robot, we need to specify a camera config xml file in the "camFileName" tag in the Cam0 - Cam4 section
-for per Camera setup, the xml files are stored in a subfolder "CamConfigs", in the xml file, the "driverType" tag specify the camera driver type: 0 means V4L driver (linux built-in for cheap webcams), 1 is for the mvux camera (800 rmb one from last year); the cameras calibration matrix can be obtained by using the provided tools with a grid, the "rotationVec" and "translationVec" specify the relative geometry of the camera to the gimbal, with x pointing to the right, y downwards and z forwards 
IMPORTANT: in setting.xml EnableSerialCom should be disabled for now 

TODO:
- tweaking the parameters
- improve filtering of noise to extract the armor better (experinment different thresholds?)
- distingish big and small armor
- camera driver for bluefox and more
- more fail-safe exception handling, e.g. when some camera can'r be read suddenly

=======
Plan:
1.ArmorDetection

	1.3 distance from stereo image

	1.4 filter invalid data from different methods of detection/ detection history
	
	1.6 minimize area of interest before processing according to past result to optimize performance
	
	1.7 experinment tracking of whole robot according to armor results, using camshift etc, first have to study how robots differ from the background in a image. this may require gpu.
	
2.Intelligence Robot control
	2.1 perform decision making arcordding to different data received from the robot

3.Communication
	3.1 Switch to can communication with robots, make a new node or repository for this, making sure its user friendly and object orientated
