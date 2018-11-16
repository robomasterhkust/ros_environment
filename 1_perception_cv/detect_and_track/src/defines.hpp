//general settings
#define CAM_CONFIG_FOLDER_STR "camConfigs/"
#define MAX_CAM_COUNT 10
#define V4LCamera 0
#define MVSDKDriver 1
#define ROS_IMAGE_IN 2
#define FLYCAP_CAMERA 3

//Armor tracker settings
#define PREDICT_DEGREE 1
#define PREDICT_DECAY 0.7
#define GROUP_SEP_DIST_SQUARE_RATIO 0.04
#define ARMOR_EXPIRE_TIME_SEC 0.1

//-------------------Armor detection settings

//in degree, the angle between the normal of the armor plate surface and
#define ARMOR_MAX_VISIBLE_ANGLE 60

//dimemsions in mm
#define SMALL_ARMOR_WIDTH 130
#define BIG_ARMOR_WIDTH 225
#define ALL_ARMOR_HEIGHT 55

//default countour filtering criteria
//DEFAULT ONLY, CAN BE CAHNGED BY THE "settings.xml"!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define ARMOR_LIGHT_MIN_RATIO 2.6
#define ARMOR_LIGHT_MIN_AREA 10
#define ARMOR_LIGHT_MAX_TILT 40

#define ARMOR_GROUPING_MAX_TILT_DIFF 25
#define ARMOR_GROUPING_MAX_ANGULAR_POS_DIFF 30
#define ARMOR_GROUPING_MIN_AREA 50
#define ARMOR_GROUPING_MAX_ASPECT_RATIO ((BIG_ARMOR_WIDTH * 1.25) / ALL_ARMOR_HEIGHT)
#define ARMOR_GROUPING_MIN_ASPECT_RATIO ((SMALL_ARMOR_WIDTH * sin(ARMOR_MAX_VISIBLE_ANGLE / 180.0 * 3.141592653)) / ALL_ARMOR_HEIGHT)
#define ARMOR_GROUPING_BIG_SMALL_ASPECT_RATIO_THRESHOLD ((SMALL_ARMOR_WIDTH * 1.25) / ALL_ARMOR_HEIGHT)
//ARMOR_GROUPING_MAX_LIGHT_LENGTH_DIFF: max (difference in length of the two light beams) / (sum of their lengths)
#define ARMOR_GROUPING_MAX_LIGHT_LENGTH_DIFF 0.4 / 1.6

#define ARMOR_LIGHT_BGRMinBlue cv::Vec3i(180, 0, 0);
#define ARMOR_LIGHT_BGRMaxBlue cv::Vec3i(255, 245, 245);
#define ARMOR_LIGHT_BGRMinRed cv::Vec3i(0, 0, 180);
#define ARMOR_LIGHT_BGRMaxRed cv::Vec3i(245, 245, 255);

#define ARMOR_LIGHT_HSVMinBlue cv::Vec3i(180, 150, 150);
#define ARMOR_LIGHT_HSVMaxBlue cv::Vec3i(255, 255, 255);
#define ARMOR_LIGHT_HSVMinRed cv::Vec3i(0, 150, 150);
#define ARMOR_LIGHT_HSVMaxRed cv::Vec3i(50, 255, 255);

// #define LIGHT_MERGE_XY_SPAN_PORTION 1.0
// #define LIGHT_MERGE_ORIENTATION_DIFF_RAD 0.15
// #define LIGHT_MERGE_RELATIVE_DIFF_RAD 0.15
#define LIGHT_MERGE_MAX_PERPENDICULAR_DISTANCE 4
#define LIGHT_MERGE_MAX_DISTANCE_MULT 0.7

//Shooter Parameters

//when no updates to the shooter is performed in this time period, it will give zero outputs
#define SHOOTER_INPUT_TIMEOUT 2.0

//-----------------------ROS related
//topic name used by the program to access parameters, e.g. PID parameters, should be the same as the name of the compiled node
#define DETECTION_NODE_NAME "/cv_ad"
#define SHOOTER_NODE_NAME "/cv_shooter"
//this topic receives a call of std_msgs::empty to terminate the program in case everything else does not work, which happens
#define TOPIC_NAME_TERMINATE "cv_ad_terminate_call"

//the topic where a geometry_msgs::Vector3, with .x=target pitch velocity, .y=target pitch velocity,
//also z= target forward velocity
//containing the result of armor detection together processed as gimbal speed command to aim will be published to
#define TOPIC_NAME_OUTPUT_GIMBAL_V "gimbal_target_veloity"

//once this topic reveive a std_msgs::empty, the gimble controller will update its pid parameters with the configured ROS parameters
//this is for tuning only, the final result should be stored elsewhere, or in the launch file
#define TOPIC_NAME_PARAMETER_UPDATE "updataParamCall"

#define TOPIC_NAME_ARMORS "detected_armor"
#define TOPIC_NAME_VERTICE "detected_vertice"
