#define DEBUG
//general settings
#define THREAD_COUNT 3
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

//countour filtering criteria
#define ARMOR_LIGHT_MIN_RATIO 3
#define ARMOR_LIGHT_MIN_AREA 10
#define ARMOR_LIGHT_MAX_TILT 40

#define ARMOR_GROUPING_MAX_TILT_DIFF 10
#define ARMOR_GROUPING_MAX_ANGULAR_POS_DIFF 20
#define ARMOR_GROUPING_MIN_AREA 50
#define ARMOR_GROUPING_MAX_ASPECT_RATIO ((BIG_ARMOR_WIDTH * 2) / ALL_ARMOR_HEIGHT)

#define ARMOR_LIGHT_BGRMinBlue Scalar(180, 0, 0);
#define ARMOR_LIGHT_BGRMaxBlue Scalar(255, 255, 255);
#define ARMOR_LIGHT_BGRMinRed Scalar(0, 0, 180);
#define ARMOR_LIGHT_BGRMaxRed Scalar(255, 255, 255);

//-----------------------ROS related
//topic name used by the program to access parameters, e.g. PID parameters, should be the same as the name of the compiled node
#define NODE_NAME "/cv_ad"
//this topic receives a call of std_msgs::empty to terminate the program in case everything else does not work, which happens
#define TOPIC_NAME_TERMINATE "cv_ad_terminate_call"

//the topic where a geometry_msgs::Vector3, with .x=target pitch velocity, .y=target pitch velocity,
//containing the result of armor detection together processed as gimbal speed command to aim will be published to
#define TOPIC_NAME_OUTPUT_GIMBAL_V "gimbal_target_veloity"

//once this topic reveive a std_msgs::empty, the gimble controller will update its pid parameters with the configured ROS parameters
//this is for tuning only, the final result should be stored elsewhere, or in the launch file
#define TOPIC_NAME_PARAMETER_UPDATE "updataParamCall"
