/**
 * @brief 
 * 
 * @file ROSHandle.cpp
 * @author Alex Au
 * @date 2018-03-27
 */
#include <cstring>
#include "defines.hpp"
#include "ROSHandle.hpp"

#define COOR_TOPIC_NAME "target_coor"

ROSHandle::ROSHandle(int argc, char **argv)
//: loop_rate(50)
{
    //ROS Init
    ros::init(argc, argv, "CV_node");
    rosNodeHandle = new ros::NodeHandle;
    cv_result_publisher = new ros::Publisher(rosNodeHandle->advertise<geometry_msgs::Point>(COOR_TOPIC_NAME, 1));
    //terminate_sub = new ros::Subscriber(rosNodeHandle->subscribe(TOPIC_NAME_TERMINATE, 1, &ROSHandle::termProgram, this));
};

ROSHandle::~ROSHandle()
{
    delete cv_result_publisher;
    delete rosNodeHandle;
}

// void ROSHandle::setTargetCoor(const double &x, const double &y, const double &z)
// {
//     msg_Point_Target.x = x;
//     msg_Point_Target.y = y;
//     msg_Point_Target.z = z;
// };

void ROSHandle::publishCoor(cv::Vec3d targetCoor)
{
    geometry_msgs::Point msg_Point_Target;
    msg_Point_Target.x = targetCoor[0];
    msg_Point_Target.y = targetCoor[1];
    msg_Point_Target.z = targetCoor[2];
    cv_result_publisher->publish(msg_Point_Target);
    std::cout << "published " << targetCoor << " to ros\n";

    //ros::spinOnce() will call all the callbacks waiting to be called at that point in time.
    ros::spinOnce();
}

void ROSHandle::termProgram(const std_msgs::Empty &in)
{
    throw;
}