/**
 * @brief
 *
 * @file ROSInterface.cpp
 * @author Alex Au
 * @date 2018-03-27
 */
#include <cstring>
#include "defines.hpp"
#include "ROSInterface.hpp"
#include "rm_cv/ArmorRecord.h"
#include "rm_cv/vertice.h"

ROSInterface::ROSInterface(int argc, char **argv)
{
    //ROS Init
    ros::init(argc, argv, "cv_armor_detection");
    rosNodeHandle = new ros::NodeHandle;
    spinner = new ros::AsyncSpinner(2);
    
    armor_publisher = ros::Publisher(rosNodeHandle->advertise<rm_cv::ArmorRecord>(TOPIC_NAME_ARMORS, 5));
    vertice_publisher = ros::Publisher(rosNodeHandle->advertise<rm_cv::vertice>(TOPIC_NAME_VERTICE, 5));
    spinner->start();
};

ROSInterface::~ROSInterface()
{
    delete rosNodeHandle;
    delete spinner;
}

bool ROSInterface::tryProcess(ConcurrentQueue<ArmorStorage> &inputQ, ConcurrentQueue<ArmorStorage> &outputQueue)
{
    ArmorStorage *armors = NULL;
    if (inputQ.dequeue(armors))
    {
        publishCoors(*armors);
        outputQueue.enqueue(armors);
        return true;
    }
    return false;
};

void ROSInterface::publishCoors(ArmorStorage &input)
{
    rm_cv::ArmorRecord pubArmorRecord;
    rm_cv::vertice vertice;

    pubArmorRecord.header = input.rosheader;
    vertice.header = input.rosheader;
    for (auto someArmor : input.armors)
    {
        pubArmorRecord.armorPose.angular.x = someArmor.rotation[0];
        pubArmorRecord.armorPose.angular.y = someArmor.rotation[1];
        pubArmorRecord.armorPose.angular.z = someArmor.rotation[2];
        pubArmorRecord.armorPose.linear.x = someArmor.translation[0];
        pubArmorRecord.armorPose.linear.y = someArmor.translation[1];
        pubArmorRecord.armorPose.linear.z = someArmor.translation[2];
        pubArmorRecord.isBlue = someArmor.isBlue;
        pubArmorRecord.isBig = someArmor.isBig;
        for (int i = 0; i < 9; i++)
        {
            pubArmorRecord.translationCovariance.at(i) = someArmor.translationCov(i / 3, i % 3);
        }
        for (int i = 0; i < 4 && i < someArmor.vertices.size(); i++)
        {
            vertice.vertex[i].x = someArmor.vertices[i].x;
            vertice.vertex[i].y = someArmor.vertices[i].y;
        }
        armor_publisher.publish(pubArmorRecord);
        vertice_publisher.publish(vertice);
    }
    ROS_INFO("Published %d armors", (int)input.armors.size());
};