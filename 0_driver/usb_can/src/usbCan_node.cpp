#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include "SerialCan.hpp"
#include "can_msgs/Frame.h"
#include "ros/ros.h"

ros::NodeHandle *nh;
SerialCan *comObj = NULL;

void subCB(const can_msgs::Frame &msg)
{
    comObj->sendCanMsg(msg.id,
                       msg.is_extended,
                       msg.is_rtr,
                       msg.dlc,
                       msg.data.begin());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "canusb0");
    nh = new ros::NodeHandle("~");
    ros::Publisher pub = nh->advertise<can_msgs::Frame>("canRx", 20);
    ros::Subscriber sub = nh->subscribe("canTx", 20, subCB);
    ros::AsyncSpinner spinner(1);
    std::string path;
    nh->getParam("path", path);

    ROS_INFO("Opening %s as usb to can module ...", path.c_str());

    comObj = createSerialCom(path, B115200, pub);

    if (comObj)
    {
        comObj->startReadThd();
    }
    else
    {
        return 1;
    }

    ros::spin();
}
