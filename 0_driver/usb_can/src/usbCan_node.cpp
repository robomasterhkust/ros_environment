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
#include "usb_can/can_frame.h"
#include "ros/ros.h"

ros::NodeHandle *nh;
SerialCan *comObj = NULL;

void subCB(const usb_can::can_frame &msg)
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
    ros::Publisher pub = nh->advertise<usb_can::can_frame>("canRx", 20);
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

    while (ros::ok())
    {
        uint8_t temp[8] = {1, 2, 3, 4, 5, 6, 7, 8};
        comObj->sendCanMsg(1234,
                           0,
                           0,
                           8,
                           temp);
        sleep(1);
    }

    // int i = 0;
    // while (1)
    // {
    //     char data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    //     comObj->sendCanMsg(i, true, false, 8, (uint8_t *)data);
    //     i++;
    //     if (i > 9999)
    //         i = 0;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
}
