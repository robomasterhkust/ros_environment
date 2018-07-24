#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include "SerialCan.hpp"
#include "can_msgs/Frame.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include "ros/ros.h"
#include <stdexcept>
#include <string>

ros::NodeHandle *nh;
SerialCan *comObj = NULL;

#define CAN_NVIDIA_TX2_BOARD_ID 0x103
#define SENTRY_GIMBAL_CONTROL_ID 0x111
#define SENTRY_SHOOT_RAIL_CONTROL_ID 0x110

void subCB(const can_msgs::Frame &msg)
{
        comObj->sendCanMsg(msg.id,
                           msg.is_extended,
                           msg.is_rtr,
                           msg.dlc,
                           msg.data.begin());
}

void cmd_cb(const geometry_msgs::Twist &t)
{
        // void cmd_cb(const geometry_msgs::Vector3 &t){
        static can_msgs::Frame f;

        // ROS_INFO("Received cmd_vel py=%f vy=%f vw=%f",t.linear.x,t.linear.y,t.angular.z);
        //f.header.frame_id="0";
        f.header.stamp = ros::Time::now();

        f.id = CAN_NVIDIA_TX2_BOARD_ID;
        f.dlc = (16 / 8) * 4;

        int16_t py = (int16_t)(t.linear.y * 1000);  // convert to mm/s
        int16_t pz = (int16_t)(t.linear.z * 1000);  // convert to mm/s
        int16_t vy = (int16_t)(t.angular.y * 1000); // pitch, rotate by Y axis
        int16_t vz = (int16_t)(t.angular.z * 1000); // yaw,   rotate by Z axis
        // int16_t py = (int16_t) (t.z * 100000); // convert to mm/s
        // int16_t vy = (int16_t) (t.x * 100000); // convert to mm/s
        // int16_t vw = (int16_t) (t.y * 100000); // convert to mm/s
        f.data[1] = (uint8_t)(py >> 8) & 0xff;
        f.data[0] = (uint8_t)py & 0xff;

        f.data[3] = (uint8_t)(pz >> 8) & 0xff;
        f.data[2] = (uint8_t)pz & 0xff;

        f.data[5] = (uint8_t)(vy >> 8) & 0xff;
        f.data[4] = (uint8_t)vy & 0xff;

        f.data[7] = (uint8_t)(vz >> 8) & 0xff;
        f.data[6] = (uint8_t)vz & 0xff;

        subCB(f);
}

void sentryGimbalControlCB(const geometry_msgs::Twist &t)
{
        // void cmd_cb(const geometry_msgs::Vector3 &t){
        static can_msgs::Frame f;
        //gimabal control id: 0x111

        f.id = SENTRY_GIMBAL_CONTROL_ID;
        f.dlc = 8;
        f.is_extended = false;
        f.is_rtr = false;

        *(float *)&f.data.elems[0] = t.angular.z;
        *(float *)&f.data.elems[4] = t.angular.y;

        subCB(f);
}

void test_sentry_cb(const std_msgs::Int8 &in)
{
        ROS_INFO("test_sentry_cb");
        static can_msgs::Frame f;
        //gimabal control id: 0x111

        f.id = SENTRY_SHOOT_RAIL_CONTROL_ID;
        f.dlc = 8;
        f.is_extended = false;
        f.is_rtr = false;

        f.data.fill(0);
        f.data.elems[0] = in.data;

        subCB(f);
}

std::string exec(const char *cmd)
{
        char buffer[128];
        std::string result = "";
        FILE *pipe = popen(cmd, "r");
        if (!pipe)
                throw std::runtime_error("popen() failed!");
        try
        {
                while (!feof(pipe))
                {
                        if (fgets(buffer, 128, pipe) != NULL)
                                result += buffer;
                }
        }
        catch (...)
        {
                pclose(pipe);
                throw;
        }
        pclose(pipe);
        return result;
}

int main(int argc, char **argv)
{
        std::string usb_can = "/dev/ttyUSB";
        std::string result = exec("bash /etc/lsusb.sh");
        std::cout << result;
        std::size_t found = result.find(usb_can);
        if (found != std::string::npos)
        {
                std::cout << "usb can: " << result[found + usb_can.length()] << std::endl;
                usb_can.push_back(result[found + usb_can.length()]);
        }

        ros::init(argc, argv, "canusb");
        nh = new ros::NodeHandle("~");
        ros::Publisher pub = nh->advertise<can_msgs::Frame>("canRx", 20);
        ros::Subscriber sub = nh->subscribe("/cmd_vel", 20, cmd_cb);
        ros::Subscriber gimbalSub = nh->subscribe("gimbal_target_veloity", 20, sentryGimbalControlCB);
        ros::Subscriber subFrame = nh->subscribe("/canTx", 20, subCB);
        ros::Subscriber testSentrySub = nh->subscribe("/test_sentry", 20, test_sentry_cb);

        ROS_INFO("Opening %s as usb to can module ...", usb_can.c_str());

        int buad_rate;
        speed_t BaudR;
        nh->getParam("buad_rate", buad_rate);
        switch (buad_rate)
        {
        case 230400:
                ROS_INFO("Opening serial com with 230400 baud rate");
                BaudR = B230400;
                break;
        case 460800:
                ROS_INFO("Opening serial com with 460800 baud rate");
                BaudR = B460800;
                break;
        default:
                ROS_INFO("Opening serial com with 115200 baud rate");
                BaudR = B115200;
        }
        comObj = createSerialCom(usb_can, BaudR, pub);

        if (comObj)
        {
                comObj->startReadThd();
        }
        else
        {
                ROS_ERROR("Opening usb to can module FAILED");
                return 1;
        }

        ros::AsyncSpinner spinner(1);

        spinner.start();

        // geometry_msgs::Twist temptwist;

        // temptwist.angular.y = temptwist.angular.x = 0;

        // static can_msgs::Frame f;
        // f.id = SENTRY_SHOOT_RAIL_CONTROL_ID;
        // f.dlc = 8;
        // f.is_extended = false;
        // f.is_rtr = false;

        // f.data.fill(0);

        // *(float *)&f.data.elems[1] = 1;

        // f.data.elems[0] = 0;

        while (ros::ok())
        {

                std::cout << "usb to can running" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
        comObj->stopReadThd();
        delete comObj;
        spinner.stop();
}
