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
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#include <stdexcept>
#include <string>
#include <signal.h>

ros::NodeHandle *nh;
SerialCan *comObj = NULL;

#define CAN_NVIDIA_TX2_BOARD_ID 0x103

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
        ros::init(argc, argv, "usb_can");

        nh = new ros::NodeHandle("~");

        std::string usb_can_path = "/dev/ttyUSB";

        int force_ttyUSB_index;
        nh->getParam("force_ttyUSB_index", force_ttyUSB_index);

        if (force_ttyUSB_index <= 0)
        {
                std::string result = exec("bash /etc/lsusb.sh");
                std::cout << result;
                std::size_t found = result.find(usb_can_path);
                if (found != std::string::npos)
                {
                        std::cout << "usb can: " << result[found + usb_can_path.length()] << std::endl;
                        usb_can_path.push_back(result[found + usb_can_path.length()]);
                }
        }
        else
        {
                usb_can_path += std::to_string(force_ttyUSB_index);
        }

        ros::Publisher pub = nh->advertise<can_msgs::Frame>("canRx", 20);
        ros::Subscriber sub = nh->subscribe("/cmd_vel", 20, cmd_cb);
        // ros::Subscriber subFrame = nh->subscribe("/canTx", 20, subCB);
        ros::AsyncSpinner spinner(1);

        ROS_INFO("Opening %s as usb to can module ...", usb_can_path.c_str());

        int buad_rate;
        nh->getParam("buad_rate", buad_rate);
        std::cout << buad_rate;
        switch (buad_rate)
        {
        case 230400:
                std::cout << " at baud rate 230400\n";
                comObj = createSerialCom(usb_can_path, B230400, pub);
                break;
        case 460800:
                std::cout << " at baud rate 460800\n";
                comObj = createSerialCom(usb_can_path, B460800, pub);
                break;
        default:
                std::cout << " at baud rate 115200\n";
                comObj = createSerialCom(usb_can_path, B115200, pub);
                break;
        }
        if (comObj)
        {
                comObj->startReadThd();
        }
        else
        {
                return 1;
        }

        ros::Rate r(1000);
        //char sth[9] = "12345678";
        //int count = 0;
        while (ros::ok())
        {
                //      std::cout << "usb to can on " << usb_can_path << " running...\n";
                //    comObj->sendCanMsg(count, 1, 0, 8, (uint8_t *)sth);
                //   if (count > 100)
                //         count = 0;
                // else
                //         count++;
                // r.sleep();
        }
        comObj->stopReadThd();
        return 0;
}
