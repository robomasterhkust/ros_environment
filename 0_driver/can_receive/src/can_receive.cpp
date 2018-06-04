#include <can_receive/dbus.h>
#include <can_msgs/Frame.h>
#include <string>
#include <ros/ros.h>

#define CAN_GIMBAL_BOARD_ID                         0x001
#define CAN_CHASSIS_BOARD_ID                        0x002

ros::Publisher dbus_publisher;
// ros::Publisher judge_publisher;

void msgCallback(const can_msgs::Frame &f)
{
    if (f.id == CAN_GIMBAL_BOARD_ID) {
        can_receive::dbus dbus_msg;
        dbus_msg.header.stamp = f.header.stamp;
        dbus_msg.header.frame_id = "world";
        dbus_msg.channel0 = (uint16_t) f.data[1] << 8 | (uint16_t) f.data[0];
        dbus_msg.channel1 = (uint16_t) f.data[3] << 8 | (uint16_t) f.data[2];
        dbus_msg.s1 = (uint16_t) f.data[4];
        dbus_msg.s2 = (uint16_t) f.data[5];
        dbus_msg.key_code = (uint16_t) f.data[7] << 8 | (uint16_t) f.data[6];

        dbus_publisher.publish(dbus_msg);
    }
    else if (f.id == CAN_CHASSIS_BOARD_ID) {

    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "can_receive_node");
    ros::NodeHandle nh("~"), nh_param("~");

    std::string can_device;
    nh_param.param<std::string>("can_device", can_device, "can1");

    dbus_publisher = nh.advertise<can_receive::dbus>("dbus", 100);
//    judge_publisher = nh.advertise<can_receive::judge>("info", 100);
    ros::Subscriber can_subscriber = nh.subscribe("/" + can_device + "_raw", 100, msgCallback);

    ros::spin();
}
