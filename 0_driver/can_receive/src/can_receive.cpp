#include <can_receive/dbus.h>

#include <can_receive/gameinfo.h>
#include <can_receive/hlth.h>
#include <can_receive/projectile.h>
#include <can_receive/power_buffer.h>
#include <can_receive/power_vol_cur.h>
#include <can_receive/power_shooter.h>
#include <can_receive/rfid.h>
#include <can_receive/bufferinfo.h>
#include <can_receive/location_xy.h>
#include <can_receive/location_zyaw.h>

#include <can_msgs/Frame.h>
#include <string>
#include <ros/ros.h>

#define CAN_GIMBAL_BOARD_ID                         0x001
#define CAN_CHASSIS_BOARD_ID                        0x002
#define CAN_CHASSIS_BOARD_GAMEINFO_ID                0x003
#define CAN_CHASSIS_BOARD_HLTH_ID                    0x004
#define CAN_CHASSIS_BOARD_PROJECTILE_ID                0x005
#define CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID        0x006
#define CAN_CHASSIS_BOARD_VOLT_CURRENT_ID            0x007
#define CAN_CHASSIS_BOARD_SHOOTERHEAT_ID            0x008
#define CAN_CHASSIS_BOARD_RFID_ID                    0x009
#define CAN_CHASSIS_BOARD_BUFFERINFO_ID                0x010
#define CAN_CHASSIS_BOARD_LOCATION_X_Y_ID            0x011
#define CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID            0x012

ros::Publisher dbus_publisher;

ros::Publisher gameinfo_publisher;
ros::Publisher hlth_publisher;
ros::Publisher projectile_publisher;
ros::Publisher power_buffer_publisher;
ros::Publisher power_vol_cur_publisher;
ros::Publisher power_shooter_publisher;
ros::Publisher rfid_publisher;
ros::Publisher bufferinfo_publisher;
ros::Publisher location_xy_publisher;
ros::Publisher location_zyaw_publisher;

float data_to_float(const can_msgs::Frame &f, int a, int b, int c, int d){
    uint32_t hehe = ((uint32_t)(f.data[d]<<24)|((uint32_t)(f.data[c]<<16)|((uint32_t)(f.data[b]<<8)|(uint32_t)f.data[a])));
    float *haha = (float*)&hehe;
    return *haha;
}

void msgCallback(const can_msgs::Frame &f) {
    switch (f.id) {
        case CAN_GIMBAL_BOARD_ID: {
            can_receive::dbus dbus_msg;
            dbus_msg.header.stamp = f.header.stamp;
            dbus_msg.header.frame_id = "world";
            dbus_msg.channel0 = (uint16_t) f.data[1] << 8 | (uint16_t) f.data[0];
            dbus_msg.channel1 = (uint16_t) f.data[3] << 8 | (uint16_t) f.data[2];
            dbus_msg.s1 = (uint16_t) f.data[4];
            dbus_msg.s2 = (uint16_t) f.data[5];
            dbus_msg.key_code = (uint16_t) f.data[7] << 8 | (uint16_t) f.data[6];

            dbus_publisher.publish(dbus_msg);
            break;
        }

        case CAN_CHASSIS_BOARD_GAMEINFO_ID: {
            can_receive::gameinfo gameinfo_msg;
            gameinfo_msg.header.stamp = f.header.stamp;
            gameinfo_msg.header.frame_id = "world";
            gameinfo_msg.remainTime = (uint16_t) f.data[1] << 8 | (uint16_t) f.data[0];
            gameinfo_msg.gameStatus = (uint8_t) f.data[2];
            gameinfo_msg.robotLevel = (uint8_t) f.data[3];
            gameinfo_msg.remainHealth = (uint16_t) f.data[5] << 8 | (uint16_t) f.data[4];
            gameinfo_msg.fullHealth = (uint16_t) f.data[7] << 8 | (uint16_t) f.data[6];

            gameinfo_publisher.publish(gameinfo_msg);
            break;
        }
        case CAN_CHASSIS_BOARD_HLTH_ID: {
            can_receive::hlth hlth_msg;
            hlth_msg.header.stamp = f.header.stamp;
            hlth_msg.header.frame_id = "world";
            hlth_msg.hitPos = (uint8_t) f.data[0];
            hlth_msg.deltaReason = (uint8_t) f.data[1];

            hlth_publisher.publish(hlth_msg);
            break;
        }
        case CAN_CHASSIS_BOARD_PROJECTILE_ID: {
            can_receive::projectile projectile_msg;
            projectile_msg.header.stamp = f.header.stamp;
            projectile_msg.header.frame_id = "world";
            projectile_msg.bulletType = (uint8_t) f.data[0];
            projectile_msg.bulletFreq = (uint8_t) f.data[1];
            projectile_msg.bulletSpeed = data_to_float(f,2,3,4,5);

            projectile_publisher.publish(projectile_msg);
            break;
        }
        case CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID: {
            can_receive::power_buffer power_buffer_msg;
            power_buffer_msg.header.stamp = f.header.stamp;
            power_buffer_msg.header.frame_id = "world";
            power_buffer_msg.power = data_to_float(f,0,1,2,3);
            power_buffer_msg.powerBuffer = data_to_float(f,4,5,6,7);

            power_buffer_publisher.publish(power_buffer_msg);
            break;
        }

        case CAN_CHASSIS_BOARD_VOLT_CURRENT_ID: {
            can_receive::power_vol_cur power_vol_cur_msg;
            power_vol_cur_msg.header.stamp = f.header.stamp;
            power_vol_cur_msg.header.frame_id = "world";
            power_vol_cur_msg.volt = data_to_float(f,0,1,2,3);
            power_vol_cur_msg.current = data_to_float(f,4,5,6,7);
                    
            power_vol_cur_publisher.publish(power_vol_cur_msg);
            break;
        }

        case CAN_CHASSIS_BOARD_SHOOTERHEAT_ID: {
            can_receive::power_shooter power_shooter_msg;
            power_shooter_msg.header.stamp = f.header.stamp;
            power_shooter_msg.header.frame_id = "world";
            power_shooter_msg.shooterHeat0 = (uint16_t) f.data[1] << 8 | (uint16_t) f.data[0];
            power_shooter_msg.shooterHeat1 = (uint16_t) f.data[3] << 8 | (uint16_t) f.data[2];

            power_shooter_publisher.publish(power_shooter_msg);
            break;
        }

        case CAN_CHASSIS_BOARD_RFID_ID: {
            can_receive::rfid rfid_msg;
            rfid_msg.header.stamp = f.header.stamp;
            rfid_msg.header.frame_id = "world";
            rfid_msg.cardType = (uint8_t) f.data[0];
            rfid_msg.cardIdx = (uint8_t) f.data[1];

            rfid_publisher.publish(rfid_msg);
            break;
        }

        case CAN_CHASSIS_BOARD_BUFFERINFO_ID: {
            can_receive::bufferinfo bufferinfo_msg;
            bufferinfo_msg.header.stamp = f.header.stamp;
            bufferinfo_msg.header.frame_id = "world";
            bufferinfo_msg.powerUpType = (uint8_t) f.data[0];
            bufferinfo_msg.powerUpPercentage = (uint8_t) f.data[1];

            bufferinfo_publisher.publish(bufferinfo_msg);
            break;
        }
        case CAN_CHASSIS_BOARD_LOCATION_X_Y_ID: {
            can_receive::location_xy location_xy_msg;
            location_xy_msg.header.stamp = f.header.stamp;
            location_xy_msg.header.frame_id = "world";
            location_xy_msg.x = data_to_float(f,0,1,2,3);
            location_xy_msg.y = data_to_float(f,4,5,6,7);

            location_xy_publisher.publish(location_xy_msg);
            break;
        }

        case CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID: {
            can_receive::location_zyaw location_zyaw_msg;
            location_zyaw_msg.header.stamp = f.header.stamp;
            location_zyaw_msg.header.frame_id = "world";
            location_zyaw_msg.z = data_to_float(f,0,1,2,3);
            location_zyaw_msg.yaw =data_to_float(f,4,5,6,7);

            location_zyaw_publisher.publish(location_zyaw_msg);
            break;
        }
    }

}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "can_receive_node");
    ros::NodeHandle nh("~"), nh_param("~");

    std::string can_device;
    nh_param.param<std::string>("can_device", can_device, "can1");

    dbus_publisher = nh.advertise<can_receive::dbus>("dbus", 100);

    gameinfo_publisher = nh.advertise<can_receive::gameinfo>("gameinfo", 100);
    hlth_publisher = nh.advertise<can_receive::hlth>("hlth", 100);
    location_zyaw_publisher = nh.advertise<can_receive::location_zyaw>("location_zyaw", 100);
    location_xy_publisher = nh.advertise<can_receive::location_xy>("location_xy", 100);
    projectile_publisher = nh.advertise<can_receive::projectile>("projectile", 100);
    rfid_publisher = nh.advertise<can_receive::rfid>("rfid", 100);
    bufferinfo_publisher = nh.advertise<can_receive::bufferinfo>("bufferinfo", 100);
    power_shooter_publisher = nh.advertise<can_receive::power_shooter>("power_shooter", 100);
    power_vol_cur_publisher = nh.advertise<can_receive::power_vol_cur>("power_vol_cur", 100);
    power_buffer_publisher = nh.advertise<can_receive::power_buffer>("power_buffer", 100);

    ros::Subscriber can_subscriber = nh.subscribe("/" + can_device + "_raw", 100, msgCallback);

    ros::spin();
}
