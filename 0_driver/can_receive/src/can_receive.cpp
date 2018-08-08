#include <can_receive_msg/dbus.h>

#include <can_receive_msg/gameinfo.h>
#include <can_receive_msg/imu_16470.h>
#include <can_receive_msg/location_xy.h>
#include <can_receive_msg/location_zyaw.h>
#include <can_receive_msg/motor_debug.h>
#include <can_receive_msg/power_buffer.h>
#include <can_receive_msg/power_shooter_rfid_bufferinfo.h>
#include <can_receive_msg/power_vol_cur.h>
#include <can_receive_msg/projectile_hlth.h>
#include "geometry_msgs/QuaternionStamped.h"

#include <std_msgs/Header.h>

#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <string>

#define CAN_GIMBAL_BOARD_ID 0x001
#define CAN_CHASSIS_BOARD_ID 0x002
#define CAN_CHASSIS_BOARD_GAMEINFO_ID 0x003
#define CAN_CHASSIS_BOARD_PROJECTILE_HLTH_ID 0x004
#define CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID 0x005
#define CAN_CHASSIS_BOARD_VOLT_CURRENT_ID 0x006
#define CAN_CHASSIS_BOARD_SHOOTERHEAT_RFID_BUFFERINFO_ID 0x007
#define CAN_CHASSIS_BOARD_LOCATION_X_Y_ID 0x008
#define CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID 0x009
#define CAN_GIMBAL_SEND_16470_ID 0x010

#define CAN_CHASSIS_DEBUG_FR 0x211
#define CAN_CHASSIS_DEBUG_FL 0x212
#define CAN_CHASSIS_DEBUG_BL 0x213
#define CAN_CHASSIS_DEBUG_BR 0x214

ros::Publisher dbus_publisher;
ros::Publisher gameinfo_publisher;
ros::Publisher projectile_hlth_publisher;
ros::Publisher power_buffer_publisher;
ros::Publisher power_vol_cur_publisher;
ros::Publisher power_shooter_rfid_bufferinfo_publisher;
ros::Publisher location_xy_publisher;
ros::Publisher location_zyaw_publisher;
ros::Publisher attitude_publisher;
ros::Publisher motor_debug_publisher;

int16_t temp0_speed;
int16_t temp1_speed;
int16_t temp2_speed;
int16_t temp0_speed_curve;
int16_t temp1_speed_curve;
int16_t temp2_speed_curve;


float data_to_float(const can_msgs::Frame &f, int a, int b, int c, int d) {
  uint32_t hehe = ((uint32_t)(f.data[d] << 24) |
                   ((uint32_t)(f.data[c] << 16) |
                    ((uint32_t)(f.data[b] << 8) | (uint32_t)f.data[a])));
  float *haha = (float *)&hehe;
  return *haha;
}

void msgCallback(const can_msgs::Frame &f) {

  switch (f.id) {
  case CAN_GIMBAL_BOARD_ID: {
    can_receive_msg::dbus dbus_msg;
    dbus_msg.header.stamp = f.header.stamp;
    dbus_msg.header.frame_id = "world";
    dbus_msg.channel0 = (uint16_t)f.data[1] << 8 | (uint16_t)f.data[0];
    dbus_msg.channel1 = (uint16_t)f.data[3] << 8 | (uint16_t)f.data[2];
    dbus_msg.s1 = (uint16_t)f.data[4];
    dbus_msg.s2 = (uint16_t)f.data[5];
    dbus_msg.key_code = (uint16_t)f.data[7] << 8 | (uint16_t)f.data[6];

    dbus_publisher.publish(dbus_msg);
    break;
  }

  case CAN_CHASSIS_BOARD_GAMEINFO_ID: {
    can_receive_msg::gameinfo gameinfo_msg;
    gameinfo_msg.header.stamp = f.header.stamp;
    gameinfo_msg.header.frame_id = "world";
    gameinfo_msg.remainTime = (uint16_t)f.data[1] << 8 | (uint16_t)f.data[0];
    gameinfo_msg.gameStatus = (uint8_t)f.data[2];
    gameinfo_msg.robotLevel = (uint8_t)f.data[3];
    gameinfo_msg.remainHealth = (uint16_t)f.data[5] << 8 | (uint16_t)f.data[4];
    gameinfo_msg.fullHealth = (uint16_t)f.data[7] << 8 | (uint16_t)f.data[6];

    gameinfo_publisher.publish(gameinfo_msg);
    break;
  }
  case CAN_CHASSIS_BOARD_PROJECTILE_HLTH_ID: {
    can_receive_msg::projectile_hlth projectile_hlth_msg;
    projectile_hlth_msg.header.stamp = f.header.stamp;
    projectile_hlth_msg.header.frame_id = "world";
    projectile_hlth_msg.bulletSpeed = data_to_float(f, 0, 1, 2, 3);
    projectile_hlth_msg.bulletType = (uint8_t)f.data[4];
    projectile_hlth_msg.bulletFreq = (uint8_t)f.data[5];
    projectile_hlth_msg.hitPos = (uint8_t)f.data[6];
    projectile_hlth_msg.deltaReason = (uint8_t)f.data[7];

    projectile_hlth_publisher.publish(projectile_hlth_msg);
    break;
  }
  case CAN_CHASSIS_BOARD_POWER_POWERBUFFER_ID: {
    can_receive_msg::power_buffer power_buffer_msg;
    power_buffer_msg.header.stamp = f.header.stamp;
    power_buffer_msg.header.frame_id = "world";
    power_buffer_msg.power = data_to_float(f, 0, 1, 2, 3);
    power_buffer_msg.powerBuffer = data_to_float(f, 4, 5, 6, 7);

    power_buffer_publisher.publish(power_buffer_msg);
    break;
  }

  case CAN_CHASSIS_BOARD_VOLT_CURRENT_ID: {
    can_receive_msg::power_vol_cur power_vol_cur_msg;
    power_vol_cur_msg.header.stamp = f.header.stamp;
    power_vol_cur_msg.header.frame_id = "world";
    power_vol_cur_msg.volt = data_to_float(f, 0, 1, 2, 3);
    power_vol_cur_msg.current = data_to_float(f, 4, 5, 6, 7);

    power_vol_cur_publisher.publish(power_vol_cur_msg);
    break;
  }

  case CAN_CHASSIS_BOARD_SHOOTERHEAT_RFID_BUFFERINFO_ID: {
    can_receive_msg::power_shooter_rfid_bufferinfo
        power_shooter_rfid_bufferinfo_msg;
    power_shooter_rfid_bufferinfo_msg.header.stamp = f.header.stamp;
    power_shooter_rfid_bufferinfo_msg.header.frame_id = "world";
    power_shooter_rfid_bufferinfo_msg.shooterHeat0 =
        (uint16_t)f.data[1] << 8 | (uint16_t)f.data[0];
    power_shooter_rfid_bufferinfo_msg.shooterHeat1 =
        (uint16_t)f.data[3] << 8 | (uint16_t)f.data[2];
    power_shooter_rfid_bufferinfo_msg.cardType = (uint8_t)f.data[4];
    power_shooter_rfid_bufferinfo_msg.cardIdx = (uint8_t)f.data[5];
    power_shooter_rfid_bufferinfo_msg.powerUpType = (uint8_t)f.data[6];
    power_shooter_rfid_bufferinfo_msg.powerUpPercentage = (uint8_t)f.data[7];

    power_shooter_rfid_bufferinfo_publisher.publish(
        power_shooter_rfid_bufferinfo_msg);
    break;
  }

  case CAN_CHASSIS_BOARD_LOCATION_X_Y_ID: {
    can_receive_msg::location_xy location_xy_msg;
    location_xy_msg.header.stamp = f.header.stamp;
    location_xy_msg.header.frame_id = "world";
    location_xy_msg.x = data_to_float(f, 0, 1, 2, 3);
    location_xy_msg.y = data_to_float(f, 4, 5, 6, 7);

    location_xy_publisher.publish(location_xy_msg);
    break;
  }

  case CAN_CHASSIS_BOARD_LOCATION_Z_YAW_ID: {
    can_receive_msg::location_zyaw location_zyaw_msg;
    location_zyaw_msg.header.stamp = f.header.stamp;
    location_zyaw_msg.header.frame_id = "world";
    location_zyaw_msg.z = data_to_float(f, 0, 1, 2, 3);
    location_zyaw_msg.yaw = data_to_float(f, 4, 5, 6, 7);

    location_zyaw_publisher.publish(location_zyaw_msg);
    break;
  }

  case CAN_GIMBAL_SEND_16470_ID: {
    geometry_msgs::QuaternionStamped imu_16470_msg;
    imu_16470_msg.header = f.header;
    int16_t a = (int16_t)((uint16_t)f.data[1] << 8 | (uint16_t)f.data[0]);
    int16_t b = (int16_t)((uint16_t)f.data[3] << 8 | (uint16_t)f.data[2]);
    int16_t c = (int16_t)((uint16_t)f.data[5] << 8 | (uint16_t)f.data[4]);
    int16_t d = (int16_t)((uint16_t)f.data[7] << 8 | (uint16_t)f.data[6]);
    imu_16470_msg.quaternion.w = a * 0.001;
    imu_16470_msg.quaternion.x = b * 0.001;
    imu_16470_msg.quaternion.y = c * 0.001;
    imu_16470_msg.quaternion.z = d * 0.001;

    attitude_publisher.publish(imu_16470_msg);

    break;
  }

  case CAN_CHASSIS_DEBUG_FR: {
    temp0_speed       = (int16_t)((uint16_t)f.data[1] << 8 | (uint16_t)f.data[0]);
    temp0_speed_curve = (int16_t)((uint16_t)f.data[3] << 8 | (uint16_t)f.data[2]);
    break;
  }

  case CAN_CHASSIS_DEBUG_FL: {
    temp1_speed       = (int16_t)((uint16_t)f.data[1] << 8 | (uint16_t)f.data[0]);
    temp1_speed_curve = (int16_t)((uint16_t)f.data[3] << 8 | (uint16_t)f.data[2]);
    break;
  }

  case CAN_CHASSIS_DEBUG_BL: {
    temp2_speed       = (int16_t)((uint16_t)f.data[1] << 8 | (uint16_t)f.data[0]);
    temp2_speed_curve = (int16_t)((uint16_t)f.data[3] << 8 | (uint16_t)f.data[2]);
    break;
  }

  case CAN_CHASSIS_DEBUG_BR: {
    can_receive_msg::motor_debug motor_debug_msg;
    motor_debug_msg.speed_[0] = temp0_speed / 60.0f;
    motor_debug_msg.speed_[1] = temp1_speed / 60.0f;
    motor_debug_msg.speed_[2] = temp2_speed / 60.0f;
    motor_debug_msg.speed_curve[0] = temp0_speed_curve / 60.0f;
    motor_debug_msg.speed_curve[1] = temp1_speed_curve / 60.0f;
    motor_debug_msg.speed_curve[2] = temp2_speed_curve / 60.0f;
    int16_t tem3_speed        = (int16_t)((uint16_t)f.data[1] << 8 | (uint16_t)f.data[0]);
    int16_t temp3_speed_curve = (int16_t)((uint16_t)f.data[1] << 8 | (uint16_t)f.data[0]);

    motor_debug_msg.speed_[3] = tem3_speed / 60.0f;
    motor_debug_msg.speed_curve[3] = temp3_speed_curve / 60.0f;
    motor_debug_msg.header = f.header;
    motor_debug_publisher.publish(motor_debug_msg);
    break;
  }
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "can_receive_node");
  ros::NodeHandle nh("~"), nh_param("~");

  dbus_publisher = nh.advertise<can_receive_msg::dbus>("dbus", 100);
  gameinfo_publisher = nh.advertise<can_receive_msg::gameinfo>("gameinfo", 100);
  projectile_hlth_publisher =
      nh.advertise<can_receive_msg::projectile_hlth>("projectile_hlth", 100);
  location_zyaw_publisher =
      nh.advertise<can_receive_msg::location_zyaw>("location_zyaw", 100);
  location_xy_publisher =
      nh.advertise<can_receive_msg::location_xy>("location_xy", 100);
  power_shooter_rfid_bufferinfo_publisher =
      nh.advertise<can_receive_msg::power_shooter_rfid_bufferinfo>(
          "power_shooter_rfid_bufferinfo", 100);
  power_vol_cur_publisher =
      nh.advertise<can_receive_msg::power_vol_cur>("power_vol_cur", 100);
  power_buffer_publisher =
      nh.advertise<can_receive_msg::power_buffer>("power_buffer", 100);
  attitude_publisher =
      nh.advertise<geometry_msgs::QuaternionStamped>("attitude", 100);
  motor_debug_publisher =
      nh.advertise<can_receive_msg::motor_debug>("motor_debug", 100);

  ros::Subscriber can_subscriber =
      nh.subscribe("/canusb/canRx", 100, msgCallback);

  ros::spin();

  ros::waitForShutdown();
}
