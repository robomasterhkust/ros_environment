#include "defines.hpp"
#include "GimbalController.hpp"
#include "interface.h"
#include <math.h>

using namespace std;
GimbalController::GimbalController(ros::NodeHandle *nh_ptr, bool enable_Serial)
    : nh_ptr(nh_ptr),
      last_pitch_e(0),
      last_yaw_e(0),
      pitch_i(0),
      yaw_i(0),
      pitch_v_out(0),
      yaw_v_out(0),
      tuner_sub(nh_ptr->subscribe(TOPIC_NAME_PARAMETER_UPDATE, 5, &GimbalController::updateParasCallback, this)),
      enable_Serial(enable_Serial),
      canPub(nh_ptr->advertise<geometry_msgs::Vector3>(TOPIC_NAME_OUTPUT_GIMBAL_V, 20))
{
    if (enable_Serial)
    {
        fcu_link = mavconn::MAVConnInterface::open_url("serial:///dev/ttyUSB0", 21, 78);

        fcu_link->set_protocol_version(mavconn::Protocol::V10);
        ROS_INFO("MAVLink Protocol Version [mavconn]: %u",
                 static_cast<unsigned int>(fcu_link->get_protocol_version()));
        mavlink_set_proto_version(MAVLINK_COMM_0, 1);
        ROS_INFO("MAVLink Protocol Version [main]: %u",
                 mavlink_get_proto_version(MAVLINK_COMM_0));

        fcu_link->message_received_cb = GimbalController::serialCallBack;
    }
    else
    {
        ROS_INFO("NOT ENABLING SERIAL, NO SERIAL COMMUNICATION IS ESTABLISHED");
    }

    updateParas();
    clock_gettime(CLOCK_MONOTONIC, &lastupdate_time);
};

bool GimbalController::updateParas()
{
    bool temp = true;
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/yaw_kp"), yaw_kp, 0.0002);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/yaw_ki"), yaw_ki, yaw_kp / 10.0);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/yaw_kd"), yaw_kd, yaw_kp / 10.0);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/pitch_kp"), pitch_kp, 0.0002);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/pitch_ki"), pitch_ki, pitch_kp / 10.0);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/pitch_kd"), pitch_kd, pitch_kp / 10.0);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/maxPitchVelocity"), maxPitch, 1.0);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/maxYawVelocity"), maxYaw, 1.0);
    temp &= nh_ptr->param<double>(string(NODE_NAME) + string("/GimbalControllerTimeout"), expireTime, 0.2);
    if (temp)
    {
        ROS_INFO("GimbalController::updateParas Success");
        return true;
    }
    else
    {
        ROS_INFO("GimbalController::updateParas Failed");
        return false;
    }
};

void GimbalController::updateParasCallback(const std_msgs::Empty &input)
{
    ROS_INFO("GimbalController::updatePIDCallback called");
    if (updateParas())
        ROS_INFO("GimbalController::updateGimbalController parameters success");
    else
        ROS_INFO(" [SHIT] GimbalController::updateGimbalController parameters FAILED");
};

// void GimbalController::sendOutToSerial()
// {
//     static mavlink_message_t mav_msg;
//     //mavlink_msg_set_attitude_target_pack_chan(21, 78, MAVLINK_COMM_0, &mav_msg, 0, 0, 0, 0, NULL, 0, gimbal_v.x, gimbal_v.y, 0);
//     mavlink_msg_attitude_pack_chan(21, 78, MAVLINK_COMM_0, &mav_msg, 0, 0, 0, 0, 0, pitch_v_out, yaw_v_out);
//     ROS_INFO("Sending pitch %f, yaw %f", pitch_v_out, yaw_v_out);
//     if (enable_Serial)
//         fcu_link->send_message(&mav_msg);
// };

void GimbalController::publishToCan()
{
    geometry_msgs::Vector3 tempvec;
    tempvec.x = pitch_v_out;
    tempvec.y = yaw_v_out;
    canPub.publish(tempvec);
}

bool GimbalController::updateTarget(float x, float y, float z)
{
    ROS_INFO("GimbalController::updateTarget(%f,%f,%f)", x, y, z);
    if (x == 0 && y == 0 && z == 0)
    {
        pitch_v_out = yaw_v_out = 0;
        return false; // angle undefined for (0,0,0)
    }

    timespec t_now;
    clock_gettime(CLOCK_MONOTONIC, &t_now);
    double dt = (t_now.tv_sec - lastupdate_time.tv_sec) + (t_now.tv_nsec - lastupdate_time.tv_nsec) / 1000000000.0;
    lastupdate_time = t_now;
    //std::cout << lastupdate_time.tv_sec << std::endl;
    double r = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

    //error angle in radian
    double pitch_e = asin(y / r); //positive pitch is down, right???
    double yaw_e = -asin(x / r);  //positive yaw is anticlockwise???
    cout << "yaw_e " << yaw_e << endl;
    //claculate differential term
    double pitch_d = (pitch_e - last_pitch_e) / dt;
    double yaw_d = (yaw_e - last_yaw_e) / dt;

    //update integral term
    double temp_pitch_i = pitch_i + (pitch_e + last_pitch_e) * dt / 2.0;
    double temp_yaw_i = yaw_i + (yaw_e + last_yaw_e) * dt / 2.0;

    pitch_v_out = pitch_kp * pitch_e + pitch_ki * temp_pitch_i + pitch_kd * pitch_d;
    yaw_v_out = yaw_kp * yaw_e + yaw_ki * temp_yaw_i + yaw_kd * yaw_d;

    //rectify output and integral
    if (pitch_v_out > maxPitch)
    {
        pitch_v_out = maxPitch;
    }
    else if (pitch_v_out < -maxPitch)
    {
        pitch_v_out = -maxPitch;
    }
    else
    {
        pitch_i = temp_pitch_i;
    }

    if (yaw_v_out > maxYaw)
    {
        yaw_v_out = maxYaw;
    }
    else if (yaw_v_out < -maxYaw)
    {
        yaw_v_out = -maxYaw;
    }
    else
    {
        yaw_i = temp_yaw_i;
    }
    //cout << yaw_kp << " " << yaw_ki << " " << yaw_kd << " " << pitch_kp << " " << pitch_ki << " " << pitch_kd << endl;
    //cout << pitch_e << " " << pitch_i << " " << pitch_d << " " << yaw_e << " " << yaw_i << " " << yaw_d << endl;
    last_pitch_e = pitch_e;
    last_yaw_e = yaw_e;
    return true;
};

void GimbalController::serialCallBack(const mavlink_message_t *message, const mavconn::Framing framing)
{
    static mavlink_heartbeat_t hb;
    static mavlink_attitude_t gimbal_attitude;
    static mavlink_attitude_quaternion_t gimbal_attitude_quad;
    static mavlink_local_position_ned_t positional_attitude;
    static mavlink_set_attitude_target_t gimbal_speed_target;

    switch (message->msgid)
    {
    case MAVLINK_MSG_ID_HEARTBEAT:
        mavlink_msg_heartbeat_decode(message, &hb);
        printf("Decoded Heartbeat: %d,%d,%d,%d\n\n", hb.type, hb.base_mode, hb.custom_mode, hb.system_status);
        break;

        //the attitude of the gimbal
    case MAVLINK_MSG_ID_ATTITUDE:
        mavlink_msg_attitude_decode(message, &gimbal_attitude);
        printf("Decoded gimbal attitude: %f,%f,%f,%f\n\n", gimbal_attitude.pitch, gimbal_attitude.pitchspeed, gimbal_attitude.yaw, gimbal_attitude.yawspeed);
        break;
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
        mavlink_msg_attitude_quaternion_decode(message, &gimbal_attitude_quad);
        printf("Decoded gimbal quad attitude: %d,%f,%f,%f,%f,%f,%f,%f\n\n",
               gimbal_attitude_quad.time_boot_ms,
               gimbal_attitude_quad.q1,
               gimbal_attitude_quad.q2,
               gimbal_attitude_quad.q3,
               gimbal_attitude_quad.q4,
               gimbal_attitude_quad.rollspeed,
               gimbal_attitude_quad.pitchspeed,
               gimbal_attitude_quad.yawspeed);
        break;
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        mavlink_msg_local_position_ned_decode(message, &positional_attitude);
        printf("Decoded positional attitude:%d, %f, %f, %f, %f, %f, %f\n\n",
               positional_attitude.time_boot_ms,
               positional_attitude.x,
               positional_attitude.y,
               positional_attitude.z,
               positional_attitude.vx,
               positional_attitude.vy,
               positional_attitude.vz);
        break;
    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        mavlink_msg_set_attitude_target_decode(message, &gimbal_speed_target);
        std::cout << "Decoded positional attitude: "
                  << "time_boot_ms" << gimbal_speed_target.time_boot_ms << std::endl
                  << "target_system" << gimbal_speed_target.target_system << std::endl
                  << "target_component" << gimbal_speed_target.target_component << std::endl
                  << "type_mask" << gimbal_speed_target.type_mask << std::endl
                  << "q0" << gimbal_speed_target.q[0] << std::endl
                  << "q1" << gimbal_speed_target.q[1] << std::endl
                  << "q2" << gimbal_speed_target.q[2] << std::endl
                  << "q3" << gimbal_speed_target.q[3] << std::endl
                  << "body_roll_rate" << gimbal_speed_target.body_roll_rate << std::endl
                  << "body_pitch_rate" << gimbal_speed_target.body_pitch_rate << std::endl
                  << "body_yaw_rate" << gimbal_speed_target.body_yaw_rate << std::endl
                  << "thrust" << gimbal_speed_target.thrust;
        break;
    default:
        printf("Unknown message\n\n");
    }
}
