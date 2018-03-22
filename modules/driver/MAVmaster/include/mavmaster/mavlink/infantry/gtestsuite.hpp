/** @file
 *	@brief MAVLink comm testsuite protocol generated from infantry.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "infantry.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(infantry, HEARTBEAT)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::infantry::msg::HEARTBEAT packet_in{};
    packet_in.type = 17;
    packet_in.base_mode = 84;
    packet_in.custom_mode = 963497464;
    packet_in.system_status = 151;
    packet_in.mavlink_version = 2;

    mavlink::infantry::msg::HEARTBEAT packet1{};
    mavlink::infantry::msg::HEARTBEAT packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.type, packet2.type);
    EXPECT_EQ(packet1.base_mode, packet2.base_mode);
    EXPECT_EQ(packet1.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet1.system_status, packet2.system_status);
    EXPECT_EQ(packet1.mavlink_version, packet2.mavlink_version);
}

#ifdef TEST_INTEROP
TEST(infantry_interop, HEARTBEAT)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_heartbeat_t packet_c {
         963497464, 17, 84, 151, 2
    };

    mavlink::infantry::msg::HEARTBEAT packet_in{};
    packet_in.type = 17;
    packet_in.base_mode = 84;
    packet_in.custom_mode = 963497464;
    packet_in.system_status = 151;
    packet_in.mavlink_version = 2;

    mavlink::infantry::msg::HEARTBEAT packet2{};

    mavlink_msg_heartbeat_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.type, packet2.type);
    EXPECT_EQ(packet_in.base_mode, packet2.base_mode);
    EXPECT_EQ(packet_in.custom_mode, packet2.custom_mode);
    EXPECT_EQ(packet_in.system_status, packet2.system_status);
    EXPECT_EQ(packet_in.mavlink_version, packet2.mavlink_version);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(infantry, ATTITUDE)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::infantry::msg::ATTITUDE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::infantry::msg::ATTITUDE packet1{};
    mavlink::infantry::msg::ATTITUDE packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.roll, packet2.roll);
    EXPECT_EQ(packet1.pitch, packet2.pitch);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(infantry_interop, ATTITUDE)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::infantry::msg::ATTITUDE packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.roll = 45.0;
    packet_in.pitch = 73.0;
    packet_in.yaw = 101.0;
    packet_in.rollspeed = 129.0;
    packet_in.pitchspeed = 157.0;
    packet_in.yawspeed = 185.0;

    mavlink::infantry::msg::ATTITUDE packet2{};

    mavlink_msg_attitude_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.roll, packet2.roll);
    EXPECT_EQ(packet_in.pitch, packet2.pitch);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(infantry, ATTITUDE_QUATERNION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::infantry::msg::ATTITUDE_QUATERNION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;

    mavlink::infantry::msg::ATTITUDE_QUATERNION packet1{};
    mavlink::infantry::msg::ATTITUDE_QUATERNION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.q1, packet2.q1);
    EXPECT_EQ(packet1.q2, packet2.q2);
    EXPECT_EQ(packet1.q3, packet2.q3);
    EXPECT_EQ(packet1.q4, packet2.q4);
    EXPECT_EQ(packet1.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet1.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet1.yawspeed, packet2.yawspeed);
}

#ifdef TEST_INTEROP
TEST(infantry_interop, ATTITUDE_QUATERNION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_attitude_quaternion_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0
    };

    mavlink::infantry::msg::ATTITUDE_QUATERNION packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.q1 = 45.0;
    packet_in.q2 = 73.0;
    packet_in.q3 = 101.0;
    packet_in.q4 = 129.0;
    packet_in.rollspeed = 157.0;
    packet_in.pitchspeed = 185.0;
    packet_in.yawspeed = 213.0;

    mavlink::infantry::msg::ATTITUDE_QUATERNION packet2{};

    mavlink_msg_attitude_quaternion_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.q1, packet2.q1);
    EXPECT_EQ(packet_in.q2, packet2.q2);
    EXPECT_EQ(packet_in.q3, packet2.q3);
    EXPECT_EQ(packet_in.q4, packet2.q4);
    EXPECT_EQ(packet_in.rollspeed, packet2.rollspeed);
    EXPECT_EQ(packet_in.pitchspeed, packet2.pitchspeed);
    EXPECT_EQ(packet_in.yawspeed, packet2.yawspeed);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(infantry, LOCAL_POSITION_NED)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::infantry::msg::LOCAL_POSITION_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;

    mavlink::infantry::msg::LOCAL_POSITION_NED packet1{};
    mavlink::infantry::msg::LOCAL_POSITION_NED packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.vx, packet2.vx);
    EXPECT_EQ(packet1.vy, packet2.vy);
    EXPECT_EQ(packet1.vz, packet2.vz);
}

#ifdef TEST_INTEROP
TEST(infantry_interop, LOCAL_POSITION_NED)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_local_position_ned_t packet_c {
         963497464, 45.0, 73.0, 101.0, 129.0, 157.0, 185.0
    };

    mavlink::infantry::msg::LOCAL_POSITION_NED packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.vx = 129.0;
    packet_in.vy = 157.0;
    packet_in.vz = 185.0;

    mavlink::infantry::msg::LOCAL_POSITION_NED packet2{};

    mavlink_msg_local_position_ned_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.vx, packet2.vx);
    EXPECT_EQ(packet_in.vy, packet2.vy);
    EXPECT_EQ(packet_in.vz, packet2.vz);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(infantry, SET_ATTITUDE_TARGET)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::infantry::msg::SET_ATTITUDE_TARGET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 113;
    packet_in.target_component = 180;
    packet_in.type_mask = 247;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.body_roll_rate = 157.0;
    packet_in.body_pitch_rate = 185.0;
    packet_in.body_yaw_rate = 213.0;
    packet_in.thrust = 241.0;

    mavlink::infantry::msg::SET_ATTITUDE_TARGET packet1{};
    mavlink::infantry::msg::SET_ATTITUDE_TARGET packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet1.target_system, packet2.target_system);
    EXPECT_EQ(packet1.target_component, packet2.target_component);
    EXPECT_EQ(packet1.type_mask, packet2.type_mask);
    EXPECT_EQ(packet1.q, packet2.q);
    EXPECT_EQ(packet1.body_roll_rate, packet2.body_roll_rate);
    EXPECT_EQ(packet1.body_pitch_rate, packet2.body_pitch_rate);
    EXPECT_EQ(packet1.body_yaw_rate, packet2.body_yaw_rate);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
}

#ifdef TEST_INTEROP
TEST(infantry_interop, SET_ATTITUDE_TARGET)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_set_attitude_target_t packet_c {
         963497464, { 45.0, 46.0, 47.0, 48.0 }, 157.0, 185.0, 213.0, 241.0, 113, 180, 247
    };

    mavlink::infantry::msg::SET_ATTITUDE_TARGET packet_in{};
    packet_in.time_boot_ms = 963497464;
    packet_in.target_system = 113;
    packet_in.target_component = 180;
    packet_in.type_mask = 247;
    packet_in.q = {{ 45.0, 46.0, 47.0, 48.0 }};
    packet_in.body_roll_rate = 157.0;
    packet_in.body_pitch_rate = 185.0;
    packet_in.body_yaw_rate = 213.0;
    packet_in.thrust = 241.0;

    mavlink::infantry::msg::SET_ATTITUDE_TARGET packet2{};

    mavlink_msg_set_attitude_target_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_boot_ms, packet2.time_boot_ms);
    EXPECT_EQ(packet_in.target_system, packet2.target_system);
    EXPECT_EQ(packet_in.target_component, packet2.target_component);
    EXPECT_EQ(packet_in.type_mask, packet2.type_mask);
    EXPECT_EQ(packet_in.q, packet2.q);
    EXPECT_EQ(packet_in.body_roll_rate, packet2.body_roll_rate);
    EXPECT_EQ(packet_in.body_pitch_rate, packet2.body_pitch_rate);
    EXPECT_EQ(packet_in.body_yaw_rate, packet2.body_yaw_rate);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
