#pragma once
// MESSAGE GIMBAL_SPEED_TARGET PACKING

#define MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET 34

MAVPACKED(
typedef struct __mavlink_gimbal_speed_target_t {
 float yawspeed; /*< Yaw angular speed (rad/s)*/
 float pitchspeed; /*< Pitch angular speed (rad/s)*/
}) mavlink_gimbal_speed_target_t;

#define MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN 8
#define MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN 8
#define MAVLINK_MSG_ID_34_LEN 8
#define MAVLINK_MSG_ID_34_MIN_LEN 8

#define MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC 138
#define MAVLINK_MSG_ID_34_CRC 138



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_SPEED_TARGET { \
    34, \
    "GIMBAL_SPEED_TARGET", \
    2, \
    {  { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_speed_target_t, yawspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_speed_target_t, pitchspeed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_SPEED_TARGET { \
    "GIMBAL_SPEED_TARGET", \
    2, \
    {  { "yawspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_speed_target_t, yawspeed) }, \
         { "pitchspeed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_speed_target_t, pitchspeed) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_speed_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param yawspeed Yaw angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_speed_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float yawspeed, float pitchspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN];
    _mav_put_float(buf, 0, yawspeed);
    _mav_put_float(buf, 4, pitchspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN);
#else
    mavlink_gimbal_speed_target_t packet;
    packet.yawspeed = yawspeed;
    packet.pitchspeed = pitchspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
}

/**
 * @brief Pack a gimbal_speed_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yawspeed Yaw angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_speed_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float yawspeed,float pitchspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN];
    _mav_put_float(buf, 0, yawspeed);
    _mav_put_float(buf, 4, pitchspeed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN);
#else
    mavlink_gimbal_speed_target_t packet;
    packet.yawspeed = yawspeed;
    packet.pitchspeed = pitchspeed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
}

/**
 * @brief Encode a gimbal_speed_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_speed_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_speed_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_speed_target_t* gimbal_speed_target)
{
    return mavlink_msg_gimbal_speed_target_pack(system_id, component_id, msg, gimbal_speed_target->yawspeed, gimbal_speed_target->pitchspeed);
}

/**
 * @brief Encode a gimbal_speed_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_speed_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_speed_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_speed_target_t* gimbal_speed_target)
{
    return mavlink_msg_gimbal_speed_target_pack_chan(system_id, component_id, chan, msg, gimbal_speed_target->yawspeed, gimbal_speed_target->pitchspeed);
}

/**
 * @brief Send a gimbal_speed_target message
 * @param chan MAVLink channel to send the message
 *
 * @param yawspeed Yaw angular speed (rad/s)
 * @param pitchspeed Pitch angular speed (rad/s)
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_speed_target_send(mavlink_channel_t chan, float yawspeed, float pitchspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN];
    _mav_put_float(buf, 0, yawspeed);
    _mav_put_float(buf, 4, pitchspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET, buf, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
#else
    mavlink_gimbal_speed_target_t packet;
    packet.yawspeed = yawspeed;
    packet.pitchspeed = pitchspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
#endif
}

/**
 * @brief Send a gimbal_speed_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_speed_target_send_struct(mavlink_channel_t chan, const mavlink_gimbal_speed_target_t* gimbal_speed_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_speed_target_send(chan, gimbal_speed_target->yawspeed, gimbal_speed_target->pitchspeed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET, (const char *)gimbal_speed_target, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_speed_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float yawspeed, float pitchspeed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, yawspeed);
    _mav_put_float(buf, 4, pitchspeed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET, buf, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
#else
    mavlink_gimbal_speed_target_t *packet = (mavlink_gimbal_speed_target_t *)msgbuf;
    packet->yawspeed = yawspeed;
    packet->pitchspeed = pitchspeed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_SPEED_TARGET UNPACKING


/**
 * @brief Get field yawspeed from gimbal_speed_target message
 *
 * @return Yaw angular speed (rad/s)
 */
static inline float mavlink_msg_gimbal_speed_target_get_yawspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitchspeed from gimbal_speed_target message
 *
 * @return Pitch angular speed (rad/s)
 */
static inline float mavlink_msg_gimbal_speed_target_get_pitchspeed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a gimbal_speed_target message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_speed_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_speed_target_decode(const mavlink_message_t* msg, mavlink_gimbal_speed_target_t* gimbal_speed_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_speed_target->yawspeed = mavlink_msg_gimbal_speed_target_get_yawspeed(msg);
    gimbal_speed_target->pitchspeed = mavlink_msg_gimbal_speed_target_get_pitchspeed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN;
        memset(gimbal_speed_target, 0, MAVLINK_MSG_ID_GIMBAL_SPEED_TARGET_LEN);
    memcpy(gimbal_speed_target, _MAV_PAYLOAD(msg), len);
#endif
}
