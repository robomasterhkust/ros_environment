#pragma once
// MESSAGE GIMBAL_COORDINATE_TARGET PACKING

#define MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET 33

MAVPACKED(
typedef struct __mavlink_gimbal_coordinate_target_t {
 float x; /*< Target coordinate, x*/
 float y; /*< Target coordinate, y*/
 float z; /*< Target coordinate, z*/
}) mavlink_gimbal_coordinate_target_t;

#define MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN 12
#define MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN 12
#define MAVLINK_MSG_ID_33_LEN 12
#define MAVLINK_MSG_ID_33_MIN_LEN 12

#define MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC 61
#define MAVLINK_MSG_ID_33_CRC 61



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GIMBAL_COORDINATE_TARGET { \
    33, \
    "GIMBAL_COORDINATE_TARGET", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_coordinate_target_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_coordinate_target_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_coordinate_target_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GIMBAL_COORDINATE_TARGET { \
    "GIMBAL_COORDINATE_TARGET", \
    3, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gimbal_coordinate_target_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_gimbal_coordinate_target_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_gimbal_coordinate_target_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a gimbal_coordinate_target message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x Target coordinate, x
 * @param y Target coordinate, y
 * @param z Target coordinate, z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_coordinate_target_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN);
#else
    mavlink_gimbal_coordinate_target_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
}

/**
 * @brief Pack a gimbal_coordinate_target message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x Target coordinate, x
 * @param y Target coordinate, y
 * @param z Target coordinate, z
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_coordinate_target_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN);
#else
    mavlink_gimbal_coordinate_target_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
}

/**
 * @brief Encode a gimbal_coordinate_target struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_coordinate_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_coordinate_target_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_coordinate_target_t* gimbal_coordinate_target)
{
    return mavlink_msg_gimbal_coordinate_target_pack(system_id, component_id, msg, gimbal_coordinate_target->x, gimbal_coordinate_target->y, gimbal_coordinate_target->z);
}

/**
 * @brief Encode a gimbal_coordinate_target struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_coordinate_target C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_coordinate_target_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_coordinate_target_t* gimbal_coordinate_target)
{
    return mavlink_msg_gimbal_coordinate_target_pack_chan(system_id, component_id, chan, msg, gimbal_coordinate_target->x, gimbal_coordinate_target->y, gimbal_coordinate_target->z);
}

/**
 * @brief Send a gimbal_coordinate_target message
 * @param chan MAVLink channel to send the message
 *
 * @param x Target coordinate, x
 * @param y Target coordinate, y
 * @param z Target coordinate, z
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_coordinate_target_send(mavlink_channel_t chan, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET, buf, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
#else
    mavlink_gimbal_coordinate_target_t packet;
    packet.x = x;
    packet.y = y;
    packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
#endif
}

/**
 * @brief Send a gimbal_coordinate_target message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gimbal_coordinate_target_send_struct(mavlink_channel_t chan, const mavlink_gimbal_coordinate_target_t* gimbal_coordinate_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gimbal_coordinate_target_send(chan, gimbal_coordinate_target->x, gimbal_coordinate_target->y, gimbal_coordinate_target->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET, (const char *)gimbal_coordinate_target, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_coordinate_target_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET, buf, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
#else
    mavlink_gimbal_coordinate_target_t *packet = (mavlink_gimbal_coordinate_target_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_MIN_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_CRC);
#endif
}
#endif

#endif

// MESSAGE GIMBAL_COORDINATE_TARGET UNPACKING


/**
 * @brief Get field x from gimbal_coordinate_target message
 *
 * @return Target coordinate, x
 */
static inline float mavlink_msg_gimbal_coordinate_target_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from gimbal_coordinate_target message
 *
 * @return Target coordinate, y
 */
static inline float mavlink_msg_gimbal_coordinate_target_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from gimbal_coordinate_target message
 *
 * @return Target coordinate, z
 */
static inline float mavlink_msg_gimbal_coordinate_target_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a gimbal_coordinate_target message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_coordinate_target C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_coordinate_target_decode(const mavlink_message_t* msg, mavlink_gimbal_coordinate_target_t* gimbal_coordinate_target)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    gimbal_coordinate_target->x = mavlink_msg_gimbal_coordinate_target_get_x(msg);
    gimbal_coordinate_target->y = mavlink_msg_gimbal_coordinate_target_get_y(msg);
    gimbal_coordinate_target->z = mavlink_msg_gimbal_coordinate_target_get_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN? msg->len : MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN;
        memset(gimbal_coordinate_target, 0, MAVLINK_MSG_ID_GIMBAL_COORDINATE_TARGET_LEN);
    memcpy(gimbal_coordinate_target, _MAV_PAYLOAD(msg), len);
#endif
}
