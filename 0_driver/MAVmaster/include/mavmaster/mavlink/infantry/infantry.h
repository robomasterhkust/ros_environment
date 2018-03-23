/** @file
 *  @brief MAVLink comm protocol generated from infantry.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_INFANTRY_H
#define MAVLINK_INFANTRY_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_INFANTRY.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 138, 8, 0, 0, 0}, {30, 39, 28, 0, 0, 0}, {31, 246, 32, 0, 0, 0}, {32, 185, 28, 0, 0, 0}, {82, 49, 39, 3, 36, 37}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_INFANTRY

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_ROBO_TYPE
#define HAVE_ENUM_ROBO_TYPE
typedef enum ROBO_TYPE
{
   ROBO_TYPE_SOLDIER=0, /* Generic micro air vehicle. | */
   ROBO_TYPE_SENTRY=1, /* Fixed wing aircraft. | */
   ROBO_TYPE_ENGINEER=2, /* Quadrotor | */
   ROBO_TYPE_HERO=3, /* Coaxial helicopter | */
   ROBO_TYPE_DRONE=4, /* Normal helicopter with tail rotor. | */
   ROBO_TYPE_ENUM_END=5, /*  | */
} ROBO_TYPE;
#endif

/** @brief These flags encode the state of FSM. */
#ifndef HAVE_ENUM_ROBO_MODE_FLAG
#define HAVE_ENUM_ROBO_MODE_FLAG
typedef enum ROBO_MODE_FLAG
{
   MAV_MODE_FLAG_FOCUS_FIRE=16, /* 0b00010000 Focus firepower when at advantage. | */
   MAV_MODE_FLAG_EVADE=32, /* 0b00100000 Evade under heavy fire. | */
   MAV_MODE_FLAG_PATROL=64, /* 0b01000000 Patrol when adversaries are out of sight. | */
   MAV_MODE_FLAG_MANUAL=128, /* 0b10000000 Robot is controlled by RC input. | */
   ROBO_MODE_FLAG_ENUM_END=129, /*  | */
} ROBO_MODE_FLAG;
#endif

/** @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
#ifndef HAVE_ENUM_ROBO_MODE_FLAG_DECODE_POSITION
#define HAVE_ENUM_ROBO_MODE_FLAG_DECODE_POSITION
typedef enum ROBO_MODE_FLAG_DECODE_POSITION
{
   MAV_MODE_FLAG_DECODE_POSITION_FOCUS=16, /* Fourth bit: 00010000 | */
   MAV_MODE_FLAG_DECODE_POSITION_EVADE=32, /* Third bit:  00100000 | */
   MAV_MODE_FLAG_DECODE_POSITION_PATROL=64, /* Second bit: 01000000 | */
   MAV_MODE_FLAG_DECODE_POSITION_MANUAL=128, /* First bit:  10000000 | */
   ROBO_MODE_FLAG_DECODE_POSITION_ENUM_END=129, /*  | */
} ROBO_MODE_FLAG_DECODE_POSITION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_ROBO_STATE
#define HAVE_ENUM_ROBO_STATE
typedef enum ROBO_STATE
{
   ROBO_STATE_UNINIT=0, /* Uninitialized system, state is unknown. | */
   ROBO_STATE_BOOT=1, /* System is booting up. | */
   ROBO_STATE_CALIBRATING=2, /* System is calibrating and not flight-ready. | */
   ROBO_STATE_STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
   ROBO_STATE_ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
   ROBO_STATE_CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
   ROBO_STATE_EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
   ROBO_STATE_POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
   ROBO_STATE_ENUM_END=8, /*  | */
} ROBO_STATE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_attitude.h"
#include "./mavlink_msg_attitude_quaternion.h"
#include "./mavlink_msg_local_position_ned.h"
#include "./mavlink_msg_set_attitude_target.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_ATTITUDE_QUATERNION, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_NED, MAVLINK_MESSAGE_INFO_SET_ATTITUDE_TARGET}
# define MAVLINK_MESSAGE_NAMES {{ "ATTITUDE", 30 }, { "ATTITUDE_QUATERNION", 31 }, { "HEARTBEAT", 0 }, { "LOCAL_POSITION_NED", 32 }, { "SET_ATTITUDE_TARGET", 82 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_INFANTRY_H
