/** @file
 *	@brief MAVLink comm protocol generated from infantry.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace infantry {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 5> MESSAGE_ENTRIES {{ {0, 138, 8, 0, 0, 0}, {30, 39, 28, 0, 0, 0}, {31, 246, 32, 0, 0, 0}, {32, 185, 28, 0, 0, 0}, {82, 49, 39, 3, 36, 37} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief  */
enum class ROBO_TYPE : uint8_t
{
    SOLDIER=0, /* Generic micro air vehicle. | */
    SENTRY=1, /* Fixed wing aircraft. | */
    ENGINEER=2, /* Quadrotor | */
    HERO=3, /* Coaxial helicopter | */
    DRONE=4, /* Normal helicopter with tail rotor. | */
};

//! ROBO_TYPE ENUM_END
constexpr auto ROBO_TYPE_ENUM_END = 5;

/** @brief These flags encode the state of FSM. */
enum class ROBO_MODE_FLAG
{
    MAV_MODE_FLAG_FOCUS_FIRE=16, /* 0b00010000 Focus firepower when at advantage. | */
    MAV_MODE_FLAG_EVADE=32, /* 0b00100000 Evade under heavy fire. | */
    MAV_MODE_FLAG_PATROL=64, /* 0b01000000 Patrol when adversaries are out of sight. | */
    MAV_MODE_FLAG_MANUAL=128, /* 0b10000000 Robot is controlled by RC input. | */
};

//! ROBO_MODE_FLAG ENUM_END
constexpr auto ROBO_MODE_FLAG_ENUM_END = 129;

/** @brief These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not. */
enum class ROBO_MODE_FLAG_DECODE_POSITION
{
    MAV_MODE_FLAG_DECODE_POSITION_FOCUS=16, /* Fourth bit: 00010000 | */
    MAV_MODE_FLAG_DECODE_POSITION_EVADE=32, /* Third bit:  00100000 | */
    MAV_MODE_FLAG_DECODE_POSITION_PATROL=64, /* Second bit: 01000000 | */
    MAV_MODE_FLAG_DECODE_POSITION_MANUAL=128, /* First bit:  10000000 | */
};

//! ROBO_MODE_FLAG_DECODE_POSITION ENUM_END
constexpr auto ROBO_MODE_FLAG_DECODE_POSITION_ENUM_END = 129;

/** @brief  */
enum class ROBO_STATE : uint8_t
{
    UNINIT=0, /* Uninitialized system, state is unknown. | */
    BOOT=1, /* System is booting up. | */
    CALIBRATING=2, /* System is calibrating and not flight-ready. | */
    STANDBY=3, /* System is grounded and on standby. It can be launched any time. | */
    ACTIVE=4, /* System is active and might be already airborne. Motors are engaged. | */
    CRITICAL=5, /* System is in a non-normal flight mode. It can however still navigate. | */
    EMERGENCY=6, /* System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down. | */
    POWEROFF=7, /* System just initialized its power-down sequence, will shut down now. | */
};

//! ROBO_STATE ENUM_END
constexpr auto ROBO_STATE_ENUM_END = 8;


} // namespace infantry
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.hpp"
#include "./mavlink_msg_attitude.hpp"
#include "./mavlink_msg_attitude_quaternion.hpp"
#include "./mavlink_msg_local_position_ned.hpp"
#include "./mavlink_msg_set_attitude_target.hpp"

// base include

