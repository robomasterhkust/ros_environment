// MESSAGE HEARTBEAT support class

#pragma once

namespace mavlink {
namespace infantry {
namespace msg {

/**
 * @brief HEARTBEAT message
 *
 * The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot).
 */
struct HEARTBEAT : mavlink::Message {
    static constexpr msgid_t MSG_ID = 0;
    static constexpr size_t LENGTH = 8;
    static constexpr size_t MIN_LENGTH = 8;
    static constexpr uint8_t CRC_EXTRA = 138;
    static constexpr auto NAME = "HEARTBEAT";


    uint8_t type; /*< Type of the MAV (soldier, sentry, etc., up to 5 types, defined in ROBO_TYPE ENUM) */
    uint8_t base_mode; /*< System mode bitfield, see ROBO_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h */
    uint32_t custom_mode; /*< A bitfield for use for autopilot-specific flags. */
    uint8_t system_status; /*< System status flag, see ROBO_STATE ENUM */
    uint8_t mavlink_version; /*< MAVLink version */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  base_mode: " << +base_mode << std::endl;
        ss << "  custom_mode: " << custom_mode << std::endl;
        ss << "  system_status: " << +system_status << std::endl;
        ss << "  mavlink_version: " << +mavlink_version << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << custom_mode;                   // offset: 0
        map << type;                          // offset: 4
        map << base_mode;                     // offset: 5
        map << system_status;                 // offset: 6
        map << uint8_t(2);               // offset: 7
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> custom_mode;                   // offset: 0
        map >> type;                          // offset: 4
        map >> base_mode;                     // offset: 5
        map >> system_status;                 // offset: 6
        map >> mavlink_version;               // offset: 7
    }
};

} // namespace msg
} // namespace infantry
} // namespace mavlink
